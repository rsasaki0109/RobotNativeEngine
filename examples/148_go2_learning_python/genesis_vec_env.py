"""Stable-Baselines3 VecEnv over the Go2 task in Genesis, its environments batched in one scene.

Genesis steps many environments of one scene together, so this VecEnv is
one scene of `n_envs` Go2s, built as `genesis_go2_learning.py` builds its
plant: the URDF without visuals, fixed links merged, the Go2's rotor
inertia, a 1 ms step, friction 1. The joints' PD (kp 20, kd 0.5) is computed
from the state at every 1 ms step and applied as torque, within the
effort limits and, with velocity limits, the torque-speed curve, as
MuJoCo's position actuators apply theirs. The episode is RNE's: the same
observation, reward, fall and time limit, starting where a robot that
settled 1 s from the spawn pose stands; on the flat floor every episode
starts there, so a reset puts the environment back in that settled state.
The flat floor only, for now.
"""

import math
import os
import sys
import tempfile
from pathlib import Path

import numpy as np
from gymnasium import spaces
from stable_baselines3.common.vec_env import VecEnv

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                                "147_go2_learning"))
import mujoco_go2_learning as mj  # noqa: E402

EFFORT_LIMIT_NM = np.tile([23.7, 23.7, 45.43], 4)


def _numpy(value):
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value, dtype=float)


def _rotations(quaternions):
    """Rotation matrices (n, 3, 3) of w x y z quaternions (n, 4)."""
    q = quaternions / np.linalg.norm(quaternions, axis=1, keepdims=True)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], 1)


def _joint_dof(robot, name):
    joint = robot.get_joint(name)
    for attribute in ("dofs_idx_local", "dof_idx_local", "dofs_idx", "dof_idx", "dof_start"):
        value = getattr(joint, attribute, None)
        if value is not None:
            return int(np.ravel(value)[0])
    raise SystemExit(f"cannot find the dof of {name}")


class GenesisGo2VecEnv(VecEnv):
    """`n_envs` Go2s in one Genesis scene, stepped together."""

    def __init__(self, n_envs=8, seed=1, velocity_limits=False, armature=0.0, **task):
        if task.get("random_starts") or task.get("height_scan"):
            raise ValueError("GenesisGo2VecEnv runs the flat floor only")
        import genesis as gs
        try:
            gs.init(backend=gs.cpu, precision="64", logging_level="error")
        except Exception:  # noqa: BLE001 - already initialized in this process
            pass
        self.velocity_limits = bool(velocity_limits)
        self.substeps = 5
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=mj.DT_S / self.substeps, substeps=1),
            rigid_options=gs.options.RigidOptions(enable_self_collision=False),
            show_viewer=False)
        material = gs.materials.Rigid(friction=1.0)
        self.scene.add_entity(gs.morphs.Plane(), material=material)
        urdf = Path(tempfile.mkdtemp()) / "go2.urdf"
        urdf.write_text(mj.urdf_without_visuals())
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(file=str(urdf), pos=(0.0, 0.0, mj.SPAWN_HEIGHT_M),
                           merge_fixed_links=True, default_armature=armature or None),
            material=material)
        self.scene.build(n_envs=n_envs)
        self.dofs = [_joint_dof(self.robot, name) for name in mj.JOINTS]
        self.n = n_envs
        self.arena_half = 0.5 * mj.FLAT_FLOOR_SIZE_M - mj.ARENA_EDGE_MARGIN_M
        # Settle every environment from the spawn pose, and keep the state.
        self.robot.set_dofs_position(np.tile(mj.DEFAULT_RAD, (n_envs, 1)), self.dofs)
        self.robot.zero_all_dofs_velocity()
        for _ in range(mj.SETTLE_STEPS):
            self._physics_period(np.tile(mj.DEFAULT_RAD, (n_envs, 1)))
        self.settled_qpos = _numpy(self.robot.get_qpos()).reshape(n_envs, -1).copy()
        self.settled_dofs_velocity = _numpy(self.robot.get_dofs_velocity()).reshape(
            n_envs, -1).copy()
        self.action = np.zeros((n_envs, mj.ACTIONS))
        self.steps = np.zeros(n_envs, dtype=int)
        self.start = np.zeros((n_envs, 3))
        self.last = np.zeros((n_envs, 3))
        self._actions = None
        self.render_mode = None
        super().__init__(
            n_envs,
            spaces.Box(-math.inf, math.inf, shape=(mj.PROPRIOCEPTION,), dtype=np.float64),
            spaces.Box(-1.0, 1.0, shape=(mj.ACTIONS,), dtype=np.float64),
        )

    def _physics_period(self, targets):
        """One control period: the PD torque set every physics step."""
        for _ in range(mj.DECIMATION * self.substeps):
            q = _numpy(self.robot.get_dofs_position(self.dofs))
            qd = _numpy(self.robot.get_dofs_velocity(self.dofs))
            torque = mj.KP_NM_PER_RAD * (targets - q) - mj.KD_NM_S_PER_RAD * qd
            lowest, highest = -EFFORT_LIMIT_NM, EFFORT_LIMIT_NM
            if self.velocity_limits:
                ratio = qd / mj.VELOCITY_LIMIT_RAD_S
                lowest = np.clip(EFFORT_LIMIT_NM * (-1.0 - ratio), -EFFORT_LIMIT_NM,
                                 EFFORT_LIMIT_NM)
                highest = np.clip(EFFORT_LIMIT_NM * (1.0 - ratio), -EFFORT_LIMIT_NM,
                                  EFFORT_LIMIT_NM)
            self.robot.control_dofs_force(np.clip(torque, lowest, highest), self.dofs)
            self.scene.step()

    def _restore(self, envs):
        envs = list(envs)
        self.robot.set_qpos(self.settled_qpos[envs], envs_idx=envs, zero_velocity=False)
        self.robot.set_dofs_velocity(self.settled_dofs_velocity[envs], envs_idx=envs)
        self.action[envs] = 0.0
        self.steps[envs] = 0
        # The free joint's position leads the settled configuration.
        position = self.settled_qpos[envs, :3]
        self.start[envs] = position
        self.last[envs] = np.stack([position[:, 0], np.zeros(len(envs)), np.zeros(len(envs))], 1)

    def _observe(self):
        rotation = _rotations(_numpy(self.robot.get_quat()).reshape(self.n, 4))
        world_spin = _numpy(self.robot.get_ang()).reshape(self.n, 3)
        spin = np.einsum("nji,nj->ni", rotation, world_spin)
        gravity = -rotation[:, 2, :]
        phase = (self.steps * mj.CONTROL_DT_S % mj.GAIT_PERIOD_S) / mj.GAIT_PERIOD_S
        angle = 2.0 * math.pi * phase
        q = _numpy(self.robot.get_dofs_position(self.dofs)).reshape(self.n, 12)
        qd = _numpy(self.robot.get_dofs_velocity(self.dofs)).reshape(self.n, 12)
        return np.concatenate([spin * 0.25, gravity, q - mj.DEFAULT_RAD, qd * 0.05, self.action,
                               np.sin(angle)[:, None], np.cos(angle)[:, None]], 1), rotation

    def reset(self):
        self._restore(range(self.n))
        return self._observe()[0]

    def step_async(self, actions):
        self._actions = np.clip(np.asarray(actions, dtype=np.float64), -1.0, 1.0)

    def step_wait(self):
        following = self._actions
        change = np.sum((following - self.action) ** 2, axis=1)
        self.action = following
        self._physics_period(mj.DEFAULT_RAD + mj.ACTION_SCALE_RAD * following)
        self.steps += 1
        observations, rotation = self._observe()
        position = _numpy(self.robot.get_pos()).reshape(self.n, 3)
        x, y, z = position[:, 0], position[:, 1], position[:, 2]
        heading = np.abs(np.arctan2(rotation[:, 1, 0], rotation[:, 0, 0]))
        fell = ((z < mj.FALL_HEIGHT_M) | (-rotation[:, 2, 2] > mj.FALL_GRAVITY_Z)
                | (np.abs(x) > self.arena_half) | (np.abs(y) > self.arena_half))
        lateral = np.abs(y - self.start[:, 1])
        rewards = ((x - self.last[:, 0]) - mj.LATERAL_PENALTY * (lateral - self.last[:, 1])
                   - mj.HEADING_PENALTY * (heading - self.last[:, 2])
                   - mj.SMOOTHNESS_PENALTY * change / mj.EPISODE_STEPS
                   - np.where(fell, mj.FALL_PENALTY, 0.0))
        self.last = np.stack([x, lateral, heading], 1)
        truncated = ~fell & (self.steps >= mj.EPISODE_STEPS)
        done = fell | truncated
        infos = [{} for _ in range(self.n)]
        ended = np.flatnonzero(done)
        if len(ended):
            for env in ended:
                infos[env]["terminal_observation"] = observations[env].copy()
                infos[env]["TimeLimit.truncated"] = bool(truncated[env])
            self._restore(ended)
            observations[ended] = self._observe()[0][ended]
        return observations, rewards, done, infos

    def close(self):
        return None

    def get_images(self):
        return [None] * self.num_envs

    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False] * self.num_envs

    def env_method(self, method_name, *method_args, indices=None, **method_kwargs):
        raise NotImplementedError("GenesisGo2VecEnv has no per-environment Python methods")

    def get_attr(self, attr_name, indices=None):
        return [getattr(self, attr_name, None)] * self.num_envs

    def set_attr(self, attr_name, value, indices=None):
        setattr(self, attr_name, value)

    def seed(self, seed=None):
        return [None] * self.num_envs
