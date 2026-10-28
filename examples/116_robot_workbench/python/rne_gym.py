"""Gymnasium joint-target tasks over independent RNE contact physics worlds."""
from __future__ import annotations

import json
from pathlib import Path
import subprocess
import tempfile

import gymnasium as gym
from gymnasium.vector import AutoresetMode, VectorEnv
from gymnasium.vector.utils import batch_space
import numpy as np


class ShoulderVectorEnv(VectorEnv):
    """Direct torque control; explicit masked reset after success/fall/time limit.

    Actions follow ``joint_names`` (sorted backend joint names), normalized by
    the reported effort limits. Targets are shoulder_pan angles in radians.
    Other joints and wheels are policy controlled, with no hidden controller.
    """

    metadata = {"render_modes": [], "autoreset_mode": AutoresetMode.DISABLED}
    target_joints = ("shoulder_pan",)
    _target_shape = ()

    def __init__(self, executable, config, *, horizon=200, physics_steps=4):
        if not isinstance(horizon, int) or horizon < 1:
            raise ValueError("horizon must be a positive integer")
        if not isinstance(physics_steps, int) or not 1 <= physics_steps <= 240:
            raise ValueError("physics_steps must be in 1..240")
        config = json.loads(json.dumps(config))
        if not config.get("environments") or any(
            len(c["agents"]) != 1 for c in config["environments"]
        ):
            raise ValueError("each training environment needs exactly one agent")
        self.num_envs = len(config["environments"])
        self.agent_ids = [c["agents"][0]["id"] for c in config["environments"]]
        self.horizon, self.physics_steps = horizon, physics_steps
        self.closed = False
        self._process = None
        self._temp = tempfile.TemporaryDirectory(prefix="rne-gym-")
        path = Path(self._temp.name) / "config.json"
        path.write_text(json.dumps(config), encoding="utf-8")
        try:
            self._process = subprocess.Popen(
                [str(executable), "--serve-vector-arena", str(path)],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
                encoding="utf-8", bufsize=1,
            )
            self._raw = self._read("ready")["observations"]
            self.joint_names = tuple(sorted(self._agent(0)["joints"]))
            if any(tuple(sorted(self._agent(i)["joints"])) != self.joint_names
                   for i in range(self.num_envs)):
                raise ValueError("all environments must have the same joint layout")
            self._caps = np.array([
                [self._agent(i)["joints"][j]["effort_limit_nm"] for j in self.joint_names]
                for i in range(self.num_envs)
            ])
            self.single_action_space = gym.spaces.Box(-1.0, 1.0, (len(self.joint_names),), np.float32)
            # q, dq, base position/quaternion/velocity, target, hold count, time remaining.
            self.single_observation_space = gym.spaces.Box(
                -np.inf, np.inf, (2 * len(self.joint_names) + 12 + len(self.target_joints),), np.float64
            )
            self.action_space = batch_space(self.single_action_space, self.num_envs)
            self.observation_space = batch_space(self.single_observation_space, self.num_envs)
            self._rngs = [np.random.default_rng() for _ in range(self.num_envs)]
            self.targets = np.zeros((self.num_envs, *self._target_shape))
            self._elapsed = np.zeros(self.num_envs, dtype=np.int64)
            self._held = np.zeros(self.num_envs, dtype=np.int64)
            self._needs_reset = np.ones(self.num_envs, dtype=bool)
        except BaseException:
            self.close()
            raise

    def _read(self, expected):
        line = self._process.stdout.readline()
        if not line:
            raise RuntimeError("RNE process closed its output; see stderr")
        reply = json.loads(line)
        if reply.get("kind") != expected:
            raise RuntimeError(f"unexpected RNE response: {reply}")
        return reply

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def _request(self, request, expected):
        if self.closed:
            raise RuntimeError("environment is closed")
        self._process.stdin.write(json.dumps(request, allow_nan=False) + "\n")
        self._process.stdin.flush()
        return self._read(expected)

    def _agent(self, i):
        return self._raw[i]["agents"][self.agent_ids[i]]

    def _observations(self):
        rows = []
        for i in range(self.num_envs):
            agent = self._agent(i)
            rows.append(
                [agent["joints"][j]["position_rad"] for j in self.joint_names]
                + [agent["joints"][j]["velocity_rad_s"] for j in self.joint_names]
                + agent["position_m"] + agent["rotation_xyzw"] + agent["linear_velocity_m_s"]
                + np.atleast_1d(self.targets[i]).tolist()
                + [self._held[i] / 10, 1 - self._elapsed[i] / self.horizon]
            )
        return np.asarray(rows, dtype=np.float64)

    def _info(self):
        return {"target_rad": self.targets.copy(),
                "sim_time_ticks": np.array([o["sim_time_ticks"] for o in self._raw]),
                "state_hash": np.array([o["state_hash"] for o in self._raw])}

    def reset(self, *, seed=None, options=None):
        """Reset all lanes, or only boolean options['reset_mask']; seed per lane."""
        mask = np.asarray((options or {}).get("reset_mask", np.ones(self.num_envs, dtype=bool)))
        if mask.shape != (self.num_envs,) or mask.dtype != np.bool_ or not mask.any():
            raise ValueError("reset_mask must be a nonempty boolean mask of num_envs")
        if seed is None:
            seeds = [None] * self.num_envs
        elif isinstance(seed, int):
            seeds = [seed + i for i in range(self.num_envs)]
        elif isinstance(seed, (list, tuple)) and len(seed) == self.num_envs:
            seeds = list(seed)
        else:
            raise ValueError("seed must be an integer or a list of num_envs seeds")
        if any(s is not None and (not isinstance(s, int) or s < 0) for s in seeds):
            raise ValueError("seeds must be nonnegative integers or None")
        # Validate seeds before touching physics or existing RNG streams.
        replacements = {i: gym.utils.seeding.np_random(seeds[i])[0]
                        for i in np.flatnonzero(mask) if seeds[i] is not None}
        indices = np.flatnonzero(mask).tolist()
        reply = self._request({"op": "reset", "indices": indices}, "reset")
        for i, raw in zip(indices, reply["observations"]):
            self._raw[i] = raw
            if i in replacements:
                self._rngs[i] = replacements[i]
            self.targets[i] = self._sample_target(self._rngs[i])
            self._elapsed[i] = self._held[i] = 0
            self._needs_reset[i] = False
        return self._observations(), self._info()

    def step(self, actions):
        if self._needs_reset.any():
            raise RuntimeError("reset uninitialized or finished lanes before stepping")
        actions = np.asarray(actions, dtype=np.float64)
        if (actions.shape != self.action_space.shape or not np.isfinite(actions).all()
                or (np.abs(actions) > 1).any()):
            raise ValueError("actions must be finite normalized torques in [-1, 1]")
        commands = [{"steps": self.physics_steps, "torques_nm": {
            agent: dict(zip(self.joint_names, (actions[i] * self._caps[i]).tolist()))
        }} for i, agent in enumerate(self.agent_ids)]
        try:
            results = self._request({"op": "step", "actions": commands}, "step")["results"]
            if any(r["status"] != "stepped" for r in results):
                raise RuntimeError(f"physics failure: {results}")
        except Exception:
            # Some worlds may already have advanced; never return a fabricated transition.
            self._needs_reset[:] = True
            raise
        self._raw = [r["observation"] for r in results]
        self._elapsed += 1
        error, speed = self._tracking()
        in_target = np.all((np.abs(error.reshape(self.num_envs, -1)) < 0.05)
                           & (np.abs(speed.reshape(self.num_envs, -1)) < 0.1), axis=1)
        self._held = np.where(in_target, self._held + 1, 0)
        fallen = np.array([self._fallen(self._agent(i)) for i in range(self.num_envs)])
        success = (self._held >= 10) & ~fallen
        terminated = success | fallen
        truncated = (self._elapsed >= self.horizon) & ~terminated
        reward = -np.mean(error.reshape(self.num_envs, -1)**2, axis=1) - 0.001 * np.mean(actions**2, axis=1) + success - fallen
        self._needs_reset = terminated | truncated
        info = self._info()
        info.update(is_success=success, fallen=fallen, angle_error_rad=error)
        return self._observations(), reward, terminated, truncated, info

    def _sample_target(self, rng):
        return rng.choice([-1, 1]) * rng.uniform(0.15, 0.3)

    def _tracking(self):
        positions = np.array([[self._agent(i)["joints"][j]["position_rad"]
                               for j in self.target_joints] for i in range(self.num_envs)])
        velocities = np.array([[self._agent(i)["joints"][j]["velocity_rad_s"]
                                for j in self.target_joints] for i in range(self.num_envs)])
        shape = (self.num_envs, *self._target_shape)
        return positions.reshape(shape) - self.targets, velocities.reshape(shape)

    @staticmethod
    def _fallen(agent):
        x, y, z, w = agent["rotation_xyzw"]
        # URDF local +Z is up; the arena maps it to world +Y at spawn.
        return agent["position_m"][1] < 0.02 or 2 * (y*z - x*w) < 0.5

    def close_extras(self, **kwargs):
        if self._process is not None:
            if self._process.stdin:
                try:
                    self._process.stdin.close()
                except BrokenPipeError:
                    pass
            try:
                self._process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._process.terminate()
                try:
                    self._process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self._process.kill()
                    self._process.wait()
            if self._process.stdout:
                self._process.stdout.close()
        self._temp.cleanup()


class ArmPoseVectorEnv(ShoulderVectorEnv):
    """Simultaneous pan/lift/elbow tracking; all three must settle to succeed."""

    target_joints = ("shoulder_pan", "shoulder_lift", "elbow_flex")
    _target_shape = (3,)

    def _sample_target(self, rng):
        # Conservative joint-space targets inside all three authored URDF limits.
        return rng.choice([-1, 1], size=3) * rng.uniform(0.1, 0.3, size=3)


class ShoulderEnv(gym.Env):
    """Single-world Gymnasium adapter, also usable with standard training libraries."""

    metadata = {"render_modes": []}
    vector_type = ShoulderVectorEnv

    def __init__(self, executable, config, **kwargs):
        if len(config["environments"]) != 1:
            raise ValueError("ShoulderEnv requires exactly one environment")
        self.vector = self.vector_type(executable, config, **kwargs)
        self.action_space = self.vector.single_action_space
        self.observation_space = self.vector.single_observation_space

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        obs, info = self.vector.reset(seed=seed, options=options)
        return obs[0], {k: v[0] for k, v in info.items()}

    def step(self, action):
        obs, reward, terminated, truncated, info = self.vector.step(np.asarray(action)[None, :])
        return obs[0], float(reward[0]), bool(terminated[0]), bool(truncated[0]), {k: v[0] for k, v in info.items()}

    def close(self):
        self.vector.close()


class ArmPoseEnv(ShoulderEnv):
    """Single-world Gymnasium adapter for the simultaneous three-joint task."""

    vector_type = ArmPoseVectorEnv
