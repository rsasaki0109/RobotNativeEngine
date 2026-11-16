"""Stable-Baselines3 VecEnv over the Go2 task in MuJoCo, as `NativeGo2VecEnv` is in RNE.

Each environment is example 147's `mujoco_go2_learning.Go2` running the
episode of `Go2.episode_with` one control period at a time: the same
observation, reward, fall and time limit as `rne_py_locomotion.Go2LearningBatch`,
starts drawn as go2.rs draws them (one stream per environment), and MuJoCo
as the comparison's reference sets it (1 ms steps, a 4 ms contact time
constant, damping ratio 5; see example 148's README, "Three references").
`workers` processes each step a block of the environments, so training
uses the runner's cores as RNE's batch does with its threads. An episode
that ends resets at once and hands SB3 its last observation, as
`NativeGo2VecEnv` does.
"""

import math
import multiprocessing
import os
import sys

import numpy as np
from gymnasium import spaces
from stable_baselines3.common.vec_env import VecEnv

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                                "147_go2_learning"))
import mujoco_go2_learning as mj  # noqa: E402

# The comparison's MuJoCo: five physics steps to each of RNE's, contacts
# with a 4 ms time constant damped five times critically.
REFERENCE_PLANT = {"floor_friction": 1.0, "cone": "elliptic", "impratio": 100.0,
                   "substeps": 5, "solref_s": 0.004, "dampratio": 5.0}


class MujocoGo2Episode:
    """One Go2 in MuJoCo, stepped a control period at a time."""

    def __init__(self, plant, task, seed):
        self.go2 = mj.Go2(plant, task)
        self.random_starts = bool(task.get("random_starts"))
        self.gaps = mj.terrain_gaps(plant)
        self.stream = mj.Normal(seed ^ mj.START_STREAM)

    def next_start(self):
        if not self.random_starts:
            return (0.0, 0.0)
        width = mj.START_X_RANGE_M[1] - mj.START_X_RANGE_M[0]
        depth = mj.START_Y_RANGE_M[1] - mj.START_Y_RANGE_M[0]
        while True:
            point = (mj.START_X_RANGE_M[0] + width * self.stream.uniform(),
                     mj.START_Y_RANGE_M[0] + depth * self.stream.uniform())
            if all(point[0] < start - mj.START_GAP_CLEARANCE_M
                   or point[0] > end + mj.START_GAP_CLEARANCE_M for start, end in self.gaps):
                return point

    def reset(self):
        self.go2.settle(self.next_start())
        self.start = np.array(self.go2.base_position(), dtype=float)
        self.action = np.zeros(mj.ACTIONS)
        self.steps = 0
        self.last = (self.start[0], 0.0, 0.0)
        return self.go2.observe(0, self.action)

    def step(self, action):
        """One control period: (observation, reward, terminated, truncated)."""
        following = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        change = float(np.sum((following - self.action) ** 2))
        self.action = following
        self.go2.step(mj.DEFAULT_RAD + mj.ACTION_SCALE_RAD * following)
        self.steps += 1
        gravity_z = -self.go2.body_rotation()[2, 2]
        x, y, z = self.go2.base_position()
        half = self.go2.arena_half
        fell = bool(z - mj.ground_height(self.go2.terrain, x, y) < mj.FALL_HEIGHT_M
                    or gravity_z > mj.FALL_GRAVITY_Z or abs(x) > half or abs(y) > half)
        lateral, heading = abs(y - self.start[1]), abs(self.go2.heading())
        last_x, last_lateral, last_heading = self.last
        reward = ((x - last_x) - mj.LATERAL_PENALTY * (lateral - last_lateral)
                  - mj.HEADING_PENALTY * (heading - last_heading)
                  - mj.SMOOTHNESS_PENALTY * change / mj.EPISODE_STEPS
                  - (mj.FALL_PENALTY if fell else 0.0))
        self.last = (x, lateral, heading)
        truncated = not fell and self.steps >= mj.EPISODE_STEPS
        return self.go2.observe(self.steps, self.action), reward, fell, truncated


def _worker(connection, plant, task, seeds):
    episodes = [MujocoGo2Episode(plant, task, seed) for seed in seeds]
    while True:
        command, payload = connection.recv()
        if command == "reset":
            connection.send([episode.reset() for episode in episodes])
        elif command == "step":
            results = []
            for episode, action in zip(episodes, payload):
                observation, reward, terminated, truncated = episode.step(action)
                terminal = None
                if terminated or truncated:
                    terminal = observation
                    observation = episode.reset()
                results.append((observation, reward, terminated, truncated, terminal))
            connection.send(results)
        else:
            connection.close()
            return


class MujocoGo2VecEnv(VecEnv):
    """Vectorized Go2 walking environments in MuJoCo, over `workers` processes.

    `task` takes `train_ppo.py`'s task keys: `height_scan`, `random_starts`,
    `velocity_limits` and `armature`; `terrain` is a `147_go2_learning
    --write-terrain` file (none: the flat floor)."""

    def __init__(self, n_envs=8, seed=1, workers=4, terrain=None, **task):
        plant = {**REFERENCE_PLANT, "terrain": terrain,
                 "armature": float(task.get("armature", 0.0)),
                 "velocity_limits": bool(task.get("velocity_limits", False))}
        episode_task = {"height_scan": bool(task.get("height_scan", False)),
                        "random_starts": bool(task.get("random_starts", False))}
        workers = max(1, min(workers, n_envs))
        blocks = np.array_split(np.arange(n_envs), workers)
        context = multiprocessing.get_context("fork")
        self._connections = []
        self._processes = []
        for block in blocks:
            parent, child = context.Pipe()
            process = context.Process(
                target=_worker,
                args=(child, plant, episode_task, [seed * 1_000_003 + int(i) for i in block]),
                daemon=True)
            process.start()
            child.close()
            self._connections.append(parent)
            self._processes.append(process)
        self._blocks = [len(block) for block in blocks]
        observations = mj.PROPRIOCEPTION + (mj.HEIGHT_SCAN_POINTS if episode_task["height_scan"]
                                            else 0)
        self._actions = None
        self.render_mode = None
        super().__init__(
            n_envs,
            spaces.Box(-math.inf, math.inf, shape=(observations,), dtype=np.float64),
            spaces.Box(-1.0, 1.0, shape=(mj.ACTIONS,), dtype=np.float64),
        )

    def reset(self):
        for connection in self._connections:
            connection.send(("reset", None))
        observations = [o for connection in self._connections for o in connection.recv()]
        return np.asarray(observations, dtype=np.float64)

    def step_async(self, actions):
        actions = np.clip(np.asarray(actions, dtype=np.float64), -1.0, 1.0)
        offset = 0
        for connection, size in zip(self._connections, self._blocks):
            connection.send(("step", actions[offset:offset + size]))
            offset += size

    def step_wait(self):
        results = [r for connection in self._connections for r in connection.recv()]
        observations = np.asarray([r[0] for r in results], dtype=np.float64)
        rewards = np.asarray([r[1] for r in results], dtype=np.float64)
        dones = np.asarray([r[2] or r[3] for r in results])
        infos = []
        for _, _, terminated, truncated, terminal in results:
            info = {}
            if terminal is not None:
                info["terminal_observation"] = np.asarray(terminal, dtype=np.float64)
                info["TimeLimit.truncated"] = bool(truncated and not terminated)
            infos.append(info)
        return observations, rewards, dones, infos

    def close(self):
        for connection in self._connections:
            try:
                connection.send(("close", None))
            except (BrokenPipeError, OSError):
                pass
        for process in self._processes:
            process.join(timeout=5)

    def get_images(self):
        return [None] * self.num_envs

    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False] * self.num_envs

    def env_method(self, method_name, *method_args, indices=None, **method_kwargs):
        raise NotImplementedError("MujocoGo2VecEnv has no per-environment Python methods")

    def get_attr(self, attr_name, indices=None):
        return [getattr(self, attr_name, None)] * self.num_envs

    def set_attr(self, attr_name, value, indices=None):
        setattr(self, attr_name, value)

    def seed(self, seed=None):
        return [None] * self.num_envs
