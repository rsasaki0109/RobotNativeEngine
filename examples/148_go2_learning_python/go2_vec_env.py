"""Stable-Baselines3 VecEnv over example 147's Go2 walking task.

`rne_py_locomotion.Go2LearningBatch` steps every environment on the available cores in
one Rust call and resets an environment as soon as its episode ends. Its
`terminal_observations` give SB3 the episode's last observation, which PPO
bootstraps a truncated episode from.
"""

import json
import math

import numpy as np
from gymnasium import spaces
from stable_baselines3.common.vec_env import VecEnv

import rne_py_locomotion


def _space_size(space):
    size = 0
    for tensor in space["tensors"]:
        count = 1
        for dimension in tensor["shape"]:
            count *= dimension
        size += count
    return size


class NativeGo2VecEnv(VecEnv):
    """Vectorized Go2 walking environments over one Rust batch call."""

    def __init__(self, n_envs=8, seed=1, threads=0, **task):
        self._seed, self._threads = seed, threads
        self._batch = rne_py_locomotion.Go2LearningBatch(n_envs, seed, threads, **task)
        self.task_spec = json.loads(
            self._batch.task_spec_json()
        )
        action_dim = _space_size(self.task_spec["action"])
        observation_dim = _space_size(self.task_spec["observation"])
        assert observation_dim == self._batch.observation_dim
        self._actions = None
        self.render_mode = None
        super().__init__(
            n_envs,
            spaces.Box(-math.inf, math.inf, shape=(observation_dim,), dtype=np.float64),
            spaces.Box(-1.0, 1.0, shape=(action_dim,), dtype=np.float64),
        )

    def reset(self):
        return np.asarray(self._batch.reset().observations, dtype=np.float64)

    def step_async(self, actions):
        self._actions = np.clip(np.asarray(actions, dtype=np.float64), -1.0, 1.0)

    def step_wait(self):
        result = self._batch.step(self._actions.tolist())
        observations = np.asarray(result.observations, dtype=np.float64)
        rewards = np.asarray(result.rewards, dtype=np.float64)
        terminated = np.asarray(result.terminated)
        truncated = np.asarray(result.truncated)
        infos = []
        for index, terminal in enumerate(result.terminal_observations):
            info = {}
            if terminal is not None:
                info["terminal_observation"] = np.asarray(terminal, dtype=np.float64)
                info["TimeLimit.truncated"] = bool(truncated[index] and not terminated[index])
            infos.append(info)
        return observations, rewards, np.logical_or(terminated, truncated), infos

    def base_positions(self):
        """Each environment's base position (x, y, z), Z-up, in meters."""
        return np.asarray(self._batch.base_positions())

    def foot_positions(self):
        """Each environment's foot centers, (n_envs, 4, 3), Z-up, in meters:
        front-left, front-right, rear-left, rear-right."""
        return np.asarray(self._batch.foot_positions())

    def levels(self):
        """Each environment's gap depth level (None: it starts anywhere)."""
        return self._batch.levels()

    def set_levels(self, levels):
        """Starts each environment's next episodes at its level in `levels`."""
        self._batch.set_levels(list(levels))

    def set_depth_curriculum(self, promote_after_gaps=2):
        """Moves each environment's level as its episodes end (None stops)."""
        self._batch.set_depth_curriculum(promote_after_gaps)

    def level_depths(self):
        """Each gap depth level's depth, in meters, shallowest first."""
        return self._batch.level_depths()

    def gap_intervals(self):
        """The gaps across the floor, (start, end) along x, in meters."""
        return [tuple(gap) for gap in self._batch.gap_intervals()]

    def set_task(self, **task):
        """Rebuilds the batch on a new task with the same environments, seed
        and spaces (a curriculum stage). Call `reset` before stepping."""
        batch = rne_py_locomotion.Go2LearningBatch(self.num_envs, self._seed, self._threads, **task)
        if batch.observation_dim != self._batch.observation_dim:
            raise ValueError("a new task must keep the observation width")
        self._batch = batch

    def close(self):
        return None

    def get_images(self):
        return [None] * self.num_envs

    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False] * self.num_envs

    def env_method(self, method_name, *method_args, indices=None, **method_kwargs):
        raise NotImplementedError("NativeGo2VecEnv has no per-environment Python methods")

    def get_attr(self, attr_name, indices=None):
        return [getattr(self, attr_name, None)] * self.num_envs

    def set_attr(self, attr_name, value, indices=None):
        setattr(self, attr_name, value)

    def seed(self, seed=None):
        return [None] * self.num_envs
