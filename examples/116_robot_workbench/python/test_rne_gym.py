"""Run against the built Rust server: RNE_WORKBENCH=/path/to/rne-workbench python -m unittest."""
import copy
import json
import os
from pathlib import Path
import unittest
from unittest.mock import patch

from gymnasium.utils.env_checker import check_env
import numpy as np

from rne_gym import ShoulderEnv, ShoulderVectorEnv

ROOT = Path(__file__).resolve().parents[3]
BINARY = os.environ.get("RNE_WORKBENCH", str(ROOT / "target/debug/rne-workbench"))
CONFIG = json.loads((Path(__file__).parents[1] / "scenarios/vector_arena.json").read_text())


class LearningTests(unittest.TestCase):
    def test_feedback_policy_can_succeed_in_real_contact_world(self):
        config = copy.deepcopy(CONFIG)
        config["environments"] = config["environments"][:1]
        gains = json.loads((Path(__file__).parents[1] / "scenarios/wheel_arm_hold.json").read_text())["arm_hold"]["alice"]
        with ShoulderVectorEnv(BINARY, config, horizon=300, physics_steps=1) as env:
            obs, _ = env.reset(seed=42)
            for _ in range(300):
                actions = np.zeros(env.action_space.shape)
                for j, name in enumerate(env.joint_names):
                    if name not in gains:
                        continue
                    g = gains[name]
                    target = env.targets[0] if name == "shoulder_pan" else 0
                    torque = g["stiffness_nm_per_rad"] * (target - obs[0, j]) - g["damping_nm_s_per_rad"] * obs[0, j + 9]
                    actions[0, j] = np.clip(torque, -g["effort_limit_nm"], g["effort_limit_nm"]) / env._caps[0, j]
                obs, _, term, trunc, info = env.step(actions)
                if (term | trunc).any():
                    self.assertTrue(info["is_success"][0])
                    self.assertFalse(info["fallen"][0])
                    break
            else:
                self.fail("feedback baseline did not finish")

    def test_gymnasium_checker_real_physics(self):
        config = copy.deepcopy(CONFIG)
        config["environments"] = config["environments"][:1]
        env = ShoulderEnv(BINARY, config)
        try:
            check_env(env, skip_render_check=True)
        finally:
            env.close()
        self.assertIsNotNone(env.vector._process.poll())

    def test_replay_parallelism_masked_reset_and_time_limit(self):
        config = copy.deepcopy(CONFIG)
        config["workers"] = 1
        with ShoulderVectorEnv(BINARY, CONFIG, horizon=2, physics_steps=24) as parallel, ShoulderVectorEnv(BINARY, config, horizon=2, physics_steps=24) as serial:
            a, _ = parallel.reset(seed=42)
            b, _ = serial.reset(seed=42)
            np.testing.assert_array_equal(a, b)
            action = np.zeros(parallel.action_space.shape)
            action[0, parallel.joint_names.index("shoulder_pan")] = 0.001
            for _ in range(2):
                a = parallel.step(action)
                b = serial.step(action)
                for left, right in zip(a[:4], b[:4]):
                    np.testing.assert_array_equal(left, right)
                np.testing.assert_array_equal(a[4]["state_hash"], b[4]["state_hash"])
            self.assertTrue(a[3].all())
            self.assertFalse(a[2].any())
            with self.assertRaises(RuntimeError):
                parallel.step(action)
            prior = parallel._observations().copy()
            target = parallel.targets.copy()
            obs, info = parallel.reset(seed=42, options={"reset_mask": np.array([True, False])})
            np.testing.assert_array_equal(prior[1], obs[1])
            self.assertEqual(info["sim_time_ticks"][0], 0)
            np.testing.assert_array_equal(target, parallel.targets)
            with self.assertRaises(RuntimeError):
                parallel.step(action)
            parallel.reset(options={"reset_mask": np.array([False, True])})
            parallel.step(action)

    def test_invalid_actions_and_reset_leave_state_unchanged(self):
        with ShoulderVectorEnv(BINARY, CONFIG) as env:
            initial, _ = env.reset(seed=7)
            for action in [np.zeros((1, 9)), np.full((2, 9), np.nan), np.full((2, 9), 1.01)]:
                with self.assertRaises(ValueError):
                    env.step(action)
            for mask in [[1, 0], [False, False], [True]]:
                with self.assertRaises(ValueError):
                    env.reset(options={"reset_mask": mask})
            with self.assertRaises(ValueError):
                env.reset(seed=-1)
            np.testing.assert_array_equal(initial, env._observations())
            self.assertTrue(all(o["sim_time_ticks"] == 0 for o in env._raw))

    def test_success_fall_and_failure_semantics(self):
        with ShoulderVectorEnv(BINARY, CONFIG, horizon=10) as env:
            env.reset(seed=5)
            raw = copy.deepcopy(env._raw)
            for i in range(2):
                raw[i]["agents"]["alice"]["joints"]["shoulder_pan"]["position_rad"] = env.targets[i]
            reply = {"results": [{"status": "stepped", "observation": r} for r in raw]}
            with patch.object(env, "_request", return_value=reply):
                for _ in range(10):
                    _, reward, term, trunc, info = env.step(np.zeros((2, 9)))
                self.assertTrue(term.all())
                self.assertFalse(trunc.any())
                self.assertTrue(info["is_success"].all())
                np.testing.assert_array_equal(reward, [1, 1])
            env.reset(seed=5)
            raw[0]["agents"]["alice"]["rotation_xyzw"] = [1, 0, 0, 0]
            with patch.object(env, "_request", return_value=reply):
                _, reward, term, _, info = env.step(np.zeros((2, 9)))
                self.assertTrue(term[0])
                self.assertFalse(info["is_success"][0])
                self.assertEqual(reward[0], -1)
            env.reset()
            with patch.object(env, "_request", return_value={"results": [{"status": "failed"}]}):
                with self.assertRaises(RuntimeError):
                    env.step(np.zeros((2, 9)))
                self.assertTrue(env._needs_reset.all())


if __name__ == "__main__":
    unittest.main()
