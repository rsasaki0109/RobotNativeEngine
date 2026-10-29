"""Long-hold success cannot reuse a short transient or bridge a violation."""
import copy
import unittest
from unittest.mock import patch

import numpy as np

from rne_gym import ArmPoseVectorEnv
from test_rne_gym import BINARY, CONFIG


class SustainedHoldTests(unittest.TestCase):
    def test_requires_entire_hold_and_restarts_after_velocity_violation(self):
        with ArmPoseVectorEnv(BINARY, CONFIG, horizon=2400, physics_steps=1, hold_steps=480) as env:
            env.reset(seed=42)
            raw = copy.deepcopy(env._raw)
            for i in range(2):
                for k, name in enumerate(env.target_joints):
                    raw[i]["agents"]["alice"]["joints"][name]["position_rad"] = env.targets[i, k]
            reply = {"results": [{"status": "stepped", "observation": r} for r in raw]}
            with patch.object(env, "_request", return_value=reply):
                for _ in range(479):
                    obs, _, done, _, _ = env.step(np.zeros((2, 9)))
                    self.assertFalse(done.any())
                np.testing.assert_allclose(obs[:, -2], 479 / 480)
                for r in raw:
                    r["agents"]["alice"]["joints"]["elbow_flex"]["velocity_rad_s"] = 0.11
                obs, _, done, _, _ = env.step(np.zeros((2, 9)))
                self.assertFalse(done.any())
                np.testing.assert_array_equal(obs[:, -2], 0)
                for r in raw:
                    r["agents"]["alice"]["joints"]["elbow_flex"]["velocity_rad_s"] = 0
                for _ in range(479):
                    _, _, done, _, _ = env.step(np.zeros((2, 9)))
                    self.assertFalse(done.any())
                obs, _, done, _, info = env.step(np.zeros((2, 9)))
                self.assertTrue(done.all())
                self.assertTrue(info["is_success"].all())
                np.testing.assert_array_equal(obs[:, -2], 1)

    def test_invalid_hold_duration_rejected_before_starting_server(self):
        for hold_steps in (0, -1, 1.5):
            with self.assertRaises(ValueError):
                ArmPoseVectorEnv("unused-binary", CONFIG, hold_steps=hold_steps)


if __name__ == "__main__":
    unittest.main()
