"""Controller state isolation, saturation, selection, and physical reset replay."""
import copy
import unittest

import numpy as np

from arm_controller import ArmPoseController, DEFAULTS
from rne_gym import ArmPoseVectorEnv
from train_shoulder import evaluate
from train_stable_arm import INITIAL, robust_score, compare
from test_rne_gym import BINARY, CONFIG
from test_train_shoulder import HOLD


class ControllerTests(unittest.TestCase):
    def test_reference_rate_saturation_and_selective_reset(self):
        settings = copy.deepcopy(DEFAULTS)
        settings["settle_steps"] = 0
        policy = ArmPoseController(2, settings)
        names = tuple(sorted(HOLD))
        caps = np.ones((2, len(names))) * 10
        obs = np.zeros((2, 2 * len(names) + 15))
        obs[:, -5:-2] = [0.2, -0.2, 0.3]
        gains = np.stack([INITIAL, INITIAL])
        # Large measured position error saturates positive torque. Integration
        # must not accumulate into that saturation.
        obs[:, names.index("shoulder_pan")] = -100
        actions = policy(obs, names, caps, gains, HOLD)
        np.testing.assert_array_equal(policy.integral[:, 0], [0, 0])
        self.assertTrue((np.abs(policy.reference) <= settings["dt_s"] * settings["reference_speed_rad_s"]).all())
        self.assertTrue((np.abs(actions * caps) <= np.array([HOLD[n]["effort_limit_nm"] for n in names])).all())
        policy.integral[:, 0] = 0.2
        obs[:, names.index("shoulder_pan")] = 0.1
        obs[:, names.index("shoulder_pan") + len(names)] = -100
        policy(obs, names, caps, gains, HOLD)
        self.assertTrue((policy.integral[:, 0] < 0.2).all())
        prior = (policy.reference[1].copy(), policy.integral[1].copy(), policy.steps[1])
        policy.reset(np.array([True, False]))
        np.testing.assert_array_equal(policy.reference[0], 0)
        np.testing.assert_array_equal(policy.integral[0], 0)
        np.testing.assert_array_equal(policy.reference[1], prior[0])
        np.testing.assert_array_equal(policy.integral[1], prior[1])
        self.assertEqual(policy.steps[1], prior[2])

    def test_selection_prefers_completion_then_worst_case(self):
        def row(success, reward):
            return {"success": success, "return": reward}
        scores = robust_score([[row(True, -20), row(True, -20)],
                               [row(True, 100), row(False, 100)],
                               [row(True, -30), row(True, 100)]])
        self.assertGreater(scores[0], scores[1])
        self.assertGreater(scores[0], scores[2])

    def test_physics_success_and_controller_reset_replay(self):
        with ArmPoseVectorEnv(BINARY, CONFIG, horizon=480, physics_steps=1) as env:
            policy = ArmPoseController(2, DEFAULTS)
            gains = np.stack([INITIAL, INITIAL])
            first = evaluate(env, gains, [505], HOLD, action_fn=policy, reset_fn=policy.reset)
            second = evaluate(env, gains, [505], HOLD, action_fn=policy, reset_fn=policy.reset)
            self.assertEqual(first, second)
            self.assertEqual(first[0], first[1])
            self.assertTrue(first[0][0]["success"])

    def test_invalid_parameters_are_rejected(self):
        for key, value in [("dt_s", 0), ("settle_steps", -1),
                           ("integral_limit_nm", [1]), ("reference_speed_rad_s", float('nan'))]:
            config = copy.deepcopy(DEFAULTS)
            config[key] = value
            with self.assertRaises(ValueError):
                ArmPoseController(2, config)


if __name__ == "__main__":
    unittest.main()
