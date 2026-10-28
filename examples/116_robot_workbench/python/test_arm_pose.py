"""Three-joint task contract and real-physics policy replay."""
import copy
import json
import unittest
from unittest.mock import patch

import numpy as np
from gymnasium.utils.env_checker import check_env

from rne_gym import ArmPoseEnv, ArmPoseVectorEnv
from train_arm_pose import INITIAL, policy_actions
from train_shoulder import evaluate
from evaluate_shoulder import replay
from test_rne_gym import BINARY, CONFIG
from test_train_shoulder import HOLD


class ArmPoseTests(unittest.TestCase):
    def test_checker_and_masked_target_reset(self):
        config = copy.deepcopy(CONFIG)
        config["environments"] = config["environments"][:1]
        env = ArmPoseEnv(BINARY, config)
        try:
            check_env(env, skip_render_check=True)
        finally:
            env.close()
        with ArmPoseVectorEnv(BINARY, CONFIG) as env:
            obs, info = env.reset(seed=42)
            self.assertEqual(obs.shape, (2, 33))
            self.assertEqual(info["target_rad"].shape, (2, 3))
            self.assertTrue((np.abs(info["target_rad"]) >= 0.1).all())
            self.assertTrue((np.abs(info["target_rad"]) <= 0.3).all())
            env.step(np.zeros((2, 9)))
            previous = env._observations().copy()
            after, _ = env.reset(seed=42, options={"reset_mask": np.array([True, False])})
            np.testing.assert_array_equal(obs[0], after[0])
            np.testing.assert_array_equal(previous[1], after[1])

    def test_all_joints_must_hold_and_reward_uses_each_error(self):
        with ArmPoseVectorEnv(BINARY, CONFIG, horizon=100) as env:
            env.reset(seed=5)
            raw = copy.deepcopy(env._raw)
            for i in range(2):
                for k, name in enumerate(env.target_joints):
                    raw[i]["agents"]["alice"]["joints"][name]["position_rad"] = env.targets[i, k]
                raw[i]["agents"]["alice"]["joints"]["elbow_flex"]["position_rad"] += 0.12
            reply = {"results": [{"status": "stepped", "observation": r} for r in raw]}
            with patch.object(env, "_request", return_value=reply):
                for _ in range(10):
                    _, reward, term, _, info = env.step(np.zeros((2, 9)))
                    self.assertFalse(term.any())
                    np.testing.assert_allclose(reward, [-0.12**2 / 3] * 2)
                for i in range(2):
                    raw[i]["agents"]["alice"]["joints"]["elbow_flex"]["position_rad"] = env.targets[i, 2]
                for _ in range(9):
                    _, _, term, _, _ = env.step(np.zeros((2, 9)))
                    self.assertFalse(term.any())
                _, _, term, _, info = env.step(np.zeros((2, 9)))
                self.assertTrue(term.all())
                self.assertTrue(info["is_success"].all())

    def test_real_policy_parallel_replay_and_saved_task(self):
        candidates = np.stack([INITIAL, INITIAL])
        with ArmPoseVectorEnv(BINARY, CONFIG, horizon=48, physics_steps=1) as env:
            parallel = evaluate(env, candidates, [101], HOLD, action_fn=policy_actions)
        self.assertEqual(parallel[0], parallel[1])
        config = copy.deepcopy(CONFIG)
        config["workers"] = 1
        with ArmPoseVectorEnv(BINARY, config, horizon=48, physics_steps=1) as env:
            serial = evaluate(env, candidates, [101], HOLD, action_fn=policy_actions)
        self.assertEqual(parallel, serial)
        report = {"schema_version": 1, "algorithm": "cross_entropy_pd_gain_search", "task": "arm_pose_v1",
                  "target_joints": list(ArmPoseVectorEnv.target_joints),
                  "world": CONFIG["environments"][0], "horizon": 48, "physics_steps": 1,
                  "policy_log_gains": {"initial": INITIAL.tolist(), "learned": INITIAL.tolist()},
                  "evaluation_seeds": [101], "fixed_arm_hold": HOLD}
        self.assertEqual(replay(BINARY, json.loads(json.dumps(report))),
                         {"initial": parallel[0], "learned": parallel[1]})


if __name__ == "__main__":
    unittest.main()
