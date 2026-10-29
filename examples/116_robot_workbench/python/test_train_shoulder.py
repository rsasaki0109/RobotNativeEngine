"""Verify policy-search episode accounting and saved-policy replay in real physics."""
import copy
import json
import unittest

import numpy as np

from rne_gym import ShoulderVectorEnv
from train_shoulder import INITIAL, SCENARIOS, evaluate, search
from evaluate_shoulder import replay
from test_rne_gym import BINARY, CONFIG

HOLD = json.loads((SCENARIOS / "wheel_arm_hold.json").read_text())["arm_hold"]["alice"]


class PolicySearchTests(unittest.TestCase):
    def test_finished_candidates_do_not_accrue_peer_rewards(self):
        config = copy.deepcopy(CONFIG)
        config["environments"].append(copy.deepcopy(config["environments"][0]))
        candidates = np.stack([np.log([3, 0.06]), np.log([3, 0.06]), INITIAL])
        with ShoulderVectorEnv(BINARY, config, horizon=300, physics_steps=1) as env:
            records = evaluate(env, candidates, [42], HOLD)
        self.assertEqual(records[0], records[1])
        self.assertEqual(len(records[0]), 1)
        self.assertTrue(records[0][0]["success"])
        self.assertLess(records[0][0]["steps"], records[2][0]["steps"])
        # Running only the fast candidate produces exactly its original episode,
        # even though batch evaluation resets it while the slow peer finishes.
        config["environments"] = config["environments"][:1]
        with ShoulderVectorEnv(BINARY, config, horizon=300, physics_steps=1) as env:
            isolated = evaluate(env, candidates[:1], [42], HOLD)
        self.assertEqual(records[0], isolated[0])

    def test_seeded_search_and_saved_policy_replay(self):
        with ShoulderVectorEnv(BINARY, CONFIG, horizon=24, physics_steps=1) as env:
            a, trace_a = search(env, HOLD, seed=8, generations=2, train_seeds=[11])
            b, trace_b = search(env, HOLD, seed=8, generations=2, train_seeds=[11])
            np.testing.assert_array_equal(a, b)
            self.assertEqual(trace_a, trace_b)
            evaluation = evaluate(env, np.stack([INITIAL, a]), [101], HOLD)
        report = {"schema_version": 1, "algorithm": "cross_entropy_pd_gain_search",
                  "world": CONFIG["environments"][0], "horizon": 24, "physics_steps": 1,
                  "initial_gains": np.exp(INITIAL).tolist(), "learned_gains": np.exp(a).tolist(),
                  "policy_log_gains": {"initial": INITIAL.tolist(), "learned": a.tolist()},
                  "evaluation_seeds": [101], "fixed_arm_hold": HOLD}
        restored = replay(BINARY, json.loads(json.dumps(report)))
        self.assertEqual(restored, {"initial": evaluation[0], "learned": evaluation[1]})


if __name__ == "__main__":
    unittest.main()
