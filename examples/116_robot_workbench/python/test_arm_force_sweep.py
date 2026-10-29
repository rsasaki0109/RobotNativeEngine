"""Force sweep coverage and honest aggregation of unsuccessful trials."""
import unittest
from unittest.mock import patch

from audit_arm_force_sweep import evaluate, make_report, summarize


class ForceSweepTests(unittest.TestCase):
    def test_all_signed_axes_keep_the_same_frozen_policy_and_seeds(self):
        source = {"task": "stable_arm_pose_v1", "world": {}, "controller": {},
                  "fixed_arm_hold": {}, "policy_log_gains": {"learned": [1, 2]},
                  "evaluation_seeds": [1001, 1002]}
        report = make_report(source, [2, 4])
        vectors = [c["protocol"]["force_world_n"] for c in report["conditions"]]
        self.assertEqual(len(vectors), 12)
        self.assertEqual(len({tuple(v) for v in vectors}), 12)
        self.assertTrue(all(sum(v) in (-4, -2, 2, 4) for v in vectors))
        with patch("audit_arm_force_sweep.run", return_value=[]) as runner:
            results = evaluate("engine", report)
        self.assertEqual([r["protocol"] for r in results],
                         [c["protocol"] for c in report["conditions"]])
        self.assertEqual(runner.call_count, 12)
        for call in runner.call_args_list:
            self.assertEqual(call.args[1]["evaluation_seeds"], [1001, 1002])
            self.assertEqual(call.args[1]["policy_log_gains"], [1, 2])
        report["policy_log_gains"][0] = 99
        self.assertEqual(source["policy_log_gains"]["learned"], [1, 2])
        for strengths in ([], [0], [-1], [21], [float("nan")], [float("inf")], [2, 2]):
            with self.assertRaises(ValueError):
                make_report(source, strengths)

    def test_undisplaced_success_is_not_counted_as_recovery_and_failures_remain(self):
        records = []
        for seed, pre, delivered, displaced, success, fallen in (
                (1, True, 24, True, True, False),
                (2, True, 24, False, True, False),
                (3, True, 24, True, False, True),
                (4, False, 0, False, False, False)):
            records.append({"seed": seed, "precondition_met": pre, "delivered_force_steps": delivered,
                            "lanes": [{"success": pre}, {"success": success, "displaced": displaced,
                                                         "fallen": fallen}]})
        self.assertEqual(summarize(records, {"pulse_steps": 24}), {
            "cases": 4, "precondition_failures": 1, "incomplete_pulses": 1,
            "unforced_successes": 3, "forced_successes": 2, "displaced": 2,
            "displaced_and_recovered": 1, "forced_falls": 1, "failed_seeds": [3, 4]})


if __name__ == "__main__":
    unittest.main()
