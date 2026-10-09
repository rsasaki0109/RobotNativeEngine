"""Regression tests for retaining failures and rejecting corrupt evidence."""

import copy
import json
import unittest
from unittest.mock import patch

import compare_heavy_grasp as comparison


def result(backend="native"):
    return {"schema_version": 1, "backend": backend, "cases": [
        {"mass_kg": mass, "case": label, "accepted": True,
         "lifted_m": 0.2, "palm_rise_m": 0.2, "slip_m": 0.0,
         "tilt_rad": 0.0, "step_us": 100.0}
        for mass, label in sorted(comparison.EXPECTED_CASES)
    ]}


class EvidenceTests(unittest.TestCase):
    def test_missing_duplicate_and_nonfinite_results_are_rejected(self):
        valid = result()
        missing = copy.deepcopy(valid)
        missing["cases"].pop()
        duplicate = copy.deepcopy(valid)
        duplicate["cases"][0] = duplicate["cases"][1]
        nonfinite = copy.deepcopy(valid)
        nonfinite["cases"][0]["slip_m"] = float("nan")
        for invalid in (missing, duplicate, nonfinite):
            with self.subTest(invalid=invalid):
                with patch.object(comparison.subprocess, "check_output",
                                  return_value=json.dumps(invalid)):
                    with self.assertRaises(ValueError):
                        comparison.run("unused", "native")

    def test_failure_in_one_run_is_retained_and_timing_is_not_physical(self):
        first = {backend: result(backend) for backend in comparison.BACKENDS}
        second = copy.deepcopy(first)
        second["rapier"]["cases"][0]["accepted"] = False
        second["native"]["cases"][0]["step_us"] = 200.0
        rows = comparison.summarize([first, second])
        native = rows[0]
        rapier = next(row for row in rows if row["backend"] == "rapier")
        self.assertTrue(native["exact_final_outputs_repeat"])
        self.assertEqual(native["step_us_median"], 150.0)
        self.assertFalse(rapier["accepted_every_run"])
        self.assertFalse(rapier["exact_final_outputs_repeat"])


if __name__ == "__main__":
    unittest.main()
