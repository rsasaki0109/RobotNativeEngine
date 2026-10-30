import copy
import gzip
from pathlib import Path
import tempfile
import unittest

import numpy as np

from matched import choose, comparison, save_trace


def record(passed=True, cost=1, mass=1):
    return {"metrics": {"passed": passed}, "median_cost": cost, "min_cost": cost * .9,
            "max_cost": cost * 1.1, "case": {"mass_kg": mass, "gap_m": .1}}


class MatchedTests(unittest.TestCase):
    def test_fast_setting_with_one_failed_calibration_cannot_win(self):
        failing = {"records": [record(cost=.1), record(False, cost=.1)]}
        passing = {"records": [record(cost=2), record(cost=2)]}
        self.assertIs(choose([failing, passing]), passing)
        self.assertIsNone(choose([failing]))

    def test_failed_or_mismatched_validation_has_no_speedup(self):
        a = [record(mass=i) for i in range(4)]
        b = [record(cost=2, mass=i) for i in range(4)]
        self.assertEqual(comparison(a, b)["equal_weight_mean_median_cost_ratio"], 2)
        b[1]["metrics"]["passed"] = False
        self.assertFalse(comparison(a, b)["eligible"])
        self.assertFalse(comparison(a, b[:3])["eligible"])
        b[1]["case"]["mass_kg"] = 999
        with self.assertRaises(ValueError):
            comparison(a, b)

    def test_lossless_trace_roundtrip(self):
        samples = [[0, .1, 0, 0], [.001, -.00000001, .002, .1]]
        r = {"audit": {"trace": copy.deepcopy(samples)}}
        with tempfile.TemporaryDirectory() as temp:
            save_trace(Path(temp), r, 0)
            with gzip.open(Path(temp) / r["audit"]["trace_file"], "rb") as stream:
                np.testing.assert_array_equal(np.load(stream, allow_pickle=False), samples)
        self.assertNotIn("trace", r["audit"])
