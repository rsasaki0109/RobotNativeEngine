"""Geometry and audit regressions for H2; inclined impacts may legitimately fail."""
import os
import hashlib
from pathlib import Path
import unittest

import numpy as np

import box
import verify_box


class BoxAuditTests(unittest.TestCase):
    def fixture(self):
        case = {"dt_ticks": 2_000_000, "angle_rad": 0, "gap_m": 0.1, "mass_kg": 1}
        x = np.zeros((1001, 15))
        x[:, 0] = np.arange(1001) * 0.002
        x[:, 2] = 0.05
        x[:, 7] = 1
        x[0, 2] += 0.1
        x[1:, 14] = box.G * 0.002
        return case, x

    def test_rotated_corner_geometry(self):
        _, x = self.fixture()
        angle = 0.4
        x[:, 6:8] = [np.sin(angle / 2), np.cos(angle / 2)]
        expected = x[:, 2] - 0.05 * (np.cos(angle) + np.sin(angle))
        np.testing.assert_allclose(box.corners(x)[:, :, 1].min(axis=1), expected, atol=1e-15)

    def test_single_corner_penetration_is_detected(self):
        case, x = self.fixture()
        self.assertTrue(box.metrics(case, x)["passed"])
        x[400, 6:8] = [np.sin(0.05), np.cos(0.05)]
        self.assertFalse(box.metrics(case, x)["gates"]["max_penetration_m"])

    def test_rotational_energy_and_settling_are_audited(self):
        case, x = self.fixture()
        x[-1, 11] = 100
        result = box.metrics(case, x)
        self.assertFalse(result["gates"]["energy_excess_fraction"])
        self.assertFalse(result["gates"]["settled_angular_speed_rad_s"])

    def test_missing_support_fails(self):
        case, x = self.fixture()
        x[:, 14] = 0
        self.assertFalse(box.metrics(case, x)["gates"]["settled_support_relative_error"])

    def test_invalid_state_rejected(self):
        for column, value in ((0, 42), (7, 2), (2, float("nan"))):
            case, x = self.fixture()
            x[40, column] = value
            with self.assertRaises(ValueError):
                box.metrics(case, x)

    def test_verifier_rejects_missing_grid_and_changed_metrics(self):
        report = {"protocol": "box_contact_h2_v1", "status": "complete",
                  "gates": box.GATES, "columns": box.COLUMNS,
                  "script_sha256": hashlib.sha256(Path(box.__file__).read_bytes()).hexdigest(),
                  "results": []}
        with self.assertRaisesRegex(ValueError, "incomplete grid"):
            verify_box.verify(report)
        case, trace = self.fixture()
        report["results"] = [{"case": case, "setting": {"regularization": 0, "iterations": 20},
                              "audit": {"trace": trace.tolist()}, "metrics": {}}]
        with self.assertRaisesRegex(ValueError, "stored metrics differ"):
            verify_box.verify(report)


@unittest.skipUnless(os.environ.get("RNE_BOX_DROP"), "set release RNE_BOX_DROP for integration")
class NativeBoxTests(unittest.TestCase):
    def test_flat_box_four_corner_support_and_exact_replay(self):
        case = {"dt_ticks": 2_000_000, "angle_rad": 0, "gap_m": 0.1, "mass_kg": 1}
        result = box.run_case(Path(os.environ["RNE_BOX_DROP"]).resolve(), case,
                              {"regularization": 0.0, "iterations": 200})
        self.assertTrue(result["metrics"]["passed"], result["metrics"])
        self.assertEqual(result["metrics"]["final_near_floor_vertices"], 4)


if __name__ == "__main__":
    unittest.main()
