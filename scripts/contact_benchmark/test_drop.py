"""Analytic audit fixtures, including failures the benchmark must retain."""
import math
import os
from pathlib import Path
import unittest

from drop import G, metrics, native


class AuditTests(unittest.TestCase):
    def ideal(self):
        case = {"dt_ticks": 1_000_000, "mass_kg": 1, "gap_m": 0.1}
        dt, height = 0.001, case["gap_m"]
        first = math.ceil(math.sqrt(2 * height / G) / dt)
        trace = []
        for i in range(2001):
            t = i * dt
            if i < first:
                trace.append([t, height - G * t * t / 2, -G * t, 0])
            else:
                trace.append([t, 0, 0, G * t if i == first else G * dt])
        return case, trace, first

    def test_analytic_inelastic_drop_passes(self):
        case, trace, _ = self.ideal()
        result = metrics(case, trace)
        self.assertTrue(result["passed"], result)
        self.assertLess(result["impact_impulse_relative_error"], 1e-12)

    def test_single_step_deep_penetration_is_not_hidden_by_settling(self):
        case, trace, first = self.ideal()
        trace[first][1] = -1e-4
        self.assertFalse(metrics(case, trace)["gates"]["max_penetration_m"])

    def test_missing_contact_fails_even_with_stationary_final_pose(self):
        case, trace, _ = self.ideal()
        for row in trace:
            row[3] = 0
        result = metrics(case, trace)
        self.assertFalse(result["passed"])
        self.assertIsNone(result["impact_impulse_relative_error"])

    def test_energy_injection_and_rebound_fail(self):
        case, trace, _ = self.ideal()
        trace[1000][1] = 0.2
        result = metrics(case, trace)
        self.assertFalse(result["gates"]["energy_excess_fraction"])
        self.assertFalse(result["gates"]["rebound_m"])

    def test_bad_clock_and_nonfinite_samples_rejected(self):
        case, trace, _ = self.ideal()
        trace[100][0] += 0.01
        with self.assertRaises(ValueError):
            metrics(case, trace)
        case, trace, _ = self.ideal()
        trace[100][1] = float("nan")
        with self.assertRaises(ValueError):
            metrics(case, trace)


@unittest.skipUnless(os.environ.get("RNE_CONTACT_DROP"), "set release RNE_CONTACT_DROP for integration")
class NativeReferenceTests(unittest.TestCase):
    def test_fixed_rigid_reference_and_audit_timing_agreement(self):
        binary = Path(os.environ["RNE_CONTACT_DROP"]).resolve()
        setting = {"regularization": 0.0, "iterations": 20}
        for mass, gap in ((1, 0.1), (0.1, 0.01), (0.1, 0.5), (10, 0.01), (10, 0.5)):
            with self.subTest(mass=mass, gap=gap):
                case = {"dt_ticks": 2_000_000, "mass_kg": mass, "gap_m": gap}
                audit = native(binary, case, setting, True)
                timing = native(binary, case, setting, False)
                self.assertFalse(audit["debug_assertions"])
                self.assertTrue(metrics(case, audit["trace"])["passed"])
                for field in ("final_gap_m", "final_velocity_m_s"):
                    self.assertEqual(audit[field], timing[field])


if __name__ == "__main__":
    unittest.main()
