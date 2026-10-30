import copy
import os
from pathlib import Path
import subprocess
import unittest
import predictive_gap as p


@unittest.skipUnless(os.environ.get('RNE_THRESHOLD'), 'requires native release example')
class PredictiveGapTests(unittest.TestCase):
    def setUp(self):
        self.binary = Path(os.environ['RNE_THRESHOLD']).resolve()

    def case(self, margin=.02, dt=2000000):
        return dict(dt_ticks=dt, initial_gap_m=.005, initial_speed_m_s=.2, margin_m=margin)

    def test_prediction_lands_without_force_before_crossing_step(self):
        c = self.case(); a = p.run(self.binary, c); m = p.summarize(c, a)
        self.assertEqual(m['first_loaded_step'], 16)
        self.assertEqual(m['landing_delay_steps'], 0)
        self.assertGreater(m['first_loaded_gap_m'], 0)
        self.assertLess(m['max_penetration_m'], 1e-8)
        self.assertLess(m['free_flight_position_error_m'], 1e-12)
        self.assertEqual(a, p.run(self.binary, c))

    def test_narrow_margin_misses_at_coarse_step_but_not_fine_step(self):
        coarse = self.case(.0001)
        fine = self.case(.0001, 250000)
        a = p.summarize(coarse, p.run(self.binary, coarse))
        b = p.summarize(fine, p.run(self.binary, fine))
        self.assertEqual(a['landing_delay_steps'], 1)
        self.assertGreater(a['max_penetration_m'], .0003)
        self.assertEqual(b['landing_delay_steps'], 0)
        self.assertLess(b['max_penetration_m'], 1e-8)

    def test_free_flight_and_pipeline_tampering_rejected(self):
        c = self.case(); a = p.run(self.binary, c)
        changed = copy.deepcopy(a)
        changed['trace'][1][2] += .001
        changed['contact_diagnostics'][1]['before'][2] += .001
        with self.assertRaises(ValueError): p.summarize(c, changed)
        changed = copy.deepcopy(a)
        changed['contact_diagnostics'][0]['points'].pop()
        with self.assertRaises(ValueError): p.summarize(c, changed)

    def test_premature_contact_is_visible_in_metric(self):
        c = self.case(); a = p.run(self.binary, c)
        a['contact_diagnostics'][0]['points'][0]['normal_impulse_n_s'] = .001
        self.assertLess(p.summarize(c, a)['landing_delay_steps'], 0)

    def test_invalid_initial_conditions_are_rejected(self):
        for gap, speed in [('NaN', '0'), ('-1', '0'), ('.005', 'inf'), ('.005', '-.2')]:
            args = [str(self.binary), '2000000', '3', '1.2', '0', '200', 'audit',
                    '.5', 'wrench', 'diagnostics', '.02', gap, speed]
            self.assertNotEqual(subprocess.run(args, capture_output=True, timeout=120).returncode, 0)


if __name__ == '__main__':
    unittest.main()
