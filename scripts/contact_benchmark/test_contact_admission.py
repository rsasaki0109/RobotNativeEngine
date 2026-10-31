import copy
import os
from pathlib import Path
import subprocess
import unittest
import contact_admission as p


class MuJoCoAdmissionTests(unittest.TestCase):
    def test_gap_only_retains_separated_points_without_constraints(self):
        probe = p.admission_probe(p.MJ_SETTINGS[1])
        self.assertEqual(len(probe), 4)
        for point in probe:
            self.assertAlmostEqual(point['distance_m'], 5e-5)
            self.assertEqual(point['include_margin_m'], 0)
            self.assertFalse(point['constraint_active'])

    def test_equal_margin_gap_already_activates_separated_points(self):
        for setting in p.MJ_SETTINGS[2:]:
            probe = p.admission_probe(setting)
            self.assertEqual(len(probe), 4)
            for point in probe:
                self.assertAlmostEqual(point['include_margin_m'], 2e-4)
                self.assertTrue(point['constraint_active'])
        self.assertEqual(p.admission_probe(p.MJ_SETTINGS[0]), [])


@unittest.skipUnless(os.environ.get('RNE_THRESHOLD'), 'requires native release example')
class AdmissionTests(unittest.TestCase):
    def setUp(self):
        self.binary = Path(os.environ['RNE_THRESHOLD']).resolve()

    def test_positive_admission_distance_prevents_observed_spin(self):
        zero = p.run_native(self.binary, 0.0)
        nearby = p.run_native(self.binary, .0001)
        default = p.run_native(self.binary, .02)
        a = p.summarize_native(zero, 0.0)
        b = p.summarize_native(nearby, .0001)
        self.assertEqual(a['retained_counts'], [2, 4])
        self.assertGreater(a['max_spin_rad_s'], .01)
        self.assertEqual(b['retained_counts'], [4])
        self.assertLess(b['max_spin_rad_s'], .01)
        self.assertGreater(b['positive_gap_loaded_samples'], 0)
        self.assertEqual(nearby['trace'], default['trace'])
        self.assertEqual(nearby, p.run_native(self.binary, .0001))

    def test_changed_distance_and_missing_candidate_rejected(self):
        audit = p.run_native(self.binary, .0001)
        with self.assertRaises(ValueError): p.summarize_native(audit, 0.0)
        changed = copy.deepcopy(audit)
        changed['contact_diagnostics'][0]['points'].pop()
        with self.assertRaises(ValueError): p.summarize_native(changed, .0001)
        changed = copy.deepcopy(audit)
        changed['contact_diagnostics'][0]['points'][0]['gap_m'] = .001
        with self.assertRaises(ValueError): p.summarize_native(changed, .0001)

    def test_invalid_margin_is_rejected(self):
        for value in ['-0.1', 'NaN', 'inf', '0.01']:
            r = subprocess.run([str(self.binary), '2000000', '3', '1.2', '0', '200',
                                'audit', '.5', 'wrench', 'diagnostics', value],
                               capture_output=True, text=True, timeout=120)
            self.assertNotEqual(r.returncode, 0)


if __name__ == '__main__':
    unittest.main()
