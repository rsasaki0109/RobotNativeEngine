import copy
import json
import os
from pathlib import Path
import subprocess
import unittest
import contact_batch as b


class BatchAccountingTests(unittest.TestCase):
    def sample(self):
        return dict(count=1, mode='timing', layout='separated', debug_assertions=False,
                    steps=100, dt_ticks=2000000, margin_m=.02, hashes=[], contact_frames=[],
                    max_penetration_m=None, max_spin_rad_s=None,
                    repeats=200, repeat_elapsed_s=[.001] * 200,
                    repeat_final_hashes=[42] * 200, final_hash=42, elapsed_s=.2)

    def test_uses_all_repeated_physical_seconds(self):
        self.assertAlmostEqual(b.cost(self.sample(), 42), .005)

    def test_a_bad_middle_repeat_is_not_hidden_by_final_hash(self):
        s = self.sample()
        s['repeat_final_hashes'][100] = 43
        with self.assertRaises(ValueError):
            b.cost(s, 42)

    def test_rejects_wrong_duration_and_missing_or_nonfinite_times(self):
        for key, value in [('elapsed_s', .001), ('elapsed_s', float('nan')),
                           ('repeat_elapsed_s', [.001] * 199),
                           ('repeat_elapsed_s', [float('inf')] * 200),
                           ('repeats', 1), ('steps', 20000), ('hashes', [42])]:
            s = copy.deepcopy(self.sample())
            s[key] = value
            with self.assertRaises(ValueError):
                b.cost(s, 42)


class BatchExampleTests(unittest.TestCase):
    def test_fresh_repeats_match_single_audit_and_reject_invalid_modes(self):
        binary = Path(os.environ.get('RNE_CROWD', 'target/release/examples/contact_crowd'))
        if not binary.is_file():
            self.skipTest('build contact_crowd release example')
        def run(*args):
            return subprocess.run([str(binary.resolve()), *args], capture_output=True,
                                  text=True, timeout=120)
        audit = run('1', 'audit')
        self.assertEqual(audit.returncode, 0, audit.stderr)
        repeated = run('1', 'timing', 'separated', '3')
        self.assertEqual(repeated.returncode, 0, repeated.stderr)
        row = json.loads(repeated.stdout)
        self.assertEqual(row['repeat_final_hashes'], [json.loads(audit.stdout)['final_hash']] * 3)
        self.assertEqual(len(row['repeat_elapsed_s']), 3)
        self.assertAlmostEqual(sum(row['repeat_elapsed_s']), row['elapsed_s'])
        for args in [('1', 'timing', 'separated', '0'),
                     ('1', 'timing', 'separated', '1001'),
                     ('1', 'audit', 'separated', '2'),
                     ('16', 'timing', 'separated', '2')]:
            self.assertNotEqual(run(*args).returncode, 0)


if __name__ == '__main__':
    unittest.main()
