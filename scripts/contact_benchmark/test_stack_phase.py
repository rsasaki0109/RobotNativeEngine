import math
import os
from pathlib import Path
import unittest
import stack
import stack_phase as p


class PhaseTests(unittest.TestCase):
    def test_heights_change_phase_in_both_engines(self):
        for phase, mjphase in zip(p.PHASES, (0, 0.6, 0.2, 0.8)):
            time = math.sqrt(2 * p.gap(phase) / 9.81)
            self.assertAlmostEqual(time / 0.002 - 16, phase)
            self.assertAlmostEqual(time / 0.000015625 - math.floor(time / 0.000015625), mjphase)

    def test_failed_or_missing_phase_suppresses_comparison(self):
        rows = [{'phase': q, 'backend': b, 'metrics': {'passed': True}, 'median_cost': 1}
                for q in p.PHASES for b in ('native', 'mujoco')]
        self.assertTrue(p.comparison(rows)['eligible'])
        rows[0]['metrics']['passed'] = False
        self.assertFalse(p.comparison(rows)['eligible'])
        rows[0] = rows[1]
        self.assertFalse(p.comparison(rows)['eligible'])

    @unittest.skipUnless(os.environ.get('RNE_STACK_DROP'), 'requires native release runner')
    def test_native_phases_keep_frozen_quality(self):
        for phase in p.PHASES:
            case = {'dt_ticks': 2000000, 'levels': 3, 'top_mass_kg': 3, 'gap_m': p.gap(phase)}
            r = stack.run_case(Path(os.environ['RNE_STACK_DROP']).resolve(), case,
                               {'iterations': 20, 'regularization': 0})
            self.assertTrue(r['metrics']['passed'], (phase, r['metrics']))


if __name__ == '__main__':
    unittest.main()
