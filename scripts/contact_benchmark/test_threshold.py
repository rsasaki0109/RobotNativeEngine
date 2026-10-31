import os
from pathlib import Path
import unittest
import numpy as np
import threshold as p


class ThresholdTests(unittest.TestCase):
    def fixture(self, ratio):
        c = {'dt_ticks': 2000000, 'mass_kg': 3, 'force_ratio': ratio, 'mu': 0.5}
        x = np.zeros((1001, 15))
        t = np.arange(1001)*0.002
        acceleration = np.sign(ratio)*max(abs(ratio)-1, 0)*0.5*9.81
        x[:, 0], x[:, 2], x[:, 7] = t, 0.05, 1
        x[:, 1], x[:, 8] = 0.5*acceleration*t*t, acceleration*t
        return c, x

    def test_both_directions_static_and_sliding_analytic_solutions(self):
        for ratio in (-1.2, -0.8, 0.8, 1.2):
            c, x = self.fixture(ratio)
            self.assertTrue(p.metrics(c, x)['passed'])

    def test_spurious_motion_below_threshold_fails(self):
        c, x = self.fixture(0.8)
        x[:, 1] = x[:, 0]*0.001
        self.assertFalse(p.metrics(c, x)['gates']['static_drift_m'])

    def test_missing_or_wrong_direction_motion_above_threshold_fails(self):
        c, x = self.fixture(-1.2)
        x[:, 1] *= -1
        self.assertFalse(p.metrics(c, x)['gates']['position_error_m'])
        x[:, 8] = 0
        self.assertFalse(p.metrics(c, x)['gates']['velocity_error_m_s'])

    @unittest.skipUnless(os.environ.get('RNE_THRESHOLD'), 'requires native release runner')
    def test_native_threshold_reference_both_directions(self):
        for ratio in (-1.2, -0.8, 0.8, 1.2):
            c, _ = self.fixture(ratio)
            r = p.measure(Path(os.environ['RNE_THRESHOLD']).resolve(), c,
                          {'regularization': 0, 'iterations': 200})
            self.assertTrue(r['metrics']['passed'], r['metrics'])


if __name__ == '__main__':
    unittest.main()
