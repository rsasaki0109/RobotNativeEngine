import os
from pathlib import Path
import unittest
import numpy as np
import slide


class SlidingTests(unittest.TestCase):
    def fixture(self):
        c = {'dt_ticks': 2000000, 'mass_kg': 1, 'speed_m_s': 0.5, 'mu': 0.2}
        x = np.zeros((1001, 15))
        t = np.arange(1001)*0.002
        deceleration = float(np.float32(0.2))*9.81
        duration = 0.5/deceleration
        moving_time = np.minimum(t, duration)
        x[:, 0], x[:, 2], x[:, 7] = t, 0.05, 1
        x[:, 1] = 0.5*moving_time - 0.5*deceleration*moving_time**2
        x[:, 8] = np.maximum(0.5-deceleration*t, 0)
        return c, x

    def test_analytic_coulomb_slide_passes(self):
        c, x = self.fixture()
        self.assertTrue(slide.metrics(c, x)['passed'])

    def test_wrong_friction_fails_trajectory_and_distance(self):
        c, x = self.fixture()
        c['mu'] = 0.5
        r = slide.metrics(c, x)
        self.assertFalse(r['gates']['distance_error_m'])
        self.assertFalse(r['gates']['velocity_error_m_s'])

    def test_injected_rotation_and_lateral_drift_are_detected(self):
        c, x = self.fixture()
        x[-1, 11], x[-1, 3] = 100, 0.01
        r = slide.metrics(c, x)
        for gate in ('settled_spin_rad_s', 'lateral_drift_m', 'energy_excess_fraction'):
            self.assertFalse(r['gates'][gate])

    def test_bad_initial_state_rejected(self):
        c, x = self.fixture()
        x[0, 8] = 0
        with self.assertRaises(ValueError):
            slide.metrics(c, x)


@unittest.skipUnless(os.environ.get('RNE_SLIDE'), 'requires native release runner')
class NativeSlidingTests(unittest.TestCase):
    def test_high_friction_reference(self):
        case = {'dt_ticks': 2000000, 'mass_kg': 3, 'speed_m_s': 0.5, 'mu': 0.5}
        for reg in (0, 0.001):
            r = slide.measure(Path(os.environ['RNE_SLIDE']).resolve(), case,
                              {'regularization': reg, 'iterations': 200})
            self.assertTrue(r['metrics']['passed'], r['metrics'])


if __name__ == '__main__':
    unittest.main()
