"""Independent geometry and load-transfer failure fixtures for stacked cubes."""
import os
from pathlib import Path
import unittest

import numpy as np
import stack


class StackAuditTests(unittest.TestCase):
    def fixture(self):
        c = {'dt_ticks': 2_000_000, 'levels': 3, 'top_mass_kg': 10, 'gap_m': 0.005}
        x = np.zeros((1001, 3, 15))
        x[:, :, 0] = (np.arange(1001) * 0.002)[:, None]
        x[:, :, 2] = [0.05, 0.15, 0.25]
        x[0, :, 2] += [0.005, 0.01, 0.015]
        x[:, :, 7] = 1
        x[1:, :, 14] = np.array([12, 11, 10]) * 9.81 * 0.002
        return c, x

    def test_oriented_separation_despite_overlapping_aabbs(self):
        a = np.zeros((1, 15))
        a[:, 6:8] = [np.sin(np.pi / 8), np.cos(np.pi / 8)]
        b = a.copy()
        b[:, 1:3] = 0.11 / np.sqrt(2)
        self.assertEqual(float(stack.pair_penetration(a, b)[0]), 0)
        b[:, 1:3] = 0.09 / np.sqrt(2)
        self.assertAlmostEqual(float(stack.pair_penetration(a, b)[0]), 0.01)

    def test_pair_overlap_is_not_hidden_by_clear_floor(self):
        c, x = self.fixture()
        self.assertTrue(stack.metrics(c, x)['passed'])
        x[400, 2, 2] -= 0.001
        r = stack.metrics(c, x)
        self.assertTrue(r['gates']['max_floor_penetration_m'])
        self.assertFalse(r['gates']['max_pair_penetration_m'])

    def test_each_interface_must_carry_all_mass_above(self):
        c, x = self.fixture()
        x[1:, :, 14] = np.array([1, 1, 10]) * 9.81 * 0.002
        self.assertFalse(stack.metrics(c, x)['gates']['max_support_relative_error'])

    def test_lost_stack_is_not_a_successful_settle(self):
        c, x = self.fixture()
        x[-1, 2, 1] = 0.2
        x[-1, 2, 2] = 0.05
        self.assertFalse(stack.metrics(c, x)['gates']['final_height_error_m'])

    def test_invalid_quaternion_rejected(self):
        c, x = self.fixture()
        x[10, 0, 7] = 2
        with self.assertRaises(ValueError):
            stack.metrics(c, x)


@unittest.skipUnless(os.environ.get('RNE_STACK_DROP'), 'set release RNE_STACK_DROP for integration')
class StackNativeTests(unittest.TestCase):
    def test_fixed_200_iteration_reference_across_grid(self):
        for dt, levels, mass, iterations in stack.grid():
            if iterations != 200:
                continue
            c = {'dt_ticks': dt, 'levels': levels, 'top_mass_kg': mass, 'gap_m': 0.005}
            with self.subTest(case=c):
                row = stack.run_case(Path(os.environ['RNE_STACK_DROP']).resolve(), c,
                                     {'regularization': 0, 'iterations': iterations})
                self.assertTrue(row['metrics']['passed'], row['metrics'])



if __name__ == '__main__':
    unittest.main()
