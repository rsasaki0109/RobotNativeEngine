import unittest
import stack_matched as p


class ComparisonTests(unittest.TestCase):
    def rows(self):
        return [{'phase': 'validation', 'backend': b,
                 'case': {'levels': n, 'top_mass_kg': m},
                 'metrics': {'passed': True}, 'median_cost': 1 if b == 'native' else 2}
                for n, m in p.VALIDATION for b in ('native', 'mujoco')]

    def test_failure_suppresses_speedup(self):
        rows = self.rows()
        self.assertEqual(p.comparison(rows)['ratio_mujoco_over_native'], 2)
        rows[-1]['metrics']['passed'] = False
        self.assertFalse(p.comparison(rows)['eligible'])

    def test_duplicate_validation_cannot_replace_missing_case(self):
        rows = self.rows()
        rows[-1] = rows[1]
        self.assertFalse(p.comparison(rows)['eligible'])


if __name__ == '__main__':
    unittest.main()
