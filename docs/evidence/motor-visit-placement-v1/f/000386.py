"""Small hand-derived synthetic rational identities only; no native worlds."""
import unittest
from fractions import Fraction as F
from physical_reference import backward_euler, ideal_row, visit, one_slice


class PhysicalReferenceTests(unittest.TestCase):
    def test_uncapped_hand_solution_satisfies_implicit_si_equations(self):
        result = backward_euler(F(2), F(4), F(2), F(1, 2), F(0), F(0), F(1), F(4))
        self.assertEqual(result["velocity"], F(1, 2))
        self.assertEqual(result["coordinate"], F(1, 4))
        self.assertEqual(result["impulse"], F(1))
        self.assertEqual(result["torque"], F(2))
        self.assertEqual(F(4) * (1 - result["coordinate"]) - F(2) * result["velocity"], result["torque"])

    def test_cap_is_torque_times_duration(self):
        result = backward_euler(F(2), F(4), F(2), F(1, 2), F(0), F(0), F(10), F(1))
        self.assertEqual(result["impulse"], F(1, 2))
        self.assertEqual(result["torque"], F(1))
        self.assertEqual(result["velocity"], F(1, 4))
        self.assertEqual(result["coordinate"], F(1, 8))
        requested = 4 * (10 - result["coordinate"]) - 2 * result["velocity"]
        self.assertGreaterEqual(requested, result["torque"])

    def test_sign_symmetry(self):
        positive = backward_euler(F(2), F(4), F(2), F(1, 2), F(0), F(0), F(1), F(4))
        negative = backward_euler(F(2), F(4), F(2), F(1, 2), F(0), F(0), F(-1), F(4))
        for key in positive:
            self.assertEqual(negative[key], -positive[key])

    def test_nonzero_incoming_velocity_is_part_of_the_reference(self):
        result = backward_euler(F(1), F(2), F(3), F(1, 2), F(0), F(1, 2), F(1), F(4))
        self.assertEqual(result["impulse"], 0)
        self.assertEqual(result["coordinate"], F(1, 4))
        self.assertEqual(result["velocity"], F(1, 2))

    def test_normalized_scalar_fixed_point_equals_backward_euler(self):
        row = ideal_row(F(2), F(4), F(2), F(1, 2), F(0), F(1))
        physical_impulse, velocity = visit(row, F(0), F(0), 1 / (row["w"] + row["g"]), F(2))
        reference = backward_euler(F(2), F(4), F(2), F(1, 2), F(0), F(0), F(1), F(4))
        self.assertEqual(physical_impulse, reference["impulse"])
        self.assertEqual(velocity, reference["velocity"])
        for _ in range(3):
            next_impulse, _ = visit(row, F(0), physical_impulse, 1 / (row["w"] + row["g"]), F(2))
            self.assertEqual(next_impulse, physical_impulse)

    def test_current_uncapped_residual_multiplier_is_minus_g_over_w(self):
        row = ideal_row(F(1), F(2), F(3), F(1, 2), F(0), F(1))
        before = -row["rhs"]
        impulse, velocity = visit(row, F(0), F(0), 1 / row["w"], F(10))
        after = -row["rhs"] - velocity - row["g"] * impulse
        self.assertEqual(after, -row["g"] / row["w"] * before)

    def test_same_visit_count_different_integration_boundary(self):
        row = ideal_row(F(1), F(2), F(3), F(1, 2), F(0), F(1))
        post = one_slice(row, F(0), F(0), F(1, 2), F(2), 1, 2)
        pre = one_slice(row, F(0), F(0), F(1, 2), F(2), 3, 0)
        self.assertEqual(post["final_velocity"], F(3, 8))
        self.assertEqual(pre["final_velocity"], F(3, 8))
        self.assertEqual(post["coordinate"], F(1, 4))
        self.assertEqual(pre["coordinate"], F(3, 16))

    def test_repeated_visits_share_one_slice_cap(self):
        row = ideal_row(F(1), F(2), F(3), F(1, 2), F(0), F(10))
        p = F(0)
        for _ in range(7):
            p, _ = visit(row, F(0), p, 1 / row["w"], F(1, 2))
            self.assertLessEqual(abs(p), F(1, 2))

    def test_ideal_uniform_sphere_inertia_is_two_fifths_m_radius_squared(self):
        self.assertEqual(F(2, 5) * F(1) * F(1, 10) ** 2, F(1, 250))

    def test_invalid_physical_inputs_fail(self):
        with self.assertRaises(ValueError):
            backward_euler(F(0), F(4), F(2), F(1, 2), F(0), F(0), F(1), F(4))
        with self.assertRaises(ValueError):
            backward_euler(F(2), F(4), F(2), F(1, 2), F(0), F(0), F(1), F(-1))


if __name__ == "__main__":
    unittest.main()
