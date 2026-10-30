import unittest
import friction_contact_diagnostics as p


class ContactDiagnosticsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.steps,cls.points,cls.warnings=p.run_mujoco([0.02,1])

    def test_contact_frame_and_moment_match_dynamics(self):
        r=p.summarize(self.steps,self.points)
        self.assertFalse(any(self.warnings))
        self.assertLess(r['force_balance_max_error_n'],1e-8)
        self.assertLess(r['moment_balance_max_error_nm'],1e-8)
        self.assertLess(r['constraint_wrench_max_error'],1e-8)
        self.assertLess(r['max_friction_cone_excess_n'],1e-8)
        self.assertAlmostEqual(r['first_spin_gate_time_s'],0.018944,places=9)
        self.assertEqual(r['at_spin_gate'][3],2)

    def test_wrong_force_sign_is_rejected(self):
        points=self.points.copy()
        points[:,10:13]*=-1
        with self.assertRaises(ValueError):
            p.summarize(self.steps,points)

    def test_wrong_moment_arm_is_rejected(self):
        points=self.points.copy()
        points[:,4]+=0.01
        with self.assertRaises(ValueError):
            p.summarize(self.steps,points)

    def test_missing_contact_is_rejected(self):
        with self.assertRaises(ValueError):
            p.summarize(self.steps,self.points[:-1])

    def test_wrong_loaded_count_is_rejected(self):
        steps=self.steps.copy()
        steps[1,3]+=1
        with self.assertRaises(ValueError):
            p.summarize(steps,self.points)

    def test_tampered_constraint_wrench_is_rejected(self):
        steps=self.steps.copy()
        steps[1,13]+=1
        with self.assertRaises(ValueError):
            p.summarize(steps,self.points)


if __name__=='__main__':
    unittest.main()
