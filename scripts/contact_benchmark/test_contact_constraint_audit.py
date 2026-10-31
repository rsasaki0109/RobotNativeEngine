import copy
import os
from pathlib import Path
import unittest
import numpy as np
import contact_constraint_audit as p


class ConstraintAuditTests(unittest.TestCase):
    def fixture(self):
        steps=np.zeros((6250,6));steps[:,0]=np.arange(1,6251)*8e-6;steps[:,1:5]=1
        points=np.zeros((6250,8));points[:,0]=np.arange(6250)
        points[:,2:5]=[10,-5,0]
        points[:,5:8]=[.5,1,0]
        return steps,points

    def test_coulomb_primal_dual_boundary_and_complementarity(self):
        s,x=self.fixture();r=p.mj_summary(s,x)
        self.assertEqual(r['max_primal_cone_excess_n'],0)
        self.assertEqual(r['max_dual_cone_violation_m_s2'],0)
        self.assertEqual(r['max_soft_complementarity_n_m_s2'],0)

    def test_dual_sign_error_is_visible(self):
        s,x=self.fixture();x[:,5]=-.5
        self.assertGreater(p.mj_summary(s,x)['max_dual_cone_violation_m_s2'],0)
        self.assertGreater(p.mj_summary(s,x)['max_soft_complementarity_n_m_s2'],0)

    def test_bad_primal_force_and_missing_points_are_detected(self):
        s,x=self.fixture();x[:,3]=-6
        self.assertEqual(p.mj_summary(s,x)['max_primal_cone_excess_n'],1)
        with self.assertRaises(ValueError):p.mj_summary(s,x[:-1])

    def test_detection_margin_control_retains_candidates_but_not_all_constraints(self):
        steps,points=p.run_mujoco([.02,1],margin_m=1e-4,gap_m=1e-4)
        summary=p.mj_summary(steps,points)
        self.assertEqual(summary['geometric_contact_counts'],[4])
        self.assertEqual(summary['constraint_contact_counts'],[0,2,4])
        self.assertIsNotNone(summary['first_spin_gate_time_s'])
        self.assertEqual(summary['iteration_cap_steps'],0)
        self.assertLess(summary['max_dual_cone_violation_m_s2'],1e-10)

    @unittest.skipUnless(os.environ.get('RNE_THRESHOLD'),'requires native release example')
    def test_native_candidate_counts_and_bias_validation(self):
        audit=p.native.run(Path(os.environ['RNE_THRESHOLD']).resolve(),True)
        r=p.native_summary(audit)
        self.assertEqual(r['candidate_counts'],[[4,4,4]]*25)
        self.assertTrue(r['all_solver_stopping_flags'])
        self.assertLess(r['max_normal_velocity_violation_m_s'],1e-7)
        changed=copy.deepcopy(audit)
        changed['contact_diagnostics'][0]['points'][0]['normal_constraint_velocity_m_s']+=.1
        with self.assertRaises(ValueError):p.native_summary(changed)
        changed=copy.deepcopy(audit);changed['settings']['regularization']=.001
        with self.assertRaises(ValueError):p.native_summary(changed)


if __name__=='__main__':
    unittest.main()
