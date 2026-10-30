import unittest
import os
from pathlib import Path
import contact_clusters
import contact_static_normal as s


class ResidualTests(unittest.TestCase):
    def audit(self):
        return dict(contact_frames=[dict(contact_samples=[[0,1,0,0,0,0,0,0,0,1,.5,0]])])

    def test_static_and_maximum_dissipation(self):
        self.assertTrue(s.residuals(self.audit())['passed'])
        a=self.audit();p=a['contact_frames'][0]['contact_samples'][0]
        p[3]=2;p[6]=-.5
        self.assertTrue(s.residuals(a)['passed'])
        p[6]=.5
        self.assertFalse(s.residuals(a)['gates']['dissipation_error_j'])

    def test_normal_gap_bias_and_loaded_slip(self):
        a=self.audit();p=a['contact_frames'][0]['contact_samples'][0]
        p[11]=.002;p[4]=-1
        self.assertTrue(s.residuals(a)['passed'])
        p[11]=-.001;p[4]=.1
        self.assertTrue(s.residuals(a)['passed'])
        p[4]=0
        self.assertFalse(s.residuals(a)['gates']['closing_velocity_m_s'])
        p[11]=0;p[3]=.01
        self.assertFalse(s.residuals(a)['gates']['interior_slip_m_s'])

    def test_invalid_and_infeasible_impulses(self):
        for index,value in [(9,-1),(6,1)]:
            a=self.audit();a['contact_frames'][0]['contact_samples'][0][index]=value
            self.assertFalse(s.residuals(a)['passed'])
        for index,value in [(1,2),(4,float('nan')),(7,.1)]:
            a=self.audit();a['contact_frames'][0]['contact_samples'][0][index]=value
            with self.assertRaises(ValueError):s.residuals(a)
        with self.assertRaises(ValueError):s.residuals(dict(contact_frames=[]))

    def test_release_tower_satisfies_independent_contact_residuals(self):
        binary=Path(os.environ.get('RNE_CROWD','target/release/examples/contact_crowd'))
        if not binary.is_file():self.skipTest('build release contact_crowd')
        for count in [4,8,16]:
            a=contact_clusters.run(binary,count,'tower','audit')
            result=s.residuals(a)
            self.assertTrue(result['passed'],str(result))


if __name__=='__main__':unittest.main()
