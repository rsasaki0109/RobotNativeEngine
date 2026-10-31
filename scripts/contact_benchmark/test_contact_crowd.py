import copy
import unittest
import contact_crowd as p


class CrowdEvidenceTests(unittest.TestCase):
    def audit(self):
        return dict(count=16, mode='audit', debug_assertions=False, steps=100,
                    dt_ticks=2000000, margin_m=.02, hashes=[123]*100, final_hash=123,
                    max_penetration_m=1e-12, max_spin_rad_s=0, elapsed_s=.1)

    def test_hash_difference_and_quality_regression_rejected(self):
        a=self.audit();b=copy.deepcopy(a);b['hashes'][10]=456
        with self.assertRaises(ValueError):p.check_audits(a,b,16)
        b=copy.deepcopy(a);b['max_penetration_m']=1e-4
        with self.assertRaises(ValueError):p.check_audits(a,b,16)

    def test_timing_must_reach_the_audited_state(self):
        a=self.audit();s=dict(a,mode='timing',hashes=[],elapsed_s=.1)
        row=dict(count=16,before=a,after=a,timing=dict(before=[s]*5,after=[s]*5))
        self.assertEqual(p.summary(row)['speedup'],1)
        changed=copy.deepcopy(row);changed['timing']['after'][0]['final_hash']=456
        with self.assertRaises(ValueError):p.summary(changed)
        changed=copy.deepcopy(row);changed['timing']['after'].pop()
        with self.assertRaises(ValueError):p.summary(changed)


if __name__=='__main__':
    unittest.main()
