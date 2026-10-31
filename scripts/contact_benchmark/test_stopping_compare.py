import copy
import unittest
import os
from pathlib import Path
import contact_clusters
import contact_stopping_compare as s
import test_disturbed_tower


class StoppingCompareTests(unittest.TestCase):
    def row(self):
        a=test_disturbed_tower.DisturbedTowerTests().audit('tower-slide')
        for f in a['contact_frames']:f['all_contact_law_flags']=True
        t=dict(count=4,layout='tower-slide',mode='timing',debug_assertions=False,hashes=[],
               contact_frames=[],final_hash=42,steps=100,dt_ticks=2000000,margin_m=.02,elapsed_s=.1)
        return dict(count=4,layout='tower-slide',before=a,after=copy.deepcopy(a),
                    timing={name:[copy.deepcopy(t) for _ in range(5)] for name in ['before','after']})

    def test_quality_failure_is_retained_and_versions_can_differ(self):
        r=self.row();r['before']['contact_frames'][20]['all_solver_stopping_flags']=False
        r['after']['hashes'][-1]=43;r['after']['final_hash']=43
        for t in r['timing']['after']:t['final_hash']=43
        q=s.summary(r)
        self.assertFalse(q['quality']['before']['passed'])
        self.assertTrue(q['quality']['after']['passed'])
        self.assertEqual(q['seconds_per_simulated_second'],dict(before=.5,after=.5))

    def test_timed_state_must_match_its_own_audit(self):
        r=self.row();r['timing']['after'][0]['final_hash']=43
        with self.assertRaises(ValueError):s.summary(r)

    def test_stop_and_contact_law_flags_are_distinct(self):
        r=self.row();r['after']['contact_frames'][20]['all_contact_law_flags']=False
        q=s.summary(r)['after_certificates']
        self.assertEqual(q['stopped_but_not_certified_steps'],1)
        self.assertEqual(q['failed_steps'],1)
        del r['after']['contact_frames'][20]['all_contact_law_flags']
        with self.assertRaises(ValueError):s.summary(r)

    def test_release_certificate_reaches_native_contact_diagnostics(self):
        binary=Path(os.environ.get('RNE_CROWD','target/release/examples/contact_crowd'))
        if not binary.is_file():self.skipTest('build release contact_crowd')
        static=contact_clusters.run(binary,4,'tower','audit')
        self.assertEqual(s.certificates(static)['failed_steps'],0)
        moving=contact_clusters.run(binary,4,'tower-slide','audit')
        q=s.certificates(moving)
        self.assertGreater(q['stopped_but_not_certified_steps'],0)
        self.assertEqual(q['certified_but_velocity_or_impulse_failed_steps'],0)


if __name__=='__main__':unittest.main()
