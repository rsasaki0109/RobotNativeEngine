import copy
import json
import os
from pathlib import Path
import unittest
import contact_tower as t
import contact_clusters


class TowerTests(unittest.TestCase):
    def audit(self, count=4):
        frames=[]
        for step in range(1,101):
            poses=[[step*.002,0,(.055+i*.105-9.81*.002**2 if step==1 else .05+i*.1),
                    0,0,0,0,1] for i in range(count)]
            frames.append(dict(post_step_poses=poses,
                loaded_dynamic_pairs=[[i,i+1] for i in range(1,count)],
                max_solver_iterations=100,all_solver_stopping_flags=True,generated=64,retained=32))
        return dict(count=count,layout='tower',mode='audit',debug_assertions=False,steps=100,
                    dt_ticks=2000000,margin_m=.02,hashes=[42]*100,final_hash=42,contact_frames=frames)

    def test_chain_support_and_geometry_across_depths(self):
        for count in t.COUNTS:
            m=t.metrics(self.audit(count),count)
            self.assertTrue(m['passed'])
            self.assertEqual(m['final_loaded_pairs'],count-1)
            self.assertIsNone(m['first_penetration_failure_step'])

    def test_unreported_overlap_is_measured_at_the_actual_step(self):
        a=self.audit();a['contact_frames'][50]['post_step_poses'][2][2]-=.01
        m=t.metrics(a,4)
        self.assertFalse(m['passed'])
        self.assertEqual(m['first_penetration_failure_step'],51)
        self.assertGreater(m['pair_penetration_m'],.009)

    def test_floor_penetration_is_independent_of_contact_diagnostics(self):
        a=self.audit();a['contact_frames'][30]['post_step_poses'][0][2]-=.01
        m=t.metrics(a,4)
        self.assertFalse(m['gates']['floor_penetration_m'])
        self.assertEqual(m['first_penetration_failure_step'],31)

    def test_solver_failure_is_not_hidden_by_good_geometry(self):
        a=self.audit();a['contact_frames'][20]['all_solver_stopping_flags']=False
        a['contact_frames'][20]['max_solver_iterations']=200
        m=t.metrics(a,4)
        self.assertFalse(m['passed'])
        self.assertEqual(m['first_unconverged_step'],21)
        self.assertEqual(m['unconverged_steps'],1)
        self.assertTrue(m['gates']['pair_penetration_m'])

    def test_missing_support_and_wrong_connected_chain_fail(self):
        a=self.audit();a['contact_frames'][-1]['loaded_dynamic_pairs'].pop()
        self.assertFalse(t.metrics(a,4)['gates']['final_connected'])
        a=self.audit();a['contact_frames'][-1]['loaded_dynamic_pairs']=[[1,2],[1,3],[3,4]]
        m=t.metrics(a,4)
        self.assertTrue(m['gates']['final_connected'])
        self.assertFalse(m['gates']['expected_final_support'])

    def test_invalid_trace_is_rejected(self):
        for field,value in [(0,0),(2,float('nan')),(7,2)]:
            a=self.audit();a['contact_frames'][50]['post_step_poses'][0][field]=value
            with self.assertRaises(ValueError):t.metrics(a,4)
        a=self.audit();a['contact_frames'][0]['post_step_poses'][0][2]+=.001
        with self.assertRaises(ValueError):t.metrics(a,4)
        a=self.audit();a['contact_frames'][0]['loaded_dynamic_pairs'][0]=[0,1]
        with self.assertRaises(ValueError):t.metrics(a,4)

    def test_timing_hash_and_quality_cannot_be_replaced(self):
        a=self.audit()
        sample=dict(count=4,layout='tower',mode='timing',hashes=[],contact_frames=[],
                    debug_assertions=False,final_hash=42,steps=100,dt_ticks=2000000,
                    margin_m=.02,elapsed_s=.1)
        row=dict(count=4,audit=a,timing=[copy.deepcopy(sample) for _ in range(5)])
        self.assertEqual(t.summary(row)['seconds_per_simulated_second'],.5)
        row['audit']['contact_frames'][5]['all_solver_stopping_flags']=False
        self.assertFalse(t.summary(row)['quality']['passed'])
        row['timing'][0]['final_hash']=43
        with self.assertRaises(ValueError):t.summary(row)

    def test_release_example_converges_and_replays(self):
        binary=Path(os.environ.get('RNE_CROWD','target/release/examples/contact_crowd'))
        if not binary.is_file():self.skipTest('build release contact_crowd')
        for count in t.COUNTS:
            a=contact_clusters.run(binary,count,'tower','audit')
            b=contact_clusters.run(binary,count,'tower','audit')
            t.equal_audits(a,b)
            m=t.metrics(a,count)
            self.assertTrue(m['passed'],json.dumps(m))
            timing=contact_clusters.run(binary,count,'tower','timing')
            self.assertEqual(timing['final_hash'],a['final_hash'])
            self.assertEqual(timing['contact_frames'],[])


if __name__=='__main__':unittest.main()
