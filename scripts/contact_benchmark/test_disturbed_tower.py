import copy
import os
from pathlib import Path
import unittest
import contact_disturbed_tower as d
import contact_clusters
import test_contact_tower


class DisturbedTowerTests(unittest.TestCase):
    def audit(self,layout,count=4):
        a=test_contact_tower.TowerTests().audit(count);a['layout']=layout
        a['contact_frames'][0]['post_step_poses']=d.initial_step(layout,count).tolist()
        for f in a['contact_frames']:
            f['contact_samples']=[[0,1,0,0,0,0,0,0,0,1,.5,0] for _ in range(f['retained'])]
        return a

    def test_initial_motion_and_rotation_are_required(self):
        for layout,count in d.CASES:
            a=self.audit(layout,count)
            self.assertTrue(d.metrics(a,count,layout)['passed'])
            a['contact_frames'][0]['post_step_poses']=d.initial_step(
                'tower-tilt' if layout=='tower-slide' else 'tower-slide',count).tolist()
            with self.assertRaises(ValueError):d.metrics(a,count,layout)

    def test_failure_gates_survive_report_generation(self):
        a=self.audit('tower-slide');a['contact_frames'][35]['all_solver_stopping_flags']=False
        a['contact_frames'][35]['max_solver_iterations']=200
        q=d.metrics(a,4,'tower-slide')
        self.assertFalse(q['passed'])
        self.assertEqual(q['geometry_and_solver']['first_unconverged_step'],36)
        a=self.audit('tower-tilt');a['contact_frames'][50]['contact_samples'][0][4]=-.01
        q=d.metrics(a,4,'tower-tilt')
        self.assertTrue(q['geometry_and_solver']['passed'])
        self.assertFalse(q['contact_law']['passed'])
        self.assertEqual(q['stopped_but_residual_failed_steps'],1)
        self.assertEqual(q['first_residual_failure_step'],51)
        self.assertFalse(q['passed'])

    def test_missing_contact_samples_and_nonfinite_clock_are_rejected(self):
        a=self.audit('tower-slide');a['contact_frames'][40]['contact_samples'].pop()
        with self.assertRaises(ValueError):d.metrics(a,4,'tower-slide')
        a=self.audit('tower-tilt');a['contact_frames'][40]['post_step_poses'][0][0]=float('nan')
        with self.assertRaises(ValueError):d.metrics(a,4,'tower-tilt')

    def test_timing_requires_matching_state(self):
        a=self.audit('tower-slide')
        sample=dict(count=4,layout='tower-slide',mode='timing',debug_assertions=False,
                    hashes=[],contact_frames=[],final_hash=42,steps=100,dt_ticks=2000000,
                    margin_m=.02,elapsed_s=.1)
        row=dict(count=4,layout='tower-slide',audit=a,timing=[copy.deepcopy(sample) for _ in range(5)])
        self.assertEqual(d.summary(row)['seconds_per_simulated_second'],.5)
        row['timing'][0]['final_hash']=43
        with self.assertRaises(ValueError):d.summary(row)

    def test_release_example_replays_and_exercises_gauss_seidel(self):
        binary=Path(os.environ.get('RNE_CROWD','target/release/examples/contact_crowd'))
        if not binary.is_file():self.skipTest('build release contact_crowd')
        for layout,count in d.CASES:
            a=contact_clusters.run(binary,count,layout,'audit')
            b=contact_clusters.run(binary,count,layout,'audit')
            d.tower.equal_audits(a,b)
            q=d.metrics(a,count,layout)
            self.assertGreater(q['gauss_seidel_steps'],0)
            t=contact_clusters.run(binary,count,layout,'timing')
            self.assertEqual(t['final_hash'],a['final_hash'])
            self.assertEqual(t['contact_frames'],[])


if __name__=='__main__':unittest.main()
