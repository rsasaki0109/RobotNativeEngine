import copy
import json
import os
from pathlib import Path
import unittest
import contact_bridge as b
import contact_clusters


class BridgeTests(unittest.TestCase):
    def audit(self):
        count, lower = 15, 8
        frames=[]
        for step in range(1,101):
            poses=[]
            for i in range(count):
                upper=i>=lower
                col=i-lower if upper else i
                y=(.160 if upper else .055)-9.81*.002**2 if step==1 else (.15 if upper else .05)
                poses.append([step*.002,(col-3.5+(.5 if upper else 0))*.105,y,0,0,0,0,1])
            frames.append(dict(post_step_poses=poses,loaded_dynamic_pairs=b.expected_pairs(count),
                               max_solver_iterations=127,all_solver_stopping_flags=True,
                               generated=120,retained=88))
        return dict(count=count,layout='bridge',mode='audit',debug_assertions=False,steps=100,
                    dt_ticks=2000000,margin_m=.02,hashes=[42]*100,final_hash=42,contact_frames=frames)

    def test_every_upper_box_bridges_two_lower_neighbors(self):
        self.assertEqual(b.expected_pairs(7),[[1,5],[2,5],[2,6],[3,6],[3,7],[4,7]])
        self.assertTrue(b.metrics(self.audit(),15)['passed'])
        self.assertTrue(b.connected(7,b.expected_pairs(7)))

    def test_floor_cannot_connect_disjoint_dynamic_groups(self):
        self.assertFalse(b.connected(4,[[1,2],[3,4]]))
        a=self.audit();a['contact_frames'][-1]['loaded_dynamic_pairs'].pop()
        m=b.metrics(a,15)
        self.assertFalse(m['gates']['final_connected'])
        self.assertFalse(m['gates']['expected_final_support'])

    def test_independent_obb_check_detects_unreported_pair_overlap(self):
        a=self.audit();poses=a['contact_frames'][50]['post_step_poses']
        poses[0][1:4]=poses[1][1:4]
        m=b.metrics(a,15)
        self.assertGreater(m['pair_penetration_m'],.09)
        self.assertFalse(m['passed'])

    def test_failed_solver_is_reported_even_with_good_geometry(self):
        a=self.audit();a['contact_frames'][50]['all_solver_stopping_flags']=False
        m=b.metrics(a,15)
        self.assertEqual(m['unconverged_steps'],1)
        self.assertFalse(m['passed'])

    def test_bad_pose_clock_and_pair_ids_are_rejected(self):
        a=self.audit();a['contact_frames'][50]['post_step_poses'][0][0]=0
        with self.assertRaises(ValueError):b.metrics(a,15)
        a=self.audit();a['contact_frames'][0]['loaded_dynamic_pairs'][0]=[0,9]
        with self.assertRaises(ValueError):b.metrics(a,15)
        a=self.audit();a['contact_frames'][0]['post_step_poses'][0][2]+=.001
        with self.assertRaises(ValueError):b.metrics(a,15)

    def test_replay_compares_diagnostics_and_poses(self):
        a=self.audit();c=copy.deepcopy(a);c['elapsed_s']=123
        b.equal_audits(a,c)
        c['contact_frames'][50]['post_step_poses'][0][2]+=.001
        with self.assertRaises(ValueError):b.equal_audits(a,c)

    def test_release_example_is_connected_and_passes_independent_geometry(self):
        binary=Path(os.environ.get('RNE_CROWD','target/release/examples/contact_crowd'))
        if not binary.is_file():self.skipTest('build release contact_crowd')
        audit=contact_clusters.run(binary,15,'bridge','audit')
        result=b.metrics(audit,15)
        self.assertTrue(result['passed'],json.dumps(result))
        self.assertGreater(result['connected_steps'],0)


if __name__=='__main__':unittest.main()
