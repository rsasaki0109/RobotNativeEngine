import copy
import json
import os
from pathlib import Path
import tempfile
import verify_force_friction_matched as verifier
import unittest
import force_friction_matched as p


class MatchedForceTests(unittest.TestCase):
    @unittest.skipUnless(os.environ.get('RNE_FORCE_MATCHED_REPORT'), 'requires calibration artifact')
    def test_verifier_rejects_tampered_quality_selection_and_digest(self):
        source=Path(os.environ['RNE_FORCE_MATCHED_REPORT'])
        report=json.loads((source/'report.json').read_text())
        with tempfile.TemporaryDirectory() as temp:
            target=Path(temp)
            for path in source.glob('trace-*.npz'):
                (target/path.name).symlink_to(path.resolve())
            mutations=[lambda r:r['records'][0]['metrics'].update(passed=False),
                       lambda r:r.update(selected={'dt_ticks':8000,'solref':[2e-6,10]}),
                       lambda r:r['records'][0].update(trace_sha256='0'*64)]
            for mutate in mutations:
                changed=copy.deepcopy(report)
                mutate(changed)
                (target/'report.json').write_text(json.dumps(changed))
                with self.assertRaises(ValueError):
                    verifier.verify(target)

    def test_calibration_and_validation_are_disjoint(self):
        self.assertFalse(set(p.CALIBRATION) & set(p.VALIDATION))
        self.assertFalse({v[0] for v in p.CALIBRATION} & {v[0] for v in p.VALIDATION})
        self.assertFalse({v[2] for v in p.CALIBRATION} & {v[2] for v in p.VALIDATION})

    def test_initial_static_speed_rejects_coarse_steps(self):
        for dt in (2_000_000,250_000,31_250):
            for ref in p.SOLREFS:
                a=p.run_mujoco(p.case(p.CALIBRATION[0],dt),ref,True,first_step=True)
                self.assertGreater(p.np.linalg.norm(a['trace'][1,8:11]),p.threshold.GATES['static_speed_m_s'])
        a=p.run_mujoco(p.case(p.CALIBRATION[0],8000),p.SOLREFS[0],True,first_step=True)
        self.assertLess(p.np.linalg.norm(a['trace'][1,8:11]),p.threshold.GATES['static_speed_m_s'])

    def test_cost_requires_complete_passing_validation(self):
        rows=[]
        for v in p.VALIDATION:
            for b,cost in [('native',2),('mujoco',4)]:
                rows.append(dict(phase='validation',backend=b,case=p.case(v,8000),
                                 metrics={'passed':True},median_cost=cost))
        self.assertEqual(p.comparison(rows)['ratio_mujoco_over_native'],2)
        self.assertFalse(p.comparison(rows[:-1])['eligible'])
        failed=copy.deepcopy(rows)
        failed[0]['metrics']['passed']=False
        self.assertFalse(p.comparison(failed)['eligible'])
        self.assertFalse(p.comparison(rows+rows)['eligible'])

    def test_solref_change_retains_force_geometry_and_material(self):
        c=p.case(p.CALIBRATION[1],8000)
        for ref in p.SOLREFS:
            model=p.mujoco.MjModel.from_xml_string(p.model_xml(c,ref))
            self.assertEqual(list(model.geom_solref[1]),ref)
            self.assertEqual(list(model.opt.gravity),[0,-9.81,0])
            self.assertEqual(model.body_mass[1],3)
            self.assertEqual(model.geom_friction[1,0],0.5)


if __name__=='__main__':
    unittest.main()
