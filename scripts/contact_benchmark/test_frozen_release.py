import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import numpy as np
import contact_frozen_release as release
import contact_frozen_newton as base


def redundant_case():
    a=np.array([[1,1,1],[1,1,-1],[1,-1,1],[1,-1,-1]],dtype=float)
    g=np.eye(12);g[np.ix_([2,5,8,11],[2,5,8,11])]=a@a.T
    b=np.zeros((4,3));b[:,2]=-1+1e-4*np.array([1,-1,-1,1])
    seed=np.zeros((4,3));seed[:,2]=.25
    return dict(label='inconsistent-loaded-plane',g=g.ravel().tolist(),free=b.ravel().tolist(),
        mu=[.5]*4,seed=seed.tolist(),gs=seed.tolist(),regularization=0,vt=1e-8,it=1e-10)


class FrozenReleaseTests(unittest.TestCase):
    def test_release_exposes_open_contacts_in_redundant_system(self):
        c=redundant_case();g,b,mu=base.system(c);p=np.array(c['seed']).ravel()
        _,s,v=np.linalg.svd(g);nullspace=v[s<1e-12*s[0]].T
        candidate,count=release.release(g,b,mu,p,nullspace,c['vt'],c['it'])
        self.assertEqual(count,2)
        self.assertLess(candidate['correction_velocity_inf'],1e-12)
        np.testing.assert_allclose(candidate['impulses'][3*candidate['contact']:3*candidate['contact']+3],0)
        result=release.solve(c,c['seed'])
        self.assertTrue(result['audit']['passed'],result)
        self.assertLessEqual(result['iterations'],50)
        self.assertLessEqual(len(result['releases']),8)
        np.testing.assert_allclose(np.array(result['impulses'])[:,2],[0,.50005,.50005,0],atol=1e-7)

    def test_projection_is_feasible_and_does_not_mutate_input(self):
        p=np.array([[2.,0.,1.],[1.,1.,-1.]])
        saved=p.copy();out=release.project(p,np.array([.5,.5])).reshape(-1,3)
        np.testing.assert_array_equal(p,saved)
        np.testing.assert_allclose(out,[[.5,0,1],[0,0,0]])

    def test_no_nullspace_and_budget_do_not_fabricate_success(self):
        c=redundant_case();g,b,mu=base.system(c);p=np.array(c['seed']).ravel()
        candidate,count=release.release(g,b,mu,p,np.zeros((12,0)),c['vt'],c['it'])
        self.assertIsNone(candidate);self.assertEqual(count,0)
        result=release.solve(c,c['seed'],max_iterations=0)
        self.assertFalse(result['audit']['passed'])
        self.assertEqual(result['iterations'],0);self.assertEqual(result['releases'],[])
        with self.assertRaises(ValueError):release.solve(c,c['seed'],max_releases=-1)

    def test_frozen_tilted_problem_passes_with_shared_iteration_budget(self):
        path=Path(__file__).resolve().parents[2]/'docs/evidence/contact-frozen-systems.json'
        c=next(c for c in json.loads(path.read_text())['cases']
               if c['label']=='tower-tilt-4-failed-solve-15')
        baseline=base.solve(c,c['gs'])
        self.assertFalse(baseline['audit']['passed'])
        result=release.solve(c,c['gs'])
        self.assertTrue(result['audit']['passed'])
        self.assertGreater(len(result['releases']),0)
        self.assertLessEqual(len(result['releases']),8)
        self.assertEqual(sum(a['iterations'] for a in result['attempts']),result['iterations'])
        self.assertLessEqual(result['iterations'],50)
        disabled=release.solve(c,c['gs'],max_releases=0)
        self.assertEqual(disabled['impulses'],baseline['impulses'])

    def test_report_verifies_replay_and_rejects_tampering(self):
        with tempfile.TemporaryDirectory() as directory:
            fixtures=Path(directory)/'fixtures.json';report=Path(directory)/'report.json'
            fixtures.write_text(json.dumps(dict(protocol='frozen_contact_systems_v1',cases=[redundant_case()])))
            args=[sys.executable,release.__file__,str(fixtures),str(report)]
            subprocess.run(args,check=True,capture_output=True)
            subprocess.run(args+['--verify'],check=True,capture_output=True)
            data=json.loads(report.read_text());data['max_releases']+=1
            report.write_text(json.dumps(data))
            self.assertNotEqual(subprocess.run(args+['--verify'],capture_output=True).returncode,0)


if __name__=='__main__':unittest.main()
