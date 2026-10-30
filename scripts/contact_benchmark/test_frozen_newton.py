import unittest
import json
import subprocess
import sys
import tempfile
from pathlib import Path
import contact_freeze as freeze
import numpy as np
import contact_frozen_newton as n


def case(g,b,mu):
    return dict(g=g.ravel().tolist(),free=b,mu=mu,regularization=0,vt=1e-8,it=1e-10,
                seed=np.zeros((len(mu),3)).tolist(),gs=np.zeros((len(mu),3)).tolist())


class FrozenNewtonTests(unittest.TestCase):
    def test_open_sticking_sliding_and_frictionless(self):
        for b,mu,expected in [([1,0,1],.5,[0,0,0]),([.1,0,-1],.5,[-.1,0,1]),
                              ([1,0,-1],.5,[-.5,0,1]),([1,0,-1],0,[0,0,1])]:
            c=case(np.eye(3),b,[mu]);r=n.solve(c,c['seed'])
            self.assertTrue(r['audit']['passed'],r)
            np.testing.assert_allclose(r['impulses'][0],expected,atol=1e-9)

    def test_redundant_contacts_need_no_compliance(self):
        g=np.kron(np.ones((2,2)),np.eye(3));c=case(g,[.1,0,-1]*2,[.5,.5])
        r=n.solve(c,c['seed']);self.assertTrue(r['audit']['passed'],r)
        np.testing.assert_allclose(np.sum(r['impulses'],axis=0),[-.1,0,1],atol=1e-9)

    def test_generalized_jacobian_matches_smooth_branches(self):
        g=np.array([[2.,.1,.2],[.1,3.,-.1],[.2,-.1,1.]])
        for b,p in [(np.array([1.,.3,-1.]),np.array([-.2,.1,1.])),
                    (np.array([.1,0.,-1.]),np.array([-.1,0.,1.])),
                    (np.array([1.,.3,1.]),np.array([.1,.1,.05]))]:
            f,j=n.residual_jacobian(g,b,np.array([.5]),p)
            numeric=np.column_stack([(n.residual_jacobian(g,b,np.array([.5]),p+1e-6*np.eye(3)[i])[0]
                                      -n.residual_jacobian(g,b,np.array([.5]),p-1e-6*np.eye(3)[i])[0])/2e-6
                                     for i in range(3)])
            np.testing.assert_allclose(j,numeric,atol=1e-8)

    def test_independent_audit_rejects_wrong_sliding_direction(self):
        g=np.eye(3);b=np.array([1.,0.,-1.]);mu=np.array([.5])
        self.assertFalse(n.audit(g,b,mu,np.array([.5,0.,1.]),1e-8,1e-10)['passed'])
        self.assertFalse(n.audit(g,b,mu,np.array([0.,0.,1.]),1e-8,1e-10)['passed'])

    def test_invalid_input_and_iteration_budget(self):
        c=case(np.eye(3),[1,0,-1],[.5])
        self.assertEqual(n.solve(c,c['seed'],max_iterations=0)['reason'],'iteration_limit')
        c['regularization']=.01
        with self.assertRaises(ValueError):n.system(c)
        c['regularization']=0;c['g'][0]=float('nan')
        with self.assertRaises(ValueError):n.system(c)

    def test_fixture_selection_and_report_verification(self):
        c=case(np.eye(3),[1,0,-1],[.5]);c['label']='sliding'
        self.assertEqual(freeze.select([c,c]),[0])
        worse=case(np.eye(3),[1,0,-2],[.5]);worse['gs']=[[0,0,1]]
        self.assertEqual(freeze.select([c,worse]),[0,1])
        with tempfile.TemporaryDirectory() as directory:
            fixtures=Path(directory)/'fixtures.json';report=Path(directory)/'report.json'
            fixtures.write_text(json.dumps(dict(protocol='frozen_contact_systems_v1',cases=[c])))
            args=[sys.executable,n.__file__,str(fixtures),str(report)]
            subprocess.run(args,check=True,capture_output=True)
            subprocess.run(args+['--verify'],check=True,capture_output=True)
            data=json.loads(report.read_text());data['results'][0]['result']['from_seed']['iterations']+=1
            report.write_text(json.dumps(data))
            self.assertNotEqual(subprocess.run(args+['--verify'],capture_output=True).returncode,0)


if __name__=='__main__':unittest.main()
