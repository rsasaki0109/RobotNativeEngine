import json
from pathlib import Path
import unittest
import numpy as np
import contact_frozen_newton as newton
import contact_frozen_release as reference
import contact_release_fast as fast
import contact_release_fast_compare as comparison


class ReleaseFastTests(unittest.TestCase):
    def test_residual_is_identical_in_open_stick_slide_and_boundary_branches(self):
        g=np.eye(3);mu=np.array([.5])
        for p in [np.zeros(3),np.array([.5,0.,1.]),np.array([-.2,.1,-.1])]:
            for b in [np.zeros(3),np.array([1.,0.,-1.]),np.array([0.,0.,1.])]:
                np.testing.assert_array_equal(fast.residual(g,b,mu,p),newton.residual_jacobian(g,b,mu,p)[0])
        rng=np.random.default_rng(496)
        for count in [1,4,16,32]:
            for _ in range(3):
                a=rng.normal(size=(3*count,3*count));g=a@a.T+np.eye(3*count)
                b=rng.normal(size=3*count);p=rng.normal(size=3*count);mu=rng.uniform(0,1,count)
                np.testing.assert_array_equal(fast.residual(g,b,mu,p),newton.residual_jacobian(g,b,mu,p)[0])

    def test_success_and_failure_trajectories_match_reference_exactly(self):
        evidence=Path(__file__).resolve().parents[2]/'docs/evidence'
        cases=json.loads((evidence/'contact-frozen-systems.json').read_text())['cases']
        for c in [cases[1],cases[5],cases[7]]:
            for start in ['seed','gs']:
                self.assertEqual(fast.solve(c,c[start]),reference.solve(c,c[start]))
        c=cases[0]
        self.assertEqual(fast.solve(c,c['seed'],max_releases=0),reference.solve(c,c['seed'],max_releases=0))
        with self.assertRaises(ValueError):fast.solve(c,c['seed'],max_iterations=-1)

    def test_second_validation_set_excludes_both_previous_sets(self):
        evidence=Path(__file__).resolve().parents[2]/'docs/evidence'
        def key(c):return json.dumps({k:c[k] for k in ['g','free','mu','seed','gs']},sort_keys=True)
        known=[key(c) for c in json.loads((evidence/'contact-frozen-systems.json').read_text())['cases']]
        for p in (evidence/'contact-release-holdout').glob('*.json'):
            known.extend(key(c) for c in json.loads(p.read_text())['cases'])
        fresh=[]
        original=json.loads((evidence/'contact-frozen-systems.json').read_text())
        for p in sorted((evidence/'contact-release-fast-validation').glob('*.json')):
            d=json.loads(p.read_text());previous=json.loads((evidence/'contact-release-holdout'/p.name).read_text())
            self.assertEqual(d['previous_selection_sha256'],comparison.digest(evidence/'contact-release-holdout'/p.name))
            self.assertTrue(set(previous['selected_indices'])<=set(d['excluded_indices']))
            count=next(c['failed_solves'] for c in original['captures'] if c['sha256']==d['raw_sha256'])
            self.assertEqual(d['selected_indices'],comparison.indices(count,d['excluded_indices']))
            self.assertFalse(set(d['selected_indices'])&set(d['excluded_indices']))
            fresh.extend(key(c) for c in d['cases'])
        self.assertEqual(len(fresh),16);self.assertEqual(len(set(fresh)),16)
        self.assertFalse(set(fresh)&set(known))


if __name__=='__main__':unittest.main()
