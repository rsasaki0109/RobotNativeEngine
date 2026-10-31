import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import contact_release_holdout as h


def prepare(root):
    raw=root/'raw';raw.mkdir();captures=[]
    c=dict(g=[1,0,0,0,1,0,0,0,1],free=[0,0,-1],mu=[.5],seed=[[0,0,0]],
           gs=[[0,0,1]],regularization=0,vt=1e-8,it=1e-10)
    for layout in ['tower-slide','tower-tilt']:
        for count in [4,8]:
            path=raw/f'{layout}-{count}.jsonl'
            path.write_text('\n'.join(json.dumps(c) for _ in range(6))+'\n')
            captures.append(dict(layout=layout,count=count,failed_solves=6,
                selected_indices=[0,5],sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    original=root/'original.json';original.write_text(json.dumps(dict(captures=captures)))
    return raw,original


class HoldoutTests(unittest.TestCase):
    def test_selection_excludes_development_and_is_deterministic(self):
        self.assertEqual(h.indices(10,[0,9]),[2,4,6,8])
        self.assertEqual(h.indices(6,[0,5]),[1,2,3,4])
        with self.assertRaises(ValueError):h.indices(5,[0,4])

    def test_freeze_checks_raw_provenance_before_writing(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);raw,original=prepare(root);out=root/'heldout'
            h.freeze(raw,original,out)
            self.assertEqual(len(list(out.glob('*.json'))),4)
            for p in out.glob('*.json'):
                data=json.loads(p.read_text());self.assertEqual(data['selected_indices'],[1,2,3,4])
                self.assertEqual(data['excluded_indices'],[0,5])
            with (raw/'tower-slide-4.jsonl').open('a') as f:f.write('\n')
            with self.assertRaises(ValueError):h.freeze(raw,original,root/'bad')
            self.assertFalse((root/'bad').exists())

    def test_replay_checks_coverage_hashes_timing_and_summary(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);raw,original=prepare(root);fixtures=root/'heldout'
            h.freeze(raw,original,fixtures);output=root/'report.json'
            baseline=dict(audit=dict(passed=False),iterations=1)
            improved=dict(audit=dict(passed=True),iterations=2,release_candidates=1)
            with patch.object(h.newton,'solve',side_effect=lambda *args:copy.deepcopy(baseline)), \
                 patch.object(h.release,'solve',side_effect=lambda *args:copy.deepcopy(improved)), \
                 patch.object(h.os,'sched_setaffinity'):
                h.run(fixtures,output,first='release');h.run(fixtures,output,verify=True)
                report=json.loads(output.read_text())
                self.assertEqual(report['summary']['release']['passed'],32)
                for mutate in [lambda r:r['results'].pop(),
                               lambda r:r['results'][0]['timing']['baseline'].__setitem__(0,-1),
                               lambda r:r['summary']['release'].__setitem__('passed',0),
                               lambda r:r.__setitem__('source_sha256',{})]:
                    bad=copy.deepcopy(report);mutate(bad);output.write_text(json.dumps(bad))
                    with self.assertRaises(ValueError):h.run(fixtures,output,verify=True)

    def test_committed_holdout_inputs_do_not_duplicate_development_inputs(self):
        evidence=Path(__file__).resolve().parents[2]/'docs/evidence'
        def key(c):return json.dumps({k:c[k] for k in ['g','free','mu','seed','gs']},sort_keys=True)
        development={key(c) for c in json.loads((evidence/'contact-frozen-systems.json').read_text())['cases']}
        selected=[]
        for p in sorted((evidence/'contact-release-holdout').glob('*.json')):
            data=json.loads(p.read_text())
            self.assertFalse(set(data['selected_indices']) & set(data['excluded_indices']))
            selected.extend(key(c) for c in data['cases'])
        self.assertEqual(len(selected),16)
        self.assertEqual(len(set(selected)),16)
        self.assertFalse(set(selected)&development)


if __name__=='__main__':unittest.main()
