"""Capture failed coupled solves with the archived instrumentation patch.

The instrumented and original binaries must produce identical scene audits.
Only the first and maximum-loaded-normal-residual failed solve per scene are
selected. All raw captures remain in a new user-chosen directory.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
import contact_frozen_newton as newton
import contact_tower as tower


def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def select(rows):
    if not rows:raise ValueError('no failed solves captured')
    scores=[]
    for r in rows:
        g,b,mu=newton.system(r)
        q=newton.audit(g,b,mu,np.array(r['gs']),r['vt'],r['it'])
        scores.append(q['maxima']['loaded_normal_velocity_m_s'])
    return sorted(set([0,int(np.argmax(scores))]))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('instrumented',type=Path);p.add_argument('baseline',type=Path)
    p.add_argument('raw_directory',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args()
    if a.output.exists() or a.raw_directory.exists():p.error('new output and raw directory required')
    a.raw_directory.mkdir(parents=True)
    cases=[];captures=[]
    for layout in ['tower-slide','tower-tilt']:
        for count in [4,8]:
            raw=a.raw_directory/f'{layout}-{count}.jsonl'
            args=[str(count),'audit',layout]
            with raw.open('w') as err:
                result=subprocess.run([str(a.instrumented.resolve())]+args,stderr=err,
                    stdout=subprocess.PIPE,text=True,check=True,timeout=120)
            reference=subprocess.run([str(a.baseline.resolve())]+args,capture_output=True,
                                     text=True,check=True,timeout=120)
            tower.equal_audits(json.loads(result.stdout),json.loads(reference.stdout))
            audit=a.raw_directory/f'{layout}-{count}-audit.json';audit.write_text(result.stdout)
            rows=[json.loads(line) for line in raw.read_text().splitlines()]
            selected=select(rows)
            for i in selected:
                c=rows[i];c['label']=f'{layout}-{count}-failed-solve-{i+1}'
                c['selection']='first' if i==0 else 'maximum_loaded_normal_residual'
                cases.append(c)
            captures.append(dict(layout=layout,count=count,failed_solves=len(rows),
                selected_indices=selected,path=str(raw),sha256=digest(raw),audit_sha256=digest(audit)))
    report=dict(protocol='frozen_contact_systems_v1',source_base='67a35679',
        binary_sha256=digest(a.instrumented),baseline_binary_sha256=digest(a.baseline),
        capture_source_sha256=digest(Path(__file__)),
        selection_source_sha256=digest(Path(newton.__file__)),captures=captures,cases=cases)
    a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('Captured',len(cases),'frozen problems; four scene audits unchanged')


if __name__=='__main__':main()
