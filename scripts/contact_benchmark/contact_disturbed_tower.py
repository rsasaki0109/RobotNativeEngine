"""Audit tilted and moving-top towers without hiding failed contact-law gates."""
import argparse
import json
import math
import os
from pathlib import Path
import statistics
import numpy as np
import contact_tower as tower
import contact_static_normal as static
import contact_clusters as clusters
import contact_crowd as base

CASES = [(layout,n) for layout in ['tower-slide','tower-tilt'] for n in [4,8]]
SOURCES = ['contact_disturbed_tower.py','contact_static_normal.py'] + tower.SOURCES


def initial_step(layout,count):
    p=np.zeros((count,8));p[:,0]=.002;p[:,2]=.055+np.arange(count)*.105-9.81*.002**2
    p[:,7]=1
    if layout=='tower-slide':p[-1,1]=.1*.002
    elif layout=='tower-tilt':p[:,6]=math.sin(.03/2);p[:,7]=math.cos(.03/2)
    else:raise ValueError('unknown disturbed layout')
    return p


def metrics(audit,count,layout):
    if (layout,count) not in CASES:raise ValueError('case differs')
    q=tower.metrics(audit,count,layout=layout,initial=initial_step(layout,count))
    if any(len(f['contact_samples']) != f['retained'] for f in audit['contact_frames']):
        raise ValueError('contact sample coverage differs')
    residual=static.residuals(audit)
    failed=[i+1 for i,f in enumerate(audit['contact_frames'])
            if not static.residuals(dict(contact_frames=[f]))['passed']]
    stopped_failed=[i for i in failed if audit['contact_frames'][i-1]['all_solver_stopping_flags']]
    return dict(geometry_and_solver=q,contact_law=residual,
                residual_failed_steps=len(failed),
                first_residual_failure_step=failed[0] if failed else None,
                stopped_but_residual_failed_steps=len(stopped_failed),
                gauss_seidel_steps=sum(f['max_solver_iterations']>0 for f in audit['contact_frames']),
                passed=q['passed'] and residual['passed'])


def summary(row):
    count,layout,audit=row['count'],row['layout'],row['audit']
    if len(row['timing'])!=5:raise ValueError('five timing samples required')
    for t in row['timing']:
        if (t['count']!=count or t['layout']!=layout or t['mode']!='timing'
                or t['debug_assertions'] or t['hashes'] or t['contact_frames']
                or t['final_hash']!=audit['final_hash'] or t['steps']!=100
                or t['dt_ticks']!=2000000 or t['margin_m']!=.02
                or not math.isfinite(t['elapsed_s']) or t['elapsed_s']<=0):
            raise ValueError('timing fixture or state differs')
    return dict(quality=metrics(audit,count,layout),seconds_per_simulated_second=
                statistics.median(t['elapsed_s']/.2 for t in row['timing']))


def hashes():return {n:base.digest(Path(__file__).with_name(n)) for n in SOURCES}


def verify(report,binary):
    if (report['protocol']!='contact_disturbed_tower_v1' or report['source_sha256']!=hashes()
            or report['binary_sha256']!=base.digest(binary)
            or report['geometry_gates']!=tower.GATES or report['residual_gates']!=static.RESIDUAL_GATES
            or [(r['layout'],r['count']) for r in report['results']]!=CASES):
        raise ValueError('provenance or cases differ')
    for row in report['results']:
        if row['summary']!=summary(row):raise ValueError('summary differs')
        tower.equal_audits(row['audit'],clusters.run(binary,row['count'],row['layout'],'audit'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('binary',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--verify',action='store_true');a=p.parse_args()
    if a.verify:
        verify(json.loads(a.output.read_text()),a.binary)
        print('Verified measured quality, including failed gates, and fresh replays');return
    if a.output.exists():p.error('new output required')
    os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
    report=dict(protocol='contact_disturbed_tower_v1',source_sha256=hashes(),
                binary_sha256=base.digest(a.binary),cpu_affinity=sorted(os.sched_getaffinity(0)),
                geometry_gates=tower.GATES,residual_gates=static.RESIDUAL_GATES,results=[])
    for layout,count in CASES:
        audit=clusters.run(a.binary,count,layout,'audit')
        clusters.run(a.binary,count,layout,'timing')
        row=dict(layout=layout,count=count,audit=audit,
                 timing=[clusters.run(a.binary,count,layout,'timing') for _ in range(5)])
        row['summary']=summary(row);report['results'].append(row)
        print(json.dumps(dict(layout=layout,count=count,summary=row['summary'])),flush=True)
    a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');verify(report,a.binary)


if __name__=='__main__':main()
