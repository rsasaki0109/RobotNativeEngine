"""Compare quality and cost when a contact algorithm intentionally changes trajectories."""
import argparse
import json
import math
import os
from pathlib import Path
import statistics
import numpy as np
import contact_bridge as bridge
import contact_clusters as clusters
import contact_crowd as base
import contact_tower as tower

CASES = [('tower',n) for n in [4,8,16]] + [('bridge',n) for n in [15,31]] + [
    (layout,n) for layout in ['stacks','clusters'] for n in [64,128]] + [('separated',1)]
SOURCES = ['contact_static_normal.py'] + tower.SOURCES
RESIDUAL_GATES = dict(negative_normal_impulse_n_s=1e-10, cone_violation_n_s=1e-10,
    closing_velocity_m_s=1e-8, loaded_normal_velocity_m_s=1e-8,
    interior_slip_m_s=2e-8, dissipation_error_j=1e-10)


def residuals(audit):
    """Reconstruct rigid normal bias and Coulomb residuals from raw world vectors.

    Loaded/interior tests use a 1e-10 Ns impulse threshold. Normal feasibility
    and cone membership are checked at every candidate, including zero load.
    """
    values = dict.fromkeys(RESIDUAL_GATES,0.0)
    samples = 0
    for f in audit['contact_frames']:
        x = np.asarray(f['contact_samples'],dtype=float)
        if not len(x):
            continue
        if (x.ndim != 2 or x.shape[1] != 12 or not np.isfinite(x).all()
                or not np.allclose(np.linalg.norm(x[:,:3],axis=1),1,atol=1e-10,rtol=0)
                or np.any(x[:,10] != .5)):
            raise ValueError('invalid contact sample')
        samples += len(x)
        n,v,jt = x[:,:3],x[:,3:6],x[:,6:9]
        jn,mu,gap = x[:,9],x[:,10],x[:,11]
        normal_speed = np.sum(n*v,axis=1)
        vn = normal_speed + np.where(gap>=0,1,.2)*gap/.002
        vt = v - normal_speed[:,None]*n
        slip = np.linalg.norm(vt,axis=1); tangent = np.linalg.norm(jt,axis=1)
        if np.any(np.abs(np.sum(n*jt,axis=1)) > 1e-10):
            raise ValueError('non-tangential friction impulse')
        loaded = jn > 1e-10
        interior = loaded & (mu*jn-tangent > 1e-10)
        row=dict(negative_normal_impulse_n_s=float(np.maximum(0,-jn).max()),
            cone_violation_n_s=float(np.maximum(0,tangent-mu*jn).max()),
            closing_velocity_m_s=float(np.maximum(0,-vn).max()),
            loaded_normal_velocity_m_s=float(np.abs(vn[loaded]).max()) if loaded.any() else 0.,
            interior_slip_m_s=float(slip[interior].max()) if interior.any() else 0.,
            dissipation_error_j=float(np.abs(tangent*slip+np.sum(jt*vt,axis=1)).max()))
        for k in values:values[k]=max(values[k],row[k])
    if not samples:raise ValueError('missing contact samples')
    gates={k:values[k]<=limit for k,limit in RESIDUAL_GATES.items()}
    return dict(maxima=values,gates=gates,passed=all(gates.values()),samples=samples)


def quality(a,layout,count):
    if layout=='tower':return tower.metrics(a,count)
    if layout=='bridge':return bridge.metrics(a,count)
    base.check_audits(a,a,count)
    if layout in ['stacks','clusters']:
        return dict(passed=True,contacts=clusters.contact_metrics(a,count,layout))
    return dict(passed=True,max_penetration_m=a['max_penetration_m'],max_spin_rad_s=a['max_spin_rad_s'])


def summary(row):
    layout,count=row['layout'],row['count']
    q={name:quality(row[name],layout,count) for name in ['before','after']}
    cost={}
    for name in ['before','after']:
        audit=row[name];times=row['timing'][name]
        if len(times)!=5:raise ValueError('five timings required')
        for t in times:
            if (t['count']!=count or t['layout']!=layout or t['mode']!='timing'
                    or t['debug_assertions'] or t['hashes'] or t['contact_frames']
                    or t['final_hash']!=audit['final_hash'] or t['steps']!=100
                    or t['dt_ticks']!=2000000 or t['margin_m']!=.02
                    or not math.isfinite(t['elapsed_s']) or t['elapsed_s']<=0):
                raise ValueError('timing fixture or state differs')
        cost[name]=statistics.median(t['elapsed_s']/.2 for t in times)
    result=dict(quality=q,seconds_per_simulated_second=cost,
                reduction_fraction=1-cost['after']/cost['before'])
    if layout=='tower':result['after_residuals']=residuals(row['after'])
    return result


def hashes():return {n:base.digest(Path(__file__).with_name(n)) for n in SOURCES}


def verify(report,binaries):
    if (report['protocol']!='contact_static_normal_v1'
            or report['source_sha256']!=hashes() or report['residual_gates']!=RESIDUAL_GATES
            or [(r['layout'],r['count']) for r in report['results']]!=CASES
            or report['binary_sha256']!={k:base.digest(v) for k,v in binaries.items()}):
        raise ValueError('provenance or cases differ')
    for row in report['results']:
        if summary(row)!=row['summary']:raise ValueError('summary differs')
        for name,binary in binaries.items():
            bridge.equal_audits(row[name],clusters.run(binary,row['count'],row['layout'],'audit'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('before',type=Path);p.add_argument('after',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--verify',action='store_true');p.add_argument('--first',choices=['before','after'],default='before')
    a=p.parse_args();binaries=dict(before=a.before,after=a.after)
    if a.verify:
        verify(json.loads(a.output.read_text()),binaries);print('Verified quality, residuals, costs and per-version replay');return
    if a.output.exists():p.error('new output required')
    os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
    report=dict(protocol='contact_static_normal_v1',source_sha256=hashes(),
        binary_sha256={k:base.digest(v) for k,v in binaries.items()},residual_gates=RESIDUAL_GATES,
        cpu_affinity=sorted(os.sched_getaffinity(0)),first=a.first,results=[])
    order=['before','after'] if a.first=='before' else ['after','before']
    for layout,count in CASES:
        row=dict(layout=layout,count=count,timing=dict(before=[],after=[]))
        for name,binary in binaries.items():
            row[name]=clusters.run(binary,count,layout,'audit')
            clusters.run(binary,count,layout,'timing')
        for i in range(5):
            for name in (order if i%2==0 else order[::-1]):
                row['timing'][name].append(clusters.run(binaries[name],count,layout,'timing'))
        row['summary']=summary(row);report['results'].append(row)
        print(json.dumps(dict(layout=layout,count=count,summary=row['summary'])),flush=True)
    a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');verify(report,binaries)


if __name__=='__main__':main()
