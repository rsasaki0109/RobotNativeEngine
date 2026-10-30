"""Paired cost and quality audits for explicit coupled contact-law certificates."""
import argparse
import json
import math
import os
from pathlib import Path
import statistics
import contact_disturbed_tower as disturbed
import contact_static_normal as static
import contact_tower as tower
import contact_bridge as bridge
import contact_clusters as clusters
import contact_crowd as base

CASES = disturbed.CASES + [('tower',n) for n in [4,8,16]] + [('bridge',n) for n in [15,31]] + [('separated',1)]
SOURCES = ['contact_stopping_compare.py'] + disturbed.SOURCES


def quality(a,count,layout):
    if layout.startswith('tower-'):return disturbed.metrics(a,count,layout)
    if layout=='tower':
        q=tower.metrics(a,count);r=static.residuals(a)
        return dict(geometry_and_solver=q,contact_law=r,passed=q['passed'] and r['passed'])
    if layout=='bridge':return bridge.metrics(a,count)
    base.check_audits(a,a,count)
    return dict(passed=True,max_penetration_m=a['max_penetration_m'],max_spin_rad_s=a['max_spin_rad_s'])


def certificates(audit):
    frames=audit['contact_frames']
    if not frames:return None
    if any(type(f.get('all_contact_law_flags')) is not bool for f in frames):
        raise ValueError('missing contact-law flag')
    result=dict(failed_steps=sum(not f['all_contact_law_flags'] for f in frames),
        stopped_but_not_certified_steps=sum(f['all_solver_stopping_flags'] and not f['all_contact_law_flags']
                                           for f in frames))
    if all('contact_samples' in f for f in frames):
        gates=[static.residuals(dict(contact_frames=[f]))['gates'] for f in frames]
        result['certified_but_external_audit_failed_steps']=sum(f['all_contact_law_flags'] and not all(g.values())
                                                               for f,g in zip(frames,gates))
        result['certified_but_velocity_or_impulse_failed_steps']=sum(f['all_contact_law_flags'] and
            not all(v for k,v in g.items() if k!='dissipation_error_j') for f,g in zip(frames,gates))
    return result


def summary(row):
    count,layout=row['count'],row['layout'];cost={}
    for name in ['before','after']:
        audit=row[name];samples=row['timing'][name]
        if len(samples)!=5:raise ValueError('five timing samples required')
        for t in samples:
            if (t['count']!=count or t['layout']!=layout or t['mode']!='timing'
                    or t['debug_assertions'] or t['hashes'] or t['contact_frames']
                    or t['final_hash']!=audit['final_hash'] or t['steps']!=100
                    or t['dt_ticks']!=2000000 or t['margin_m']!=.02
                    or not math.isfinite(t['elapsed_s']) or t['elapsed_s']<=0):
                raise ValueError('timing fixture or state differs')
        cost[name]=statistics.median(t['elapsed_s']/.2 for t in samples)
    return dict(quality={k:quality(row[k],count,layout) for k in ['before','after']},
                after_certificates=certificates(row['after']),
                seconds_per_simulated_second=cost,reduction_fraction=1-cost['after']/cost['before'])


def hashes():return {n:base.digest(Path(__file__).with_name(n)) for n in SOURCES}


def verify(report,binaries):
    if (report['protocol']!='contact_stopping_compare_v2' or report['source_sha256']!=hashes()
            or report['binary_sha256']!={k:base.digest(v) for k,v in binaries.items()}
            or report['geometry_gates']!=tower.GATES or report['residual_gates']!=static.RESIDUAL_GATES
            or [(r['layout'],r['count']) for r in report['results']]!=CASES):
        raise ValueError('provenance or cases differ')
    for row in report['results']:
        if row['summary']!=summary(row):raise ValueError('summary differs')
        for name,binary in binaries.items():
            tower.equal_audits(row[name],clusters.run(binary,row['count'],row['layout'],'audit'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('before',type=Path);p.add_argument('after',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--verify',action='store_true');p.add_argument('--first',choices=['before','after'],default='before')
    a=p.parse_args();binaries=dict(before=a.before,after=a.after)
    if a.verify:
        verify(json.loads(a.output.read_text()),binaries);print('Verified measured quality, costs and fresh per-version replays');return
    if a.output.exists():p.error('new output required')
    os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
    report=dict(protocol='contact_stopping_compare_v2',source_sha256=hashes(),
        binary_sha256={k:base.digest(v) for k,v in binaries.items()},first=a.first,
        cpu_affinity=sorted(os.sched_getaffinity(0)),geometry_gates=tower.GATES,
        residual_gates=static.RESIDUAL_GATES,results=[])
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
