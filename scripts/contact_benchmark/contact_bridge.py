"""Audit a load-connected staggered wall and compare exact-replay solver costs."""
import argparse
import json
import math
import os
from pathlib import Path
import statistics
import numpy as np
import box
import stack
import contact_crowd as base
import contact_clusters as clusters

COUNTS = [15, 31]
GATES = dict(floor_penetration_m=1e-5, pair_penetration_m=1e-5)
SOURCES = ['contact_bridge.py', 'contact_clusters.py', 'contact_crowd.py', 'stack.py', 'box.py']


def expected_pairs(count):
    lower = (count+1)//2
    return sorted([i+1+j, lower+i+1] for i in range(lower-1) for j in [0, 1])


def connected(count, pairs):
    neighbors = {i: set() for i in range(1, count+1)}
    for a, b in pairs:
        if not 1 <= a < b <= count:
            raise ValueError('invalid dynamic pair')
        neighbors[a].add(b)
        neighbors[b].add(a)
    seen, todo = set(), [1]
    while todo:
        i = todo.pop()
        if i not in seen:
            seen.add(i)
            todo.extend(neighbors[i]-seen)
    return len(seen) == count


def metrics(audit, count):
    frames = audit['contact_frames']
    if (count not in [15, 31, 63] or audit['count'] != count or audit['layout'] != 'bridge'
            or audit['mode'] != 'audit' or audit['debug_assertions']
            or audit['steps'] != 100 or audit['dt_ticks'] != 2000000
            or audit['margin_m'] != .02 or len(frames) != 100
            or len(audit['hashes']) != 100 or audit['hashes'][-1] != audit['final_hash']):
        raise ValueError('fixture differs')
    poses = np.asarray([f['post_step_poses'] for f in frames], dtype=float)
    if (poses.shape != (100, count, 8) or not np.isfinite(poses).all()
            or not np.allclose(poses[:, :, 0], (np.arange(1,101)*.002)[:,None], atol=1e-12, rtol=0)
            or not np.allclose(np.linalg.norm(poses[:,:,4:8],axis=2), 1, atol=1e-10, rtol=0)):
        raise ValueError('invalid pose trace')
    lower = (count+1)//2
    initial = np.zeros((count,8))
    initial[:,0] = .002
    initial[:,7] = 1
    for i in range(count):
        upper = i >= lower
        column = i-lower if upper else i
        initial[i,1] = (column-(lower-1)/2+(.5 if upper else 0))*.105
        initial[i,2] = (.160 if upper else .055)-9.81*.002**2
    if not np.allclose(poses[0], initial, atol=1e-12, rtol=0):
        raise ValueError('first free-flight step differs')
    for f in frames:
        pairs = f['loaded_dynamic_pairs']
        connected(count, pairs)  # validates IDs even before landing
        if (pairs != sorted(pairs) or len({tuple(p) for p in pairs}) != len(pairs)
                or not 0 <= f['max_solver_iterations'] <= 200
                or not f['generated'] >= f['retained'] >= len(pairs)):
            raise ValueError('invalid contact frame')
    floor = max(0, -min(float(box.corners(poses[:,i])[:,:,1].min()) for i in range(count)))
    pair = max(float(stack.pair_penetration(poses[:,i],poses[:,j]).max())
               for i in range(count) for j in range(i+1,count))
    result = dict(floor_penetration_m=floor, pair_penetration_m=pair,
                  final_loaded_pairs=len(frames[-1]['loaded_dynamic_pairs']),
                  connected_steps=sum(connected(count,f['loaded_dynamic_pairs']) for f in frames),
                  max_solver_iterations=max(f['max_solver_iterations'] for f in frames),
                  unconverged_steps=sum(not f['all_solver_stopping_flags'] for f in frames))
    result['gates'] = {k: result[k] <= v for k,v in GATES.items()}
    result['gates'].update(final_connected=connected(count,frames[-1]['loaded_dynamic_pairs']),
                           expected_final_support=frames[-1]['loaded_dynamic_pairs']==expected_pairs(count),
                           solver_stopped=result['unconverged_steps']==0)
    result['passed'] = all(result['gates'].values())
    return result


def equal_audits(a, b):
    if {k:v for k,v in a.items() if k!='elapsed_s'} != {k:v for k,v in b.items() if k!='elapsed_s'}:
        raise ValueError('audit state or diagnostics changed')


def summary(row):
    count = row['count']
    equal_audits(row['before'],row['after'])
    quality = metrics(row['after'], count)
    costs = {}
    for name in ['before','after']:
        samples = row['timing'][name]
        if len(samples) != 5:
            raise ValueError('five timing samples required')
        for s in samples:
            if (s['count'] != count or s['layout'] != 'bridge' or s['mode'] != 'timing'
                    or s['hashes'] or s['contact_frames'] or s['debug_assertions']
                    or s['final_hash'] != row[name]['final_hash'] or s['steps'] != 100
                    or s['dt_ticks'] != 2000000 or s['margin_m'] != .02
                    or not math.isfinite(s['elapsed_s']) or s['elapsed_s'] <= 0):
                raise ValueError('timed fixture differs')
        costs[name] = statistics.median(s['elapsed_s']/.2 for s in samples)
    return dict(quality=quality, seconds_per_simulated_second=costs,
                reduction_fraction=1-costs['after']/costs['before'])


def verify(report, binaries):
    if (report['protocol'] != 'contact_bridge_v1' or report['gates'] != GATES
            or [r['count'] for r in report['results']] != COUNTS
            or report['binary_sha256'] != {k:base.digest(v) for k,v in binaries.items()}
            or report['source_sha256'] != {n:base.digest(Path(__file__).with_name(n)) for n in SOURCES}):
        raise ValueError('provenance or cases differ')
    for row in report['results']:
        if row['summary'] != summary(row):
            raise ValueError('summary differs')
        for name,binary in binaries.items():
            equal_audits(row[name], clusters.run(binary,row['count'],'bridge','audit'))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('before',type=Path);p.add_argument('after',type=Path)
    p.add_argument('output',type=Path);p.add_argument('--verify',action='store_true')
    a = p.parse_args(); binaries=dict(before=a.before,after=a.after)
    if a.verify:
        verify(json.loads(a.output.read_text()),binaries)
        print('Verified poses, OBB quality, dynamic connectivity, costs and fresh replays')
        return
    if a.output.exists():p.error('new output required')
    os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
    report=dict(protocol='contact_bridge_v1',gates=GATES,cpu_affinity=sorted(os.sched_getaffinity(0)),
                binary_sha256={k:base.digest(v) for k,v in binaries.items()},
                source_sha256={n:base.digest(Path(__file__).with_name(n)) for n in SOURCES},results=[])
    for count in COUNTS:
        row=dict(count=count,timing=dict(before=[],after=[]))
        for name,binary in binaries.items():row[name]=clusters.run(binary,count,'bridge','audit')
        equal_audits(row['before'],row['after'])
        metrics(row['after'],count)
        for binary in binaries.values():clusters.run(binary,count,'bridge','timing')
        for i in range(5):
            for name in (['before','after'] if i%2==0 else ['after','before']):
                row['timing'][name].append(clusters.run(binaries[name],count,'bridge','timing'))
        row['summary']=summary(row); report['results'].append(row)
        print(json.dumps(dict(count=count,summary=row['summary'])),flush=True)
    a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    verify(report,binaries)


if __name__=='__main__':main()
