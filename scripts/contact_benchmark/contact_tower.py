"""Frictional 4/8/16-level landing: independent geometry, convergence and cost.

Quality failures are saved as failed gates, not discarded or treated as malformed
reports. This short landing fixture does not establish long-term equilibrium.
"""
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
from contact_bridge import connected, equal_audits

COUNTS = [4, 8, 16]
GATES = dict(floor_penetration_m=1e-5, pair_penetration_m=1e-5)
SOURCES = ['contact_tower.py', 'contact_bridge.py', 'contact_clusters.py',
           'contact_crowd.py', 'stack.py', 'box.py']


def metrics(audit, count):
    frames = audit['contact_frames']
    if (count not in COUNTS or audit['count'] != count or audit['layout'] != 'tower'
            or audit['mode'] != 'audit' or audit['debug_assertions']
            or audit['steps'] != 100 or audit['dt_ticks'] != 2000000
            or audit['margin_m'] != .02 or len(frames) != 100
            or len(audit['hashes']) != 100 or audit['hashes'][-1] != audit['final_hash']):
        raise ValueError('fixture differs')
    poses = np.asarray([f['post_step_poses'] for f in frames], dtype=float)
    if (poses.shape != (100, count, 8) or not np.isfinite(poses).all()
            or not np.allclose(poses[:,:,0], (np.arange(1,101)*.002)[:,None], atol=1e-12, rtol=0)
            or not np.allclose(np.linalg.norm(poses[:,:,4:8],axis=2), 1, atol=1e-10, rtol=0)):
        raise ValueError('invalid pose trace')
    initial = np.zeros((count,8))
    initial[:,0] = .002
    initial[:,2] = .055 + np.arange(count)*.105 - 9.81*.002**2
    initial[:,7] = 1
    if not np.allclose(poses[0], initial, atol=1e-12, rtol=0):
        raise ValueError('first free-flight step differs')
    for f in frames:
        pairs = f['loaded_dynamic_pairs']
        connected(count, pairs)
        if (pairs != sorted(pairs) or len({tuple(p) for p in pairs}) != len(pairs)
                or type(f['all_solver_stopping_flags']) is not bool
                or not 0 <= f['max_solver_iterations'] <= 200
                or not f['generated'] >= f['retained'] >= len(pairs)):
            raise ValueError('invalid contact frame')
    floor = np.maximum(0, -np.stack([box.corners(poses[:,i])[:,:,1].min(axis=1)
                                    for i in range(count)]).min(axis=0))
    pair = np.stack([stack.pair_penetration(poses[:,i],poses[:,j])
                     for i in range(count) for j in range(i+1,count)]).max(axis=0)
    failed_steps = [i+1 for i,f in enumerate(frames) if not f['all_solver_stopping_flags']]
    overlap_steps = np.flatnonzero((floor > GATES['floor_penetration_m']) |
                                  (pair > GATES['pair_penetration_m'])) + 1
    result = dict(floor_penetration_m=float(floor.max()), pair_penetration_m=float(pair.max()),
                  final_loaded_pairs=len(frames[-1]['loaded_dynamic_pairs']),
                  connected_steps=sum(connected(count,f['loaded_dynamic_pairs']) for f in frames),
                  max_solver_iterations=max(f['max_solver_iterations'] for f in frames),
                  unconverged_steps=len(failed_steps),
                  first_unconverged_step=failed_steps[0] if failed_steps else None,
                  first_penetration_failure_step=int(overlap_steps[0]) if len(overlap_steps) else None)
    result['gates'] = {k: result[k] <= v for k,v in GATES.items()}
    result['gates'].update(final_connected=connected(count,frames[-1]['loaded_dynamic_pairs']),
        expected_final_support=frames[-1]['loaded_dynamic_pairs']==[[i,i+1] for i in range(1,count)],
        solver_stopped=not failed_steps)
    result['passed'] = all(result['gates'].values())
    return result


def summary(row):
    count, audit, samples = row['count'], row['audit'], row['timing']
    quality = metrics(audit,count)
    if len(samples) != 5:
        raise ValueError('five timing samples required')
    for s in samples:
        if (s['count'] != count or s['layout'] != 'tower' or s['mode'] != 'timing'
                or s['hashes'] or s['contact_frames'] or s['debug_assertions']
                or s['final_hash'] != audit['final_hash'] or s['steps'] != 100
                or s['dt_ticks'] != 2000000 or s['margin_m'] != .02
                or not math.isfinite(s['elapsed_s']) or s['elapsed_s'] <= 0):
            raise ValueError('timed fixture differs')
    return dict(quality=quality, seconds_per_simulated_second=statistics.median(
        s['elapsed_s']/.2 for s in samples))


def source_hashes():
    return {n:base.digest(Path(__file__).with_name(n)) for n in SOURCES}


def verify(report, binary):
    if (report['protocol'] != 'contact_tower_v1' or report['gates'] != GATES
            or [r['count'] for r in report['results']] != COUNTS
            or report['binary_sha256'] != base.digest(binary)
            or report['source_sha256'] != source_hashes()):
        raise ValueError('provenance or cases differ')
    for row in report['results']:
        if row['summary'] != summary(row):
            raise ValueError('summary differs')
        equal_audits(row['audit'], clusters.run(binary,row['count'],'tower','audit'))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('binary',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--verify',action='store_true')
    a = p.parse_args()
    if a.verify:
        verify(json.loads(a.output.read_text()),a.binary)
        print('Verified quality results (including failed gates), costs and fresh replays')
        return
    if a.output.exists():p.error('new output required')
    os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
    report=dict(protocol='contact_tower_v1',gates=GATES,cpu_affinity=sorted(os.sched_getaffinity(0)),
                binary_sha256=base.digest(a.binary),source_sha256=source_hashes(),results=[])
    for count in COUNTS:
        audit = clusters.run(a.binary,count,'tower','audit')
        clusters.run(a.binary,count,'tower','timing')
        row=dict(count=count,audit=audit,timing=[clusters.run(a.binary,count,'tower','timing')
                                               for _ in range(5)])
        row['summary']=summary(row);report['results'].append(row)
        print(json.dumps(dict(count=count,summary=row['summary'])),flush=True)
    a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    verify(report,a.binary)


if __name__=='__main__':main()
