"""H3: aligned dynamic cube stacks, independent OBB geometry and load transfer."""
import argparse
import hashlib
import itertools
import json
import lzma
import os
from pathlib import Path
import platform
import statistics
import subprocess

import numpy as np
import box

GATES = {"max_floor_penetration_m": 1e-5, "max_pair_penetration_m": 1e-5,
         "settled_linear_speed_m_s": 1e-3, "settled_angular_speed_rad_s": 1e-2,
         "energy_excess_fraction": 1e-6, "max_support_relative_error": 0.01,
         "final_height_error_m": 1e-4}


def axes(x):
    """World-space local basis columns from xyzw quaternions."""
    qv = x[:, None, 4:7]
    t = 2 * np.cross(qv, np.eye(3)[None, :, :])
    return np.eye(3)[None, :, :] + x[:, None, 7:8] * t + np.cross(qv, t)


def pair_penetration(a, b):
    """Minimum translation overlap from all 15 separating axes, not AABB overlap."""
    aa, bb = axes(a), axes(b)
    cross = np.cross(aa[:, :, None, :], bb[:, None, :, :]).reshape(-1, 9, 3)
    candidates = np.concatenate((aa, bb, cross), axis=1)
    lengths = np.linalg.norm(candidates, axis=2)
    valid = lengths > 1e-12
    unit = candidates / np.where(valid, lengths, 1)[:, :, None]
    ra = 0.05 * np.abs(np.einsum('tki,tji->tkj', unit, aa)).sum(axis=2)
    rb = 0.05 * np.abs(np.einsum('tki,tji->tkj', unit, bb)).sum(axis=2)
    distance = np.abs(np.einsum('ti,tki->tk', b[:, 1:4] - a[:, 1:4], unit))
    overlap = np.where(valid, ra + rb - distance, np.inf)
    return np.maximum(0, overlap.min(axis=1))


def metrics(case, trace):
    x = np.asarray(trace, dtype=float)
    steps = 2_000_000_000 // case['dt_ticks']
    n = case['levels']
    dt = case['dt_ticks'] * 1e-9
    if x.shape != (steps + 1, n, 15) or not np.isfinite(x).all():
        raise ValueError('invalid trace')
    if not np.allclose(x[:, :, 0], (np.arange(steps + 1) * dt)[:, None], atol=1e-12, rtol=0):
        raise ValueError('invalid clock')
    if not np.allclose(np.linalg.norm(x[:, :, 4:8], axis=2), 1, atol=1e-10, rtol=0):
        raise ValueError('invalid quaternion')
    initial = np.zeros((n, 15))
    initial[:, 2] = 0.05 + case['gap_m'] + np.arange(n) * (0.1 + case['gap_m'])
    initial[:, 7] = 1
    if not np.allclose(x[0], initial, atol=1e-12, rtol=0):
        raise ValueError('initial state differs')
    masses = np.ones(n)
    masses[-1] = case['top_mass_kg']
    floor = min(float(box.corners(x[:, i])[:, :, 1].min()) for i in range(n))
    pair = max(float(pair_penetration(x[:, i], x[:, j]).max()) for i in range(n) for j in range(i + 1, n))
    speed = np.linalg.norm(x[:, :, 8:11], axis=2)
    spin = np.linalg.norm(x[:, :, 11:14], axis=2)
    energy = (masses * (9.81 * x[:, :, 2] + 0.5 * speed**2 + 0.5 * 0.1**2 / 6 * spin**2)).sum(axis=1)
    settled = x[:, 0, 0] > 1.5 + 1e-12
    expected = np.cumsum(masses[::-1])[::-1] * 9.81 * 0.5
    support_error = np.abs(x[settled, :, 14].sum(axis=0) / expected - 1)
    result = {'max_floor_penetration_m': max(0, -floor), 'max_pair_penetration_m': pair,
              'settled_linear_speed_m_s': float(speed[settled].max()),
              'settled_angular_speed_rad_s': float(spin[settled].max()),
              'energy_excess_fraction': max(0, float(energy.max() / energy[0] - 1)),
              'max_support_relative_error': float(support_error.max()),
              'support_relative_error_by_interface': support_error.tolist(),
              'final_height_error_m': float(np.abs(x[-1, :, 2] - (0.05 + np.arange(n) * 0.1)).max())}
    result['gates'] = {k: result[k] <= v for k, v in GATES.items()}
    result['gates']['nonnegative_normal_impulse'] = bool(x[:, :, 14].min() >= -1e-10)
    result['passed'] = all(result['gates'].values())
    return result


def run(binary, case, setting, audit):
    command = [str(binary), str(case['dt_ticks']), str(case['top_mass_kg']), str(case['gap_m']),
               str(setting['regularization']), str(setting['iterations']),
               'audit' if audit else 'timing', str(case['levels'])]
    return json.loads(subprocess.run(command, check=True, capture_output=True, text=True, timeout=120).stdout)


def run_case(binary, case, setting):
    audit = run(binary, case, setting, True)
    if audit['debug_assertions'] or audit['trace'] != run(binary, case, setting, True)['trace']:
        raise ValueError('release/exact replay requirement failed')
    if [r[1:14] for r in audit['trace'][-1]] != [r[1:14] for r in audit['final_state']]:
        raise ValueError('audit final state differs')
    run(binary, case, setting, False)
    costs = []
    for _ in range(5):
        timed = run(binary, case, setting, False)
        if timed['final_state'] != audit['final_state']:
            raise ValueError('timing final state differs')
        costs.append(timed['elapsed_s'] / 2)
    return {'case': case, 'setting': setting, 'audit': audit, 'metrics': metrics(case, audit['trace']),
            'exact_replay': True, 'seconds_per_simulated_second': costs, 'median_cost': statistics.median(costs)}


def grid():
    return itertools.product((1_000_000, 2_000_000), (2, 3), (0.1, 1, 10), (20, 200))


def verify(report):
    hashes = {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
              for name in ('stack.py', 'box.py')}
    if report['source_sha256'] != hashes or report['gates'] != GATES or report['protocol'] != 'stack_h3_v1':
        raise ValueError('evaluator/protocol differs')
    seen = set()
    for row in report['results']:
        c, s = row['case'], row['setting']
        key = (c['dt_ticks'], c['levels'], c['top_mass_kg'], s['iterations'])
        if key in seen or c['gap_m'] != 0.005 or s['regularization'] != 0:
            raise ValueError('invalid/duplicate case')
        seen.add(key)
        if row['metrics'] != metrics(c, row['audit']['trace']):
            raise ValueError('metrics differ')
        times = row['seconds_per_simulated_second']
        if len(times) != 5 or not all(0 < t < float('inf') for t in times) or statistics.median(times) != row['median_cost']:
            raise ValueError('cost differs')
        if row['audit']['debug_assertions'] or not row['exact_replay']:
            raise ValueError('release/replay missing')
        if [r[1:14] for r in row['audit']['trace'][-1]] != [r[1:14] for r in row['audit']['final_state']]:
            raise ValueError('final state differs')
    if seen != set(grid()) or report['status'] != 'complete':
        raise ValueError('incomplete grid')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('binary', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--verify', action='store_true', help='verify saved archive instead of simulating')
    args = p.parse_args()
    if args.verify:
        with lzma.open(args.output, 'rt') as f:
            report = json.load(f)
        verify(report)
        print('Verified 24 full-state stack records')
        return
    if args.output.exists():
        p.error('provide a new output path')
    binary = args.binary.resolve()
    os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    report = {'protocol': 'stack_h3_v1', 'status': 'running', 'gates': GATES,
              'columns_per_body': box.COLUMNS, 'impulse_semantics': 'normal impulse from immediately lower body (floor at level 0)',
              'half_extents_m': [0.05] * 3, 'gravity_m_s2': [0, -9.81, 0], 'friction': 0, 'restitution': 0,
              'duration_s': 2, 'platform': platform.platform(), 'cpu_affinity': sorted(os.sched_getaffinity(0)),
              'timing_scope': 'release backend batch; excludes setup, IPC, traces and ECS sync',
              'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
              'source_sha256': {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                                for name in ('stack.py', 'box.py')}, 'results': []}
    for dt, levels, mass, iterations in grid():
        case = {'dt_ticks': dt, 'levels': levels, 'top_mass_kg': mass, 'gap_m': 0.005}
        row = run_case(binary, case, {'regularization': 0, 'iterations': iterations})
        report['results'].append(row)
        print(json.dumps({'case': case, 'iterations': iterations, 'metrics': row['metrics']}), flush=True)
    report['status'] = 'complete'
    verify(report)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with lzma.open(args.output, 'xt') as f:
        json.dump(report, f, allow_nan=False)


if __name__ == '__main__':
    main()
