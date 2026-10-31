"""H4: Coulomb sliding on a horizontal plane; frozen analytic quality audit."""
import argparse
import hashlib
import itertools
import json
import lzma
import os
from pathlib import Path
import statistics
import subprocess
import numpy as np
import box

GATES = {'max_penetration_m': 1e-5, 'distance_error_m': 0.001, 'stop_time_error_s': 0.004,
         'velocity_error_m_s': 0.01, 'settled_speed_m_s': 0.001,
         'settled_spin_rad_s': 0.01, 'lateral_drift_m': 1e-5, 'energy_excess_fraction': 1e-6}


def metrics(case, trace):
    x = np.asarray(trace, dtype=float)
    dt = case['dt_ticks'] * 1e-9
    count = 2_000_000_000 // case['dt_ticks'] + 1
    if x.shape != (count, 15) or not np.isfinite(x).all():
        raise ValueError('invalid trace')
    if not np.allclose(x[:, 0], np.arange(count) * dt, atol=1e-12, rtol=0):
        raise ValueError('clock differs')
    initial = np.zeros(15)
    initial[2], initial[7], initial[8] = 0.05, 1, case['speed_m_s']
    if not np.allclose(x[0], initial, atol=1e-12, rtol=0) or not np.allclose(np.linalg.norm(x[:, 4:8], axis=1), 1, atol=1e-10, rtol=0):
        raise ValueError('initial pose or quaternion differs')
    v, mu = case['speed_m_s'], float(np.float32(case['mu']))
    stop = v / (mu * 9.81)
    distance = v*v / (2*mu*9.81)
    speed = np.linalg.norm(x[:, 8:11], axis=1)
    spin = np.linalg.norm(x[:, 11:14], axis=1)
    indices = np.flatnonzero(speed > 0.001)
    stop_index = int(indices[-1] + 1) if len(indices) else 0
    actual_stop = x[stop_index, 0] if stop_index < count else None
    energy = 9.81 * x[:, 2] + 0.5*speed**2 + 0.5*0.1**2/6*spin**2
    settled = x[:, 0] >= 1.5
    r = {'max_penetration_m': max(0, float(-box.corners(x)[:, :, 1].min())),
         'distance_error_m': abs(float(x[-1, 1]) - distance),
         'stop_time_error_s': abs(float(actual_stop) - stop) if actual_stop is not None else None,
         'velocity_error_m_s': float(np.abs(x[:, 8] - np.maximum(v - mu*9.81*x[:, 0], 0)).max()),
         'settled_speed_m_s': float(speed[settled].max()), 'settled_spin_rad_s': float(spin[settled].max()),
         'lateral_drift_m': float(np.abs(x[:, 3]).max()),
         'energy_excess_fraction': max(0, float((energy.max()-energy[0])/(0.5*v*v))),
         'analytic_distance_m': distance, 'analytic_stop_time_s': stop,
         'final_distance_m': float(x[-1, 1]), 'measured_stop_time_s': float(actual_stop) if actual_stop is not None else None}
    r['gates'] = {k: r[k] is not None and r[k] <= bound for k, bound in GATES.items()}
    r['passed'] = all(r['gates'].values())
    return r


def run(binary, case, setting, audit):
    args = [str(binary), str(case['dt_ticks']), str(case['mass_kg']), str(case['speed_m_s']),
            str(setting['regularization']), '200', 'audit' if audit else 'timing', str(case['mu'])]
    return json.loads(subprocess.run(args, capture_output=True, text=True, check=True, timeout=120).stdout)


def measure(binary, case, setting):
    a = run(binary, case, setting, True)
    if a['debug_assertions'] or a['trace'] != run(binary, case, setting, True)['trace']:
        raise ValueError('release/exact replay requirement failed')
    run(binary, case, setting, False)
    costs = []
    for _ in range(5):
        b = run(binary, case, setting, False)
        if b['final_state'] != a['final_state']:
            raise ValueError('timing final state differs')
        costs.append(b['elapsed_s']/2)
    return {'case': case, 'setting': setting, 'audit': a, 'metrics': metrics(case, a['trace']),
            'exact_replay': True, 'seconds_per_simulated_second': costs, 'median_cost': statistics.median(costs)}


def grid():
    return itertools.product((1000000, 2000000), (0.3, 3), (0.2, 0.5), (0.2, 0.5), (0, 0.001))


def verify(report):
    if report['protocol'] != 'friction_slide_h4_v1' or report['gates'] != GATES or report['status'] != 'complete':
        raise ValueError('protocol differs')
    for name in ('slide.py', 'box.py'):
        if report['source_sha256'][name] != hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest():
            raise ValueError('source differs')
    seen = set()
    for r in report['results']:
        c, a = r['case'], r['audit']
        key = (c['dt_ticks'], c['mass_kg'], c['speed_m_s'], c['mu'], r['setting']['regularization'])
        if key in seen or metrics(c, a['trace']) != r['metrics']:
            raise ValueError('duplicate or changed record')
        seen.add(key)
        if a['debug_assertions'] or not r['exact_replay'] or a['trace'][-1][1:14] != a['final_state'][1:14]:
            raise ValueError('state/replay differs')
        costs = r['seconds_per_simulated_second']
        if len(costs) != 5 or not all(0 < v < float('inf') for v in costs) or statistics.median(costs) != r['median_cost']:
            raise ValueError('cost differs')
    if seen != set(grid()):
        raise ValueError('incomplete grid')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('binary', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--verify', action='store_true')
    args = p.parse_args()
    if args.verify:
        with lzma.open(args.output, 'rt') as f:
            report = json.load(f)
        verify(report)
        print('Verified 32 sliding records')
        return
    if args.output.exists():
        p.error('new output required')
    binary = args.binary.resolve()
    os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    report = {'protocol': 'friction_slide_h4_v1', 'status': 'running', 'gates': GATES,
              'cpu_affinity': sorted(os.sched_getaffinity(0)), 'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
              'source_sha256': {n: hashlib.sha256(Path(__file__).with_name(n).read_bytes()).hexdigest() for n in ('slide.py', 'box.py')}, 'results': []}
    for dt, mass, speed, mu, reg in grid():
        c = {'dt_ticks': dt, 'mass_kg': mass, 'speed_m_s': speed, 'mu': mu}
        r = measure(binary, c, {'regularization': reg, 'iterations': 200})
        report['results'].append(r)
        print(json.dumps({'case': c, 'setting': r['setting'], 'metrics': r['metrics']}), flush=True)
    report['status'] = 'complete'
    verify(report)
    with lzma.open(args.output, 'xt') as f:
        json.dump(report, f, allow_nan=False)


if __name__ == '__main__':
    main()
