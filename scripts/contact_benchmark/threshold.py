"""H5: constant COM-load threshold via equivalent horizontal body acceleration."""
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

GATES = {'max_penetration_m': 1e-5, 'position_error_m': 0.003,
         'velocity_error_m_s': 0.002, 'static_drift_m': 1e-5, 'static_speed_m_s': 1e-4,
         'spin_rad_s': 0.01, 'lateral_drift_m': 1e-5, 'work_energy_excess_fraction': 1e-6}


def metrics(case, trace):
    x = np.asarray(trace, dtype=float)
    dt = case['dt_ticks'] * 1e-9
    count = 2_000_000_000 // case['dt_ticks'] + 1
    if x.shape != (count, 15) or not np.isfinite(x).all():
        raise ValueError('invalid trace')
    if not np.allclose(x[:, 0], np.arange(count) * dt, atol=1e-12, rtol=0):
        raise ValueError('clock differs')
    initial = np.zeros(15)
    initial[2], initial[7] = 0.05, 1
    if not np.allclose(x[0], initial, atol=1e-12, rtol=0) or not np.allclose(np.linalg.norm(x[:, 4:8], axis=1), 1, atol=1e-10, rtol=0):
        raise ValueError('initial state or quaternion differs')
    ratio, mu = case['force_ratio'], float(np.float32(case['mu']))
    applied_acceleration = ratio * mu * 9.81
    acceleration = np.sign(ratio) * max(abs(ratio)-1, 0) * mu * 9.81
    speed = np.linalg.norm(x[:, 8:11], axis=1)
    spin = np.linalg.norm(x[:, 11:14], axis=1)
    energy = 9.81*x[:, 2] + 0.5*speed**2 + 0.5*0.1**2/6*spin**2
    residual = energy - energy[0] - applied_acceleration*x[:, 1]
    static = abs(ratio) < 1
    r = {'max_penetration_m': max(0, float(-box.corners(x)[:, :, 1].min())),
         'position_error_m': float(np.abs(x[:, 1] - 0.5*acceleration*x[:, 0]**2).max()),
         'velocity_error_m_s': float(np.abs(x[:, 8] - acceleration*x[:, 0]).max()),
         'static_drift_m': float(np.abs(x[:, 1]).max()) if static else 0,
         'static_speed_m_s': float(speed.max()) if static else 0,
         'spin_rad_s': float(spin.max()), 'lateral_drift_m': float(np.abs(x[:, 3]).max()),
         'work_energy_excess_fraction': max(0, float(residual.max()/energy[0])),
         'analytic_acceleration_m_s2': float(acceleration),
         'equivalent_force_n': case['mass_kg']*applied_acceleration,
         'final_displacement_m': float(x[-1, 1]), 'final_velocity_m_s': float(x[-1, 8])}
    r['gates'] = {k: r[k] <= bound for k, bound in GATES.items()}
    r['passed'] = all(r['gates'].values())
    return r


def run(binary, case, setting, audit):
    args = [str(binary), str(case['dt_ticks']), str(case['mass_kg']), str(case['force_ratio']),
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
    return itertools.product((2000000,), (0.3, 3), (-1.2, -0.8, 0.8, 1.2), (0.2, 0.5), (0, 0.001))


def verify(report):
    if report['protocol'] != 'friction_threshold_h5_v1' or report['gates'] != GATES or report['status'] != 'complete':
        raise ValueError('protocol differs')
    for name in ('threshold.py', 'box.py'):
        if report['source_sha256'][name] != hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest():
            raise ValueError('source differs')
    seen = set()
    for r in report['results']:
        c, a = r['case'], r['audit']
        key = (c['dt_ticks'], c['mass_kg'], c['force_ratio'], c['mu'], r['setting']['regularization'])
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
        print('Verified 32 friction-threshold records')
        return
    if args.output.exists():
        p.error('new output required')
    binary = args.binary.resolve()
    os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    report = {'protocol': 'friction_threshold_h5_v1', 'status': 'running', 'gates': GATES,
              'cpu_affinity': sorted(os.sched_getaffinity(0)), 'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
              'source_sha256': {n: hashlib.sha256(Path(__file__).with_name(n).read_bytes()).hexdigest() for n in ('threshold.py', 'box.py')}, 'results': []}
    for dt, mass, speed, mu, reg in grid():
        c = {'dt_ticks': dt, 'mass_kg': mass, 'force_ratio': speed, 'mu': mu}
        r = measure(binary, c, {'regularization': reg, 'iterations': 200})
        report['results'].append(r)
        print(json.dumps({'case': c, 'setting': r['setting'], 'metrics': r['metrics']}), flush=True)
    report['status'] = 'complete'
    verify(report)
    with lzma.open(args.output, 'xt') as f:
        json.dump(report, f, allow_nan=False)


if __name__ == '__main__':
    main()
