"""Frozen H3 impact-phase robustness check at the previously selected settings."""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import statistics

import mujoco
import numpy as np
import matched
import mujoco_stack
import stack

PHASES = (0.0, 0.2, 0.4, 0.6)


def gap(phase):
    # Free-fall impact time spans one native 2 ms step near the old 5 mm gap.
    return 0.5 * 9.81 * ((16 + phase) * 0.002)**2


def comparison(rows):
    expected = {(p, b) for p in PHASES for b in ('native', 'mujoco')}
    if len(rows) != 8 or {(r['phase'], r['backend']) for r in rows} != expected:
        return {'eligible': False, 'reason': 'missing or duplicate phase'}
    if not all(r['metrics']['passed'] for r in rows):
        return {'eligible': False, 'reason': 'phase quality failure'}
    cost = {b: statistics.mean(r['median_cost'] for r in rows if r['backend'] == b)
            for b in ('native', 'mujoco')}
    return {'eligible': True, 'ratio_mujoco_over_native': cost['mujoco'] / cost['native']}


def verify(report, directory):
    if report['status'] != 'complete' or report['protocol'] != 'stack_phase_v1' or report['gates'] != stack.GATES:
        raise ValueError('protocol differs')
    for name in ('stack_phase.py', 'stack.py', 'box.py', 'mujoco_stack.py', 'matched.py'):
        if report['source_sha256'][name] != hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest():
            raise ValueError('source differs')
    seen = set()
    for r in report['records']:
        b, phase = r['backend'], r['phase']
        key = (phase, b)
        if key in seen or b not in ('native', 'mujoco') or phase not in PHASES:
            raise ValueError('invalid phase')
        seen.add(key)
        case = {'dt_ticks': 2000000 if b == 'native' else 15625, 'levels': 3,
                'top_mass_kg': 3, 'gap_m': gap(phase)}
        setting = {'iterations': 20, 'regularization': 0} if b == 'native' else {'solref': [2e-6, 10]}
        if r['case'] != case or r['setting'] != setting:
            raise ValueError('frozen case differs')
        a = r['audit']
        if Path(a['trace_file']).name != a['trace_file']:
            raise ValueError('invalid filename')
        p = directory / a['trace_file']
        if hashlib.sha256(p.read_bytes()).hexdigest() != a['trace_sha256']:
            raise ValueError('trace hash differs')
        with gzip.open(p, 'rb') as f:
            x = np.load(f, allow_pickle=False)
        if list(x.shape) != a['trace_shape'] or not np.array_equal(x[-1, :, 1:14], np.asarray(a['final_state'])[:, 1:14]):
            raise ValueError('state differs')
        m = stack.metrics(case, x)
        if b == 'mujoco':
            m['gates']['backend_health'] = not any(a['warnings']) and abs(a['simulation_time_s'] - 2) < 1e-8
            m['passed'] = all(m['gates'].values())
        elif a['debug_assertions']:
            raise ValueError('release required')
        costs = r['seconds_per_simulated_second']
        if m != r['metrics'] or not r['exact_replay'] or len(costs) != 5 or not all(0 < c < float('inf') for c in costs) or statistics.median(costs) != r['median_cost']:
            raise ValueError('metrics or cost differs')
    if len(seen) != 8 or report['comparison'] != comparison(report['records']):
        raise ValueError('incomplete or different comparison')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('binary', type=Path)
    p.add_argument('directory', type=Path)
    p.add_argument('--verify', action='store_true')
    args = p.parse_args()
    if args.verify:
        report = json.loads((args.directory / 'report.json').read_text())
        verify(report, args.directory)
        print(report['comparison'])
        return
    if args.directory.exists() or mujoco.__version__ != '3.9.0':
        p.error('new directory and MuJoCo 3.9.0 required')
    args.directory.mkdir(parents=True)
    os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    report = {'protocol': 'stack_phase_v1', 'status': 'running', 'gates': stack.GATES,
              'phases': PHASES, 'mujoco': mujoco.__version__, 'cpu_affinity': sorted(os.sched_getaffinity(0)),
              'binary_sha256': hashlib.sha256(args.binary.read_bytes()).hexdigest(),
              'source_sha256': {n: hashlib.sha256(Path(__file__).with_name(n).read_bytes()).hexdigest()
                                for n in ('stack_phase.py', 'stack.py', 'box.py', 'mujoco_stack.py', 'matched.py')},
              'records': []}
    def checkpoint():
        (args.directory / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    checkpoint()
    for i, phase in enumerate(PHASES):
        for b in (('native', 'mujoco') if i % 2 == 0 else ('mujoco', 'native')):
            case = {'dt_ticks': 2000000 if b == 'native' else 15625, 'levels': 3,
                    'top_mass_kg': 3, 'gap_m': gap(phase)}
            r = (stack.run_case(args.binary.resolve(), case, {'iterations': 20, 'regularization': 0})
                 if b == 'native' else mujoco_stack.run_case(case, {'solref': [2e-6, 10]}))
            r.update(backend=b, phase=phase)
            matched.save_trace(args.directory, r, len(report['records']))
            report['records'].append(r)
            checkpoint()
            print(json.dumps({'backend': b, 'phase': phase, 'metrics': r['metrics']}), flush=True)
    report['comparison'] = comparison(report['records'])
    report['status'] = 'complete'
    checkpoint()


if __name__ == '__main__':
    main()
