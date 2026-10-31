"""Frozen H3 finer-step calibration followed by independent mass validation."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import statistics

import mujoco
import matched
import mujoco_stack
import stack

TIMESTEPS = (125000, 62500, 31250, 15625)
CALIBRATION = ((3, 1), (3, 10))
VALIDATION = ((2, 0.3), (2, 3), (3, 0.3), (3, 3))


def comparison(rows):
    a = [r for r in rows if r['phase'] == 'validation' and r['backend'] == 'native']
    b = [r for r in rows if r['phase'] == 'validation' and r['backend'] == 'mujoco']
    expected = set(VALIDATION)
    if any({(r['case']['levels'], r['case']['top_mass_kg']) for r in group} != expected
           or len(group) != len(expected) for group in (a, b)):
        return {'eligible': False, 'reason': 'missing validation'}
    if not all(r['metrics']['passed'] for r in a + b):
        return {'eligible': False, 'reason': 'validation quality failure'}
    return {'eligible': True, 'ratio_mujoco_over_native':
            statistics.mean(r['median_cost'] for r in b) / statistics.mean(r['median_cost'] for r in a)}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('binary', type=Path)
    p.add_argument('directory', type=Path)
    args = p.parse_args()
    if args.directory.exists() or mujoco.__version__ != '3.9.0':
        p.error('new directory and MuJoCo 3.9.0 required')
    args.directory.mkdir(parents=True)
    os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    report = {'protocol': 'stack_matched_v1', 'status': 'running', 'gates': stack.GATES,
              'mujoco': mujoco.__version__, 'cpu_affinity': sorted(os.sched_getaffinity(0)),
              'timesteps': TIMESTEPS, 'calibration': CALIBRATION, 'validation': VALIDATION,
              'selection': 'first timestep where both calibration cases pass; fixed solref [2e-6,10]',
              'binary_sha256': hashlib.sha256(args.binary.read_bytes()).hexdigest(),
              'source_sha256': {n: hashlib.sha256(Path(__file__).with_name(n).read_bytes()).hexdigest()
                                for n in ('stack_matched.py', 'mujoco_stack.py', 'stack.py', 'box.py', 'matched.py')},
              'records': [], 'selected_dt_ticks': None}
    def checkpoint():
        (args.directory / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    def measure(backend, dt, cases, phase):
        rows = []
        for levels, mass in cases:
            case = {'dt_ticks': dt, 'levels': levels, 'top_mass_kg': mass, 'gap_m': 0.005}
            r = (stack.run_case(args.binary.resolve(), case, {'iterations': 20, 'regularization': 0})
                 if backend == 'native' else mujoco_stack.run_case(case, {'solref': [2e-6, 10]}))
            r.update(backend=backend, phase=phase)
            matched.save_trace(args.directory, r, len(report['records']))
            report['records'].append(r)
            rows.append(r)
            checkpoint()
            print(json.dumps({'backend': backend, 'case': case, 'phase': phase, 'metrics': r['metrics']}), flush=True)
        return rows
    checkpoint()
    measure('native', 2000000, CALIBRATION, 'calibration')
    for dt in TIMESTEPS:
        rows = measure('mujoco', dt, CALIBRATION, 'calibration')
        if all(r['metrics']['passed'] for r in rows):
            report['selected_dt_ticks'] = dt
            checkpoint()
            break
    if report['selected_dt_ticks'] is not None:
        for i, case in enumerate(VALIDATION):
            for backend in (('native', 'mujoco') if i % 2 == 0 else ('mujoco', 'native')):
                measure(backend, 2000000 if backend == 'native' else report['selected_dt_ticks'], [case], 'validation')
    report['comparison'] = comparison(report['records'])
    report['status'] = 'complete'
    checkpoint()


if __name__ == '__main__':
    main()
