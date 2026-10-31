"""Verify lossless H3 matched-quality records and selection provenance."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import statistics
import numpy as np
import stack
import stack_matched as protocol


def verify(report, directory):
    if report['status'] != 'complete' or report['protocol'] != 'stack_matched_v1' or report['gates'] != stack.GATES:
        raise ValueError('incomplete or different protocol')
    for name in ('stack_matched.py', 'mujoco_stack.py', 'stack.py', 'box.py', 'matched.py'):
        if report['source_sha256'][name] != hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest():
            raise ValueError('source differs')
    for r in report['records']:
        expected_setting = {'iterations': 20, 'regularization': 0} if r['backend'] == 'native' else {'solref': [2e-6, 10]}
        if r['setting'] != expected_setting or r['case']['gap_m'] != 0.005:
            raise ValueError('setting differs')
        a = r['audit']
        name = a['trace_file']
        if Path(name).name != name:
            raise ValueError('invalid trace name')
        path = directory / name
        if hashlib.sha256(path.read_bytes()).hexdigest() != a['trace_sha256']:
            raise ValueError('trace hash differs')
        with gzip.open(path, 'rb') as f:
            trace = np.load(f, allow_pickle=False)
        if list(trace.shape) != a['trace_shape']:
            raise ValueError('trace shape differs')
        m = stack.metrics(r['case'], trace)
        if r['backend'] == 'mujoco':
            m['gates']['backend_health'] = not any(a['warnings']) and abs(a['simulation_time_s'] - 2) < 1e-8
            m['passed'] = all(m['gates'].values())
        elif a['debug_assertions']:
            raise ValueError('release required')
        if m != r['metrics'] or not r['exact_replay']:
            raise ValueError('metrics differ')
        if not np.array_equal(trace[-1, :, 1:14], np.asarray(a['final_state'])[:, 1:14]):
            raise ValueError('final state differs')
        costs = r['seconds_per_simulated_second']
        if len(costs) != 5 or not all(0 < x < float('inf') for x in costs) or statistics.median(costs) != r['median_cost']:
            raise ValueError('cost differs')
    selected = None
    expected = [('native', 2000000, n, m, 'calibration') for n, m in protocol.CALIBRATION]
    for dt in protocol.TIMESTEPS:
        rows = [r for r in report['records'] if r['backend'] == 'mujoco' and r['phase'] == 'calibration' and r['case']['dt_ticks'] == dt]
        expected.extend(('mujoco', dt, n, m, 'calibration') for n, m in protocol.CALIBRATION)
        if len(rows) != 2:
            raise ValueError('calibration missing')
        if all(r['metrics']['passed'] for r in rows):
            selected = dt
            break
    if selected is not None:
        expected.extend((b, 2000000 if b == 'native' else selected, n, m, 'validation')
                        for n, m in protocol.VALIDATION for b in ('native', 'mujoco'))
    actual = [(r['backend'], r['case']['dt_ticks'], r['case']['levels'], r['case']['top_mass_kg'], r['phase']) for r in report['records']]
    if sorted(actual) != sorted(expected) or selected != report['selected_dt_ticks']:
        raise ValueError('selection or cases differ')
    if report['comparison'] != protocol.comparison(report['records']):
        raise ValueError('comparison differs')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory', type=Path)
    args = p.parse_args()
    r = json.loads((args.directory / 'report.json').read_text())
    verify(r, args.directory)
    print(f"Verified {len(r['records'])} records: {r['comparison']}")


if __name__ == '__main__':
    main()
