"""Validate the complete calibration grid and recompute geometric quality."""
import argparse
import hashlib
import itertools
import json
import lzma
from pathlib import Path
import statistics
import stack


def verify(report):
    if report['protocol'] != 'mujoco_stack_calibration_v1' or report['status'] != 'complete' or report['gates'] != stack.GATES:
        raise ValueError('protocol differs')
    for name in ('mujoco_stack.py', 'stack.py', 'box.py'):
        if report['source_sha256'][name] != hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest():
            raise ValueError('evaluator differs')
    seen = set()
    for r in report['results']:
        c = r['case']
        key = (c['dt_ticks'], c['levels'], c['top_mass_kg'], tuple(r['setting']['solref']))
        if key in seen or c['gap_m'] != 0.005:
            raise ValueError('duplicate or changed case')
        seen.add(key)
        audit = r['audit']
        m = stack.metrics(c, audit['trace'])
        m['gates']['backend_health'] = not any(audit['warnings']) and abs(audit['simulation_time_s'] - 2) < 1e-8
        m['passed'] = all(m['gates'].values())
        if m != r['metrics'] or not r['exact_replay']:
            raise ValueError('metrics or replay differs')
        if [x[1:14] for x in audit['trace'][-1]] != [x[1:14] for x in audit['final_state']]:
            raise ValueError('final state differs')
        costs = r['seconds_per_simulated_second']
        if len(costs) != 5 or not all(0 < x < float('inf') for x in costs) or statistics.median(costs) != r['median_cost']:
            raise ValueError('cost differs')
    expected = set(itertools.product((2000000, 1000000, 250000), (2, 3), (1, 10), ((0.02, 1), (2e-6, 10))))
    if seen != expected or report['comparison_eligible']:
        raise ValueError('incomplete calibration or unsupported comparison')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('archive', type=Path)
    args = p.parse_args()
    with lzma.open(args.archive, 'rt') as f:
        r = json.load(f)
    verify(r)
    print('Verified 24 MuJoCo calibration records; no matched-quality speed claim')


if __name__ == '__main__':
    main()
