"""Recompute a paired H3 reference comparison from lossless raw states."""
import argparse
import hashlib
import json
import lzma
from pathlib import Path
import statistics

import stack


def verify(report):
    if report['protocol'] != 'stack_h3_paired_reference_v1' or report['gates'] != stack.GATES:
        raise ValueError('protocol differs')
    for name, digest in report['source_sha256'].items():
        if name not in ('compare_stack.py', 'stack.py', 'box.py') or hashlib.sha256(
                Path(__file__).with_name(name).read_bytes()).hexdigest() != digest:
            raise ValueError('source differs')
    if set(report['source_sha256']) != {'compare_stack.py', 'stack.py', 'box.py'}:
        raise ValueError('source missing')
    seen = set()
    for row in report['results']:
        if row['before']['case'] != row['after']['case'] or row['before']['setting'] != row['after']['setting']:
            raise ValueError('mismatched cases')
        case = row['before']['case']
        key = (case['dt_ticks'], case['levels'], case['top_mass_kg'], 200)
        if key in seen or case['gap_m'] != 0.005 or row['before']['setting'] != {'iterations': 200, 'regularization': 0}:
            raise ValueError('invalid case')
        seen.add(key)
        for name in ('before', 'after'):
            r = row[name]
            if stack.metrics(case, r['audit']['trace']) != r['metrics']:
                raise ValueError('quality differs')
            costs = r['seconds_per_simulated_second']
            if len(costs) != 5 or not all(0 < c < float('inf') for c in costs) or statistics.median(costs) != r['median_cost']:
                raise ValueError('cost differs')
            if r['audit']['debug_assertions'] or not r['exact_replay']:
                raise ValueError('release/replay missing')
            if [x[1:14] for x in r['audit']['trace'][-1]] != [x[1:14] for x in r['audit']['final_state']]:
                raise ValueError('final state differs')
        eligible = all(row[n]['metrics']['passed'] for n in ('before', 'after'))
        ratio = row['before']['median_cost'] / row['after']['median_cost'] if eligible else None
        if row['eligible'] != eligible or row['speedup'] != ratio:
            raise ValueError('case comparison differs')
    if seen != {c for c in stack.grid() if c[3] == 200}:
        raise ValueError('incomplete comparison')
    eligible = all(r['eligible'] for r in report['results'])
    ratio = (statistics.mean(r['before']['median_cost'] for r in report['results']) /
             statistics.mean(r['after']['median_cost'] for r in report['results']) if eligible else None)
    if report['eligible'] != eligible or report['ratio_of_mean_median_costs'] != ratio:
        raise ValueError('aggregate differs')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('archive', type=Path)
    args = p.parse_args()
    with lzma.open(args.archive, 'rt') as f:
        report = json.load(f)
    verify(report)
    print('Verified 12 paired comparisons (24 full-state records)')


if __name__ == '__main__':
    main()
