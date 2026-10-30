"""Paired before/after cost at the unchanged H3 200-iteration quality reference."""
import argparse
import hashlib
import json
import lzma
import os
from pathlib import Path
import platform
import statistics

import stack


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('before', type=Path)
    p.add_argument('after', type=Path)
    p.add_argument('output', type=Path)
    args = p.parse_args()
    if args.output.exists():
        p.error('provide a new output path')
    os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    binaries = {k: getattr(args, k).resolve() for k in ('before', 'after')}
    report = {'protocol': 'stack_h3_paired_reference_v1', 'gates': stack.GATES,
              'platform': platform.platform(), 'cpu_affinity': sorted(os.sched_getaffinity(0)),
              'binary_sha256': {k: hashlib.sha256(v.read_bytes()).hexdigest() for k, v in binaries.items()},
              'source_sha256': {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                                for name in ('compare_stack.py', 'stack.py', 'box.py')},
              'results': []}
    for dt, levels, mass, iterations in stack.grid():
        if iterations != 200:
            continue
        case = {'dt_ticks': dt, 'levels': levels, 'top_mass_kg': mass, 'gap_m': 0.005}
        setting = {'regularization': 0, 'iterations': iterations}
        order = ('before', 'after') if len(report['results']) % 2 == 0 else ('after', 'before')
        row = {'order': order}
        for name in order:
            row[name] = stack.run_case(binaries[name], case, setting)
        row['eligible'] = all(row[name]['metrics']['passed'] for name in binaries)
        row['speedup'] = row['before']['median_cost'] / row['after']['median_cost'] if row['eligible'] else None
        report['results'].append(row)
        print(json.dumps({'case': case, 'speedup': row['speedup']}), flush=True)
    report['eligible'] = len(report['results']) == 12 and all(r['eligible'] for r in report['results'])
    report['ratio_of_mean_median_costs'] = (
        statistics.mean(r['before']['median_cost'] for r in report['results']) /
        statistics.mean(r['after']['median_cost'] for r in report['results']) if report['eligible'] else None)
    with lzma.open(args.output, 'xt') as f:
        json.dump(report, f, allow_nan=False)
    if not report['eligible']:
        raise SystemExit('quality reference failed; retained evidence, no aggregate speedup')


if __name__ == '__main__':
    main()
