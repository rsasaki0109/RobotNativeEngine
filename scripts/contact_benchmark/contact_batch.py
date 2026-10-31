"""Paired timings of 200 fresh one-box landings per process; setup excluded."""
import argparse
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import contact_crowd as base

REPEATS = 200
SAMPLES = 9


def run(binary):
    return json.loads(subprocess.run(
        [str(binary.resolve()), '1', 'timing', 'separated', str(REPEATS)],
        check=True, capture_output=True, text=True, timeout=120).stdout)


def cost(sample, expected_hash):
    times = sample['repeat_elapsed_s']
    if (sample['count'] != 1 or sample['mode'] != 'timing'
            or sample['layout'] != 'separated' or sample['debug_assertions']
            or sample['steps'] != 100 or sample['dt_ticks'] != 2000000
            or sample['margin_m'] != .02 or sample['hashes'] or sample['contact_frames']
            or sample['max_penetration_m'] is not None or sample['max_spin_rad_s'] is not None
            or sample['repeats'] != REPEATS or len(times) != REPEATS
            or sample['repeat_final_hashes'] != [expected_hash] * REPEATS
            or sample['final_hash'] != expected_hash
            or not all(math.isfinite(t) and t > 0 for t in times)
            or not math.isfinite(sample['elapsed_s'])
            or not math.isclose(sum(times), sample['elapsed_s'], rel_tol=1e-12, abs_tol=0)):
        raise ValueError('batch fixture, repeat hashes or time accounting differs')
    return sample['elapsed_s'] / (REPEATS * .2)


def summary(report):
    base.check_audits(report['audit']['before'], report['audit']['after'], 1)
    costs = {}
    for name in ['before', 'after']:
        rows = report['timing'][name]
        if len(rows) != SAMPLES:
            raise ValueError('nine paired samples required')
        costs[name] = [cost(s, report['audit'][name]['final_hash']) for s in rows]
    med = {k: statistics.median(v) for k, v in costs.items()}
    return dict(seconds_per_simulated_second=med,
                reduction_fraction=1-med['after']/med['before'],
                paired_reduction_fractions=[1-a/b for b, a in zip(costs['before'], costs['after'])])


def verify(report, binaries):
    if (report['protocol'] != 'contact_batch_v1'
            or report['first'] not in ['before', 'after']
            or report['binary_sha256'] != {k: base.digest(v) for k, v in binaries.items()}
            or report['source_sha256'] != {n: base.digest(Path(__file__).with_name(n))
                                          for n in ['contact_batch.py', 'contact_crowd.py']}
            or summary(report) != report['summary']):
        raise ValueError('batch provenance/summary differs')
    for name, binary in binaries.items():
        base.check_audits(report['audit'][name], base.run(binary, 1, 'audit'), 1)
        cost(run(binary), report['audit'][name]['final_hash'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('before', type=Path)
    parser.add_argument('after', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--first', choices=['before', 'after'], default='before')
    parser.add_argument('--verify', action='store_true')
    args = parser.parse_args()
    binaries = dict(before=args.before, after=args.after)
    if args.verify:
        verify(json.loads(args.output.read_text()), binaries)
        print('Verified all repeat hashes, costs, provenance and fresh batches')
        return
    if args.output.exists():
        parser.error('new output required')
    os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    report = dict(protocol='contact_batch_v1', first=args.first,
                  cpu_affinity=sorted(os.sched_getaffinity(0)),
                  binary_sha256={k: base.digest(v) for k, v in binaries.items()},
                  source_sha256={n: base.digest(Path(__file__).with_name(n))
                                 for n in ['contact_batch.py', 'contact_crowd.py']},
                  audit={k: base.run(b, 1, 'audit') for k, b in binaries.items()},
                  timing=dict(before=[], after=[]))
    for name, binary in binaries.items():
        cost(run(binary), report['audit'][name]['final_hash'])
    for i in range(SAMPLES):
        order = ['before', 'after']
        if (i + (args.first == 'after')) % 2:
            order.reverse()
        for name in order:
            report['timing'][name].append(run(binaries[name]))
    report['summary'] = summary(report)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    verify(report, binaries)
    print(json.dumps(report['summary']))


if __name__ == '__main__':
    main()
