"""Paired Native box-crowd benchmark with unchanged 20 mm contact margin."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess

COUNTS = [1, 16, 64, 128]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(binary, count, mode):
    return json.loads(subprocess.run([str(binary.resolve()), str(count), mode],
        check=True, capture_output=True, text=True, timeout=120).stdout)


def check_audits(before, after, count):
    for audit in [before, after]:
        if (audit['count'] != count or audit['mode'] != 'audit' or audit['debug_assertions']
                or audit['steps'] != 100 or audit['dt_ticks'] != 2000000
                or audit['margin_m'] != .02 or len(audit['hashes']) != 100
                or audit['hashes'][-1] != audit['final_hash']
                or not 0 <= audit['max_penetration_m'] < 1e-8
                or not 0 <= audit['max_spin_rad_s'] < .01):
            raise ValueError('fixture/quality differs')
    if any(before[key] != after[key] for key in before if key != 'elapsed_s'):
        raise ValueError('state replay differs')


def summary(row):
    check_audits(row['before'], row['after'], row['count'])
    costs = []
    for name in ['before', 'after']:
        samples = row['timing'][name]
        if len(samples) != 5: raise ValueError('five samples required')
        for s in samples:
            if (s['final_hash'] != row[name]['final_hash'] or s['count'] != row['count']
                    or s['mode'] != 'timing' or s['hashes'] or s['debug_assertions']
                    or s['steps'] != 100 or s['dt_ticks'] != 2000000 or s['margin_m'] != .02
                    or not 0 < s['elapsed_s'] < float('inf')):
                raise ValueError('timed fixture/replay differs')
        costs.append(statistics.median(s['elapsed_s']/.2 for s in samples))
    return dict(before_seconds_per_simulated_second=costs[0],
                after_seconds_per_simulated_second=costs[1], speedup=costs[0]/costs[1],
                reduction_fraction=1-costs[1]/costs[0])


def verify(report, before, after):
    if (report['protocol'] != 'contact_crowd_v1'
            or [r['count'] for r in report['results']] != COUNTS
            or report['binary_sha256'] != dict(before=digest(before), after=digest(after))
            or report['script_sha256'] != digest(Path(__file__))):
        raise ValueError('provenance differs')
    for row in report['results']:
        if summary(row) != row['summary']: raise ValueError('summary differs')
        for name, binary in [('before', before), ('after', after)]:
            check_audits(row[name], run(binary, row['count'], 'audit'), row['count'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('before', type=Path); parser.add_argument('after', type=Path)
    parser.add_argument('output', type=Path); parser.add_argument('--verify', action='store_true')
    args = parser.parse_args()
    if args.verify:
        verify(json.loads(args.output.read_text()), args.before, args.after)
        print('Verified crowd hashes, quality, costs and fresh audit replays'); return
    if args.output.exists(): parser.error('new output required')
    os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    report = dict(protocol='contact_crowd_v1', cpu_affinity=sorted(os.sched_getaffinity(0)),
        binary_sha256=dict(before=digest(args.before), after=digest(args.after)),
        script_sha256=digest(Path(__file__)), results=[])
    for count in COUNTS:
        row = dict(count=count, before=run(args.before, count, 'audit'),
            after=run(args.after, count, 'audit'), timing=dict(before=[], after=[]))
        check_audits(row['before'], row['after'], count)
        for binary in [args.before, args.after]: run(binary, count, 'timing')
        for iteration in range(5):
            order = [('before', args.before), ('after', args.after)]
            if iteration % 2: order.reverse()
            for name, binary in order: row['timing'][name].append(run(binary, count, 'timing'))
        row['summary'] = summary(row)
        report['results'].append(row)
        print(json.dumps(dict(count=count, summary=row['summary'])), flush=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    verify(report, args.before, args.after)


if __name__ == '__main__':
    main()
