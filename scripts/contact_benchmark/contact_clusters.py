"""Compare sweep/exhaustive enumeration with actual dynamic support contacts."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import contact_crowd as base

CASES = [(n, layout) for n in [64, 128] for layout in ['stacks', 'clusters']]


def run(binary, count, layout, mode):
    return json.loads(subprocess.run([str(binary.resolve()), str(count), mode, layout],
        check=True, capture_output=True, text=True, timeout=120).stdout)


def expected_support_pairs(count, layout):
    if layout == 'stacks':
        return [[i+1, i+2] for i in range(0, count, 2)]
    if layout == 'clusters':
        return sorted([[i+j+1, i+j+3] for i in range(0, count, 4) for j in range(2)])
    raise ValueError('unknown contact layout')


def contact_metrics(audit, count, layout):
    frames = audit['contact_frames']
    if audit['layout'] != layout or len(frames) != 100:
        raise ValueError('layout/contact trace differs')
    expected = expected_support_pairs(count, layout)
    if frames[-1]['loaded_dynamic_pairs'] != expected:
        raise ValueError('final support pairs differ')
    for f in frames:
        pairs = f['loaded_dynamic_pairs']
        if (pairs != sorted(pairs) or len({tuple(p) for p in pairs}) != len(pairs)
                or any(p not in expected for p in pairs)
                or not 0 <= f['pre_step_dynamic_penetration_m'] < 1e-8
                or not f['all_solver_stopping_flags']
                or not 0 <= f['max_solver_iterations'] < 200
                or not f['generated'] >= f['retained'] >= len(pairs)):
            raise ValueError('dynamic contact/solver check failed')
    return dict(final_loaded_dynamic_pairs=len(expected),
        max_pre_step_dynamic_penetration_m=max(f['pre_step_dynamic_penetration_m'] for f in frames),
        max_solver_iterations=max(f['max_solver_iterations'] for f in frames),
        max_generated=max(f['generated'] for f in frames),
        max_retained=max(f['retained'] for f in frames))


def summary(row):
    cost = base.summary(row)
    for name in ['before', 'after']:
        contact = contact_metrics(row[name], row['count'], row['layout'])
        for s in row['timing'][name]:
            if s['layout'] != row['layout'] or s['contact_frames']:
                raise ValueError('timing contains wrong layout/diagnostics')
    return dict(cost=cost, contacts=contact)


def verify(report, before, after):
    if (report['protocol'] != 'contact_clusters_v1'
            or [(r['count'], r['layout']) for r in report['results']] != CASES
            or report['binary_sha256'] != dict(before=base.digest(before), after=base.digest(after))
            or report['source_sha256'] != {n: base.digest(Path(__file__).with_name(n)) for n in ['contact_clusters.py', 'contact_crowd.py']}):
        raise ValueError('provenance/cases differ')
    for row in report['results']:
        if row['summary'] != summary(row): raise ValueError('summary differs')
        for name, binary in [('before', before), ('after', after)]:
            base.check_audits(row[name], run(binary, row['count'], row['layout'], 'audit'), row['count'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('before', type=Path); parser.add_argument('after', type=Path)
    parser.add_argument('output', type=Path); parser.add_argument('--verify', action='store_true')
    args = parser.parse_args()
    if args.verify:
        verify(json.loads(args.output.read_text()), args.before, args.after)
        print('Verified loaded support pairs, state hashes and fresh replays'); return
    if args.output.exists(): parser.error('new output required')
    os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    report = dict(protocol='contact_clusters_v1', cpu_affinity=sorted(os.sched_getaffinity(0)),
        binary_sha256=dict(before=base.digest(args.before), after=base.digest(args.after)),
        source_sha256={n: base.digest(Path(__file__).with_name(n)) for n in ['contact_clusters.py', 'contact_crowd.py']}, results=[])
    for count, layout in CASES:
        row = dict(count=count, layout=layout, before=run(args.before,count,layout,'audit'),
            after=run(args.after,count,layout,'audit'), timing=dict(before=[],after=[]))
        base.check_audits(row['before'],row['after'],count)
        contact_metrics(row['after'],count,layout)
        for binary in [args.before,args.after]: run(binary,count,layout,'timing')
        for iteration in range(5):
            order=[('before',args.before),('after',args.after)]
            if iteration%2: order.reverse()
            for name,binary in order: row['timing'][name].append(run(binary,count,layout,'timing'))
        row['summary']=summary(row);report['results'].append(row)
        print(json.dumps(dict(count=count,layout=layout,summary=row['summary'])),flush=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    verify(report,args.before,args.after)


if __name__ == '__main__':
    main()
