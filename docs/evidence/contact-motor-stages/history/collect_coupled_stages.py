#!/usr/bin/env python3
"""Run the fixed eighteen source-bound fixture processes, without selecting outcomes."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess

REPO = Path('/workspace/RobotNativeEngine')
SOURCE_SCOPE = 'Nonexhaustive repository extension-filtered input snapshot; not compiler/dependency closure'
BUILDS = ('baseline', 'rejected_inverse_w_plus_g')
MODES = ('on', 'fresh_on', 'off')


def now():
    return datetime.now(timezone.utc).isoformat()


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def artifact(path):
    path = Path(path).resolve()
    return {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size}


def validate_artifact(record):
    if set(record) != {'path', 'sha256', 'bytes'} or artifact(record['path']) != record:
        raise ValueError('artifact binding mismatch')


def snapshot():
    names = subprocess.check_output(
        ['git', 'ls-files', '--cached', '--others', '--exclude-standard', '-z'], cwd=REPO
    ).decode().split('\0')
    selected = sorted(set(name for name in names if name and 'target' not in Path(name).parts
                          and Path(name).suffix in {'.rs', '.toml', '.lock', '.py', '.json', '.yaml', '.yml', '.urdf', '.xml'}))
    return {'scope': SOURCE_SCOPE, 'files': {name: sha(REPO / name) for name in selected
            if (REPO / name).is_file()}}


def write_new(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')


def save_index(path, value):
    temporary = path.with_suffix('.temporary')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser()
    for name in ('protocol', 'protocol-sha256', 'baseline-binding', 'candidate-binding',
                 'restored-source-manifest', 'index-template', 'output', 'traces-directory'):
        parser.add_argument('--' + name, required=True)
    args = parser.parse_args()
    if sha(args.protocol) != args.protocol_sha256:
        raise ValueError('protocol changed before collection')
    output = Path(args.output).resolve()
    index = json.loads(Path(args.index_template).read_text())
    cells = [(build, mass, mode, f'{build}-m{mass}-{mode}') for build in BUILDS
             for mass in (5, 20, 50) for mode in MODES]
    if [(r['build'], r['mass_kg'], r['mode'], r['run_label']) for r in index['runs']] != cells:
        raise ValueError('fixed eighteen-cell matrix changed')
    bindings = {}
    for build, path in zip(BUILDS, (args.baseline_binding, args.candidate_binding)):
        binding = json.loads(Path(path).read_text())
        if (binding['build'] != build or binding['protocol_sha256'] != args.protocol_sha256
                or binding['authoritative_worlds_of_this_build_executed_before_freeze'] != 0):
            raise ValueError('unfrozen or wrong source/binary binding')
        validate_artifact(binding['binary'])
        validate_artifact(binding['source_manifest'])
        bindings[build] = binding
        index['source_binary_bindings'][build] = artifact(path)
    expected = json.loads(Path(args.restored_source_manifest).read_text())
    if snapshot() != expected:
        raise ValueError('restored source inputs changed before collection')
    traces = Path(args.traces_directory).resolve()
    traces.mkdir(parents=True, exist_ok=False)
    index.update(protocol_sha256=args.protocol_sha256, root_driver=artifact(__file__),
                 started_utc=now(), status='ACTUAL_COUPLED_COLLECTION_RUNNING')
    write_new(output, index)
    try:
        for row in index['runs']:
            label = row['run_label']
            before_path = traces / (label + '.source-before.json')
            before = snapshot()
            write_new(before_path, before)
            row['source_guard_before'] = artifact(before_path)
            if before != expected:
                raise ValueError('source changed before ' + label)
            binary = bindings[row['build']]['binary']
            validate_artifact(binary)
            trace = traces / (label + '.jsonl.gz')
            stdout = traces / (label + '.stdout')
            stderr = traces / (label + '.stderr')
            command = [binary['path'], '--mass-kg', str(row['mass_kg']), '--observer',
                       'off' if row['mode'] == 'off' else 'on', '--label', label,
                       '--output', str(trace)]
            row.update(attempted=True, started_utc=now(), argv=command)
            index['worlds_attempted'] += 1
            save_index(output, index)
            with stdout.open('xb') as out, stderr.open('xb') as err:
                result = subprocess.run(command, cwd=REPO, stdout=out, stderr=err)
            row.update(exit_code=result.returncode, ended_utc=now(),
                       stdout=artifact(stdout), stderr=artifact(stderr),
                       trace=artifact(trace) if trace.exists() else None,
                       error=None if result.returncode == 0 else 'fixture process failed')
            after_path = traces / (label + '.source-after.json')
            after = snapshot()
            write_new(after_path, after)
            row['source_guard_after'] = artifact(after_path)
            if after != before:
                raise ValueError('source changed during ' + label)
            if result.returncode == 0:
                index['worlds_completed'] += 1
            save_index(output, index)
            print(json.dumps({'label': label, 'exit_code': result.returncode,
                              'trace_bytes': row['trace']['bytes'] if row['trace'] else None}), flush=True)
        if index['worlds_completed'] != 18:
            raise ValueError('one or more fixture processes failed')
        index['status'] = 'ACTUAL_COUPLED_18_WORLD_COLLECTION_COMPLETE'
    except Exception as error:
        index.update(status='ACTUAL_COUPLED_COLLECTION_FAILED', error=str(error))
    index['ended_utc'] = now()
    save_index(output, index)
    return 0 if index['error'] is None and index['worlds_completed'] == 18 else 1


if __name__ == '__main__':
    raise SystemExit(main())
