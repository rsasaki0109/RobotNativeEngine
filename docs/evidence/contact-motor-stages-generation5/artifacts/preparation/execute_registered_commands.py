"""Retain actual subprocess exits and immutable before/after file descriptors."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess


def stamp():
    return datetime.now(timezone.utc).isoformat()


def record(path):
    path = Path(path).resolve()
    with path.open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    return dict(path=str(path), bytes=path.stat().st_size, sha256=digest)


def write_new(path, data):
    with path.open('x') as stream:
        json.dump(data, stream, sort_keys=True, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--inputs', required=True)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    output = Path(args.output).resolve()
    if not str(output).startswith('/workspace/'):
        raise ValueError('requires durable workspace output')
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        raise ValueError('missing literal command')
    descriptors = json.loads(Path(args.inputs).read_text())
    for item in descriptors:
        if record(item['path']) != item:
            raise ValueError('prelaunch immutable input changed: ' + item['path'])
    output.mkdir(exist_ok=False)
    claim = dict(argv=command, started_utc=stamp(), inputs_before=descriptors, wrapper=record(__file__), rerun_or_retry=False)
    write_new(output / 'actual-command-claim.json', claim)
    with (output / 'stdout.log').open('xb') as stdout, (output / 'stderr.log').open('xb') as stderr:
        process = subprocess.Popen(command, cwd='/workspace/RobotNativeEngine', stdout=stdout, stderr=stderr)
        print(json.dumps({'pid': process.pid, 'argv': command, 'status': 'ORIGINAL_PROCESS_RUNNING'}), flush=True)
        while True:
            try:
                code = process.wait(timeout=30)
                break
            except subprocess.TimeoutExpired:
                print(json.dumps({'pid': process.pid, 'observed_utc': stamp(), 'status': 'ORIGINAL_PROCESS_RUNNING'}), flush=True)
        for stream in (stdout, stderr):
            stream.flush()
            os.fsync(stream.fileno())
    after = [record(item['path']) for item in descriptors]
    outcome = dict(claim, ended_utc=stamp(), actual_exit_code=code, inputs_after=after,
                   guarded_inputs_unchanged=after == descriptors, stdout=record(output / 'stdout.log'), stderr=record(output / 'stderr.log'))
    write_new(output / 'actual-command-terminal.json', outcome)
    print(json.dumps({'actual_exit_code': code, 'inputs_unchanged': after == descriptors, 'terminal': record(output / 'actual-command-terminal.json')}), flush=True)
    return code if code else (0 if after == descriptors else 2)


if __name__ == '__main__':
    raise SystemExit(main())
