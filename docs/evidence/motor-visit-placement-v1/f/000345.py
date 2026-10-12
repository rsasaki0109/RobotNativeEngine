#!/usr/bin/python3
"""Retain the original collection/audit CLI exit behind an exclusive root claim."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

REC = Path('/workspace/.rne-onboarding/recovery-20261012')
STUDY = REC / 'next-motor-visit-placement-18'

def checked(d):
    p = Path(d['path'])
    assert p.stat().st_size == d['bytes'] and hashlib.sha256(p.read_bytes()).hexdigest() == d['sha256'], p

def descriptor(p):
    return dict(path=str(p), sha256=hashlib.sha256(p.read_bytes()).hexdigest(), bytes=p.stat().st_size)

def save(p, value):
    with p.open('x') as f:
        json.dump(value, f, indent=2)
        f.write('\n')
        f.flush()
        os.fsync(f.fileno())
    fd = os.open(p.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)

def main():
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt('root invocation interrupted')
    signal.signal(signal.SIGTERM, interrupted)
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['collect', 'audit'])
    parser.add_argument('--spec', required=True)
    parser.add_argument('--spec-sha256', required=True)
    args = parser.parse_args()
    wrapper_path = Path(args.spec)
    assert hashlib.sha256(wrapper_path.read_bytes()).hexdigest() == args.spec_sha256
    wrapper = json.loads(wrapper_path.read_text())
    checked(wrapper['candidate']); checked(wrapper['admission'])
    spec = json.loads(Path(wrapper['candidate']['path']).read_text())
    admission = json.loads(Path(wrapper['admission']['path']).read_text())
    assert admission['status'] == 'PASS_PRE_DATA_SOURCE_PROTOCOL_AND_DRIVER'
    assert admission['reviewed_candidate_sha256'] == wrapper['candidate']['sha256']
    for item in spec['frozen_inputs']:
        checked(item)
    checked(spec['binary']); checked(spec['driver_interpreter'])
    if args.mode == 'collect':
        command = ['/usr/bin/python3', '-B', spec['checked_driver_source']['path'], '--spec', args.spec, '--spec-sha256', args.spec_sha256, '--launch-native']
    else:
        assert (STUDY / 'collect-invocation/actual-command.json').is_file()
        actual = json.loads((STUDY / 'collect-invocation/actual-command.json').read_text())
        assert actual['original_exit'] == 0 and actual['source_inputs_unchanged'] is True and actual['error'] is None
        assert actual['mode'] == 'collect' and actual['registered_wrapper_sha256'] == args.spec_sha256
        readers = [item for item in spec['frozen_inputs'] if Path(item['path']).name == 'cohort_reader.py']
        assert len(readers) == 1
        outer = STUDY / 'collect-invocation/actual-command.json'
        command = ['/usr/bin/python3', '-B', readers[0]['path'], '--spec', args.spec, '--spec-sha256', args.spec_sha256, '--result-root', spec['output_root'], '--output', str(STUDY / 'actual-cohort-audit.json'), '--checked-outer-receipt', str(outer), '--checked-outer-sha256', descriptor(outer)['sha256']]
    folder = STUDY / (args.mode + '-invocation')
    folder.mkdir()
    fd = os.open(folder.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)
    claim = dict(kind='exclusive_original_cli_claim', mode=args.mode, command=command,
        registered_wrapper_sha256=args.spec_sha256, created_utc=datetime.now(timezone.utc).isoformat(),
        cwd=spec['cwd'], retries=0, resume=False)
    save(folder / 'command-claim.json', claim)
    receipt = dict(claim, process_started=False, pid=None, original_exit=None,
        launch_outcome='reserved_known_not_launched', error=None, start_unix_ns=time.time_ns())
    process = None
    try:
        with (folder / 'stdout.log').open('xb') as out, (folder / 'stderr.log').open('xb') as err:
            receipt['launch_outcome'] = 'launch_call_outcome_unknown'
            process = subprocess.Popen(command, cwd=spec['cwd'], stdout=out, stderr=err, env=os.environ.copy())
            receipt.update(process_started=True, pid=process.pid, launch_outcome='known_started_exit_unknown')
            save(folder / 'process-started.json', dict(receipt))
            receipt['original_exit'] = process.wait()
            receipt['launch_outcome'] = 'known_started_exit_observed'
            for f in (out, err):
                f.flush(); os.fsync(f.fileno())
    except BaseException as error:
        receipt['error'] = type(error).__name__ + ': ' + str(error)
        if process is not None and process.poll() is None:
            try:
                # The checked driver catches KeyboardInterrupt and reaps its
                # owned coordinator/collector process group before returning.
                process.send_signal(signal.SIGINT)
                receipt['cleanup_requested'] = 'SIGINT to known child; bounded grace'
                process.wait(timeout=30)
            except BaseException as cleanup:
                receipt['cleanup_error'] = type(cleanup).__name__ + ': ' + str(cleanup)
        # Do not invent an unobserved exit or retry a partially known launch.
        if process is not None and process.poll() is not None:
            receipt['original_exit'] = process.returncode
            receipt['launch_outcome'] = 'known_started_exit_observed'
    finally:
        receipt['end_unix_ns'] = time.time_ns()
        try:
            for item in spec['frozen_inputs']:
                checked(item)
            checked(spec['binary'])
            receipt['source_inputs_unchanged'] = True
        except BaseException as error:
            receipt['source_inputs_unchanged'] = False
            receipt['source_guard_error'] = type(error).__name__ + ': ' + str(error)
        for name in ('stdout', 'stderr'):
            path = folder / (name + '.log')
            if path.is_file():
                receipt[name] = descriptor(path)
        save(folder / 'actual-command.json', receipt)
    print(json.dumps({'mode': args.mode, 'original_exit': receipt['original_exit'], 'source_inputs_unchanged': receipt['source_inputs_unchanged'], 'error': receipt['error']}))
    return receipt['original_exit'] if receipt['original_exit'] is not None and receipt['error'] is None and receipt['source_inputs_unchanged'] else 1

if __name__ == '__main__':
    raise SystemExit(main())
