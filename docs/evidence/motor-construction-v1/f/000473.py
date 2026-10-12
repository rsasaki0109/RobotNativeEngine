"""Exclusive outer checked command; this Python child driver is not a native world."""
import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import subprocess
from cohort_driver import guard
from cohort_reader import expected_driver_command, load_spec
from strict_io import bound, descriptor, sync_directory, write_exclusive


def utc():
    return datetime.now(timezone.utc).isoformat()


def checked_guard(spec):
    guard(spec)
    bound(spec['driver_interpreter']); bound(spec['checked_driver_source'])
    load_spec(spec['_authenticated_wrapper_path'], spec['_authenticated_wrapper_sha256'])


def run_checked(spec, *, launch_native=False, spawn=None):
    checked_guard(spec)
    if not launch_native:
        return dict(status='validated_no_process_started', actual_driver_processes=0)
    spawn = subprocess.Popen if spawn is None else spawn
    receipt_path = Path(spec['driver_run_receipt_path'])
    folder = receipt_path.parent
    folder.mkdir(exist_ok=False)
    sync_directory(folder.parent)
    command = expected_driver_command(spec)
    write_exclusive(folder / 'checked-command-claim.json', dict(kind='exclusive_checked_driver_command_claim',
        command=command, cwd=spec['cwd'], created_utc=utc(),
        registered_wrapper_sha256=spec['_authenticated_wrapper_sha256']))
    receipt = dict(kind='original_checked_driver_command', schema_version=2,
        command=command, cwd=spec['cwd'], registered_wrapper_sha256=spec['_authenticated_wrapper_sha256'],
        registered_candidate_sha256=spec['_authenticated_candidate_sha256'],
        started_utc=utc(), ended_utc=None, driver_process_started=False, driver_process_id=None,
        launch_outcome='reserved_known_not_launched', actual_exit_code=None,
        source_inputs_unchanged=False, source_guard_error=None, runtime_error=None,
        stdout=None, stderr=None)
    process = None
    try:
        checked_guard(spec)
        with (folder / 'stdout.log').open('xb') as stdout, (folder / 'stderr.log').open('xb') as stderr:
            try:
                receipt['launch_outcome'] = 'launch_call_outcome_unknown'
                process = spawn(command, cwd=spec['cwd'], stdout=stdout, stderr=stderr,
                    env=os.environ.copy(), start_new_session=True)
                receipt['driver_process_started'] = True
                receipt['driver_process_id'] = process.pid
                receipt['launch_outcome'] = 'known_started_exit_unknown'
                receipt['actual_exit_code'] = process.wait(timeout=spec['timeout_seconds'] * 6 + 120)
                receipt['launch_outcome'] = 'known_started_exit_observed'
            except BaseException as error:
                receipt['runtime_error'] = type(error).__name__ + ': ' + str(error)
                if process is not None:
                    try:
                        if process.poll() is None: process.kill()
                        receipt['actual_exit_code'] = process.wait()
                        receipt['launch_outcome'] = 'known_started_exit_observed'
                    except BaseException as reap:
                        receipt['runtime_error'] += '; driver reap: ' + type(reap).__name__ + ': ' + str(reap)
            finally:
                for file in (stdout, stderr):
                    file.flush(); os.fsync(file.fileno())
    except BaseException as error:
        receipt['runtime_error'] = (receipt['runtime_error'] or '') + '; outer storage: ' + type(error).__name__ + ': ' + str(error)
    finally:
        receipt['ended_utc'] = utc()
        try:
            checked_guard(spec)
            receipt['source_inputs_unchanged'] = True
        except BaseException as error:
            receipt['source_guard_error'] = type(error).__name__ + ': ' + str(error)
        for key in ('stdout', 'stderr'):
            path = folder / (key + '.log')
            if path.exists(): receipt[key] = descriptor(path)
        # A final write/fsync failure propagates as nonzero to the checked parent;
        # root must retain and require this outer CLI's actual original exit too.
        write_exclusive(receipt_path, receipt)
    return receipt


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--spec', required=True)
    parser.add_argument('--spec-sha256', required=True); parser.add_argument('--launch-native', action='store_true')
    args = parser.parse_args()
    result = run_checked(load_spec(args.spec, args.spec_sha256), launch_native=args.launch_native)
    import json
    print(json.dumps(result, sort_keys=True))
    if result.get('status') == 'validated_no_process_started': return 0
    return 0 if result['actual_exit_code'] == 0 and result['source_inputs_unchanged'] and result['runtime_error'] is None and result['source_guard_error'] is None else 2


if __name__ == '__main__':
    raise SystemExit(main())
