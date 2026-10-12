"""Fixed six unique processes; preparation is pure unless explicitly launched."""
import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import subprocess
from cohort_reader import CASES, load_spec
from strict_io import bound, descriptor, need, write_exclusive


def utc():
    return datetime.now(timezone.utc).isoformat()


def guard(spec):
    bound(spec['binary'])
    for item in spec['frozen_inputs']: bound(item)
    for item in spec['registration_artifacts'].values(): bound(item)
    for name, value in spec['environment'].items():
        need(os.environ.get(name) == value, 'registered runtime variable differs: ' + name)


def argv(spec, case, folder):
    return [spec['binary']['path'], '--target', case['target'], '--observer', case['observer'],
            '--label', case['label'], '--output', str(folder / 'raw.jsonl.gz')]


def drive(spec, *, launch_native=False, spawn=None):
    """Validate first. Tests may inject a mock; native launch requires explicit flag."""
    guard(spec)
    if not launch_native:
        return dict(status='validated_no_process_started', actual_processes_started=0,
                    registered_spec_sha256=spec['_authenticated_wrapper_sha256'])
    spawn = subprocess.Popen if spawn is None else spawn
    root = Path(spec['output_root'])
    # mkdir is the exclusive whole-cohort claim; no resume or retry path exists.
    root.mkdir(exist_ok=False)
    claim = dict(kind='exclusive_fixed_six_cohort_claim', created_utc=utc(), cases=CASES,
                 registered_spec_sha256=spec['_authenticated_wrapper_sha256'],
                 registered_candidate_sha256=spec['_authenticated_candidate_sha256'],
                 binary=spec['binary'], source_inputs=spec['frozen_inputs'])
    write_exclusive(root / 'cohort-claim.json', claim)
    folders = []
    for index, case in enumerate(CASES):
        folder = root / case['label']; folder.mkdir(exist_ok=False)
        write_exclusive(folder / 'launch-claim.json', dict(kind='exclusive_process_claim',
            case=case, attempt_index=index, argv=argv(spec, case, folder), cwd=spec['cwd'],
            binary=spec['binary'], created_utc=utc(),
            registered_spec_sha256=spec['_authenticated_wrapper_sha256']))
        folders.append(folder)
    started = 0
    results = []
    error = None
    unchanged = True
    try:
        for index, (case, folder) in enumerate(zip(CASES, folders)):
            guard(spec)
            command = argv(spec, case, folder)
            receipt = dict(kind='original_process_terminal', case=case, attempt_index=index,
                argv=command, cwd=spec['cwd'], started_utc=utc(), native_process_started=False,
                actual_exit_code=None, timed_out=False, error=None)
            with (folder / 'stdout.log').open('xb') as stdout, (folder / 'stderr.log').open('xb') as stderr:
                process = None
                try:
                    process = spawn(command, cwd=spec['cwd'], stdout=stdout, stderr=stderr,
                                    env=os.environ.copy(), start_new_session=True)
                    receipt['native_process_started'] = True
                    receipt['pid'] = process.pid
                    started += 1
                    write_exclusive(folder / 'process-started.json', dict(receipt))
                    try:
                        receipt['actual_exit_code'] = process.wait(timeout=spec['timeout_seconds'])
                    except subprocess.TimeoutExpired:
                        receipt['timed_out'] = True
                        process.kill()
                        receipt['actual_exit_code'] = process.wait()
                except BaseException as failure:
                    receipt['error'] = type(failure).__name__ + ': ' + str(failure)
                    if process is not None:
                        if process.poll() is None:
                            process.kill()
                        receipt['actual_exit_code'] = process.wait()
                finally:
                    stdout.flush(); os.fsync(stdout.fileno())
                    stderr.flush(); os.fsync(stderr.fileno())
            receipt['ended_utc'] = utc()
            receipt['stdout'] = descriptor(folder / 'stdout.log')
            receipt['stderr'] = descriptor(folder / 'stderr.log')
            output = folder / 'raw.jsonl.gz'
            if output.exists():
                with output.open('rb') as file: os.fsync(file.fileno())
                receipt['raw_output'] = descriptor(output)
            else:
                receipt['raw_output'] = None
            write_exclusive(folder / 'process-terminal.json', receipt)
            results.append(receipt)
            guard(spec)
            if receipt['actual_exit_code'] != 0 or receipt['timed_out'] or receipt['error'] is not None or receipt['raw_output'] is None:
                error = 'Original process failure at ' + case['label'] + '; remaining prelaunch claims retained, no replacement'
                break
    except Exception as failure:
        error = type(failure).__name__ + ': ' + str(failure)
        unchanged = False
    finally:
        try: guard(spec)
        except Exception as failure:
            unchanged = False
            error = error or type(failure).__name__ + ': ' + str(failure)
        passed = unchanged and started == 6 and len(results) == 6 and all(
            r['actual_exit_code'] == 0 and not r['timed_out'] and r['error'] is None and
            r['raw_output'] is not None for r in results)
        terminal = dict(kind='fixed_six_process_driver_terminal',
            status='all_six_processes_completed' if passed else 'FAILED_OR_PARTIAL_RETAINED',
            ended_utc=utc(), actual_processes_started=started, original_process_terminals=len(results),
            source_inputs_unchanged=unchanged, error=error,
            registered_spec_sha256=spec['_authenticated_wrapper_sha256'],
            registered_candidate_sha256=spec['_authenticated_candidate_sha256'],
            cases=results, retries=0, resumed=False)
        terminal['unlaunched_case_labels'] = [case['label'] for case in CASES[len(results):]]
        write_exclusive(root / 'driver-terminal.json', terminal)
    return terminal


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--spec', required=True); parser.add_argument('--spec-sha256', required=True)
    parser.add_argument('--launch-native', action='store_true')
    args = parser.parse_args()
    spec = load_spec(args.spec, args.spec_sha256)
    result = drive(spec, launch_native=args.launch_native)
    print(result['status'])
    return 0 if result['status'] in ('validated_no_process_started', 'all_six_processes_completed') else 2


if __name__ == '__main__':
    raise SystemExit(main())
