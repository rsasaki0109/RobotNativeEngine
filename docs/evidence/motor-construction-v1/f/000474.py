"""Version-two fixed-six driver with durable reservations and truthful fault accounting."""
import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import subprocess
from cohort_reader import CASES, load_spec
from strict_io import bound, descriptor, need, sync_directory, write_exclusive


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


def error_text(error):
    return type(error).__name__ + ': ' + str(error)


def retain_process(spec, case, index, folder, accounting, spawn):
    """Always return memory of an attempted case; storage never determines launch status."""
    receipt = dict(kind='original_process_terminal', case=case, attempt_index=index,
        argv=argv(spec, case, folder), cwd=spec['cwd'], started_utc=utc(),
        native_process_started=False, actual_exit_code=None, timed_out=False, error=None,
        pid=None, ended_utc=None, stdout=None, stderr=None, raw_output=None)
    process = None
    try:
        with (folder / 'stdout.log').open('xb') as stdout, (folder / 'stderr.log').open('xb') as stderr:
            try:
                write_exclusive(folder / 'launch-intent.json', dict(kind='durable_launch_intent',
                    case=case, attempt_index=index, argv=receipt['argv'], created_utc=utc(),
                    registered_spec_sha256=spec['_authenticated_wrapper_sha256']))
                # Popen exceptions cannot prove that no child ever existed.
                accounting['status'] = 'launch_call_outcome_unknown'
                process = spawn(receipt['argv'], cwd=spec['cwd'], stdout=stdout, stderr=stderr,
                    env=os.environ.copy(), start_new_session=True)
                accounting['status'] = 'known_started_exit_unknown'
                accounting['pid'] = process.pid
                receipt['native_process_started'] = True
                receipt['pid'] = process.pid
                write_exclusive(folder / 'process-started.json', dict(receipt))
                try:
                    receipt['actual_exit_code'] = process.wait(timeout=spec['timeout_seconds'])
                except subprocess.TimeoutExpired:
                    receipt['timed_out'] = True
                    process.kill()
                    receipt['actual_exit_code'] = process.wait()
                accounting['status'] = 'known_started_exit_observed'
                accounting['actual_exit_code'] = receipt['actual_exit_code']
            except BaseException as failure:
                receipt['error'] = error_text(failure)
                if process is not None:
                    try:
                        if process.poll() is None:
                            process.kill()
                        receipt['actual_exit_code'] = process.wait()
                        accounting['status'] = 'known_started_exit_observed'
                        accounting['actual_exit_code'] = receipt['actual_exit_code']
                    except BaseException as reap_failure:
                        receipt['error'] += '; owned process reap: ' + error_text(reap_failure)
            finally:
                for file in (stdout, stderr):
                    try:
                        file.flush(); os.fsync(file.fileno())
                    except BaseException as sync_failure:
                        receipt['error'] = (receipt['error'] or '') + '; log fsync: ' + error_text(sync_failure)
    except BaseException as failure:
        receipt['error'] = (receipt['error'] or '') + '; attempt storage: ' + error_text(failure)
    receipt['ended_utc'] = utc()
    for key, filename in [('stdout', 'stdout.log'), ('stderr', 'stderr.log'), ('raw_output', 'raw.jsonl.gz')]:
        path = folder / filename
        if path.exists():
            try:
                with path.open('rb') as file: os.fsync(file.fileno())
                receipt[key] = descriptor(path)
            except BaseException as failure:
                receipt['error'] = (receipt['error'] or '') + '; artifact retention: ' + error_text(failure)
    try:
        write_exclusive(folder / 'process-terminal.json', receipt)
        accounting['process_terminal_storage'] = 'stored_fsynced'
    except BaseException as failure:
        accounting['process_terminal_storage'] = 'failed_or_uncertain'
        accounting['storage_error'] = error_text(failure)
    return receipt


def drive(spec, *, launch_native=False, spawn=None):
    """Pure validation by default; execution requires separate explicit authorization."""
    guard(spec)
    if not launch_native:
        return dict(status='validated_no_process_started', actual_processes_started=0,
                    registered_spec_sha256=spec['_authenticated_wrapper_sha256'])
    spawn = subprocess.Popen if spawn is None else spawn
    root = Path(spec['output_root'])
    root.mkdir(exist_ok=False)
    accounting = [dict(case=case, attempt_index=i, status='reserved_known_not_launched',
        reservation_durable=False, pid=None, actual_exit_code=None,
        process_terminal_storage='not_attempted', storage_error=None)
        for i, case in enumerate(CASES)]
    results = []
    error = None
    barriers_complete = False
    unchanged = True
    try:
        sync_directory(root.parent)
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
        # Case-file/directory syncs do not persist the six entries in root.
        sync_directory(root)
        barriers_complete = True
        for item in accounting: item['reservation_durable'] = True
        write_exclusive(root / 'prelaunch-durability.json', dict(kind='completed_prelaunch_directory_barriers',
            schema_version=2, root_parent=str(root.parent), cohort_root=str(root),
            completed_utc=utc(), all_six_claims_durable=True,
            registered_spec_sha256=spec['_authenticated_wrapper_sha256']))
        for index, (case, folder) in enumerate(zip(CASES, folders)):
            guard(spec)
            receipt = retain_process(spec, case, index, folder, accounting[index], spawn)
            results.append(receipt)
            guard(spec)
            if (accounting[index]['process_terminal_storage'] != 'stored_fsynced' or
                receipt['actual_exit_code'] != 0 or receipt['timed_out'] or
                receipt['error'] is not None or receipt['raw_output'] is None):
                error = 'Original failure at ' + case['label'] + '; unattempted reservations retained, no replacement'
                break
    except BaseException as failure:
        error = error_text(failure)
    finally:
        try: guard(spec)
        except BaseException as failure:
            unchanged = False
            error = error or error_text(failure)
        started = sum(item['status'].startswith('known_started_') for item in accounting)
        unknown = [item['case']['label'] for item in accounting if item['status'] == 'launch_call_outcome_unknown']
        not_launched = [item['case']['label'] for item in accounting if item['status'] == 'reserved_known_not_launched']
        stored = sum(item['process_terminal_storage'] == 'stored_fsynced' for item in accounting)
        passed = (error is None and unchanged and barriers_complete and started == 6 and
            not unknown and len(results) == stored == 6 and all(
                item['status'] == 'known_started_exit_observed' and item['actual_exit_code'] == 0
                for item in accounting))
        terminal = dict(kind='fixed_six_process_driver_terminal', driver_schema_version=2,
            status='all_six_processes_completed' if passed else 'FAILED_OR_PARTIAL_RETAINED',
            ended_utc=utc(), actual_processes_started=started,
            native_processes_started_count_is_complete=not unknown,
            original_process_terminals=stored, observed_process_terminal_memories=len(results),
            prelaunch_directory_barriers_complete=barriers_complete,
            source_inputs_unchanged=unchanged, error=error,
            registered_spec_sha256=spec['_authenticated_wrapper_sha256'],
            registered_candidate_sha256=spec['_authenticated_candidate_sha256'],
            cases=results, case_launch_accounting=accounting, retries=0, resumed=False,
            unlaunched_case_labels=not_launched, launch_outcome_unknown_case_labels=unknown)
        try:
            write_exclusive(root / 'driver-terminal.json', terminal)
        except BaseException as final_storage_failure:
            terminal['status'] = 'FAILED_OR_PARTIAL_RETAINED'
            terminal['driver_terminal_storage_error'] = error_text(final_storage_failure)
    return terminal


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--spec', required=True); parser.add_argument('--spec-sha256', required=True)
    parser.add_argument('--launch-native', action='store_true')
    args = parser.parse_args()
    result = drive(load_spec(args.spec, args.spec_sha256), launch_native=args.launch_native)
    # The checked caller preserves memory and nonzero exit even if disk rejects
    # this final terminal. No inherited credentials or environment are printed.
    import json
    print(json.dumps(result, sort_keys=True))
    return 0 if result['status'] in ('validated_no_process_started', 'all_six_processes_completed') else 2


if __name__ == '__main__':
    raise SystemExit(main())
