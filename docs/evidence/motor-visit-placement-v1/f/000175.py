"""Retained fake-process fault tests only; zero native/subprocess execution."""
import copy
from datetime import datetime, timezone
import errno
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.dont_write_bytecode = True
import checked_driver
import cohort_driver
from cohort_contract import CASES, PROFILES, expected_driver_command, load_spec
from strict_io import canonical, descriptor, loads, write_exclusive

HERE = Path(__file__).resolve().parent
RUN = HERE / 'synthetic-tests' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
RUN.mkdir(parents=True, exist_ok=False)


class FakeNative:
    def __init__(self, argv, stdout, stderr, index, *, code=0, interrupt=False, timeout=False):
        self.pid = 700000 + index
        self.returncode = None
        self.code = code
        self.interrupt = interrupt
        self.timeout = timeout
        self.waits = 0
        self.killed = False
        Path(argv[argv.index('--output') + 1]).write_bytes(b'PURE_FAKE_RAW_NOT_NATIVE_GZIP\n')
        stdout.write(b'pure injected native stand-in\n')
        stderr.write(b'')

    def wait(self, timeout=None):
        self.waits += 1
        if self.interrupt and self.waits == 1:
            raise KeyboardInterrupt('injected owned child wait interruption')
        if self.timeout and self.waits == 1:
            raise subprocess.TimeoutExpired('fake native', timeout)
        if self.returncode is None:
            self.returncode = self.code
        return self.returncode

    def poll(self):
        return self.returncode

    def kill(self):
        self.killed = True
        self.returncode = -9


class FakeCoordinator:
    def __init__(self, code=0, fail_wait=False, grace_timeout=False):
        self.pid = 900001
        self.code = code
        self.fail_wait = fail_wait
        self.grace_timeout = grace_timeout
        self.calls = 0

    def wait(self, timeout=None):
        self.calls += 1
        if self.fail_wait and self.calls == 1:
            raise KeyboardInterrupt('injected checked parent interruption')
        if self.grace_timeout and self.calls == 2:
            raise subprocess.TimeoutExpired('fake coordinator grace', timeout)
        return self.code


class DriverTests(unittest.TestCase):
    def setUp(self):
        self.folder = Path(tempfile.mkdtemp(prefix=self._testMethodName + '-', dir=RUN))
        self.cwd = self.folder / 'collector'; self.cwd.mkdir()
        binary = self.folder / 'NONEXECUTABLE_MOCK_BINARY'; binary.write_bytes(b'not a native executable')
        self.dummy = self.folder / 'frozen-source.txt'; self.dummy.write_bytes(b'fixed synthetic source')
        roles = {}
        for name in ('protocol', 'collector_source_manifest', 'compiler_feature_receipt', 'producer_review', 'pure_tests_review'):
            path = self.folder / (name + '.json'); path.write_bytes(b'{"pure_mock":true}\n')
            roles[name] = descriptor(path)
        self.candidate = dict(kind='frozen_motor_visit_placement_cohort', schema_version=1,
            registered_utc=datetime.now(timezone.utc).isoformat(), collection_worlds_before_registration=0,
            cases=copy.deepcopy(CASES), binary=descriptor(binary), cwd=str(self.cwd),
            output_root=str(self.folder / 'collection'), frozen_inputs=[descriptor(HERE / n) for n in
                ('strict_io.py', 'cohort_contract.py', 'cohort_driver.py', 'checked_driver.py')] + [descriptor(self.dummy)],
            registration_artifacts=roles, source_scope='PURE_MOCK_NO_ENGINE',
            environment={n: os.environ.get(n) for n in ('RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTDOCFLAGS', 'RUST_TEST_THREADS')},
            timeout_seconds=1, process_failure_policy='stop_on_first_failure_no_resume_or_replacement',
            driver_run_receipt_path=str(self.folder / 'checked-driver/actual-command.json'),
            driver_interpreter=descriptor(Path('/usr/bin/python3')),
            checked_driver_source=descriptor(HERE / 'checked_driver.py'))
        self.calls = []

    def prepare(self, candidate=None):
        candidate = copy.deepcopy(self.candidate if candidate is None else candidate)
        parent = self.folder / ('registration-' + str(len(list(self.folder.glob('registration-*')))))
        parent.mkdir()
        path = parent / 'candidate.json'; write_exclusive(path, candidate)
        review = dict(status='PASS_PRE_DATA_SOURCE_PROTOCOL_AND_DRIVER', reviewed_candidate_sha256=descriptor(path)['sha256'],
            binary=candidate['binary'], frozen_inputs_sha256=hashlib.sha256(canonical(candidate['frozen_inputs'])).hexdigest(),
            pure_mock_only=True)
        admission = parent / 'mock-admission.json'; write_exclusive(admission, review)
        wrapper = parent / 'wrapper.json'; write_exclusive(wrapper, dict(kind='admitted_motor_visit_placement_cohort',
            schema_version=1, candidate=descriptor(path), admission=descriptor(admission)))
        return load_spec(wrapper, descriptor(wrapper)['sha256']), wrapper

    def spawn(self, argv, *, cwd, stdout, stderr, env, start_new_session):
        self.assertFalse(start_new_session, 'native must inherit checked-owned group')
        self.assertEqual(cwd, str(self.cwd))
        index = len(self.calls); self.calls.append(argv)
        return FakeNative(argv, stdout, stderr, index)

    def test_exact_fixed18_profile_sign_observer_order_and_argv(self):
        self.assertEqual(len(CASES), 18)
        self.assertEqual(list(PROFILES), ['split_1_2', 'early_1_0', 'early_3_0'])
        spec, _ = self.prepare()
        for i, case in enumerate(CASES):
            actual = cohort_driver.argv(spec, case, Path(spec['output_root']) / case['label'])
            self.assertEqual(actual, [spec['binary']['path'], '--profile', case['profile'], '--target', case['target'],
                '--observer', case['observer'], '--label', case['label'], '--output',
                str(Path(spec['output_root']) / case['label'] / 'raw.jsonl.gz')])
            self.assertEqual(case['observer'], 'off' if i % 3 == 2 else 'on')

    def test_validate_only_no_claim_and_no_spawn(self):
        spec, _ = self.prepare()
        with patch('subprocess.Popen', side_effect=AssertionError('no actual subprocess permitted')):
            result = cohort_driver.drive(spec, spawn=self.spawn)
        self.assertEqual(result['actual_processes_started'], 0)
        self.assertFalse(Path(spec['output_root']).exists())
        self.assertFalse(self.calls)

    def test_all18_fake_receipts_index_and_no_resume(self):
        spec, _ = self.prepare(); result = cohort_driver.drive(spec, launch_native=True, spawn=self.spawn)
        self.assertEqual(result['status'], 'all_eighteen_processes_completed')
        self.assertEqual(result['actual_processes_started'], 18)
        self.assertEqual(len(self.calls), 18)
        root = Path(spec['output_root']); index = loads((root / 'collection-index.json').read_bytes())
        self.assertEqual(len(index['entries']), 18)
        for i, entry in enumerate(index['entries']):
            self.assertEqual(entry['attempt_index'], i)
            self.assertEqual(entry['case'], CASES[i])
            self.assertEqual(entry['argv'], self.calls[i])
            folder = root / CASES[i]['label']
            self.assertTrue((folder / 'source-guard-before.json').exists())
            self.assertTrue((folder / 'source-guard-after.json').exists())
            self.assertEqual(loads((folder / 'process-terminal.json').read_bytes())['actual_exit_code'], 0)
        with self.assertRaises(FileExistsError): cohort_driver.drive(spec, launch_native=True, spawn=self.spawn)
        self.assertEqual(len(self.calls), 18)

    def test_parent_all18_claims_and_root_barrier_precede_spawn(self):
        spec, _ = self.prepare(); root = Path(spec['output_root']); synced = []
        original = cohort_driver.sync_directory
        def sync(path): synced.append(str(path)); original(path)
        def spawn(*args, **kwargs):
            self.assertIn(str(root.parent), synced); self.assertIn(str(root), synced)
            self.assertTrue((root / 'collection-index.json').exists())
            self.assertTrue((root / 'prelaunch-durability.json').exists())
            self.assertTrue(all((root / c['label'] / 'launch-claim.json').exists() for c in CASES))
            return self.spawn(*args, **kwargs)
        with patch.object(cohort_driver, 'sync_directory', side_effect=sync):
            result = cohort_driver.drive(spec, launch_native=True, spawn=spawn)
        self.assertEqual(result['status'], 'all_eighteen_processes_completed')

    def test_original_nonzero_stop_one_and_retain_all_reservations(self):
        spec, _ = self.prepare()
        def spawn(argv, **kwargs):
            self.calls.append(argv); return FakeNative(argv, kwargs['stdout'], kwargs['stderr'], 0, code=13)
        result = cohort_driver.drive(spec, launch_native=True, spawn=spawn)
        self.assertEqual(len(self.calls), 1); self.assertEqual(result['cases'][0]['actual_exit_code'], 13)
        self.assertEqual(result['actual_processes_started'], 1); self.assertEqual(len(result['unlaunched_case_labels']), 17)
        self.assertEqual(len(result['case_launch_accounting']), 18); self.assertEqual(result['retries'], 0)

    def test_popen_outcome_unknown_never_claims_verified_zero(self):
        spec, _ = self.prepare()
        def spawn(*_args, **_kwargs): raise OSError('unknown injected Popen outcome')
        result = cohort_driver.drive(spec, launch_native=True, spawn=spawn)
        self.assertFalse(result['native_processes_started_count_is_complete'])
        self.assertEqual(result['launch_outcome_unknown_case_labels'], [CASES[0]['label']])
        self.assertIsNone(result['case_launch_accounting'][0]['actual_exit_code'])
        self.assertEqual(len(result['unlaunched_case_labels']), 17)

    def test_process_started_receipt_fault_reaps_actual_child(self):
        spec, _ = self.prepare(); original = cohort_driver.write_exclusive; children = []
        def write(path, value):
            if Path(path).name == 'process-started.json': raise OSError(errno.ENOSPC, 'injected started storage')
            original(path, value)
        def spawn(argv, **kwargs):
            child = FakeNative(argv, kwargs['stdout'], kwargs['stderr'], 0); children.append(child); return child
        with patch.object(cohort_driver, 'write_exclusive', side_effect=write):
            result = cohort_driver.drive(spec, launch_native=True, spawn=spawn)
        self.assertTrue(children[0].killed); self.assertEqual(result['actual_processes_started'], 1)
        self.assertEqual(result['cases'][0]['actual_exit_code'], -9)

    def test_late_process_terminal_enospc_keeps_actual_start_exit(self):
        spec, _ = self.prepare(); original = cohort_driver.write_exclusive
        def write(path, value):
            if Path(path).name == 'process-terminal.json': raise OSError(errno.ENOSPC, 'injected late terminal storage')
            original(path, value)
        with patch.object(cohort_driver, 'write_exclusive', side_effect=write):
            result = cohort_driver.drive(spec, launch_native=True, spawn=self.spawn)
        self.assertEqual(len(self.calls), 1); self.assertEqual(result['actual_processes_started'], 1)
        self.assertEqual(result['cases'][0]['actual_exit_code'], 0)
        self.assertEqual(result['case_launch_accounting'][0]['status'], 'known_started_exit_observed')
        self.assertEqual(result['case_launch_accounting'][0]['process_terminal_storage'], 'failed_or_uncertain')
        self.assertTrue(Path(result['cases'][0]['raw_output']['path']).exists())

    def test_driver_terminal_enospc_cannot_claim_completed(self):
        spec, _ = self.prepare(); original = cohort_driver.write_exclusive
        def write(path, value):
            if Path(path).name == 'driver-terminal.json': raise OSError(errno.ENOSPC, 'injected final storage')
            original(path, value)
        with patch.object(cohort_driver, 'write_exclusive', side_effect=write):
            result = cohort_driver.drive(spec, launch_native=True, spawn=self.spawn)
        self.assertEqual(result['status'], 'FAILED_OR_PARTIAL_RETAINED')
        self.assertEqual(result['actual_processes_started'], 18); self.assertIn('driver_terminal_storage_error', result)

    def test_parent_fsync_failure_starts_none(self):
        spec, _ = self.prepare()
        with patch.object(cohort_driver, 'sync_directory', side_effect=OSError(errno.ENOSPC, 'parent barrier')):
            result = cohort_driver.drive(spec, launch_native=True, spawn=self.spawn)
        self.assertFalse(self.calls); self.assertEqual(result['actual_processes_started'], 0)
        self.assertFalse(result['prelaunch_directory_barriers_complete'])

    def test_interrupted_wait_retains_original_child_exit(self):
        spec, _ = self.prepare()
        def spawn(argv, **kwargs): return FakeNative(argv, kwargs['stdout'], kwargs['stderr'], 0, interrupt=True)
        result = cohort_driver.drive(spec, launch_native=True, spawn=spawn)
        self.assertEqual(result['actual_processes_started'], 1); self.assertEqual(result['cases'][0]['actual_exit_code'], -9)
        self.assertIn('KeyboardInterrupt', result['cases'][0]['error'])

    def test_native_timeout_retains_original_negative_exit(self):
        spec, _ = self.prepare()
        def spawn(argv, **kwargs): return FakeNative(argv, kwargs['stdout'], kwargs['stderr'], 0, timeout=True)
        result = cohort_driver.drive(spec, launch_native=True, spawn=spawn)
        self.assertTrue(result['cases'][0]['timed_out']); self.assertEqual(result['cases'][0]['actual_exit_code'], -9)

    def test_source_mutation_after_first_mock_stops(self):
        spec, _ = self.prepare()
        def spawn(*args, **kwargs):
            result = self.spawn(*args, **kwargs); self.dummy.write_bytes(b'mutated'); return result
        result = cohort_driver.drive(spec, launch_native=True, spawn=spawn)
        self.assertEqual(len(self.calls), 1); self.assertFalse(result['source_inputs_unchanged'])
        self.assertEqual(result['actual_processes_started'], 1)

    def test_wrapper_candidate_type_source_and_case_tamper_rejected(self):
        spec, wrapper = self.prepare()
        with self.assertRaises(ValueError): load_spec(wrapper, '0' * 64)
        for change in ('wrong_type', 'wrong_case', 'missing_helper', 'duplicate_driver'):
            value = copy.deepcopy(self.candidate)
            if change == 'wrong_type': value['schema_version'] = True
            if change == 'wrong_case': value['cases'][0]['profile'] = 'other'
            if change == 'missing_helper': value['frozen_inputs'] = [d for d in value['frozen_inputs'] if Path(d['path']).name != 'strict_io.py']
            if change == 'duplicate_driver': value['frozen_inputs'].append(copy.deepcopy(value['frozen_inputs'][2]))
            with self.assertRaises(ValueError): self.prepare(value)
        raw = (wrapper.parent / 'candidate.json').read_bytes()
        (wrapper.parent / 'candidate.json').write_bytes(raw + b' ')
        with self.assertRaises(ValueError): load_spec(wrapper, descriptor(wrapper)['sha256'])

    def test_contract_copy_separate_reader_folder_derives_actual_driver(self):
        folder = self.folder / 'reader'; folder.mkdir()
        path = folder / 'cohort_contract.py'; path.write_bytes((HERE / 'cohort_contract.py').read_bytes())
        value = copy.deepcopy(self.candidate); value['frozen_inputs'].append(descriptor(path))
        _spec, wrapper = self.prepare(value)
        loader = importlib.util.spec_from_file_location('pure_reader_contract_copy', path)
        module = importlib.util.module_from_spec(loader); loader.loader.exec_module(module)
        spec = module.load_spec(wrapper, descriptor(wrapper)['sha256'])
        self.assertEqual(module.expected_driver_command(spec)[2], str(HERE / 'cohort_driver.py'))

    def test_checked_validate_only_no_actual_process(self):
        spec, _ = self.prepare()
        with patch('subprocess.Popen', side_effect=AssertionError('no actual subprocess permitted')):
            result = checked_driver.run_checked(spec)
        self.assertEqual(result['actual_driver_processes'], 0)
        self.assertFalse(Path(spec['driver_run_receipt_path']).parent.exists())

    def outer_spawn(self, process):
        def spawn(command, **kwargs):
            self.assertTrue(kwargs['start_new_session'])
            kwargs['stdout'].write(b'{"status":"synthetic_positive_disk_is_not_actual_exit"}\n')
            return process
        return spawn

    def test_checked_nonzero_exit_is_preserved_even_positive_stdout(self):
        spec, _ = self.prepare()
        result = checked_driver.run_checked(spec, launch_native=True, spawn=self.outer_spawn(FakeCoordinator(code=9)))
        self.assertEqual(result['actual_exit_code'], 9)
        self.assertEqual(loads(Path(spec['driver_run_receipt_path']).read_bytes())['actual_exit_code'], 9)

    def test_outer_interrupt_signals_owned_whole_group_and_preserves_exit(self):
        spec, _ = self.prepare(); signals = []
        with patch.object(checked_driver.os, 'killpg', side_effect=lambda pid, sig: signals.append((pid, sig))):
            result = checked_driver.run_checked(spec, launch_native=True,
                spawn=self.outer_spawn(FakeCoordinator(code=-15, fail_wait=True)))
        self.assertEqual(signals, [(900001, signal.SIGTERM)])
        self.assertEqual(result['actual_exit_code'], -15); self.assertIn('owned process group', result['runtime_error'])

    def test_outer_group_grace_timeout_signals_sigkill_and_preserves_exit(self):
        spec, _ = self.prepare(); signals = []
        with patch.object(checked_driver.os, 'killpg', side_effect=lambda pid, sig: signals.append((pid, sig))):
            result = checked_driver.run_checked(spec, launch_native=True,
                spawn=self.outer_spawn(FakeCoordinator(code=-9, fail_wait=True, grace_timeout=True)))
        self.assertEqual(signals, [(900001, signal.SIGTERM), (900001, signal.SIGKILL)])
        self.assertEqual(result['actual_exit_code'], -9)

    def test_outer_final_receipt_storage_failure_propagates(self):
        spec, _ = self.prepare(); original = checked_driver.write_exclusive
        def write(path, value):
            if Path(path).name == 'actual-command.json': raise OSError(errno.ENOSPC, 'injected outer terminal')
            original(path, value)
        with patch.object(checked_driver, 'write_exclusive', side_effect=write):
            with self.assertRaises(OSError): checked_driver.run_checked(spec, launch_native=True,
                spawn=self.outer_spawn(FakeCoordinator(code=0)))


if __name__ == '__main__':
    before = {str(p): descriptor(p) for p in sorted(HERE.glob('*.py'))}
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(DriverTests)
    with (RUN / 'unittest.log').open('x') as log:
        with patch('subprocess.Popen', side_effect=AssertionError('all process calls must be injected')):
            result = unittest.TextTestRunner(stream=log, verbosity=2).run(suite)
    after = {str(p): descriptor(p) for p in sorted(HERE.glob('*.py'))}
    receipt = dict(kind='pure_fake_process_fixed18_driver_tests', tests=result.testsRun,
        failures=len(result.failures), errors=len(result.errors), passed=result.wasSuccessful(),
        source_before=before, source_after=after, source_inputs_unchanged=before == after,
        native_worlds=0, native_processes=0, actual_subprocesses=0, actual_process_signals=0,
        old_numeric_or_raw_data_read=False, retained_fake_failure_artifacts=True)
    write_exclusive(RUN / 'actual-pure-tests.json', receipt)
    print(json.dumps({'tests': result.testsRun, 'passed': result.wasSuccessful(), 'receipt': descriptor(RUN / 'actual-pure-tests.json'),
                      'log': descriptor(RUN / 'unittest.log'), 'native_worlds': 0}))
    raise SystemExit(0 if result.wasSuccessful() and before == after else 1)
