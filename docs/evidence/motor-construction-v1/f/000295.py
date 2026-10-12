"""Synthetic files and stub processes only; real 141/143 binaries never run."""

import copy
import hashlib
import json
import os
from pathlib import Path
import sys
import unittest
from unittest import mock

import driver


STUB = r'''#!INTERPRETER
import json,os,sys
from pathlib import Path
args=sys.argv[1:]
mode=os.environ.get('STUB_MODE','success')
event=Path(os.environ['STUB_EVENTS'])
with event.open('a') as stream: stream.write(json.dumps(args)+'\n')
if args[0]=='--output':
 out=Path(args[1])
 with out.open('xb') as stream:
  stream.write(b'{"synthetic_partial":' if mode=='partial' else Path(os.environ['STUB_CURRENT']).read_bytes())
 if mode=='mutate_source': Path(os.environ['STUB_SOURCE']).write_bytes(b'SYNTHETIC_CHANGED_SOURCE\n')
 if mode=='partial':
  print('synthetic partial collection');print('synthetic retained error',file=sys.stderr);sys.exit(7)
 print('collection_complete=true completed_worlds=72/72 replay_qualification=true primary_calibration=FAIL mode=collect')
 sys.exit(1 if mode=='wrong_exit' else 2)
if args[0]=='--verify-report':
 if args[1]==os.environ['STUB_BASELINE']:
  print('report selected sources are stale/mixed',file=sys.stderr);sys.exit(1)
 print('collection_complete=true completed_worlds=72/72 replay_qualification=true primary_calibration=FAIL mode=verify-report-no-worlds')
 sys.exit(3 if mode=='bad_verify' else 2 if '--require-calibration' in args else 0)
sys.exit(93)
'''


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('xb') as stream:
        stream.write(data)


class StubQualification(unittest.TestCase):
    def setUp(self):
        base = Path(os.environ['SYNTHETIC_FIXTURE_ROOT'])
        self.root = base / self._testMethodName
        self.root.mkdir(exist_ok=False)
        self.source = self.root / 'synthetic-source'
        self.source.mkdir()
        self.names = sorted(driver.compare.BEFORE_KEYS | {driver.compare.ADDED_KEY})
        for name in self.names:
            save(self.source / name, ('SYNTHETIC SOURCE ONLY ' + name + '\n').encode())
        self.inventory = mock.patch.object(driver, 'discover_source_inputs', return_value=self.names)
        self.inventory.start()
        self.addCleanup(self.inventory.stop)
        guard = driver.source_snapshot(self.source)
        after = {entry['path']: entry['sha256'] for entry in guard['inputs']}
        before = {name: sha for name, sha in after.items() if name != driver.compare.ADDED_KEY}
        changed = sorted(driver.compare.PERMITTED_EXISTING_CHANGES)
        for name in changed:
            before[name] = hashlib.sha256(('SYNTHETIC BEFORE ' + name).encode()).hexdigest()
        report = {'schema_version': 2, 'protocol': {'SYNTHETIC_ONLY': True},
                  'selected_source_sha256': dict(sorted(before.items())),
                  'cases': [{'synthetic_index': i, 'runs': [
                      {'collector_enabled': on, 'world_created': True, 'completed_step': True,
                       'error': None, 'opaque_native_bytes': [0, 128, 255], 'native_word': '80000000'}
                      for on in [True, True, False]]} for i in range(24)],
                  'numerical_sensitivity': [{'opaque': -0.0}], 'summary': driver.SUMMARY}
        baseline = (json.dumps(report, indent=2) + '\n').encode()
        self.transition = {'kind': 'rne_141_motor_construction_source_transition', 'schema_version': 1,
                           'status': 'FROZEN_BEFORE_COLLECTION', 'baseline_report_sha256': driver.digest(baseline),
                           'before_source_sha256': before, 'after_source_sha256': dict(sorted(after.items())),
                           'changed_existing_paths': changed, 'added_paths': [driver.compare.ADDED_KEY], 'removed_paths': []}
        self.baseline = self.root / 'SYNTHETIC-baseline-report.json'
        self.current = self.root / 'SYNTHETIC-current-template.json'
        save(self.baseline, baseline)
        save(self.current, driver.compare.expected_bytes(baseline, self.transition))
        guard_path = self.root / 'source-guard.json'
        save(guard_path, driver.canonical(guard))
        binary = self.root / 'SYNTHETIC-STUB-NOT-PHYSICS'
        save(binary, STUB.replace('INTERPRETER', sys.executable).encode())
        binary.chmod(0o755)
        protocol = self.root / 'protocol.json'
        save(protocol, driver.canonical(driver.fixed_protocol()))
        old_binding = self.root / 'baseline-source-map-binding.json'
        save(old_binding, driver.canonical({'kind': 'rne_141_baseline_source_map_binding', 'schema_version': 1,
                                            'status': 'PASS_BASELINE_SOURCE_MAP_BINDING',
                                            'baseline_report_sha256': driver.digest(baseline), 'before_source_sha256': before}))
        build = self.root / 'SYNTHETIC-build-receipt.json'
        save(build, driver.canonical({'kind': 'rne_141_compiler_build_binding', 'schema_version': 1,
                                      'original_exit_code': 0, 'binary_sha256': driver.stream_descriptor(binary)['sha256'],
                                      'source_guard_sha256': driver.stream_descriptor(guard_path)['sha256'],
                                      'profile': 'release', 'scalar': 'f32', 'dimension': 'dim3',
                                      'features': ['SYNTHETIC_ONLY'], 'rustc_version': 'SYNTHETIC NO COMPILATION',
                                      'cargo_version': 'SYNTHETIC NO BUILD', 'target': 'SYNTHETIC', 'build_flags': {}}))
        test_log = self.root / 'SYNTHETIC-pure-test.log'
        save(test_log, b'SYNTHETIC pre-collection receipt fixture, not actual qualification\n')
        artifacts = {role: driver.stream_descriptor(path) for role, path in {
            'binary': binary, 'build_receipt': build, 'source_guard': guard_path, 'protocol': protocol,
            'baseline_report': self.baseline, 'baseline_source_map_receipt': old_binding,
            'driver_source': Path(driver.__file__).resolve(), 'comparator_source': driver.COMPARATOR,
            'test_source': Path(__file__).resolve(), 'test_runner_source': driver.HERE / 'run_tests.py',
            'pure_test_log': test_log, 'synthetic_current_template': self.current}.items()}
        pure = self.root / 'SYNTHETIC-pure-test-receipt.json'
        save(pure, driver.canonical({'kind': 'rne_141_synthetic_driver_test_receipt', 'schema_version': 1,
                                     'status': 'PASS', 'original_exit_code': 0, 'actual_report_reads': 0, 'worlds': 0,
                                     'source_inputs_unchanged': True, 'log': artifacts['pure_test_log'],
                                     'tested_tool_sha256': {role: artifacts[role]['sha256'] for role in
                                         ['driver_source', 'comparator_source', 'test_source', 'test_runner_source']},
                                     'synthetic_fixture_not_actual_test_receipt': True}))
        artifacts['pure_test_receipt'] = driver.stream_descriptor(pure)
        self.events = self.root / 'SYNTHETIC-process-events.jsonl'
        self.inputs = {'kind': 'rne_141_source_transition_inputs', 'schema_version': 1,
                       'run_root': str(self.root / 'once-only-attempt'), 'source_root': str(self.source),
                       'environment': {'STUB_CURRENT': str(self.current), 'STUB_BASELINE': str(self.baseline),
                                       'STUB_SOURCE': str(self.source / self.names[0]), 'STUB_EVENTS': str(self.events),
                                       'STUB_MODE': 'success'},
                       'transition': self.transition, 'artifacts': artifacts}

    def setup_freeze(self):
        inputs = self.root / 'supplied-inputs.json'
        save(inputs, driver.canonical(self.inputs))
        prepared = driver.prepare(inputs, self.root / 'prepared')
        review = self.root / 'SYNTHETIC-independent-review.json'
        descriptor = driver.stream_descriptor(prepared)
        save(review, driver.canonical({'kind': 'rne_141_independent_precollection_review',
                                       'status': 'PASS_INDEPENDENT_PRE_COLLECTION_SOURCE_TRANSITION_REVIEW',
                                       'before_world_collection': True,
                                       'reviewed_prepared_binding_sha256': descriptor['sha256'],
                                       'reviewed_prepared_binding_size_bytes': descriptor['size_bytes'], 'world_budget': 72,
                                       'synthetic_fixture_not_actual_independent_review': True}))
        frozen = driver.freeze(prepared, driver.stream_descriptor(review), self.root / 'frozen')
        self.frozen = frozen
        self.freeze_sha = driver.stream_descriptor(frozen)['sha256']
        return frozen

    def run_stub(self, mode='success'):
        self.inputs['environment']['STUB_MODE'] = mode
        self.setup_freeze()
        return self.run_frozen()

    def run_frozen(self):
        return driver.run(self.frozen, self.freeze_sha)

    def event_count(self):
        return len(self.events.read_bytes().splitlines()) if self.events.exists() else 0

    def test_complete_one_collection_and_three_zero_world_gates(self):
        result = self.run_stub()
        self.assertTrue(result['status'].startswith('PASS_'))
        self.assertEqual(result['collector_original_exit_code'], 2)
        self.assertEqual(result['collector_processes_started'], 1)
        self.assertEqual(result['postcollection_processes_started'], 3)
        self.assertEqual(self.event_count(), 4)
        self.assertTrue(result['source_inputs_unchanged'])
        terminal = driver.load(Path(self.inputs['run_root']) / 'collection/terminal.json')
        self.assertEqual(terminal['argv'][1:], ['--output', self.inputs['run_root'] + '/report.json', '--require-calibration'])

    def test_second_attempt_refused_without_extra_stub_launch(self):
        self.run_stub()
        root = Path(self.inputs['run_root'])
        before = driver.stream_descriptor(root / 'report.json')
        with self.assertRaises(driver.Refused): self.run_frozen()
        self.assertEqual(self.event_count(), 4)
        self.assertEqual(driver.stream_descriptor(root / 'report.json'), before)

    def test_partial_original_exit_and_all_partial_bytes_retained_no_retry(self):
        result = self.run_stub('partial')
        self.assertEqual(result['status'], 'FAIL_RETAINED_NO_RETRY')
        self.assertEqual(result['collector_original_exit_code'], 7)
        self.assertEqual(result['postcollection_processes_started'], 0)
        self.assertEqual(self.event_count(), 1)
        root = Path(self.inputs['run_root'])
        self.assertEqual((root / 'report.json').read_bytes(), b'{"synthetic_partial":')
        self.assertIn(b'synthetic retained error', (root / 'collection/stderr.log').read_bytes())
        with self.assertRaises(driver.Refused): self.run_frozen()
        self.assertEqual(self.event_count(), 1)

    def test_unexpected_collection_exit_is_failure_despite_full_report(self):
        result = self.run_stub('wrong_exit')
        self.assertEqual(result['collector_original_exit_code'], 1)
        self.assertEqual(result['status'], 'FAIL_RETAINED_NO_RETRY')
        self.assertEqual(self.event_count(), 1)

    def test_native_payload_change_rejects_after_collection_before_verifiers(self):
        altered = self.current.read_bytes().replace(b'"native_word": "80000000"', b'"native_word": "00000000"', 1)
        self.current.write_bytes(altered)
        self.inputs['artifacts']['synthetic_current_template'] = driver.stream_descriptor(self.current)
        result = self.run_stub()
        self.assertEqual(result['collector_original_exit_code'], 2)
        self.assertEqual(result['status'], 'FAIL_RETAINED_NO_RETRY')
        self.assertEqual(self.event_count(), 1)
        self.assertIn('bytes outside', result['error'])

    def test_summary_false_cannot_replace_integer_zero(self):
        altered = self.current.read_bytes().replace(b'"primary_nonzero_passes": 0', b'"primary_nonzero_passes": false', 1)
        self.current.write_bytes(altered)
        self.inputs['artifacts']['synthetic_current_template'] = driver.stream_descriptor(self.current)
        result = self.run_stub()
        self.assertEqual(result['status'], 'FAIL_RETAINED_NO_RETRY')
        self.assertEqual(self.event_count(), 1)
        self.assertEqual(result['world_accounting']['status'], 'RETAINED_INCOMPLETE_OR_CHANGED')

    def test_zero_world_verifier_failure_preserves_original_exit_and_stops(self):
        result = self.run_stub('bad_verify')
        self.assertEqual(result['status'], 'FAIL_RETAINED_NO_RETRY')
        self.assertEqual(result['postcollection_processes_started'], 1)
        self.assertEqual(self.event_count(), 2)
        terminal = driver.load(Path(self.inputs['run_root']) / 'verify-current/terminal.json')
        self.assertEqual(terminal['original_exit_code'], 3)

    def test_mutated_source_fails_guard_and_stops_before_comparison(self):
        result = self.run_stub('mutate_source')
        self.assertEqual(result['collector_original_exit_code'], 2)
        self.assertEqual(result['status'], 'FAIL_RETAINED_NO_RETRY')
        self.assertFalse(result['source_inputs_unchanged'])
        self.assertEqual(self.event_count(), 1)

    def test_baseline_hash_tamper_refuses_before_stub(self):
        self.setup_freeze()
        self.baseline.write_bytes(self.baseline.read_bytes() + b'\n')
        with self.assertRaises(driver.Refused): self.run_frozen()
        self.assertEqual(self.event_count(), 0)

    def test_source_manifest_omission_is_refused(self):
        guard_path = Path(self.inputs['artifacts']['source_guard']['path'])
        guard = driver.load(guard_path)
        guard['inputs'].pop()
        guard_path.write_bytes(driver.canonical(guard))
        self.inputs['artifacts']['source_guard'] = driver.stream_descriptor(guard_path)
        with self.assertRaises(driver.Refused): driver.validate_inputs(self.inputs)
        self.assertEqual(self.event_count(), 0)

    def test_pending_or_undeclared_map_cannot_freeze(self):
        self.inputs['transition']['status'] = 'DRAFT_NOT_FROZEN'
        with self.assertRaises(driver.compare.RejectedTransition): driver.validate_inputs(self.inputs)
        self.assertEqual(self.event_count(), 0)

    def test_environment_is_explicit_not_inherited(self):
        with mock.patch.dict(os.environ, {'STUB_MODE': 'partial'}):
            result = self.run_stub()
        self.assertTrue(result['status'].startswith('PASS_'))

    def test_review_wrong_binding_sha_refuses(self):
        self.setup_freeze()
        review_path = self.root / 'SYNTHETIC-independent-review.json'
        review = driver.load(review_path)
        review['reviewed_prepared_binding_sha256'] = '0' * 64
        review_path.write_bytes(driver.canonical(review))
        with self.assertRaises(driver.Refused): self.run_frozen()
        self.assertEqual(self.event_count(), 0)

    def test_baseline_never_decoded_during_prepare_or_freeze(self):
        old = driver.load
        def deny_baseline(path):
            self.assertNotEqual(Path(path), self.baseline)
            return old(path)
        with mock.patch.object(driver, 'load', side_effect=deny_baseline): self.setup_freeze()
        self.assertEqual(self.event_count(), 0)

    def test_integer_schema_cannot_be_boolean(self):
        self.inputs['schema_version'] = True
        with self.assertRaises(driver.Refused): driver.validate_inputs(self.inputs)

    def test_binary_hash_mismatch_refuses(self):
        self.inputs['artifacts']['binary']['sha256'] = '0' * 64
        with self.assertRaises(driver.Refused): driver.validate_inputs(self.inputs)
        self.assertEqual(self.event_count(), 0)

    def test_output_root_cannot_replace_baseline(self):
        self.inputs['run_root'] = str(self.baseline)
        with self.assertRaises(driver.Refused): driver.validate_inputs(self.inputs)
        self.assertEqual(self.event_count(), 0)

    def test_frozen_file_external_hash_must_match(self):
        self.setup_freeze()
        with self.assertRaises(driver.Refused): driver.run(self.frozen, '0' * 64)
        self.assertEqual(self.event_count(), 0)

    def test_launch_failure_retains_consumed_claim_without_retry(self):
        self.setup_freeze()
        with mock.patch.object(driver.subprocess, 'Popen', side_effect=FileNotFoundError('SYNTHETIC launch fault')):
            result = self.run_frozen()
        self.assertEqual(result['collector_processes_started'], 0)
        self.assertIsNone(result['collector_original_exit_code'])
        self.assertEqual(result['status'], 'FAIL_RETAINED_NO_RETRY')
        self.assertTrue((Path(self.inputs['run_root']) / 'claim.json').exists())
        with self.assertRaises(driver.Refused): self.run_frozen()

    def test_pid_receipt_fault_still_waits_and_retains_original_exit(self):
        self.setup_freeze()
        original = driver.exclusive_json
        def fail_pid(path, value):
            if path.name == 'pid.json': raise OSError('SYNTHETIC storage fault')
            return original(path, value)
        with mock.patch.object(driver, 'exclusive_json', side_effect=fail_pid): result = self.run_frozen()
        self.assertEqual(result['collector_original_exit_code'], 2)
        self.assertEqual(result['collector_processes_started'], 1)
        self.assertEqual(result['status'], 'FAIL_RETAINED_NO_RETRY')
        self.assertEqual(self.event_count(), 1)
        self.assertEqual(driver.load(Path(self.inputs['run_root']) / 'collection/terminal.json')['original_exit_code'], 2)


if __name__ == '__main__': unittest.main(verbosity=2)
