#!/usr/bin/env python3
"""Pure before-frozen qualification reader; creates no physics worlds."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import re
import sys

ROOT = Path('/workspace/.rne-kick-demo/single-joint-motor-calibration')
OUT = ROOT / 'next-experiment/vendor-lint-qualification'
REPO = Path('/workspace/RobotNativeEngine')
BASE = 'febe966d2d24b3808180e415f0376f628b90b9b6'
BASELINE = REPO / 'docs/media/position-motor-calibration.json'
BASELINE_SHA = '0e4e32a62a39ebce083b957f42025abd3e9b41700de8f78806fb7697beb15e2a'
ALLOWED = {
    'third_party/rapier3d/src/dynamics/joint/multibody_joint/multibody.rs',
    'third_party/rapier3d/src/dynamics/rigid_body_components.rs',
}
EXPECTED_STDOUT = (
    b'collection_complete=true completed_worlds=72/72 replay_qualification=true '
    b'primary_calibration=FAIL mode=collect\n'
)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def artifact(path, digest=None):
    path = Path(path)
    data = path.read_bytes()
    require(digest is None or sha(data) == digest, 'artifact hash mismatch: ' + str(path))
    return {'path': str(path), 'bytes': len(data), 'sha256': sha(data)}


def pairs_no_duplicates(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, 'duplicate JSON key: ' + key)
        result[key] = value
    return result


def reject_constant(value):
    raise ValueError('nonstandard JSON numeric constant: ' + value)


def load(raw):
    return json.loads(raw, object_pairs_hook=pairs_no_duplicates,
                      parse_constant=reject_constant)


def expected_bytes(original, selected):
    baseline = load(original)
    old = baseline['selected_source_sha256']
    require(len(old) == len(selected) == 17 and set(old) == set(selected),
            'selected source set is not the frozen 17 paths')
    changes = {name for name in old if old[name] != selected[name]}
    require(changes == ALLOWED, 'selected source changes differ from the allowed two')
    for name in selected:
        require(re.fullmatch(r'[0-9a-f]{64}', selected[name]) is not None,
                'invalid selected digest: ' + name)
    marker = b'  "selected_source_sha256": {\n'
    start = original.index(marker)
    end = original.index(b'\n  },\n  "cases":', start)
    source_object = original[start:end]
    for name in sorted(ALLOWED):
        old_line = json.dumps(name).encode() + b': ' + json.dumps(old[name]).encode()
        new_line = json.dumps(name).encode() + b': ' + json.dumps(selected[name]).encode()
        require(source_object.count(old_line) == 1, 'source replacement not unique: ' + name)
        source_object = source_object.replace(old_line, new_line, 1)
    return original[:start] + source_object + original[end:]


def compare_exact(original, current, selected):
    actual = load(current)
    require(actual.get('selected_source_sha256') == selected,
            'new report hashes differ from frozen current sources')
    expected = expected_bytes(original, selected)
    if current != expected:
        first = next((i for i, (a, b) in enumerate(zip(current, expected)) if a != b),
                     min(len(current), len(expected)))
        raise ValueError('report has a forbidden literal-byte difference at byte ' + str(first))
    return actual


def counts_and_coverage(report):
    require(list(report) == ['schema_version', 'protocol', 'selected_source_sha256',
                            'cases', 'numerical_sensitivity', 'summary'], 'unexpected report keys/order')
    require(report['schema_version'] == 2, 'schema changed')
    require(report['protocol']['native_entity_codec'] == {
        'crate': 'postcard', 'features': ['alloc'], 'framing': 'raw_exact_consumption',
        'wire_format_version': 1}, 'native codec changed')
    require(report['summary'] == {
        'collection_complete': True, 'worlds_attempted': 72, 'worlds_created': 72,
        'completed_worlds': 72, 'replay_qualification_pass': True,
        'primary_calibration_pass': False, 'primary_nonzero_cells': 4,
        'primary_nonzero_passes': 0, 'zero_target_cells': 8, 'zero_target_passes': 8},
        'summary does not preserve complete replay and the original FAIL')
    cases = report['cases']
    require(len(cases) == 24 and len(report['numerical_sensitivity']) == 24,
            'fixed matrix or sensitivity coverage changed')
    states = entities = native_bytes = state_words = telemetry_words = slices = 0
    primary = nonzero_passes = zero = zero_passes = 0
    identities = set()
    for index, case in enumerate(cases):
        cell = case['cell']
        key = (cell['gain'], cell['target_position_rad']['native_f32_bits'],
               cell['solver_small_steps'], cell['internal_pgs_passes'])
        require(key not in identities, 'duplicate cell ' + str(index)); identities.add(key)
        require(case['replay'] == {
            'native_state_exact_fresh_repeat': True, 'native_state_exact_on_off': True,
            'telemetry_exact_fresh_repeat': True, 'disabled_telemetry_absent': True},
            'literal replay qualification missing in cell ' + str(index))
        runs = case['runs']; require(len(runs) == 3, 'missing run')
        require([r['collector_enabled'] for r in runs] == [True, True, False], 'run order changed')
        for run in runs:
            require(run['world_created'] and run['completed_step'] and run['error'] is None,
                    'uncompleted or errored world')
            for field in ['initial', 'final_state']:
                state = run[field]; require(state is not None, 'missing native state')
                states += 1; state_words += len(state['canonical_words'])
                for entity in state['native_entities']:
                    require(all(type(x) is int and 0 <= x <= 255 for x in entity['bytes']),
                            'invalid native byte')
                    data = bytes(entity['bytes'])
                    require(sha(data) == entity['sha256'], 'native entity byte hash mismatch')
                    entities += 1; native_bytes += len(data)
                require(state == runs[0][field], 'ON/fresh/OFF native state differs')
            telemetry = run['telemetry']
            if run['collector_enabled']:
                require(telemetry == runs[0]['telemetry'] and telemetry is not None,
                        'fresh ON ordered telemetry differs')
                telemetry_words += len(telemetry['canonical_words'])
                require(len(telemetry['motors']) == 1, 'motor coverage changed')
                for motor in telemetry['motors']:
                    require(motor['full_step_coverage'], 'partial telemetry coverage')
                    require(len(motor['slices']) == cell['solver_small_steps'], 'slice count changed')
                    require([s['internal_slice_index'] for s in motor['slices']] ==
                            list(range(cell['solver_small_steps'])), 'slice order changed')
                    slices += len(motor['slices'])
            else:
                require(telemetry is None, 'disabled telemetry present')
        assessment = case['assessment']
        primary += int(assessment['primary_cell'])
        if cell['target_position_rad']['finite'] == 0:
            zero += 1; zero_passes += int(assessment['zero_target_motionless'] is True)
        else:
            nonzero_passes += int(assessment['first_slice_tolerance_pass'])
    require(primary == 4 and zero == zero_passes == 8, 'primary/zero coverage changed')
    return {'cells': 24, 'worlds': 72, 'native_states': states, 'native_entities': entities,
            'native_entity_bytes': native_bytes, 'native_state_canonical_words': state_words,
            'enabled_telemetry_runs': 48, 'ordered_telemetry_slices': slices,
            'telemetry_canonical_words': telemetry_words, 'primary_nonzero_passes': 0,
            'all_nonzero_tolerance_passes': nonzero_passes, 'zero_target_passes': zero_passes}


def bound_record(record):
    actual = artifact(record['path'], record['sha256'])
    require(actual['bytes'] == record['bytes'], 'artifact byte count mismatch')
    return actual


def timestamp(value):
    return datetime.datetime.fromisoformat(value.replace('Z', '+00:00'))


def audit(args):
    protocol_record = artifact(args.protocol, args.protocol_sha256)
    protocol = load(Path(args.protocol).read_bytes())
    require(protocol['reader_sha256'] == artifact(__file__)['sha256'], 'reader not frozen')
    require(protocol['baseline_schema2_sha256'] == BASELINE_SHA, 'baseline protocol changed')
    original_record = artifact(BASELINE, BASELINE_SHA)
    original = BASELINE.read_bytes()
    for row in protocol['immutable_published_artifacts']:
        bound_record(row)
    frozen_before = protocol['historical118_source_sha256_before']
    require(len(frozen_before) == 118, 'historical source scope changed')
    binding_record = artifact(args.binding, args.binding_sha256)
    binding = load(Path(args.binding).read_bytes())
    require(binding['status'] == 'FROZEN_BEFORE_AUTHORITATIVE_VENDOR_LINT_WORLDS', 'binding not frozen')
    require(binding['authoritative_worlds_executed_before_freeze'] == 0, 'freeze occurred after worlds')
    require(binding['base_commit'] == BASE and binding['protocol_sha256'] == args.protocol_sha256
            and binding['reader_sha256'] == protocol['reader_sha256'], 'binding provenance mismatch')
    owned = protocol['owned_vendor_source_sha256_before']
    require(len(owned) == 11 and binding['owned_vendor_source_sha256_before'] == owned,
            'owned source-before scope mismatch')
    after = binding['owned_vendor_source_sha256_after']
    require(set(after) == set(owned) and all(after[k] != owned[k] for k in owned),
            'expected eleven mechanical source changes not bound')
    files = binding['files']
    require(set(files) == set(frozen_before), 'current frozen 118 source paths changed')
    allowed_inputs = set(owned) | {'xtask/src/main.rs'}
    require({k for k in files if files[k] != frozen_before[k]} <= allowed_inputs,
            'unexpected frozen source change outside owned files and xtask report-path routing')
    for name, digest in files.items():
        require(artifact(REPO / name)['sha256'] == digest, 'frozen source is stale: ' + name)
    require(all(files[name] == digest for name, digest in after.items()), 'owned after hashes mismatch')
    selected = binding['selected_source_sha256_after']
    require(set(selected) == set(protocol['selected_source_sha256_before']) and
            all(files[name] == digest for name, digest in selected.items()), 'selected source binding mismatch')
    expected_bytes(original, selected)
    require(isinstance(binding['compiler'], str) and binding['compiler'] and
            isinstance(binding['cargo'], str) and binding['cargo'], 'compiler/cargo unbound')
    features = bound_record(binding['resolved_features'])
    binary = bound_record(binding['binary'])
    require('--locked' in binding['build_command'] and '--release' in binding['build_command'],
            'build mode is not bound locked release')
    collection_record = artifact(args.collection, args.collection_sha256)
    collection = load(Path(args.collection).read_bytes())
    require(collection['status'] == 'ACTUAL_VENDOR_LINT_72_WORLD_COLLECTION_COMPLETE' and
            collection['exit_code'] == 0 and collection['source_inputs_unchanged'] is True,
            'actual collection did not qualify')
    require(collection['binding_sha256'] == args.binding_sha256 and
            collection['report_sha256'] == args.report_sha256, 'collection artifact binding mismatch')
    require(timestamp(binding['registered_utc']) <= timestamp(collection['started_utc']) <=
            timestamp(collection['ended_utc']), 'freeze/collection timestamps out of order')
    stdout = bound_record(collection['stdout'])
    require(Path(stdout['path']).read_bytes() == EXPECTED_STDOUT, 'actual collector stdout differs')
    before_guard = bound_record(collection['source_guard_before'])
    after_guard = bound_record(collection['source_guard_after'])
    b = load(Path(before_guard['path']).read_bytes()); a = load(Path(after_guard['path']).read_bytes())
    require(b['files'] == a['files'] and all(b['files'].get(k) == v for k, v in files.items()),
            'collection source guard differs from frozen files')
    for name, digest in a['files'].items():
        require(artifact(REPO / name)['sha256'] == digest, 'current collection guard is stale: ' + name)
    new_record = artifact(args.report, args.report_sha256)
    report = compare_exact(original, Path(args.report).read_bytes(), selected)
    counts = counts_and_coverage(report)
    return {'status': 'PASS_LITERAL_VENDOR_LINT_CURRENT_REPORT_QUALIFICATION',
            'protocol': protocol_record, 'reader': artifact(__file__), 'baseline': original_record,
            'new_report': new_record, 'source_binary_binding': binding_record,
            'actual_collection': collection_record, 'actual_stdout': stdout,
            'compiler_features': features, 'frozen_binary': binary,
            'allowed_two_selected_hash_changes': {k: {'before': protocol['selected_source_sha256_before'][k],
                                                       'after': selected[k]} for k in sorted(ALLOWED)},
            'owned_eleven_before_after_hashes_qualified': True,
            'frozen_118_sources_current_and_collection_guard_unchanged': True,
            'all_other_literal_report_bytes_unchanged': True, 'coverage': counts,
            'summary': report['summary'], 'protocol_values_thresholds_schema_and_assessments_unchanged': True,
            'native_state_postcard_bytes_ieee_words_and_ordered_telemetry_literal_unchanged': True,
            'baseline_accuracy_outcome': 'FAIL, 0/4 primary; not improved or relabelled',
            'historical_candidate_admission': 'generation3 REJECTED, unchanged',
            'strict_vendor_clippy_rustdoc_qualification_claimed_by_this_reader': False,
            'old_report_current_source_verifier_qualification': 'separate actual stale-source exit1 required',
            'coupled_cause_or_maturity_or_hardware_gain_claim': False, 'reader_worlds_builds_tests': 0}


def self_test():
    raw = BASELINE.read_bytes(); require(sha(raw) == BASELINE_SHA, 'immutable baseline differs')
    doc = load(raw); counts = counts_and_coverage(doc)
    selected = dict(doc['selected_source_sha256'])
    for index, name in enumerate(sorted(ALLOWED)):
        selected[name] = str(index + 1) * 64
    synthetic = expected_bytes(raw, selected)
    compare_exact(raw, synthetic, selected)
    rejected = []
    def fails(name, callback):
        try:
            callback()
        except (ValueError, KeyError):
            rejected.append(name)
        else:
            raise ValueError('tamper not rejected: ' + name)
    third = next(k for k in selected if k not in ALLOWED)
    wrong = dict(selected); wrong[third] = '3' * 64
    fails('third_selected_hash', lambda: expected_bytes(raw, wrong))
    missing = dict(selected); missing.pop(third)
    fails('missing_selected_path', lambda: expected_bytes(raw, missing))
    fails('forged_accuracy_PASS', lambda: compare_exact(raw, synthetic.replace(
        b'"primary_calibration_pass": false', b'"primary_calibration_pass": true', 1), selected))
    fails('numeric_signed_zero_spelling', lambda: compare_exact(raw, synthetic.replace(
        b'"finite": 0.0', b'"finite": -0.0', 1), selected))
    fails('native_byte_tamper', lambda: compare_exact(raw, synthetic.replace(
        b'"bytes": [\n                0,', b'"bytes": [\n                1,', 1), selected))
    fails('canonical_word_tamper', lambda: compare_exact(raw, synthetic.replace(
        b'"canonical_words": [\n            ', b'"canonical_words": [\n            42,\n            ', 1), selected))
    fails('formatting_changed', lambda: compare_exact(raw, synthetic + b' ', selected))
    fails('duplicate_JSON_key', lambda: load(b'{"x":1,"x":2}'))
    return {'status': 'PASS_PURE_READER_SELF_TEST', 'synthetic_allowed_two_hash_changes_accepted': True,
            'tamper_classes_rejected': rejected, 'immutable_baseline_structure': counts,
            'worlds_builds_physics_tests': 0, 'new_report_audit_executed': False}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--self-test', action='store_true')
    for name in ['protocol', 'protocol-sha256', 'binding', 'binding-sha256', 'report',
                 'report-sha256', 'collection', 'collection-sha256', 'output']:
        parser.add_argument('--' + name)
    args = parser.parse_args()
    if args.self_test:
        print(json.dumps(self_test(), indent=2)); return 0
    require(all(getattr(args, name.replace('-', '_')) is not None for name in
                ['protocol', 'protocol-sha256', 'binding', 'binding-sha256', 'report',
                 'report-sha256', 'collection', 'collection-sha256', 'output']), 'missing audit arguments')
    output = Path(args.output).resolve()
    require(output.is_relative_to(OUT.resolve()), 'reader output must stay outside checkout in its qualification directory')
    try:
        receipt = audit(args); code = 0
    except Exception as error:
        receipt = {'status': 'FAIL_VENDOR_LINT_CURRENT_REPORT_QUALIFICATION',
                   'error_type': type(error).__name__, 'error': str(error),
                   'reader_worlds_builds_tests': 0, 'no_failure_reinterpretation': True}
        code = 1
    receipt['audited_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    output.write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(artifact(output))); return code


if __name__ == '__main__':
    sys.exit(main())
