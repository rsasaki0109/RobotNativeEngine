"""Root-only literal checkout export after independent scientific qualification."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import shutil
import subprocess

REC = Path('/workspace/.rne-onboarding/recovery-20261012')
REPO = Path('/workspace/RobotNativeEngine')
PUBLIC = REPO / 'docs/evidence/motor-construction-v1'
CAP = 1 << 20


def desc(path):
    data = path.read_bytes()
    return {'path': str(path), 'sha256': hashlib.sha256(data).hexdigest(), 'size_bytes': len(data)}


def write_new(path, data):
    with path.open('xb') as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())


def expected(path, digest):
    record = desc(path)
    assert record['sha256'] == digest, str(path)
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--plan-sha256', required=True)
    parser.add_argument('--stage', type=Path, required=True)
    parser.add_argument('--readme', type=Path, required=True)
    args = parser.parse_args()
    assert not PUBLIC.exists(), 'Never replace an earlier or partial export.'
    assert REC in args.stage.resolve().parents
    plan_descriptor = expected(args.plan, args.plan_sha256)
    plan = json.loads(args.plan.read_bytes())
    assert plan['public_target'] == str(PUBLIC.relative_to(REPO))
    assert plan['actual_six_audit']['sha256'] == 'a3f3e60fb199390e1dd761f6e96357df684bbcb0bdc996d5ec24a0745c3e107c'
    assert len(plan['actual_six_whole_raw']) == 6
    verifier = args.stage / 'verify_literals.py'
    verifier_rows = [p for p in plan['literal_payloads'] if p['export_path'] == 'verify_literals.py']
    assert len(verifier_rows) == 1
    assert desc(verifier)['sha256'] == verifier_rows[0]['sha256']
    assert desc(verifier)['size_bytes'] == verifier_rows[0]['size_bytes']
    subprocess.run(['/usr/bin/python3', '-B', str(verifier), '--root', str(args.stage)], check=True)
    mapping = json.loads((args.stage / 'origin-map.json').read_bytes())
    aliases = {row[0]: row[3] for row in mapping['rows']}
    six_review = expected(REC / 'next-motor-construction-six/independent-postcollection-review/independent-postcollection-review.json',
                          'fbbfba0beead844fa1b217c83070e864e81ef22f2e20cf337701143377623bbe')
    transition_review = expected(REC / 'next-motor-source-transition-72/independent-postcollection-review/independent-postcollection-source-transition-review.json',
                                 '12b08d6018456e9b2a9fbf5959da8ad6e79f2dc3714f602428308990549d060d')
    cases = ['negative_on', 'negative_fresh_on', 'negative_off', 'positive_on', 'positive_fresh_on', 'positive_off']
    expected_raw_paths = {str(REC / 'next-motor-construction-six/collection' / case / 'raw.jsonl.gz') for case in cases}
    actual_raw_paths = [row['path'] for row in plan['actual_six_whole_raw']]
    assert len(set(actual_raw_paths)) == 6 and set(actual_raw_paths) == expected_raw_paths
    ledger_path = REC / 'next-motor-construction-six/independent-postcollection-review/retained-result-and-source-hash-ledger.json'
    expected(ledger_path, '126911554312497a807280e88bb52ba014f1bf3cc6b78774cb7c7ced74c1d895')
    ledger = json.loads(ledger_path.read_bytes())
    for row in plan['actual_six_whole_raw']:
        matches = [entry for entry in ledger if entry['actual']['path'] == row['path']]
        assert len(matches) == 1 and matches[0]['actual'] == matches[0]['expected']
        independently_reviewed = matches[0]['actual']
        assert row['sha256'] == independently_reviewed['sha256'] and row['size_bytes'] == independently_reviewed['bytes']
    for item in [six_review, transition_review, *plan['actual_six_whole_raw'], plan['actual_six_audit']]:
        assert item['path'] in aliases, 'Required whole evidence absent from public map.'
        assert desc(Path(item['path'])) == item, 'Retained qualification bytes changed.'
    local_ci = json.loads((REC / 'next-companion-validation/full-ci/actual.json').read_bytes())
    assert local_ci['original_exit_code'] == 0 and local_ci['source_guard_pass']
    readme = args.readme.read_bytes()
    assert len(readme) <= CAP and b'Final map' not in readme and b'PUBLIC_ALIAS_PENDING' not in readme
    # Whole literal stage is copied first; its provisional README is preserved externally.
    shutil.copytree(args.stage, PUBLIC, symlinks=False)
    (PUBLIC / 'README.md').unlink()
    write_new(PUBLIC / 'README.md', readme)
    write_new(PUBLIC / 'export-plan.json', args.plan.read_bytes())
    write_new(PUBLIC / 'checkout-export.py', Path(__file__).read_bytes())
    for original, public_name in [('independent-current-source-review.json', 'source-review.json'),
                                  ('current-source-and-diff-ledger.json', 'source-review-ledger.json'),
                                  ('inspect_source_preservation.py', 'source-review-tool.py'),
                                  ('tracked-current-diff.patch', 'source-review-diff.patch')]:
        write_new(PUBLIC / public_name, (REC / 'next-companion-final-source-review' / original).read_bytes())
    media = expected(REPO / 'docs/media/position-motor-calibration-motor-construction-source.json',
                     'bf6a721ca92d609c2e80df2e3b7f1bddeb79fcb5974934351d615a88f18db662')
    inventory = []
    paths = []
    for path in sorted(PUBLIC.rglob('*')):
        assert not path.is_symlink(), 'No exported symlinks.'
        if path.is_file():
            record = desc(path)
            relative = str(path.relative_to(REPO))
            assert relative.isascii() and len(relative) <= 166 and record['size_bytes'] <= CAP
            inventory.append({'repository_path': relative, 'sha256': record['sha256'], 'size_bytes': record['size_bytes']})
            paths.append(relative.casefold())
    assert len(set(paths)) == len(paths)
    source_files = ['Cargo.toml', 'Cargo.lock', 'xtask/src/main.rs', 'CHANGELOG.md', 'docs/DEMOS.md',
                    'docs/architecture/016_contact_dynamics.md', 'examples/141_position_motor_calibration/README.md',
                    'examples/143_motor_construction/README.md', 'examples/143_motor_construction/.gitignore',
                    'third_party/rapier3d/RNE_PATCH.md']
    metadata = {
        'kind': 'rne_motor_construction_publication_manifest', 'schema_version': 1,
        'status': 'QUALIFIED_EVIDENCE_PREPARED_FOR_PR',
        'qualification_scope': 'Captured state and declared source-ordered scalar normal/zero checks, not motor accuracy or hardware validation.',
        'git_source_base': '3be26ef0bd6967cf3e0b466a5b63b11795f40feb',
        'literal_export_plan': {**plan_descriptor, 'repository_path': 'docs/evidence/motor-construction-v1/export-plan.json'},
        'origin_map': desc(PUBLIC / 'origin-map.json'),
        'omitted_regenerable_copies': desc(PUBLIC / 'omitted-copies.json'),
        'byte_verifier': desc(PUBLIC / 'verify_literals.py'),
        'fixed_six': {'world_attempts': 6, 'scientific_reader_invocations': 1,
                      'constructor_checks': 128, 'coordinate_checks': 128, 'companion_events': 1028, 'stage_v1_events': 3328,
                      'result': 'PASS_COMPLETE_DECLARED_SCOPE', 'whole_raw': plan['actual_six_whole_raw'],
                      'whole_audit': plan['actual_six_audit'], 'independent_postcollection_review': six_review,
                      'original_native_exit_codes': [0, 0, 0, 0, 0, 0], 'original_coordinator_exit_code': 0,
                      'original_checked_outer_exit_code': 0, 'original_reader_exit_code': 0},
        'source_transition_72': {'world_attempts': 72, 'native_collector_processes': 1,
                                 'literal_comparison': 'PASS_SOURCE_OBJECT_ONLY', 'calibration': 'FAIL_0_OF_4',
                                 'zero_target_passes': '8_OF_8', 'original_native_exit_code': 2,
                                 'original_outer_exit_code': 0, 'loaded_report_exit_codes': [0, 2, 1],
                                 'complete_primary_report': media, 'independent_postcollection_review': transition_review},
        'local_validation_full_ci': desc(REC / 'next-companion-validation/full-ci/actual.json'),
        'publication_source_and_docs': [desc(REPO / name) for name in source_files],
        'whole_large_or_native_files_not_distributed': plan['excluded_whole_large_or_native'],
        'inline_inventory_excluding_this_manifest': inventory,
        'inventory_authentication': 'Review this manifest and origin-map through their committed Git blobs; the portable verifier alone does not authenticate either.',
        'no_new_release_publication': True, 'native_binaries_publicly_available': False,
        'hardware_validation': False, 'independent_external_user_validation': False,
        'runtime_float_mode_measured': False, 'complete_compiler_closure': False,
        'full_private_solver_cache_equivalence': False, 'physical_cause_or_candidate_adoption': False,
        'historical_source_guards': 'Captured before postcollection documentation and the one xtask report-path update; not a claim about the entire final checkout.'
    }
    data = (json.dumps(metadata, indent=2, sort_keys=True) + '\n').encode()
    assert len(data) <= CAP
    write_new(PUBLIC / 'publication-manifest.json', data)
    for path in PUBLIC.rglob('*'):
        if path.is_file():
            with path.open('rb') as stream:
                os.fsync(stream.fileno())
    directories = [p for p in PUBLIC.rglob('*') if p.is_dir()] + [PUBLIC, PUBLIC.parent]
    for directory in sorted(directories, key=lambda p: len(p.parts), reverse=True):
        handle = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(handle)
        finally:
            os.close(handle)
    subprocess.run(['/usr/bin/python3', '-B', str(PUBLIC / 'verify_literals.py'), '--root', str(PUBLIC)], check=True)
    print(json.dumps({'publication_manifest': desc(PUBLIC / 'publication-manifest.json'),
                      'public_files': len(inventory) + 1, 'scientific_worlds_launched': 0}, sort_keys=True))


if __name__ == '__main__':
    main()
