#!/usr/bin/env python3
"""Read actual release receipts and apply frozen admission rules; no executions."""
import ast
from datetime import datetime, timezone
import difflib
import hashlib
import json
from pathlib import Path
import re

ROOT = Path('/workspace/.rne-kick-demo/single-joint-motor-calibration')
GEN3 = ROOT / 'diagonal-intervention-generation-3'
HERE = GEN3 / 'independent-oracle'
REPO = Path('/workspace/RobotNativeEngine')
BASE = Path('/workspace/.rne-kick-demo/g1-motor-torque-diagnostics/baseline/examples')
RUN = ROOT / 'checks/gen3-release-regressions'
RESULTS = GEN3 / 'release-regressions'
EXPRESSIONS = [(r',\s*[0-9.]+ us per step', ''),
               (r'\d+ env steps/s \(\d+ sim steps/s\)', '<throughput>'),
               (r'parallel speedup [0-9.]+x', 'parallel speedup <timing>')]
EXPECTED = [('go2_contact_terrain', '133_go2_contact_terrain'), ('go2_terrain_trot', '134_go2_terrain_trot'),
            ('go2_feedback_trot', '135_go2_feedback_trot'), ('go2_parallel_rl', '136_go2_parallel_rl'),
            ('go2_native_backend', '137_go2_native_backend'), ('heavy_grasp', '138_heavy_grasp')]


def load(path):
    return json.loads(Path(path).read_bytes())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def ref(path):
    path = Path(path)
    return {'path': str(path), 'bytes': path.stat().st_size, 'sha256': sha(path)}


def physical(text):
    for pattern, replacement in EXPRESSIONS:
        text = re.sub(pattern, replacement, text)
    return text


def guard_sources():
    source_path = GEN3 / 'source-compiler-protocol.json'
    binding_path = GEN3 / 'frozen-binary-binding.json'
    binding = load(binding_path)
    assert sha(binding_path) == 'c7cccc666b5208fbf99e7b0962a3e65a20839ca2ffc689a8182c1002a458f29b'
    assert sha(source_path) == binding['source_compiler_protocol']['sha256'] == '16c085e73638c360126c57d05bd00ee67f3173c5a790ea17b4af528b39642aa8'
    source = load(source_path)
    assert len(source['files']) == binding['selected_source_count'] == 118
    for name, digest in source['files'].items():
        assert sha(REPO / name) == digest, name
    assert sha(binding['binary_path']) == binding['binary_sha256'] == '329215b1507993c1ce604c6b9a5089912c241c2e46d56d8cea969cde37bbdb2a'
    return {'source_freeze': ref(source_path), 'frozen_binding': ref(binding_path),
            'frozen_binary': ref(binding['binary_path']), 'all118_current_source_bindings_match': True}


def parse138(text):
    rows = []
    condition = None
    for line in text.splitlines():
        match = re.fullmatch(r'(\d+) kg cube, (\d+) N per finger \((held|under-squeezed); holding needs (\d+) N\):', line)
        if match:
            mass, squeeze, mode, needed = match.groups()
            condition = {'mass_kg': int(mass), 'per_finger_force_n': int(squeeze),
                         'mode': mode, 'nominal_holding_force_n': int(needed)}
        match = re.fullmatch(r'\s*(native|rapier): lifted\s+([-+0-9.e]+) m \(palm ([-+0-9.e]+) m\), slip\s+([-+0-9.e]+) m, tilt\s+([-+0-9.e]+) rad', line)
        if match:
            assert condition is not None
            backend, lift, palm, slip, tilt = match.groups()
            strings = {'lift_m': lift, 'palm_lift_m': palm, 'slip_m': slip, 'tilt_rad': tilt}
            rows.append({'condition': dict(condition), 'backend': backend,
                         'literal_physical_line': line,
                         'displayed_metric_strings': strings,
                         'displayed_metric_values': {key: float(value) for key, value in strings.items()}})
    assert len(rows) == 12 and [x['backend'] for x in rows] == ['native', 'rapier'] * 6
    return rows


def main():
    before_guard = guard_sources()
    protocol_path = HERE / 'preregistered-diagonal-intervention-protocol.json'
    protocol = load(protocol_path)
    assert sha(protocol_path) == '72fb721272e7f42c38e67560f551dfe041f9b7102219f546906596fcd4868457'
    assert protocol['release_133_138_admission']['unchanged_timing_exclusions'] == [{'regex': a, 'replacement': b} for a, b in EXPRESSIONS]
    assert 'full physical stdout must exactly match' in protocol['release_133_138_admission']['required']
    assert 'rollback' in protocol['freeze_order'][5] or 'roll back' in protocol['admission']
    assert 'without' not in protocol['release_133_138_admission']['required']
    for entry in protocol['immutable_prior_bindings']:
        assert sha(entry['path']) == entry['sha256']
    freeze_path = HERE / 'pre-world-independent-tool-freeze.json'
    freeze = load(freeze_path)
    for entry in freeze['frozen_tools']:
        assert sha(entry['path']) == entry['sha256']
    runner = GEN3 / 'run_release_regressions.py'
    reader_freeze_path = GEN3 / 'release-regression-reader-freeze.json'
    reader_freeze = load(reader_freeze_path)
    original_runner = Path(reader_freeze['source'])
    assert sha(original_runner) == reader_freeze['source_sha256']
    assert sha(runner) == reader_freeze['reader_sha256']
    assert original_runner.read_bytes().replace(b"'codec-migration-generation-2'", b"'diagonal-intervention-generation-3'") == runner.read_bytes()
    tree = ast.parse(runner.read_bytes())
    function = next(x for x in tree.body if isinstance(x, ast.FunctionDef) and x.name == 'physical')
    extracted = []
    for node in ast.walk(function):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == 'sub':
            extracted.append((ast.literal_eval(node.args[0]), ast.literal_eval(node.args[1])))
    assert extracted == EXPRESSIONS
    rows = load(RESULTS / 'results.json')
    started, terminal = load(RUN / 'started.json'), load(RUN / 'terminal.json')
    assert started['command'] == ['python3', str(runner)] and started['cwd'] == str(REPO)
    assert started['cargo_build_jobs'] == '2'
    assert terminal['exit_code'] == 1 and terminal['source_inputs_unchanged'] is True and terminal['changed_inputs'] == []
    before, after = load(RUN / 'inputs-before.json'), load(RUN / 'inputs-after.json')
    assert before['files'] == after['files'] and before['count'] == after['count'] == len(before['files']) == 2289
    for name, digest in before['files'].items():
        assert sha(REPO / name) == digest, name
    for filename, field in [('command.log', 'log_sha256'), ('inputs-before.json', 'before_manifest_sha256'), ('inputs-after.json', 'after_manifest_sha256')]:
        assert sha(RUN / filename) == terminal[field]
    assert sha(RUN / 'command.log') == 'bcaa1d6415e53dba0b82c26aa9086b732192547a365882908ebaaddcc4dc916f'
    log_rows = [json.loads(x) for x in (RUN / 'command.log').read_text().splitlines() if x.startswith('{')]
    assert rows == log_rows and len(rows) == 6
    baseline_guard_path = ROOT / 'independent-ci-audit/fresh-baseline-guard.json'
    baseline_guard = load(baseline_guard_path)
    preserved = {x['example']: x['prior_baseline_physical_output'] for x in baseline_guard['prior_required_release_physical_receipts']}
    evidence = []
    for row, (package, name) in zip(rows, EXPECTED):
        assert row['example'] == name and row['command'] == ['cargo', 'run', '--locked', '--release', '-p', package, '--example', name]
        assert row['exit_code'] == 0 and row['has_ok'] is True
        stdout, stderr = RESULTS / (name + '.stdout'), RESULTS / (name + '.stderr')
        baseline_stdout, baseline_physical = BASE / (name + '.stdout'), Path(preserved[name]['path'])
        generated_physical = RESULTS / (name + '.physical.txt')
        transformed = physical(stdout.read_text()).encode()
        old_transformed = physical(baseline_stdout.read_text()).encode()
        assert transformed == generated_physical.read_bytes()
        assert old_transformed == baseline_physical.read_bytes() and ref(baseline_physical) == preserved[name]
        exact_match = transformed == old_transformed
        assert row['physical_stdout_matches_baseline'] is exact_match
        assert row['physical_stdout_sha256'] == sha(generated_physical)
        assert row['baseline_stdout_sha256'] == sha(baseline_stdout)
        assert ': ok' in transformed.decode()
        assert load(RESULTS / (name + '-process.json'))['command'] == row['command']
        evidence.append({'example': name, 'actual_command': row['command'], 'actual_exit_code': row['exit_code'],
                         'has_ok': True, 'whole_physical_stdout_literal_match': exact_match,
                         'stdout': ref(stdout), 'stderr': ref(stderr), 'physical_stdout': ref(generated_physical),
                         'baseline_stdout': ref(baseline_stdout), 'preserved_baseline_physical': ref(baseline_physical),
                         'child_process_receipt': ref(RESULTS / (name + '-process.json')),
                         'unified_physical_diff': list(difflib.unified_diff(old_transformed.decode().splitlines(), transformed.decode().splitlines(), fromfile='baseline', tofile='candidate', lineterm=''))})
    assert [x['whole_physical_stdout_literal_match'] for x in evidence] == [True] * 5 + [False]
    old138 = parse138(physical((BASE / '138_heavy_grasp.stdout').read_text()))
    new138 = parse138(physical((RESULTS / '138_heavy_grasp.stdout').read_text()))
    differences = []
    native_rows = []
    rapier_rows = []
    for old, new in zip(old138, new138):
        assert old['condition'] == new['condition'] and old['backend'] == new['backend']
        changed = old['literal_physical_line'] != new['literal_physical_line']
        item = {'condition': old['condition'], 'backend': old['backend'], 'physical_line_literal_unchanged': not changed,
                'baseline': old, 'candidate': new,
                'changed_displayed_metrics': {key: {'baseline': old['displayed_metric_strings'][key], 'candidate': new['displayed_metric_strings'][key]}
                                             for key in old['displayed_metric_strings']
                                             if old['displayed_metric_strings'][key] != new['displayed_metric_strings'][key]}}
        if old['backend'] == 'native':
            assert not changed
            native_rows.append(item)
        else:
            rapier_rows.append(item)
            if changed:
                differences.append(item)
    assert len(native_rows) == len(rapier_rows) == 6 and len(differences) == 4
    numeric_proof_path = HERE / 'independent-generation3-exact-fraction-audit.json'
    numeric = load(numeric_proof_path)
    assert sha(numeric_proof_path) == '7311d98d43470b1bc0ef8cf120307c6417dc213719238484872c650077770611'
    assert numeric['primary_numerical_outcome']['status'] == 'PASS' and numeric['primary_numerical_outcome']['passed'] == 4
    after_guard = guard_sources()
    assert before_guard == after_guard
    for name, digest in before['files'].items():
        assert sha(REPO / name) == digest, name
    output = {'status': 'PASS_INDEPENDENT_ADMISSION_REJECTION_AUDIT',
              'admission_decision': 'REJECTED_DO_NOT_PROMOTE_ROLL_BACK_CANDIDATE',
              'audited_utc': datetime.now(timezone.utc).isoformat(), 'auditor': ref(__file__),
              'reason': 'Frozen whole physical stdout gate fails for example138 despite program exit0 and ok. Root wrapper correctly exits1. Preserve numerical primary4/4 success separately; it does not override failed compatibility.',
              'actual_release_program_exit_codes': [x['exit_code'] for x in rows],
              'actual_release_programs_has_ok': [x['has_ok'] for x in rows],
              'physical_stdout_literal_matches': 5, 'required_physical_stdout_literal_matches': 6,
              'wrapper_actual_exit_code': terminal['exit_code'], 'protocol': ref(protocol_path),
              'protocol_required_literal_physical_preservation_and_rollback': True,
              'exact_existing_three_timing_exclusions': [{'regex': a, 'replacement': b} for a, b in EXPRESSIONS],
              'reader': ref(runner), 'reader_freeze': ref(reader_freeze_path), 'source_runner': ref(original_runner),
              'reader_only_output_directory_changed': True, 'baseline_guard': ref(baseline_guard_path),
              'started': ref(RUN / 'started.json'), 'terminal': ref(RUN / 'terminal.json'),
              'actual_terminal': terminal, 'log': ref(RUN / 'command.log'),
              'inputs_before': ref(RUN / 'inputs-before.json'), 'inputs_after': ref(RUN / 'inputs-after.json'),
              'all2289_execution_inputs_before_after_current_literal_match': True,
              'all118_candidate_source_inputs_current_literal_match_before_after': True,
              'source_guard': before_guard, 'actual_results': ref(RESULTS / 'results.json'),
              'all6_release_receipts': evidence, 'example138_all6_native_rows_literal_unchanged': native_rows,
              'example138_all6_rapier_rows': rapier_rows, 'example138_all4_differing_rapier_rows': differences,
              'independent_primary4of4_pass_proof_retained': ref(numeric_proof_path),
              'displayed_metrics_scope': 'Numbers are displayed rounded stdout values; no unrecorded native precision inferred. Native6 rows unchanged, Rapier4 changed and2 unchanged after timing exclusion.',
              'limitations': ['No new physics, builds, tests, oracle rerun, source changes, rollback, retuning, process control or promotion by this auditor',
                              'No causal claim for coupled heavy-grasp differences or G1 kick recovery',
                              'Full CI and AccelerationBased baseline/candidate replay are not qualified by this receipt'],
              'auditor_native_worlds_builds_tests_checkout_edits_process_control': 0}
    out = HERE / 'admission-rejected-release133-138-independent-proof.json'
    assert not out.exists()
    out.write_text(json.dumps(output, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'output': ref(out), 'status': output['status'], 'admission': output['admission_decision'],
                      'all6_exit_codes': output['actual_release_program_exit_codes'],
                      'physical_stdout_literal_matches': 5, 'native138_unchanged_rows': 6,
                      'all4_differing_rapier_rows': [{'condition': x['condition'], 'changes': x['changed_displayed_metrics']} for x in differences]}, indent=2))


if __name__ == '__main__':
    main()
