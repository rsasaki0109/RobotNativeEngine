"""Read required release receipts; require whole stdout and actual process exits."""
import argparse
import difflib
from pathlib import Path
from release_regression_common import (BASE, EXAMPLES, HERE, REPO, expected_argv,
    guard_tools, load, now, physical, ref, save_new, sha, source_files)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tool-freeze-sha', required=True)
    parser.add_argument('--baseline-only', action='store_true')
    parser.add_argument('--source-freeze', type=Path)
    parser.add_argument('--execution-check', type=Path)
    parser.add_argument('--results', type=Path, default=HERE / 'actual-results')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    criteria = guard_tools(args.tool_freeze_sha)
    baseline = []
    for (package, example), status in zip(EXAMPLES, criteria['actual_ok_lines']):
        stdout = BASE / (example + '.stdout')
        normalized = physical(stdout.read_bytes())
        assert normalized == (BASE / (example + '.physical.txt')).read_bytes()
        assert status in stdout.read_bytes().decode('utf-8').splitlines()
        receipt = load(BASE / (example + '-result.json'))
        assert receipt['command'] == expected_argv(package, example)
        assert receipt['exit_code'] == 0
        baseline.append({'example': example, 'stdout': ref(stdout),
            'physical_stdout': ref(BASE / (example + '.physical.txt')),
            'receipt': ref(BASE / (example + '-result.json'))})
    if args.baseline_only:
        save_new(args.output, {'status': 'BASELINE_READONLY_PASS', 'audited_utc': now(),
            'baseline': baseline, 'new_release_programs_executed': 0,
            'planned18_coupled_world_count_contribution': 0,
            'planned72_scalar_world_count_contribution': 0})
        return
    assert args.source_freeze and args.execution_check
    frozen = load(args.source_freeze)
    assert frozen['phase'] == 'final-restored-default'
    assert frozen['tool_freeze_sha256'] == args.tool_freeze_sha
    assert frozen['files'] == source_files(), 'final restored source freeze changed'
    check = args.execution_check
    started, terminal = load(check / 'started.json'), load(check / 'terminal.json')
    assert started['command'] == ['python3', str(HERE / 'run_release_regressions.py')]
    assert started['cwd'] == str(REPO)
    assert terminal['source_inputs_unchanged'] is True and terminal['changed_inputs'] == []
    before, after = load(check / 'inputs-before.json'), load(check / 'inputs-after.json')
    assert before['files'] == after['files'] and before['count'] == after['count']
    for name, digest in before['files'].items():
        assert sha(REPO / name) == digest, name
    for name, field in [('command.log', 'log_sha256'), ('inputs-before.json', 'before_manifest_sha256'),
                        ('inputs-after.json', 'after_manifest_sha256')]:
        assert sha(check / name) == terminal[field]
    rows = load(args.results / 'results.json')
    assert 0 < len(rows) <= len(EXAMPLES)
    log_rows = [load_row(line) for line in (check / 'command.log').read_text().splitlines()
                if line.startswith('{')]
    assert rows == log_rows
    evidence = []
    for row, (package, example), status in zip(rows, EXAMPLES, criteria['actual_ok_lines']):
        assert row['example'] == example and row['command'] == expected_argv(package, example)
        process = load(args.results / (example + '-process.json'))
        assert process['command'] == row['command'] and process['pid'] > 0
        stdout, stderr = args.results / (example + '.stdout'), args.results / (example + '.stderr')
        original = stdout.read_bytes()
        candidate = physical(original)
        original_baseline = (BASE / (example + '.stdout')).read_bytes()
        expected = physical(original_baseline)
        generated = args.results / (example + '.physical.txt')
        assert candidate == generated.read_bytes()
        matches = candidate == expected
        actual_ok = status in original.decode('utf-8').splitlines()
        assert row['physical_stdout_matches_baseline'] is matches
        assert row['has_ok'] is (': ok' in candidate.decode('utf-8'))
        assert row['baseline_stdout_sha256'] == sha(BASE / (example + '.stdout'))
        assert row['physical_stdout_sha256'] == sha(generated)
        evidence.append({'example': example, 'actual_argv': row['command'],
            'actual_exit_code': row['exit_code'], 'actual_ok_line_present': actual_ok,
            'whole_stdout_timing_only_literal_match': matches,
            'stdout': ref(stdout), 'stderr': ref(stderr), 'physical_stdout': ref(generated),
            'process_receipt': ref(args.results / (example + '-process.json')),
            'unified_physical_diff': list(difflib.unified_diff(expected.decode().splitlines(),
                candidate.decode().splitlines(), fromfile='baseline', tofile='current', lineterm=''))})
    expected_exit = 0 if len(evidence) == 6 and all(x['actual_exit_code'] == 0
        and x['actual_ok_line_present'] and x['whole_stdout_timing_only_literal_match']
        for x in evidence) else 1
    assert terminal['exit_code'] == expected_exit
    guard_tools(args.tool_freeze_sha)
    assert frozen['files'] == source_files()
    save_new(args.output, {'status': 'PASS' if expected_exit == 0 else 'FAIL',
        'audited_utc': now(), 'wrapper_actual_exit_code': terminal['exit_code'],
        'source_freeze': ref(args.source_freeze), 'execution_started': ref(check / 'started.json'),
        'execution_terminal': ref(check / 'terminal.json'), 'execution_log': ref(check / 'command.log'),
        'raw_results': ref(args.results / 'results.json'), 'tool_freeze_sha256': args.tool_freeze_sha,
        'all6_required_complete': len(evidence) == 6, 'baseline': baseline, 'evidence': evidence,
        'selected_source_count': frozen['count'], 'wrapper_selected_input_count': before['count'],
        'comparison': 'Full original UTF-8 stdout after exactly the three frozen timing regex substitutions; no numerical tolerance or line omission.',
        'world_accounting': 'Separate required example regression collection; not part of planned18 coupled or72 scalar authoritative worlds.',
        'planned18_coupled_world_count_contribution': 0, 'planned72_scalar_world_count_contribution': 0,
        'reader_native_worlds_executed': 0,
        'scope': 'Printed rounded metrics are compared literally; unprinted precision and exhaustive compiler closure are not inferred.'})
    raise SystemExit(expected_exit)


def load_row(text):
    import json
    return json.loads(text)


if __name__ == '__main__':
    main()
