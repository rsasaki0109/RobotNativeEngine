"""Read-only NEW120 byte/provenance and direct-label review, never a scientific audit."""
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import zlib

OWN = Path(__file__).resolve().parent
PREP = OWN.parent
STUDY = PREP.parent / 'next-motor-physical-120'
ROOT = Path('/workspace/RobotNativeEngine')
WRAPPER_SHA = '6755bb5d73294fe67353df7ff5ab555314ecd6d425e6343f411132942d35b20c'


def canonical(v): return json.dumps(v, sort_keys=True, separators=(',', ':'), ensure_ascii=True).encode()


def descriptor(path):
    path = Path(path); raw = path.read_bytes()
    return {'path': str(path), 'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest()}


def bound(d): assert descriptor(d['path']) == d


def pairs(items):
    out = {}
    for k, v in items:
        if k in out: raise ValueError('duplicate JSON key')
        out[k] = v
    return out


def parse(raw):
    return json.loads(raw, object_pairs_hook=pairs,
                      parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))


def load(path): return parse(Path(path).read_bytes())


def save(name, v):
    with (OWN / name).open('xb') as f:
        f.write(json.dumps(v, sort_keys=True, indent=2).encode() + b'\n'); f.flush(); os.fsync(f.fileno())
    fd = os.open(OWN, os.O_RDONLY | os.O_DIRECTORY)
    try: os.fsync(fd)
    finally: os.close(fd)


def utc(value):
    result = datetime.fromisoformat(value); assert result.tzinfo is not None; return result


def ungzip(path):
    raw = Path(path).read_bytes(); dec = zlib.decompressobj(16 + zlib.MAX_WBITS)
    restored = dec.decompress(raw, 32 * 1024 * 1024 + 1)
    assert dec.eof and not dec.unused_data and not dec.unconsumed_tail
    assert len(restored) <= 32 * 1024 * 1024
    return restored


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT,
                                  env={**os.environ, 'GIT_OPTIONAL_LOCKS': '0'})


def main():
    result = {'status': 'FAILED_POSTCOLLECTION_BYTE_PROVENANCE_REVIEW',
              'new_native_worlds': 0, 'scientific_reader_executions': 0, 'SI_or_source_arithmetic_rederivations': 0,
              'old_numerical_dataset_reads': 0, 'cargo_commands': 0, 'github_calls': 0, 'checkout_edits': 0}
    try:
        wrapper_path = STUDY / 'admitted-spec.json'
        assert descriptor(wrapper_path)['sha256'] == WRAPPER_SHA
        wrapper = load(wrapper_path); bound(wrapper['candidate']); bound(wrapper['admission'])
        spec = load(wrapper['candidate']['path']); admission = load(wrapper['admission']['path'])
        assert wrapper['candidate']['sha256'] == admission['reviewed_candidate_sha256'] == 'e81d000fecaaa612aed14b72dfe639679da45448b5f4cfc5d62b4532152a40df'
        assert admission['status'] == 'PASS_PRE_DATA_PHYSICAL20_SOURCE_PROTOCOL_DRIVER_READER'
        for d in spec['frozen_inputs']: bound(d)
        for d in spec['binaries'].values(): bound(d)
        before = [descriptor(p) for p in sorted(STUDY.rglob('*')) if p.is_file()]
        source_before = [descriptor(d['path']) for d in spec['frozen_inputs']]
        save('whole-study-file-ledger-before.json', before)
        save('frozen-source-inputs-before.json', source_before)
        master = load(STUDY / 'master/master-terminal.json')
        assert master['status'] == 'completed_fixed120_collection_and_one_reader'
        assert master['error'] is None and master['source_inputs_unchanged'] is True and master['retries'] == 0 and not master['resumed']
        assert master['registered_wrapper_sha256'] == WRAPPER_SHA and master['reader_launch_outcome'] == 'known_started_exit_observed'
        root_receipt = load(STUDY / 'root-original-launch.json')
        assert root_receipt['original_exit'] == root_receipt['actual_tool_result']['exit_code'] == 0 and root_receipt['retries'] == 0
        bound(root_receipt['registered_wrapper']); assert root_receipt['registered_wrapper']['sha256'] == WRAPPER_SHA
        assert canonical(parse((STUDY / 'root-stdout.log').read_bytes().splitlines()[-1])) == canonical(master)
        assert (STUDY / 'root-stderr.log').stat().st_size == 0
        outer = load(STUDY / 'master/collect-invocation/actual-command.json')
        reader = load(STUDY / 'master/audit-invocation/actual-command.json')
        checked = load(spec['driver_run_receipt_path'])
        common = ['--spec', str(wrapper_path), '--spec-sha256', WRAPPER_SHA, '--launch-native']
        assert outer['command'] == [spec['driver_interpreter']['path'], '-B', spec['checked_driver_source']['path']] + common
        assert checked['command'] == [spec['driver_interpreter']['path'], '-B', str(PREP / 'driver/cohort_driver.py')] + common
        assert reader['command'] == [spec['driver_interpreter']['path'], '-B', spec['scientific_reader_source']['path'],
            '--spec', str(wrapper_path), '--spec-sha256', WRAPPER_SHA, '--result-root', spec['output_root'],
            '--output', spec['scientific_reader_output_path'], '--checked-outer-receipt', str(STUDY / 'master/collect-invocation/actual-command.json'),
            '--checked-outer-sha256', descriptor(STUDY / 'master/collect-invocation/actual-command.json')['sha256']]
        assert canonical(master['original_collection_CLI']) == canonical(outer) and canonical(master['original_reader_CLI']) == canonical(reader)
        for value in (outer, reader):
            assert value['original_exit'] == 0 and value['error'] is None and value['source_inputs_unchanged'] is True
            assert value['process_started'] is True and value['launch_outcome'] == 'known_started_exit_observed'
            assert value['registered_wrapper_sha256'] == WRAPPER_SHA and value['retries'] == 0 and value['resume'] is False
            bound(value['stdout']); bound(value['stderr']); assert value['stderr']['bytes'] == 0
        assert checked['actual_exit_code'] == 0 and checked['driver_process_started'] is True and checked['source_inputs_unchanged'] is True
        assert checked['runtime_error'] is checked['source_guard_error'] is None
        assert checked['launch_outcome'] == 'known_started_exit_observed'
        bound(checked['stdout']); bound(checked['stderr'])
        assert canonical(parse(Path(outer['stdout']['path']).read_bytes().splitlines()[-1])) == canonical(checked)
        cohort = Path(spec['output_root']); terminal = load(cohort / 'driver-terminal.json')
        assert canonical(parse(Path(checked['stdout']['path']).read_bytes().splitlines()[-1])) == canonical(terminal)
        assert terminal['status'] == 'all_120_processes_completed'
        assert terminal['actual_processes_started'] == terminal['original_process_terminals'] == terminal['observed_process_terminal_memories'] == 120
        assert terminal['native_processes_started_count_is_complete'] and terminal['prelaunch_directory_barriers_complete'] and terminal['source_inputs_unchanged']
        assert terminal['error'] is None and terminal['unlaunched_case_labels'] == terminal['launch_outcome_unknown_case_labels'] == []
        assert terminal['retries'] == 0 and terminal['resumed'] is False
        claim = load(cohort / 'cohort-claim.json'); index = load(cohort / 'collection-index.json')
        barrier = load(cohort / 'prelaunch-durability.json')
        assert canonical(claim['cases']) == canonical(spec['process_order'])
        assert claim['registered_spec_sha256'] == index['registered_spec_sha256'] == WRAPPER_SHA
        assert claim['registered_candidate_sha256'] == index['registered_candidate_sha256'] == wrapper['candidate']['sha256']
        assert barrier['all_120_claims_durable'] is True and barrier['registered_spec_sha256'] == WRAPPER_SHA
        assert utc(spec['registered_utc']) <= utc(admission['completed_utc']) < utc(master['created_utc']) <= utc(claim['created_utc']) <= utc(barrier['completed_utc'])
        assert utc(barrier['completed_utc']) <= utc(terminal['cases'][0]['started_utc'])
        assert len(index['entries']) == len(terminal['cases']) == len(terminal['case_launch_accounting']) == 120
        pids = [outer['pid'], reader['pid'], checked['driver_process_id']]
        snapshots = []; entries = []
        for i, process in enumerate(spec['process_order']):
            folder = cohort / process['variant'] / process['case_id'] / process['label']
            receipt = load(folder / 'process-terminal.json'); accounting = terminal['case_launch_accounting'][i]
            assert canonical(receipt) == canonical(terminal['cases'][i])
            assert canonical(receipt['case']) == canonical(accounting['case']) == canonical(process)
            assert receipt['attempt_index'] == accounting['attempt_index'] == i
            assert receipt['native_process_started'] is True and receipt['actual_exit_code'] == accounting['actual_exit_code'] == 0
            assert receipt['error'] is None and receipt['timed_out'] is False
            assert accounting['status'] == 'known_started_exit_observed' and accounting['reservation_durable'] is True
            assert accounting['process_terminal_storage'] == 'stored_fsynced' and accounting['storage_error'] is None
            argv = [spec['binaries'][process['variant']]['path'], '--case', process['case_id'], '--variant', process['variant'],
                    '--observer', process['observer'], '--label', process['label'], '--output', str(folder / 'raw.jsonl.gz')]
            assert receipt['argv'] == argv and receipt['cwd'] == spec['cwd']
            assert index['entries'][i]['argv'] == argv and index['entries'][i]['raw_output_path'] == str(folder / 'raw.jsonl.gz')
            assert canonical(index['entries'][i]['case']) == canonical(process) and index['entries'][i]['attempt_index'] == i
            leaf_claim = load(folder / 'launch-claim.json')
            assert leaf_claim['argv'] == argv and utc(leaf_claim['created_utc']) <= utc(terminal['cases'][0]['started_utc'])
            assert leaf_claim['registered_spec_sha256'] == WRAPPER_SHA and leaf_claim['attempt_index'] == i and canonical(leaf_claim['case']) == canonical(process)
            for k in ('stdout', 'stderr', 'raw_output'): bound(receipt[k])
            assert receipt['raw_output']['path'] == str(folder / 'raw.jsonl.gz') and receipt['stderr']['bytes'] == 0
            for phase in ('before', 'after'):
                guard = load(folder / ('source-guard-' + phase + '.json'))
                assert guard['phase'] == phase and guard['attempt_index'] == i and canonical(guard['case']) == canonical(process)
                assert canonical(guard['frozen_inputs']) == canonical(spec['frozen_inputs']) and canonical(guard['binaries']) == canonical(spec['binaries'])
                assert canonical(guard['registration_artifacts']) == canonical(spec['registration_artifacts'])
                assert guard['registered_spec_sha256'] == WRAPPER_SHA
                assert utc(guard['verified_utc']) <= utc(receipt['started_utc']) if phase == 'before' else utc(guard['verified_utc']) >= utc(receipt['ended_utc'])
            raw_path = folder / 'raw.jsonl.gz'; raw = raw_path.read_bytes()
            assert raw[:10] == bytes.fromhex('1f8b08000000000000ff')
            restored = ungzip(raw_path); lines = restored.splitlines(keepends=True)
            assert len(lines) == 4 and restored.endswith(b'\n')
            records = [parse(line) for line in lines]
            assert [r['kind'] for r in records] == ['run_start', 'initial_state', 'completed_interval', 'run_end']
            assert records[0]['case_id'] == process['case_id'] and records[0]['variant'] == process['variant'] and records[0]['run_label'] == process['label']
            assert records[0]['observer_enabled'] is (process['observer'] == 'on')
            assert records[2]['completed_outer_interval'] == 1 and records[3]['status'] == 'collected' and records[3]['error'] is None
            snapshots.append((records[1]['checkpoint'], records[2]['checkpoint'], lines[2], records[2]))
            pids.append(receipt['pid']); assert receipt['pid'] == accounting['pid']
            entries.append({'process': process, 'pid': receipt['pid'], 'original_exit': 0, 'raw': receipt['raw_output'],
                            'whole_uncompressed_bytes': len(restored), 'whole_uncompressed_sha256': hashlib.sha256(restored).hexdigest()})
        assert len(pids) == len(set(pids)) == 123 and all(not Path('/proc/' + str(p)).exists() for p in pids)
        assert len(list(cohort.rglob('raw.jsonl.gz'))) == 120
        for i in range(0, 120, 3):
            on, fresh, off = snapshots[i:i+3]
            assert canonical(on[0]) == canonical(fresh[0]) == canonical(off[0])
            assert canonical(on[1]) == canonical(fresh[1]) == canonical(off[1]) and on[2] == fresh[2]
            assert all(off[3][k] is None for k in ('construction_companion', 'stage_v1', 'impulse_v1_ieee_words'))
        audit_path = Path(spec['scientific_reader_output_path']); audit = load(audit_path)
        assert descriptor(audit_path) == {'path': str(audit_path), 'bytes': 2049298, 'sha256': 'a43b19474de7478faa43bad15de2ee977f51473896575f6c282d381dfe8ecf99'}
        assert audit['fidelity_status'] == 'PASS_COMPLETE_DECLARED_SCOPE' and audit['issues'] == []
        assert audit['physical_SI_status'] == 'SOME_NEW_CASE_DIAGNOSTICS_OUTSIDE_DECLARED_ALLOWANCES'
        assert audit['actual_processes'] == 120 and len(audit['processes']) == 120
        for key, expected in [('constructor_checks',80),('coordinate_checks',80),('native_motor_solve_checks',240),('companion_events',720),('stage_v1_events',2080)]: assert audit[key] == expected
        for key in ('native_motor_accuracy_verified', 'old_calibration_reclassified', 'production_normalization_intervention'): assert audit[key] is False
        bound(audit['original_checked_CLI_receipt'])
        reader_claim = load(cohort / 'reader-audit-claim.json'); reader_intent = load(cohort / 'scientific-reader-launch-claim.json')
        assert reader_claim['externally_supplied_wrapper_sha256'] == WRAPPER_SHA and reader_claim['output'] == str(audit_path)
        assert reader_intent['command'] == reader['command'] and reader_claim['checked_outer_sha256'] == descriptor(STUDY / 'master/collect-invocation/actual-command.json')['sha256']
        assert reader_intent['maximum_invocations'] == 1 and reader_intent['retries'] == 0
        assert utc(terminal['ended_utc']) <= utc(reader_intent['created_utc']) <= utc(reader_claim['created_utc']) <= utc(master['ended_utc'])
        counts = {}
        for variant in ('baseline', 'candidate'):
            rows = [r for r in audit['physical_diagnostics'] if r['variant'] == variant]
            assert len(rows) == len({r['case_id'] for r in rows}) == 20
            assert {r['case_id'] for r in rows} == {c['case_id'] for c in spec['cases']}
            labels = Counter(); regimes = defaultdict(Counter)
            for row in rows:
                label = row['diagnostics']['main_physical_label']; regime = row['diagnostics']['nominal_SI_reference']['regime']
                kind = 'saturated' if regime.startswith('saturated_') else regime
                labels[label] += 1; regimes[kind][label] += 1
            counts[variant] = {'variant': variant, 'unique_cases': 20, 'labels': dict(labels), 'by_regime': {k:dict(v) for k,v in regimes.items()}}
        summary_path = STUDY / 'result-summary.json'; summary = load(summary_path)
        assert descriptor(summary_path)['sha256'] == '0e318c57b08265bf7eaa001203a5368e7f5877671b9d4b9763dfd3674fa29900'
        assert canonical(summary['rows']) == canonical([counts['baseline'], counts['candidate']])
        assert counts['baseline']['labels'] == {'OUTSIDE_DECLARED_NEW_CASE_ALLOWANCE':8,'MEETS_DECLARED_NEW_CASE_ALLOWANCE':12}
        assert counts['candidate']['labels'] == {'MEETS_DECLARED_NEW_CASE_ALLOWANCE':20}
        assert all(v is False for v in summary['claims'].values())
        for k in ('audit', 'lossless_whole_audit_carrier', 'master', 'root_original_launch'): bound(summary[k])
        carrier = Path(summary['lossless_whole_audit_carrier']['path'])
        assert descriptor(carrier)['sha256'] == '016d5524c6ee239da8b4ebf7a3856076ff2e52f399484cb3c7c4a0b2ca7d2b69'
        assert carrier.stat().st_size == 61534 and ungzip(carrier) == audit_path.read_bytes()
        assert descriptor(STUDY / 'README.md')['sha256'] == 'f6808063dab5d1751f1b13c6632fdb0a2cfce7b36b53255b806caa688be394ef'
        after = [descriptor(p) for p in sorted(STUDY.rglob('*')) if p.is_file()]
        source_after = [descriptor(d['path']) for d in spec['frozen_inputs']]
        assert before == after and source_before == source_after
        save('whole-study-file-ledger-after.json', after); save('frozen-source-inputs-after.json', source_after)
        head = git('rev-parse', 'HEAD').decode().strip(); tree = git('rev-parse', 'HEAD^{tree}').decode().strip()
        assert head == admission['root_head'] == '423a3fd7f44128aa440afe47f937039de504f275' and tree == admission['root_tree'] and git('status', '--porcelain=v1') == b''
        save('direct-label-projection-and-original-process-map.json', {'method':'Direct categorical label counts and literal-byte/provenance comparison only; no SI or source rederivation', 'rows':[counts['baseline'],counts['candidate']], 'original_process_map':entries})
        result.update(status='PASS_COMPLETED_NEW120_BYTE_PROVENANCE_AND_DIRECT_PRESENTATION_REVIEW',
            wrapper=descriptor(wrapper_path), audit=descriptor(audit_path), summary=descriptor(summary_path), readme=descriptor(STUDY / 'README.md'),
            whole_audit_carrier=descriptor(carrier), carrier_complete_single_member_CRC_EOF_and_literal_full_restoration=True,
            original_native_exits_zero=120, original_coordinator_and_checked_and_reader_CLI_exits_zero=True,
            root_original_tool_exit_zero_observed_receipt=descriptor(STUDY / 'root-original-launch.json'),
            root_outer_tool_receipt_scope='Parent-observed actual original tool exit retained after completion; not an external execution attestation.',
            recorded_123_process_ids_absent=True, launch_outcome_unknown_count=0, native_or_audit_retries=0,
            all_120_whole_raw_originals_and_receipt_bindings_verified=True, same_captured_ON_fresh_OFF_bytes_and_fresh_record_literal=True,
            frozen536_inputs_current_unchanged=True, all_study_artifacts_before_after_unchanged=True,
            root_head=head, root_tree=tree, root_checkout_clean=True,
            prereg_admission_and_freeze_before_all_native_claims=True, fixed_120_claims_durable_before_first_start=True,
            one_scientific_launch_and_audit_claim_bound=True,
            direct_label_projection=[counts['baseline'],counts['candidate']],
            proof_ledgers=[descriptor(OWN / n) for n in ('whole-study-file-ledger-before.json','whole-study-file-ledger-after.json',
                'frozen-source-inputs-before.json','frozen-source-inputs-after.json','direct-label-projection-and-original-process-map.json')],
            presentation_scope_review='README and summary correctly restrict the20-case/one-interval nominal diagnostic, separate integration/writeback quantities, preserve baseline outside-labels and all originals, and exclude adoption/old calibration/coupled-contact/hardware/maturity claims.',
            limitations=['This is byte/provenance and direct categorical presentation review, not a second scientific audit or SI/source-arithmetic derivation.',
                'Source-fidelity PASS and candidate20/20 concern this new isolated nominal diagnostic only; historical four criteria and globally rejected candidate remain unchanged.',
                'All123 recorded native/coordinator/checked/reader PIDs are absent; the master shell completion is evidenced by root original tool receipt, not an account-wide process/schedule audit.',
                'No public upload, production correction, real-world validation, general motor accuracy or maturity gain is established.'])
    except BaseException as failure:
        result['error'] = type(failure).__name__ + ': ' + str(failure)
        raise
    finally:
        result['completed_utc'] = datetime.now(timezone.utc).isoformat()
        result['review_source'] = descriptor(Path(__file__))
        save('independent-postcollection-review.json', result)
        print(json.dumps({'status':result['status'],'receipt':descriptor(OWN / 'independent-postcollection-review.json')}), flush=True)


if __name__ == '__main__': main()
