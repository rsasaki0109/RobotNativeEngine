"""Read-only retained-result binding/framing/equality checks; no numeric oracle replay."""

import datetime
import hashlib
import json
import os
from pathlib import Path
import sys

BASE = Path('/workspace/.rne-onboarding/recovery-20261012/next-motor-construction-six')
OUT = BASE / 'independent-postcollection-review'
TOOLS = Path('/workspace/.rne-onboarding/recovery-20261012/next-motor-construction-cohort-reader-v2')
sys.path.insert(0, str(TOOLS))
from strict_io import canonical, gzip_records, loads


def require(value, message):
    if not value: raise ValueError(message)


def load(path): return loads(Path(path).read_bytes())
def same(a, b): return canonical(a) == canonical(b)
def time(value): return datetime.datetime.fromisoformat(value)


def desc(path):
    path = Path(path); size = 0; digest = hashlib.sha256()
    with path.open('rb') as stream:
        while block := stream.read(1 << 20): size += len(block); digest.update(block)
    return dict(path=str(path), sha256=digest.hexdigest(), bytes=size)


def unsigned(value, width):
    require(type(value) is int and 0 <= value < 1 << width, 'invalid typed native integer')


def native(value, width=32):
    if type(value) is dict:
        for key, item in value.items():
            if key in ('public_body_words', 'public_multibody_words'):
                native(item, width)
            elif key == 'impulse_v1_ieee_words' and item is not None:
                native(item, 64)
            elif key.endswith('_word') and item is not None:
                unsigned(item, width)
            elif key.endswith('_words') and item is not None:
                require(type(item) is list, 'native words are not a vector')
                for word in item: unsigned(word, 32 if 'handle' in key else width)
            elif key == 'postcard_bytes':
                require(type(item) is list and item, 'missing native object bytes')
                for byte in item: unsigned(byte, 8)
            elif key == 'native_counts':
                require(type(item) is list, 'bad native count vector')
                for number in item: unsigned(number, 64)
            else: native(item, width)
    elif type(value) is list:
        for item in value: native(item, width)
    elif type(value) is int: unsigned(value, 64)
    else: require(value is None or type(value) in (str, bool), 'unexpected scalar payload type')


def save(name, value):
    raw = (json.dumps(value, indent=2, sort_keys=True) + '\n').encode()
    with (OUT / name).open('xb') as stream:
        stream.write(raw); stream.flush(); os.fsync(stream.fileno())
    fd = os.open(OUT, os.O_RDONLY | os.O_DIRECTORY)
    try: os.fsync(fd)
    finally: os.close(fd)
    return desc(OUT / name)


def main():
    ledger = []
    def check(expected):
        actual = desc(expected['path']); require(actual == expected, 'artifact hash/size/path changed')
        ledger.append(dict(expected=expected, actual=actual)); return actual
    wrapper_path = BASE / 'admitted-wrapper-v2.json'
    wrapper_sha = 'f78dce9830973d98de4bd86bf43fd73689859f3ed1f08880b96b0daad15523ce'
    check(load(BASE / 'independently-retained-wrapper-descriptor-v2.json'))
    require(desc(wrapper_path)['sha256'] == wrapper_sha, 'wrapper SHA mismatch')
    wrapper = load(wrapper_path)
    check(wrapper['candidate']); check(wrapper['admission'])
    candidate = load(wrapper['candidate']['path']); admission = load(wrapper['admission']['path'])
    candidate_sha = wrapper['candidate']['sha256']
    require(candidate_sha == '0d5912950c5d6d3d980262f1f263746d6fe820d17d92cf0bd95829304364c194', 'candidate changed')
    require(admission['status'] == 'PASS_PRE_DATA_SOURCE_PROTOCOL_AND_DRIVER' and admission['reviewed_candidate_sha256'] == candidate_sha, 'predata admission mismatch')
    require(admission['frozen_inputs_sha256'] == hashlib.sha256(canonical(candidate['frozen_inputs'])).hexdigest(), 'predata input list changed')
    for item in [candidate['binary'], candidate['driver_interpreter'], candidate['checked_driver_source'], *candidate['frozen_inputs'], *candidate['registration_artifacts'].values()]: check(item)
    protocol = load(candidate['registration_artifacts']['protocol']['path'])
    cases = candidate['cases']; root = Path(candidate['output_root'])
    require(len(cases) == 6 and same(cases, protocol['cases']), 'registered cases differ')
    outer = load(BASE / 'outer-command-v2/actual-command.json')
    checked = load(BASE / 'checked-driver-v2/actual-command.json')
    terminal = load(root / 'driver-terminal.json')
    reader = load(BASE / 'actual-reader-command-v2.json')
    for record in (outer, checked):
        require(type(record['actual_exit_code']) is int and record['actual_exit_code'] == 0, 'actual outer/coordinator exit is nonzero')
        for key in ('stdout', 'stderr'): check(record[key])
    outer_expected = ['/usr/bin/python3', '-B', str(TOOLS / 'checked_driver.py'), '--spec', str(wrapper_path), '--spec-sha256', wrapper_sha, '--launch-native']
    inner_expected = outer_expected.copy(); inner_expected[2] = str(TOOLS / 'cohort_driver.py')
    require(same(outer['argv'], outer_expected) and same(checked['command'], inner_expected), 'actual Python command differs')
    require(same(loads(Path(outer['stdout']['path']).read_bytes().splitlines()[-1]), checked), 'root outer stdout differs from checked receipt')
    require(same(loads(Path(checked['stdout']['path']).read_bytes().splitlines()[-1]), terminal), 'checked stdout final memory differs from disk terminal')
    require(checked['source_inputs_unchanged'] is True and checked['source_guard_error'] is None and checked['runtime_error'] is None, 'checked source/runtime guard failed')
    require(checked['registered_wrapper_sha256'] == wrapper_sha and checked['registered_candidate_sha256'] == candidate_sha, 'coordinator binding differs')
    require(terminal['status'] == 'all_six_processes_completed' and terminal['source_inputs_unchanged'] is True and terminal['error'] is None, 'driver incomplete')
    require(terminal['actual_processes_started'] == terminal['original_process_terminals'] == terminal['observed_process_terminal_memories'] == 6, 'driver counts differ')
    require(terminal['native_processes_started_count_is_complete'] is True and terminal['prelaunch_directory_barriers_complete'] is True, 'unknown/durability failure')
    require(terminal['retries'] == 0 and terminal['resumed'] is False and terminal['unlaunched_case_labels'] == terminal['launch_outcome_unknown_case_labels'] == [], 'retry/partial launch')
    require(reader['original_exit_code'] == 0 and reader['audit_invocations'] == 1 and reader['reader_native_worlds'] == 0 and reader['root_actual_checked_outer_exit'] == 0, 'reader original invocation differs')
    reader_expected = ['/usr/bin/python3', '-B', str(TOOLS / 'cohort_reader.py'), '--spec', str(wrapper_path), '--spec-sha256', wrapper_sha, '--result-root', str(root), '--output', str(BASE / 'actual-cohort-audit-v2.json')]
    require(same(reader['command'], reader_expected), 'actual reader command differs')
    check(reader['audit']); require(reader['audit']['sha256'] == 'a3f3e60fb199390e1dd761f6e96357df684bbcb0bdc996d5ec24a0745c3e107c', 'audit changed')
    audit = load(reader['audit']['path'])
    require(audit['status'] == 'PASS_COMPLETE_DECLARED_SCOPE' and audit['issues'] == [] and audit['native_motor_accuracy_verified'] is False, 'qualified outcome differs')
    require(audit['actual_processes'] == audit['actual_outer_intervals'] == 6 and audit['observed_ON_processes'] == 4, 'audit process scope differs')
    require(audit['construction_events'] == 1028 and audit['stage_events'] == 3328 and audit['constructor_checks'] == audit['coordinate_advance_checks'] == 128, 'audit source counts differ')
    require(same(audit['source_scope'], candidate['source_scope']), 'audit scope expanded')
    claim = load(root / 'cohort-claim.json'); barriers = load(root / 'prelaunch-durability.json')
    require(same(claim['cases'], cases) and same(claim['binary'], candidate['binary']) and same(claim['source_inputs'], candidate['frozen_inputs']), 'claimed closure differs')
    require(claim['registered_spec_sha256'] == wrapper_sha and claim['registered_candidate_sha256'] == candidate_sha, 'claim hashes differ')
    require(barriers['all_six_claims_durable'] is True and barriers['registered_spec_sha256'] == wrapper_sha, 'barrier claim differs')
    registration = time(candidate['registered_utc']); admitted = time(admission['reviewed_at'])
    claimed = time(claim['created_utc']); barriers_at = time(barriers['completed_utc'])
    require(registration <= admitted <= time(outer['started_at']) <= time(checked['started_utc']) <= claimed <= barriers_at, 'predata chronology differs')
    require(time(terminal['ended_utc']) <= time(checked['ended_utc']) <= time(outer['completed_at']) <= time(reader['recorded_at']), 'completion chronology differs')
    runs = []; process_ids = []; claims_at = []; previous_end = barriers_at
    for index, case in enumerate(cases):
        folder = root / case['label']; launch = load(folder / 'launch-claim.json')
        intent = load(folder / 'launch-intent.json'); started = load(folder / 'process-started.json')
        result = load(folder / 'process-terminal.json'); accounting = terminal['case_launch_accounting'][index]
        expected_argv = [candidate['binary']['path'], '--target', case['target'], '--observer', case['observer'], '--label', case['label'], '--output', str(folder / 'raw.jsonl.gz')]
        require(same(result, terminal['cases'][index]) and same(result['case'], case) and same(launch['case'], case), 'original process case differs')
        require(same(result['argv'], expected_argv) and same(launch['argv'], expected_argv) and same(intent['argv'], expected_argv), 'native argv differs')
        require(result['actual_exit_code'] == 0 and type(result['actual_exit_code']) is int and result['native_process_started'] is True and result['timed_out'] is False and result['error'] is None, 'original native process failed')
        require(accounting['status'] == 'known_started_exit_observed' and accounting['reservation_durable'] is True and accounting['process_terminal_storage'] == 'stored_fsynced' and accounting['storage_error'] is None and accounting['pid'] == result['pid'], 'case accounting differs')
        require(accounting['actual_exit_code'] == 0 and accounting['attempt_index'] == index and result['attempt_index'] == launch['attempt_index'] == intent['attempt_index'] == index, 'case ordinal/exit differs')
        expected_started = dict(result); expected_started.update(actual_exit_code=None, timed_out=False, error=None, ended_utc=None, stdout=None, stderr=None, raw_output=None)
        require(same(started, expected_started), 'original process-start receipt differs')
        left, right = time(result['started_utc']), time(result['ended_utc']); ct = time(launch['created_utc']); it = time(intent['created_utc'])
        require(claimed <= ct <= left <= it <= right and previous_end <= left and barriers_at <= left, 'native process chronology differs')
        previous_end = right; claims_at.append(ct); process_ids.append(result['pid'])
        for key, name in [('stdout', 'stdout.log'), ('stderr', 'stderr.log'), ('raw_output', 'raw.jsonl.gz')]:
            require(result[key]['path'] == str(folder / name), 'substituted native artifact path'); check(result[key])
        records, lines, framing = gzip_records(folder / 'raw.jsonl.gz')
        require(len(records) == 4 and [r['kind'] for r in records] == ['run_start', 'initial_state', 'completed_interval', 'run_end'], 'whole successful record grammar differs')
        for record in records: native(record)
        require(records[0]['run_label'] == case['label'] and records[0]['target'] == case['target'] and records[0]['observer_enabled'] is (case['observer'] == 'on'), 'raw run identity differs')
        require(records[3]['status'] == 'collected' and records[3]['error'] is None and records[3]['progress']['completed_outer_intervals'] == 1 and records[3]['progress']['partial_records_written'] == 0, 'raw native completion differs')
        for checkpoint in [records[1]['checkpoint'], records[2]['checkpoint']]:
            require(checkpoint['native_scalar_bits'] == 32 and checkpoint['integration_parameters']['native_counts'] == [32, 0, 1, 2, 128, 1], 'checkpoint topology/counts differ')
            require([(v['kind'], v['handle_words']) for v in checkpoint['records']] == [('Body', [0, 0]), ('Body', [1, 0]), ('Collider', [0, 0]), ('Multibody', [0, 0])], 'captured object ordering differs')
        audit_case = audit['cases'][index]
        require(same(audit_case['case'], case) and same(audit_case['framing'], framing), 'audit/raw case framing differs')
        done = records[2]; analyses = audit_case['analyses']
        if case['observer'] == 'off':
            require(all(done[k] is None for k in ['construction_companion', 'stage_v1', 'impulse_v1_ieee_words']) and analyses == [], 'OFF observer scope differs')
        else:
            comp = done['construction_companion']['events']; stage = done['stage_v1']['events']
            require(len(comp) == 257 and len(stage) == 832 and len(analyses) == 32, 'complete source event counts differ')
            for slice_ in range(32):
                for offset, (phase, iteration) in enumerate(protocol['companion_schedule']):
                    event = comp[slice_ * 8 + offset]
                    require(event['phase'] == phase and event['iteration_index'] == iteration and event['internal_slice_index'] == slice_, 'companion source order differs')
                    require(event['row_cache_slice_index'] == (None if offset < 2 else slice_), 'companion cache generation differs')
                for offset, (phase, iteration, visit) in enumerate(protocol['stage_v1_schedule']):
                    event = stage[slice_ * 26 + offset]
                    require(event['stage'] == phase and event['iteration_index'] == iteration and event['joint_visit_index'] == visit and event['internal_slice_index'] == slice_, 'v1 source order differs')
                    require(event['row_cache_slice_index'] == (None if offset < 2 else slice_), 'v1 cache generation differs')
                a = analyses[slice_]; require(a['slice'] == slice_, 'retained analysis order differs')
                require(a['diagonal_exclusions'] == [] and all(a[k]['status'] == 'DECLARED_FORMULAS_MATCH' and a[k]['exclusions'] == [] and a[k]['mismatches'] == {} and a[k]['motor_accuracy_verified'] is False for k in ['construction', 'integration']), 'retained source qualifier outcome differs')
                require(same(a['construction']['expected_words'], a['construction']['observed_words']), 'retained constructor expected/observed bits differ')
                before = comp[slice_ * 8 + 3]['endpoints'][0]; after = comp[slice_ * 8 + 4]['endpoints'][0]
                require(a['integration']['expected_coordinate_word'] == after['coordinate_word'] == a['q_after_rad_word'] and a['q_before_rad_word'] == before['coordinate_word'], 'retained coordinate diagnostics differ from capture')
                require(a['integration_input_velocity_rad_s_word'] == before['multibody_velocity_words'][0], 'integration copied V diagnostic differs')
                require(a['stabilization_generic_velocity_rad_s_words'] == [comp[slice_ * 8 + j]['generalized_velocity_words'][0] for j in [5, 6]], 'stabilization generic V copied words differ')
                require(a['stabilized_MB_velocity_rad_s_words'] == [comp[slice_ * 8 + j]['endpoints'][0]['multibody_velocity_words'][0] for j in [5, 6]], 'stabilization MB V copied words differ')
                require(a['final_slice_generic_velocity_rad_s_word'] == comp[slice_ * 8 + 7]['generalized_velocity_words'][0], 'final slice copied words differ')
            require(comp[-1]['phase'] == 'AfterVelocityWriteback' and comp[-1]['internal_slice_index'] == 31 and analyses[-1]['final_writeback_velocity_rad_s_word'] == comp[-1]['generalized_velocity_words'][0], 'final writeback scope differs')
        runs.append(dict(case=case, initial=records[1]['checkpoint'], completed=done['checkpoint'], literal_completed_record=lines[2], framing=framing))
    require(len(set(process_ids)) == 6 and all(v <= time(terminal['cases'][0]['started_utc']) for v in claims_at), 'process uniqueness/claim chronology differs')
    for group in [runs[:3], runs[3:]]:
        require(all(same(r['initial'], group[0]['initial']) and same(r['completed'], group[0]['completed']) for r in group[1:]), 'captured ON/fresh/OFF equality differs')
        require(group[0]['literal_completed_record'] == group[1]['literal_completed_record'], 'fresh ON complete record bytes differ')
    require(all(audit[k] is True for k in ['source_inputs_unchanged', 'captured_checkpoint_ON_repeat_OFF_exact', 'fresh_ON_complete_record_literal_exact', 'off_observers_absent']), 'audit capture/source claims differ')
    for entry in ledger: require(desc(entry['expected']['path']) == entry['expected'], 'review inputs changed during checks')
    ledger_desc = save('retained-result-and-source-hash-ledger.json', ledger)
    details = save('retained-result-verification-details.json', dict(status='PASS_RETAINED_SCOPE_CHECKS', wrapper=desc(wrapper_path), audit=reader['audit'], candidate_sha256=candidate_sha, hash_checks=len(ledger), ledger=ledger_desc, raw_files=[r['framing'] for r in runs], native_process_ids=process_ids, actual_native_exit_codes=[v['actual_exit_code'] for v in terminal['cases']], original_root_outer_exit=outer['actual_exit_code'], original_coordinator_exit=checked['actual_exit_code'], actual_reader_original_exit=reader['original_exit_code'], native_collection_count=6, observed_ON_worlds=4, companion_events=1028, stage_events=3328, frozen_reader_invocations_recorded=1, additional_reader_invocations=0, additional_numeric_analysis=0, reviewer_worlds=0, reviewer_Cargo_builds=0, old_numeric_report_reads=0, input_sources_unchanged_during_review=True))
    print(json.dumps(dict(status='PASS_RETAINED_SCOPE_CHECKS', details=details, hash_checks=len(ledger)), sort_keys=True))


if __name__ == '__main__': main()
