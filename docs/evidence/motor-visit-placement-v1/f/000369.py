"""New fixed18 admission and once-only audit; source/synthetic preparation only."""
import argparse
from datetime import datetime, timezone
import sys
from pathlib import Path
import run_schema
import stage_numeric
import construction_oracle
import integration_oracle
import physical_reference
import strict_io
import cohort_contract
from cohort_contract import CASES, load_spec, expected_driver_command, persistent
from run_schema import PROFILES, schedules, validate_run, same
from strict_io import bound, boolean, canonical, descriptor, integer, keys, loads, need, write_exclusive


def reader_closure(spec):
    paths = {item['path'] for item in spec['frozen_inputs']}
    for module in (sys.modules[__name__], run_schema, stage_numeric, construction_oracle,
                   integration_oracle, physical_reference, strict_io, cohort_contract):
        need(str(Path(module.__file__).resolve()) in paths, 'actually loaded reader source not frozen')
    same(CASES, run_schema.CASES, 'reader/driver case order differs')
    need(Path(sys.executable).resolve() == Path(spec['driver_interpreter']['path']).resolve(),
         'active scientific reader interpreter differs from pinned interpreter')
    for name, pair in PROFILES.items():
        same(list(pair), [cohort_contract.PROFILES[name]['pgs_passes'], cohort_contract.PROFILES[name]['stabilization_passes']], 'profile contract differs')


def validate_collection_index(spec, root):
    index = loads((root / 'collection-index.json').read_bytes())
    keys(index, 'kind schema_version registered_spec_sha256 registered_candidate_sha256 binary entries')
    integer(index['schema_version'])
    need(index['kind'] == 'fixed_motor_visit_placement_collection_index' and index['schema_version'] == 1, 'wrong prelaunch index')
    need(index['registered_spec_sha256'] == spec['_authenticated_wrapper_sha256'] and index['registered_candidate_sha256'] == spec['_authenticated_candidate_sha256'], 'index registration differs')
    same(index['binary'], spec['binary'], 'index binary differs')
    need(type(index['entries']) is list and len(index['entries']) == 18, 'incomplete prelaunch index')
    for i, case in enumerate(CASES):
        folder=root / case['label']
        expected=dict(attempt_index=i,case=case,
            argv=[spec['binary']['path'],'--profile',case['profile'],'--target',case['target'],
                  '--observer',case['observer'],'--label',case['label'],'--output',str(folder/'raw.jsonl.gz')],
            cwd=spec['cwd'],folder=str(folder),raw_output_path=str(folder/'raw.jsonl.gz'),
            launch_claim_path=str(folder/'launch-claim.json'),process_terminal_path=str(folder/'process-terminal.json'))
        same(index['entries'][i], expected, 'prelaunch index entry differs')


def check_process_guards(spec, case, index, folder, started, ended):
    for phase in ('before','after'):
        value=loads((folder / ('source-guard-'+phase+'.json')).read_bytes())
        keys(value,'kind schema_version phase case attempt_index verified_utc binary frozen_inputs registration_artifacts registered_spec_sha256')
        integer(value['schema_version']);integer(value['attempt_index'])
        need(value['kind']=='fixed_case_frozen_input_guard' and value['schema_version']==1 and
             value['attempt_index']==index and value['phase']==phase, 'wrong process source guard')
        same(value['case'],case,'guard case differs');same(value['binary'],spec['binary'],'guard binary differs')
        same(value['frozen_inputs'],spec['frozen_inputs'],'guard source descriptors differ')
        same(value['registration_artifacts'],spec['registration_artifacts'],'guard registration descriptors differ')
        need(value['registered_spec_sha256']==spec['_authenticated_wrapper_sha256'],'guard wrapper differs')
        time=datetime.fromisoformat(value['verified_utc'])
        need(time.tzinfo is not None and (time<=started if phase=='before' else time>=ended),'source guard chronology differs')


def verify_checked_cli(spec, receipt_path, supplied_sha, inner):
    """Require the checked script's original exit, distinct from its child exit."""
    p = persistent(str(Path(receipt_path)), 'outer original receipt must be persistent canonical path')
    need(type(supplied_sha) is str and len(supplied_sha)==64 and
         all(c in '0123456789abcdef' for c in supplied_sha), 'invalid outer receipt SHA')
    need(descriptor(p)['sha256']==supplied_sha, 'outer original receipt hash differs')
    value=loads(p.read_bytes())
    keys(value,'kind mode command registered_wrapper_sha256 created_utc cwd retries resume process_started pid original_exit launch_outcome error start_unix_ns end_unix_ns source_inputs_unchanged stdout stderr')
    for key in ('retries','pid','original_exit','start_unix_ns','end_unix_ns'):integer(value[key])
    for key in ('resume','process_started','source_inputs_unchanged'):boolean(value[key])
    need(value['kind']=='exclusive_original_cli_claim' and value['mode']=='collect' and
         value['original_exit']==0 and value['retries']==0 and value['resume'] is False and
         value['process_started'] is True and value['source_inputs_unchanged'] is True and
         value['launch_outcome']=='known_started_exit_observed' and value['error'] is None,
         'root checked CLI actual exit/launch/source guard failed')
    expected=[spec['driver_interpreter']['path'],'-B',spec['checked_driver_source']['path'],
              '--spec',spec['_authenticated_wrapper_path'],'--spec-sha256',spec['_authenticated_wrapper_sha256'],'--launch-native']
    same(value['command'],expected,'original checked CLI command differs')
    need(value['registered_wrapper_sha256']==spec['_authenticated_wrapper_sha256'] and
         value['cwd']==spec['cwd'],'root checked CLI binding differs')
    for key in ('stdout','stderr'):
        bound(value[key])
        need(value[key]['path']==str(p.parent/(key+'.log')),'outer stdout/stderr path substituted')
    stdout=Path(value['stdout']['path']).read_bytes()
    need(stdout.endswith(b'\n') and stdout.splitlines(),'outer checked CLI stdout incomplete')
    same(loads(stdout.splitlines()[-1]),inner,'outer checked CLI memory stdout differs from original inner receipt')
    need(value['start_unix_ns']<=value['end_unix_ns'],'outer checked CLI clock bounds reversed')
    inner_start=datetime.fromisoformat(inner['started_utc'])
    inner_end=datetime.fromisoformat(inner['ended_utc'])
    created=datetime.fromisoformat(value['created_utc'])
    need(created.tzinfo is not None and created<=inner_start<=inner_end,'outer/inner checked CLI UTC order differs')
    # ISO timestamps have microsecond precision; compare integer microsecond epochs.
    epoch=datetime(1970,1,1,tzinfo=timezone.utc)
    def micros(t):
        delta=t-epoch
        return ((delta.days*86400+delta.seconds)*1000000+delta.microseconds)
    need(value['start_unix_ns']//1000<=micros(inner_start) and
         micros(inner_end)<=value['end_unix_ns']//1000,'outer/inner actual timing bounds differ')
    return descriptor(p)


def analysis_issues(run):
    issues=[]
    for analysis in run['analyses']:
        for key in ('construction','integration','initialized_diagonal'):
            if analysis[key]['status']!='DECLARED_FORMULAS_MATCH':
                issues.append(dict(case=run['case']['label'],slice=analysis['slice'],check=key,result=analysis[key]))
        for visit in analysis['native_visits']:
            if visit['check']['status']!='DECLARED_FORMULAS_MATCH':
                issues.append(dict(case=run['case']['label'],slice=analysis['slice'],check='native_solve',result=visit))
        if analysis['row_algebra']['status']!='CONDITIONAL_STORED_OPERANDS_REFERENCE':
            issues.append(dict(case=run['case']['label'],slice=analysis['slice'],check='row_algebra',result=analysis['row_algebra']))
        if analysis['physical_SI']['status']!='REAL_SI_DIAGNOSTIC_ONLY_NO_ACCURACY_VERDICT':
            issues.append(dict(case=run['case']['label'],slice=analysis['slice'],check='physical_SI_availability',result=analysis['physical_SI']))
    return issues


def discriminate(runs, fidelity_issues):
    results=[];issues=[]
    for sign in ('negative','positive'):
        selected={p:next(r for r in runs if r['case']['profile']==p and r['case']['target']==sign and r['case']['label'].endswith('_on')) for p in PROFILES}
        baseline=selected['split_1_2'];a=baseline['analyses'][0]
        common={k:a[k] for k in ('first_row','first_constructor_inputs','first_incoming_velocity_word')}
        for p,r in selected.items():
            same({k:r['analyses'][0][k] for k in common},common,'first-row native cache/input differs across profiles')
            same({k:v for k,v in r['initial'].items() if k!='integration_parameters'},
                 {k:v for k,v in baseline['initial'].items() if k!='integration_parameters'},'initial native physical state differs across profiles')
        b=selected['early_1_0']['analyses'][0];c=selected['early_3_0']['analyses'][0]
        first_same=(a['q_after_rad_word']==b['q_after_rad_word'] and a['integration_input_velocity_rad_s_word']==b['integration_input_velocity_rad_s_word'])
        final_same=a['final_slice_generic_velocity_rad_s_word']==c['final_slice_generic_velocity_rad_s_word']
        different_q=a['q_after_rad_word']!=c['q_after_rad_word']
        different_final=a['final_slice_generic_velocity_rad_s_word']!=b['final_slice_generic_velocity_rad_s_word']
        if not fidelity_issues and (not first_same or not final_same):
            issues.append(dict(target=sign,check='source_conditional_first_slice_map_identity',baseline_early1_integration_equal=first_same,baseline_early3_final_equal=final_same))
        results.append(dict(target=sign,source_conditional_same_first_row_and_initial_physics=True,
            baseline_early1_first_integration_equal=first_same,baseline_early3_first_final_equal=final_same,
            baseline_early3_first_q_different=different_q,baseline_early1_first_final_different=different_final,
            status='UNQUALIFIED_FIDELITY_OR_SOURCE_IDENTITY' if fidelity_issues or not first_same or not final_same else
                   'OBSERVED_FIRST_SLICE_VISIT_TIMING_DIFFERENCE' if different_q or different_final else
                   'INCONCLUSIVE_NO_CAPTURED_FIRST_SLICE_DIFFERENCE',
            unequal_q_required_for_fidelity=False,accuracy_verified=False))
    return results,issues


def qualify_cohort(spec, result_root, checked_outer_receipt, checked_outer_sha256):
    root = Path(result_root)
    reader_closure(spec)
    need(spec.get('_authenticated_wrapper_sha256') and spec.get('_authenticated_candidate_sha256'), 'mandatory admission authentication missing')
    need(root.resolve() == Path(spec['output_root']).resolve(), 'wrong registered cohort root')
    terminal = loads((root / 'driver-terminal.json').read_bytes())
    need(Path(spec['driver_run_receipt_path']).is_file(), 'mandatory checked-driver original command receipt missing')
    command_receipt = loads(Path(spec['driver_run_receipt_path']).read_bytes())
    keys(command_receipt, 'kind schema_version command cwd registered_wrapper_sha256 registered_candidate_sha256 started_utc ended_utc driver_process_started driver_process_id launch_outcome actual_exit_code source_inputs_unchanged source_guard_error runtime_error stdout stderr')
    integer(command_receipt['schema_version']); integer(command_receipt['actual_exit_code']); integer(command_receipt['driver_process_id'])
    boolean(command_receipt['driver_process_started']); boolean(command_receipt['source_inputs_unchanged'])
    need(command_receipt['kind'] == 'original_checked_driver_command' and command_receipt['schema_version'] == 2 and command_receipt['actual_exit_code'] == 0 and command_receipt['driver_process_started'] is True and command_receipt['launch_outcome'] == 'known_started_exit_observed' and command_receipt['source_inputs_unchanged'] is True and command_receipt['source_guard_error'] is None and command_receipt['runtime_error'] is None, 'checked outer driver original exit or source guard failed')
    same(command_receipt['command'], expected_driver_command(spec), 'checked original driver command differs')
    need(command_receipt['cwd'] == spec['cwd'] and command_receipt['registered_wrapper_sha256'] == spec['_authenticated_wrapper_sha256'] and command_receipt['registered_candidate_sha256'] == spec['_authenticated_candidate_sha256'], 'checked driver binding differs')
    for key in ('stdout', 'stderr'): bound(command_receipt[key])
    outer_binding=verify_checked_cli(spec, checked_outer_receipt, checked_outer_sha256, command_receipt)
    stdout = Path(command_receipt['stdout']['path']).read_bytes()
    need(stdout.endswith(b'\n') and stdout.splitlines(), 'checked driver stdout not complete')
    same(loads(stdout.splitlines()[-1]), terminal, 'checked driver final memory JSON differs from disk terminal')
    keys(terminal, 'kind driver_schema_version status ended_utc actual_processes_started native_processes_started_count_is_complete original_process_terminals observed_process_terminal_memories prelaunch_directory_barriers_complete source_inputs_unchanged error registered_spec_sha256 registered_candidate_sha256 cases case_launch_accounting retries resumed unlaunched_case_labels launch_outcome_unknown_case_labels')
    for key in ('driver_schema_version', 'actual_processes_started', 'original_process_terminals', 'observed_process_terminal_memories', 'retries'): integer(terminal[key])
    boolean(terminal['source_inputs_unchanged']); boolean(terminal['resumed'])
    boolean(terminal['native_processes_started_count_is_complete']); boolean(terminal['prelaunch_directory_barriers_complete'])
    need(terminal['driver_schema_version'] == 2 and terminal['observed_process_terminal_memories'] == 18 and terminal['native_processes_started_count_is_complete'] is True and terminal['prelaunch_directory_barriers_complete'] is True and terminal['launch_outcome_unknown_case_labels'] == [], 'incomplete version-two launch/storage accounting')
    need(type(terminal['case_launch_accounting']) is list and len(terminal['case_launch_accounting']) == 18, 'missing per-case launch accounting')
    for index, entry in enumerate(terminal['case_launch_accounting']):
        keys(entry, 'case attempt_index status reservation_durable pid actual_exit_code process_terminal_storage storage_error')
        integer(entry['attempt_index']); integer(entry['pid']); integer(entry['actual_exit_code']); boolean(entry['reservation_durable'])
        need(entry['attempt_index'] == index and entry['case'] == CASES[index] and entry['status'] == 'known_started_exit_observed' and entry['reservation_durable'] is True and entry['actual_exit_code'] == 0 and entry['process_terminal_storage'] == 'stored_fsynced' and entry['storage_error'] is None, 'unknown/partial case launch or terminal durability')
    need(terminal['kind'] == 'fixed_eighteen_process_driver_terminal' and terminal['original_process_terminals'] == 18 and terminal['retries'] == 0 and terminal['resumed'] is False and terminal['error'] is None and terminal['unlaunched_case_labels'] == [], 'invalid driver terminal')
    need(terminal.get('status') == 'all_eighteen_processes_completed' and terminal.get('actual_processes_started') == 18, 'driver did not complete eighteen actual unique processes')
    need(terminal['source_inputs_unchanged'] is True and terminal['registered_spec_sha256'] == spec['_authenticated_wrapper_sha256'] and terminal['registered_candidate_sha256'] == spec['_authenticated_candidate_sha256'], 'missing/wrong driver source/spec guard')
    claim = loads((root / 'cohort-claim.json').read_bytes())
    keys(claim, 'kind created_utc cases registered_spec_sha256 registered_candidate_sha256 binary source_inputs')
    need(claim['kind'] == 'exclusive_fixed_eighteen_cohort_claim', 'wrong cohort claim')
    same(claim['cases'], CASES, 'wrong claimed order'); same(claim['binary'], spec['binary'], 'claim binary differs')
    same(claim['source_inputs'], spec['frozen_inputs'], 'claim sources differ')
    need(claim['registered_spec_sha256'] == spec['_authenticated_wrapper_sha256'] and claim['registered_candidate_sha256'] == spec['_authenticated_candidate_sha256'], 'claim registration differs')
    registered = datetime.fromisoformat(spec['registered_utc'])
    claimed = datetime.fromisoformat(claim['created_utc']); need(claimed.tzinfo and claimed >= registered, 'cohort claim preceded registration')
    checked_start = datetime.fromisoformat(command_receipt['started_utc'])
    checked_end = datetime.fromisoformat(command_receipt['ended_utc'])
    driver_end = datetime.fromisoformat(terminal['ended_utc'])
    need(checked_start.tzinfo and checked_end.tzinfo and driver_end.tzinfo and registered <= checked_start <= claimed <= driver_end <= checked_end, 'checked driver command chronology differs')
    durability = loads((root / 'prelaunch-durability.json').read_bytes())
    keys(durability, 'kind schema_version root_parent cohort_root completed_utc all_eighteen_claims_durable registered_spec_sha256')
    integer(durability['schema_version']); boolean(durability['all_eighteen_claims_durable'])
    need(durability['kind'] == 'completed_prelaunch_directory_barriers' and durability['schema_version'] == 2 and durability['all_eighteen_claims_durable'] is True and durability['root_parent'] == str(root.parent) and durability['cohort_root'] == str(root) and durability['registered_spec_sha256'] == spec['_authenticated_wrapper_sha256'], 'wrong prelaunch directory barrier record')
    barriers_at = datetime.fromisoformat(durability['completed_utc']); need(barriers_at.tzinfo and claimed <= barriers_at, 'invalid prelaunch barrier chronology')
    need(type(terminal['cases']) is list and len(terminal['cases']) == 18, 'wrong original terminal count')
    validate_collection_index(spec, root)
    runs, issues = [], []
    pids = []
    prelaunch_claim_times = []
    previous_end = claimed
    for index, case in enumerate(CASES):
        folder = root / case['label']
        launch = loads((folder / 'launch-claim.json').read_bytes())
        result = loads((folder / 'process-terminal.json').read_bytes())
        keys(launch, 'kind case attempt_index argv cwd binary created_utc registered_spec_sha256')
        keys(result, 'kind case attempt_index argv cwd started_utc native_process_started actual_exit_code timed_out error pid ended_utc stdout stderr raw_output')
        expected_argv = [spec['binary']['path'], '--profile', case['profile'], '--target', case['target'], '--observer', case['observer'], '--label', case['label'], '--output', str(folder / 'raw.jsonl.gz')]
        for obj in (launch, result):
            integer(obj['attempt_index']); need(obj['attempt_index'] == index, 'wrong process ordinal')
            same(obj['argv'], expected_argv, 'wrong original argv'); need(obj['cwd'] == spec['cwd'], 'wrong original cwd')
        need(launch['kind'] == 'exclusive_process_claim' and launch['registered_spec_sha256'] == spec['_authenticated_wrapper_sha256'], 'wrong prelaunch claim')
        same(launch['binary'], spec['binary'], 'prelaunch binary differs')
        boolean(result['native_process_started']); boolean(result['timed_out']); integer(result['actual_exit_code']); integer(result['pid'])
        need(result['kind'] == 'original_process_terminal' and not result['timed_out'] and result['error'] is None, 'failed/timed-out native process')
        start_time = datetime.fromisoformat(result['started_utc']); end_time = datetime.fromisoformat(result['ended_utc'])
        claim_time = datetime.fromisoformat(launch['created_utc'])
        prelaunch_claim_times.append(claim_time)
        need(start_time.tzinfo and end_time.tzinfo and claim_time.tzinfo and claimed <= claim_time <= start_time <= end_time and previous_end <= start_time, 'bad sequential process chronology')
        previous_end = end_time; pids.append(result['pid'])
        need(barriers_at <= start_time and terminal['case_launch_accounting'][index]['pid'] == result['pid'], 'process started before directory barriers or PID differs')
        intent = loads((folder / 'launch-intent.json').read_bytes())
        keys(intent, 'kind case attempt_index argv created_utc registered_spec_sha256')
        integer(intent['attempt_index']); intent_at = datetime.fromisoformat(intent['created_utc'])
        need(intent['kind'] == 'durable_launch_intent' and intent['case'] == case and intent['attempt_index'] == index and intent['registered_spec_sha256'] == spec['_authenticated_wrapper_sha256'] and intent_at.tzinfo and start_time <= intent_at <= end_time, 'bad durable launch intent')
        same(intent['argv'], expected_argv, 'launch intent argv differs')
        same(terminal['cases'][index], result, 'driver terminal differs from original terminal')
        started_record = loads((folder / 'process-started.json').read_bytes())
        expected_started = dict(result)
        expected_started.update(actual_exit_code=None, timed_out=False, error=None,
            ended_utc=None, stdout=None, stderr=None, raw_output=None)
        same(started_record, expected_started, 'original process-start receipt differs')
        need(launch.get('case') == case and result.get('case') == case and result.get('actual_exit_code') == 0, 'failed/mismatched original process exit')
        need(result.get('native_process_started') is True, 'process was not started')
        for key in ('stdout', 'stderr', 'raw_output'): bound(result[key])
        for key, filename in [('stdout', 'stdout.log'), ('stderr', 'stderr.log'), ('raw_output', 'raw.jsonl.gz')]:
            need(result[key]['path'] == str(folder / filename), 'substituted process artifact path')
        check_process_guards(spec, case, index, folder, start_time, end_time)
        run = validate_run(folder / 'raw.jsonl.gz', case)
        runs.append(run)
        issues.extend(analysis_issues(run))
    need(len(pids) == len(set(pids)), 'native process IDs reused within bounded eighteen-process cohort')
    first_start = datetime.fromisoformat(terminal['cases'][0]['started_utc'])
    need(all(t <= first_start for t in prelaunch_claim_times), 'not all eighteen process claims preceded the first launch')
    for index in range(0, 18, 3):
        group = runs[index:index+3]
        for item in group[1:]:
            same(item['initial'], group[0]['initial'], 'ON/repeat/OFF initial checkpoint differs')
            same(item['completed'], group[0]['completed'], 'ON/repeat/OFF completed checkpoint differs')
        same(group[0]['telemetry'], group[1]['telemetry'], 'fresh ON telemetry typed value differs')
        need(group[0]['literal_telemetry_bytes'] == group[1]['literal_telemetry_bytes'], 'fresh ON completed-record bytes differ')
    for item in spec['frozen_inputs']: bound(item)
    bound(spec['binary'])
    discriminator, cross_issues = discriminate(runs, issues)
    issues.extend(cross_issues)
    return dict(kind='strict_motor_visit_placement_cohort_qualification',
        fidelity_status='PASS_COMPLETE_DECLARED_SCOPE' if not issues else 'FAILED_OR_EXCLUDED_ARITHMETIC_RETAINED',
        discriminator=discriminator,
        physical_SI_status='DIAGNOSTICS_ONLY_NO_ACCURACY_VERDICT',
        actual_processes=18, actual_outer_intervals=18, observed_ON_processes=12,
        construction_events=sum(32 * len(schedules(p)[0]) + 1 for p in PROFILES) * 4,
        stage_events=sum(32 * len(schedules(p)[1]) for p in PROFILES) * 4,
        constructor_checks=12 * 32, coordinate_advance_checks=12 * 32,
        native_motor_solve_checks=sum(sum(PROFILES[p]) * 32 for p in PROFILES) * 4,
        issues=issues,
        cases=[dict(case=r['case'], framing=r['framing'], analyses=r['analyses']) for r in runs],
        source_scope=spec['source_scope'], source_inputs_unchanged=True,
        captured_checkpoint_ON_repeat_OFF_exact=True, fresh_ON_complete_record_literal_exact=True,
        off_observers_absent=True, native_motor_accuracy_verified=False,
        old_calibration_reclassified=False, normalization_intervention=False,
        original_checked_CLI_exit_verified=0, original_checked_CLI_receipt=outer_binding,
        exclusions=['Full native sets/private pipeline/contact caches', 'runtime MXCSR/FTZ/DAZ/rounding',
                    'mass-matrix/LU and external-acceleration construction', 'impulse-v1 promoted aggregate arithmetic',
                    'SI accuracy verdict', 'coupled-contact causal attribution', 'production correction/adoption', 'maturity'])


def audit_once(spec_path, expected_sha, result_root, output, checked_outer_receipt, checked_outer_sha256):
    """Claim this audit before processing collection input; never retry implicitly."""
    out = persistent(str(Path(output)), 'audit output must be canonical persistent /workspace path')
    need(out.parent.is_dir() and not out.exists(), 'audit output parent missing or output already exists')
    cohort_root = persistent(str(Path(result_root)), 'cohort root must be canonical persistent /workspace path')
    need(cohort_root.is_dir(), 'cohort root is absent')
    # One fixed claim per cohort blocks reanalysis using a different output path.
    claim = cohort_root / 'reader-audit-claim.json'
    write_exclusive(claim, dict(kind='exclusive_once_only_visit_placement_audit',
        schema_version=1, created_utc=datetime.now(timezone.utc).isoformat(),
        spec_path=str(spec_path), externally_supplied_wrapper_sha256=expected_sha,
        result_root=str(result_root), output=str(out),
        checked_outer_receipt=str(checked_outer_receipt), checked_outer_sha256=checked_outer_sha256,
        reader_source=descriptor(Path(__file__).resolve()), native_processes_launched=0))
    try:
        spec = load_spec(spec_path, expected_sha)
        result = qualify_cohort(spec, result_root, checked_outer_receipt, checked_outer_sha256)
    except Exception as error:
        result = dict(kind='strict_motor_visit_placement_cohort_qualification',
            fidelity_status='FAILED_SCHEMA_BINDING_OR_PARTIAL_COLLECTION_RETAINED',
            discriminator_status='UNQUALIFIED', physical_SI_status='UNQUALIFIED',
            error=type(error).__name__ + ': ' + str(error), original_input_files_modified=False,
            native_motor_accuracy_verified=False, registered_wrapper_sha256=expected_sha)
    write_exclusive(out, result)
    return 0 if result['fidelity_status'] == 'PASS_COMPLETE_DECLARED_SCOPE' else 2


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec',required=True);parser.add_argument('--spec-sha256',required=True)
    parser.add_argument('--result-root',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--checked-outer-receipt',required=True);parser.add_argument('--checked-outer-sha256',required=True)
    args=parser.parse_args()
    return audit_once(args.spec,args.spec_sha256,args.result_root,args.output,args.checked_outer_receipt,args.checked_outer_sha256)


if __name__=='__main__':
    raise SystemExit(main())
