"""Read-only retained receipt/opaque-byte audit; never invokes native collectors."""
import datetime
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import stat
import subprocess

P = Path('/workspace/.rne-onboarding/recovery-20261012/next-motor-source-transition-72')
OUT = P / 'independent-postcollection-review'
FROZEN_SHA = '6a99b2ccb8573c1141369c5498bcea6e9bc8f61f62189988066cd7f020f2270b'
PREPARED_SHA = '50b9db35f54a3b3e8803ce5c40d65a4332ca1b3d17037b3aeb3ae1151f9360ff'
BASELINE_SHA = 'db0dee42a3c9067b38241112d5ee402341c0729466d7fc22f12f31504000db76'
REPORT_SHA = 'bf6a721ca92d609c2e80df2e3b7f1bddeb79fcb5974934351d615a88f18db662'

def require(condition, message):
    if not condition:
        raise RuntimeError(message)

def read(path):
    path = Path(path)
    require(path.is_absolute() and path == path.resolve(), 'noncanonical or symlink path')
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        before = os.fstat(fd)
        require(stat.S_ISREG(before.st_mode), 'nonregular input')
        with os.fdopen(os.dup(fd), 'rb') as stream:
            data = stream.read()
        after = os.fstat(fd)
        require((before.st_ino, before.st_size, before.st_mtime_ns, before.st_ctime_ns) ==
                (after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns), 'input changed during read')
        require(len(data) == after.st_size, 'short read')
        return data
    finally:
        os.close(fd)

def desc(path):
    data = read(path)
    return {'path': str(path), 'sha256': hashlib.sha256(data).hexdigest(), 'size_bytes': len(data)}

def check(expected):
    require(desc(expected['path']) == expected, 'descriptor mismatch: ' + expected['path'])
    return expected

def load(path):
    return json.loads(read(path))

def time(value):
    return datetime.datetime.fromisoformat(value.replace('Z', '+00:00'))

fpath = P / 'frozen-v2/frozen-bindings.json'
fd = desc(fpath)
require(fd['sha256'] == FROZEN_SHA and fd['size_bytes'] == 21977, 'frozen mismatch')
frozen = load(fpath)
check(frozen['prepared']); check(frozen['independent_review'])
require(frozen['prepared']['sha256'] == PREPARED_SHA and frozen['prepared']['size_bytes'] == 20444, 'prepared mismatch')
prepared = load(frozen['prepared']['path'])
require(prepared == frozen['prepared_binding'], 'embedded prepared differs')
review = load(frozen['independent_review']['path'])
require(review['kind'] == 'rne_141_independent_precollection_review' and
        review['status'] == 'PASS_INDEPENDENT_PRE_COLLECTION_SOURCE_TRANSITION_REVIEW' and
        review['before_world_collection'] is True and review['world_budget'] == 72 and
        review['reviewed_prepared_binding_sha256'] == PREPARED_SHA and
        review['reviewed_prepared_binding_size_bytes'] == 20444, 'admission mismatch')
inputs = prepared['inputs']; artifacts = inputs['artifacts']
require(len(artifacts) == 43, 'wrong artifact count')
initial_artifacts = {role: check(d) for role, d in artifacts.items()}
check(prepared['supplied_inputs'])
require(load(prepared['supplied_inputs']['path']) == inputs, 'supplied input differs')

terminal = load(P / 'collection/terminal.json')
outer = load(P / 'outer-command/actual-command.json')
claim = load(P / 'collection/claim.json')
reservation = load(P / 'collection/collector-launch-reservation.json')
require(terminal['status'] == 'PASS_FIXED72_LITERAL_SOURCE_TRANSITION_RETAINED_FAIL0_OF4' and
        terminal['collector_processes_started'] == 1 and terminal['collector_original_exit_code'] == 2 and
        terminal['postcollection_processes_started'] == 3 and terminal['retry_allowed'] is False and
        terminal['source_inputs_unchanged'] is True, 'main terminal mismatch')
require(terminal['world_accounting'] == {'case_count_retained':24,'status':'COMPLETE_FIXED72_FAIL0_OF4',
        'world_counts':{'completed_worlds':72,'worlds_attempted':72,'worlds_created':72}}, 'accounting receipt differs')
require(claim['frozen'] == fd and terminal['frozen'] == fd and claim['world_attempt_budget'] == 72 and
        claim['collector_process_budget'] == 1 and claim['retry_allowed'] is False, 'claim differs')
require(reservation['actual_processes_started'] is None and reservation['world_counts'] is None and
        reservation['retry_allowed'] is False, 'reservation does not preserve unknown')
require(time(prepared['created_at']) < time(review['reviewed_utc']) < time(frozen['created_at']) <
        time(outer['started_at']) < time(claim['created_at']) < time(terminal['started_at']) <
        time(reservation['created_at']), 'preclaim chronology differs')
require(outer['actual_exit_code'] == 0 and outer['argv'] ==
        ['/usr/bin/python3','-B',artifacts['driver_source']['path'],'run','--frozen',str(fpath),
         '--expected-frozen-sha256',FROZEN_SHA] and outer['cwd'] == inputs['source_root'], 'outer argv/exit differs')
check(outer['stdout']); check(outer['stderr'])
require(load(outer['stdout']['path']) == terminal and read(outer['stderr']['path']) == b'', 'outer logs differ')
require(outer['native_accounting_authority'] == str(P/'collection/terminal.json') and
        outer['native_worlds_not_inferred_from_external_python_process_count'] is True, 'outer accounting scope differs')

binary = artifacts['binary']['path']; report_path = str(P/'collection/report.json')
plans = [
 ('collection',[binary,'--output',report_path,'--require-calibration'],2),
 ('verify-current',[binary,'--verify-report',report_path],0),
 ('verify-current-require',[binary,'--verify-report',report_path,'--require-calibration'],2),
 ('verify-old-stale',[binary,'--verify-report',artifacts['baseline_report']['path']],1),
]
require(reservation['argv'] == plans[0][1], 'reservation argv mismatch')
processes = []
last_finish = time(reservation['created_at'])
expected_collect = b'collection_complete=true completed_worlds=72/72 replay_qualification=true primary_calibration=FAIL mode=collect\n'
expected_verify = b'collection_complete=true completed_worlds=72/72 replay_qualification=true primary_calibration=FAIL mode=verify-report-no-worlds\n'
for index, (phase, argv, exitcode) in enumerate(plans):
    folder = P/'collection'/phase
    start, pid, actual = (load(folder/name) for name in ('start.json','pid.json','terminal.json'))
    state = terminal['collector_launch_state'] if index == 0 else terminal['postcollection_launch_states'][index-1]
    require(state['returned_terminal'] == actual and state['actual_process_launched'] is True and
            state['launch_reserved'] is True and state['terminal_written'] is True and
            state['observed_pid'] == pid['pid'] and state['observed_wait_return'] == exitcode, 'state/receipt mismatch')
    require(actual['argv'] == argv and actual['original_exit_code'] == exitcode and
            actual['process_count'] == 1 and actual['process_launched'] is True and
            actual['environment'] == {'LANG':'C','LC_ALL':'C'} and actual['cwd'] == inputs['source_root'], 'process contract mismatch')
    require(all(actual[k] is None for k in ('cleanup_error','execution_error','launch_error','pid_receipt_error')), 'process error')
    require(all(actual[k] == value for k, value in start.items()), 'start receipt differs')
    require(last_finish < time(actual['started_at']) <= time(pid['started_at']) < time(actual['finished_at']), 'process chronology differs')
    last_finish = time(actual['finished_at'])
    check(actual['stdout']); check(actual['stderr'])
    stdout, stderr = read(actual['stdout']['path']), read(actual['stderr']['path'])
    require(stdout == (expected_collect if index == 0 else expected_verify if index < 3 else b''), 'full stdout differs')
    require(stderr == (b'' if index < 3 else b'position-motor calibration: report selected sources are stale/mixed\n'), 'full stderr differs')
    require(actual['world_role'] == ('one collector; internal fixed24 x ON/fresh_ON/OFF; max72 world attempts' if index == 0
                                    else 'zero worlds: --verify-report only'), 'role mismatch')
    processes.append({'phase':phase,'pid':pid['pid'],'argv':argv,'original_exit_code':exitcode,
                      'start':desc(folder/'start.json'),'pid_receipt':desc(folder/'pid.json'),
                      'terminal':desc(folder/'terminal.json'),'stdout':actual['stdout'],'stderr':actual['stderr'],
                      'started_at':actual['started_at'],'finished_at':actual['finished_at'],
                      'world_scope':'fixed72 collector' if index == 0 else 'loaded-report verification; no worlds'})
require(len({item['pid'] for item in processes}) == 4, 'PID reuse in receipts')
require(last_finish < time(terminal['finished_at']) < time(outer['completed_at']), 'terminal chronology differs')
require(sorted(path.parent.name for path in (P/'collection').glob('*/start.json')) == sorted(x[0] for x in plans), 'extra registered child starts')

before_path, after_path = P/'collection/source-before.json', P/'collection/source-after.json'
before, after = load(before_path), load(after_path)
guard = load(artifacts['source_guard']['path'])
require(read(before_path) == read(after_path) == read(artifacts['source_guard']['path']), 'full source guards differ')
require(len(guard['inputs']) == 1344 and guard['source_root'] == inputs['source_root'] and
        'non-exhaustive' in guard['scope'], 'guard scope differs')
source_root = Path(inputs['source_root'])
source_actual = {}
for row in guard['inputs']:
    d = desc(source_root/row['path'])
    require({k:d[k] for k in ('sha256','size_bytes')} == {k:row[k] for k in ('sha256','size_bytes')}, 'current source bytes differ: '+row['path'])
    source_actual[row['path']] = {'sha256':d['sha256'],'size_bytes':d['size_bytes']}
raw = subprocess.check_output(['git','ls-files','-z','--cached','--others','--exclude-standard'], cwd=source_root,
                              env=dict(os.environ,GIT_OPTIONAL_LOCKS='0'))
names = {item.decode('utf-8') for item in raw.split(b'\0') if item}
selected = {name for name in names if name.endswith(('.rs','.toml')) or Path(name).name in {'Cargo.lock','rust-toolchain'} or name.startswith('.cargo/')}
for current, directories, files in os.walk(source_root/'third_party/rapier3d'):
    directories[:] = sorted(name for name in directories if name not in {'target','.git'})
    for name in sorted(files):
        if name.endswith(('.rs','.toml')) or name == 'Cargo.lock':
            selected.add(str((Path(current)/name).relative_to(source_root)))
require(sorted(selected) == [row['path'] for row in guard['inputs']], 'current source path set differs')
build_guard = load(artifacts['original_build_guard']['path'])
require(len(build_guard) == 1343 and all(source_actual.get(path) == d for path,d in build_guard.items()), 'original compiler guard differs')
require(sorted(set(source_actual)-set(build_guard)) == ['third_party/rapier3d/Cargo.lock'], 'stronger guard addition differs')
for path, sha in inputs['transition']['after_source_sha256'].items():
    require(source_actual[path]['sha256'] == sha, 'selected source map differs')

check(terminal['report'])
require(terminal['report']['sha256'] == REPORT_SHA and terminal['report']['size_bytes'] == 8193579, 'report differs')
require(artifacts['baseline_report']['sha256'] == BASELINE_SHA and artifacts['baseline_report']['size_bytes'] == 8193438, 'old baseline binding differs')
old, new = read(artifacts['baseline_report']['path']), read(report_path)
# The frozen comparator validates JSON framing/types and source maps. Its numeric
# tokens are opaque Decimal objects; no native arithmetic/calibration is invoked.
spec = importlib.util.spec_from_file_location('frozen_literal_comparator', artifacts['comparator_source']['path'])
comparator = importlib.util.module_from_spec(spec); spec.loader.exec_module(comparator)
proof = comparator.compare_reports(old,new,inputs['transition'])
require(proof == load(P/'collection/literal-comparison.json'), 'reproduced frozen proof differs')
a,b = comparator._source_span(old); c,d = comparator._source_span(new)
require(a == c and old[:a] == new[:c] and old[b:] == new[d:], 'independent prefix/suffix byte comparison fails')
require(old[a:b] == comparator.canonical_source_value(inputs['transition']['before_source_sha256']) and
        new[c:d] == comparator.canonical_source_value(inputs['transition']['after_source_sha256']), 'literal source encoding differs')
require(new == old[:a] + new[c:d] + old[b:], 'independent single-source splice differs')

# Recheck every frozen input and collection-evidence bytes after the audit.
final_artifacts = {role:check(expected) for role,expected in initial_artifacts.items()}
require(final_artifacts == initial_artifacts, 'bound artifact changed')
check(fd); check(frozen['prepared']); check(frozen['independent_review']); check(terminal['report'])
for row in guard['inputs']:
    value=desc(source_root/row['path'])
    require(value['sha256'] == row['sha256'] and value['size_bytes'] == row['size_bytes'], 'source changed during audit')

receipt = {
 'kind':'rne_141_independent_postcollection_source_transition_review','schema_version':1,
 'status':'PASS_INDEPENDENT_POST_COLLECTION_LITERAL_SOURCE_TRANSITION_RETAINED_FAIL0_OF4',
 'reviewed_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
 'review_scope':'Read-only retained metadata, complete opaque file hashes and frozen source-only byte comparison. No collector, native verifier, Cargo, world, numerical recalibration, source edit or remote operation.',
 'frozen_binding':fd,'prepared_binding':frozen['prepared'],'independent_predata_admission':frozen['independent_review'],
 'guard_admission_chronology':{'prepared_at':prepared['created_at'],'independent_review_at':review['reviewed_utc'],
   'frozen_at':frozen['created_at'],'outer_started_at':outer['started_at'],'claim_at':claim['created_at'],
   'reservation_at':reservation['created_at'],'final_terminal_at':terminal['finished_at'],'outer_completed_at':outer['completed_at'],
   'all_ordered':True},
 'collection':{'outer_actual_exit_code':0,'native_original_exit_code':2,'collector_processes_started':1,
   'world_budget':72,'cases_retained':24,'worlds_created':72,'worlds_attempted':72,'completed_worlds':72,
   'accounting_authority':desc(P/'collection/terminal.json'),'claim':desc(P/'collection/claim.json'),
   'launch_reservation':desc(P/'collection/collector-launch-reservation.json'),'retry_allowed':False,
   'world_count_check_scope':'Retained native accounting/stdout cross-checked against fixed frozen driver; independent native execution or numerical payload accounting was not performed.',
   'outer_receipt':desc(P/'outer-command/actual-command.json'),'outer_stdout':outer['stdout'],'outer_stderr':outer['stderr']},
 'actual_process_receipts':processes,'loaded_verification_original_exits':[0,2,1],
 'source_guards':{'before':desc(before_path),'after':desc(after_path),'bound':artifacts['source_guard'],
   'all_bytes_equal':True,'guarded_inputs':1344,'current_all_hashes_sizes_and_path_set_match':True,
   'current_source_rechecked_after_review':True,'original_compiler_guard':artifacts['original_build_guard'],
   'original_compiler_guard_matching_inputs':1343,'stronger_addition':['third_party/rapier3d/Cargo.lock'],
   'scope':guard['scope'],'later_source_changes_require_fresh_binding':True},
 'bound_artifacts':{'count':43,'all_size_sha256_checks_before_and_after_pass':True,'inputs':initial_artifacts},
 'literal_report_comparison':{'baseline':artifacts['baseline_report'],'new_report':terminal['report'],
   'frozen_comparator':artifacts['comparator_source'],'recorded_proof':desc(P/'collection/literal-comparison.json'),
   'independently_reproduced_proof':proof,'baseline_source_span':[a,b],'new_source_span':[c,d],
   'prefix_size_bytes':a,'suffix_size_bytes':len(old)-b,'prefix_and_suffix_literal_bytes_equal':True,
   'whole_report_equals_exact_one_source_object_splice':True,'source_count_before':21,'source_count_after':22,
   'existing_changed_paths_count':11,'added_paths_count':1,'removed_paths_count':0,
   'numerical_reanalysis_performed':False,'baseline_full_json_parser_invoked_only_by_frozen_source_comparator':True,
   'fail_0_of_4_and_zero_target_8_of_8_preserved':'All non-source report bytes, including retained findings and zero-target findings, are byte-identical. No recalibration or promotion to accuracy PASS.'},
 'durability_source_review':{'driver':artifacts['driver_source'],'exclusive_receipt_file_and_parent_fsync_lines':[80,93],
   'persistent_once_claim_before_collector_lines':[502,509],'source_before_and_launch_reservation_lines':[523,538],
   'collector_report_fsync_and_loaded_gates_lines':[541,584],'finally_source_after_report_and_terminal_fsync_lines':[614,643],
   'process_log_flush_fsync_lines':[409,412],'all_four_formed_process_receipts_have_null_storage_execution_cleanup_errors':True,
   'claim_and_receipts_retained':True,'limitation':'Source/terminal agreement establishes successful documented calls; this review is not kernel syscall tracing or an independent crash/restart durability experiment.'},
 'admission_result':'Suitable for retaining the complete new report as a current-source literal-equivalent FAIL report. Not evidence of motor accuracy improvement or companion143 candidate-six admission.',
 'publication_scope':'Review precedes any report copy or later docs/xtask changes. External receipts and binaries are locally retained; this review does not publish them.',
 'material_blockers':[],
 'exclusions':['No new worlds, builds, calibration, native report-verifier invocations or hidden-state examination.',
   'No causal neutrality inference beyond the source-bound literal report equality for this fixed72 arm.',
   'No new independent audit of native numeric or physical calibration calculations.',
   'No promotion of old failed cohorts or permission for later candidate-six collection.',
   'Once-only conclusion is for this durable registered run directory and receipts, not global proof of all machine activity.',
   'Hashes bind bytes and timestamps record chronology; they do not supply cryptographic actor identity.'],
 'review_tool':desc(Path(__file__).resolve()),
}
path=OUT/'independent-postcollection-source-transition-review.json'
with path.open('xb') as stream:
    stream.write((json.dumps(receipt,indent=2,sort_keys=True)+'\n').encode());stream.flush();os.fsync(stream.fileno())
print(json.dumps(desc(path),sort_keys=True))
print('PASS: 43 inputs; 1344 source hashes before/after; one original exit2 collector; 72 worlds; loaded 0/2/1; exact non-source bytes retained; no new native invocation.')
