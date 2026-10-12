#!/usr/bin/python3
"""NEW18 retention/lineage review only; never execute the scientific reader/native."""
from datetime import datetime,timezone
import hashlib,json
from pathlib import Path
import subprocess,sys
OWN=Path(__file__).resolve().parent
REC=OWN.parent;STUDY=REC/'next-motor-visit-placement-18';ROOT=Path('/workspace/RobotNativeEngine')
DRIVER=REC/'next-motor-visit-placement-driver'
assert hashlib.sha256((DRIVER/'strict_io.py').read_bytes()).hexdigest()=='27ad4b0d7351f93c4c378f3f41f69ecbb917569594cd1b980a2a7d6bbae07abc'
sys.path.insert(0,str(DRIVER))
from strict_io import bound,canonical,descriptor,gzip_records,integer,loads,write_exclusive

def read(p):return loads(Path(p).read_bytes())
def same(a,b):assert canonical(a)==canonical(b)
def when(s):
    t=datetime.fromisoformat(s);assert t.tzinfo is not None;return t

def word(v,width=32):integer(v,width)
def git_read(argv):
    p=subprocess.run(['git',*argv],cwd=ROOT,capture_output=True,check=False)
    assert p.returncode==0
    return dict(argv=['git',*argv],cwd=str(ROOT),original_exit=0,stdout=p.stdout.decode(),stderr=p.stderr.decode())

candidate_path=STUDY/'frozen-candidate.json';s=read(candidate_path)
assert descriptor(candidate_path)['sha256']=='14566a9ffbe6a6e89b1ead544385f85c22782d26ce9a9b3514e9f8bc2a6d558c'
wrapper_path=STUDY/'admitted-wrapper.json';wrapper=read(wrapper_path)
wrapper_sha=descriptor(wrapper_path)['sha256'];assert wrapper_sha=='552f76408fb4abe82793e66aa75d685185709b9c3676d096d7d0bcce78c16f1e'
bound(wrapper['candidate']);bound(wrapper['admission']);admission=read(wrapper['admission']['path'])
assert wrapper['candidate']==descriptor(candidate_path) and admission['status']=='PASS_PRE_DATA_SOURCE_PROTOCOL_AND_DRIVER'
assert wrapper['admission']['sha256']=='d258b81d19d83a4c1d8b1d65c798a55fb1e63e7dc279cd010039a7e7005dfa2f'
assert admission['reviewed_candidate_sha256']==wrapper['candidate']['sha256'] and admission['binary']==s['binary']
assert admission['frozen_inputs_sha256']==hashlib.sha256(canonical(s['frozen_inputs'])).hexdigest()
assert len(s['frozen_inputs'])==508
for d in s['frozen_inputs']:bound(d)
bound(s['binary']);bound(s['driver_interpreter'])
assert s['collection_worlds_before_registration']==0 and len(s['cases'])==18
registered=when(s['registered_utc']);admitted=when(admission['created_utc']);assert registered<=admitted
root=Path(s['output_root']);assert root==STUDY/'collection'
collect=read(STUDY/'collect-invocation/actual-command.json');audit_command=read(STUDY/'audit-invocation/actual-command.json')
checked=read(s['driver_run_receipt_path']);terminal=read(root/'driver-terminal.json')
for mode,r in [('collect',collect),('audit',audit_command)]:
    assert r['mode']==mode and r['kind']=='exclusive_original_cli_claim' and r['original_exit']==0
    assert r['process_started'] and r['launch_outcome']=='known_started_exit_observed' and r['error'] is None
    assert r['source_inputs_unchanged'] and r['retries']==0 and r['resume'] is False
    assert r['registered_wrapper_sha256']==wrapper_sha and r['cwd']==s['cwd']
    for name in ('stdout','stderr'):bound(r[name]);assert r[name]['path']==str(STUDY/(mode+'-invocation')/(name+'.log'))
    claim=read(STUDY/(mode+'-invocation')/'command-claim.json')
    assert claim['command']==r['command'] and claim['created_utc']==r['created_utc']
    started=read(STUDY/(mode+'-invocation')/'process-started.json')
    assert started['pid']==r['pid'] and started['launch_outcome']=='known_started_exit_unknown' and started['original_exit'] is None
    assert r['start_unix_ns']<=r['end_unix_ns']
assert collect['command']==[s['driver_interpreter']['path'],'-B',s['checked_driver_source']['path'],'--spec',str(wrapper_path),'--spec-sha256',wrapper_sha,'--launch-native']
assert checked['actual_exit_code']==0 and checked['driver_process_started'] and checked['launch_outcome']=='known_started_exit_observed'
assert checked['source_inputs_unchanged'] and checked['runtime_error'] is checked['source_guard_error'] is None
assert checked['registered_wrapper_sha256']==wrapper_sha and checked['registered_candidate_sha256']==wrapper['candidate']['sha256']
for name in ('stdout','stderr'):bound(checked[name])
same(loads(Path(collect['stdout']['path']).read_bytes().splitlines()[-1]),checked)
same(loads(Path(checked['stdout']['path']).read_bytes().splitlines()[-1]),terminal)
assert terminal['status']=='all_eighteen_processes_completed' and terminal['actual_processes_started']==18
assert terminal['original_process_terminals']==terminal['observed_process_terminal_memories']==18
assert terminal['native_processes_started_count_is_complete'] and terminal['prelaunch_directory_barriers_complete'] and terminal['source_inputs_unchanged']
assert terminal['retries']==0 and terminal['resumed'] is False and terminal['error'] is None
assert terminal['unlaunched_case_labels']==terminal['launch_outcome_unknown_case_labels']==[]
assert len(terminal['cases'])==len(terminal['case_launch_accounting'])==18
assert admitted<=when(collect['created_utc'])<=when(checked['started_utc'])<=when(terminal['ended_utc'])<=when(checked['ended_utc'])
assert collect['end_unix_ns']<=audit_command['start_unix_ns']
claim=read(root/'cohort-claim.json');same(claim['cases'],s['cases']);same(claim['source_inputs'],s['frozen_inputs']);same(claim['binary'],s['binary'])
assert claim['registered_spec_sha256']==wrapper_sha and claim['registered_candidate_sha256']==wrapper['candidate']['sha256']
barrier=read(root/'prelaunch-durability.json');assert barrier['all_eighteen_claims_durable'] and barrier['registered_spec_sha256']==wrapper_sha
index=read(root/'collection-index.json');assert len(index['entries'])==18 and index['registered_spec_sha256']==wrapper_sha
case_dirs=sorted(p.name for p in root.iterdir() if p.is_dir());assert case_dirs==sorted(c['label'] for c in s['cases'])
raws=[];pids=[];before_claim_times=[];previous_end=when(claim['created_utc'])
exact_case_files={'launch-claim.json','launch-intent.json','process-started.json','process-terminal.json','raw.jsonl.gz','source-guard-before.json','source-guard-after.json','stdout.log','stderr.log'}
for i,c in enumerate(s['cases']):
    folder=root/c['label'];assert {p.name for p in folder.iterdir()}==exact_case_files
    r=read(folder/'process-terminal.json');lc=read(folder/'launch-claim.json');intent=read(folder/'launch-intent.json')
    same(r,terminal['cases'][i]);same(r['case'],c);same(lc['case'],c);same(intent['case'],c)
    assert r['attempt_index']==lc['attempt_index']==intent['attempt_index']==i
    argv=[s['binary']['path'],'--profile',c['profile'],'--target',c['target'],'--observer',c['observer'],'--label',c['label'],'--output',str(folder/'raw.jsonl.gz')]
    same(r['argv'],argv);same(lc['argv'],argv);same(intent['argv'],argv);same(index['entries'][i]['argv'],argv)
    assert r['cwd']==lc['cwd']==s['cwd'] and r['native_process_started'] and r['actual_exit_code']==0
    assert r['error'] is None and r['timed_out'] is False
    start,end=when(r['started_utc']),when(r['ended_utc']);assert previous_end<=start<=end
    previous_end=end;pids.append(r['pid']);word(r['pid'],64)
    lc_time=when(lc['created_utc']);before_claim_times.append(lc_time)
    assert when(claim['created_utc'])<=lc_time<=start<=when(intent['created_utc'])<=end
    assert when(barrier['completed_utc'])<=start
    started=read(folder/'process-started.json');assert started['pid']==r['pid'] and started['actual_exit_code'] is None
    for key,name in [('stdout','stdout.log'),('stderr','stderr.log'),('raw_output','raw.jsonl.gz')]:
        bound(r[key]);assert r[key]['path']==str(folder/name)
    accounting=terminal['case_launch_accounting'][i]
    assert accounting['status']=='known_started_exit_observed' and accounting['actual_exit_code']==0 and accounting['pid']==r['pid']
    assert accounting['reservation_durable'] and accounting['process_terminal_storage']=='stored_fsynced' and accounting['storage_error'] is None
    for phase in ('before','after'):
        g=read(folder/('source-guard-'+phase+'.json'));same(g['frozen_inputs'],s['frozen_inputs']);same(g['registration_artifacts'],s['registration_artifacts']);same(g['binary'],s['binary']);same(g['case'],c)
        assert g['attempt_index']==i and g['registered_spec_sha256']==wrapper_sha
        assert when(g['verified_utc'])<=start if phase=='before' else when(g['verified_utc'])>=end
    records,lines,framing=gzip_records(folder/'raw.jsonl.gz');assert len(records)==4
    a,b,d,e=records;assert [r['kind'] for r in records]==['run_start','initial_state','completed_interval','run_end']
    assert a['study_id']=='motor_visit_placement_v1' and a['profile']==c['profile'] and a['run_label']==c['label'] and a['target']==c['target']
    assert a['observer_enabled'] is (c['observer']=='on') and a['expected_outer_intervals']==d['completed_outer_interval']==1
    assert a['native_scalar_bits']==32 and e['status']=='collected' and e['error'] is None and e['records_written_before_terminal']==3
    assert e['progress']['completed_outer_intervals']==1 and e['progress']['partial_records_written']==0 and e['progress']['pipeline_step_returned']
    if c['observer']=='off':assert all(d[k] is None for k in ('construction_companion','stage_v1','impulse_v1_ieee_words'))
    raws.append(dict(case=c,records=records,lines=lines,framing=framing))
assert len(pids)==len(set(pids))==18
assert all(t<=when(terminal['cases'][0]['started_utc']) for t in before_claim_times)
for i in range(0,18,3):
    on,fresh,off=raws[i:i+3]
    same(on['records'][1]['checkpoint'],fresh['records'][1]['checkpoint']);same(on['records'][1]['checkpoint'],off['records'][1]['checkpoint'])
    same(on['records'][2]['checkpoint'],fresh['records'][2]['checkpoint']);same(on['records'][2]['checkpoint'],off['records'][2]['checkpoint'])
    assert on['lines'][2]==fresh['lines'][2]
audit_path=STUDY/'actual-cohort-audit.json';assert descriptor(audit_path)==dict(path=str(audit_path),sha256='d7022bb982ab997862da1f3aabbfa4306a6fa568b4707a2b2a9aa0bd9581acfe',bytes=5447072)
audit=read(audit_path)
assert audit['fidelity_status']=='PASS_COMPLETE_DECLARED_SCOPE' and audit['issues']==[]
assert audit['actual_processes']==audit['actual_outer_intervals']==18 and audit['observed_ON_processes']==12
assert audit['constructor_checks']==audit['coordinate_advance_checks']==384 and audit['native_motor_solve_checks']==896
assert audit['construction_events']==2572 and audit['stage_events']==8192
assert audit['original_checked_CLI_exit_verified']==0 and audit['original_checked_CLI_receipt']==descriptor(STUDY/'collect-invocation/actual-command.json')
assert audit['captured_checkpoint_ON_repeat_OFF_exact'] and audit['fresh_ON_complete_record_literal_exact'] and audit['off_observers_absent'] and audit['source_inputs_unchanged']
assert audit['native_motor_accuracy_verified'] is audit['old_calibration_reclassified'] is audit['normalization_intervention'] is False
assert audit['physical_SI_status']=='DIAGNOSTICS_ONLY_NO_ACCURACY_VERDICT'
assert len(audit['discriminator'])==2 and {d['target'] for d in audit['discriminator']}=={'negative','positive'}
for d in audit['discriminator']:
    assert d['status']=='INCONCLUSIVE_NO_CAPTURED_FIRST_SLICE_DIFFERENCE' and d['baseline_early1_first_integration_equal'] and d['baseline_early3_first_final_equal']
    assert not d['baseline_early3_first_q_different'] and not d['baseline_early1_first_final_different'] and not d['accuracy_verified']
assert len(audit['cases'])==18
constructors=advances=visits=companions=stages=0;word_binding_count=0;first_slice=[]
for raw,entry in zip(raws,audit['cases']):
    same(entry['case'],raw['case']);same(entry['framing'],raw['framing'])
    done=raw['records'][2]
    if raw['case']['observer']=='off':assert entry['analyses']==[];continue
    ce=done['construction_companion']['events'];se=done['stage_v1']['events'];companions+=len(ce);stages+=len(se)
    assert done['construction_companion']['native_scalar_bits']==done['stage_v1']['native_scalar_bits']==32
    assert len(entry['analyses'])==32
    for n,a in enumerate(entry['analyses']):
        assert a['slice']==n
        for name in ('construction','integration','initialized_diagonal'):assert a[name]['status']=='DECLARED_FORMULAS_MATCH'
        constructors+=1;advances+=1
        for v in a['native_visits']:assert v['check']['status']=='DECLARED_FORMULAS_MATCH';visits+=1
        events=[e for e in ce if e['internal_slice_index']==n]
        before=next(e for e in events if e['phase']=='BeforePositionIntegration')['endpoints'][0]
        after=next(e for e in events if e['phase']=='AfterPositionIntegration')['endpoints'][0]
        final=next(e for e in events if e['phase']=='FinalSlice')
        built=next(e for e in events if e['phase']=='AfterRowConstruction')['constructed_rows'][0]
        for k,val in [('q_before_rad_word',before['coordinate_word']),('q_after_rad_word',after['coordinate_word']),('integration_input_velocity_rad_s_word',before['multibody_velocity_words'][0]),('final_slice_generic_velocity_rad_s_word',final['generalized_velocity_words'][0])]:
            word(a[k]);assert a[k]==val;word_binding_count+=1
        same(a['first_row'],built['row']);same(a['first_constructor_inputs'],{k:v for k,v in built.items() if k!='row'})
        expected_stab=[e['generalized_velocity_words'][0] for e in events if e['phase']=='AfterJointStabilization']
        same(a['stabilization_generic_velocity_rad_s_words'],expected_stab)
        if n==0:first_slice.append(dict(case=raw['case'],q_before_rad_word=a['q_before_rad_word'],q_after_rad_word=a['q_after_rad_word'],integration_input_velocity_rad_s_word=a['integration_input_velocity_rad_s_word'],final_slice_generic_velocity_rad_s_word=a['final_slice_generic_velocity_rad_s_word']))
assert (constructors,advances,visits,companions,stages)==(384,384,896,2572,8192)
audit_claim=read(root/'reader-audit-claim.json')
assert audit_claim['output']==str(audit_path) and audit_claim['result_root']==str(root) and audit_claim['spec_path']==str(wrapper_path)
assert audit_claim['externally_supplied_wrapper_sha256']==wrapper_sha and audit_claim['native_processes_launched']==0
assert audit_claim['checked_outer_sha256']==descriptor(STUDY/'collect-invocation/actual-command.json')['sha256']
assert audit_claim['reader_source']==descriptor(REC/'next-motor-visit-placement-reader/cohort_reader.py')
assert when(audit_claim['created_utc'])>=when(audit_command['created_utc'])
assert audit_command['command']==['/usr/bin/python3','-B',audit_claim['reader_source']['path'],'--spec',str(wrapper_path),'--spec-sha256',wrapper_sha,'--result-root',str(root),'--output',str(audit_path),'--checked-outer-receipt',str(STUDY/'collect-invocation/actual-command.json'),'--checked-outer-sha256',audit_claim['checked_outer_sha256']]
for d in s['frozen_inputs']:bound(d)
bound(s['binary']);bound(wrapper['candidate']);bound(wrapper['admission'])
git_records=[git_read(['status','--porcelain=v1']),git_read(['rev-parse','HEAD']),git_read(['rev-parse','HEAD^{tree}'])]
assert git_records[0]['stdout']=='' and git_records[1]['stdout'].strip()=='5e8e3627cd91df767ae41e8364f9401a8cd4e2d7' and git_records[2]['stdout'].strip()=='1e1a12da95768a61baa4109cbb7fb815adb79ae9'
ledger_paths={d['path'] for d in s['frozen_inputs']}
ledger_paths.update(str(p) for p in STUDY.rglob('*') if p.is_file())
ledger_paths.update([wrapper['admission']['path'],s['binary']['path'],s['driver_interpreter']['path'],str(Path(__file__).resolve())])
ledger=dict(kind='independent_new18_postcollection_whole_file_ledger',created_utc=datetime.now(timezone.utc).isoformat(),whole_files=[descriptor(p) for p in sorted(ledger_paths)],scope='All508 frozen inputs and every whole study artifact present at review, admission, native binary/interpreter and this review source. No splitting/truncation/formula replay.')
ledger_path=OWN/'whole-file-ledger.json';write_exclusive(ledger_path,ledger)
report=dict(kind='independent_new18_retention_lineage_postcollection_review',schema_version=1,status='PASS_RETAINED_LINEAGE_ORIGINAL_EXITS_FIXED_BUDGET_AND_WORD_BINDINGS',created_utc=datetime.now(timezone.utc).isoformat(),candidate=descriptor(candidate_path),admission=wrapper['admission'],wrapper=descriptor(wrapper_path),audit=descriptor(audit_path),whole_ledger=descriptor(ledger_path),whole_ledger_files=len(ledger['whole_files']),all508_frozen_inputs_and_binary_unchanged=True,main_readonly_git_records=git_records,actual_original_native_exits=[r['actual_exit_code'] for r in terminal['cases']],original_coordinator_exit=checked['actual_exit_code'],original_checked_CLI_exit=collect['original_exit'],original_root_reader_CLI_exit=audit_command['original_exit'],native_processes=18,outer_intervals=18,scientific_reader_invocations=1,retries=0,resumed=False,constructor_status_bindings=constructors,coordinate_status_bindings=advances,motor_solve_status_bindings=visits,construction_event_counts=companions,stage_event_counts=stages,coordinate_velocity_word_bindings=word_binding_count,all18_whole_gzip_single_member_complete=True,captured_ON_fresh_OFF_checkpoint_equal=True,ON_fresh_completed_record_literal_equal=True,OFF_all_observers_null=True,first_slice_captured_word_bindings=first_slice,first_slice_discriminator=[dict(target=d['target'],status=d['status']) for d in audit['discriminator']],physical_SI_status=audit['physical_SI_status'],audit_arithmetic_or_SI_recomputed=False,native_or_scientific_reader_launched_by_review=0,old_numerical_data_read=False,main_source_or_frozen_input_edits=False,public_retention='Whole audit5447072B and complete cohort retained locally; publication/recovery from public storage unverified. No Release/Git publication authorized by this review.',limits=['Reported audit fidelity is bound to the once-only actual reader, not independently recomputed here.','First-slice INCONCLUSIVE does not assert identical all32 profiles or improved physical accuracy.','Full private caches/runtime hardware float mode/compiler-runtime closure and external/hardware validation remain unmeasured.'],issues=[])
report_path=OWN/'postcollection-review.json';write_exclusive(report_path,report)
print(json.dumps(dict(review=descriptor(report_path),ledger=descriptor(ledger_path),files=len(ledger['whole_files'])),sort_keys=True))
