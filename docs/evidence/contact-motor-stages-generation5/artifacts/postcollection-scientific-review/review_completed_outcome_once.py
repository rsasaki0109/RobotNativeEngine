#!/usr/bin/env python3
"""Review outcome summaries/provenance and opaque whole-file hashes, not raw rows."""
from collections import Counter
import datetime
import hashlib
import json
import pathlib
import re
import subprocess

G=pathlib.Path('/workspace/.rne-kick-demo/single-joint-motor-calibration/next-experiment/generation-5')
REPO=pathlib.Path('/workspace/RobotNativeEngine')
OUT=pathlib.Path(__file__).parent
COHORT='generation5-independent-all-motor-stage'
HEAD='2c3586fa9168f13c774fb8f0c0d2cd359eba1287'
STATUS='PASS_COMPLETE_REPLAY_AND_SOURCE_ORDERED_NORMAL_ZERO_STAGE_QUALIFICATION'
BOUND={}

def stamp(s):
    d=datetime.datetime.fromisoformat(s.replace('Z','+00:00'));assert d.tzinfo is not None;return d
def rec(p):
    p=pathlib.Path(p);h=hashlib.sha256();size=0
    with p.open('rb') as f:
        while b:=f.read(1048576):h.update(b);size+=len(b)
    return {'path':str(p),'bytes':size,'sha256':h.hexdigest()}
def bind(d):
    p=d['path'];actual=BOUND.get(p)
    if actual is None:actual=rec(p);BOUND[p]=actual
    assert actual==d,('artifact SHA/size mismatch',p);return d
def read(d):bind(d);return json.loads(pathlib.Path(d['path']).read_bytes())
def descriptor(p):d=rec(p);BOUND[d['path']]=d;return d
def write(name,o):
    p=OUT/name;b=(json.dumps(o,indent=2,sort_keys=True)+'\n').encode()
    with p.open('xb') as f:f.write(b)
    return {'path':str(p),'bytes':len(b),'sha256':hashlib.sha256(b).hexdigest()}
def git(*args):return subprocess.check_output(['git',*args],cwd=REPO).decode().strip()

def main():
    began=datetime.datetime.now(datetime.timezone.utc).isoformat()
    head_before=git('rev-parse','HEAD');status_before=git('status','--porcelain')
    assert head_before==HEAD
    report_d=descriptor(G/'analysis/independent-numerical-audit.json')
    assert report_d['sha256']=='121e92aaecb215d84220974c8997c929b07b80fb0e019afc0db93c39e99f0dc2' and report_d['bytes']==266673
    report=read(report_d);assert report['status']==STATUS and report['cohort_id']==COHORT
    assert report['complete_native_normal_zero_replay'] is True and report['complete_finite_valid_reference'] is True
    assert report['all_qualification_checks_no_issues'] is True and report['runtime_registers_measured'] is False
    assert report['completed_coupled_worlds']==report['registered_authoritative_coupled_worlds']==18
    assert report['total_completed_coupled_outer_steps']==22500 and report['numerically_analyzed_ON_outer_steps']==7500
    assert report['registered_scalar_worlds_separately_audited']==report['native_worlds_executed_by_reader']==0
    assert report['hidden_per_contact_coupling_or_causal_adoption_or_maturity_claim'] is False
    assert report['generation4_palm_only_gate_and_failed_outcome_unchanged'] is True
    collection=read(report['collection']);protocol=read(report['protocol']);spec=read(collection['frozen_spec'])
    assert collection['schema_version']==2 and collection['cohort_id']==protocol['cohort_id']==spec['cohort_id']==COHORT
    assert collection['generation']==protocol['generation']==spec['generation']==5
    assert collection['status']=='GENERATION5_18_WORLD_PROCESSES_COMPLETED' and collection['error'] is None
    assert all(type(collection[k]) is int and collection[k]==18 for k in ['worlds_launch_reserved','worlds_attempted','worlds_completed','declared_authoritative_coupled_worlds'])
    assert collection['declared_completed_outer_steps_per_world']==1250
    assert protocol['status']=='FROZEN_PRE_DATA_GENERATION5_ALL_MOTOR_STAGE_PROTOCOL'
    assert spec['status']=='FROZEN_BEFORE_GENERATION5_COLLECTION' and spec['authoritative_worlds_of_this_cohort_before_freeze']==0
    assert protocol['scalar_qualification_worlds']==0 and protocol['scalar_qualification']=={'new_worlds':0,'status':'NOT_PART_OF_NEW_COUPLED_COLLECTION'}
    cells=[{'build':b,'mass_kg':m,'mode':mode,'run_label':f'{b}-m{m}-{mode}'} for b in ['baseline','rejected_inverse_w_plus_g'] for m in [5,20,50] for mode in ['on','fresh_on','off']]
    assert protocol['fixed_cells']==spec['fixed_cells']==cells
    assert collection['protocol_sha256']==report['protocol']['sha256'] and spec['protocol']==report['protocol']
    assert spec['bindings']==collection['source_binary_bindings']
    assert spec['reader_artifacts']==protocol['reader_artifacts'] and len(spec['reader_artifacts'])==6
    for d in protocol['reader_artifacts']+spec['driver_artifacts']:bind(d)
    assert len(spec['driver_artifacts'])==3
    bindings={b:read(d) for b,d in spec['bindings'].items()}
    for build,b in bindings.items():
        assert b['build']==build and b['cohort_id']==COHORT and b['status']=='FROZEN_BEFORE_GENERATION5_COHORT_WORLDS'
        assert b['authoritative_worlds_of_this_cohort_before_freeze']==0 and b['error'] is None
        assert b['protocol_sha256']==report['protocol']['sha256'] and b['reader_artifacts']==protocol['reader_artifacts']
        assert b['schema_source_sha256']==protocol['schema_source_sha256']
        assert stamp(protocol['registered_utc'])<=stamp(b['registered_utc'])<=stamp(spec['registered_utc'])
        bind(b['binary']);bind(b['resolved_features']);manifest=read(b['source_manifest'])
        assert manifest['files']==b['repo_source_sha256'] and 'not compiler/dependency closure' in manifest['scope']
        assert b['rustflags']==[] and '--locked' in b['build_command'] and '--release' in b['build_command']
        assert b['native_float_mode']['scope']=='source_bound_runtime_register_unmeasured' and b['native_float_mode']['replay_domain']=='normal_or_zero_only'
        for d in b['native_float_mode']['evidence']:bind(d)
    a=bindings['baseline'];b=bindings['rejected_inverse_w_plus_g'];unit='third_party/rapier3d/src/dynamics/joint/multibody_joint/unit_multibody_joint.rs'
    assert set(a['repo_source_sha256'])==set(b['repo_source_sha256'])
    assert [k for k in a['repo_source_sha256'] if a['repo_source_sha256'][k]!=b['repo_source_sha256'][k]]==[unit]
    for name in ['compiler','cargo','target','rustflags','native_float_mode']:assert a[name]==b[name]
    assert a['resolved_features']['sha256']==b['resolved_features']['sha256']
    assert a['rejected_candidate_patch'] is None and b['rejected_candidate_patch']==protocol['candidate_runtime_patch']
    bind(protocol['candidate_runtime_patch']);bind(protocol['historical_complete_rejected_patch'])
    live=dict(protocol['schema_source_sha256']);live[unit]=a['repo_source_sha256'][unit]
    for p,h in live.items():assert rec(REPO/p)['sha256']==h,p

    pure=read(spec['reader_pure_test_receipt'])
    assert pure['status']=='PASS' and pure['actual_exit_code']==0 and pure['unittest_methods']==15
    for d in pure['runtime_reader_closure']+pure['pure_test_source_closure']:bind(d)
    assert pure['runtime_reader_closure']==protocol['reader_artifacts']
    pure_command=read(pure['actual_command']);bind(pure['actual_stderr']);bind(pure['actual_stdout'])
    assert pure_command['exit_code']==0
    assert all(rec(G/'reader'/p)['sha256']==h for p,h in pure_command['source_sha256'].items())
    pure_log=pathlib.Path(pure['actual_stderr']['path']).read_text()
    assert len(re.findall(r'^test_[^\n]+ \.\.\. ok$',pure_log,re.M))==15 and 'Ran 15 tests' in pure_log and pure_log.rstrip().endswith('OK')
    independent=read(spec['independent_readonly_review'])
    for d in independent['reviews']:bind(d)
    assert independent['status']=='PASS_ALL_THREE_INDEPENDENT_PRE_DATA_SCOPES'
    assert stamp(pure['recorded_utc'])<=stamp(protocol['registered_utc'])
    assert stamp(independent['registered_utc'])<=stamp(spec['registered_utc'])<=stamp(collection['started_utc'])
    final_review=read(descriptor(G/'reader/final-driver-spec-review.json'))
    assert final_review['status']=='PASS_FINAL_PRE_DATA_DRIVER_SPEC_ADMISSION'
    assert stamp(final_review['recorded_utc'])<=stamp(collection['started_utc'])
    assert final_review['spec']==collection['frozen_spec']
    preflight=read(descriptor(G/'freeze/actual-final-driver-spec-preflight.json'))
    assert preflight['actual_exit_code']==0 and preflight['worlds']==0 and preflight['spec']==collection['frozen_spec']
    assert stamp(preflight['registered_utc'])<=stamp(collection['started_utc'])
    guard=read(spec['source_guard_manifest']);assert guard['files']==a['repo_source_sha256'] and len(guard['files'])==1640

    command_evidence={}
    for phase in ['actual-collection-command','actual-scientific-audit-command']:
        claim_d=descriptor(G/phase/'actual-command-claim.json');terminal_d=descriptor(G/phase/'actual-command-terminal.json')
        claim=read(claim_d);t=read(terminal_d)
        assert t['actual_exit_code']==0 and t['guarded_inputs_unchanged'] is True and t['rerun_or_retry'] is False
        assert claim['argv']==t['argv'] and claim['started_utc']==t['started_utc'] and claim['inputs_before']==t['inputs_before']==t['inputs_after']
        for d in t['inputs_before']:bind(d)
        bind(t['stdout']);bind(t['stderr']);bind(t['wrapper'])
        assert pathlib.Path(t['stderr']['path']).read_bytes()==b''
        command_evidence[phase]={'claim':claim_d,'terminal':terminal_d,'start':t['started_utc'],'end':t['ended_utc'],'guarded_inputs':len(t['inputs_before']),'exit_code':0,'stdout':t['stdout'],'stderr':t['stderr']}
    audit_t=read(command_evidence['actual-scientific-audit-command']['terminal'])
    stdout=json.loads(pathlib.Path(audit_t['stdout']['path']).read_bytes())
    assert stdout=={'native_worlds':0,'output':report_d,'status':STATUS}
    assert stamp(collection['ended_utc'])<=stamp(audit_t['started_utc'])<=stamp(audit_t['ended_utc'])
    registered=read(descriptor(G/'analysis/registered-analysis-argv.json'))
    registered_argv=registered['argv'] if isinstance(registered,dict) else registered
    assert registered_argv==audit_t['argv']
    collection_terminal=read(descriptor(G/'collection/collection-terminal.json'))
    assert collection_terminal['driver_exit_code']==0 and collection_terminal['index']==report['collection']
    assert collection_terminal['frozen_spec']==collection['frozen_spec'] and collection_terminal['retry_or_resume_permitted'] is False
    assert all(collection_terminal[k]==18 for k in ['worlds_launch_reserved','worlds_attempted','worlds_completed'])
    assert len(collection['runs'])==len(collection['case_receipts'])==18
    cases=[];raw_files=[];previous=stamp(collection['started_utc'])
    for ordinal,(row,expected,case_d) in enumerate(zip(collection['runs'],cells,collection['case_receipts'])):
        assert all(row[k]==v for k,v in expected.items()) and row['attempted'] is True and row['exit_code']==0 and row['error'] is None
        case=read(case_d);assert case['case_index']==ordinal and case['row']==row and case['cohort_id']==COHORT
        assert case['original_exit_code']==0 and case['launch_reserved'] is True and case['process_started'] is True
        assert case['source_inputs_unchanged'] is True and case['launch_error'] is None and case['retry_or_resume_permitted'] is False
        assert case['binary']==bindings[row['build']]['binary']
        original=read(case['original_process_exit_receipt']);assert type(original['original_exit_code']) is int and original['original_exit_code']==0
        assert original['source']=='Actual subprocess.Popen.returncode; no status substitution' and original['case_index']==ordinal
        folder=pathlib.Path(case_d['path']).parent;start_d=descriptor(folder/'process-started.json');start=read(start_d)
        assert start['case_index']==ordinal and type(start['pid']) is int and start['pid']==original['pid']>0 and start['argv']==row['argv']
        launch=read(start['launch_claim']);assert launch['launch_reserved'] is True and launch['attempted'] is False and launch['original_exit_code'] is None
        assert launch['exit_code'] is None and launch['retry_or_resume_permitted'] is False and launch['frozen_spec']==collection['frozen_spec']
        assert launch['binary']==case['binary'] and launch['source_guard_before']==row['source_guard_before']
        for d in [row['trace'],row['stdout'],row['stderr']]:bind(d)
        raw_files.append(row['trace'])
        before=read(row['source_guard_before']);after=read(row['source_guard_after']);assert before==after==guard
        assert previous<=stamp(row['started_utc'])<=stamp(start['started_utc'])<=stamp(original['ended_utc'])<=stamp(row['ended_utc'])<=stamp(collection['ended_utc'])
        previous=stamp(row['ended_utc'])
        expected_argv=[case['binary']['path'],'--mass-kg',str(row['mass_kg']),'--observer','off' if row['mode']=='off' else 'on','--label',row['run_label'],'--output',row['trace']['path']]
        assert row['argv']==expected_argv
        cases.append({'ordinal':ordinal,'cell':expected,'case_receipt':case_d,'Popen_start':start_d,'Popen_original_exit':case['original_process_exit_receipt'],'pid':start['pid'],'original_exit_code':0,'source_guard_before_after':row['source_guard_before']['sha256'],'trace':row['trace']})

    totals=Counter();role_totals={r:Counter() for r in ['palm','finger_slot_0','finger_slot_1']};profiles=[];detail_files=[]
    assert len(report['results'])==6
    for i,r in enumerate(report['results']):
        rows=collection['runs'][3*i:3*i+3]
        assert r['build']==rows[0]['build'] and r['mass_kg']==rows[0]['mass_kg'] and r['runs']==[x['run_label'] for x in rows]
        assert r['compressed_inputs']==[x['trace'] for x in rows]
        assert all(r[k] is True for k in ['within_build_native_checkpoint_typed_and_postcard_bytes_exact','within_build_ecs_physics_ieee_words_exact','fresh_ON_ordered_trace_exact','OFF_trace_absent'])
        assert r['completed_steps_each']==1250 and r['physics_checkpoints']==3750 and r['detail_records']==505000
        assert all(s['records']==1252 for s in r['input_streams'])
        assert [s['run_label'] for s in r['input_streams']]==r['runs']
        bind(r['details']);detail_files.append(r['details'])
        for row in rows:
            assert pathlib.Path(row['stdout']['path']).read_bytes()==(r['physical_report']['physical_stdout_row']+'\ncontact motor stage collection: ok\n').encode()
        analysis=r['analysis'];counts=analysis['counts']
        assert analysis['qualification_issues']==0 and analysis['first_qualification_issue'] is None
        assert analysis['complete_native_replay'] is True and analysis['complete_finite_valid_reference'] is True
        assert counts['row_snapshots']==counts['native_projection:qualified']==counts['finite_valid_reference_snapshots']==360000
        assert counts['immediate_motor_visits']==counts['immediate_motor_visits:qualified_match']==45000
        assert counts['contact_row_intervals']==counts['contact_intervals:native_qualified']==counts['contact_intervals:finite_fixed_cached_row']==90000
        assert counts['initialized_motor_cache_generations']==counts['rebuilt_diagonal:qualified_match']==15000
        assert counts['zero_authority_cache_generations']==10000 and counts['completed_outer_steps']==1250
        assert all(v==0 for k,v in counts.items() if any(x in k for x in ['unavailable','unqualified','mismatch','native_exclusion:','field_or_cache_changed']))
        for role,v in analysis['roles'].items():
            assert v['complete_native_replay'] is True and v['complete_finite_valid_reference'] is True
            assert v['authority']==('controlled' if role=='palm' else 'zero_authority')
            assert v['counts']['row_snapshots']==120000 and v['counts']['immediate_motor_visits']==15000 and v['counts']['contact_row_intervals']==30000
            role_totals[role].update(v['counts'])
        totals.update(counts)
        profiles.append({'build':r['build'],'mass_kg':r['mass_kg'],'runs':r['runs'],'details':r['details'],'detail_records':r['detail_records'],'counts':counts,'qualification_issues':0,'complete_native_normal_zero_replay':True,'complete_reference':True,'role_scopes':{role:{'authority':v['authority'],'complete_native_replay':v['complete_native_replay'],'complete_reference':v['complete_finite_valid_reference']} for role,v in analysis['roles'].items()}})
    assert sum(r['detail_records'] for r in report['results'])==3030000
    assert totals['immediate_motor_visits']==270000 and totals['row_snapshots']==2160000 and totals['contact_row_intervals']==540000

    assert len(protocol['immutable_published_artifacts'])==312
    for d in protocol['immutable_published_artifacts']:bind(d)
    scalar=descriptor(REPO/'docs/media/position-motor-calibration-contact-stages.json')
    assert scalar['sha256']=='db0dee42a3c9067b38241112d5ee402341c0729466d7fc22f12f31504000db76'
    old_manifest=json.loads((REPO/'docs/evidence/contact-motor-stages/artifact-manifest.json').read_bytes())
    assert old_manifest['scalar_primary_accuracy']=='FAIL_0_OF_4' and old_manifest['coupled_qualification']=='FAILED_UNQUALIFIED'
    old_failures=[]
    for p in ['history/actual-independent-coupled-audit.json','history/postcollection-handle-schema-repair-v1/actual-repaired-independent-coupled-audit.json','history/postcollection-native-frame-schema-repair-v2/actual-v2-independent-coupled-audit.json']:
        path=REPO/'docs/evidence/contact-motor-stages'/p
        if path.exists():
            d=descriptor(path);o=read(d);assert o['status']=='FAIL_RETAINED_WITHOUT_CRITERIA_CHANGE';old_failures.append({'descriptor':d,'status':o['status']})
    assert len(old_failures)==3
    assert git('rev-parse','HEAD')==head_before and git('status','--porcelain')==status_before
    assert descriptor(G/'analysis/independent-numerical-audit.json')==report_d
    ledger=write('final-report-provenance-and-opaque-file-binding-ledger.json',{'all_fresh_verified_artifact_descriptors':list(BOUND.values()),'case18_source_original_exit_bindings':cases,'raw18_opaque_compressed_file_SHA_size_bindings':raw_files,'details6_opaque_compressed_file_SHA_size_bindings':detail_files,'profiles6_summary_counts':profiles,'aggregate_counts':dict(sorted(totals.items())),'aggregate_role_counts':{k:dict(sorted(v.items())) for k,v in role_totals.items()}})
    proof={'schema_version':1,'status':'PASS_INDEPENDENT_COMPLETED_GEN5_RESULT_PROVENANCE_AND_SCOPED_CLAIM_REVIEW','review_started_utc':began,'review_ended_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'head':HEAD,'worktree_before_after_identical':True,'report':report_d,'report_status':STATUS,'protocol':report['protocol'],'driver_spec':collection['frozen_spec'],'collection':report['collection'],'actual_commands':command_evidence,'reader_six_artifacts_and_pure15_current_exact':True,'root_scientific_audit_once_exit0_with15_guarded_inputs_unchanged':True,'root_collection_once_exit0_with16_guarded_inputs_unchanged':True,'chronology_protocol_bindings_independent_reviews_final_spec_preflight_before_first_world':True,'baseline_and_registered_rejected_binary_hashes_verified':{k:v['binary'] for k,v in bindings.items()},'source_guard_paths':1640,'live_selected_runtime_source_paths':len(live),'all18_new_original_Popen_case_launch_exit_source_receipts_verified':True,'counts':{'new_worlds':18,'completed_outer_steps_all_modes':22500,'independent_ON_profiles':6,'analyzed_ON_outer_steps':7500,'complete_detail_files':6,'reported_details_each':505000,'reported_details_total':3030000,'opaque_details_compressed_bytes':sum(d['bytes'] for d in detail_files),'opaque_raw_compressed_bytes':sum(d['bytes'] for d in raw_files),'native_motor_visits_literal_match':totals['immediate_motor_visits:qualified_match'],'row_snapshots_reference_and_native_qualified':totals['row_snapshots'],'contact_row_intervals_native_and_reference_qualified':totals['contact_row_intervals'],'rebuilt_motor_cache_source_diagonal_match':totals['rebuilt_diagonal:qualified_match'],'zero_authority_cache_generations':totals['zero_authority_cache_generations'],'qualification_issues':0,'native_projection_strict_reopenings':totals['native_contact_projection:reopened'],'exact_reference_strict_reopenings':totals['reference_contact_projection:reopened'],'exact_complementarity_violated_palm_snapshots':totals['complementarity:violated'],'zero_authority_snapshots':totals['complementarity:zero_authority']},'all_profiles_all_palm_and_two_zero_authority_fingers_complete_native_normal_zero_and_reference':True,'new_same_scope_replay_reported':{'typed_checkpoint_and_postcard_byte_vectors':True,'ECS_physics_words':True,'fresh_ON_ordered_telemetry':True,'OFF_telemetry_null':True},'old312_immutable_frozen_scientific_descriptors_verified':True,'scalar_report_unchanged':scalar,'scalar_primary_accuracy':'FAIL_0_OF_4_UNCHANGED_NOT_REQUALIFIED','old_generation4_failed_audits_preserved':old_failures,'generation3_candidate':'REJECTED_UNCHANGED_NOT_PROMOTED','qualification_meaning':'Declared captured-row arithmetic/replay fidelity within source-ordered f32 normal/zero model; valid exact reference computation does not imply complementarity convergence, physical motor accuracy or calibration improvement.','exclusions':['full private set/pipeline/solver/contact cache and hidden-state replay','independent native opaque per-object layout decoding','absolute row-build coordinates and RHS-origin reconstruction','runtime MXCSR/rounding register absolute verification','per-contact columns/impulses or causal attribution','anatomical finger frame labeling','hardware/external-use validation','candidate promotion or performance/maturity gain'],'publication_scope':{'new_derived_details_archive':'PROPOSED_NOT_PUBLIC_AVAILABILITY_VERIFIED','old_exact_TAR_recovery_and_upload':'Separate root/recovery review; no upload/download/availability claim by this review.'},'ledger':ledger,'helper':descriptor(pathlib.Path(__file__)),'limitations':['No reader, physics world, Cargo command or numerical raw-row recomputation was rerun.','Six details and eighteen new raw files were checked as opaque whole compressed bytes/SHA/size only; their decoded record/framing/CRC/numerical completeness is the bound once-only reader result.','Source maps remain explicitly non-exhaustive snapshots, not compiler/dependency closure.','Counters and scope flags were independently checked from the actual final summary; no per-contact causality was inferred.'],'activity_counts':{'scientific_reader_reruns':0,'numerical_raw_reanalysis':0,'old_numerical_payload_parses':0,'worlds_or_builds_started':0,'checkout_or_Git_edits':0,'GitHub_or_network_mutations':0,'network_queries':0,'CI_monitors':0}}
    result=write('independent-completed-gen5-scientific-scope-review.json',proof)
    print(json.dumps({'status':proof['status'],'proof':result,'ledger':ledger,'report_sha256':report_d['sha256'],'counts':proof['counts'],'public_availability_qualified':False}))

if __name__=='__main__':main()
