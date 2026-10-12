#!/usr/bin/env python3
"""Capture static source/provenance evidence only; never import/execute reader logic."""
import ast
import datetime
import hashlib
import json
import pathlib
import re
import subprocess

G5=pathlib.Path('/workspace/.rne-kick-demo/single-joint-motor-calibration/next-experiment/generation-5')
R=G5/'reader'
OUT=pathlib.Path(__file__).parent
REPO=pathlib.Path('/workspace/RobotNativeEngine')
HEAD='2c3586fa9168f13c774fb8f0c0d2cd359eba1287'

def sha(b):return hashlib.sha256(b).hexdigest()
def rec(p):
    p=pathlib.Path(p);b=p.read_bytes();return {'path':str(p),'bytes':len(b),'sha256':sha(b)}
def check(d):
    assert rec(d['path'])==d,d['path'];return d
def write(name,obj):
    p=OUT/name;b=(json.dumps(obj,indent=2,sort_keys=True)+'\n').encode()
    with p.open('xb') as f:f.write(b)
    return {'path':str(p),'bytes':len(b),'sha256':sha(b)}
def git(*args):return subprocess.check_output(['git',*args],cwd=REPO).decode().strip()

def main():
    started=datetime.datetime.now(datetime.timezone.utc).isoformat()
    assert git('rev-parse','HEAD')==HEAD and git('status','--porcelain')==''
    ready=json.loads((R/'reader-review-ready.json').read_bytes())
    assert ready['status']=='READY_FOR_INDEPENDENT_PRE_DATA_SOURCE_REVIEW_NOT_YET_WORLD_FREEZE'
    assert ready['worlds_builds_old_native_data_reads_checkout_edits']==0
    inputs=[rec(R/'reader-review-ready.json')]
    for name in ['baseline_source_reference','binding_template','preregistration','protocol_template','pure_qualification','source_diff']:
        inputs.append(check(ready[name]))
    pure=json.loads((R/'actual-pure-test-receipt.json').read_bytes())
    assert pure['status']=='PASS' and pure['actual_exit_code']==0 and pure['unittest_methods']==15
    assert pure['native_worlds_builds_old_native_data_reads']==0 and pure['authoritative_generation5_collection_or_numeric_findings_qualified'] is False
    closure=[check(x) for x in pure['runtime_reader_closure']]
    assert [pathlib.Path(x['path']).name for x in closure]==['audit_coupled_stages.py','stage_analysis.py','stage_numeric.py','stage_schema.py','stage_roles.py','synthetic-profile.json']
    assert len(closure)==6 and sum(pathlib.Path(x['path']).suffix=='.py' for x in closure)==5
    inputs.extend(closure)
    tests=[check(x) for x in pure['pure_test_source_closure']];inputs.extend(tests)
    for name in ['actual_command','actual_stderr','actual_stdout','preserved_predata_first_failed_attempt','prior_pass_before_actual_schema2_integration']:
        inputs.append(check(pure[name]))
    command=json.loads(pathlib.Path(pure['actual_command']['path']).read_bytes())
    assert command['exit_code']==0 and command['argv']==['python3','-B','-m','unittest','-v','test_stage_numeric','test_stage_reader','test_stage_provenance']
    assert command['cwd']==str(R)
    assert all(sha((R/n).read_bytes())==h for n,h in command['source_sha256'].items())
    assert len(command['source_sha256'])==8
    log=pathlib.Path(pure['actual_stderr']['path']).read_text()
    labels=re.findall(r'^test_[^\n]+ \.\.\. ok$',log,re.M)
    assert len(labels)==15 and 'Ran 15 tests' in log and log.rstrip().endswith('OK')
    old1=json.loads(pathlib.Path(pure['preserved_predata_first_failed_attempt']['path']).read_bytes())
    old2=json.loads(pathlib.Path(pure['prior_pass_before_actual_schema2_integration']['path']).read_bytes())
    assert old1['exit_code']==1 and old2['exit_code']==0

    protocol=json.loads((R/'protocol-template.json').read_bytes())
    binding=json.loads((R/'source-binary-binding-template.json').read_bytes())
    assert protocol['status']==binding['status']=='UNFROZEN_TEMPLATE_DO_NOT_EXECUTE'
    assert protocol['registered_utc'] is None and protocol['generation']==5
    assert protocol['cohort_id']==binding['cohort_id']=='generation5-independent-all-motor-stage'
    assert protocol['independence']=='NEW_INDEPENDENT_COHORT_NOT_RESTORATION_OF_LOST_GENERATION4_TAR'
    assert protocol['authoritative_coupled_worlds']==18 and protocol['scalar_qualification_worlds']==0
    cells=[{'build':b,'mass_kg':m,'mode':mode,'run_label':f'{b}-m{m}-{mode}'} for b in ['baseline','rejected_inverse_w_plus_g'] for m in [5,20,50] for mode in ['on','fresh_on','off']]
    assert protocol['fixed_cells']==cells
    assert protocol['reader_artifacts']==binding['reader_artifacts']==closure
    assert binding['protocol_sha256']=='0'*64 and binding['binary']['sha256']=='0'*64
    assert binding['authoritative_worlds_of_this_cohort_before_freeze']==0
    assert not any(x['path'] in [str(R/'protocol-template.json'),str(R/'source-binary-binding-template.json')] for x in closure)
    # Templates are inputs to this review, not records of the separately filled freeze.
    for name in ['role_admission','preregistration','baseline_source_reference','root_driver']:
        inputs.append(check(protocol[name]))
    for x in protocol['dependency_source_artifacts']:inputs.append(check(x))
    profile=json.loads((R/'synthetic-profile.json').read_bytes())
    history_resource=json.loads((R/'historical-resource-policy-reference.json').read_bytes())
    assert protocol['resource_ceilings']==profile['resource_ceilings']==history_resource['unchanged_resource_ceilings']
    assert protocol['expected_runtime_parameters']==profile['expected_runtime_parameters']
    assert protocol['expected_runtime_parameters']['num_solver_iterations']==4
    assert protocol['expected_runtime_parameters']['num_internal_pgs_iterations']==1
    assert protocol['expected_runtime_parameters']['num_internal_stabilization_iterations']==2

    reference=json.loads((R/'baseline-source-reference-receipt.json').read_bytes())
    assert reference['baseline_commit']==HEAD and reference['checkout_status']==''
    assert len(reference['files'])==20
    reference_records=[]
    for x in reference['files']:
        live=rec(REPO/x['repository_path']);copy=rec(x['reference_path'])
        assert live['sha256']==copy['sha256']==x['baseline_sha256'] and live['bytes']==copy['bytes']==x['bytes']
        reference_records.append({'repository_path':x['repository_path'],'live':live,'source_reference':copy})
    fixture_start=json.loads((G5/'fixture-review/source-review-start.json').read_bytes())
    fixture_proof=json.loads((G5/'fixture-review/source-review-proof.json').read_bytes())
    assert fixture_proof['source_file_count']==33 and fixture_proof['head_before']==fixture_proof['head_after']==HEAD
    assert fixture_proof['source_byte_changes']==[]
    assert fixture_proof['source_inventory_sha256_before']==fixture_proof['source_inventory_sha256_after']==fixture_start['source_inventory_sha256']
    for x in fixture_start['sources']:
        live=rec(REPO/x['path']) if not pathlib.Path(x['path']).is_absolute() else rec(x['path'])
        assert live['sha256']==x['sha256'] and live['bytes']==x['bytes']
    for x in fixture_proof['artifact_bindings']:inputs.append(check(x))
    inputs.extend([rec(G5/'fixture-review/source-review-proof.json'),rec(R/'README.md'),rec(R/'historical-resource-policy-reference.json')])
    old_code=REPO/'docs/evidence/contact-motor-stages/history/postcollection-native-frame-schema-repair-v2'
    assert (R/'stage_numeric.py').read_bytes()==(old_code/'stage_numeric.py').read_bytes()
    assert (R/'stage_schema.py').read_bytes()==(old_code/'stage_schema.py').read_bytes()
    ast_refs=[];local_modules={'audit_coupled_stages','stage_analysis','stage_schema','stage_numeric','stage_roles'}
    for name in sorted(local_modules):
        p=R/(name+'.py');text=p.read_text();module=ast.parse(text)
        for node in ast.walk(module):
            if isinstance(node,ast.ImportFrom) and node.module and node.module.startswith('stage_'):
                assert node.module in local_modules
            if isinstance(node,(ast.FunctionDef,ast.ClassDef)):
                ast_refs.append({'source_file':str(p),'name':node.name,'line':node.lineno,'end_line':node.end_lineno,'source_AST_segment_sha256':sha(ast.get_source_segment(text,node).encode())})
    assert not pathlib.Path(json.loads((G5/'driver/spec-template.json').read_bytes())['cohort_root']).exists(),'actual cohort directory already exists; review cannot imply pre-data completion'
    assert git('rev-parse','HEAD')==HEAD and git('status','--porcelain')==''
    ledger=write('source-and-receipt-binding-ledger.json',{'inputs':inputs,'runtime_reader_artifacts_six':closure,'actual_test_source_hashes_eight':command['source_sha256'],'actual_test_method_labels_15':labels,'baseline_live_and_source_reference_20':reference_records,'reader_source_AST_references':ast_refs})
    findings=[
        {'finding':'All three internal roles are prospectively admitted','evidence':['stage_roles.py:20-113','fixture-review/source-derived-fixture-review.md'],'details':'Unique authored 1kg palm and two .2kg prismatic children join checkpoint handles to link1/2/3, native LinX, ForceBased/AccelerationBased and 3-coordinate full J/W/V blocks; slots are append order, not anatomical labels.'},
        {'finding':'Actual builder and phase order is retained','evidence':['stage_roles.py:90-113','stage_analysis.py:304-511'],'details':'Observed motor ordering derives Motor+Limit reservations and global visit/J offsets; every initialized aggregate has all3 roles, each immediate pass matches aggregate order; uncaptured limit writes are not attributed to palm.'},
        {'finding':'Zero authority remains explicit and literal','evidence':['stage_analysis.py:226-265','stage_numeric.py:244-272','stage_roles.py:65-85'],'details':'Finger caps/gains/coefficient zero, [-0,+0] bounds and signed-zero lambdas retained; RHS is not forced zero, native AXPY still runs, and zero authority is not successful control or convergence failure.'},
        {'finding':'Arithmetic source and replay exclusions are preserved','evidence':['stage_numeric.py:129-272','operation-source-reference/nalgebra-0.33.3/src/lib.rs:240-249','source-reference/third_party/rapier3d/src/dynamics/solver/joint_constraint/joint_generic_constraint.rs:511-540'],'details':'Exact Fraction common projection and native Dyn dot8/tail, ordered separate f32 operations, nalgebra clamp branch, negative delta AXPY and inverse guard match source; tiny/subnormal/nonfinite/overflow exclusions cannot satisfy complete replay.'},
        {'finding':'Cached-stage scope prevents unsupported reconstruction','evidence':['stage_analysis.py:391-511','reader/preregistration.md'],'details':'Before/after external force cache absent, each joint rebuild new lambda+0 and same rhs/rhs_without_bias, post-integration old J/W retained; absolute RHS origin, per-contact columns/causal attribution and hidden cache replay stay unmeasured.'},
        {'finding':'Fixed18 are a new cohort, analyzed ON once','evidence':['audit_coupled_stages.py:67-130','audit_coupled_stages.py:222-282','audit_coupled_stages.py:408-482'],'details':'Strict build2×mass3×mode3 fixed order, 1250frames each, all18 complete EOF/CRC/exit/source guards; ON6×1250 analysis, freshON literal replay and OFF null; no scalar worlds or candidate promotion inferred.'},
        {'finding':'Actual durable schema2 integration is strict','evidence':['audit_coupled_stages.py:284-400','driver/collect_once.py','reader/test_stage_provenance.py'],'details':'Exact cohort/spec/case/launch/Popen start and original returncode linkage, timezone/order, 18 reserved/attempted/completed, serial source guards and no retry; incomplete or failed outputs remain FAIL.'},
        {'finding':'Freeze graph is acyclic at this source-review stage','evidence':['reader/protocol-template.json','reader/source-binary-binding-template.json','reader/README.md'],'details':'Six runtime artifacts exclude protocol and binding; protocol binds source/preregistration/driver, bindings point to protocol digest, final driver spec can point to this review. Unfrozen placeholders are not scientific qualification.'},
    ]
    proof={'schema_version':1,'status':'PASS','scope':'INDEPENDENT_PRE_DATA_SOURCE_ONLY_GENERATION5_READER_REVIEW_NOT_NATIVE_OR_COMPILER_QUALIFICATION','started_utc':started,'ended_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'head_before_after':HEAD,'git_clean_before_after':True,'cohort_id':'generation5-independent-all-motor-stage','reader_review_ready':rec(R/'reader-review-ready.json'),'runtime_reader_artifact_count':6,'runtime_Python_module_count':5,'profile_JSON_count':1,'runtime_closure':closure,'actual_pure_qualification':{'receipt':rec(R/'actual-pure-test-receipt.json'),'actual_command':pure['actual_command'],'exit_code':0,'unittest_methods':15,'actual_stderr':pure['actual_stderr'],'command_snapshot_Python_sources_eight_match_current':True,'fresh_runtime_artifacts_six_match_ready_receipt':True,'worlds_and_old_native_reads':0,'earlier_attempt1_failed_exit1_retained':True,'earlier_attempt2_prior_pass_exit0_retained':True,'qualification_is_synthetic_only':True},'unchanged_historical_code_only':{'stage_numeric_sha256':rec(R/'stage_numeric.py')['sha256'],'stage_schema_sha256':rec(R/'stage_schema.py')['sha256'],'native_observations_read_from_history':0},'source_reference_live_count':20,'fixture_inventory_live_count':33,'source_only_findings':findings,'material_blockers':[],'required_final_driver_spec_fill':['cohort_id generation5-independent-all-motor-stage','timezone registered_utc before first authoritative world','all6 exact current reader artifacts:5Python+profileJSON','actual frozen protocol/bindings/source guard/compiler/features/binary receipts','preflight PASS descriptors bound before exclusive new collection creation'],'concrete_final_artifacts_validated_by_this_review':False,'world_execution_authorized_by_this_review':False,'new_cohort_directory_absent_at_review_completion':True,'scientific_outcomes':{'generation4':'FAILED_UNQUALIFIED unchanged','generation3_candidate':'REJECTED unchanged','generation5_actual_measurement_qualified':False,'accuracy_hardware_maturity_gain_claimed':False},'explicit_exclusions':['independent opaque native per-object layout decode','full hidden-set/private pipeline/solver/contact cache replay','absolute runtime MXCSR/rounding verification','row-build coordinates and absolute RHS origin','individual contact impulses/columns and per-contact causal attribution','anatomical left/right finger frames','compiler/dependency complete closure'],'binding_ledger':ledger,'helper':rec(pathlib.Path(__file__)),'activity_counts':{'native_values_or_old_numerical_payload_reads':0,'worlds':0,'Cargo_or_native_builds':0,'new_test_executions':0,'checkout_or_Git_mutations':0,'CI_monitoring':0,'network_operations':0,'scientific_reader_execution':0}}
    result=write('independent-predata-reader-source-review.json',proof)
    print(json.dumps({'status':'PASS','descriptor':result,'ledger':ledger,'scope':proof['scope'],'material_blockers':[],'actual_pure_methods':15,'runtime_artifacts':6,'concrete_freeze_or_native_world_qualification':False}))

if __name__=='__main__':main()
