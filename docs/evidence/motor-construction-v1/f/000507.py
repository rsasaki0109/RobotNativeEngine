"""Synthetic schema/cohort and injected driver mocks. No native executable calls."""
import copy
from datetime import datetime, timedelta, timezone
import gzip
import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest
from construction_oracle import _expected_words
from stage_numeric import F32, ONE
from cohort_driver import drive
from cohort_reader import CASES, COMPANION_SCHEDULE, STAGE_SCHEDULE, DT, H, K, D, CAP, STATE_SCOPE, checkpoint, conditions, load_spec, qualify_cohort, validate_run
from strict_io import canonical, descriptor, gzip_records, loads

HERE = Path(__file__).parent


def gz(data):
    value = gzip.compress(data, compresslevel=6, mtime=0)
    return value[:9] + b'\xff' + value[10:]


def mock_row(target):
    inputs = dict(coordinate_word=0, target_position_word=target, target_velocity_word=0,
        stiffness_word=K, damping_word=D, duration_word=H)
    expected = _expected_words(inputs, CAP, None, 'ForceBased', F32())
    row = dict(joint_handle={'index': 1, 'generation': 0}, link_id=1, axis='AngX',
        gain_model='ForceBased', constraint_index=0, solver_velocity_offset=0, ndofs2=1,
        j_id2=0, jacobian_words=[ONE], weighted_jacobian_words=[ONE],
        rhs_word=expected['rhs_word'], rhs_without_bias_word=expected['rhs_without_bias_word'],
        cfm_gain_word=expected['cfm_gain_word'], cfm_coefficient_word=0,
        inverse_lhs_word=ONE, impulse_word=0,
        impulse_bound_words=[expected['lower_impulse_bound_word'], expected['upper_impulse_bound_word']],
        effort_ceiling_word=CAP)
    build = dict(inputs, row=row, limits_words=None,
        inverse_duration_word=expected['inverse_duration_word'],
        erp_inverse_duration_word=expected['erp_inverse_duration_word'],
        effective_target_velocity_word=0)
    return row, build


def mock_checkpoint():
    f = F32(); min_dt = f.divide(f.divide(ONE, 0x42700000), 0x42c80000)
    c = dict(schema_version=1, native_scalar_bits=32, state_scope=STATE_SCOPE,
        gravity_words=[0]*3, integration_parameters=dict(native_f32_words=[DT,min_dt,
        0x40a00000,0x41f00000,0x49742400,ONE,ONE,ONE,DT,0x41200000,0x3b03126f],
        native_counts=[32,0,1,2,128,1], postcard_bytes=[1]),
        records=[dict(kind=k,handle_words=h,postcard_bytes=[1]) for k,h in
            [('Body',[0,0]),('Body',[1,0]),('Collider',[0,0]),('Multibody',[0,0])]],
        public_body_words=[dict(handle_words=[i,0],position_words=[0]*6+[ONE],
            linear_velocity_words=[0]*3, angular_velocity_words=[0]*3,
            user_force_words=[0]*3,user_torque_words=[0]*3,mass_query_kg_word=ONE,
            effective_angular_inertia_query_kg_m2_words=[ONE]*9) for i in range(2)],
        public_multibody_words=[dict(handle_words=[0,0],ndofs=1,links=2,
            generalized_velocity_words=[0],damping_words=[0],
            joints=[dict(link_index=i,body_handle_words=[i,0],coordinates_words=[0]*6,
                joint_rotation_words=[0,0,0,ONE],descriptor_motor_impulse_words=[0]*6)
                for i in range(2)])],parent_handle_words=[0,0],child_handle_words=[1,0],
        joint_handle_words=[1,0],child_link_index=1)
    return c


def mock_impulse():
    value = dict(kind='Revolute',applied_impulse_word=0,effort_ceiling_word=0x4000000000000000,
                 rounded_bound_word=0x3f10624de0000000)
    slices=[dict(ccd_partition_index=0,internal_slice_index=i,expected_internal_slice_count=32,
        duration_s_word=0x3f00624de0000000,impulse=copy.deepcopy(value),
        finite_positive_bound_eligible=True,finite_positive_bound_active=False) for i in range(32)]
    return dict(scalar_bits=64,numerical_outer_interval_s_word=0x3f50624de0000000,
        ccd_partition_duration_s_words=[0x3f50624de0000000],motors=[dict(
            joint_handle_words=[1,0],link_id=1,joint_dof_id=0,axis='AngX',impulse=value,
            observed_solver_interval_s_word=0x3f50624de0000000,observed_solver_slice_count=32,
            full_step_coverage=True,observed_partition_count=1,
            finite_positive_bound_eligible_slice_count=32,finite_positive_bound_eligible_duration_s_word=0x3f50624de0000000,
            finite_positive_bound_active_slice_count=0,finite_positive_bound_active_duration_s_word=0,
            slices=slices)])


def mock_records(case):
    target=DT | (0x80000000 if case['target']=='negative' else 0)
    row, build = mock_row(target)
    declared=dict(sphere_radius_m_word=0x3dcccccd,sphere_mass_kg_word=ONE,
        outer_dt_s_word=DT,stiffness_nm_per_rad_word=K,damping_nm_s_per_rad_word=D,
        ceiling_nm_word=CAP,target_velocity_rad_s_word=0,gravity_words=[0]*3,
        initial_generalized_velocity_word=0,passive_damping_word=0,armature_word=0,
        small_steps=32,pgs_passes=1,stabilization_passes=2,contacts_enabled=False,
        sleep_enabled=False,ccd_enabled=False,identity_coincident_x_frames=True)
    start=dict(kind='run_start',schema_version=1,run_label=case['label'],target=case['target'],
        target_position_rad_word=target,observer_enabled=case['observer']=='on',
        expected_outer_intervals=1,native_scalar_bits=32,state_codec='postcard-1.1.3-raw-exact-consumption',
        state_scope=STATE_SCOPE,declared_conditions=declared)
    events=[];stages=[]
    for slice_ in range(32):
        for phase,iteration in COMPANION_SCHEDULE:
            uninit=phase in ('BeforeExternalForces','AfterExternalForces')
            events.append(dict(ccd_partition_index=0,island_index=0,internal_slice_index=slice_,
                expected_internal_slice_count=32,iteration_index=iteration,phase=phase,
                native_scalar_bits=32,duration_word=H,row_cache_slice_index=None if uninit else slice_,
                availability='Uninitialized' if uninit else 'Available',
                integration_multibody_handle_words=[0,0] if phase in ('BeforePositionIntegration','AfterPositionIntegration') else None,
                generalized_velocity_words=[0],constructed_rows=[copy.deepcopy(build)] if phase=='AfterRowConstruction' else [],
                endpoints=[] if uninit else [dict(row=copy.deepcopy(row),coordinate_word=0,multibody_velocity_words=[0])]))
        for phase,iteration,visit in STAGE_SCHEDULE:
            uninit=phase in ('BeforeExternalForces','AfterExternalForces')
            stages.append(dict(ccd_partition_index=0,island_index=0,internal_slice_index=slice_,
                expected_internal_slice_count=32,iteration_index=iteration,joint_visit_index=visit,
                stage=phase,native_scalar_bits=32,duration_word=H,
                row_cache_slice_index=None if uninit else slice_,generalized_velocity_words=[0],
                rows=[] if uninit else [copy.deepcopy(row)]))
    final=copy.deepcopy(events[-1]);final['phase']='AfterVelocityWriteback';events.append(final)
    comp=dict(schema_version=1,source_contract='InternalUnitMotorConstructionV1',native_scalar_bits=32,
        numerical_outer_interval_word=DT,ccd_partition_duration_words=[DT],events=events)
    stage=dict(native_scalar_bits=32,numerical_outer_interval_word=DT,ccd_partition_duration_words=[DT],events=stages)
    on=case['observer']=='on'
    done=dict(kind='completed_interval',schema_version=1,completed_outer_interval=1,
        checkpoint=mock_checkpoint(),construction_companion=comp if on else None,
        stage_v1=stage if on else None,impulse_v1_ieee_words=mock_impulse() if on else None)
    end=dict(kind='run_end',schema_version=1,status='collected',progress=dict(
        fixture_construction_started=True,fixture_constructed=True,pipeline_step_started=True,
        pipeline_step_returned=True,completed_outer_intervals=1,initial_records_written=1,
        completed_records_written=1,partial_records_written=0),records_written_before_terminal=3,error=None)
    return [start,dict(kind='initial_state',schema_version=1,checkpoint=mock_checkpoint()),done,end]


def raw(records):
    return b''.join(json.dumps(x,separators=(',',':')).encode()+b'\n' for x in records)


class CohortTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(dir=HERE);self.folder=Path(self.tmp.name)
    def tearDown(self): self.tmp.cleanup()
    def save(self, data):
        path=self.folder/'stream.gz';path.write_bytes(gz(data));return path
    def test_complete_synthetic_schema(self):
        result=validate_run(self.save(raw(mock_records(CASES[0]))),CASES[0])
        self.assertEqual(len(result['analyses']),32)
    def test_gzip_truncation_trailing_second_member(self):
        good=gz(raw(mock_records(CASES[0])))
        for data in (good[:-1],good+b'junk',good+gz(b'{}\n')):
            path=self.folder/'stream.gz';path.write_bytes(data)
            with self.assertRaises((ValueError,Exception)):gzip_records(path)
    def test_duplicate_key_float_and_boolean_word(self):
        for data in (b'{"x":1,"x":2}',b'{"x":1.0}',b'{"x":NaN}'):
            with self.assertRaises(ValueError):loads(data)
        records=mock_records(CASES[0]);records[0]['native_scalar_bits']=True
        with self.assertRaises(ValueError):validate_run(self.save(raw(records)),CASES[0])
    def test_fixed_gzip_header_mtime_flags_os_compression(self):
        good=gz(raw(mock_records(CASES[0])))
        for offset in (3,4,8,9):
            changed=bytearray(good);changed[offset]^=1
            path=self.folder/'stream.gz';path.write_bytes(changed)
            with self.assertRaises(ValueError):gzip_records(path)
    def test_missing_final_lf_and_partial_records(self):
        records=mock_records(CASES[0])
        for data in (raw(records)[:-1],raw(records[:-1])):
            with self.assertRaises(ValueError):validate_run(self.save(data),CASES[0])
    def test_extra_nested_schema_field_and_word_width(self):
        for key,val in [('extra',0),('native_scalar_bits',1<<32)]:
            records=mock_records(CASES[0]);records[2]['construction_companion']['events'][2][key]=val
            with self.assertRaises(ValueError):validate_run(self.save(raw(records)),CASES[0])
    def test_missing_or_extra_event_and_root_cache_tamper(self):
        for mutate in (lambda e:e.pop(),lambda e:e.append(copy.deepcopy(e[-1])),
            lambda e:e[3].update(integration_multibody_handle_words=[1,0]),
            lambda e:e[2].update(row_cache_slice_index=1)):
            records=mock_records(CASES[0]);mutate(records[2]['construction_companion']['events'])
            with self.assertRaises(ValueError):validate_run(self.save(raw(records)),CASES[0])
    def test_stage_snapshot_mismatch(self):
        records=mock_records(CASES[0]);records[2]['stage_v1']['events'][2]['rows'][0]['rhs_word']^=1
        with self.assertRaises(ValueError):validate_run(self.save(raw(records)),CASES[0])
    def test_zero_contact_generic_delta_rejected(self):
        records=mock_records(CASES[0]);records[2]['stage_v1']['events'][9]['generalized_velocity_words']=[ONE]
        with self.assertRaises(ValueError):validate_run(self.save(raw(records)),CASES[0])
    def test_constructor_mismatch_is_retained(self):
        records=mock_records(CASES[0]);records[2]['construction_companion']['events'][2]['constructed_rows'][0]['erp_inverse_duration_word']^=1
        result=validate_run(self.save(raw(records)),CASES[0])
        self.assertEqual(result['analyses'][0]['construction']['status'],'MISMATCH')
    def test_off_telemetry_null(self):
        records=mock_records(CASES[2]);validate_run(self.save(raw(records)),CASES[2])
        records[2]['stage_v1']={}
        with self.assertRaises(ValueError):validate_run(self.save(raw(records)),CASES[2])
    def test_failed_native_terminal_rejected(self):
        records=mock_records(CASES[0]);records[3]['status']='failed';records[3]['error']='synthetic failure'
        with self.assertRaises(ValueError):validate_run(self.save(raw(records)),CASES[0])

    def spec(self):
        binary=self.folder/'mock-binary-not-executable';binary.write_bytes(b'SYNTHETIC NEVER EXECUTED')
        artifact=self.folder/'artifact.json';artifact.write_text('{}\n')
        spec=dict(kind='frozen_isolated_motor_construction_cohort',schema_version=1,
            registered_utc=(datetime.now(timezone.utc)-timedelta(seconds=1)).isoformat(),
            collection_worlds_before_registration=0,cases=CASES,binary=descriptor(binary),cwd=str(self.folder),
            output_root=str(self.folder/'outputs'),frozen_inputs=[descriptor(HERE/name) for name in
                ('strict_io.py','cohort_reader.py','cohort_driver.py','construction_oracle.py','integration_oracle.py','stage_numeric.py')],
            registration_artifacts={role:descriptor(artifact) for role in
                ('protocol','collector_source_manifest','compiler_feature_receipt','producer_review','pure_tests_review')},
            source_scope='SYNTHETIC mocks only, not authentic engine data',
            process_failure_policy='stop_on_first_failure_no_resume_or_replacement',
            environment={name:os.environ.get(name) for name in
                ('RUSTFLAGS','CARGO_ENCODED_RUSTFLAGS','RUSTDOCFLAGS','RUST_TEST_THREADS')},timeout_seconds=1)
        candidate=self.folder/'candidate.json';candidate.write_text(json.dumps(spec))
        admission=self.folder/'admission.json';admission.write_text(json.dumps(dict(
            status='PASS_PRE_DATA_SOURCE_PROTOCOL_AND_DRIVER',reviewed_candidate_sha256=descriptor(candidate)['sha256'],
            binary=spec['binary'],frozen_inputs_sha256=hashlib.sha256(canonical(spec['frozen_inputs'])).hexdigest())))
        wrapper=self.folder/'wrapper.json';wrapper.write_text(json.dumps(dict(kind='admitted_isolated_motor_construction_cohort',
            schema_version=1,candidate=descriptor(candidate),admission=descriptor(admission))))
        return load_spec(wrapper,descriptor(wrapper)['sha256']),wrapper

    def mock_spawn(self, spec, failures=False):
        class Mock:
            def __init__(mock,command,**kwargs):
                index=CASES.index(next(c for c in CASES if c['label']==command[6]))
                self.assertTrue(all((Path(spec['output_root'])/c['label']/'launch-claim.json').exists() for c in CASES))
                mock.pid=1000+index;mock.code=9 if failures and index==0 else 0
                Path(command[8]).write_bytes(gz(raw(mock_records(CASES[index]))))
            def wait(mock,timeout=None):return mock.code
            def poll(mock):return mock.code
            def kill(mock):raise AssertionError('unexpected mock kill')
        return Mock

    def test_fixed_six_mock_claims_equality_and_no_resume(self):
        spec,_=self.spec();terminal=drive(spec,launch_native=True,spawn=self.mock_spawn(spec))
        self.assertEqual(terminal['actual_processes_started'],6)
        result=qualify_cohort(spec,spec['output_root']);self.assertEqual(result['status'],'PASS_COMPLETE_DECLARED_SCOPE')
        with self.assertRaises(FileExistsError):drive(spec,launch_native=True,spawn=self.mock_spawn(spec))
    def test_driver_validate_only_never_spawns(self):
        spec,_=self.spec()
        def no_spawn(*a,**k):raise AssertionError('process spawned')
        result=drive(spec,spawn=no_spawn)
        self.assertEqual(result['actual_processes_started'],0);self.assertFalse(Path(spec['output_root']).exists())
    def test_original_mock_failure_retained_without_retry(self):
        spec,_=self.spec();terminal=drive(spec,launch_native=True,spawn=self.mock_spawn(spec,True))
        self.assertEqual(terminal['actual_processes_started'],1)
        self.assertEqual(terminal['cases'][0]['actual_exit_code'],9)
        self.assertEqual(terminal['unlaunched_case_labels'],[c['label'] for c in CASES[1:]])
        self.assertEqual(terminal['status'],'FAILED_OR_PARTIAL_RETAINED')
        with self.assertRaises(ValueError):qualify_cohort(spec,spec['output_root'])
    def test_changed_source_mid_mock_run_stops_remaining_claims(self):
        spec,_=self.spec();original=self.mock_spawn(spec)
        def mutate(*args,**kwargs):
            value=original(*args,**kwargs);Path(spec['binary']['path']).write_bytes(b'CHANGED MOCK SOURCE');return value
        terminal=drive(spec,launch_native=True,spawn=mutate)
        self.assertEqual(terminal['actual_processes_started'],1)
        self.assertFalse(terminal['source_inputs_unchanged'])
        self.assertEqual(len(terminal['unlaunched_case_labels']),5)
    def test_raw_process_artifact_sha_tamper_rejected(self):
        spec,_=self.spec();drive(spec,launch_native=True,spawn=self.mock_spawn(spec))
        p=Path(spec['output_root'])/CASES[0]['label']/'stdout.log';p.write_bytes(b'tamper')
        with self.assertRaises(ValueError):qualify_cohort(spec,spec['output_root'])
    def test_mandatory_binding_and_changed_source(self):
        spec,wrapper=self.spec()
        with self.assertRaises(ValueError):load_spec(wrapper,'0'*64)
        Path(spec['binary']['path']).write_bytes(b'TAMPER')
        with self.assertRaises(ValueError):drive(spec)


if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(CohortTests))
    Path('actual-pure-tests.json').write_text(json.dumps(dict(kind='pure_synthetic_cohort_reader_and_injected_driver_tests',
        methods=result.testsRun,failures=len(result.failures),errors=len(result.errors),actual_exit_code=0 if result.wasSuccessful() else 1,
        native_processes_started=0,native_worlds=0,historical_native_data_read=False,driver_processes_are_injected_mocks=True),indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
