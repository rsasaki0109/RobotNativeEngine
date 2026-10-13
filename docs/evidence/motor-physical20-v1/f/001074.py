"""Invented framing/stage fixtures; no native object/world or old data."""
import copy
import gzip
import json
from run_schema import *
from construction_oracle import _expected_words
from stage_numeric import F32, rounded
from physical_oracle import reference
from case_context import CaseContext
from fixed_contract import PROCESS_ORDER


def gz(data):
    value = gzip.compress(data, compresslevel=6, mtime=0)
    return value[:9] + b'\xff' + value[10:]


def raw(records):
    return b''.join(json.dumps(x,separators=(',',':')).encode()+b'\n' for x in records)

def mock_checkpoint(ctx):
    pgs, stabilization = 1, 2
    f = F32(); min_dt = f.divide(f.divide(ONE, 0x42700000), 0x42c80000)
    c = dict(schema_version=1, native_scalar_bits=32, state_scope=STATE_SCOPE,
        gravity_words=[0]*3, integration_parameters=dict(native_f32_words=[ctx.h,min_dt,
        0x40a00000,0x41f00000,0x49742400,ONE,ONE,ONE,0x3a83126f,0x41200000,0x3b03126f],
        native_counts=[1,0,pgs,stabilization,128,1], postcard_bytes=[1]),
        records=[dict(kind=k,handle_words=h,postcard_bytes=[1]) for k,h in
            [('Body',[0,0]),('Body',[1,0]),('Collider',[0,0]),('Multibody',[0,0])]],
        public_body_words=[dict(handle_words=[i,0],position_words=[0]*6+[ONE],
            linear_velocity_words=[0]*3, angular_velocity_words=[0]*3,
            user_force_words=[0]*3,user_torque_words=[0]*3,mass_query_kg_word=ctx.mass if i else 0,
            effective_angular_inertia_query_kg_m2_words=[rounded(reference(ctx.case)['inertia']) if i and j in (0,4,8) else 0 for j in range(9)]) for i in range(2)],
        public_multibody_words=[dict(handle_words=[0,0],ndofs=1,links=2,
            generalized_velocity_words=[0],damping_words=[0],
            joints=[dict(link_index=i,body_handle_words=[i,0],coordinates_words=[0]*6,
                joint_rotation_words=[0,0,0,ONE],descriptor_motor_impulse_words=[0]*6)
                for i in range(2)])],parent_handle_words=[0,0],child_handle_words=[1,0],
        joint_handle_words=[1,0],child_link_index=1)
    return c


def mock_impulse(ctx, final_row):
    h64 = promoted_word(ctx.h)
    bound = final_row['impulse_bound_words'][1]
    active = exact(final_row['impulse_word']) in (exact(bound), -exact(bound))
    value = dict(kind='Revolute', applied_impulse_word=promoted_word(final_row['impulse_word']) ^ (1<<63),
                 effort_ceiling_word=promoted_word(ctx.cap), rounded_bound_word=promoted_word(bound))
    slices=[dict(ccd_partition_index=0,internal_slice_index=0,expected_internal_slice_count=1,
        duration_s_word=h64,impulse=copy.deepcopy(value),
        finite_positive_bound_eligible=True,finite_positive_bound_active=active)]
    return dict(scalar_bits=64,numerical_outer_interval_s_word=h64,
        ccd_partition_duration_s_words=[h64],motors=[dict(
            joint_handle_words=[1,0],link_id=1,joint_dof_id=0,axis='AngX',impulse=value,
            observed_solver_interval_s_word=h64,observed_solver_slice_count=1,
            full_step_coverage=True,observed_partition_count=1,
            finite_positive_bound_eligible_slice_count=1,finite_positive_bound_eligible_duration_s_word=h64,
            finite_positive_bound_active_slice_count=int(active),finite_positive_bound_active_duration_s_word=h64 if active else 0,
            slices=slices)])


def mock_records(process):
    """Build invented full records with mathematical words, not native bytes."""
    ctx = CaseContext(process['case_id'], process['variant'])
    pgs, stabilization = 1, 2
    target = ctx.target
    declared = dict(sphere_radius_m_word=ctx.radius, sphere_mass_kg_word=ctx.mass,
        outer_dt_s_word=ctx.h, stiffness_nm_per_rad_word=ctx.k, damping_nm_s_per_rad_word=ctx.d,
        ceiling_nm_word=ctx.cap, target_velocity_rad_s_word=0, gravity_words=[0, 0, 0],
        initial_generalized_velocity_word=0, passive_damping_word=0, armature_word=0,
        small_steps=1, pgs_passes=pgs, stabilization_passes=stabilization,
        contacts_enabled=False, sleep_enabled=False, ccd_enabled=False,
        identity_coincident_x_frames=True, model='ForceBased', initial_coordinate_rad_word=0,
        body_linear_damping_word=0, body_angular_damping_word=0, limits_enabled=False,
        additional_solver_iterations=0, max_ccd_substeps=1)
    start = dict(kind='run_start', schema_version=1, study_kind=STUDY_KIND,
        case_id=ctx.case_id, variant=ctx.variant, run_label=process['label'], target=ctx.target_name,
        target_position_rad_word=target, observer_enabled=process['observer']=='on',
        expected_outer_intervals=1, native_scalar_bits=32,
        state_codec='postcard-1.1.3-raw-exact-consumption', state_scope=STATE_SCOPE,
        declared_conditions=declared)
    companion_schedule, stage_schedule = schedules()
    companion, stages = [], []
    velocity, stored_velocity, q = 0, 0, 0
    for slice_ in range(1):
        inputs = dict(coordinate_word=q, target_position_word=target,
                      target_velocity_word=0, stiffness_word=ctx.k, damping_word=ctx.d, duration_word=ctx.h)
        expected = _expected_words(inputs, ctx.cap, None, 'ForceBased', F32())
        row_ = dict(joint_handle={'index':1,'generation':0}, link_id=1, axis='AngX',
            gain_model='ForceBased', constraint_index=0, solver_velocity_offset=0,
            ndofs2=1,j_id2=0,jacobian_words=[ONE],weighted_jacobian_words=[rounded(1/reference(ctx.case)['inertia'])],
            rhs_word=expected['rhs_word'],rhs_without_bias_word=expected['rhs_without_bias_word'],
            cfm_gain_word=expected['cfm_gain_word'],cfm_coefficient_word=0,
            inverse_lhs_word=0,impulse_word=0,
            impulse_bound_words=[expected['lower_impulse_bound_word'],expected['upper_impulse_bound_word']],
            effort_ceiling_word=ctx.cap)
        den = F32().add(row_['weighted_jacobian_words'][0], row_['cfm_gain_word']) if ctx.variant == 'candidate' else row_['weighted_jacobian_words'][0]
        row_['inverse_lhs_word'] = F32().inverse(den)
        build = dict(inputs,row=copy.deepcopy(row_),limits_words=None,
            inverse_duration_word=expected['inverse_duration_word'],
            erp_inverse_duration_word=expected['erp_inverse_duration_word'],
            effective_target_velocity_word=0)
        local = {}
        q_before, prior_mb = q, stored_velocity
        for phase, iteration, visit in stage_schedule:
            if phase == 'AfterMotorSolve':
                computed = F32().projection(row_, [velocity])
                row_['impulse_word'] = computed['next_impulse_word']
                velocity = computed['next_generalized_velocity_words'][0]
            if phase == 'AfterIntegration':
                stored_velocity = velocity
                f = F32();q = f.add(q, f.multiply(velocity, ctx.h))
            uninit = phase in ('BeforeExternalForces','AfterExternalForces')
            e=dict(ccd_partition_index=0,island_index=0,internal_slice_index=slice_,
                expected_internal_slice_count=1,iteration_index=iteration,
                joint_visit_index=visit,stage=phase,native_scalar_bits=32,duration_word=ctx.h,
                row_cache_slice_index=None if uninit else slice_,
                generalized_velocity_words=[velocity],rows=[] if uninit else [copy.deepcopy(row_)])
            stages.append(e);local[(phase,iteration)]=e
        for phase, iteration in companion_schedule:
            uninit=phase in ('BeforeExternalForces','AfterExternalForces')
            mapping={'AfterRowConstruction':'AfterJointUpdate','BeforePositionIntegration':'AfterContactFriction','AfterPositionIntegration':'AfterIntegration'}
            visit_iteration=pgs-1 if phase=='BeforePositionIntegration' else iteration
            e=local[(mapping.get(phase,phase),visit_iteration)]
            coordinate=q_before if phase in ('AfterRowConstruction','BeforePositionIntegration') else q
            mb_v=prior_mb if phase=='AfterRowConstruction' else stored_velocity
            companion.append(dict(ccd_partition_index=0,island_index=0,internal_slice_index=slice_,
                expected_internal_slice_count=1,iteration_index=iteration,phase=phase,
                native_scalar_bits=32,duration_word=ctx.h,row_cache_slice_index=None if uninit else slice_,
                availability='Uninitialized' if uninit else 'Available',
                integration_multibody_handle_words=[0,0] if phase in ('BeforePositionIntegration','AfterPositionIntegration') else None,
                generalized_velocity_words=copy.deepcopy(e['generalized_velocity_words']),
                constructed_rows=[copy.deepcopy(build)] if phase=='AfterRowConstruction' else [],
                endpoints=[] if uninit else [dict(row=copy.deepcopy(e['rows'][0]),coordinate_word=coordinate,multibody_velocity_words=[mb_v])]))
    final=copy.deepcopy(companion[-1]);final['phase']='AfterVelocityWriteback'
    final['endpoints'][0]['multibody_velocity_words']=[velocity]
    companion.append(final)
    checkpoint_=mock_checkpoint(ctx)
    checkpoint_['public_multibody_words'][0]['generalized_velocity_words']=[velocity]
    checkpoint_['public_multibody_words'][0]['joints'][1]['coordinates_words'][3]=q
    comp=dict(schema_version=1,source_contract='InternalUnitMotorConstructionV1',native_scalar_bits=32,
        numerical_outer_interval_word=ctx.h,ccd_partition_duration_words=[ctx.h],events=companion)
    stage=dict(native_scalar_bits=32,numerical_outer_interval_word=ctx.h,ccd_partition_duration_words=[ctx.h],events=stages)
    on=process['observer']=='on'
    done=dict(kind='completed_interval',schema_version=1,completed_outer_interval=1,
        checkpoint=checkpoint_,construction_companion=comp if on else None,
        stage_v1=stage if on else None,impulse_v1_ieee_words=mock_impulse(ctx, final['endpoints'][0]['row']) if on else None)
    end=dict(kind='run_end',schema_version=1,status='collected',progress=dict(
        fixture_construction_started=True,fixture_constructed=True,pipeline_step_started=True,
        pipeline_step_returned=True,completed_outer_intervals=1,initial_records_written=1,
        completed_records_written=1,partial_records_written=0),records_written_before_terminal=3,error=None)
    return [start,dict(kind='initial_state',schema_version=1,checkpoint=mock_checkpoint(ctx)),done,end]

