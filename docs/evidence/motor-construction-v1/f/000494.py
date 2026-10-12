"""Strict whole-cohort admission for the prospective isolated one-interval fixture."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from construction_oracle import verify_construction
from integration_oracle import verify_scalar_advance
from stage_numeric import F32, ONE, ZERO, exact, kind
from strict_io import bound, boolean, canonical, descriptor, gzip_records, integer, keys, loads, need, words, write_exclusive

DT = 0x3a83126f
H = 0x3803126f
K = 0x43960000
D = 0x41400000
CAP = 0x40000000
STATE_SCOPE = 'sorted individual native rigid bodies, collider, unique multibody, exact integration parameters and public exposed IEEE words; excludes full-set metadata, broad/narrow phase, private pipeline/solver/contact caches and observer buffers'
CASES = [dict(label=target + '_' + mode, target=target, observer=observer)
    for target in ('negative', 'positive')
    for mode, observer in [('on', 'on'), ('fresh_on', 'on'), ('off', 'off')]]
ROW_KEYS = 'joint_handle link_id axis gain_model constraint_index solver_velocity_offset ndofs2 j_id2 jacobian_words weighted_jacobian_words rhs_word rhs_without_bias_word cfm_gain_word cfm_coefficient_word inverse_lhs_word impulse_word impulse_bound_words effort_ceiling_word'
ROW_SCALARS = 'rhs_word rhs_without_bias_word cfm_gain_word cfm_coefficient_word inverse_lhs_word impulse_word effort_ceiling_word'.split()
BUILD_KEYS = 'row coordinate_word target_position_word target_velocity_word stiffness_word damping_word duration_word inverse_duration_word limits_words effective_target_velocity_word erp_inverse_duration_word'
EVENT_KEYS = 'ccd_partition_index island_index internal_slice_index expected_internal_slice_count iteration_index phase native_scalar_bits duration_word row_cache_slice_index availability integration_multibody_handle_words generalized_velocity_words constructed_rows endpoints'
STAGE_KEYS = 'ccd_partition_index island_index internal_slice_index expected_internal_slice_count iteration_index joint_visit_index stage native_scalar_bits duration_word row_cache_slice_index generalized_velocity_words rows'
COMPANION_SCHEDULE = [('BeforeExternalForces', None), ('AfterExternalForces', None),
    ('AfterRowConstruction', None), ('BeforePositionIntegration', None),
    ('AfterPositionIntegration', None), ('AfterJointStabilization', 0),
    ('AfterJointStabilization', 1), ('FinalSlice', None)]
STAGE_SCHEDULE = [(s, None, None) for s in ('BeforeExternalForces', 'AfterExternalForces',
    'AfterJointUpdate', 'AfterContactUpdate', 'AfterContactWarmstart')]
STAGE_SCHEDULE += [(s, 0, 0 if 'Motor' in s else None) for s in
    ('BeforeJointSolve', 'BeforeMotorSolve', 'AfterMotorSolve', 'AfterJointSolve',
     'AfterContactNormal', 'AfterContactFriction')]
STAGE_SCHEDULE += [('AfterIntegration', None, None)]
for i in range(2):
    STAGE_SCHEDULE += [(s, i, 0 if 'Motor' in s else None) for s in
        ('BeforeJointStabilization', 'BeforeMotorBiasRemoval', 'BeforeMotorSolve',
         'AfterMotorSolve', 'AfterJointStabilization', 'AfterContactStabilization')]
STAGE_SCHEDULE += [('AfterStabilizationFriction', None, None), ('FinalSlice', None, None)]


def same(a, b, message):
    need(canonical(a) == canonical(b), message)


def nullable_index(value):
    if value is not None:
        integer(value)


def row(value):
    keys(value, ROW_KEYS)
    keys(value['joint_handle'], 'index generation')
    for x in value['joint_handle'].values(): integer(x, 32)
    same(value['joint_handle'], {'index': 1, 'generation': 0}, 'wrong native child handle')
    for name in ('link_id', 'constraint_index', 'solver_velocity_offset', 'ndofs2', 'j_id2'):
        integer(value[name])
    need((value['link_id'], value['constraint_index'], value['solver_velocity_offset'],
          value['ndofs2'], value['j_id2']) == (1, 0, 0, 1, 0), 'wrong native row/block identity')
    need(value['axis'] == 'AngX' and value['gain_model'] == 'ForceBased', 'unsupported row')
    words(value['jacobian_words'], 1); words(value['weighted_jacobian_words'], 1)
    need(value['jacobian_words'] == [ONE], 'wrong unit angular Jacobian')
    for name in ROW_SCALARS: integer(value[name], 32)
    words(value['impulse_bound_words'], 2)
    need(value['effort_ceiling_word'] == CAP, 'wrong bound motor descriptor ceiling')


def endpoint(value):
    keys(value, 'row coordinate_word multibody_velocity_words')
    row(value['row']); integer(value['coordinate_word'], 32)
    words(value['multibody_velocity_words'], 1)


def conditions(value):
    expected = dict(sphere_radius_m_word=0x3dcccccd, sphere_mass_kg_word=ONE,
        outer_dt_s_word=DT, stiffness_nm_per_rad_word=K, damping_nm_s_per_rad_word=D,
        ceiling_nm_word=CAP, target_velocity_rad_s_word=0, gravity_words=[0, 0, 0],
        initial_generalized_velocity_word=0, passive_damping_word=0, armature_word=0,
        small_steps=32, pgs_passes=1, stabilization_passes=2, contacts_enabled=False,
        sleep_enabled=False, ccd_enabled=False, identity_coincident_x_frames=True)
    same(value, expected, 'declared fixture differs')


def checkpoint(c, initial=False):
    keys(c, 'schema_version native_scalar_bits state_scope gravity_words integration_parameters records public_body_words public_multibody_words parent_handle_words child_handle_words joint_handle_words child_link_index')
    for name in ('schema_version', 'native_scalar_bits', 'child_link_index'): integer(c[name])
    need((c['schema_version'], c['native_scalar_bits'], c['child_link_index']) == (1, 32, 1), 'checkpoint version/child')
    need(c['state_scope'] == STATE_SCOPE, 'checkpoint scope differs')
    same(c['gravity_words'], [0, 0, 0], 'nonzero gravity')
    for name, expected in [('parent_handle_words', [0, 0]), ('child_handle_words', [1, 0]), ('joint_handle_words', [1, 0])]:
        words(c[name], 2); same(c[name], expected, 'checkpoint handle differs')
    p = c['integration_parameters']; keys(p, 'native_f32_words native_counts postcard_bytes')
    words(p['native_f32_words'], 11); words(p['native_counts'], 6, 64); words(p['postcard_bytes'], width=8)
    need(p['postcard_bytes'], 'empty parameter serialization')
    f = F32(); min_ccd = f.divide(f.divide(ONE, 0x42700000), 0x42c80000)
    expected_params = [DT, min_ccd, 0x40a00000, 0x41f00000, 0x49742400, ONE,
                       ONE, ONE, DT, 0x41200000, 0x3b03126f]
    same(p['native_f32_words'], expected_params, 'integration parameter words differ')
    same(p['native_counts'], [32, 0, 1, 2, 128, 1], 'integration parameter counts differ')
    need(type(c['records']) is list and len(c['records']) == 4, 'wrong source-bound native object count')
    for obj, (kind_, handle) in zip(c['records'], [('Body', [0, 0]), ('Body', [1, 0]), ('Collider', [0, 0]), ('Multibody', [0, 0])]):
        keys(obj, 'kind handle_words postcard_bytes'); words(obj['handle_words'], 2)
        need(obj['kind'] == kind_ and obj['handle_words'] == handle, 'wrong sorted native object identity')
        words(obj['postcard_bytes'], width=8); need(obj['postcard_bytes'], 'empty native object serialization')
    need(type(c['public_body_words']) is list and len(c['public_body_words']) == 2, 'wrong body count')
    for body, handle in zip(c['public_body_words'], ([0, 0], [1, 0])):
        keys(body, 'handle_words position_words linear_velocity_words angular_velocity_words user_force_words user_torque_words mass_query_kg_word effective_angular_inertia_query_kg_m2_words')
        words(body['handle_words'], 2); same(body['handle_words'], handle, 'wrong public body handle')
        for name, n in [('position_words', 7), ('linear_velocity_words', 3), ('angular_velocity_words', 3), ('user_force_words', 3), ('user_torque_words', 3), ('effective_angular_inertia_query_kg_m2_words', 9)]: words(body[name], n)
        integer(body['mass_query_kg_word'], 32)
    need(type(c['public_multibody_words']) is list and len(c['public_multibody_words']) == 1, 'wrong multibody count')
    mb = c['public_multibody_words'][0]
    keys(mb, 'handle_words ndofs links generalized_velocity_words damping_words joints')
    words(mb['handle_words'], 2); same(mb['handle_words'], [0, 0], 'wrong multibody root')
    integer(mb['ndofs']); integer(mb['links']); need((mb['ndofs'], mb['links']) == (1, 2), 'wrong native topology')
    words(mb['generalized_velocity_words'], 1); words(mb['damping_words'], 1)
    same(mb['damping_words'], [0], 'nonzero passive damping')
    need(type(mb['joints']) is list and len(mb['joints']) == 2, 'wrong joint count')
    for i, joint in enumerate(mb['joints']):
        keys(joint, 'link_index body_handle_words coordinates_words joint_rotation_words descriptor_motor_impulse_words')
        integer(joint['link_index']); need(joint['link_index'] == i, 'wrong joint link index')
        words(joint['body_handle_words'], 2); same(joint['body_handle_words'], [i, 0], 'wrong joint body')
        words(joint['coordinates_words'], 6); words(joint['joint_rotation_words'], 4)
        words(joint['descriptor_motor_impulse_words'], 6)
    if initial:
        same(mb['generalized_velocity_words'], [0], 'nonzero initial generalized velocity')
        for joint in mb['joints']: same(joint['coordinates_words'], [0] * 6, 'nonzero initial coordinates')
    return mb


def context(e, slice_, iteration):
    for name in ('ccd_partition_index', 'island_index', 'internal_slice_index',
                 'expected_internal_slice_count', 'native_scalar_bits', 'duration_word'): integer(e[name])
    nullable_index(e['iteration_index']); nullable_index(e['row_cache_slice_index'])
    need((e['ccd_partition_index'], e['island_index'], e['internal_slice_index'],
          e['expected_internal_slice_count'], e['native_scalar_bits'], e['duration_word']) ==
         (0, 0, slice_, 32, 32, H), 'wrong event context/duration')
    same(e['iteration_index'], iteration, 'wrong visit iteration')
    words(e['generalized_velocity_words'], 1)


def telemetry_header(t, names):
    keys(t, names)
    integer(t['native_scalar_bits']); integer(t['numerical_outer_interval_word'], 32)
    need((t['native_scalar_bits'], t['numerical_outer_interval_word']) == (32, DT), 'wrong telemetry interval')
    words(t['ccd_partition_duration_words'], 1); same(t['ccd_partition_duration_words'], [DT], 'unexpected CCD partition')
    need(type(t['events']) is list, 'bad events')


def qualify_observers(done, target_word):
    comp, stage = done['construction_companion'], done['stage_v1']
    telemetry_header(comp, 'schema_version source_contract native_scalar_bits numerical_outer_interval_word ccd_partition_duration_words events')
    integer(comp['schema_version']); need(comp['schema_version'] == 1 and comp['source_contract'] == 'InternalUnitMotorConstructionV1', 'wrong construction source contract')
    telemetry_header(stage, 'numerical_outer_interval_word native_scalar_bits ccd_partition_duration_words events')
    need(len(comp['events']) == 257, 'incomplete/unexpected construction event schedule')
    need(len(stage['events']) == 832, 'incomplete/unexpected stage-v1 event schedule')
    stages = {}
    for slice_ in range(32):
        for offset, (phase, iteration, visit) in enumerate(STAGE_SCHEDULE):
            e = stage['events'][slice_ * len(STAGE_SCHEDULE) + offset]
            keys(e, STAGE_KEYS); context(e, slice_, iteration)
            nullable_index(e['joint_visit_index']); same(e['joint_visit_index'], visit, 'wrong joint visit')
            need(e['stage'] == phase, 'wrong stage order')
            uninit = phase in ('BeforeExternalForces', 'AfterExternalForces')
            same(e['row_cache_slice_index'], None if uninit else slice_, 'wrong stage cache generation')
            need(type(e['rows']) is list and len(e['rows']) == (0 if uninit else 1), 'wrong stage availability')
            for r in e['rows']: row(r)
            stages[(slice_, phase, iteration)] = e
        # This source-bound fixture has only one child collider and no contacts.
        # Check recorded no-op contact boundaries without attributing motor force.
        local = stage['events'][slice_ * 26:(slice_ + 1) * 26]
        for before, after in [(2, 3), (3, 4), (8, 9), (9, 10), (16, 17), (22, 23), (23, 24)]:
            same(local[before]['generalized_velocity_words'], local[after]['generalized_velocity_words'], 'zero-contact boundary changed generic V')
            same(local[before]['rows'], local[after]['rows'], 'zero-contact boundary changed motor row')
    analyses = []
    q = 0
    for slice_ in range(32):
        events = {}
        for offset, (phase, iteration) in enumerate(COMPANION_SCHEDULE):
            e = comp['events'][slice_ * 8 + offset]
            keys(e, EVENT_KEYS); context(e, slice_, iteration)
            need(e['phase'] == phase, 'wrong companion phase order')
            uninit = phase in ('BeforeExternalForces', 'AfterExternalForces')
            same(e['row_cache_slice_index'], None if uninit else slice_, 'wrong companion cache generation')
            need(e['availability'] == ('Uninitialized' if uninit else 'Available'), 'unexpected row availability')
            root = [0, 0] if phase in ('BeforePositionIntegration', 'AfterPositionIntegration') else None
            same(e['integration_multibody_handle_words'], root, 'wrong integration multibody/root availability')
            if root is not None: words(e['integration_multibody_handle_words'], 2)
            need(type(e['endpoints']) is list and len(e['endpoints']) == (0 if uninit else 1), 'missing/extra endpoint')
            for item in e['endpoints']: endpoint(item)
            need(type(e['constructed_rows']) is list and len(e['constructed_rows']) == (1 if phase == 'AfterRowConstruction' else 0), 'missing/extra actual constructor input')
            mapping = {'AfterRowConstruction': 'AfterJointUpdate', 'BeforePositionIntegration': 'AfterContactFriction', 'AfterPositionIntegration': 'AfterIntegration'}
            stage_iteration = 0 if phase == 'BeforePositionIntegration' else iteration
            v1 = stages[(slice_, mapping.get(phase, phase), stage_iteration)]
            same(e['generalized_velocity_words'], v1['generalized_velocity_words'], 'companion/stage generic V differs')
            same([x['row'] for x in e['endpoints']], v1['rows'], 'companion/stage row snapshot differs')
            events[(phase, iteration)] = e
        build = events[('AfterRowConstruction', None)]['constructed_rows'][0]
        keys(build, BUILD_KEYS); row(build['row'])
        for name in BUILD_KEYS.split():
            if name not in ('row', 'limits_words'): integer(build[name], 32)
        need(build['limits_words'] is None, 'fixture unexpectedly has joint limits')
        need((build['target_position_word'], build['target_velocity_word'], build['stiffness_word'], build['damping_word'], build['duration_word']) == (target_word, 0, K, D, H), 'actual motor inputs differ from registered fixture')
        same(build['row'], events[('AfterRowConstruction', None)]['endpoints'][0]['row'], 'constructed and initialized endpoint rows differ')
        need(build['coordinate_word'] == q, 'wrong build coordinate chronology')
        need(build['row']['impulse_word'] == 0, 'constructor impulse is not +zero')
        weighted = build['row']['weighted_jacobian_words'][0]
        diag = F32(); inverse = diag.inverse(weighted)
        need(build['row']['inverse_lhs_word'] == inverse, 'wrong native initialized inverse lhs')
        construction = verify_construction(build)
        before = events[('BeforePositionIntegration', None)]['endpoints'][0]
        after = events[('AfterPositionIntegration', None)]['endpoints'][0]
        need(before['coordinate_word'] == q, 'wrong integration-input coordinate')
        integration = verify_scalar_advance(H, before, after,
            events[('BeforePositionIntegration', None)]['generalized_velocity_words'])
        same(events[('BeforePositionIntegration', None)]['generalized_velocity_words'],
             events[('AfterPositionIntegration', None)]['generalized_velocity_words'], 'integration changed generic V')
        q = after['coordinate_word']
        for phase, iteration in [('AfterRowConstruction', None), ('BeforePositionIntegration', None)]:
            need(events[(phase, iteration)]['endpoints'][0]['coordinate_word'] == build['coordinate_word'], 'coordinate changed before integration')
        for phase, iteration in [('AfterPositionIntegration', None), ('AfterJointStabilization', 0), ('AfterJointStabilization', 1), ('FinalSlice', None)]:
            ep = events[(phase, iteration)]['endpoints'][0]
            need(ep['coordinate_word'] == q, 'coordinate changed after integration')
            same(ep['multibody_velocity_words'], after['multibody_velocity_words'], 'multibody V changed before final writeback')
        analyses.append(dict(slice=slice_, construction=construction, integration=integration,
            diagonal_exclusions=sorted(diag.exclusions), q_before_rad_word=before['coordinate_word'],
            q_after_rad_word=after['coordinate_word'],
            integration_input_velocity_rad_s_word=before['multibody_velocity_words'][0],
            stabilization_generic_velocity_rad_s_words=[events[('AfterJointStabilization', i)]['generalized_velocity_words'][0] for i in range(2)],
            stabilized_MB_velocity_rad_s_words=[events[('AfterJointStabilization', i)]['endpoints'][0]['multibody_velocity_words'][0] for i in range(2)],
            final_slice_generic_velocity_rad_s_word=events[('FinalSlice', None)]['generalized_velocity_words'][0]))
    final = comp['events'][-1]; keys(final, EVENT_KEYS); context(final, 31, None)
    need(final['phase'] == 'AfterVelocityWriteback' and final['availability'] == 'Available', 'missing final writeback')
    same(final['row_cache_slice_index'], 31, 'wrong writeback cache'); need(final['integration_multibody_handle_words'] is None, 'unexpected writeback integration root')
    need(final['constructed_rows'] == [] and type(final['endpoints']) is list and len(final['endpoints']) == 1, 'bad writeback endpoint')
    endpoint(final['endpoints'][0])
    last = comp['events'][-2]
    same(final['generalized_velocity_words'], last['generalized_velocity_words'], 'writeback changed generic V')
    same(final['endpoints'][0]['row'], last['endpoints'][0]['row'], 'writeback changed final row')
    same(final['endpoints'][0]['multibody_velocity_words'], final['generalized_velocity_words'], 'final generic/MB copy differs')
    mb = done['checkpoint']['public_multibody_words'][0]
    same(mb['generalized_velocity_words'], final['generalized_velocity_words'], 'completed MB V differs from writeback')
    need(final['endpoints'][0]['coordinate_word'] == q == mb['joints'][1]['coordinates_words'][3], 'completed q/writeback differs')
    analyses[-1]['final_writeback_velocity_rad_s_word'] = final['generalized_velocity_words'][0]
    impulse_schema(done['impulse_v1_ieee_words'])
    return analyses


def impulse_value(v):
    keys(v, 'kind applied_impulse_word effort_ceiling_word rounded_bound_word')
    need(v['kind'] == 'Revolute', 'wrong motor impulse kind')
    integer(v['applied_impulse_word'])
    for key in ('effort_ceiling_word', 'rounded_bound_word'):
        need(v[key] is not None, 'unexpected unbounded motor impulse'); integer(v[key])


def impulse_schema(t):
    keys(t, 'scalar_bits numerical_outer_interval_s_word ccd_partition_duration_s_words motors')
    integer(t['scalar_bits']); need(t['scalar_bits'] == 64, 'wrong promoted impulse width')
    integer(t['numerical_outer_interval_s_word']); words(t['ccd_partition_duration_s_words'], 1, 64)
    need(type(t['motors']) is list and len(t['motors']) == 1, 'missing/extra impulse motor')
    r = t['motors'][0]
    keys(r, 'joint_handle_words link_id joint_dof_id axis impulse observed_solver_interval_s_word observed_solver_slice_count full_step_coverage observed_partition_count finite_positive_bound_eligible_slice_count finite_positive_bound_eligible_duration_s_word finite_positive_bound_active_slice_count finite_positive_bound_active_duration_s_word slices')
    words(r['joint_handle_words'], 2); same(r['joint_handle_words'], [1, 0], 'wrong impulse motor handle')
    for key in ('link_id', 'joint_dof_id', 'observed_solver_interval_s_word', 'observed_solver_slice_count', 'observed_partition_count', 'finite_positive_bound_eligible_slice_count', 'finite_positive_bound_eligible_duration_s_word', 'finite_positive_bound_active_slice_count', 'finite_positive_bound_active_duration_s_word'): integer(r[key])
    boolean(r['full_step_coverage']); need(r['full_step_coverage'] is True and r['observed_solver_slice_count'] == 32 and r['observed_partition_count'] == 1 and r['link_id'] == 1 and r['joint_dof_id'] == 0 and r['axis'] == 'AngX', 'incomplete impulse identity/coverage')
    impulse_value(r['impulse']); need(type(r['slices']) is list and len(r['slices']) == 32, 'incomplete impulse slices')
    for i, item in enumerate(r['slices']):
        keys(item, 'ccd_partition_index internal_slice_index expected_internal_slice_count duration_s_word impulse finite_positive_bound_eligible finite_positive_bound_active')
        for key in ('ccd_partition_index', 'internal_slice_index', 'expected_internal_slice_count', 'duration_s_word'): integer(item[key])
        need((item['ccd_partition_index'], item['internal_slice_index'], item['expected_internal_slice_count']) == (0, i, 32), 'wrong impulse ordinal')
        impulse_value(item['impulse']); boolean(item['finite_positive_bound_eligible']); boolean(item['finite_positive_bound_active'])


def validate_run(path, case):
    records, lines, framing = gzip_records(path)
    need(len(records) == 4, 'incomplete/failed/unexpected run record count')
    start, initial, done, end = records
    keys(start, 'kind schema_version run_label target target_position_rad_word observer_enabled expected_outer_intervals native_scalar_bits state_codec state_scope declared_conditions')
    target = DT | (0x80000000 if case['target'] == 'negative' else 0)
    need(start['kind'] == 'run_start' and start['run_label'] == case['label'] and start['target'] == case['target'], 'run identity differs')
    for key in ('schema_version', 'target_position_rad_word', 'expected_outer_intervals', 'native_scalar_bits'): integer(start[key])
    boolean(start['observer_enabled'])
    need((start['schema_version'], start['target_position_rad_word'], start['expected_outer_intervals'], start['native_scalar_bits']) == (1, target, 1, 32), 'run header differs')
    need(start['observer_enabled'] is (case['observer'] == 'on') and start['state_codec'] == 'postcard-1.1.3-raw-exact-consumption' and start['state_scope'] == STATE_SCOPE, 'observer/codec/scope differs')
    conditions(start['declared_conditions'])
    keys(initial, 'kind schema_version checkpoint'); integer(initial['schema_version'])
    need(initial['kind'] == 'initial_state' and initial['schema_version'] == 1, 'bad initial record')
    checkpoint(initial['checkpoint'], initial=True)
    keys(done, 'kind schema_version completed_outer_interval checkpoint construction_companion stage_v1 impulse_v1_ieee_words')
    integer(done['schema_version']); integer(done['completed_outer_interval'])
    need(done['kind'] == 'completed_interval' and (done['schema_version'], done['completed_outer_interval']) == (1, 1), 'bad completed record')
    checkpoint(done['checkpoint'])
    analyses = []
    if case['observer'] == 'on': analyses = qualify_observers(done, target)
    else:
        need(all(done[key] is None for key in ('construction_companion', 'stage_v1', 'impulse_v1_ieee_words')), 'OFF telemetry must be absent')
    keys(end, 'kind schema_version status progress records_written_before_terminal error')
    integer(end['schema_version']); integer(end['records_written_before_terminal'])
    need(end['kind'] == 'run_end' and end['schema_version'] == 1 and end['status'] == 'collected' and end['error'] is None and end['records_written_before_terminal'] == 3, 'failed native terminal')
    expected_progress = dict(fixture_construction_started=True, fixture_constructed=True,
        pipeline_step_started=True, pipeline_step_returned=True, completed_outer_intervals=1,
        initial_records_written=1, completed_records_written=1, partial_records_written=0)
    same(end['progress'], expected_progress, 'native progress incomplete')
    return dict(case=case, framing=framing, initial=initial['checkpoint'], completed=done['checkpoint'],
        telemetry={key: done[key] for key in ('construction_companion', 'stage_v1', 'impulse_v1_ieee_words')},
        literal_telemetry_bytes=lines[2], analyses=analyses)


def load_spec(path, expected_sha):
    need(descriptor(path)['sha256'] == expected_sha, 'registration hash differs from independently supplied SHA')
    wrapper = loads(Path(path).read_bytes())
    keys(wrapper, 'kind schema_version candidate admission')
    need(wrapper['kind'] == 'admitted_isolated_motor_construction_cohort' and type(wrapper['schema_version']) is int and wrapper['schema_version'] == 1, 'invalid final admission wrapper')
    bound(wrapper['candidate']); bound(wrapper['admission'])
    spec = loads(Path(wrapper['candidate']['path']).read_bytes())
    keys(spec, 'kind schema_version registered_utc collection_worlds_before_registration cases binary cwd output_root frozen_inputs registration_artifacts source_scope environment timeout_seconds process_failure_policy driver_run_receipt_path driver_interpreter checked_driver_source')
    need(spec['kind'] == 'frozen_isolated_motor_construction_cohort' and spec['schema_version'] == 1 and type(spec['schema_version']) is int and spec['collection_worlds_before_registration'] == 0 and type(spec['collection_worlds_before_registration']) is int, 'not prospectively registered')
    same(spec['cases'], CASES, 'case order must be fixed six')
    dt = datetime.fromisoformat(spec['registered_utc']); need(dt.tzinfo is not None and dt <= datetime.now(timezone.utc), 'bad registration timestamp')
    for key in ('cwd', 'output_root'): need(type(spec[key]) is str and Path(spec[key]).is_absolute(), 'absolute registered path required')
    for key in ('cwd', 'output_root'):
        p = Path(spec[key]); need(p.resolve() == p and p != Path('/workspace') and Path('/workspace') in p.parents, 'registered paths must be canonical persistent /workspace paths without symlink parents')
    need(Path(spec['cwd']).is_dir(), 'registered cwd missing')
    receipt_path = Path(spec['driver_run_receipt_path'])
    need(receipt_path.is_absolute() and receipt_path.resolve() == receipt_path and Path('/workspace') in receipt_path.parents and receipt_path.name == 'actual-command.json', 'invalid persistent checked-driver receipt path')
    bound(spec['driver_interpreter']); need(spec['driver_interpreter']['path'] == '/usr/bin/python3', 'checked driver interpreter must be pinned /usr/bin/python3')
    bound(spec['checked_driver_source']); need(spec['checked_driver_source']['path'] == str(Path(__file__).parent / 'checked_driver.py'), 'wrong checked-driver source')
    need(spec['process_failure_policy'] == 'stop_on_first_failure_no_resume_or_replacement', 'undeclared process failure policy')
    integer(spec['timeout_seconds']); need(1 <= spec['timeout_seconds'] <= 120, 'timeout must be bounded')
    need(type(spec['source_scope']) is str and spec['source_scope'], 'source scope must be explicit')
    need(type(spec['environment']) is dict and set(spec['environment']) == {'RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTDOCFLAGS', 'RUST_TEST_THREADS'}, 'unexpected environment binding scope')
    for value in spec['environment'].values(): need(value is None or type(value) is str, 'invalid environment binding')
    bound(spec['binary'])
    need(type(spec['frozen_inputs']) is list and spec['frozen_inputs'], 'missing frozen source/tool closure')
    paths = []
    for item in spec['frozen_inputs']: bound(item); paths.append(item['path'])
    need(len(paths) == len(set(paths)), 'duplicate frozen path')
    for name in ('strict_io.py', 'cohort_reader.py', 'cohort_driver.py', 'checked_driver.py', 'construction_oracle.py', 'integration_oracle.py', 'stage_numeric.py'):
        need(str(Path(__file__).parent / name) in paths, 'reader/driver runtime source is not frozen')
    artifacts = spec['registration_artifacts']
    keys(artifacts, 'protocol collector_source_manifest compiler_feature_receipt producer_review pure_tests_review')
    for artifact in artifacts.values(): bound(artifact)
    review = loads(Path(wrapper['admission']['path']).read_bytes())
    need(review.get('status') == 'PASS_PRE_DATA_SOURCE_PROTOCOL_AND_DRIVER' and
         review.get('reviewed_candidate_sha256') == wrapper['candidate']['sha256'] and
         review.get('binary') == spec['binary'] and
         review.get('frozen_inputs_sha256') == hashlib.sha256(canonical(spec['frozen_inputs'])).hexdigest(), 'independent admission does not bind candidate/source/binary')
    spec['_authenticated_wrapper_sha256'] = expected_sha
    spec['_authenticated_candidate_sha256'] = wrapper['candidate']['sha256']
    spec['_authenticated_wrapper_path'] = str(Path(path))
    return spec


def expected_driver_command(spec):
    return ['/usr/bin/python3', '-B', str(Path(__file__).parent / 'cohort_driver.py'),
            '--spec', spec['_authenticated_wrapper_path'], '--spec-sha256',
            spec['_authenticated_wrapper_sha256'], '--launch-native']


def qualify_cohort(spec, result_root):
    root = Path(result_root)
    need(spec.get('_authenticated_wrapper_sha256') and spec.get('_authenticated_candidate_sha256'), 'mandatory admission authentication missing')
    need(root.resolve() == Path(spec['output_root']).resolve(), 'wrong registered cohort root')
    terminal = loads((root / 'driver-terminal.json').read_bytes())
    command_receipt = loads(Path(spec['driver_run_receipt_path']).read_bytes())
    keys(command_receipt, 'kind schema_version command cwd registered_wrapper_sha256 registered_candidate_sha256 started_utc ended_utc driver_process_started driver_process_id launch_outcome actual_exit_code source_inputs_unchanged source_guard_error runtime_error stdout stderr')
    integer(command_receipt['schema_version']); integer(command_receipt['actual_exit_code']); integer(command_receipt['driver_process_id'])
    boolean(command_receipt['driver_process_started']); boolean(command_receipt['source_inputs_unchanged'])
    need(command_receipt['kind'] == 'original_checked_driver_command' and command_receipt['schema_version'] == 2 and command_receipt['actual_exit_code'] == 0 and command_receipt['driver_process_started'] is True and command_receipt['launch_outcome'] == 'known_started_exit_observed' and command_receipt['source_inputs_unchanged'] is True and command_receipt['source_guard_error'] is None and command_receipt['runtime_error'] is None, 'checked outer driver original exit or source guard failed')
    same(command_receipt['command'], expected_driver_command(spec), 'checked original driver command differs')
    need(command_receipt['cwd'] == spec['cwd'] and command_receipt['registered_wrapper_sha256'] == spec['_authenticated_wrapper_sha256'] and command_receipt['registered_candidate_sha256'] == spec['_authenticated_candidate_sha256'], 'checked driver binding differs')
    for key in ('stdout', 'stderr'): bound(command_receipt[key])
    stdout = Path(command_receipt['stdout']['path']).read_bytes()
    need(stdout.endswith(b'\n') and stdout.splitlines(), 'checked driver stdout not complete')
    same(loads(stdout.splitlines()[-1]), terminal, 'checked driver final memory JSON differs from disk terminal')
    keys(terminal, 'kind driver_schema_version status ended_utc actual_processes_started native_processes_started_count_is_complete original_process_terminals observed_process_terminal_memories prelaunch_directory_barriers_complete source_inputs_unchanged error registered_spec_sha256 registered_candidate_sha256 cases case_launch_accounting retries resumed unlaunched_case_labels launch_outcome_unknown_case_labels')
    for key in ('driver_schema_version', 'actual_processes_started', 'original_process_terminals', 'observed_process_terminal_memories', 'retries'): integer(terminal[key])
    boolean(terminal['source_inputs_unchanged']); boolean(terminal['resumed'])
    boolean(terminal['native_processes_started_count_is_complete']); boolean(terminal['prelaunch_directory_barriers_complete'])
    need(terminal['driver_schema_version'] == 2 and terminal['observed_process_terminal_memories'] == 6 and terminal['native_processes_started_count_is_complete'] is True and terminal['prelaunch_directory_barriers_complete'] is True and terminal['launch_outcome_unknown_case_labels'] == [], 'incomplete version-two launch/storage accounting')
    need(type(terminal['case_launch_accounting']) is list and len(terminal['case_launch_accounting']) == 6, 'missing per-case launch accounting')
    for index, entry in enumerate(terminal['case_launch_accounting']):
        keys(entry, 'case attempt_index status reservation_durable pid actual_exit_code process_terminal_storage storage_error')
        integer(entry['attempt_index']); integer(entry['pid']); integer(entry['actual_exit_code']); boolean(entry['reservation_durable'])
        need(entry['attempt_index'] == index and entry['case'] == CASES[index] and entry['status'] == 'known_started_exit_observed' and entry['reservation_durable'] is True and entry['actual_exit_code'] == 0 and entry['process_terminal_storage'] == 'stored_fsynced' and entry['storage_error'] is None, 'unknown/partial case launch or terminal durability')
    need(terminal['kind'] == 'fixed_six_process_driver_terminal' and terminal['original_process_terminals'] == 6 and terminal['retries'] == 0 and terminal['resumed'] is False and terminal['error'] is None and terminal['unlaunched_case_labels'] == [], 'invalid driver terminal')
    need(terminal.get('status') == 'all_six_processes_completed' and terminal.get('actual_processes_started') == 6, 'driver did not complete six actual unique processes')
    need(terminal['source_inputs_unchanged'] is True and terminal['registered_spec_sha256'] == spec['_authenticated_wrapper_sha256'] and terminal['registered_candidate_sha256'] == spec['_authenticated_candidate_sha256'], 'missing/wrong driver source/spec guard')
    claim = loads((root / 'cohort-claim.json').read_bytes())
    keys(claim, 'kind created_utc cases registered_spec_sha256 registered_candidate_sha256 binary source_inputs')
    need(claim['kind'] == 'exclusive_fixed_six_cohort_claim', 'wrong cohort claim')
    same(claim['cases'], CASES, 'wrong claimed order'); same(claim['binary'], spec['binary'], 'claim binary differs')
    same(claim['source_inputs'], spec['frozen_inputs'], 'claim sources differ')
    need(claim['registered_spec_sha256'] == spec['_authenticated_wrapper_sha256'] and claim['registered_candidate_sha256'] == spec['_authenticated_candidate_sha256'], 'claim registration differs')
    registered = datetime.fromisoformat(spec['registered_utc'])
    claimed = datetime.fromisoformat(claim['created_utc']); need(claimed.tzinfo and claimed >= registered, 'cohort claim preceded registration')
    durability = loads((root / 'prelaunch-durability.json').read_bytes())
    keys(durability, 'kind schema_version root_parent cohort_root completed_utc all_six_claims_durable registered_spec_sha256')
    integer(durability['schema_version']); boolean(durability['all_six_claims_durable'])
    need(durability['kind'] == 'completed_prelaunch_directory_barriers' and durability['schema_version'] == 2 and durability['all_six_claims_durable'] is True and durability['root_parent'] == str(root.parent) and durability['cohort_root'] == str(root) and durability['registered_spec_sha256'] == spec['_authenticated_wrapper_sha256'], 'wrong prelaunch directory barrier record')
    barriers_at = datetime.fromisoformat(durability['completed_utc']); need(barriers_at.tzinfo and claimed <= barriers_at, 'invalid prelaunch barrier chronology')
    need(type(terminal['cases']) is list and len(terminal['cases']) == 6, 'wrong original terminal count')
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
        expected_argv = [spec['binary']['path'], '--target', case['target'], '--observer', case['observer'], '--label', case['label'], '--output', str(folder / 'raw.jsonl.gz')]
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
        need(launch.get('case') == case and result.get('case') == case and result.get('actual_exit_code') == 0, 'failed/mismatched original process exit')
        need(result.get('native_process_started') is True, 'process was not started')
        for key in ('stdout', 'stderr', 'raw_output'): bound(result[key])
        for key, filename in [('stdout', 'stdout.log'), ('stderr', 'stderr.log'), ('raw_output', 'raw.jsonl.gz')]:
            need(result[key]['path'] == str(folder / filename), 'substituted process artifact path')
        run = validate_run(folder / 'raw.jsonl.gz', case)
        runs.append(run)
        for analysis in run['analyses']:
            if any(analysis[key]['status'] != 'DECLARED_FORMULAS_MATCH' for key in ('construction', 'integration')) or analysis['diagonal_exclusions']:
                issues.append(dict(case=case['label'], analysis=analysis))
    need(len(pids) == len(set(pids)), 'native process IDs reused within bounded six-process cohort')
    first_start = datetime.fromisoformat(terminal['cases'][0]['started_utc'])
    need(all(t <= first_start for t in prelaunch_claim_times), 'not all six process claims preceded the first launch')
    for group in (runs[:3], runs[3:]):
        for item in group[1:]:
            same(item['initial'], group[0]['initial'], 'ON/repeat/OFF initial checkpoint differs')
            same(item['completed'], group[0]['completed'], 'ON/repeat/OFF completed checkpoint differs')
        same(group[0]['telemetry'], group[1]['telemetry'], 'fresh ON telemetry typed value differs')
        need(group[0]['literal_telemetry_bytes'] == group[1]['literal_telemetry_bytes'], 'fresh ON completed-record bytes differ')
    for item in spec['frozen_inputs']: bound(item)
    bound(spec['binary'])
    return dict(kind='strict_isolated_motor_construction_cohort_qualification',
        status='PASS_COMPLETE_DECLARED_SCOPE' if not issues else 'FAILED_OR_EXCLUDED_ARITHMETIC_RETAINED',
        actual_processes=6, actual_outer_intervals=6, observed_ON_processes=4,
        construction_events=4 * 257, stage_events=4 * 832, constructor_checks=4 * 32,
        coordinate_advance_checks=4 * 32, issues=issues,
        cases=[dict(case=r['case'], framing=r['framing'], analyses=r['analyses']) for r in runs],
        source_scope=spec['source_scope'], source_inputs_unchanged=True,
        captured_checkpoint_ON_repeat_OFF_exact=True, fresh_ON_complete_record_literal_exact=True,
        off_observers_absent=True, native_motor_accuracy_verified=False,
        exclusions='Full-set/private pipeline caches, runtime MXCSR/FTZ/DAZ, convergence, SI accuracy, physical force origin, coupled/contact causal attribution and candidate promotion are outside this scope. Captured constructor ERP/RHS/CFM/bounds expression fidelity is checked.')


def main():
    p = argparse.ArgumentParser(); p.add_argument('--spec', required=True); p.add_argument('--spec-sha256', required=True)
    p.add_argument('--result-root', required=True); p.add_argument('--output', required=True)
    args = p.parse_args()
    try:
        spec = load_spec(args.spec, args.spec_sha256)
        need(Path(args.result_root).resolve() == Path(spec['output_root']).resolve(), 'wrong registered output root')
        result = qualify_cohort(spec, args.result_root)
    except Exception as error:
        result = dict(kind='strict_isolated_motor_construction_cohort_qualification',
            status='FAILED_SCHEMA_BINDING_OR_PARTIAL_COLLECTION_RETAINED',
            error=type(error).__name__ + ': ' + str(error),
            original_input_files_modified=False, native_motor_accuracy_verified=False,
            registered_wrapper_sha256=args.spec_sha256)
    write_exclusive(args.output, result)
    return 0 if result['status'] == 'PASS_COMPLETE_DECLARED_SCOPE' else 2


if __name__ == '__main__':
    raise SystemExit(main())
