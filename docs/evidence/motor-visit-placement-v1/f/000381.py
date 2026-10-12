"""Fixed18 per-run schema/arithmetic; no native data read during preparation."""
import json
from pathlib import Path
from construction_oracle import verify_construction
from integration_oracle import verify_scalar_advance
from stage_numeric import F32, ONE, ZERO, exact, kind
from stage_numeric import exact_snapshot, serialize_snapshot, rational
from fractions import Fraction
from physical_reference import backward_euler
from strict_io import bound, boolean, canonical, descriptor, gzip_records, integer, keys, loads, need, words, write_exclusive

DT = 0x3a83126f
H = 0x3803126f
K = 0x43960000
D = 0x41400000
CAP = 0x40000000
STATE_SCOPE = 'sorted individual native rigid bodies, collider, unique multibody, exact integration parameters and public exposed IEEE words; excludes full-set metadata, broad/narrow phase, private pipeline/solver/contact caches and observer buffers'
STUDY_ID = 'motor_visit_placement_v1'
PROFILES = {'split_1_2': (1, 2), 'early_1_0': (1, 0), 'early_3_0': (3, 0)}
CASES = [dict(label=profile + '_' + target + '_' + mode, profile=profile,
              target=target, observer=observer)
    for profile in PROFILES
    for target in ('negative', 'positive')
    for mode, observer in [('on', 'on'), ('fresh', 'on'), ('off', 'off')]]
ROW_KEYS = 'joint_handle link_id axis gain_model constraint_index solver_velocity_offset ndofs2 j_id2 jacobian_words weighted_jacobian_words rhs_word rhs_without_bias_word cfm_gain_word cfm_coefficient_word inverse_lhs_word impulse_word impulse_bound_words effort_ceiling_word'
ROW_SCALARS = 'rhs_word rhs_without_bias_word cfm_gain_word cfm_coefficient_word inverse_lhs_word impulse_word effort_ceiling_word'.split()
BUILD_KEYS = 'row coordinate_word target_position_word target_velocity_word stiffness_word damping_word duration_word inverse_duration_word limits_words effective_target_velocity_word erp_inverse_duration_word'
EVENT_KEYS = 'ccd_partition_index island_index internal_slice_index expected_internal_slice_count iteration_index phase native_scalar_bits duration_word row_cache_slice_index availability integration_multibody_handle_words generalized_velocity_words constructed_rows endpoints'
STAGE_KEYS = 'ccd_partition_index island_index internal_slice_index expected_internal_slice_count iteration_index joint_visit_index stage native_scalar_bits duration_word row_cache_slice_index generalized_velocity_words rows'
def schedules(profile):
    need(type(profile) is str and profile in PROFILES, 'unsupported fixed profile')
    pgs, stabilization = PROFILES[profile]
    companion = [('BeforeExternalForces', None), ('AfterExternalForces', None),
        ('AfterRowConstruction', None), ('BeforePositionIntegration', None),
        ('AfterPositionIntegration', None)]
    companion += [('AfterJointStabilization', i) for i in range(stabilization)]
    companion += [('FinalSlice', None)]
    stage = [(s, None, None) for s in ('BeforeExternalForces', 'AfterExternalForces',
        'AfterJointUpdate', 'AfterContactUpdate', 'AfterContactWarmstart')]
    for i in range(pgs):
        stage += [(s, i, 0 if 'Motor' in s else None) for s in
            ('BeforeJointSolve', 'BeforeMotorSolve', 'AfterMotorSolve', 'AfterJointSolve',
             'AfterContactNormal', 'AfterContactFriction')]
    stage += [('AfterIntegration', None, None)]
    for i in range(stabilization):
        stage += [(s, i, 0 if 'Motor' in s else None) for s in
            ('BeforeJointStabilization', 'BeforeMotorBiasRemoval', 'BeforeMotorSolve',
             'AfterMotorSolve', 'AfterJointStabilization', 'AfterContactStabilization')]
    if stabilization:
        stage += [('AfterStabilizationFriction', None, None)]
    stage += [('FinalSlice', None, None)]
    return companion, stage


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


def conditions(value, profile):
    pgs, stabilization = PROFILES[profile]
    expected = dict(sphere_radius_m_word=0x3dcccccd, sphere_mass_kg_word=ONE,
        outer_dt_s_word=DT, stiffness_nm_per_rad_word=K, damping_nm_s_per_rad_word=D,
        ceiling_nm_word=CAP, target_velocity_rad_s_word=0, gravity_words=[0, 0, 0],
        initial_generalized_velocity_word=0, passive_damping_word=0, armature_word=0,
        small_steps=32, pgs_passes=pgs, stabilization_passes=stabilization, contacts_enabled=False,
        sleep_enabled=False, ccd_enabled=False, identity_coincident_x_frames=True)
    same(value, expected, 'declared fixture differs')


def checkpoint(c, profile, initial=False):
    pgs, stabilization = PROFILES[profile]
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
    same(p['native_counts'], [32, 0, pgs, stabilization, 128, 1], 'integration parameter counts differ')
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


def native_visit(before, after):
    """Compare an actual supported solve pair in the declared source order."""
    left, right = before['rows'][0], after['rows'][0]
    same({k: v for k, v in left.items() if k != 'impulse_word'},
         {k: v for k, v in right.items() if k != 'impulse_word'}, 'solve changed cached motor data')
    f = F32()
    try:
        result = f.projection(left, before['generalized_velocity_words'])
    except ValueError:
        if not f.exclusions:
            raise
        return dict(status='EXCLUDED_SOURCE_FLOAT_DOMAIN', exclusions=sorted(f.exclusions),
                    operation_count=f.operation_count, mismatches=None)
    differences = {}
    if result['next_impulse_word'] != right['impulse_word']:
        differences['impulse'] = dict(expected=result['next_impulse_word'], observed=right['impulse_word'])
    if result['next_generalized_velocity_words'] != after['generalized_velocity_words']:
        differences['generalized_velocity'] = dict(expected=result['next_generalized_velocity_words'],
                                                  observed=after['generalized_velocity_words'])
    return dict(status='EXCLUDED_SOURCE_FLOAT_DOMAIN' if f.exclusions else
                'DECLARED_FORMULAS_MATCH' if not differences else 'MISMATCH',
                exclusions=sorted(f.exclusions), mismatches=differences,
                expected_impulse_word=result['next_impulse_word'],
                expected_velocity_words=result['next_generalized_velocity_words'],
                operation_count=f.operation_count)


def si_step(q, velocity, target):
    radius, mass = exact(0x3dcccccd), exact(ONE)
    inertia = Fraction(2, 5) * mass * radius * radius
    return backward_euler(inertia, exact(K), exact(D), exact(H), q, velocity, target, exact(CAP))


def si_diagnostics(build, incoming, integrated, final, reference_q, reference_v):
    operands = [build['coordinate_word'], build['target_position_word'], incoming,
                integrated['coordinate_word'], integrated['multibody_velocity_words'][0], final]
    if any(kind(w) in ('nan', 'positive_infinity', 'negative_infinity') for w in operands):
        return dict(status='UNQUALIFIED_NONFINITE_PHYSICAL_DIAGNOSTIC'), reference_q, reference_v
    target = exact(build['target_position_word'])
    conditional = si_step(exact(build['coordinate_word']), exact(incoming), target)
    independent = si_step(reference_q, reference_v, target)
    observed_q = exact(integrated['coordinate_word'])
    observed_integrated_v = exact(integrated['multibody_velocity_words'][0])
    return dict(status='REAL_SI_DIAGNOSTIC_ONLY_NO_ACCURACY_VERDICT',
        physical_inertia_kg_m2=rational(Fraction(2, 5) * exact(ONE) * exact(0x3dcccccd) ** 2),
        model='exact real values of declared f32 radius/mass/gains/targets/duration; geometric I; stationary target',
        conditional_native_incoming={k: rational(v) for k, v in conditional.items()},
        independent_reference_trajectory={k: rational(v) for k, v in independent.items()},
        native_minus_conditional_q=rational(observed_q - conditional['coordinate']),
        native_minus_independent_q=rational(observed_q - independent['coordinate']),
        native_integration_minus_independent_v=rational(observed_integrated_v - independent['velocity']),
        native_final_minus_independent_v=rational(exact(final) - independent['velocity']),
        accuracy_verified=False), independent['coordinate'], independent['velocity']


def qualify_observers(done, target_word, profile):
    comp, stage = done['construction_companion'], done['stage_v1']
    companion_schedule, stage_schedule = schedules(profile)
    pgs, stabilization = PROFILES[profile]
    telemetry_header(comp, 'schema_version source_contract native_scalar_bits numerical_outer_interval_word ccd_partition_duration_words events')
    integer(comp['schema_version'])
    need(comp['schema_version'] == 1 and comp['source_contract'] == 'InternalUnitMotorConstructionV1', 'wrong construction source contract')
    telemetry_header(stage, 'numerical_outer_interval_word native_scalar_bits ccd_partition_duration_words events')
    need(len(comp['events']) == 32 * len(companion_schedule) + 1, 'incomplete/unexpected companion schedule')
    need(len(stage['events']) == 32 * len(stage_schedule), 'incomplete/unexpected stage schedule')
    stages, visits = {}, {}
    previous_final_v = [0]
    for slice_ in range(32):
        previous = None
        cache = None
        visits[slice_] = []
        for offset, (phase, iteration, visit) in enumerate(stage_schedule):
            e = stage['events'][slice_ * len(stage_schedule) + offset]
            keys(e, STAGE_KEYS); context(e, slice_, iteration)
            nullable_index(e['joint_visit_index']); same(e['joint_visit_index'], visit, 'wrong joint visit')
            need(e['stage'] == phase, 'wrong stage order')
            uninit = phase in ('BeforeExternalForces', 'AfterExternalForces')
            same(e['row_cache_slice_index'], None if uninit else slice_, 'wrong cache generation')
            need(type(e['rows']) is list and len(e['rows']) == (0 if uninit else 1), 'wrong stage availability')
            for r in e['rows']: row(r)
            if phase == 'BeforeExternalForces':
                same(e['generalized_velocity_words'], previous_final_v, 'generic velocity chronology before external forces differs')
            if not uninit:
                current_cache = {k: v for k, v in e['rows'][0].items() if k != 'impulse_word'}
                if cache is None:
                    cache = current_cache
                    need(e['rows'][0]['impulse_word'] == 0, 'new slice did not reset motor accumulator')
                same(current_cache, cache, 'cached row mutated within slice')
            if previous is not None and phase != 'AfterExternalForces':
                if phase == 'AfterMotorSolve':
                    need(previous['stage'] == 'BeforeMotorSolve', 'solve lacks immediate before pair')
                    visits[slice_].append(dict(phase='stabilization' if offset >
                        next(i for i, x in enumerate(stage_schedule) if x[0] == 'AfterIntegration') else 'pgs',
                        iteration=iteration, visit=visit,
                        before_velocity_words=previous['generalized_velocity_words'],
                        after_velocity_words=e['generalized_velocity_words'],
                        check=native_visit(previous, e)))
                else:
                    same(previous['generalized_velocity_words'], e['generalized_velocity_words'], 'non-solve boundary changed generic V')
                    if previous['rows'] and e['rows']:
                        same(previous['rows'], e['rows'], 'non-solve boundary changed motor row')
            # Immediate enum/iteration names repeat in PGS and stabilization.
            # Ordered solve pairs retain those events; only aggregate names are keys.
            if phase not in ('BeforeMotorSolve', 'AfterMotorSolve', 'BeforeMotorBiasRemoval'):
                stages[(slice_, phase, iteration)] = e
            previous = e
        previous_final_v = previous['generalized_velocity_words']
        need(len(visits[slice_]) == pgs + stabilization, 'incomplete motor visits')
    analyses = []
    q = 0
    previous_mb_v = [0]
    reference_q, reference_v = Fraction(), Fraction()
    for slice_ in range(32):
        events = {}
        for offset, (phase, iteration) in enumerate(companion_schedule):
            e = comp['events'][slice_ * len(companion_schedule) + offset]
            keys(e, EVENT_KEYS); context(e, slice_, iteration)
            need(e['phase'] == phase, 'wrong companion phase order')
            uninit = phase in ('BeforeExternalForces', 'AfterExternalForces')
            same(e['row_cache_slice_index'], None if uninit else slice_, 'wrong companion cache')
            need(e['availability'] == ('Uninitialized' if uninit else 'Available'), 'unexpected availability')
            root = [0, 0] if phase in ('BeforePositionIntegration', 'AfterPositionIntegration') else None
            same(e['integration_multibody_handle_words'], root, 'wrong integration root')
            if root is not None: words(e['integration_multibody_handle_words'], 2)
            need(type(e['endpoints']) is list and len(e['endpoints']) == (0 if uninit else 1), 'missing/extra endpoint')
            for item in e['endpoints']: endpoint(item)
            need(type(e['constructed_rows']) is list and len(e['constructed_rows']) == (1 if phase == 'AfterRowConstruction' else 0), 'missing/extra constructor')
            mapping = {'AfterRowConstruction': 'AfterJointUpdate', 'BeforePositionIntegration': 'AfterContactFriction', 'AfterPositionIntegration': 'AfterIntegration'}
            stage_iteration = pgs - 1 if phase == 'BeforePositionIntegration' else iteration
            v1 = stages[(slice_, mapping.get(phase, phase), stage_iteration)]
            same(e['generalized_velocity_words'], v1['generalized_velocity_words'], 'companion/stage generic V differs')
            same([x['row'] for x in e['endpoints']], v1['rows'], 'companion/stage row differs')
            events[(phase, iteration)] = e
        build = events[('AfterRowConstruction', None)]['constructed_rows'][0]
        keys(build, BUILD_KEYS); row(build['row'])
        for name in BUILD_KEYS.split():
            if name not in ('row', 'limits_words'): integer(build[name], 32)
        need(build['limits_words'] is None, 'unexpected limits')
        need((build['target_position_word'], build['target_velocity_word'], build['stiffness_word'], build['damping_word'], build['duration_word']) == (target_word, 0, K, D, H), 'actual constructor inputs differ')
        ep_build = events[('AfterRowConstruction', None)]['endpoints'][0]
        same(build['row'], ep_build['row'], 'constructor/initialized row differs')
        same(ep_build['multibody_velocity_words'], previous_mb_v, 'stored MB velocity changed before integration copy')
        need(build['coordinate_word'] == q and ep_build['coordinate_word'] == q, 'constructor q chronology differs')
        weighted = build['row']['weighted_jacobian_words'][0]
        diag = F32()
        diagonal_result = dict(status='DECLARED_FORMULAS_MATCH', exclusions=[], mismatches={})
        try:
            inverse = diag.inverse(weighted)
            diagonal_result['exclusions'] = sorted(diag.exclusions)
            diagonal_result['status'] = 'EXCLUDED_SOURCE_FLOAT_DOMAIN' if diag.exclusions else 'DECLARED_FORMULAS_MATCH'
            if inverse != build['row']['inverse_lhs_word']:
                diagonal_result['mismatches'] = dict(expected=inverse, observed=build['row']['inverse_lhs_word'])
                if not diag.exclusions: diagonal_result['status'] = 'MISMATCH'
        except ValueError:
            if not diag.exclusions: raise
            diagonal_result = dict(status='EXCLUDED_SOURCE_FLOAT_DOMAIN', exclusions=sorted(diag.exclusions), mismatches=None)
        construction = verify_construction(build)
        before = events[('BeforePositionIntegration', None)]['endpoints'][0]
        after = events[('AfterPositionIntegration', None)]['endpoints'][0]
        need(before['coordinate_word'] == q, 'wrong integration-input q')
        integration = verify_scalar_advance(H, before, after, events[('BeforePositionIntegration', None)]['generalized_velocity_words'])
        same(events[('BeforePositionIntegration', None)]['generalized_velocity_words'], events[('AfterPositionIntegration', None)]['generalized_velocity_words'], 'integration changed generic V')
        q = after['coordinate_word']
        previous_mb_v = after['multibody_velocity_words']
        for phase, iteration in [('AfterPositionIntegration', None)] + [('AfterJointStabilization', i) for i in range(stabilization)] + [('FinalSlice', None)]:
            ep = events[(phase, iteration)]['endpoints'][0]
            need(ep['coordinate_word'] == q, 'postintegration coordinate mutated')
            same(ep['multibody_velocity_words'], previous_mb_v, 'MB velocity changed before writeback')
        first_v = visits[slice_][0]['before_velocity_words'][0]
        final_v = events[('FinalSlice', None)]['generalized_velocity_words'][0]
        physical, reference_q, reference_v = si_diagnostics(build, first_v, after, final_v, reference_q, reference_v)
        try:
            row_algebra = dict(status='CONDITIONAL_STORED_OPERANDS_REFERENCE',
                snapshot=serialize_snapshot(exact_snapshot(before['row'], events[('BeforePositionIntegration', None)]['generalized_velocity_words'])),
                physical_inertia_validated=False)
        except ValueError as error:
            row_algebra = dict(status='UNQUALIFIED_CONDITIONAL_ROW_REFERENCE', error=str(error))
        analyses.append(dict(slice=slice_, construction=construction, integration=integration,
            initialized_diagonal=diagonal_result, native_visits=visits[slice_],
            row_algebra=row_algebra, physical_SI=physical,
            q_before_rad_word=before['coordinate_word'], q_after_rad_word=after['coordinate_word'],
            integration_input_velocity_rad_s_word=before['multibody_velocity_words'][0],
            stabilization_generic_velocity_rad_s_words=[events[('AfterJointStabilization', i)]['generalized_velocity_words'][0] for i in range(stabilization)],
            final_slice_generic_velocity_rad_s_word=final_v,
            first_row=build['row'], first_constructor_inputs={k:v for k,v in build.items() if k != 'row'},
            first_incoming_velocity_word=first_v,
            unmeasured_computations=['external acceleration input computation', 'mass matrix and LU construction']))
    final = comp['events'][-1]; keys(final, EVENT_KEYS); context(final, 31, None)
    need(final['phase'] == 'AfterVelocityWriteback' and final['availability'] == 'Available', 'missing final writeback')
    same(final['row_cache_slice_index'], 31, 'wrong writeback cache')
    need(final['integration_multibody_handle_words'] is None and final['constructed_rows'] == [], 'bad writeback shape')
    need(type(final['endpoints']) is list and len(final['endpoints']) == 1, 'bad writeback endpoint')
    endpoint(final['endpoints'][0])
    last = comp['events'][-2]
    same(final['generalized_velocity_words'], last['generalized_velocity_words'], 'writeback changed generic V')
    same(final['endpoints'][0]['row'], last['endpoints'][0]['row'], 'writeback changed row')
    same(final['endpoints'][0]['multibody_velocity_words'], final['generalized_velocity_words'], 'final MB/generic copy differs')
    mb = done['checkpoint']['public_multibody_words'][0]
    same(mb['generalized_velocity_words'], final['generalized_velocity_words'], 'completed MB V differs')
    need(final['endpoints'][0]['coordinate_word'] == q == mb['joints'][1]['coordinates_words'][3], 'completed q differs')
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
    keys(start, 'kind schema_version study_id profile run_label target target_position_rad_word observer_enabled expected_outer_intervals native_scalar_bits state_codec state_scope declared_conditions')
    need(start['study_id'] == STUDY_ID and start['profile'] == case['profile'], 'study/profile header differs')
    target = DT | (0x80000000 if case['target'] == 'negative' else 0)
    need(start['kind'] == 'run_start' and start['run_label'] == case['label'] and start['target'] == case['target'], 'run identity differs')
    for key in ('schema_version', 'target_position_rad_word', 'expected_outer_intervals', 'native_scalar_bits'): integer(start[key])
    boolean(start['observer_enabled'])
    need((start['schema_version'], start['target_position_rad_word'], start['expected_outer_intervals'], start['native_scalar_bits']) == (1, target, 1, 32), 'run header differs')
    need(start['observer_enabled'] is (case['observer'] == 'on') and start['state_codec'] == 'postcard-1.1.3-raw-exact-consumption' and start['state_scope'] == STATE_SCOPE, 'observer/codec/scope differs')
    conditions(start['declared_conditions'], case['profile'])
    keys(initial, 'kind schema_version checkpoint'); integer(initial['schema_version'])
    need(initial['kind'] == 'initial_state' and initial['schema_version'] == 1, 'bad initial record')
    checkpoint(initial['checkpoint'], case['profile'], initial=True)
    keys(done, 'kind schema_version completed_outer_interval checkpoint construction_companion stage_v1 impulse_v1_ieee_words')
    integer(done['schema_version']); integer(done['completed_outer_interval'])
    need(done['kind'] == 'completed_interval' and (done['schema_version'], done['completed_outer_interval']) == (1, 1), 'bad completed record')
    checkpoint(done['checkpoint'], case['profile'])
    analyses = []
    if case['observer'] == 'on': analyses = qualify_observers(done, target, case['profile'])
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

