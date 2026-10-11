#!/usr/bin/env python3
"""Read retained native measurements once; never construct a physics world."""
import argparse
import hashlib
import sys
import importlib.util
import itertools
import json
import math
from pathlib import Path
import struct
from datetime import datetime, timezone

ROOT = Path('/workspace/.rne-kick-demo/single-joint-motor-calibration')
REPO = Path('/workspace/RobotNativeEngine')
HERE = ROOT / 'diagonal-intervention-generation-3/independent-oracle'
PROTOCOL = HERE / 'preregistered-diagonal-intervention-protocol.json'
TOOL_FREEZE = HERE / 'pre-world-independent-tool-freeze.json'
REPORT = None
EXPECTED_REPORT_SHA = None
BINARY_BINDING = None
EXPECTED_BINDING_SHA = None
OUTPUT = None
EXPECTED_LIB_SHA = 'e36a14f545bcdbb64c5a43cbc9c1bb4cc089fc4487dfef039ad6e43191479df2'
BASE_COMMIT = 'b0bf79eaf765596fede4130111596ac1d721262e'
UNIT_PATH = 'third_party/rapier3d/src/dynamics/joint/multibody_joint/unit_multibody_joint.rs'
ORIGINAL_UNIT_SHA = '0e8eaa25f2ed772fc7de1e15b084c0761700d9f2afb5eeb17b73247af51246fe'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def ref(path):
    return {'path': str(path), 'bytes': path.stat().st_size, 'sha256': sha(path)}


def require(condition, text):
    if not condition:
        raise ValueError(text)


def bits64(x):
    return struct.unpack('<Q', struct.pack('<d', x))[0]


def value(s):
    return struct.unpack('<d', struct.pack('<Q', s['ieee754_f64_bits']))[0]


def finite_scalar(s):
    require(set(s) == {'finite', 'ieee754_f64_bits', 'native_f32_bits'}, 'Scalar key/type/presence')
    b = s['ieee754_f64_bits']
    require(type(b) is int and 0 <= b < (1 << 64), 'Scalar f64 bits integer')
    v = value(s)
    require(math.isfinite(v), 'Retained nonfinite scalar; not an available finite observation')
    require(type(s['finite']) is float and bits64(s['finite']) == b, 'Finite presentation/IEEE mismatch')
    n = s['native_f32_bits']
    if n is not None:
        require(type(n) is int and 0 <= n < (1 << 32), 'Native f32 bits integer')
        native = struct.unpack('<f', struct.pack('<I', n))[0]
        require(bits64(float(native)) == b, 'Native bits/promotion mismatch')
    return v


def walk_scalars(tree):
    if isinstance(tree, dict):
        if set(tree) == {'finite', 'ieee754_f64_bits', 'native_f32_bits'}:
            finite_scalar(tree)
            return 1
        return sum(walk_scalars(v) for v in tree.values())
    if isinstance(tree, list):
        return sum(walk_scalars(v) for v in tree)
    return 0


def exact_tree(a, b, path='$'):
    require(type(a) is type(b), path + ' type differs')
    if isinstance(a, dict):
        require(set(a) == set(b), path + ' key/presence differs')
        for k in a:
            exact_tree(a[k], b[k], path + '.' + k)
    elif isinstance(a, list):
        require(len(a) == len(b), path + ' length differs')
        for i, (x, y) in enumerate(zip(a, b)):
            exact_tree(x, y, path + f'[{i}]')
    elif isinstance(a, float):
        require(bits64(a) == bits64(b), path + ' IEEE float differs')
    else:
        require(a == b, path + ' value differs')


def state_words(state):
    entities = state['native_entities']
    require(len(entities) == 3, 'Three known native entities required')
    require([e['kind'] for e in entities] == ['rigid_body', 'rigid_body', 'multibody'], 'State entity type/order')
    require([e['handle'] for e in entities] == [[0, 0], [1, 0], [1, 0]], 'State actual stable handle identity/order')
    words = [2, len(entities)]
    total_bytes = 0
    for e in entities:
        require(all(type(b) is int and 0 <= b <= 255 for b in e['bytes']), 'Native serialized byte type')
        data = bytes(e['bytes'])
        require(hashlib.sha256(data).hexdigest() == e['sha256'], 'Native entity SHA mismatch')
        total_bytes += len(data)
        words += [int(e['kind'] == 'multibody'), *e['handle'], len(data)]
        for i in range(0, len(data), 8):
            words.append(int.from_bytes(data[i:i+8].ljust(8, b'\0'), 'little'))
    exposed = state['exposed_state_f32_bits']
    require(len(exposed) == 63, 'Declared native two-body/one-DOF exposed layout')
    require(all(type(x) is int and 0 <= x < (1 << 32) for x in exposed), 'Exposed native float bits')
    require(all(math.isfinite(struct.unpack('<f', struct.pack('<I', b))[0]) for b in exposed), 'Finite exposed actual state')
    words += [len(exposed), *exposed]
    require(all(type(x) is int and 0 <= x < (1 << 64) for x in state['canonical_words']), 'State canonical word integer/type')
    require(words == state['canonical_words'], 'Native serialized state/canonical words differ')
    require(exposed[38] == state['generalized_velocity_rad_s']['native_f32_bits'], 'Exposed actual qdot differs')
    require(exposed[54] == state['actual_serialized_coordinate_rad']['native_f32_bits'], 'Exposed actual child AngX coordinate differs')
    return total_bytes, len(words)


def motor_words(m):
    require(m['kind'] == 'revolute', 'Actual qualified motor must be revolute')
    out = [0, m['applied_impulse_nm_s']['ieee754_f64_bits']]  # source: int(!is_revolute)
    for k in ['effective_effort_ceiling_nm', 'rounded_impulse_bound_nm_s']:
        s = m[k]
        out.append(int(s is not None))
        if s is not None:
            out.append(s['ieee754_f64_bits'])
    return out


def telemetry_words(t):
    out = [1, t['numerical_outer_interval_s']['ieee754_f64_bits'], len(t['ccd_partition_durations_s'])]
    out += [s['ieee754_f64_bits'] for s in t['ccd_partition_durations_s']]
    out.append(len(t['motors']))
    for m in t['motors']:
        out += [*m['joint_handle'], m['link_id'], m['joint_dof_id'], m['axis']]
        out += motor_words(m['impulse'])
        out += [m['observed_solver_interval_s']['ieee754_f64_bits'], m['observed_solver_slice_count'],
                int(m['full_step_coverage']), m['observed_partition_count'],
                m['finite_positive_bound_eligible_slice_count'], m['finite_positive_bound_eligible_duration_s']['ieee754_f64_bits'],
                m['finite_positive_bound_active_slice_count'], m['finite_positive_bound_active_duration_s']['ieee754_f64_bits'], len(m['slices'])]
        for s in m['slices']:
            out += [s['ccd_partition_index'], s['internal_slice_index'], s['expected_internal_slice_count'],
                    s['duration_s']['ieee754_f64_bits'], int(s['finite_positive_bound_eligible']), int(s['finite_positive_bound_active'])]
            out += motor_words(s['impulse'])
    require(all(type(x) is int and 0 <= x < (1 << 64) for x in t['canonical_words']), 'Telemetry canonical word integer/type')
    require(out == t['canonical_words'], 'Telemetry canonical words/type/presence/order differs')
    return len(out)


def bit_equal_scalar(s, expected, message):
    require(s['ieee754_f64_bits'] == bits64(expected), message)


def close(actual, expected):
    return abs(actual - expected) <= max(1e-8, 2e-5 * abs(expected))


def source_bindings():
    require(sha(BINARY_BINDING) == EXPECTED_BINDING_SHA, 'Explicit binary binding digest differs')
    binding = json.loads(BINARY_BINDING.read_text())
    require(binding['measurement_generation'] == 3 and binding['generation_3_native_worlds_executed_before_freeze'] == 0, 'New generation3 binary freeze must precede all matrix worlds')
    frozen_path = Path(binding['source_compiler_protocol']['path'])
    require(sha(frozen_path) == binding['source_compiler_protocol']['sha256'], 'Freeze receipt binding')
    frozen = json.loads(frozen_path.read_text())
    require(frozen['measurement_generation'] == 3 and frozen['generation_3_native_worlds_executed_before_freeze'] == 0, 'New generation3 source freeze must precede all matrix worlds')
    require(frozen['base_commit'] == BASE_COMMIT, 'Expected approved merged base')
    require(len(frozen['files']) == binding['selected_source_count'] == 118, 'Bounded118 selected source count')
    require(frozen['files']['examples/141_position_motor_calibration/lib.rs'] == EXPECTED_LIB_SHA, 'Unchanged original codec2 physical harness and fixed-point reference')
    for name, digest in frozen['files'].items():
        require(sha(REPO / name) == digest, 'Current frozen source mismatch: ' + name)
    unit = (REPO / UNIT_PATH).read_bytes()
    new = b'inv_lhs: if motor.model == crate::dynamics::MotorModel::ForceBased {\r\n            crate::utils::inv(lhs + motor_params.cfm_gain)\r\n        } else {\r\n            crate::utils::inv(lhs)\r\n        },'
    old = b'inv_lhs: crate::utils::inv(lhs),'
    require(unit.count(new) == 1, 'Exactly declared candidate ForceBased inverse-diagonal branch')
    require(hashlib.sha256(unit.replace(new, old, 1)).hexdigest() == ORIGINAL_UNIT_SHA, 'Only declared motor diagonal intervention in unit file; AccelerationBased and limit arithmetic unchanged')
    require(sha(Path(binding['binary_path'])) == binding['binary_sha256'], 'Frozen actual candidate binary hash')
    require(sha(REPORT) == EXPECTED_REPORT_SHA, 'Explicit authoritative new report SHA')
    protocol = json.loads(PROTOCOL.read_text())
    require(binding['predeclared_protocol_sha256'] == frozen['predeclared_protocol_sha256'] == sha(PROTOCOL), 'Protocol binding')
    require(binding['independent_oracle_sha256'] == frozen['independent_oracle_sha256'] == sha(HERE / 'frozen_candidate_row_oracle.py'), 'Independent candidate oracle binding')
    require(binding['independent_auditor_sha256'] == frozen['independent_auditor_sha256'] == sha(Path(__file__)), 'Independent pre-world auditor binding')
    require(binding['pre_world_independent_tool_freeze_sha256'] == frozen['pre_world_independent_tool_freeze_sha256'] == sha(TOOL_FREEZE), 'Independent pre-world tool receipt binding')
    tool_freeze = json.loads(TOOL_FREEZE.read_text())
    require(tool_freeze['generation_3_native_worlds_executed_at_tool_freeze'] == 0, 'Tool freeze must precede worlds')
    for entry in tool_freeze['frozen_tools']:
        require(sha(Path(entry['path'])) == entry['sha256'], 'Frozen tool artifact changed')
    for entry in protocol['immutable_prior_bindings']:
        require(sha(Path(entry['path'])) == entry['sha256'], 'Immutable generation1/2/proposal evidence changed')
    return {'binary': ref(Path(binding['binary_path'])), 'binding_receipt': ref(BINARY_BINDING),
            'source_compiler_protocol': ref(frozen_path), 'selected_source_count': len(frozen['files']),
            'selected_source_sha256': frozen['files'], 'protocol': ref(PROTOCOL), 'raw_report': ref(REPORT),
            'independent_oracle': ref(HERE / 'frozen_candidate_row_oracle.py'), 'independent_auditor': ref(Path(__file__)),
            'pre_world_tool_freeze': ref(TOOL_FREEZE), 'immutable_prior_bindings': protocol['immutable_prior_bindings']}


def audit():
    before = source_bindings()
    module_spec = importlib.util.spec_from_file_location('frozen_row_independent', HERE / 'frozen_candidate_row_oracle.py')
    oracle = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(oracle)
    report = json.loads(REPORT.read_text())
    historical = json.loads((ROOT / 'codec-migration-generation-2/raw-report.json').read_text())
    exact_tree(report['protocol'], historical['protocol'], 'Unchanged physical/report protocol across diagonal intervention')
    scalar_count = walk_scalars(report)
    for name, digest in report['selected_source_sha256'].items():
        require(name in before['selected_source_sha256'] and sha(REPO / name) == digest, 'Report compiled selected source: ' + name)
    require(report['schema_version'] == 2 and len(report['cases']) == 24, 'Complete fixed cells retained')
    expected_cells = list(itertools.product(['G1Posture', 'G1Ankle'], [-.001, 0.0, .001], [1, 32], [1, 2]))
    results, primary = [], []
    counts = {'worlds': 0, 'state_snapshots': 0, 'native_entity_snapshots': 0, 'native_serialized_state_bytes': 0,
              'state_canonical_words': 0, 'telemetry_worlds': 0, 'telemetry_canonical_words': 0, 'ordered_slices': 0,
              'coefficient_native_oracle_matches': 0, 'complete_coverage_worlds': 0, 'zero_controls': 0, 'zero_control_passes': 0}
    for index, (case, expected_cell) in enumerate(zip(report['cases'], expected_cells)):
        gain, target, n, pgs = expected_cell
        cell = case['cell'];target_bits = oracle.bits_from_decimal(target)
        require(cell['gain'] == gain and cell['target_position_rad']['native_f32_bits'] == target_bits and
                cell['solver_small_steps'] == n and cell['internal_pgs_passes'] == pgs, 'Predeclared cell/order')
        require(len(case['runs']) == 3, 'ON2/OFF1 attempts retained')
        r0, r1, off = case['runs']
        require([r['collector_enabled'] for r in case['runs']] == [True, True, False], 'ON2/OFF1 order')
        for run in case['runs']:
            counts['worlds'] += 1
            require(run['world_created'] is True and run['completed_step'] is True and run['error'] is None,
                    'Completed actual world/error retention differs')
            require(run['initial'] is not None and run['final_state'] is not None, 'Available initial/final retained')
            for stage in ['initial', 'final_state']:
                total_bytes, total_words = state_words(run[stage])
                counts['state_snapshots'] += 1;counts['native_entity_snapshots'] += 3
                counts['native_serialized_state_bytes'] += total_bytes;counts['state_canonical_words'] += total_words
            initial, final = run['initial'], run['final_state']
            require(len(initial['initial_fixture_checks']) == 7 and all(x is True for x in initial['initial_fixture_checks'].values()), 'Actual fixture checks')
            require(initial['post_step_lu_inverse_inertia_kg_m2_inv'] is None and final['initial_fixture_checks'] is None, 'Capture timing/presence')
            inertia = finite_scalar(initial['native_body_inertia_x_kg_m2'])
            w_bits = oracle.divide(oracle.ONE, initial['native_body_inertia_x_kg_m2']['native_f32_bits'])
            require(close(inertia, .004), 'Actual pre effective Ixx physical sphere check')
            require(close(value(final['post_step_lu_inverse_inertia_kg_m2_inv']), oracle.scalar(w_bits)), 'Separate post-LU consistency')
            require(value(initial['generalized_velocity_rad_s']) == 0 and value(initial['actual_serialized_coordinate_rad']) == 0, 'Initial actual q/v')
            if run['collector_enabled']:
                t = run['telemetry'];require(t is not None and len(t['motors']) == 1, 'Available unique motor telemetry')
                counts['telemetry_worlds'] += 1;counts['telemetry_canonical_words'] += telemetry_words(t)
                m = t['motors'][0]
                require(m['joint_handle'] == [1, 0] and m['link_id'] == 1 and m['joint_dof_id'] == 0 and m['axis'] == 3, 'Actual motor identity')
                h_bits = oracle.divide(oracle.bits_from_decimal(.001), oracle.bits_from_decimal(n))
                h = oracle.scalar(h_bits);b = oracle.scalar(oracle.multiply(oracle.bits_from_decimal(2), h_bits))
                bit_equal_scalar(t['numerical_outer_interval_s'], oracle.scalar(oracle.bits_from_decimal(.001)), 'Actual numerical outer dt')
                require(len(t['ccd_partition_durations_s']) == 1, 'No extra CCD partitions')
                bit_equal_scalar(t['ccd_partition_durations_s'][0], oracle.scalar(oracle.bits_from_decimal(.001)), 'Actual sole partition dt')
                require(m['full_step_coverage'] is True and m['observed_solver_slice_count'] == n and m['observed_partition_count'] == 1 and len(m['slices']) == n, 'Integer exact full coverage')
                sum_h=0.;sum_p=None;active=0;active_h=0.;absolute=0.;previous=0;reversals=0
                for ordinal, s in enumerate(m['slices']):
                    counts['ordered_slices'] += 1
                    require((s['ccd_partition_index'],s['internal_slice_index'],s['expected_internal_slice_count']) == (0,ordinal,n), 'Actual exact-once slice ordinals')
                    bit_equal_scalar(s['duration_s'], h, 'Actual native smallstep duration')
                    require(s['impulse']['kind'] == 'revolute', 'Actual slice motor type')
                    bit_equal_scalar(s['impulse']['effective_effort_ceiling_nm'], 2., 'Actual native effort ceiling')
                    bit_equal_scalar(s['impulse']['rounded_impulse_bound_nm_s'], b, 'Actual rounded bound')
                    impulse = finite_scalar(s['impulse']['applied_impulse_nm_s'])
                    require(oracle.scalar(oracle.bits_from_decimal(impulse)) == impulse and abs(impulse) <= b, 'Native promoted impulse/bounds')
                    at_bound=bits64(abs(impulse))==bits64(b)
                    require(s['finite_positive_bound_eligible'] is True and s['finite_positive_bound_active'] is at_bound, 'Actual final-bound flag')
                    sum_h += h;sum_p=impulse if sum_p is None else sum_p+impulse;absolute += abs(impulse)
                    if at_bound:active += 1;active_h += h
                    if impulse:
                        sign=-1 if impulse<0 else 1
                        if previous and sign != previous:reversals += 1
                        previous=sign
                for s, expected in [(m['observed_solver_interval_s'],sum_h),(m['impulse']['applied_impulse_nm_s'],sum_p),
                                    (m['finite_positive_bound_eligible_duration_s'],sum_h),(m['finite_positive_bound_active_duration_s'],active_h)]:
                    bit_equal_scalar(s,expected,'Actual ordered aggregate exact IEEE value')
                require(m['finite_positive_bound_eligible_slice_count'] == n and m['finite_positive_bound_active_slice_count'] == active, 'Actual aggregate counts')
                bit_equal_scalar(m['impulse']['effective_effort_ceiling_nm'], 2., 'Aggregate ceiling')
                bit_equal_scalar(m['impulse']['rounded_impulse_bound_nm_s'], b, 'Aggregate bound')
                counts['complete_coverage_worlds'] += 1
                if run is r0:
                    a=case['assessment']
                    bit_equal_scalar(a['signed_total_impulse_nm_s'],sum_p,'Descriptive signed sum')
                    bit_equal_scalar(a['sum_absolute_slice_impulses_nm_s'],absolute,'Descriptive absolute sum')
                    require(a['nonzero_slice_sign_reversals']==reversals,'Descriptive sign reversals')
                    if absolute:
                        bit_equal_scalar(a['cancellation_ratio'],1-abs(sum_p)/absolute,'Descriptive cancellation')
                    else:require(a['cancellation_ratio'] is None,'All-zero cancellation unavailable')
                    bit_equal_scalar(a['final_bound_active_duration_fraction'],active_h/sum_h,'Descriptive final-bound duration fraction')
            else:require(run['telemetry'] is None, 'Disabled observation must be absent')
        for stage in ['initial', 'final_state']:
            exact_tree(r0[stage],r1[stage],f'case{index}.ON_fresh.{stage}')
            exact_tree(r0[stage],off[stage],f'case{index}.ON_OFF.{stage}')
        exact_tree(r0['telemetry'],r1['telemetry'],f'case{index}.ON_fresh.telemetry')
        require(all(v is True for v in case['replay'].values()) and set(case['replay']) == {'native_state_exact_fresh_repeat','native_state_exact_on_off','telemetry_exact_fresh_repeat','disabled_telemetry_absent'},'Replay stored flags match literal actual arrays/bytes')
        initial,final=r0['initial'],r0['final_state'];s=r0['telemetry']['motors'][0]['slices'][0];stored=case['assessment']['first_slice_oracle']
        k,d=(300.,12.) if gain=='G1Posture' else (250.,8.)
        row={'outer_interval_bits_u32':oracle.bits_from_decimal(.001),'small_step_count':n,
             'stiffness_bits_u32':oracle.bits_from_decimal(k),'damping_bits_u32':oracle.bits_from_decimal(d),
             'pre_effective_inertia_xx_bits_u32':initial['native_body_inertia_x_kg_m2']['native_f32_bits'],
             'initial_coordinate_bits_u32':initial['actual_serialized_coordinate_rad']['native_f32_bits'],
             'initial_velocity_bits_u32':initial['generalized_velocity_rad_s']['native_f32_bits'],
             'target_coordinate_bits_u32':target_bits,'target_velocity_bits_u32':0,'effort_cap_bits_u32':oracle.bits_from_decimal(2.),
             'post_step_inverse_mass_bits_u32':final['post_step_lu_inverse_inertia_kg_m2_inv']['native_f32_bits'],
             'actual_first_applied_impulse_nm_s':value(s['impulse']['applied_impulse_nm_s'])}
        independently=oracle.evaluate(row);expr=independently['native_f32_expression']
        mapping={'actual_slice_h_s':'small_step_duration','native_api_a_s_inv':'erp_inv_dt','native_api_cfm_coeff':'cfm_coeff','native_api_g_kg_m2_inv':'cfm_gain',
                 'independently_rounded_a_s_inv':'erp_inv_dt','independently_rounded_g_kg_m2_inv':'cfm_gain','independently_rounded_rhs_rad_s':'native_rhs_wo_bias',
                 'pre_body_derived_inverse_inertia_kg_m2_inv':'fixture_derived_inverse_mass','independently_rounded_bound_nm_s':'native_impulse_bound'}
        for reported, derived in mapping.items():
            require(stored[reported]['native_f32_bits']==expr[derived]['bits_u32'],'Exact Fraction/native scalar operation bits: '+reported)
        counts['coefficient_native_oracle_matches'] += 1
        ref_value=independently['reference'];comparison=independently['actual_first_slice_comparison']
        ieee_reference = ref_value['applied_impulse_nm_s']
        if ieee_reference == 0.0:
            # Fraction has no signed zero. Derive only its IEEE presentation sign
            # from frozen native operands and the source leading negation.
            rhs_value=expr['native_rhs_wo_bias']['promoted_f64']
            w_value=expr['fixture_derived_inverse_mass']['promoted_f64']
            g_value=expr['cfm_gain']['promoted_f64']
            ieee_reference=-(rhs_value+value(initial['generalized_velocity_rad_s']))/(w_value+g_value)
            require(ieee_reference == 0.0,'Zero-sign presentation cannot change mathematical value')
        for reported, expected in [('expected_applied_impulse_nm_s',ieee_reference),('actual_applied_impulse_nm_s',row['actual_first_applied_impulse_nm_s']),
                                   ('error_nm_s',comparison['absolute_error_nm_s']),('tolerance_nm_s',ref_value['tolerance_nm_s']),('fixed_point_residual_rad_s',comparison['frozen_row_residual_rad_s'])]:
            bit_equal_scalar(stored[reported],expected,'Independent frozen-row f64 reference/error/tolerance/residual: '+reported)
        bit_equal_scalar(stored['actual_native_bound_nm_s'],expr['native_impulse_bound']['promoted_f64'],'Actual native bound independent check')
        require(stored['native_and_independent_coefficients_bits_match'] is True and stored['native_bound_matches_independent_rounding'] is True,'Recorded coefficient/bound matching flags')
        actual_pass=comparison['within_declared_tolerance']
        require(case['assessment']['first_slice_tolerance_pass'] is actual_pass,'Retained actual numerical tolerance verdict differs')
        is_primary=n==32 and pgs==1 and target!=0
        require(case['assessment']['primary_cell'] is is_primary,'Primary predeclared membership')
        for flag in ['initial_fixture_qualification','finite_exposed_state','full_actual_slice_coverage','pre_body_inertia_matches_nominal_sphere','post_lu_matches_pre_body_derived_inverse']:
            require(case['assessment'][flag] is True,'Retained qualified fixture/finite/coverage/cache flag: '+flag)
        bit_equal_scalar(case['assessment']['final_kinetic_energy_j'],.5*value(initial['native_body_inertia_x_kg_m2'])*value(final['generalized_velocity_rad_s'])*value(final['generalized_velocity_rad_s']),'Actual final generalized kinetic energy')
        if target==0:
            counts['zero_controls'] += 1
            require(case['assessment']['zero_target_motionless'] is True,'Retained zero control actual verdict')
            require(all(struct.unpack('<f',struct.pack('<I',a))[0]==struct.unpack('<f',struct.pack('<I',b))[0] for a,b in zip(initial['exposed_state_f32_bits'],final['exposed_state_f32_bits'])),'Actual zero-control exposed motion')
            require(all(value(sl['impulse']['applied_impulse_nm_s'])==0 for sl in r0['telemetry']['motors'][0]['slices']),'Actual zero-control motor impulse')
            counts['zero_control_passes'] += 1
        else:require(case['assessment']['zero_target_motionless'] is None,'Nonzero zero-control unavailable')
        result={'case_index':index,'gain':gain,'target_position_rad':value(cell['target_position_rad']),'small_steps':n,'PGS_passes':pgs,
                'actual_native_input':row,'independent_exact_Fraction_oracle':independently,
                'numerical_tolerance_pass':actual_pass,'primary_member':is_primary,
                'source_derived_fixed_point_IEEE_presentation':{'value':ieee_reference,'bits_u64':bits64(ieee_reference),'scope':'Fraction mathematical zero is signless; only zero IEEE sign separately derives from source leading minus on frozen operands'},
                'initial_native_Ixx_kg_m2':value(initial['native_body_inertia_x_kg_m2']),
                'post_step_native_inverse_mass_kg_m2_inv':value(final['post_step_lu_inverse_inertia_kg_m2_inv']),
                'coverage_and_literal_replay_qualification':True}
        results.append(result)
        if is_primary:primary.append(result)
    require(counts['worlds']==72 and counts['telemetry_worlds']==48 and len(primary)==4,'Actual final matrix counts')
    require(counts['zero_controls']==counts['zero_control_passes']==8,'Eight predeclared zero controls')
    summary={'collection_complete':True,'worlds_attempted':72,'worlds_created':72,'completed_worlds':72,'replay_qualification_pass':True,
             'primary_calibration_pass':all(x['numerical_tolerance_pass'] for x in primary),'primary_nonzero_cells':4,
             'primary_nonzero_passes':sum(x['numerical_tolerance_pass'] for x in primary),'zero_target_cells':8,'zero_target_passes':8}
    exact_tree(summary,report['summary'],'actual independently recalculated summary')
    after=source_bindings();exact_tree(before,after,'Read-only allsource/data/binary bindings before-after')
    output={'status':'PASS_INDEPENDENT_GENERATION3_DATA_REPLAY_AND_ORACLE_CALCULATION_QUALIFICATION','created_utc':datetime.now(timezone.utc).isoformat(),
            'primary_numerical_outcome':{'status':'PASS' if summary['primary_calibration_pass'] else 'FAIL','passed':summary['primary_nonzero_passes'],'required':4,'zero_controls_passed':8,'zero_controls_required':8,
              'scope':'Source-bound isolated ForceBased diagonal intervention; outcome read from this new report only, no G1/hardware/recovery conclusion'},
            'actual_summary_independently_recomputed':summary,'counts':dict(counts,validated_scalar_wrappers=scalar_count),
            'primary_values':[{'case_index':x['case_index'],'gain':x['gain'],'target_position_rad':x['target_position_rad'],
                 'actual_first_impulse_nm_s':x['actual_native_input']['actual_first_applied_impulse_nm_s'],
                 'fixed_point_impulse_nm_s':x['independent_exact_Fraction_oracle']['reference']['applied_impulse_nm_s'],
                 'absolute_error_nm_s':x['independent_exact_Fraction_oracle']['actual_first_slice_comparison']['absolute_error_nm_s'],
                 'declared_tolerance_nm_s':x['independent_exact_Fraction_oracle']['reference']['tolerance_nm_s'],
                 'numerical_tolerance_pass':x['numerical_tolerance_pass']} for x in primary],
            'all24retained_case_calculations':results,'before_after_bindings':before,'current_auditor_tool':ref(Path(__file__)),'pre_world_auditor_tool_freeze':ref(TOOL_FREEZE),
            'coverage_scope':'48 actual ON reports, every ordered qualified native slice verified by identities, integer ordinal/count, duration/bound bits and independently accumulated public aggregates',
            'replay_scope':'Actual retained source-bound individually serialized native body/multibody bytes and complete exposed/canonical state ON/fresh and ON/OFF; native telemetry IEEE/type/key/presence/order ON/fresh. Not all private pipeline/cache state.',
            'serialized_state_decode_scope':'Entity bytes/hash and independent canonical words reconstructed; scalar exposed q/qdot consistency checked. Rust zero-world byte-deserialize/roundtrip verification is separate root receipt; this Python auditor does not independently deserialize all postcard structs.',
            'candidate_diagonal_scope':'Exact Fraction source-derived fl32(w+g) and guarded reciprocal expected from frozen candidate source; no direct native inv_lhs telemetry or within-pass iterate measurement',
            'not_evaluated':['Hardware accuracy, real kick recovery, cause of G1 falls', 'AccelerationBased baseline/candidate worlds or unchanged-path state compatibility','Unrecorded within-pass motor iterates or direct first-slice native LU','Selection/adoption of new gains/tau/numerical solver settings'],
            'auditor_native_worlds_builds_tests_git_repo_edits':0,'owned_processes_remaining_after_script_completion':0}
    out=OUTPUT;require(not out.exists(),'Preserve existing final audit')
    out.write_text(json.dumps(output,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'path':str(out),'sha256':sha(out),'status':output['status'],'actual_primary':output['primary_numerical_outcome'],'counts':output['counts'],'primary_values':output['primary_values']},indent=2))


def main():
    global REPORT, EXPECTED_REPORT_SHA, BINARY_BINDING, EXPECTED_BINDING_SHA, OUTPUT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', required=True, type=Path)
    parser.add_argument('--report-sha256', required=True)
    parser.add_argument('--binary-binding', required=True, type=Path)
    parser.add_argument('--binary-binding-sha256', required=True)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    for digest in [args.report_sha256, args.binary_binding_sha256]:
        require(len(digest) == 64 and all(c in '0123456789abcdef' for c in digest), 'Explicit lowercase SHA256 required')
    REPORT, EXPECTED_REPORT_SHA = args.report.resolve(), args.report_sha256
    BINARY_BINDING, EXPECTED_BINDING_SHA = args.binary_binding.resolve(), args.binary_binding_sha256
    OUTPUT = args.output.resolve()
    require(OUTPUT.is_relative_to(HERE) and not OUTPUT.exists(), 'New audit output must be non-overwriting and under owned independent-oracle directory')
    require(REPORT.is_relative_to(ROOT / 'diagonal-intervention-generation-3'), 'Audit only newly retained generation3 report, never relabel prior report')
    try:
        audit()
    except Exception as error:
        rejected = {'status': 'FAIL_INDEPENDENT_GENERATION3_AUDIT_QUALIFICATION',
                    'created_utc': datetime.now(timezone.utc).isoformat(),
                    'error_type': type(error).__name__, 'error': str(error),
                    'report': ref(REPORT) if REPORT.is_file() else {'path': str(REPORT), 'unavailable': True},
                    'binary_binding': ref(BINARY_BINDING) if BINARY_BINDING.is_file() else {'path': str(BINARY_BINDING), 'unavailable': True},
                    'declared_report_sha256': EXPECTED_REPORT_SHA, 'declared_binding_sha256': EXPECTED_BINDING_SHA,
                    'primary_numerical_outcome': 'Not recalculated/admitted after this failed qualification; preserve raw report and its outcomes',
                    'native_worlds_builds_tests_checkout_edits': 0}
        require(not OUTPUT.exists(), 'Never overwrite partially saved audit')
        OUTPUT.write_text(json.dumps(rejected, indent=2, allow_nan=False) + '\n')
        print(json.dumps({'path': str(OUTPUT), 'sha256': sha(OUTPUT), 'status': rejected['status'], 'error': str(error)}, indent=2))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
