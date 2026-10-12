#!/usr/bin/env python3
"""Bounded synthetic probes only; no engine, historical data, or source edits."""
import copy
import hashlib
import json
from pathlib import Path
import sys
from datetime import datetime, timezone

INPUT = Path('/workspace/.rne-onboarding/recovery-20261012/next-motor-construction-protocol')
OUT = Path(__file__).parent
sys.dont_write_bytecode = True
sys.path.insert(0, str(INPUT))
from construction_oracle import verify_construction
from integration_oracle import verify_scalar_advance
from stage_numeric import F32, EPSILON, ONE, ZERO, SIGN, rounded, exact


def golden():
    return dict(coordinate_word=0, target_position_word=ONE, target_velocity_word=0,
                stiffness_word=ONE, damping_word=ONE, duration_word=ONE,
                inverse_duration_word=ONE, limits_words=None,
                effective_target_velocity_word=0, erp_inverse_duration_word=0x3f000000,
                row=dict(gain_model='ForceBased', effort_ceiling_word=0x40000000,
                         rhs_word=0xbf000000, rhs_without_bias_word=0xbf000000,
                         cfm_gain_word=0x3f000000, cfm_coefficient_word=0,
                         impulse_bound_words=[0xc0000000, 0x40000000]))


results = []
def probe(name, condition, observed):
    assert condition, name
    results.append(dict(name=name, assertion_passed=True, observed=observed))


probe('normal_half_valued_golden', verify_construction(golden())['status'] ==
      'DECLARED_FORMULAS_MATCH', verify_construction(golden()))
for sign in (ZERO, SIGN):
    r = golden()
    r.update(coordinate_word=ZERO, target_position_word=ZERO, target_velocity_word=sign,
             limits_words=[SIGN, ZERO], effective_target_velocity_word=sign)
    r['row'].update(rhs_word=ZERO, rhs_without_bias_word=ZERO)
    v = verify_construction(r)
    probe('f32_clamp_retains_equal_' + hex(sign), v['status'] == 'DECLARED_FORMULAS_MATCH', v)
probe('impulse_clamp_differs_at_zero', F32().clamp(ZERO, SIGN, ZERO) == SIGN,
      {'generic_impulse_clamp_word': F32().clamp(ZERO, SIGN, ZERO),
       'finite_f32_clamp_word': ZERO})
for sign in (ZERO, SIGN):
    r = golden(); r['row'].update(effort_ceiling_word=sign,
                                 impulse_bound_words=[sign ^ SIGN, sign])
    v = verify_construction(r)
    probe('zero_authority_bound_sign_' + hex(sign), v['status'] ==
          'DECLARED_FORMULAS_MATCH', v)
for h in (ZERO, SIGN):
    r = golden(); r.update(duration_word=h, inverse_duration_word=ZERO,
                          erp_inverse_duration_word=ONE)
    r['row'].update(cfm_gain_word=ZERO, rhs_word=0xbf800000,
                    rhs_without_bias_word=0xbf800000, impulse_bound_words=[h ^ SIGN, h])
    v = verify_construction(r)
    probe('integration_inv_dt_exact_zero_guard_' + hex(h), v['status'] ==
          'DECLARED_FORMULAS_MATCH', v)
for word in (EPSILON - 1, EPSILON, EPSILON | SIGN, (EPSILON - 1) | SIGN):
    probe('utils_inverse_inclusive_guard_' + hex(word), F32().inverse(word) == ZERO,
          {'input_word': word, 'inverse_word': F32().inverse(word)})
for word in (EPSILON + 1, (EPSILON + 1) | SIGN):
    v = F32().inverse(word)
    probe('utils_inverse_outside_guard_' + hex(word), v == rounded(1 / exact(word)),
          {'input_word': word, 'inverse_word': v})
r = golden(); r['inverse_duration_word'] = 0x7fc00000
v = verify_construction(r)
probe('repaired_nonfinite_captured_inverse_is_output_mismatch',
      v['status'] == 'MISMATCH' and v['exclusions'] == [] and
      'inverse_duration_word' in v['mismatches'], v)
r = golden(); r['row']['rhs_word'] = 0x7fc00000
v = verify_construction(r)
probe('nonfinite_captured_rhs_is_mismatch', v['status'] == 'MISMATCH' and
      'rhs_word' in v['mismatches'], v)
r = golden(); r.update(duration_word=0x7f7fffff, stiffness_word=0x7f7fffff)
try:
    v = verify_construction(r)
except ValueError as error:
    v = {'exception_type': type(error).__name__, 'message': str(error)}
probe('repaired_finite_overflow_returns_structured_exclusion',
      v.get('status') == 'EXCLUDED_SOURCE_FLOAT_DOMAIN' and
      'overflow_intermediate' in v['exclusions'] and v['expected_words'] is None, v)
r = golden(); r['row'].update(impulse_word=ONE, inverse_lhs_word=0x7fc00000,
    ndofs2=-123, solver_velocity_offset=-456, jacobian_words=[], weighted_jacobian_words=[])
v = verify_construction(r)
probe('deliberately_narrow_formula_scope_does_not_validate_full_row',
      v['status'] == 'DECLARED_FORMULAS_MATCH', v)
r = golden(); r['row'].update(effort_ceiling_word=ONE,
    impulse_bound_words=[0xbf800000, ONE])
r['captured_original_motor_max_force_word'] = 0x40000000
v = verify_construction(r)
probe('original_motor_ceiling_not_current_schema_guard',
      v['status'] == 'DECLARED_FORMULAS_MATCH', v)
for key, word in [('coordinate_word', 0x7fc00000), ('coordinate_word', 1)]:
    r = golden(); r[key] = word; v = verify_construction(r)
    probe('excluded_operand_' + hex(word), v['status'] == 'EXCLUDED_SOURCE_FLOAT_DOMAIN', v)



def endpoints(v=ONE, q=ZERO, q_after=ONE, offset=0):
    row = dict(joint_handle={'index': 1, 'generation': 0}, link_id=1, axis='AngX',
        gain_model='ForceBased', constraint_index=0, solver_velocity_offset=offset,
        ndofs2=1, j_id2=0)
    before = dict(row=row, coordinate_word=q, multibody_velocity_words=[v])
    after = copy.deepcopy(before); after['coordinate_word'] = q_after
    return before, after

for v, q_after in [(ONE, ONE), (0xbf800000, 0xbf800000), (SIGN, ZERO)]:
    before, after = endpoints(v=v, q_after=q_after, offset=2)
    result = verify_scalar_advance(ONE, before, after, [0x40000000, ZERO, v])
    probe('integration_copy_and_advance_' + hex(v), result['status'] ==
        'DECLARED_FORMULAS_MATCH', result)
for field in ('coordinate_word', 'multibody_velocity_words'):
    before, after = endpoints()
    if field == 'coordinate_word': after[field] = 0x7fc00000
    else: after[field] = [0x7fc00000]
    result = verify_scalar_advance(ONE, before, after, [ONE])
    probe('integration_nonfinite_output_' + field, result['status'] == 'MISMATCH', result)
before, after = endpoints()
result = verify_scalar_advance(ONE, before, after, [0x40000000])
probe('integration_rejects_later_generic_velocity',
    'copied_integration_velocity_word' in result['mismatches'], result)
before, after = endpoints(); after['multibody_velocity_words'] = [0x40000000]
result = verify_scalar_advance(ONE, before, after, [ONE])
probe('integration_rejects_later_stabilized_multibody_velocity',
    'unchanged_multibody_velocity_word' in result['mismatches'], result)
before, after = endpoints(v=0x7f7fffff, q_after=ONE)
result = verify_scalar_advance(0x7f7fffff, before, after, [0x7f7fffff])
probe('integration_finite_overflow_is_excluded', result['status'] ==
    'EXCLUDED_SOURCE_FLOAT_DOMAIN' and 'overflow_intermediate' in result['exclusions'], result)
for field in ('index', 'generation'):
    before, after = endpoints(); before['row']['joint_handle'][field] = True
    try: verify_scalar_advance(ONE, before, after, [ONE])
    except ValueError as error: result = str(error)
    else: raise AssertionError('boolean handle accepted')
    probe('integration_boolean_handle_' + field, True, result)
for field in ('link_id', 'constraint_index', 'solver_velocity_offset', 'ndofs2', 'j_id2'):
    before, after = endpoints(); before['row'][field] = True
    try: verify_scalar_advance(ONE, before, after, [ONE])
    except ValueError as error: result = str(error)
    else: raise AssertionError('boolean row index accepted')
    probe('integration_boolean_index_' + field, True, result)
before, after = endpoints(); after['row']['joint_handle']['generation'] = 1
try: verify_scalar_advance(ONE, before, after, [ONE])
except ValueError as error: result = str(error)
else: raise AssertionError('mixed identities accepted')
probe('integration_mixed_handle_generation', True, result)
for axis, model, ndofs in [('LinX', 'ForceBased', 1), ('AngY', 'ForceBased', 1),
                         ('AngX', 'AccelerationBased', 1), ('AngX', 'ForceBased', 2)]:
    before, after = endpoints()
    for endpoint in (before, after):
        endpoint['row'].update(axis=axis, gain_model=model, ndofs2=ndofs)
    try: verify_scalar_advance(ONE, before, after, [ONE])
    except ValueError as error: result = str(error)
    else: raise AssertionError('unsupported scalar scope accepted')
    probe('integration_declared_scope_' + axis + '_' + model + '_' + str(ndofs), True, result)

proof = dict(kind='independent_final_synthetic_probes_of_corrected_prototypes',
             ended_utc=datetime.now(timezone.utc).isoformat(),
             assertions_passed=len(results), native_worlds_launched=0,
             native_data_read=False, qualification_claim='source-only prototype review',
             probes=results)
p = OUT / 'independent-final-pure-probe-results.json'
p.write_text(json.dumps(proof, indent=2) + '\n')
print(json.dumps(dict(status='passed', assertions=len(results), output=str(p),
    sha256=hashlib.sha256(p.read_bytes()).hexdigest(), bytes=p.stat().st_size)))
