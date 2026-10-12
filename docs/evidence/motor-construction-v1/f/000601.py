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
probe('nonfinite_captured_inverse_is_currently_input_exclusion',
      v['status'] == 'EXCLUDED_SOURCE_FLOAT_DOMAIN' and
      v['nonfinite_named_inputs'] == ['inverse_duration_word'], v)
r = golden(); r['row']['rhs_word'] = 0x7fc00000
v = verify_construction(r)
probe('nonfinite_captured_rhs_is_mismatch', v['status'] == 'MISMATCH' and
      'rhs_word' in v['mismatches'], v)
r = golden(); r.update(duration_word=0x7f7fffff, stiffness_word=0x7f7fffff)
try:
    v = verify_construction(r)
except ValueError as error:
    v = {'exception_type': type(error).__name__, 'message': str(error)}
probe('finite_overflow_has_no_structured_exclusion_return',
      v.get('exception_type') == 'ValueError' and
      v.get('message') == 'nonfinite IEEE word has no finite rational value', v)
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

proof = dict(kind='independent_synthetic_probes_of_prospective_prototype',
             ended_utc=datetime.now(timezone.utc).isoformat(),
             assertions_passed=len(results), native_worlds_launched=0,
             native_data_read=False, qualification_claim='source-only prototype review',
             probes=results)
p = OUT / 'independent-pure-probe-results.json'
p.write_text(json.dumps(proof, indent=2) + '\n')
print(json.dumps(dict(status='passed', assertions=len(results), output=str(p),
    sha256=hashlib.sha256(p.read_bytes()).hexdigest(), bytes=p.stat().st_size)))
