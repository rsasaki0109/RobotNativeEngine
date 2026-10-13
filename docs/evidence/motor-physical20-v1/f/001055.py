"""Prospective pure construction check; no native execution or accuracy claim.

This prototype consumes companion construction rows, never historical traces.
It uses separately rounded f32 operations and the declared source order. Runtime
source/feature/float-mode binding and whole-cohort qualification are still required.
"""
from stage_numeric import F32, ONE, ZERO, checked_word, exact, kind, negate


def require(condition, message):
    if not condition:
        raise ValueError(message)


def _word(record, key):
    require(key in record, 'missing word: ' + key)
    return checked_word(record[key])


def verify_construction(record, native_scalar_bits=32):
    """Check actual captured inputs against ERP/RHS/CFM/bounds only.

    Do not pass a later row with removed bias or a post-integration coordinate.
    Endpoint identity and event/source binding belong to the future cohort reader.
    Source-consistent arithmetic does not imply a converged motor or correct plant.
    """
    require(type(native_scalar_bits) is int and native_scalar_bits == 32,
            'unsupported native width')
    require(type(record) is dict and type(record.get('row')) is dict,
            'missing construction row')
    row = record['row']
    model = row.get('gain_model')
    require(model in ('ForceBased', 'AccelerationBased'), 'unsupported gain model')
    inputs = {key: _word(record, key) for key in (
        'coordinate_word', 'target_position_word', 'target_velocity_word',
        'stiffness_word', 'damping_word', 'duration_word')}
    cap = _word(row, 'effort_ceiling_word')
    bounds = row.get('impulse_bound_words')
    require(type(bounds) is list and len(bounds) == 2, 'missing bounds')
    bounds = [checked_word(word) for word in bounds]
    require('limits_words' in record, 'missing limit availability')
    limits = record['limits_words']
    if limits is not None:
        require(type(limits) is list and len(limits) == 2, 'bad limits')
        limits = [checked_word(word) for word in limits]
    checked_outputs = {
        'inverse_duration_word': _word(record, 'inverse_duration_word'),
        'erp_inverse_duration_word': _word(record, 'erp_inverse_duration_word'),
        'effective_target_velocity_word': _word(record, 'effective_target_velocity_word'),
        'rhs_word': _word(row, 'rhs_word'),
        'rhs_without_bias_word': _word(row, 'rhs_without_bias_word'),
        'cfm_gain_word': _word(row, 'cfm_gain_word'),
        'cfm_coefficient_word': _word(row, 'cfm_coefficient_word'),
        'lower_impulse_bound_word': bounds[0],
        'upper_impulse_bound_word': bounds[1],
    }
    arithmetic_inputs = list(inputs.values()) + [cap] + (limits or [])
    nonfinite = [key for key, value in inputs.items()
                 if kind(value) in ('nan', 'positive_infinity', 'negative_infinity')]
    if any(kind(word) in ('nan', 'positive_infinity', 'negative_infinity')
           for word in arithmetic_inputs):
        return {'status': 'EXCLUDED_SOURCE_FLOAT_DOMAIN', 'exclusions':
                ['nonfinite_arithmetic_input'], 'nonfinite_named_inputs': nonfinite,
                'observed_words': checked_outputs, 'mismatches': None,
                'motor_accuracy_verified': False}
    h = inputs['duration_word']
    require(exact(h) >= 0 and exact(cap) >= 0, 'negative duration or effort cap')
    f = F32()
    for word in arithmetic_inputs:
        f.input(word)
    try:
        expected = _expected_words(inputs, cap, limits, model, f)
    except ValueError:
        if not f.exclusions:
            raise
        return {'status': 'EXCLUDED_SOURCE_FLOAT_DOMAIN',
                'exclusions': sorted(f.exclusions), 'expected_words': None,
                'observed_words': checked_outputs, 'mismatches': None,
                'operation_count': f.operation_count,
                'motor_accuracy_verified': False}
    mismatches = {key: {'expected_word': value, 'observed_word': checked_outputs[key]}
                  for key, value in expected.items() if value != checked_outputs[key]}
    status = ('EXCLUDED_SOURCE_FLOAT_DOMAIN' if f.exclusions else
              'DECLARED_FORMULAS_MATCH' if not mismatches else 'MISMATCH')
    return {'status': status, 'exclusions': sorted(f.exclusions),
            'expected_words': expected, 'observed_words': checked_outputs,
            'mismatches': mismatches, 'operation_count': f.operation_count,
            'motor_accuracy_verified': False}


def _expected_words(inputs, cap, limits, model, f):
    h, k, d = (inputs[name] for name in ('duration_word', 'stiffness_word', 'damping_word'))
    q, target, velocity = (inputs[name] for name in ('coordinate_word', 'target_position_word', 'target_velocity_word'))
    inv_h = ZERO if exact(h) == 0 else f.divide(ONE, h)
    erp = f.multiply(k, f.inverse(f.add(f.multiply(h, k), d)))
    compliance = f.inverse(f.add(f.multiply(f.multiply(h, h), k), f.multiply(h, d)))
    # motor_params computes max_impulse before the constructor builds RHS.
    bound = f.multiply(cap, h)
    rhs = ZERO
    if exact(erp) != 0:
        rhs = f.add(rhs, f.multiply(f.subtract(q, target), erp))
    effective_velocity = velocity
    if limits is not None:
        lower = f.multiply(f.subtract(limits[0], q), inv_h)
        upper = f.multiply(f.subtract(limits[1], q), inv_h)
        require(exact(lower) <= exact(upper), 'unordered velocity limits')
        # Rust f32::clamp uses strict comparisons and retains equal signed zero.
        if exact(velocity) < exact(lower):
            effective_velocity = lower
        elif exact(velocity) > exact(upper):
            effective_velocity = upper
    rhs = f.add(rhs, negate(effective_velocity))
    expected = {
        'inverse_duration_word': inv_h,
        'erp_inverse_duration_word': erp,
        'effective_target_velocity_word': effective_velocity,
        'rhs_word': rhs,
        'rhs_without_bias_word': rhs,
        'cfm_gain_word': compliance if model == 'ForceBased' else ZERO,
        'cfm_coefficient_word': ZERO if model == 'ForceBased' else compliance,
        'lower_impulse_bound_word': negate(bound),
        'upper_impulse_bound_word': bound,
    }
    return expected
