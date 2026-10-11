#!/usr/bin/env python3
"""Generation-3 independent exact-binary32 ForceBased diagonal oracle; no physics worlds.

Inputs are native IEEE-754 binary32 bit patterns. Every source scalar operation
is rounded independently to nearest, ties-to-even using exact rational arithmetic.
The reference impulse is the mathematical projected fixed point of the frozen
native coefficients, not a prediction of a finite number of solver passes.
"""
from fractions import Fraction
import argparse
import json
import math
from pathlib import Path
import struct

SIGN = 1 << 31
ONE = 0x3f800000
ZERO = 0


def checked_bits(bits):
    if type(bits) is not int or not 0 <= bits < (1 << 32):
        raise ValueError('binary32 bits must be an unsigned integer, not bool')
    if (bits >> 23) & 255 == 255:
        raise ValueError('nonfinite native scalar is unavailable, never zero-filled')
    return bits


def scalar(bits):
    checked_bits(bits)
    return struct.unpack('<f', struct.pack('<I', bits))[0]


def bits_from_decimal(value):
    """Used only for constant and pure-fixture construction, never measured input."""
    return struct.unpack('<I', struct.pack('<f', value))[0]


def exact(bits):
    checked_bits(bits)
    sign = -1 if bits & SIGN else 1
    exponent = (bits >> 23) & 255
    fraction = bits & ((1 << 23) - 1)
    if exponent == 0:
        mantissa, power = fraction, -149
    else:
        mantissa, power = (1 << 23) | fraction, exponent - 127 - 23
    return sign * Fraction(mantissa) * Fraction(2) ** power


def rounded(value, zero_negative=False):
    """Round an exact rational to binary32, with explicit signed-zero handling."""
    if not value:
        return SIGN if zero_negative else ZERO
    negative = value < 0
    x = abs(value)
    exponent = x.numerator.bit_length() - x.denominator.bit_length()
    if x < Fraction(2) ** exponent:
        exponent -= 1
    quantum = max(exponent - 23, -149)
    scaled = x / (Fraction(2) ** quantum)
    integer, remainder = divmod(scaled.numerator, scaled.denominator)
    twice = 2 * remainder
    if twice > scaled.denominator or (twice == scaled.denominator and integer & 1):
        integer += 1
    sign = SIGN if negative else ZERO
    if not integer:
        return sign
    if quantum == -149 and integer < (1 << 23):
        return sign | integer
    if integer == (1 << 24):
        integer >>= 1
        quantum += 1
    biased_exponent = quantum + 23 + 127
    if biased_exponent >= 255:
        raise ValueError('binary32 arithmetic overflow is unavailable')
    return sign | (biased_exponent << 23) | (integer - (1 << 23))


def add(a, b):
    return rounded(exact(a) + exact(b), a == SIGN and b == SIGN)


def negate(a):
    checked_bits(a)
    return a ^ SIGN


def subtract(a, b):
    return add(a, negate(b))


def multiply(a, b):
    return rounded(exact(a) * exact(b), bool((a ^ b) & SIGN))


def divide(a, b):
    if not exact(b):
        raise ValueError('zero division outside Rapier guarded inv')
    return rounded(exact(a) / exact(b), bool((a ^ b) & SIGN))


EPSILON = bits_from_decimal(1.0e-20)


def rapier_inv(value):
    # src/utils.rs:25-32, closed interval including both signed zeros.
    if -exact(EPSILON) <= exact(value) <= exact(EPSILON):
        return ZERO
    return divide(ONE, value)


def describe(value):
    return {'bits_u32': value, 'hex': f'{value:08x}', 'promoted_f64': scalar(value)}


def evaluate(row):
    """Evaluate one initial isolated row from explicit native inputs.

    post_step_inverse_mass_bits_u32 is a separate optional cache cross-check;
    it is never silently substituted for pre-step fixture-derived inertia.
    """
    names = ['outer_interval_bits_u32', 'stiffness_bits_u32', 'damping_bits_u32',
             'pre_effective_inertia_xx_bits_u32', 'initial_coordinate_bits_u32',
             'initial_velocity_bits_u32', 'target_coordinate_bits_u32',
             'target_velocity_bits_u32', 'effort_cap_bits_u32']
    for name in names:
        checked_bits(row[name])
    count = row['small_step_count']
    if type(count) is not int or count <= 0 or count > (1 << 24):
        raise ValueError('small-step count must be positive/exactly representable in f32')
    dt, k, d = (row[name] for name in names[:3])
    inertia = row['pre_effective_inertia_xx_bits_u32']
    cap = row['effort_cap_bits_u32']
    if scalar(dt) <= 0 or scalar(inertia) <= 0 or scalar(k) < 0 or scalar(d) < 0 or scalar(cap) < 0:
        raise ValueError('positive dt/inertia and nonnegative gains/cap are required')
    h = divide(dt, bits_from_decimal(count))
    # Exact left-associated source expression: h*h*k + h*d.
    h_k_plus_d = add(multiply(h, k), d)
    h_h_k_plus_h_d = add(multiply(multiply(h, h), k), multiply(h, d))
    a = multiply(k, rapier_inv(h_k_plus_d))
    g = rapier_inv(h_h_k_plus_h_d)
    # For identity-X coincident COM/pivot and no passive damping/armature:
    # a 1x1 native LU solve of J=[1] is f32(1 / native Ixx).
    w = divide(ONE, inertia)
    legacy_inv_lhs = rapier_inv(w)
    candidate_diagonal_sum = add(w, g)
    inv_lhs = rapier_inv(candidate_diagonal_sum)
    error = subtract(row['initial_coordinate_bits_u32'], row['target_coordinate_bits_u32'])
    rhs = ZERO
    if scalar(a) != 0:
        rhs = add(rhs, multiply(error, a))
    rhs = add(rhs, negate(row['target_velocity_bits_u32']))
    bound = multiply(cap, h)
    b = exact(bound)
    forcing = exact(rhs) + exact(row['initial_velocity_bits_u32'])
    denominator = exact(w) + exact(g)
    if denominator <= 0:
        raise ValueError('frozen-row reference denominator must be positive')
    reference = max(-b, min(b, -forcing / denominator))
    unbounded = -forcing / denominator
    tolerance = max(1.0e-8, 0.01 * abs(float(reference)))
    coeffs = {'small_step_duration': h, 'erp_inv_dt': a, 'cfm_coeff': ZERO,
              'cfm_gain': g, 'fixture_derived_inverse_mass': w,
              'unit_motor_inv_lhs': inv_lhs, 'candidate_rounded_diagonal_sum': candidate_diagonal_sum,
              'legacy_unit_motor_inv_lhs': legacy_inv_lhs, 'native_rhs_wo_bias': rhs,
              'native_impulse_bound': bound}
    output = {
        'scope': 'Initial isolated frozen row, source-derived candidate inv(fl32(w+g)); no simulated outcome or G1 recovery claim',
        'diagonal_acquisition': 'Derived from preregistered source expression plus pre-step inertia; inv_lhs is not directly measured in native telemetry',
        'fixed_point_contract': 'Unchanged exact native-coefficient projected fixed point; candidate diagonal rounds the sum before reciprocal, while reference denominator promotes w and g independently to f64',
        'acquisition': 'pre-step native effective Ixx plus explicit fixture invariants; post-step LU is separately named',
        'native_f32_expression': {name: describe(bits) for name, bits in coeffs.items()},
        'reference': {'applied_impulse_nm_s': float(reference),
                      'unbounded_applied_impulse_nm_s': float(unbounded),
                      'exact_rational_numerator': str(reference.numerator),
                      'exact_rational_denominator': str(reference.denominator),
                      'native_solver_lambda_nm_s': -float(reference),
                      'interior': abs(reference) < b,
                      'tolerance_nm_s': tolerance,
                      'unclipped_homogeneous_multiplier': float(1 - exact(inv_lhs) * denominator),
                      'legacy_rounded_diagonal_multiplier': float(1 - exact(legacy_inv_lhs) * denominator),
                      'legacy_ideal_diagonal_multiplier': -float(exact(g) / exact(w)),
                      'finite_pass_convergence_not_assumed': True},
        'actual_first_slice_comparison': None,
        'post_step_lu_comparison': None,
        'source_coefficient_comparison': None,
    }
    observed = row.get('observed_native_coefficient_bits_u32')
    if observed is not None:
        wanted = {name: coeffs[name] for name in ['erp_inv_dt', 'cfm_coeff', 'cfm_gain', 'native_rhs_wo_bias', 'native_impulse_bound']}
        if set(observed) != set(wanted):
            raise ValueError('native coefficient key/presence mismatch')
        for value in observed.values():
            checked_bits(value)
        output['source_coefficient_comparison'] = {'match': observed == wanted, 'expected_bits': wanted, 'actual_bits': observed}
    post = row.get('post_step_inverse_mass_bits_u32')
    if post is not None:
        post_value = scalar(post)
        difference = abs(post_value - scalar(w))
        threshold = max(1.0e-8, 2.0e-5 * abs(scalar(w)))
        output['post_step_lu_comparison'] = {'actual': describe(post), 'absolute_error': difference,
                                                'tolerance': threshold, 'match': difference <= threshold,
                                                'scope': 'Post-step cache cross-check; not direct first-row cache measurement'}
    actual = row.get('actual_first_applied_impulse_nm_s')
    if actual is not None:
        if type(actual) not in (int, float) or not math.isfinite(actual):
            raise ValueError('nonfinite/malformed actual impulse remains unavailable')
        actual = float(actual)
        if scalar(bits_from_decimal(actual)) != actual:
            raise ValueError('actual impulse must preserve native f32 promoted value')
        error_nm_s = abs(actual - float(reference))
        lam = -actual
        residual = float(forcing) - float(denominator) * lam
        projected_lam = max(-float(b), min(float(b), lam + float(exact(inv_lhs)) * residual))
        output['actual_first_slice_comparison'] = {
            'actual_applied_impulse_nm_s': actual, 'absolute_error_nm_s': error_nm_s,
            'within_declared_tolerance': error_nm_s <= tolerance,
            'frozen_row_residual_rad_s': residual,
            'projected_fixed_point_residual_nm_s': lam - projected_lam,
            'all_other_physical_and_replay_gates_not_evaluated_by_this_oracle': True,
        }
    return output


def self_test():
    assert scalar(rounded(Fraction(1, 3))) == scalar(bits_from_decimal(1 / 3))
    assert add(SIGN, SIGN) == SIGN
    assert add(SIGN, ZERO) == ZERO
    assert multiply(SIGN, ONE) == SIGN
    assert divide(SIGN, ONE) == SIGN
    assert rapier_inv(EPSILON) == ZERO
    assert rapier_inv(negate(EPSILON)) == ZERO
    assert rapier_inv(EPSILON + 1) != ZERO
    assert add(ONE, rounded(Fraction(1, 1 << 24))) == ONE  # ties-to-even
    assert add(ONE + 1, rounded(Fraction(1, 1 << 24))) == ONE + 2
    base = {'outer_interval_bits_u32': bits_from_decimal(.001),
            'stiffness_bits_u32': bits_from_decimal(300), 'damping_bits_u32': bits_from_decimal(12),
            'pre_effective_inertia_xx_bits_u32': bits_from_decimal(.004),
            'initial_coordinate_bits_u32': ZERO, 'initial_velocity_bits_u32': ZERO,
            'target_coordinate_bits_u32': bits_from_decimal(.001), 'target_velocity_bits_u32': ZERO,
            'effort_cap_bits_u32': bits_from_decimal(2), 'small_step_count': 32}
    positive = evaluate(base)
    negative = evaluate(dict(base, target_coordinate_bits_u32=negate(base['target_coordinate_bits_u32'])))
    zero = evaluate(dict(base, target_coordinate_bits_u32=ZERO))
    assert positive['reference']['applied_impulse_nm_s'] == -negative['reference']['applied_impulse_nm_s']
    assert positive['reference']['interior']
    assert zero['reference']['applied_impulse_nm_s'] == 0.0
    assert positive['actual_first_slice_comparison'] is None
    assert evaluate(dict(base, effort_cap_bits_u32=ZERO))['reference']['applied_impulse_nm_s'] == 0
    for malformed in [True, -1, 1 << 32, 0x7f800000, 0x7fc00000]:
        try:
            checked_bits(malformed)
        except ValueError:
            pass
        else:
            raise AssertionError('invalid native value admitted')
    print(json.dumps({'status': 'PASS', 'scope': 'Pure exact-arithmetic and input-semantics tests only; zero worlds/builds',
                      'illustration_native_Ixx': '.004 literal rounded to f32, not measured sphere inertia',
                      'illustration': positive}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('input', nargs='?', type=Path)
    args = parser.parse_args()
    if args.self_test:
        if args.input:
            parser.error('--self-test does not read measurement inputs')
        self_test()
    elif args.input:
        print(json.dumps(evaluate(json.loads(args.input.read_text())), indent=2, allow_nan=False))
    else:
        parser.error('provide an input JSON or --self-test')


if __name__ == '__main__':
    main()
