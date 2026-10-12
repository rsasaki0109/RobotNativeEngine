#!/usr/bin/env python3
"""Pure independent analytical, IEEE, bounds and domain checks; zero worlds."""

import json
import struct
from fractions import Fraction

from stage_numeric import (EPSILON, F32, MIN_NORMAL, NEG_INF, ONE, POS_INF, SIGN,
                           ZERO, bound_compare, exact, exact_snapshot, negate,
                           rounded)


def bits(value):
    return struct.unpack("<I", struct.pack("<f", value))[0]


def row(jacobian, weighted, rhs, gain, inverse, impulse=0, bounds=(-100, 100)):
    return {"solver_velocity_offset": 1, "ndofs2": len(jacobian),
            "jacobian_words": [bits(x) for x in jacobian],
            "weighted_jacobian_words": [bits(x) for x in weighted],
            "rhs_word": bits(rhs), "cfm_gain_word": bits(gain),
            "inverse_lhs_word": bits(inverse), "impulse_word": bits(impulse),
            "impulse_bound_words": [bits(x) for x in bounds]}


def main():
    checks = []
    def check(name, condition):
        if not condition:
            raise AssertionError(name)
        checks.append(name)

    check("epsilon literal agrees with independently packed decimal", EPSILON == bits(1e-20))
    check("1/3 correctly rounded literal", rounded(Fraction(1, 3)) == 0x3eaaaaab)
    check("positive midpoint chooses even lower", rounded(Fraction(1) + Fraction(1, 1 << 24)) == ONE)
    check("odd lower midpoint chooses even upper", rounded(Fraction(1) + Fraction(3, 1 << 24)) == ONE + 2)
    check("negative midpoint preserves symmetric ties", rounded(-Fraction(1) - Fraction(1, 1 << 24)) == negate(ONE))
    check("least subnormal exact decode", exact(1) == Fraction(1, 1 << 149))
    check("least subnormal midpoint chooses signed zero", rounded(-Fraction(1, 1 << 150)) == SIGN)
    check("largest subnormal/min-normal midpoint chooses min-normal", rounded(MIN_NORMAL - Fraction(1, 1 << 150)) == 0x00800000)
    arithmetic = F32()
    check("negative zero addition", arithmetic.add(SIGN, SIGN) == SIGN)
    check("opposite signed zeros choose positive zero", arithmetic.add(SIGN, ZERO) == ZERO)
    check("signed zero multiplication", arithmetic.multiply(SIGN, ONE) == SIGN)
    check("positive zero divided by negative", arithmetic.divide(ZERO, negate(ONE)) == SIGN)
    check("normal overflow becomes infinity", arithmetic.multiply(0x7f7fffff, bits(2)) == POS_INF)
    check("overflow retained as unqualified", "overflow_intermediate" in arithmetic.exclusions)

    dot = F32()
    # Exact dot is 1; lane-pair reduction first cancels +/-2^24, then adds 1.
    vector = [bits(1 << 24), bits(1), ZERO, ZERO, bits(-(1 << 24)), ZERO, ZERO, ZERO]
    check("eight-lane reduction differs from ascending scalar reduction", dot.dot_dyn([ONE] * 8, vector) == ONE)
    sequential = F32()
    value = ZERO
    for word in vector:
        value = sequential.add(value, word)
    check("independent ascending counterexample is zero", value == ZERO)
    check("Dyn one-element negative-zero dot starts positive", F32().dot_dyn([ONE], [SIGN]) == ZERO)
    check("negative infinity bound orders below finite", bound_compare(NEG_INF, SIGN) < 0)
    check("signed zeros compare equal", bound_compare(SIGN, ZERO) == 0)
    check("native equality returns literal lower signed-zero word", F32().clamp(ZERO, SIGN, ONE) == SIGN)
    check("native equality returns literal upper signed-zero word", F32().clamp(SIGN, negate(ONE), ZERO) == ZERO)
    inverse = F32()
    check("guard inclusive positive epsilon", inverse.inverse(EPSILON) == ZERO)
    check("guard inclusive negative epsilon", inverse.inverse(negate(EPSILON)) == ZERO)
    check("guard rejects next positive word", inverse.inverse(EPSILON + 1) != ZERO)
    tiny = F32()
    tiny.multiply(0x00800000, bits(0.5))
    check("normal inputs with tiny result are unqualified", "tiny_nonzero_exact_intermediate" in tiny.exclusions)
    subnormal = F32()
    subnormal.multiply(1, ONE)
    check("subnormal inputs remain explicit", "subnormal_input" in subnormal.exclusions)

    coupled = row([1, 0], [2, 3], rhs=4, gain=2, inverse=.25)
    velocity = [bits(19), ZERO, ZERO, bits(-7)]
    reference = exact_snapshot(coupled, velocity)
    replay = F32().projection(coupled, velocity)
    check("analytic common projection impulse sign", reference["projection"] == -1)
    check("native accumulator positive but applied negative", replay["next_impulse_word"] == ONE and replay["applied_projection_word"] == negate(ONE))
    check("motor minus AXPY reaches analytic velocity", replay["next_generalized_velocity_words"] == [bits(19), bits(-2), bits(-3), bits(-7)])
    check("uncoupled velocity coordinates unchanged", replay["next_generalized_velocity_words"][0] == velocity[0] and replay["next_generalized_velocity_words"][-1] == velocity[-1])
    check("common conditional one-visit reference satisfies residual", exact_snapshot(dict(coupled, impulse_word=ONE), replay["next_generalized_velocity_words"])["residual"] == 0)
    capped = row([1], [1], rhs=3, gain=0, inverse=1, impulse=1, bounds=(-1, 1))
    cap_reference = exact_snapshot(capped, [ZERO, ZERO])
    check("upper cap positive residual is complementary", cap_reference["projection"] == 0 and cap_reference["complementarity"] == "satisfied")
    lower = dict(capped, rhs_word=bits(-3), impulse_word=bits(-1))
    check("lower cap negative residual is complementary", exact_snapshot(lower, [ZERO, ZERO])["complementarity"] == "satisfied")
    lower_bad = dict(lower, rhs_word=bits(3))
    check("lower cap positive residual violates complementarity", exact_snapshot(lower_bad, [ZERO, ZERO])["complementarity"] == "violated")
    no_authority = dict(capped, impulse_word=ZERO, impulse_bound_words=[SIGN, ZERO])
    check("zero-width projection is not authority", exact_snapshot(no_authority, [ZERO, ZERO])["position"] == "zero_authority")
    unbounded = dict(capped, impulse_word=ZERO, impulse_bound_words=[NEG_INF, POS_INF])
    check("infinite bounds retain finite reference", exact_snapshot(unbounded, [ZERO, ZERO])["projection"] == -3)
    before = exact_snapshot(coupled, velocity)
    after_velocity = [bits(19), bits(.5), bits(-.25), bits(-7)]
    after = exact_snapshot(coupled, after_velocity)
    check("contact aggregate exact delta-r identity", after["residual"] - before["residual"] == Fraction(1, 2))
    for name, operation in [
        ("bool word rejected", lambda: exact(True)),
        ("out-of-width word rejected", lambda: exact(1 << 32)),
        ("NaN finite rational rejected", lambda: exact(0x7fc00001)),
        ("nonpositive denominator rejected", lambda: exact_snapshot(dict(capped, cfm_gain_word=bits(-1)), [ZERO, ZERO])),
        ("reversed bounds rejected", lambda: exact_snapshot(dict(capped, impulse_bound_words=[ONE, negate(ONE)]), [ZERO, ZERO])),
        ("out-of-range block rejected", lambda: exact_snapshot(coupled, [ZERO])),
    ]:
        try:
            operation()
        except ValueError:
            checks.append(name)
        else:
            raise AssertionError(name)
    print(json.dumps({"status": "PASS", "scope": "pure analytical/IEEE/domain checks only", "native_worlds": 0, "checks": checks}, indent=2))


if __name__ == "__main__":
    main()
