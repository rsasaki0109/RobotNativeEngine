#!/usr/bin/env python3
"""Independent exact-rational and source-ordered binary32 motor-row arithmetic.

No simulation, native library, historical oracle import, or tolerance is used.
Native replay is qualified only in the normal-or-zero arithmetic domain. Tiny
nonzero exact intermediates and subnormal inputs remain explicit exclusions.
"""

from fractions import Fraction

SIGN = 0x80000000
ONE = 0x3f800000
ZERO = 0
POS_INF = 0x7f800000
NEG_INF = 0xff800000
MIN_NORMAL = Fraction(1, 1 << 126)
EPSILON = 0x1e3ce508  # Literal f32(1e-20), independently checked in pure tests.


def checked_word(value, width=32):
    if type(value) is not int or not 0 <= value < (1 << width):
        raise ValueError("native word must be a width-bounded integer, not bool")
    return value


def kind(word):
    word = checked_word(word)
    exponent = (word >> 23) & 255
    fraction = word & 0x7fffff
    if exponent == 255:
        return "nan" if fraction else "negative_infinity" if word & SIGN else "positive_infinity"
    if exponent == 0:
        return "subnormal" if fraction else "negative_zero" if word & SIGN else "positive_zero"
    return "normal"


def exact(word):
    word = checked_word(word)
    exponent = (word >> 23) & 255
    mantissa = word & 0x7fffff
    if exponent == 255:
        raise ValueError("nonfinite IEEE word has no finite rational value")
    if exponent:
        mantissa += 1 << 23
        power = exponent - 150
    else:
        power = -149
    numerator = -mantissa if word & SIGN else mantissa
    return Fraction(numerator << power) if power >= 0 else Fraction(numerator, 1 << -power)


def rational(value):
    return {"numerator": str(value.numerator), "denominator": str(value.denominator)}


def _floor_log2(value):
    result = value.numerator.bit_length() - value.denominator.bit_length()
    power = Fraction(1 << result) if result >= 0 else Fraction(1, 1 << -result)
    return result - (value < power)


def _ties_even(value):
    integer, remainder = divmod(value.numerator, value.denominator)
    twice = remainder * 2
    return integer + (twice > value.denominator or (twice == value.denominator and integer % 2))


def rounded(value, negative_zero=False):
    """Round a finite Fraction once to binary32, RN ties-to-even, gradual underflow."""
    value = Fraction(value)
    if value == 0:
        return SIGN if negative_zero else ZERO
    sign = SIGN if value < 0 else ZERO
    magnitude = abs(value)
    exponent = _floor_log2(magnitude)
    quantum = max(exponent - 23, -149)
    scaled = magnitude / (1 << quantum) if quantum >= 0 else magnitude * (1 << -quantum)
    mantissa = _ties_even(scaled)
    if not mantissa:
        return sign
    if mantissa >= (1 << 24):
        mantissa >>= 1
        quantum += 1
    if quantum == -149 and mantissa < (1 << 23):
        return sign | mantissa
    biased = quantum + 150
    if biased >= 255:
        return sign | POS_INF
    return sign | (biased << 23) | (mantissa - (1 << 23))


def negate(word):
    return checked_word(word) ^ SIGN


def _bound(word):
    word = checked_word(word)
    if word == NEG_INF:
        return -1, None
    if word == POS_INF:
        return 1, None
    return 0, exact(word)


def bound_compare(left, right):
    la, lv = _bound(left)
    ra, rv = _bound(right)
    if la != ra:
        return (la > ra) - (la < ra)
    if la:
        return 0
    return (lv > rv) - (lv < rv)


def clamp_fraction(value, lower_word, upper_word):
    lower_tag, lower = _bound(lower_word)
    upper_tag, upper = _bound(upper_word)
    if bound_compare(lower_word, upper_word) > 0:
        raise ValueError("unordered native accumulator bounds")
    if lower_tag == 1 or upper_tag == -1:
        raise ValueError("no finite feasible accumulator")
    if not lower_tag and value < lower:
        return lower
    if not upper_tag and value > upper:
        return upper
    return value


class F32:
    """Source-order RN emulator, with independent eligibility accounting.

    Its gradual-underflow results are illustrative only outside the declared
    normal-or-zero domain. The mode binding must separately establish RN and
    source/compiler operation order; FTZ/DAZ cannot be inferred from snapshots.
    """

    def __init__(self):
        self.exclusions = set()
        self.operation_count = 0

    def input(self, word):
        category = kind(word)
        if category == "subnormal":
            self.exclusions.add("subnormal_input")
        elif "infinity" in category or category == "nan":
            self.exclusions.add("nonfinite_arithmetic_input")
        return checked_word(word)

    def finish(self, value, negative_zero=False):
        self.operation_count += 1
        if value and abs(value) < MIN_NORMAL:
            self.exclusions.add("tiny_nonzero_exact_intermediate")
        result = rounded(value, negative_zero)
        if "infinity" in kind(result):
            self.exclusions.add("overflow_intermediate")
        return result

    def add(self, left, right):
        self.input(left)
        self.input(right)
        return self.finish(exact(left) + exact(right), left == SIGN and right == SIGN)

    def subtract(self, left, right):
        return self.add(left, negate(right))

    def multiply(self, left, right):
        self.input(left)
        self.input(right)
        return self.finish(exact(left) * exact(right), bool((left ^ right) & SIGN))

    def divide(self, numerator, denominator):
        self.input(numerator)
        self.input(denominator)
        if exact(denominator) == 0:
            raise ValueError("native replay division by zero")
        return self.finish(exact(numerator) / exact(denominator), bool((numerator ^ denominator) & SIGN))

    def dot_dyn(self, left, right):
        if len(left) != len(right):
            raise ValueError("dot shape mismatch")
        # nalgebra 0.33.3 Dyn/Dyn dotx, one column. Do not use Python sum.
        result = ZERO
        accumulators = [ZERO] * 8
        index = 0
        while index + 8 <= len(left):
            for lane in range(8):
                accumulators[lane] = self.add(
                    accumulators[lane], self.multiply(left[index + lane], right[index + lane]))
            index += 8
        for lane in range(4):
            result = self.add(result, self.add(accumulators[lane], accumulators[lane + 4]))
        for tail in range(index, len(left)):
            result = self.add(result, self.multiply(left[tail], right[tail]))
        return result

    def clamp(self, value, lower, upper):
        self.input(value)
        # Bounds may be infinite. They are comparison operands, not arithmetic.
        if bound_compare(lower, upper) > 0:
            raise ValueError("unordered native accumulator bounds")
        return value if bound_compare(value, lower) > 0 and bound_compare(value, upper) < 0 else (
            upper if bound_compare(value, lower) > 0 else lower)

    def inverse(self, value):
        self.input(value)
        if -exact(EPSILON) <= exact(value) <= exact(EPSILON):
            return ZERO
        return self.divide(ONE, value)

    def axpy(self, a, x, y):
        if len(x) != len(y):
            raise ValueError("AXPY shape mismatch")
        return [self.add(self.multiply(self.multiply(a, xi), ONE), self.multiply(ONE, yi))
                for xi, yi in zip(x, y)]

    def projection(self, row, velocity_words, include_axpy=True):
        offset = row["solver_velocity_offset"]
        length = row["ndofs2"]
        block = velocity_words[offset:offset + length]
        if len(block) != length:
            raise ValueError("motor velocity block out of range")
        dot = self.dot_dyn(row["jacobian_words"], block)
        dvel = self.add(row["rhs_word"], dot)
        gain_term = self.multiply(row["cfm_gain_word"], row["impulse_word"])
        residual = self.subtract(dvel, gain_term)
        correction = self.multiply(row["inverse_lhs_word"], residual)
        trial = self.add(row["impulse_word"], correction)
        next_word = self.clamp(trial, *row["impulse_bound_words"])
        delta = self.subtract(next_word, row["impulse_word"])
        full = None
        if include_axpy:
            next_block = self.axpy(negate(delta), row["weighted_jacobian_words"], block)
            full = list(velocity_words)
            full[offset:offset + length] = next_block
        return {"dot_word": dot, "dvel_word": dvel, "gain_term_word": gain_term,
                "residual_word": residual, "correction_word": correction,
                "trial_word": trial, "next_impulse_word": next_word,
                "delta_impulse_word": delta, "applied_projection_word": negate(delta),
                "next_generalized_velocity_words": full,
                "eligibility": "qualified" if not self.exclusions else "unqualified",
                "exclusions": sorted(self.exclusions), "operation_count": self.operation_count}


def exact_snapshot(row, velocity_words):
    """Common conditional projection: exact stored operands, not rounded D."""
    offset, length = row["solver_velocity_offset"], row["ndofs2"]
    block = velocity_words[offset:offset + length]
    if len(block) != length or len(row["jacobian_words"]) != length or len(row["weighted_jacobian_words"]) != length:
        raise ValueError("motor vector shape mismatch")
    jacobian = [exact(word) for word in row["jacobian_words"]]
    weighted = [exact(word) for word in row["weighted_jacobian_words"]]
    velocities = [exact(word) for word in block]
    impulse = exact(row["impulse_word"])
    gain = exact(row["cfm_gain_word"])
    residual = exact(row["rhs_word"]) + sum((j * v for j, v in zip(jacobian, velocities)), Fraction()) - gain * impulse
    diagonal = sum((j * w for j, w in zip(jacobian, weighted)), Fraction())
    denominator = diagonal + gain
    if denominator <= 0:
        raise ValueError("nonpositive common conditional denominator")
    lower, upper = row["impulse_bound_words"]
    projected = clamp_fraction(impulse + residual / denominator, lower, upper)
    position = ("zero_authority" if bound_compare(lower, upper) == 0 else
                "at_lower_bound" if bound_compare(row["impulse_word"], lower) == 0 else
                "at_upper_bound" if bound_compare(row["impulse_word"], upper) == 0 else
                "outside_bounds" if bound_compare(row["impulse_word"], lower) < 0 or bound_compare(row["impulse_word"], upper) > 0 else "interior")
    complementarity = ("zero_authority" if position == "zero_authority" else
                       "satisfied" if (position == "at_lower_bound" and residual <= 0) or
                       (position == "at_upper_bound" and residual >= 0) or
                       (position == "interior" and residual == 0) else "violated")
    return {"residual": residual, "diagonal": diagonal, "denominator": denominator,
            "conditional_impulse": projected, "projection": impulse - projected,
            "position": position, "complementarity": complementarity}


def serialize_snapshot(values):
    return {key: rational(value) if isinstance(value, Fraction) else value for key, value in values.items()}
