"""Exact real scalar reference and synthetic row algebra; no native engine/data."""
from fractions import Fraction as F


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def backward_euler(inertia, stiffness, damping, duration, coordinate, velocity,
                   target, ceiling):
    """Capped implicit PD; arguments and results are exact rational SI quantities."""
    if inertia <= 0 or duration <= 0 or stiffness < 0 or damping < 0 or ceiling < 0:
        raise ValueError("invalid scalar physical model")
    error = target - coordinate
    denominator = inertia + duration * damping + duration * duration * stiffness
    free_velocity = (inertia * velocity + duration * stiffness * error) / denominator
    free_impulse = inertia * (free_velocity - velocity)
    impulse = clamp(free_impulse, -duration * ceiling, duration * ceiling)
    next_velocity = velocity + impulse / inertia
    next_coordinate = coordinate + duration * next_velocity
    return {
        "impulse": impulse, "torque": impulse / duration,
        "velocity": next_velocity, "coordinate": next_coordinate,
        "free_impulse": free_impulse,
    }


def ideal_row(inertia, stiffness, damping, duration, coordinate, target):
    """Conditional exact-real translation of this one positive-gain scalar row."""
    if inertia <= 0 or duration <= 0 or duration * stiffness + damping <= 0:
        raise ValueError("unsupported ideal-row denominator")
    a = stiffness / (duration * stiffness + damping)
    return {"w": 1 / inertia, "g": 1 / (duration * (duration * stiffness + damping)),
            "rhs": a * (coordinate - target)}


def visit(row, incoming_velocity, impulse, diagonal, cap_impulse):
    """One synthetic exact-real native-sign visit, returning physical p=-lambda."""
    velocity = incoming_velocity + row["w"] * impulse
    residual = -row["rhs"] - velocity - row["g"] * impulse
    new_impulse = clamp(impulse + diagonal * residual, -cap_impulse, cap_impulse)
    return new_impulse, incoming_velocity + row["w"] * new_impulse


def one_slice(row, incoming_velocity, coordinate, duration, cap_impulse, pgs, stabilization):
    """Synthetic fixed-cache timing; does not model any native rounded execution."""
    impulse = F(0)
    velocity = incoming_velocity
    for _ in range(pgs):
        impulse, velocity = visit(row, incoming_velocity, impulse, 1 / row["w"], cap_impulse)
    integration_velocity = velocity
    next_coordinate = coordinate + duration * integration_velocity
    for _ in range(stabilization):
        impulse, velocity = visit(row, incoming_velocity, impulse, 1 / row["w"], cap_impulse)
    return {"coordinate": next_coordinate, "integration_velocity": integration_velocity,
            "final_velocity": velocity, "final_impulse": impulse}
