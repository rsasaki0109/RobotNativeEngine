"""Walking metrics shared by the MuJoCo reference and checked against Rust.

Inputs are 50 Hz policy-step records in the upstream Z-up world: pelvis
position and wxyz orientation. The Rust runner computes the same quantities in
`walk.rs`; keep both definitions identical.
"""
import math

COMMANDS = [[0.5, 0.0, 0.0], [0.0, 0.0, 0.0], [0.3, 0.0, 0.5], [0.8, 0.0, 0.0]]
FALL_HEIGHT_M = 0.55


def yaw(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def summarize(records, command):
    positions = [r["base_position_m"] for r in records]
    yaws = [yaw(r["base_quaternion_wxyz"]) for r in records]
    heights = [p[2] for p in positions]
    fell_at = next((r["time_s"] for r, h in zip(records, heights) if h < FALL_HEIGHT_M), None)
    half = len(records) // 2
    forward, lateral, turn = [], [], []
    for index in range(half, len(records)):
        dt = records[index]["time_s"] - records[index - 1]["time_s"]
        dx = positions[index][0] - positions[index - 1][0]
        dy = positions[index][1] - positions[index - 1][1]
        heading = yaws[index - 1]
        forward.append((math.cos(heading) * dx + math.sin(heading) * dy) / dt)
        lateral.append((-math.sin(heading) * dx + math.cos(heading) * dy) / dt)
        turn.append(math.remainder(yaws[index] - yaws[index - 1], math.tau) / dt)
    mean = lambda values: sum(values) / len(values)
    return {
        "fell": fell_at is not None, "fall_time_s": fell_at,
        "final_base_position_m": [round(v, 4) for v in positions[-1]],
        "min_base_height_m": round(min(heights), 4),
        "mean_base_height_m": round(mean(heights), 4),
        "second_half_forward_speed_m_s": round(mean(forward), 4),
        "second_half_lateral_speed_m_s": round(mean(lateral), 4),
        "second_half_yaw_rate_rad_s": round(mean(turn), 4),
        "forward_speed_error_m_s": round(mean(forward) - command[0], 4),
        "yaw_rate_error_rad_s": round(mean(turn) - command[2], 4),
    }
