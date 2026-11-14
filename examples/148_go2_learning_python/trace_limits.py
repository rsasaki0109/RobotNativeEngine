"""How hard a traced walk drives the Go2's actuators: joint speeds and PD
torques against the URDF's limits, and the base's speed.

Reads a trace written by `walk_policy.py --trace` (RNE) or by
`mujoco_go2_learning.py evaluate --trace` (MuJoCo). Both record the
joints' states and targets as each control period begins, so the torque
here is the PD law's at that instant: kp (target - q) - kd q', clamped to
the effort limit. Neither simulator enforces the velocity limits.

    python examples/148_go2_learning_python/trace_limits.py trace-rne.json trace-mujoco.json
"""

import json
import sys

# The PD gains of example 147's plant, and the control period.
KP_NM_PER_RAD, KD_NM_S_PER_RAD, CONTROL_DT_S = 20.0, 0.5, 0.02
# go2_description.rne.urdf: hip, thigh, calf of each leg.
EFFORT_NM = (23.7, 23.7, 45.43)
VELOCITY_RAD_S = (30.1, 30.1, 15.7)
JOINTS = ("hip", "thigh", "calf")


def summarize(path):
    trace = json.load(open(path))
    periods = trace["periods"][trace.get("settle_periods", 0):]
    by_joint = {name: {"speeds": [], "saturated": 0, "over_speed": 0, "power": []}
                for name in JOINTS}
    samples = 0
    for period in periods:
        for index in range(12):
            kind = index % 3
            entry = by_joint[JOINTS[kind]]
            speed = period["joint_velocities_rad_s"][index]
            raw = (KP_NM_PER_RAD * (period["targets_rad"][index] - period["joints_rad"][index])
                   - KD_NM_S_PER_RAD * speed)
            torque = max(-EFFORT_NM[kind], min(EFFORT_NM[kind], raw))
            entry["speeds"].append(abs(speed))
            entry["saturated"] += abs(raw) >= EFFORT_NM[kind]
            entry["over_speed"] += abs(speed) > VELOCITY_RAD_S[kind]
            entry["power"].append(torque * speed)
        samples += 1
    xs = [period["position_m"][0] for period in periods]
    report = {"trace": path, "engine": trace.get("engine"), "periods": samples,
              "mean_speed_m_s": (xs[-1] - xs[0]) / (CONTROL_DT_S * max(1, samples - 1)),
              "joints": {}}
    for kind, name in enumerate(JOINTS):
        entry, count = by_joint[name], 4 * samples
        speeds = sorted(entry["speeds"])
        report["joints"][name] = {
            "velocity_limit_rad_s": VELOCITY_RAD_S[kind],
            "max_speed_rad_s": speeds[-1],
            "p95_speed_rad_s": speeds[int(0.95 * (len(speeds) - 1))],
            "over_velocity_limit": entry["over_speed"] / count,
            "torque_saturated": entry["saturated"] / count,
            "peak_power_w": max(entry["power"]),
        }
    # Positive joint power summed over the twelve joints, per period.
    totals = [sum(max(0.0, by_joint[JOINTS[i % 3]]["power"][period * 4 + i // 3])
                  for i in range(12)) for period in range(samples)]
    report["mean_positive_power_w"] = sum(totals) / max(1, samples)
    return report


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    print(json.dumps([summarize(path) for path in sys.argv[1:]], indent=1))


if __name__ == "__main__":
    main()
