"""H2 v1 native box diagnostic: frozen gates, full poses, no MuJoCo speed claim."""
import argparse
import hashlib
import itertools
import json
import lzma
import os
from pathlib import Path
import platform
import statistics
import subprocess

import numpy as np

HALF = 0.05
G = 9.81
GATES = {"max_penetration_m": 1e-5, "settled_linear_speed_m_s": 1e-3,
         "settled_angular_speed_rad_s": 1e-2, "energy_excess_fraction": 1e-6,
         "settled_support_relative_error": 0.01}
COLUMNS = ["time_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw",
           "vx_m_s", "vy_m_s", "vz_m_s", "wx_rad_s", "wy_rad_s", "wz_rad_s", "normal_impulse_n_s"]


def corners(trace):
    """Transform all eight cube vertices, independently of collision manifolds."""
    x = np.asarray(trace, dtype=float)
    vertices = np.array(list(itertools.product((-HALF, HALF), repeat=3)))
    qv, qw = x[:, None, 4:7], x[:, None, 7:8]
    t = 2 * np.cross(qv, vertices[None, :, :])
    return x[:, None, 1:4] + vertices[None, :, :] + qw * t + np.cross(qv, t)


def metrics(case, trace):
    x = np.asarray(trace, dtype=float)
    dt = case["dt_ticks"] * 1e-9
    steps = 2_000_000_000 // case["dt_ticks"]
    if x.shape != (steps + 1, 15) or not np.isfinite(x).all():
        raise ValueError("invalid trace shape or nonfinite state")
    if not np.allclose(x[:, 0], np.arange(steps + 1) * dt, atol=1e-12, rtol=0):
        raise ValueError("invalid clock")
    if not np.allclose(np.linalg.norm(x[:, 4:8], axis=1), 1, atol=1e-10, rtol=0):
        raise ValueError("invalid quaternion")
    angle = case["angle_rad"]
    initial = np.zeros(15)
    initial[2] = HALF * (np.cos(angle) + np.sin(angle)) + case["gap_m"]
    initial[6:8] = [np.sin(angle / 2), np.cos(angle / 2)]
    if not np.allclose(x[0], initial, atol=1e-12, rtol=0):
        raise ValueError("initial condition differs")
    gaps = corners(x)[:, :, 1].min(axis=1)
    speed = np.linalg.norm(x[:, 8:11], axis=1)
    spin = np.linalg.norm(x[:, 11:14], axis=1)
    mass = case["mass_kg"]
    inertia = mass * (2 * HALF)**2 / 6
    energy = mass * G * x[:, 2] + 0.5 * mass * speed**2 + 0.5 * inertia * spin**2
    # Half-open 0.5 s window: impulses belong to intervals ending at each sample.
    settled = x[:, 0] > 1.5 + 1e-12
    result = {"max_penetration_m": max(0.0, float(-gaps.min())),
              "settled_linear_speed_m_s": float(speed[settled].max()),
              "settled_angular_speed_rad_s": float(spin[settled].max()),
              "energy_excess_fraction": max(0.0, float(energy.max() / energy[0] - 1)),
              "settled_support_relative_error": abs(float(x[settled, 14].sum()) / (mass * G * 0.5) - 1),
              "final_gap_m": float(gaps[-1]),
              "final_near_floor_vertices": int((np.abs(corners(x[-1:])[0, :, 1]) <= 1e-5).sum())}
    result["gates"] = {k: result[k] <= limit for k, limit in GATES.items()}
    result["gates"]["nonnegative_normal_impulse"] = bool(x[:, 14].min() >= -mass * 1e-10)
    result["passed"] = all(result["gates"].values())
    return result


def run(binary, case, setting, audit):
    command = [str(binary), str(case["dt_ticks"]), str(case["mass_kg"]), str(case["gap_m"]),
               str(setting["regularization"]), str(setting["iterations"]),
               "audit" if audit else "timing", str(case["angle_rad"])]
    return json.loads(subprocess.run(command, check=True, capture_output=True, text=True, timeout=120).stdout)


def run_case(binary, case, setting):
    audit = run(binary, case, setting, True)
    if audit["debug_assertions"]:
        raise ValueError("release build required")
    if audit["trace"] != run(binary, case, setting, True)["trace"]:
        raise ValueError("exact replay differs")
    if audit["trace"][-1][1:14] != audit["final_state"][1:14]:
        raise ValueError("audit final state differs")
    run(binary, case, setting, False)
    costs = []
    for _ in range(5):
        timed = run(binary, case, setting, False)
        if timed["final_state"] != audit["final_state"]:
            raise ValueError("timing final state differs")
        costs.append(timed["elapsed_s"] / 2)
    return {"case": case, "setting": setting, "audit": audit,
            "metrics": metrics(case, audit["trace"]), "exact_replay": True,
            "seconds_per_simulated_second": costs, "median_cost": statistics.median(costs)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("provide a new output path")
    binary = args.binary.resolve()
    os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    report = {"protocol": "box_contact_h2_v1", "gates": GATES, "columns": COLUMNS,
              "half_extents_m": [HALF] * 3, "duration_s": 2, "gravity_m_s2": [0, -G, 0],
              "friction": 0, "restitution": 0, "status": "running",
              "platform": platform.platform(), "cpu_affinity": sorted(os.sched_getaffinity(0)),
              "binary_sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
              "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "timing_scope": "backend batch only, excluding ECS writeback, trace, setup and IPC",
              "results": []}
    for dt, angle, gap, reg, iterations in itertools.product(
            (1_000_000, 2_000_000), (0, 0.1, 0.4), (0.1, 0.5), (0.0, 0.001), (20, 200)):
        case = {"dt_ticks": dt, "angle_rad": angle, "gap_m": gap, "mass_kg": 1}
        result = run_case(binary, case, {"regularization": reg, "iterations": iterations})
        report["results"].append(result)
        print(json.dumps({"case": case, "setting": result["setting"], "metrics": result["metrics"]}), flush=True)
    report["status"] = "complete"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with lzma.open(args.output, "xt") as stream:
        json.dump(report, stream, allow_nan=False)


if __name__ == "__main__":
    main()
