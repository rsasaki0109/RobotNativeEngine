"""H1 v1: native/MuJoCo sphere drop, geometric quality audit and batched cost.

Run with pinned MuJoCo 3.9.0 and a release-built contact_drop example.
No claim of general solver superiority follows from this one-contact fixture.
"""
import argparse
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import subprocess
import time

import mujoco
import numpy as np

G = 9.81
RADIUS = 0.05
DURATION = 2.0
GATES = {"max_penetration_m": 1e-5, "settled_speed_m_s": 1e-3,
         "impact_impulse_relative_error": 0.01, "rebound_m": 1e-5,
         "energy_excess_fraction": 1e-8}


def native(binary, case, setting, audit):
    command = [str(binary), str(case["dt_ticks"]), str(case["mass_kg"]), str(case["gap_m"]),
               str(setting["regularization"]), str(setting["iterations"]), "audit" if audit else "timing"]
    return json.loads(subprocess.run(command, check=True, capture_output=True, text=True,
                                     timeout=120).stdout)


def mj(case, setting, audit):
    mass, height = case["mass_kg"], RADIUS + case["gap_m"]
    dt = case["dt_ticks"] * 1e-9
    inertia = 0.4 * mass * RADIUS**2
    solref = setting["solref"]
    xml = f'''<mujoco><option timestep="{dt}" gravity="0 -9.81 0" integrator="Euler"
        solver="Newton" iterations="{setting['iterations']}" tolerance="1e-10" cone="elliptic"/>
      <default><geom friction="0 0 0" condim="1" margin="0" gap="0"
        solref="{solref[0]} {solref[1]}" solimp="0.999 0.999 0.001 0.5 2"/></default>
      <worldbody><geom type="box" size="5 0.5 5" pos="0 -0.5 0"/>
        <body pos="0 {height} 0"><freejoint/>
          <inertial pos="0 0 0" mass="{mass}" diaginertia="{inertia} {inertia} {inertia}"/>
          <geom type="sphere" size="{RADIUS}"/></body></worldbody></mujoco>'''
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    steps = 2_000_000_000 // case["dt_ticks"]
    trace = [[0, case["gap_m"], 0, 0]] if audit else []
    force = np.zeros(6)
    start = time.perf_counter()
    if audit:
        for i in range(steps):
            mujoco.mj_step(model, data)
            impulse = 0.0
            for c in range(data.ncon):
                mujoco.mj_contactForce(model, data, c, force)
                impulse += force[0] * dt
            # qpos is post-step; contact.dist is pre-step and is not used here.
            trace.append([(i + 1) * dt, float(data.qpos[1] - RADIUS),
                          float(data.qvel[1]), float(impulse)])
    else:
        # One Python/C transition for the full batch, matching the native loop.
        mujoco.mj_step(model, data, nstep=steps)
    elapsed = time.perf_counter() - start
    return {"backend": "mujoco", "mode": "audit" if audit else "timing", "elapsed_s": elapsed,
            "warnings": [int(w.number) for w in data.warning], "simulation_time_s": float(data.time),
            "trace": trace, "final_gap_m": float(data.qpos[1] - RADIUS),
            "final_velocity_m_s": float(data.qvel[1]), "model_xml": xml,
            "settings": {**setting, "solver": "Newton", "integrator": "Euler", "tolerance": 1e-10,
                         "solimp": model.geom_solimp[0].tolist(), "condim": 1,
                         "margin_m": 0, "gap_m": 0, "disableflags": int(model.opt.disableflags)}}


def metrics(case, trace):
    """Audit post-step sphere/floor geometry; never substitute solver separation."""
    x = np.asarray(trace, dtype=float)
    dt = case["dt_ticks"] * 1e-9
    expected_steps = 2_000_000_000 // case["dt_ticks"]
    if x.shape != (expected_steps + 1, 4) or not np.isfinite(x).all():
        raise ValueError("invalid trace shape or nonfinite state")
    if not np.allclose(x[:, 0], np.arange(expected_steps + 1) * dt, atol=1e-12, rtol=0):
        raise ValueError("trace clock differs from requested timestep")
    if not np.allclose(x[0], [0, case["gap_m"], 0, 0], atol=1e-12, rtol=0):
        raise ValueError("initial condition differs")
    mass, height = case["mass_kg"], case["gap_m"]
    analytic_time = math.sqrt(2 * height / G)
    support = np.flatnonzero(x[:, 3] > mass * 1e-10)
    first = int(support[0]) if len(support) else None
    # End the impact window at the first small vertical speed at/after support,
    # within 100 ms of analytic impact; otherwise the impact gate fails.
    end = None
    if first is not None:
        eligible = np.flatnonzero((np.arange(len(x)) >= first) &
                                 (x[:, 0] <= analytic_time + 0.1) &
                                 (np.abs(x[:, 2]) <= GATES["settled_speed_m_s"]))
        if len(eligible):
            end = int(eligible[0])
    time_error = max(0.0, x[first, 0] - dt - analytic_time, analytic_time - x[first, 0]) if first is not None else None
    impulse_error = None
    if end is not None:
        # Subtract post-analytic-impact support gravity before comparing with
        # m sqrt(2gh), the ideal inelastic collision impulse.
        collision_impulse = float(x[:end + 1, 3].sum()) - mass * G * (x[end, 0] - analytic_time)
        ideal = mass * math.sqrt(2 * G * height)
        impulse_error = abs(collision_impulse - ideal) / ideal
    initial_energy = mass * G * height
    energy = mass * G * x[:, 1] + 0.5 * mass * x[:, 2]**2
    result = {"max_penetration_m": max(0.0, float(-x[:, 1].min())),
              "settled_speed_m_s": float(np.abs(x[x[:, 0] >= 1.5, 2]).max()),
              "first_support_time_s": float(x[first, 0]) if first is not None else None,
              "impact_time_error_s": float(time_error) if time_error is not None else None,
              "impact_end_time_s": float(x[end, 0]) if end is not None else None,
              "impact_impulse_relative_error": float(impulse_error) if impulse_error is not None else None,
              "rebound_m": max(0.0, float(x[end:, 1].max())) if end is not None else None,
              "energy_excess_fraction": max(0.0, float(energy.max() / initial_energy - 1)),
              "min_normal_impulse_n_s": float(x[:, 3].min())}
    passed = {key: result[key] is not None and result[key] <= bound for key, bound in GATES.items()}
    passed["impact_time"] = bool(time_error is not None and time_error <= dt)
    passed["nonnegative_normal_impulse"] = result["min_normal_impulse_n_s"] >= -mass * 1e-10
    result["gates"] = passed
    result["passed"] = all(passed.values())
    return result


def grid():
    for dt in (250_000, 500_000, 1_000_000, 2_000_000):
        for iterations in (20, 200):
            for reg in (0.0, 0.001):
                yield "native", dt, {"iterations": iterations, "regularization": reg}
            # Positive solref uses damping ratio one; direct stiffness uses
            # critical damping. MuJoCo refsafe remains enabled and is recorded.
            for solref in ([0.02, 1], [0.0002, 1], [-1e6, -2000], [-1e8, -20000]):
                yield "mujoco", dt, {"iterations": iterations, "solref": solref}


def run_case(binary, backend, case, setting, repeats):
    runner = (lambda audit: native(binary, case, setting, audit)) if backend == "native" else (
        lambda audit: mj(case, setting, audit))
    audit = runner(True)
    if backend == "native" and audit.get("debug_assertions", True):
        raise ValueError("use a release build for the cost audit")
    if (audit["final_gap_m"], audit["final_velocity_m_s"]) != tuple(audit["trace"][-1][1:3]):
        raise ValueError("audit final state is not its last sample")
    repeated = runner(True)
    if audit["trace"] != repeated["trace"]:
        raise ValueError("within-backend exact audit replay failed")
    runner(False)  # full-trajectory warm-up, fresh world each run
    runs = [runner(False) for _ in range(repeats)]
    for r in runs:
        if (r["final_gap_m"], r["final_velocity_m_s"]) != (audit["final_gap_m"], audit["final_velocity_m_s"]):
            raise ValueError("timing/audit final state differs")
    quality = metrics(case, audit["trace"])
    quality["gates"]["backend_health"] = not any(audit.get("warnings", [])) and abs(
        audit.get("simulation_time_s", DURATION) - DURATION) < 1e-8
    quality["passed"] = all(quality["gates"].values())
    times = [r["elapsed_s"] / DURATION for r in runs]
    return {"backend": backend, "case": case, "setting": setting, "audit": audit,
            "metrics": quality, "exact_replay": True,
            "seconds_per_simulated_second": times, "median_cost": statistics.median(times),
            "min_cost": min(times), "max_cost": max(times)}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("binary", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--repeats", type=int, default=5)
    p.add_argument("--require-native-pass", action="store_true",
                   help="CI gate: a nominal native setting and its four holdouts must pass")
    args = p.parse_args()
    if args.output.exists() or args.repeats < 5:
        p.error("provide a new output path and at least five repeats")
    if mujoco.__version__ != "3.9.0":
        p.error("this protocol pins MuJoCo 3.9.0")
    binary = args.binary.resolve()
    # Pin both runners to the same logical CPU; native subprocesses inherit it.
    os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    report = {"protocol": "hard_contact_drop_h1_v1", "gates": GATES,
              "radius_m": RADIUS, "duration_s": DURATION, "gravity_m_s2": [0, -G, 0],
              "requested_friction": 0, "requested_restitution": 0,
              "timing_scope": "backend batch step loop; no IPC, traces, model creation or ECS sync; not end-to-end",
              "platform": platform.platform(), "python": platform.python_version(),
              "mujoco": mujoco.__version__, "cpu_affinity": sorted(os.sched_getaffinity(0)),
              "binary_sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
              "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "native_build_requirement": "cargo build --release; timing is backend-loop cost only",
              "results": [], "holdout": []}
    for backend, dt, setting in grid():
        r = run_case(binary, backend, {"dt_ticks": dt, "mass_kg": 1, "gap_m": 0.1}, setting, args.repeats)
        report["results"].append(r)
        print(json.dumps({"backend": backend, "dt_ticks": dt, "setting": setting,
                          "metrics": r["metrics"], "median_cost": r["median_cost"]}), flush=True)
    report["selected"] = {}
    for backend in ("native", "mujoco"):
        passing = [r for r in report["results"] if r["backend"] == backend and r["metrics"]["passed"]]
        if not passing:
            report["selected"][backend] = None
            continue
        best = min(passing, key=lambda r: r["median_cost"])
        report["selected"][backend] = {"dt_ticks": best["case"]["dt_ticks"], "setting": best["setting"]}
        for mass in (0.1, 10):
            for height in (0.01, 0.5):
                case = {"dt_ticks": best["case"]["dt_ticks"], "mass_kg": mass, "gap_m": height}
                report["holdout"].append(run_case(binary, backend, case, best["setting"], args.repeats))
    raw = (json.dumps(report, allow_nan=False) + "\n").encode()
    with args.output.open("xb") as f:
        f.write(gzip.compress(raw, mtime=0) if args.output.suffix == ".gz" else raw)
    native_holdouts = [r for r in report["holdout"] if r["backend"] == "native"]
    if args.require_native_pass and (len(native_holdouts) != 4 or
                                     not all(r["metrics"]["passed"] for r in native_holdouts)):
        raise SystemExit("native H1 quality gate failed; failure traces retained in output")


if __name__ == "__main__":
    main()
