#!/usr/bin/env python3
"""MuJoCo side of example 146, and the comparison with the closed forms.

Reads the scenarios from example 146's report and builds each in MuJoCo:
- a floor plane;
- every body as a free body with one box or sphere geom of the same mass (MuJoCo
  gives it the solid shape's inertia, as RNE's report does);
- the same friction and gravity (an incline tilts gravity, as in RNE);
- MuJoCo's default contact parameters and pyramidal cone unless
  `--cone elliptic`.

It runs every scenario at the same 2 ms step (or `--timestep-divisor K` times
shorter) and summarizes every physics step as the example does. `--impratio`
and `--noslip-iterations` set those MuJoCo options. MuJoCo has no restitution
coefficient, so scenarios with restitution above 0 are RNE-only. It prints,
per scenario, the closed form, RNE's value and MuJoCo's.

`--json OUT` writes MuJoCo's summaries and the comparison. Uses mujoco and
numpy.

Usage: mujoco_contact_closed_form.py RNE_REPORT.json [--cone elliptic]
       [--impratio X] [--noslip-iterations N] [--timestep-divisor K]
       [--json OUT.json]
"""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

LOAD_WINDOW_S = 0.5
STOP_SPEED_M_S = 0.01


def geom(body, friction):
    shape = body["shape"]
    if shape["kind"] == "box":
        size = " ".join(str(h) for h in shape["half_extents_m"])
        kind = "box"
    else:
        size, kind = str(shape["radius_m"]), "sphere"
    return (f'<geom type="{kind}" size="{size}" mass="{body["mass_kg"]}" '
            f'friction="{friction} 0.005 0.0001"/>')


def mjcf(scenario, cone):
    friction = scenario["friction"]
    gravity = " ".join(str(g) for g in scenario["gravity_m_s2"])
    bodies = "\n".join(
        f'<body name="body{index}" pos="{" ".join(str(p) for p in body["position_m"])}">'
        f'<freejoint/>{geom(body, friction)}</body>'
        for index, body in enumerate(scenario["bodies"]))
    return f"""
<mujoco>
  <option timestep="{scenario["dt_s"]}" gravity="{gravity}" cone="{cone}"/>
  <worldbody>
    <geom name="floor" type="plane" size="20 20 0.1" friction="{friction} 0.005 0.0001"/>
    {bodies}
  </worldbody>
</mujoco>"""


def run(scenario, options):
    """Every physics step's record, the initial state first."""
    model = mujoco.MjModel.from_xml_string(mjcf(scenario, options.cone))
    model.opt.timestep = scenario["dt_s"] / options.timestep_divisor
    model.opt.impratio = options.impratio
    model.opt.noslip_iterations = options.noslip_iterations
    data = mujoco.MjData(model)
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"body{index}")
           for index in range(len(scenario["bodies"]))]
    for index, body in enumerate(scenario["bodies"]):
        address = model.jnt_dofadr[model.body_jntadr[ids[index]]]
        data.qvel[address:address + 3] = body["velocity_m_s"]
    mujoco.mj_forward(model, data)
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    stacked = set(ids[:2]) if len(ids) > 1 else set()

    def state(time_s):
        velocity, wrench = np.zeros(6), np.zeros(6)
        record = {"time_s": time_s, "position_m": [], "velocity_m_s": [], "spin_rad_s": [],
                  "ground_load_n": 0.0, "stack_load_n": 0.0}
        for body in ids:
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body, velocity, 0)
            record["position_m"].append(data.xpos[body].tolist())
            record["spin_rad_s"].append(velocity[:3].tolist())
            record["velocity_m_s"].append(velocity[3:].tolist())
        for index in range(data.ncon):
            contact = data.contact[index]
            mujoco.mj_contactForce(model, data, index, wrench)
            pair = {contact.geom1, contact.geom2}
            if floor in pair:
                record["ground_load_n"] += wrench[0]
            elif {model.geom_bodyid[g] for g in pair} == stacked:
                record["stack_load_n"] += wrench[0]
        return record

    records = [state(0.0)]
    records[0]["ground_load_n"] = records[0]["stack_load_n"] = 0.0
    dt = model.opt.timestep
    for step in range(round(scenario["duration_s"] / dt)):
        mujoco.mj_step(model, data)
        records.append(state((step + 1) * dt))
    return records


def summarize(scenario, steps):
    """As scenario.rs's summarize."""
    first, last = steps[0], steps[-1]
    moved = lambda axis: last["position_m"][0][axis] - first["position_m"][0][axis]
    check = scenario["check"]
    kind = check["kind"]
    if kind == "slide":
        sin, cos = np.sin(check["direction_rad"]), np.cos(check["direction_rad"])
        dx, dy = moved(0), moved(1)
        stop = next((s["time_s"] for s in steps[1:]
                     if np.hypot(*s["velocity_m_s"][0][:2]) < STOP_SPEED_M_S), None)
        return {"distance_m": dx * cos + dy * sin, "lateral_m": abs(dy * cos - dx * sin),
                "stop_s": stop}
    if kind == "incline":
        return {"displacement_m": moved(0)}
    if kind == "roll":
        speed, spin = last["velocity_m_s"][0][0], last["spin_rad_s"][0][1]
        return {"displacement_m": moved(0), "speed_m_s": speed, "spin_rad_s": spin,
                "slip_m_s": speed - spin * last["position_m"][0][2]}
    if kind == "bounce":
        radius = scenario["bodies"][0]["shape"]["radius_m"]
        falling = next((i for i, s in enumerate(steps) if s["velocity_m_s"][0][2] < -0.1),
                       len(steps))
        impact = next((falling + i for i, s in enumerate(steps[falling:])
                       if s["velocity_m_s"][0][2] >= 0.0), None)
        apex = None if impact is None else max(
            s["position_m"][0][2] - radius for s in steps[impact:])
        return {"apex_m": apex}
    window = [s for s in steps if s["time_s"] > scenario["duration_s"] - LOAD_WINDOW_S + 1e-9]
    mean = lambda key: sum(s[key] for s in window) / len(window)
    return {"top_sink_m": -moved(2), "top_drift_m": float(np.hypot(moved(0), moved(1))),
            "ground_load_n": mean("ground_load_n"), "stack_load_n": mean("stack_load_n")}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("rne_report", type=Path)
    parser.add_argument("--cone", choices=["pyramidal", "elliptic"], default="pyramidal")
    parser.add_argument("--impratio", type=float, default=1.0)
    parser.add_argument("--noslip-iterations", type=int, default=0)
    parser.add_argument("--timestep-divisor", type=int, default=1,
                        help="physics steps this many times shorter than RNE's")
    parser.add_argument("--json", type=Path)
    arguments = parser.parse_args()
    rne = json.loads(arguments.rne_report.read_text())
    rows, results = [], []
    fmt = lambda v: "—" if v is None else f"{v:.4f}"
    for entry in rne["results"]:
        scenario = entry["scenario"]
        print(f"{scenario['name']}: {scenario['what']}")
        summary = None
        if scenario["restitution"] == 0.0:
            summary = summarize(scenario, run(scenario, arguments))
            results.append({"scenario": scenario, "summary": summary})
        for metric, rne_value in entry["summary"].items():
            exact = entry["closed_form"].get(metric)
            mujoco_value = None if summary is None else summary[metric]
            print(f"  {metric:16s} closed form {fmt(exact):>9s}  RNE {fmt(rne_value):>9s}  "
                  f"MuJoCo {fmt(mujoco_value) if summary else 'n/a':>9s}")
            rows.append({"scenario": scenario["name"], "metric": metric, "closed_form": exact,
                         "rne": rne_value, "mujoco": mujoco_value})
    if arguments.json:
        arguments.json.write_text(json.dumps(
            {"schema_version": 1, "engine": f"mujoco {mujoco.__version__}",
             "options": {"cone": arguments.cone, "impratio": arguments.impratio,
                         "noslip_iterations": arguments.noslip_iterations,
                         "timestep_divisor": arguments.timestep_divisor},
             "rne_settings": rne["settings"], "results": results, "comparison": rows},
            indent=2) + "\n")


if __name__ == "__main__":
    main()
