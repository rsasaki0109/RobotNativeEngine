#!/usr/bin/env python3
"""MuJoCo side of example 145, and the comparison with Coulomb's law.

Reads the block and the scenarios from example 145's report, builds the same
block in MuJoCo (a free body of the same mass and principal inertia on four
5 mm sphere geoms, a floor plane, the same friction, MuJoCo's default contact
parameters and pyramidal cone unless `--cone elliptic`), runs every scenario
at the same 2 ms step (or `--timestep-divisor K` times shorter) with the push
applied by `mj_applyFT` at the same block-fixed point, and summarizes every
physics step as the example does. `--impratio` and `--noslip-iterations` set
those MuJoCo options. It then prints, per
scenario, the closed-form answer where one exists, RNE's and MuJoCo's:

- stick: no slide; static friction F at the contact plane shifts load
  forward by F (c + r) / a, where c + r is the depth of the contact plane
  below the centre of mass and a the half-spacing of the spheres;
- slide: (F/m - mu g) T of speed after a push of T, a slide of half of that
  times T plus v^2 / (2 mu g), a stop v / (mu g) after the push, and a
  forward load shift of mu m g (c + r) / a;
- rock: the rigid-pivot rotation about the front contact line, integrated
  with the force and gravity torques until the block falls back, gives the
  largest pitch.

`--json OUT` writes MuJoCo's report (the same layout as RNE's) and the
comparison. Uses mujoco and numpy.

Usage: mujoco_contact_slide.py RNE_REPORT.json [--cone elliptic] [--impratio X]
       [--noslip-iterations N] [--timestep-divisor K] [--json OUT.json]
"""
import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np

SAMPLE_EVERY = 5


def mjcf(block, scenario, cone):
    friction = scenario["friction"]
    ixx, iyy, izz = block["principal_inertia_kg_m2"]
    spheres = "\n".join(
        f'<geom type="sphere" size="{block["sphere_radius_m"]}" pos="{x} {y} {z}" '
        f'friction="{friction} 0.005 0.0001"/>' for x, y, z in block["sphere_centres_m"])
    return f"""
<mujoco>
  <option timestep="{scenario["dt_s"]}" gravity="0 0 {-block["gravity_m_s2"]}" cone="{cone}"/>
  <worldbody>
    <geom name="floor" type="plane" size="20 20 0.1" friction="{friction} 0.005 0.0001"/>
    <body name="block">
      <freejoint/>
      <inertial pos="0 0 0" mass="{block["mass_kg"]}" diaginertia="{ixx} {iyy} {izz}"/>
      {spheres}
    </body>
  </worldbody>
</mujoco>"""


def initial_state(block, scenario):
    """Centre-of-mass position, quaternion (w, x, y, z) and forward speed."""
    start = scenario["start"]
    radius = block["sphere_radius_m"]
    if start["kind"] == "rest":
        depth = -min(z for _, _, z in block["sphere_centres_m"])
        return np.array([0.0, 0.0, depth + radius]), np.array([1.0, 0.0, 0.0, 0.0]), 0.0
    pitch = start["pitch_rad"]
    quat = np.array([math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0])
    rotation = np.zeros(9)
    mujoco.mju_quat2Mat(rotation, quat)
    rotation = rotation.reshape(3, 3)
    lowest = min((rotation @ np.array(c))[2] for c in block["sphere_centres_m"])
    height = start["drop_m"] + radius - lowest
    return np.array([0.0, 0.0, height]), quat, start["forward_m_s"]


def run(block, scenario, options):
    """Every physics step's record (time, position, pitch, speed, loads)."""
    model = mujoco.MjModel.from_xml_string(mjcf(block, scenario, options.cone))
    model.opt.timestep = scenario["dt_s"] / options.timestep_divisor
    model.opt.impratio = options.impratio
    model.opt.noslip_iterations = options.noslip_iterations
    data = mujoco.MjData(model)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "block")
    position, quat, forward = initial_state(block, scenario)
    data.qpos[:3], data.qpos[3:7], data.qvel[0] = position, quat, forward
    mujoco.mj_forward(model, data)
    dt = model.opt.timestep
    steps = round(scenario["end_s"] / dt)
    pushed = scenario["push_start_s"] is not None
    first = round(scenario["push_start_s"] / dt) if pushed else 0
    push_steps = range(first, first + round(scenario["push_duration_s"] / dt)) if pushed else ()
    force = np.array([scenario["force_n"], 0.0, 0.0])
    lever = np.array([0.0, 0.0, scenario["force_height_m"]])
    records, wrench = [], np.zeros(6)
    for step in range(steps):
        data.qfrc_applied[:] = 0.0
        if step in push_steps:
            point = data.xpos[body] + data.xmat[body].reshape(3, 3) @ lever
            mujoco.mj_applyFT(model, data, force, np.zeros(3), point, body, data.qfrc_applied)
        mujoco.mj_step(model, data)
        front = back = 0.0
        for index in range(data.ncon):
            mujoco.mj_contactForce(model, data, index, wrench)
            if data.contact[index].pos[0] > data.xpos[body][0]:
                front += wrench[0]
            else:
                back += wrench[0]
        x_axis = data.xmat[body].reshape(3, 3)[:, 0]
        records.append({"time_s": (step + 1) * dt, "position_m": data.xpos[body].tolist(),
                        "pitch_rad": math.atan2(-x_axis[2], x_axis[0]),
                        "forward_m_s": float(data.qvel[0]), "front_normal_n": front,
                        "back_normal_n": back})
    return records


WINDOW_S = 0.3


def summarize(scenario, steps):
    """As scenario.rs's summarize, from every physics step."""
    pushed = scenario["push_start_s"] is not None
    push_start = scenario["push_start_s"] if pushed else 0.0
    push_end = push_start + scenario["push_duration_s"]
    at = lambda t: min(steps, key=lambda s: abs(s["time_s"] - t))
    last = steps[-1]
    after = [s for s in steps if s["time_s"] > push_end + 1e-9]
    stop = next((s["time_s"] - push_end for s in after if abs(s["forward_m_s"]) < 0.01), None)
    during = [s for s in steps if push_start + 0.02 + 1e-9 < s["time_s"] <= push_end + 1e-9]
    mean = lambda key: sum(s[key] for s in during) / len(during) if during else None
    peak = max(steps, key=lambda s: abs(s["pitch_rad"]))
    flat = None
    if abs(peak["pitch_rad"]) > 0.05:
        flat = next((s["time_s"] for s in steps
                     if s["time_s"] > peak["time_s"] and abs(s["pitch_rad"]) < 0.01), None)
    window_start = flat if abs(peak["pitch_rad"]) > 0.05 else push_start
    window = [] if window_start is None else [
        s for s in steps if window_start + 1e-9 < s["time_s"] <= window_start + WINDOW_S + 1e-9]
    pitches = [s["pitch_rad"] for s in steps]
    return {"slide_m": last["position_m"][0] - (at(push_start)["position_m"][0] if pushed else 0.0),
            "speed_after_push_m_s": at(push_end)["forward_m_s"], "stop_after_s": stop,
            "max_pitch_rad": max(pitches), "min_pitch_rad": min(pitches),
            "final_pitch_rad": last["pitch_rad"], "flat_at_s": flat,
            "push_front_normal_n": mean("front_normal_n"),
            "push_back_normal_n": mean("back_normal_n"),
            "airborne_share": (sum(s["front_normal_n"] + s["back_normal_n"] == 0.0 for s in window)
                               / len(window)) if window else None,
            "hop_m": hop(window) if window else None}


def hop(window):
    """The largest rise of the centre of mass above its lowest point so far."""
    lowest, largest = math.inf, 0.0
    for sample in window:
        lowest = min(lowest, sample["position_m"][2])
        largest = max(largest, sample["position_m"][2] - lowest)
    return largest


def rigid_pivot_max_pitch(block, scenario):
    """Largest pitch of the block rotating about its front contact line."""
    m, g = block["mass_kg"], block["gravity_m_s2"]
    centres = block["sphere_centres_m"]
    a = max(x for x, _, _ in centres)
    depth = -min(z for _, _, z in centres) + block["sphere_radius_m"]
    com = np.array([-a, depth])  # (x, z) from the pivot
    point = com + np.array([0.0, scenario["force_height_m"]])
    inertia = block["principal_inertia_kg_m2"][1] + m * com @ com
    rotate = lambda v, t: np.array([v[0] * math.cos(t) + v[1] * math.sin(t),
                                    -v[0] * math.sin(t) + v[1] * math.cos(t)])
    dt, theta, omega, time, largest = 1e-5, 0.0, 0.0, 0.0, 0.0
    while True:
        torque = m * g * rotate(com, theta)[0]
        if time < scenario["push_duration_s"]:
            torque += scenario["force_n"] * rotate(point, theta)[1]
        omega += torque / inertia * dt
        theta += omega * dt
        time += dt
        if theta <= 0.0 and time > 0.0:
            return largest
        largest = max(largest, theta)


def closed_form(block, scenario):
    m, g, mu = block["mass_kg"], block["gravity_m_s2"], scenario["friction"]
    centres = block["sphere_centres_m"]
    a = max(x for x, _, _ in centres)
    depth = -min(z for _, _, z in centres) + block["sphere_radius_m"]
    weight, force, push = m * g, scenario["force_n"], scenario["push_duration_s"]
    name = scenario["name"]
    if scenario["start"]["kind"] != "rest":
        return {}
    if scenario["force_height_m"] > 0:
        return {"max_pitch_rad": rigid_pivot_max_pitch(block, scenario),
                "push_back_normal_n": 0.0}
    if force <= mu * weight:
        shift = force * depth / a
        return {"slide_m": 0.0, "push_front_normal_n": (weight + shift) / 2,
                "push_back_normal_n": (weight - shift) / 2}
    speed = (force / m - mu * g) * push
    shift = mu * weight * depth / a
    return {"slide_m": speed * push / 2 + speed ** 2 / (2 * mu * g),
            "speed_after_push_m_s": speed, "stop_after_s": speed / (mu * g),
            "push_front_normal_n": (weight + shift) / 2,
            "push_back_normal_n": (weight - shift) / 2}


METRICS = ["slide_m", "speed_after_push_m_s", "stop_after_s", "max_pitch_rad", "min_pitch_rad",
           "flat_at_s", "push_front_normal_n", "push_back_normal_n", "airborne_share", "hop_m"]


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
    block, rows, results = rne["block"], [], []
    fmt = lambda v: "—" if v is None else f"{v:.4f}"
    for entry in rne["results"]:
        scenario = entry["scenario"]
        steps = run(block, scenario, arguments)
        summary = summarize(scenario, steps)
        exact = closed_form(block, scenario)
        every = SAMPLE_EVERY * arguments.timestep_divisor
        results.append({"scenario": scenario, "summary": summary,
                        "samples": steps[every - 1::every]})
        print(f"{scenario['name']}: {scenario['what']}")
        for metric in METRICS:
            values = (exact.get(metric), entry["summary"][metric], summary[metric])
            if all(v is None for v in values[1:]):
                continue
            print(f"  {metric:22s} closed form {fmt(values[0]):>9s}  RNE {fmt(values[1]):>9s}  "
                  f"MuJoCo {fmt(values[2]):>9s}")
            rows.append({"scenario": scenario["name"], "metric": metric,
                         "closed_form": values[0], "rne": values[1], "mujoco": values[2]})
    if arguments.json:
        arguments.json.write_text(json.dumps(
            {"schema_version": 1, "engine": f"mujoco {mujoco.__version__}",
             "options": {"cone": arguments.cone, "impratio": arguments.impratio,
                         "noslip_iterations": arguments.noslip_iterations,
                         "timestep_divisor": arguments.timestep_divisor},
             "rne_settings": rne["settings"], "block": block, "results": results,
             "comparison": rows}, indent=2) + "\n")


if __name__ == "__main__":
    main()
