#!/usr/bin/env python3
"""The sliding edge strike of `mujoco_strike.py`, in Genesis.

The same 2 kg box, pitched 0.2 rad nose-down, slides forward at 2.6 m/s
and comes down at 0.8 m/s onto its front bottom edge, a 1 mm sphere, with no
gravity, for 50 ms at a 1 ms step. The impulse is the box's momentum change,
against the rigid closed form `mujoco_strike.closed_form`. Genesis on the
CPU; it logs to stdout, so the report goes to `--output`.
"""

import argparse
import json
import math
import tempfile
from pathlib import Path

import numpy as np

import genesis as gs
import mujoco_strike as strike


def numpy(value):
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value, dtype=float).reshape(-1)


def box_urdf():
    """One link: the box's mass and inertia, and the edge sphere."""
    ixx, iyy, izz = strike.inertia()
    hx, _, hz = strike.HALF_M
    edge = (hx, 0.0, -hz + strike.RADIUS_M)
    return f"""<?xml version="1.0"?>
<robot name="strike_box">
  <link name="box">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="{strike.MASS_KG}"/>
      <inertia ixx="{ixx}" ixy="0" ixz="0" iyy="{iyy}" iyz="0" izz="{izz}"/>
    </inertial>
    <collision>
      <origin xyz="{edge[0]} {edge[1]} {edge[2]}" rpy="0 0 0"/>
      <geometry><sphere radius="{strike.RADIUS_M}"/></geometry>
    </collision>
  </link>
</robot>
"""


def run(mu):
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=strike.STEP_S, substeps=1, gravity=(0.0, 0.0, 0.0)),
        show_viewer=False)
    material = gs.materials.Rigid(friction=mu)
    scene.add_entity(gs.morphs.Plane(), material=material)
    urdf = Path(tempfile.mkdtemp()) / "strike_box.urdf"
    urdf.write_text(box_urdf())
    half = 0.5 * strike.PITCH_RAD
    quat = (math.cos(half), 0.0, math.sin(half), 0.0)
    # The edge's sphere bottom on the floor: its centre sits RADIUS above the
    # box's bottom edge, which the pitch lowers by hx sin + hz cos.
    hx, _, hz = strike.HALF_M
    drop = hx * math.sin(strike.PITCH_RAD) + hz * math.cos(strike.PITCH_RAD)
    height = drop - strike.RADIUS_M * math.cos(strike.PITCH_RAD) + strike.RADIUS_M
    box = scene.add_entity(gs.morphs.URDF(file=str(urdf), pos=(0.0, 0.0, height), quat=quat,
                                          fixed=False, default_armature=None),
                           material=material)
    scene.build()
    velocity = np.array([*strike.VELOCITY_M_S, 0.0, 0.0, 0.0])
    box.set_dofs_velocity(velocity, list(range(6)))
    before = numpy(box.get_vel())
    for _ in range(round(strike.DURATION_S / strike.STEP_S)):
        scene.step()
    change = (numpy(box.get_vel()) - before) * strike.MASS_KG
    return {"tangential_n_s": float(change[0]), "normal_n_s": float(change[2]),
            "mass_kg": float(numpy(box.get_mass()).sum()) if hasattr(box, "get_mass") else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    gs.init(backend=gs.cpu, precision="64", logging_level="error")
    report = {"engine": f"genesis {getattr(gs, '__version__', '?')}", "cases": []}
    for mu in (0.0, 1.0):
        # Genesis may reject a friction of zero; then the least it takes.
        try:
            used, result = mu, run(mu)
        except Exception as error:  # noqa: BLE001 - reported, then retried
            report.setdefault("notes", []).append(f"friction {mu}: {error}")
            used = 0.01
            result = run(used)
        report["cases"].append({"friction": used, "closed_form": strike.closed_form(used),
                                "genesis": result})
    arguments.output.write_text(json.dumps(report, indent=1) + "\n")


if __name__ == "__main__":
    main()
