#!/usr/bin/env python3
"""A sliding edge strike in MuJoCo, against its closed form.

The Go2's head and shins strike the floor while the robot runs forward, and
where they do MuJoCo's impulse is larger than RNE's. Without friction the two
agree, so the difference lies in how friction couples into the strike. This
is the smallest such case, the one `rne_dynamics`'
`a_sliding_edge_strike_takes_the_coulomb_impulse` test checks RNE against: a
2 kg box, 0.2 m long, 0.1 m tall and 0.2 m wide, pitched 0.2 rad nose-down,
slides forward at 2.6 m/s and comes down at 0.8 m/s onto its front bottom
edge, with no gravity. A rigid impact without restitution, sliding
throughout, gives the edge the normal impulse

    P_n = -u_n / (1/m + r_x^2/I + mu r_x r_y / I)

(r the edge's offset from the centre of mass, I the pitch inertia), against
the slip P_t = -mu P_n. Friction ahead of and below the centre of mass pitches
the box further into the floor, so the impulse exceeds the frictionless one.

The edge is a 1 mm sphere. Each setting runs 50 ms at a 1 ms step; the
impulse is the box's momentum change. Prints JSON.
"""

import argparse
import json
import math

import mujoco
import numpy as np

MASS_KG = 2.0
HALF_M = (0.1, 0.1, 0.05)  # x forward, y across, z up
PITCH_RAD = 0.2  # nose down
VELOCITY_M_S = (2.6, 0.0, -0.8)
RADIUS_M = 0.001
STEP_S = 0.001
DURATION_S = 0.05

SETTINGS = {
    "default": {},
    "stiff": {"solref": (0.004, 1.0)},
    "stiff, damped 5x": {"solref": (0.004, 5.0)},
    "stiff, pyramidal": {"solref": (0.004, 1.0), "cone": "pyramidal", "impratio": 1.0},
    "stiff, impratio 1": {"solref": (0.004, 1.0), "impratio": 1.0},
}


def inertia():
    """Principal inertia of the solid box (kg·m²) about x, y and z."""
    hx, hy, hz = HALF_M
    third = MASS_KG / 3.0
    return (third * (hy * hy + hz * hz), third * (hx * hx + hz * hz), third * (hx * hx + hy * hy))


def closed_form(mu):
    """The rigid impulse (P_t, P_n) and the edge's offset (r_x, r_z)."""
    hx, _, hz = HALF_M
    cos, sin = math.cos(PITCH_RAD), math.sin(PITCH_RAD)
    # Nose down about +y: x maps to (cos, 0, -sin), z to (sin, 0, cos).
    edge = (hx, -hz)
    r_x = edge[0] * cos + edge[1] * sin
    r_z = -edge[0] * sin + edge[1] * cos
    pitch_inertia = inertia()[1]
    u_n = VELOCITY_M_S[2]
    # The edge's normal velocity per unit normal impulse, with the friction
    # impulse -mu P_n along x riding along.
    response = 1.0 / MASS_KG + r_x * r_x / pitch_inertia + mu * r_x * r_z / pitch_inertia
    normal = -u_n / response
    return {"tangential_n_s": -mu * normal, "normal_n_s": normal,
            "frictionless_normal_n_s": -u_n / (1.0 / MASS_KG + r_x * r_x / pitch_inertia),
            "edge_m": [r_x, r_z]}


def run(mu, setting):
    spec = mujoco.MjSpec()
    spec.compiler.inertiafromgeom = mujoco.mjtInertiaFromGeom.mjINERTIAFROMGEOM_FALSE
    spec.option.timestep = STEP_S
    spec.option.gravity = [0.0, 0.0, 0.0]
    spec.option.cone = {"pyramidal": mujoco.mjtCone.mjCONE_PYRAMIDAL,
                        "elliptic": mujoco.mjtCone.mjCONE_ELLIPTIC}[setting.get("cone", "elliptic")]
    spec.option.impratio = setting.get("impratio", 100.0)
    floor = spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[5, 5, 0.1],
                                    friction=[mu, 0.005, 0.0001])
    floor.contype, floor.conaffinity = 0, 1
    body = spec.worldbody.add_body(name="box", mass=MASS_KG, inertia=list(inertia()))
    if hasattr(body, "explicitinertial"):
        body.explicitinertial = True
    body.add_freejoint()
    hx, _, hz = HALF_M
    geom = body.add_geom(name="edge", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[RADIUS_M, 0, 0],
                         pos=[hx, 0.0, -hz + RADIUS_M], friction=[mu, 0.005, 0.0001])
    geom.contype, geom.conaffinity = 1, 0
    if "solref" in setting:
        for g in (floor, geom):
            g.solref = list(setting["solref"])
    model = spec.compile()
    data = mujoco.MjData(model)
    edge = model.geom("edge").id
    assert abs(model.body_mass[1] - MASS_KG) < 1e-12, model.body_mass
    half = 0.5 * PITCH_RAD
    data.qpos[3:7] = [math.cos(half), 0.0, math.sin(half), 0.0]
    mujoco.mj_forward(model, data)
    # The sphere's bottom on the floor.
    data.qpos[2] -= data.geom_xpos[edge][2] - RADIUS_M
    data.qvel[:3] = VELOCITY_M_S
    mujoco.mj_forward(model, data)
    before = data.qvel[:3].copy()
    for _ in range(round(DURATION_S / STEP_S)):
        mujoco.mj_step(model, data)
    change = (data.qvel[:3] - before) * MASS_KG
    edge_velocity = np.zeros(6)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_GEOM, edge, edge_velocity, 0)
    return {"tangential_n_s": float(change[0]), "normal_n_s": float(change[2]),
            "edge_velocity_m_s": edge_velocity[3:].tolist()}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.parse_args()
    report = {"engine": f"mujoco {mujoco.__version__}", "cases": []}
    for mu in (0.0, 1.0):
        case = {"friction": mu, "closed_form": closed_form(mu), "mujoco": {}}
        for name, setting in SETTINGS.items():
            case["mujoco"][name] = run(mu, setting)
        report["cases"].append(case)
    print(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
