#!/usr/bin/env python3
"""MuJoCo side of example 144's `--robustness` sweep.

Reads the protocol from RNE's report (forces, push timing, directions,
frictions) so both engines run identical trials, then repeats them on
unitree_rl_gym's MuJoCo scene with the loop of mujoco_reference.py. A push
is applied with `mj_applyFT` at the pelvis origin, as RNE applies it, for
the physics steps of the pushed policy periods. Friction trials set every
geom's sliding friction, since MuJoCo combines a contact's friction as the
larger of its two geoms'. Each push trial lists the pairs of robot bodies
that touched before the fall (MuJoCo skips parent-child pairs);
`--no-self-collision` turns those contacts off.

Usage: mujoco_robustness.py UNITREE_RL_GYM RNE_REPORT.json OUTPUT.json
       [--processes 4] [--no-self-collision]
"""
import argparse
import json
import multiprocessing
from pathlib import Path

import mujoco
import numpy as np

from metrics import summarize
from mujoco_reference import ROBOTS, Policy, projected_gravity


def walk(upstream, robot, protocol, policy_steps, push=None, friction=None, fall_m=None,
         self_collision=True, self_contacts=None):
    """Walks `policy_steps` policy periods; returns (fall time or None, records).

    Adds the name pairs of robot bodies in contact to `self_contacts`, if given.
    """
    model = mujoco.MjModel.from_xml_path(str(Path(upstream) / ROBOTS[robot]["scene"]))
    if friction is not None:
        model.geom_friction[:, 0] = friction
    if not self_collision:
        # Robot geoms keep colliding with the world, not with each other.
        robot_geoms = (model.geom_bodyid != 0) & (model.geom_contype != 0)
        model.geom_conaffinity[robot_geoms] = 0
    data = mujoco.MjData(model)
    policy = Policy(ROBOTS[robot]["policy"])
    spec = policy.deployment
    dt, decimation, period = spec["physics_dt_s"], spec["control_decimation"], spec["gait_period_s"]
    model.opt.timestep = dt
    pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    kp = np.array(spec["kp_nm_per_rad"], np.float32)
    kd = np.array(spec["kd_nm_s_per_rad"], np.float32)
    default = np.array(spec["default_joint_rad"], np.float32)
    cmd = np.array(protocol["command"], np.float32) * np.array(spec["command_scale"], np.float32)
    joints = len(default)
    action = np.zeros(joints, np.float32)
    target = default.copy()
    obs = np.zeros(9 + 3 * joints + 2, np.float32)
    zero = np.zeros(3)
    records = []
    for counter in range(1, policy_steps * decimation + 1):
        data.ctrl[:] = (target - data.qpos[7:]) * kp - data.qvel[6:] * kd
        data.qfrc_applied[:] = 0.0
        if push is not None and push[0] <= (counter - 1) // decimation < push[1]:
            mujoco.mj_applyFT(model, data, push[2], zero, data.xpos[pelvis].copy(), pelvis,
                              data.qfrc_applied)
        mujoco.mj_step(model, data)
        if self_contacts is not None and data.ncon:
            bodies = model.geom_bodyid[data.contact.geom].reshape(-1, 2)
            for pair in bodies[(bodies != 0).all(axis=1)]:
                self_contacts.add(tuple(sorted(model.body(int(b)).name for b in pair)))
        if counter % decimation:
            continue
        phase = (counter * dt) % period / period
        obs[:3] = data.qvel[3:6] * spec["angular_velocity_scale"]
        obs[3:6] = projected_gravity(data.qpos[3:7])
        obs[6:9] = cmd
        obs[9:9 + joints] = data.qpos[7:] - default
        obs[9 + joints:9 + 2 * joints] = data.qvel[6:] * spec["joint_velocity_scale"]
        obs[9 + 2 * joints:9 + 3 * joints] = action
        obs[9 + 3 * joints:] = [np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)]
        action = policy(obs.copy())
        target = action * spec["action_scale"] + default
        time_s = counter * dt
        records.append({"time_s": time_s, "base_position_m": data.qpos[:3].tolist(),
                        "base_quaternion_wxyz": data.qpos[3:7].tolist()})
        if fall_m is not None and data.qpos[2] < fall_m:
            return time_s, records
    return None, records


def push_trial(job):
    upstream, robot, protocol, case, self_collision = job
    directions = {d["name"]: d["z_up_xy"] for d in protocol["directions"]}
    steps = lambda seconds: int(round(seconds / 0.02))
    start = steps(protocol["walk_before_push_s"]) + case["phase"] * steps(protocol["phase_spacing_s"])
    end = start + steps(protocol["push_duration_s"])
    x, y = directions[case["direction"]]
    force = np.array([x * case["force_n"], y * case["force_n"], 0.0])
    contacts = set()
    fall, _ = walk(upstream, robot, protocol, end + steps(protocol["recovery_s"]),
                   push=(start, end, force), fall_m=protocol["fall_height_m"],
                   self_collision=self_collision, self_contacts=contacts)
    return {**case, "fell": fall is not None, "fall_time_s": fall,
            "self_contacts": [list(pair) for pair in sorted(contacts)]}


def friction_trial(job):
    upstream, robot, protocol, friction, self_collision = job
    _, records = walk(upstream, robot, protocol, int(round(protocol["friction_walk_s"] / 0.02)),
                      friction=friction, self_collision=self_collision)
    return {"friction": friction, "summary": summarize(records, protocol["command"])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("upstream", type=Path)
    parser.add_argument("rne_report", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--processes", type=int, default=multiprocessing.cpu_count())
    parser.add_argument("--no-self-collision", action="store_true",
                        help="let robot geoms collide only with the floor, as in RNE's model")
    arguments = parser.parse_args()
    rne = json.loads(arguments.rne_report.read_text())
    robot, protocol = rne["robot"], rne["protocol"]
    cases = [{"direction": r["direction"], "force_n": r["force_n"], "phase": r["phase"]}
             for r in rne["push"]]
    upstream = str(arguments.upstream)
    with multiprocessing.get_context("fork").Pool(arguments.processes) as pool:
        self_collision = not arguments.no_self_collision
        push = pool.map(push_trial, [(upstream, robot, protocol, c, self_collision)
                                     for c in cases])
        friction = pool.map(friction_trial, [(upstream, robot, protocol, f, self_collision)
                                             for f in protocol["frictions"]])
    arguments.output.write_text(json.dumps(
        {"schema_version": 1, "engine": f"mujoco {mujoco.__version__}", "robot": robot,
         "self_collision": not arguments.no_self_collision,
         "protocol": protocol, "push": push, "friction": friction}, indent=2) + "\n")
    print(f"wrote {arguments.output}")


if __name__ == "__main__":
    main()
