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
`--no-self-collision` turns those contacts off. `--residual` adds the
residual (linear or MLP) that example 144's `--train-residual` writes to every
joint target, computed from the same observation as the policy's action, as
RNE does.

`--trace DIRECTION,FORCE_N,PHASE` instead writes that one push trial's
policy steps, observations and actions included, to compare with example
144's `--trace-push`; RNE_REPORT.json may then be a sweep or that trace.

Usage: mujoco_robustness.py UNITREE_RL_GYM RNE_REPORT.json OUTPUT.json
       [--processes 4] [--no-self-collision] [--residual RESIDUAL.json]
       [--residual-generation G] [--trace DIRECTION,FORCE_N,PHASE]
"""
import argparse
import json
import math
import multiprocessing
from pathlib import Path

import mujoco
import numpy as np

from metrics import summarize
from mujoco_reference import ROBOTS, Policy, projected_gravity


class Residual:
    """Example 144's residual, added to the joint targets.

    Linear: clip(W · obs[:5], ±limit). MLP (`"model": "mlp"`):
    limit · tanh(W2 · tanh(W1 · obs[indices] + b1)).
    """

    def __init__(self, spec):
        self.limit = float(spec["limit_rad"])
        self.mlp = spec.get("model") == "mlp"
        if self.mlp:
            self.indices = np.array(spec["observation_indices"], np.int64)
            self.w1 = np.array(spec["w1"], np.float64)
            self.b1 = np.array(spec["b1"], np.float64)
            self.w2 = np.array(spec["w2"], np.float64)
            if self.w1.shape != (len(self.b1), len(self.indices)) or \
                    self.w2.shape[1] != len(self.b1):
                raise ValueError("inconsistent MLP residual shapes")
        else:
            self.weights = np.array(spec["weights_rad"], np.float64)
            if self.weights.shape[1] != len(spec["features"]) or len(spec["features"]) != 5:
                raise ValueError("the residual must read the first five observation entries")

    def __call__(self, obs):
        x = obs.astype(np.float64)
        if self.mlp:
            hidden = np.tanh(self.w1 @ x[self.indices] + self.b1)
            return self.limit * np.tanh(self.w2 @ hidden)
        return np.clip(self.weights @ x[:5], -self.limit, self.limit)


# Per-process caches: compiling the scene (meshes and their hulls) and reading
# the policy cost more than many of the short trials they serve. A cached model
# is reset with mj_resetData, which restores a freshly made MjData's state.
_SIMULATIONS = {}
_POLICIES = {}


def simulation(upstream, robot, friction, self_collision):
    key = (upstream, robot, friction, self_collision)
    if key not in _SIMULATIONS:
        model = mujoco.MjModel.from_xml_path(str(Path(upstream) / ROBOTS[robot]["scene"]))
        if friction is not None:
            model.geom_friction[:, 0] = friction
        if not self_collision:
            # Robot geoms keep colliding with the world, not with each other.
            robot_geoms = (model.geom_bodyid != 0) & (model.geom_contype != 0)
            model.geom_conaffinity[robot_geoms] = 0
        _SIMULATIONS[key] = (model, mujoco.MjData(model))
    model, data = _SIMULATIONS[key]
    mujoco.mj_resetData(model, data)
    return model, data


def fresh_policy(robot):
    if robot not in _POLICIES:
        _POLICIES[robot] = Policy(ROBOTS[robot]["policy"])
    policy = _POLICIES[robot]
    policy.h = np.zeros_like(policy.h)
    policy.c = np.zeros_like(policy.c)
    return policy


def walk(upstream, robot, protocol, policy_steps, push=None, friction=None, fall_m=None,
         self_collision=True, self_contacts=None, residual=None, offset_squares=None,
         tilts=None, penetrations=None, trace=False):
    """Walks `policy_steps` policy periods; returns (fall time or None, records).

    Adds the name pairs of robot bodies in contact to `self_contacts`, if given.
    `residual` is the JSON `residual` object of example 144's training report;
    `offset_squares`, if given, receives each policy step's mean squared offset,
    `tilts` each policy step's pelvis tilt in radians and `penetrations` each
    policy step's deepest floor contact in metres (zero if nothing touches).
    With `trace`, each record also holds the policy's observation and action.
    """
    residual = Residual(residual) if residual else None
    model, data = simulation(upstream, robot, friction, self_collision)
    policy = fresh_policy(robot)
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
        if penetrations is not None:
            bodies = model.geom_bodyid[data.contact.geom].reshape(-1, 2)
            floor = data.contact.dist[(bodies == 0).any(axis=1)]
            penetrations.append(float(max(-floor.min(), 0.0)) if len(floor) else 0.0)
        phase = (counter * dt) % period / period
        obs[:3] = data.qvel[3:6] * spec["angular_velocity_scale"]
        obs[3:6] = projected_gravity(data.qpos[3:7])
        if tilts is not None:
            tilts.append(math.acos(min(max(-float(obs[5]), -1.0), 1.0)))
        obs[6:9] = cmd
        obs[9:9 + joints] = data.qpos[7:] - default
        obs[9 + joints:9 + 2 * joints] = data.qvel[6:] * spec["joint_velocity_scale"]
        obs[9 + 2 * joints:9 + 3 * joints] = action
        obs[9 + 3 * joints:] = [np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)]
        action = policy(obs.copy())
        target = action * spec["action_scale"] + default
        if residual is not None:
            offsets = residual(obs)
            target = target + offsets
            if offset_squares is not None:
                offset_squares.append(float(np.mean(offsets * offsets)))
        time_s = counter * dt
        records.append({"time_s": time_s, "base_position_m": data.qpos[:3].tolist(),
                        "base_quaternion_wxyz": data.qpos[3:7].tolist()})
        if trace:
            records[-1].update(observation=obs.tolist(), action=action.tolist())
        if fall_m is not None and data.qpos[2] < fall_m:
            return time_s, records
    return None, records


def push_trial(job, trace=False):
    upstream, robot, protocol, case, self_collision, residual = job
    directions = {d["name"]: d["z_up_xy"] for d in protocol["directions"]}
    steps = lambda seconds: int(round(seconds / 0.02))
    start = steps(protocol["walk_before_push_s"]) + case["phase"] * steps(protocol["phase_spacing_s"])
    end = start + steps(protocol["push_duration_s"])
    x, y = directions[case["direction"]]
    force = np.array([x * case["force_n"], y * case["force_n"], 0.0])
    contacts = set()
    fall, records = walk(upstream, robot, protocol, end + steps(protocol["recovery_s"]),
                         push=(start, end, force), fall_m=protocol["fall_height_m"],
                         self_collision=self_collision, self_contacts=contacts,
                         residual=residual, trace=trace)
    result = {**case, "fell": fall is not None, "fall_time_s": fall,
              "self_contacts": [list(pair) for pair in sorted(contacts)]}
    return {**result, "records": records} if trace else result


def friction_trial(job):
    upstream, robot, protocol, friction, self_collision, residual = job
    _, records = walk(upstream, robot, protocol, int(round(protocol["friction_walk_s"] / 0.02)),
                      friction=friction, self_collision=self_collision, residual=residual)
    return {"friction": friction, "summary": summarize(records, protocol["command"])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("upstream", type=Path)
    parser.add_argument("rne_report", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--processes", type=int, default=multiprocessing.cpu_count())
    parser.add_argument("--no-self-collision", action="store_true",
                        help="let robot geoms collide only with the floor, as in RNE's model")
    parser.add_argument("--residual", type=Path,
                        help="training report from example 144's --train-residual")
    parser.add_argument("--residual-generation", type=int,
                        help="use the residual checkpoint stored at this generation")
    parser.add_argument("--trace", help="write one push trial's steps: DIRECTION,FORCE_N,PHASE")
    arguments = parser.parse_args()
    rne = json.loads(arguments.rne_report.read_text())
    robot, protocol = rne["robot"], rne["protocol"]
    upstream = str(arguments.upstream)
    residual = None
    if arguments.residual:
        trained = json.loads(arguments.residual.read_text())
        if trained["robot"] != robot:
            raise SystemExit(f"{arguments.residual} was trained for {trained['robot']}")
        residual = trained["residual"]
        if arguments.residual_generation is not None:
            stored = {c["generation"]: c["residual"]
                      for c in trained["training"].get("checkpoints", [])}
            if arguments.residual_generation not in stored:
                raise SystemExit(f"no checkpoint at generation {arguments.residual_generation}")
            residual = stored[arguments.residual_generation]
    self_collision = not arguments.no_self_collision
    if arguments.trace:
        direction, force_n, phase = arguments.trace.split(",")
        case = {"direction": direction, "force_n": float(force_n), "phase": int(phase)}
        trial = push_trial((upstream, robot, protocol, case, self_collision, residual), trace=True)
        arguments.output.write_text(json.dumps(
            {"schema_version": 1, "engine": f"mujoco {mujoco.__version__}", "robot": robot,
             "self_collision": self_collision, "residual": residual, "case": case,
             "protocol": protocol, "fall_time_s": trial["fall_time_s"],
             "self_contacts": trial["self_contacts"], "records": trial["records"]}) + "\n")
        print(f"wrote {arguments.output}; fell at {trial['fall_time_s']}")
        return
    cases = [{"direction": r["direction"], "force_n": r["force_n"], "phase": r["phase"]}
             for r in rne["push"]]
    with multiprocessing.get_context("fork").Pool(arguments.processes) as pool:
        push = pool.map(push_trial, [(upstream, robot, protocol, c, self_collision, residual)
                                     for c in cases])
        friction = pool.map(friction_trial, [(upstream, robot, protocol, f, self_collision,
                                              residual) for f in protocol["frictions"]])
    arguments.output.write_text(json.dumps(
        {"schema_version": 1, "engine": f"mujoco {mujoco.__version__}", "robot": robot,
         "self_collision": not arguments.no_self_collision, "residual": residual,
         "protocol": protocol, "push": push, "friction": friction}, indent=2) + "\n")
    print(f"wrote {arguments.output}")


if __name__ == "__main__":
    main()
