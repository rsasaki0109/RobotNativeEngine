#!/usr/bin/env python3
"""mujoco_robustness.py's push sweep with one MuJoCo modelling choice changed.

Example 144's study of where RNE and MuJoCo disagree measures how far
MuJoCo's own push outcomes move when one of its modelling choices changes
(friction cone, friction softness, joint limits, time step), against which the
RNE–MuJoCo disagreement can be read, and which choices bring MuJoCo closer to
RNE. Every option edits the compiled model, or the deployment's time step,
before each trial; the trials are mujoco_robustness.py's. The report has the
same `push` list as mujoco_robustness.py's, so push_agreement.py compares it
with any sweep.

Options: --cone elliptic, --impratio X, --noslip-iterations N,
--no-joint-limits, --joint-friction NM (every hinge's frictionloss),
--floor-friction MU (every geom's sliding friction), --timestep-divisor K
(K-times-smaller physics steps, K-times-larger decimation, same 50 Hz policy)
and --perturb RAD (offsets the first joint's initial angle).

Usage: mujoco_ablation.py UNITREE_RL_GYM RNE_REPORT.json OUTPUT.json
       [--no-self-collision] [--processes 4] [options]
"""
import argparse
import copy
import json
import multiprocessing
from pathlib import Path

import mujoco

import mujoco_robustness


def edited(arguments):
    """mujoco_robustness's `simulation` and `fresh_policy` with the edits."""
    simulation, fresh_policy = mujoco_robustness.simulation, mujoco_robustness.fresh_policy

    def edited_simulation(upstream, robot, friction, self_collision):
        model, data = simulation(upstream, robot, friction, self_collision)
        if arguments.cone:
            model.opt.cone = getattr(mujoco.mjtCone, f"mjCONE_{arguments.cone.upper()}")
        if arguments.impratio is not None:
            model.opt.impratio = arguments.impratio
        if arguments.noslip_iterations is not None:
            model.opt.noslip_iterations = arguments.noslip_iterations
        if arguments.no_joint_limits:
            model.jnt_limited[:] = 0
        if arguments.joint_friction is not None:
            model.dof_frictionloss[6:] = arguments.joint_friction
        if arguments.floor_friction is not None:
            model.geom_friction[:, 0] = arguments.floor_friction
        if arguments.perturb:
            data.qpos[7] += arguments.perturb
        return model, data

    def divided_policy(robot):
        policy = fresh_policy(robot)
        if arguments.timestep_divisor > 1 and not getattr(policy, "divided", False):
            policy.deployment = copy.deepcopy(policy.deployment)
            policy.deployment["physics_dt_s"] /= arguments.timestep_divisor
            policy.deployment["control_decimation"] *= arguments.timestep_divisor
            policy.divided = True
        return policy

    return edited_simulation, divided_policy


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("upstream", type=Path)
    parser.add_argument("rne_report", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--processes", type=int, default=multiprocessing.cpu_count())
    parser.add_argument("--no-self-collision", action="store_true")
    parser.add_argument("--cone", choices=["pyramidal", "elliptic"])
    parser.add_argument("--impratio", type=float)
    parser.add_argument("--noslip-iterations", type=int)
    parser.add_argument("--no-joint-limits", action="store_true")
    parser.add_argument("--joint-friction", type=float)
    parser.add_argument("--floor-friction", type=float)
    parser.add_argument("--timestep-divisor", type=int, default=1)
    parser.add_argument("--perturb", type=float)
    arguments = parser.parse_args()
    rne = json.loads(arguments.rne_report.read_text())
    robot, protocol = rne["robot"], rne["protocol"]
    cases = [{"direction": r["direction"], "force_n": r["force_n"], "phase": r["phase"]}
             for r in rne["push"]]
    self_collision = not arguments.no_self_collision
    # Forked workers inherit the edited functions; walk() looks them up per trial.
    mujoco_robustness.simulation, mujoco_robustness.fresh_policy = edited(arguments)
    with multiprocessing.get_context("fork").Pool(arguments.processes) as pool:
        push = pool.map(mujoco_robustness.push_trial,
                        [(str(arguments.upstream), robot, protocol, case, self_collision, None)
                         for case in cases])
    ablation = {k: v for k, v in vars(arguments).items()
                if k not in ("upstream", "rne_report", "output", "processes", "no_self_collision")
                and v is not None and v is not False
                and not (k == "timestep_divisor" and v == 1)}
    arguments.output.write_text(json.dumps(
        {"schema_version": 1, "engine": f"mujoco {mujoco.__version__}", "robot": robot,
         "self_collision": self_collision, "ablation": ablation, "protocol": protocol,
         "push": push}, indent=2) + "\n")
    print(f"wrote {arguments.output}: {sum(t['fell'] for t in push)} falls, ablation {ablation}")


if __name__ == "__main__":
    main()
