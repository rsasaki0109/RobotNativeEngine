"""Learn six PD gains for simultaneous SO-101 pan/lift/elbow pose tracking."""
import argparse
import copy
import json
from pathlib import Path

import numpy as np

from rne_gym import ArmPoseVectorEnv
from train_shoulder import SCENARIOS, evaluate, search, summary

INITIAL = np.log([0.3, 0.02, 8.0, 0.16, 6.0, 0.12])
LOW = np.log([0.05, 0.002] * 3)
HIGH = np.log([20.0, 0.5] * 3)


def policy_actions(observations, names, caps, log_gains, hold):
    """Learned pan/lift/elbow PD; fixed wrist/gripper PD and coasting wheels."""
    actions = np.zeros((len(observations), len(names)))
    gains = np.exp(log_gains).reshape(len(observations), 3, 2)
    targets = observations[:, -5:-2]
    for j, name in enumerate(names):
        if name not in hold:
            continue
        g = hold[name]
        if name in ArmPoseVectorEnv.target_joints:
            k = ArmPoseVectorEnv.target_joints.index(name)
            kp, kd = gains[:, k, 0], gains[:, k, 1]
            target = targets[:, k]
        else:
            kp, kd, target = g["stiffness_nm_per_rad"], g["damping_nm_s_per_rad"], 0.0
        torque = kp * (target - observations[:, j]) - kd * observations[:, j + len(names)]
        actions[:, j] = np.clip(torque, -g["effort_limit_nm"], g["effort_limit_nm"]) / caps[:, j]
    return actions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--generations", type=int, default=6)
    parser.add_argument("--population", type=int, default=8)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    if not 2 <= args.population <= 16 or args.generations < 1 or args.seed < 0:
        parser.error("population must be 2..16, generations positive, seed nonnegative")
    if args.output.exists():
        parser.error("output already exists")
    world = json.loads((SCENARIOS / "vector_arena.json").read_text())["environments"][0]
    hold = json.loads((SCENARIOS / "wheel_arm_hold.json").read_text())["arm_hold"]["alice"]
    config = {"workers": min(args.population, 8),
              "environments": [copy.deepcopy(world) for _ in range(args.population)]}
    train_seeds = [11, 29, 47, 83]
    eval_seeds = [101, 202, 303, 404, 505, 606, 707, 808]
    with ArmPoseVectorEnv(args.executable.resolve(), config, horizon=480, physics_steps=1) as env:
        best, history = search(env, hold, seed=args.seed, generations=args.generations,
                               train_seeds=train_seeds, initial=INITIAL, low=LOW, high=HIGH,
                               initial_std=0.6, action_fn=policy_actions)
    config["environments"] = config["environments"][:2]
    config["workers"] = 2
    with ArmPoseVectorEnv(args.executable.resolve(), config, horizon=480, physics_steps=1) as env:
        evaluation = evaluate(env, np.stack([INITIAL, best]), eval_seeds, hold, action_fn=policy_actions)
    report = {"schema_version": 1, "algorithm": "cross_entropy_pd_gain_search", "task": "arm_pose_v1",
              "target_joints": list(ArmPoseVectorEnv.target_joints),
              "seed": args.seed, "train_seeds": train_seeds, "evaluation_seeds": eval_seeds,
              "world": world, "horizon": 480, "physics_steps": 1,
              "fixed_arm_hold": hold, "initial_gains": np.exp(INITIAL).tolist(),
              "policy_log_gains": {"initial": INITIAL.tolist(), "learned": best.tolist()},
              "learned_gains": np.exp(best).tolist(), "training": history,
              "evaluation": {"initial": evaluation[0], "learned": evaluation[1]},
              "summary": {"initial": summary(evaluation[0]), "learned": summary(evaluation[1])}}
    with args.output.open("x", encoding="utf-8") as out:
        json.dump(report, out, indent=2, allow_nan=False)
        out.write("\n")
    print(json.dumps(report["summary"]), flush=True)


if __name__ == "__main__":
    main()
