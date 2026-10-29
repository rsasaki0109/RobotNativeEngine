"""Train bounded-reference PID with success/worst-case-first policy selection."""
import argparse
import copy
import json
from pathlib import Path

import numpy as np

from arm_controller import ArmPoseController, DEFAULTS
from rne_gym import ArmPoseVectorEnv
from train_arm_pose import LOW, HIGH, policy_actions as pd_actions
from train_shoulder import evaluate, search, summary

INITIAL = np.log([3.0, 0.06, 8.0, 0.16, 6.0, 0.12])
TRAIN_SEEDS = [11, 29, 47, 83, 505, 909]
EVAL_SEEDS = list(range(1001, 1009))


def robust_score(records):
    """Lexicographic: success count, worst episode return, then mean return."""
    return [(sum(r["success"] for r in rows), min(r["return"] for r in rows),
             float(np.mean([r["return"] for r in rows]))) for rows in records]


class ComparisonPolicy:
    """Two historical PD lanes followed by a bounded-reference PID lane."""

    def __init__(self, settings):
        self.pid = ArmPoseController(3, settings)

    def reset(self, mask):
        self.pid.reset(mask)

    def __call__(self, obs, names, caps, gains, hold):
        actions = pd_actions(obs, names, caps, gains, hold)
        controlled = self.pid(obs, names, caps, gains, hold)
        actions[2] = controlled[2]
        return actions


def compare(executable, report):
    if report["schema_version"] != 1 or report["task"] != "stable_arm_pose_v1":
        raise ValueError("unsupported stable-arm report")
    config = {"workers": 3, "environments": [report["world"]] * 3}
    gains = np.asarray([report["policy_log_gains"][k] for k in ("initial", "previous", "learned")])
    if gains.shape != (3, 6) or not np.isfinite(gains).all() or (np.abs(gains) > 20).any():
        raise ValueError("invalid policy gains")
    settings = report["controller"]
    if settings["dt_s"] != 0.004166667 or report["physics_steps"] != 1:
        raise ValueError("controller and physical timestep must match")
    with ArmPoseVectorEnv(executable, config, horizon=report["horizon"], physics_steps=1,
                          hold_steps=report.get("hold_steps", 10)) as env:
        policy = ComparisonPolicy(settings)
        rows = evaluate(env, gains, report["evaluation_seeds"], report["fixed_arm_hold"],
                        action_fn=policy, reset_fn=policy.reset)
    return dict(zip(("initial", "previous", "learned"), rows))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    parser.add_argument("report", type=Path, help="new output, or saved input with --replay")
    parser.add_argument("--replay", action="store_true", help="verify exact saved evaluation records")
    parser.add_argument("--generations", type=int, default=4)
    parser.add_argument("--population", type=int, default=6)
    args = parser.parse_args()
    binary = args.executable.resolve()
    if args.replay:
        report = json.loads(args.report.read_text())
        records = compare(binary, report)
        print(json.dumps({k: summary(v) for k, v in records.items()}))
        if records != report["evaluation"]:
            raise SystemExit("saved stable-arm evaluation mismatch")
        return
    if not 2 <= args.population <= 16 or args.generations < 1:
        parser.error("population must be 2..16 and generations positive")
    if args.report.exists():
        parser.error("output already exists")
    source = json.loads((Path(__file__).parent / "results/arm_pose_search.json").read_text())
    config = {"workers": min(args.population, 8), "environments": [source["world"]] * args.population}
    controller = copy.deepcopy(DEFAULTS)
    with ArmPoseVectorEnv(binary, config, horizon=480, physics_steps=1) as env:
        policy = ArmPoseController(args.population, controller)
        best, history = search(env, source["fixed_arm_hold"], seed=42, generations=args.generations,
                               train_seeds=TRAIN_SEEDS, initial=INITIAL, low=LOW, high=HIGH,
                               initial_std=0.3, action_fn=policy, reset_fn=policy.reset, score_fn=robust_score)
    report = {"schema_version": 1, "task": "stable_arm_pose_v1", "algorithm": "cem_lexicographic",
              "selection_order": ["success_count", "worst_return", "mean_return"],
              "seed": 42, "train_seeds": TRAIN_SEEDS, "evaluation_seeds": EVAL_SEEDS,
              "world": source["world"], "horizon": 480, "physics_steps": 1,
              "fixed_arm_hold": source["fixed_arm_hold"], "controller": controller,
              "policy_log_gains": {"initial": source["policy_log_gains"]["initial"],
                                   "previous": source["policy_log_gains"]["learned"], "learned": best.tolist()},
              "training_initial_log_gains": INITIAL.tolist(), "training": history}
    report["evaluation"] = compare(binary, report)
    report["summary"] = {k: summary(v) for k, v in report["evaluation"].items()}
    with args.report.open("x", encoding="utf-8") as output:
        json.dump(report, output, indent=2, allow_nan=False)
        output.write("\n")
    print(json.dumps(report["summary"]), flush=True)


if __name__ == "__main__":
    main()
