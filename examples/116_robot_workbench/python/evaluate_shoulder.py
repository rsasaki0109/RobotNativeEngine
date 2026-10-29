"""Reload a policy-search report and repeat its held-out paired evaluation."""
import argparse
import json
from pathlib import Path

import numpy as np

from rne_gym import ShoulderVectorEnv
from train_shoulder import evaluate, summary
from train_shoulder import policy_actions as shoulder_actions
from train_arm_pose import policy_actions as arm_actions
from rne_gym import ArmPoseVectorEnv


def replay(executable, report):
    if report["schema_version"] != 1 or report["algorithm"] != "cross_entropy_pd_gain_search":
        raise ValueError("unsupported policy report")
    gains = np.asarray([report["policy_log_gains"]["initial"], report["policy_log_gains"]["learned"]])
    task = report.get("task", "shoulder")
    if task not in ("shoulder", "arm_pose_v1"):
        raise ValueError("unsupported task")
    width = 6 if task == "arm_pose_v1" else 2
    env_type = ArmPoseVectorEnv if task == "arm_pose_v1" else ShoulderVectorEnv
    action_fn = arm_actions if task == "arm_pose_v1" else shoulder_actions
    if task == "arm_pose_v1" and report["target_joints"] != list(env_type.target_joints):
        raise ValueError("unexpected target joint order")
    if gains.shape != (2, width) or not np.isfinite(gains).all() or (np.abs(gains) > 20).any():
        raise ValueError("policy log gains must be two finite bounded pairs")
    config = {"workers": 2, "environments": [report["world"], report["world"]]}
    with env_type(executable, config, horizon=report["horizon"],
                           physics_steps=report["physics_steps"]) as env:
        records = evaluate(env, gains, report["evaluation_seeds"], report["fixed_arm_hold"], action_fn=action_fn)
    return {"initial": records[0], "learned": records[1]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    parser.add_argument("report", type=Path)
    parser.add_argument("--check", action="store_true", help="require exact saved transitions and metrics")
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    records = replay(args.executable.resolve(), report)
    print(json.dumps({k: summary(v) for k, v in records.items()}))
    if args.check and records != report["evaluation"]:
        raise SystemExit("evaluation differs from saved report (including state hashes)")


if __name__ == "__main__":
    main()
