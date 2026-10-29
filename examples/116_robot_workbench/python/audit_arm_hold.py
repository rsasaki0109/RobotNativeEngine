"""Audit frozen controllers with a two-second hold and ten-second time budget."""
import argparse
import copy
import json
from pathlib import Path

from train_stable_arm import compare
from train_shoulder import summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    parser.add_argument("source", type=Path, help="saved stable-arm policy report")
    parser.add_argument("output", type=Path, help="new audit report")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists")
    source = json.loads(args.source.read_text())
    if source.get("task") != "stable_arm_pose_v1":
        parser.error("source must be a stable-arm report")
    report = {k: copy.deepcopy(source[k]) for k in (
        "schema_version", "task", "evaluation_seeds", "world", "physics_steps",
        "fixed_arm_hold", "controller", "policy_log_gains")}
    report.update(horizon=2400, hold_steps=480, audit="sustained_hold_v1",
                  hold_duration_s=480 * 0.004166667,
                  policy_selection="Frozen source policies; no retraining or target selection")
    report["evaluation"] = compare(args.executable.resolve(), report)
    report["summary"] = {k: summary(v) for k, v in report["evaluation"].items()}
    with args.output.open("x", encoding="utf-8") as out:
        json.dump(report, out, indent=2, allow_nan=False)
        out.write("\n")
    print(json.dumps(report["summary"]), flush=True)


if __name__ == "__main__":
    main()
