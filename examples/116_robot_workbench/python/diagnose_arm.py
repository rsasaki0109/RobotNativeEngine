"""Record joint trajectories and peaks for a saved policy's regression target."""
import argparse
import json
from pathlib import Path

import numpy as np

from rne_gym import ArmPoseVectorEnv
from train_arm_pose import policy_actions
from train_stable_arm import ComparisonPolicy


def diagnose(executable, report, seed):
    stable = report.get("task") == "stable_arm_pose_v1"
    if not stable and report.get("task") != "arm_pose_v1":
        raise ValueError("expected a three-joint arm report")
    labels = ["initial", "previous", "learned"] if stable else ["initial", "learned"]
    count = len(labels)
    config = {"workers": count, "environments": [report["world"]] * count}
    gains = np.array([report["policy_log_gains"][k] for k in labels])
    policy = ComparisonPolicy(report["controller"]) if stable else policy_actions
    records = [{"samples": [], "peak_abs_position_rad": np.zeros(3),
                "peak_abs_velocity_rad_s": np.zeros(3), "peak_abs_error_rad": np.zeros(3),
                "return": 0.0} for _ in labels]
    with ArmPoseVectorEnv(executable, config, horizon=report["horizon"],
                          physics_steps=report["physics_steps"]) as env:
        obs, _ = env.reset(seed=[seed] * count)
        if stable:
            policy.reset(np.ones(count, dtype=bool))
        indices = [env.joint_names.index(j) for j in env.target_joints]
        active = np.ones(count, dtype=bool)
        for step in range(env.horizon):
            actions = policy(obs, env.joint_names, env._caps, gains, report["fixed_arm_hold"])
            obs, reward, term, trunc, info = env.step(actions)
            done = term | trunc
            for i in np.flatnonzero(active):
                row = records[i]
                q, dq = obs[i, indices], obs[i, np.array(indices) + len(env.joint_names)]
                error = info["angle_error_rad"][i]
                row["return"] += float(reward[i])
                for key, values in [("peak_abs_position_rad", q), ("peak_abs_velocity_rad_s", dq),
                                    ("peak_abs_error_rad", error)]:
                    row[key] = np.maximum(row[key], np.abs(values))
                if step % 12 == 0 or done[i]:
                    row["samples"].append({"step": step + 1, "q_rad": q.tolist(), "dq_rad_s": dq.tolist(),
                                           "error_rad": error.tolist(),
                                           "commanded_nm": (actions[i, indices] * env._caps[i, indices]).tolist()})
                if done[i]:
                    row.update(steps=step + 1, success=bool(info["is_success"][i]), fallen=bool(info["fallen"][i]),
                               target_rad=info["target_rad"][i].tolist(), state_hash=str(info["state_hash"][i]))
            active &= ~done
            if not active.any():
                break
            if done.any():
                obs, _ = env.reset(options={"reset_mask": done})
                if stable:
                    policy.reset(done)
    for row in records:
        for key in ("peak_abs_position_rad", "peak_abs_velocity_rad_s", "peak_abs_error_rad"):
            row[key] = row[key].tolist()
    return {"schema_version": 1, "seed": seed, "target_joints": list(ArmPoseVectorEnv.target_joints),
            "task": report["task"], "policies": dict(zip(labels, records))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    parser.add_argument("report", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seed", type=int, default=505)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists")
    result = diagnose(args.executable.resolve(), json.loads(args.report.read_text()), args.seed)
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(result, output, indent=2, allow_nan=False)
        output.write("\n")
    print(json.dumps({k: {a: b for a, b in v.items() if a != "samples"} for k, v in result["policies"].items()}))


if __name__ == "__main__":
    main()
