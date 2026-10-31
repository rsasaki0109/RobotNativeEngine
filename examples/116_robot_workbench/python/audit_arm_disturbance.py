"""Paired physical force-pulse audit after a two-second joint-pose hold."""
import argparse
import json
from pathlib import Path

import numpy as np

from arm_controller import ArmPoseController
from rne_gym import ArmPoseVectorEnv

PROTOCOL = {"horizon": 2400, "hold_steps": 480, "pulse_steps": 24,
            "target_joint": "wrist_flex", "force_world_n": [0.0, 0.0, 2.0],
            "dt_s": 0.004166667}


def run_case(env, policy, gains, hold, seed, protocol):
    obs, _ = env.reset(seed=[seed, seed])
    policy.reset(np.ones(2, dtype=bool))
    indices = [env.joint_names.index(j) for j in env.target_joints]
    pre_count = np.zeros(2, dtype=int)
    recovery_count = np.zeros(2, dtype=int)
    pulse_start = None
    delivered = 0
    peak_error = np.zeros((2, 3))
    peak_speed = np.zeros((2, 3))
    displaced = np.zeros(2, dtype=bool)
    peak_joint_delta = np.zeros(3)
    returns = np.zeros(2)
    for step in range(protocol["horizon"]):
        actions = policy(obs, env.joint_names, env._caps, gains, hold)
        pulse = pulse_start is not None and pulse_start <= step < pulse_start + protocol["pulse_steps"]
        forces = [{}, {}]
        if pulse:
            forces[1] = {env.agent_ids[1]: {protocol["target_joint"]: protocol["force_world_n"]}}
        obs, reward, terminated, truncated, info = env.step(actions, forces_n=forces)
        delivered += int(pulse)
        returns += reward
        error = np.abs(info["angle_error_rad"])
        speed = np.abs(obs[:, np.array(indices) + len(env.joint_names)])
        qualified = (error < 0.05).all(axis=1) & (speed < 0.1).all(axis=1) & ~info["fallen"]
        if pulse_start is None:
            # Both worlds must be identical and stable before the intervention.
            if info["state_hash"][0] != info["state_hash"][1]:
                raise RuntimeError("paired worlds diverged before the force pulse")
            pre_count = np.where(qualified, pre_count + 1, 0)
            if (pre_count >= protocol["hold_steps"]).all():
                pulse_start = step + 1
        else:
            peak_error = np.maximum(peak_error, error)
            peak_speed = np.maximum(peak_speed, speed)
            displaced |= ~qualified
            peak_joint_delta = np.maximum(peak_joint_delta, np.abs(obs[1, indices] - obs[0, indices]))
            if step >= pulse_start + protocol["pulse_steps"]:
                recovery_count = np.where(qualified, recovery_count + 1, 0)
                if (recovery_count >= protocol["hold_steps"]).all():
                    break
        if terminated.any() or truncated.any():
            break
    complete_pulse = delivered == protocol["pulse_steps"]
    success = (recovery_count >= protocol["hold_steps"]) & ~info["fallen"] & complete_pulse
    return {"seed": seed, "steps": step + 1, "pulse_start_step": pulse_start,
            "delivered_force_steps": delivered, "precondition_met": pulse_start is not None,
            "force_impulse_ns": (np.array(protocol["force_world_n"]) * delivered * protocol["dt_s"]).tolist(),
            "peak_joint_difference_rad": peak_joint_delta.tolist(),
            "lanes": [{"kind": name, "success": bool(success[i]), "displaced": bool(displaced[i]),
                       "fallen": bool(info["fallen"][i]), "final_hold_steps": int(recovery_count[i]),
                       "peak_abs_error_rad": peak_error[i].tolist(), "peak_abs_speed_rad_s": peak_speed[i].tolist(),
                       "final_error_rad": info["angle_error_rad"][i].tolist(), "return": float(returns[i]),
                       "state_hash": str(info["state_hash"][i])}
                      for i, name in enumerate(("unforced", "forced"))]}


def run(executable, report):
    protocol = report["protocol"]
    if protocol["dt_s"] != 0.004166667 or report["controller"]["dt_s"] != protocol["dt_s"]:
        raise ValueError("physics and controller timestep must match")
    config = {"workers": 2, "environments": [report["world"]] * 2}
    # The audit, not the environment's short-hold terminal rule, owns success.
    with ArmPoseVectorEnv(executable, config, horizon=protocol["horizon"], physics_steps=1,
                          hold_steps=protocol["horizon"] + 1) as env:
        policy = ArmPoseController(2, report["controller"])
        gains = np.array([report["policy_log_gains"]] * 2)
        return [run_case(env, policy, gains, report["fixed_arm_hold"], seed, protocol)
                for seed in report["evaluation_seeds"]]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    parser.add_argument("source", type=Path, help="frozen stable-arm report; with --replay, disturbance report")
    parser.add_argument("output", type=Path, nargs="?")
    parser.add_argument("--replay", action="store_true")
    args = parser.parse_args()
    source = json.loads(args.source.read_text())
    if args.replay:
        if source.get("audit") != "physical_force_recovery_v1":
            parser.error("expected a disturbance audit")
        records = run(args.executable.resolve(), source)
        if records != source["evaluation"]:
            raise SystemExit("force audit replay mismatch")
        print("exact disturbance replay passed")
        return
    if args.output is None or args.output.exists():
        parser.error("provide a new output path")
    if source.get("task") != "stable_arm_pose_v1":
        parser.error("expected a stable-arm policy")
    report = {"audit": "physical_force_recovery_v1", "protocol": PROTOCOL,
              "world": source["world"], "controller": source["controller"],
              "fixed_arm_hold": source["fixed_arm_hold"],
              "policy_log_gains": source["policy_log_gains"]["learned"],
              "evaluation_seeds": source["evaluation_seeds"]}
    report["evaluation"] = run(args.executable.resolve(), report)
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(report, output, indent=2, allow_nan=False)
        output.write("\n")
    print(json.dumps({"cases": len(report["evaluation"]),
                      "successes": sum(r["lanes"][1]["success"] for r in report["evaluation"]),
                      "displaced": sum(r["lanes"][1]["displaced"] for r in report["evaluation"])}))


if __name__ == "__main__":
    main()
