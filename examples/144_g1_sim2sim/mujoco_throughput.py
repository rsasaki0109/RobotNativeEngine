#!/usr/bin/env python3
"""Time MuJoCo on the same walking loop, for comparison with `--throughput`.

Runs mujoco_reference.py's loop (500 Hz PD, 50 Hz policy in numpy) in 1, 2, 4
and 8 processes at once. Each process reports the time spent inside
`mj_step` and its whole loop; the Python PD law and policy are not MuJoCo's
cost, so the physics-only figures are the fair comparison with RNE's loop,
which runs PD and policy in Rust. Throughput is processes times simulated
seconds over the mean time per process, as `--throughput` computes it for
RNE's walkers; the slowest process gives a lower bound.

Usage: mujoco_throughput.py UNITREE_RL_GYM_ROOT OUTPUT.json [--robot g1|h1]
       [--seconds 10]
"""
import argparse
import json
import multiprocessing
import os
import time
from pathlib import Path

import mujoco
import numpy as np

from mujoco_reference import ROBOTS, Policy, projected_gravity

COMMAND = [0.5, 0.0, 0.0]


def walk(arguments):
    upstream, robot, seconds, start = arguments
    model = mujoco.MjModel.from_xml_path(str(Path(upstream) / ROBOTS[robot]["scene"]))
    data = mujoco.MjData(model)
    policy = Policy(ROBOTS[robot]["policy"])
    spec = policy.deployment
    dt, decimation, period = spec["physics_dt_s"], spec["control_decimation"], spec["gait_period_s"]
    model.opt.timestep = dt
    kp = np.array(spec["kp_nm_per_rad"], np.float32)
    kd = np.array(spec["kd_nm_s_per_rad"], np.float32)
    default = np.array(spec["default_joint_rad"], np.float32)
    cmd = np.array(COMMAND, np.float32) * np.array(spec["command_scale"], np.float32)
    joints = len(default)
    action = np.zeros(joints, np.float32)
    target = default.copy()
    obs = np.zeros(9 + 3 * joints + 2, np.float32)
    start.wait()
    step_s = 0.0
    begin = time.perf_counter()
    for counter in range(1, int(round(seconds / dt)) + 1):
        data.ctrl[:] = (target - data.qpos[7:]) * kp - data.qvel[6:] * kd
        t0 = time.perf_counter()
        mujoco.mj_step(model, data)
        step_s += time.perf_counter() - t0
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
    return {"loop_s": time.perf_counter() - begin, "mj_step_s": step_s,
            "base_height_m": float(data.qpos[2])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("upstream", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--robot", choices=sorted(ROBOTS), default="g1")
    parser.add_argument("--seconds", type=float, default=10.0)
    arguments = parser.parse_args()
    rows = []
    context = multiprocessing.get_context("fork")
    for envs in [1, 2, 4, 8]:
        with context.Manager() as manager:
            start = manager.Barrier(envs)
            with context.Pool(envs) as pool:
                results = pool.map(walk, [(str(arguments.upstream), arguments.robot,
                                           arguments.seconds, start)] * envs)
        step_s = float(np.mean([r["mj_step_s"] for r in results]))
        loop_s = float(np.mean([r["loop_s"] for r in results]))
        row = {"processes": envs,
               "per_process_mj_step_realtime_factor": arguments.seconds / step_s,
               "per_process_loop_realtime_factor": arguments.seconds / loop_s,
               "mj_step_simulated_s_per_wall_s": envs * arguments.seconds / step_s,
               "loop_simulated_s_per_wall_s": envs * arguments.seconds / loop_s,
               "slowest_mj_step_simulated_s_per_wall_s":
                   envs * arguments.seconds / max(r["mj_step_s"] for r in results),
               "min_base_height_m": min(r["base_height_m"] for r in results)}
        rows.append(row)
        print(json.dumps(row))
    arguments.output.write_text(json.dumps(
        {"engine": f"mujoco {mujoco.__version__}", "robot": arguments.robot,
         "cores": os.cpu_count(), "command": COMMAND, "seconds_per_process": arguments.seconds,
         "rows": rows}, indent=2) + "\n")


if __name__ == "__main__":
    main()
