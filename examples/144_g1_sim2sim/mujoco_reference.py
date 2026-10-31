#!/usr/bin/env python3
"""Run the vendored G1 walking policy in MuJoCo as the sim2sim reference.

This mirrors unitree_rl_gym's deploy/deploy_mujoco/deploy_mujoco.py (500 Hz
physics, 50 Hz policy, the same 47-value observation and PD law) but runs the
policy from RNE's weight blob in numpy, so both engines use identical weights
and nothing from the TorchScript archive is executed. Requires mujoco and numpy
plus a checkout of unitree_rl_gym for its MuJoCo scene.

Usage: mujoco_reference.py UNITREE_RL_GYM_ROOT OUTPUT.json [--seconds 10]
       [--fixture POLICY_IO.json]
"""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from metrics import COMMANDS, summarize

REPO = Path(__file__).resolve().parents[2]
POLICY = REPO / "assets/policies/unitree_g1_walk/g1_walk_lstm.policy.json"


class Policy:
    """LSTM(47->64) -> Linear(64,32) -> ELU -> Linear(32,12) in float32."""

    def __init__(self, manifest_path):
        manifest = json.loads(manifest_path.read_text())
        weights = manifest["weights"]
        blob = (manifest_path.parent / weights["file"]).read_bytes()
        if hashlib.sha256(blob).hexdigest() != weights["sha256"]:
            raise ValueError("Policy blob does not match its manifest")
        values = np.frombuffer(blob, "<f4")
        t = {}
        for tensor in weights["tensors"]:
            count = int(np.prod(tensor["shape"]))
            start = tensor["offset_f32"]
            t[tensor["name"]] = values[start:start + count].reshape(tensor["shape"])
        self.w_ih, self.w_hh = t["memory.weight_ih_l0"], t["memory.weight_hh_l0"]
        self.b = t["memory.bias_ih_l0"] + t["memory.bias_hh_l0"]
        self.w0, self.b0 = t["actor.0.weight"], t["actor.0.bias"]
        self.w1, self.b1 = t["actor.2.weight"], t["actor.2.bias"]
        self.deployment = manifest["deployment"]
        self.h = np.zeros(64, np.float32)
        self.c = np.zeros(64, np.float32)

    def __call__(self, obs):
        gates = self.w_ih @ obs + self.w_hh @ self.h + self.b
        i, f, g, o = np.split(gates, 4)

        def sigmoid(x):
            return 1.0 / (1.0 + np.exp(-x))

        self.c = sigmoid(f) * self.c + sigmoid(i) * np.tanh(g)
        self.h = sigmoid(o) * np.tanh(self.c)
        x = self.w0 @ self.h + self.b0
        x = np.where(x > 0, x, np.expm1(x))
        return (self.w1 @ x + self.b1).astype(np.float32)


def projected_gravity(q):
    w, x, y, z = q
    return np.array([2 * (-z * x + w * y), -2 * (z * y + w * x), 1 - 2 * (w * w + z * z)])


def run(upstream, command, seconds):
    model = mujoco.MjModel.from_xml_path(str(upstream / "resources/robots/g1_description/scene.xml"))
    data = mujoco.MjData(model)
    policy = Policy(POLICY)
    spec = policy.deployment
    dt, decimation, period = spec["physics_dt_s"], spec["control_decimation"], spec["gait_period_s"]
    model.opt.timestep = dt
    kp = np.array(spec["kp_nm_per_rad"], np.float32)
    kd = np.array(spec["kd_nm_s_per_rad"], np.float32)
    default = np.array(spec["default_joint_rad"], np.float32)
    cmd = np.array(command, np.float32)
    action = np.zeros(12, np.float32)
    target = default.copy()
    obs = np.zeros(47, np.float32)
    records = []
    for counter in range(1, int(round(seconds / dt)) + 1):
        data.ctrl[:] = (target - data.qpos[7:]) * kp - data.qvel[6:] * kd
        mujoco.mj_step(model, data)
        if counter % decimation:
            continue
        phase = (counter * dt) % period / period
        obs[:3] = data.qvel[3:6] * spec["angular_velocity_scale"]
        obs[3:6] = projected_gravity(data.qpos[3:7])
        obs[6:9] = cmd * np.array(spec["command_scale"], np.float32)
        obs[9:21] = data.qpos[7:] - default
        obs[21:33] = data.qvel[6:] * spec["joint_velocity_scale"]
        obs[33:45] = action
        obs[45:47] = [np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)]
        action = policy(obs.copy())
        target = action * spec["action_scale"] + default
        records.append({"time_s": counter * dt, "obs": obs.tolist(), "action": action.tolist(),
                        "base_position_m": data.qpos[:3].tolist(),
                        "base_quaternion_wxyz": data.qpos[3:7].tolist()})
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("upstream", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--fixture", type=Path, help="Write the first policy steps' inputs/outputs")
    arguments = parser.parse_args()
    runs = []
    for command in COMMANDS:
        records = run(arguments.upstream, command, arguments.seconds)
        runs.append({"command": command, "summary": summarize(records, command),
                     "base_path_xy_m_10hz": [[round(v, 4) for v in r["base_position_m"][:2]]
                                             for r in records[4::5]]})
        print(json.dumps(runs[-1]["summary"]))
        if arguments.fixture and command == COMMANDS[0]:
            arguments.fixture.write_text(json.dumps(
                {"engine": f"mujoco {mujoco.__version__}", "command": command,
                 "steps": [{"obs": r["obs"], "action": r["action"]} for r in records[:50]]}) + "\n")
    arguments.output.write_text(json.dumps(
        {"schema_version": 1, "engine": f"mujoco {mujoco.__version__}",
         "scene": "unitree_rl_gym resources/robots/g1_description/scene.xml",
         "seconds": arguments.seconds, "runs": runs}, indent=2) + "\n")


if __name__ == "__main__":
    main()
