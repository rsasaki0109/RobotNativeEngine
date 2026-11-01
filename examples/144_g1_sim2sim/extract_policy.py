#!/usr/bin/env python3
"""Convert a unitree_rl_gym walking policy into RNE's plain weight blob.

The TorchScript archive is read as a zip of raw little-endian float32
storages. Nothing in it is unpickled or executed: the storage-to-tensor map
below was read from a `pickletools` disassembly of `data.pkl` (the G1, H1 and
H1_2 policies share it) and every entry is checked by byte size. The initial
LSTM state storages must be zero. The deployment contract comes from the
robot's `deploy/deploy_mujoco/configs/<robot>.yaml` and the actuator order of
its MJCF.

Usage: extract_policy.py UNITREE_RL_GYM_ROOT OUTPUT_DIR [--robot g1|h1]
"""
import argparse
import hashlib
import json
import struct
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

import yaml

COMMIT = "276801e46c5d433564f24658bac64f254b7d2d4b"
ROBOTS = {
    "g1": {"source_sha256": "cf668f75b90d1abf73d2b87612a6e76bccc61ff7e083b63582d3f6aaa3c1759d",
           "mjcf": "resources/robots/g1_description/g1_12dof.xml"},
    "h1": {"source_sha256": "44a0fbceb81f3877833ae9a398d039bea1759cb0d3c8188181013885f70589eb",
           "mjcf": "resources/robots/h1/h1.xml"},
}
# deploy_mujoco.py uses one gait period for every robot.
GAIT_PERIOD_S = 0.8
HIDDEN = 64
FEATURES = 32


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("upstream", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--robot", choices=sorted(ROBOTS), default="g1")
    arguments = parser.parse_args()
    robot = ROBOTS[arguments.robot]
    source_path = f"deploy/pre_train/{arguments.robot}/motion.pt"
    config_path = f"deploy/deploy_mujoco/configs/{arguments.robot}.yaml"
    source = arguments.upstream / source_path
    raw = source.read_bytes()
    if hashlib.sha256(raw).hexdigest() != robot["source_sha256"]:
        raise ValueError(f"{source} is not the audited policy file")
    config = yaml.safe_load((arguments.upstream / config_path).read_text())
    inputs, actions = config["num_obs"], config["num_actions"]
    if inputs != 9 + 3 * actions + 2:
        raise ValueError("Observation is not unitree_rl_gym's walking layout")
    joints = [motor.get("joint") for motor in
              ET.parse(arguments.upstream / robot["mjcf"]).getroot().iter("motor")]
    if len(joints) != actions:
        raise ValueError(f"MJCF has {len(joints)} motors for {actions} actions")
    # Storage key, tensor name, shape; blob order follows this list.
    tensors = [
        ("6", "memory.weight_ih_l0", [4 * HIDDEN, inputs]),
        ("7", "memory.weight_hh_l0", [4 * HIDDEN, HIDDEN]),
        ("8", "memory.bias_ih_l0", [4 * HIDDEN]),
        ("9", "memory.bias_hh_l0", [4 * HIDDEN]),
        ("2", "actor.0.weight", [FEATURES, HIDDEN]),
        ("3", "actor.0.bias", [FEATURES]),
        ("4", "actor.2.weight", [actions, FEATURES]),
        ("5", "actor.2.bias", [actions]),
    ]
    archive = zipfile.ZipFile(source)
    if archive.read("policy_lstm_1/byteorder") != b"little":
        raise ValueError("Policy storages are not little-endian")
    for key, name in [("0", "hidden_state"), ("1", "cell_state")]:
        values = struct.unpack(f"<{HIDDEN}f", archive.read(f"policy_lstm_1/data/{key}"))
        if any(values):
            raise ValueError(f"{name} is not zero")
    blob, table, offset = bytearray(), [], 0
    for key, name, shape in tensors:
        data = archive.read(f"policy_lstm_1/data/{key}")
        count = 1
        for dim in shape:
            count *= dim
        if len(data) != 4 * count:
            raise ValueError(f"{name}: {len(data)} bytes for shape {shape}")
        table.append({"name": name, "shape": shape, "offset_f32": offset})
        blob += data
        offset += count
    stem = f"{arguments.robot}_walk_lstm"
    arguments.output.mkdir(parents=True, exist_ok=True)
    (arguments.output / f"{stem}.f32").write_bytes(bytes(blob))
    manifest = {
        "schema_version": 1,
        "source": {"repository": "unitreerobotics/unitree_rl_gym", "commit": COMMIT,
                   "path": source_path, "sha256": robot["source_sha256"], "license": "BSD-3-Clause"},
        "architecture": f"LSTM({inputs}->{HIDDEN}, 1 layer, gate order i,f,g,o; zero initial state) -> "
                        f"Linear({HIDDEN},{FEATURES}) -> ELU(alpha 1) -> Linear({FEATURES},{actions})",
        "weights": {"file": f"{stem}.f32", "encoding": "float32 little-endian",
                    "count": offset, "sha256": hashlib.sha256(blob).hexdigest(),
                    "tensors": table},
        "deployment": {
            "source": config_path,
            "physics_dt_s": config["simulation_dt"], "control_decimation": config["control_decimation"],
            "gait_period_s": GAIT_PERIOD_S,
            "joint_order": joints,
            "kp_nm_per_rad": config["kps"],
            "kd_nm_s_per_rad": config["kds"],
            "default_joint_rad": config["default_angles"],
            "action_scale": config["action_scale"], "angular_velocity_scale": config["ang_vel_scale"],
            "joint_velocity_scale": config["dof_vel_scale"], "command_scale": config["cmd_scale"],
            "observation": ["base angular velocity (pelvis frame) x3", "projected gravity x3",
                            "scaled command x3", f"joint position - default x{actions}",
                            f"scaled joint velocity x{actions}", f"previous action x{actions}",
                            "sin(2 pi phase)", "cos(2 pi phase)"],
        },
    }
    if config["dof_pos_scale"] != 1.0:
        raise ValueError("RNE's walker assumes unscaled joint positions")
    (arguments.output / f"{stem}.policy.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest["weights"]["sha256"]))


if __name__ == "__main__":
    main()
