#!/usr/bin/env python3
"""Convert unitree_rl_gym's G1 walking policy into RNE's plain weight blob.

The TorchScript archive is read as a zip of raw little-endian float32
storages. Nothing in it is unpickled or executed: the storage-to-tensor map
below was read from a `pickletools` disassembly of `data.pkl` and every
entry is checked by byte size. The initial LSTM state storages must be zero.

Usage: extract_policy.py UNITREE_RL_GYM_ROOT OUTPUT_DIR
"""
import argparse
import hashlib
import json
import struct
import zipfile
from pathlib import Path

COMMIT = "276801e46c5d433564f24658bac64f254b7d2d4b"
SOURCE = "deploy/pre_train/g1/motion.pt"
SOURCE_SHA256 = "cf668f75b90d1abf73d2b87612a6e76bccc61ff7e083b63582d3f6aaa3c1759d"
# Storage key, tensor name, shape; blob order follows this list.
TENSORS = [
    ("6", "memory.weight_ih_l0", [256, 47]),
    ("7", "memory.weight_hh_l0", [256, 64]),
    ("8", "memory.bias_ih_l0", [256]),
    ("9", "memory.bias_hh_l0", [256]),
    ("2", "actor.0.weight", [32, 64]),
    ("3", "actor.0.bias", [32]),
    ("4", "actor.2.weight", [12, 32]),
    ("5", "actor.2.bias", [12]),
]
ZERO_STATE = [("0", "hidden_state", 64), ("1", "cell_state", 64)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("upstream", type=Path)
    parser.add_argument("output", type=Path)
    arguments = parser.parse_args()
    source = arguments.upstream / SOURCE
    raw = source.read_bytes()
    if hashlib.sha256(raw).hexdigest() != SOURCE_SHA256:
        raise ValueError(f"{source} is not the audited policy file")
    archive = zipfile.ZipFile(source)
    if archive.read("policy_lstm_1/byteorder") != b"little":
        raise ValueError("Policy storages are not little-endian")
    for key, name, count in ZERO_STATE:
        values = struct.unpack(f"<{count}f", archive.read(f"policy_lstm_1/data/{key}"))
        if any(values):
            raise ValueError(f"{name} is not zero")
    blob, table, offset = bytearray(), [], 0
    for key, name, shape in TENSORS:
        data = archive.read(f"policy_lstm_1/data/{key}")
        count = 1
        for dim in shape:
            count *= dim
        if len(data) != 4 * count:
            raise ValueError(f"{name}: {len(data)} bytes for shape {shape}")
        table.append({"name": name, "shape": shape, "offset_f32": offset})
        blob += data
        offset += count
    arguments.output.mkdir(parents=True, exist_ok=True)
    (arguments.output / "g1_walk_lstm.f32").write_bytes(bytes(blob))
    manifest = {
        "schema_version": 1,
        "source": {"repository": "unitreerobotics/unitree_rl_gym", "commit": COMMIT,
                   "path": SOURCE, "sha256": SOURCE_SHA256, "license": "BSD-3-Clause"},
        "architecture": "LSTM(47->64, 1 layer, gate order i,f,g,o; zero initial state) -> "
                        "Linear(64,32) -> ELU(alpha 1) -> Linear(32,12)",
        "weights": {"file": "g1_walk_lstm.f32", "encoding": "float32 little-endian",
                    "count": offset, "sha256": hashlib.sha256(blob).hexdigest(),
                    "tensors": table},
        "deployment": {
            "source": "deploy/deploy_mujoco/configs/g1.yaml",
            "physics_dt_s": 0.002, "control_decimation": 10, "gait_period_s": 0.8,
            "joint_order": [f"{side}_{joint}_joint" for side in ("left", "right")
                            for joint in ("hip_pitch", "hip_roll", "hip_yaw", "knee",
                                          "ankle_pitch", "ankle_roll")],
            "kp_nm_per_rad": [100, 100, 100, 150, 40, 40] * 2,
            "kd_nm_s_per_rad": [2, 2, 2, 4, 2, 2] * 2,
            "default_joint_rad": [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0] * 2,
            "action_scale": 0.25, "angular_velocity_scale": 0.25,
            "joint_velocity_scale": 0.05, "command_scale": [2.0, 2.0, 0.25],
            "observation": ["base angular velocity (pelvis frame) x3", "projected gravity x3",
                            "scaled command x3", "joint position - default x12",
                            "scaled joint velocity x12", "previous action x12",
                            "sin(2 pi phase)", "cos(2 pi phase)"],
        },
    }
    (arguments.output / "g1_walk_lstm.policy.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest["weights"]["sha256"]))


if __name__ == "__main__":
    main()
