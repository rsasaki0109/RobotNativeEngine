#!/usr/bin/env python3
"""Write the vendored G1 walking policy as a standard ONNX graph.

The float32 blob in assets/policies/unitree_g1_walk becomes an ONNX LSTM
(gate order converted from PyTorch's i,f,g,o to ONNX's i,o,f,c) followed by
Gemm -> Elu -> Gemm. The recurrent state is an explicit input and output, so
a caller steps it once per policy period. Needs the `onnx` package; only
weights are read, nothing is executed.

Usage: g1_policy_to_onnx.py OUTPUT.onnx
"""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

POLICY = Path(__file__).resolve().parents[2] / "assets/policies/unitree_g1_walk"


def main():
    manifest = json.loads((POLICY / "g1_walk_lstm.policy.json").read_text())
    weights = manifest["weights"]
    blob = (POLICY / weights["file"]).read_bytes()
    if hashlib.sha256(blob).hexdigest() != weights["sha256"]:
        raise ValueError("Weight blob does not match its manifest")
    values = np.frombuffer(blob, "<f4")
    t = {}
    for tensor in weights["tensors"]:
        count = int(np.prod(tensor["shape"]))
        t[tensor["name"]] = values[tensor["offset_f32"]:tensor["offset_f32"] + count].reshape(tensor["shape"])
    hidden = t["memory.weight_hh_l0"].shape[1]
    inputs = t["memory.weight_ih_l0"].shape[1]

    def onnx_gates(matrix):
        i, f, g, o = np.split(matrix, 4, axis=0)
        return np.concatenate([i, o, f, g], axis=0)

    w = onnx_gates(t["memory.weight_ih_l0"])[None]
    r = onnx_gates(t["memory.weight_hh_l0"])[None]
    b = np.concatenate([onnx_gates(t["memory.bias_ih_l0"]), onnx_gates(t["memory.bias_hh_l0"])])[None]
    initializers = [
        numpy_helper.from_array(w.astype(np.float32), "W"),
        numpy_helper.from_array(r.astype(np.float32), "R"),
        numpy_helper.from_array(b.astype(np.float32), "B"),
        numpy_helper.from_array(t["actor.0.weight"].astype(np.float32), "W0"),
        numpy_helper.from_array(t["actor.0.bias"].astype(np.float32), "B0"),
        numpy_helper.from_array(t["actor.2.weight"].astype(np.float32), "W1"),
        numpy_helper.from_array(t["actor.2.bias"].astype(np.float32), "B1"),
        numpy_helper.from_array(np.array([1, hidden], np.int64), "hidden_shape"),
    ]
    nodes = [
        helper.make_node("LSTM", ["observation", "W", "R", "B", "", "h_in", "c_in"],
                         ["", "h_out", "c_out"], hidden_size=hidden),
        helper.make_node("Reshape", ["h_out", "hidden_shape"], ["features_in"]),
        helper.make_node("Gemm", ["features_in", "W0", "B0"], ["pre_elu"], transB=1),
        helper.make_node("Elu", ["pre_elu"], ["features"], alpha=1.0),
        helper.make_node("Gemm", ["features", "W1", "B1"], ["action"], transB=1),
    ]
    actions = t["actor.2.bias"].shape[0]
    graph = helper.make_graph(
        nodes, "unitree_g1_walk_lstm",
        [helper.make_tensor_value_info("observation", TensorProto.FLOAT, [1, 1, inputs]),
         helper.make_tensor_value_info("h_in", TensorProto.FLOAT, [1, 1, hidden]),
         helper.make_tensor_value_info("c_in", TensorProto.FLOAT, [1, 1, hidden])],
        [helper.make_tensor_value_info("action", TensorProto.FLOAT, [1, actions]),
         helper.make_tensor_value_info("h_out", TensorProto.FLOAT, [1, 1, hidden]),
         helper.make_tensor_value_info("c_out", TensorProto.FLOAT, [1, 1, hidden])],
        initializers)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)],
                              producer_name="rne g1_policy_to_onnx.py")
    model.ir_version = 8
    onnx.checker.check_model(model)
    output = Path(sys.argv[1])
    output.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, output)
    print(output, output.stat().st_size, "bytes")


if __name__ == "__main__":
    main()
