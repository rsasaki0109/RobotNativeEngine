"""Six-condition shoulder-pan response audit, optionally against an older binary."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import platform

import numpy as np

from benchmark_mujoco import compare, normalize_urdf, run_mujoco, run_rne


def requests(urdf, names):
    for amplitude in (0.001, 0.01):
        for dt_ticks in (1_000_000, 500_000, 250_000):
            torques = np.zeros((200_000_000 // dt_ticks, len(names)))
            j = names.index("shoulder_pan")
            torques[20_000_000 // dt_ticks:60_000_000 // dt_ticks, j] = amplitude
            torques[60_000_000 // dt_ticks:100_000_000 // dt_ticks, j] = -amplitude
            yield amplitude, {"urdf": urdf, "joint_names": names, "dt_ticks": dt_ticks,
                              "torques_nm": torques.tolist()}


def initial_acceleration(result, dt_ticks):
    """Finite difference over the first driven step, in rad/s², for every joint."""
    k = 20_000_000 // dt_ticks
    velocities = np.asarray(result["dq_rad_s"])
    return ((velocities[k + 1] - velocities[k]) / (dt_ticks * 1e-9)).tolist()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--reference-binary", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("provide a new output path")
    source = args.source.read_bytes()
    urdf, names = normalize_urdf(source.decode())
    binaries = {"rne": args.binary.resolve()}
    if args.reference_binary:
        binaries["reference_rne"] = args.reference_binary.resolve()
    report = {"audit": "so101_pan_zero_loss_v1", "platform": platform.platform(),
              "source_urdf_sha256": hashlib.sha256(source).hexdigest(),
              "normalized_urdf": urdf, "normalized_urdf_sha256": hashlib.sha256(urdf.encode()).hexdigest(),
              "joint_names": names,
              "binary_sha256": {k: hashlib.sha256(p.read_bytes()).hexdigest() for k, p in binaries.items()},
              "conditions": "fixed base, zero gravity, no contact/stops/losses; 200 ms biphasic pulse",
              "results": []}
    for amplitude, request in requests(urdf, names):
        responses = {k: run_rne(p, request) for k, p in binaries.items()}
        responses["mujoco"] = run_mujoco(request)
        comparisons = {k: compare(responses[k], responses["mujoco"]) for k in binaries}
        record = {"amplitude_nm": amplitude, "dt_ticks": request["dt_ticks"],
                  "torques_nm": request["torques_nm"], "responses": responses,
                  "first_step_acceleration_rad_s2": {
                      k: initial_acceleration(r, request["dt_ticks"]) for k, r in responses.items()},
                  "comparisons": comparisons}
        report["results"].append(record)
        print(json.dumps({"amplitude_nm": amplitude, "dt_ticks": request["dt_ticks"],
                          "max_angle_difference_rad": {k: max(c["max_abs_difference_rad"])
                                                       for k, c in comparisons.items()}}), flush=True)
    raw = (json.dumps(report, allow_nan=False) + "\n").encode()
    with args.output.open("xb") as output:
        output.write(gzip.compress(raw, mtime=0) if args.output.suffix == ".gz" else raw)


if __name__ == "__main__":
    main()
