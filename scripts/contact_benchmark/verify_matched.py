"""Verify a matched H1 report against its lossless trace directory."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np

import drop
from matched import comparison


def verify(report_path, traces):
    report = json.loads(report_path.read_text())
    if report["status"] != "complete" or report["gates"] != drop.GATES:
        raise ValueError("incomplete report or different quality gates")
    if report["source_sha256"]["drop.py"] != hashlib.sha256(Path(drop.__file__).read_bytes()).hexdigest():
        raise ValueError("quality evaluator source differs")
    for record in report["records"]:
        audit = record["audit"]
        name = audit["trace_file"]
        if Path(name).name != name:
            raise ValueError("invalid trace filename")
        path = traces / name
        if hashlib.sha256(path.read_bytes()).hexdigest() != audit["trace_sha256"]:
            raise ValueError(f"trace checksum mismatch: {name}")
        with gzip.open(path, "rb") as stream:
            values = np.load(stream, allow_pickle=False)
        if list(values.shape) != audit["trace_shape"]:
            raise ValueError("trace shape differs")
        quality = drop.metrics(record["case"], values)
        quality["gates"]["backend_health"] = not any(audit.get("warnings", [])) and abs(
            audit.get("simulation_time_s", drop.DURATION) - drop.DURATION) < 1e-8
        quality["passed"] = all(quality["gates"].values())
        if quality != record["metrics"]:
            raise ValueError(f"quality metrics differ: {name}")
        if tuple(values[-1, 1:3]) != (audit["final_gap_m"], audit["final_velocity_m_s"]):
            raise ValueError("final state differs from trace")
    validation = [r for r in report["records"] if r["phase"] == "validation"]
    expected = comparison([r for r in validation if r["backend"] == "native"],
                          [r for r in validation if r["backend"] == "mujoco"])
    if expected != report["comparison"]:
        raise ValueError("comparison summary differs")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("--traces", type=Path)
    parser.add_argument("--require-comparison", action="store_true")
    args = parser.parse_args()
    report = verify(args.report, args.traces or args.report.parent)
    if args.require_comparison and not report["comparison"]["eligible"]:
        raise SystemExit("no eligible quality-matched comparison")
    print(f"Verified {len(report['records'])} complete traces and quality/cost summary")


if __name__ == "__main__":
    main()
