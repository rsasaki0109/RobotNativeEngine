"""Recompute every H2 audit from its lossless full-state archive."""
import argparse
import hashlib
import itertools
import json
import lzma
from pathlib import Path
import statistics

import box


def verify(report):
    if (report["protocol"] != "box_contact_h2_v1" or report["status"] != "complete"
            or report["gates"] != box.GATES or report["columns"] != box.COLUMNS
            or report["script_sha256"] != hashlib.sha256(Path(box.__file__).read_bytes()).hexdigest()):
        raise ValueError("protocol or evaluator differs")
    expected = set(itertools.product((1_000_000, 2_000_000), (0, 0.1, 0.4),
                                     (0.1, 0.5), (0.0, 0.001), (20, 200)))
    seen = set()
    for row in report["results"]:
        c, s = row["case"], row["setting"]
        key = (c["dt_ticks"], c["angle_rad"], c["gap_m"], s["regularization"], s["iterations"])
        if key in seen or c["mass_kg"] != 1:
            raise ValueError("duplicate case or wrong mass")
        seen.add(key)
        if box.metrics(c, row["audit"]["trace"]) != row["metrics"]:
            raise ValueError("stored metrics differ from trace")
        if (len(row["seconds_per_simulated_second"]) != 5
                or any(not 0 < v < float("inf") for v in row["seconds_per_simulated_second"])
                or statistics.median(row["seconds_per_simulated_second"]) != row["median_cost"]):
            raise ValueError("invalid cost samples")
        if row["audit"]["debug_assertions"] or not row["exact_replay"]:
            raise ValueError("release/replay requirement missing")
        if row["audit"]["trace"][-1][1:14] != row["audit"]["final_state"][1:14]:
            raise ValueError("final state differs")
    if seen != expected:
        raise ValueError("incomplete grid")
    return len(seen)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("--require-pass", action="store_true")
    args = parser.parse_args()
    with lzma.open(args.archive, "rt") as stream:
        report = json.load(stream)
    count = verify(report)
    print(f"Verified {count} full-pose records")
    if args.require_pass and not all(r["metrics"]["passed"] for r in report["results"]):
        raise SystemExit("H2 quality gate failed; failure evidence retained")


if __name__ == "__main__":
    main()
