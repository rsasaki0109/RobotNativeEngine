#!/usr/bin/env python3
"""Pairwise agreement of push sweeps, and where each falls earlier or later.

Reads any number of push sweeps of one robot (example 144's `--robustness`,
mujoco_robustness.py, mujoco_ablation.py), each given as LABEL=FILE, and
prints the share of the 640 trials on which every pair agrees about falling.
For each sweep and push direction it then prints how many force levels later
than the first sweep it first falls, averaged over the eight gait phases
(negative: earlier), and its number of falls. `--json OUT` also writes both.
Uses no third-party packages.

Usage: push_agreement.py LABEL=SWEEP.json LABEL=SWEEP.json ... [--json OUT.json]
"""
import argparse
import json
from pathlib import Path

from robustness_compare import fall_force_shift, key


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("sweeps", nargs="+", help="LABEL=SWEEP.json, the reference first")
    parser.add_argument("--json", type=Path)
    arguments = parser.parse_args()
    sweeps = []
    for spec in arguments.sweeps:
        label, path = spec.split("=", 1)
        sweeps.append((label, json.loads(Path(path).read_text())))
    robots = {sweep["robot"] for _, sweep in sweeps}
    protocols = {json.dumps(sweep["protocol"], sort_keys=True) for _, sweep in sweeps}
    if len(robots) != 1 or len(protocols) != 1:
        raise SystemExit("the sweeps are of different robots or protocols")
    fell = {label: {key(t): t["fell"] for t in sweep["push"]} for label, sweep in sweeps}
    labels = [label for label, _ in sweeps]
    agreement = {a: {b: sum(fell[a][k] == fell[b][k] for k in fell[a]) / len(fell[a])
                     for b in labels} for a in labels}
    width = max(len(label) for label in labels)
    print(" " * width + "".join(f" {label[:10]:>10s}" for label in labels))
    for a in labels:
        print(f"{a:{width}s}" + "".join(f" {100 * agreement[a][b]:9.1f}%" for b in labels))
    reference = sweeps[0][1]
    shifts = {}
    print(f"force levels later than {labels[0]} that each first falls, by direction; falls")
    for label, sweep in sweeps:
        shifts[label] = fall_force_shift(sweep, reference)
        falls = sum(fell[label].values())
        print(f"{label:{width}s} " + " ".join(f"{d} {v:+.2f}" for d, v in shifts[label].items())
              + f"  {falls}")
    if arguments.json:
        arguments.json.write_text(json.dumps(
            {"schema_version": 1, "robot": robots.pop(), "reference": labels[0],
             "agreement": agreement, "fall_force_shift_levels": shifts,
             "falls": {label: sum(fell[label].values()) for label in labels}}, indent=2) + "\n")


if __name__ == "__main__":
    main()
