#!/usr/bin/env python3
"""Run example 138's identical ECS scene through three real physics backends.

Build the optional MuJoCo feature first; see docs/HEAVY_GRASP_COMPARISON.md.
Task failures remain in the report. Timings include ECS synchronization and
first-step setup, and are not steady-state solver-only microbenchmarks.
"""

import argparse
import hashlib
import json
import math
import os
import platform
from pathlib import Path
import statistics
import subprocess

BACKENDS = ("native", "rapier", "mujoco")
EXPECTED_CASES = {(mass, case) for mass in (5, 20, 50)
                  for case in ("held", "under-squeezed")}


def run(binary, backend):
    output = subprocess.check_output(
        [str(binary), "--benchmark-json", backend], text=True, timeout=120
    )
    result = json.loads(output)
    if result.get("schema_version") != 1 or result.get("backend") != backend:
        raise ValueError("unexpected benchmark schema or backend")
    cases = result["cases"]
    if len(cases) != 6 or {(c["mass_kg"], c["case"]) for c in cases} != EXPECTED_CASES:
        raise ValueError("missing, duplicate, or unexpected comparison case")
    for case in cases:
        for field in ("lifted_m", "palm_rise_m", "slip_m", "tilt_rad", "step_us"):
            if not isinstance(case[field], (int, float)) or not math.isfinite(case[field]):
                raise ValueError(f"non-finite result: {backend} {field}")
        if case["step_us"] <= 0 or not isinstance(case["accepted"], bool):
            raise ValueError("invalid timing or acceptance value")
    return result


def physical(case):
    return {key: value for key, value in case.items() if key != "step_us"}


def summarize(trials):
    rows = []
    for backend in BACKENDS:
        results = [trial[backend] for trial in trials]
        for mass, label in sorted(EXPECTED_CASES):
            cases = [next(c for c in result["cases"]
                          if c["mass_kg"] == mass and c["case"] == label)
                     for result in results]
            times = [case["step_us"] for case in cases]
            rows.append({
                "backend": backend, "mass_kg": mass, "case": label,
                "accepted_every_run": all(c["accepted"] for c in cases),
                "exact_final_outputs_repeat": all(physical(c) == physical(cases[0])
                                                  for c in cases[1:]),
                "step_us_median": statistics.median(times),
                "step_us_min": min(times), "step_us_max": max(times),
                "physical": physical(cases[0]),
            })
    return rows


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path,
                        default=Path("target/release/examples/138_heavy_grasp"))
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--runtime-library", type=Path, required=True,
                        help="MuJoCo shared library selected by the platform loader")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.runs < 2:
        parser.error("--runs must be at least 2 to check repeatability")
    binary = args.binary.resolve(strict=True)
    runtime_library = args.runtime_library.resolve(strict=True)
    root = Path(__file__).resolve().parents[1]
    # Warm the process/library/filesystem caches, without retaining these runs.
    for backend in BACKENDS:
        run(binary, backend)
    trials = []
    orders = []
    for trial in range(args.runs):
        order = BACKENDS if trial % 2 == 0 else tuple(reversed(BACKENDS))
        orders.append(list(order))
        trials.append({backend: run(binary, backend) for backend in order})
    sources = [root / "examples/138_heavy_grasp/main.rs",
               root / "examples/138_heavy_grasp/Cargo.toml", root / "Cargo.lock",
               Path(__file__).resolve()]
    cpu = "unavailable"
    if Path("/proc/cpuinfo").exists():
        cpu = next((line.split(":", 1)[1].strip()
                    for line in Path("/proc/cpuinfo").read_text().splitlines()
                    if line.startswith("model name")), cpu)
    report = {
        "schema_version": 1,
        "scope": "static-base example 138; identical ECS scene and commands",
        "limitations": [
            "Same-host diagnostic; no claim about latest MuJoCo, GPU batches, or other tasks.",
            "Final output equality only; not a complete trajectory or cross-platform replay proof.",
            "Contact models and motor integration differ; failed cases are retained.",
            "Timing includes first-step model compilation and ECS sync, not only solver work.",
        ],
        "environment": {"platform": platform.platform(), "cpu": cpu,
                        "python": platform.python_version(),
                        "cpu_affinity": sorted(os.sched_getaffinity(0))
                        if hasattr(os, "sched_getaffinity") else None},
        "binary_sha256": sha256(binary),
        "mujoco_library_sha256": sha256(runtime_library),
        "mujoco_library_name": runtime_library.name,
        "rustc": subprocess.check_output(["rustc", "-vV"], text=True).strip(),
        "git_base_revision": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
        "tracked_diff_sha256": hashlib.sha256(subprocess.check_output(
            ["git", "diff", "HEAD", "--"], cwd=root)).hexdigest(),
        "source_sha256": {str(path.relative_to(root)): sha256(path) for path in sources},
        "run_orders": orders, "warmup_runs_per_backend": 1,
        "summary": summarize(trials), "trials": trials,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print("backend mass_kg case accepted median_us min_us max_us final_repeat")
    for row in report["summary"]:
        print(row["backend"], row["mass_kg"], row["case"], row["accepted_every_run"],
              *(f'{row[key]:.2f}' for key in
                ("step_us_median", "step_us_min", "step_us_max")),
              row["exact_final_outputs_repeat"])
    native = [row for row in report["summary"] if row["backend"] == "native"]
    if not all(row["accepted_every_run"] and row["exact_final_outputs_repeat"]
               for row in native):
        raise SystemExit("native correctness or final-output repeatability failed; report retained")


if __name__ == "__main__":
    main()
