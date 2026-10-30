"""H1 v2/v3: frozen staged search for matched-quality sphere-drop cost.

Uses unmodified H1 v1 runners, metrics and gates. Prior 0.5 m failures inform
calibration; new validation cases never participate in selection.
"""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import platform
import statistics

import mujoco
import numpy as np

import drop

TIMESTEPS = (250_000, 125_000, 62_500, 31_250, 15_625, 10_000, 5_000, 2_500)
SOLREFS = ((0.000002, 1), (-1e10, -200000), (-1e12, -2000000))
CALIBRATION = ((1.0, 0.1), (1.0, 0.5))
VALIDATION = ((0.3, 0.03), (0.3, 0.3), (3.0, 0.03), (3.0, 0.3))
NATIVE_SETTING = {"iterations": 20, "regularization": 0.0}


def choose(candidates):
    """Choose only among settings that passed BOTH calibration cases."""
    passing = [c for c in candidates if all(r["metrics"]["passed"] for r in c["records"])
               and len(c["records"]) == len(CALIBRATION)]
    return min(passing, key=lambda c: statistics.mean(r["median_cost"] for r in c["records"]),
               default=None)


def save_trace(directory, record, index):
    """Store lossless full float64 samples separately to bound report memory."""
    trace = np.asarray(record["audit"].pop("trace"), dtype="<f8")
    path = directory / f"trace-{index:03d}.npy.gz"
    with path.open("xb") as output:
        with gzip.GzipFile(fileobj=output, mode="wb", mtime=0) as compressed:
            np.save(compressed, trace, allow_pickle=False)
    record["audit"]["trace_file"] = path.name
    record["audit"]["trace_shape"] = list(trace.shape)
    record["audit"]["trace_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()


def comparison(native_records, mujoco_records):
    """No speedup for failed quality; preserve each case and raw timing samples."""
    if len(native_records) != len(VALIDATION) or len(mujoco_records) != len(VALIDATION):
        return {"eligible": False, "reason": "missing validation"}
    for a, b in zip(native_records, mujoco_records):
        if (a["case"]["mass_kg"], a["case"]["gap_m"]) != (b["case"]["mass_kg"], b["case"]["gap_m"]):
            raise ValueError("validation cases differ")
    if not all(r["metrics"]["passed"] for r in native_records + mujoco_records):
        return {"eligible": False, "reason": "validation quality failure"}
    return {"eligible": True,
            "ratio_definition": "MuJoCo cost / native cost; >1 favors native, batch loop only",
            "equal_weight_mean_median_cost_ratio": statistics.mean(r["median_cost"] for r in mujoco_records)
                / statistics.mean(r["median_cost"] for r in native_records),
            "cases": [{"mass_kg": a["case"]["mass_kg"], "gap_m": a["case"]["gap_m"],
                       "median_cost_ratio": b["median_cost"] / a["median_cost"],
                       "observed_range_ratio": [b["min_cost"] / a["max_cost"],
                                                b["max_cost"] / a["min_cost"]]}
                      for a, b in zip(native_records, mujoco_records)]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--overdamped", action="store_true",
                        help="H1 v3: separately frozen overdamped search following v2 rebound failures")
    args = parser.parse_args()
    timesteps = (31_250, 15_625, 10_000, 5_000, 2_500) if args.overdamped else TIMESTEPS
    solrefs = ((0.000002, 10), (0.000002, 20)) if args.overdamped else SOLREFS
    if args.output_dir.exists():
        parser.error("provide a new output directory")
    if mujoco.__version__ != "3.9.0":
        parser.error("requires MuJoCo 3.9.0")
    binary = args.binary.resolve()
    os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    args.output_dir.mkdir(parents=True)
    report = {"protocol": "hard_contact_drop_h1_v3" if args.overdamped else "hard_contact_drop_h1_v2", "status": "running",
              "gates": drop.GATES, "duration_s": drop.DURATION,
              "radius_m": drop.RADIUS, "gravity_m_s2": [0, -drop.G, 0],
              "platform": platform.platform(), "python": platform.python_version(),
              "mujoco": mujoco.__version__, "cpu_affinity": sorted(os.sched_getaffinity(0)),
              "binary_sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
              "source_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in (Path(__file__), Path(drop.__file__))},
              "planned_timesteps_ticks": timesteps, "planned_solrefs": solrefs,
              "calibration_cases": CALIBRATION, "validation_cases": VALIDATION,
              "selection": "all planned settings per stage; first stage with both calibration cases passing; lowest mean median cost within that stage",
              "timing_scope": "H1 v1 release backend batch loop; one warm-up, five fresh-world samples; no ECS or per-step IPC; one logical CPU",
              "records": [], "selected_mujoco": None}

    def checkpoint():
        (args.output_dir / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")

    def measure(backend, dt, setting, cases, phase):
        records = []
        for mass, height in cases:
            case = {"dt_ticks": dt, "mass_kg": mass, "gap_m": height}
            record = drop.run_case(binary, backend, case, setting, 5)
            record["phase"] = phase
            save_trace(args.output_dir, record, len(report["records"]))
            report["records"].append(record)
            records.append(record)
            checkpoint()
            print(json.dumps({"backend": backend, "phase": phase, "case": case,
                              "setting": setting, "passed": record["metrics"]["passed"],
                              "penetration_um": record["metrics"]["max_penetration_m"] * 1e6,
                              "median_cost": record["median_cost"]}), flush=True)
        return records

    checkpoint()
    native_calibration = measure("native", 2_000_000, NATIVE_SETTING, CALIBRATION, "calibration")
    chosen = None
    for dt in timesteps:
        candidates = []
        for solref in solrefs:
            setting = {"iterations": 20, "solref": list(solref)}
            records = measure("mujoco", dt, setting, CALIBRATION, "calibration")
            candidates.append({"dt_ticks": dt, "setting": setting, "records": records})
        chosen = choose(candidates)
        if chosen is not None:
            report["selected_mujoco"] = {"dt_ticks": dt, "setting": chosen["setting"]}
            break
    native_validation, mj_validation = [], []
    if chosen is not None and all(r["metrics"]["passed"] for r in native_calibration):
        # Alternate backend order by case; selection is now frozen.
        for i, case in enumerate(VALIDATION):
            order = ("native", "mujoco") if i % 2 == 0 else ("mujoco", "native")
            for backend in order:
                if backend == "native":
                    native_validation += measure(backend, 2_000_000, NATIVE_SETTING, [case], "validation")
                else:
                    mj_validation += measure(backend, chosen["dt_ticks"], chosen["setting"], [case], "validation")
    report["comparison"] = comparison(native_validation, mj_validation)
    report["status"] = "complete"
    checkpoint()
    print(json.dumps(report["comparison"]), flush=True)


if __name__ == "__main__":
    main()
