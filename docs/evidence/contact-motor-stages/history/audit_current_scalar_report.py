#!/usr/bin/env python3
"""Pure preregistered 72-world source-object transition; no physics reruns."""

import argparse
import datetime
import hashlib
import json
from pathlib import Path
import sys

from stage_schema import artifact, keys, require, sha_bytes, strict_json
from audit_coupled_stages import record_for, stamp, source_map

BASELINE_SHA = "a2f9b90b964123b1ee5dc69d2f2d10e320882c2b3317d6f49b0c8b490dcef8b1"
ADDITIONS = {
    "third_party/rapier3d/src/pipeline/multibody_motor_stage.rs",
    "third_party/rapier3d/src/pipeline/mod.rs",
    "third_party/rapier3d/src/dynamics/solver/joint_constraint/joint_generic_constraint_builder.rs",
    "third_party/rapier3d/src/dynamics/solver/joint_constraint/joint_constraints_set.rs",
}
CHANGES = {"Cargo.toml", "Cargo.lock", "examples/141_position_motor_calibration/lib.rs",
           "third_party/rapier3d/src/dynamics/solver/velocity_solver.rs",
           "third_party/rapier3d/src/dynamics/solver/island_solver.rs",
           "third_party/rapier3d/src/pipeline/physics_pipeline.rs"}
STDOUT = b"collection_complete=true completed_worlds=72/72 replay_qualification=true primary_calibration=FAIL mode=collect\n"


def load_report(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, "duplicate report JSON key")
            result[key] = value
        return result
    def reject(value):
        raise ValueError("nonstandard numeric report constant: " + value)
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=reject)


def expected_bytes(original, before, after):
    original_doc = load_report(original)
    require(original_doc["selected_source_sha256"] == before and len(before) == 17 and len(after) == 21,
            "scalar source-before or registered 17/21 scope differs")
    source_map(before); source_map(after)
    require(set(after) - set(before) == ADDITIONS and not (set(before) - set(after)), "scalar selected key additions/removals differ")
    require({name for name in before if before[name] != after[name]} == CHANGES, "scalar existing source hash changes differ from preregistered six")
    marker = b'  "selected_source_sha256": {'
    start = original.index(marker)
    end = original.index(b'\n  },\n  "cases":', start) + len(b'\n  }')
    replacement = b'  "selected_source_sha256": ' + json.dumps(dict(sorted(after.items())), indent=2).encode().replace(b"\n", b"\n  ")
    return original[:start] + replacement + original[end:]


def compare_reports(original, current, before, after):
    require(current == expected_bytes(original, before, after),
            "scalar report contains forbidden raw bytes outside registered selected source object")
    return load_report(current)


def coverage(report):
    expected_summary = {"collection_complete": True, "worlds_attempted": 72, "worlds_created": 72,
                        "completed_worlds": 72, "replay_qualification_pass": True, "primary_calibration_pass": False,
                        "primary_nonzero_cells": 4, "primary_nonzero_passes": 0, "zero_target_cells": 8, "zero_target_passes": 8}
    require(report["schema_version"] == 2 and report["summary"] == expected_summary and len(report["cases"]) == 24 and
            len(report["numerical_sensitivity"]) == 24, "original schema/matrix/complete replay/accuracy FAIL changed")
    states = entities = native_bytes = state_words = enabled_runs = telemetry_words = slices = zero = nonzero_passes = primary = 0
    cells = set()
    for case in report["cases"]:
        cell = case["cell"]
        cell_id = (cell["gain"], cell["target_position_rad"]["native_f32_bits"], cell["solver_small_steps"], cell["internal_pgs_passes"])
        require(cell_id not in cells, "duplicate scalar cell"); cells.add(cell_id)
        require(len(case["runs"]) == 3 and [run["collector_enabled"] for run in case["runs"]] == [True, True, False], "scalar run matrix changed")
        first = case["runs"][0]
        for run in case["runs"]:
            require(run["world_created"] and run["completed_step"] and run["error"] is None, "scalar partial/errored world")
            for field in ["initial", "final_state"]:
                require(run[field] == first[field] and run[field] is not None, "scalar state replay changed")
                states += 1; state_words += len(run[field]["canonical_words"])
                for entity in run[field]["native_entities"]:
                    data = bytes(entity["bytes"])
                    require(sha_bytes(data) == entity["sha256"], "scalar native entity hash differs")
                    entities += 1; native_bytes += len(data)
            if run["collector_enabled"]:
                require(run["telemetry"] == first["telemetry"] and run["telemetry"] is not None, "scalar ordered telemetry differs")
                enabled_runs += 1; telemetry_words += len(run["telemetry"]["canonical_words"])
                for motor in run["telemetry"]["motors"]:
                    require(motor["full_step_coverage"] and len(motor["slices"]) == cell["solver_small_steps"], "scalar motor coverage incomplete")
                    slices += len(motor["slices"])
            else:
                require(run["telemetry"] is None, "scalar OFF telemetry present")
        assessment = case["assessment"]
        primary += int(assessment["primary_cell"])
        zero += int(cell["target_position_rad"]["finite"] == 0)
        nonzero_passes += int(cell["target_position_rad"]["finite"] != 0 and assessment["first_slice_tolerance_pass"])
    require(primary == 4 and zero == 8 and nonzero_passes == 0, "original primary/zero/accuracy outcome changed")
    return {"cells": 24, "worlds": 72, "native_states": states, "native_entities": entities,
            "native_entity_bytes": native_bytes, "native_state_canonical_words": state_words,
            "enabled_telemetry_runs": enabled_runs, "ordered_telemetry_slices": slices,
            "telemetry_canonical_words": telemetry_words, "primary_nonzero_passes": 0,
            "all_nonzero_tolerance_passes": 0, "zero_target_passes": 8}


def audit(args):
    protocol_record = record_for(args.protocol)
    require(protocol_record["sha256"] == args.protocol_sha256, "scalar protocol literal SHA differs")
    protocol = strict_json(Path(args.protocol).read_bytes())
    frozen = {Path(record["path"]).resolve(): record for record in protocol["reader_artifacts"]}
    for name in ["audit_current_scalar_report.py", "audit_coupled_stages.py", "stage_analysis.py", "stage_schema.py", "stage_numeric.py"]:
        actual = record_for(Path(__file__).parent / name)
        require(frozen.get(Path(actual["path"])) == actual, "scalar reader dependency changed after freeze")
    scalar = protocol["scalar_qualification"]
    artifact(scalar["baseline_report"])
    require(scalar["baseline_report"]["sha256"] == BASELINE_SHA, "old vendor-lint baseline changed")
    original = Path(scalar["baseline_report"]["path"]).read_bytes()
    expected = expected_bytes(original, scalar["selected_source_sha256_before"], scalar["selected_source_sha256_after"])
    require(set(scalar["added_selected_paths"]) == ADDITIONS and set(scalar["changed_selected_paths"]) == CHANGES,
            "scalar transition criteria changed after preregistration")
    binding_record = record_for(args.binding)
    require(binding_record["sha256"] == args.binding_sha256, "scalar binding literal SHA differs")
    binding = strict_json(Path(args.binding).read_bytes())
    keys(binding, ["schema_version", "status", "registered_utc", "protocol_sha256", "authoritative_scalar_worlds_executed_before_freeze",
                   "selected_source_sha256", "source_manifest", "compiler", "cargo", "resolved_features", "binary", "build_command"], "scalar binding")
    require(binding["schema_version"] == 1 and binding["status"] == "FROZEN_BEFORE_CURRENT_SCALAR_72_WORLDS" and
            binding["protocol_sha256"] == args.protocol_sha256 and binding["authoritative_scalar_worlds_executed_before_freeze"] == 0 and
            binding["selected_source_sha256"] == scalar["selected_source_sha256_after"], "scalar source/binary binding differs")
    artifact(binding["source_manifest"]); artifact(binding["resolved_features"]); artifact(binding["binary"])
    require("--locked" in binding["build_command"] and "--release" in binding["build_command"] and binding["compiler"] and binding["cargo"], "scalar build/compiler unbound")
    manifest = strict_json(Path(binding["source_manifest"]["path"]).read_bytes())
    require(all(manifest["files"].get(name) == digest for name, digest in binding["selected_source_sha256"].items()), "scalar selected hashes not in source manifest")
    report_record = record_for(args.report)
    require(report_record["sha256"] == args.report_sha256, "scalar report literal SHA differs")
    actual = Path(args.report).read_bytes()
    report = compare_reports(original, actual, scalar["selected_source_sha256_before"], scalar["selected_source_sha256_after"])
    collection_record = record_for(args.collection)
    require(collection_record["sha256"] == args.collection_sha256, "scalar collection literal SHA differs")
    collection = strict_json(Path(args.collection).read_bytes())
    keys(collection, ["schema_version", "status", "started_utc", "ended_utc", "exit_code", "binding_sha256", "report_sha256",
                      "source_inputs_unchanged", "argv", "stdout", "stderr", "source_guard_before", "source_guard_after"], "scalar collection")
    require(collection["schema_version"] == 1 and collection["status"] == "ACTUAL_CURRENT_SCALAR_72_WORLD_COLLECTION_COMPLETE" and
            type(collection["exit_code"]) is int and collection["exit_code"] == 0 and collection["source_inputs_unchanged"] is True and
            collection["binding_sha256"] == args.binding_sha256 and collection["report_sha256"] == args.report_sha256, "scalar collection incomplete/stale")
    require(stamp(protocol["registered_utc"]) <= stamp(binding["registered_utc"]) <= stamp(collection["started_utc"]) <= stamp(collection["ended_utc"]), "scalar freeze happened after collection")
    require(collection["argv"] == [binding["binary"]["path"], "--output", str(Path(args.report).resolve())], "scalar once-only CLI differs")
    for name in ["stdout", "stderr", "source_guard_before", "source_guard_after"]:
        artifact(collection[name])
    require(Path(collection["stdout"]["path"]).read_bytes() == STDOUT, "scalar stdout changed original FAIL")
    before = strict_json(Path(collection["source_guard_before"]["path"]).read_bytes())
    after = strict_json(Path(collection["source_guard_after"]["path"]).read_bytes())
    require(before == after == manifest, "scalar source guards changed from frozen current baseline")
    for name, digest in manifest["files"].items():
        require(record_for(Path("/workspace/RobotNativeEngine") / name)["sha256"] == digest, "live scalar source guard stale")
    return {"status": "PASS_LITERAL_CURRENT_SCALAR_SOURCE_OBJECT_TRANSITION", "protocol": protocol_record,
            "source_binary_binding": binding_record, "actual_collection": collection_record, "report": report_record,
            "baseline_report": scalar["baseline_report"], "only_registered_selected_source_object_changed": True,
            "old_report_bytes_preserved": True, "coverage": coverage(report), "summary": report["summary"],
            "native_worlds_executed_by_reader": 0, "motor_calibration_outcome": "FAIL, 0/4 primary, unchanged",
            "candidate_admission": "generation3 REJECTED, unchanged", "coupled_causal_or_maturity_gain_claim": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ["protocol", "protocol-sha256", "binding", "binding-sha256", "collection", "collection-sha256", "report", "report-sha256", "output"]:
        parser.add_argument("--" + name, required=True)
    args = parser.parse_args()
    with Path(args.output).open("x") as destination:
        try:
            result = audit(args); success = True
        except Exception as error:
            result = {"status": "FAIL_RETAINED_WITHOUT_CRITERIA_CHANGE", "error_type": type(error).__name__, "error": str(error), "native_worlds_executed_by_reader": 0}
            success = False
        json.dump(result, destination, indent=2, sort_keys=True, allow_nan=False); destination.write("\n")
    print(json.dumps({"status": result["status"], "output": record_for(args.output), "native_worlds": 0}, sort_keys=True))
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
