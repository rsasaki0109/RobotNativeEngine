#!/usr/bin/env python3
"""Frozen independent streaming reader for the fixed eighteen coupled worlds.

All calculations are pure integer/Fraction calculations. Raw measurement inputs
are never rewritten. Unavailable/subnormal statuses remain explicit and cannot
be counted as complete native replay or hidden contact attribution.
"""

import argparse
import datetime
import gzip
import hashlib
import json
from pathlib import Path
import sys

from stage_analysis import Analysis, analyze_telemetry
from stage_schema import (GzipJsonLines, artifact, byte_vector, encode_checkpoint,
                          integer, keys, literal_equal, parse_ecs, require,
                          sha_bytes, strict_json, validate_end, validate_start)

REPO = Path("/workspace/RobotNativeEngine")
BASE = "2c3586fa9168f13c774fb8f0c0d2cd359eba1287"
BUILDS = ["baseline", "rejected_inverse_w_plus_g"]
MODES = ["on", "fresh_on", "off"]
UNIT_SOURCE = "third_party/rapier3d/src/dynamics/joint/multibody_joint/unit_multibody_joint.rs"
HISTORICAL_PATCH_SHA = "08b5932ec3ac3271709c3e665ed631b76bbb51190abe86117333f19ea1b1f3c7"
RUNTIME_PATCH_SHA = "a4d75610e97ac5fcca76115b854cc7c790c8907e77bb12cf45bd6336af0503e7"
SOURCE_SCOPE = "Non-exhaustive repository extension-filtered input snapshot; not compiler/dependency closure"
COHORT_ID = "generation5-independent-all-motor-stage"
INDEPENDENCE = "NEW_INDEPENDENT_COHORT_NOT_RESTORATION_OF_LOST_GENERATION4_TAR"
RUN_FIELDS = ["build", "mass_kg", "mode", "run_label", "attempted", "exit_code", "started_utc", "ended_utc", "argv",
              "trace", "stdout", "stderr", "source_guard_before", "source_guard_after", "error"]


def stamp(value):
    require(type(value) is str, "missing actual timestamp")
    result = datetime.datetime.fromisoformat(value.replace("Z", "+00:00"))
    require(result.tzinfo is not None, "timestamp has no timezone")
    return result


def record_for(path):
    path = Path(path).resolve()
    digest = hashlib.sha256()
    count = 0
    with path.open("rb") as stream:
        while chunk := stream.read(1 << 20):
            digest.update(chunk); count += len(chunk)
    return {"path": str(path), "sha256": digest.hexdigest(), "bytes": count}


def read_artifact(record):
    artifact(record)
    return strict_json(Path(record["path"]).read_bytes())


def source_map(value):
    require(type(value) is dict and value, "missing source map")
    for path, digest in value.items():
        require(type(path) is str and not Path(path).is_absolute() and ".." not in Path(path).parts,
                "source path escapes checkout")
        require(type(digest) is str and len(digest) == 64 and all(ch in "0123456789abcdef" for ch in digest), "invalid source digest")
    return value


def check_protocol(protocol, protocol_record):
    keys(protocol, ["schema_version", "status", "registered_utc", "base_commit", "authoritative_coupled_worlds",
                    "scalar_qualification_worlds", "fixed_cells", "resource_ceilings", "reader_artifacts",
                    "dependency_source_artifacts", "candidate_runtime_patch", "historical_complete_rejected_patch",
                    "preregistration", "schema_source_sha256", "expected_runtime_parameters", "scalar_qualification",
                    "immutable_published_artifacts", "root_driver", "role_admission", "baseline_source_reference",
                    "audit_live_source_policy", "generation", "cohort_id", "independence", "checksum_only_recovery"], "protocol")
    require(protocol["schema_version"] == 1 and protocol["status"] == "FROZEN_PRE_DATA_GENERATION5_ALL_MOTOR_STAGE_PROTOCOL" and
            protocol["base_commit"] == BASE and protocol["authoritative_coupled_worlds"] == 18 and protocol["scalar_qualification_worlds"] == 0,
            "protocol scope is not the registered new eighteen-cell all-motor cohort; no fresh scalar worlds inferred")
    require(protocol["generation"] == 5 and type(protocol["generation"]) is int and
            protocol["cohort_id"] == COHORT_ID and protocol["independence"] == INDEPENDENCE,
            "new cohort/generation/independence identity differs")
    stamp(protocol["registered_utc"])
    if protocol["checksum_only_recovery"] is not None:
        artifact(protocol["checksum_only_recovery"])
    expected = [{"build": build, "mass_kg": mass, "mode": mode, "run_label": f"{build}-m{mass}-{mode}"}
                for build in BUILDS for mass in [5, 20, 50] for mode in MODES]
    require(literal_equal(protocol["fixed_cells"], expected), "fixed eighteen-cell matrix changed")
    frozen = {str(Path(record["path"]).resolve()): record for record in protocol["reader_artifacts"]}
    require(len(frozen) == len(protocol["reader_artifacts"]), "duplicate frozen reader artifact")
    for name in ["audit_coupled_stages.py", "stage_analysis.py", "stage_numeric.py", "stage_schema.py", "stage_roles.py", "synthetic-profile.json"]:
        actual = record_for(Path(__file__).parent / name)
        require(frozen.get(actual["path"]) == actual, "reader module changed after freeze: " + name)
    for record in protocol["reader_artifacts"] + protocol["dependency_source_artifacts"] + protocol["immutable_published_artifacts"]:
        artifact(record)
    artifact(protocol["preregistration"])
    artifact(protocol["role_admission"])
    artifact(protocol["baseline_source_reference"])
    require(protocol["audit_live_source_policy"] == "Selected schema/runtime sources must remain restored baseline; recorded complete guards are historical, publication-only metadata outside this set may change after collection",
            "audit live-source scope/metadata policy differs")
    artifact(protocol["root_driver"])
    historical, runtime = protocol["historical_complete_rejected_patch"], protocol["candidate_runtime_patch"]
    require(artifact(historical)["sha256"] == HISTORICAL_PATCH_SHA and artifact(runtime)["sha256"] == RUNTIME_PATCH_SHA,
            "registered rejected candidate patch changed")
    full = Path(historical["path"]).read_bytes()
    first = full.split(b"diff --git ")[1]
    require(b"diff --git " + first == Path(runtime["path"]).read_bytes(), "runtime patch is not the literal historical first file section")
    references = read_artifact(protocol["baseline_source_reference"])
    reference_map = {record["repository_path"]: record["baseline_sha256"] for record in references["files"]}
    require(references["baseline_commit"] == BASE and references["checkout_status"] == "" and
            source_map(protocol["schema_source_sha256"]) == {path: digest for path, digest in reference_map.items() if path != UNIT_SOURCE},
            "schema source closure differs from clean pre-data baseline references")
    for record in references["files"]:
        actual = record_for(record["reference_path"])
        require(actual["sha256"] == record["baseline_sha256"] and actual["bytes"] == record["bytes"],
                "baseline source reference bytes changed")

    ceilings = protocol["resource_ceilings"]
    keys(ceilings, ["compressed_bytes_per_run", "uncompressed_bytes_per_run", "json_line_bytes", "native_record_bytes",
                   "native_records_per_step", "native_bindings_per_step", "ecs_words_per_step", "motor_ndofs",
                   "generic_velocity_words", "motor_rows_per_event", "events_per_step", "slices_per_island_partition",
                   "ccd_partitions_per_step"], "resource ceilings")
    for value in ceilings.values():
        integer(value, minimum=1)
    profile = strict_json((Path(__file__).parent / "synthetic-profile.json").read_bytes())
    require(literal_equal(ceilings, profile["resource_ceilings"]) and
            literal_equal(protocol["expected_runtime_parameters"], profile["expected_runtime_parameters"]),
            "fixed original resource ceilings or independently source-derived parameters changed")
    require(protocol["scalar_qualification"] == {"status": "NOT_PART_OF_NEW_COUPLED_COLLECTION", "new_worlds": 0},
            "new scalar worlds must not be inferred from coupled qualification")
    return expected


def check_binding(record, build, protocol, protocol_record, collection_start):
    binding = read_artifact(record)
    keys(binding, ["schema_version", "status", "build", "base_commit", "registered_utc",
                   "cohort_id", "authoritative_worlds_of_this_cohort_before_freeze", "protocol_sha256", "reader_artifacts",
                   "schema_source_sha256", "repo_source_sha256", "source_manifest", "compiler", "cargo", "target",
                   "resolved_features", "rustflags", "build_command", "binary", "native_float_mode", "rejected_candidate_patch", "error"],
         "source/binary binding")
    require(binding["schema_version"] == 1 and binding["status"] == "FROZEN_BEFORE_GENERATION5_COHORT_WORLDS" and binding["build"] == build and
            binding["base_commit"] == BASE and binding["protocol_sha256"] == protocol_record["sha256"] and
            binding["authoritative_worlds_of_this_cohort_before_freeze"] == 0 and
            type(binding["authoritative_worlds_of_this_cohort_before_freeze"]) is int and binding["cohort_id"] == COHORT_ID and binding["error"] is None, "binding provenance/world count differs")
    require(stamp(protocol["registered_utc"]) <= stamp(binding["registered_utc"]) <= collection_start, "reader/source/binary freeze occurred after first world")
    require(literal_equal(binding["reader_artifacts"], protocol["reader_artifacts"]) and
            literal_equal(binding["schema_source_sha256"], protocol["schema_source_sha256"]), "binding reader/schema differs from pre-data protocol")
    manifest = read_artifact(binding["source_manifest"])
    keys(manifest, ["scope", "files"], "source manifest")
    require(manifest["scope"] == SOURCE_SCOPE and literal_equal(source_map(manifest["files"]), source_map(binding["repo_source_sha256"])),
            "source manifest/map differs or closure caveat missing")
    for path, digest in protocol["schema_source_sha256"].items():
        require(binding["repo_source_sha256"].get(path) == digest, "schema source is absent from frozen source map")
    for name in ["compiler", "cargo", "target"]:
        require(type(binding[name]) is str and binding[name], "unbound compiler/cargo/target")
    require(binding["rustflags"] == [] and type(binding["rustflags"]) is list, "new cohort requires unchanged empty Rust flags")
    require(type(binding["build_command"]) is list and "--release" in binding["build_command"] and "--locked" in binding["build_command"], "binary was not bound to locked release build")
    artifact(binding["resolved_features"]); artifact(binding["binary"])
    mode = binding["native_float_mode"]
    keys(mode, ["native_scalar_bits", "rounding_mode", "operation_contract", "replay_domain", "ftz_daz_source_predicate",
                "scope", "rounding_mode_basis", "evidence"], "native float mode")
    require(mode["native_scalar_bits"] == 32 and mode["rounding_mode"] == "roundTiesToEven" and
            mode["operation_contract"] == "rust-f32-default-strict-separate-ops-no-mul_add" and mode["replay_domain"] == "normal_or_zero_only" and
            mode["scope"] == "source_bound_runtime_register_unmeasured" and type(mode["rounding_mode_basis"]) is str and mode["rounding_mode_basis"],
            "unknown native scalar operation/mode binding")
    predicate = mode["ftz_daz_source_predicate"]
    keys(predicate, ["target_arch", "target_feature_sse", "enhanced_determinism_feature"], "FTZ/DAZ predicate")
    require(type(predicate["target_arch"]) is str and type(predicate["target_feature_sse"]) is bool and
            type(predicate["enhanced_determinism_feature"]) is bool, "unbound FTZ/DAZ compile predicate")
    for evidence in mode["evidence"]:
        artifact(evidence)
    expected_patch = None if build == "baseline" else protocol["candidate_runtime_patch"]
    require(literal_equal(binding["rejected_candidate_patch"], expected_patch), "candidate build patch registration differs")
    return binding


def check_candidate_delta(baseline, candidate):
    first, second = baseline["repo_source_sha256"], candidate["repo_source_sha256"]
    require(set(first) == set(second), "candidate added/removed source input paths")
    require({path for path in first if first[path] != second[path]} == {UNIT_SOURCE}, "candidate source delta is not exactly the old runtime unit-row file")
    before = (Path(__file__).parent / "source-reference" / UNIT_SOURCE).read_bytes()
    require(sha_bytes(before) == first[UNIT_SOURCE], "restored live unit source differs from baseline binding")
    prefix = b"        cfm_gain: motor_params.cfm_gain,\r\n"
    old = prefix + b"        inv_lhs: crate::utils::inv(lhs),\r\n"
    new = (prefix + b"        inv_lhs: if motor.model == crate::dynamics::MotorModel::ForceBased {\r\n"
           b"            crate::utils::inv(lhs + motor_params.cfm_gain)\r\n"
           b"        } else {\r\n"
           b"            crate::utils::inv(lhs)\r\n"
           b"        },\r\n")
    require(before.count(old) == 1, "registered unit reciprocal source replacement is not unique")
    require(sha_bytes(before.replace(old, new, 1)) == second[UNIT_SOURCE], "candidate compiled source is not exact historical reciprocal hunk reconstruction")
    require(baseline["resolved_features"]["sha256"] == candidate["resolved_features"]["sha256"] and
            baseline["resolved_features"]["bytes"] == candidate["resolved_features"]["bytes"], "cross-build resolved feature content changed")
    for name in ["compiler", "cargo", "target", "rustflags", "native_float_mode"]:
        require(literal_equal(baseline[name], candidate[name]), "cross-build compiler/feature/float contract changed: " + name)


def validate_step(record, expected_step, observer, protocol, mass=None):
    keys(record, ["kind", "schema_version", "completed_outer_step", "native_checkpoint",
                  "native_checkpoint_postcard_bytes", "ecs_physics_ieee_words", "stage_telemetry"], "completed step")
    require(record["kind"] == "completed_step" and record["schema_version"] == 1 and type(record["schema_version"]) is int and
            record["completed_outer_step"] == expected_step and type(record["completed_outer_step"]) is int, "missing/duplicate/reordered completed step")
    encoded = encode_checkpoint(record["native_checkpoint"], protocol["resource_ceilings"])
    actual = byte_vector(record["native_checkpoint_postcard_bytes"], protocol["resource_ceilings"]["json_line_bytes"])
    require(encoded == actual, "whole native checkpoint postcard does not match independently reconstructed typed fields")
    require(literal_equal(record["native_checkpoint"]["integration_parameters"], protocol["expected_runtime_parameters"]), "actual integration parameters retuned from registered baseline")
    from stage_schema import f32_bits
    require(record["native_checkpoint"]["gravity_ieee_words"] == [0, f32_bits(-9.81), 0], "native gravity changed")
    entities = parse_ecs(record["ecs_physics_ieee_words"], protocol["resource_ceilings"])
    from stage_schema import f64_bits
    require(entities[0]["body"][1] == entities[2]["body"][1] == f64_bits(1),
            "fixed floor/anchor authored mass differs")
    require(entities[1]["body"][1] in [f64_bits(value) for value in (5, 20, 50)] and
            (mass is None or entities[1]["body"][1] == f64_bits(mass)), "ECS cube authored mass differs from declared case")
    for entity in entities:
        for word in entity["pose"] + entity["body"][1:] + entity["joint"]:
            require((word >> 52) & 0x7ff != 0x7ff, "nonfinite captured ECS physics scalar")
    if observer:
        require(type(record["stage_telemetry"]) is dict, "enabled stage data missing")
    else:
        require(record["stage_telemetry"] is None, "disabled stage data was present")
    return entities


def audit_cell(rows, bindings, protocol, details_directory):
    streams = [GzipJsonLines(row["trace"]["path"], protocol["resource_ceilings"]) for row in rows]
    iterators = [iter(stream) for stream in streams]
    for row, iterator in zip(rows, iterators):
        validate_start(next(iterator), row)
    detail_path = details_directory / (rows[0]["run_label"] + ".independent-details.jsonl.gz")
    output = detail_path.open("xb")
    encoder = gzip.GzipFile(filename="", fileobj=output, mode="wb", mtime=0)
    details_count = 0
    def emit(record):
        nonlocal details_count
        encoder.write(json.dumps(record, sort_keys=True, separators=(",", ":"), allow_nan=False).encode() + b"\n")
        details_count += 1
    analysis = Analysis(rows[0]["build"], emit)
    checkpoints = native_bytes = ecs_words = 0
    last_entities = None
    try:
        for step in range(1, 1251):
            frames = [next(iterator) for iterator in iterators]
            entities = [validate_step(frame, step, row["mode"] != "off", protocol, row["mass_kg"]) for frame, row in zip(frames, rows)]
            first = frames[0]
            for frame in frames[1:]:
                for field in ["native_checkpoint", "native_checkpoint_postcard_bytes", "ecs_physics_ieee_words"]:
                    require(literal_equal(first[field], frame[field]), "within-build observer physical replay differs at step " + str(step) + ": " + field)
            require(literal_equal(first["stage_telemetry"], frames[1]["stage_telemetry"]), "fresh ON ordered stage words differ at step " + str(step))
            analyze_telemetry(first["stage_telemetry"], first["native_checkpoint"], entities[0], step, protocol, analysis)
            for frame in frames:
                checkpoints += 1; native_bytes += len(frame["native_checkpoint_postcard_bytes"]); ecs_words += len(frame["ecs_physics_ieee_words"])
            last_entities = entities[0]
        terminals = [next(iterator) for iterator in iterators]
        for terminal in terminals:
            validate_end(terminal)
        require(all(literal_equal(terminals[0]["physical_report"], terminal["physical_report"]) for terminal in terminals[1:]), "within-build final physical report differs")
        require(terminals[0]["physical_report"]["cube_ecs_pose_ieee_words"] == last_entities[1]["pose"], "final cube pose is not the literal final ECS cube pose")
        for row, terminal in zip(rows, terminals):
            wanted_stdout = (terminal["physical_report"]["physical_stdout_row"] + "\ncontact motor stage collection: ok\n").encode()
            require(Path(row["stdout"]["path"]).read_bytes() == wanted_stdout, "actual fixture stdout does not match its collected terminal")
        for iterator, stream in zip(iterators, streams):
            try:
                next(iterator)
            except StopIteration:
                pass
            else:
                raise ValueError("extra record after terminal")
            require(stream.complete and stream.lines == 1252, "gzip/frame collection incomplete")
    finally:
        encoder.close(); output.close()
        for iterator in iterators:
            iterator.close()
    summary = analysis.summary()
    return {"build": rows[0]["build"], "mass_kg": rows[0]["mass_kg"], "runs": [row["run_label"] for row in rows],
            "within_build_native_checkpoint_typed_and_postcard_bytes_exact": True, "within_build_ecs_physics_ieee_words_exact": True,
            "fresh_ON_ordered_trace_exact": True, "OFF_trace_absent": True, "completed_steps_each": 1250,
            "physics_checkpoints": checkpoints, "native_checkpoint_bytes": native_bytes, "ecs_words": ecs_words,
            "compressed_inputs": [row["trace"] for row in rows],
            "input_streams": [{"run_label": row["run_label"], "records": stream.lines, "uncompressed_bytes": stream.uncompressed_bytes,
                               "uncompressed_sha256": stream.uncompressed_sha256} for row, stream in zip(rows, streams)],
            "numerical_analysis_scope": "ON trace once; fresh ON is literal-equal, not another independent numerical sample",
            "details": record_for(detail_path), "detail_records": details_count, "physical_report": terminals[0]["physical_report"],
            "analysis": summary}


def check_durable_provenance(collection, collection_record, protocol, protocol_record, bindings, start, end):
    """Strict actual schema2, original exit receipts and prospective spec linkage.

    This reads JSON/file metadata only. It never imports the driver's modules,
    executes a binary, resumes a cohort, or repairs any receipt.
    """
    spec = read_artifact(collection["frozen_spec"])
    keys(spec, ["schema_version", "generation", "status", "independence", "cohort_id", "registered_utc",
                "authoritative_worlds_of_this_cohort_before_freeze", "fixed_cells", "world_limit",
                "preflight_gates", "cohort_root", "repository_root", "protocol", "reader_artifacts",
                "reader_pure_test_receipt", "independent_readonly_review", "source_guard_manifest",
                "driver_artifacts", "bindings"], "frozen durable driver spec")
    require(spec["schema_version"] == 1 and type(spec["schema_version"]) is int and
            spec["generation"] == 5 and type(spec["generation"]) is int and
            spec["status"] == "FROZEN_BEFORE_GENERATION5_COLLECTION" and
            spec["cohort_id"] == COHORT_ID and spec["independence"] == INDEPENDENCE and
            spec["authoritative_worlds_of_this_cohort_before_freeze"] == 0 and
            type(spec["authoritative_worlds_of_this_cohort_before_freeze"]) is int and
            type(spec["world_limit"]) is int and spec["world_limit"] == 18,
            "driver spec is not the prospectively frozen new cohort")
    require(stamp(protocol["registered_utc"]) <= stamp(spec["registered_utc"]) <= start,
            "driver spec freeze postdates first collection")
    require(literal_equal(spec["fixed_cells"], protocol["fixed_cells"]) and
            literal_equal(spec["protocol"], protocol_record) and
            literal_equal(spec["reader_artifacts"], protocol["reader_artifacts"]) and
            literal_equal(spec["bindings"], collection["source_binary_bindings"]),
            "driver spec matrix/protocol/readers/bindings differ")
    require(spec["preflight_gates"] == {"reader_pure_tests": "PASS", "independent_readonly_review": "PASS"},
            "actual pre-world pure/review gates were not admitted")
    for name in ["reader_pure_test_receipt", "independent_readonly_review"]:
        artifact(spec[name])
    require(spec["repository_root"] == str(REPO), "driver repository differs")
    cohort_root = Path(spec["cohort_root"])
    require(cohort_root.is_absolute() and cohort_root.resolve() == collection_record_path_parent(collection_record),
            "durable index is outside its registered cohort directory")
    manifest = read_artifact(spec["source_guard_manifest"])
    require(literal_equal(manifest, {"scope": SOURCE_SCOPE, "files": bindings["baseline"]["repo_source_sha256"]}),
            "durable source guard differs from restored baseline source map")
    require(type(spec["driver_artifacts"]) is list and len(spec["driver_artifacts"]) == 3,
            "driver source closure must contain exactly its three modules")
    driver = Path(protocol["root_driver"]["path"])
    records = {Path(record["path"]).name: record for record in spec["driver_artifacts"]}
    require(set(records) == {"collect_once.py", "durable_common.py", "prepare_archive.py"},
            "driver module source closure differs")
    for name, record in records.items():
        artifact(record)
        require(Path(record["path"]).resolve() == driver.parent / name,
                "driver module is outside its registered source directory")
    require(literal_equal(records["collect_once.py"], protocol["root_driver"]) and
            literal_equal(collection["root_driver"], protocol["root_driver"]),
            "actual collection driver changed after protocol registration")
    require(type(collection["case_receipts"]) is list and len(collection["case_receipts"]) == 18 and
            type(collection["runs"]) is list and len(collection["runs"]) == 18,
            "all eighteen original durable case receipts are required")
    for ordinal, (row, case_record) in enumerate(zip(collection["runs"], collection["case_receipts"])):
        case = read_artifact(case_record)
        keys(case, ["schema_version", "cohort_id", "case_index", "original_exit_code", "launch_reserved",
                    "process_started", "binary", "original_process_exit_receipt", "raw_output_exclusive_create_contract",
                    "source_inputs_unchanged", "launch_error", "row", "retry_or_resume_permitted"], "durable case terminal")
        require(type(case["schema_version"]) is int and case["schema_version"] == 1 and
                type(case["case_index"]) is int and case["case_index"] == ordinal and case["cohort_id"] == COHORT_ID and
                type(case["original_exit_code"]) is int and case["original_exit_code"] == 0 and
                case["launch_reserved"] is True and case["process_started"] is True and
                case["source_inputs_unchanged"] is True and case["launch_error"] is None and
                case["retry_or_resume_permitted"] is False and literal_equal(case["row"], row) and
                literal_equal(case["binary"], bindings[row["build"]]["binary"]) and
                case["raw_output_exclusive_create_contract"] == "Unchanged collector142 OpenOptions::create_new(true)",
                "durable original case outcome/provenance differs")
        folder = cohort_root / "cases" / f"{ordinal:02d}-{row['run_label']}"
        require(Path(case_record["path"]).resolve() == folder / "case-terminal.json",
                "durable case terminal path/index differs")
        exit_record = case["original_process_exit_receipt"]
        original_exit = read_artifact(exit_record)
        keys(original_exit, ["case_index", "pid", "ended_utc", "original_exit_code", "source"], "original actual Popen exit")
        require(Path(exit_record["path"]).resolve() == folder / "process-exit.json" and
                type(original_exit["case_index"]) is int and original_exit["case_index"] == ordinal and
                type(original_exit["original_exit_code"]) is int and original_exit["original_exit_code"] == row["exit_code"] == 0 and
                original_exit["source"] == "Actual subprocess.Popen.returncode; no status substitution",
                "actual original process exit receipt differs")
        integer(original_exit["pid"], minimum=1)
        process_started = strict_json((folder / "process-started.json").read_bytes())
        keys(process_started, ["case_index", "pid", "started_utc", "argv", "launch_claim"], "actual Popen start")
        require(type(process_started["case_index"]) is int and process_started["case_index"] == ordinal and
                type(process_started["pid"]) is int and process_started["pid"] == original_exit["pid"] and
                process_started["argv"] == row["argv"], "actual process start identity differs")
        launch = read_artifact(process_started["launch_claim"])
        require(Path(process_started["launch_claim"]["path"]).resolve() == folder / "launch-claim.json",
                "durable launch claim path differs")
        keys(launch, RUN_FIELDS + ["case_index", "cohort_id", "launch_reserved", "original_exit_code", "binary", "frozen_spec",
                                  "environment_policy", "retry_or_resume_permitted"], "durable launch reservation")
        require(type(launch["case_index"]) is int and launch["case_index"] == ordinal and launch["cohort_id"] == COHORT_ID and
                launch["launch_reserved"] is True and launch["original_exit_code"] is None and launch["attempted"] is False and
                launch["exit_code"] is None and launch["ended_utc"] is None and launch["error"] is None and
                launch["environment_policy"] == "Inherited process environment; no driver overrides" and
                launch["retry_or_resume_permitted"] is False and literal_equal(launch["frozen_spec"], collection["frozen_spec"]) and
                literal_equal(launch["binary"], bindings[row["build"]]["binary"]),
                "durable pre-Popen reservation differs")
        for field in ["build", "mass_kg", "mode", "run_label", "started_utc", "argv", "source_guard_before"]:
            require(literal_equal(launch[field], row[field]), "launch/index fields differ: " + field)
        require(all(launch[field] is None for field in ["trace", "stdout", "stderr", "source_guard_after"]),
                "launch reservation wrongly contains post-process artifacts")
        require(start <= stamp(row["started_utc"]) <= stamp(process_started["started_utc"]) <=
                stamp(original_exit["ended_utc"]) <= stamp(row["ended_utc"]) <= end,
                "durable reserve/start/original-exit/terminal timestamps are inconsistent")
    terminal = strict_json((cohort_root / "collection-terminal.json").read_bytes())
    keys(terminal, ["schema_version", "status", "cohort_id", "independence", "index", "frozen_spec", "worlds_launch_reserved",
                    "worlds_attempted", "worlds_completed", "driver_exit_code", "case_exit_codes_preserved", "retry_or_resume_permitted"],
         "durable collection terminal")
    require(type(terminal["schema_version"]) is int and terminal["schema_version"] == 1 and
            terminal["status"] == collection["status"] and terminal["cohort_id"] == COHORT_ID and terminal["independence"] == INDEPENDENCE and
            literal_equal(terminal["index"], collection_record) and literal_equal(terminal["frozen_spec"], collection["frozen_spec"]) and
            type(terminal["driver_exit_code"]) is int and terminal["driver_exit_code"] == 0 and
            terminal["case_exit_codes_preserved"] is True and terminal["retry_or_resume_permitted"] is False and
            all(type(terminal[field]) is int and terminal[field] == 18 for field in
                ["worlds_launch_reserved", "worlds_attempted", "worlds_completed"]),
            "durable collection terminal outcome/counts differ")


def collection_record_path_parent(record):
    path = Path(record["path"]).resolve()
    require(path.name == "collection-index.json", "actual durable collection index name differs")
    return path.parent


def audit(args):
    protocol_record = record_for(args.protocol)
    require(protocol_record["sha256"] == args.protocol_sha256, "protocol literal SHA differs")
    protocol = strict_json(Path(args.protocol).read_bytes())
    expected = check_protocol(protocol, protocol_record)
    collection_record = record_for(args.collection)
    require(collection_record["sha256"] == args.collection_sha256, "collection literal SHA differs")
    collection = strict_json(Path(args.collection).read_bytes())
    keys(collection, ["schema_version", "generation", "cohort_id", "independence", "status", "frozen_spec",
                      "protocol_sha256", "root_driver", "started_utc", "ended_utc",
                      "declared_authoritative_coupled_worlds", "declared_completed_outer_steps_per_world",
                      "worlds_launch_reserved", "worlds_attempted", "worlds_completed", "source_binary_bindings",
                      "case_receipts", "runs", "error"], "generation-five collection index")
    require(collection["schema_version"] == 2 and type(collection["schema_version"]) is int and
            collection["generation"] == 5 and type(collection["generation"]) is int and
            collection["cohort_id"] == COHORT_ID and collection["independence"] == INDEPENDENCE and
            collection["status"] == "GENERATION5_18_WORLD_PROCESSES_COMPLETED" and
            collection["protocol_sha256"] == args.protocol_sha256 and collection["error"] is None,
            "new collection incomplete/error/identity differs")
    for field in ["declared_authoritative_coupled_worlds", "worlds_launch_reserved", "worlds_attempted", "worlds_completed"]:
        require(type(collection[field]) is int and collection[field] == 18, "actual reserved/started/completed count differs")
    require(type(collection["declared_completed_outer_steps_per_world"]) is int and
            collection["declared_completed_outer_steps_per_world"] == 1250, "declared outer-step count differs")
    start, end = stamp(collection["started_utc"]), stamp(collection["ended_utc"])
    require(start <= end and literal_equal(collection["root_driver"], protocol["root_driver"]), "collection ordering/driver differs")
    keys(collection["source_binary_bindings"], BUILDS, "build bindings")
    bindings = {build: check_binding(collection["source_binary_bindings"][build], build, protocol, protocol_record, start) for build in BUILDS}
    check_candidate_delta(bindings["baseline"], bindings["rejected_inverse_w_plus_g"])
    baseline_source = bindings["baseline"]["repo_source_sha256"]
    check_durable_provenance(collection, collection_record, protocol, protocol_record, bindings, start, end)
    live_runtime = dict(protocol["schema_source_sha256"], **{UNIT_SOURCE: baseline_source[UNIT_SOURCE]})
    for path, digest in live_runtime.items():
        require(record_for(REPO / path)["sha256"] == digest, "restored live baseline source changed before audit: " + path)
    require(type(collection["runs"]) is list and len(collection["runs"]) == 18, "missing matrix entry")
    previous_end = start
    for row, expected_cell in zip(collection["runs"], expected):
        keys(row, RUN_FIELDS, "actual run entry")
        require(all(literal_equal(row[name], value) for name, value in expected_cell.items()), "collection matrix/order changed")
        require(row["attempted"] is True and type(row["exit_code"]) is int and row["exit_code"] == 0 and row["error"] is None, "actual run errored/not attempted")
        run_start, run_end = stamp(row["started_utc"]), stamp(row["ended_utc"])
        require(previous_end <= run_start <= run_end <= end, "run ordering overlaps or timestamps are invalid")
        previous_end = run_end
        for name in ["trace", "stdout", "stderr"]:
            artifact(row[name])
        before, after = read_artifact(row["source_guard_before"]), read_artifact(row["source_guard_after"])
        require(literal_equal(before, after) and before == {"scope": SOURCE_SCOPE, "files": baseline_source}, "run live source guard differs from restored baseline")
        binary = bindings[row["build"]]["binary"]
        expected_argv = [binary["path"], "--mass-kg", str(row["mass_kg"]), "--observer", "off" if row["mode"] == "off" else "on",
                         "--label", row["run_label"], "--output", row["trace"]["path"]]
        require(row["argv"] == expected_argv, "actual run argv differs from fixed fixture call")
    detail_directory = Path(args.details_directory).resolve()
    detail_directory.mkdir(parents=True, exist_ok=False)
    results = []
    for offset in range(0, 18, 3):
        results.append(audit_cell(collection["runs"][offset:offset + 3], bindings, protocol, detail_directory))
    complete_native = all(result["analysis"]["complete_native_replay"] for result in results)
    complete_reference = all(result["analysis"]["complete_finite_valid_reference"] for result in results)
    no_issues = all(result["analysis"]["qualification_issues"] == 0 for result in results)
    return {"status": "PASS_COMPLETE_REPLAY_AND_SOURCE_ORDERED_NORMAL_ZERO_STAGE_QUALIFICATION" if complete_native and complete_reference and no_issues else
                      "INCOMPLETE_OR_FAILED_NUMERICAL_STAGE_QUALIFICATION", "protocol": protocol_record, "collection": collection_record,
                      "cohort_id": COHORT_ID, "independence": INDEPENDENCE,
            "native_worlds_executed_by_reader": 0, "registered_authoritative_coupled_worlds": 18,
            "registered_scalar_worlds_separately_audited": 0, "completed_coupled_worlds": 18,
            "total_completed_coupled_outer_steps": 22500, "numerically_analyzed_ON_outer_steps": 7500,
            "complete_native_normal_zero_replay": complete_native, "complete_finite_valid_reference": complete_reference,
            "all_qualification_checks_no_issues": no_issues, "results": results,
            "native_checkpoint_scope": "sorted individual native bodies/colliders/unique multibodies/impulse joints, params/gravity/bindings; opaque per-object layout source-bound",
            "runtime_registers_measured": False, "nonexhaustive_source_guard_caveat": SOURCE_SCOPE,
            "historical_candidate_admission": "generation3 REJECTED; this measurement does not promote it",
            "hidden_per_contact_coupling_or_causal_adoption_or_maturity_claim": False,
            "separate_scalar_qualification_not_inferred_from_this_reader": True,
            "reader_frozen_before_generation5_data": True,
            "new_cohort_all_supported_internal_motor_rows_preregistered": True,
            "generation4_palm_only_gate_and_failed_outcome_unchanged": True}



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ["protocol", "protocol-sha256", "collection", "collection-sha256", "output", "details-directory"]:
        parser.add_argument("--" + name, required=True)
    args = parser.parse_args()
    output = Path(args.output).resolve()
    with output.open("x") as destination:
        try:
            result = audit(args)
            success = result["status"].startswith("PASS_")
        except Exception as error:
            result = {"status": "FAIL_RETAINED_WITHOUT_CRITERIA_CHANGE", "error_type": type(error).__name__, "error": str(error),
                      "native_worlds_executed_by_reader": 0, "protocol_sha256_argument": args.protocol_sha256,
                      "collection_sha256_argument": args.collection_sha256,
                      "reader_is_predata_generation5_all_motor_reader": True,
                      "historical_candidate_admission": "generation3 REJECTED, unchanged"}
            success = False
        json.dump(result, destination, indent=2, sort_keys=True, allow_nan=False); destination.write("\n")
    print(json.dumps({"status": result["status"], "output": record_for(output), "native_worlds": 0}, sort_keys=True))
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
