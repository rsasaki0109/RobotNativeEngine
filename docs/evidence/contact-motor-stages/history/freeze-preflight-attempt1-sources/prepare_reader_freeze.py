#!/usr/bin/env python3
"""Read-only source/pure-test preflight and outside-only pre-data reader freeze."""

import datetime
import json
from pathlib import Path

from audit_coupled_stages import check_candidate_delta, check_protocol, record_for
from stage_schema import artifact, require, strict_json

HERE = Path(__file__).parent
REPO = Path("/workspace/RobotNativeEngine")
ROOT = Path("/workspace/.rne-kick-demo/single-joint-motor-calibration")


def main():
    before = strict_json((HERE / "baseline-source-manifest.json").read_bytes())
    after = strict_json((HERE / "rejected-source-manifest.json").read_bytes())
    for name, digest in before["files"].items():
        require(record_for(REPO / name)["sha256"] == digest, "live restored source stale: " + name)
    normalized = record_for(HERE / "common142-resolved-features.txt")
    compiler = strict_json((HERE / "actual-compiler.json").read_bytes())
    common = {"compiler": compiler["rustc_vv"], "cargo": compiler["cargo_v"], "target": "x86_64-unknown-linux-gnu",
              "resolved_features": normalized, "rustflags": [], "native_float_mode": {}}
    check_candidate_delta(dict(common, repo_source_sha256=before["files"]), dict(common, repo_source_sha256=after["files"]))
    normalized_record = record_for(HERE / "actual-exact142-feature-normalization.json")
    artifact(normalized_record)
    baseline_features = (HERE / "baseline-resolved-features.txt").read_bytes()
    candidate_features = (HERE / "rejected-resolved-features.txt").read_bytes()
    require(baseline_features[:len(candidate_features)] == candidate_features == Path(normalized["path"]).read_bytes(),
            "exact 142 feature normalization differs")
    require(baseline_features[len(candidate_features):].startswith(b"\nposition_motor_calibration "), "feature suffix is not the separate scalar root")
    pure_record = record_for(HERE / "final-pure-reader-qualification.json")
    pure = strict_json(Path(pure_record["path"]).read_bytes())
    require(pure["status"] == "ACTUAL_FINAL_PURE_TESTS_PASS_BEFORE_AUTHORITATIVE_DATA" and pure["native_worlds"] == 0 and
            all(test["actual_exit_code"] == 0 for test in pure["tests"]), "pure reader qualification incomplete")
    for name, record in pure["sources"].items():
        require(record_for(HERE / name) == record, "pure-tested reader source changed: " + name)
    for test in pure["tests"]:
        artifact(test["stdout"]); artifact(test["stderr"])
    protocol = strict_json((HERE / "protocol-template.json").read_bytes())
    now = datetime.datetime.now(datetime.timezone.utc).isoformat()
    protocol.update(status="FROZEN_PRE_DATA_COUPLED_STAGE_PROTOCOL", registered_utc=now)
    reader_names = ["audit_coupled_stages.py", "audit_current_scalar_report.py", "stage_analysis.py", "stage_numeric.py", "stage_schema.py",
                    "test_stage_numeric.py", "test_stage_reader.py", "test_scalar_transition.py", "test_coupled_stream.py",
                    "collection-index-template.json", "source-binary-binding-template.json", "protocol-template.json",
                    "collection-contract.md", "final-pure-reader-qualification.json", "pure-reader-attempt1-receipt.json"]
    protocol["reader_artifacts"] = [record_for(HERE / name) for name in reader_names]
    for test in pure["tests"]:
        protocol["reader_artifacts"].extend([test["stdout"], test["stderr"]])
    registry = Path("/workspace/.rne-tools/cargo/registry/src/index.crates.io-1949cf8c6b5b557f/nalgebra-0.33.3")
    dependency_sources = [registry / name for name in ["src/base/blas.rs", "src/base/blas_uninit.rs", "src/base/alias_view.rs",
                                                     "src/base/matrix_view.rs", "src/base/dimension.rs", "src/lib.rs"]]
    dependency_sources += [REPO / name for name in ["third_party/rapier3d/src/utils.rs",
                                                   "third_party/rapier3d/src/dynamics/solver/joint_constraint/joint_generic_constraint.rs",
                                                   "third_party/rapier3d/src/dynamics/joint/motor_model.rs",
                                                   "third_party/rapier3d/src/dynamics/joint/multibody_joint/unit_multibody_joint.rs",
                                                   "examples/142_contact_motor_stages/README.md"]]
    dependency_sources += [HERE / "actual-compiler.json", HERE / "actual-exact142-feature-normalization.json",
                           HERE / "common142-resolved-features.txt", HERE / "baseline-build-capture.json", HERE / "rejected-build-capture.json",
                           HERE / "actual-exact-candidate-rollback.json", HERE / "scalar-source-transition-preregistration.json"]
    protocol["dependency_source_artifacts"] = [record_for(path) for path in dependency_sources]
    protocol["candidate_runtime_patch"] = record_for(HERE / "registered-runtime-only-rejected.patch")
    protocol["historical_complete_rejected_patch"] = record_for(ROOT / "diagonal-intervention-generation-3/complete-rejected-candidate.patch")
    protocol["preregistration"] = record_for(HERE / "preregistration.md")
    schema_paths = ["third_party/rapier3d/src/pipeline/multibody_motor_stage.rs", "crates/rne_physics_rapier/src/motor_stages.rs",
                    "examples/142_contact_motor_stages/lib.rs", "examples/142_contact_motor_stages/main.rs",
                    "examples/142_contact_motor_stages/Cargo.toml", "third_party/rapier3d/src/dynamics/integration_parameters.rs"]
    protocol["schema_source_sha256"] = {name: before["files"][name] for name in schema_paths}
    immutable = []
    for name in ["position-motor-calibration.json", "position-motor-calibration-bincode-generation-1.json",
                 "position-motor-calibration-codec-migration.json", "position-motor-diagonal-calibration.json",
                 "position-motor-calibration-vendor-lint.json"]:
        immutable.append(record_for(REPO / "docs/media" / name))
    for bundle in ["position-motor-diagonal-experiment", "rapier-strict-validation"]:
        manifest = REPO / "docs/evidence" / bundle / "artifact-manifest.json"
        manifest_data = strict_json(manifest.read_bytes())
        for item in manifest_data["files"]:
            record = dict(item, path=str(REPO / item["path"]))
            artifact(record); immutable.append(record)
        immutable.append(record_for(manifest))
    protocol["immutable_published_artifacts"] = immutable
    transition = strict_json((HERE / "scalar-source-transition-preregistration.json").read_bytes())
    scalar = protocol["scalar_qualification"]
    scalar["baseline_report"] = transition["original_report"]
    scalar["selected_source_sha256_before"] = json.loads(Path(transition["original_report"]["path"]).read_bytes())["selected_source_sha256"]
    scalar["selected_source_sha256_after"] = transition["expected_selected_source_sha256"]
    scalar["changed_selected_paths"] = transition["changed_keys"]
    require(set(scalar["added_selected_paths"]) == set(transition["added_keys"]), "selected-source additions disagree")
    require(all(before["files"].get(name) == digest for name, digest in scalar["selected_source_sha256_after"].items()),
            "registered current scalar selected source hashes differ")
    protocol["root_driver"] = record_for(HERE / "collect_coupled_stages.py")
    protocol_path = HERE / "frozen-protocol.json"
    with protocol_path.open("x") as stream:
        json.dump(protocol, stream, indent=2, sort_keys=True); stream.write("\n")
    protocol_record = record_for(protocol_path)
    check_protocol(protocol, protocol_record)
    receipt = {"status": "READER_SCHEMA_PROTOCOL_SOURCE_PREFLIGHT_PASS_BEFORE_AUTHORITATIVE_WORLDS", "registered_utc": now,
               "protocol": protocol_record, "reader_artifacts": protocol["reader_artifacts"], "pure_tests": pure_record,
               "actual_live_baseline_sources_equal_prebuild_manifest": True, "source_input_count": len(before["files"]),
               "candidate_exact_runtime_hunk_reconstruction": True, "candidate_specific_source_delta": ["third_party/rapier3d/src/dynamics/joint/multibody_joint/unit_multibody_joint.rs"],
               "exact_common_142_feature_normalization": normalized_record,
               "separate_scalar_metadata_transition": {"old_keys": 17, "new_keys": 21, "changed_existing_keys": 6, "added_keys": 4},
               "immutable_public_artifact_count": len(immutable), "authoritative_coupled_worlds_before_freeze": 0,
               "authoritative_current_scalar_worlds_before_freeze": 0, "native_worlds_executed_by_reader_preparation": 0,
               "source_binary_bindings_still_required_before_parent_collection": True,
               "historical_candidate_admission": "generation3 REJECTED, unchanged", "measured_numerical_or_causal_outcome": None}
    receipt_path = HERE / "actual-reader-protocol-freeze.json"
    with receipt_path.open("x") as stream:
        json.dump(receipt, stream, indent=2, sort_keys=True); stream.write("\n")
    for record in protocol["reader_artifacts"] + [protocol_record, record_for(receipt_path), protocol["preregistration"]]:
        Path(record["path"]).chmod(0o444)
    print(json.dumps({"protocol": protocol_record, "freeze_receipt": record_for(receipt_path), "native_worlds": 0}, indent=2))


if __name__ == "__main__":
    main()
