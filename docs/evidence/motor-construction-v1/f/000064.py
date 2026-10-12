"""Create source-only prospective metadata, never read a report/raw artifact."""

import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

import compare_scalar_source_transition as reader


ROOT = Path(__file__).resolve().parent
REPO = Path("/workspace/RobotNativeEngine")
SOURCE_PATHS = [
    "AGENTS.md", "examples/141_position_motor_calibration/lib.rs",
    "examples/141_position_motor_calibration/README.md", "xtask/src/main.rs",
    "third_party/rapier3d/Cargo.toml", "docs/adr/034-external-large-evidence-retention.md",
    ".github/workflows/ci.yml", ".github/workflows/release.yml",
    ".github/workflows/mujoco.yml", ".github/workflows/gazebo.yml",
    ".github/workflows/ros2-node.yml", ".github/workflows/ros2-bridge.yml",
    ".github/workflows/mjx-warp.yml",
]


def descriptor(data):
    return {"sha256": hashlib.sha256(data).hexdigest(), "size_bytes": len(data)}


def save(name, value):
    raw = (json.dumps(value, indent=2, sort_keys=True) + "\n").encode()
    with open(ROOT / name, "xb") as stream:
        stream.write(raw)
        stream.flush()
        os.fsync(stream.fileno())


def main():
    before = {name: (REPO / name).read_bytes() for name in SOURCE_PATHS}
    lib = before["examples/141_position_motor_calibration/lib.rs"].decode()
    selected_fn = lib.split("fn selected_sources()", 1)[1].split("\nstruct Fixture", 1)[0]
    actual_keys = re.findall(r'source!\(\s*"([^\"]+)"', selected_fn)
    if len(actual_keys) != 21 or set(actual_keys) != reader.BEFORE_KEYS:
        raise ValueError("current source declaration differs from the planned 21-key baseline")
    env = dict(os.environ, GIT_OPTIONAL_LOCKS="0")
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO, env=env).decode().strip()
    status = subprocess.check_output(["git", "status", "--porcelain"], cwd=REPO, env=env).decode()
    cells = []
    for gain in ["G1Posture", "G1Ankle"]:
        for target in ["-0.001_f32", "0.0_f32", "0.001_f32"]:
            for small_steps in [1, 32]:
                for pgs in [1, 2]:
                    cells.append({"index": len(cells), "gain_rust_variant": gain,
                                  "target_rust_literal": target,
                                  "solver_small_steps": small_steps, "internal_pgs_passes": pgs,
                                  "fresh_world_order": ["ON", "fresh_ON", "OFF"]})
    planned = sorted(reader.PERMITTED_EXISTING_CHANGES)
    provisional = sorted(reader.PERMITTED_EXISTING_CHANGES - {"Cargo.toml", "Cargo.lock"})
    draft = {
        "kind": "rne_prospective_141_motor_construction_source_transition_arm",
        "schema_version": 1, "status": "DRAFT_NOT_FROZEN",
        "created_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "arm_id": "motor-construction-source-transition-141",
        "baseline_merged_commit": "3be26ef0bd6967cf3e0b466a5b63b11795f40feb",
        "authority": "Proposal only; actual maps/binaries/protocol/tools require independent pre-data freeze",
        "historical_report": {"path": "docs/media/position-motor-calibration-contact-stages.json",
                              "full_sha256": None, "parent_supplied_sha_prefix": "db0",
                              "read_in_preparation": False, "preserve_literal": True},
        "proposed_current_report": "docs/media/position-motor-calibration-motor-construction-source.json",
        "separate_from": ["generation-5 fixed18", "proposed example-143 six construction worlds"],
        "world_budget": {"condition_count": 24, "worlds_per_condition": 3,
                         "maximum_worlds": 72, "collector_invocations": 1,
                         "partial_failure_action": "stop; preserve actual outputs/exits; no retry/replacement",
                         "numeric_filtering": False, "new_construction_observer_enabled": False},
        "cells_in_unchanged_collector_order": cells,
        "source_binding": {"before_key_count": 21, "after_key_count": 22,
                           "before_keys": sorted(reader.BEFORE_KEYS),
                           "after_keys": sorted(reader.BEFORE_KEYS | {reader.ADDED_KEY}),
                           "required_addition": reader.ADDED_KEY,
                           "removed_keys": [], "provisional_existing_changes": provisional,
                           "possible_integration_changes": ["Cargo.lock", "Cargo.toml"],
                           "permitted_existing_changes": planned,
                           "final_hashes_and_exact_diff": "PENDING actual integrated source freeze; not guessed",
                           "scope": "selected non-exhaustive source inputs, not full compiler closure"},
        "forbidden_changes": ["report schema", "collector cases/logic", "physics",
                              "codec", "criteria/tolerances", "old reports/raw/historical findings",
                              "unregistered source keys", "verifier source binding"],
        "freeze_prerequisites": [
            "143 standalone-to-workspace integration complete; Cargo manifests/locks/features final",
            "producer explains graph/query/interaction lifetime scope and preserves CRLF/EOF",
            "full literal before/after selected maps and precise changed list frozen",
            "baseline report full literal digest frozen without rewriting its bytes",
            "source tree/patch plus non-exhaustive broader input guard frozen",
            "compiler/Cargo/linker/target/features/flags and native binary descriptors frozen",
            "protocol/comparator/tool closure/pure-test receipt and independent review frozen",
            "persistent exclusive once-only claim/output/terminal/sourceguard plan frozen",
        ],
        "comparator_spec_template_NOT_EXECUTABLE": {
            "kind": "rne_141_motor_construction_source_transition", "schema_version": 1,
            "status": "DRAFT_NOT_FROZEN", "baseline_report_sha256": None,
            "before_source_sha256": None, "after_source_sha256": None,
            "changed_existing_paths": None, "added_paths": [reader.ADDED_KEY], "removed_paths": [],
        },
        "actual_execution_in_preparation": {"report_reads": 0, "report_comparisons": 0,
                                            "numerical_reanalyses": 0, "worlds": 0, "builds": 0,
                                            "checkout_or_source_copy_edits": 0, "network_writes": 0},
    }
    after = {name: (REPO / name).read_bytes() for name in SOURCE_PATHS}
    if before != after:
        raise ValueError("source-only inspection inputs changed during preparation")
    receipt = {
        "kind": "rne_source_only_141_transition_inspection", "schema_version": 1,
        "status": "PASS_SOURCE_ONLY_INSPECTION_NOT_SCIENTIFIC_QUALIFICATION",
        "head_observed": head, "checkout_porcelain_observed": status,
        "source_inputs_unchanged_during_inspection": True,
        "source_files_read": {name: descriptor(raw) for name, raw in before.items()},
        "selected_source_declaration_count": len(actual_keys),
        "selected_declaration_matches_comparator_baseline": True,
        "actual_report_reads": 0, "worlds": 0, "builds": 0,
        "draft_registration": descriptor((json.dumps(draft, indent=2, sort_keys=True) + "\n").encode()),
    }
    save("prospective-registration-draft.json", draft)
    save("source-only-inspection-receipt.json", receipt)
    print(json.dumps({"status": draft["status"], "selected_keys": len(actual_keys),
                      "conditions": len(cells), "worlds_run": 0, "actual_report_reads": 0,
                      "observed_head": head, "checkout_clean_observed": not status}, sort_keys=True))


if __name__ == "__main__":
    main()
