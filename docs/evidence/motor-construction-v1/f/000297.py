"""Prospective, durable once-only 141 source-transition preparation and driver.

No build or world is launched by prepare/freeze/source-guard. Run is a separate
explicit command after authenticated pre-data inputs and independent review.
One collector process internally attempts 24 cells x ON/fresh_ON/OFF worlds.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import stat
import subprocess
from typing import Any


HERE = Path(__file__).resolve().parent
COMPARATOR = HERE.parent / "compare_scalar_source_transition.py"
_module = importlib.util.spec_from_file_location("scalar_transition_comparator", COMPARATOR)
assert _module is not None and _module.loader is not None
compare = importlib.util.module_from_spec(_module)
_module.loader.exec_module(compare)
HEX = re.compile(r"[0-9a-f]{64}\Z")
COLLECTION_STDOUT = b"collection_complete=true completed_worlds=72/72 replay_qualification=true primary_calibration=FAIL mode=collect\n"
VERIFY_STDOUT = COLLECTION_STDOUT.replace(b"mode=collect", b"mode=verify-report-no-worlds")
STALE_MESSAGE = b"report selected sources are stale/mixed"
SUMMARY = {"collection_complete": True, "worlds_attempted": 72, "worlds_created": 72,
           "completed_worlds": 72, "replay_qualification_pass": True,
           "primary_calibration_pass": False, "primary_nonzero_cells": 4,
           "primary_nonzero_passes": 0, "zero_target_cells": 8, "zero_target_passes": 8}
ROLES = {"binary", "build_receipt", "source_guard", "protocol", "baseline_report",
         "baseline_source_map_receipt", "driver_source", "comparator_source", "test_source",
         "test_runner_source", "pure_test_receipt", "pure_test_log"}


class Refused(ValueError):
    """An input is not qualified; no retry permission follows."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise Refused(message)


def now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def pairs(items):
    result = {}
    for key, value in items:
        require(key not in result, f"duplicate JSON key: {key}")
        result[key] = value
    return result


def nonfinite(value):
    raise Refused(f"nonfinite JSON literal: {value}")


def load(path: Path) -> Any:
    return json.loads(path.read_bytes().decode("utf-8"), object_pairs_hook=pairs,
                      parse_constant=nonfinite)


def canonical(value: Any) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def fsync_directory(path: Path) -> None:
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def exclusive_json(path: Path, value: Any) -> None:
    with path.open("xb") as stream:
        stream.write(canonical(value))
        stream.flush()
        os.fsync(stream.fileno())
    fsync_directory(path.parent)


def plain_absolute(value: str) -> Path:
    require(isinstance(value, str), "path is not a string")
    path = Path(value)
    require(path.is_absolute() and path == path.resolve(), "path must be absolute, canonical, and nonsymlinked")
    require(path == Path("/workspace") or Path("/workspace") in path.parents,
            "persistent artifacts must be under /workspace")
    return path


def stream_descriptor(path: Path, sync: bool = False) -> dict[str, Any]:
    path = plain_absolute(str(path))
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        before = os.fstat(fd)
        require(stat.S_ISREG(before.st_mode), f"not a regular file: {path}")
        h = hashlib.sha256()
        length = 0
        while True:
            block = os.read(fd, 1 << 20)
            if not block:
                break
            length += len(block)
            h.update(block)
        if sync:
            os.fsync(fd)
        after = os.fstat(fd)
        require((before.st_ino, before.st_size, before.st_mtime_ns, before.st_ctime_ns)
                == (after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns)
                and length == after.st_size, f"file changed during streaming hash: {path}")
        return {"path": str(path), "sha256": h.hexdigest(), "size_bytes": length}
    finally:
        os.close(fd)


def check_descriptor(expected: dict[str, Any]) -> dict[str, Any]:
    require(isinstance(expected, dict) and set(expected) == {"path", "sha256", "size_bytes"},
            "artifact descriptor fields differ")
    require(isinstance(expected["sha256"], str) and bool(HEX.fullmatch(expected["sha256"])),
            "artifact digest is not lowercase full SHA-256")
    require(type(expected["size_bytes"]) is int and expected["size_bytes"] >= 0,
            "artifact size is not a nonnegative integer")
    actual = stream_descriptor(plain_absolute(expected["path"]))
    require(actual == expected, f"artifact bytes differ: {expected['path']}")
    return actual


def discover_source_inputs(root: Path) -> list[str]:
    """Tracked/untracked nonignored Rust/Cargo/config plus all vendor inputs.

    This explicitly does not claim an exhaustive compiler/dependency closure.
    The binary/build receipt binds external compiler/dependency configuration.
    """
    env = dict(os.environ, GIT_OPTIONAL_LOCKS="0")
    raw = subprocess.check_output(["git", "ls-files", "-z", "--cached", "--others", "--exclude-standard"],
                                  cwd=root, env=env)
    names = {item.decode("utf-8") for item in raw.split(b"\0") if item}
    selected = {name for name in names if (name.endswith((".rs", ".toml"))
                or Path(name).name in {"Cargo.lock", "rust-toolchain"}
                or name.startswith(".cargo/"))}
    vendor = root / "third_party/rapier3d"
    for current, directories, files in os.walk(vendor):
        directories[:] = sorted(name for name in directories if name not in {"target", ".git"})
        for name in sorted(files):
            if name.endswith((".rs", ".toml")) or name == "Cargo.lock":
                selected.add(str((Path(current) / name).relative_to(root)))
    require(all(name.isascii() and not name.startswith("/") and ".." not in Path(name).parts
                for name in selected), "noncanonical source guard path")
    return sorted(selected)


def source_snapshot(root: Path) -> dict[str, Any]:
    names = discover_source_inputs(root)
    return {"kind": "rne_141_source_guard", "schema_version": 1, "source_root": str(root),
            "scope": "tracked/nonignored Rust/Cargo/config plus all vendor .rs/.toml/lock; non-exhaustive compiler closure",
            "inputs": [{"path": name, **{k: v for k, v in stream_descriptor(root / name).items() if k != "path"}}
                       for name in names]}


def check_source_guard(guard: dict[str, Any], after: dict[str, str]) -> dict[str, Any]:
    require(isinstance(guard, dict) and set(guard) == {"kind", "schema_version", "source_root", "scope", "inputs"},
            "source guard fields differ")
    require(guard["kind"] == "rne_141_source_guard" and type(guard["schema_version"]) is int
            and guard["schema_version"] == 1 and "non-exhaustive" in guard["scope"], "source guard scope/schema differs")
    root = plain_absolute(guard["source_root"])
    actual = source_snapshot(root)
    require(canonical(actual) == canonical(guard), "source guard file list/bytes/types differ")
    mapping = {entry["path"]: entry["sha256"] for entry in actual["inputs"]}
    require(all(mapping.get(name) == sha for name, sha in after.items()),
            "selected after-map is not the actual frozen source input bytes")
    return actual


def fixed_cells() -> list[dict[str, Any]]:
    cells = []
    for gain in ["G1Posture", "G1Ankle"]:
        for target in ["-0.001_f32", "0.0_f32", "0.001_f32"]:
            for small_steps in [1, 32]:
                for pgs in [1, 2]:
                    cells.append({"index": len(cells), "gain_rust_variant": gain,
                                  "target_rust_literal": target, "solver_small_steps": small_steps,
                                  "internal_pgs_passes": pgs, "fresh_world_order": ["ON", "fresh_ON", "OFF"]})
    return cells


def fixed_protocol() -> dict[str, Any]:
    return {"kind": "rne_141_separate72_motor_construction_source_transition", "schema_version": 1,
            "status": "REGISTERED_BEFORE_COLLECTION", "condition_count": 24, "worlds_per_condition": 3,
            "maximum_world_attempts": 72, "collector_process_count": 1,
            "collector_cli": ["--output", "<new-report-path>", "--require-calibration"],
            "expected_collector_original_exit": 2, "primary_calibration_required_result": "FAIL_0_OF_4",
            "new_construction_observer_enabled": False, "collector_logic_change_allowed": False,
            "case_schema_codec_criteria_change_allowed": False, "numeric_filtering_allowed": False,
            "retry_or_world_replacement_allowed": False,
            "separate_from": ["generation-5 fixed18", "example-143 six construction worlds"],
            "cells": fixed_cells(), "postcollection_gate_order": ["literal_comparator", "verify_current",
                                                                    "verify_current_require", "verify_old_stale"],
            "postcollection_verification_worlds": 0}


def validate_inputs(inputs: dict[str, Any]) -> dict[str, Any]:
    expected = {"kind", "schema_version", "run_root", "source_root", "environment", "transition", "artifacts"}
    require(isinstance(inputs, dict) and set(inputs) == expected, "supplied binding fields differ")
    require(inputs["kind"] == "rne_141_source_transition_inputs" and type(inputs["schema_version"]) is int
            and inputs["schema_version"] == 1, "supplied binding kind/schema differs")
    run_root = plain_absolute(inputs["run_root"])
    require(not run_root.exists(), "run root already exists; no retry/overwrite")
    source_root = plain_absolute(inputs["source_root"])
    require(isinstance(inputs["environment"], dict)
            and all(isinstance(k, str) and isinstance(v, str) for k, v in inputs["environment"].items()),
            "child environment must be an explicit literal string map")
    artifacts = inputs["artifacts"]
    require(isinstance(artifacts, dict) and ROLES <= set(artifacts), "required artifact roles are missing")
    require(all(isinstance(role, str) and role for role in artifacts), "artifact role is invalid")
    for role, descriptor in artifacts.items():
        check_descriptor(descriptor)
        path = plain_absolute(descriptor["path"])
        require(path != run_root and run_root not in path.parents, "an input artifact overlaps the new output root")
    require(artifacts["driver_source"]["path"] == str(Path(__file__).resolve())
            and artifacts["comparator_source"]["path"] == str(COMPARATOR)
            and artifacts["test_source"]["path"] == str(HERE / "test_driver.py")
            and artifacts["test_runner_source"]["path"] == str(HERE / "run_tests.py"), "current tool paths are not pinned")
    before, after = compare.validate_frozen_transition(inputs["transition"])
    require(artifacts["baseline_report"]["sha256"] == inputs["transition"]["baseline_report_sha256"],
            "baseline original-file digest differs from transition")
    protocol = load(Path(artifacts["protocol"]["path"]))
    require(canonical(protocol) == canonical(fixed_protocol()), "registered protocol differs from unchanged fixed72 arm")
    guard = load(Path(artifacts["source_guard"]["path"]))
    require(guard["source_root"] == str(source_root), "guard root differs from bound working directory")
    check_source_guard(guard, after)
    baseline_binding = load(Path(artifacts["baseline_source_map_receipt"]["path"]))
    require(canonical(baseline_binding) == canonical({"kind": "rne_141_baseline_source_map_binding", "schema_version": 1,
                                 "status": "PASS_BASELINE_SOURCE_MAP_BINDING",
                                 "baseline_report_sha256": artifacts["baseline_report"]["sha256"],
                                 "before_source_sha256": before}), "baseline literal source-map provenance is not bound")
    build = load(Path(artifacts["build_receipt"]["path"]))
    require(build.get("kind") == "rne_141_compiler_build_binding"
            and type(build.get("schema_version")) is int and build["schema_version"] == 1
            and type(build.get("original_exit_code")) is int and build["original_exit_code"] == 0
            and build.get("binary_sha256") == artifacts["binary"]["sha256"]
            and build.get("source_guard_sha256") == artifacts["source_guard"]["sha256"]
            and build.get("profile") == "release" and build.get("scalar") == "f32" and build.get("dimension") == "dim3"
            and isinstance(build.get("features"), list) and all(isinstance(v, str) for v in build["features"])
            and build["features"] == sorted(set(build["features"]))
            and all(isinstance(build.get(key), str) and build[key] for key in ["rustc_version", "cargo_version", "target"])
            and isinstance(build.get("build_flags"), dict), "actual binary/source/compiler/feature build binding is incomplete")
    require(os.access(artifacts["binary"]["path"], os.X_OK), "frozen binary is not executable")
    pure = load(Path(artifacts["pure_test_receipt"]["path"]))
    require(pure.get("kind") == "rne_141_synthetic_driver_test_receipt"
            and type(pure.get("schema_version")) is int and pure["schema_version"] == 1
            and pure.get("status") == "PASS" and type(pure.get("original_exit_code")) is int and pure["original_exit_code"] == 0
            and type(pure.get("actual_report_reads")) is int and pure["actual_report_reads"] == 0
            and type(pure.get("worlds")) is int and pure["worlds"] == 0
            and pure.get("source_inputs_unchanged") is True
            and canonical(pure.get("log")) == canonical(artifacts["pure_test_log"])
            and pure.get("tested_tool_sha256") == {role: artifacts[role]["sha256"]
                                                  for role in ["driver_source", "comparator_source", "test_source", "test_runner_source"]},
            "pure-test receipt does not bind the actual tool closure")
    # Rehash every input after all source/metadata checks. This includes the old
    # report as an opaque streamed file, never decoded before collection.
    for descriptor in artifacts.values():
        check_descriptor(descriptor)
    return inputs


def prepare(inputs_path: Path, destination: Path) -> Path:
    supplied = check_descriptor(stream_descriptor(inputs_path))
    inputs = validate_inputs(load(inputs_path))
    destination = plain_absolute(str(destination))
    destination.mkdir(exist_ok=False)
    fsync_directory(destination.parent)
    value = {"kind": "rne_141_prepared_precollection_binding", "schema_version": 1,
             "status": "PREPARED_REVIEW_REQUIRED_NOT_AUTHORIZED_TO_RUN", "created_at": now(),
             "supplied_inputs": supplied, "inputs": inputs,
             "worlds_started": 0, "baseline_numeric_report_decodes": 0}
    output = destination / "prepared-bindings.json"
    exclusive_json(output, value)
    exclusive_json(destination / "terminal.json", {"status": "PASS_PREPARATION_ONLY", "original_exit_code": 0,
                                                    "prepared": stream_descriptor(output), "worlds": 0})
    return output


def freeze(prepared_path: Path, review_descriptor: dict[str, Any], destination: Path) -> Path:
    prepared_descriptor = stream_descriptor(prepared_path)
    prepared = load(prepared_path)
    require(prepared.get("kind") == "rne_141_prepared_precollection_binding"
            and prepared.get("status") == "PREPARED_REVIEW_REQUIRED_NOT_AUTHORIZED_TO_RUN",
            "prepared binding is not the reviewable pre-data artifact")
    validate_inputs(prepared["inputs"])
    check_descriptor(prepared["supplied_inputs"])
    check_descriptor(review_descriptor)
    review = load(Path(review_descriptor["path"]))
    require(review.get("kind") == "rne_141_independent_precollection_review"
            and review.get("status") == "PASS_INDEPENDENT_PRE_COLLECTION_SOURCE_TRANSITION_REVIEW"
            and review.get("before_world_collection") is True
            and review.get("reviewed_prepared_binding_sha256") == prepared_descriptor["sha256"]
            and review.get("reviewed_prepared_binding_size_bytes") == prepared_descriptor["size_bytes"]
            and review.get("world_budget") == 72 and type(review.get("world_budget")) is int,
            "independent pre-data review does not authenticate this exact preparation")
    destination = plain_absolute(str(destination))
    destination.mkdir(exist_ok=False)
    fsync_directory(destination.parent)
    result = {"kind": "rne_141_frozen_precollection_binding", "schema_version": 1,
              "status": "FROZEN_BEFORE_COLLECTION", "created_at": now(),
              "prepared": prepared_descriptor, "independent_review": review_descriptor,
              "prepared_binding": prepared, "worlds_started": 0,
              "authentication_scope": "hash-bound original prepared artifact and independent review; no cryptographic identity attestation"}
    output = destination / "frozen-bindings.json"
    exclusive_json(output, result)
    exclusive_json(destination / "terminal.json", {"status": "PASS_FREEZE_NO_WORLDS", "original_exit_code": 0,
                                                    "frozen": stream_descriptor(output), "worlds": 0})
    return output


def validate_frozen(path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    frozen = load(path)
    require(frozen.get("kind") == "rne_141_frozen_precollection_binding"
            and type(frozen.get("schema_version")) is int and frozen["schema_version"] == 1
            and frozen.get("status") == "FROZEN_BEFORE_COLLECTION"
            and type(frozen.get("worlds_started")) is int and frozen["worlds_started"] == 0,
            "run requires the actual pre-data freeze")
    check_descriptor(frozen["prepared"])
    check_descriptor(frozen["independent_review"])
    require(canonical(load(Path(frozen["prepared"]["path"]))) == canonical(frozen["prepared_binding"]),
            "embedded prepared binding differs from its original file")
    review = load(Path(frozen["independent_review"]["path"]))
    require(review.get("status") == "PASS_INDEPENDENT_PRE_COLLECTION_SOURCE_TRANSITION_REVIEW"
            and review.get("before_world_collection") is True
            and review.get("reviewed_prepared_binding_sha256") == frozen["prepared"]["sha256"]
            and review.get("reviewed_prepared_binding_size_bytes") == frozen["prepared"]["size_bytes"]
            and type(review.get("world_budget")) is int and review["world_budget"] == 72,
            "review is stale or not precollection")
    prepared = frozen["prepared_binding"]
    check_descriptor(prepared["supplied_inputs"])
    return frozen, validate_inputs(prepared["inputs"])


def process(root: Path, name: str, argv: list[str], cwd: Path, environment: dict[str, str], world_role: str,
            progress: dict[str, Any] | None = None) -> dict[str, Any]:
    directory = root / name
    directory.mkdir(exist_ok=False)
    fsync_directory(root)
    start = {"kind": "rne_141_process_start", "argv": argv, "cwd": str(cwd), "environment": environment,
             "started_at": now(), "world_role": world_role, "process_count": 1}
    exclusive_json(directory / "start.json", start)
    exit_code = None
    failure = None
    pid_receipt_error = None
    child = None
    execution_error = None
    cleanup_error = None
    pending_interrupt = None
    progress = {} if progress is None else progress
    progress.update(actual_process_launched=None, observed_pid=None, observed_wait_return=None,
                    returned_terminal=None, terminal_written=False)
    with (directory / "stdout.log").open("xb") as stdout, (directory / "stderr.log").open("xb") as stderr:
        try:
            child = subprocess.Popen(argv, cwd=cwd, env=environment, stdout=stdout, stderr=stderr)
            progress.update(actual_process_launched=True, observed_pid=child.pid)
            try:
                exclusive_json(directory / "pid.json", {"pid": child.pid, "started_at": now()})
            except OSError as error:
                # A launched child is still waited exactly once. Preserve its
                # original exit even if durable PID receipt writing failed.
                pid_receipt_error = f"{type(error).__name__}: {error}"
            exit_code = child.wait()
            progress["observed_wait_return"] = exit_code
        except BaseException as error:
            if not isinstance(error, Exception):
                pending_interrupt = error
                progress["_pending_interrupt"] = error
                progress["interrupt_received"] = f"{type(error).__name__}: {error}"
            if child is None:
                failure = f"{type(error).__name__}: {error}"
                # A normal Popen exec failure proves the requested executable
                # did not start. An interrupted call without a handle is unknown.
                if isinstance(error, OSError):
                    progress["actual_process_launched"] = False
            else:
                execution_error = f"{type(error).__name__}: {error}"
                try:
                    if child.poll() is None:
                        child.terminate()
                    try:
                        exit_code = child.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        exit_code = child.wait()
                    progress["observed_wait_return"] = exit_code
                except BaseException as cleanup:
                    cleanup_error = f"{type(cleanup).__name__}: {cleanup}"
                    progress["cleanup_error"] = cleanup_error
        finally:
            try:
                stdout.flush()
                stderr.flush()
                os.fsync(stdout.fileno())
                os.fsync(stderr.fileno())
            except BaseException as storage:
                progress["storage_error"] = f"{type(storage).__name__}: {storage}"
                if pending_interrupt is not None:
                    raise pending_interrupt from storage
                raise
    terminal = {**start, "finished_at": now(), "original_exit_code": exit_code, "launch_error": failure,
                "process_launched": progress["actual_process_launched"], "pid_receipt_error": pid_receipt_error,
                "execution_error": execution_error, "cleanup_error": cleanup_error,
                "stdout": stream_descriptor(directory / "stdout.log"),
                "stderr": stream_descriptor(directory / "stderr.log")}
    # Preserve the actual completed return receipt in caller-owned memory before
    # a late storage error can interrupt its durable per-process serialization.
    progress["returned_terminal"] = terminal
    try:
        exclusive_json(directory / "terminal.json", terminal)
        progress["terminal_written"] = True
    except BaseException as storage:
        progress["storage_error"] = f"{type(storage).__name__}: {storage}"
        if pending_interrupt is not None:
            raise pending_interrupt from storage
        raise
    if pending_interrupt is not None:
        raise pending_interrupt
    return terminal


def public_progress(progress: dict[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in progress.items() if not key.startswith("_")}


def recover_collector_receipt(result: dict[str, Any], progress: dict[str, Any]) -> None:
    terminal = progress.get("returned_terminal")
    if terminal is not None and type(terminal["process_launched"]) is bool:
        check_descriptor(terminal["stdout"])
        check_descriptor(terminal["stderr"])
        result["collector_processes_started"] = int(terminal["process_launched"])
        result["collector_original_exit_code"] = terminal["original_exit_code"]
        result["collector_receipt_recovery"] = "ACTUAL_FORMED_RETURN_RECEIPT_VERIFIED"
    elif progress.get("launch_reserved"):
        result["collector_processes_started"] = None
        result["collector_original_exit_code"] = None
        result["collector_receipt_recovery"] = "UNKNOWN_ON_RESERVED_ATTEMPT"
        result["world_accounting"] = {"status": "UNKNOWN_ON_RESERVED_ATTEMPT", "world_counts": None}
    result["collector_launch_state"] = public_progress(progress)


def accounting(report_path: Path) -> dict[str, Any]:
    if not report_path.exists():
        return {"status": "UNAVAILABLE_NO_REPORT", "world_counts": None}
    try:
        report = load(report_path)
        summary = report["summary"]
        # These are literal acquisition counters, not numerical reanalysis.
        require(isinstance(summary, dict), "missing retained summary")
        for name in ["worlds_attempted", "worlds_created", "completed_worlds"]:
            require(type(summary.get(name)) is int and 0 <= summary[name] <= 72,
                    "invalid retained world counter")
        counts = {name: summary[name] for name in ["worlds_attempted", "worlds_created", "completed_worlds"]}
        valid = (set(summary) == set(SUMMARY)
                 and all(type(summary[k]) is type(v) and summary[k] == v for k, v in SUMMARY.items())
                 and isinstance(report.get("cases"), list) and len(report["cases"]) == 24)
        if valid:
            for case in report["cases"]:
                runs = case.get("runs")
                valid = valid and isinstance(runs, list) and len(runs) == 3
                if not valid:
                    break
                valid = all(isinstance(run, dict) and run.get("collector_enabled") is enabled
                            and run.get("world_created") is True and run.get("completed_step") is True
                            and "error" in run and run["error"] is None
                            for run, enabled in zip(runs, [True, True, False]))
                if not valid:
                    break
        return {"status": "COMPLETE_FIXED72_FAIL0_OF4" if valid else "RETAINED_INCOMPLETE_OR_CHANGED",
                "world_counts": counts, "case_count_retained": len(report.get("cases", []))}
    except Exception as error:
        return {"status": "UNAVAILABLE_UNREADABLE_PARTIAL_REPORT", "world_counts": None,
                "error": f"{type(error).__name__}: {error}"}


def run(frozen_path: Path, expected_frozen_sha256: str) -> dict[str, Any]:
    require(isinstance(expected_frozen_sha256, str) and bool(HEX.fullmatch(expected_frozen_sha256)),
            "run requires the independently retained expected frozen-file SHA-256")
    require(stream_descriptor(frozen_path)["sha256"] == expected_frozen_sha256,
            "frozen file differs from independently retained pre-data SHA-256")
    frozen, inputs = validate_frozen(frozen_path)
    frozen_descriptor = stream_descriptor(frozen_path)
    root = plain_absolute(inputs["run_root"])
    root.mkdir(exist_ok=False)  # Durable global once-only claim. Never removed.
    fsync_directory(root.parent)
    exclusive_json(root / "claim.json", {"kind": "rne_141_once_only_attempt_claim", "created_at": now(),
                                          "frozen": frozen_descriptor, "world_attempt_budget": 72,
                                          "collector_process_budget": 1, "retry_allowed": False})
    artifacts = inputs["artifacts"]
    source_root = plain_absolute(inputs["source_root"])
    binary = artifacts["binary"]["path"]
    report_path = root / "report.json"  # Collector creates this with create_new.
    result = {"kind": "rne_141_once_only_attempt_terminal", "schema_version": 1,
              "started_at": now(), "frozen": frozen_descriptor, "status": "FAIL_RETAINED_NO_RETRY",
              "collector_processes_started": 0, "collector_original_exit_code": None,
              "report": None, "world_accounting": {"status": "NOT_LAUNCHED", "world_counts": None},
              "postcollection_processes_started": 0, "source_inputs_unchanged": None,
              "retry_allowed": False, "publication_status": "PROPOSED_NOT_PUBLISHED"}
    before = None
    collector_progress = {}
    gate_progresses = []
    pending_interrupt = None
    try:
        guard = load(Path(artifacts["source_guard"]["path"]))
        before = check_source_guard(guard, inputs["transition"]["after_source_sha256"])
        exclusive_json(root / "source-before.json", before)
        for descriptor in artifacts.values():
            check_descriptor(descriptor)
        # From this reservation onward zero/NOT_LAUNCHED is not a safe default:
        # a child can finish before late receipt storage raises. The persistent
        # reservation and whole-attempt claim remain consumed on every fault.
        collector_progress["launch_reserved"] = True
        result["collector_processes_started"] = None
        result["world_accounting"] = {"status": "UNKNOWN_ON_RESERVED_ATTEMPT", "world_counts": None}
        exclusive_json(root / "collector-launch-reservation.json", {
            "kind": "rne_141_collector_launch_reservation", "created_at": now(),
            "argv": [binary, "--output", str(report_path), "--require-calibration"],
            "actual_processes_started": None, "world_counts": None, "retry_allowed": False})
        child = process(root, "collection", [binary, "--output", str(report_path), "--require-calibration"],
                        source_root, inputs["environment"], "one collector; internal fixed24 x ON/fresh_ON/OFF; max72 world attempts",
                        collector_progress)
        result["collector_processes_started"] = int(child["process_launched"])
        result["collector_original_exit_code"] = child["original_exit_code"]
        result["world_accounting"] = accounting(report_path)
        if report_path.exists():
            result["report"] = stream_descriptor(report_path, sync=True)
            fsync_directory(root)
        require(child["original_exit_code"] == 2 and child["launch_error"] is None,
                "original collection did not return preregistered FAIL exit2")
        require(child["pid_receipt_error"] is None, "collector PID receipt failed; retain original exit without admission")
        require(child["execution_error"] is None and child["cleanup_error"] is None,
                "collector execution/interruption failed; no admission")
        require(Path(child["stdout"]["path"]).read_bytes() == COLLECTION_STDOUT,
                "collection stdout differs from unchanged FAIL/completeness contract")
        require(result["world_accounting"]["status"] == "COMPLETE_FIXED72_FAIL0_OF4",
                "not all 72 unchanged worlds and FAIL0/4 were retained")
        for descriptor in artifacts.values():
            check_descriptor(descriptor)
        check_source_guard(guard, inputs["transition"]["after_source_sha256"])
        # Actual baseline decode occurs only here, after authenticated freeze and
        # terminal collection. No numeric baseline reanalysis is performed.
        proof = compare.compare_reports(Path(artifacts["baseline_report"]["path"]).read_bytes(),
                                        report_path.read_bytes(), inputs["transition"])
        exclusive_json(root / "literal-comparison.json", proof)
        checks = [
            ("verify-current", [binary, "--verify-report", str(report_path)], 0, VERIFY_STDOUT, False),
            ("verify-current-require", [binary, "--verify-report", str(report_path), "--require-calibration"], 2, VERIFY_STDOUT, False),
            ("verify-old-stale", [binary, "--verify-report", artifacts["baseline_report"]["path"]], 1, None, True),
        ]
        for name, argv, expected_exit, expected_stdout, stale in checks:
            for descriptor in artifacts.values():
                check_descriptor(descriptor)
            check_source_guard(guard, inputs["transition"]["after_source_sha256"])
            original_report = stream_descriptor(report_path)
            gate_progress = {"launch_reserved": True, "phase": name}
            gate_progresses.append(gate_progress)
            result["postcollection_processes_started"] = None
            gate = process(root, name, argv, source_root, inputs["environment"], "zero worlds: --verify-report only", gate_progress)
            require(gate["original_exit_code"] == expected_exit and gate["launch_error"] is None,
                    f"{name} original exit differs")
            require(gate["pid_receipt_error"] is None, f"{name} PID receipt failed")
            require(gate["execution_error"] is None and gate["cleanup_error"] is None, f"{name} execution/interruption failed")
            if expected_stdout is not None:
                require(Path(gate["stdout"]["path"]).read_bytes() == expected_stdout, f"{name} stdout differs")
            if stale:
                require(STALE_MESSAGE in Path(gate["stderr"]["path"]).read_bytes(), "old report did not reject stale/mixed sources")
            require(stream_descriptor(report_path) == original_report, "loaded verification changed retained report")
        result["status"] = "PASS_FIXED72_LITERAL_SOURCE_TRANSITION_RETAINED_FAIL0_OF4"
    except BaseException as error:
        result["error"] = f"{type(error).__name__}: {error}"
        if not isinstance(error, Exception):
            pending_interrupt = error
    finally:
        if collector_progress.get("_pending_interrupt") is not None:
            pending_interrupt = collector_progress["_pending_interrupt"]
            result["interrupt_error"] = f"{type(pending_interrupt).__name__}: {pending_interrupt}"
        if gate_progresses:
            launched = []
            for progress in gate_progresses:
                if progress.get("_pending_interrupt") is not None:
                    pending_interrupt = progress["_pending_interrupt"]
                    result["interrupt_error"] = f"{type(pending_interrupt).__name__}: {pending_interrupt}"
                receipt = progress.get("returned_terminal")
                launched.append(receipt["process_launched"] if receipt is not None else None)
            result["postcollection_processes_started"] = (sum(int(value) for value in launched)
                if all(type(value) is bool for value in launched) else None)
            result["postcollection_launch_states"] = [public_progress(value) for value in gate_progresses]
        try:
            recover_collector_receipt(result, collector_progress)
        except Exception as recovery:
            result["collector_processes_started"] = None
            result["collector_original_exit_code"] = None
            result["collector_receipt_recovery"] = "UNKNOWN_ON_RESERVED_ATTEMPT"
            result["world_accounting"] = {"status": "UNKNOWN_ON_RESERVED_ATTEMPT", "world_counts": None}
            result["collector_recovery_error"] = f"{type(recovery).__name__}: {recovery}"
            result["status"] = "FAIL_RETAINED_NO_RETRY"
        try:
            after = source_snapshot(source_root)
            exclusive_json(root / "source-after.json", after)
            result["source_inputs_unchanged"] = before is not None and before == after
            for descriptor in artifacts.values():
                check_descriptor(descriptor)
            require(result["source_inputs_unchanged"], "source inputs changed or before snapshot unavailable")
            check_descriptor(frozen_descriptor)
            if report_path.exists():
                result["report"] = stream_descriptor(report_path, sync=True)
                if result["world_accounting"]["status"] == "UNKNOWN_ON_RESERVED_ATTEMPT":
                    observed = accounting(report_path)
                    if result.get("collector_receipt_recovery") == "ACTUAL_FORMED_RETURN_RECEIPT_VERIFIED":
                        result["world_accounting"] = observed
                    else:
                        result["world_accounting"] = {"status": "UNKNOWN_ON_RESERVED_ATTEMPT",
                            "world_counts": None, "retained_report_observation": observed}
        except Exception as error:
            result["status"] = "FAIL_RETAINED_NO_RETRY"
            result["final_guard_error"] = f"{type(error).__name__}: {error}"
        result["finished_at"] = now()
        try:
            exclusive_json(root / "terminal.json", result)
        except BaseException as storage:
            if pending_interrupt is not None:
                raise pending_interrupt from storage
            raise
    if pending_interrupt is not None:
        raise pending_interrupt
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    guard = commands.add_parser("source-guard")
    guard.add_argument("--source-root", type=Path, required=True)
    guard.add_argument("--output", type=Path, required=True)
    prep = commands.add_parser("prepare")
    prep.add_argument("--inputs", type=Path, required=True)
    prep.add_argument("--out-dir", type=Path, required=True)
    lock = commands.add_parser("freeze")
    lock.add_argument("--prepared", type=Path, required=True)
    lock.add_argument("--review-descriptor", type=Path, required=True)
    lock.add_argument("--out-dir", type=Path, required=True)
    execute = commands.add_parser("run")
    execute.add_argument("--frozen", type=Path, required=True)
    execute.add_argument("--expected-frozen-sha256", required=True)
    args = parser.parse_args()
    if args.command == "source-guard":
        exclusive_json(plain_absolute(str(args.output)), source_snapshot(plain_absolute(str(args.source_root))))
        print(str(args.output))
    elif args.command == "prepare":
        print(prepare(plain_absolute(str(args.inputs)), plain_absolute(str(args.out_dir))))
    elif args.command == "freeze":
        print(freeze(plain_absolute(str(args.prepared)), load(args.review_descriptor), plain_absolute(str(args.out_dir))))
    else:
        result = run(plain_absolute(str(args.frozen)), args.expected_frozen_sha256)
        print(json.dumps(result, sort_keys=True))
        return 0 if result["status"].startswith("PASS_") else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
