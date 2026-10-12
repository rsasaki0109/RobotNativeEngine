"""Pure byte comparison for the prospective, separately registered 141 arm.

This module never opens a report, launches a world, or writes an artifact. Its
caller must authenticate all frozen protocol/tool/binary/source bindings before
supplying actual bytes. The synthetic tests are deliberately self-contained.
"""

from __future__ import annotations

import hashlib
import json
import re
from decimal import Decimal
from typing import Any


SOURCE_FIELD = "selected_source_sha256"
REPORT_KEYS = (
    "schema_version", "protocol", SOURCE_FIELD, "cases",
    "numerical_sensitivity", "summary",
)
BEFORE_KEYS = frozenset({
    "examples/141_position_motor_calibration/lib.rs",
    "examples/141_position_motor_calibration/main.rs",
    "examples/141_position_motor_calibration/Cargo.toml",
    "Cargo.toml", "Cargo.lock", "third_party/rapier3d/Cargo.toml",
    "third_party/rapier3d/src/dynamics/joint/motor_model.rs",
    "third_party/rapier3d/src/dynamics/joint/multibody_joint/multibody.rs",
    "third_party/rapier3d/src/dynamics/joint/multibody_joint/multibody_joint.rs",
    "third_party/rapier3d/src/dynamics/joint/multibody_joint/unit_multibody_joint.rs",
    "third_party/rapier3d/src/dynamics/solver/joint_constraint/joint_generic_constraint.rs",
    "third_party/rapier3d/src/dynamics/solver/velocity_solver.rs",
    "third_party/rapier3d/src/dynamics/solver/island_solver.rs",
    "third_party/rapier3d/src/pipeline/physics_pipeline.rs",
    "third_party/rapier3d/src/pipeline/multibody_motor_impulse.rs",
    "third_party/rapier3d/src/pipeline/multibody_motor_stage.rs",
    "third_party/rapier3d/src/pipeline/mod.rs",
    "third_party/rapier3d/src/dynamics/solver/joint_constraint/joint_generic_constraint_builder.rs",
    "third_party/rapier3d/src/dynamics/solver/joint_constraint/joint_constraints_set.rs",
    "third_party/rapier3d/src/dynamics/rigid_body.rs",
    "third_party/rapier3d/src/dynamics/rigid_body_components.rs",
})
ADDED_KEY = "third_party/rapier3d/src/pipeline/multibody_motor_construction.rs"
PERMITTED_EXISTING_CHANGES = frozenset({
    "examples/141_position_motor_calibration/lib.rs",
    "Cargo.toml", "Cargo.lock",
    "third_party/rapier3d/src/dynamics/joint/multibody_joint/multibody_joint.rs",
    "third_party/rapier3d/src/dynamics/joint/multibody_joint/unit_multibody_joint.rs",
    "third_party/rapier3d/src/dynamics/solver/velocity_solver.rs",
    "third_party/rapier3d/src/dynamics/solver/island_solver.rs",
    "third_party/rapier3d/src/pipeline/physics_pipeline.rs",
    "third_party/rapier3d/src/pipeline/mod.rs",
    "third_party/rapier3d/src/dynamics/solver/joint_constraint/joint_generic_constraint_builder.rs",
    "third_party/rapier3d/src/dynamics/solver/joint_constraint/joint_constraints_set.rs",
})
_HEX = re.compile(r"[0-9a-f]{64}\Z")
_SPACE = b" \t\r\n"


class RejectedTransition(ValueError):
    """The prospective literal transition contract was not met."""


def _reject(message: str) -> None:
    raise RejectedTransition(message)


def _pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            _reject(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _constant(value: str) -> None:
    _reject(f"non-JSON numeric constant: {value}")


def _load(data: bytes) -> dict[str, Any]:
    try:
        value = json.loads(data.decode("utf-8"), object_pairs_hook=_pairs,
                           parse_float=Decimal, parse_constant=_constant)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        _reject(f"invalid JSON report: {error}")
    if not isinstance(value, dict) or tuple(value) != REPORT_KEYS:
        _reject("unexpected report root fields/order")
    if type(value["schema_version"]) is not int or value["schema_version"] != 2:
        _reject("expected unchanged report schema 2")
    return value


def _source_map(value: Any, keys: frozenset[str], role: str) -> dict[str, str]:
    if not isinstance(value, dict) or set(value) != keys:
        _reject(f"{role} source map has missing/extra keys")
    if any(not isinstance(digest, str) or not _HEX.fullmatch(digest)
           for digest in value.values()):
        _reject(f"{role} source map has a noncanonical digest")
    return value


def validate_frozen_transition(spec: dict[str, Any]) -> tuple[dict[str, str], dict[str, str]]:
    """Validate literal maps; authenticity of the freeze is the caller's duty.

    A status string cannot authenticate a freeze receipt. Actual invocation must
    independently verify the pre-collection registration and tool/source/binary
    descriptors. This pure function intentionally performs no filesystem IO.
    """
    keys = {"kind", "schema_version", "status", "baseline_report_sha256",
            "before_source_sha256", "after_source_sha256",
            "changed_existing_paths", "added_paths", "removed_paths"}
    if not isinstance(spec, dict) or set(spec) != keys:
        _reject("unexpected frozen transition fields")
    if (spec["kind"] != "rne_141_motor_construction_source_transition"
            or type(spec["schema_version"]) is not int or spec["schema_version"] != 1
            or spec["status"] != "FROZEN_BEFORE_COLLECTION"):
        _reject("transition is not a frozen pre-collection contract")
    if not isinstance(spec["baseline_report_sha256"], str) or not _HEX.fullmatch(spec["baseline_report_sha256"]):
        _reject("baseline report digest is not frozen")
    before = _source_map(spec["before_source_sha256"], BEFORE_KEYS, "before")
    after = _source_map(spec["after_source_sha256"], BEFORE_KEYS | {ADDED_KEY}, "after")
    changed = spec["changed_existing_paths"]
    if (not isinstance(changed, list) or any(not isinstance(path, str) for path in changed)
            or changed != sorted(set(changed))):
        _reject("changed paths must be a sorted, unique literal list")
    if not set(changed) <= PERMITTED_EXISTING_CHANGES:
        _reject("source change is outside the prospectively planned scope")
    if "examples/141_position_motor_calibration/lib.rs" not in changed:
        _reject("141 self-binding must change when the source list is extended")
    actual = sorted(path for path in BEFORE_KEYS if before[path] != after[path])
    if changed != actual:
        _reject("source values differ from the precisely declared transition")
    if spec["added_paths"] != [ADDED_KEY] or spec["removed_paths"] != []:
        _reject("only the declared construction module addition is permitted")
    return before, after


def _skip_space(data: bytes, index: int) -> int:
    while index < len(data) and data[index] in _SPACE:
        index += 1
    return index


def _string_end(data: bytes, start: int) -> int:
    if start >= len(data) or data[start] != 34:
        _reject("JSON string expected while locating source span")
    index = start + 1
    while index < len(data):
        byte = data[index]
        if byte == 92:
            index += 2
        elif byte == 34:
            return index + 1
        else:
            index += 1
    _reject("unterminated JSON string")


def _value_end(data: bytes, start: int) -> int:
    if data[start] == 34:
        return _string_end(data, start)
    if data[start] in (91, 123):
        stack = [93 if data[start] == 91 else 125]
        index = start + 1
        while index < len(data):
            byte = data[index]
            if byte == 34:
                index = _string_end(data, index)
                continue
            if byte in (91, 123):
                stack.append(93 if byte == 91 else 125)
            elif byte in (93, 125):
                if not stack or stack.pop() != byte:
                    _reject("invalid JSON nesting")
                if not stack:
                    return index + 1
            index += 1
        _reject("unterminated JSON value")
    index = start
    while index < len(data) and data[index] not in b",}" + _SPACE:
        index += 1
    return index


def _source_span(data: bytes) -> tuple[int, int]:
    """Locate a top-level value in UTF-8 bytes, never a lookalike in a string."""
    index = _skip_space(data, 0)
    if index >= len(data) or data[index] != 123:
        _reject("JSON object expected")
    index += 1
    found: tuple[int, int] | None = None
    while True:
        index = _skip_space(data, index)
        if data[index] == 125:
            break
        end = _string_end(data, index)
        key = json.loads(data[index:end].decode("utf-8"))
        index = _skip_space(data, end)
        if data[index] != 58:
            _reject("JSON colon expected")
        start = _skip_space(data, index + 1)
        end = _value_end(data, start)
        if key == SOURCE_FIELD:
            if found is not None:
                _reject("source field is repeated")
            found = start, end
        index = _skip_space(data, end)
        if data[index] == 125:
            break
        if data[index] != 44:
            _reject("JSON comma expected")
        index += 1
    if found is None:
        _reject("source field is absent")
    return found


def canonical_source_value(mapping: dict[str, str]) -> bytes:
    """Match the unchanged collector's two-space pretty BTreeMap encoding."""
    return json.dumps(dict(sorted(mapping.items())), indent=2).replace("\n", "\n  ").encode("ascii")


def expected_bytes(baseline: bytes, spec: dict[str, Any]) -> bytes:
    """Replace the source object only; preserve every other original byte."""
    before, after = validate_frozen_transition(spec)
    if hashlib.sha256(baseline).hexdigest() != spec["baseline_report_sha256"]:
        _reject("baseline report differs from its frozen literal digest")
    report = _load(baseline)
    if report[SOURCE_FIELD] != before:
        _reject("baseline source map differs from the frozen before map")
    start, end = _source_span(baseline)
    if baseline[start:end] != canonical_source_value(before):
        _reject("baseline source encoding differs from the collector's literal encoding")
    return baseline[:start] + canonical_source_value(after) + baseline[end:]


def compare_reports(baseline: bytes, current: bytes, spec: dict[str, Any]) -> dict[str, Any]:
    """Require byte equality after exactly the registered source replacement.

    All numeric/native/state/calibration contents are opaque and are retained
    literally. This neither recalculates numerical findings nor converts FAIL
    into PASS. A separate collector verifier must still validate the new report.
    """
    expected = expected_bytes(baseline, spec)
    _, after = validate_frozen_transition(spec)
    report = _load(current)
    if report[SOURCE_FIELD] != after:
        _reject("current source map differs from the frozen after map")
    if current != expected:
        _reject("bytes outside the precisely registered source transition changed")
    return {
        "kind": "rne_literal_141_source_transition_comparison", "schema_version": 1,
        "status": "PASS_LITERAL_SOURCE_OBJECT_ONLY",
        "baseline_report_sha256": hashlib.sha256(baseline).hexdigest(),
        "current_report_sha256": hashlib.sha256(current).hexdigest(),
        "before_selected_source_count": 21, "after_selected_source_count": 22,
        "changed_existing_paths": spec["changed_existing_paths"],
        "added_paths": spec["added_paths"], "removed_paths": [],
        "all_other_report_bytes_equal": True,
        "numerical_reanalysis_performed": False,
    }
