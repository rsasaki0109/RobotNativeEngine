#!/usr/bin/env python3
"""Pure source-derived native serde shape and identity tamper checks."""

import json

from stage_analysis import identity
from stage_schema import native_joint_handle_words, validate_row
from test_stage_reader import fixture


def main():
    checked = []
    rejected = []
    source_json = b'{"index":3,"generation":0}'
    native = json.loads(source_json)
    assert native_joint_handle_words(native) == (3, 0)
    assert native_joint_handle_words({"generation": 7, "index": (1 << 32) - 1}) == ((1 << 32) - 1, 7)
    checked.extend(["literal Rust Index object", "u32 maximum and full generation", "JSON object key order does not change identity"])
    protocol, frame = fixture()
    event = frame["stage_telemetry"]["events"][2]
    row = event["rows"][0]
    validate_row(row, len(event["generalized_velocity_words"]), protocol["resource_ceilings"])
    assert identity(row) == (3, 0, 1, "LinY", 0)
    assert row["joint_handle"] == native
    checked.extend(["row validation uses native object", "identity retains index and generation", "raw native object unmodified"])
    invalid = [
        ("normalized array is rejected", [3, 0]),
        ("tuple fallback rejected", (3, 0)),
        ("missing generation", {"index": 3}),
        ("missing index", {"generation": 0}),
        ("unexpected field", {"index": 3, "generation": 0, "extra": 1}),
        ("bool index", {"index": True, "generation": 0}),
        ("bool generation", {"index": 3, "generation": False}),
        ("negative index", {"index": -1, "generation": 0}),
        ("negative generation", {"index": 3, "generation": -1}),
        ("index overflow", {"index": 1 << 32, "generation": 0}),
        ("generation overflow", {"index": 3, "generation": 1 << 32}),
        ("float index", {"index": 3.0, "generation": 0}),
        ("string generation", {"index": 3, "generation": "0"}),
        ("null handle", None),
    ]
    for name, value in invalid:
        try:
            native_joint_handle_words(value)
        except ValueError:
            rejected.append(name)
        else:
            raise AssertionError("invalid native handle admitted: " + name)
    print(json.dumps({"status": "PASS", "native_worlds": 0, "scope": "pure Rust-serde object and identity checks; no raw trace read",
                      "checked": checked, "rejected": rejected}, indent=2))


if __name__ == "__main__":
    main()
