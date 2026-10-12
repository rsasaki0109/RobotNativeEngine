#!/usr/bin/env python3
"""Pure historical-baseline/source-map tamper checks; no new collected report."""

import json
from pathlib import Path

from audit_current_scalar_report import (ADDITIONS, BASELINE_SHA, CHANGES, compare_reports,
                                         coverage, expected_bytes, load_report)
from stage_schema import require, sha_bytes


def main():
    directory = Path(__file__).parent
    transition = json.loads((directory / "scalar-source-transition-preregistration.json").read_bytes())
    original = Path(transition["original_report"]["path"]).read_bytes()
    require(sha_bytes(original) == BASELINE_SHA, "immutable historical baseline changed")
    baseline = load_report(original)
    before, after = baseline["selected_source_sha256"], transition["expected_selected_source_sha256"]
    synthetic = expected_bytes(original, before, after)
    parsed = compare_reports(original, synthetic, before, after)
    reference = dict(baseline, selected_source_sha256=dict(sorted(after.items())))
    require(parsed == reference, "source object reconstruction changed semantic report fields")
    require(coverage(parsed) == {"cells": 24, "worlds": 72, "native_states": 144, "native_entities": 432,
                               "native_entity_bytes": 247248, "native_state_canonical_words": 42264,
                               "enabled_telemetry_runs": 48, "ordered_telemetry_slices": 792,
                               "telemetry_canonical_words": 10704, "primary_nonzero_passes": 0,
                               "all_nonzero_tolerance_passes": 0, "zero_target_passes": 8}, "historical coverage changed")
    rejected = []
    def fails(name, operation):
        try:
            operation()
        except ValueError:
            rejected.append(name)
        else:
            raise ValueError("tamper admitted: " + name)
    unchanged = next(name for name in before if name not in CHANGES)
    fifth = dict(after); fifth[unchanged] = "1" * 64
    fails("seventh existing source change", lambda: expected_bytes(original, before, fifth))
    missing = dict(after); missing.pop(next(iter(ADDITIONS)))
    fails("missing preregistered source addition", lambda: expected_bytes(original, before, missing))
    extra = dict(after, unexpected="2" * 64)
    fails("undeclared added source path", lambda: expected_bytes(original, before, extra))
    fails("accuracy PASS forgery", lambda: compare_reports(original, synthetic.replace(
        b'"primary_calibration_pass": false', b'"primary_calibration_pass": true', 1), before, after))
    fails("literal signed zero spelling", lambda: compare_reports(original, synthetic.replace(b'"finite": 0.0', b'"finite": -0.0', 1), before, after))
    native_marker = b'"canonical_words": [\n'
    offset = synthetic.index(native_marker) + len(native_marker)
    forged = synthetic[:offset] + synthetic[offset:].replace(b'0,', b'1,', 1)
    require(forged != synthetic, "synthetic native tamper was not created")
    fails("native IEEE state tamper", lambda: compare_reports(original, forged, before, after))
    fails("trailing bytes", lambda: compare_reports(original, synthetic + b' ', before, after))
    fails("old report reinterpreted as new hashes", lambda: compare_reports(original, original, before, after))
    print(json.dumps({"status": "PASS", "scope": "pure synthetic source-object transitions only; historical baseline read, no new report read",
                      "native_worlds": 0, "checks": ["source object semantic parity", "literal byte transition only", "historical coverage and 0/4 FAIL retained"],
                      "tamper_rejections": rejected}, indent=2))


if __name__ == "__main__":
    main()
