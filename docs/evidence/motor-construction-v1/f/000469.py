"""Synthetic reports only: no checkout/report/raw-data access or world creation."""

import copy
import hashlib
import json
import unittest

import compare_scalar_source_transition as reader


def digest(label):
    return hashlib.sha256(label.encode("ascii")).hexdigest()


def encoded(value):
    return (json.dumps(value, indent=2, ensure_ascii=True) + "\n").encode("ascii")


def fixture():
    before = {key: digest("synthetic before " + key) for key in sorted(reader.BEFORE_KEYS)}
    after = dict(before)
    changed = sorted(reader.PERMITTED_EXISTING_CHANGES - {"Cargo.toml", "Cargo.lock"})
    for key in changed:
        after[key] = digest("synthetic after " + key)
    after[reader.ADDED_KEY] = digest("synthetic added module")
    # The protocol string contains delimiter/lookalike text to exercise span selection.
    report = {
        "schema_version": 2,
        "protocol": {"fixture": "SYNTHETIC ONLY", "nested": {"selected_source_sha256": {}},
                     "lookalike": "\"selected_source_sha256\": { \\" + "quote}"},
        "selected_source_sha256": before,
        "cases": [{"synthetic_index": i,
                   "cell": {"gain": "synthetic gain", "target": -0.0, "small_steps": 32},
                   "native_state": {"postcard": [0, 128, 255], "word_bits": "80000000"},
                   "motor_impulse": {"finite": 0.001, "native_bits": "3a83126f"},
                   "observers": ["ON", "fresh_ON", "OFF"]} for i in range(24)],
        "numerical_sensitivity": [{"opaque_synthetic": -0.0}],
        "summary": {"primary_calibration": "FAIL", "primary_nonzero_passes": 0,
                    "primary_nonzero_cells": 4, "completed_worlds": 72},
    }
    old = encoded(report)
    spec = {
        "kind": "rne_141_motor_construction_source_transition", "schema_version": 1,
        "status": "FROZEN_BEFORE_COLLECTION",
        "baseline_report_sha256": hashlib.sha256(old).hexdigest(),
        "before_source_sha256": before, "after_source_sha256": dict(sorted(after.items())),
        "changed_existing_paths": changed, "added_paths": [reader.ADDED_KEY], "removed_paths": [],
    }
    return report, old, spec


class SyntheticFullReportTransition(unittest.TestCase):
    def setUp(self):
        self.report, self.old, self.spec = fixture()
        self.current = reader.expected_bytes(self.old, self.spec)

    def mutated(self, change):
        value = json.loads(self.current)
        change(value)
        return encoded(value)

    def reject_current(self, data):
        with self.assertRaises(reader.RejectedTransition):
            reader.compare_reports(self.old, data, self.spec)

    def test_exact_source_replacement(self):
        result = reader.compare_reports(self.old, self.current, self.spec)
        self.assertTrue(result["all_other_report_bytes_equal"])
        self.assertFalse(result["numerical_reanalysis_performed"])
        self.assertIn(b'"primary_calibration": "FAIL"', self.current)

    def test_independent_full_encoding_matches_source_only_replacement(self):
        reference = copy.deepcopy(self.report)
        reference[reader.SOURCE_FIELD] = self.spec["after_source_sha256"]
        independently_encoded = encoded(reference)
        self.assertEqual(self.current, independently_encoded)
        reader.compare_reports(self.old, independently_encoded, self.spec)

    def test_cargo_integration_values_only_when_declared_before_collection(self):
        spec = copy.deepcopy(self.spec)
        for key in ("Cargo.toml", "Cargo.lock"):
            spec["after_source_sha256"][key] = digest("synthetic integrated " + key)
        spec["changed_existing_paths"] = sorted(spec["changed_existing_paths"] + ["Cargo.toml", "Cargo.lock"])
        current = reader.expected_bytes(self.old, spec)
        reader.compare_reports(self.old, current, spec)

    def test_drop_case(self):
        self.reject_current(self.mutated(lambda r: r["cases"].pop()))

    def test_extra_case(self):
        self.reject_current(self.mutated(lambda r: r["cases"].append(copy.deepcopy(r["cases"][0]))))

    def test_reordered_cases(self):
        self.reject_current(self.mutated(lambda r: r["cases"].reverse()))

    def test_changed_gain(self):
        self.reject_current(self.mutated(lambda r: r["cases"][0]["cell"].update(gain="tampered")))

    def test_changed_native_postcard_byte(self):
        self.reject_current(self.mutated(lambda r: r["cases"][0]["native_state"]["postcard"].__setitem__(1, 129)))

    def test_changed_native_word(self):
        self.reject_current(self.mutated(lambda r: r["cases"][0]["native_state"].update(word_bits="00000000")))

    def test_changed_impulse(self):
        self.reject_current(self.mutated(lambda r: r["cases"][0]["motor_impulse"].update(finite=0.002)))

    def test_changed_signed_zero(self):
        self.reject_current(self.current.replace(b'"target": -0.0', b'"target": 0.0', 1))

    def test_bool_integer_equality_cannot_hide_payload_change(self):
        self.reject_current(self.current.replace(b'"synthetic_index": 1,', b'"synthetic_index": true,', 1))

    def test_integer_float_equality_cannot_hide_payload_change(self):
        self.reject_current(self.current.replace(b'"small_steps": 32', b'"small_steps": 32.0', 1))

    def test_nonfinite_literal_spellings(self):
        for literal in (b"NaN", b"Infinity", b"-Infinity"):
            with self.subTest(literal=literal):
                self.reject_current(self.current.replace(b'"target": -0.0', b'"target": ' + literal, 1))

    def test_forged_primary_pass(self):
        self.reject_current(self.mutated(lambda r: r["summary"].update(primary_calibration="PASS", primary_nonzero_passes=4)))

    def test_missing_source(self):
        self.reject_current(self.mutated(lambda r: r[reader.SOURCE_FIELD].pop(reader.ADDED_KEY)))

    def test_extra_source(self):
        self.reject_current(self.mutated(lambda r: r[reader.SOURCE_FIELD].update({"extra.rs": digest("extra")})))

    def test_undeclared_source_hash(self):
        self.reject_current(self.mutated(lambda r: r[reader.SOURCE_FIELD].update({"Cargo.lock": digest("undeclared")})))

    def test_unchanged_old_report_not_current(self):
        self.reject_current(self.old)

    def test_whitespace_outside_source(self):
        self.reject_current(self.current.replace(b'"schema_version": 2', b'"schema_version":  2', 1))

    def test_extra_trailing_whitespace(self):
        self.reject_current(self.current + b" ")

    def test_duplicate_nested_key(self):
        self.reject_current(self.current.replace(b'"word_bits": "80000000"', b'"word_bits": "80000000", "word_bits": "80000000"', 1))

    def test_duplicate_source_key(self):
        source = next(iter(self.spec["after_source_sha256"]))
        token = json.dumps(source).encode() + b": " + json.dumps(self.spec["after_source_sha256"][source]).encode()
        self.reject_current(self.current.replace(token, token + b",\n    " + token, 1))

    def test_source_object_reordering(self):
        r = json.loads(self.current)
        r[reader.SOURCE_FIELD] = dict(reversed(list(r[reader.SOURCE_FIELD].items())))
        self.reject_current(encoded(r))

    def test_invalid_json_constant(self):
        self.reject_current(self.current.replace(b'"target": -0.0', b'"target": NaN', 1))

    def test_malformed_digest(self):
        spec = copy.deepcopy(self.spec)
        spec["after_source_sha256"][reader.ADDED_KEY] = "A" * 64
        with self.assertRaises(reader.RejectedTransition):
            reader.expected_bytes(self.old, spec)

    def test_draft_contract_rejected(self):
        spec = copy.deepcopy(self.spec)
        spec["status"] = "DRAFT_NOT_FROZEN"
        with self.assertRaises(reader.RejectedTransition):
            reader.expected_bytes(self.old, spec)

    def test_declared_change_must_actually_change(self):
        spec = copy.deepcopy(self.spec)
        spec["changed_existing_paths"] = sorted(spec["changed_existing_paths"] + ["Cargo.lock"])
        with self.assertRaises(reader.RejectedTransition):
            reader.expected_bytes(self.old, spec)

    def test_changed_hash_must_be_declared(self):
        spec = copy.deepcopy(self.spec)
        spec["after_source_sha256"]["Cargo.lock"] = digest("unregistered")
        with self.assertRaises(reader.RejectedTransition):
            reader.expected_bytes(self.old, spec)

    def test_outside_planned_source_scope(self):
        spec = copy.deepcopy(self.spec)
        key = "third_party/rapier3d/src/dynamics/rigid_body.rs"
        spec["after_source_sha256"][key] = digest("unplanned")
        spec["changed_existing_paths"] = sorted(spec["changed_existing_paths"] + [key])
        with self.assertRaises(reader.RejectedTransition):
            reader.expected_bytes(self.old, spec)

    def test_baseline_digest_cannot_change(self):
        with self.assertRaises(reader.RejectedTransition):
            reader.expected_bytes(self.old + b"\n", self.spec)

    def test_baseline_source_payload_cannot_be_rebound(self):
        old = self.old.replace(b'"Cargo.lock": "', b'"Cargo.lock": "0', 1)
        spec = copy.deepcopy(self.spec)
        spec["baseline_report_sha256"] = hashlib.sha256(old).hexdigest()
        with self.assertRaises(reader.RejectedTransition):
            reader.expected_bytes(old, spec)

    def test_new_root_or_schema_cannot_change(self):
        self.reject_current(self.mutated(lambda r: r.update(schema_version=3)))
        self.reject_current(self.mutated(lambda r: r.update(extra_root=True)))

    def test_source_span_lookalikes_are_not_replaced(self):
        before = json.loads(self.old)["protocol"]
        after = json.loads(self.current)["protocol"]
        self.assertEqual(before, after)
        self.assertEqual(json.loads(self.current)[reader.SOURCE_FIELD], self.spec["after_source_sha256"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
