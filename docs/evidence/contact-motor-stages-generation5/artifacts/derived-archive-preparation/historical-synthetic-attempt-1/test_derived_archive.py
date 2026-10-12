"""Small retained synthetic archive/fault tests; no actual audit files are read."""

import errno
import gzip
import io
import json
from pathlib import Path
import tarfile
import unittest
from unittest import mock

import prepare_derived_archive as archive


TEST_ROOT = archive.OWN / "synthetic-tests" / archive.now().replace(":", "-")
TEST_ROOT.parent.mkdir(exist_ok=True)
archive.new_directory(TEST_ROOT)


class Fixture:
    def __init__(self, name):
        self.root = TEST_ROOT / name
        archive.new_directory(self.root)
        control_paths = {"actual-scientific-audit-command/actual-command-terminal.json",
                         "collection/collection-index.json",
                         "postcollection-retention-review/actual-old-tar-retention-review.json"}
        for relative in sorted(archive.REQUIRED - control_paths):
            value = self.root / relative
            value.parent.mkdir(parents=True, exist_ok=True)
            payload = b"SYNTHETIC_OPAQUE_LITERAL_ONLY\n"
            if relative in archive.DETAIL_PATHS:
                payload = gzip.compress(payload + relative.encode(), mtime=0)
            archive.write_bytes(value, payload)
        self.prep = self.root / "derived-archive-preparation"
        archive.new_directory(self.prep)
        archive.write_bytes(self.prep / "mock-tooling.py", b"# synthetic tooling excluded with full descriptor\n")
        logs = []
        for filename in ("stdout.log", "stderr.log"):
            logs.append(archive.write_bytes(self.root / "actual-scientific-audit-command" / filename,
                                            b"SYNTHETIC closed audit log\n"))
        terminal = {"actual_exit_code": 0, "ended_utc": archive.now(), "guarded_inputs_unchanged": True,
                    "rerun_or_retry": False, "stdout": logs[0], "stderr": logs[1]}
        self.cohort_id = "SYNTHETIC_DERIVED_ARCHIVE_" + name
        runs = []
        raw = self.root / "collection/raw"
        archive.new_directory(raw)
        for build in ("baseline", "rejected_inverse_w_plus_g"):
            for mass in (5, 20, 50):
                for mode in ("on", "fresh_on", "off"):
                    label = f"{build}-m{mass}-{mode}"
                    record = archive.write_bytes(raw / (label + ".jsonl.gz"), gzip.compress(b"SYNTHETIC_RAW\n" + label.encode(), mtime=0))
                    runs.append({"run_label": label, "trace": record})
        collection = {"cohort_id": self.cohort_id, "worlds_attempted": 18, "runs": runs}
        old_path = self.root / "recovery-attempt/old-authorized-raw.tar"
        old_path.parent.mkdir(parents=True, exist_ok=True)
        old = archive.write_bytes(old_path, b"SYNTHETIC_OLD_LOCAL_AUTHORIZED_REFERENCE_ONLY")
        raw_manifest = archive.write_bytes(self.root / "original-old-raw-manifest.json", b'{"synthetic_reference":true}\n')
        review = {"status": "PASS_EXACT_APPROVED_OLD_TAR_BYTES_FROM_NEW_INDEPENDENT18_COHORT",
                  "archive": old, "cohort_id": self.cohort_id}
        # Control metadata is created exactly once, independently of opaque
        # synthetic scientific payloads. No real cohort input is accessed.
        self._construct_control("actual-scientific-audit-command/actual-command-terminal.json", terminal)
        self._construct_control("collection/collection-index.json", collection)
        self._construct_control("postcollection-retention-review/actual-old-tar-retention-review.json", review)
        final_review = self.root / "postcollection-scientific-review/final-review.json"
        final_review.parent.mkdir(exist_ok=True)
        final_review_record = archive.write_json(final_review, {"synthetic_metadata_only": True})
        native = self.root / "binaries/fake-native"
        native.parent.mkdir(exist_ok=True)
        archive.write_bytes(native, b"\x7fELF_SYNTHETIC_NONEXECUTED_BINARY")
        long_path = self.root / "retained-fixtures" / ("a" * 110) / "whole-file.txt"
        long_path.parent.mkdir(parents=True, exist_ok=True)
        archive.write_bytes(long_path, b"whole unmodified long-path content")
        self.release = {
            "status": "ROOT_RELEASED_AFTER_ORIGINAL_AUDIT_TERMINAL", "synthetic_only": True,
            "new_archive_publication_authorized": False, "cohort_root": str(self.root),
            "cohort_id": self.cohort_id, "excluded_preparation_directory": str(self.prep),
            "output_directory": str(self.prep / "actual-archive"),
            "audit_terminal": archive.artifact(self.root / "actual-scientific-audit-command/actual-command-terminal.json"),
            "collection_index": archive.artifact(self.root / "collection/collection-index.json"),
            "final_audit": archive.artifact(self.root / "analysis/independent-numerical-audit.json"),
            "completed_scientific_review": final_review_record,
            "old_authorized_raw_archive": old, "old_raw_member_manifest": raw_manifest,
            "old_tar_retention_review": archive.artifact(self.root / "postcollection-retention-review/actual-old-tar-retention-review.json"),
            "helper_artifacts": [archive.artifact(archive.OWN / "prepare_derived_archive.py")],
        }
        self.record = archive.write_json(self.prep / "root-release.json", self.release)

    def _construct_control(self, relative, data):
        value = self.root / relative
        value.parent.mkdir(parents=True, exist_ok=True)
        archive.write_json(value, data)


class Tests(unittest.TestCase):
    def test_complete_closure_short_mapping_empty_pax_and_whole_six(self):
        fixture = Fixture("complete")
        pointer = archive.prepare(fixture.record)
        saved = archive.strict_json(pointer["path"])
        self.assertIsNone(saved["url"])
        self.assertFalse(saved["publication_authorized"])
        manifest = archive.strict_json(saved["manifest"]["path"])
        records = {row["original_relative_path"]: row for row in manifest["members"]}
        self.assertEqual(manifest["whole_scientific_detail_count"], 6)
        with tarfile.open(saved["archive"]["path"], "r:") as tar:
            self.assertTrue(all(row.pax_headers == {} for row in tar))
            for relative in archive.DETAIL_PATHS:
                row = records[relative]
                self.assertEqual(tar.extractfile(row["archive_member"]).read(), (fixture.root / relative).read_bytes())
        reasons = {row["reason"] for row in manifest["exclusions"]}
        self.assertTrue(any("LOCAL_ONLY_NATIVE_BINARY" in value for value in reasons))
        self.assertEqual(len(manifest["raw_member_map"]), 18)
        verification = archive.strict_json(saved["verification"]["path"])
        self.assertTrue(verification["second_stream_literal_byte_comparison"])
        self.assertGreater(verification["tar_member_count"], 19)
        with self.assertRaises(FileExistsError):
            archive.prepare(fixture.record)

    def test_unreleased_blocks_before_input_scan(self):
        fixture = Fixture("unreleased")
        fixture.release["status"] = "WAITING_FOR_ORIGINAL_AUDITOR_TERMINAL"
        rejected = archive.write_json(fixture.prep / "unreleased.json", fixture.release)
        with mock.patch.object(archive, "freeze_inputs", side_effect=AssertionError("must not scan")):
            with self.assertRaisesRegex(ValueError, "released"):
                archive.prepare(rejected)

    def test_missing_complete_scientific_review_blocks(self):
        fixture = Fixture("missing-review")
        fixture.release["completed_scientific_review"]["path"] = str(fixture.root / "postcollection-scientific-review/not-yet-created.json")
        rejected = archive.write_json(fixture.prep / "missing-review.json", fixture.release)
        with self.assertRaises(ValueError):
            archive.prepare(rejected)

    def test_closed_input_file_set_drift_blocks_proposed_pointer(self):
        fixture = Fixture("file-set-drift")
        original = archive.tar_stream
        def changed(writer, members):
            if writer.compare:
                archive.write_bytes(fixture.root / "later-unfrozen-proof.txt", b"SYNTHETIC drift after freeze")
            return original(writer, members)
        with mock.patch.object(archive, "tar_stream", side_effect=changed):
            with self.assertRaisesRegex(ValueError, "file set changed"):
                archive.prepare(fixture.record)
        self.assertFalse((fixture.prep / "actual-archive/proposed-derived-archive-pointer.json").exists())

    def test_enospc_preserves_partial_tar_and_original_inputs(self):
        fixture = Fixture("enospc")
        details = [archive.artifact(fixture.root / relative) for relative in archive.DETAIL_PATHS]
        with mock.patch.object(archive, "tar_stream", side_effect=OSError(errno.ENOSPC, "SYNTHETIC fault only")):
            with self.assertRaises(OSError):
                archive.prepare(fixture.record)
        output = fixture.prep / "actual-archive"
        self.assertTrue((output / "rne-contact-motor-stage-generation5-complete-derived-evidence.tar").exists())
        self.assertEqual(archive.strict_json(output / "archive-terminal.json")["exit_code"], 1)
        self.assertEqual(details, [archive.artifact(fixture.root / relative) for relative in archive.DETAIL_PATHS])

    def test_byte_mismatch_and_nonpersistent_paths_reject(self):
        comparison = archive.Writer(io.BytesIO(b"original"), compare=True)
        with self.assertRaisesRegex(ValueError, "byte-for-byte"):
            comparison.write(b"mutated!")
        with self.assertRaisesRegex(ValueError, "nonpersistent"):
            archive.path("/tmp/never-an-archive")


if __name__ == "__main__":
    with archive.new_stream(TEST_ROOT / "actual-tests.log") as logfile:
        stream = io.TextIOWrapper(logfile, encoding="utf-8", write_through=True)
        result = unittest.TextTestRunner(stream=stream, verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Tests))
        archive.sync(stream)
        stream.detach()
    receipt = archive.write_json(TEST_ROOT / "actual-pure-tests.json", {
        "status": "PASS_SMALL_SYNTHETIC_DERIVED_ARCHIVE_TESTS" if result.wasSuccessful() else "FAIL_SMALL_SYNTHETIC_DERIVED_ARCHIVE_TESTS",
        "tests": result.testsRun, "failures": len(result.failures), "errors": len(result.errors),
        "worlds_builds_network_actual_audit_reads": 0, "log": archive.artifact(TEST_ROOT / "actual-tests.log"),
        "sources": [archive.artifact(archive.OWN / name) for name in ("prepare_derived_archive.py", "test_derived_archive.py")],
        "retained_synthetic_root": str(TEST_ROOT),
    })
    print(json.dumps(receipt, sort_keys=True))
    raise SystemExit(0 if result.wasSuccessful() else 1)
