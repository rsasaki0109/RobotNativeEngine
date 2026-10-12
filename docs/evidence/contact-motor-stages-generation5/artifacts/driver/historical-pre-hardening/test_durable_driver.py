"""Synthetic subprocess and file-integrity tests; no robot binary is executed."""

import contextlib
import errno
import gzip
import io
import json
import os
from pathlib import Path
import signal
import threading
import unittest
from unittest import mock

import collect_once
import durable_common as common
import prepare_archive


DRIVER = Path(__file__).resolve().parent
TEST_ROOT = DRIVER / "synthetic-tests" / (common.now().replace(":", "-") + f"-{os.getpid()}")
TEST_ROOT.parent.mkdir(exist_ok=True)
common.mkdir_new(TEST_ROOT)

SYNTHETIC_BINARY = '''#!/usr/bin/env python3
import argparse, gzip, json, os, signal
from pathlib import Path
p=argparse.ArgumentParser()
for name in ('mass-kg','observer','label','output'):
    p.add_argument('--'+name, required=True)
a=p.parse_args()
config=json.loads(Path(__file__).with_suffix('.config.json').read_text())
with Path(config['counter']).open('ab') as counter:
    counter.write((a.label+'\\n').encode());counter.flush();os.fsync(counter.fileno())
with Path(a.output).open('xb') as raw:
    if config.get('bad_gzip'):
        raw.write(b'SYNTHETIC_INVALID_GZIP_NOT_A_ROBOT_OBSERVATION')
    else:
        with gzip.GzipFile(filename='', mode='wb', fileobj=raw, mtime=0) as encoder:
            encoder.write((json.dumps({'synthetic_process_only':True,'label':a.label,'mass':a.mass_kg,'observer':a.observer})+'\\n').encode())
    raw.flush();os.fsync(raw.fileno())
print('SYNTHETIC_PROCESS_ONLY '+a.label, flush=True)
if config.get('change_source') and a.label=='baseline-m5-on':
    with Path(config['change_source']).open('ab') as changed:
        changed.write(b'// synthetic source drift\\n');changed.flush();os.fsync(changed.fileno())
if config.get('signal_label')==a.label:
    os.kill(os.getpid(), signal.SIGTERM)
raise SystemExit(7 if config.get('fail_label')==a.label else 0)
'''


class SyntheticFixture:
    def __init__(self, name, *, fail_label=None, signal_label=None, change_source=False, bad_gzip=False):
        self.root = TEST_ROOT / name
        common.mkdir_new(self.root)
        self.repository = self.root / "fake-repository"
        common.mkdir_new(self.repository)
        self.source = self.repository / "source.rs"
        common.write_new_bytes(self.source, b"// SYNTHETIC INPUT ONLY: no robot source\n")
        self.source_original = common.write_new_bytes(self.root / "frozen-source-original.rs", self.source.read_bytes())
        self.counter = self.root / "synthetic-process-starts.log"
        protocol = common.write_new_json(self.root / "protocol.json", {
            "fixed_cells": common.fixed_cells(), "authoritative_coupled_worlds": 18,
            "synthetic_test_only": True,
        })
        reader = common.write_new_bytes(self.root / "fake-reader.py", b"# SYNTHETIC FROZEN READER PLACEHOLDER\n")
        pure = common.write_new_json(self.root / "reader-pure-tests.json", {"synthetic": True, "native_worlds": 0})
        review = common.write_new_json(self.root / "readonly-review.json", {"synthetic": True, "native_worlds": 0})
        self.guard = common.write_new_json(self.root / "source-guard.json", {
            "scope": "Fixed synthetic sorted input map; non-exhaustive compiler/dependency closure",
            "files": {"source.rs": common.artifact(self.source)["sha256"]},
        })
        bindings = {}
        self.binaries = {}
        for build in common.BUILDS:
            executable = self.root / (build + ".py")
            common.write_new_bytes(executable, SYNTHETIC_BINARY.encode())
            executable.chmod(0o700)
            common.fsync_directory(executable.parent)
            common.write_new_json(executable.with_suffix(".config.json"), {
                "counter": str(self.counter), "fail_label": fail_label, "signal_label": signal_label,
                "change_source": str(self.source) if change_source else None, "bad_gzip": bad_gzip,
            })
            binary_record = common.artifact(executable)
            self.binaries[build] = binary_record
            bindings[build] = common.write_new_json(self.root / (build + "-binding.json"), {
                "status": "FROZEN_BEFORE_GENERATION5_COHORT_WORLDS", "build": build,
                "cohort_id": "synthetic-" + name, "protocol_sha256": protocol["sha256"],
                "authoritative_worlds_of_this_cohort_before_freeze": 0,
                "binary": binary_record, "source_manifest": self.guard,
            })
        self.spec = {
            "schema_version": 1, "generation": 5, "cohort_id": "synthetic-" + name,
            "status": common.SPEC_STATUS, "independence": common.INDEPENDENCE,
            "authoritative_worlds_of_this_cohort_before_freeze": 0, "world_limit": 18,
            "fixed_cells": common.fixed_cells(), "cohort_root": str(self.root / "collection"),
            "repository_root": str(self.repository), "source_guard_manifest": self.guard,
            "protocol": protocol, "reader_artifacts": [reader], "reader_pure_test_receipt": pure,
            "independent_readonly_review": review,
            "preflight_gates": {"reader_pure_tests": "PASS", "independent_readonly_review": "PASS"},
            "driver_artifacts": [common.artifact(DRIVER / name) for name in
                                 ("durable_common.py", "collect_once.py", "prepare_archive.py")],
            "bindings": bindings,
        }
        self.record = common.write_new_json(self.root / "frozen-driver-spec.json", self.spec)

    def collect(self, **kwargs):
        log = io.StringIO()
        with contextlib.redirect_stdout(log):
            result = collect_once.collect(self.record, **kwargs)
        common.write_new_bytes(self.root / "actual-synthetic-driver.stdout", log.getvalue().encode())
        return result

    def starts(self):
        return self.counter.read_text().splitlines() if self.counter.exists() else []

    def index(self):
        return common.strict_json(Path(self.spec["cohort_root"]) / "collection-index.json")

    def raw_descriptors(self):
        return [common.artifact(Path(self.spec["cohort_root"]) / "raw" / (cell["run_label"] + ".jsonl.gz"))
                for cell in common.fixed_cells()]


class DriverTests(unittest.TestCase):
    def test_complete_once_and_archive_literal_second_stream(self):
        fixture = SyntheticFixture("complete")
        self.assertEqual(fixture.collect(), 0)
        self.assertEqual(fixture.starts(), [cell["run_label"] for cell in common.fixed_cells()])
        before = fixture.raw_descriptors()
        with self.assertRaises(FileExistsError):
            collect_once.collect(fixture.record)
        self.assertEqual(len(fixture.starts()), 18)
        pointer = prepare_archive.prepare(fixture.record)
        value = common.strict_json(pointer["path"])
        self.assertEqual(value["status"], "PROPOSED_LOCAL_ARCHIVE_NOT_PUBLISHED")
        self.assertIsNone(value["remote_url"])
        self.assertFalse(value["restores_lost_generation4_archive"])
        verified = common.strict_json(value["verification"]["path"])
        self.assertTrue(verified["second_stream_byte_comparison"])
        self.assertEqual(len(verified["members"]), 19)
        self.assertEqual(verified["first_stream_sha256"], verified["second_stream_sha256"])
        self.assertEqual(before, fixture.raw_descriptors())
        with self.assertRaises(FileExistsError):
            prepare_archive.prepare(fixture.record)

    def test_nonzero_original_exit_does_not_filter_remaining_cases(self):
        fixture = SyntheticFixture("exit-seven", fail_label="baseline-m5-on")
        self.assertEqual(fixture.collect(), 1)
        index = fixture.index()
        self.assertEqual(len(fixture.starts()), 18)
        self.assertEqual(index["runs"][0]["exit_code"], 7)
        self.assertEqual(index["worlds_completed"], 17)
        receipt = common.strict_json(index["case_receipts"][0]["path"])
        self.assertEqual(receipt["original_exit_code"], 7)
        pointer = prepare_archive.prepare(fixture.record)
        self.assertEqual(common.strict_json(pointer["path"])["collection_status"], index["status"])

    def test_signal_original_exit_is_retained_without_substitution(self):
        fixture = SyntheticFixture("signal", signal_label="baseline-m5-on")
        self.assertEqual(fixture.collect(), 1)
        self.assertEqual(fixture.index()["runs"][0]["exit_code"], -signal.SIGTERM)
        self.assertEqual(len(fixture.starts()), 18)

    def test_spawn_failure_reservation_cannot_retry(self):
        fixture = SyntheticFixture("spawn-failure")
        def failure(*_args, **_kwargs):
            raise OSError(errno.ENOENT, "synthetic spawn failure")
        self.assertEqual(fixture.collect(popen=failure), 1)
        index = fixture.index()
        self.assertEqual(index["worlds_launch_reserved"], 1)
        self.assertEqual(index["worlds_attempted"], 0)
        self.assertIsNone(index["runs"][0]["exit_code"])
        with self.assertRaises(FileExistsError):
            collect_once.collect(fixture.record)
        self.assertEqual(fixture.starts(), [])

    def test_abrupt_interruption_after_claim_refuses_resume(self):
        fixture = SyntheticFixture("abrupt-interruption")
        def interruption(*_args, **_kwargs):
            raise KeyboardInterrupt("synthetic abrupt driver interruption before launch")
        with self.assertRaises(KeyboardInterrupt):
            fixture.collect(popen=interruption)
        inspected = collect_once.inspect(fixture.spec["cohort_root"])
        self.assertIsNone(inspected["final_terminal"])
        self.assertTrue(inspected["cases"][0]["launch_reserved"])
        self.assertFalse(inspected["retry_or_resume_permitted"])
        with self.assertRaises(FileExistsError):
            collect_once.collect(fixture.record)
        self.assertEqual(fixture.starts(), [])

    def test_changed_source_stops_with_original_exit_and_raw_retained(self):
        fixture = SyntheticFixture("source-drift", change_source=True)
        self.assertEqual(fixture.collect(), 1)
        index = fixture.index()
        self.assertEqual(len(fixture.starts()), 1)
        self.assertEqual(index["runs"][0]["exit_code"], 0)
        self.assertIsNotNone(index["runs"][0]["trace"])
        receipt = common.strict_json(index["case_receipts"][0]["path"])
        self.assertFalse(receipt["source_inputs_unchanged"])
        self.assertNotEqual(common.strict_json(index["runs"][0]["source_guard_before"]["path"]),
                            common.strict_json(index["runs"][0]["source_guard_after"]["path"]))
        common.validate_artifact(fixture.source_original)

    def test_changed_binary_blocks_before_cohort_claim(self):
        fixture = SyntheticFixture("binary-drift")
        binary = Path(fixture.binaries["baseline"]["path"])
        common.write_new_bytes(fixture.root / "frozen-original-executable.py", binary.read_bytes())
        with binary.open("ab") as changed:
            changed.write(b"# synthetic binary drift\n")
            common.sync_stream(changed)
        with self.assertRaisesRegex(ValueError, "binding mismatch"):
            collect_once.collect(fixture.record)
        self.assertFalse(Path(fixture.spec["cohort_root"]).exists())
        self.assertEqual(fixture.starts(), [])

    def test_missing_reader_gate_and_changed_matrix_block_before_claim(self):
        for name, mutate in [
            ("missing-gate", lambda spec: spec["preflight_gates"].update(reader_pure_tests="PENDING")),
            ("changed-matrix", lambda spec: spec["fixed_cells"].reverse()),
            ("unfrozen", lambda spec: spec.update(status="UNFROZEN_TEMPLATE")),
        ]:
            fixture = SyntheticFixture(name)
            mutate(fixture.spec)
            record = common.write_new_json(fixture.root / "rejected-spec.json", fixture.spec)
            with self.assertRaises(ValueError):
                collect_once.collect(record)
            self.assertFalse(Path(fixture.spec["cohort_root"]).exists())
            self.assertEqual(fixture.starts(), [])

    def test_concurrent_second_driver_cannot_launch_another_cohort(self):
        fixture = SyntheticFixture("concurrent")
        started = threading.Event()
        release = threading.Event()
        real_popen = collect_once.subprocess.Popen
        outcomes = []
        def blocked(*args, **kwargs):
            started.set()
            self.assertTrue(release.wait(timeout=10))
            return real_popen(*args, **kwargs)
        def first():
            outcomes.append(fixture.collect(popen=blocked))
        thread = threading.Thread(target=first)
        thread.start()
        self.assertTrue(started.wait(timeout=10))
        with self.assertRaises(FileExistsError):
            collect_once.collect(fixture.record)
        release.set()
        thread.join(timeout=60)
        self.assertFalse(thread.is_alive())
        self.assertEqual(outcomes, [0])
        self.assertEqual(len(fixture.starts()), 18)

    def test_invalid_gzip_retained_and_no_proposed_pointer_created(self):
        fixture = SyntheticFixture("invalid-gzip", bad_gzip=True)
        self.assertEqual(fixture.collect(), 0)
        before = fixture.raw_descriptors()
        with self.assertRaises(gzip.BadGzipFile):
            prepare_archive.prepare(fixture.record)
        destination = Path(fixture.spec["cohort_root"]) / "archive-preparation"
        self.assertFalse((destination / "proposed-archive-pointer.json").exists())
        self.assertEqual(common.strict_json(destination / "archive-terminal.json")["exit_code"], 1)
        self.assertEqual(before, fixture.raw_descriptors())

    def test_synthetic_enospc_preserves_partial_tar_and_raws(self):
        fixture = SyntheticFixture("synthetic-enospc")
        self.assertEqual(fixture.collect(), 0)
        before = fixture.raw_descriptors()
        with mock.patch.object(prepare_archive, "write_tar_stream", side_effect=OSError(errno.ENOSPC, "SYNTHETIC fault only")):
            with self.assertRaises(OSError):
                prepare_archive.prepare(fixture.record)
        destination = Path(fixture.spec["cohort_root"]) / "archive-preparation"
        self.assertTrue((destination / "contact-motor-stages-generation5-raw.tar").exists())
        self.assertFalse((destination / "proposed-archive-pointer.json").exists())
        self.assertEqual(before, fixture.raw_descriptors())

    def test_byte_mismatch_is_rejected_even_without_hash_comparison(self):
        comparison = prepare_archive.ComparingWriter(io.BytesIO(b"literal-original"))
        with self.assertRaisesRegex(ValueError, "byte-for-byte"):
            comparison.write(b"literal-mutated!")

    def test_duplicate_json_and_nonpersistent_paths_are_rejected(self):
        duplicate = TEST_ROOT / "duplicate.json"
        common.write_new_bytes(duplicate, b'{"a":1,"a":2}')
        with self.assertRaisesRegex(ValueError, "duplicate"):
            common.strict_json(duplicate)
        with self.assertRaisesRegex(ValueError, "persistent"):
            common.persistent_path("/tmp/not-durable")


if __name__ == "__main__":
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(DriverTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    starts = sorted(TEST_ROOT.glob("*/synthetic-process-starts.log"))
    receipt = common.write_new_json(TEST_ROOT / "actual-synthetic-tests-receipt.json", {
        "status": "PASS_SYNTHETIC_DRIVER_FAIL_PATHS" if result.wasSuccessful() else "FAIL_SYNTHETIC_DRIVER_FAIL_PATHS",
        "tests_run": result.testsRun, "failures": len(result.failures), "errors": len(result.errors),
        "synthetic_subprocess_starts": sum(len(path.read_text().splitlines()) for path in starts),
        "robot_worlds": 0, "builds": 0, "network_publications": 0,
        "artifacts_preserved_under": str(TEST_ROOT),
        "driver_artifacts": [common.artifact(DRIVER / name) for name in
                             ("durable_common.py", "collect_once.py", "prepare_archive.py", "test_durable_driver.py")],
    })
    print(json.dumps(receipt, sort_keys=True))
    raise SystemExit(0 if result.wasSuccessful() else 1)
