"""All-or-none checksum recovery, separate from numerical qualification."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import tarfile


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sync_directory(path):
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def record(path):
    path = Path(path).resolve()
    with path.open("rb") as stream:
        digest = hashlib.file_digest(stream, "sha256").hexdigest()
    return {"path": str(path), "bytes": path.stat().st_size, "sha256": digest}


def validate(descriptor):
    require(record(descriptor["path"]) == descriptor, "immutable descriptor changed")


def write_new(path, value):
    with Path(path).open("x") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    sync_directory(Path(path).parent)


class CompareStream:
    def __init__(self, path):
        self.stream = Path(path).open("rb")
        self.digest = hashlib.sha256()
        self.count = 0

    def write(self, data):
        require(self.stream.read(len(data)) == data, "second TAR stream differs")
        self.digest.update(data)
        self.count += len(data)
        return len(data)

    def finish(self):
        require(not self.stream.read(1), "second TAR stream ended early")
        self.stream.close()
        return {"bytes": self.count, "sha256": self.digest.hexdigest()}


def tar_stream(destination, members):
    with tarfile.open(fileobj=destination, mode="w|", format=tarfile.PAX_FORMAT) as archive:
        for name, path in sorted(members.items()):
            info = tarfile.TarInfo(name)
            info.size = path.stat().st_size
            info.uid = info.gid = info.mtime = 0
            info.uname = info.gname = ""
            info.mode = 0o444
            info.type = tarfile.REGTYPE
            info.pax_headers = {}
            with path.open("rb") as stream:
                archive.addfile(info, stream)


def reconstruct(collection_path, registration_path, registration_sha, output):
    registration_descriptor = record(registration_path)
    require(registration_descriptor["sha256"] == registration_sha, "registration SHA changed")
    registration = json.loads(Path(registration_path).read_text())
    validate(registration["original_member_manifest"])
    manifest_path = Path(registration["original_member_manifest"]["path"])
    manifest = json.loads(manifest_path.read_text())
    collection_descriptor = record(collection_path)
    collection = json.loads(Path(collection_path).read_text())
    require(type(collection["schema_version"]) is int and collection["schema_version"] == 2 and
            type(collection["generation"]) is int and collection["generation"] == 5, "wrong collection schema")
    require(collection["cohort_id"] == registration["new_cohort_id"], "wrong new cohort")
    require(len(collection["runs"]) == len(manifest["members"]) == 18, "missing fixed cases")
    rows = {row["run_label"]: row for row in collection["runs"]}
    require(len(rows) == 18 and set(rows) == {row["run_label"] for row in manifest["members"]}, "duplicate or unknown case")
    output = Path(output).resolve()
    require(str(output).startswith("/workspace/"), "output must be persistent workspace storage")
    output.mkdir(exist_ok=False)
    sync_directory(output.parent)
    receipt = {
        "kind": "independent_checksum_only_original_archive_reconstruction",
        "registered_arm": registration_descriptor,
        "new_collection": collection_descriptor,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "old_numerical_qualification": "FAILED_UNQUALIFIED_UNCHANGED",
        "new_collection_remains_independent": True,
        "extra_worlds": 0,
        "numerical_native_payload_parsing": 0,
        "original_collection_records_rewritten": False,
        "matches": [],
        "status": "COMPARISON_STARTED",
    }
    write_new(output / "reconstruction-claim.json", receipt)
    members = {"raw-trace-manifest.json": manifest_path}
    try:
        for expected in manifest["members"]:
            row = rows[expected["run_label"]]
            require(row["attempted"] is True and type(row["exit_code"]) is int and row["exit_code"] == 0,
                    "original new-cohort process did not complete")
            require(all(row[key] == expected[key] for key in ("build", "mass_kg", "mode")), "fixed cell identity changed")
            validate(row["trace"])
            actual = record(row["trace"]["path"])
            matched = actual["bytes"] == expected["size_bytes"] and actual["sha256"] == expected["sha256"]
            receipt["matches"].append({"run_label": expected["run_label"], "expected_sha256": expected["sha256"],
                                       "expected_bytes": expected["size_bytes"], "actual": actual, "matched": matched})
            members[expected["basename"]] = Path(actual["path"])
        if not all(row["matched"] for row in receipt["matches"]):
            receipt["status"] = "RAW_CHECKSUMS_DO_NOT_ALL_MATCH_ORIGINAL"
            return 1
        expected_archive = registration["expected_original_archive"]
        archive_path = output / expected_archive["basename"]
        with archive_path.open("xb") as destination:
            tar_stream(destination, members)
            destination.flush()
            os.fsync(destination.fileno())
        sync_directory(archive_path.parent)
        actual_archive = record(archive_path)
        receipt["archive"] = actual_archive
        require(actual_archive["bytes"] == expected_archive["bytes"], "original archive byte count differs")
        require(actual_archive["sha256"] == expected_archive["sha256"], "original archive SHA differs")
        compare = CompareStream(archive_path)
        tar_stream(compare, members)
        second = compare.finish()
        require(second == {"bytes": actual_archive["bytes"], "sha256": actual_archive["sha256"]}, "second stream digest differs")
        for row in receipt["matches"]:
            validate(row["actual"])
        validate(registration["original_member_manifest"])
        validate(collection_descriptor)
        receipt["second_stream"] = second
        receipt["status"] = "BYTE_IDENTICAL_ORIGINAL_ARCHIVE_RECONSTRUCTED_FROM_INDEPENDENT_NEW_COHORT"
        return 0
    except BaseException as error:
        receipt["status"] = "RECONSTRUCTION_FAILED_WITHOUT_RETRY_OR_SUBSTITUTION"
        receipt["error"] = {"type": type(error).__name__, "message": str(error)}
        raise
    finally:
        receipt["ended_utc"] = datetime.now(timezone.utc).isoformat()
        write_new(output / "reconstruction-terminal.json", receipt)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--collection", required=True)
    parser.add_argument("--registration", required=True)
    parser.add_argument("--registration-sha256", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    raise SystemExit(reconstruct(args.collection, args.registration, args.registration_sha256, args.output))
