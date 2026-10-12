#!/usr/bin/env python3
"""Retain the complete closed generation-5 derived evidence, without publication.

Scientific gzip/JSON payloads are copied and hashed as opaque bytes. No numerical
payload is parsed, selected, shortened, repaired or rerun by this helper.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import tarfile


G5 = Path("/workspace/.rne-kick-demo/single-joint-motor-calibration/next-experiment/generation-5")
OWN = Path(__file__).resolve().parent
BLOCK = 1 << 20
OLD_BYTES = 97996800
OLD_SHA = "534e38498ccbbcba99bf1e1b26a77f37469d8c5596429cbf0dd13b5a55d329b0"
DETAIL_PATHS = tuple(sorted(
    f"analysis/details/{build}-m{mass}-on.independent-details.jsonl.gz"
    for build in ("baseline", "rejected_inverse_w_plus_g") for mass in (5, 20, 50)
))
REQUIRED = set(DETAIL_PATHS) | {
    "analysis/independent-numerical-audit.json",
    "actual-scientific-audit-command/actual-command-terminal.json",
    "actual-collection-command/actual-command-terminal.json",
    "actual-checksum-recovery-command/actual-command-terminal.json",
    "collection/collection-index.json", "collection/collection-terminal.json",
    "freeze/frozen-protocol-v2.json", "freeze/frozen-driver-spec.json",
    "freeze/baseline-source-binary-binding-v2.json",
    "freeze/rejected_inverse_w_plus_g-source-binary-binding-v2.json",
    "reader/audit_coupled_stages.py", "reader/stage_analysis.py", "reader/stage_numeric.py",
    "reader/stage_schema.py", "reader/stage_roles.py", "reader/synthetic-profile.json",
    "reader/actual-pure-test-receipt.json",
    "fixture-review/source-derived-fixture-spec.json", "fixture-review/source-review-proof.json",
    "fixture-review/source-derived-fixture-review.md",
    "driver/durable_common.py", "driver/collect_once.py", "driver/prepare_archive.py",
    "recovery/predata-recovery-registration-v2.json", "recovery-attempt/reconstruction-terminal.json",
    "postcollection-retention-review/actual-old-tar-retention-review.json",
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def now():
    return datetime.now(timezone.utc).isoformat()


def strict_json(path):
    def pairs(values):
        result = {}
        for key, value in values:
            require(key not in result, "duplicate JSON key")
            result[key] = value
        return result
    def nonfinite(_value):
        raise ValueError("nonfinite metadata JSON token")
    return json.loads(Path(path).read_bytes(), object_pairs_hook=pairs, parse_constant=nonfinite)


def literal(value):
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


def path(value, beneath=None):
    value = Path(value)
    require(value.is_absolute() and value == value.resolve(), "noncanonical/symlink path")
    require(value.is_relative_to(Path("/workspace")), "nonpersistent archive/input path")
    if beneath is not None:
        require(value.is_relative_to(Path(beneath)), "path outside registered scope")
    return value


def hash_file(value):
    digest, count = hashlib.sha256(), 0
    with Path(value).open("rb") as source:
        for chunk in iter(lambda: source.read(BLOCK), b""):
            digest.update(chunk)
            count += len(chunk)
    require(Path(value).stat().st_size == count, "file size changed during stream hash")
    return count, digest.hexdigest()


def artifact(value):
    value = path(value)
    require(value.is_file(), "missing regular artifact: " + str(value))
    count, digest = hash_file(value)
    return {"path": str(value), "bytes": count, "sha256": digest}


def validate(record):
    require(type(record) is dict and set(record) == {"path", "bytes", "sha256"}, "invalid full artifact descriptor")
    require(artifact(record["path"]) == record, "input artifact digest/size differs: " + record["path"])
    return record


def sync_directory(value):
    descriptor = os.open(value, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def new_directory(value):
    value = path(value)
    value.mkdir(exist_ok=False)
    sync_directory(value.parent)


def new_stream(value):
    value = path(value)
    descriptor = os.open(value, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    sync_directory(value.parent)
    return os.fdopen(descriptor, "wb")


def sync(stream):
    stream.flush()
    os.fsync(stream.fileno())


def write_bytes(value, content):
    with new_stream(value) as destination:
        destination.write(content)
        sync(destination)
    sync_directory(Path(value).parent)
    return artifact(value)


def write_json(value, data):
    return write_bytes(value, literal(data))


def validate_release(record):
    """Validate root's explicit release and actual original terminal before scan."""
    validate(record)
    path(record["path"], OWN)
    release = strict_json(record["path"])
    require(release.get("status") == "ROOT_RELEASED_AFTER_ORIGINAL_AUDIT_TERMINAL", "original audit has not been released")
    require(release.get("new_archive_publication_authorized") is False, "new bytes must remain PROPOSED")
    root = path(release["cohort_root"])
    prep = path(release["excluded_preparation_directory"], root)
    require(prep == root / "derived-archive-preparation", "incorrect excluded preparation directory")
    output = path(release["output_directory"], prep)
    require(output != prep, "output must be a new preparation child directory")
    if root == G5:
        require(prep == OWN and release.get("synthetic_only") is False, "incorrect actual archive scope")
        require(release["old_authorized_raw_archive"]["bytes"] == OLD_BYTES
                and release["old_authorized_raw_archive"]["sha256"] == OLD_SHA, "approved old archive identity differs")
    else:
        require(root.is_relative_to(OWN / "synthetic-tests") and release.get("synthetic_only") is True,
                "only retained small synthetic cohorts can use alternate root")
    for role in ("audit_terminal", "collection_index", "final_audit", "old_authorized_raw_archive",
                 "old_raw_member_manifest", "old_tar_retention_review", "completed_scientific_review"):
        validate(release[role])
    path(release["completed_scientific_review"]["path"], root / "postcollection-scientific-review")
    require(path(release["audit_terminal"]["path"]) == root / "actual-scientific-audit-command/actual-command-terminal.json",
            "audit terminal path differs")
    require(path(release["final_audit"]["path"]) == root / "analysis/independent-numerical-audit.json",
            "final audit path differs")
    terminal = strict_json(release["audit_terminal"]["path"])
    require(type(terminal.get("actual_exit_code")) is int and terminal.get("ended_utc")
            and terminal.get("guarded_inputs_unchanged") is True and terminal.get("rerun_or_retry") is False,
            "original audit lacks a completed unchanged-input terminal")
    require(terminal.get("stdout") is not None and terminal.get("stderr") is not None, "missing original audit logs")
    validate(terminal["stdout"])
    validate(terminal["stderr"])
    collection = strict_json(release["collection_index"]["path"])
    require(collection.get("cohort_id") == release["cohort_id"] and len(collection.get("runs", [])) == 18,
            "new cohort identity/count differs")
    require(collection.get("worlds_attempted") == 18, "collection did not attempt the fixed 18")
    review = strict_json(release["old_tar_retention_review"]["path"])
    require(review.get("archive") == release["old_authorized_raw_archive"]
            and review.get("status") == "PASS_EXACT_APPROVED_OLD_TAR_BYTES_FROM_NEW_INDEPENDENT18_COHORT",
            "old raw retention reference is not the exact independently reviewed archive")
    require(review.get("cohort_id") == release["cohort_id"], "old retention proof references another cohort")
    helper = release.get("helper_artifacts")
    require(type(helper) is list and helper, "archive helper sources are not bound")
    for row in helper:
        validate(row)
    require(artifact(__file__) in helper, "running archive helper omitted from release binding")
    return release, collection


def file_names(root, *, skip_preparation=False):
    names = []
    for directory, children, files in os.walk(root):
        directory = Path(directory)
        for child in children + files:
            require(not (directory / child).is_symlink(), "symlink in evidence closure")
        if skip_preparation and directory == root:
            children[:] = [name for name in children if name != "derived-archive-preparation"]
        for filename in files:
            relative = (directory / filename).relative_to(root).as_posix()
            require(relative.isascii() and not relative.startswith("/") and ".." not in relative.split("/"),
                    "noncanonical/non-ASCII evidence relative path")
            names.append(relative)
    return sorted(names, key=lambda value: value.encode("ascii"))


def freeze_inputs(release, collection):
    root = path(release["cohort_root"])
    prep = path(release["excluded_preparation_directory"])
    require(REQUIRED <= set(file_names(root, skip_preparation=True)), "missing required complete derived/provenance artifact")
    raw_map = {}
    for row in collection["runs"]:
        raw = row["trace"]
        validate(raw)
        relative = path(raw["path"], root).relative_to(root).as_posix()
        require(relative == "collection/raw/" + row["run_label"] + ".jsonl.gz", "raw member identity differs")
        require(relative not in raw_map, "duplicate raw case")
        raw_map[relative] = {"cohort_id": release["cohort_id"], "run_label": row["run_label"],
                             "source": raw, "old_tar_member": Path(raw["path"]).name,
                             "old_authorized_raw_archive": release["old_authorized_raw_archive"],
                             "public_availability": "LOCAL_NOT_PUBLIC"}
    require(len(raw_map) == 18, "raw map does not cover all fixed cases")
    included, excluded = [], []
    old_tar = path(release["old_authorized_raw_archive"]["path"])
    for relative in file_names(root):
        value = root / relative
        record = artifact(value)
        reason = None
        if value.is_relative_to(prep):
            reason = "PREPARATION_TOOLING_AND_LATER_ARCHIVE_PUBLICATION_PROOFS_OUTSIDE_INPUT_FREEZE_TO_AVOID_CIRCULAR_HASH"
        elif value == old_tar:
            reason = "OLD_AUTHORIZED_EXACT_RAW_TAR_REFERENCED_LOCAL_NOT_PUBLIC_NOT_DUPLICATED"
        elif relative.startswith("collection/raw/"):
            require(relative in raw_map and record == raw_map[relative]["source"], "unexpected or changed raw evidence")
            reason = "WHOLE_RAW_GZIP_BOUND_TO_EXACT_OLD_AUTHORIZED_TAR_LOCAL_NOT_PUBLIC"
        elif relative.startswith("binaries/"):
            with value.open("rb") as stream:
                require(stream.read(4) == b"\x7fELF", "unknown non-native artifact in local-only binary scope")
            reason = "LOCAL_ONLY_NATIVE_BINARY_FULL_DESCRIPTOR_PRESERVED_IN_BINDINGS_AND_EXCLUSION_LEDGER"
        if reason:
            excluded.append({"original_relative_path": relative, "artifact": record, "reason": reason})
        else:
            included.append({"original_relative_path": relative, "artifact": record,
                             "origin": "COMPLETE_CLOSED_GENERATION5_ARTIFACT"})
    require({item["original_relative_path"] for item in included} >= REQUIRED, "required scientific/provenance file excluded")
    review_relative = path(release["completed_scientific_review"]["path"]).relative_to(root).as_posix()
    require(review_relative in {item["original_relative_path"] for item in included}, "final independent scientific review excluded")
    # Preserve the exact approved small member manifest alongside the new maps.
    included.append({"original_relative_path": "external-references/old-approved-raw-member-manifest.json",
                     "artifact": release["old_raw_member_manifest"], "origin": "EXACT_EXTERNAL_REFERENCE_LITERAL_FILE"})
    included.sort(key=lambda item: item["original_relative_path"].encode("ascii"))
    require(len({item["original_relative_path"] for item in included}) == len(included), "logical member path collision")
    require(len(included) < 1_000_000, "short fixed-width member index exhausted")
    for index, item in enumerate(included):
        item["archive_member"] = f"artifacts/{index:06d}"
        require(item["artifact"]["bytes"] < 8 ** 11, "PAX extension needed for member size; strict empty headers would fail")
    guarded = {item["original_relative_path"]: item["artifact"] for item in included
               if item["origin"] == "COMPLETE_CLOSED_GENERATION5_ARTIFACT"}
    guarded.update({item["original_relative_path"]: item["artifact"] for item in excluded
                    if not path(item["artifact"]["path"]).is_relative_to(prep)})
    return {"included": included, "excluded": excluded, "raw_member_map": [raw_map[key] for key in sorted(raw_map)],
            "guarded_nonpreparation_paths": guarded}


class Reader:
    def __init__(self, stream):
        self.stream, self.digest, self.count = stream, hashlib.sha256(), 0
    def read(self, size=-1):
        data = self.stream.read(size)
        self.digest.update(data)
        self.count += len(data)
        return data


class Writer:
    def __init__(self, stream, *, compare=False):
        self.stream, self.compare = stream, compare
        self.digest, self.count = hashlib.sha256(), 0
    def write(self, data):
        if self.compare:
            require(self.stream.read(len(data)) == data, "regenerated second TAR stream differs byte-for-byte")
        else:
            require(self.stream.write(data) == len(data), "short TAR write")
        self.digest.update(data)
        self.count += len(data)
        return len(data)
    def finish(self):
        if self.compare:
            require(self.stream.read(1) == b"", "second stream is shorter")


def tar_header(name, size):
    require(name.isascii() and len(name.encode("ascii")) <= 100, "member name requires nonempty PAX extension")
    header = tarfile.TarInfo(name)
    header.size, header.mode = size, 0o444
    header.uid = header.gid = header.mtime = 0
    header.uname = header.gname = ""
    header.type, header.pax_headers = tarfile.REGTYPE, {}
    return header


def tar_stream(destination, members):
    with tarfile.open(fileobj=destination, mode="w|", format=tarfile.PAX_FORMAT,
                      encoding="ascii", errors="strict", pax_headers={}) as archive:
        for item in members:
            validate(item["artifact"])
            with Path(item["artifact"]["path"]).open("rb") as source:
                reader = Reader(source)
                archive.addfile(tar_header(item["archive_member"], item["artifact"]["bytes"]), reader)
                require(reader.count == item["artifact"]["bytes"]
                        and reader.digest.hexdigest() == item["artifact"]["sha256"], "consumed member source differs")
                require(source.read(1) == b"", "member source grew")
            validate(item["artifact"])


def verify_saved_tar(record, members):
    validate(record)
    observed = []
    with tarfile.open(record["path"], mode="r:", encoding="ascii", errors="strict") as archive:
        for item in archive:
            require(item.isfile() and item.type == tarfile.REGTYPE and item.pax_headers == {}, "noncanonical PAX member type/headers")
            require(item.mode == 0o444 and item.uid == item.gid == item.mtime == 0
                    and item.uname == item.gname == "", "nondeterministic member metadata")
            reader = Reader(archive.extractfile(item))
            while reader.read(BLOCK):
                pass
            observed.append({"name": item.name, "bytes": reader.count, "sha256": reader.digest.hexdigest()})
    expected = [{"name": item["archive_member"], "bytes": item["artifact"]["bytes"],
                 "sha256": item["artifact"]["sha256"]} for item in members]
    require(observed == expected, "saved TAR member count/order/full SHA/size differs")
    require([item["name"] for item in observed] == sorted(item["name"] for item in observed), "archive names not sorted ASCII")
    return observed


def recheck_inputs(release, frozen):
    root, prep = path(release["cohort_root"]), path(release["excluded_preparation_directory"])
    require(file_names(root, skip_preparation=True) == sorted(frozen["guarded_nonpreparation_paths"]),
            "closed G5 input file set changed after freeze")
    for item in frozen["included"] + frozen["excluded"]:
        validate(item["artifact"])
    require(prep == root / "derived-archive-preparation", "preparation scope changed")


def prepare(release_record):
    release, collection = validate_release(release_record)
    frozen = freeze_inputs(release, collection)
    output = path(release["output_directory"])
    new_directory(output)
    write_json(output / "archive-claim.json", {
        "status": "DERIVED_ARCHIVE_EXCLUSIVE_CLAIM", "started_utc": now(),
        "release": release_record, "cohort_id": release["cohort_id"], "worlds": 0,
        "publication": "PROPOSED_ONLY_NEW_BYTES_NOT_AUTHORIZED",
    })
    try:
        freeze_record = write_json(output / "input-freeze.json", frozen)
        manifest = {
            "schema_version": 1, "kind": "GENERATION5_COMPLETE_DERIVED_EVIDENCE_LITERAL_MEMBER_MAP",
            "cohort_id": release["cohort_id"], "release": release_record,
            "input_freeze": freeze_record, "members": frozen["included"], "exclusions": frozen["excluded"],
            "whole_scientific_detail_paths": list(DETAIL_PATHS), "whole_scientific_detail_count": 6,
            "raw_member_map": frozen["raw_member_map"],
            "old_authorized_raw_archive": release["old_authorized_raw_archive"],
            "raw_archive_public_availability": "LOCAL_NOT_PUBLIC",
            "historical_scientific_status": "FAILED_UNQUALIFIED_UNCHANGED",
            "new_scientific_result_interpretation_by_archive_helper": None,
            "numeric_payload_parsing_selection_splitting_truncation": False,
            "layout": "Whole files enumerated by original relative ASCII path; artifacts/NNNNNN maps one-to-one to the full original bytes",
            "freeze_boundary": "All completed G5 provenance before archive generation; preparation directory and later publication verification are outside this freeze to avoid circular hashes",
        }
        manifest_record = write_bytes(output / "literal-member-manifest.json", literal(manifest))
        members = list(frozen["included"]) + [{"archive_member": "literal-member-manifest.json", "artifact": manifest_record}]
        tar_path = output / "rne-contact-motor-stage-generation5-complete-derived-evidence.tar"
        with new_stream(tar_path) as saved:
            first = Writer(saved)
            tar_stream(first, members)
            sync(saved)
        sync_directory(output)
        tar_record = artifact(tar_path)
        require(tar_record["bytes"] == first.count and tar_record["sha256"] == first.digest.hexdigest(), "first TAR stream differs from saved file")
        verified_members = verify_saved_tar(tar_record, members)
        with tar_path.open("rb") as original:
            second = Writer(original, compare=True)
            tar_stream(second, members)
            second.finish()
        require(second.count == first.count and second.digest.hexdigest() == first.digest.hexdigest(), "second stream SHA/size differs")
        recheck_inputs(release, frozen)
        validate_release(release_record)
        validate(tar_record)
        verification = write_json(output / "archive-verification.json", {
            "status": "PASS_COMPLETE_LITERAL_DERIVED_ARCHIVE_INTEGRITY",
            "archive": tar_record, "manifest": manifest_record, "input_freeze": freeze_record,
            "members": verified_members, "payload_file_count": len(frozen["included"]),
            "tar_member_count": len(members), "whole_details": 6,
            "first_stream": {"bytes": first.count, "sha256": first.digest.hexdigest()},
            "second_stream": {"bytes": second.count, "sha256": second.digest.hexdigest()},
            "second_stream_literal_byte_comparison": True,
            "all_original_input_bytes_and_nonpreparation_path_set_unchanged": True,
            "native_worlds_builds_numeric_parsing_network_publication": 0,
        })
        pointer = write_json(output / "proposed-derived-archive-pointer.json", {
            "kind": "LOCAL_PROPOSED_GENERATION5_DERIVED_EVIDENCE_POINTER", "schema_version": 1,
            "status": "PROPOSED_NEW_DERIVED_BYTES_FRESH_USER_AUTHORIZATION_PENDING",
            "cohort_id": release["cohort_id"], "archive": tar_record, "manifest": manifest_record,
            "verification": verification, "url": None, "tag": None, "publication_authorized": False,
            "old_raw_archive_authorization_does_not_authorize_these_new_bytes": True,
            "historical_scientific_status": "FAILED_UNQUALIFIED_UNCHANGED",
        })
        write_json(output / "archive-terminal.json", {
            "exit_code": 0, "ended_utc": now(), "release": release_record,
            "pointer": pointer, "originals_preserved": True, "publication": "PROPOSED_ONLY",
        })
        return pointer
    except Exception as error:
        failed = {"exit_code": 1, "ended_utc": now(), "error": f"{type(error).__name__}: {error}",
                  "originals_and_partial_tar_preserved": True,
                  "proposed_pointer_exists": (output / "proposed-derived-archive-pointer.json").exists(),
                  "publication": "NOT_PUBLISHED", "worlds": 0}
        write_json(output / "archive-failure.json", failed)
        if not (output / "archive-terminal.json").exists():
            write_json(output / "archive-terminal.json", failed)
        raise


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--release", required=True)
    parser.add_argument("--release-sha256", required=True)
    args = parser.parse_args()
    record = artifact(args.release)
    require(record["sha256"] == args.release_sha256, "root release SHA differs")
    print(json.dumps(prepare(record), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
