#!/usr/bin/env python3
"""Once-only outside command receipt wrapper; no numerical/physics logic."""

import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).parent
ORIGINAL = HERE.parent


def record(path):
    path = Path(path).resolve()
    digest = hashlib.sha256()
    count = 0
    with path.open("rb") as stream:
        while block := stream.read(1 << 20):
            digest.update(block); count += len(block)
    return {"path": str(path), "sha256": digest.hexdigest(), "bytes": count}


def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def main():
    output = HERE / "actual-v2-independent-coupled-audit.json"
    receipt_path = HERE / "actual-v2-reader-command.json"
    stdout, stderr = HERE / "actual-v2-reader-command.stdout", HERE / "actual-v2-reader-command.stderr"
    if output.exists() or receipt_path.exists():
        raise ValueError("once-only repaired audit already retained")
    argv = ["python3", "-B", str(HERE / "audit_coupled_stages.py"),
            "--protocol", str(ORIGINAL / "frozen-protocol.json"), "--protocol-sha256",
            "8275933754f12274ffd7a7abe4f6540daf8ea1c3e4a29aa495bfce87691c6d4a",
            "--collection", str(ORIGINAL / "actual-coupled-collection-index.json"), "--collection-sha256",
            "44ab1050c9386703f65cf765e56d5fa3d09c5b78beba064b6e21f7bb0db65880",
            "--repair-manifest", str(HERE / "frozen-repair-manifest.json"), "--repair-manifest-sha256",
            "3eb73c2cc2d8a924492b1ccef1c066e79ad54e2b1d91d4c0816bab8d6a174901",
            "--output", str(output), "--details-directory", "/tmp/rne-contact-stage-generation4/audited-details-native-frame-v2"]
    source_paths = [HERE / name for name in ["audit_coupled_stages.py", "stage_analysis.py", "stage_numeric.py", "stage_schema.py", "frozen-repair-manifest.json"]]
    source_paths += [ORIGINAL / name for name in ["frozen-protocol.json", "actual-coupled-collection-index.json",
                     "actual-independent-coupled-audit.json", "actual-independent-current-scalar-audit.json"]]
    source_paths += [ORIGINAL / "postcollection-handle-schema-repair-v1" / name for name in ["frozen-repair-manifest.json", "actual-repaired-independent-coupled-audit.json", "actual-repaired-reader-command.json"]]
    before = [record(path) for path in source_paths]
    started = now()
    with stdout.open("xb") as out, stderr.open("xb") as err:
        result = subprocess.run(argv, cwd="/workspace/RobotNativeEngine", stdout=out, stderr=err)
    ended = now()
    after = [record(path) for path in source_paths]
    receipt = {"status": "ACTUAL_ONCE_ONLY_POSTCOLLECTION_NATIVE_FRAME_V2_READER_TERMINAL", "argv": argv,
               "started_utc": started, "ended_utc": ended, "actual_exit_code": result.returncode,
               "stdout": record(stdout), "stderr": record(stderr), "output": record(output) if output.exists() else None,
               "original_and_repaired_reader_protocol_collection_audit_bytes_unchanged": before == after,
               "source_before": before, "source_after": after,
               "native_worlds_builds_checkout_mutations_or_original_reader_reruns": 0,
               "repaired_reader_registered_after_collection": True}
    with receipt_path.open("x") as stream:
        json.dump(receipt, stream, indent=2, sort_keys=True); stream.write("\n")
    print(json.dumps({"actual_exit_code": result.returncode, "receipt": record(receipt_path)}, sort_keys=True))
    return result.returncode


if __name__ == "__main__":
    sys.exit(main())
