"""Retain allowed preparation commands and immutable dependency input guards."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
PREPARATION = ROOT.parent / "next-motor-construction-preparation"
VENDOR = PREPARATION / "source-copy/third_party/rapier3d"
EXPECTED = "babaf825819242859b87ad87f48b708e78d835ff0aaa632590dbb63a34a995d4"


def sha(data):
    return hashlib.sha256(data).hexdigest()


def dependency_snapshot():
    return [
        {"path": str(path.relative_to(PREPARATION / "source-copy")),
         "sha256": sha(path.read_bytes()), "bytes": path.stat().st_size}
        for path in sorted(VENDOR.rglob("*")) if path.is_file()
    ]


def own_snapshot():
    paths = [ROOT / "Cargo.toml", ROOT / "Cargo.lock", *sorted((ROOT / "src").glob("*.rs"))]
    return [{"path": str(path.relative_to(ROOT)), "sha256": sha(path.read_bytes()),
             "bytes": path.stat().st_size} for path in paths]


def canonical_sha(value):
    return sha(json.dumps(value, sort_keys=True, separators=(",", ":")).encode())


label, *command = sys.argv[1:]
if not label or not command or any(c not in "abcdefghijklmnopqrstuvwxyz0123456789-_" for c in label):
    raise SystemExit("invalid explicit command label")
destination = ROOT / "checks" / label
destination.mkdir()
before = dependency_snapshot()
before_own = own_snapshot()
before_hash = canonical_sha(before)
if before_hash != EXPECTED:
    failure = {"status": "REJECTED_BEFORE_COMMAND", "expected_dependency_inventory": EXPECTED,
               "actual_dependency_inventory": before_hash, "command": command, "worlds": 0}
    (destination / "receipt.json").write_text(json.dumps(failure, indent=2) + "\n")
    raise SystemExit("final producer inventory mismatch before command; retained")
started = datetime.now(timezone.utc).isoformat()
with (destination / "stdout.log").open("xb") as stdout, (destination / "stderr.log").open("xb") as stderr:
    result = subprocess.run(command, cwd=ROOT, stdout=stdout, stderr=stderr)
ended = datetime.now(timezone.utc).isoformat()
after = dependency_snapshot()
after_own = own_snapshot()
receipt = {
    "schema_version": 1, "label": label, "argv": command, "cwd": str(ROOT),
    "started_utc": started, "ended_utc": ended, "actual_exit_code": result.returncode,
    "activation_script": "/workspace/.rne-onboarding/env.sh",
    "environment": {key: os.environ.get(key) for key in
                    ["CARGO_HOME", "RUSTUP_HOME", "CARGO_BUILD_JOBS", "CARGO_TARGET_DIR",
                     "CARGO_INCREMENTAL", "CARGO_PROFILE_DEV_DEBUG", "CARGO_PROFILE_TEST_DEBUG",
                     "RUSTUP_TOOLCHAIN", "RUSTDOCFLAGS", "RUSTFLAGS"]},
    "dependency_inventory_sha256_before": before_hash,
    "dependency_inventory_sha256_after": canonical_sha(after),
    "dependency_source_unchanged": before == after,
    "owned_source_unchanged": before_own == after_own,
    "owned_source_before": before_own, "owned_source_after": after_own,
    "dependency_inventory": after,
    "native_worlds_executed": 0,
    "scope": "Compilation/docs/scoped format check or only owned world-free synthetic tests; no collector run",
    "logs": {name: {"path": str(destination / name),
                    "sha256": sha((destination / name).read_bytes()),
                    "bytes": (destination / name).stat().st_size}
             for name in ["stdout.log", "stderr.log"]},
}
path = destination / "receipt.json"
path.write_text(json.dumps(receipt, indent=2) + "\n")
print(json.dumps({"label": label, "actual_exit_code": result.returncode,
                  "dependency_unchanged": before == after, "owned_unchanged": before_own == after_own,
                  "receipt_path": str(path), "receipt_sha256": sha(path.read_bytes())}), flush=True)
if result.returncode:
    print((destination / "stderr.log").read_text(errors="replace")[-12000:])
if before != after or before_own != after_own:
    raise SystemExit("inputs changed during validation; actual result retained but not qualified")
raise SystemExit(result.returncode)
