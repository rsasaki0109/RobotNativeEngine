"""Once-only synthetic test receipt; never reads repository reports or raw data."""

import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parent
FILES = ["compare_scalar_source_transition.py", "test_compare_scalar_source_transition.py"]


def inputs():
    return {name: {"sha256": hashlib.sha256((ROOT / name).read_bytes()).hexdigest(),
                   "size_bytes": (ROOT / name).stat().st_size} for name in FILES}


def save(path, value):
    raw = (json.dumps(value, indent=2, sort_keys=True) + "\n").encode()
    with open(path, "xb") as stream:
        stream.write(raw)
        stream.flush()
        os.fsync(stream.fileno())


def main():
    destination = ROOT / sys.argv[1]
    destination.mkdir(exist_ok=False)
    started = datetime.datetime.now(datetime.timezone.utc).isoformat()
    before = inputs()
    save(destination / "inputs-before.json", before)
    argv = [sys.executable, "-B", "-m", "unittest", "test_compare_scalar_source_transition"]
    with open(destination / "command.log", "xb") as stream:
        result = subprocess.run(argv, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, check=False)
        stream.flush()
        os.fsync(stream.fileno())
    after = inputs()
    save(destination / "inputs-after.json", after)
    log = (destination / "command.log").read_bytes()
    terminal = {
        "kind": "rne_synthetic_only_comparator_test_receipt", "schema_version": 1,
        "argv": argv, "cwd": str(ROOT), "started_at": started,
        "finished_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "original_exit_code": result.returncode, "source_inputs_unchanged": before == after,
        "log_sha256": hashlib.sha256(log).hexdigest(), "log_size_bytes": len(log),
        "synthetic_reports_only": True, "actual_report_reads": 0, "worlds_started": 0,
        "builds_started": 0, "checkout_edits": 0,
        "status": "PASS" if result.returncode == 0 and before == after else "FAIL_RETAINED",
    }
    save(destination / "terminal.json", terminal)
    print(json.dumps(terminal, sort_keys=True))
    return result.returncode if result.returncode else (0 if before == after else 1)


if __name__ == "__main__":
    raise SystemExit(main())
