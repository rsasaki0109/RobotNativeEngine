"""Capture the single actual helper command, its original exit and durable logs."""

import json
from pathlib import Path
import subprocess
import sys
import time

import prepare_derived_archive as archive


def main():
    root = archive.OWN
    release = archive.artifact(root / "actual-pack-release.json")
    archive.validate_release(release)
    folder = root / "actual-pack-command"
    archive.new_directory(folder)
    command = [sys.executable, "-B", str(root / "prepare_derived_archive.py"),
               "--release", release["path"], "--release-sha256", release["sha256"]]
    inputs = [release, archive.artifact(root / "prepare_derived_archive.py"), archive.artifact(__file__)]
    started = time.monotonic()
    claim = archive.write_json(folder / "command-claim.json", {
        "argv": command, "cwd": str(root), "started_utc": archive.now(),
        "inputs_before": inputs, "no_retry": True, "worlds_builds_network": 0,
    })
    with archive.new_stream(folder / "stdout.log") as stdout, archive.new_stream(folder / "stderr.log") as stderr:
        process = subprocess.Popen(command, cwd=root, stdout=stdout, stderr=stderr)
        while process.poll() is None:
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                print(json.dumps({"status": "ORIGINAL_DERIVED_ARCHIVE_HELPER_RUNNING", "pid": process.pid,
                                  "elapsed_s": time.monotonic() - started}), flush=True)
        archive.sync(stdout)
        archive.sync(stderr)
    after = [archive.artifact(record["path"]) for record in inputs]
    terminal = archive.write_json(folder / "actual-command-terminal.json", {
        "argv": command, "claim": claim, "ended_utc": archive.now(),
        "actual_exit_code": process.returncode, "runtime_s": time.monotonic() - started,
        "inputs_before": inputs, "inputs_after": after, "guarded_inputs_unchanged": inputs == after,
        "stdout": archive.artifact(folder / "stdout.log"), "stderr": archive.artifact(folder / "stderr.log"),
        "rerun_or_retry": False, "worlds_builds_numeric_parsing_network_publication": 0,
    })
    print(json.dumps(terminal, sort_keys=True), flush=True)
    return process.returncode if process.returncode != 0 else (0 if inputs == after else 1)


if __name__ == "__main__":
    raise SystemExit(main())
