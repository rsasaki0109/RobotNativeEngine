#!/usr/bin/env python3
"""Outside-only command/exit receipt wrapper; no numerical or physics logic."""

import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).parent


def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def record(path):
    path = Path(path).resolve()
    digest = hashlib.sha256()
    count = 0
    with path.open("rb") as stream:
        while block := stream.read(1 << 20):
            digest.update(block); count += len(block)
    return {"path": str(path), "sha256": digest.hexdigest(), "bytes": count}


def main():
    mode = sys.argv[1]
    protocol = str(HERE / "frozen-protocol.json")
    common = ["--protocol", protocol, "--protocol-sha256", "8275933754f12274ffd7a7abe4f6540daf8ea1c3e4a29aa495bfce87691c6d4a"]
    if mode == "coupled":
        output = HERE / "actual-independent-coupled-audit.json"
        argv = ["python3", "-B", str(HERE / "audit_coupled_stages.py")] + common + [
            "--collection", str(HERE / "actual-coupled-collection-index.json"),
            "--collection-sha256", "44ab1050c9386703f65cf765e56d5fa3d09c5b78beba064b6e21f7bb0db65880",
            "--output", str(output), "--details-directory", "/tmp/rne-contact-stage-generation4/audited-details"]
    elif mode == "scalar":
        output = HERE / "actual-independent-current-scalar-audit.json"
        argv = ["python3", "-B", str(HERE / "audit_current_scalar_report.py")] + common + [
            "--binding", str(HERE / "scalar-source-binary-binding.json"), "--binding-sha256",
            "dcb1cb64d326bafc30c4a9aa862c19854935c7467b7e254e227e4a1ae3c3a056",
            "--collection", str(HERE / "actual-scalar72-collection.json"), "--collection-sha256",
            "e8beae74681d6d43f72141cba438d03cee5711418e6024ff7dafc4cc779fc308",
            "--report", "/tmp/rne-contact-stage-generation4/current-scalar72.json", "--report-sha256",
            "db0dee42a3c9067b38241112d5ee402341c0729466d7fc22f12f31504000db76", "--output", str(output)]
    else:
        raise ValueError("unknown audit mode")
    stem = HERE / ("actual-independent-" + mode + "-command")
    receipt_path = Path(str(stem) + ".json")
    if receipt_path.exists() or output.exists():
        raise ValueError("retained once-only audit already exists")
    sources = [HERE / name for name in ["audit_coupled_stages.py", "audit_current_scalar_report.py", "stage_analysis.py",
                                       "stage_numeric.py", "stage_schema.py", "frozen-protocol.json"]]
    before = [record(path) for path in sources]
    stdout, stderr = Path(str(stem) + ".stdout"), Path(str(stem) + ".stderr")
    started = now()
    with stdout.open("xb") as out, stderr.open("xb") as err:
        result = subprocess.run(argv, cwd="/workspace/RobotNativeEngine", stdout=out, stderr=err)
    ended = now()
    after = [record(path) for path in sources]
    receipt = {"status": "ACTUAL_ONCE_ONLY_FROZEN_INDEPENDENT_READER_TERMINAL", "mode": mode,
               "argv": argv, "started_utc": started, "ended_utc": ended, "actual_exit_code": result.returncode,
               "stdout": record(stdout), "stderr": record(stderr), "output": record(output) if output.exists() else None,
               "reader_protocol_bytes_unchanged": before == after, "source_before": before, "source_after": after,
               "native_worlds_builds_or_checkout_mutations_by_reader": 0}
    with receipt_path.open("x") as stream:
        json.dump(receipt, stream, indent=2, sort_keys=True); stream.write("\n")
    print(json.dumps({"actual_exit_code": result.returncode, "receipt": record(receipt_path)}, sort_keys=True))
    return result.returncode


if __name__ == "__main__":
    sys.exit(main())
