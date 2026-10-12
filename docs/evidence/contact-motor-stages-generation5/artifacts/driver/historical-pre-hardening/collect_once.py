#!/usr/bin/env python3
"""Collect one fixed generation-5 cohort, without resuming or selecting outcomes."""

import argparse
import copy
import os
from pathlib import Path
import signal
import subprocess
import time

from durable_common import (
    artifact, fixed_cells, fsync_directory, INDEPENDENCE, mkdir_new, now, open_new,
    persistent_path, source_snapshot, strict_json, sync_stream, validate_artifact,
    validate_spec, write_new_json,
)


def index_template(spec_record, spec):
    rows = []
    for cell in fixed_cells():
        rows.append(dict(cell, attempted=False, exit_code=None, started_utc=None,
                         ended_utc=None, argv=None, trace=None, stdout=None, stderr=None,
                         source_guard_before=None, source_guard_after=None, error=None))
    return {
        "schema_version": 2, "generation": 5, "cohort_id": spec["cohort_id"],
        "independence": INDEPENDENCE, "status": "GENERATION5_COLLECTION_CLAIMED",
        "frozen_spec": spec_record, "protocol_sha256": spec["protocol"]["sha256"],
        "root_driver": artifact(__file__), "started_utc": now(), "ended_utc": None,
        "declared_authoritative_coupled_worlds": 18,
        "declared_completed_outer_steps_per_world": 1250,
        "worlds_launch_reserved": 0, "worlds_attempted": 0, "worlds_completed": 0,
        "source_binary_bindings": spec["bindings"], "case_receipts": [], "runs": rows,
        "error": None,
    }


def collect(spec_record, *, popen=subprocess.Popen, stop_requested=lambda: False):
    """Reserve the whole cohort exactly once. A partial collection cannot restart.

    ``popen`` and ``stop_requested`` are injectable only for synthetic unit tests;
    the CLI always launches the pinned native binary with subprocess.Popen.
    """
    spec, bindings, expected = validate_spec(spec_record)
    root = persistent_path(spec["cohort_root"])
    mkdir_new(root)
    index = index_template(spec_record, spec)
    # This durable claim precedes every subprocess. Reopening this root is never
    # allowed, even if a crash left no terminal receipt or no launched process.
    write_new_json(root / "cohort-claim.json", index)
    for name in ("cases", "raw", "index-snapshots"):
        mkdir_new(root / name)
    write_new_json(root / "collection-started.json", index)
    write_new_json(root / "index-snapshots/index-000.json", index)
    fatal = None
    try:
        for ordinal, row in enumerate(index["runs"]):
            if stop_requested():
                fatal = "operator interruption: no further cases launched"
                break
            # Revalidate every reader/protocol/binary/file binding before launch.
            validate_spec(spec_record)
            folder = root / "cases" / f"{ordinal:02d}-{row['run_label']}"
            mkdir_new(folder)
            before = source_snapshot(spec["repository_root"], expected)
            row["source_guard_before"] = write_new_json(folder / "source-before.json", before)
            if before != expected:
                fatal = "source inputs changed before launch"
                write_new_json(folder / "prelaunch-rejection.json", {"case_index": ordinal, "error": fatal})
                break
            trace = root / "raw" / (row["run_label"] + ".jsonl.gz")
            if trace.exists():
                raise FileExistsError("refusing an existing raw case output")
            stdout, stderr = folder / "stdout.log", folder / "stderr.log"
            binary = bindings[row["build"]]["binary"]
            validate_artifact(binary)
            command = [binary["path"], "--mass-kg", str(row["mass_kg"]), "--observer",
                       "off" if row["mode"] == "off" else "on", "--label", row["run_label"],
                       "--output", str(trace)]
            row.update(started_utc=now(), argv=command)
            claim = dict(row, case_index=ordinal, cohort_id=spec["cohort_id"],
                         launch_reserved=True, original_exit_code=None,
                         binary=binary, frozen_spec=spec_record,
                         environment_policy="Inherited process environment; no driver overrides",
                         retry_or_resume_permitted=False)
            with open_new(stdout) as out, open_new(stderr) as err:
                write_new_json(folder / "launch-claim.json", claim)
                index["worlds_launch_reserved"] += 1
                write_new_json(root / "index-snapshots" / f"reserved-{ordinal:02d}.json", index)
                process = None
                launch_error = None
                try:
                    process = popen(command, cwd=spec["repository_root"], stdout=out, stderr=err)
                    row["attempted"] = True
                    index["worlds_attempted"] += 1
                    write_new_json(folder / "process-started.json", {
                        "case_index": ordinal, "pid": process.pid, "started_utc": now(),
                        "argv": command, "launch_claim": artifact(folder / "launch-claim.json"),
                    })
                    while process.poll() is None:
                        try:
                            process.wait(timeout=30)
                        except subprocess.TimeoutExpired:
                            print({"case_index": ordinal, "label": row["run_label"], "pid": process.pid,
                                   "status": "RUNNING_ORIGINAL_PROCESS"}, flush=True)
                    row["exit_code"] = process.returncode
                except Exception as error:
                    # A failed spawn or receipt write is retained, never retried.
                    launch_error = f"{type(error).__name__}: {error}"
                    if process is not None:
                        # Never invent an exit or kill the running original world.
                        while process.poll() is None:
                            try:
                                process.wait(timeout=30)
                            except subprocess.TimeoutExpired:
                                pass
                        row["exit_code"] = process.returncode
                finally:
                    sync_stream(out)
                    sync_stream(err)
            row["ended_utc"] = now()
            row["stdout"], row["stderr"] = artifact(stdout), artifact(stderr)
            if trace.is_file():
                # Flush the collector-created exclusive output to durable storage.
                with trace.open("rb") as raw:
                    os.fsync(raw.fileno())
                fsync_directory(trace.parent)
                row["trace"] = artifact(trace)
            after_error = None
            try:
                after = source_snapshot(spec["repository_root"], expected)
                row["source_guard_after"] = write_new_json(folder / "source-after.json", after)
                if after != before:
                    after_error = "source inputs changed during case"
                validate_spec(spec_record)
            except Exception as error:
                after_error = f"{type(error).__name__}: {error}"
            errors = [value for value in (launch_error, after_error,
                      None if row["exit_code"] == 0 else "original process exit is nonzero or unavailable",
                      None if row["trace"] is not None else "raw output unavailable") if value]
            row["error"] = "; ".join(errors) or None
            receipt = {
                "schema_version": 1, "cohort_id": spec["cohort_id"], "case_index": ordinal,
                "original_exit_code": row["exit_code"], "launch_reserved": True,
                "process_started": row["attempted"], "binary": binary,
                "raw_output_exclusive_create_contract": "Unchanged collector142 OpenOptions::create_new(true)",
                "source_inputs_unchanged": after_error is None,
                "launch_error": launch_error, "row": copy.deepcopy(row),
                "retry_or_resume_permitted": False,
            }
            index["case_receipts"].append(write_new_json(folder / "case-terminal.json", receipt))
            if row["exit_code"] == 0:
                index["worlds_completed"] += 1
            write_new_json(root / "index-snapshots" / f"index-{ordinal + 1:03d}.json", index)
            print({"case_index": ordinal, "label": row["run_label"],
                   "original_exit_code": row["exit_code"], "trace": row["trace"]}, flush=True)
            if launch_error or after_error:
                fatal = row["error"]
                break
            # A nonzero process exit or missing raw data does not select away
            # the next fixed case, provided provenance/source inputs still bind.
    except Exception as error:
        fatal = f"{type(error).__name__}: {error}"
    index["ended_utc"] = now()
    complete = index["worlds_attempted"] == 18 and len(index["case_receipts"]) == 18
    errors = [row["error"] for row in index["runs"] if row["error"]]
    index["error"] = fatal or ("one or more original case outcomes failed" if errors else None)
    index["status"] = ("GENERATION5_18_WORLD_PROCESSES_COMPLETED" if complete and not index["error"]
                       else "GENERATION5_PARTIAL_OR_FAILED_COHORT_RETAINED_NO_RERUN")
    final = write_new_json(root / "collection-index.json", index)
    write_new_json(root / "collection-terminal.json", {
        "schema_version": 1, "status": index["status"], "cohort_id": spec["cohort_id"],
        "independence": INDEPENDENCE, "index": final, "frozen_spec": spec_record,
        "worlds_launch_reserved": index["worlds_launch_reserved"],
        "worlds_attempted": index["worlds_attempted"], "worlds_completed": index["worlds_completed"],
        "driver_exit_code": 0 if complete and not index["error"] else 1,
        "case_exit_codes_preserved": True, "retry_or_resume_permitted": False,
    })
    return 0 if complete and not index["error"] else 1


def inspect(root):
    """Read receipts only; this function never launches or repairs a case."""
    root = persistent_path(root)
    claim = strict_json(root / "cohort-claim.json")
    records = []
    for folder in sorted((root / "cases").glob("*")):
        launch, terminal = folder / "launch-claim.json", folder / "case-terminal.json"
        records.append({"case_directory": str(folder), "launch_reserved": launch.exists(),
                        "terminal": artifact(terminal) if terminal.is_file() else None,
                        "state": "TERMINAL_RECEIPT" if terminal.is_file() else
                        "CLAIMED_OR_PRELAUNCH_PARTIAL_NEVER_RERUN"})
    return {"cohort_id": claim["cohort_id"], "independence": INDEPENDENCE,
            "cohort_claim": artifact(root / "cohort-claim.json"), "cases": records,
            "final_terminal": artifact(root / "collection-terminal.json")
            if (root / "collection-terminal.json").is_file() else None,
            "native_worlds_launched_by_inspection": 0, "retry_or_resume_permitted": False}


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="action", required=True)
    run = subparsers.add_parser("run")
    run.add_argument("--spec", required=True)
    run.add_argument("--spec-sha256", required=True)
    review = subparsers.add_parser("inspect")
    review.add_argument("--cohort-root", required=True)
    args = parser.parse_args()
    if args.action == "inspect":
        import json
        print(json.dumps(inspect(args.cohort_root), sort_keys=True))
        return 0
    record = artifact(args.spec)
    if record["sha256"] != args.spec_sha256:
        raise ValueError("frozen driver spec SHA mismatch")
    stop = []
    def request_stop(signum, _frame):
        stop.append(signum)
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, request_stop)
    return collect(record, stop_requested=lambda: bool(stop))


if __name__ == "__main__":
    raise SystemExit(main())
