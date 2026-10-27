#!/usr/bin/env python3
"""Exercise the hardware-free evaluation CLI and retain evidence for CI debugging."""
import copy
import hashlib
import html
import math
import os
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CARGO = ["cargo", "run", "--locked", "-p", "robot_workbench", "--no-default-features", "--"]


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def run(output, label, *args):
    # File-backed logs also retain partial output if the process times out.
    with (output / f"{label}.stdout.log").open("w") as stdout, (output / f"{label}.stderr.log").open("w") as stderr:
        result = subprocess.run(CARGO + list(map(str, args)), cwd=ROOT,
                                stdout=stdout, stderr=stderr, timeout=600, check=False)
    return result.returncode


def read_stdout(output, label):
    return json.loads((output / f"{label}.stdout.log").read_text())


def fingerprints(directory):
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in directory.iterdir() if p.is_file()}


def check(output):
    evidence = output / "evaluation"
    require(run(output, "evaluate", "--evaluate-pick-place", evidence) == 0,
            "evaluation failed; inspect evaluate.stderr.log and evaluation/report.json")
    report_path = evidence / "report.json"
    report = json.loads(report_path.read_text())
    require(report["passed"] is True, "catalog expectations did not pass")
    require((report["completed_cases"], report["executed_cases"], report["rejected_cases"], report["error_cases"])
            == (3, 4, 2, 0), "unexpected catalog outcome counts")
    expected = {"nominal": "completed", "diagonal": "completed", "reverse": "completed",
                "missed-block": "failed", "unequal-height": "rejected", "out-of-range": "rejected"}
    require(len(report["cases"]) == 6, "catalog case count changed")
    require({c["name"]: c["outcome"] for c in report["cases"]} == expected, "catalog outcomes changed")
    for case in report["cases"]:
        require(case["expectation_met"] is True, f"{case['name']}: expectation failed")
        executed = case["outcome"] != "rejected"
        require(case["replay_verified"] is (True if executed else None), "incorrect replay evidence")
        if executed:
            require(case["recording"] == f"{case['name']}.recording.json", "unexpected recording path")
            require((evidence / case["recording"]).is_file(), "recording missing")

    before = fingerprints(evidence)
    require(run(output, "unchanged", "--compare-evaluations", report_path, report_path) == 0,
            "identical report comparison failed")
    require(read_stdout(output, "unchanged")["passed"] is True, "identical report did not pass")

    # Keep passed=true deliberately: the comparator must inspect per-case evidence.
    degraded = copy.deepcopy(report)
    for case in degraded["cases"]:
        if case["name"] == "nominal":
            case["placement"]["placement_error_x_z_m"] = [0.1, 0.0]
        if case["name"] == "diagonal":
            case["replay_verified"] = False
    degraded_path = output / "degraded.json"
    degraded_path.write_text(json.dumps(degraded, indent=2))
    require(run(output, "degraded", "--compare-evaluations", report_path, degraded_path) == 1,
            "regression was accepted")
    comparison = read_stdout(output, "degraded")
    require(comparison["passed"] is False and len(comparison["regressions"]) == 2,
            "both placement and replay regressions must be reported")
    require(any("nominal: placement error" in r for r in comparison["regressions"])
            and any("diagonal:" in r for r in comparison["regressions"]), "wrong regressions reported")

    incomplete = copy.deepcopy(report)
    incomplete["cases"].pop()
    incomplete_path = output / "incomplete.json"
    incomplete_path.write_text(json.dumps(incomplete, indent=2))
    require(run(output, "incomplete", "--compare-evaluations", report_path, incomplete_path) == 1,
            "incomplete catalog was accepted")
    require("six unique cases" in (output / "incomplete.stderr.log").read_text(),
            "incomplete catalog failed for an unrelated reason")
    require(run(output, "overwrite", "--evaluate-pick-place", evidence) == 1,
            "existing evidence directory was reused")
    require(fingerprints(evidence) == before, "evaluation evidence was modified")
    return {"passed": True, "catalog_cases": 6, "verified_recordings": 4,
            "checks": ["catalog", "unchanged", "degraded", "incomplete", "no-overwrite"]}



def markdown_summary(output, summary):
    """Render actual evaluation evidence separately from the CI contract verdict."""
    def cell(value):
        return html.escape(str(value)).replace("|", "&#124;").replace("\n", " ").replace("\r", " ")

    lines = ["## LeKiwi + SO-101 evaluation", "",
             f"CI gate: **{'PASS' if summary['passed'] else 'FAIL'}**", "",
             "Hardware-free simulation; expected task failures are not successful placements.", ""]
    if not summary["passed"]:
        lines += [f"Failure: {cell(summary.get('error', 'unknown'))}", ""]
    try:
        report = json.loads((output / "evaluation" / "report.json").read_text())
        lines += [f"Physical completion: {report['completed_cases']}/{report['executed_cases']} executed cases; "
                  f"{report['rejected_cases']} preflight rejections; {report['error_cases']} errors.", "",
                  "| Case | Outcome | Expected result met | Replay | Placement error (mm) |",
                  "| --- | --- | --- | --- | --- |"]
        for case in report["cases"]:
            placement = case.get("placement")
            error = "—"
            if placement:
                value = math.hypot(*placement["placement_error_x_z_m"]) * 1000
                error = f"{value:.3f}" if math.isfinite(value) else "invalid"
            replay = {True: "pass", False: "FAIL", None: "not executed"}[case["replay_verified"]]
            lines.append("| " + " | ".join(map(cell, [case["name"], case["outcome"],
                         "yes" if case["expectation_met"] else "NO", replay, error])) + " |")
    except (OSError, ValueError, KeyError, TypeError):
        lines += ["Evaluation report unavailable or incomplete; inspect retained logs."]
    lines += ["", "Reports, recordings, and CLI logs are retained in the `lekiwi-evaluation` artifact.",
              "Comparison fixtures are synthetic checks, not additional physical trials.", ""]
    return "\n".join(lines)


def save_summary(output, summary):
    (output / "ci-summary.json").write_text(json.dumps(summary, indent=2))
    markdown = markdown_summary(output, summary)
    (output / "summary.md").write_text(markdown, encoding="utf-8")
    if os.environ.get("GITHUB_STEP_SUMMARY"):
        with open(os.environ["GITHUB_STEP_SUMMARY"], "a", encoding="utf-8") as stream:
            stream.write(markdown)


def main():
    parent = ROOT / "target" / "lekiwi-evaluation-runs"
    parent.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix="run-", dir=parent))
    print(f"LeKiwi evaluation CI evidence: {output}", flush=True)
    try:
        summary = check(output)
    except Exception as error:
        save_summary(output, {"passed": False, "error": str(error)})
        raise
    save_summary(output, summary)
    print(json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()
