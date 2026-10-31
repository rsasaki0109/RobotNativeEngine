"""Frozen-controller recovery across six world-axis force directions and strengths."""
import argparse
import copy
import json
import math
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from audit_arm_disturbance import PROTOCOL, run


def make_report(source, strengths):
    """Fix all cases before simulation; preserve the policy and evaluation seeds."""
    if source.get("task") != "stable_arm_pose_v1":
        raise ValueError("expected a stable-arm policy")
    if not strengths or any(not math.isfinite(s) or not 0 < s <= 20 for s in strengths):
        raise ValueError("force strengths must be finite and in (0, 20] N")
    if len(set(strengths)) != len(strengths):
        raise ValueError("force strengths must be unique")
    protocols = []
    for strength in strengths:
        for axis, name in enumerate("xyz"):
            for sign, label in ((1, "+"), (-1, "-")):
                force = [0.0] * 3
                force[axis] = sign * strength
                protocols.append({"direction": label + name, "strength_n": strength,
                                  "protocol": {**PROTOCOL, "force_world_n": force}})
    return copy.deepcopy({"audit": "physical_force_sweep_v1", "world": source["world"],
                          "controller": source["controller"], "fixed_arm_hold": source["fixed_arm_hold"],
                          "policy_log_gains": source["policy_log_gains"]["learned"],
                          "evaluation_seeds": source["evaluation_seeds"], "conditions": protocols})


def summarize(records, protocol):
    """Count every scheduled case, including missing preconditions and failed recovery."""
    forced = [r["lanes"][1] for r in records]
    return {"cases": len(records),
            "precondition_failures": sum(not r["precondition_met"] for r in records),
            "incomplete_pulses": sum(r["delivered_force_steps"] != protocol["pulse_steps"] for r in records),
            "unforced_successes": sum(r["lanes"][0]["success"] for r in records),
            "forced_successes": sum(l["success"] for l in forced),
            "displaced": sum(l["displaced"] for l in forced),
            "displaced_and_recovered": sum(l["displaced"] and l["success"] for l in forced),
            "forced_falls": sum(l["fallen"] for l in forced),
            "failed_seeds": [r["seed"] for r in records if not r["lanes"][1]["success"]]}


def evaluate(executable, report):
    def evaluate_condition(condition):
        records = run(executable, {**report, "protocol": condition["protocol"]})
        summary = summarize(records, condition["protocol"])
        return {**condition, "summary": summary, "evaluation": records}

    results = []
    # Each condition owns a separate engine process and independent paired worlds.
    # map yields in protocol order regardless of worker completion order.
    with ThreadPoolExecutor(max_workers=4) as executor:
        for result in executor.map(evaluate_condition, report["conditions"]):
            results.append(result)
            condition, summary = result, result["summary"]
            print(json.dumps({"direction": condition["direction"], "strength_n": condition["strength_n"],
                              **summary}), flush=True)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path, nargs="?")
    parser.add_argument("--strengths-n", type=float, nargs="+", default=[2.0, 4.0])
    parser.add_argument("--replay", action="store_true")
    args = parser.parse_args()
    source = json.loads(args.source.read_text())
    if args.replay:
        if source.get("audit") != "physical_force_sweep_v1":
            parser.error("expected a force sweep report")
        if evaluate(args.executable.resolve(), source) != source["results"]:
            raise SystemExit("force sweep replay mismatch")
        print("exact force sweep replay passed")
        return
    if args.output is None or args.output.exists():
        parser.error("provide a new output path")
    try:
        report = make_report(source, args.strengths_n)
    except ValueError as error:
        parser.error(str(error))
    report["results"] = evaluate(args.executable.resolve(), report)
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(report, output, indent=2, allow_nan=False)
        output.write("\n")


if __name__ == "__main__":
    main()
