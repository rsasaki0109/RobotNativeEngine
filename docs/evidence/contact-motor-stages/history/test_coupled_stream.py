#!/usr/bin/env python3
"""One synthetic 1250-frame triad and first-frame literal replay tampering.

All states/events are synthetic arithmetic words. No binary is invoked and no
physics world is created; this is reader framing/coverage qualification only.
"""

import copy
import gzip
import json
from pathlib import Path
import tempfile

from audit_coupled_stages import audit_cell, record_for
from stage_schema import CHECKPOINT_SCOPE, expected_conditions, f64_bits, parse_ecs
from test_stage_reader import fixture


def main():
    protocol, frame = fixture()
    entities = parse_ecs(frame["ecs_physics_ieee_words"], protocol["resource_ceilings"])
    physical = {"cube_ecs_pose_ieee_words": entities[1]["pose"], "physical_ieee_words": [0] * 4,
                "physical_stdout_row": "synthetic-not-a-physics-run"}
    with tempfile.TemporaryDirectory(prefix="rne-pure-coupled-stream-", dir="/tmp") as directory:
        directory = Path(directory)
        def triad(tag, mutate=None):
            rows = []
            for mode in ["on", "fresh_on", "off"]:
                label = "baseline-m5-" + mode
                path = directory / (tag + "-" + mode + ".jsonl.gz")
                with path.open("xb") as raw:
                    with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as compressed:
                        start = {"kind": "run_start", "schema_version": 1, "run_label": label,
                                 "mass_kg_ieee754_bits": f64_bits(5), "observer_enabled": mode != "off",
                                 "expected_completed_outer_steps": 1250, "expected_ecs_entity_count": 6,
                                 "ecs_word_schema_version": 1, "checkpoint_codec": "postcard-1.1.3-raw-native",
                                 "checkpoint_scope": CHECKPOINT_SCOPE, "declared_conditions": expected_conditions(5)}
                        compressed.write(json.dumps(start).encode() + b"\n")
                        for step in range(1, 1251):
                            sample = copy.deepcopy(frame)
                            sample["completed_outer_step"] = step
                            if mode == "off":
                                sample["stage_telemetry"] = None
                            if mutate is not None and step == 1:
                                mutate(mode, sample)
                            compressed.write(json.dumps(sample, separators=(",", ":")).encode() + b"\n")
                        terminal = {"kind": "run_end", "schema_version": 1, "status": "collected",
                                    "completed_outer_steps": 1250, "frames_written": 1250, "error": None,
                                    "physical_report": physical}
                        compressed.write(json.dumps(terminal).encode() + b"\n")
                stdout = directory / (tag + "-" + mode + ".stdout")
                stdout.write_bytes(b"synthetic-not-a-physics-run\ncontact motor stage collection: ok\n")
                rows.append({"build": "baseline", "mass_kg": 5, "mode": mode, "run_label": label,
                             "trace": record_for(path), "stdout": record_for(stdout)})
            return rows
        complete = triad("complete")
        details = directory / "complete-details"; details.mkdir()
        result = audit_cell(complete, {}, protocol, details)
        assert result["physics_checkpoints"] == 3750
        assert result["analysis"]["counts"]["completed_outer_steps"] == 1250
        assert result["analysis"]["counts"]["immediate_motor_visits:qualified_match"] == 2500
        assert result["analysis"]["complete_native_replay"]
        rejected = []
        def tamper(name, mutate, wanted):
            rows = triad(name, mutate)
            output = directory / (name + "-details"); output.mkdir()
            try:
                audit_cell(rows, {}, protocol, output)
            except ValueError as error:
                assert wanted in str(error), str(error)
                rejected.append({"name": name, "actual_error": str(error)})
            else:
                raise AssertionError("literal replay tamper admitted")
        tamper("off-signed-zero", lambda mode, sample: sample["ecs_physics_ieee_words"].__setitem__(
            4, 1 << 63) if mode == "off" else None, "observer physical replay differs")
        tamper("fresh-ordered-trace", lambda mode, sample: sample["stage_telemetry"]["events"].__setitem__(
            slice(9, 11), sample["stage_telemetry"]["events"][9:11][::-1]) if mode == "fresh_on" else None,
            "fresh ON ordered stage words differ")
        print(json.dumps({"status": "PASS", "native_worlds": 0,
                          "scope": "one synthetic 1250-frame triad plus literal replay tampering; no compiled binary invoked",
                          "synthetic_coverage": {"physics_checkpoints": result["physics_checkpoints"],
                                                 "immediate_motor_visits": result["analysis"]["counts"]["immediate_motor_visits:qualified_match"],
                                                 "stream_record_counts": [x["records"] for x in result["input_streams"]]},
                          "rejected": rejected}, indent=2))


if __name__ == "__main__":
    main()
