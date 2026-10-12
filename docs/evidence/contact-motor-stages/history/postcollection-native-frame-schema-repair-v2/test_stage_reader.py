#!/usr/bin/env python3
"""Synthetic framing/identity/cache/phase/tamper checks, without native worlds."""

import copy
import gzip
import json
from pathlib import Path
import struct
import tempfile

from stage_analysis import Analysis, analyze_telemetry
from stage_numeric import F32, ONE, SIGN
from stage_schema import (CHECKPOINT_SCOPE, GzipJsonLines, PARAMETER_COUNTS,
                          PARAMETER_WORDS, encode_checkpoint, expected_conditions,
                          f32_bits, f64_bits, parse_ecs, strict_json, validate_end,
                          validate_start, varint)
from audit_coupled_stages import validate_step

HERE = Path(__file__).parent


def fixture():
    protocol = json.loads((HERE.parent / "protocol-template.json").read_bytes())
    parameters = dict(protocol["expected_runtime_parameters"])
    # Synthetic one-slice arithmetic profile, deliberately not a physical run.
    parameters.update(num_solver_iterations=1, num_internal_stabilization_iterations=1)
    h = parameters["dt_word"]
    numeric = F32()
    g = numeric.inverse(numeric.add(numeric.multiply(numeric.multiply(h, h), f32_bits(1e5)), numeric.multiply(h, f32_bits(1e4))))
    bound = numeric.multiply(f32_bits(5000), h)
    row = {"joint_handle": {"index": 3, "generation": 0}, "link_id": 1, "axis": "LinX", "gain_model": "ForceBased",
           "constraint_index": 0, "solver_velocity_offset": 0, "ndofs2": 3, "j_id2": 0,
           "jacobian_words": [ONE, 0, 0], "weighted_jacobian_words": [f32_bits(2), f32_bits(3), 0],
           "rhs_word": f32_bits(4), "rhs_without_bias_word": f32_bits(4), "cfm_gain_word": g,
           "cfm_coefficient_word": 0, "inverse_lhs_word": f32_bits(.5), "impulse_word": 0,
           "impulse_bound_words": [bound ^ SIGN, bound], "effort_ceiling_word": f32_bits(5000)}
    velocity = [0, 0, 0]
    events = []
    def event(stage, iteration=None, visit=None, initialized=True):
        events.append({"ccd_partition_index": 0, "island_index": 0, "internal_slice_index": 0,
                       "expected_internal_slice_count": 1, "iteration_index": iteration, "joint_visit_index": visit,
                       "stage": stage, "native_scalar_bits": 32, "duration_word": h,
                       "row_cache_slice_index": 0 if initialized else None,
                       "generalized_velocity_words": list(velocity), "rows": [copy.deepcopy(row)] if initialized else []})
    event("BeforeExternalForces", initialized=False)
    event("AfterExternalForces", initialized=False)
    event("AfterJointUpdate")
    event("AfterContactUpdate")
    event("AfterContactWarmstart")
    event("BeforeJointSolve", 0)
    event("BeforeMotorSolve", 0, 2)
    next_visit = F32().projection(row, velocity)
    row["impulse_word"] = next_visit["next_impulse_word"]
    velocity[:] = next_visit["next_generalized_velocity_words"]
    event("AfterMotorSolve", 0, 2)
    event("AfterJointSolve", 0)
    velocity[0] = F32().add(velocity[0], f32_bits(.5))
    event("AfterContactNormal", 0)
    velocity[0] = F32().subtract(velocity[0], f32_bits(.25))
    event("AfterContactFriction", 0)
    event("AfterIntegration")
    event("BeforeJointStabilization", 0)
    event("BeforeMotorBiasRemoval", 0, 2)
    event("BeforeMotorSolve", 0, 2)
    next_visit = F32().projection(row, velocity)
    row["impulse_word"] = next_visit["next_impulse_word"]
    velocity[:] = next_visit["next_generalized_velocity_words"]
    event("AfterMotorSolve", 0, 2)
    event("AfterJointStabilization", 0)
    event("AfterContactStabilization", 0)
    event("AfterStabilizationFriction")
    event("FinalSlice")
    telemetry = {"numerical_outer_interval_word": h, "native_scalar_bits": 32,
                 "ccd_partition_duration_words": [h], "events": events}
    native_parameters = b"".join(parameters[name].to_bytes(4, "little") for name in PARAMETER_WORDS) + b"".join(varint(parameters[name]) for name in PARAMETER_COUNTS)
    checkpoint = {"schema_version": 1, "native_scalar_bits": 32, "gravity_ieee_words": [0, f32_bits(-9.81), 0],
                  "integration_parameters": parameters, "integration_parameters_postcard_bytes": list(native_parameters),
                  "records": [{"kind": kind, "handle_words": [index, 0], "postcard_bytes": [42, index]}
                              for kind, count in [("Body", 6), ("Collider", 5), ("Multibody", 1)] for index in range(count)],
                  "bindings": [{"entity_bits": 100 + index, "joint_handle_words": [index, 0],
                                "multibody_handle_words": [0, 0], "link_index": index - 2} for index in [3, 4, 5]]}
    ecs = [1, 6]
    for index in range(6):
        pose = [f64_bits(0)] * 6 + [f64_bits(1)] + [f64_bits(1)] * 3
        body = [1 if index in [0, 2] else 0, f64_bits(1)] + [f64_bits(0)] * 6
        ecs.extend([100 + index, 1] + pose + [1] + body + ([2, 0, 0] if index >= 3 else [0]))
    protocol["expected_runtime_parameters"] = parameters
    frame = {"kind": "completed_step", "schema_version": 1, "completed_outer_step": 1,
             "native_checkpoint": checkpoint, "native_checkpoint_postcard_bytes": list(encode_checkpoint(checkpoint, protocol["resource_ceilings"])),
             "ecs_physics_ieee_words": ecs, "stage_telemetry": telemetry}
    return protocol, frame


def main():
    passed, rejected = [], []
    def check(name, condition):
        if not condition:
            raise AssertionError(name)
        passed.append(name)
    def fails(name, operation):
        try:
            operation()
        except (ValueError, TypeError, IndexError, KeyError, UnicodeError, EOFError, __import__("zlib").error):
            rejected.append(name)
        else:
            raise AssertionError("tamper admitted: " + name)
    protocol, frame = fixture()
    entities = validate_step(frame, 1, True, protocol)
    details = []
    analysis = Analysis("baseline", details.append)
    analyze_telemetry(frame["stage_telemetry"], frame["native_checkpoint"], entities, 1, protocol, analysis)
    summary = analysis.summary()
    check("synthetic typed checkpoint independently reconstructs postcard", len(frame["native_checkpoint_postcard_bytes"]) > 0)
    check("synthetic immediate native word replay fully qualified", summary["complete_native_replay"])
    check("both PGS and stabilization visit coverage", summary["counts"]["immediate_motor_visits:qualified_match"] == 2)
    check("all contact classes retained without cutoff", summary["counts"]["aggregate_contact_boundaries"] == 5)
    check("cached row remains explicitly old after integration", summary["counts"]["old_cached_row_after_integration_events"] == 9)
    check("uninitialized events retain unavailable rows", sum(x["kind"] == "row_availability" and x["status"] == "uninitialized" for x in details) == 2)
    check("off null trace is valid physical checkpoint", validate_step(dict(frame, stage_telemetry=None), 1, False, protocol) == entities)
    def changed_frame(name, change):
        altered = copy.deepcopy(frame); change(altered)
        fails(name, lambda: validate_step(altered, 1, True, protocol))
    changed_frame("postcard byte tamper", lambda x: x["native_checkpoint_postcard_bytes"].__setitem__(5, x["native_checkpoint_postcard_bytes"][5] ^ 1))
    changed_frame("native parameter IEEE field without byte transition", lambda x: x["native_checkpoint"]["integration_parameters"].__setitem__("dt_word", 1))
    changed_frame("native record duplicate", lambda x: x["native_checkpoint"]["records"].__setitem__(1, x["native_checkpoint"]["records"][0]))
    changed_frame("ECS presence tag tamper", lambda x: x["ecs_physics_ieee_words"].__setitem__(3, 2))
    changed_frame("ECS signed-zero word is not normalized", lambda x: x["ecs_physics_ieee_words"].append(1 << 63))
    changed_frame("bool step rejected", lambda x: x.__setitem__("completed_outer_step", True))
    fails("OFF with non-null observer payload", lambda: validate_step(frame, 1, False, protocol))
    def phase_tamper(name, mutate):
        telemetry = copy.deepcopy(frame["stage_telemetry"]); mutate(telemetry)
        candidate = Analysis("baseline", lambda value: None)
        fails(name, lambda: analyze_telemetry(telemetry, frame["native_checkpoint"], entities, 1, protocol, candidate))
    phase_tamper("missing warmstart stage", lambda x: x["events"].pop(4))
    phase_tamper("missing immediate after-visit", lambda x: x["events"].pop(7))
    phase_tamper("reordered contact stages", lambda x: x["events"].__setitem__(slice(9, 11), x["events"][9:11][::-1]))
    phase_tamper("uninitialized row storage exposure", lambda x: x["events"][0].__setitem__("rows", x["events"][2]["rows"]))
    phase_tamper("wrong cached generation", lambda x: x["events"][12].__setitem__("row_cache_slice_index", 1))
    phase_tamper("wrong block offset", lambda x: x["events"][2]["rows"][0].__setitem__("solver_velocity_offset", 1))
    phase_tamper("unknown field hidden in row", lambda x: x["events"][2]["rows"][0].__setitem__("diagnostic", 0))
    phase_tamper("registered diagonal wrong build", lambda x: [event["rows"][0].__setitem__("inverse_lhs_word", f32_bits(.25)) for event in x["events"] if event["rows"]])
    mismatch = copy.deepcopy(frame["stage_telemetry"])
    mismatch["events"][7]["generalized_velocity_words"][1] = ONE
    mismatched_analysis = Analysis("baseline", lambda value: None)
    analyze_telemetry(mismatch, frame["native_checkpoint"], entities, 1, protocol, mismatched_analysis)
    check("literal native replay mismatch retained as failure", mismatched_analysis.issues > 0 and not mismatched_analysis.summary()["complete_native_replay"])
    contact_mutated = copy.deepcopy(frame["stage_telemetry"])
    contact_mutated["events"][9]["rows"][0]["impulse_word"] = ONE
    changed_analysis = Analysis("baseline", lambda value: None)
    analyze_telemetry(contact_mutated, frame["native_checkpoint"], entities, 1, protocol, changed_analysis)
    check("contact lambda mutation retained ineligible not relabelled", changed_analysis.counts["contact_intervals:field_or_cache_changed"] > 0 and changed_analysis.issues > 0)
    fails("duplicate JSON key", lambda: strict_json(b'{"a":1,"a":2}'))
    fails("JSON NaN constant", lambda: strict_json(b'{"a":NaN}'))
    fails("JSON float metadata", lambda: strict_json(b'{"a":1.0}'))
    fails("JSON invalid UTF8", lambda: strict_json(b'{"a":"\xff"}'))
    cell = {"run_label": "baseline-m5-on", "mass_kg": 5, "mode": "on"}
    start = {"kind": "run_start", "schema_version": 1, "run_label": cell["run_label"], "mass_kg_ieee754_bits": f64_bits(5),
             "observer_enabled": True, "expected_completed_outer_steps": 1250, "expected_ecs_entity_count": 6,
             "ecs_word_schema_version": 1, "checkpoint_codec": "postcard-1.1.3-raw-native", "checkpoint_scope": CHECKPOINT_SCOPE,
             "declared_conditions": expected_conditions(5)}
    validate_start(start, cell)
    altered = copy.deepcopy(start); altered["declared_conditions"]["expected_default_warmstart_f32_ieee754_bits"] = 0
    fails("retuned declared warmstart", lambda: validate_start(altered, cell))
    terminal = {"kind": "run_end", "schema_version": 1, "status": "collected", "completed_outer_steps": 1250, "frames_written": 1250,
                "error": None, "physical_report": {"cube_ecs_pose_ieee_words": [0] * 10, "physical_ieee_words": [0] * 4, "physical_stdout_row": "synthetic"}}
    validate_end(terminal)
    fails("partial terminal count", lambda: validate_end(dict(terminal, frames_written=1249)))
    fails("failed terminal retained as rejection", lambda: validate_end(dict(terminal, status="failed", error="retained failure")))
    with tempfile.TemporaryDirectory(prefix="rne-pure-stage-reader-", dir="/tmp") as directory:
        directory = Path(directory)
        raw = b"".join(json.dumps(record, separators=(",", ":")).encode() + b"\n" for record in
                       [start] + [{"kind": "synthetic_frame", "ordinal": i} for i in range(1, 1251)] + [terminal])
        compressed = gzip.compress(raw, mtime=0)
        # Python 3.12 gzip.compress uses OS=3; declare the producer's OS=255.
        compressed = compressed[:9] + b"\xff" + compressed[10:]
        def stream(data, ceilings=None):
            path = directory / (str(len(list(directory.iterdir()))) + ".jsonl.gz")
            path.write_bytes(data)
            reader = GzipJsonLines(path, ceilings or protocol["resource_ceilings"])
            decoded = list(reader)
            return reader, decoded
        reader, decoded = stream(compressed)
        check("complete 1252 record stream and CRC", reader.complete and reader.lines == 1252 and len(decoded) == 1252)
        check("exact decompressed framing hash", reader.uncompressed_bytes == len(raw))
        fails("truncated gzip trailer", lambda: stream(compressed[:-3]))
        damaged = bytearray(compressed); damaged[-8] ^= 1
        fails("gzip CRC corruption", lambda: stream(bytes(damaged)))
        fails("concatenated gzip member", lambda: stream(compressed + compressed))
        fails("trailing gzip bytes", lambda: stream(compressed + b"suffix"))
        truncated_line = gzip.compress(raw[:-1], mtime=0); truncated_line = truncated_line[:9] + b"\xff" + truncated_line[10:]
        fails("unterminated JSONL record", lambda: stream(truncated_line))
        oversize = dict(protocol["resource_ceilings"], json_line_bytes=100)
        fails("declared line resource ceiling", lambda: stream(compressed, oversize))
    print(json.dumps({"status": "PASS", "native_worlds": 0, "scope": "synthetic primitive/framing/phase/tamper checks only", "passed": passed, "rejected": rejected}, indent=2))


if __name__ == "__main__":
    main()
