#!/usr/bin/env python3
"""Strict streaming/framing and independent primitive postcard checks."""

import hashlib
import json
import struct
import zlib
from pathlib import Path

from stage_numeric import checked_word, exact, kind

PARAMETER_WORDS = ["dt_word", "min_ccd_dt_word", "contact_damping_ratio_word",
                   "contact_natural_frequency_word", "joint_natural_frequency_word",
                   "joint_damping_ratio_word", "warmstart_coefficient_word",
                   "length_unit_word", "normalized_allowed_linear_error_word",
                   "normalized_max_corrective_velocity_word", "normalized_prediction_distance_word"]
PARAMETER_COUNTS = ["num_solver_iterations", "num_additional_friction_iterations",
                    "num_internal_pgs_iterations", "num_internal_stabilization_iterations",
                    "min_island_size", "max_ccd_substeps"]
ROW_FIELDS = ["joint_handle", "link_id", "axis", "gain_model", "constraint_index",
              "solver_velocity_offset", "ndofs2", "j_id2", "jacobian_words",
              "weighted_jacobian_words", "rhs_word", "rhs_without_bias_word",
              "cfm_gain_word", "cfm_coefficient_word", "inverse_lhs_word",
              "impulse_word", "impulse_bound_words", "effort_ceiling_word"]
EVENT_FIELDS = ["ccd_partition_index", "island_index", "internal_slice_index",
                "expected_internal_slice_count", "iteration_index", "joint_visit_index",
                "stage", "native_scalar_bits", "duration_word", "row_cache_slice_index",
                "generalized_velocity_words", "rows"]
STAGES = ["BeforeExternalForces", "AfterExternalForces", "AfterJointUpdate",
          "AfterContactUpdate", "AfterContactWarmstart", "BeforeJointSolve",
          "BeforeMotorBiasRemoval", "BeforeMotorSolve", "AfterMotorSolve",
          "AfterJointSolve", "AfterContactNormal", "AfterContactFriction",
          "AfterAdditionalFriction", "AfterIntegration", "BeforeJointStabilization",
          "AfterJointStabilization", "AfterContactStabilization",
          "AfterStabilizationFriction", "FinalSlice"]
RECORD_KINDS = ["Body", "Collider", "Multibody", "ImpulseJoint"]
CHECKPOINT_SCOPE = "sorted individual native bodies, colliders, unique multibodies, impulse joints, gravity and integration parameters; excludes full-set metadata and private pipeline/solver/contact caches"


def require(condition, message):
    if not condition:
        raise ValueError(message)


def keys(value, fields, context):
    require(type(value) is dict and set(value) == set(fields), "unexpected fields/type: " + context)


def integer(value, maximum=(1 << 64) - 1, minimum=0):
    require(type(value) is int and minimum <= value <= maximum, "invalid bounded integer")
    return value


def literal_equal(left, right):
    if type(left) is not type(right):
        return False
    if type(left) is dict:
        return set(left) == set(right) and all(literal_equal(left[key], right[key]) for key in left)
    if type(left) is list:
        return len(left) == len(right) and all(literal_equal(a, b) for a, b in zip(left, right))
    return left == right


def words(value, width, expected=None, maximum=None):
    require(type(value) is list, "word vector must be a list")
    require(expected is None or len(value) == expected, "word vector length mismatch")
    require(maximum is None or len(value) <= maximum, "word vector resource ceiling exceeded")
    for word in value:
        checked_word(word, width)
    return value


def byte_vector(value, maximum):
    words(value, 8, maximum=maximum)
    return bytes(value)


def _pairs(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, "duplicate JSON key: " + key)
        result[key] = value
    return result


def _reject_number(value):
    raise ValueError("JSON floating/nonstandard numeric literal forbidden: " + value)


def strict_json(data):
    return json.loads(data.decode("utf-8"), object_pairs_hook=_pairs,
                      parse_float=_reject_number, parse_constant=_reject_number)


def sha_bytes(value):
    return hashlib.sha256(value).hexdigest()


def artifact(record):
    keys(record, ["path", "sha256", "bytes"], "artifact")
    require(type(record["path"]) is str and Path(record["path"]).is_absolute(), "artifact path must be absolute")
    require(type(record["sha256"]) is str and len(record["sha256"]) == 64 and
            all(x in "0123456789abcdef" for x in record["sha256"]), "invalid SHA256")
    integer(record["bytes"])
    digest, count = hashlib.sha256(), 0
    with Path(record["path"]).open("rb") as source:
        while block := source.read(1 << 20):
            digest.update(block); count += len(block)
    require(count == record["bytes"] and digest.hexdigest() == record["sha256"], "artifact bytes/hash mismatch: " + record["path"])
    return record


class GzipJsonLines:
    """One gzip member with checked CRC, finite ceilings and complete line framing."""

    def __init__(self, path, ceilings):
        self.path = Path(path)
        self.ceilings = ceilings
        self.lines = 0
        self.uncompressed_bytes = 0
        self.uncompressed_sha256 = None
        self.complete = False

    def __iter__(self):
        require(self.path.stat().st_size <= self.ceilings["compressed_bytes_per_run"], "compressed resource ceiling exceeded")
        decoder = zlib.decompressobj(31)
        buffer = bytearray()
        digest = hashlib.sha256()
        with self.path.open("rb") as source:
            header = source.read(10)
            require(len(header) == 10 and header[:4] == bytes.fromhex("1f8b0800") and
                    header[4:8] == b"\0" * 4 and header[9] == 255 and header[8] in (0, 2, 4),
                    "noncanonical deterministic gzip header")
            source.seek(0)
            while chunk := source.read(1 << 16):
                require(not decoder.eof, "trailing gzip bytes or concatenated member")
                pending = chunk
                while pending:
                    decoded = decoder.decompress(pending, 1 << 16)
                    pending = decoder.unconsumed_tail
                    self.uncompressed_bytes += len(decoded)
                    require(self.uncompressed_bytes <= self.ceilings["uncompressed_bytes_per_run"], "uncompressed resource ceiling exceeded")
                    digest.update(decoded)
                    buffer.extend(decoded)
                    while (newline := buffer.find(b"\n")) >= 0:
                        require(0 < newline + 1 <= self.ceilings["json_line_bytes"], "empty or oversized JSONL line")
                        line = bytes(buffer[:newline]); del buffer[:newline + 1]
                        require(not line.endswith(b"\r"), "CRLF JSONL record forbidden")
                        self.lines += 1
                        require(self.lines <= 1252, "extra JSONL records")
                        yield strict_json(line)
                    require(len(buffer) <= self.ceilings["json_line_bytes"], "oversized partial JSONL line")
                    if decoder.eof:
                        require(not decoder.unused_data and not pending, "trailing gzip bytes or concatenated member")
                        break
            require(decoder.eof, "truncated gzip stream or missing trailer")
            require(not decoder.unused_data and not buffer, "trailing bytes or unterminated JSONL record")
        self.uncompressed_sha256 = digest.hexdigest()
        self.complete = True


def varint(value):
    value = integer(value)
    output = bytearray()
    while value >= 128:
        output.append((value & 127) | 128)
        value >>= 7
    output.append(value)
    return bytes(output)


def _encoded_bytes(value, maximum):
    data = byte_vector(value, maximum)
    return varint(len(data)) + data


def encode_checkpoint(checkpoint, ceilings):
    """Primitive outer struct: postcard varints/u8/enums/Vec, actual declared order."""
    keys(checkpoint, ["schema_version", "native_scalar_bits", "gravity_ieee_words",
                      "integration_parameters", "integration_parameters_postcard_bytes",
                      "records", "bindings"], "native checkpoint")
    require(checkpoint["schema_version"] == 1 and type(checkpoint["schema_version"]) is int and
            checkpoint["native_scalar_bits"] == 32 and type(checkpoint["native_scalar_bits"]) is int,
            "checkpoint schema/scalar width differs")
    output = bytearray(varint(1) + b"\x20")
    for word in words(checkpoint["gravity_ieee_words"], 32, expected=3):
        output.extend(varint(word))
    parameters = checkpoint["integration_parameters"]
    keys(parameters, PARAMETER_WORDS + PARAMETER_COUNTS, "integration parameters")
    parameter_native = bytearray()
    for name in PARAMETER_WORDS:
        word = checked_word(parameters[name])
        output.extend(varint(word))
        parameter_native.extend(word.to_bytes(4, "little"))
    for name in PARAMETER_COUNTS:
        value = integer(parameters[name])
        output.extend(varint(value)); parameter_native.extend(varint(value))
    require(parameters["num_solver_iterations"] > 0, "zero native NonZeroUsize solver count")
    raw_parameters = byte_vector(checkpoint["integration_parameters_postcard_bytes"], ceilings["native_record_bytes"])
    require(raw_parameters == parameter_native, "native IntegrationParameters postcard differs from declared IEEE fields/counts")
    output.extend(_encoded_bytes(checkpoint["integration_parameters_postcard_bytes"], ceilings["native_record_bytes"]))
    records = checkpoint["records"]
    require(type(records) is list and len(records) <= ceilings["native_records_per_step"], "native record resource ceiling")
    output.extend(varint(len(records)))
    previous = None
    record_counts = {name: 0 for name in RECORD_KINDS}
    handles = {name: set() for name in RECORD_KINDS}
    for record in records:
        keys(record, ["kind", "handle_words", "postcard_bytes"], "native record")
        require(record["kind"] in RECORD_KINDS, "unknown native record kind")
        rank = RECORD_KINDS.index(record["kind"])
        handle = tuple(words(record["handle_words"], 32, expected=2))
        identity = (rank,) + handle
        require(previous is None or previous < identity, "native records unsorted or duplicated")
        previous = identity; record_counts[record["kind"]] += 1; handles[record["kind"]].add(handle)
        output.extend(varint(rank))
        for word in handle:
            output.extend(varint(word))
        require(record["postcard_bytes"], "empty native entity payload")
        output.extend(_encoded_bytes(record["postcard_bytes"], ceilings["native_record_bytes"]))
    require(record_counts == {"Body": 6, "Collider": 5, "Multibody": 1, "ImpulseJoint": 0},
            "native object coverage differs from the fixed example138 scene")
    bindings = checkpoint["bindings"]
    require(type(bindings) is list and len(bindings) <= ceilings["native_bindings_per_step"], "native binding resource ceiling")
    output.extend(varint(len(bindings)))
    previous = None
    for binding in bindings:
        keys(binding, ["entity_bits", "joint_handle_words", "multibody_handle_words", "link_index"], "joint binding")
        entity = integer(binding["entity_bits"])
        require(previous is None or previous < entity, "native ECS bindings unsorted or duplicated")
        previous = entity; output.extend(varint(entity))
        for name in ["joint_handle_words", "multibody_handle_words"]:
            handle = tuple(words(binding[name], 32, expected=2))
            if name == "joint_handle_words":
                require(handle in handles["Body"], "joint binding references absent native body")
            else:
                require(handle in handles["Multibody"], "joint binding references absent multibody")
            for word in handle:
                output.extend(varint(word))
        output.extend(varint(integer(binding["link_index"])))
    require(len(bindings) == 3, "fixed scene requires three child joint bindings")
    return bytes(output)


def parse_ecs(value, ceilings):
    words(value, 64, maximum=ceilings["ecs_words_per_step"])
    require(value[:2] == [1, 6], "ECS schema/entity count mismatch")
    cursor, entities = 2, []
    def take(count):
        nonlocal cursor
        require(cursor + count <= len(value), "truncated ECS word payload")
        result = value[cursor:cursor + count]; cursor += count
        return result
    for _ in range(6):
        entity, transform_present = take(2)
        require(transform_present in (0, 1), "invalid transform presence")
        pose = take(10) if transform_present else None
        body_present = take(1)[0]
        require(body_present in (0, 1), "invalid body presence")
        body = take(8) if body_present else None
        require(body is None or body[0] in (0, 1, 2), "invalid body type")
        joint_tag = take(1)[0]
        require(joint_tag in (0, 1, 2, 3), "invalid joint-state tag")
        joint = take(2) if joint_tag >= 2 else []
        entities.append({"entity_bits": entity, "pose": pose, "body": body,
                         "joint_tag": joint_tag, "joint": joint})
    require(cursor == len(value), "trailing ECS words")
    require(len({x["entity_bits"] for x in entities}) == 6, "duplicate ECS entity")
    # The frozen fixture has six first-generation entities. Their Entity Ord and
    # raw bit order agree; later generations are not silently generalized here.
    require([x["entity_bits"] for x in entities] == sorted(x["entity_bits"] for x in entities), "ECS entities out of declared fixture order")
    require(all(x["pose"] is not None and x["body"] is not None for x in entities), "fixed scene missing pose/body")
    require([x["body"][0] for x in entities] == [1, 0, 1, 0, 0, 0], "fixed scene body order/types differ")
    require([x["joint_tag"] for x in entities] == [0, 0, 0, 2, 2, 2], "fixed scene joint-state coverage differs")
    return entities


def validate_row(row, vector_length, ceilings):
    keys(row, ROW_FIELDS, "motor row")
    words(row["joint_handle"], 32, expected=2)
    for field in ["link_id", "constraint_index", "solver_velocity_offset", "j_id2"]:
        integer(row[field])
    integer(row["ndofs2"], maximum=ceilings["motor_ndofs"], minimum=1)
    require(row["solver_velocity_offset"] + row["ndofs2"] <= vector_length, "motor velocity block out of bounds")
    require(row["axis"] in ["LinX", "LinY", "LinZ", "AngX", "AngY", "AngZ"], "unknown motor axis")
    require(row["gain_model"] in ["ForceBased", "AccelerationBased"], "unknown motor gain model")
    for field in ["jacobian_words", "weighted_jacobian_words"]:
        words(row[field], 32, expected=row["ndofs2"])
    words(row["impulse_bound_words"], 32, expected=2)
    for field in ROW_FIELDS[10:16] + ["effort_ceiling_word"]:
        checked_word(row[field])


def validate_event(event, partitions, ceilings):
    keys(event, EVENT_FIELDS, "stage event")
    for name in ["ccd_partition_index", "island_index", "internal_slice_index"]:
        integer(event[name])
    require(event["ccd_partition_index"] < len(partitions), "stage partition out of range")
    integer(event["expected_internal_slice_count"], maximum=ceilings["slices_per_island_partition"], minimum=1)
    require(event["internal_slice_index"] < event["expected_internal_slice_count"], "slice ordinal out of range")
    for name in ["iteration_index", "joint_visit_index", "row_cache_slice_index"]:
        if event[name] is not None:
            integer(event[name])
    require(event["stage"] in STAGES and event["native_scalar_bits"] == 32 and type(event["native_scalar_bits"]) is int,
            "stage name or scalar width mismatch")
    checked_word(event["duration_word"])
    require(exact(event["duration_word"]) > 0, "nonpositive/nonfinite native slice duration")
    words(event["generalized_velocity_words"], 32, maximum=ceilings["generic_velocity_words"])
    require(type(event["rows"]) is list and len(event["rows"]) <= ceilings["motor_rows_per_event"], "row resource ceiling")
    for row in event["rows"]:
        validate_row(row, len(event["generalized_velocity_words"]), ceilings)
    identities = [tuple(row["joint_handle"]) + (row["link_id"], row["axis"], row["constraint_index"]) for row in event["rows"]]
    require(len(set(identities)) == len(identities), "duplicate supported row identity")
    before_update = event["stage"] in ["BeforeExternalForces", "AfterExternalForces"]
    if before_update:
        require(event["rows"] == [] and event["row_cache_slice_index"] is None, "uninitialized row storage was exposed")
    else:
        require(event["row_cache_slice_index"] == event["internal_slice_index"], "wrong or unavailable cached row generation")
    immediate = event["stage"] in ["BeforeMotorBiasRemoval", "BeforeMotorSolve", "AfterMotorSolve"]
    require((event["joint_visit_index"] is not None) == immediate, "immediate/global-visit metadata mismatch")
    if immediate:
        require(len(event["rows"]) == 1, "immediate event must expose exactly the selected row")


def f64_bits(value):
    return struct.unpack("<Q", struct.pack("<d", value))[0]


def f32_bits(value):
    return struct.unpack("<I", struct.pack("<f", value))[0]


def expected_conditions(mass):
    friction = struct.unpack("<f", struct.pack("<f", .8))[0]
    return {"step_hz_ieee754_bits": f64_bits(500), "outer_duration_s_ieee754_bits": f64_bits(.002),
            "gravity_y_m_s2_ieee754_bits": f64_bits(-9.81), "friction_f32_ieee754_bits": f32_bits(.8),
            "squeeze_n_ieee754_bits": f64_bits(1.25 * (mass * 9.81 / (2 * friction))),
            "lift_distance_m_ieee754_bits": f64_bits(.2), "lift_stiffness_n_per_m_ieee754_bits": f64_bits(1e5),
            "lift_damping_n_s_per_m_ieee754_bits": f64_bits(1e4), "lift_ceiling_n_ieee754_bits": f64_bits(5000),
            "settle_steps": 250, "lift_steps": 500, "hold_steps": 500, "expected_default_small_steps": 4,
            "expected_default_pgs_passes": 1, "expected_default_stabilization_passes": 2,
            "expected_default_warmstart_f32_ieee754_bits": f32_bits(1)}


def validate_start(record, cell):
    keys(record, ["kind", "schema_version", "run_label", "mass_kg_ieee754_bits", "observer_enabled",
                  "expected_completed_outer_steps", "expected_ecs_entity_count", "ecs_word_schema_version",
                  "checkpoint_codec", "checkpoint_scope", "declared_conditions"], "run_start")
    require(record["kind"] == "run_start" and record["schema_version"] == 1 and type(record["schema_version"]) is int,
            "wrong initial frame")
    require(record["run_label"] == cell["run_label"] and record["mass_kg_ieee754_bits"] == f64_bits(cell["mass_kg"]), "run identity/mass mismatch")
    require(type(record["observer_enabled"]) is bool and record["observer_enabled"] == (cell["mode"] != "off"), "observer mode mismatch")
    require(record["expected_completed_outer_steps"] == 1250 and record["expected_ecs_entity_count"] == 6 and record["ecs_word_schema_version"] == 1,
            "declared fixture scope changed")
    require(record["checkpoint_codec"] == "postcard-1.1.3-raw-native" and record["checkpoint_scope"] == CHECKPOINT_SCOPE, "checkpoint codec/scope changed")
    require(literal_equal(record["declared_conditions"], expected_conditions(cell["mass_kg"])), "physical conditions/phase counts changed")


def validate_end(record):
    keys(record, ["kind", "schema_version", "status", "completed_outer_steps", "frames_written", "error", "physical_report"], "run_end")
    require(record["kind"] == "run_end" and record["schema_version"] == 1 and record["status"] == "collected" and
            record["completed_outer_steps"] == record["frames_written"] == 1250 and record["error"] is None, "missing/error/partial terminal")
    integer(record["schema_version"], maximum=1, minimum=1)
    integer(record["completed_outer_steps"], maximum=1250, minimum=1250)
    integer(record["frames_written"], maximum=1250, minimum=1250)
    physical = record["physical_report"]
    keys(physical, ["cube_ecs_pose_ieee_words", "physical_ieee_words", "physical_stdout_row"], "physical report")
    words(physical["cube_ecs_pose_ieee_words"], 64, expected=10)
    words(physical["physical_ieee_words"], 64, expected=4)
    require(type(physical["physical_stdout_row"]) is str and "\n" not in physical["physical_stdout_row"], "invalid physical stdout row")
