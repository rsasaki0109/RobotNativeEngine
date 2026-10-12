#!/usr/bin/env python3
"""Source-derived three-role synthetic framing/identity/phase/tamper tests.

All values are invented pure operands, never observations of a physics world.
Native object payloads below are synthetic opaque codec subjects only.
"""
import copy
import gzip
import itertools
import json
from pathlib import Path
import tempfile
import unittest

from audit_coupled_stages import validate_step
from stage_analysis import Analysis, analyze_telemetry
from stage_numeric import F32, ONE, SIGN, exact, exact_snapshot, rounded
from stage_roles import Roles, ROLES
from stage_schema import (GzipJsonLines, PARAMETER_COUNTS, PARAMETER_WORDS,
                          encode_checkpoint, expected_conditions, f32_bits,
                          f64_bits, strict_json, validate_end, validate_start,
                          varint, CHECKPOINT_SCOPE)

HERE = Path(__file__).parent


def fixture(order=ROLES, build="baseline"):
    protocol = json.loads((HERE / "synthetic-profile.json").read_bytes())
    parameters = dict(protocol["expected_runtime_parameters"])
    parameters.update(num_solver_iterations=1, num_internal_stabilization_iterations=1)
    protocol["expected_runtime_parameters"] = parameters
    h = parameters["dt_word"]
    n = F32()
    g = n.inverse(n.add(n.multiply(n.multiply(h, h), f32_bits(1e5)), n.multiply(h, f32_bits(1e4))))
    cap = n.multiply(f32_bits(5000), h)
    # A symmetric positive definite synthetic W; no native factorization/data.
    columns = [[f32_bits(2), f32_bits(.25), f32_bits(.125)],
               [f32_bits(.25), f32_bits(1.5), f32_bits(.0625)],
               [f32_bits(.125), f32_bits(.0625), ONE]]
    rows, constraint = [], 0
    for role in order:
        coordinate = ROLES.index(role)
        diagonal = columns[coordinate][coordinate]
        if build == "rejected_inverse_w_plus_g" and role == "palm":
            diagonal = F32().add(diagonal, g)
        rhs = f32_bits([4, 3, -2][coordinate])
        rows.append({"joint_handle": {"index": 30 + coordinate, "generation": 7},
                     "link_id": 1 + coordinate, "axis": "LinX",
                     "gain_model": "ForceBased" if role == "palm" else "AccelerationBased",
                     "constraint_index": constraint, "solver_velocity_offset": 0,
                     "ndofs2": 3, "j_id2": 6 * constraint,
                     "jacobian_words": [ONE if i == coordinate else 0 for i in range(3)],
                     "weighted_jacobian_words": columns[coordinate], "rhs_word": rhs,
                     "rhs_without_bias_word": rhs, "cfm_gain_word": g if role == "palm" else 0,
                     "cfm_coefficient_word": 0, "inverse_lhs_word": F32().inverse(diagonal),
                     "impulse_word": 0, "impulse_bound_words": [cap ^ SIGN, cap] if role == "palm" else [SIGN, 0],
                     "effort_ceiling_word": f32_bits(5000) if role == "palm" else 0})
        constraint += 1 if role == "palm" else 2
    velocity, events = [0, 0, 0], []
    def event(stage, iteration=None, selected=None, initialized=True):
        events.append({"ccd_partition_index": 0, "island_index": 0, "internal_slice_index": 0,
                       "expected_internal_slice_count": 1, "iteration_index": iteration,
                       "joint_visit_index": selected["constraint_index"] if selected is not None else None,
                       "stage": stage, "native_scalar_bits": 32, "duration_word": h,
                       "row_cache_slice_index": 0 if initialized else None,
                       "generalized_velocity_words": list(velocity),
                       "rows": copy.deepcopy([selected] if selected is not None else rows) if initialized else []})
    def visits(stabilization=False):
        for row in rows:
            if stabilization:
                event("BeforeMotorBiasRemoval", 0, row)
                row["rhs_word"] = row["rhs_without_bias_word"]
            event("BeforeMotorSolve", 0, row)
            predicted = F32().projection(row, velocity)
            row["impulse_word"] = predicted["next_impulse_word"]
            velocity[:] = predicted["next_generalized_velocity_words"]
            event("AfterMotorSolve", 0, row)
            # Synthetic uncaptured limit-row write: a full joint-pass velocity
            # difference cannot be attributed to the selected palm motor alone.
            if row["gain_model"] == "AccelerationBased":
                velocity[0] = F32().add(velocity[0], f32_bits(.03125))
    event("BeforeExternalForces", initialized=False)
    event("AfterExternalForces", initialized=False)
    event("AfterJointUpdate")
    event("AfterContactUpdate")
    event("AfterContactWarmstart")
    event("BeforeJointSolve", 0)
    visits()
    event("AfterJointSolve", 0)
    velocity[0] = F32().add(velocity[0], f32_bits(.5))
    event("AfterContactNormal", 0)
    velocity[1] = F32().subtract(velocity[1], f32_bits(.25))
    event("AfterContactFriction", 0)
    event("AfterIntegration")
    event("BeforeJointStabilization", 0)
    visits(True)
    event("AfterJointStabilization", 0)
    event("AfterContactStabilization", 0)
    event("AfterStabilizationFriction")
    event("FinalSlice")
    native_parameters = (b"".join(parameters[name].to_bytes(4, "little") for name in PARAMETER_WORDS) +
                         b"".join(varint(parameters[name]) for name in PARAMETER_COUNTS))
    handles = [20, 21, 22, 30, 31, 32]
    records = [{"kind": kind, "handle_words": [index, 7], "postcard_bytes": [42, index]}
               for kind, indices in [("Body", handles), ("Collider", [0, 1, 2, 3, 4]), ("Multibody", [6])]
               for index in indices]
    bindings = [{"entity_bits": 700 + i, "joint_handle_words": [27 + i, 7],
                 "multibody_handle_words": [6, 7], "link_index": i - 2} for i in [3, 4, 5]]
    checkpoint = {"schema_version": 1, "native_scalar_bits": 32, "gravity_ieee_words": [0, f32_bits(-9.81), 0],
                  "integration_parameters": parameters, "integration_parameters_postcard_bytes": list(native_parameters),
                  "records": records, "bindings": bindings}
    ecs = [1, 6]
    for i in range(6):
        pose = [f64_bits(0)] * 6 + [f64_bits(1)] * 4
        body = [1 if i in [0, 2] else 0, f64_bits(.2 if i >= 4 else 5 if i == 1 else 1)] + [0] * 6
        ecs.extend([700 + i, 1] + pose + [1] + body + ([2, 0, 0] if i >= 3 else [0]))
    telemetry = {"numerical_outer_interval_word": h, "native_scalar_bits": 32,
                 "ccd_partition_duration_words": [h], "events": events}
    frame = {"kind": "completed_step", "schema_version": 1, "completed_outer_step": 1,
             "native_checkpoint": checkpoint, "native_checkpoint_postcard_bytes": list(encode_checkpoint(checkpoint, protocol["resource_ceilings"])),
             "ecs_physics_ieee_words": ecs, "stage_telemetry": telemetry}
    return protocol, frame


def analyze(protocol, frame, build="baseline"):
    entities = validate_step(frame, 1, True, protocol)
    details = []
    analysis = Analysis(build, details.append)
    analyze_telemetry(frame["stage_telemetry"], frame["native_checkpoint"], entities, 1, protocol, analysis)
    return analysis, details


class StageReaderTests(unittest.TestCase):
    def test_all_source_builder_permutations(self):
        for build in ("baseline", "rejected_inverse_w_plus_g"):
            for order in itertools.permutations(ROLES):
                with self.subTest(build=build, order=order):
                    p, f = fixture(order, build)
                    a, details = analyze(p, f, build)
                    result = a.summary()
                    self.assertTrue(result["complete_native_replay"])
                    self.assertTrue(result["complete_finite_valid_reference"])
                    self.assertEqual(result["counts"]["immediate_motor_visits:qualified_match"], 6)
                    self.assertEqual(result["counts"]["contact_row_intervals"], 15)
                    for role in ROLES:
                        self.assertEqual(result["roles"][role]["counts"]["immediate_motor_visits:qualified_match"], 2)
                    for role in ROLES[1:]:
                        self.assertEqual(result["roles"][role]["maxima"]["reference_projected_residual"]["absolute_magnitude"]["numerator"], "0")
                        self.assertEqual(result["roles"][role]["counts"]["zero_authority_cache_generations"], 1)
                    self.assertEqual(sum(d["kind"] == "row_availability" and d["status"] == "uninitialized" for d in details), 2)
                    self.assertTrue(result["counts"]["old_cached_row_after_integration_events"] > 0)

    def test_each_gate_fails_closed(self):
        p, original = fixture()
        def row_all(frame, field, value, handle=30):
            for event in frame["stage_telemetry"]["events"]:
                for row in event["rows"]:
                    if row["joint_handle"]["index"] == handle:
                        row[field] = copy.deepcopy(value)
        mutations = {
            "wrong_axis": lambda f: row_all(f,"axis","LinY"),
            "wrong_model": lambda f: row_all(f,"gain_model","ForceBased",31),
            "unknown_handle": lambda f: row_all(f,"joint_handle",{"index":99,"generation":7}),
            "array_handle_fallback": lambda f: row_all(f,"joint_handle",[30,7]),
            "bool_handle": lambda f: row_all(f,"joint_handle",{"index":True,"generation":7}),
            "negative_handle": lambda f: row_all(f,"joint_handle",{"index":-1,"generation":7}),
            "overflow_handle": lambda f: row_all(f,"joint_handle",{"index":1<<32,"generation":7}),
            "wrong_link": lambda f: row_all(f,"link_id",3),
            "wrong_selector": lambda f: row_all(f,"jacobian_words",[0,ONE,0]),
            "wrong_offset": lambda f: row_all(f,"solver_velocity_offset",1),
            "wrong_ndofs": lambda f: row_all(f,"ndofs2",2),
            "wrong_buffer_start": lambda f: row_all(f,"j_id2",6),
            "wrong_constraint_index": lambda f: row_all(f,"constraint_index",1),
            "finger_authority": lambda f: row_all(f,"effort_ceiling_word",ONE,31),
            "finger_nonzero_lambda": lambda f: row_all(f,"impulse_word",ONE,31),
            "zero_bound_signed_word_drift": lambda f: row_all(f,"impulse_bound_words",[0,0],31),
            "weighted_shape": lambda f: row_all(f,"weighted_jacobian_words",[ONE]),
            "wrong_diagonal": lambda f: row_all(f,"inverse_lhs_word",ONE),
            "extra_row_field": lambda f: f["stage_telemetry"]["events"][2]["rows"][0].update(extra=0),
            "missing_finger_row": lambda f: f["stage_telemetry"]["events"][2]["rows"].pop(),
            "duplicate_row": lambda f: f["stage_telemetry"]["events"][2]["rows"].__setitem__(2,copy.deepcopy(f["stage_telemetry"]["events"][2]["rows"][1])),
            "uninitialized_row": lambda f: f["stage_telemetry"]["events"][0].__setitem__("rows",f["stage_telemetry"]["events"][2]["rows"]),
            "old_generation": lambda f: f["stage_telemetry"]["events"][-1].__setitem__("row_cache_slice_index",1),
            "missing_phase": lambda f: f["stage_telemetry"]["events"].pop(4),
            "duplicate_phase": lambda f: f["stage_telemetry"]["events"].insert(5,copy.deepcopy(f["stage_telemetry"]["events"][4])),
            "missing_after_visit": lambda f: f["stage_telemetry"]["events"].pop(7),
            "wrong_visit": lambda f: f["stage_telemetry"]["events"][6].__setitem__("joint_visit_index",1),
            "wrong_partition": lambda f: f["stage_telemetry"]["events"][0].__setitem__("ccd_partition_index",1),
            "wrong_slice_count": lambda f: f["stage_telemetry"]["events"][0].__setitem__("expected_internal_slice_count",2),
            "empty_vector_with_rows": lambda f: f["stage_telemetry"]["events"][2].__setitem__("generalized_velocity_words",[]),
            "unknown_binding_entity": lambda f: f["native_checkpoint"]["bindings"][0].__setitem__("entity_bits",999),
            "binding_alias": lambda f: f["native_checkpoint"]["bindings"][1].__setitem__("joint_handle_words",[30,7]),
            "new_child_mass": lambda f: f["ecs_physics_ieee_words"].__setitem__(f["ecs_physics_ieee_words"].index(f64_bits(.2)),f64_bits(.3)),
            "postcard_tamper": lambda f: f["native_checkpoint_postcard_bytes"].__setitem__(5,0),
            "wrong_outer_step": lambda f: f.__setitem__("completed_outer_step",True),
        }
        for name, mutate in mutations.items():
            with self.subTest(name=name):
                altered = copy.deepcopy(original); mutate(altered)
                with self.assertRaises((ValueError, TypeError, KeyError, IndexError, AssertionError)):
                    analyze(p, altered)

    def test_identity_drift_and_optional_modes(self):
        p,f=fixture(); a,_=analyze(p,f)
        entities=validate_step(f,1,True,p)
        changed=copy.deepcopy(f["native_checkpoint"])
        changed["bindings"][1]["joint_handle_words"]=[35,7]
        with self.assertRaises(ValueError): a.roles.admit_checkpoint(changed,entities)
        self.assertEqual(validate_step(dict(f,stage_telemetry=None),1,False,p),entities)
        with self.assertRaises(ValueError): validate_step(f,1,False,p)

    def test_literal_numeric_mismatch_is_not_pass(self):
        p,f=fixture()
        f["stage_telemetry"]["events"][7]["generalized_velocity_words"][0]=ONE
        a,_=analyze(p,f)
        self.assertGreater(a.issues,0)
        self.assertFalse(a.summary()["complete_native_replay"])

    def test_native_exclusion_is_not_complete_replay(self):
        p,f=fixture()
        # A tiny but finite weighted term outside the replay domain. Keep it
        # identical throughout each cached row so this tests eligibility, not
        # a cache-field mutation or a native simulation.
        for event in f["stage_telemetry"]["events"]:
            for row in event["rows"]:
                if row["joint_handle"]["index"]==30: row["weighted_jacobian_words"][2]=1
        a,_=analyze(p,f)
        self.assertFalse(a.summary()["complete_native_replay"])
        self.assertGreater(a.counts["immediate_motor_visits:unqualified"],0)

    def test_framing_crc_truncation_and_strict_numbers(self):
        for data in (b'{"a":1,"a":2}',b'{"a":NaN}',b'{"a":1.0}',b'{"a":"\xff"}'):
            with self.assertRaises((ValueError,UnicodeError)): strict_json(data)
        p,_=fixture()
        raw=b''.join(json.dumps({'ordinal':i}).encode()+b'\n' for i in range(1252))
        packed=gzip.compress(raw,mtime=0);packed=packed[:9]+b'\xff'+packed[10:]
        with tempfile.TemporaryDirectory(prefix='rne-gen5-pure-',dir='/tmp') as temp:
            temp=Path(temp)
            def read(data,ceilings=None):
                path=temp/(str(len(list(temp.iterdir())))+'.gz');path.write_bytes(data)
                stream=GzipJsonLines(path,ceilings or p['resource_ceilings']); records=list(stream)
                return stream,records
            stream,records=read(packed)
            self.assertTrue(stream.complete);self.assertEqual(len(records),1252)
            damaged=bytearray(packed);damaged[-8]^=1
            for data in (packed[:-3],bytes(damaged),packed+packed,packed+b'suffix'):
                with self.assertRaises((ValueError,__import__('zlib').error)): read(data)
            tail=gzip.compress(raw[:-1],mtime=0);tail=tail[:9]+b'\xff'+tail[10:]
            with self.assertRaises(ValueError): read(tail)
            with self.assertRaises(ValueError): read(packed,dict(p['resource_ceilings'],json_line_bytes=4))

    def test_complete_framing_counters_and_conditions(self):
        cell={'run_label':'baseline-m5-on','mass_kg':5,'mode':'on'}
        start={'kind':'run_start','schema_version':1,'run_label':cell['run_label'],
               'mass_kg_ieee754_bits':f64_bits(5),'observer_enabled':True,
               'expected_completed_outer_steps':1250,'expected_ecs_entity_count':6,
               'ecs_word_schema_version':1,'checkpoint_codec':'postcard-1.1.3-raw-native',
               'checkpoint_scope':CHECKPOINT_SCOPE,'declared_conditions':expected_conditions(5)}
        validate_start(start,cell)
        changed=copy.deepcopy(start);changed['declared_conditions']['settle_steps']=251
        with self.assertRaises(ValueError): validate_start(changed,cell)
        end={'kind':'run_end','schema_version':1,'status':'collected','completed_outer_steps':1250,
             'frames_written':1250,'error':None,'physical_report':{'cube_ecs_pose_ieee_words':[0]*10,
             'physical_ieee_words':[0]*4,'physical_stdout_row':'synthetic'}}
        validate_end(end)
        for change in ({'frames_written':1249},{'status':'failed','error':'preserved'}, {'schema_version':True}):
            with self.assertRaises(ValueError): validate_end(dict(end,**change))

if __name__=='__main__': unittest.main(verbosity=2)
