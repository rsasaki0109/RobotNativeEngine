#!/usr/bin/env python3
"""Phase-qualified aggregate analysis; no per-contact-column attribution."""

from collections import Counter
from fractions import Fraction

from stage_numeric import F32, ONE, exact, exact_snapshot, rational, rounded, serialize_snapshot
from stage_schema import (keys, literal_equal, require, sha_bytes, validate_event,
                          words, f32_bits, native_joint_handle_words)
import json
from stage_roles import Roles, ROLES


def identity(row):
    return native_joint_handle_words(row["joint_handle"]) + (row["link_id"], row["axis"], row["constraint_index"])


def difference(left, right):
    return [name for name in sorted(set(left) | set(right)) if not literal_equal(left.get(name), right.get(name))]


class Analysis:
    def __init__(self, build, sink):
        self.build = build
        self.sink = sink
        self.counts = Counter()
        self.maximum = {}
        self.issues = 0
        self.first_issue = None
        self.roles = Roles()
        self.role_counts = {role: Counter() for role in ROLES}
        self.role_maximum = {role: {} for role in ROLES}

    def emit(self, record):
        self.sink(record)

    def issue(self, context, reason, observed=None):
        record = {"kind": "qualification_issue", "context": context, "reason": reason}
        if observed is not None:
            record["observed"] = observed
        self.issues += 1
        if "role" in context:
            self.role_counts[context["role"]]["qualification_issues"] += 1
        if self.first_issue is None:
            self.first_issue = record
        self.emit(record)

    def role_context(self, row, context):
        role = self.roles.role(row)
        binding = self.roles.frozen_identity[role]
        return dict(context, role=role, entity_bits=binding["entity_bits"],
                    zero_authority=(role != "palm"))

    def distribute_counts(self, role, previous):
        for name, count in self.counts.items():
            self.role_counts[role][name] += count - previous.get(name, 0)

    def track(self, name, value, context):
        magnitude = abs(value)
        if name not in self.maximum or magnitude > self.maximum[name][0]:
            self.maximum[name] = (magnitude, value, dict(context))
        if "role" in context:
            maxima = self.role_maximum[context["role"]]
            if name not in maxima or magnitude > maxima[name][0]:
                maxima[name] = (magnitude, value, dict(context))

    def snapshot(self, event, row, context):
        context = self.role_context(row, context)
        previous_counts = dict(self.counts)
        self.counts["row_snapshots"] += 1
        raw = json.dumps(row, separators=(",", ":"), sort_keys=True).encode()
        record = {"kind": "row_snapshot", "context": context, "row_identity": list(identity(row)),
                  "cached_row_sha256": sha_bytes(raw), "cache_after_integration_is_retained_not_rebuilt":
                  context["after_integration"]}
        reference = native = None
        try:
            reference = exact_snapshot(row, event["generalized_velocity_words"])
            self.counts["finite_valid_reference_snapshots"] += 1
            self.counts["bound_position:" + reference["position"]] += 1
            self.counts["complementarity:" + reference["complementarity"]] += 1
            self.track("reference_projected_residual", reference["projection"], context)
            self.track("exact_velocity_residual", reference["residual"], context)
            record["reference"] = serialize_snapshot(reference)
        except ValueError as error:
            self.counts["reference_unavailable_snapshots"] += 1
            record["reference"] = {"status": "unavailable", "reason": str(error)}
        try:
            native = F32().projection(row, event["generalized_velocity_words"], include_axpy=False)
            record["native_projection"] = native
            self.counts["native_projection:" + native["eligibility"]] += 1
            for exclusion in native["exclusions"]:
                self.counts["native_exclusion:" + exclusion] += 1
            if native["eligibility"] == "qualified":
                projection = exact(row["impulse_word"]) - exact(native["next_impulse_word"])
                record["native_projected_residual_exact_from_rounded_words"] = rational(projection)
                self.track("native_projected_residual", projection, context)
                if reference is not None:
                    rounding_error = exact(native["residual_word"]) - reference["residual"]
                    record["native_residual_rounding_error"] = rational(rounding_error)
                    self.track("native_residual_rounding_error", rounding_error, context)
        except ValueError as error:
            self.counts["native_projection:unavailable"] += 1
            record["native_projection"] = {"eligibility": "unavailable", "reason": str(error)}
        self.emit(record)
        self.distribute_counts(context["role"], previous_counts)
        return reference, native

    def contact(self, before, after, category, context):
        self.counts["aggregate_contact_boundaries"] += 1
        self.counts["contact_category:" + category] += 1
        before_rows = {identity(row): row for row in before["rows"]}
        after_rows = {identity(row): row for row in after["rows"]}
        if not before_rows and not after_rows:
            self.counts["contact_intervals:no_supported_rows"] += 1
            self.emit({"kind": "contact_interval", "context": context, "category": category,
                       "eligibility": "unavailable", "reason": "no_supported_motor_rows"})
            return
        for row_id in sorted(set(before_rows) | set(after_rows)):
            context = self.role_context(before_rows.get(row_id, after_rows.get(row_id)), context)
            previous_counts = dict(self.counts)
            self.counts["contact_row_intervals"] += 1
            record = {"kind": "contact_interval", "context": context, "category": category,
                      "row_identity": list(row_id)}
            reasons = []
            if row_id not in before_rows or row_id not in after_rows:
                reasons.append("row_missing_at_endpoint")
            context_fields = ["ccd_partition_index", "island_index", "internal_slice_index",
                              "row_cache_slice_index", "native_scalar_bits", "duration_word"]
            reasons.extend("context_changed:" + name for name in context_fields if before[name] != after[name])
            if not reasons:
                changed = difference(before_rows[row_id], after_rows[row_id])
                reasons.extend("row_field_changed:" + name for name in changed)
            if reasons:
                record.update(eligibility="ineligible", reasons=reasons)
                self.counts["contact_intervals:field_or_cache_changed"] += 1
                self.issue(context, "contact-only stage changed motor row identity/cache/words", reasons)
                self.emit(record)
                self.distribute_counts(context["role"], previous_counts)
                continue
            row = before_rows[row_id]
            try:
                first = exact_snapshot(row, before["generalized_velocity_words"])
                second = exact_snapshot(row, after["generalized_velocity_words"])
                offset, length = row["solver_velocity_offset"], row["ndofs2"]
                delta_velocity = [exact(right) - exact(left) for left, right in zip(
                    before["generalized_velocity_words"][offset:offset + length],
                    after["generalized_velocity_words"][offset:offset + length])]
                j_delta_v = sum((exact(j) * dv for j, dv in zip(row["jacobian_words"], delta_velocity)), Fraction())
                delta_r = second["residual"] - first["residual"]
                record.update(eligibility="finite_fixed_cached_row", delta_velocity=[rational(value) for value in delta_velocity],
                              exact_delta_r=rational(delta_r), exact_j_delta_v=rational(j_delta_v),
                              exact_identity=True, before_reference=serialize_snapshot(first), after_reference=serialize_snapshot(second))
                self.counts["contact_intervals:finite_fixed_cached_row"] += 1
                self.track("contact_exact_delta_r", delta_r, context)
                self._change(record, "reference", first["projection"], second["projection"], context)
            except ValueError as error:
                record.update(eligibility="unavailable", reason=str(error))
                self.counts["contact_intervals:reference_unavailable"] += 1
                self.emit(record)
                self.distribute_counts(context["role"], previous_counts)
                continue
            require(delta_r == j_delta_v, "exact contact delta-r identity failed")
            try:
                native_before = F32().projection(row, before["generalized_velocity_words"], include_axpy=False)
                native_after = F32().projection(row, after["generalized_velocity_words"], include_axpy=False)
                record["native_before"] = native_before
                record["native_after"] = native_after
                if native_before["eligibility"] == native_after["eligibility"] == "qualified":
                    e_before = exact(native_before["residual_word"]) - first["residual"]
                    e_after = exact(native_after["residual_word"]) - second["residual"]
                    native_delta = exact(native_after["residual_word"]) - exact(native_before["residual_word"])
                    discrepancy = native_delta - j_delta_v
                    if discrepancy != e_after - e_before:
                        raise AssertionError("native endpoint rounding accounting failed")
                    record["native_rounding_accounting"] = {"before_error": rational(e_before), "after_error": rational(e_after),
                        "native_residual_delta": rational(native_delta), "exact_discrepancy": rational(discrepancy), "identity": True}
                    self.counts["contact_intervals:native_qualified"] += 1
                    self.track("contact_native_rounding_discrepancy", discrepancy, context)
                    self._change(record, "native", exact(row["impulse_word"]) - exact(native_before["next_impulse_word"]),
                                 exact(row["impulse_word"]) - exact(native_after["next_impulse_word"]), context)
                else:
                    self.counts["contact_intervals:native_unqualified"] += 1
            except ValueError as error:
                record["native_rounding_accounting"] = {"status": "unavailable", "reason": str(error)}
                self.counts["contact_intervals:native_unavailable"] += 1
            self.emit(record)
            self.distribute_counts(context["role"], previous_counts)

    def _change(self, record, name, before, after, context):
        reopened = before == 0 and after != 0
        change = "reopened" if reopened else "both_zero" if before == after == 0 else (
            "increased_magnitude" if abs(after) > abs(before) else "decreased_magnitude" if abs(after) < abs(before) else
            "equal_value" if after == before else "equal_magnitude_different_sign")
        record[name + "_projection_change"] = {"before": rational(before), "after": rational(after),
                                                  "strict_reopened": reopened, "classification": change}
        self.counts[name + "_contact_projection:" + change] += 1
        self.track(name + "_contact_projection_change", after - before, context)

    def immediate(self, before, after, context):
        first, second = before["rows"][0], after["rows"][0]
        context = self.role_context(first, context)
        previous_counts = dict(self.counts)
        self.counts["immediate_motor_visits"] += 1
        changes = difference(first, second)
        require(set(changes) <= {"impulse_word"}, "selected motor solve changed fields beyond lambda")
        for field in ["ccd_partition_index", "island_index", "internal_slice_index", "row_cache_slice_index",
                      "iteration_index", "joint_visit_index", "duration_word", "native_scalar_bits"]:
            require(before[field] == after[field], "motor pair metadata differs: " + field)
        record = {"kind": "immediate_motor_visit", "context": context, "row_identity": list(identity(first))}
        try:
            prediction = F32().projection(first, before["generalized_velocity_words"])
            record["prediction"] = prediction
            if prediction["eligibility"] == "qualified":
                record["impulse_word_match"] = prediction["next_impulse_word"] == second["impulse_word"]
                record["full_generic_velocity_word_match"] = prediction["next_generalized_velocity_words"] == after["generalized_velocity_words"]
                if not record["impulse_word_match"] or not record["full_generic_velocity_word_match"]:
                    self.counts["immediate_motor_visits:mismatch"] += 1
                    self.issue(context, "qualified native immediate motor replay word mismatch", record)
                else:
                    self.counts["immediate_motor_visits:qualified_match"] += 1
            else:
                self.counts["immediate_motor_visits:unqualified"] += 1
        except ValueError as error:
            record["prediction"] = {"eligibility": "unavailable", "reason": str(error)}
            self.counts["immediate_motor_visits:unavailable"] += 1
        self.emit(record)
        self.distribute_counts(context["role"], previous_counts)

    def rebuilt(self, event, context):
        for row in event["rows"]:
            role = self.roles.role(row)
            previous_counts = dict(self.counts)
            require(row["impulse_word"] == 0, "rebuilt motor accumulator is not literal +0")
            require(row["rhs_word"] == row["rhs_without_bias_word"], "unit motor initialized RHS differs from bias-free RHS")
            coordinate = row["link_id"] - 1
            require(row["cfm_coefficient_word"] == 0, "registered fixed motor coefficient differs")
            coefficients = F32()
            h = event["duration_word"]
            if role == "palm":
                ceiling = f32_bits(5000)
                gain = coefficients.inverse(coefficients.add(
                    coefficients.multiply(coefficients.multiply(h, h), f32_bits(1e5)),
                    coefficients.multiply(h, f32_bits(1e4))))
            else:
                ceiling = 0
                gain = 0  # AccelerationBased k=d=0 invokes guarded inverse(0).
            bound = coefficients.multiply(ceiling, h)
            require(row["effort_ceiling_word"] == ceiling and row["cfm_gain_word"] == gain and
                    row["impulse_bound_words"] == [bound ^ 0x80000000, bound],
                    "stored native role gain/bounds differ from fixed source expressions: " + role)
            require(not coefficients.exclusions, "fixed registered duration/coefficient expression left native replay domain")
            # Finger RHS is deliberately not forced to zero: source limit
            # clamping can make the zero-target-velocity RHS nonzero.
            native = F32()
            lhs = row["weighted_jacobian_words"][coordinate]
            if self.build == "rejected_inverse_w_plus_g" and role == "palm":
                lhs = native.add(lhs, row["cfm_gain_word"])
            try:
                expected = native.inverse(lhs)
            except ValueError:
                self.counts["rebuilt_diagonal:unavailable"] += 1
            else:
                if not native.exclusions:
                    require(expected == row["inverse_lhs_word"], "stored D differs from source-bound registered build diagonal")
                    self.counts["rebuilt_diagonal:qualified_match"] += 1
                else:
                    self.counts["rebuilt_diagonal:unqualified"] += 1
            self.counts["initialized_motor_cache_generations"] += 1
            if role != "palm":
                self.counts["zero_authority_cache_generations"] += 1
            self.distribute_counts(role, previous_counts)

    def summary(self):
        def maxima(values):
            return {name: {"absolute_magnitude": rational(value[0]), "signed_value": rational(value[1]),
                           "first_context": value[2]} for name, value in sorted(values.items())}
        def native_complete(counts):
            return (counts["immediate_motor_visits"] > 0 and
                    counts["immediate_motor_visits"] == counts["immediate_motor_visits:qualified_match"] and
                    counts["row_snapshots"] == counts["native_projection:qualified"] and
                    counts["initialized_motor_cache_generations"] == counts["rebuilt_diagonal:qualified_match"] and
                    counts["contact_row_intervals"] == counts["contact_intervals:native_qualified"])
        def reference_complete(counts):
            return (counts["row_snapshots"] > 0 and
                    counts["row_snapshots"] == counts["finite_valid_reference_snapshots"] and
                    counts["contact_row_intervals"] == counts["contact_intervals:finite_fixed_cached_row"])
        roles = {role: {"counts": dict(sorted(self.role_counts[role].items())),
                        "maxima": maxima(self.role_maximum[role]),
                        "complete_native_replay": self.role_counts[role]["qualification_issues"] == 0 and native_complete(self.role_counts[role]),
                        "complete_finite_valid_reference": reference_complete(self.role_counts[role]),
                        "authority": "controlled" if role == "palm" else "zero_authority"} for role in ROLES}
        return {"counts": dict(sorted(self.counts.items())), "qualification_issues": self.issues,
                "first_qualification_issue": self.first_issue, "maxima": maxima(self.maximum),
                "roles": roles, "role_identity_scope": self.roles.summary(),
                "complete_native_replay": self.issues == 0 and native_complete(self.counts) and
                    all(role["complete_native_replay"] for role in roles.values()),
                "complete_finite_valid_reference": reference_complete(self.counts) and
                    all(role["complete_finite_valid_reference"] for role in roles.values()),
                "unmeasured": ["per-contact J/W columns", "individual contact impulses", "J_m W_c",
                               "full set metadata", "private pipeline/solver/contact caches", "runtime MXCSR register",
                               "independent native per-object deserialization", "anatomical finger frame labels"]}


def analyze_telemetry(telemetry, checkpoint, ecs_entities, step, protocol, analysis):
    keys(telemetry, ["numerical_outer_interval_word", "native_scalar_bits", "ccd_partition_duration_words", "events"], "telemetry")
    require(telemetry["native_scalar_bits"] == 32 and type(telemetry["native_scalar_bits"]) is int, "telemetry native width changed")
    parameters = checkpoint["integration_parameters"]
    require(telemetry["numerical_outer_interval_word"] == parameters["dt_word"], "telemetry outer duration differs from native runtime parameters")
    partitions = words(telemetry["ccd_partition_duration_words"], 32,
                       maximum=protocol["resource_ceilings"]["ccd_partitions_per_step"])
    require(partitions == [parameters["dt_word"]] and parameters["max_ccd_substeps"] == 1,
            "fixed no-CCD fixture partition duration/count differs")
    require(all(exact(word) > 0 for word in partitions), "nonpositive partition duration")
    events = telemetry["events"]
    ceilings = protocol["resource_ceilings"]
    require(type(events) is list and len(events) <= ceilings["events_per_step"], "stage event resource ceiling")
    require(events, "ON step has no stage events")
    analysis.roles.admit_checkpoint(checkpoint, ecs_entities)
    seen_slices, groups = {}, []
    current = None
    for ordinal, event in enumerate(events):
        validate_event(event, partitions, ceilings)
        group_key = (event["ccd_partition_index"], event["island_index"], event["internal_slice_index"])
        if group_key != current:
            require(current is None or current < group_key, "partition/island/slice event order is not strictly grouped")
            groups.append((group_key, [])); current = group_key
        groups[-1][1].append((ordinal, event))
        for row in event["rows"]:
            analysis.roles.admit_row(row, len(event["generalized_velocity_words"]))
    motor_slices = 0
    motor_islands = {}
    for group_key, group in groups:
        partition, island, slice_index = group_key
        first = group[0][1]
        expected_slices = first["expected_internal_slice_count"]
        group_context = {"completed_outer_step": step, "partition": partition, "island": island, "slice": slice_index}
        require(all(event["expected_internal_slice_count"] == expected_slices and event["duration_word"] == first["duration_word"]
                    for _, event in group), "slice metadata changed within a group")
        require(expected_slices == parameters["num_solver_iterations"], "fixed fixture slice count differs from configured base")
        expected_duration = F32().divide(partitions[partition], rounded(Fraction(expected_slices)))
        require(first["duration_word"] == expected_duration, "native small-step duration disagrees with actual partition/count")
        island_key = (partition, island)
        if island_key not in seen_slices:
            require(slice_index == 0, "island begins at nonzero slice")
            seen_slices[island_key] = (0, expected_slices)
        last_slice, same_expected = seen_slices[island_key]
        require(slice_index == last_slice and expected_slices == same_expected, "missing/duplicate island slice")
        seen_slices[island_key] = (last_slice + 1, expected_slices)
        _group_grammar(group, group_context, parameters, analysis)
        analysis.counts["island_slices"] += 1
        if group[2][1]["rows"]:
            motor_slices += 1
            require(partition not in motor_islands or motor_islands[partition] == island,
                    "one multibody motor group appeared in multiple islands")
            motor_islands[partition] = island
        after_integration, pass_kind = False, None
        for ordinal, event in group:
            if event["stage"] == "AfterIntegration":
                after_integration = True
            if event["stage"] == "BeforeJointSolve":
                pass_kind = "pgs"
            elif event["stage"] == "BeforeJointStabilization":
                pass_kind = "stabilization"
            context = dict(group_context, event_ordinal=ordinal, stage=event["stage"], iteration=event["iteration_index"],
                           visit=event["joint_visit_index"], after_integration=after_integration, pass_kind=pass_kind)
            if after_integration and event["rows"]:
                analysis.counts["old_cached_row_after_integration_events"] += 1
            analysis.counts["stage:" + event["stage"]] += 1
            analysis.counts["events"] += 1
            if not event["rows"]:
                analysis.counts["events:no_supported_rows"] += 1
                analysis.emit({"kind": "row_availability", "context": context, "status": "uninitialized" if event["row_cache_slice_index"] is None else "no_supported_rows"})
            for row in event["rows"]:
                analysis.snapshot(event, row, context)
            if event["stage"] in ["AfterJointSolve", "AfterJointStabilization"]:
                pass_kind = None
    for island_key, (observed, expected) in seen_slices.items():
        require(observed == expected, "island slice coverage incomplete")
    for partition in range(len(partitions)):
        island_ids = sorted(island for p, island in seen_slices if p == partition)
        require(island_ids and island_ids == list(range(len(island_ids))), "partition island coverage/order missing")
    require(set(motor_islands) == set(range(len(partitions))) and
            motor_slices == len(partitions) * parameters["num_solver_iterations"],
            "every partition/slice must expose all three supported internal motors exactly once")
    analysis.counts["completed_outer_steps"] += 1
    analysis.counts["ccd_partitions"] += len(partitions)
    analysis.counts["active_island_partitions"] += len(seen_slices)
    analysis.counts["selected_motor_slices"] += motor_slices


def _group_grammar(group, context, parameters, analysis):
    cursor = 0
    def take(stage, iteration=None):
        nonlocal cursor
        require(cursor < len(group), "missing stage: " + stage)
        ordinal, event = group[cursor]; cursor += 1
        require(event["stage"] == stage and event["iteration_index"] == iteration, "unexpected phase/iteration: wanted " + stage)
        return event, dict(context, event_ordinal=ordinal, stage=stage, iteration=iteration, visit=event["joint_visit_index"])
    take("BeforeExternalForces")
    take("AfterExternalForces")
    rebuilt, rebuilt_context = take("AfterJointUpdate")
    analysis.rebuilt(rebuilt, rebuilt_context)
    actual_rows = [identity(row) for row in rebuilt["rows"]]
    analysis.roles.admit_aggregate(rebuilt["rows"], len(rebuilt["generalized_velocity_words"]))
    immutable_rows = {identity(row): {name: value for name, value in row.items() if name not in ["rhs_word", "impulse_word"]}
                      for row in rebuilt["rows"]}
    immediate_stages = {"BeforeMotorBiasRemoval", "BeforeMotorSolve", "AfterMotorSolve"}
    vector_length = len(rebuilt["generalized_velocity_words"])
    for _, event in group:
        require(len(event["generalized_velocity_words"]) == vector_length, "generic vector shape changed within island slice")
        if event["stage"] not in immediate_stages and event["row_cache_slice_index"] is not None:
            require([identity(row) for row in event["rows"]] == actual_rows, "aggregate supported role order/coverage changed")
    for _, event in group[2:]:
        for row in event["rows"]:
            require(identity(row) in immutable_rows and literal_equal(
                {name: value for name, value in row.items() if name not in ["rhs_word", "impulse_word"]},
                immutable_rows[identity(row)]), "cached motor fields changed without the registered joint rebuild")
    contact_update, _ = take("AfterContactUpdate")
    require(literal_equal(rebuilt["rows"], contact_update["rows"]) and rebuilt["generalized_velocity_words"] == contact_update["generalized_velocity_words"],
            "contact update changed initialized motor rows or generic velocities")
    last = contact_update
    if exact(parameters["warmstart_coefficient_word"]) != 0:
        last, ctx = take("AfterContactWarmstart")
        analysis.contact(contact_update, last, "contact_warmstart", ctx)
    def joint_pass(start_stage, end_stage, iteration, stabilization):
        nonlocal cursor
        beginning, _ = take(start_stage, iteration)
        require(literal_equal(beginning["rows"], last["rows"]) and
                beginning["generalized_velocity_words"] == last["generalized_velocity_words"],
                "joint-pass entry changed preceding observed motor rows or generic velocity")
        require([identity(row) for row in beginning["rows"]] == actual_rows, "aggregate supported row list changed during slice")
        observed, visits, last_impulses = [], [], {}
        beginning_rows = {identity(row): row for row in beginning["rows"]}
        while cursor < len(group) and group[cursor][1]["stage"] in ["BeforeMotorBiasRemoval", "BeforeMotorSolve"]:
            if stabilization:
                biased, _ = take("BeforeMotorBiasRemoval", iteration)
                before, _ = take("BeforeMotorSolve", iteration)
                require(biased["generalized_velocity_words"] == before["generalized_velocity_words"], "bias removal changed generic velocity")
                expected = dict(biased["rows"][0], rhs_word=biased["rows"][0]["rhs_without_bias_word"])
                require(literal_equal(before["rows"][0], expected), "bias removal changed anything beyond RHS replacement")
                require(biased["joint_visit_index"] == before["joint_visit_index"], "bias removal/motor visit mismatch")
            else:
                before, _ = take("BeforeMotorSolve", iteration)
            after, ctx = take("AfterMotorSolve", iteration)
            ctx["pass_kind"] = "stabilization" if stabilization else "pgs"
            ctx["after_integration"] = stabilization
            selected = before["rows"][0]
            selected_id = identity(selected)
            require(selected_id in beginning_rows and selected["impulse_word"] == beginning_rows[selected_id]["impulse_word"],
                    "unvisited selected motor accumulator changed during joint pass")
            require(before["joint_visit_index"] == selected["constraint_index"],
                    "global visit differs from fixed fixture's sole generic-one-body constraint index")
            observed.append(selected_id); visits.append(before["joint_visit_index"])
            last_impulses[selected_id] = after["rows"][0]["impulse_word"]
            analysis.immediate(before, after, ctx)
        require(observed == actual_rows, "all internal motor visits must cover the literal aggregate builder order exactly")
        require(visits == sorted(set(visits)), "motor global visit order duplicated/reversed")
        ending, _ = take(end_stage, iteration)
        require([identity(row) for row in ending["rows"]] == actual_rows, "aggregate row identities changed after joint pass")
        require(all(row["impulse_word"] == last_impulses[identity(row)] for row in ending["rows"]),
                "final aggregate lambda differs from each selected motor's last visit")
        return ending
    for iteration in range(parameters["num_internal_pgs_iterations"]):
        last = joint_pass("BeforeJointSolve", "AfterJointSolve", iteration, False)
        normal, ctx = take("AfterContactNormal", iteration)
        analysis.contact(last, normal, "normal_restitution", ctx)
        friction, ctx = take("AfterContactFriction", iteration)
        analysis.contact(normal, friction, "friction", ctx)
        last = friction
    if context["slice"] == group[0][1]["expected_internal_slice_count"] - 1:
        for iteration in range(parameters["num_additional_friction_iterations"]):
            friction, ctx = take("AfterAdditionalFriction", iteration)
            analysis.contact(last, friction, "additional_final_slice_friction", ctx)
            last = friction
    integration, _ = take("AfterIntegration")
    require(literal_equal(last["rows"], integration["rows"]), "integration silently rebuilt or changed cached motor row")
    last = integration
    for iteration in range(parameters["num_internal_stabilization_iterations"]):
        last = joint_pass("BeforeJointStabilization", "AfterJointStabilization", iteration, True)
        normal, ctx = take("AfterContactStabilization", iteration)
        analysis.contact(last, normal, "stabilization_normal", ctx)
        last = normal
    if parameters["num_internal_stabilization_iterations"] > 0:
        friction, ctx = take("AfterStabilizationFriction")
        analysis.contact(last, friction, "final_stabilization_friction", ctx)
        last = friction
    final, _ = take("FinalSlice")
    require(literal_equal(final["rows"], last["rows"]) and final["generalized_velocity_words"] == last["generalized_velocity_words"], "FinalSlice changed last observed native state")
    require(cursor == len(group), "unexpected extra or reordered stage")
