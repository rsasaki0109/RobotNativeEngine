#!/usr/bin/env python3
"""Generation-five source-derived admission for every supported internal motor.

No runtime numeric entity or handle value is guessed. The typed checkpoint
binds entities to native handles/links; fixture body types/masses identify the
palm and two finger slots. Slot names refer to append/link order, not a decoded
native left/right frame. Native per-object payloads remain source-bound opaque
bytes, with the original declared checkpoint exclusions.
"""

from stage_numeric import ONE, exact
from stage_schema import (f64_bits, literal_equal, native_joint_handle_words,
                          require)

ROLES = ("palm", "finger_slot_0", "finger_slot_1")
MODEL = {"palm": "ForceBased", "finger_slot_0": "AccelerationBased",
         "finger_slot_1": "AccelerationBased"}


class Roles:
    def __init__(self):
        self.frozen_identity = None
        self.by_handle = None

    def admit_checkpoint(self, checkpoint, entities):
        """Freeze identities from source roles once, require them on every step."""
        require(len(entities) == 6, "fixed six-entity role coverage differs")
        # parse_ecs already enforces the source's sorted fixed/dynamic/tag layout.
        children = [entity for entity in entities if entity["joint_tag"] == 2]
        require(len(children) == 3 and all(entity["body"][0] == 0 for entity in children),
                "all three prismatic children must remain dynamic")
        palms = [entity for entity in children if entity["body"][1] == f64_bits(1)]
        fingers = [entity for entity in children if entity["body"][1] == f64_bits(.2)]
        require(len(palms) == 1 and len(fingers) == 2,
                "fixture child masses do not uniquely identify palm and finger pair")
        bindings = checkpoint["bindings"]
        require(len(bindings) == 3, "all three native bindings are required")
        by_entity = {binding["entity_bits"]: binding for binding in bindings}
        require(len(by_entity) == 3 and set(by_entity) == {entity["entity_bits"] for entity in children},
                "binding entities differ from all three prismatic children")
        require(len({tuple(binding["joint_handle_words"]) for binding in bindings}) == 3,
                "native motor handle aliases")
        require(len({tuple(binding["multibody_handle_words"]) for binding in bindings}) == 1,
                "fixture child bindings do not share the sole multibody")
        palm = by_entity[palms[0]["entity_bits"]]
        require(palm["link_index"] == 1, "fixed-base palm must be first appended child link")
        finger_bindings = sorted((by_entity[entity["entity_bits"]] for entity in fingers),
                                 key=lambda binding: binding["link_index"])
        require([binding["link_index"] for binding in finger_bindings] == [2, 3],
                "finger pair append/link coverage must be exactly two and three")
        ordered = [palm] + finger_bindings
        identity = {role: dict(binding, joint_handle_words=list(binding["joint_handle_words"]),
                               multibody_handle_words=list(binding["multibody_handle_words"]))
                    for role, binding in zip(ROLES, ordered)}
        if self.frozen_identity is None:
            self.frozen_identity = identity
            self.by_handle = {tuple(binding["joint_handle_words"]): role
                              for role, binding in identity.items()}
        else:
            require(literal_equal(self.frozen_identity, identity),
                    "source-role native/entity identity changed during collection")
        return identity

    def admit_row(self, row, vector_length):
        require(self.by_handle is not None, "row observed before checkpoint role admission")
        handle = native_joint_handle_words(row["joint_handle"])
        require(handle in self.by_handle, "unexpected supported native internal motor handle")
        role = self.by_handle[handle]
        binding = self.frozen_identity[role]
        require(row["link_id"] == binding["link_index"], "native row/binding link differs")
        require(row["axis"] == "LinX" and row["gain_model"] == MODEL[role],
                "native joint-frame axis or source-derived gain model differs: " + role)
        require(vector_length == row["ndofs2"] == 3 and row["solver_velocity_offset"] == 0,
                "sole fixed-base gripper requires the full three-coordinate generic velocity block")
        coordinate = binding["link_index"] - 1
        require(row["jacobian_words"] == [ONE if index == coordinate else 0 for index in range(3)],
                "unit motor stored J does not match link assembly coordinate: " + role)
        require(row["rhs_word"] == row["rhs_without_bias_word"],
                "internal Motor(0) cached RHS must equal its source bias-free RHS")
        # Accumulator is always finite and feasible, including signed zero caps.
        from stage_numeric import bound_compare
        require(bound_compare(row["impulse_bound_words"][0], row["impulse_word"]) <= 0 and
                bound_compare(row["impulse_word"], row["impulse_bound_words"][1]) <= 0,
                "stored native motor accumulator is outside registered bounds")
        require(exact(row["impulse_word"]) == 0 if role != "palm" else True,
                "zero-authority finger accumulated a nonzero impulse")
        return role

    def admit_aggregate(self, rows, vector_length):
        require((vector_length, len(rows)) in [(0, 0), (3, 3)],
                "initialized island must expose zero rows or every one of the three motors")
        if not rows:
            return []
        observed = [self.admit_row(row, vector_length) for row in rows]
        require(set(observed) == set(ROLES) and len(observed) == 3,
                "aggregate role coverage duplicated or missing")
        # Builder traversal follows actual active-island ordering. Each finger
        # reserves motor then limit; palm reserves motor only. No impulse joints
        # means all preceding ordinary/generic-two-body joint vectors are empty.
        constraint = 0
        for row, role in zip(rows, observed):
            require(row["constraint_index"] == constraint and row["j_id2"] == 6 * constraint,
                    "motor/limit builder constraint or J-buffer reservation differs")
            constraint += 1 if role == "palm" else 2
        return observed

    def role(self, row):
        return self.by_handle[native_joint_handle_words(row["joint_handle"])]

    def summary(self):
        return {"identities": self.frozen_identity,
                "labels": "palm and two native append-order finger slots; anatomical left/right frames are not decoded",
                "zero_authority_roles": list(ROLES[1:])}
