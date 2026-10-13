"""Prospective pure endpoint check for the isolated fixed-root X revolute only."""
from construction_oracle import require
from stage_numeric import F32, checked_word, exact, kind


def verify_scalar_advance(duration_word, before, after, generic_velocity_words):
    """Bind the captured integration-input velocity, not final stabilized velocity.

    Caller must first bind event phase/root/cache/source identity and schedule.
    The source-specific one-DOF restriction excludes floating/multi-DOF models.
    No event source or runtime float-mode qualification follows from this helper.
    """
    h = checked_word(duration_word)
    require(type(before) is dict and type(after) is dict, 'missing endpoint')
    left, right = before.get('row'), after.get('row')
    require(type(left) is dict and type(right) is dict, 'missing endpoint row')
    identity = ('joint_handle', 'link_id', 'axis', 'gain_model',
                'constraint_index', 'solver_velocity_offset', 'ndofs2', 'j_id2')
    require(all(key in left and key in right for key in identity),
            'missing row identity')
    for row in (left,right):
        handle=row['joint_handle']
        require(type(handle) is dict and set(handle)=={'index','generation'},
                'invalid native handle shape')
        for value in handle.values():
            checked_word(value)
        require(all(type(row[key]) is int and 0 <= row[key] < (1 << 64)
                    for key in ('link_id','constraint_index','solver_velocity_offset','ndofs2','j_id2')),
                'invalid native row index')
    require(all(left[key] == right[key] for key in identity), 'mixed endpoint rows')
    require(left['ndofs2'] == 1 and
            left['axis'] == 'AngX' and left['gain_model'] == 'ForceBased',
            'outside declared isolated X-revolute scope')
    offset = left['solver_velocity_offset']
    require(type(offset) is int and offset >= 0 and
            type(generic_velocity_words) is list and offset < len(generic_velocity_words),
            'invalid solver velocity offset')
    q0, q1 = checked_word(before.get('coordinate_word')), checked_word(after.get('coordinate_word'))
    before_v, after_v = before.get('multibody_velocity_words'), after.get('multibody_velocity_words')
    require(type(before_v) is list and len(before_v) == 1 and
            type(after_v) is list and len(after_v) == 1, 'wrong native velocity length')
    v0, v1, generic = (checked_word(value) for value in
                       (before_v[0], after_v[0], generic_velocity_words[offset]))
    inputs = (h, q0, v0)
    if any(kind(word) in ('nan', 'positive_infinity', 'negative_infinity') for word in inputs):
        return {'status': 'EXCLUDED_SOURCE_FLOAT_DOMAIN',
                'exclusions': ['nonfinite_arithmetic_input'], 'mismatches': None,
                'motor_accuracy_verified': False}
    require(exact(h) >= 0, 'negative duration')
    f = F32()
    for word in inputs:
        f.input(word)
    try:
        expected_q = f.add(q0, f.multiply(v0, h))
    except ValueError:
        if not f.exclusions:
            raise
        return {'status': 'EXCLUDED_SOURCE_FLOAT_DOMAIN',
                'exclusions': sorted(f.exclusions), 'mismatches': None,
                'motor_accuracy_verified': False}
    mismatches = {}
    for field, expected, observed in (
            ('coordinate_after_word', expected_q, q1),
            ('copied_integration_velocity_word', generic, v0),
            ('unchanged_multibody_velocity_word', v0, v1)):
        if expected != observed:
            mismatches[field] = {'expected_word': expected, 'observed_word': observed}
    return {'status': ('EXCLUDED_SOURCE_FLOAT_DOMAIN' if f.exclusions else
                       'DECLARED_FORMULAS_MATCH' if not mismatches else 'MISMATCH'),
            'exclusions': sorted(f.exclusions), 'mismatches': mismatches,
            'expected_coordinate_word': expected_q, 'operation_count': f.operation_count,
            'motor_accuracy_verified': False}
