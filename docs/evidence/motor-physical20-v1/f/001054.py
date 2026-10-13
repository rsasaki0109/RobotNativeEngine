"""Immutable explicit case context; no global current-case state or inference."""
from fixed_contract import CASES
from strict_io import need

class CaseContext:
    def __init__(self, case_id, variant):
        need(type(case_id) is str and variant in ('baseline', 'candidate'), 'unsupported case/variant')
        selected = [c for c in CASES if c['case_id'] == case_id]
        need(len(selected) == 1, 'unknown or duplicate fixed case')
        self.case = selected[0]
        self.case_id, self.variant = case_id, variant
        mapping = dict(h='outer_dt_s', k='stiffness_nm_per_rad', d='damping_nm_s_per_rad',
                       cap='max_torque_nm', target='target_pos_rad', mass='mass_kg', radius='radius_m')
        for attr, field in mapping.items():
            setattr(self, attr, int(self.case['input_f32_words'][field], 16))
        self.target_name = 'zero' if self.case['target_pos_rad'] == 0 else 'negative' if self.case['target_pos_rad'] < 0 else 'positive'
