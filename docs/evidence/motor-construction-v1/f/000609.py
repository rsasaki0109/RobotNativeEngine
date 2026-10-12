"""Synthetic exact/golden and tamper tests; no engine calls or native data."""
import copy
import json
from pathlib import Path
import unittest

from construction_oracle import verify_construction

PZERO = 0x00000000
NZERO = 0x80000000
ONE = 0x3f800000
TWO = 0x40000000
THREE = 0x40400000
HALF = 0x3f000000
N_HALF = 0xbf000000


def golden():
    # h=k=d=1, q=0, target=1, velocity target=0; a=g=1/2, rhs=-1/2.
    return dict(coordinate_word=PZERO, target_position_word=ONE,
                target_velocity_word=PZERO, stiffness_word=ONE, damping_word=ONE,
                duration_word=ONE, inverse_duration_word=ONE, limits_words=None,
                effective_target_velocity_word=PZERO, erp_inverse_duration_word=HALF,
                row=dict(gain_model='ForceBased', effort_ceiling_word=TWO,
                         rhs_word=N_HALF, rhs_without_bias_word=N_HALF,
                         cfm_gain_word=HALF, cfm_coefficient_word=PZERO,
                         impulse_bound_words=[0xc0000000, TWO]))


class ConstructionTests(unittest.TestCase):
    def test_hand_derived_force_based_golden(self):
        result = verify_construction(golden())
        self.assertEqual(result['status'], 'DECLARED_FORMULAS_MATCH')
        self.assertFalse(result['motor_accuracy_verified'])

    def test_acceleration_based_places_compliance_in_coefficient(self):
        record = golden(); record['row'].update(gain_model='AccelerationBased',
                                               cfm_gain_word=PZERO, cfm_coefficient_word=HALF)
        self.assertEqual(verify_construction(record)['status'], 'DECLARED_FORMULAS_MATCH')

    def test_positive_target_velocity_is_clamped_from_actual_limits(self):
        record = golden(); record.update(coordinate_word=TWO, target_velocity_word=THREE,
            limits_words=[PZERO,ONE], effective_target_velocity_word=0xbf800000)
        record['row'].update(rhs_word=0x3fc00000, rhs_without_bias_word=0x3fc00000)
        self.assertEqual(verify_construction(record)['status'], 'DECLARED_FORMULAS_MATCH')

    def test_equal_signed_zero_velocity_is_not_replaced_by_clamp_bound(self):
        record = golden(); record.update(coordinate_word=PZERO, target_position_word=PZERO,
            target_velocity_word=NZERO, limits_words=[PZERO,PZERO],
            effective_target_velocity_word=NZERO)
        record['row'].update(rhs_word=PZERO,rhs_without_bias_word=PZERO)
        self.assertEqual(verify_construction(record)['status'], 'DECLARED_FORMULAS_MATCH')
        record['effective_target_velocity_word']=PZERO
        self.assertIn('effective_target_velocity_word',verify_construction(record)['mismatches'])

    def test_every_checked_output_tamper_is_rejected(self):
        baseline=golden()
        for field in ('inverse_duration_word','erp_inverse_duration_word',
                      'effective_target_velocity_word'):
            with self.subTest(field=field):
                record=copy.deepcopy(baseline);record[field]^=1
                self.assertIn(field,verify_construction(record)['mismatches'])
        for field in ('rhs_word','rhs_without_bias_word','cfm_gain_word','cfm_coefficient_word'):
            with self.subTest(field=field):
                record=copy.deepcopy(baseline);record['row'][field]^=1
                self.assertIn(field,verify_construction(record)['mismatches'])
        for index,name in enumerate(('lower_impulse_bound_word','upper_impulse_bound_word')):
            record=copy.deepcopy(baseline);record['row']['impulse_bound_words'][index]^=1
            self.assertIn(name,verify_construction(record)['mismatches'])

    def test_missing_availability_unknown_model_width_and_boolean_word_fail(self):
        record=golden(); del record['limits_words']
        with self.assertRaises(ValueError): verify_construction(record)
        record=golden();record['row']['gain_model']='guessed'
        with self.assertRaises(ValueError): verify_construction(record)
        with self.assertRaises(ValueError): verify_construction(golden(),64)
        record=golden();record['coordinate_word']=False
        with self.assertRaises(ValueError): verify_construction(record)

    def test_nonfinite_and_subnormal_are_not_promoted_to_qualified(self):
        record=golden();record['coordinate_word']=0x7fc00000
        self.assertEqual(verify_construction(record)['status'],'EXCLUDED_SOURCE_FLOAT_DOMAIN')
        record=golden();record['coordinate_word']=1
        result=verify_construction(record)
        self.assertEqual(result['status'],'EXCLUDED_SOURCE_FLOAT_DOMAIN')
        self.assertIn('subnormal_input',result['exclusions'])

    def test_zero_cap_preserves_negative_lower_zero(self):
        record=golden();record['row'].update(effort_ceiling_word=PZERO,
                                         impulse_bound_words=[NZERO,PZERO])
        self.assertEqual(verify_construction(record)['status'],'DECLARED_FORMULAS_MATCH')
        record['row']['impulse_bound_words'][0]=PZERO
        self.assertIn('lower_impulse_bound_word',verify_construction(record)['mismatches'])

    def test_nonfinite_calculated_output_is_a_mismatch_not_input_exclusion(self):
        record=golden();record['inverse_duration_word']=0x7fc00000
        result=verify_construction(record)
        self.assertEqual(result['status'],'MISMATCH')
        self.assertEqual(result['exclusions'],[])
        self.assertIn('inverse_duration_word',result['mismatches'])

    def test_finite_input_overflow_retains_explicit_excluded_result(self):
        record=golden();record['duration_word']=0x7f7fffff
        result=verify_construction(record)
        self.assertEqual(result['status'],'EXCLUDED_SOURCE_FLOAT_DOMAIN')
        self.assertIn('overflow_intermediate',result['exclusions'])
        self.assertIsNone(result['expected_words'])
        self.assertFalse(result['motor_accuracy_verified'])


if __name__=='__main__':
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(ConstructionTests)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    receipt=dict(kind='prospective_synthetic_construction_oracle_tests',
                 tests_run=result.testsRun,failures=len(result.failures),errors=len(result.errors),
                 actual_exit_code=0 if result.wasSuccessful() else 1,
                 native_worlds_launched=0,native_data_read=False)
    Path('actual-pure-tests.json').write_text(json.dumps(receipt,indent=2)+'\n')
    raise SystemExit(receipt['actual_exit_code'])
