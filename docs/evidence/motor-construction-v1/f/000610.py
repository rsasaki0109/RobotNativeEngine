"""Pure golden, mixed-identity and wrong-endpoint tests; no engine data."""
import copy
import json
from pathlib import Path
import unittest

from integration_oracle import verify_scalar_advance


def endpoints():
    row=dict(joint_handle={'index':1,'generation':0}, link_id=1, axis='AngX',
             gain_model='ForceBased', constraint_index=0, solver_velocity_offset=0,
             ndofs2=1, j_id2=0)
    before=dict(row=row, coordinate_word=0, multibody_velocity_words=[0x3f800000])
    after=copy.deepcopy(before);after['coordinate_word']=0x3f800000
    return before,after


class IntegrationTests(unittest.TestCase):
    def test_hand_derived_unit_advance_uses_actual_copied_velocity(self):
        before,after=endpoints()
        result=verify_scalar_advance(0x3f800000,before,after,[0x3f800000])
        self.assertEqual(result['status'],'DECLARED_FORMULAS_MATCH')
        self.assertFalse(result['motor_accuracy_verified'])

    def test_stabilized_velocity_cannot_substitute_for_integration_velocity(self):
        before,after=endpoints()
        result=verify_scalar_advance(0x3f800000,before,after,[0x40000000])
        self.assertIn('copied_integration_velocity_word',result['mismatches'])
        wrong=copy.deepcopy(after);wrong['coordinate_word']=0x40000000
        self.assertIn('coordinate_after_word',verify_scalar_advance(
            0x3f800000,before,wrong,[0x3f800000])['mismatches'])

    def test_mixed_row_and_multidof_scope_are_rejected(self):
        before,after=endpoints();after['row']['joint_handle']['generation']=1
        with self.assertRaises(ValueError):verify_scalar_advance(0x3f800000,before,after,[0x3f800000])
        before,after=endpoints();before['row']['ndofs2']=2;after['row']['ndofs2']=2
        with self.assertRaises(ValueError):verify_scalar_advance(0x3f800000,before,after,[0x3f800000])

    def test_output_nan_is_mismatch_and_input_subnormal_is_explicit_exclusion(self):
        before,after=endpoints();after['coordinate_word']=0x7fc00000
        self.assertEqual(verify_scalar_advance(0x3f800000,before,after,[0x3f800000])['status'],'MISMATCH')
        before,after=endpoints();before['multibody_velocity_words']=[1]
        result=verify_scalar_advance(0x3f800000,before,after,[1])
        self.assertEqual(result['status'],'EXCLUDED_SOURCE_FLOAT_DOMAIN')
        self.assertIn('subnormal_input',result['exclusions'])

    def test_changed_native_velocity_between_coordinate_endpoints_is_retained(self):
        before,after=endpoints();after['multibody_velocity_words']=[0x40000000]
        result=verify_scalar_advance(0x3f800000,before,after,[0x3f800000])
        self.assertIn('unchanged_multibody_velocity_word',result['mismatches'])

    def test_boolean_native_handle_or_index_cannot_compare_equal_to_integer(self):
        before,after=endpoints();after['row']['joint_handle']['index']=True
        with self.assertRaises(ValueError):verify_scalar_advance(0x3f800000,before,after,[0x3f800000])
        before,after=endpoints();before['row']['link_id']=True;after['row']['link_id']=True
        with self.assertRaises(ValueError):verify_scalar_advance(0x3f800000,before,after,[0x3f800000])


if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(IntegrationTests))
    receipt=dict(kind='prospective_synthetic_integration_oracle_tests',tests_run=result.testsRun,
                 failures=len(result.failures),errors=len(result.errors),
                 actual_exit_code=0 if result.wasSuccessful() else 1,
                 native_worlds_launched=0,native_data_read=False)
    Path('actual-pure-tests.json').write_text(json.dumps(receipt,indent=2)+'\n')
    raise SystemExit(receipt['actual_exit_code'])
