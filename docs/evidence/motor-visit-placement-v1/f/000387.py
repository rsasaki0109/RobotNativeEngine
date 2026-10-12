"""Synthetic whole framing/tamper tests plus independent small solve identities."""
import copy
from pathlib import Path
import tempfile
import unittest
from run_schema import *
from synthetic_fixtures import mock_records, raw, gz


class RunTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.path=Path(self.tmp.name)/'SYNTHETIC_ONLY.gz'

    def tearDown(self):
        self.tmp.cleanup()

    def check(self, records, case):
        self.path.write_bytes(gz(raw(records)))
        return validate_run(self.path,case)

    def test_complete_all_profiles_and_both_signs_synthetic(self):
        for case in CASES:
            if case['observer']=='off':continue
            run=self.check(mock_records(case),case)
            self.assertEqual(len(run['analyses']),32)
            for a in run['analyses']:
                self.assertEqual(a['construction']['status'],'DECLARED_FORMULAS_MATCH')
                self.assertEqual(a['integration']['status'],'DECLARED_FORMULAS_MATCH')
                self.assertEqual(len(a['native_visits']),sum(PROFILES[case['profile']]))
                for v in a['native_visits']:
                    self.assertEqual(v['check']['status'],'DECLARED_FORMULAS_MATCH')

    def test_source_derived_schedule_counts(self):
        for name,expected in [('split_1_2',(257,832)),('early_1_0',(193,416)),('early_3_0',(193,800))]:
            c,s=schedules(name)
            self.assertEqual((len(c)*32+1,len(s)*32),expected)
            if name!='split_1_2':
                self.assertFalse(any('Stabilization' in x[0] for x in c+s))

    def test_solve_pair_has_hand_derived_half_response(self):
        left={'rows':[dict(rhs_word=0xbf000000,impulse_word=0,cfm_gain_word=0x3f000000,
            inverse_lhs_word=ONE,impulse_bound_words=[0xc0000000,CAP],
            solver_velocity_offset=0,ndofs2=1,jacobian_words=[ONE],weighted_jacobian_words=[ONE])],
            'generalized_velocity_words':[0]}
        right=copy.deepcopy(left)
        right['rows'][0]['impulse_word']=0xbf000000
        right['generalized_velocity_words']=[0x3f000000]
        self.assertEqual(native_visit(left,right)['status'],'DECLARED_FORMULAS_MATCH')
        right['generalized_velocity_words']=[ONE]
        self.assertEqual(native_visit(left,right)['status'],'MISMATCH')

    def test_solve_rounding_domain_excluded_not_passed(self):
        left={'rows':[dict(rhs_word=0x7fc00001,impulse_word=0,cfm_gain_word=ONE,
            inverse_lhs_word=ONE,impulse_bound_words=[0xc0000000,CAP],
            solver_velocity_offset=0,ndofs2=1,jacobian_words=[ONE],weighted_jacobian_words=[ONE])],
            'generalized_velocity_words':[0]}
        self.assertEqual(native_visit(left,copy.deepcopy(left))['status'],'EXCLUDED_SOURCE_FLOAT_DOMAIN')

    def test_wrong_profile_or_false_boolean_words_fail(self):
        case=CASES[6]
        for mutate in [lambda r:r[0].update(profile='split_1_2'),
                       lambda r:r[0]['declared_conditions'].update(stabilization_passes=2),
                       lambda r:r[0].update(native_scalar_bits=True),
                       lambda r:r[2]['checkpoint']['integration_parameters']['native_counts'].__setitem__(3,2)]:
            records=mock_records(case);mutate(records)
            with self.assertRaises(ValueError):self.check(records,case)

    def test_malformed_row_axis_handle_block_and_schedule_fail_closed(self):
        case=CASES[0]
        for key,value in [('axis','LinX'),('gain_model','AccelerationBased'),('ndofs2',2),
                          ('j_id2',True),('joint_handle',{'index':2,'generation':0})]:
            records=mock_records(case)
            records[2]['stage_v1']['events'][2]['rows'][0][key]=value
            with self.assertRaises(ValueError):self.check(records,case)
        records=mock_records(case);records[2]['stage_v1']['events'].pop()
        with self.assertRaises(ValueError):self.check(records,case)

    def test_velocity_substitution_and_cache_mutation_fail(self):
        case=CASES[0]
        records=mock_records(case)
        records[2]['construction_companion']['events'][3]['endpoints'][0]['multibody_velocity_words']=[ONE]
        result=self.check(records,case)
        self.assertEqual(result['analyses'][0]['integration']['status'],'MISMATCH')
        records=mock_records(case)
        records[2]['stage_v1']['events'][3]['rows'][0]['weighted_jacobian_words']=[CAP]
        with self.assertRaises(ValueError):self.check(records,case)

    def test_native_solve_mismatch_is_retained_not_reinterpreted(self):
        case=CASES[0];records=mock_records(case)
        records[2]['stage_v1']['events'][7]['rows'][0]['impulse_word']^=1
        # A single inconsistent row also breaks subsequent no-op continuity.
        with self.assertRaises(ValueError):self.check(records,case)

    def test_off_has_three_nulls_and_can_not_smuggle_observer(self):
        case=CASES[2];records=mock_records(case)
        self.assertEqual(self.check(records,case)['analyses'],[])
        records[2]['construction_companion']={}
        with self.assertRaises(ValueError):self.check(records,case)

    def test_truncated_multiple_member_float_and_duplicate_json_fail(self):
        case=CASES[0];value=gz(raw(mock_records(case)))
        for data in [value[:-1],value+b'junk',value+gz(b'{}\n')]:
            self.path.write_bytes(data)
            with self.assertRaises(ValueError):validate_run(self.path,case)
        for data in [b'{"x":1.0}',b'{"x":1,"x":2}',b'{"x":NaN}']:
            with self.assertRaises(ValueError):loads(data)


if __name__=='__main__':
    unittest.main()
