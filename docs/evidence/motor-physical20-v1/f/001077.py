"""Invented word records only; never native fixtures or archived observations."""
import copy
from pathlib import Path
import tempfile
import unittest
from fixed_contract import PROCESS_ORDER
from run_schema import validate_run, schedules, native_visit, loads, ONE
from stage_numeric import F32, rounded
from fractions import Fraction as F
from synthetic_fixtures import mock_records, raw, gz

class RunTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.path=Path(self.tmp.name)/'SYNTHETIC_ONLY.gz'
    def tearDown(self):self.tmp.cleanup()
    def check(self, records, process):
        self.path.write_bytes(gz(raw(records)))
        return validate_run(self.path, process)
    def test_all_twenty_both_variants_on_and_off_synthetic(self):
        for process in PROCESS_ORDER:
            run=self.check(mock_records(process),process)
            self.assertEqual(len(run['analyses']),int(process['observer']=='on'))
            for a in run['analyses']:
                for key in ('construction','integration','initialized_diagonal'):
                    self.assertEqual(a[key]['status'],'DECLARED_FORMULAS_MATCH')
                self.assertEqual(len(a['native_visits']),3)
                for v in a['native_visits']:self.assertEqual(v['check']['status'],'DECLARED_FORMULAS_MATCH')
                self.assertTrue(a['physical_SI']['independent_inertia_gate'])
    def test_schedule_counts_and_iteration_collision_are_not_collapsed(self):
        c,s=schedules();self.assertEqual((len(c)+1,len(s)),(9,26))
        process=PROCESS_ORDER[0];run=self.check(mock_records(process),process)
        visits=run['analyses'][0]['native_visits']
        self.assertEqual([(v['phase'],v['iteration']) for v in visits],[('pgs',0),('stabilization',0),('stabilization',1)])
    def test_hand_half_response_and_mismatch(self):
        left={'rows':[dict(rhs_word=0xbf000000,impulse_word=0,cfm_gain_word=0x3f000000,inverse_lhs_word=ONE,
             impulse_bound_words=[0xc0000000,0x40000000],solver_velocity_offset=0,ndofs2=1,jacobian_words=[ONE],weighted_jacobian_words=[ONE])],
             'generalized_velocity_words':[0]}
        right=copy.deepcopy(left);right['rows'][0]['impulse_word']=0xbf000000;right['generalized_velocity_words']=[0x3f000000]
        self.assertEqual(native_visit(left,right)['status'],'DECLARED_FORMULAS_MATCH')
        right['generalized_velocity_words']=[ONE];self.assertEqual(native_visit(left,right)['status'],'MISMATCH')
    def test_nonfinite_rounding_is_excluded_not_passed(self):
        left={'rows':[dict(rhs_word=0x7fc00001,impulse_word=0,cfm_gain_word=ONE,inverse_lhs_word=ONE,
             impulse_bound_words=[0xc0000000,0x40000000],solver_velocity_offset=0,ndofs2=1,jacobian_words=[ONE],weighted_jacobian_words=[ONE])],
             'generalized_velocity_words':[0]}
        self.assertEqual(native_visit(left,copy.deepcopy(left))['status'],'EXCLUDED_SOURCE_FLOAT_DOMAIN')
    def test_native_rounding_tie_signed_zero_and_tiny_domain_are_explicit(self):
        self.assertEqual(rounded(F(1,2)+F(1,1<<25)),0x3f000000)  # halfway, ties to even
        self.assertEqual(rounded(F(0),negative_zero=True),0x80000000)
        f=F32();f.multiply(0x00800000,0x3f000000)
        self.assertIn('tiny_nonzero_exact_intermediate',f.exclusions)
        f=F32();f.add(1,0)
        self.assertIn('subnormal_input',f.exclusions)

    def test_case_variant_actual_control_boolean_and_row_faults_fail_closed(self):
        process=PROCESS_ORDER[0]
        mutations=[lambda r:r[0].update(case_id='unknown'),lambda r:r[0].update(variant='candidate'),
            lambda r:r[0]['declared_conditions'].update(sphere_mass_kg_word=ONE),lambda r:r[0].update(native_scalar_bits=True),
            lambda r:r[1]['checkpoint']['public_body_words'][1].update(mass_query_kg_word=ONE),
            lambda r:r[2]['stage_v1']['events'][2]['rows'][0].update(axis='LinX'),
            lambda r:r[2]['stage_v1']['events'][2]['rows'][0].update(gain_model='AccelerationBased'),
            lambda r:r[2]['stage_v1']['events'][2]['rows'][0].update(ndofs2=2),
            lambda r:r[2]['stage_v1']['events'][2]['rows'][0].update(j_id2=True),
            lambda r:r[2]['stage_v1']['events'][2]['rows'][0].update(joint_handle={'index':2,'generation':0}),
            lambda r:r[2]['construction_companion']['events'].pop(),
            lambda r:r[2]['checkpoint']['public_body_words'][1]['angular_velocity_words'].__setitem__(0,0x7fc00000)]
        for mutate in mutations:
            records=mock_records(process);mutate(records)
            with self.assertRaises(ValueError):self.check(records,process)
    def test_integration_and_final_writeback_not_substitutable(self):
        process=PROCESS_ORDER[0];records=mock_records(process)
        records[2]['construction_companion']['events'][3]['endpoints'][0]['multibody_velocity_words']=[ONE]
        result=self.check(records,process)
        self.assertEqual(result['analyses'][0]['integration']['status'],'MISMATCH')
    def test_impulse_sign_duration_ceiling_and_saturation_linkage_fail(self):
        process=PROCESS_ORDER[0]
        mutations=[lambda t:t['motors'][0]['impulse'].update(applied_impulse_word=0),
            lambda t:t.update(numerical_outer_interval_s_word=0),
            lambda t:t['motors'][0]['slices'][0].update(finite_positive_bound_active=True),
            lambda t:t['motors'][0]['impulse'].update(effort_ceiling_word=0)]
        for mutate in mutations:
            records=mock_records(process);mutate(records[2]['impulse_v1_ieee_words'])
            with self.assertRaises(ValueError):self.check(records,process)
    def test_off_nulls_and_truncated_duplicate_float_multiple_members_fail(self):
        process=PROCESS_ORDER[2];records=mock_records(process)
        self.assertEqual(self.check(records,process)['analyses'],[])
        records[2]['stage_v1']={}
        with self.assertRaises(ValueError):self.check(records,process)
        value=gz(raw(mock_records(process)))
        for data in [value[:-1],value+b'junk',value+gz(b'{}\n')]:
            self.path.write_bytes(data)
            with self.assertRaises(ValueError):validate_run(self.path,process)
        for data in [b'{"x":1.0}',b'{"x":1,"x":2}',b'{"x":NaN}']:
            with self.assertRaises(ValueError):loads(data)

if __name__=='__main__':unittest.main()
