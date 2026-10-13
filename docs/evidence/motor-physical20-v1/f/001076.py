"""Independent hand mechanics, tolerance boundaries, and wrong-inertia fault tests."""
from fractions import Fraction as F
import unittest
from fixed_contract import CASES
from physical_oracle import reference, diagnostics, measurement, P_ABS, P_REL, I_ABS, I_REL
from stage_numeric import rounded, exact
from physical_reference import backward_euler

class PhysicalTests(unittest.TestCase):
    def ideal(self,case):
        r=reference(case)
        return dict(public_mass_word=rounded(F(str(case['mass_kg']))),public_inertia_word=rounded(r['inertia']),weighted_word=rounded(1/r['inertia']),
            incoming_velocity_word=0,integration_velocity_word=rounded(r['velocity']),final_velocity_word=rounded(r['velocity']),
            integration_impulse_word=rounded(-r['impulse']),final_impulse_word=rounded(-r['impulse']),coordinate_word=rounded(r['coordinate']))
    def test_hand_backward_euler_interior_and_cap_units(self):
        r=backward_euler(F(2),F(4),F(2),F(1,2),F(0),F(0),F(1),F(10))
        self.assertEqual(r['velocity'],F(1,2));self.assertEqual(r['coordinate'],F(1,4))
        self.assertEqual(r['impulse'],F(1));self.assertEqual(r['torque'],F(2))
        capped=backward_euler(F(2),F(4),F(2),F(1,2),F(0),F(0),F(10),F(1,2))
        self.assertEqual(capped['impulse'],F(1,4));self.assertEqual(capped['velocity'],F(1,8))
        self.assertEqual(capped['coordinate'],F(1,16));self.assertEqual(capped['torque'],F(1,2))
    def test_exact_nominal_inertias_and_declared_regimes(self):
        regimes={}
        for c in CASES:
            r=reference(c);regimes[r['regime']]=regimes.get(r['regime'],0)+1
            self.assertEqual(r['inertia'],F(1,500) if c['mass_kg']==.5 else F(1,125))
            result=diagnostics(c,**self.ideal(c))
            self.assertTrue(result['independent_inertia_gate'])
            self.assertEqual(result['main_physical_label'],'MEETS_DECLARED_NEW_CASE_ALLOWANCE')
        self.assertEqual(regimes,{'interior':8,'saturated_negative':4,'saturated_positive':4,'null':4})
    def test_saturated_scaling_and_sign(self):
        capcases=[c for c in CASES if c['max_torque_nm']==.01]
        for c in capcases:
            r=reference(c);h=F(str(c['outer_dt_s']));tau=F(str(c['max_torque_nm']))*(1 if c['target_pos_rad']>0 else -1)
            self.assertEqual(r['impulse'],h*tau);self.assertEqual(r['velocity'],h*tau/r['inertia'])
            self.assertEqual(r['coordinate'],h*h*tau/r['inertia'])
    def test_same_impulse_allowance_derives_velocity_coordinate_torque(self):
        c=CASES[0];r=reference(c);d=diagnostics(c,**self.ideal(c));q=d['quantities']
        def fraction(x):return F(int(x['numerator']),int(x['denominator']))
        dp=max(P_ABS,P_REL*abs(r['impulse']))
        self.assertEqual(fraction(q['integration_input_velocity']['allowance']),dp/r['inertia'])
        self.assertEqual(fraction(q['integrated_coordinate']['allowance']),r['duration']*dp/r['inertia'])
        self.assertEqual(fraction(q['PGS_torque_over_nominal_duration']['allowance']),dp/r['duration'])
    def test_boundary_is_literal_not_expanded(self):
        allowance=F(1,100)
        self.assertEqual(measurement(F(1)+allowance,F(1),allowance,'Nm s')['status'],'MEETS_DECLARED_NEW_CASE_ALLOWANCE')
        self.assertEqual(measurement(F(1)+allowance+F(1,10**12),F(1),allowance,'Nm s')['status'],'OUTSIDE_DECLARED_NEW_CASE_ALLOWANCE')
    def test_wrong_inertia_not_fitted_to_row(self):
        c=CASES[0];values=self.ideal(c);values['weighted_word']=rounded(F(100))
        r=diagnostics(c,**values);self.assertFalse(r['independent_inertia_gate'])
        self.assertEqual(r['main_physical_label'],'UNQUALIFIED_NONFINITE_OR_MISSING')
        values=self.ideal(c);values['public_inertia_word']=rounded(F(1,100))
        self.assertFalse(diagnostics(c,**values)['independent_inertia_gate'])
    def test_final_velocity_does_not_replace_integrated_coordinate(self):
        c=CASES[0];values=self.ideal(c);values['final_velocity_word']=0
        d=diagnostics(c,**values)
        self.assertEqual(d['quantities']['integrated_coordinate']['status'],'MEETS_DECLARED_NEW_CASE_ALLOWANCE')
        self.assertEqual(d['quantities']['final_writeback_velocity']['status'],'OUTSIDE_DECLARED_NEW_CASE_ALLOWANCE')
    def test_zero_signed_words_nonfinite_and_bool(self):
        c=CASES[4];values=self.ideal(c)
        for name in ('incoming_velocity_word','integration_velocity_word','final_velocity_word','integration_impulse_word','final_impulse_word','coordinate_word'):values[name]=0x80000000
        d=diagnostics(c,**values);self.assertTrue(d['exact_null_motion']);self.assertEqual(d['signed_raw_words_preserved']['coordinate'],0x80000000)
        values['final_velocity_word']=0x7fc00001
        self.assertEqual(diagnostics(c,**values)['status'],'UNQUALIFIED_NONFINITE_OR_MISSING')
        values['final_velocity_word']=True
        with self.assertRaises(ValueError):diagnostics(c,**values)
    def test_nominal_is_not_f32_geometry(self):
        c=CASES[0];d=diagnostics(c,**self.ideal(c))
        residual=d['nominal_to_authoritative_f32_residuals']['radius_m']
        self.assertNotEqual(F(int(residual['numerator']),int(residual['denominator'])),0)
        self.assertFalse(d['historical_primary_reclassified']);self.assertFalse(d['accuracy_or_adoption_verified'])

if __name__=='__main__':unittest.main()
