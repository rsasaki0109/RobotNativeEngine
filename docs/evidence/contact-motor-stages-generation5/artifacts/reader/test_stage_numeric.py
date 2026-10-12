#!/usr/bin/env python3
"""Analytical binary32/Fraction expectations; zero native worlds or old data."""
from fractions import Fraction
import unittest
from stage_numeric import (EPSILON,F32,MIN_NORMAL,ONE,POS_INF,NEG_INF,SIGN,
                           clamp_fraction,exact,exact_snapshot,kind,rounded)
from stage_schema import f32_bits

class NumericTests(unittest.TestCase):
    def test_exact_binary32_boundaries_and_ties(self):
        self.assertEqual(exact(1),Fraction(1,1<<149))
        self.assertEqual(exact(0x00800000),MIN_NORMAL)
        self.assertEqual(exact(ONE),1)
        self.assertEqual(rounded(Fraction(1)+Fraction(1,1<<24)),ONE)
        self.assertEqual(rounded(Fraction(1)+Fraction(3,1<<24)),ONE+2)
        self.assertEqual(rounded(Fraction(1,1<<150)),0)
        self.assertEqual(rounded(-Fraction(1,1<<150)),SIGN)
        self.assertEqual(rounded(0,negative_zero=True),SIGN)
        for bad in (True,-1,1<<32,1.0):
            with self.assertRaises(ValueError): exact(bad)

    def test_signed_zero_native_operations_and_guard(self):
        n=F32()
        self.assertEqual(n.add(SIGN,SIGN),SIGN)
        self.assertEqual(n.add(ONE,ONE^SIGN),0)
        self.assertEqual(n.multiply(SIGN,ONE),SIGN)
        self.assertEqual(n.subtract(0,0),0)
        self.assertEqual(EPSILON,f32_bits(1e-20))
        self.assertEqual(n.inverse(EPSILON),0)
        self.assertEqual(n.inverse(EPSILON^SIGN),0)
        self.assertEqual(n.inverse(f32_bits(2)),f32_bits(.5))
        self.assertEqual(n.clamp(0,SIGN,0),SIGN) # equality returns literal lower
        self.assertFalse(n.exclusions)

    def test_dyn8_source_order_known_nonassociative_dot(self):
        big=f32_bits(1<<30)
        values=[big,ONE,big^SIGN,ONE,big^SIGN,ONE,big,ONE]
        self.assertEqual(F32().dot_dyn(values,[ONE]*8),f32_bits(4))
        self.assertEqual(F32().dot_dyn([big,ONE,big^SIGN],[ONE]*3),0)
        self.assertEqual(F32().dot_dyn(values+[ONE],[ONE]*9),f32_bits(5))

    def test_row_exact_reference_and_native_operation_signs(self):
        row={'solver_velocity_offset':0,'ndofs2':3,'jacobian_words':[ONE,0,0],
             'weighted_jacobian_words':[f32_bits(2),f32_bits(3),0],
             'rhs_word':f32_bits(4),'cfm_gain_word':ONE,'inverse_lhs_word':f32_bits(.5),
             'impulse_word':ONE,'impulse_bound_words':[f32_bits(-10),f32_bits(10)]}
        velocity=[f32_bits(2),f32_bits(4),f32_bits(-1)]
        ref=exact_snapshot(row,velocity)
        self.assertEqual(ref['residual'],5)
        self.assertEqual(ref['diagonal'],2)
        self.assertEqual(ref['denominator'],3)
        self.assertEqual(ref['conditional_impulse'],Fraction(8,3))
        self.assertEqual(ref['projection'],-Fraction(5,3))
        native=F32().projection(row,velocity)
        self.assertEqual(native['next_impulse_word'],f32_bits(3.5))
        self.assertEqual(native['delta_impulse_word'],f32_bits(2.5))
        self.assertEqual(native['applied_projection_word'],f32_bits(-2.5))
        self.assertEqual(native['next_generalized_velocity_words'],[f32_bits(-3),f32_bits(-3.5),f32_bits(-1)])
        self.assertEqual(native['eligibility'],'qualified')

    def test_caps_and_zero_authority_complementarity(self):
        base={'solver_velocity_offset':0,'ndofs2':1,'jacobian_words':[ONE],
              'weighted_jacobian_words':[ONE],'cfm_gain_word':0,'inverse_lhs_word':ONE,
              'impulse_bound_words':[f32_bits(-2),f32_bits(2)]}
        for impulse,rhs,position in ((-2,-3,'at_lower_bound'),(2,3,'at_upper_bound'),(0,0,'interior')):
            row=dict(base,impulse_word=f32_bits(impulse),rhs_word=f32_bits(rhs))
            ref=exact_snapshot(row,[0])
            self.assertEqual(ref['complementarity'],'satisfied');self.assertEqual(ref['position'],position)
            self.assertEqual(ref['projection'],0)
        for impulse in (0,SIGN):
            for rhs in (f32_bits(3),f32_bits(-3)):
                row=dict(base,impulse_word=impulse,rhs_word=rhs,impulse_bound_words=[SIGN,0])
                ref=exact_snapshot(row,[0])
                self.assertEqual(ref['position'],'zero_authority');self.assertEqual(ref['projection'],0)
                native=F32().projection(row,[0])
                self.assertEqual(exact(native['next_impulse_word']),0)
                self.assertEqual(native['eligibility'],'qualified')
        self.assertEqual(clamp_fraction(Fraction(100),NEG_INF,POS_INF),100)
        with self.assertRaises(ValueError): clamp_fraction(Fraction(0),ONE,0)

    def test_normal_zero_domain_exclusions_retained(self):
        n=F32();self.assertEqual(n.multiply(0x00800000,f32_bits(.5)),0x00400000)
        self.assertIn('tiny_nonzero_exact_intermediate',n.exclusions)
        n=F32();n.multiply(1,ONE);self.assertIn('subnormal_input',n.exclusions)
        n=F32();self.assertEqual(n.multiply(0x7f7fffff,f32_bits(2)),POS_INF)
        self.assertIn('overflow_intermediate',n.exclusions)
        with self.assertRaises(ValueError): F32().add(POS_INF,ONE)

if __name__=='__main__': unittest.main(verbosity=2)
