#!/usr/bin/env python3
"""Pure rational arithmetic and retained-data structure checks, zero native worlds."""
from datetime import datetime, timezone
from fractions import Fraction
import hashlib
import importlib.util
import itertools
import json
from pathlib import Path
import struct

ROOT = Path('/workspace/.rne-kick-demo/single-joint-motor-calibration')
HERE = ROOT / 'diagonal-intervention-generation-3/independent-oracle'


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def ref(path):
    return {'path': str(path), 'bytes': path.stat().st_size,
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def require_rejection(function, *args):
    try:
        function(*args)
    except (ValueError, KeyError, TypeError, OverflowError):
        return
    raise AssertionError('Malformed/unavailable input was admitted')


def main():
    old_path = ROOT / 'independent-oracle/frozen_row_oracle.py'
    candidate_path = HERE / 'frozen_candidate_row_oracle.py'
    auditor_path = HERE / 'audit_candidate_report.py'
    old = module('immutable_original_oracle', old_path)
    candidate = module('independent_candidate_oracle', candidate_path)
    audit = module('new_retained_report_auditor', auditor_path)
    for oracle in (old, candidate):
        assert oracle.add(oracle.SIGN, oracle.SIGN) == oracle.SIGN
        assert oracle.add(oracle.SIGN, oracle.ZERO) == oracle.ZERO
        assert oracle.multiply(oracle.SIGN, oracle.ONE) == oracle.SIGN
        assert oracle.divide(oracle.SIGN, oracle.ONE) == oracle.SIGN
        assert oracle.rapier_inv(oracle.EPSILON) == 0
        assert oracle.rapier_inv(oracle.negate(oracle.EPSILON)) == 0
        assert oracle.rapier_inv(oracle.EPSILON + 1) != 0
        assert oracle.add(oracle.ONE, oracle.rounded(Fraction(1, 1 << 24))) == oracle.ONE
        assert oracle.add(oracle.ONE + 1, oracle.rounded(Fraction(1, 1 << 24))) == oracle.ONE + 2
        for invalid in (True, -1, 1 << 32, 0x7f800000, 0x7fc00000):
            require_rejection(oracle.checked_bits, invalid)
    base = {'outer_interval_bits_u32': candidate.bits_from_decimal(.001),
            'pre_effective_inertia_xx_bits_u32': candidate.bits_from_decimal(.004),
            'initial_coordinate_bits_u32': 0, 'initial_velocity_bits_u32': 0,
            'target_velocity_bits_u32': 0, 'effort_cap_bits_u32': candidate.bits_from_decimal(2)}
    calculations = []
    reference_comparisons = 0
    for (k, d), target, n, pgs in itertools.product(((300, 12), (250, 8)), (-.001, 0, .001), (1, 32), (1, 2)):
        row = dict(base, stiffness_bits_u32=candidate.bits_from_decimal(k),
                   damping_bits_u32=candidate.bits_from_decimal(d),
                   target_coordinate_bits_u32=candidate.bits_from_decimal(target), small_step_count=n)
        original, result = old.evaluate(row), candidate.evaluate(row)
        expr = result['native_f32_expression']
        expected_sum = candidate.add(expr['fixture_derived_inverse_mass']['bits_u32'], expr['cfm_gain']['bits_u32'])
        expected_inv = candidate.rapier_inv(expected_sum)
        assert expr['candidate_rounded_diagonal_sum']['bits_u32'] == expected_sum
        assert expr['unit_motor_inv_lhs']['bits_u32'] == expected_inv
        assert expr['legacy_unit_motor_inv_lhs'] == original['native_f32_expression']['unit_motor_inv_lhs']
        for key in ('applied_impulse_nm_s', 'unbounded_applied_impulse_nm_s',
                    'exact_rational_numerator', 'exact_rational_denominator',
                    'native_solver_lambda_nm_s', 'interior', 'tolerance_nm_s'):
            assert result['reference'][key] == original['reference'][key]
            reference_comparisons += 1
        assert result['actual_first_slice_comparison'] is None
        assert result['source_coefficient_comparison'] is None
        assert result['post_step_lu_comparison'] is None
        assert (target == 0) == (result['reference']['applied_impulse_nm_s'] == 0)
        for key in original['native_f32_expression']:
            if key != 'unit_motor_inv_lhs':
                assert result['native_f32_expression'][key] == original['native_f32_expression'][key]
        calculations.append({'gain': [k, d], 'target': target, 'small_steps': n, 'PGS_passes': pgs,
                             'candidate_diagonal_sum': expr['candidate_rounded_diagonal_sum'],
                             'candidate_inverse_diagonal': expr['unit_motor_inv_lhs'],
                             'unchanged_fixed_point': result['reference']['applied_impulse_nm_s'],
                             'measured_outcome': None})
    assert len(calculations) == 24
    sample = dict(base, stiffness_bits_u32=candidate.bits_from_decimal(300), damping_bits_u32=candidate.bits_from_decimal(12),
                  target_coordinate_bits_u32=candidate.bits_from_decimal(.001), small_step_count=32)
    for invalid in (True, 0, -1, 1 << 25):
        require_rejection(candidate.evaluate, dict(sample, small_step_count=invalid))
    require_rejection(candidate.evaluate, dict(sample, pre_effective_inertia_xx_bits_u32=0))
    require_rejection(candidate.evaluate, dict(sample, effort_cap_bits_u32=candidate.bits_from_decimal(-1)))
    require_rejection(candidate.evaluate, dict(sample, actual_first_applied_impulse_nm_s=float('nan')))
    require_rejection(candidate.evaluate, dict(sample, actual_first_applied_impulse_nm_s=.123))
    require_rejection(candidate.evaluate, dict(sample, observed_native_coefficient_bits_u32={}))
    assert candidate.evaluate(dict(sample, effort_cap_bits_u32=0))['reference']['applied_impulse_nm_s'] == 0
    historical_path = ROOT / 'codec-migration-generation-2/raw-report.json'
    historical = json.loads(historical_path.read_text())
    counts = {'historical_state_structures': 0, 'historical_telemetry_structures': 0,
              'historical_scalar_wrappers': audit.walk_scalars(historical)}
    for case in historical['cases']:
        for run in case['runs']:
            for stage in ('initial', 'final_state'):
                audit.state_words(run[stage])
                counts['historical_state_structures'] += 1
            if run['telemetry'] is not None:
                audit.telemetry_words(run['telemetry'])
                counts['historical_telemetry_structures'] += 1
    assert counts['historical_state_structures'] == 144
    assert counts['historical_telemetry_structures'] == 48
    require_rejection(audit.exact_tree, {'optional': None}, {})
    require_rejection(audit.exact_tree, [0.0], [-0.0])
    require_rejection(audit.exact_tree, [0], [False])
    require_rejection(audit.finite_scalar, {'finite': None, 'ieee754_f64_bits': 0x7ff0000000000000, 'native_f32_bits': None})
    output = {'status': 'PASS_PURE_ORACLE_AND_RETAINED_STRUCTURE_CHECKS_ONLY',
              'created_utc': datetime.now(timezone.utc).isoformat(),
              'scope': '24 independently constructed arithmetic fixture rows; .004 literal f32 inertia, no candidate measured state and no generation3 result',
              'native_worlds_builds_cargo_tests_checkout_edits': 0,
              'unchanged_original_reference_field_comparisons': reference_comparisons,
              'counts': counts, 'pure24calculations': calculations,
              'tools': [ref(path) for path in (old_path, candidate_path, auditor_path, Path(__file__))],
              'historical_read_only_report': ref(historical_path)}
    out = HERE / 'pre-world-pure-oracle-tests.json'
    assert not out.exists()
    out.write_text(json.dumps(output, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'status': output['status'], 'output': ref(out), 'counts': counts,
                      'reference_comparisons': reference_comparisons, 'worlds': 0}, indent=2))


if __name__ == '__main__':
    main()
