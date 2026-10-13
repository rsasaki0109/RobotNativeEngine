"""Independent nominal-SI diagnostics, separate from native rounded row fidelity."""
from fractions import Fraction as F
from stage_numeric import exact, kind, rational, checked_word
from physical_reference import backward_euler

I_ABS = F(1, 10**8)
I_REL = F(2, 10**5)
P_ABS = F(1, 10**8)
P_REL = F(1, 100)


def nominal(case, field):
    """Fixed source nominal decimals, never inferred native inertia or fitted data."""
    return F(str(case[field]))


def reference(case):
    mass, radius = nominal(case, 'mass_kg'), nominal(case, 'radius_m')
    h = nominal(case, 'outer_dt_s')
    inertia = F(2, 5) * mass * radius * radius
    result = backward_euler(inertia, nominal(case, 'stiffness_nm_per_rad'),
        nominal(case, 'damping_nm_s_per_rad'), h, F(0), F(0),
        nominal(case, 'target_pos_rad'), nominal(case, 'max_torque_nm'))
    result['inertia'] = inertia
    result['duration'] = h
    impulse = result['impulse']
    bound = h * nominal(case, 'max_torque_nm')
    result['regime'] = ('null' if nominal(case, 'target_pos_rad') == 0 else
                        'saturated_positive' if impulse == bound else
                        'saturated_negative' if impulse == -bound else 'interior')
    return result


def measurement(value, expected, allowance, unit):
    error = value - expected
    return dict(observed=rational(value), reference=rational(expected),
        signed_error=rational(error), absolute_error=rational(abs(error)),
        allowance=rational(allowance), unit=unit,
        status='MEETS_DECLARED_NEW_CASE_ALLOWANCE' if abs(error) <= allowance else
               'OUTSIDE_DECLARED_NEW_CASE_ALLOWANCE')


def diagnostics(case, *, public_mass_word, public_inertia_word, weighted_word,
                incoming_velocity_word, integration_velocity_word, final_velocity_word,
                integration_impulse_word, final_impulse_word, coordinate_word):
    """Actual motor lambda words use opposite sign from applied physical impulse."""
    values = dict(public_mass=public_mass_word, public_inertia=public_inertia_word,
        weighted=weighted_word, incoming_velocity=incoming_velocity_word,
        integration_velocity=integration_velocity_word, final_velocity=final_velocity_word,
        integration_impulse=integration_impulse_word, final_impulse=final_impulse_word,
        coordinate=coordinate_word)
    for word in values.values(): checked_word(word)
    nonfinite = [name for name, word in values.items() if kind(word) in
                 ('nan', 'positive_infinity', 'negative_infinity')]
    if nonfinite:
        return dict(status='UNQUALIFIED_NONFINITE_OR_MISSING', nonfinite_fields=nonfinite,
                    historical_primary_reclassified=False, accuracy_or_adoption_verified=False)
    observed = {name: exact(word) for name, word in values.items()}
    if observed['weighted'] <= 0 or observed['public_inertia'] <= 0 or observed['public_mass'] <= 0:
        return dict(status='UNQUALIFIED_NONFINITE_OR_MISSING',
                    reason='nonpositive independent public inertia/mass or row inverse mass',
                    historical_primary_reclassified=False, accuracy_or_adoption_verified=False)
    model = reference(case)
    inertia, h, p = model['inertia'], model['duration'], model['impulse']
    i_allowance = max(I_ABS, I_REL * abs(inertia))
    p_allowance = max(P_ABS, P_REL * abs(p))
    v_allowance, q_allowance, tau_allowance = p_allowance/inertia, h*p_allowance/inertia, p_allowance/h
    integration_p = -observed['integration_impulse']
    final_p = -observed['final_impulse']
    results = {
        'prestep_public_inertia': measurement(observed['public_inertia'], inertia, i_allowance, 'kg m^2'),
        'first_row_inverse_mass_inertia': measurement(1/observed['weighted'], inertia, i_allowance, 'kg m^2'),
        'PGS_applied_impulse_before_integration': measurement(integration_p, p, p_allowance, 'Nm s'),
        'final_net_applied_impulse_after_stabilization': measurement(final_p, p, p_allowance, 'Nm s'),
        'geometric_inertia_times_integration_deltaV': measurement(inertia*(observed['integration_velocity']-observed['incoming_velocity']), p, p_allowance, 'Nm s'),
        'geometric_inertia_times_final_deltaV': measurement(inertia*(observed['final_velocity']-observed['incoming_velocity']), p, p_allowance, 'Nm s'),
        'integrated_coordinate': measurement(observed['coordinate'], model['coordinate'], q_allowance, 'rad'),
        'integration_input_velocity': measurement(observed['integration_velocity'], model['velocity'], v_allowance, 'rad/s'),
        'final_writeback_velocity': measurement(observed['final_velocity'], model['velocity'], v_allowance, 'rad/s'),
        'PGS_torque_over_nominal_duration': measurement(integration_p/h, model['torque'], tau_allowance, 'Nm'),
        'final_net_torque_over_nominal_duration': measurement(final_p/h, model['torque'], tau_allowance, 'Nm'),
    }
    null = None
    if model['regime'] == 'null':
        null = all(observed[name] == 0 for name in ('incoming_velocity','integration_velocity',
                    'final_velocity','integration_impulse','final_impulse','coordinate'))
    input_residuals = {}
    for field, encoded in case['input_f32_words'].items():
        input_residuals[field] = rational(exact(int(encoded,16)) - nominal(case,field))
    inertia_gate = all(results[key]['status']=='MEETS_DECLARED_NEW_CASE_ALLOWANCE' for key in
                       ('prestep_public_inertia','first_row_inverse_mass_inertia'))
    return dict(status='NEW_PHYSICAL_DIAGNOSTICS_ONLY_NOT_HISTORICAL_CALIBRATION',
        nominal_SI_reference={key:rational(value) if isinstance(value,F) else value for key,value in model.items()},
        independent_inertia_gate=inertia_gate, quantities=results,
        allowance_scope='Independent nominal SI comparisons; not a propagated f32/input/inertia uncertainty envelope or consequence of any other metric passing',
        exact_null_motion=null, signed_raw_words_preserved=values,
        nominal_to_authoritative_f32_residuals=input_residuals,
        prestep_public_mass_query_error_kg=rational(observed['public_mass']-nominal(case,'mass_kg')),
        incoming_velocity_is_registered_zero=observed['incoming_velocity']==0,
        main_physical_label=('UNQUALIFIED_NONFINITE_OR_MISSING' if not inertia_gate or observed['incoming_velocity']!=0 else
            'MEETS_DECLARED_NEW_CASE_ALLOWANCE' if all(x['status']=='MEETS_DECLARED_NEW_CASE_ALLOWANCE' for x in results.values()) and null is not False else
            'OUTSIDE_DECLARED_NEW_CASE_ALLOWANCE'),
        historical_primary_reclassified=False, accuracy_or_adoption_verified=False)
