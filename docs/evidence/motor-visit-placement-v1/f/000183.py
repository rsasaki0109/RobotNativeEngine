#!/usr/bin/python3
"""Check displays of captured words/existing rational values; no physical derivation."""
from fractions import Fraction
import hashlib,json
from pathlib import Path
import struct
from datetime import datetime,timezone
OWN=Path(__file__).resolve().parent
STUDY=OWN.parent/'next-motor-visit-placement-18'
def desc(p):
    p=Path(p);r=p.read_bytes();return dict(path=str(p),sha256=hashlib.sha256(r).hexdigest(),bytes=len(r))
def display_word(w):return format(struct.unpack('<f',struct.pack('<I',w))[0],'.9g')
def display_rational(r):return format(float(Fraction(int(r['numerator']),int(r['denominator']))),'.9g')
audit_path=STUDY/'actual-cohort-audit.json';assert desc(audit_path)['sha256']=='d7022bb982ab997862da1f3aabbfa4306a6fa568b4707a2b2a9aa0bd9581acfe'
a=json.loads(audit_path.read_bytes());summary_path=STUDY/'result-summary.json';s=json.loads(summary_path.read_bytes());readme=(STUDY/'README.md').read_text()
assert s['audit_binding']==desc(audit_path) and s['fidelity_status']==a['fidelity_status']
assert s['original_processes']==a['actual_processes']==18
assert s['constructor_checks']==a['constructor_checks']==384 and s['coordinate_checks']==a['coordinate_advance_checks']==384 and s['motor_solve_checks']==a['native_motor_solve_checks']==896
assert s['first_slice_discriminator']==a['discriminator'] and not s['physical_accuracy_verified'] and not s['production_adoption']
representatives=[c for c in a['cases'] if c['case']['observer']=='on' and c['case']['label'].endswith('_on')]
assert len(representatives)==len(s['last_slice_rows'])==6
native_displays=rational_displays=0
for case,row in zip(representatives,s['last_slice_rows']):
    assert row['case']==case['case'] and row['internal_slice_index']==case['analyses'][-1]['slice']==31
    last=case['analyses'][-1]
    for key,label in [('q_after_rad_word','q_rad'),('integration_input_velocity_rad_s_word','integration_velocity_rad_s'),('final_writeback_velocity_rad_s_word','writeback_velocity_rad_s')]:
        assert row['native_words'][key]==last[key] and row[label]==display_word(last[key]);native_displays+=1
    si=last['physical_SI']
    for label,rational in [('independent_SI_q_rad',si['independent_reference_trajectory']['coordinate']),('independent_SI_v_rad_s',si['independent_reference_trajectory']['velocity']),('native_q_minus_independent_SI_rad',si['native_minus_independent_q']),('native_writeback_v_minus_independent_SI_rad_s',si['native_final_minus_independent_v'])]:
        assert row[label]==display_rational(rational);rational_displays+=1
    if row['case']['target']=='positive':assert f"| {row['case']['profile']} | {row['q_rad']} | {row['integration_velocity_rad_s']} | {row['writeback_velocity_rad_s']} |" in readme
assert 'INCONCLUSIVE for both signs' in readme and 'These are observations in this fixture' in readme and 'no solver normalization or accuracy criterion was adopted' in readme
assert 'Public retention has not been completed' in readme and '5,447,072-byte' in readme and desc(audit_path)['sha256'] in readme
assert 'This work does not increase practical maturity' in readme
ledger=json.loads((OWN/'whole-file-ledger.json').read_bytes());by={d['path']:d for d in ledger['whole_files']}
for p in (STUDY/'make_result_summary.py',summary_path,STUDY/'README.md'):
    assert by[str(p)]==desc(p)
x=dict(kind='independent_presentation_only_word_and_existing_rational_display_binding',status='PASS_CAPTURED_AUDIT_DISPLAY_BINDINGS',created_utc=datetime.now(timezone.utc).isoformat(),audit=desc(audit_path),presentation_whole_files=[desc(p) for p in (STUDY/'make_result_summary.py',summary_path,STUDY/'README.md')],already_in_primary_whole_ledger=True,representative_rows=6,fixed_slice_index=31,native_word_displays_verified=native_displays,existing_rational_displays_verified=rational_displays,README_positive_rows_verified=3,first_slice_INCONCLUSIVE_claim_preserved=True,native_or_scientific_reader_reruns=0,physical_equations_or_SI_trajectories_rederived=False,new_scientific_predicates_or_selection=False,limits=['Only formatting already captured words and existing exact-rational audit values; no new physical model, accuracy verdict or adoption.','Descriptive last-slice differences do not change first-slice INCONCLUSIVE or prove all32 profiles identical.','Public retention remains unverified; no Git/Release action.'],issues=[])
out=OWN/'presentation-binding-supplement.json'
with out.open('x') as f:json.dump(x,f,indent=2);f.write('\n')
print(json.dumps(desc(out),sort_keys=True))
