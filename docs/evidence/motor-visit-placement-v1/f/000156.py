#!/usr/bin/python3
"""Display already registered audit values; no new equations or accuracy gate."""
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import struct

OWN = Path(__file__).resolve().parent
source = OWN / 'actual-cohort-audit.json'
raw = source.read_bytes()
audit = json.loads(raw)

def display_word(word):
    return format(struct.unpack('<f', struct.pack('<I', word))[0], '.9g')

def display_rational(value):
    return format(float(Fraction(int(value['numerator']), int(value['denominator']))), '.9g')

rows = []
for case in audit['cases']:
    if case['case']['observer'] != 'on' or not case['case']['label'].endswith('_on'):
        continue
    last = case['analyses'][-1]
    words = {key:last[key] for key in ('q_after_rad_word', 'integration_input_velocity_rad_s_word', 'final_writeback_velocity_rad_s_word')}
    si = last['physical_SI']
    rows.append(dict(case=case['case'], internal_slice_index=last['slice'], native_words=words,
        q_rad=display_word(words['q_after_rad_word']),
        integration_velocity_rad_s=display_word(words['integration_input_velocity_rad_s_word']),
        writeback_velocity_rad_s=display_word(words['final_writeback_velocity_rad_s_word']),
        independent_SI_q_rad=display_rational(si['independent_reference_trajectory']['coordinate']),
        independent_SI_v_rad_s=display_rational(si['independent_reference_trajectory']['velocity']),
        native_q_minus_independent_SI_rad=display_rational(si['native_minus_independent_q']),
        native_writeback_v_minus_independent_SI_rad_s=display_rational(si['native_final_minus_independent_v'])))
result=dict(kind='presentation_of_fixed_registered_audit_only', schema_version=1,
    audit_binding=dict(path=str(source),bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest()),
    fidelity_status=audit['fidelity_status'], original_processes=audit['actual_processes'],
    constructor_checks=audit['constructor_checks'], coordinate_checks=audit['coordinate_advance_checks'],
    motor_solve_checks=audit['native_motor_solve_checks'], first_slice_discriminator=audit['discriminator'],
    last_slice_rows=rows,
    scope='Nine significant digit displays of captured native words and existing exact-rational audit values for the last slice of every profile/sign representative ON. Fresh ON and OFF remain retained and already qualified by the one registered reader. No new model derivation, acceptance threshold, resampling, reader execution or accuracy verdict.',
    physical_accuracy_verified=False, production_adoption=False)
with (OWN/'result-summary.json').open('x') as f:
    json.dump(result,f,indent=2);f.write('\n')
text='''# Fixed eighteen motor visitation-placement diagnostic

This study uses the unchanged source at PR #410 head
`5e8e3627cd91df767ae41e8364f9401a8cd4e2d7`. It ran in an external source copy
while #410 awaited explicit merge approval; no repository source was changed.
It is a new eighteen-process budget, separate from every completed old cohort.

All eighteen original native exits, the coordinator, checked CLI, root collection
CLI and the one frozen reader exit are zero. The admitted reader verified
384 constructors, 384 coordinate advances and 896 motor visits, with no arithmetic
exclusion or mismatch. Captured ON/fresh ON/OFF checkpoints are exact; fresh ON
completed-record bytes match; OFF emits no observer payload.

The preregistered first-slice discriminator is INCONCLUSIVE for both signs.
Clipping/odd-cycle equality was allowed before collection; this is not a failed
fidelity gate or an accuracy improvement. The full interval retains later
differences. The following positive-target values display the last slice of the
one 1 ms interval; the JSON summary also displays every negative-target profile.

| Profile | q after interval (rad) | V used for last position advance (rad/s) | V written back (rad/s) |
|---|---:|---:|---:|
'''
for row in rows:
    if row['case']['target']=='positive':
        text+=f"| {row['case']['profile']} | {row['q_rad']} | {row['integration_velocity_rad_s']} | {row['writeback_velocity_rad_s']} |\n"
text+='''
For this fixed isolated fixture, removing the two postintegration stabilization
visits preserves the captured final q and makes writeback V equal the integration
input. Moving the same total three visits before integration changes the captured
final q. These are observations in this fixture, not a general coupled-contact
cause or evidence that any profile should replace production settings. Geometric
backward-Euler SI references are diagnostics only; the old calibration remains
unchanged and no solver normalization or accuracy criterion was adopted.

The complete audit is retained as one 5,447,072-byte JSON file with SHA-256
`d7022bb982ab997862da1f3aabbfa4306a6fa568b4707a2b2a9aa0bd9581acfe`.
It exceeds ADR 034's inline threshold. Public retention has not been completed;
do not commit that file inline or reuse prior large-asset release authorization.
All raw files and original failure/preparation receipts remain local whole files.

Runtime floating-point modes, full compiler/registry closure, private world/
pipeline/contact caches, mass-matrix/LU construction and physical hardware remain
outside the admitted scope. This work does not increase practical maturity.
'''
with (OWN/'README.md').open('x') as f:
    f.write(text)
print(json.dumps({'summary':str(OWN/'result-summary.json'),'rows':rows,'new_native_or_scientific_reader_invocations':0}))
