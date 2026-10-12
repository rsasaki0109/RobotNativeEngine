#!/usr/bin/python3
"""Prepare the fixed prospective protocol and literal selected-source ledger."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

OWN = Path(__file__).resolve().parent
ROOT = Path('/workspace/RobotNativeEngine')
REC = OWN.parent

def d(p):
    p = Path(p)
    return dict(path=str(p), sha256=hashlib.sha256(p.read_bytes()).hexdigest(), bytes=p.stat().st_size)

profiles = [('split_1_2', 1, 2), ('early_1_0', 1, 0), ('early_3_0', 3, 0)]
cases = [dict(profile=profile, label=f'{profile}_{target}_{mode}', target=target, observer=observer)
    for profile, _, _ in profiles for target in ('negative', 'positive')
    for mode, observer in [('on', 'on'), ('fresh_on', 'on'), ('off', 'off')]]
protocol = dict(kind='rne_fixed_eighteen_motor_visit_placement_protocol', schema_version=1,
    study_id='motor_visit_placement_v1', created_utc=datetime.now(timezone.utc).isoformat(),
    native_processes_before_registration=0, maximum_native_processes=18,
    outer_intervals_per_world=1, cases=cases,
    profiles=[dict(profile=p, small_steps=32, pgs_passes=n, stabilization_passes=s,
        expected_constructor_checks=32, expected_coordinate_advances=32,
        expected_motor_visits=32*(n+s), expected_companion_events=32*(6+s)+1,
        expected_stage_events=32*(7+6*n+6*s+int(s>0))) for p,n,s in profiles],
    fixed_fixture=dict(radius_m_word=0x3dcccccd, mass_kg_word=0x3f800000,
        stiffness_nm_rad_word=0x43960000, damping_nm_s_rad_word=0x41400000,
        ceiling_nm_word=0x40000000, target_negative_rad_word=0xba83126f,
        target_positive_rad_word=0x3a83126f, outer_dt_s_word=0x3a83126f,
        slice_dt_s_word=0x3803126f, target_velocity_word=0, initial_coordinate_word=0,
        initial_velocity_word=0, gravity_words=[0,0,0], armature_word=0,
        passive_damping_word=0, contacts=False, sleep=False, ccd=False,
        identity_coincident_x_revolute_frames=True),
    collection_policy='One new fixture and one interval per fresh process. Preclaim all18 cases; preserve actual exits/whole output/error/partial/unlaunched cases. Stop on first failure; never retry, resume, replace, retune or append cases.',
    reader_policy='One frozen scientific reader execution after complete cohort; failures retained. Pure synthetic tests do not construct native worlds or read any completed old dataset.',
    fidelity_gate='Exact all-event identity/cache/phase/visit coverage and source-ordered f32 constructor/motor-visit/coordinate advancement reconstruction. Normal/zero arithmetic required; exclusions or incomplete/extra events make fidelity fail rather than become a successful physical result.',
    neutrality_gate='Within each profile/sign captured initial/final checkpoints are exact ON/fresh ON/OFF; complete ON/fresh completed record literal bytes equal; OFF construction/stage/impulse null. Different profile configuration metadata is never normalized to assert whole-checkpoint equality.',
    conditional_first_slice_hypotheses=[
        'After admitting matching constructor/cache/incomingV inputs, split_1_2 and early_1_0 share the first motor visit and integration input.',
        'After admitting matching fixed-row/cache map and no intervening genericV writer, split_1_2 and early_3_0 share three total visits but integrate after one versus three.',
        'Clamping, projected fixed points, odd cycles or native rounding may make first and third visits equal; unequal coordinate/velocity is not a PASS requirement.',
        'Later slices may have distinct native incoming states; first-slice identities are not extrapolated to all slices without actual conditions.'],
    interpretation='Fidelity may PASS while profile discriminator is INCONCLUSIVE. Record SI diagnostics separately with declared sphere inertia 2*m*R^2/5, independent backward-Euler trajectory and distinctly labeled native-incoming one-step counterfactuals. Row-derived1/w is not independent geometric inertia validation.',
    arithmetic_and_state_limits='Runtime MXCSR/rounding/FTZ/DAZ, full compiler/linker/registry closure, full-set metadata/private pipeline/solver/contact caches and observer buffers remain unmeasured. Source-bound postcard roundtrip is not an independent hidden-state decoder. External-force inputs and mass-matrix/LU construction are not fully captured.',
    old_evidence_policy='Completed six/72/18 cohorts, old calibration thresholds/results and rejected inverse(w+g) candidate remain immutable and are not re-executed or numerically reanalyzed.',
    adoption='No solver/vendor normalization intervention, tuning, production adoption or accuracy improvement is selected by this diagnostic. Main branch remains unchanged pending explicit #410 merge; any later implementation retains all required regression/CI gates.',
    physical_reference_preregistration=d(REC/'next-motor-actuation-reference-study/PREREGISTRATION.md'),
    physical_reference_fields=d(REC/'next-motor-actuation-reference-study/preregistration-fields-template.json'))
(OWN/'fixed-protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
binding=json.loads((OWN/'actual-build-binding.json').read_text())
sources=binding['source_descriptors'][:]
for p in sorted((ROOT/'third_party/rapier3d').rglob('*')):
    if p.is_file() and 'target' not in p.relative_to(ROOT/'third_party/rapier3d').parts and p.name!='Cargo.lock':
        sources.append(d(p))
for name in ('Cargo.toml','Cargo.lock','AGENTS.md','clippy.toml','examples/143_motor_construction/lib.rs','examples/143_motor_construction/main.rs','examples/143_motor_construction/tests.rs','examples/143_motor_construction/Cargo.toml'):
    sources.append(d(ROOT/name))
sources=sorted({x['path']:x for x in sources}.values(),key=lambda x:x['path'])
x=dict(kind='rne_visit_placement_selected_source_manifest',schema_version=1,
    baseline_head=binding['baseline_head'], baseline_tree=binding['baseline_tree'],
    actual_build_binding=d(OWN/'actual-build-binding.json'),
    actual_toolchain_qualification=d(OWN/'actual-toolchain-executable-qualification.json'),
    source_files=sources, compiled_science_source_files=binding['source_descriptors'],
    vendor_copy_proof=d(OWN/'unchanged-vendor-copy.json'),
    scope=binding['scope'],native_worlds_before_manifest=0)
(OWN/'selected-source-manifest.json').write_text(json.dumps(x,indent=2)+'\n')
print(json.dumps({'protocol':d(OWN/'fixed-protocol.json'),'source_manifest':d(OWN/'selected-source-manifest.json'),'selected_files':len(sources),'native_worlds':0}))
