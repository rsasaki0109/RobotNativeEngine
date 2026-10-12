#!/usr/bin/env python3
"""Persist a source-only, version-specific review. Never load native reports."""
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
import shutil
import subprocess

OUT = Path(__file__).parent
PROTO = OUT.parent / 'next-motor-construction-protocol'
REPO = Path('/workspace/RobotNativeEngine')
source_paths = [
    'third_party/rapier3d/src/dynamics/joint/motor_model.rs',
    'third_party/rapier3d/src/dynamics/joint/multibody_joint/unit_multibody_joint.rs',
    'third_party/rapier3d/src/dynamics/joint/generic_joint.rs',
    'third_party/rapier3d/src/utils.rs',
    'third_party/rapier3d/src/dynamics/integration_parameters.rs',
    'AGENTS.md',
]
prototype_paths = [
    'construction_oracle.py', 'test_construction_oracle.py', 'stage_numeric.py',
    'numeric-helper-source.json', 'README.md',
    'pure-tests-attempt-1/actual-pure-tests.json',
    'pure-tests-attempt-1/actual-tests.stdout.log',
    'pure-tests-attempt-1/actual-tests.stderr.log',
]
def descriptor(path):
    data = path.read_bytes()
    return dict(path=str(path), sha256=hashlib.sha256(data).hexdigest(), bytes=len(data))

snapshot = OUT / 'reviewed-initial-version'
snapshot.mkdir(exist_ok=False)
entries = []
for base, paths, group in [(PROTO, prototype_paths, 'prototype'), (REPO, source_paths, 'source')]:
    for rel in paths:
        src = base / rel
        dst = snapshot / group / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src, dst)
        original, copied = descriptor(src), descriptor(dst)
        assert original['sha256'] == copied['sha256'] and original['bytes'] == copied['bytes']
        entries.append(dict(group=group, relative_path=rel, original=original, preserved_copy=copied))

head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=REPO, text=True).strip()
status = subprocess.check_output(['git', 'status', '--porcelain=v1'], cwd=REPO, text=True)
assert head == '776ecebbd07897f35872128cfe855cdb8d99d8e3' and status == ''
root_tests = json.loads((PROTO / 'pure-tests-attempt-1/actual-pure-tests.json').read_text())
assert root_tests['tests_run'] == 8 and root_tests['actual_exit_code'] == 0
assert root_tests['failures'] == root_tests['errors'] == root_tests['native_worlds_launched'] == 0
numeric = json.loads((PROTO / 'numeric-helper-source.json').read_text())
assert descriptor(PROTO / 'stage_numeric.py')['sha256'] == numeric['sha256'] == \
       '9ede5d788bd5bfb2de5d604cba53ccd872acb8796cd45641865c5a5b3a7a46cc'
probes_path = OUT / 'independent-pure-probe-results.json'
probes = json.loads(probes_path.read_text())
assert probes['assertions_passed'] == 21 and len(probes['probes']) == 21
assert all(p['assertion_passed'] for p in probes['probes'])
for entry in entries:
    assert descriptor(Path(entry['original']['path'])) == entry['original']

ledger = OUT / 'reviewed-initial-version-source-ledger.json'
ledger.write_text(json.dumps(dict(kind='nonexhaustive_bounded_source_review_snapshot',
    captured_after_pure_probes=True, not_pre_run_compiler_closure=True,
    entries=entries), indent=2) + '\n')
proof = dict(
    kind='independent_prospective_constructor_prototype_source_review',
    status='SOURCE_REVIEW_COMPLETE_TWO_PRE_DATA_RETURN_CONTRACT_CORRECTIONS',
    completed_utc=datetime.now(timezone.utc).isoformat(),
    reviewed_head=head, checkout_clean=True,
    reviewed_prototype=descriptor(snapshot / 'prototype/construction_oracle.py'),
    source_ledger=descriptor(ledger),
    reviewed_originals_retained=True,
    root_pure_tests=dict(receipt=descriptor(PROTO / 'pure-tests-attempt-1/actual-pure-tests.json'),
                        actual_tests=8, actual_exit_code=0, failures=0, errors=0),
    independent_pure_probes=dict(source=descriptor(OUT / 'independent_pure_probes.py'),
        results=descriptor(probes_path), actual_command='python -B independent_pure_probes.py',
        actual_cwd=str(OUT), actual_tool_command_exit_code=0, assertions=21,
        scope='Synthetic opaque word fixtures only. Probe passes establish observations, not prototype qualification.'),
    checks=[
      dict(topic='Coefficient arithmetic', conclusion='Finite normal/zero expressions agree with source, including left-associated dt*dt*k, separately rounded additions and inclusive utils::inv threshold.',
           refs=['motor_model.rs:28-35', 'utils.rs:25-32', 'construction_oracle.py:74-75']),
      dict(topic='Different inverse guards', conclusion='Integration inverse uses exactly dt==0 then +0; other finite dt uses 1/dt. It must not be replaced by utils::inv. Both signed dt zeros match the source prototype.',
           refs=['integration_parameters.rs:113-118', 'construction_oracle.py:73']),
      dict(topic='Velocity limit clamp', conclusion='Manual strict finite comparisons preserve equal signed zero, matching native f32::clamp. The generic impulse clamp chooses a boundary for equality and must not be reused here.',
           refs=['unit_multibody_joint.rs:101-109', 'construction_oracle.py:80-89', 'stage_numeric.py:196-202']),
      dict(topic='RHS and gain model', conclusion='Conditional positional RHS term, unary negation of effective velocity, copied RHS/RHS-without-bias and ForceBased versus AccelerationBased CFM placement match covered source formulas.',
           refs=['unit_multibody_joint.rs:96-122', 'generic_joint.rs:183-195']),
      dict(topic='Zero cap and signed zeros', conclusion='Both positive and negative zero ceilings/durations preserve native multiplication/unary-negation bound words; expected outputs compare literal words.',
           refs=['generic_joint.rs:194', 'unit_multibody_joint.rs:94', 'construction_oracle.py:90-103']),
    ],
    required_pre_data_corrections=[
      dict(id='R1', severity='Prospective return-contract defect, not an engine defect',
           finding='inverse_duration_word is a captured computed output but is included in arithmetic_inputs. With all true inputs finite, a NaN captured inverse is classified as nonfinite_arithmetic_input and no mismatch is retained; RHS NaN already produces MISMATCH.',
           refs=['construction_oracle.py:34-36,46-65'],
           repro='independent probe nonfinite_captured_inverse_is_currently_input_exclusion',
           correction='Remove inverse_duration_word from input-domain accounting while retaining strict word parsing and output equality. Nonfinite captured outputs for eligible true inputs must remain mismatches.'),
      dict(id='R2', severity='Prospective explicit-failure retention defect, not a false PASS',
           finding='Finite normal h=k=0x7f7fffff produces overflow in h*k. The helper marks overflow but inverse attempts exact(infinity), raising ValueError with no structured exclusion result returned.',
           refs=['construction_oracle.py:74', 'stage_numeric.py:149-176,204-208'],
           repro='independent probe finite_overflow_has_no_structured_exclusion_return',
           correction='Retain an explicit arithmetic-domain exclusion/failure record carrying the captured inputs/outputs and accumulated exclusions. Do not swallow malformed schema or other errors into PASS.'),
    ],
    scope_and_freeze_requirements=[
      'This is a draft narrow ERP/RHS/CFM/bounds verifier, not a frozen preregistration, collector, complete construction-row or cohort/replay admission verifier.',
      'The prototype intentionally does not validate row identity, native frame/DOF, full J/W blocks, solver offsets, initial impulse +0, or inv_lhs=utils::inv(actual solved weighted selector). Synthetic inconsistent extra fields still receive narrow DECLARED_FORMULAS_MATCH; a future exact schema and source-bound producer must reject them before any whole-construction claim.',
      'The cap/model currently come from the nested row; future producer/reader must bind them independently to the actual JointMotor.max_force/model and MotorParameters.max_impulse/target fields at the build site, so a self-consistent but wrong cap cannot masquerade as source fidelity.',
      'Actual full event order evaluates max_impulse and bounds before RHS; the prototype evaluates independent bound multiplication afterward. Covered output words agree, but operation_count is not a native ordered-operation trace. Either match this order before freeze or limit the claim to each expression order.',
      'Nonnegative duration/cap and finite-normal-or-zero qualification restrictions are reader eligibility choices; source consistency alone does not establish valid physical gains or plant accuracy. Native nonfinite limit/cap cases are conservatively excluded, not interpreted as zero.',
      'Before collection freeze actual constructor locals, availability, every expected slice/build, joint/root/cache identity, endpoint timing, source/features/compiler/environment flags, binary and complete-failure retention. BeforePositionIntegration must be the actual copied multibody velocity before q advance, not final stabilized velocity.',
      'No runtime floating point mode/MXCSR/FTZ/DAZ, private-cache full state equivalence, RHS-origin/contact attribution, physical motor accuracy, solver convergence, candidate adoption or maturity claim is established.',
    ],
    claim_discipline=dict(readme_explicitly_preparation=True, native_collection_worlds=0,
        native_simulation_runs_by_auditor=0, cargo_builds_by_auditor=0,
        historical_native_values_or_reports_read=False, repository_or_prototype_edits=False,
        network_calls_by_auditor=0, engine_defect_or_accuracy_qualification=False),
    review_versioning='Only the preserved initial prototype is reviewed. Any subsequent repair or integration helper needs its own source/tamper review; this receipt cannot qualify later bytes.',
)
target = OUT / 'independent-initial-prototype-source-review.json'
target.write_text(json.dumps(proof, indent=2) + '\n')
print(json.dumps(descriptor(target)))
print(json.dumps(descriptor(ledger)))
print(json.dumps(descriptor(snapshot / 'prototype/construction_oracle.py')))
