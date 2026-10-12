#!/usr/bin/env python3
"""Bounded source/pure-proof snapshot; no native data or engine execution."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
from datetime import datetime, timezone

OUT = Path(__file__).parent
PROTO = OUT.parent / 'next-motor-construction-protocol'
REPO = Path('/workspace/RobotNativeEngine')
E = '776ecebbd07897f35872128cfe855cdb8d99d8e3'
def describe(path):
    b = path.read_bytes()
    return dict(path=str(path), sha256=hashlib.sha256(b).hexdigest(), bytes=len(b))
def git(*args):
    return subprocess.check_output(['git', *args], cwd=REPO, text=True).strip()

source = [
    'AGENTS.md',
    'third_party/rapier3d/src/dynamics/joint/motor_model.rs',
    'third_party/rapier3d/src/dynamics/joint/multibody_joint/unit_multibody_joint.rs',
    'third_party/rapier3d/src/dynamics/joint/generic_joint.rs',
    'third_party/rapier3d/src/utils.rs',
    'third_party/rapier3d/src/dynamics/integration_parameters.rs',
    'third_party/rapier3d/src/dynamics/joint/multibody_joint/multibody_joint.rs',
    'third_party/rapier3d/src/dynamics/joint/multibody_joint/multibody.rs',
    'third_party/rapier3d/src/dynamics/solver/velocity_solver.rs',
]
proto = ['construction_oracle.py', 'integration_oracle.py', 'test_construction_oracle.py',
    'test_integration_oracle.py', 'stage_numeric.py', 'numeric-helper-source.json', 'README.md']
for folder in ['pure-construction-tests-attempt-3', 'pure-integration-tests-attempt-2']:
    proto += [folder + '/' + name for name in ['actual-pure-tests.json',
               'actual-tests.stdout.log', 'actual-tests.stderr.log']]

head = git('rev-parse', 'HEAD')
assert head == E or head.startswith('3be26ef0')
assert git('status', '--porcelain=v1') == ''
tree = git('rev-parse', head + '^{tree}')
assert tree == git('rev-parse', E + '^{tree}')
snapshot = OUT / 'reviewed-final-version'
snapshot.mkdir(exist_ok=False)
entries = []
for group, base, paths in [('prototype', PROTO, proto), ('source', REPO, source)]:
    for rel in paths:
        p = base / rel
        d = snapshot / group / rel
        d.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(p, d)
        original, copied = describe(p), describe(d)
        assert original['sha256'] == copied['sha256'] and original['bytes'] == copied['bytes']
        entry = dict(group=group, relative_path=rel, original=original, preserved_copy=copied)
        if group == 'source':
            e = subprocess.check_output(['git', 'show', E + ':' + rel], cwd=REPO)
            assert hashlib.sha256(e).hexdigest() == original['sha256']
            entry['exact_historical_E_git_blob_bytes'] = True
        entries.append(entry)

assert describe(PROTO / 'stage_numeric.py')['sha256'] == \
    '9ede5d788bd5bfb2de5d604cba53ccd872acb8796cd45641865c5a5b3a7a46cc'
root_tests = []
for folder, count in [('pure-construction-tests-attempt-3', 10),
                      ('pure-integration-tests-attempt-2', 6)]:
    p = PROTO / folder / 'actual-pure-tests.json'
    receipt = json.loads(p.read_text())
    assert receipt['tests_run'] == count and receipt['actual_exit_code'] == 0
    assert receipt['failures'] == receipt['errors'] == receipt['native_worlds_launched'] == 0
    assert receipt['native_data_read'] is False
    root_tests.append(dict(receipt=describe(p), actual_count=count, actual_exit_code=0))

probes_file = OUT / 'independent-final-pure-probe-results.json'
probes = json.loads(probes_file.read_text())
assert probes['assertions_passed'] == len(probes['probes']) == 41
assert all(r['assertion_passed'] for r in probes['probes'])
initial = OUT / 'independent-initial-prototype-source-review.json'
assert describe(initial)['sha256'] == '3bd4e7aa3376000a8a2e4f0097707d265de64be7d098dfbeadd841281c983c9c'
for entry in entries:
    assert describe(Path(entry['original']['path'])) == entry['original']
assert git('status', '--porcelain=v1') == ''

ledger = OUT / 'reviewed-final-version-source-ledger.json'
ledger.write_text(json.dumps(dict(kind='bounded_source_and_pure_receipt_snapshot',
    actual_compiler_dependency_closure=False, capture_after_pure_probes=True,
    entries=entries), indent=2) + '\n')
proof = dict(
    kind='independent_corrected_prospective_constructor_and_scalar_endpoint_source_review',
    status='PASS_SOURCE_ONLY_CORRECTED_PROTOTYPES',
    completed_utc=datetime.now(timezone.utc).isoformat(), material_blockers_in_declared_scope=[],
    source_epoch=dict(historical_head_E=E, observed_head_at_completion=head,
        observed_head_tree=tree, exact_tree_equal_to_E=True, checkout_clean=True,
        intentional_transition='User-approved PR409 squash merge and branch reset to origin/main. HEAD identity changed while all reviewed source bytes and full tree remain equal to E.'),
    final_constructor=describe(snapshot / 'prototype/construction_oracle.py'),
    final_endpoint_helper=describe(snapshot / 'prototype/integration_oracle.py'),
    source_ledger=describe(ledger), retained_initial_review=describe(initial),
    root_actual_pure_test_receipts=root_tests,
    independent_pure_probes=dict(source=describe(OUT / 'independent_final_pure_probes.py'),
        result=describe(probes_file), actual_command='python -B independent_final_pure_probes.py',
        actual_cwd=str(OUT), actual_tool_command_exit_code=0, assertions_passed=41,
        native_worlds_launched=0, native_data_read=False),
    fixed_findings=[
      dict(id='R1', confirmed=True,
           result='Captured inverse_duration_word is parsed and compared strictly as an output, excluded from arithmetic_inputs. A NaN output with eligible finite true inputs is MISMATCH, not input exclusion.',
           refs=['construction_oracle.py:34-36,46-57,81-87']),
      dict(id='R2', confirmed=True,
           result='Finite overflow retains EXCLUDED_SOURCE_FLOAT_DOMAIN and overflow_intermediate instead of losing the result to an unstructured ValueError. The try scope is arithmetic only; malformed input parsing remains outside, and errors without recorded arithmetic exclusions are re-raised.',
           refs=['construction_oracle.py:27-70,71-80']),
      dict(id='Independent_bound_order', confirmed=True,
           result='cap*h is now evaluated after coefficients and before RHS as native motor_params does. Modeled expression order agrees; operation_count remains a modeled expression count, not the entire native mass/constructor trace.',
           refs=['construction_oracle.py:94-112', 'generic_joint.rs:183-195', 'unit_multibody_joint.rs:81-109']),
    ],
    endpoint_source_review=[
      dict(topic='Actual copied integration velocity',
           result='The selected full generic velocity block is copied into multibody.velocities before multibody.integrate. Helper strictly compares the selected generic word with the captured before multibody word.',
           refs=['velocity_solver.rs:456-465', 'integration_oracle.py:34-43,62-67']),
      dict(topic='Isolated angular coordinate update',
           result='For fixed-root one-DOF AngX, Multibody::integrate passes the link velocity slice and MultibodyJoint::integrate advances coords[DIM+0] by v*dt. The helper performs one rounded multiply followed by one rounded add; it does not reconstruct q from final stabilized V.',
           refs=['multibody.rs:1037-1041', 'multibody_joint.rs:89-117', 'integration_oracle.py:31-33,53-54']),
      dict(topic='Velocity and identity retention',
           result='Integration leaves the multibody velocity array unchanged. Helper compares before/after selected velocity, exact matching row identity, dictionary native handle {index,generation}, u32 handle words, non-bool bounded integer indices, one DOF, ForceBased AngX. Changed final V, mixed identity and wrong axis/model/DOF are rejected or retained as mismatches.',
           refs=['multibody_joint.rs:89-117', 'integration_oracle.py:17-43,62-67']),
      dict(topic='Float exclusions and signed zeros',
           result='Both helpers retain normal-or-zero scope, tiny/subnormal/overflow exclusion, and literal output word mismatches. Finite f32 limit clamp remains distinct from generic impulse clamp. Selected nonfinite computed coordinate/after-velocity outputs are MISMATCH for eligible inputs.',
           refs=['construction_oracle.py:103-112', 'integration_oracle.py:44-72', 'stage_numeric.py:129-208']),
    ],
    limitations=[
      'This source/pure PASS is not a frozen preregistration, actual observer/producer qualification, whole event/schema/replay or source/compiler closure verifier. Only bounded files and synthetic opaque words were examined.',
      'Caller must bind actual BeforePositionIntegration after generic-V copy and before coordinate advancement, actual AfterPositionIntegration immediately after advancement, actual native coords[3], multibody root/link, cache and slice identity, event schedule, fixed-root topology, scalar width and compiler/runtime mode. Matching row metadata cannot alone establish those facts.',
      'Constructor helper still deliberately checks only ERP/RHS/CFM/bounds. It does not admit complete row shape, original motor cap/model binding, actual MotorParameters, initial impulse +0, inv_lhs, full J/W arrays, joint/root/DOF identity or full solver offsets. Self-consistent extra malformed row fields can still match narrow formulas; whole-cohort admission must reject them separately.',
      'Endpoint helper checks only the declared scalar native block/coordinate and selected generic velocity word; unrelated generic blocks and full private state are not validated.',
      'Malformed schema errors remain errors. An arithmetic error after a recorded float-domain exclusion returns EXCLUDED rather than PASS; whole-cohort retention must count and preserve every excluded/failed/missing record without resampling.',
      'No full hidden-state/private-cache equivalence, RHS origin, per-contact causality, actual runtime MXCSR/FTZ/DAZ or universal native operation order is established.',
      'No motor physical accuracy, convergence, candidate adoption, new engine defect, hardware/external validation, or maturity improvement follows. Existing numerical evidence and historical failed outcomes were not read or reinterpreted.',
    ],
    readme_claim_discipline='Explicitly preparation, no native worlds, unchanged prior outcomes, pending freeze/cohort/producer bindings, initial review separated from later bytes and count limited to modeled expressions.',
    actions=dict(checkout_edits=0, root_prototype_edits=0, cargo_or_native_engine_runs=0,
        native_worlds=0, historical_native_values_or_reports_read=False,
        github_queries_or_mutations=0, independent_ci_monitor_started=False),
    qualification_versioning='This receipt binds only copied final prototype/source bytes and the actual referenced pure receipts. Any later source/schema/producer/reader change requires separate review.',
)
dest = OUT / 'independent-final-prototype-source-review.json'
dest.write_text(json.dumps(proof, indent=2) + '\n')
for path in [dest, ledger, snapshot / 'prototype/construction_oracle.py',
             snapshot / 'prototype/integration_oracle.py']:
    print(json.dumps(describe(path)))
