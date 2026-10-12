"""Static source token and SHA audit only; no native/report numeric evaluation."""
import hashlib
import json
import mmap
from pathlib import Path
import re
import subprocess
from datetime import datetime, timezone

ROOT = Path('/workspace/RobotNativeEngine')
OUT = Path(__file__).parent
HEAD = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
def digest(data): return hashlib.sha256(data).hexdigest()
def desc(path):
    data = path.read_bytes()
    return dict(path=str(path), sha256=digest(data), bytes=len(data))
def git_bytes(*args): return subprocess.check_output(['git', *args], cwd=ROOT)
def old(rel): return git_bytes('show', HEAD + ':' + rel).decode()
def live(rel): return (ROOT / rel).read_text()

# Scope is a Rust token comparison of these simple native function bodies, not
# a compiler/parser equivalence proof. Comments and whitespace are discarded.
TOKEN = re.compile(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|[A-Za-z_]\w*|\d[\w.]*|::|->|=>|\.\.|!=|==|\+=|-=|\*=|/=|&&|\|\||\S')
def tokens(src): return [t for t in TOKEN.findall(src) if not t.startswith(('//', '/*'))]
def function(src, name):
    match = re.search(r'\bfn\s+' + re.escape(name) + r'\s*\(', src)
    assert match, name
    all_ = tokens(src[match.start():]); start = all_.index('{'); depth = 0
    for i in range(start, len(all_)):
        depth += (all_[i] == '{') - (all_[i] == '}')
        if depth == 0: return all_[start + 1:i]
    raise AssertionError('unterminated function')
def remove_if_blocks(seq, prefix):
    seq = list(seq); removed = 0
    while True:
        where = next((i for i in range(len(seq)) if seq[i:i+len(prefix)] == prefix), None)
        if where is None: return seq, removed
        start = seq.index('{', where); depth = 0
        for end in range(start, len(seq)):
            depth += (seq[end] == '{') - (seq[end] == '}')
            if depth == 0: break
        del seq[where:end+1]; removed += 1

unit = 'third_party/rapier3d/src/dynamics/joint/multibody_joint/unit_multibody_joint.rs'
joint = 'third_party/rapier3d/src/dynamics/joint/multibody_joint/multibody_joint.rs'
velocity = 'third_party/rapier3d/src/dynamics/solver/velocity_solver.rs'
old_unit = function(old(unit), 'unit_joint_motor_constraint')
new_unit = function(live(unit), 'unit_joint_motor_constraint_with_construction_inputs')
new_unit, removed_unit = remove_if_blocks(new_unit, ['if', 'let', 'Some', '(', 'recorder', ')', '=', 'recorder'])
assert removed_unit == 1 and old_unit == new_unit
old_integrate = function(old(velocity), 'integrate_positions')
new_integrate = function(live(velocity), 'integrate_positions_with_construction')
new_integrate, removed_integrate = remove_if_blocks(new_integrate, ['if', 'let', 'Some', '(', '(', 'context', ',', 'constraints', ',', 'bindings', ')', ')', '=', 'construction_observation'])
assert removed_integrate == 2 and old_integrate == new_integrate
assert function(old(joint), 'integrate') == function(live(joint), 'integrate')
assert function(old(velocity), 'writeback_bodies') == function(live(velocity), 'writeback_bodies')

unchanged = [
 'third_party/rapier3d/src/pipeline/multibody_motor_stage.rs',
 'third_party/rapier3d/src/pipeline/multibody_motor_impulse.rs',
 'third_party/rapier3d/src/dynamics/joint/motor_model.rs',
 'third_party/rapier3d/src/dynamics/joint/generic_joint.rs',
 'third_party/rapier3d/src/utils.rs',
 'third_party/rapier3d/src/dynamics/integration_parameters.rs',
 'third_party/rapier3d/src/dynamics/solver/joint_constraint/joint_generic_constraint.rs',
 'examples/142_contact_motor_stages/lib.rs',
 'examples/141_position_motor_calibration/main.rs',
]
for rel in unchanged: assert (ROOT / rel).read_bytes() == git_bytes('show', HEAD + ':' + rel), rel

base = OUT.parent
build = base / 'next-motor-integrated-build-binding/actual-build-binding.json'
binding = json.loads(build.read_text())
maps_path = Path(binding['selected_source_transition_maps']['path'])
maps = json.loads(maps_path.read_text()); assert len(maps['before']) == 21 and len(maps['after']) == 22
assert len(maps['changed_existing_paths']) == 11 and maps['removed_paths'] == []
for rel, expected in maps['after'].items(): assert desc(ROOT / rel)['sha256'] == expected, rel
for name in ['lib.rs', 'main.rs', 'tests.rs']:
    assert (ROOT / 'examples/143_motor_construction' / name).read_bytes() == \
        (base / 'next-motor-construction-collector/src' / name).read_bytes(), name

# Extract only the source-hash object from the authoritative report. Whole file
# hashing does not interpret any native value, case, result, criterion or scalar.
report = ROOT / 'docs/media/position-motor-calibration-motor-construction-source.json'
with report.open('rb') as file:
    with mmap.mmap(file.fileno(), 0, access=mmap.ACCESS_READ) as data:
        marker = b'"selected_source_sha256"'
        start = data.find(marker); assert start >= 0 and data.find(marker, start + 1) == -1
        begin = data.find(b'{', start); end = data.find(b'}', begin) + 1
        selected = json.loads(data[begin:end])
assert selected == maps['after']

guard_path = Path(binding['build_source_guard']['path'])
guard = json.loads(guard_path.read_text())
guard_differences = []
for rel, expected in guard.items():
    actual = desc(ROOT / rel)
    if (actual['sha256'], actual['bytes']) != (expected['sha256'], expected['size_bytes']):
        guard_differences.append(dict(path=rel, build_epoch=expected, current=actual))
assert [d['path'] for d in guard_differences] == ['xtask/src/main.rs'], guard_differences

tracked_diff = git_bytes('diff', '--binary', '--')
(OUT / 'tracked-current-diff.patch').write_bytes(tracked_diff)
paths = set(git_bytes('diff', '--name-only').decode().splitlines())
paths.update(git_bytes('ls-files', '--others', '--exclude-standard').decode().splitlines())
assert all((ROOT / p).is_file() for p in paths)
snapshot = OUT / 'reviewed-current-files'; snapshot.mkdir(exist_ok=False)
entries = []
for rel in sorted(paths):
    src = ROOT / rel
    # Reports are bound by whole bytes but their numeric payload is not copied or parsed.
    record = desc(src); record['relative_path'] = rel
    if rel.endswith(('.rs', '.toml', '.md', '.lock')):
        dst = snapshot / rel; dst.parent.mkdir(parents=True, exist_ok=True); dst.write_bytes(src.read_bytes())
        record['preserved_source_copy'] = desc(dst)
    entries.append(record)
ledger = OUT / 'current-source-and-diff-ledger.json'
ledger.write_text(json.dumps(dict(head=HEAD, entries=entries, diff=desc(OUT/'tracked-current-diff.patch'),
    unchanged_files=[desc(ROOT / p) for p in unchanged], build_binding=desc(build),
    selected_maps=desc(maps_path), authoritative_report_source_object=selected,
    whole_report=desc(report), build_guard=desc(guard_path), build_guard_differences=guard_differences,
    report_native_payload_parsed=False), indent=2)+'\n')
proof = dict(kind='independent_static_current_companion_source_preservation',
    status='PASS_STATIC_SOURCE_SCOPE_WITH_ONE_DOCUMENTATION_CORRECTION_AND_PUBLICATION_PENDING',
    completed_utc=datetime.now(timezone.utc).isoformat(), head=HEAD,
    ledger=desc(ledger), source_only=True,
    constructor_native_tokens_exact_after_removing_one_readonly_capture_block=True,
    constructor_native_token_count=len(old_unit),
    integrate_positions_native_tokens_exact_after_removing_two_readonly_endpoint_blocks=True,
    integration_native_token_count=len(old_integrate),
    multibody_joint_integrate_tokens_exact=True, writeback_bodies_tokens_exact=True,
    old_v1_stage_and_impulse_modules_literal_HEAD_exact=True,
    native_math_model_inverse_parameters_and_141_142_physical_sources_unchanged=True,
    collector_three_source_files_literal_frozen_copy_exact=True,
    current_selected_sources_all_22_live_and_frozen_and_report_map_exact=True,
    selected_transition=dict(previous=21,current=22,changed_existing=11,added=1,removed=0),
    build_guard_scope=dict(paths=len(guard), differences=guard_differences,
        interpretation='Only xtask source-report pointer changed after the native example build. This non-native CI submission change is intentional and is covered by the current running full CI guard separately; the historical build guard is not claimed identical to the whole current checkout.'),
    default_vendor_lifetime_fix='interaction_groups.rs removes only an unused explicit reference lifetime; same trait/receiver/body and no arithmetic.',
    known_documentation_issue=dict(path='third_party/rapier3d/RNE_PATCH.md',
        issue='Frozen preparation-era wording says collectors unchanged and a future qualification remains necessary. Current publication should distinguish 141 selected-source-only changes and completed cohort from the historical preparation snapshot.',
        proposed_replacement=desc(OUT/'RNE_PATCH-current-doc-replacement-proposal.md')),
    pending=['Referenced docs/evidence/motor-construction-v1 bundle not yet placed in checkout; final publication audit must bind the completed bundle/claims.',
             'Full local CI still running; no success is inferred from this source review.'],
    no_actions=dict(Cargo=0,simulations=0,scientific_reader_or_numeric_reanalysis=0,
        old_native_payload_reads=0,remote_writes=0,checkout_edits=0),
    limitations='Token comparison is a narrow static body preservation check, not compiler, full-host, private-cache, runtime-floating-mode or physical-accuracy qualification. Scientific result counts are root supplied and independently postreviewed separately; this script reads only source binding metadata, not native numeric observations.')
target = OUT / 'independent-current-source-review.json'; target.write_text(json.dumps(proof,indent=2)+'\n')
for p in [target, ledger, OUT/'tracked-current-diff.patch']: print(json.dumps(desc(p)))
