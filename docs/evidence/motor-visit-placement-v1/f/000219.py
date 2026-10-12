#!/usr/bin/python3
"""Bind an actual guarded build and its explicit source/feature scope."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import tomllib

OWN = Path(__file__).resolve().parent
COLLECTOR = OWN / 'collector'
ROOT = Path('/workspace/RobotNativeEngine')
OUT = OWN / 'actual-build-binding'
OUT.mkdir()

def descriptor(p):
    p = Path(p)
    return dict(path=str(p), sha256=hashlib.sha256(p.read_bytes()).hexdigest(), bytes=p.stat().st_size)

def sources():
    paths = [COLLECTOR / n for n in ('Cargo.toml', 'Cargo.lock', 'lib.rs', 'main.rs', 'tests.rs', 'clippy.toml')]
    paths += [p for p in sorted((OWN / 'rapier3d').rglob('*')) if p.is_file()]
    return [descriptor(p) for p in paths]

env = os.environ.copy()
env.update(CARGO_BUILD_JOBS='2', RUST_TEST_THREADS='2', CARGO_INCREMENTAL='0', CARGO_PROFILE_DEV_DEBUG='0', CARGO_PROFILE_TEST_DEBUG='0', CARGO_PROFILE_RELEASE_DEBUG='0', CARGO_TARGET_DIR=str(ROOT / 'target'))
before = sources()
(OUT / 'sources-before.json').write_text(json.dumps(before, indent=2) + '\n')

def command(name, argv):
    start = time.time_ns()
    with (OUT / f'{name}-stdout.log').open('xb') as out, (OUT / f'{name}-stderr.log').open('xb') as err:
        p = subprocess.run(argv, cwd=COLLECTOR, env=env, stdout=out, stderr=err, check=False)
    x = dict(argv=argv, cwd=str(COLLECTOR), original_exit=p.returncode, start_unix_ns=start, end_unix_ns=time.time_ns())
    (OUT / f'{name}-actual.json').write_text(json.dumps(x, indent=2) + '\n')
    if p.returncode:
        raise SystemExit(p.returncode)
    return x

actual = {}
for name, argv in [
    ('rustc', ['rustc', '-Vv']), ('cargo', ['cargo', '-Vv']),
    ('target-cfg', ['rustc', '--print', 'cfg']), ('cc', ['cc', '--version']),
    ('ld', ['ld', '--version']),
    ('metadata', ['cargo', 'metadata', '--locked', '--format-version', '1']),
    ('features', ['cargo', 'tree', '--locked', '-e', 'features']),
    ('guarded-release-build', ['cargo', 'build', '--locked', '--release', '--bin', 'motor_visit_placement']),
]:
    actual[name] = command(name, argv)
after = sources()
assert before == after, 'Source mutation during qualified build'
(OUT / 'sources-after.json').write_text(json.dumps(after, indent=2) + '\n')
metadata = json.loads((OUT / 'metadata-stdout.log').read_text())
rapier = next(p for p in metadata['packages'] if p['name'] == 'rapier3d')
node = next(n for n in metadata['resolve']['nodes'] if n['id'] == rapier['id'])
assert set(node['features']) == {'default', 'dim3', 'f32', 'serde-serialize'}, node['features']
root_lock = tomllib.loads((ROOT / 'Cargo.lock').read_text())
study_lock = tomllib.loads((COLLECTOR / 'Cargo.lock').read_text())
baseline = {(p['name'], p['version'], p.get('checksum')) for p in root_lock['package']}
differences = [(p['name'], p['version'], p.get('checksum')) for p in study_lock['package'] if (p['name'], p['version'], p.get('checksum')) not in baseline]
assert differences == [('motor_visit_placement_collector', '0.1.0', None)], differences
binary = OWN / 'motor_visit_placement.native'
assert binary.read_bytes() == (ROOT / 'target/release/motor_visit_placement').read_bytes()
compiler_paths = {}
for name in ('rustc', 'cargo', 'cc', 'ld'):
    location = shutil.which(name, path=env['PATH'])
    compiler_paths[name] = dict(invoked_path=location, resolved_executable=descriptor(Path(location).resolve()))
binding = dict(kind='rne_visit_placement_actual_guarded_build', schema_version=1,
    created_utc=datetime.now(timezone.utc).isoformat(), native_worlds_before_binding=0,
    baseline_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
    baseline_tree=subprocess.check_output(['git', 'rev-parse', 'HEAD^{tree}'], cwd=ROOT, text=True).strip(),
    baseline_worktree_status=subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True),
    binary=descriptor(binary), source_descriptors=after,
    rapier_features=node['features'], root_lock_noncollector_version_checksum_differences=[],
    commands=actual, compiler_executables=compiler_paths,
    explicit_build_environment={k: env.get(k) for k in ('CARGO_BUILD_JOBS', 'RUST_TEST_THREADS', 'CARGO_INCREMENTAL', 'CARGO_PROFILE_DEV_DEBUG', 'CARGO_PROFILE_TEST_DEBUG', 'CARGO_PROFILE_RELEASE_DEBUG', 'CARGO_TARGET_DIR', 'RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTDOCFLAGS', 'CC', 'CXX', 'CARGO_BUILD_TARGET')},
    receipt_files=[descriptor(p) for p in sorted(OUT.iterdir()) if p.is_file()],
    scope='Collector and entire123-file unchanged vendored-source copy, exact Cargo lock/version-checksum graph, explicit metadata/features/compiler paths and commands. Does not attest every compiler/linker internal library, platform runtime or registry source byte, nor runtime MXCSR/rounding/FTZ/DAZ. All profiles use this identical binary.')
(OWN / 'actual-build-binding.json').write_text(json.dumps(binding, indent=2) + '\n')
print(json.dumps({'binding': descriptor(OWN / 'actual-build-binding.json'), 'binary': descriptor(binary), 'source_files': len(after), 'native_worlds': 0}))
