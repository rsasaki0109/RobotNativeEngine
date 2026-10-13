"""Retain literal preparation exits and bind two builds; never run a collector."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import time
import tomllib

OWN = Path(__file__).resolve().parent
ROOT = Path('/workspace/RobotNativeEngine')
COLLECTOR = OWN / 'collector'

def descriptor(path):
    path = Path(path)
    data = path.read_bytes()
    return {'path': str(path), 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}

def save(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')

def sources():
    paths = [p for p in sorted(COLLECTOR.rglob('*')) if p.is_file()
             and 'pure-test-artifacts' not in p.relative_to(COLLECTOR).parts]
    assert not any('target' in p.parts for p in paths)
    for name in ('vendor_base', 'vendor_candidate'):
        paths.extend(p for p in sorted((OWN / name).rglob('*')) if p.is_file())
    paths.extend(OWN / n for n in ('baseline-cargo-config.toml', 'candidate-cargo-config.toml',
                                  'source-copy-binding.json', 'protocol-source-template.json', 'prepare_sources.py',
                                  'qualify_builds.py'))
    return [descriptor(p) for p in paths]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--attempt', type=int, default=1)
    args = parser.parse_args()
    assert args.attempt > 0
    out = OWN / f'build-qualification-attempt-{args.attempt}'
    out.mkdir()
    env = os.environ.copy()
    env.update(CARGO_BUILD_JOBS='2', CARGO_INCREMENTAL='0', RUST_TEST_THREADS='2',
               CARGO_PROFILE_DEV_DEBUG='0', CARGO_PROFILE_TEST_DEBUG='0', CARGO_PROFILE_RELEASE_DEBUG='0')
    receipts = []

    def command(label, argv, command_env=None):
        effective = env.copy() if command_env is None else command_env.copy()
        started = time.time_ns()
        receipt = {'label': label, 'argv': argv, 'cwd': str(COLLECTOR), 'start_unix_ns': started,
                   'native_fixture_invoked': False}
        save(out / f'{label}-command.json', receipt)
        with (out / f'{label}-stdout.log').open('xb') as stdout, (out / f'{label}-stderr.log').open('xb') as stderr:
            proc = subprocess.run(argv, cwd=COLLECTOR, env=effective, stdout=stdout, stderr=stderr, check=False)
        receipt.update(original_exit=proc.returncode, end_unix_ns=time.time_ns())
        save(out / f'{label}-actual.json', receipt)
        receipts.append(receipt)
        print(json.dumps({'label': label, 'original_exit': proc.returncode}), flush=True)
        if proc.returncode != 0:
            raise SystemExit(proc.returncode)
        return (out / f'{label}-stdout.log').read_text()

    command('format-collector-only', ['rustfmt', '--edition', '2021', 'lib.rs', 'main.rs', 'tests.rs'])
    command('format-check-collector-only', ['rustfmt', '--edition', '2021', '--check', 'lib.rs', 'main.rs', 'tests.rs'])
    before = sources()
    save(out / 'sources-before.json', before)
    command('rustc-version', ['rustc', '-Vv'])
    command('cargo-version', ['cargo', '-Vv'])
    command('target-cfg', ['rustc', '--print', 'cfg'])
    command('cc-version', ['cc', '--version'])
    command('ld-version', ['ld', '--version'])
    compilers = {}
    for name in ('rustc', 'cargo'):
        actual_path = command(f'actual-{name}-path', ['rustup', 'which', name]).strip()
        compilers[name] = {'invoked_shim': descriptor(Path(shutil.which(name)).resolve()),
                           'actual_toolchain_executable': descriptor(actual_path)}
    for name in ('cc', 'ld'):
        compilers[name] = {'actual_toolchain_executable': descriptor(Path(shutil.which(name)).resolve())}
    binaries = {}
    feature_bindings = {}
    environments = {}
    for variant in ('baseline', 'candidate'):
        config = str(OWN / f'{variant}-cargo-config.toml')
        cargo = ['cargo', '--config', config]
        venv = env.copy()
        venv['CARGO_TARGET_DIR'] = str(OWN / f'target_{variant}')
        metadata = json.loads(command(f'{variant}-metadata', cargo + ['metadata', '--locked', '--format-version', '1'], venv))
        rapier = next(p for p in metadata['packages'] if p['name'] == 'rapier3d')
        expected_vendor = OWN / ('vendor_base' if variant == 'baseline' else 'vendor_candidate')
        assert Path(rapier['manifest_path']) == expected_vendor / 'Cargo.toml'
        node = next(n for n in metadata['resolve']['nodes'] if n['id'] == rapier['id'])
        assert set(node['features']) == {'default', 'dim3', 'f32', 'serde-serialize'}
        feature_bindings[variant] = {'rapier_manifest': rapier['manifest_path'], 'features': node['features']}
        command(f'{variant}-features', cargo + ['tree', '--locked', '-e', 'features'], venv)
        command(f'{variant}-pure-tests', cargo + ['test', '--locked', '--lib'], venv)
        command(f'{variant}-strict-clippy', ['cargo', 'clippy', '--config', config,
                '--locked', '--all-targets', '--', '-D', 'warnings'], venv)
        docenv = venv.copy()
        docenv['RUSTDOCFLAGS'] = '-D warnings'
        command(f'{variant}-strict-rustdoc', cargo + ['doc', '--locked', '--no-deps'], docenv)
        command(f'{variant}-release-build', cargo + ['build', '--locked', '--release', '--bin', 'motor_physical'], venv)
        binary = OWN / f'motor_physical_{variant}.native'
        with binary.open('xb') as stream:
            stream.write((Path(venv['CARGO_TARGET_DIR']) / 'release/motor_physical').read_bytes())
        binary.chmod(0o555)
        binaries[variant] = descriptor(binary)
        environments[variant] = {k: venv.get(k) for k in ('CARGO_BUILD_JOBS', 'CARGO_INCREMENTAL', 'RUST_TEST_THREADS',
            'CARGO_PROFILE_DEV_DEBUG', 'CARGO_PROFILE_TEST_DEBUG', 'CARGO_PROFILE_RELEASE_DEBUG', 'CARGO_TARGET_DIR',
            'RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTDOCFLAGS', 'CC', 'CXX', 'CARGO_BUILD_TARGET')}
    after = sources()
    assert before == after, 'Qualified source bytes mutated during build'
    root_lock = tomllib.loads((ROOT / 'Cargo.lock').read_text())
    study_lock = tomllib.loads((COLLECTOR / 'Cargo.lock').read_text())
    upstream = {(p['name'], p['version'], p.get('checksum')) for p in root_lock['package']}
    diff = [(p['name'], p['version'], p.get('checksum')) for p in study_lock['package']
            if (p['name'], p['version'], p.get('checksum')) not in upstream]
    assert diff == [('motor_physical_collector', '0.1.0', None)], diff
    save(out / 'sources-after.json', after)
    binding = {'kind': 'NEW_PHYSICAL_MOTOR_ACTUAL_GUARDED_BUILDS', 'schema_version': 1,
        'created_utc': datetime.now(timezone.utc).isoformat(), 'native_worlds_before_binding': 0,
        'root_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'root_worktree_status': subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True),
        'binaries': binaries, 'source_descriptors': after, 'compiler_executables': compilers,
        'variant_features': feature_bindings, 'explicit_build_environments': environments,
        'version_checksum_differences_excluding_new_collector': [], 'original_command_receipts': receipts,
        'receipt_files': [descriptor(p) for p in sorted(out.iterdir()) if p.is_file()],
        'scope': 'Literal collector/vendor/lock/config and actual compiler executable/features/build binding; no claim of full hermetic toolchain, registry source bytes, runtime floating point modes, or native worlds.'}
    save(OWN / 'actual-build-binding.json', binding)
    print(json.dumps({'build_binding': descriptor(OWN / 'actual-build-binding.json'), 'binaries': binaries,
                      'native_worlds_launched': 0}), flush=True)

if __name__ == '__main__':
    main()
