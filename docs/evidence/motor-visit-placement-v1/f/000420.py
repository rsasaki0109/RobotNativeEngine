#!/usr/bin/python3
"""Retain preparation command exits; never run the native collector."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

OWN = Path(__file__).resolve().parent
COLLECTOR = OWN / 'collector'
OUT = OWN / 'validation'
OUT.mkdir()
env = os.environ.copy()
env.update(CARGO_BUILD_JOBS='2', RUST_TEST_THREADS='2', CARGO_INCREMENTAL='0', CARGO_PROFILE_DEV_DEBUG='0', CARGO_PROFILE_TEST_DEBUG='0', CARGO_PROFILE_RELEASE_DEBUG='0', CARGO_TARGET_DIR='/workspace/RobotNativeEngine/target')
commands = [
    ('format', ['cargo', 'fmt', '--all']),
    ('fmt-check', ['cargo', 'fmt', '--all', '--', '--check']),
    ('pure-tests', ['cargo', 'test', '--locked', '--lib']),
    ('clippy', ['cargo', 'clippy', '--locked', '--all-targets', '--', '-D', 'warnings']),
    ('rustdoc', ['cargo', 'doc', '--locked', '--no-deps']),
    ('release-build', ['cargo', 'build', '--locked', '--release', '--bin', 'motor_visit_placement']),
]
for label, argv in commands:
    command_env = env.copy()
    if label == 'rustdoc':
        command_env['RUSTDOCFLAGS'] = '-D warnings'
    start = time.time_ns()
    (OUT / f'{label}-command.json').write_text(json.dumps({'argv': argv, 'cwd': str(COLLECTOR), 'start_unix_ns': start, 'explicit_nonsecret_environment': {k: command_env[k] for k in ('CARGO_BUILD_JOBS', 'RUST_TEST_THREADS', 'CARGO_INCREMENTAL', 'CARGO_PROFILE_DEV_DEBUG', 'CARGO_PROFILE_TEST_DEBUG', 'CARGO_PROFILE_RELEASE_DEBUG', 'CARGO_TARGET_DIR')}, 'native_fixture_invoked': False}, indent=2) + '\n')
    with (OUT / f'{label}-stdout.log').open('wb') as stdout, (OUT / f'{label}-stderr.log').open('wb') as stderr:
        result = subprocess.run(argv, cwd=COLLECTOR, env=command_env, stdout=stdout, stderr=stderr, check=False)
    receipt = {'argv': argv, 'original_exit': result.returncode, 'start_unix_ns': start, 'end_unix_ns': time.time_ns(), 'native_fixture_invoked': False}
    (OUT / f'{label}-actual.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps({'command': label, 'original_exit': result.returncode}), flush=True)
    if result.returncode:
        raise SystemExit(result.returncode)
binary = Path(env['CARGO_TARGET_DIR']) / 'release/motor_visit_placement'
frozen = OWN / 'motor_visit_placement.native'
with frozen.open('xb') as output:
    output.write(binary.read_bytes())
frozen.chmod(0o555)
print(json.dumps({'binary': str(frozen), 'bytes': frozen.stat().st_size, 'sha256': hashlib.sha256(frozen.read_bytes()).hexdigest(), 'native_worlds_executed': 0}), flush=True)
