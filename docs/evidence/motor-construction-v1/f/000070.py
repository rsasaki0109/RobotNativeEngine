"""Exclusive, retained test receipts for synthetic stub-process driver tests."""

import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parent
FILES = {'driver_source': ROOT / 'driver.py', 'comparator_source': ROOT.parent / 'compare_scalar_source_transition.py',
         'test_source': ROOT / 'test_driver.py', 'test_runner_source': ROOT / 'run_tests.py'}


def desc(path):
    raw = path.read_bytes()
    return {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest(), 'size_bytes': len(raw)}


def save(path, value):
    with path.open('xb') as stream:
        stream.write((json.dumps(value, indent=2, sort_keys=True) + '\n').encode())
        stream.flush(); os.fsync(stream.fileno())


def main():
    destination = ROOT / sys.argv[1]
    destination.mkdir(exist_ok=False)
    fixtures = destination / 'SYNTHETIC-STUB-FIXTURES-NOT-SCIENTIFIC-DATA'
    fixtures.mkdir()
    before = {role: desc(path) for role, path in FILES.items()}
    save(destination / 'tools-before.json', before)
    started = datetime.datetime.now(datetime.timezone.utc).isoformat()
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', SYNTHETIC_FIXTURE_ROOT=str(fixtures))
    argv = [sys.executable, '-B', '-m', 'unittest', '-v', 'test_driver']
    with (destination / 'command.log').open('xb') as output:
        result = subprocess.run(argv, cwd=ROOT, env=env, stdout=output, stderr=subprocess.STDOUT, check=False)
        output.flush(); os.fsync(output.fileno())
    after = {role: desc(path) for role, path in FILES.items()}
    save(destination / 'tools-after.json', after)
    receipt = {'kind': 'rne_141_synthetic_driver_test_receipt', 'schema_version': 1,
               'status': 'PASS' if result.returncode == 0 and before == after else 'FAIL_RETAINED',
               'started_at': started, 'finished_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
               'argv': argv, 'original_exit_code': result.returncode, 'source_inputs_unchanged': before == after,
               'tested_tool_sha256': {role: value['sha256'] for role, value in before.items()},
               'log': desc(destination / 'command.log'), 'synthetic_stub_fixture_root': str(fixtures),
               'actual_report_reads': 0, 'worlds': 0, 'builds': 0, 'checkout_edits': 0,
               'scope': 'Only synthetic reports/stub collector processes. Internal mock72 counters are not actual scientific world counts.'}
    save(destination / 'terminal.json', receipt)
    print(json.dumps(receipt, sort_keys=True))
    return result.returncode if result.returncode else (0 if before == after else 1)


if __name__ == '__main__': raise SystemExit(main())
