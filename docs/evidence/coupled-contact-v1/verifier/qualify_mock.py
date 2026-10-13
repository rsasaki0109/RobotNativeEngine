"""Retain original synthetic archive test exit and exact source guards."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
from whole_archive import descriptor, write_json

BASE = Path(__file__).resolve().parent


def main():
    output = BASE / 'pure-checks' / sys.argv[1]; output.mkdir(parents=True, exist_ok=False)
    source = output / 'source'; source.mkdir()
    before = [descriptor(p) for p in sorted(BASE.glob('*.py'))]
    for item in before: (source / Path(item['path']).name).write_bytes(Path(item['path']).read_bytes())
    command = [str(Path(sys.executable).resolve()), '-B', '-m', 'unittest', '-v', 'test_whole_archive_mock']
    start = datetime.now(timezone.utc).isoformat()
    with (output / 'stdout.log').open('xb') as stdout, (output / 'stderr.log').open('xb') as stderr:
        process = subprocess.Popen(command, cwd=BASE, stdout=stdout, stderr=stderr)
        code = process.wait(); stdout.flush(); stderr.flush(); os.fsync(stdout.fileno()); os.fsync(stderr.fileno())
    after = [descriptor(p) for p in sorted(BASE.glob('*.py'))]
    receipt = dict(kind='actual_original_archive_pure_tests', execution_kind='PURE_MOCK_ONLY', command=command,
                   pid=process.pid, original_exit=code, started_utc=start, ended_utc=datetime.now(timezone.utc).isoformat(),
                   actual_interpreter=descriptor(Path(sys.executable).resolve()), python_version=sys.version,
                   source_before=before, source_after=after, source_unchanged=before == after,
                   stdout=descriptor(output / 'stdout.log'), stderr=descriptor(output / 'stderr.log'),
                   actual_study_inventory_or_archive_verifications=0, native_worlds=0, scientific_readers=0)
    write_json(output / 'actual-command.json', receipt)
    print(json.dumps(dict(original_exit=code, source_unchanged=before == after, receipt=descriptor(output / 'actual-command.json'))))
    return 0 if code == 0 and before == after else 1


if __name__ == '__main__': raise SystemExit(main())
