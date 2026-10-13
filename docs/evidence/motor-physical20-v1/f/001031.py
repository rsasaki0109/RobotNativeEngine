"""Bind the admitted protocol and sealed preparation; no world or audit launch."""
from pathlib import Path
import hashlib
import json
import os
import subprocess
import time

OWN = Path(__file__).resolve().parent
STUDY = OWN.parent / 'next-motor-physical-120'

def descriptor(path):
    data = path.read_bytes()
    return {'path': str(path), 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}

def main():
    STUDY.mkdir()
    extras = [OWN / 'README.md', OWN / 'post-build-root-review.json', Path(__file__).resolve(),
              OWN / 'operative-protocol-preparation-attempt-1-failure.json',
              OWN / 'independent-source-review/actual-source-only-review/independent-source-only-review.json',
              OWN / 'independent-physics-review/independent-physics-review.json']
    extras.extend(p for p in sorted((OWN / 'reader').iterdir()) if p.is_file() and p.suffix in ('.json', '.md'))
    for role in ('reader', 'driver'):
        handoff = json.loads((OWN / role / 'handoff.json').read_text())
        if role == 'reader':
            extras.extend(Path(handoff[key]['path']) for key in ('final_pure_receipt', 'final_pure_stdout', 'final_pure_stderr'))
            for attempt in handoff['all_pure_attempts']:
                extras.extend(p for p in sorted(Path(attempt['path']).parent.iterdir()) if p.is_file())
        else:
            extras.extend(Path(d['path']) for d in handoff['pure_outputs'])
            for folder in sorted((OWN / 'driver/validation').iterdir()):
                if folder.is_dir():
                    extras.extend(p for p in sorted(folder.iterdir()) if p.is_file())
    for attempt in (1, 2):
        extras.extend(p for p in sorted((OWN / f'build-qualification-attempt-{attempt}').iterdir()) if p.is_file())
    extras.extend(p for p in sorted((OWN / 'unadmitted-predata-build-helper-history').iterdir()) if p.is_file())
    extras.extend(p for p in sorted((OWN / 'unadmitted-predata-clippy-attempt-2-sources').iterdir()) if p.is_file())
    argv = ['/usr/bin/python3', '-B', str(OWN / 'driver/assemble_candidate.py'),
        '--binding', str(OWN / 'actual-build-binding.json'),
        '--protocol', str(OWN / 'operative-protocol-v1.json'),
        '--collector-schema', str(OWN / 'collector/schema-source-contract.json'),
        '--reader-source', str(OWN / 'reader/physical_reader.py'),
        '--producer-review', str(OWN / 'post-build-root-review.json'),
        '--pure-tests-review', str(OWN / 'pure-tests-review.json'),
        '--cwd', str(OWN / 'collector'), '--output-root', str(STUDY / 'cohort'),
        '--master-run-root', str(STUDY / 'master'),
        '--driver-run-receipt', str(STUDY / 'checked-driver-original/actual-command.json'),
        '--reader-output', str(STUDY / 'actual-cohort-audit.json'),
        '--output', str(STUDY / 'frozen-candidate.json')]
    for path in sorted(set(extras)):
        argv.extend(['--extra-path', str(path)])
    started = time.time_ns()
    with (STUDY / 'freeze-stdout.log').open('xb') as out, (STUDY / 'freeze-stderr.log').open('xb') as err:
        process = subprocess.run(argv, cwd=OWN, env=os.environ.copy(), stdout=out, stderr=err, check=False)
    receipt = {'kind': 'NEW_PHYSICAL_PRE_DATA_FREEZE_ORIGINAL', 'argv': argv, 'cwd': str(OWN),
        'original_exit': process.returncode, 'start_unix_ns': started, 'end_unix_ns': time.time_ns(),
        'native_worlds': 0, 'scientific_reader_invocations': 0,
        'stdout': descriptor(STUDY / 'freeze-stdout.log'), 'stderr': descriptor(STUDY / 'freeze-stderr.log')}
    with (STUDY / 'freeze-original.json').open('x') as stream:
        json.dump(receipt, stream, indent=2)
        stream.write('\n')
    if process.returncode:
        raise SystemExit(process.returncode)
    print(json.dumps({'frozen_candidate': descriptor(STUDY / 'frozen-candidate.json'),
                      'freeze_original': descriptor(STUDY / 'freeze-original.json'), 'native_worlds': 0}))

if __name__ == '__main__':
    main()
