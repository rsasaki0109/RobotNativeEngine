"""Pure archival fault checks; never starts a robot or parses native payloads."""
import copy
from datetime import datetime, timezone
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import unittest
from unittest.mock import patch
import checksum_reconstruct as recovery

ROOT = Path(__file__).resolve().parent
RUN = ROOT / 'synthetic-tests-hardened' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
RUN.mkdir(parents=True, exist_ok=False)


def save(path, data):
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n')


def make_fixture(name):
    root = RUN / name
    root.mkdir()
    members, rows = [], []
    for build in ('baseline', 'rejected_inverse_w_plus_g'):
        for mass in (5, 20, 50):
            for mode in ('on', 'fresh_on', 'off'):
                label = f'{build}-m{mass}-{mode}'
                raw = root / (label + '.ndjson.gz')
                raw.write_bytes(('SYNTHETIC-NON-NATIVE:' + label).encode())
                descriptor = recovery.record(raw)
                cell = dict(build=build, mass_kg=mass, mode=mode, run_label=label)
                members.append(dict(cell, basename=raw.name, size_bytes=descriptor['bytes'], sha256=descriptor['sha256']))
                rows.append(dict(cell, attempted=True, exit_code=0, trace=descriptor))
    manifest = root / 'raw-trace-manifest.json'
    save(manifest, {'members': members})
    # Independently produce the expected archive using seekable tarfile writing.
    expected = io.BytesIO()
    paths = {row['basename']: root / row['basename'] for row in members}
    paths['raw-trace-manifest.json'] = manifest
    with tarfile.open(fileobj=expected, mode='w', format=tarfile.PAX_FORMAT) as archive:
        for basename in sorted(paths):
            info = tarfile.TarInfo(basename)
            info.size = paths[basename].stat().st_size
            info.mtime = info.uid = info.gid = 0
            info.mode = 0o444
            archive.addfile(info, io.BytesIO(paths[basename].read_bytes()))
    content = expected.getvalue()
    registration = root / 'registration.json'
    save(registration, {'new_cohort_id': 'pure-fixture', 'original_member_manifest': recovery.record(manifest),
                        'expected_original_archive': {'basename': 'original.tar', 'bytes': len(content), 'sha256': hashlib.sha256(content).hexdigest()}})
    collection = root / 'collection.json'
    save(collection, {'schema_version': 2, 'generation': 5, 'cohort_id': 'pure-fixture', 'runs': rows})
    return root, collection, registration, content


class ArchiveFaultChecks(unittest.TestCase):
    def test_all_eighteen_exact_and_directory_fsync(self):
        root, collection, registration, expected = make_fixture('exact')
        original = recovery.sync_directory
        with patch.object(recovery, 'sync_directory', wraps=original) as synced:
            code = recovery.reconstruct(collection, registration, recovery.record(registration)['sha256'], root / 'output')
            self.assertEqual(code, 0)
            self.assertGreaterEqual(synced.call_count, 4)
        self.assertEqual((root / 'output/original.tar').read_bytes(), expected)
        with tarfile.open(root / 'output/original.tar') as archive:
            entries = archive.getmembers()
            self.assertEqual(len(entries), 19)
            self.assertEqual([entry.name for entry in entries], sorted(entry.name for entry in entries))
            for entry in entries:
                self.assertEqual((entry.mode, entry.mtime, entry.uid, entry.gid, entry.uname, entry.gname), (0o444, 0, 0, 0, '', ''))
        with self.assertRaises(FileExistsError):
            recovery.reconstruct(collection, registration, recovery.record(registration)['sha256'], root / 'output')

    def test_one_changed_member_all_or_none(self):
        root, collection, registration, _ = make_fixture('mismatch')
        data = json.loads(collection.read_text())
        raw = Path(data['runs'][0]['trace']['path'])
        raw.write_bytes(b'SYNTHETIC-CHANGED')
        data['runs'][0]['trace'] = recovery.record(raw)
        save(collection, data)
        self.assertEqual(recovery.reconstruct(collection, registration, recovery.record(registration)['sha256'], root / 'output'), 1)
        self.assertFalse((root / 'output/original.tar').exists())
        result = json.loads((root / 'output/reconstruction-terminal.json').read_text())
        self.assertEqual(len(result['matches']), 18)
        self.assertEqual(result['status'], 'RAW_CHECKSUMS_DO_NOT_ALL_MATCH_ORIGINAL')

    def test_duplicate_and_failed_case_reject(self):
        for kind in ('duplicate', 'failed'):
            root, collection, registration, _ = make_fixture(kind)
            data = json.loads(collection.read_text())
            if kind == 'duplicate':
                data['runs'][1] = copy.deepcopy(data['runs'][0])
            else:
                data['runs'][0]['exit_code'] = 101
            save(collection, data)
            with self.assertRaises(ValueError):
                recovery.reconstruct(collection, registration, recovery.record(registration)['sha256'], root / 'output')
            self.assertFalse((root / 'output/original.tar').exists())

    def test_archive_sha_fault_retains_failed_output(self):
        root, collection, registration, _ = make_fixture('wrong-archive-sha')
        data = json.loads(registration.read_text())
        data['expected_original_archive']['sha256'] = '0' * 64
        save(registration, data)
        with self.assertRaisesRegex(ValueError, 'archive SHA'):
            recovery.reconstruct(collection, registration, recovery.record(registration)['sha256'], root / 'output')
        self.assertTrue((root / 'output/original.tar').exists())
        result = json.loads((root / 'output/reconstruction-terminal.json').read_text())
        self.assertEqual(result['status'], 'RECONSTRUCTION_FAILED_WITHOUT_RETRY_OR_SUBSTITUTION')

    def test_second_stream_byte_mismatch_reject(self):
        path = RUN / 'second-stream.bin'
        path.write_bytes(b'SYNTHETIC-A')
        compare = recovery.CompareStream(path)
        try:
            with self.assertRaisesRegex(ValueError, 'stream differs'):
                compare.write(b'SYNTHETIC-B')
        finally:
            compare.stream.close()

    def test_optimized_python_does_not_disable_guards(self):
        root, collection, registration, _ = make_fixture('optimized')
        command = [sys.executable, '-O', '-B', str(ROOT / 'checksum_reconstruct.py'), '--collection', str(collection), '--registration', str(registration), '--registration-sha256', '0' * 64, '--output', str(root / 'output')]
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        (root / 'stdout.log').write_bytes(result.stdout)
        (root / 'stderr.log').write_bytes(result.stderr)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(b'registration SHA changed', result.stderr)
        self.assertFalse((root / 'output').exists())


if __name__ == '__main__':
    with (RUN / 'actual-tests.log').open('w') as stream:
        result = unittest.TextTestRunner(stream=stream, verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ArchiveFaultChecks))
    receipt = {'status': 'PASS' if result.wasSuccessful() else 'FAIL', 'actual_exit_code': 0 if result.wasSuccessful() else 1,
               'tests_run': result.testsRun, 'worlds': 0, 'builds': 0, 'native_numeric_payload_reads': 0,
               'tested_tool': recovery.record(ROOT / 'checksum_reconstruct.py'), 'test_script': recovery.record(__file__),
               'test_log': recovery.record(RUN / 'actual-tests.log'), 'synthetic_directory': str(RUN), 'ended_utc': datetime.now(timezone.utc).isoformat()}
    save(RUN / 'actual-pure-checks.json', receipt)
    print(json.dumps({'receipt': recovery.record(RUN / 'actual-pure-checks.json'), 'status': receipt['status'], 'tests_run': result.testsRun}))
    raise SystemExit(receipt['actual_exit_code'])
