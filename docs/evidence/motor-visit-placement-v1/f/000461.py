"""Portable-retention fault tests using invented bytes; never import archived code."""
import copy
import gzip
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import verify_literals as verifier

ARTIFACTS = Path(__file__).resolve().parent / 'pure-test-artifacts'


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def original(path, raw):
    return dict(path=path, sha256=digest(raw), bytes=len(raw))


def representation(path, raw):
    return dict(relative_path=path, sha256=digest(raw), bytes=len(raw))


class PortableEvidenceTests(unittest.TestCase):
    def setUp(self):
        ARTIFACTS.mkdir(exist_ok=True)
        self.root = Path(tempfile.mkdtemp(prefix=self._testMethodName + '-', dir=ARTIFACTS))
        (self.root / 'f').mkdir()
        (self.root / '.gitattributes').write_bytes(b'* -text\n')
        (self.root / 'README.md').write_bytes(b'Synthetic byte-only fixture.\n')
        receipt = b'Invented restoration receipt; not scientific evidence.\n'
        literal = b'Literal bytes preserve CRLF\r\nand final whitespace. \n'
        self.audit = b'Whole invented opaque audit, with all digits: 123456789012345678901234567890.\n' * 7
        gz = gzip.compress(self.audit, mtime=0)
        rows = [('f/receipt.bin', receipt, '/historical/receipt.json', receipt, 'whole-file-exact-bytes'),
                ('f/literal.bin', literal, '/historical/raw.jsonl.gz', literal, 'whole-file-exact-bytes'),
                ('f/audit.json.gz', gz, '/historical/audit.json', self.audit, 'lossless-single-member-gzip-original-exact-bytes')]
        payloads, aliases = [], []
        for name, stored, origin, restored, encoding in rows:
            (self.root / name).write_bytes(stored)
            p = representation(name, stored)
            payloads.append(p)
            aliases.append(dict(original=original(origin, restored), representation=copy.deepcopy(p), encoding=encoding))
        self.manifest = dict(kind=verifier.KIND, schema_version=1, aliases=aliases,
            payloads=payloads,
            local_only=[dict(original=dict(path='/nonexistent/historical/python3', sha256='0' * 64, bytes=6812336), reason='Executable metadata binding only; no recoverable payload.')],
            original_audit_restoration_receipt=copy.deepcopy(aliases[0]['original']),
            scope='Invented retained bytes for tamper tests only.')
        self.save()

    def save(self):
        (self.root / 'manifest.json').write_text(json.dumps(self.manifest, indent=2) + '\n')

    def fails(self):
        self.save()
        with self.assertRaises(verifier.ValidationError):
            verifier.verify(self.root)

    def replace_gzip(self, raw):
        (self.root / 'f/audit.json.gz').write_bytes(raw)
        p = representation('f/audit.json.gz', raw)
        self.manifest['payloads'][2] = p
        self.manifest['aliases'][2]['representation'] = copy.deepcopy(p)

    def test_success_distinguishes_missing_local_only_from_recoverable_bytes(self):
        result = verifier.verify(self.root)
        self.assertEqual(result['aliases'], 3)
        self.assertEqual(result['unique_payloads'], 3)
        self.assertEqual(result['gzip_original_representations'], 1)
        self.assertEqual(result['local_only_bindings'], 1)
        self.assertIn('no absent original bytes read, fetched or recovered', result['local_only_scope'])
        self.assertFalse(Path(self.manifest['local_only'][0]['original']['path']).exists())
        self.assertEqual(result['scientific_reader_invocations'], 0)

    def test_archive_python_bytes_are_never_imported_or_executed(self):
        raw = b'raise RuntimeError("THIS ARCHIVED MODULE MUST NEVER EXECUTE")\n'
        (self.root / 'f/archived.py').write_bytes(raw)
        p = representation('f/archived.py', raw)
        self.manifest['payloads'].append(p)
        self.manifest['aliases'].append(dict(original=original('/historical/module.py', raw), representation=copy.deepcopy(p), encoding='whole-file-exact-bytes'))
        self.save()
        self.assertEqual(verifier.verify(self.root)['unique_payloads'], 4)

    def test_corrupt_stored_bytes_rejected(self):
        (self.root / 'f/literal.bin').write_bytes(b'altered original byte content')
        self.fails()

    def test_wrong_stored_size_rejected(self):
        self.manifest['payloads'][1]['bytes'] += 1
        self.manifest['aliases'][1]['representation']['bytes'] += 1
        self.fails()

    def test_wrong_stored_sha_rejected(self):
        self.manifest['payloads'][1]['sha256'] = '1' * 64
        self.manifest['aliases'][1]['representation']['sha256'] = '1' * 64
        self.fails()

    def test_whole_alias_original_hash_and_size_must_equal_payload(self):
        for key, value in [('bytes', 1), ('sha256', '1' * 64)]:
            with self.subTest(key=key):
                old = self.manifest['aliases'][1]['original'][key]
                self.manifest['aliases'][1]['original'][key] = value
                self.fails()
                self.manifest['aliases'][1]['original'][key] = old

    def test_bool_and_float_numeric_fields_are_rejected(self):
        targets = [(self.manifest, 'schema_version'),
                   (self.manifest['payloads'][0], 'bytes'),
                   (self.manifest['aliases'][1]['original'], 'bytes'),
                   (self.manifest['local_only'][0]['original'], 'bytes')]
        for target, key in targets:
            for value in (True, False, 1.0, -1, 1 << 64):
                with self.subTest(key=key, value=value):
                    old = target[key]
                    target[key] = value
                    self.fails()
                    target[key] = old

    def test_sha_requires_lowercase_exact_hex(self):
        for value in ('a' * 63, 'g' * 64, 'A' * 64, True, 123):
            with self.subTest(value=value):
                self.manifest['payloads'][0]['sha256'] = value
                self.fails()
        self.manifest['payloads'][0]['sha256'] = digest((self.root / 'f/receipt.bin').read_bytes())

    def test_duplicate_json_keys_rejected(self):
        raw = json.dumps(self.manifest)
        (self.root / 'manifest.json').write_text(raw.replace('"schema_version": 1', '"schema_version": 1, "schema_version": 1', 1))
        with self.assertRaises(verifier.ValidationError):
            verifier.verify(self.root)

    def test_manifest_nan_and_json_float_rejected(self):
        for literal in ('NaN', 'Infinity', '1.0'):
            with self.subTest(literal=literal):
                raw = json.dumps(self.manifest).replace('"schema_version": 1', '"schema_version": ' + literal, 1)
                (self.root / 'manifest.json').write_text(raw)
                with self.assertRaises(verifier.ValidationError):
                    verifier.verify(self.root)

    def test_unsafe_and_noncanonical_payload_paths_rejected(self):
        for value in ('../outside', '/absolute', 'f/../outside', 'f//x', 'f/./x', 'f/x/', 'f\\x', 'f/C:x', 'f/\x00x', 'f/é.bin'):
            with self.subTest(value=value):
                self.manifest['payloads'][0]['relative_path'] = value
                self.fails()

    def test_windows_reserved_and_trailing_dot_space_rejected(self):
        for value in ('f/CON.txt', 'f/nul', 'f/Lpt9.bin', 'f/CONIN$', 'f/file.', 'f/file ', 'f/bad?.bin'):
            with self.subTest(value=value):
                self.manifest['payloads'][0]['relative_path'] = value
                self.fails()

    def test_full_repository_relative_windows_length_boundary(self):
        self.manifest['payloads'][0]['relative_path'] = 'f/' + 'x' * (167 - len(verifier.PREFIX) - 2)
        self.fails()
        accepted = 'f/' + 'x' * (166 - len(verifier.PREFIX) - 2)
        self.assertEqual(len(verifier.PREFIX + accepted), 166)
        self.assertEqual(verifier.relative_path(accepted).as_posix(), accepted)

    def test_casefold_file_collision_rejected(self):
        p = copy.deepcopy(self.manifest['payloads'][1])
        p['relative_path'] = 'f/LITERAL.bin'
        self.manifest['payloads'].append(p)
        self.fails()

    def test_casefold_directory_collision_rejected(self):
        for name in ('f/Dir/first.bin', 'f/dir/second.bin'):
            raw = b'new byte'
            self.manifest['payloads'].append(representation(name, raw))
        self.fails()

    def test_payload_symlink_rejected(self):
        target = self.root / 'f/literal.bin'
        target.unlink()
        target.symlink_to(self.root / 'f/receipt.bin')
        self.fails()

    def test_parent_directory_symlink_rejected(self):
        (self.root / 'linked').symlink_to(self.root / 'f', target_is_directory=True)
        (self.root / 'f/link').symlink_to(self.root / 'f', target_is_directory=True)
        self.manifest['payloads'][0]['relative_path'] = 'f/link/receipt.bin'
        self.manifest['aliases'][0]['representation']['relative_path'] = 'f/link/receipt.bin'
        self.fails()

    def test_sidecar_and_bundle_root_symlink_rejected(self):
        attrs = self.root / '.gitattributes'
        saved = self.root / 'attrs.txt'
        attrs.rename(saved)
        attrs.symlink_to(saved)
        self.fails()
        alias = self.root.parent / (self.root.name + '-symlink')
        alias.symlink_to(self.root, target_is_directory=True)
        with self.assertRaises(verifier.ValidationError):
            verifier.verify(alias)

    def test_missing_and_undeclared_payloads_rejected(self):
        target = self.root / 'f/literal.bin'
        old = target.read_bytes()
        target.unlink()
        self.fails()
        target.write_bytes(old)
        (self.root / 'f/unlisted.bin').write_bytes(b'unlisted')
        self.fails()

    def test_alias_cannot_point_at_missing_or_conflicting_payload(self):
        self.manifest['aliases'][1]['representation']['relative_path'] = 'f/missing.bin'
        self.fails()
        self.manifest['aliases'][1]['representation']['relative_path'] = 'f/literal.bin'
        self.manifest['aliases'][1]['representation']['bytes'] += 1
        self.fails()

    def test_duplicate_origins_and_local_only_overlap_rejected(self):
        self.manifest['aliases'].append(copy.deepcopy(self.manifest['aliases'][1]))
        self.fails()
        self.manifest['aliases'].pop()
        self.manifest['local_only'][0]['original'] = copy.deepcopy(self.manifest['aliases'][1]['original'])
        self.fails()

    def test_duplicate_payload_and_unreferenced_payload_rejected(self):
        self.manifest['payloads'].append(copy.deepcopy(self.manifest['payloads'][1]))
        self.fails()
        self.manifest['payloads'].pop()
        raw = b'unreferenced'
        (self.root / 'f/unreferenced.bin').write_bytes(raw)
        self.manifest['payloads'].append(representation('f/unreferenced.bin', raw))
        self.fails()

    def test_single_member_rejects_extra_member_and_trailing_data(self):
        for extra in (gzip.compress(b'new member', mtime=0), b'trailing'):
            with self.subTest(extra=extra):
                self.replace_gzip(gzip.compress(self.audit, mtime=0) + extra)
                self.fails()

    def test_gzip_truncation_and_crc_corruption_rejected_after_hash_rebinding(self):
        good = gzip.compress(self.audit, mtime=0)
        bad_crc = bytearray(good)
        bad_crc[-8] ^= 0xff
        for bad in (good[:-4], bytes(bad_crc), good[:-9]):
            with self.subTest(length=len(bad)):
                self.replace_gzip(bad)
                self.fails()

    def test_gzip_excess_restored_bytes_are_bounded_and_rejected(self):
        self.manifest['aliases'][2]['original']['bytes'] = len(self.audit) - 1
        self.fails()

    def test_gzip_declared_restored_size_and_sha_must_match(self):
        self.manifest['aliases'][2]['original']['bytes'] += 1
        self.fails()
        self.manifest['aliases'][2]['original']['bytes'] -= 1
        self.manifest['aliases'][2]['original']['sha256'] = 'f' * 64
        self.fails()

    def test_whole_original_larger_than_inline_limit_is_valid_as_small_gzip(self):
        self.audit = b'All complete opaque digits and rows retained.\r\n' * 50000
        self.assertGreater(len(self.audit), verifier.CAP)
        self.replace_gzip(gzip.compress(self.audit, mtime=0))
        self.manifest['aliases'][2]['original'] = original('/historical/audit.json', self.audit)
        self.save()
        self.assertEqual(verifier.verify(self.root)['gzip_original_representations'], 1)

    def test_stored_file_and_manifest_inline_limits_rejected(self):
        self.manifest['payloads'][1]['bytes'] = verifier.CAP + 1
        self.fails()
        self.manifest['payloads'][1]['bytes'] = len((self.root / 'f/literal.bin').read_bytes())
        (self.root / 'manifest.json').write_bytes(b' ' * (verifier.CAP + 1))
        with self.assertRaises(verifier.ValidationError):
            verifier.verify(self.root)

    def test_exact_attributes_and_unknown_schema_keys_rejected(self):
        (self.root / '.gitattributes').write_bytes(b'* text\n')
        self.fails()
        (self.root / '.gitattributes').write_bytes(b'* -text\n')
        self.manifest['new_accuracy_flag'] = True
        self.fails()

    def test_restoration_receipt_must_be_retained_whole_and_not_local_only(self):
        self.manifest['original_audit_restoration_receipt']['path'] = '/nonexistent/historical/python3'
        self.manifest['original_audit_restoration_receipt']['sha256'] = '0' * 64
        self.manifest['original_audit_restoration_receipt']['bytes'] = 6812336
        self.fails()

    def test_local_only_schema_requires_reason_without_claiming_recovery(self):
        self.manifest['local_only'][0]['reason'] = ''
        self.fails()
        self.manifest['local_only'][0]['reason'] = 'Metadata only.'
        self.manifest['local_only'][0]['recovered'] = True
        self.fails()


if __name__ == '__main__':
    unittest.main(verbosity=2)
