"""Synthetic whole-archive corruption/retention tests, never a scientific corpus."""
import gzip
import copy
import io
import json
from pathlib import Path
from types import SimpleNamespace
import tarfile
import tempfile
import unittest
import zlib
import whole_archive as archive

BASE = Path(__file__).resolve().parent
FIXTURES = BASE / 'pure-MOCK-fixtures'; FIXTURES.mkdir(exist_ok=True)


class ArchiveTests(unittest.TestCase):
    def setUp(self):
        self.work = Path(tempfile.mkdtemp(prefix=self._testMethodName + '-', dir=FIXTURES))
        self.inputs = self.work / 'MOCK_INPUT'; self.inputs.mkdir()
        self.file = self.inputs / 'opaque-original.bin'
        self.file.write_bytes(b'MOCK_OPAQUE_IEEE_WORD_BYTES_' + bytes(range(256)) * 400)
        self.raw = self.inputs / 'MOCK.raw.jsonl.gz'
        self.raw.write_bytes(gzip.compress(b'MOCK_ONLY_NO_NUMERIC_TRAJECTORY\n', mtime=0))
        self.roots = {'PREP': str(self.inputs)}
        self.output = self.work / 'MOCK.tar.gz'; self.receipt = self.work / 'create.json'; self.verified = self.work / 'verified.json'

    def make(self, **kwargs):
        return archive.create(self.roots, [], self.output, self.receipt,
                              disk_free=lambda _: SimpleNamespace(free=1 << 40), **kwargs)

    def check(self, **kwargs): return archive.verify(self.output, self.receipt, self.verified, **kwargs)

    def alias(self, path):
        with tarfile.open(self.output, 'r:gz') as standard:
            inventory = json.load(standard.extractfile('inventory.json'))
        return next(member['member_path'] for member in inventory['members'] if member['original']['path'] == str(path))

    def rewrite_tar(self, data):
        with self.output.open('wb') as file:
            writer = archive.StoredGzip(file)
            for offset in range(0, len(data), 65536): writer.write(data[offset:offset + 65536])
            writer.finish()
        receipt = archive.read_json(self.receipt); receipt['archive'] = archive.descriptor(self.output)
        self.receipt.write_bytes(archive.canonical(receipt) + b'\n')

    def test_whole_bytes_standard_gzip_tar_and_bound(self):
        made = self.make(); result = self.check()
        self.assertEqual(result['status'], 'PASS_WHOLE_LITERAL_ARCHIVE_BYTES_ONLY')
        self.assertEqual(self.output.stat().st_size, made['budget']['archive_upper_bound_bytes'])
        with tarfile.open(self.output, 'r:gz') as tar:
            self.assertEqual(tar.extractfile(self.alias(self.file)).read(), self.file.read_bytes())
            self.assertEqual(tar.extractfile(self.alias(self.raw)).read(), self.raw.read_bytes())

    def test_long_literal_pax_member_and_empty_original(self):
        folder = self.inputs / ('long-' + 'a' * 90); folder.mkdir()
        long = folder / ('original-' + 'b' * 90 + '.txt'); long.write_bytes(b'PAX whole original')
        (self.inputs / 'empty').write_bytes(b'')
        self.make(); self.check()
        with tarfile.open(self.output, 'r:gz') as tar:
            self.assertEqual(tar.extractfile(self.alias(long)).read(), b'PAX whole original')

    def test_capacity_rejection_before_archive_creation(self):
        with self.assertRaisesRegex(ValueError, 'does not fit'):
            archive.create(self.roots, [], self.output, self.receipt, disk_free=lambda _: SimpleNamespace(free=0))
        self.assertFalse(self.output.exists()); self.assertFalse(self.receipt.exists())

    def test_archive_inside_input_root_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'inside'):
            archive.create(self.roots, [], self.inputs / 'self.tar.gz', self.receipt)

    def test_symlink_input_stored_as_literal_object(self):
        (self.inputs / 'link').symlink_to(self.file)
        self.make(); result = self.check(); self.assertEqual(result['symlink_objects'], 1)
        with tarfile.open(self.output, 'r:gz') as tar:
            link = tar.getmember(self.alias(self.inputs / 'link'))
            self.assertTrue(link.issym()); self.assertEqual(link.linkname, str(self.file)); self.assertEqual(link.size, 0)

    def test_symlink_directory_not_dereferenced_or_recurred(self):
        external = self.work / 'NOT_AN_INPUT_ROOT'; external.mkdir()
        (external / 'NOT_TO_BE_ARCHIVED').write_bytes(b'MOCK external target never read')
        (self.inputs / 'linked-directory').symlink_to(external, target_is_directory=True)
        self.make(); self.check()
        with tarfile.open(self.output, 'r:gz') as tar:
            self.assertTrue(tar.getmember(self.alias(self.inputs / 'linked-directory')).issym())
            inventory = json.load(tar.extractfile('inventory.json'))
            self.assertFalse(any('NOT_TO_BE_ARCHIVED' in member['original']['path'] for member in inventory['members']))

    def test_overlapping_named_roots_rejected(self):
        nested = self.inputs / 'nested'; nested.mkdir()
        with self.assertRaisesRegex(ValueError, 'overlapping'):
            archive.inventory_for({'A': str(self.inputs), 'B': str(nested)}, [])

    def test_unlisted_executable_not_implicitly_packaged(self):
        path = self.inputs / 'ELF_NATIVE_MOCK'
        path.write_bytes(b'\x7fELFopaque executable not executed')
        path.chmod(0o755)
        try:
            with self.assertRaisesRegex(ValueError, 'ledger'): self.make()
        finally:
            # Restore the synthetic byte fixture's nonexecutable storage mode.
            # Every byte remains retained; no program was executed.
            path.chmod(0o644)

    def test_local_only_binding_not_recoverable_member(self):
        binary = self.inputs / 'ELF_NATIVE_MOCK'; binary.write_bytes(b'\x7fELFnot executed')
        local = [archive.descriptor(binary)]
        archive.create(self.roots, local, self.output, self.receipt, disk_free=lambda _: SimpleNamespace(free=1 << 40))
        result = self.check(); self.assertEqual(result['local_only_bindings'], 1)
        with tarfile.open(self.output, 'r:gz') as tar:
            inventory = json.load(tar.extractfile('inventory.json'))
            self.assertNotIn(str(binary), [member['original']['path'] for member in inventory['members']])
            self.assertEqual(inventory['local_only'], local)

    def test_changed_original_during_create_preserves_partial(self):
        def change(): self.file.write_bytes(b'MOCK changed after frozen inventory')
        with self.assertRaisesRegex(ValueError, 'changed'): self.make(after_inventory=change)
        self.assertTrue(self.output.exists()); self.assertFalse(self.receipt.exists())
        self.assertEqual(self.file.read_bytes(), b'MOCK changed after frozen inventory')

    def test_changed_original_after_archive_rejects_local_verification(self):
        self.make(); self.file.write_bytes(b'MOCK changed after archive')
        with self.assertRaisesRegex(ValueError, 'changed'): self.check()

    def test_portable_mode_never_reads_absent_originals(self):
        self.make(); self.file.unlink(); self.raw.unlink(); self.inputs.rmdir()
        result = self.check(recheck_originals=False)
        self.assertFalse(result['originals_rechecked']); self.assertFalse(result['public_availability_verified'])

    def test_member_tamper_even_with_updated_whole_archive_sha_rejected(self):
        self.make(); data = bytearray(gzip.decompress(self.output.read_bytes()))
        index = data.find(b'MOCK_OPAQUE_IEEE_WORD_BYTES_'); self.assertGreater(index, 0)
        data[index] ^= 1; self.rewrite_tar(bytes(data))
        with self.assertRaisesRegex(ValueError, 'member SHA'): self.check()

    def test_header_tamper_even_with_updated_whole_archive_sha_rejected(self):
        self.make(); data = bytearray(gzip.decompress(self.output.read_bytes()))
        with tarfile.open(fileobj=io.BytesIO(data), mode='r:') as standard:
            index = standard.getmember(self.alias(self.raw)).offset
        self.assertGreater(index, 0)
        data[index] ^= 1; self.rewrite_tar(bytes(data))
        with self.assertRaisesRegex(ValueError, 'header mismatch'): self.check()

    def test_gzip_crc_corruption_rejected(self):
        self.make(); data = bytearray(self.output.read_bytes()); data[-8] ^= 1; self.output.write_bytes(data)
        receipt = archive.read_json(self.receipt); receipt['archive'] = archive.descriptor(self.output)
        self.receipt.write_bytes(archive.canonical(receipt) + b'\n')
        with self.assertRaises(zlib.error): self.check()

    def test_trailing_or_second_member_rejected(self):
        self.make()
        with self.output.open('ab') as file: file.write(gzip.compress(b'MOCK extra member'))
        receipt = archive.read_json(self.receipt); receipt['archive'] = archive.descriptor(self.output)
        self.receipt.write_bytes(archive.canonical(receipt) + b'\n')
        with self.assertRaisesRegex(ValueError, 'trailing gzip'): self.check()

    def test_truncated_gzip_rejected(self):
        self.make(); self.output.write_bytes(self.output.read_bytes()[:-8])
        receipt = archive.read_json(self.receipt); receipt['archive'] = archive.descriptor(self.output)
        self.receipt.write_bytes(archive.canonical(receipt) + b'\n')
        with self.assertRaisesRegex(ValueError, 'truncated'): self.check()

    def test_extra_tar_payload_rejected(self):
        self.make(); data = bytearray(gzip.decompress(self.output.read_bytes())); data[-1] = 1
        self.rewrite_tar(bytes(data))
        with self.assertRaisesRegex(ValueError, 'trailer'): self.check()

    def test_duplicate_json_bool_size_and_unsafe_paths_rejected(self):
        with self.assertRaises(ValueError): archive.loads(b'{"a":1,"a":2}')
        item = archive.descriptor(self.file); item['bytes'] = True
        with self.assertRaisesRegex(ValueError, 'size type'): archive.descriptor_shape(item)
        for name in ['../out', '/abs', 'A//bad', 'A/./bad', 'A/back\\slash', 'A/x:y']:
            with self.assertRaises(ValueError): archive.safe_member(name)

    def test_original_case_pair_both_whole_bytes_retained(self):
        (self.inputs / 'Case').write_bytes(b'A'); (self.inputs / 'case').write_bytes(b'B')
        self.make(); self.check()
        upper, lower = self.alias(self.inputs / 'Case'), self.alias(self.inputs / 'case')
        self.assertNotEqual(upper.casefold(), lower.casefold())
        with tarfile.open(self.output, 'r:gz') as tar:
            self.assertEqual(tar.extractfile(upper).read(), b'A')
            self.assertEqual(tar.extractfile(lower).read(), b'B')

    def test_missing_or_added_original_changes_inventory(self):
        self.make(); (self.inputs / 'MOCK_EXTRA_UNREGISTERED').write_bytes(b'no filtering')
        with self.assertRaisesRegex(ValueError, 'changed'): self.check()

    def test_dangling_and_long_pax_link_targets_retained(self):
        target = 'NOT_PRESENT/' + 'literal-long-target-' * 20
        link = self.inputs / 'dangling-long-link'; link.symlink_to(target)
        self.make(); self.check()
        with tarfile.open(self.output, 'r:gz') as tar:
            member = tar.getmember(self.alias(link))
            self.assertTrue(member.issym()); self.assertEqual(member.linkname, target)
            inventory = json.load(tar.extractfile('inventory.json'))
            retained = next(m for m in inventory['members'] if m['original']['path'] == str(link))
            self.assertEqual(retained['kind'], 'symlink')
            self.assertEqual(retained['original'], archive.link_descriptor(link))

    def test_symlink_metadata_changed_during_archive_preserves_partial(self):
        link = self.inputs / 'link'; link.symlink_to('old-literal-target')
        def changed(): link.unlink(); link.symlink_to('new-literal-target')
        with self.assertRaisesRegex(ValueError, 'symlink changed'): self.make(after_inventory=changed)
        self.assertTrue(self.output.exists()); self.assertFalse(self.receipt.exists())

    def test_symlink_replaced_after_archive_fails_lstat_guard(self):
        link = self.inputs / 'link'; link.symlink_to('same-target')
        self.make(); link.unlink(); link.symlink_to('same-target')
        with self.assertRaisesRegex(ValueError, 'changed'): self.check()

    def test_external_link_target_bytes_never_bound_or_followed(self):
        target = self.work / 'EXTERNAL_TARGET'; target.write_bytes(b'MOCK unarchived target')
        (self.inputs / 'link').symlink_to(target)
        self.make(); target.write_bytes(b'MOCK external changed; symlink itself unchanged')
        result = self.check(); self.assertTrue(result['originals_rechecked'])

    def test_symlink_literal_header_tamper_rejected(self):
        (self.inputs / 'link').symlink_to('old-target')
        self.make(); data = bytearray(gzip.decompress(self.output.read_bytes()))
        with tarfile.open(fileobj=io.BytesIO(data), mode='r:') as standard:
            offset = standard.getmember(self.alias(self.inputs / 'link')).offset
        data[offset + 157] ^= 1
        self.rewrite_tar(bytes(data))
        with self.assertRaisesRegex(ValueError, 'header mismatch'): self.check()

    def test_magic_only_nonexecutable_fixture_retained_whole(self):
        path = self.inputs / 'ELF_NATIVE_MOCK_DATA'
        raw = b'\x7fELFsynthetic literal bytes, not an executable claim'
        path.write_bytes(raw); path.chmod(0o644)
        self.make(); self.check()
        with tarfile.open(self.output, 'r:gz') as tar:
            self.assertEqual(tar.extractfile(self.alias(path)).read(), raw)

    def test_duplicate_and_casefold_alias_rejected(self):
        inventory = archive.inventory_for(self.roots, [])
        for alias in [inventory['members'][0]['member_path'], inventory['members'][0]['member_path'].swapcase()]:
            corrupt = copy.deepcopy(inventory); corrupt['members'][1]['member_path'] = alias
            with self.assertRaisesRegex(ValueError, 'duplicate/casefold'): archive.validate_inventory(corrupt)

    def test_wrong_alias_ordinal_and_original_root_relationship_rejected(self):
        inventory = archive.inventory_for(self.roots, [])
        corrupt = copy.deepcopy(inventory); corrupt['members'][0]['member_path'] = 'PREP/objects/99999999'
        with self.assertRaises(ValueError): archive.validate_inventory(corrupt)
        corrupt = copy.deepcopy(inventory); corrupt['members'][0]['original']['path'] = str(self.work / 'OUTSIDE_ROOT')
        with self.assertRaisesRegex(ValueError, 'root/alias'): archive.validate_inventory(corrupt)

    def test_exact_duplicate_original_origin_rejected(self):
        inventory = archive.inventory_for(self.roots, [])
        inventory['members'][1]['original'] = copy.deepcopy(inventory['members'][0]['original'])
        with self.assertRaisesRegex(ValueError, 'duplicate original'): archive.validate_inventory(inventory)


if __name__ == '__main__': unittest.main()
