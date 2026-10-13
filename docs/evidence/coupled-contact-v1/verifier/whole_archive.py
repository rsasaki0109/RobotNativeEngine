"""Whole-file PAX archive and streaming literal-byte verification; no numerical analysis."""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import struct
import tarfile
import zlib

CHUNK, INVENTORY_CAP = 65536, 64 * (1 << 20)
FREE_FLOOR, VALIDATION_RESERVE = 4 * (1 << 30), 64 * (1 << 20)
KIND = 'rne_whole_coupled9_literal_archive_v3'


def need(condition, message):
    if not condition: raise ValueError(message)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=True, allow_nan=False).encode()


def loads(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            need(key not in result, 'duplicate JSON key')
            result[key] = value
        return result
    def bad(value): raise ValueError('nonfinite JSON: ' + value)
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=bad)


def read_json(path):
    with Path(path).open('rb') as file: raw = file.read(INVENTORY_CAP + 1)
    need(len(raw) <= INVENTORY_CAP, 'inventory/receipt metadata cap')
    return loads(raw)


def safe_member(name):
    need(type(name) is str and name and '\\' not in name and '\0' not in name, 'unsafe archive member')
    parts = name.split('/')
    need(not name.startswith('/') and all(p not in ('', '.', '..') for p in parts), 'unsafe member traversal')
    need(all(not any(ord(c) < 32 for c in p) and ':' not in p for p in parts), 'unsafe member component')
    need(str(PurePosixPath(name)) == name, 'noncanonical member path')
    return name


def descriptor(path):
    path = Path(path)
    need(path.is_absolute() and stat.S_ISREG(path.lstat().st_mode), 'absolute regular nonsymlink file')
    digest, size = hashlib.sha256(), 0
    with path.open('rb') as file:
        while block := file.read(CHUNK): digest.update(block); size += len(block)
    return dict(path=str(path), bytes=size, sha256=digest.hexdigest())


def descriptor_shape(value):
    need(type(value) is dict and set(value) == {'path', 'bytes', 'sha256'}, 'descriptor keys')
    need(type(value['path']) is str and Path(value['path']).is_absolute(), 'original absolute label')
    need(type(value['bytes']) is int and value['bytes'] >= 0, 'descriptor size type')
    sha = value['sha256']
    need(type(sha) is str and len(sha) == 64 and all(c in '0123456789abcdef' for c in sha), 'SHA256 lowercase')


def bound(value):
    descriptor_shape(value)
    need(descriptor(value['path']) == value, 'whole original changed: ' + value['path'])


def link_descriptor(path):
    path = Path(path); info = path.lstat()
    need(path.is_absolute() and stat.S_ISLNK(info.st_mode), 'absolute symlink object required')
    return dict(path=str(path), link_target=os.readlink(path), size=info.st_size,
                mode=info.st_mode, uid=info.st_uid, gid=info.st_gid, device=info.st_dev,
                inode=info.st_ino, mtime_ns=info.st_mtime_ns, ctime_ns=info.st_ctime_ns)


def link_shape(value):
    keys = {'path', 'link_target', 'size', 'mode', 'uid', 'gid', 'device', 'inode', 'mtime_ns', 'ctime_ns'}
    need(type(value) is dict and set(value) == keys, 'symlink metadata keys')
    need(type(value['path']) is str and Path(value['path']).is_absolute(), 'symlink original label')
    need(type(value['link_target']) is str and '\0' not in value['link_target'], 'literal symlink target')
    for key in keys - {'path', 'link_target'}: need(type(value[key]) is int, 'symlink metadata integer')
    need(stat.S_ISLNK(value['mode']) and value['size'] == len(os.fsencode(value['link_target'])), 'symlink lstat type/size')


def member_header(member):
    original = member['original']
    return header(member['member_path'], original['bytes']) if member['kind'] == 'regular_file' else header(member['member_path'], 0, original['link_target'])


def member_size(member): return member['original']['bytes'] if member['kind'] == 'regular_file' else 0


def header(name, size, link_target=None):
    safe_member(name)
    info = tarfile.TarInfo(name)
    info.size, info.mtime, info.uid, info.gid, info.mode = size, 0, 0, 0, 0o644
    info.uname = info.gname = ''
    info.type = tarfile.REGTYPE
    if link_target is not None:
        info.type, info.linkname, info.mode, info.size = tarfile.SYMTYPE, link_target, 0o777, 0
    return info.tobuf(format=tarfile.PAX_FORMAT, encoding='utf-8', errors='strict')


def padding(size): return (-size) % 512


def validate_inventory(inventory):
    need(type(inventory) is dict and set(inventory) == {'kind', 'schema_version', 'roots', 'members', 'local_only', 'scope'}, 'inventory keys')
    need(inventory['kind'] == KIND and type(inventory['schema_version']) is int and inventory['schema_version'] == 3, 'inventory version')
    need(type(inventory['roots']) is dict and inventory['roots'], 'named roots required')
    for label, path in inventory['roots'].items():
        need(re.fullmatch(r'[A-Za-z0-9_-]{1,64}', label) is not None and label != 'inventory.json', 'root prefix')
        need(type(path) is str and Path(path).is_absolute(), 'root original label')
    need(len({label.casefold() for label in inventory['roots']}) == len(inventory['roots']), 'casefold root label collision')
    need(type(inventory['members']) is list and type(inventory['local_only']) is list, 'inventory lists')
    names, paths, folded = set(), set(), set()
    for member in inventory['members']:
        need(type(member) is dict and set(member) == {'member_path', 'kind', 'original'}, 'member keys')
        name = safe_member(member['member_path'])
        need(member['kind'] in ('regular_file', 'symlink'), 'member object kind')
        if member['kind'] == 'regular_file': descriptor_shape(member['original'])
        else: link_shape(member['original'])
        need(name not in names and name.casefold() not in folded, 'duplicate/casefold member')
        need(name.split('/')[0] in inventory['roots'] and '/' in name, 'member root mismatch')
        root = Path(inventory['roots'][name.split('/')[0]])
        original = Path(member['original']['path'])
        need(str(root) == inventory['roots'][name.split('/')[0]] and str(original) == member['original']['path'],
             'canonical original root/path labels')
        need(original != root and original.is_relative_to(root) and '..' not in original.parts,
             'original root/alias relationship')
        need(member['original']['path'] not in paths, 'duplicate original path')
        names.add(name); folded.add(name.casefold()); paths.add(member['original']['path'])
    need([m['member_path'] for m in inventory['members']] == sorted(names), 'member order')
    for label in sorted(inventory['roots']):
        selected = [member for member in inventory['members'] if member['member_path'].split('/')[0] == label]
        need([member['original']['path'] for member in selected] == sorted(member['original']['path'] for member in selected),
             'original lexical order')
        for ordinal, member in enumerate(selected, 1):
            need(member['member_path'] == f'{label}/objects/{ordinal:08d}', 'deterministic portable alias ordinal')
    for value in inventory['local_only']:
        descriptor_shape(value)
        need(value['path'] not in paths, 'duplicate/local-only recoverable alias')
        paths.add(value['path'])
    return inventory


def inventory_for(roots, local_only):
    need(type(roots) is dict and roots and type(local_only) is list, 'roots/local-only types')
    excluded = {}
    for item in local_only:
        bound(item); need(item['path'] not in excluded, 'duplicate executable binding')
        excluded[item['path']] = item
    members, root_paths = [], []
    for label, text in sorted(roots.items()):
        need(type(label) is str and re.fullmatch(r'[A-Za-z0-9_-]{1,64}', label), 'root prefix')
        root = Path(text)
        need(root.is_absolute() and root.resolve() == root and stat.S_ISDIR(root.lstat().st_mode), 'real named root')
        need(not any(root.is_relative_to(p) or p.is_relative_to(root) for p in root_paths), 'overlapping roots')
        root_paths.append(root)
        for folder, dirs, files in os.walk(root, followlinks=False):
            dirs.sort(); files.sort()
            for name in list(dirs):
                path = Path(folder) / name
                if stat.S_ISLNK(path.lstat().st_mode):
                    members.append(dict(member_path=safe_member(label + '/' + path.relative_to(root).as_posix()),
                                        kind='symlink', original=link_descriptor(path)))
                    dirs.remove(name)
                else: need(stat.S_ISDIR(path.lstat().st_mode), 'nonregular directory')
            for name in files:
                path = Path(folder) / name
                if stat.S_ISLNK(path.lstat().st_mode):
                    members.append(dict(member_path=safe_member(label + '/' + path.relative_to(root).as_posix()),
                                        kind='symlink', original=link_descriptor(path)))
                    continue
                need(stat.S_ISREG(path.lstat().st_mode), 'nonregular input')
                if str(path) in excluded:
                    bound(excluded[str(path)]); continue
                with path.open('rb') as file: magic = file.read(4)
                executable_mode = bool(path.lstat().st_mode & 0o111)
                need(not (executable_mode and (magic == b'\x7fELF' or magic[:2] == b'MZ' or magic in
                          (b'\xcf\xfa\xed\xfe', b'\xfe\xed\xfa\xcf', b'\xce\xfa\xed\xfe', b'\xfe\xed\xfa\xce'))),
                     'compiled executable missing explicit local-only ledger: ' + str(path))
                members.append(dict(member_path=safe_member(label + '/' + path.relative_to(root).as_posix()), kind='regular_file', original=descriptor(path)))
    aliased = []
    for label in sorted(roots):
        selected = sorted((member for member in members if member['member_path'].split('/')[0] == label),
                          key=lambda member: member['original']['path'])
        for ordinal, member in enumerate(selected, 1):
            aliased.append(dict(member_path=f'{label}/objects/{ordinal:08d}', kind=member['kind'], original=member['original']))
    return validate_inventory(dict(kind=KIND, schema_version=3, roots=roots,
                         members=aliased, local_only=local_only,
                         scope='Whole regular original bytes plus exact lstat/readlink symlink objects, never dereferenced/extracted. Executable descriptors are local-only, not recoverable bytes. No scientific accuracy/adoption/public-availability verdict.'))


def plan(inventory):
    raw = canonical(validate_inventory(inventory)) + b'\n'
    need(len(raw) <= INVENTORY_CAP, 'inventory metadata cap')
    size = len(header('inventory.json', len(raw))) + len(raw) + padding(len(raw))
    for member in inventory['members']:
        count = member_size(member)
        size += len(member_header(member)) + count + padding(count)
    size += 1024
    size += (-size) % tarfile.RECORDSIZE
    # Literal stored-deflate block encoding: no compression-ratio prediction.
    gzip_bytes = size + 5 * (size // 65535 + 1) + 18
    return raw, dict(tar_bytes=size, archive_upper_bound_bytes=gzip_bytes,
                     free_floor_bytes=FREE_FLOOR, validation_reservation_bytes=VALIDATION_RESERVE,
                     required_free_bytes=gzip_bytes + FREE_FLOOR + VALIDATION_RESERVE)


class StoredGzip:
    """Canonical gzip with streaming stored DEFLATE blocks and exact size bound."""
    def __init__(self, file):
        self.file, self.buffer, self.crc, self.total = file, bytearray(), 0, 0
        file.write(bytes.fromhex('1f8b08000000000000ff'))
    def write(self, raw):
        self.crc = zlib.crc32(raw, self.crc); self.total += len(raw)
        self.buffer.extend(raw)
        while len(self.buffer) >= 65535:
            block = bytes(self.buffer[:65535]); del self.buffer[:65535]
            self.file.write(b'\0' + struct.pack('<HH', len(block), len(block) ^ 65535) + block)
    def finish(self):
        block = bytes(self.buffer)
        self.file.write(b'\x01' + struct.pack('<HH', len(block), len(block) ^ 65535) + block)
        self.file.write(struct.pack('<II', self.crc, self.total & 0xffffffff))


def sync_directory(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try: os.fsync(fd)
    finally: os.close(fd)


def write_json(path, value):
    raw = canonical(value) + b'\n'; need(len(raw) <= INVENTORY_CAP, 'receipt metadata cap')
    with Path(path).open('xb') as file: file.write(raw); file.flush(); os.fsync(file.fileno())
    sync_directory(Path(path).parent)


def outside(path, roots):
    path = Path(path)
    need(path.is_absolute() and path.parent.is_dir() and path.parent.resolve() == path.parent, 'absolute real output parent')
    need(not any(path.is_relative_to(Path(root)) for root in roots.values()), 'output inside input root')
    need(not path.exists(), 'exclusive no overwrite archive/receipt')


def create(roots, local_only, archive, receipt, *, disk_free=shutil.disk_usage, after_inventory=None):
    outside(archive, roots); outside(receipt, roots)
    inventory = inventory_for(roots, local_only)
    raw, budget = plan(inventory)
    need(disk_free(Path(archive).parent).free >= budget['required_free_bytes'], 'full archive bound/freefloor/reservation does not fit')
    if after_inventory is not None: after_inventory()
    wrote = 0
    with Path(archive).open('xb') as file:
        gzip = StoredGzip(file)
        def emit(data):
            nonlocal wrote
            need(wrote + len(data) <= budget['tar_bytes'], 'planned tar upper bound')
            gzip.write(data); wrote += len(data)
        try:
            emit(header('inventory.json', len(raw))); emit(raw); emit(b'\0' * padding(len(raw)))
            for member in inventory['members']:
                original = member['original']; emit(member_header(member))
                if member['kind'] == 'symlink':
                    need(link_descriptor(original['path']) == original, 'symlink changed during archive')
                    continue
                digest, count = hashlib.sha256(), 0
                with Path(original['path']).open('rb') as source:
                    while block := source.read(CHUNK):
                        count += len(block); need(count <= original['bytes'], 'original grew during archive')
                        digest.update(block); emit(block)
                need(count == original['bytes'] and digest.hexdigest() == original['sha256'], 'original changed during archive')
                emit(b'\0' * padding(count))
            emit(b'\0' * (budget['tar_bytes'] - wrote)); gzip.finish()
        finally:
            file.flush(); os.fsync(file.fileno())
    sync_directory(Path(archive).parent)
    need(inventory_for(roots, local_only) == inventory, 'original inventory changed after archive')
    archived = descriptor(archive)
    need(archived['bytes'] == budget['archive_upper_bound_bytes'], 'stored gzip bound differs')
    result = dict(kind='rne_whole_archive_create_receipt', schema_version=1, archive=archived,
                  inventory_sha256=hashlib.sha256(raw).hexdigest(), inventory_bytes=len(raw),
                  budget=budget, members=len(inventory['members']), local_only=len(local_only),
                  originals_unchanged=True, publication='NOT_AUTHORIZED_PUBLIC_RETENTION_PENDING',
                  scientific_reader_invocations=0, numerical_analysis=False)
    write_json(receipt, result)
    return result


class GzipReader:
    def __init__(self, path, expected):
        self.file = Path(path).open('rb'); self.decoder = zlib.decompressobj(31)
        self.pending = b''; self.buffer = bytearray(); self.digest = hashlib.sha256(); self.count = 0
        self.expected, self.done = expected, False
    def read(self, count):
        while len(self.buffer) < count and not self.done:
            if not self.pending:
                self.pending = self.file.read(CHUNK)
                if not self.pending:
                    need(self.decoder.eof, 'truncated gzip/CRC/EOF'); self.done = True; break
                self.digest.update(self.pending); self.count += len(self.pending)
                need(self.count <= self.expected['bytes'], 'archive exceeds declared stored size')
                need(not self.decoder.eof, 'trailing gzip or multiple members')
            decoded = self.decoder.decompress(self.pending, CHUNK)
            self.pending = self.decoder.unconsumed_tail; self.buffer.extend(decoded)
            need(not self.decoder.unused_data, 'trailing gzip or multiple members')
        result = bytes(self.buffer[:count]); del self.buffer[:count]
        return result
    def exact(self, count):
        data = self.read(count); need(len(data) == count, 'truncated tar/member'); return data
    def finish(self):
        need(self.read(1) == b'', 'extra decoded tar bytes')
        need(self.decoder.eof and not self.pending and not self.decoder.unused_data, 'single gzip CRC/EOF')
        need(self.count == self.expected['bytes'] and self.digest.hexdigest() == self.expected['sha256'], 'whole archive SHA/length')
    def close(self): self.file.close()


def verify(archive, create_receipt, output, *, recheck_originals=True):
    receipt = read_json(create_receipt)
    expected = receipt['archive']; descriptor_shape(expected)
    need(Path(archive).stat().st_size == expected['bytes'], 'whole archive stored size')
    stream = GzipReader(archive, expected)
    try:
        first = stream.exact(512)
        need(first[:14] == b'inventory.json', 'inventory must be first literal tar member')
        inventory_size = int(first[124:136].strip(b'\0 ') or b'0', 8)
        need(inventory_size <= INVENTORY_CAP and first == header('inventory.json', inventory_size), 'inventory tar header/cap')
        raw = stream.exact(inventory_size)
        need(raw.endswith(b'\n'), 'inventory LF framing')
        inventory = validate_inventory(loads(raw))
        canonical_raw, budget = plan(inventory)
        need(raw == canonical_raw and receipt['inventory_sha256'] == hashlib.sha256(raw).hexdigest() and
             receipt['inventory_bytes'] == len(raw) and receipt['budget'] == budget, 'inventory/plan binding')
        need(stream.exact(padding(len(raw))) == b'\0' * padding(len(raw)), 'inventory tar padding')
        used = 512 + len(raw) + padding(len(raw))
        for member in inventory['members']:
            original = member['original']; literal = member_header(member)
            need(stream.exact(len(literal)) == literal, 'member literal PAX/header mismatch')
            count = member_size(member)
            digest, left = hashlib.sha256(), count
            while left:
                block = stream.exact(min(CHUNK, left)); digest.update(block); left -= len(block)
            if member['kind'] == 'regular_file': need(digest.hexdigest() == original['sha256'], 'whole member SHA mismatch')
            pad = padding(count)
            need(stream.exact(pad) == b'\0' * pad, 'member tar padding')
            used += len(literal) + count + pad
        left = budget['tar_bytes'] - used
        need(left >= 1024, 'missing tar end marker')
        while left:
            count = min(CHUNK, left); need(stream.exact(count) == b'\0' * count, 'extra member/nonzero tar trailer'); left -= count
        stream.finish()
    finally: stream.close()
    if recheck_originals: need(inventory_for(inventory['roots'], inventory['local_only']) == inventory, 'originals changed after verification')
    result = dict(kind='rne_whole_archive_verify_receipt', status='PASS_WHOLE_LITERAL_ARCHIVE_BYTES_ONLY',
                  archive=descriptor(archive), create_receipt=descriptor(create_receipt),
                  members=len(inventory['members']), local_only_bindings=len(inventory['local_only']),
                  symlink_objects=sum(member['kind'] == 'symlink' for member in inventory['members']),
                  originals_rechecked=recheck_originals, public_availability_verified=False,
                  scientific_reader_invocations=0, numerical_analysis=False)
    outside(output, inventory['roots']); write_json(output, result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='operation', required=True)
    make = sub.add_parser('create')
    for name in ['roots-json', 'local-only-json', 'archive', 'receipt']: make.add_argument('--' + name, required=True)
    check = sub.add_parser('verify')
    for name in ['archive', 'create-receipt', 'output']: check.add_argument('--' + name, required=True)
    check.add_argument('--no-originals', action='store_true')
    args = parser.parse_args()
    if args.operation == 'create':
        result = create(read_json(args.roots_json), read_json(args.local_only_json), args.archive, args.receipt)
    else: result = verify(args.archive, args.create_receipt, args.output, recheck_originals=not args.no_originals)
    print(json.dumps(result, sort_keys=True, allow_nan=False))


if __name__ == '__main__': main()
