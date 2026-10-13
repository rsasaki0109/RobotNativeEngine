#!/usr/bin/env python3
"""Verify declared portable evidence bytes without running archived programs.

Original absolute paths are historical labels, never extraction destinations.
This checker uses only the manifest and local payload files. It does not fetch
or read original files, run snapshots/readers/worlds, or infer numerical accuracy.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import stat
import sys
import zlib

CAP = 1 << 20
PREFIX = 'docs/evidence/motor-visit-placement-v1/'
MAX_CHECKOUT_PATH = 166
CHUNK = 65536
KIND = 'rne_whole_visit_placement_prepared_inline_retention'
MANIFEST_KEYS = {'kind', 'schema_version', 'aliases', 'payloads', 'local_only',
                 'original_audit_restoration_receipt', 'scope'}
SIDECARS = {'manifest.json', '.gitattributes', 'verify_literals.py', 'README.md'}
RESERVED = {'CON', 'PRN', 'AUX', 'NUL', 'CONIN$', 'CONOUT$'} | {
    stem + str(number) for stem in ('COM', 'LPT') for number in range(1, 10)}


class ValidationError(ValueError):
    """A declared byte, schema, framing or portable-path check failed."""


def require(condition, message):
    """Fail validation without recovering, importing or executing any payload."""
    if not condition:
        raise ValidationError(message)


def exact_keys(value, names, label):
    require(type(value) is dict and set(value) == set(names), label + ': unexpected keys/type')


def integer(value, label):
    require(type(value) is int and 0 <= value < (1 << 64), label + ': expected unsigned integer, not bool/float')


def sha256(value, label):
    require(type(value) is str and re.fullmatch('[0-9a-f]{64}', value) is not None,
            label + ': expected lowercase SHA-256')


def original_descriptor(value, label):
    exact_keys(value, {'path', 'sha256', 'bytes'}, label)
    require(type(value['path']) is str and bool(value['path']) and '\0' not in value['path'],
            label + ': invalid historical origin label')
    sha256(value['sha256'], label)
    integer(value['bytes'], label)


def relative_path(value):
    """Validate a portable relative payload name, without using origin labels."""
    require(type(value) is str and value.isascii() and bool(value), 'payload path must be nonempty ASCII')
    require(not any(ord(c) < 32 or ord(c) == 127 or c in '<>:"\\|?*' for c in value),
            'payload path has forbidden/control characters')
    path = PurePosixPath(value)
    require(not path.is_absolute() and path.as_posix() == value and
            all(part not in ('', '.', '..') for part in path.parts), 'noncanonical/traversing payload path')
    require(len(path.parts) >= 2 and path.parts[0] == 'f', 'payload must be under f/')
    require(len(PREFIX + value) <= MAX_CHECKOUT_PATH, 'Windows checkout path exceeds166 characters')
    for component in path.parts:
        require(not component.endswith(('.', ' ')), 'Windows trailing dot/space path')
        require(component.split('.')[0].upper() not in RESERVED, 'Windows reserved device path')
    return path


def payload_descriptor(value, label):
    exact_keys(value, {'relative_path', 'sha256', 'bytes'}, label)
    relative_path(value['relative_path'])
    sha256(value['sha256'], label)
    integer(value['bytes'], label)
    require(value['bytes'] <= CAP, label + ': stored payload exceeds1MiB')


def no_duplicates(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, 'duplicate JSON key: ' + key)
        result[key] = value
    return result


def no_float(_value):
    raise ValidationError('manifest JSON floating values/constants are unsupported')


def parse_manifest(raw):
    """Parse strict descriptor JSON only; archived payload JSON is never parsed."""
    try:
        return json.loads(raw.decode('utf-8', errors='strict'), object_pairs_hook=no_duplicates,
                          parse_float=no_float, parse_constant=no_float)
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ValidationError('invalid manifest UTF-8/JSON') from error


def regular_file(root, relative):
    """Reject symlink components before reading a declared local file."""
    path = root
    components = PurePosixPath(relative).parts
    for index, component in enumerate(components):
        path = path / component
        try:
            mode = path.lstat().st_mode
        except OSError as error:
            raise ValidationError('missing/inaccessible path: ' + relative) from error
        require(not stat.S_ISLNK(mode), 'symlink path: ' + relative)
        if index == len(components) - 1:
            require(stat.S_ISREG(mode), 'nonregular file: ' + relative)
        else:
            require(stat.S_ISDIR(mode), 'nondirectory parent: ' + relative)
    return path


def read_stored(root, relative):
    path = regular_file(root, relative)
    require(path.stat().st_size <= CAP, 'inline evidence file exceeds1MiB: ' + relative)
    with path.open('rb') as file:
        raw = file.read(CAP + 1)
    require(len(raw) <= CAP, 'inline evidence grew beyond1MiB: ' + relative)
    return raw


def check_gzip(raw, expected):
    """Stream one complete member with bounded memory and declared-size overflow detection.

    The original may exceed1MiB. Only the stored file has that policy threshold.
    No decompressed bytes are written; chunks are hashed and immediately discarded.
    """
    require(raw[:3] == bytes.fromhex('1f8b08'), 'not a gzip member')
    decoder = zlib.decompressobj(16 + zlib.MAX_WBITS)
    pending = raw
    size = 0
    digest = hashlib.sha256()
    try:
        while True:
            chunk = decoder.decompress(pending, min(CHUNK, expected['bytes'] - size + 1))
            size += len(chunk)
            require(size <= expected['bytes'], 'gzip restored bytes exceed declared original size')
            digest.update(chunk)
            pending = decoder.unconsumed_tail
            if decoder.eof:
                require(decoder.unused_data == b'' and pending == b'',
                        'multiple gzip members or trailing data')
                break
            require(bool(pending), 'truncated/incomplete gzip member')
    except zlib.error as error:
        raise ValidationError('invalid gzip stream/CRC') from error
    require(size == expected['bytes'], 'gzip restored size mismatch')
    require(digest.hexdigest() == expected['sha256'], 'gzip restored SHA-256 mismatch')


def verify(root):
    """Validate all declared local representations, while preserving local-only distinctions."""
    root = Path(root).absolute()
    try:
        mode = root.lstat().st_mode
    except OSError as error:
        raise ValidationError('bundle root is missing') from error
    require(stat.S_ISDIR(mode) and not stat.S_ISLNK(mode), 'bundle root must be a nonsymlink directory')
    root = root.resolve()
    require(read_stored(root, '.gitattributes') == b'* -text\n', 'literal-byte .gitattributes missing')
    manifest_raw = read_stored(root, 'manifest.json')
    manifest = parse_manifest(manifest_raw)
    exact_keys(manifest, MANIFEST_KEYS, 'manifest')
    require(manifest['kind'] == KIND and type(manifest['schema_version']) is int and
            manifest['schema_version'] == 1, 'unsupported manifest kind/schema')
    require(type(manifest['scope']) is str and bool(manifest['scope']), 'missing manifest scope')
    for field in ('aliases', 'payloads', 'local_only'):
        require(type(manifest[field]) is list, field + ': expected array')
    expected = {}
    folded_components = {}
    for payload in manifest['payloads']:
        payload_descriptor(payload, 'payload')
        relative = payload['relative_path']
        require(relative not in expected, 'duplicate payload declaration')
        expected[relative] = payload
        parts = PurePosixPath(relative).parts
        for index in range(1, len(parts) + 1):
            component_path = '/'.join(parts[:index])
            identity = (component_path, 'file' if index == len(parts) else 'directory')
            folded = component_path.casefold()
            require(folded not in folded_components or folded_components[folded] == identity,
                    'casefold/component-kind collision')
            folded_components[folded] = identity
    stored = {}
    for relative, payload in expected.items():
        raw = read_stored(root, relative)
        require(len(raw) == payload['bytes'] and hashlib.sha256(raw).hexdigest() == payload['sha256'],
                'stored whole payload SHA/size mismatch: ' + relative)
        stored[relative] = raw
    origins = {}
    referenced = set()
    gzip_aliases = 0
    for alias in manifest['aliases']:
        exact_keys(alias, {'original', 'representation', 'encoding'}, 'alias')
        original_descriptor(alias['original'], 'alias original')
        payload_descriptor(alias['representation'], 'alias representation')
        original = alias['original']
        require(original['path'] not in origins, 'duplicate historical origin label')
        origins[original['path']] = original
        representation = alias['representation']
        relative = representation['relative_path']
        require(relative in expected and representation == expected[relative],
                'alias representation missing/conflicts with payload declaration')
        referenced.add(relative)
        if alias['encoding'] == 'whole-file-exact-bytes':
            require(original['bytes'] == representation['bytes'] and original['sha256'] == representation['sha256'],
                    'whole-file alias original SHA/size differs from representation')
        elif alias['encoding'] == 'lossless-single-member-gzip-original-exact-bytes':
            check_gzip(stored[relative], original)
            gzip_aliases += 1
        else:
            raise ValidationError('unsupported alias encoding')
    require(referenced == set(expected), 'unreferenced payload declaration')
    for item in manifest['local_only']:
        exact_keys(item, {'original', 'reason'}, 'local-only')
        original_descriptor(item['original'], 'local-only original')
        require(type(item['reason']) is str and bool(item['reason']), 'local-only reason missing')
        require(item['original']['path'] not in origins, 'duplicate local-only/recoverable origin label')
        origins[item['original']['path']] = item['original']
    original_descriptor(manifest['original_audit_restoration_receipt'], 'restoration receipt')
    receipt = manifest['original_audit_restoration_receipt']
    require(any(alias['original'] == receipt and alias['encoding'] == 'whole-file-exact-bytes'
                for alias in manifest['aliases']), 'restoration receipt must be included as exact whole bytes')
    actual = set()
    all_folded = {}
    for path in root.rglob('*'):
        relative = path.relative_to(root).as_posix()
        mode = path.lstat().st_mode
        require(not stat.S_ISLNK(mode), 'symlink in bundle tree: ' + relative)
        require(stat.S_ISREG(mode) or stat.S_ISDIR(mode), 'nonregular bundle entry: ' + relative)
        require(relative.casefold() not in all_folded or all_folded[relative.casefold()] == relative,
                'casefold collision in bundle tree')
        all_folded[relative.casefold()] = relative
        if stat.S_ISREG(mode):
            require(path.stat().st_size <= CAP, 'bundle file exceeds1MiB: ' + relative)
            actual.add(relative)
    require(actual - SIDECARS == set(expected), 'undeclared or missing payload file')
    return dict(status='PASS_DECLARED_PORTABLE_LITERAL_BYTES_ONLY',
                manifest_sha256=hashlib.sha256(manifest_raw).hexdigest(),
                manifest_bytes=len(manifest_raw), aliases=len(manifest['aliases']),
                unique_payloads=len(expected), gzip_original_representations=gzip_aliases,
                local_only_bindings=len(manifest['local_only']),
                local_only_scope='Historical executable/hash metadata only; no absent original bytes read, fetched or recovered.',
                native_invocations=0, scientific_reader_invocations=0, numerical_reanalysis=False,
                qualification_scope='Declared local payload/whole restoration bytes only; no scientific accuracy, production adoption, complete historical coverage or public availability verdict.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    try:
        result = verify(args.root)
    except (OSError, ValidationError) as error:
        print('literal verification failed: ' + str(error), file=sys.stderr)
        return 1
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
