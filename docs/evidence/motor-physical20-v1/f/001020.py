"""Strict, bounded framing and prospective artifact binding; no engine calls."""
import hashlib
import json
import os
from pathlib import Path
import zlib


def need(ok, message):
    if not ok:
        raise ValueError(message)


def keys(value, names):
    need(type(value) is dict and set(value) == set(names.split()), 'unexpected object schema')


def integer(value, width=64):
    need(type(value) is int and 0 <= value < 1 << width, 'invalid unsigned integer')
    return value


def words(value, count=None, width=32):
    need(type(value) is list and (count is None or len(value) == count), 'invalid word vector')
    for item in value:
        integer(item, width)
    return value


def boolean(value):
    need(type(value) is bool, 'invalid boolean')


def canonical(value):
    # Raw streams contain no JSON floating values: all native values are integers.
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=True).encode()


def _pairs(items):
    out = {}
    for key, value in items:
        need(key not in out, 'duplicate JSON key')
        out[key] = value
    return out


def _no_float(value):
    raise ValueError('JSON floating values are forbidden; use native IEEE words')


def loads(data):
    if type(data) is bytes:
        data = data.decode('utf-8', errors='strict')
    return json.loads(data, object_pairs_hook=_pairs, parse_float=_no_float, parse_constant=_no_float)


def descriptor(path):
    path = Path(path)
    data = path.read_bytes()
    return dict(path=str(path), sha256=hashlib.sha256(data).hexdigest(), bytes=len(data))


def bound(desc):
    keys(desc, 'path sha256 bytes')
    need(type(desc['path']) is str and Path(desc['path']).is_absolute(), 'artifact path must be absolute')
    integer(desc['bytes'])
    need(type(desc['sha256']) is str and len(desc['sha256']) == 64 and
         all(c in '0123456789abcdef' for c in desc['sha256']), 'invalid SHA256')
    need(descriptor(desc['path']) == desc, 'artifact SHA/size/path mismatch')


def gzip_records(path, max_compressed=16 * 1024 * 1024, max_raw=32 * 1024 * 1024):
    need(Path(path).stat().st_size <= max_compressed, 'compressed file exceeds registered bound')
    with Path(path).open('rb') as file:
        data = file.read(max_compressed + 1)
    need(len(data) <= max_compressed, 'compressed file exceeds registered bound')
    need(data[:10] == bytes.fromhex('1f8b08000000000000ff'), 'gzip header differs from fixed source GzBuilder settings')
    decoder = zlib.decompressobj(16 + zlib.MAX_WBITS)
    raw = decoder.decompress(data, max_raw + 1)
    need(len(raw) <= max_raw and not decoder.unconsumed_tail, 'raw file exceeds registered bound')
    need(decoder.eof, 'truncated gzip member')
    need(not decoder.unused_data, 'trailing bytes or multiple gzip members')
    need(raw.endswith(b'\n') and b'\r' not in raw, 'JSONL must finish with LF without CR')
    lines = [line + b'\n' for line in raw.split(b'\n')[:-1]]
    need(all(line != b'\n' for line in lines), 'blank JSONL record')
    records = [loads(line[:-1]) for line in lines]
    return records, lines, dict(stored=descriptor(path),
        uncompressed_sha256=hashlib.sha256(raw).hexdigest(), uncompressed_bytes=len(raw),
        gzip_members=1, complete_eof=True)


def write_exclusive(path, value):
    path = Path(path)
    with path.open('xb') as file:
        file.write(json.dumps(value, indent=2, ensure_ascii=True).encode() + b'\n')
        file.flush()
        os.fsync(file.fileno())
    sync_directory(path.parent)


def sync_directory(path):
    """Persist directory entries, including newly created child directories."""
    fd = os.open(path, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def loads_metadata(data):
    """Only frozen case metadata admits defined nominal decimal literals.

    Native framing still calls loads(), which requires native IEEE integer words.
    Descriptor/count fields are separately checked as exact ints, excluding bools.
    """
    if type(data) is bytes:
        data = data.decode('utf-8', errors='strict')
    return json.loads(data, object_pairs_hook=_pairs, parse_constant=_no_float)
