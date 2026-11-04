#!/usr/bin/env python3
"""Check that a new example-141 report differs from the previous one only in
the expected source hashes.

The two reports must be byte-identical outside their `selected_source_sha256`
objects. Inside, the keys must be the same, exactly the expected sources may
change hash, and each changed hash must match that file in the checkout.
Prints a JSON summary; exits 1 on any other difference. Uses no third-party
packages.

Usage: compare_reports.py OLD.json NEW.json SOURCE...
"""
import hashlib
import json
import sys
from pathlib import Path

KEY = b'"selected_source_sha256": {'


def split(raw):
    """The bytes before the source object, the object and the bytes after."""
    start = raw.index(KEY)
    if raw.count(KEY) != 1:
        raise SystemExit("the source object must appear once")
    end = raw.index(b"}", start) + 1
    return raw[:start], raw[start:end], raw[end:]


def main():
    old_path, new_path, *expected = sys.argv[1:]
    old, new = Path(old_path).read_bytes(), Path(new_path).read_bytes()
    old_head, old_sources, old_tail = split(old)
    new_head, new_sources, new_tail = split(new)
    outside_identical = old_head == new_head and old_tail == new_tail
    old_map = json.loads(old)["selected_source_sha256"]
    new_map = json.loads(new)["selected_source_sha256"]
    changed = sorted(key for key in old_map if old_map[key] != new_map.get(key))
    current = {key: hashlib.sha256(Path(key).read_bytes()).hexdigest() for key in changed}
    summary = {
        "old": {"path": old_path, "bytes": len(old), "sha256": hashlib.sha256(old).hexdigest()},
        "new": {"path": new_path, "bytes": len(new), "sha256": hashlib.sha256(new).hexdigest()},
        "bytes_outside_source_object_identical": outside_identical,
        "same_source_keys": sorted(old_map) == sorted(new_map),
        "changed_sources": {key: {"old": old_map[key], "new": new_map[key]} for key in changed},
        "changed_sources_are_expected": changed == sorted(expected),
        "changed_hashes_match_checkout": all(new_map[key] == current[key] for key in changed),
    }
    print(json.dumps(summary, indent=2))
    ok = (outside_identical and summary["same_source_keys"]
          and summary["changed_sources_are_expected"]
          and summary["changed_hashes_match_checkout"])
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
