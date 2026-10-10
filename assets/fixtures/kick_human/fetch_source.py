#!/usr/bin/env python3
"""Fetch the pinned CC0 model inputs and verify every byte before generation."""

import hashlib
import json
from pathlib import Path
import urllib.request


def main():
    root = Path(__file__).resolve().parent
    output = root / "source"
    output.mkdir(exist_ok=True)
    for entry in json.loads((root / "source-manifest.json").read_text()):
        path = output / entry["local_filename"]
        if path.exists():
            data = path.read_bytes()
        else:
            with urllib.request.urlopen(entry["download_url"], timeout=60) as response:
                data = response.read()
        if len(data) != entry["size_bytes"]:
            raise ValueError(f"Unexpected size: {path.name}")
        if hashlib.sha256(data).hexdigest() != entry["sha256"]:
            raise ValueError(f"Unexpected SHA-256: {path.name}")
        path.write_bytes(data)
        print(f"verified {path.name}")


if __name__ == "__main__":
    main()
