#!/usr/bin/env python3
"""Download the pinned models in models.toml into a cache, checking SHA-256.

Usage: fetch_models.py [--cache DIR] [--model ID] [--variant NAME ...]
The default cache is target/models under the repository root. Existing files
with the right digest are kept. Nothing downloaded is executed.
"""
import argparse
import hashlib
import tomllib
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]


def digest(path):
    hasher = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            hasher.update(block)
    return hasher.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", type=Path, default=REPO / "target/models")
    parser.add_argument("--model")
    parser.add_argument("--variant", action="append")
    arguments = parser.parse_args()
    manifest = tomllib.loads((HERE / "models.toml").read_text())
    for model in manifest["model"]:
        if arguments.model and model["id"] != arguments.model:
            continue
        for file in model["file"]:
            if arguments.variant and file["variant"] not in arguments.variant:
                continue
            target = arguments.cache / model["id"] / model["revision"] / file["path"]
            if target.is_file() and digest(target) == file["sha256"]:
                print(f"cached  {target}")
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            url = f"{model['repository']}/resolve/{model['revision']}/{file['path']}"
            partial = target.with_suffix(target.suffix + ".part")
            with urllib.request.urlopen(url) as response, partial.open("wb") as out:
                while block := response.read(1 << 20):
                    out.write(block)
            if partial.stat().st_size != file["bytes"] or digest(partial) != file["sha256"]:
                partial.unlink()
                raise ValueError(f"{url} does not match the pinned size and SHA-256")
            partial.rename(target)
            print(f"fetched {target}")


if __name__ == "__main__":
    main()
