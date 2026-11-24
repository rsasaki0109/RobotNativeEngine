#!/usr/bin/env python3
"""Add the workspace's vendored patch crates to a maturin sdist.

The root `Cargo.toml` patches crates.io dependencies (`rapier3d`,
`wgpu-3dgs-viewer`) with path dependencies under `third_party/`. maturin does
not copy path *patches* into the sdist and its `include` option cannot reach
outside the project directory, so building the sdist from source fails in
`cargo metadata`. This repacks the sdist with those directories added at the
same relative paths the patches expect.
"""

from __future__ import annotations

import sys
import tarfile
import tomllib
from pathlib import Path

SKIP_DIRS = {"target"}


def patch_paths(root: Path) -> list[str]:
    manifest = tomllib.loads((root / "Cargo.toml").read_text())
    paths = []
    for registry in manifest.get("patch", {}).values():
        for spec in registry.values():
            if isinstance(spec, dict) and "path" in spec:
                paths.append(spec["path"])
    return sorted(set(paths))


def main(sdist: str, repo_root: str) -> None:
    sdist_path = Path(sdist)
    root = Path(repo_root)
    vendored = patch_paths(root)
    if not vendored:
        raise SystemExit("no [patch] path dependencies found; nothing to add")
    for rel in vendored:
        if not (root / rel / "Cargo.toml").is_file():
            raise SystemExit(f"missing vendored crate: {root / rel}")
    tmp = sdist_path.with_suffix(".tmp")
    with tarfile.open(sdist_path, "r:gz") as old:
        names = old.getnames()
        prefix = names[0].split("/")[0]
        with tarfile.open(tmp, "w:gz") as new:
            for member in old.getmembers():
                new.addfile(member, old.extractfile(member) if member.isfile() else None)
            for vendored_rel in vendored:
                for path in sorted((root / vendored_rel).rglob("*")):
                    rel = path.relative_to(root)
                    if SKIP_DIRS.intersection(rel.parts) or not path.is_file():
                        continue
                    new.add(path, arcname=f"{prefix}/{rel.as_posix()}", recursive=False)
    tmp.replace(sdist_path)
    print(f"added {', '.join(vendored)} to {sdist_path.name}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("usage: sdist_add_vendored.py <sdist.tar.gz> <repo-root>")
    main(sys.argv[1], sys.argv[2])
