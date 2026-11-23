#!/usr/bin/env python3
"""Fetches PLATEAU Tokyo 23-ward CityGML meshes without downloading the archive.

The official archive (3D City Model, Tokyo 23 wards, 2020, CityGML v2) is a
5.3 GB zip. Zip files keep a central directory at the end, so this script
reads it with HTTP range requests and then extracts only the requested
members (a building and a road file per mesh, 10-50 MB each).

    python3 scripts/fetch_plateau_tokyo.py            # Shibuya and Shinjuku
    python3 scripts/fetch_plateau_tokyo.py 53393586   # one third-level mesh

Files land in target/plateau/tokyo23/. Data: 3D City Model (Tokyo 23 wards)
2020, Project PLATEAU / MLIT, under the PLATEAU site policy (compatible with
CC BY 4.0); attribute it when redistributing results.
"""

import argparse
import io
import pathlib
import sys
import urllib.request
import zipfile

ARCHIVE = (
    "https://assets.cms.plateau.reearth.io/assets/ec/d51c64-a47f-4a56-aa64-340d1d3c720b/"
    "13100_tokyo23-ku_2020_citygml_4_2_op.zip"
)
ROOT = "13100_tokyo23-ku_2020_citygml_4_2_op/udx"
# Third-level (about 1 km) meshes covering 450 m around the stations (each
# station sits near a mesh corner).
DEFAULT_MESHES = {
    "53393585": "Shibuya",
    "53393586": "Shibuya",
    "53393595": "Shibuya",
    "53393596": "Shibuya",
    "53394525": "Shinjuku",
    "53394526": "Shinjuku",
    "53394535": "Shinjuku",
    "53394536": "Shinjuku",
}


class HttpRangeFile(io.RawIOBase):
    """A read-only seekable file over HTTP range requests."""

    def __init__(self, url):
        self.url = url
        self.pos = 0
        head = urllib.request.urlopen(urllib.request.Request(url, method="HEAD"))
        self.size = int(head.headers["Content-Length"])

    def readable(self):
        return True

    def seekable(self):
        return True

    def tell(self):
        return self.pos

    def seek(self, offset, whence=io.SEEK_SET):
        base = {io.SEEK_SET: 0, io.SEEK_CUR: self.pos, io.SEEK_END: self.size}[whence]
        self.pos = base + offset
        return self.pos

    def readinto(self, buffer):
        if self.pos >= self.size or len(buffer) == 0:
            return 0
        end = min(self.pos + len(buffer), self.size) - 1
        request = urllib.request.Request(self.url, headers={"Range": f"bytes={self.pos}-{end}"})
        data = urllib.request.urlopen(request).read()
        buffer[: len(data)] = data
        self.pos += len(data)
        return len(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("meshes", nargs="*", help="third-level mesh codes (default: Shibuya, Shinjuku)")
    parser.add_argument("--output", type=pathlib.Path, default=pathlib.Path("target/plateau/tokyo23"))
    args = parser.parse_args()
    meshes = args.meshes or list(DEFAULT_MESHES)
    args.output.mkdir(parents=True, exist_ok=True)

    archive = zipfile.ZipFile(io.BufferedReader(HttpRangeFile(ARCHIVE), buffer_size=1 << 20))
    names = archive.namelist()
    for mesh in meshes:
        wanted = [
            n
            for n in names
            if n.startswith(f"{ROOT}/bldg/{mesh}_bldg") or n.startswith(f"{ROOT}/tran/{mesh}_tran")
        ]
        wanted = [n for n in wanted if n.endswith(".gml")]
        if not wanted:
            sys.exit(f"mesh {mesh} not found in the archive")
        for name in wanted:
            target = args.output / pathlib.Path(name).name
            if target.exists() and target.stat().st_size == archive.getinfo(name).file_size:
                print(f"have {target}")
                continue
            with archive.open(name) as source, open(target, "wb") as sink:
                while chunk := source.read(1 << 20):
                    sink.write(chunk)
            label = DEFAULT_MESHES.get(mesh, mesh)
            print(f"wrote {target} ({target.stat().st_size / 1e6:.1f} MB, {label})")


if __name__ == "__main__":
    main()
