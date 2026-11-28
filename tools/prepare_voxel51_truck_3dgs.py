#!/usr/bin/env python3
"""Prepare the Voxel51 Tanks and Temples truck 3DGS as an outdoor scene for example 113.

Keeps every Nth Gaussian (position, DC colour, opacity, scale, rotation), fits the
parking-lot plane with RANSAC, and writes a splat manifest that puts that plane at
world y = 0 (RNE y up) at an assumed metric scale (a ~6.6 m truck: 5.5 COLMAP units
x 1.2). The robot's spawn in the Dr Johnson collision scene, world (x, z) = (-2.2, -4.0),
lands at plane coordinates (u, v) = (-5.0, -0.65), beside the truck's rear. The
printed `RNE_WAYPOINTS` drive a loop around the truck with about 1 m of clearance.

Usage:
  prepare_voxel51_truck_3dgs.py SOURCE_PLY OUT_DIR

SOURCE_PLY is the Graphdeco PLY from
https://huggingface.co/datasets/Voxel51/gaussian_splatting/resolve/main/FO_dataset/truck/point_cloud/iteration_30000/point_cloud.ply
(510,049,492 bytes). OUT_DIR gets `truck_dc_every2.ply` and `truck.rne.splat.toml`.
Neither is committed: the derivative is 58 MB.
"""

from __future__ import annotations

import argparse
import math
import pathlib

import numpy as np

STRIDE = 2
KEEP = [
    "x", "y", "z", "f_dc_0", "f_dc_1", "f_dc_2", "opacity",
    "scale_0", "scale_1", "scale_2", "rot_0", "rot_1", "rot_2", "rot_3",
]
SCALE = 1.2
SPAWN_WORLD_XZ = (-2.2, -4.0)
SPAWN_UV = (-5.0, -0.65)
ROUTE_UV = [(-5.0, -2.4), (4.0, -2.4), (4.0, 1.1), (-5.0, 1.1)]


def read_ply(path: pathlib.Path) -> tuple[list[str], np.ndarray]:
    with path.open("rb") as stream:
        header = b""
        while not header.endswith(b"end_header\n"):
            line = stream.readline()
            if not line:
                raise ValueError("no end_header")
            header += line
        lines = header.split(b"\n")
        props = [l.split()[-1].decode() for l in lines if l.startswith(b"property")]
        count = int(next(l for l in lines if l.startswith(b"element vertex")).split()[-1])
        data = np.fromfile(stream, dtype="<f4", count=count * len(props))
    return props, data.reshape(count, len(props))


def fit_ground(points: np.ndarray) -> tuple[np.ndarray, float]:
    """RANSAC plane (2 cm in COLMAP units) over the central 80 %, refined by SVD."""
    rng = np.random.default_rng(0)
    centre = np.median(points, 0)
    radius = np.linalg.norm(points - centre, axis=1)
    sample = points[radius < np.percentile(radius, 80)]
    best = (-1, None, 0.0)
    for _ in range(3000):
        tri = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        length = np.linalg.norm(normal)
        if length < 1e-9:
            continue
        normal /= length
        offset = -normal @ tri[0]
        inliers = int((np.abs(sample @ normal + offset) < 0.02).sum())
        if inliers > best[0]:
            best = (inliers, normal, offset)
    _, normal, offset = best
    plane = sample[np.abs(sample @ normal + offset) < 0.02]
    mean = plane.mean(0)
    normal = np.linalg.svd(plane - mean)[2][2]
    offset = -normal @ mean
    if np.median(sample @ normal + offset) < 0:  # up: towards most of the scene
        normal, offset = -normal, -offset
    return normal, float(offset)


def quaternion_xyzw(m: np.ndarray) -> list[float]:
    trace = m.trace()
    if trace > 0:
        s = math.sqrt(trace + 1) * 2
        return [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4]
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1 + m[i, i] - m[j, j] - m[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = s / 4
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    q[3] = (m[k, j] - m[j, k]) / s
    return q


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=pathlib.Path)
    parser.add_argument("out_dir", type=pathlib.Path)
    args = parser.parse_args()

    props, data = read_ply(args.source)
    kept = data[::STRIDE][:, [props.index(name) for name in KEEP]].astype("<f4")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    ply = args.out_dir / "truck_dc_every2.ply"
    with ply.open("wb") as stream:
        header = "ply\nformat binary_little_endian 1.0\nelement vertex %d\n" % len(kept)
        header += "".join(f"property float {name}\n" for name in KEEP) + "end_header\n"
        stream.write(header.encode())
        stream.write(kept.tobytes())

    opacity = 1 / (1 + np.exp(-kept[:, 6]))
    normal, offset = fit_ground(kept[opacity > 0.5, :3])
    e1 = np.cross(normal, [0.0, 0.0, 1.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(normal, e1)
    rotation = np.array([e1, normal, -e2])  # COLMAP -> RNE world (x, y up, z)
    translation = [
        SPAWN_WORLD_XZ[0] - SCALE * SPAWN_UV[0],
        SCALE * offset,
        SPAWN_WORLD_XZ[1] + SCALE * SPAWN_UV[1],
    ]
    q = quaternion_xyzw(rotation)
    manifest = args.out_dir / "truck.rne.splat.toml"
    manifest.write_text(
        'kind = "rne_gaussian_splat_environment"\n'
        "schema_version = 1\n"
        'environment_id = "voxel51.truck.outdoor.v1"\n'
        f'ply_path = "{ply.name}"\n'
        "standin = false\n"
        f"translation_m = [{translation[0]:.6f}, {translation[1]:.6f}, {translation[2]:.6f}]\n"
        "rotation_y_rad = 0.0\n"
        f"rotation_xyzw = [{q[0]:.9f}, {q[1]:.9f}, {q[2]:.9f}, {q[3]:.9f}]\n"
        f"scale = {SCALE}\n"
        'renderer_identity = "rne.gaussian_splat.v1"\n'
        'coordinate_note = "Voxel51 Tanks and Temples truck (every 2nd gaussian, DC colour). '
        'Parking-lot plane (RANSAC) at world y=0; scale assumes a ~6.6 m truck."\n'
    )

    def world(u: float, v: float) -> tuple[float, float]:
        return (
            SPAWN_WORLD_XZ[0] + SCALE * (u - SPAWN_UV[0]),
            SPAWN_WORLD_XZ[1] - SCALE * (v - SPAWN_UV[1]),
        )

    route = ";".join("%.3f,%.3f" % world(u, v) for u, v in ROUTE_UV)
    print(f"wrote {ply} ({len(kept)} gaussians) and {manifest}")
    print(f'RNE_SPLAT_MANIFEST={manifest} RNE_WAYPOINTS="{route}"')


if __name__ == "__main__":
    main()
