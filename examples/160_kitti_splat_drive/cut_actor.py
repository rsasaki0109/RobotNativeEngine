"""Cut the Gaussians inside an oriented box out of a splat PLY into an actor PLY.

The actor frame: origin at the box's bottom centre, +x forward, +y left, +z up (metres).
  cut_actor.py SRC.ply OUT.ply fx fy along_min along_max lat_min lat_max z_min z_max [max_scale_m]
               [--largest-cluster RADIUS_M]
--largest-cluster keeps only the largest group of Gaussians whose centres are linked within
RADIUS_M, dropping bins, poles or kerb clutter caught in the box.
(fx, fy) is the forward direction in the source frame's xy plane; lateral is its left normal.
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R

args = sys.argv[1:]
cluster_radius = None
if '--largest-cluster' in args:
    k = args.index('--largest-cluster')
    cluster_radius = float(args[k + 1])
    del args[k:k + 2]
src, out = args[0], args[1]
fx, fy, a0, a1, l0, l1, z0, z1 = map(float, args[2:10])
max_scale = float(args[10]) if len(args) > 10 else 0.4
with open(src, 'rb') as f:
    hdr = b''
    while True:
        line = f.readline(); hdr += line
        if line.strip() == b'end_header':
            break
    props = [l.split()[-1].decode() for l in hdr.split(b'\n') if l.startswith(b'property')]
    n = int([l for l in hdr.split(b'\n') if l.startswith(b'element vertex')][0].split()[-1])
    a = np.fromfile(f, dtype=np.float32, count=n * len(props)).reshape(n, len(props))
ix = {p: i for i, p in enumerate(props)}
fwd = np.array([fx, fy, 0.0]); fwd /= np.linalg.norm(fwd)
up = np.array([0, 0, 1.0]); left = np.cross(up, fwd)
pos = a[:, [ix['x'], ix['y'], ix['z']]].astype(np.float64)
s, t, z = pos @ fwd, pos @ left, pos[:, 2]
scale = np.exp(a[:, [ix['scale_0'], ix['scale_1'], ix['scale_2']]]).max(1)
keep = (s > a0) & (s < a1) & (t > l0) & (t < l1) & (z > z0) & (z < z1) & (scale < max_scale)
if cluster_radius:
    from scipy.sparse.csgraph import connected_components
    from scipy.spatial import cKDTree
    idx = np.nonzero(keep)[0]
    graph = cKDTree(pos[idx]).sparse_distance_matrix(cKDTree(pos[idx]), cluster_radius)
    _, labels = connected_components(graph, directed=False)
    biggest = np.bincount(labels).argmax()
    keep = np.zeros(n, bool)
    keep[idx[labels == biggest]] = True
b = a[keep].copy()
origin = np.array([(a0 + a1) / 2, (l0 + l1) / 2, z0])
local = np.c_[s[keep] - origin[0], t[keep] - origin[1], z[keep] - origin[2]]
b[:, ix['x']], b[:, ix['y']], b[:, ix['z']] = local.T
# Rotate each Gaussian's orientation into the actor frame (world -> actor is the transpose).
world_from_actor = np.stack([fwd, left, up], 1)
q = b[:, [ix['rot_1'], ix['rot_2'], ix['rot_3'], ix['rot_0']]]  # PLY stores w, x, y, z
rot = R.from_matrix(world_from_actor.T) * R.from_quat(q)
qq = rot.as_quat()
b[:, ix['rot_0']], b[:, ix['rot_1']], b[:, ix['rot_2']], b[:, ix['rot_3']] = qq[:, 3], qq[:, 0], qq[:, 1], qq[:, 2]
hdr = hdr.replace(b'element vertex %d' % n, b'element vertex %d' % len(b))
with open(out, 'wb') as f:
    f.write(hdr); b.tofile(f)
print(f'{len(b)} of {n} Gaussians -> {out}; box origin (source frame) {origin.tolist()}')
