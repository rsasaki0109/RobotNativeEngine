"""Turn a textured car mesh (glTF/GLB, OBJ, ...) into an actor splat PLY with baked lighting.

Cars cut out of a street splat were seen only from a distance and blur close to a chase
camera. A CG car sampled into small flat Gaussians stays sharp. Each mesh part is sampled
evenly (about one Gaussian per SPACING^2 of surface), every sample becomes a flat disc along
the surface, and a fixed sun, sky light, simple car-paint reflections and a soft contact shadow
are baked into the colours (actors have no view-dependent colour).

Materials are chosen from the material name: paint, glass/window/mirror, tyre/tire,
rim/disc/chrome, brake/tail light, signal, head light, licence plate (textures dropped, so
logos on plates do not show). Anything else keeps its base colour factor. Parts whose mesh
name starts with a --skip prefix (default: Interior, Engine) are left out; the glass is
drawn opaque and dark instead.

The actor frame: +x forward, +y left, +z up, origin at the bottom centre (as cut_actor.py).
The input is taken as glTF axes (+y up); the forward axis is the side with the head lights,
or --forward.

  mesh_to_actor.py IN.glb OUT.ply [--spacing 0.02] [--paint R,G,B] [--paint-material NAME]
                   [--skip Interior,Engine] [--forward +z|-z|+x|-x]

Needs numpy, scipy and trimesh (pip install trimesh).
"""
import argparse

import numpy as np
import trimesh
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

ap = argparse.ArgumentParser()
ap.add_argument('src')
ap.add_argument('out')
ap.add_argument('--spacing', type=float, default=0.02)
ap.add_argument('--paint', default='0.08,0.20,0.50', help='body colour, linear 0..1')
ap.add_argument('--paint-material', default='Paint 1', help='material name prefix of the body paint')
ap.add_argument('--skip', default='Interior,Engine')
ap.add_argument('--forward', default='auto')
args = ap.parse_args()
SP = args.spacing
PAINT = np.array([float(v) for v in args.paint.split(',')])
SKIP = tuple(s for s in args.skip.split(',') if s)

FIXED = [  # (name keywords, colour, kind)
    (('glass', 'window'), (0.04, 0.05, 0.06), 'glass'),
    (('mirror',), (0.55, 0.58, 0.62), 'glass'),
    (('licen', 'plate'), (0.90, 0.90, 0.86), 'plate'),
    (('brakelight', 'taillight', 'brake light', 'tail light'), (0.70, 0.03, 0.03), 'lamp'),
    (('signal', 'indicator'), (0.80, 0.35, 0.02), 'lamp'),
    (('headlight', 'head light'), (0.92, 0.92, 0.88), 'lamp'),
    (('rim', 'disc', 'chrome'), (0.45, 0.46, 0.48), 'metal'),
    (('tire', 'tyre'), (0.035, 0.035, 0.04), 'tyre'),
]


def material(mname, factor):
    low = mname.lower()
    if mname.startswith(args.paint_material):
        return PAINT, 'paint'
    if low.startswith('paint'):
        return np.array([0.03, 0.03, 0.035]), 'trim'
    for keys, colour, kind in FIXED:
        if any(k in low for k in keys):
            return np.array(colour), kind
    base = np.array(factor[:3]) / 255.0 if factor is not None else np.array([0.5, 0.5, 0.5])
    return base, 'other'


scene = trimesh.load(args.src, force='scene')
pts, nrm, col, kind = [], [], [], []
for node in scene.graph.nodes_geometry:
    T, gname = scene.graph[node]
    if gname.startswith(SKIP):
        continue
    g = scene.geometry[gname]
    mat = getattr(g.visual, 'material', None)
    base, k = material(getattr(mat, 'name', None) or '', getattr(mat, 'baseColorFactor', None))
    m = g.copy()
    m.apply_transform(T)
    cnt = max(int(m.area / (SP * SP) * 1.3), 20)
    p, fi = trimesh.sample.sample_surface_even(m, cnt, radius=SP * 0.55, seed=1)
    pts.append(p); nrm.append(m.face_normals[fi]); col.append(np.tile(base, (len(p), 1))); kind += [k] * len(p)
p = np.concatenate(pts); n = np.concatenate(nrm); c = np.concatenate(col); kind = np.array(kind)

lo, hi = p.min(0), p.max(0)
ctr = (lo + hi) / 2
if args.forward == 'auto':
    head = (kind == 'lamp') & (c[:, 0] > 0.9)
    axis = 2 if np.ptp(p[:, 2]) >= np.ptp(p[:, 0]) else 0
    sign = 1.0 if not head.any() or p[head, axis].mean() >= ctr[axis] else -1.0
else:
    axis, sign = {'z': 2, 'x': 0}[args.forward[1]], (1.0 if args.forward[0] == '+' else -1.0)
fwd = np.zeros(3); fwd[axis] = sign
up = np.array([0.0, 1.0, 0.0])
left = np.cross(up, fwd)
M = np.stack([fwd, left, up])
p = (p - [ctr[0], lo[1], ctr[2]]) @ M.T
n = n @ M.T
flip = (n * (p - [0, 0, 0.55])).sum(1) < 0  # double-sided panels: point normals outward
n[flip] *= -1

sun = np.array([0.4, 0.35, 0.85]); sun /= np.linalg.norm(sun)
lam = np.clip(n @ sun, 0, 1)
upw = np.clip(n[:, 2], 0, 1)
ao = np.clip(0.5 + 0.7 * p[:, 2], 0.5, 1.0)
rgb = c * ((0.45 + 0.25 * upw + 0.55 * lam) * ao)[:, None]
paint = kind == 'paint'
rgb[paint] = rgb[paint] * (0.75 + 0.15 * upw[paint, None]) + 0.16 * upw[paint, None] * np.array([0.85, 0.9, 1.0])
side = paint & (np.abs(n[:, 2]) < 0.6)
horizon = 1 / (1 + np.exp(-(p[:, 2] - 0.62) / 0.03))
rgb[side] = rgb[side] * (0.70 + 0.45 * horizon[side, None])
half = (sun + [0, 0, 1]) / np.linalg.norm(sun + [0, 0, 1])
spec = np.clip(n @ half, 0, 1) ** 30
shiny = (kind == 'paint') | (kind == 'glass') | (kind == 'trim')
rgb[shiny] += 0.35 * spec[shiny, None]
glass = kind == 'glass'
rgb[glass] += 0.12 * upw[glass, None] * np.array([0.8, 0.9, 1.0])
rgb[kind == 'lamp'] = c[kind == 'lamp'] * 0.95
rgb = np.clip(rgb, 0, 1)
for kk in np.unique(kind):  # smooth shading noise from flipped normals, per material kind
    idx = np.nonzero(kind == kk)[0]
    if len(idx) >= 10:
        _, nb = cKDTree(p[idx]).query(p[idx], k=min(12, len(idx)))
        rgb[idx] = np.median(rgb[idx][nb], axis=1)

# Soft contact shadow under the car.
L, Wd = np.ptp(p[:, 0]), np.ptp(p[:, 1]) * 0.85
gx, gy = np.meshgrid(np.arange(-L / 2 - 0.2, L / 2 + 0.2, 0.06), np.arange(-Wd / 2 - 0.2, Wd / 2 + 0.2, 0.06))
gp = np.c_[gx.ravel(), gy.ravel(), np.full(gx.size, 0.012)]
fall = np.clip(1 - (np.maximum(np.abs(gp[:, 0]) - L / 2 + 0.3, 0) / 0.5) ** 2
               - (np.maximum(np.abs(gp[:, 1]) - Wd / 2 + 0.2, 0) / 0.4) ** 2, 0, 1)
gp, fall = gp[fall > 0.02], fall[fall > 0.02]

C0 = 0.28209479177387814  # degree-0 spherical harmonic
allp = np.r_[p, gp]
alln = np.r_[n, np.tile([0, 0, 1.0], (len(gp), 1))]
a = np.zeros((len(allp), 62), np.float32)
a[:, 0:3] = allp
a[:, 3:6] = alln
a[:, 6:9] = (np.r_[rgb, np.zeros((len(gp), 3))] - 0.5) / C0
opacity = np.r_[np.full(len(p), 0.995), 0.55 * fall]
a[:, 54] = np.log(opacity / (1 - opacity))
a[:, 55:58] = np.log(np.r_[np.tile([SP * 0.85, SP * 0.85, 0.002], (len(p), 1)),
                           np.tile([0.05, 0.05, 0.002], (len(gp), 1))])
t1 = np.cross(alln, [0.0, 0.0, 1.0])
bad = np.linalg.norm(t1, axis=1) < 1e-3
t1[bad] = np.cross(alln[bad], [1.0, 0.0, 0.0])
t1 /= np.linalg.norm(t1, axis=1, keepdims=True)
q = R.from_matrix(np.stack([t1, np.cross(alln, t1), alln], 2)).as_quat()
a[:, 58], a[:, 59], a[:, 60], a[:, 61] = q[:, 3], q[:, 0], q[:, 1], q[:, 2]  # PLY stores w, x, y, z
props = (['x', 'y', 'z', 'nx', 'ny', 'nz', 'f_dc_0', 'f_dc_1', 'f_dc_2'] + [f'f_rest_{i}' for i in range(45)]
         + ['opacity', 'scale_0', 'scale_1', 'scale_2', 'rot_0', 'rot_1', 'rot_2', 'rot_3'])
hdr = ('ply\nformat binary_little_endian 1.0\nelement vertex %d\n' % len(a)
       + ''.join(f'property float {k}\n' for k in props) + 'end_header\n')
with open(args.out, 'wb') as fh:
    fh.write(hdr.encode())
    a.tofile(fh)
print(f'{len(a)} Gaussians ({len(p)} surface, {len(gp)} shadow), size L W H {np.ptp(p, 0).round(2).tolist()} -> {args.out}')
