# RNE wgpu-3dgs-viewer EWA footprint patch

Based on the crates.io wgpu-3dgs-viewer 0.2.0 package (MIT, LICENSE retained).
`RNE_UPSTREAM_SHA256.json` records the untouched upstream files; the package's
`Cargo.lock` is not vendored (a patched dependency's lockfile is unused).
`RNE.patch` is the full diff against upstream. `Cargo.toml` adds an empty
workspace so the package stays outside the engine workspace (it is listed in
the root `exclude` and wired in through `[patch.crates-io]`).

## Splat footprint (`src/shader/render.wgsl`)

Upstream 0.2.0 projected the 3D covariance with a pixel focal length of
`proj[i][i] * size`, twice the true `proj[i][i] * size / 2`, so the 2D
covariance was 4x too large. It added a 0.1 low-pass in those doubled units
(about 0.025 px^2) and drew `exp(-q^2)` with `q` measured in `sqrt(lambda)` of
that covariance. Net effect: every splat was drawn sqrt(2) wider than the
trained Gaussian. RNE previously compensated by setting the per-splat display
size to 1/sqrt(2), which left the low-pass and extent wrong.

The patched vertex/fragment shader follows the reference rasterizer
(Kerbl et al. 2023, diff-gaussian-rasterization):

- focal length `f = proj[i][i] * size / 2` px, with the reference
  `1.3 * tan(fov / 2)` clamp of view-space x/z and y/z in the Jacobian;
- `+0.3 px^2` low-pass on the 2D covariance diagonal (no Mip-Splatting
  opacity compensation);
- quad half-axes of 3 sigma along the covariance eigenvectors; the
  interpolated `quad_offset` is the offset in sigma units of each axis, so
  `|q|^2 = d^T Sigma^-1 d`. Extents beyond 1024 px are truncated rather than
  rescaled;
- `alpha = min(0.99, opacity * exp(-0.5 |q|^2))`, discarded below 1/255.

Ellipse display mode shares the 3 sigma quad. Point mode is unchanged. The
viewer's `GaussianTransform::size` still scales sigma and defaults to 1.

Verification: a synthetic PLY of isotropic Gaussians (scales 0.02/0.05/0.10 m
at 10 m, f = 707.09 px) renders with measured sigmas matching the reference
`sqrt((f s / z)^2 + 0.3)` = 1.52/3.58/7.09 px.
