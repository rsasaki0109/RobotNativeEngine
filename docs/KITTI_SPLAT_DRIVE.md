# KITTI splat drive and visual-localization closed loop

Examples 160 and 161 (`examples/160_kitti_splat_drive`, package `kitti_splat_drive`) drive an
Ackermann car (`rne_robot::ackermann_kinematics`, pure pursuit) through a 3D Gaussian Splatting
reconstruction of KITTI raw drive 0018. The splat is trained with visloc-rs `gsplat_train` on
camera poses from LiDAR SLAM (lidar_slam), so it is metric and gravity-aligned; its manifest
rotates the z-up SLAM frame -90 deg about X into RNE's y-up frame. The car follows the road
height of the SLAM path.

- **160** renders the car's front camera (KITTI `image_02` pinhole, principal point at the
  centre) and a chase view with the car drawn as a box.
- **161** steers on the car's *estimated* pose: dead reckoning from wheel speed and steering
  (2 % scale error, 0.003 rad/s yaw-rate bias) between visual fixes; every 0.5 s the front
  camera is rendered at the true pose (`--manifest`: RNE's renderer; otherwise visloc-rs
  `gsplat_render_server`) and localized by visloc-rs `basalt_euroc_localize_demo` against a
  map built from splat renders at the SLAM camera poses. Fixes more than 3 m from the
  estimate are rejected. The car starts at a known pose.

Inputs (PLY, path CSV, visloc map) are not in the repository; see lidar_slam
`tools/rne_kitti_splat_drive`.

## Splat colour pass fixes

- `rne_render_3dgs` draws the splat through a non-sRGB view of the hybrid target: splat colours
  are display-referred sRGB and were encoded a second time (washed out).
- `third_party/wgpu-3dgs-viewer` (vendored 0.2.0, see its `RNE_PATCH.md`): the shader used 2f as
  the focal length for the 2D covariance and an `exp(-|q|^2)` falloff, drawing every splat
  sqrt(2) too wide. It now follows the Inria EWA splatting (focal f, +0.3 px^2 low-pass, 3-sigma
  quads, `exp(-0.5 q^T Sigma^-1 q)`). Synthetic isotropic splats at 10 m measure 1.52 / 3.58 /
  7.17 px against 1.52 / 3.58 / 7.09 px expected (upstream: 2.04 px for the first).

## Results (KITTI 0018, first 290 m, 7 m/s, CPU lavapipe)

| | visual fixes | cross-track mean / max | estimate error mean / max |
|---|---|---|---|
| odometry only | - | 5.55 / 17.17 m | 5.97 / 17.37 m |
| closed loop, visloc-rs renderer | 65 / 86 | 0.07 / 0.71 m | 0.13 / 0.82 m |
| closed loop, RNE renderer (sqrt(2) workaround) | 52 / 86 | 0.10 / 0.80 m | 0.16 / 0.98 m |

Open-loop localization of 172 RNE front-camera frames: none before the footprint fix,
115 (median 5.2 cm) with a 1/sqrt(2) size workaround, 128 (median 6.2 cm) with the shader fix.
