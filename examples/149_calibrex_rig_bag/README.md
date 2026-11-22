# 149 — Calibrex rig bag

A multi-sensor rig simulated in RNE and recorded as a ROS 2 bag, so
[Calibrex](https://github.com/rsasaki0109/Calibrex) can check its sensor
calibration as it would on a real robot's recording.

```bash
cargo run --release -p calibrex_rig_bag --example 149_calibrex_rig_bag -- --out target/calibrex_rig
calibrex check target/calibrex_rig/rig.mcap --tf target/calibrex_rig/good.yaml          # every judged pair passes
calibrex check target/calibrex_rig/rig.mcap --tf target/calibrex_rig/turned_lidar2.yaml # lidar2's pairs fail
```

- `--duration-s` sets the recording length (default 40 s).
- `--sensors` picks the sensors (default `lidar,imu,lidar2,camera`; the primary
  LiDAR is always recorded). `--sensors imu` writes a small IMU + LiDAR bag.
- `--rotation-speed` multiplies the frequencies of the rig's rotation (default
  1; the amplitude and the translation are unchanged). At 2 the IMU's lever arm
  is observable, see [The lever arm](#the-lever-arm).
- `--motion turn` turns the rig about the camera's optical centre instead of
  carrying it by hand, `--lens K1,K2,P1,P2[,K3]` gives the camera lens
  distortion (OpenCV / ROS `plumb_bob` coefficients), and `--camera-info
  true|zero|none` publishes the true `CameraInfo`, an uncalibrated one (all-zero
  `K`) or none. See [Camera intrinsics from the bag](#camera-intrinsics-from-the-bag).
- `--board COLSxROWSxSQUARE_M` stands a checkerboard in the room and `--motion
  board` holds the rig still at 28 poses in front of it; `--sensors camera2` adds a
  second camera 10 cm to the right; `--readout-ms` reads the cameras out row by row
  (rolling shutter). See [A checkerboard](#a-checkerboard) and
  [Rolling shutter](#rolling-shutter).
- Install Calibrex from its repository
  (`pip install "calibrex[opencv] @ git+https://github.com/rsasaki0109/Calibrex.git"`;
  OpenCV is needed for camera-imu). It reads the bag with its own pure-Python
  MCAP reader, no ROS install needed.

## What is simulated

- **Scene**: a 12 m × 10 m × 3 m room of Rapier cuboid colliders, with two
  slanted panels, two pillars, a crate and a ramp so scan registration is
  constrained in every direction, and a ring of 36 thin floor-to-ceiling posts
  2–4 m from the rig, two in three leaning 35° sideways. Wherever the camera
  looks, several posts stand in front of a wall, so every LiDAR beam has depth
  discontinuities in the image; the leaning ones give slanted edges, which a
  vertical shift of the image moves sideways.
- **Motion**: a hand-held rig that turns about all three axes (up to about
  30°, 26° and 57°) and translates by about half a metre, so every rotation
  axis of the mountings is observable. The rig's path keeps every sensor at
  least 0.4 m from the posts (a test checks it at both rotation speeds).
- **LiDARs**: RNE's `LidarSpec`, 16 channels over ±15°, 360 columns per scan at
  10 Hz, 1 cm range noise. Each scan is ray-cast in 20 sub-sweeps between
  exactly sampled rig poses, so it is motion-distorted like a real spinning
  LiDAR; every point is expressed in the sensor frame at its own firing time and
  carries that time (`time`, seconds after the header stamp). `lidar` is the rig
  frame; `lidar2` is mounted at (0.30, −0.25, 0.10) m, turned 8°/−15°/20°.
- **IMU**: RNE's mounted IMU model (`ImuMount` with a 2°/−3°/92° rotation and a
  (0.10, −0.06, 0.15) m lever arm), sampled at 200 Hz from the rig's rigid-body
  kinematics, with white noise and turn-on biases on gyro and accelerometer.
- **Camera**: RNE's camera model (`sample_camera_rgbd_keyed`, global shutter,
  read noise) at 10 Hz, 480 × 360, 70° vertical field of view, looking along
  the LiDAR's x axis and tilted 4°/−2.5°/1.5°, 0.12 m ahead. There is no GPU in
  the loop: the example's `ColliderRenderer` implements `RenderBackend` by
  ray-casting the room's cuboid colliders, read from the ECS world. Each face
  has its own gray level, varied by ±10 % in 15 cm cells, so geometric edges
  are the strongest image edges and the cells give corners to track. A test
  holds the rig still and projects LiDAR points through the published
  `T_lidar_camera` and `CameraInfo`: they land on the rendered depth (median
  difference under 5 mm).
- **Frames**: ROS conventions in the bag (x forward, z up; the camera frame is
  the optical frame, z forward and y down) although the RNE world is y-up and
  RNE cameras look down their local −z. `/tf_static` carries the true
  mountings of `imu`, `lidar2` and `camera` in `lidar`.

## Output

| File | Contents |
|---|---|
| `rig.mcap` | ROS 2 bag (MCAP storage, CDR): `/imu/data` (`sensor_msgs/Imu`), `/lidar/points` and `/lidar2/points` (`sensor_msgs/PointCloud2`, `x y z intensity ring time`), `/camera/image_raw` (`mono8`) and `/camera/camera_info`, `/tf_static`; only the recorded sensors' topics are declared |
| `good.yaml` | the true mountings as a Calibrex `--tf` frames file |
| `turned_imu.yaml` (also `turned.yaml`) | the IMU turned 3° about the LiDAR z axis |
| `shifted_imu.yaml` | the IMU moved 5 cm along the LiDAR x axis |
| `turned_lidar2.yaml` | `lidar2` turned 3° about its own z axis |
| `turned_camera.yaml` | the camera turned 3° about its optical x axis |

The bag is written by `rne_adapter_ros2`'s `cdr` encoders and `mcap` writer,
which need no ROS 2 installation.

## Results

Checked with Calibrex at commit `5ce60ac0` on a 4-core container. Tolerances are
0.5° per rotation axis and 2 cm per translation axis unless shown.

### All sensors, 90 s

A 90 s bag (393 MB: 18,000 IMU messages, 899 scans per LiDAR, 900 images)
takes 7 min 7 s to check against `good.yaml`; the other candidates reuse
Calibrex's cache and take about 40 s each.

| Pair | `good.yaml` | Largest judged error |
|---|---|---|
| imu–lidar | **pass** (roll, pitch, yaw) | 0.020° |
| imu–lidar2 | **pass** (roll, pitch, yaw) | 0.045° |
| lidar–lidar | **pass** (all six axes) | 0.037°, 1.0 mm |
| camera–lidar | **pass** (roll) | 0.10° (tolerance 0.63°) |
| camera–lidar2 | **pass** (roll, pitch, yaw) | 0.25° (tolerance 0.56°) |
| camera–imu | inconclusive | — |
| camera-focal | inconclusive | — |

Each turned candidate fails exactly the pairs of the sensor it turns, and every
other pair passes as with `good.yaml`:

| Candidate | Failing pairs |
|---|---|
| `turned_imu.yaml` | imu–lidar (yaw 3.01°), imu–lidar2 (yaw 2.85°) |
| `turned_lidar2.yaml` | imu–lidar2 (yaw 3.02°), lidar–lidar (yaw 2.86°), camera–lidar2 (pitch 2.9°) |
| `turned_camera.yaml` | camera–lidar (roll 3.0°), camera–lidar2 (roll 3.4°) |

A 3° turn about one sensor axis shows up on the pair's own axes; the turn of
`lidar2` about its z axis reads as pitch in the camera's optical frame.

### IMU + LiDAR, 40 s

`--sensors imu` (56 MB): `good.yaml` passes imu–lidar (roll 0.054°, pitch
0.011°, yaw 0.009°) and `turned.yaml` fails on yaw (2.99°). The first check took
86 s; the second reused Calibrex's cache and took 1.3 s.

### The camera pairs

camera–lidar aligns LiDAR depth discontinuities with image edges, so it needs
many of them: at least 150 edge points of a frame must fall in the image. The
room alone gave 35–105 per frame (inconclusive); with the posts it gives
enough on most frames, and their slanted silhouettes constrain the camera's
roll and yaw as well as its pitch. On a 20 s bag both camera–lidar pairs judge
all three rotation axes.

camera–imu is inconclusive: Calibrex's held-out control does not detect a 1°
shift of its roll, and its pitch and yaw are not constrained to its bound. Its
yaw estimate is 1.05° off. That error is on Calibrex's side, not the camera's:
the estimator tracks image features and, where the essential matrix is
ambiguous (most frame pairs of this 480 × 360 camera, whose parallax is about
half a pixel), fits a pure rotation, which takes up the rig's translation. With
the translation switched off the same check passes with errors under 0.02°, and
neither an ideal IMU nor a camera at the rig's rotation centre changes the
1.03° error. On a hand-held rig the translation follows the rotation (here
their frequencies beat over about a minute), so the error does not average out.

### Camera intrinsics from the bag

A robot's camera often has no calibrated `CameraInfo`. Without one Calibrex
skips the camera pairs (`missing_intrinsics`); with `--motion turn` the bag is
also what its targetless intrinsics calibration needs:

```bash
cargo run --release -p calibrex_rig_bag --example 149_calibrex_rig_bag -- \
    --out target/calibrex_intrinsics --duration-s 40 --sensors imu,camera \
    --motion turn --lens -0.18,0.04,0.0008,-0.0006 --camera-info none
calibrex intrinsics rotate --bag target/calibrex_intrinsics/rig.mcap \
    --topic /camera/image_raw --seconds 20 --output target/calibrex_intrinsics/intrinsics_camchain.yaml
calibrex check target/calibrex_intrinsics/rig.mcap --tf target/calibrex_intrinsics/good.yaml \
    --intrinsics target/calibrex_intrinsics/intrinsics_camchain.yaml
```

From the first 20 s (1 min 45 s on a 4-core container), against the true
camera (f = 257.07 px, principal point 239.5, 179.5, k1 −0.18, k2 0.04):

| | fx | fy | cx | cy | k1 | k2 |
|---|---|---|---|---|---|---|
| estimated | 257.42 | 258.64 | 239.67 | 179.75 | −0.1786 | 0.0392 |
| error | +0.13 % | +0.61 % | +0.17 px | +0.25 px | +0.0014 | −0.0008 |

Its transfer RMS is 1.13 px (Calibrex warns above 1 px): the room's flat
15 cm cells are rendered one ray per pixel. The check with those intrinsics, on
the 40 s bag:

| Candidate | imu–lidar | camera–imu | camera–lidar |
|---|---|---|---|
| `good.yaml` | **pass** (within 0.053°) | **pass** (roll 0.063°, pitch 0.0006°, yaw 0.003°) | **pass** (within 0.25°) |
| `turned_imu.yaml` | **fail**: yaw 2.95° | **fail**: pitch 2.99° | pass |
| `turned_camera.yaml` | pass | **fail**: roll 3.06° | **fail**: roll 2.9° |

camera–imu, inconclusive on the hand-held bag, passes here: turning about the
camera leaves no translation for its pure-rotation fits to absorb. camera-focal
is inconclusive (its held-out check fails) but reads an fy scale of 0.9938, the
estimated intrinsics' 0.6 % long fy seen from the IMU.

### A checkerboard

`--board 7x5x0.08` stands a 7 × 5 (inner corners), 8 cm checkerboard 1.6 m ahead of
the rig, with one white square of margin, as a 1 cm thick collider: the LiDARs see
it and the camera images its pattern, with 3 × 3 jittered rays per board pixel
(a regular grid quantized edges parallel to the pixel rows and biased the corners
of a board seen square on by 2 %). `--motion board` holds the camera still for
1.5 s at each of 28 poses (the board in every cell of a 5 × 4 image grid, tilted
20–32° four ways, nearer and farther), moving 1 s between them:

```bash
cargo run --release -p calibrex_rig_bag --example 149_calibrex_rig_bag -- \
    --out target/calibrex_board --duration-s 70 --sensors imu,camera,camera2 \
    --motion board --board 7x5x0.08 --lens -0.18,0.04,0.0008,-0.0006 --camera-info none
calibrex intrinsics guide --board 7x5x0.08 --bag target/calibrex_board/rig.mcap \
    --topic /camera/image_raw --right-topic /camera2/image_raw --no-window \
    --output target/calibrex_board/intrinsics_camchain.yaml
calibrex check target/calibrex_board/rig.mcap --tf target/calibrex_board/good.yaml \
    --intrinsics target/calibrex_board/intrinsics_camchain.yaml --board 7x5x0.08
```

The guide's stereo calibration (16 and 13 views; the tour ends before its
precision goals) gave both focal lengths within 0.34 %, the principal points within
0.4 px, k1 within 0.0012, and the baseline 0.0996 ± 0.0008 m (true 0.10 m). With
those intrinsics and the board:

| Candidate | camera–lidar | camera2–lidar | camera–camera2 | camera–imu |
|---|---|---|---|---|
| `good.yaml` | **pass**, all six axes (within 0.073°, 5 mm) | **pass**, all six axes (within 0.081°, 2.4 mm) | **pass** (within 0.044°) | pass (roll 0.17°) |
| `turned_camera.yaml` | **fail**: roll 2.93° | pass | pass (roll 0.2°) | **fail**: roll 2.84° |

camera–lidar judges the translation only with the board (`--board`); the
holds leave camera–imu and imu–lidar only roll to judge.

### Rolling shutter

`--readout-ms 30` reads each frame out over 30 ms in 24 bands of 15 rows, each
rendered from the pose at its own time, the middle row at the stamp. On a 20 s
`--motion turn` bag, `calibrex intrinsics rotate --rolling-shutter` estimated a
readout of 25.9 ± 0.5 ms (the bands step the continuous readout Calibrex models)
and moved the principal point from 0.9 px off to within 0.1 px, the focal lengths
staying within 0.5 %. The turn is slow enough here that the readout matters
little; Calibrex's synthetic test, turned faster, shows the 1.5 % it can cost.

### The lever arm

At the default speed imu–lidar does not judge the IMU's lever arm: its data-only
std is 1.1–1.8 cm against Calibrex's 1 cm bound and does not shrink with a longer
recording (4 min gave the same), and Calibrex's segment-duration sensitivity
(1.0–3.3 cm) is above the bound too. The lever arm shows in the accelerometer
through the angular velocity and acceleration, so `--rotation-speed 2`, which
doubles the first and quadruples the second, makes it observable. On a 90 s
`--sensors imu --rotation-speed 2` bag (125 MB):

| Candidate | imu–lidar |
|---|---|
| `good.yaml` | **pass** (all six axes): rotation within 0.026°, lever arm within 5 mm |
| `turned.yaml` | **fail**: yaw 3.01° |
| `shifted_imu.yaml` | **fail**: x 4.7 cm; the rotation passes |

### The reported time offset

Calibrex also reports an IMU–LiDAR time offset although both sensors are
stamped from the same simulation clock (reported, not judged): −3.3 ms on the
40 s bag, −3.1 and −3.6 ms for the two LiDARs on the 90 s bag (the room without
posts gave −1.9 to −2.2 ms), and −1.0 ms at `--rotation-speed 2`. It is not RNE's
sensors: on true poses Calibrex's solver recovers the offset to the
microsecond, and its own noise-free synthetic bag shows a bias of −1.1 to
−1.7 ms with or without translation. Calibrex fits the offset jointly with the
rotation and the gyro bias, and the scan-to-scan odometry's rotation errors,
which follow the motion and depend on the scene, move it the same way in every
window, beyond its data-only std (0.2–0.6 ms). Calibrex now prints that std
and says so next to the offset. Calibrex's
[simulated-rig tutorial](https://github.com/rsasaki0109/Calibrex/blob/main/docs/tutorials/simulated_rig_rne.md)
has the experiments.

This is simulated data: it shows that RNE's sensors produce a recording a
calibration tool reads and judges like a real one, not how accurate Calibrex is
on hardware.
