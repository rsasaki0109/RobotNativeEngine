# 149 — Calibrex rig bag

A multi-sensor rig simulated in RNE and recorded as a ROS 2 bag, so
[Calibrex](https://github.com/rsasaki0109/Calibrex) can check its sensor
calibration as it would on a real robot's recording.

```bash
cargo run --release -p calibrex_rig_bag --example 149_calibrex_rig_bag -- --out target/calibrex_rig
calibrex check target/calibrex_rig/rig.mcap --tf target/calibrex_rig/good.yaml          # IMU and LiDAR pairs pass
calibrex check target/calibrex_rig/rig.mcap --tf target/calibrex_rig/turned_lidar2.yaml # lidar2's pairs fail
```

- `--duration-s` sets the recording length (default 40 s; the camera pairs
  need 60 s or more, see below).
- `--sensors` picks the sensors (default `lidar,imu,lidar2,camera`; the primary
  LiDAR is always recorded). `--sensors imu` writes the small IMU + LiDAR bag
  that Calibrex's own CI uses.
- Install Calibrex from its repository
  (`pip install "calibrex[opencv] @ git+https://github.com/rsasaki0109/Calibrex.git"`;
  OpenCV is needed for the camera pairs). It reads the bag with its own
  pure-Python MCAP reader, no ROS install needed.

## What is simulated

- **Scene**: a 12 m × 10 m × 3 m room of Rapier cuboid colliders, with two
  slanted panels, two pillars, a crate and a ramp so scan registration is
  constrained in every direction.
- **Motion**: a hand-held rig that turns about all three axes (up to about
  30°, 26° and 57°) and translates by about half a metre, so every rotation
  axis of the mountings is observable.
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
  has its own gray level, varied by low-contrast 15 cm cells, so geometric edges
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
| `turned_lidar2.yaml` | `lidar2` turned 3° about its own z axis |
| `turned_camera.yaml` | the camera turned 3° about its optical x axis |

The bag is written by `rne_adapter_ros2`'s `cdr` encoders and `mcap` writer,
which need no ROS 2 installation.

## Results

Checked with Calibrex at commit `c8ac18dd` on a 4-core container.

### IMU + LiDAR, 40 s

| Candidate | Verdict | roll | pitch | yaw |
|---|---|---|---|---|
| `good.yaml` | **pass** (roll, pitch, yaw) | 0.002° | 0.006° | 0.009° |
| `turned.yaml` | **fail** | 0.002° | 0.006° | **2.99°** |

Tolerance 0.5° per axis. The first check took 69 s; the second reused
Calibrex's cache and took 1.5 s. The lever arm (x, y, z) was reported but not
judged: Calibrex's held-out control did not detect a 2 cm error on x, and y and
z were not constrained to its 1 cm bound by this motion. A 20 s `--sensors imu`
bag (29 MB) gives the same verdicts in 31 s.

### All sensors, 90 s

A 90 s bag (411 MB: 18,000 IMU messages, 899 scans per LiDAR, 900 images)
takes 7 min 41 s to check against `good.yaml`:

| Pair | Verdict with the true mountings | Largest judged error |
|---|---|---|
| imu–lidar | **pass** (roll, pitch, yaw, y) | 0.008°, 0.2 mm |
| imu–lidar2 | **pass** (roll, pitch, yaw) | 0.024° |
| lidar–lidar | **pass** (all six axes) | 0.019°, 1.0 mm |
| camera–imu | **warn** (pitch, yaw) | yaw 1.12° (tolerance 0.84°) |
| camera-focal | inconclusive | — |
| camera–lidar | **fail** (roll, pitch) | roll 2.65° |
| camera–lidar2 | inconclusive | — |

The turned candidates fail exactly the pairs of the sensor they turn:
`turned_imu.yaml` fails imu–lidar (yaw 2.99°) and imu–lidar2 (yaw 2.86°) and
passes lidar–lidar; `turned_lidar2.yaml` fails imu–lidar2 (yaw 3.01°) and
lidar–lidar (yaw 2.85°) and passes imu–lidar. At 40 s, camera–imu is
inconclusive; Calibrex asks for about 60 s of the same motion.

### The camera pairs

The camera pairs report errors against the true mountings, although the bag's
camera and LiDAR geometry agree to millimetres when the rig is still (the test
above). camera–lidar's roll error persists when the motion is slowed: a 60 s
bag at half speed estimates roll +3.0° (not judged, its control undetected). A
likely cause is on Calibrex's side: its edge objective recovers LiDAR beams
from point elevation after moving the points to the image time, and on this
fast hand-held motion (1.9° median motion per frame) no beams are recovered
(`median_rings: 0` in its evidence), so it falls back to unstructured edges.
The bag's `ring` field would give the beams directly. This was not changed or
verified further here.

### The reported time offset

Calibrex also reports a −1.87 ms IMU–LiDAR time offset, although both sensors
are stamped from the same simulation clock (reported, not judged). It is not
RNE's sensors. Bags written with one change each, all checked against the true
mounting:

| Bag | Time offset |
|---|---|
| as above | −1.87 ms |
| ideal IMU (no noise or bias) | −1.79 ms |
| ideal IMU, accelerometer replaced by the exact specific force | −1.79 ms |
| as previous, noiseless LiDAR | −1.70 ms |
| ideal IMU, exact specific force, 1000 Hz | −1.80 ms |
| ideal IMU, 2000 Hz | −1.71 ms |

Calibrex's own synthetic LiDAR + IMU bag (`calibrex.data.synthetic_lidar_imu_bag`,
40 s, no RNE code) shows the same bias: a true offset of 0 is reported as
−1.46 ms and a true offset of +7 ms as +5.52 ms. At 5, 10 and 20 Hz LiDAR the
zero-offset bag reports −1.13, −1.46 and −1.73 ms, so the bias does not scale
with the scan period. It is a near-constant shift of about −1.1 to −1.9 ms in
Calibrex's rotation-based offset estimate; its source inside the estimator was
not traced.

This is simulated data: it shows that RNE's sensors produce a recording a
calibration tool reads and judges like a real one, not how accurate Calibrex is
on hardware.
