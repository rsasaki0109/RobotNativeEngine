# 149 — Calibrex rig bag

A LiDAR + IMU rig simulated in RNE and recorded as a ROS 2 bag, so
[Calibrex](https://github.com/rsasaki0109/Calibrex) can check its sensor
calibration as it would on a real robot's recording.

```bash
cargo run --release -p calibrex_rig_bag --example 149_calibrex_rig_bag -- --out target/calibrex_rig
calibrex check target/calibrex_rig/rig.mcap --tf target/calibrex_rig/good.yaml     # pass
calibrex check target/calibrex_rig/rig.mcap --tf target/calibrex_rig/turned.yaml   # fail on yaw
```

`--duration-s` sets the recording length (default 40 s). Install Calibrex from
its repository (`pip install git+https://github.com/rsasaki0109/Calibrex.git`):
it reads the bag with its own pure-Python MCAP reader, no ROS install needed.

## What is simulated

- **Scene**: a 12 m × 10 m × 3 m room of Rapier cuboid colliders, with two
  slanted panels, two pillars, a crate and a ramp so scan registration is
  constrained in every direction.
- **Motion**: a hand-held rig that turns about all three axes (up to about
  30°, 26° and 57°) and translates by about half a metre, so every rotation
  axis of the mounting is observable.
- **LiDAR**: RNE's `LidarSpec`, 16 channels over ±15°, 360 columns per scan at
  10 Hz, 1 cm range noise. Each scan is ray-cast in 20 sub-sweeps between
  exactly sampled rig poses, so it is motion-distorted like a real spinning
  LiDAR; every point is expressed in the sensor frame at its own firing time and
  carries that time (`time`, seconds after the header stamp).
- **IMU**: RNE's mounted IMU model (`ImuMount` with a 2°/−3°/92° rotation and a
  (0.10, −0.06, 0.15) m lever arm), sampled at 200 Hz from the rig's rigid-body
  kinematics, with white noise and turn-on biases on gyro and accelerometer.
- **Frames**: ROS conventions (x forward, z up) in the bag although the RNE
  world is y-up; `/tf_static` carries the true `lidar -> imu` mounting.

## Output

| File | Contents |
|---|---|
| `rig.mcap` | ROS 2 bag (MCAP storage, CDR): `/imu/data` (`sensor_msgs/Imu`), `/lidar/points` (`sensor_msgs/PointCloud2`, `x y z intensity ring time`), `/tf_static` |
| `good.yaml` | the true mounting as a Calibrex `--tf` frames file |
| `turned.yaml` | the IMU turned 3° about the LiDAR z axis |

The bag is written by `rne_adapter_ros2`'s `cdr` encoders and `mcap` writer,
which need no ROS 2 installation.

## Result

A 40 s bag (8,000 IMU messages, 399 scans, 2.3 M points; 1.5 min to simulate
with `--release` on a 4-core container), checked with Calibrex at commit
`c8ac18dd`:

| Candidate | Verdict | roll | pitch | yaw |
|---|---|---|---|---|
| `good.yaml` | **pass** (roll, pitch, yaw) | 0.002° | 0.006° | 0.009° |
| `turned.yaml` | **fail** | 0.002° | 0.006° | **2.99°** |

Tolerance 0.5° per axis. The first check took 69 s; the second reused
Calibrex's cache and took 1.5 s. The lever arm (x, y, z) was reported but not
judged: Calibrex's held-out control did not detect a 2 cm error on x, and y and
z were not constrained to its 1 cm bound by this motion. Calibrex also reported
a −1.87 ms IMU–LiDAR time offset although both are stamped from the same
simulation clock; RNE's IMU derives acceleration from a backward difference of
the body velocity (half a 5 ms sample late), which is a likely contributor, but
this was not isolated.

This is simulated data: it shows that RNE's sensors produce a recording a
calibration tool reads and judges like a real one, not how accurate Calibrex is
on hardware.
