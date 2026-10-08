# Checked LiDAR acquisition

`rne_sensor::sample_lidar_checked` is a renderer-independent instantaneous default-material sampler with explicit `SensorNoiseKey`. It returns `Result<PointCloud, LidarSampleError>`: a healthy zero-return scan is `Ok(empty)`; invalid geometry/mount or a failed physics raycast is `Err`. Any failed ray invalidates this strict scan, regardless of the legacy `DropRay` setting. Existing samplers retain their permissive behavior and are unchanged.

The output is world-frame meters with zero per-point emission offsets. Adapters must supply an absolute acquisition timestamp and delivery latency and must transform world points to their declared sensor/body frame. The function does not attach a timestamp or pretend a transport failure is free space. This is an instantaneous default-material boundary; material-aware swept/pattern diagnostic acquisition remains future work.

RustDrive consumes this API in a CPU-only Ackermann/LiDAR closed loop and maps errors to `lidar_failed`, which requests emergency braking. Tests distinguish healthy empty clouds, failures under both legacy policies, invalid geometry and invalid mounts.

Rapier pose synchronization now propagates modified body poses to attached colliders before updating the query pipeline. Raycasts immediately after `sync_from_ecs` therefore use current geometry without an artificial physics tick. A regression covers both fixed and kinematic bodies moving from 5 m to 10 m: hit range changes from 4 m to 9 m before stepping.
