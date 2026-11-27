# rne_adapter_lidarloc

[lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2)'s
scan-to-map registration for Robot Native Engine, without ROS 2: its
`lidar_localization_core` (the code the ROS 2 node runs) through the C API
`core_c_api.h`. NDT_OMP, GICP_OMP, PCL NDT/GICP or small_gicp; the node's
voxel filter, local-map crop and NDT initializer; the pose, convergence,
fitness, inlier ratio and (NDT_OMP) localizability of each alignment.

```bash
sudo apt-get install libpcl-dev libeigen3-dev libomp-dev
scripts/build_lidarloc.sh        # pinned core into target/lidarloc/install
cargo test --release -p rne_adapter_lidarloc
```

`build.rs` links `target/lidarloc/install/lib/liblidar_localization_core.a`
(or `$RNE_LIDARLOC_DIR/lib/...`) and PCL. Without it, or with
`RNE_LIDARLOC_DISABLE=1`, the crate compiles as a stub whose constructor
returns `LidarLocError::Unavailable`, and its tests skip (unless
`RNE_LIDARLOC_REQUIRED=1`, which CI's `lidarloc` job sets).

```rust
use rne_adapter_lidarloc::{LidarLoc, LidarLocConfig};

let mut loc = LidarLoc::new(&LidarLocConfig::default())?;   // NDT_OMP
loc.set_map(&map_points)?;
let aligned = loc.align(&scan_points, &guess)?;            // guess: map <- sensor
if aligned.converged && aligned.inlier_ratio > 0.6 { /* use aligned.pose */ }
```

The node's per-scan tracking, without the IMU: the seed (yours, or the
previous motion), the NDT initializer, the alignment (with the map-distance
prefilter once tracking is trusted), a retry from the last accepted pose,
the measurement gate, and local re-acquisition after an outage:

```rust
use rne_adapter_lidarloc::{Reacquisition, Tracking};

loc.set_tracking(&Tracking { enable_seed_correction_guard: true, ..Tracking::default() })?;
loc.set_reacquisition(&Reacquisition::default())?;          // occupancy grid from the map
loc.set_initial_pose(&start, 0.0)?;
let update = loc.localize(&scan_points, stamp, Some(&odometry_seed))?;
if update.accepted { /* use update.pose */ }
```

`Alignment::planar_covariance` gives the shape of the covariance from the
NDT score Hessian. It does not detect a scan of bare ground, which NDT's
cells pin to their lattice.

Example 152 uses it with `--lidarloc`.
