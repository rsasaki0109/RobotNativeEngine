//! [lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2)'s
//! scan-to-map registration for Robot Native Engine, through its ROS-free
//! core (`lidar_localization_core`, `core_c_api.h`): the code the ROS 2 node
//! runs, without ROS.
//!
//! [`LidarLoc`] holds a point-cloud map and aligns scans to it from a guess:
//! it voxel-filters the scan as the node does, crops the target around the
//! guess when asked to, refines the guess with the NDT initializer while that
//! runs, aligns with NDT_OMP, GICP_OMP, PCL NDT/GICP or small_gicp, and
//! reports the pose, whether it converged, the fitness, the inlier ratio and
//! (NDT_OMP) the localizability of the match.
//!
//! The core is linked when `scripts/build_lidarloc.sh` (or `RNE_LIDARLOC_DIR`)
//! provides it; otherwise [`LidarLoc::new`] returns
//! [`LidarLocError::Unavailable`] and [`available`] is false.

use rne_math::{Mat4, Vec3};
#[cfg(lidarloc)]
use std::ffi::CStr;
use std::ffi::{c_char, c_double, c_int, CString};

/// Errors from the adapter.
#[derive(Debug, thiserror::Error)]
pub enum LidarLocError {
    /// The crate was built without lidar_localization_core.
    #[error("lidar_localization_core is not linked: run scripts/build_lidarloc.sh")]
    Unavailable,
    /// The core refused the configuration.
    #[error("lidar_localization_core: {0}")]
    Config(String),
    /// The map could not be set (empty, or the core failed).
    #[error("map not set: {0}")]
    Map(String),
    /// No alignment ran (no map, or an empty scan).
    #[error("no alignment: {0}")]
    Align(String),
}

/// Whether lidar_localization_core is linked.
#[must_use]
pub const fn available() -> bool {
    cfg!(lidarloc)
}

/// Registration settings, the node's parameters of the same names.
#[derive(Debug, Clone, PartialEq)]
pub struct LidarLocConfig {
    /// `NDT_OMP`, `GICP_OMP`, `NDT`, `GICP`, `SMALL_GICP`, `SMALL_VGICP`.
    pub registration_method: String,
    pub transform_epsilon: f64,
    pub ndt_resolution: f64,
    pub ndt_step_size: f64,
    pub ndt_max_iterations: i32,
    /// 0 or less: all OpenMP threads.
    pub ndt_num_threads: i32,
    pub gicp_corr_randomness: i32,
    pub gicp_max_correspondence_distance: f64,
    pub vgicp_voxel_resolution: f64,
    /// Voxel leaf of the filtered targets and, with `filter_scan`, the scans.
    pub voxel_leaf_size: f64,
    pub filter_scan: bool,
    pub enable_local_map_crop: bool,
    pub local_map_radius: f64,
    pub local_map_min_points: usize,
    pub local_map_update_distance: f64,
    pub scan_max_range: f64,
    pub ndt_init_scans_required: i32,
    /// A scan point within this distance of the map counts as an inlier.
    pub inlier_ratio_distance_m: f64,
    /// Scans with fewer points get an inlier ratio of 0.
    pub inlier_ratio_min_points: usize,
    /// Analyse the NDT_OMP Hessian.
    pub localizability: bool,
    /// Above 0: [`LidarLoc::align`] aligns only the scan points within this
    /// distance of the map at the guess (people and vehicles the map lacks),
    /// or the whole scan when fewer than `map_distance_filter_min_kept_ratio`
    /// of them are. Leave it 0 for a guess that is not a tracked pose.
    pub map_distance_filter_m: f64,
    pub map_distance_filter_min_kept_ratio: f64,
}

impl Default for LidarLocConfig {
    /// The node's parameter defaults, with NDT_OMP and the localizability.
    fn default() -> Self {
        Self {
            registration_method: "NDT_OMP".into(),
            transform_epsilon: 0.01,
            ndt_resolution: 1.0,
            ndt_step_size: 0.1,
            ndt_max_iterations: 35,
            ndt_num_threads: 4,
            gicp_corr_randomness: 20,
            gicp_max_correspondence_distance: 2.0,
            vgicp_voxel_resolution: 1.0,
            voxel_leaf_size: 0.2,
            filter_scan: true,
            enable_local_map_crop: false,
            local_map_radius: 150.0,
            local_map_min_points: 100,
            local_map_update_distance: 0.0,
            scan_max_range: 100.0,
            ndt_init_scans_required: 5,
            inlier_ratio_distance_m: 0.5,
            inlier_ratio_min_points: 200,
            localizability: true,
            map_distance_filter_m: 0.0,
            map_distance_filter_min_kept_ratio: 0.3,
        }
    }
}

/// The NDT_OMP Hessian's localizability: its eigenvalues in ascending
/// absolute value, and the weakest one's direction (x, y, z, roll, pitch,
/// yaw).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Localizability {
    pub eigenvalues: [f64; 6],
    pub weakest_direction: [f64; 6],
    /// Weakest over strongest absolute eigenvalue.
    pub weak_ratio: f64,
    pub correspondences: usize,
}

/// One alignment.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Alignment {
    /// Whether a target could be set around the guess.
    pub target_ready: bool,
    pub converged: bool,
    /// `None` when the backend does not report it.
    pub iterations: Option<u32>,
    /// The aligned pose (map from sensor); the guess when not converged.
    pub pose: Mat4,
    pub fitness_score: f64,
    pub inlier_ratio: f64,
    pub seconds: f64,
    /// Scan points given to the backend, after the voxel filter.
    pub source_points: usize,
    pub localizability: Option<Localizability>,
    /// With `map_distance_filter_m`: the scan points within it of the map at
    /// the guess, and whether only those were aligned.
    pub map_distance_filter_kept_points: usize,
    pub map_distance_filter_applied: bool,
    /// NDT_OMP with `localizability`: the Hessian of the NDT score at the
    /// pose, row-major in (x, y, z, roll, pitch, yaw). The score is at a
    /// maximum, so it is negative definite; see [`Alignment::planar_covariance`].
    pub score_hessian: Option<[f64; 36]>,
}

impl Alignment {
    /// The shape of the planar pose covariance from the NDT score Hessian:
    /// the (x, y, yaw) block of its negated inverse, as `([[xx, xy], [xy,
    /// yy]], yaw)`. The NDT score's scale is not the sensor's noise, so the
    /// caller scales it (it calibrates the factor once). `None` without a
    /// Hessian or when the Hessian is not negative definite.
    ///
    /// It does not detect degenerate scans on flat ground: NDT's cells pin
    /// a scan of bare ground to their lattice, and the Hessian is then as
    /// definite as among walls.
    #[must_use]
    pub fn planar_covariance(&self) -> Option<([[f64; 2]; 2], f64)> {
        let h = self.score_hessian?;
        let mut information = [[0.0; 6]; 6];
        for (r, row) in information.iter_mut().enumerate() {
            for (c, value) in row.iter_mut().enumerate() {
                *value = -0.5 * (h[6 * r + c] + h[6 * c + r]);
            }
        }
        let c = inverse_spd(&information)?;
        Some(([[c[0][0], c[0][1]], [c[1][0], c[1][1]]], c[5][5]))
    }
}

/// The inverse of a symmetric positive-definite 6x6 by Cholesky; `None` when
/// it is not positive definite.
#[allow(clippy::needless_range_loop)] // Triangular index arithmetic.
fn inverse_spd(a: &[[f64; 6]; 6]) -> Option<[[f64; 6]; 6]> {
    let mut l = [[0.0; 6]; 6];
    for i in 0..6 {
        for j in 0..=i {
            let sum: f64 = (0..j).map(|k| l[i][k] * l[j][k]).sum();
            if i == j {
                let d = a[i][i] - sum;
                if !(d > 0.0 && d.is_finite()) {
                    return None;
                }
                l[i][i] = d.sqrt();
            } else {
                l[i][j] = (a[i][j] - sum) / l[j][j];
            }
        }
    }
    // Columns of the inverse: solve L Lᵀ x = e_k.
    let mut inverse = [[0.0; 6]; 6];
    for k in 0..6 {
        let mut y = [0.0; 6];
        for i in 0..6 {
            let e = if i == k { 1.0 } else { 0.0 };
            y[i] = (e - (0..i).map(|j| l[i][j] * y[j]).sum::<f64>()) / l[i][i];
        }
        for i in (0..6).rev() {
            let x = (y[i] - (i + 1..6).map(|j| l[j][i] * inverse[j][k]).sum::<f64>()) / l[i][i];
            inverse[i][k] = x;
        }
    }
    Some(inverse)
}

/// The tracking around each alignment in [`LidarLoc::localize`]: the node's
/// measurement gate, its retry from the last accepted pose and its
/// prediction (the node's parameters of the same names; no IMU).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tracking {
    /// The gate rejects a score above this (NDT fitness, or with
    /// `min_inlier_ratio` the inlier ratio on that scale).
    pub score_threshold: f64,
    pub min_inlier_ratio: f64,
    pub enable_seed_correction_guard: bool,
    pub seed_correction_guard_translation_m: f64,
    pub seed_correction_guard_yaw_deg: f64,
    pub seed_correction_guard_release_rejections: i32,
    pub seed_correction_guard_warmup_accepts: i32,
    pub enable_seed_correction_window_guard: bool,
    pub seed_correction_window_guard_sec: f64,
    pub seed_correction_window_guard_translation_m: f64,
    pub enable_recovery_retry_from_last_pose: bool,
    pub recovery_retry_from_last_pose_min_rejections: i32,
    pub recovery_retry_from_last_pose_max_accepted_gap_sec: f64,
    /// Without a seed: the last accepted motion extrapolated, else the
    /// tracked pose.
    pub predict_pose_from_previous_delta: bool,
    pub previous_delta_max_extrapolation_sec: f64,
    /// The map-distance prefilter on the primary alignment (with the
    /// config's `map_distance_filter_m`), as the node decides it: from this
    /// many accepted scans after the initial pose, growing by
    /// `growth_m_per_s` per second since the last accepted scan beyond one
    /// `scan_period_sec`, up to `max_distance_m`.
    pub map_distance_filter_growth_m_per_s: f64,
    pub map_distance_filter_max_distance_m: f64,
    pub map_distance_filter_warmup_accepts: i32,
    pub scan_period_sec: f64,
}

impl Default for Tracking {
    /// The node's defaults (`lidarloc_default_tracking`).
    fn default() -> Self {
        Self {
            score_threshold: 2.0,
            min_inlier_ratio: 0.0,
            enable_seed_correction_guard: false,
            seed_correction_guard_translation_m: 0.5,
            seed_correction_guard_yaw_deg: 15.0,
            seed_correction_guard_release_rejections: 10,
            seed_correction_guard_warmup_accepts: 5,
            enable_seed_correction_window_guard: false,
            seed_correction_window_guard_sec: 1.0,
            seed_correction_window_guard_translation_m: 0.3,
            enable_recovery_retry_from_last_pose: false,
            recovery_retry_from_last_pose_min_rejections: 1,
            recovery_retry_from_last_pose_max_accepted_gap_sec: 1.0,
            predict_pose_from_previous_delta: true,
            previous_delta_max_extrapolation_sec: 1.0,
            map_distance_filter_growth_m_per_s: 1.0,
            map_distance_filter_max_distance_m: 3.0,
            map_distance_filter_warmup_accepts: 5,
            scan_period_sec: 0.1,
        }
    }
}

/// One scan through [`LidarLoc::localize`].
#[derive(Debug, Clone, PartialEq)]
pub struct Update {
    /// Whether the gate accepted the scan and the tracked pose moved to it.
    pub accepted: bool,
    /// 0 ok, 1 warn, 2 error.
    pub status_level: i32,
    /// The node's status, e.g. `ok`, `score_threshold_rejected`.
    pub status: String,
    /// The tracked pose after this scan.
    pub pose: Mat4,
    /// The seed the scan was aligned from.
    pub seed: Mat4,
    pub retried_from_last_pose: bool,
    pub consecutive_rejected_updates: usize,
    /// The alignment the gate judged.
    pub alignment: Alignment,
    /// Local re-acquisition ([`LidarLoc::set_reacquisition`]): whether this
    /// scan was aligned from the previous scan's proposal (`accepted` then
    /// says whether the gate confirmed it), whether the search ran after
    /// this scan's rejection, its outcome (`proposed`, `ambiguous`, ...) and,
    /// when proposed, the pose the next scan is seeded from.
    pub seeded_from_reacquisition: bool,
    pub reacquisition_attempted: bool,
    pub reacquisition_reason: String,
    pub reacquisition_proposal: Option<Mat4>,
}

/// Global localization ([`LidarLoc::global_localize`]): the G2 engine
/// "bbs3d" search over the whole map, its candidates refined and the best
/// kept when it passes the tracking's score threshold and no other place
/// fits nearly as well.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GlobalSearch {
    /// The map's voxel hierarchy: thinned to `map_leaf_m`, finest voxels
    /// `min_level_res_m`, `max_level` coarser levels.
    pub map_leaf_m: f64,
    pub min_level_res_m: f64,
    pub max_level: i32,
    /// The scan as the search sees it (base frame, levelled).
    pub scan_min_range_m: f64,
    pub scan_max_range_m: f64,
    pub scan_leaf_m: f64,
    pub top_k: i32,
    pub min_separation_m: f64,
    pub relative_score_floor: f64,
    pub timeout_ms: i32,
    pub max_queue_nodes: usize,
    pub num_threads: i32,
    pub max_tilt_rad: f64,
    pub candidate_separation_m: f64,
    pub min_fitness_ratio: f64,
}

impl Default for GlobalSearch {
    /// The G2 bbs3d defaults (`lidarloc_default_global_config`).
    fn default() -> Self {
        Self {
            map_leaf_m: 0.1,
            min_level_res_m: 0.5,
            max_level: 6,
            scan_min_range_m: 1.0,
            scan_max_range_m: 30.0,
            scan_leaf_m: 0.5,
            top_k: 8,
            min_separation_m: 3.0,
            relative_score_floor: 0.9,
            timeout_ms: 10_000,
            max_queue_nodes: 4_000_000,
            num_threads: 4,
            max_tilt_rad: 0.02,
            candidate_separation_m: 2.0,
            min_fitness_ratio: 1.2,
        }
    }
}

/// One global localization.
#[derive(Debug, Clone, PartialEq)]
pub struct GlobalFix {
    /// The pose found (map from sensor), when one was.
    pub pose: Option<Mat4>,
    /// `proposed`, `ambiguous`, `best_fitness_above_threshold`, ...
    pub reason: String,
    pub fitness: f64,
    pub candidates: usize,
    /// The best fitness among the other candidates.
    pub second_fitness: f64,
    pub scan_points: usize,
    pub timed_out: bool,
    pub voxelmap_build_ms: f64,
    pub search_ms: f64,
    pub total_ms: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(not(lidarloc), allow(dead_code))]
struct RawGlobalConfig {
    map_leaf_m: c_double,
    min_level_res_m: c_double,
    max_level: c_int,
    scan_min_range_m: c_double,
    scan_max_range_m: c_double,
    scan_leaf_m: c_double,
    top_k: c_int,
    min_separation_m: c_double,
    relative_score_floor: c_double,
    timeout_ms: c_int,
    max_queue_nodes: usize,
    num_threads: c_int,
    max_tilt_rad: c_double,
    candidate_separation_m: c_double,
    min_fitness_ratio: c_double,
}

impl From<&GlobalSearch> for RawGlobalConfig {
    fn from(g: &GlobalSearch) -> Self {
        Self {
            map_leaf_m: g.map_leaf_m,
            min_level_res_m: g.min_level_res_m,
            max_level: g.max_level,
            scan_min_range_m: g.scan_min_range_m,
            scan_max_range_m: g.scan_max_range_m,
            scan_leaf_m: g.scan_leaf_m,
            top_k: g.top_k,
            min_separation_m: g.min_separation_m,
            relative_score_floor: g.relative_score_floor,
            timeout_ms: g.timeout_ms,
            max_queue_nodes: g.max_queue_nodes,
            num_threads: g.num_threads,
            max_tilt_rad: g.max_tilt_rad,
            candidate_separation_m: g.candidate_separation_m,
            min_fitness_ratio: g.min_fitness_ratio,
        }
    }
}

#[repr(C)]
#[cfg_attr(not(lidarloc), allow(dead_code))]
struct RawGlobalResult {
    found: c_int,
    reason: [c_char; 32],
    pose: [c_double; 16],
    fitness: c_double,
    candidates: usize,
    second_fitness: c_double,
    scan_points: usize,
    timed_out: c_int,
    voxelmap_build_ms: c_double,
    search_ms: c_double,
    total_ms: c_double,
}

/// Local re-acquisition after an odometry-bridged outage, the node's
/// `enable_local_reacquisition` on an occupancy grid generated from the map:
/// after `min_rejections` rejected scans given a seed (the odometry bridge:
/// the last tracked pose moved by odometry), a 2D branch-and-bound search
/// within `search_radius_m` of the seed near its heading, each candidate
/// refined; the best is proposed when it passes the score threshold and no
/// other place fits nearly as well, and the next scan with a seed is
/// aligned from it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Reacquisition {
    pub enable: bool,
    pub min_rejections: i32,
    pub attempt_interval_scans: i32,
    pub search_radius_m: f64,
    pub yaw_window_deg: f64,
    pub max_candidates: i32,
    pub candidate_separation_m: f64,
    pub min_fitness_ratio: f64,
    pub scan_min_z_m: f64,
    pub scan_max_z_m: f64,
    pub scan_min_range_m: f64,
    pub scan_voxel_m: f64,
    pub scan_max_points: i32,
    pub angular_resolution_deg: f64,
    pub pyramid_depth: i32,
    /// The occupancy grid: a cell is occupied with
    /// `occupancy_min_points_per_cell` points between
    /// `occupancy_obstacle_height_m` and `occupancy_max_obstacle_height_m`
    /// (0: no limit) above its lowest point, inflated by
    /// `occupancy_inflate_radius_m`.
    pub occupancy_resolution_m: f64,
    pub occupancy_obstacle_height_m: f64,
    pub occupancy_max_obstacle_height_m: f64,
    pub occupancy_min_points_per_cell: i32,
    pub occupancy_inflate_radius_m: f64,
}

impl Default for Reacquisition {
    /// The node's defaults, enabled.
    fn default() -> Self {
        Self {
            enable: true,
            min_rejections: 10,
            attempt_interval_scans: 10,
            search_radius_m: 15.0,
            yaw_window_deg: 20.0,
            max_candidates: 6,
            candidate_separation_m: 2.0,
            min_fitness_ratio: 1.2,
            scan_min_z_m: 0.5,
            scan_max_z_m: 5.0,
            scan_min_range_m: 1.0,
            scan_voxel_m: 0.2,
            scan_max_points: 512,
            angular_resolution_deg: 2.0,
            pyramid_depth: 4,
            occupancy_resolution_m: 0.2,
            occupancy_obstacle_height_m: 0.4,
            occupancy_max_obstacle_height_m: 0.0,
            occupancy_min_points_per_cell: 2,
            occupancy_inflate_radius_m: 0.6,
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(not(lidarloc), allow(dead_code))]
struct RawReacquisition {
    enable: c_int,
    min_rejections: c_int,
    attempt_interval_scans: c_int,
    search_radius_m: c_double,
    yaw_window_deg: c_double,
    max_candidates: c_int,
    candidate_separation_m: c_double,
    min_fitness_ratio: c_double,
    scan_min_z_m: c_double,
    scan_max_z_m: c_double,
    scan_min_range_m: c_double,
    scan_voxel_m: c_double,
    scan_max_points: c_int,
    angular_resolution_deg: c_double,
    pyramid_depth: c_int,
    occupancy_resolution_m: c_double,
    occupancy_obstacle_height_m: c_double,
    occupancy_max_obstacle_height_m: c_double,
    occupancy_min_points_per_cell: c_int,
    occupancy_inflate_radius_m: c_double,
}

impl From<&Reacquisition> for RawReacquisition {
    fn from(r: &Reacquisition) -> Self {
        Self {
            enable: c_int::from(r.enable),
            min_rejections: r.min_rejections,
            attempt_interval_scans: r.attempt_interval_scans,
            search_radius_m: r.search_radius_m,
            yaw_window_deg: r.yaw_window_deg,
            max_candidates: r.max_candidates,
            candidate_separation_m: r.candidate_separation_m,
            min_fitness_ratio: r.min_fitness_ratio,
            scan_min_z_m: r.scan_min_z_m,
            scan_max_z_m: r.scan_max_z_m,
            scan_min_range_m: r.scan_min_range_m,
            scan_voxel_m: r.scan_voxel_m,
            scan_max_points: r.scan_max_points,
            angular_resolution_deg: r.angular_resolution_deg,
            pyramid_depth: r.pyramid_depth,
            occupancy_resolution_m: r.occupancy_resolution_m,
            occupancy_obstacle_height_m: r.occupancy_obstacle_height_m,
            occupancy_max_obstacle_height_m: r.occupancy_max_obstacle_height_m,
            occupancy_min_points_per_cell: r.occupancy_min_points_per_cell,
            occupancy_inflate_radius_m: r.occupancy_inflate_radius_m,
        }
    }
}

#[repr(C)]
struct RawConfig {
    registration_method: *const c_char,
    transform_epsilon: c_double,
    ndt_resolution: c_double,
    ndt_step_size: c_double,
    ndt_max_iterations: c_int,
    ndt_num_threads: c_int,
    gicp_corr_randomness: c_int,
    gicp_max_correspondence_distance: c_double,
    vgicp_voxel_resolution: c_double,
    voxel_leaf_size: c_double,
    filter_scan: c_int,
    enable_local_map_crop: c_int,
    local_map_radius: c_double,
    local_map_min_points: usize,
    local_map_update_distance: c_double,
    scan_max_range: c_double,
    ndt_init_scans_required: c_int,
    inlier_ratio_distance_m: c_double,
    inlier_ratio_min_points: usize,
    localizability: c_int,
    map_distance_filter_m: c_double,
    map_distance_filter_min_kept_ratio: c_double,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(not(lidarloc), allow(dead_code))]
struct RawTracking {
    score_threshold: c_double,
    min_inlier_ratio: c_double,
    enable_seed_correction_guard: c_int,
    seed_correction_guard_translation_m: c_double,
    seed_correction_guard_yaw_deg: c_double,
    seed_correction_guard_release_rejections: c_int,
    seed_correction_guard_warmup_accepts: c_int,
    enable_seed_correction_window_guard: c_int,
    seed_correction_window_guard_sec: c_double,
    seed_correction_window_guard_translation_m: c_double,
    enable_recovery_retry_from_last_pose: c_int,
    recovery_retry_from_last_pose_min_rejections: c_int,
    recovery_retry_from_last_pose_max_accepted_gap_sec: c_double,
    predict_pose_from_previous_delta: c_int,
    previous_delta_max_extrapolation_sec: c_double,
    map_distance_filter_growth_m_per_s: c_double,
    map_distance_filter_max_distance_m: c_double,
    map_distance_filter_warmup_accepts: c_int,
    scan_period_sec: c_double,
}

impl From<&Tracking> for RawTracking {
    fn from(t: &Tracking) -> Self {
        Self {
            score_threshold: t.score_threshold,
            min_inlier_ratio: t.min_inlier_ratio,
            enable_seed_correction_guard: c_int::from(t.enable_seed_correction_guard),
            seed_correction_guard_translation_m: t.seed_correction_guard_translation_m,
            seed_correction_guard_yaw_deg: t.seed_correction_guard_yaw_deg,
            seed_correction_guard_release_rejections: t.seed_correction_guard_release_rejections,
            seed_correction_guard_warmup_accepts: t.seed_correction_guard_warmup_accepts,
            enable_seed_correction_window_guard: c_int::from(t.enable_seed_correction_window_guard),
            seed_correction_window_guard_sec: t.seed_correction_window_guard_sec,
            seed_correction_window_guard_translation_m: t
                .seed_correction_window_guard_translation_m,
            enable_recovery_retry_from_last_pose: c_int::from(
                t.enable_recovery_retry_from_last_pose,
            ),
            recovery_retry_from_last_pose_min_rejections: t
                .recovery_retry_from_last_pose_min_rejections,
            recovery_retry_from_last_pose_max_accepted_gap_sec: t
                .recovery_retry_from_last_pose_max_accepted_gap_sec,
            predict_pose_from_previous_delta: c_int::from(t.predict_pose_from_previous_delta),
            previous_delta_max_extrapolation_sec: t.previous_delta_max_extrapolation_sec,
            map_distance_filter_growth_m_per_s: t.map_distance_filter_growth_m_per_s,
            map_distance_filter_max_distance_m: t.map_distance_filter_max_distance_m,
            map_distance_filter_warmup_accepts: t.map_distance_filter_warmup_accepts,
            scan_period_sec: t.scan_period_sec,
        }
    }
}

#[repr(C)]
#[cfg_attr(not(lidarloc), allow(dead_code))]
struct RawUpdate {
    accepted: c_int,
    status_level: c_int,
    status: [c_char; 64],
    pose: [c_double; 16],
    seed: [c_double; 16],
    retried_from_last_pose: c_int,
    consecutive_rejected_updates: usize,
    alignment: RawResult,
    seeded_from_reacquisition: c_int,
    reacquisition_attempted: c_int,
    reacquisition_proposed: c_int,
    reacquisition_reason: [c_char; 32],
    reacquisition_pose: [c_double; 16],
}

#[repr(C)]
#[cfg_attr(not(lidarloc), allow(dead_code))]
struct RawResult {
    target_ready: c_int,
    has_converged: c_int,
    iterations: c_int,
    pose: [c_double; 16],
    fitness_score: c_double,
    inlier_ratio: c_double,
    alignment_time_sec: c_double,
    source_points: usize,
    localizability_valid: c_int,
    eigenvalues: [c_double; 6],
    weakest_eigenvector: [c_double; 6],
    weak_ratio: c_double,
    correspondence_count: usize,
    map_distance_filter_kept_points: usize,
    map_distance_filter_applied: c_int,
    score_hessian_valid: c_int,
    score_hessian: [c_double; 36],
}

#[cfg_attr(not(lidarloc), allow(dead_code))]
impl RawResult {
    fn alignment(&self) -> Alignment {
        Alignment {
            target_ready: self.target_ready != 0,
            converged: self.has_converged != 0,
            iterations: u32::try_from(self.iterations).ok(),
            pose: from_row_major(&self.pose),
            fitness_score: self.fitness_score,
            inlier_ratio: self.inlier_ratio,
            seconds: self.alignment_time_sec,
            source_points: self.source_points,
            localizability: (self.localizability_valid != 0).then_some(Localizability {
                eigenvalues: self.eigenvalues,
                weakest_direction: self.weakest_eigenvector,
                weak_ratio: self.weak_ratio,
                correspondences: self.correspondence_count,
            }),
            map_distance_filter_kept_points: self.map_distance_filter_kept_points,
            map_distance_filter_applied: self.map_distance_filter_applied != 0,
            score_hessian: (self.score_hessian_valid != 0).then_some(self.score_hessian),
        }
    }
}

#[repr(C)]
struct RawCore {
    _private: [u8; 0],
}

#[cfg(lidarloc)]
extern "C" {
    fn lidarloc_create(config: *const RawConfig, error: *mut c_char, size: usize) -> *mut RawCore;
    fn lidarloc_destroy(core: *mut RawCore);
    fn lidarloc_set_map(core: *mut RawCore, xyz: *const f32, count: usize) -> c_int;
    fn lidarloc_align(
        core: *mut RawCore,
        xyz: *const f32,
        count: usize,
        guess: *const c_double,
        result: *mut RawResult,
    ) -> c_int;
    fn lidarloc_last_log(core: *const RawCore) -> *const c_char;
    #[cfg(test)]
    fn lidarloc_default_tracking() -> RawTracking;
    #[cfg(test)]
    fn lidarloc_default_reacquisition() -> RawReacquisition;
    fn lidarloc_set_reacquisition(core: *mut RawCore, r: *const RawReacquisition) -> c_int;
    #[cfg(test)]
    fn lidarloc_default_global_config() -> RawGlobalConfig;
    fn lidarloc_global_localize(
        core: *mut RawCore,
        xyz: *const f32,
        count: usize,
        config: *const RawGlobalConfig,
        result: *mut RawGlobalResult,
    ) -> c_int;
    fn lidarloc_set_tracking(core: *mut RawCore, tracking: *const RawTracking) -> c_int;
    fn lidarloc_set_initial_pose(
        core: *mut RawCore,
        pose: *const c_double,
        stamp: c_double,
    ) -> c_int;
    fn lidarloc_localize(
        core: *mut RawCore,
        xyz: *const f32,
        count: usize,
        stamp: c_double,
        seed: *const c_double,
        update: *mut RawUpdate,
    ) -> c_int;
}

/// The row-major 4x4 the C API takes.
fn row_major(m: &Mat4) -> [f64; 16] {
    let c = m.to_cols_array();
    let mut r = [0.0; 16];
    for row in 0..4 {
        for col in 0..4 {
            r[4 * row + col] = c[4 * col + row];
        }
    }
    r
}

fn from_row_major(r: &[f64; 16]) -> Mat4 {
    let mut c = [0.0; 16];
    for row in 0..4 {
        for col in 0..4 {
            c[4 * col + row] = r[4 * row + col];
        }
    }
    Mat4::from_cols_array(&c)
}

fn flatten(points: &[Vec3]) -> Vec<f32> {
    points
        .iter()
        .flat_map(|p| [p.x as f32, p.y as f32, p.z as f32])
        .collect()
}

/// lidar_localization_ros2's registration on a map.
pub struct LidarLoc {
    core: *mut RawCore,
    // Keeps the method string the core copied alive for the call.
    _method: CString,
}

// SAFETY: the core is used from one thread at a time through &mut self.
unsafe impl Send for LidarLoc {}

impl std::fmt::Debug for LidarLoc {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LidarLoc").finish_non_exhaustive()
    }
}

impl LidarLoc {
    /// A registration with `config`.
    ///
    /// # Errors
    /// [`LidarLocError::Unavailable`] without the core;
    /// [`LidarLocError::Config`] when the core refuses the configuration.
    pub fn new(config: &LidarLocConfig) -> Result<Self, LidarLocError> {
        let method = CString::new(config.registration_method.as_str())
            .map_err(|e| LidarLocError::Config(e.to_string()))?;
        let raw = RawConfig {
            registration_method: method.as_ptr(),
            transform_epsilon: config.transform_epsilon,
            ndt_resolution: config.ndt_resolution,
            ndt_step_size: config.ndt_step_size,
            ndt_max_iterations: config.ndt_max_iterations,
            ndt_num_threads: config.ndt_num_threads,
            gicp_corr_randomness: config.gicp_corr_randomness,
            gicp_max_correspondence_distance: config.gicp_max_correspondence_distance,
            vgicp_voxel_resolution: config.vgicp_voxel_resolution,
            voxel_leaf_size: config.voxel_leaf_size,
            filter_scan: c_int::from(config.filter_scan),
            enable_local_map_crop: c_int::from(config.enable_local_map_crop),
            local_map_radius: config.local_map_radius,
            local_map_min_points: config.local_map_min_points,
            local_map_update_distance: config.local_map_update_distance,
            scan_max_range: config.scan_max_range,
            ndt_init_scans_required: config.ndt_init_scans_required,
            inlier_ratio_distance_m: config.inlier_ratio_distance_m,
            inlier_ratio_min_points: config.inlier_ratio_min_points,
            localizability: c_int::from(config.localizability),
            map_distance_filter_m: config.map_distance_filter_m,
            map_distance_filter_min_kept_ratio: config.map_distance_filter_min_kept_ratio,
        };
        Self::create(&raw, method)
    }

    #[cfg(lidarloc)]
    fn create(raw: &RawConfig, method: CString) -> Result<Self, LidarLocError> {
        let mut error = [0 as c_char; 256];
        // SAFETY: raw and its method string outlive the call; error is a
        // buffer of the size given.
        let core = unsafe { lidarloc_create(raw, error.as_mut_ptr(), error.len()) };
        if core.is_null() {
            // SAFETY: the core wrote a NUL-terminated message into error.
            let message = unsafe { CStr::from_ptr(error.as_ptr()) };
            return Err(LidarLocError::Config(
                message.to_string_lossy().into_owned(),
            ));
        }
        Ok(Self {
            core,
            _method: method,
        })
    }

    #[cfg(not(lidarloc))]
    fn create(_raw: &RawConfig, _method: CString) -> Result<Self, LidarLocError> {
        Err(LidarLocError::Unavailable)
    }

    /// The core's last log message.
    #[must_use]
    pub fn last_log(&self) -> String {
        #[cfg(lidarloc)]
        {
            // SAFETY: the core returns a NUL-terminated string it owns,
            // valid until its next call.
            unsafe { CStr::from_ptr(lidarloc_last_log(self.core)) }
                .to_string_lossy()
                .into_owned()
        }
        #[cfg(not(lidarloc))]
        {
            let _ = self.core;
            String::new()
        }
    }

    /// Sets the map (map-frame points).
    ///
    /// # Errors
    /// [`LidarLocError::Map`] for an empty map or a core failure.
    pub fn set_map(&mut self, points: &[Vec3]) -> Result<(), LidarLocError> {
        let xyz = flatten(points);
        #[cfg(lidarloc)]
        {
            // SAFETY: xyz holds points.len() triplets for the call.
            let status = unsafe { lidarloc_set_map(self.core, xyz.as_ptr(), points.len()) };
            if status != 0 {
                return Err(LidarLocError::Map(self.last_log()));
            }
            Ok(())
        }
        #[cfg(not(lidarloc))]
        {
            let _ = xyz;
            Err(LidarLocError::Unavailable)
        }
    }

    /// Aligns a scan (sensor-frame points) to the map from `guess` (map from
    /// sensor).
    ///
    /// # Errors
    /// [`LidarLocError::Align`] when no alignment ran (no map, or the scan
    /// is empty after the voxel filter).
    pub fn align(&mut self, scan: &[Vec3], guess: &Mat4) -> Result<Alignment, LidarLocError> {
        let xyz = flatten(scan);
        let guess = row_major(guess);
        #[cfg(lidarloc)]
        {
            let mut raw = std::mem::MaybeUninit::<RawResult>::zeroed();
            // SAFETY: xyz holds scan.len() triplets, guess 16 values, and raw
            // is a result the core fills.
            let status = unsafe {
                lidarloc_align(
                    self.core,
                    xyz.as_ptr(),
                    scan.len(),
                    guess.as_ptr(),
                    raw.as_mut_ptr(),
                )
            };
            if status != 0 {
                return Err(LidarLocError::Align(self.last_log()));
            }
            // SAFETY: zero-initialized and filled by the core.
            let raw = unsafe { raw.assume_init() };
            Ok(raw.alignment())
        }
        #[cfg(not(lidarloc))]
        {
            let _ = (xyz, guess, from_row_major(&[0.0; 16]));
            Err(LidarLocError::Unavailable)
        }
    }
}

impl LidarLoc {
    /// Sets the tracking of [`LidarLoc::localize`].
    ///
    /// # Errors
    /// [`LidarLocError::Unavailable`] without the core.
    pub fn set_tracking(&mut self, tracking: &Tracking) -> Result<(), LidarLocError> {
        let raw = RawTracking::from(tracking);
        #[cfg(lidarloc)]
        {
            // SAFETY: raw lives for the call; the core copies it.
            let status = unsafe { lidarloc_set_tracking(self.core, &raw) };
            if status != 0 {
                return Err(LidarLocError::Config(self.last_log()));
            }
            Ok(())
        }
        #[cfg(not(lidarloc))]
        {
            let _ = raw;
            Err(LidarLocError::Unavailable)
        }
    }

    /// Sets local re-acquisition; enabled, the occupancy grid is built from
    /// the map (now, or when the map is set).
    ///
    /// # Errors
    /// [`LidarLocError::Unavailable`] without the core;
    /// [`LidarLocError::Config`] when the grid cannot be built.
    pub fn set_reacquisition(
        &mut self,
        reacquisition: &Reacquisition,
    ) -> Result<(), LidarLocError> {
        let raw = RawReacquisition::from(reacquisition);
        #[cfg(lidarloc)]
        {
            // SAFETY: raw lives for the call; the core copies it.
            let status = unsafe { lidarloc_set_reacquisition(self.core, &raw) };
            if status != 0 {
                return Err(LidarLocError::Config(self.last_log()));
            }
            Ok(())
        }
        #[cfg(not(lidarloc))]
        {
            let _ = raw;
            Err(LidarLocError::Unavailable)
        }
    }

    /// Where in the whole map the scan (sensor-frame points, levelled) was
    /// taken, with no prior. The voxel maps are built on the first call and
    /// kept. Start tracking at the pose found with
    /// [`LidarLoc::set_initial_pose`].
    ///
    /// # Errors
    /// [`LidarLocError::Align`] when the search could not run (no map, no
    /// scan points in range).
    pub fn global_localize(
        &mut self,
        scan: &[Vec3],
        search: &GlobalSearch,
    ) -> Result<GlobalFix, LidarLocError> {
        let xyz = flatten(scan);
        let config = RawGlobalConfig::from(search);
        #[cfg(lidarloc)]
        {
            let mut raw = std::mem::MaybeUninit::<RawGlobalResult>::zeroed();
            // SAFETY: xyz holds scan.len() triplets, config lives for the
            // call, and raw is a result the core fills.
            let status = unsafe {
                lidarloc_global_localize(
                    self.core,
                    xyz.as_ptr(),
                    scan.len(),
                    &config,
                    raw.as_mut_ptr(),
                )
            };
            if status != 0 {
                return Err(LidarLocError::Align(self.last_log()));
            }
            // SAFETY: zero-initialized and filled by the core.
            let raw = unsafe { raw.assume_init() };
            // SAFETY: the core writes a NUL-terminated reason.
            let reason = unsafe { CStr::from_ptr(raw.reason.as_ptr()) };
            Ok(GlobalFix {
                pose: (raw.found != 0).then(|| from_row_major(&raw.pose)),
                reason: reason.to_string_lossy().into_owned(),
                fitness: raw.fitness,
                candidates: raw.candidates,
                second_fitness: raw.second_fitness,
                scan_points: raw.scan_points,
                timed_out: raw.timed_out != 0,
                voxelmap_build_ms: raw.voxelmap_build_ms,
                search_ms: raw.search_ms,
                total_ms: raw.total_ms,
            })
        }
        #[cfg(not(lidarloc))]
        {
            let _ = (xyz, config);
            Err(LidarLocError::Unavailable)
        }
    }

    /// Starts tracking at `pose` (an initial pose, a reset).
    ///
    /// # Errors
    /// [`LidarLocError::Unavailable`] without the core.
    pub fn set_initial_pose(&mut self, pose: &Mat4, stamp_sec: f64) -> Result<(), LidarLocError> {
        let pose = row_major(pose);
        #[cfg(lidarloc)]
        {
            // SAFETY: pose holds 16 values for the call.
            let status = unsafe { lidarloc_set_initial_pose(self.core, pose.as_ptr(), stamp_sec) };
            if status != 0 {
                return Err(LidarLocError::Config(self.last_log()));
            }
            Ok(())
        }
        #[cfg(not(lidarloc))]
        {
            let _ = (pose, stamp_sec);
            Err(LidarLocError::Unavailable)
        }
    }

    /// One scan (sensor-frame points) through the node's per-scan flow
    /// without the IMU: the seed (`seed`, e.g. predicted from odometry, else
    /// the prediction), the NDT initializer, the alignment, a retry from the
    /// last accepted pose, the measurement gate and the update of the
    /// tracked pose.
    ///
    /// # Errors
    /// [`LidarLocError::Align`] when the scan could not be processed (no map
    /// or initial pose, an empty scan).
    pub fn localize(
        &mut self,
        scan: &[Vec3],
        stamp_sec: f64,
        seed: Option<&Mat4>,
    ) -> Result<Update, LidarLocError> {
        let xyz = flatten(scan);
        let seed = seed.map(row_major);
        #[cfg(lidarloc)]
        {
            let mut raw = std::mem::MaybeUninit::<RawUpdate>::zeroed();
            // SAFETY: xyz holds scan.len() triplets, seed 16 values or null,
            // and raw is an update the core fills.
            let status = unsafe {
                lidarloc_localize(
                    self.core,
                    xyz.as_ptr(),
                    scan.len(),
                    stamp_sec,
                    seed.as_ref().map_or(std::ptr::null(), |s| s.as_ptr()),
                    raw.as_mut_ptr(),
                )
            };
            if status != 0 {
                return Err(LidarLocError::Align(self.last_log()));
            }
            // SAFETY: zero-initialized and filled by the core.
            let raw = unsafe { raw.assume_init() };
            // SAFETY: the core writes a NUL-terminated status.
            let status = unsafe { CStr::from_ptr(raw.status.as_ptr()) };
            // SAFETY: the core writes a NUL-terminated reason (or none).
            let reason = unsafe { CStr::from_ptr(raw.reacquisition_reason.as_ptr()) };
            Ok(Update {
                seeded_from_reacquisition: raw.seeded_from_reacquisition != 0,
                reacquisition_attempted: raw.reacquisition_attempted != 0,
                reacquisition_reason: reason.to_string_lossy().into_owned(),
                reacquisition_proposal: (raw.reacquisition_proposed != 0)
                    .then(|| from_row_major(&raw.reacquisition_pose)),
                accepted: raw.accepted != 0,
                status_level: raw.status_level,
                status: status.to_string_lossy().into_owned(),
                pose: from_row_major(&raw.pose),
                seed: from_row_major(&raw.seed),
                retried_from_last_pose: raw.retried_from_last_pose != 0,
                consecutive_rejected_updates: raw.consecutive_rejected_updates,
                alignment: raw.alignment.alignment(),
            })
        }
        #[cfg(not(lidarloc))]
        {
            let _ = (xyz, seed, stamp_sec);
            Err(LidarLocError::Unavailable)
        }
    }
}

impl Drop for LidarLoc {
    fn drop(&mut self) {
        #[cfg(lidarloc)]
        // SAFETY: core came from lidarloc_create and is destroyed once.
        unsafe {
            lidarloc_destroy(self.core);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn row_major_round_trips() {
        let m = Mat4::from_cols_array(&[
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        ]);
        let r = row_major(&m);
        // The translation is the last column: row-major indices 3, 7, 11.
        assert_eq!([r[3], r[7], r[11]], [13.0, 14.0, 15.0]);
        assert_eq!(from_row_major(&r), m);
    }

    #[cfg(lidarloc)]
    #[test]
    fn tracking_defaults_are_the_cores() {
        // SAFETY: returns a value.
        let core = unsafe { lidarloc_default_tracking() };
        assert_eq!(RawTracking::from(&Tracking::default()), core);
        // SAFETY: returns a value.
        let core = unsafe { lidarloc_default_reacquisition() };
        let ours = RawReacquisition::from(&Reacquisition::default());
        assert_eq!(RawReacquisition { enable: 0, ..ours }, core);
        // SAFETY: returns a value.
        let core = unsafe { lidarloc_default_global_config() };
        assert_eq!(RawGlobalConfig::from(&GlobalSearch::default()), core);
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn inverts_a_positive_definite_matrix() {
        let mut a = [[0.0; 6]; 6];
        for (i, row) in a.iter_mut().enumerate() {
            for (j, v) in row.iter_mut().enumerate() {
                *v = if i == j { 4.0 + i as f64 } else { 0.5 };
            }
        }
        let inv = inverse_spd(&a).expect("positive definite");
        for i in 0..6 {
            for j in 0..6 {
                let p: f64 = (0..6).map(|k| a[i][k] * inv[k][j]).sum();
                assert!((p - if i == j { 1.0 } else { 0.0 }).abs() < 1e-12);
            }
        }
        a[2][2] = -1.0;
        assert!(inverse_spd(&a).is_none());
    }

    #[test]
    fn unavailable_without_the_core() {
        if !available() {
            assert!(matches!(
                LidarLoc::new(&LidarLocConfig::default()),
                Err(LidarLocError::Unavailable)
            ));
        }
    }
}
