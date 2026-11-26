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
            Ok(Alignment {
                target_ready: raw.target_ready != 0,
                converged: raw.has_converged != 0,
                iterations: u32::try_from(raw.iterations).ok(),
                pose: from_row_major(&raw.pose),
                fitness_score: raw.fitness_score,
                inlier_ratio: raw.inlier_ratio,
                seconds: raw.alignment_time_sec,
                source_points: raw.source_points,
                localizability: (raw.localizability_valid != 0).then_some(Localizability {
                    eigenvalues: raw.eigenvalues,
                    weakest_direction: raw.weakest_eigenvector,
                    weak_ratio: raw.weak_ratio,
                    correspondences: raw.correspondence_count,
                }),
            })
        }
        #[cfg(not(lidarloc))]
        {
            let _ = (xyz, guess, from_row_major(&[0.0; 16]));
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
