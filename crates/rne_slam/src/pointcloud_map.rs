//! Localization of a level 3D LiDAR scan in a prior point-cloud map.
//!
//! [`PointCloudMap`] holds map points with their surface normals (for
//! example sampled from a city model's triangles, whose normals are exact)
//! behind a voxel index. [`PointCloudMap::match_scan`] aligns a scan taken
//! by a LiDAR held level (a gimballed or gravity-compensated sensor, or one
//! whose roll and pitch are corrected beforehand) by point-to-plane
//! Gauss–Newton over four degrees of freedom — position and yaw — with each
//! residual weighted by a Huber kernel, so returns off things that are not
//! in the map (people, vehicles, rain) lose their pull.
//!
//! The match reports its planar covariance from the inverse of the
//! Gauss–Newton information: a scan along a street whose facades are flat
//! constrains the position across the street but hardly along it, and the
//! covariance says so. Unlike a planar slice, a 3D scan also sees the
//! facades above the people on the sidewalk, setbacks, eaves and the
//! changing heights of the buildings along the street.

use crate::point_to_plane::VoxelPointIndex;
use rne_math::Vec3;
use rne_nav::Pose2d;
use serde::{Deserialize, Serialize};

/// Errors raised when building a [`PointCloudMap`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum PointCloudMapError {
    /// There were no points.
    #[error("the point-cloud map is empty")]
    Empty,
    /// Points and normals differ in number.
    #[error("the map has a different number of points and normals")]
    NormalLengthMismatch,
    /// A point, normal or the voxel size was not finite or a normal was zero.
    #[error("the map has a non-finite point, a zero normal or a bad voxel size")]
    NonFinite,
}

/// Tuning of [`PointCloudMap::match_scan`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PointCloudMatchConfig {
    /// Gauss–Newton iterations at most.
    pub max_iterations: usize,
    /// Scan points farther than this from their nearest map point are not
    /// used, meters.
    pub max_correspondence_distance_m: f64,
    /// Huber kernel threshold on the point-to-plane residual, meters.
    pub huber_m: f64,
    /// Fewest used points for a match.
    pub min_inliers: usize,
    /// Least share of the scan's points that must be used for a match.
    pub min_inlier_fraction: f64,
    /// The iteration stops once the step is below this, meters (and
    /// radians).
    pub step_tolerance: f64,
    /// Noise of one point-to-plane residual for the covariance, meters.
    /// Neighbouring points of a scan are not independent (the map and the
    /// surfaces have errors that span many of them), so this is larger than
    /// the range noise.
    pub point_sigma_m: f64,
    /// Floor on each planar position variance, m².
    pub min_position_variance_m2: f64,
    /// Cap on each planar position variance (a direction the scan does not
    /// constrain), m².
    pub max_position_variance_m2: f64,
    /// Floor on the yaw variance, rad².
    pub min_yaw_variance_rad2: f64,
}

impl Default for PointCloudMatchConfig {
    fn default() -> Self {
        Self {
            max_iterations: 20,
            max_correspondence_distance_m: 1.0,
            huber_m: 0.1,
            min_inliers: 50,
            min_inlier_fraction: 0.3,
            step_tolerance: 1.0e-4,
            point_sigma_m: 0.1,
            min_position_variance_m2: 4.0e-4,
            max_position_variance_m2: 100.0,
            min_yaw_variance_rad2: 1.0e-5,
        }
    }
}

/// A scan's pose in the map.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PointCloudMatch {
    /// Planar pose of the sensor.
    pub pose: Pose2d,
    /// Height of the sensor, meters.
    pub z_m: f64,
    /// Planar position covariance, m².
    pub position_covariance_m2: [[f64; 2]; 2],
    /// Yaw variance, rad².
    pub yaw_variance_rad2: f64,
    /// Root-mean-square point-to-plane residual of the used points, meters.
    pub rmse_m: f64,
    /// Share of the scan's points that were used.
    pub inlier_fraction: f64,
    /// Whether the iteration converged.
    pub converged: bool,
}

/// Map points with normals, indexed for nearest-neighbour queries.
#[derive(Clone, Debug)]
pub struct PointCloudMap {
    index: VoxelPointIndex,
    normals: Vec<Vec3>,
}

impl PointCloudMap {
    /// Builds a map from points and their unit (or any non-zero) normals.
    pub fn new(
        points: &[Vec3],
        normals: &[Vec3],
        voxel_size_m: f64,
    ) -> Result<Self, PointCloudMapError> {
        if points.is_empty() {
            return Err(PointCloudMapError::Empty);
        }
        if points.len() != normals.len() {
            return Err(PointCloudMapError::NormalLengthMismatch);
        }
        if !(voxel_size_m.is_finite() && voxel_size_m > 0.0)
            || points.iter().any(|p| !p.is_finite())
            || normals
                .iter()
                .any(|n| !n.is_finite() || n.length() < 1.0e-12)
        {
            return Err(PointCloudMapError::NonFinite);
        }
        Ok(Self {
            index: VoxelPointIndex::new(points, voxel_size_m),
            normals: normals.iter().map(|n| *n / n.length()).collect(),
        })
    }

    /// Number of map points.
    pub fn len(&self) -> usize {
        self.index.len()
    }

    /// Whether the map has no points (never, once built).
    pub fn is_empty(&self) -> bool {
        self.index.is_empty()
    }

    /// Aligns a scan, given in the level sensor frame (x forward, z up),
    /// starting from the sensor's planar pose `initial` at height `z_m`.
    /// Returns `None` when too few points find the map.
    pub fn match_scan(
        &self,
        points_sensor: &[Vec3],
        initial: Pose2d,
        z_m: f64,
        config: &PointCloudMatchConfig,
    ) -> Option<PointCloudMatch> {
        if points_sensor.is_empty() || !initial.x_m.is_finite() || !z_m.is_finite() {
            return None;
        }
        // State: x, y, z, yaw of the sensor.
        let mut state = [initial.x_m, initial.y_m, z_m, initial.yaw_rad];
        let mut converged = false;
        let mut last = None;
        for _ in 0..config.max_iterations {
            let normal = self.normal_equations(points_sensor, state, config)?;
            let (h, b, inliers, rmse) = normal;
            last = Some((h, inliers, rmse));
            // Levenberg damping relative to the information's scale keeps a
            // direction the scan does not constrain where it started.
            let scale = (0..4).map(|i| h[i][i]).fold(0.0, f64::max);
            let mut damped = h;
            for (i, row) in damped.iter_mut().enumerate() {
                row[i] += 1.0e-6 * scale + 1.0e-9;
            }
            let delta = solve4(damped, b)?;
            for (s, d) in state.iter_mut().zip(delta) {
                *s += d;
            }
            if delta.iter().all(|d| d.abs() <= config.step_tolerance) {
                converged = true;
                break;
            }
        }
        let (h, inliers, rmse) = last?;
        let fraction = inliers as f64 / points_sensor.len() as f64;
        if inliers < config.min_inliers || fraction < config.min_inlier_fraction {
            return None;
        }
        // Covariance from the inverse information, in the world frame.
        let sigma2 = config.point_sigma_m * config.point_sigma_m;
        let mut damped = h;
        for (i, row) in damped.iter_mut().enumerate() {
            row[i] += 1.0e-9;
        }
        let columns = [0, 1, 2, 3].map(|k| {
            let mut e = [0.0; 4];
            e[k] = 1.0;
            solve4(damped, e)
        });
        let [Some(c0), Some(c1), _, Some(c3)] = columns else {
            return None;
        };
        let (lo, hi) = (
            config.min_position_variance_m2,
            config.max_position_variance_m2,
        );
        let position_covariance_m2 = clamp_covariance(
            [
                [sigma2 * c0[0], sigma2 * c1[0]],
                [sigma2 * c0[1], sigma2 * c1[1]],
            ],
            lo,
            hi,
        );
        Some(PointCloudMatch {
            pose: Pose2d::new(state[0], state[1], rne_nav::wrap_angle(state[3])),
            z_m: state[2],
            position_covariance_m2,
            yaw_variance_rad2: (sigma2 * c3[3]).max(config.min_yaw_variance_rad2),
            rmse_m: rmse,
            inlier_fraction: fraction,
            converged,
        })
    }

    /// Huber-weighted Gauss–Newton normal equations at `state`: information,
    /// right-hand side, inliers and residual RMS.
    #[allow(clippy::type_complexity)]
    fn normal_equations(
        &self,
        points_sensor: &[Vec3],
        state: [f64; 4],
        config: &PointCloudMatchConfig,
    ) -> Option<([[f64; 4]; 4], [f64; 4], usize, f64)> {
        let (sin, cos) = state[3].sin_cos();
        let mut h = [[0.0; 4]; 4];
        let mut b = [0.0; 4];
        let (mut inliers, mut squared) = (0_usize, 0.0);
        for p in points_sensor {
            // The rotated point relative to the sensor, and in the world.
            let rotated = Vec3::new(cos * p.x - sin * p.y, sin * p.x + cos * p.y, p.z);
            let world = rotated + Vec3::new(state[0], state[1], state[2]);
            let Some((nearest, distance)) = self.index.nearest(world) else {
                continue;
            };
            if distance > config.max_correspondence_distance_m {
                continue;
            }
            let n = self.normals[nearest];
            let residual = n.dot(world - self.index.point(nearest));
            let weight = if residual.abs() <= config.huber_m {
                1.0
            } else {
                config.huber_m / residual.abs()
            };
            // d residual / d (x, y, z, yaw).
            let jacobian = [n.x, n.y, n.z, n.x * -rotated.y + n.y * rotated.x];
            for row in 0..4 {
                for col in 0..4 {
                    h[row][col] += weight * jacobian[row] * jacobian[col];
                }
                b[row] -= weight * jacobian[row] * residual;
            }
            inliers += 1;
            squared += residual * residual;
        }
        (inliers > 0).then(|| (h, b, inliers, (squared / inliers as f64).sqrt()))
    }
}

/// Clamps a symmetric 2x2 covariance's eigenvalues to `[lo, hi]`.
fn clamp_covariance(c: [[f64; 2]; 2], lo: f64, hi: f64) -> [[f64; 2]; 2] {
    let [[a, b0], [b1, d]] = c;
    let b = 0.5 * (b0 + b1);
    let mean = 0.5 * (a + d);
    let spread = (0.25 * (a - d) * (a - d) + b * b).sqrt();
    let (l1, l2) = ((mean + spread).clamp(lo, hi), (mean - spread).clamp(lo, hi));
    let angle = 0.5 * (2.0 * b).atan2(a - d);
    let (s, c) = angle.sin_cos();
    [
        [l1 * c * c + l2 * s * s, (l1 - l2) * s * c],
        [(l1 - l2) * s * c, l1 * s * s + l2 * c * c],
    ]
}

/// Solves a 4x4 linear system by Gaussian elimination with partial pivoting.
fn solve4(mut a: [[f64; 4]; 4], mut b: [f64; 4]) -> Option<[f64; 4]> {
    for col in 0..4 {
        let pivot = (col..4).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[pivot][col].abs() < 1.0e-15 {
            return None;
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        for row in col + 1..4 {
            let f = a[row][col] / a[col][col];
            for k in col..4 {
                a[row][k] -= f * a[col][k];
            }
            b[row] -= f * b[col];
        }
    }
    let mut x = [0.0; 4];
    for row in (0..4).rev() {
        let sum: f64 = (row + 1..4).map(|k| a[row][k] * x[k]).sum();
        x[row] = (b[row] - sum) / a[row][row];
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A street 12 m wide along x with flat facades 15 m tall, a ground, and
    /// (optionally) a building that stands 2 m out into the street beyond
    /// x = 10 m on the north side.
    fn street(corner: bool) -> (Vec<Vec3>, Vec<Vec3>) {
        let (mut points, mut normals) = (Vec::new(), Vec::new());
        let step = 0.25;
        let mut x = -40.0;
        while x <= 40.0 {
            let mut z = 0.0;
            while z <= 15.0 {
                for (y, ny) in [(6.0, -1.0), (-6.0, 1.0)] {
                    // The corner: the north facade steps out 2 m beyond x = 10.
                    let y = if corner && y > 0.0 && x > 10.0 {
                        4.0
                    } else {
                        y
                    };
                    points.push(Vec3::new(x, y, z));
                    normals.push(Vec3::new(0.0, ny, 0.0));
                }
                z += step;
            }
            let mut y = -6.0;
            while y <= 6.0 {
                points.push(Vec3::new(x, y, 0.0));
                normals.push(Vec3::Z);
                y += step;
            }
            x += step;
        }
        if corner {
            let mut z = 0.0;
            while z <= 15.0 {
                let mut y = 4.0;
                while y <= 6.0 {
                    points.push(Vec3::new(10.0, y, z));
                    normals.push(Vec3::new(-1.0, 0.0, 0.0));
                    y += step;
                }
                z += step;
            }
        }
        (points, normals)
    }

    /// A scan of the same street from `(x, y, z)` with yaw 0: rings at
    /// -15..15 degrees, rays every 2 degrees, hitting the planes analytically.
    fn scan(corner: bool, x0: f64, y0: f64, z0: f64) -> Vec<Vec3> {
        let mut out = Vec::new();
        for ring in 0..16 {
            let el = (-15.0 + 2.0 * f64::from(ring)).to_radians();
            for k in 0..180 {
                let az = (2.0 * f64::from(k)).to_radians();
                let d = Vec3::new(el.cos() * az.cos(), el.cos() * az.sin(), el.sin());
                let mut best = f64::INFINITY;
                // Facades.
                for y in [6.0, -6.0, 4.0] {
                    if d.y.abs() > 1e-9 {
                        let t = (y - y0) / d.y;
                        let px = x0 + t * d.x;
                        let on = if y == 4.0 {
                            corner && px > 10.0
                        } else if y == 6.0 {
                            !(corner && px > 10.0)
                        } else {
                            true
                        };
                        if t > 0.0 && on && (0.0..=15.0).contains(&(z0 + t * d.z)) {
                            best = best.min(t);
                        }
                    }
                }
                if corner && d.x.abs() > 1e-9 {
                    let t = (10.0 - x0) / d.x;
                    let py = y0 + t * d.y;
                    if t > 0.0
                        && (4.0..=6.0).contains(&py)
                        && (0.0..=15.0).contains(&(z0 + t * d.z))
                    {
                        best = best.min(t);
                    }
                }
                if d.z < 0.0 {
                    best = best.min(-z0 / d.z);
                }
                if best < 40.0 {
                    out.push(d * best);
                }
            }
        }
        out
    }

    #[test]
    fn a_scan_is_found_from_a_nearby_start() {
        let (points, normals) = street(true);
        let map = PointCloudMap::new(&points, &normals, 1.0).unwrap();
        let points = scan(true, 2.0, -1.0, 0.6);
        let m = map
            .match_scan(
                &points,
                Pose2d::new(2.4, -0.7, 0.03),
                0.7,
                &PointCloudMatchConfig::default(),
            )
            .unwrap();
        assert!(m.converged);
        assert!((m.pose.x_m - 2.0).abs() < 0.05, "{m:?}");
        assert!((m.pose.y_m + 1.0).abs() < 0.05, "{m:?}");
        assert!(m.pose.yaw_rad.abs() < 0.005, "{m:?}");
        assert!((m.z_m - 0.6).abs() < 0.05, "{m:?}");
        assert!(m.inlier_fraction > 0.9, "{m:?}");
    }

    #[test]
    fn a_featureless_street_is_loose_along_it_and_tight_across_it() {
        let config = PointCloudMatchConfig::default();
        let (points, normals) = street(false);
        let map = PointCloudMap::new(&points, &normals, 1.0).unwrap();
        let m = map
            .match_scan(
                &scan(false, 0.0, 0.0, 0.6),
                Pose2d::new(0.0, 0.2, 0.0),
                0.6,
                &config,
            )
            .unwrap();
        let [[xx, _], [_, yy]] = m.position_covariance_m2;
        assert!(xx >= 0.99 * config.max_position_variance_m2, "{m:?}");
        assert!(yy < 1.0e-3, "{m:?}");
        // The corner pins it along the street too.
        let (points, normals) = street(true);
        let map = PointCloudMap::new(&points, &normals, 1.0).unwrap();
        let m = map
            .match_scan(
                &scan(true, 0.0, 0.0, 0.6),
                Pose2d::new(0.2, 0.2, 0.0),
                0.6,
                &config,
            )
            .unwrap();
        let [[xx, _], [_, yy]] = m.position_covariance_m2;
        assert!(xx < 1.0e-2 && yy < 1.0e-3, "{m:?}");
        assert!((m.pose.x_m).abs() < 0.05, "{m:?}");
    }

    #[test]
    fn people_in_the_scan_do_not_pull_the_match() {
        let (points, normals) = street(true);
        let map = PointCloudMap::new(&points, &normals, 1.0).unwrap();
        let mut points = scan(true, 2.0, -1.0, 0.6);
        // A crowd standing 1.5-3 m around the sensor, 1.7 m tall.
        for k in 0..12 {
            let a = f64::from(k) * 0.5;
            for z in 0..17 {
                points.push(Vec3::new(
                    (1.5 + 0.1 * f64::from(k)) * a.cos(),
                    (1.5 + 0.1 * f64::from(k)) * a.sin(),
                    -0.6 + 0.1 * f64::from(z),
                ));
            }
        }
        let m = map
            .match_scan(
                &points,
                Pose2d::new(2.3, -0.8, 0.0),
                0.6,
                &PointCloudMatchConfig::default(),
            )
            .unwrap();
        assert!((m.pose.x_m - 2.0).abs() < 0.05, "{m:?}");
        assert!((m.pose.y_m + 1.0).abs() < 0.05, "{m:?}");
    }

    #[test]
    fn bad_maps_are_refused() {
        assert_eq!(
            PointCloudMap::new(&[], &[], 1.0).unwrap_err(),
            PointCloudMapError::Empty
        );
        assert_eq!(
            PointCloudMap::new(&[Vec3::ZERO], &[], 1.0).unwrap_err(),
            PointCloudMapError::NormalLengthMismatch
        );
        assert_eq!(
            PointCloudMap::new(&[Vec3::ZERO], &[Vec3::ZERO], 1.0).unwrap_err(),
            PointCloudMapError::NonFinite
        );
    }
}
