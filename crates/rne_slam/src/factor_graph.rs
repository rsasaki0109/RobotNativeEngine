//! A sliding-window factor graph that fuses wheel/gyro odometry, GNSS
//! position and heading fixes, and LiDAR scan matches against a prior map.
//!
//! Every GNSS epoch and LiDAR scan adds a pose node `(x, y, θ)`; consecutive
//! nodes are linked by the odometry measured between them. GNSS fixes and
//! headings are unary factors under a Cauchy kernel, so a wrong RTK fix that
//! disagrees with the odometry and the LiDAR is down-weighted instead of
//! gated by a single threshold; LiDAR matches are unary factors with the
//! curvature-based covariance of [`match_position_covariance`] (a match
//! along a corridor constrains only the cross-track position) under a Huber
//! kernel. The window is solved by iteratively reweighted Gauss–Newton on
//! its block-tridiagonal normal equations, and the oldest node is
//! marginalized into a Gaussian prior on the next one.
//!
//! Unlike [`crate::HybridLocalizer`], nothing switches between sources: all
//! evidence of the last seconds is weighed together.
//! [`FactorGraphLocalizer::source`] only reports which absolute factors are
//! currently carrying weight.

use crate::hybrid::{AbsoluteHeadingFix, AbsolutePositionFix, FixKind, LocalizationSource};
use crate::likelihood::LikelihoodField;
use crate::localization::{Amcl, AmclConfig};
use crate::pointcloud_map::PointCloudMatch;
use crate::scan_match::{match_position_covariance, scan_points_2d, ScanMatchConfig, ScanMatcher};
use rne_nav::{wrap_angle, GridError, LaserScan2d, OccupancyGrid, Pose2d};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

type Mat3 = [[f64; 3]; 3];
type Vec3a = [f64; 3];

/// Factor-graph localizer settings.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct FactorGraphConfig {
    /// Pose nodes kept in the window.
    pub window: usize,
    /// Gauss–Newton iterations per update.
    pub iterations: usize,
    /// Odometry translation noise: a constant plus a share of the distance.
    pub odometry_std_m: f64,
    /// Odometry translation noise per meter travelled.
    pub odometry_std_per_m: f64,
    /// Odometry heading noise: a constant, a share of the turn, and a gyro
    /// bias random walk per second.
    pub odometry_std_rad: f64,
    /// Odometry heading noise per radian turned.
    pub odometry_std_per_rad: f64,
    /// Gyro heading drift per second.
    pub gyro_drift_rad_per_s: f64,
    /// Floors on the reported GNSS sigma per solution kind, meters.
    pub gnss_floor_fixed_m: f64,
    /// Floor for RTK float solutions, meters.
    pub gnss_floor_float_m: f64,
    /// Floor for single-point solutions, meters.
    pub gnss_floor_single_m: f64,
    /// Cauchy kernel scale for GNSS factors, in sigmas.
    pub gnss_cauchy_scale: f64,
    /// Fixes with fewer satellites, a worse HDOP or a failed RAIM test are
    /// not added.
    pub gnss_min_satellites: usize,
    /// Maximum HDOP of an added fix.
    pub gnss_max_hdop: f64,
    /// Isotropic variance of a well-constrained LiDAR match, m².
    pub lidar_position_variance_m2: f64,
    /// Variance cap of a degenerate LiDAR direction, m².
    pub lidar_max_variance_m2: f64,
    /// Variance of a LiDAR match's yaw, rad².
    pub lidar_yaw_variance_rad2: f64,
    /// Huber kernel scale for LiDAR factors, in sigmas.
    pub lidar_huber_scale: f64,
    /// Minimum scan-match score for a LiDAR factor.
    pub lidar_min_score: f64,
    /// Scan-match search half-window, meters.
    pub lidar_window_m: f64,
    /// Beams used for matching.
    pub max_beams: usize,
    /// The particle filter run alongside the graph (its likelihood field is
    /// also the scan matcher's). It tracks the scans with several hypotheses
    /// and seeds a second scan match, so a graph that slid along a
    /// featureless street can be pulled back.
    pub amcl: AmclConfig,
    /// Initial pose uncertainty, meters and radians.
    pub initial_std_m: f64,
    /// Initial heading uncertainty, radians.
    pub initial_std_rad: f64,
    /// A source counts as in use while a factor of it carried at least half
    /// weight within this many seconds.
    pub source_hold_s: f64,
    /// After this many consecutive RTK fixed fixes are rejected by the
    /// kernel while agreeing among themselves on the offset (within
    /// `gnss_reanchor_spread_m`), the window is shifted onto them: the
    /// estimate, not the receiver, is what is wrong.
    pub gnss_reanchor_fixes: usize,
    /// Spread of the rejected fixes' offsets allowed for re-anchoring,
    /// meters (or a tenth of the offset, if larger).
    pub gnss_reanchor_spread_m: f64,
    /// The particle filter's match is preferred unless the graph-seeded one
    /// scores higher by more than this.
    pub lidar_seed_score_margin: f64,
    /// A preferred particle-filter match farther than this from the graph
    /// for `lidar_reanchor_scans` consecutive scans shifts the window onto
    /// it, meters.
    pub lidar_reanchor_m: f64,
    /// Consecutive scans of such disagreement before re-anchoring.
    pub lidar_reanchor_scans: usize,
}

impl Default for FactorGraphConfig {
    fn default() -> Self {
        Self {
            window: 60,
            iterations: 3,
            odometry_std_m: 0.01,
            odometry_std_per_m: 0.02,
            odometry_std_rad: 0.002,
            odometry_std_per_rad: 0.02,
            gyro_drift_rad_per_s: 0.002,
            gnss_floor_fixed_m: 0.03,
            gnss_floor_float_m: 0.5,
            gnss_floor_single_m: 3.0,
            gnss_cauchy_scale: 2.0,
            gnss_min_satellites: 5,
            gnss_max_hdop: 5.0,
            lidar_position_variance_m2: 0.01,
            lidar_max_variance_m2: 100.0,
            lidar_yaw_variance_rad2: 0.0025,
            lidar_huber_scale: 2.0,
            lidar_min_score: 0.35,
            lidar_window_m: 1.0,
            max_beams: 180,
            amcl: AmclConfig::default(),
            initial_std_m: 0.5,
            initial_std_rad: 0.05,
            source_hold_s: 1.0,
            gnss_reanchor_fixes: 15,
            gnss_reanchor_spread_m: 0.5,
            lidar_seed_score_margin: 0.02,
            lidar_reanchor_m: 2.0,
            lidar_reanchor_scans: 10,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kernel {
    Cauchy(u8),
    Huber(u8),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Origin {
    Gnss,
    Lidar,
}

#[derive(Clone, Copy, Debug)]
enum Measurement {
    Position { x: f64, y: f64, info: [[f64; 2]; 2] },
    Yaw { yaw: f64, info: f64 },
}

#[derive(Clone, Copy, Debug)]
struct Unary {
    node: u64,
    measurement: Measurement,
    origin: Origin,
    scale: f64,
    kernel: Kernel,
    weight: f64,
    time_s: f64,
}

#[derive(Clone, Copy, Debug)]
struct Node {
    id: u64,
    pose: Vec3a,
    /// Odometry from the previous node, in its frame, and its information.
    odometry: Option<(Vec3a, Mat3)>,
}

/// Odometry accumulated since the last node.
#[derive(Clone, Copy, Debug, Default)]
struct Accumulated {
    delta: Vec3a,
    distance_m: f64,
    turn_rad: f64,
    time_s: f64,
}

/// Errors from the factor-graph localizer.
#[derive(Debug, thiserror::Error)]
pub enum FactorGraphError {
    /// The prior map could not be turned into a likelihood field.
    #[error(transparent)]
    Map(#[from] GridError),
}

/// A sliding-window GNSS / LiDAR / odometry factor graph.
#[derive(Clone, Debug)]
pub struct FactorGraphLocalizer {
    config: FactorGraphConfig,
    map: OccupancyGrid,
    field: LikelihoodField,
    amcl: Amcl,
    lidar_reanchor_streak: usize,
    nodes: VecDeque<Node>,
    unaries: Vec<Unary>,
    /// Prior on the oldest node: node id, mean and information.
    prior: (u64, Vec3a, Mat3),
    next_id: u64,
    accumulated: Accumulated,
    last_gnss_s: f64,
    last_lidar_s: f64,
    time_s: f64,
    /// Offsets (fix − estimate) of consecutive rejected RTK fixed fixes.
    rejected_offsets: Vec<(f64, f64)>,
}

impl FactorGraphLocalizer {
    /// Starts at `initial_pose` with the configured initial uncertainty.
    ///
    /// # Errors
    ///
    /// Returns an error when the map cannot be turned into a likelihood
    /// field.
    pub fn new(
        map: &OccupancyGrid,
        initial_pose: Pose2d,
        config: FactorGraphConfig,
    ) -> Result<Self, FactorGraphError> {
        let field = LikelihoodField::from_occupancy(map, &config.amcl.likelihood)?;
        let amcl = Amcl::new(map, initial_pose, config.amcl)?;
        let pose = [initial_pose.x_m, initial_pose.y_m, initial_pose.yaw_rad];
        let (a, b) = (
            1.0 / (config.initial_std_m * config.initial_std_m),
            1.0 / (config.initial_std_rad * config.initial_std_rad),
        );
        Ok(Self {
            config,
            map: map.clone(),
            field,
            amcl,
            lidar_reanchor_streak: 0,
            nodes: VecDeque::from([Node {
                id: 0,
                pose,
                odometry: None,
            }]),
            unaries: Vec::new(),
            prior: (0, pose, [[a, 0.0, 0.0], [0.0, a, 0.0], [0.0, 0.0, b]]),
            next_id: 1,
            accumulated: Accumulated::default(),
            last_gnss_s: f64::NEG_INFINITY,
            last_lidar_s: f64::NEG_INFINITY,
            time_s: 0.0,
            rejected_offsets: Vec::new(),
        })
    }

    /// The current pose: the newest node plus the odometry since.
    pub fn pose(&self) -> Pose2d {
        let last = self.nodes.back().expect("a node").pose;
        let p = compose(last, self.accumulated.delta);
        Pose2d::new(p[0], p[1], p[2])
    }

    /// Which absolute factors carried weight within `source_hold_s`.
    pub fn source(&self) -> LocalizationSource {
        let hold = self.config.source_hold_s;
        if self.time_s - self.last_gnss_s <= hold {
            LocalizationSource::Gnss
        } else if self.time_s - self.last_lidar_s <= hold {
            LocalizationSource::Lidar
        } else {
            LocalizationSource::DeadReckoning
        }
    }

    /// Integrates wheel speed and yaw rate over `dt_s`.
    pub fn predict(&mut self, dt_s: f64, linear_m_s: f64, angular_rad_s: f64) {
        let a = &mut self.accumulated;
        let mid = a.delta[2] + 0.5 * angular_rad_s * dt_s;
        a.delta[0] += linear_m_s * dt_s * mid.cos();
        a.delta[1] += linear_m_s * dt_s * mid.sin();
        a.delta[2] = wrap_angle(a.delta[2] + angular_rad_s * dt_s);
        a.distance_m += (linear_m_s * dt_s).abs();
        a.turn_rad += (angular_rad_s * dt_s).abs();
        a.time_s += dt_s;
        self.time_s += dt_s;
    }

    /// Adds a GNSS position fix (`None` for an epoch without one).
    pub fn update_gnss(&mut self, fix: Option<&AbsolutePositionFix>, time_s: f64) {
        self.time_s = self.time_s.max(time_s);
        let Some(fix) = fix else { return };
        let c = self.config;
        if !fix.consistent
            || fix.satellites_used < c.gnss_min_satellites
            || fix.hdop > c.gnss_max_hdop
            || !fix.x_m.is_finite()
            || !fix.y_m.is_finite()
        {
            return;
        }
        let floor = match fix.kind {
            FixKind::Fixed => c.gnss_floor_fixed_m,
            FixKind::Float => c.gnss_floor_float_m,
            FixKind::Single => c.gnss_floor_single_m,
        };
        let sigma = fix.horizontal_std_m.max(floor);
        let info = 1.0 / (sigma * sigma);
        let node = self.add_node(time_s);
        self.unaries.push(Unary {
            node,
            measurement: Measurement::Position {
                x: fix.x_m,
                y: fix.y_m,
                info: [[info, 0.0], [0.0, info]],
            },
            origin: Origin::Gnss,
            scale: c.gnss_cauchy_scale,
            kernel: Kernel::Cauchy(0),
            weight: 1.0,
            time_s,
        });
        self.optimize();
        self.check_reanchor(fix, node);
    }

    /// Shifts the window onto a run of consistent but rejected RTK fixes.
    fn check_reanchor(&mut self, fix: &AbsolutePositionFix, node: u64) {
        let weight = self
            .unaries
            .iter()
            .rev()
            .find(|u| u.node == node && u.origin == Origin::Gnss)
            .map_or(1.0, |u| u.weight);
        let Some(index) = self.index_of(node) else {
            return;
        };
        if fix.kind != FixKind::Fixed || weight >= 0.1 {
            self.rejected_offsets.clear();
            return;
        }
        let pose = self.nodes[index].pose;
        self.rejected_offsets
            .push((fix.x_m - pose[0], fix.y_m - pose[1]));
        let n = self.rejected_offsets.len();
        if n < self.config.gnss_reanchor_fixes {
            return;
        }
        let (mx, my) = self
            .rejected_offsets
            .iter()
            .fold((0.0, 0.0), |(a, b), (x, y)| {
                (a + x / n as f64, b + y / n as f64)
            });
        let spread = self
            .rejected_offsets
            .iter()
            .map(|(x, y)| (x - mx).hypot(y - my))
            .fold(0.0, f64::max);
        // The fixes must agree on the offset to within the spread, or a tenth
        // of the offset when the estimate is far off (a lost estimate keeps
        // moving under the other factors).
        if spread > self.config.gnss_reanchor_spread_m.max(0.1 * mx.hypot(my)) {
            self.rejected_offsets.remove(0);
            return;
        }
        // The GNSS position factors now agree; the rest described the old,
        // wrong placement.
        let kept: Vec<Unary> = self
            .unaries
            .iter()
            .filter(|u| {
                u.origin == Origin::Gnss && matches!(u.measurement, Measurement::Position { .. })
            })
            .map(|u| Unary { weight: 1.0, ..*u })
            .collect();
        self.shift_window(mx, my);
        self.unaries = kept;
        self.optimize();
    }

    /// Adds a moving-baseline heading to the newest node.
    pub fn update_gnss_heading(&mut self, heading: &AbsoluteHeadingFix) {
        if !heading.yaw_rad.is_finite() || heading.yaw_std_rad <= 0.0 {
            return;
        }
        let node = self.add_node(heading.time_s.max(self.time_s));
        self.unaries.push(Unary {
            node,
            measurement: Measurement::Yaw {
                yaw: heading.yaw_rad,
                info: 1.0 / (heading.yaw_std_rad * heading.yaw_std_rad),
            },
            origin: Origin::Gnss,
            scale: self.config.gnss_cauchy_scale,
            kernel: Kernel::Cauchy(0),
            weight: 1.0,
            time_s: heading.time_s,
        });
        self.optimize();
    }

    /// Updates the particle filter with the scan, matches the scan against
    /// the prior map from the graph's pose and from the particle filter's
    /// estimate, and adds the preferred match as a factor.
    pub fn update_lidar(
        &mut self,
        scan: &LaserScan2d,
        odom_pose: Pose2d,
        sensor_from_base: Pose2d,
        time_s: f64,
    ) {
        self.time_s = self.time_s.max(time_s);
        let c = self.config;
        let update = self.amcl.update(scan, odom_pose, sensor_from_base);
        let points = scan_points_2d(scan, c.max_beams);
        let matcher = ScanMatcher::new(ScanMatchConfig {
            linear_window_m: c.lidar_window_m,
            linear_samples: 9,
            levels: 4,
            ..ScanMatchConfig::default()
        });
        let graph_pose = self.pose();
        let mut matches = [graph_pose, update.estimate].into_iter().map(|seed| {
            matcher
                .match_scan(&self.field, &points, sensor_from_base, seed)
                .filter(|m| m.score >= c.lidar_min_score)
        });
        let (graph_match, particle_match) = (matches.next().flatten(), matches.next().flatten());
        let apart = |p: Pose2d| (p.x_m - graph_pose.x_m).hypot(p.y_m - graph_pose.y_m);
        let chosen = match (graph_match, particle_match) {
            (Some(g), Some(p)) if p.score + c.lidar_seed_score_margin >= g.score => Some((p, true)),
            (Some(g), _) => Some((g, false)),
            (None, p) => p.map(|p| (p, true)),
        };
        let Some((matched, from_particles)) = chosen else {
            return;
        };
        if from_particles && apart(matched.pose) > c.lidar_reanchor_m {
            // The graph and the particle filter disagree by metres: wait for
            // the disagreement to persist, then shift the window onto it.
            self.lidar_reanchor_streak += 1;
            if self.lidar_reanchor_streak < c.lidar_reanchor_scans {
                return;
            }
            self.lidar_reanchor_streak = 0;
            let (dx, dy) = (
                matched.pose.x_m - graph_pose.x_m,
                matched.pose.y_m - graph_pose.y_m,
            );
            self.shift_window(dx, dy);
        } else {
            self.lidar_reanchor_streak = 0;
            if !from_particles && apart(update.estimate) > c.lidar_reanchor_m {
                // The particle filter wandered off while the graph's match
                // fits better: re-seed it there.
                let config = AmclConfig {
                    initial_translation_std_m: 0.15,
                    initial_rotation_std_rad: 0.05,
                    ..c.amcl
                };
                if let Ok(mut amcl) = Amcl::new(&self.map, matched.pose, config) {
                    amcl.update(scan, odom_pose, sensor_from_base);
                    self.amcl = amcl;
                }
            }
        }
        let covariance = match_position_covariance(
            &self.field,
            &points,
            sensor_from_base,
            matched.pose,
            c.lidar_position_variance_m2,
            c.lidar_max_variance_m2,
        );
        let node = self.add_node(time_s);
        let info = invert2(covariance);
        self.unaries.push(Unary {
            node,
            measurement: Measurement::Position {
                x: matched.pose.x_m,
                y: matched.pose.y_m,
                info,
            },
            origin: Origin::Lidar,
            scale: c.lidar_huber_scale,
            kernel: Kernel::Huber(0),
            weight: 1.0,
            time_s,
        });
        self.unaries.push(Unary {
            node,
            measurement: Measurement::Yaw {
                yaw: matched.pose.yaw_rad,
                info: 1.0 / c.lidar_yaw_variance_rad2,
            },
            origin: Origin::Lidar,
            scale: c.lidar_huber_scale,
            kernel: Kernel::Huber(1),
            weight: 1.0,
            time_s,
        });
        self.optimize();
    }

    /// Adds a 3D LiDAR scan's match in a point-cloud map as a position and a
    /// yaw factor (Huber kernel), in place of [`Self::update_lidar`]. The
    /// caller matches the scan from [`Self::pose`].
    pub fn update_lidar_match(&mut self, matched: &PointCloudMatch, time_s: f64) {
        self.time_s = self.time_s.max(time_s);
        let c = self.config;
        let node = self.add_node(time_s);
        self.unaries.push(Unary {
            node,
            measurement: Measurement::Position {
                x: matched.pose.x_m,
                y: matched.pose.y_m,
                info: invert2(matched.position_covariance_m2),
            },
            origin: Origin::Lidar,
            scale: c.lidar_huber_scale,
            kernel: Kernel::Huber(0),
            weight: 1.0,
            time_s,
        });
        self.unaries.push(Unary {
            node,
            measurement: Measurement::Yaw {
                yaw: matched.pose.yaw_rad,
                info: 1.0 / matched.yaw_variance_rad2.max(1.0e-12),
            },
            origin: Origin::Lidar,
            scale: c.lidar_huber_scale,
            kernel: Kernel::Huber(1),
            weight: 1.0,
            time_s,
        });
        self.optimize();
    }

    /// Translates every node and the prior, dropping the absolute factors
    /// that described the old placement.
    fn shift_window(&mut self, dx: f64, dy: f64) {
        for node in &mut self.nodes {
            node.pose[0] += dx;
            node.pose[1] += dy;
        }
        self.prior.1[0] += dx;
        self.prior.1[1] += dy;
        self.unaries.clear();
        self.rejected_offsets.clear();
    }

    /// Number of pose nodes in the window.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Appends a node at the current odometry (or reuses the newest one if no
    /// time passed) and returns its id.
    fn add_node(&mut self, _time_s: f64) -> u64 {
        if self.accumulated.time_s <= 1.0e-9 {
            return self.nodes.back().expect("a node").id;
        }
        let c = self.config;
        let a = self.accumulated;
        let lin = c.odometry_std_m + c.odometry_std_per_m * a.distance_m;
        let ang = c.odometry_std_rad
            + c.odometry_std_per_rad * a.turn_rad
            + c.gyro_drift_rad_per_s * a.time_s;
        let info = diag3(1.0 / (lin * lin), 1.0 / (lin * lin), 1.0 / (ang * ang));
        let pose = compose(self.nodes.back().expect("a node").pose, a.delta);
        let id = self.next_id;
        self.next_id += 1;
        self.nodes.push_back(Node {
            id,
            pose,
            odometry: Some((a.delta, info)),
        });
        self.accumulated = Accumulated::default();
        while self.nodes.len() > self.config.window.max(2) {
            self.marginalize_oldest();
        }
        id
    }

    fn index_of(&self, id: u64) -> Option<usize> {
        let first = self.nodes.front()?.id;
        let index = id.checked_sub(first)? as usize;
        (index < self.nodes.len()).then_some(index)
    }

    /// Accumulates the normal equations: diagonal blocks, upper blocks and
    /// the gradient. Updates the robust weights when `reweight`.
    fn normal_equations(&mut self, reweight: bool) -> (Vec<Mat3>, Vec<Mat3>, Vec<Vec3a>) {
        let n = self.nodes.len();
        let mut diag = vec![[[0.0; 3]; 3]; n];
        let mut upper = vec![[[0.0; 3]; 3]; n.saturating_sub(1)];
        let mut grad = vec![[0.0; 3]; n];
        // Prior on the oldest node.
        let (prior_id, mean, info) = self.prior;
        if let Some(i) = self.index_of(prior_id) {
            let x = self.nodes[i].pose;
            let r = [x[0] - mean[0], x[1] - mean[1], wrap_angle(x[2] - mean[2])];
            add3(&mut diag[i], info);
            add_vec(&mut grad[i], mul3v(info, r));
        }
        // Odometry between consecutive nodes.
        for j in 1..n {
            let (z, info) = self.nodes[j].odometry.expect("odometry");
            let (xi, xj) = (self.nodes[j - 1].pose, self.nodes[j].pose);
            let (s, c) = xi[2].sin_cos();
            let (dx, dy) = (xj[0] - xi[0], xj[1] - xi[1]);
            let e = [
                c * dx + s * dy - z[0],
                -s * dx + c * dy - z[1],
                wrap_angle(xj[2] - xi[2] - z[2]),
            ];
            let ji: Mat3 = [
                [-c, -s, -s * dx + c * dy],
                [s, -c, -c * dx - s * dy],
                [0.0, 0.0, -1.0],
            ];
            let jj: Mat3 = [[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]];
            let (jit, jjt) = (transpose(ji), transpose(jj));
            add3(&mut diag[j - 1], mul3(mul3(jit, info), ji));
            add3(&mut diag[j], mul3(mul3(jjt, info), jj));
            add3(&mut upper[j - 1], mul3(mul3(jit, info), jj));
            let ie = mul3v(info, e);
            add_vec(&mut grad[j - 1], mul3v(jit, ie));
            add_vec(&mut grad[j], mul3v(jjt, ie));
        }
        // Unary factors with robust weights.
        for k in 0..self.unaries.len() {
            let u = self.unaries[k];
            let Some(i) = self.index_of(u.node) else {
                continue;
            };
            let x = self.nodes[i].pose;
            let (block, g, squared) = match u.measurement {
                Measurement::Position { x: mx, y: my, info } => {
                    let r = [x[0] - mx, x[1] - my];
                    let ir = [
                        info[0][0] * r[0] + info[0][1] * r[1],
                        info[1][0] * r[0] + info[1][1] * r[1],
                    ];
                    (
                        [
                            [info[0][0], info[0][1], 0.0],
                            [info[1][0], info[1][1], 0.0],
                            [0.0; 3],
                        ],
                        [ir[0], ir[1], 0.0],
                        r[0] * ir[0] + r[1] * ir[1],
                    )
                }
                Measurement::Yaw { yaw, info } => {
                    let r = wrap_angle(x[2] - yaw);
                    (
                        [[0.0; 3], [0.0; 3], [0.0, 0.0, info]],
                        [0.0, 0.0, info * r],
                        r * r * info,
                    )
                }
            };
            if reweight {
                let s = squared.sqrt();
                self.unaries[k].weight = match u.kernel {
                    Kernel::Cauchy(_) => 1.0 / (1.0 + squared / (u.scale * u.scale)),
                    Kernel::Huber(_) => {
                        if s <= u.scale {
                            1.0
                        } else {
                            u.scale / s
                        }
                    }
                };
            }
            let w = self.unaries[k].weight;
            add3(&mut diag[i], scale3(block, w));
            add_vec(&mut grad[i], [g[0] * w, g[1] * w, g[2] * w]);
        }
        (diag, upper, grad)
    }

    /// Iteratively reweighted Gauss–Newton over the window.
    fn optimize(&mut self) {
        for _ in 0..self.config.iterations {
            let (diag, upper, grad) = self.normal_equations(true);
            let Some(step) = solve_block_tridiagonal(&diag, &upper, &grad) else {
                return;
            };
            for (node, d) in self.nodes.iter_mut().zip(step) {
                node.pose[0] -= d[0];
                node.pose[1] -= d[1];
                node.pose[2] = wrap_angle(node.pose[2] - d[2]);
            }
        }
        // Which absolute sources carry weight now.
        for u in &self.unaries {
            if u.weight >= 0.5 {
                match u.origin {
                    Origin::Gnss => self.last_gnss_s = self.last_gnss_s.max(u.time_s),
                    Origin::Lidar => self.last_lidar_s = self.last_lidar_s.max(u.time_s),
                }
            }
        }
    }

    /// Folds the oldest node into a Gaussian prior on the next one (Schur
    /// complement at the current estimate).
    fn marginalize_oldest(&mut self) {
        let old = self.nodes[0];
        let next = self.nodes[1];
        // Blocks from every factor touching the oldest node.
        let mut h00 = [[0.0; 3]; 3];
        let mut g0 = [0.0; 3];
        let (prior_id, mean, info) = self.prior;
        if prior_id == old.id {
            let x = old.pose;
            let r = [x[0] - mean[0], x[1] - mean[1], wrap_angle(x[2] - mean[2])];
            add3(&mut h00, info);
            add_vec(&mut g0, mul3v(info, r));
        }
        for u in self.unaries.iter().filter(|u| u.node == old.id) {
            let x = old.pose;
            let (block, g) = match u.measurement {
                Measurement::Position { x: mx, y: my, info } => {
                    let r = [x[0] - mx, x[1] - my];
                    (
                        [
                            [info[0][0], info[0][1], 0.0],
                            [info[1][0], info[1][1], 0.0],
                            [0.0; 3],
                        ],
                        [
                            info[0][0] * r[0] + info[0][1] * r[1],
                            info[1][0] * r[0] + info[1][1] * r[1],
                            0.0,
                        ],
                    )
                }
                Measurement::Yaw { yaw, info } => {
                    let r = wrap_angle(x[2] - yaw);
                    ([[0.0; 3], [0.0; 3], [0.0, 0.0, info]], [0.0, 0.0, info * r])
                }
            };
            add3(&mut h00, scale3(block, u.weight));
            add_vec(&mut g0, [g[0] * u.weight, g[1] * u.weight, g[2] * u.weight]);
        }
        let (z, oinfo) = next.odometry.expect("odometry");
        let (xi, xj) = (old.pose, next.pose);
        let (s, c) = xi[2].sin_cos();
        let (dx, dy) = (xj[0] - xi[0], xj[1] - xi[1]);
        let e = [
            c * dx + s * dy - z[0],
            -s * dx + c * dy - z[1],
            wrap_angle(xj[2] - xi[2] - z[2]),
        ];
        let ji: Mat3 = [
            [-c, -s, -s * dx + c * dy],
            [s, -c, -c * dx - s * dy],
            [0.0, 0.0, -1.0],
        ];
        let jj: Mat3 = [[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]];
        let (jit, jjt) = (transpose(ji), transpose(jj));
        add3(&mut h00, mul3(mul3(jit, oinfo), ji));
        let h01 = mul3(mul3(jit, oinfo), jj);
        let h11 = mul3(mul3(jjt, oinfo), jj);
        let ie = mul3v(oinfo, e);
        add_vec(&mut g0, mul3v(jit, ie));
        let g1 = mul3v(jjt, ie);
        // Schur complement.
        let new_prior = invert3(h00).and_then(|h00_inv| {
            let h10 = transpose(h01);
            let lambda = sub3(h11, mul3(mul3(h10, h00_inv), h01));
            let g = sub_vec(g1, mul3v(mul3(h10, h00_inv), g0));
            let lambda = symmetrize(lambda);
            let lambda_inv = invert3(lambda)?;
            let shift = mul3v(lambda_inv, g);
            let x = next.pose;
            Some((
                next.id,
                [
                    x[0] - shift[0],
                    x[1] - shift[1],
                    wrap_angle(x[2] - shift[2]),
                ],
                lambda,
            ))
        });
        self.prior = new_prior.unwrap_or((next.id, next.pose, diag3(1.0, 1.0, 100.0)));
        self.unaries.retain(|u| u.node != old.id);
        self.nodes.pop_front();
        self.nodes[0].odometry = None;
    }
}

fn compose(a: Vec3a, b: Vec3a) -> Vec3a {
    let (s, c) = a[2].sin_cos();
    [
        a[0] + c * b[0] - s * b[1],
        a[1] + s * b[0] + c * b[1],
        wrap_angle(a[2] + b[2]),
    ]
}

fn diag3(a: f64, b: f64, c: f64) -> Mat3 {
    [[a, 0.0, 0.0], [0.0, b, 0.0], [0.0, 0.0, c]]
}

fn transpose(m: Mat3) -> Mat3 {
    let mut t = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            t[i][j] = m[j][i];
        }
    }
    t
}

fn mul3(a: Mat3, b: Mat3) -> Mat3 {
    let mut m = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            m[i][j] = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    m
}

fn mul3v(a: Mat3, v: Vec3a) -> Vec3a {
    [0, 1, 2].map(|i| (0..3).map(|k| a[i][k] * v[k]).sum())
}

fn add3(a: &mut Mat3, b: Mat3) {
    for i in 0..3 {
        for j in 0..3 {
            a[i][j] += b[i][j];
        }
    }
}

fn sub3(a: Mat3, b: Mat3) -> Mat3 {
    let mut m = a;
    for i in 0..3 {
        for j in 0..3 {
            m[i][j] -= b[i][j];
        }
    }
    m
}

fn scale3(a: Mat3, s: f64) -> Mat3 {
    a.map(|row| row.map(|v| v * s))
}

fn symmetrize(a: Mat3) -> Mat3 {
    let mut m = a;
    for i in 0..3 {
        for j in 0..3 {
            m[i][j] = 0.5 * (a[i][j] + a[j][i]);
        }
    }
    m
}

fn add_vec(a: &mut Vec3a, b: Vec3a) {
    for i in 0..3 {
        a[i] += b[i];
    }
}

fn sub_vec(a: Vec3a, b: Vec3a) -> Vec3a {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn invert3(m: Mat3) -> Option<Mat3> {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if !det.is_finite() || det.abs() < 1.0e-300 {
        return None;
    }
    let inv = 1.0 / det;
    Some([
        [
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv,
        ],
        [
            (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv,
        ],
        [
            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv,
        ],
    ])
}

fn invert2(m: [[f64; 2]; 2]) -> [[f64; 2]; 2] {
    let det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
    [
        [m[1][1] / det, -m[0][1] / det],
        [-m[1][0] / det, m[0][0] / det],
    ]
}

/// Solves the block-tridiagonal system `H x = g` (block Thomas algorithm).
fn solve_block_tridiagonal(diag: &[Mat3], upper: &[Mat3], grad: &[Vec3a]) -> Option<Vec<Vec3a>> {
    let n = diag.len();
    let mut d = diag.to_vec();
    let mut g = grad.to_vec();
    let mut d_inv = Vec::with_capacity(n);
    for i in 0..n {
        if i > 0 {
            let lower = transpose(upper[i - 1]);
            let prev_inv: Mat3 = d_inv[i - 1];
            let factor = mul3(lower, prev_inv);
            d[i] = sub3(d[i], mul3(factor, upper[i - 1]));
            g[i] = sub_vec(g[i], mul3v(factor, g[i - 1]));
        }
        d_inv.push(invert3(d[i])?);
    }
    let mut x = vec![[0.0; 3]; n];
    x[n - 1] = mul3v(d_inv[n - 1], g[n - 1]);
    for i in (0..n - 1).rev() {
        x[i] = mul3v(d_inv[i], sub_vec(g[i], mul3v(upper[i], x[i + 1])));
    }
    Some(x)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_nav::{FrameId, GridCoord};
    use std::f64::consts::TAU;

    fn open_map() -> OccupancyGrid {
        let mut grid = OccupancyGrid::new(100, 100, 0.5, Pose2d::new(-25.0, -25.0, 0.0)).unwrap();
        for y in 0..100 {
            for x in 0..100 {
                grid.mark_free(GridCoord { x, y });
            }
        }
        grid
    }

    fn fix(x: f64, y: f64, kind: FixKind, std: f64) -> AbsolutePositionFix {
        AbsolutePositionFix {
            time_s: 0.0,
            x_m: x,
            y_m: y,
            horizontal_std_m: std,
            satellites_used: 10,
            hdop: 1.0,
            consistent: true,
            satellites_excluded: 0,
            kind,
        }
    }

    #[test]
    fn point_cloud_matches_pull_the_graph() {
        let mut graph = FactorGraphLocalizer::new(
            &open_map(),
            Pose2d::new(0.0, 0.0, 0.0),
            FactorGraphConfig::default(),
        )
        .unwrap();
        for step in 0..20 {
            graph.predict(0.1, 1.0, 0.0);
            let t = 0.1 * f64::from(step + 1);
            // The scans say the robot is 0.5 m north of where odometry puts it,
            // and do not constrain it along x.
            let matched = PointCloudMatch {
                pose: Pose2d::new(t, 0.5, 0.0),
                z_m: 0.6,
                position_covariance_m2: [[100.0, 0.0], [0.0, 0.01]],
                yaw_variance_rad2: 1.0e-4,
                rmse_m: 0.02,
                inlier_fraction: 0.9,
                fit_fraction: 0.9,
                converged: true,
            };
            graph.update_lidar_match(&matched, t);
        }
        let pose = graph.pose();
        assert!((pose.y_m - 0.5).abs() < 0.1, "{pose:?}");
        assert!((pose.x_m - 2.0).abs() < 0.2, "{pose:?}");
    }

    #[test]
    fn block_tridiagonal_solver_matches_a_dense_solve() {
        let diag = vec![
            diag3(4.0, 5.0, 6.0),
            diag3(7.0, 8.0, 9.0),
            diag3(5.0, 5.0, 5.0),
        ];
        let upper = vec![
            [[1.0, 0.5, 0.0], [0.0, 1.0, 0.2], [0.1, 0.0, 1.0]],
            [[0.5, 0.0, 0.0], [0.3, 0.5, 0.0], [0.0, 0.1, 0.5]],
        ];
        let grad = vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
        let x = solve_block_tridiagonal(&diag, &upper, &grad).unwrap();
        // Residual H x - g.
        for i in 0..3 {
            let mut r = mul3v(diag[i], x[i]);
            if i > 0 {
                add_vec(&mut r, mul3v(transpose(upper[i - 1]), x[i - 1]));
            }
            if i < 2 {
                add_vec(&mut r, mul3v(upper[i], x[i + 1]));
            }
            for k in 0..3 {
                assert!((r[k] - grad[i][k]).abs() < 1e-9, "{r:?} vs {:?}", grad[i]);
            }
        }
    }

    #[test]
    fn rtk_fixes_and_odometry_track_a_drive_and_a_wrong_fix_is_down_weighted() {
        let mut graph = FactorGraphLocalizer::new(
            &open_map(),
            Pose2d::new(0.0, 0.0, 0.0),
            FactorGraphConfig::default(),
        )
        .unwrap();
        let (dt, v) = (0.05, 1.0);
        let mut t = 0.0;
        for step in 0..400 {
            // Odometry reads 2 % long; the robot really drives at 1 m/s east.
            graph.predict(dt, v * 1.02, 0.0);
            t += dt;
            if step % 4 == 3 {
                let truth_x = t * v;
                // One fix in the middle is 4 m off north (an NLOS "fix").
                let y = if (200..204).contains(&step) { 4.0 } else { 0.0 };
                graph.update_gnss(Some(&fix(truth_x, y, FixKind::Fixed, 0.01)), t);
                let pose = graph.pose();
                assert!(pose.y_m.abs() < 0.1, "step {step}: {pose:?}");
            }
        }
        let pose = graph.pose();
        assert!((pose.x_m - t * v).abs() < 0.05, "{pose:?} vs {t}");
        assert!(graph.node_count() <= FactorGraphConfig::default().window);
        assert_eq!(graph.source(), LocalizationSource::Gnss);
    }

    /// A 4 m wide corridor along x.
    fn corridor() -> OccupancyGrid {
        let mut grid = OccupancyGrid::new(800, 100, 0.1, Pose2d::new(-40.0, -5.0, 0.0)).unwrap();
        for y in 0..100 {
            for x in 0..800 {
                let coord = GridCoord { x, y };
                if grid.grid_to_world(coord).y.abs() >= 2.0 {
                    grid.mark_occupied(coord);
                } else {
                    grid.mark_free(coord);
                }
            }
        }
        grid
    }

    fn corridor_scan(y: f64) -> LaserScan2d {
        let beams = 180;
        LaserScan2d {
            time_s: 0.0,
            frame: FrameId::new("laser"),
            angle_min_rad: 0.0,
            angle_increment_rad: TAU / beams as f64,
            range_min_m: 0.05,
            range_max_m: 12.0,
            ranges_m: (0..beams)
                .map(|b| {
                    let a = TAU * b as f64 / beams as f64;
                    let wall = if a.sin() > 0.0 { 2.0 - y } else { 2.0 + y };
                    let s = a.sin().abs();
                    if s > 0.1 {
                        (wall / s).min(12.0)
                    } else {
                        f64::INFINITY
                    }
                })
                .collect(),
        }
    }

    #[test]
    fn a_corridor_scan_pins_only_the_cross_track_position() {
        let mut graph = FactorGraphLocalizer::new(
            &corridor(),
            Pose2d::new(0.0, 0.0, 0.0),
            FactorGraphConfig::default(),
        )
        .unwrap();
        // Odometry reads 10 % long and 0.3 m to the side; scans see the walls.
        let mut t = 0.0;
        for step in 0..200 {
            graph.predict(0.05, 1.1, 0.0);
            t += 0.05;
            if step == 10 {
                // A sideways kick the odometry did not see.
                graph.nodes.back_mut().unwrap().pose[1] += 0.3;
            }
            if step % 2 == 1 {
                graph.update_lidar(
                    &corridor_scan(0.0),
                    Pose2d::new(1.1 * t, 0.0, 0.0),
                    Pose2d::IDENTITY,
                    t,
                );
            }
        }
        let pose = graph.pose();
        // Cross-track: the walls hold it at y = 0.
        assert!(pose.y_m.abs() < 0.05, "{pose:?}");
        // Along-track: nothing in the scans; the 10 % odometry error remains.
        assert!((pose.x_m - 1.1 * t).abs() < 0.5, "{pose:?} vs {}", 1.1 * t);
        assert_eq!(graph.source(), LocalizationSource::Lidar);
        // An RTK fix then fixes the along-track error.
        for k in 0..20 {
            graph.predict(0.05, 1.1, 0.0);
            t += 0.05;
            if k % 4 == 0 {
                graph.update_gnss(Some(&fix(t, 0.0, FixKind::Fixed, 0.01)), t);
            }
        }
        let pose = graph.pose();
        assert!((pose.x_m - t).abs() < 0.2, "{pose:?} vs {t}");
    }

    #[test]
    fn a_run_of_consistent_rejected_rtk_fixes_re_anchors_a_lost_estimate() {
        let mut graph = FactorGraphLocalizer::new(
            &open_map(),
            Pose2d::new(0.0, 0.0, 0.0),
            FactorGraphConfig::default(),
        )
        .unwrap();
        let mut t = 0.0;
        // Converge on RTK at x = t.
        for step in 0..80 {
            graph.predict(0.05, 1.0, 0.0);
            t += 0.05;
            if step % 4 == 3 {
                graph.update_gnss(Some(&fix(t, 0.0, FixKind::Fixed, 0.01)), t);
            }
        }
        // The estimate is knocked 6 m off, as a long LiDAR stretch that slid
        // could leave it (no GNSS factor left in the window).
        for node in &mut graph.nodes {
            node.pose[1] += 6.0;
        }
        graph.prior.1[1] += 6.0;
        graph.unaries.clear();
        let mut recovered_at = None;
        for step in 0..200 {
            graph.predict(0.05, 1.0, 0.0);
            t += 0.05;
            if step % 4 == 3 {
                graph.update_gnss(Some(&fix(t, 0.0, FixKind::Fixed, 0.01)), t);
                if recovered_at.is_none() && graph.pose().y_m.abs() < 0.1 {
                    recovered_at = Some(step);
                }
            }
        }
        let step = recovered_at.expect("re-anchored");
        // Within a few seconds of fixes (at most the re-anchoring run).
        assert!(
            step <= 4 * FactorGraphConfig::default().gnss_reanchor_fixes + 4,
            "{step}"
        );
        assert!(graph.pose().y_m.abs() < 0.05);
    }

    #[test]
    fn without_absolute_factors_it_reports_dead_reckoning() {
        let mut graph =
            FactorGraphLocalizer::new(&open_map(), Pose2d::IDENTITY, FactorGraphConfig::default())
                .unwrap();
        for _ in 0..60 {
            graph.predict(0.05, 1.0, 0.1);
        }
        let scan = LaserScan2d {
            time_s: 3.0,
            frame: FrameId::new("laser"),
            angle_min_rad: 0.0,
            angle_increment_rad: TAU / 36.0,
            range_min_m: 0.05,
            range_max_m: 10.0,
            ranges_m: vec![f64::INFINITY; 36],
        };
        graph.update_lidar(&scan, Pose2d::IDENTITY, Pose2d::IDENTITY, 3.0);
        assert_eq!(graph.source(), LocalizationSource::DeadReckoning);
        // Pure odometry: a 3 m arc turning 0.3 rad.
        let pose = graph.pose();
        assert!((pose.yaw_rad - 0.3).abs() < 1e-9, "{pose:?}");
    }
}
