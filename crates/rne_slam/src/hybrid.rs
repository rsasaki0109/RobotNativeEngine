//! GNSS outdoors, LiDAR indoors: a localization arbiter.
//!
//! [`HybridLocalizer`] keeps one [`EkfFusion`] driven by wheel odometry and
//! chooses which absolute source corrects it:
//!
//! * [`LocalizationSource::Gnss`] — position fixes that pass a quality check
//!   (satellites used, HDOP, reported sigma, RAIM consistency and exclusion
//!   count) and a Mahalanobis innovation gate against the EKF are fused.
//! * [`LocalizationSource::DeadReckoning`] — after `leave_gnss_epochs`
//!   unhealthy GNSS epochs, odometry alone carries the estimate.
//! * [`LocalizationSource::Lidar`] — while dead reckoning inside the prior
//!   occupancy map, each scan runs a wide correlative scan match whose window
//!   is sized from the EKF covariance. A good match relocalizes the robot and
//!   seeds an [`Amcl`] particle filter; from then on each AMCL update is
//!   refined by a local scan match and the refined pose corrects the EKF while
//!   its score stays high.
//!
//! GNSS takes back over after `enter_gnss_epochs` consecutive healthy, gated
//! fixes. Fixes are fused only while GNSS is in charge, so isolated
//! healthy-looking fixes in a degraded stretch, often NLOS-biased, are not. A
//! fix that keeps failing only the gate for `gate_reset_epochs` is accepted, to
//! recover from a drifted, over-confident estimate. The arbiter takes a plain
//! [`AbsolutePositionFix`], so it does not depend on a GNSS simulator.

use crate::likelihood::LikelihoodField;
use crate::localization::{Amcl, AmclConfig, AmclUpdate};
use crate::scan_match::{scan_points_2d, ScanMatchConfig, ScanMatcher};
use rne_math::Vec3;
use rne_nav::{
    EkfConfig, EkfFusion, FusionError, GridError, LaserScan2d, OccupancyGrid, Pose2d,
    VelocityCommand2d,
};
use serde::{Deserialize, Serialize};

/// A planar absolute position fix from a satellite receiver.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AbsolutePositionFix {
    /// Measurement time in seconds.
    pub time_s: f64,
    /// Map-frame x in meters.
    pub x_m: f64,
    /// Map-frame y in meters.
    pub y_m: f64,
    /// Reported one-sigma horizontal error in meters.
    pub horizontal_std_m: f64,
    /// Satellites used in the solution.
    pub satellites_used: usize,
    /// Horizontal dilution of precision.
    pub hdop: f64,
    /// Whether the receiver's residual (RAIM) test passed.
    pub consistent: bool,
    /// Satellites the receiver's RAIM excluded this epoch.
    pub satellites_excluded: usize,
    /// Solution type.
    pub kind: FixKind,
}

/// The type of a GNSS position solution, ordered by quality.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum FixKind {
    /// Code-only single-point (or differential) positioning.
    Single,
    /// Carrier-phase RTK with float ambiguities.
    Float,
    /// Carrier-phase RTK with fixed integer ambiguities.
    Fixed,
}

/// A GNSS heading from a moving-baseline between two antennas.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AbsoluteHeadingFix {
    /// Measurement time in seconds.
    pub time_s: f64,
    /// Body heading (from east, counter-clockwise) in radians.
    pub yaw_rad: f64,
    /// One-sigma heading error in radians.
    pub yaw_std_rad: f64,
}

/// The absolute source currently correcting the estimate.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum LocalizationSource {
    /// GNSS fixes.
    Gnss,
    /// LiDAR Monte Carlo localization against the prior map.
    Lidar,
    /// Odometry only.
    DeadReckoning,
}

/// What happened to a GNSS epoch.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum GnssDecision {
    /// Fused into the EKF.
    Fused,
    /// Healthy and gated, but not fused because GNSS is not in charge yet.
    Healthy,
    /// No fix this epoch.
    Missing,
    /// Failed the quality check.
    RejectedQuality,
    /// Failed the innovation gate.
    RejectedGate,
}

/// Arbiter configuration.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct HybridLocalizerConfig {
    /// EKF tuning.
    pub ekf: EkfConfig,
    /// AMCL tuning (initial spreads are overridden from the EKF covariance).
    pub amcl: AmclConfig,
    /// Minimum satellites used for a healthy fix.
    pub min_satellites: usize,
    /// Maximum HDOP for a healthy fix.
    pub max_hdop: f64,
    /// Maximum reported horizontal sigma for a healthy fix, in meters.
    pub max_horizontal_std_m: f64,
    /// Maximum RAIM exclusions for a healthy fix: an epoch that needed many
    /// exclusions is dominated by multipath, and what remains is suspect.
    pub max_excluded: usize,
    /// Squared-Mahalanobis gate for GNSS fixes (2 DOF; 9.21 is 99 %).
    pub gate_chi_square: f64,
    /// Floor on the per-axis variance of a single-point fix, in m² (the
    /// per-axis variance is the reported horizontal sigma squared, a
    /// conservative split).
    pub min_gnss_variance_m2: f64,
    /// Floor on the per-axis variance of an RTK float fix, in m².
    pub min_float_variance_m2: f64,
    /// Floor on the per-axis variance of an RTK fixed fix, in m².
    pub min_fixed_variance_m2: f64,
    /// Heading innovations beyond this many sigma are rejected.
    pub heading_gate_sigma: f64,
    /// Consecutive healthy epochs before GNSS takes over.
    pub enter_gnss_epochs: usize,
    /// Least solution type that counts toward taking over. With RTK, use
    /// [`FixKind::Fixed`]: float solutions coming out of an urban canyon can
    /// carry metres of bias with a confident covariance.
    pub enter_gnss_min_kind: FixKind,
    /// Consecutive unhealthy epochs before GNSS hands over.
    pub leave_gnss_epochs: usize,
    /// Consecutive gate-only rejections after which a fix is accepted anyway,
    /// if it is an RTK fixed solution or the EKF's horizontal sigma exceeds
    /// `gate_reset_min_std_m` (a single sustained NLOS or float bias must not
    /// drag a well-localized estimate).
    pub gate_reset_epochs: usize,
    /// EKF horizontal sigma above which any gated-out fix may reset, meters.
    pub gate_reset_min_std_m: f64,
    /// Minimum scan-match score to fuse a LiDAR pose.
    pub lidar_min_score: f64,
    /// Minimum score of the wide relocalization match that starts LiDAR mode.
    pub lidar_init_min_score: f64,
    /// How far inside the map the estimate must be before relocalizing, in
    /// meters (LiDAR mode ends only when the estimate leaves the map).
    pub lidar_map_margin_m: f64,
    /// Bounds on the relocalization half-window in meters (3σ of the EKF).
    pub lidar_init_window_m: (f64, f64),
    /// Bounds on the relocalization angular half-window in radians.
    pub lidar_init_window_rad: (f64, f64),
    /// Consecutive low-score scans before LiDAR gives up.
    pub lidar_max_low_scores: usize,
    /// Per-axis variance of a fused AMCL position, in m².
    pub lidar_position_variance_m2: f64,
    /// Variance of a fused AMCL yaw, in rad².
    pub lidar_yaw_variance_rad2: f64,
    /// Odometry forward-velocity variance.
    pub odometry_linear_variance: f64,
    /// Odometry (gyro) yaw-rate variance.
    pub odometry_angular_variance: f64,
}

impl Default for HybridLocalizerConfig {
    fn default() -> Self {
        Self {
            // Heading comes from a gyro-grade yaw rate, so position fixes
            // should move position rather than bend the heading.
            ekf: EkfConfig {
                process_noise_position: 0.05,
                process_noise_yaw: 2.0e-5,
                initial_yaw_variance_rad2: 1.0e-3,
                ..EkfConfig::default()
            },
            amcl: AmclConfig::default(),
            min_satellites: 6,
            max_hdop: 2.5,
            max_horizontal_std_m: 4.0,
            max_excluded: 2,
            gate_chi_square: 9.21,
            min_gnss_variance_m2: 1.0,
            min_float_variance_m2: 0.25,
            min_fixed_variance_m2: 4.0e-4,
            heading_gate_sigma: 4.0,
            enter_gnss_epochs: 5,
            enter_gnss_min_kind: FixKind::Single,
            leave_gnss_epochs: 3,
            gate_reset_epochs: 150,
            gate_reset_min_std_m: 3.0,
            lidar_min_score: 0.35,
            lidar_init_min_score: 0.5,
            lidar_map_margin_m: 2.0,
            lidar_init_window_m: (0.5, 6.0),
            lidar_init_window_rad: (0.1, 0.6),
            lidar_max_low_scores: 20,
            lidar_position_variance_m2: 0.01,
            lidar_yaw_variance_rad2: 0.0025,
            odometry_linear_variance: 0.0025,
            odometry_angular_variance: 1.0e-4,
        }
    }
}

/// A source change.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SourceTransition {
    /// Time of the change in seconds.
    pub time_s: f64,
    /// Previous source.
    pub from: LocalizationSource,
    /// New source.
    pub to: LocalizationSource,
}

/// Errors raised by the arbiter.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum HybridError {
    /// The EKF rejected an input.
    #[error(transparent)]
    Fusion(#[from] FusionError),
    /// The prior map or its likelihood field is invalid.
    #[error(transparent)]
    Map(#[from] GridError),
}

/// GNSS-outdoors / LiDAR-indoors localization arbiter.
#[derive(Clone, Debug)]
pub struct HybridLocalizer {
    config: HybridLocalizerConfig,
    ekf: EkfFusion,
    map: OccupancyGrid,
    field: LikelihoodField,
    amcl: Option<Amcl>,
    lidar_search: bool,
    source: LocalizationSource,
    gnss_good_streak: usize,
    gnss_bad_streak: usize,
    gate_streak: usize,
    lidar_low_streak: usize,
    last_amcl: Option<AmclUpdate>,
    transitions: Vec<SourceTransition>,
}

impl HybridLocalizer {
    /// Creates an arbiter at a known initial pose, starting in GNSS mode.
    pub fn new(
        map: OccupancyGrid,
        initial_pose: Pose2d,
        config: HybridLocalizerConfig,
    ) -> Result<Self, HybridError> {
        let ekf = EkfFusion::with_initial_state(config.ekf, initial_pose, VelocityCommand2d::ZERO)?;
        let field = LikelihoodField::from_occupancy(&map, &config.amcl.likelihood)?;
        Ok(Self {
            config,
            ekf,
            map,
            field,
            amcl: None,
            lidar_search: false,
            source: LocalizationSource::Gnss,
            gnss_good_streak: 0,
            gnss_bad_streak: 0,
            gate_streak: 0,
            lidar_low_streak: 0,
            last_amcl: None,
            transitions: Vec::new(),
        })
    }

    /// Fused pose.
    pub fn pose(&self) -> Pose2d {
        self.ekf.pose()
    }

    /// The underlying EKF.
    pub fn ekf(&self) -> &EkfFusion {
        &self.ekf
    }

    /// Current source.
    pub fn source(&self) -> LocalizationSource {
        self.source
    }

    /// The running particle filter, in LiDAR mode.
    pub fn amcl(&self) -> Option<&Amcl> {
        self.amcl.as_ref()
    }

    /// The most recent AMCL update.
    pub fn last_amcl_update(&self) -> Option<AmclUpdate> {
        self.last_amcl
    }

    /// Every source change so far.
    pub fn transitions(&self) -> &[SourceTransition] {
        &self.transitions
    }

    /// Whether the prior map covers a pose.
    pub fn map_covers(&self, pose: Pose2d) -> bool {
        self.map
            .world_to_grid(Vec3::new(pose.x_m, pose.y_m, 0.0))
            .is_some()
    }

    /// Predicts by `dt_s` and fuses a wheel-odometry velocity.
    pub fn predict(
        &mut self,
        dt_s: f64,
        linear_m_s: f64,
        angular_rad_s: f64,
    ) -> Result<(), HybridError> {
        self.ekf.predict(dt_s)?;
        self.ekf.update_odometry(
            linear_m_s,
            angular_rad_s,
            self.config.odometry_linear_variance,
            self.config.odometry_angular_variance,
        )?;
        Ok(())
    }

    /// Processes one GNSS epoch (`None` when the receiver reported no fix).
    pub fn update_gnss(
        &mut self,
        fix: Option<&AbsolutePositionFix>,
        time_s: f64,
    ) -> Result<GnssDecision, HybridError> {
        let decision = self.judge_gnss(fix);
        let healthy = matches!(decision, GnssDecision::Fused);
        if healthy {
            let qualifies = fix.is_some_and(|f| f.kind >= self.config.enter_gnss_min_kind);
            self.gnss_good_streak = if qualifies {
                self.gnss_good_streak + 1
            } else {
                0
            };
            self.gnss_bad_streak = 0;
        } else {
            self.gnss_good_streak = 0;
            self.gnss_bad_streak += 1;
        }

        match self.source {
            LocalizationSource::Gnss if self.gnss_bad_streak >= self.config.leave_gnss_epochs => {
                self.switch(LocalizationSource::DeadReckoning, time_s);
                self.lidar_search = true;
            }
            LocalizationSource::Lidar | LocalizationSource::DeadReckoning
                if self.gnss_good_streak >= self.config.enter_gnss_epochs =>
            {
                self.amcl = None;
                self.lidar_search = false;
                self.switch(LocalizationSource::Gnss, time_s);
            }
            _ => {}
        }

        if !healthy {
            return Ok(decision);
        }
        // Only fuse once GNSS is in charge: isolated healthy-looking fixes in
        // a degraded stretch are often NLOS-biased.
        if self.source != LocalizationSource::Gnss {
            return Ok(GnssDecision::Healthy);
        }
        let fix = fix.ok_or(HybridError::Fusion(FusionError::NonFiniteInput))?;
        self.ekf
            .update_position(fix.x_m, fix.y_m, self.gnss_variance_m2(fix))?;
        Ok(GnssDecision::Fused)
    }

    /// Fuses a moving-baseline heading while GNSS is in charge; returns
    /// whether it was fused. Headings off the estimate by more than
    /// `heading_gate_sigma` combined sigmas are rejected.
    pub fn update_gnss_heading(
        &mut self,
        heading: &AbsoluteHeadingFix,
    ) -> Result<bool, HybridError> {
        if self.source != LocalizationSource::Gnss
            || !heading.yaw_rad.is_finite()
            || heading.yaw_std_rad.is_nan()
            || heading.yaw_std_rad <= 0.0
        {
            return Ok(false);
        }
        let variance = heading.yaw_std_rad * heading.yaw_std_rad;
        let innovation = rne_nav::wrap_angle(heading.yaw_rad - self.ekf.pose().yaw_rad);
        let sigma = (self.ekf.covariance()[2][2] + variance).sqrt();
        if innovation.abs() > self.config.heading_gate_sigma * sigma {
            return Ok(false);
        }
        self.ekf.update_yaw(heading.yaw_rad, variance)?;
        Ok(true)
    }

    /// Processes one LiDAR scan: relocalizes in the map after a GNSS
    /// hand-over, then tracks with AMCL refined by a local scan match.
    ///
    /// Returns the AMCL update when the particle filter ran.
    pub fn update_lidar(
        &mut self,
        scan: &LaserScan2d,
        odom_pose: Pose2d,
        sensor_from_base: Pose2d,
        time_s: f64,
    ) -> Result<Option<AmclUpdate>, HybridError> {
        if self.amcl.is_none() {
            if self.lidar_search
                && self.source == LocalizationSource::DeadReckoning
                && self.map_covers(self.pose())
            {
                self.relocalize(scan, odom_pose, sensor_from_base, time_s)?;
            }
            return Ok(None);
        }
        let Some(amcl) = self.amcl.as_mut() else {
            return Ok(None);
        };
        let update = amcl.update(scan, odom_pose, sensor_from_base);
        self.last_amcl = Some(update);
        // Refine with a local scan match seeded by the odometry-predicted EKF
        // pose (already corrected by earlier scans); if that fails, seed it
        // with the particle filter's estimate, which can recover a larger
        // error but has drifted more on its own.
        let points = scan_points_2d(scan, self.config.amcl.max_beams);
        let matcher = ScanMatcher::new(ScanMatchConfig::default());
        let min_score = self.config.lidar_min_score;
        let refined = [self.ekf.pose(), update.estimate]
            .into_iter()
            .find_map(|seed| {
                matcher
                    .match_scan(&self.field, &points, sensor_from_base, seed)
                    .filter(|matched| matched.score >= min_score)
            });
        match refined {
            Some(matched) => {
                self.lidar_low_streak = 0;
                self.fuse_lidar_pose(matched.pose)?;
            }
            None => self.lidar_low_streak += 1,
        }
        if self.lidar_low_streak >= self.config.lidar_max_low_scores
            || !self.map_covers(self.pose())
        {
            self.amcl = None;
            self.switch(LocalizationSource::DeadReckoning, time_s);
        }
        Ok(Some(update))
    }

    /// Whether the arbiter is searching the map for a LiDAR relocalization.
    pub fn lidar_searching(&self) -> bool {
        self.lidar_search && self.amcl.is_none()
    }

    fn relocalize(
        &mut self,
        scan: &LaserScan2d,
        odom_pose: Pose2d,
        sensor_from_base: Pose2d,
        time_s: f64,
    ) -> Result<(), HybridError> {
        let covariance = self.ekf.covariance();
        let (min_m, max_m) = self.config.lidar_init_window_m;
        let (min_rad, max_rad) = self.config.lidar_init_window_rad;
        let window_m = (3.0 * covariance[0][0].max(covariance[1][1]).sqrt()).clamp(min_m, max_m);
        let window_rad = (3.0 * covariance[2][2].sqrt()).clamp(min_rad, max_rad);
        // About 0.25 m and 0.03 rad between coarse samples, then refine.
        let linear_samples = ((2.0 * window_m / 0.25).ceil() as usize | 1).max(5);
        let angular_samples = ((2.0 * window_rad / 0.03).ceil() as usize | 1).max(5);
        let matcher = ScanMatcher::new(ScanMatchConfig {
            linear_window_m: window_m,
            angular_window_rad: window_rad,
            levels: 4,
            linear_samples,
            angular_samples,
            ..ScanMatchConfig::default()
        });
        let points = scan_points_2d(scan, self.config.amcl.max_beams);
        let Some(matched) = matcher.match_scan(&self.field, &points, sensor_from_base, self.pose())
        else {
            return Ok(());
        };
        if matched.score < self.config.lidar_init_min_score {
            return Ok(());
        }
        let config = AmclConfig {
            initial_translation_std_m: 0.15,
            initial_rotation_std_rad: 0.05,
            ..self.config.amcl
        };
        let mut amcl = Amcl::new(&self.map, matched.pose, config)?;
        // Anchor AMCL's odometry deltas at this scan.
        amcl.update(scan, odom_pose, sensor_from_base);
        self.amcl = Some(amcl);
        self.lidar_low_streak = 0;
        self.fuse_lidar_pose(matched.pose)?;
        self.switch(LocalizationSource::Lidar, time_s);
        Ok(())
    }

    fn fuse_lidar_pose(&mut self, pose: Pose2d) -> Result<(), FusionError> {
        self.ekf
            .update_position(pose.x_m, pose.y_m, self.config.lidar_position_variance_m2)?;
        self.ekf
            .update_yaw(pose.yaw_rad, self.config.lidar_yaw_variance_rad2)
    }

    fn judge_gnss(&mut self, fix: Option<&AbsolutePositionFix>) -> GnssDecision {
        let Some(fix) = fix else {
            return GnssDecision::Missing;
        };
        let finite = fix.x_m.is_finite() && fix.y_m.is_finite() && fix.horizontal_std_m.is_finite();
        if !finite
            || !fix.consistent
            || fix.satellites_used < self.config.min_satellites
            || fix.hdop > self.config.max_hdop
            || fix.horizontal_std_m > self.config.max_horizontal_std_m
            || fix.satellites_excluded > self.config.max_excluded
        {
            return GnssDecision::RejectedQuality;
        }
        let d2 =
            self.ekf
                .position_mahalanobis_squared(fix.x_m, fix.y_m, self.gnss_variance_m2(fix));
        if d2 > self.config.gate_chi_square {
            self.gate_streak += 1;
            let covariance = self.ekf.covariance();
            let lost =
                (covariance[0][0] + covariance[1][1]).sqrt() >= self.config.gate_reset_min_std_m;
            if self.gate_streak < self.config.gate_reset_epochs
                || !(fix.kind == FixKind::Fixed || lost)
            {
                return GnssDecision::RejectedGate;
            }
        }
        self.gate_streak = 0;
        GnssDecision::Fused
    }

    fn gnss_variance_m2(&self, fix: &AbsolutePositionFix) -> f64 {
        let floor = match fix.kind {
            FixKind::Single => self.config.min_gnss_variance_m2,
            FixKind::Float => self.config.min_float_variance_m2,
            FixKind::Fixed => self.config.min_fixed_variance_m2,
        };
        (fix.horizontal_std_m * fix.horizontal_std_m).max(floor)
    }

    fn switch(&mut self, to: LocalizationSource, time_s: f64) {
        if self.source != to {
            self.transitions.push(SourceTransition {
                time_s,
                from: self.source,
                to,
            });
            self.source = to;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_nav::{FrameId, GridCoord};
    use std::f64::consts::TAU;

    /// A 10 m x 6 m room with a pillar, inside a 30 m x 30 m map.
    fn room_map() -> OccupancyGrid {
        let mut grid = OccupancyGrid::new(300, 300, 0.1, Pose2d::new(0.0, -15.0, 0.0)).unwrap();
        for y in 0..grid.height() {
            for x in 0..grid.width() {
                let world = grid.grid_to_world(GridCoord {
                    x: x as isize,
                    y: y as isize,
                });
                if is_wall(world.x, world.y) {
                    grid.mark_occupied(GridCoord {
                        x: x as isize,
                        y: y as isize,
                    });
                } else {
                    grid.mark_free(GridCoord {
                        x: x as isize,
                        y: y as isize,
                    });
                }
            }
        }
        grid
    }

    fn is_wall(x: f64, y: f64) -> bool {
        let in_room = (10.0..=20.0).contains(&x) && (-3.0..=3.0).contains(&y);
        let border = (x - 10.0).abs() < 0.1
            || (x - 20.0).abs() < 0.1
            || (y - 3.0).abs() < 0.1
            || (y + 3.0).abs() < 0.1;
        let pillar = (x - 16.0).hypot(y - 1.0) < 0.4;
        (in_room && border) || pillar
    }

    fn scan_at(pose: Pose2d) -> LaserScan2d {
        let beams = 180;
        let ranges_m = (0..beams)
            .map(|beam| {
                let angle = pose.yaw_rad + TAU * beam as f64 / beams as f64;
                let mut r = 0.0;
                while r < 12.0 {
                    r += 0.02;
                    if is_wall(pose.x_m + r * angle.cos(), pose.y_m + r * angle.sin()) {
                        return r;
                    }
                }
                f64::INFINITY
            })
            .collect();
        LaserScan2d {
            time_s: 0.0,
            frame: FrameId::new("laser"),
            angle_min_rad: 0.0,
            angle_increment_rad: TAU / beams as f64,
            range_min_m: 0.05,
            range_max_m: 12.0,
            ranges_m,
        }
    }

    fn good_fix(x: f64, y: f64) -> AbsolutePositionFix {
        AbsolutePositionFix {
            time_s: 0.0,
            x_m: x,
            y_m: y,
            horizontal_std_m: 1.0,
            satellites_used: 10,
            hdop: 1.0,
            consistent: true,
            satellites_excluded: 0,
            kind: FixKind::Single,
        }
    }

    #[test]
    fn hands_over_to_lidar_indoors_and_back_to_gnss_outdoors() {
        let config = HybridLocalizerConfig {
            amcl: AmclConfig {
                particle_count: 300,
                ..AmclConfig::default()
            },
            ..HybridLocalizerConfig::default()
        };
        let mut localizer =
            HybridLocalizer::new(room_map(), Pose2d::new(13.0, 0.0, 0.0), config).unwrap();
        // GNSS healthy at the start.
        for _ in 0..3 {
            localizer
                .update_gnss(Some(&good_fix(13.0, 0.0)), 0.0)
                .unwrap();
        }
        assert_eq!(localizer.source(), LocalizationSource::Gnss);
        // Indoors: no fixes. The arbiter drops to dead reckoning and searches
        // the map; the start pose is 1.2 m off, inside the relocalization
        // window, and the first scan pulls it in.
        for step in 0..3 {
            localizer.update_gnss(None, step as f64).unwrap();
        }
        assert_eq!(localizer.source(), LocalizationSource::DeadReckoning);
        assert!(localizer.lidar_searching());
        let truth = Pose2d::new(14.0, -0.7, 0.05);
        for _ in 0..10 {
            localizer
                .update_lidar(&scan_at(truth), Pose2d::IDENTITY, Pose2d::IDENTITY, 4.0)
                .unwrap();
        }
        assert_eq!(localizer.source(), LocalizationSource::Lidar);
        assert!(localizer.last_amcl_update().unwrap().mean_score > 0.35);
        let pose = localizer.pose();
        assert!(
            (pose.x_m - truth.x_m).hypot(pose.y_m - truth.y_m) < 0.15,
            "{pose:?}"
        );
        assert!((pose.yaw_rad - truth.yaw_rad).abs() < 0.05, "{pose:?}");
        // Outdoors again: healthy fixes bring GNSS back.
        for step in 0..5 {
            localizer
                .update_gnss(Some(&good_fix(13.0, 0.0)), 10.0 + step as f64)
                .unwrap();
        }
        assert_eq!(localizer.source(), LocalizationSource::Gnss);
        assert!(localizer.amcl().is_none());
        let kinds: Vec<_> = localizer.transitions().iter().map(|t| t.to).collect();
        assert_eq!(
            kinds,
            vec![
                LocalizationSource::DeadReckoning,
                LocalizationSource::Lidar,
                LocalizationSource::Gnss
            ]
        );
    }

    #[test]
    fn rtk_fixed_fixes_and_moving_base_headings_tighten_the_estimate() {
        let mut localizer = HybridLocalizer::new(
            room_map(),
            Pose2d::new(-50.0, 0.0, 0.0),
            HybridLocalizerConfig::default(),
        )
        .unwrap();
        let fixed = AbsolutePositionFix {
            horizontal_std_m: 0.01,
            kind: FixKind::Fixed,
            ..good_fix(-50.2, 0.1)
        };
        for _ in 0..10 {
            localizer.update_gnss(Some(&fixed), 0.0).unwrap();
        }
        let pose = localizer.pose();
        assert!(
            (pose.x_m + 50.2).abs() < 0.01 && (pose.y_m - 0.1).abs() < 0.01,
            "{pose:?}"
        );
        assert!(localizer.ekf().covariance()[0][0] < 1.0e-3);
        let heading = AbsoluteHeadingFix {
            time_s: 0.0,
            yaw_rad: 0.05,
            yaw_std_rad: 0.005,
        };
        assert!(localizer.update_gnss_heading(&heading).unwrap());
        assert!((localizer.pose().yaw_rad - 0.05).abs() < 0.01);
        // A wild heading is gated out.
        let wild = AbsoluteHeadingFix {
            yaw_rad: 2.5,
            ..heading
        };
        assert!(!localizer.update_gnss_heading(&wild).unwrap());
    }

    #[test]
    fn outside_the_map_gnss_loss_falls_back_to_dead_reckoning() {
        let mut localizer = HybridLocalizer::new(
            room_map(),
            Pose2d::new(-50.0, 0.0, 0.0),
            HybridLocalizerConfig::default(),
        )
        .unwrap();
        for step in 0..3 {
            localizer.update_gnss(None, step as f64).unwrap();
        }
        assert_eq!(localizer.source(), LocalizationSource::DeadReckoning);
        // No map here, so a scan does not start LiDAR mode.
        localizer
            .update_lidar(
                &scan_at(Pose2d::new(-50.0, 0.0, 0.0)),
                Pose2d::IDENTITY,
                Pose2d::IDENTITY,
                3.0,
            )
            .unwrap();
        assert_eq!(localizer.source(), LocalizationSource::DeadReckoning);
    }

    #[test]
    fn gate_rejects_an_nlos_jump_but_an_rtk_fix_resets_it() {
        let config = HybridLocalizerConfig {
            gate_reset_epochs: 4,
            leave_gnss_epochs: 100,
            ..HybridLocalizerConfig::default()
        };
        let mut localizer =
            HybridLocalizer::new(room_map(), Pose2d::new(-50.0, 0.0, 0.0), config).unwrap();
        for _ in 0..20 {
            localizer
                .update_gnss(Some(&good_fix(-50.0, 0.0)), 0.0)
                .unwrap();
        }
        // A sustained single-point bias never resets a well-localized EKF...
        let jump = good_fix(-50.0, 25.0);
        for _ in 0..10 {
            assert_eq!(
                localizer.update_gnss(Some(&jump), 0.0).unwrap(),
                GnssDecision::RejectedGate
            );
        }
        // ...but an RTK fixed solution that keeps disagreeing does.
        let fixed = AbsolutePositionFix {
            horizontal_std_m: 0.02,
            kind: FixKind::Fixed,
            ..jump
        };
        assert_eq!(
            localizer.update_gnss(Some(&fixed), 0.0).unwrap(),
            GnssDecision::Fused
        );
        for weak in [
            AbsolutePositionFix {
                hdop: 9.0,
                ..good_fix(-50.0, 25.0)
            },
            AbsolutePositionFix {
                satellites_excluded: 4,
                ..good_fix(-50.0, 25.0)
            },
        ] {
            assert_eq!(
                localizer.update_gnss(Some(&weak), 0.0).unwrap(),
                GnssDecision::RejectedQuality
            );
        }
    }
}
