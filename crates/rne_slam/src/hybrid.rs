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
//! recover from a drifted, over-confident estimate; outside GNSS mode it
//! re-anchors the estimate, so the following fixes pass the gate. With
//! `gnss_reanchor_fixes`, a run of gate-rejected RTK fixed fixes that agree
//! among themselves relative to the odometry re-anchors it sooner and
//! restarts LiDAR tracking there; with `lidar_drop_unmapped_returns`,
//! returns in mapped free space (people, vehicles) are left out of LiDAR
//! tracking; with `lidar_motion_check_scans`, matches that stay put while
//! the wheels move (locked onto someone walking alongside) are not fused.
//! These, with a finite `lidar_gate_chi_square`, keep a crowded street from
//! dragging the estimate. The arbiter takes a plain
//! [`AbsolutePositionFix`], so it does not depend on a GNSS simulator.

use crate::likelihood::LikelihoodField;
use crate::localization::{Amcl, AmclConfig, AmclUpdate};
use crate::pointcloud_map::PointCloudMatch;
use crate::scan_match::{match_position_covariance, scan_points_2d, ScanMatchConfig, ScanMatcher};
use rne_math::Vec3;
use rne_nav::{
    EkfConfig, EkfFusion, FusionError, GridError, LaserScan2d, OccupancyGrid, Pose2d,
    VelocityCommand2d,
};
use serde::{Deserialize, Serialize};

/// With anisotropic LiDAR fusion, the particle filter's match is used unless
/// the EKF-seeded one scores higher by more than this.
const LIDAR_SEED_SCORE_MARGIN: f64 = 0.02;

/// With anisotropic LiDAR fusion, a particle-filter match farther than this
/// from the EKF re-anchors the EKF there, meters.
const LIDAR_REANCHOR_M: f64 = 2.0;

/// Consecutive scans of such disagreement before the EKF is re-anchored.
const LIDAR_REANCHOR_SCANS: usize = 10;

/// Least travel over the motion-check window for the check to judge, meters.
const LIDAR_LOCK_MIN_TRAVEL_M: f64 = 0.5;

/// A locked match is released once a match is within this of the estimate
/// along the direction of travel, meters.
const LIDAR_LOCK_RELEASE_M: f64 = 0.25;

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
    /// After this many RTK fixed fixes in a row fail only the gate while
    /// agreeing among themselves relative to the odometry (to within
    /// `gnss_reanchor_spread_m`), the estimate is re-anchored on them: the
    /// receiver keeps reporting the same place as the wheels move, so the
    /// estimate, not the receiver, is wrong. LiDAR tracking restarts from
    /// there, searching a window sized by the fixes' variance. While dead
    /// reckoning, float fixes count too (a lost robot that has stopped may
    /// never get a fixed solution). Zero (off) by default.
    pub gnss_reanchor_fixes: usize,
    /// Largest distance of a rejected fix's offset from the run's mean
    /// offset for re-anchoring, meters.
    pub gnss_reanchor_spread_m: f64,
    /// Least distance the wheels must cover over a run of rejected fixes
    /// for it to re-anchor, meters. A receiver standing still in a canyon
    /// can report the same biased float solution for minutes; once the
    /// robot moves, the bias changes with the geometry. Zero by default.
    pub gnss_reanchor_min_travel_m: f64,
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
    /// Fuse each scan match with a covariance from the curvature of the match
    /// score around it, instead of the isotropic
    /// `lidar_position_variance_m2`: a scan along a featureless corridor
    /// pins the cross-track position but not the along-track one, and an
    /// isotropic update would drag the estimate toward wherever the match
    /// stopped. With it, the scan is also matched from the particle filter's
    /// estimate over a wider window, and that match replaces the EKF-seeded
    /// one (re-anchoring the EKF if it is metres away) unless it explains the
    /// scan clearly worse. Off by default.
    pub lidar_anisotropic: bool,
    /// Upper bound on a direction's variance from the curvature, m².
    pub lidar_max_variance_m2: f64,
    /// Squared Mahalanobis distance beyond which a LiDAR match is not fused.
    /// Infinite (off) by default: a gate protects a well-localized estimate
    /// from matches sliding along a corridor or street, but also keeps LiDAR
    /// from correcting an estimate that a wrong GNSS fix left off. Enable it
    /// (for example 9.21, the 99 % gate) when GNSS is RTK-grade.
    pub lidar_gate_chi_square: f64,
    /// Drop returns the prior map cannot explain before tracking: beams
    /// whose endpoint, placed at the current estimate, lies in mapped free
    /// space at least `lidar_unmapped_distance_m` from any mapped structure
    /// (a cell counts as mapped free once it has any free evidence). These
    /// are pedestrians, vehicles and other things that move; left in, a
    /// person walking beside the robot looks like a pole that keeps pace
    /// with it and holds the match back. When more than half the returns
    /// would go, the estimate is the more likely culprit and the scan is
    /// used whole. Off by default.
    pub lidar_drop_unmapped_returns: bool,
    /// Distance from mapped structure beyond which a return is unexplained,
    /// meters; capped at the likelihood field's reach
    /// (`amcl.likelihood.max_distance_m`). Smaller drops people walking
    /// close to facades and poles too, but tolerates less error in the
    /// estimate.
    pub lidar_unmapped_distance_m: f64,
    /// When, over this many scans, the matched positions advance along the
    /// direction of travel by less than half the distance the wheels covered
    /// (at least `LIDAR_LOCK_MIN_TRAVEL_M` of it), the matches are not fused:
    /// they have locked onto something that moves with the robot, such as a
    /// person walking beside it, and would hold the estimate back while it
    /// walks on. A correct match that is merely offset from the estimate
    /// advances with the robot and passes. Once locked, matches stay
    /// unfused until one agrees with the estimate along the direction of
    /// travel (within `LIDAR_LOCK_RELEASE_M`), or for three windows at
    /// most. Zero (off) by default.
    pub lidar_motion_check_scans: usize,
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
            gnss_reanchor_fixes: 0,
            gnss_reanchor_spread_m: 0.5,
            gnss_reanchor_min_travel_m: 0.0,
            lidar_min_score: 0.35,
            lidar_init_min_score: 0.5,
            lidar_map_margin_m: 2.0,
            lidar_init_window_m: (0.5, 6.0),
            lidar_init_window_rad: (0.1, 0.6),
            lidar_max_low_scores: 20,
            lidar_position_variance_m2: 0.01,
            lidar_gate_chi_square: f64::INFINITY,
            lidar_anisotropic: false,
            lidar_max_variance_m2: 100.0,
            lidar_drop_unmapped_returns: false,
            lidar_unmapped_distance_m: 1.0,
            lidar_motion_check_scans: 0,
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
    /// Consecutive scans whose preferred particle-filter match lay metres
    /// from the EKF.
    reanchor_streak: usize,
    /// Signed distance the wheels covered since the last scan, meters.
    travelled_since_scan_m: f64,
    /// Per recent scan: the wheels' distance so far, and the matched
    /// position, if any.
    motion_history: std::collections::VecDeque<(f64, Option<(f64, f64)>)>,
    /// The wheels' distance since the start, meters.
    travelled_m: f64,
    /// Scans skipped since the matches locked (zero when not locked).
    motion_lock_scans: usize,
    /// Odometry-only pose, re-synced to the estimate when a run of rejected
    /// RTK fixes starts.
    dead_reckoned: Pose2d,
    /// Offsets of the current run of gate-rejected RTK fixed fixes from
    /// `dead_reckoned`, with their variances and where `dead_reckoned` was.
    rejected_offsets: Vec<(f64, f64, f64, (f64, f64))>,
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
            reanchor_streak: 0,
            travelled_since_scan_m: 0.0,
            motion_history: std::collections::VecDeque::new(),
            travelled_m: 0.0,
            motion_lock_scans: 0,
            dead_reckoned: initial_pose,
            rejected_offsets: Vec::new(),
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
        if linear_m_s.is_finite() && dt_s.is_finite() {
            self.travelled_since_scan_m += linear_m_s * dt_s;
        }
        if linear_m_s.is_finite() && angular_rad_s.is_finite() && dt_s.is_finite() {
            self.dead_reckoned = self.dead_reckoned.compose(Pose2d::new(
                linear_m_s * dt_s,
                0.0,
                angular_rad_s * dt_s,
            ));
        }
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
        let (decision, gate_reset) = self.judge_gnss(fix);
        if let Some(fix) = fix {
            if self.reanchor_on_rejected_fixes(fix, decision, time_s)? {
                return Ok(GnssDecision::RejectedGate);
            }
        }
        let healthy = matches!(decision, GnssDecision::Fused);
        if let (true, Some(fix)) = (gate_reset && self.source != LocalizationSource::Gnss, fix) {
            // The estimate is wrong, not the fix: re-anchor it there, or the
            // next fixes would fail the gate again and GNSS could never
            // collect the consecutive fixes it needs to take over.
            self.ekf
                .update_position(fix.x_m, fix.y_m, self.gnss_variance_m2(fix))?;
        }
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
        let filtered = if self.config.lidar_drop_unmapped_returns {
            self.without_unmapped_returns(scan, sensor_from_base)
        } else {
            None
        };
        let scan = filtered.as_ref().unwrap_or(scan);
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
        // Degeneracy-aware tracking searches a wider window, so the particle
        // filter's seed can reach the pose even when it is off by a metre.
        let matcher = ScanMatcher::new(if self.config.lidar_anisotropic {
            ScanMatchConfig {
                linear_window_m: 1.0,
                linear_samples: 9,
                levels: 4,
                ..ScanMatchConfig::default()
            }
        } else {
            ScanMatchConfig::default()
        });
        let min_score = self.config.lidar_min_score;
        let anisotropic = self.config.lidar_anisotropic;
        let mut candidates = [self.ekf.pose(), update.estimate].into_iter().map(|seed| {
            matcher
                .match_scan(&self.field, &points, sensor_from_base, seed)
                .filter(|matched| matched.score >= min_score)
        });
        let refined = if anisotropic {
            // A local match seeded by the EKF can lock onto a nearby score
            // peak (facade steps repeat along a street) and hold the estimate
            // back; the particle filter integrates the scans over time. Use
            // its match unless it fits clearly worse.
            let ekf_match = candidates.next().flatten();
            let amcl_match = candidates.next().flatten();
            match (ekf_match, amcl_match) {
                (Some(e), Some(a)) if a.score + LIDAR_SEED_SCORE_MARGIN >= e.score => {
                    Some((a, true))
                }
                (Some(e), _) => Some((e, false)),
                (None, a) => a.map(|a| (a, true)),
            }
        } else {
            candidates.flatten().next().map(|m| (m, false))
        };
        // With a finite gate, a match that disagrees with the EKF beyond it
        // (a parked car taken for a wall, a scan sliding along a facade)
        // counts as a poor score instead of dragging the estimate.
        let refined = refined
            .map(|(mut matched, from_particles)| {
                let covariance = self.match_covariance(&points, sensor_from_base, matched.pose);
                // The particle filter has integrated the scans along the
                // corridor; only a local match's degenerate direction is
                // meaningless.
                if !from_particles {
                    matched.pose = self.drop_degenerate_innovation(matched.pose, covariance);
                }
                (matched, covariance, from_particles)
            })
            .filter(|(matched, covariance, _)| {
                self.ekf.position_mahalanobis_squared_covariance(
                    matched.pose.x_m,
                    matched.pose.y_m,
                    *covariance,
                ) <= self.config.lidar_gate_chi_square
            });
        if self.matches_hold_the_robot_back(refined.as_ref().map(|(m, _, _)| m.pose)) {
            // Neither fused nor counted as a poor match: odometry carries the
            // estimate until the matches follow the robot again.
            return Ok(Some(update));
        }
        match refined {
            Some((matched, covariance, from_particles)) => {
                self.lidar_low_streak = 0;
                let estimate = self.ekf.pose();
                let apart = |p: Pose2d| (p.x_m - estimate.x_m).hypot(p.y_m - estimate.y_m);
                if from_particles && apart(matched.pose) > LIDAR_REANCHOR_M {
                    // The EKF and the particle filter disagree by metres;
                    // when the particle filter keeps explaining the scans at
                    // least as well, the EKF has lost it (typically along a
                    // corridor, where single scans cannot pull it):
                    // re-anchor it there.
                    self.reanchor_streak += 1;
                    if self.reanchor_streak >= LIDAR_REANCHOR_SCANS {
                        self.ekf = EkfFusion::with_initial_state(
                            self.config.ekf,
                            matched.pose,
                            self.ekf.velocity(),
                        )?;
                        self.reanchor_streak = 0;
                    } else {
                        // Not yet: fuse nothing from this scan.
                        return Ok(Some(update));
                    }
                } else {
                    self.reanchor_streak = 0;
                    if anisotropic && !from_particles && apart(update.estimate) > LIDAR_REANCHOR_M {
                        // The particle filter wandered off while the EKF's
                        // match explains the scan better: re-seed it.
                        let config = AmclConfig {
                            initial_translation_std_m: 0.15,
                            initial_rotation_std_rad: 0.05,
                            ..self.config.amcl
                        };
                        let mut amcl = Amcl::new(&self.map, matched.pose, config)?;
                        amcl.update(scan, odom_pose, sensor_from_base);
                        self.amcl = Some(amcl);
                    }
                }
                self.fuse_lidar_pose(matched.pose, covariance)?;
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

    /// The scan without the returns the map cannot explain at the current
    /// estimate (see `lidar_drop_unmapped_returns`); `None` when nothing is
    /// dropped or too much would be.
    fn without_unmapped_returns(
        &self,
        scan: &LaserScan2d,
        sensor_from_base: Pose2d,
    ) -> Option<LaserScan2d> {
        let sensor = self.ekf.pose().compose(sensor_from_base);
        // The field is exp(-d²/2σ²) within its reach and zero beyond.
        let likelihood = self.config.amcl.likelihood;
        let distance = self.config.lidar_unmapped_distance_m;
        let threshold = if distance >= likelihood.max_distance_m {
            0.0
        } else {
            (-(distance * distance) / (2.0 * likelihood.sigma_m * likelihood.sigma_m)).exp()
        };
        let mut filtered = scan.clone();
        let (mut valid, mut dropped) = (0_usize, 0_usize);
        for (index, range) in filtered.ranges_m.iter_mut().enumerate() {
            if !(range.is_finite() && *range >= scan.range_min_m && *range <= scan.range_max_m) {
                continue;
            }
            valid += 1;
            let angle = scan.angle_min_rad + scan.angle_increment_rad * index as f64;
            let end =
                sensor.transform_point(Vec3::new(angle.cos() * *range, angle.sin() * *range, 0.0));
            let free = self
                .map
                .world_to_grid(end)
                .and_then(|coord| self.map.probability(coord))
                .is_some_and(|p| p < 0.5);
            if free && self.field.score(end) <= threshold {
                *range = f64::INFINITY;
                dropped += 1;
            }
        }
        (dropped > 0 && 2 * dropped <= valid).then_some(filtered)
    }

    /// Tracks runs of gate-rejected RTK fixed fixes (see
    /// `gnss_reanchor_fixes`); returns whether the estimate was re-anchored.
    fn reanchor_on_rejected_fixes(
        &mut self,
        fix: &AbsolutePositionFix,
        decision: GnssDecision,
        time_s: f64,
    ) -> Result<bool, HybridError> {
        let needed = self.config.gnss_reanchor_fixes;
        if needed == 0 {
            return Ok(false);
        }
        let lost = self.source == LocalizationSource::DeadReckoning;
        match (decision, fix.kind) {
            (GnssDecision::RejectedGate, FixKind::Fixed) => {}
            (GnssDecision::RejectedGate, FixKind::Float) if lost => {}
            // Other rejections neither confirm nor break a run.
            (GnssDecision::RejectedGate | GnssDecision::RejectedQuality, _) => return Ok(false),
            _ => {
                self.rejected_offsets.clear();
                return Ok(false);
            }
        }
        if self.rejected_offsets.is_empty() {
            self.dead_reckoned = self.ekf.pose();
        }
        let origin = self.dead_reckoned;
        self.rejected_offsets.push((
            fix.x_m - origin.x_m,
            fix.y_m - origin.y_m,
            self.gnss_variance_m2(fix),
            (origin.x_m, origin.y_m),
        ));
        if self.rejected_offsets.len() > needed {
            self.rejected_offsets.remove(0);
        }
        if self.rejected_offsets.len() < needed {
            return Ok(false);
        }
        let n = needed as f64;
        let (mx, my) = self
            .rejected_offsets
            .iter()
            .fold((0.0, 0.0), |(a, b), (x, y, _, _)| (a + x / n, b + y / n));
        let spread = self
            .rejected_offsets
            .iter()
            .map(|(x, y, _, _)| (x - mx).hypot(y - my))
            .fold(0.0, f64::max);
        let variance_m2 = self
            .rejected_offsets
            .iter()
            .map(|(_, _, v, _)| *v)
            .fold(self.config.ekf.initial_position_variance_m2, f64::max);
        let travelled = match (self.rejected_offsets.first(), self.rejected_offsets.last()) {
            (Some(first), Some(last)) => (last.3 .0 - first.3 .0).hypot(last.3 .1 - first.3 .1),
            _ => 0.0,
        };
        if spread > self.config.gnss_reanchor_spread_m
            || travelled < self.config.gnss_reanchor_min_travel_m
        {
            return Ok(false);
        }
        let pose = Pose2d::new(origin.x_m + mx, origin.y_m + my, self.ekf.pose().yaw_rad);
        let ekf = EkfConfig {
            initial_position_variance_m2: variance_m2,
            ..self.config.ekf
        };
        self.ekf = EkfFusion::with_initial_state(ekf, pose, self.ekf.velocity())?;
        self.rejected_offsets.clear();
        self.gate_streak = 0;
        // The particle filter followed the old placement; search the map
        // again around the new one.
        if self.amcl.take().is_some() || self.source == LocalizationSource::DeadReckoning {
            self.lidar_search = true;
            self.switch(LocalizationSource::DeadReckoning, time_s);
        }
        Ok(true)
    }

    /// Records this scan's travel and matched position (see
    /// `lidar_motion_check_scans`); returns whether this match is not to be
    /// fused because the matches stay put while the wheels move.
    fn matches_hold_the_robot_back(&mut self, matched: Option<Pose2d>) -> bool {
        self.travelled_m += std::mem::take(&mut self.travelled_since_scan_m);
        let window = self.config.lidar_motion_check_scans;
        if window == 0 {
            return false;
        }
        self.motion_history
            .push_back((self.travelled_m, matched.map(|m| (m.x_m, m.y_m))));
        while self.motion_history.len() > window {
            self.motion_history.pop_front();
        }
        let estimate = self.ekf.pose();
        let (sin, cos) = estimate.yaw_rad.sin_cos();
        let locked = self.matches_stay_put(sin, cos);
        if self.motion_lock_scans == 0 {
            if locked {
                self.motion_lock_scans = 1;
            }
            return locked;
        }
        let agrees = matched.is_some_and(|m| {
            ((m.x_m - estimate.x_m) * cos + (m.y_m - estimate.y_m) * sin).abs()
                <= LIDAR_LOCK_RELEASE_M
        });
        if (agrees && !locked) || self.motion_lock_scans >= 3 * window {
            self.motion_lock_scans = 0;
            self.motion_history.clear();
            return false;
        }
        self.motion_lock_scans += 1;
        true
    }

    /// Whether the matched positions in the window advanced by less than
    /// half the wheels' travel, along the heading `(cos, sin)`.
    fn matches_stay_put(&self, sin: f64, cos: f64) -> bool {
        let matches = || {
            self.motion_history
                .iter()
                .filter_map(|(d, p)| p.map(|p| (*d, p)))
        };
        let (Some((d0, p0)), Some((d1, p1))) = (matches().next(), matches().next_back()) else {
            return false;
        };
        let travel = d1 - d0;
        if travel.abs() < LIDAR_LOCK_MIN_TRAVEL_M {
            return false;
        }
        let progress = (p1.0 - p0.0) * cos + (p1.1 - p0.1) * sin;
        progress * travel.signum() < 0.5 * travel.abs()
    }

    /// Processes a 3D LiDAR scan's match in a point-cloud map (or `None`
    /// when the scan did not match), in place of [`Self::update_lidar`]:
    /// the caller matches the scan, from [`Self::pose`] while tracking and
    /// over [`Self::lidar_search_window_m`] while searching. A match while
    /// searching relocalizes the robot and starts LiDAR mode; while
    /// tracking, matches pass `lidar_gate_chi_square` (against their own
    /// covariance) and `lidar_motion_check_scans` before they are fused, and
    /// `lidar_max_low_scores` consecutive scans without one end LiDAR mode.
    pub fn update_lidar_match(
        &mut self,
        matched: Option<&PointCloudMatch>,
        time_s: f64,
    ) -> Result<(), HybridError> {
        if self.source != LocalizationSource::Lidar {
            if let Some(m) = matched {
                if self.lidar_search
                    && self.source == LocalizationSource::DeadReckoning
                    && self.map_covers(self.pose())
                {
                    self.fuse_point_cloud_match(m)?;
                    self.lidar_low_streak = 0;
                    self.motion_history.clear();
                    self.motion_lock_scans = 0;
                    self.switch(LocalizationSource::Lidar, time_s);
                }
            }
            return Ok(());
        }
        let gated = matched.filter(|m| {
            self.ekf.position_mahalanobis_squared_covariance(
                m.pose.x_m,
                m.pose.y_m,
                m.position_covariance_m2,
            ) <= self.config.lidar_gate_chi_square
        });
        if self.matches_hold_the_robot_back(gated.map(|m| m.pose)) {
            return Ok(());
        }
        match gated {
            Some(m) => {
                self.lidar_low_streak = 0;
                self.fuse_point_cloud_match(m)?;
            }
            None => self.lidar_low_streak += 1,
        }
        if self.lidar_low_streak >= self.config.lidar_max_low_scores
            || !self.map_covers(self.pose())
        {
            self.switch(LocalizationSource::DeadReckoning, time_s);
            self.lidar_search = true;
        }
        Ok(())
    }

    /// Half-width of the area a relocalizing match should search, meters:
    /// three sigma of the estimate, within `lidar_init_window_m`.
    pub fn lidar_search_window_m(&self) -> f64 {
        let covariance = self.ekf.covariance();
        let (min_m, max_m) = self.config.lidar_init_window_m;
        (3.0 * covariance[0][0].max(covariance[1][1]).sqrt()).clamp(min_m, max_m)
    }

    fn fuse_point_cloud_match(&mut self, m: &PointCloudMatch) -> Result<(), FusionError> {
        self.ekf
            .update_position_covariance(m.pose.x_m, m.pose.y_m, m.position_covariance_m2)?;
        self.ekf.update_yaw(m.pose.yaw_rad, m.yaw_variance_rad2)
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
        let covariance = self.match_covariance(&points, sensor_from_base, matched.pose);
        self.fuse_lidar_pose(matched.pose, covariance)?;
        self.switch(LocalizationSource::Lidar, time_s);
        Ok(())
    }

    fn fuse_lidar_pose(
        &mut self,
        pose: Pose2d,
        covariance_m2: [[f64; 2]; 2],
    ) -> Result<(), FusionError> {
        self.ekf
            .update_position_covariance(pose.x_m, pose.y_m, covariance_m2)?;
        self.ekf
            .update_yaw(pose.yaw_rad, self.config.lidar_yaw_variance_rad2)
    }

    /// Removes the part of a match's position innovation along a direction
    /// the scan does not constrain at all (variance at
    /// `lidar_max_variance_m2`): many weak updates toward where a match
    /// happened to stop along a corridor would otherwise add up.
    fn drop_degenerate_innovation(&self, pose: Pose2d, covariance_m2: [[f64; 2]; 2]) -> Pose2d {
        let [[a, b], [_, d]] = covariance_m2;
        let mean = 0.5 * (a + d);
        let spread = (0.25 * (a - d) * (a - d) + b * b).sqrt();
        if mean + spread < 0.99 * self.config.lidar_max_variance_m2 {
            return pose;
        }
        // Unit eigenvector of the large eigenvalue.
        let angle = 0.5 * (2.0 * b).atan2(a - d);
        let (sin, cos) = angle.sin_cos();
        let estimate = self.ekf.pose();
        let (dx, dy) = (pose.x_m - estimate.x_m, pose.y_m - estimate.y_m);
        let along = dx * cos + dy * sin;
        Pose2d::new(pose.x_m - along * cos, pose.y_m - along * sin, pose.yaw_rad)
    }

    /// Position covariance of a scan match (see [`match_position_covariance`]).
    fn match_covariance(
        &self,
        points: &[Vec3],
        sensor_from_base: Pose2d,
        pose: Pose2d,
    ) -> [[f64; 2]; 2] {
        let base = self.config.lidar_position_variance_m2;
        if !self.config.lidar_anisotropic || points.is_empty() {
            return [[base, 0.0], [0.0, base]];
        }
        match_position_covariance(
            &self.field,
            points,
            sensor_from_base,
            pose,
            base,
            self.config.lidar_max_variance_m2,
        )
    }

    /// The decision on a fix, and whether it passed only by a gate reset.
    fn judge_gnss(&mut self, fix: Option<&AbsolutePositionFix>) -> (GnssDecision, bool) {
        let Some(fix) = fix else {
            return (GnssDecision::Missing, false);
        };
        let finite = fix.x_m.is_finite() && fix.y_m.is_finite() && fix.horizontal_std_m.is_finite();
        if !finite
            || !fix.consistent
            || fix.satellites_used < self.config.min_satellites
            || fix.hdop > self.config.max_hdop
            || fix.horizontal_std_m > self.config.max_horizontal_std_m
            || fix.satellites_excluded > self.config.max_excluded
        {
            return (GnssDecision::RejectedQuality, false);
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
                return (GnssDecision::RejectedGate, false);
            }
        }
        let reset = d2 > self.config.gate_chi_square;
        self.gate_streak = 0;
        (GnssDecision::Fused, reset)
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
    fn returns_from_people_in_mapped_free_space_are_dropped() {
        let config = HybridLocalizerConfig {
            lidar_drop_unmapped_returns: true,
            ..HybridLocalizerConfig::default()
        };
        let truth = Pose2d::new(13.0, 0.0, 0.0);
        let localizer = HybridLocalizer::new(room_map(), truth, config).unwrap();
        let clean = scan_at(truth);
        assert!(localizer
            .without_unmapped_returns(&clean, Pose2d::IDENTITY)
            .is_none());
        // A person 1.5 m ahead hides the beams around 0°; the pillar at
        // (16, 1) and the walls stay.
        let mut crowded = clean.clone();
        for beam in [0, 1, 2, 178, 179] {
            crowded.ranges_m[beam] = 1.5;
        }
        let filtered = localizer
            .without_unmapped_returns(&crowded, Pose2d::IDENTITY)
            .unwrap();
        for (beam, (&kept, &original)) in filtered.ranges_m.iter().zip(&clean.ranges_m).enumerate()
        {
            if [0, 1, 2, 178, 179].contains(&beam) {
                assert!(kept.is_infinite());
            } else {
                assert_eq!(kept, original, "beam {beam}");
            }
        }
        // Placed 3 m off, most returns look unexplained: the estimate is
        // suspect, not the scan, so nothing is dropped.
        let lost = HybridLocalizer::new(room_map(), Pose2d::new(16.0, 2.0, 1.0), config).unwrap();
        assert!(lost
            .without_unmapped_returns(&crowded, Pose2d::IDENTITY)
            .is_none());
    }

    #[test]
    fn rtk_fixes_that_agree_with_the_odometry_re_anchor_a_lost_estimate() {
        let config = HybridLocalizerConfig {
            gnss_reanchor_fixes: 10,
            ..HybridLocalizerConfig::default()
        };
        // The estimate starts 20 m off and confident; the robot drives east
        // at 1 m/s and the receiver reports where it really is.
        let mut localizer =
            HybridLocalizer::new(room_map(), Pose2d::new(0.0, 20.0, 0.0), config).unwrap();
        let fixed = |x: f64| AbsolutePositionFix {
            horizontal_std_m: 0.02,
            kind: FixKind::Fixed,
            ..good_fix(x, 0.0)
        };
        let mut reanchored_at = None;
        for step in 0..30 {
            let t = f64::from(step) * 0.2;
            localizer.predict(0.2, 1.0, 0.0).unwrap();
            let decision = localizer.update_gnss(Some(&fixed(t + 0.2)), t).unwrap();
            if reanchored_at.is_none() && localizer.pose().y_m.abs() < 1.0 {
                reanchored_at = Some(step);
            }
            if reanchored_at.is_none() {
                assert_eq!(decision, GnssDecision::RejectedGate);
            }
        }
        assert_eq!(reanchored_at, Some(9));
        let pose = localizer.pose();
        assert!(
            (pose.x_m - 6.0).abs() < 0.1 && pose.y_m.abs() < 0.1,
            "{pose:?}"
        );
        assert_eq!(localizer.source(), LocalizationSource::Gnss);

        // Fixes that wander (a wrong ambiguity in multipath) never do.
        let mut localizer =
            HybridLocalizer::new(room_map(), Pose2d::new(0.0, 20.0, 0.0), config).unwrap();
        for step in 0..60 {
            let t = f64::from(step) * 0.2;
            localizer.predict(0.2, 1.0, 0.0).unwrap();
            let jitter = if step % 2 == 0 { 1.0 } else { -1.0 };
            localizer
                .update_gnss(Some(&fixed(t + 0.2 + jitter)), t)
                .unwrap();
        }
        assert!(localizer.pose().y_m > 19.0, "{:?}", localizer.pose());

        // Float fixes re-anchor only once the arbiter is dead reckoning,
        // with the float fixes' uncertainty.
        let float = |x: f64| AbsolutePositionFix {
            kind: FixKind::Float,
            ..good_fix(x, 0.0)
        };
        let confident = HybridLocalizerConfig {
            ekf: EkfConfig {
                initial_position_variance_m2: 0.01,
                ..config.ekf
            },
            ..config
        };
        let mut localizer =
            HybridLocalizer::new(room_map(), Pose2d::new(0.0, 20.0, 0.0), confident).unwrap();
        // The first `leave_gnss_epochs` rejections, while GNSS is still in
        // charge, do not count; the next ten do.
        let leave = confident.leave_gnss_epochs;
        for step in 0..leave + 9 {
            let decision = localizer
                .update_gnss(Some(&float(0.0)), f64::from(step as u32))
                .unwrap();
            assert_eq!(decision, GnssDecision::RejectedGate, "step {step}");
        }
        assert_eq!(localizer.source(), LocalizationSource::DeadReckoning);
        assert!(localizer.pose().y_m > 19.0, "{:?}", localizer.pose());
        localizer.update_gnss(Some(&float(0.0)), 20.0).unwrap();
        let pose = localizer.pose();
        assert!(pose.x_m.hypot(pose.y_m) < 0.1, "{pose:?}");
        let covariance = localizer.ekf().covariance();
        assert!(covariance[0][0] >= 0.99, "{covariance:?}");

        // With a minimum travel, a robot standing still does not take the
        // same (possibly biased) float solution repeated.
        let still = HybridLocalizerConfig {
            gnss_reanchor_min_travel_m: 3.0,
            ..confident
        };
        let mut localizer =
            HybridLocalizer::new(room_map(), Pose2d::new(0.0, 20.0, 0.0), still).unwrap();
        for step in 0..60 {
            localizer.predict(0.2, 0.0, 0.0).unwrap();
            localizer
                .update_gnss(Some(&float(0.0)), f64::from(step))
                .unwrap();
        }
        assert!(localizer.pose().y_m > 19.0, "{:?}", localizer.pose());
    }

    #[test]
    fn matches_that_cancel_the_wheels_motion_are_not_fused() {
        // In LiDAR mode, the wheels drive the robot 2 m east while every
        // scan looks as if it had not moved (locked onto something that
        // keeps pace with it).
        let run = |window: usize| {
            let config = HybridLocalizerConfig {
                amcl: AmclConfig {
                    particle_count: 300,
                    ..AmclConfig::default()
                },
                lidar_motion_check_scans: window,
                ..HybridLocalizerConfig::default()
            };
            let start = Pose2d::new(13.0, 0.0, 0.0);
            let mut localizer = HybridLocalizer::new(room_map(), start, config).unwrap();
            for step in 0..3 {
                localizer.update_gnss(None, f64::from(step)).unwrap();
            }
            for _ in 0..3 {
                localizer
                    .update_lidar(&scan_at(start), start, Pose2d::IDENTITY, 3.0)
                    .unwrap();
            }
            assert_eq!(localizer.source(), LocalizationSource::Lidar);
            let mut odom = start;
            for step in 0..20 {
                localizer.predict(0.1, 1.0, 0.0).unwrap();
                odom = Pose2d::new(odom.x_m + 0.1, 0.0, 0.0);
                localizer
                    .update_lidar(
                        &scan_at(start),
                        odom,
                        Pose2d::IDENTITY,
                        4.0 + f64::from(step),
                    )
                    .unwrap();
            }
            assert_eq!(localizer.source(), LocalizationSource::Lidar);
            localizer.pose().x_m
        };
        let held = run(0);
        let followed = run(10);
        assert!(held < 13.5, "{held}");
        assert!(followed > 14.4, "{followed}");
    }

    fn cloud_match(x: f64, y: f64, variance: [f64; 2]) -> PointCloudMatch {
        PointCloudMatch {
            pose: Pose2d::new(x, y, 0.0),
            z_m: 0.6,
            position_covariance_m2: [[variance[0], 0.0], [0.0, variance[1]]],
            yaw_variance_rad2: 1.0e-4,
            rmse_m: 0.02,
            inlier_fraction: 0.9,
            converged: true,
        }
    }

    #[test]
    fn point_cloud_matches_relocalize_track_and_hand_back() {
        let config = HybridLocalizerConfig {
            lidar_gate_chi_square: 16.0,
            ..HybridLocalizerConfig::default()
        };
        let mut localizer =
            HybridLocalizer::new(room_map(), Pose2d::new(13.0, 0.0, 0.0), config).unwrap();
        // GNSS mode ignores matches.
        localizer
            .update_lidar_match(Some(&cloud_match(14.0, 0.0, [0.01, 0.01])), 0.0)
            .unwrap();
        assert_eq!(localizer.source(), LocalizationSource::Gnss);
        assert!((localizer.pose().x_m - 13.0).abs() < 1e-9);
        for step in 0..3 {
            localizer.update_gnss(None, f64::from(step)).unwrap();
        }
        assert_eq!(localizer.source(), LocalizationSource::DeadReckoning);
        assert!(localizer.lidar_search_window_m() >= 0.5);
        // A match relocalizes the robot (1 m off) and starts LiDAR mode.
        localizer
            .update_lidar_match(Some(&cloud_match(14.0, 0.0, [0.01, 0.01])), 3.0)
            .unwrap();
        assert_eq!(localizer.source(), LocalizationSource::Lidar);
        assert!(
            (localizer.pose().x_m - 14.0).abs() < 0.1,
            "{:?}",
            localizer.pose()
        );
        // A match 3 m along a direction it constrains fails the gate; one
        // whose covariance says that direction is loose passes.
        localizer
            .update_lidar_match(Some(&cloud_match(17.0, 0.0, [0.01, 0.01])), 3.1)
            .unwrap();
        assert!(
            (localizer.pose().x_m - 14.0).abs() < 0.1,
            "{:?}",
            localizer.pose()
        );
        localizer
            .update_lidar_match(Some(&cloud_match(17.0, 0.05, [100.0, 0.01])), 3.2)
            .unwrap();
        let pose = localizer.pose();
        assert!((pose.x_m - 14.0).abs() < 0.1 && pose.y_m > 0.02, "{pose:?}");
        // Scans without a match end LiDAR mode.
        for step in 0..config.lidar_max_low_scores {
            localizer
                .update_lidar_match(None, 4.0 + 0.1 * step as f64)
                .unwrap();
        }
        assert_eq!(localizer.source(), LocalizationSource::DeadReckoning);
    }

    #[test]
    fn lidar_matches_that_disagree_with_the_estimate_are_not_fused() {
        let config = HybridLocalizerConfig {
            amcl: AmclConfig {
                particle_count: 300,
                ..AmclConfig::default()
            },
            lidar_gate_chi_square: 16.0,
            ..HybridLocalizerConfig::default()
        };
        let truth = Pose2d::new(14.0, -0.7, 0.05);
        let mut localizer = HybridLocalizer::new(room_map(), truth, config).unwrap();
        for step in 0..3 {
            localizer.update_gnss(None, step as f64).unwrap();
        }
        for _ in 0..10 {
            localizer
                .update_lidar(&scan_at(truth), Pose2d::IDENTITY, Pose2d::IDENTITY, 4.0)
                .unwrap();
        }
        assert_eq!(localizer.source(), LocalizationSource::Lidar);
        // Scans that match 0.5 m away (as if the room had slid) would drag a
        // well-localized estimate; the gate keeps them out.
        let elsewhere = Pose2d::new(14.5, -0.7, 0.05);
        for _ in 0..10 {
            localizer
                .update_lidar(&scan_at(elsewhere), Pose2d::IDENTITY, Pose2d::IDENTITY, 5.0)
                .unwrap();
        }
        let pose = localizer.pose();
        assert!(
            (pose.x_m - truth.x_m).hypot(pose.y_m - truth.y_m) < 0.2,
            "{pose:?}"
        );
    }

    #[test]
    fn corridor_matches_are_loose_along_the_corridor_and_tight_across_it() {
        // A 60 m corridor, 4 m wide, along x.
        let mut grid = OccupancyGrid::new(800, 100, 0.1, Pose2d::new(-40.0, -5.0, 0.0)).unwrap();
        for y in 0..grid.height() {
            for x in 0..grid.width() {
                let coord = GridCoord {
                    x: x as isize,
                    y: y as isize,
                };
                let world = grid.grid_to_world(coord);
                if world.y.abs() >= 2.0 {
                    grid.mark_occupied(coord);
                } else {
                    grid.mark_free(coord);
                }
            }
        }
        let config = HybridLocalizerConfig {
            lidar_anisotropic: true,
            ..HybridLocalizerConfig::default()
        };
        let corridor = HybridLocalizer::new(grid, Pose2d::IDENTITY, config).unwrap();
        let beams = 180;
        let ranges_m: Vec<f64> = (0..beams)
            .map(|b| {
                let angle = TAU * b as f64 / beams as f64;
                let s = angle.sin().abs();
                if s > 0.1 {
                    (2.0 / s).min(12.0)
                } else {
                    f64::INFINITY
                }
            })
            .collect();
        let scan = LaserScan2d {
            time_s: 0.0,
            frame: FrameId::new("laser"),
            angle_min_rad: 0.0,
            angle_increment_rad: TAU / beams as f64,
            range_min_m: 0.05,
            range_max_m: 12.0,
            ranges_m,
        };
        let points = scan_points_2d(&scan, 180);
        let c = corridor.match_covariance(&points, Pose2d::IDENTITY, Pose2d::IDENTITY);
        assert!(c[0][0] > 50.0 * c[1][1], "{c:?}");
        assert!(c[1][1] <= 0.011, "{c:?}");

        // The closed room with a pillar constrains both directions.
        let truth = Pose2d::new(14.0, -0.7, 0.0);
        let room = HybridLocalizer::new(room_map(), truth, config).unwrap();
        let points = scan_points_2d(&scan_at(truth), 180);
        let c = room.match_covariance(&points, Pose2d::IDENTITY, truth);
        assert!(c[0][0] < 3.0 * c[1][1] && c[1][1] < 3.0 * c[0][0], "{c:?}");
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

    #[test]
    fn a_gate_reset_outside_gnss_mode_lets_gnss_take_back_over() {
        let config = HybridLocalizerConfig {
            gate_reset_epochs: 4,
            ..HybridLocalizerConfig::default()
        };
        let mut localizer =
            HybridLocalizer::new(room_map(), Pose2d::new(-50.0, 0.0, 0.0), config).unwrap();
        // GNSS drops out: dead reckoning.
        for _ in 0..20 {
            localizer.update_gnss(None, 0.0).unwrap();
        }
        assert_eq!(localizer.source(), LocalizationSource::DeadReckoning);
        // RTK comes back 20 m from the (wrong) estimate.
        let fixed = AbsolutePositionFix {
            horizontal_std_m: 0.02,
            kind: FixKind::Fixed,
            ..good_fix(-50.0, 20.0)
        };
        for _ in 0..40 {
            localizer.update_gnss(Some(&fixed), 1.0).unwrap();
        }
        assert_eq!(localizer.source(), LocalizationSource::Gnss);
        assert!(
            (localizer.pose().y_m - 20.0).abs() < 0.1,
            "{:?}",
            localizer.pose()
        );
    }
}
