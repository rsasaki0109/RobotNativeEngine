//! The robot's GNSS: two antennas on the body, a base station on a rooftop,
//! and libgnss++ RTK (rear antenna against the base) plus moving-baseline RTK
//! (front antenna against the rear) for heading. Without libgnss++ the rear
//! antenna's built-in single-point fixes are used instead.

use rne_adapter_gnsspp::{Navigation, Rtk, RtkConfig, RtkMode, Solution, SolutionStatus};
use rne_gnss::{
    AntennaRig, BodyPose, Constellation, EnuFrame, GnssEpoch, GnssFix, GnssReceiver, GnssScene,
    NavigationMessage, ReceiverConfig,
};
use rne_math::Vec3;
use rne_slam::{AbsoluteHeadingFix, AbsolutePositionFix, FixKind};

/// Antenna lever arms in the body frame: front and rear, 0.8 m apart.
pub(crate) const LEVER_ARMS_M: [Vec3; 2] = [Vec3::new(0.4, 0.0, 0.5), Vec3::new(-0.4, 0.0, 0.5)];
const FRONT: usize = 0;
const REAR: usize = 1;
/// Height of the body origin above the ground.
pub(crate) const BODY_HEIGHT_M: f64 = 0.3;
/// Base-station antenna: on the roof of the 40 m glass tower.
pub(crate) const BASE_ENU_M: Vec3 = Vec3::new(110.0, 28.0, 41.5);
const ELEVATION_MASK_DEG: f64 = 10.0;

/// What the GNSS produced at one epoch.
#[derive(Clone, Debug)]
pub(crate) struct GnssOutput {
    /// Body-origin position fix for the localizer, if any.
    pub fix: Option<AbsolutePositionFix>,
    /// Solution label for the logs.
    pub label: SolutionLabel,
    /// Moving-baseline heading, if fixed.
    pub heading: Option<AbsoluteHeadingFix>,
}

/// Solution type for statistics.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SolutionLabel {
    /// No position.
    None,
    /// Single point (or code differential).
    Single,
    /// RTK float.
    Float,
    /// RTK fixed.
    Fixed,
}

enum Solver {
    Libgnss {
        navigation: Navigation,
        rtk: Rtk,
        heading: Rtk,
        base: Box<GnssReceiver>,
    },
    BuiltIn,
}

/// The robot's GNSS pipeline.
pub(crate) struct RobotGnss {
    rig: AntennaRig,
    solver: Solver,
    frame: EnuFrame,
}

impl std::fmt::Debug for RobotGnss {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RobotGnss")
            .field("libgnss", &self.uses_libgnss())
            .finish()
    }
}

impl RobotGnss {
    /// Builds the rig and, when libgnss++ is linked, the RTK solvers.
    pub(crate) fn new(
        constellation: &Constellation,
        frame: EnuFrame,
        config: ReceiverConfig,
        reference_s: f64,
    ) -> Self {
        let rig = AntennaRig::new(&LEVER_ARMS_M, config).expect("rig");
        let solver = (|| {
            let message = NavigationMessage::from_constellation(
                constellation,
                config.epoch,
                reference_s,
                config.atmosphere.klobuchar,
            );
            let navigation = Navigation::new(&message).ok()?;
            // libgnss++'s urban "low-cost" tuning, with a clean ambiguity
            // reacquisition after 10 s without a fix (canyon exits).
            let mut rtk = Rtk::with_config(&RtkConfig {
                elevation_mask_deg: ELEVATION_MASK_DEG,
                preset: "low-cost".to_string(),
                max_nonfix_for_reset: 50,
                ..RtkConfig::new(RtkMode::Kinematic)
            })
            .ok()?;
            rtk.set_base_position(frame.enu_to_ecef(BASE_ENU_M)).ok()?;
            let heading = Rtk::with_config(&RtkConfig {
                elevation_mask_deg: ELEVATION_MASK_DEG,
                preset: "moving-base".to_string(),
                max_nonfix_for_reset: 50,
                ..RtkConfig::new(RtkMode::MovingBase)
            })
            .ok()?;
            let base = GnssReceiver::new(ReceiverConfig {
                seed: config.seed ^ 0xBA5E,
                ..config
            })
            .ok()?;
            Some(Solver::Libgnss {
                navigation,
                rtk,
                heading,
                base: Box::new(base),
            })
        })()
        .unwrap_or(Solver::BuiltIn);
        Self { rig, solver, frame }
    }

    /// Whether libgnss++ RTK is in use.
    pub(crate) fn uses_libgnss(&self) -> bool {
        matches!(self.solver, Solver::Libgnss { .. })
    }

    /// The rear antenna's last epoch (ground truth for statistics).
    pub(crate) fn rear_epoch(&self) -> Option<&GnssEpoch> {
        self.rig.antennas()[REAR].receiver.last_epoch()
    }

    /// The base station's last epoch.
    pub(crate) fn base_epoch(&self) -> Option<&GnssEpoch> {
        match &self.solver {
            Solver::Libgnss { base, .. } => base.last_epoch(),
            Solver::BuiltIn => None,
        }
    }

    /// Steps all receivers to `time_s` with the body at its true pose.
    /// `estimate` (the localizer's body pose) moves the antenna fix to the
    /// body origin and, with a `city_model`, predicts from that 3D model
    /// which satellites are hidden at each antenna; their measurements are
    /// withheld from RTK (3D-map-aided satellite selection).
    pub(crate) fn step(
        &mut self,
        scene: &GnssScene,
        city_model: Option<&GnssScene>,
        constellation: &Constellation,
        truth: BodyPose,
        estimate: BodyPose,
        time_s: f64,
    ) -> Vec<GnssOutput> {
        let estimated_yaw_rad = estimate.yaw_rad;
        let frame = self.frame;
        let released = self.rig.step(scene, constellation, &frame, truth, time_s);
        match &mut self.solver {
            Solver::BuiltIn => released[REAR]
                .iter()
                .map(|fix| builtin_output(fix, estimated_yaw_rad))
                .collect(),
            Solver::Libgnss {
                navigation,
                rtk,
                heading,
                base,
            } => {
                base.step(scene, constellation, &frame, BASE_ENU_M, time_s);
                let rear_receiver = &self.rig.antennas()[REAR].receiver;
                if !rear_receiver.measured_at(time_s) || !base.measured_at(time_s) {
                    return Vec::new();
                }
                let (Some(rear_epoch), Some(front_epoch), Some(base_epoch)) = (
                    rear_receiver.last_epoch(),
                    self.rig.antennas()[FRONT].receiver.last_epoch(),
                    base.last_epoch(),
                ) else {
                    return Vec::new();
                };
                let (rear_epoch, front_epoch) = if let Some(city_model) = city_model {
                    (
                        mask_predicted_nlos(
                            city_model,
                            rear_epoch,
                            estimate.transform(LEVER_ARMS_M[REAR]),
                        ),
                        mask_predicted_nlos(
                            city_model,
                            front_epoch,
                            estimate.transform(LEVER_ARMS_M[FRONT]),
                        ),
                    )
                } else {
                    (rear_epoch.clone(), front_epoch.clone())
                };
                let (rear_epoch, front_epoch) = (&rear_epoch, &front_epoch);
                let base_ecef = frame.enu_to_ecef(BASE_ENU_M);
                let Ok(solution) =
                    rtk.process(rear_epoch, None, base_epoch, Some(base_ecef), navigation)
                else {
                    return Vec::new();
                };
                let label = label(solution.status);
                let fix = solution_fix(&frame, &solution, estimated_yaw_rad, time_s);
                // Moving base: the rear antenna is the base, at its solution.
                let heading_fix = solution.status.has_position().then(|| {
                    heading
                        .process(
                            front_epoch,
                            None,
                            rear_epoch,
                            Some(solution.position_ecef_m),
                            navigation,
                        )
                        .ok()
                        .and_then(|baseline| {
                            moving_base_heading(&frame, &baseline, solution.position_ecef_m, time_s)
                        })
                });
                vec![GnssOutput {
                    fix,
                    label,
                    heading: heading_fix.flatten(),
                }]
            }
        }
    }
}

/// Withholds the measurements of satellites that the city model predicts are
/// hidden from `antenna_enu_m` (the estimated antenna position): their
/// signals can only be reflections.
fn mask_predicted_nlos(scene: &GnssScene, epoch: &GnssEpoch, antenna_enu_m: Vec3) -> GnssEpoch {
    let mut masked = epoch.clone();
    for satellite in &mut masked.observations {
        let (sin_az, cos_az) = satellite.azimuth_rad.sin_cos();
        let (sin_el, cos_el) = satellite.elevation_rad.sin_cos();
        let direction = Vec3::new(sin_az * cos_el, cos_az * cos_el, sin_el);
        if scene.occluded(antenna_enu_m, direction, f64::INFINITY) {
            for signal in &mut satellite.signals {
                signal.pseudorange_m = None;
                signal.carrier_phase_cycles = None;
                signal.doppler_hz = None;
            }
        }
    }
    masked
}

fn label(status: SolutionStatus) -> SolutionLabel {
    match status {
        SolutionStatus::Fixed => SolutionLabel::Fixed,
        SolutionStatus::Float => SolutionLabel::Float,
        SolutionStatus::None => SolutionLabel::None,
        _ => SolutionLabel::Single,
    }
}

/// Body origin from the rear antenna's position and the heading.
fn body_from_rear(rear_enu: Vec3, yaw_rad: f64) -> (f64, f64) {
    let lever = LEVER_ARMS_M[REAR];
    let (sin, cos) = yaw_rad.sin_cos();
    (
        rear_enu.x - (cos * lever.x - sin * lever.y),
        rear_enu.y - (sin * lever.x + cos * lever.y),
    )
}

fn solution_fix(
    frame: &EnuFrame,
    solution: &Solution,
    yaw_rad: f64,
    time_s: f64,
) -> Option<AbsolutePositionFix> {
    if !solution.status.has_position() {
        return None;
    }
    let enu = frame.ecef_to_enu(solution.position_ecef_m);
    let (x_m, y_m) = body_from_rear(enu, yaw_rad);
    let kind = match solution.status {
        SolutionStatus::Fixed => FixKind::Fixed,
        SolutionStatus::Float => FixKind::Float,
        _ => FixKind::Single,
    };
    // Horizontal sigma from the ECEF covariance rotated into ENU.
    let c = solution.covariance_ecef_m2;
    let east = frame.enu_vector_to_ecef(Vec3::X);
    let north = frame.enu_vector_to_ecef(Vec3::Y);
    let quad = |a: Vec3| {
        let v = [a.x, a.y, a.z];
        (0..3)
            .map(|i| (0..3).map(|j| v[i] * c[i][j] * v[j]).sum::<f64>())
            .sum::<f64>()
    };
    let horizontal = (quad(east) + quad(north)).sqrt();
    let fallback = match kind {
        FixKind::Fixed => 0.02,
        FixKind::Float => 0.5,
        FixKind::Single => 3.0,
    };
    let hdop = if solution.hdop.is_finite() && solution.hdop < 99.0 {
        solution.hdop
    } else {
        1.5
    };
    Some(AbsolutePositionFix {
        time_s,
        x_m,
        y_m,
        horizontal_std_m: if horizontal.is_finite() && horizontal > 0.0 {
            horizontal
        } else {
            fallback
        },
        satellites_used: usize::try_from(solution.satellites).unwrap_or(0),
        hdop,
        consistent: true,
        satellites_excluded: 0,
        kind,
    })
}

/// Heading from a fixed moving baseline whose length matches the rig.
fn moving_base_heading(
    frame: &EnuFrame,
    baseline: &Solution,
    rear_ecef: Vec3,
    time_s: f64,
) -> Option<AbsoluteHeadingFix> {
    if baseline.status != SolutionStatus::Fixed {
        return None;
    }
    let vector = frame.ecef_vector_to_enu(baseline.position_ecef_m - rear_ecef);
    let expected = (LEVER_ARMS_M[FRONT] - LEVER_ARMS_M[REAR]).length();
    let horizontal = vector.x.hypot(vector.y);
    if (vector.length() - expected).abs() > 0.05 || horizontal < 0.5 * expected {
        return None;
    }
    Some(AbsoluteHeadingFix {
        time_s,
        yaw_rad: vector.y.atan2(vector.x),
        // ~1 cm of relative position error across the baseline.
        yaw_std_rad: (0.01 / horizontal).max(0.005),
    })
}

fn builtin_output(fix: &GnssFix, yaw_rad: f64) -> GnssOutput {
    let position = fix.has_position().then(|| {
        let (x_m, y_m) = body_from_rear(fix.position_enu_m, yaw_rad);
        AbsolutePositionFix {
            time_s: fix.measurement_time_s,
            x_m,
            y_m,
            horizontal_std_m: fix.horizontal_std_m,
            satellites_used: fix.satellites_used,
            hdop: fix.dop.hdop,
            consistent: fix.consistent,
            satellites_excluded: fix.excluded.len(),
            kind: FixKind::Single,
        }
    });
    GnssOutput {
        label: if position.is_some() {
            SolutionLabel::Single
        } else {
            SolutionLabel::None
        },
        fix: position,
        heading: None,
    }
}
