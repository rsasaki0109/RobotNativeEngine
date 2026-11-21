//! A simulated single-frequency GNSS receiver.
//!
//! Each epoch the receiver ray traces every satellite above the horizon
//! through the [`GnssScene`], builds a pseudorange from the geometric range of
//! the tracked path, and solves single-point positioning with RAIM.
//!
//! * **Timestamp**: an epoch is measured at the simulation time passed to
//!   [`GnssReceiver::observe`] / [`GnssReceiver::step`], with the antenna at
//!   the position given for that time. Satellites are propagated to the
//!   signal transmission time and rotated for Earth rotation during the
//!   signal flight (Sagnac), as a real receiver does from ephemerides.
//! * **Latency**: a fix computed from an epoch measured at `t` is released by
//!   [`GnssReceiver::step`] at `t + latency_s`; epochs fire at `update_rate_hz`.
//! * **Noise**: per-satellite orbit/clock errors (constant per run, seeded per
//!   satellite id), residual ionospheric and tropospheric delays, a receiver
//!   clock random walk, DLL thermal noise from the ray-traced C/N0, code
//!   multipath from every reflected replica, and the full excess delay of NLOS
//!   receptions. All random draws come from a seeded
//!   [`rne_core::DeterministicRng`] in satellite-id order.

use crate::geodesy::{azimuth_elevation, EnuFrame, Geodetic};
use crate::orbit::{Constellation, SatelliteId, EARTH_ROTATION_RAD_S, SPEED_OF_LIGHT_M_S};
use crate::raytrace::trace_satellite;
use crate::scene::GnssScene;
use crate::signal::{AtmosphereConfig, SignalConfig};
use crate::solver::{solve_position, Dop, PseudorangeInput, SolveError, SolverConfig};
use rne_core::DeterministicRng;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::f64::consts::TAU;

/// Receiver clock error model.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ClockConfig {
    /// Clock bias at the first epoch, in meters.
    pub initial_bias_m: f64,
    /// Clock drift at the first epoch, in m/s.
    pub initial_drift_m_s: f64,
    /// Drift random-walk density in (m/s)/√s.
    pub drift_random_walk_m_s_per_sqrt_s: f64,
}

impl Default for ClockConfig {
    fn default() -> Self {
        Self {
            initial_bias_m: 15_000.0,
            initial_drift_m_s: 0.6,
            drift_random_walk_m_s_per_sqrt_s: 0.02,
        }
    }
}

/// Receiver configuration.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ReceiverConfig {
    /// Signal and tracking parameters.
    pub signal: SignalConfig,
    /// Residual atmosphere.
    pub atmosphere: AtmosphereConfig,
    /// Position solver.
    pub solver: SolverConfig,
    /// Receiver clock.
    pub clock: ClockConfig,
    /// Satellites below this elevation are not used, in radians.
    pub elevation_mask_rad: f64,
    /// Tracked signals weaker than this are not used for the fix, in dB-Hz.
    pub fix_cn0_mask_dbhz: f64,
    /// One-sigma combined orbit and satellite clock error in meters.
    pub satellite_error_std_m: f64,
    /// Weighting model: elevation-independent sigma in meters.
    pub sigma_floor_m: f64,
    /// Weighting model: sigma scaled by `1/sin(elevation)` in meters.
    pub sigma_elevation_m: f64,
    /// Epoch rate in Hz.
    pub update_rate_hz: f64,
    /// Delay between measurement and fix output in seconds.
    pub latency_s: f64,
    /// Seed for all receiver noise.
    pub seed: u64,
}

impl Default for ReceiverConfig {
    fn default() -> Self {
        Self {
            signal: SignalConfig::gps_l1_ca(),
            atmosphere: AtmosphereConfig::default(),
            solver: SolverConfig::default(),
            clock: ClockConfig::default(),
            elevation_mask_rad: 10.0_f64.to_radians(),
            fix_cn0_mask_dbhz: 30.0,
            satellite_error_std_m: 0.6,
            sigma_floor_m: 1.0,
            sigma_elevation_m: 1.2,
            update_rate_hz: 5.0,
            latency_s: 0.05,
            seed: 0x6E55_5EED,
        }
    }
}

impl ReceiverConfig {
    /// Whether the configuration is usable.
    pub fn is_valid(&self) -> bool {
        self.signal.is_valid()
            && self.elevation_mask_rad.is_finite()
            && self.fix_cn0_mask_dbhz.is_finite()
            && self.satellite_error_std_m >= 0.0
            && self.sigma_floor_m > 0.0
            && self.sigma_elevation_m >= 0.0
            && self.update_rate_hz > 0.0
            && self.update_rate_hz.is_finite()
            && self.latency_s >= 0.0
            && self.latency_s.is_finite()
            && self.clock.initial_bias_m.is_finite()
            && self.clock.initial_drift_m_s.is_finite()
            && self.clock.drift_random_walk_m_s_per_sqrt_s >= 0.0
    }
}

/// How a satellite's signal reached the antenna (ground truth of the ray trace).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SignalPath {
    /// Below the elevation mask.
    BelowMask,
    /// Direct path clear and no wall reflection within one chip (a ground
    /// bounce may still add a small code error).
    LineOfSight,
    /// Direct path clear, with a wall-reflected replica distorting the code.
    LineOfSightMultipath,
    /// Direct path blocked; a reflected signal is tracked instead.
    NonLineOfSight,
    /// Direct path blocked and no reflection reaches the antenna.
    Blocked,
    /// A path exists but is too weak to track or to use.
    TooWeak,
}

impl SignalPath {
    /// Whether a pseudorange is produced.
    pub fn is_tracked(self) -> bool {
        matches!(
            self,
            Self::LineOfSight | Self::LineOfSightMultipath | Self::NonLineOfSight
        )
    }
}

/// One satellite in one epoch.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SatelliteObservation {
    /// Satellite id.
    pub id: SatelliteId,
    /// Azimuth from north, clockwise, in radians.
    pub azimuth_rad: f64,
    /// Elevation in radians.
    pub elevation_rad: f64,
    /// Propagation class (ground truth).
    pub path: SignalPath,
    /// Received C/N0 in dB-Hz (`-inf` when nothing arrives).
    pub cn0_dbhz: f64,
    /// Satellite ECEF position the receiver computes, in meters.
    pub satellite_ecef_m: Vec3,
    /// Pseudorange in meters, when tracked.
    pub pseudorange_m: Option<f64>,
    /// Pseudorange error excluding the receiver clock (ground truth), in meters.
    pub error_m: f64,
    /// NLOS excess path or LOS multipath code error (ground truth), in meters.
    pub multipath_error_m: f64,
    /// The reflection point of a tracked NLOS signal, in ENU meters.
    pub reflection_point_m: Option<Vec3>,
}

/// All satellites at one epoch.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GnssEpoch {
    /// Measurement time in seconds.
    pub time_s: f64,
    /// True antenna position in ENU meters.
    pub antenna_enu_m: Vec3,
    /// Observations in satellite-id order.
    pub observations: Vec<SatelliteObservation>,
}

impl GnssEpoch {
    /// Number of satellites with a given path class.
    pub fn count(&self, path: SignalPath) -> usize {
        self.observations.iter().filter(|o| o.path == path).count()
    }
}

/// Fix quality.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FixStatus {
    /// No position (fewer than four usable satellites or a failed solve).
    NoFix,
    /// Single-point position.
    SinglePoint,
}

/// A position fix, as a receiver would report it (plus ground-truth counts).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GnssFix {
    /// Time the measurements were taken, in seconds.
    pub measurement_time_s: f64,
    /// Time the fix becomes available, in seconds.
    pub available_time_s: f64,
    /// Fix status.
    pub status: FixStatus,
    /// Position in local ENU meters (zero without a fix).
    pub position_enu_m: Vec3,
    /// Geodetic position.
    pub geodetic: Geodetic,
    /// One-sigma horizontal error estimate in meters (`inf` without a fix),
    /// scaled up by the a-posteriori variance factor `chi²/dof` when the
    /// residuals are larger than the weighting model expects.
    pub horizontal_std_m: f64,
    /// One-sigma vertical error estimate in meters.
    pub vertical_std_m: f64,
    /// Dilution of precision.
    pub dop: Dop,
    /// Satellites tracked.
    pub satellites_tracked: usize,
    /// Satellites used in the solution.
    pub satellites_used: usize,
    /// Satellites RAIM excluded.
    pub excluded: Vec<SatelliteId>,
    /// Whether the residual test passed.
    pub consistent: bool,
    /// Used satellites that were actually NLOS (ground truth).
    pub nlos_used: usize,
}

impl GnssFix {
    /// Whether the fix carries a position.
    pub fn has_position(&self) -> bool {
        self.status != FixStatus::NoFix
    }
}

/// Receiver construction errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum ReceiverError {
    /// The configuration is invalid.
    #[error("invalid GNSS receiver configuration")]
    InvalidConfig,
}

/// A deterministic simulated GNSS receiver.
#[derive(Clone, Debug)]
pub struct GnssReceiver {
    config: ReceiverConfig,
    rng: DeterministicRng,
    clock_bias_m: f64,
    clock_drift_m_s: f64,
    last_time_s: Option<f64>,
    next_epoch_s: Option<f64>,
    last_position_ecef_m: Option<Vec3>,
    pending: VecDeque<GnssFix>,
    last_epoch: Option<GnssEpoch>,
}

impl GnssReceiver {
    /// Creates a receiver.
    pub fn new(config: ReceiverConfig) -> Result<Self, ReceiverError> {
        if !config.is_valid() {
            return Err(ReceiverError::InvalidConfig);
        }
        Ok(Self {
            rng: DeterministicRng::new(config.seed),
            clock_bias_m: config.clock.initial_bias_m,
            clock_drift_m_s: config.clock.initial_drift_m_s,
            config,
            last_time_s: None,
            next_epoch_s: None,
            last_position_ecef_m: None,
            pending: VecDeque::new(),
            last_epoch: None,
        })
    }

    /// Configuration.
    pub fn config(&self) -> &ReceiverConfig {
        &self.config
    }

    /// The most recent epoch measured by [`Self::step`].
    pub fn last_epoch(&self) -> Option<&GnssEpoch> {
        self.last_epoch.as_ref()
    }

    /// True receiver clock bias in meters (ground truth).
    pub fn clock_bias_m(&self) -> f64 {
        self.clock_bias_m
    }

    /// Advances the simulation to `time_s`: measures an epoch when one is due
    /// and returns every fix whose latency has elapsed.
    pub fn step(
        &mut self,
        scene: &GnssScene,
        constellation: &Constellation,
        frame: &EnuFrame,
        antenna_enu_m: Vec3,
        time_s: f64,
    ) -> Vec<GnssFix> {
        const EPS: f64 = 1.0e-9;
        let period = 1.0 / self.config.update_rate_hz;
        let next = *self.next_epoch_s.get_or_insert(time_s);
        if time_s + EPS >= next {
            let epoch = self.observe(scene, constellation, frame, antenna_enu_m, time_s);
            let fix = self.solve(&epoch, frame);
            self.pending.push_back(fix);
            self.last_epoch = Some(epoch);
            let mut next_epoch = next + period;
            while next_epoch <= time_s + EPS {
                next_epoch += period;
            }
            self.next_epoch_s = Some(next_epoch);
        }
        let mut released = Vec::new();
        while self
            .pending
            .front()
            .is_some_and(|fix| fix.available_time_s <= time_s + EPS)
        {
            released.extend(self.pending.pop_front());
        }
        released
    }

    /// Measures one epoch at `time_s` with the antenna at `antenna_enu_m`.
    pub fn observe(
        &mut self,
        scene: &GnssScene,
        constellation: &Constellation,
        frame: &EnuFrame,
        antenna_enu_m: Vec3,
        time_s: f64,
    ) -> GnssEpoch {
        self.advance_clock(time_s);
        let receiver_ecef = frame.enu_to_ecef(antenna_enu_m);
        let signal = self.config.signal;
        let mut observations = Vec::with_capacity(constellation.len());
        for satellite in constellation.satellites() {
            // Always draw the same numbers per satellite so that one
            // satellite's visibility never shifts another's noise.
            let thermal = gaussian(&mut self.rng);

            let satellite_ecef = transmit_position(satellite.orbit, receiver_ecef, time_s);
            let line_ecef = satellite_ecef - receiver_ecef;
            let range = line_ecef.length();
            let direction = frame.ecef_vector_to_enu(line_ecef / range);
            let (azimuth_rad, elevation_rad) = azimuth_elevation(direction);
            let mut observation = SatelliteObservation {
                id: satellite.id,
                azimuth_rad,
                elevation_rad,
                path: SignalPath::BelowMask,
                cn0_dbhz: f64::NEG_INFINITY,
                satellite_ecef_m: satellite_ecef,
                pseudorange_m: None,
                error_m: 0.0,
                multipath_error_m: 0.0,
                reflection_point_m: None,
            };
            if elevation_rad < self.config.elevation_mask_rad.max(0.0) {
                observations.push(observation);
                continue;
            }

            let propagation = trace_satellite(scene, antenna_enu_m, direction);
            let replica = |delay_m: f64, arrival: Vec3, loss_db: f64| {
                let (_, arrival_el) = azimuth_elevation(arrival);
                (signal.antenna_cn0_dbhz(arrival_el) - loss_db, delay_m)
            };
            let (cn0, multipath_m, reflection_point) = if propagation.line_of_sight {
                let direct_cn0 = signal.antenna_cn0_dbhz(elevation_rad);
                let replicas: Vec<(f64, f64)> = propagation
                    .reflections
                    .iter()
                    .map(|r| {
                        let (cn0, delay) =
                            replica(r.excess_delay_m, r.arrival_direction, r.reflection_loss_db);
                        (10.0_f64.powf((cn0 - direct_cn0) / 20.0), delay)
                    })
                    .collect();
                let code_error: f64 = replicas
                    .iter()
                    .map(|&(amplitude, delay)| signal.multipath_code_error_m(amplitude, delay))
                    .sum();
                // Ground bounces under the antenna are always present and
                // weak; only a wall reflection within one chip counts as
                // multipath for classification (both affect the error).
                let wall_replica = propagation.reflections.iter().any(|r| {
                    r.source != crate::scene::FacetSource::Ground
                        && r.excess_delay_m < signal.chip_length_m
                });
                observation.path = if wall_replica {
                    SignalPath::LineOfSightMultipath
                } else {
                    SignalPath::LineOfSight
                };
                (
                    direct_cn0 + signal.multipath_fading_db(replicas.iter().copied()),
                    code_error,
                    None,
                )
            } else if let Some(strongest) = propagation.strongest_reflection() {
                let (cn0, delay) = replica(
                    strongest.excess_delay_m,
                    strongest.arrival_direction,
                    strongest.reflection_loss_db,
                );
                observation.path = SignalPath::NonLineOfSight;
                (cn0, delay, Some(strongest.point_m))
            } else {
                observation.path = SignalPath::Blocked;
                observations.push(observation);
                continue;
            };

            observation.cn0_dbhz = cn0;
            observation.multipath_error_m = multipath_m;
            observation.reflection_point_m = reflection_point;
            if cn0 < signal.tracking_threshold_dbhz {
                observation.path = SignalPath::TooWeak;
                observations.push(observation);
                continue;
            }
            let error_m = satellite_error_m(
                self.config.seed,
                satellite.id,
                self.config.satellite_error_std_m,
            ) + self.config.atmosphere.residual_delay_m(elevation_rad)
                + multipath_m
                + thermal * signal.dll_noise_std_m(cn0);
            observation.error_m = error_m;
            observation.pseudorange_m = Some(range + self.clock_bias_m + error_m);
            observations.push(observation);
        }
        GnssEpoch {
            time_s,
            antenna_enu_m,
            observations,
        }
    }

    /// Solves a position from an epoch's tracked pseudoranges.
    pub fn solve(&mut self, epoch: &GnssEpoch, frame: &EnuFrame) -> GnssFix {
        let usable: Vec<&SatelliteObservation> = epoch
            .observations
            .iter()
            .filter(|o| o.path.is_tracked() && o.cn0_dbhz >= self.config.fix_cn0_mask_dbhz)
            .collect();
        let inputs: Vec<PseudorangeInput> = usable
            .iter()
            .filter_map(|o| {
                Some(PseudorangeInput {
                    id: o.id,
                    satellite_ecef_m: o.satellite_ecef_m,
                    pseudorange_m: o.pseudorange_m?,
                    sigma_m: self.measurement_sigma_m(o.elevation_rad, o.cn0_dbhz),
                })
            })
            .collect();
        let tracked = epoch
            .observations
            .iter()
            .filter(|o| o.path.is_tracked())
            .count();
        let initial = self.last_position_ecef_m.unwrap_or(frame.origin_ecef_m);
        let available_time_s = epoch.time_s + self.config.latency_s;
        let no_fix = |dop: Dop| GnssFix {
            measurement_time_s: epoch.time_s,
            available_time_s,
            status: FixStatus::NoFix,
            position_enu_m: Vec3::ZERO,
            geodetic: frame.datum,
            horizontal_std_m: f64::INFINITY,
            vertical_std_m: f64::INFINITY,
            dop,
            satellites_tracked: tracked,
            satellites_used: 0,
            excluded: Vec::new(),
            consistent: false,
            nlos_used: 0,
        };
        let solution = match solve_position(&inputs, initial, frame, &self.config.solver) {
            Ok(solution) => solution,
            Err(SolveError::TooFewSatellites | SolveError::Degenerate | SolveError::Diverged) => {
                return no_fix(Dop::default());
            }
        };
        self.last_position_ecef_m = Some(solution.position_ecef_m);
        let nlos_used = solution
            .used
            .iter()
            .filter(|id| {
                usable
                    .iter()
                    .any(|o| o.id == **id && o.path == SignalPath::NonLineOfSight)
            })
            .count();
        // Scale the a-priori covariance by the a-posteriori variance factor
        // so that a solution with large residuals reports itself as poor.
        let dof = solution.used.len().saturating_sub(4);
        let variance_factor = if dof > 0 {
            (solution.chi_square / dof as f64).max(1.0)
        } else {
            1.0
        };
        let cov = solution
            .covariance_enu_m2
            .map(|row| row.map(|v| v * variance_factor));
        GnssFix {
            measurement_time_s: epoch.time_s,
            available_time_s,
            status: FixStatus::SinglePoint,
            position_enu_m: frame.ecef_to_enu(solution.position_ecef_m),
            geodetic: Geodetic::from_ecef(solution.position_ecef_m),
            horizontal_std_m: (cov[0][0] + cov[1][1]).max(0.0).sqrt(),
            vertical_std_m: cov[2][2].max(0.0).sqrt(),
            dop: solution.dop,
            satellites_tracked: tracked,
            satellites_used: solution.used.len(),
            excluded: solution.excluded,
            consistent: solution.consistent,
            nlos_used,
        }
    }

    /// The a-priori measurement sigma used to weight a pseudorange.
    pub fn measurement_sigma_m(&self, elevation_rad: f64, cn0_dbhz: f64) -> f64 {
        let sin_el = elevation_rad.sin().max(0.05);
        let elevation_term = self.config.sigma_elevation_m / sin_el;
        let thermal = self.config.signal.dll_noise_std_m(cn0_dbhz);
        (self.config.sigma_floor_m.powi(2) + elevation_term.powi(2) + thermal.powi(2)).sqrt()
    }

    fn advance_clock(&mut self, time_s: f64) {
        let dt = self
            .last_time_s
            .map_or(0.0, |last| (time_s - last).max(0.0));
        self.last_time_s = Some(time_s);
        if dt > 0.0 {
            let walk = self.config.clock.drift_random_walk_m_s_per_sqrt_s * dt.sqrt();
            self.clock_drift_m_s += walk * gaussian(&mut self.rng);
            self.clock_bias_m += self.clock_drift_m_s * dt;
        }
    }
}

/// Satellite position at signal transmission, expressed in the Earth-fixed
/// frame at reception (light-time iteration plus Sagnac rotation).
fn transmit_position(orbit: crate::orbit::KeplerOrbit, receiver_ecef_m: Vec3, time_s: f64) -> Vec3 {
    let mut flight_s = 0.075;
    let mut rotated = Vec3::ZERO;
    for _ in 0..3 {
        let at_transmit = orbit.position_ecef(time_s - flight_s);
        let angle = EARTH_ROTATION_RAD_S * flight_s;
        let (sin, cos) = angle.sin_cos();
        rotated = Vec3::new(
            cos * at_transmit.x + sin * at_transmit.y,
            -sin * at_transmit.x + cos * at_transmit.y,
            at_transmit.z,
        );
        flight_s = (rotated - receiver_ecef_m).length() / SPEED_OF_LIGHT_M_S;
    }
    rotated
}

/// Constant orbit-and-clock error of one satellite, seeded by its id.
fn satellite_error_m(seed: u64, id: SatelliteId, std_m: f64) -> f64 {
    let key = ((id.system as u64) << 8) | u64::from(id.prn);
    let mut rng = DeterministicRng::new(seed ^ key.wrapping_mul(0x9E37_79B9_7F4A_7C15));
    std_m * gaussian(&mut rng)
}

fn gaussian(rng: &mut DeterministicRng) -> f64 {
    let u1 = rng.uniform_f64(1.0e-12, 1.0);
    let u2 = rng.uniform_f64(0.0, 1.0);
    (-2.0 * u1.ln()).sqrt() * (TAU * u2).cos()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::{SceneBox, SurfaceMaterial};

    fn frame() -> EnuFrame {
        EnuFrame::new(Geodetic::from_degrees(35.68, 139.76, 40.0))
    }

    fn horizontal_error(fix: &GnssFix, truth: Vec3) -> f64 {
        (fix.position_enu_m.x - truth.x).hypot(fix.position_enu_m.y - truth.y)
    }

    #[test]
    fn open_sky_fix_is_metre_level() {
        let scene = GnssScene::open_sky();
        let constellation = Constellation::gps_galileo_nominal();
        let mut receiver = GnssReceiver::new(ReceiverConfig::default()).unwrap();
        let antenna = Vec3::new(5.0, 3.0, 1.0);
        let mut errors = Vec::new();
        for step in 0..50 {
            let epoch =
                receiver.observe(&scene, &constellation, &frame(), antenna, step as f64 * 0.2);
            assert_eq!(epoch.count(SignalPath::NonLineOfSight), 0);
            let fix = receiver.solve(&epoch, &frame());
            assert!(fix.has_position());
            errors.push(horizontal_error(&fix, antenna));
        }
        let mean = errors.iter().sum::<f64>() / errors.len() as f64;
        assert!(mean < 3.0, "mean horizontal error {mean}");
    }

    #[test]
    fn roofed_room_has_no_fix() {
        let mut scene = GnssScene::open_sky();
        scene.add_box(SceneBox::new(
            Vec3::new(-20.0, -20.0, 3.0),
            Vec3::new(20.0, 20.0, 3.3),
            SurfaceMaterial::CONCRETE,
        ));
        let constellation = Constellation::gps_galileo_nominal();
        let mut receiver = GnssReceiver::new(ReceiverConfig::default()).unwrap();
        let epoch = receiver.observe(
            &scene,
            &constellation,
            &frame(),
            Vec3::new(0.0, 0.0, 1.0),
            0.0,
        );
        let fix = receiver.solve(&epoch, &frame());
        assert_eq!(fix.status, FixStatus::NoFix);
        assert_eq!(fix.satellites_tracked, 0);
    }

    #[test]
    fn urban_canyon_produces_nlos_receptions() {
        let mut scene = GnssScene::open_sky();
        scene.add_box(SceneBox::building(
            -200.0,
            10.0,
            200.0,
            30.0,
            45.0,
            SurfaceMaterial::GLASS,
        ));
        scene.add_box(SceneBox::building(
            -200.0,
            -30.0,
            200.0,
            -10.0,
            45.0,
            SurfaceMaterial::GLASS,
        ));
        let constellation = Constellation::gps_galileo_nominal();
        let mut receiver = GnssReceiver::new(ReceiverConfig::default()).unwrap();
        let mut nlos = 0;
        let mut blocked = 0;
        for step in 0..20 {
            let epoch = receiver.observe(
                &scene,
                &constellation,
                &frame(),
                Vec3::new(0.0, 0.0, 1.0),
                step as f64 * 600.0,
            );
            nlos += epoch.count(SignalPath::NonLineOfSight);
            blocked += epoch.count(SignalPath::Blocked);
            for o in &epoch.observations {
                if o.path == SignalPath::NonLineOfSight {
                    // NLOS errors are dominated by the positive excess path.
                    assert!(o.multipath_error_m > 0.0);
                }
            }
        }
        assert!(nlos > 0, "no NLOS receptions");
        assert!(blocked + nlos > 0);
    }

    #[test]
    fn step_applies_rate_and_latency() {
        let scene = GnssScene::open_sky();
        let constellation = Constellation::gps_galileo_nominal();
        let config = ReceiverConfig {
            update_rate_hz: 2.0,
            latency_s: 0.1,
            ..ReceiverConfig::default()
        };
        let mut receiver = GnssReceiver::new(config).unwrap();
        let mut released = Vec::new();
        for step in 0..=20 {
            let t = step as f64 * 0.05;
            for fix in receiver.step(&scene, &constellation, &frame(), Vec3::ZERO, t) {
                assert!((fix.available_time_s - t).abs() < 1e-9);
                released.push(fix.measurement_time_s);
            }
        }
        // Epochs at 0.0 and 0.5 (released at 0.1 and 0.6); 1.0 is not yet out.
        assert_eq!(released.len(), 2);
        assert!((released[0] - 0.0).abs() < 1e-9 && (released[1] - 0.5).abs() < 1e-9);
    }

    #[test]
    fn replay_is_bit_identical() {
        let mut scene = GnssScene::open_sky();
        scene.add_box(SceneBox::building(
            10.0,
            -30.0,
            30.0,
            30.0,
            40.0,
            SurfaceMaterial::GLASS,
        ));
        let constellation = Constellation::gps_galileo_nominal();
        let run = || {
            let mut receiver = GnssReceiver::new(ReceiverConfig::default()).unwrap();
            (0..10)
                .map(|step| {
                    let epoch = receiver.observe(
                        &scene,
                        &constellation,
                        &frame(),
                        Vec3::new(step as f64, 0.0, 1.0),
                        step as f64,
                    );
                    receiver.solve(&epoch, &frame()).position_enu_m
                })
                .collect::<Vec<_>>()
        };
        assert_eq!(run(), run());
    }
}
