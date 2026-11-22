//! A simulated multi-frequency GNSS receiver.
//!
//! Each epoch the receiver ray traces every satellite above the horizon
//! through the [`GnssScene`] (direct, multi-bounce reflected, and diffracted
//! paths), tracks the strongest path of every signal, and produces RINEX-style
//! observations: pseudorange, carrier phase with an integer ambiguity per lock
//! arc, Doppler, C/N0, and a loss-of-lock flag. A built-in weighted least
//! squares solver turns the primary-signal pseudoranges into a fix; external
//! solvers (RTK, PPP) consume the observations directly.
//!
//! * **Timestamp**: an epoch is measured at the simulation time passed to
//!   [`GnssReceiver::observe`] / [`GnssReceiver::step`] (GPS time
//!   `config.epoch + time_s`), with the antenna at the position given for that
//!   time. Satellites are propagated to the transmission time and rotated for
//!   Earth rotation during the flight (Sagnac).
//! * **Latency**: a fix measured at `t` is released by [`GnssReceiver::step`]
//!   at `t + latency_s`; epochs fire at `update_rate_hz`.
//! * **Noise**: per-satellite orbit/clock errors (constant per run, seeded by
//!   satellite id and shared by every receiver), true ionosphere and troposphere (the broadcast models
//!   scaled by the truth factors), a receiver clock random walk, DLL and PLL
//!   thermal noise from the ray-traced C/N0, code and carrier multipath from
//!   every replica sharing the tracked correlation peak, the full excess path
//!   of NLOS and diffracted receptions, C/N0-dependent cycle slips, a slip
//!   whenever the tracked path changes (its phase jumps by the change in
//!   path length), and
//!   Doppler noise. All draws come from a seeded
//!   [`rne_core::DeterministicRng`]: a fixed number per satellite and signal,
//!   in satellite-id order, so one satellite's visibility never shifts
//!   another's noise. Ambiguities are seeded by satellite, signal, and arc.

use crate::atmosphere::{saastamoinen_delay_m, AtmosphereConfig};
use crate::geodesy::{azimuth_elevation, EnuFrame, Geodetic};
use crate::orbit::{
    Constellation, KeplerOrbit, SatelliteId, EARTH_ROTATION_RAD_S, SPEED_OF_LIGHT_M_S,
};
use crate::raytrace::{trace_satellite, PathKind, Propagation, TraceConfig};
use crate::scene::{FacetSource, GnssScene};
use crate::signal::{SignalConfig, SignalKind};
use crate::solver::{solve_position, Dop, PseudorangeInput, SolverConfig};
use crate::time::GnssTime;
use rne_core::DeterministicRng;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, VecDeque};
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
    /// Antenna pattern and tracking loops.
    pub signal: SignalConfig,
    /// Whether to track the second frequency (GPS L2C, Galileo E5a).
    pub dual_frequency: bool,
    /// Atmosphere truth relative to the broadcast models.
    pub atmosphere: AtmosphereConfig,
    /// Ray-tracing options.
    pub trace: TraceConfig,
    /// Built-in position solver.
    pub solver: SolverConfig,
    /// Receiver clock.
    pub clock: ClockConfig,
    /// GPS time of simulation time zero (and of the orbits' time zero).
    pub epoch: GnssTime,
    /// Satellites below this elevation are not tracked, in radians.
    pub elevation_mask_rad: f64,
    /// Tracked signals weaker than this are not used for the built-in fix.
    pub fix_cn0_mask_dbhz: f64,
    /// One-sigma combined orbit and satellite clock error in meters.
    pub satellite_error_std_m: f64,
    /// Seed of the satellite errors. They belong to the satellites, so every
    /// receiver in a scenario (rover antennas, base station) must share it;
    /// they cancel in between-receiver differences as in reality.
    pub satellite_error_seed: u64,
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
            signal: SignalConfig::default(),
            dual_frequency: true,
            atmosphere: AtmosphereConfig::default(),
            trace: TraceConfig::default(),
            solver: SolverConfig::default(),
            clock: ClockConfig::default(),
            epoch: GnssTime::default(),
            elevation_mask_rad: 10.0_f64.to_radians(),
            fix_cn0_mask_dbhz: 30.0,
            satellite_error_std_m: 0.6,
            satellite_error_seed: 0x5A7E_1117,
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
            && self.atmosphere.is_valid()
            && self.trace.max_diffraction_excess_m.is_finite()
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

    /// The signals tracked for a satellite.
    pub fn tracked_signals(&self, id: SatelliteId) -> impl Iterator<Item = SignalKind> + '_ {
        SignalKind::ALL
            .into_iter()
            .filter(move |s| s.system() == id.system && (s.is_primary() || self.dual_frequency))
    }
}

/// How a signal reaches the antenna (ground truth of the ray trace).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SignalPath {
    /// Below the elevation mask.
    BelowMask,
    /// Direct path clear and no wall reflection within one chip (a ground
    /// bounce may still add a small error).
    LineOfSight,
    /// Direct path clear, with a wall-reflected replica distorting the code.
    LineOfSightMultipath,
    /// Direct path blocked; a reflected signal is tracked instead.
    NonLineOfSight,
    /// Direct path blocked; a signal diffracted over a building edge is
    /// tracked instead.
    Diffracted,
    /// Direct path blocked and nothing else reaches the antenna.
    Blocked,
    /// A path exists but is too weak to track.
    TooWeak,
}

impl SignalPath {
    /// Whether a pseudorange is produced.
    pub fn is_tracked(self) -> bool {
        matches!(
            self,
            Self::LineOfSight
                | Self::LineOfSightMultipath
                | Self::NonLineOfSight
                | Self::Diffracted
        )
    }

    /// Whether the tracked signal is not the direct one.
    pub fn is_indirect(self) -> bool {
        matches!(self, Self::NonLineOfSight | Self::Diffracted)
    }
}

/// One signal of one satellite in one epoch.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SignalObservation {
    /// Signal.
    pub signal: SignalKind,
    /// Propagation class (ground truth).
    pub path: SignalPath,
    /// Received C/N0 in dB-Hz (`-inf` when nothing arrives).
    pub cn0_dbhz: f64,
    /// Pseudorange in meters, when code is tracked.
    pub pseudorange_m: Option<f64>,
    /// Carrier phase in cycles (RINEX sign convention), when phase-locked.
    pub carrier_phase_cycles: Option<f64>,
    /// Doppler in Hz (positive when approaching), when tracked continuously.
    pub doppler_hz: Option<f64>,
    /// Loss-of-lock indicator: the phase starts a new ambiguity arc here.
    pub loss_of_lock: bool,
    /// Pseudorange error excluding the receiver clock (ground truth), meters.
    pub code_error_m: f64,
    /// Carrier-phase error excluding the receiver clock and the integer
    /// ambiguity (ground truth), meters.
    pub phase_error_m: f64,
    /// Extra path of the tracked signal over the direct one (ground truth).
    pub excess_path_m: f64,
    /// The first interaction point of a tracked indirect signal, ENU meters.
    pub interaction_point_m: Option<Vec3>,
}

impl SignalObservation {
    fn untracked(signal: SignalKind, path: SignalPath) -> Self {
        Self {
            signal,
            path,
            cn0_dbhz: f64::NEG_INFINITY,
            pseudorange_m: None,
            carrier_phase_cycles: None,
            doppler_hz: None,
            loss_of_lock: false,
            code_error_m: 0.0,
            phase_error_m: 0.0,
            excess_path_m: 0.0,
            interaction_point_m: None,
        }
    }
}

/// One satellite in one epoch.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SatelliteObservation {
    /// Satellite id.
    pub id: SatelliteId,
    /// Azimuth from north, clockwise, in radians.
    pub azimuth_rad: f64,
    /// Elevation in radians.
    pub elevation_rad: f64,
    /// Satellite ECEF position at transmission, in the reception-time frame.
    pub satellite_ecef_m: Vec3,
    /// Signals tracked for this satellite (primary first).
    pub signals: Vec<SignalObservation>,
}

impl SatelliteObservation {
    /// The primary (L1-band) signal.
    pub fn primary(&self) -> Option<&SignalObservation> {
        self.signals.iter().find(|s| s.signal.is_primary())
    }

    /// The primary signal's propagation class.
    pub fn path(&self) -> SignalPath {
        self.primary().map_or(SignalPath::BelowMask, |s| s.path)
    }
}

/// All satellites at one epoch.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GnssEpoch {
    /// Simulation time in seconds.
    pub time_s: f64,
    /// GPS time.
    pub time: GnssTime,
    /// True antenna position in ENU meters.
    pub antenna_enu_m: Vec3,
    /// Observations in satellite-id order.
    pub observations: Vec<SatelliteObservation>,
}

impl GnssEpoch {
    /// Number of satellites whose primary signal has a given path class.
    pub fn count(&self, path: SignalPath) -> usize {
        self.observations
            .iter()
            .filter(|o| o.path() == path)
            .count()
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
    /// Simulation time the measurements were taken, in seconds.
    pub measurement_time_s: f64,
    /// Simulation time the fix becomes available, in seconds.
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
    /// Used satellites that were actually NLOS or diffracted (ground truth).
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

/// Per-signal tracking state.
#[derive(Clone, Copy, Debug, PartialEq)]
struct TrackState {
    /// Range plus clock of the tracked path at the last epoch (for Doppler).
    last_range_m: f64,
    last_time_s: f64,
    /// Integer ambiguity of the current phase arc, when phase-locked.
    ambiguity_cycles: Option<f64>,
    /// Identity of the tracked path (kind and surfaces).
    path_signature: u64,
    /// Excess path of the tracked path.
    excess_m: f64,
}

/// A stable identity for a propagation path: its kind and the surfaces it
/// touches.
fn path_signature(path: &crate::raytrace::PropagationPath) -> u64 {
    let mut hash = match path.kind {
        PathKind::Direct => 1_u64,
        PathKind::Reflected { bounces } => 2 + u64::from(bounces),
        PathKind::Diffracted => 7,
    };
    for source in &path.sources {
        let code = match source {
            FacetSource::Box(index) => *index as u64 + 1,
            FacetSource::Ground => 0,
        };
        hash = hash.wrapping_mul(0x100_0000_01B3) ^ code;
    }
    hash
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
    tracks: BTreeMap<(SatelliteId, SignalKind), TrackState>,
    arcs: BTreeMap<(SatelliteId, SignalKind), u64>,
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
            tracks: BTreeMap::new(),
            arcs: BTreeMap::new(),
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

    /// Whether [`Self::step`] measured a new epoch at `time_s`.
    pub fn measured_at(&self, time_s: f64) -> bool {
        self.last_epoch
            .as_ref()
            .is_some_and(|e| (e.time_s - time_s).abs() < 1.0e-9)
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
        let time = self.config.epoch.add_seconds(time_s);
        let receiver_ecef = frame.enu_to_ecef(antenna_enu_m);
        let receiver_geodetic = Geodetic::from_ecef(receiver_ecef);
        let mut observations = Vec::with_capacity(constellation.len());
        for satellite in constellation.satellites() {
            // A fixed number of draws per satellite and signal.
            let mut draws = [[0.0; 4]; 4];
            for row in &mut draws {
                for value in row.iter_mut() {
                    *value = gaussian(&mut self.rng);
                }
            }

            let satellite_ecef = transmit_position(satellite.orbit, receiver_ecef, time_s);
            let line_ecef = satellite_ecef - receiver_ecef;
            let range = line_ecef.length();
            let direction = frame.ecef_vector_to_enu(line_ecef / range);
            let (azimuth_rad, elevation_rad) = azimuth_elevation(direction);
            let mut observation = SatelliteObservation {
                id: satellite.id,
                azimuth_rad,
                elevation_rad,
                satellite_ecef_m: satellite_ecef,
                signals: Vec::new(),
            };
            let signals: Vec<SignalKind> = self.config.tracked_signals(satellite.id).collect();
            if elevation_rad < self.config.elevation_mask_rad.max(0.0) {
                for signal in signals {
                    self.tracks.remove(&(satellite.id, signal));
                    observation
                        .signals
                        .push(SignalObservation::untracked(signal, SignalPath::BelowMask));
                }
                observations.push(observation);
                continue;
            }

            let propagation = trace_satellite(scene, antenna_enu_m, direction, &self.config.trace);
            let atmosphere = self.config.atmosphere;
            let iono_l1 = atmosphere.klobuchar.delay_l1_m(
                time,
                receiver_geodetic.latitude_rad,
                receiver_geodetic.longitude_rad,
                azimuth_rad,
                elevation_rad,
            ) * atmosphere.iono_truth_scale;
            let tropo = saastamoinen_delay_m(
                receiver_geodetic.latitude_rad,
                receiver_geodetic.height_m,
                elevation_rad,
                atmosphere.humidity,
            ) * atmosphere.tropo_truth_scale;
            let satellite_error = satellite_error_m(
                self.config.satellite_error_seed,
                satellite.id,
                self.config.satellite_error_std_m,
            );

            for signal in signals {
                let draw = draws[signal as usize];
                let common = range + satellite_error + tropo;
                let signal_obs = self.track_signal(
                    satellite.id,
                    signal,
                    &propagation,
                    common,
                    iono_l1 * signal.iono_scale(),
                    satellite_error + tropo,
                    time_s,
                    draw,
                );
                observation.signals.push(signal_obs);
            }
            observations.push(observation);
        }
        GnssEpoch {
            time_s,
            time,
            antenna_enu_m,
            observations,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn track_signal(
        &mut self,
        id: SatelliteId,
        signal: SignalKind,
        propagation: &Propagation,
        common_m: f64,
        iono_m: f64,
        common_error_m: f64,
        time_s: f64,
        draw: [f64; 4],
    ) -> SignalObservation {
        let tracking = self.config.signal;
        let lambda = signal.wavelength_m();
        let key = (id, signal);
        // Power of every path at this frequency.
        let powers: Vec<f64> = propagation
            .paths
            .iter()
            .map(|p| {
                let (_, arrival_el) = azimuth_elevation(p.arrival_direction);
                tracking.antenna_cn0_dbhz(signal, arrival_el) - p.loss_db(lambda)
            })
            .collect();
        let tracked = if propagation.line_of_sight {
            Some(0)
        } else {
            (0..powers.len()).max_by(|&a, &b| {
                powers[a].total_cmp(&powers[b]).then(
                    propagation.paths[b]
                        .excess_delay_m
                        .total_cmp(&propagation.paths[a].excess_delay_m),
                )
            })
        };
        let Some(tracked) = tracked else {
            self.tracks.remove(&key);
            return SignalObservation::untracked(signal, SignalPath::Blocked);
        };
        let path = &propagation.paths[tracked];
        let excess = path.excess_delay_m;
        // Later replicas sharing the correlation peak.
        let mut wall_replica = false;
        let replicas: Vec<(f64, f64)> = propagation
            .paths
            .iter()
            .zip(&powers)
            .enumerate()
            .filter(|(i, (p, _))| *i != tracked && p.excess_delay_m > excess)
            .map(|(_, (p, power))| {
                let delay = p.excess_delay_m - excess;
                if delay < signal.chip_length_m()
                    && p.sources.iter().any(|s| *s != FacetSource::Ground)
                {
                    wall_replica = true;
                }
                (10.0_f64.powf((power - powers[tracked]) / 20.0), delay)
            })
            .collect();
        let cn0 = powers[tracked] + tracking.multipath_fading_db(signal, &replicas);
        let class = match path.kind {
            PathKind::Direct if wall_replica => SignalPath::LineOfSightMultipath,
            PathKind::Direct => SignalPath::LineOfSight,
            PathKind::Reflected { .. } => SignalPath::NonLineOfSight,
            PathKind::Diffracted => SignalPath::Diffracted,
        };
        if cn0 < tracking.tracking_threshold_dbhz {
            self.tracks.remove(&key);
            let mut weak = SignalObservation::untracked(signal, SignalPath::TooWeak);
            weak.cn0_dbhz = cn0;
            return weak;
        }

        let code_multipath: f64 = replicas
            .iter()
            .map(|&(amplitude, delay)| tracking.multipath_code_error_m(signal, amplitude, delay))
            .sum();
        let phase_multipath = tracking.multipath_phase_error_m(signal, &replicas);
        let code_error = common_error_m
            + iono_m
            + excess
            + code_multipath
            + draw[0] * tracking.dll_noise_std_m(signal, cn0);
        let phase_noise_m = draw[1] * tracking.pll_noise_std_cycles(cn0) * lambda;
        let phase_error = common_error_m - iono_m + excess + phase_multipath + phase_noise_m;
        let geometric = common_m - common_error_m + self.clock_bias_m;
        let range_for_doppler = geometric + excess;

        // When the tracked path changes (an NLOS reflection giving way to the
        // direct signal, or one wall to another) the carrier phase jumps by
        // the change in path length: the loop slips.
        let signature = path_signature(path);
        let previous = self.tracks.get(&key).copied().filter(|state| {
            state.path_signature == signature || (state.excess_m - excess).abs() < 0.25 * lambda
        });
        let doppler_hz = previous.map(|state| {
            let dt = (time_s - state.last_time_s).max(1.0e-6);
            -(range_for_doppler - state.last_range_m) / dt / lambda
                + draw[3] * tracking.doppler_noise_hz
        });

        // Carrier phase: locked above the threshold; a slip or a fresh lock
        // starts a new arc with a new integer ambiguity.
        let mut loss_of_lock = false;
        let ambiguity = if cn0 >= tracking.phase_lock_threshold_dbhz {
            let margin = ((tracking.phase_lock_threshold_dbhz + 6.0 - cn0) / 6.0).clamp(0.0, 1.0);
            let slip = 0.5 * (1.0 + erf_approx(draw[2] / std::f64::consts::SQRT_2))
                < tracking.slip_probability_at_threshold * margin;
            match previous.and_then(|s| s.ambiguity_cycles) {
                Some(n) if !slip => Some(n),
                _ => {
                    loss_of_lock = true;
                    let arc = self.arcs.entry(key).or_insert(0);
                    *arc += 1;
                    Some(ambiguity_cycles(self.config.seed, id, signal, *arc))
                }
            }
        } else {
            None
        };
        self.tracks.insert(
            key,
            TrackState {
                last_range_m: range_for_doppler,
                last_time_s: time_s,
                ambiguity_cycles: ambiguity,
                path_signature: signature,
                excess_m: excess,
            },
        );

        let true_range = common_m - common_error_m;
        SignalObservation {
            signal,
            path: class,
            cn0_dbhz: cn0,
            pseudorange_m: Some(true_range + self.clock_bias_m + code_error),
            carrier_phase_cycles: ambiguity
                .map(|n| (true_range + self.clock_bias_m + phase_error) / lambda + n),
            doppler_hz,
            loss_of_lock,
            code_error_m: code_error,
            phase_error_m: phase_error,
            excess_path_m: excess,
            interaction_point_m: path.points_m.first().copied(),
        }
    }

    /// Solves a single-point position from an epoch's primary pseudoranges,
    /// applying the broadcast Klobuchar and Saastamoinen corrections.
    pub fn solve(&mut self, epoch: &GnssEpoch, frame: &EnuFrame) -> GnssFix {
        let initial = self.last_position_ecef_m.unwrap_or(frame.origin_ecef_m);
        let approx = Geodetic::from_ecef(initial);
        let atmosphere = self.config.atmosphere;
        let mut inputs = Vec::new();
        let mut usable_paths = BTreeMap::new();
        let mut tracked = 0;
        for o in &epoch.observations {
            let Some(primary) = o.primary() else {
                continue;
            };
            if primary.path.is_tracked() {
                tracked += 1;
            }
            let Some(pseudorange) = primary.pseudorange_m else {
                continue;
            };
            if primary.cn0_dbhz < self.config.fix_cn0_mask_dbhz {
                continue;
            }
            let correction = atmosphere.klobuchar.delay_l1_m(
                epoch.time,
                approx.latitude_rad,
                approx.longitude_rad,
                o.azimuth_rad,
                o.elevation_rad,
            ) + saastamoinen_delay_m(
                approx.latitude_rad,
                approx.height_m,
                o.elevation_rad,
                atmosphere.humidity,
            );
            usable_paths.insert(o.id, primary.path);
            inputs.push(PseudorangeInput {
                id: o.id,
                satellite_ecef_m: o.satellite_ecef_m,
                pseudorange_m: pseudorange - correction,
                sigma_m: self.measurement_sigma_m(o.elevation_rad, primary.cn0_dbhz),
            });
        }
        let available_time_s = epoch.time_s + self.config.latency_s;
        let Ok(solution) = solve_position(&inputs, initial, frame, &self.config.solver) else {
            return GnssFix {
                measurement_time_s: epoch.time_s,
                available_time_s,
                status: FixStatus::NoFix,
                position_enu_m: Vec3::ZERO,
                geodetic: frame.datum,
                horizontal_std_m: f64::INFINITY,
                vertical_std_m: f64::INFINITY,
                dop: Dop::default(),
                satellites_tracked: tracked,
                satellites_used: 0,
                excluded: Vec::new(),
                consistent: false,
                nlos_used: 0,
            };
        };
        self.last_position_ecef_m = Some(solution.position_ecef_m);
        let nlos_used = solution
            .used
            .iter()
            .filter(|id| {
                usable_paths
                    .get(*id)
                    .is_some_and(|p: &SignalPath| p.is_indirect())
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
        let thermal = self
            .config
            .signal
            .dll_noise_std_m(SignalKind::GpsL1Ca, cn0_dbhz);
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
fn transmit_position(orbit: KeplerOrbit, receiver_ecef_m: Vec3, time_s: f64) -> Vec3 {
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

fn id_key(id: SatelliteId) -> u64 {
    ((id.system as u64) << 8) | u64::from(id.prn)
}

/// Constant orbit-and-clock error of one satellite, seeded by its id.
fn satellite_error_m(seed: u64, id: SatelliteId, std_m: f64) -> f64 {
    let mut rng = DeterministicRng::new(seed ^ id_key(id).wrapping_mul(0x9E37_79B9_7F4A_7C15));
    std_m * gaussian(&mut rng)
}

/// Integer ambiguity of one lock arc, seeded by satellite, signal, and arc.
fn ambiguity_cycles(seed: u64, id: SatelliteId, signal: SignalKind, arc: u64) -> f64 {
    let key = (id_key(id) << 16) ^ ((signal as u64) << 12) ^ arc;
    let mut rng = DeterministicRng::new(seed ^ key.wrapping_mul(0xD1B5_4A32_D192_ED03) ^ 0xA5A5);
    rng.uniform_usize(2_000_001) as f64 - 1_000_000.0
}

fn gaussian(rng: &mut DeterministicRng) -> f64 {
    let u1 = rng.uniform_f64(1.0e-12, 1.0);
    let u2 = rng.uniform_f64(0.0, 1.0);
    (-2.0 * u1.ln()).sqrt() * (TAU * u2).cos()
}

/// Error function (Abramowitz-Stegun 7.1.26, |error| < 1.5e-7).
fn erf_approx(x: f64) -> f64 {
    let t = 1.0 / (1.0 + 0.327_591_1 * x.abs());
    let poly = t
        * (0.254_829_592
            + t * (-0.284_496_736
                + t * (1.421_413_741 + t * (-1.453_152_027 + t * 1.061_405_429))));
    let value = 1.0 - poly * (-x * x).exp();
    if x >= 0.0 {
        value
    } else {
        -value
    }
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
    fn carrier_phase_is_continuous_within_an_arc_and_matches_the_code() {
        let scene = GnssScene::open_sky();
        let constellation = Constellation::gps_galileo_nominal();
        let mut receiver = GnssReceiver::new(ReceiverConfig::default()).unwrap();
        let antenna = Vec3::new(0.0, 0.0, 1.0);
        let first = receiver.observe(&scene, &constellation, &frame(), antenna, 0.0);
        let second = receiver.observe(&scene, &constellation, &frame(), antenna, 0.2);
        let mut checked = 0;
        for (a, b) in first.observations.iter().zip(&second.observations) {
            for (sa, sb) in a.signals.iter().zip(&b.signals) {
                let (Some(pa), Some(pb)) = (sa.carrier_phase_cycles, sb.carrier_phase_cycles)
                else {
                    continue;
                };
                assert!(sa.loss_of_lock, "first epoch starts an arc");
                if sb.loss_of_lock {
                    continue;
                }
                checked += 1;
                let lambda = sa.signal.wavelength_m();
                // Phase and code move together (code-minus-carrier changes by
                // noise, multipath and twice the tiny iono change only).
                let delta_phase_m = (pb - pa) * lambda;
                let delta_code_m = sb.pseudorange_m.unwrap() - sa.pseudorange_m.unwrap();
                assert!((delta_phase_m - delta_code_m).abs() < 2.0, "{}", a.id);
                // Doppler agrees with the phase rate.
                let doppler = sb.doppler_hz.unwrap();
                assert!(
                    (doppler + (pb - pa) / 0.2).abs() < 1.0,
                    "{} doppler {doppler}",
                    a.id
                );
                // The phase equals the code to within an integer number of
                // cycles plus the iono divergence and noise.
                let cmc = sb.pseudorange_m.unwrap() - pb * lambda;
                assert!(cmc.abs() > 1.0, "ambiguity present");
            }
        }
        assert!(checked > 10, "{checked}");
    }

    #[test]
    fn switching_from_nlos_to_direct_slips_the_carrier() {
        // A street whose east wall ends at y = 50 m: walking north past its
        // end, eastern satellites tracked by their reflection off the west
        // wall become line-of-sight.
        let mut scene = GnssScene::default();
        scene.add_box(SceneBox::building(
            10.0,
            -50.0,
            12.0,
            50.0,
            40.0,
            SurfaceMaterial::CONCRETE,
        ));
        scene.add_box(SceneBox::building(
            -12.0,
            -200.0,
            -10.0,
            200.0,
            15.0,
            SurfaceMaterial::GLASS,
        ));
        let constellation = Constellation::gps_galileo_nominal();
        let mut switched = 0;
        for start in [0.0, 3_000.0, 6_000.0, 9_000.0] {
            let mut receiver = GnssReceiver::new(ReceiverConfig::default()).unwrap();
            let mut previous: Option<GnssEpoch> = None;
            for step in 0..80 {
                let antenna = Vec3::new(0.0, 30.0 + step as f64 * 0.5, 1.0);
                let epoch = receiver.observe(
                    &scene,
                    &constellation,
                    &frame(),
                    antenna,
                    start + step as f64 * 0.2,
                );
                if let Some(before) = &previous {
                    for (a, b) in before.observations.iter().zip(&epoch.observations) {
                        let (Some(pa), Some(pb)) = (a.primary(), b.primary()) else {
                            continue;
                        };
                        // A jump of under a quarter wavelength (diffraction
                        // right at the edge) keeps the loop locked.
                        if pa.path.is_indirect()
                            && pa.excess_path_m > 0.25 * pa.signal.wavelength_m()
                            && pa.carrier_phase_cycles.is_some()
                            && matches!(
                                pb.path,
                                SignalPath::LineOfSight | SignalPath::LineOfSightMultipath
                            )
                            && pb.carrier_phase_cycles.is_some()
                        {
                            assert!(pb.loss_of_lock, "{} switched path without a slip", a.id);
                            assert!(pb.doppler_hz.is_none());
                            switched += 1;
                        }
                    }
                }
                previous = Some(epoch);
            }
        }
        assert!(switched > 0, "no NLOS-to-LOS switch in the walks");
    }

    #[test]
    fn dual_frequency_iono_ratio() {
        let scene = GnssScene::open_sky();
        let constellation = Constellation::gps_galileo_nominal();
        let config = ReceiverConfig {
            satellite_error_std_m: 0.0,
            ..ReceiverConfig::default()
        };
        let mut receiver = GnssReceiver::new(config).unwrap();
        let epoch = receiver.observe(
            &scene,
            &constellation,
            &frame(),
            Vec3::new(0.0, 0.0, 2.0),
            0.0,
        );
        for o in epoch
            .observations
            .iter()
            .filter(|o| o.path() == SignalPath::LineOfSight)
        {
            assert_eq!(o.signals.len(), 2);
            let (l1, l2) = (&o.signals[0], &o.signals[1]);
            // Geometry-free code combination is dominated by the iono.
            let gf = l2.pseudorange_m.unwrap() - l1.pseudorange_m.unwrap();
            assert!(gf > 0.0, "{} {gf}", o.id);
        }
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
        for step in 0..20 {
            let epoch = receiver.observe(
                &scene,
                &constellation,
                &frame(),
                Vec3::new(0.0, 0.0, 1.0),
                step as f64 * 600.0,
            );
            nlos += epoch.count(SignalPath::NonLineOfSight);
            for o in &epoch.observations {
                if let Some(s) = o.primary().filter(|s| s.path.is_indirect()) {
                    // Indirect receptions carry their positive excess path.
                    assert!(s.excess_path_m > 0.0);
                }
            }
        }
        assert!(nlos > 0, "no NLOS receptions");
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
                    (receiver.solve(&epoch, &frame()).position_enu_m, epoch)
                })
                .collect::<Vec<_>>()
        };
        assert_eq!(run(), run());
    }
}
