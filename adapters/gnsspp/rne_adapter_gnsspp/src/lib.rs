//! libgnss++ adapter: SPP, RTK, and moving-baseline RTK on simulated
//! `rne_gnss` observations.
//!
//! [libgnss++](https://github.com/rsasaki0109/gnssplusplus-library) is a C++20
//! GNSS toolkit (SPP/RTK/PPP, RINEX/RTCM). This adapter exposes its online
//! solvers to Robot Native Engine through a small C ABI shim
//! (`cpp/rne_gnsspp.cpp`):
//!
//! * [`Navigation`] — broadcast ephemerides and Klobuchar coefficients from an
//!   [`rne_gnss::NavigationMessage`].
//! * [`Spp`] — single-point positioning on one antenna's epoch.
//! * [`Rtk`] — carrier-phase RTK of a rover against a base station
//!   ([`RtkMode::Kinematic`]), or of one robot antenna against another
//!   ([`RtkMode::MovingBase`], whose baseline gives the robot's heading).
//!
//! The shim is compiled only when a libgnss++ build is found (see
//! `build.rs` and `scripts/build_gnsspp.sh`); otherwise every constructor
//! returns [`GnssppError::Unavailable`] and [`is_available`] is false. Core
//! crates never depend on this adapter, and it does not change `rne_gnss`
//! types to fit libgnss++.

#![deny(missing_docs)]

use rne_gnss::{GnssEpoch, GnssSystem, NavigationMessage, SignalKind};
use rne_math::Vec3;

/// Errors raised by the adapter.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum GnssppError {
    /// The adapter was built without libgnss++.
    #[error("libgnss++ is not available in this build (see scripts/build_gnsspp.sh)")]
    Unavailable,
    /// libgnss++ failed to construct an object.
    #[error("libgnss++ failed to construct a solver")]
    Construction,
    /// libgnss++ threw while processing.
    #[error("libgnss++ failed to process an epoch")]
    Solver,
}

/// Whether this build links libgnss++.
pub fn is_available() -> bool {
    cfg!(gnsspp)
}

/// Solution status reported by libgnss++.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SolutionStatus {
    /// No solution.
    None,
    /// Single-point positioning.
    Single,
    /// Code differential.
    Dgps,
    /// RTK with float ambiguities.
    Float,
    /// RTK with fixed integer ambiguities.
    Fixed,
    /// Any other libgnss++ status code.
    Other(i32),
}

impl SolutionStatus {
    #[cfg_attr(not(gnsspp), allow(dead_code))]
    fn from_code(code: i32) -> Self {
        match code {
            0 => Self::None,
            1 => Self::Single,
            2 => Self::Dgps,
            3 => Self::Float,
            4 => Self::Fixed,
            other => Self::Other(other),
        }
    }

    /// Whether the solution carries a position.
    pub fn has_position(self) -> bool {
        !matches!(self, Self::None)
    }
}

/// A libgnss++ position solution.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Solution {
    /// Status.
    pub status: SolutionStatus,
    /// Antenna position in ECEF meters.
    pub position_ecef_m: Vec3,
    /// Position covariance in ECEF, m² (NaN when not reported).
    pub covariance_ecef_m2: [[f64; 3]; 3],
    /// Satellites used.
    pub satellites: i32,
    /// Fixed ambiguities (RTK).
    pub fixed_ambiguities: i32,
    /// Ambiguity ratio-test value (RTK).
    pub ratio: f64,
    /// Horizontal dilution of precision.
    pub hdop: f64,
    /// Baseline length in meters (RTK).
    pub baseline_length_m: f64,
}

/// libgnss++'s telemetry of the last RTK epoch.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RtkDiagnostics {
    /// Whether ambiguity resolution was attempted.
    pub ar_attempted: bool,
    /// libgnss++ `ARSkipReason` code (0 none, 3/4 too few pairs, 5 LAMBDA
    /// failed, 6 ratio failed).
    pub ar_skip_reason: i32,
    /// Code rows kept in the filter update.
    pub retained_code_rows: i32,
    /// Carrier-phase rows kept in the filter update.
    pub retained_phase_rows: i32,
    /// Double-difference ambiguity pairs used for AR.
    pub pair_count: i32,
    /// Largest ambiguity variance among the AR candidates, cycles².
    pub max_ambiguity_variance: f64,
}

/// RTK position mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RtkMode {
    /// Static rover.
    Static,
    /// Moving rover against a fixed base.
    Kinematic,
    /// Moving rover against a moving base whose position is given per epoch.
    MovingBase,
}

#[cfg_attr(not(gnsspp), allow(dead_code))]
fn signal_code(signal: SignalKind) -> u8 {
    match signal {
        SignalKind::GpsL1Ca => 0,
        SignalKind::GpsL2C => 1,
        SignalKind::GalE1 => 2,
        SignalKind::GalE5a => 3,
        SignalKind::QzsL1Ca => 4,
        SignalKind::QzsL2C => 5,
    }
}

#[cfg_attr(not(gnsspp), allow(dead_code))]
fn system_code(system: GnssSystem) -> u8 {
    match system {
        GnssSystem::Gps => 1,
        GnssSystem::Galileo => 4,
        GnssSystem::Qzss => 0x10,
    }
}

#[cfg(gnsspp)]
mod ffi {
    use std::ffi::c_void;

    #[repr(C)]
    pub(crate) struct Ephemeris {
        pub(crate) system: u8,
        pub(crate) prn: u8,
        pub(crate) iode: u16,
        pub(crate) toe_week: u32,
        pub(crate) toe_tow: f64,
        pub(crate) sqrt_a: f64,
        pub(crate) e: f64,
        pub(crate) i0: f64,
        pub(crate) omega0: f64,
        pub(crate) omega: f64,
        pub(crate) m0: f64,
        pub(crate) delta_n: f64,
        pub(crate) omega_dot: f64,
        pub(crate) idot: f64,
        pub(crate) af0: f64,
        pub(crate) af1: f64,
    }

    #[repr(C)]
    #[derive(Clone, Copy)]
    pub(crate) struct Observation {
        pub(crate) system: u8,
        pub(crate) prn: u8,
        pub(crate) signal: u8,
        pub(crate) has_pseudorange: u8,
        pub(crate) has_carrier_phase: u8,
        pub(crate) has_doppler: u8,
        pub(crate) loss_of_lock: u8,
        pub(crate) pseudorange_m: f64,
        pub(crate) carrier_phase_cycles: f64,
        pub(crate) doppler_hz: f64,
        pub(crate) cn0_dbhz: f64,
    }

    #[repr(C)]
    pub(crate) struct Epoch {
        pub(crate) week: u32,
        pub(crate) tow: f64,
        pub(crate) approx_ecef_m: [f64; 3],
        pub(crate) observations: *const Observation,
        pub(crate) count: usize,
    }

    #[repr(C)]
    #[derive(Default)]
    pub(crate) struct Solution {
        pub(crate) status: i32,
        pub(crate) ecef_m: [f64; 3],
        pub(crate) covariance_m2: [f64; 9],
        pub(crate) satellites: i32,
        pub(crate) fixed_ambiguities: i32,
        pub(crate) ratio: f64,
        pub(crate) hdop: f64,
        pub(crate) baseline_length_m: f64,
    }

    #[repr(C)]
    #[derive(Default)]
    pub(crate) struct RtkDiagnostics {
        pub(crate) ar_attempted: i32,
        pub(crate) ar_skip_reason: i32,
        pub(crate) retained_code_rows: i32,
        pub(crate) retained_phase_rows: i32,
        pub(crate) pair_count: i32,
        pub(crate) max_ambiguity_variance: f64,
    }

    extern "C" {
        pub(crate) fn rne_gnsspp_rtk_diagnostics_get(
            rtk: *const c_void,
            out: *mut RtkDiagnostics,
        ) -> i32;
        pub(crate) fn rne_gnsspp_nav_new() -> *mut c_void;
        pub(crate) fn rne_gnsspp_nav_free(nav: *mut c_void);
        pub(crate) fn rne_gnsspp_nav_add_ephemeris(nav: *mut c_void, eph: *const Ephemeris) -> i32;
        pub(crate) fn rne_gnsspp_nav_set_klobuchar(
            nav: *mut c_void,
            alpha: *const f64,
            beta: *const f64,
        ) -> i32;
        pub(crate) fn rne_gnsspp_spp_new(elevation_mask_deg: f64) -> *mut c_void;
        pub(crate) fn rne_gnsspp_spp_free(spp: *mut c_void);
        pub(crate) fn rne_gnsspp_spp_process(
            spp: *mut c_void,
            epoch: *const Epoch,
            nav: *const c_void,
            out: *mut Solution,
        ) -> i32;
        pub(crate) fn rne_gnsspp_rtk_new(
            position_mode: i32,
            elevation_mask_deg: f64,
            ratio: f64,
            preset: *const std::ffi::c_char,
            max_nonfix_for_reset: i32,
        ) -> *mut c_void;
        pub(crate) fn rne_gnsspp_rtk_free(rtk: *mut c_void);
        pub(crate) fn rne_gnsspp_rtk_set_base_position(rtk: *mut c_void, ecef_m: *const f64)
            -> i32;
        pub(crate) fn rne_gnsspp_rtk_process(
            rtk: *mut c_void,
            rover: *const Epoch,
            base: *const Epoch,
            nav: *const c_void,
            out: *mut Solution,
        ) -> i32;
    }
}

/// Owned observation buffer laid out for the C ABI.
#[cfg(gnsspp)]
struct EpochBuffer {
    observations: Vec<ffi::Observation>,
    week: u32,
    tow: f64,
    approx: [f64; 3],
}

#[cfg(gnsspp)]
impl EpochBuffer {
    fn new(epoch: &GnssEpoch, approx_ecef_m: Option<Vec3>) -> Self {
        let mut observations = Vec::new();
        for satellite in &epoch.observations {
            for signal in &satellite.signals {
                let Some(pseudorange_m) = signal.pseudorange_m else {
                    continue;
                };
                observations.push(ffi::Observation {
                    system: system_code(satellite.id.system),
                    prn: satellite.id.prn,
                    signal: signal_code(signal.signal),
                    has_pseudorange: 1,
                    has_carrier_phase: u8::from(signal.carrier_phase_cycles.is_some()),
                    has_doppler: u8::from(signal.doppler_hz.is_some()),
                    loss_of_lock: u8::from(signal.loss_of_lock),
                    pseudorange_m,
                    carrier_phase_cycles: signal.carrier_phase_cycles.unwrap_or(0.0),
                    doppler_hz: signal.doppler_hz.unwrap_or(0.0),
                    cn0_dbhz: signal.cn0_dbhz,
                });
            }
        }
        let approx = approx_ecef_m.map_or([0.0; 3], |p| [p.x, p.y, p.z]);
        Self {
            observations,
            week: epoch.time.week,
            tow: epoch.time.tow_s,
            approx,
        }
    }

    /// A view valid while `self` lives and is not mutated.
    fn view(&self) -> ffi::Epoch {
        ffi::Epoch {
            week: self.week,
            tow: self.tow,
            approx_ecef_m: self.approx,
            observations: self.observations.as_ptr(),
            count: self.observations.len(),
        }
    }
}

#[cfg(gnsspp)]
fn convert(solution: &ffi::Solution) -> Solution {
    let c = solution.covariance_m2;
    Solution {
        status: SolutionStatus::from_code(solution.status),
        position_ecef_m: Vec3::new(solution.ecef_m[0], solution.ecef_m[1], solution.ecef_m[2]),
        covariance_ecef_m2: [[c[0], c[1], c[2]], [c[3], c[4], c[5]], [c[6], c[7], c[8]]],
        satellites: solution.satellites,
        fixed_ambiguities: solution.fixed_ambiguities,
        ratio: solution.ratio,
        hdop: solution.hdop,
        baseline_length_m: solution.baseline_length_m,
    }
}

/// libgnss++ navigation data (ephemerides and ionosphere coefficients).
#[derive(Debug)]
pub struct Navigation {
    #[cfg(gnsspp)]
    handle: *mut std::ffi::c_void,
}

impl Navigation {
    /// Loads a navigation message.
    pub fn new(message: &NavigationMessage) -> Result<Self, GnssppError> {
        #[cfg(gnsspp)]
        {
            // SAFETY: the constructor has no preconditions; a null return is
            // checked before the handle is used.
            let handle = unsafe { ffi::rne_gnsspp_nav_new() };
            if handle.is_null() {
                return Err(GnssppError::Construction);
            }
            let navigation = Self { handle };
            for eph in &message.ephemerides {
                let record = ffi::Ephemeris {
                    system: system_code(eph.id.system),
                    prn: eph.id.prn,
                    iode: eph.iode,
                    toe_week: eph.toe.week,
                    toe_tow: eph.toe.tow_s,
                    sqrt_a: eph.sqrt_a,
                    e: eph.eccentricity,
                    i0: eph.i0_rad,
                    omega0: eph.omega0_rad,
                    omega: eph.omega_rad,
                    m0: eph.m0_rad,
                    delta_n: eph.delta_n_rad_s,
                    omega_dot: eph.omega_dot_rad_s,
                    idot: eph.idot_rad_s,
                    af0: eph.af0_s,
                    af1: eph.af1,
                };
                // SAFETY: `handle` is a live NavigationData from the shim and
                // `record` outlives the call, which copies it.
                if unsafe { ffi::rne_gnsspp_nav_add_ephemeris(navigation.handle, &record) } != 0 {
                    return Err(GnssppError::Construction);
                }
            }
            let k = message.klobuchar;
            // SAFETY: both pointers reference four f64 values that outlive the
            // call; the shim copies them.
            if unsafe {
                ffi::rne_gnsspp_nav_set_klobuchar(
                    navigation.handle,
                    k.alpha.as_ptr(),
                    k.beta.as_ptr(),
                )
            } != 0
            {
                return Err(GnssppError::Construction);
            }
            Ok(navigation)
        }
        #[cfg(not(gnsspp))]
        {
            let _ = message;
            Err(GnssppError::Unavailable)
        }
    }
}

impl Drop for Navigation {
    fn drop(&mut self) {
        #[cfg(gnsspp)]
        // SAFETY: the handle came from `rne_gnsspp_nav_new` and is freed once.
        unsafe {
            ffi::rne_gnsspp_nav_free(self.handle);
        }
    }
}

/// libgnss++ single-point positioning.
#[derive(Debug)]
pub struct Spp {
    #[cfg(gnsspp)]
    handle: *mut std::ffi::c_void,
}

impl Spp {
    /// Creates a solver with an elevation mask in degrees.
    pub fn new(elevation_mask_deg: f64) -> Result<Self, GnssppError> {
        #[cfg(gnsspp)]
        {
            // SAFETY: no preconditions; null is checked.
            let handle = unsafe { ffi::rne_gnsspp_spp_new(elevation_mask_deg) };
            if handle.is_null() {
                return Err(GnssppError::Construction);
            }
            Ok(Self { handle })
        }
        #[cfg(not(gnsspp))]
        {
            let _ = elevation_mask_deg;
            Err(GnssppError::Unavailable)
        }
    }

    /// Solves one epoch; `approx_ecef_m` seeds the iteration when known.
    pub fn process(
        &mut self,
        epoch: &GnssEpoch,
        approx_ecef_m: Option<Vec3>,
        navigation: &Navigation,
    ) -> Result<Solution, GnssppError> {
        #[cfg(gnsspp)]
        {
            let buffer = EpochBuffer::new(epoch, approx_ecef_m);
            let view = buffer.view();
            let mut out = ffi::Solution::default();
            // SAFETY: `view` points into `buffer`, which lives across the
            // call; the handles are live; `out` is a valid exclusive pointer.
            let code = unsafe {
                ffi::rne_gnsspp_spp_process(self.handle, &view, navigation.handle, &mut out)
            };
            if code != 0 {
                return Err(GnssppError::Solver);
            }
            Ok(convert(&out))
        }
        #[cfg(not(gnsspp))]
        {
            let _ = (epoch, approx_ecef_m, navigation);
            Err(GnssppError::Unavailable)
        }
    }
}

impl Drop for Spp {
    fn drop(&mut self) {
        #[cfg(gnsspp)]
        // SAFETY: the handle came from `rne_gnsspp_spp_new` and is freed once.
        unsafe {
            ffi::rne_gnsspp_spp_free(self.handle);
        }
    }
}

/// libgnss++ carrier-phase RTK.
#[derive(Debug)]
pub struct Rtk {
    #[cfg(gnsspp)]
    handle: *mut std::ffi::c_void,
}

/// RTK solver settings.
#[derive(Clone, Debug, PartialEq)]
pub struct RtkConfig {
    /// Position mode.
    pub mode: RtkMode,
    /// Elevation mask in degrees.
    pub elevation_mask_deg: f64,
    /// Ambiguity ratio-test threshold (ignored when a preset sets it).
    pub ratio_threshold: f64,
    /// libgnss++ RTK preset: "" (none), "survey", "low-cost", "odaiba", or
    /// "moving-base".
    pub preset: String,
    /// Reset the ambiguities after this many consecutive non-fixed epochs
    /// (0 disables).
    pub max_nonfix_for_reset: i32,
}

impl RtkConfig {
    /// Library defaults for a mode.
    pub fn new(mode: RtkMode) -> Self {
        Self {
            mode,
            elevation_mask_deg: 15.0,
            ratio_threshold: 3.0,
            preset: String::new(),
            max_nonfix_for_reset: 0,
        }
    }
}

impl Rtk {
    /// Creates a solver: position mode, elevation mask in degrees, and the
    /// ambiguity ratio-test threshold.
    pub fn new(
        mode: RtkMode,
        elevation_mask_deg: f64,
        ratio_threshold: f64,
    ) -> Result<Self, GnssppError> {
        Self::with_config(&RtkConfig {
            elevation_mask_deg,
            ratio_threshold,
            ..RtkConfig::new(mode)
        })
    }

    /// Creates a solver from full settings.
    pub fn with_config(config: &RtkConfig) -> Result<Self, GnssppError> {
        #[cfg(gnsspp)]
        {
            let code = match config.mode {
                RtkMode::Static => 0,
                RtkMode::Kinematic => 1,
                RtkMode::MovingBase => 2,
            };
            let preset = std::ffi::CString::new(config.preset.as_str())
                .map_err(|_| GnssppError::Construction)?;
            // SAFETY: `preset` is a NUL-terminated string alive across the
            // call (the shim copies what it needs); null is checked.
            let handle = unsafe {
                ffi::rne_gnsspp_rtk_new(
                    code,
                    config.elevation_mask_deg,
                    config.ratio_threshold,
                    preset.as_ptr(),
                    config.max_nonfix_for_reset,
                )
            };
            if handle.is_null() {
                return Err(GnssppError::Construction);
            }
            Ok(Self { handle })
        }
        #[cfg(not(gnsspp))]
        {
            let _ = config;
            Err(GnssppError::Unavailable)
        }
    }

    /// Sets the (fixed) base position in ECEF meters.
    pub fn set_base_position(&mut self, ecef_m: Vec3) -> Result<(), GnssppError> {
        #[cfg(gnsspp)]
        {
            let position = [ecef_m.x, ecef_m.y, ecef_m.z];
            // SAFETY: the handle is live and `position` holds three f64.
            if unsafe { ffi::rne_gnsspp_rtk_set_base_position(self.handle, position.as_ptr()) } != 0
            {
                return Err(GnssppError::Solver);
            }
            Ok(())
        }
        #[cfg(not(gnsspp))]
        {
            let _ = ecef_m;
            Err(GnssppError::Unavailable)
        }
    }

    /// Processes one synchronized rover / base epoch. In moving-base mode,
    /// `base_ecef_m` is the base antenna's position this epoch; otherwise it
    /// is an optional hint (the fixed base is set by
    /// [`Self::set_base_position`]). `rover_approx_ecef_m` seeds the rover.
    pub fn process(
        &mut self,
        rover: &GnssEpoch,
        rover_approx_ecef_m: Option<Vec3>,
        base: &GnssEpoch,
        base_ecef_m: Option<Vec3>,
        navigation: &Navigation,
    ) -> Result<Solution, GnssppError> {
        #[cfg(gnsspp)]
        {
            let rover_buffer = EpochBuffer::new(rover, rover_approx_ecef_m);
            let base_buffer = EpochBuffer::new(base, base_ecef_m);
            let (rover_view, base_view) = (rover_buffer.view(), base_buffer.view());
            let mut out = ffi::Solution::default();
            // SAFETY: both views point into buffers alive across the call;
            // the handles are live; `out` is a valid exclusive pointer.
            let code = unsafe {
                ffi::rne_gnsspp_rtk_process(
                    self.handle,
                    &rover_view,
                    &base_view,
                    navigation.handle,
                    &mut out,
                )
            };
            if code != 0 {
                return Err(GnssppError::Solver);
            }
            Ok(convert(&out))
        }
        #[cfg(not(gnsspp))]
        {
            let _ = (rover, rover_approx_ecef_m, base, base_ecef_m, navigation);
            Err(GnssppError::Unavailable)
        }
    }
}

impl Rtk {
    /// libgnss++'s telemetry of the last processed epoch.
    pub fn diagnostics(&self) -> Result<RtkDiagnostics, GnssppError> {
        #[cfg(gnsspp)]
        {
            let mut out = ffi::RtkDiagnostics::default();
            // SAFETY: the handle is live and `out` is a valid exclusive pointer.
            if unsafe { ffi::rne_gnsspp_rtk_diagnostics_get(self.handle, &mut out) } != 0 {
                return Err(GnssppError::Solver);
            }
            Ok(RtkDiagnostics {
                ar_attempted: out.ar_attempted != 0,
                ar_skip_reason: out.ar_skip_reason,
                retained_code_rows: out.retained_code_rows,
                retained_phase_rows: out.retained_phase_rows,
                pair_count: out.pair_count,
                max_ambiguity_variance: out.max_ambiguity_variance,
            })
        }
        #[cfg(not(gnsspp))]
        {
            Err(GnssppError::Unavailable)
        }
    }
}

impl Drop for Rtk {
    fn drop(&mut self) {
        #[cfg(gnsspp)]
        // SAFETY: the handle came from `rne_gnsspp_rtk_new` and is freed once.
        unsafe {
            ffi::rne_gnsspp_rtk_free(self.handle);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status_codes_map() {
        assert_eq!(SolutionStatus::from_code(4), SolutionStatus::Fixed);
        assert_eq!(SolutionStatus::from_code(3), SolutionStatus::Float);
        assert!(!SolutionStatus::from_code(0).has_position());
        assert_eq!(signal_code(SignalKind::GalE5a), 3);
        assert_eq!(system_code(GnssSystem::Galileo), 4);
        assert_eq!(system_code(GnssSystem::Qzss), 0x10);
        assert_eq!(signal_code(SignalKind::QzsL2C), 5);
    }

    #[test]
    fn availability_matches_constructors() {
        let available = is_available();
        assert_eq!(Spp::new(10.0).is_ok(), available);
        assert_eq!(Rtk::new(RtkMode::Kinematic, 10.0, 3.0).is_ok(), available);
        let preset = RtkConfig {
            preset: "low-cost".to_string(),
            ..RtkConfig::new(RtkMode::Kinematic)
        };
        assert_eq!(Rtk::with_config(&preset).is_ok(), available);
        let unknown = RtkConfig {
            preset: "no-such-preset".to_string(),
            ..RtkConfig::new(RtkMode::Kinematic)
        };
        assert!(Rtk::with_config(&unknown).is_err());
    }
}
