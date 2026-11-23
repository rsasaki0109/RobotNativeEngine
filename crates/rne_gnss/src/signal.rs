//! Signals and signal-level error models: carrier frequencies, antenna gain,
//! code and carrier tracking noise, and code/carrier multipath.

use crate::atmosphere::GPS_L1_HZ;
use crate::orbit::GnssSystem;
use crate::orbit::SPEED_OF_LIGHT_M_S;
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;

/// A simulated ranging signal.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum SignalKind {
    /// GPS L1 C/A.
    GpsL1Ca,
    /// GPS L2C.
    GpsL2C,
    /// Galileo E1 (B/C).
    GalE1,
    /// Galileo E5a.
    GalE5a,
    /// QZSS L1 C/A.
    QzsL1Ca,
    /// QZSS L2C.
    QzsL2C,
}

impl SignalKind {
    /// Every simulated signal.
    pub const ALL: [Self; 6] = [
        Self::GpsL1Ca,
        Self::GpsL2C,
        Self::GalE1,
        Self::GalE5a,
        Self::QzsL1Ca,
        Self::QzsL2C,
    ];

    /// The system transmitting the signal.
    pub fn system(self) -> GnssSystem {
        match self {
            Self::GpsL1Ca | Self::GpsL2C => GnssSystem::Gps,
            Self::GalE1 | Self::GalE5a => GnssSystem::Galileo,
            Self::QzsL1Ca | Self::QzsL2C => GnssSystem::Qzss,
        }
    }

    /// Carrier frequency in Hz.
    pub fn frequency_hz(self) -> f64 {
        match self {
            Self::GpsL1Ca | Self::GalE1 | Self::QzsL1Ca => GPS_L1_HZ,
            Self::GpsL2C | Self::QzsL2C => 1_227.60e6,
            Self::GalE5a => 1_176.45e6,
        }
    }

    /// Carrier wavelength in meters.
    pub fn wavelength_m(self) -> f64 {
        SPEED_OF_LIGHT_M_S / self.frequency_hz()
    }

    /// Length of one code chip in meters.
    pub fn chip_length_m(self) -> f64 {
        match self {
            Self::GpsL1Ca | Self::GpsL2C | Self::GalE1 | Self::QzsL1Ca | Self::QzsL2C => {
                SPEED_OF_LIGHT_M_S / 1.023e6
            }
            Self::GalE5a => SPEED_OF_LIGHT_M_S / 10.23e6,
        }
    }

    /// Received power relative to L1 C/A in dB.
    pub fn power_offset_db(self) -> f64 {
        match self {
            Self::GpsL1Ca | Self::GalE1 | Self::QzsL1Ca => 0.0,
            Self::GpsL2C | Self::QzsL2C => -1.5,
            Self::GalE5a => 1.0,
        }
    }

    /// Whether this is the system's first (L1-band) signal.
    pub fn is_primary(self) -> bool {
        matches!(self, Self::GpsL1Ca | Self::GalE1 | Self::QzsL1Ca)
    }

    /// Ionospheric delay at this frequency relative to L1.
    pub fn iono_scale(self) -> f64 {
        (GPS_L1_HZ / self.frequency_hz()).powi(2)
    }

    /// RINEX 3 observation code (frequency band and attribute), e.g. `1C`.
    pub fn rinex_code(self) -> &'static str {
        match self {
            Self::GpsL1Ca | Self::GalE1 | Self::QzsL1Ca => "1C",
            Self::GpsL2C | Self::QzsL2C => "2L",
            Self::GalE5a => "5Q",
        }
    }
}

/// Antenna pattern and tracking-loop parameters shared by all signals.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SignalConfig {
    /// Early-minus-late correlator spacing in chips.
    pub correlator_spacing_chips: f64,
    /// Delay-lock-loop noise bandwidth in Hz.
    pub dll_bandwidth_hz: f64,
    /// Phase-lock-loop noise bandwidth in Hz.
    pub pll_bandwidth_hz: f64,
    /// Coherent integration time in seconds.
    pub coherent_integration_s: f64,
    /// C/N0 of a direct L1 signal at zenith, in dB-Hz.
    pub cn0_zenith_dbhz: f64,
    /// C/N0 of a direct L1 signal at the horizon, in dB-Hz.
    pub cn0_horizon_dbhz: f64,
    /// Extra antenna loss for signals arriving from below the horizon, in dB.
    pub below_horizon_loss_db: f64,
    /// Minimum C/N0 for code tracking, in dB-Hz.
    pub tracking_threshold_dbhz: f64,
    /// Minimum C/N0 for carrier-phase lock, in dB-Hz.
    pub phase_lock_threshold_dbhz: f64,
    /// Cycle-slip probability per epoch at the phase-lock threshold; it falls
    /// linearly to zero 6 dB above it.
    pub slip_probability_at_threshold: f64,
    /// One-sigma Doppler noise in Hz.
    pub doppler_noise_hz: f64,
}

impl Default for SignalConfig {
    fn default() -> Self {
        Self::gps_l1_ca()
    }
}

impl SignalConfig {
    /// A narrow-correlator geodetic-grade receiver with a patch antenna.
    pub fn gps_l1_ca() -> Self {
        Self {
            correlator_spacing_chips: 0.1,
            dll_bandwidth_hz: 1.0,
            pll_bandwidth_hz: 15.0,
            coherent_integration_s: 0.02,
            cn0_zenith_dbhz: 48.0,
            cn0_horizon_dbhz: 36.0,
            below_horizon_loss_db: 10.0,
            tracking_threshold_dbhz: 28.0,
            phase_lock_threshold_dbhz: 31.0,
            slip_probability_at_threshold: 0.05,
            doppler_noise_hz: 0.05,
        }
    }

    /// Whether every parameter is finite and in range.
    pub fn is_valid(&self) -> bool {
        self.correlator_spacing_chips > 0.0
            && self.correlator_spacing_chips <= 1.0
            && self.dll_bandwidth_hz > 0.0
            && self.pll_bandwidth_hz > 0.0
            && self.coherent_integration_s > 0.0
            && self.cn0_zenith_dbhz.is_finite()
            && self.cn0_horizon_dbhz.is_finite()
            && self.below_horizon_loss_db.is_finite()
            && self.below_horizon_loss_db >= 0.0
            && self.tracking_threshold_dbhz.is_finite()
            && self.phase_lock_threshold_dbhz.is_finite()
            && (0.0..=1.0).contains(&self.slip_probability_at_threshold)
            && self.doppler_noise_hz >= 0.0
    }

    /// C/N0 of an unattenuated signal arriving at `elevation_rad` (antenna
    /// pattern only).
    pub fn antenna_cn0_dbhz(&self, signal: SignalKind, elevation_rad: f64) -> f64 {
        let pattern = if elevation_rad >= 0.0 {
            self.cn0_horizon_dbhz
                + (self.cn0_zenith_dbhz - self.cn0_horizon_dbhz) * elevation_rad.sin()
        } else {
            self.cn0_horizon_dbhz - self.below_horizon_loss_db
        };
        pattern + signal.power_offset_db()
    }

    /// One-sigma code tracking noise of an early-minus-late power DLL, meters.
    pub fn dll_noise_std_m(&self, signal: SignalKind, cn0_dbhz: f64) -> f64 {
        let cn0 = 10.0_f64.powf(cn0_dbhz / 10.0);
        let d = self.correlator_spacing_chips;
        let variance_chips2 = self.dll_bandwidth_hz * d / (2.0 * cn0)
            * (1.0 + 2.0 / ((2.0 - d) * self.coherent_integration_s * cn0));
        signal.chip_length_m() * variance_chips2.sqrt()
    }

    /// One-sigma carrier tracking noise of a Costas PLL, in cycles.
    pub fn pll_noise_std_cycles(&self, cn0_dbhz: f64) -> f64 {
        let cn0 = 10.0_f64.powf(cn0_dbhz / 10.0);
        let variance_rad2 =
            self.pll_bandwidth_hz / cn0 * (1.0 + 1.0 / (2.0 * self.coherent_integration_s * cn0));
        variance_rad2.sqrt() / TAU
    }

    /// Code-phase error in meters from one replica with `relative_amplitude`
    /// (to the tracked signal) delayed by `excess_delay_m`: the narrow
    /// correlator multipath envelope at the geometric carrier phase.
    pub fn multipath_code_error_m(
        &self,
        signal: SignalKind,
        relative_amplitude: f64,
        excess_delay_m: f64,
    ) -> f64 {
        let chip = signal.chip_length_m();
        let tau = excess_delay_m / chip;
        let half = 0.5 * self.correlator_spacing_chips;
        if !(0.0..1.0 + half).contains(&tau) {
            return 0.0;
        }
        let phase = TAU * excess_delay_m / signal.wavelength_m();
        let in_phase = relative_amplitude * phase.cos();
        let denominator = (1.0 + in_phase).max(0.05);
        let lever_chips = if tau < half {
            tau
        } else if tau < 1.0 - half {
            half
        } else {
            0.5 * (1.0 + half - tau)
        };
        chip * in_phase * lever_chips / denominator
    }

    /// Carrier-phase error in meters from replicas `(relative amplitude,
    /// excess delay)` within one chip of the tracked signal.
    pub fn multipath_phase_error_m(&self, signal: SignalKind, replicas: &[(f64, f64)]) -> f64 {
        let (re, im) = composite(signal, replicas);
        im.atan2(re) / TAU * signal.wavelength_m()
    }

    /// Power change in dB when replicas interfere with the tracked signal.
    pub fn multipath_fading_db(&self, signal: SignalKind, replicas: &[(f64, f64)]) -> f64 {
        let (re, im) = composite(signal, replicas);
        10.0 * (re * re + im * im).max(1.0e-3).log10()
    }
}

/// Phasor sum of the tracked signal and the replicas that share its
/// correlation peak.
fn composite(signal: SignalKind, replicas: &[(f64, f64)]) -> (f64, f64) {
    let (mut re, mut im) = (1.0, 0.0);
    for &(amplitude, delay_m) in replicas {
        if !(0.0..signal.chip_length_m()).contains(&delay_m) {
            continue;
        }
        let phase = TAU * delay_m / signal.wavelength_m();
        re += amplitude * phase.cos();
        im += amplitude * phase.sin();
    }
    (re, im)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn signal_constants() {
        assert_relative_eq!(
            SignalKind::GpsL1Ca.wavelength_m(),
            0.190_294,
            epsilon = 1e-5
        );
        assert_relative_eq!(SignalKind::GpsL2C.wavelength_m(), 0.244_210, epsilon = 1e-5);
        assert_relative_eq!(SignalKind::GalE5a.chip_length_m(), 29.305, epsilon = 1e-3);
        assert_relative_eq!(SignalKind::GpsL2C.iono_scale(), 1.6469, epsilon = 1e-3);
    }

    #[test]
    fn tracking_noise_falls_with_cn0() {
        let config = SignalConfig::default();
        let strong = config.dll_noise_std_m(SignalKind::GpsL1Ca, 48.0);
        let weak = config.dll_noise_std_m(SignalKind::GpsL1Ca, 30.0);
        assert!(strong < 0.3, "{strong}");
        assert!(weak > 4.0 * strong);
        // E5a's ten times faster chipping gives ten times lower code noise.
        assert_relative_eq!(
            config.dll_noise_std_m(SignalKind::GalE5a, 48.0) * 10.0,
            config.dll_noise_std_m(SignalKind::GalE1, 48.0),
            epsilon = 1e-9
        );
        // Millimetre-level carrier noise at 48 dB-Hz.
        let phase_m = config.pll_noise_std_cycles(48.0) * SignalKind::GpsL1Ca.wavelength_m();
        assert!(phase_m < 0.002, "{phase_m}");
    }

    #[test]
    fn multipath_error_is_bounded_and_vanishes_past_one_chip() {
        let config = SignalConfig::default();
        let signal = SignalKind::GpsL1Ca;
        let half = 0.5 * config.correlator_spacing_chips * signal.chip_length_m();
        for step in 0..400 {
            let error = config.multipath_code_error_m(signal, 0.5, step as f64);
            assert!(error.abs() <= half + 1e-9, "delay {step}: {error}");
        }
        assert_eq!(
            config.multipath_code_error_m(signal, 0.5, 1.1 * signal.chip_length_m()),
            0.0
        );
        // In phase: the linear part of the envelope.
        let delay = 10.0 * signal.wavelength_m();
        assert_relative_eq!(
            config.multipath_code_error_m(signal, 0.3, delay),
            0.3 * delay / 1.3,
            epsilon = 1e-9
        );
    }

    #[test]
    fn carrier_multipath_is_at_most_a_quarter_wavelength() {
        let config = SignalConfig::default();
        let signal = SignalKind::GpsL1Ca;
        let lambda = signal.wavelength_m();
        let mut worst: f64 = 0.0;
        for step in 0..200 {
            let delay = step as f64 * lambda / 50.0;
            worst = worst.max(
                config
                    .multipath_phase_error_m(signal, &[(0.9, delay)])
                    .abs(),
            );
        }
        assert!(
            worst < 0.25 * lambda + 1e-9 && worst > 0.15 * lambda,
            "{worst}"
        );
        assert!(config.multipath_fading_db(signal, &[(0.5, 0.5 * lambda)]) < -5.0);
    }
}
