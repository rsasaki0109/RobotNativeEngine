//! Signal-level error models: antenna gain, code tracking noise, code
//! multipath, and residual atmospheric delays.

use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};

/// Code and carrier tracking parameters for one signal.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SignalConfig {
    /// Length of one code chip in meters (GPS L1 C/A: 293.05 m).
    pub chip_length_m: f64,
    /// Carrier wavelength in meters (L1: 0.1903 m).
    pub wavelength_m: f64,
    /// Early-minus-late correlator spacing in chips.
    pub correlator_spacing_chips: f64,
    /// Delay-lock-loop noise bandwidth in Hz.
    pub dll_bandwidth_hz: f64,
    /// Coherent integration time in seconds.
    pub coherent_integration_s: f64,
    /// Carrier-to-noise density of a direct signal at zenith, in dB-Hz.
    pub cn0_zenith_dbhz: f64,
    /// Carrier-to-noise density of a direct signal at the horizon, in dB-Hz.
    pub cn0_horizon_dbhz: f64,
    /// Extra antenna loss for signals arriving from below the horizon, in dB.
    pub below_horizon_loss_db: f64,
    /// Minimum C/N0 the receiver can track, in dB-Hz.
    pub tracking_threshold_dbhz: f64,
}

impl SignalConfig {
    /// A GPS L1 C/A narrow-correlator receiver with a patch antenna.
    pub fn gps_l1_ca() -> Self {
        Self {
            chip_length_m: 293.052,
            wavelength_m: 0.190_294,
            correlator_spacing_chips: 0.1,
            dll_bandwidth_hz: 1.0,
            coherent_integration_s: 0.02,
            cn0_zenith_dbhz: 48.0,
            cn0_horizon_dbhz: 36.0,
            below_horizon_loss_db: 10.0,
            tracking_threshold_dbhz: 28.0,
        }
    }

    /// Whether every parameter is finite and in range.
    pub fn is_valid(&self) -> bool {
        self.chip_length_m > 0.0
            && self.wavelength_m > 0.0
            && self.correlator_spacing_chips > 0.0
            && self.correlator_spacing_chips <= 1.0
            && self.dll_bandwidth_hz > 0.0
            && self.coherent_integration_s > 0.0
            && self.cn0_zenith_dbhz.is_finite()
            && self.cn0_horizon_dbhz.is_finite()
            && self.below_horizon_loss_db.is_finite()
            && self.below_horizon_loss_db >= 0.0
            && self.tracking_threshold_dbhz.is_finite()
    }

    /// C/N0 of an unreflected signal arriving at `elevation_rad` (antenna
    /// pattern only).
    pub fn antenna_cn0_dbhz(&self, elevation_rad: f64) -> f64 {
        if elevation_rad >= 0.0 {
            self.cn0_horizon_dbhz
                + (self.cn0_zenith_dbhz - self.cn0_horizon_dbhz) * elevation_rad.sin()
        } else {
            self.cn0_horizon_dbhz - self.below_horizon_loss_db
        }
    }

    /// One-sigma code tracking (thermal) noise of an early-minus-late power
    /// DLL in meters at the given C/N0.
    pub fn dll_noise_std_m(&self, cn0_dbhz: f64) -> f64 {
        let cn0 = 10.0_f64.powf(cn0_dbhz / 10.0);
        let d = self.correlator_spacing_chips;
        let variance_chips2 = self.dll_bandwidth_hz * d / (2.0 * cn0)
            * (1.0 + 2.0 / ((2.0 - d) * self.coherent_integration_s * cn0));
        self.chip_length_m * variance_chips2.sqrt()
    }

    /// Code-phase error in meters caused by one reflected replica with
    /// `relative_amplitude` (to the tracked signal) and `excess_delay_m`.
    ///
    /// This is the early-minus-late multipath error envelope for an
    /// infinite-bandwidth narrow correlator, evaluated at the carrier phase
    /// `2π·excess_delay/λ` that the geometry implies.
    pub fn multipath_code_error_m(&self, relative_amplitude: f64, excess_delay_m: f64) -> f64 {
        let tau = excess_delay_m / self.chip_length_m;
        let half = 0.5 * self.correlator_spacing_chips;
        if !(0.0..1.0 + half).contains(&tau) {
            return 0.0;
        }
        let phase = TAU * excess_delay_m / self.wavelength_m;
        let in_phase = relative_amplitude * phase.cos();
        let denominator = (1.0 + in_phase).max(0.05);
        let lever_chips = if tau < half {
            tau
        } else if tau < 1.0 - half {
            half
        } else {
            0.5 * (1.0 + half - tau)
        };
        self.chip_length_m * in_phase * lever_chips / denominator
    }

    /// Power change in dB when reflections with the given relative
    /// amplitudes and delays interfere with the tracked signal.
    pub fn multipath_fading_db(&self, replicas: impl IntoIterator<Item = (f64, f64)>) -> f64 {
        let (mut re, mut im) = (1.0, 0.0);
        for (amplitude, delay_m) in replicas {
            if delay_m / self.chip_length_m >= 1.0 {
                continue;
            }
            let phase = TAU * delay_m / self.wavelength_m;
            re += amplitude * phase.cos();
            im += amplitude * phase.sin();
        }
        10.0 * (re * re + im * im).max(1.0e-3).log10()
    }
}

/// Residual atmospheric delays after the receiver's broadcast corrections.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AtmosphereConfig {
    /// Vertical ionospheric delay at L1 in meters.
    pub zenith_iono_delay_m: f64,
    /// Fraction of the ionospheric delay the broadcast model leaves.
    pub iono_residual_fraction: f64,
    /// Zenith tropospheric delay in meters.
    pub zenith_tropo_delay_m: f64,
    /// Fraction of the tropospheric delay the receiver model leaves.
    pub tropo_residual_fraction: f64,
}

impl Default for AtmosphereConfig {
    fn default() -> Self {
        Self {
            zenith_iono_delay_m: 5.0,
            iono_residual_fraction: 0.4,
            zenith_tropo_delay_m: 2.4,
            tropo_residual_fraction: 0.05,
        }
    }
}

impl AtmosphereConfig {
    /// Residual (uncorrected) delay in meters for a satellite at `elevation_rad`.
    ///
    /// Uses the Klobuchar single-layer obliquity factor for the ionosphere
    /// and a Black-Eisner style mapping for the troposphere.
    pub fn residual_delay_m(&self, elevation_rad: f64) -> f64 {
        let el = elevation_rad.max(0.0);
        let semicircles = el / PI;
        let iono_obliquity = 1.0 + 16.0 * (0.53 - semicircles).powi(3);
        let tropo_mapping = 1.001 / (0.002_001 + el.sin().powi(2)).sqrt();
        self.zenith_iono_delay_m * self.iono_residual_fraction * iono_obliquity
            + self.zenith_tropo_delay_m * self.tropo_residual_fraction * tropo_mapping
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn dll_noise_falls_with_cn0() {
        let config = SignalConfig::gps_l1_ca();
        let strong = config.dll_noise_std_m(48.0);
        let weak = config.dll_noise_std_m(30.0);
        assert!(strong < 0.3, "{strong}");
        assert!(weak > 4.0 * strong);
    }

    #[test]
    fn multipath_error_is_bounded_and_vanishes_past_one_chip() {
        let config = SignalConfig::gps_l1_ca();
        let half = 0.5 * config.correlator_spacing_chips * config.chip_length_m;
        for step in 0..400 {
            let delay = step as f64 * 1.0;
            let error = config.multipath_code_error_m(0.5, delay);
            // Envelope: |e| <= a * (d/2) Tc / (1 - a).
            assert!(
                error.abs() <= 0.5 * half / 0.5 + 1e-9,
                "delay {delay}: {error}"
            );
        }
        assert_eq!(
            config.multipath_code_error_m(0.5, 1.1 * config.chip_length_m),
            0.0
        );
    }

    #[test]
    fn short_delay_in_phase_multipath_is_linear() {
        let config = SignalConfig::gps_l1_ca();
        // A delay of an integer number of wavelengths is in phase.
        let delay = 10.0 * config.wavelength_m;
        let error = config.multipath_code_error_m(0.3, delay);
        assert_relative_eq!(error, 0.3 * delay / 1.3, epsilon = 1e-9);
    }

    #[test]
    fn atmosphere_grows_toward_the_horizon() {
        let atmosphere = AtmosphereConfig::default();
        let zenith = atmosphere.residual_delay_m(std::f64::consts::FRAC_PI_2);
        let low = atmosphere.residual_delay_m(10.0_f64.to_radians());
        assert!(low > 2.0 * zenith);
        assert!(zenith > 1.0 && zenith < 4.0, "{zenith}");
    }

    #[test]
    fn destructive_interference_fades_the_signal() {
        let config = SignalConfig::gps_l1_ca();
        let half_wave = 0.5 * config.wavelength_m;
        assert!(config.multipath_fading_db([(0.5, half_wave)]) < -5.0);
        assert!(config.multipath_fading_db([(0.5, 2.0 * half_wave)]) > 3.0);
    }
}
