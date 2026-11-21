//! Broadcast-model atmospheric delays: the GPS Klobuchar ionosphere and the
//! Saastamoinen troposphere with a standard atmosphere.
//!
//! The receiver simulation applies the *true* delays (the models scaled by
//! [`AtmosphereConfig`] truth factors) to the measurements, while the
//! broadcast Klobuchar coefficients are published with the ephemerides; a
//! solver that applies the broadcast models is left with the difference, the
//! way a real single-frequency receiver is.

use crate::time::GnssTime;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// GPS L1 frequency in Hz; ionospheric delays are quoted at L1.
pub const GPS_L1_HZ: f64 = 1_575.42e6;

/// Klobuchar broadcast ionosphere coefficients.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct KlobucharParams {
    /// Amplitude coefficients (s, s/semicircle, ...).
    pub alpha: [f64; 4],
    /// Period coefficients (s, s/semicircle, ...).
    pub beta: [f64; 4],
}

impl Default for KlobucharParams {
    /// A moderate-activity broadcast set.
    fn default() -> Self {
        Self {
            alpha: [0.1118e-7, -0.7451e-8, -0.5961e-7, 0.1192e-6],
            beta: [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
        }
    }
}

impl KlobucharParams {
    /// Klobuchar L1 ionospheric delay in meters (IS-GPS-200 20.3.3.5.2.5).
    pub fn delay_l1_m(
        &self,
        time: GnssTime,
        latitude_rad: f64,
        longitude_rad: f64,
        azimuth_rad: f64,
        elevation_rad: f64,
    ) -> f64 {
        if elevation_rad <= 0.0 {
            return 0.0;
        }
        let el = elevation_rad / PI;
        let psi = 0.0137 / (el + 0.11) - 0.022;
        let mut phi = (latitude_rad / PI + psi * azimuth_rad.cos()).clamp(-0.416, 0.416);
        let lambda = longitude_rad / PI + psi * azimuth_rad.sin() / (phi * PI).cos();
        phi += 0.064 * ((lambda - 1.617) * PI).cos();
        let mut local_s = 43_200.0 * lambda + time.tow_s;
        local_s -= (local_s / 86_400.0).floor() * 86_400.0;
        let obliquity = 1.0 + 16.0 * (0.53 - el).powi(3);
        let poly = |c: &[f64; 4]| c[0] + phi * (c[1] + phi * (c[2] + phi * c[3]));
        let amplitude = poly(&self.alpha).max(0.0);
        let period = poly(&self.beta).max(72_000.0);
        let x = 2.0 * PI * (local_s - 50_400.0) / period;
        let seconds = if x.abs() < 1.57 {
            5.0e-9 + amplitude * (1.0 + x * x * (-0.5 + x * x / 24.0))
        } else {
            5.0e-9
        };
        crate::orbit::SPEED_OF_LIGHT_M_S * obliquity * seconds
    }
}

/// Saastamoinen zenith-mapped tropospheric delay in meters with a standard
/// atmosphere (relative humidity `humidity`).
pub fn saastamoinen_delay_m(
    latitude_rad: f64,
    height_m: f64,
    elevation_rad: f64,
    humidity: f64,
) -> f64 {
    if elevation_rad <= 0.0 || !(-100.0..=1.0e4).contains(&height_m) {
        return 0.0;
    }
    let h = height_m.max(0.0);
    let pressure = 1013.25 * (1.0 - 2.2557e-5 * h).powf(5.2568);
    let temperature = 15.0 - 6.5e-3 * h + 273.16;
    let vapour = 6.108 * humidity * ((17.15 * temperature - 4684.0) / (temperature - 38.45)).exp();
    let zenith = std::f64::consts::FRAC_PI_2 - elevation_rad;
    let hydrostatic = 0.002_276_8 * pressure
        / (1.0 - 0.002_66 * (2.0 * latitude_rad).cos() - 0.000_28 * h / 1.0e3)
        / zenith.cos();
    let wet = 0.002_277 * (1255.0 / temperature + 0.05) * vapour / zenith.cos();
    hydrostatic + wet
}

/// Truth model of the atmosphere relative to the broadcast models.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AtmosphereConfig {
    /// Broadcast Klobuchar coefficients (published with the navigation data).
    pub klobuchar: KlobucharParams,
    /// True ionosphere = Klobuchar x this factor (Klobuchar removes ~50-70 %).
    pub iono_truth_scale: f64,
    /// True troposphere = Saastamoinen x this factor.
    pub tropo_truth_scale: f64,
    /// Relative humidity used by the Saastamoinen model.
    pub humidity: f64,
}

impl Default for AtmosphereConfig {
    fn default() -> Self {
        Self {
            klobuchar: KlobucharParams::default(),
            iono_truth_scale: 1.4,
            tropo_truth_scale: 1.03,
            humidity: 0.7,
        }
    }
}

impl AtmosphereConfig {
    /// Whether the configuration is finite.
    pub fn is_valid(&self) -> bool {
        self.iono_truth_scale.is_finite()
            && self.iono_truth_scale >= 0.0
            && self.tropo_truth_scale.is_finite()
            && self.tropo_truth_scale >= 0.0
            && (0.0..=1.0).contains(&self.humidity)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn klobuchar_is_metres_and_grows_toward_the_horizon() {
        let params = KlobucharParams::default();
        let noon = GnssTime::new(2400, 2.0 * 86_400.0 + 4.0 * 3_600.0);
        let (lat, lon) = (36.0_f64.to_radians(), 140.0_f64.to_radians());
        let zenith = params.delay_l1_m(noon, lat, lon, 0.0, std::f64::consts::FRAC_PI_2);
        let low = params.delay_l1_m(noon, lat, lon, 0.0, 10.0_f64.to_radians());
        assert!(zenith > 1.0 && zenith < 15.0, "{zenith}");
        assert!(low > 2.0 * zenith);
    }

    #[test]
    fn saastamoinen_zenith_is_about_2_4_m_at_sea_level() {
        let zenith = saastamoinen_delay_m(0.6, 0.0, std::f64::consts::FRAC_PI_2, 0.7);
        assert!((2.3..2.6).contains(&zenith), "{zenith}");
        let low = saastamoinen_delay_m(0.6, 0.0, 10.0_f64.to_radians(), 0.7);
        assert!(low > 5.0 * zenith);
    }
}
