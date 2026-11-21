//! Keplerian satellite orbits and nominal GPS / Galileo constellations.
//!
//! Orbits are propagated from Keplerian elements with a Newton solve of
//! Kepler's equation and rotated into the Earth-fixed frame with the WGS-84
//! Earth rotation rate, as the GPS interface specification does for broadcast
//! ephemerides (without the harmonic correction terms). Time is simulation
//! time in seconds since the constellation epoch, never wall-clock time.

use rne_math::Vec3;
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;

/// WGS-84 Earth gravitational parameter in m^3/s^2.
pub const EARTH_MU_M3_S2: f64 = 3.986_005e14;
/// WGS-84 Earth rotation rate in rad/s.
pub const EARTH_ROTATION_RAD_S: f64 = 7.292_115_146_7e-5;
/// Speed of light in m/s.
pub const SPEED_OF_LIGHT_M_S: f64 = 299_792_458.0;

/// A satellite navigation system.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum GnssSystem {
    /// US GPS.
    Gps,
    /// European Galileo.
    Galileo,
}

impl GnssSystem {
    /// One-letter RINEX system code.
    pub fn code(self) -> char {
        match self {
            Self::Gps => 'G',
            Self::Galileo => 'E',
        }
    }
}

/// Identifies one satellite.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct SatelliteId {
    /// Navigation system.
    pub system: GnssSystem,
    /// Pseudo-random noise code number within the system.
    pub prn: u8,
}

impl std::fmt::Display for SatelliteId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}{:02}", self.system.code(), self.prn)
    }
}

/// Keplerian orbital elements at the constellation epoch.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct KeplerOrbit {
    /// Semi-major axis in meters.
    pub semi_major_axis_m: f64,
    /// Eccentricity.
    pub eccentricity: f64,
    /// Inclination in radians.
    pub inclination_rad: f64,
    /// Longitude of the ascending node at the epoch (Earth-fixed) in radians.
    pub ascending_node_rad: f64,
    /// Argument of perigee in radians.
    pub argument_of_perigee_rad: f64,
    /// Mean anomaly at the epoch in radians.
    pub mean_anomaly_rad: f64,
}

impl KeplerOrbit {
    /// A circular orbit with the given argument of latitude at the epoch.
    pub fn circular(
        semi_major_axis_m: f64,
        inclination_rad: f64,
        ascending_node_rad: f64,
        argument_of_latitude_rad: f64,
    ) -> Self {
        Self {
            semi_major_axis_m,
            eccentricity: 0.0,
            inclination_rad,
            ascending_node_rad,
            argument_of_perigee_rad: 0.0,
            mean_anomaly_rad: argument_of_latitude_rad,
        }
    }

    /// Mean motion in rad/s.
    pub fn mean_motion_rad_s(&self) -> f64 {
        (EARTH_MU_M3_S2 / self.semi_major_axis_m.powi(3)).sqrt()
    }

    /// Earth-fixed (ECEF) satellite position at `time_s` after the epoch.
    pub fn position_ecef(&self, time_s: f64) -> Vec3 {
        let mean = self.mean_anomaly_rad + self.mean_motion_rad_s() * time_s;
        let e = self.eccentricity;
        let mut eccentric = mean;
        for _ in 0..10 {
            let step = (eccentric - e * eccentric.sin() - mean) / (1.0 - e * eccentric.cos());
            eccentric -= step;
            if step.abs() < 1.0e-14 {
                break;
            }
        }
        let true_anomaly = ((1.0 - e * e).sqrt() * eccentric.sin()).atan2(eccentric.cos() - e);
        let radius = self.semi_major_axis_m * (1.0 - e * eccentric.cos());
        let latitude_arg = true_anomaly + self.argument_of_perigee_rad;
        let (x_plane, y_plane) = (radius * latitude_arg.cos(), radius * latitude_arg.sin());
        let node = self.ascending_node_rad - EARTH_ROTATION_RAD_S * time_s;
        let (sin_node, cos_node) = node.sin_cos();
        let (sin_inc, cos_inc) = self.inclination_rad.sin_cos();
        Vec3::new(
            x_plane * cos_node - y_plane * cos_inc * sin_node,
            x_plane * sin_node + y_plane * cos_inc * cos_node,
            y_plane * sin_inc,
        )
    }
}

/// One satellite of a constellation.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Satellite {
    /// Satellite id.
    pub id: SatelliteId,
    /// Orbit.
    pub orbit: KeplerOrbit,
}

/// A set of satellites, kept sorted by id for deterministic iteration.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Constellation {
    satellites: Vec<Satellite>,
}

/// GPS nominal semi-major axis in meters.
pub const GPS_SEMI_MAJOR_AXIS_M: f64 = 26_559_700.0;
/// Galileo nominal semi-major axis in meters.
pub const GALILEO_SEMI_MAJOR_AXIS_M: f64 = 29_600_318.0;

impl Constellation {
    /// Builds a constellation; satellites are sorted by id.
    pub fn new(mut satellites: Vec<Satellite>) -> Self {
        satellites.sort_by_key(|s| s.id);
        satellites.dedup_by_key(|s| s.id);
        Self { satellites }
    }

    /// The nominal 24-slot GPS constellation: six planes at 55° inclination
    /// with the unevenly phased slot layout of the GPS baseline constellation.
    pub fn gps_nominal() -> Self {
        // Argument of latitude (deg) per slot, planes A..F.
        const SLOTS_DEG: [[f64; 4]; 6] = [
            [268.126, 161.786, 11.676, 41.806],
            [80.956, 173.336, 309.976, 204.376],
            [111.876, 11.796, 339.666, 241.556],
            [135.226, 265.446, 35.156, 167.356],
            [197.046, 302.596, 66.066, 333.686],
            [238.886, 345.226, 105.206, 135.346],
        ];
        let mut satellites = Vec::with_capacity(24);
        for (plane, slots) in SLOTS_DEG.iter().enumerate() {
            let node_deg = 272.847 + 60.0 * plane as f64;
            for (slot, latitude_deg) in slots.iter().enumerate() {
                satellites.push(Satellite {
                    id: SatelliteId {
                        system: GnssSystem::Gps,
                        prn: (plane * 4 + slot + 1) as u8,
                    },
                    orbit: KeplerOrbit::circular(
                        GPS_SEMI_MAJOR_AXIS_M,
                        55.0_f64.to_radians(),
                        node_deg.to_radians(),
                        latitude_deg.to_radians(),
                    ),
                });
            }
        }
        Self::new(satellites)
    }

    /// The nominal Galileo constellation: a Walker 24/3/1 at 56° inclination.
    pub fn galileo_nominal() -> Self {
        let mut satellites = Vec::with_capacity(24);
        for plane in 0..3 {
            for slot in 0..8 {
                let latitude_deg = 45.0 * slot as f64 + 15.0 * plane as f64;
                satellites.push(Satellite {
                    id: SatelliteId {
                        system: GnssSystem::Galileo,
                        prn: (plane * 8 + slot + 1) as u8,
                    },
                    orbit: KeplerOrbit::circular(
                        GALILEO_SEMI_MAJOR_AXIS_M,
                        56.0_f64.to_radians(),
                        (120.0 * plane as f64).to_radians(),
                        latitude_deg.to_radians(),
                    ),
                });
            }
        }
        Self::new(satellites)
    }

    /// GPS and Galileo together.
    pub fn gps_galileo_nominal() -> Self {
        let mut satellites = Self::gps_nominal().satellites;
        satellites.extend(Self::galileo_nominal().satellites);
        Self::new(satellites)
    }

    /// Satellites in id order.
    pub fn satellites(&self) -> &[Satellite] {
        &self.satellites
    }

    /// Number of satellites.
    pub fn len(&self) -> usize {
        self.satellites.len()
    }

    /// Whether the constellation is empty.
    pub fn is_empty(&self) -> bool {
        self.satellites.is_empty()
    }
}

/// Wraps an angle to `[0, 2π)`.
pub fn wrap_two_pi(angle_rad: f64) -> f64 {
    angle_rad.rem_euclid(TAU)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn gps_period_is_half_a_sidereal_day() {
        let orbit = KeplerOrbit::circular(GPS_SEMI_MAJOR_AXIS_M, 0.96, 0.0, 0.0);
        let period_s = TAU / orbit.mean_motion_rad_s();
        assert_relative_eq!(period_s, 43_082.0, epsilon = 30.0);
    }

    #[test]
    fn circular_orbit_keeps_its_radius() {
        let orbit = KeplerOrbit::circular(GPS_SEMI_MAJOR_AXIS_M, 0.96, 1.0, 0.3);
        for t in [0.0, 1000.0, 7200.0, 40_000.0] {
            assert_relative_eq!(
                orbit.position_ecef(t).length(),
                GPS_SEMI_MAJOR_AXIS_M,
                epsilon = 1e-3
            );
        }
    }

    #[test]
    fn eccentric_orbit_spans_perigee_to_apogee() {
        let orbit = KeplerOrbit {
            eccentricity: 0.1,
            ..KeplerOrbit::circular(GPS_SEMI_MAJOR_AXIS_M, 0.96, 0.0, 0.0)
        };
        let perigee = orbit.position_ecef(0.0).length();
        let half_period = std::f64::consts::PI / orbit.mean_motion_rad_s();
        let apogee = orbit.position_ecef(half_period).length();
        assert_relative_eq!(perigee, GPS_SEMI_MAJOR_AXIS_M * 0.9, epsilon = 1e-3);
        assert_relative_eq!(apogee, GPS_SEMI_MAJOR_AXIS_M * 1.1, epsilon = 1e-3);
    }

    #[test]
    fn nominal_constellations_are_complete_and_sorted() {
        let all = Constellation::gps_galileo_nominal();
        assert_eq!(all.len(), 48);
        assert!(all.satellites().windows(2).all(|w| w[0].id < w[1].id));
        assert_eq!(all.satellites()[0].id.to_string(), "G01");
        assert_eq!(all.satellites()[47].id.to_string(), "E24");
    }
}
