//! Broadcast ephemerides in the GPS LNAV / Galileo I/NAV Keplerian form.
//!
//! The simulated orbits are exported as broadcast records so that an external
//! solver (for example libgnss++ through `adapters/gnsspp`) computes exactly
//! the satellite positions the simulation used. Propagation follows
//! IS-GPS-200 Table 20-IV with the GPS value of the Earth's gravitational
//! constant for both systems, as libgnss++ does by default.

use crate::atmosphere::KlobucharParams;
use crate::orbit::{Constellation, KeplerOrbit, SatelliteId, EARTH_MU_M3_S2, EARTH_ROTATION_RAD_S};
use crate::time::GnssTime;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// One broadcast ephemeris record.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct BroadcastEphemeris {
    /// Satellite.
    pub id: SatelliteId,
    /// Reference time of ephemeris (and of clock).
    pub toe: GnssTime,
    /// Square root of the semi-major axis in √m.
    pub sqrt_a: f64,
    /// Eccentricity.
    pub eccentricity: f64,
    /// Inclination at toe in radians.
    pub i0_rad: f64,
    /// Longitude of the ascending node at the weekly epoch in radians.
    pub omega0_rad: f64,
    /// Argument of perigee in radians.
    pub omega_rad: f64,
    /// Mean anomaly at toe in radians.
    pub m0_rad: f64,
    /// Mean motion difference in rad/s.
    pub delta_n_rad_s: f64,
    /// Rate of right ascension in rad/s.
    pub omega_dot_rad_s: f64,
    /// Rate of inclination in rad/s.
    pub idot_rad_s: f64,
    /// Clock bias in seconds.
    pub af0_s: f64,
    /// Clock drift in s/s.
    pub af1: f64,
    /// Issue of data.
    pub iode: u16,
}

impl BroadcastEphemeris {
    /// Exports a Keplerian orbit, whose time zero is `epoch`, as a broadcast
    /// record with reference time `toe = epoch + reference_s`.
    pub fn from_orbit(
        id: SatelliteId,
        orbit: &KeplerOrbit,
        epoch: GnssTime,
        reference_s: f64,
    ) -> Self {
        let toe = epoch.add_seconds(reference_s);
        Self {
            id,
            toe,
            sqrt_a: orbit.semi_major_axis_m.sqrt(),
            eccentricity: orbit.eccentricity,
            i0_rad: orbit.inclination_rad,
            omega0_rad: orbit.ascending_node_rad - EARTH_ROTATION_RAD_S * reference_s
                + EARTH_ROTATION_RAD_S * toe.tow_s,
            omega_rad: orbit.argument_of_perigee_rad,
            m0_rad: orbit.mean_anomaly_rad + orbit.mean_motion_rad_s() * reference_s,
            delta_n_rad_s: 0.0,
            omega_dot_rad_s: 0.0,
            idot_rad_s: 0.0,
            af0_s: 0.0,
            af1: 0.0,
            iode: 1,
        }
    }

    /// ECEF satellite position at `time` (IS-GPS-200 algorithm).
    pub fn position_ecef(&self, time: GnssTime) -> Vec3 {
        let a = self.sqrt_a * self.sqrt_a;
        let tk = time.seconds_since(self.toe);
        let n = (EARTH_MU_M3_S2 / (a * a * a)).sqrt() + self.delta_n_rad_s;
        let mean = self.m0_rad + n * tk;
        let e = self.eccentricity;
        let mut eccentric = mean;
        for _ in 0..12 {
            let step = (eccentric - e * eccentric.sin() - mean) / (1.0 - e * eccentric.cos());
            eccentric -= step;
            if step.abs() < 1.0e-14 {
                break;
            }
        }
        let true_anomaly = ((1.0 - e * e).sqrt() * eccentric.sin()).atan2(eccentric.cos() - e);
        let u = true_anomaly + self.omega_rad;
        let r = a * (1.0 - e * eccentric.cos());
        let inclination = self.i0_rad + self.idot_rad_s * tk;
        let node = self.omega0_rad + (self.omega_dot_rad_s - EARTH_ROTATION_RAD_S) * tk
            - EARTH_ROTATION_RAD_S * self.toe.tow_s;
        let (x, y) = (r * u.cos(), r * u.sin());
        let (sin_node, cos_node) = node.sin_cos();
        let (sin_i, cos_i) = inclination.sin_cos();
        Vec3::new(
            x * cos_node - y * cos_i * sin_node,
            x * sin_node + y * cos_i * cos_node,
            y * sin_i,
        )
    }
}

/// Everything a receiver decodes from the navigation message.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct NavigationMessage {
    /// One broadcast record per satellite, in satellite-id order.
    pub ephemerides: Vec<BroadcastEphemeris>,
    /// Broadcast Klobuchar coefficients.
    pub klobuchar: KlobucharParams,
}

impl NavigationMessage {
    /// The navigation message for a constellation whose orbit time zero is
    /// `epoch`, with reference time `epoch + reference_s`.
    pub fn from_constellation(
        constellation: &Constellation,
        epoch: GnssTime,
        reference_s: f64,
        klobuchar: KlobucharParams,
    ) -> Self {
        Self {
            ephemerides: constellation.broadcast_ephemerides(epoch, reference_s),
            klobuchar,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::orbit::Constellation;

    #[test]
    fn broadcast_record_reproduces_the_simulated_orbit() {
        let epoch = GnssTime::default();
        for sat in Constellation::gps_galileo_nominal().satellites() {
            let eph = BroadcastEphemeris::from_orbit(sat.id, &sat.orbit, epoch, 1_800.0);
            for t in [0.0, 600.0, 3_600.0, 20_000.0] {
                let truth = sat.orbit.position_ecef(t);
                let broadcast = eph.position_ecef(epoch.add_seconds(t));
                assert!((truth - broadcast).length() < 1.0e-4, "{} at {t}", sat.id);
            }
        }
    }
}
