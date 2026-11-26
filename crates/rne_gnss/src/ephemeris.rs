//! Broadcast ephemerides in the GPS LNAV / Galileo I/NAV Keplerian form.
//!
//! The simulated orbits are exported as broadcast records so that an external
//! solver (for example libgnss++ through `adapters/gnsspp`) computes exactly
//! the satellite positions the simulation used. Propagation follows
//! IS-GPS-200 Table 20-IV with the GPS value of the Earth's gravitational
//! constant for GPS, Galileo and QZSS, as libgnss++ does by default, and the
//! BeiDou ICD's CGCS2000 constants for BeiDou, whose geostationary
//! satellites (PRN 1-5 and 59-63) are given in a frame tilted by 5° and
//! turned with the Earth afterwards. The records reproduce the simulated
//! orbits in the form the receiver applies (geostationary ones within a few
//! tenths of a metre over two hours).

use crate::atmosphere::KlobucharParams;
use crate::orbit::{
    Constellation, GnssSystem, KeplerOrbit, SatelliteId, BEIDOU_MU_M3_S2, BEIDOU_ROTATION_RAD_S,
    EARTH_MU_M3_S2, EARTH_ROTATION_RAD_S,
};
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
        if id.is_beidou_geo() {
            return Self::beidou_geo(id, orbit, toe, reference_s);
        }
        let (mu, rotation) = constants(id.system);
        let a = orbit.semi_major_axis_m;
        Self {
            id,
            toe,
            sqrt_a: a.sqrt(),
            eccentricity: orbit.eccentricity,
            i0_rad: orbit.inclination_rad,
            omega0_rad: orbit.ascending_node_rad - EARTH_ROTATION_RAD_S * reference_s
                + rotation * toe.tow_s,
            omega_rad: orbit.argument_of_perigee_rad,
            m0_rad: orbit.mean_anomaly_rad + orbit.mean_motion_rad_s() * reference_s,
            // The simulation's orbits use the GPS constants; a system with its
            // own takes the difference in its mean motion and node rate.
            delta_n_rad_s: orbit.mean_motion_rad_s() - (mu / (a * a * a)).sqrt(),
            omega_dot_rad_s: rotation - EARTH_ROTATION_RAD_S,
            idot_rad_s: 0.0,
            af0_s: 0.0,
            af1: 0.0,
            iode: 1,
        }
    }

    /// A BeiDou geostationary record: the orbit's state at toe, in the
    /// inertial frame aligned with the Earth at toe and tilted by -5° about x,
    /// as Keplerian elements.
    fn beidou_geo(id: SatelliteId, orbit: &KeplerOrbit, toe: GnssTime, reference_s: f64) -> Self {
        let r = orbit.position_ecef(reference_s);
        let dt = 1.0;
        let v_ecef = (orbit.position_ecef(reference_s + dt)
            - orbit.position_ecef(reference_s - dt))
            / (2.0 * dt);
        let v = v_ecef
            + Vec3::new(
                -BEIDOU_ROTATION_RAD_S * r.y,
                BEIDOU_ROTATION_RAD_S * r.x,
                0.0,
            );
        let tilt = |p: Vec3| {
            let (sin, cos) = (-5.0_f64).to_radians().sin_cos();
            Vec3::new(p.x, p.y * cos - p.z * sin, p.y * sin + p.z * cos)
        };
        let k = kepler_from_state(tilt(r), tilt(v), BEIDOU_MU_M3_S2);
        Self {
            id,
            toe,
            sqrt_a: k.semi_major_axis_m.sqrt(),
            eccentricity: k.eccentricity,
            i0_rad: k.inclination_rad,
            omega0_rad: k.ascending_node_rad + BEIDOU_ROTATION_RAD_S * toe.tow_s,
            omega_rad: k.argument_of_perigee_rad,
            m0_rad: k.mean_anomaly_rad,
            // The elements come from the state seen from the Earth turning
            // at BeiDou's rate, which already keeps the satellite over its
            // longitude; the simulated Earth turns at the GPS rate, a
            // difference of about 0.15 m an hour from toe.
            delta_n_rad_s: 0.0,
            omega_dot_rad_s: 0.0,
            idot_rad_s: 0.0,
            af0_s: 0.0,
            af1: 0.0,
            iode: 1,
        }
    }

    /// ECEF satellite position at `time` (IS-GPS-200 algorithm; the BeiDou
    /// ICD's for BeiDou).
    pub fn position_ecef(&self, time: GnssTime) -> Vec3 {
        let (mu, rotation) = constants(self.id.system);
        let a = self.sqrt_a * self.sqrt_a;
        let tk = time.seconds_since(self.toe);
        let n = (mu / (a * a * a)).sqrt() + self.delta_n_rad_s;
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
        let (x, y) = (r * u.cos(), r * u.sin());
        let (sin_i, cos_i) = inclination.sin_cos();
        if self.id.is_beidou_geo() {
            let node = self.omega0_rad + self.omega_dot_rad_s * tk - rotation * self.toe.tow_s;
            let (sin_node, cos_node) = node.sin_cos();
            let g = Vec3::new(
                x * cos_node - y * cos_i * sin_node,
                x * sin_node + y * cos_i * cos_node,
                y * sin_i,
            );
            // Untilt by 5° about x, then turn with the Earth.
            let (sin5, cos5) = 5.0_f64.to_radians().sin_cos();
            let (gx, gy, gz) = (g.x, g.y * cos5 - g.z * sin5, g.y * sin5 + g.z * cos5);
            let (sin_r, cos_r) = (rotation * tk).sin_cos();
            return Vec3::new(gx * cos_r + gy * sin_r, -gx * sin_r + gy * cos_r, gz);
        }
        let node =
            self.omega0_rad + (self.omega_dot_rad_s - rotation) * tk - rotation * self.toe.tow_s;
        let (sin_node, cos_node) = node.sin_cos();
        Vec3::new(
            x * cos_node - y * cos_i * sin_node,
            x * sin_node + y * cos_i * cos_node,
            y * sin_i,
        )
    }
}

/// The gravitational parameter and Earth rotation rate a system's broadcast
/// ephemerides use.
fn constants(system: GnssSystem) -> (f64, f64) {
    match system {
        GnssSystem::BeiDou => (BEIDOU_MU_M3_S2, BEIDOU_ROTATION_RAD_S),
        _ => (EARTH_MU_M3_S2, EARTH_ROTATION_RAD_S),
    }
}

/// Keplerian elements of an inertial position and velocity.
fn kepler_from_state(r: Vec3, v: Vec3, mu: f64) -> KeplerOrbit {
    let radius = r.length();
    let h = r.cross(v);
    let node_vec = Vec3::new(-h.y, h.x, 0.0);
    let e_vec = (r * (v.length_squared() - mu / radius) - v * r.dot(v)) / mu;
    let a = 1.0 / (2.0 / radius - v.length_squared() / mu);
    let inclination = (h.z / h.length()).clamp(-1.0, 1.0).acos();
    let node = node_vec.y.atan2(node_vec.x);
    let (sin_node, cos_node) = node.sin_cos();
    let sin_i = inclination.sin().max(1.0e-12);
    // Argument of latitude from the node.
    let u = (r.z / sin_i).atan2(r.x * cos_node + r.y * sin_node);
    let e = e_vec.length();
    let (perigee, true_anomaly) = if e < 1.0e-10 {
        (0.0, u)
    } else {
        let perigee = (e_vec.z / sin_i).atan2(e_vec.x * cos_node + e_vec.y * sin_node);
        (perigee, u - perigee)
    };
    let e = if e < 1.0e-10 { 0.0 } else { e };
    let eccentric = 2.0 * (((1.0 - e) / (1.0 + e)).sqrt() * (0.5 * true_anomaly).tan()).atan();
    KeplerOrbit {
        semi_major_axis_m: a,
        eccentricity: e,
        inclination_rad: inclination,
        ascending_node_rad: node,
        argument_of_perigee_rad: perigee,
        mean_anomaly_rad: eccentric - e * eccentric.sin(),
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
    fn beidou_records_reproduce_the_orbits_in_the_beidou_forms() {
        let constellation = Constellation::beidou_nominal();
        let epoch = GnssTime::new(2300, 345_600.0);
        let records = constellation.broadcast_ephemerides(epoch, 3_600.0);
        assert_eq!(records.len(), 30);
        let geo = records.iter().filter(|r| r.id.is_beidou_geo()).count();
        assert_eq!(geo, 3);
        for (satellite, record) in constellation.satellites().iter().zip(&records) {
            for t in [0.0, 1_800.0, 3_600.0, 5_400.0, 7_200.0] {
                let truth = satellite.orbit.position_ecef(t);
                let broadcast = record.position_ecef(epoch.add_seconds(t));
                let error = (truth - broadcast).length();
                // Geostationary records drift with the difference between
                // the BeiDou and GPS Earth rotation rates.
                let tolerance = if record.id.is_beidou_geo() { 0.3 } else { 0.05 };
                assert!(error < tolerance, "{} at {t}: {error} m", record.id);
            }
        }
    }

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
