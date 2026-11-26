//! Keplerian satellite orbits and nominal GPS, Galileo, QZSS and BeiDou
//! constellations.
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
/// CGCS2000 (BeiDou) Earth gravitational parameter in m^3/s^2.
pub const BEIDOU_MU_M3_S2: f64 = 3.986_004_418e14;
/// CGCS2000 (BeiDou) Earth rotation rate in rad/s.
pub const BEIDOU_ROTATION_RAD_S: f64 = 7.292_115e-5;

/// A satellite navigation system.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum GnssSystem {
    /// US GPS.
    Gps,
    /// European Galileo.
    Galileo,
    /// Japanese Quasi-Zenith Satellite System (Michibiki), GPS-compatible
    /// L1 C/A and L2C on GPS time.
    Qzss,
    /// Chinese BeiDou (BDS-3): medium Earth orbits worldwide, and inclined
    /// geosynchronous and geostationary satellites over Asia, on B1I and B3I.
    BeiDou,
}

impl GnssSystem {
    /// One-letter RINEX system code.
    pub fn code(self) -> char {
        match self {
            Self::Gps => 'G',
            Self::Galileo => 'E',
            Self::Qzss => 'J',
            Self::BeiDou => 'C',
        }
    }
}

impl SatelliteId {
    /// Whether this is a BeiDou geostationary satellite (PRN 1-5 or 59-63),
    /// whose broadcast ephemeris is given in a frame tilted by 5°.
    pub fn is_beidou_geo(self) -> bool {
        self.system == GnssSystem::BeiDou && (self.prn <= 5 || (59..=63).contains(&self.prn))
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

    /// Eccentric anomaly at `time_s` after the epoch (Kepler's equation).
    pub fn eccentric_anomaly_rad(&self, time_s: f64) -> f64 {
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
        eccentric
    }

    /// The relativistic satellite-clock offset of an eccentric orbit,
    /// `F·e·√A·sin E` with `F = −2√μ / c²` (IS-GPS-200 20.3.3.3.3.1), in
    /// seconds; zero for a circular orbit.
    pub fn relativistic_clock_s(&self, time_s: f64) -> f64 {
        const F: f64 = -4.442_807_633e-10;
        F * self.eccentricity
            * self.semi_major_axis_m.sqrt()
            * self.eccentric_anomaly_rad(time_s).sin()
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

    /// The four-satellite QZSS: QZS-1R, -2 and -4 (J01-J03) in quasi-zenith
    /// orbits (geosynchronous, e 0.075, i 41°, perigee argument 270°) sharing
    /// one figure-eight ground track whose northern loop, at apogee, is over
    /// 135° E, one passing every 8 hours, and QZS-3 (J07) geostationary at
    /// 127° E. From Japan one QZO satellite is always high in the sky.
    pub fn qzss_nominal() -> Self {
        let a = (EARTH_MU_M3_S2 / EARTH_ROTATION_RAD_S.powi(2)).cbrt();
        let mut satellites: Vec<Satellite> = (0..3)
            .map(|k| {
                // Apogee (argument of latitude 90°, longitude node + 90°) at
                // t = 120°·k / n with the Earth-fixed node then at 45° E.
                let k = f64::from(k);
                Satellite {
                    id: SatelliteId {
                        system: GnssSystem::Qzss,
                        prn: k as u8 + 1,
                    },
                    orbit: KeplerOrbit {
                        semi_major_axis_m: a,
                        eccentricity: 0.075,
                        inclination_rad: 41.0_f64.to_radians(),
                        ascending_node_rad: (45.0 + 120.0 * k).to_radians(),
                        argument_of_perigee_rad: 270.0_f64.to_radians(),
                        mean_anomaly_rad: (180.0 - 120.0 * k).to_radians(),
                    },
                }
            })
            .collect();
        satellites.push(Satellite {
            id: SatelliteId {
                system: GnssSystem::Qzss,
                prn: 7,
            },
            orbit: KeplerOrbit::circular(a, 0.1_f64.to_radians(), 0.0, 127.0_f64.to_radians()),
        });
        Self::new(satellites)
    }

    /// Nominal BeiDou (BDS-3): 24 medium-Earth-orbit satellites in a Walker
    /// 24/3/1 constellation at 21 528 km and 55°, three inclined
    /// geosynchronous satellites (55°) whose figure-of-eight ground tracks
    /// cross the equator at 118° E, and three geostationary satellites at
    /// 140°, 80° and 110.5° E (PRNs 59-61).
    pub fn beidou_nominal() -> Self {
        let meo_a = 6_378_137.0 + 21_528_000.0;
        let meo_prns: Vec<u8> = (19..=30).chain(32..=37).chain(41..=46).collect();
        let mut satellites: Vec<Satellite> = meo_prns
            .iter()
            .enumerate()
            .map(|(k, &prn)| {
                let (plane, slot) = ((k / 8) as f64, (k % 8) as f64);
                Satellite {
                    id: SatelliteId {
                        system: GnssSystem::BeiDou,
                        prn,
                    },
                    orbit: KeplerOrbit::circular(
                        meo_a,
                        55.0_f64.to_radians(),
                        (120.0 * plane + 30.0).to_radians(),
                        (45.0 * slot + 15.0 * plane).to_radians(),
                    ),
                }
            })
            .collect();
        let geo_a = (EARTH_MU_M3_S2 / EARTH_ROTATION_RAD_S.powi(2)).cbrt();
        for (k, prn) in [38_u8, 39, 40].into_iter().enumerate() {
            // Node 120° apart; each reaches its node when the Earth has turned
            // it to 118° E, so the three share one ground track.
            let k = k as f64;
            satellites.push(Satellite {
                id: SatelliteId {
                    system: GnssSystem::BeiDou,
                    prn,
                },
                orbit: KeplerOrbit::circular(
                    geo_a,
                    55.0_f64.to_radians(),
                    (118.0 + 120.0 * k).to_radians(),
                    (-120.0 * k).to_radians(),
                ),
            });
        }
        for (prn, longitude_deg) in [(59_u8, 140.0_f64), (60, 80.0), (61, 110.5)] {
            satellites.push(Satellite {
                id: SatelliteId {
                    system: GnssSystem::BeiDou,
                    prn,
                },
                orbit: KeplerOrbit::circular(
                    geo_a,
                    0.5_f64.to_radians(),
                    0.0,
                    longitude_deg.to_radians(),
                ),
            });
        }
        Self::new(satellites)
    }

    /// GPS, Galileo, QZSS and BeiDou together (a receiver in Japan tracking
    /// four systems).
    pub fn gps_galileo_qzss_beidou_nominal() -> Self {
        let mut satellites = Self::gps_galileo_qzss_nominal().satellites;
        satellites.extend(Self::beidou_nominal().satellites);
        Self::new(satellites)
    }

    /// GPS, Galileo and QZSS together (a receiver in Japan).
    pub fn gps_galileo_qzss_nominal() -> Self {
        let mut satellites = Self::gps_galileo_nominal().satellites;
        satellites.extend(Self::qzss_nominal().satellites);
        Self::new(satellites)
    }

    /// GPS and Galileo together.
    pub fn gps_galileo_nominal() -> Self {
        let mut satellites = Self::gps_nominal().satellites;
        satellites.extend(Self::galileo_nominal().satellites);
        Self::new(satellites)
    }

    /// Broadcast ephemerides of every satellite, with orbit time zero at
    /// `epoch` and reference time `epoch + reference_s` (keep simulation
    /// times within two hours of it, as real broadcast records are).
    pub fn broadcast_ephemerides(
        &self,
        epoch: crate::time::GnssTime,
        reference_s: f64,
    ) -> Vec<crate::ephemeris::BroadcastEphemeris> {
        self.satellites
            .iter()
            .map(|s| {
                crate::ephemeris::BroadcastEphemeris::from_orbit(s.id, &s.orbit, epoch, reference_s)
            })
            .collect()
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

    #[test]
    fn beidou_over_tokyo_has_its_geostationary_and_inclined_satellites_up() {
        use crate::geodesy::{EnuFrame, Geodetic};
        let tokyo = EnuFrame::new(Geodetic::from_degrees(35.69, 139.70, 40.0));
        let beidou = Constellation::beidou_nominal();
        assert_eq!(beidou.len(), 30);
        let elevation = |s: &Satellite, t: f64| {
            let enu = tokyo.ecef_to_enu(s.orbit.position_ecef(t));
            enu.z.atan2(enu.x.hypot(enu.y)).to_degrees()
        };
        let by_prn = |prn: u8| {
            beidou
                .satellites()
                .iter()
                .find(|s| s.id.prn == prn)
                .copied()
                .expect("prn")
        };
        // The geostationary satellites stay put: 140° E high in the south
        // (48.5°), 110.5° E lower (38°), 80° E low in the south-west (16°).
        for t in [0.0, 21_600.0, 43_200.0] {
            assert!(
                (40.0..55.0).contains(&elevation(&by_prn(59), t)),
                "{}",
                elevation(&by_prn(59), t)
            );
            assert!((33.0..43.0).contains(&elevation(&by_prn(61), t)));
            assert!((11.0..21.0).contains(&elevation(&by_prn(60), t)));
        }
        // At any time of day at least one inclined geosynchronous satellite is
        // well up, and the whole system adds several satellites.
        for k in 0..24 {
            let t = 3_600.0 * f64::from(k);
            let igso = [38, 39, 40]
                .iter()
                .map(|p| elevation(&by_prn(*p), t))
                .fold(f64::MIN, f64::max);
            assert!(igso > 25.0, "{igso} at {t}");
            let up = beidou
                .satellites()
                .iter()
                .filter(|s| elevation(s, t) > 15.0)
                .count();
            assert!(up >= 6, "{up} at {t}");
        }
    }

    #[test]
    fn relativistic_clock_term_vanishes_for_circular_orbits_and_is_large_for_qzo() {
        let gps = Constellation::gps_nominal();
        assert_eq!(gps.satellites()[0].orbit.relativistic_clock_s(1234.0), 0.0);
        let qzo = Constellation::qzss_nominal().satellites()[0].orbit;
        let peak = (0..96)
            .map(|k| (qzo.relativistic_clock_s(f64::from(k) * 900.0) * SPEED_OF_LIGHT_M_S).abs())
            .fold(0.0, f64::max);
        // F·e·√A·c ≈ 4.44e-10 · 0.075 · 6493 · 3e8 ≈ 65 m.
        assert!((peak - 64.9).abs() < 1.5, "{peak}");
    }

    #[test]
    fn a_qzss_satellite_is_always_high_over_tokyo() {
        let qzss = Constellation::qzss_nominal();
        let tokyo = crate::geodesy::Geodetic::from_degrees(35.68, 139.77, 40.0);
        let frame = crate::geodesy::EnuFrame::new(tokyo);
        let elevation = |s: &Satellite, t: f64| {
            let d = frame.ecef_vector_to_enu(s.orbit.position_ecef(t) - tokyo.to_ecef());
            (d.z / d.length()).asin().to_degrees()
        };
        for hour in 0..48 {
            let t = f64::from(hour) * 1800.0;
            let highest = qzss.satellites()[..3]
                .iter()
                .map(|s| elevation(s, t))
                .fold(f64::NEG_INFINITY, f64::max);
            assert!(highest > 60.0, "t {t}: {highest}");
            // The geostationary QZS-3 stays put, about 48° up to the south.
            assert!((elevation(&qzss.satellites()[3], t) - 48.0).abs() < 3.0);
        }
        // Apogee over 135° E, about 41° N.
        let apogee =
            crate::geodesy::Geodetic::from_ecef(qzss.satellites()[0].orbit.position_ecef(0.0));
        assert!(
            (apogee.longitude_rad.to_degrees() - 135.0).abs() < 1.0,
            "{apogee:?}"
        );
        assert!(
            (apogee.latitude_rad.to_degrees() - 41.0).abs() < 2.0,
            "{apogee:?}"
        );
    }
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
