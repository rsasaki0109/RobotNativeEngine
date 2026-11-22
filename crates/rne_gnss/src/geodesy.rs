//! WGS-84 geodesy: geodetic, Earth-centred Earth-fixed (ECEF), and local
//! east-north-up (ENU) frames.
//!
//! The scene that GNSS signals are ray traced through lives in a local ENU
//! frame fixed at a datum; satellites and the receiver solution live in ECEF.
//! Unlike `rne_nav::NavSatTransform`'s equirectangular projection, these
//! conversions are exact on the ellipsoid, because pseudorange positioning
//! needs metre-level consistency over 20 000 km ranges.

use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// WGS-84 semi-major axis in meters.
pub const WGS84_A_M: f64 = 6_378_137.0;
/// WGS-84 flattening.
pub const WGS84_F: f64 = 1.0 / 298.257_223_563;
/// WGS-84 first eccentricity squared.
pub const WGS84_E2: f64 = WGS84_F * (2.0 - WGS84_F);

/// A geodetic position on the WGS-84 ellipsoid.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Geodetic {
    /// Geodetic latitude in radians.
    pub latitude_rad: f64,
    /// Longitude in radians.
    pub longitude_rad: f64,
    /// Height above the ellipsoid in meters.
    pub height_m: f64,
}

impl Geodetic {
    /// Creates a geodetic position from degrees and meters.
    pub fn from_degrees(latitude_deg: f64, longitude_deg: f64, height_m: f64) -> Self {
        Self {
            latitude_rad: latitude_deg.to_radians(),
            longitude_rad: longitude_deg.to_radians(),
            height_m,
        }
    }

    /// Converts to ECEF meters.
    pub fn to_ecef(self) -> Vec3 {
        let (sin_lat, cos_lat) = self.latitude_rad.sin_cos();
        let (sin_lon, cos_lon) = self.longitude_rad.sin_cos();
        let n = WGS84_A_M / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();
        Vec3::new(
            (n + self.height_m) * cos_lat * cos_lon,
            (n + self.height_m) * cos_lat * sin_lon,
            (n * (1.0 - WGS84_E2) + self.height_m) * sin_lat,
        )
    }

    /// Converts ECEF meters to geodetic coordinates (iterative, sub-millimetre).
    pub fn from_ecef(ecef_m: Vec3) -> Self {
        let p = ecef_m.x.hypot(ecef_m.y);
        let longitude_rad = ecef_m.y.atan2(ecef_m.x);
        let mut latitude_rad = ecef_m.z.atan2(p * (1.0 - WGS84_E2));
        let mut height_m = 0.0;
        for _ in 0..8 {
            let sin_lat = latitude_rad.sin();
            let n = WGS84_A_M / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();
            height_m = if latitude_rad.cos().abs() > 1.0e-9 {
                p / latitude_rad.cos() - n
            } else {
                ecef_m.z.abs() - n * (1.0 - WGS84_E2)
            };
            latitude_rad = ecef_m.z.atan2(p * (1.0 - WGS84_E2 * n / (n + height_m)));
        }
        Self {
            latitude_rad,
            longitude_rad,
            height_m,
        }
    }
}

/// A local east-north-up frame fixed at a geodetic datum.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct EnuFrame {
    /// The datum (ENU origin).
    pub datum: Geodetic,
    /// The datum in ECEF meters.
    pub origin_ecef_m: Vec3,
    east: Vec3,
    north: Vec3,
    up: Vec3,
}

impl EnuFrame {
    /// Creates an ENU frame at the given datum.
    pub fn new(datum: Geodetic) -> Self {
        let (sin_lat, cos_lat) = datum.latitude_rad.sin_cos();
        let (sin_lon, cos_lon) = datum.longitude_rad.sin_cos();
        Self {
            datum,
            origin_ecef_m: datum.to_ecef(),
            east: Vec3::new(-sin_lon, cos_lon, 0.0),
            north: Vec3::new(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat),
            up: Vec3::new(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat),
        }
    }

    /// Rotates an ECEF vector (a direction or difference) into ENU.
    pub fn ecef_vector_to_enu(&self, vector: Vec3) -> Vec3 {
        Vec3::new(
            self.east.dot(vector),
            self.north.dot(vector),
            self.up.dot(vector),
        )
    }

    /// Rotates an ENU vector into ECEF.
    pub fn enu_vector_to_ecef(&self, vector: Vec3) -> Vec3 {
        self.east * vector.x + self.north * vector.y + self.up * vector.z
    }

    /// Converts an ECEF point to local ENU meters.
    pub fn ecef_to_enu(&self, ecef_m: Vec3) -> Vec3 {
        self.ecef_vector_to_enu(ecef_m - self.origin_ecef_m)
    }

    /// Converts a local ENU point to ECEF meters.
    pub fn enu_to_ecef(&self, enu_m: Vec3) -> Vec3 {
        self.origin_ecef_m + self.enu_vector_to_ecef(enu_m)
    }
}

/// Azimuth (from north, clockwise toward east) and elevation of an ENU vector.
pub fn azimuth_elevation(enu: Vec3) -> (f64, f64) {
    let horizontal = enu.x.hypot(enu.y);
    let mut azimuth_rad = enu.x.atan2(enu.y);
    if azimuth_rad < 0.0 {
        azimuth_rad += std::f64::consts::TAU;
    }
    (azimuth_rad, enu.z.atan2(horizontal))
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn geodetic_ecef_round_trip() {
        let tokyo = Geodetic::from_degrees(35.681, 139.767, 40.0);
        let back = Geodetic::from_ecef(tokyo.to_ecef());
        assert_relative_eq!(back.latitude_rad, tokyo.latitude_rad, epsilon = 1e-11);
        assert_relative_eq!(back.longitude_rad, tokyo.longitude_rad, epsilon = 1e-11);
        assert_relative_eq!(back.height_m, tokyo.height_m, epsilon = 1e-4);
    }

    #[test]
    fn equator_prime_meridian_is_on_the_x_axis() {
        let ecef = Geodetic::from_degrees(0.0, 0.0, 0.0).to_ecef();
        assert_relative_eq!(ecef.x, WGS84_A_M, epsilon = 1e-6);
        assert_relative_eq!(ecef.y, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn enu_round_trip_and_axes() {
        let frame = EnuFrame::new(Geodetic::from_degrees(36.08, 140.11, 25.0));
        let point = Vec3::new(120.0, -35.0, 12.0);
        let back = frame.ecef_to_enu(frame.enu_to_ecef(point));
        assert_relative_eq!(back.x, point.x, epsilon = 1e-6);
        assert_relative_eq!(back.y, point.y, epsilon = 1e-6);
        assert_relative_eq!(back.z, point.z, epsilon = 1e-6);
        // Straight up in ENU raises the ellipsoidal height by the same amount.
        let up = Geodetic::from_ecef(frame.enu_to_ecef(Vec3::new(0.0, 0.0, 100.0)));
        assert_relative_eq!(up.height_m, 125.0, epsilon = 1e-4);
    }

    #[test]
    fn azimuth_is_clockwise_from_north() {
        let (az, el) = azimuth_elevation(Vec3::new(1.0, 0.0, 0.0));
        assert_relative_eq!(az, std::f64::consts::FRAC_PI_2, epsilon = 1e-12);
        assert_relative_eq!(el, 0.0, epsilon = 1e-12);
        let (_, el) = azimuth_elevation(Vec3::new(0.0, 1.0, 1.0));
        assert_relative_eq!(el, std::f64::consts::FRAC_PI_4, epsilon = 1e-12);
    }
}
