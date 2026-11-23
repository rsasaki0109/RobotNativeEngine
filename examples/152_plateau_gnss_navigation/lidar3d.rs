//! A 16-ring 3D LiDAR on the robot, ray traced through the PLATEAU mesh,
//! and its prior map: points sampled from the same mesh and the terrain.

use crate::crowd::{PEDESTRIAN_HEIGHT_M, PEDESTRIAN_RADIUS_M};
use crate::nav::gaussian;
use crate::plateau::PlateauCity;
use crate::traffic::{vehicle_range, Movers};
use rne_core::DeterministicRng;
use rne_math::Vec3;
use rne_nav::Pose2d;
use rne_slam::{voxel_downsample, PointCloudMap, PointCloudMatch, PointCloudMatchConfig};

/// Rings from -15 to +15 degrees, 2 degrees apart (a VLP-16-like sensor).
pub(crate) const RINGS: usize = 16;
/// Azimuth steps per revolution.
pub(crate) const AZIMUTHS: usize = 360;
/// Maximum range, meters.
pub(crate) const RANGE_M: f64 = 60.0;
/// Height of the sensor above the ground, meters; it is held level.
pub(crate) const SENSOR_HEIGHT_M: f64 = 0.6;
const NOISE_M: f64 = 0.02;
/// Map points are kept up to this height above the local ground, meters
/// (the top ring reaches 0.6 + 60 tan 15° ≈ 16.7 m at full range).
const MAP_MAX_HEIGHT_M: f64 = 18.0;
/// Spacing of the map's samples on facades and the ground, meters.
const MAP_SPACING_M: f64 = 0.4;
/// Scan voxel size for matching, meters.
const SCAN_VOXEL_M: f64 = 0.6;

fn elevation(ring: usize) -> f64 {
    (-15.0 + 2.0 * ring as f64).to_radians()
}

/// One scan in the level sensor frame (x forward), from the sensor at
/// `pose` (planar) and height `ground_m + SENSOR_HEIGHT_M`. Pedestrians are
/// upright cylinders; rain drops returns, adds noise and returns some beams
/// early off drops. The random draws per ray are fixed, so runs stay
/// comparable.
pub(crate) fn scan(
    city: &PlateauCity,
    movers: &Movers,
    rain: bool,
    pose: Pose2d,
    ground_m: f64,
    rng: &mut DeterministicRng,
) -> Vec<Vec3> {
    let origin = Vec3::new(pose.x_m, pose.y_m, ground_m + SENSOR_HEIGHT_M);
    let mut out = Vec::with_capacity(RINGS * AZIMUTHS);
    for ring in 0..RINGS {
        let (sin_el, cos_el) = elevation(ring).sin_cos();
        for k in 0..AZIMUTHS {
            let az = std::f64::consts::TAU * k as f64 / AZIMUTHS as f64;
            let (noise, drop, rain_hit, rain_range) = (
                gaussian(rng),
                rng.uniform_f64(0.0, 1.0),
                rng.uniform_f64(0.0, 1.0),
                rng.uniform_f64(0.3, 4.0),
            );
            let (sin_w, cos_w) = (pose.yaw_rad + az).sin_cos();
            let world = Vec3::new(cos_el * cos_w, cos_el * sin_w, sin_el);
            let mut range = city
                .mesh
                .raycast(origin, world, RANGE_M)
                .map_or(f64::INFINITY, |(t, _)| t);
            if let Some(t) = ground_hit(city, origin, world, range) {
                range = t;
            }
            if let Some(t) = person_hit(city, &movers.people, origin, world) {
                range = range.min(t);
            }
            if let Some(t) = vehicle_range(&movers.vehicles, origin, world) {
                range = range.min(t);
            }
            if !range.is_finite() || range > RANGE_M {
                continue;
            }
            let range = if rain {
                if drop < 0.15 {
                    continue;
                }
                if rain_hit < 0.02 && rain_range < range {
                    rain_range
                } else {
                    range + 3.0 * NOISE_M * noise
                }
            } else {
                range + NOISE_M * noise
            };
            let (sin_a, cos_a) = az.sin_cos();
            out.push(Vec3::new(cos_el * cos_a, cos_el * sin_a, sin_el) * range);
        }
    }
    out
}

/// First crossing of the terrain along a downward ray before `max_t`.
fn ground_hit(city: &PlateauCity, origin: Vec3, dir: Vec3, max_t: f64) -> Option<f64> {
    if dir.z >= -1.0e-3 {
        return None;
    }
    // The terrain is smooth: a few fixed-point steps on its height.
    let mut t = (origin.z - city.ground_m(origin.x, origin.y)) / -dir.z;
    for _ in 0..4 {
        let p = origin + dir * t;
        t = (origin.z - city.ground_m(p.x, p.y)) / -dir.z;
    }
    (t > 0.0 && t < max_t).then_some(t)
}

/// Range to the nearest pedestrian (an upright cylinder) along a ray.
fn person_hit(city: &PlateauCity, people: &[(f64, f64)], origin: Vec3, dir: Vec3) -> Option<f64> {
    let horizontal = dir.x.hypot(dir.y);
    if horizontal < 1.0e-9 {
        return None;
    }
    let (dx, dy) = (dir.x / horizontal, dir.y / horizontal);
    people
        .iter()
        .filter_map(|&(px, py)| {
            let (fx, fy) = (origin.x - px, origin.y - py);
            let b = fx * dx + fy * dy;
            let c = fx * fx + fy * fy - PEDESTRIAN_RADIUS_M * PEDESTRIAN_RADIUS_M;
            let disc = b * b - c;
            let along = -b - disc.max(0.0).sqrt();
            if disc < 0.0 || along <= 0.0 {
                return None;
            }
            let t = along / horizontal;
            let z = origin.z + dir.z * t - city.ground_m(px, py);
            (0.0..=PEDESTRIAN_HEIGHT_M).contains(&z).then_some(t)
        })
        .fold(None, |best: Option<f64>, t| {
            Some(best.map_or(t, |b| b.min(t)))
        })
}

/// The prior map: the mesh's surfaces and the open terrain within
/// `corridor_m` of the route, up to `MAP_MAX_HEIGHT_M` above the ground,
/// sampled every `MAP_SPACING_M`, with exact normals. Prints its size.
pub(crate) fn build_map(
    city: &PlateauCity,
    route: &[(f64, f64)],
    corridor_m: f64,
) -> PointCloudMap {
    let started = std::time::Instant::now();
    let near = |x: f64, y: f64| crate::nav::distance_to_route(route, x, y) <= corridor_m;
    let (mut points, mut normals) = (Vec::new(), Vec::new());
    for triangle in city.mesh.triangles() {
        let [a, b, c] = triangle.vertices;
        let normal = triangle.normal;
        let longest = (b - a).length().max((c - b).length()).max((a - c).length());
        let k = (longest / MAP_SPACING_M).ceil().max(1.0) as usize;
        for i in 0..=k {
            for j in 0..=(k - i) {
                let (u, v) = (i as f64 / k as f64, j as f64 / k as f64);
                let p = a + (b - a) * u + (c - a) * v;
                if p.z - city.ground_m(p.x, p.y) <= MAP_MAX_HEIGHT_M && near(p.x, p.y) {
                    points.push(p);
                    normals.push(normal);
                }
            }
        }
    }
    // The open ground, with normals from the terrain's slope.
    if let Some((lo, hi)) = city.mesh.bounds() {
        let mut y = lo.y;
        while y <= hi.y {
            let mut x = lo.x;
            while x <= hi.x {
                if near(x, y) {
                    let z = city.ground_m(x, y);
                    if !city.mesh.contains(Vec3::new(x, y, z + 0.3)) {
                        let e = 0.5;
                        let gx = (city.ground_m(x + e, y) - city.ground_m(x - e, y)) / (2.0 * e);
                        let gy = (city.ground_m(x, y + e) - city.ground_m(x, y - e)) / (2.0 * e);
                        points.push(Vec3::new(x, y, z));
                        normals.push(Vec3::new(-gx, -gy, 1.0));
                    }
                }
                x += MAP_SPACING_M;
            }
            y += MAP_SPACING_M;
        }
    }
    let map = PointCloudMap::new(&points, &normals, 1.0).expect("point-cloud map");
    println!(
        "  3D LiDAR map: {} points ({:.1} s)",
        map.len(),
        started.elapsed().as_secs_f64()
    );
    map
}

/// The scan reduced for matching.
pub(crate) fn downsample(points: &[Vec3]) -> Vec<Vec3> {
    voxel_downsample(points, SCAN_VOXEL_M)
}

/// Matching tuning: a scan's points beyond a metre from the map are ignored.
pub(crate) fn match_config() -> PointCloudMatchConfig {
    PointCloudMatchConfig {
        max_correspondence_distance_m: 1.0,
        min_inlier_fraction: 0.5,
        ..PointCloudMatchConfig::default()
    }
}

/// Whether a match is good enough to use: most of the scan lies on mapped
/// surfaces (the RMS residual would also count people next to walls).
pub(crate) fn acceptable(m: &PointCloudMatch) -> bool {
    m.fit_fraction >= 0.6
}

/// A relocalizing match: tries starts on a grid over `±window_m` around
/// `around` and keeps the best acceptable one (most of the scan on mapped
/// surfaces).
pub(crate) fn search(
    map: &PointCloudMap,
    points: &[Vec3],
    around: Pose2d,
    z_m: f64,
    window_m: f64,
) -> Option<PointCloudMatch> {
    let step = (window_m / 3.0).max(0.75);
    let n = (window_m / step).round() as i32;
    let config = match_config();
    let mut best: Option<PointCloudMatch> = None;
    for i in -n..=n {
        for j in -n..=n {
            let seed = Pose2d::new(
                around.x_m + f64::from(i) * step,
                around.y_m + f64::from(j) * step,
                around.yaw_rad,
            );
            let Some(m) = map.match_scan(points, seed, z_m, &config) else {
                continue;
            };
            if acceptable(&m) && best.is_none_or(|b| m.fit_fraction > b.fit_fraction) {
                best = Some(m);
            }
        }
    }
    best
}
