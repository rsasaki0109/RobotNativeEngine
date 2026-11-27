//! A 16-ring 3D LiDAR on the robot, ray traced through the PLATEAU mesh,
//! and its prior map: points sampled from the same mesh and the terrain.

use crate::crowd::{PEDESTRIAN_HEIGHT_M, PEDESTRIAN_RADIUS_M};
use crate::nav::gaussian;
use crate::plateau::PlateauCity;
use crate::traffic::{vehicle_range, Movers};
use rne_adapter_lidarloc::{GlobalSearch, LidarLoc, LidarLocConfig, Reacquisition, Tracking};
use rne_core::DeterministicRng;
use rne_math::Vec3;
use rne_math::{DQuat, Mat4};
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

/// How the prior map differs from the city, by stretches of the route
/// (meters along it: a point belongs to the stretch its nearest route point
/// lies in).
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct MapEdit {
    /// Nothing mapped here yet.
    pub gap: Option<(f64, f64)>,
    /// The buildings here were rebuilt since the survey: the map has their
    /// facades this far (east, north, meters) from where they stand.
    pub shift: Option<((f64, f64), (f64, f64))>,
    /// An aged survey ([`Aging`], [`phantom_cars`]) instead of the world's
    /// exact surfaces.
    pub aged: bool,
}

/// The 3D LiDAR's prior map: indexed for matching, and its points (for
/// lidar_localization_ros2's registration with `--lidarloc`).
pub(crate) struct Map3d {
    pub(crate) cloud: PointCloudMap,
    pub(crate) points: Vec<Vec3>,
}

/// A fixed pseudo-random number in [0, 1) for `key` and `salt`.
fn unit_hash(key: u64, salt: u64) -> f64 {
    let mut h = key.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ salt.wrapping_mul(0xC2B2_AE3D_27D4_EB4F);
    h ^= h >> 31;
    h = h.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    h ^= h >> 29;
    (h >> 11) as f64 / (1_u64 << 53) as f64
}

/// A fixed standard normal draw for a point (Box-Muller on two hashes).
fn point_gaussian(p: Vec3, salt: u64) -> f64 {
    let key = ((p.x * 1000.0) as i64 as u64)
        ^ ((p.y * 1000.0) as i64 as u64).rotate_left(21)
        ^ ((p.z * 1000.0) as i64 as u64).rotate_left(42);
    let u1 = unit_hash(key, salt).max(1.0e-12);
    let u2 = unit_hash(key, salt ^ 0x5555);
    (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
}

/// An aged survey (`--aged-map`): where a map point ends up, if anywhere.
/// The survey's own error is 2 cm of noise per point and a slow warp of up
/// to about 8 cm across the area (a SLAM map's drift); since then a third
/// of the street furniture and a quarter of the trees and hedges have gone
/// or moved, so the map has none of them.
struct Aging;

impl Aging {
    const NOISE_M: f64 = 0.02;
    const FURNITURE_GONE: f64 = 0.35;
    const VEGETATION_GONE: f64 = 0.25;

    /// Whether the map still has an object (by the mesh's object id).
    fn keeps(object: u32, vegetation: bool) -> bool {
        let gone = if object >= 1_000_000 {
            Self::FURNITURE_GONE
        } else if vegetation {
            Self::VEGETATION_GONE
        } else {
            0.0
        };
        unit_hash(u64::from(object), 0xA6ED) >= gone
    }

    /// Where the survey put a point.
    fn surveyed(p: Vec3) -> Vec3 {
        let w =
            |x: f64, period: f64, phase: f64| (std::f64::consts::TAU * x / period + phase).sin();
        let warp = Vec3::new(
            0.05 * w(p.x, 180.0, 0.3) + 0.03 * w(p.y, 130.0, 1.1),
            0.05 * w(p.y, 170.0, 2.0) + 0.03 * w(p.x, 120.0, 0.7),
            0.02 * w(p.x + p.y, 150.0, 1.7),
        );
        let noise = Vec3::new(
            point_gaussian(p, 1),
            point_gaussian(p, 2),
            point_gaussian(p, 3),
        ) * Self::NOISE_M;
        p + warp + noise
    }
}

/// Cars parked at the kerb when the map was surveyed and gone since
/// (`--aged-map`): every 45-75 m along the route, 3.5 m to one side where
/// the street is open and built up, the box surfaces of a 4.4 x 1.8 x 1.5 m
/// car.
fn phantom_cars(city: &PlateauCity, route: &[(f64, f64)]) -> (Vec<Vec3>, Vec<Vec3>) {
    let (mut points, mut normals) = (Vec::new(), Vec::new());
    let (mut walked, mut next, mut k) = (0.0, 30.0, 0_u64);
    for leg in route.windows(2) {
        let (a, b) = (leg[0], leg[1]);
        let length = (b.0 - a.0).hypot(b.1 - a.1);
        if length < 1.0e-6 {
            continue;
        }
        let (dx, dy) = ((b.0 - a.0) / length, (b.1 - a.1) / length);
        while next < walked + length {
            let s = next - walked;
            let side = if unit_hash(k, 0xCA5) < 0.5 { 1.0 } else { -1.0 };
            let (cx, cy) = (
                a.0 + dx * s - dy * side * 3.5,
                a.1 + dy * s + dx * side * 3.5,
            );
            let ground = city.ground_m(cx, cy);
            // Parked where the street is built up: a facade within 20 m
            // across from the kerb.
            let built_up = [1.0, -1.0].iter().any(|s: &f64| {
                city.mesh
                    .raycast(
                        Vec3::new(cx, cy, ground + 1.0),
                        Vec3::new(-dy * s, dx * s, 0.0),
                        20.0,
                    )
                    .is_some()
            });
            let free = built_up
                && !city.mesh.contains(Vec3::new(cx, cy, ground + 0.8))
                && crate::nav::distance_to_route(route, cx, cy) > 2.6;
            if free {
                let along = Vec3::new(dx, dy, 0.0);
                let across = Vec3::new(-dy, dx, 0.0);
                let centre = Vec3::new(cx, cy, ground + 0.75);
                let half = [2.2, 0.9, 0.75];
                let axes = [along, across, Vec3::Z];
                for (f, axis) in axes.iter().enumerate() {
                    for sign in [-1.0, 1.0] {
                        let (u, v) = ((f + 1) % 3, (f + 2) % 3);
                        let (nu, nv) = (
                            (2.0 * half[u] / MAP_SPACING_M).ceil() as i32,
                            (2.0 * half[v] / MAP_SPACING_M).ceil() as i32,
                        );
                        for i in 0..=nu {
                            for j in 0..=nv {
                                let pu = -half[u] + 2.0 * half[u] * f64::from(i) / f64::from(nu);
                                let pv = -half[v] + 2.0 * half[v] * f64::from(j) / f64::from(nv);
                                points.push(
                                    centre + *axis * (sign * half[f]) + axes[u] * pu + axes[v] * pv,
                                );
                                normals.push(*axis * sign);
                            }
                        }
                    }
                }
            }
            k += 1;
            next += 45.0 + 30.0 * unit_hash(k, 0xD15);
        }
        walked += length;
    }
    (points, normals)
}

/// The prior map: the mesh's surfaces and the open terrain within
/// `corridor_m` of the route, up to `MAP_MAX_HEIGHT_M` above the ground,
/// sampled every `MAP_SPACING_M`, with exact normals, as edited by `edit`.
/// Prints its size.
pub(crate) fn build_map(
    city: &PlateauCity,
    route: &[(f64, f64)],
    corridor_m: f64,
    edit: MapEdit,
) -> Map3d {
    let started = std::time::Instant::now();
    let within = |s: f64, range: (f64, f64)| (range.0..=range.1).contains(&s);
    // Where a sample goes in the map, if anywhere.
    let place = |p: Vec3, facade: bool| {
        let (d, s) = crate::nav::station_on_route(route, p.x, p.y);
        if d > corridor_m || edit.gap.is_some_and(|g| within(s, g)) {
            return None;
        }
        match edit.shift {
            Some((range, (dx, dy))) if facade && within(s, range) => {
                Some(Vec3::new(p.x + dx, p.y + dy, p.z))
            }
            _ => Some(p),
        }
    };
    let (mut points, mut normals) = (Vec::new(), Vec::new());
    for triangle in city.mesh.triangles() {
        let vegetation =
            city.scenery.is_some() && triangle.material == rne_gnss::SurfaceMaterial::ABSORBER;
        if edit.aged && !Aging::keeps(triangle.object, vegetation) {
            continue;
        }
        let [a, b, c] = triangle.vertices;
        let normal = triangle.normal;
        let longest = (b - a).length().max((c - b).length()).max((a - c).length());
        let k = (longest / MAP_SPACING_M).ceil().max(1.0) as usize;
        for i in 0..=k {
            for j in 0..=(k - i) {
                let (u, v) = (i as f64 / k as f64, j as f64 / k as f64);
                let p = a + (b - a) * u + (c - a) * v;
                let height = p.z - city.ground_m(p.x, p.y);
                if height <= MAP_MAX_HEIGHT_M {
                    if let Some(q) = place(p, height > 0.5) {
                        points.push(if edit.aged { Aging::surveyed(q) } else { q });
                        normals.push(normal);
                    }
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
                let z = city.ground_m(x, y);
                if place(Vec3::new(x, y, z), false).is_some()
                    && !city.mesh.contains(Vec3::new(x, y, z + 0.3))
                {
                    {
                        let e = 0.5;
                        let gx = (city.ground_m(x + e, y) - city.ground_m(x - e, y)) / (2.0 * e);
                        let gy = (city.ground_m(x, y + e) - city.ground_m(x, y - e)) / (2.0 * e);
                        let p = Vec3::new(x, y, z);
                        points.push(if edit.aged { Aging::surveyed(p) } else { p });
                        normals.push(Vec3::new(-gx, -gy, 1.0));
                    }
                }
                x += MAP_SPACING_M;
            }
            y += MAP_SPACING_M;
        }
    }
    if edit.aged {
        let (cars, car_normals) = phantom_cars(city, route);
        for (p, n) in cars.into_iter().zip(car_normals) {
            if place(p, true).is_some() {
                points.push(Aging::surveyed(p));
                normals.push(n);
            }
        }
    }
    let map = PointCloudMap::new(&points, &normals, 1.0).expect("point-cloud map");
    println!(
        "  3D LiDAR map: {} points ({:.1} s)",
        map.len(),
        started.elapsed().as_secs_f64()
    );
    Map3d { cloud: map, points }
}

/// lidar_localization_ros2 on the map (`--lidarloc`), tracking as its node
/// does: NDT_OMP at 1 m on the whole map, one thread so that runs repeat
/// exactly, the scans already reduced by [`downsample`]; the node's
/// measurement gate; the map-distance prefilter at 1 m for the people, cars
/// and rain the map lacks; and local re-acquisition on an occupancy grid of
/// the map after the odometry carried the pose through an outage.
pub(crate) fn lidarloc(map: &Map3d) -> Result<Tracker, String> {
    let mut loc = LidarLoc::new(&LidarLocConfig {
        registration_method: "NDT_OMP".into(),
        ndt_num_threads: 1,
        filter_scan: false,
        inlier_ratio_min_points: 50,
        map_distance_filter_m: 1.0,
        ..LidarLocConfig::default()
    })
    .map_err(|e| e.to_string())?;
    // The guards the node's quickstart turns on with an odometry seed: a
    // scan may not move the pose more than 0.5 m from the seed, nor 0.3 m
    // in all over 2 s. Without them NDT slides along a tunnel, whose walls
    // pin nothing along it, and the estimate it seeds slides with it.
    loc.set_tracking(&Tracking {
        scan_period_sec: LIDAR3D_PERIOD_S,
        enable_seed_correction_guard: true,
        enable_seed_correction_window_guard: true,
        seed_correction_window_guard_sec: 2.0,
        // Solution remapping, by NDT's Hessian (along a tunnel) and by the
        // map's surfaces under the scan (bare ground): without it NDT and
        // the point-to-plane fit slid along the tunnel together.
        enable_degeneracy_remapping: true,
        degeneracy_remapping_min_horizontal_constraint: 0.02,
        ..Tracking::default()
    })
    .map_err(|e| e.to_string())?;
    loc.set_reacquisition(&Reacquisition {
        // Two seconds of rejected scans, then a try every second.
        min_rejections: 10,
        attempt_interval_scans: 5,
        // Street level: kerbs and parked cars are obstacles, not the
        // buildings' upper floors.
        scan_min_z_m: -1.0,
        scan_max_z_m: 3.0,
        occupancy_max_obstacle_height_m: 4.0,
        ..Reacquisition::default()
    })
    .map_err(|e| e.to_string())?;
    loc.set_map(&map.points).map_err(|e| e.to_string())?;
    Ok(Tracker {
        loc,
        started: false,
    })
}

/// lidar_localization_ros2's tracking, started at the localizer's estimate
/// on the first scan.
pub(crate) struct Tracker {
    loc: LidarLoc,
    started: bool,
}

/// Seconds between 3D LiDAR scans.
pub(crate) const LIDAR3D_PERIOD_S: f64 = 0.2;

/// One scan through lidar_localization_ros2's tracking, seeded with the
/// localizer's estimate (the odometry bridge). An accepted scan is a match
/// at the node's pose, with the fit and the covariance of the map's
/// point-to-plane fit started there ([`PointCloudMap::match_scan`]), so that
/// the hybrid weighs and gates it as it does its own matches. A scan the
/// node's gate rejects, or whose point-to-plane fit settles more than
/// [`LIDARLOC_AGREEMENT_M`] away, is none.
///
/// Not the NDT score Hessian's covariance: on bare ground NDT's cells pin
/// the scan to their lattice, and the Hessian claims 2 cm where nothing
/// pins the position at all.
pub(crate) fn lidarloc_match(
    tracker: &mut Tracker,
    map: &PointCloudMap,
    points: &[Vec3],
    pose: Pose2d,
    z_m: f64,
    t: f64,
) -> Option<PointCloudMatch> {
    let seed = Mat4::from_rotation_translation(
        DQuat::from_rotation_z(pose.yaw_rad),
        Vec3::new(pose.x_m, pose.y_m, z_m),
    );
    if !tracker.started {
        tracker.loc.set_initial_pose(&seed, t).ok()?;
        tracker.started = true;
    }
    let update = tracker.loc.localize(points, t, Some(&seed)).ok()?;
    if !update.accepted {
        return None;
    }
    let p = update.pose;
    let at = Pose2d::new(p.w_axis.x, p.w_axis.y, p.x_axis.y.atan2(p.x_axis.x));
    // The map's point-to-plane fit, started at the node's pose, must settle
    // there: where it moves away, NDT has stopped in a minimum of its own
    // (its cells pin a scan of open ground with a wall at the edge of range;
    // it slides along a tunnel, whose walls pin nothing along it), and the
    // scan is not used.
    let refined = map.match_scan(points, at, p.w_axis.z, &match_config())?;
    if (refined.pose.x_m - at.x_m).hypot(refined.pose.y_m - at.y_m) > LIDARLOC_AGREEMENT_M {
        return None;
    }
    Some(PointCloudMatch {
        pose: at,
        z_m: p.w_axis.z,
        ..refined
    })
}

/// lidar_localization_ros2's global localization while the localizer dead
/// reckons: where in the whole map the scan was taken, with no prior. A pose
/// found, and confirmed by the point-to-plane fit started there, restarts
/// the node's tracking there and is a match.
pub(crate) fn lidarloc_global(
    tracker: &mut Tracker,
    map: &PointCloudMap,
    points: &[Vec3],
    t: f64,
) -> Option<PointCloudMatch> {
    let fix = tracker
        .loc
        .global_localize(
            points,
            &GlobalSearch {
                // The street-level scan of a 3D LiDAR on a robot.
                scan_max_range_m: RANGE_M,
                num_threads: 1,
                ..GlobalSearch::default()
            },
        )
        .ok()?;
    println!(
        "  [{t:.0} s] lidarloc global search: {} ({} candidates, fitness {:.3}, second {:.3}, {:.0} ms)",
        fix.reason, fix.candidates, fix.fitness, fix.second_fitness, fix.total_ms
    );
    let p = fix.pose?;
    let at = Pose2d::new(p.w_axis.x, p.w_axis.y, p.x_axis.y.atan2(p.x_axis.x));
    let refined = map.match_scan(points, at, p.w_axis.z, &match_config())?;
    if (refined.pose.x_m - at.x_m).hypot(refined.pose.y_m - at.y_m) > LIDARLOC_AGREEMENT_M {
        return None;
    }
    let start = Mat4::from_rotation_translation(
        DQuat::from_rotation_z(refined.pose.yaw_rad),
        Vec3::new(refined.pose.x_m, refined.pose.y_m, refined.z_m),
    );
    tracker.loc.set_initial_pose(&start, t).ok()?;
    tracker.started = true;
    Some(refined)
}

/// How far, meters, the point-to-plane fit may settle from lidar_localization_ros2's
/// pose for the scan to be used.
const LIDARLOC_AGREEMENT_M: f64 = 0.3;

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

/// Position variance, m², above which a match pins no horizontal direction:
/// on open ground the scan holds only the ground, which fits anywhere.
const UNCONSTRAINED_VARIANCE_M2: f64 = 25.0;

/// Whether a match is good enough to use: most of the scan lies on mapped
/// surfaces (the RMS residual would also count people next to walls), and
/// something in it pins the position in at least one horizontal direction.
pub(crate) fn acceptable(m: &PointCloudMatch) -> bool {
    let [[xx, xy], [_, yy]] = m.position_covariance_m2;
    let smaller = 0.5 * (xx + yy) - (0.25 * (xx - yy).powi(2) + xy * xy).sqrt();
    m.fit_fraction >= 0.6 && smaller < UNCONSTRAINED_VARIANCE_M2
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
            // A match that slid out of the window is not an answer to this
            // search (it fits some other street).
            let inside =
                (m.pose.x_m - around.x_m).hypot(m.pose.y_m - around.y_m) <= window_m * 1.5 + 1.0;
            if inside && acceptable(&m) && best.is_none_or(|b| m.fit_fraction > b.fit_fraction) {
                best = Some(m);
            }
        }
    }
    best
}
