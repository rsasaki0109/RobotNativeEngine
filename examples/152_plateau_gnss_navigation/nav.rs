//! A delivery robot walking real streets: route planning over the PLATEAU
//! footprint, the ray-traced GNSS / libgnss++ RTK pipeline of example 151,
//! a LiDAR prior map rasterized from the city model along the route, and
//! `rne_slam::HybridLocalizer` with the same safety logic.

use crate::avoid::Avoider;
use crate::crowd::{pedestrian_range, Weather};
use crate::explain::{Explainer, Status};
use crate::gnss::{GnssOutput, RobotGnss, SolutionLabel, BODY_HEIGHT_M};
use crate::lidar3d;
use crate::plateau::PlateauCity;
use crate::traffic::{vehicle_range_planar, Movers, Street};
use rne_core::DeterministicRng;
use rne_gnss::{BodyPose, Constellation, GnssScene, ReceiverConfig, SignalPath, TraceConfig};
use rne_math::Vec3;
use rne_nav::{
    pure_pursuit_follow, FrameId, GridCoord, LaserScan2d, OccupancyGrid, Path2d, Pose2d,
    PurePursuitConfig, VelocityCommand2d,
};
use rne_slam::{
    AbsoluteHeadingFix, AbsolutePositionFix, AmclConfig, FactorGraphConfig, FactorGraphLocalizer,
    FixKind, GnssDecision, HybridLocalizer, HybridLocalizerConfig, LocalizationSource,
    PointCloudMap,
};
use std::collections::BinaryHeap;
use std::f64::consts::TAU;

const DT_S: f64 = 0.05;
const LIDAR_EVERY_TICKS: usize = 2;
pub(crate) const LIDAR_HEIGHT_M: f64 = 0.3;
const LIDAR_BEAMS: usize = 360;
const LIDAR_RANGE_M: f64 = 40.0;
const LIDAR_NOISE_M: f64 = 0.02;
const MAP_RESOLUTION_M: f64 = 0.1;
/// Prior-map cells farther than this from the route are left unknown.
const MAP_CORRIDOR_M: f64 = LIDAR_RANGE_M + 4.0;
/// Route planning grid and clearance.
const PLAN_CELL_M: f64 = 1.0;
const MAX_DEAD_RECKONING_S: f64 = 3.0;
const STOP_DISTANCE_M: f64 = 0.6;
/// Constellation time at the start of the run (sets the sky geometry).
pub(crate) const SKY_EPOCH_S: f64 = 20_000.0;
/// Give up after this much simulated time, or two seconds a metre of route
/// if that is longer.
const TIMEOUT_S: f64 = 1_800.0;
const TIMEOUT_S_PER_M: f64 = 2.0;

/// Options for one run.
#[derive(Clone, Copy, Debug)]
pub(crate) struct RunOptions {
    /// Withhold satellites the city model predicts are hidden before RTK.
    pub map_aided_gnss: bool,
    /// Include QZSS.
    pub qzss: bool,
    /// Include BeiDou too (with QZSS).
    pub beidou: bool,
    /// Record frames for the 3D render.
    pub record: bool,
    /// Use the sliding-window factor graph instead of the hybrid arbiter.
    pub factor_graph: bool,
    /// Pedestrians and rain.
    pub weather: Weather,
    /// Noise realization: 0 is the documented run; others reseed the
    /// receiver, the sensors and the crowd.
    pub seed: u64,
    /// Localize with a 16-ring 3D LiDAR matched against a point-cloud map of
    /// the mesh, instead of the planar slice.
    pub lidar3d: bool,
    /// Walk around people (`avoid.rs`) instead of stopping for them.
    pub avoid: bool,
    /// Also run a localizer that never sees GNSS (the same LiDAR and wheels)
    /// and record where it thinks the robot is: what GNSS adds, side by side.
    pub ghost: bool,
    /// Start the localizers 25 m from the truth (sigma 30 m), as if switched
    /// on somewhere unknown.
    pub unknown_start: bool,
    /// Match the 3D scans with lidar_localization_ros2's registration
    /// (`rne_adapter_lidarloc`) instead of `rne_slam::PointCloudMap`.
    pub lidarloc: bool,
}

/// The robot's localizer: the rule-based arbiter or the factor graph.
enum Localizer {
    Hybrid(Box<HybridLocalizer>),
    Graph(Box<FactorGraphLocalizer>),
}

impl Localizer {
    fn pose(&self) -> Pose2d {
        match self {
            Self::Hybrid(h) => h.pose(),
            Self::Graph(g) => g.pose(),
        }
    }

    fn source(&self) -> LocalizationSource {
        match self {
            Self::Hybrid(h) => h.source(),
            Self::Graph(g) => g.source(),
        }
    }

    fn predict(&mut self, dt_s: f64, v: f64, w: f64) {
        match self {
            Self::Hybrid(h) => h.predict(dt_s, v, w).expect("predict"),
            Self::Graph(g) => g.predict(dt_s, v, w),
        }
    }

    /// Feeds a GNSS epoch; the hybrid's verdict on it, if it judges.
    fn update_gnss(
        &mut self,
        fix: Option<&AbsolutePositionFix>,
        heading: Option<&AbsoluteHeadingFix>,
        t: f64,
    ) -> Option<GnssDecision> {
        match self {
            Self::Hybrid(h) => {
                let decision = h.update_gnss(fix, t).expect("gnss");
                if let Some(heading) = heading {
                    h.update_gnss_heading(heading).expect("heading");
                }
                Some(decision)
            }
            Self::Graph(g) => {
                g.update_gnss(fix, t);
                if let Some(heading) = heading {
                    g.update_gnss_heading(heading);
                }
                None
            }
        }
    }

    /// The LiDAR's state in words: the latest 3D match's fit, or the particle
    /// filter's mean score with the planar scan.
    fn lidar_words(&self, fit3d: Option<f64>, open_ground: bool) -> String {
        if open_ground {
            return "LiDAR: only open ground in reach, nothing to pin the position".to_string();
        }
        match (self, fit3d) {
            (_, Some(fit)) => format!(
                "LiDAR: {:.0} % of the 3D scan on mapped surfaces",
                100.0 * fit
            ),
            (Self::Hybrid(h), None) => h.last_amcl_update().map_or_else(
                || "LiDAR: not tracking (GNSS in charge)".to_string(),
                |u| format!("LiDAR: map match score {:.2}", u.mean_score),
            ),
            (Self::Graph(_), None) => "LiDAR: planar scan matched every 0.1 s".to_string(),
        }
    }

    /// Every [`lidar3d::LIDAR3D_PERIOD_S`], takes a 3D scan from the true pose
    /// (`sensed`: the pedestrians, rain and the pose), matches it against
    /// the point-cloud map and feeds the match (see [`Self::match_lidar3d`]).
    /// Returns the scan, reduced for matching, when one was taken.
    fn update_lidar3d(
        &mut self,
        map: &PointCloudMap,
        city: &PlateauCity,
        sensed: (&Movers, bool, Pose2d),
        t: f64,
        state: &mut Lidar3dState,
    ) -> Option<Vec<Vec3>> {
        if t - state.last_scan_s < lidar3d::LIDAR3D_PERIOD_S - 1.0e-6 {
            return None;
        }
        state.last_scan_s = t;
        let (movers, rain, truth) = sensed;
        let ground = city.ground_m(truth.x_m, truth.y_m);
        let points = lidar3d::scan(city, movers, rain, truth, ground, &mut state.rng);
        let points = lidar3d::downsample(&points);
        self.match_lidar3d(map, city, &points, t, &mut state.matching);
        Some(points)
    }

    /// Matches a scan against the point-cloud map and feeds the match: from
    /// the estimate while tracking or while GNSS is in charge, over the
    /// search window (once a second) while the hybrid is dead reckoning.
    fn match_lidar3d(
        &mut self,
        map: &PointCloudMap,
        city: &PlateauCity,
        points: &[Vec3],
        t: f64,
        state: &mut MatchState,
    ) {
        let MatchState {
            last_search_s,
            last_global_s,
            localized,
            last_fit,
            open_ground,
            lidarloc,
        } = state;
        let pose = self.pose();
        let z = city.ground_m(pose.x_m, pose.y_m) + lidar3d::SENSOR_HEIGHT_M;
        let tracked_by_lidarloc = lidarloc.is_some();
        if let Self::Hybrid(h) = self {
            let c = h.ekf().covariance();
            *localized |= (c[0][0] + c[1][1]).sqrt() < PLACED_SIGMA_M;
        }
        let dead_reckoning =
            matches!(self, Self::Hybrid(h) if h.source() == LocalizationSource::DeadReckoning);
        let mut global = None;
        if dead_reckoning && !*localized && t - *last_global_s >= GLOBAL_SEARCH_PERIOD_S {
            if let Some(loc) = lidarloc.as_mut() {
                *last_global_s = t;
                global = lidar3d::lidarloc_global(loc, map, points, t);
            }
        }
        let mut raw = || match lidarloc {
            Some(loc) => lidar3d::lidarloc_match(loc, map, points, pose, z, t),
            None => map.match_scan(points, pose, z, &lidar3d::match_config()),
        };
        let mut local = || {
            let m = raw();
            *open_ground = m
                .as_ref()
                .is_some_and(|m| m.fit_fraction >= 0.6 && !lidar3d::acceptable(m));
            m.filter(lidar3d::acceptable)
        };
        match self {
            Self::Hybrid(h) => match h.source() {
                // While GNSS is in charge a match that agrees with it takes
                // over (with `lidar_keeps_charge`; ignored otherwise).
                LocalizationSource::Lidar | LocalizationSource::Gnss => {
                    let m = local();
                    *last_fit = m.map(|m| m.fit_fraction);
                    h.update_lidar_match(m.as_ref(), t).expect("lidar");
                }
                // Until the robot has first been placed (an unknown start),
                // lidar_localization_ros2 searches the whole map every 5 s;
                // not later, since the suburb's rows of similar houses hold
                // other places that fit as well as the right one. Once
                // placed, it keeps tracking every scan from the odometry
                // bridge (and re-acquires locally on its own), and without a
                // match `rne_slam` searches the window once a second.
                LocalizationSource::DeadReckoning if tracked_by_lidarloc => {
                    let mut m = global;
                    if m.is_none() && *localized {
                        m = local();
                        if m.is_none() && t - *last_search_s >= 1.0 {
                            *last_search_s = t;
                            m = lidar3d::search(map, points, pose, z, h.lidar_search_window_m());
                        }
                    }
                    *last_fit = m.map(|m| m.fit_fraction);
                    h.update_lidar_match(m.as_ref(), t).expect("lidar");
                }
                LocalizationSource::DeadReckoning if t - *last_search_s >= 1.0 => {
                    *last_search_s = t;
                    local();
                    let found = lidar3d::search(map, points, pose, z, h.lidar_search_window_m());
                    *last_fit = found.map(|m| m.fit_fraction);
                    h.update_lidar_match(found.as_ref(), t).expect("lidar");
                }
                _ => {}
            },
            Self::Graph(g) => {
                let m = local();
                *last_fit = m.map(|m| m.fit_fraction);
                if let Some(m) = m {
                    g.update_lidar_match(&m, t);
                }
            }
        }
    }

    fn update_lidar(&mut self, scan: &LaserScan2d, odom: Pose2d, t: f64) {
        match self {
            Self::Hybrid(h) => {
                h.update_lidar(scan, odom, Pose2d::IDENTITY, t)
                    .expect("lidar");
            }
            Self::Graph(g) => g.update_lidar(scan, odom, Pose2d::IDENTITY, t),
        }
    }
}

/// The 3D LiDAR's noise stream and timing.
struct Lidar3dState {
    rng: DeterministicRng,
    last_scan_s: f64,
    matching: MatchState,
}

/// [`TIMEOUT_S`], or [`TIMEOUT_S_PER_M`] a metre of the route if longer.
fn timeout_s(route: &[(f64, f64)]) -> f64 {
    let route_m: f64 = route
        .windows(2)
        .map(|w| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1))
        .sum();
    TIMEOUT_S.max(TIMEOUT_S_PER_M * route_m)
}

/// Position sigma, meters, below which the localizer counts as placed.
const PLACED_SIGMA_M: f64 = 1.0;

/// Seconds between lidar_localization_ros2's global searches while the
/// localizer dead reckons with no match.
const GLOBAL_SEARCH_PERIOD_S: f64 = 5.0;

/// A localizer that never sees GNSS, on the same LiDAR and wheels
/// (`--ghost`): what the robot would know without it.
struct Ghost {
    localizer: Localizer,
    matching: MatchState,
}

impl Ghost {
    fn error_m(&self, truth: Pose2d) -> f64 {
        let p = self.localizer.pose();
        (p.x_m - truth.x_m).hypot(p.y_m - truth.y_m)
    }
}

/// A localizer's 3D matching state.
struct MatchState {
    last_search_s: f64,
    /// When lidar_localization_ros2 last searched the whole map.
    last_global_s: f64,
    /// Whether the localizer has been placed: its position sigma below
    /// [`PLACED_SIGMA_M`] once.
    localized: bool,
    /// Share of the latest scan on mapped surfaces, once one matched.
    last_fit: Option<f64>,
    /// Whether the latest scan fit the map but held nothing that pins the
    /// position (only open ground in reach).
    open_ground: bool,
    /// lidar_localization_ros2's tracking, with `--lidarloc`.
    lidarloc: Option<lidar3d::Tracker>,
}

impl MatchState {
    fn new(lidarloc: Option<lidar3d::Tracker>) -> Self {
        Self {
            last_search_s: f64::NEG_INFINITY,
            last_global_s: f64::NEG_INFINITY,
            localized: false,
            last_fit: None,
            open_ground: false,
            lidarloc,
        }
    }
}

impl Lidar3dState {
    fn new(seed: u64, lidarloc: Option<lidar3d::Tracker>) -> Self {
        Self {
            rng: DeterministicRng::new(seed),
            last_scan_s: f64::NEG_INFINITY,
            matching: MatchState::new(lidarloc),
        }
    }
}

/// One recorded instant for the 3D render.
#[derive(Clone, Debug)]
#[cfg_attr(not(feature = "capture"), allow(dead_code))]
pub(crate) struct FrameRecord {
    pub t: f64,
    pub truth: Pose2d,
    pub ground_m: f64,
    pub estimate: Pose2d,
    pub source: LocalizationSource,
    pub label: Option<SolutionLabel>,
    /// The rear antenna's latest epoch.
    pub epoch: Option<rne_gnss::GnssEpoch>,
    /// LiDAR returns in the map frame.
    pub scan: Vec<(f64, f64)>,
    /// Pedestrians and vehicles.
    pub movers: Movers,
    /// Whether it is raining.
    pub rain: bool,
    /// A 3D LiDAR scan in the map frame (with `--lidar3d`).
    pub scan3d: Vec<Vec3>,
    /// What the localizer did and why.
    pub status: Status,
    /// The estimate of the localizer without GNSS (`--ghost`).
    pub ghost: Option<Pose2d>,
}

/// A 3D scan for the render, from its own noise stream so recording does not
/// change the run (empty without `--lidar3d`).
fn record_scan3d(
    city: &PlateauCity,
    movers: &Movers,
    options: RunOptions,
    truth: Pose2d,
) -> Vec<Vec3> {
    if !options.lidar3d {
        return Vec::new();
    }
    let ground = city.ground_m(truth.x_m, truth.y_m);
    let mut rng = DeterministicRng::new(0x0005_CA3D);
    let (sin, cos) = truth.yaw_rad.sin_cos();
    let origin = Vec3::new(truth.x_m, truth.y_m, ground + lidar3d::SENSOR_HEIGHT_M);
    lidar3d::scan(city, movers, options.weather.rain, truth, ground, &mut rng)
        .into_iter()
        .map(|p| origin + Vec3::new(cos * p.x - sin * p.y, sin * p.x + cos * p.y, p.z))
        .collect()
}

/// Seconds between recorded frames.
pub(crate) const RECORD_PERIOD_S: f64 = 1.0;

/// Plans a walking route through the given ENU waypoints over open street
/// cells (nothing overhead below 3 m), preferring to keep clear of facades.
pub(crate) fn plan_route(city: &PlateauCity, waypoints: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let (mut x0, mut y0, mut x1, mut y1) = (
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    );
    for &(x, y) in waypoints {
        x0 = x0.min(x);
        y0 = y0.min(y);
        x1 = x1.max(x);
        y1 = y1.max(y);
    }
    let margin = 150.0;
    let (x0, y0) = (x0 - margin, y0 - margin);
    let w = ((x1 + margin - x0) / PLAN_CELL_M) as usize;
    let h = ((y1 + margin - y0) / PLAN_CELL_M) as usize;
    let center = |i: usize, j: usize| {
        (
            x0 + (i as f64 + 0.5) * PLAN_CELL_M,
            y0 + (j as f64 + 0.5) * PLAN_CELL_M,
        )
    };
    let blocked: Vec<bool> = (0..w * h)
        .map(|k| {
            let (x, y) = center(k % w, k / w);
            city.mesh
                .contains(Vec3::new(x, y, city.ground_m(x, y) + 1.0))
        })
        .collect();
    // Distance to the nearest blocked cell (two-pass chamfer).
    let mut clearance = vec![f64::INFINITY; w * h];
    for k in 0..w * h {
        if blocked[k] {
            clearance[k] = 0.0;
        }
    }
    let diag = std::f64::consts::SQRT_2;
    for j in 0..h {
        for i in 0..w {
            let k = j * w + i;
            let mut best = clearance[k];
            if i > 0 {
                best = best.min(clearance[k - 1] + 1.0);
            }
            if j > 0 {
                best = best.min(clearance[k - w] + 1.0);
                if i > 0 {
                    best = best.min(clearance[k - w - 1] + diag);
                }
                if i + 1 < w {
                    best = best.min(clearance[k - w + 1] + diag);
                }
            }
            clearance[k] = best;
        }
    }
    for j in (0..h).rev() {
        for i in (0..w).rev() {
            let k = j * w + i;
            let mut best = clearance[k];
            if i + 1 < w {
                best = best.min(clearance[k + 1] + 1.0);
            }
            if j + 1 < h {
                best = best.min(clearance[k + w] + 1.0);
                if i + 1 < w {
                    best = best.min(clearance[k + w + 1] + diag);
                }
                if i > 0 {
                    best = best.min(clearance[k + w - 1] + diag);
                }
            }
            clearance[k] = best;
        }
    }
    let cell_of = |(x, y): (f64, f64)| {
        let i = ((x - x0) / PLAN_CELL_M) as usize;
        let j = ((y - y0) / PLAN_CELL_M) as usize;
        // Snap to the nearest cell with 3 m clearance.
        let mut best = (usize::MAX, f64::INFINITY);
        for dj in -20_i64..=20 {
            for di in -20_i64..=20 {
                let (ii, jj) = (i as i64 + di, j as i64 + dj);
                if ii < 0 || jj < 0 || ii >= w as i64 || jj >= h as i64 {
                    continue;
                }
                let k = jj as usize * w + ii as usize;
                let d = ((di * di + dj * dj) as f64).sqrt();
                if clearance[k] >= 3.0 && d < best.1 {
                    best = (k, d);
                }
            }
        }
        best.0
    };
    let mut route: Vec<(f64, f64)> = Vec::new();
    for pair in waypoints.windows(2) {
        let (start, goal) = (cell_of(pair[0]), cell_of(pair[1]));
        let leg: Vec<(f64, f64)> = astar(w, h, &clearance, start, goal)
            .into_iter()
            .map(|k| center(k % w, k / w))
            .collect();
        for p in simplify(&leg, 0.4) {
            if route.last() != Some(&p) {
                route.push(p);
            }
        }
    }
    route
}

fn astar(w: usize, h: usize, clearance: &[f64], start: usize, goal: usize) -> Vec<usize> {
    #[derive(PartialEq)]
    struct Node(f64, usize);
    impl Eq for Node {}
    impl Ord for Node {
        fn cmp(&self, other: &Self) -> std::cmp::Ordering {
            other.0.total_cmp(&self.0).then(other.1.cmp(&self.1))
        }
    }
    impl PartialOrd for Node {
        fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
            Some(self.cmp(other))
        }
    }
    let goal_xy = ((goal % w) as f64, (goal / w) as f64);
    let heuristic = |k: usize| ((k % w) as f64 - goal_xy.0).hypot((k / w) as f64 - goal_xy.1);
    let mut cost = vec![f64::INFINITY; w * h];
    let mut parent = vec![usize::MAX; w * h];
    let mut open = BinaryHeap::new();
    cost[start] = 0.0;
    open.push(Node(heuristic(start), start));
    while let Some(Node(_, k)) = open.pop() {
        if k == goal {
            break;
        }
        let (i, j) = ((k % w) as i64, (k / w) as i64);
        for (di, dj) in [
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ] {
            let (ii, jj) = (i + di, j + dj);
            if ii < 0 || jj < 0 || ii >= w as i64 || jj >= h as i64 {
                continue;
            }
            let n = jj as usize * w + ii as usize;
            // Need 1.5 m to the nearest wall; prefer 4 m or more.
            if clearance[n] < 1.5 {
                continue;
            }
            let step = if di != 0 && dj != 0 {
                std::f64::consts::SQRT_2
            } else {
                1.0
            };
            let penalty = 1.0 + 3.0 * (4.0 - clearance[n]).max(0.0);
            let c = cost[k] + step * penalty;
            if c < cost[n] {
                cost[n] = c;
                parent[n] = k;
                open.push(Node(c + heuristic(n), n));
            }
        }
    }
    let mut path = vec![goal];
    let mut k = goal;
    while k != start {
        k = parent[k];
        assert!(k != usize::MAX, "no route between waypoints");
        path.push(k);
    }
    path.reverse();
    path
}

/// Ramer–Douglas–Peucker.
fn simplify(points: &[(f64, f64)], tolerance: f64) -> Vec<(f64, f64)> {
    if points.len() < 3 {
        return points.to_vec();
    }
    let (a, b) = (points[0], points[points.len() - 1]);
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let length = dx.hypot(dy).max(1.0e-9);
    let (index, distance) = points[1..points.len() - 1]
        .iter()
        .enumerate()
        .map(|(i, p)| (i + 1, ((p.0 - a.0) * dy - (p.1 - a.1) * dx).abs() / length))
        .fold((0, 0.0), |best, x| if x.1 > best.1 { x } else { best });
    if distance <= tolerance {
        return vec![a, b];
    }
    let mut left = simplify(&points[..=index], tolerance);
    let right = simplify(&points[index..], tolerance);
    left.pop();
    left.extend(right);
    left
}

/// Prior occupancy map at LiDAR height in a corridor along the route.
fn build_prior_map(city: &PlateauCity, route: &[(f64, f64)]) -> OccupancyGrid {
    let (mut x0, mut y0, mut x1, mut y1) = (
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    );
    for &(x, y) in route {
        x0 = x0.min(x - MAP_CORRIDOR_M);
        y0 = y0.min(y - MAP_CORRIDOR_M);
        x1 = x1.max(x + MAP_CORRIDOR_M);
        y1 = y1.max(y + MAP_CORRIDOR_M);
    }
    let width = ((x1 - x0) / MAP_RESOLUTION_M).ceil() as usize;
    let height = ((y1 - y0) / MAP_RESOLUTION_M).ceil() as usize;
    let mut grid =
        OccupancyGrid::new(width, height, MAP_RESOLUTION_M, Pose2d::new(x0, y0, 0.0)).expect("map");
    for j in 0..height {
        for i in 0..width {
            let coord = GridCoord {
                x: i as isize,
                y: j as isize,
            };
            let c = grid.grid_to_world(coord);
            if distance_to_route(route, c.x, c.y) > MAP_CORRIDOR_M {
                continue;
            }
            let z = city.ground_m(c.x, c.y) + LIDAR_HEIGHT_M;
            if city.mesh.contains(Vec3::new(c.x, c.y, z)) {
                grid.mark_occupied(coord);
            } else {
                grid.mark_free(coord);
            }
        }
    }
    grid
}

/// Distance from a point to the route and the arc length along the route
/// of the nearest point on it, meters.
pub(crate) fn station_on_route(route: &[(f64, f64)], x: f64, y: f64) -> (f64, f64) {
    let (mut best, mut station, mut walked) = (f64::INFINITY, 0.0, 0.0);
    for s in route.windows(2) {
        let (a, b) = (s[0], s[1]);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let l2 = (dx * dx + dy * dy).max(1.0e-12);
        let t = (((x - a.0) * dx + (y - a.1) * dy) / l2).clamp(0.0, 1.0);
        let d = (x - a.0 - t * dx).hypot(y - a.1 - t * dy);
        if d < best {
            best = d;
            station = walked + t * l2.sqrt();
        }
        walked += l2.sqrt();
    }
    (best, station)
}

pub(crate) fn distance_to_route(route: &[(f64, f64)], x: f64, y: f64) -> f64 {
    route
        .windows(2)
        .map(|s| {
            let (a, b) = (s[0], s[1]);
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let l2 = (dx * dx + dy * dy).max(1.0e-12);
            let t = (((x - a.0) * dx + (y - a.1) * dy) / l2).clamp(0.0, 1.0);
            (x - a.0 - t * dx).hypot(y - a.1 - t * dy)
        })
        .fold(f64::INFINITY, f64::min)
}

impl RunReport {
    /// Counts a GNSS solution by type.
    fn count(&mut self, label: SolutionLabel) {
        match label {
            SolutionLabel::Fixed => self.fixed += 1,
            SolutionLabel::Float => self.float += 1,
            SolutionLabel::Single => self.single += 1,
            SolutionLabel::None => self.none += 1,
        }
    }
}

/// The figures of a run compared across seeds.
#[derive(Clone, Copy, Debug)]
pub(crate) struct RunSummary {
    pub reached: bool,
    pub rmse_m: f64,
    pub max_m: f64,
    pub longest_dead_reckoning_s: f64,
}

/// What a run produced.
#[derive(Clone, Debug, Default)]
pub(crate) struct RunReport {
    pub reached: bool,
    pub duration_s: f64,
    pub distance_m: f64,
    pub errors: Vec<f64>,
    pub fixed: usize,
    pub float: usize,
    pub single: usize,
    pub none: usize,
    pub los: f64,
    pub nlos: f64,
    pub tracked: f64,
    pub epochs: usize,
    pub time_in: [f64; 3],
    pub longest_dead_reckoning_s: f64,
    pub safety_stops: usize,
    /// Time stopped for something within 0.6 m ahead (pedestrians), s.
    pub obstacle_wait_s: f64,
    pub stopped_s: f64,
    pub clearance_m: f64,
    /// Time with a person or a vehicle within 0.5 m of the robot's centre
    /// (closer than its 0.35 m footprint and a 0.15 m margin), s.
    pub close_s: f64,
    /// `(truth, estimate, source)` every 0.5 s, for drawing.
    pub track: Vec<(Pose2d, Pose2d, LocalizationSource)>,
    /// Errors of the localizer without GNSS every 0.5 s (`--ghost`), meters.
    pub ghost_errors: Vec<f64>,
    /// Frames for the 3D render (when recording).
    pub frames: Vec<FrameRecord>,
    /// The constellations the receiver tracked, e.g. "GPS+Galileo+QZSS".
    #[cfg_attr(not(feature = "capture"), allow(dead_code))]
    pub constellation: &'static str,
}

impl RunReport {
    pub(crate) fn summary(&self) -> RunSummary {
        let (rmse_m, max_m) = self.rmse_max();
        RunSummary {
            reached: self.reached,
            rmse_m,
            max_m,
            longest_dead_reckoning_s: self.longest_dead_reckoning_s,
        }
    }

    pub(crate) fn rmse_max(&self) -> (f64, f64) {
        let n = self.errors.len().max(1) as f64;
        let rmse = (self.errors.iter().map(|e| e * e).sum::<f64>() / n).sqrt();
        (rmse, self.errors.iter().copied().fold(0.0, f64::max))
    }

    fn record_sky(&mut self, epoch: &rne_gnss::GnssEpoch) {
        self.epochs += 1;
        self.tracked += epoch
            .observations
            .iter()
            .filter(|o| o.path().is_tracked())
            .count() as f64;
        self.los += (epoch.count(SignalPath::LineOfSight)
            + epoch.count(SignalPath::LineOfSightMultipath)) as f64;
        self.nlos +=
            (epoch.count(SignalPath::NonLineOfSight) + epoch.count(SignalPath::Diffracted)) as f64;
    }

    pub(crate) fn fix_shares(&self) -> [f64; 4] {
        let n = (self.fixed + self.float + self.single + self.none).max(1) as f64;
        [self.fixed, self.float, self.single, self.none].map(|c| 100.0 * c as f64 / n)
    }
}

/// Receivers for a city model: surfaces beyond 90 m are not traced, and
/// two-bounce paths combine only the 48 nearest facets.
pub(crate) fn receiver_config() -> ReceiverConfig {
    ReceiverConfig {
        trace: TraceConfig {
            max_interaction_distance_m: 90.0,
            max_multi_bounce_facets: 48,
            ..TraceConfig::default()
        },
        ..ReceiverConfig::default()
    }
}

/// The satellites a run's receiver tracks, and their name for the HUD.
fn constellation(options: &RunOptions) -> (Constellation, &'static str) {
    match (options.qzss, options.beidou) {
        (true, true) => (
            Constellation::gps_galileo_qzss_beidou_nominal(),
            "GPS+Galileo+QZSS+BeiDou",
        ),
        (true, false) => (
            Constellation::gps_galileo_qzss_nominal(),
            "GPS+Galileo+QZSS",
        ),
        (false, _) => (Constellation::gps_galileo_nominal(), "GPS+Galileo"),
    }
}

/// The ray-traced GNSS scene (the ground wet in rain), and the city model
/// used for NLOS prediction, which never has the pedestrians added.
fn gnss_scene(city: &PlateauCity, rain: bool) -> (GnssScene, GnssScene) {
    let mut scene = GnssScene {
        mesh: Some(city.mesh.clone()),
        ..GnssScene::open_sky()
    };
    if rain {
        if let Some(ground) = &mut scene.ground {
            ground.material = crate::crowd::wet(ground.material);
        }
    }
    let model = scene.clone();
    (scene, model)
}

/// Walks the route once.
pub(crate) fn run(
    city: &PlateauCity,
    route: &[(f64, f64)],
    base_enu_m: Vec3,
    options: RunOptions,
    cloud: Option<&lidar3d::Map3d>,
) -> RunReport {
    let (constellation, constellation_name) = constellation(&options);
    let (mut scene, model) = gnss_scene(city, options.weather.rain);
    let street = Street::new(route, options.weather, options.seed);
    // A large odd multiplier spreads small seeds over the whole range.
    let reseed = options.seed.wrapping_mul(0x9E37_79B9_7F4A_7C15);
    let mut gnss = robot_gnss(city, &constellation, base_enu_m, reseed);
    let start = Pose2d::new(
        route[0].0,
        route[0].1,
        (route[1].1 - route[0].1).atan2(route[1].0 - route[0].0),
    );
    // The terrain-following slice at LiDAR height along the route: what the
    // simulated LiDAR sees and the robot's prior map.
    let slice = build_prior_map(city, route);
    let (mut hybrid, mut ghost) = localizers(&slice, start, gnss.uses_libgnss(), &options, cloud);
    let mut rng = DeterministicRng::new(0x5_1B_0A ^ reseed);
    // The 3D LiDAR draws from its own stream (the rest of the noise is the
    // same as with the planar slice).
    let mut lidar3d_state = Lidar3dState::new(0x3D_11DA ^ reseed, lidarloc(cloud, &options));
    let (mut truth, mut odom) = (start, start);
    let mut report = RunReport {
        clearance_m: f64::INFINITY,
        constellation: constellation_name,
        ..RunReport::default()
    };
    let mut leg = 0;
    let mut drive = Driver::new(options.avoid);
    let mut explain = Explainer::new();
    let body = |p: Pose2d| BodyPose {
        x_m: p.x_m,
        y_m: p.y_m,
        z_m: city.ground_m(p.x_m, p.y_m) + BODY_HEIGHT_M,
        yaw_rad: p.yaw_rad,
    };
    let timeout_s = timeout_s(route);
    let mut tick = 0_usize;
    let wall = std::time::Instant::now();
    let verbose = std::env::args().any(|a| a == "--verbose");
    let mut last_scan = Vec::new();
    let mut last_label = None;
    loop {
        let t = tick as f64 * DT_S;
        if t > timeout_s {
            break;
        }
        let command = drive.command(
            route,
            &mut leg,
            (hybrid.pose(), hybrid.source()),
            &mut report,
        );
        let Some(command) = command else {
            report.reached = true;
            break;
        };
        let previous = truth;
        truth = integrate(truth, command.linear_m_s, command.angular_rad_s, DT_S);
        report.distance_m += (truth.x_m - previous.x_m).hypot(truth.y_m - previous.y_m);
        let odom_v = command.linear_m_s * 1.01 + 0.01 * gaussian(&mut rng);
        let odom_w = command.angular_rad_s + 0.001 + 0.005 * gaussian(&mut rng);
        odom = integrate(odom, odom_v, odom_w, DT_S);
        hybrid.predict(DT_S, odom_v, odom_w);
        ghost
            .iter_mut()
            .for_each(|g| g.localizer.predict(DT_S, odom_v, odom_w));

        let movers = street.at(t, (truth.x_m, truth.y_m), city);
        report.close_s +=
            DT_S * f64::from(u8::from(movers.nearest_m((truth.x_m, truth.y_m)) < 0.5));
        scene.boxes = movers.gnss_boxes(city);
        // NLOS prediction needs a pose: once the robot is dead reckoning
        // it may be lost, and predicting from a wrong pose would withhold
        // the very satellites that could relocalize it.
        let map_aid =
            options.map_aided_gnss && hybrid.source() != LocalizationSource::DeadReckoning;
        let outputs: Vec<GnssOutput> = gnss.step(
            &scene,
            map_aid.then_some(&model),
            &constellation,
            body(truth),
            body(hybrid.pose()),
            SKY_EPOCH_S + t,
        );
        if let Some(epoch) = gnss.rear_epoch() {
            if (epoch.time_s - (SKY_EPOCH_S + t)).abs() < 1.0e-9 {
                report.record_sky(epoch);
            }
        }
        for output in &outputs {
            last_label = Some(output.label);
            let decision = hybrid.update_gnss(output.fix.as_ref(), output.heading.as_ref(), t);
            // No solution, ever: it hands over to the LiDAR at once.
            ghost
                .iter_mut()
                .for_each(|g| _ = g.localizer.update_gnss(None, None, t));
            explain.gnss(output, decision, hybrid.pose());
            report.count(output.label);
        }
        if tick.is_multiple_of(LIDAR_EVERY_TICKS) {
            let scan = lidar_scan(&slice, &movers, options.weather.rain, truth, t, &mut rng);
            if options.record {
                last_scan = scan_points(&scan, truth);
            }
            drive.sense(&scan, hybrid.pose(), &slice, t);
            report.clearance_m = report
                .clearance_m
                .min(scan.ranges_m.iter().copied().fold(f64::INFINITY, f64::min));
            match cloud.filter(|_| options.lidar3d).map(|m| &m.cloud) {
                Some(map) => {
                    let sensed = (&movers, options.weather.rain, truth);
                    let points = hybrid.update_lidar3d(map, city, sensed, t, &mut lidar3d_state);
                    if let (Some(g), Some(points)) = (ghost.as_mut(), points) {
                        g.localizer
                            .match_lidar3d(map, city, &points, t, &mut g.matching);
                    }
                }
                None => hybrid.update_lidar(&scan, odom, t),
            }
        }
        explain.lidar(hybrid.lidar_words(
            lidar3d_state.matching.last_fit,
            lidar3d_state.matching.open_ground,
        ));
        explain.source(t, hybrid.source());
        let estimate = hybrid.pose();
        report
            .errors
            .push((estimate.x_m - truth.x_m).hypot(estimate.y_m - truth.y_m));
        report.time_in[source_index(hybrid.source())] += DT_S;
        if tick.is_multiple_of(10) {
            report.track.push((truth, estimate, hybrid.source()));
            report
                .ghost_errors
                .extend(ghost.as_ref().map(|g| g.error_m(truth)));
        }
        if options.record && t + 1.0e-9 >= report.frames.len() as f64 * RECORD_PERIOD_S {
            report.frames.push(FrameRecord {
                t,
                truth,
                ground_m: city.ground_m(truth.x_m, truth.y_m),
                estimate,
                source: hybrid.source(),
                label: last_label,
                epoch: gnss.rear_epoch().cloned(),
                scan: last_scan.clone(),
                movers: movers.clone(),
                rain: options.weather.rain,
                scan3d: record_scan3d(city, &movers, options, truth),
                ghost: ghost.as_ref().map(|g| g.localizer.pose()),
                status: explain.status(t),
            });
        }
        if verbose && tick.is_multiple_of(600) {
            log_progress(t, &report, &hybrid, truth, wall.elapsed().as_secs_f64());
        }
        tick += 1;
    }
    report.duration_s = tick as f64 * DT_S;
    report
}

/// The hybrid arbiter (degeneracy-aware LiDAR fusion) or the factor graph,
/// both with a 600-particle filter.
/// The receiver pair, its noise reseeded by `reseed`.
fn robot_gnss(
    city: &PlateauCity,
    constellation: &Constellation,
    base_enu_m: Vec3,
    reseed: u64,
) -> RobotGnss {
    let mut receiver = receiver_config();
    receiver.seed ^= reseed;
    receiver.satellite_error_seed ^= reseed;
    RobotGnss::new(
        constellation,
        city.frame,
        receiver,
        SKY_EPOCH_S + 300.0,
        base_enu_m,
    )
}

/// lidar_localization_ros2's registration on the map, with `--lidarloc`
/// (one per localizer).
fn lidarloc(cloud: Option<&lidar3d::Map3d>, options: &RunOptions) -> Option<lidar3d::Tracker> {
    cloud
        .filter(|_| options.lidar3d && options.lidarloc)
        .map(|map| lidar3d::lidarloc(map).unwrap_or_else(|e| panic!("--lidarloc: {e}")))
}

/// The run's localizer, and the one without GNSS when asked for
/// (`--ghost`, 3D LiDAR only), both from [`initial_estimate`].
fn localizers(
    slice: &OccupancyGrid,
    start: Pose2d,
    uses_libgnss: bool,
    options: &RunOptions,
    cloud: Option<&lidar3d::Map3d>,
) -> (Localizer, Option<Ghost>) {
    let initial = initial_estimate(start, options);
    let localizer = |factor_graph: bool| {
        build_localizer(slice, initial, uses_libgnss, factor_graph, options.lidar3d)
    };
    let ghost = (options.ghost && options.lidar3d).then(|| Ghost {
        localizer: localizer(false),
        matching: MatchState::new(lidarloc(cloud, options)),
    });
    (localizer(options.factor_graph), ghost)
}

/// Index of a source in [`RunReport::time_in`].
fn source_index(source: LocalizationSource) -> usize {
    match source {
        LocalizationSource::Gnss => 0,
        LocalizationSource::Lidar => 1,
        LocalizationSource::DeadReckoning => 2,
    }
}

/// Where the localizers start and their position sigma: the truth to a
/// decimetre, or 25 m off (in a direction set by the seed) with a 30 m sigma
/// and the heading within about 10 degrees, as from a magnetometer.
fn initial_estimate(start: Pose2d, options: &RunOptions) -> (Pose2d, f64) {
    if !options.unknown_start {
        return (start, 0.1);
    }
    let angle = 0.7 + options.seed as f64 * 2.3;
    (
        Pose2d::new(
            start.x_m + 25.0 * angle.cos(),
            start.y_m + 25.0 * angle.sin(),
            start.yaw_rad + 0.15,
        ),
        30.0,
    )
}

fn build_localizer(
    slice: &OccupancyGrid,
    (start, start_sigma_m): (Pose2d, f64),
    uses_libgnss: bool,
    factor_graph: bool,
    lidar3d: bool,
) -> Localizer {
    let mut config = HybridLocalizerConfig {
        amcl: AmclConfig {
            particle_count: 600,
            ..AmclConfig::default()
        },
        ..HybridLocalizerConfig::default()
    };
    if uses_libgnss {
        config.enter_gnss_min_kind = FixKind::Fixed;
        config.min_float_variance_m2 = 4.0;
    }
    // PLATEAU LOD2 streets are featureless corridors at LiDAR height: fuse
    // scan matches with their degeneracy-aware covariance, preferring the
    // particle filter's match (no fixed gate, so it can correct the EKF).
    config.lidar_anisotropic = true;
    // Pedestrians are not in the map: keep them out of the scan matches.
    config.lidar_drop_unmapped_returns = true;
    config.gnss_reanchor_fixes = 15;
    config.lidar_gate_chi_square = 16.0;
    // The 3D LiDAR holds the robot wherever the map reaches: it keeps
    // charge, and RTK fixes correct it instead of taking over.
    config.lidar_keeps_charge = lidar3d;
    // RTK shows the map wrong only where the matches pulled the estimate off
    // the wheels; among Nishi-Shinjuku's towers RTK fixes its ambiguities
    // wrongly and jumps 3.5 m while the estimate follows the wheels.
    config.gnss_reanchor_min_departure_ratio = 0.5;
    if start_sigma_m > 1.0 {
        config.ekf.initial_position_variance_m2 = start_sigma_m * start_sigma_m;
        config.ekf.initial_yaw_variance_rad2 = 0.04;
    }
    if factor_graph {
        Localizer::Graph(Box::new(
            FactorGraphLocalizer::new(
                slice,
                start,
                FactorGraphConfig {
                    amcl: AmclConfig {
                        particle_count: 600,
                        ..AmclConfig::default()
                    },
                    ..FactorGraphConfig::default()
                },
            )
            .expect("graph"),
        ))
    } else {
        Localizer::Hybrid(Box::new(
            HybridLocalizer::new(slice.clone(), start, config).expect("localizer"),
        ))
    }
}

/// Pure pursuit along the route one leg at a time (it doubles back on
/// itself), turning in place when the next leg points away; `None` at the
/// goal.
fn follow_route(
    route: &[(f64, f64)],
    leg: &mut usize,
    estimate: Pose2d,
) -> Option<VelocityCommand2d> {
    let follow = PurePursuitConfig {
        lookahead_m: 1.2,
        max_linear_m_s: 1.2,
        max_angular_rad_s: 1.0,
        goal_tolerance_m: 0.6,
        slow_radius_m: 0.0,
    };
    while *leg + 1 < route.len() {
        let (a, b) = (route[*leg], route[*leg + 1]);
        let last = *leg + 2 == route.len();
        let config = PurePursuitConfig {
            slow_radius_m: if last { 1.5 } else { 0.0 },
            goal_tolerance_m: if last { 0.4 } else { follow.goal_tolerance_m },
            ..follow
        };
        let path = Path2d::from_points(&[Vec3::new(a.0, a.1, 0.0), Vec3::new(b.0, b.1, 0.0)]);
        let result = pure_pursuit_follow(&path, estimate, &config).expect("follow");
        if result.reached {
            *leg += 1;
            continue;
        }
        let heading = (b.1 - estimate.y_m).atan2(b.0 - estimate.x_m);
        let error = rne_nav::wrap_angle(heading - estimate.yaw_rad);
        return Some(if error.abs() > 0.9 {
            VelocityCommand2d::new(0.0, config.max_angular_rad_s * error.signum())
        } else {
            result.command
        });
    }
    None
}

/// The route follower, the optional avoidance and the safety gate.
struct Driver {
    safety: Safety,
    front_clearance_m: f64,
    avoider: Option<Avoider>,
}

impl Driver {
    fn new(avoid: bool) -> Self {
        Self {
            safety: Safety::default(),
            front_clearance_m: f64::INFINITY,
            avoider: avoid.then(Avoider::new),
        }
    }

    /// Updates the clearance ahead and the obstacles from a scan.
    fn sense(&mut self, scan: &LaserScan2d, estimate: Pose2d, map: &OccupancyGrid, t: f64) {
        self.front_clearance_m = front_range(scan);
        if let Some(avoider) = &mut self.avoider {
            avoider.sense(scan, estimate, map, t);
        }
    }

    /// The command for this tick (`None` at the goal).
    fn command(
        &mut self,
        route: &[(f64, f64)],
        leg: &mut usize,
        (estimate, source): (Pose2d, LocalizationSource),
        report: &mut RunReport,
    ) -> Option<VelocityCommand2d> {
        let mut command = follow_route(route, leg, estimate)?;
        if let Some(avoider) = &self.avoider {
            command = avoider.command(command, estimate);
        }
        Some(
            self.safety
                .gate(command, source, self.front_clearance_m, report),
        )
    }
}

/// Stops after 3 s of dead reckoning (until GNSS or LiDAR localizes again)
/// and stops forward motion before anything within 0.6 m ahead.
#[derive(Debug, Default)]
struct Safety {
    dead_reckoning_s: f64,
    stopped: bool,
}

impl Safety {
    fn gate(
        &mut self,
        mut command: VelocityCommand2d,
        source: LocalizationSource,
        front_clearance_m: f64,
        report: &mut RunReport,
    ) -> VelocityCommand2d {
        if source == LocalizationSource::DeadReckoning {
            self.dead_reckoning_s += DT_S;
        } else {
            self.dead_reckoning_s = 0.0;
        }
        report.longest_dead_reckoning_s =
            report.longest_dead_reckoning_s.max(self.dead_reckoning_s);
        if self.dead_reckoning_s > MAX_DEAD_RECKONING_S {
            if !self.stopped {
                report.safety_stops += 1;
            }
            self.stopped = true;
            report.stopped_s += DT_S;
            command = VelocityCommand2d::ZERO;
        } else {
            self.stopped = false;
        }
        if command.linear_m_s > 0.0 && front_clearance_m < STOP_DISTANCE_M {
            command.linear_m_s = 0.0;
            report.obstacle_wait_s += DT_S;
        }
        command
    }
}

fn log_progress(t: f64, report: &RunReport, hybrid: &Localizer, truth: Pose2d, wall_s: f64) {
    eprintln!(
        "    t {t:5.0} s truth ({:7.1}, {:7.1}) error {:5.2} m {:?}; RTK fixed/float/single/none {}/{}/{}/{} [{wall_s:.0} s]",
        truth.x_m,
        truth.y_m,
        report.errors.last().copied().unwrap_or(0.0),
        hybrid.source(),
        report.fixed,
        report.float,
        report.single,
        report.none,
    );
}

fn integrate(pose: Pose2d, v: f64, w: f64, dt: f64) -> Pose2d {
    let yaw = pose.yaw_rad + 0.5 * w * dt;
    Pose2d::new(
        pose.x_m + v * dt * yaw.cos(),
        pose.y_m + v * dt * yaw.sin(),
        rne_nav::wrap_angle(pose.yaw_rad + w * dt),
    )
}

/// A 3D LiDAR reduced to a planar scan the usual way (ground removed, the
/// nearest return in a band about 0.3 m above the local terrain per
/// azimuth): beams march through the terrain-following slice of the city
/// that the prior map also rasterizes. On a slope this keeps uphill
/// buildings in view, as a 3D sensor would.
fn lidar_scan(
    slice: &OccupancyGrid,
    movers: &Movers,
    rain: bool,
    pose: Pose2d,
    t: f64,
    rng: &mut DeterministicRng,
) -> LaserScan2d {
    const STEP_M: f64 = 0.05;
    let ranges_m = (0..LIDAR_BEAMS)
        .map(|beam| {
            let angle = pose.yaw_rad + TAU * beam as f64 / LIDAR_BEAMS as f64;
            // A fixed number of draws per beam keeps runs comparable.
            let (n, drop, rain_hit, rain_range) = (
                gaussian(rng),
                rng.uniform_f64(0.0, 1.0),
                rng.uniform_f64(0.0, 1.0),
                rng.uniform_f64(0.3, 4.0),
            );
            let (dx, dy) = (angle.cos(), angle.sin());
            let mut r = STEP_M;
            let mut range = f64::INFINITY;
            while r <= LIDAR_RANGE_M {
                let p = Vec3::new(pose.x_m + r * dx, pose.y_m + r * dy, 0.0);
                match slice.world_to_grid(p) {
                    Some(coord) if slice.is_occupied(coord) => {
                        range = r;
                        break;
                    }
                    Some(_) => {}
                    None => break,
                }
                r += STEP_M;
            }
            if let Some(person) = pedestrian_range(&movers.people, pose.x_m, pose.y_m, dx, dy) {
                range = range.min(person);
            }
            let at = (pose.x_m, pose.y_m);
            if let Some(car) = vehicle_range_planar(&movers.vehicles, at, (dx, dy), LIDAR_HEIGHT_M)
            {
                range = range.min(car);
            }
            if rain {
                // Rain: wet, scattered returns drop out, ranges get noisier,
                // and drops close to the sensor return the beam early.
                if drop < 0.15 {
                    return f64::INFINITY;
                }
                if rain_hit < 0.02 && rain_range < range {
                    return rain_range;
                }
                return range + 3.0 * LIDAR_NOISE_M * n;
            }
            range + LIDAR_NOISE_M * n
        })
        .collect();
    LaserScan2d {
        time_s: t,
        frame: FrameId::new("laser"),
        angle_min_rad: 0.0,
        angle_increment_rad: TAU / LIDAR_BEAMS as f64,
        range_min_m: 0.05,
        range_max_m: LIDAR_RANGE_M,
        ranges_m,
    }
}

/// LiDAR returns in the map frame.
fn scan_points(scan: &LaserScan2d, pose: Pose2d) -> Vec<(f64, f64)> {
    scan.ranges_m
        .iter()
        .enumerate()
        .filter(|(_, r)| r.is_finite())
        .map(|(i, r)| {
            let angle = pose.yaw_rad + scan.angle_min_rad + i as f64 * scan.angle_increment_rad;
            (pose.x_m + r * angle.cos(), pose.y_m + r * angle.sin())
        })
        .collect()
}

/// The third-nearest return within ±35° of straight ahead (an obstacle hits
/// several beams; a raindrop one).
fn front_range(scan: &LaserScan2d) -> f64 {
    let mut ahead: Vec<f64> = scan
        .ranges_m
        .iter()
        .enumerate()
        .filter(|(i, r)| {
            let angle = scan.angle_min_rad + *i as f64 * scan.angle_increment_rad;
            r.is_finite() && angle.sin().atan2(angle.cos()).abs() < 35.0_f64.to_radians()
        })
        .map(|(_, r)| *r)
        .collect();
    ahead.sort_by(f64::total_cmp);
    ahead.get(2).copied().unwrap_or(f64::INFINITY)
}

pub(crate) fn gaussian(rng: &mut DeterministicRng) -> f64 {
    let u1 = rng.uniform_f64(1.0e-12, 1.0);
    let u2 = rng.uniform_f64(0.0, 1.0);
    (-2.0 * u1.ln()).sqrt() * (TAU * u2).cos()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn astar_goes_around_a_wall_and_keeps_clear_of_it() {
        // A 30 x 20 grid with a wall at x = 15 open only near the top.
        let (w, h) = (30, 20);
        let mut clearance = vec![f64::INFINITY; w * h];
        for j in 0..h {
            for i in 0..w {
                let blocked = i == 15 && j < 16;
                let d = if blocked {
                    0.0
                } else {
                    ((i as f64 - 15.0).abs()).min(if j < 16 { f64::INFINITY } else { 99.0 })
                };
                clearance[j * w + i] = d.max(if blocked { 0.0 } else { 1.6 });
            }
        }
        let path = astar(w, h, &clearance, 2 * w + 2, 2 * w + 27);
        assert_eq!(*path.first().unwrap(), 2 * w + 2);
        assert_eq!(*path.last().unwrap(), 2 * w + 27);
        assert!(path.iter().all(|&k| clearance[k] >= 1.5));
        // It had to pass the wall's opening.
        assert!(path.iter().any(|&k| k % w == 15 && k / w >= 16));
    }

    #[test]
    fn simplify_keeps_corners_and_drops_collinear_points() {
        let points: Vec<(f64, f64)> = (0..=10)
            .map(|i| (f64::from(i), 0.0))
            .chain((1..=10).map(|j| (10.0, f64::from(j))))
            .collect();
        assert_eq!(
            simplify(&points, 0.1),
            vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]
        );
    }

    #[test]
    fn front_range_only_looks_ahead() {
        let mut ranges_m = vec![f64::INFINITY; 360];
        ranges_m[180] = 0.5; // behind
        ranges_m[20] = 3.0; // 20° to the left
        ranges_m[21] = 3.1;
        ranges_m[22] = 3.2;
        let scan = LaserScan2d {
            time_s: 0.0,
            frame: FrameId::new("laser"),
            angle_min_rad: 0.0,
            angle_increment_rad: TAU / 360.0,
            range_min_m: 0.05,
            range_max_m: 40.0,
            ranges_m,
        };
        assert_eq!(front_range(&scan), 3.2);
        let points = scan_points(&scan, Pose2d::new(1.0, 2.0, 0.0));
        assert_eq!(points.len(), 4);
    }

    #[test]
    fn a_pedestrian_ahead_stops_the_robot_and_rain_degrades_the_scan() {
        // An empty 20 x 20 m slice; one pedestrian 1.5 m ahead.
        let slice = OccupancyGrid::new(200, 200, 0.1, Pose2d::new(-10.0, -10.0, 0.0)).expect("map");
        let pose = Pose2d::new(0.0, 0.0, 0.0);
        let mut rng = DeterministicRng::new(3);
        let person = Movers {
            people: vec![(1.5, 0.0)],
            vehicles: Vec::new(),
        };
        let scan = lidar_scan(&slice, &person, false, pose, 0.0, &mut rng);
        let ahead = front_range(&scan);
        assert!((ahead - 1.25).abs() < 0.1, "{ahead}");
        // Rain: some beams drop out, a few return from drops close by.
        let dry = scan.ranges_m.iter().filter(|r| r.is_finite()).count();
        let mut finite = 0;
        for _ in 0..20 {
            let wet = lidar_scan(&slice, &person, true, pose, 0.0, &mut rng);
            finite += wet.ranges_m.iter().filter(|r| r.is_finite()).count();
        }
        assert!((15..30).contains(&dry), "only the pedestrian: {dry}");
        assert!(
            finite > 20 * dry,
            "drops return beams that would see nothing: {finite}"
        );
    }
}
