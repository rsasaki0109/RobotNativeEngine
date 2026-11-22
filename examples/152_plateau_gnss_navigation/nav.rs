//! A delivery robot walking real streets: route planning over the PLATEAU
//! footprint, the ray-traced GNSS / libgnss++ RTK pipeline of example 151,
//! a LiDAR prior map rasterized from the city model along the route, and
//! `rne_slam::HybridLocalizer` with the same safety logic.

use crate::gnss::{GnssOutput, RobotGnss, SolutionLabel, BODY_HEIGHT_M};
use crate::plateau::PlateauCity;
use rne_core::DeterministicRng;
use rne_gnss::{BodyPose, Constellation, GnssScene, ReceiverConfig, SignalPath, TraceConfig};
use rne_math::Vec3;
use rne_nav::{
    pure_pursuit_follow, FrameId, GridCoord, LaserScan2d, OccupancyGrid, Path2d, Pose2d,
    PurePursuitConfig, VelocityCommand2d,
};
use rne_slam::{
    AbsoluteHeadingFix, AbsolutePositionFix, AmclConfig, FactorGraphConfig, FactorGraphLocalizer,
    FixKind, HybridLocalizer, HybridLocalizerConfig, LocalizationSource,
};
use std::collections::BinaryHeap;
use std::f64::consts::TAU;

const DT_S: f64 = 0.05;
const LIDAR_EVERY_TICKS: usize = 2;
const LIDAR_HEIGHT_M: f64 = 0.3;
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
/// Give up after this much simulated time.
const TIMEOUT_S: f64 = 1_800.0;

/// Options for one run.
#[derive(Clone, Copy, Debug)]
pub(crate) struct RunOptions {
    /// Withhold satellites the city model predicts are hidden before RTK.
    pub map_aided_gnss: bool,
    /// Include QZSS.
    pub qzss: bool,
    /// Record frames for the 3D render.
    pub record: bool,
    /// Use the sliding-window factor graph instead of the hybrid arbiter.
    pub factor_graph: bool,
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

    fn update_gnss(
        &mut self,
        fix: Option<&AbsolutePositionFix>,
        heading: Option<&AbsoluteHeadingFix>,
        t: f64,
    ) {
        match self {
            Self::Hybrid(h) => {
                h.update_gnss(fix, t).expect("gnss");
                if let Some(heading) = heading {
                    h.update_gnss_heading(heading).expect("heading");
                }
            }
            Self::Graph(g) => {
                g.update_gnss(fix, t);
                if let Some(heading) = heading {
                    g.update_gnss_heading(heading);
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

fn distance_to_route(route: &[(f64, f64)], x: f64, y: f64) -> f64 {
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
    pub stopped_s: f64,
    pub clearance_m: f64,
    /// `(truth, estimate, source)` every 0.5 s, for drawing.
    pub track: Vec<(Pose2d, Pose2d, LocalizationSource)>,
    /// Frames for the 3D render (when recording).
    pub frames: Vec<FrameRecord>,
}

impl RunReport {
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

/// Walks the route once.
pub(crate) fn run(
    city: &PlateauCity,
    route: &[(f64, f64)],
    base_enu_m: Vec3,
    options: RunOptions,
) -> RunReport {
    let constellation = if options.qzss {
        Constellation::gps_galileo_qzss_nominal()
    } else {
        Constellation::gps_galileo_nominal()
    };
    let scene = GnssScene {
        mesh: Some(city.mesh.clone()),
        ..GnssScene::open_sky()
    };
    let receiver = receiver_config();
    let mut gnss = RobotGnss::new(
        &constellation,
        city.frame,
        receiver,
        SKY_EPOCH_S + 300.0,
        base_enu_m,
    );
    let start = Pose2d::new(
        route[0].0,
        route[0].1,
        (route[1].1 - route[0].1).atan2(route[1].0 - route[0].0),
    );
    // The terrain-following slice at LiDAR height along the route: what the
    // simulated LiDAR sees and the robot's prior map.
    let slice = build_prior_map(city, route);
    let mut hybrid = build_localizer(&slice, start, gnss.uses_libgnss(), options.factor_graph);
    let mut rng = DeterministicRng::new(0x5_1B_0A);
    let (mut truth, mut odom) = (start, start);
    let mut report = RunReport {
        clearance_m: f64::INFINITY,
        ..RunReport::default()
    };
    let mut leg = 0;
    let mut safety = Safety::default();
    let mut front_clearance = f64::INFINITY;
    let body = |p: Pose2d| BodyPose {
        x_m: p.x_m,
        y_m: p.y_m,
        z_m: city.ground_m(p.x_m, p.y_m) + BODY_HEIGHT_M,
        yaw_rad: p.yaw_rad,
    };
    let mut tick = 0_usize;
    let wall = std::time::Instant::now();
    let verbose = std::env::args().any(|a| a == "--verbose");
    let mut last_scan = Vec::new();
    let mut last_label = None;
    loop {
        let t = tick as f64 * DT_S;
        if t > TIMEOUT_S {
            break;
        }
        let command = follow_route(route, &mut leg, hybrid.pose());
        let Some(command) = command else {
            report.reached = true;
            break;
        };
        let command = safety.gate(command, hybrid.source(), front_clearance, &mut report);
        let previous = truth;
        truth = integrate(truth, command.linear_m_s, command.angular_rad_s, DT_S);
        report.distance_m += (truth.x_m - previous.x_m).hypot(truth.y_m - previous.y_m);
        let odom_v = command.linear_m_s * 1.01 + 0.01 * gaussian(&mut rng);
        let odom_w = command.angular_rad_s + 0.001 + 0.005 * gaussian(&mut rng);
        odom = integrate(odom, odom_v, odom_w, DT_S);
        hybrid.predict(DT_S, odom_v, odom_w);

        let outputs: Vec<GnssOutput> = gnss.step(
            &scene,
            options.map_aided_gnss.then_some(&scene),
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
            hybrid.update_gnss(output.fix.as_ref(), output.heading.as_ref(), t);
            match output.label {
                SolutionLabel::Fixed => report.fixed += 1,
                SolutionLabel::Float => report.float += 1,
                SolutionLabel::Single => report.single += 1,
                SolutionLabel::None => report.none += 1,
            }
        }
        if tick.is_multiple_of(LIDAR_EVERY_TICKS) {
            let scan = lidar_scan(&slice, truth, t, &mut rng);
            if options.record {
                last_scan = scan_points(&scan, truth);
            }
            front_clearance = front_range(&scan);
            report.clearance_m = report
                .clearance_m
                .min(scan.ranges_m.iter().copied().fold(f64::INFINITY, f64::min));
            hybrid.update_lidar(&scan, odom, t);
        }
        let estimate = hybrid.pose();
        report
            .errors
            .push((estimate.x_m - truth.x_m).hypot(estimate.y_m - truth.y_m));
        report.time_in[match hybrid.source() {
            LocalizationSource::Gnss => 0,
            LocalizationSource::Lidar => 1,
            LocalizationSource::DeadReckoning => 2,
        }] += DT_S;
        if tick.is_multiple_of(10) {
            report.track.push((truth, estimate, hybrid.source()));
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
fn build_localizer(
    slice: &OccupancyGrid,
    start: Pose2d,
    uses_libgnss: bool,
    factor_graph: bool,
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
    pose: Pose2d,
    t: f64,
    rng: &mut DeterministicRng,
) -> LaserScan2d {
    const STEP_M: f64 = 0.05;
    let ranges_m = (0..LIDAR_BEAMS)
        .map(|beam| {
            let angle = pose.yaw_rad + TAU * beam as f64 / LIDAR_BEAMS as f64;
            let noise = LIDAR_NOISE_M * gaussian(rng);
            let (dx, dy) = (angle.cos(), angle.sin());
            let mut r = STEP_M;
            while r <= LIDAR_RANGE_M {
                let p = Vec3::new(pose.x_m + r * dx, pose.y_m + r * dy, 0.0);
                match slice.world_to_grid(p) {
                    Some(coord) if slice.is_occupied(coord) => return r + noise,
                    Some(_) => {}
                    None => return f64::INFINITY,
                }
                r += STEP_M;
            }
            f64::INFINITY
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

/// Nearest return within ±35° of straight ahead.
fn front_range(scan: &LaserScan2d) -> f64 {
    scan.ranges_m
        .iter()
        .enumerate()
        .filter(|(i, r)| {
            let angle = scan.angle_min_rad + *i as f64 * scan.angle_increment_rad;
            r.is_finite() && angle.sin().atan2(angle.cos()).abs() < 35.0_f64.to_radians()
        })
        .map(|(_, r)| *r)
        .fold(f64::INFINITY, f64::min)
}

fn gaussian(rng: &mut DeterministicRng) -> f64 {
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
        let scan = LaserScan2d {
            time_s: 0.0,
            frame: FrameId::new("laser"),
            angle_min_rad: 0.0,
            angle_increment_rad: TAU / 360.0,
            range_min_m: 0.05,
            range_max_m: 40.0,
            ranges_m,
        };
        assert_eq!(front_range(&scan), 3.0);
        let points = scan_points(&scan, Pose2d::new(1.0, 2.0, 0.0));
        assert_eq!(points.len(), 2);
    }
}
