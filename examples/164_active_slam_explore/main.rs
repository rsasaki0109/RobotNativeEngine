//! Active SLAM: a robot with no map and no goals explores the warehouse of
//! example 127 by itself, choosing where to go with `rne_explore`.
//!
//! The robot has a 4 m 2D LiDAR (sampled from the Rapier scene) and wheel
//! odometry that drifts (2 % scale, a yaw bias). It runs `Slam2d` on
//! keyframes and steers on the SLAM estimate. About once a second
//! `rne_explore::Explorer` picks a frontier of the SLAM map, `rne_nav` plans a
//! path to it through mapped space, and the robot follows it. Exploration ends
//! when no reachable frontier is left.
//!
//! The same drive runs twice, with the two policies of `rne_explore`:
//!
//! - nearest frontier (Yamauchi 1997), the baseline;
//! - information gain: unknown area visible from the goal against the path
//!   length to it.
//!
//! Ground truth is used only to score: the area mapped against the true free
//! area of the warehouse, the SLAM trajectory against the true one, and
//! collisions against the true footprints. `docs/PLAN_ACTIVE_SLAM.md` P1.
//!
//! ```text
//! cargo run --release -p active_slam_explore --example 164_active_slam_explore -- --smoke
//! cargo run --release -p active_slam_explore --example 164_active_slam_explore
//! ```
//!
//! Without `--smoke` it also writes `docs/media/active-slam-explore.gif`:
//! both policies side by side, top-down.

mod render;
mod warehouse;

use rne_core::SimDuration;
use rne_data::PointCloud;
use rne_explore::{
    extract_frontiers, Decision, Explorer, ExplorerConfig, FrontierConfig, ScorePolicy, SensorModel,
};
use rne_math::{Hertz, Quat, Vec3};
use rne_nav::{CostmapConfig, GridCoord, LaserScan2d, OccupancyGrid, Path2d, Pose2d};
use rne_physics::{PhysicsBackend, PhysicsWorldDesc, PhysicsWorldId};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_sensor::{sample_lidar, LidarSpec};
use rne_slam::{Slam2d, SlamConfig};
use rne_world::Transform3;
use std::path::PathBuf;

const SCAN_HZ: f64 = 10.0;
const LIDAR_RANGE_M: f64 = 4.0;
const LIDAR_HEIGHT_M: f64 = 0.5;
const ROBOT_RADIUS_M: f64 = 0.25;
const MAX_SPEED_M_S: f64 = 0.6;
const MAX_TURN_RAD_S: f64 = 1.0;
const LOOKAHEAD_M: f64 = 0.5;
/// Clearance kept from mapped obstacles beyond the robot's radius.
const SAFETY_MARGIN_M: f64 = 0.3;
/// After a bump the robot backs off for this long.
const BACKOFF_S: f64 = 1.0;
const BACKOFF_M_S: f64 = 0.2;
const GOAL_TOLERANCE_M: f64 = 0.3;
/// A keyframe is taken after this much odometry travel or turn.
const KEYFRAME_M: f64 = 0.3;
const KEYFRAME_RAD: f64 = 0.3;
/// The explorer is asked again this often while a goal is being driven to.
const REPLAN_S: f64 = 1.0;
/// Exploration gives up after this long.
const MAX_TIME_S: f64 = 600.0;
/// After this many goals in a row that cannot be planned to, give up.
const MAX_PLAN_FAILURES: usize = 20;
const RESOLUTION_M: f64 = 0.05;
const GRID_M: f64 = 32.0;
/// A picture is taken this often (simulation steps).
const FRAME_EVERY_STEPS: usize = 15;

/// Where the robot starts. The first, the south-west aisle facing east, is
/// the one in the GIF and the smoke test; the others are the other aisles.
const STARTS: [Pose2d; 4] = [
    Pose2d {
        x_m: -5.0,
        y_m: -2.0,
        yaw_rad: 0.0,
    },
    Pose2d {
        x_m: 5.0,
        y_m: -2.0,
        yaw_rad: std::f64::consts::PI,
    },
    Pose2d {
        x_m: -5.0,
        y_m: 1.9,
        yaw_rad: 0.0,
    },
    Pose2d {
        x_m: 3.0,
        y_m: 1.5,
        yaw_rad: std::f64::consts::FRAC_PI_2,
    },
];

/// One exploration run, scored against ground truth.
pub(crate) struct Run {
    pub(crate) name: &'static str,
    pub(crate) time_s: f64,
    pub(crate) distance_m: f64,
    pub(crate) coverage: f64,
    /// Time and distance at which 90 % of the free area was mapped.
    pub(crate) time_to_90_s: Option<f64>,
    pub(crate) distance_to_90_m: Option<f64>,
    pub(crate) goals: usize,
    pub(crate) plan_failures: usize,
    pub(crate) collisions: usize,
    pub(crate) loop_closures: usize,
    /// Mean keyframe position error of SLAM against truth, meters.
    pub(crate) trajectory_error_m: f64,
    /// Final odometry position error against truth, meters.
    pub(crate) odometry_error_m: f64,
    pub(crate) done: bool,
    /// A stable hash of the final state, for the replay check.
    pub(crate) hash: u64,
    pub(crate) frames: Vec<render::Snapshot>,
}

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let policies = [
        ("NEAREST FRONTIER", ScorePolicy::NearestFrontier),
        ("INFORMATION GAIN", gain_policy()),
    ];
    let starts = if smoke { &STARTS[..1] } else { &STARTS[..] };
    let mut runs: Vec<Vec<Run>> = Vec::new();
    for (index, start) in starts.iter().enumerate() {
        // Only the first start is drawn.
        let render = !smoke && index == 0;
        let pair: Vec<Run> = policies
            .iter()
            .map(|(name, policy)| explore(name, *policy, *start, render))
            .collect();
        println!(
            "start {index}: ({:.1}, {:.1}) facing {:.0} deg",
            start.x_m,
            start.y_m,
            start.yaw_rad.to_degrees()
        );
        report(&pair.iter().collect::<Vec<_>>());
        runs.push(pair);
    }
    if runs.len() > 1 {
        summary(&runs);
    }
    for run in runs.iter().flatten() {
        check(run);
    }
    if smoke {
        // Replay: the same run gives the same hash.
        let again = explore(policies[1].0, policies[1].1, STARTS[0], false);
        assert_eq!(again.hash, runs[0][1].hash, "replay diverged");
        println!("replay hash {:016x}", again.hash);
        println!("smoke ok");
        return;
    }
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let gif = root.join("docs/media/active-slam-explore.gif");
    render::render_gif(
        &runs[0].iter().collect::<Vec<_>>(),
        STARTS[0],
        &root.join("target/rne-active-slam-frames"),
        &gif,
    )
    .expect("render the GIF");
    println!("wrote {}", gif.display());
}

/// Means over the starts, per policy.
fn summary(runs: &[Vec<Run>]) {
    println!("mean over {} starts:", runs.len());
    for policy in 0..runs[0].len() {
        let column: Vec<&Run> = runs.iter().map(|pair| &pair[policy]).collect();
        let n = column.len() as f64;
        let mean = |f: &dyn Fn(&Run) -> f64| column.iter().map(|r| f(r)).sum::<f64>() / n;
        println!(
            "{:<18} time {:>5.0}s  distance {:>5.1}m  mapped {:>5.1}%  90% at {:>5.1}m  SLAM err {:.3}m",
            column[0].name,
            mean(&|r| r.time_s),
            mean(&|r| r.distance_m),
            100.0 * mean(&|r| r.coverage),
            mean(&|r| r.distance_to_90_m.unwrap_or(f64::NAN)),
            mean(&|r| r.trajectory_error_m),
        );
    }
}

/// The information-gain policy, with `--distance-weight W` (square metres of
/// new area one metre of driving must buy; default 4).
fn gain_policy() -> ScorePolicy {
    let args: Vec<String> = std::env::args().collect();
    let distance_weight = args
        .iter()
        .position(|a| a == "--distance-weight")
        .and_then(|i| args.get(i + 1))
        .map_or(4.0, |w| {
            w.parse().expect("--distance-weight takes a number")
        });
    ScorePolicy::InformationGain { distance_weight }
}

fn explorer_config(policy: ScorePolicy) -> ExplorerConfig {
    ExplorerConfig {
        frontier: FrontierConfig {
            min_cluster_cells: 8,
        },
        sensor: SensorModel {
            range_m: LIDAR_RANGE_M,
            fov_rad: std::f64::consts::TAU,
            rays: 180,
        },
        costmap: costmap_config(),
        policy,
        min_gain_m2: 0.3,
        switch_margin: 0.5,
        hysteresis_radius_m: 1.0,
        escape_radius_m: 0.3,
    }
}

fn costmap_config() -> CostmapConfig {
    CostmapConfig {
        // A margin of 0.2 m over the robot's radius absorbs the SLAM
        // estimate's error (a few centimetres, up to about 0.3 m).
        inscribed_radius_m: ROBOT_RADIUS_M + SAFETY_MARGIN_M,
        inflation_radius_m: ROBOT_RADIUS_M + SAFETY_MARGIN_M + 0.4,
        ..CostmapConfig::default()
    }
}

/// Explores the warehouse with `policy` until nothing reachable is left.
fn explore(name: &'static str, policy: ScorePolicy, start: Pose2d, render: bool) -> Run {
    let (_world, backend, physics_world, spec) = warehouse_lidar();
    let mut slam = new_slam();
    let mut explorer = Explorer::new(explorer_config(policy));
    let free_samples = free_samples(start);
    let dt = 1.0 / SCAN_HZ;

    let mut truth = start;
    let mut odom = Pose2d::IDENTITY;
    let mut keyframe_odom: Option<Pose2d> = None;
    let mut keyframe_truth: Vec<Pose2d> = Vec::new();
    let mut path: Option<Path2d> = None;
    let mut since_plan_s = f64::INFINITY;
    let (mut goals, mut plan_failures, mut failures_in_row) = (0, 0, 0);
    let mut collisions = 0;
    let mut distance_m = 0.0;
    let (mut time_to_90_s, mut distance_to_90_m) = (None, None);
    let mut trail = vec![truth];
    let mut frames = Vec::new();
    let mut done = false;
    let mut step = 0usize;
    let mut backoff_s = 0.0;

    while (step as f64) * dt < MAX_TIME_S {
        let time_s = step as f64 * dt;
        // Sense, and map on keyframes.
        let scan = scan_at(&backend, physics_world, truth, &spec);
        let due = keyframe_odom.is_none_or(|last| {
            let delta = last.inverse().compose(odom);
            delta.x_m.hypot(delta.y_m) >= KEYFRAME_M || delta.yaw_rad.abs() >= KEYFRAME_RAD
        });
        if due {
            slam.process(&scan, odom, Pose2d::IDENTITY)
                .expect("slam update");
            keyframe_odom = Some(odom);
            keyframe_truth.push(start.inverse().compose(truth));
        }
        let estimate = slam.pose().compose(
            keyframe_odom
                .expect("first keyframe")
                .inverse()
                .compose(odom),
        );

        // Decide.
        since_plan_s += dt;
        if path.is_none() || since_plan_s >= REPLAN_S {
            since_plan_s = 0.0;
            match choose_goal(&mut explorer, slam.grid(), estimate, path.as_ref()) {
                Choice::Done => done = true,
                Choice::Path {
                    path: planned,
                    new_goal,
                } => {
                    goals += usize::from(new_goal);
                    path = Some(planned);
                    failures_in_row = 0;
                }
                Choice::NoPath => {
                    plan_failures += 1;
                    failures_in_row += 1;
                    path = None;
                }
            }
        }
        if done || failures_in_row >= MAX_PLAN_FAILURES {
            break;
        }

        // Act.
        let (linear, angular, reached) = if backoff_s > 0.0 {
            backoff_s -= dt;
            (-BACKOFF_M_S, 0.0, false)
        } else {
            match path.as_ref() {
                Some(path) => follow(path, estimate),
                None => (0.0, 0.0, false),
            }
        };
        if reached {
            explorer.clear_goal();
            path = None;
        }
        let next = integrate(truth, linear, angular, dt);
        if warehouse::collides(next.x_m, next.y_m, ROBOT_RADIUS_M) {
            collisions += 1;
            // A bump: turning in place is still allowed; back off, and choose
            // a goal afresh from wherever that leaves the robot.
            truth.yaw_rad = next.yaw_rad;
            if backoff_s <= 0.0 {
                backoff_s = BACKOFF_S;
                explorer.clear_goal();
                path = None;
            }
        } else {
            distance_m += (next.x_m - truth.x_m).hypot(next.y_m - truth.y_m);
            truth = next;
        }
        odom = odom.compose(drifting_increment(trail[trail.len() - 1], truth));
        trail.push(truth);

        let coverage = coverage(slam.grid(), &free_samples);
        if coverage >= 0.9 && time_to_90_s.is_none() {
            time_to_90_s = Some(time_s);
            distance_to_90_m = Some(distance_m);
        }
        if render && step.is_multiple_of(FRAME_EVERY_STEPS) {
            frames.push(snapshot(
                start,
                &slam,
                &scan,
                truth,
                estimate,
                &trail,
                path.as_ref(),
                explorer.current().map(|c| c.goal),
                time_s,
                distance_m,
                coverage,
            ));
        }
        step += 1;
    }

    let time_s = step as f64 * dt;
    let coverage = coverage(slam.grid(), &free_samples);
    let estimate = slam.pose();
    if render {
        let scan = scan_at(&backend, physics_world, truth, &spec);
        frames.push(snapshot(
            start, &slam, &scan, truth, estimate, &trail, None, None, time_s, distance_m, coverage,
        ));
    }
    let nodes = slam.graph().nodes();
    let trajectory_error_m = nodes
        .iter()
        .zip(&keyframe_truth)
        .map(|(a, b)| (a.x_m - b.x_m).hypot(a.y_m - b.y_m))
        .sum::<f64>()
        / nodes.len().max(1) as f64;
    let truth_map = start.inverse().compose(truth);
    let hash = state_hash(slam.grid(), nodes, truth);
    Run {
        name,
        time_s,
        distance_m,
        coverage,
        time_to_90_s,
        distance_to_90_m,
        goals,
        plan_failures,
        collisions,
        loop_closures: slam.loop_edge_indices().len(),
        trajectory_error_m,
        odometry_error_m: (odom.x_m - truth_map.x_m).hypot(odom.y_m - truth_map.y_m),
        done,
        hash,
        frames,
    }
}

/// The warehouse in a Rapier world, and the robot's 4 m LiDAR. The ECS world
/// is returned so it outlives the physics bodies synced from it.
fn warehouse_lidar() -> (rne_ecs::World, RapierBackend, PhysicsWorldId, LidarSpec) {
    let mut world = rne_ecs::World::new();
    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    warehouse::spawn(&mut world);
    backend
        .sync_from_ecs(&mut world, physics_world)
        .expect("sync");
    step_physics(
        &mut backend,
        &mut world,
        physics_world,
        SimDuration::from_hertz(Hertz::new(SCAN_HZ)),
    )
    .expect("step");
    let spec = LidarSpec {
        ray_count: 360,
        min_angle_rad: -std::f64::consts::PI,
        max_angle_rad: std::f64::consts::PI,
        max_range_m: LIDAR_RANGE_M,
        height_offset_m: 0.0,
        ..LidarSpec::default()
    };
    (world, backend, physics_world, spec)
}

/// `Slam2d` on an empty grid. The map frame is the robot's odometry frame:
/// it starts at the origin and does not know where in the warehouse it is.
fn new_slam() -> Slam2d {
    let template = OccupancyGrid::new(
        (GRID_M / RESOLUTION_M) as usize,
        (GRID_M / RESOLUTION_M) as usize,
        RESOLUTION_M,
        Pose2d::new(-0.5 * GRID_M, -0.5 * GRID_M, 0.0),
    )
    .expect("grid");
    Slam2d::new(
        template,
        SlamConfig {
            max_beams: 360,
            ..SlamConfig::default()
        },
    )
}

/// The outcome of asking the explorer where to go.
enum Choice {
    Done,
    /// Drive this path; `new_goal` when the goal differs from the current
    /// path's.
    Path {
        path: Path2d,
        new_goal: bool,
    },
    /// The chosen goal has no path (the explorer's goal is cleared).
    NoPath,
}

fn choose_goal(
    explorer: &mut Explorer,
    grid: &OccupancyGrid,
    estimate: Pose2d,
    current: Option<&Path2d>,
) -> Choice {
    let Decision::Goal(candidate) = explorer.decide(grid, estimate).expect("decide") else {
        return Choice::Done;
    };
    let new_goal = !current.and_then(Path2d::goal).is_some_and(|goal| {
        (goal.x_m - candidate.goal.x_m).hypot(goal.y_m - candidate.goal.y_m) < 1e-9
    });
    // The explorer's own reachability gives the path, so the goal it chose is
    // always one the robot can drive to.
    match explorer.path_to(grid, candidate.goal_cell) {
        Some(mut points) => {
            points[0] = Vec3::new(estimate.x_m, estimate.y_m, 0.0);
            Choice::Path {
                path: Path2d::from_points(&points),
                new_goal,
            }
        }
        None => {
            explorer.clear_goal();
            Choice::NoPath
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn snapshot(
    start: Pose2d,
    slam: &Slam2d,
    scan: &LaserScan2d,
    truth: Pose2d,
    estimate: Pose2d,
    trail: &[Pose2d],
    path: Option<&Path2d>,
    goal: Option<Pose2d>,
    time_s: f64,
    distance_m: f64,
    coverage: f64,
) -> render::Snapshot {
    let grid = slam.grid();
    let frontier_cells = extract_frontiers(
        grid,
        &FrontierConfig {
            min_cluster_cells: 8,
        },
    )
    .into_iter()
    .flat_map(|frontier| frontier.cells)
    .map(|cell| grid.grid_to_world(cell))
    .map(|p| (p.x, p.y))
    .collect();
    render::Snapshot {
        cells: grid_values(grid, start),
        frontier_m: frontier_cells,
        scan_m: scan_points(scan, truth),
        truth,
        estimate,
        trail: trail.to_vec(),
        path_m: path
            .map(|p| p.waypoints().iter().map(|w| (w.x_m, w.y_m)).collect())
            .unwrap_or_default(),
        goal,
        time_s,
        distance_m,
        coverage,
    }
}

/// The part of the map the picture shows, as `-1` / `0` / `100`, row-major.
fn grid_values(grid: &OccupancyGrid, start: Pose2d) -> render::MapCells {
    let (min_x, max_x, min_z, max_z) = render::VIEW;
    let corners = [
        (min_x, min_z),
        (max_x, min_z),
        (max_x, max_z),
        (min_x, max_z),
    ]
    .map(|(x, z)| {
        let local = grid
            .origin()
            .inverse_transform_point(start.inverse().transform_point(Vec3::new(x, z, 0.0)));
        (local.x / grid.resolution_m(), local.y / grid.resolution_m())
    });
    let clamp_x = |v: f64| v.clamp(0.0, grid.width() as f64) as usize;
    let clamp_y = |v: f64| v.clamp(0.0, grid.height() as f64) as usize;
    let x0 = clamp_x(
        corners
            .iter()
            .map(|c| c.0)
            .fold(f64::INFINITY, f64::min)
            .floor(),
    );
    let x1 = clamp_x(
        corners
            .iter()
            .map(|c| c.0)
            .fold(f64::NEG_INFINITY, f64::max)
            .ceil(),
    );
    let y0 = clamp_y(
        corners
            .iter()
            .map(|c| c.1)
            .fold(f64::INFINITY, f64::min)
            .floor(),
    );
    let y1 = clamp_y(
        corners
            .iter()
            .map(|c| c.1)
            .fold(f64::NEG_INFINITY, f64::max)
            .ceil(),
    );
    let mut values = Vec::with_capacity((x1 - x0) * (y1 - y0));
    for y in y0..y1 {
        for x in x0..x1 {
            values.push(grid.cell_value(GridCoord {
                x: x as isize,
                y: y as isize,
            }));
        }
    }
    let resolution = grid.resolution_m();
    render::MapCells {
        width: x1 - x0,
        height: y1 - y0,
        resolution_m: resolution,
        origin: grid.origin().compose(Pose2d::new(
            x0 as f64 * resolution,
            y0 as f64 * resolution,
            0.0,
        )),
        values,
    }
}

/// A heading-first follower: turn in place toward the path when facing away
/// from it, otherwise drive and steer toward the lookahead point.
fn follow(path: &Path2d, pose: Pose2d) -> (f64, f64, bool) {
    let Some(goal) = path.goal() else {
        return (0.0, 0.0, true);
    };
    let position = Vec3::new(pose.x_m, pose.y_m, 0.0);
    let to_goal = (goal.x_m - pose.x_m).hypot(goal.y_m - pose.y_m);
    if to_goal <= GOAL_TOLERANCE_M {
        return (0.0, 0.0, true);
    }
    let arc = path.closest_point(position).map_or(0.0, |c| c.arc_length_m);
    let target = path
        .point_at_distance(arc + LOOKAHEAD_M)
        .unwrap_or(Vec3::new(goal.x_m, goal.y_m, 0.0));
    let heading_error = wrap((target.y - pose.y_m).atan2(target.x - pose.x_m) - pose.yaw_rad);
    let angular = (2.0 * heading_error).clamp(-MAX_TURN_RAD_S, MAX_TURN_RAD_S);
    if heading_error.abs() > 0.8 {
        return (0.0, angular, false);
    }
    let slow = (to_goal / 0.6).clamp(0.3, 1.0);
    (MAX_SPEED_M_S * heading_error.cos() * slow, angular, false)
}

fn integrate(pose: Pose2d, linear: f64, angular: f64, dt: f64) -> Pose2d {
    let yaw = pose.yaw_rad + 0.5 * angular * dt;
    Pose2d::new(
        pose.x_m + linear * dt * yaw.cos(),
        pose.y_m + linear * dt * yaw.sin(),
        wrap(pose.yaw_rad + angular * dt),
    )
}

fn wrap(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// The odometry increment between two true poses, with a 2 % scale error and
/// a yaw bias, as in example 127.
fn drifting_increment(from: Pose2d, to: Pose2d) -> Pose2d {
    let delta = from.inverse().compose(to);
    let travelled = delta.x_m.hypot(delta.y_m);
    Pose2d::new(
        delta.x_m * 1.02,
        delta.y_m * 1.02,
        delta.yaw_rad * 1.03 + 0.004 * travelled,
    )
}

/// Points of the warehouse's free floor every 10 cm, in the map frame.
fn free_samples(start: Pose2d) -> Vec<Vec3> {
    let (min_x, max_x, min_z, max_z) = warehouse::INTERIOR;
    let step = 0.1;
    let mut samples = Vec::new();
    let mut z = min_z + 0.5 * step;
    while z < max_z {
        let mut x = min_x + 0.5 * step;
        while x < max_x {
            if !warehouse::collides(x, z, 0.0) {
                samples.push(start.inverse().transform_point(Vec3::new(x, z, 0.0)));
            }
            x += step;
        }
        z += step;
    }
    samples
}

/// Share of the true free floor the map knows (free or occupied).
fn coverage(grid: &OccupancyGrid, samples: &[Vec3]) -> f64 {
    let known = samples
        .iter()
        .filter(|p| grid.world_to_grid(**p).is_some_and(|c| grid.is_known(c)))
        .count();
    known as f64 / samples.len().max(1) as f64
}

fn state_hash(grid: &OccupancyGrid, nodes: &[Pose2d], truth: Pose2d) -> u64 {
    // FNV-1a over the map's log-odds, the SLAM nodes and the final pose.
    let mut hash = 0xcbf2_9ce4_8422_2325u64;
    let mut eat = |bytes: &[u8]| {
        for byte in bytes {
            hash ^= u64::from(*byte);
            hash = hash.wrapping_mul(0x0100_0000_01b3);
        }
    };
    for value in grid.log_odds() {
        eat(&value.to_le_bytes());
    }
    for node in nodes.iter().chain(std::iter::once(&truth)) {
        eat(&node.x_m.to_le_bytes());
        eat(&node.y_m.to_le_bytes());
        eat(&node.yaw_rad.to_le_bytes());
    }
    hash
}

fn base_transform(pose: Pose2d) -> Transform3 {
    // The SLAM plane is (x, z) with yaw turning x toward z, which is a
    // rotation of -yaw about +y.
    Transform3::from_translation_rotation(
        Vec3::new(pose.x_m, LIDAR_HEIGHT_M, pose.y_m),
        Quat::from_rotation_y(-pose.yaw_rad),
    )
}

fn scan_at(
    backend: &RapierBackend,
    physics_world: PhysicsWorldId,
    pose: Pose2d,
    spec: &LidarSpec,
) -> LaserScan2d {
    let base = base_transform(pose);
    let cloud = sample_lidar(backend, physics_world, &base, spec);
    cloud_to_scan(&cloud, &base, spec)
}

/// Bins a world-frame point cloud into a planar scan in the base frame.
///
/// A beam with no return gets a range past the maximum, so the map clears
/// free space along it. A bin left empty only because its neighbours took
/// the returns (binning, not open space) is left infinite instead, which the
/// map skips: clearing it would carve free space through a wall.
fn cloud_to_scan(cloud: &PointCloud, base_world: &Transform3, spec: &LidarSpec) -> LaserScan2d {
    let count = spec.ray_count.max(1) as usize;
    let increment = (spec.max_angle_rad - spec.min_angle_rad) / count as f64;
    let no_return = 2.0 * spec.max_range_m;
    let mut ranges = vec![no_return; count];
    let inverse = rne_math::Transform3 {
        translation: base_world.translation,
        rotation: base_world.rotation,
        scale: base_world.scale,
    }
    .inverse();
    for point in &cloud.points_m {
        let local = inverse.transform_point(*point);
        let range = local.x.hypot(local.z);
        if range < spec.min_range_m || range > spec.max_range_m {
            continue;
        }
        let angle = local.z.atan2(local.x);
        let index = (((angle - spec.min_angle_rad) / increment).round() as isize)
            .rem_euclid(count as isize) as usize;
        ranges[index] = ranges[index].min(range);
    }
    let hit = |i: usize| ranges[i] <= spec.max_range_m;
    let gaps: Vec<usize> = (0..count)
        .filter(|&i| !hit(i) && (hit((i + count - 1) % count) || hit((i + 1) % count)))
        .collect();
    for i in gaps {
        ranges[i] = f64::INFINITY;
    }
    LaserScan2d {
        time_s: 0.0,
        frame: rne_nav::FrameId::new("lidar"),
        angle_min_rad: spec.min_angle_rad,
        angle_increment_rad: increment,
        range_min_m: spec.min_range_m,
        range_max_m: spec.max_range_m,
        ranges_m: ranges,
    }
}

/// The scan's returns in the world, at the true pose.
fn scan_points(scan: &LaserScan2d, pose: Pose2d) -> Vec<(f64, f64)> {
    scan.ranges_m
        .iter()
        .enumerate()
        .filter(|(_, range)| **range <= scan.range_max_m)
        .map(|(beam, range)| {
            let angle = scan.angle_min_rad + scan.angle_increment_rad * beam as f64;
            let p = pose.transform_point(Vec3::new(range * angle.cos(), range * angle.sin(), 0.0));
            (p.x, p.y)
        })
        .collect()
}

fn report(runs: &[&Run]) {
    println!(
        "{:<18} {:>6} {:>8} {:>7} {:>9} {:>9} {:>5} {:>6} {:>5} {:>5} {:>8} {:>8}",
        "policy",
        "time",
        "distance",
        "mapped",
        "90% time",
        "90% dist",
        "goals",
        "nopath",
        "coll",
        "loops",
        "SLAM err",
        "odom err"
    );
    for run in runs {
        println!(
            "{:<18} {:>5.0}s {:>7.1}m {:>6.1}% {:>9} {:>9} {:>5} {:>6} {:>5} {:>5} {:>7.3}m {:>7.2}m{}",
            run.name,
            run.time_s,
            run.distance_m,
            100.0 * run.coverage,
            run.time_to_90_s
                .map_or("-".to_string(), |t| format!("{t:.0}s")),
            run.distance_to_90_m
                .map_or("-".to_string(), |d| format!("{d:.1}m")),
            run.goals,
            run.plan_failures,
            run.collisions,
            run.loop_closures,
            run.trajectory_error_m,
            run.odometry_error_m,
            if run.done { "" } else { "  (not done)" },
        );
    }
}

/// The gates every run must pass, baseline included. Not every corner of
/// the floor can be seen: the 0.75 m strip between the north pallets and the
/// wall is narrower than the robot plus its clearance, and the LiDAR at
/// 0.5 m cannot see over the pallets. Over eight starts the lowest
/// coverage measured was 93.0 % (nearest frontier), so the gate is 92 %.
fn check(run: &Run) {
    assert!(run.done, "{}: exploration did not finish", run.name);
    assert!(
        run.coverage >= 0.92,
        "{}: mapped only {:.1}% of the floor",
        run.name,
        100.0 * run.coverage
    );
    assert_eq!(run.collisions, 0, "{}: collided", run.name);
    assert!(
        run.trajectory_error_m < 0.3,
        "{}: SLAM trajectory error {:.3} m",
        run.name,
        run.trajectory_error_m
    );
}
