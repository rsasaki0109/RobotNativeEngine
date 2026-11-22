//! GNSS outdoors, LiDAR indoors: a robot walks from a plaza along the
//! sidewalk of a downtown main street (crossing side streets on the
//! pedestrian signal, among traffic that obeys the signals) into an office
//! building and back, localizing with ray-traced GNSS outside and LiDAR
//! Monte Carlo localization inside.
//!
//! * GNSS: `rne_gnss` ray traces 48 GPS + Galileo satellites through the
//!   buildings every epoch (direct paths, up to two wall/ground bounces, and
//!   knife-edge diffraction over building edges) on two frequencies, for two
//!   antennas on the robot and a base station on a rooftop. libgnss++
//!   (through `rne_adapter_gnsspp`) solves carrier-phase RTK for the rear
//!   antenna and moving-baseline RTK between the antennas for heading. Without
//!   libgnss++ (see `scripts/build_gnsspp.sh`) the rear antenna's built-in
//!   single-point fixes are used.
//! * LiDAR: a 360-beam planar scan ray cast against the same geometry, matched
//!   against a prior occupancy map of the route (buildings and street
//!   furniture), and used for a reactive obstacle stop.
//! * `rne_slam::HybridLocalizer` decides which source corrects the wheel/gyro
//!   EKF, and pure pursuit steers on the *estimated* pose.
//!
//! The run is compared against odometry alone and against an EKF that fuses
//! every GNSS solution without checks. An SVG of the site, trajectories, and
//! a sky plot from inside the canyon is written (`--svg <path>` to choose).
//! `--no-map-aided-gnss` keeps predicted-NLOS satellites in RTK and
//! `--indoor-map-only` limits the LiDAR map to the office, for comparison.
//! `--frames <dir>` also writes one SVG per 1.5 s of the run, which
//! `scripts/svg_frames_to_gif.py` turns into an animation, and `--trace
//! <path>` a JSON trace (every 0.5 s) for the 3D renderer
//! `151_gnss_lidar_navigation_render`.
//!
//! Run with `cargo run --release -p gnss_lidar_navigation --example 151_gnss_lidar_navigation`.

mod city;
mod frames;
mod gnss;
mod trace;

use city::BASE_ENU_M;
use gnss::{GnssOutput, RobotGnss, SolutionLabel, BODY_HEIGHT_M};
use rne_core::DeterministicRng;
use rne_gnss::{
    BodyPose, Constellation, EnuFrame, Geodetic, GnssEpoch, GnssScene, ReceiverConfig, SignalPath,
    TraceConfig,
};
use rne_math::Vec3;
use rne_nav::{
    pure_pursuit_follow, EkfFusion, FrameId, GridCoord, LaserScan2d, OccupancyGrid, Path2d, Pose2d,
    PurePursuitConfig, VelocityCommand2d,
};
use rne_slam::{AmclConfig, FixKind, HybridLocalizer, HybridLocalizerConfig, LocalizationSource};
use std::f64::consts::{FRAC_PI_2, TAU};
use std::fmt::Write as _;

const DT_S: f64 = 0.05;
const LIDAR_EVERY_TICKS: usize = 2;
const LIDAR_HEIGHT_M: f64 = 0.3;
const LIDAR_BEAMS: usize = 360;
const LIDAR_RANGE_M: f64 = 20.0;
const LIDAR_NOISE_M: f64 = 0.02;
/// Constellation time at the start of the run (sets the sky geometry).
const SKY_EPOCH_S: f64 = 20_000.0;
const MAX_TIME_S: f64 = 900.0;
use city::OFFICE;
/// Prior LiDAR map extent `[x0, y0, x1, y1]`: the whole route, outdoors
/// included, as a robot that maps its site beforehand would have.
const MAP: [f64; 4] = [10.0, -55.0, 180.0, 22.0];
/// The office-only map of `--indoor-map-only`.
const INDOOR_MAP: [f64; 4] = [132.0, -52.0, 178.0, -14.0];
const MAP_RESOLUTION_M: f64 = 0.1;
/// Dead reckoning longer than this stops the robot until it relocalizes.
const MAX_DEAD_RECKONING_S: f64 = 3.0;
/// A LiDAR return closer than this ahead stops forward motion.
const STOP_DISTANCE_M: f64 = 0.6;

fn main() {
    let (svg_path, frames_dir, trace_path) = output_args();
    let mut run = Run::new(frames_dir.is_some() || trace_path.is_some());
    let mut tick = 0_usize;
    let mut reached = false;
    while (tick as f64) * DT_S < MAX_TIME_S {
        if !run.step(tick) {
            reached = true;
            break;
        }
        tick += 1;
    }
    run.log.print_summary(
        reached,
        tick as f64 * DT_S,
        &run.hybrid,
        run.gnss.base_epoch(),
    );
    let map_scene = run.city.map_scene();
    let ground = run.city.ground.clone();
    write_outputs(
        &svg_path,
        frames_dir.as_deref(),
        (&map_scene, &ground),
        &run.log,
        &run.frames,
    );
    if let Some(path) = trace_path {
        trace_output(&path, &run.city, &run.log, &run.frames);
    }
}

/// The simulation: the world, the robot, its sensors and localizer.
#[allow(missing_debug_implementations)]
struct Run {
    city: city::City,
    model_scene: GnssScene,
    constellation: Constellation,
    follower: RouteFollower,
    gnss: RobotGnss,
    config: HybridLocalizerConfig,
    hybrid: HybridLocalizer,
    gnss_only: EkfFusion,
    rng: DeterministicRng,
    truth: Pose2d,
    odom: Pose2d,
    log: RunLog,
    recording: bool,
    frames: Vec<frames::FrameRecord>,
    last_label: Option<SolutionLabel>,
    last_heading: bool,
    last_scan_points: Vec<(f64, f64)>,
    dead_reckoning_s: f64,
    stopped: bool,
    waiting: bool,
    blocked: bool,
    /// Nearest LiDAR return ahead of the robot, from the latest scan.
    front_clearance_m: f64,
    map_aided_gnss: bool,
}

impl Run {
    fn new(recording: bool) -> Self {
        let frame = EnuFrame::new(Geodetic::from_degrees(36.0827, 140.0764, 25.0));
        let city = city::City::build();
        let constellation = Constellation::gps_galileo_nominal();
        let indoor_only = std::env::args().any(|a| a == "--indoor-map-only");
        let prior_map = build_prior_map(
            &city.map_scene(),
            if indoor_only { INDOOR_MAP } else { MAP },
        );
        // Withhold satellites the city model predicts are hidden (NLOS).
        let map_aided_gnss = !std::env::args().any(|a| a == "--no-map-aided-gnss");
        let follower = RouteFollower::new();
        let start = follower.start();
        let gnss = RobotGnss::new(
            &constellation,
            frame,
            receiver_config(),
            SKY_EPOCH_S + 300.0,
        );
        println!(
            "GNSS solver: {}",
            if gnss.uses_libgnss() {
                "libgnss++ RTK (rear antenna vs rooftop base) + moving-baseline heading"
            } else {
                "built-in single-point positioning (libgnss++ not linked; run scripts/build_gnsspp.sh)"
            }
        );
        let mut config = HybridLocalizerConfig {
            amcl: AmclConfig {
                particle_count: 600,
                ..AmclConfig::default()
            },
            ..HybridLocalizerConfig::default()
        };
        if gnss.uses_libgnss() {
            // Hand back to GNSS only on RTK fixed solutions; float solutions
            // out of the canyon can be metres off with a confident covariance.
            config.enter_gnss_min_kind = FixKind::Fixed;
            config.min_float_variance_m2 = 4.0;
            // With RTK in charge outdoors the estimate going into LiDAR mode
            // is good; gate LiDAR matches so that scans sliding along a
            // facade or matching parked cars cannot drag it.
            config.lidar_gate_chi_square = 9.21;
        }
        let hybrid = HybridLocalizer::new(prior_map, start, config).expect("localizer");
        let gnss_only =
            EkfFusion::with_initial_state(config.ekf, start, VelocityCommand2d::ZERO).expect("ekf");
        Self {
            model_scene: city.model_scene(),
            city,
            constellation,
            follower,
            gnss,
            config,
            hybrid,
            gnss_only,
            rng: DeterministicRng::new(0x0D0_5EED),
            truth: start,
            odom: start,
            log: RunLog::default(),
            recording,
            frames: Vec::new(),
            last_label: None,
            last_heading: false,
            last_scan_points: Vec::new(),
            dead_reckoning_s: 0.0,
            stopped: false,
            waiting: false,
            blocked: false,
            front_clearance_m: f64::INFINITY,
            map_aided_gnss,
        }
    }

    /// Advances one tick; returns `false` once the route is complete.
    fn step(&mut self, tick: usize) -> bool {
        let t = tick as f64 * DT_S;
        let Some(command) = self.follower.command(self.hybrid.pose()) else {
            return false;
        };
        let command = self.crossing_gate(command, t);
        let command = self.safety_gate(command);
        let command = self.obstacle_gate(command);
        self.truth = integrate(self.truth, command.linear_m_s, command.angular_rad_s, DT_S);

        // Wheel speed with a 1 % scale error and a gyro yaw rate with a
        // 0.06 °/s residual bias, both with white noise.
        let odom_v = command.linear_m_s * 1.01 + 0.01 * gaussian(&mut self.rng);
        let odom_w = command.angular_rad_s + 0.001 + 0.005 * gaussian(&mut self.rng);
        self.odom = integrate(self.odom, odom_v, odom_w, DT_S);
        self.hybrid.predict(DT_S, odom_v, odom_w).expect("predict");
        self.gnss_only.predict(DT_S).expect("predict");
        self.gnss_only
            .update_odometry(
                odom_v,
                odom_w,
                self.config.odometry_linear_variance,
                self.config.odometry_angular_variance,
            )
            .expect("odometry");

        self.city.step_traffic(t, DT_S);
        let world = self.city.world_scene();
        self.step_gnss(&world, t);
        if tick.is_multiple_of(LIDAR_EVERY_TICKS) {
            let scan = lidar_scan(&world, self.truth, t, &mut self.rng);
            self.log.clearance_m = self.log.clearance_m.min(min_range(&scan));
            self.front_clearance_m = front_range(&scan);
            if self.recording {
                self.last_scan_points = scan_world_points(&scan, self.truth);
            }
            self.hybrid
                .update_lidar(&scan, self.odom, Pose2d::IDENTITY, t)
                .expect("lidar");
        }

        self.log.record_pose(
            t,
            self.truth,
            self.hybrid.pose(),
            self.hybrid.source(),
            self.gnss_only.pose(),
            self.odom,
        );
        if self.recording && t + 1.0e-9 >= self.frames.len() as f64 * frames::RECORD_PERIOD_S {
            self.frames.push(frames::FrameRecord {
                t,
                sample_count: self.log.samples.len(),
                truth: self.truth,
                estimate: self.hybrid.pose(),
                source: self.hybrid.source(),
                label: self.last_label,
                heading_fixed: self.last_heading,
                epoch: self.gnss.rear_epoch().cloned(),
                scan_points: self.last_scan_points.clone(),
                vehicles: self.city.vehicles.clone(),
                signals: city::signals_at(t),
            });
        }
        true
    }

    /// Waits at the kerb of a cross street until pedestrians may cross: the
    /// main-street green with enough of it left to reach the far side.
    fn crossing_gate(&mut self, command: VelocityCommand2d, t: f64) -> VelocityCommand2d {
        let pose = self.hybrid.pose();
        let on_sidewalk = (-city::MAIN_HALF_ROW_M..-city::MAIN_HALF_ROAD_M).contains(&pose.y_m);
        let east = pose.yaw_rad.cos() > 0.5;
        let west = pose.yaw_rad.cos() < -0.5;
        let at_kerb = city::CROSS_STREETS_X.iter().any(|c| {
            let near = c - pose.x_m;
            (east && (9.0..11.5).contains(&near)) || (west && (-11.5..-9.0).contains(&near))
        });
        if on_sidewalk && at_kerb && !city::signals_at(t).walk_along_main {
            if !self.waiting {
                self.log.crossing_waits += 1;
            }
            self.waiting = true;
            self.log.waiting_s += DT_S;
            VelocityCommand2d::ZERO
        } else {
            self.waiting = false;
            command
        }
    }

    /// Safety stop: an autonomous robot does not keep driving on odometry
    /// alone. It stops until GNSS or LiDAR localizes it again.
    fn safety_gate(&mut self, command: VelocityCommand2d) -> VelocityCommand2d {
        if self.hybrid.source() == LocalizationSource::DeadReckoning {
            self.dead_reckoning_s += DT_S;
        } else {
            self.dead_reckoning_s = 0.0;
        }
        self.log.longest_dead_reckoning_s =
            self.log.longest_dead_reckoning_s.max(self.dead_reckoning_s);
        if self.dead_reckoning_s > MAX_DEAD_RECKONING_S {
            if !self.stopped {
                self.log.safety_stops += 1;
            }
            self.stopped = true;
            self.log.stopped_s += DT_S;
            VelocityCommand2d::ZERO
        } else {
            self.stopped = false;
            command
        }
    }

    /// Reactive stop on raw LiDAR ranges, independent of localization: the
    /// robot does not drive into anything ahead of it even if it is lost.
    fn obstacle_gate(&mut self, command: VelocityCommand2d) -> VelocityCommand2d {
        if command.linear_m_s > 0.0 && self.front_clearance_m < STOP_DISTANCE_M {
            if !self.blocked {
                self.log.obstacle_stops += 1;
            }
            self.blocked = true;
            VelocityCommand2d {
                linear_m_s: 0.0,
                ..command
            }
        } else {
            self.blocked = false;
            command
        }
    }

    fn step_gnss(&mut self, world: &GnssScene, t: f64) {
        let pose = |p: Pose2d| BodyPose {
            x_m: p.x_m,
            y_m: p.y_m,
            z_m: BODY_HEIGHT_M,
            yaw_rad: p.yaw_rad,
        };
        let outputs = self.gnss.step(
            world,
            self.map_aided_gnss.then_some(&self.model_scene),
            &self.constellation,
            pose(self.truth),
            pose(self.hybrid.pose()),
            SKY_EPOCH_S + t,
        );
        if let Some(epoch) = self.gnss.rear_epoch() {
            if (epoch.time_s - (SKY_EPOCH_S + t)).abs() < 1.0e-9 {
                self.log.record_epoch(epoch, zone(self.truth));
            }
        }
        for output in &outputs {
            self.hybrid
                .update_gnss(output.fix.as_ref(), t)
                .expect("gnss");
            if let Some(fix) = output.fix {
                let floor = match fix.kind {
                    FixKind::Fixed => self.config.min_fixed_variance_m2,
                    FixKind::Float => self.config.min_float_variance_m2,
                    FixKind::Single => self.config.min_gnss_variance_m2,
                };
                let variance = (fix.horizontal_std_m * fix.horizontal_std_m).max(floor);
                self.gnss_only
                    .update_position(fix.x_m, fix.y_m, variance)
                    .expect("gnss only");
            }
            let fused_heading = match &output.heading {
                Some(heading) => self.hybrid.update_gnss_heading(heading).expect("heading"),
                None => false,
            };
            self.log.record_solution(output, self.truth, fused_heading);
            self.last_label = Some(output.label);
            self.last_heading = output.heading.is_some();
        }
    }
}

/// The robot's and the base station's receivers. Surfaces more than 90 m
/// from an antenna are not traced: their reflections arrive more than a
/// code chip late (no code multipath) and, after two such bounces, too weak.
fn receiver_config() -> ReceiverConfig {
    let default = ReceiverConfig::default();
    ReceiverConfig {
        trace: TraceConfig {
            max_interaction_distance_m: 90.0,
            ..default.trace
        },
        ..default
    }
}

/// `--svg <path>`, `--frames <dir>` and `--trace <path>`.
fn output_args() -> (
    String,
    Option<std::path::PathBuf>,
    Option<std::path::PathBuf>,
) {
    let value = |flag: &str| std::env::args().skip_while(|a| a != flag).nth(1);
    (
        value("--svg").unwrap_or_else(|| "target/151_gnss_lidar_navigation.svg".to_string()),
        value("--frames").map(std::path::PathBuf::from),
        value("--trace").map(std::path::PathBuf::from),
    )
}

/// Writes the JSON trace for the 3D renderer.
fn trace_output(
    path: &std::path::Path,
    city: &city::City,
    log: &RunLog,
    frames: &[frames::FrameRecord],
) {
    match trace::write_trace(path, city, &receiver_config().trace, log, frames) {
        Ok(()) => println!("wrote 3D render trace {}", path.display()),
        Err(error) => println!("could not write {}: {error}", path.display()),
    }
}

/// Writes the summary SVG and, when asked, the animation frames.
fn write_outputs(
    svg_path: &str,
    frames_dir: Option<&std::path::Path>,
    (scene, ground): (&GnssScene, &[city::Patch]),
    log: &RunLog,
    frames: &[frames::FrameRecord],
) {
    let svg = render_svg(scene, ground, log, SKY_EPOCH_S);
    if let Some(parent) = std::path::Path::new(svg_path).parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    match std::fs::write(svg_path, svg) {
        Ok(()) => println!("\nwrote {svg_path}"),
        Err(error) => println!("\ncould not write {svg_path}: {error}"),
    }
    if let Some(dir) = frames_dir {
        match frames::write_frames(dir, scene, ground, log, frames) {
            Ok(count) => println!("wrote {count} animation frames to {}", dir.display()),
            Err(error) => println!("could not write frames to {}: {error}", dir.display()),
        }
    }
}

/// LiDAR returns in the map frame.
fn scan_world_points(scan: &LaserScan2d, pose: Pose2d) -> Vec<(f64, f64)> {
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

/// Rasterizes the scene at LiDAR height into the prior occupancy map.
fn build_prior_map(scene: &GnssScene, extent: [f64; 4]) -> OccupancyGrid {
    let [x0, y0, x1, y1] = extent;
    let width = ((x1 - x0) / MAP_RESOLUTION_M).round() as usize;
    let height = ((y1 - y0) / MAP_RESOLUTION_M).round() as usize;
    let mut grid =
        OccupancyGrid::new(width, height, MAP_RESOLUTION_M, Pose2d::new(x0, y0, 0.0)).expect("map");
    for y in 0..height {
        for x in 0..width {
            let coord = GridCoord {
                x: x as isize,
                y: y as isize,
            };
            let center = grid.grid_to_world(coord);
            if scene.inside_solid(Vec3::new(center.x, center.y, LIDAR_HEIGHT_M)) {
                grid.mark_occupied(coord);
            } else {
                grid.mark_free(coord);
            }
        }
    }
    grid
}

/// Plaza → south sidewalk of the main street (over two crossings) → office
/// forecourt → office (north room, south room) → back the same way, as a
/// delivery robot would go.
const ROUTE: [(f64, f64); 14] = [
    (22.0, -42.0),
    (30.0, -12.5),
    (155.0, -12.5),
    (155.0, -18.0),
    (155.0, -26.0),
    (154.0, -36.5),
    (146.0, -38.0),
    (154.0, -37.5),
    (154.5, -28.0),
    (155.0, -17.0),
    (152.0, -12.0),
    (30.0, -12.0),
    (25.0, -30.0),
    (22.0, -40.0),
];

/// Follows the route one straight leg at a time (the route doubles back on
/// itself, so a single path would let pure pursuit skip ahead), turning in
/// place when the next leg points away.
#[derive(Debug)]
struct RouteFollower {
    leg: usize,
    config: PurePursuitConfig,
}

impl RouteFollower {
    fn new() -> Self {
        Self {
            leg: 0,
            config: PurePursuitConfig {
                lookahead_m: 1.2,
                max_linear_m_s: 1.2,
                max_angular_rad_s: 1.0,
                goal_tolerance_m: 0.6,
                slow_radius_m: 0.0,
            },
        }
    }

    fn start(&self) -> Pose2d {
        let (a, b) = (ROUTE[0], ROUTE[1]);
        Pose2d::new(a.0, a.1, (b.1 - a.1).atan2(b.0 - a.0))
    }

    /// Returns `None` once the final waypoint is reached.
    fn command(&mut self, estimate: Pose2d) -> Option<VelocityCommand2d> {
        loop {
            if self.leg + 1 >= ROUTE.len() {
                return None;
            }
            let (a, b) = (ROUTE[self.leg], ROUTE[self.leg + 1]);
            let last = self.leg + 2 == ROUTE.len();
            let config = PurePursuitConfig {
                slow_radius_m: if last { 1.5 } else { 0.0 },
                goal_tolerance_m: if last {
                    0.4
                } else {
                    self.config.goal_tolerance_m
                },
                ..self.config
            };
            let path = Path2d::from_points(&[Vec3::new(a.0, a.1, 0.0), Vec3::new(b.0, b.1, 0.0)]);
            let result = pure_pursuit_follow(&path, estimate, &config).expect("follow");
            if result.reached {
                self.leg += 1;
                continue;
            }
            let heading = (b.1 - estimate.y_m).atan2(b.0 - estimate.x_m);
            let error = rne_nav::wrap_angle(heading - estimate.yaw_rad);
            if error.abs() > 0.9 {
                return Some(VelocityCommand2d::new(
                    0.0,
                    config.max_angular_rad_s * error.signum(),
                ));
            }
            return Some(result.command);
        }
    }
}

fn integrate(pose: Pose2d, v: f64, w: f64, dt: f64) -> Pose2d {
    let yaw = pose.yaw_rad + 0.5 * w * dt;
    Pose2d::new(
        pose.x_m + v * dt * yaw.cos(),
        pose.y_m + v * dt * yaw.sin(),
        rne_nav::wrap_angle(pose.yaw_rad + w * dt),
    )
}

fn lidar_scan(scene: &GnssScene, pose: Pose2d, t: f64, rng: &mut DeterministicRng) -> LaserScan2d {
    let origin = Vec3::new(pose.x_m, pose.y_m, LIDAR_HEIGHT_M);
    let ranges_m = (0..LIDAR_BEAMS)
        .map(|beam| {
            let angle = pose.yaw_rad + TAU * beam as f64 / LIDAR_BEAMS as f64;
            let noise = LIDAR_NOISE_M * gaussian(rng);
            match scene.raycast(
                origin,
                Vec3::new(angle.cos(), angle.sin(), 0.0),
                LIDAR_RANGE_M,
            ) {
                Some(hit) => hit.distance_m + noise,
                None => f64::INFINITY,
            }
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

fn min_range(scan: &LaserScan2d) -> f64 {
    scan.ranges_m
        .iter()
        .copied()
        .filter(|r| r.is_finite())
        .fold(f64::INFINITY, f64::min)
}

fn gaussian(rng: &mut DeterministicRng) -> f64 {
    let u1 = rng.uniform_f64(1.0e-12, 1.0);
    let u2 = rng.uniform_f64(0.0, 1.0);
    (-2.0 * u1.ln()).sqrt() * (TAU * u2).cos()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Zone {
    Plaza,
    Canyon,
    Approach,
    Indoor,
}

impl Zone {
    const ALL: [Self; 4] = [Self::Plaza, Self::Canyon, Self::Approach, Self::Indoor];

    fn label(self) -> &'static str {
        match self {
            Self::Plaza => "plaza",
            Self::Canyon => "main street",
            Self::Approach => "office forecourt",
            Self::Indoor => "indoor office",
        }
    }
}

fn zone(pose: Pose2d) -> Zone {
    let [x0, y0, x1, y1] = OFFICE;
    if (x0..=x1).contains(&pose.x_m) && (y0..=y1).contains(&pose.y_m) {
        Zone::Indoor
    } else if pose.y_m < -15.0 && pose.x_m < 59.0 {
        Zone::Plaza
    } else if pose.x_m < 138.0 {
        Zone::Canyon
    } else {
        Zone::Approach
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct ErrorStats {
    count: usize,
    sum_sq: f64,
    max: f64,
}

impl ErrorStats {
    fn add(&mut self, error: f64) {
        self.count += 1;
        self.sum_sq += error * error;
        self.max = self.max.max(error);
    }

    fn rmse(&self) -> f64 {
        if self.count == 0 {
            f64::NAN
        } else {
            (self.sum_sq / self.count as f64).sqrt()
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct SkyStats {
    epochs: usize,
    tracked: usize,
    los: usize,
    multipath: usize,
    nlos: usize,
    diffracted: usize,
    blocked: usize,
    phase_locked: usize,
    solutions: [usize; 4],
    fix_error: ErrorStats,
    headings: usize,
    heading_error_deg: ErrorStats,
}

#[derive(Clone, Copy, Debug)]
struct Sample {
    t: f64,
    truth: Pose2d,
    hybrid: Pose2d,
    source: LocalizationSource,
    gnss_only: Pose2d,
    odom: Pose2d,
}

#[derive(Debug)]
struct RunLog {
    samples: Vec<Sample>,
    hybrid: [ErrorStats; 4],
    gnss_only: [ErrorStats; 4],
    odom: [ErrorStats; 4],
    sky: [SkyStats; 4],
    fixes: Vec<(Vec3, SolutionLabel)>,
    snapshot: Option<GnssEpoch>,
    clearance_m: f64,
    headings_fused: usize,
    longest_dead_reckoning_s: f64,
    safety_stops: usize,
    obstacle_stops: usize,
    stopped_s: f64,
    crossing_waits: usize,
    waiting_s: f64,
    /// Zone of each measured epoch whose solution has not been recorded yet.
    pending_zones: std::collections::VecDeque<Zone>,
}

impl Default for RunLog {
    fn default() -> Self {
        Self {
            samples: Vec::new(),
            hybrid: [ErrorStats::default(); 4],
            gnss_only: [ErrorStats::default(); 4],
            odom: [ErrorStats::default(); 4],
            sky: [SkyStats::default(); 4],
            fixes: Vec::new(),
            snapshot: None,
            clearance_m: f64::INFINITY,
            headings_fused: 0,
            longest_dead_reckoning_s: 0.0,
            safety_stops: 0,
            obstacle_stops: 0,
            stopped_s: 0.0,
            crossing_waits: 0,
            waiting_s: 0.0,
            pending_zones: std::collections::VecDeque::new(),
        }
    }
}

fn planar_error(a: Pose2d, b: Pose2d) -> f64 {
    (a.x_m - b.x_m).hypot(a.y_m - b.y_m)
}

fn label_index(label: SolutionLabel) -> usize {
    match label {
        SolutionLabel::Fixed => 0,
        SolutionLabel::Float => 1,
        SolutionLabel::Single => 2,
        SolutionLabel::None => 3,
    }
}

impl RunLog {
    fn record_pose(
        &mut self,
        t: f64,
        truth: Pose2d,
        hybrid: Pose2d,
        source: LocalizationSource,
        gnss_only: Pose2d,
        odom: Pose2d,
    ) {
        let z = zone(truth) as usize;
        self.hybrid[z].add(planar_error(hybrid, truth));
        self.gnss_only[z].add(planar_error(gnss_only, truth));
        self.odom[z].add(planar_error(odom, truth));
        self.samples.push(Sample {
            t,
            truth,
            hybrid,
            source,
            gnss_only,
            odom,
        });
    }

    fn record_epoch(&mut self, epoch: &GnssEpoch, zone: Zone) {
        self.pending_zones.push_back(zone);
        let sky = &mut self.sky[zone as usize];
        sky.epochs += 1;
        sky.tracked += epoch
            .observations
            .iter()
            .filter(|o| o.path().is_tracked())
            .count();
        sky.los += epoch.count(SignalPath::LineOfSight);
        sky.multipath += epoch.count(SignalPath::LineOfSightMultipath);
        sky.nlos += epoch.count(SignalPath::NonLineOfSight);
        sky.diffracted += epoch.count(SignalPath::Diffracted);
        sky.blocked += epoch.count(SignalPath::Blocked) + epoch.count(SignalPath::TooWeak);
        sky.phase_locked += epoch
            .observations
            .iter()
            .filter(|o| {
                o.primary()
                    .is_some_and(|p| p.carrier_phase_cycles.is_some())
            })
            .count();
        if self.snapshot.is_none()
            && zone == Zone::Canyon
            && (epoch.antenna_enu_m.x - 93.0).abs() < 1.0
        {
            self.snapshot = Some(epoch.clone());
        }
    }

    /// Records a solution against the zone its epoch was measured in.
    fn record_solution(&mut self, output: &GnssOutput, truth: Pose2d, fused_heading: bool) {
        let measured_in = self.pending_zones.pop_front().unwrap_or(zone(truth));
        let sky = &mut self.sky[measured_in as usize];
        sky.solutions[label_index(output.label)] += 1;
        if let Some(fix) = output.fix {
            let error = (fix.x_m - truth.x_m).hypot(fix.y_m - truth.y_m);
            if output.label == SolutionLabel::Fixed {
                sky.fix_error.add(error);
            }
            self.fixes
                .push((Vec3::new(fix.x_m, fix.y_m, 0.0), output.label));
        }
        if let Some(heading) = output.heading {
            sky.headings += 1;
            let error = rne_nav::wrap_angle(heading.yaw_rad - truth.yaw_rad)
                .abs()
                .to_degrees();
            sky.heading_error_deg.add(error);
        }
        if fused_heading {
            self.headings_fused += 1;
        }
    }

    fn print_summary(
        &self,
        reached: bool,
        duration_s: f64,
        hybrid: &HybridLocalizer,
        base: Option<&GnssEpoch>,
    ) {
        let distance: f64 = self
            .samples
            .windows(2)
            .map(|w| planar_error(w[0].truth, w[1].truth))
            .sum();
        println!("\nGNSS outdoors / LiDAR indoors navigation");
        println!(
            "  goal {} after {:.1} s, {:.0} m driven, min LiDAR clearance {:.2} m",
            if reached { "reached" } else { "NOT reached" },
            duration_s,
            distance,
            self.clearance_m
        );
        if let Some(base) = base {
            println!(
                "  base station on the rooftop at ({:.0}, {:.0}, {:.1}) m tracks {} satellites",
                BASE_ENU_M.x,
                BASE_ENU_M.y,
                BASE_ENU_M.z,
                base.observations
                    .iter()
                    .filter(|o| o.path().is_tracked())
                    .count()
            );
        }

        println!(
            "\nRay-traced sky at the rear antenna (mean satellites per epoch, of 48 GPS+Galileo):"
        );
        println!(
            "  {:<17} {:>7} {:>5} {:>6} {:>5} {:>6} {:>7} {:>6}",
            "zone", "tracked", "LOS", "LOS+MP", "NLOS", "diffr", "blocked", "phase"
        );
        for zone in Zone::ALL {
            let s = &self.sky[zone as usize];
            if s.epochs == 0 {
                continue;
            }
            let n = s.epochs as f64;
            println!(
                "  {:<17} {:>7.1} {:>5.1} {:>6.1} {:>5.1} {:>6.1} {:>7.1} {:>6.1}",
                zone.label(),
                s.tracked as f64 / n,
                s.los as f64 / n,
                s.multipath as f64 / n,
                s.nlos as f64 / n,
                s.diffracted as f64 / n,
                s.blocked as f64 / n,
                s.phase_locked as f64 / n,
            );
        }

        println!("\nGNSS solutions per zone (% of epochs) and accuracy:");
        println!(
            "  {:<17} {:>6} {:>6} {:>6} {:>6} {:>13} {:>9} {:>14}",
            "zone",
            "fixed",
            "float",
            "single",
            "none",
            "fixed RMSE m",
            "heading %",
            "heading RMSE °"
        );
        for zone in Zone::ALL {
            let s = &self.sky[zone as usize];
            let total: usize = s.solutions.iter().sum();
            if total == 0 {
                continue;
            }
            let pct = |i: usize| 100.0 * s.solutions[i] as f64 / total as f64;
            println!(
                "  {:<17} {:>6.0} {:>6.0} {:>6.0} {:>6.0} {:>13.3} {:>9.0} {:>14.2}",
                zone.label(),
                pct(0),
                pct(1),
                pct(2),
                pct(3),
                s.fix_error.rmse(),
                100.0 * s.headings as f64 / total as f64,
                s.heading_error_deg.rmse(),
            );
        }

        println!("\nPosition error per zone, RMSE / max in meters:");
        println!(
            "  {:<17} {:>17} {:>17} {:>17}",
            "zone", "hybrid", "EKF, every fix", "odometry only"
        );
        for zone in Zone::ALL {
            let z = zone as usize;
            if self.hybrid[z].count == 0 {
                continue;
            }
            let cell = |s: &ErrorStats| format!("{:.2} / {:.2}", s.rmse(), s.max);
            println!(
                "  {:<17} {:>17} {:>17} {:>17}",
                zone.label(),
                cell(&self.hybrid[z]),
                cell(&self.gnss_only[z]),
                cell(&self.odom[z])
            );
        }

        println!("\nLocalization source switches:");
        for transition in hybrid.transitions() {
            let at = self
                .samples
                .iter()
                .find(|s| s.t >= transition.time_s)
                .map_or(Pose2d::IDENTITY, |s| s.truth);
            println!(
                "  t = {:6.1} s  at ({:6.1}, {:5.1})  {:?} -> {:?}",
                transition.time_s, at.x_m, at.y_m, transition.from, transition.to
            );
        }
        let mut time_in = [0.0; 3];
        for s in &self.samples {
            time_in[source_index(s.source)] += DT_S;
        }
        println!(
            "  time in GNSS {:.1} s, LiDAR {:.1} s, dead reckoning {:.1} s; {} moving-baseline headings fused",
            time_in[0], time_in[1], time_in[2], self.headings_fused
        );
        println!(
            "  longest dead-reckoning stretch {:.1} s; {} safety stops ({:.1} s stopped, limit {MAX_DEAD_RECKONING_S:.0} s)",
            self.longest_dead_reckoning_s, self.safety_stops, self.stopped_s
        );
        println!(
            "  {} stops for an obstacle within {STOP_DISTANCE_M} m ahead",
            self.obstacle_stops
        );
        println!(
            "  waited {} times ({:.1} s) at crossings for the pedestrian signal",
            self.crossing_waits, self.waiting_s
        );
    }
}

fn source_index(source: LocalizationSource) -> usize {
    match source {
        LocalizationSource::Gnss => 0,
        LocalizationSource::Lidar => 1,
        LocalizationSource::DeadReckoning => 2,
    }
}

// ---------------------------------------------------------------- SVG output

const MAP_VIEW: [f64; 4] = [-5.0, -68.0, 185.0, 40.0];
const PX_PER_M: f64 = 5.0;
const SOURCE_COLORS: [&str; 3] = ["#1a9850", "#2c7fb8", "#f46d43"];

fn to_px(x: f64, y: f64) -> (f64, f64) {
    let [x0, _, _, y1] = MAP_VIEW;
    ((x - x0) * PX_PER_M + 10.0, (y1 - y) * PX_PER_M + 40.0)
}

fn render_svg(scene: &GnssScene, ground: &[city::Patch], log: &RunLog, sky_epoch_s: f64) -> String {
    let [x0, y0, x1, y1] = MAP_VIEW;
    let map_w = (x1 - x0) * PX_PER_M + 20.0;
    let map_h = (y1 - y0) * PX_PER_M + 50.0;
    let sky_size = 330.0;
    let width = map_w;
    let height = map_h + sky_size + 60.0;
    let mut svg = String::new();
    let _ = writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0}" height="{height:.0}" viewBox="0 0 {width:.0} {height:.0}" font-family="Helvetica, Arial, sans-serif" font-size="12">"##
    );
    let _ = writeln!(
        svg,
        r##"<rect width="100%" height="100%" fill="#ffffff"/>"##
    );
    let _ = writeln!(
        svg,
        r##"<text x="10" y="24" font-size="16" font-weight="bold">GNSS outdoors (ray traced) / LiDAR indoors: top view</text>"##
    );

    draw_ground(&mut svg, ground);
    draw_buildings(&mut svg, scene);

    // Raw GNSS fixes.
    for (p, label) in log.fixes.iter().step_by(2) {
        let (px, py) = to_px(p.x, p.y);
        let color = match label {
            SolutionLabel::Fixed => "#1a9850",
            SolutionLabel::Float => "#fdae61",
            _ => "#d7301f",
        };
        let _ = writeln!(
            svg,
            r##"<circle cx="{px:.1}" cy="{py:.1}" r="1.3" fill="{color}" fill-opacity="0.6"/>"##
        );
    }

    // NLOS reflection paths at the snapshot epoch.
    if let Some(epoch) = &log.snapshot {
        let (ax, ay) = to_px(epoch.antenna_enu_m.x, epoch.antenna_enu_m.y);
        for o in &epoch.observations {
            let primary = o.primary();
            if let Some(r) = primary
                .filter(|p| p.path.is_indirect())
                .and_then(|p| p.interaction_point_m)
            {
                let (rx, ry) = to_px(r.x, r.y);
                let _ = writeln!(
                    svg,
                    r##"<line x1="{ax:.1}" y1="{ay:.1}" x2="{rx:.1}" y2="{ry:.1}" stroke="#d7301f" stroke-width="1" stroke-dasharray="3,2"/>"##
                );
            }
        }
        let _ = writeln!(
            svg,
            r##"<circle cx="{ax:.1}" cy="{ay:.1}" r="4" fill="none" stroke="#000" stroke-width="1.5"/>"##
        );
    }

    // Trajectories.
    let polyline = |svg: &mut String,
                    points: &[(f64, f64)],
                    color: &str,
                    width: f64,
                    dash: &str| {
        let mut d = String::new();
        for (x, y) in points {
            let (px, py) = to_px(*x, *y);
            let _ = write!(d, "{px:.1},{py:.1} ");
        }
        let _ = writeln!(
            svg,
            r##"<polyline points="{d}" fill="none" stroke="{color}" stroke-width="{width}" stroke-dasharray="{dash}"/>"##
        );
    };
    let stride = 4;
    let odom: Vec<(f64, f64)> = log
        .samples
        .iter()
        .step_by(stride)
        .map(|s| (s.odom.x_m, s.odom.y_m))
        .collect();
    polyline(&mut svg, &odom, "#fdae61", 1.2, "4,3");
    let gnss_only: Vec<(f64, f64)> = log
        .samples
        .iter()
        .step_by(stride)
        .map(|s| (s.gnss_only.x_m, s.gnss_only.y_m))
        .collect();
    polyline(&mut svg, &gnss_only, "#984ea3", 1.0, "2,2");
    let truth: Vec<(f64, f64)> = log
        .samples
        .iter()
        .step_by(stride)
        .map(|s| (s.truth.x_m, s.truth.y_m))
        .collect();
    polyline(&mut svg, &truth, "#000000", 2.5, "none");
    // Hybrid estimate, colored by source.
    let mut run: Vec<(f64, f64)> = Vec::new();
    let mut run_source = None;
    for s in log.samples.iter().step_by(stride) {
        if run_source.is_some_and(|src| src != s.source) {
            polyline(
                &mut svg,
                &run,
                SOURCE_COLORS[source_index(run_source.unwrap_or(s.source))],
                1.6,
                "none",
            );
            run = vec![*run.last().unwrap_or(&(s.hybrid.x_m, s.hybrid.y_m))];
        }
        run_source = Some(s.source);
        run.push((s.hybrid.x_m, s.hybrid.y_m));
    }
    if let Some(src) = run_source {
        polyline(
            &mut svg,
            &run,
            SOURCE_COLORS[source_index(src)],
            1.6,
            "none",
        );
    }

    // Legend.
    let legend = [
        ("#000000", "truth"),
        (SOURCE_COLORS[0], "hybrid: GNSS"),
        (SOURCE_COLORS[1], "hybrid: LiDAR AMCL"),
        (SOURCE_COLORS[2], "hybrid: dead reckoning"),
        ("#984ea3", "EKF fusing every GNSS fix"),
        ("#fdae61", "odometry only"),
        ("#1a9850", "GNSS solution: RTK fixed"),
        ("#d7301f", "GNSS solution: float / single"),
    ];
    for (i, (color, label)) in legend.iter().enumerate() {
        let lx = 14.0 + (i % 4) as f64 * 225.0;
        let ly = map_h + 2.0 + (i / 4) as f64 * 16.0;
        let _ = writeln!(
            svg,
            r##"<rect x="{lx:.0}" y="{:.0}" width="14" height="4" fill="{color}"/><text x="{:.0}" y="{:.0}">{label}</text>"##,
            ly - 4.0,
            lx + 20.0,
            ly + 1.0
        );
    }

    if let Some(epoch) = &log.snapshot {
        render_sky_plot(&mut svg, scene, epoch, map_h + 40.0, sky_size, sky_epoch_s);
    }
    svg.push_str("</svg>\n");
    svg
}

/// Carriageways, sidewalks, plaza and markings, clipped to the map view.
fn draw_ground(svg: &mut String, ground: &[city::Patch]) {
    let [x0, y0, x1, y1] = MAP_VIEW;
    for patch in ground {
        let [a, b, c, d] = patch.rect;
        let (a, b, c, d) = (a.max(x0), b.max(y0), c.min(x1), d.min(y1));
        if a >= c || b >= d {
            continue;
        }
        let fill = match patch.kind {
            "road" => "#8c8c8c",
            "sidewalk" => "#e3ded4",
            "plaza" | "forecourt" => "#ece7dc",
            "block" => "#d4d0c8",
            "floor" => "#f4f2ee",
            _ => "#ffffff",
        };
        let (px0, py1) = to_px(a, b);
        let (px1, py0) = to_px(c, d);
        let _ = write!(
            svg,
            r##"<rect x="{px0:.1}" y="{py0:.1}" width="{:.1}" height="{:.1}" fill="{fill}"/>"##,
            px1 - px0,
            py1 - py0
        );
    }
    svg.push('\n');
}

fn draw_buildings(svg: &mut String, scene: &GnssScene) {
    let [x0, y0, x1, y1] = MAP_VIEW;
    let (cx0, cy0) = to_px(x0, y1);
    let (cx1, cy1) = to_px(x1, y0);
    let _ = writeln!(
        svg,
        r##"<defs><clipPath id="map-clip"><rect x="{cx0:.1}" y="{cy0:.1}" width="{:.1}" height="{:.1}"/></clipPath></defs><g clip-path="url(#map-clip)">"##,
        cx1 - cx0,
        cy1 - cy0
    );
    // Lower boxes first so towers draw over their podiums; darker = taller.
    let mut boxes: Vec<_> = scene
        .boxes
        .iter()
        .filter(|b| b.min_m.z <= 1.0 || b.max_m.z > 20.0)
        .collect();
    boxes.sort_by(|a, b| a.max_m.z.total_cmp(&b.max_m.z));
    for b in boxes {
        let height_m = b.max_m.z;
        let area = (b.max_m.x - b.min_m.x) * (b.max_m.y - b.min_m.y);
        let shade = if !b.material.is_reflective() {
            "#525252"
        } else if height_m < 1.0 {
            "#d9d9d9"
        } else if height_m < 5.0 {
            "#969696"
        } else if b.material.reflection_loss_db < 5.0 {
            "#9ecae1"
        } else {
            "#bdbdbd"
        };
        let (px0, py1) = to_px(b.min_m.x, b.min_m.y);
        let (px1, py0) = to_px(b.max_m.x, b.max_m.y);
        let _ = writeln!(
            svg,
            r##"<rect x="{px0:.1}" y="{py0:.1}" width="{:.1}" height="{:.1}" fill="{shade}" stroke="#636363" stroke-width="0.5"/>"##,
            px1 - px0,
            py1 - py0
        );
        if height_m >= 9.0 && area >= 150.0 {
            let _ = writeln!(
                svg,
                r##"<text x="{:.1}" y="{:.1}" text-anchor="middle" fill="#252525">{height_m:.0} m</text>"##,
                0.5 * (px0 + px1),
                0.5 * (py0 + py1) + 4.0
            );
        }
    }
    svg.push_str("</g>\n");
    let (ox, oy) = to_px(OFFICE[0], OFFICE[3]);
    let _ = writeln!(
        svg,
        r##"<text x="{:.1}" y="{:.1}" fill="#252525">office (roofed, 3.7 m)</text>"##,
        ox,
        oy - 6.0
    );
}

fn render_sky_plot(
    svg: &mut String,
    scene: &GnssScene,
    epoch: &GnssEpoch,
    top: f64,
    size: f64,
    sky_epoch_s: f64,
) {
    let radius = 0.5 * size - 20.0;
    let (cx, cy) = (20.0 + 0.5 * size, top + 0.5 * size);
    let polar = |az: f64, el: f64| {
        let r = radius * (1.0 - el / FRAC_PI_2);
        (cx + r * az.sin(), cy - r * az.cos())
    };
    let _ = writeln!(
        svg,
        r##"<text x="20" y="{:.0}" font-size="14" font-weight="bold">Sky at the circled point on the main street (t = {:.0} s)</text>"##,
        top - 4.0,
        epoch.time_s - sky_epoch_s
    );
    // Building skyline: highest blocked elevation per azimuth.
    let antenna = epoch.antenna_enu_m;
    let mut skyline = String::new();
    for step in 0..=180 {
        let az = TAU * step as f64 / 180.0;
        let (mut lo, mut hi) = (0.0, FRAC_PI_2);
        let dir = |el: f64| Vec3::new(az.sin() * el.cos(), az.cos() * el.cos(), el.sin());
        if scene.occluded(antenna, dir(0.001), f64::INFINITY) {
            for _ in 0..24 {
                let mid = 0.5 * (lo + hi);
                if scene.occluded(antenna, dir(mid), f64::INFINITY) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
        }
        let (px, py) = polar(az, lo);
        let _ = write!(skyline, "{px:.1},{py:.1} ");
    }
    let _ = writeln!(
        svg,
        r##"<circle cx="{cx:.1}" cy="{cy:.1}" r="{radius:.1}" fill="#bdbdbd"/><polygon points="{skyline}" fill="#f7fbff" stroke="#636363" stroke-width="0.8"/>"##
    );
    for el_deg in [30.0_f64, 60.0] {
        let r = radius * (1.0 - el_deg / 90.0);
        let _ = writeln!(
            svg,
            r##"<circle cx="{cx:.1}" cy="{cy:.1}" r="{r:.1}" fill="none" stroke="#969696" stroke-dasharray="2,3"/><text x="{:.1}" y="{:.1}" fill="#636363" font-size="10">{el_deg:.0}°</text>"##,
            cx + 2.0,
            cy - r - 2.0
        );
    }
    for (label, az) in [("N", 0.0), ("E", FRAC_PI_2), ("S", PI_), ("W", 1.5 * PI_)] {
        let (px, py) = polar(az, -0.12);
        let _ = writeln!(
            svg,
            r##"<text x="{px:.1}" y="{:.1}" text-anchor="middle" font-weight="bold">{label}</text>"##,
            py + 4.0
        );
    }
    let mut counts = [0usize; 5];
    for o in &epoch.observations {
        if o.path() == SignalPath::BelowMask && o.elevation_rad < 0.0 {
            continue;
        }
        let (color, index) = match o.path() {
            SignalPath::LineOfSight => ("#1a9850", 0),
            SignalPath::LineOfSightMultipath => ("#fee08b", 1),
            SignalPath::NonLineOfSight => ("#d7301f", 2),
            SignalPath::Diffracted => ("#9e9ac8", 3),
            _ => ("#737373", 4),
        };
        counts[index] += 1;
        let (px, py) = polar(o.azimuth_rad, o.elevation_rad.max(0.0));
        let _ = writeln!(
            svg,
            r##"<circle cx="{px:.1}" cy="{py:.1}" r="8" fill="{color}" stroke="#252525" stroke-width="0.8"/><text x="{px:.1}" y="{:.1}" text-anchor="middle" font-size="8">{}</text>"##,
            py + 3.0,
            o.id
        );
    }
    let lx = 40.0 + size;
    let rows = [
        ("#1a9850", format!("line of sight ({})", counts[0])),
        (
            "#fee08b",
            format!("LOS + reflected replica ({})", counts[1]),
        ),
        (
            "#d7301f",
            format!("NLOS: only a reflection arrives ({})", counts[2]),
        ),
        (
            "#9e9ac8",
            format!("diffracted over a building edge ({})", counts[3]),
        ),
        (
            "#737373",
            format!("blocked / too weak / below mask ({})", counts[4]),
        ),
        ("#f7fbff", "open sky above the building skyline".to_string()),
        ("#bdbdbd", "sky masked by buildings".to_string()),
    ];
    for (i, (color, label)) in rows.iter().enumerate() {
        let ly = top + 30.0 + i as f64 * 22.0;
        let _ = writeln!(
            svg,
            r##"<circle cx="{lx:.0}" cy="{ly:.0}" r="7" fill="{color}" stroke="#252525" stroke-width="0.8"/><text x="{:.0}" y="{:.0}">{label}</text>"##,
            lx + 14.0,
            ly + 4.0
        );
    }
    let _ = writeln!(
        svg,
        r##"<text x="{lx:.0}" y="{:.0}" fill="#636363">Dashed red lines in the top view: antenna to the first</text><text x="{lx:.0}" y="{:.0}" fill="#636363">wall point or edge of each NLOS / diffracted signal.</text>"##,
        top + 30.0 + 7.0 * 22.0 + 10.0,
        top + 30.0 + 7.0 * 22.0 + 26.0
    );
}

const PI_: f64 = std::f64::consts::PI;
