//! Navigates the welded Go2 between two rooms on what its Mid-360 sees: online
//! 2D SLAM from the recording-matched sensor model, A* on an inflated costmap,
//! and a lookahead path follower into the model-based trot. No map is given in advance.
//!
//! What the robot uses, per 0.1 s Mid-360 frame:
//!
//! * **Points in the sensor frame.** Each return is expressed in the sensor
//!   frame at its own emission time, which is what the real driver publishes.
//! * **IMU attitude.** Roll and pitch level the points, as an IMU's gravity
//!   estimate does; heading is never taken from the simulation.
//! * **Leg odometry.** Body velocity and yaw rate integrated with a 4 % scale
//!   error and a 0.02 rad/s yaw-rate bias, so dead reckoning drifts. It also
//!   de-skews each frame to the pose at the frame's end.
//! * **A 2D scan** of the returns 0.15–0.65 m above the floor, 720 beams.
//!
//! `rne_slam::Slam2d` builds the occupancy map and corrects the odometry by scan
//! matching on keyframes (every 0.15 m or 0.15 rad). Planning treats unknown space as traversable and replans every
//! second, so the robot heads for a goal it cannot see yet and routes through
//! the doorway once the partition shows up in the map.
//!
//! The gate compares the estimate with the simulation's truth: both goals
//! reached, localization error, clearance, and that the map has the walls.
//!
//! With `--explore` no goals are given: `rne_explore` picks the next frontier
//! from the SLAM map each second until nothing reachable is left unknown
//! (`docs/PLAN_ACTIVE_SLAM.md` P3). That gate asks for both rooms mapped
//! instead of goals reached.
//!
//! ```text
//! cargo run --release -p go2_mid360_navigation --example 131_go2_mid360_navigation -- --smoke
//! cargo run --release -p go2_mid360_navigation --example 131_go2_mid360_navigation
//! cargo run --release -p go2_mid360_navigation --example 131_go2_mid360_navigation -- --explore --smoke
//! cargo run --release -p go2_mid360_navigation --example 131_go2_mid360_navigation -- --explore
//! ```

mod guard;
mod layouts;

use std::f64::consts::PI;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use go2_indoor::{
    clearance_m, static_obstacles, two_room_floors, DecorOptions, Finish, Interior, StaticObject,
};
use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, unitree_go2_mid360_mount, UnitreeGo2ModelTrot,
    UnitreeGo2TrotCommand, UrdfSceneSim, UNITREE_GO2_MID360_FORWARD_OF_BASE_M,
    UNITREE_GO2_MODEL_TROT_CONTROL_HZ,
};
use rne_data::PointCloud;
use rne_explore::{Decision, Explorer, ExplorerConfig, FrontierConfig, ScorePolicy, SensorModel};
use rne_math::{Quat, Transform3, Vec3};
use rne_nav::{
    integrate_scan, plan_path, Costmap, CostmapConfig, FrameId, GlobalPlannerConfig, GridCoord,
    LaserScan2d, OccupancyGrid, Path2d, Pose2d, ScanIntegrationConfig, COST_INSCRIBED, COST_LETHAL,
};
use rne_render::{
    Camera, MeshRenderCache, RenderBackend, RenderScene, RenderSceneItem, TriangleMesh, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_sensor::{
    livox_mid360_spec, LidarRigOcclusion, LidarSpec, LidarSweep, LivoxMid360Pattern, SensorNoiseKey,
};
use rne_slam::{Slam2d, SlamConfig};
use rne_world::Transform3 as WorldTransform3;

/// Goals in world `[x, z]`: the far corner of room B, then back into room A.
const GOALS: [[f64; 2]; 2] = [[6.2, 1.6], [-1.5, -0.3]];
const MAX_DURATION_S: f64 = 180.0;
/// Exploration gets longer: it has to find the rooms before it can cross them.
const EXPLORE_MAX_DURATION_S: f64 = 300.0;
/// Range the explorer credits the Mid-360's scan band with when it scores a
/// frontier by the unknown area it would see. The band reaches 20 m, but the
/// rooms are under 5 m across.
const EXPLORE_SENSOR_RANGE_M: f64 = 5.0;
/// Floor counted as mapped: true floor at least this far from every obstacle,
/// on a grid of this pitch.
const COVERAGE_CLEARANCE_M: f64 = 0.10;
const COVERAGE_PITCH_M: f64 = 0.10;
/// Exploration plans the base this far off mapped obstacles. The body is
/// kept clear by the footprint guard (`guard.rs`), so this only has to leave
/// paths a disc can follow without squeezing the guard: 0.42 m closed the
/// 1.0 m doorway once its jambs fell on 5 cm cells, 0.38 m leaves 0.24 m.
const EXPLORE_INSCRIBED_RADIUS_M: f64 = 0.38;
/// Weight of the inflation cost in the exploration planner. At the default
/// 0.5 a path runs along the inscribed band, and turning there brought a foot
/// within 1–2 cm of furniture and the trot fell; at 3 to 5 paths keep to the
/// middle of the free space.
const EXPLORE_COST_WEIGHT: f64 = 4.0;
/// Links whose origins bound the Go2's footprint: head, hips, knees and feet.
/// Their distance to the obstacles' footprints, sampled at 50 Hz, says how
/// close the body came; link radii (2–5 cm) are not subtracted.
const BODY_LINKS: [&str; 13] = [
    "Head_upper",
    "FL_hip",
    "FR_hip",
    "RL_hip",
    "RR_hip",
    "FL_calf",
    "FR_calf",
    "RL_calf",
    "RR_calf",
    "FL_foot",
    "FR_foot",
    "RL_foot",
    "RR_foot",
];
const BODY_CLEARANCE_EVERY_STEPS: u64 = 10;
/// Each room's floor must be at least this much mapped.
const EXPLORE_MIN_ROOM_COVERAGE: f64 = 0.90;
/// Path following.
const LOOKAHEAD_M: f64 = 0.6;
const MAX_SPEED_M_S: f64 = 0.25;
const MAX_YAW_RATE_RAD_S: f64 = 0.5;
const HEADING_GAIN_PER_S: f64 = 1.5;
const SLOW_RADIUS_M: f64 = 0.6;
const GOAL_TOLERANCE_M: f64 = 0.25;
/// Control steps per Mid-360 frame: 0.1 s at 500 Hz.
const STEPS_PER_LIDAR_FRAME: u64 = 50;
const REPLAN_EVERY_FRAMES: u64 = 10;
/// SLAM keyframes: a scan is processed once the odometry has moved this far.
const KEYFRAME_TRANSLATION_M: f64 = 0.15;
const KEYFRAME_ROTATION_RAD: f64 = 0.15;

/// Leg odometry errors: speed scale and yaw-rate bias. Chosen, not measured.
const ODOM_SCALE_ERROR: f64 = 0.04;
const ODOM_YAW_RATE_BIAS_RAD_S: f64 = 0.02;

/// Scan band relative to the sensor, which rides 0.447 m above the floor.
const SCAN_MIN_HEIGHT_M: f64 = -0.30;
const SCAN_MAX_HEIGHT_M: f64 = 0.20;
/// The Mid-360's height above the floor on the standing Go2.
const MID360_HEIGHT_M: f64 = 0.447;
/// Heights above the floor the Go2's body and legs can hit: returns there
/// mark the obstacle layer.
const COLLISION_BAND_M: [f64; 2] = [0.10, 0.55];
/// Returns below this height above the floor are floor: their rays clear
/// the obstacle layer.
const FLOOR_RETURN_M: f64 = 0.05;
/// The footprint guard checks the follower's command at 20 Hz and holds a
/// replacement for 0.5 s.
const GUARD_EVERY_STEPS: u64 = 25;
const GUARD_HOLD_S: f64 = 0.5;
/// With `--explore`, commands change at most this fast.
const MAX_ACCELERATION_M_S2: f64 = 0.5;
const MAX_YAW_ACCELERATION_RAD_S2: f64 = 1.0;
/// A frontier is given up when the robot moves less than this in this long
/// while heading for it, and the cells within this radius of it are filled.
const STUCK_DISTANCE_M: f64 = 0.25;
const STUCK_TIME_S: f64 = 10.0;
const ABANDON_RADIUS_M: f64 = 0.3;
/// Log-odds given to obstacle-layer cells copied into the navigation map.
const OBSTACLE_LOG_ODDS: f64 = 5.0;
/// Returns closer than this are the robot itself.
const SCAN_MIN_RANGE_M: f64 = 0.40;
const SCAN_MAX_RANGE_M: f64 = 20.0;
const SCAN_BEAMS: usize = 720;

/// Map extent in the navigation frame (x forward, y = -world z), 5 cm cells.
const MAP_ORIGIN: [f64; 2] = [-3.0, -3.0];
const MAP_SIZE_M: [f64; 2] = [10.5, 6.0];
const MAP_RESOLUTION_M: f64 = 0.05;

const WIDTH: u32 = 960;
const HEIGHT: u32 = 540;
const CLEAR_COLOR: [f32; 4] = [0.035, 0.05, 0.08, 1.0];
const LIDAR_FRAMES_PER_GIF_FRAME: u64 = 8;
const COLORMAP_BUCKETS: usize = 16;
const CUTAWAY_HEIGHT_M: f64 = 0.12;

fn repo_path(relative: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join(relative)
}

/// World `[x, z]` to the navigation frame's `(x, y)`.
fn nav_xy(world: [f64; 2]) -> (f64, f64) {
    (world[0], -world[1])
}

/// Yaw of a world rotation about +y, zero along +x, positive turning left.
fn yaw_of(rotation: Quat) -> f64 {
    let facing = rotation * Vec3::X;
    (-facing.z).atan2(facing.x)
}

fn wrap_angle(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

struct Nav {
    sim: UrdfSceneSim,
    trot: UnitreeGo2ModelTrot,
    spec: LidarSpec,
    rig: LidarRigOcclusion,
    pattern: LivoxMid360Pattern,
    frame_index: u64,
    frame_start_pose: WorldTransform3,
    slam: Slam2d,
    /// Dead-reckoned base pose from leg odometry.
    odom: Pose2d,
    odom_at_frame_start: Pose2d,
    /// Latest SLAM pose and the odometry it was computed at.
    slam_pose: Pose2d,
    odom_at_slam: Pose2d,
    path: Option<Path2d>,
    /// Present with `--explore`: chooses frontiers instead of [`GOALS`].
    explorer: Option<Explorer>,
    /// Set each frame with `--explore`: distance to obstacles on the
    /// navigation map, for the footprint guard.
    clearance: Option<guard::ClearanceField>,
    /// The guard's replacement command and until when it holds.
    guard_hold: Option<(f64, UnitreeGo2TrotCommand)>,
    /// How often the guard replaced the follower's command.
    guard_overrides: usize,
    /// The command sent last, for the slew limit.
    last_command: UnitreeGo2TrotCommand,
    /// When and where the robot last made progress toward a goal.
    progress_anchor: (f64, Pose2d),
    /// Frontier goals given up on (see [`Nav::watch_progress`]).
    abandoned: Vec<Vec3>,
    /// Present with `--explore`: obstacles at the Go2's height, kept apart
    /// from the SLAM map (see [`Nav::update_obstacle_layer`]).
    obstacle_layer: Option<OccupancyGrid>,
    /// Set once the explorer finds nothing reachable left to explore.
    explored: bool,
    /// Frontier goals reached, and how often no path to one was found.
    frontier_goals: usize,
    plan_failures: usize,
    goal: usize,
    goal_true_error_m: Vec<f64>,
    frames_since_plan: u64,
    last_cloud: Option<PointCloud>,
    true_trail: Vec<[f64; 2]>,
    slam_error_sq: f64,
    slam_error_max_m: f64,
    odom_error_max_m: f64,
    error_samples: u64,
    min_clearance_m: f64,
    /// Closest any of [`BODY_LINKS`] came to an obstacle's footprint.
    min_body_clearance_m: f64,
    /// Where that was: link, obstacle, time.
    closest_approach: (&'static str, String, f64),
    min_height_m: f64,
    obstacles: Vec<StaticObject>,
    matched_frames: u64,
    loop_closures: usize,
}

impl Nav {
    fn new(explore: bool, scene: &Path) -> Self {
        let mut sim = UrdfSceneSim::from_scene_path(scene).expect("load two-room scene");
        // Ease each walk in over a second: a full-speed first step can tip
        // the 16 kg Go2 into a stumble.
        let trot = UnitreeGo2ModelTrot::stand_up(&mut sim).with_command_ease_in_s(1.0);
        let rig: LidarRigOcclusion = serde_json::from_str(
            &fs::read_to_string(repo_path(
                "assets/sensors/livox_mid360/go2_rig_occlusion.json",
            ))
            .expect("read rig table"),
        )
        .expect("parse rig table");
        let frame_start_pose = sim
            .named_mount_transform("base", &unitree_go2_mid360_mount())
            .expect("mount pose");
        let grid = OccupancyGrid::new(
            (MAP_SIZE_M[0] / MAP_RESOLUTION_M) as usize,
            (MAP_SIZE_M[1] / MAP_RESOLUTION_M) as usize,
            MAP_RESOLUTION_M,
            Pose2d::new(MAP_ORIGIN[0], MAP_ORIGIN[1], 0.0),
        )
        .expect("map grid");
        let obstacle_layer = explore.then(|| grid.clone());
        // Odometry starts at the true start pose, so the map frame is the world's.
        let start = true_pose(&sim);
        let obstacles = static_obstacles(sim.world());
        Self {
            sim,
            trot,
            spec: livox_mid360_spec(),
            rig,
            pattern: LivoxMid360Pattern::new(),
            frame_index: 0,
            frame_start_pose,
            slam: Slam2d::new(grid, slam_config()),
            odom: start,
            odom_at_frame_start: start,
            slam_pose: start,
            odom_at_slam: start,
            path: None,
            explorer: explore.then(|| Explorer::new(explorer_config())),
            obstacle_layer,
            clearance: None,
            guard_hold: None,
            guard_overrides: 0,
            last_command: UnitreeGo2TrotCommand::default(),
            progress_anchor: (0.0, start),
            abandoned: Vec::new(),
            explored: false,
            frontier_goals: 0,
            plan_failures: 0,
            goal: 0,
            goal_true_error_m: Vec::new(),
            frames_since_plan: REPLAN_EVERY_FRAMES,
            last_cloud: None,
            true_trail: Vec::new(),
            slam_error_sq: 0.0,
            slam_error_max_m: 0.0,
            odom_error_max_m: 0.0,
            error_samples: 0,
            min_clearance_m: f64::MAX,
            min_body_clearance_m: f64::MAX,
            closest_approach: ("", String::new(), 0.0),
            min_height_m: f64::MAX,
            obstacles,
            matched_frames: 0,
            loop_closures: 0,
        }
    }

    fn done(&self) -> bool {
        if self.explorer.is_some() {
            self.explored || self.trot.time_s() >= EXPLORE_MAX_DURATION_S
        } else {
            self.goal >= GOALS.len() || self.trot.time_s() >= MAX_DURATION_S
        }
    }

    /// The robot's best pose estimate now: the last SLAM pose moved by the
    /// odometry since.
    fn estimate(&self) -> Pose2d {
        self.slam_pose
            .compose(self.odom_at_slam.inverse().compose(self.odom))
    }

    fn step_frame(&mut self) {
        for _ in 0..STEPS_PER_LIDAR_FRAME {
            self.step_control();
        }
        let end_pose = self
            .sim
            .named_mount_transform("base", &unitree_go2_mid360_mount())
            .expect("mount pose");
        let sweep = LidarSweep::new(self.frame_start_pose, end_pose);
        let cloud = self.sim.sample_livox_mid360(
            &sweep,
            &self.spec,
            &self.pattern,
            self.frame_index,
            Some(&self.rig),
            SensorNoiseKey::new(self.sim.world_seed(), self.spec.seed, 1, self.frame_index),
        );
        let returns = self.levelled_returns(&cloud, &sweep);
        let scan = self.build_scan(&returns);
        let since_slam = self.odom_at_slam.inverse().compose(self.odom);
        let keyframe = self.slam.scans_processed() == 0
            || since_slam.x_m.hypot(since_slam.y_m) >= KEYFRAME_TRANSLATION_M
            || since_slam.yaw_rad.abs() >= KEYFRAME_ROTATION_RAD;
        if keyframe {
            self.process_keyframe(&scan);
        }
        self.update_obstacle_layer(&returns);
        if self.explorer.is_some() {
            self.clearance = Some(guard::ClearanceField::from_grid(&self.navigation_grid()));
        }
        self.odom_at_frame_start = self.odom;
        self.frame_start_pose = end_pose;
        self.last_cloud = Some(cloud);
        self.frame_index += 1;

        let truth = true_pose(&self.sim);
        let estimate = self.estimate();
        let slam_error = (truth.x_m - estimate.x_m).hypot(truth.y_m - estimate.y_m);
        let odom_error = (truth.x_m - self.odom.x_m).hypot(truth.y_m - self.odom.y_m);
        self.slam_error_sq += slam_error * slam_error;
        self.slam_error_max_m = self.slam_error_max_m.max(slam_error);
        self.odom_error_max_m = self.odom_error_max_m.max(odom_error);
        self.error_samples += 1;

        self.frames_since_plan += 1;
        if self.frames_since_plan >= REPLAN_EVERY_FRAMES {
            self.replan();
        }
    }

    fn process_keyframe(&mut self, scan: &LaserScan2d) {
        let sensor_from_base = Pose2d::new(UNITREE_GO2_MID360_FORWARD_OF_BASE_M, 0.0, 0.0);
        let update = self
            .slam
            .process(scan, self.odom, sensor_from_base)
            .expect("slam update");
        if update.matched {
            self.matched_frames += 1;
        }
        self.loop_closures += update.loop_closures;
        self.slam_pose = update.pose;
        self.odom_at_slam = self.odom;
    }

    /// Each return levelled by the IMU attitude and de-skewed to the sensor
    /// pose at the frame end: its planar position there, its height relative
    /// to the sensor, and its planar range at emission (for the self filter).
    fn levelled_returns(&self, cloud: &PointCloud, sweep: &LidarSweep) -> Vec<LevelledReturn> {
        let frame_motion = self.odom_at_frame_start.inverse().compose(self.odom);
        let sensor_from_base = Pose2d::new(UNITREE_GO2_MID360_FORWARD_OF_BASE_M, 0.0, 0.0);
        let mut out = Vec::with_capacity(cloud.points_m.len());
        for (point, time_s) in cloud.points_m.iter().zip(cloud.timestamps_s.iter()) {
            let fraction = (time_s / self.spec.rotation_period_s).clamp(0.0, 1.0);
            // What the driver reports: the return in the sensor frame at emission.
            let sensor = sweep.pose_at(fraction);
            let raw = sensor.rotation.inverse() * (*point - sensor.translation);
            // IMU attitude: remove roll and pitch, keep the sensor's own heading.
            let yaw = yaw_of(sensor.rotation);
            let tilt = Quat::from_rotation_y(yaw).inverse() * sensor.rotation;
            let level = tilt * raw;
            // Leveled axes: +x forward, +y left.
            let at_emission = Vec3::new(level.x, -level.z, 0.0);
            // De-skew with odometry: the base pose at emission, relative to the
            // frame end, interpolated along the frame's odometry motion.
            let partial = Pose2d::new(
                frame_motion.x_m * fraction,
                frame_motion.y_m * fraction,
                frame_motion.yaw_rad * fraction,
            );
            let emission_from_end = frame_motion.inverse().compose(partial);
            let base_point = sensor_from_base.transform_point(at_emission);
            let at_end = sensor_from_base
                .inverse()
                .transform_point(emission_from_end.transform_point(base_point));
            out.push(LevelledReturn {
                at_end,
                height_m: level.y,
                emission_range_m: at_emission.length(),
            });
        }
        out
    }

    /// Turns a frame's returns into a 2D scan at the sensor's pose at the frame end.
    fn build_scan(&self, returns: &[LevelledReturn]) -> LaserScan2d {
        let mut ranges = vec![f64::NAN; SCAN_BEAMS];
        for point in returns {
            if !(SCAN_MIN_HEIGHT_M..=SCAN_MAX_HEIGHT_M).contains(&point.height_m)
                || point.emission_range_m < SCAN_MIN_RANGE_M
            {
                continue;
            }
            let (beam, range) = point.beam();
            if ranges[beam].is_nan() || range < ranges[beam] {
                ranges[beam] = range;
            }
        }
        level_scan(self.trot.time_s(), ranges)
    }

    /// Updates the obstacle layer from a frame: the nearest return at a
    /// height the Go2 can hit marks a cell, and rays that reach the floor
    /// clear the cells they cross, up to the nearest such return.
    fn update_obstacle_layer(&mut self, returns: &[LevelledReturn]) {
        let sensor_pose =
            self.estimate()
                .compose(Pose2d::new(UNITREE_GO2_MID360_FORWARD_OF_BASE_M, 0.0, 0.0));
        let time_s = self.trot.time_s();
        let Some(layer) = self.obstacle_layer.as_mut() else {
            return;
        };
        let mut nearest = vec![f64::NAN; SCAN_BEAMS];
        let mut floor = vec![f64::NAN; SCAN_BEAMS];
        for point in returns {
            if point.emission_range_m < SCAN_MIN_RANGE_M {
                continue;
            }
            let (beam, range) = point.beam();
            let above_floor = point.height_m + MID360_HEIGHT_M;
            if (COLLISION_BAND_M[0]..=COLLISION_BAND_M[1]).contains(&above_floor) {
                if nearest[beam].is_nan() || range < nearest[beam] {
                    nearest[beam] = range;
                }
            } else if above_floor < FLOOR_RETURN_M && (floor[beam].is_nan() || range > floor[beam])
            {
                floor[beam] = range;
            }
        }
        let clear: Vec<f64> = floor
            .iter()
            .zip(&nearest)
            .map(|(&f, &n)| {
                if n.is_finite() {
                    f.min(n - MAP_RESOLUTION_M)
                } else {
                    f
                }
            })
            .collect();
        integrate_scan(
            layer,
            &level_scan(time_s, clear),
            sensor_pose,
            &ScanIntegrationConfig {
                occupied_log_odds: 0.0,
                ..ScanIntegrationConfig::default()
            },
        )
        .expect("clear obstacle layer");
        integrate_scan(
            layer,
            &level_scan(time_s, nearest),
            sensor_pose,
            &ScanIntegrationConfig {
                mark_free: false,
                ..ScanIntegrationConfig::default()
            },
        )
        .expect("mark obstacle layer");
    }

    /// Gives up the current frontier when the robot has not moved
    /// [`STUCK_DISTANCE_M`] in [`STUCK_TIME_S`] while heading for it: the
    /// planner's path fits a disc, but the footprint guard found no way for
    /// the body through, and the explorer would choose the same frontier
    /// again forever.
    fn watch_progress(&mut self, estimate: Pose2d) {
        let now_s = self.trot.time_s();
        let (since_s, anchor) = self.progress_anchor;
        let moved = (estimate.x_m - anchor.x_m).hypot(estimate.y_m - anchor.y_m);
        if self.path.is_none() || moved >= STUCK_DISTANCE_M {
            self.progress_anchor = (now_s, estimate);
            return;
        }
        if now_s - since_s < STUCK_TIME_S {
            return;
        }
        if let Some(explorer) = self.explorer.as_mut() {
            if let Some(goal) = explorer.current() {
                self.abandoned
                    .push(Vec3::new(goal.goal.x_m, goal.goal.y_m, 0.0));
            }
            explorer.clear_goal();
        }
        self.path = None;
        self.progress_anchor = (now_s, estimate);
    }

    /// The map the explorer chooses on: the navigation map with the
    /// frontiers given up on filled in, so they are no longer frontiers.
    fn exploration_grid(&self) -> OccupancyGrid {
        let mut grid = self.navigation_grid();
        let reach = (ABANDON_RADIUS_M / grid.resolution_m()).ceil() as isize;
        for goal in &self.abandoned {
            let Some(centre) = grid.world_to_grid(*goal) else {
                continue;
            };
            for dy in -reach..=reach {
                for dx in -reach..=reach {
                    let cell = GridCoord {
                        x: centre.x + dx,
                        y: centre.y + dy,
                    };
                    let at = grid.grid_to_world(cell);
                    if (at - *goal).length() <= ABANDON_RADIUS_M && !grid.is_occupied(cell) {
                        grid.reset(cell);
                        grid.apply_occupied(cell, OBSTACLE_LOG_ODDS);
                    }
                }
            }
        }
        grid
    }

    /// The map exploration plans on: the SLAM map with the obstacle layer's
    /// occupied cells added.
    fn navigation_grid(&self) -> OccupancyGrid {
        let mut grid = self.slam.grid().clone();
        if let Some(layer) = &self.obstacle_layer {
            for y in 0..layer.height() {
                for x in 0..layer.width() {
                    let cell = GridCoord {
                        x: x as isize,
                        y: y as isize,
                    };
                    if layer.is_occupied(cell) && !grid.is_occupied(cell) {
                        grid.reset(cell);
                        grid.apply_occupied(cell, OBSTACLE_LOG_ODDS);
                    }
                }
            }
        }
        grid
    }

    fn replan(&mut self) {
        if self.explorer.is_some() {
            self.replan_explore();
            return;
        }
        let Some(goal) = GOALS.get(self.goal) else {
            return;
        };
        self.frames_since_plan = 0;
        let costmap =
            Costmap::from_occupancy(self.slam.grid(), &costmap_config()).expect("costmap");
        let estimate = self.estimate();
        let (gx, gy) = nav_xy(*goal);
        let planner = GlobalPlannerConfig {
            allow_unknown: true,
            ..GlobalPlannerConfig::default()
        };
        if let Ok(path) = plan_path(
            &costmap,
            Vec3::new(estimate.x_m, estimate.y_m, 0.0),
            Vec3::new(gx, gy, 0.0),
            &planner,
        ) {
            self.path = Some(path.prune_collinear(0.02));
        }
    }

    /// Asks the explorer for a frontier and follows the path it found there,
    /// through known free space only (unknown space is what is being mapped).
    fn replan_explore(&mut self) {
        self.frames_since_plan = 0;
        let estimate = self.estimate();
        self.watch_progress(estimate);
        let navigation = self.exploration_grid();
        let grid = &navigation;
        let Some(explorer) = self.explorer.as_mut() else {
            return;
        };
        match explorer.decide(grid, estimate).expect("explore decision") {
            Decision::Done => {
                self.explored = true;
                self.path = None;
            }
            Decision::Goal(candidate) | Decision::Revisit(candidate) => {
                if candidate.path_m < GOAL_TOLERANCE_M {
                    // Already there: count it and choose again next frame.
                    explorer.clear_goal();
                    self.frontier_goals += 1;
                    self.path = None;
                    self.frames_since_plan = REPLAN_EVERY_FRAMES;
                    return;
                }
                // Cost-aware A* keeps the trot off walls; the inscribed band is
                // made lethal (except right around the robot, so it can back
                // out) because `plan_path` only refuses lethal cells. The
                // explorer's own shortest path, which hugs that band, is the
                // fallback.
                let planned = plan_path(
                    &explore_planning_costmap(grid, estimate),
                    Vec3::new(estimate.x_m, estimate.y_m, 0.0),
                    Vec3::new(candidate.goal.x_m, candidate.goal.y_m, 0.0),
                    &GlobalPlannerConfig {
                        cost_weight: EXPLORE_COST_WEIGHT,
                        ..GlobalPlannerConfig::default()
                    },
                )
                .ok()
                .or_else(|| {
                    explorer
                        .path_to(grid, candidate.goal_cell)
                        .filter(|cells| cells.len() >= 2)
                        .map(|mut cells| {
                            cells[0] = Vec3::new(estimate.x_m, estimate.y_m, 0.0);
                            Path2d::from_points(&cells)
                        })
                });
                match planned {
                    Some(path) => self.path = Some(path.prune_collinear(0.02)),
                    None => {
                        explorer.clear_goal();
                        self.plan_failures += 1;
                        self.path = None;
                    }
                }
            }
        }
    }

    fn step_control(&mut self) {
        let mut command = self.follow();
        if self.explorer.is_some() {
            command = self.slew_limited(command);
        }
        self.trot.step(&mut self.sim, command);
        self.integrate_odometry();
        let observed = self.sim.observe();
        if self.trot.steps().is_multiple_of(125) {
            self.true_trail.push([observed.base_x_m, observed.base_z_m]);
        }
        self.min_height_m = self.min_height_m.min(observed.base_y_m);
        if self.trot.steps().is_multiple_of(BODY_CLEARANCE_EVERY_STEPS) {
            for link in BODY_LINKS {
                let at = self.sim.named_transform(link).expect("Go2 link");
                let position = [at.translation.x, at.translation.z];
                for obstacle in &self.obstacles {
                    let clearance = obstacle.clearance_m(position);
                    if clearance < self.min_body_clearance_m {
                        self.min_body_clearance_m = clearance;
                        self.closest_approach = (link, obstacle.name.clone(), self.trot.time_s());
                    }
                }
            }
        }
        self.min_clearance_m = self.min_clearance_m.min(clearance_m(
            &self.obstacles,
            [observed.base_x_m, observed.base_z_m],
        ));
    }

    /// Heads for a point on the path 0.6 m ahead of the closest one: the yaw rate
    /// turns the bearing error, and the trot slows while that error is large and
    /// when the goal is near.
    fn follow(&mut self) -> UnitreeGo2TrotCommand {
        let Some(path) = &self.path else {
            return UnitreeGo2TrotCommand::default();
        };
        let estimate = self.estimate();
        let position = Vec3::new(estimate.x_m, estimate.y_m, 0.0);
        let (Some(goal), Some(closest)) = (path.goal(), path.closest_point(position)) else {
            return UnitreeGo2TrotCommand::default();
        };
        let to_goal_m = (goal.x_m - estimate.x_m).hypot(goal.y_m - estimate.y_m);
        if to_goal_m < GOAL_TOLERANCE_M && self.explorer.is_some() {
            if let Some(explorer) = self.explorer.as_mut() {
                explorer.clear_goal();
            }
            self.frontier_goals += 1;
            self.path = None;
            self.frames_since_plan = REPLAN_EVERY_FRAMES;
            return UnitreeGo2TrotCommand::default();
        }
        if to_goal_m < GOAL_TOLERANCE_M {
            let truth = true_pose(&self.sim);
            let (gx, gy) = nav_xy(GOALS[self.goal]);
            self.goal_true_error_m
                .push((truth.x_m - gx).hypot(truth.y_m - gy));
            self.goal += 1;
            self.path = None;
            self.frames_since_plan = REPLAN_EVERY_FRAMES;
            return UnitreeGo2TrotCommand::default();
        }
        let lookahead = path
            .point_at_distance(closest.arc_length_m + LOOKAHEAD_M)
            .unwrap_or(Vec3::new(goal.x_m, goal.y_m, 0.0));
        let bearing = wrap_angle(
            (lookahead.y - estimate.y_m).atan2(lookahead.x - estimate.x_m) - estimate.yaw_rad,
        );
        let alignment = bearing.cos().max(0.0).powi(2);
        let approach = (to_goal_m / SLOW_RADIUS_M).clamp(0.4, 1.0);
        let nominal = UnitreeGo2TrotCommand {
            forward_speed_m_s: if bearing.abs() > 1.0 {
                0.05
            } else {
                MAX_SPEED_M_S * alignment * approach
            },
            yaw_rate_rad_s: (HEADING_GAIN_PER_S * bearing)
                .clamp(-MAX_YAW_RATE_RAD_S, MAX_YAW_RATE_RAD_S),
        };
        self.guarded(nominal, estimate, lookahead)
    }

    /// Limits how fast the command changes: the guard's choice and the
    /// follower's can differ in sign, and a yaw rate flipped from +0.5 to
    /// -0.25 rad/s within 0.2 s made the trot fall.
    fn slew_limited(&mut self, wanted: UnitreeGo2TrotCommand) -> UnitreeGo2TrotCommand {
        let dt = 1.0 / UNITREE_GO2_MODEL_TROT_CONTROL_HZ;
        let step = |from: f64, to: f64, rate: f64| from + (to - from).clamp(-rate * dt, rate * dt);
        let last = self.last_command;
        let command = UnitreeGo2TrotCommand {
            forward_speed_m_s: step(
                last.forward_speed_m_s,
                wanted.forward_speed_m_s,
                MAX_ACCELERATION_M_S2,
            ),
            yaw_rate_rad_s: step(
                last.yaw_rate_rad_s,
                wanted.yaw_rate_rad_s,
                MAX_YAW_ACCELERATION_RAD_S2,
            ),
        };
        self.last_command = command;
        command
    }

    /// With `--explore`, the footprint guard's command instead of `nominal`
    /// when the nominal one would bring the body too close to an obstacle.
    /// A replacement is held for [`GUARD_HOLD_S`] so the two do not
    /// alternate step by step.
    fn guarded(
        &mut self,
        nominal: UnitreeGo2TrotCommand,
        estimate: Pose2d,
        target: Vec3,
    ) -> UnitreeGo2TrotCommand {
        let Some(field) = &self.clearance else {
            return nominal;
        };
        let now_s = self.trot.time_s();
        if let Some((until_s, command)) = self.guard_hold {
            if now_s < until_s {
                return command;
            }
            self.guard_hold = None;
        }
        if !self.trot.steps().is_multiple_of(GUARD_EVERY_STEPS) {
            return nominal;
        }
        let ((speed, yaw_rate), replaced) = guard::choose(
            field,
            estimate,
            (nominal.forward_speed_m_s, nominal.yaw_rate_rad_s),
            (
                self.last_command.forward_speed_m_s,
                self.last_command.yaw_rate_rad_s,
            ),
            target,
        );
        if !replaced {
            return nominal;
        }
        let command = UnitreeGo2TrotCommand {
            forward_speed_m_s: speed,
            yaw_rate_rad_s: yaw_rate,
        };
        self.guard_hold = Some((now_s + GUARD_HOLD_S, command));
        self.guard_overrides += 1;
        command
    }

    /// Leg odometry: the body's planar velocity and yaw rate with a scale error
    /// and a yaw-rate bias, integrated at the control rate.
    fn integrate_odometry(&mut self) {
        let observed = self.sim.observe();
        let base = self.sim.named_transform("base").expect("base pose");
        let yaw = yaw_of(base.rotation);
        let (vx, vy) = (
            observed.base_linear_velocity_x_m_s,
            -observed.base_linear_velocity_z_m_s,
        );
        let (sin, cos) = yaw.sin_cos();
        let forward = cos * vx + sin * vy;
        let left = -sin * vx + cos * vy;
        let dt = 1.0 / UNITREE_GO2_MODEL_TROT_CONTROL_HZ;
        let scale = 1.0 + ODOM_SCALE_ERROR;
        let delta = Pose2d::new(
            forward * scale * dt,
            left * scale * dt,
            (observed.base_angular_velocity_y_rad_s + ODOM_YAW_RATE_BIAS_RAD_S) * dt,
        );
        self.odom = self.odom.compose(delta);
    }

    fn slam_rms_m(&self) -> f64 {
        (self.slam_error_sq / self.error_samples.max(1) as f64).sqrt()
    }
}

/// Frontier exploration for the Go2: nearest frontier first (P1 and P2 found
/// it the steadier policy), the same inflation the planner uses, and a lidar
/// model for scoring.
fn explorer_config() -> ExplorerConfig {
    ExplorerConfig {
        frontier: FrontierConfig {
            min_cluster_cells: 8,
        },
        sensor: SensorModel {
            range_m: EXPLORE_SENSOR_RANGE_M,
            fov_rad: 2.0 * PI,
            rays: 180,
        },
        costmap: explore_costmap_config(),
        policy: ScorePolicy::NearestFrontier,
        min_gain_m2: 0.3,
        switch_margin: 0.5,
        hysteresis_radius_m: 1.0,
        escape_radius_m: 0.3,
    }
}

/// Fraction of each room's true floor that the map knows (free or occupied),
/// room A (wood) then room B (tile). The map frame is the world's: odometry
/// starts at the true pose.
fn room_coverage(grid: &OccupancyGrid, obstacles: &[StaticObject]) -> Vec<f64> {
    [Finish::Wood, Finish::Tile]
        .iter()
        .map(|finish| {
            let room = two_room_floors()
                .into_iter()
                .find(|patch| patch.finish == *finish)
                .expect("room floor");
            let (mut floor, mut known) = (0_usize, 0_usize);
            let cells = |lo: f64, hi: f64| ((hi - lo) / COVERAGE_PITCH_M) as usize;
            for i in 0..cells(room.min[0], room.max[0]) {
                for j in 0..cells(room.min[1], room.max[1]) {
                    let x = room.min[0] + (i as f64 + 0.5) * COVERAGE_PITCH_M;
                    let z = room.min[1] + (j as f64 + 0.5) * COVERAGE_PITCH_M;
                    if clearance_m(obstacles, [x, z]) < COVERAGE_CLEARANCE_M {
                        continue;
                    }
                    floor += 1;
                    let (nx, ny) = nav_xy([x, z]);
                    if grid
                        .world_to_grid(Vec3::new(nx, ny, 0.0))
                        .is_some_and(|cell| grid.is_known(cell))
                    {
                        known += 1;
                    }
                }
            }
            known as f64 / floor.max(1) as f64
        })
        .collect()
}

/// A return of a Mid-360 frame, levelled and de-skewed to the frame end.
struct LevelledReturn {
    /// Planar position in the levelled sensor frame (+x forward, +y left).
    at_end: Vec3,
    /// Height relative to the sensor.
    height_m: f64,
    /// Planar range when it was emitted.
    emission_range_m: f64,
}

impl LevelledReturn {
    /// The scan beam it falls in, and its range.
    fn beam(&self) -> (usize, f64) {
        let increment = 2.0 * PI / SCAN_BEAMS as f64;
        let beam =
            ((self.at_end.y.atan2(self.at_end.x) + PI) / increment).floor() as usize % SCAN_BEAMS;
        (beam, self.at_end.x.hypot(self.at_end.y))
    }
}

/// A levelled 720-beam scan with these ranges (NaN: no return).
fn level_scan(time_s: f64, ranges_m: Vec<f64>) -> LaserScan2d {
    let increment = 2.0 * PI / SCAN_BEAMS as f64;
    LaserScan2d {
        time_s,
        frame: FrameId::new("mid360_level"),
        angle_min_rad: -PI + 0.5 * increment,
        angle_increment_rad: increment,
        range_min_m: SCAN_MIN_RANGE_M,
        range_max_m: SCAN_MAX_RANGE_M,
        ranges_m,
    }
}

fn costmap_config() -> CostmapConfig {
    CostmapConfig {
        // The Go2 is 0.31 m wide; keep its centre 0.3 m off obstacles.
        inscribed_radius_m: 0.30,
        inflation_radius_m: 0.75,
        ..CostmapConfig::default()
    }
}

/// Exploration drives to frontiers, which lie where the map ends: often
/// right beside an obstacle whose near face the scan has only just seen, and
/// the Go2 turns toward them. The base is kept [`EXPLORE_INSCRIBED_RADIUS_M`]
/// off mapped obstacles.
fn explore_costmap_config() -> CostmapConfig {
    CostmapConfig {
        inscribed_radius_m: EXPLORE_INSCRIBED_RADIUS_M,
        ..costmap_config()
    }
}

/// The exploration costmap with its inscribed band made lethal, except within
/// the explorer's escape radius of the robot.
fn explore_planning_costmap(grid: &OccupancyGrid, robot: Pose2d) -> Costmap {
    let mut costmap = Costmap::from_occupancy(grid, &explore_costmap_config()).expect("costmap");
    let escape = explorer_config().escape_radius_m;
    for y in 0..costmap.height() {
        for x in 0..costmap.width() {
            let cell = GridCoord {
                x: x as isize,
                y: y as isize,
            };
            if costmap.cost_at(cell) != Some(COST_INSCRIBED) {
                continue;
            }
            let centre = costmap.grid_to_world(cell);
            if (centre.x - robot.x_m).hypot(centre.y - robot.y_m) > escape {
                costmap.apply_cost(cell, COST_LETHAL);
            }
        }
    }
    costmap
}

/// Scan matching over all 360 beams, searched on a finer grid than the default.
fn slam_config() -> SlamConfig {
    let mut config = SlamConfig {
        max_beams: 360,
        ..SlamConfig::default()
    };
    config.matcher.linear_samples = 7;
    config.matcher.angular_samples = 7;
    config.matcher.levels = 4;
    config
}

/// The base's true planar pose in the navigation frame.
fn true_pose(sim: &UrdfSceneSim) -> Pose2d {
    let base = sim.named_transform("base").expect("base pose");
    Pose2d::new(
        base.translation.x,
        -base.translation.z,
        yaw_of(base.rotation),
    )
}

/// Occupied map cells, and the fraction lying within 10 cm of a real obstacle.
fn map_precision(grid: &OccupancyGrid, obstacles: &[StaticObject]) -> (usize, f64) {
    let mut occupied = 0;
    let mut on_obstacle = 0;
    for y in 0..grid.height() {
        for x in 0..grid.width() {
            let coord = GridCoord {
                x: x as isize,
                y: y as isize,
            };
            if !grid.is_occupied(coord) {
                continue;
            }
            occupied += 1;
            let center = grid.grid_to_world(coord);
            if clearance_m(obstacles, [center.x, -center.y]) < 0.10 {
                on_obstacle += 1;
            }
        }
    }
    (occupied, on_obstacle as f64 / occupied.max(1) as f64)
}

fn run(explore: bool, mut on_frame: impl FnMut(&Nav)) -> Nav {
    let mut nav = Nav::new(explore, &scene_path());
    while !nav.done() {
        nav.step_frame();
        on_frame(&nav);
    }
    nav
}

fn report_and_gate(nav: &Nav) {
    let (occupied, precision) = map_precision(nav.slam.grid(), &nav.obstacles);
    if nav.explorer.is_some() {
        report_and_gate_exploration(nav);
    } else {
        report_and_gate_goals(nav);
    }
    println!(
        "localization: SLAM {:.3} m RMS ({:.3} max) vs odometry alone {:.3} m max; {} keyframes matched over {} frames, {} loop closures",
        nav.slam_rms_m(),
        nav.slam_error_max_m,
        nav.odom_error_max_m,
        nav.matched_frames,
        nav.frame_index,
        nav.loop_closures
    );
    println!(
        "map: {occupied} occupied cells, {:.1} % within 10 cm of a real obstacle",
        precision * 100.0
    );
    assert!(
        nav.min_height_m > 0.2,
        "the walk sagged: {:.3} m",
        nav.min_height_m
    );
    assert!(
        nav.min_clearance_m > 0.2,
        "came within {:.2} m of an obstacle",
        nav.min_clearance_m
    );
    assert!(
        nav.slam_error_max_m < nav.odom_error_max_m,
        "SLAM did not beat odometry"
    );
    assert!(
        nav.slam_rms_m() < 0.15,
        "localization RMS {:.3} m",
        nav.slam_rms_m()
    );
    // Measured 100 % with the finer matcher.
    assert!(precision > 0.95, "map precision {:.3}", precision);
}

/// The hand-given goals: both reached, close to where they are.
fn report_and_gate_goals(nav: &Nav) {
    println!(
        "reached {}/{} goals in {:.1} s (true error at arrival {:?} m); clearance >= {:.2} m, lowest body {:.3} m",
        nav.goal,
        GOALS.len(),
        nav.trot.time_s(),
        nav.goal_true_error_m
            .iter()
            .map(|e| (e * 100.0).round() / 100.0)
            .collect::<Vec<_>>(),
        nav.min_clearance_m,
        nav.min_height_m
    );
    assert_eq!(nav.goal, GOALS.len(), "not every goal was reached");
    assert!(
        nav.goal_true_error_m.iter().all(|e| *e < 0.5),
        "arrived off goal: {:?}",
        nav.goal_true_error_m
    );
}

/// Exploration: the explorer finished on its own, and both rooms are mapped.
fn report_and_gate_exploration(nav: &Nav) {
    let coverage = room_coverage(nav.slam.grid(), &nav.obstacles);
    println!(
        "explored in {:.1} s ({}): {} frontier goals, {} without a path, {} given up, {} guard overrides; mapped room A {:.1} %, room B {:.1} %; clearance >= {:.2} m (body {:.2} m: {} to {} at {:.1} s), lowest body {:.3} m",
        nav.trot.time_s(),
        if nav.explored { "nothing left" } else { "timed out" },
        nav.frontier_goals,
        nav.plan_failures,
        nav.abandoned.len(),
        nav.guard_overrides,
        coverage[0] * 100.0,
        coverage[1] * 100.0,
        nav.min_clearance_m,
        nav.min_body_clearance_m,
        nav.closest_approach.0,
        nav.closest_approach.1,
        nav.closest_approach.2,
        nav.min_height_m
    );
    assert!(nav.explored, "exploration did not finish");
    // Measured 0.17 m or more over 41 layouts with the guard; contact makes
    // the trot fall.
    assert!(
        nav.min_body_clearance_m > 0.10,
        "a leg came within {:.2} m of an obstacle",
        nav.min_body_clearance_m
    );
    assert!(
        coverage.iter().all(|c| *c > EXPLORE_MIN_ROOM_COVERAGE),
        "a room is not mapped: {coverage:?}"
    );
}

/// The dressed two-room scene, or with `--layout-seed N` a generated layout
/// written under `target/`.
fn scene_path() -> PathBuf {
    let Some(seed) = layout_seed() else {
        return repo_path("assets/scenes/unitree_go2_two_rooms.rne.scene.toml");
    };
    let robot = repo_path("assets/robots/unitree_go2_jump.rne.robot.toml");
    let robot = robot.canonicalize().expect("Go2 robot file");
    let layout = layouts::generate(seed);
    let dir = repo_path("target/rne-go2-layouts");
    fs::create_dir_all(&dir).expect("create layout directory");
    let path = dir.join(format!("layout-{seed}.rne.scene.toml"));
    let robot = robot.to_str().expect("UTF-8 path");
    fs::write(&path, layouts::scene_toml(&layout, robot)).expect("write layout scene");
    path
}

fn layout_seed() -> Option<u64> {
    let args: Vec<String> = std::env::args().collect();
    let at = args.iter().position(|a| a == "--layout-seed")?;
    Some(
        args.get(at + 1)
            .and_then(|s| s.parse().ok())
            .expect("--layout-seed takes a number"),
    )
}

fn main() {
    let explore = std::env::args().any(|arg| arg == "--explore");
    assert!(
        explore || layout_seed().is_none(),
        "--layout-seed needs --explore: the goals are placed for the dressed scene"
    );
    if std::env::args().any(|arg| arg == "--smoke") {
        let nav = run(explore, |_| {});
        if let Some(seed) = layout_seed() {
            println!("layout seed {seed}: {:?}", layouts::generate(seed));
        }
        report_and_gate(&nav);
        if explore {
            println!("smoke ok: the Go2 explored both rooms on its Mid-360 with no goals given");
        } else {
            println!("smoke ok: the Go2 mapped and navigated both rooms on its Mid-360");
        }
        return;
    }
    render_media(explore);
}

fn render_media(explore: bool) {
    let view = CameraOrbit {
        focus: Vec3::new(2.3, 0.0, 0.2),
        yaw_rad: -2.05,
        pitch_rad: 0.95,
        distance_m: 8.4,
    };
    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let mut mesh_cache = MeshRenderCache::new();
    let interior = Interior::new(two_room_floors());
    let frames_dir = repo_path(if explore {
        "target/rne-go2-mid360-explore-frames"
    } else {
        "target/rne-go2-mid360-navigation-frames"
    });
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create frame directory");
    let mut frame = 0_usize;
    let nav = run(explore, |nav| {
        if !nav.frame_index.is_multiple_of(LIDAR_FRAMES_PER_GIF_FRAME) {
            return;
        }
        let rgba = render_frame(
            &mut backend,
            &camera,
            &mut mesh_cache,
            &interior,
            nav,
            &view,
        );
        write_png(&frames_dir.join(format!("frame-{frame:03}.png")), &rgba).expect("write frame");
        frame += 1;
    });
    report_and_gate(&nav);
    let media_dir = repo_path("docs/media");
    let name = if explore {
        "go2-mid360-explore"
    } else {
        "go2-mid360-navigation"
    };
    let gif_path = media_dir.join(format!("{name}.gif"));
    build_gif(&frames_dir, &gif_path).expect("encode gif");
    image::open(frames_dir.join(format!("frame-{:03}.png", frame * 3 / 5)))
        .expect("read poster frame")
        .save(media_dir.join(format!("{name}.png")))
        .expect("write poster");
    println!("wrote {} ({frame} frames)", gif_path.display());
}

fn render_frame(
    backend: &mut WgpuRenderBackend,
    camera: &Camera,
    mesh_cache: &mut MeshRenderCache,
    interior: &Interior,
    nav: &Nav,
    view: &CameraOrbit,
) -> Vec<u8> {
    let mut scene = build_visual_render_scene(nav.sim.world());
    // Cutaway: walls drawn 12 cm tall, so the map on the floor and the returns
    // on the walls show. Their true heights are in the scene.
    interior.decorate(
        &mut scene,
        nav.sim.world(),
        DecorOptions {
            wall_height_cap_m: Some(CUTAWAY_HEIGHT_M),
        },
    );
    append_occupancy(&mut scene, nav.slam.grid());
    if let Some(path) = &nav.path {
        let mut mesh = QuadMesh::default();
        for pair in path.waypoints().windows(2) {
            mesh.add_segment(
                Vec3::new(pair[0].x_m, 0.02, -pair[0].y_m),
                Vec3::new(pair[1].x_m, 0.02, -pair[1].y_m),
                0.035,
            );
        }
        push_mesh(&mut scene, mesh, [0.98, 0.84, 0.25, 1.0]);
    }
    let goals: &[[f64; 2]] = if nav.explorer.is_some() { &[] } else { &GOALS };
    for (index, goal) in goals.iter().enumerate() {
        let color = if index < nav.goal {
            [0.35, 0.85, 0.45, 1.0]
        } else {
            [0.95, 0.35, 0.30, 1.0]
        };
        scene.items.push(box_item(
            Vec3::new(goal[0], 0.015, goal[1]),
            Vec3::new(0.22, 0.02, 0.22),
            color,
        ));
    }
    let mut trail = QuadMesh::default();
    for pair in nav.true_trail.windows(2) {
        trail.add_segment(
            Vec3::new(pair[0][0], 0.012, pair[0][1]),
            Vec3::new(pair[1][0], 0.012, pair[1][1]),
            0.02,
        );
    }
    push_mesh(&mut scene, trail, [0.55, 0.62, 0.72, 1.0]);
    if let Some(cloud) = &nav.last_cloud {
        append_height_colored_points(&mut scene, cloud);
    }
    let roots: Vec<&Path> = nav
        .sim
        .mesh_package_roots()
        .iter()
        .map(PathBuf::as_path)
        .collect();
    mesh_cache
        .resolve_scene(&mut scene, &roots)
        .expect("resolve official Go2 meshes");
    backend
        .render_scene_camera(camera, &view.camera_transform(), &scene, CLEAR_COLOR)
        .expect("render frame")
        .color
        .rgba8
}

/// The SLAM map on the floor, drawn as fog of war: unexplored cells dark, known
/// free cells left clear so the floor shows, occupied cells bright.
fn append_occupancy(scene: &mut RenderScene, grid: &OccupancyGrid) {
    let mut occupied = QuadMesh::default();
    let mut unknown = QuadMesh::default();
    let half = grid.resolution_m() * 0.5;
    for y in 0..grid.height() {
        for x in 0..grid.width() {
            let coord = GridCoord {
                x: x as isize,
                y: y as isize,
            };
            let center = grid.grid_to_world(coord);
            let world = Vec3::new(center.x, 0.0, -center.y);
            if grid.is_occupied(coord) {
                occupied.add_floor_square(world + Vec3::Y * 0.008, half);
            } else if !grid.is_known(coord) {
                unknown.add_floor_square(world + Vec3::Y * 0.005, half);
            }
        }
    }
    push_mesh(scene, unknown, [0.035, 0.045, 0.06, 1.0]);
    push_mesh(scene, occupied, [0.92, 0.36, 0.55, 1.0]);
}

fn append_height_colored_points(scene: &mut RenderScene, cloud: &PointCloud) {
    let mut buckets: Vec<QuadMesh> = (0..COLORMAP_BUCKETS).map(|_| QuadMesh::default()).collect();
    for point in &cloud.points_m {
        let t = point.y.clamp(0.0, 1.0);
        let bucket =
            ((t * (COLORMAP_BUCKETS - 1) as f64).round() as usize).min(COLORMAP_BUCKETS - 1);
        buckets[bucket].add_marker(*point, 0.02);
    }
    for (bucket, mesh) in buckets.into_iter().enumerate() {
        let t = bucket as f64 / (COLORMAP_BUCKETS - 1) as f64;
        push_mesh(scene, mesh, turbo_colormap(0.1 + 0.85 * t));
    }
}

fn box_item(translation: Vec3, scale: Vec3, color: [f32; 4]) -> RenderSceneItem {
    RenderSceneItem {
        transform: Transform3 {
            translation,
            rotation: Quat::IDENTITY,
            scale,
        },
        shape: VisualShape::Box { size_m: Vec3::ONE },
        color_rgba: color,
        mesh: None,
        base_color_texture: None,
        material: Default::default(),
    }
}

#[derive(Default)]
struct QuadMesh {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    texcoords: Vec<[f32; 2]>,
    indices: Vec<u32>,
}

impl QuadMesh {
    fn add_marker(&mut self, center: Vec3, radius_m: f64) {
        let x = Vec3::X * radius_m;
        let y = Vec3::Y * radius_m;
        let z = Vec3::Z * radius_m;
        self.add_quad(center - x, center + y, center - y, center + x);
        self.add_quad(center - z, center + y, center - y, center + z);
        self.add_quad(center - x, center + z, center - z, center + x);
    }

    fn add_floor_square(&mut self, center: Vec3, half_m: f64) {
        self.add_quad(
            center + Vec3::new(-half_m, 0.0, -half_m),
            center + Vec3::new(half_m, 0.0, -half_m),
            center + Vec3::new(-half_m, 0.0, half_m),
            center + Vec3::new(half_m, 0.0, half_m),
        );
    }

    fn add_segment(&mut self, start: Vec3, end: Vec3, width_m: f64) {
        let delta = end - start;
        let side = Vec3::new(-delta.z, 0.0, delta.x).normalize_or_zero() * width_m * 0.5;
        self.add_quad(start - side, start + side, end - side, end + side);
    }

    fn add_quad(&mut self, first: Vec3, second: Vec3, third: Vec3, fourth: Vec3) {
        let base = self.positions.len() as u32;
        self.positions
            .extend([first, second, third, fourth].map(|p| [p.x as f32, p.y as f32, p.z as f32]));
        self.normals.extend([[0.0, 1.0, 0.0]; 4]);
        self.texcoords.extend([[0.0, 0.0]; 4]);
        // Both windings, so the quad shows from either side.
        self.indices.extend([
            base,
            base + 1,
            base + 2,
            base + 2,
            base + 1,
            base + 3,
            base,
            base + 2,
            base + 1,
            base + 2,
            base + 3,
            base + 1,
        ]);
    }
}

fn push_mesh(scene: &mut RenderScene, mesh: QuadMesh, color_rgba: [f32; 4]) {
    if mesh.indices.is_empty() {
        return;
    }
    scene.items.push(RenderSceneItem {
        transform: Transform3::IDENTITY,
        shape: VisualShape::DynamicMesh,
        color_rgba,
        mesh: Some(Arc::new(TriangleMesh {
            positions: mesh.positions,
            normals: mesh.normals,
            texcoords: mesh.texcoords,
            indices: mesh.indices,
            skinning: None,
        })),
        base_color_texture: None,
        material: Default::default(),
    });
}

/// Google's Turbo colormap, polynomial approximation.
fn turbo_colormap(t: f64) -> [f32; 4] {
    let t = t.clamp(0.0, 1.0);
    let polynomial = |c: [f64; 6]| -> f32 {
        (c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * (c[4] + t * c[5]))))).clamp(0.0, 1.0) as f32
    };
    [
        polynomial([
            0.135_721_38,
            4.615_392_60,
            -42.660_322_58,
            132.131_082_34,
            -152.942_393_96,
            59.286_379_43,
        ]),
        polynomial([
            0.091_402_61,
            2.194_188_39,
            4.842_966_58,
            -14.185_033_33,
            4.277_298_57,
            2.829_566_04,
        ]),
        polynomial([
            0.106_673_30,
            12.641_946_08,
            -60.582_048_36,
            110.362_767_71,
            -89.903_109_12,
            27.348_249_73,
        ]),
        1.0,
    ]
}

fn build_gif(frames_dir: &Path, gif_path: &Path) -> std::io::Result<()> {
    let status = std::process::Command::new("ffmpeg")
        .args(["-y", "-loglevel", "error", "-framerate", "8", "-i"])
        .arg(frames_dir.join("frame-%03d.png"))
        .args([
            "-vf",
            "scale=720:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96:stats_mode=diff[p];[b][p]paletteuse=dither=none:diff_mode=rectangle",
        ])
        .arg(gif_path)
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg gif encode failed"))
}

fn write_png(path: &Path, rgba: &[u8]) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, WIDTH, HEIGHT);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    encoder
        .write_header()?
        .write_image_data(rgba)
        .map_err(std::io::Error::other)
}
