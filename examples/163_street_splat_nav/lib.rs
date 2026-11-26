//! A delivery robot on a real street rebuilt from a 360-degree video (visloc-rs `real2sim`).
//!
//! visloc-rs builds the street and writes it as a *street bundle* (`rne_bundle.py`): the
//! splat and its manifest, the solid cells the robot can hit, the lane map it plans on, the
//! ground height, the walk and the kerbs. This library loads a bundle and drives the robot
//! through its goals with `rne_nav` on ground truth: an A* plan on the lane costmap (the
//! lane free, off it costly but not lethal, walls lethal, inflated for the robot's body) and
//! pure pursuit along it. Where the robot crosses a street the road between the kerbs is
//! 10 cm lower: the robot pitches on its front and rear wheels and crawls while a wheel is
//! on a kerb edge.
//!
//! Everything here is in the bundle's frame, the visloc-rs map frame: z up, metres, yaw
//! counter-clockwise about z. [`map_to_rne`] turns a point into RNE's y-up world, as the
//! bundle's splat manifest does.

pub mod crowd;

use crowd::Crowd;
use rne_math::Vec3;
use rne_nav::{
    avoid_velocities, from_map_json, plan_path, pure_pursuit_follow, AvoidanceConfig,
    CircularObstacle, Costmap, CostmapConfig, GlobalPlannerConfig, GridCoord, OccupancyGrid,
    Path2d, Pose2d, PurePursuitConfig, VelocityCommand2d, COST_LETHAL,
};
use serde::Deserialize;
use std::fmt;
use std::path::{Path, PathBuf};

/// A point of the bundle's (z-up) frame in RNE's y-up world: RNE (x, y, z) = map (x, z, -y).
pub fn map_to_rne(x_m: f64, y_m: f64, z_m: f64) -> Vec3 {
    Vec3::new(x_m, z_m, -y_m)
}

/// Error loading or driving a street.
#[derive(Debug)]
pub enum StreetError {
    /// A bundle file could not be read.
    Io(PathBuf, std::io::Error),
    /// A bundle file did not parse.
    Parse(PathBuf, String),
    /// A goal could not be planned to.
    Plan(usize, String),
}

impl fmt::Display for StreetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(path, e) => write!(f, "{}: {e}", path.display()),
            Self::Parse(path, e) => write!(f, "{}: {e}", path.display()),
            Self::Plan(goal, e) => write!(f, "goal {}: {e}", goal + 1),
        }
    }
}

impl std::error::Error for StreetError {}

/// The ground's height on a regular grid (row `j` at `y0_m + j * resolution_m`).
#[derive(Clone, Debug)]
pub struct Ground {
    /// x of column 0 (m).
    pub x0_m: f64,
    /// y of row 0 (m).
    pub y0_m: f64,
    /// Cell size (m).
    pub resolution_m: f64,
    /// Columns.
    pub width: usize,
    /// Rows.
    pub height: usize,
    /// Heights (m), row-major.
    pub z_m: Vec<f32>,
}

impl Ground {
    /// The height of the cell holding (x, y), clamped to the grid's edge.
    pub fn height_at(&self, x_m: f64, y_m: f64) -> f64 {
        let col = ((x_m - self.x0_m) / self.resolution_m).floor() as isize;
        let row = ((y_m - self.y0_m) / self.resolution_m).floor() as isize;
        let col = col.clamp(0, self.width as isize - 1) as usize;
        let row = row.clamp(0, self.height as isize - 1) as usize;
        f64::from(self.z_m[row * self.width + col])
    }
}

/// A road crossed between two kerbs, lowered by `drop_m`.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize)]
pub struct Kerb {
    /// One kerb (x, y).
    pub a: [f64; 2],
    /// The other kerb (x, y).
    pub b: [f64; 2],
    /// Half the width of the lowered strip, across the crossing (m).
    pub half_width: f64,
    /// How far the road is below the pavement (m).
    pub drop: f64,
}

/// The kerbs of a street: the ground's drop between them and the robot's pose over them.
#[derive(Clone, Debug, Default)]
pub struct Kerbs {
    kerbs: Vec<Kerb>,
    /// The kerb's face: the drop eases over this many metres.
    pub edge_m: f64,
}

impl Kerbs {
    /// Kerbs with a 6 cm face.
    pub fn new(kerbs: Vec<Kerb>) -> Self {
        Self {
            kerbs,
            edge_m: 0.06,
        }
    }

    /// How far the ground at (x, y) is below the pavement (m).
    pub fn drop_at(&self, x_m: f64, y_m: f64) -> f64 {
        for k in &self.kerbs {
            let (dx, dy) = (k.b[0] - k.a[0], k.b[1] - k.a[1]);
            let len = dx.hypot(dy).max(1e-9);
            let (ux, uy) = (dx / len, dy / len);
            let (px, py) = (x_m - k.a[0], y_m - k.a[1]);
            let t = px * ux + py * uy;
            let lateral = (ux * py - uy * px).abs();
            if lateral > k.half_width {
                continue;
            }
            let inside = t.min(len - t);
            if inside > -self.edge_m / 2.0 {
                return k.drop * ((inside + self.edge_m / 2.0) / self.edge_m).min(1.0);
            }
        }
        0.0
    }

    /// (drop of the body's middle, pitch) from the ground under the front and rear wheels;
    /// pitch > 0 is nose up.
    pub fn pose(&self, x_m: f64, y_m: f64, yaw_rad: f64, wheelbase_m: f64) -> (f64, f64) {
        let (s, c) = yaw_rad.sin_cos();
        let h = wheelbase_m / 2.0;
        let front = -self.drop_at(x_m + c * h, y_m + s * h);
        let rear = -self.drop_at(x_m - c * h, y_m - s * h);
        ((front + rear) / 2.0, (front - rear).atan2(wheelbase_m))
    }

    /// Whether a wheel is on (or about to reach) a kerb edge.
    pub fn on_edge(&self, x_m: f64, y_m: f64, yaw_rad: f64, wheelbase_m: f64) -> bool {
        let (s, c) = yaw_rad.sin_cos();
        let margin = 0.12;
        [wheelbase_m / 2.0, -wheelbase_m / 2.0].iter().any(|off| {
            let (px, py) = (x_m + c * off, y_m + s * off);
            (self.drop_at(px + c * margin, py + s * margin)
                - self.drop_at(px - c * margin, py - s * margin))
            .abs()
                > 1e-3
        })
    }
}

#[derive(Deserialize)]
struct RawStreet {
    start: [f64; 3],
    goals: Vec<[f64; 3]>,
    robot_radius_m: Option<f64>,
    camera_height_m: Option<f64>,
    walk: Option<Vec<[f64; 2]>>,
    #[serde(default)]
    kerbs: Vec<Kerb>,
    ground: Option<RawGround>,
    credit: Option<String>,
    robot_actor: Option<String>,
}

#[derive(Deserialize)]
struct RawGround {
    file: String,
    x0: f64,
    y0: f64,
    resolution_m: f64,
    width: usize,
    height: usize,
}

/// A street bundle (visloc-rs `rne_bundle.py`).
#[derive(Clone, Debug)]
pub struct Street {
    /// The bundle's directory.
    pub dir: PathBuf,
    /// Start (x, y, yaw).
    pub start: [f64; 3],
    /// Goals (x, y, yaw), in order.
    pub goals: Vec<[f64; 3]>,
    /// The robot's footprint in the simulator's collision check (m).
    pub robot_radius_m: f64,
    /// Camera height above the ground (m).
    pub camera_height_m: f64,
    /// The line the video was walked: the pavement's.
    pub walk: Vec<[f64; 2]>,
    /// Kerbs where the robot crosses a street.
    pub kerbs: Kerbs,
    /// The ground's height, when the bundle has one.
    pub ground: Option<Ground>,
    /// Solid cells: walls, posts, trunks.
    pub solid: OccupancyGrid,
    /// The map the robot plans on: the lane free, off it costly, walls occupied.
    pub lane: OccupancyGrid,
    /// The robot drawn in the chase view (a splat actor PLY in the bundle), if any.
    pub robot_actor: Option<PathBuf>,
    /// The footage's credit line.
    pub credit: String,
}

impl Street {
    /// Loads a bundle directory.
    pub fn load(dir: &Path) -> Result<Self, StreetError> {
        let read = |name: &str| {
            let path = dir.join(name);
            std::fs::read_to_string(&path).map_err(|e| StreetError::Io(path, e))
        };
        let path = dir.join("street.json");
        let raw: RawStreet = serde_json::from_str(&read("street.json")?)
            .map_err(|e| StreetError::Parse(path, e.to_string()))?;
        let map = |name: &str| {
            from_map_json(&read(name)?)
                .map_err(|e| StreetError::Parse(dir.join(name), e.to_string()))
        };
        let ground = match raw.ground {
            Some(g) => {
                let path = dir.join(&g.file);
                let bytes = std::fs::read(&path).map_err(|e| StreetError::Io(path.clone(), e))?;
                if bytes.len() != g.width * g.height * 4 {
                    return Err(StreetError::Parse(path, "ground size".into()));
                }
                let z_m = bytes
                    .chunks_exact(4)
                    .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
                    .collect();
                Some(Ground {
                    x0_m: g.x0,
                    y0_m: g.y0,
                    resolution_m: g.resolution_m,
                    width: g.width,
                    height: g.height,
                    z_m,
                })
            }
            None => None,
        };
        Ok(Self {
            dir: dir.to_path_buf(),
            start: raw.start,
            goals: raw.goals,
            robot_radius_m: raw.robot_radius_m.unwrap_or(0.2),
            camera_height_m: raw.camera_height_m.unwrap_or(1.2),
            walk: raw.walk.unwrap_or_default(),
            kerbs: Kerbs::new(raw.kerbs),
            ground,
            solid: map("solid.rne.map")?,
            lane: map("lane.rne.map")?,
            robot_actor: raw.robot_actor.map(|f| dir.join(f)),
            credit: raw.credit.unwrap_or_default(),
        })
    }

    /// The bundle's crowd (`crowd.json`, visloc-rs `rne_crowd.py`), placed; `seed` seeds its
    /// behaviour. `None` when the bundle has no crowd.
    pub fn crowd(&self, seed: u64) -> Result<Option<Crowd>, StreetError> {
        let path = self.dir.join("crowd.json");
        if !path.exists() {
            return Ok(None);
        }
        let text = std::fs::read_to_string(&path).map_err(|e| StreetError::Io(path.clone(), e))?;
        let spec: crowd::CrowdSpec =
            serde_json::from_str(&text).map_err(|e| StreetError::Parse(path, e.to_string()))?;
        let mut crowd = Crowd::new(spec, seed, &|x, y| self.solid_at(x, y));
        crowd.robot_radius_m = 0.6;
        Ok(Some(crowd))
    }

    /// The splat manifest.
    pub fn manifest(&self) -> PathBuf {
        self.dir.join("street.rne.splat.toml")
    }

    /// Ground height at (x, y), with the road between kerbs lowered.
    pub fn ground_at(&self, x_m: f64, y_m: f64) -> f64 {
        self.ground.as_ref().map_or(0.0, |g| g.height_at(x_m, y_m))
    }

    /// Whether the cell at (x, y) is solid; outside the map counts as solid.
    pub fn solid_at(&self, x_m: f64, y_m: f64) -> bool {
        self.solid
            .world_to_grid(Vec3::new(x_m, y_m, 0.0))
            .is_none_or(|c| self.solid.is_occupied(c))
    }

    /// Whether a circle of `radius_m` at (x, y) covers a solid cell's centre.
    pub fn hits_solid(&self, x_m: f64, y_m: f64, radius_m: f64) -> bool {
        let grid = &self.solid;
        let res = grid.resolution_m();
        let o = grid.origin();
        let c0 = ((x_m - radius_m - o.x_m) / res).floor() as isize;
        let c1 = ((x_m + radius_m - o.x_m) / res).ceil() as isize;
        let r0 = ((y_m - radius_m - o.y_m) / res).floor() as isize;
        let r1 = ((y_m + radius_m - o.y_m) / res).ceil() as isize;
        for row in r0..=r1 {
            for col in c0..=c1 {
                let coord = GridCoord { x: col, y: row };
                if grid.is_occupied(coord) {
                    let cx = o.x_m + col as f64 * res;
                    let cy = o.y_m + row as f64 * res;
                    if (cx - x_m).hypot(cy - y_m) <= radius_m {
                        return true;
                    }
                }
            }
        }
        false
    }

    /// Distance from (x, y) to the nearest solid cell's centre, searched out to `reach_m`.
    pub fn solid_clearance(&self, x_m: f64, y_m: f64, reach_m: f64) -> f64 {
        let grid = &self.solid;
        let res = grid.resolution_m();
        let o = grid.origin();
        let n = (reach_m / res).ceil() as isize;
        let col = ((x_m - o.x_m) / res).round() as isize;
        let row = ((y_m - o.y_m) / res).round() as isize;
        let mut best = reach_m;
        for r in row - n..=row + n {
            for c in col - n..=col + n {
                if grid.is_occupied(GridCoord { x: c, y: r }) {
                    let d = (o.x_m + c as f64 * res - x_m).hypot(o.y_m + r as f64 * res - y_m);
                    best = best.min(d);
                }
            }
        }
        best
    }
}

/// How the robot plans and drives.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NavConfig {
    /// The robot's body as the planner sees it (m): the drawn robot is 0.87 x 0.70 m.
    pub footprint_radius_m: f64,
    /// Inflation beyond the footprint (m).
    pub inflation_m: f64,
    /// Cost of a cell off the lane (of 252): costly, not lethal, as in the Nav2 runs.
    pub off_lane_cost: u8,
    /// Planner weight on cell cost.
    pub cost_weight: f64,
    /// Top speed (m/s).
    pub max_speed_m_s: f64,
    /// Speed while a wheel is on a kerb edge (m/s).
    pub kerb_speed_m_s: f64,
    /// Front-to-rear wheel distance for the kerb pitch (m).
    pub wheelbase_m: f64,
    /// Goal tolerance (m).
    pub goal_tolerance_m: f64,
    /// Time allowed per goal (s).
    pub goal_timeout_s: f64,
    /// Simulation step (s).
    pub step_s: f64,
    /// Wheel odometry's speed scale error (fraction; the visloc-rs simulator's default).
    pub odom_scale_error: f64,
    /// Wheel odometry's yaw-rate bias while moving (rad/s).
    pub odom_yaw_bias_rad_s: f64,
    /// Wheel odometry's yaw-rate scale error (fraction).
    pub odom_yaw_scale_error: f64,
    /// Simulated seconds between localization attempts.
    pub fix_every_s: f64,
    /// A fix further than this from the estimate is rejected (m) ...
    pub fix_gate_m: f64,
    /// ... plus this much per second since the last accepted fix (m/s).
    pub fix_gate_growth_m_s: f64,
    /// How far an accepted fix moves the estimate towards it (1: all the way). Below 1,
    /// one wrong fix that passes the gate moves the estimate only part of the way.
    pub fix_blend: f64,
    /// Held up by people this long (s), the robot plans again with them as obstacles (at
    /// most once per this long); 0, the default, never does. On the OverMaps360 street it
    /// made things worse: plans round people left the lane by posts and ended in deadlocks.
    pub replan_after_s: f64,
}

impl Default for NavConfig {
    fn default() -> Self {
        Self {
            footprint_radius_m: 0.45,
            inflation_m: 0.25,
            off_lane_cost: 234,
            cost_weight: 3.0,
            max_speed_m_s: 0.5,
            kerb_speed_m_s: 0.12,
            wheelbase_m: 0.5,
            goal_tolerance_m: 0.3,
            goal_timeout_s: 600.0,
            step_s: 0.02,
            odom_scale_error: 0.02,
            odom_yaw_bias_rad_s: 0.002,
            odom_yaw_scale_error: 0.03,
            fix_every_s: 0.5,
            fix_gate_m: 0.25,
            fix_gate_growth_m_s: 0.1,
            fix_blend: 0.5,
            replan_after_s: 0.0,
        }
    }
}

/// The robot's state at one step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RobotState {
    /// Simulated time (s).
    pub t_s: f64,
    /// Position (m) and heading (rad) in the bundle's frame.
    pub x_m: f64,
    /// See `x_m`.
    pub y_m: f64,
    /// See `x_m`.
    pub yaw_rad: f64,
    /// Height of the body's base (m): the ground, less the kerb drop.
    pub z_m: f64,
    /// Nose-up pitch over a kerb (rad).
    pub pitch_rad: f64,
    /// Forward speed (m/s).
    pub speed_m_s: f64,
    /// Goal being driven to (0-based).
    pub goal: usize,
    /// The robot's own estimate of its pose (x, y, yaw), which it steers on.
    pub estimate: Pose2d,
}

/// What a drive did.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DriveReport {
    /// Goals reached.
    pub reached: usize,
    /// Goals in the street.
    pub goals: usize,
    /// Steps a solid cell stopped the robot.
    pub contacts: usize,
    /// Simulated time (s).
    pub time_s: f64,
    /// Distance driven (m).
    pub distance_m: f64,
    /// Time with a wheel on a kerb edge (s).
    pub kerb_time_s: f64,
    /// Fastest speed with a wheel on a kerb edge (m/s).
    pub kerb_max_speed_m_s: f64,
    /// Nearest the robot's centre came to a solid cell (m, searched to 2 m).
    pub min_solid_clearance_m: f64,
    /// Median and largest distance from the walk (m).
    pub walk_distance_median_m: f64,
    /// See `walk_distance_median_m`.
    pub walk_distance_max_m: f64,
    /// A hash of every step's pose, for replay checks.
    pub trajectory_hash: u64,
    /// Localization attempts, fixes accepted, and fixes rejected by the gate.
    pub fixes_attempted: usize,
    /// See `fixes_attempted`.
    pub fixes_accepted: usize,
    /// See `fixes_attempted`.
    pub fixes_rejected: usize,
    /// Position error of the estimate (m): RMSE, median and largest, sampled every 0.2 s.
    pub pose_error_rmse_m: f64,
    /// See `pose_error_rmse_m`.
    pub pose_error_median_m: f64,
    /// See `pose_error_rmse_m`.
    pub pose_error_max_m: f64,
    /// Steps a person, bike or car stopped the robot (its 0.2 m circle).
    pub people_contacts: usize,
    /// Plans made again round people who held the robot up.
    pub replans: usize,
}

/// One localization: the true state (to render the camera from) and the robot's estimate
/// (the prior) in; a fix (x, y, yaw) or nothing out.
pub type Localize<'a> = dyn FnMut(&RobotState, Pose2d) -> Option<Pose2d> + 'a;

/// The lane costmap: walls and the street's solid cells (what the robot's scan would show:
/// posts, trunks) lethal and inflated for the footprint, off the lane costly.
pub fn lane_costmap(street: &Street, config: &NavConfig) -> Costmap {
    let mut grid = street.lane.clone();
    let solid = &street.solid;
    let (so, sr) = (solid.origin(), solid.resolution_m());
    for row in 0..solid.height() as isize {
        for col in 0..solid.width() as isize {
            if solid.is_occupied(GridCoord { x: col, y: row }) {
                let point = Vec3::new(so.x_m + col as f64 * sr, so.y_m + row as f64 * sr, 0.0);
                if let Some(coord) = grid.world_to_grid(point) {
                    grid.apply_occupied(coord, 10.0);
                }
            }
        }
    }
    let costmap_config = CostmapConfig {
        occupied_probability: 0.95,
        free_probability: 0.35,
        inscribed_radius_m: config.footprint_radius_m,
        inflation_radius_m: config.footprint_radius_m + config.inflation_m,
        cost_scaling_factor: 5.0,
    };
    let mut costmap =
        Costmap::from_occupancy(&grid, &costmap_config).expect("valid costmap config");
    let lane = &grid;
    for y in 0..lane.height() as isize {
        for x in 0..lane.width() as isize {
            let coord = GridCoord { x, y };
            let p = lane.probability(coord).unwrap_or(0.5);
            if p > 0.35 && p < 0.95 {
                costmap.apply_cost(coord, config.off_lane_cost);
            }
        }
    }
    costmap
}

fn distance_to_polyline(points: &[[f64; 2]], x: f64, y: f64) -> f64 {
    points
        .windows(2)
        .map(|w| {
            let (ax, ay, bx, by) = (w[0][0], w[0][1], w[1][0], w[1][1]);
            let (dx, dy) = (bx - ax, by - ay);
            let l2 = (dx * dx + dy * dy).max(1e-12);
            let t = (((x - ax) * dx + (y - ay) * dy) / l2).clamp(0.0, 1.0);
            (x - ax - t * dx).hypot(y - ay - t * dy)
        })
        .fold(f64::INFINITY, f64::min)
}

/// Drives the robot through the street's goals on ground truth. `on_step` sees every step
/// (for rendering or logging).
pub fn drive(
    street: &Street,
    config: &NavConfig,
    mut on_step: impl FnMut(&RobotState),
) -> Result<DriveReport, StreetError> {
    drive_localized(street, config, None, None, |s, _| on_step(s))
}

/// Drives the robot through the street's goals. With `localize`, the robot steers on its own
/// estimate: wheel odometry with `config`'s errors, replaced by each fix that passes the gate
/// (every `fix_every_s`); without it, on ground truth. The robot starts at a known pose.
pub fn drive_localized(
    street: &Street,
    config: &NavConfig,
    mut localize: Option<&mut Localize<'_>>,
    mut crowd: Option<&mut Crowd>,
    mut on_step: impl FnMut(&RobotState, Option<&Crowd>),
) -> Result<DriveReport, StreetError> {
    let costmap = lane_costmap(street, config);
    let (planner, follow) = controllers(config);
    let (mut x, mut y, mut yaw) = (street.start[0], street.start[1], street.start[2]);
    let mut estimate = Pose2d::new(x, y, yaw);
    let mut report = DriveReport {
        goals: street.goals.len(),
        min_solid_clearance_m: f64::INFINITY,
        ..DriveReport::default()
    };
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut walk_distances = Vec::new();
    let mut errors = Vec::new();
    let mut t = 0.0;
    let mut speed = 0.0;
    let mut next_fix = 0.0;
    let mut held_s = 0.0;
    let mut last_replan = f64::NEG_INFINITY;
    let mut last_fix = 0.0;
    for (index, goal) in street.goals.iter().enumerate() {
        let mut path: Path2d = plan_path(
            &costmap,
            Vec3::new(estimate.x_m, estimate.y_m, 0.0),
            Vec3::new(goal[0], goal[1], 0.0),
            &planner,
        )
        .map_err(|e| StreetError::Plan(index, e.to_string()))?;
        let started = t;
        loop {
            let (drop, pitch) = street.kerbs.pose(x, y, yaw, config.wheelbase_m);
            let mut state = RobotState {
                t_s: t,
                x_m: x,
                y_m: y,
                yaw_rad: yaw,
                z_m: street.ground_at(x, y) + drop,
                pitch_rad: pitch,
                speed_m_s: speed,
                goal: index,
                estimate,
            };
            if let Some(localize) = localize.as_mut() {
                if t + 1e-9 >= next_fix {
                    next_fix += config.fix_every_s;
                    report.fixes_attempted += 1;
                    if let Some(fix) = localize(&state, estimate) {
                        if let Some(next) = accept_fix(config, estimate, fix, t - last_fix) {
                            estimate = next;
                            last_fix = t;
                            report.fixes_accepted += 1;
                        } else {
                            report.fixes_rejected += 1;
                        }
                    }
                    state.estimate = estimate;
                }
            } else {
                estimate = Pose2d::new(x, y, yaw);
                state.estimate = estimate;
            }
            if let Some(crowd) = crowd.as_deref_mut() {
                crowd.step(config.step_s, [x, y], &|px, py| street.solid_at(px, py));
            }
            on_step(&state, crowd.as_deref());
            for value in [x, y, yaw, estimate.x_m, estimate.y_m] {
                hash = (hash ^ value.to_bits()).wrapping_mul(0x0100_0000_01b3);
            }
            let result = pure_pursuit_follow(&path, estimate, &follow)
                .map_err(|e| StreetError::Plan(index, e.to_string()))?;
            if result.reached || t - started > config.goal_timeout_s {
                // the goal counts when the robot is really there
                let truly = (goal[0] - x).hypot(goal[1] - y) <= config.goal_tolerance_m + 0.2;
                report.reached += usize::from(result.reached && truly);
                break;
            }
            let on_edge = street.kerbs.on_edge(x, y, yaw, config.wheelbase_m);
            let mut v = result.command.linear_m_s;
            if on_edge {
                v = v.min(config.kerb_speed_m_s);
            }
            let mut w = result.command.angular_rad_s;
            if let Some(crowd) = crowd.as_deref() {
                (v, w, held_s) =
                    steer_round_people(crowd, config, [x, y, yaw], estimate, [v, w], held_s);
                if held_s > config.replan_after_s && t - last_replan > config.replan_after_s {
                    last_replan = t;
                    report.replans += 1;
                    let round = plan_round_people(
                        &costmap,
                        crowd,
                        [x, y, yaw],
                        estimate,
                        goal,
                        &planner,
                        config,
                    );
                    path = round.unwrap_or(path);
                }
            }
            let nyaw = yaw + w * config.step_s;
            let nx = x + v * config.step_s * nyaw.cos();
            let ny = y + v * config.step_s * nyaw.sin();
            let blocked_by_people = crowd
                .as_deref()
                .is_some_and(|c| touches_people(c, nx, ny, street.robot_radius_m));
            report.people_contacts += usize::from(blocked_by_people && v > 0.0);
            let moved = if blocked_by_people {
                speed = 0.0;
                0.0
            } else if street.hits_solid(nx, ny, street.robot_radius_m) {
                report.contacts += 1;
                yaw = nyaw;
                speed = 0.0;
                0.0
            } else {
                let d = (nx - x).hypot(ny - y);
                report.distance_m += d;
                (x, y, yaw) = (nx, ny, nyaw);
                speed = v;
                d * v.signum() // backing up: negative
            };
            estimate = if localize.is_some() {
                odometry_step(config, estimate, moved, w)
            } else {
                Pose2d::new(x, y, yaw) // on ground truth
            };
            if on_edge {
                report.kerb_time_s += config.step_s;
                report.kerb_max_speed_m_s = report.kerb_max_speed_m_s.max(speed);
            }
            if ((t / config.step_s).round() as u64).is_multiple_of(10) {
                sample_drive(
                    street,
                    &mut report,
                    &mut walk_distances,
                    &mut errors,
                    [x, y],
                    estimate,
                );
            }
            t += config.step_s;
        }
    }
    report.time_s = t;
    report.trajectory_hash = hash;
    finish_report(&mut report, walk_distances, errors);
    Ok(report)
}

/// Checks on the crowd during a drive, sampled every call to [`CrowdStats::sample`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CrowdStats {
    /// Samples taken.
    pub samples: usize,
    /// Footprint samples of people and bikes within 3.2 m of the road's centre line while not
    /// on the crossing.
    pub on_road: usize,
    /// Deepest any footprint came into the robot's drawn body (0.87 x 0.70 m; m, < 0 is a gap).
    pub body_intrusion_m: f64,
    /// Deepest a person's or bike's footprint came into a car's (m).
    pub car_intrusion_m: f64,
    /// Longest anyone stood still (s).
    pub longest_still_s: f64,
    still: Vec<f64>,
    last: Vec<[f64; 2]>,
}

impl CrowdStats {
    /// Takes one sample, `dt_s` after the last.
    pub fn sample(&mut self, crowd: &Crowd, robot: &RobotState, dt_s: f64) {
        if self.samples == 0 {
            self.body_intrusion_m = f64::NEG_INFINITY;
            self.car_intrusion_m = f64::NEG_INFINITY;
            self.still = vec![0.0; crowd.len()];
            self.last = (0..crowd.len()).map(|i| crowd.agent(i).1).collect();
        }
        self.samples += 1;
        let (s, c) = robot.yaw_rad.sin_cos();
        let circles = crowd.circles();
        let line = crowd
            .spec()
            .road_lines
            .as_ref()
            .and_then(|l| l.first().copied());
        for &(x, y, r, i) in &circles {
            let (dx, dy) = (x - robot.x_m, y - robot.y_m);
            let (u, v) = (c * dx + s * dy, -s * dx + c * dy);
            let (ex, ey) = ((u.abs() - 0.435).max(0.0), (v.abs() - 0.35).max(0.0));
            let depth = if ex > 0.0 || ey > 0.0 {
                r - ex.hypot(ey)
            } else {
                r + (0.435 - u.abs()).min(0.35 - v.abs())
            };
            self.body_intrusion_m = self.body_intrusion_m.max(depth);
            let kind = crowd.agent(i).0;
            if kind == crowd::CAR {
                continue;
            }
            if let Some(l) = line {
                let lateral = (l[4] * (y - l[3]) - l[5] * (x - l[2])).abs();
                if lateral < 3.2 && !crowd.is_crossing(i) {
                    self.on_road += 1;
                }
            }
            for &(cx, cy, cr, j) in &circles {
                if crowd.agent(j).0 == crowd::CAR {
                    self.car_intrusion_m =
                        self.car_intrusion_m.max(r + cr - (x - cx).hypot(y - cy));
                }
            }
        }
        for i in 0..crowd.len() {
            let xy = crowd.agent(i).1;
            let moved = (xy[0] - self.last[i][0]).hypot(xy[1] - self.last[i][1]) > 0.005;
            self.still[i] = if moved { 0.0 } else { self.still[i] + dt_s };
            self.longest_still_s = self.longest_still_s.max(self.still[i]);
            self.last[i] = xy;
        }
    }
}

/// The robot's command bent round the crowd (`rne_nav::avoid_velocities`): the people it
/// sees, from where it really is, put round its estimate, as its scan would; the closest
/// command that keeps its body clear of them for 2 s, or a stop.
fn avoid_people(
    crowd: &Crowd,
    config: &NavConfig,
    truth: [f64; 3],
    estimate: Pose2d,
    v: f64,
    w: f64,
    margin_m: f64,
) -> (f64, f64) {
    let (sd, cd) = (truth[2].sin(), truth[2].cos());
    let (se, ce) = (estimate.yaw_rad.sin(), estimate.yaw_rad.cos());
    let obstacles: Vec<CircularObstacle> = crowd
        .moving_circles()
        .into_iter()
        .filter(|c| (c.0 - truth[0]).hypot(c.1 - truth[1]) < 5.0)
        .map(|(x, y, r, vx, vy)| {
            // in the robot's frame, then round the estimate
            let (dx, dy) = (x - truth[0], y - truth[1]);
            let (fx, fy) = (cd * dx + sd * dy, -sd * dx + cd * dy);
            let (fvx, fvy) = (cd * vx + sd * vy, -sd * vx + cd * vy);
            CircularObstacle {
                center_m: Vec3::new(
                    estimate.x_m + ce * fx - se * fy,
                    estimate.y_m + se * fx + ce * fy,
                    0.0,
                ),
                velocity_m_s: Vec3::new(ce * fvx - se * fvy, se * fvx + ce * fvy, 0.0),
                radius_m: r,
            }
        })
        .collect();
    if obstacles.is_empty() {
        return (v, w);
    }
    let command = avoid_velocities(
        estimate,
        config.footprint_radius_m,
        &obstacles,
        VelocityCommand2d::new(v, w),
        v.max(0.05),
        1.0,
        &AvoidanceConfig {
            time_horizon_s: 1.5,
            safety_margin_m: margin_m,
            ..AvoidanceConfig::default()
        },
    );
    (command.linear_m_s, command.angular_rad_s)
}

/// The command round the crowd and the time held up. The longer the robot is held up, the
/// closer it lets people come (0.2 m margin, 0.05 m after 3 s, none after 6 s), as people
/// squeeze past each other. Backing up instead was worse: with people all round, it backed
/// into them, and runs deadlocked.
fn steer_round_people(
    crowd: &Crowd,
    config: &NavConfig,
    truth: [f64; 3],
    estimate: Pose2d,
    command: [f64; 2],
    held_s: f64,
) -> (f64, f64, f64) {
    let [v, w] = command;
    let margin = if held_s > 6.0 {
        0.0
    } else if held_s > 3.0 {
        0.05
    } else {
        0.2
    };
    let (nv, nw) = avoid_people(crowd, config, truth, estimate, v, w, margin);
    // held while it gets under half the speed it wants; a creep under the narrower margin
    // does not end it, or the margin would widen again and hold it once more
    let held = if v > 0.05 && nv < 0.5 * v {
        held_s + config.step_s
    } else {
        (held_s - 2.0 * config.step_s).max(0.0)
    };
    (nv, nw, held)
}

/// A plan from the estimate to the goal with the people near the robot as obstacles: their
/// footprints, seen from where it is and put round its estimate, grown by the robot's own
/// footprint. `None` when there is no way round.
fn plan_round_people(
    costmap: &Costmap,
    crowd: &Crowd,
    truth: [f64; 3],
    estimate: Pose2d,
    goal: &[f64; 3],
    planner: &GlobalPlannerConfig,
    config: &NavConfig,
) -> Option<Path2d> {
    if config.replan_after_s <= 0.0 {
        return None;
    }
    let mut blocked = costmap.clone();
    let (sd, cd) = (truth[2].sin(), truth[2].cos());
    let (se, ce) = (estimate.yaw_rad.sin(), estimate.yaw_rad.cos());
    let res = blocked.resolution_m();
    for (x, y, r, _, _) in crowd.moving_circles() {
        let (dx, dy) = (x - truth[0], y - truth[1]);
        if dx.hypot(dy) > 6.0 {
            continue;
        }
        let (fx, fy) = (cd * dx + sd * dy, -sd * dx + cd * dy);
        let (px, py) = (
            estimate.x_m + ce * fx - se * fy,
            estimate.y_m + se * fx + ce * fy,
        );
        let reach = r + config.footprint_radius_m + 0.05;
        let n = (reach / res).ceil() as isize;
        let Some(c) = blocked.world_to_grid(Vec3::new(px, py, 0.0)) else {
            continue;
        };
        for j in -n..=n {
            for i in -n..=n {
                if ((i * i + j * j) as f64).sqrt() * res <= reach {
                    blocked.apply_cost(
                        GridCoord {
                            x: c.x + i,
                            y: c.y + j,
                        },
                        COST_LETHAL,
                    );
                }
            }
        }
    }
    plan_path(
        &blocked,
        Vec3::new(estimate.x_m, estimate.y_m, 0.0),
        Vec3::new(goal[0], goal[1], 0.0),
        planner,
    )
    .ok()
}

/// Whether a circle of `radius_m` at (x, y) overlaps anyone's footprint.
fn touches_people(crowd: &Crowd, x: f64, y: f64, radius_m: f64) -> bool {
    crowd
        .circles()
        .iter()
        .any(|c| (c.0 - x).hypot(c.1 - y) < c.2 + radius_m)
}

/// The estimate after a fix `since_s` after the last accepted one, or `None` when the gate
/// rejects the fix.
fn accept_fix(config: &NavConfig, estimate: Pose2d, fix: Pose2d, since_s: f64) -> Option<Pose2d> {
    let gate = config.fix_gate_m + config.fix_gate_growth_m_s * since_s;
    if (fix.x_m - estimate.x_m).hypot(fix.y_m - estimate.y_m) > gate {
        return None;
    }
    let a = config.fix_blend;
    let dyaw = (fix.yaw_rad - estimate.yaw_rad + std::f64::consts::PI)
        .rem_euclid(std::f64::consts::TAU)
        - std::f64::consts::PI;
    Some(Pose2d::new(
        estimate.x_m + a * (fix.x_m - estimate.x_m),
        estimate.y_m + a * (fix.y_m - estimate.y_m),
        estimate.yaw_rad + a * dyaw,
    ))
}

/// One step of wheel odometry: the distance the wheels really moved (negative backwards) and
/// the commanded yaw rate, with `config`'s errors.
fn odometry_step(config: &NavConfig, estimate: Pose2d, moved_m: f64, w_rad_s: f64) -> Pose2d {
    let ov = moved_m / config.step_s * (1.0 + config.odom_scale_error);
    let moving = moved_m.abs() > 1e-6 || w_rad_s.abs() > 1e-6;
    let ow = w_rad_s * (1.0 + config.odom_yaw_scale_error)
        + if moving {
            config.odom_yaw_bias_rad_s
        } else {
            0.0
        };
    let yaw = estimate.yaw_rad + ow * config.step_s;
    Pose2d::new(
        estimate.x_m + ov * config.step_s * yaw.cos(),
        estimate.y_m + ov * config.step_s * yaw.sin(),
        yaw,
    )
}

/// The global planner and the path follower for `config`.
fn controllers(config: &NavConfig) -> (GlobalPlannerConfig, PurePursuitConfig) {
    (
        GlobalPlannerConfig {
            cost_weight: config.cost_weight,
            ..GlobalPlannerConfig::default()
        },
        PurePursuitConfig {
            lookahead_m: 0.8,
            max_linear_m_s: config.max_speed_m_s,
            max_angular_rad_s: 1.0,
            goal_tolerance_m: config.goal_tolerance_m,
            slow_radius_m: 0.75,
        },
    )
}

/// Every 0.2 s of a drive: clearance from solid cells, distance from the walk, and the
/// estimate's error.
fn sample_drive(
    street: &Street,
    report: &mut DriveReport,
    walk_distances: &mut Vec<f64>,
    errors: &mut Vec<f64>,
    truth: [f64; 2],
    estimate: Pose2d,
) {
    let [x, y] = truth;
    report.min_solid_clearance_m = report
        .min_solid_clearance_m
        .min(street.solid_clearance(x, y, 2.0));
    if street.walk.len() > 1 {
        walk_distances.push(distance_to_polyline(&street.walk, x, y));
    }
    errors.push((estimate.x_m - x).hypot(estimate.y_m - y));
}

fn finish_report(report: &mut DriveReport, mut walk: Vec<f64>, mut errors: Vec<f64>) {
    if !walk.is_empty() {
        walk.sort_by(f64::total_cmp);
        report.walk_distance_median_m = walk[walk.len() / 2];
        report.walk_distance_max_m = *walk.last().unwrap_or(&0.0);
    }
    if !errors.is_empty() {
        report.pose_error_rmse_m =
            (errors.iter().map(|e| e * e).sum::<f64>() / errors.len() as f64).sqrt();
        errors.sort_by(f64::total_cmp);
        report.pose_error_median_m = errors[errors.len() / 2];
        report.pose_error_max_m = *errors.last().unwrap_or(&0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RES: f64 = 0.1;

    /// A 30 x 6 m street: a 2 m lane along y = 0, a post beside it, a kerb crossing at x 12-16.
    fn street() -> Street {
        let (w, h) = (300usize, 60usize);
        let origin = Pose2d::new(-5.0, -3.0, 0.0);
        let lo = |p: f64| ((p / (1.0 - p)).ln() * 1000.0).round() as i16;
        let mut lane = vec![lo(0.9); w * h];
        let mut solid = vec![lo(0.001); w * h];
        for row in 0..h {
            for col in 0..w {
                let y = origin.y_m + row as f64 * RES;
                if y.abs() <= 1.0 {
                    lane[row * w + col] = lo(0.004);
                }
                if y > 2.6 {
                    lane[row * w + col] = lo(0.999); // a wall along the far side
                    solid[row * w + col] = lo(0.999);
                }
            }
        }
        // a post 0.9 m off the lane's centre line at x = 8
        let (pc, pr) = (
            ((8.0 - origin.x_m) / RES) as usize,
            ((0.9 - origin.y_m) / RES) as usize,
        );
        solid[pr * w + pc] = lo(0.999);
        lane[pr * w + pc] = lo(0.999);
        let grid = |v: Vec<i16>| OccupancyGrid::from_log_odds(w, h, RES, origin, v).unwrap();
        Street {
            dir: PathBuf::from("."),
            start: [0.0, 0.0, 0.0],
            goals: vec![[10.0, 0.0, 0.0], [22.0, 0.0, 0.0]],
            robot_radius_m: 0.2,
            camera_height_m: 1.2,
            walk: vec![[-5.0, 0.0], [25.0, 0.0]],
            kerbs: Kerbs::new(vec![Kerb {
                a: [12.0, 0.0],
                b: [16.0, 0.0],
                half_width: 3.0,
                drop: 0.1,
            }]),
            ground: None,
            robot_actor: None,
            solid: grid(solid),
            lane: grid(lane),
            credit: String::new(),
        }
    }

    #[test]
    fn map_to_rne_turns_z_up_into_y_up() {
        assert_eq!(map_to_rne(1.0, 2.0, 3.0), Vec3::new(1.0, 3.0, -2.0));
    }

    #[test]
    fn kerbs_drop_the_road_and_pitch_the_robot() {
        let kerbs = street().kerbs;
        assert_eq!(kerbs.drop_at(10.0, 0.0), 0.0);
        assert!((kerbs.drop_at(14.0, 0.0) - 0.1).abs() < 1e-12);
        assert_eq!(kerbs.drop_at(14.0, 3.5), 0.0); // beside the crossing
        let (_, pitch) = kerbs.pose(12.0, 0.0, 0.0, 0.5); // front wheels down the kerb
        assert!(pitch < -0.15, "{pitch}");
        assert!(kerbs.on_edge(11.75, 0.0, 0.0, 0.5)); // the front wheel on the kerb
        assert!(!kerbs.on_edge(14.0, 0.0, 0.0, 0.5));
    }

    #[test]
    fn robot_reaches_the_goals_on_the_lane_and_crawls_over_kerbs() {
        let street = street();
        let mut states = Vec::new();
        let report = drive(&street, &NavConfig::default(), |s| states.push(*s)).unwrap();
        assert_eq!(report.reached, 2, "{report:?}");
        assert_eq!(report.contacts, 0);
        assert!(report.walk_distance_max_m < 0.6, "{report:?}");
        // the post is 0.9 m off the walk: the footprint keeps the robot's centre clear of it
        assert!(report.min_solid_clearance_m > 0.45, "{report:?}");
        assert!(report.kerb_time_s > 1.0, "{report:?}");
        assert!(report.kerb_max_speed_m_s <= 0.12 + 1e-9, "{report:?}");
        let lowest = states.iter().map(|s| s.z_m).fold(f64::INFINITY, f64::min);
        assert!(
            (lowest + 0.1).abs() < 1e-9,
            "the road between the kerbs is 10 cm down"
        );
    }

    /// A stand-in localizer: the true pose with a seeded few-centimetre error, no fix on
    /// one attempt in four, and a wild one now and then.
    fn noisy_localizer() -> impl FnMut(&RobotState, Pose2d) -> Option<Pose2d> {
        let mut n: u64 = 0;
        move |truth, _prior| {
            n += 1;
            let r = |k: u64| (((n * 2_654_435_761 + k * 40_503) % 1000) as f64 / 1000.0) - 0.5;
            match n % 4 {
                0 => None,
                _ if n.is_multiple_of(37) => {
                    Some(Pose2d::new(truth.x_m + 3.0, truth.y_m, truth.yaw_rad))
                }
                _ => Some(Pose2d::new(
                    truth.x_m + 0.06 * r(1),
                    truth.y_m + 0.06 * r(2),
                    truth.yaw_rad + 0.02 * r(3),
                )),
            }
        }
    }

    #[test]
    fn robot_steers_on_its_estimate_with_fixes() {
        let street = street();
        let mut localize = noisy_localizer();
        let report = drive_localized(
            &street,
            &NavConfig::default(),
            Some(&mut localize),
            None,
            |_, _| {},
        )
        .unwrap();
        assert_eq!(report.reached, 2, "{report:?}");
        assert_eq!(report.contacts, 0);
        assert!(
            report.fixes_rejected > 0,
            "the wild fixes are gated: {report:?}"
        );
        assert!(report.pose_error_rmse_m < 0.1, "{report:?}");
    }

    #[test]
    fn odometry_alone_drifts() {
        let street = street();
        let mut never = |_: &RobotState, _: Pose2d| None;
        let report = drive_localized(
            &street,
            &NavConfig::default(),
            Some(&mut never),
            None,
            |_, _| {},
        )
        .unwrap();
        assert!(report.pose_error_max_m > 0.3, "{report:?}");
    }

    #[test]
    fn a_drive_replays_bit_for_bit() {
        let street = street();
        let a = drive(&street, &NavConfig::default(), |_| {}).unwrap();
        let b = drive(&street, &NavConfig::default(), |_| {}).unwrap();
        assert_eq!(a, b);
    }
}
