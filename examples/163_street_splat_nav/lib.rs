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

use rne_math::Vec3;
use rne_nav::{
    from_map_json, plan_path, pure_pursuit_follow, Costmap, CostmapConfig, GlobalPlannerConfig,
    GridCoord, OccupancyGrid, Path2d, Pose2d, PurePursuitConfig,
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
            credit: raw.credit.unwrap_or_default(),
        })
    }

    /// The splat manifest.
    pub fn manifest(&self) -> PathBuf {
        self.dir.join("street.rne.splat.toml")
    }

    /// Ground height at (x, y), with the road between kerbs lowered.
    pub fn ground_at(&self, x_m: f64, y_m: f64) -> f64 {
        self.ground.as_ref().map_or(0.0, |g| g.height_at(x_m, y_m))
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
}

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
    let costmap = lane_costmap(street, config);
    let planner = GlobalPlannerConfig {
        cost_weight: config.cost_weight,
        ..GlobalPlannerConfig::default()
    };
    let follow = PurePursuitConfig {
        lookahead_m: 0.8,
        max_linear_m_s: config.max_speed_m_s,
        max_angular_rad_s: 1.0,
        goal_tolerance_m: config.goal_tolerance_m,
        slow_radius_m: 0.75,
    };
    let (mut x, mut y, mut yaw) = (street.start[0], street.start[1], street.start[2]);
    let mut report = DriveReport {
        goals: street.goals.len(),
        min_solid_clearance_m: f64::INFINITY,
        ..DriveReport::default()
    };
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut walk_distances = Vec::new();
    let mut t = 0.0;
    let mut speed = 0.0;
    for (index, goal) in street.goals.iter().enumerate() {
        let path: Path2d = plan_path(
            &costmap,
            Vec3::new(x, y, 0.0),
            Vec3::new(goal[0], goal[1], 0.0),
            &planner,
        )
        .map_err(|e| StreetError::Plan(index, e.to_string()))?;
        let started = t;
        loop {
            let result = pure_pursuit_follow(&path, Pose2d::new(x, y, yaw), &follow)
                .map_err(|e| StreetError::Plan(index, e.to_string()))?;
            let on_edge = street.kerbs.on_edge(x, y, yaw, config.wheelbase_m);
            let mut v = result.command.linear_m_s;
            if on_edge {
                v = v.min(config.kerb_speed_m_s);
            }
            let w = result.command.angular_rad_s;
            let (drop, pitch) = street.kerbs.pose(x, y, yaw, config.wheelbase_m);
            let state = RobotState {
                t_s: t,
                x_m: x,
                y_m: y,
                yaw_rad: yaw,
                z_m: street.ground_at(x, y) + drop,
                pitch_rad: pitch,
                speed_m_s: speed,
                goal: index,
            };
            on_step(&state);
            for value in [x, y, yaw] {
                hash = (hash ^ value.to_bits()).wrapping_mul(0x0100_0000_01b3);
            }
            if result.reached || t - started > config.goal_timeout_s {
                report.reached += usize::from(result.reached);
                break;
            }
            let nyaw = yaw + w * config.step_s;
            let nx = x + v * config.step_s * nyaw.cos();
            let ny = y + v * config.step_s * nyaw.sin();
            if street.hits_solid(nx, ny, street.robot_radius_m) {
                report.contacts += 1;
                yaw = nyaw;
                speed = 0.0;
            } else {
                report.distance_m += (nx - x).hypot(ny - y);
                (x, y, yaw) = (nx, ny, nyaw);
                speed = v;
            }
            if on_edge {
                report.kerb_time_s += config.step_s;
                report.kerb_max_speed_m_s = report.kerb_max_speed_m_s.max(speed);
            }
            if ((t / config.step_s).round() as u64).is_multiple_of(10) {
                report.min_solid_clearance_m = report
                    .min_solid_clearance_m
                    .min(street.solid_clearance(x, y, 2.0));
                if street.walk.len() > 1 {
                    walk_distances.push(distance_to_polyline(&street.walk, x, y));
                }
            }
            t += config.step_s;
        }
    }
    report.time_s = t;
    report.trajectory_hash = hash;
    if !walk_distances.is_empty() {
        walk_distances.sort_by(f64::total_cmp);
        report.walk_distance_median_m = walk_distances[walk_distances.len() / 2];
        report.walk_distance_max_m = *walk_distances.last().unwrap_or(&0.0);
    }
    Ok(report)
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

    #[test]
    fn a_drive_replays_bit_for_bit() {
        let street = street();
        let a = drive(&street, &NavConfig::default(), |_| {}).unwrap();
        let b = drive(&street, &NavConfig::default(), |_| {}).unwrap();
        assert_eq!(a, b);
    }
}
