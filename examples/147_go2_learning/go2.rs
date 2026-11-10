//! The Go2 plant and the walking episode both engines run.

use rne_ai::{PdJointConfig, PdJointWalker, PolicyWalkPlant, UrdfSceneSim};
use rne_core::SimDuration;
use rne_math::{Quat, Vec3};
use serde_json::{json, Value};

use crate::Result;

/// Actuated joints' child links, in action order (unitree_rl_gym's).
pub(crate) const LINKS: [&str; 12] = [
    "FL_hip", "FL_thigh", "FL_calf", "FR_hip", "FR_thigh", "FR_calf", "RL_hip", "RL_thigh",
    "RL_calf", "RR_hip", "RR_thigh", "RR_calf",
];
/// unitree_rl_gym's Go2 default joint angles.
pub(crate) const DEFAULT_RAD: [f64; 12] = [
    0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0, -1.5,
];
/// unitree_rl_gym's Go2 PD gains.
pub(crate) const KP_NM_PER_RAD: f64 = 20.0;
pub(crate) const KD_NM_S_PER_RAD: f64 = 0.5;
/// Physics step and steps per control period (50 Hz control).
pub(crate) const DT_S: f64 = 0.005;
pub(crate) const DECIMATION: usize = 4;
pub(crate) const CONTROL_DT_S: f64 = DT_S * DECIMATION as f64;
/// Control periods the robot stands on its default pose before an episode,
/// after it starts in that pose, at rest, with its base this high.
pub(crate) const SETTLE_STEPS: usize = 50;
pub(crate) const SPAWN_HEIGHT_M: f64 = 0.36;
/// Control periods in an episode (5 s).
pub(crate) const EPISODE_STEPS: usize = 250;
/// Period of the gait clock the policy observes.
pub(crate) const GAIT_PERIOD_S: f64 = 0.5;
/// Largest target offset from the default pose.
pub(crate) const ACTION_SCALE_RAD: f64 = 0.5;
/// The robot has fallen when its base is this low, or tilted more than 60°.
pub(crate) const FALL_HEIGHT_M: f64 = 0.10;
pub(crate) const FALL_GRAVITY_Z: f64 = -0.5;
/// Proprioceptive observation width: angular velocity, gravity, joint
/// positions and velocities, previous action, gait clock.
pub(crate) const PROPRIOCEPTION: usize = 3 + 3 + 12 + 12 + 12 + 2;
/// Height scan (`--height-scan`): vertical rays at these offsets from the
/// base, forward and to the left in its heading frame (roll and pitch
/// ignored), x outer and y inner, appended to the proprioception.
pub(crate) const HEIGHT_SCAN_X_M: [f64; 5] = [-0.2, 0.0, 0.2, 0.4, 0.6];
pub(crate) const HEIGHT_SCAN_Y_M: [f64; 3] = [-0.15, 0.0, 0.15];
pub(crate) const HEIGHT_SCAN_POINTS: usize = HEIGHT_SCAN_X_M.len() * HEIGHT_SCAN_Y_M.len();
/// Each reading is the base's height above the ray's hit, less this nominal
/// stance height, times this gain, clipped to [-1, 1]: 0 under a standing
/// robot on level ground, about ±0.1 over the terrain's 2 cm bumps.
pub(crate) const HEIGHT_SCAN_NOMINAL_M: f64 = 0.3;
pub(crate) const HEIGHT_SCAN_GAIN_PER_M: f64 = 5.0;
/// Random starts (`--random-starts`): every generation's episodes start at a
/// point drawn uniformly from this box on the terrain, so a policy cannot
/// learn one path by heart; it leaves room to walk 10 m forward and 4 m
/// sideways before the terrain's 12 m edge.
pub(crate) const START_X_RANGE_M: [f64; 2] = [-8.0, 2.0];
pub(crate) const START_Y_RANGE_M: [f64; 2] = [-8.0, 8.0];
/// Mixed into the ES seed for the training starts' stream, and the seed of
/// the held-out evaluation starts (`--evaluate --starts N`).
pub(crate) const START_STREAM: u64 = 0x5354_4152_5453_0147;
pub(crate) const EVALUATION_STARTS_SEED: u64 = 0x4556_414C_0147;
/// Fitness weights: per metre sideways, per radian of heading, per fall and
/// on the mean squared action change per step.
pub(crate) const LATERAL_PENALTY: f64 = 0.5;
pub(crate) const HEADING_PENALTY: f64 = 0.5;
pub(crate) const FALL_PENALTY: f64 = 1.0;
pub(crate) const SMOOTHNESS_PENALTY: f64 = 0.02;
/// Rough terrain (`--terrain SEED`): fractal heights over a square this wide,
/// sampled every 10 cm, with `rne_locomotion`'s episode relief (0.5 cycles
/// per metre, 4 cm peak first octave), shifted to height zero at the origin.
pub(crate) const TERRAIN_SIZE_M: f64 = 24.0;
pub(crate) const TERRAIN_SAMPLES: u32 = 241;
pub(crate) const TERRAIN_FREQUENCY_PER_M: f64 = 0.5;
pub(crate) const TERRAIN_AMPLITUDE_M: f64 = 0.04;
/// Mixed into the terrain seed for the gaps' stream.
pub(crate) const GAP_STREAM: u64 = 0x4741_5053_0147;
/// How far from a gap's edge a start (and the origin) stays: farther than
/// a foot reaches from the base, plus a cell of the height field.
pub(crate) const START_GAP_CLEARANCE_M: f64 = 0.45;

/// Trenches across the terrain (`--gap-depth`, `--gap-width`,
/// `--gap-spacing`): each runs the terrain's whole width in y, at a random
/// width, after a random run of floor along x. A depth of zero, the default,
/// leaves the terrain without gaps.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Gaps {
    pub(crate) depth_m: f64,
    pub(crate) width_m: [f64; 2],
    pub(crate) spacing_m: [f64; 2],
}

impl Default for Gaps {
    fn default() -> Self {
        Self {
            depth_m: 0.0,
            width_m: [0.25, 0.35],
            spacing_m: [0.8, 1.6],
        }
    }
}

/// The terrain's first-octave frequency and amplitude
/// (`--terrain-frequency`, `--terrain-amplitude`) and its side
/// (`--terrain-size`, sampled every 10 cm); the defaults are
/// [`TERRAIN_FREQUENCY_PER_M`], [`TERRAIN_AMPLITUDE_M`] and
/// [`TERRAIN_SIZE_M`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Relief {
    pub(crate) frequency_per_m: f64,
    pub(crate) amplitude_m: f64,
    pub(crate) size_m: f64,
    /// The height field's sample spacing (`--terrain-cell`), 10 cm by default.
    pub(crate) cell_m: f64,
    pub(crate) gaps: Gaps,
}

impl Default for Relief {
    fn default() -> Self {
        Self {
            frequency_per_m: TERRAIN_FREQUENCY_PER_M,
            amplitude_m: TERRAIN_AMPLITUDE_M,
            size_m: TERRAIN_SIZE_M,
            cell_m: TERRAIN_SIZE_M / f64::from(TERRAIN_SAMPLES - 1),
            gaps: Gaps::default(),
        }
    }
}

impl Relief {
    /// Samples along each side: one every `cell_m`, 241 for the default 24 m
    /// at 10 cm.
    pub(crate) fn samples(self) -> u32 {
        (self.size_m / self.cell_m).round() as u32 + 1
    }
}

/// The gaps of the terrain of `seed`, as `[start, end]` along x in metres,
/// west to east: none within [`START_GAP_CLEARANCE_M`] of the origin or
/// within [`ARENA_EDGE_MARGIN_M`] of the terrain's edge.
pub(crate) fn gap_intervals(seed: u64, relief: Relief) -> Vec<[f64; 2]> {
    let gaps = relief.gaps;
    if gaps.depth_m <= 0.0 {
        return Vec::new();
    }
    let mut stream = crate::es::Normal(seed ^ GAP_STREAM);
    let lerp = |[low, high]: [f64; 2], t: f64| low + (high - low) * t;
    let half = 0.5 * relief.size_m;
    let mut x = -half + ARENA_EDGE_MARGIN_M;
    let mut intervals = Vec::new();
    loop {
        x += lerp(gaps.spacing_m, stream.uniform());
        let width = lerp(gaps.width_m, stream.uniform());
        if x + width > half - ARENA_EDGE_MARGIN_M {
            return intervals;
        }
        if x + width < -START_GAP_CLEARANCE_M || x > START_GAP_CLEARANCE_M {
            intervals.push([x, x + width]);
        }
        x += width;
    }
}

/// A report's plant. The terrain's sample spacing and gaps appear only when
/// they differ from the defaults, so reports without them read as before.
pub(crate) fn plant_json(plant: Plant) -> Value {
    let mut value = json!({"backend": plant.backend.name(),
                           "solver_iterations": plant.solver_iterations,
                           "inner_passes": plant.inner_passes,
                           "floor_friction": plant.floor_friction,
                           "terrain_seed": plant.terrain_seed,
                           "terrain_frequency_per_m": plant.relief.frequency_per_m,
                           "terrain_amplitude_m": plant.relief.amplitude_m,
                           "terrain_size_m": plant.relief.size_m});
    let default = Relief::default();
    if plant.relief.cell_m != default.cell_m {
        value["terrain_cell_m"] = json!(plant.relief.cell_m);
    }
    if plant.relief.gaps.depth_m > 0.0 {
        let gaps = plant.relief.gaps;
        value["gaps"] = json!({"depth_m": gaps.depth_m, "width_m": gaps.width_m,
                               "spacing_m": gaps.spacing_m});
    }
    value
}

/// The gaps of `plant`'s terrain; none on a flat floor.
pub(crate) fn plant_gaps(plant: Plant) -> Vec<[f64; 2]> {
    plant
        .terrain_seed
        .map_or_else(Vec::new, |seed| gap_intervals(seed, plant.relief))
}

/// Which RNE backend simulates the robot.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Backend {
    /// Rapier through `UrdfSceneSim` and `PdJointWalker`.
    Rapier,
    /// `rne_physics_native`: `rne_dynamics`' articulated hard-contact stepping.
    Native,
    /// `rne_locomotion`'s terrain step: `rne_dynamics`' hard-contact stepping
    /// with every URDF collision shape as spheres and points, on a flat floor
    /// or rough terrain.
    Locomotion,
}

impl Backend {
    pub(crate) fn parse(text: &str) -> Result<Self> {
        match text {
            "rapier" => Ok(Self::Rapier),
            "native" => Ok(Self::Native),
            "locomotion" => Ok(Self::Locomotion),
            other => Err(format!("unknown backend {other} (rapier, native or locomotion)").into()),
        }
    }

    pub(crate) fn name(self) -> &'static str {
        match self {
            Self::Rapier => "RNE Rapier",
            Self::Native => "RNE native",
            Self::Locomotion => "RNE locomotion",
        }
    }
}

/// RNE plant settings.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Plant {
    pub(crate) backend: Backend,
    /// Rapier's solver iterations; the native backend has none to set.
    pub(crate) solver_iterations: usize,
    /// Rapier's inner constraint passes per solver iteration.
    pub(crate) inner_passes: usize,
    pub(crate) floor_friction: f64,
    /// Seed of the rough terrain, or a flat floor.
    pub(crate) terrain_seed: Option<u64>,
    /// The rough terrain's relief.
    pub(crate) relief: Relief,
}

impl Default for Plant {
    fn default() -> Self {
        Self {
            backend: Backend::Rapier,
            solver_iterations: 4,
            inner_passes: 1,
            floor_friction: 1.0,
            terrain_seed: None,
            relief: Relief::default(),
        }
    }
}

/// What the policy observes, and where episodes start.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct Task {
    /// Append the height scan to the proprioception.
    pub(crate) height_scan: bool,
    /// Start every generation at its own point of the terrain.
    pub(crate) random_starts: bool,
}

impl Task {
    /// The policy's input width.
    pub(crate) fn observations(self) -> usize {
        PROPRIOCEPTION
            + if self.height_scan {
                HEIGHT_SCAN_POINTS
            } else {
                0
            }
    }

    pub(crate) fn to_json(self) -> Value {
        json!({"height_scan": self.height_scan, "random_starts": self.random_starts})
    }

    /// A report's task; reports from before tasks existed had neither.
    pub(crate) fn from_json(value: &Value) -> Self {
        let flag = |key: &str| value[key].as_bool().unwrap_or(false);
        Self {
            height_scan: flag("height_scan"),
            random_starts: flag("random_starts"),
        }
    }
}

/// `count` start points uniform in the start box, from SplitMix64 `seed`,
/// skipping any within [`START_GAP_CLEARANCE_M`] of a gap: without gaps,
/// the stream's first `2 count` uniforms.
fn start_points(seed: u64, count: usize, gaps: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let mut stream = crate::es::Normal(seed);
    let lerp = |[low, high]: [f64; 2], t: f64| low + (high - low) * t;
    let clear = |x: f64| {
        gaps.iter().all(|[start, end]| {
            x < start - START_GAP_CLEARANCE_M || x > end + START_GAP_CLEARANCE_M
        })
    };
    let mut points = Vec::with_capacity(count);
    while points.len() < count {
        let (u, v) = (stream.uniform(), stream.uniform());
        let point = [lerp(START_X_RANGE_M, u), lerp(START_Y_RANGE_M, v)];
        if clear(point[0]) {
            points.push(point);
        }
    }
    points
}

/// Each generation's start for ES seed `seed`, generation 0 first, clear of
/// `gaps`.
pub(crate) fn training_starts(seed: u64, generations: usize, gaps: &[[f64; 2]]) -> Vec<[f64; 2]> {
    start_points(seed ^ START_STREAM, generations + 1, gaps)
}

/// The held-out starts every policy is evaluated from, clear of `gaps`.
pub(crate) fn evaluation_starts(count: usize, gaps: &[[f64; 2]]) -> Vec<[f64; 2]> {
    start_points(EVALUATION_STARTS_SEED, count, gaps)
}

/// What an episode needs of a simulated Go2, in the Z-up world.
pub(crate) trait Robot {
    /// Back to the start: default pose and targets, at rest, base at
    /// [`SPAWN_HEIGHT_M`].
    fn restart(&mut self) -> Result<()>;
    /// [`Self::restart`] with the base above `start` instead of the origin;
    /// only a backend with a terrain has anywhere else to start.
    fn restart_at(&mut self, start: [f64; 2]) -> Result<()> {
        if start != [0.0, 0.0] {
            return Err("random starts need --backend locomotion and a terrain".into());
        }
        self.restart()
    }
    fn set_targets_rad(&mut self, targets_rad: &[f64; 12]) -> Result<()>;
    /// One control period of PD-controlled physics steps.
    fn step(&mut self) -> Result<()>;
    fn base_pose_z_up(&self) -> (Vec3, Quat);
    /// Linear and angular velocity in the world frame.
    fn base_velocity_z_up(&self) -> (Vec3, Vec3);
    /// Positions (rad) and velocities (rad/s) in [`LINKS`] order.
    fn joint_states(&self) -> Result<[(f64, f64); 12]>;
    /// Height of the ground below `(x, y)` in the Z-up world, in meters.
    fn ground_height_m(&self, _x_m: f64, _y_m: f64) -> f64 {
        0.0
    }
    /// Half the side of the square around the origin the base must stay
    /// inside: a robot that walks out of it would put a foot off the floor's
    /// edge, so the episode ends there as a fall.
    fn arena_half_m(&self) -> f64 {
        f64::INFINITY
    }
    /// By link: the ground's normal impulse through it (N·s) and the physics
    /// steps it carried any, summed since the robot was built; `None` where
    /// the backend does not report contacts.
    fn contact_load(&self) -> Option<Vec<(String, f64, usize)>> {
        None
    }
}

/// How far inside the floor's edge the arena ends, in meters: farther than a
/// foot reaches from the base, or the base moves in a control period.
pub(crate) const ARENA_EDGE_MARGIN_M: f64 = 1.0;

/// The Go2 on Rapier.
pub(crate) struct RapierGo2(PdJointWalker);

impl Robot for RapierGo2 {
    fn restart(&mut self) -> Result<()> {
        self.0.reset()?;
        self.0.set_joint_positions_rad(&DEFAULT_RAD)?;
        Ok(())
    }

    fn set_targets_rad(&mut self, targets_rad: &[f64; 12]) -> Result<()> {
        Ok(self.0.set_targets_rad(targets_rad)?)
    }

    fn step(&mut self) -> Result<()> {
        self.0.step()?;
        Ok(())
    }

    fn base_pose_z_up(&self) -> (Vec3, Quat) {
        self.0.base_pose_z_up()
    }

    fn base_velocity_z_up(&self) -> (Vec3, Vec3) {
        self.0.base_velocity_z_up()
    }

    fn joint_states(&self) -> Result<[(f64, f64); 12]> {
        let states = self.0.joint_states()?;
        Ok(std::array::from_fn(|index| states[index]))
    }
}

/// A Go2 on `plant`'s backend, started (but not yet settled).
pub(crate) fn robot(plant: Plant) -> Result<Box<dyn Robot>> {
    if plant.terrain_seed.is_some() && plant.backend != Backend::Locomotion {
        return Err("rough terrain needs --backend locomotion".into());
    }
    Ok(match plant.backend {
        Backend::Rapier => {
            let mut robot = RapierGo2(walker(plant)?);
            robot.restart()?;
            Box::new(robot)
        }
        Backend::Native => Box::new(crate::native::NativeGo2::new(plant.floor_friction)?),
        Backend::Locomotion => Box::new(crate::locomotion::LocomotionGo2::new(
            plant.floor_friction,
            plant.terrain_seed,
            plant.relief,
        )?),
    })
}

/// A Go2 walker on the scene whose fixed URDF children (feet, lower-calf
/// bumpers, head) are fused into their links and whose links carry the URDF's
/// declared masses, as in MuJoCo.
fn walker(plant: Plant) -> Result<PdJointWalker> {
    let sim = UrdfSceneSim::from_scene_path_with_solver_iterations(
        &UrdfSceneSim::unitree_go2_fused_scene_path(),
        plant.solver_iterations,
    )?;
    let config = PdJointConfig {
        link_names: LINKS.iter().map(|name| name.to_string()).collect(),
        kp_nm_per_rad: vec![KP_NM_PER_RAD; 12],
        kd_nm_s_per_rad: vec![KD_NM_S_PER_RAD; 12],
        default_joint_rad: DEFAULT_RAD.to_vec(),
        control_period: SimDuration::from_ticks((CONTROL_DT_S * 1e9).round() as u64),
        decimation: DECIMATION,
        plant: PolicyWalkPlant {
            armature_kg_m2: 0.0,
            joint_damping_nm_s_per_rad: 0.0,
            joint_friction_nm: 0.0,
            joint_friction_transition_rad_s: 0.05,
            floor_friction: plant.floor_friction,
        },
    };
    let mut walker = PdJointWalker::new(sim, config)?;
    if !walker.set_inner_solver_passes(plant.inner_passes) {
        return Err("inner solver passes must be positive".into());
    }
    Ok(walker)
}

/// The Rapier Go2's total mass: every dynamic body in its scene.
pub(crate) fn rapier_mass_kg(plant: Plant) -> Result<f64> {
    use rne_physics::{RigidBody, RigidBodyType};
    let walker = walker(plant)?;
    Ok(walker
        .sim()
        .world()
        .iter_entities()
        .filter_map(|entity| entity.get::<RigidBody>())
        .filter(|body| body.body_type == RigidBodyType::Dynamic)
        .map(|body| body.mass_kg)
        .sum())
}

/// Restarts the robot above `start` and stands it on its default pose for
/// [`SETTLE_STEPS`] control periods.
pub(crate) fn settle(robot: &mut dyn Robot, start: [f64; 2]) -> Result<()> {
    robot.restart_at(start)?;
    robot.set_targets_rad(&DEFAULT_RAD)?;
    for _ in 0..SETTLE_STEPS {
        robot.step()?;
    }
    Ok(())
}

/// The rough terrain for `seed`: a Y-up height field, zero at the origin.
pub(crate) fn terrain(seed: u64, relief: Relief) -> Result<rne_physics::ColliderShape> {
    let spec = rne_physics::FractalTerrain {
        size_x_m: relief.size_m,
        size_z_m: relief.size_m,
        columns: relief.samples(),
        rows: relief.samples(),
        frequency_per_m: relief.frequency_per_m,
        amplitude_m: relief.amplitude_m,
        ..rne_physics::FractalTerrain::default()
    };
    let raw = spec.height_field(seed)?;
    let origin = rne_physics::height_field_surface(&raw, 0.0, 0.0)
        .ok_or("the terrain does not cover the origin")?
        .height_m;
    let shifted = rne_physics::FractalTerrain {
        height_offset_m: -origin,
        ..spec
    };
    let gaps = gap_intervals(seed, relief);
    if gaps.is_empty() {
        return Ok(shifted.height_field(seed)?);
    }
    // Lower every column of samples whose x lies in a gap: the floor between
    // a gap's last sample and the next sample outside it slopes across one
    // cell.
    let mut heights = shifted.heights_m(seed)?;
    let columns = spec.columns as usize;
    let dx = spec.size_x_m / (columns - 1) as f64;
    for column in 0..columns {
        let x = -0.5 * spec.size_x_m + column as f64 * dx;
        if gaps.iter().any(|[start, end]| x >= *start && x <= *end) {
            for row in 0..spec.rows as usize {
                heights[row * columns + column] -= relief.gaps.depth_m;
            }
        }
    }
    Ok(rne_physics::ColliderShape::HeightField {
        nrows: spec.rows,
        ncols: spec.columns,
        heights_m: std::sync::Arc::from(heights),
        scale: Vec3::new(spec.size_x_m, 1.0, spec.size_z_m),
    })
}

/// Height of the Y-up `terrain` below Z-up `(x, y)`, in meters.
pub(crate) fn terrain_height_m(terrain: &rne_physics::ColliderShape, x_m: f64, y_m: f64) -> f64 {
    // Z-up (x, y) is Y-up (x, -z).
    rne_physics::height_field_surface(terrain, x_m, -y_m).map_or(f64::NEG_INFINITY, |s| s.height_m)
}

/// The terrain for `seed` in the Z-up frame MuJoCo's height field uses:
/// row `r` at `y = -size/2 + r size/(rows - 1)`, column `c` at
/// `x = -size/2 + c size/(columns - 1)`, and the heights at a few probe
/// points, so another engine can check its copy.
pub(crate) fn terrain_json(seed: u64, relief: Relief) -> Result<Value> {
    let shape = terrain(seed, relief)?;
    let rne_physics::ColliderShape::HeightField {
        nrows,
        ncols,
        heights_m,
        scale,
    } = &shape
    else {
        return Err("a fractal terrain is a height field".into());
    };
    let (rows, columns) = (*nrows as usize, *ncols as usize);
    // Y-up row r lies at z = -size/2 + r dz, that is at Z-up y = -z: the
    // Z-up rows run the other way.
    let heights: Vec<f64> = (0..rows)
        .rev()
        .flat_map(|row| (0..columns).map(move |column| (row, column)))
        .map(|(row, column)| heights_m[row * columns + column] * scale.y)
        .collect();
    let mut normal = crate::es::Normal(seed);
    let probes: Vec<[f64; 3]> = (0..40)
        .map(|_| {
            let x = (normal.sample() * 3.0).clamp(-10.0, 10.0);
            let y = (normal.sample() * 3.0).clamp(-10.0, 10.0);
            [x, y, terrain_height_m(&shape, x, y)]
        })
        .collect();
    // The height scan at a few poses on the terrain, so another engine can
    // check its scan too.
    let scan_probes: Vec<Value> = (0..10)
        .map(|_| {
            let x = (normal.sample() * 3.0).clamp(-8.0, 8.0);
            let y = (normal.sample() * 3.0).clamp(-8.0, 8.0);
            let heading = normal.sample();
            let base = Vec3::new(x, y, terrain_height_m(&shape, x, y) + 0.28);
            let readings = height_scan(base, heading, |x, y| terrain_height_m(&shape, x, y));
            json!({"base_m": base.to_array(), "heading_rad": heading, "readings": readings.to_vec()})
        })
        .collect();
    Ok(json!({
        "seed": seed,
        "frame": "z-up: row r at y = -size/2 + r size/(rows-1), column c at x = -size/2 + c size/(columns-1)",
        "size_m": relief.size_m,
        "rows": rows,
        "columns": columns,
        "frequency_per_m": relief.frequency_per_m,
        "amplitude_m": relief.amplitude_m,
        "gaps_m": gap_intervals(seed, relief),
        "gap_depth_m": relief.gaps.depth_m,
        "heights_m": heights,
        "probes": probes,
        "height_scan": {
            "x_m": HEIGHT_SCAN_X_M, "y_m": HEIGHT_SCAN_Y_M,
            "nominal_m": HEIGHT_SCAN_NOMINAL_M, "gain_per_m": HEIGHT_SCAN_GAIN_PER_M,
            "probes": scan_probes,
        },
    }))
}

/// Heading about +z of a Z-up orientation.
pub(crate) fn yaw(rotation: Quat) -> f64 {
    let Quat { x, y, z, w } = rotation;
    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}

/// The height scan's readings around a base at `position` facing `heading`,
/// where `ground` gives the terrain height below a point.
pub(crate) fn height_scan(
    position: Vec3,
    heading_rad: f64,
    ground: impl Fn(f64, f64) -> f64,
) -> [f64; HEIGHT_SCAN_POINTS] {
    let (sin, cos) = heading_rad.sin_cos();
    let mut readings = [0.0; HEIGHT_SCAN_POINTS];
    for (row, x) in HEIGHT_SCAN_X_M.iter().enumerate() {
        for (column, y) in HEIGHT_SCAN_Y_M.iter().enumerate() {
            let hit = ground(
                position.x + cos * x - sin * y,
                position.y + sin * x + cos * y,
            );
            readings[row * HEIGHT_SCAN_Y_M.len() + column] =
                ((position.z - hit - HEIGHT_SCAN_NOMINAL_M) * HEIGHT_SCAN_GAIN_PER_M)
                    .clamp(-1.0, 1.0);
        }
    }
    readings
}

/// The policy's observation of `robot` at control step `step` of an
/// episode, after the previous `action`.
pub(crate) fn observe(
    robot: &dyn Robot,
    task: Task,
    step: usize,
    action: &[f64; 12],
) -> Result<Vec<f64>> {
    let (position, rotation) = robot.base_pose_z_up();
    let (_, spin_world) = robot.base_velocity_z_up();
    let inverse = rotation.inverse();
    let spin = inverse * spin_world;
    let gravity = inverse * Vec3::new(0.0, 0.0, -1.0);
    let joints = robot.joint_states()?;
    let mut observation = vec![0.0; task.observations()];
    observation[..3].copy_from_slice(&[spin.x * 0.25, spin.y * 0.25, spin.z * 0.25]);
    observation[3..6].copy_from_slice(&[gravity.x, gravity.y, gravity.z]);
    for (index, (position, velocity)) in joints.iter().enumerate() {
        observation[6 + index] = position - DEFAULT_RAD[index];
        observation[18 + index] = velocity * 0.05;
    }
    observation[30..42].copy_from_slice(action);
    let phase = (step as f64 * CONTROL_DT_S % GAIT_PERIOD_S) / GAIT_PERIOD_S;
    let angle = std::f64::consts::TAU * phase;
    observation[42] = angle.sin();
    observation[43] = angle.cos();
    if task.height_scan {
        let readings = height_scan(position, yaw(rotation), |x, y| robot.ground_height_m(x, y));
        observation[PROPRIOCEPTION..].copy_from_slice(&readings);
    }
    Ok(observation)
}

/// How an episode went.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct Outcome {
    pub(crate) fitness: f64,
    /// Base origin's forward and sideways travel, and final heading.
    pub(crate) forward_m: f64,
    pub(crate) lateral_m: f64,
    pub(crate) heading_rad: f64,
    /// Seconds walked before a fall, or the full episode.
    pub(crate) survived_s: f64,
    pub(crate) fell: bool,
    /// Control periods simulated, settling included.
    pub(crate) control_steps: usize,
}

/// One control period of an episode, for a learner that needs rewards.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Step {
    /// This period's share of the episode's fitness: the episode's rewards
    /// sum to its fitness, except that the smoothness penalty divides by
    /// [`EPISODE_STEPS`] where the fitness divides by the periods walked.
    pub(crate) reward: f64,
    /// The episode ended in this period with a fall.
    pub(crate) fell: bool,
}

/// Runs one episode from `start`, restarting the robot, with `policy`
/// mapping the task's observation to an action in [-1, 1].
pub(crate) fn episode(
    robot: &mut dyn Robot,
    task: Task,
    start: [f64; 2],
    policy: impl FnMut(&[f64]) -> [f64; 12],
) -> Result<Outcome> {
    Ok(episode_with(robot, task, start, policy, |_| {})?.0)
}

/// [`episode`], calling `on_step` after every control period, and returning
/// the observation after the last one too (a learner's bootstrap state when
/// the episode ran its full length).
pub(crate) fn episode_with(
    robot: &mut dyn Robot,
    task: Task,
    start: [f64; 2],
    mut policy: impl FnMut(&[f64]) -> [f64; 12],
    mut on_step: impl FnMut(Step),
) -> Result<(Outcome, Vec<f64>)> {
    settle(robot, start)?;
    let (start, _) = robot.base_pose_z_up();
    let mut action = [0.0; 12];
    let mut change_square = 0.0;
    let mut steps = 0;
    let mut fell = false;
    // The previous period's position, sideways offset and heading, for the
    // per-period reward.
    let (mut last_x, mut last_lateral, mut last_heading) = (start.x, 0.0, 0.0);
    for step in 0..EPISODE_STEPS {
        let observation = observe(robot, task, step, &action)?;
        let next = policy(&observation);
        let mut targets = [0.0; 12];
        let mut step_change = 0.0;
        for index in 0..12 {
            let change = (next[index] - action[index]).powi(2);
            change_square += change;
            step_change += change;
            targets[index] = DEFAULT_RAD[index] + ACTION_SCALE_RAD * next[index];
        }
        action = next;
        robot.set_targets_rad(&targets)?;
        robot.step()?;
        steps += 1;
        let (position, rotation) = robot.base_pose_z_up();
        let gravity_z = (rotation.inverse() * Vec3::new(0.0, 0.0, -1.0)).z;
        let clearance = position.z - robot.ground_height_m(position.x, position.y);
        let arena = robot.arena_half_m();
        let outside = position.x.abs() > arena || position.y.abs() > arena;
        fell = clearance < FALL_HEIGHT_M || gravity_z > FALL_GRAVITY_Z || outside;
        let (lateral, heading) = ((position.y - start.y).abs(), yaw(rotation).abs());
        on_step(Step {
            reward: (position.x - last_x)
                - LATERAL_PENALTY * (lateral - last_lateral)
                - HEADING_PENALTY * (heading - last_heading)
                - SMOOTHNESS_PENALTY * step_change / EPISODE_STEPS as f64
                - if fell { FALL_PENALTY } else { 0.0 },
            fell,
        });
        (last_x, last_lateral, last_heading) = (position.x, lateral, heading);
        if fell {
            break;
        }
    }
    let last = observe(robot, task, steps, &action)?;
    let (end, rotation) = robot.base_pose_z_up();
    let (forward_m, lateral_m, heading_rad) = (end.x - start.x, end.y - start.y, yaw(rotation));
    let fitness = forward_m
        - LATERAL_PENALTY * lateral_m.abs()
        - HEADING_PENALTY * heading_rad.abs()
        - SMOOTHNESS_PENALTY * change_square / steps as f64
        - if fell { FALL_PENALTY } else { 0.0 };
    Ok((
        Outcome {
            fitness,
            forward_m,
            lateral_m,
            heading_rad,
            survived_s: steps as f64 * CONTROL_DT_S,
            fell,
            control_steps: SETTLE_STEPS + steps,
        },
        last,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heading_is_the_rotation_about_z() {
        assert!((yaw(Quat::from_rotation_z(0.3)) - 0.3).abs() < 1e-12);
        assert!(yaw(Quat::from_rotation_x(0.4)).abs() < 1e-12);
    }

    #[test]
    fn the_exported_terrain_matches_rne_at_its_probes() {
        let terrain = terrain_json(7, Relief::default()).unwrap();
        let (rows, columns) = (
            terrain["rows"].as_u64().unwrap() as usize,
            terrain["columns"].as_u64().unwrap() as usize,
        );
        let size = terrain["size_m"].as_f64().unwrap();
        let heights: Vec<f64> = terrain["heights_m"]
            .as_array()
            .unwrap()
            .iter()
            .map(|value| value.as_f64().unwrap())
            .collect();
        assert_eq!(heights.len(), rows * columns);
        // Bilinear in the exported Z-up grid, as mujoco_go2_learning.py reads it.
        let at = |x: f64, y: f64| {
            let u = (x + 0.5 * size) / (size / (columns - 1) as f64);
            let v = (y + 0.5 * size) / (size / (rows - 1) as f64);
            let (column, row) = ((u as usize).min(columns - 2), (v as usize).min(rows - 2));
            let (fu, fv) = (u - column as f64, v - row as f64);
            let h = |r: usize, c: usize| heights[r * columns + c];
            h(row, column) * (1.0 - fu) * (1.0 - fv)
                + h(row, column + 1) * fu * (1.0 - fv)
                + h(row + 1, column) * (1.0 - fu) * fv
                + h(row + 1, column + 1) * fu * fv
        };
        let probes = terrain["probes"].as_array().unwrap();
        assert_eq!(probes.len(), 40);
        for probe in probes {
            let [x, y, height] = [0, 1, 2].map(|i| probe[i].as_f64().unwrap());
            assert!((at(x, y) - height).abs() < 1e-12, "({x}, {y})");
        }
        // Zero at the origin, where the robot starts, and not flat.
        assert!(at(0.0, 0.0).abs() < 1e-12);
        let spread = heights.iter().copied().fold(f64::MIN, f64::max)
            - heights.iter().copied().fold(f64::MAX, f64::min);
        assert!(spread > 0.02, "relief {spread}");
    }

    #[test]
    fn the_height_scan_reads_the_ground_around_the_heading() {
        // Level ground: a base at the nominal height reads zero everywhere,
        // and 0.2 m higher it saturates.
        let level = height_scan(Vec3::new(1.0, 2.0, 0.3), 0.7, |_, _| 0.0);
        assert!(
            level.iter().all(|reading| reading.abs() < 1e-12),
            "{level:?}"
        );
        let high = height_scan(Vec3::new(1.0, 2.0, 0.5), 0.7, |_, _| 0.0);
        assert!(
            high.iter().all(|reading| (reading - 1.0).abs() < 1e-12),
            "{high:?}"
        );
        // Ground rising 10 cm per metre along +y, the base facing +y: the
        // rays ahead (x offsets) see it rise, the ones beside it do not.
        let heading = std::f64::consts::FRAC_PI_2;
        let scan = height_scan(Vec3::new(0.0, 0.0, 0.3), heading, |_, y| 0.1 * y);
        for (row, x) in HEIGHT_SCAN_X_M.iter().enumerate() {
            for column in 0..HEIGHT_SCAN_Y_M.len() {
                let expected = -0.1 * x * HEIGHT_SCAN_GAIN_PER_M;
                let reading = scan[row * HEIGHT_SCAN_Y_M.len() + column];
                assert!(
                    (reading - expected).abs() < 1e-12,
                    "{x}: {reading} vs {expected}"
                );
            }
        }
        // Off the terrain the ground is minus infinity: the reading saturates.
        let off = height_scan(Vec3::new(0.0, 0.0, 0.3), 0.0, |_, _| f64::NEG_INFINITY);
        assert!(off.iter().all(|reading| *reading == 1.0));
    }

    #[test]
    fn random_starts_are_reproducible_and_on_the_terrain() {
        let starts = training_starts(2026, 200, &[]);
        assert_eq!(starts.len(), 201);
        assert_eq!(starts, training_starts(2026, 200, &[]));
        assert_ne!(starts[..10], training_starts(11, 200, &[])[..10]);
        let held_out = evaluation_starts(16, &[]);
        assert!(held_out.iter().all(|start| !starts.contains(start)));
        let inside = |[x, y]: [f64; 2]| {
            (START_X_RANGE_M[0]..=START_X_RANGE_M[1]).contains(&x)
                && (START_Y_RANGE_M[0]..=START_Y_RANGE_M[1]).contains(&y)
        };
        assert!(starts.iter().chain(&held_out).all(|start| inside(*start)));
        // Room to walk 10 m forward and 4 m sideways on the 24 m terrain.
        let half = 0.5 * TERRAIN_SIZE_M;
        assert!(START_X_RANGE_M[1] + 10.0 <= half && START_Y_RANGE_M[1] + 4.0 <= half);
    }

    /// The gap terrain the workflow's `gap_depth` input builds: flat, 48 m,
    /// sampled every 5 cm, with 30 cm deep gaps.
    fn gap_relief() -> Relief {
        Relief {
            amplitude_m: 0.0,
            size_m: 48.0,
            cell_m: 0.05,
            gaps: Gaps {
                depth_m: 0.3,
                ..Gaps::default()
            },
            ..Relief::default()
        }
    }

    #[test]
    fn gaps_cross_the_terrain_and_starts_keep_clear_of_them() {
        let relief = gap_relief();
        let gaps = gap_intervals(7, relief);
        assert_eq!(gaps, gap_intervals(7, relief));
        assert!(gaps.len() > 20, "{}", gaps.len());
        let half = 0.5 * relief.size_m - ARENA_EDGE_MARGIN_M;
        for pair in gaps.windows(2) {
            // A run of 0.8-1.6 m of floor, longer only where the gaps
            // around the origin were left out.
            let run = pair[1][0] - pair[0][1];
            let around_origin = pair[0][1] < 0.0 && pair[1][0] > 0.0;
            assert!(run >= 0.8 && (run <= 1.6 || around_origin), "{run}");
        }
        for [start, end] in &gaps {
            assert!((0.25..=0.35).contains(&(end - start)));
            assert!(*start >= -half && *end <= half);
            assert!(*end < -START_GAP_CLEARANCE_M || *start > START_GAP_CLEARANCE_M);
        }
        assert!(gap_intervals(7, Relief::default()).is_empty());
        // The floor is flat between gaps and 30 cm down inside them.
        let shape = terrain(7, relief).unwrap();
        let [start, end] = gaps[gaps.len() / 2];
        assert_eq!(terrain_height_m(&shape, 0.5 * (start + end), 3.0), -0.3);
        assert_eq!(terrain_height_m(&shape, start - 0.06, -3.0), 0.0);
        assert_eq!(terrain_height_m(&shape, 0.0, 0.0), 0.0);
        // Starts skip the gaps, drawn from the same stream as without them.
        let free = training_starts(2026, 200, &[]);
        let clear = training_starts(2026, 200, &gaps);
        assert_eq!(clear.len(), 201);
        let away = |[x, _]: [f64; 2]| {
            gaps.iter().all(|[start, end]| {
                x < start - START_GAP_CLEARANCE_M || x > end + START_GAP_CLEARANCE_M
            })
        };
        assert!(clear.iter().all(|start| away(*start)));
        let kept: Vec<_> = free.iter().copied().filter(|start| away(*start)).collect();
        assert_eq!(clear[..kept.len()], kept[..]);
        assert!(evaluation_starts(16, &gaps)
            .iter()
            .all(|start| away(*start)));
    }

    #[test]
    fn the_height_scan_sees_a_gap_ahead() {
        let relief = gap_relief();
        let shape = terrain(7, relief).unwrap();
        let gaps = gap_intervals(7, relief);
        let [start, end] = gaps[gaps.len() / 2];
        let ground = |x: f64, y: f64| terrain_height_m(&shape, x, y);
        // Facing +x with the gap's middle 0.4 m ahead: the rays 0.4 m ahead
        // read the gap's floor, clipped, and those over the floor do not.
        let middle = 0.5 * (start + end);
        let base = Vec3::new(middle - 0.4, 0.0, 0.28);
        let readings = height_scan(base, 0.0, ground);
        for (index, reading) in readings.iter().enumerate() {
            let x = HEIGHT_SCAN_X_M[index / HEIGHT_SCAN_Y_M.len()];
            if (x - 0.4).abs() < 1e-12 {
                assert_eq!(*reading, 1.0, "{readings:?}");
            } else if x <= 0.0 {
                assert!(reading.abs() < 0.11, "{readings:?}");
            }
        }
    }

    #[test]
    fn a_height_scan_policy_stands_from_a_random_start() {
        let plant = Plant {
            backend: Backend::Locomotion,
            terrain_seed: Some(7),
            ..Plant::default()
        };
        let task = Task {
            height_scan: true,
            random_starts: true,
        };
        let mut go2 = robot(plant).unwrap();
        let start = training_starts(2026, 0, &[])[0];
        let mut widths = Vec::new();
        let outcome = episode(go2.as_mut(), task, start, |observation| {
            widths.push(observation.len());
            [0.0; 12]
        })
        .unwrap();
        assert!(!outcome.fell && outcome.survived_s == 5.0, "{outcome:?}");
        assert!(widths.iter().all(|width| *width == 59));
        // The base settled above the start, at its spawn height over the ground.
        let (position, _) = go2.base_pose_z_up();
        assert!((position.x - start[0]).abs() < 0.3 && (position.y - start[1]).abs() < 0.3);
        let clearance = position.z - go2.ground_height_m(position.x, position.y);
        assert!(clearance > 0.2 && clearance < 0.35, "{clearance}");
    }

    #[test]
    fn walking_near_the_terrain_edge_ends_the_episode_as_a_fall() {
        let plant = Plant {
            backend: Backend::Locomotion,
            terrain_seed: Some(7),
            ..Plant::default()
        };
        let mut go2 = robot(plant).unwrap();
        let arena = go2.arena_half_m();
        assert_eq!(arena, 0.5 * TERRAIN_SIZE_M - ARENA_EDGE_MARGIN_M);
        let still = |_: &[f64]| [0.0; 12];
        // Standing just inside the arena is fine; just outside it, the first
        // control period ends the episode before a foot could leave the field.
        let inside = episode(go2.as_mut(), Task::default(), [arena - 0.2, 0.0], still).unwrap();
        assert!(!inside.fell, "{inside:?}");
        let outside = episode(go2.as_mut(), Task::default(), [0.0, -arena - 0.2], still).unwrap();
        assert!(
            outside.fell && outside.survived_s == CONTROL_DT_S,
            "{outside:?}"
        );
    }

    #[test]
    fn a_still_policy_stands_on_the_terrain_and_rapier_refuses_it() {
        let plant = Plant {
            backend: Backend::Locomotion,
            terrain_seed: Some(7),
            ..Plant::default()
        };
        let mut go2 = robot(plant).unwrap();
        let outcome = episode(go2.as_mut(), Task::default(), [0.0, 0.0], |_| [0.0; 12]).unwrap();
        assert!(!outcome.fell, "{outcome:?}");
        assert!(robot(Plant {
            backend: Backend::Rapier,
            ..plant
        })
        .is_err());
    }

    #[test]
    fn a_still_policy_stands_through_an_episode_on_every_backend() {
        for backend in [Backend::Rapier, Backend::Native, Backend::Locomotion] {
            let mut robot = robot(Plant {
                backend,
                ..Plant::default()
            })
            .unwrap();
            let still = |_: &[f64]| [0.0; 12];
            let outcome = episode(robot.as_mut(), Task::default(), [0.0, 0.0], still).unwrap();
            assert!(!outcome.fell, "{backend:?}: {outcome:?}");
            assert!(outcome.forward_m.abs() < 0.05, "{backend:?}: {outcome:?}");
            assert_eq!(outcome.control_steps, SETTLE_STEPS + EPISODE_STEPS);
            // A second episode on the same robot replays the first.
            assert_eq!(
                episode(robot.as_mut(), Task::default(), [0.0, 0.0], still).unwrap(),
                outcome
            );
            // Only the terrain backend has anywhere else to start.
            if backend != Backend::Locomotion {
                assert!(episode(robot.as_mut(), Task::default(), [1.0, 0.0], still).is_err());
            }
        }
    }
}
