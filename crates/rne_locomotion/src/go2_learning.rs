//! The Go2 walking task of example 147, as a library: the plant, the
//! terrains, the observation, the reward and the episode.
//!
//! [`Go2LearningRobot`] is the URDF's articulated Go2 with its fixed links
//! fused and the declared masses (16.087 kg) on [`QuadrupedTerrainSim`]:
//! `rne_dynamics`' hard-contact step, with every URDF collision shape
//! touching the ground ([`collision_spheres`]). Its joints track position
//! targets through an explicit PD torque, clamped to the effort limits, at
//! [`DT_S`] physics steps and [`DECIMATION`] steps a control period.
//!
//! The floor is flat, or a terrain from [`terrain`]: a fractal height field
//! with optional gaps across it ([`Gaps`]). [`Go2LearningEpisode`] walks the
//! robot for [`EPISODE_STEPS`] control periods from the origin or from a
//! random start, observing [`observation`] and earning [`period_reward`]: as
//! an [`rne_ai::Episode`], it runs in [`rne_ai::VectorizedEpisode`] and in
//! `rne_py`.
//!
//! Positions and orientations are in the Z-up locomotion frame (see the
//! crate docs); the terrain itself is an ordinary Y-up height field.

use std::sync::Arc;

use rne_ai::{
    ActionSpec, Episode, EpisodeStep, ObservationSpec, ResetSpec, RewardSpec, RewardTermSpec,
    TaskSpec, TensorBounds, TensorDType, TensorSpec, TerminationConditionSpec, TerminationKind,
    TerminationSpec,
};
use rne_math::{Quat, Vec3};
use rne_physics::{ColliderShape, FractalTerrain};
use rne_urdf_import::{parse_urdf, rpy_to_quat, UrdfGeometry};

use crate::quadruped::GO2_URDF;
use crate::{
    BodySphere, JointCommand, LocomotionError, Quadruped, QuadrupedTerrainSim, TerrainSimConfig,
};

/// Actuated joints' child links, in action order (unitree_rl_gym's).
pub const LINKS: [&str; 12] = [
    "FL_hip", "FL_thigh", "FL_calf", "FR_hip", "FR_thigh", "FR_calf", "RL_hip", "RL_thigh",
    "RL_calf", "RR_hip", "RR_thigh", "RR_calf",
];
/// unitree_rl_gym's Go2 default joint angles, in [`LINKS`] order.
pub const DEFAULT_RAD: [f64; 12] = [
    0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0, -1.5,
];
/// unitree_rl_gym's Go2 proportional gain, in N·m per radian.
pub const KP_NM_PER_RAD: f64 = 20.0;
/// unitree_rl_gym's Go2 derivative gain, in N·m·s per radian.
pub const KD_NM_S_PER_RAD: f64 = 0.5;
/// Physics step, in seconds.
pub const DT_S: f64 = 0.005;
/// Physics steps a control period (50 Hz control).
pub const DECIMATION: usize = 4;
/// Control period, in seconds.
pub const CONTROL_DT_S: f64 = DT_S * DECIMATION as f64;
/// Control periods the robot stands on its default pose before an episode,
/// after it starts in that pose, at rest, with its base
/// [`SPAWN_HEIGHT_M`] above the ground.
pub const SETTLE_STEPS: usize = 50;
/// Height of the base above the ground at a restart, in meters.
pub const SPAWN_HEIGHT_M: f64 = 0.36;
/// Control periods in an episode (5 s).
pub const EPISODE_STEPS: usize = 250;
/// Period of the gait clock the policy observes, in seconds.
pub const GAIT_PERIOD_S: f64 = 0.5;
/// Largest target offset from the default pose, in radians: an action of
/// ±1 moves a target this far.
pub const ACTION_SCALE_RAD: f64 = 0.5;
/// The robot has fallen when its base is this close to the ground, in
/// meters...
pub const FALL_HEIGHT_M: f64 = 0.10;
/// ...or when the base's downward axis component of gravity, in the base
/// frame, exceeds this (a tilt over 60°).
pub const FALL_GRAVITY_Z: f64 = -0.5;
/// Proprioceptive observation width: angular velocity, gravity, joint
/// positions and velocities, previous action, gait clock.
pub const PROPRIOCEPTION: usize = 3 + 3 + 12 + 12 + 12 + 2;
/// The height scan's vertical rays: at these offsets from the base,
/// forward, in meters, in its heading frame (roll and pitch ignored)...
pub const HEIGHT_SCAN_X_M: [f64; 5] = [-0.2, 0.0, 0.2, 0.4, 0.6];
/// ...and to the left, x outer and y inner.
pub const HEIGHT_SCAN_Y_M: [f64; 3] = [-0.15, 0.0, 0.15];
/// Readings in the height scan.
pub const HEIGHT_SCAN_POINTS: usize = HEIGHT_SCAN_X_M.len() * HEIGHT_SCAN_Y_M.len();
/// Each reading is the base's height above the ray's hit, less this nominal
/// stance height, in meters...
pub const HEIGHT_SCAN_NOMINAL_M: f64 = 0.3;
/// ...times this gain, clipped to [-1, 1]: 0 under a standing robot on
/// level ground.
pub const HEIGHT_SCAN_GAIN_PER_M: f64 = 5.0;
/// Random starts: points drawn uniformly from this x range, in meters...
pub const START_X_RANGE_M: [f64; 2] = [-8.0, 2.0];
/// ...and this y range, leaving room to walk 10 m forward and 4 m sideways
/// before the default terrain's 12 m edge.
pub const START_Y_RANGE_M: [f64; 2] = [-8.0, 8.0];
/// Mixed into a seed for the training starts' stream.
pub const START_STREAM: u64 = 0x5354_4152_5453_0147;
/// Seed of the held-out evaluation starts.
pub const EVALUATION_STARTS_SEED: u64 = 0x4556_414C_0147;
/// Fitness and reward weight per meter sideways.
pub const LATERAL_PENALTY: f64 = 0.5;
/// Fitness and reward weight per radian of heading.
pub const HEADING_PENALTY: f64 = 0.5;
/// Fitness and reward cost of a fall.
pub const FALL_PENALTY: f64 = 1.0;
/// Fitness weight on the mean squared action change per control period.
pub const SMOOTHNESS_PENALTY: f64 = 0.02;
/// The default terrain's side, in meters.
pub const TERRAIN_SIZE_M: f64 = 24.0;
/// The default terrain's samples along each side: one every 10 cm.
pub const TERRAIN_SAMPLES: u32 = 241;
/// The default terrain's first-octave frequency, in cycles per meter.
pub const TERRAIN_FREQUENCY_PER_M: f64 = 0.5;
/// The default terrain's first-octave amplitude, in meters.
pub const TERRAIN_AMPLITUDE_M: f64 = 0.04;
/// Mixed into the terrain seed for the gaps' stream.
pub const GAP_STREAM: u64 = 0x4741_5053_0147;
/// How far from a gap's edge a start (and the origin) stays, in meters:
/// farther than a foot reaches from the base, plus a cell of the height
/// field.
pub const START_GAP_CLEARANCE_M: f64 = 0.45;
/// How far inside the floor's edge the arena ends, in meters: farther than a
/// foot reaches from the base, or the base moves in a control period.
pub const ARENA_EDGE_MARGIN_M: f64 = 1.0;
/// Side of the square flat floor, in meters.
pub const FLOOR_SIZE_M: f64 = 40.0;

/// Bisection steps of the sliding-friction solve: 30 resolve the friction
/// direction to 2e-10 rad, where the default 60 run past double precision.
const SLIDING_BISECTION_ITERATIONS: usize = 30;
/// Contact solver stopping rule: a sweep that changes no impulse by more than
/// 1e-5 N·s (a stance foot carries about 0.8 N·s per step) and no contact
/// velocity by more than 0.1 mm/s, or 30 sweeps. The library's defaults
/// (1e-10 N·s, 1e-8 m/s, 200 sweeps) let a kneeling gait, whose foot, knee
/// and shin contacts on one calf are nearly redundant, run hundreds of sweeps
/// a step; an unconverged sweep's impulses still lie in the friction cone.
const SOLVER_TOLERANCE_N_S: f64 = 1.0e-5;
const SOLVER_VELOCITY_TOLERANCE_M_S: f64 = 1.0e-4;
const SOLVER_MAX_SWEEPS: usize = 30;

/// SplitMix64 uniform deviates: the stream the starts and the gaps draw.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SplitMix64(pub u64);

impl SplitMix64 {
    /// A uniform deviate in (0, 1).
    pub fn uniform(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        ((z >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    /// A standard normal deviate (Box-Muller, from two uniforms).
    pub fn normal(&mut self) -> f64 {
        let (u, v) = (self.uniform(), self.uniform());
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

/// Trenches across the terrain: each runs the terrain's whole width in y,
/// at a random width, after a random run of floor along x. A depth of zero,
/// the default, leaves the terrain without gaps.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gaps {
    /// Depth below the floor, in meters.
    pub depth_m: f64,
    /// Range of widths along x, in meters.
    pub width_m: [f64; 2],
    /// Range of the runs of floor between gaps, in meters.
    pub spacing_m: [f64; 2],
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

/// A terrain's fractal relief, size, sample spacing and gaps.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Relief {
    /// First-octave frequency, in cycles per meter.
    pub frequency_per_m: f64,
    /// First-octave amplitude, in meters.
    pub amplitude_m: f64,
    /// Side of the square terrain, in meters.
    pub size_m: f64,
    /// Sample spacing, in meters.
    pub cell_m: f64,
    /// Gaps across the terrain.
    pub gaps: Gaps,
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
    pub fn samples(self) -> u32 {
        (self.size_m / self.cell_m).round() as u32 + 1
    }
}

/// The gaps of the terrain of `seed`, as `[start, end]` along x in meters,
/// west to east: none within [`START_GAP_CLEARANCE_M`] of the origin or
/// within [`ARENA_EDGE_MARGIN_M`] of the terrain's edge.
pub fn gap_intervals(seed: u64, relief: Relief) -> Vec<[f64; 2]> {
    let gaps = relief.gaps;
    if gaps.depth_m <= 0.0 {
        return Vec::new();
    }
    let mut stream = SplitMix64(seed ^ GAP_STREAM);
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

/// The terrain for `seed`: a Y-up height field, zero at the origin, with
/// `relief`'s gaps cut into it.
pub fn terrain(seed: u64, relief: Relief) -> Result<ColliderShape, LocomotionError> {
    let spec = FractalTerrain {
        size_x_m: relief.size_m,
        size_z_m: relief.size_m,
        columns: relief.samples(),
        rows: relief.samples(),
        frequency_per_m: relief.frequency_per_m,
        amplitude_m: relief.amplitude_m,
        ..FractalTerrain::default()
    };
    let raw = spec.height_field(seed)?;
    let origin = rne_physics::height_field_surface(&raw, 0.0, 0.0)
        .ok_or(LocomotionError::InvalidInput(
            "the terrain does not cover the origin",
        ))?
        .height_m;
    let shifted = FractalTerrain {
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
    Ok(ColliderShape::HeightField {
        nrows: spec.rows,
        ncols: spec.columns,
        heights_m: Arc::from(heights),
        scale: Vec3::new(spec.size_x_m, 1.0, spec.size_z_m),
    })
}

/// Height of the Y-up `terrain` below Z-up `(x, y)`, in meters; negative
/// infinity off the terrain.
pub fn terrain_height_m(terrain: &ColliderShape, x_m: f64, y_m: f64) -> f64 {
    // Z-up (x, y) is Y-up (x, -z).
    rne_physics::height_field_surface(terrain, x_m, -y_m).map_or(f64::NEG_INFINITY, |s| s.height_m)
}

/// Heading about +z of a Z-up orientation, in radians.
pub fn yaw(rotation: Quat) -> f64 {
    let Quat { x, y, z, w } = rotation;
    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}

/// The height scan's readings around a base at `position` facing
/// `heading_rad`, where `ground` gives the terrain height below a point.
pub fn height_scan(
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

/// `count` start points uniform in the start box, from SplitMix64 `seed`,
/// skipping any within [`START_GAP_CLEARANCE_M`] of a gap: without gaps,
/// the stream's first `2 count` uniforms.
pub fn start_points(seed: u64, count: usize, gaps: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let mut stream = SplitMix64(seed);
    (0..count).map(|_| next_start(&mut stream, gaps)).collect()
}

/// The next start point in `stream`, clear of `gaps`.
fn next_start(stream: &mut SplitMix64, gaps: &[[f64; 2]]) -> [f64; 2] {
    let lerp = |[low, high]: [f64; 2], t: f64| low + (high - low) * t;
    loop {
        let (u, v) = (stream.uniform(), stream.uniform());
        let point = [lerp(START_X_RANGE_M, u), lerp(START_Y_RANGE_M, v)];
        if gaps.iter().all(|[start, end]| {
            point[0] < start - START_GAP_CLEARANCE_M || point[0] > end + START_GAP_CLEARANCE_M
        }) {
            return point;
        }
    }
}

/// A learner's starts for seed `seed`, `count + 1` of them (one per ES
/// generation, generation 0 first), clear of `gaps`.
pub fn training_starts(seed: u64, count: usize, gaps: &[[f64; 2]]) -> Vec<[f64; 2]> {
    start_points(seed ^ START_STREAM, count + 1, gaps)
}

/// The held-out starts every policy is evaluated from, clear of `gaps`.
pub fn evaluation_starts(count: usize, gaps: &[[f64; 2]]) -> Vec<[f64; 2]> {
    start_points(EVALUATION_STARTS_SEED, count, gaps)
}

/// Contacts for every collision shape in the Go2 URDF besides the feet,
/// which are the quadruped's foot spheres already:
/// - a sphere as itself (the head's lower half);
/// - a cylinder as spheres of its radius along its axis, one radius apart and
///   one radius in from its ends, or one sphere at its centre when it is
///   shorter than its diameter (the hips and the head's upper half);
/// - a box as points at its eight corners (the body and the thighs): on a
///   plane, a box's lowest point is a corner.
pub fn collision_spheres(quadruped: &Quadruped) -> Result<Vec<BodySphere>, LocomotionError> {
    let urdf = parse_urdf(GO2_URDF).map_err(|error| LocomotionError::Urdf(error.to_string()))?;
    let links = quadruped.model().kinematic();
    let feet: Vec<_> = quadruped.legs().iter().map(|leg| leg.foot_link).collect();
    let mut spheres = Vec::new();
    for link in urdf.links.iter().filter(|link| !link.collisions.is_empty()) {
        let entity = links
            .link_entity_by_name(&link.name)
            .ok_or_else(|| LocomotionError::MissingLink(link.name.clone()))?;
        if feet.contains(&entity) {
            continue;
        }
        for collision in &link.collisions {
            let rotation = rpy_to_quat(collision.origin_rpy);
            let mut add = |center: Vec3, radius_m: f64| {
                spheres.push(BodySphere {
                    link: entity,
                    center_local_m: collision.origin_xyz + rotation * center,
                    radius_m,
                })
            };
            match collision.geometry {
                UrdfGeometry::Sphere { radius_m } => add(Vec3::ZERO, radius_m),
                UrdfGeometry::Cylinder { radius_m, length_m } => {
                    let reach = (0.5 * length_m - radius_m).max(0.0);
                    let gaps = (2.0 * reach / radius_m).ceil() as usize;
                    for index in 0..=gaps {
                        let along = if gaps == 0 {
                            0.0
                        } else {
                            -reach + 2.0 * reach * index as f64 / gaps as f64
                        };
                        add(Vec3::new(0.0, 0.0, along), radius_m);
                    }
                }
                UrdfGeometry::Box { size_m } => {
                    let half = size_m * 0.5;
                    for corner in 0..8 {
                        let sign = |bit: usize| if corner & bit == 0 { -1.0 } else { 1.0 };
                        add(
                            Vec3::new(sign(1) * half.x, sign(2) * half.y, sign(4) * half.z),
                            0.0,
                        );
                    }
                }
                UrdfGeometry::Mesh { .. } => {
                    return Err(LocomotionError::InvalidInput(
                        "a Go2 collision shape is a mesh",
                    ))
                }
            }
        }
    }
    Ok(spheres)
}

/// Where a Go2 learns to walk: a flat floor, or the terrain of a seed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Floor {
    /// A flat floor [`FLOOR_SIZE_M`] square.
    Flat,
    /// The terrain of `seed` with `relief` ([`terrain`]).
    Terrain {
        /// The terrain's seed.
        seed: u64,
        /// Its relief.
        relief: Relief,
    },
}

/// The Go2 on [`QuadrupedTerrainSim`], restarted in its default pose by
/// [`Self::restart_at`].
#[derive(Clone, Debug)]
pub struct Go2LearningRobot {
    sim: QuadrupedTerrainSim,
    /// Actuated coordinate of each joint in [`LINKS`] order.
    actuated: [usize; 12],
    /// PD targets, by actuated coordinate.
    targets_rad: [f64; 12],
    command: JointCommand,
    arena_half_m: f64,
    /// By kinematic link: the ground's normal impulse through it and the
    /// physics steps it carried any.
    contact_load: Vec<(f64, usize)>,
}

impl Go2LearningRobot {
    /// A Go2 on `floor` with Coulomb friction `floor_friction`, standing in
    /// its default pose above the origin.
    pub fn new(floor_friction: f64, floor: Floor) -> Result<Self, LocomotionError> {
        let quadruped = Arc::new(Quadruped::go2()?);
        // LINKS runs leg by leg (FL, FR, RL, RR), hip, thigh and calf, as the
        // Go2 spec lists its legs.
        let legs = quadruped.legs();
        let actuated = std::array::from_fn(|index| legs[index / 3].joints[index % 3]);
        let (side_m, ground) = match floor {
            Floor::Terrain { seed, relief } => (relief.size_m, terrain(seed, relief)?),
            Floor::Flat => (
                FLOOR_SIZE_M,
                FractalTerrain {
                    size_x_m: FLOOR_SIZE_M,
                    size_z_m: FLOOR_SIZE_M,
                    columns: 5,
                    rows: 5,
                    amplitude_m: 0.0,
                    ..FractalTerrain::default()
                }
                .height_field(0)?,
            ),
        };
        let mut config = TerrainSimConfig {
            step_time_s: DT_S,
            // The PD is explicit, in the commanded torque.
            position_gain_nm_rad: 0.0,
            velocity_gain_nm_s_rad: 0.0,
            friction_coefficient: floor_friction,
            ..TerrainSimConfig::default()
        };
        config.contact.solver.bisection_iterations = SLIDING_BISECTION_ITERATIONS;
        config.contact.solver.tolerance_n_s = SOLVER_TOLERANCE_N_S;
        config.contact.solver.velocity_tolerance_m_s = SOLVER_VELOCITY_TOLERANCE_M_S;
        config.contact.solver.max_iterations = SOLVER_MAX_SWEEPS;
        let spheres = collision_spheres(&quadruped)?;
        let mut sim = QuadrupedTerrainSim::new(quadruped, ground, config)?;
        sim.set_body_spheres(spheres)?;
        let kinematic = sim.quadruped().model().kinematic();
        let links = (0..)
            .take_while(|&index| kinematic.link_entity(index).is_some())
            .count();
        let mut robot = Self {
            sim,
            actuated,
            targets_rad: [0.0; 12],
            command: JointCommand::positions(vec![0.0; 12]),
            arena_half_m: 0.5 * side_m - ARENA_EDGE_MARGIN_M,
            contact_load: vec![(0.0, 0); links],
        };
        robot.restart_at([0.0, 0.0])?;
        Ok(robot)
    }

    /// The robot's total mass, in kilograms.
    pub fn mass_kg(&self) -> f64 {
        self.sim.quadruped().mass_kg()
    }

    /// Back to the default pose and targets, at rest, with the base
    /// [`SPAWN_HEIGHT_M`] above the ground at `[x_m, y_m]`.
    pub fn restart_at(&mut self, [x_m, y_m]: [f64; 2]) -> Result<(), LocomotionError> {
        let nv = self.sim.quadruped().model().nv();
        let mut q = vec![0.0; nv];
        q[0] = x_m;
        q[1] = y_m;
        // The terrain is shifted to height zero at the origin, so a start there
        // keeps the flat floor's spawn height exactly.
        let ground = if [x_m, y_m] == [0.0, 0.0] {
            0.0
        } else {
            self.ground_height_m(x_m, y_m)
        };
        if !ground.is_finite() {
            return Err(LocomotionError::OffTerrain { x_m, y_m });
        }
        q[2] = SPAWN_HEIGHT_M + ground;
        for (index, angle) in DEFAULT_RAD.iter().enumerate() {
            q[6 + self.actuated[index]] = *angle;
        }
        self.sim.set_state(&q, &vec![0.0; nv])?;
        self.set_targets_rad(&DEFAULT_RAD);
        Ok(())
    }

    /// Sets the joints' position targets, in [`LINKS`] order.
    pub fn set_targets_rad(&mut self, targets_rad: &[f64; 12]) {
        for (index, target) in targets_rad.iter().enumerate() {
            self.targets_rad[self.actuated[index]] = *target;
        }
    }

    /// One control period: [`DECIMATION`] physics steps, each with the PD
    /// torque computed at its start and clamped to the effort limits.
    pub fn step(&mut self) -> Result<(), LocomotionError> {
        for _ in 0..DECIMATION {
            let positions = self.sim.joint_positions_rad();
            let velocities = self.sim.joint_velocities_rad_s();
            let limits = self.sim.quadruped().effort_limits();
            let torques: Vec<f64> = (0..12)
                .map(|joint| {
                    (KP_NM_PER_RAD * (self.targets_rad[joint] - positions[joint])
                        - KD_NM_S_PER_RAD * velocities[joint])
                        .clamp(-limits[joint], limits[joint])
                })
                .collect();
            // With zero PD gains the position targets only fill the command.
            self.command.positions_rad.copy_from_slice(positions);
            self.command.feedforward_nm = torques;
            let step = self.sim.step(&self.command, Vec3::ZERO)?;
            let kinematic = self.sim.quadruped().model().kinematic();
            let mut loaded = vec![false; self.contact_load.len()];
            for (contact, outcome) in step.contacts.iter().zip(&step.outcomes) {
                let normal = outcome.impulse_world_n_s.dot(contact.normal_world);
                if normal <= 0.0 {
                    continue;
                }
                if let Some(index) = kinematic.link_index(contact.link) {
                    self.contact_load[index].0 += normal;
                    loaded[index] = true;
                }
            }
            for (load, loaded) in self.contact_load.iter_mut().zip(loaded) {
                load.1 += usize::from(loaded);
            }
        }
        Ok(())
    }

    /// The base's position and orientation, Z-up.
    pub fn base_pose_z_up(&self) -> (Vec3, Quat) {
        let state = self.sim.base_state();
        (state.position_m, state.rotation)
    }

    /// The base's linear and angular velocity in the world frame, Z-up.
    pub fn base_velocity_z_up(&self) -> (Vec3, Vec3) {
        let state = self.sim.base_state();
        (state.linear_velocity_m_s, state.angular_velocity_rad_s)
    }

    /// Height of the ground below `(x_m, y_m)`, in meters; negative infinity
    /// off the floor.
    pub fn ground_height_m(&self, x_m: f64, y_m: f64) -> f64 {
        terrain_height_m(self.sim.terrain(), x_m, y_m)
    }

    /// Half the side of the square around the origin the base must stay
    /// inside, in meters: a robot that walks out of it would put a foot off
    /// the floor's edge.
    pub fn arena_half_m(&self) -> f64 {
        self.arena_half_m
    }

    /// Joint positions (rad) and velocities (rad/s), in [`LINKS`] order.
    pub fn joint_states(&self) -> [(f64, f64); 12] {
        let positions = self.sim.joint_positions_rad();
        let velocities = self.sim.joint_velocities_rad_s();
        std::array::from_fn(|index| {
            let joint = self.actuated[index];
            (positions[joint], velocities[joint])
        })
    }

    /// By link name: the ground's normal impulse through it (N·s) and the
    /// physics steps it carried any, summed since the robot was built.
    pub fn contact_load(&self) -> Vec<(String, f64, usize)> {
        let kinematic = self.sim.quadruped().model().kinematic();
        self.contact_load
            .iter()
            .enumerate()
            .map(|(index, (impulse, steps))| {
                let name = kinematic.link_name(index).unwrap_or("?").to_string();
                (name, *impulse, *steps)
            })
            .collect()
    }

    /// The underlying simulation.
    pub fn sim(&self) -> &QuadrupedTerrainSim {
        &self.sim
    }
}

/// The policy's observation: [`PROPRIOCEPTION`] values, then
/// [`HEIGHT_SCAN_POINTS`] when `scan` is given.
///
/// - the base's angular velocity in its own frame, times 0.25;
/// - gravity's direction in the base frame;
/// - the joints' offsets from [`DEFAULT_RAD`], then their velocities times
///   0.05, in [`LINKS`] order;
/// - the previous action;
/// - the sine and cosine of the [`GAIT_PERIOD_S`] gait clock at control
///   period `step`.
pub fn observation(
    rotation: Quat,
    spin_world_rad_s: Vec3,
    joints: &[(f64, f64); 12],
    step: usize,
    action: &[f64; 12],
    scan: Option<&[f64; HEIGHT_SCAN_POINTS]>,
) -> Vec<f64> {
    let inverse = rotation.inverse();
    let spin = inverse * spin_world_rad_s;
    let gravity = inverse * Vec3::new(0.0, 0.0, -1.0);
    let width = PROPRIOCEPTION + scan.map_or(0, |_| HEIGHT_SCAN_POINTS);
    let mut observation = vec![0.0; width];
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
    if let Some(readings) = scan {
        observation[PROPRIOCEPTION..].copy_from_slice(readings);
    }
    observation
}

/// Where the base was at the end of the previous control period, for
/// [`period_reward`]: its x, its sideways offset from the start and its
/// absolute heading.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RewardState {
    /// The base's x, in meters.
    pub x_m: f64,
    /// Its distance sideways from the start, in meters.
    pub lateral_m: f64,
    /// Its absolute heading, in radians.
    pub heading_rad: f64,
}

/// One control period's reward, and the state the next period's reward
/// starts from: forward progress, less the growth of the sideways offset and
/// of the heading, less the smoothness penalty on `step_change` (the squared
/// change of the action), less [`FALL_PENALTY`] if the robot fell. An
/// episode's rewards sum to its fitness, except that the smoothness penalty
/// divides by [`EPISODE_STEPS`] where the fitness divides by the periods
/// walked.
pub fn period_reward(
    last: RewardState,
    position: Vec3,
    rotation: Quat,
    start_y_m: f64,
    step_change: f64,
    fell: bool,
) -> (f64, RewardState) {
    let now = RewardState {
        x_m: position.x,
        lateral_m: (position.y - start_y_m).abs(),
        heading_rad: yaw(rotation).abs(),
    };
    let reward = (now.x_m - last.x_m)
        - LATERAL_PENALTY * (now.lateral_m - last.lateral_m)
        - HEADING_PENALTY * (now.heading_rad - last.heading_rad)
        - SMOOTHNESS_PENALTY * step_change / EPISODE_STEPS as f64
        - if fell { FALL_PENALTY } else { 0.0 };
    (reward, now)
}

/// Whether a robot whose base is at `position`, `rotation`, has fallen:
/// its base within [`FALL_HEIGHT_M`] of the ground below it, tilted past
/// 60°, or outside the arena.
pub fn has_fallen(position: Vec3, rotation: Quat, ground_m: f64, arena_half_m: f64) -> bool {
    let gravity_z = (rotation.inverse() * Vec3::new(0.0, 0.0, -1.0)).z;
    let clearance = position.z - ground_m;
    let outside = position.x.abs() > arena_half_m || position.y.abs() > arena_half_m;
    clearance < FALL_HEIGHT_M || gravity_z > FALL_GRAVITY_Z || outside
}

/// A [`Go2LearningEpisode`]'s floor and task.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Go2LearningConfig {
    /// The floor's Coulomb friction coefficient.
    pub floor_friction: f64,
    /// The floor.
    pub floor: Floor,
    /// Append the height scan to the observation.
    pub height_scan: bool,
    /// Start each episode at the next point of the start stream (see
    /// [`training_starts`]) instead of the origin; needs a terrain.
    pub random_starts: bool,
    /// Seed of the start stream.
    pub seed: u64,
}

impl Default for Go2LearningConfig {
    fn default() -> Self {
        Self {
            floor_friction: 1.0,
            floor: Floor::Flat,
            height_scan: false,
            random_starts: false,
            seed: 0,
        }
    }
}

impl Go2LearningConfig {
    /// The observation's width.
    pub fn observation_dim(&self) -> usize {
        PROPRIOCEPTION
            + if self.height_scan {
                HEIGHT_SCAN_POINTS
            } else {
                0
            }
    }

    /// The gaps of the configured floor; none on a flat floor.
    pub fn gaps(&self) -> Vec<[f64; 2]> {
        match self.floor {
            Floor::Flat => Vec::new(),
            Floor::Terrain { seed, relief } => gap_intervals(seed, relief),
        }
    }
}

/// Example 147's walking task as an [`Episode`]: each [`Episode::reset`]
/// restarts the robot (at the origin, or at the next random start), stands
/// it for [`SETTLE_STEPS`] control periods, and returns the first
/// observation; each [`Episode::step`] takes an action in [-1, 1]
/// (clamped), offsets the targets [`ACTION_SCALE_RAD`] per unit from
/// [`DEFAULT_RAD`], and runs one control period.
///
/// The step's reward is [`period_reward`]. A fall ends the episode as
/// terminated; [`EPISODE_STEPS`] periods end it as truncated. A simulation
/// error ends it as terminated with [`FALL_PENALTY`], and
/// [`Self::last_error`] holds it; [`Self::try_reset`] and
/// [`Self::try_step`] return it instead.
#[derive(Clone, Debug)]
pub struct Go2LearningEpisode {
    config: Go2LearningConfig,
    robot: Go2LearningRobot,
    gaps: Vec<[f64; 2]>,
    starts: SplitMix64,
    action: [f64; 12],
    step: usize,
    start_y_m: f64,
    reward_state: RewardState,
    episode_index: u32,
    last_error: Option<String>,
}

impl Go2LearningEpisode {
    /// An episode with `config`; call [`Episode::reset`] before stepping.
    pub fn new(config: Go2LearningConfig) -> Result<Self, LocomotionError> {
        if config.random_starts && config.floor == Floor::Flat {
            return Err(LocomotionError::InvalidInput(
                "random starts need a terrain",
            ));
        }
        Ok(Self {
            robot: Go2LearningRobot::new(config.floor_friction, config.floor)?,
            gaps: config.gaps(),
            starts: SplitMix64(config.seed ^ START_STREAM),
            config,
            action: [0.0; 12],
            step: 0,
            start_y_m: 0.0,
            reward_state: RewardState::default(),
            episode_index: 0,
            last_error: None,
        })
    }

    /// The configuration.
    pub fn config(&self) -> &Go2LearningConfig {
        &self.config
    }

    /// The robot.
    pub fn robot(&self) -> &Go2LearningRobot {
        &self.robot
    }

    /// The message of the simulation error that ended the last episode, if
    /// one did.
    pub fn last_error(&self) -> Option<&str> {
        self.last_error.as_deref()
    }

    /// The observation now.
    pub fn observe(&self) -> Vec<f64> {
        let (position, rotation) = self.robot.base_pose_z_up();
        let (_, spin) = self.robot.base_velocity_z_up();
        let scan = self.config.height_scan.then(|| {
            height_scan(position, yaw(rotation), |x, y| {
                self.robot.ground_height_m(x, y)
            })
        });
        observation(
            rotation,
            spin,
            &self.robot.joint_states(),
            self.step,
            &self.action,
            scan.as_ref(),
        )
    }

    /// [`Episode::reset`], returning a simulation error instead of ending
    /// the episode on it.
    pub fn try_reset(&mut self) -> Result<Vec<f64>, LocomotionError> {
        let start = if self.config.random_starts {
            next_start(&mut self.starts, &self.gaps)
        } else {
            [0.0, 0.0]
        };
        self.robot.restart_at(start)?;
        self.robot.set_targets_rad(&DEFAULT_RAD);
        for _ in 0..SETTLE_STEPS {
            self.robot.step()?;
        }
        let (position, _) = self.robot.base_pose_z_up();
        self.action = [0.0; 12];
        self.step = 0;
        self.start_y_m = position.y;
        self.reward_state = RewardState {
            x_m: position.x,
            ..RewardState::default()
        };
        self.last_error = None;
        Ok(self.observe())
    }

    /// [`Episode::step`], returning a simulation error instead of ending the
    /// episode on it.
    pub fn try_step(
        &mut self,
        action: [f64; 12],
    ) -> Result<EpisodeStep<Vec<f64>>, LocomotionError> {
        let next = action.map(|value| value.clamp(-1.0, 1.0));
        let mut targets = [0.0; 12];
        let mut step_change = 0.0;
        for index in 0..12 {
            step_change += (next[index] - self.action[index]).powi(2);
            targets[index] = DEFAULT_RAD[index] + ACTION_SCALE_RAD * next[index];
        }
        self.action = next;
        self.robot.set_targets_rad(&targets);
        self.robot.step()?;
        self.step += 1;
        let (position, rotation) = self.robot.base_pose_z_up();
        let ground = self.robot.ground_height_m(position.x, position.y);
        let fell = has_fallen(position, rotation, ground, self.robot.arena_half_m());
        let (reward, now) = period_reward(
            self.reward_state,
            position,
            rotation,
            self.start_y_m,
            step_change,
            fell,
        );
        self.reward_state = now;
        Ok(EpisodeStep {
            observation: self.observe(),
            reward,
            terminated: fell,
            truncated: !fell && self.step >= EPISODE_STEPS,
        })
    }

    fn fail(&mut self, error: LocomotionError) -> EpisodeStep<Vec<f64>> {
        self.last_error = Some(error.to_string());
        EpisodeStep {
            observation: vec![0.0; self.config.observation_dim()],
            reward: -FALL_PENALTY,
            terminated: true,
            truncated: false,
        }
    }
}

impl Episode for Go2LearningEpisode {
    type Observation = Vec<f64>;
    type Action = [f64; 12];

    fn reset(&mut self) -> EpisodeStep<Vec<f64>> {
        self.episode_index = self.episode_index.wrapping_add(1);
        match self.try_reset() {
            Ok(observation) => EpisodeStep {
                observation,
                reward: 0.0,
                terminated: false,
                truncated: false,
            },
            Err(error) => self.fail(error),
        }
    }

    fn step(&mut self, action: [f64; 12]) -> EpisodeStep<Vec<f64>> {
        match self.try_step(action) {
            Ok(step) => step,
            Err(error) => self.fail(error),
        }
    }

    fn episode_index(&self) -> u32 {
        self.episode_index.saturating_sub(1)
    }

    fn step_in_episode(&self) -> u64 {
        self.step as u64
    }
}

/// Portable task contract of [`Go2LearningEpisode`] with `config`.
pub fn go2_learning_task_spec(config: &Go2LearningConfig) -> TaskSpec {
    let mut observation = vec![
        TensorSpec::new(
            "base_angular_velocity_quarter_rad_s",
            TensorDType::F64,
            vec![3],
            "rad/s",
        ),
        TensorSpec::new("projected_gravity", TensorDType::F64, vec![3], "1"),
        TensorSpec::new(
            "joint_position_offset_rad",
            TensorDType::F64,
            vec![12],
            "rad",
        ),
        TensorSpec::new(
            "joint_velocity_twentieth_rad_s",
            TensorDType::F64,
            vec![12],
            "rad/s",
        ),
        TensorSpec::new("previous_action", TensorDType::F64, vec![12], "1"),
        TensorSpec::new("gait_clock", TensorDType::F64, vec![2], "1"),
    ];
    if config.height_scan {
        observation.push(
            TensorSpec::new(
                "height_scan",
                TensorDType::F64,
                vec![HEIGHT_SCAN_POINTS],
                "1",
            )
            .with_bounds(TensorBounds::broadcast(-1.0, 1.0)),
        );
    }
    TaskSpec::new(
        "rne.unitree_go2.learning.v1",
        CONTROL_DT_S,
        ObservationSpec::new(observation),
        ActionSpec::new(vec![TensorSpec::new(
            "joint_action",
            TensorDType::F64,
            vec![12],
            "1",
        )
        .with_bounds(TensorBounds::broadcast(-1.0, 1.0))]),
        RewardSpec::weighted_sum(vec![
            RewardTermSpec::new("forward_progress", 1.0, "m"),
            RewardTermSpec::new("lateral_offset_growth", -LATERAL_PENALTY, "m"),
            RewardTermSpec::new("heading_growth", -HEADING_PENALTY, "rad"),
            RewardTermSpec::new(
                "action_change_squared",
                -SMOOTHNESS_PENALTY / EPISODE_STEPS as f64,
                "1",
            ),
            RewardTermSpec::new("fallen", -FALL_PENALTY, "1"),
        ]),
        TerminationSpec::new(
            vec![TerminationConditionSpec::new(
                "fallen",
                TerminationKind::Failure,
            )],
            Some(EPISODE_STEPS as u64),
        ),
        ResetSpec::splitmix64(true),
    )
}

/// One [`Go2LearningBatch`] reset or step, in environment order.
#[derive(Clone, Debug, PartialEq)]
pub struct Go2LearningBatchStep {
    /// Each environment's observation: after a step that ended its episode,
    /// the first observation of the next one.
    pub observations: Vec<Vec<f64>>,
    /// Each environment's reward for the step; zero at a reset.
    pub rewards: Vec<f64>,
    /// Whether the step ended the environment's episode with a fall.
    pub terminated: Vec<bool>,
    /// Whether the step ended it at [`EPISODE_STEPS`].
    pub truncated: Vec<bool>,
    /// The last observation of an episode the step ended, before the reset;
    /// `None` where the episode goes on.
    pub terminal_observations: Vec<Option<Vec<f64>>>,
}

/// Many [`Go2LearningEpisode`]s stepped together on worker threads, each
/// reset as soon as its episode ends, keeping the step's reward, its end
/// flags and the episode's last observation, as vectorized reinforcement
/// learning expects.
///
/// Environment `i` draws its starts from seed `seed + i`. The environments
/// are independent and the threads take contiguous chunks of them, so the
/// results are the same for any thread count.
#[derive(Debug)]
pub struct Go2LearningBatch {
    episodes: Vec<Go2LearningEpisode>,
    threads: usize,
}

impl Go2LearningBatch {
    /// `num_envs` episodes of `config`, environment `i` with seed
    /// `seed + i`, stepped on up to `threads` threads (zero or one: the
    /// calling thread).
    pub fn new(
        config: Go2LearningConfig,
        num_envs: usize,
        seed: u64,
        threads: usize,
    ) -> Result<Self, LocomotionError> {
        if num_envs == 0 {
            return Err(LocomotionError::InvalidInput(
                "a batch needs an environment",
            ));
        }
        let episodes = (0..num_envs)
            .map(|index| {
                Go2LearningEpisode::new(Go2LearningConfig {
                    seed: seed.wrapping_add(index as u64),
                    ..config
                })
            })
            .collect::<Result<_, _>>()?;
        Ok(Self { episodes, threads })
    }

    /// The number of environments.
    pub fn num_envs(&self) -> usize {
        self.episodes.len()
    }

    /// One environment's episode.
    pub fn episode(&self, index: usize) -> &Go2LearningEpisode {
        &self.episodes[index]
    }

    /// Resets every environment.
    pub fn reset(&mut self) -> Go2LearningBatchStep {
        let results = self.run(None, |episode, _| (episode.reset(), None));
        collect(results)
    }

    /// Steps every environment with its action, resetting those whose
    /// episode ended.
    ///
    /// # Panics
    ///
    /// Panics when `actions` does not hold one action per environment.
    pub fn step(&mut self, actions: &[[f64; 12]]) -> Go2LearningBatchStep {
        assert_eq!(
            actions.len(),
            self.episodes.len(),
            "one action per environment"
        );
        let results = self.run(Some(actions), |episode, action| {
            let step = episode.step(*action.expect("an action per environment"));
            if step.terminated || step.truncated {
                let next = episode.reset();
                let terminal = Some(step.observation);
                (
                    EpisodeStep {
                        observation: next.observation,
                        ..step_flags(step.reward, step.terminated, step.truncated)
                    },
                    terminal,
                )
            } else {
                (step, None)
            }
        });
        collect(results)
    }

    fn run<F>(&mut self, actions: Option<&[[f64; 12]]>, run: F) -> Vec<EnvResult>
    where
        F: Fn(&mut Go2LearningEpisode, Option<&[f64; 12]>) -> EnvResult + Sync,
    {
        let threads = self.threads.clamp(1, self.episodes.len());
        let chunk = self.episodes.len().div_ceil(threads);
        let run = &run;
        std::thread::scope(|scope| {
            let handles: Vec<_> = self
                .episodes
                .chunks_mut(chunk)
                .enumerate()
                .map(|(part, episodes)| {
                    scope.spawn(move || {
                        episodes
                            .iter_mut()
                            .enumerate()
                            .map(|(offset, episode)| {
                                let action = actions.map(|all| &all[part * chunk + offset]);
                                run(episode, action)
                            })
                            .collect::<Vec<_>>()
                    })
                })
                .collect();
            handles
                .into_iter()
                .flat_map(|handle| handle.join().expect("an environment thread"))
                .collect()
        })
    }
}

/// One environment's step, and its episode's last observation when the
/// step ended it.
type EnvResult = (EpisodeStep<Vec<f64>>, Option<Vec<f64>>);

/// An episode step with no observation yet.
fn step_flags(reward: f64, terminated: bool, truncated: bool) -> EpisodeStep<Vec<f64>> {
    EpisodeStep {
        observation: Vec::new(),
        reward,
        terminated,
        truncated,
    }
}

fn collect(results: Vec<EnvResult>) -> Go2LearningBatchStep {
    let mut step = Go2LearningBatchStep {
        observations: Vec::with_capacity(results.len()),
        rewards: Vec::with_capacity(results.len()),
        terminated: Vec::with_capacity(results.len()),
        truncated: Vec::with_capacity(results.len()),
        terminal_observations: Vec::with_capacity(results.len()),
    };
    for (result, terminal) in results {
        step.observations.push(result.observation);
        step.rewards.push(result.reward);
        step.terminated.push(result.terminated);
        step.truncated.push(result.truncated);
        step.terminal_observations.push(terminal);
    }
    step
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_standing_go2_settles_on_its_feet_and_earns_nothing() {
        let mut episode = Go2LearningEpisode::new(Go2LearningConfig::default()).unwrap();
        let first = episode.reset();
        assert_eq!(first.observation.len(), PROPRIOCEPTION);
        assert!(!first.terminated && !first.truncated);
        let mut total = 0.0;
        let mut last = first;
        for _ in 0..EPISODE_STEPS {
            last = episode.step([0.0; 12]);
            total += last.reward;
            assert!(!last.terminated);
        }
        assert!(last.truncated, "{last:?}");
        assert_eq!(episode.step_in_episode(), EPISODE_STEPS as u64);
        // Holding the default pose goes nowhere.
        assert!(total.abs() < 0.05, "{total}");
        let (position, _) = episode.robot().base_pose_z_up();
        assert!(position.z > 0.2 && position.z < 0.35, "{position:?}");
        let feet: f64 = episode
            .robot()
            .contact_load()
            .iter()
            .filter(|(name, _, _)| name.ends_with("_foot"))
            .map(|(_, impulse, _)| impulse)
            .sum();
        assert!(feet > 0.0);
    }

    #[test]
    fn random_starts_follow_the_training_stream_and_need_a_terrain() {
        assert!(Go2LearningEpisode::new(Go2LearningConfig {
            random_starts: true,
            ..Go2LearningConfig::default()
        })
        .is_err());
        let config = Go2LearningConfig {
            floor: Floor::Terrain {
                seed: 7,
                relief: Relief::default(),
            },
            random_starts: true,
            height_scan: true,
            seed: 2026,
            ..Go2LearningConfig::default()
        };
        let mut episode = Go2LearningEpisode::new(config).unwrap();
        let starts = training_starts(2026, 2, &[]);
        for start in &starts {
            let reset = episode.reset();
            assert_eq!(reset.observation.len(), PROPRIOCEPTION + HEIGHT_SCAN_POINTS);
            let (position, _) = episode.robot().base_pose_z_up();
            assert!((position.x - start[0]).abs() < 0.2 && (position.y - start[1]).abs() < 0.2);
        }
        assert_eq!(episode.episode_index(), 2);
    }

    #[test]
    fn a_fall_ends_the_episode_with_its_penalty() {
        let mut episode = Go2LearningEpisode::new(Go2LearningConfig::default()).unwrap();
        episode.reset();
        // Throwing every joint to one limit tips the robot over.
        let mut step = episode.step([1.0; 12]);
        let mut periods = 1;
        while !(step.terminated || step.truncated) {
            step = episode.step(if periods % 2 == 0 {
                [1.0; 12]
            } else {
                [-1.0; 12]
            });
            periods += 1;
        }
        if step.terminated {
            assert!(step.reward < -0.5, "{step:?}");
            assert!(periods < EPISODE_STEPS);
        }
        // Out-of-range actions are clamped to [-1, 1].
        episode.reset();
        let clamped = episode.step([5.0; 12]);
        assert!(clamped.observation[30..42].iter().all(|a| *a == 1.0));
    }

    #[test]
    fn a_batch_steps_the_same_on_any_thread_count() {
        use rne_ai::{VectorizedEpisode, VectorizedEpisodeConfig};
        let factory = |seed: u64| {
            Go2LearningEpisode::new(Go2LearningConfig {
                floor: Floor::Terrain {
                    seed: 7,
                    relief: Relief::default(),
                },
                height_scan: true,
                random_starts: true,
                seed,
                ..Go2LearningConfig::default()
            })
            .unwrap()
        };
        let config = VectorizedEpisodeConfig {
            num_envs: 3,
            seed: 11,
            auto_reset: true,
        };
        let mut serial = VectorizedEpisode::from_seeded(config, factory);
        let mut parallel = VectorizedEpisode::from_seeded(config, factory);
        let first = serial.reset();
        assert_eq!(first, parallel.reset_parallel(3));
        // Each environment starts on its own seed's stream.
        assert_ne!(first.observations[0], first.observations[1]);
        for period in 0..20 {
            let actions: Vec<[f64; 12]> = (0..3)
                .map(|env| std::array::from_fn(|joint| ((period + env + joint) as f64).sin() * 0.5))
                .collect();
            assert_eq!(serial.step(&actions), parallel.step_parallel(&actions, 2));
        }
        assert_eq!(serial.replay_digest(), parallel.replay_digest());
    }

    #[test]
    fn a_batch_resets_finished_episodes_and_keeps_their_ends() {
        let config = Go2LearningConfig::default();
        let mut serial = Go2LearningBatch::new(config, 3, 5, 1).unwrap();
        let mut threaded = Go2LearningBatch::new(config, 3, 5, 3).unwrap();
        assert_eq!(serial.reset(), threaded.reset());
        let mut ends = 0;
        for period in 0..EPISODE_STEPS + 5 {
            // Environment 0 stands; 1 and 2 thrash and fall.
            let actions: Vec<[f64; 12]> = (0..3)
                .map(|env| {
                    if env == 0 {
                        [0.0; 12]
                    } else {
                        [if period % 2 == 0 { 1.0 } else { -1.0 }; 12]
                    }
                })
                .collect();
            let step = serial.step(&actions);
            assert_eq!(step, threaded.step(&actions));
            for env in 0..3 {
                let ended = step.terminated[env] || step.truncated[env];
                assert_eq!(ended, step.terminal_observations[env].is_some());
                if ended {
                    ends += 1;
                    // The batch already holds the next episode's first state.
                    assert_eq!(serial.episode(env).step_in_episode(), 0);
                    assert_eq!(step.observations[env][30..42], [0.0; 12]);
                }
            }
            if period + 1 == EPISODE_STEPS {
                assert!(step.truncated[0], "{step:?}");
            }
        }
        assert!(ends >= 1);
        assert!(Go2LearningBatch::new(config, 0, 5, 1).is_err());
    }

    #[test]
    fn the_task_spec_describes_the_observation_and_action() {
        let blind = go2_learning_task_spec(&Go2LearningConfig::default());
        if let Err(error) = blind.validate() {
            panic!("{error}");
        }
        let width = |spec: &TaskSpec| {
            spec.observation
                .tensors
                .iter()
                .map(|tensor| tensor.shape.iter().product::<usize>())
                .sum::<usize>()
        };
        assert_eq!(width(&blind), PROPRIOCEPTION);
        let scan = go2_learning_task_spec(&Go2LearningConfig {
            height_scan: true,
            ..Go2LearningConfig::default()
        });
        assert_eq!(width(&scan), PROPRIOCEPTION + HEIGHT_SCAN_POINTS);
    }

    #[test]
    fn the_rewards_sum_to_the_progress_less_the_penalties() {
        let position = Vec3::new(1.0, 0.2, 0.3);
        let rotation = Quat::from_rotation_z(0.1);
        let start = RewardState {
            x_m: 0.25,
            lateral_m: 0.0,
            heading_rad: 0.0,
        };
        let (reward, now) = period_reward(start, position, rotation, 0.0, 2.0, false);
        let expected = 0.75
            - LATERAL_PENALTY * 0.2
            - HEADING_PENALTY * 0.1
            - SMOOTHNESS_PENALTY * 2.0 / EPISODE_STEPS as f64;
        assert!((reward - expected).abs() < 1e-12, "{reward} {expected}");
        assert_eq!(now.x_m, 1.0);
        let (fallen, _) = period_reward(now, position, rotation, 0.0, 0.0, true);
        assert!((fallen + FALL_PENALTY).abs() < 1e-12);
    }
}
