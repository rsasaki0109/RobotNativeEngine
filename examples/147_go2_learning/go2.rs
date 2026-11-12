//! The Go2 plant and the walking episode both engines run.

use rne_ai::{PdJointConfig, PdJointWalker, PolicyWalkPlant, UrdfSceneSim};
use rne_core::SimDuration;
use rne_math::{Quat, Vec3};
use serde_json::{json, Value};

use crate::Result;
use rne_locomotion::go2_learning::{has_fallen, period_reward, RewardState};

/// Example 147's task lives in `rne_locomotion::go2_learning`: its
/// constants, terrains, starts and height scan are the library's.
pub(crate) use rne_locomotion::go2_learning::{
    evaluation_starts, gap_intervals, height_scan, terrain, terrain_height_m, training_starts, yaw,
    Relief, ACTION_SCALE_RAD, CONTROL_DT_S, DECIMATION, DEFAULT_RAD, DT_S, EPISODE_STEPS,
    FALL_PENALTY, HEADING_PENALTY, HEIGHT_SCAN_GAIN_PER_M, HEIGHT_SCAN_NOMINAL_M,
    HEIGHT_SCAN_POINTS, HEIGHT_SCAN_X_M, HEIGHT_SCAN_Y_M, KD_NM_S_PER_RAD, KP_NM_PER_RAD,
    LATERAL_PENALTY, LINKS, PROPRIOCEPTION, SETTLE_STEPS, SMOOTHNESS_PENALTY, SPAWN_HEIGHT_M,
};

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
    let scan = task
        .height_scan
        .then(|| height_scan(position, yaw(rotation), |x, y| robot.ground_height_m(x, y)));
    Ok(rne_locomotion::go2_learning::observation(
        rotation,
        spin_world,
        &robot.joint_states()?,
        step,
        action,
        scan.as_ref(),
    ))
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
    let mut last = RewardState {
        x_m: start.x,
        ..RewardState::default()
    };
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
        let ground = robot.ground_height_m(position.x, position.y);
        fell = has_fallen(position, rotation, ground, robot.arena_half_m());
        let (reward, now) = period_reward(last, position, rotation, start.y, step_change, fell);
        on_step(Step { reward, fell });
        last = now;
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
    use rne_locomotion::go2_learning::{
        Gaps, ARENA_EDGE_MARGIN_M, START_GAP_CLEARANCE_M, START_X_RANGE_M, START_Y_RANGE_M,
        TERRAIN_SIZE_M,
    };

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

#[cfg(test)]
mod library_episode {
    use super::*;
    use rne_ai::Episode;
    use rne_locomotion::go2_learning::{Floor, Go2LearningConfig, Go2LearningEpisode};

    /// A fixed policy that moves: each joint follows a mix of the clock and
    /// its own velocity.
    fn policy(observation: &[f64]) -> [f64; 12] {
        std::array::from_fn(|joint| {
            (0.8 * observation[42 + joint % 2] - 0.3 * observation[18 + joint]).tanh()
        })
    }

    #[test]
    fn the_library_episode_is_this_examples_episode() {
        let relief = Relief::default();
        let plant = Plant {
            backend: Backend::Locomotion,
            terrain_seed: Some(7),
            relief,
            ..Plant::default()
        };
        let task = Task {
            height_scan: true,
            random_starts: true,
        };
        let start = training_starts(2026, 0, &[])[0];
        let mut robot = robot(plant).unwrap();
        let mut rewards = Vec::new();
        let (outcome, last) = episode_with(robot.as_mut(), task, start, policy, |step| {
            rewards.push(step.reward)
        })
        .unwrap();
        let mut episode = Go2LearningEpisode::new(Go2LearningConfig {
            floor: Floor::Terrain { seed: 7, relief },
            height_scan: true,
            random_starts: true,
            seed: 2026,
            ..Go2LearningConfig::default()
        })
        .unwrap();
        let mut observation = episode.reset().observation;
        let mut library_rewards = Vec::new();
        loop {
            let step = episode.step(policy(&observation));
            library_rewards.push(step.reward);
            observation = step.observation;
            if step.terminated || step.truncated {
                assert_eq!(step.terminated, outcome.fell);
                break;
            }
        }
        assert!(rewards.len() > 50, "{}", rewards.len());
        assert_eq!(library_rewards, rewards);
        assert_eq!(observation, last);
    }
}
