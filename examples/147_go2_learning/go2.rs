//! The Go2 plant and the walking episode both engines run.

use rne_ai::{PdJointConfig, PdJointWalker, PolicyWalkPlant, UrdfSceneSim};
use rne_core::SimDuration;
use rne_math::{Quat, Vec3};

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
/// Observation width: angular velocity, gravity, joint positions and
/// velocities, previous action, gait clock.
pub(crate) const OBSERVATIONS: usize = 3 + 3 + 12 + 12 + 12 + 2;
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

/// Which RNE backend simulates the robot.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Backend {
    /// Rapier through `UrdfSceneSim` and `PdJointWalker`.
    Rapier,
    /// `rne_physics_native`: `rne_dynamics`' articulated hard-contact stepping.
    Native,
    /// `rne_locomotion`'s terrain step: `rne_dynamics`' hard-contact stepping
    /// with spheres for the feet, knees and shins on a flat floor.
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
}

impl Default for Plant {
    fn default() -> Self {
        Self {
            backend: Backend::Rapier,
            solver_iterations: 4,
            inner_passes: 1,
            floor_friction: 1.0,
            terrain_seed: None,
        }
    }
}

/// What an episode needs of a simulated Go2, in the Z-up world.
pub(crate) trait Robot {
    /// Back to the start: default pose and targets, at rest, base at
    /// [`SPAWN_HEIGHT_M`].
    fn restart(&mut self) -> Result<()>;
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

/// Restarts the robot and stands it on its default pose for
/// [`SETTLE_STEPS`] control periods.
pub(crate) fn settle(robot: &mut dyn Robot) -> Result<()> {
    robot.restart()?;
    robot.set_targets_rad(&DEFAULT_RAD)?;
    for _ in 0..SETTLE_STEPS {
        robot.step()?;
    }
    Ok(())
}

/// The rough terrain for `seed`: a Y-up height field, zero at the origin.
pub(crate) fn terrain(seed: u64) -> Result<rne_physics::ColliderShape> {
    let spec = rne_physics::FractalTerrain {
        size_x_m: TERRAIN_SIZE_M,
        size_z_m: TERRAIN_SIZE_M,
        columns: TERRAIN_SAMPLES,
        rows: TERRAIN_SAMPLES,
        frequency_per_m: TERRAIN_FREQUENCY_PER_M,
        amplitude_m: TERRAIN_AMPLITUDE_M,
        ..rne_physics::FractalTerrain::default()
    };
    let raw = spec.height_field(seed)?;
    let origin = rne_physics::height_field_surface(&raw, 0.0, 0.0)
        .ok_or("the terrain does not cover the origin")?
        .height_m;
    Ok(rne_physics::FractalTerrain {
        height_offset_m: -origin,
        ..spec
    }
    .height_field(seed)?)
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
pub(crate) fn terrain_json(seed: u64) -> Result<serde_json::Value> {
    let shape = terrain(seed)?;
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
    Ok(serde_json::json!({
        "seed": seed,
        "frame": "z-up: row r at y = -size/2 + r size/(rows-1), column c at x = -size/2 + c size/(columns-1)",
        "size_m": TERRAIN_SIZE_M,
        "rows": rows,
        "columns": columns,
        "frequency_per_m": TERRAIN_FREQUENCY_PER_M,
        "amplitude_m": TERRAIN_AMPLITUDE_M,
        "heights_m": heights,
        "probes": probes,
    }))
}

/// Heading about +z of a Z-up orientation.
pub(crate) fn yaw(rotation: Quat) -> f64 {
    let Quat { x, y, z, w } = rotation;
    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}

/// The policy's observation of `walker` at control step `step` of an
/// episode, after the previous `action`.
pub(crate) fn observe(
    robot: &dyn Robot,
    step: usize,
    action: &[f64; 12],
) -> Result<[f64; OBSERVATIONS]> {
    let (_, rotation) = robot.base_pose_z_up();
    let (_, spin_world) = robot.base_velocity_z_up();
    let inverse = rotation.inverse();
    let spin = inverse * spin_world;
    let gravity = inverse * Vec3::new(0.0, 0.0, -1.0);
    let joints = robot.joint_states()?;
    let mut observation = [0.0; OBSERVATIONS];
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

/// Runs one episode, restarting the robot, with `policy` mapping an
/// observation to an action in [-1, 1].
pub(crate) fn episode(
    robot: &mut dyn Robot,
    mut policy: impl FnMut(&[f64; OBSERVATIONS]) -> [f64; 12],
) -> Result<Outcome> {
    settle(robot)?;
    let (start, _) = robot.base_pose_z_up();
    let mut action = [0.0; 12];
    let mut change_square = 0.0;
    let mut steps = 0;
    let mut fell = false;
    for step in 0..EPISODE_STEPS {
        let observation = observe(robot, step, &action)?;
        let next = policy(&observation);
        let mut targets = [0.0; 12];
        for index in 0..12 {
            change_square += (next[index] - action[index]).powi(2);
            targets[index] = DEFAULT_RAD[index] + ACTION_SCALE_RAD * next[index];
        }
        action = next;
        robot.set_targets_rad(&targets)?;
        robot.step()?;
        steps += 1;
        let (position, rotation) = robot.base_pose_z_up();
        let gravity_z = (rotation.inverse() * Vec3::new(0.0, 0.0, -1.0)).z;
        let clearance = position.z - robot.ground_height_m(position.x, position.y);
        if clearance < FALL_HEIGHT_M || gravity_z > FALL_GRAVITY_Z {
            fell = true;
            break;
        }
    }
    let (end, rotation) = robot.base_pose_z_up();
    let (forward_m, lateral_m, heading_rad) = (end.x - start.x, end.y - start.y, yaw(rotation));
    let fitness = forward_m
        - LATERAL_PENALTY * lateral_m.abs()
        - HEADING_PENALTY * heading_rad.abs()
        - SMOOTHNESS_PENALTY * change_square / steps as f64
        - if fell { FALL_PENALTY } else { 0.0 };
    Ok(Outcome {
        fitness,
        forward_m,
        lateral_m,
        heading_rad,
        survived_s: steps as f64 * CONTROL_DT_S,
        fell,
        control_steps: SETTLE_STEPS + steps,
    })
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
        let terrain = terrain_json(7).unwrap();
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
    fn a_still_policy_stands_on_the_terrain_and_rapier_refuses_it() {
        let plant = Plant {
            backend: Backend::Locomotion,
            terrain_seed: Some(7),
            ..Plant::default()
        };
        let mut go2 = robot(plant).unwrap();
        let outcome = episode(go2.as_mut(), |_| [0.0; 12]).unwrap();
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
            let outcome = episode(robot.as_mut(), |_| [0.0; 12]).unwrap();
            assert!(!outcome.fell, "{backend:?}: {outcome:?}");
            assert!(outcome.forward_m.abs() < 0.05, "{backend:?}: {outcome:?}");
            assert_eq!(outcome.control_steps, SETTLE_STEPS + EPISODE_STEPS);
            // A second episode on the same robot replays the first.
            assert_eq!(episode(robot.as_mut(), |_| [0.0; 12]).unwrap(), outcome);
        }
    }
}
