//! Runs the unitree_rl_gym G1 walking policy on RNE's Rapier backend.
//!
//! The loop follows the upstream MuJoCo deployment: the PD law
//! `kp (target - q) - kd dq` is re-evaluated at every 2 ms physics step and
//! clamped to the URDF effort limits, and the policy runs every tenth step.
//! RNE's world is Y-up; observations and reported poses are converted to the
//! policy's Z-up frame, where `(x, y, z)_zup = (x, -z, y)_yup`.

use std::f64::consts::FRAC_PI_2;
use std::path::Path;

use rne_ai::{UrdfJointPdEffortTarget, UrdfSceneSim};
use rne_core::SimDuration;
use rne_math::{Quat, Vec3};
use rne_physics::{JointPassiveDynamics, RevoluteJointArmature};
use rne_robot::{Joint, Link};
use serde_json::{json, Value};

use crate::policy::{LstmPolicy, Result, ACTION_LEN, OBSERVATION_LEN};

/// Pelvis height below which a run counts as a fall, in metres.
pub(crate) const FALL_HEIGHT_M: f64 = 0.55;
/// Speed ceiling passed to the portable PD so it never derates torque:
/// the MuJoCo deployment applies no torque-speed curve.
const NO_SPEED_DERATING_RAD_S: f64 = 1.0e12;

/// Plant settings that make RNE's model match unitree_rl_gym's MJCF.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PlantConfig {
    /// Constraint-solver iterations per physics step.
    pub(crate) solver_iterations: usize,
    /// Reflected rotor inertia per joint, MJCF `armature`.
    pub(crate) armature_kg_m2: f64,
    /// Joint viscous damping, MJCF `damping`.
    pub(crate) joint_damping_nm_s_per_rad: f64,
    /// Joint dry friction, MJCF `frictionloss` (regularized in RNE).
    pub(crate) joint_friction_nm: f64,
    /// Floor and foot friction coefficient (MuJoCo default 1).
    pub(crate) floor_friction: f64,
}

impl Default for PlantConfig {
    fn default() -> Self {
        Self {
            solver_iterations: 4,
            armature_kg_m2: 0.01,
            joint_damping_nm_s_per_rad: 0.001,
            joint_friction_nm: 0.1,
            floor_friction: 1.0,
        }
    }
}

/// Velocity scale of RNE's smooth Coulomb transition for `frictionloss`.
const FRICTION_TRANSITION_RAD_S: f64 = 0.05;

/// One 50 Hz policy step in the policy's Z-up world.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct StepRecord {
    pub(crate) time_s: f64,
    pub(crate) observation: [f32; OBSERVATION_LEN],
    pub(crate) action: [f32; ACTION_LEN],
    pub(crate) base_position_m: [f64; 3],
    pub(crate) base_quaternion_wxyz: [f64; 4],
    pub(crate) saturated_joints: usize,
}

/// G1 plant, policy and controller state.
pub(crate) struct G1Walker {
    pub(crate) sim: UrdfSceneSim,
    pub(crate) policy: LstmPolicy,
    links: Vec<String>,
    effort_limits_nm: [f64; ACTION_LEN],
    targets_rad: [f64; ACTION_LEN],
    action: [f32; ACTION_LEN],
}

/// Scene used by the walker.
pub(crate) fn scene_path() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/scenes/unitree_g1_12dof_walk.rne.scene.toml")
}

fn link_entity(sim: &UrdfSceneSim, name: &str) -> Option<rne_ecs::Entity> {
    let world = sim.world();
    world
        .iter_entities()
        .map(|entity| entity.id())
        .find(|entity| {
            world
                .get::<Link>(*entity)
                .is_some_and(|link| link.name == name)
        })
}

fn effort_limit_nm(sim: &UrdfSceneSim, link: rne_ecs::Entity) -> Option<f64> {
    let world = sim.world();
    world
        .iter_entities()
        .filter_map(|entity| world.get::<Joint>(entity.id()))
        .find(|joint| joint.child_link == link)
        .map(|joint| joint.limits.max_effort)
}

/// Rotation taking RNE's Y-up world to the policy's Z-up world.
fn y_up_to_z_up() -> Quat {
    Quat::from_rotation_x(FRAC_PI_2)
}

fn z_up(vector: Vec3) -> [f64; 3] {
    [vector.x, -vector.z, vector.y]
}

impl G1Walker {
    /// Loads the scene, configures the matched plant and the PD actuators.
    pub(crate) fn new(policy: LstmPolicy, plant: PlantConfig) -> Result<Self> {
        let deployment = policy.deployment.clone();
        let control_ticks = (deployment.control_dt_s() * 1.0e9).round() as u64;
        let mut sim = UrdfSceneSim::from_scene_path_with_solver_iterations_and_fixed_delta(
            &scene_path(),
            plant.solver_iterations,
            SimDuration::from_ticks(control_ticks),
        )?;
        for name in ["ground", "left_ankle_roll_link", "right_ankle_roll_link"] {
            if !sim.set_named_collider_friction(name, plant.floor_friction) {
                return Err(format!("{name} has no collider to receive friction").into());
            }
        }
        let mut links = Vec::with_capacity(ACTION_LEN);
        let mut effort_limits_nm = [0.0; ACTION_LEN];
        for (index, joint) in deployment.joint_names.iter().enumerate() {
            let link = joint.replace("_joint", "_link");
            let entity = link_entity(&sim, &link).ok_or_else(|| format!("missing link {link}"))?;
            effort_limits_nm[index] =
                effort_limit_nm(&sim, entity).ok_or_else(|| format!("no joint drives {link}"))?;
            if !sim.configure_named_revolute_effort_actuation(&link, effort_limits_nm[index]) {
                return Err(format!("{link} is not a revolute joint").into());
            }
            let mut entity_mut = sim.world_mut().entity_mut(entity);
            entity_mut.insert(JointPassiveDynamics::Revolute {
                viscous_damping_nm_s_per_rad: plant.joint_damping_nm_s_per_rad,
                coulomb_friction_nm: plant.joint_friction_nm,
                coulomb_transition_velocity_rad_s: FRICTION_TRANSITION_RAD_S,
            });
            if plant.armature_kg_m2 > 0.0 {
                entity_mut.insert(RevoluteJointArmature {
                    inertia_kg_m2: plant.armature_kg_m2,
                });
            }
            links.push(link);
        }
        Ok(Self {
            targets_rad: deployment.default_joint_rad,
            sim,
            policy,
            links,
            effort_limits_nm,
            action: [0.0; ACTION_LEN],
        })
    }

    /// Pelvis position and orientation in the policy's Z-up world.
    pub(crate) fn base_pose(&self) -> Result<(Vec3, Quat)> {
        let pelvis = self
            .sim
            .named_transform("pelvis")
            .ok_or("pelvis is missing")?;
        let rotation = y_up_to_z_up() * pelvis.rotation;
        let [x, y, z] = z_up(pelvis.translation);
        Ok((Vec3::new(x, y, z), rotation))
    }

    fn observe(&self, command: [f64; 3]) -> Result<[f32; OBSERVATION_LEN]> {
        let spec = &self.policy.deployment;
        let pelvis = self
            .sim
            .named_transform("pelvis")
            .ok_or("pelvis is missing")?;
        let inverse = pelvis.rotation.inverse();
        let observed = self.sim.observe();
        let omega_world = Vec3::new(
            observed.base_angular_velocity_x_rad_s,
            observed.base_angular_velocity_y_rad_s,
            observed.base_angular_velocity_z_rad_s,
        );
        let omega_body = inverse * omega_world;
        let gravity_body = inverse * Vec3::new(0.0, -1.0, 0.0);
        let time_s = self.sim.sim_time().as_seconds().value();
        let phase = (time_s % spec.gait_period_s) / spec.gait_period_s;
        let mut obs = [0.0_f32; OBSERVATION_LEN];
        let values = [
            omega_body.x * spec.angular_velocity_scale,
            omega_body.y * spec.angular_velocity_scale,
            omega_body.z * spec.angular_velocity_scale,
            gravity_body.x,
            gravity_body.y,
            gravity_body.z,
            command[0] * spec.command_scale[0],
            command[1] * spec.command_scale[1],
            command[2] * spec.command_scale[2],
        ];
        for (slot, value) in obs.iter_mut().zip(values) {
            *slot = value as f32;
        }
        for (index, link) in self.links.iter().enumerate() {
            let position = self
                .sim
                .named_joint_position(link)
                .ok_or_else(|| format!("{link} has no joint position"))?;
            let velocity = self
                .sim
                .named_joint_velocity(link)
                .ok_or_else(|| format!("{link} has no joint velocity"))?;
            obs[9 + index] = (position - spec.default_joint_rad[index]) as f32;
            obs[21 + index] = (velocity * spec.joint_velocity_scale) as f32;
        }
        obs[33..45].copy_from_slice(&self.action);
        let angle = std::f64::consts::TAU * phase;
        obs[45] = angle.sin() as f32;
        obs[46] = angle.cos() as f32;
        Ok(obs)
    }

    /// Runs one policy period: ten PD-controlled physics steps, then the policy.
    pub(crate) fn step(&mut self, command: [f64; 3]) -> Result<StepRecord> {
        let spec = &self.policy.deployment;
        let targets: Vec<_> = self
            .links
            .iter()
            .enumerate()
            .map(|(index, link)| UrdfJointPdEffortTarget {
                link_name: link,
                target_position_rad: self.targets_rad[index],
                stiffness_nm_per_rad: spec.kp_nm_per_rad[index],
                damping_nm_s_per_rad: spec.kd_nm_s_per_rad[index],
                max_effort_nm: self.effort_limits_nm[index],
                max_velocity_rad_s: NO_SPEED_DERATING_RAD_S,
                transmission_efficiency: 1.0,
            })
            .collect();
        let applied = self
            .sim
            .step_joint_pd_effort_targets_substeps(&targets, spec.control_decimation)?;
        let saturated_joints = applied
            .iter()
            .zip(&self.effort_limits_nm)
            .filter(|(effort, limit)| effort.effort_nm.abs() >= **limit)
            .count();
        let observation = self.observe(command)?;
        self.action = self.policy.step(&observation);
        let spec = &self.policy.deployment;
        for index in 0..ACTION_LEN {
            self.targets_rad[index] =
                f64::from(self.action[index]) * spec.action_scale + spec.default_joint_rad[index];
        }
        let (position, rotation) = self.base_pose()?;
        Ok(StepRecord {
            time_s: self.sim.sim_time().as_seconds().value(),
            observation,
            action: self.action,
            base_position_m: position.to_array(),
            base_quaternion_wxyz: [rotation.w, rotation.x, rotation.y, rotation.z],
            saturated_joints,
        })
    }
}

fn yaw(q: [f64; 4]) -> f64 {
    let [w, x, y, z] = q;
    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}

fn round4(value: f64) -> f64 {
    (value * 1.0e4).round() / 1.0e4
}

/// Walking metrics; keep identical to `metrics.py`.
pub(crate) fn summarize(records: &[StepRecord], command: [f64; 3]) -> Value {
    let heights: Vec<f64> = records.iter().map(|r| r.base_position_m[2]).collect();
    let fall_time_s = records
        .iter()
        .zip(&heights)
        .find(|(_, height)| **height < FALL_HEIGHT_M)
        .map(|(record, _)| record.time_s);
    let half = records.len() / 2;
    let (mut forward, mut lateral, mut turn) = (Vec::new(), Vec::new(), Vec::new());
    for index in half.max(1)..records.len() {
        let (previous, current) = (&records[index - 1], &records[index]);
        let dt = current.time_s - previous.time_s;
        let dx = current.base_position_m[0] - previous.base_position_m[0];
        let dy = current.base_position_m[1] - previous.base_position_m[1];
        let heading = yaw(previous.base_quaternion_wxyz);
        forward.push((heading.cos() * dx + heading.sin() * dy) / dt);
        lateral.push((-heading.sin() * dx + heading.cos() * dy) / dt);
        let delta = yaw(current.base_quaternion_wxyz) - heading;
        let wrapped = delta - std::f64::consts::TAU * (delta / std::f64::consts::TAU).round();
        turn.push(wrapped / dt);
    }
    let mean = |values: &[f64]| values.iter().sum::<f64>() / values.len().max(1) as f64;
    let last = records
        .last()
        .map(|r| r.base_position_m)
        .unwrap_or_default();
    json!({
        "fell": fall_time_s.is_some(), "fall_time_s": fall_time_s,
        "final_base_position_m": last.map(round4),
        "min_base_height_m": round4(heights.iter().copied().fold(f64::INFINITY, f64::min)),
        "mean_base_height_m": round4(mean(&heights)),
        "second_half_forward_speed_m_s": round4(mean(&forward)),
        "second_half_lateral_speed_m_s": round4(mean(&lateral)),
        "second_half_yaw_rate_rad_s": round4(mean(&turn)),
        "forward_speed_error_m_s": round4(mean(&forward) - command[0]),
        "yaw_rate_error_rad_s": round4(mean(&turn) - command[2]),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn walker() -> G1Walker {
        let policy = LstmPolicy::load(&crate::policy::default_manifest()).expect("policy");
        G1Walker::new(policy, PlantConfig::default()).expect("walker")
    }

    fn walk(seconds: f64, command: [f64; 3]) -> Vec<StepRecord> {
        let mut walker = walker();
        (0..(seconds / 0.02).round() as usize)
            .map(|_| walker.step(command).expect("step"))
            .collect()
    }

    #[test]
    fn plant_mass_and_effort_limits_match_the_mjcf() {
        let walker = walker();
        // The MJCF's 12-DoF G1 weighs 32.107 kg. The sensor frames must not
        // pick up the importer's 1 kg default for links without <inertial>.
        let (_, mass_kg) = walker.sim.dynamic_center_of_mass_m().expect("mass");
        assert!((mass_kg - 32.107).abs() < 1.0e-3, "mass {mass_kg}");
        assert_eq!(
            walker.effort_limits_nm,
            [88.0, 139.0, 88.0, 139.0, 50.0, 50.0, 88.0, 139.0, 88.0, 139.0, 50.0, 50.0]
        );
    }

    #[test]
    fn replay_is_bit_exact() {
        let first = walk(1.0, [0.5, 0.0, 0.0]);
        let second = walk(1.0, [0.5, 0.0, 0.0]);
        assert_eq!(first.len(), 50);
        for (a, b) in first.iter().zip(&second) {
            assert_eq!(a.time_s.to_bits(), b.time_s.to_bits());
            assert_eq!(a.action.map(f32::to_bits), b.action.map(f32::to_bits));
            assert_eq!(
                a.base_position_m.map(f64::to_bits),
                b.base_position_m.map(f64::to_bits)
            );
            assert_eq!(
                a.base_quaternion_wxyz.map(f64::to_bits),
                b.base_quaternion_wxyz.map(f64::to_bits)
            );
        }
    }

    #[test]
    fn walks_like_the_mujoco_reference() {
        let reference: Value =
            serde_json::from_str(include_str!("fixtures/mujoco_reference.json")).expect("fixture");
        let mujoco = &reference["runs"][0];
        let command: [f64; 3] = serde_json::from_value(mujoco["command"].clone()).unwrap();
        assert_eq!(command, [0.5, 0.0, 0.0]);
        let summary = summarize(&walk(10.0, command), command);
        let field = |value: &Value, key: &str| value[key].as_f64().expect(key);
        assert_eq!(summary["fell"], false);
        for (key, tolerance) in [
            ("second_half_forward_speed_m_s", 0.04),
            ("second_half_yaw_rate_rad_s", 0.03),
            ("mean_base_height_m", 0.005),
        ] {
            let (rne, reference) = (field(&summary, key), field(&mujoco["summary"], key));
            assert!(
                (rne - reference).abs() < tolerance,
                "{key}: RNE {rne}, MuJoCo {reference}"
            );
        }
    }
}
