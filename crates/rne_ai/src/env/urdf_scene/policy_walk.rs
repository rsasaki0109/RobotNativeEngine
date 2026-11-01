//! unitree_rl_gym walking policies on an articulated URDF scene.
//!
//! [`UnitreeRlGymWalker`] runs a pretrained [`LstmPolicy`] the way
//! unitree_rl_gym deploys it in MuJoCo: the PD law `kp (target − q) − kd q̇`
//! is recomputed at every physics step and clamped to the URDF effort limit,
//! and the policy runs once per `control_decimation` steps. Its observation is
//! the base angular velocity and projected gravity in the base frame, the
//! scaled command, joint positions relative to the default pose, scaled joint
//! velocities, the previous action and the gait phase as `sin`/`cos`.
//!
//! RNE's world is Y-up while the policy was trained Z-up, so poses reported by
//! [`PolicyWalkStep`] use `(x, y, z)_zup = (x, −z, y)_yup`. Scenes must spawn
//! the robot with the URDF's Z-up frame rotated −90° about X, as the vendored
//! 12-DoF G1 scene does.

use std::f64::consts::{FRAC_PI_2, TAU};
use std::num::NonZeroUsize;
use std::path::PathBuf;

use rne_core::SimDuration;
use rne_ecs::Entity;
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, JointActuation, JointPassiveDynamics, PhysicsBackend, RevoluteJointArmature,
    RigidBody,
};
use rne_robot::{Joint, Link};

use super::{assets_scene_path, find_link_by_name, world_transform_of, UrdfSceneSim};
use crate::lstm_policy::{LstmPolicy, LstmPolicyError};

/// Vendored 12-DoF walking G1 scene for the unitree_rl_gym policy.
pub fn unitree_g1_12dof_walk_scene_path() -> PathBuf {
    assets_scene_path("unitree_g1_12dof_walk.rne.scene.toml")
}

/// Failure to configure or step a [`UnitreeRlGymWalker`].
#[derive(Debug, thiserror::Error)]
pub enum PolicyWalkError {
    /// The scene does not provide what the policy's deployment needs.
    #[error("policy walk model: {0}")]
    Model(String),
    /// The policy rejected its input.
    #[error(transparent)]
    Policy(#[from] LstmPolicyError),
}

/// Passive plant details matched to the MJCF the policy was deployed on.
///
/// The defaults reproduce unitree_rl_gym's G1 MJCF: joint `armature` 0.01,
/// `damping` 0.001, `frictionloss` 0.1 and MuJoCo's default friction of 1.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PolicyWalkPlant {
    /// Reflected rotor inertia added to each actuated joint in kg·m².
    ///
    /// A nonzero value needs `rne_physics_rapier`'s `experimental-armature`
    /// feature; without it [`UnitreeRlGymWalker::new`] returns an error.
    pub armature_kg_m2: f64,
    /// Viscous joint damping in N·m·s/rad.
    pub joint_damping_nm_s_per_rad: f64,
    /// Dry joint friction in N·m, regularized as `tanh(q̇ / transition)`.
    pub joint_friction_nm: f64,
    /// Velocity scale of the regularized dry friction in rad/s.
    pub joint_friction_transition_rad_s: f64,
    /// Friction coefficient given to the ground and every robot collider.
    pub floor_friction: f64,
}

impl Default for PolicyWalkPlant {
    fn default() -> Self {
        Self {
            armature_kg_m2: 0.01,
            joint_damping_nm_s_per_rad: 0.001,
            joint_friction_nm: 0.1,
            joint_friction_transition_rad_s: 0.05,
            floor_friction: 1.0,
        }
    }
}

/// One policy step.
#[derive(Clone, Debug, PartialEq)]
pub struct PolicyWalkStep {
    /// Simulation time after the step in seconds.
    pub time_s: f64,
    /// Observation the policy received.
    pub observation: Vec<f32>,
    /// Action the policy returned; it sets the targets for the next period.
    pub action: Vec<f32>,
    /// Base origin in the policy's Z-up world, in metres.
    pub base_position_m: [f64; 3],
    /// Base orientation in the policy's Z-up world as `[w, x, y, z]`.
    pub base_quaternion_wxyz: [f64; 4],
    /// Joints whose PD effort hit the effort limit in the last physics step.
    pub saturated_joints: usize,
}

#[derive(Clone, Copy, Debug)]
struct WalkJoint {
    link: Entity,
    effort_limit_nm: f64,
}

/// A pretrained unitree_rl_gym walking policy driving a URDF scene.
pub struct UnitreeRlGymWalker {
    sim: UrdfSceneSim,
    policy: LstmPolicy,
    joints: Vec<WalkJoint>,
    substeps: Vec<SimDuration>,
    targets_rad: Vec<f64>,
    observation: Vec<f32>,
    action: Vec<f32>,
    base_push_n: Option<Vec3>,
}

impl std::fmt::Debug for UnitreeRlGymWalker {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("UnitreeRlGymWalker")
            .field("joints", &self.joints.len())
            .field("time_s", &self.sim.sim_time().as_seconds().value())
            .finish_non_exhaustive()
    }
}

/// Rotation taking RNE's Y-up world to the policy's Z-up world.
fn y_up_to_z_up() -> Quat {
    Quat::from_rotation_x(FRAC_PI_2)
}

impl UnitreeRlGymWalker {
    /// Configures the scene's actuators and plant for the policy's deployment.
    ///
    /// Every joint in the deployment's `joint_names` must drive a revolute
    /// child link named `<joint without _joint>_link`; its URDF effort limit
    /// becomes the PD clamp. The scene's fixed step is set to the policy
    /// period and split into `control_decimation` physics steps.
    pub fn new(
        mut sim: UrdfSceneSim,
        policy: LstmPolicy,
        plant: PolicyWalkPlant,
    ) -> Result<Self, PolicyWalkError> {
        let model = |message: String| PolicyWalkError::Model(message);
        let deployment = policy.deployment().clone();
        let joint_count = deployment.joint_names.len();
        if policy.observation_len() != 9 + 3 * joint_count + 2 {
            return Err(model(format!(
                "the policy reads {} observations, the unitree_rl_gym layout for {joint_count} joints has {}",
                policy.observation_len(),
                9 + 3 * joint_count + 2
            )));
        }
        let plant_values = [
            plant.armature_kg_m2,
            plant.joint_damping_nm_s_per_rad,
            plant.joint_friction_nm,
            plant.floor_friction,
        ];
        if plant_values
            .iter()
            .any(|value| !value.is_finite() || *value < 0.0)
            || plant.joint_friction_transition_rad_s.is_nan()
            || plant.joint_friction_transition_rad_s <= 0.0
        {
            return Err(model("plant values must be finite and non-negative".into()));
        }
        let control_ticks = (deployment.control_dt_s() * 1.0e9).round() as u64;
        let control = SimDuration::from_ticks(control_ticks);
        let substeps = NonZeroUsize::new(deployment.control_decimation)
            .and_then(|count| control.split_evenly(count))
            .ok_or_else(|| model("the control period does not split into physics steps".into()))?;
        sim.set_fixed_delta(control);

        let collider_links: Vec<String> = sim
            .world
            .iter_entities()
            .filter(|entity| entity.get::<Collider>().is_some())
            .filter_map(|entity| entity.get::<Link>().map(|link| link.name.clone()))
            .collect();
        for name in std::iter::once("ground").chain(collider_links.iter().map(String::as_str)) {
            if !sim.set_named_collider_friction(name, plant.floor_friction) {
                return Err(model(format!("{name} has no collider to receive friction")));
            }
        }

        let mut joints = Vec::with_capacity(joint_count);
        for joint_name in &deployment.joint_names {
            let link_name = joint_name
                .strip_suffix("_joint")
                .map(|stem| format!("{stem}_link"))
                .ok_or_else(|| model(format!("{joint_name} does not end in _joint")))?;
            let link = find_link_by_name(&sim.world, &link_name)
                .ok_or_else(|| model(format!("missing link {link_name}")))?;
            let effort_limit_nm = sim
                .world
                .iter_entities()
                .filter_map(|entity| entity.get::<Joint>())
                .find(|joint| joint.child_link == link)
                .map(|joint| joint.limits.max_effort)
                .filter(|limit| limit.is_finite() && *limit > 0.0)
                .ok_or_else(|| model(format!("{link_name} has no positive effort limit")))?;
            if !sim.configure_named_revolute_effort_actuation(&link_name, effort_limit_nm) {
                return Err(model(format!("{link_name} is not a revolute joint")));
            }
            let mut entity = sim.world.entity_mut(link);
            entity.insert(JointPassiveDynamics::Revolute {
                viscous_damping_nm_s_per_rad: plant.joint_damping_nm_s_per_rad,
                coulomb_friction_nm: plant.joint_friction_nm,
                coulomb_transition_velocity_rad_s: plant.joint_friction_transition_rad_s,
            });
            if plant.armature_kg_m2 > 0.0 {
                entity.insert(RevoluteJointArmature {
                    inertia_kg_m2: plant.armature_kg_m2,
                });
            }
            joints.push(WalkJoint {
                link,
                effort_limit_nm,
            });
        }
        // Surface unsupported plant settings (a nonzero armature needs
        // rne_physics_rapier's `experimental-armature`) now, not mid-step.
        sim.backend
            .sync_from_ecs(&mut sim.world, sim.physics_world)
            .map_err(|error| model(error.to_string()))?;
        Ok(Self {
            targets_rad: deployment.default_joint_rad.clone(),
            observation: vec![0.0; policy.observation_len()],
            action: vec![0.0; policy.action_len()],
            sim,
            policy,
            joints,
            substeps,
            base_push_n: None,
        })
    }

    /// Scene being driven.
    pub fn sim(&self) -> &UrdfSceneSim {
        &self.sim
    }

    /// Policy being run.
    pub fn policy(&self) -> &LstmPolicy {
        &self.policy
    }

    /// URDF effort limits of the actuated joints in N·m, in action order.
    pub fn effort_limits_nm(&self) -> Vec<f64> {
        self.joints
            .iter()
            .map(|joint| joint.effort_limit_nm)
            .collect()
    }

    /// Applies a world-frame (Y-up) force at the base origin on every physics
    /// step until cleared with `None`, for push disturbances.
    pub fn set_base_push_n(&mut self, force_world_n: Option<Vec3>) {
        self.base_push_n = force_world_n.filter(|force| force.is_finite());
    }

    /// Base position and orientation in the policy's Z-up world.
    pub fn base_pose_z_up(&self) -> (Vec3, Quat) {
        let base = world_transform_of(&self.sim.world, self.sim.base_link);
        let t = base.translation;
        (Vec3::new(t.x, -t.z, t.y), y_up_to_z_up() * base.rotation)
    }

    fn joint_state(&self, link: Entity) -> Result<(f64, f64), PolicyWalkError> {
        let position = self
            .sim
            .backend
            .multibody_joint_position(self.sim.physics_world, link);
        let velocity = self
            .sim
            .backend
            .multibody_joint_velocity(self.sim.physics_world, link);
        position.zip(velocity).ok_or_else(|| {
            PolicyWalkError::Model("an actuated joint has no multibody state".into())
        })
    }

    fn observe(&mut self, command: [f64; 3]) -> Result<(), PolicyWalkError> {
        let deployment = self.policy.deployment();
        let base = world_transform_of(&self.sim.world, self.sim.base_link);
        let inverse = base.rotation.inverse();
        let omega_world = self
            .sim
            .world
            .get::<RigidBody>(self.sim.base_link)
            .map(|body| body.angular_velocity_rad_s)
            .unwrap_or(Vec3::ZERO);
        let omega = inverse * omega_world;
        let gravity = inverse * Vec3::new(0.0, -1.0, 0.0);
        let head = [
            omega.x * deployment.angular_velocity_scale,
            omega.y * deployment.angular_velocity_scale,
            omega.z * deployment.angular_velocity_scale,
            gravity.x,
            gravity.y,
            gravity.z,
            command[0] * deployment.command_scale[0],
            command[1] * deployment.command_scale[1],
            command[2] * deployment.command_scale[2],
        ];
        let joints = self.joints.len();
        let time_s = self.sim.sim_time().as_seconds().value();
        let phase = (time_s % deployment.gait_period_s) / deployment.gait_period_s;
        let default = deployment.default_joint_rad.clone();
        let velocity_scale = deployment.joint_velocity_scale;
        for (slot, value) in self.observation.iter_mut().zip(head) {
            *slot = value as f32;
        }
        for (index, default_rad) in default.iter().enumerate() {
            let (position, velocity) = self.joint_state(self.joints[index].link)?;
            self.observation[9 + index] = (position - default_rad) as f32;
            self.observation[9 + joints + index] = (velocity * velocity_scale) as f32;
        }
        self.observation[9 + 2 * joints..9 + 3 * joints].copy_from_slice(&self.action);
        self.observation[9 + 3 * joints] = (TAU * phase).sin() as f32;
        self.observation[10 + 3 * joints] = (TAU * phase).cos() as f32;
        Ok(())
    }

    /// Runs one policy period: PD-controlled physics steps, then the policy.
    ///
    /// `command` is (forward m/s, lateral m/s, yaw rate rad/s) in the base
    /// heading frame; unitree_rl_gym trains within ±1 on each.
    pub fn step(&mut self, command: [f64; 3]) -> Result<PolicyWalkStep, PolicyWalkError> {
        if command.iter().any(|value| !value.is_finite()) {
            return Err(PolicyWalkError::Model("command must be finite".into()));
        }
        let deployment = self.policy.deployment().clone();
        let mut saturated_joints = 0;
        for delta in self.substeps.clone() {
            saturated_joints = 0;
            for (index, joint) in self.joints.clone().iter().enumerate() {
                let (position, velocity) = self.joint_state(joint.link)?;
                let raw_nm = deployment.kp_nm_per_rad[index] * (self.targets_rad[index] - position)
                    - deployment.kd_nm_s_per_rad[index] * velocity;
                let effort_nm = raw_nm.clamp(-joint.effort_limit_nm, joint.effort_limit_nm);
                if effort_nm != raw_nm {
                    saturated_joints += 1;
                }
                if let Some(mut actuation) = self.sim.world.get_mut::<JointActuation>(joint.link) {
                    *actuation = JointActuation::RevoluteEffort {
                        effort_nm,
                        max_effort_nm: joint.effort_limit_nm,
                    };
                }
            }
            if let Some(force) = self.base_push_n {
                let base = self.sim.base_link;
                let point = world_transform_of(&self.sim.world, base).translation;
                self.sim
                    .pending_link_wrenches
                    .push(rne_physics::ExternalBodyWrench {
                        entity: base,
                        point_world_m: point,
                        force_world_n: force,
                        torque_world_nm: Vec3::ZERO,
                    });
            }
            self.sim.step_physics_with_delta(delta);
        }
        self.observe(command)?;
        self.policy.step(&self.observation, &mut self.action)?;
        for (index, target) in self.targets_rad.iter_mut().enumerate() {
            *target = f64::from(self.action[index]) * deployment.action_scale
                + deployment.default_joint_rad[index];
        }
        let (position, rotation) = self.base_pose_z_up();
        Ok(PolicyWalkStep {
            time_s: self.sim.sim_time().as_seconds().value(),
            observation: self.observation.clone(),
            action: self.action.clone(),
            base_position_m: position.to_array(),
            base_quaternion_wxyz: [rotation.w, rotation.x, rotation.y, rotation.z],
            saturated_joints,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lstm_policy::unitree_g1_walk_policy_manifest;

    fn walker(iterations: usize) -> UnitreeRlGymWalker {
        let sim = UrdfSceneSim::from_scene_path_with_solver_iterations(
            &unitree_g1_12dof_walk_scene_path(),
            iterations,
        )
        .expect("12-DoF G1 scene");
        let policy = LstmPolicy::load(&unitree_g1_walk_policy_manifest()).expect("policy");
        // rne_ai alone builds Rapier without the armature extension.
        let plant = PolicyWalkPlant {
            armature_kg_m2: 0.0,
            ..PolicyWalkPlant::default()
        };
        UnitreeRlGymWalker::new(sim, policy, plant).expect("walker")
    }

    #[test]
    fn an_in_memory_scene_walks_exactly_like_the_file_scene() {
        let scene_path = unitree_g1_12dof_walk_scene_path();
        let robot_path = scene_path
            .parent()
            .unwrap()
            .join("../robots/unitree_g1_12dof_walk.rne.robot.toml");
        let urdf_path = robot_path
            .parent()
            .unwrap()
            .join("g1_description/g1_12dof.rne.urdf");
        let scene_text = std::fs::read_to_string(&scene_path).unwrap();
        let robot_text = std::fs::read_to_string(&robot_path).unwrap();
        let urdf_text = std::fs::read_to_string(&urdf_path).unwrap();
        // A path that does not exist, so nothing can be read from disk.
        let virtual_scene = PathBuf::from("/embedded/scenes/walk.rne.scene.toml");
        let virtual_robot =
            PathBuf::from("/embedded/scenes/../robots/unitree_g1_12dof_walk.rne.robot.toml");
        let virtual_urdf =
            PathBuf::from("/embedded/scenes/../robots/g1_description/g1_12dof.rne.urdf");
        let mut sources = rne_assets::UrdfSourceMap::new();
        sources.insert(virtual_urdf, urdf_text.as_str());
        sources.insert(virtual_robot.clone(), robot_text.as_str());
        let bundle = rne_assets::parse_scene_bundle_with_sources(
            &virtual_scene,
            &scene_text,
            &[(virtual_robot, robot_text.as_str())],
            Some(&sources),
        )
        .expect("embedded bundle");
        let sim = UrdfSceneSim::from_scene_bundle(
            &virtual_scene,
            &bundle,
            &sources,
            4,
            SimDuration::from_ticks(20_000_000),
        )
        .expect("embedded scene");
        let policy = LstmPolicy::load(&unitree_g1_walk_policy_manifest()).unwrap();
        let plant = PolicyWalkPlant {
            armature_kg_m2: 0.0,
            ..PolicyWalkPlant::default()
        };
        let mut embedded = UnitreeRlGymWalker::new(sim, policy, plant).unwrap();
        let mut file = walker(4);
        for _ in 0..25 {
            let a = embedded.step([0.5, 0.0, 0.2]).unwrap();
            let b = file.step([0.5, 0.0, 0.2]).unwrap();
            assert_eq!(
                a.base_position_m.map(f64::to_bits),
                b.base_position_m.map(f64::to_bits)
            );
        }
    }

    #[test]
    fn armature_without_backend_support_is_an_error_not_a_panic() {
        let sim = UrdfSceneSim::from_scene_path(&unitree_g1_12dof_walk_scene_path()).unwrap();
        let policy = LstmPolicy::load(&unitree_g1_walk_policy_manifest()).unwrap();
        match UnitreeRlGymWalker::new(sim, policy, PolicyWalkPlant::default()) {
            Ok(mut walker) => {
                walker
                    .step([0.0; 3])
                    .expect("armature is supported in this build");
            }
            Err(PolicyWalkError::Model(message)) => assert!(message.contains("armature")),
            Err(other) => panic!("unexpected error {other}"),
        }
    }

    #[test]
    fn merged_g1_has_upstream_mass_and_limits() {
        let walker = walker(4);
        let (_, mass_kg) = walker.sim().dynamic_center_of_mass_m().expect("mass");
        assert!((mass_kg - 32.107).abs() < 1.0e-3, "mass {mass_kg}");
        assert_eq!(
            walker.effort_limits_nm(),
            [88.0, 139.0, 88.0, 139.0, 50.0, 50.0, 88.0, 139.0, 88.0, 139.0, 50.0, 50.0]
        );
    }

    #[test]
    fn walks_forward_upright_and_replays_exactly() {
        let run = || {
            let mut walker = walker(4);
            (0..150)
                .map(|_| walker.step([0.5, 0.0, 0.0]).expect("step"))
                .collect::<Vec<_>>()
        };
        let first = run();
        let last = first.last().unwrap();
        assert!((last.time_s - 3.0).abs() < 1e-9);
        assert!(
            last.base_position_m[0] > 0.9,
            "walked {:?}",
            last.base_position_m
        );
        assert!(first.iter().all(|step| step.base_position_m[2] > 0.7));
        let again = run();
        assert_eq!(
            first.last().unwrap().base_position_m.map(f64::to_bits),
            again.last().unwrap().base_position_m.map(f64::to_bits)
        );
    }

    #[test]
    fn a_sustained_push_moves_the_base() {
        let mut quiet = walker(2);
        let mut pushed = walker(2);
        pushed.set_base_push_n(Some(Vec3::new(0.0, 0.0, -60.0)));
        for _ in 0..10 {
            quiet.step([0.0; 3]).unwrap();
            pushed.step([0.0; 3]).unwrap();
        }
        // -Z in RNE's Y-up world is +Y (left) in the policy's Z-up frame.
        let drift = pushed.base_pose_z_up().0.y - quiet.base_pose_z_up().0.y;
        assert!(drift > 0.01, "push moved the base {drift} m");
    }
}
