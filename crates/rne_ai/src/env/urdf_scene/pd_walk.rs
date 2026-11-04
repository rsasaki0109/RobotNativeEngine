//! PD joint-target control of an articulated URDF scene.
//!
//! [`PdJointWalker`] drives a scene the way [`super::UnitreeRlGymWalker`] does,
//! without a policy of its own: the caller sets joint targets once per control
//! period and the PD law `kp (target − q) − kd q̇`, clamped to each joint's URDF
//! effort limit, is recomputed at every physics step. Policies trained outside
//! RNE's crates, such as an evolution-strategies MLP, step a robot through it.
//!
//! RNE's world is Y-up; poses and velocities it reports use the Z-up
//! convention `(x, y, z)_zup = (x, −z, y)_yup`, as the walker's do.

use std::num::NonZeroUsize;

use rne_core::SimDuration;
use rne_ecs::Entity;
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, JointActuation, JointPassiveDynamics, JointState, MultibodyLink, PhysicsBackend,
    PhysicsOwnedPose, RevoluteJointArmature, RigidBody,
};
use rne_robot::{Joint, Link};

use super::{
    find_link_by_name, world_transform_of, PolicyWalkError, PolicyWalkPlant, UrdfSceneSim,
};

/// One PD-driven joint: its child link and its URDF effort limit.
#[derive(Clone, Copy, Debug)]
pub(super) struct PdJoint {
    pub(super) link: Entity,
    pub(super) effort_limit_nm: f64,
}

/// Configures `sim` for PD control of the revolute joints whose child links
/// are `link_names`: the floor and every collider get the plant's friction,
/// each joint gets effort actuation at its URDF limit and the plant's passive
/// dynamics, Rapier owns every articulated pose, and the fixed step becomes
/// `control` split into `decimation` physics steps.
pub(super) fn configure_pd_joints(
    sim: &mut UrdfSceneSim,
    link_names: &[String],
    plant: PolicyWalkPlant,
    control: SimDuration,
    decimation: usize,
) -> Result<(Vec<PdJoint>, Vec<SimDuration>), PolicyWalkError> {
    let model = |message: String| PolicyWalkError::Model(message);
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
    let substeps = NonZeroUsize::new(decimation)
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

    let mut joints = Vec::with_capacity(link_names.len());
    for link_name in link_names {
        let link = find_link_by_name(&sim.world, link_name)
            .ok_or_else(|| model(format!("missing link {link_name}")))?;
        let effort_limit_nm = sim
            .world
            .iter_entities()
            .filter_map(|entity| entity.get::<Joint>())
            .find(|joint| joint.child_link == link)
            .map(|joint| joint.limits.max_effort)
            .filter(|limit| limit.is_finite() && *limit > 0.0)
            .ok_or_else(|| model(format!("{link_name} has no positive effort limit")))?;
        if !sim.configure_named_revolute_effort_actuation(link_name, effort_limit_nm) {
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
        joints.push(PdJoint {
            link,
            effort_limit_nm,
        });
    }
    // The walker never moves a link through the ECS, so Rapier owns every
    // articulated pose. Without the marker each step pushed the ECS copy
    // back, which differs from Rapier's by the f32/f64 and world/local
    // round trip, injecting that rounding error into every step.
    let articulated: Vec<Entity> = sim
        .world
        .iter_entities()
        .filter(|entity| entity.get::<MultibodyLink>().is_some())
        .map(|entity| entity.id())
        .collect();
    for entity in articulated {
        sim.world.entity_mut(entity).insert(PhysicsOwnedPose);
    }
    // Surface unsupported plant settings (a nonzero armature needs
    // rne_physics_rapier's `experimental-armature`) now, not mid-step.
    sim.backend
        .sync_from_ecs(&mut sim.world, sim.physics_world)
        .map_err(|error| model(error.to_string()))?;
    Ok((joints, substeps))
}

/// A joint's position and velocity.
pub(super) fn joint_state(
    sim: &UrdfSceneSim,
    stepped: bool,
    link: Entity,
) -> Result<(f64, f64), PolicyWalkError> {
    // After a step the backend has decoded each joint into `JointState`
    // from the same Rapier state, with the same function; reuse it.
    if stepped {
        match sim.world.get::<JointState>(link) {
            Some(JointState::Revolute {
                position_rad,
                velocity_rad_s,
            }) => return Ok((*position_rad, *velocity_rad_s)),
            Some(JointState::Prismatic {
                position_m,
                velocity_m_s,
            }) => return Ok((*position_m, *velocity_m_s)),
            _ => {}
        }
    }
    sim.backend
        .multibody_joint_state(sim.physics_world, link)
        .ok_or_else(|| PolicyWalkError::Model("an actuated joint has no multibody state".into()))
}

/// The gains, targets and push of one control period.
pub(super) struct PdPeriod<'a> {
    pub(super) targets_rad: &'a [f64],
    pub(super) kp_nm_per_rad: &'a [f64],
    pub(super) kd_nm_s_per_rad: &'a [f64],
    pub(super) base_push_n: Option<Vec3>,
}

/// Runs the PD-controlled physics steps of one control period; returns how
/// many joints hit their effort limit in the last of them.
pub(super) fn step_pd(
    sim: &mut UrdfSceneSim,
    joints: &[PdJoint],
    substeps: &[SimDuration],
    period: &PdPeriod,
    stepped: &mut bool,
) -> Result<usize, PolicyWalkError> {
    let mut saturated_joints = 0;
    for &delta in substeps {
        saturated_joints = 0;
        for (index, joint) in joints.iter().enumerate() {
            let (position, velocity) = joint_state(sim, *stepped, joint.link)?;
            let raw_nm = period.kp_nm_per_rad[index] * (period.targets_rad[index] - position)
                - period.kd_nm_s_per_rad[index] * velocity;
            let effort_nm = raw_nm.clamp(-joint.effort_limit_nm, joint.effort_limit_nm);
            if effort_nm != raw_nm {
                saturated_joints += 1;
            }
            if let Some(mut actuation) = sim.world.get_mut::<JointActuation>(joint.link) {
                *actuation = JointActuation::RevoluteEffort {
                    effort_nm,
                    max_effort_nm: joint.effort_limit_nm,
                };
            }
        }
        if let Some(force) = period.base_push_n {
            let base = sim.base_link;
            let point = world_transform_of(&sim.world, base).translation;
            sim.pending_link_wrenches
                .push(rne_physics::ExternalBodyWrench {
                    entity: base,
                    point_world_m: point,
                    force_world_n: force,
                    torque_world_nm: Vec3::ZERO,
                });
        }
        // After a full step, only joint actuation changes in the ECS
        // between steps (pushes go straight to the backend) and Rapier
        // owns every articulated pose, so the lighter synchronization
        // gives the same result.
        if *stepped {
            sim.step_physics_actuated(delta);
        } else {
            sim.step_physics_with_delta(delta);
        }
        *stepped = true;
    }
    Ok(saturated_joints)
}

/// Rotation taking RNE's Y-up world to the Z-up world.
fn y_up_to_z_up() -> Quat {
    Quat::from_rotation_x(std::f64::consts::FRAC_PI_2)
}

/// A Y-up vector in the Z-up world.
fn z_up(v: Vec3) -> Vec3 {
    Vec3::new(v.x, -v.z, v.y)
}

/// What [`PdJointWalker`] drives and how.
#[derive(Clone, Debug, PartialEq)]
pub struct PdJointConfig {
    /// Child links of the actuated revolute joints, in target order.
    pub link_names: Vec<String>,
    /// Proportional gain of every joint in N·m/rad.
    pub kp_nm_per_rad: Vec<f64>,
    /// Derivative gain of every joint in N·m·s/rad.
    pub kd_nm_s_per_rad: Vec<f64>,
    /// Targets after construction and [`PdJointWalker::reset`].
    pub default_joint_rad: Vec<f64>,
    /// Control period: targets hold for one period.
    pub control_period: SimDuration,
    /// Physics steps per control period.
    pub decimation: usize,
    /// Passive joint dynamics and friction.
    pub plant: PolicyWalkPlant,
}

/// An articulated URDF scene whose joints track targets set once per control
/// period through a PD law recomputed at every physics step.
pub struct PdJointWalker {
    sim: UrdfSceneSim,
    config: PdJointConfig,
    joints: Vec<PdJoint>,
    substeps: Vec<SimDuration>,
    targets_rad: Vec<f64>,
    kp_nm_per_rad: Vec<f64>,
    kd_nm_s_per_rad: Vec<f64>,
    base_push_n: Option<Vec3>,
    stepped: bool,
}

impl std::fmt::Debug for PdJointWalker {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("PdJointWalker")
            .field("joints", &self.joints.len())
            .field("time_s", &self.time_s())
            .finish_non_exhaustive()
    }
}

impl PdJointWalker {
    /// Configures `sim` for `config`: every link must hang from a revolute
    /// joint with a positive URDF effort limit, and every gain and default
    /// target list must have one entry per link.
    pub fn new(mut sim: UrdfSceneSim, config: PdJointConfig) -> Result<Self, PolicyWalkError> {
        let joints = config.link_names.len();
        if joints == 0
            || [
                config.kp_nm_per_rad.len(),
                config.kd_nm_s_per_rad.len(),
                config.default_joint_rad.len(),
            ]
            .iter()
            .any(|&len| len != joints)
        {
            return Err(PolicyWalkError::Model(
                "gains and default targets need one entry per joint".into(),
            ));
        }
        if config
            .kp_nm_per_rad
            .iter()
            .chain(&config.kd_nm_s_per_rad)
            .chain(&config.default_joint_rad)
            .any(|value| !value.is_finite())
        {
            return Err(PolicyWalkError::Model(
                "gains and default targets must be finite".into(),
            ));
        }
        let (pd_joints, substeps) = configure_pd_joints(
            &mut sim,
            &config.link_names,
            config.plant,
            config.control_period,
            config.decimation,
        )?;
        Ok(Self {
            sim,
            targets_rad: config.default_joint_rad.clone(),
            kp_nm_per_rad: config.kp_nm_per_rad.clone(),
            kd_nm_s_per_rad: config.kd_nm_s_per_rad.clone(),
            config,
            joints: pd_joints,
            substeps,
            base_push_n: None,
            stepped: false,
        })
    }

    /// Returns the walker to the state [`Self::new`] left it in, without
    /// reloading the scene; stepping afterwards replays a new walker exactly.
    pub fn reset(&mut self) -> Result<(), PolicyWalkError> {
        self.sim
            .reset()
            .map_err(|error| PolicyWalkError::Model(error.to_string()))?;
        let (joints, substeps) = configure_pd_joints(
            &mut self.sim,
            &self.config.link_names,
            self.config.plant,
            self.config.control_period,
            self.config.decimation,
        )?;
        self.joints = joints;
        self.substeps = substeps;
        self.targets_rad.clone_from(&self.config.default_joint_rad);
        self.kp_nm_per_rad.clone_from(&self.config.kp_nm_per_rad);
        self.kd_nm_s_per_rad
            .clone_from(&self.config.kd_nm_s_per_rad);
        self.base_push_n = None;
        self.stepped = false;
        Ok(())
    }

    /// Scene being driven.
    pub fn sim(&self) -> &UrdfSceneSim {
        &self.sim
    }

    /// Configuration the walker was built with.
    pub fn config(&self) -> &PdJointConfig {
        &self.config
    }

    /// Simulation time in seconds.
    pub fn time_s(&self) -> f64 {
        self.sim.sim_time().as_seconds().value()
    }

    /// URDF effort limits in N·m, in target order.
    pub fn effort_limits_nm(&self) -> Vec<f64> {
        self.joints
            .iter()
            .map(|joint| joint.effort_limit_nm)
            .collect()
    }

    /// Sets how soft the scene's contacts are, as
    /// [`UrdfSceneSim::set_contact_softness`]; kept across [`Self::reset`].
    pub fn set_contact_softness(&mut self, natural_frequency_hz: f64, damping_ratio: f64) -> bool {
        self.sim
            .set_contact_softness(natural_frequency_hz, damping_ratio)
    }

    /// Sets the constraint solver's inner passes per solver iteration, as
    /// [`UrdfSceneSim::set_inner_solver_passes`]; kept across [`Self::reset`].
    pub fn set_inner_solver_passes(&mut self, passes: usize) -> bool {
        self.sim.set_inner_solver_passes(passes)
    }

    /// Applies a world-frame (Y-up) force at the base origin on every physics
    /// step until cleared with `None`.
    pub fn set_base_push_n(&mut self, force_world_n: Option<Vec3>) {
        self.base_push_n = force_world_n.filter(|force| force.is_finite());
    }

    /// Sets every joint to `positions_rad`, in target order, with the robot
    /// at rest and its base where it is, as
    /// [`UrdfSceneSim::set_joint_positions`]; to start an episode in a pose.
    pub fn set_joint_positions_rad(
        &mut self,
        positions_rad: &[f64],
    ) -> Result<(), PolicyWalkError> {
        if positions_rad.len() != self.joints.len() {
            return Err(PolicyWalkError::Model(format!(
                "{} positions for {} joints",
                positions_rad.len(),
                self.joints.len()
            )));
        }
        let names = &self.config.link_names;
        let positions: Vec<(&str, f64)> = names
            .iter()
            .map(String::as_str)
            .zip(positions_rad.iter().copied())
            .collect();
        if self.sim.set_joint_positions(&positions) {
            Ok(())
        } else {
            Err(PolicyWalkError::Model(
                "joint positions must be finite and on articulated joints".into(),
            ))
        }
    }

    /// Replaces the PD gains, in target order, until [`Self::reset`] restores
    /// the configured ones; for example stiffer gains while a robot settles.
    pub fn set_gains(
        &mut self,
        kp_nm_per_rad: &[f64],
        kd_nm_s_per_rad: &[f64],
    ) -> Result<(), PolicyWalkError> {
        let joints = self.joints.len();
        if kp_nm_per_rad.len() != joints || kd_nm_s_per_rad.len() != joints {
            return Err(PolicyWalkError::Model(format!(
                "gains for {} and {} joints, the walker has {joints}",
                kp_nm_per_rad.len(),
                kd_nm_s_per_rad.len()
            )));
        }
        if kp_nm_per_rad
            .iter()
            .chain(kd_nm_s_per_rad)
            .any(|gain| !gain.is_finite())
        {
            return Err(PolicyWalkError::Model("gains must be finite".into()));
        }
        self.kp_nm_per_rad.copy_from_slice(kp_nm_per_rad);
        self.kd_nm_s_per_rad.copy_from_slice(kd_nm_s_per_rad);
        Ok(())
    }

    /// Joint targets the next [`Self::step`] tracks, in radians.
    pub fn set_targets_rad(&mut self, targets_rad: &[f64]) -> Result<(), PolicyWalkError> {
        if targets_rad.len() != self.targets_rad.len() {
            return Err(PolicyWalkError::Model(format!(
                "{} targets for {} joints",
                targets_rad.len(),
                self.targets_rad.len()
            )));
        }
        if targets_rad.iter().any(|target| !target.is_finite()) {
            return Err(PolicyWalkError::Model("targets must be finite".into()));
        }
        self.targets_rad.copy_from_slice(targets_rad);
        Ok(())
    }

    /// Runs one control period of PD-controlled physics steps; returns how
    /// many joints hit their effort limit in its last step.
    pub fn step(&mut self) -> Result<usize, PolicyWalkError> {
        step_pd(
            &mut self.sim,
            &self.joints,
            &self.substeps,
            &PdPeriod {
                targets_rad: &self.targets_rad,
                kp_nm_per_rad: &self.kp_nm_per_rad,
                kd_nm_s_per_rad: &self.kd_nm_s_per_rad,
                base_push_n: self.base_push_n,
            },
            &mut self.stepped,
        )
    }

    /// Joint positions (rad) and velocities (rad/s), in target order.
    pub fn joint_states(&self) -> Result<Vec<(f64, f64)>, PolicyWalkError> {
        self.joints
            .iter()
            .map(|joint| joint_state(&self.sim, self.stepped, joint.link))
            .collect()
    }

    /// Base position and orientation in the Z-up world.
    pub fn base_pose_z_up(&self) -> (Vec3, Quat) {
        let base = world_transform_of(&self.sim.world, self.sim.base_link);
        (z_up(base.translation), y_up_to_z_up() * base.rotation)
    }

    /// Base linear and angular velocity in the Z-up world.
    pub fn base_velocity_z_up(&self) -> (Vec3, Vec3) {
        self.sim.world.get::<RigidBody>(self.sim.base_link).map_or(
            (Vec3::ZERO, Vec3::ZERO),
            |body| {
                (
                    z_up(body.linear_velocity_m_s),
                    z_up(body.angular_velocity_rad_s),
                )
            },
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LINKS: [&str; 12] = [
        "FL_hip", "FL_thigh", "FL_calf", "FR_hip", "FR_thigh", "FR_calf", "RL_hip", "RL_thigh",
        "RL_calf", "RR_hip", "RR_thigh", "RR_calf",
    ];
    const STAND_RAD: [f64; 12] = [
        0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0, -1.5,
    ];

    fn go2() -> PdJointWalker {
        go2_with_solver_iterations(4)
    }

    fn go2_with_solver_iterations(solver_iterations: usize) -> PdJointWalker {
        let sim = UrdfSceneSim::from_scene_path_with_solver_iterations(
            &UrdfSceneSim::unitree_go2_fused_scene_path(),
            solver_iterations,
        )
        .expect("Go2 scene");
        let config = PdJointConfig {
            link_names: LINKS.iter().map(|name| name.to_string()).collect(),
            kp_nm_per_rad: vec![20.0; 12],
            kd_nm_s_per_rad: vec![0.5; 12],
            default_joint_rad: STAND_RAD.to_vec(),
            control_period: SimDuration::from_ticks(20_000_000),
            decimation: 4,
            plant: PolicyWalkPlant {
                armature_kg_m2: 0.0,
                joint_damping_nm_s_per_rad: 0.0,
                joint_friction_nm: 0.0,
                ..PolicyWalkPlant::default()
            },
        };
        PdJointWalker::new(sim, config).expect("walker")
    }

    #[test]
    fn joint_positions_are_set_at_rest_without_moving_the_base() {
        let mut walker = go2();
        let before = walker.base_pose_z_up();
        walker.set_joint_positions_rad(&STAND_RAD).unwrap();
        for ((position, velocity), target) in walker.joint_states().unwrap().iter().zip(STAND_RAD) {
            assert!((position - target).abs() < 1e-5, "{position} {target}");
            assert_eq!(*velocity, 0.0);
        }
        let after = walker.base_pose_z_up();
        assert!((after.0 - before.0).length() < 1e-6);
        assert!(after.1.angle_between(before.1) < 1e-6);
        // Wrong lengths and non-finite values change nothing.
        assert!(walker.set_joint_positions_rad(&STAND_RAD[..11]).is_err());
        let mut bad = STAND_RAD;
        bad[0] = f64::NAN;
        assert!(walker.set_joint_positions_rad(&bad).is_err());
        assert!((walker.joint_states().unwrap()[0].0 - STAND_RAD[0]).abs() < 1e-5);
    }

    #[test]
    fn a_posed_go2_stands_on_its_pd_targets_and_a_reset_replays_it() {
        let run = |walker: &mut PdJointWalker| {
            walker.set_joint_positions_rad(&STAND_RAD).unwrap();
            for _ in 0..50 {
                walker.step().unwrap();
            }
            (walker.base_pose_z_up(), walker.joint_states().unwrap())
        };
        let mut walker = go2();
        let ((position, rotation), joints) = run(&mut walker);
        // Upright, on its feet, sagging under unitree_rl_gym's soft gains. It
        // is still settling: the foot spheres roll as the knees fold.
        assert!(position.z > 0.15 && position.z < 0.33, "{position}");
        assert!((rotation * Vec3::Z).z > 0.98);
        assert!(joints.iter().all(|(_, velocity)| velocity.abs() < 3.0));
        walker.reset().unwrap();
        assert_eq!(run(&mut walker), ((position, rotation), joints));
    }

    #[test]
    fn inner_solver_passes_stand_the_go2_on_one_solver_iteration_and_survive_a_reset() {
        let run = |walker: &mut PdJointWalker| {
            walker.set_joint_positions_rad(&STAND_RAD).unwrap();
            for _ in 0..50 {
                walker.step().unwrap();
            }
            walker.base_pose_z_up()
        };
        let mut walker = go2_with_solver_iterations(1);
        let single = run(&mut walker);
        assert!(!walker.set_inner_solver_passes(0));
        assert!(walker.set_inner_solver_passes(4));
        walker.reset().unwrap();
        let (position, rotation) = run(&mut walker);
        assert!(position.z > 0.15 && position.z < 0.33, "{position}");
        assert!((rotation * Vec3::Z).z > 0.99);
        // The extra passes change the solve, and a second reset keeps them.
        assert_ne!((position, rotation), single);
        walker.reset().unwrap();
        assert_eq!(run(&mut walker), (position, rotation));
    }

    #[test]
    fn mismatched_configuration_is_rejected() {
        let sim =
            UrdfSceneSim::from_scene_path(&UrdfSceneSim::unitree_go2_fused_scene_path()).unwrap();
        let mut config = go2().config().clone();
        config.kp_nm_per_rad.pop();
        assert!(PdJointWalker::new(sim, config).is_err());
        let mut walker = go2();
        assert!(walker.set_targets_rad(&[0.0; 3]).is_err());
        assert!(walker.set_gains(&[1.0; 12], &[f64::INFINITY; 12]).is_err());
    }
}
