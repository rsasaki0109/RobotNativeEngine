//! Quadruped model, leg geometry, inverse kinematics, and base state.

use rne_dynamics::{center_of_mass, ArticulatedModel, ContactPoint, DynamicsError};
use rne_ecs::{Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{height_field_surface, ColliderShape, PhysicsError};
use rne_robot::{FloatingBase, KinematicsError, Robot};
use rne_urdf_import::{parse_urdf_document, spawn_urdf_document_with_config, UrdfSpawnConfig};
use std::collections::HashMap;
use thiserror::Error;

/// Gravity in the Z-up locomotion frame, in meters per second squared.
pub(crate) const GRAVITY_M_S2: Vec3 = Vec3::new(0.0, 0.0, -9.81);

/// Vendored Unitree Go2 description.
const GO2_URDF: &str =
    include_str!("../../../assets/robots/go2_description/go2_description.rne.urdf");

/// Converts a point or direction from RNE's Y-up world into the Z-up
/// locomotion frame (a +90° rotation about X: `(x, y, z) → (x, -z, y)`).
pub fn locomotion_from_world(world: Vec3) -> Vec3 {
    Vec3::new(world.x, -world.z, world.y)
}

/// Converts a point or direction from the Z-up locomotion frame back into
/// RNE's Y-up world (`(x, y, z) → (x, z, -y)`).
pub fn world_from_locomotion(locomotion: Vec3) -> Vec3 {
    Vec3::new(locomotion.x, locomotion.z, -locomotion.y)
}

/// Errors raised while building or simulating a quadruped.
#[derive(Debug, Error)]
pub enum LocomotionError {
    /// The URDF could not be parsed or spawned.
    #[error("URDF: {0}")]
    Urdf(String),
    /// The articulated model or a dynamics step failed.
    #[error(transparent)]
    Dynamics(#[from] DynamicsError),
    /// Forward kinematics failed.
    #[error(transparent)]
    Kinematics(#[from] KinematicsError),
    /// The terrain could not be generated.
    #[error(transparent)]
    Physics(#[from] PhysicsError),
    /// A joint the leg layout expects is missing or not actuated.
    #[error("missing actuated joint `{0}`")]
    MissingJoint(String),
    /// A link the leg layout expects is missing.
    #[error("missing link `{0}`")]
    MissingLink(String),
    /// The model does not have a floating base.
    #[error("the quadruped base must be floating")]
    FixedBase,
    /// A foot or the base left the terrain's footprint.
    #[error("the robot left the terrain at ({x_m:.3}, {y_m:.3})")]
    OffTerrain {
        /// Locomotion-frame X of the point outside the terrain, in meters.
        x_m: f64,
        /// Locomotion-frame Y of the point outside the terrain, in meters.
        y_m: f64,
    },
    /// A command or configuration value was out of range.
    #[error("invalid input: {0}")]
    InvalidInput(&'static str),
}

/// Description of a quadruped that follows the Unitree URDF convention.
///
/// Each leg `L` (for example `FL`) has the joints `L_hip_joint` (abduction
/// about the body X axis), `L_thigh_joint` and `L_calf_joint` (flexion about
/// the body Y axis), and the links `L_hip`, `L_thigh`, `L_calf`, and `L_foot`.
/// At zero joint angles every leg must hang straight down along the base's
/// −Z axis, which is how [`Quadruped::new`] measures the leg geometry.
#[derive(Clone, Debug, PartialEq)]
pub struct QuadrupedSpec {
    /// URDF text. The base frame must be X forward, Y left, Z up.
    pub urdf: String,
    /// Leg prefixes in front-left, front-right, rear-left, rear-right order.
    pub legs: [String; 4],
    /// Radius of each foot's contact sphere, in meters.
    pub foot_radius_m: f64,
    /// Center of the foot sphere in the foot link frame, in meters.
    pub foot_center_local_m: Vec3,
    /// Standing hip, thigh, and calf angles, in radians.
    pub stand_pose_rad: [f64; 3],
}

impl QuadrupedSpec {
    /// The Unitree Go2 from the vendored description.
    pub fn go2() -> Self {
        Self {
            urdf: GO2_URDF.to_owned(),
            legs: ["FL", "FR", "RL", "RR"].map(str::to_owned),
            foot_radius_m: 0.022,
            foot_center_local_m: Vec3::new(-0.002, 0.0, 0.0),
            stand_pose_rad: [0.0, 0.8, -1.5],
        }
    }
}

/// Geometry and joint indices of one leg.
///
/// Positions are in the base frame (X forward, Y left, Z up) with all joints at
/// zero. Joint indices count actuated coordinates, after the floating base.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LegGeometry {
    /// Actuated indices of the hip, thigh, and calf joints.
    pub joints: [usize; 3],
    /// Foot link entity.
    pub foot_link: Entity,
    /// Foot link index in the kinematic model.
    pub foot_index: usize,
    /// Hip abduction joint origin in the base frame, in meters.
    pub hip_offset_m: Vec3,
    /// Lateral offset of the thigh joint from the hip joint, in meters
    /// (positive for left legs).
    pub thigh_offset_y_m: f64,
    /// Thigh-joint-to-calf-joint length, in meters.
    pub thigh_length_m: f64,
    /// Calf-joint-to-foot length, in meters.
    pub calf_length_m: f64,
    /// Foot link origin in the base frame at the stand pose, in meters.
    pub stand_foot_m: Vec3,
}

/// A floating-base quadruped ready to simulate in the Z-up locomotion frame.
#[derive(Clone, Debug)]
pub struct Quadruped {
    model: ArticulatedModel,
    legs: [LegGeometry; 4],
    foot_radius_m: f64,
    foot_center_local_m: Vec3,
    stand_pose_rad: Vec<f64>,
    effort_limits: Vec<f64>,
    position_limits: Vec<(f64, f64)>,
    mass_kg: f64,
    stand_com_m: Vec3,
}

impl Quadruped {
    /// Builds a quadruped from `spec`.
    ///
    /// The URDF is spawned into a private ECS world with its base floating and
    /// declared inertial masses, and modeled with gravity along −Z.
    pub fn new(spec: &QuadrupedSpec) -> Result<Self, LocomotionError> {
        if !(spec.foot_radius_m.is_finite()
            && spec.foot_radius_m > 0.0
            && spec.foot_center_local_m.is_finite())
            || spec.stand_pose_rad.iter().any(|angle| !angle.is_finite())
        {
            return Err(LocomotionError::InvalidInput("quadruped spec"));
        }
        let document = parse_urdf_document(&spec.urdf)
            .map_err(|error| LocomotionError::Urdf(error.to_string()))?;
        let mut world = World::new();
        let config = UrdfSpawnConfig {
            attach_colliders: false,
            attach_mesh_colliders: false,
            self_collisions: false,
            use_declared_inertial_masses: true,
            ..UrdfSpawnConfig::default()
        };
        let spawned = spawn_urdf_document_with_config(&mut world, &document, config)
            .map_err(|error| LocomotionError::Urdf(error.to_string()))?;
        let base_link = world
            .get::<Robot>(spawned.robot)
            .ok_or(LocomotionError::Urdf(
                "spawned robot has no Robot".to_owned(),
            ))?
            .base_link;
        world.entity_mut(base_link).insert(FloatingBase);
        let model = ArticulatedModel::from_robot_with_gravity(&world, spawned.robot, GRAVITY_M_S2)?;
        if model.base_dof() != 6 {
            return Err(LocomotionError::FixedBase);
        }
        let base = model.base_dof();
        let actuated = model.nv() - base;

        let zero = model
            .kinematic()
            .forward_kinematics(&vec![0.0; model.nv()])?;
        let link_position = |name: &str| -> Result<(Entity, usize, Vec3), LocomotionError> {
            let entity = model
                .kinematic()
                .link_entity_by_name(name)
                .ok_or_else(|| LocomotionError::MissingLink(name.to_owned()))?;
            let index = model
                .kinematic()
                .link_index(entity)
                .ok_or_else(|| LocomotionError::MissingLink(name.to_owned()))?;
            Ok((entity, index, zero.transforms()[index].translation))
        };
        let joint_index = |joints: &HashMap<String, Entity>, name: &str| {
            joints
                .get(name)
                .and_then(|entity| model.kinematic().dof_index_of_joint(*entity))
                .ok_or_else(|| LocomotionError::MissingJoint(name.to_owned()))
        };

        let mut stand_pose_rad = vec![0.0; actuated];
        let mut legs = Vec::with_capacity(4);
        for leg in &spec.legs {
            let joints = [
                joint_index(&spawned.joints, &format!("{leg}_hip_joint"))?,
                joint_index(&spawned.joints, &format!("{leg}_thigh_joint"))?,
                joint_index(&spawned.joints, &format!("{leg}_calf_joint"))?,
            ];
            for (joint, angle) in joints.iter().zip(spec.stand_pose_rad) {
                stand_pose_rad[*joint] = angle;
            }
            let (_, _, hip) = link_position(&format!("{leg}_hip"))?;
            let (_, _, thigh) = link_position(&format!("{leg}_thigh"))?;
            let (_, _, calf) = link_position(&format!("{leg}_calf"))?;
            let (foot_link, foot_index, foot) = link_position(&format!("{leg}_foot"))?;
            legs.push(LegGeometry {
                joints,
                foot_link,
                foot_index,
                hip_offset_m: hip,
                thigh_offset_y_m: thigh.y - hip.y,
                thigh_length_m: (calf - thigh).length(),
                calf_length_m: (foot - calf).length(),
                stand_foot_m: Vec3::ZERO,
            });
        }
        let mut q = vec![0.0; model.nv()];
        q[base..].copy_from_slice(&stand_pose_rad);
        let stand = model.kinematic().forward_kinematics(&q)?;
        let stand_com_m = center_of_mass(&model, &q)?;
        for leg in &mut legs {
            leg.stand_foot_m = stand.transforms()[leg.foot_index].translation;
        }
        let effort_limits = (0..actuated)
            .map(|joint| {
                model
                    .joint_effort_limit(base + joint)
                    .unwrap_or(f64::INFINITY)
            })
            .collect();
        let position_limits = (0..actuated)
            .map(|joint| {
                model
                    .joint_position_limits(base + joint)
                    .unwrap_or((f64::NEG_INFINITY, f64::INFINITY))
            })
            .collect();
        let legs = <[LegGeometry; 4]>::try_from(legs)
            .map_err(|_| LocomotionError::InvalidInput("a quadruped has four legs"))?;
        let mass_kg = (0..model.link_count())
            .filter_map(|index| model.link_inertia(index))
            .map(|inertia| inertia.mass_kg)
            .sum();
        Ok(Self {
            model,
            legs,
            foot_radius_m: spec.foot_radius_m,
            foot_center_local_m: spec.foot_center_local_m,
            stand_pose_rad,
            effort_limits,
            position_limits,
            mass_kg,
            stand_com_m,
        })
    }

    /// The Unitree Go2.
    pub fn go2() -> Result<Self, LocomotionError> {
        Self::new(&QuadrupedSpec::go2())
    }

    /// Articulated model in the Z-up locomotion frame.
    pub fn model(&self) -> &ArticulatedModel {
        &self.model
    }

    /// Leg geometry in front-left, front-right, rear-left, rear-right order.
    pub fn legs(&self) -> &[LegGeometry; 4] {
        &self.legs
    }

    /// Number of actuated joints.
    pub fn actuated_count(&self) -> usize {
        self.stand_pose_rad.len()
    }

    /// Standing joint angles, in actuated order, in radians.
    pub fn stand_pose_rad(&self) -> &[f64] {
        &self.stand_pose_rad
    }

    /// Effort limit of each actuated joint, in N·m (infinite when undeclared).
    pub fn effort_limits(&self) -> &[f64] {
        &self.effort_limits
    }

    /// Foot sphere radius, in meters.
    pub fn foot_radius_m(&self) -> f64 {
        self.foot_radius_m
    }

    /// Total mass, in kilograms.
    pub fn mass_kg(&self) -> f64 {
        self.mass_kg
    }

    /// Center of mass in the base frame at the stand pose, in meters.
    pub fn stand_com_m(&self) -> Vec3 {
        self.stand_com_m
    }

    /// Foot link origin of leg `leg` in the base frame for hip, thigh, and
    /// calf `angles_rad`, in meters.
    pub fn leg_forward_kinematics(&self, leg: usize, angles_rad: [f64; 3]) -> Vec3 {
        let geometry = &self.legs[leg];
        let [hip, thigh, calf] = angles_rad;
        let (l1, l2) = (geometry.thigh_length_m, geometry.calf_length_m);
        let x = -l1 * thigh.sin() - l2 * (thigh + calf).sin();
        let z_plane = -l1 * thigh.cos() - l2 * (thigh + calf).cos();
        let offset = geometry.thigh_offset_y_m;
        geometry.hip_offset_m
            + Vec3::new(
                x,
                offset * hip.cos() - z_plane * hip.sin(),
                offset * hip.sin() + z_plane * hip.cos(),
            )
    }

    /// Actuated torques of leg `leg` that hold `force_base_n` — the force the
    /// ground exerts on the foot, in the base frame — at `angles_rad`
    /// (`-Jᵀ f`), in hip, thigh, calf order.
    pub fn leg_force_torques(
        &self,
        leg: usize,
        angles_rad: [f64; 3],
        force_base_n: Vec3,
    ) -> [f64; 3] {
        const STEP_RAD: f64 = 1.0e-6;
        let mut torques = [0.0; 3];
        for (joint, torque) in torques.iter_mut().enumerate() {
            let mut plus = angles_rad;
            let mut minus = angles_rad;
            plus[joint] += STEP_RAD;
            minus[joint] -= STEP_RAD;
            let column = (self.leg_forward_kinematics(leg, plus)
                - self.leg_forward_kinematics(leg, minus))
                / (2.0 * STEP_RAD);
            *torque = -column.dot(force_base_n);
        }
        torques
    }

    /// Joint angles that put leg `leg`'s foot link origin at `foot_base_m`
    /// (base frame, meters), clamped to the joint position limits.
    ///
    /// Out-of-reach targets are projected onto the leg's workspace. The knee
    /// bends backward, as on Unitree quadrupeds.
    pub fn leg_inverse_kinematics(&self, leg: usize, foot_base_m: Vec3) -> [f64; 3] {
        let geometry = &self.legs[leg];
        let relative = foot_base_m - geometry.hip_offset_m;
        let offset = geometry.thigh_offset_y_m;
        // Abduction rotates the leg plane about X; in that plane the foot sits
        // at (x, offset, z_plane) with z_plane below the hip.
        let radius_sq = relative.y * relative.y + relative.z * relative.z;
        let z_plane = -(radius_sq - offset * offset).max(1.0e-6).sqrt();
        let hip = relative.z.atan2(relative.y) - z_plane.atan2(offset);
        let (thigh_length, calf_length) = (geometry.thigh_length_m, geometry.calf_length_m);
        let reach_sq = relative.x * relative.x + z_plane * z_plane;
        let cos_knee = ((reach_sq - thigh_length * thigh_length - calf_length * calf_length)
            / (2.0 * thigh_length * calf_length))
            .clamp(-1.0, 1.0);
        let calf = -cos_knee.acos();
        let thigh = (-relative.x).atan2(-z_plane)
            - (calf_length * calf.sin()).atan2(thigh_length + calf_length * calf.cos());
        let mut angles = [hip, thigh, calf];
        for (angle, joint) in angles.iter_mut().zip(geometry.joints) {
            let (lower, upper) = self.position_limits[joint];
            *angle = angle.clamp(lower, upper);
        }
        angles
    }

    /// Base state at configuration `q` and velocity `qd`.
    pub fn base_state(&self, q: &[f64], qd: &[f64]) -> BaseState {
        let rotation =
            Quat::from_rotation_z(q[5]) * Quat::from_rotation_y(q[4]) * Quat::from_rotation_x(q[3]);
        BaseState {
            position_m: Vec3::new(q[0], q[1], q[2]),
            rotation,
            linear_velocity_m_s: rotation * Vec3::new(qd[0], qd[1], qd[2]),
            angular_velocity_rad_s: rotation * Vec3::new(qd[3], qd[4], qd[5]),
        }
    }

    /// One sphere contact per foot against the Y-up `terrain` beneath it, at
    /// configuration `q`, in leg order.
    pub fn foot_contacts(
        &self,
        terrain: &ColliderShape,
        q: &[f64],
        friction_coefficient: f64,
    ) -> Result<Vec<ContactPoint>, LocomotionError> {
        let kinematics = self.model.kinematic().forward_kinematics(q)?;
        self.legs
            .iter()
            .map(|leg| {
                let foot = kinematics.transforms()[leg.foot_index];
                let center = foot.translation + foot.rotation * self.foot_center_local_m;
                let (height, normal) = terrain_surface(terrain, center.x, center.y)?;
                let clearance = (center.z - height) * normal.z;
                let point = center - normal * self.foot_radius_m;
                Ok(ContactPoint {
                    link: leg.foot_link,
                    point_local_m: foot.rotation.conjugate() * (point - foot.translation),
                    normal_world: normal,
                    gap_m: clearance - self.foot_radius_m,
                    friction_coefficient,
                })
            })
            .collect()
    }
}

/// Terrain height and unit normal at locomotion-frame `(x, y)`.
pub(crate) fn terrain_surface(
    terrain: &ColliderShape,
    x_m: f64,
    y_m: f64,
) -> Result<(f64, Vec3), LocomotionError> {
    let world = world_from_locomotion(Vec3::new(x_m, y_m, 0.0));
    let surface = height_field_surface(terrain, world.x, world.z)
        .ok_or(LocomotionError::OffTerrain { x_m, y_m })?;
    Ok((surface.height_m, locomotion_from_world(surface.normal)))
}

/// Pose and velocity of the floating base in the Z-up locomotion frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BaseState {
    /// Base link origin, in meters.
    pub position_m: Vec3,
    /// Base link orientation (base frame to locomotion frame).
    pub rotation: Quat,
    /// Base linear velocity in the locomotion frame, in meters per second.
    pub linear_velocity_m_s: Vec3,
    /// Base angular velocity in the locomotion frame, in radians per second.
    pub angular_velocity_rad_s: Vec3,
}

impl BaseState {
    /// Heading of the base's forward (X) axis about +Z, in radians.
    pub fn heading_rad(&self) -> f64 {
        let forward = self.rotation * Vec3::X;
        forward.y.atan2(forward.x)
    }

    /// Orientation of the base relative to its heading-only (level) frame,
    /// as the quaternion with a non-negative scalar part (the short way
    /// round, so its scaled axis is the tilt itself).
    pub fn tilt(&self) -> Quat {
        let tilt = Quat::from_rotation_z(-self.heading_rad()) * self.rotation;
        if tilt.w < 0.0 {
            -tilt
        } else {
            tilt
        }
    }

    /// Angle between the base's up axis and the vertical, in radians.
    pub fn tilt_rad(&self) -> f64 {
        (self.rotation * Vec3::Z).z.clamp(-1.0, 1.0).acos()
    }

    /// Unit gravity direction in the base frame.
    pub fn projected_gravity(&self) -> Vec3 {
        self.rotation.conjugate() * Vec3::NEG_Z
    }

    /// Linear velocity in the level frame (X along the heading, Z up).
    pub fn level_linear_velocity_m_s(&self) -> Vec3 {
        Quat::from_rotation_z(-self.heading_rad()) * self.linear_velocity_m_s
    }

    /// Linear velocity in the base frame, in meters per second.
    pub fn body_linear_velocity_m_s(&self) -> Vec3 {
        self.rotation.conjugate() * self.linear_velocity_m_s
    }

    /// Angular velocity in the base frame, in radians per second.
    pub fn body_angular_velocity_rad_s(&self) -> Vec3 {
        self.rotation.conjugate() * self.angular_velocity_rad_s
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_dynamics::integrate_configuration;

    #[test]
    fn frame_conversions_are_inverse_rotations() {
        let point = Vec3::new(0.3, -1.2, 2.5);
        assert_eq!(world_from_locomotion(locomotion_from_world(point)), point);
        // The locomotion frame is the world rotated +90° about X.
        let rotated = Quat::from_rotation_x(std::f64::consts::FRAC_PI_2) * point;
        assert!((locomotion_from_world(point) - rotated).length() < 1.0e-12);
        assert_eq!(locomotion_from_world(Vec3::Y), Vec3::Z);
    }

    #[test]
    fn go2_leg_geometry_matches_its_urdf() {
        let go2 = Quadruped::go2().expect("go2");
        assert_eq!(go2.actuated_count(), 12);
        let front_left = go2.legs()[0];
        assert!((front_left.hip_offset_m - Vec3::new(0.1934, 0.0465, 0.0)).length() < 1.0e-9);
        assert!((front_left.thigh_offset_y_m - 0.0955).abs() < 1.0e-9);
        assert!((go2.legs()[1].thigh_offset_y_m + 0.0955).abs() < 1.0e-9);
        assert!((front_left.thigh_length_m - 0.213).abs() < 1.0e-9);
        assert!((front_left.calf_length_m - 0.213).abs() < 1.0e-9);
        assert!(front_left.stand_foot_m.z < -0.25);
    }

    #[test]
    fn leg_inverse_kinematics_inverts_forward_kinematics() {
        let go2 = Quadruped::go2().expect("go2");
        let model = go2.model();
        let base = model.base_dof();
        let poses = [
            [0.0, 0.8, -1.5],
            [0.2, 0.5, -1.2],
            [-0.25, 1.1, -2.0],
            [0.1, 0.3, -0.9],
        ];
        for (leg_index, leg) in go2.legs().iter().enumerate() {
            for pose in poses {
                let mut q = vec![0.0; model.nv()];
                for (joint, angle) in leg.joints.iter().zip(pose) {
                    q[base + joint] = angle;
                }
                let kinematics = model.kinematic().forward_kinematics(&q).expect("fk");
                let foot = kinematics.transforms()[leg.foot_index].translation;
                let solved = go2.leg_inverse_kinematics(leg_index, foot);
                for (angle, expected) in solved.iter().zip(pose) {
                    assert!(
                        (angle - expected).abs() < 1.0e-9,
                        "leg {leg_index}: {solved:?} vs {pose:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn leg_forward_kinematics_matches_the_model() {
        let go2 = Quadruped::go2().expect("go2");
        let model = go2.model();
        let base = model.base_dof();
        for (leg_index, leg) in go2.legs().iter().enumerate() {
            let angles = [0.3, 0.6, -1.7];
            let mut q = vec![0.0; model.nv()];
            for (joint, angle) in leg.joints.iter().zip(angles) {
                q[base + joint] = angle;
            }
            let kinematics = model.kinematic().forward_kinematics(&q).expect("fk");
            let foot = kinematics.transforms()[leg.foot_index].translation;
            assert!((go2.leg_forward_kinematics(leg_index, angles) - foot).length() < 1.0e-12);
        }
        // A vertical ground force on the left foot twists the hip by the
        // thigh offset and folds the knee, which the calf resists.
        let torques = go2.leg_force_torques(0, [0.0, 0.8, -1.5], Vec3::new(0.0, 0.0, 40.0));
        assert!((torques[0] + 0.0955 * 40.0).abs() < 1.0e-6, "{torques:?}");
        let knee = 0.213 * (0.8_f64 - 1.5).sin();
        assert!((torques[2] + knee * 40.0).abs() < 1.0e-6, "{torques:?}");
    }

    #[test]
    fn tilt_is_continuous_across_the_heading_wrap() {
        let go2 = Quadruped::go2().expect("go2");
        let qd = vec![0.0; go2.model().nv()];
        let mut previous: Option<Vec3> = None;
        for offset in [-0.02, -0.005, 0.0, 0.005, 0.02] {
            let heading = std::f64::consts::PI + offset;
            let mut q = vec![0.0; go2.model().nv()];
            q[3] = 0.05;
            q[4] = -0.02;
            q[5] = heading;
            let tilt = go2.base_state(&q, &qd).tilt().to_scaled_axis();
            assert!(tilt.length() < 0.06, "{tilt}");
            if let Some(previous) = previous {
                assert!((tilt - previous).length() < 0.01);
            }
            previous = Some(tilt);
        }
    }

    #[test]
    fn base_state_matches_integrated_motion() {
        let go2 = Quadruped::go2().expect("go2");
        let model = go2.model();
        let mut q = vec![0.0; model.nv()];
        q[..6].copy_from_slice(&[0.1, -0.2, 0.3, 0.05, -0.1, 2.5]);
        let mut qd = vec![0.0; model.nv()];
        qd[..6].copy_from_slice(&[0.4, -0.2, 0.1, 0.3, -0.2, 0.5]);
        let state = go2.base_state(&q, &qd);
        assert!((state.heading_rad() - 2.5).abs() < 0.2);
        let dt = 1.0e-6;
        let next = go2.base_state(
            &integrate_configuration(model, &q, &qd, dt).expect("integrate"),
            &qd,
        );
        let velocity = (next.position_m - state.position_m) / dt;
        assert!((velocity - state.linear_velocity_m_s).length() < 1.0e-5);
        let delta = next.rotation * state.rotation.conjugate();
        let angular = delta.to_scaled_axis() / dt;
        assert!((angular - state.angular_velocity_rad_s).length() < 1.0e-5);
    }
}
