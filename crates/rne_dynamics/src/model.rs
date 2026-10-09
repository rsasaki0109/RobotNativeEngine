//! Articulated tree model derived from the robot link/joint graph.

use crate::spatial::{inverse_transform, mat6_add, mat6_mul, motion_transform, Mat6, SpatialVec};
use crate::SpatialInertia;
use rne_ecs::{Entity, World};
use rne_math::Quat;
use rne_math::Vec3;
use rne_physics::{RigidBody, RigidBodyInertia};
use rne_robot::components::Inertial;
use rne_robot::{Joint, JointKind, KinematicModel, KinematicsError};
use rne_world::Transform3;
use std::collections::HashMap;
use thiserror::Error;

/// Error returned while building or evaluating an articulated model.
#[derive(Clone, Debug, PartialEq, Eq, Error)]
pub enum DynamicsError {
    /// The underlying kinematic model could not be derived.
    #[error(transparent)]
    Kinematics(#[from] KinematicsError),
    /// The supplied configuration or velocity vector has the wrong length.
    #[error("expected {expected} values but received {provided}")]
    DimensionMismatch {
        /// Number of values supplied.
        provided: usize,
        /// Number of values the model requires.
        expected: usize,
    },
    /// A configuration, velocity, acceleration, or torque vector was not finite.
    #[error("dynamics input contains a non-finite value")]
    NonFiniteInput,
    /// Gravity was not a finite vector.
    #[error("gravity must be a finite vector")]
    InvalidGravity,
    /// The mass matrix could not be inverted because it is singular.
    #[error("mass matrix is singular and cannot be inverted")]
    SingularMassMatrix,
    /// An internal invariant was violated: a degree of freedom had no owner.
    #[error("internal error: degree of freedom {0} has no owner link")]
    MissingDofOwner(usize),
    /// A contact, contact-solver, or step parameter was out of range.
    #[error("invalid contact input: {0}")]
    InvalidContact(&'static str),
}

#[derive(Clone, Debug)]
pub(crate) struct ArticulatedLink {
    pub(crate) entity: Entity,
    pub(crate) parent: Option<usize>,
    pub(crate) inertia: SpatialInertia,
    pub(crate) joint: Option<ArticulatedJoint>,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct ArticulatedJoint {
    pub(crate) dof: Option<usize>,
}

/// A rigid body of the dynamics tree: a moving link together with every link
/// welded to it by a chain of fixed joints.
#[derive(Clone, Debug)]
pub(crate) struct DynamicsBody {
    /// Kinematic link index whose frame is the body frame.
    pub(crate) link: usize,
    /// Parent body index.
    pub(crate) parent: Option<usize>,
    /// Spatial inertia of the link and its welded links, in the body frame.
    pub(crate) inertia: Mat6,
    /// Velocity coordinates of the joint into this body (all base coordinates
    /// for the root).
    pub(crate) dofs: Vec<usize>,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct DofSpec {
    pub(crate) link: usize,
    /// Dynamics body that owns this coordinate.
    pub(crate) body: usize,
    pub(crate) s: SpatialVec,
    /// Position limits `(lower, upper)` of a bounded revolute or prismatic joint.
    pub(crate) limits: Option<(f64, f64)>,
    /// Actuator effort bound (N·m or N) of the joint, when finite.
    pub(crate) max_effort: Option<f64>,
}

/// Prescribed motion of a fixed base, in the model's world frame: the
/// velocity and acceleration of the base frame's origin and its angular
/// velocity and acceleration. A base on a moving support uses it so the
/// support's motion drives the tree, as gravity does.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BaseMotion {
    /// Velocity of the base frame's origin, in m/s.
    pub linear_velocity_m_s: Vec3,
    /// Angular velocity of the base, in rad/s.
    pub angular_velocity_rad_s: Vec3,
    /// Acceleration of the base frame's origin, in m/s².
    pub linear_acceleration_m_s2: Vec3,
    /// Angular acceleration of the base, in rad/s².
    pub angular_acceleration_rad_s2: Vec3,
}

impl BaseMotion {
    /// The base's spatial velocity and spatial acceleration (linear part
    /// first) in its own frame, for a base turned by `rotation`. The spatial
    /// acceleration is the time derivative of the body-frame velocity, so its
    /// linear part is the origin's acceleration less `ω × v`.
    pub(crate) fn body_frame(&self, rotation: Quat) -> (SpatialVec, SpatialVec) {
        let inverse = rotation.conjugate();
        let velocity = inverse * self.linear_velocity_m_s;
        let angular = inverse * self.angular_velocity_rad_s;
        let acceleration = inverse * self.linear_acceleration_m_s2 - angular.cross(velocity);
        let angular_acceleration = inverse * self.angular_acceleration_rad_s2;
        (
            [
                velocity.x, velocity.y, velocity.z, angular.x, angular.y, angular.z,
            ],
            [
                acceleration.x,
                acceleration.y,
                acceleration.z,
                angular_acceleration.x,
                angular_acceleration.y,
                angular_acceleration.z,
            ],
        )
    }
}

/// A floating- or fixed-base articulated tree with link spatial inertias.
///
/// The model is a plain value derived from the scene. It owns a
/// [`KinematicModel`] for link ordering and forward kinematics and adds the
/// inertial and joint-motion data needed by the dynamics algorithms. It never
/// depends on a physics backend, a renderer, or wall-clock time.
#[derive(Clone, Debug)]
pub struct ArticulatedModel {
    pub(crate) robot: Entity,
    pub(crate) gravity_m_s2: Vec3,
    /// Prescribed motion of a fixed base; zero unless set.
    pub(crate) base_motion: BaseMotion,
    pub(crate) base_dof: usize,
    pub(crate) nv: usize,
    pub(crate) links: Vec<ArticulatedLink>,
    pub(crate) dofs: Vec<DofSpec>,
    pub(crate) link_dofs: Vec<Vec<usize>>,
    /// Links merged across fixed joints; the recursive algorithms run on these.
    pub(crate) bodies: Vec<DynamicsBody>,
    /// Parent velocity coordinate of each coordinate in the tree (`λ(i)`), or
    /// `None` when coordinates are not numbered parents-first.
    pub(crate) dof_parents: Option<Vec<Option<usize>>>,
    pub(crate) kinematic: KinematicModel,
}

impl ArticulatedModel {
    /// Builds an articulated model with the RNE default gravity `(0, -9.81, 0)`
    /// in a Y-up world.
    pub fn from_robot(world: &World, robot: Entity) -> Result<Self, DynamicsError> {
        Self::from_robot_with_gravity(world, robot, Vec3::new(0.0, -9.81, 0.0))
    }

    /// Builds an articulated model with an explicit gravity vector, in
    /// meters per second squared.
    ///
    /// Use it to simulate a model in a frame other than RNE's Y-up world, such
    /// as a Z-up frame where a floating base's heading is its yaw coordinate,
    /// so its roll-pitch-yaw coordinates stay continuous as it turns.
    pub fn from_robot_with_gravity(
        world: &World,
        robot: Entity,
        gravity_m_s2: Vec3,
    ) -> Result<Self, DynamicsError> {
        if !gravity_m_s2.is_finite() {
            return Err(DynamicsError::InvalidGravity);
        }
        let kinematic = KinematicModel::from_robot(world, robot)?;
        let base_dof = kinematic.base_dof();
        let nv = kinematic.dof();
        let link_count = kinematic.link_count();

        // Joint entity, kind, axis, and (lower, upper, max effort) by child link.
        type JointRecord = (Entity, JointKind, Vec3, (f64, f64, f64));
        let mut joint_by_child: HashMap<Entity, JointRecord> = HashMap::new();
        for entity_ref in world.iter_entities() {
            let Some(joint) = entity_ref.get::<Joint>() else {
                continue;
            };
            if joint.robot == robot {
                joint_by_child.insert(
                    joint.child_link,
                    (
                        entity_ref.id(),
                        joint.kind,
                        joint.axis,
                        (
                            joint.limits.lower,
                            joint.limits.upper,
                            joint.limits.max_effort,
                        ),
                    ),
                );
            }
        }

        let mut dofs: Vec<Option<DofSpec>> = vec![None; nv];
        let mut link_dofs: Vec<Vec<usize>> = vec![Vec::new(); link_count];
        for (dof, slot) in dofs.iter_mut().enumerate().take(base_dof) {
            let mut s = [0.0; 6];
            s[dof] = 1.0;
            *slot = Some(DofSpec {
                link: 0,
                body: 0,
                s,
                limits: None,
                max_effort: None,
            });
            link_dofs[0].push(dof);
        }

        let mut links: Vec<ArticulatedLink> = Vec::with_capacity(link_count);
        let mut welded = vec![false; link_count];
        for (index, link_dof) in link_dofs.iter_mut().enumerate() {
            let entity = kinematic
                .link_entity(index)
                .ok_or(DynamicsError::MissingDofOwner(index))?;
            let parent = kinematic.link_parent(index);
            let inertia = link_spatial_inertia(world, entity);
            welded[index] = parent.is_some()
                && matches!(
                    joint_by_child.get(&entity),
                    Some((_, JointKind::Fixed, _, _))
                );
            let joint = joint_by_child.get(&entity).map(
                |(joint_entity, kind, axis, (lower, upper, effort))| {
                    let dof = kinematic
                        .dof_index_of_joint(*joint_entity)
                        .map(|joint_dof| base_dof + joint_dof);
                    if let Some(global_dof) = dof {
                        if let Some(s) = joint_motion_subspace(*kind, *axis) {
                            dofs[global_dof] = Some(DofSpec {
                                link: index,
                                body: 0,
                                s,
                                limits: position_limits(*kind, *lower, *upper),
                                max_effort: (effort.is_finite() && *effort > 0.0)
                                    .then_some(*effort),
                            });
                            link_dof.push(global_dof);
                        }
                    }
                    ArticulatedJoint { dof }
                },
            );
            links.push(ArticulatedLink {
                entity,
                parent,
                inertia,
                joint,
            });
        }

        let mut resolved = Vec::with_capacity(nv);
        for (index, slot) in dofs.into_iter().enumerate() {
            resolved.push(slot.ok_or(DynamicsError::MissingDofOwner(index))?);
        }
        let bodies = build_bodies(&kinematic, &links, &link_dofs, &welded, &mut resolved)?;
        let dof_parents = dof_parents(&bodies, nv);

        Ok(Self {
            robot,
            gravity_m_s2,
            base_motion: BaseMotion::default(),
            base_dof,
            nv,
            links,
            dofs: resolved,
            link_dofs,
            bodies,
            dof_parents,
            kinematic,
        })
    }

    /// Moves a fixed base to `pose`, in the model's world frame, so the tree
    /// rides on a moving support. Only the placement changes: the support's
    /// own velocity and acceleration do not enter the dynamics. Has no effect
    /// on a floating base, whose pose is its base coordinates.
    pub fn set_fixed_base_pose(&mut self, pose: Transform3) {
        if self.base_dof == 0 {
            self.kinematic.set_root_transform(pose);
        }
    }

    /// Prescribes the motion of a fixed base, in the model's world frame, for
    /// the dynamics algorithms ([`crate::rnea`], [`crate::aba()`], and the
    /// contact steps built on them): a base that accelerates or turns drives
    /// the tree through the inertial forces it causes. Contact steps also
    /// include its linear and angular point velocities in relative motion.
    /// Has no effect on a
    /// floating base.
    pub fn set_fixed_base_motion(&mut self, motion: BaseMotion) {
        if self.base_dof == 0 {
            self.base_motion = motion;
        }
    }

    /// The prescribed motion of a fixed base; zero for a floating base.
    pub fn fixed_base_motion(&self) -> BaseMotion {
        self.base_motion
    }

    /// Owning robot entity.
    pub fn robot(&self) -> Entity {
        self.robot
    }

    /// Gravity vector in the world frame, in m/s².
    pub fn gravity_m_s2(&self) -> Vec3 {
        self.gravity_m_s2
    }

    /// Number of floating-base degrees of freedom (0 or 6).
    pub fn base_dof(&self) -> usize {
        self.base_dof
    }

    /// Total number of generalized velocity coordinates.
    pub fn nv(&self) -> usize {
        self.nv
    }

    /// Number of links in topological order.
    pub fn link_count(&self) -> usize {
        self.links.len()
    }

    /// Entity of the link at topological index `index`.
    pub fn link_entity(&self, index: usize) -> Option<Entity> {
        self.links.get(index).map(|link| link.entity)
    }

    /// Topological index of a link's parent, if any.
    pub fn link_parent(&self, index: usize) -> Option<usize> {
        self.links.get(index).and_then(|link| link.parent)
    }

    /// Spatial inertia of the link at topological index `index`.
    pub fn link_inertia(&self, index: usize) -> Option<&SpatialInertia> {
        self.links.get(index).map(|link| &link.inertia)
    }

    /// Position limits `(lower, upper)` of velocity coordinate `dof`.
    ///
    /// Returns `None` for floating-base coordinates, continuous joints, and
    /// joints whose limits are not both finite with `lower <= upper`.
    pub fn joint_position_limits(&self, dof: usize) -> Option<(f64, f64)> {
        self.dofs.get(dof).and_then(|spec| spec.limits)
    }

    /// Actuator effort bound of velocity coordinate `dof`, in N·m (revolute)
    /// or N (prismatic).
    ///
    /// Returns `None` for floating-base coordinates and joints whose
    /// `max_effort` is not finite and positive.
    pub fn joint_effort_limit(&self, dof: usize) -> Option<f64> {
        self.dofs.get(dof).and_then(|spec| spec.max_effort)
    }

    /// Underlying kinematic model used for link ordering and forward kinematics.
    pub fn kinematic(&self) -> &KinematicModel {
        &self.kinematic
    }
}

/// Merges every link welded by fixed joints into its nearest moving ancestor.
///
/// Links arrive in topological order, so a welded link's parent already has a
/// body. The welded offset is read from forward kinematics at the zero
/// configuration, which is exact because a chain of fixed joints does not move.
fn build_bodies(
    kinematic: &KinematicModel,
    links: &[ArticulatedLink],
    link_dofs: &[Vec<usize>],
    welded: &[bool],
    dofs: &mut [DofSpec],
) -> Result<Vec<DynamicsBody>, DynamicsError> {
    let zero = kinematic.forward_kinematics(&vec![0.0; kinematic.dof()])?;
    let transforms = zero.transforms();
    let mut body_of = vec![0; links.len()];
    let mut bodies: Vec<DynamicsBody> = Vec::new();
    for (index, link) in links.iter().enumerate() {
        let inertia = link.inertia.matrix();
        match link.parent {
            Some(parent) if welded[index] => {
                let body = body_of[parent];
                body_of[index] = body;
                let frame = transforms[bodies[body].link];
                let link_in_body = inverse_transform(&frame).mul_transform(&transforms[index]);
                let to_link = motion_transform(&inverse_transform(&link_in_body));
                let moved = mat6_mul(&mat6_transpose(&to_link), &mat6_mul(&inertia, &to_link));
                bodies[body].inertia = mat6_add(&bodies[body].inertia, &moved);
            }
            parent => {
                body_of[index] = bodies.len();
                bodies.push(DynamicsBody {
                    link: index,
                    parent: parent.map(|parent| body_of[parent]),
                    inertia,
                    dofs: link_dofs[index].clone(),
                });
            }
        }
    }
    for spec in dofs.iter_mut() {
        spec.body = body_of[spec.link];
    }
    Ok(bodies)
}

/// Parent coordinate `λ(i)` of each velocity coordinate: the previous
/// coordinate of the same body, else the last coordinate of the nearest
/// ancestor body that has one. Returns `None` unless every `λ(i) < i`, which
/// the tree-sparse factorization of the mass matrix relies on.
fn dof_parents(bodies: &[DynamicsBody], nv: usize) -> Option<Vec<Option<usize>>> {
    let mut parents = vec![None; nv];
    let mut last_dof: Vec<Option<usize>> = vec![None; bodies.len()];
    for (index, body) in bodies.iter().enumerate() {
        let mut previous = None;
        let mut ancestor = body.parent;
        while let Some(parent) = ancestor {
            if let Some(dof) = last_dof[parent] {
                previous = Some(dof);
                break;
            }
            ancestor = bodies[parent].parent;
        }
        for &dof in &body.dofs {
            if previous.is_some_and(|parent| parent >= dof) {
                return None;
            }
            parents[dof] = previous;
            previous = Some(dof);
        }
        last_dof[index] = previous;
    }
    Some(parents)
}

fn mat6_transpose(matrix: &Mat6) -> Mat6 {
    let mut out = [[0.0; 6]; 6];
    for (row, values) in matrix.iter().enumerate() {
        for (col, value) in values.iter().enumerate() {
            out[col][row] = *value;
        }
    }
    out
}

fn link_spatial_inertia(world: &World, entity: Entity) -> SpatialInertia {
    let mass_kg = world
        .get::<RigidBody>(entity)
        .map(|body| body.mass_kg)
        .unwrap_or(0.0);
    if let Some(inertia) = world.get::<RigidBodyInertia>(entity) {
        return SpatialInertia::new(
            mass_kg,
            inertia.center_of_mass_local_m,
            [
                [inertia.ixx_kg_m2, inertia.ixy_kg_m2, inertia.ixz_kg_m2],
                [inertia.ixy_kg_m2, inertia.iyy_kg_m2, inertia.iyz_kg_m2],
                [inertia.ixz_kg_m2, inertia.iyz_kg_m2, inertia.izz_kg_m2],
            ],
        );
    }
    let center_of_mass_m = world
        .get::<Inertial>(entity)
        .map(|inertial| inertial.center_of_mass_m)
        .unwrap_or(Vec3::ZERO);
    SpatialInertia::point_mass(mass_kg, center_of_mass_m)
}

fn position_limits(kind: JointKind, lower: f64, upper: f64) -> Option<(f64, f64)> {
    match kind {
        JointKind::Revolute | JointKind::Prismatic
            if lower.is_finite() && upper.is_finite() && lower <= upper =>
        {
            Some((lower, upper))
        }
        _ => None,
    }
}

fn joint_motion_subspace(kind: JointKind, axis: Vec3) -> Option<SpatialVec> {
    let normalized = axis.normalize_or_zero();
    let axis = if normalized.length_squared() <= f64::EPSILON {
        Vec3::Y
    } else {
        normalized
    };
    match kind {
        JointKind::Fixed => None,
        JointKind::Revolute | JointKind::Continuous => {
            Some([0.0, 0.0, 0.0, axis.x, axis.y, axis.z])
        }
        JointKind::Prismatic => Some([axis.x, axis.y, axis.z, 0.0, 0.0, 0.0]),
    }
}
