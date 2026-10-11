//! Physics ECS components.

use std::sync::Arc;

use bevy_ecs::prelude::Component;
use rne_ecs::Entity;
use rne_math::{Quat, Vec3};
use rne_world::Transform3;
use serde::{Deserialize, Serialize};

/// Rigid body motion type.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum RigidBodyType {
    /// Fully simulated dynamic body.
    #[default]
    Dynamic,
    /// Immovable static body.
    Fixed,
    /// User-driven body with collision response.
    Kinematic,
}

/// Rigid body simulation properties.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RigidBody {
    /// Motion type.
    pub body_type: RigidBodyType,
    /// Mass in kilograms. Ignored for fixed bodies.
    pub mass_kg: f64,
    /// Linear velocity in meters per second.
    pub linear_velocity_m_s: Vec3,
    /// Angular velocity in radians per second.
    pub angular_velocity_rad_s: Vec3,
}

impl Default for RigidBody {
    fn default() -> Self {
        Self {
            body_type: RigidBodyType::Dynamic,
            mass_kg: 1.0,
            linear_velocity_m_s: Vec3::ZERO,
            angular_velocity_rad_s: Vec3::ZERO,
        }
    }
}

/// Exact rigid-body centre of mass and symmetric inertia tensor.
///
/// When present, [`RigidBody::mass_kg`] together with this component defines the
/// body's complete inertial properties. Physics backends must not add collider
/// mass or infer a replacement tensor. Tensor entries are expressed about
/// [`Self::center_of_mass_local_m`] in the rigid body's local frame.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RigidBodyInertia {
    /// Centre of mass in the rigid body's local frame, in metres.
    pub center_of_mass_local_m: Vec3,
    /// Inertia tensor x-x entry in kg·m².
    pub ixx_kg_m2: f64,
    /// Inertia tensor x-y entry in kg·m².
    pub ixy_kg_m2: f64,
    /// Inertia tensor x-z entry in kg·m².
    pub ixz_kg_m2: f64,
    /// Inertia tensor y-y entry in kg·m².
    pub iyy_kg_m2: f64,
    /// Inertia tensor y-z entry in kg·m².
    pub iyz_kg_m2: f64,
    /// Inertia tensor z-z entry in kg·m².
    pub izz_kg_m2: f64,
}

impl RigidBodyInertia {
    /// Returns true when every value is finite, the symmetric tensor is
    /// positive definite, and its principal moments satisfy the physical
    /// triangle inequalities.
    pub fn is_valid(self) -> bool {
        let values = [
            self.center_of_mass_local_m.x,
            self.center_of_mass_local_m.y,
            self.center_of_mass_local_m.z,
            self.ixx_kg_m2,
            self.ixy_kg_m2,
            self.ixz_kg_m2,
            self.iyy_kg_m2,
            self.iyz_kg_m2,
            self.izz_kg_m2,
        ];
        if values.into_iter().any(|value| !value.is_finite()) {
            return false;
        }
        let leading_minor_2 = self.ixx_kg_m2 * self.iyy_kg_m2 - self.ixy_kg_m2 * self.ixy_kg_m2;
        let determinant = self.ixx_kg_m2
            * (self.iyy_kg_m2 * self.izz_kg_m2 - self.iyz_kg_m2 * self.iyz_kg_m2)
            - self.ixy_kg_m2 * (self.ixy_kg_m2 * self.izz_kg_m2 - self.iyz_kg_m2 * self.ixz_kg_m2)
            + self.ixz_kg_m2 * (self.ixy_kg_m2 * self.iyz_kg_m2 - self.iyy_kg_m2 * self.ixz_kg_m2);
        if !(self.ixx_kg_m2 > 0.0 && leading_minor_2 > 0.0 && determinant > 0.0) {
            return false;
        }

        // Positive-definite symmetric matrices are not automatically
        // realizable inertia tensors. Principal moments must also satisfy the
        // triangle inequalities. Equivalently, trace(I)/2 * identity - I is a
        // positive-semidefinite second-moment matrix.
        let half_trace = (self.ixx_kg_m2 + self.iyy_kg_m2 + self.izz_kg_m2) * 0.5;
        let covariance = [
            [
                half_trace - self.ixx_kg_m2,
                -self.ixy_kg_m2,
                -self.ixz_kg_m2,
            ],
            [
                -self.ixy_kg_m2,
                half_trace - self.iyy_kg_m2,
                -self.iyz_kg_m2,
            ],
            [
                -self.ixz_kg_m2,
                -self.iyz_kg_m2,
                half_trace - self.izz_kg_m2,
            ],
        ];
        let tolerance = half_trace.abs().max(1.0) * 1.0e-12;
        let principal_minor_xy =
            covariance[0][0] * covariance[1][1] - covariance[0][1] * covariance[0][1];
        let principal_minor_xz =
            covariance[0][0] * covariance[2][2] - covariance[0][2] * covariance[0][2];
        let principal_minor_yz =
            covariance[1][1] * covariance[2][2] - covariance[1][2] * covariance[1][2];
        let covariance_determinant = covariance[0][0]
            * (covariance[1][1] * covariance[2][2] - covariance[1][2] * covariance[1][2])
            - covariance[0][1]
                * (covariance[0][1] * covariance[2][2] - covariance[1][2] * covariance[0][2])
            + covariance[0][2]
                * (covariance[0][1] * covariance[1][2] - covariance[1][1] * covariance[0][2]);
        covariance[0][0] >= -tolerance
            && covariance[1][1] >= -tolerance
            && covariance[2][2] >= -tolerance
            && principal_minor_xy >= -tolerance * tolerance
            && principal_minor_xz >= -tolerance * tolerance
            && principal_minor_yz >= -tolerance * tolerance
            && covariance_determinant >= -tolerance * tolerance * tolerance
    }
}

/// Collision shape definition.
///
/// Variable-size shapes ([`Self::ConvexHull`]) store their vertex data behind an
/// [`Arc`] so cloning a collider stays cheap. This makes the type `Clone` rather
/// than `Copy`; callers must clone or borrow instead of implicitly copying.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ColliderShape {
    /// Sphere with radius in meters.
    Sphere {
        /// Radius in meters.
        radius_m: f64,
    },
    /// Axis-aligned box half extents in meters.
    Cuboid {
        /// Half extents in meters.
        half_extents_m: Vec3,
    },
    /// Capsule aligned with the Y axis.
    Capsule {
        /// Half height in meters (excluding hemispheres).
        half_height_m: f64,
        /// Radius in meters.
        radius_m: f64,
    },
    /// Infinite plane with outward normal.
    Plane {
        /// Unit normal vector.
        normal: Vec3,
    },
    /// Convex hull of a point cloud expressed in the collider's local frame.
    ///
    /// The points are the hull vertices; their convex hull defines the collider
    /// volume. At least four points are required.
    ConvexHull {
        /// Hull points in the collider's local frame.
        points: Arc<[Vec3]>,
    },
    /// Triangle mesh with indexed faces in the collider's local frame.
    TriMesh {
        /// Mesh vertices in the collider's local frame.
        vertices: Arc<[Vec3]>,
        /// Triangle indices (flat triples) into `vertices`.
        indices: Arc<[u32]>,
    },
    /// Regular height field sampled over the local XZ plane.
    ///
    /// `heights_m` is row-major with `nrows * ncols` samples spanning
    /// `scale.x` by `scale.z` metres; `scale.y` multiplies each height.
    HeightField {
        /// Number of grid rows (Z direction).
        nrows: u32,
        /// Number of grid columns (X direction).
        ncols: u32,
        /// Row-major height samples.
        heights_m: Arc<[f64]>,
        /// Axis-aligned extent of the field in metres.
        scale: Vec3,
    },
    /// Union of child shapes with local offsets.
    Compound {
        /// Child shapes and their collider-local placements.
        parts: Arc<[CompoundPart]>,
    },
}

/// One child of a [`ColliderShape::Compound`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CompoundPart {
    /// Child shape.
    pub shape: ColliderShape,
    /// Pose of the child relative to the compound collider's local frame.
    pub local_offset: Transform3,
}

impl Default for ColliderShape {
    fn default() -> Self {
        Self::Cuboid {
            half_extents_m: Vec3::splat(0.5),
        }
    }
}

/// Optional convex-hull geometry replacing the companion [`Collider`]'s shape.
///
/// Vertices are expressed in entity-local meters; the companion's local offset
/// is ignored. Material, sensor and collision groups still come from `Collider`.
/// Author before the first physics synchronization; runtime geometry edits are
/// unsupported. Rapier builds a three-dimensional convex hull and rejects
/// nonfinite, degenerate or conflicting compound geometry. Backends without
/// support must not be used to qualify convex-contact behavior. The companion
/// primitive remains a bounding approximation for other consumers.
#[derive(Component, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ConvexCollider {
    /// Deterministically ordered point cloud spanning a nonzero 3D volume.
    pub vertices_m: Vec<Vec3>,
}

/// Marks a kinematic body whose pose is a *command* rather than a teleport.
///
/// Without this, a kinematic body's ECS pose is written straight into the
/// backend. That leaves its velocity at zero from the solver's point of view,
/// so nothing standing on it is carried: a platform moving laterally slides
/// out from under its cargo, and a rider on a rising platform trails it by the
/// penetration the solver has to resolve each step. With this component the
/// backend commands the body toward the pose over the step instead, so contact
/// behaves as it would for a real moving platform.
///
/// It is opt-in because teleport semantics are what some callers need. An
/// object carried by a gripper, for instance, should arrive exactly where it is
/// put each step, with no lag and no momentum handed to it on release.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CommandedKinematicPose;

/// One finite primitive in a compound collision shape.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ColliderPart {
    /// Primitive geometry; infinite planes are not supported in compounds.
    pub shape: ColliderShape,
    /// Pose relative to the entity, not to the companion collider's offset.
    pub local_offset: Transform3,
}

/// Optional compound geometry replacing the companion [`Collider`]'s shape.
///
/// The companion retains material, sensor and collision-group behavior. Parts
/// share one rigid body and do not add mass when declared inertia is present.
/// Rapier supports this component at collider creation; author it before the
/// first physics synchronization. Backends without compound support must not
/// be used to validate compound-contact behavior. Legacy `Collider` geometry
/// remains available as a broad bounding approximation for other consumers.
#[derive(Component, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CompoundCollider {
    /// Nonempty, deterministically ordered finite primitives.
    pub parts: Vec<ColliderPart>,
}

impl CompoundCollider {
    /// Returns whether all parts have finite poses and positive finite extents.
    pub fn is_valid(&self) -> bool {
        let positive =
            |x: f64| x.is_finite() && x > 0.0 && (x as f32).is_finite() && (x as f32) > 0.0;
        !self.parts.is_empty()
            && self.parts.iter().all(|part| {
                let pose = part.local_offset;
                pose.translation.is_finite()
                    && pose.rotation.is_finite()
                    && (pose.rotation.length_squared() - 1.0).abs() < 1e-6
                    && match part.shape {
                        ColliderShape::Sphere { radius_m } => positive(radius_m),
                        ColliderShape::Cuboid { half_extents_m } => {
                            half_extents_m.to_array().into_iter().all(positive)
                        }
                        ColliderShape::Capsule {
                            half_height_m,
                            radius_m,
                        } => {
                            half_height_m.is_finite()
                                && (half_height_m as f32).is_finite()
                                && half_height_m >= 0.0
                                && positive(radius_m)
                        }
                        ColliderShape::Plane { .. }
                        | ColliderShape::ConvexHull { .. }
                        | ColliderShape::TriMesh { .. }
                        | ColliderShape::HeightField { .. }
                        | ColliderShape::Compound { .. } => false,
                    }
            })
    }
}

/// Collider attached to an entity.
#[derive(Component, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Collider {
    /// Shape definition.
    pub shape: ColliderShape,
    /// Surface material properties.
    pub material: PhysicsMaterial,
    /// Pose relative to the entity transform.
    pub local_offset: Transform3,
    /// Whether this collider reports overlap without applying contact forces.
    ///
    /// Sensor overlaps are exposed as zero-impulse [`crate::ContactEvent`] values.
    #[serde(default)]
    pub sensor: bool,
}

impl Default for Collider {
    fn default() -> Self {
        Self {
            shape: ColliderShape::default(),
            material: PhysicsMaterial::default(),
            local_offset: Transform3::IDENTITY,
            sensor: false,
        }
    }
}

impl Collider {
    /// Creates a cuboid collider with the given half extents.
    pub fn cuboid(half_extents_m: Vec3) -> Self {
        Self {
            shape: ColliderShape::Cuboid { half_extents_m },
            material: PhysicsMaterial::default(),
            local_offset: Transform3::IDENTITY,
            sensor: false,
        }
    }

    /// Creates a sphere collider with the given radius.
    pub fn sphere(radius_m: f64) -> Self {
        Self {
            shape: ColliderShape::Sphere { radius_m },
            material: PhysicsMaterial::default(),
            local_offset: Transform3::IDENTITY,
            sensor: false,
        }
    }

    /// Creates a convex-hull collider from local-frame hull points.
    pub fn convex_hull(points: impl Into<Arc<[Vec3]>>) -> Self {
        Self {
            shape: ColliderShape::ConvexHull {
                points: points.into(),
            },
            material: PhysicsMaterial::default(),
            local_offset: Transform3::IDENTITY,
            sensor: false,
        }
    }
}

/// Pairwise collider filtering masks.
///
/// Two colliders interact when each collider's membership overlaps the other
/// collider's filter. Missing components are treated as [`Self::default`].
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollisionGroups {
    /// Groups this collider belongs to.
    pub memberships: u32,
    /// Groups this collider accepts interactions from.
    pub filter: u32,
}

impl Default for CollisionGroups {
    fn default() -> Self {
        Self {
            memberships: u32::MAX,
            filter: u32::MAX,
        }
    }
}

impl CollisionGroups {
    /// Creates masks that disable interactions with colliders in the same group.
    pub const fn without_self_collision(group_bit: u32) -> Self {
        Self {
            memberships: group_bit,
            filter: !group_bit,
        }
    }
}

/// Physical surface material.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PhysicsMaterial {
    /// Coulomb friction coefficient.
    pub friction: f32,
    /// Coefficient of restitution.
    pub restitution: f32,
}

impl Default for PhysicsMaterial {
    fn default() -> Self {
        Self {
            friction: 0.5,
            restitution: 0.0,
        }
    }
}

/// Constant diagonal inertia added to one revolute generalized coordinate.
///
/// Attach to the child link alongside `RevoluteJointDesc` and `MultibodyLink`.
/// The Rapier backend with `experimental-armature` and the repository patch
/// applies this to force and constraint dynamics without
/// changing link spatial mass/inertia. Zero or absence preserves the original
/// plant. Non-finite, negative, unrepresentable or unsupported configurations
/// are rejected. Other backends do not yet implement this component.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RevoluteJointArmature {
    /// Reflected motor inertia in kilogram-square-metres.
    pub inertia_kg_m2: f64,
}

/// Revolute joint description for physics backends.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct RevoluteJointDesc {
    /// Parent rigid body entity.
    pub parent: Entity,
    /// Joint axis before applying [`Self::relative_rotation`] and the parent
    /// body rotation; the identity relative rotation preserves parent-local axes.
    pub axis: Vec3,
    /// Anchor point in the parent body's local frame.
    pub anchor_parent_m: Vec3,
    /// Anchor point in the child body's local frame.
    pub anchor_child_m: Vec3,
    /// Orientation of the child joint frame relative to the parent joint
    /// frame at joint angle zero. [`Quat::IDENTITY`] preserves the legacy
    /// axis-aligned convention; URDF import sets the joint-origin rotation
    /// when the asset opts into `use_joint_origin_rpy` so angle zero matches
    /// the authored pose.
    pub relative_rotation: Quat,
    /// Optional lower angle limit in radians.
    pub lower_rad: Option<f64>,
    /// Optional upper angle limit in radians.
    pub upper_rad: Option<f64>,
}

/// Prismatic (linear sliding) joint description for physics backends.
///
/// The single free degree of freedom translates the child body along `axis`,
/// expressed in the parent body's local frame.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct PrismaticJointDesc {
    /// Parent rigid body entity.
    pub parent: Entity,
    /// Sliding axis before applying [`Self::relative_rotation`] and the parent
    /// body rotation; the identity relative rotation preserves parent-local axes.
    pub axis: Vec3,
    /// Anchor point in the parent body's local frame.
    pub anchor_parent_m: Vec3,
    /// Anchor point in the child body's local frame.
    pub anchor_child_m: Vec3,
    /// Orientation of the child joint frame relative to the parent joint
    /// frame at zero displacement (see [`RevoluteJointDesc::relative_rotation`]).
    pub relative_rotation: Quat,
    /// Optional lower translation limit in meters.
    pub lower_m: Option<f64>,
    /// Optional upper translation limit in meters.
    pub upper_m: Option<f64>,
}

/// Fixed (weld) joint description for physics backends.
///
/// Rigidly locks the child body to the parent at the relative pose implied by the
/// anchors and `relative_rotation`, removing all six relative degrees of freedom.
/// Inserting this component attaches the weld on the next sync; removing it releases
/// the weld. Intended for attach-on-contact grasping (weld a grasped object to the
/// gripper at its current relative pose so it neither snaps nor drifts).
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct FixedJointDesc {
    /// Parent rigid body entity (e.g. the gripper link).
    pub parent: Entity,
    /// Anchor point in the parent body's local frame.
    pub anchor_parent_m: Vec3,
    /// Anchor point in the child body's local frame.
    pub anchor_child_m: Vec3,
    /// Orientation of the child frame relative to the parent frame.
    pub relative_rotation: Quat,
}

/// Marks a rigid body and its joint as part of a reduced-coordinate multibody.
///
/// Backends that support multibodies use this instead of an impulse joint. The
/// marker may also be placed on the root so it is simulated without a collider.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MultibodyLink;

/// Marks a rigid body whose pose is owned by its reduced-coordinate assembly
/// rather than by the ECS.
///
/// A backend must not write the entity's ECS transform back into its physics
/// body: when an assembly's root is re-pinned each step, writing stale child
/// poses desynchronizes the chain. The root itself is not marked and remains
/// ECS-driven.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PhysicsOwnedPose;

/// Per-body gravity multiplier.
///
/// `1.0` (the default when absent) applies the world gravity unchanged; `0.0`
/// makes the body weightless. Used for a velocity-driven mobile chassis whose
/// reduced-coordinate arm must not be dragged down by a falling base.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct GravityScale(pub f64);

impl Default for GravityScale {
    fn default() -> Self {
        Self(1.0)
    }
}

/// Velocity motor command applied to a joint before each physics step.
///
/// The value is interpreted as an angular velocity (rad/s) for revolute joints
/// and as a linear velocity (m/s) for prismatic joints.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct JointMotor {
    /// Target velocity: radians per second (revolute) or meters per second (prismatic).
    pub velocity_rad_s: f64,
    /// Velocity-tracking gain (motor damping factor). Higher values track the target
    /// velocity more stiffly under load — e.g. a joint holding weight against gravity —
    /// up to the backend's motor force cap. Defaults to `1.0`.
    #[serde(default = "default_motor_gain")]
    pub gain: f64,
    /// Position-tracking stiffness. When `0.0` (the default) the motor is a pure
    /// velocity motor. When positive, the motor also pulls the joint toward
    /// [`Self::target_position`] like a spring (with `gain` acting as its damping),
    /// which lets a joint *hold* a load against gravity without drift — required
    /// for a stable vertical lift carrying a multi-link arm.
    #[serde(default)]
    pub stiffness: f64,
    /// Position target the motor pulls toward when [`Self::stiffness`] is positive:
    /// radians (revolute) or meters along the slide axis (prismatic).
    #[serde(default)]
    pub target_position: f64,
    /// Maximum force/torque the motor may apply. `0.0` (the default) uses the backend's
    /// per-joint-type cap; a positive value overrides it (e.g. a heavier arm joint that
    /// needs more torque to track its target quickly).
    #[serde(default)]
    pub max_force: f64,
}

/// Selects how a backend interprets position/velocity servo gains.
///
/// The default preserves the historical mass-normalized response. Controllers
/// that declare gains in force or torque units must opt into [`Self::ForceBased`]
/// and retain that choice as part of their configuration evidence.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointMotorGainModel {
    /// Gains produce a target acceleration and are normalized by body inertia.
    #[default]
    AccelerationBased,
    /// Gains produce force/torque directly in the units declared by actuation.
    ForceBased,
}

/// Backend-neutral passive dynamics of a single-degree-of-freedom joint.
///
/// The component describes plant loss independently from actuator servo gains.
/// Physics backends apply viscous damping against joint velocity. Coulomb loss
/// uses the backend-neutral regularization `-magnitude * tanh(velocity /
/// transition_velocity)`. This is a smooth kinetic-friction model, not a claim
/// of true set-valued static friction or breakaway behavior.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum JointPassiveDynamics {
    /// Passive dynamics of a revolute joint.
    Revolute {
        /// Viscous damping coefficient in newton-metre-seconds per radian.
        viscous_damping_nm_s_per_rad: f64,
        /// Requested Coulomb-friction magnitude in newton-metres.
        coulomb_friction_nm: f64,
        /// Velocity scale of the smooth Coulomb transition in radians per second.
        #[serde(default)]
        coulomb_transition_velocity_rad_s: f64,
    },
    /// Passive dynamics of a prismatic joint.
    Prismatic {
        /// Viscous damping coefficient in newton-seconds per metre.
        viscous_damping_n_s_per_m: f64,
        /// Requested Coulomb-friction magnitude in newtons.
        coulomb_friction_n: f64,
        /// Velocity scale of the smooth Coulomb transition in metres per second.
        #[serde(default)]
        coulomb_transition_velocity_m_s: f64,
    },
}

impl JointPassiveDynamics {
    /// Returns true when every coefficient is finite and non-negative.
    pub fn has_valid_values(self) -> bool {
        match self {
            Self::Revolute {
                viscous_damping_nm_s_per_rad,
                coulomb_friction_nm,
                coulomb_transition_velocity_rad_s,
            } => {
                non_negative(viscous_damping_nm_s_per_rad)
                    && valid_coulomb_transition(
                        coulomb_friction_nm,
                        coulomb_transition_velocity_rad_s,
                    )
            }
            Self::Prismatic {
                viscous_damping_n_s_per_m,
                coulomb_friction_n,
                coulomb_transition_velocity_m_s,
            } => {
                non_negative(viscous_damping_n_s_per_m)
                    && valid_coulomb_transition(coulomb_friction_n, coulomb_transition_velocity_m_s)
            }
        }
    }

    /// Computes the signed generalized Coulomb-loss effort opposing `velocity`.
    ///
    /// Callers must first require [`Self::has_valid_values`] and a finite
    /// velocity. The result is a torque in newton-metres for revolute joints and
    /// a force in newtons for prismatic joints.
    pub fn regularized_coulomb_effort(self, velocity: f64) -> f64 {
        let (magnitude, transition_velocity) = match self {
            Self::Revolute {
                coulomb_friction_nm,
                coulomb_transition_velocity_rad_s,
                ..
            } => (coulomb_friction_nm, coulomb_transition_velocity_rad_s),
            Self::Prismatic {
                coulomb_friction_n,
                coulomb_transition_velocity_m_s,
                ..
            } => (coulomb_friction_n, coulomb_transition_velocity_m_s),
        };
        if magnitude == 0.0 {
            0.0
        } else {
            -magnitude * (velocity / transition_velocity).tanh()
        }
    }
}

fn valid_coulomb_transition(magnitude: f64, transition_velocity: f64) -> bool {
    non_negative(magnitude)
        && non_negative(transition_velocity)
        && (magnitude == 0.0 || transition_velocity > 0.0)
}

/// Unit-explicit actuation command for a one-degree-of-freedom joint.
///
/// Physics backends apply this component before each fixed step. Commands with
/// non-finite values, negative gains, negative limits, or a revolute/prismatic
/// mode that disagrees with the joint description must be rejected before the
/// step. A zero maximum effort/force disables output rather than meaning
/// unbounded output.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case", deny_unknown_fields)]
pub enum JointActuation {
    /// No actuator output.
    #[default]
    Disabled,
    /// Revolute position servo.
    RevolutePosition {
        /// Desired angular position in radians.
        target_position_rad: f64,
        /// Proportional stiffness in newton-metres per radian.
        stiffness_nm_per_rad: f64,
        /// Damping in newton-metre-seconds per radian.
        damping_nm_s_per_rad: f64,
        /// Symmetric output limit in newton-metres.
        max_effort_nm: f64,
    },
    /// Revolute velocity servo.
    RevoluteVelocity {
        /// Desired angular velocity in radians per second.
        target_velocity_rad_s: f64,
        /// Velocity error gain in newton-metre-seconds per radian.
        gain_nm_s_per_rad: f64,
        /// Symmetric output limit in newton-metres.
        max_effort_nm: f64,
    },
    /// Direct revolute torque command.
    RevoluteEffort {
        /// Requested torque in newton-metres.
        effort_nm: f64,
        /// Symmetric output limit in newton-metres.
        max_effort_nm: f64,
    },
    /// Prismatic position servo.
    PrismaticPosition {
        /// Desired translation in metres.
        target_position_m: f64,
        /// Proportional stiffness in newtons per metre.
        stiffness_n_per_m: f64,
        /// Damping in newton-seconds per metre.
        damping_n_s_per_m: f64,
        /// Symmetric output limit in newtons.
        max_force_n: f64,
    },
    /// Prismatic velocity servo.
    PrismaticVelocity {
        /// Desired linear velocity in metres per second.
        target_velocity_m_s: f64,
        /// Velocity error gain in newton-seconds per metre.
        gain_n_s_per_m: f64,
        /// Symmetric output limit in newtons.
        max_force_n: f64,
    },
    /// Direct prismatic force command.
    PrismaticEffort {
        /// Requested force in newtons.
        force_n: f64,
        /// Symmetric output limit in newtons.
        max_force_n: f64,
    },
}

impl JointActuation {
    /// Returns true when every value is finite and every gain/limit is non-negative.
    pub fn has_valid_values(self) -> bool {
        match self {
            Self::Disabled => true,
            Self::RevolutePosition {
                target_position_rad,
                stiffness_nm_per_rad,
                damping_nm_s_per_rad,
                max_effort_nm,
            } => {
                target_position_rad.is_finite()
                    && non_negative(stiffness_nm_per_rad)
                    && non_negative(damping_nm_s_per_rad)
                    && non_negative(max_effort_nm)
            }
            Self::RevoluteVelocity {
                target_velocity_rad_s,
                gain_nm_s_per_rad,
                max_effort_nm,
            } => {
                target_velocity_rad_s.is_finite()
                    && non_negative(gain_nm_s_per_rad)
                    && non_negative(max_effort_nm)
            }
            Self::RevoluteEffort {
                effort_nm,
                max_effort_nm,
            } => effort_nm.is_finite() && non_negative(max_effort_nm),
            Self::PrismaticPosition {
                target_position_m,
                stiffness_n_per_m,
                damping_n_s_per_m,
                max_force_n,
            } => {
                target_position_m.is_finite()
                    && non_negative(stiffness_n_per_m)
                    && non_negative(damping_n_s_per_m)
                    && non_negative(max_force_n)
            }
            Self::PrismaticVelocity {
                target_velocity_m_s,
                gain_n_s_per_m,
                max_force_n,
            } => {
                target_velocity_m_s.is_finite()
                    && non_negative(gain_n_s_per_m)
                    && non_negative(max_force_n)
            }
            Self::PrismaticEffort {
                force_n,
                max_force_n,
            } => force_n.is_finite() && non_negative(max_force_n),
        }
    }

    /// Returns true for disabled or revolute commands.
    pub const fn supports_revolute(self) -> bool {
        matches!(
            self,
            Self::Disabled
                | Self::RevolutePosition { .. }
                | Self::RevoluteVelocity { .. }
                | Self::RevoluteEffort { .. }
        )
    }

    /// Returns true for disabled or prismatic commands.
    pub const fn supports_prismatic(self) -> bool {
        matches!(
            self,
            Self::Disabled
                | Self::PrismaticPosition { .. }
                | Self::PrismaticVelocity { .. }
                | Self::PrismaticEffort { .. }
        )
    }
}

/// Backend-reported effort realized during the most recently completed step.
///
/// Backends insert or replace this component during physics-to-ECS
/// synchronization only when they can expose a qualifying realized actuator
/// effort. Absence is meaningful and must remain distinguishable from a
/// measured zero. Joint-feedback sensors sample the value at their simulation
/// capture time and apply their declared transport latency without adding
/// noise.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum JointEffortMeasurement {
    /// Realized revolute-joint effort in newton-metres.
    Revolute {
        /// Measured effort in newton-metres.
        measured_effort_nm: f64,
    },
    /// Realized prismatic-joint force in newtons.
    Prismatic {
        /// Measured force in newtons.
        measured_force_n: f64,
    },
}

impl JointEffortMeasurement {
    /// Returns true when the measured value is finite.
    pub fn has_valid_value(self) -> bool {
        match self {
            Self::Revolute { measured_effort_nm } => measured_effort_nm.is_finite(),
            Self::Prismatic { measured_force_n } => measured_force_n.is_finite(),
        }
    }
}

/// Time scope and exact coverage counters of completed position-motor samples.
///
/// Numerical durations describe the backend's executed solver intervals; the
/// nominal tick count describes the requested engine step. They may differ by
/// numeric representation. Producers establish full coverage from exact
/// per-partition and per-slice counts, never from approximately equal durations.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PositionMotorDiagnosticInterval {
    /// Owning physics-world identifier within this backend instance.
    ///
    /// This namespace is not globally unique and must not be compared across
    /// different backend instances or backend implementations.
    pub physics_world_id: u32,
    /// Requested outer physics-step duration in nanosecond simulation ticks.
    pub nominal_step_ticks: u64,
    /// Backend numerical duration of the outer physics step, in seconds.
    pub numerical_outer_interval_s: f64,
    /// Numerical durations of executed CCD partitions, in execution order.
    pub ccd_partition_durations_s: Vec<f64>,
    /// Sum of the durations of qualifying observed motor solver slices.
    ///
    /// This is the denominator of the observed-interval mean effort or force.
    pub observed_solver_interval_s: f64,
    /// Number of qualifying completed motor solver slices actually observed.
    pub observed_solver_slice_count: u64,
    /// Whether exact integer coverage includes every outer-step partition/slice.
    ///
    /// A floating duration match alone must never set this flag.
    pub full_step_coverage: bool,
    /// Number of executed CCD partitions containing qualifying observed slices.
    pub observed_partition_count: u64,
    /// Observed slices with a finite, strictly positive motor impulse bound.
    pub finite_positive_bound_eligible_slice_count: u64,
    /// Sum of the durations of finite-positive-bound eligible slices, in seconds.
    pub finite_positive_bound_eligible_duration_s: f64,
    /// Eligible slices whose completed motor impulse magnitude reached the bound.
    ///
    /// This counts solver-bound activity, not target clipping or total joint load.
    pub finite_positive_bound_active_slice_count: u64,
    /// Sum of the durations of finite-positive-bound active slices, in seconds.
    pub finite_positive_bound_active_duration_s: f64,
}

impl PositionMotorDiagnosticInterval {
    /// Checks finite durations and nested sample-counter/duration consistency.
    ///
    /// This structural check cannot prove a producer's per-slice coverage claim
    /// and does not infer full coverage from numerical duration equality.
    pub fn has_valid_values(&self) -> bool {
        let partition_count = self.ccd_partition_durations_s.len() as u64;
        self.nominal_step_ticks > 0
            && self.numerical_outer_interval_s.is_finite()
            && self.numerical_outer_interval_s > 0.0
            && partition_count > 0
            && self
                .ccd_partition_durations_s
                .iter()
                .all(|duration| duration.is_finite() && *duration > 0.0)
            && self.observed_solver_slice_count > 0
            && self.observed_partition_count > 0
            && self.observed_partition_count <= partition_count
            && self.observed_partition_count <= self.observed_solver_slice_count
            && (!self.full_step_coverage || self.observed_partition_count == partition_count)
            && sample_duration_is_valid(
                self.observed_solver_slice_count,
                self.observed_solver_interval_s,
            )
            && sample_duration_is_valid(
                self.finite_positive_bound_eligible_slice_count,
                self.finite_positive_bound_eligible_duration_s,
            )
            && sample_duration_is_valid(
                self.finite_positive_bound_active_slice_count,
                self.finite_positive_bound_active_duration_s,
            )
            && self.finite_positive_bound_eligible_slice_count <= self.observed_solver_slice_count
            && self.finite_positive_bound_active_slice_count
                <= self.finite_positive_bound_eligible_slice_count
            && self.finite_positive_bound_eligible_duration_s <= self.observed_solver_interval_s
            && self.finite_positive_bound_active_duration_s
                <= self.finite_positive_bound_eligible_duration_s
    }
}

/// Optional backend-observed position-motor impulses from a completed step.
///
/// The component describes motor-row realization, not a command, total joint
/// reaction, target clipping, or hardware measurement. It is separate from
/// [`JointEffortMeasurement`]. Absence means unsupported or inactive diagnostics,
/// a sleeping motor, or no qualifying completed motor sample; it is not zero.
/// A ceiling of `Some(0.0)` is a measured zero bound. A ceiling of `None` means
/// positive infinity, never a missing motor measurement.
///
/// A new [`crate::PhysicsBackend::sync_from_ecs`] clears the owning world's old
/// completed diagnostic data before accepting new inputs. An invalid sync must
/// not retain an earlier successful sample. Data becomes available again only
/// after a qualifying completed step and physics-to-ECS publication.
#[derive(Component, Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum JointPositionMotorDiagnostics {
    /// Completed angular position-motor impulses with torque units.
    Revolute {
        /// Signed sum of observed completed motor impulses, in N·m·s.
        applied_motor_impulse_nm_s: f64,
        /// Signed impulse divided by the observed solver interval, in N·m.
        mean_observed_motor_effort_nm: f64,
        /// Observed-interval mean effort qualified by full coverage, in N·m.
        ///
        /// Present only when exact full partition/slice coverage is established.
        /// The denominator remains `observed_solver_interval_s`; the numerical
        /// outer interval is separate metadata, never this mean's denominator.
        mean_completed_motor_effort_nm: Option<f64>,
        /// Effective symmetric motor torque ceiling in N·m; `None` is unbounded.
        effective_effort_ceiling_nm: Option<f64>,
        /// Completed solver timing, coverage, and finite-bound activity.
        interval: PositionMotorDiagnosticInterval,
    },
    /// Completed linear position-motor impulses with force units.
    Prismatic {
        /// Signed sum of observed completed motor impulses, in N·s.
        applied_motor_impulse_n_s: f64,
        /// Signed impulse divided by the observed solver interval, in newtons.
        mean_observed_motor_force_n: f64,
        /// Observed-interval mean force qualified by full coverage, in newtons.
        ///
        /// Present only when exact full partition/slice coverage is established.
        /// The denominator remains `observed_solver_interval_s`; the numerical
        /// outer interval is separate metadata, never this mean's denominator.
        mean_completed_motor_force_n: Option<f64>,
        /// Effective symmetric motor force ceiling in newtons; `None` is unbounded.
        effective_force_ceiling_n: Option<f64>,
        /// Completed solver timing, coverage, and finite-bound activity.
        interval: PositionMotorDiagnosticInterval,
    },
}

impl JointPositionMotorDiagnostics {
    /// Checks finite measured values, ceiling semantics, and interval structure.
    ///
    /// A completed-step mean requires full integer coverage. A zero ceiling
    /// requires measured zero and no finite-positive-bound eligible slices.
    /// This check does not independently establish the physical observation or
    /// recompute a producer's signed impulse accumulation.
    pub fn has_valid_values(&self) -> bool {
        let (impulse, observed_mean, completed_mean, ceiling, interval) = match self {
            Self::Revolute {
                applied_motor_impulse_nm_s,
                mean_observed_motor_effort_nm,
                mean_completed_motor_effort_nm,
                effective_effort_ceiling_nm,
                interval,
            } => (
                *applied_motor_impulse_nm_s,
                *mean_observed_motor_effort_nm,
                *mean_completed_motor_effort_nm,
                *effective_effort_ceiling_nm,
                interval,
            ),
            Self::Prismatic {
                applied_motor_impulse_n_s,
                mean_observed_motor_force_n,
                mean_completed_motor_force_n,
                effective_force_ceiling_n,
                interval,
            } => (
                *applied_motor_impulse_n_s,
                *mean_observed_motor_force_n,
                *mean_completed_motor_force_n,
                *effective_force_ceiling_n,
                interval,
            ),
        };
        impulse.is_finite()
            && observed_mean.is_finite()
            && completed_mean.is_none_or(|mean| mean.is_finite() && interval.full_step_coverage)
            && ceiling.is_none_or(non_negative)
            && interval.has_valid_values()
            && (ceiling != Some(0.0)
                || (impulse == 0.0
                    && observed_mean == 0.0
                    && completed_mean.is_none_or(|mean| mean == 0.0)
                    && interval.finite_positive_bound_eligible_slice_count == 0))
    }
}

fn sample_duration_is_valid(count: u64, duration_s: f64) -> bool {
    non_negative(duration_s)
        && ((count == 0 && duration_s == 0.0) || (count > 0 && duration_s > 0.0))
}

fn non_negative(value: f64) -> bool {
    value.is_finite() && value >= 0.0
}

/// Backend-neutral completed-step state of a single-degree-of-freedom joint.
///
/// Physics backends insert or update this component during
/// [`crate::PhysicsBackend::sync_to_ecs`]. The enum keeps revolute and
/// prismatic units explicit instead of overloading one untyped coordinate.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum JointState {
    /// Angular coordinate and velocity of a revolute joint.
    Revolute {
        /// Joint position in radians.
        position_rad: f64,
        /// Joint velocity in radians per second.
        velocity_rad_s: f64,
    },
    /// Linear coordinate and velocity of a prismatic joint.
    Prismatic {
        /// Joint position in metres.
        position_m: f64,
        /// Joint velocity in metres per second.
        velocity_m_s: f64,
    },
    /// A fixed joint with no free coordinate.
    Fixed,
}

impl JointState {
    /// Returns the revolute position in radians when this is a revolute joint.
    pub const fn position_rad(self) -> Option<f64> {
        match self {
            Self::Revolute { position_rad, .. } => Some(position_rad),
            Self::Prismatic { .. } | Self::Fixed => None,
        }
    }

    /// Returns the prismatic position in metres when this is a prismatic joint.
    pub const fn position_m(self) -> Option<f64> {
        match self {
            Self::Prismatic { position_m, .. } => Some(position_m),
            Self::Revolute { .. } | Self::Fixed => None,
        }
    }
}

fn default_motor_gain() -> f64 {
    1.0
}

impl Default for JointMotor {
    fn default() -> Self {
        Self {
            velocity_rad_s: 0.0,
            gain: default_motor_gain(),
            stiffness: 0.0,
            target_position: 0.0,
            max_force: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        Collider, ColliderShape, JointActuation, JointPassiveDynamics,
        JointPositionMotorDiagnostics, PositionMotorDiagnosticInterval, RigidBodyInertia,
    };
    use rne_math::Vec3;
    use std::sync::Arc;

    fn completed_motor_interval() -> PositionMotorDiagnosticInterval {
        PositionMotorDiagnosticInterval {
            physics_world_id: 4,
            nominal_step_ticks: 1_000_000,
            numerical_outer_interval_s: 0.001,
            ccd_partition_durations_s: vec![0.0004, 0.0006],
            observed_solver_interval_s: 0.001,
            observed_solver_slice_count: 4,
            full_step_coverage: true,
            observed_partition_count: 2,
            finite_positive_bound_eligible_slice_count: 4,
            finite_positive_bound_eligible_duration_s: 0.001,
            finite_positive_bound_active_slice_count: 2,
            finite_positive_bound_active_duration_s: 0.0004,
        }
    }

    #[test]
    fn position_motor_measured_zero_and_unbounded_ceiling_are_distinct() {
        let mut interval = completed_motor_interval();
        interval.finite_positive_bound_eligible_slice_count = 0;
        interval.finite_positive_bound_eligible_duration_s = 0.0;
        interval.finite_positive_bound_active_slice_count = 0;
        interval.finite_positive_bound_active_duration_s = 0.0;
        let zero = JointPositionMotorDiagnostics::Revolute {
            applied_motor_impulse_nm_s: 0.0,
            mean_observed_motor_effort_nm: 0.0,
            mean_completed_motor_effort_nm: Some(0.0),
            effective_effort_ceiling_nm: Some(0.0),
            interval: interval.clone(),
        };
        assert!(zero.has_valid_values());
        let unbounded = JointPositionMotorDiagnostics::Revolute {
            applied_motor_impulse_nm_s: -0.003,
            mean_observed_motor_effort_nm: -3.0,
            mean_completed_motor_effort_nm: Some(-3.0),
            effective_effort_ceiling_nm: None,
            interval,
        };
        assert!(unbounded.has_valid_values());
        let zero_json = serde_json::to_value(zero).expect("measured zero serializes");
        let unbounded_json = serde_json::to_value(unbounded).expect("unbounded sample serializes");
        assert_eq!(zero_json["effective_effort_ceiling_nm"], 0.0);
        assert_eq!(
            unbounded_json["effective_effort_ceiling_nm"],
            serde_json::Value::Null
        );
        assert_eq!(unbounded_json["mean_observed_motor_effort_nm"], -3.0);
        assert_ne!(zero_json, serde_json::Value::Null);
        assert_ne!(unbounded_json, serde_json::Value::Null);
    }

    #[test]
    fn partial_motor_coverage_cannot_report_a_completed_step_mean() {
        let mut interval = completed_motor_interval();
        interval.full_step_coverage = false;
        // Equal numerical durations do not establish exact solver-slice coverage.
        let mut sample = JointPositionMotorDiagnostics::Prismatic {
            applied_motor_impulse_n_s: 0.003,
            mean_observed_motor_force_n: 3.0,
            mean_completed_motor_force_n: None,
            effective_force_ceiling_n: Some(10.0),
            interval,
        };
        assert!(sample.has_valid_values());
        if let JointPositionMotorDiagnostics::Prismatic {
            mean_completed_motor_force_n,
            ..
        } = &mut sample
        {
            *mean_completed_motor_force_n = Some(3.0);
        }
        assert!(!sample.has_valid_values());
    }

    #[test]
    fn position_motor_values_reject_nonfinite_negative_or_false_zero_bounds() {
        for ceiling in [Some(f64::INFINITY), Some(f64::NAN), Some(-1.0), Some(0.0)] {
            assert!(!JointPositionMotorDiagnostics::Revolute {
                applied_motor_impulse_nm_s: 0.003,
                mean_observed_motor_effort_nm: 3.0,
                mean_completed_motor_effort_nm: Some(3.0),
                effective_effort_ceiling_nm: ceiling,
                interval: completed_motor_interval(),
            }
            .has_valid_values());
        }
        for (impulse, observed, completed) in [
            (f64::NAN, 3.0, Some(3.0)),
            (0.003, f64::INFINITY, Some(3.0)),
            (0.003, 3.0, Some(f64::NEG_INFINITY)),
        ] {
            assert!(!JointPositionMotorDiagnostics::Prismatic {
                applied_motor_impulse_n_s: impulse,
                mean_observed_motor_force_n: observed,
                mean_completed_motor_force_n: completed,
                effective_force_ceiling_n: Some(10.0),
                interval: completed_motor_interval(),
            }
            .has_valid_values());
        }
        assert!(
            !JointPositionMotorDiagnostics::Revolute {
                applied_motor_impulse_nm_s: 0.0,
                mean_observed_motor_effort_nm: 0.0,
                mean_completed_motor_effort_nm: Some(0.0),
                effective_effort_ceiling_nm: Some(0.0),
                interval: completed_motor_interval(),
            }
            .has_valid_values(),
            "zero bounds must not claim positive-bound eligibility"
        );
    }

    #[test]
    fn motor_interval_counters_and_duration_subsets_must_agree() {
        let valid = completed_motor_interval();
        assert!(valid.has_valid_values());
        let mutators: [fn(&mut PositionMotorDiagnosticInterval); 13] = [
            |interval| interval.nominal_step_ticks = 0,
            |interval| interval.numerical_outer_interval_s = f64::NAN,
            |interval| interval.ccd_partition_durations_s[0] = 0.0,
            |interval| interval.observed_solver_slice_count = 0,
            |interval| interval.observed_partition_count = 1,
            |interval| interval.observed_partition_count = 3,
            |interval| interval.finite_positive_bound_eligible_slice_count = 5,
            |interval| interval.finite_positive_bound_active_slice_count = 5,
            |interval| interval.finite_positive_bound_eligible_slice_count = 0,
            |interval| interval.finite_positive_bound_eligible_duration_s = 0.0,
            |interval| interval.finite_positive_bound_active_duration_s = 0.002,
            |interval| interval.finite_positive_bound_active_duration_s = -0.1,
            |interval| interval.finite_positive_bound_active_duration_s = f64::INFINITY,
        ];
        for mutate in mutators {
            let mut invalid = valid.clone();
            mutate(&mut invalid);
            assert!(!invalid.has_valid_values());
        }
        let partial = PositionMotorDiagnosticInterval {
            full_step_coverage: false,
            observed_partition_count: 1,
            observed_solver_slice_count: 2,
            observed_solver_interval_s: 0.0004,
            finite_positive_bound_eligible_slice_count: 2,
            finite_positive_bound_eligible_duration_s: 0.0004,
            finite_positive_bound_active_slice_count: 1,
            finite_positive_bound_active_duration_s: 0.0002,
            ..valid
        };
        assert!(partial.has_valid_values());
    }

    #[test]
    fn position_motor_diagnostics_round_trip_units_and_reject_unknown_fields() {
        for sample in [
            JointPositionMotorDiagnostics::Revolute {
                applied_motor_impulse_nm_s: -0.003,
                mean_observed_motor_effort_nm: -3.0,
                mean_completed_motor_effort_nm: Some(-3.0),
                effective_effort_ceiling_nm: Some(10.0),
                interval: completed_motor_interval(),
            },
            JointPositionMotorDiagnostics::Prismatic {
                applied_motor_impulse_n_s: 0.003,
                mean_observed_motor_force_n: 3.0,
                mean_completed_motor_force_n: Some(3.0),
                effective_force_ceiling_n: Some(10.0),
                interval: completed_motor_interval(),
            },
        ] {
            let value = serde_json::to_value(&sample).expect("serialize motor diagnostics");
            let decoded: JointPositionMotorDiagnostics =
                serde_json::from_value(value.clone()).expect("deserialize motor diagnostics");
            assert_eq!(decoded, sample);
            assert!(decoded.has_valid_values());
            assert_eq!(value["interval"]["physics_world_id"], 4);
            assert!(matches!(
                value["kind"].as_str(),
                Some("revolute" | "prismatic")
            ));
            let mut unknown = value.clone();
            unknown["total_joint_reaction"] = serde_json::json!(1.0);
            assert!(serde_json::from_value::<JointPositionMotorDiagnostics>(unknown).is_err());
            let mut unknown_interval = value;
            unknown_interval["interval"]["estimated_full_coverage"] = serde_json::json!(true);
            assert!(
                serde_json::from_value::<JointPositionMotorDiagnostics>(unknown_interval).is_err()
            );
        }
    }

    #[test]
    fn convex_hull_collider_round_trips_through_serde() {
        let points: Arc<[Vec3]> = vec![
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ]
        .into();
        let collider = Collider::convex_hull(points);
        match &collider.shape {
            ColliderShape::ConvexHull { points } => assert_eq!(points.len(), 4),
            other => panic!("unexpected shape {other:?}"),
        }

        let json = serde_json::to_string(&collider).expect("serialize collider");
        let decoded: Collider = serde_json::from_str(&json).expect("deserialize collider");
        assert_eq!(decoded, collider);
    }

    #[test]
    fn passive_joint_dynamics_require_finite_non_negative_coefficients() {
        assert!(JointPassiveDynamics::Revolute {
            viscous_damping_nm_s_per_rad: 2.5,
            coulomb_friction_nm: 0.4,
            coulomb_transition_velocity_rad_s: 0.01,
        }
        .has_valid_values());
        assert!(!JointPassiveDynamics::Prismatic {
            viscous_damping_n_s_per_m: f64::NAN,
            coulomb_friction_n: 0.0,
            coulomb_transition_velocity_m_s: 0.0,
        }
        .has_valid_values());
        assert!(!JointPassiveDynamics::Revolute {
            viscous_damping_nm_s_per_rad: 0.0,
            coulomb_friction_nm: -0.1,
            coulomb_transition_velocity_rad_s: 0.01,
        }
        .has_valid_values());
        assert!(!JointPassiveDynamics::Revolute {
            viscous_damping_nm_s_per_rad: 0.0,
            coulomb_friction_nm: 0.1,
            coulomb_transition_velocity_rad_s: 0.0,
        }
        .has_valid_values());
    }

    #[test]
    fn regularized_coulomb_effort_is_smooth_bounded_and_opposes_motion() {
        let dynamics = JointPassiveDynamics::Revolute {
            viscous_damping_nm_s_per_rad: 0.0,
            coulomb_friction_nm: 0.4,
            coulomb_transition_velocity_rad_s: 0.02,
        };
        assert_eq!(dynamics.regularized_coulomb_effort(0.0), 0.0);
        let positive = dynamics.regularized_coulomb_effort(0.02);
        let negative = dynamics.regularized_coulomb_effort(-0.02);
        assert!(positive < 0.0);
        assert_eq!(positive, -negative);
        assert!(positive.abs() < 0.4);
        assert!(dynamics.regularized_coulomb_effort(1.0).abs() <= 0.4);
    }

    #[test]
    fn exact_inertia_requires_positive_definite_and_physically_realizable_tensor() {
        let valid = RigidBodyInertia {
            center_of_mass_local_m: Vec3::new(0.01, -0.02, 0.03),
            ixx_kg_m2: 0.4,
            ixy_kg_m2: 0.01,
            ixz_kg_m2: -0.02,
            iyy_kg_m2: 0.5,
            iyz_kg_m2: 0.03,
            izz_kg_m2: 0.6,
        };
        assert!(valid.is_valid());
        assert!(!RigidBodyInertia {
            iyy_kg_m2: -0.5,
            ..valid
        }
        .is_valid());
        assert!(!RigidBodyInertia {
            ixx_kg_m2: 1.0,
            ixy_kg_m2: 0.0,
            ixz_kg_m2: 0.0,
            iyy_kg_m2: 1.0,
            iyz_kg_m2: 0.0,
            izz_kg_m2: 3.0,
            ..valid
        }
        .is_valid());
    }

    #[test]
    fn joint_actuation_has_explicit_units_and_fail_closed_values() {
        let command = JointActuation::PrismaticVelocity {
            target_velocity_m_s: 0.5,
            gain_n_s_per_m: 12.0,
            max_force_n: 40.0,
        };
        assert!(command.has_valid_values());
        assert!(command.supports_prismatic());
        assert!(!command.supports_revolute());
        assert_eq!(
            serde_json::to_value(command).unwrap(),
            serde_json::json!({
                "mode": "prismatic_velocity",
                "target_velocity_m_s": 0.5,
                "gain_n_s_per_m": 12.0,
                "max_force_n": 40.0,
            })
        );

        assert!(!JointActuation::RevoluteEffort {
            effort_nm: f64::NAN,
            max_effort_nm: 10.0,
        }
        .has_valid_values());
        assert!(!JointActuation::PrismaticPosition {
            target_position_m: 0.0,
            stiffness_n_per_m: -1.0,
            damping_n_s_per_m: 1.0,
            max_force_n: 10.0,
        }
        .has_valid_values());
    }
}
