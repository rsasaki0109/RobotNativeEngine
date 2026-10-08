//! Articulated assemblies mirrored from the ECS into `rne_dynamics` models.

use crate::collide::{collider_samples, mesh_trees, sim_from_world, world_from_sim, Sample};
use crate::mesh::MeshTree;
use rne_dynamics::{ArticulatedModel, CoupledBody, JointPdControl};
use rne_ecs::{Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, CollisionGroups, FixedJointDesc, JointActuation, JointMotor,
    JointPassiveDynamics, PhysicsError, PrismaticJointDesc, RevoluteJointDesc, RigidBody,
    RigidBodyInertia, RigidBodyType,
};
use rne_robot::{FloatingBase, Joint, JointKind, JointLimits, Link, Robot, RobotId};
use rne_world::{world_transform_of, Transform3};
use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;

/// Radius of the inertia sphere assumed for a body with neither an exact
/// inertia nor a collider, in meters.
const DEFAULT_INERTIA_RADIUS_M: f64 = 0.05;

/// Joint type mirrored from a physics joint description.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) enum DescKind {
    Revolute,
    Prismatic,
    Fixed,
}

/// A physics joint description on a child body, in a common form.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct JointDesc {
    pub parent: Entity,
    pub kind: DescKind,
    pub axis: Vec3,
    pub anchor_parent_m: Vec3,
    pub anchor_child_m: Vec3,
    pub relative_rotation: Quat,
    pub lower: Option<f64>,
    pub upper: Option<f64>,
}

impl JointDesc {
    /// Reads the joint description of `entity`, if any.
    pub(crate) fn of(world: &World, entity: Entity) -> Option<Self> {
        if let Some(desc) = world.get::<RevoluteJointDesc>(entity) {
            return Some(Self {
                parent: desc.parent,
                kind: DescKind::Revolute,
                axis: desc.axis.try_normalize().unwrap_or(Vec3::X),
                anchor_parent_m: desc.anchor_parent_m,
                anchor_child_m: desc.anchor_child_m,
                relative_rotation: desc.relative_rotation,
                lower: desc.lower_rad,
                upper: desc.upper_rad,
            });
        }
        if let Some(desc) = world.get::<PrismaticJointDesc>(entity) {
            return Some(Self {
                parent: desc.parent,
                kind: DescKind::Prismatic,
                axis: desc.axis.try_normalize().unwrap_or(Vec3::X),
                anchor_parent_m: desc.anchor_parent_m,
                anchor_child_m: desc.anchor_child_m,
                relative_rotation: desc.relative_rotation,
                lower: desc.lower_m,
                upper: desc.upper_m,
            });
        }
        world.get::<FixedJointDesc>(entity).map(|desc| Self {
            parent: desc.parent,
            kind: DescKind::Fixed,
            axis: Vec3::X,
            anchor_parent_m: desc.anchor_parent_m,
            anchor_child_m: desc.anchor_child_m,
            relative_rotation: desc.relative_rotation,
            lower: None,
            upper: None,
        })
    }

    /// Joint frame relative to the parent body at zero displacement.
    fn frame(&self) -> Transform3 {
        Transform3::from_translation_rotation(self.anchor_parent_m, self.relative_rotation)
    }
}

/// One ECS rigid body of an assembly.
#[derive(Clone, Debug)]
pub(crate) struct AssemblyBody {
    /// ECS entity.
    pub entity: Entity,
    /// Link index in the model.
    pub link_index: usize,
    /// Model link entity (in the private model world).
    pub model_link: Entity,
    /// Collider samples in the body frame.
    pub samples: Vec<Sample>,
    /// Friction coefficient of the body's collider material.
    pub friction: f64,
    /// Collider shape and its offset in the body frame, for contacts that
    /// other bodies' samples make with this body.
    pub collider: Option<(ColliderShape, Transform3)>,
    /// Trees of the triangle meshes in the collider.
    pub meshes: Vec<Arc<MeshTree>>,
    /// Collision filtering masks.
    pub groups: CollisionGroups,
    /// Joint description connecting the body to its parent, if any.
    pub joint: Option<JointDesc>,
}

/// One driven joint coordinate of an assembly.
#[derive(Clone, Copy, Debug)]
pub(crate) struct AssemblyJoint {
    /// Index into [`Assembly::bodies`] of the child body.
    pub body: usize,
    /// Actuated coordinate index.
    pub dof: usize,
    /// Whether the coordinate is prismatic.
    pub prismatic: bool,
}

/// One contact of the last step, for contact events.
#[derive(Clone, Copy, Debug)]
pub(crate) struct ContactRecord {
    /// Body whose collider sample made the contact.
    pub body_entity: Entity,
    /// Static or dynamic body it touched.
    pub other_entity: Entity,
    pub normal: Vec3,
    pub gap_m: f64,
    pub normal_impulse_n_s: f64,
}

/// A tree of dynamic bodies simulated as one articulated model.
#[derive(Clone, Debug)]
pub(crate) struct Assembly {
    model: ArticulatedModel,
    floating: bool,
    /// Bodies in parent-first order; the first is the root when floating.
    pub bodies: Vec<AssemblyBody>,
    pub joints: Vec<AssemblyJoint>,
    pub q: Vec<f64>,
    pub qd: Vec<f64>,
    pd: JointPdControl,
    tau: Vec<f64>,
    /// World pose and velocities last written to each body.
    pub written: Vec<(Transform3, Vec3, Vec3)>,
}

impl Assembly {
    /// Builds the assembly of `members` (parent-first ECS bodies). When the
    /// first member's joint attaches it to a non-dynamic body, the assembly has
    /// a fixed base posed where that body stands now; otherwise it floats.
    pub(crate) fn build(
        world: &World,
        members: &[Entity],
        gravity_sim_m_s2: Vec3,
        mesh_cache: &mut Vec<Arc<MeshTree>>,
    ) -> Result<Self, PhysicsError> {
        let root = members[0];
        let root_joint = JointDesc::of(world, root);
        let floating = root_joint.is_none();

        let mut model_world = World::new();
        let robot = model_world.spawn_empty().id();
        let mut order = 0_usize;
        let mut next_name = || {
            order += 1;
            format!("link_{order:06}")
        };
        let base_link = model_world
            .spawn((
                Link {
                    robot,
                    name: next_name(),
                },
                Transform3::IDENTITY,
            ))
            .id();
        model_world.entity_mut(robot).insert(Robot {
            robot_id: RobotId(uuid::Uuid::nil()),
            model_name: "rne_physics_native".to_owned(),
            base_link,
        });

        let mut model_links: HashMap<Entity, Entity> = HashMap::new();
        let mut bodies = Vec::with_capacity(members.len());
        if floating {
            model_world.entity_mut(base_link).insert(FloatingBase);
            insert_inertia(&mut model_world, base_link, world, root)?;
            model_links.insert(root, base_link);
        } else {
            // The base is the anchor body, fixed where it stands.
            let anchor = root_joint.expect("a fixed base comes from a joint").parent;
            let pose = sim_from_world(&world_transform_of(world, anchor));
            model_world.entity_mut(base_link).insert(pose);
        }

        let mut joint_entities: Vec<(usize, Entity)> = Vec::new();
        for &entity in members {
            let joint = JointDesc::of(world, entity);
            let model_link = match joint {
                None => base_link,
                Some(desc) => {
                    let parent_link = model_links.get(&desc.parent).copied().unwrap_or(base_link);
                    let link = mirror_joint(
                        &mut model_world,
                        robot,
                        parent_link,
                        &desc,
                        &mut next_name,
                        &mut joint_entities,
                        bodies.len(),
                    );
                    insert_inertia(&mut model_world, link, world, entity)?;
                    link
                }
            };
            model_links.insert(entity, model_link);
            let (samples, friction, collider) = body_samples(world, entity);
            let mut meshes = Vec::new();
            if let Some((shape, _)) = &collider {
                mesh_trees(shape, mesh_cache, &mut meshes);
            }
            bodies.push(AssemblyBody {
                entity,
                link_index: 0,
                model_link,
                samples,
                friction,
                collider,
                meshes,
                groups: world
                    .get::<CollisionGroups>(entity)
                    .copied()
                    .unwrap_or_default(),
                joint,
            });
        }

        let model =
            ArticulatedModel::from_robot_with_gravity(&model_world, robot, gravity_sim_m_s2)
                .map_err(|_| PhysicsError::InitializationFailed)?;
        for body in &mut bodies {
            body.link_index = model
                .kinematic()
                .link_index(body.model_link)
                .ok_or(PhysicsError::InitializationFailed)?;
        }
        let mut joints = Vec::new();
        for (body, joint_entity) in joint_entities {
            let dof = model
                .kinematic()
                .dof_index_of_joint(joint_entity)
                .ok_or(PhysicsError::InitializationFailed)?;
            let prismatic = bodies[body]
                .joint
                .is_some_and(|desc| desc.kind == DescKind::Prismatic);
            joints.push(AssemblyJoint {
                body,
                dof,
                prismatic,
            });
        }
        let nv = model.nv();
        let actuated = nv - model.base_dof();
        let mut assembly = Self {
            model,
            floating,
            written: vec![(Transform3::IDENTITY, Vec3::ZERO, Vec3::ZERO); bodies.len()],
            bodies,
            joints,
            q: vec![0.0; nv],
            qd: vec![0.0; nv],
            pd: JointPdControl {
                position_gains: vec![0.0; actuated],
                velocity_gains: vec![0.0; actuated],
                target_positions: vec![0.0; actuated],
                target_velocities: vec![0.0; actuated],
                effort_limits: vec![f64::INFINITY; actuated],
            },
            tau: vec![0.0; nv],
        };
        assembly.load_state(world);
        Ok(assembly)
    }

    /// Sets the configuration and velocity from the ECS poses and rigid-body
    /// velocities.
    pub(crate) fn load_state(&mut self, world: &World) {
        let base = self.model.base_dof();
        self.q.iter_mut().for_each(|value| *value = 0.0);
        self.qd.iter_mut().for_each(|value| *value = 0.0);
        let to_sim = crate::collide::world_to_sim();
        let velocity = |entity: Entity| {
            world
                .get::<RigidBody>(entity)
                .map_or((Vec3::ZERO, Vec3::ZERO), |body| {
                    (
                        to_sim * body.linear_velocity_m_s,
                        to_sim * body.angular_velocity_rad_s,
                    )
                })
        };
        if self.floating {
            let root = self.bodies[0].entity;
            let pose = sim_from_world(&world_transform_of(world, root));
            let [roll, pitch, yaw] = roll_pitch_yaw(pose.rotation);
            self.q[..6].copy_from_slice(&[
                pose.translation.x,
                pose.translation.y,
                pose.translation.z,
                roll,
                pitch,
                yaw,
            ]);
            let (linear, angular) = velocity(root);
            let body_linear = pose.rotation.conjugate() * linear;
            let body_angular = pose.rotation.conjugate() * angular;
            self.qd[..6].copy_from_slice(&[
                body_linear.x,
                body_linear.y,
                body_linear.z,
                body_angular.x,
                body_angular.y,
                body_angular.z,
            ]);
        }
        for joint in &self.joints {
            let body = &self.bodies[joint.body];
            let desc = body.joint.expect("a driven joint has a description");
            let parent = sim_from_world(&world_transform_of(world, desc.parent));
            let child = sim_from_world(&world_transform_of(world, body.entity));
            let frame = parent.mul_transform(&desc.frame());
            let axis_sim = frame.rotation * desc.axis;
            let (parent_linear, parent_angular) = velocity(desc.parent);
            let (child_linear, child_angular) = velocity(body.entity);
            let (position, rate) = if joint.prismatic {
                let offset = frame.rotation.conjugate() * (child.translation - frame.translation)
                    + desc.anchor_child_m;
                (
                    offset.dot(desc.axis),
                    axis_sim.dot(child_linear - parent_linear),
                )
            } else {
                let relative = frame.rotation.conjugate() * child.rotation;
                let twist = Vec3::new(relative.x, relative.y, relative.z).dot(desc.axis);
                (
                    2.0 * twist.atan2(relative.w),
                    axis_sim.dot(child_angular - parent_angular),
                )
            };
            self.q[base + joint.dof] = wrap_revolute(position, joint.prismatic);
            self.qd[base + joint.dof] = rate;
        }
    }

    /// Reads the joint commands of every driven joint.
    pub(crate) fn load_commands(&mut self, world: &World) -> Result<(), PhysicsError> {
        let base = self.model.base_dof();
        self.tau.iter_mut().for_each(|value| *value = 0.0);
        for joint in &self.joints {
            let entity = self.bodies[joint.body].entity;
            let index = joint.dof;
            let mut command = JointCommand::from_world(world, entity, joint.prismatic)?;
            if let Some(passive) = world.get::<JointPassiveDynamics>(entity).copied() {
                let matches_kind = matches!(
                    (passive, joint.prismatic),
                    (JointPassiveDynamics::Revolute { .. }, false)
                        | (JointPassiveDynamics::Prismatic { .. }, true)
                );
                if !passive.has_valid_values() || !matches_kind {
                    return Err(PhysicsError::InvalidPassiveDynamics {
                        entity_index: entity.index(),
                        reason: "passive dynamics must be finite, non-negative, and match the joint type",
                    });
                }
                let damping = match passive {
                    JointPassiveDynamics::Revolute {
                        viscous_damping_nm_s_per_rad,
                        ..
                    } => viscous_damping_nm_s_per_rad,
                    JointPassiveDynamics::Prismatic {
                        viscous_damping_n_s_per_m,
                        ..
                    } => viscous_damping_n_s_per_m,
                };
                // Viscous damping is a damper toward zero velocity, folded into
                // the implicit PD; Coulomb friction is an explicit force.
                let gain = command.damping + damping;
                if gain > 0.0 {
                    command.target_velocity = command.damping * command.target_velocity / gain;
                }
                command.damping = gain;
                self.tau[base + index] += passive.regularized_coulomb_effort(self.qd[base + index]);
            }
            self.pd.position_gains[index] = command.stiffness;
            self.pd.velocity_gains[index] = command.damping;
            self.pd.target_positions[index] = command.target_position;
            self.pd.target_velocities[index] = command.target_velocity;
            self.pd.effort_limits[index] = command.max_effort;
            self.tau[base + index] += command.effort;
        }
        Ok(())
    }

    /// The assembly's model, state, and commands for a coupled step.
    pub(crate) fn coupled_body(&self) -> CoupledBody<'_> {
        CoupledBody {
            model: &self.model,
            q: &self.q,
            qd: &self.qd,
            tau: &self.tau,
            pd: (!self.joints.is_empty()).then_some(&self.pd),
        }
    }

    /// Link poses in the simulation frame at the current configuration.
    pub(crate) fn sim_transforms(&self) -> Result<Vec<Transform3>, PhysicsError> {
        Ok(self
            .model
            .kinematic()
            .forward_kinematics(&self.q)
            .map_err(|_| PhysicsError::InitializationFailed)?
            .transforms()
            .to_vec())
    }

    /// Whether bodies `first` and `second` of this assembly are joined
    /// directly by a joint, which keeps them from colliding with each other.
    pub(crate) fn adjacent(&self, first: usize, second: usize) -> bool {
        let parent_of = |index: usize| self.bodies[index].joint.map(|desc| desc.parent);
        parent_of(first) == Some(self.bodies[second].entity)
            || parent_of(second) == Some(self.bodies[first].entity)
    }

    /// World pose of every body at the current configuration.
    pub(crate) fn body_poses(&self) -> Result<Vec<Transform3>, PhysicsError> {
        let kinematics = self
            .model
            .kinematic()
            .forward_kinematics(&self.q)
            .map_err(|_| PhysicsError::InitializationFailed)?;
        Ok(self
            .bodies
            .iter()
            .map(|body| world_from_sim(&kinematics.transforms()[body.link_index]))
            .collect())
    }

    /// Position and rate of the driven joint on `entity`.
    pub(crate) fn joint_state(&self, entity: Entity) -> Option<(f64, f64, bool)> {
        let base = self.model.base_dof();
        self.joints
            .iter()
            .find(|joint| self.bodies[joint.body].entity == entity)
            .map(|joint| {
                (
                    self.q[base + joint.dof],
                    self.qd[base + joint.dof],
                    joint.prismatic,
                )
            })
    }
}

/// Mirrors one physics joint into the model world and returns the child link.
fn mirror_joint(
    model_world: &mut World,
    robot: Entity,
    parent_link: Entity,
    desc: &JointDesc,
    next_name: &mut impl FnMut() -> String,
    joint_entities: &mut Vec<(usize, Entity)>,
    body_index: usize,
) -> Entity {
    let frame = desc.frame();
    let offset = Transform3::from_translation_rotation(-desc.anchor_child_m, Quat::IDENTITY);
    let spawn_joint = |world: &mut World, parent: Entity, child: Entity, kind, limits, axis| {
        world
            .spawn(Joint {
                robot,
                parent_link: parent,
                child_link: child,
                kind,
                limits,
                axis,
                position: 0.0,
                velocity: 0.0,
            })
            .id()
    };
    if desc.kind == DescKind::Fixed {
        let link = model_world
            .spawn((
                Link {
                    robot,
                    name: next_name(),
                },
                frame.mul_transform(&offset),
            ))
            .id();
        spawn_joint(
            model_world,
            parent_link,
            link,
            JointKind::Fixed,
            JointLimits::default(),
            Vec3::X,
        );
        return link;
    }
    let kind = match (desc.kind, desc.lower, desc.upper) {
        (DescKind::Prismatic, ..) => JointKind::Prismatic,
        (_, None, None) => JointKind::Continuous,
        _ => JointKind::Revolute,
    };
    let limits = JointLimits {
        lower: desc.lower.unwrap_or(f64::NEG_INFINITY),
        upper: desc.upper.unwrap_or(f64::INFINITY),
        ..JointLimits::default()
    };
    // The joint moves its own frame; a child anchored away from its origin
    // hangs off that frame through a weld.
    let has_offset = desc.anchor_child_m.length_squared() > 0.0;
    let joint_link = model_world
        .spawn((
            Link {
                robot,
                name: next_name(),
            },
            frame,
        ))
        .id();
    let joint = spawn_joint(
        model_world,
        parent_link,
        joint_link,
        kind,
        limits,
        desc.axis,
    );
    joint_entities.push((body_index, joint));
    if !has_offset {
        return joint_link;
    }
    let link = model_world
        .spawn((
            Link {
                robot,
                name: next_name(),
            },
            offset,
        ))
        .id();
    spawn_joint(
        model_world,
        joint_link,
        link,
        JointKind::Fixed,
        JointLimits::default(),
        Vec3::X,
    );
    link
}

/// Gives a model link the mass and inertia of ECS body `source`.
fn insert_inertia(
    model_world: &mut World,
    link: Entity,
    world: &World,
    source: Entity,
) -> Result<(), PhysicsError> {
    let mass_kg = world
        .get::<RigidBody>(source)
        .map_or(0.0, |body| body.mass_kg);
    if !(mass_kg.is_finite() && mass_kg > 0.0) {
        return Err(PhysicsError::InvalidInertia {
            entity_index: source.index(),
            reason: "a dynamic body needs a positive finite mass",
        });
    }
    let inertia = match world.get::<RigidBodyInertia>(source).copied() {
        Some(inertia) if inertia.is_valid() => inertia,
        Some(_) => {
            return Err(PhysicsError::InvalidInertia {
                entity_index: source.index(),
                reason: "rigid-body inertia must be finite and physically valid",
            })
        }
        None => shape_inertia(mass_kg, world.get::<Collider>(source)),
    };
    model_world.entity_mut(link).insert((
        RigidBody {
            body_type: RigidBodyType::Dynamic,
            mass_kg,
            ..RigidBody::default()
        },
        inertia,
    ));
    Ok(())
}

/// Solid-shape inertia of a body's collider about its center.
fn shape_inertia(mass_kg: f64, collider: Option<&Collider>) -> RigidBodyInertia {
    let sphere = |radius: f64| [0.4 * mass_kg * radius * radius; 3];
    let (center, rotation, diagonal) = match collider {
        Some(collider) => {
            let offset = collider.local_offset;
            let diagonal = match &collider.shape {
                ColliderShape::Sphere { radius_m } => sphere(*radius_m),
                ColliderShape::Cuboid { half_extents_m } => {
                    let size = *half_extents_m * 2.0;
                    let sq = size * size;
                    [
                        mass_kg * (sq.y + sq.z) / 12.0,
                        mass_kg * (sq.x + sq.z) / 12.0,
                        mass_kg * (sq.x + sq.y) / 12.0,
                    ]
                }
                ColliderShape::Capsule {
                    half_height_m,
                    radius_m,
                } => {
                    // A solid cylinder spanning the capsule's length.
                    let length = 2.0 * (half_height_m + radius_m);
                    let side = mass_kg * (3.0 * radius_m * radius_m + length * length) / 12.0;
                    [side, 0.5 * mass_kg * radius_m * radius_m, side]
                }
                _ => sphere(DEFAULT_INERTIA_RADIUS_M),
            };
            (offset.translation, offset.rotation, diagonal)
        }
        None => (Vec3::ZERO, Quat::IDENTITY, sphere(DEFAULT_INERTIA_RADIUS_M)),
    };
    // I = R diag(d) Rᵀ.
    let axes = [rotation * Vec3::X, rotation * Vec3::Y, rotation * Vec3::Z];
    let entry =
        |i: usize, j: usize| -> f64 { (0..3).map(|k| diagonal[k] * axes[k][i] * axes[k][j]).sum() };
    RigidBodyInertia {
        center_of_mass_local_m: center,
        ixx_kg_m2: entry(0, 0),
        ixy_kg_m2: entry(0, 1),
        ixz_kg_m2: entry(0, 2),
        iyy_kg_m2: entry(1, 1),
        iyz_kg_m2: entry(1, 2),
        izz_kg_m2: entry(2, 2),
    }
}

/// Collider samples, friction, and shape of an ECS body.
#[allow(clippy::type_complexity)]
fn body_samples(
    world: &World,
    entity: Entity,
) -> (Vec<Sample>, f64, Option<(ColliderShape, Transform3)>) {
    let mut samples = Vec::new();
    match world.get::<Collider>(entity) {
        Some(collider) if !collider.sensor => {
            collider_samples(&collider.shape, &collider.local_offset, &mut samples);
            (
                samples,
                f64::from(collider.material.friction),
                Some((collider.shape.clone(), collider.local_offset)),
            )
        }
        _ => (samples, 0.5, None),
    }
}

/// Fixed-axis roll, pitch, and yaw of `rotation = Rz(yaw) Ry(pitch) Rx(roll)`.
fn roll_pitch_yaw(rotation: Quat) -> [f64; 3] {
    let x = rotation * Vec3::X;
    let y = rotation * Vec3::Y;
    let z = rotation * Vec3::Z;
    let pitch = (-x.z).clamp(-1.0, 1.0).asin();
    [y.z.atan2(z.z), pitch, x.y.atan2(x.x)]
}

fn wrap_revolute(angle: f64, prismatic: bool) -> f64 {
    if prismatic {
        angle
    } else {
        (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
    }
}

/// A joint command in PD-plus-feed-forward form.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct JointCommand {
    stiffness: f64,
    damping: f64,
    target_position: f64,
    target_velocity: f64,
    effort: f64,
    max_effort: f64,
}

impl JointCommand {
    /// Reads `JointActuation`, or the legacy `JointMotor` when there is none.
    fn from_world(world: &World, entity: Entity, prismatic: bool) -> Result<Self, PhysicsError> {
        let invalid = |reason| PhysicsError::InvalidActuation {
            entity_index: entity.index(),
            reason,
        };
        let unbounded = Self {
            max_effort: f64::INFINITY,
            ..Self::default()
        };
        if let Some(actuation) = world.get::<JointActuation>(entity).copied() {
            if !actuation.has_valid_values() {
                return Err(invalid(
                    "joint actuation values must be finite and non-negative",
                ));
            }
            let supported = if prismatic {
                actuation.supports_prismatic()
            } else {
                actuation.supports_revolute()
            };
            if !supported {
                return Err(invalid("joint actuation does not match the joint type"));
            }
            return Ok(match actuation {
                JointActuation::Disabled => unbounded,
                JointActuation::RevolutePosition {
                    target_position_rad: target,
                    stiffness_nm_per_rad: stiffness,
                    damping_nm_s_per_rad: damping,
                    max_effort_nm: max,
                }
                | JointActuation::PrismaticPosition {
                    target_position_m: target,
                    stiffness_n_per_m: stiffness,
                    damping_n_s_per_m: damping,
                    max_force_n: max,
                } => Self {
                    stiffness,
                    damping,
                    target_position: target,
                    max_effort: max,
                    ..Self::default()
                },
                JointActuation::RevoluteVelocity {
                    target_velocity_rad_s: target,
                    gain_nm_s_per_rad: gain,
                    max_effort_nm: max,
                }
                | JointActuation::PrismaticVelocity {
                    target_velocity_m_s: target,
                    gain_n_s_per_m: gain,
                    max_force_n: max,
                } => Self {
                    damping: gain,
                    target_velocity: target,
                    max_effort: max,
                    ..Self::default()
                },
                JointActuation::RevoluteEffort {
                    effort_nm: effort,
                    max_effort_nm: max,
                }
                | JointActuation::PrismaticEffort {
                    force_n: effort,
                    max_force_n: max,
                } => Self {
                    effort,
                    max_effort: max,
                    ..Self::default()
                },
            });
        }
        if let Some(motor) = world.get::<JointMotor>(entity).copied() {
            let values = [
                motor.velocity_rad_s,
                motor.gain,
                motor.stiffness,
                motor.target_position,
                motor.max_force,
            ];
            if values.iter().any(|value| !value.is_finite())
                || motor.gain < 0.0
                || motor.stiffness < 0.0
                || motor.max_force < 0.0
            {
                return Err(invalid(
                    "joint motor values must be finite and non-negative",
                ));
            }
            return Ok(Self {
                stiffness: motor.stiffness,
                damping: motor.gain,
                target_position: motor.target_position,
                target_velocity: motor.velocity_rad_s,
                effort: 0.0,
                max_effort: if motor.max_force > 0.0 {
                    motor.max_force
                } else {
                    f64::INFINITY
                },
            });
        }
        Ok(unbounded)
    }
}

/// Groups dynamic bodies into assemblies: each tree of dynamic bodies joined
/// by joint descriptions, rooted at a body with no joint or with a joint to a
/// non-dynamic body. Members are listed parent-first, siblings by entity index.
pub(crate) fn group_assemblies(world: &World, dynamic: &[Entity]) -> Vec<Vec<Entity>> {
    let is_dynamic = |entity: Entity| {
        dynamic
            .binary_search_by_key(&entity.index(), |candidate| candidate.index())
            .is_ok()
    };
    let mut children: BTreeMap<u32, Vec<Entity>> = BTreeMap::new();
    let mut roots = Vec::new();
    for &entity in dynamic {
        match JointDesc::of(world, entity) {
            Some(desc) if is_dynamic(desc.parent) && desc.parent != entity => {
                children
                    .entry(desc.parent.index())
                    .or_default()
                    .push(entity);
            }
            _ => roots.push(entity),
        }
    }
    roots
        .into_iter()
        .map(|root| {
            let mut members = vec![root];
            let mut cursor = 0;
            while cursor < members.len() {
                if let Some(kids) = children.get(&members[cursor].index()) {
                    members.extend(kids.iter().copied());
                }
                cursor += 1;
            }
            members
        })
        .collect()
}
