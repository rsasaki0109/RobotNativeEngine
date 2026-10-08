//! Native articulated physics backend for Robot Native Engine.
//!
//! [`NativeBackend`] implements [`PhysicsBackend`] on top of
//! [`rne_dynamics::contact_step_coupled`], the RaiSim-style hard-contact step: every
//! tree of dynamic bodies joined by [`rne_physics::RevoluteJointDesc`],
//! [`rne_physics::PrismaticJointDesc`], or [`rne_physics::FixedJointDesc`] is
//! simulated as one reduced-coordinate articulated model (a lone dynamic body
//! is a one-body model with a floating base), contacts are resolved with the
//! exact Coulomb cone by per-contact bisection, and joint commands are folded
//! in as implicit PD. A URDF robot attached with
//! `rne_urdf_import::attach_urdf_document_articulation` runs on it unchanged.
//!
//! # What it simulates
//!
//! - **Bodies.** Dynamic [`RigidBody`] entities, with their
//!   [`rne_physics::RigidBodyInertia`] or, without one, the solid inertia of
//!   their collider. A tree whose root joint attaches to a fixed or kinematic
//!   body has a fixed base that follows that body: each sync moves the base
//!   to where the body stands, so an arm on a kinematic cart rides along. The
//!   cart's velocity and acceleration do not enter the arm's dynamics.
//! - **Contacts.** Moving colliders are sampled — spheres exactly, capsules as
//!   a row of spheres, boxes, convex hulls, and triangle meshes by their
//!   vertices — against the planes, boxes, spheres, capsules, height fields,
//!   triangle meshes, convex hulls, and compounds of fixed and kinematic
//!   bodies, and against the spheres, boxes, capsules, triangle meshes, convex
//!   hulls, and compounds of other dynamic bodies, including other links of the same robot (except
//!   the two sides of a joint). Triangle meshes wind counter-clockwise seen
//!   from outside, and each gets a bounding-volume tree; a convex hull is
//!   built from its points once, with its face planes. A box against a box instead runs the full
//!   separating-axis test, so crossing edges touch too.
//!   [`rne_physics::CollisionGroups`] filter every pair.
//!   Assemblies that touch are solved in one contact problem, as RaiSim solves
//!   a world; the rest are solved one island each. Contacts between two
//!   entities along one normal are reduced to four that span their support. Friction is the mean of
//!   the two materials' coefficients; restitution is ignored (contacts are
//!   inelastic).
//! - **Actuation.** [`rne_physics::JointActuation`] position, velocity, and
//!   effort commands become implicit PD gains or feed-forward forces with their
//!   effort limits, the legacy [`rne_physics::JointMotor`] is read with
//!   force-based gains when no actuation is present, and
//!   [`rne_physics::JointPassiveDynamics`] adds viscous damping and
//!   regularized Coulomb friction.
//!
//! The simulation runs in a Z-up frame (RNE's world rotated +90° about X) so
//! each floating base's heading is its yaw coordinate; poses, velocities, and
//! [`ContactEvent`]s are converted back to the Y-up world on output.
//!
//! An ECS pose or velocity edit on a simulated body (other than the backend's
//! own write-back) re-reads that assembly's state from the ECS, and adding or
//! removing bodies or joints rebuilds the assemblies.
//!
//! Raycasts hit every non-sensor collider at its current pose and return the
//! hits by distance. Spheres, capsules, boxes, planes, and convex hulls are
//! solid (a ray that starts inside one hits it at distance zero with a zero
//! normal, as in the Rapier backend); height fields and triangle meshes are
//! surfaces.

#![deny(missing_docs)]

mod assembly;
mod collide;
mod hull;
mod mesh;
mod raycast;
mod world_step;

use assembly::{group_assemblies, Assembly, JointDesc};
use collide::{mesh_trees, sim_from_world, world_to_sim, StaticCollider};
use mesh::MeshTree;
use rne_core::SimDuration;
use rne_dynamics::ContactStepConfig;
use rne_ecs::{Entity, Parent, World};
use rne_math::Vec3;
use rne_physics::{
    Collider, ColliderShape, ContactEvent, JointState, PhysicsBackend, PhysicsBackendManifest,
    PhysicsBackendRepeatability, PhysicsCapability, PhysicsError, PhysicsWorldDesc, PhysicsWorldId,
    RaycastHit, RaycastQuery, RigidBody, RigidBodyType,
};
use rne_world::{world_transform_of, Transform3};
use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;
use world_step::{step_world, WarmKey};

/// Capabilities of [`NativeBackend`], in [`PhysicsCapability::ALL`] order.
const CAPABILITIES: &[PhysicsCapability] = &[
    PhysicsCapability::RigidBody,
    PhysicsCapability::Articulation,
    PhysicsCapability::DeterministicStep,
    PhysicsCapability::ContactForce,
    PhysicsCapability::RaycastBatch,
];

/// Largest pose or velocity change that still counts as the backend's own
/// write-back rather than an external edit.
const EDIT_TOLERANCE: f64 = 1.0e-12;

/// Settings of [`NativeBackend`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NativeBackendConfig {
    /// Gap below which a sample becomes a contact candidate, in meters.
    /// Candidates close their gap within a step before they push.
    pub contact_margin_m: f64,
    /// Contact step settings; its step time is replaced by each step's `dt`.
    pub contact: ContactStepConfig,
}

impl Default for NativeBackendConfig {
    fn default() -> Self {
        Self {
            contact_margin_m: 0.02,
            contact: ContactStepConfig::default(),
        }
    }
}

/// Topology entry of one rigid body: entity, type, and joint parent and kind.
type TopologyEntry = (Entity, RigidBodyType, Option<(Entity, u8)>);

#[derive(Debug, Default)]
struct NativeWorld {
    gravity_m_s2: Vec3,
    topology: Vec<TopologyEntry>,
    assemblies: Vec<Assembly>,
    statics: Vec<StaticCollider>,
    /// Trees of the triangle meshes in use, found again by their allocations.
    mesh_cache: Vec<Arc<MeshTree>>,
    contacts: Vec<ContactEvent>,
    /// Contact impulses of the last step in contact frames, for warm starts.
    warm: HashMap<WarmKey, [f64; 3]>,
    /// Assembly index of each simulated body entity.
    body_assembly: HashMap<Entity, usize>,
    last_dt_s: Option<f64>,
}

/// Physics backend that simulates articulated assemblies with
/// [`rne_dynamics::contact_step_coupled`]. See the crate documentation.
#[derive(Debug, Default)]
pub struct NativeBackend {
    config: NativeBackendConfig,
    worlds: HashMap<PhysicsWorldId, NativeWorld>,
    next_world_id: u32,
}

impl NativeBackend {
    /// Creates a backend with default settings.
    pub fn new() -> Self {
        Self::default()
    }

    /// Creates a backend with `config`.
    pub fn with_config(config: NativeBackendConfig) -> Self {
        Self {
            config,
            ..Self::default()
        }
    }

    /// Returns the versioned conformance manifest for this backend.
    pub fn manifest() -> PhysicsBackendManifest {
        PhysicsBackendManifest::new(
            "native",
            env!("CARGO_PKG_VERSION"),
            "rne_dynamics",
            "coupled_contact_step_per_contact_bisection_v2",
            CAPABILITIES.iter().copied(),
            PhysicsBackendRepeatability::SameRuntimeExact,
        )
        .expect("the native backend manifest is valid")
    }

    /// Number of articulated assemblies in `physics_world` after the last
    /// sync.
    pub fn assembly_count(&self, physics_world: PhysicsWorldId) -> Result<usize, PhysicsError> {
        Ok(self.world(physics_world)?.assemblies.len())
    }

    fn world(&self, physics_world: PhysicsWorldId) -> Result<&NativeWorld, PhysicsError> {
        self.worlds
            .get(&physics_world)
            .ok_or(PhysicsError::WorldNotFound)
    }

    fn world_mut(
        &mut self,
        physics_world: PhysicsWorldId,
    ) -> Result<&mut NativeWorld, PhysicsError> {
        self.worlds
            .get_mut(&physics_world)
            .ok_or(PhysicsError::WorldNotFound)
    }
}

impl NativeWorld {
    fn rebuild(&mut self, world: &World, dynamic: &[Entity]) -> Result<(), PhysicsError> {
        let gravity = world_to_sim() * self.gravity_m_s2;
        self.warm.clear();
        self.assemblies = group_assemblies(world, dynamic)
            .iter()
            .map(|members| Assembly::build(world, members, gravity, &mut self.mesh_cache))
            .collect::<Result<_, _>>()?;
        self.body_assembly = self
            .assemblies
            .iter()
            .enumerate()
            .flat_map(|(index, assembly)| {
                assembly.bodies.iter().map(move |body| (body.entity, index))
            })
            .collect();
        for assembly in &mut self.assemblies {
            remember_ecs_state(assembly, world);
        }
        Ok(())
    }
}

/// Inverse of a rigid transform.
fn inverse(pose: &Transform3) -> Transform3 {
    let rotation = pose.rotation.conjugate();
    Transform3::from_translation_rotation(-(rotation * pose.translation), rotation)
}

/// Records the current ECS poses and velocities as the assembly's baseline
/// for edit detection.
fn remember_ecs_state(assembly: &mut Assembly, world: &World) {
    for (slot, body) in assembly.written.iter_mut().zip(&assembly.bodies) {
        let (linear, angular) = world
            .get::<RigidBody>(body.entity)
            .map_or((Vec3::ZERO, Vec3::ZERO), |rigid| {
                (rigid.linear_velocity_m_s, rigid.angular_velocity_rad_s)
            });
        *slot = (world_transform_of(world, body.entity), linear, angular);
    }
}

/// Whether an ECS body differs from what the backend last wrote.
fn edited(world: &World, entity: Entity, written: &(Transform3, Vec3, Vec3)) -> bool {
    let pose = world_transform_of(world, entity);
    let (linear, angular) = world
        .get::<RigidBody>(entity)
        .map_or((Vec3::ZERO, Vec3::ZERO), |rigid| {
            (rigid.linear_velocity_m_s, rigid.angular_velocity_rad_s)
        });
    (pose.translation - written.0.translation).length() > EDIT_TOLERANCE
        || pose.rotation.dot(written.0.rotation).abs() < 1.0 - EDIT_TOLERANCE
        || (linear - written.1).length() > EDIT_TOLERANCE
        || (angular - written.2).length() > EDIT_TOLERANCE
}

impl PhysicsBackend for NativeBackend {
    type BodyHandle = Entity;
    type ColliderHandle = Entity;

    fn create_world(&mut self, desc: PhysicsWorldDesc) -> Result<PhysicsWorldId, PhysicsError> {
        if !desc.gravity_m_s2.is_finite() {
            return Err(PhysicsError::InitializationFailed);
        }
        let id = PhysicsWorldId(self.next_world_id);
        self.next_world_id += 1;
        self.worlds.insert(
            id,
            NativeWorld {
                gravity_m_s2: desc.gravity_m_s2,
                ..NativeWorld::default()
            },
        );
        Ok(id)
    }

    fn sync_from_ecs(
        &mut self,
        world: &mut World,
        physics_world: PhysicsWorldId,
    ) -> Result<(), PhysicsError> {
        let state = self.world_mut(physics_world)?;
        let mut bodies: Vec<(Entity, RigidBodyType)> = world
            .iter_entities()
            .filter_map(|entity_ref| {
                entity_ref
                    .get::<RigidBody>()
                    .map(|body| (entity_ref.id(), body.body_type))
            })
            .collect();
        bodies.sort_unstable_by_key(|(entity, _)| entity.index());

        let topology: Vec<TopologyEntry> = bodies
            .iter()
            .map(|&(entity, body_type)| {
                let joint = JointDesc::of(world, entity).map(|desc| (desc.parent, desc.kind as u8));
                (entity, body_type, joint)
            })
            .collect();
        if topology != state.topology {
            let dynamic: Vec<Entity> = bodies
                .iter()
                .filter(|(_, body_type)| *body_type == RigidBodyType::Dynamic)
                .map(|(entity, _)| *entity)
                .collect();
            state.rebuild(world, &dynamic)?;
            state.topology = topology;
        } else {
            for assembly in &mut state.assemblies {
                let changed = assembly
                    .bodies
                    .iter()
                    .zip(&assembly.written)
                    .any(|(body, written)| edited(world, body.entity, written));
                if changed {
                    assembly.load_state(world);
                    remember_ecs_state(assembly, world);
                    state.warm.clear();
                }
            }
        }

        state.statics = bodies
            .iter()
            .filter(|(_, body_type)| *body_type != RigidBodyType::Dynamic)
            .filter_map(|&(entity, _)| {
                let collider = world.get::<Collider>(entity)?;
                if collider.sensor {
                    return None;
                }
                let pose = world_transform_of(world, entity).mul_transform(&collider.local_offset);
                let mut meshes = Vec::new();
                mesh_trees(&collider.shape, &mut state.mesh_cache, &mut meshes);
                Some(StaticCollider {
                    entity,
                    shape: collider.shape.clone(),
                    pose: sim_from_world(&pose),
                    friction: f64::from(collider.material.friction),
                    meshes,
                })
            })
            .collect();
        // Drop the trees no collider uses any more.
        state.mesh_cache.retain(|tree| Arc::strong_count(tree) > 1);
        for assembly in &mut state.assemblies {
            assembly.follow_anchor(world);
            assembly.load_commands(world)?;
        }
        Ok(())
    }

    fn step(&mut self, physics_world: PhysicsWorldId, dt: SimDuration) -> Result<(), PhysicsError> {
        let config = self.config;
        let dt_s = dt.as_seconds().value();
        if !(dt_s.is_finite() && dt_s > 0.0) {
            return Err(PhysicsError::InitializationFailed);
        }
        let state = self.world_mut(physics_world)?;
        let records = step_world(
            &mut state.assemblies,
            &state.statics,
            &mut state.warm,
            &config.contact,
            config.contact_margin_m,
            dt_s,
        )?;
        let mut events: BTreeMap<(u32, u32), ContactEvent> = BTreeMap::new();
        let to_world = world_to_sim().conjugate();
        for record in &records {
            if record.normal_impulse_n_s <= 0.0 && record.gap_m > 0.0 {
                continue;
            }
            // One event per pair, A being the lower entity index. The record's
            // normal points from the touched entity into the sampled body.
            let normal = to_world * record.normal;
            let (entity_a, entity_b, normal_a_to_b) =
                if record.body_entity.index() <= record.other_entity.index() {
                    (record.body_entity, record.other_entity, -normal)
                } else {
                    (record.other_entity, record.body_entity, normal)
                };
            let event = events
                .entry((entity_a.index(), entity_b.index()))
                .or_insert(ContactEvent {
                    entity_a,
                    entity_b,
                    normal: normal_a_to_b,
                    impulse: 0.0,
                });
            event.impulse += record.normal_impulse_n_s.max(0.0) as f32;
        }
        state.contacts = events.into_values().collect();
        state.last_dt_s = Some(dt_s);
        Ok(())
    }

    fn sync_to_ecs(
        &mut self,
        world: &mut World,
        physics_world: PhysicsWorldId,
    ) -> Result<(), PhysicsError> {
        let state = self.world_mut(physics_world)?;
        let dt_s = state.last_dt_s;
        for assembly in &mut state.assemblies {
            let poses = assembly.body_poses()?;
            let pose_of: HashMap<Entity, Transform3> = assembly
                .bodies
                .iter()
                .zip(&poses)
                .map(|(body, pose)| (body.entity, *pose))
                .collect();
            for (index, body) in assembly.bodies.iter().enumerate() {
                let pose = poses[index];
                let previous = assembly.written[index].0;
                let (linear, angular) = match dt_s {
                    Some(dt) => (
                        (pose.translation - previous.translation) / dt,
                        (pose.rotation * previous.rotation.conjugate()).to_scaled_axis() / dt,
                    ),
                    None => (assembly.written[index].1, assembly.written[index].2),
                };
                let local = match world.get::<Parent>(body.entity).map(|parent| parent.0) {
                    Some(parent) => {
                        let parent_pose = pose_of
                            .get(&parent)
                            .copied()
                            .unwrap_or_else(|| world_transform_of(world, parent));
                        inverse(&parent_pose).mul_transform(&pose)
                    }
                    None => pose,
                };
                world.entity_mut(body.entity).insert(local);
                if let Some(mut rigid) = world.get_mut::<RigidBody>(body.entity) {
                    rigid.linear_velocity_m_s = linear;
                    rigid.angular_velocity_rad_s = angular;
                }
                if let Some((position, rate, prismatic)) = assembly.joint_state(body.entity) {
                    let joint_state = if prismatic {
                        JointState::Prismatic {
                            position_m: position,
                            velocity_m_s: rate,
                        }
                    } else {
                        JointState::Revolute {
                            position_rad: position,
                            velocity_rad_s: rate,
                        }
                    };
                    world.entity_mut(body.entity).insert(joint_state);
                } else if body.joint.is_some() {
                    world.entity_mut(body.entity).insert(JointState::Fixed);
                }
            }
            remember_ecs_state(assembly, world);
        }
        Ok(())
    }

    fn raycast(
        &self,
        physics_world: PhysicsWorldId,
        query: RaycastQuery,
    ) -> Result<Vec<RaycastHit>, PhysicsError> {
        let state = self.world(physics_world)?;
        let Some(direction) = query.direction.try_normalize() else {
            return Ok(Vec::new());
        };
        if !(query.origin_m.is_finite() && query.max_distance_m >= 0.0) {
            return Ok(Vec::new());
        }
        let to_sim = world_to_sim();
        let (origin, direction) = (to_sim * query.origin_m, to_sim * direction);
        let mut targets: Vec<(Entity, &ColliderShape, Transform3, &[Arc<MeshTree>])> = state
            .statics
            .iter()
            .map(|collider| {
                (
                    collider.entity,
                    &collider.shape,
                    collider.pose,
                    collider.meshes.as_slice(),
                )
            })
            .collect();
        for assembly in &state.assemblies {
            let transforms = assembly.sim_transforms()?;
            for body in &assembly.bodies {
                if let Some((shape, offset)) = &body.collider {
                    let pose = transforms[body.link_index].mul_transform(offset);
                    targets.push((body.entity, shape, pose, body.meshes.as_slice()));
                }
            }
        }
        let mut hits: Vec<RaycastHit> = targets
            .into_iter()
            .filter_map(|(entity, shape, pose, meshes)| {
                let (distance_m, normal) = raycast::cast(
                    shape,
                    &pose,
                    meshes,
                    origin,
                    direction,
                    query.max_distance_m,
                )?;
                Some(RaycastHit {
                    entity,
                    point_m: query.origin_m + to_sim.conjugate() * direction * distance_m,
                    normal: to_sim.conjugate() * normal,
                    distance_m,
                })
            })
            .collect();
        hits.sort_by(|left, right| {
            left.distance_m
                .total_cmp(&right.distance_m)
                .then_with(|| left.entity.index().cmp(&right.entity.index()))
        });
        Ok(hits)
    }

    fn contacts(&self, physics_world: PhysicsWorldId) -> Result<&[ContactEvent], PhysicsError> {
        Ok(&self.world(physics_world)?.contacts)
    }

    fn multibody_joint_state(
        &self,
        physics_world: PhysicsWorldId,
        entity: Entity,
    ) -> Option<(f64, f64)> {
        let state = self.worlds.get(&physics_world)?;
        let assembly = state.assemblies.get(*state.body_assembly.get(&entity)?)?;
        assembly
            .joint_state(entity)
            .map(|(position, rate, _)| (position, rate))
    }

    fn capabilities(&self) -> &[PhysicsCapability] {
        CAPABILITIES
    }
}

#[cfg(test)]
mod tests;
