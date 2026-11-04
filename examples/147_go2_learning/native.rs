//! The Go2 on RNE's native articulated backend (`rne_physics_native`, on
//! `rne_dynamics`' hard-contact stepping), behind the same [`Robot`] interface
//! as the Rapier walker.

use std::collections::HashMap;

use rne_core::SimDuration;
use rne_ecs::{Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, JointActuation, JointState, PhysicsBackend, PhysicsMaterial,
    PhysicsWorldDesc, PhysicsWorldId, RigidBody, RigidBodyType,
};
use rne_physics_native::NativeBackend;
use rne_robot::Joint;
use rne_urdf_import::{
    attach_urdf_document_articulation, parse_urdf_document, spawn_urdf_document_with_config,
    UrdfArticulationConfig, UrdfDocument, UrdfSpawnConfig,
};
use rne_world::{world_transform_of, Transform3};

use crate::go2::{
    Robot, DECIMATION, DEFAULT_RAD, DT_S, KD_NM_S_PER_RAD, KP_NM_PER_RAD, LINKS, SPAWN_HEIGHT_M,
};
use crate::Result;

const GO2_URDF: &str = include_str!("../../assets/robots/go2_description/go2_description.rne.urdf");

/// One actuated joint: its child link, URDF effort limit and joint axis.
#[derive(Clone, Copy, Debug)]
struct NativeJoint {
    link: Entity,
    effort_limit_nm: f64,
}

/// A Go2 on the native backend, rebuilt in its default pose by
/// [`Robot::restart`].
pub(crate) struct NativeGo2 {
    document: UrdfDocument,
    floor_friction: f64,
    world: World,
    backend: NativeBackend,
    physics_world: PhysicsWorldId,
    base: Entity,
    joints: Vec<NativeJoint>,
    targets_rad: [f64; 12],
}

impl NativeGo2 {
    pub(crate) fn new(floor_friction: f64) -> Result<Self> {
        let mut robot = Self {
            document: parse_urdf_document(GO2_URDF)?,
            floor_friction,
            world: World::new(),
            backend: NativeBackend::new(),
            physics_world: PhysicsWorldId(0),
            base: Entity::PLACEHOLDER,
            joints: Vec::new(),
            targets_rad: DEFAULT_RAD,
        };
        robot.restart()?;
        Ok(robot)
    }

    /// The Go2 in its default pose, at rest, its base `SPAWN_HEIGHT_M` up,
    /// on a floor, with the given friction on every collider.
    fn build(&mut self) -> Result<()> {
        let mut world = World::new();
        let config = UrdfSpawnConfig {
            attach_physics: true,
            attach_colliders: true,
            attach_mesh_colliders: false,
            self_collisions: false,
            base_body_type: RigidBodyType::Dynamic,
            use_declared_inertial_masses: true,
            ..UrdfSpawnConfig::default()
        };
        let spawned = spawn_urdf_document_with_config(&mut world, &self.document, config)?;
        attach_urdf_document_articulation(
            &mut world,
            &self.document,
            &spawned,
            UrdfArticulationConfig {
                base_body_type: RigidBodyType::Dynamic,
                multibody: true,
                weld_fixed_children: true,
                ..UrdfArticulationConfig::default()
            },
        )?;
        // The URDF is Z-up; RNE's world is Y-up.
        world
            .entity_mut(spawned.base_link)
            .insert(Transform3::from_translation_rotation(
                Vec3::new(0.0, SPAWN_HEIGHT_M, 0.0),
                Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2),
            ));
        let joint_by_child: HashMap<Entity, Joint> = spawned
            .joints
            .values()
            .filter_map(|&entity| world.get::<Joint>(entity).cloned())
            .map(|joint| (joint.child_link, joint))
            .collect();
        let mut joints = Vec::with_capacity(12);
        for (name, default_rad) in LINKS.iter().zip(DEFAULT_RAD) {
            let link = *spawned
                .links
                .get(*name)
                .ok_or_else(|| format!("missing link {name}"))?;
            let joint = joint_by_child
                .get(&link)
                .ok_or_else(|| format!("{name} hangs from no joint"))?;
            // Start in the default pose: rotate the child about its joint axis.
            let origin = *world.get::<Transform3>(link).ok_or("link has no pose")?;
            world
                .entity_mut(link)
                .insert(Transform3::from_translation_rotation(
                    origin.translation,
                    origin.rotation * Quat::from_axis_angle(joint.axis.normalize(), default_rad),
                ));
            world
                .entity_mut(link)
                .insert(JointActuation::RevoluteEffort {
                    effort_nm: 0.0,
                    max_effort_nm: joint.limits.max_effort,
                });
            joints.push(NativeJoint {
                link,
                effort_limit_nm: joint.limits.max_effort,
            });
        }
        let colliders: Vec<Entity> = world
            .iter_entities()
            .filter(|entity| entity.get::<Collider>().is_some())
            .map(|entity| entity.id())
            .collect();
        for entity in colliders {
            if let Some(mut collider) = world.get_mut::<Collider>(entity) {
                collider.material.friction = self.floor_friction as f32;
            }
        }
        world.spawn((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Cuboid {
                    half_extents_m: Vec3::new(20.0, 0.5, 20.0),
                },
                material: PhysicsMaterial {
                    friction: self.floor_friction as f32,
                    restitution: 0.0,
                },
                local_offset: Transform3::default(),
                sensor: false,
            },
            Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
        ));
        let mut backend = NativeBackend::new();
        self.physics_world = backend.create_world(PhysicsWorldDesc::default())?;
        self.world = world;
        self.backend = backend;
        self.base = spawned.base_link;
        self.joints = joints;
        Ok(())
    }

    fn joint_state(&self, index: usize) -> (f64, f64) {
        match self.world.get::<JointState>(self.joints[index].link) {
            Some(JointState::Revolute {
                position_rad,
                velocity_rad_s,
            }) => (*position_rad, *velocity_rad_s),
            // Before the first step, the pose the robot was built in.
            _ => (DEFAULT_RAD[index], 0.0),
        }
    }
}

/// A Y-up vector in the Z-up world.
fn z_up(v: Vec3) -> Vec3 {
    Vec3::new(v.x, -v.z, v.y)
}

impl Robot for NativeGo2 {
    fn restart(&mut self) -> Result<()> {
        self.targets_rad = DEFAULT_RAD;
        self.build()
    }

    fn set_targets_rad(&mut self, targets_rad: &[f64; 12]) -> Result<()> {
        self.targets_rad = *targets_rad;
        Ok(())
    }

    fn step(&mut self) -> Result<()> {
        let dt = SimDuration::from_ticks((DT_S * 1e9).round() as u64);
        for _ in 0..DECIMATION {
            for index in 0..self.joints.len() {
                let (position, velocity) = self.joint_state(index);
                let joint = self.joints[index];
                let effort_nm = (KP_NM_PER_RAD * (self.targets_rad[index] - position)
                    - KD_NM_S_PER_RAD * velocity)
                    .clamp(-joint.effort_limit_nm, joint.effort_limit_nm);
                if let Some(mut actuation) = self.world.get_mut::<JointActuation>(joint.link) {
                    *actuation = JointActuation::RevoluteEffort {
                        effort_nm,
                        max_effort_nm: joint.effort_limit_nm,
                    };
                }
            }
            self.backend
                .sync_from_ecs(&mut self.world, self.physics_world)?;
            self.backend.step(self.physics_world, dt)?;
            self.backend
                .sync_to_ecs(&mut self.world, self.physics_world)?;
        }
        Ok(())
    }

    fn base_pose_z_up(&self) -> (Vec3, Quat) {
        let base = world_transform_of(&self.world, self.base);
        (
            z_up(base.translation),
            Quat::from_rotation_x(std::f64::consts::FRAC_PI_2) * base.rotation,
        )
    }

    fn base_velocity_z_up(&self) -> (Vec3, Vec3) {
        self.world
            .get::<RigidBody>(self.base)
            .map_or((Vec3::ZERO, Vec3::ZERO), |body| {
                (
                    z_up(body.linear_velocity_m_s),
                    z_up(body.angular_velocity_rad_s),
                )
            })
    }

    fn joint_states(&self) -> Result<[(f64, f64); 12]> {
        Ok(std::array::from_fn(|index| self.joint_state(index)))
    }
}
