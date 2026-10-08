//! Shared articulated models for unit tests.

use crate::ArticulatedModel;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{RigidBody, RigidBodyInertia};
use rne_robot::{FloatingBase, Joint, JointKind, JointLimits, Link, Robot, RobotId};
use rne_world::Transform3;

/// Small deterministic generator for test inputs.
pub(crate) struct Lcg(pub(crate) u64);

impl Lcg {
    /// Next value in `[-1, 1)`.
    pub(crate) fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        ((self.0 >> 11) as f64 / (1_u64 << 53) as f64) * 2.0 - 1.0
    }

    /// Vector with components in `[-1, 1)`.
    pub(crate) fn vec3(&mut self) -> Vec3 {
        Vec3::new(self.next(), self.next(), self.next())
    }
}

fn body(world: &mut World, robot: Entity, name: &str, rng: &mut Lcg, local: Transform3) -> Entity {
    let entity = spawn_named(world, name);
    let principal = [
        0.02 + 0.05 * rng.next().abs(),
        0.02 + 0.05 * rng.next().abs(),
        0.02 + 0.05 * rng.next().abs(),
    ];
    world.entity_mut(entity).insert((
        Link {
            robot,
            name: name.into(),
        },
        local,
        RigidBody {
            mass_kg: 0.5 + rng.next().abs(),
            ..RigidBody::default()
        },
        RigidBodyInertia {
            center_of_mass_local_m: rng.vec3() * 0.1,
            ixx_kg_m2: principal[0],
            ixy_kg_m2: 0.002 * rng.next(),
            ixz_kg_m2: 0.002 * rng.next(),
            iyy_kg_m2: principal[1],
            iyz_kg_m2: 0.002 * rng.next(),
            izz_kg_m2: principal[2],
        },
    ));
    entity
}

/// A branching tree mixing revolute, prismatic, and fixed joints with
/// skewed axes, offset origins, and full inertia tensors.
pub(crate) fn branching_tree(floating: bool, seed: u64) -> ArticulatedModel {
    let mut rng = Lcg(seed);
    let mut world = World::new();
    let robot = spawn_named(&mut world, "robot");
    let base = body(&mut world, robot, "base", &mut rng, Transform3::IDENTITY);
    world.entity_mut(robot).insert(Robot {
        robot_id: RobotId::new_v4(),
        model_name: "tree".into(),
        base_link: base,
    });
    if floating {
        world.entity_mut(base).insert(FloatingBase);
    }
    // (parent index, kind) for each child link; index 0 is the base.
    let layout = [
        (0, JointKind::Revolute),
        (1, JointKind::Revolute),
        (2, JointKind::Prismatic),
        (0, JointKind::Revolute),
        (4, JointKind::Fixed),
        (5, JointKind::Revolute),
        (0, JointKind::Prismatic),
        (7, JointKind::Continuous),
    ];
    let mut links = vec![base];
    for (child, (parent, kind)) in layout.into_iter().enumerate() {
        let rotation = Quat::from_scaled_axis(rng.vec3() * 0.8);
        let local = Transform3::from_translation_rotation(rng.vec3() * 0.3, rotation);
        let link = body(&mut world, robot, &format!("link{child}"), &mut rng, local);
        let joint = spawn_named(&mut world, format!("joint{child}"));
        world.entity_mut(joint).insert(Joint {
            robot,
            parent_link: links[parent],
            child_link: link,
            kind,
            limits: JointLimits::default(),
            axis: rng.vec3().normalize(),
            position: 0.0,
            velocity: 0.0,
        });
        links.push(link);
    }
    ArticulatedModel::from_robot(&world, robot).expect("model")
}
