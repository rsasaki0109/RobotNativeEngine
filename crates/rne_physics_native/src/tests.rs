use super::*;
use rne_math::{Hertz, Quat};
use rne_physics::{
    CollisionGroups, CompoundPart, FixedJointDesc, JointActuation, PhysicsMaterial,
    RevoluteJointDesc,
};
use rne_physics_conformance::{
    run_external_backend_conformance, ExternalPhysicsBackendCheckStatus,
    ExternalPhysicsBackendConformanceConfig, ExternalPhysicsBackendSubject,
};

fn dt() -> SimDuration {
    SimDuration::from_hertz(Hertz::new(500.0))
}

fn step(backend: &mut NativeBackend, world: &mut World, id: PhysicsWorldId) {
    backend.sync_from_ecs(world, id).expect("sync from ecs");
    backend.step(id, dt()).expect("step");
    backend.sync_to_ecs(world, id).expect("sync to ecs");
}

fn ground(world: &mut World) -> Entity {
    world
        .spawn((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider::cuboid(Vec3::new(5.0, 0.5, 5.0)),
            Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
        ))
        .id()
}

#[test]
fn a_free_body_falls_with_gravity() {
    let mut backend = NativeBackend::new();
    let id = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("world");
    let mut world = World::new();
    let ball = world
        .spawn((
            RigidBody {
                mass_kg: 2.0,
                linear_velocity_m_s: Vec3::new(1.0, 0.0, 0.0),
                ..RigidBody::default()
            },
            Collider::sphere(0.05),
            Transform3::from_translation_rotation(Vec3::new(0.0, 10.0, 0.0), Quat::IDENTITY),
        ))
        .id();
    for _ in 0..500 {
        step(&mut backend, &mut world, id);
    }
    let pose = world.get::<Transform3>(ball).expect("pose");
    let body = world.get::<RigidBody>(ball).expect("body");
    // One second of semi-implicit Euler at 500 Hz.
    let expected_y = 10.0 - 0.5 * 9.81 * (1.0 + 1.0 / 500.0);
    assert!(
        (pose.translation.y - expected_y).abs() < 1.0e-9,
        "{}",
        pose.translation.y
    );
    assert!((pose.translation.x - 1.0).abs() < 1.0e-9);
    assert!((body.linear_velocity_m_s.y + 9.81).abs() < 1.0e-9);
    assert_eq!(backend.assembly_count(id).expect("count"), 1);
}

#[test]
fn a_box_rests_on_the_ground_and_reports_its_weight() {
    let mut backend = NativeBackend::new();
    let id = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("world");
    let mut world = World::new();
    let floor = ground(&mut world);
    let cube = world
        .spawn((
            RigidBody {
                mass_kg: 2.0,
                ..RigidBody::default()
            },
            Collider::cuboid(Vec3::splat(0.25)),
            Transform3::from_translation_rotation(
                Vec3::new(0.0, 0.3, 0.0),
                Quat::from_rotation_z(0.2),
            ),
        ))
        .id();
    for _ in 0..1500 {
        step(&mut backend, &mut world, id);
    }
    let pose = world.get::<Transform3>(cube).expect("pose");
    // It tipped onto a face and settled with less than a millimeter of
    // penetration.
    assert!(
        (pose.translation.y - 0.25).abs() < 1.0e-3,
        "{}",
        pose.translation.y
    );
    assert!((pose.rotation * Vec3::Y).y > 1.0 - 1.0e-6);
    // One event per pair, A being the lower entity index (the floor).
    let event = backend
        .contacts(id)
        .expect("contacts")
        .iter()
        .find(|event| event.entity_a == floor && event.entity_b == cube)
        .copied()
        .expect("resting contact");
    let weight_impulse = 2.0 * 9.81 / 500.0;
    assert!((f64::from(event.impulse) - weight_impulse).abs() < 1.0e-3 * weight_impulse);
    // The normal points from A (the floor) to B (the cube).
    assert!((event.normal - Vec3::Y).length() < 1.0e-9);
}

fn pendulum(world: &mut World, actuation: JointActuation) -> (Entity, Entity) {
    let pivot = world
        .spawn((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Transform3::from_translation_rotation(Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY),
        ))
        .id();
    let bob = world
        .spawn((
            RigidBody {
                mass_kg: 1.0,
                ..RigidBody::default()
            },
            Collider::sphere(0.05),
            Transform3::from_translation_rotation(Vec3::new(1.0, 2.0, 0.0), Quat::IDENTITY),
            RevoluteJointDesc {
                parent: pivot,
                axis: Vec3::Z,
                anchor_parent_m: Vec3::ZERO,
                anchor_child_m: Vec3::new(-1.0, 0.0, 0.0),
                relative_rotation: Quat::IDENTITY,
                lower_rad: Some(-1.0),
                upper_rad: Some(1.0),
            },
            actuation,
        ))
        .id();
    (pivot, bob)
}

#[test]
fn a_pendulum_swings_about_its_anchor_and_stops_at_its_limit() {
    let mut backend = NativeBackend::new();
    let id = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("world");
    let mut world = World::new();
    let (pivot, bob) = pendulum(&mut world, JointActuation::Disabled);
    let mut lowest: f64 = 0.0;
    for _ in 0..1000 {
        step(&mut backend, &mut world, id);
        let angle = world
            .get::<JointState>(bob)
            .and_then(|state| state.position_rad())
            .expect("joint state");
        lowest = lowest.min(angle);
        let pose = world.get::<Transform3>(bob).expect("pose");
        let anchor = pose.translation + pose.rotation * Vec3::new(-1.0, 0.0, 0.0);
        assert!((anchor - Vec3::new(0.0, 2.0, 0.0)).length() < 1.0e-9);
    }
    // Released level, the arm falls clockwise (negative about +Z) and stops
    // at its −1 rad limit.
    assert!(lowest > -1.0 - 2.0e-3 && lowest < -0.99, "{lowest}");
    let (angle, _) = backend.multibody_joint_state(id, bob).expect("joint");
    assert!((angle + 1.0).abs() < 2.0e-3);
    assert!(backend.multibody_joint_state(id, pivot).is_none());
}

#[test]
fn an_arm_on_a_moving_kinematic_cart_rides_along_with_it() {
    let mut backend = NativeBackend::new();
    let id = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("world");
    let mut world = World::new();
    let start = Vec3::new(0.0, 2.0, 0.0);
    let cart = world
        .spawn((
            RigidBody {
                body_type: RigidBodyType::Kinematic,
                ..RigidBody::default()
            },
            Transform3::from_translation_rotation(start, Quat::IDENTITY),
        ))
        .id();
    let shoulder = Vec3::new(0.0, 0.3, 0.0);
    let arm = world
        .spawn((
            RigidBody {
                mass_kg: 1.0,
                ..RigidBody::default()
            },
            Collider::sphere(0.05),
            Transform3::from_translation_rotation(start + shoulder + Vec3::X, Quat::IDENTITY),
            RevoluteJointDesc {
                parent: cart,
                axis: Vec3::Z,
                anchor_parent_m: shoulder,
                anchor_child_m: Vec3::new(-1.0, 0.0, 0.0),
                relative_rotation: Quat::IDENTITY,
                lower_rad: None,
                upper_rad: None,
            },
            JointActuation::RevolutePosition {
                target_position_rad: 0.3,
                stiffness_nm_per_rad: 2000.0,
                damping_nm_s_per_rad: 100.0,
                max_effort_nm: 1000.0,
            },
        ))
        .id();
    // The cart drives 1 m along x while turning a quarter turn about y.
    let steps = 1000;
    for index in 1..=steps {
        let s = (index as f64 / steps as f64).min(1.0);
        world
            .entity_mut(cart)
            .insert(Transform3::from_translation_rotation(
                start + Vec3::new(s, 0.0, 0.0),
                Quat::from_rotation_y(s * std::f64::consts::FRAC_PI_2),
            ));
        step(&mut backend, &mut world, id);
        // The shoulder stays on the cart at every step.
        let cart_pose = *world.get::<Transform3>(cart).expect("cart");
        let arm_pose = *world.get::<Transform3>(arm).expect("arm");
        let on_cart = cart_pose.translation + cart_pose.rotation * shoulder;
        let on_arm = arm_pose.translation + arm_pose.rotation * Vec3::new(-1.0, 0.0, 0.0);
        assert!((on_cart - on_arm).length() < 1.0e-9, "step {index}");
    }
    for _ in 0..500 {
        step(&mut backend, &mut world, id);
    }
    // At rest on the turned cart, the arm holds its angle in the cart's frame.
    let (angle, _) = backend.multibody_joint_state(id, arm).expect("joint");
    assert!((angle - 0.3).abs() < 0.01, "angle {angle}");
    let cart_pose = *world.get::<Transform3>(cart).expect("cart");
    let arm_pose = *world.get::<Transform3>(arm).expect("arm");
    let expected = cart_pose.rotation * Quat::from_rotation_z(angle);
    assert!(arm_pose.rotation.dot(expected).abs() > 1.0 - 1.0e-9);
    assert!((cart_pose.translation - Vec3::new(1.0, 2.0, 0.0)).length() < 1.0e-12);
}

#[test]
fn a_pendulum_on_an_accelerating_cart_leans_back_by_the_acceleration_angle() {
    let mut backend = NativeBackend::new();
    let id = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("world");
    let mut world = World::new();
    let start = Vec3::new(0.0, 2.0, 0.0);
    let cart = world
        .spawn((
            RigidBody {
                body_type: RigidBodyType::Kinematic,
                ..RigidBody::default()
            },
            Transform3::from_translation_rotation(start, Quat::IDENTITY),
        ))
        .id();
    // A bob hanging 1 m below a pivot on the cart, swinging about z, with a
    // viscous damper so it settles.
    let bob = world
        .spawn((
            RigidBody {
                mass_kg: 1.0,
                ..RigidBody::default()
            },
            Collider::sphere(0.05),
            Transform3::from_translation_rotation(start - Vec3::Y, Quat::IDENTITY),
            RevoluteJointDesc {
                parent: cart,
                axis: Vec3::Z,
                anchor_parent_m: Vec3::ZERO,
                anchor_child_m: Vec3::Y,
                relative_rotation: Quat::IDENTITY,
                lower_rad: None,
                upper_rad: None,
            },
            JointActuation::RevoluteVelocity {
                target_velocity_rad_s: 0.0,
                gain_nm_s_per_rad: 3.0,
                max_effort_nm: 100.0,
            },
        ))
        .id();
    // The cart accelerates along +x at 2 m/s² for 6 s.
    let acceleration = 2.0;
    let dt = 1.0 / 500.0;
    for index in 1..=3000 {
        let t = index as f64 * dt;
        world
            .entity_mut(cart)
            .insert(Transform3::from_translation_rotation(
                start + Vec3::new(0.5 * acceleration * t * t, 0.0, 0.0),
                Quat::IDENTITY,
            ));
        step(&mut backend, &mut world, id);
    }
    // The bob trails the cart: it turns by -atan(a / g) about z.
    let (angle, rate) = backend.multibody_joint_state(id, bob).expect("joint");
    let expected = -(acceleration / 9.81_f64).atan();
    assert!(
        (angle - expected).abs() < 2.0e-3,
        "angle {angle}, expected {expected}"
    );
    assert!(rate.abs() < 1.0e-2, "rate {rate}");
}

#[test]
fn position_and_effort_commands_drive_the_joint_within_their_limits() {
    let mut backend = NativeBackend::new();
    let id = backend
        .create_world(PhysicsWorldDesc {
            gravity_m_s2: Vec3::ZERO,
            ..PhysicsWorldDesc::default()
        })
        .expect("world");
    let mut world = World::new();
    let (_, bob) = pendulum(
        &mut world,
        JointActuation::RevolutePosition {
            target_position_rad: 0.5,
            stiffness_nm_per_rad: 200.0,
            damping_nm_s_per_rad: 30.0,
            max_effort_nm: 100.0,
        },
    );
    for _ in 0..1000 {
        step(&mut backend, &mut world, id);
    }
    let (angle, rate) = backend.multibody_joint_state(id, bob).expect("joint");
    assert!(
        (angle - 0.5).abs() < 1.0e-6 && rate.abs() < 1.0e-6,
        "{angle} {rate}"
    );

    // A 10 N·m command limited to 1 N·m accelerates the 1 kg·m² arm at 1 rad/s².
    world
        .entity_mut(bob)
        .insert(JointActuation::RevoluteEffort {
            effort_nm: 10.0,
            max_effort_nm: 1.0,
        });
    for _ in 0..250 {
        step(&mut backend, &mut world, id);
    }
    let (_, rate) = backend.multibody_joint_state(id, bob).expect("joint");
    assert!((rate - 0.5).abs() < 0.01, "{rate}");
}

#[test]
fn an_ecs_pose_edit_teleports_the_body() {
    let mut backend = NativeBackend::new();
    let id = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("world");
    let mut world = World::new();
    let ball = world
        .spawn((
            RigidBody::default(),
            Collider::sphere(0.1),
            Transform3::from_translation_rotation(Vec3::new(0.0, 5.0, 0.0), Quat::IDENTITY),
        ))
        .id();
    for _ in 0..100 {
        step(&mut backend, &mut world, id);
    }
    world
        .entity_mut(ball)
        .insert(Transform3::from_translation_rotation(
            Vec3::new(3.0, 8.0, 0.0),
            Quat::IDENTITY,
        ));
    world
        .get_mut::<RigidBody>(ball)
        .expect("body")
        .linear_velocity_m_s = Vec3::ZERO;
    step(&mut backend, &mut world, id);
    let pose = world.get::<Transform3>(ball).expect("pose");
    assert!((pose.translation - Vec3::new(3.0, 8.0 - 9.81 / 500.0 / 500.0, 0.0)).length() < 1.0e-9);
}

#[test]
fn invalid_actuation_is_rejected() {
    let mut backend = NativeBackend::new();
    let id = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("world");
    let mut world = World::new();
    pendulum(
        &mut world,
        JointActuation::PrismaticEffort {
            force_n: 1.0,
            max_force_n: 1.0,
        },
    );
    assert!(matches!(
        backend.sync_from_ecs(&mut world, id),
        Err(PhysicsError::InvalidActuation { .. })
    ));
}

#[test]
fn friction_holds_a_box_on_a_slope_only_when_it_exceeds_the_tangent() {
    for (friction, holds) in [(0.6_f32, true), (0.2, false)] {
        let mut backend = NativeBackend::new();
        let id = backend
            .create_world(PhysicsWorldDesc::default())
            .expect("world");
        let mut world = World::new();
        let slope = Quat::from_rotation_z(0.3);
        let material = PhysicsMaterial {
            friction,
            ..PhysicsMaterial::default()
        };
        world.spawn((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider {
                material,
                ..Collider::cuboid(Vec3::new(5.0, 0.5, 5.0))
            },
            Transform3::from_translation_rotation(slope * Vec3::new(0.0, -0.5, 0.0), slope),
        ));
        let cube = world
            .spawn((
                RigidBody::default(),
                Collider {
                    material,
                    ..Collider::cuboid(Vec3::splat(0.1))
                },
                Transform3::from_translation_rotation(slope * Vec3::new(0.0, 0.1, 0.0), slope),
            ))
            .id();
        for _ in 0..500 {
            step(&mut backend, &mut world, id);
        }
        let moved = world
            .get::<Transform3>(cube)
            .expect("pose")
            .translation
            .distance(slope * Vec3::new(0.0, 0.1, 0.0));
        // tan 0.3 ≈ 0.31.
        assert_eq!(moved < 1.0e-3, holds, "friction {friction}: moved {moved}");
    }
}

#[test]
fn the_backend_passes_external_conformance_for_its_capabilities() {
    let subject = ExternalPhysicsBackendSubject::from_bytes(
        "rne_physics_native",
        b"rne_physics_native conformance subject",
    )
    .expect("subject");
    let report = run_external_backend_conformance::<NativeBackend, _>(
        ExternalPhysicsBackendConformanceConfig::new(subject, NativeBackend::manifest()),
        NativeBackend::new,
    )
    .expect("report");
    for check in &report.checks {
        if let Some(capability) = check.capability {
            let expected = if CAPABILITIES.contains(&capability) {
                ExternalPhysicsBackendCheckStatus::Passed
            } else {
                ExternalPhysicsBackendCheckStatus::NotAdvertised
            };
            assert_eq!(check.status, expected, "{}: {}", check.id, check.detail);
        }
    }
    assert!(
        report.passed(),
        "{}",
        report.to_json_pretty().expect("json")
    );
}

fn free_box(world: &mut World, half_m: Vec3, translation: Vec3) -> Entity {
    world
        .spawn((
            RigidBody {
                mass_kg: 1.0,
                ..RigidBody::default()
            },
            Collider::cuboid(half_m),
            Transform3::from_translation_rotation(translation, Quat::IDENTITY),
        ))
        .id()
}

#[test]
fn a_stack_of_boxes_rests_and_each_contact_carries_the_weight_above_it() {
    let mut backend = NativeBackend::new();
    let id = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("world");
    let mut world = World::new();
    let floor = ground(&mut world);
    let half = Vec3::new(0.2, 0.1, 0.2);
    let boxes: Vec<Entity> = (0..3)
        .map(|level| {
            free_box(
                &mut world,
                half,
                Vec3::new(0.01 * level as f64, 0.1 + 0.205 * level as f64, 0.0),
            )
        })
        .collect();
    for _ in 0..1500 {
        step(&mut backend, &mut world, id);
    }
    for (level, entity) in boxes.iter().enumerate() {
        let pose = world.get::<Transform3>(*entity).expect("pose");
        assert!(
            (pose.translation.y - (0.1 + 0.2 * level as f64)).abs() < 1.0e-3,
            "box {level} at {}",
            pose.translation.y
        );
    }
    let impulse = |a: Entity, b: Entity| {
        backend
            .contacts(id)
            .expect("contacts")
            .iter()
            .find(|event| event.entity_a == a && event.entity_b == b)
            .map(|event| f64::from(event.impulse))
            .expect("contact event")
    };
    let weight = 9.81 / 500.0;
    assert!((impulse(floor, boxes[0]) - 3.0 * weight).abs() < 1.0e-3 * weight);
    assert!((impulse(boxes[0], boxes[1]) - 2.0 * weight).abs() < 1.0e-3 * weight);
    assert!((impulse(boxes[1], boxes[2]) - weight).abs() < 1.0e-3 * weight);
    // Separate bodies that touch are solved as one assembly island; the
    // backend still reports one assembly per body.
    assert_eq!(backend.assembly_count(id).expect("count"), 3);
}

#[test]
fn a_plank_rests_crosswise_on_a_beam_through_their_crossing_edges() {
    let mut backend = NativeBackend::new();
    let id = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("world");
    let mut world = World::new();
    let floor = ground(&mut world);
    // No corner of the plank lies over the beam's top face, nor a corner of
    // the beam under the plank's: only the crossing edges hold the plank.
    let beam = free_box(
        &mut world,
        Vec3::new(0.05, 0.05, 0.5),
        Vec3::new(0.0, 0.05, 0.0),
    );
    let plank = free_box(
        &mut world,
        Vec3::new(0.5, 0.05, 0.05),
        Vec3::new(0.0, 0.155, 0.0),
    );
    for _ in 0..1000 {
        step(&mut backend, &mut world, id);
    }
    let height = |entity: Entity| world.get::<Transform3>(entity).expect("pose").translation.y;
    assert!(
        (height(beam) - 0.05).abs() < 1.0e-3,
        "beam at {}",
        height(beam)
    );
    assert!(
        (height(plank) - 0.15).abs() < 1.0e-3,
        "plank at {}",
        height(plank)
    );
    let impulse = |a: Entity, b: Entity| {
        backend
            .contacts(id)
            .expect("contacts")
            .iter()
            .find(|event| event.entity_a == a && event.entity_b == b)
            .map(|event| f64::from(event.impulse))
            .expect("contact event")
    };
    let weight = 9.81 / 500.0;
    assert!((impulse(floor, beam) - 2.0 * weight).abs() < 1.0e-3 * weight);
    assert!((impulse(beam, plank) - weight).abs() < 1.0e-3 * weight);
}

#[test]
fn raycasts_hit_fixed_and_moving_bodies_at_their_current_pose() {
    let mut backend = NativeBackend::new();
    let id = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("world");
    let mut world = World::new();
    let floor = ground(&mut world);
    let crate_box = free_box(
        &mut world,
        Vec3::new(0.2, 0.1, 0.2),
        Vec3::new(0.0, 0.5, 0.0),
    );
    backend.sync_from_ecs(&mut world, id).expect("sync");
    let down = RaycastQuery::downward(Vec3::new(0.05, 2.0, 0.03), 10.0);
    let hits = backend.raycast(id, down).expect("raycast");
    // Before any step the box hangs at 0.5 m: its top is 1.4 m below.
    assert_eq!(hits.len(), 2);
    assert_eq!((hits[0].entity, hits[1].entity), (crate_box, floor));
    assert!((hits[0].distance_m - 1.4).abs() < 1.0e-9);
    assert!((hits[1].distance_m - 2.0).abs() < 1.0e-9);
    for _ in 0..500 {
        step(&mut backend, &mut world, id);
    }
    // At rest on the ground, the box top is at 0.2 m.
    let hits = backend.raycast(id, down).expect("raycast");
    assert_eq!(hits[0].entity, crate_box);
    assert!((hits[0].distance_m - 1.8).abs() < 1.0e-4);
    assert!((hits[0].point_m - Vec3::new(0.05, 0.2, 0.03)).length() < 1.0e-4);
    assert!((hits[0].normal - Vec3::Y).length() < 1.0e-6);
    // Sideways, at mid height: the box's -x face, and the ground is not hit.
    let side = RaycastQuery {
        origin_m: Vec3::new(-2.0, 0.1, 0.0),
        direction: Vec3::new(2.0, 0.0, 0.0),
        max_distance_m: 10.0,
    };
    let hits = backend.raycast(id, side).expect("raycast");
    assert_eq!(hits.len(), 1);
    assert!((hits[0].distance_m - 1.8).abs() < 1.0e-4);
    assert!((hits[0].normal - Vec3::NEG_X).length() < 1.0e-6);
    // Too short to reach, or with no direction: nothing.
    let short = RaycastQuery {
        max_distance_m: 1.0,
        ..side
    };
    assert!(backend.raycast(id, short).expect("raycast").is_empty());
    let still = RaycastQuery {
        direction: Vec3::ZERO,
        ..side
    };
    assert!(backend.raycast(id, still).expect("raycast").is_empty());
    let batch = backend
        .raycast_batch(id, &[down, side])
        .expect("raycast batch");
    assert_eq!(batch[0], backend.raycast(id, down).expect("raycast"));
    assert_eq!(batch[1], backend.raycast(id, side).expect("raycast"));
}

/// A fixed collider of `shape` at `translation`.
fn fixed_shape(world: &mut World, shape: ColliderShape, translation: Vec3) -> Entity {
    world
        .spawn((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider {
                shape,
                ..Collider::cuboid(Vec3::ONE)
            },
            Transform3::from_translation_rotation(translation, Quat::IDENTITY),
        ))
        .id()
}

/// A 10 x 10 m floor of two triangles facing up.
fn mesh_floor() -> ColliderShape {
    ColliderShape::TriMesh {
        vertices: vec![
            Vec3::new(-5.0, 0.0, -5.0),
            Vec3::new(-5.0, 0.0, 5.0),
            Vec3::new(5.0, 0.0, 5.0),
            Vec3::new(5.0, 0.0, -5.0),
        ]
        .into(),
        indices: vec![0, 1, 2, 0, 2, 3].into(),
    }
}

/// A closed cube of side `2 * half_m` around its origin, wound outward.
fn mesh_cube(half_m: f64) -> ColliderShape {
    let vertices: Vec<Vec3> = (0..8)
        .map(|corner| {
            let sign = |bit: usize| if corner & bit == 0 { -half_m } else { half_m };
            Vec3::new(sign(1), sign(2), sign(4))
        })
        .collect();
    ColliderShape::TriMesh {
        vertices: vertices.into(),
        indices: vec![
            0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4, 2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4,
            6, 1, 3, 5, 3, 7, 5,
        ]
        .into(),
    }
}

fn free_ball(world: &mut World, radius_m: f64, translation: Vec3) -> Entity {
    world
        .spawn((
            RigidBody {
                mass_kg: 1.0,
                ..RigidBody::default()
            },
            Collider::sphere(radius_m),
            Transform3::from_translation_rotation(translation, Quat::IDENTITY),
        ))
        .id()
}

fn contact_impulse(backend: &NativeBackend, id: PhysicsWorldId, a: Entity, b: Entity) -> f64 {
    backend
        .contacts(id)
        .expect("contacts")
        .iter()
        .filter(|event| event.entity_a == a && event.entity_b == b)
        .map(|event| f64::from(event.impulse))
        .sum()
}

#[test]
fn bodies_rest_on_triangle_meshes() {
    let mut backend = NativeBackend::new();
    let id = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("world");
    let mut world = World::new();
    let floor = fixed_shape(&mut world, mesh_floor(), Vec3::ZERO);
    let block = fixed_shape(&mut world, mesh_cube(0.5), Vec3::new(2.0, 0.5, 0.0));
    let crate_box = free_box(
        &mut world,
        Vec3::new(0.2, 0.1, 0.2),
        Vec3::new(0.0, 0.15, 0.0),
    );
    let ball = free_ball(&mut world, 0.1, Vec3::new(2.3, 1.3, 0.2));
    for _ in 0..1000 {
        step(&mut backend, &mut world, id);
    }
    let height = |entity: Entity| world.get::<Transform3>(entity).expect("pose").translation.y;
    assert!(
        (height(crate_box) - 0.1).abs() < 1.0e-3,
        "box at {}",
        height(crate_box)
    );
    assert!(
        (height(ball) - 1.1).abs() < 1.0e-3,
        "ball at {}",
        height(ball)
    );
    let weight = 9.81 / 500.0;
    let (low, high) = if floor.index() < crate_box.index() {
        (floor, crate_box)
    } else {
        (crate_box, floor)
    };
    assert!((contact_impulse(&backend, id, low, high) - weight).abs() < 1.0e-3 * weight);
    assert!((contact_impulse(&backend, id, block, ball) - weight).abs() < 1.0e-3 * weight);
    // Rays meet the meshes through their trees.
    let hits = backend
        .raycast(id, RaycastQuery::downward(Vec3::new(2.1, 3.0, -0.3), 5.0))
        .expect("raycast");
    assert_eq!(hits.len(), 2);
    assert_eq!((hits[0].entity, hits[1].entity), (block, floor));
    assert!((hits[0].distance_m - 2.0).abs() < 1.0e-9);
    assert!((hits[0].normal - Vec3::Y).length() < 1.0e-9);
}

#[test]
fn a_ball_rests_on_a_free_compound_table() {
    let mut backend = NativeBackend::new();
    let id = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("world");
    let mut world = World::new();
    let floor = fixed_shape(&mut world, mesh_floor(), Vec3::ZERO);
    let part = |half: Vec3, at: Vec3| CompoundPart {
        shape: ColliderShape::Cuboid {
            half_extents_m: half,
        },
        local_offset: Transform3::from_translation_rotation(at, Quat::IDENTITY),
    };
    let leg = Vec3::new(0.05, 0.225, 0.05);
    let mut parts = vec![part(Vec3::new(0.5, 0.05, 0.5), Vec3::new(0.0, 0.5, 0.0))];
    for (x, z) in [(0.4, 0.4), (-0.4, 0.4), (0.4, -0.4), (-0.4, -0.4)] {
        parts.push(part(leg, Vec3::new(x, 0.225, z)));
    }
    let table = world
        .spawn((
            RigidBody {
                mass_kg: 2.0,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Compound {
                    parts: parts.into(),
                },
                ..Collider::cuboid(Vec3::ONE)
            },
            Transform3::from_translation_rotation(Vec3::new(0.0, 0.002, 0.0), Quat::IDENTITY),
        ))
        .id();
    let ball = free_ball(&mut world, 0.1, Vec3::new(0.1, 0.8, -0.2));
    for _ in 0..1500 {
        step(&mut backend, &mut world, id);
    }
    let height = |entity: Entity| world.get::<Transform3>(entity).expect("pose").translation.y;
    assert!(height(table).abs() < 1.0e-3, "table at {}", height(table));
    assert!(
        (height(ball) - 0.65).abs() < 1.0e-3,
        "ball at {}",
        height(ball)
    );
    let weight = 9.81 / 500.0;
    assert!((contact_impulse(&backend, id, floor, table) - 3.0 * weight).abs() < 1.0e-3 * weight);
    assert!((contact_impulse(&backend, id, table, ball) - weight).abs() < 1.0e-3 * weight);
}

/// The convex hull of a box's corners, with an interior point.
fn hull_block(half_m: Vec3) -> ColliderShape {
    let mut points: Vec<Vec3> = (0..8)
        .map(|corner| {
            let sign = |bit: usize, half: f64| if corner & bit == 0 { -half } else { half };
            Vec3::new(sign(1, half_m.x), sign(2, half_m.y), sign(4, half_m.z))
        })
        .collect();
    points.push(Vec3::new(0.1 * half_m.x, -0.2 * half_m.y, 0.3 * half_m.z));
    ColliderShape::ConvexHull {
        points: points.into(),
    }
}

#[test]
fn bodies_rest_on_and_against_convex_hulls() {
    let mut backend = NativeBackend::new();
    let id = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("world");
    let mut world = World::new();
    let floor = ground(&mut world);
    // A fixed hull slab, a box resting on it, and a ball on a free hull block.
    let slab = fixed_shape(
        &mut world,
        hull_block(Vec3::new(0.5, 0.25, 0.5)),
        Vec3::new(-1.0, 0.25, 0.0),
    );
    let crate_box = free_box(
        &mut world,
        Vec3::new(0.2, 0.1, 0.2),
        Vec3::new(-1.0, 0.65, 0.0),
    );
    let block = world
        .spawn((
            RigidBody {
                mass_kg: 1.0,
                ..RigidBody::default()
            },
            Collider {
                shape: hull_block(Vec3::new(0.3, 0.1, 0.3)),
                ..Collider::cuboid(Vec3::ONE)
            },
            Transform3::from_translation_rotation(Vec3::new(1.0, 0.102, 0.0), Quat::IDENTITY),
        ))
        .id();
    let ball = free_ball(&mut world, 0.1, Vec3::new(1.1, 0.4, -0.05));
    for _ in 0..1000 {
        step(&mut backend, &mut world, id);
    }
    let height = |entity: Entity| world.get::<Transform3>(entity).expect("pose").translation.y;
    assert!(
        (height(crate_box) - 0.6).abs() < 1.0e-3,
        "box at {}",
        height(crate_box)
    );
    assert!(
        (height(block) - 0.1).abs() < 1.0e-3,
        "block at {}",
        height(block)
    );
    assert!(
        (height(ball) - 0.3).abs() < 1.0e-3,
        "ball at {}",
        height(ball)
    );
    let weight = 9.81 / 500.0;
    let pair = |a: Entity, b: Entity| {
        if a.index() < b.index() {
            contact_impulse(&backend, id, a, b)
        } else {
            contact_impulse(&backend, id, b, a)
        }
    };
    assert!((pair(slab, crate_box) - weight).abs() < 1.0e-3 * weight);
    assert!((pair(floor, block) - 2.0 * weight).abs() < 1.0e-3 * weight);
    assert!((pair(block, ball) - weight).abs() < 1.0e-3 * weight);
    // Rays hit the hulls: the slab's top from above, zero from inside.
    let hits = backend
        .raycast(id, RaycastQuery::downward(Vec3::new(-0.6, 2.0, 0.4), 5.0))
        .expect("raycast");
    assert_eq!(hits[0].entity, slab);
    assert!((hits[0].distance_m - 1.5).abs() < 1.0e-9);
    assert!((hits[0].normal - Vec3::Y).length() < 1.0e-9);
    let inside = backend
        .raycast(id, RaycastQuery::downward(Vec3::new(-1.0, 0.3, 0.0), 5.0))
        .expect("raycast");
    assert_eq!((inside[0].entity, inside[0].distance_m), (slab, 0.0));
}

#[test]
fn colliding_bodies_exchange_momentum_inelastically() {
    let mut backend = NativeBackend::new();
    let id = backend
        .create_world(PhysicsWorldDesc {
            gravity_m_s2: Vec3::ZERO,
            ..PhysicsWorldDesc::default()
        })
        .expect("world");
    let mut world = World::new();
    let slippery = PhysicsMaterial {
        friction: 0.0,
        ..PhysicsMaterial::default()
    };
    let ball = |world: &mut World, x: f64, velocity: f64| {
        world
            .spawn((
                RigidBody {
                    mass_kg: 1.0,
                    linear_velocity_m_s: Vec3::new(velocity, 0.0, 0.0),
                    ..RigidBody::default()
                },
                Collider {
                    material: slippery,
                    ..Collider::sphere(0.1)
                },
                Transform3::from_translation_rotation(Vec3::new(x, 1.0, 0.0), Quat::IDENTITY),
            ))
            .id()
    };
    let left = ball(&mut world, -0.5, 2.0);
    let right = ball(&mut world, 0.5, 0.0);
    for _ in 0..500 {
        step(&mut backend, &mut world, id);
    }
    let velocity = |entity| {
        world
            .get::<RigidBody>(entity)
            .expect("body")
            .linear_velocity_m_s
    };
    // Equal masses stick together at half the incoming speed.
    assert!(
        (velocity(left) - Vec3::new(1.0, 0.0, 0.0)).length() < 1.0e-6,
        "{}",
        velocity(left)
    );
    assert!((velocity(right) - Vec3::new(1.0, 0.0, 0.0)).length() < 1.0e-6);
    let gap = world.get::<Transform3>(right).expect("pose").translation.x
        - world.get::<Transform3>(left).expect("pose").translation.x;
    assert!((gap - 0.2).abs() < 1.0e-3, "{gap}");
}

/// A fixed plate, a welded arm, and a hinged flap that folds back toward the
/// plate under a constant torque.
fn folding_chain(world: &mut World, flap_groups: Option<CollisionGroups>) -> Entity {
    let pivot = world
        .spawn((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Transform3::default(),
        ))
        .id();
    let welded = |world: &mut World, parent: Entity, half: Vec3, x: f64| {
        world
            .spawn((
                RigidBody::default(),
                Collider::cuboid(half),
                Transform3::from_translation_rotation(Vec3::new(x, 0.0, 0.0), Quat::IDENTITY),
                FixedJointDesc {
                    parent,
                    anchor_parent_m: Vec3::new(x, 0.0, 0.0),
                    anchor_child_m: Vec3::ZERO,
                    relative_rotation: Quat::IDENTITY,
                },
            ))
            .id()
    };
    let plate = welded(world, pivot, Vec3::new(0.2, 0.08, 0.05), 0.2);
    let arm = welded(world, plate, Vec3::new(0.2, 0.025, 0.05), 0.2);
    let flap = world
        .spawn((
            RigidBody::default(),
            Collider::cuboid(Vec3::new(0.3, 0.025, 0.05)),
            Transform3::from_translation_rotation(Vec3::new(0.9, 0.0, 0.0), Quat::IDENTITY),
            RevoluteJointDesc {
                parent: arm,
                axis: Vec3::Z,
                anchor_parent_m: Vec3::new(0.2, 0.0, 0.0),
                anchor_child_m: Vec3::new(-0.3, 0.0, 0.0),
                relative_rotation: Quat::IDENTITY,
                lower_rad: Some(0.0),
                upper_rad: Some(3.1),
            },
            JointActuation::RevoluteEffort {
                effort_nm: 2.0,
                max_effort_nm: 2.0,
            },
        ))
        .id();
    if let Some(groups) = flap_groups {
        world.entity_mut(flap).insert(groups);
    }
    flap
}

#[test]
fn a_robot_link_collides_with_a_non_adjacent_link_of_the_same_robot() {
    let run = |groups| {
        let mut backend = NativeBackend::new();
        let id = backend
            .create_world(PhysicsWorldDesc {
                gravity_m_s2: Vec3::ZERO,
                ..PhysicsWorldDesc::default()
            })
            .expect("world");
        let mut world = World::new();
        let flap = folding_chain(&mut world, groups);
        for _ in 0..1500 {
            step(&mut backend, &mut world, id);
        }
        assert_eq!(backend.assembly_count(id).expect("count"), 1);
        backend.multibody_joint_state(id, flap).expect("joint").0
    };
    // The flap folds over the arm (its joint neighbor, never tested) until
    // its underside meets the plate's far top edge at (0.4, 0.08) from the
    // hinge at (0.6, 0): -0.2 sin θ - 0.08 cos θ = -0.025 (its half thickness).
    let contact_angle = {
        let residual = |angle: f64| 0.2 * angle.sin() + 0.08 * angle.cos() - 0.025;
        let (mut low, mut high) = (2.0_f64, 3.0_f64);
        for _ in 0..60 {
            let middle = 0.5 * (low + high);
            if residual(middle) > 0.0 {
                low = middle;
            } else {
                high = middle;
            }
        }
        low
    };
    let blocked = run(None);
    assert!(
        (blocked - contact_angle).abs() < 2.0e-3,
        "{blocked} vs {contact_angle}"
    );
    // With the flap's collisions filtered out it folds to its limit.
    let free = run(Some(CollisionGroups {
        memberships: 2,
        filter: 0,
    }));
    assert!((free - 3.1).abs() < 2.0e-3, "{free}");
}
