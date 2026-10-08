//! Runs the Go2 through the backend-neutral `PhysicsBackend` interface on the
//! native articulated backend, and on Rapier for comparison.
//!
//! The scene is built the way the asset loader builds a dynamic Go2: the URDF
//! is spawned with colliders and declared inertias, wired as a multibody with
//! `attach_urdf_document_articulation`, posed Y-up above seeded fractal
//! terrain, and commanded through `JointActuation` position targets on each
//! joint's child link. The same generic loop — `sync_from_ecs`, `step`,
//! `sync_to_ecs` — then drives [`rne_physics_native::NativeBackend`] and
//! `RapierBackend`. The native run must land on all four feet, carry the
//! robot's weight through its contact events, come to rest level, and replay
//! bit for bit.
//!
//! Run with `cargo run --release -p go2_native_backend --example 137_go2_native_backend`.

use rne_core::SimDuration;
use rne_ecs::{Entity, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    height_field_surface, Collider, ColliderShape, FractalTerrain, JointActuation, PhysicsBackend,
    PhysicsMaterial, PhysicsWorldDesc, RigidBody, RigidBodyType,
};
use rne_physics_native::NativeBackend;
use rne_physics_rapier::RapierBackend;
use rne_robot::Joint;
use rne_urdf_import::{
    attach_urdf_document_articulation, parse_urdf_document, spawn_urdf_document_with_config,
    UrdfArticulationConfig, UrdfSpawnConfig,
};
use rne_world::{world_transform_of, Transform3};
use std::time::Instant;

const GO2_URDF: &str = include_str!("../../assets/robots/go2_description/go2_description.rne.urdf");
const FOOT_LINKS: [&str; 4] = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"];
const STAND_POSE_RAD: [(&str, f64); 3] = [("hip", 0.0), ("thigh", 0.8), ("calf", -1.5)];
const STIFFNESS_NM_RAD: f64 = 120.0;
const DAMPING_NM_S_RAD: f64 = 6.0;
const STEP_HZ: f64 = 500.0;
const DURATION_S: f64 = 2.0;
const DROP_HEIGHT_M: f64 = 0.42;
const TERRAIN_SEED: u64 = 2018;
const FRICTION: f32 = 0.8;

struct Scene {
    world: World,
    base_link: Entity,
    feet: Vec<Entity>,
    mass_kg: f64,
}

#[derive(Debug, PartialEq)]
struct Report {
    base_pose: Transform3,
    clearance_m: f64,
    tilt_rad: f64,
    speed_m_s: f64,
    loaded_feet: usize,
    support_n: f64,
    weight_n: f64,
    step_us: f64,
}

fn build_scene(terrain: &ColliderShape) -> Scene {
    let document = parse_urdf_document(GO2_URDF).expect("parse Go2 URDF");
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
    let spawned =
        spawn_urdf_document_with_config(&mut world, &document, config).expect("spawn Go2");
    attach_urdf_document_articulation(
        &mut world,
        &document,
        &spawned,
        UrdfArticulationConfig {
            base_body_type: RigidBodyType::Dynamic,
            multibody: true,
            // Weld the feet and other fixed-joint links to their parents
            // instead of leaving them as free bodies.
            weld_fixed_children: true,
            ..UrdfArticulationConfig::default()
        },
    )
    .expect("articulate Go2");
    let start = height_field_surface(terrain, 0.0, 0.0).expect("origin on terrain");
    // The vendored Go2 URDF is Z-up; rotate the base into RNE's Y-up world.
    world
        .entity_mut(spawned.base_link)
        .insert(Transform3::from_translation_rotation(
            Vec3::new(0.0, start.height_m + DROP_HEIGHT_M, 0.0),
            Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2),
        ));

    // Stand-pose position commands on every leg joint's child link.
    for (name, joint_entity) in &spawned.joints {
        let Some((_, target)) = STAND_POSE_RAD
            .iter()
            .find(|(part, _)| name.ends_with(&format!("_{part}_joint")))
        else {
            continue;
        };
        let joint = world.get::<Joint>(*joint_entity).expect("joint").clone();
        // Start in the stand pose: rotate the child link about its joint axis.
        let origin = *world
            .get::<Transform3>(joint.child_link)
            .expect("link pose");
        world
            .entity_mut(joint.child_link)
            .insert(Transform3::from_translation_rotation(
                origin.translation,
                origin.rotation * Quat::from_axis_angle(joint.axis.normalize(), *target),
            ));
        world
            .entity_mut(joint.child_link)
            .insert(JointActuation::RevolutePosition {
                target_position_rad: *target,
                stiffness_nm_per_rad: STIFFNESS_NM_RAD,
                damping_nm_s_per_rad: DAMPING_NM_S_RAD,
                max_effort_nm: joint.limits.max_effort,
            });
    }
    let mass_kg = spawned
        .links
        .values()
        .filter_map(|link| world.get::<RigidBody>(*link))
        .map(|body| body.mass_kg)
        .sum();

    world.spawn((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider {
            shape: terrain.clone(),
            material: PhysicsMaterial {
                friction: FRICTION,
                ..PhysicsMaterial::default()
            },
            local_offset: Transform3::default(),
            sensor: false,
        },
        Transform3::default(),
    ));
    let feet = FOOT_LINKS
        .iter()
        .map(|name| *spawned.links.get(*name).expect("foot link"))
        .collect();
    Scene {
        world,
        base_link: spawned.base_link,
        feet,
        mass_kg,
    }
}

fn simulate<B: PhysicsBackend>(mut backend: B, terrain: &ColliderShape) -> Report {
    let mut scene = build_scene(terrain);
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    let dt = SimDuration::from_hertz(Hertz::new(STEP_HZ));
    let steps = (DURATION_S * STEP_HZ).round() as usize;
    let started = Instant::now();
    for _ in 0..steps {
        backend
            .sync_from_ecs(&mut scene.world, physics_world)
            .expect("sync from ECS");
        backend.step(physics_world, dt).expect("step");
        backend
            .sync_to_ecs(&mut scene.world, physics_world)
            .expect("sync to ECS");
    }
    let step_us = started.elapsed().as_secs_f64() * 1.0e6 / steps as f64;

    let base_pose = world_transform_of(&scene.world, scene.base_link);
    let ground = height_field_surface(terrain, base_pose.translation.x, base_pose.translation.z)
        .expect("base over terrain");
    let up = base_pose.rotation * Vec3::Z;
    let body = scene
        .world
        .get::<RigidBody>(scene.base_link)
        .expect("base body");
    let contacts = backend.contacts(physics_world).expect("contacts");
    let loaded_feet = scene
        .feet
        .iter()
        .filter(|foot| {
            contacts
                .iter()
                .any(|event| event.entity_a == **foot && event.impulse > 0.0)
        })
        .count();
    let support_n = contacts
        .iter()
        .map(|event| f64::from(event.impulse) * -event.normal.y * STEP_HZ)
        .sum();
    Report {
        base_pose,
        clearance_m: base_pose.translation.y - ground.height_m,
        tilt_rad: up.y.clamp(-1.0, 1.0).acos(),
        speed_m_s: body.linear_velocity_m_s.length(),
        loaded_feet,
        support_n,
        weight_n: scene.mass_kg * 9.81,
        step_us,
    }
}

fn print_report(name: &str, report: &Report) {
    println!(
        "{name}: base clearance {:.3} m, tilt {:.3} rad, speed {:.1e} m/s, {} feet loaded",
        report.clearance_m, report.tilt_rad, report.speed_m_s, report.loaded_feet
    );
    println!(
        "{name}: contact support {:.1} N vs weight {:.1} N, {:.0} us per step",
        report.support_n, report.weight_n, report.step_us
    );
}

fn main() {
    let spec = FractalTerrain {
        size_x_m: 6.0,
        size_z_m: 6.0,
        columns: 61,
        rows: 61,
        frequency_per_m: 0.4,
        amplitude_m: 0.12,
        ..FractalTerrain::default()
    };
    let terrain = spec.height_field(TERRAIN_SEED).expect("terrain");
    println!(
        "go2 on seeded fractal terrain (seed {TERRAIN_SEED}), dropped {DROP_HEIGHT_M:.2} m, {DURATION_S:.0} s at {STEP_HZ:.0} Hz through PhysicsBackend"
    );
    let native = simulate(NativeBackend::new(), &terrain);
    let again = simulate(NativeBackend::new(), &terrain);
    let deterministic = native.base_pose == again.base_pose;
    print_report("native", &native);
    let rapier = simulate(RapierBackend::new(), &terrain);
    print_report("rapier", &rapier);
    println!("native deterministic replay: {deterministic}");

    let ok = deterministic
        && native.loaded_feet == 4
        && (native.support_n - native.weight_n).abs() < 0.02 * native.weight_n
        && native.clearance_m > 0.2
        && native.tilt_rad < 0.15
        && native.speed_m_s < 0.01;
    if !ok {
        eprintln!("go2 native backend: failed");
        std::process::exit(1);
    }
    println!("go2 native backend: ok");
}
