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
//! bit for bit. Rays cast down onto the base and onto the terrain check each
//! backend's raycasts.
//!
//! Run with `cargo run --release -p go2_native_backend --example 137_go2_native_backend`.

use rne_core::SimDuration;
use rne_ecs::{Entity, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    height_field_surface, Collider, ColliderShape, FractalTerrain, JointActuation, PhysicsBackend,
    PhysicsMaterial, PhysicsWorldDesc, PhysicsWorldId, RaycastQuery, RigidBody, RigidBodyType,
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
    /// Whether a ray cast down onto the base hits the base first.
    base_ray_hits_base: bool,
    /// Largest distance between a terrain ray hit and the bilinear surface.
    terrain_ray_error_m: f64,
}

/// Spawns the Go2 in its stand pose with its base `base_y_m` above the world
/// origin and returns it as a scene without ground.
fn spawn_go2(base_y_m: f64) -> Scene {
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
    // The vendored Go2 URDF is Z-up; rotate the base into RNE's Y-up world.
    world
        .entity_mut(spawned.base_link)
        .insert(Transform3::from_translation_rotation(
            Vec3::new(0.0, base_y_m, 0.0),
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
    // Sum in link-name order: the links map's order varies between runs, and
    // so would the sum's last bits.
    let mut links: Vec<(&String, &Entity)> = spawned.links.iter().collect();
    links.sort();
    let mass_kg = links
        .into_iter()
        .filter_map(|(_, link)| world.get::<RigidBody>(*link))
        .map(|body| body.mass_kg)
        .sum();
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

/// A fixed collider with the example's friction.
fn fixed_collider(world: &mut World, shape: ColliderShape, pose: Transform3) -> Entity {
    world
        .spawn((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider {
                shape,
                material: PhysicsMaterial {
                    friction: FRICTION,
                    ..PhysicsMaterial::default()
                },
                local_offset: Transform3::default(),
                sensor: false,
            },
            pose,
        ))
        .id()
}

/// Runs `world` for `seconds` through the generic `PhysicsBackend` loop and
/// returns the mean wall time per step in microseconds.
fn run<B: PhysicsBackend>(
    backend: &mut B,
    world: &mut World,
    physics_world: PhysicsWorldId,
    seconds: f64,
) -> f64 {
    let dt = SimDuration::from_hertz(Hertz::new(STEP_HZ));
    let steps = (seconds * STEP_HZ).round() as usize;
    let started = Instant::now();
    for _ in 0..steps {
        backend
            .sync_from_ecs(world, physics_world)
            .expect("sync from ECS");
        backend.step(physics_world, dt).expect("step");
        backend
            .sync_to_ecs(world, physics_world)
            .expect("sync to ECS");
    }
    started.elapsed().as_secs_f64() * 1.0e6 / steps as f64
}

fn simulate<B: PhysicsBackend>(mut backend: B, terrain: &ColliderShape) -> Report {
    let start = height_field_surface(terrain, 0.0, 0.0).expect("origin on terrain");
    let mut scene = spawn_go2(start.height_m + DROP_HEIGHT_M);
    let terrain_entity = fixed_collider(&mut scene.world, terrain.clone(), Transform3::default());
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    let step_us = run(&mut backend, &mut scene.world, physics_world, DURATION_S);

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
    let base_ray_hits_base = backend
        .raycast(
            physics_world,
            RaycastQuery::downward(base_pose.translation + Vec3::new(0.0, 1.0, 0.0), 2.0),
        )
        .expect("raycast")
        .first()
        .is_some_and(|hit| hit.entity == scene.base_link);
    let terrain_ray_error_m = terrain_scan(&backend, physics_world, terrain, terrain_entity);
    Report {
        base_pose,
        clearance_m: base_pose.translation.y - ground.height_m,
        tilt_rad: up.y.clamp(-1.0, 1.0).acos(),
        speed_m_s: body.linear_velocity_m_s.length(),
        loaded_feet,
        support_n,
        weight_n: scene.mass_kg * 9.81,
        step_us,
        base_ray_hits_base,
        terrain_ray_error_m,
    }
}

/// Casts a 9 x 9 grid of downward rays over the terrain, away from its edges,
/// and returns the largest distance between a terrain hit and the bilinear
/// surface of `height_field_surface`.
fn terrain_scan<B: PhysicsBackend>(
    backend: &B,
    physics_world: PhysicsWorldId,
    terrain: &ColliderShape,
    terrain_entity: Entity,
) -> f64 {
    let queries: Vec<RaycastQuery> = (0..81)
        .map(|index| {
            let x = -2.4 + 0.6 * (index % 9) as f64 + 0.013;
            let z = -2.4 + 0.6 * (index / 9) as f64 + 0.029;
            RaycastQuery::downward(Vec3::new(x, 2.0, z), 5.0)
        })
        .collect();
    let hits = backend
        .raycast_batch(physics_world, &queries)
        .expect("raycast batch");
    queries
        .iter()
        .zip(&hits)
        .map(|(query, hits)| {
            let surface = height_field_surface(terrain, query.origin_m.x, query.origin_m.z)
                .expect("ray over terrain");
            hits.iter()
                .find(|hit| hit.entity == terrain_entity)
                .map_or(f64::INFINITY, |hit| {
                    (hit.point_m.y - surface.height_m).abs()
                })
        })
        .fold(0.0, f64::max)
}

/// Half extents of the free crate the Go2 stands on, in meters.
const CRATE_HALF_M: Vec3 = Vec3::new(0.45, 0.1, 0.3);
const CRATE_MASS_KG: f64 = 8.0;
/// Half extent and count of the boxes in the free-standing tower.
const TOWER_HALF_M: f64 = 0.1;
const TOWER_BOXES: usize = 5;

#[derive(Debug, PartialEq)]
struct CrateReport {
    base_pose: Transform3,
    feet_on_crate: usize,
    feet_force_n: f64,
    crate_ground_force_n: f64,
    go2_weight_n: f64,
    crate_weight_n: f64,
    tower_drift_m: f64,
}

/// The Go2 dropped onto a free crate resting on the ground, beside a tower of
/// free boxes: every contact between the robot, the crate, the tower, and
/// the ground is solved together.
fn simulate_on_crate() -> CrateReport {
    let crate_top_m = 2.0 * CRATE_HALF_M.y;
    let mut scene = spawn_go2(crate_top_m + DROP_HEIGHT_M);
    let world = &mut scene.world;
    let ground = fixed_collider(
        world,
        ColliderShape::Cuboid {
            half_extents_m: Vec3::new(5.0, 0.5, 5.0),
        },
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
    );
    let free_box = |world: &mut World, half: Vec3, mass_kg: f64, center: Vec3| {
        world
            .spawn((
                RigidBody {
                    mass_kg,
                    ..RigidBody::default()
                },
                Collider {
                    material: PhysicsMaterial {
                        friction: FRICTION,
                        ..PhysicsMaterial::default()
                    },
                    ..Collider::cuboid(half)
                },
                Transform3::from_translation_rotation(center, Quat::IDENTITY),
            ))
            .id()
    };
    let crate_box = free_box(
        world,
        CRATE_HALF_M,
        CRATE_MASS_KG,
        Vec3::new(0.0, CRATE_HALF_M.y, 0.0),
    );
    let tower: Vec<Entity> = (0..TOWER_BOXES)
        .map(|level| {
            free_box(
                world,
                Vec3::splat(TOWER_HALF_M),
                1.0,
                Vec3::new(1.2, TOWER_HALF_M * (1.0 + 2.0 * level as f64), 0.0),
            )
        })
        .collect();
    let mut backend = NativeBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    run(&mut backend, world, physics_world, DURATION_S);

    let contacts = backend.contacts(physics_world).expect("contacts");
    let vertical = |a: Entity, b: Entity| {
        contacts
            .iter()
            .filter(|event| {
                (event.entity_a == a && event.entity_b == b)
                    || (event.entity_a == b && event.entity_b == a)
            })
            .map(|event| f64::from(event.impulse) * event.normal.y.abs() * STEP_HZ)
            .sum::<f64>()
    };
    let feet_on_crate = scene
        .feet
        .iter()
        .filter(|foot| vertical(**foot, crate_box) > 0.0)
        .count();
    let feet_force_n = scene
        .feet
        .iter()
        .map(|foot| vertical(*foot, crate_box))
        .sum();
    let tower_drift_m = tower
        .iter()
        .enumerate()
        .map(|(level, entity)| {
            let pose = world_transform_of(world, *entity);
            (pose.translation - Vec3::new(1.2, TOWER_HALF_M * (1.0 + 2.0 * level as f64), 0.0))
                .length()
        })
        .fold(0.0, f64::max);
    CrateReport {
        base_pose: world_transform_of(world, scene.base_link),
        feet_on_crate,
        feet_force_n,
        crate_ground_force_n: vertical(crate_box, ground),
        go2_weight_n: scene.mass_kg * 9.81,
        crate_weight_n: CRATE_MASS_KG * 9.81,
        tower_drift_m,
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
    println!(
        "{name}: a ray cast down onto the base hits {}; 81 terrain rays within {:.1e} m of the bilinear surface",
        if report.base_ray_hits_base { "the base" } else { "something else" },
        report.terrain_ray_error_m
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

    let on_crate = simulate_on_crate();
    let crate_deterministic = on_crate == simulate_on_crate();
    println!(
        "go2 on a free {:.0} kg crate: {} feet on the crate carrying {:.1} N (Go2 weight {:.1} N)",
        CRATE_MASS_KG, on_crate.feet_on_crate, on_crate.feet_force_n, on_crate.go2_weight_n
    );
    println!(
        "crate on the ground: {:.1} N (Go2 + crate {:.1} N); {TOWER_BOXES}-box tower drifted {:.2} mm; replay identical: {crate_deterministic}",
        on_crate.crate_ground_force_n,
        on_crate.go2_weight_n + on_crate.crate_weight_n,
        1.0e3 * on_crate.tower_drift_m
    );
    let crate_total = on_crate.go2_weight_n + on_crate.crate_weight_n;
    let crate_ok = crate_deterministic
        && on_crate.feet_on_crate == 4
        && (on_crate.feet_force_n - on_crate.go2_weight_n).abs() < 0.02 * on_crate.go2_weight_n
        && (on_crate.crate_ground_force_n - crate_total).abs() < 0.02 * crate_total
        && on_crate.tower_drift_m < 1.0e-3;

    let ok = deterministic
        && crate_ok
        && native.loaded_feet == 4
        && (native.support_n - native.weight_n).abs() < 0.02 * native.weight_n
        && native.clearance_m > 0.2
        && native.tilt_rad < 0.15
        && native.speed_m_s < 0.01
        && native.base_ray_hits_base
        && native.terrain_ray_error_m < 1.0e-9;
    if !ok {
        eprintln!("go2 native backend: failed");
        std::process::exit(1);
    }
    println!("go2 native backend: ok");
}
