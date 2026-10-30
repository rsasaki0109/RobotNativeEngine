//! Separated boxes landing together: pair rejection cost with a fixed contact margin.
use rne_core::SimDuration;
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_physics::{
    capture_physics_snapshot, Collider, PhysicsBackend, PhysicsMaterial, PhysicsWorldDesc,
    RigidBody, RigidBodyInertia, RigidBodyType,
};
use rne_physics_native::{NativeBackend, NativeBackendConfig};
use rne_world::Transform3;
use serde_json::json;
use std::{error::Error, time::Instant};

#[allow(clippy::too_many_lines)]
fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.len() != 2 {
        return Err("usage: contact_crowd COUNT audit|timing".into());
    }
    let count: usize = args[0].parse()?;
    let audit = match args[1].as_str() {
        "audit" => true,
        "timing" => false,
        _ => return Err("expected audit or timing".into()),
    };
    if ![1, 16, 64, 128].contains(&count) {
        return Err("count outside benchmark grid".into());
    }
    let mut config = NativeBackendConfig::default();
    config.contact.solver.regularization = 0.0;
    config.contact.solver.max_iterations = 200;
    let mut backend = NativeBackend::with_config(config);
    let id = backend.create_world(PhysicsWorldDesc {
        gravity_m_s2: Vec3::new(0.0, -9.81, 0.0),
        ..Default::default()
    })?;
    let mut world = World::new();
    let material = PhysicsMaterial {
        friction: 0.5,
        restitution: 0.0,
    };
    let mut floor = Collider::cuboid(Vec3::new(5.0, 0.5, 5.0));
    floor.material = material;
    world.spawn((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..Default::default()
        },
        floor,
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
    ));
    let mut entities = Vec::new();
    for index in 0..count {
        let mut cube = Collider::cuboid(Vec3::splat(0.05));
        cube.material = material;
        let entity = world
            .spawn((
                RigidBody {
                    mass_kg: 3.0,
                    ..Default::default()
                },
                cube,
                RigidBodyInertia {
                    center_of_mass_local_m: Vec3::ZERO,
                    ixx_kg_m2: 0.005,
                    iyy_kg_m2: 0.005,
                    izz_kg_m2: 0.005,
                    ixy_kg_m2: 0.0,
                    ixz_kg_m2: 0.0,
                    iyz_kg_m2: 0.0,
                },
                Transform3::from_translation_rotation(
                    Vec3::new(
                        (index % 16) as f64 * 0.15,
                        0.055,
                        (index / 16) as f64 * 0.15,
                    ),
                    Quat::IDENTITY,
                ),
            ))
            .id();
        entities.push(entity);
    }
    backend.sync_from_ecs(&mut world, id)?;
    let mut hashes = Vec::new();
    let mut max_penetration: f64 = 0.0;
    let mut max_spin: f64 = 0.0;
    let start = Instant::now();
    for tick in 0..100_u64 {
        backend.step(id, SimDuration::from_ticks(2_000_000))?;
        backend.sync_to_ecs(&mut world, id)?;
        if audit {
            hashes.push(
                capture_physics_snapshot(
                    &world,
                    backend.contacts(id)?,
                    tick + 1,
                    (tick + 1) * 2_000_000,
                )?
                .stable_hash(),
            );
            for &entity in &entities {
                let pose = world.get::<Transform3>(entity).ok_or("missing pose")?;
                let body = world.get::<RigidBody>(entity).ok_or("missing body")?;
                let extent = (pose.rotation * Vec3::X).y.abs()
                    + (pose.rotation * Vec3::Y).y.abs()
                    + (pose.rotation * Vec3::Z).y.abs();
                max_penetration = max_penetration.max(0.05 * extent - pose.translation.y);
                max_spin = max_spin.max(body.angular_velocity_rad_s.length());
            }
        }
    }
    let elapsed_s = start.elapsed().as_secs_f64();
    let final_hash =
        capture_physics_snapshot(&world, backend.contacts(id)?, 100, 200_000_000)?.stable_hash();
    println!(
        "{}",
        json!({"count":count,"mode":args[1],"debug_assertions":cfg!(debug_assertions),
        "elapsed_s":elapsed_s,"hashes":hashes,"final_hash":final_hash,
        "max_penetration_m":if audit {Some(max_penetration)} else {None},
        "max_spin_rad_s":if audit {Some(max_spin)} else {None},
        "steps":100,"dt_ticks":2_000_000,"margin_m":config.contact_margin_m})
    );
    Ok(())
}
