//! H3 stack: pair-resolved support impulses and full-pose audit.
use rne_core::SimDuration;
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, PhysicsBackend, PhysicsMaterial, PhysicsWorldDesc, RigidBody, RigidBodyInertia,
    RigidBodyType,
};
use rne_physics_native::{NativeBackend, NativeBackendConfig};
use rne_world::Transform3;
use serde_json::json;
use std::{error::Error, time::Instant};

#[allow(clippy::too_many_lines)]
fn main() -> Result<(), Box<dyn Error>> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    if args.len() != 7 {
        return Err(
            "usage: stack_drop DT_TICKS TOP_MASS_KG GAP_M REGULARIZATION ITERATIONS audit|timing LEVELS"
                .into(),
        );
    }
    let ticks: u64 = args[0].parse()?;
    let mass: f64 = args[1].parse()?;
    let gap: f64 = args[2].parse()?;
    let regularization: f64 = args[3].parse()?;
    let iterations: usize = args[4].parse()?;
    let levels: usize = args[6].parse()?;
    let audit = match args[5].as_str() {
        "audit" => true,
        "timing" => false,
        _ => return Err("mode must be audit or timing".into()),
    };
    if ![250_000, 500_000, 1_000_000, 2_000_000].contains(&ticks)
        || ![2, 3].contains(&levels)
        || !mass.is_finite()
        || !(0.1..=10.0).contains(&mass)
        || !gap.is_finite()
        || !(0.001..=0.05).contains(&gap)
        || ![0.0, 0.001].contains(&regularization)
        || ![20, 200].contains(&iterations)
    {
        return Err("input outside H3 protocol grid".into());
    }
    let mut config = NativeBackendConfig::default();
    config.contact.solver.regularization = regularization;
    config.contact.solver.max_iterations = iterations;
    let mut backend = NativeBackend::with_config(config);
    let id = backend.create_world(PhysicsWorldDesc {
        gravity_m_s2: Vec3::new(0.0, -9.81, 0.0),
        ..Default::default()
    })?;
    let mut world = World::new();
    let material = PhysicsMaterial {
        friction: 0.0,
        restitution: 0.0,
    };
    let mut floor = Collider::cuboid(Vec3::new(5.0, 0.5, 5.0));
    floor.material = material;
    let floor_entity = world
        .spawn((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..Default::default()
            },
            floor,
            Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
        ))
        .id();
    let half: f64 = 0.05;
    let mut bodies = Vec::new();
    for level in 0..levels {
        let mass = if level + 1 == levels { mass } else { 1.0 };
        let inertia = mass * (2.0 * half).powi(2) / 6.0;
        let mut cube = Collider::cuboid(Vec3::splat(half));
        cube.material = material;
        let body = world
            .spawn((
                RigidBody {
                    mass_kg: mass,
                    ..Default::default()
                },
                cube,
                RigidBodyInertia {
                    center_of_mass_local_m: Vec3::ZERO,
                    ixx_kg_m2: inertia,
                    iyy_kg_m2: inertia,
                    izz_kg_m2: inertia,
                    ixy_kg_m2: 0.0,
                    ixz_kg_m2: 0.0,
                    iyz_kg_m2: 0.0,
                },
                Transform3::from_translation_rotation(
                    Vec3::new(0.0, half + gap + level as f64 * (2.0 * half + gap), 0.0),
                    Quat::IDENTITY,
                ),
            ))
            .id();
        bodies.push(body);
    }
    backend.sync_from_ecs(&mut world, id)?;
    let steps = 2_000_000_000 / ticks;
    let dt = SimDuration::from_ticks(ticks);
    let mut trace = Vec::new();
    if audit {
        trace.push(
            bodies
                .iter()
                .map(|b| sample(&world, *b, 0.0, 0.0))
                .collect::<Result<Vec<_>, _>>()?,
        );
    }
    let mut start = Instant::now();
    let mut prefix_elapsed = 0.0;
    for i in 0..steps {
        if !audit && i + 1 == steps {
            // ECS velocity is pose-differenced against the last write-back.
            // Sample the penultimate pose outside timing so final velocity
            // has the same one-step meaning as the audit, without full tracing.
            prefix_elapsed = start.elapsed().as_secs_f64();
            backend.sync_to_ecs(&mut world, id)?;
            start = Instant::now();
        }
        backend.step(id, dt)?;
        if audit {
            backend.sync_to_ecs(&mut world, id)?;
            let mut samples = Vec::new();
            for (level, body) in bodies.iter().enumerate() {
                let lower = if level == 0 {
                    floor_entity
                } else {
                    bodies[level - 1]
                };
                let impulse = backend
                    .contacts(id)?
                    .iter()
                    .filter(|c| {
                        (c.entity_a == lower && c.entity_b == *body)
                            || (c.entity_b == lower && c.entity_a == *body)
                    })
                    .map(|c| f64::from(c.impulse))
                    .sum();
                samples.push(sample(
                    &world,
                    *body,
                    (i + 1) as f64 * ticks as f64 * 1e-9,
                    impulse,
                )?);
            }
            trace.push(samples);
        }
    }
    let elapsed = prefix_elapsed + start.elapsed().as_secs_f64();
    if !audit {
        backend.sync_to_ecs(&mut world, id)?;
    }
    let final_state = bodies
        .iter()
        .map(|b| sample(&world, *b, 2.0, 0.0))
        .collect::<Result<Vec<_>, _>>()?;
    println!(
        "{}",
        json!({"backend":"rne_native", "debug_assertions":cfg!(debug_assertions), "mode":args[5], "elapsed_s":elapsed,
        "trace":trace, "final_state":final_state,
        "settings":{"regularization":regularization,"max_iterations":iterations,
        "contact_margin_m":config.contact_margin_m,"penetration_recovery":config.contact.penetration_recovery,
        "tolerance_n_s":config.contact.solver.tolerance_n_s,
        "velocity_tolerance_m_s":config.contact.solver.velocity_tolerance_m_s,
        "bisection_iterations":config.contact.solver.bisection_iterations}})
    );
    Ok(())
}

// Full pose and world-frame velocities allow independent corner/energy auditing.
fn sample(
    world: &World,
    body: rne_ecs::Entity,
    time: f64,
    impulse: f64,
) -> Result<[f64; 15], Box<dyn Error>> {
    let pose = world.get::<Transform3>(body).ok_or("missing pose")?;
    let rb = world.get::<RigidBody>(body).ok_or("missing body")?;
    let p = pose.translation;
    let q = pose.rotation;
    let v = rb.linear_velocity_m_s;
    let w = rb.angular_velocity_rad_s;
    Ok([
        time, p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z, impulse,
    ])
}
