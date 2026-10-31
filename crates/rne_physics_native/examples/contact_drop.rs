//! H1 sphere drop: batch physics timing or post-step geometric audit.
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
    if args.len() != 6 {
        return Err(
            "usage: contact_drop DT_TICKS MASS_KG GAP_M REGULARIZATION ITERATIONS audit|timing"
                .into(),
        );
    }
    let ticks: u64 = args[0].parse()?;
    let mass: f64 = args[1].parse()?;
    let gap: f64 = args[2].parse()?;
    let regularization: f64 = args[3].parse()?;
    let iterations: usize = args[4].parse()?;
    let audit = match args[5].as_str() {
        "audit" => true,
        "timing" => false,
        _ => return Err("mode must be audit or timing".into()),
    };
    if ![250_000, 500_000, 1_000_000, 2_000_000].contains(&ticks)
        || !mass.is_finite()
        || !(0.1..=10.0).contains(&mass)
        || !gap.is_finite()
        || !(0.01..=0.5).contains(&gap)
        || ![0.0, 0.001].contains(&regularization)
        || ![20, 200].contains(&iterations)
    {
        return Err("input outside H1 protocol grid".into());
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
    world.spawn((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..Default::default()
        },
        floor,
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
    ));
    let radius = 0.05;
    let inertia = 0.4 * mass * radius * radius;
    let mut sphere = Collider::sphere(radius);
    sphere.material = material;
    let ball = world
        .spawn((
            RigidBody {
                mass_kg: mass,
                ..Default::default()
            },
            sphere,
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
                Vec3::new(0.0, radius + gap, 0.0),
                Quat::IDENTITY,
            ),
        ))
        .id();
    backend.sync_from_ecs(&mut world, id)?;
    let steps = 2_000_000_000 / ticks;
    let dt = SimDuration::from_ticks(ticks);
    let mut trace = Vec::new();
    if audit {
        trace.push([0.0, gap, 0.0, 0.0]);
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
            // Geometric separation at the updated pose, not solver-manifold gap.
            let y = world
                .get::<Transform3>(ball)
                .ok_or("missing pose")?
                .translation
                .y;
            let v = world
                .get::<RigidBody>(ball)
                .ok_or("missing body")?
                .linear_velocity_m_s
                .y;
            let impulse: f64 = backend
                .contacts(id)?
                .iter()
                .map(|c| f64::from(c.impulse))
                .sum();
            trace.push([(i + 1) as f64 * ticks as f64 * 1e-9, y - radius, v, impulse]);
        }
    }
    let elapsed = prefix_elapsed + start.elapsed().as_secs_f64();
    if !audit {
        backend.sync_to_ecs(&mut world, id)?;
    }
    let final_gap = world
        .get::<Transform3>(ball)
        .ok_or("missing final pose")?
        .translation
        .y
        - radius;
    let final_v = world
        .get::<RigidBody>(ball)
        .ok_or("missing final velocity")?
        .linear_velocity_m_s
        .y;
    println!(
        "{}",
        json!({"backend":"rne_native", "debug_assertions":cfg!(debug_assertions), "mode":args[5], "elapsed_s":elapsed,
        "trace":trace, "final_gap_m":final_gap, "final_velocity_m_s":final_v,
        "settings":{"regularization":regularization,"max_iterations":iterations,
        "contact_margin_m":config.contact_margin_m,"penetration_recovery":config.contact.penetration_recovery,
        "tolerance_n_s":config.contact.solver.tolerance_n_s,
        "velocity_tolerance_m_s":config.contact.solver.velocity_tolerance_m_s,
        "bisection_iterations":config.contact.solver.bisection_iterations}})
    );
    Ok(())
}
