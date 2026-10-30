//! H5 constant COM load via horizontal body acceleration, with a fixed floor.
use rne_core::SimDuration;
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, ExternalBodyWrench, PhysicsBackend, PhysicsMaterial, PhysicsWorldDesc, RigidBody,
    RigidBodyInertia, RigidBodyType,
};
use rne_physics_native::{NativeBackend, NativeBackendConfig};
use rne_world::Transform3;
use serde_json::json;
use std::{error::Error, time::Instant};

#[allow(clippy::too_many_lines)]
fn main() -> Result<(), Box<dyn Error>> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    if !(7..=8).contains(&args.len()) {
        return Err(
            "usage: friction_threshold DT_TICKS MASS_KG FORCE_RATIO REGULARIZATION ITERATIONS audit|timing MU [wrench]"
                .into(),
        );
    }
    let wrench = match args.get(7).map(String::as_str) {
        None => false,
        Some("wrench") => true,
        _ => return Err("load mode must be wrench when provided".into()),
    };
    let ticks: u64 = args[0].parse()?;
    let mass: f64 = args[1].parse()?;
    let ratio: f64 = args[2].parse()?;
    let regularization: f64 = args[3].parse()?;
    let iterations: usize = args[4].parse()?;
    let mu: f32 = args[6].parse()?;
    let audit = match args[5].as_str() {
        "audit" => true,
        "timing" => false,
        _ => return Err("mode must be audit or timing".into()),
    };
    if ![250_000, 500_000, 1_000_000, 2_000_000].contains(&ticks)
        || !mu.is_finite()
        || !(0.1..=0.5).contains(&mu)
        || !mass.is_finite()
        || !(0.1..=10.0).contains(&mass)
        || !ratio.is_finite()
        || ![0.8, 1.2, -0.8, -1.2].contains(&ratio)
        || ![0.0, 0.001].contains(&regularization)
        || ![20, 200].contains(&iterations)
    {
        return Err("input outside H5 protocol grid".into());
    }
    let mut config = NativeBackendConfig::default();
    config.contact.solver.regularization = regularization;
    config.contact.solver.max_iterations = iterations;
    let mut backend = NativeBackend::with_config(config);
    let id = backend.create_world(PhysicsWorldDesc {
        gravity_m_s2: Vec3::new(
            if wrench {
                0.0
            } else {
                ratio * f64::from(mu) * 9.81
            },
            -9.81,
            0.0,
        ),
        ..Default::default()
    })?;
    let mut world = World::new();
    let material = PhysicsMaterial {
        friction: mu,
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
    let half: f64 = 0.05;
    let inertia = mass * (2.0 * half).powi(2) / 6.0;
    let rotation = Quat::IDENTITY;
    let mut cube = Collider::cuboid(Vec3::splat(half));
    cube.material = material;
    let ball = world
        .spawn((
            RigidBody {
                mass_kg: mass,
                linear_velocity_m_s: Vec3::ZERO,
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
            Transform3::from_translation_rotation(Vec3::new(0.0, half, 0.0), rotation),
        ))
        .id();
    backend.sync_from_ecs(&mut world, id)?;
    let steps = 2_000_000_000 / ticks;
    let dt = SimDuration::from_ticks(ticks);
    let mut trace = Vec::new();
    if audit {
        trace.push(sample(&world, ball, 0.0, 0.0)?);
    }
    let mut start = Instant::now();
    let mut prefix_elapsed = 0.0;
    for i in 0..steps {
        if !wrench && !audit && i + 1 == steps {
            // ECS velocity is pose-differenced against the last write-back.
            // Sample the penultimate pose outside timing so final velocity
            // has the same one-step meaning as the audit, without full tracing.
            prefix_elapsed = start.elapsed().as_secs_f64();
            backend.sync_to_ecs(&mut world, id)?;
            start = Instant::now();
        }
        if wrench {
            backend.apply_external_body_wrench(
                id,
                ExternalBodyWrench {
                    entity: ball,
                    point_world_m: world
                        .get::<Transform3>(ball)
                        .ok_or("missing pose")?
                        .translation,
                    force_world_n: Vec3::X * (mass * ratio * f64::from(mu) * 9.81),
                    torque_world_nm: Vec3::ZERO,
                },
            )?;
        }
        backend.step(id, dt)?;
        if wrench {
            backend.sync_to_ecs(&mut world, id)?;
        }
        if audit {
            if !wrench {
                backend.sync_to_ecs(&mut world, id)?;
            }
            let impulse: f64 = backend
                .contacts(id)?
                .iter()
                .map(|c| f64::from(c.impulse))
                .sum();
            trace.push(sample(
                &world,
                ball,
                (i + 1) as f64 * ticks as f64 * 1e-9,
                impulse,
            )?);
        }
    }
    let elapsed = prefix_elapsed + start.elapsed().as_secs_f64();
    if !audit && !wrench {
        backend.sync_to_ecs(&mut world, id)?;
    }
    let final_state = sample(&world, ball, 2.0, 0.0)?;
    println!(
        "{}",
        json!({"backend":"rne_native", "debug_assertions":cfg!(debug_assertions), "mode":args[5], "elapsed_s":elapsed,
        "trace":trace, "final_state":final_state,
        "load_mode":if wrench { "external_body_wrench" } else { "equivalent_gravity" },
        "settings":{"equivalent_force_n":mass * ratio * f64::from(mu) * 9.81,"friction_coefficient":mu,"regularization":regularization,"max_iterations":iterations,
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
