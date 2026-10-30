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

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if !(2..=4).contains(&args.len()) {
        return Err(
            "usage: contact_crowd COUNT audit|timing [separated|stacks|clusters|bridge] [REPEATS]"
                .into(),
        );
    }
    let layout = args.get(2).map_or("separated", String::as_str);
    if !["separated", "stacks", "clusters", "bridge"].contains(&layout) {
        return Err("unknown layout".into());
    }
    let count: usize = args[0].parse()?;
    let audit = match args[1].as_str() {
        "audit" => true,
        "timing" => false,
        _ => return Err("expected audit or timing".into()),
    };
    if layout == "bridge" && ![15, 31, 63].contains(&count) {
        return Err("bridge requires 15, 31 or 63 boxes".into());
    }
    if layout != "bridge" && ![1, 16, 64, 128].contains(&count) {
        return Err("count outside benchmark grid".into());
    }
    if ["stacks", "clusters"].contains(&layout) && ![64, 128].contains(&count) {
        return Err("contact layouts require 64 or 128 boxes".into());
    }
    let repeats: usize = args.get(3).map_or(Ok(1), |s| s.parse())?;
    if !(1..=1000).contains(&repeats) || (repeats != 1 && (audit || count != 1)) {
        return Err("repeats must be 1..=1000; batching requires one box in timing mode".into());
    }
    let mut result = run(count, audit, layout)?;
    if repeats > 1 {
        let mut elapsed = vec![result["elapsed_s"].as_f64().ok_or("missing elapsed")?];
        let mut hashes = vec![result["final_hash"].as_u64().ok_or("missing hash")?];
        for _ in 1..repeats {
            // Fresh backend and ECS: repeat the same landing, not a longer settle.
            let next = run(count, audit, layout)?;
            elapsed.push(next["elapsed_s"].as_f64().ok_or("missing elapsed")?);
            hashes.push(next["final_hash"].as_u64().ok_or("missing hash")?);
        }
        result["elapsed_s"] = json!(elapsed.iter().sum::<f64>());
        result["repeats"] = json!(repeats);
        result["repeat_elapsed_s"] = json!(elapsed);
        result["repeat_final_hashes"] = json!(hashes);
    }
    println!("{result}");
    Ok(())
}

#[allow(clippy::too_many_lines)]
fn run(count: usize, audit: bool, layout: &str) -> Result<serde_json::Value, Box<dyn Error>> {
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
                    initial_position(index, count, layout),
                    Quat::IDENTITY,
                ),
            ))
            .id();
        entities.push(entity);
    }
    backend.sync_from_ecs(&mut world, id)?;
    backend.set_contact_diagnostics_enabled(id, audit && layout != "separated")?;
    let mut hashes = Vec::new();
    let mut contact_frames = Vec::new();
    let mut max_penetration: f64 = 0.0;
    let mut max_spin: f64 = 0.0;
    let start = Instant::now();
    for tick in 0..100_u64 {
        backend.step(id, SimDuration::from_ticks(2_000_000))?;
        backend.sync_to_ecs(&mut world, id)?;
        if audit {
            if layout != "separated" {
                let points = backend.contact_impulse_diagnostics(id)?;
                let mut loaded_pairs = std::collections::BTreeSet::new();
                let mut penetration: f64 = 0.0;
                let mut iterations = 0;
                let mut converged = true;
                for point in points {
                    if point.body_entity != floor_entity && point.other_entity != floor_entity {
                        penetration = penetration.max(-point.gap_m);
                        if point.normal_impulse_n_s > 1e-10 {
                            let a = point.body_entity.index();
                            let b = point.other_entity.index();
                            loaded_pairs.insert((a.min(b), a.max(b)));
                        }
                    }
                    iterations = iterations.max(point.solver_iterations);
                    converged &= point.solver_converged;
                }
                let pipeline = backend
                    .contact_pipeline_diagnostics(id)?
                    .ok_or("missing pipeline")?;
                let mut frame = json!({"loaded_dynamic_pairs":loaded_pairs,
                    "pre_step_dynamic_penetration_m":penetration,"max_solver_iterations":iterations,
                    "all_solver_stopping_flags":converged,"generated":pipeline.generated_candidates,
                    "retained":pipeline.retained_candidates});
                if layout == "bridge" {
                    // Post-step poses allow an independent all-pairs OBB audit.
                    let poses: Vec<_> = entities
                        .iter()
                        .map(|&entity| {
                            let pose = world.get::<Transform3>(entity).expect("box pose");
                            let q = pose.rotation;
                            vec![
                                (tick + 1) as f64 * 0.002,
                                pose.translation.x,
                                pose.translation.y,
                                pose.translation.z,
                                q.x,
                                q.y,
                                q.z,
                                q.w,
                            ]
                        })
                        .collect();
                    frame["post_step_poses"] = json!(poses);
                }
                contact_frames.push(frame);
            }
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
    Ok(
        json!({"count":count,"mode":if audit { "audit" } else { "timing" },"debug_assertions":cfg!(debug_assertions),
        "elapsed_s":elapsed_s,"hashes":hashes,"final_hash":final_hash,
        "max_penetration_m":if audit {Some(max_penetration)} else {None},
        "max_spin_rad_s":if audit {Some(max_spin)} else {None},
        "steps":100,"dt_ticks":2_000_000,"margin_m":config.contact_margin_m,
        "layout":layout,"contact_frames":contact_frames}),
    )
}

fn initial_position(index: usize, count: usize, layout: &str) -> Vec3 {
    match layout {
        "bridge" => {
            let lower = count.div_ceil(2);
            let upper = index >= lower;
            let column = if upper { index - lower } else { index };
            Vec3::new(
                (column as f64 - (lower - 1) as f64 / 2.0 + if upper { 0.5 } else { 0.0 }) * 0.105,
                if upper { 0.160 } else { 0.055 },
                0.0,
            )
        }
        "stacks" => Vec3::new(
            ((index / 2) % 8) as f64 * 0.3,
            0.055 + (index % 2) as f64 * 0.105,
            (index / 16) as f64 * 0.3,
        ),
        "clusters" => Vec3::new(
            ((index / 4) % 8) as f64 * 0.35 + (index % 2) as f64 * 0.105,
            0.055 + ((index / 2) % 2) as f64 * 0.105,
            (index / 32) as f64 * 0.35,
        ),
        _ => Vec3::new(
            (index % 16) as f64 * 0.15,
            0.055,
            (index / 16) as f64 * 0.15,
        ),
    }
}
