//! Fixed-base, contact-free joint response runner for cross-simulator diagnostics.
use anyhow::{ensure, Context, Result};
use rne_core::SimDuration;
use rne_ecs::World;
use rne_math::Vec3;
use rne_physics::{
    JointActuation, JointMotor, JointPassiveDynamics, PhysicsBackend, PhysicsOwnedPose,
    PhysicsWorldDesc, RigidBodyType,
};
use rne_physics_rapier::RapierBackend;
use rne_urdf_import::{
    attach_urdf_document_articulation, parse_urdf_document, spawn_urdf_document_with_config,
    UrdfArticulationConfig, UrdfJointType, UrdfSpawnConfig,
};
use rne_world::world_transform_of;
use serde::{Deserialize, Serialize};
use std::time::Instant;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Input {
    pub urdf: String,
    pub joint_names: Vec<String>,
    pub dt_ticks: u64,
    #[serde(default)]
    pub gravity_m_s2: [f64; 3],
    pub torques_nm: Vec<Vec<f64>>,
}
#[derive(Debug, Serialize)]
pub(crate) struct Output {
    pub backend: &'static str,
    pub joint_names: Vec<String>,
    pub q_rad: Vec<Vec<f64>>,
    pub dq_rad_s: Vec<Vec<f64>>,
    /// Host timing includes ECS synchronization, stepping and state collection.
    pub runner_loop_s: f64,
}

fn validate(input: &Input) -> Result<(rne_urdf_import::UrdfDocument, Vec<String>)> {
    ensure!(
        input
            .gravity_m_s2
            .iter()
            .all(|v| v.is_finite() && v.abs() <= 100.0),
        "gravity components must be finite and at most 100 m/s² in magnitude"
    );
    ensure!(
        (1..=10_000_000).contains(&input.dt_ticks),
        "invalid timestep"
    );
    ensure!(
        (1..=10_000).contains(&input.torques_nm.len()),
        "invalid trajectory length"
    );
    let doc = parse_urdf_document(&input.urdf)?;
    ensure!(
        doc.robot
            .links
            .iter()
            .all(|l| doc.inertials.contains_key(&l.name)),
        "every link needs declared inertial properties"
    );
    ensure!(
        doc.joint_dynamics
            .values()
            .all(|d| d.damping == 0.0 && d.friction == 0.0),
        "benchmark requires zero joint losses"
    );
    ensure!(
        doc.robot.links.iter().all(|l| l.collisions.is_empty()),
        "benchmark must be contact-free"
    );
    ensure!(
        doc.robot.joints.iter().all(|j| matches!(
            j.joint_type,
            UrdfJointType::Continuous | UrdfJointType::Fixed
        )),
        "benchmark requires unlimited rotary or fixed joints"
    );
    let mut names = doc
        .robot
        .joints
        .iter()
        .filter(|j| j.joint_type == UrdfJointType::Continuous)
        .map(|j| j.name.clone())
        .collect::<Vec<_>>();
    names.sort();
    ensure!(
        !names.is_empty() && names == input.joint_names,
        "joint order must be the sorted movable-joint catalog"
    );
    ensure!(
        input
            .torques_nm
            .iter()
            .all(|row| row.len() == names.len()
                && row.iter().all(|v| v.is_finite() && v.abs() <= 2.0)),
        "invalid torque matrix"
    );
    Ok((doc, names))
}

pub(crate) fn run(input: Input) -> Result<Output> {
    let (doc, names) = validate(&input)?;
    let mut world = World::new();
    let robot = spawn_urdf_document_with_config(
        &mut world,
        &doc,
        UrdfSpawnConfig {
            base_body_type: RigidBodyType::Fixed,
            use_declared_inertial_masses: true,
            ..Default::default()
        },
    )?;
    attach_urdf_document_articulation(
        &mut world,
        &doc,
        &robot,
        UrdfArticulationConfig {
            base_body_type: RigidBodyType::Fixed,
            multibody: true,
            use_joint_origin_rpy: true,
            weld_fixed_children: true,
            ..Default::default()
        },
    )?;
    let poses = robot
        .links
        .values()
        .map(|e| (*e, world_transform_of(&world, *e)))
        .collect::<Vec<_>>();
    for (entity, pose) in poses {
        world
            .entity_mut(entity)
            .remove::<rne_ecs::Parent>()
            .insert((pose, PhysicsOwnedPose));
    }
    let joints = names
        .iter()
        .map(|name| {
            let joint = doc
                .robot
                .joints
                .iter()
                .find(|j| &j.name == name)
                .expect("catalog checked");
            robot.links[&joint.child]
        })
        .collect::<Vec<_>>();
    for entity in &joints {
        world.entity_mut(*entity).remove::<JointMotor>().insert((
            JointActuation::Disabled,
            // Missing URDF dynamics preserves Rapier's legacy damping.
            // This runner promises a lossless plant, so explicitly own zero.
            JointPassiveDynamics::Revolute {
                viscous_damping_nm_s_per_rad: 0.0,
                coulomb_friction_nm: 0.0,
                coulomb_transition_velocity_rad_s: 0.01,
            },
        ));
    }
    let mut backend = RapierBackend::new();
    let id = backend.create_world(PhysicsWorldDesc {
        gravity_m_s2: Vec3::new(
            input.gravity_m_s2[0],
            input.gravity_m_s2[1],
            input.gravity_m_s2[2],
        ),
        solver_iterations: 12,
    })?;
    backend.sync_from_ecs(&mut world, id)?;
    let initial = joints
        .iter()
        .map(|entity| {
            backend
                .multibody_joint_state(id, *entity)
                .context("missing initial feedback")
        })
        .collect::<Result<Vec<_>>>()?;
    let mut q_rad = vec![initial.iter().map(|(q, _)| *q).collect::<Vec<_>>()];
    let mut dq_rad_s = vec![initial.iter().map(|(_, v)| *v).collect::<Vec<_>>()];
    let start = Instant::now();
    for torques in &input.torques_nm {
        for (entity, effort) in joints.iter().zip(torques) {
            world
                .entity_mut(*entity)
                .insert(JointActuation::RevoluteEffort {
                    effort_nm: *effort,
                    max_effort_nm: 2.0,
                });
        }
        backend.sync_from_ecs(&mut world, id)?;
        backend.step(id, SimDuration::from_ticks(input.dt_ticks))?;
        backend.sync_to_ecs(&mut world, id)?;
        let states = joints
            .iter()
            .map(|entity| {
                backend
                    .multibody_joint_state(id, *entity)
                    .context("missing joint feedback")
            })
            .collect::<Result<Vec<_>>>()?;
        ensure!(
            states.iter().all(|(q, v)| q.is_finite() && v.is_finite()),
            "non-finite response"
        );
        q_rad.push(states.iter().map(|(q, _)| *q).collect());
        dq_rad_s.push(states.iter().map(|(_, v)| *v).collect());
    }
    Ok(Output {
        backend: "rne_rapier",
        joint_names: names,
        q_rad,
        dq_rad_s,
        runner_loop_s: start.elapsed().as_secs_f64(),
    })
}
