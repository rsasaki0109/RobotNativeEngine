//! Trots the 12-DoF Unitree Go2 across seeded fractal terrain on the native
//! hard-contact step.
//!
//! The gait is an open-loop diagonal trot: the front-left and rear-right legs
//! swing while the other pair stands, and the pairs swap every half period.
//! Each swing leg flexes its calf to clear the ground and sweeps its thigh
//! forward; each stance leg sweeps its thigh back and pushes the body ahead.
//! The targets go to implicit joint PD, the actuators are clamped to the
//! Go2's declared effort limits, and the feet are sphere contacts on
//! [`rne_physics::FractalTerrain`] resolved by [`rne_dynamics::contact_step`]
//! with the exact friction cone. No physics backend, renderer, or wall clock is
//! involved, and the run is repeated to check that it is deterministic.
//!
//! Run with `cargo run -p go2_terrain_trot --example 134_go2_terrain_trot`.

use rne_dynamics::{
    contact_frame, contact_step, ArticulatedModel, ContactPoint, ContactStepConfig, JointPdControl,
};
use rne_ecs::{Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{height_field_surface, ColliderShape, FractalTerrain};
use rne_robot::{FloatingBase, Robot};
use rne_urdf_import::{parse_urdf_document, spawn_urdf_document_with_config, UrdfSpawnConfig};
use rne_world::Transform3;

const GO2_URDF: &str = include_str!("../../assets/robots/go2_description/go2_description.rne.urdf");
/// Legs in gait order; the first and last form one diagonal pair.
const LEGS: [&str; 4] = ["FL", "FR", "RL", "RR"];
/// Phase offset of each leg in [`LEGS`], as a fraction of the period.
const LEG_PHASE: [f64; 4] = [0.0, 0.5, 0.5, 0.0];
const FOOT_RADIUS_M: f64 = 0.022;
const FOOT_CENTER_LOCAL_M: Vec3 = Vec3::new(-0.002, 0.0, 0.0);
const FRICTION_COEFFICIENT: f64 = 0.8;
const STAND_HIP_RAD: f64 = 0.0;
const STAND_THIGH_RAD: f64 = 0.8;
const STAND_CALF_RAD: f64 = -1.5;
/// Gait period (one full stride of each leg), in seconds.
const PERIOD_S: f64 = 0.36;
/// Thigh sweep amplitude about the stand pose, in radians.
const THIGH_SWEEP_RAD: f64 = 0.15;
/// Extra calf flexion at mid-swing, in radians.
const CALF_LIFT_RAD: f64 = 0.7;
const POSITION_GAIN_NM_RAD: f64 = 100.0;
const VELOCITY_GAIN_NM_S_RAD: f64 = 3.0;
const STEP_TIME_S: f64 = 0.002;
/// Standing time before the gait starts, in seconds.
const SETTLE_S: f64 = 1.0;
/// Trotting time, in seconds.
const TROT_S: f64 = 8.0;
const START_X_M: f64 = -2.5;
const TERRAIN_SEED: u64 = 2024;

struct Scene {
    model: ArticulatedModel,
    feet: Vec<(Entity, usize)>,
    /// Actuated indices of each leg's hip, thigh, and calf joints, in [`LEGS`] order.
    legs: Vec<[usize; 3]>,
}

#[derive(Debug, Default, PartialEq)]
struct Report {
    final_q: Vec<f64>,
    distance_m: f64,
    lateral_drift_m: f64,
    min_clearance_m: f64,
    max_tilt_rad: f64,
    deepest_penetration_m: f64,
    saturated_steps: usize,
    peak_torque_ratio: f64,
    mean_solver_iterations: f64,
    unconverged_steps: usize,
}

fn build_scene() -> Scene {
    let document = parse_urdf_document(GO2_URDF).expect("parse Go2 URDF");
    let mut world = World::new();
    let config = UrdfSpawnConfig {
        attach_colliders: false,
        attach_mesh_colliders: false,
        self_collisions: false,
        use_declared_inertial_masses: true,
        ..UrdfSpawnConfig::default()
    };
    let spawned =
        spawn_urdf_document_with_config(&mut world, &document, config).expect("spawn Go2");
    let base_link = world
        .get::<Robot>(spawned.robot)
        .expect("robot component")
        .base_link;
    // The vendored Go2 URDF is Z-up; rotate the base into RNE's Y-up world.
    world.entity_mut(base_link).insert((
        Transform3::from_translation_rotation(
            Vec3::ZERO,
            Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2),
        ),
        FloatingBase,
    ));
    let model = ArticulatedModel::from_robot(&world, spawned.robot).expect("articulated model");
    let actuated_index = |joint: &str| {
        let entity = spawned.joints.get(joint).expect("leg joint");
        model
            .kinematic()
            .dof_index_of_joint(*entity)
            .expect("actuated leg joint")
    };
    let legs = LEGS
        .iter()
        .map(|leg| {
            [
                actuated_index(&format!("{leg}_hip_joint")),
                actuated_index(&format!("{leg}_thigh_joint")),
                actuated_index(&format!("{leg}_calf_joint")),
            ]
        })
        .collect();
    let feet = LEGS
        .iter()
        .map(|leg| {
            let entity = model
                .kinematic()
                .link_entity_by_name(&format!("{leg}_foot"))
                .expect("foot link");
            let index = model.kinematic().link_index(entity).expect("foot index");
            (entity, index)
        })
        .collect();
    Scene { model, feet, legs }
}

/// Joint targets of the trot at time `t` (stand pose before the gait starts).
fn gait_targets(scene: &Scene, t: f64) -> Vec<f64> {
    let actuated = scene.model.nv() - scene.model.base_dof();
    let mut targets = vec![0.0; actuated];
    // Ramp the stride in over the first period so the gait starts smoothly.
    let ramp = ((t - SETTLE_S) / PERIOD_S).clamp(0.0, 1.0);
    for (leg, joints) in scene.legs.iter().enumerate() {
        let phase = std::f64::consts::TAU * ((t - SETTLE_S) / PERIOD_S + LEG_PHASE[leg]);
        // The first half of each cycle is swing, the second half stance. A
        // smaller thigh angle puts the foot further forward, so the thigh
        // closes during swing and opens during stance.
        let swing = phase.sin().max(0.0);
        targets[joints[0]] = STAND_HIP_RAD;
        targets[joints[1]] = STAND_THIGH_RAD + ramp * THIGH_SWEEP_RAD * phase.cos();
        targets[joints[2]] = STAND_CALF_RAD - ramp * CALF_LIFT_RAD * swing;
    }
    targets
}

/// One sphere contact per foot against the terrain surface beneath it.
fn foot_contacts(scene: &Scene, terrain: &ColliderShape, q: &[f64]) -> Vec<ContactPoint> {
    let kinematics = scene
        .model
        .kinematic()
        .forward_kinematics(q)
        .expect("forward kinematics");
    scene
        .feet
        .iter()
        .map(|(link, index)| {
            let foot = kinematics.transforms()[*index];
            let center = foot.translation + foot.rotation * FOOT_CENTER_LOCAL_M;
            let surface =
                height_field_surface(terrain, center.x, center.z).expect("foot over the terrain");
            let normal = surface.normal;
            let clearance = (center.y - surface.height_m) * normal.y;
            let point_world = center - normal * FOOT_RADIUS_M;
            ContactPoint {
                link: *link,
                point_local_m: foot.rotation.conjugate() * (point_world - foot.translation),
                normal_world: normal,
                gap_m: clearance - FOOT_RADIUS_M,
                friction_coefficient: FRICTION_COEFFICIENT,
            }
        })
        .collect()
}

fn simulate(scene: &Scene, terrain: &ColliderShape) -> Report {
    let model = &scene.model;
    let nv = model.nv();
    let base = model.base_dof();
    let actuated = nv - base;
    let mut q = vec![0.0; nv];
    q[base..].copy_from_slice(&gait_targets(scene, 0.0));
    q[0] = START_X_M;
    let start = height_field_surface(terrain, START_X_M, 0.0).expect("start on terrain");
    q[1] = start.height_m + 0.36;
    let mut qd = vec![0.0; nv];
    let tau = vec![0.0; nv];
    let config = ContactStepConfig {
        step_time_s: STEP_TIME_S,
        ..ContactStepConfig::default()
    };
    let limits: Vec<f64> = (0..actuated)
        .map(|joint| {
            model
                .joint_effort_limit(base + joint)
                .unwrap_or(f64::INFINITY)
        })
        .collect();

    let steps = ((SETTLE_S + TROT_S) / STEP_TIME_S).round() as usize;
    let settle_steps = (SETTLE_S / STEP_TIME_S).round() as usize;
    let mut warm: Option<Vec<[f64; 3]>> = None;
    let mut report = Report {
        min_clearance_m: f64::INFINITY,
        ..Report::default()
    };
    let mut total_iterations = 0;
    let mut gait_start = None;
    for index in 0..steps {
        let t = index as f64 * STEP_TIME_S;
        let pd = JointPdControl {
            position_gains: vec![POSITION_GAIN_NM_RAD; actuated],
            velocity_gains: vec![VELOCITY_GAIN_NM_S_RAD; actuated],
            target_positions: gait_targets(scene, t),
            target_velocities: vec![0.0; actuated],
        };
        let contacts = foot_contacts(scene, terrain, &q);
        for contact in &contacts {
            report.deepest_penetration_m = report.deepest_penetration_m.min(contact.gap_m);
        }
        let step = contact_step(
            model,
            &q,
            &qd,
            &tau,
            &contacts,
            Some(&pd),
            &config,
            warm.as_deref(),
        )
        .expect("contact step");
        total_iterations += step.solver_iterations;
        if !step.solver_converged {
            report.unconverged_steps += 1;
        }
        if !step.saturated_joints.is_empty() {
            report.saturated_steps += 1;
        }
        for (torque, limit) in step.actuator_torques.iter().zip(&limits) {
            report.peak_torque_ratio = report.peak_torque_ratio.max(torque.abs() / limit);
        }
        warm = Some(
            step.contacts
                .iter()
                .zip(&contacts)
                .map(|(outcome, contact)| {
                    let [t1, t2, n] = contact_frame(contact.normal_world);
                    let impulse = outcome.impulse_world_n_s;
                    [impulse.dot(t1), impulse.dot(t2), impulse.dot(n)]
                })
                .collect(),
        );
        q = step.q;
        qd = step.qd;
        if index + 1 == settle_steps {
            gait_start = Some((q[0], q[2]));
        }
        let ground = height_field_surface(terrain, q[0], q[2]).expect("base over terrain");
        report.min_clearance_m = report.min_clearance_m.min(q[1] - ground.height_m);
        // In the Y-up world the base rolls about X and pitches about Z; q[4]
        // (about Y) is its heading.
        report.max_tilt_rad = report.max_tilt_rad.max(q[3].abs()).max(q[5].abs());
    }
    let (start_x, start_z) = gait_start.expect("gait started");
    report.distance_m = q[0] - start_x;
    report.lateral_drift_m = (q[2] - start_z).abs();
    report.mean_solver_iterations = total_iterations as f64 / steps as f64;
    report.final_q = q;
    report
}

fn main() {
    let spec = FractalTerrain {
        size_x_m: 8.0,
        size_z_m: 4.0,
        columns: 161,
        rows: 81,
        frequency_per_m: 0.5,
        amplitude_m: 0.04,
        ..FractalTerrain::default()
    };
    let terrain = spec.height_field(TERRAIN_SEED).expect("terrain");
    let ColliderShape::HeightField { heights_m, .. } = &terrain else {
        unreachable!("height_field returns a height field");
    };
    let relief = heights_m.iter().copied().fold(f64::NEG_INFINITY, f64::max)
        - heights_m.iter().copied().fold(f64::INFINITY, f64::min);
    println!(
        "terrain: {:.1}x{:.1} m, seed {TERRAIN_SEED}, relief {relief:.3} m",
        spec.size_x_m, spec.size_z_m
    );

    let scene = build_scene();
    let report = simulate(&scene, &terrain);
    let again = simulate(&scene, &terrain);
    let deterministic = report == again;

    let speed = report.distance_m / TROT_S;
    println!(
        "trot: {:.2} s period over {TROT_S:.1} s, {:.2} m forward ({speed:.2} m/s), {:.2} m lateral drift",
        PERIOD_S, report.distance_m, report.lateral_drift_m
    );
    println!(
        "body: min clearance {:.3} m, max roll/pitch {:.3} rad",
        report.min_clearance_m, report.max_tilt_rad
    );
    println!(
        "feet: deepest penetration {:.2} mm",
        -1.0e3 * report.deepest_penetration_m
    );
    println!(
        "actuators: peak torque {:.0} % of the effort limit, saturated in {} of {} steps",
        100.0 * report.peak_torque_ratio,
        report.saturated_steps,
        ((SETTLE_S + TROT_S) / STEP_TIME_S).round() as usize
    );
    println!(
        "solver: mean {:.1} sweeps, unconverged steps {}",
        report.mean_solver_iterations, report.unconverged_steps
    );
    println!("deterministic replay: {deterministic}");

    let ok = deterministic
        && report.distance_m > 1.0
        && report.min_clearance_m > 0.18
        && report.max_tilt_rad < 0.4
        && report.deepest_penetration_m > -0.01
        && report.peak_torque_ratio <= 1.0 + 1.0e-9
        && report.final_q.iter().all(|value| value.is_finite());
    if !ok {
        eprintln!("go2 terrain trot: failed");
        std::process::exit(1);
    }
    println!("go2 terrain trot: ok");
}
