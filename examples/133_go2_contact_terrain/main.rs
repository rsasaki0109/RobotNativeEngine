//! Drops the 12-DoF Unitree Go2 onto seeded fractal terrain and lets it stand,
//! with hard contacts, an exact friction cone, and implicit joint PD.
//!
//! This is the RaiSim-style simulation loop built only from backend-neutral
//! pieces: [`rne_physics::FractalTerrain`] generates the ground,
//! [`rne_physics::height_field_surface`] places one sphere contact per foot on
//! it, and [`rne_dynamics::contact_step`] advances the floating-base model with
//! the per-contact bisection solver and stand-pose PD folded into the effective
//! mass. No physics backend, renderer, or wall clock is involved, and the run is
//! repeated to check that it is bit-for-bit deterministic.
//!
//! Run with `cargo run -p go2_contact_terrain --example 133_go2_contact_terrain`.

use rne_dynamics::{
    contact_frame, contact_step, ArticulatedModel, ContactPoint, ContactState, ContactStepConfig,
    JointPdControl,
};
use rne_ecs::{Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{height_field_surface, ColliderShape, FractalTerrain};
use rne_robot::{FloatingBase, Robot};
use rne_urdf_import::{parse_urdf_document, spawn_urdf_document_with_config, UrdfSpawnConfig};
use rne_world::Transform3;

const GO2_URDF: &str = include_str!("../../assets/robots/go2_description/go2_description.rne.urdf");
const FOOT_LINKS: [&str; 4] = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"];
/// Foot collision sphere from the Go2 URDF.
const FOOT_RADIUS_M: f64 = 0.022;
const FOOT_CENTER_LOCAL_M: Vec3 = Vec3::new(-0.002, 0.0, 0.0);
const FRICTION_COEFFICIENT: f64 = 0.8;
/// Stand pose per leg: hip, thigh, calf, in radians.
const STAND_POSE_RAD: [(&str, f64); 3] = [("hip", 0.0), ("thigh", 0.8), ("calf", -1.5)];
const POSITION_GAIN_NM_RAD: f64 = 120.0;
const VELOCITY_GAIN_NM_S_RAD: f64 = 6.0;
const STEP_TIME_S: f64 = 0.002;
const DURATION_S: f64 = 2.0;
const DROP_HEIGHT_M: f64 = 0.42;
const TERRAIN_SEED: u64 = 2018;

struct Scene {
    model: ArticulatedModel,
    feet: Vec<(Entity, usize)>,
    pd: JointPdControl,
    stand_q: Vec<f64>,
}

#[derive(Debug, Default, PartialEq)]
struct Report {
    final_q: Vec<f64>,
    final_qd: Vec<f64>,
    base_clearance_m: f64,
    vertical_force_n: f64,
    weight_n: f64,
    deepest_penetration_m: f64,
    max_solver_iterations: usize,
    mean_solver_iterations: f64,
    unconverged_steps: usize,
    stance: Vec<ContactState>,
    max_joint_error_rad: f64,
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
    let base = model.base_dof();
    let actuated = model.nv() - base;

    let mut stand_q = vec![0.0; model.nv()];
    for (name, joint) in &spawned.joints {
        let Some(dof) = model.kinematic().dof_index_of_joint(*joint) else {
            continue;
        };
        if let Some((_, angle)) = STAND_POSE_RAD
            .iter()
            .find(|(part, _)| name.ends_with(&format!("_{part}_joint")))
        {
            stand_q[base + dof] = *angle;
        }
    }
    let feet = FOOT_LINKS
        .iter()
        .map(|name| {
            let entity = model
                .kinematic()
                .link_entity_by_name(name)
                .expect("foot link");
            let index = model.kinematic().link_index(entity).expect("foot index");
            (entity, index)
        })
        .collect();
    let pd = JointPdControl {
        position_gains: vec![POSITION_GAIN_NM_RAD; actuated],
        velocity_gains: vec![VELOCITY_GAIN_NM_S_RAD; actuated],
        target_positions: stand_q[base..].to_vec(),
        target_velocities: vec![0.0; actuated],
        effort_limits: Vec::new(),
    };
    Scene {
        model,
        feet,
        pd,
        stand_q,
    }
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
    let mut q = scene.stand_q.clone();
    let start = height_field_surface(terrain, 0.0, 0.0).expect("origin on terrain");
    q[1] = start.height_m + DROP_HEIGHT_M;
    let mut qd = vec![0.0; nv];
    let tau = vec![0.0; nv];
    let config = ContactStepConfig {
        step_time_s: STEP_TIME_S,
        ..ContactStepConfig::default()
    };

    let steps = (DURATION_S / STEP_TIME_S).round() as usize;
    let mut warm: Option<Vec<[f64; 3]>> = None;
    let mut report = Report::default();
    let mut total_iterations = 0;
    let mut last = None;
    for _ in 0..steps {
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
            Some(&scene.pd),
            &config,
            warm.as_deref(),
        )
        .expect("contact step");
        total_iterations += step.solver_iterations;
        report.max_solver_iterations = report.max_solver_iterations.max(step.solver_iterations);
        if !step.solver_converged {
            report.unconverged_steps += 1;
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
        q.clone_from(&step.q);
        qd.clone_from(&step.qd);
        last = Some(step);
    }
    let last = last.expect("at least one step");

    let mass_kg: f64 = (0..model.link_count())
        .filter_map(|index| model.link_inertia(index))
        .map(|inertia| inertia.mass_kg)
        .sum();
    report.weight_n = mass_kg * 9.81;
    report.vertical_force_n = last
        .contacts
        .iter()
        .map(|outcome| outcome.impulse_world_n_s.y / STEP_TIME_S)
        .sum();
    report.stance = last.contacts.iter().map(|outcome| outcome.state).collect();
    let ground = height_field_surface(terrain, q[0], q[2]).expect("base over terrain");
    report.base_clearance_m = q[1] - ground.height_m;
    report.max_joint_error_rad = q[model.base_dof()..]
        .iter()
        .zip(&scene.pd.target_positions)
        .map(|(actual, target)| (actual - target).abs())
        .fold(0.0, f64::max);
    report.mean_solver_iterations = total_iterations as f64 / steps as f64;
    report.final_q = q;
    report.final_qd = qd;
    report
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
    let ColliderShape::HeightField { heights_m, .. } = &terrain else {
        unreachable!("height_field returns a height field");
    };
    let relief = heights_m.iter().copied().fold(f64::NEG_INFINITY, f64::max)
        - heights_m.iter().copied().fold(f64::INFINITY, f64::min);
    println!(
        "terrain: {}x{} samples over {:.1}x{:.1} m, seed {TERRAIN_SEED}, relief {relief:.3} m",
        spec.columns, spec.rows, spec.size_x_m, spec.size_z_m
    );

    let scene = build_scene();
    let report = simulate(&scene, &terrain);
    let again = simulate(&scene, &terrain);
    let deterministic = report == again;

    let speed = report.final_qd.iter().map(|v| v * v).sum::<f64>().sqrt();
    println!(
        "go2: nv={} dropped {DROP_HEIGHT_M:.2} m, simulated {DURATION_S:.1} s at {:.0} Hz",
        scene.model.nv(),
        1.0 / STEP_TIME_S
    );
    println!(
        "stance: {:?}, base clearance {:.3} m, residual speed {speed:.2e}",
        report.stance, report.base_clearance_m
    );
    println!(
        "support: vertical contact force {:.2} N vs weight {:.2} N",
        report.vertical_force_n, report.weight_n
    );
    println!(
        "contacts: deepest penetration {:.2} mm, max joint error {:.4} rad",
        -1.0e3 * report.deepest_penetration_m,
        report.max_joint_error_rad
    );
    println!(
        "solver: mean {:.1} / max {} sweeps, unconverged steps {}",
        report.mean_solver_iterations, report.max_solver_iterations, report.unconverged_steps
    );
    println!("deterministic replay: {deterministic}");

    let standing = report
        .stance
        .iter()
        .all(|state| *state != ContactState::Open);
    let supported = (report.vertical_force_n - report.weight_n).abs() < 0.02 * report.weight_n;
    let ok = deterministic
        && standing
        && supported
        && report.base_clearance_m > 0.2
        && report.deepest_penetration_m > -0.005
        && speed < 1.0e-2
        && report.final_q.iter().all(|value| value.is_finite());
    if !ok {
        eprintln!("go2 contact terrain: failed");
        std::process::exit(1);
    }
    println!("go2 contact terrain: ok");
}
