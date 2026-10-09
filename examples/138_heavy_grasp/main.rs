//! A two-finger gripper squeezes, lifts, and holds heavy cubes through the
//! backend-neutral `PhysicsBackend` interface, on the native articulated
//! backend and on Rapier for comparison.
//!
//! The gripper is a multibody: a palm on a vertical prismatic lift under a
//! fixed anchor, with a PD position servo, and two finger pads on horizontal
//! prismatic slides, each pushing inward with a constant effort. Each cube
//! rests on the floor; everything has friction 0.8. The fingers close, the
//! palm lifts 0.2 m over 1 s, and holds for 1 s.
//!
//! Friction holds a cube once both pads together press with more than its
//! weight over the friction coefficient. Each cube is squeezed with a 25 %
//! margin over that, and again with 35 % too little. The native run must lift
//! every held cube with the palm, with no slip and no twist, leave every
//! under-squeezed cube on the floor, and replay bit for bit.
//!
//! Run with `cargo run --release -p heavy_grasp --example 138_heavy_grasp`.

use rne_core::SimDuration;
use rne_ecs::{Entity, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    Collider, JointActuation, JointMotorGainModel, MultibodyLink, PhysicsBackend, PhysicsMaterial,
    PhysicsWorldDesc, PrismaticJointDesc, RigidBody, RigidBodyType,
};
use rne_physics_native::NativeBackend;
use rne_physics_rapier::RapierBackend;
use rne_world::Transform3;
use std::time::Instant;

const STEP_HZ: f64 = 500.0;
const GRAVITY_M_S2: f64 = 9.81;
const FRICTION: f32 = 0.8;
const CUBE_HALF_M: f64 = 0.05;
const CUBE_MASSES_KG: [f64; 3] = [5.0, 20.0, 50.0];
/// Height of the palm's center above the floor while it reaches down.
const PALM_Y_M: f64 = 0.26;
/// The finger pads hang this far below the palm's center.
const FINGER_DROP_M: f64 = 0.19;
const FINGER_X_M: f64 = 0.08;
const LIFT_M: f64 = 0.2;
const SETTLE_STEPS: usize = 250;
const LIFT_STEPS: usize = 500;
const HOLD_STEPS: usize = 500;

#[derive(Debug, PartialEq)]
struct Report {
    cube_pose: Transform3,
    /// How far the cube rose off the floor, in meters.
    lifted_m: f64,
    /// How far the palm rose, in meters.
    palm_rise_m: f64,
    /// How far the cube fell behind the palm, in meters.
    slip_m: f64,
    /// Tilt of the cube at the end, in radians.
    tilt_rad: f64,
    step_us: f64,
}

fn collider(half: Vec3) -> Collider {
    Collider {
        material: PhysicsMaterial {
            friction: FRICTION,
            ..PhysicsMaterial::default()
        },
        ..Collider::cuboid(half)
    }
}

/// The palm's lift servo at `target_m` above where it starts.
fn lift(target_m: f64) -> JointActuation {
    JointActuation::PrismaticPosition {
        target_position_m: target_m,
        stiffness_n_per_m: 1.0e5,
        damping_n_s_per_m: 1.0e4,
        max_force_n: 5000.0,
    }
}

/// The force per finger that holds a cube of `mass_kg` without margin.
fn holding_squeeze_n(mass_kg: f64) -> f64 {
    mass_kg * GRAVITY_M_S2 / (2.0 * f64::from(FRICTION))
}

/// Grips a cube of `mass_kg` with `squeeze_n` per finger, lifts it, and
/// holds it on `backend`.
fn grasp<B: PhysicsBackend>(mut backend: B, mass_kg: f64, squeeze_n: f64) -> Report {
    let mut world = World::new();
    world.spawn((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        collider(Vec3::new(5.0, 0.5, 5.0)),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
    ));
    let cube = world
        .spawn((
            RigidBody {
                mass_kg,
                ..RigidBody::default()
            },
            collider(Vec3::splat(CUBE_HALF_M)),
            Transform3::from_translation_rotation(Vec3::new(0.0, CUBE_HALF_M, 0.0), Quat::IDENTITY),
        ))
        .id();
    let anchor_y_m = 1.0;
    let anchor = world
        .spawn((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            MultibodyLink,
            Transform3::from_translation_rotation(Vec3::new(0.0, anchor_y_m, 0.0), Quat::IDENTITY),
        ))
        .id();
    // Joint coordinates are zero in the spawn pose: each joint's parent
    // anchor sits where its child starts.
    let palm = world
        .spawn((
            RigidBody {
                mass_kg: 1.0,
                ..RigidBody::default()
            },
            collider(Vec3::new(0.12, 0.02, 0.05)),
            MultibodyLink,
            Transform3::from_translation_rotation(Vec3::new(0.0, PALM_Y_M, 0.0), Quat::IDENTITY),
            PrismaticJointDesc {
                parent: anchor,
                axis: Vec3::Y,
                anchor_parent_m: Vec3::new(0.0, PALM_Y_M - anchor_y_m, 0.0),
                anchor_child_m: Vec3::ZERO,
                relative_rotation: Quat::IDENTITY,
                lower_m: None,
                upper_m: None,
            },
            lift(0.0),
            JointMotorGainModel::ForceBased,
        ))
        .id();
    for side in [-1.0, 1.0] {
        world.spawn((
            RigidBody {
                mass_kg: 0.2,
                ..RigidBody::default()
            },
            collider(Vec3::new(0.01, 0.06, 0.04)),
            MultibodyLink,
            Transform3::from_translation_rotation(
                Vec3::new(side * FINGER_X_M, PALM_Y_M - FINGER_DROP_M, 0.0),
                Quat::IDENTITY,
            ),
            PrismaticJointDesc {
                parent: palm,
                axis: Vec3::X,
                anchor_parent_m: Vec3::new(side * FINGER_X_M, -FINGER_DROP_M, 0.0),
                anchor_child_m: Vec3::ZERO,
                relative_rotation: Quat::IDENTITY,
                lower_m: Some(-0.05),
                upper_m: Some(0.05),
            },
            JointActuation::PrismaticEffort {
                force_n: -side * squeeze_n,
                max_force_n: squeeze_n,
            },
        ));
    }

    let id = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    let dt = SimDuration::from_hertz(Hertz::new(STEP_HZ));
    let step = |backend: &mut B, world: &mut World| {
        backend.sync_from_ecs(world, id).expect("sync from ECS");
        backend.step(id, dt).expect("step");
        backend.sync_to_ecs(world, id).expect("sync to ECS");
    };
    let height = |world: &World, entity: Entity| {
        world.get::<Transform3>(entity).expect("pose").translation.y
    };
    let started = Instant::now();
    for _ in 0..SETTLE_STEPS {
        step(&mut backend, &mut world);
    }
    let grip = height(&world, cube) - height(&world, palm);
    let palm_start = height(&world, palm);
    for index in 1..=LIFT_STEPS + HOLD_STEPS {
        let s = (index as f64 / LIFT_STEPS as f64).min(1.0);
        world.entity_mut(palm).insert(lift(LIFT_M * s));
        step(&mut backend, &mut world);
    }
    let steps = SETTLE_STEPS + LIFT_STEPS + HOLD_STEPS;
    let step_us = started.elapsed().as_secs_f64() * 1.0e6 / steps as f64;
    let cube_pose = *world.get::<Transform3>(cube).expect("cube pose");
    let palm_end = height(&world, palm);
    Report {
        cube_pose,
        lifted_m: cube_pose.translation.y - CUBE_HALF_M,
        palm_rise_m: palm_end - palm_start,
        slip_m: grip - (cube_pose.translation.y - palm_end),
        tilt_rad: (cube_pose.rotation * Vec3::Y).y.clamp(-1.0, 1.0).acos(),
        step_us,
    }
}

fn print_row(name: &str, report: &Report) {
    println!(
        "  {name}: lifted {:6.3} m (palm {:5.3} m), slip {:8.1e} m, tilt {:7.1e} rad, {:4.0} us per step",
        report.lifted_m, report.palm_rise_m, report.slip_m, report.tilt_rad, report.step_us
    );
}

fn main() {
    println!(
        "two-finger grasp, friction {FRICTION}, lift {LIFT_M} m over 1 s and hold 1 s at {STEP_HZ:.0} Hz through PhysicsBackend"
    );
    let mut ok = true;
    for mass_kg in CUBE_MASSES_KG {
        let needed_n = holding_squeeze_n(mass_kg);
        for (label, squeeze_n, held) in [
            ("held", 1.25 * needed_n, true),
            ("under-squeezed", 0.65 * needed_n, false),
        ] {
            println!(
                "{mass_kg:.0} kg cube, {squeeze_n:.0} N per finger ({label}; holding needs {needed_n:.0} N):"
            );
            let native = grasp(NativeBackend::new(), mass_kg, squeeze_n);
            print_row("native", &native);
            let rapier = grasp(RapierBackend::new(), mass_kg, squeeze_n);
            print_row("rapier", &rapier);
            ok &= if held {
                (native.lifted_m - native.palm_rise_m).abs() < 1.0e-3
                    && native.palm_rise_m > 0.95 * LIFT_M
                    && native.slip_m.abs() < 1.0e-4
                    && native.tilt_rad < 1.0e-3
            } else {
                native.lifted_m.abs() < 1.0e-3
            };
        }
    }
    let heaviest = CUBE_MASSES_KG[CUBE_MASSES_KG.len() - 1];
    let squeeze_n = 1.25 * holding_squeeze_n(heaviest);
    let deterministic = grasp(NativeBackend::new(), heaviest, squeeze_n).cube_pose
        == grasp(NativeBackend::new(), heaviest, squeeze_n).cube_pose;
    println!("native deterministic replay: {deterministic}");
    if !(ok && deterministic) {
        eprintln!("heavy grasp: failed");
        std::process::exit(1);
    }
    println!("heavy grasp: ok");
}
