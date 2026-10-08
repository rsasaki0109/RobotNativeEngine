//! Drives the Go2 with the feedback trot of `rne_locomotion` over rough
//! fractal terrain: a straight run, a turn well past a quarter turn, and a
//! lateral push, then repeats the course with the open-loop gait of example
//! 134 for comparison.
//!
//! The feedback trot plans its feet in the level frame, lands each swing foot
//! at the Raibert point for the measured velocity, tracks a heading integrated
//! from the yaw-rate command, carries the body's weight with feed-forward
//! stance torques, and leans the foot plan against the measured tilt. The
//! simulation is the native hard-contact step of example 133 in the Z-up
//! locomotion frame, so the heading is unbounded. Both runs are repeated to
//! check that they are deterministic.
//!
//! Run with `cargo run --release -p go2_feedback_trot --example 135_go2_feedback_trot`.

use rne_locomotion::{
    JointCommand, Quadruped, QuadrupedTerrainSim, TerrainSimConfig, TrotController, TrotParams,
    VelocityCommand,
};
use rne_math::{Quat, Vec3};
use rne_physics::{ColliderShape, FractalTerrain};
use std::sync::Arc;

const TERRAIN_SEED: u64 = 135;
const STAND_S: f64 = 0.5;
const STRAIGHT_END_S: f64 = 3.5;
const TURN_END_S: f64 = 7.5;
const END_S: f64 = 10.0;
const FORWARD_M_S: f64 = 0.4;
const TURN_FORWARD_M_S: f64 = 0.3;
const TURN_RATE_RAD_S: f64 = 0.5;
const PUSH_AT_S: f64 = 8.0;
const PUSH_S: f64 = 0.1;
/// Lateral push to the robot's left, in newtons (close to its 266 N weight).
const PUSH_N: f64 = 250.0;
/// Open-loop gait of example 134: thigh sweep and calf lift, in radians.
const OPEN_LOOP_THIGH_RAD: f64 = 0.15;
const OPEN_LOOP_CALF_RAD: f64 = 0.7;
const OPEN_LOOP_PERIOD_S: f64 = 0.36;
const LEG_PHASE: [f64; 4] = [0.0, 0.5, 0.5, 0.0];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Gait {
    Feedback,
    OpenLoop,
}

#[derive(Debug, Default, PartialEq)]
struct Report {
    final_q: Vec<f64>,
    fell_at_s: Option<f64>,
    straight_speed_m_s: f64,
    turn_rad: f64,
    max_tilt_rad: f64,
    push_drift_m: f64,
    tilt_after_push_rad: f64,
    min_clearance_m: f64,
    deepest_penetration_m: f64,
    peak_torque_ratio: f64,
}

fn command_at(t: f64) -> VelocityCommand {
    if t < STRAIGHT_END_S {
        VelocityCommand {
            forward_m_s: FORWARD_M_S,
            ..VelocityCommand::default()
        }
    } else if t < TURN_END_S {
        VelocityCommand {
            forward_m_s: TURN_FORWARD_M_S,
            yaw_rate_rad_s: TURN_RATE_RAD_S,
            ..VelocityCommand::default()
        }
    } else {
        VelocityCommand {
            forward_m_s: FORWARD_M_S,
            ..VelocityCommand::default()
        }
    }
}

fn open_loop_command(go2: &Quadruped, t: f64) -> JointCommand {
    let mut positions = go2.stand_pose_rad().to_vec();
    let ramp = ((t - STAND_S) / OPEN_LOOP_PERIOD_S).clamp(0.0, 1.0);
    for (leg, geometry) in go2.legs().iter().enumerate() {
        let phase = std::f64::consts::TAU * ((t - STAND_S) / OPEN_LOOP_PERIOD_S + LEG_PHASE[leg]);
        positions[geometry.joints[1]] += ramp * OPEN_LOOP_THIGH_RAD * phase.cos();
        positions[geometry.joints[2]] -= ramp * OPEN_LOOP_CALF_RAD * phase.sin().max(0.0);
    }
    JointCommand::positions(positions)
}

fn simulate(go2: &Arc<Quadruped>, terrain: &ColliderShape, gait: Gait) -> Report {
    let mut sim =
        QuadrupedTerrainSim::new(go2.clone(), terrain.clone(), TerrainSimConfig::default())
            .expect("simulation");
    let mut controller = TrotController::new(TrotParams::default());
    let dt = sim.config().step_time_s;
    let mut report = Report {
        min_clearance_m: f64::INFINITY,
        ..Report::default()
    };
    let mut straight_start = None;
    let mut turn_start = None;
    let mut push_start = None;
    for index in 0..(END_S / dt).round() as usize {
        let t = index as f64 * dt;
        let state = sim.base_state();
        if t >= 1.5 && straight_start.is_none() {
            straight_start = Some((t, state.position_m, state.heading_rad()));
        }
        if t >= STRAIGHT_END_S && turn_start.is_none() {
            let (t0, p0, h0) = straight_start.expect("straight run started");
            // Average forward speed along the initial heading.
            let along = Vec3::new(h0.cos(), h0.sin(), 0.0);
            report.straight_speed_m_s = (state.position_m - p0).dot(along) / (t - t0);
            turn_start = Some(state.heading_rad());
        }
        if t >= TURN_END_S && report.turn_rad == 0.0 {
            report.turn_rad = state.heading_rad() - turn_start.expect("turn started");
            if report.turn_rad < -1.0 {
                report.turn_rad += std::f64::consts::TAU;
            }
        }
        let command = if t < STAND_S {
            controller.stand_targets(go2, &state)
        } else {
            match gait {
                Gait::Feedback => controller.update(go2, &state, &command_at(t), dt),
                Gait::OpenLoop => open_loop_command(go2, t),
            }
        };
        let pushing = (PUSH_AT_S..PUSH_AT_S + PUSH_S).contains(&t);
        if pushing && push_start.is_none() {
            push_start = Some(state);
        }
        let push = if pushing {
            Quat::from_rotation_z(state.heading_rad()) * Vec3::new(0.0, PUSH_N, 0.0)
        } else {
            Vec3::ZERO
        };
        let step = sim.step(&command, push).expect("step");
        report.deepest_penetration_m = report.deepest_penetration_m.min(step.deepest_gap_m());
        for (torque, limit) in step.actuator_torques.iter().zip(go2.effort_limits()) {
            report.peak_torque_ratio = report.peak_torque_ratio.max(torque.abs() / limit);
        }
        let state = sim.base_state();
        let clearance = sim.base_clearance_m().expect("base over the terrain");
        report.min_clearance_m = report.min_clearance_m.min(clearance);
        report.max_tilt_rad = report.max_tilt_rad.max(state.tilt_rad());
        if report.fell_at_s.is_none() && (clearance < 0.15 || state.tilt_rad() > 0.8) {
            report.fell_at_s = Some(t);
        }
        if let Some(before) = push_start {
            let after = t - PUSH_AT_S;
            if (0.999..1.001).contains(&after) {
                let left = Quat::from_rotation_z(before.heading_rad()) * Vec3::Y;
                report.push_drift_m = (state.position_m - before.position_m).dot(left);
                report.tilt_after_push_rad = state.tilt_rad();
            }
        }
    }
    report.final_q = sim.q().to_vec();
    report
}

fn print_report(name: &str, report: &Report) {
    println!(
        "{name}: straight {:.2} m/s (command {FORWARD_M_S:.1}), turned {:.2} rad (command {:.1}), max tilt {:.3} rad",
        report.straight_speed_m_s,
        report.turn_rad,
        TURN_RATE_RAD_S * (TURN_END_S - STRAIGHT_END_S),
        report.max_tilt_rad
    );
    match report.fell_at_s {
        Some(t) => println!("{name}: fell at {t:.2} s"),
        None => println!(
            "{name}: {PUSH_N:.0} N push for {PUSH_S:.1} s -> {:.2} m sideways, tilt {:.3} rad one second later",
            report.push_drift_m, report.tilt_after_push_rad
        ),
    }
    println!(
        "{name}: min clearance {:.3} m, deepest penetration {:.2} mm, peak torque {:.0} % of the limit",
        report.min_clearance_m,
        -1.0e3 * report.deepest_penetration_m,
        100.0 * report.peak_torque_ratio
    );
}

fn main() {
    let spec = FractalTerrain {
        size_x_m: 10.0,
        size_z_m: 10.0,
        columns: 201,
        rows: 201,
        frequency_per_m: 0.5,
        amplitude_m: 0.08,
        ..FractalTerrain::default()
    };
    let terrain = spec.height_field(TERRAIN_SEED).expect("terrain");
    let ColliderShape::HeightField { heights_m, .. } = &terrain else {
        unreachable!("height_field returns a height field");
    };
    let relief = heights_m.iter().copied().fold(f64::NEG_INFINITY, f64::max)
        - heights_m.iter().copied().fold(f64::INFINITY, f64::min);
    println!(
        "terrain: {:.0}x{:.0} m, seed {TERRAIN_SEED}, relief {relief:.3} m",
        spec.size_x_m, spec.size_z_m
    );

    let go2 = Arc::new(Quadruped::go2().expect("Go2"));
    let feedback = simulate(&go2, &terrain, Gait::Feedback);
    let deterministic = feedback == simulate(&go2, &terrain, Gait::Feedback);
    let open_loop = simulate(&go2, &terrain, Gait::OpenLoop);
    let open_loop_deterministic = open_loop == simulate(&go2, &terrain, Gait::OpenLoop);
    print_report("feedback trot", &feedback);
    print_report("open-loop trot", &open_loop);
    println!(
        "deterministic replay: {}",
        deterministic && open_loop_deterministic
    );

    let commanded_turn = TURN_RATE_RAD_S * (TURN_END_S - STRAIGHT_END_S);
    let ok = deterministic
        && open_loop_deterministic
        && feedback.fell_at_s.is_none()
        && (feedback.straight_speed_m_s - FORWARD_M_S).abs() < 0.1
        && (feedback.turn_rad - commanded_turn).abs() < 0.4
        && feedback.max_tilt_rad < 0.4
        && feedback.tilt_after_push_rad < 0.15
        && feedback.min_clearance_m > 0.2
        && feedback.deepest_penetration_m > -0.01
        && feedback.peak_torque_ratio <= 1.0 + 1.0e-9
        && feedback.final_q.iter().all(|value| value.is_finite());
    if !ok {
        eprintln!("go2 feedback trot: failed");
        std::process::exit(1);
    }
    println!("go2 feedback trot: ok");
}
