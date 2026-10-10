//! Sensor-feedback standing controller for the dedicated physical G1 plant.
//!
//! This is a bounded ankle/hip strategy, not a learned Unitree policy or a
//! hardware-qualified whole-body controller. The stance creates a measured
//! support polygon; the DCM and IMU feedback respond to observed motion only.

use super::model::center_of_mass;
use rne_ai::{UrdfJointPositionTarget, UrdfSceneSim};
use rne_math::Vec3;
use rne_robot::{Joint, JointKind, JointLimits, Link};
use serde_json::{json, Value};
use std::{error::Error, io};

type ControllerResult<T> = Result<T, Box<dyn Error>>;

const STANCE_ROLL_RAD: f64 = 0.12;
const POSTURE_STIFFNESS_NM_PER_RAD: f64 = 300.0;
const POSTURE_DAMPING_NM_S_PER_RAD: f64 = 12.0;
const ANKLE_STIFFNESS_NM_PER_RAD: f64 = 250.0;
const ANKLE_DAMPING_NM_S_PER_RAD: f64 = 8.0;
const LATERAL_CAPTURE_GAIN_RAD_PER_M: f64 = 0.5;
const FORWARD_CAPTURE_GAIN_RAD_PER_M: f64 = 0.8;
const ATTITUDE_GAIN: f64 = 0.5;
const ANGULAR_RATE_HORIZON_S: f64 = 0.1;
const HIP_CORRECTION_LIMIT_RAD: f64 = 0.20;
const ANKLE_CORRECTION_LIMIT_RAD: f64 = 0.15;
const TARGET_RATE_LIMIT_RAD_S: f64 = 6.0;
const FOOT_NAMES: [&str; 2] = ["left_ankle_roll_link", "right_ankle_roll_link"];

#[derive(Debug)]
struct Servo {
    name: String,
    limits: JointLimits,
    previous_target_rad: f64,
}

/// State and bounded joint servos for one deterministic G1 replay.
#[derive(Debug)]
pub(super) struct G1Controller {
    servos: Vec<Servo>,
    balance_feedback_enabled: bool,
    last_feedback: Value,
}

impl G1Controller {
    /// Configures unit-bearing implicit PD motors and the authored effort caps.
    ///
    /// Disabling balance feedback retains exactly the same posture, stance,
    /// motor gains, effort ceilings, and target-rate limit for the ablation.
    pub(super) fn new(
        sim: &mut UrdfSceneSim,
        balance_feedback_enabled: bool,
    ) -> ControllerResult<Self> {
        let mut servos: Vec<_> = sim
            .world()
            .iter_entities()
            .filter_map(|entity| {
                let joint = entity.get::<Joint>()?;
                (joint.kind != JointKind::Fixed).then(|| {
                    let link = sim
                        .world()
                        .get::<Link>(joint.child_link)
                        .expect("joint link");
                    Servo {
                        name: link.name.clone(),
                        limits: joint.limits,
                        previous_target_rad: joint.position,
                    }
                })
            })
            .collect();
        servos.sort_unstable_by(|a, b| a.name.cmp(&b.name));
        if servos.len() != 23 {
            return Err(
                io::Error::other("the G1 controller requires its 23 authored joints").into(),
            );
        }
        for servo in &servos {
            let ankle = servo.name.contains("ankle_");
            if !sim.configure_named_revolute_position_actuation(
                &servo.name,
                if ankle {
                    ANKLE_STIFFNESS_NM_PER_RAD
                } else {
                    POSTURE_STIFFNESS_NM_PER_RAD
                },
                if ankle {
                    ANKLE_DAMPING_NM_S_PER_RAD
                } else {
                    POSTURE_DAMPING_NM_S_PER_RAD
                },
                servo.limits.max_effort,
            ) {
                return Err(io::Error::other("a G1 unit-bearing position servo is invalid").into());
            }
        }
        Ok(Self {
            servos,
            balance_feedback_enabled,
            last_feedback: Value::Null,
        })
    }

    /// Measures the current state, updates bounded targets, and advances one tick.
    ///
    /// No disturbance timing, force magnitude, render data, or future state is
    /// available to this controller. The simulation owns its current fixed step.
    pub(super) fn step(&mut self, sim: &mut UrdfSceneSim) -> ControllerResult<()> {
        let (com_m, com_velocity_m_s, _) = center_of_mass(sim)?;
        let pelvis = sim
            .named_transform("pelvis")
            .ok_or_else(|| io::Error::other("G1 pelvis is missing"))?;
        let feet: Vec<_> = FOOT_NAMES
            .iter()
            .map(|name| {
                sim.named_transform(name)
                    .map(|transform| transform.translation)
                    .ok_or_else(|| io::Error::other("G1 foot is missing"))
            })
            .collect::<Result<_, _>>()?;
        let stance_center_m = (feet[0] + feet[1]) * 0.5;
        let omega_rad_s = (9.81 / com_m.y.max(0.4)).sqrt();
        let dcm_m = com_m + com_velocity_m_s / omega_rad_s;
        let capture_point_m = Vec3::new(dcm_m.x, 0.0, dcm_m.z);
        let capture_error_m = capture_point_m - stance_center_m;
        let observation = sim.observe();
        let up_world = pelvis.rotation * Vec3::Z;
        let enabled = if self.balance_feedback_enabled {
            1.0
        } else {
            0.0
        };
        let hip_lateral_rad = (enabled * LATERAL_CAPTURE_GAIN_RAD_PER_M * capture_error_m.z)
            .clamp(-HIP_CORRECTION_LIMIT_RAD, HIP_CORRECTION_LIMIT_RAD);
        let hip_forward_rad = (enabled * FORWARD_CAPTURE_GAIN_RAD_PER_M * capture_error_m.x)
            .clamp(-ANKLE_CORRECTION_LIMIT_RAD, ANKLE_CORRECTION_LIMIT_RAD);
        let ankle_lateral_rad = (enabled
            * ATTITUDE_GAIN
            * (up_world.z.atan2(up_world.y)
                + ANGULAR_RATE_HORIZON_S * observation.base_angular_velocity_x_rad_s))
            .clamp(-ANKLE_CORRECTION_LIMIT_RAD, ANKLE_CORRECTION_LIMIT_RAD);
        let ankle_forward_rad = (enabled
            * ATTITUDE_GAIN
            * (up_world.x.atan2(up_world.y)
                - ANGULAR_RATE_HORIZON_S * observation.base_angular_velocity_z_rad_s))
            .clamp(-ANKLE_CORRECTION_LIMIT_RAD, ANKLE_CORRECTION_LIMIT_RAD);
        let step_s = sim.fixed_delta().as_seconds().value();
        let targets: Vec<_> = self
            .servos
            .iter_mut()
            .map(|servo| {
                let mut position_rad = corrected_position_rad(
                    &servo.name,
                    hip_lateral_rad,
                    hip_forward_rad,
                    ankle_lateral_rad,
                    ankle_forward_rad,
                );
                let rate_rad_s = TARGET_RATE_LIMIT_RAD_S.min(servo.limits.max_velocity);
                position_rad = position_rad
                    .clamp(servo.limits.lower, servo.limits.upper)
                    .clamp(
                        servo.previous_target_rad - rate_rad_s * step_s,
                        servo.previous_target_rad + rate_rad_s * step_s,
                    );
                servo.previous_target_rad = position_rad;
                UrdfJointPositionTarget {
                    link_name: servo.name.as_str(),
                    position: position_rad,
                }
            })
            .collect();
        let support_points: Vec<_> = FOOT_NAMES
            .iter()
            .flat_map(|name| sim.named_body_contact_points_m(name))
            .map(|point| point.to_array())
            .collect();
        self.last_feedback = json!({
            "balance_feedback_enabled": self.balance_feedback_enabled,
            "com_world_m": com_m.to_array(),
            "com_velocity_world_m_s": com_velocity_m_s.to_array(),
            "capture_point_world_m": capture_point_m.to_array(),
            "stance_center_world_m": stance_center_m.to_array(),
            "actual_support_points_world_m": support_points,
            "hip_lateral_correction_rad": hip_lateral_rad,
            "hip_forward_correction_rad": hip_forward_rad,
            "ankle_lateral_correction_rad": ankle_lateral_rad,
            "ankle_forward_correction_rad": ankle_forward_rad,
            "target_rate_limit_rad_s": TARGET_RATE_LIMIT_RAD_S,
        });
        sim.step_joint_position_actuation_targets(&targets);
        Ok(())
    }

    /// Returns the last observed feedback; the values precede the completed tick.
    pub(super) fn telemetry(&self) -> Value {
        self.last_feedback.clone()
    }

    /// Describes solver-enforced servo limits without claiming measured torque.
    pub(super) fn configuration(&self) -> Value {
        json!({
            "controller": "bounded DCM and IMU ankle/hip standing feedback",
            "motor_gain_model": "ForceBased implicit PD; gains have N m units",
            "balance_feedback_enabled": self.balance_feedback_enabled,
            "stance_hip_roll_rad": STANCE_ROLL_RAD,
            "lateral_capture_gain_rad_per_m": LATERAL_CAPTURE_GAIN_RAD_PER_M,
            "forward_capture_gain_rad_per_m": FORWARD_CAPTURE_GAIN_RAD_PER_M,
            "attitude_gain_rad_per_rad": ATTITUDE_GAIN,
            "angular_rate_horizon_s": ANGULAR_RATE_HORIZON_S,
            "lateral_hip_correction_limit_rad": HIP_CORRECTION_LIMIT_RAD,
            "forward_hip_correction_limit_rad": ANKLE_CORRECTION_LIMIT_RAD,
            "ankle_correction_limit_rad": ANKLE_CORRECTION_LIMIT_RAD,
            "posture_stiffness_nm_per_rad": POSTURE_STIFFNESS_NM_PER_RAD,
            "posture_damping_nm_s_per_rad": POSTURE_DAMPING_NM_S_PER_RAD,
            "ankle_stiffness_nm_per_rad": ANKLE_STIFFNESS_NM_PER_RAD,
            "ankle_damping_nm_s_per_rad": ANKLE_DAMPING_NM_S_PER_RAD,
            "limits": self.servos.iter().map(|servo| json!({
                "link": servo.name,
                "solver_enforced_effort_ceiling_nm": servo.limits.max_effort,
                "authored_velocity_limit_rad_s": servo.limits.max_velocity,
                "target_rate_limit_rad_s": TARGET_RATE_LIMIT_RAD_S.min(servo.limits.max_velocity),
            })).collect::<Vec<_>>(),
        })
    }
}

fn nominal_position_rad(name: &str) -> f64 {
    match name {
        "left_hip_pitch_link"
        | "right_hip_pitch_link"
        | "left_ankle_pitch_link"
        | "right_ankle_pitch_link" => -0.18,
        "left_knee_link" | "right_knee_link" => 0.36,
        "left_elbow_link" | "right_elbow_link" => 0.42,
        _ => 0.0,
    }
}

fn corrected_position_rad(
    name: &str,
    hip_lateral_rad: f64,
    hip_forward_rad: f64,
    ankle_lateral_rad: f64,
    ankle_forward_rad: f64,
) -> f64 {
    let mut position_rad = nominal_position_rad(name);
    match name {
        "left_hip_roll_link" => position_rad += STANCE_ROLL_RAD - hip_lateral_rad,
        "right_hip_roll_link" => position_rad -= STANCE_ROLL_RAD + hip_lateral_rad,
        "left_ankle_roll_link" => {
            position_rad += hip_lateral_rad + ankle_lateral_rad - STANCE_ROLL_RAD;
        }
        "right_ankle_roll_link" => {
            position_rad += hip_lateral_rad + ankle_lateral_rad + STANCE_ROLL_RAD;
        }
        "left_hip_pitch_link" | "right_hip_pitch_link" => {
            position_rad -= hip_forward_rad;
        }
        "left_ankle_pitch_link" | "right_ankle_pitch_link" => {
            position_rad += hip_forward_rad + ankle_forward_rad;
        }
        _ => {}
    }
    position_rad
}
