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

const STANCE_ROLL_RAD: f64 = 0.24;
const POSTURE_STIFFNESS_NM_PER_RAD: f64 = 300.0;
const POSTURE_DAMPING_NM_S_PER_RAD: f64 = 12.0;
const ANKLE_STIFFNESS_NM_PER_RAD: f64 = 250.0;
const ANKLE_DAMPING_NM_S_PER_RAD: f64 = 8.0;
const LATERAL_CAPTURE_GAIN_RAD_PER_M: f64 = 0.5;
const FORWARD_CAPTURE_GAIN_RAD_PER_M: f64 = 0.8;
const HIP_ATTITUDE_GAIN: f64 = 0.5;
const HIP_ANGULAR_RATE_HORIZON_S: f64 = 0.05;
const ATTITUDE_GAIN: f64 = 0.5;
const ANGULAR_RATE_HORIZON_S: f64 = 0.1;
const HIP_CORRECTION_LIMIT_RAD: f64 = 0.40;
const ANKLE_CORRECTION_LIMIT_RAD: f64 = 0.15;
const TARGET_RATE_LIMIT_RAD_S: f64 = 6.0;
const ARM_ATTITUDE_GAIN: f64 = 4.0;
const ARM_PREDICTION_DEADBAND_RAD: f64 = 0.02;
const ARM_CORRECTION_LIMIT_RAD: f64 = 1.0;
const ARM_RETURN_RATE_RAD_S: f64 = 0.5;
const ARM_QUIET_DURATION_S: f64 = 0.3;
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
    arm_roll_targets_rad: [f64; 2],
    arm_quiet_duration_s: f64,
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
            arm_roll_targets_rad: [0.0; 2],
            arm_quiet_duration_s: 0.0,
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
        let roll_rad = up_world.z.atan2(up_world.y);
        let hip_lateral_rad = lateral_hip_correction_rad(
            capture_error_m.z,
            roll_rad,
            observation.base_angular_velocity_x_rad_s,
            self.balance_feedback_enabled,
        );
        let hip_forward_rad = (enabled * FORWARD_CAPTURE_GAIN_RAD_PER_M * capture_error_m.x)
            .clamp(-ANKLE_CORRECTION_LIMIT_RAD, ANKLE_CORRECTION_LIMIT_RAD);
        let ankle_lateral_rad = (enabled
            * ATTITUDE_GAIN
            * (roll_rad + ANGULAR_RATE_HORIZON_S * observation.base_angular_velocity_x_rad_s))
            .clamp(-ANKLE_CORRECTION_LIMIT_RAD, ANKLE_CORRECTION_LIMIT_RAD);
        let ankle_forward_rad = (enabled
            * ATTITUDE_GAIN
            * (up_world.x.atan2(up_world.y)
                - ANGULAR_RATE_HORIZON_S * observation.base_angular_velocity_z_rad_s))
            .clamp(-ANKLE_CORRECTION_LIMIT_RAD, ANKLE_CORRECTION_LIMIT_RAD);
        let step_s = sim.fixed_delta().as_seconds().value();
        let arm_quiet = capture_error_m.z.abs() < 0.05
            && com_velocity_m_s.length() < 0.10
            && roll_rad.abs() < 0.05
            && observation.base_angular_velocity_x_rad_s.abs() < 0.20;
        self.arm_quiet_duration_s = if arm_quiet {
            self.arm_quiet_duration_s + step_s
        } else {
            0.0
        };
        update_arm_targets_rad(
            &mut self.arm_roll_targets_rad,
            roll_rad,
            observation.base_angular_velocity_x_rad_s,
            self.arm_quiet_duration_s >= ARM_QUIET_DURATION_S,
            step_s,
            self.balance_feedback_enabled,
        );
        let arm_targets_rad = self.arm_roll_targets_rad;
        let mut joint_feedback = Vec::with_capacity(self.servos.len());
        let targets: Vec<_> = self
            .servos
            .iter_mut()
            .map(|servo| {
                let requested_rad = corrected_position_rad(
                    &servo.name,
                    hip_lateral_rad,
                    hip_forward_rad,
                    ankle_lateral_rad,
                    ankle_forward_rad,
                ) + arm_joint_target_rad(&servo.name, arm_targets_rad);
                let position_limited_rad =
                    requested_rad.clamp(servo.limits.lower, servo.limits.upper);
                let rate_rad_s = TARGET_RATE_LIMIT_RAD_S.min(servo.limits.max_velocity);
                let position_rad = position_limited_rad.clamp(
                    servo.previous_target_rad - rate_rad_s * step_s,
                    servo.previous_target_rad + rate_rad_s * step_s,
                );
                joint_feedback.push(json!({
                    "link":servo.name,
                    "position_rad":sim.named_joint_position(&servo.name),
                    "velocity_rad_s":sim.named_joint_velocity(&servo.name),
                    "requested_position_rad":requested_rad,
                    "position_limited_target_rad":position_limited_rad,
                    "target_position_rad":position_rad,
                    "position_limit_active":requested_rad != position_limited_rad,
                    "target_rate_limit_active":position_limited_rad != position_rad,
                }));
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
            "pelvis_up_world": up_world.to_array(),
            "pelvis_roll_world_x_rad": roll_rad,
            "pelvis_angular_velocity_world_x_rad_s": observation.base_angular_velocity_x_rad_s,
            "hip_capture_term_rad": enabled * LATERAL_CAPTURE_GAIN_RAD_PER_M * capture_error_m.z,
            "hip_attitude_term_rad": -enabled * HIP_ATTITUDE_GAIN * roll_rad,
            "hip_angular_rate_term_rad": -enabled * HIP_ANGULAR_RATE_HORIZON_S
                * observation.base_angular_velocity_x_rad_s,
            "arm_roll_targets_rad": arm_targets_rad,
            "arm_quiet_duration_s": self.arm_quiet_duration_s,
            "hip_lateral_correction_rad": hip_lateral_rad,
            "hip_forward_correction_rad": hip_forward_rad,
            "ankle_lateral_correction_rad": ankle_lateral_rad,
            "ankle_forward_correction_rad": ankle_forward_rad,
            "target_rate_limit_rad_s": TARGET_RATE_LIMIT_RAD_S,
            "joint_feedback": joint_feedback,
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
            "controller": "bounded DCM and IMU ankle/hip feedback with outward arm reactions",
            "motor_gain_model": "ForceBased implicit PD; gains have N m units",
            "balance_feedback_enabled": self.balance_feedback_enabled,
            "stance_hip_roll_rad": STANCE_ROLL_RAD,
            "lateral_capture_gain_rad_per_m": LATERAL_CAPTURE_GAIN_RAD_PER_M,
            "forward_capture_gain_rad_per_m": FORWARD_CAPTURE_GAIN_RAD_PER_M,
            "hip_attitude_gain_rad_per_rad": HIP_ATTITUDE_GAIN,
            "hip_angular_rate_horizon_s": HIP_ANGULAR_RATE_HORIZON_S,
            "attitude_gain_rad_per_rad": ATTITUDE_GAIN,
            "angular_rate_horizon_s": ANGULAR_RATE_HORIZON_S,
            "lateral_hip_correction_limit_rad": HIP_CORRECTION_LIMIT_RAD,
            "forward_hip_correction_limit_rad": ANKLE_CORRECTION_LIMIT_RAD,
            "ankle_correction_limit_rad": ANKLE_CORRECTION_LIMIT_RAD,
            "arm_attitude_gain_rad_per_rad": ARM_ATTITUDE_GAIN,
            "arm_prediction_horizon_s": ANGULAR_RATE_HORIZON_S,
            "arm_prediction_deadband_rad": ARM_PREDICTION_DEADBAND_RAD,
            "arm_correction_limit_rad": ARM_CORRECTION_LIMIT_RAD,
            "arm_return_rate_rad_s": ARM_RETURN_RATE_RAD_S,
            "arm_quiet_duration_s": ARM_QUIET_DURATION_S,
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

fn update_arm_targets_rad(
    targets_rad: &mut [f64; 2],
    roll_rad: f64,
    angular_rate_rad_s: f64,
    quiet: bool,
    step_s: f64,
    enabled: bool,
) {
    if !enabled {
        *targets_rad = [0.0; 2];
        return;
    }
    if quiet {
        let return_step_rad = ARM_RETURN_RATE_RAD_S * step_s;
        targets_rad[0] = (targets_rad[0] - return_step_rad).max(0.0);
        targets_rad[1] = (targets_rad[1] + return_step_rad).min(0.0);
        return;
    }
    let prediction_rad = roll_rad + ANGULAR_RATE_HORIZON_S * angular_rate_rad_s;
    let excursion_rad = (ARM_ATTITUDE_GAIN
        * (prediction_rad.abs() - ARM_PREDICTION_DEADBAND_RAD).max(0.0))
    .min(ARM_CORRECTION_LIMIT_RAD);
    // Move only the contralateral arm outward. Holding the excursion avoids
    // reversing its angular reaction while the capture point still diverges.
    if prediction_rad > 0.0 {
        targets_rad[0] = targets_rad[0].max(excursion_rad);
    } else if prediction_rad < 0.0 {
        targets_rad[1] = targets_rad[1].min(-excursion_rad);
    }
}

fn arm_joint_target_rad(name: &str, arm_targets_rad: [f64; 2]) -> f64 {
    match name {
        "left_shoulder_roll_link" => arm_targets_rad[0],
        "right_shoulder_roll_link" => arm_targets_rad[1],
        _ => 0.0,
    }
}

fn lateral_hip_correction_rad(
    capture_error_m: f64,
    roll_rad: f64,
    angular_rate_rad_s: f64,
    enabled: bool,
) -> f64 {
    if enabled {
        // The target mapping subtracts this correction from both hip rolls.
        // Positive body roll/rate therefore requests positive hip movement,
        // whose motor reaction opposes pelvis roll. This is not foot-pose IK.
        (LATERAL_CAPTURE_GAIN_RAD_PER_M * capture_error_m
            - HIP_ATTITUDE_GAIN * roll_rad
            - HIP_ANGULAR_RATE_HORIZON_S * angular_rate_rad_s)
            .clamp(-HIP_CORRECTION_LIMIT_RAD, HIP_CORRECTION_LIMIT_RAD)
    } else {
        0.0
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

#[cfg(test)]
mod tests {
    use super::*;
    use rne_physics::{RigidBody, RigidBodyInertia};
    use rne_robot::KinematicModel;
    use std::path::Path;

    fn mass_and_foot_geometry(
        sim: &UrdfSceneSim,
        model: &KinematicModel,
        hip_correction_rad: f64,
        ankle_correction_rad: f64,
        arm_targets_rad: [f64; 2],
    ) -> (Vec3, [Vec3; 2]) {
        let mut positions_rad = vec![0.0; model.dof()];
        for (index, entity) in model.movable_joint_entities().iter().enumerate() {
            let joint = sim.world().get::<Joint>(*entity).expect("joint");
            let link = sim
                .world()
                .get::<Link>(joint.child_link)
                .expect("joint child");
            let target_rad = corrected_position_rad(
                &link.name,
                hip_correction_rad,
                0.0,
                ankle_correction_rad,
                0.0,
            ) + arm_joint_target_rad(&link.name, arm_targets_rad);
            let bounded_rad = target_rad.clamp(joint.limits.lower, joint.limits.upper);
            assert!((joint.limits.lower..=joint.limits.upper).contains(&bounded_rad));
            if link.name.contains("hip_roll") || link.name.contains("shoulder_roll") {
                assert_eq!(
                    target_rad, bounded_rad,
                    "target must fit the authored range"
                );
            }
            positions_rad[model.base_dof() + index] = bounded_rad;
        }
        let fk = model
            .forward_kinematics(&positions_rad)
            .expect("source-model forward kinematics");
        let mut weighted_com_m_kg = Vec3::ZERO;
        let mut mass_kg = 0.0;
        for entity in fk.links() {
            let body = sim.world().get::<RigidBody>(*entity).expect("body");
            let inertia = sim
                .world()
                .get::<RigidBodyInertia>(*entity)
                .expect("declared inertia");
            let pose = fk.link_transform(*entity).expect("link pose");
            weighted_com_m_kg +=
                (pose.translation + pose.rotation * inertia.center_of_mass_local_m) * body.mass_kg;
            mass_kg += body.mass_kg;
        }
        let feet_m = FOOT_NAMES.map(|name| {
            fk.link_transform(model.link_entity_by_name(name).expect("foot link"))
                .expect("foot pose")
                .translation
        });
        (weighted_com_m_kg / mass_kg, feet_m)
    }

    #[test]
    fn bounded_hip_excursion_moves_mass_inward_without_crossing_source_model_feet() {
        let sim = UrdfSceneSim::from_scene_path(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("g1.rne.scene.toml"),
        )
        .expect("dedicated physical G1 scene");
        let robot = sim
            .world()
            .iter_entities()
            .find_map(|entity| {
                let link = entity.get::<Link>()?;
                (link.name == "pelvis").then_some(link.robot)
            })
            .expect("G1 robot");
        let model = KinematicModel::from_robot(sim.world(), robot).expect("G1 kinematic model");
        let (nominal_com_m, nominal_feet_m) =
            mass_and_foot_geometry(&sim, &model, 0.0, 0.0, [0.0; 2]);
        assert!(nominal_feet_m[1].z - nominal_feet_m[0].z > 0.49);
        for lateral_direction in [-1.0, 1.0] {
            let hip_rad = HIP_CORRECTION_LIMIT_RAD * lateral_direction;
            let (com_m, feet_m) =
                mass_and_foot_geometry(&sim, &model, hip_rad, 0.10 * lateral_direction, [0.0; 2]);
            let support_index = usize::from(lateral_direction > 0.0);
            let nominal_offset_m = nominal_com_m.z - nominal_feet_m[support_index].z;
            let corrected_offset_m = com_m.z - feet_m[support_index].z;
            // Use the imported origins, axes, masses and COMs: a support foot
            // held by contact must see the robot mass move against its drift.
            assert!((corrected_offset_m - nominal_offset_m) * lateral_direction < -0.10);
            assert!(feet_m[1].z - feet_m[0].z > 0.30);
        }
    }

    #[test]
    fn outward_arm_reaction_moves_source_model_mass_against_either_lateral_drift() {
        let sim = UrdfSceneSim::from_scene_path(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("g1.rne.scene.toml"),
        )
        .expect("dedicated physical G1 scene");
        let robot = sim
            .world()
            .iter_entities()
            .find_map(|entity| {
                let link = entity.get::<Link>()?;
                (link.name == "pelvis").then_some(link.robot)
            })
            .expect("G1 robot");
        let model = KinematicModel::from_robot(sim.world(), robot).expect("G1 kinematic model");
        let (nominal_com_m, nominal_feet_m) =
            mass_and_foot_geometry(&sim, &model, 0.0, 0.0, [0.0; 2]);
        for direction in [-1.0, 1.0] {
            let mut arm_targets_rad = [0.0; 2];
            update_arm_targets_rad(
                &mut arm_targets_rad,
                0.20 * direction,
                2.0 * direction,
                false,
                0.001,
                true,
            );
            assert!(arm_targets_rad[0] >= 0.0 && arm_targets_rad[1] <= 0.0);
            assert_eq!(arm_targets_rad[usize::from(direction > 0.0)], 0.0);
            let (com_m, feet_m) = mass_and_foot_geometry(&sim, &model, 0.0, 0.0, arm_targets_rad);
            assert!((com_m.z - nominal_com_m.z) * direction < -0.003);
            assert_eq!(feet_m, nominal_feet_m);
            let arm_name = if direction > 0.0 {
                "left_shoulder_roll_link"
            } else {
                "right_shoulder_roll_link"
            };
            let joint_name = arm_name.replace("_link", "_joint");
            let joint_entity = model
                .joint_entity_by_name(&joint_name)
                .expect("source arm joint");
            let jacobian = model
                .jacobian(
                    &vec![0.0; model.dof()],
                    model.link_entity_by_name(arm_name).expect("arm link"),
                    Vec3::ZERO,
                )
                .expect("source joint axis");
            let joint_column =
                model.base_dof() + model.dof_index_of_joint(joint_entity).expect("arm column");
            // The source axis has a positive world-X component. At the
            // nominal arm state, the target's signed motor reaction opposes
            // the observed pelvis roll; no applied torque is read back here.
            let axis_world_x = jacobian.get(3, joint_column);
            assert!(axis_world_x > 0.99);
            let arm_index = usize::from(direction < 0.0);
            assert!(-axis_world_x * arm_targets_rad[arm_index] * direction < 0.0);
        }
    }

    #[test]
    fn arm_excursion_holds_until_quiet_and_returns_slowly_with_ablation() {
        let mut targets_rad = [0.0; 2];
        update_arm_targets_rad(&mut targets_rad, 0.20, 2.0, false, 0.001, true);
        let held_rad = targets_rad;
        update_arm_targets_rad(&mut targets_rad, 0.0, 0.0, false, 0.001, true);
        assert_eq!(targets_rad, held_rad);
        update_arm_targets_rad(&mut targets_rad, 0.0, 0.0, true, 0.001, true);
        assert!((held_rad[0] - targets_rad[0] - ARM_RETURN_RATE_RAD_S * 0.001).abs() < 1.0e-12);
        update_arm_targets_rad(&mut targets_rad, -0.20, -2.0, false, 0.001, false);
        assert_eq!(targets_rad, [0.0; 2]);
        update_arm_targets_rad(&mut targets_rad, 0.001, 0.01, false, 0.001, true);
        assert_eq!(targets_rad, [0.0; 2]);
    }

    #[test]
    fn hip_imu_command_has_the_motor_reaction_sign_against_pelvis_roll() {
        for direction in [-1.0, 1.0] {
            for (roll_rad, rate_rad_s) in [(0.10 * direction, 0.0), (0.0, 2.0 * direction)] {
                let correction_rad = lateral_hip_correction_rad(0.0, roll_rad, rate_rad_s, true);
                for name in ["left_hip_roll_link", "right_hip_roll_link"] {
                    let nominal_rad = corrected_position_rad(name, 0.0, 0.0, 0.0, 0.0);
                    let target_rad = corrected_position_rad(name, correction_rad, 0.0, 0.0, 0.0);
                    // Both URDF hip-roll axes are +X. At the nominal joint
                    // state, this position error requests leg torque in the
                    // drift direction and the opposite reaction on the pelvis.
                    let requested_leg_torque_nm =
                        POSTURE_STIFFNESS_NM_PER_RAD * (target_rad - nominal_rad);
                    let pelvis_reaction_nm = -requested_leg_torque_nm;
                    assert!(pelvis_reaction_nm * direction < 0.0);
                }
            }
        }
    }

    #[test]
    fn hip_feedback_is_symmetric_bounded_and_disabled_by_ablation() {
        for (error_m, roll_rad, rate_rad_s) in [(0.10, 0.20, 4.0), (10.0, -1.0, -10.0)] {
            let positive = lateral_hip_correction_rad(error_m, roll_rad, rate_rad_s, true);
            let negative = lateral_hip_correction_rad(-error_m, -roll_rad, -rate_rad_s, true);
            assert_eq!(positive, -negative);
            assert!(positive.abs() <= HIP_CORRECTION_LIMIT_RAD);
            for sign in [-1.0, 1.0] {
                assert_eq!(
                    lateral_hip_correction_rad(
                        error_m * sign,
                        roll_rad * sign,
                        rate_rad_s * sign,
                        false
                    ),
                    0.0
                );
            }
        }
    }
}
