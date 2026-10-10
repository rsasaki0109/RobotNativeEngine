//! Bounded joint and inertial feedback for the dedicated physical Go2 plant.
//!
//! Four-leg stance servos use unit-bearing, force-based implicit PD. Measured
//! roll and filtered angular rate change opposing leg lengths. Filtered foot
//! loads and the measured center of mass adjust bounded per-leg calf/thigh biases.
//! Ground reaction comes from the colliders and contact solver. This controller
//! cannot read a kick schedule and never applies a base wrench or changes a pose.

use super::model::center_of_mass;
use rne_ai::{UrdfJointPositionTarget, UrdfSceneSim};
use rne_math::Vec3;
use rne_robot::{Joint, JointKind, JointLimits, Link};
use serde_json::{json, Value};
use std::{error::Error, io};

type ControllerResult<T> = Result<T, Box<dyn Error>>;

// A wider authored-range stance increases lateral support and lowers the
// naturally settled body. Both the feedback controller and its ablation use
// this same joint reference; masses, friction and effort limits are unchanged.
const STANCE_ABDUCTION_RAD: f64 = 0.45;
const STIFFNESS_NM_PER_RAD: f64 = 100.0;
const DAMPING_NM_S_PER_RAD: f64 = 4.0;
const ROLL_LENGTH_GAIN: f64 = 0.8;
const ROLL_RATE_HORIZON_S: f64 = 0.05;
const CALF_CORRECTION_LIMIT_RAD: f64 = 0.32;
const TARGET_RATE_LIMIT_RAD_S: f64 = 6.0;
const GYRO_FILTER_TIME_CONSTANT_S: f64 = 0.005;
const LOAD_FILTER_TIME_CONSTANT_S: f64 = 0.05;
const GRAVITY_M_S2: f64 = 9.81;
const LOAD_REGULATION_MIN_SUPPORT_FRACTION: f64 = 0.5;
const LOAD_REGULATION_MAX_ROLL_RAD: f64 = 0.04;
const LOAD_REGULATION_MAX_HORIZONTAL_SPEED_M_S: f64 = 0.15;
const MIN_LOAD_SHARE_FRACTION: f64 = 0.15;
const MAX_LOAD_SHARE_FRACTION: f64 = 0.85;
const LOAD_TRIM_GAIN_RAD_S: f64 = 0.10;
const LOAD_TRIM_LIMIT_RAD: f64 = 0.03;
const ZERO_SUM_PROJECTION_STEPS: usize = 48;
const MIN_SUPPORT_SPAN_M: f64 = 1e-6;
const FOOT_NAMES: [&str; 4] = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"];

#[derive(Debug, Clone, Copy)]
struct LoadRegulationObservation {
    com_world_m: Vec3,
    com_velocity_world_m_s: Vec3,
    mass_kg: f64,
    roll_rad: f64,
    forward_unit: Vec3,
    lateral_unit: Vec3,
    dt_s: f64,
}

#[derive(Debug)]
struct Servo {
    name: String,
    limits: JointLimits,
    previous_target_rad: f64,
}

/// Encoder and contact-load controlled stance, with optional IMU correction.
#[derive(Debug)]
pub(super) struct Go2Controller {
    servos: Vec<Servo>,
    balance_feedback_enabled: bool,
    last_feedback: Value,
    filtered_roll_rate_rad_s: f64,
    filtered_foot_loads_n: [f64; 4],
    calf_trim_rad: [f64; 4],
}

impl Go2Controller {
    /// Configures physical PD gains and the individual authored torque caps.
    ///
    /// The feedback ablation retains the same wide stance, gains, caps and
    /// target-rate limits. Encoder PD and slow contact-load regulation remain
    /// active; the ablation disables only the IMU opposing leg-length correction.
    pub(super) fn new(
        sim: &mut UrdfSceneSim,
        balance_feedback_enabled: bool,
    ) -> ControllerResult<Self> {
        let mut servos: Vec<_> = sim
            .world()
            .iter_entities()
            .filter_map(|entity| {
                let joint = entity.get::<Joint>()?;
                (joint.kind != JointKind::Fixed).then(|| Servo {
                    name: sim
                        .world()
                        .get::<Link>(joint.child_link)
                        .expect("joint child link")
                        .name
                        .clone(),
                    limits: joint.limits,
                    previous_target_rad: joint.position,
                })
            })
            .collect();
        servos.sort_unstable_by(|a, b| a.name.cmp(&b.name));
        if servos.len() != 12
            || servos.iter().any(|servo| {
                !(servo.name.ends_with("_hip")
                    || servo.name.ends_with("_thigh")
                    || servo.name.ends_with("_calf"))
                    || !servo.limits.max_effort.is_finite()
                    || servo.limits.max_effort <= 0.0
                    || !servo.limits.max_velocity.is_finite()
                    || servo.limits.max_velocity <= 0.0
            })
        {
            return Err(
                io::Error::other("the Go2 controller requires its 12 authored leg joints").into(),
            );
        }
        for servo in &servos {
            if !sim.configure_named_revolute_position_actuation(
                &servo.name,
                STIFFNESS_NM_PER_RAD,
                DAMPING_NM_S_PER_RAD,
                servo.limits.max_effort,
            ) {
                return Err(
                    io::Error::other("a Go2 unit-bearing position servo is invalid").into(),
                );
            }
        }
        Ok(Self {
            servos,
            balance_feedback_enabled,
            last_feedback: Value::Null,
            filtered_roll_rate_rad_s: 0.0,
            filtered_foot_loads_n: [0.0; 4],
            calf_trim_rad: [0.0; 4],
        })
    }

    /// Measures the plant, commands bounded joints, and advances exactly one tick.
    pub(super) fn step(&mut self, sim: &mut UrdfSceneSim) -> ControllerResult<()> {
        let (com_m, com_velocity_m_s, mass_kg) = center_of_mass(sim)?;
        let base = sim
            .named_transform("base")
            .ok_or_else(|| io::Error::other("Go2 base pose is missing"))?;
        let observation = sim.observe();
        let up = base.rotation * Vec3::Z;
        let forward = base.rotation * Vec3::X;
        let lateral = Vec3::new(forward.x, 0.0, forward.z)
            .normalize_or_zero()
            .cross(Vec3::Y);
        let roll_rad = up.dot(lateral).atan2(up.y);
        let angular_velocity_rad_s = Vec3::new(
            observation.base_angular_velocity_x_rad_s,
            observation.base_angular_velocity_y_rad_s,
            observation.base_angular_velocity_z_rad_s,
        );
        let roll_rate_rad_s = angular_velocity_rad_s.dot(forward);
        let dt_s = sim.fixed_delta().as_seconds().value();
        if ![roll_rad, roll_rate_rad_s, dt_s, mass_kg]
            .iter()
            .all(|value| value.is_finite())
            || dt_s <= 0.0
            || mass_kg <= 0.0
            || !com_m.is_finite()
            || !com_velocity_m_s.is_finite()
        {
            return Err(io::Error::other("Go2 feedback or fixed step is invalid").into());
        }
        let gyro_alpha = dt_s / (GYRO_FILTER_TIME_CONSTANT_S + dt_s);
        self.filtered_roll_rate_rad_s +=
            gyro_alpha * (roll_rate_rad_s - self.filtered_roll_rate_rad_s);
        let (load_regulation_active, desired_loads_n) = self.update_contact_load_regulation(
            sim,
            LoadRegulationObservation {
                com_world_m: com_m,
                com_velocity_world_m_s: com_velocity_m_s,
                mass_kg,
                roll_rad,
                forward_unit: forward,
                lateral_unit: lateral,
                dt_s,
            },
        )?;
        let correction_rad = if self.balance_feedback_enabled {
            (ROLL_LENGTH_GAIN * roll_rad + ROLL_RATE_HORIZON_S * self.filtered_roll_rate_rad_s)
                .clamp(-CALF_CORRECTION_LIMIT_RAD, CALF_CORRECTION_LIMIT_RAD)
        } else {
            0.0
        };
        if ![roll_rad, roll_rate_rad_s, correction_rad, dt_s]
            .iter()
            .all(|value| value.is_finite())
            || dt_s <= 0.0
        {
            return Err(io::Error::other("Go2 feedback or fixed step is invalid").into());
        }
        let measured_joints = measured_joint_states(sim, &self.servos);
        let targets = position_targets(&mut self.servos, self.calf_trim_rad, correction_rad, dt_s);
        let loads_n: [f64; 4] = FOOT_NAMES.map(|name| {
            sim.named_body_contact_loads(name)
                .iter()
                .map(|(_, normal_force_n)| *normal_force_n)
                .sum()
        });
        self.last_feedback = json!({
            "balance_feedback_enabled": self.balance_feedback_enabled,
            "imu_feedback_enabled": self.balance_feedback_enabled,
            "contact_load_regulation_enabled": true,
            "contact_load_regulation_active": load_regulation_active,
            "desired_foot_normal_loads_n": desired_loads_n,
            "foot_order": FOOT_NAMES,
            "com_world_m": com_m.to_array(),
            "com_velocity_world_m_s": com_velocity_m_s.to_array(),
            "mass_kg": mass_kg,
            "imu_roll_rad": roll_rad,
            "imu_roll_rate_rad_s": roll_rate_rad_s,
            "filtered_roll_rate_rad_s": self.filtered_roll_rate_rad_s,
            "filtered_foot_loads_n": self.filtered_foot_loads_n,
            "calf_load_trim_rad": self.calf_trim_rad,
            "opposing_calf_correction_rad": correction_rad,
            "foot_normal_loads_n": loads_n,
            "measured_joint_states": measured_joints,
            "target_rate_limit_rad_s": TARGET_RATE_LIMIT_RAD_S,
            "joint_position_targets_rad": targets.iter().map(|target| json!({
                "link": target.link_name,
                "target_rad": target.position,
            })).collect::<Vec<_>>(),
        });
        sim.step_joint_position_actuation_targets(&targets);
        Ok(())
    }

    fn update_contact_load_regulation(
        &mut self,
        sim: &UrdfSceneSim,
        observation: LoadRegulationObservation,
    ) -> ControllerResult<(bool, Option<[f64; 4]>)> {
        let LoadRegulationObservation {
            com_world_m: com_m,
            com_velocity_world_m_s: com_velocity_m_s,
            mass_kg,
            roll_rad,
            forward_unit: forward,
            lateral_unit: lateral,
            dt_s,
        } = observation;
        let observed_loads_n: [f64; 4] = FOOT_NAMES.map(|name| {
            sim.named_body_contact_loads(name)
                .iter()
                .map(|(_, force_n)| force_n.max(0.0))
                .sum()
        });
        let load_alpha = dt_s / (LOAD_FILTER_TIME_CONSTANT_S + dt_s);
        for (filtered, observed) in self.filtered_foot_loads_n.iter_mut().zip(observed_loads_n) {
            *filtered += load_alpha * (observed - *filtered);
        }
        let total_load_n = self.filtered_foot_loads_n.iter().sum::<f64>();
        // The slow load regulator is frozen during a fast recovery. It changes
        // joint references only after measured support and motion settle.
        let load_regulation_active = total_load_n
            > mass_kg * GRAVITY_M_S2 * LOAD_REGULATION_MIN_SUPPORT_FRACTION
            && roll_rad.abs() < LOAD_REGULATION_MAX_ROLL_RAD
            && Vec3::new(com_velocity_m_s.x, 0.0, com_velocity_m_s.z).length()
                < LOAD_REGULATION_MAX_HORIZONTAL_SPEED_M_S;
        let mut desired_loads_n = None;
        if load_regulation_active {
            let share_n = total_load_n / 4.0;
            let feet = FOOT_NAMES.map(|name| {
                sim.named_transform(name)
                    .expect("validated Go2 foot pose")
                    .translation
            });
            // Bilinear shares place the desired normal-force resultant at the
            // measured CoM projection within the four-foot support rectangle.
            let forward_front_m = ((feet[0] + feet[1]) * 0.5).dot(forward);
            let forward_rear_m = ((feet[2] + feet[3]) * 0.5).dot(forward);
            let lateral_left_m = ((feet[0] + feet[2]) * 0.5).dot(lateral);
            let lateral_right_m = ((feet[1] + feet[3]) * 0.5).dot(lateral);
            if (forward_front_m - forward_rear_m).abs() < MIN_SUPPORT_SPAN_M
                || (lateral_right_m - lateral_left_m).abs() < MIN_SUPPORT_SPAN_M
            {
                return Err(io::Error::other("Go2 support span is degenerate").into());
            }
            let front_fraction = ((com_m.dot(forward) - forward_rear_m)
                / (forward_front_m - forward_rear_m))
                .clamp(MIN_LOAD_SHARE_FRACTION, MAX_LOAD_SHARE_FRACTION);
            let right_fraction = ((com_m.dot(lateral) - lateral_left_m)
                / (lateral_right_m - lateral_left_m))
                .clamp(MIN_LOAD_SHARE_FRACTION, MAX_LOAD_SHARE_FRACTION);
            let fractions = [
                front_fraction * (1.0 - right_fraction),
                front_fraction * right_fraction,
                (1.0 - front_fraction) * (1.0 - right_fraction),
                (1.0 - front_fraction) * right_fraction,
            ];
            desired_loads_n = Some(fractions.map(|fraction| total_load_n * fraction));
            let rates_rad_s: [f64; 4] = std::array::from_fn(|i| {
                LOAD_TRIM_GAIN_RAD_S
                    * ((total_load_n * fractions[i] - self.filtered_foot_loads_n[i]) / share_n)
                        .clamp(-1.0, 1.0)
            });
            let mean_rate_rad_s = rates_rad_s.iter().sum::<f64>() / 4.0;
            for (trim, rate) in self.calf_trim_rad.iter_mut().zip(rates_rad_s) {
                *trim = (*trim + (rate - mean_rate_rad_s) * dt_s)
                    .clamp(-LOAD_TRIM_LIMIT_RAD, LOAD_TRIM_LIMIT_RAD);
            }
            self.project_calf_load_trim();
        }
        Ok((load_regulation_active, desired_loads_n))
    }

    fn project_calf_load_trim(&mut self) {
        // Project onto the bounded, zero-sum bias set in a fixed iteration
        // order. This redistributes leg length without a common extension.
        let mut low = -2.0 * LOAD_TRIM_LIMIT_RAD;
        let mut high = 2.0 * LOAD_TRIM_LIMIT_RAD;
        for _ in 0..ZERO_SUM_PROJECTION_STEPS {
            let shift = (low + high) * 0.5;
            let sum = self
                .calf_trim_rad
                .iter()
                .map(|trim| (trim - shift).clamp(-LOAD_TRIM_LIMIT_RAD, LOAD_TRIM_LIMIT_RAD))
                .sum::<f64>();
            if sum > 0.0 {
                low = shift;
            } else {
                high = shift;
            }
        }
        let shift = (low + high) * 0.5;
        for trim in &mut self.calf_trim_rad {
            *trim = (*trim - shift).clamp(-LOAD_TRIM_LIMIT_RAD, LOAD_TRIM_LIMIT_RAD);
        }
    }

    /// Returns observations from immediately before the latest completed tick.
    pub(super) fn telemetry(&self) -> Value {
        self.last_feedback.clone()
    }

    /// Describes solver-enforced torque ceilings without claiming measured effort.
    pub(super) fn configuration(&self) -> Value {
        json!({
            "controller": "joint-encoder stance PD, CoM contact-load regulation, and optional IMU opposing leg-length feedback",
            "imu_feedback_enabled": self.balance_feedback_enabled,
            "contact_load_regulation_enabled": true,
            "ablation_scope": "disables only IMU correction; joint PD and contact-load regulation remain active",
            "foot_order": FOOT_NAMES,
            "gyro_filter_time_constant_s": GYRO_FILTER_TIME_CONSTANT_S,
            "load_filter_time_constant_s": LOAD_FILTER_TIME_CONSTANT_S,
            "load_regulation_min_support_fraction": LOAD_REGULATION_MIN_SUPPORT_FRACTION,
            "load_regulation_max_roll_rad": LOAD_REGULATION_MAX_ROLL_RAD,
            "load_regulation_max_horizontal_speed_m_s": LOAD_REGULATION_MAX_HORIZONTAL_SPEED_M_S,
            "load_share_fraction_bounds": [MIN_LOAD_SHARE_FRACTION,MAX_LOAD_SHARE_FRACTION],
            "load_trim_gain_rad_s": LOAD_TRIM_GAIN_RAD_S,
            "load_trim_limit_rad": LOAD_TRIM_LIMIT_RAD,
            "load_trim_constraint": "zero sum across FL/FR/RL/RR, bounded per leg",
            "desired_load_source": "bilinear measured-CoM projection over the measured foot rectangle",
            "motor_gain_model": "ForceBased implicit PD; gains have N m units",
            "balance_feedback_enabled": self.balance_feedback_enabled,
            "stance_hip_abduction_rad": STANCE_ABDUCTION_RAD,
            "stance_strategy": "same wider joint stance for feedback and ablation; gravity and source geometry determine settled height",
            "stiffness_nm_per_rad": STIFFNESS_NM_PER_RAD,
            "damping_nm_s_per_rad": DAMPING_NM_S_PER_RAD,
            "roll_length_gain": ROLL_LENGTH_GAIN,
            "roll_rate_horizon_s": ROLL_RATE_HORIZON_S,
            "calf_correction_limit_rad": CALF_CORRECTION_LIMIT_RAD,
            "limits": self.servos.iter().map(|servo| json!({
                "link": servo.name,
                "solver_enforced_effort_ceiling_nm": servo.limits.max_effort,
                "authored_velocity_limit_rad_s": servo.limits.max_velocity,
                "target_rate_limit_rad_s": TARGET_RATE_LIMIT_RAD_S.min(servo.limits.max_velocity),
            })).collect::<Vec<_>>(),
        })
    }
}

fn measured_joint_states(sim: &UrdfSceneSim, servos: &[Servo]) -> Vec<Value> {
    servos
        .iter()
        .map(|servo| {
            json!({
                "link": servo.name,
                "position_rad": sim.named_joint_position(&servo.name),
                "velocity_rad_s": sim.named_joint_velocity(&servo.name),
            })
        })
        .collect()
}

fn position_targets(
    servos: &mut [Servo],
    calf_trim_rad: [f64; 4],
    correction_rad: f64,
    dt_s: f64,
) -> Vec<UrdfJointPositionTarget<'_>> {
    let mut targets = Vec::with_capacity(servos.len());
    for servo in servos {
        let left = servo.name.starts_with("FL_") || servo.name.starts_with("RL_");
        let foot_index = if servo.name.starts_with("FL_") {
            0
        } else if servo.name.starts_with("FR_") {
            1
        } else if servo.name.starts_with("RL_") {
            2
        } else {
            3
        };
        let calf_delta_rad = calf_trim_rad[foot_index]
            + if left {
                -correction_rad
            } else {
                correction_rad
            };
        // The dedicated model's q=0 is the physical thigh/calf standing
        // pose. Positive calf bias extends the leg near this reference;
        // the opposing half-sized thigh bias limits fore/aft foot motion.
        let desired_rad = if servo.name.ends_with("_hip") {
            if left {
                STANCE_ABDUCTION_RAD
            } else {
                -STANCE_ABDUCTION_RAD
            }
        } else if servo.name.ends_with("_thigh") {
            -0.5 * calf_delta_rad
        } else {
            calf_delta_rad
        };
        let rate_rad_s = TARGET_RATE_LIMIT_RAD_S.min(servo.limits.max_velocity);
        let target_rad = desired_rad
            .clamp(
                servo.previous_target_rad - rate_rad_s * dt_s,
                servo.previous_target_rad + rate_rad_s * dt_s,
            )
            .clamp(servo.limits.lower, servo.limits.upper);
        servo.previous_target_rad = target_rad;
        targets.push(UrdfJointPositionTarget {
            link_name: &servo.name,
            position: target_rad,
        });
    }
    targets
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_robot::KinematicModel;
    use std::path::Path;

    #[test]
    fn wider_stance_fits_imported_joint_limits_and_expands_lateral_support() {
        let sim = UrdfSceneSim::from_scene_path(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("go2.rne.scene.toml"),
        )
        .expect("dedicated physical Go2 scene");
        let robot = sim
            .world()
            .iter_entities()
            .find_map(|entity| {
                let link = entity.get::<Link>()?;
                (link.name == "base").then_some(link.robot)
            })
            .expect("Go2 robot");
        let model = KinematicModel::from_robot(sim.world(), robot).expect("imported Go2 model");
        let foot_geometry = |hip_abduction_rad: f64| {
            let mut positions = vec![0.0; model.dof()];
            for (index, entity) in model.movable_joint_entities().iter().enumerate() {
                let joint = sim.world().get::<Joint>(*entity).expect("source joint");
                let name = &sim
                    .world()
                    .get::<Link>(joint.child_link)
                    .expect("child link")
                    .name;
                let angle_rad = if name.ends_with("_hip") {
                    if name.starts_with("FL_") || name.starts_with("RL_") {
                        hip_abduction_rad
                    } else {
                        -hip_abduction_rad
                    }
                } else {
                    0.0
                };
                assert!((joint.limits.lower..=joint.limits.upper).contains(&angle_rad));
                positions[model.base_dof() + index] = angle_rad;
            }
            let fk = model
                .forward_kinematics(&positions)
                .expect("actual imported FK");
            let root_y_m = fk
                .link_transform(model.base_link())
                .expect("base pose")
                .translation
                .y;
            let feet = FOOT_NAMES.map(|name| {
                fk.link_transform(model.link_entity_by_name(name).expect("foot"))
                    .expect("foot pose")
                    .translation
            });
            (root_y_m, feet)
        };
        let (root_y_m, wide_feet) = foot_geometry(STANCE_ABDUCTION_RAD);
        let (_, original_feet) = foot_geometry(0.25);
        for front_index in [0, 2] {
            let old_span_m = original_feet[front_index + 1].z - original_feet[front_index].z;
            let wide_span_m = wide_feet[front_index + 1].z - wide_feet[front_index].z;
            assert!(wide_span_m > old_span_m + 0.10);
            assert!(wide_feet[front_index].z < 0.0 && wide_feet[front_index + 1].z > 0.0);
        }
        for foot in wide_feet {
            // The imported foot sphere has radius 0.022 m. Ground contact
            // requires a base height above the existing recovery threshold;
            // the test uses geometry, not an assigned simulated base pose.
            let geometric_contact_base_height_m = 0.022 - (foot.y - root_y_m);
            assert!(geometric_contact_base_height_m > 0.22);
            assert!(geometric_contact_base_height_m < 0.30);
        }
    }
}
