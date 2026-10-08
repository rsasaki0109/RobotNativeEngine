//! Feedback trot controller.

use crate::quadruped::{BaseState, Quadruped};
use crate::sim::JointCommand;
use rne_math::{Quat, Vec3};
use std::f64::consts::PI;

/// Gravitational acceleration carried by the stance legs, in m/s².
const GRAVITY_M_S2: f64 = 9.81;

/// Phase offset of each leg (front-left, front-right, rear-left, rear-right)
/// as a fraction of the gait period: the diagonal pairs alternate.
const LEG_PHASE: [f64; 4] = [0.0, 0.5, 0.5, 0.0];

/// Tuning of [`TrotController`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrotParams {
    /// Full stride period of each leg, in seconds; each leg swings for the
    /// first half and stands for the second.
    pub period_s: f64,
    /// Peak foot lift during swing, in meters.
    pub swing_height_m: f64,
    /// Raibert gain: touchdown shift per unit of velocity error, in seconds.
    pub raibert_gain_s: f64,
    /// Share of the velocity and yaw-rate error added to the stance-foot
    /// sweep, so the stance legs push the body toward the command.
    pub stance_feedback_gain: f64,
    /// Multiplier on the measured roll and pitch when the foot plan is rotated
    /// from the level frame into the body (1 levels the body geometrically).
    pub attitude_gain: f64,
    /// Heading correction rate per radian of heading error, in 1/s.
    pub heading_gain_per_s: f64,
    /// Largest horizontal touchdown offset from the stand foot, in meters.
    pub max_step_m: f64,
}

impl Default for TrotParams {
    fn default() -> Self {
        Self {
            period_s: 0.36,
            swing_height_m: 0.07,
            raibert_gain_s: 0.03,
            stance_feedback_gain: 1.0,
            attitude_gain: 0.5,
            heading_gain_per_s: 2.0,
            max_step_m: 0.14,
        }
    }
}

/// Commanded body velocity in the level frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VelocityCommand {
    /// Speed along the heading, in meters per second.
    pub forward_m_s: f64,
    /// Speed to the left of the heading, in meters per second.
    pub lateral_m_s: f64,
    /// Turn rate about the vertical, in radians per second (positive left).
    pub yaw_rate_rad_s: f64,
}

/// A trot that uses the measured base state.
///
/// Feet are planned in the *level frame* — the base frame with roll and
/// pitch removed — and rotated into the base frame through the measured
/// tilt before leg inverse kinematics, so a tilted body pushes its low side up
/// and stays level on uneven ground. A stance foot moves backward at the
/// commanded body velocity (including the turn), and a swing foot lands at
/// the Raibert point: the symmetric touchdown plus a shift proportional to
/// the error between the measured and commanded velocity, which also
/// recovers from pushes. The commanded heading is integrated from the yaw
/// rate and tracked with a proportional correction.
#[derive(Clone, Debug, PartialEq)]
pub struct TrotController {
    params: TrotParams,
    time_s: f64,
    heading_target_rad: Option<f64>,
    /// Level-frame foot at the start of the current swing or stance.
    phase_start: [Vec3; 4],
    /// Level-frame foot planned on the previous update.
    last_foot: [Vec3; 4],
    was_swing: [Option<bool>; 4],
    /// Joint targets of the previous update, for the velocity targets.
    last_positions_rad: Option<Vec<f64>>,
}

impl TrotController {
    /// Creates a controller at the start of its gait cycle.
    pub fn new(params: TrotParams) -> Self {
        Self {
            params,
            time_s: 0.0,
            heading_target_rad: None,
            phase_start: [Vec3::ZERO; 4],
            last_foot: [Vec3::ZERO; 4],
            was_swing: [None; 4],
            last_positions_rad: None,
        }
    }

    /// Restarts the gait cycle and forgets the heading target.
    pub fn reset(&mut self) {
        *self = Self::new(self.params);
    }

    /// Controller tuning.
    pub fn params(&self) -> &TrotParams {
        &self.params
    }

    /// Gait clock in `[0, 1)`: the phase of the front-left leg.
    pub fn phase(&self) -> f64 {
        (self.time_s / self.params.period_s).rem_euclid(1.0)
    }

    /// Command that holds every foot at its stand position in the level
    /// frame, leveling the body without stepping.
    pub fn stand_targets(&self, quadruped: &Quadruped, state: &BaseState) -> JointCommand {
        let feet = quadruped.legs().map(|leg| leg.stand_foot_m);
        self.to_joint_command(quadruped, state, &feet, [true; 4])
    }

    /// Advances the gait by `dt_s` and returns the actuator command that
    /// tracks `command` at the measured `state`.
    pub fn update(
        &mut self,
        quadruped: &Quadruped,
        state: &BaseState,
        command: &VelocityCommand,
        dt_s: f64,
    ) -> JointCommand {
        let params = self.params;
        let heading = state.heading_rad();
        let target = self.heading_target_rad.get_or_insert(heading);
        let heading_error = wrap_angle(*target - heading);
        *target += command.yaw_rate_rad_s * dt_s;
        let yaw_rate = command.yaw_rate_rad_s + params.heading_gain_per_s * heading_error;
        let body_velocity = Vec3::new(command.forward_m_s, command.lateral_m_s, 0.0);
        let measured = state.level_linear_velocity_m_s();
        let velocity_error = Vec3::new(
            measured.x - command.forward_m_s,
            measured.y - command.lateral_m_s,
            0.0,
        );
        let half_period = 0.5 * params.period_s;

        let mut feet = [Vec3::ZERO; 4];
        let mut stance = [false; 4];
        for (leg, geometry) in quadruped.legs().iter().enumerate() {
            let stand = geometry.stand_foot_m;
            let phase = (self.time_s / params.period_s + LEG_PHASE[leg]).rem_euclid(1.0);
            let swing = phase < 0.5;
            stance[leg] = !swing;
            let progress = if swing {
                2.0 * phase
            } else {
                2.0 * phase - 1.0
            };
            if self.was_swing[leg] != Some(swing) {
                self.phase_start[leg] = if self.was_swing[leg].is_some() {
                    self.last_foot[leg]
                } else {
                    stand
                };
                self.was_swing[leg] = Some(swing);
            }
            // Velocity of the ground under this foot relative to the body.
            // The stance feet sweep at the commanded velocity plus a share of
            // its error, so they push the body toward the command.
            let stance_velocity = body_velocity - velocity_error * params.stance_feedback_gain;
            let stance_yaw_rate = yaw_rate
                + (yaw_rate - state.angular_velocity_rad_s.z) * params.stance_feedback_gain;
            let sweep = stance_velocity
                + Vec3::new(-stance_yaw_rate * stand.y, stance_yaw_rate * stand.x, 0.0);
            let foot = if swing {
                // Raibert: the neutral point for the measured velocity, plus a
                // correction toward the commanded one.
                let neutral = (sweep + velocity_error) * (0.5 * half_period);
                let mut offset = neutral + velocity_error * params.raibert_gain_s;
                offset.z = 0.0;
                let length = offset.length();
                if length > params.max_step_m {
                    offset *= params.max_step_m / length;
                }
                let touchdown = Vec3::new(stand.x + offset.x, stand.y + offset.y, stand.z);
                let blend = 0.5 * (1.0 - (PI * progress).cos());
                let start = self.phase_start[leg];
                let mut foot = start + (touchdown - start) * blend;
                foot.z = stand.z + params.swing_height_m * (PI * progress).sin();
                foot
            } else {
                let mut foot = self.phase_start[leg] - sweep * (half_period * progress);
                foot.z = stand.z;
                foot
            };
            self.last_foot[leg] = foot;
            feet[leg] = foot;
        }
        self.time_s += dt_s;
        let mut command = self.to_joint_command(quadruped, state, &feet, stance);
        // Track the plan's joint rates so the PD damping does not resist it.
        command.velocities_rad_s = match &self.last_positions_rad {
            Some(last) if dt_s > 0.0 => command
                .positions_rad
                .iter()
                .zip(last)
                .map(|(now, before)| (now - before) / dt_s)
                .collect(),
            _ => vec![0.0; command.positions_rad.len()],
        };
        self.last_positions_rad = Some(command.positions_rad.clone());
        command
    }

    /// Leg inverse kinematics of the level-frame feet, plus feed-forward
    /// torques that carry the body's weight on the stance legs.
    fn to_joint_command(
        &self,
        quadruped: &Quadruped,
        state: &BaseState,
        feet_level: &[Vec3; 4],
        stance: [bool; 4],
    ) -> JointCommand {
        let tilt = state.tilt();
        let correction = Quat::from_scaled_axis(tilt.to_scaled_axis() * self.params.attitude_gain);
        let stance_count = stance.iter().filter(|leg| **leg).count().max(1);
        // Ground force on each stance foot, from the level frame into the base.
        let support = tilt.conjugate()
            * Vec3::new(
                0.0,
                0.0,
                quadruped.mass_kg() * GRAVITY_M_S2 / stance_count as f64,
            );
        let mut command = JointCommand {
            positions_rad: quadruped.stand_pose_rad().to_vec(),
            velocities_rad_s: Vec::new(),
            feedforward_nm: vec![0.0; quadruped.actuated_count()],
        };
        for (leg, (geometry, foot)) in quadruped.legs().iter().zip(feet_level).enumerate() {
            let angles = quadruped.leg_inverse_kinematics(leg, correction * *foot);
            let torques = if stance[leg] {
                quadruped.leg_force_torques(leg, angles, support)
            } else {
                [0.0; 3]
            };
            for ((joint, angle), torque) in geometry.joints.iter().zip(angles).zip(torques) {
                command.positions_rad[*joint] = angle;
                command.feedforward_nm[*joint] = torque;
            }
        }
        command
    }
}

/// Wraps an angle into `(-π, π]`.
fn wrap_angle(angle: f64) -> f64 {
    let wrapped = (angle + PI).rem_euclid(2.0 * PI) - PI;
    if wrapped <= -PI {
        wrapped + 2.0 * PI
    } else {
        wrapped
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::{QuadrupedTerrainSim, TerrainSimConfig};
    use rne_physics::FractalTerrain;
    use std::sync::Arc;

    fn rough_terrain(seed: u64) -> rne_physics::ColliderShape {
        FractalTerrain {
            size_x_m: 12.0,
            size_z_m: 12.0,
            columns: 241,
            rows: 241,
            frequency_per_m: 0.5,
            amplitude_m: 0.04,
            ..FractalTerrain::default()
        }
        .height_field(seed)
        .expect("terrain")
    }

    fn run(
        command: VelocityCommand,
        seconds: f64,
        push: Option<(f64, Vec3)>,
    ) -> (QuadrupedTerrainSim, f64) {
        let go2 = Arc::new(Quadruped::go2().expect("go2"));
        let mut sim =
            QuadrupedTerrainSim::new(go2.clone(), rough_terrain(7), TerrainSimConfig::default())
                .expect("sim");
        let mut controller = TrotController::new(TrotParams::default());
        let dt = sim.config().step_time_s;
        let mut max_tilt: f64 = 0.0;
        for index in 0..(seconds / dt).round() as usize {
            let t = index as f64 * dt;
            let state = sim.base_state();
            let targets = if t < 0.5 {
                controller.stand_targets(&go2, &state)
            } else {
                controller.update(&go2, &state, &command, dt)
            };
            let force = match push {
                Some((at, force)) if (at..at + 0.1).contains(&t) => force,
                _ => Vec3::ZERO,
            };
            sim.step(&targets, force).expect("step");
            max_tilt = max_tilt.max(sim.base_state().tilt_rad());
        }
        (sim, max_tilt)
    }

    #[test]
    fn wrap_angle_stays_in_range() {
        assert!((wrap_angle(3.0 * PI) - PI).abs() < 1.0e-12);
        assert!((wrap_angle(-0.5) + 0.5).abs() < 1.0e-12);
        assert!((wrap_angle(7.0) - (7.0 - 2.0 * PI)).abs() < 1.0e-12);
    }

    #[test]
    fn trot_tracks_a_forward_command_on_rough_terrain() {
        let command = VelocityCommand {
            forward_m_s: 0.4,
            ..VelocityCommand::default()
        };
        let (sim, max_tilt) = run(command, 4.5, None);
        let state = sim.base_state();
        let distance = state.position_m.x;
        assert!(distance > 1.0, "distance {distance}");
        assert!(
            state.position_m.y.abs() < 0.3,
            "drift {}",
            state.position_m.y
        );
        assert!(state.heading_rad().abs() < 0.15);
        assert!(max_tilt < 0.25, "tilt {max_tilt}");
    }

    #[test]
    fn trot_turns_past_a_quarter_turn() {
        let command = VelocityCommand {
            forward_m_s: 0.2,
            yaw_rate_rad_s: 0.6,
            ..VelocityCommand::default()
        };
        let (sim, max_tilt) = run(command, 4.5, None);
        let heading = sim.base_state().heading_rad();
        // 4 s of turning at 0.6 rad/s is 2.4 rad.
        assert!(heading > 1.8, "heading {heading}");
        assert!(max_tilt < 0.3, "tilt {max_tilt}");
    }

    #[test]
    fn trot_recovers_from_a_lateral_push() {
        let command = VelocityCommand {
            forward_m_s: 0.3,
            ..VelocityCommand::default()
        };
        let (sim, max_tilt) = run(command, 4.0, Some((2.0, Vec3::new(0.0, 120.0, 0.0))));
        let state = sim.base_state();
        assert!(max_tilt < 0.5, "tilt {max_tilt}");
        assert!(state.tilt_rad() < 0.15);
        assert!(state.level_linear_velocity_m_s().y.abs() < 0.2);
    }
}
