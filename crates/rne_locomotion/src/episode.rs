//! Reinforcement-learning episode: a quadruped on randomized terrain.

use crate::quadruped::{LocomotionError, Quadruped};
use crate::sim::{JointCommand, QuadrupedTerrainSim, TerrainSimConfig};
use crate::trot::{TrotController, TrotParams, VelocityCommand};
use rne_ai::{
    ActionSpec, Episode, EpisodeStep, ObservationSpec, ResetSpec, RewardSpec, RewardTermSpec,
    TaskSpec, TensorBounds, TensorDType, TensorSpec, TerminationConditionSpec, TerminationKind,
    TerminationSpec,
};
use rne_math::Vec3;
use rne_physics::FractalTerrain;
use std::f64::consts::PI;
use std::sync::Arc;

/// Number of actuated joints, and so of action entries.
pub const QUADRUPED_ACTION_DIM: usize = 12;

/// Length of [`QuadrupedObservation::to_vec`].
pub const QUADRUPED_OBSERVATION_DIM: usize = 3 + 3 + 3 + 3 + 12 + 12 + 12 + 2;

/// Simulation steps the robot stands before an episode hands over control.
const SETTLE_STEPS: usize = 150;

/// What a zero action means.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ActionReference {
    /// Actions are offsets from the stand pose; the policy must learn to walk.
    StandPose,
    /// Actions are residual offsets on top of [`TrotController`] tracking the
    /// episode's command; a zero action trots.
    #[default]
    Trot,
}

/// Settings of [`QuadrupedTerrainEpisode`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QuadrupedTerrainConfig {
    /// Terrain generator; the terrain is centered on the start.
    pub terrain: FractalTerrain,
    /// Draw a new terrain seed every episode (otherwise one per environment).
    pub randomize_terrain: bool,
    /// Simulation settings.
    pub sim: TerrainSimConfig,
    /// Trot tuning for [`ActionReference::Trot`].
    pub trot: TrotParams,
    /// Meaning of a zero action.
    pub reference: ActionReference,
    /// Joint offset for an action of ±1, in radians.
    pub action_scale_rad: f64,
    /// Simulation steps per control step.
    pub control_decimation: u32,
    /// Control steps before an episode is truncated.
    pub max_steps: u64,
    /// Range of the commanded forward speed, in meters per second.
    pub forward_command_m_s: [f64; 2],
    /// Range of the commanded yaw rate, in radians per second.
    pub yaw_rate_command_rad_s: [f64; 2],
    /// Base clearance below which the robot has fallen, in meters.
    pub fall_clearance_m: f64,
    /// Tilt beyond which the robot has fallen, in radians.
    pub max_tilt_rad: f64,
}

impl Default for QuadrupedTerrainConfig {
    fn default() -> Self {
        Self {
            terrain: FractalTerrain {
                size_x_m: 12.0,
                size_z_m: 12.0,
                columns: 121,
                rows: 121,
                frequency_per_m: 0.5,
                amplitude_m: 0.04,
                ..FractalTerrain::default()
            },
            randomize_terrain: true,
            sim: TerrainSimConfig::default(),
            trot: TrotParams::default(),
            reference: ActionReference::Trot,
            action_scale_rad: 0.25,
            control_decimation: 10,
            max_steps: 500,
            forward_command_m_s: [0.2, 0.5],
            yaw_rate_command_rad_s: [-0.4, 0.4],
            fall_clearance_m: 0.15,
            max_tilt_rad: 0.8,
        }
    }
}

/// Normalized joint action in `[-1, 1]^12`, in actuated-joint order.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct QuadrupedJointAction {
    /// Joint offsets as fractions of the action scale.
    pub joint_action: [f64; QUADRUPED_ACTION_DIM],
}

impl QuadrupedJointAction {
    /// Returns the action with non-finite entries zeroed and the rest clamped
    /// to `[-1, 1]`.
    pub fn clamped(self) -> Self {
        Self {
            joint_action: self.joint_action.map(|value| {
                if value.is_finite() {
                    value.clamp(-1.0, 1.0)
                } else {
                    0.0
                }
            }),
        }
    }
}

/// Observation of [`QuadrupedTerrainEpisode`]; every vector is in the base
/// frame unless stated otherwise.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct QuadrupedObservation {
    /// Base linear velocity, in meters per second.
    pub base_linear_velocity_m_s: [f64; 3],
    /// Base angular velocity, in radians per second.
    pub base_angular_velocity_rad_s: [f64; 3],
    /// Unit gravity direction.
    pub projected_gravity: [f64; 3],
    /// Commanded forward speed, lateral speed (m/s), and yaw rate (rad/s).
    pub command: [f64; 3],
    /// Joint angles minus the stand pose, in radians.
    pub joint_position_offset_rad: [f64; QUADRUPED_ACTION_DIM],
    /// Joint rates, in radians per second.
    pub joint_velocity_rad_s: [f64; QUADRUPED_ACTION_DIM],
    /// Previous clamped action.
    pub previous_action: [f64; QUADRUPED_ACTION_DIM],
    /// Sine and cosine of the trot clock.
    pub gait_clock: [f64; 2],
}

impl QuadrupedObservation {
    /// Flattens the observation in field order
    /// ([`QUADRUPED_OBSERVATION_DIM`] values).
    pub fn to_vec(&self) -> Vec<f64> {
        let mut values = Vec::with_capacity(QUADRUPED_OBSERVATION_DIM);
        values.extend_from_slice(&self.base_linear_velocity_m_s);
        values.extend_from_slice(&self.base_angular_velocity_rad_s);
        values.extend_from_slice(&self.projected_gravity);
        values.extend_from_slice(&self.command);
        values.extend_from_slice(&self.joint_position_offset_rad);
        values.extend_from_slice(&self.joint_velocity_rad_s);
        values.extend_from_slice(&self.previous_action);
        values.extend_from_slice(&self.gait_clock);
        values
    }
}

/// Portable task contract of [`QuadrupedTerrainEpisode`].
pub fn quadruped_terrain_task_spec(config: &QuadrupedTerrainConfig) -> TaskSpec {
    let n = QUADRUPED_ACTION_DIM;
    TaskSpec::new(
        "rne.quadruped.terrain_locomotion.v1",
        config.sim.step_time_s * f64::from(config.control_decimation),
        ObservationSpec::new(vec![
            TensorSpec::new("base_linear_velocity_m_s", TensorDType::F64, vec![3], "m/s"),
            TensorSpec::new(
                "base_angular_velocity_rad_s",
                TensorDType::F64,
                vec![3],
                "rad/s",
            ),
            TensorSpec::new("projected_gravity", TensorDType::F64, vec![3], "1"),
            TensorSpec::new("command", TensorDType::F64, vec![3], "m/s, m/s, rad/s"),
            TensorSpec::new(
                "joint_position_offset_rad",
                TensorDType::F64,
                vec![n],
                "rad",
            ),
            TensorSpec::new("joint_velocity_rad_s", TensorDType::F64, vec![n], "rad/s"),
            TensorSpec::new("previous_action", TensorDType::F64, vec![n], "1"),
            TensorSpec::new("gait_clock", TensorDType::F64, vec![2], "1"),
        ]),
        ActionSpec::new(vec![TensorSpec::new(
            "joint_action",
            TensorDType::F64,
            vec![n],
            "1",
        )
        .with_bounds(TensorBounds::broadcast(-1.0, 1.0))]),
        RewardSpec::weighted_sum(vec![
            RewardTermSpec::new("forward_velocity_tracking", 1.0, "1"),
            RewardTermSpec::new("yaw_rate_tracking", 0.5, "1"),
            RewardTermSpec::new("tilt_squared", -1.0, "1"),
            RewardTermSpec::new("torque_ratio_squared", -0.05, "1"),
            RewardTermSpec::new("action_rate_squared", -0.01, "1"),
            RewardTermSpec::new("fallen", -10.0, "1"),
        ]),
        TerminationSpec::new(
            vec![TerminationConditionSpec::new(
                "fallen",
                TerminationKind::Failure,
            )],
            Some(config.max_steps),
        ),
        ResetSpec::splitmix64(true),
    )
}

/// A quadruped trotting over randomized fractal terrain, as an
/// [`rne_ai::Episode`].
///
/// Every reset draws, from the episode's own SplitMix64 stream, a terrain
/// seed (when [`QuadrupedTerrainConfig::randomize_terrain`] is set), a heading,
/// and a forward-speed and yaw-rate command, then lets the robot settle in its
/// stand pose. Each step holds the action for
/// [`QuadrupedTerrainConfig::control_decimation`] simulation steps. The
/// episode terminates when the robot falls and is truncated at the step budget
/// or when it walks off the terrain. Episodes are independent, so
/// [`rne_ai::VectorizedEpisode::step_parallel`] can step many at once.
#[derive(Clone, Debug)]
pub struct QuadrupedTerrainEpisode {
    config: QuadrupedTerrainConfig,
    quadruped: Arc<Quadruped>,
    sim: QuadrupedTerrainSim,
    controller: TrotController,
    rng_state: u64,
    terrain_seed: u64,
    command: VelocityCommand,
    previous_action: [f64; QUADRUPED_ACTION_DIM],
    resets: u32,
    step_in_episode: u64,
}

impl QuadrupedTerrainEpisode {
    /// Creates an episode whose randomness derives from `seed`.
    pub fn new(
        quadruped: Arc<Quadruped>,
        config: QuadrupedTerrainConfig,
        seed: u64,
    ) -> Result<Self, LocomotionError> {
        if quadruped.actuated_count() != QUADRUPED_ACTION_DIM {
            return Err(LocomotionError::InvalidInput("twelve actuated joints"));
        }
        let ranges_valid = [config.forward_command_m_s, config.yaw_rate_command_rad_s]
            .iter()
            .all(|[low, high]| low.is_finite() && high.is_finite() && low <= high);
        if !ranges_valid
            || config.control_decimation == 0
            || config.max_steps == 0
            || !(config.action_scale_rad.is_finite() && config.action_scale_rad >= 0.0)
        {
            return Err(LocomotionError::InvalidInput("episode config"));
        }
        let mut rng_state = seed;
        let terrain_seed = splitmix64(&mut rng_state);
        let terrain = config.terrain.height_field(terrain_seed)?;
        let sim = QuadrupedTerrainSim::new(quadruped.clone(), terrain, config.sim)?;
        Ok(Self {
            config,
            quadruped,
            sim,
            controller: TrotController::new(config.trot),
            rng_state,
            terrain_seed,
            command: VelocityCommand::default(),
            previous_action: [0.0; QUADRUPED_ACTION_DIM],
            resets: 0,
            step_in_episode: 0,
        })
    }

    /// The simulation.
    pub fn sim(&self) -> &QuadrupedTerrainSim {
        &self.sim
    }

    /// The current episode's command.
    pub fn command(&self) -> VelocityCommand {
        self.command
    }

    /// The current episode's terrain seed.
    pub fn terrain_seed(&self) -> u64 {
        self.terrain_seed
    }

    fn uniform(&mut self, [low, high]: [f64; 2]) -> f64 {
        let unit = (splitmix64(&mut self.rng_state) >> 11) as f64 / (1u64 << 53) as f64;
        low + (high - low) * unit
    }

    fn try_reset(&mut self) -> Result<(), LocomotionError> {
        if self.config.randomize_terrain && self.resets > 0 {
            self.terrain_seed = splitmix64(&mut self.rng_state);
            let terrain = self.config.terrain.height_field(self.terrain_seed)?;
            self.sim = QuadrupedTerrainSim::new(self.quadruped.clone(), terrain, self.config.sim)?;
        }
        let heading = self.uniform([-PI, PI]);
        let forward = self.uniform(self.config.forward_command_m_s);
        let yaw_rate = self.uniform(self.config.yaw_rate_command_rad_s);
        self.command = VelocityCommand {
            forward_m_s: forward,
            lateral_m_s: 0.0,
            yaw_rate_rad_s: yaw_rate,
        };
        self.sim.reset_standing(0.0, 0.0, heading)?;
        self.controller.reset();
        for _ in 0..SETTLE_STEPS {
            let targets = self
                .controller
                .stand_targets(&self.quadruped, &self.sim.base_state());
            self.sim.step(&targets, Vec3::ZERO)?;
        }
        Ok(())
    }

    fn observation(&self) -> QuadrupedObservation {
        let state = self.sim.base_state();
        let stand = self.quadruped.stand_pose_rad();
        let mut observation = QuadrupedObservation {
            base_linear_velocity_m_s: state.body_linear_velocity_m_s().to_array(),
            base_angular_velocity_rad_s: state.body_angular_velocity_rad_s().to_array(),
            projected_gravity: state.projected_gravity().to_array(),
            command: [
                self.command.forward_m_s,
                self.command.lateral_m_s,
                self.command.yaw_rate_rad_s,
            ],
            previous_action: self.previous_action,
            ..QuadrupedObservation::default()
        };
        for (index, (angle, rate)) in self
            .sim
            .joint_positions_rad()
            .iter()
            .zip(self.sim.joint_velocities_rad_s())
            .enumerate()
        {
            observation.joint_position_offset_rad[index] = angle - stand[index];
            observation.joint_velocity_rad_s[index] = *rate;
        }
        let clock = 2.0 * PI * self.controller.phase();
        observation.gait_clock = [clock.sin(), clock.cos()];
        observation
    }

    /// Runs one control step; `Ok(None)` when the robot left the terrain.
    fn advance(
        &mut self,
        action: &QuadrupedJointAction,
    ) -> Result<Option<(f64, bool)>, LocomotionError> {
        let dt = self.sim.config().step_time_s;
        let mut torque_ratio_sq = 0.0;
        for _ in 0..self.config.control_decimation {
            let state = self.sim.base_state();
            let mut command = match self.config.reference {
                ActionReference::StandPose => {
                    JointCommand::positions(self.quadruped.stand_pose_rad().to_vec())
                }
                ActionReference::Trot => {
                    self.controller
                        .update(&self.quadruped, &state, &self.command, dt)
                }
            };
            for (target, offset) in command.positions_rad.iter_mut().zip(action.joint_action) {
                *target += self.config.action_scale_rad * offset;
            }
            let step = match self.sim.step(&command, Vec3::ZERO) {
                Ok(step) => step,
                Err(LocomotionError::OffTerrain { .. }) => return Ok(None),
                Err(error) => return Err(error),
            };
            torque_ratio_sq += step
                .actuator_torques
                .iter()
                .zip(self.quadruped.effort_limits())
                .map(|(torque, limit)| (torque / limit).powi(2))
                .sum::<f64>()
                / QUADRUPED_ACTION_DIM as f64;
        }
        let state = self.sim.base_state();
        let clearance = match self.sim.base_clearance_m() {
            Ok(clearance) => clearance,
            Err(LocomotionError::OffTerrain { .. }) => return Ok(None),
            Err(error) => return Err(error),
        };
        let fallen =
            clearance < self.config.fall_clearance_m || state.tilt_rad() > self.config.max_tilt_rad;
        let forward_error = state.level_linear_velocity_m_s().x - self.command.forward_m_s;
        let yaw_error = state.angular_velocity_rad_s.z - self.command.yaw_rate_rad_s;
        let gravity = state.projected_gravity();
        let action_rate: f64 = action
            .joint_action
            .iter()
            .zip(self.previous_action)
            .map(|(now, before)| (now - before).powi(2))
            .sum();
        let reward = (-forward_error * forward_error / 0.1).exp()
            + 0.5 * (-yaw_error * yaw_error / 0.1).exp()
            - (gravity.x * gravity.x + gravity.y * gravity.y)
            - 0.05 * torque_ratio_sq / f64::from(self.config.control_decimation)
            - 0.01 * action_rate
            - if fallen { 10.0 } else { 0.0 };
        Ok(Some((reward, fallen)))
    }
}

impl Episode for QuadrupedTerrainEpisode {
    type Observation = QuadrupedObservation;
    type Action = QuadrupedJointAction;

    fn reset(&mut self) -> EpisodeStep<QuadrupedObservation> {
        self.step_in_episode = 0;
        self.previous_action = [0.0; QUADRUPED_ACTION_DIM];
        self.try_reset()
            .expect("a fresh standing reset stays on the terrain");
        self.resets += 1;
        EpisodeStep {
            observation: self.observation(),
            reward: 0.0,
            terminated: false,
            truncated: false,
        }
    }

    fn step(&mut self, action: QuadrupedJointAction) -> EpisodeStep<QuadrupedObservation> {
        let action = action.clamped();
        self.step_in_episode += 1;
        let outcome = self.advance(&action).expect("quadruped step");
        self.previous_action = action.joint_action;
        let (reward, terminated, off_terrain) = match outcome {
            Some((reward, fallen)) => (reward, fallen, false),
            None => (0.0, false, true),
        };
        EpisodeStep {
            observation: self.observation(),
            reward,
            terminated,
            truncated: off_terrain || self.step_in_episode >= self.config.max_steps,
        }
    }

    fn episode_index(&self) -> u32 {
        self.resets.saturating_sub(1)
    }

    fn step_in_episode(&self) -> u64 {
        self.step_in_episode
    }
}

/// SplitMix64 step.
fn splitmix64(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9e37_79b9_7f4a_7c15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn episode(seed: u64, reference: ActionReference) -> QuadrupedTerrainEpisode {
        let go2 = Arc::new(Quadruped::go2().expect("go2"));
        let config = QuadrupedTerrainConfig {
            reference,
            max_steps: 100,
            action_scale_rad: 1.0,
            ..QuadrupedTerrainConfig::default()
        };
        QuadrupedTerrainEpisode::new(go2, config, seed).expect("episode")
    }

    #[test]
    fn task_spec_matches_the_observation_and_action() {
        let spec = quadruped_terrain_task_spec(&QuadrupedTerrainConfig::default());
        let observation_len: usize = spec
            .observation
            .tensors
            .iter()
            .map(|tensor| tensor.shape.iter().product::<usize>())
            .sum();
        assert_eq!(observation_len, QUADRUPED_OBSERVATION_DIM);
        assert_eq!(
            QuadrupedObservation::default().to_vec().len(),
            QUADRUPED_OBSERVATION_DIM
        );
        assert!((spec.control_step_s - 0.02).abs() < 1.0e-12);
    }

    #[test]
    fn zero_action_trots_and_earns_tracking_reward() {
        let mut episode = episode(11, ActionReference::Trot);
        let first = episode.reset();
        assert!(!first.terminated && !first.truncated);
        assert_eq!(episode.episode_index(), 0);
        let mut total = 0.0;
        let mut last = None;
        for _ in 0..100 {
            let step = episode.step(QuadrupedJointAction::default());
            assert!(!step.terminated, "fell at {}", episode.step_in_episode());
            total += step.reward;
            last = Some(step);
        }
        assert!(last.expect("stepped").truncated);
        // A tracking trot earns most of the 1.5 available per step.
        assert!(total / 100.0 > 0.8, "mean reward {}", total / 100.0);
    }

    #[test]
    fn resets_draw_new_terrain_and_commands_deterministically() {
        let mut a = episode(5, ActionReference::Trot);
        let mut b = episode(5, ActionReference::Trot);
        let first = (a.reset(), a.terrain_seed(), a.command());
        assert_eq!((b.reset(), b.terrain_seed(), b.command()), first);
        let second = (a.reset(), a.terrain_seed(), a.command());
        assert_ne!(second.1, first.1);
        assert_ne!(second.2, first.2);
        assert_eq!(a.episode_index(), 1);
        let action = QuadrupedJointAction {
            joint_action: [0.3; QUADRUPED_ACTION_DIM],
        };
        b.reset();
        assert_eq!(a.step(action), b.step(action));
    }

    #[test]
    fn a_large_stand_pose_action_makes_the_robot_fall() {
        let mut episode = episode(3, ActionReference::StandPose);
        episode.reset();
        // Fold every thigh forward and every calf in by 1 rad.
        let mut collapse = QuadrupedJointAction::default();
        for leg in episode.sim().quadruped().legs() {
            collapse.joint_action[leg.joints[1]] = 1.0;
            collapse.joint_action[leg.joints[2]] = -1.0;
        }
        let fell = (0..100).any(|_| episode.step(collapse).terminated);
        assert!(fell);
    }
}
