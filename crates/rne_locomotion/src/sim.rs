//! Hard-contact quadruped simulation over a height-field terrain.

use crate::quadruped::{terrain_surface, BaseState, LocomotionError, Quadruped};
use rne_dynamics::{
    contact_frame, contact_step, ContactOutcome, ContactPoint, ContactStepConfig, JointPdControl,
};
use rne_math::Vec3;
use rne_physics::ColliderShape;
use std::sync::Arc;

/// Settings of [`QuadrupedTerrainSim`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TerrainSimConfig {
    /// Step size, in seconds.
    pub step_time_s: f64,
    /// Joint PD stiffness, in N·m per radian.
    pub position_gain_nm_rad: f64,
    /// Joint PD damping, in N·m·s per radian.
    pub velocity_gain_nm_s_rad: f64,
    /// Foot-terrain Coulomb friction coefficient.
    pub friction_coefficient: f64,
    /// Contact step settings; its `step_time_s` is replaced by
    /// [`Self::step_time_s`].
    pub contact: ContactStepConfig,
}

impl Default for TerrainSimConfig {
    fn default() -> Self {
        Self {
            step_time_s: 0.002,
            position_gain_nm_rad: 100.0,
            velocity_gain_nm_s_rad: 3.0,
            friction_coefficient: 0.8,
            contact: ContactStepConfig::default(),
        }
    }
}

/// Actuator command for one step.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct JointCommand {
    /// PD position target of each actuated joint, in radians.
    pub positions_rad: Vec<f64>,
    /// PD velocity target of each actuated joint, in radians per second
    /// (empty for zero).
    pub velocities_rad_s: Vec<f64>,
    /// Feed-forward torque of each actuated joint, in N·m (empty for none).
    pub feedforward_nm: Vec<f64>,
}

impl JointCommand {
    /// A pure position command.
    pub fn positions(positions_rad: Vec<f64>) -> Self {
        Self {
            positions_rad,
            velocities_rad_s: Vec::new(),
            feedforward_nm: Vec::new(),
        }
    }
}

/// Result of one [`QuadrupedTerrainSim::step`].
#[derive(Clone, Debug, PartialEq)]
pub struct SimStep {
    /// Foot contacts that entered the step, in leg order.
    pub contacts: Vec<ContactPoint>,
    /// Their resolved impulses and states.
    pub outcomes: Vec<ContactOutcome>,
    /// Torque applied to each actuated joint, in N·m.
    pub actuator_torques: Vec<f64>,
    /// Number of actuators held at their effort limit.
    pub saturated_joints: usize,
    /// Gauss-Seidel sweeps the contact solver used.
    pub solver_iterations: usize,
    /// Whether the contact solver converged.
    pub solver_converged: bool,
}

impl SimStep {
    /// Smallest foot-terrain gap entering the step, in meters (negative when
    /// a foot penetrates).
    pub fn deepest_gap_m(&self) -> f64 {
        self.contacts
            .iter()
            .map(|contact| contact.gap_m)
            .fold(f64::INFINITY, f64::min)
    }

    /// Whether leg `leg`'s foot carried a contact impulse.
    pub fn foot_loaded(&self, leg: usize) -> bool {
        self.outcomes
            .get(leg)
            .is_some_and(|outcome| outcome.impulse_world_n_s.length() > 0.0)
    }
}

/// A quadruped stepping over a Y-up height-field terrain.
///
/// The state lives in the Z-up locomotion frame (see the crate docs). Each
/// [`Self::step`] places one sphere contact per foot on the terrain, applies
/// implicit joint PD toward the given targets within the actuators' effort
/// limits, and advances the floating-base model with
/// [`rne_dynamics::contact_step`], warm-starting the contact solver from the
/// previous step.
#[derive(Clone, Debug)]
pub struct QuadrupedTerrainSim {
    quadruped: Arc<Quadruped>,
    terrain: ColliderShape,
    config: TerrainSimConfig,
    q: Vec<f64>,
    qd: Vec<f64>,
    warm: Option<Vec<[f64; 3]>>,
    time_s: f64,
}

impl QuadrupedTerrainSim {
    /// Creates a simulation standing at the locomotion-frame origin.
    pub fn new(
        quadruped: Arc<Quadruped>,
        terrain: ColliderShape,
        config: TerrainSimConfig,
    ) -> Result<Self, LocomotionError> {
        let valid = config.step_time_s.is_finite()
            && config.step_time_s > 0.0
            && config.position_gain_nm_rad.is_finite()
            && config.position_gain_nm_rad >= 0.0
            && config.velocity_gain_nm_s_rad.is_finite()
            && config.velocity_gain_nm_s_rad >= 0.0
            && config.friction_coefficient.is_finite()
            && config.friction_coefficient >= 0.0;
        if !valid {
            return Err(LocomotionError::InvalidInput("terrain sim config"));
        }
        let nv = quadruped.model().nv();
        let mut sim = Self {
            quadruped,
            terrain,
            config,
            q: vec![0.0; nv],
            qd: vec![0.0; nv],
            warm: None,
            time_s: 0.0,
        };
        sim.reset_standing(0.0, 0.0, 0.0)?;
        Ok(sim)
    }

    /// Puts the robot at rest in its stand pose above locomotion-frame
    /// `(x, y)`, facing `heading_rad`, with its lowest foot 5 mm above the
    /// terrain, and resets the clock.
    pub fn reset_standing(
        &mut self,
        x_m: f64,
        y_m: f64,
        heading_rad: f64,
    ) -> Result<(), LocomotionError> {
        if !(x_m.is_finite() && y_m.is_finite() && heading_rad.is_finite()) {
            return Err(LocomotionError::InvalidInput("reset pose"));
        }
        let model = self.quadruped.model();
        let base = model.base_dof();
        let mut q = vec![0.0; model.nv()];
        q[0] = x_m;
        q[1] = y_m;
        q[5] = heading_rad;
        q[base..].copy_from_slice(self.quadruped.stand_pose_rad());
        let lowest = self
            .quadruped
            .foot_contacts(&self.terrain, &q, self.config.friction_coefficient)?
            .iter()
            .map(|contact| contact.gap_m)
            .fold(f64::INFINITY, f64::min);
        q[2] = 0.005 - lowest;
        self.q = q;
        self.qd = vec![0.0; model.nv()];
        self.warm = None;
        self.time_s = 0.0;
        Ok(())
    }

    /// Advances one step under `command`, with an extra `base_force_n`
    /// (locomotion frame, through the base origin) held over the step.
    pub fn step(
        &mut self,
        command: &JointCommand,
        base_force_n: Vec3,
    ) -> Result<SimStep, LocomotionError> {
        let model = self.quadruped.model();
        let base = model.base_dof();
        let actuated = self.quadruped.actuated_count();
        let optional = |values: &[f64]| values.is_empty() || values.len() == actuated;
        if command.positions_rad.len() != actuated
            || !optional(&command.velocities_rad_s)
            || !optional(&command.feedforward_nm)
        {
            return Err(LocomotionError::InvalidInput(
                "one value per actuated joint",
            ));
        }
        if !base_force_n.is_finite() {
            return Err(LocomotionError::InvalidInput("base force"));
        }
        let contacts = self.quadruped.foot_contacts(
            &self.terrain,
            &self.q,
            self.config.friction_coefficient,
        )?;
        let mut tau = vec![0.0; model.nv()];
        let rotation = self.base_state().rotation;
        let body_force = rotation.conjugate() * base_force_n;
        tau[..3].copy_from_slice(&[body_force.x, body_force.y, body_force.z]);
        for (slot, torque) in tau[base..].iter_mut().zip(&command.feedforward_nm) {
            *slot = *torque;
        }
        let pd = JointPdControl {
            position_gains: vec![self.config.position_gain_nm_rad; actuated],
            velocity_gains: vec![self.config.velocity_gain_nm_s_rad; actuated],
            target_positions: command.positions_rad.clone(),
            target_velocities: if command.velocities_rad_s.is_empty() {
                vec![0.0; actuated]
            } else {
                command.velocities_rad_s.clone()
            },
        };
        let config = ContactStepConfig {
            step_time_s: self.config.step_time_s,
            ..self.config.contact
        };
        let step = contact_step(
            model,
            &self.q,
            &self.qd,
            &tau,
            &contacts,
            Some(&pd),
            &config,
            self.warm.as_deref(),
        )?;
        self.warm = Some(
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
        self.q = step.q;
        self.qd = step.qd;
        self.time_s += self.config.step_time_s;
        Ok(SimStep {
            contacts,
            outcomes: step.contacts,
            actuator_torques: step.actuator_torques,
            saturated_joints: step.saturated_joints.len(),
            solver_iterations: step.solver_iterations,
            solver_converged: step.solver_converged,
        })
    }

    /// The simulated quadruped.
    pub fn quadruped(&self) -> &Arc<Quadruped> {
        &self.quadruped
    }

    /// The Y-up terrain height field.
    pub fn terrain(&self) -> &ColliderShape {
        &self.terrain
    }

    /// Simulation settings.
    pub fn config(&self) -> &TerrainSimConfig {
        &self.config
    }

    /// Configuration `(x, y, z, roll, pitch, yaw, joints…)` in the locomotion
    /// frame.
    pub fn q(&self) -> &[f64] {
        &self.q
    }

    /// Generalized velocity (base body twist, then joint rates).
    pub fn qd(&self) -> &[f64] {
        &self.qd
    }

    /// Actuated joint angles, in radians.
    pub fn joint_positions_rad(&self) -> &[f64] {
        &self.q[self.quadruped.model().base_dof()..]
    }

    /// Actuated joint rates, in radians per second.
    pub fn joint_velocities_rad_s(&self) -> &[f64] {
        &self.qd[self.quadruped.model().base_dof()..]
    }

    /// Simulated time since the last reset, in seconds.
    pub fn time_s(&self) -> f64 {
        self.time_s
    }

    /// Base pose and velocity.
    pub fn base_state(&self) -> BaseState {
        self.quadruped.base_state(&self.q, &self.qd)
    }

    /// Height of the base origin above the terrain beneath it, in meters.
    pub fn base_clearance_m(&self) -> Result<f64, LocomotionError> {
        let (height, _) = terrain_surface(&self.terrain, self.q[0], self.q[1])?;
        Ok(self.q[2] - height)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_physics::FractalTerrain;

    fn flat_terrain() -> ColliderShape {
        FractalTerrain {
            size_x_m: 4.0,
            size_z_m: 4.0,
            columns: 41,
            rows: 41,
            amplitude_m: 0.0,
            ..FractalTerrain::default()
        }
        .height_field(1)
        .expect("terrain")
    }

    #[test]
    fn go2_stands_on_flat_ground_and_carries_its_weight() {
        let go2 = Arc::new(Quadruped::go2().expect("go2"));
        let mut sim =
            QuadrupedTerrainSim::new(go2.clone(), flat_terrain(), TerrainSimConfig::default())
                .expect("sim");
        let targets = JointCommand::positions(go2.stand_pose_rad().to_vec());
        let mut last = None;
        for _ in 0..1000 {
            last = Some(sim.step(&targets, Vec3::ZERO).expect("step"));
        }
        let last = last.expect("stepped");
        let mass: f64 = (0..go2.model().link_count())
            .filter_map(|index| go2.model().link_inertia(index))
            .map(|inertia| inertia.mass_kg)
            .sum();
        let support: f64 = last
            .outcomes
            .iter()
            .map(|outcome| outcome.impulse_world_n_s.z / sim.config().step_time_s)
            .sum();
        assert!(
            (support - 9.81 * mass).abs() < 0.02 * 9.81 * mass,
            "{support}"
        );
        assert!((0..4).all(|leg| last.foot_loaded(leg)));
        let state = sim.base_state();
        // Joint PD alone lets the off-center body settle a few mrad nose-up.
        assert!(state.tilt_rad() < 0.02);
        assert!(state.linear_velocity_m_s.length() < 0.02);
        assert!(sim.base_clearance_m().expect("clearance") > 0.25);
    }

    #[test]
    fn a_base_force_pushes_the_robot_along_the_locomotion_frame() {
        let go2 = Arc::new(Quadruped::go2().expect("go2"));
        let config = TerrainSimConfig {
            friction_coefficient: 0.0,
            ..TerrainSimConfig::default()
        };
        let mut sim = QuadrupedTerrainSim::new(go2.clone(), flat_terrain(), config).expect("sim");
        let targets = JointCommand::positions(go2.stand_pose_rad().to_vec());
        for _ in 0..100 {
            sim.step(&targets, Vec3::new(0.0, 20.0, 0.0)).expect("step");
        }
        let velocity = sim.base_state().linear_velocity_m_s;
        assert!(
            velocity.y > 0.1 && velocity.x.abs() < 0.1 * velocity.y,
            "{velocity}"
        );
    }
}
