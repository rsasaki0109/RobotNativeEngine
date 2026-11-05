//! The Go2 on `rne_locomotion`'s terrain simulation: `rne_dynamics`' hard
//! contact step on a flat height field, behind the same [`Robot`] interface as
//! the Rapier walker.
//!
//! The model is the URDF's articulated Go2 with its fixed links fused and the
//! declared masses (16.087 kg), as on the Rapier and MuJoCo plants. The feet
//! are spheres, and spheres along each leg stand in for the knee, the calf
//! cylinder and the two lower-calf bumpers, so a stride that kneels lands on
//! them. The thighs, hips, head and body box have no contact: a robot that
//! falls onto them sinks through the floor, which the episode's fall test ends
//! as a fall.
//!
//! The joints track their targets through an explicit PD torque computed at
//! the start of every physics step and clamped to the effort limits, as
//! Rapier's walker and MuJoCo's position actuators do, rather than through
//! `rne_locomotion`'s implicit PD: with the implicit law the same ES finds
//! only a 0.4 m shuffle in 200 generations.

use std::sync::Arc;

use rne_locomotion::{BodySphere, JointCommand, Quadruped, QuadrupedTerrainSim, TerrainSimConfig};
use rne_math::{Quat, Vec3};
use rne_physics::FractalTerrain;

use crate::go2::{
    Robot, DECIMATION, DEFAULT_RAD, DT_S, KD_NM_S_PER_RAD, KP_NM_PER_RAD, SPAWN_HEIGHT_M,
};
use crate::Result;

/// Side of the square flat floor, in meters: an episode walks a few meters
/// from the origin.
const FLOOR_SIZE_M: f64 = 40.0;
/// Bisection steps of the sliding-friction solve: 30 resolve the friction
/// direction to 2e-10 rad, where the default 60 run past double precision.
const SLIDING_BISECTION_ITERATIONS: usize = 30;
/// Contact solver stopping rule: a sweep that changes no impulse by more than
/// 1e-5 N·s (a stance foot carries about 0.8 N·s per step) and no contact
/// velocity by more than 0.1 mm/s, or 30 sweeps. The library's defaults
/// (1e-10 N·s, 1e-8 m/s, 200 sweeps) let a kneeling gait, whose foot, knee
/// and shin contacts on one calf are nearly redundant, run hundreds of sweeps
/// a step; an unconverged sweep's impulses still lie in the friction cone.
const SOLVER_TOLERANCE_N_S: f64 = 1.0e-5;
const SOLVER_VELOCITY_TOLERANCE_M_S: f64 = 1.0e-4;
const SOLVER_MAX_SWEEPS: usize = 30;

/// A Go2 on the native terrain step, restarted in its default pose by
/// [`Robot::restart`].
pub(crate) struct LocomotionGo2 {
    sim: QuadrupedTerrainSim,
    /// Actuated coordinate of each joint in [`crate::go2::LINKS`] order.
    actuated: [usize; 12],
    /// PD targets, by actuated coordinate.
    targets_rad: [f64; 12],
    command: JointCommand,
}

impl LocomotionGo2 {
    pub(crate) fn new(floor_friction: f64) -> Result<Self> {
        let quadruped = Arc::new(Quadruped::go2()?);
        // LINKS runs leg by leg (FL, FR, RL, RR), hip, thigh and calf, as the
        // Go2 spec lists its legs.
        let legs = quadruped.legs();
        let actuated = std::array::from_fn(|index| legs[index / 3].joints[index % 3]);
        let floor = FractalTerrain {
            size_x_m: FLOOR_SIZE_M,
            size_z_m: FLOOR_SIZE_M,
            columns: 5,
            rows: 5,
            amplitude_m: 0.0,
            ..FractalTerrain::default()
        }
        .height_field(0)?;
        let mut config = TerrainSimConfig {
            step_time_s: DT_S,
            // The PD is explicit, in the commanded torque.
            position_gain_nm_rad: 0.0,
            velocity_gain_nm_s_rad: 0.0,
            friction_coefficient: floor_friction,
            ..TerrainSimConfig::default()
        };
        config.contact.solver.bisection_iterations = SLIDING_BISECTION_ITERATIONS;
        config.contact.solver.tolerance_n_s = SOLVER_TOLERANCE_N_S;
        config.contact.solver.velocity_tolerance_m_s = SOLVER_VELOCITY_TOLERANCE_M_S;
        config.contact.solver.max_iterations = SOLVER_MAX_SWEEPS;
        let spheres = leg_spheres(&quadruped)?;
        let mut sim = QuadrupedTerrainSim::new(quadruped, floor, config)?;
        sim.set_body_spheres(spheres)?;
        let mut robot = Self {
            sim,
            actuated,
            targets_rad: [0.0; 12],
            command: JointCommand::positions(vec![0.0; 12]),
        };
        robot.restart()?;
        Ok(robot)
    }

    /// The robot's total mass, in kilograms.
    pub(crate) fn mass_kg(&self) -> f64 {
        self.sim.quadruped().mass_kg()
    }
}

impl Robot for LocomotionGo2 {
    fn restart(&mut self) -> Result<()> {
        let nv = self.sim.quadruped().model().nv();
        let mut q = vec![0.0; nv];
        q[2] = SPAWN_HEIGHT_M;
        for (index, angle) in DEFAULT_RAD.iter().enumerate() {
            q[6 + self.actuated[index]] = *angle;
        }
        self.sim.set_state(&q, &vec![0.0; nv])?;
        self.set_targets_rad(&DEFAULT_RAD)
    }

    fn set_targets_rad(&mut self, targets_rad: &[f64; 12]) -> Result<()> {
        for (index, target) in targets_rad.iter().enumerate() {
            self.targets_rad[self.actuated[index]] = *target;
        }
        Ok(())
    }

    fn step(&mut self) -> Result<()> {
        for _ in 0..DECIMATION {
            let positions = self.sim.joint_positions_rad();
            let velocities = self.sim.joint_velocities_rad_s();
            let limits = self.sim.quadruped().effort_limits();
            let torques: Vec<f64> = (0..12)
                .map(|joint| {
                    (KP_NM_PER_RAD * (self.targets_rad[joint] - positions[joint])
                        - KD_NM_S_PER_RAD * velocities[joint])
                        .clamp(-limits[joint], limits[joint])
                })
                .collect();
            // With zero PD gains the position targets only fill the command.
            self.command.positions_rad.copy_from_slice(positions);
            self.command.feedforward_nm = torques;
            self.sim.step(&self.command, Vec3::ZERO)?;
        }
        Ok(())
    }

    fn base_pose_z_up(&self) -> (Vec3, Quat) {
        let state = self.sim.base_state();
        (state.position_m, state.rotation)
    }

    fn base_velocity_z_up(&self) -> (Vec3, Vec3) {
        let state = self.sim.base_state();
        (state.linear_velocity_m_s, state.angular_velocity_rad_s)
    }

    fn joint_states(&self) -> Result<[(f64, f64); 12]> {
        let positions = self.sim.joint_positions_rad();
        let velocities = self.sim.joint_velocities_rad_s();
        Ok(std::array::from_fn(|index| {
            let joint = self.actuated[index];
            (positions[joint], velocities[joint])
        }))
    }
}

/// Spheres standing in for each leg's collision shapes besides the foot, from
/// the URDF: the knee, the calf cylinder and the two lower-calf bumpers.
fn leg_spheres(quadruped: &Quadruped) -> Result<Vec<BodySphere>> {
    let links = quadruped.model().kinematic();
    let mut spheres = Vec::new();
    for leg in ["FL", "FR", "RL", "RR"] {
        let link = |name: &str| {
            links
                .link_entity_by_name(&format!("{leg}_{name}"))
                .ok_or_else(|| format!("missing link {leg}_{name}"))
        };
        let calf = link("calf")?;
        // The knee: where the 34 mm thigh box ends and the calf begins.
        spheres.push(BodySphere {
            link: calf,
            center_local_m: Vec3::ZERO,
            radius_m: 0.017,
        });
        // The calf cylinder: radius 12 mm, 0.12 m long, centred 6 cm down
        // and tilted 0.21 rad about y.
        let axis = Quat::from_rotation_y(-0.21) * Vec3::Z;
        for offset in [-0.06, 0.0, 0.06] {
            spheres.push(BodySphere {
                link: calf,
                center_local_m: Vec3::new(0.008, 0.0, -0.06) + axis * offset,
                radius_m: 0.012,
            });
        }
        // The lower-calf bumpers: cylinders along their own z.
        for (name, radius_m, half_length_m) in
            [("calflower", 0.011, 0.0325), ("calflower1", 0.0155, 0.015)]
        {
            let bumper = link(name)?;
            for offset in [-half_length_m, 0.0, half_length_m] {
                spheres.push(BodySphere {
                    link: bumper,
                    center_local_m: Vec3::new(0.0, 0.0, offset),
                    radius_m,
                });
            }
        }
    }
    Ok(spheres)
}
