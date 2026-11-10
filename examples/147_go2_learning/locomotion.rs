//! The Go2 on `rne_locomotion`'s terrain simulation: `rne_dynamics`' hard
//! contact step on a flat height field, behind the same [`Robot`] interface as
//! the Rapier walker.
//!
//! The model is the URDF's articulated Go2 with its fixed links fused and the
//! declared masses (16.087 kg), as on the Rapier and MuJoCo plants. Every URDF
//! collision shape touches the ground, as in MuJoCo and on Rapier: the feet
//! are `rne_locomotion`'s foot spheres, and [`collision_spheres`] turns the
//! head, body, hips, thighs, calves and lower-calf bumpers into spheres and
//! points. MuJoCo's walks lean on the head's lower sphere at every stride;
//! with contacts at the feet and lower legs only, they fell through the floor
//! within a second.
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
use rne_urdf_import::{parse_urdf, rpy_to_quat, UrdfGeometry};

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
    /// See [`Robot::arena_half_m`]: the floor's or the terrain's half side,
    /// less [`crate::go2::ARENA_EDGE_MARGIN_M`].
    arena_half_m: f64,
    /// See [`Robot::contact_load`]: by kinematic link, the normal impulse
    /// the ground has put through it and the physics steps it carried any.
    contact_load: Vec<(f64, usize)>,
}

impl LocomotionGo2 {
    /// A Go2 on a flat floor, or on the rough terrain of `terrain_seed` with
    /// `relief`.
    pub(crate) fn new(
        floor_friction: f64,
        terrain_seed: Option<u64>,
        relief: crate::go2::Relief,
    ) -> Result<Self> {
        let quadruped = Arc::new(Quadruped::go2()?);
        // LINKS runs leg by leg (FL, FR, RL, RR), hip, thigh and calf, as the
        // Go2 spec lists its legs.
        let legs = quadruped.legs();
        let actuated = std::array::from_fn(|index| legs[index / 3].joints[index % 3]);
        let side_m = match terrain_seed {
            Some(_) => relief.size_m,
            None => FLOOR_SIZE_M,
        };
        let floor = match terrain_seed {
            Some(seed) => crate::go2::terrain(seed, relief)?,
            None => FractalTerrain {
                size_x_m: FLOOR_SIZE_M,
                size_z_m: FLOOR_SIZE_M,
                columns: 5,
                rows: 5,
                amplitude_m: 0.0,
                ..FractalTerrain::default()
            }
            .height_field(0)?,
        };
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
        let spheres = collision_spheres(&quadruped)?;
        let mut sim = QuadrupedTerrainSim::new(quadruped, floor, config)?;
        sim.set_body_spheres(spheres)?;
        let mut robot = Self {
            sim,
            actuated,
            targets_rad: [0.0; 12],
            command: JointCommand::positions(vec![0.0; 12]),
            arena_half_m: 0.5 * side_m - crate::go2::ARENA_EDGE_MARGIN_M,
            contact_load: Vec::new(),
        };
        let kinematic = robot.sim.quadruped().model().kinematic();
        let links = (0..)
            .take_while(|&index| kinematic.link_entity(index).is_some())
            .count();
        robot.contact_load = vec![(0.0, 0); links];
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
        self.restart_at([0.0, 0.0])
    }

    fn restart_at(&mut self, [x_m, y_m]: [f64; 2]) -> Result<()> {
        let nv = self.sim.quadruped().model().nv();
        let mut q = vec![0.0; nv];
        q[0] = x_m;
        q[1] = y_m;
        // The terrain is shifted to height zero at the origin, so a start there
        // keeps the flat floor's spawn height exactly.
        let ground = if [x_m, y_m] == [0.0, 0.0] {
            0.0
        } else {
            self.ground_height_m(x_m, y_m)
        };
        if !ground.is_finite() {
            return Err(format!("start ({x_m}, {y_m}) is off the terrain").into());
        }
        q[2] = SPAWN_HEIGHT_M + ground;
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
            let step = self.sim.step(&self.command, Vec3::ZERO)?;
            let kinematic = self.sim.quadruped().model().kinematic();
            let mut loaded = vec![false; self.contact_load.len()];
            for (contact, outcome) in step.contacts.iter().zip(&step.outcomes) {
                let normal = outcome.impulse_world_n_s.dot(contact.normal_world);
                if normal <= 0.0 {
                    continue;
                }
                if let Some(index) = kinematic.link_index(contact.link) {
                    self.contact_load[index].0 += normal;
                    loaded[index] = true;
                }
            }
            for (load, loaded) in self.contact_load.iter_mut().zip(loaded) {
                load.1 += usize::from(loaded);
            }
        }
        Ok(())
    }

    fn contact_load(&self) -> Option<Vec<(String, f64, usize)>> {
        let kinematic = self.sim.quadruped().model().kinematic();
        Some(
            self.contact_load
                .iter()
                .enumerate()
                .map(|(index, (impulse, steps))| {
                    let name = kinematic.link_name(index).unwrap_or("?").to_string();
                    (name, *impulse, *steps)
                })
                .collect(),
        )
    }

    fn base_pose_z_up(&self) -> (Vec3, Quat) {
        let state = self.sim.base_state();
        (state.position_m, state.rotation)
    }

    fn base_velocity_z_up(&self) -> (Vec3, Vec3) {
        let state = self.sim.base_state();
        (state.linear_velocity_m_s, state.angular_velocity_rad_s)
    }

    fn ground_height_m(&self, x_m: f64, y_m: f64) -> f64 {
        crate::go2::terrain_height_m(self.sim.terrain(), x_m, y_m)
    }

    fn arena_half_m(&self) -> f64 {
        self.arena_half_m
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

/// Contacts for every collision shape in the URDF MuJoCo and Rapier load,
/// besides the feet, which are the quadruped's foot spheres already:
/// - a sphere as itself (the head's lower half);
/// - a cylinder as spheres of its radius along its axis, one radius apart and
///   one radius in from its ends, or one sphere at its centre when it is
///   shorter than its diameter (the hips and the head's upper half);
/// - a box as points at its eight corners (the body and the thighs): on a
///   plane, a box's lowest point is a corner.
fn collision_spheres(quadruped: &Quadruped) -> Result<Vec<BodySphere>> {
    let urdf = parse_urdf(crate::native::GO2_URDF)?;
    let links = quadruped.model().kinematic();
    let feet: Vec<_> = quadruped.legs().iter().map(|leg| leg.foot_link).collect();
    let mut spheres = Vec::new();
    for link in urdf.links.iter().filter(|link| !link.collisions.is_empty()) {
        let entity = links
            .link_entity_by_name(&link.name)
            .ok_or_else(|| format!("missing link {}", link.name))?;
        if feet.contains(&entity) {
            continue;
        }
        for collision in &link.collisions {
            let rotation = rpy_to_quat(collision.origin_rpy);
            let mut add = |center: Vec3, radius_m: f64| {
                spheres.push(BodySphere {
                    link: entity,
                    center_local_m: collision.origin_xyz + rotation * center,
                    radius_m,
                })
            };
            match collision.geometry {
                UrdfGeometry::Sphere { radius_m } => add(Vec3::ZERO, radius_m),
                UrdfGeometry::Cylinder { radius_m, length_m } => {
                    let reach = (0.5 * length_m - radius_m).max(0.0);
                    let gaps = (2.0 * reach / radius_m).ceil() as usize;
                    for index in 0..=gaps {
                        let along = if gaps == 0 {
                            0.0
                        } else {
                            -reach + 2.0 * reach * index as f64 / gaps as f64
                        };
                        add(Vec3::new(0.0, 0.0, along), radius_m);
                    }
                }
                UrdfGeometry::Box { size_m } => {
                    let half = size_m * 0.5;
                    for corner in 0..8 {
                        let sign = |bit: usize| if corner & bit == 0 { -1.0 } else { 1.0 };
                        add(
                            Vec3::new(sign(1) * half.x, sign(2) * half.y, sign(4) * half.z),
                            0.0,
                        );
                    }
                }
                UrdfGeometry::Mesh { .. } => {
                    return Err(format!("{} collides with a mesh", link.name).into())
                }
            }
        }
    }
    Ok(spheres)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_urdf_collision_shape_but_the_feet_touches_the_ground() {
        let quadruped = Quadruped::go2().unwrap();
        let spheres = collision_spheres(&quadruped).unwrap();
        let links = quadruped.model().kinematic();
        let count = |name: &str| {
            let link = links.link_entity_by_name(name).unwrap();
            spheres.iter().filter(|sphere| sphere.link == link).count()
        };
        // Head: the lower sphere and one sphere for the upper cylinder; the
        // body and each thigh box: eight corners; each hip cylinder: one.
        assert_eq!(count("Head_lower"), 1);
        assert_eq!(count("Head_upper"), 1);
        assert_eq!(count("base"), 8);
        for leg in ["FL", "FR", "RL", "RR"] {
            assert_eq!(count(&format!("{leg}_hip")), 1, "{leg}");
            assert_eq!(count(&format!("{leg}_thigh")), 8, "{leg}");
            // The calf cylinder (12-13 mm radius, 0.12 m long) as spheres
            // a radius apart, and the two bumpers.
            assert!(count(&format!("{leg}_calf")) >= 8, "{leg}");
            assert_eq!(count(&format!("{leg}_calflower")), 5, "{leg}");
            assert_eq!(count(&format!("{leg}_calflower1")), 1, "{leg}");
            assert_eq!(count(&format!("{leg}_foot")), 0, "{leg}");
        }
        // The head's lower sphere is the URDF's: 47 mm, at its link's origin.
        let head = links.link_entity_by_name("Head_lower").unwrap();
        let sphere = spheres.iter().find(|sphere| sphere.link == head).unwrap();
        assert_eq!(sphere.radius_m, 0.047);
        assert_eq!(sphere.center_local_m, Vec3::ZERO);
        // A thigh's corners span the 0.213 m box below the hip pitch joint.
        let thigh = links.link_entity_by_name("FL_thigh").unwrap();
        let heights: Vec<f64> = spheres
            .iter()
            .filter(|sphere| sphere.link == thigh)
            .map(|sphere| sphere.center_local_m.z)
            .collect();
        let (low, high) = heights.iter().fold((f64::MAX, f64::MIN), |(low, high), z| {
            (low.min(*z), high.max(*z))
        });
        assert!(
            (low + 0.213).abs() < 1e-12 && high.abs() < 1e-12,
            "{low} {high}"
        );
    }
}
