//! Hard-contact quadruped simulation over a height-field terrain.

use crate::quadruped::{terrain_surface, BaseState, LocomotionError, Quadruped};
use rne_dynamics::{
    contact_frame, contact_step_with_kinematics, ContactOutcome, ContactPoint, ContactStepConfig,
    JointPdControl,
};
use rne_ecs::Entity;
use rne_math::Vec3;
use rne_physics::ColliderShape;
use rne_robot::ForwardKinematics;
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
    body_spheres: Vec<BodySphere>,
    /// Kinematic link index of each body sphere.
    body_sphere_links: Vec<usize>,
    /// Bounds of the terrain: see [`TerrainBounds`].
    terrain_bounds: TerrainBounds,
}

/// The highest point of a height field, in the locomotion frame's z, and the
/// least vertical component of its surface normal. A point `d` above the
/// highest point lies at least `d` times that component from the surface's
/// tangent plane anywhere below it, so a body sphere that far above the
/// terrain cannot touch it this step and needs no terrain query.
#[derive(Clone, Copy, Debug, PartialEq)]
struct TerrainBounds {
    top_m: f64,
    min_normal_z: f64,
}

impl TerrainBounds {
    /// Bounds that never skip a query, for a terrain that is not a valid
    /// height field (whose queries then fail as before).
    const NONE: Self = Self {
        top_m: f64::INFINITY,
        min_normal_z: 0.0,
    };

    fn of(terrain: &ColliderShape) -> Self {
        let ColliderShape::HeightField {
            nrows,
            ncols,
            heights_m,
            scale,
        } = terrain
        else {
            return Self::NONE;
        };
        let (rows, columns) = (*nrows as usize, *ncols as usize);
        if rows < 2 || columns < 2 || heights_m.len() != rows * columns {
            return Self::NONE;
        }
        let at = |row: usize, column: usize| heights_m[row * columns + column] * scale.y;
        let top_m = (0..rows * columns)
            .map(|index| heights_m[index] * scale.y)
            .fold(f64::NEG_INFINITY, f64::max);
        // The surface's slopes blend its cell edges' slopes, so the steepest
        // edges bound them.
        let (dx, dz) = (scale.x / (columns - 1) as f64, scale.z / (rows - 1) as f64);
        let mut slope_x: f64 = 0.0;
        let mut slope_z: f64 = 0.0;
        for row in 0..rows {
            for column in 0..columns {
                if column + 1 < columns {
                    slope_x = slope_x.max(((at(row, column + 1) - at(row, column)) / dx).abs());
                }
                if row + 1 < rows {
                    slope_z = slope_z.max(((at(row + 1, column) - at(row, column)) / dz).abs());
                }
            }
        }
        let min_normal_z = 1.0 / (1.0 + slope_x * slope_x + slope_z * slope_z).sqrt();
        if top_m.is_finite() && min_normal_z.is_finite() {
            Self {
                top_m,
                min_normal_z,
            }
        } else {
            Self::NONE
        }
    }

    /// Whether a sphere centered at height `z_m` lies further than `margin_m`
    /// from the terrain everywhere.
    fn clears(&self, z_m: f64, radius_m: f64, margin_m: f64) -> bool {
        let above = z_m - self.top_m;
        above > 0.0 && above * self.min_normal_z - radius_m > margin_m
    }
}

/// A collision sphere fixed to a link of the robot, besides the feet: a knee,
/// a shin or a body corner that can touch the terrain. A zero radius makes it
/// a point, such as a box's corner.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodySphere {
    /// Kinematic link the sphere moves with.
    pub link: Entity,
    /// Sphere center in the link frame, in meters.
    pub center_local_m: Vec3,
    /// Sphere radius, in meters.
    pub radius_m: f64,
}

/// A body sphere joins a step's contacts when it is this close to the terrain.
const BODY_CONTACT_MARGIN_M: f64 = 0.01;

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
            body_spheres: Vec::new(),
            body_sphere_links: Vec::new(),
            terrain_bounds: TerrainBounds::NONE,
        };
        sim.terrain_bounds = TerrainBounds::of(&sim.terrain);
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

    /// Lets `spheres` touch the terrain besides the feet. Each step adds a
    /// contact for every sphere within 1 cm of the terrain, after the feet,
    /// so a robot that kneels or falls lands on its knees or body instead of
    /// sinking through the ground. None by default. A sphere higher above the
    /// terrain's highest point than the terrain's steepest slope lets it come
    /// within 1 cm costs no terrain query, so spheres on the body and the
    /// upper legs cost little while they are clear of the ground.
    pub fn set_body_spheres(&mut self, spheres: Vec<BodySphere>) -> Result<(), LocomotionError> {
        let links = self.quadruped.model().kinematic();
        if spheres.iter().any(|sphere| {
            links.link_index(sphere.link).is_none()
                || !sphere.center_local_m.is_finite()
                || !(sphere.radius_m.is_finite() && sphere.radius_m >= 0.0)
        }) {
            return Err(LocomotionError::InvalidInput("body sphere"));
        }
        self.body_sphere_links = spheres
            .iter()
            .filter_map(|sphere| links.link_index(sphere.link))
            .collect();
        self.body_spheres = spheres;
        Ok(())
    }

    /// Spheres besides the feet that can touch the terrain.
    pub fn body_spheres(&self) -> &[BodySphere] {
        &self.body_spheres
    }

    /// Puts the robot in configuration `q` (`(x, y, z, roll, pitch, yaw,
    /// joints…)` in the locomotion frame) with generalized velocity `qd` (base
    /// body twist, then joint rates), clears the contact warm start, and resets
    /// the clock. Use it to start from a pose other than the stand pose, such
    /// as a policy's default pose held above the ground.
    pub fn set_state(&mut self, q: &[f64], qd: &[f64]) -> Result<(), LocomotionError> {
        let nv = self.quadruped.model().nv();
        if q.len() != nv || qd.len() != nv {
            return Err(LocomotionError::InvalidInput(
                "one value per generalized coordinate",
            ));
        }
        if q.iter().chain(qd).any(|value| !value.is_finite()) {
            return Err(LocomotionError::InvalidInput("finite state"));
        }
        self.q = q.to_vec();
        self.qd = qd.to_vec();
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
        // One forward-kinematics pass places the contacts and serves the step.
        let kinematics = model.kinematic().forward_kinematics(&self.q)?;
        let mut contacts = self.quadruped.foot_contacts_from(
            &kinematics,
            &self.terrain,
            self.config.friction_coefficient,
        )?;
        let feet = contacts.len();
        self.append_body_contacts(&kinematics, &mut contacts)?;
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
            effort_limits: Vec::new(),
        };
        let config = ContactStepConfig {
            step_time_s: self.config.step_time_s,
            ..self.config.contact
        };
        // The feet warm-start from the previous step; body contacts come and
        // go, so they start cold.
        let warm = self.warm.as_ref().map(|feet_warm| {
            let mut warm = feet_warm.clone();
            warm.resize(contacts.len(), [0.0; 3]);
            warm
        });
        let step = contact_step_with_kinematics(
            model,
            &kinematics,
            &self.q,
            &self.qd,
            &tau,
            &contacts,
            Some(&pd),
            &config,
            warm.as_deref(),
        )?;
        self.warm = Some(
            step.contacts
                .iter()
                .zip(&contacts)
                .take(feet)
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

    /// Appends a contact for each body sphere near the terrain.
    fn append_body_contacts(
        &self,
        kinematics: &ForwardKinematics,
        contacts: &mut Vec<ContactPoint>,
    ) -> Result<(), LocomotionError> {
        for (sphere, &index) in self.body_spheres.iter().zip(&self.body_sphere_links) {
            let frame = kinematics.transforms()[index];
            let center = frame.translation + frame.rotation * sphere.center_local_m;
            if self
                .terrain_bounds
                .clears(center.z, sphere.radius_m, BODY_CONTACT_MARGIN_M)
            {
                continue;
            }
            let (height, normal) = terrain_surface(&self.terrain, center.x, center.y)?;
            let gap = (center.z - height) * normal.z - sphere.radius_m;
            if gap > BODY_CONTACT_MARGIN_M {
                continue;
            }
            let point = center - normal * sphere.radius_m;
            contacts.push(ContactPoint {
                link: sphere.link,
                point_local_m: frame.rotation.conjugate() * (point - frame.translation),
                normal_world: normal,
                gap_m: gap,
                friction_coefficient: self.config.friction_coefficient,
            });
        }
        Ok(())
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
    fn body_spheres_hold_a_limp_robot_off_the_floor() {
        let go2 = Arc::new(Quadruped::go2().expect("go2"));
        // No PD: the legs give way and the body drops.
        let limp = TerrainSimConfig {
            position_gain_nm_rad: 0.0,
            velocity_gain_nm_s_rad: 0.0,
            ..TerrainSimConfig::default()
        };
        let base = go2
            .model()
            .kinematic()
            .link_entity_by_name("base")
            .expect("base link");
        let belly = BodySphere {
            link: base,
            center_local_m: Vec3::ZERO,
            radius_m: 0.06,
        };
        let command = JointCommand::positions(go2.stand_pose_rad().to_vec());
        let lowest = |spheres: Vec<BodySphere>| {
            let mut sim = QuadrupedTerrainSim::new(go2.clone(), flat_terrain(), limp).expect("sim");
            sim.set_body_spheres(spheres).expect("spheres");
            let mut lowest = f64::INFINITY;
            let mut body_contacts = 0;
            for _ in 0..1000 {
                let step = sim.step(&command, Vec3::ZERO).expect("step");
                body_contacts = body_contacts.max(step.contacts.len() - 4);
                lowest = lowest.min(sim.base_clearance_m().expect("clearance"));
            }
            (lowest, body_contacts)
        };
        let (sunk, none) = lowest(Vec::new());
        assert!(sunk < 0.0, "without spheres the body sinks: {sunk}");
        assert_eq!(none, 0);
        let (held, touching) = lowest(vec![belly]);
        assert!(held > 0.05, "the belly sphere holds the body up: {held}");
        assert_eq!(touching, 1);

        let mut sim = QuadrupedTerrainSim::new(go2.clone(), flat_terrain(), limp).expect("sim");
        for bad in [
            BodySphere {
                radius_m: -0.01,
                ..belly
            },
            BodySphere {
                center_local_m: Vec3::new(f64::NAN, 0.0, 0.0),
                ..belly
            },
            BodySphere {
                link: Entity::PLACEHOLDER,
                ..belly
            },
        ] {
            assert!(sim.set_body_spheres(vec![bad]).is_err());
        }
        assert!(sim.body_spheres().is_empty());
    }

    #[test]
    fn terrain_bounds_bound_every_surface_query() {
        let terrain = FractalTerrain {
            size_x_m: 6.0,
            size_z_m: 6.0,
            columns: 61,
            rows: 61,
            amplitude_m: 0.08,
            ..FractalTerrain::default()
        }
        .height_field(3)
        .expect("terrain");
        let bounds = TerrainBounds::of(&terrain);
        assert!(bounds.top_m > 0.0 && bounds.top_m < 0.2, "{bounds:?}");
        assert!(
            bounds.min_normal_z > 0.5 && bounds.min_normal_z < 1.0,
            "{bounds:?}"
        );
        let (mut highest, mut flattest) = (f64::NEG_INFINITY, f64::INFINITY);
        for i in 0..200 {
            for j in 0..200 {
                let (x, y) = (-2.99 + 0.03 * i as f64, -2.99 + 0.03 * j as f64);
                let (height, normal) = terrain_surface(&terrain, x, y).expect("on the terrain");
                highest = highest.max(height);
                flattest = flattest.min(normal.z);
            }
        }
        assert!(highest <= bounds.top_m && bounds.top_m - highest < 0.01);
        assert!(flattest >= bounds.min_normal_z);
        let flat = TerrainBounds::of(&flat_terrain());
        assert_eq!((flat.top_m, flat.min_normal_z), (0.0, 1.0));
        assert!(flat.clears(0.031, 0.02, 0.01) && !flat.clears(0.029, 0.02, 0.01));
        let none = TerrainBounds::of(&ColliderShape::Sphere { radius_m: 1.0 });
        assert!(!none.clears(1.0e9, 0.0, 0.0));
    }

    #[test]
    fn skipping_spheres_above_the_terrain_changes_no_step() {
        let go2 = Arc::new(Quadruped::go2().expect("go2"));
        let terrain = FractalTerrain {
            size_x_m: 4.0,
            size_z_m: 4.0,
            columns: 41,
            rows: 41,
            amplitude_m: 0.05,
            ..FractalTerrain::default()
        }
        .height_field(5)
        .expect("terrain");
        let limp = TerrainSimConfig {
            position_gain_nm_rad: 0.0,
            velocity_gain_nm_s_rad: 0.0,
            ..TerrainSimConfig::default()
        };
        let links = go2.model().kinematic();
        let link = |name: &str| links.link_entity_by_name(name).expect("link");
        // A belly, corner points and knees, so the collapse lands on some.
        let mut spheres = vec![BodySphere {
            link: link("base"),
            center_local_m: Vec3::ZERO,
            radius_m: 0.05,
        }];
        for (x, y) in [(0.19, 0.05), (0.19, -0.05), (-0.19, 0.05), (-0.19, -0.05)] {
            spheres.push(BodySphere {
                link: link("base"),
                center_local_m: Vec3::new(x, y, -0.057),
                radius_m: 0.0,
            });
        }
        for leg in ["FL", "FR", "RL", "RR"] {
            spheres.push(BodySphere {
                link: link(&format!("{leg}_calf")),
                center_local_m: Vec3::ZERO,
                radius_m: 0.017,
            });
        }
        let command = JointCommand::positions(go2.stand_pose_rad().to_vec());
        let run = |bounds: Option<TerrainBounds>| {
            let mut sim =
                QuadrupedTerrainSim::new(go2.clone(), terrain.clone(), limp).expect("sim");
            sim.set_body_spheres(spheres.clone()).expect("spheres");
            if let Some(bounds) = bounds {
                sim.terrain_bounds = bounds;
            }
            let mut body_contacts = 0;
            for _ in 0..600 {
                let step = sim.step(&command, Vec3::ZERO).expect("step");
                body_contacts += step.contacts.len() - 4;
            }
            (sim.q().to_vec(), sim.qd().to_vec(), body_contacts)
        };
        let skipping = run(None);
        let querying = run(Some(TerrainBounds::NONE));
        assert!(skipping.2 > 0, "the body lands on its spheres");
        assert_eq!(skipping, querying);
    }

    #[test]
    fn set_state_places_the_robot_and_rejects_bad_states() {
        let go2 = Arc::new(Quadruped::go2().expect("go2"));
        let mut sim =
            QuadrupedTerrainSim::new(go2.clone(), flat_terrain(), TerrainSimConfig::default())
                .expect("sim");
        let targets = JointCommand::positions(go2.stand_pose_rad().to_vec());
        sim.step(&targets, Vec3::ZERO).expect("step");
        let nv = go2.model().nv();
        let mut q = vec![0.0; nv];
        q[0] = 0.4;
        q[2] = 0.36;
        q[5] = 0.3;
        for (slot, angle) in q[6..].iter_mut().zip(go2.stand_pose_rad()) {
            *slot = angle + 0.1;
        }
        let mut qd = vec![0.0; nv];
        qd[0] = 0.2;
        sim.set_state(&q, &qd).expect("set state");
        assert_eq!(sim.q(), q.as_slice());
        assert_eq!(sim.qd(), qd.as_slice());
        assert_eq!(sim.time_s(), 0.0);
        let state = sim.base_state();
        assert!((state.position_m - Vec3::new(0.4, 0.0, 0.36)).length() < 1e-12);
        assert!((state.heading_rad() - 0.3).abs() < 1e-12);
        // Released above the ground, it falls and lands on its feet.
        for _ in 0..500 {
            sim.step(&targets, Vec3::ZERO).expect("step");
        }
        assert!(sim.base_clearance_m().expect("clearance") > 0.25);
        assert!(sim.set_state(&q[1..], &qd).is_err());
        assert!(sim.set_state(&q, &qd[1..]).is_err());
        let mut bad = q.clone();
        bad[3] = f64::NAN;
        assert!(sim.set_state(&bad, &qd).is_err());
        // A rejected state leaves the last one in place.
        assert_eq!(sim.q().len(), nv);
        assert!(sim.q().iter().all(|value| value.is_finite()));
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
