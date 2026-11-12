//! The Go2 on `rne_locomotion`'s terrain simulation: `rne_dynamics`' hard
//! contact step on a flat height field, behind the same [`Robot`] interface as
//! the Rapier walker.
//!
//! The plant is `rne_locomotion::go2_learning::Go2LearningRobot`: the URDF's
//! articulated Go2 with its fixed links fused and the declared masses
//! (16.087 kg), as on the Rapier and MuJoCo plants. Every URDF collision
//! shape touches the ground, as in MuJoCo and on Rapier: the feet are
//! `rne_locomotion`'s foot spheres, and `collision_spheres` turns the head,
//! body, hips, thighs, calves and lower-calf bumpers into spheres and points.
//! MuJoCo's walks lean on the head's lower sphere at every stride; with
//! contacts at the feet and lower legs only, they fell through the floor
//! within a second.
//!
//! The joints track their targets through an explicit PD torque computed at
//! the start of every physics step and clamped to the effort limits, as
//! Rapier's walker and MuJoCo's position actuators do, rather than through
//! `rne_locomotion`'s implicit PD: with the implicit law the same ES finds
//! only a 0.4 m shuffle in 200 generations.

use rne_locomotion::go2_learning::{Floor, Go2LearningRobot};
use rne_math::{Quat, Vec3};

use crate::go2::Robot;
use crate::Result;

/// A Go2 on the native terrain step, restarted in its default pose by
/// [`Robot::restart`].
pub(crate) struct LocomotionGo2(Go2LearningRobot);

impl LocomotionGo2 {
    /// A Go2 on a flat floor, or on the rough terrain of `terrain_seed` with
    /// `relief`.
    pub(crate) fn new(
        floor_friction: f64,
        terrain_seed: Option<u64>,
        relief: crate::go2::Relief,
    ) -> Result<Self> {
        let floor = match terrain_seed {
            Some(seed) => Floor::Terrain { seed, relief },
            None => Floor::Flat,
        };
        Ok(Self(Go2LearningRobot::new(floor_friction, floor)?))
    }

    /// The robot's total mass, in kilograms.
    pub(crate) fn mass_kg(&self) -> f64 {
        self.0.mass_kg()
    }
}

impl Robot for LocomotionGo2 {
    fn restart(&mut self) -> Result<()> {
        self.restart_at([0.0, 0.0])
    }

    fn restart_at(&mut self, start: [f64; 2]) -> Result<()> {
        Ok(self.0.restart_at(start)?)
    }

    fn set_targets_rad(&mut self, targets_rad: &[f64; 12]) -> Result<()> {
        self.0.set_targets_rad(targets_rad);
        Ok(())
    }

    fn step(&mut self) -> Result<()> {
        Ok(self.0.step()?)
    }

    fn contact_load(&self) -> Option<Vec<(String, f64, usize)>> {
        Some(self.0.contact_load())
    }

    fn base_pose_z_up(&self) -> (Vec3, Quat) {
        self.0.base_pose_z_up()
    }

    fn base_velocity_z_up(&self) -> (Vec3, Vec3) {
        self.0.base_velocity_z_up()
    }

    fn ground_height_m(&self, x_m: f64, y_m: f64) -> f64 {
        self.0.ground_height_m(x_m, y_m)
    }

    fn arena_half_m(&self) -> f64 {
        self.0.arena_half_m()
    }

    fn joint_states(&self) -> Result<[(f64, f64); 12]> {
        Ok(self.0.joint_states())
    }
}

#[cfg(test)]
mod tests {
    use rne_locomotion::go2_learning::collision_spheres;
    use rne_locomotion::Quadruped;
    use rne_math::Vec3;

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
