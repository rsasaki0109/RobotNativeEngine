//! Contact points and Coulomb friction handling.

use rne_ecs::Entity;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// An active point contact on a link.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactPoint {
    /// Link entity the contact belongs to.
    pub link: Entity,
    /// Contact position in the link frame, in meters.
    pub point_local_m: Vec3,
    /// Coulomb friction coefficient.
    pub friction_coefficient: f64,
    /// Outward contact normal in world coordinates; normalized internally.
    pub normal_world: Vec3,
}

impl ContactPoint {
    /// Creates a contact point with a world-up normal.
    pub fn new(link: Entity, point_local_m: Vec3, friction_coefficient: f64) -> Self {
        Self {
            link,
            point_local_m,
            friction_coefficient,
            normal_world: Vec3::Y,
        }
    }

    /// Creates a contact point with an explicit outward normal, for worlds or
    /// models that are not Y-up (for example a Z-up floating-base model with
    /// gravity along -Z).
    pub fn with_normal(
        link: Entity,
        point_local_m: Vec3,
        friction_coefficient: f64,
        normal_world: Vec3,
    ) -> Self {
        Self {
            link,
            point_local_m,
            friction_coefficient,
            normal_world,
        }
    }

    /// Returns whether the contact geometry and friction are valid.
    pub fn is_valid(&self) -> bool {
        self.point_local_m.is_finite()
            && self.friction_coefficient.is_finite()
            && self.friction_coefficient >= 0.0
            && self.normal_world.is_finite()
            && self.normal_world.length_squared() > 1.0e-9
    }
}

/// Linearized Coulomb friction cone for a point contact.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct FrictionCone {
    /// Unit outward normal in world coordinates.
    pub normal: Vec3,
    /// Coulomb friction coefficient.
    pub coefficient: f64,
}

impl FrictionCone {
    /// Builds a cone from a contact point, normalizing the normal.
    pub fn from_contact(contact: &ContactPoint) -> Self {
        Self {
            normal: contact.normal_world.normalize_or_zero(),
            coefficient: contact.friction_coefficient,
        }
    }

    /// Edges of the four-sided friction pyramid inscribed in the cone: unit
    /// normal plus `coefficient` along each of two orthogonal tangents. Any
    /// non-negative combination of them lies inside the cone.
    pub fn pyramid_edges(&self) -> [Vec3; 4] {
        let n = self.normal;
        let helper = if n.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
        let t1 = n.cross(helper).normalize_or_zero();
        let t2 = n.cross(t1);
        let mu = self.coefficient;
        [n + t1 * mu, n - t1 * mu, n + t2 * mu, n - t2 * mu]
    }

    /// Projects a world-frame force onto the friction cone.
    ///
    /// The normal component is clamped to be non-negative and the tangential
    /// component is limited to `coefficient * normal`.
    pub fn project(&self, force_world_n: Vec3) -> Vec3 {
        if self.normal.length_squared() <= 1.0e-9 {
            return force_world_n;
        }
        let normal_force = force_world_n.dot(self.normal).max(0.0);
        let tangential = force_world_n - self.normal * force_world_n.dot(self.normal);
        let max_tangential = self.coefficient * normal_force;
        let tangential_norm = tangential.length();
        let tangential = if tangential_norm > max_tangential && tangential_norm > 0.0 {
            tangential * (max_tangential / tangential_norm)
        } else {
            tangential
        };
        self.normal * normal_force + tangential
    }

    /// Returns whether a force already lies inside the cone.
    pub fn contains(&self, force_world_n: Vec3) -> bool {
        if self.normal.length_squared() <= 1.0e-9 {
            return false;
        }
        let normal_force = force_world_n.dot(self.normal);
        if normal_force < 0.0 {
            return false;
        }
        let tangential = force_world_n - self.normal * normal_force;
        tangential.length() <= self.coefficient * normal_force + 1.0e-9
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cone() -> FrictionCone {
        FrictionCone {
            normal: Vec3::Y,
            coefficient: 0.5,
        }
    }

    #[test]
    fn projection_keeps_forces_inside_the_cone() {
        let cone = cone();
        let projected = cone.project(Vec3::new(10.0, 5.0, 0.0));
        assert!(cone.contains(projected));
        // tangential limited to mu * normal = 2.5
        assert!((projected.x - 2.5).abs() < 1.0e-12);
        assert!((projected.y - 5.0).abs() < 1.0e-12);
    }

    #[test]
    fn projection_removes_negative_normal_force() {
        let cone = cone();
        let projected = cone.project(Vec3::new(1.0, -2.0, 0.0));
        assert!(projected.y >= 0.0);
        assert!(cone.contains(projected));
    }
}
