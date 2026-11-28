//! Static workspace geometry.

use glam::DVec3;
use serde::{Deserialize, Serialize};

/// A static sphere.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Sphere {
    /// Centre (m).
    pub center_m: DVec3,
    /// Radius (m).
    pub radius_m: f64,
}

/// A static axis-aligned box.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Aabb {
    /// Minimum corner (m).
    pub min_m: DVec3,
    /// Maximum corner (m).
    pub max_m: DVec3,
}

impl Aabb {
    /// Distance (m) from `p` to the box (0 inside).
    #[must_use]
    pub fn distance(&self, p: DVec3) -> f64 {
        (self.min_m - p)
            .max(p - self.max_m)
            .max(DVec3::ZERO)
            .length()
    }
}

/// Static obstacles around an arm. Self-collision is not modelled.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ManipScene {
    /// Spheres.
    pub spheres: Vec<Sphere>,
    /// Boxes (tables, shelves, walls).
    pub boxes: Vec<Aabb>,
}

impl ManipScene {
    /// Smallest surface distance (m) from a sphere at `p` with `radius_m`
    /// to any obstacle (infinite with none; negative on overlap).
    #[must_use]
    pub fn clearance(&self, p: DVec3, radius_m: f64) -> f64 {
        let s = self
            .spheres
            .iter()
            .map(|s| s.center_m.distance(p) - s.radius_m - radius_m);
        let b = self.boxes.iter().map(|b| b.distance(p) - radius_m);
        s.chain(b).fold(f64::INFINITY, f64::min)
    }
}
