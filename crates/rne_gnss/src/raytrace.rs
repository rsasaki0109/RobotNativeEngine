//! Ray-traced GNSS signal propagation: line-of-sight occlusion and
//! single-bounce specular reflections.
//!
//! At 20 000 km a satellite's signal arrives as a plane wave, so each
//! satellite is a direction `u` (receiver toward satellite, ENU). The direct
//! path is a ray from the antenna along `u`; it is line-of-sight (LOS) when no
//! box blocks it. A reflection off a facet with outward normal `n` arrives
//! from the mirrored direction `u' = u - 2(u·n)n`: the ray from the antenna
//! along `u'` must hit the facet within its bounds unobstructed, and the
//! reflection point must itself see the satellite along `u`. For a plane wave
//! the reflected path is longer than the direct one by `d (1 - u·u')`, where
//! `d` is the antenna-to-reflection-point distance. A satellite whose direct
//! path is blocked but that has a reflected path is received non-line-of-sight
//! (NLOS) — the dominant urban GNSS error. Diffraction and multi-bounce paths
//! are not modelled.

use crate::scene::{FacetSource, GnssScene};
use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// Small offset that lifts secondary rays off the surface they start on.
const SURFACE_EPSILON_M: f64 = 1.0e-6;

/// A single-bounce reflected path.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ReflectedPath {
    /// The reflecting surface.
    pub source: FacetSource,
    /// Reflection point in ENU meters.
    pub point_m: Vec3,
    /// Direction from the antenna toward the reflection point (unit, ENU).
    pub arrival_direction: Vec3,
    /// Extra path length relative to the direct path in meters.
    pub excess_delay_m: f64,
    /// Reflection loss in dB.
    pub reflection_loss_db: f64,
}

/// How a satellite signal reaches the antenna.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Propagation {
    /// Whether the direct path is unobstructed.
    pub line_of_sight: bool,
    /// The box blocking the direct path, if any.
    pub blocked_by: Option<usize>,
    /// Reflected paths, in facet order.
    pub reflections: Vec<ReflectedPath>,
}

impl Propagation {
    /// The reflection with the least loss (ties: shortest delay).
    pub fn strongest_reflection(&self) -> Option<&ReflectedPath> {
        self.reflections.iter().min_by(|a, b| {
            a.reflection_loss_db
                .total_cmp(&b.reflection_loss_db)
                .then(a.excess_delay_m.total_cmp(&b.excess_delay_m))
        })
    }
}

/// Traces direct and single-bounce paths from an antenna toward a satellite.
///
/// `satellite_direction` is the unit ENU direction from the antenna to the
/// satellite. Satellites below the geometric horizon get no paths.
pub fn trace_satellite(
    scene: &GnssScene,
    antenna_m: Vec3,
    satellite_direction: Vec3,
) -> Propagation {
    let u = satellite_direction.normalize();
    let blocked_by = scene
        .raycast(antenna_m, u, f64::INFINITY)
        .and_then(|hit| match hit.source {
            FacetSource::Box(index) => Some(index),
            FacetSource::Ground => None,
        });
    let mut propagation = Propagation {
        line_of_sight: blocked_by.is_none(),
        blocked_by,
        reflections: Vec::new(),
    };
    if u.z <= 0.0 || scene.inside_solid(antenna_m) {
        propagation.line_of_sight = false;
        return propagation;
    }

    for facet in scene.facets() {
        let n = facet.normal;
        let incidence = u.dot(n);
        // The satellite and the antenna must both be in front of the facet.
        if incidence <= 0.0 || (antenna_m - facet.point_m).dot(n) <= 0.0 {
            continue;
        }
        let mirrored = u - n * (2.0 * incidence);
        let toward = mirrored.dot(n);
        if toward >= 0.0 {
            continue;
        }
        let distance_m = (facet.point_m - antenna_m).dot(n) / toward;
        if !distance_m.is_finite() || distance_m <= 0.0 {
            continue;
        }
        let point_m = antenna_m + mirrored * distance_m;
        if !facet.contains_in_plane(point_m) {
            continue;
        }
        // Antenna to reflection point must be clear (other boxes only: the
        // facet's own box is entered exactly at `distance_m`).
        if scene.occluded(antenna_m, mirrored, distance_m - 1.0e-6) {
            continue;
        }
        // The reflection point must see the satellite.
        if scene.occluded(point_m + n * SURFACE_EPSILON_M, u, f64::INFINITY) {
            continue;
        }
        propagation.reflections.push(ReflectedPath {
            source: facet.source,
            point_m,
            arrival_direction: mirrored,
            excess_delay_m: distance_m * (1.0 - u.dot(mirrored)),
            reflection_loss_db: facet.material.reflection_loss_db,
        });
    }
    propagation
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::{SceneBox, SurfaceMaterial};
    use approx::assert_relative_eq;

    fn direction(azimuth_deg: f64, elevation_deg: f64) -> Vec3 {
        let (az, el) = (azimuth_deg.to_radians(), elevation_deg.to_radians());
        Vec3::new(az.sin() * el.cos(), az.cos() * el.cos(), el.sin())
    }

    #[test]
    fn open_sky_is_los_with_ground_multipath_of_two_h_sin_el() {
        let scene = GnssScene::open_sky();
        let height = 1.5;
        let elevation = 30.0_f64;
        let p = trace_satellite(
            &scene,
            Vec3::new(0.0, 0.0, height),
            direction(45.0, elevation),
        );
        assert!(p.line_of_sight);
        assert_eq!(p.reflections.len(), 1);
        let ground = p.reflections[0];
        assert_eq!(ground.source, FacetSource::Ground);
        assert_relative_eq!(
            ground.excess_delay_m,
            2.0 * height * elevation.to_radians().sin(),
            epsilon = 1e-9
        );
    }

    #[test]
    fn wall_blocks_a_low_satellite_behind_it() {
        let mut scene = GnssScene::default();
        // Wall 10 m east, 30 m tall: masks eastern satellites below ~71°.
        scene.add_box(SceneBox::building(
            10.0,
            -50.0,
            12.0,
            50.0,
            30.0,
            SurfaceMaterial::CONCRETE,
        ));
        let antenna = Vec3::new(0.0, 0.0, 1.0);
        let low_east = trace_satellite(&scene, antenna, direction(90.0, 40.0));
        assert!(!low_east.line_of_sight);
        assert_eq!(low_east.blocked_by, Some(0));
        let high_east = trace_satellite(&scene, antenna, direction(90.0, 80.0));
        assert!(high_east.line_of_sight);
    }

    #[test]
    fn opposite_wall_creates_an_nlos_path_with_image_method_delay() {
        // A street between a 30 m concrete wall 10 m east and a 20 m glass
        // wall 10 m west.
        let mut scene = GnssScene::default();
        scene.add_box(SceneBox::building(
            10.0,
            -50.0,
            12.0,
            50.0,
            30.0,
            SurfaceMaterial::CONCRETE,
        ));
        scene.add_box(SceneBox::building(
            -12.0,
            -50.0,
            -10.0,
            50.0,
            20.0,
            SurfaceMaterial::GLASS,
        ));
        let antenna = Vec3::new(0.0, 0.0, 1.0);
        let elevation = 40.0_f64.to_radians();
        let p = trace_satellite(&scene, antenna, direction(270.0, 40.0));
        // The western satellite is masked by the 20 m glass wall, but its
        // signal bounces off the east wall's face and clears the glass wall
        // on the way in: a pure NLOS reception.
        assert!(!p.line_of_sight);
        let east_wall = p
            .reflections
            .iter()
            .find(|r| r.source == FacetSource::Box(0))
            .expect("reflection off the east wall");
        // Image method: excess = 2 * distance_to_wall * cos(elevation).
        assert_relative_eq!(
            east_wall.excess_delay_m,
            2.0 * 10.0 * elevation.cos(),
            epsilon = 1e-9
        );
        assert_relative_eq!(east_wall.point_m.x, 10.0, epsilon = 1e-9);
    }

    #[test]
    fn wide_roof_blocks_everything() {
        let mut scene = GnssScene::open_sky();
        scene.add_box(SceneBox::new(
            Vec3::new(-20.0, -20.0, 3.0),
            Vec3::new(20.0, 20.0, 3.3),
            SurfaceMaterial::CONCRETE,
        ));
        for el in [20.0, 50.0, 89.0] {
            let p = trace_satellite(&scene, Vec3::new(0.0, 0.0, 1.0), direction(10.0, el));
            assert!(!p.line_of_sight);
            assert!(p.reflections.is_empty(), "elevation {el}");
        }
    }
}
