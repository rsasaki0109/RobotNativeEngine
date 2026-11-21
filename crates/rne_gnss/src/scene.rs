//! Ray-traceable scene geometry for GNSS propagation.
//!
//! Buildings and walls are axis-aligned boxes in the local ENU frame, each
//! with a [`SurfaceMaterial`] that sets how much a specular reflection off it
//! attenuates a GNSS signal. An optional horizontal ground plane adds ground
//! multipath. Boxes are tested by the slab method in index order, so results
//! are deterministic.

use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// Electromagnetic surface properties at L1.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SurfaceMaterial {
    /// Power lost on a single specular reflection, including the right- to
    /// left-hand circular polarisation flip seen by an RHCP antenna, in dB.
    pub reflection_loss_db: f64,
}

impl SurfaceMaterial {
    /// Reinforced concrete.
    pub const CONCRETE: Self = Self {
        reflection_loss_db: 8.0,
    };
    /// Coated glass curtain wall; a strong reflector.
    pub const GLASS: Self = Self {
        reflection_loss_db: 4.0,
    };
    /// Metal cladding; nearly lossless.
    pub const METAL: Self = Self {
        reflection_loss_db: 2.0,
    };
    /// Asphalt or soil ground.
    pub const GROUND: Self = Self {
        reflection_loss_db: 12.0,
    };
    /// Small or rough objects (poles, trunks, benches): they block signals
    /// but are not traced as specular reflectors.
    pub const ABSORBER: Self = Self {
        reflection_loss_db: f64::INFINITY,
    };

    /// Whether specular reflections off this material are traced.
    pub fn is_reflective(&self) -> bool {
        self.reflection_loss_db.is_finite()
    }
}

/// A volume of foliage (a tree canopy) that attenuates signals passing
/// through it without blocking them.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct FoliageVolume {
    /// Minimum corner in ENU meters.
    pub min_m: Vec3,
    /// Maximum corner in ENU meters.
    pub max_m: Vec3,
    /// Attenuation per meter of path inside the volume at L1, in dB/m
    /// (deciduous canopies: about 0.5-1.5 dB/m).
    pub loss_db_per_m: f64,
}

impl FoliageVolume {
    /// Length of the segment `origin + t·direction`, `t ∈ [0, max_t]`, inside
    /// the volume.
    pub fn path_length_m(&self, origin_m: Vec3, direction: Vec3, max_t: f64) -> f64 {
        let (mut t0, mut t1) = (0.0_f64, max_t);
        for axis in 0..3 {
            let (o, d) = (origin_m[axis], direction[axis]);
            let (lo, hi) = (self.min_m[axis], self.max_m[axis]);
            if d.abs() < 1.0e-15 {
                if o < lo || o > hi {
                    return 0.0;
                }
                continue;
            }
            let (mut a, mut b) = ((lo - o) / d, (hi - o) / d);
            if a > b {
                std::mem::swap(&mut a, &mut b);
            }
            t0 = t0.max(a);
            t1 = t1.min(b);
            if t0 >= t1 {
                return 0.0;
            }
        }
        (t1 - t0).max(0.0) * direction.length()
    }
}

/// An axis-aligned solid box in the ENU frame.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SceneBox {
    /// Minimum corner in ENU meters.
    pub min_m: Vec3,
    /// Maximum corner in ENU meters.
    pub max_m: Vec3,
    /// Surface material of every face.
    pub material: SurfaceMaterial,
}

impl SceneBox {
    /// Creates a box from two corners in any order.
    pub fn new(a_m: Vec3, b_m: Vec3, material: SurfaceMaterial) -> Self {
        Self {
            min_m: a_m.min(b_m),
            max_m: a_m.max(b_m),
            material,
        }
    }

    /// A box standing on `z = 0` with the given footprint and height.
    pub fn building(
        x0_m: f64,
        y0_m: f64,
        x1_m: f64,
        y1_m: f64,
        height_m: f64,
        material: SurfaceMaterial,
    ) -> Self {
        Self::new(
            Vec3::new(x0_m, y0_m, 0.0),
            Vec3::new(x1_m, y1_m, height_m),
            material,
        )
    }

    /// Whether a point lies inside or on the box.
    pub fn contains(&self, point_m: Vec3) -> bool {
        point_m.cmpge(self.min_m).all() && point_m.cmple(self.max_m).all()
    }

    /// Entry distance and entry-face outward normal of a ray, if it hits.
    ///
    /// A ray starting inside the box hits at distance zero.
    pub fn intersect(
        &self,
        origin_m: Vec3,
        direction: Vec3,
        max_distance_m: f64,
    ) -> Option<(f64, Vec3)> {
        let mut t_enter = f64::NEG_INFINITY;
        let mut t_exit = f64::INFINITY;
        let mut normal = Vec3::ZERO;
        for axis in 0..3 {
            let o = origin_m[axis];
            let d = direction[axis];
            let (lo, hi) = (self.min_m[axis], self.max_m[axis]);
            if d.abs() < 1.0e-15 {
                if o < lo || o > hi {
                    return None;
                }
                continue;
            }
            let (mut t0, mut t1) = ((lo - o) / d, (hi - o) / d);
            let mut face = Vec3::ZERO;
            face[axis] = -d.signum();
            if t0 > t1 {
                std::mem::swap(&mut t0, &mut t1);
            }
            if t0 > t_enter {
                t_enter = t0;
                normal = face;
            }
            t_exit = t_exit.min(t1);
            if t_enter > t_exit {
                return None;
            }
        }
        if t_exit < 0.0 || t_enter > max_distance_m {
            return None;
        }
        if t_enter < 0.0 {
            return Some((0.0, -direction));
        }
        Some((t_enter, normal))
    }

    /// The twelve edges as segment endpoints.
    pub fn edges(&self) -> [(Vec3, Vec3); 12] {
        let (lo, hi) = (self.min_m, self.max_m);
        let corner = |x: bool, y: bool, z: bool| {
            Vec3::new(
                if x { hi.x } else { lo.x },
                if y { hi.y } else { lo.y },
                if z { hi.z } else { lo.z },
            )
        };
        [
            // Along x.
            (corner(false, false, false), corner(true, false, false)),
            (corner(false, true, false), corner(true, true, false)),
            (corner(false, false, true), corner(true, false, true)),
            (corner(false, true, true), corner(true, true, true)),
            // Along y.
            (corner(false, false, false), corner(false, true, false)),
            (corner(true, false, false), corner(true, true, false)),
            (corner(false, false, true), corner(false, true, true)),
            (corner(true, false, true), corner(true, true, true)),
            // Along z.
            (corner(false, false, false), corner(false, false, true)),
            (corner(true, false, false), corner(true, false, true)),
            (corner(false, true, false), corner(false, true, true)),
            (corner(true, true, false), corner(true, true, true)),
        ]
    }

    /// The four vertical faces as reflecting facets.
    pub fn vertical_facets(&self, box_index: usize) -> [Facet; 4] {
        let (lo, hi) = (self.min_m, self.max_m);
        let facet = |normal: Vec3, point: Vec3| Facet {
            normal,
            point_m: point,
            min_m: lo,
            max_m: hi,
            material: self.material,
            source: FacetSource::Box(box_index),
        };
        [
            facet(Vec3::X, Vec3::new(hi.x, lo.y, lo.z)),
            facet(Vec3::NEG_X, lo),
            facet(Vec3::Y, Vec3::new(lo.x, hi.y, lo.z)),
            facet(Vec3::NEG_Y, lo),
        ]
    }
}

/// Where a reflecting facet comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FacetSource {
    /// A face of the box with this index in [`GnssScene::boxes`].
    Box(usize),
    /// The ground plane.
    Ground,
}

/// An axis-aligned planar reflector, bounded by a box's extent.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Facet {
    /// Outward unit normal (an axis direction).
    pub normal: Vec3,
    /// A point on the plane.
    pub point_m: Vec3,
    /// Lower bound of the facet's extent.
    pub min_m: Vec3,
    /// Upper bound of the facet's extent.
    pub max_m: Vec3,
    /// Material.
    pub material: SurfaceMaterial,
    /// Origin of the facet.
    pub source: FacetSource,
}

impl Facet {
    /// Whether an in-plane point lies within the facet's bounds.
    pub fn contains_in_plane(&self, point_m: Vec3) -> bool {
        const EPS: f64 = 1.0e-9;
        (0..3).all(|axis| {
            self.normal[axis] != 0.0
                || (point_m[axis] >= self.min_m[axis] - EPS
                    && point_m[axis] <= self.max_m[axis] + EPS)
        })
    }
}

/// A horizontal ground plane.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct GroundPlane {
    /// Ground height in ENU meters.
    pub height_m: f64,
    /// Ground material.
    pub material: SurfaceMaterial,
}

/// The closest hit of a ray.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
    /// Distance along the ray in meters.
    pub distance_m: f64,
    /// Hit point in ENU meters.
    pub point_m: Vec3,
    /// Outward surface normal at the hit.
    pub normal: Vec3,
    /// What was hit.
    pub source: FacetSource,
}

/// A static GNSS propagation scene in the ENU frame.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct GnssScene {
    /// Solid boxes (buildings, walls, roofs).
    pub boxes: Vec<SceneBox>,
    /// Optional ground plane for ground multipath.
    pub ground: Option<GroundPlane>,
    /// Foliage volumes that attenuate signals passing through them.
    #[serde(default)]
    pub foliage: Vec<FoliageVolume>,
}

impl GnssScene {
    /// An empty open-sky scene with an asphalt ground at `z = 0`.
    pub fn open_sky() -> Self {
        Self {
            boxes: Vec::new(),
            ground: Some(GroundPlane {
                height_m: 0.0,
                material: SurfaceMaterial::GROUND,
            }),
            foliage: Vec::new(),
        }
    }

    /// Adds a box and returns its index.
    pub fn add_box(&mut self, scene_box: SceneBox) -> usize {
        self.boxes.push(scene_box);
        self.boxes.len() - 1
    }

    /// Closest box hit along a ray (the ground plane is not an occluder).
    pub fn raycast(&self, origin_m: Vec3, direction: Vec3, max_distance_m: f64) -> Option<RayHit> {
        let mut best: Option<RayHit> = None;
        for (index, scene_box) in self.boxes.iter().enumerate() {
            let limit = best.map_or(max_distance_m, |hit| hit.distance_m);
            if let Some((distance_m, normal)) = scene_box.intersect(origin_m, direction, limit) {
                if best.is_none_or(|hit| distance_m < hit.distance_m) {
                    best = Some(RayHit {
                        distance_m,
                        point_m: origin_m + direction * distance_m,
                        normal,
                        source: FacetSource::Box(index),
                    });
                }
            }
        }
        best
    }

    /// Whether anything blocks the ray before `max_distance_m`.
    pub fn occluded(&self, origin_m: Vec3, direction: Vec3, max_distance_m: f64) -> bool {
        self.boxes
            .iter()
            .any(|b| b.intersect(origin_m, direction, max_distance_m).is_some())
    }

    /// Whether a point is inside any solid box.
    pub fn inside_solid(&self, point_m: Vec3) -> bool {
        self.boxes.iter().any(|b| b.contains(point_m))
    }

    /// Foliage attenuation in dB at L1 along a ray segment.
    pub fn foliage_loss_db(&self, origin_m: Vec3, direction: Vec3, max_t: f64) -> f64 {
        self.foliage
            .iter()
            .map(|f| f.loss_db_per_m * f.path_length_m(origin_m, direction, max_t))
            .sum()
    }

    /// All reflecting facets: walls of reflective boxes in box order, then
    /// the ground.
    pub fn facets(&self) -> Vec<Facet> {
        let mut facets: Vec<Facet> = self
            .boxes
            .iter()
            .enumerate()
            .filter(|(_, b)| b.material.is_reflective())
            .flat_map(|(index, b)| b.vertical_facets(index))
            .collect();
        if let Some(ground) = self.ground {
            facets.push(Facet {
                normal: Vec3::Z,
                point_m: Vec3::new(0.0, 0.0, ground.height_m),
                min_m: Vec3::splat(f64::NEG_INFINITY),
                max_m: Vec3::splat(f64::INFINITY),
                material: ground.material,
                source: FacetSource::Ground,
            });
        }
        facets
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn unit_box() -> SceneBox {
        SceneBox::new(Vec3::ZERO, Vec3::ONE, SurfaceMaterial::CONCRETE)
    }

    #[test]
    fn ray_hits_the_near_face_with_its_normal() {
        let (t, normal) = unit_box()
            .intersect(Vec3::new(-2.0, 0.5, 0.5), Vec3::X, 10.0)
            .unwrap();
        assert_relative_eq!(t, 2.0, epsilon = 1e-12);
        assert_eq!(normal, Vec3::NEG_X);
    }

    #[test]
    fn ray_misses_and_respects_range() {
        let b = unit_box();
        assert!(b
            .intersect(Vec3::new(-2.0, 2.0, 0.5), Vec3::X, 10.0)
            .is_none());
        assert!(b
            .intersect(Vec3::new(-2.0, 0.5, 0.5), Vec3::X, 1.5)
            .is_none());
        assert!(b
            .intersect(Vec3::new(-2.0, 0.5, 0.5), Vec3::NEG_X, 10.0)
            .is_none());
    }

    #[test]
    fn ray_from_inside_hits_at_zero() {
        let (t, _) = unit_box()
            .intersect(Vec3::splat(0.5), Vec3::Z, 10.0)
            .unwrap();
        assert_eq!(t, 0.0);
    }

    #[test]
    fn foliage_attenuates_by_path_length() {
        let mut scene = GnssScene::default();
        scene.foliage.push(FoliageVolume {
            min_m: Vec3::new(5.0, -2.0, 0.0),
            max_m: Vec3::new(9.0, 2.0, 5.0),
            loss_db_per_m: 1.0,
        });
        // 4 m through the canopy along x.
        assert_relative_eq!(
            scene.foliage_loss_db(Vec3::new(0.0, 0.0, 1.0), Vec3::X, f64::INFINITY),
            4.0,
            epsilon = 1e-9
        );
        // Stops where the segment ends.
        assert_relative_eq!(
            scene.foliage_loss_db(Vec3::new(0.0, 0.0, 1.0), Vec3::X, 7.0),
            2.0,
            epsilon = 1e-9
        );
        assert_eq!(
            scene.foliage_loss_db(Vec3::new(0.0, 0.0, 6.0), Vec3::X, f64::INFINITY),
            0.0
        );
        // Absorbers are not reflectors.
        scene.add_box(SceneBox::building(
            0.0,
            5.0,
            1.0,
            6.0,
            4.0,
            SurfaceMaterial::ABSORBER,
        ));
        assert!(scene.facets().is_empty());
        assert!(scene.occluded(Vec3::new(0.5, 0.0, 1.0), Vec3::Y, 10.0));
    }

    #[test]
    fn scene_raycast_returns_the_closest_box() {
        let mut scene = GnssScene::open_sky();
        scene.add_box(SceneBox::building(
            10.0,
            -1.0,
            11.0,
            1.0,
            5.0,
            SurfaceMaterial::GLASS,
        ));
        scene.add_box(SceneBox::building(
            5.0,
            -1.0,
            6.0,
            1.0,
            5.0,
            SurfaceMaterial::GLASS,
        ));
        let hit = scene
            .raycast(Vec3::new(0.0, 0.0, 1.0), Vec3::X, 100.0)
            .unwrap();
        assert_eq!(hit.source, FacetSource::Box(1));
        assert_relative_eq!(hit.distance_m, 5.0, epsilon = 1e-12);
        assert!(scene.occluded(Vec3::new(0.0, 0.0, 1.0), Vec3::X, 100.0));
        assert!(!scene.occluded(Vec3::new(0.0, 0.0, 6.0), Vec3::X, 100.0));
    }
}
