//! Contact candidates between moving collider samples and static colliders.

use rne_ecs::Entity;
use rne_math::{Quat, Vec3};
use rne_physics::{height_field_surface, ColliderShape};
use rne_world::Transform3;

/// Most hull vertices sampled from one convex collider.
const MAX_HULL_SAMPLES: usize = 256;
/// Most spheres along one capsule.
const MAX_CAPSULE_SPHERES: usize = 16;

/// A sphere (or, with zero radius, a point) that stands in for part of a
/// moving collider, in its body frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Sample {
    /// Sphere center in the body frame, in meters.
    pub center_m: Vec3,
    /// Sphere radius, in meters.
    pub radius_m: f64,
}

/// Appends the samples of `shape` placed at `offset` in the body frame.
///
/// Spheres are exact; a capsule is a row of spheres along its axis; boxes and
/// convex hulls contribute their vertices. Other shapes are not sampled.
pub(crate) fn collider_samples(shape: &ColliderShape, offset: &Transform3, out: &mut Vec<Sample>) {
    let place = |point: Vec3| offset.translation + offset.rotation * point;
    match shape {
        ColliderShape::Sphere { radius_m } => out.push(Sample {
            center_m: place(Vec3::ZERO),
            radius_m: *radius_m,
        }),
        ColliderShape::Capsule {
            half_height_m,
            radius_m,
        } => {
            let segments = if *radius_m > 0.0 {
                ((2.0 * half_height_m / radius_m).ceil() as usize).clamp(1, MAX_CAPSULE_SPHERES)
            } else {
                1
            };
            for index in 0..=segments {
                let y = -half_height_m + 2.0 * half_height_m * index as f64 / segments as f64;
                out.push(Sample {
                    center_m: place(Vec3::new(0.0, y, 0.0)),
                    radius_m: *radius_m,
                });
            }
        }
        ColliderShape::Cuboid { half_extents_m } => {
            for corner in 0..8 {
                let sign = |bit: usize| if corner & bit == 0 { -1.0 } else { 1.0 };
                out.push(Sample {
                    center_m: place(Vec3::new(
                        sign(1) * half_extents_m.x,
                        sign(2) * half_extents_m.y,
                        sign(4) * half_extents_m.z,
                    )),
                    radius_m: 0.0,
                });
            }
        }
        ColliderShape::ConvexHull { points } => {
            out.extend(points.iter().take(MAX_HULL_SAMPLES).map(|point| Sample {
                center_m: place(*point),
                radius_m: 0.0,
            }));
        }
        ColliderShape::Compound { parts } => {
            for part in parts.iter() {
                collider_samples(&part.shape, &offset.mul_transform(&part.local_offset), out);
            }
        }
        ColliderShape::Plane { .. }
        | ColliderShape::TriMesh { .. }
        | ColliderShape::HeightField { .. } => {}
    }
}

/// A collider on a fixed or kinematic body, posed in the simulation frame.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct StaticCollider {
    /// Entity carrying the collider.
    pub entity: Entity,
    /// Collider shape.
    pub shape: ColliderShape,
    /// Collider pose in the simulation frame.
    pub pose: Transform3,
    /// Friction coefficient of its material.
    pub friction: f64,
}

/// Closest approach of a sample sphere to a static collider.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Hit {
    /// Signed distance between the surfaces, in meters.
    pub gap_m: f64,
    /// Unit normal from the static collider toward the sample.
    pub normal: Vec3,
    /// Point on the sample's surface closest to the collider.
    pub point_m: Vec3,
}

impl StaticCollider {
    /// Closest approach of the sphere at `center_m` (simulation frame) with
    /// radius `radius_m`, or `None` when the shape has no answer there (for
    /// example outside a height field's footprint).
    pub(crate) fn query(&self, center_m: Vec3, radius_m: f64) -> Option<Hit> {
        let local = self.pose.rotation.conjugate() * (center_m - self.pose.translation);
        let (distance, normal_local) = match &self.shape {
            ColliderShape::Plane { normal } => {
                let normal = normal.try_normalize()?;
                (local.dot(normal), normal)
            }
            ColliderShape::Sphere { radius_m: sphere } => {
                let length = local.length();
                (length - sphere, local.try_normalize()?)
            }
            ColliderShape::Capsule {
                half_height_m,
                radius_m: capsule,
            } => {
                let axis = Vec3::new(0.0, local.y.clamp(-half_height_m, *half_height_m), 0.0);
                let offset = local - axis;
                (offset.length() - capsule, offset.try_normalize()?)
            }
            ColliderShape::Cuboid { half_extents_m } => box_distance(local, *half_extents_m)?,
            ColliderShape::HeightField { .. } => {
                let surface = height_field_surface(&self.shape, local.x, local.z)?;
                (
                    (local.y - surface.height_m) * surface.normal.y,
                    surface.normal,
                )
            }
            ColliderShape::ConvexHull { .. }
            | ColliderShape::TriMesh { .. }
            | ColliderShape::Compound { .. } => return None,
        };
        let normal = self.pose.rotation * normal_local;
        Some(Hit {
            gap_m: distance - radius_m,
            normal,
            point_m: center_m - normal * radius_m,
        })
    }
}

/// Signed distance and outward normal from a box centered at the origin.
fn box_distance(local: Vec3, half: Vec3) -> Option<(f64, Vec3)> {
    let clamped = local.clamp(-half, half);
    let outside = local - clamped;
    if let Some(normal) = outside.try_normalize() {
        return Some((outside.length(), normal));
    }
    // Inside: leave through the nearest face.
    let depth = half - local.abs();
    let (axis, depth) = [(Vec3::X, depth.x), (Vec3::Y, depth.y), (Vec3::Z, depth.z)]
        .into_iter()
        .min_by(|a, b| a.1.total_cmp(&b.1))?;
    let sign = if local.dot(axis) < 0.0 { -1.0 } else { 1.0 };
    Some((-depth, axis * sign))
}

/// Rotation that takes RNE's Y-up world into the Z-up simulation frame.
pub(crate) fn world_to_sim() -> Quat {
    Quat::from_rotation_x(std::f64::consts::FRAC_PI_2)
}

/// A world pose in the simulation frame.
pub(crate) fn sim_from_world(pose: &Transform3) -> Transform3 {
    let rotation = world_to_sim();
    Transform3::from_translation_rotation(rotation * pose.translation, rotation * pose.rotation)
}

/// A simulation-frame pose in RNE's world.
pub(crate) fn world_from_sim(pose: &Transform3) -> Transform3 {
    let rotation = world_to_sim().conjugate();
    Transform3::from_translation_rotation(rotation * pose.translation, rotation * pose.rotation)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_ecs::World;

    fn entity() -> Entity {
        World::new().spawn_empty().id()
    }

    fn collider(shape: ColliderShape, pose: Transform3) -> StaticCollider {
        StaticCollider {
            entity: entity(),
            shape,
            pose,
            friction: 0.5,
        }
    }

    #[test]
    fn plane_box_sphere_and_capsule_distances() {
        let up = Transform3::from_translation_rotation(Vec3::new(0.0, 0.0, 1.0), Quat::IDENTITY);
        let plane = collider(ColliderShape::Plane { normal: Vec3::Z }, up);
        let hit = plane.query(Vec3::new(3.0, -2.0, 1.5), 0.2).expect("plane");
        assert!((hit.gap_m - 0.3).abs() < 1.0e-12);
        assert_eq!(hit.normal, Vec3::Z);
        assert!((hit.point_m - Vec3::new(3.0, -2.0, 1.3)).length() < 1.0e-12);

        let cuboid = collider(
            ColliderShape::Cuboid {
                half_extents_m: Vec3::new(1.0, 2.0, 0.5),
            },
            Transform3::default(),
        );
        let above = cuboid.query(Vec3::new(0.2, 0.3, 0.8), 0.1).expect("box");
        assert!((above.gap_m - 0.2).abs() < 1.0e-12 && above.normal == Vec3::Z);
        let inside = cuboid.query(Vec3::new(0.95, 0.0, 0.0), 0.0).expect("box");
        assert!((inside.gap_m + 0.05).abs() < 1.0e-12 && inside.normal == Vec3::X);
        let corner = cuboid.query(Vec3::new(2.0, 3.0, 0.0), 0.0).expect("box");
        assert!((corner.gap_m - 2.0_f64.sqrt()).abs() < 1.0e-12);

        let sphere = collider(
            ColliderShape::Sphere { radius_m: 1.0 },
            Transform3::default(),
        );
        assert!(
            (sphere
                .query(Vec3::new(0.0, 3.0, 0.0), 0.5)
                .expect("sphere")
                .gap_m
                - 1.5)
                .abs()
                < 1.0e-12
        );

        let capsule = collider(
            ColliderShape::Capsule {
                half_height_m: 1.0,
                radius_m: 0.25,
            },
            Transform3::default(),
        );
        let side = capsule
            .query(Vec3::new(1.0, 0.5, 0.0), 0.0)
            .expect("capsule");
        assert!((side.gap_m - 0.75).abs() < 1.0e-12 && side.normal == Vec3::X);
    }

    #[test]
    fn samples_cover_spheres_capsules_boxes_and_compounds() {
        let offset =
            Transform3::from_translation_rotation(Vec3::new(0.0, 1.0, 0.0), Quat::IDENTITY);
        let mut samples = Vec::new();
        collider_samples(
            &ColliderShape::Sphere { radius_m: 0.1 },
            &offset,
            &mut samples,
        );
        assert_eq!(
            samples,
            vec![Sample {
                center_m: Vec3::Y,
                radius_m: 0.1
            }]
        );
        samples.clear();
        collider_samples(
            &ColliderShape::Capsule {
                half_height_m: 0.2,
                radius_m: 0.1,
            },
            &Transform3::default(),
            &mut samples,
        );
        assert_eq!(samples.len(), 5);
        assert!((samples[0].center_m.y + 0.2).abs() < 1.0e-12);
        samples.clear();
        collider_samples(
            &ColliderShape::Cuboid {
                half_extents_m: Vec3::splat(0.5),
            },
            &offset,
            &mut samples,
        );
        assert_eq!(samples.len(), 8);
        assert!(samples.iter().all(|sample| sample.radius_m == 0.0));
        assert!(samples
            .iter()
            .any(|sample| sample.center_m == Vec3::new(0.5, 1.5, 0.5)));
    }

    #[test]
    fn world_and_simulation_frames_round_trip() {
        let pose = Transform3::from_translation_rotation(
            Vec3::new(1.0, 2.0, 3.0),
            Quat::from_rotation_y(0.7),
        );
        let sim = sim_from_world(&pose);
        assert!((sim.translation - Vec3::new(1.0, -3.0, 2.0)).length() < 1.0e-12);
        let back = world_from_sim(&sim);
        assert!((back.translation - pose.translation).length() < 1.0e-12);
        assert!(back.rotation.dot(pose.rotation).abs() > 1.0 - 1.0e-12);
    }
}
