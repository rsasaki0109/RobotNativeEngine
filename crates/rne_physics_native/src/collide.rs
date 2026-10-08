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

/// Lateral slack, in meters, within which a corner still counts as over a face.
const FACE_SLACK_M: f64 = 1.0e-4;

/// A corner of one box against a face of the other, from [`box_box`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct BoxCorner {
    /// Whether the corner belongs to the first box.
    pub on_first: bool,
    /// Corner index, in [`collider_samples`] order.
    pub corner: usize,
    /// Corner position in the simulation frame.
    pub point_m: Vec3,
    /// Unit normal from the face's box into the corner's box.
    pub normal: Vec3,
    /// Signed distance from the face plane to the corner, in meters.
    pub gap_m: f64,
}

/// Corner-on-face contacts between two boxes within `margin_m`.
///
/// The face normal is the separating-axis candidate among the six face
/// normals with the least penetration (or the largest separation). The corners
/// of each box that lie over the other box's face along that axis become
/// contacts, so a box resting on a larger or a smaller box, or on one of the
/// same size, gets its support points; crossing edges with no corner over a
/// face are not detected.
pub(crate) fn box_box(
    first_half: Vec3,
    first: &Transform3,
    second_half: Vec3,
    second: &Transform3,
    margin_m: f64,
) -> Vec<BoxCorner> {
    let axes = |pose: &Transform3| {
        [
            pose.rotation * Vec3::X,
            pose.rotation * Vec3::Y,
            pose.rotation * Vec3::Z,
        ]
    };
    let (first_axes, second_axes) = (axes(first), axes(second));
    let radius = |half: Vec3, box_axes: &[Vec3; 3], axis: Vec3| {
        half.x * box_axes[0].dot(axis).abs()
            + half.y * box_axes[1].dot(axis).abs()
            + half.z * box_axes[2].dot(axis).abs()
    };
    let offset = first.translation - second.translation;
    let mut best: Option<(f64, Vec3)> = None;
    for axis in first_axes.iter().chain(&second_axes) {
        let separation = offset.dot(*axis).abs()
            - radius(first_half, &first_axes, *axis)
            - radius(second_half, &second_axes, *axis);
        if best.is_none_or(|(best_separation, _)| separation > best_separation + 1.0e-9) {
            let normal = if offset.dot(*axis) >= 0.0 {
                *axis
            } else {
                -*axis
            };
            best = Some((separation, normal));
        }
    }
    let Some((separation, normal)) = best else {
        return Vec::new();
    };
    if separation > margin_m {
        return Vec::new();
    }
    let mut corners = Vec::new();
    // `normal` points from the second box into the first.
    for (on_first, corner_half, corner_pose, face_half, face_pose, face_axes, toward) in [
        (
            true,
            first_half,
            first,
            second_half,
            second,
            &second_axes,
            normal,
        ),
        (
            false,
            second_half,
            second,
            first_half,
            first,
            &first_axes,
            -normal,
        ),
    ] {
        let face_offset = radius(face_half, face_axes, toward);
        let mut samples = Vec::with_capacity(8);
        collider_samples(
            &ColliderShape::Cuboid {
                half_extents_m: corner_half,
            },
            corner_pose,
            &mut samples,
        );
        for (corner, sample) in samples.iter().enumerate() {
            let relative = sample.center_m - face_pose.translation;
            let gap_m = relative.dot(toward) - face_offset;
            if gap_m > margin_m {
                continue;
            }
            // Over the face: within the box's extent along the other two axes.
            let over_face = face_axes
                .iter()
                .zip([face_half.x, face_half.y, face_half.z])
                .filter(|(axis, _)| axis.dot(toward).abs() < 0.5)
                .all(|(axis, half)| relative.dot(*axis).abs() <= half + FACE_SLACK_M);
            if over_face {
                corners.push(BoxCorner {
                    on_first,
                    corner,
                    point_m: sample.center_m,
                    normal: toward,
                    gap_m,
                });
            }
        }
    }
    corners
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
    fn box_box_finds_the_supporting_face_for_aligned_and_offset_stacks() {
        let half = Vec3::new(0.2, 0.2, 0.1);
        let lower = Transform3::from_translation_rotation(Vec3::new(0.0, 0.0, 0.1), Quat::IDENTITY);
        // Same size, exactly aligned: four bottom corners of the upper box on
        // the lower box's top face, and four top corners of the lower box
        // under the upper box's bottom face.
        let upper =
            Transform3::from_translation_rotation(Vec3::new(0.0, 0.0, 0.302), Quat::IDENTITY);
        let corners = box_box(half, &upper, half, &lower, 0.02);
        assert_eq!(corners.len(), 8);
        for corner in &corners {
            assert!((corner.gap_m - 0.002).abs() < 1.0e-12);
            let expected = if corner.on_first {
                Vec3::Z
            } else {
                Vec3::NEG_Z
            };
            assert_eq!(corner.normal, expected);
        }
        // A small box on a big one: only its own corners touch.
        let small = Vec3::new(0.05, 0.05, 0.05);
        let on_top =
            Transform3::from_translation_rotation(Vec3::new(0.1, 0.0, 0.25), Quat::IDENTITY);
        let corners = box_box(small, &on_top, half, &lower, 0.02);
        assert_eq!(corners.len(), 4);
        assert!(corners
            .iter()
            .all(|corner| corner.on_first && corner.gap_m.abs() < 1.0e-12));
        // Far apart: nothing.
        let far = Transform3::from_translation_rotation(Vec3::new(0.0, 0.0, 1.0), Quat::IDENTITY);
        assert!(box_box(half, &far, half, &lower, 0.02).is_empty());
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
