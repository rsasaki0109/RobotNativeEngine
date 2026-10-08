//! Ray casts against the colliders of a native world.

use crate::collide::{find_hull, find_tree};
use crate::mesh::{ray_triangle, MeshTree};
use rne_math::Vec3;
use rne_physics::{height_field_surface, ColliderShape};
use rne_world::Transform3;
use std::sync::Arc;

/// Directions with a smaller component than this are parallel to a slab or
/// a plane.
const PARALLEL: f64 = 1.0e-12;

/// First hit of the ray `origin + t * direction` (unit `direction`, both in
/// the frame of `pose`) on `shape` placed at `pose`, with `0 <= t <=
/// max_distance_m`: the distance and the surface normal, facing the ray.
///
/// Spheres, capsules, boxes, planes (as half-spaces), and convex hulls are
/// solid: a ray that starts inside one hits it at distance zero with a zero
/// normal, as Rapier reports solid hits. Height fields and triangle meshes
/// are surfaces, hit from either side.
pub(crate) fn cast(
    shape: &ColliderShape,
    pose: &Transform3,
    meshes: &[Arc<MeshTree>],
    origin: Vec3,
    direction: Vec3,
    max_distance_m: f64,
) -> Option<(f64, Vec3)> {
    let local_origin = pose.rotation.conjugate() * (origin - pose.translation);
    let local_direction = pose.rotation.conjugate() * direction;
    let (distance, normal) = match shape {
        ColliderShape::Sphere { radius_m } => sphere(local_origin, local_direction, *radius_m),
        ColliderShape::Capsule {
            half_height_m,
            radius_m,
        } => capsule(local_origin, local_direction, *half_height_m, *radius_m),
        ColliderShape::Cuboid { half_extents_m } => {
            cuboid(local_origin, local_direction, *half_extents_m)
        }
        ColliderShape::Plane { normal } => plane(local_origin, local_direction, *normal),
        ColliderShape::HeightField { .. } => {
            height_field(shape, local_origin, local_direction, max_distance_m)
        }
        ColliderShape::TriMesh { vertices, indices } => {
            match find_tree(meshes, vertices, indices) {
                Some(tree) => tree.ray(local_origin, local_direction, max_distance_m),
                None => tri_mesh(vertices, indices, local_origin, local_direction),
            }
            .map(|(t, normal)| {
                let facing = if normal.dot(local_direction) > 0.0 {
                    -normal
                } else {
                    normal
                };
                (t, facing)
            })
        }
        ColliderShape::Compound { parts } => {
            return parts
                .iter()
                .filter_map(|part| {
                    cast(
                        &part.shape,
                        &pose.mul_transform(&part.local_offset),
                        meshes,
                        origin,
                        direction,
                        max_distance_m,
                    )
                })
                .min_by(|a, b| a.0.total_cmp(&b.0));
        }
        ColliderShape::ConvexHull { points } => hull(
            find_hull(meshes, points)?.planes(),
            local_origin,
            local_direction,
        ),
    }?;
    (distance <= max_distance_m).then(|| (distance, pose.rotation * normal))
}

fn sphere(origin: Vec3, direction: Vec3, radius_m: f64) -> Option<(f64, Vec3)> {
    let c = origin.length_squared() - radius_m * radius_m;
    if c <= 0.0 {
        return Some((0.0, Vec3::ZERO));
    }
    let b = origin.dot(direction);
    let discriminant = b * b - c;
    if b > 0.0 || discriminant < 0.0 {
        return None;
    }
    let t = -b - discriminant.sqrt();
    Some((t, (origin + direction * t) / radius_m))
}

fn capsule(
    origin: Vec3,
    direction: Vec3,
    half_height_m: f64,
    radius_m: f64,
) -> Option<(f64, Vec3)> {
    let axis_point =
        |point: Vec3| Vec3::new(0.0, point.y.clamp(-half_height_m, half_height_m), 0.0);
    if (origin - axis_point(origin)).length_squared() <= radius_m * radius_m {
        return Some((0.0, Vec3::ZERO));
    }
    let mut best: Option<(f64, Vec3)> = None;
    // The side: an infinite cylinder about Y, cut to the segment.
    let a = direction.x * direction.x + direction.z * direction.z;
    if a > PARALLEL {
        let b = origin.x * direction.x + origin.z * direction.z;
        let c = origin.x * origin.x + origin.z * origin.z - radius_m * radius_m;
        let discriminant = b * b - a * c;
        if discriminant >= 0.0 {
            let t = (-b - discriminant.sqrt()) / a;
            let point = origin + direction * t;
            if t >= 0.0 && point.y.abs() <= half_height_m {
                best = Some((t, Vec3::new(point.x, 0.0, point.z) / radius_m));
            }
        }
    }
    for end in [-half_height_m, half_height_m] {
        let center = Vec3::new(0.0, end, 0.0);
        if let Some(hit) = sphere(origin - center, direction, radius_m) {
            if best.is_none_or(|(t, _)| hit.0 < t) {
                best = Some(hit);
            }
        }
    }
    best
}

fn cuboid(origin: Vec3, direction: Vec3, half: Vec3) -> Option<(f64, Vec3)> {
    let (o, d, h) = (origin.to_array(), direction.to_array(), half.to_array());
    if (0..3).all(|k| o[k].abs() <= h[k]) {
        return Some((0.0, Vec3::ZERO));
    }
    let (mut enter, mut exit, mut normal) = (f64::NEG_INFINITY, f64::INFINITY, Vec3::ZERO);
    for k in 0..3 {
        if d[k].abs() < PARALLEL {
            if o[k].abs() > h[k] {
                return None;
            }
            continue;
        }
        let (near, far) = {
            let first = (-h[k] - o[k]) / d[k];
            let second = (h[k] - o[k]) / d[k];
            (first.min(second), first.max(second))
        };
        if near > enter {
            enter = near;
            let mut axis = [0.0; 3];
            axis[k] = -d[k].signum();
            normal = Vec3::from_array(axis);
        }
        exit = exit.min(far);
    }
    (enter <= exit && enter >= 0.0).then_some((enter, normal))
}

fn plane(origin: Vec3, direction: Vec3, normal: Vec3) -> Option<(f64, Vec3)> {
    let normal = normal.try_normalize()?;
    let height = origin.dot(normal);
    if height <= 0.0 {
        return Some((0.0, Vec3::ZERO));
    }
    let rate = direction.dot(normal);
    (rate < -PARALLEL).then(|| (-height / rate, normal))
}

/// The first crossing of the bilinear surface of [`height_field_surface`].
///
/// The ray is cut where it crosses the grid lines; within each cell the
/// height difference along the ray is quadratic, so it is solved exactly.
fn height_field(
    shape: &ColliderShape,
    origin: Vec3,
    direction: Vec3,
    max_distance_m: f64,
) -> Option<(f64, Vec3)> {
    let ColliderShape::HeightField {
        nrows,
        ncols,
        heights_m,
        scale,
    } = shape
    else {
        return None;
    };
    let (rows, columns) = (*nrows as usize, *ncols as usize);
    if rows < 2 || columns < 2 || heights_m.len() != rows * columns {
        return None;
    }
    let heights = heights_m.iter().map(|height| height * scale.y);
    let low = heights.clone().fold(f64::INFINITY, f64::min);
    let high = heights.fold(f64::NEG_INFINITY, f64::max);
    let half = Vec3::new(0.5 * scale.x.abs(), 0.5 * (high - low), 0.5 * scale.z.abs());
    let center = Vec3::new(0.0, 0.5 * (high + low), 0.0);
    let (start, end) = slab_interval(origin - center, direction, half)?;
    let (start, end) = (start.max(0.0), end.min(max_distance_m));
    if start > end {
        return None;
    }
    let gap = |t: f64| {
        let point = origin + direction * t;
        height_field_surface(shape, point.x, point.z).map(|surface| point.y - surface.height_m)
    };
    let mut breaks = vec![start, end];
    let lines = |count: usize, size: f64, o: f64, d: f64, breaks: &mut Vec<f64>| {
        if d.abs() < PARALLEL {
            return;
        }
        let spacing = size / (count - 1) as f64;
        for line in 1..count - 1 {
            let t = (-0.5 * size + spacing * line as f64 - o) / d;
            if t > start && t < end {
                breaks.push(t);
            }
        }
    };
    lines(columns, scale.x, origin.x, direction.x, &mut breaks);
    lines(rows, scale.z, origin.z, direction.z, &mut breaks);
    breaks.sort_by(f64::total_cmp);
    for pair in breaks.windows(2) {
        let (from, to) = (pair[0], pair[1]);
        let (Some(first), Some(middle), Some(last)) = (gap(from), gap(0.5 * (from + to)), gap(to))
        else {
            continue;
        };
        let Some(s) = first_root(first, middle, last) else {
            continue;
        };
        let t = from + (to - from) * s;
        let point = origin + direction * t;
        let mut normal = height_field_surface(shape, point.x, point.z)?.normal;
        if normal.dot(direction) > 0.0 {
            normal = -normal;
        }
        return Some((t, normal));
    }
    None
}

/// A ray against the solid convex hull with these face planes.
fn hull(planes: &[(Vec3, f64)], origin: Vec3, direction: Vec3) -> Option<(f64, Vec3)> {
    let (mut enter, mut exit, mut normal) = (0.0_f64, f64::INFINITY, Vec3::ZERO);
    for (plane, offset) in planes {
        let height = plane.dot(origin) - offset;
        let rate = plane.dot(direction);
        if rate.abs() < PARALLEL {
            if height > 0.0 {
                return None;
            }
            continue;
        }
        let t = -height / rate;
        if rate < 0.0 {
            if t > enter {
                enter = t;
                normal = *plane;
            }
        } else {
            exit = exit.min(t);
        }
    }
    // A ray from inside enters at zero, with a zero normal.
    (enter <= exit).then_some((enter, normal))
}

/// Entry and exit distances of a ray through the box `[-half, half]`.
fn slab_interval(origin: Vec3, direction: Vec3, half: Vec3) -> Option<(f64, f64)> {
    let (o, d, h) = (origin.to_array(), direction.to_array(), half.to_array());
    let (mut enter, mut exit) = (f64::NEG_INFINITY, f64::INFINITY);
    for k in 0..3 {
        if d[k].abs() < PARALLEL {
            if o[k].abs() > h[k] {
                return None;
            }
            continue;
        }
        let first = (-h[k] - o[k]) / d[k];
        let second = (h[k] - o[k]) / d[k];
        enter = enter.max(first.min(second));
        exit = exit.min(first.max(second));
    }
    (enter <= exit).then_some((enter, exit))
}

/// Smallest `s` in `[0, 1]` where the quadratic through `(0, first)`,
/// `(0.5, middle)`, and `(1, last)` is zero.
fn first_root(first: f64, middle: f64, last: f64) -> Option<f64> {
    if first == 0.0 {
        return Some(0.0);
    }
    let a = 2.0 * first - 4.0 * middle + 2.0 * last;
    let b = 4.0 * middle - 3.0 * first - last;
    let c = first;
    let scale = a.abs().max(b.abs()).max(c.abs());
    let roots: Vec<f64> = if a.abs() <= 1.0e-12 * scale {
        if b.abs() <= 1.0e-12 * scale {
            return None;
        }
        vec![-c / b]
    } else {
        let discriminant = b * b - 4.0 * a * c;
        if discriminant < 0.0 {
            return None;
        }
        // The stable pair of roots.
        let q = -0.5 * (b + b.signum() * discriminant.sqrt());
        let mut roots = vec![q / a];
        if q != 0.0 {
            roots.push(c / q);
        }
        roots
    };
    roots
        .into_iter()
        .filter(|s| (0.0..=1.0).contains(s))
        .min_by(f64::total_cmp)
}

/// Nearest triangle hit, without a tree: its distance and face normal.
fn tri_mesh(
    vertices: &[Vec3],
    indices: &[u32],
    origin: Vec3,
    direction: Vec3,
) -> Option<(f64, Vec3)> {
    indices
        .chunks_exact(3)
        .filter_map(|triangle| {
            let corner = |index: u32| vertices.get(index as usize).copied();
            let corners = [
                corner(triangle[0])?,
                corner(triangle[1])?,
                corner(triangle[2])?,
            ];
            let normal = (corners[1] - corners[0])
                .cross(corners[2] - corners[0])
                .try_normalize()?;
            Some((ray_triangle(origin, direction, &corners)?, normal))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_math::Quat;
    use rne_physics::CompoundPart;

    fn at(translation: Vec3) -> Transform3 {
        Transform3::from_translation_rotation(translation, Quat::IDENTITY)
    }

    fn down(shape: &ColliderShape, pose: &Transform3, origin: Vec3) -> Option<(f64, Vec3)> {
        cast(shape, pose, &[], origin, Vec3::NEG_Y, 100.0)
    }

    #[test]
    fn solid_primitives_are_hit_on_their_surface_or_at_zero_from_inside() {
        let sphere = ColliderShape::Sphere { radius_m: 0.5 };
        let (t, normal) = down(
            &sphere,
            &at(Vec3::new(0.0, 1.0, 0.0)),
            Vec3::new(0.3, 5.0, 0.0),
        )
        .expect("sphere hit");
        assert!((t - (4.0 - 0.4)).abs() < 1.0e-12);
        assert!((normal - Vec3::new(0.6, 0.8, 0.0)).length() < 1.0e-12);
        assert_eq!(
            down(&sphere, &at(Vec3::ZERO), Vec3::new(0.1, 0.1, 0.0)),
            Some((0.0, Vec3::ZERO))
        );
        assert!(down(&sphere, &at(Vec3::ZERO), Vec3::new(0.6, 5.0, 0.0)).is_none());

        let capsule = ColliderShape::Capsule {
            half_height_m: 0.5,
            radius_m: 0.2,
        };
        // Down the axis: the top cap at y = 0.7.
        let (t, normal) = down(&capsule, &at(Vec3::ZERO), Vec3::new(0.0, 3.0, 0.0)).expect("cap");
        assert!((t - 2.3).abs() < 1.0e-12 && (normal - Vec3::Y).length() < 1.0e-12);
        // Sideways at the middle: the cylinder at x = 0.2.
        let (t, normal) = cast(
            &capsule,
            &at(Vec3::ZERO),
            &[],
            Vec3::new(2.0, 0.1, 0.0),
            Vec3::NEG_X,
            10.0,
        )
        .expect("side");
        assert!((t - 1.8).abs() < 1.0e-12 && (normal - Vec3::X).length() < 1.0e-12);

        let cuboid = ColliderShape::Cuboid {
            half_extents_m: Vec3::new(0.5, 0.25, 0.5),
        };
        let pose = Transform3::from_translation_rotation(
            Vec3::new(1.0, 0.0, 0.0),
            Quat::from_rotation_z(std::f64::consts::FRAC_PI_2),
        );
        // Rotated a quarter turn, its 0.5 half extent is now vertical.
        let (t, normal) = down(&cuboid, &pose, Vec3::new(1.1, 2.0, 0.0)).expect("box");
        assert!((t - 1.5).abs() < 1.0e-12 && (normal - Vec3::Y).length() < 1.0e-12);
        assert!(down(&cuboid, &pose, Vec3::new(1.3, 2.0, 0.0)).is_none());
        // Beyond the maximum distance: nothing.
        assert!(cast(
            &cuboid,
            &pose,
            &[],
            Vec3::new(1.1, 2.0, 0.0),
            Vec3::NEG_Y,
            1.4
        )
        .is_none());

        let plane = ColliderShape::Plane { normal: Vec3::Y };
        let (t, normal) = down(&plane, &at(Vec3::ZERO), Vec3::new(3.0, 2.0, 1.0)).expect("plane");
        assert!((t - 2.0).abs() < 1.0e-12 && normal == Vec3::Y);
        assert!(cast(&plane, &at(Vec3::ZERO), &[], Vec3::Y, Vec3::Y, 10.0).is_none());
    }

    #[test]
    fn height_fields_and_meshes_are_hit_on_their_surface() {
        // A 2 x 2 m field rising along x from 0 to 1 m, over 3 x 3 samples.
        let field = ColliderShape::HeightField {
            nrows: 3,
            ncols: 3,
            heights_m: vec![0.0, 0.5, 1.0, 0.0, 0.5, 1.0, 0.0, 0.5, 1.0].into(),
            scale: Vec3::new(2.0, 1.0, 2.0),
        };
        let (t, normal) = down(&field, &at(Vec3::ZERO), Vec3::new(0.5, 3.0, 0.2)).expect("field");
        assert!((t - 2.25).abs() < 1.0e-12);
        assert!((normal - Vec3::new(-0.5, 1.0, 0.0).normalize()).length() < 1.0e-12);
        // A level ray along +x meets the slope where its height reaches 0.6 m.
        let (t, _) = cast(
            &field,
            &at(Vec3::ZERO),
            &[],
            Vec3::new(-3.0, 0.6, 0.3),
            Vec3::X,
            10.0,
        )
        .expect("slope");
        assert!((t - 3.2).abs() < 1.0e-12);
        assert!(down(&field, &at(Vec3::ZERO), Vec3::new(1.5, 3.0, 0.0)).is_none());

        let mesh = ColliderShape::TriMesh {
            vertices: vec![
                Vec3::new(-1.0, 0.2, -1.0),
                Vec3::new(1.0, 0.2, -1.0),
                Vec3::new(0.0, 0.2, 1.0),
            ]
            .into(),
            indices: vec![0, 1, 2].into(),
        };
        let (t, normal) = down(&mesh, &at(Vec3::ZERO), Vec3::new(0.0, 1.0, 0.0)).expect("mesh");
        assert!((t - 0.8).abs() < 1.0e-12 && normal == Vec3::Y);
        let (_, normal) = cast(
            &mesh,
            &at(Vec3::ZERO),
            &[],
            Vec3::new(0.0, -1.0, 0.0),
            Vec3::Y,
            5.0,
        )
        .expect("under");
        assert_eq!(normal, Vec3::NEG_Y);

        let compound = ColliderShape::Compound {
            parts: vec![
                CompoundPart {
                    shape: ColliderShape::Sphere { radius_m: 0.1 },
                    local_offset: at(Vec3::new(0.0, 0.5, 0.0)),
                },
                CompoundPart {
                    shape: ColliderShape::Sphere { radius_m: 0.1 },
                    local_offset: at(Vec3::new(0.0, 1.5, 0.0)),
                },
            ]
            .into(),
        };
        let (t, _) = down(&compound, &at(Vec3::ZERO), Vec3::new(0.0, 3.0, 0.0)).expect("parts");
        assert!((t - 1.4).abs() < 1.0e-12);
    }
}
