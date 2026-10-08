//! Incremental 3D convex hull of a point cloud.

use rne_math::Vec3;

/// Triangles of the convex hull of `points`, as indices into it, wound
/// counter-clockwise seen from outside; `None` when the points do not span
/// a volume (fewer than four, or all on one plane).
///
/// Points are added in order: each one outside the hull so far replaces the
/// faces it sees with a fan from the horizon to itself. Points within a
/// tolerance of the hull's faces (relative to the cloud's size) are treated
/// as on it, so near-coplanar faces do not split into slivers.
pub(crate) fn convex_hull(points: &[Vec3]) -> Option<Vec<[u32; 3]>> {
    if points.len() < 4 || points.iter().any(|point| !point.is_finite()) {
        return None;
    }
    let extent = points
        .iter()
        .map(|point| (*point - points[0]).length())
        .fold(0.0, f64::max);
    let tolerance = 1.0e-9 * extent.max(f64::MIN_POSITIVE);
    let [a, b, c, d] = initial_tetrahedron(points, tolerance)?;
    let mut faces: Vec<[u32; 3]> = vec![[a, b, c], [a, c, d], [a, d, b], [b, d, c]];
    // Orient the tetrahedron outward: its fourth corner must lie behind the
    // first face.
    if height(points, faces[0], points[d as usize]) > 0.0 {
        for face in &mut faces {
            face.swap(1, 2);
        }
    }
    for (index, point) in points.iter().enumerate() {
        let index = index as u32;
        if [a, b, c, d].contains(&index) {
            continue;
        }
        let visible: Vec<bool> = faces
            .iter()
            .map(|face| height(points, *face, *point) > tolerance)
            .collect();
        if !visible.contains(&true) {
            continue;
        }
        // Horizon: edges of visible faces whose reverse is on no visible face.
        let visible_edges: Vec<(u32, u32)> = faces
            .iter()
            .zip(&visible)
            .filter(|(_, seen)| **seen)
            .flat_map(|(face, _)| [(face[0], face[1]), (face[1], face[2]), (face[2], face[0])])
            .collect();
        let horizon: Vec<(u32, u32)> = visible_edges
            .iter()
            .copied()
            .filter(|(from, to)| !visible_edges.contains(&(*to, *from)))
            .collect();
        let mut kept: Vec<[u32; 3]> = faces
            .iter()
            .zip(&visible)
            .filter(|(_, seen)| !**seen)
            .map(|(face, _)| *face)
            .collect();
        kept.extend(horizon.into_iter().map(|(from, to)| [from, to, index]));
        faces = kept;
    }
    Some(faces)
}

/// Signed distance of `point` above the plane of `face`, along its
/// counter-clockwise normal (unnormalized: scaled by twice the face area).
fn height(points: &[Vec3], face: [u32; 3], point: Vec3) -> f64 {
    let [a, b, c] = face.map(|index| points[index as usize]);
    let normal = (b - a).cross(c - a);
    let length = normal.length();
    if length == 0.0 {
        return 0.0;
    }
    normal.dot(point - a) / length
}

/// Four points spanning a volume: the first point, the one farthest from it,
/// the one farthest from their line, and the one farthest from their plane.
fn initial_tetrahedron(points: &[Vec3], tolerance: f64) -> Option<[u32; 4]> {
    let farthest = |score: &dyn Fn(Vec3) -> f64| {
        points
            .iter()
            .enumerate()
            .max_by(|x, y| score(*x.1).total_cmp(&score(*y.1)).then(y.0.cmp(&x.0)))
            .map(|(index, point)| (index as u32, score(*point)))
    };
    let origin = points[0];
    let (b, reach) = farthest(&|point| (point - origin).length())?;
    if reach <= tolerance {
        return None;
    }
    let axis = (points[b as usize] - origin) / reach;
    let (c, off_line) = farthest(&|point| (point - origin).cross(axis).length())?;
    if off_line <= tolerance {
        return None;
    }
    let normal = (points[b as usize] - origin)
        .cross(points[c as usize] - origin)
        .normalize();
    let (d, off_plane) = farthest(&|point| (point - origin).dot(normal).abs())?;
    (off_plane > tolerance).then_some([0, b, c, d])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn check_hull(points: &[Vec3], faces: &[[u32; 3]]) {
        // Closed: every directed edge appears once, its reverse once.
        let edges: Vec<(u32, u32)> = faces
            .iter()
            .flat_map(|face| [(face[0], face[1]), (face[1], face[2]), (face[2], face[0])])
            .collect();
        for (from, to) in &edges {
            assert_eq!(
                edges.iter().filter(|edge| **edge == (*from, *to)).count(),
                1
            );
            assert!(edges.contains(&(*to, *from)), "edge {from}-{to} is open");
        }
        // Convex and outward: every point lies on or behind every face.
        for face in faces {
            for point in points {
                assert!(
                    height(points, *face, *point) < 1.0e-9,
                    "point above {face:?}"
                );
            }
        }
    }

    #[test]
    fn hulls_of_a_cube_cloud_and_a_sphere_cloud_are_closed_and_convex() {
        // A cube's corners plus interior and face-center points.
        let mut cube: Vec<Vec3> = (0..8)
            .map(|corner| {
                let sign = |bit: usize| if corner & bit == 0 { -1.0 } else { 1.0 };
                Vec3::new(sign(1), sign(2), sign(4))
            })
            .collect();
        cube.extend([
            Vec3::ZERO,
            Vec3::new(0.2, -0.3, 0.1),
            Vec3::new(1.0, 0.0, 0.0),
        ]);
        let faces = convex_hull(&cube).expect("cube hull");
        check_hull(&cube, &faces);
        // Interior points are never hull vertices; 12 triangles cover the cube.
        assert!(faces
            .iter()
            .flatten()
            .all(|index| *index < 8 || *index == 10));
        // A point cloud on a sphere: every point ends on the hull.
        let sphere: Vec<Vec3> = (0..60)
            .map(|index| {
                let t = index as f64 / 60.0;
                let height = 1.0 - 2.0 * t - 1.0 / 60.0;
                let radius = (1.0 - height * height).sqrt();
                let angle = index as f64 * 2.399_963;
                Vec3::new(radius * angle.cos(), height, radius * angle.sin())
            })
            .collect();
        let faces = convex_hull(&sphere).expect("sphere hull");
        check_hull(&sphere, &faces);
        assert_eq!(faces.len(), 2 * sphere.len() - 4);
        // Flat and degenerate clouds have no hull.
        let flat = [Vec3::ZERO, Vec3::X, Vec3::Z, Vec3::new(1.0, 0.0, 1.0)];
        assert!(convex_hull(&flat).is_none());
        assert!(convex_hull(&flat[..3]).is_none());
    }
}
