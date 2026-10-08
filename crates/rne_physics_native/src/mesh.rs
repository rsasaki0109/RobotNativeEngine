//! Bounding-volume tree over a triangle mesh, for contact and ray queries.

use rne_math::Vec3;
use std::sync::Arc;

/// Most triangles in one leaf of a [`MeshTree`].
const LEAF_TRIANGLES: usize = 4;

/// Axis-aligned box.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Bounds {
    min: Vec3,
    max: Vec3,
}

impl Bounds {
    const EMPTY: Self = Self {
        min: Vec3::splat(f64::INFINITY),
        max: Vec3::splat(f64::NEG_INFINITY),
    };

    fn grow(self, point: Vec3) -> Self {
        Self {
            min: self.min.min(point),
            max: self.max.max(point),
        }
    }

    fn union(self, other: Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    fn distance_squared(&self, point: Vec3) -> f64 {
        (point - point.clamp(self.min, self.max)).length_squared()
    }

    /// Entry distance of a ray, if it meets the box within `limit`.
    fn ray_entry(&self, origin: Vec3, inverse: Vec3, limit: f64) -> Option<f64> {
        let first = (self.min - origin) * inverse;
        let second = (self.max - origin) * inverse;
        let near = first.min(second).max_element().max(0.0);
        let far = first.max(second).min_element().min(limit);
        // A ray lying in a slab's plane gives NaN there, which min and max
        // skip, keeping the box.
        (near <= far).then_some(near)
    }
}

/// One tree node: a leaf holds `count` triangles from `start` in
/// [`MeshTree::triangles`]; an inner node's children are `start` and
/// `start + 1` in [`MeshTree::nodes`].
#[derive(Clone, Copy, Debug)]
struct Node {
    bounds: Bounds,
    start: u32,
    count: u32,
}

/// A triangle mesh with a bounding-volume tree, built once per mesh.
///
/// Triangles wind counter-clockwise seen from outside: their face normals
/// point out of the mesh, which tells a contact on which side it is.
#[derive(Debug)]
pub(crate) struct MeshTree {
    /// The mesh the tree was built from, kept alive so that its allocations
    /// identify it in a cache.
    source: (Arc<[Vec3]>, Arc<[u32]>),
    triangles: Vec<[Vec3; 3]>,
    nodes: Vec<Node>,
}

/// Closest point of a mesh to a query point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Closest {
    /// Distance from the query point, in meters.
    pub distance_m: f64,
    /// Closest point on the mesh.
    pub point_m: Vec3,
    /// Outward face normal of the triangle holding the closest point.
    pub face_normal: Vec3,
}

impl MeshTree {
    /// Builds the tree over the valid, non-degenerate triangles of a mesh.
    pub(crate) fn build(vertices: &Arc<[Vec3]>, indices: &Arc<[u32]>) -> Self {
        let mut triangles: Vec<[Vec3; 3]> = indices
            .chunks_exact(3)
            .filter_map(|triangle| {
                let corner = |index: u32| vertices.get(index as usize).copied();
                let [a, b, c] = [
                    corner(triangle[0])?,
                    corner(triangle[1])?,
                    corner(triangle[2])?,
                ];
                let finite = a.is_finite() && b.is_finite() && c.is_finite();
                (finite && (b - a).cross(c - a).length_squared() > 0.0).then_some([a, b, c])
            })
            .collect();
        let mut nodes = vec![Node {
            bounds: Bounds::EMPTY,
            start: 0,
            count: triangles.len() as u32,
        }];
        Self::split(&mut triangles, &mut nodes, 0);
        Self {
            source: (vertices.clone(), indices.clone()),
            triangles,
            nodes,
        }
    }

    /// Whether the tree was built from exactly these allocations.
    pub(crate) fn is_built_from(&self, vertices: &Arc<[Vec3]>, indices: &Arc<[u32]>) -> bool {
        Arc::ptr_eq(&self.source.0, vertices) && Arc::ptr_eq(&self.source.1, indices)
    }

    fn split(triangles: &mut [[Vec3; 3]], nodes: &mut Vec<Node>, index: usize) {
        let Node { start, count, .. } = nodes[index];
        let range = start as usize..(start + count) as usize;
        let slice = &mut triangles[range.clone()];
        nodes[index].bounds = slice
            .iter()
            .flatten()
            .fold(Bounds::EMPTY, |bounds, point| bounds.grow(*point));
        if slice.len() <= LEAF_TRIANGLES {
            return;
        }
        // Median split of the centroids along their widest axis.
        let centroid = |triangle: &[Vec3; 3]| (triangle[0] + triangle[1] + triangle[2]) / 3.0;
        let spread = slice.iter().fold(Bounds::EMPTY, |bounds, triangle| {
            bounds.grow(centroid(triangle))
        });
        let extent = spread.max - spread.min;
        let axis = if extent.x >= extent.y && extent.x >= extent.z {
            0
        } else if extent.y >= extent.z {
            1
        } else {
            2
        };
        let half = slice.len() / 2;
        slice.select_nth_unstable_by(half, |a, b| {
            centroid(a).to_array()[axis].total_cmp(&centroid(b).to_array()[axis])
        });
        let first = nodes.len();
        nodes.push(Node {
            bounds: Bounds::EMPTY,
            start,
            count: half as u32,
        });
        nodes.push(Node {
            bounds: Bounds::EMPTY,
            start: start + half as u32,
            count: count - half as u32,
        });
        nodes[index] = Node {
            bounds: nodes[index].bounds,
            start: first as u32,
            count: 0,
        };
        Self::split(triangles, nodes, first);
        Self::split(triangles, nodes, first + 1);
        debug_assert!(nodes[index].bounds == nodes[first].bounds.union(nodes[first + 1].bounds));
    }

    fn is_leaf(&self, node: &Node) -> bool {
        node.count > 0 || self.nodes.len() == 1
    }

    /// The closest point of the mesh to `point` within `reach_m`.
    pub(crate) fn closest(&self, point: Vec3, reach_m: f64) -> Option<Closest> {
        let mut best: Option<Closest> = None;
        let mut limit = reach_m * reach_m;
        let mut stack = vec![0usize];
        while let Some(index) = stack.pop() {
            let node = self.nodes[index];
            if node.bounds.distance_squared(point) > limit {
                continue;
            }
            if !self.is_leaf(&node) {
                stack.extend([node.start as usize + 1, node.start as usize]);
                continue;
            }
            for triangle in &self.triangles[node.start as usize..(node.start + node.count) as usize]
            {
                let closest = closest_on_triangle(point, triangle);
                let distance_squared = (point - closest).length_squared();
                if distance_squared <= limit
                    && best.is_none_or(|best| distance_squared < best.distance_m.powi(2))
                {
                    limit = distance_squared;
                    best = Some(Closest {
                        distance_m: distance_squared.sqrt(),
                        point_m: closest,
                        face_normal: face_normal(triangle),
                    });
                }
            }
        }
        best
    }

    /// The nearest triangle hit of a ray within `limit_m`: its distance and
    /// face normal.
    pub(crate) fn ray(&self, origin: Vec3, direction: Vec3, limit_m: f64) -> Option<(f64, Vec3)> {
        let inverse = Vec3::ONE / direction;
        let mut best: Option<(f64, Vec3)> = None;
        let mut stack = vec![0usize];
        while let Some(index) = stack.pop() {
            let node = self.nodes[index];
            let limit = best.map_or(limit_m, |(distance, _)| distance);
            if node.bounds.ray_entry(origin, inverse, limit).is_none() {
                continue;
            }
            if !self.is_leaf(&node) {
                stack.extend([node.start as usize + 1, node.start as usize]);
                continue;
            }
            for triangle in &self.triangles[node.start as usize..(node.start + node.count) as usize]
            {
                if let Some(t) = ray_triangle(origin, direction, triangle) {
                    if t <= limit_m && best.is_none_or(|(near, _)| t < near) {
                        best = Some((t, face_normal(triangle)));
                    }
                }
            }
        }
        best
    }
}

fn face_normal(triangle: &[Vec3; 3]) -> Vec3 {
    (triangle[1] - triangle[0])
        .cross(triangle[2] - triangle[0])
        .normalize()
}

/// Möller–Trumbore distance along a ray to a triangle, from either side.
pub(crate) fn ray_triangle(origin: Vec3, direction: Vec3, triangle: &[Vec3; 3]) -> Option<f64> {
    let [a, b, c] = *triangle;
    let (edge1, edge2) = (b - a, c - a);
    let p = direction.cross(edge2);
    let determinant = edge1.dot(p);
    if determinant.abs() < 1.0e-12 * edge1.length() * edge2.length() {
        return None;
    }
    let offset = origin - a;
    let u = offset.dot(p) / determinant;
    let q = offset.cross(edge1);
    let v = direction.dot(q) / determinant;
    let t = edge2.dot(q) / determinant;
    (u >= 0.0 && v >= 0.0 && u + v <= 1.0 && t >= 0.0).then_some(t)
}

/// Closest point of a triangle to `point` (Ericson, Real-Time Collision
/// Detection, 5.1.5).
fn closest_on_triangle(point: Vec3, triangle: &[Vec3; 3]) -> Vec3 {
    let [a, b, c] = *triangle;
    let (ab, ac, ap) = (b - a, c - a, point - a);
    let (d1, d2) = (ab.dot(ap), ac.dot(ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = point - b;
    let (d3, d4) = (ab.dot(bp), ac.dot(bp));
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return a + ab * (d1 / (d1 - d3));
    }
    let cp = point - c;
    let (d5, d6) = (ab.dot(cp), ac.dot(cp));
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return a + ac * (d2 / (d2 - d6));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    let denominator = 1.0 / (va + vb + vc);
    a + ab * (vb * denominator) + ac * (vc * denominator)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A closed unit cube centered at the origin, wound outward, and a
    /// row of 50 grid squares beside it to give the tree some depth.
    fn cube_and_strip() -> (Arc<[Vec3]>, Arc<[u32]>) {
        let mut vertices: Vec<Vec3> = (0..8)
            .map(|corner| {
                let sign = |bit: usize| if corner & bit == 0 { -0.5 } else { 0.5 };
                Vec3::new(sign(1), sign(2), sign(4))
            })
            .collect();
        let mut indices: Vec<u32> = vec![
            0, 2, 1, 1, 2, 3, // -z
            4, 5, 6, 5, 7, 6, // +z
            0, 1, 4, 1, 5, 4, // -y
            2, 6, 3, 3, 6, 7, // +y
            0, 4, 2, 2, 4, 6, // -x
            1, 3, 5, 3, 7, 5, // +x
        ];
        for square in 0..50 {
            let x = 2.0 + square as f64 * 0.1;
            let base = vertices.len() as u32;
            vertices.extend([
                Vec3::new(x, 0.0, 0.0),
                Vec3::new(x + 0.1, 0.0, 0.0),
                Vec3::new(x, 0.0, -0.1),
                Vec3::new(x + 0.1, 0.0, -0.1),
            ]);
            indices.extend([base, base + 1, base + 2, base + 1, base + 3, base + 2]);
        }
        (vertices.into(), indices.into())
    }

    #[test]
    fn closest_points_and_rays_match_a_brute_force_search() {
        let (vertices, indices) = cube_and_strip();
        let tree = MeshTree::build(&vertices, &indices);
        assert!(tree.nodes.len() > 10, "{} nodes", tree.nodes.len());
        assert!(tree.is_built_from(&vertices, &indices));
        assert!(!tree.is_built_from(&vertices.to_vec().into(), &indices));
        // Above the cube's top face: straight down onto it.
        let closest = tree
            .closest(Vec3::new(0.1, 0.8, 0.2), 1.0)
            .expect("closest");
        assert!((closest.distance_m - 0.3).abs() < 1.0e-12);
        assert!((closest.point_m - Vec3::new(0.1, 0.5, 0.2)).length() < 1.0e-12);
        assert!((closest.face_normal - Vec3::Y).length() < 1.0e-12);
        // Inside, nearer the +x face: that face, whose normal points away.
        let inside = tree.closest(Vec3::new(0.4, 0.0, 0.1), 1.0).expect("inside");
        assert!((inside.distance_m - 0.1).abs() < 1.0e-12);
        assert!((inside.face_normal - Vec3::X).length() < 1.0e-12);
        // Out of reach: nothing.
        assert!(tree.closest(Vec3::new(0.0, 3.0, 0.0), 1.0).is_none());

        let triangles: Vec<[Vec3; 3]> = indices
            .chunks_exact(3)
            .map(|t| [0, 1, 2].map(|k| vertices[t[k] as usize]))
            .collect();
        for step in 0..200 {
            let s = step as f64 * 0.037;
            let point = Vec3::new(-1.0 + s, 0.3 * (s * 3.0).sin(), 0.4 * (s * 5.0).cos());
            let brute = triangles
                .iter()
                .map(|triangle| (point - closest_on_triangle(point, triangle)).length())
                .fold(f64::INFINITY, f64::min);
            let found = tree.closest(point, 10.0).expect("closest").distance_m;
            assert!(
                (found - brute).abs() < 1.0e-12,
                "at {point}: {found} vs {brute}"
            );
            let direction = Vec3::new(0.3, -1.0, 0.2 * s.sin()).normalize();
            let brute_ray = triangles
                .iter()
                .filter_map(|triangle| ray_triangle(point, direction, triangle))
                .fold(f64::INFINITY, f64::min);
            let ray = tree
                .ray(point, direction, 10.0)
                .map_or(f64::INFINITY, |hit| hit.0);
            assert!(
                (ray - brute_ray).abs() < 1.0e-12 || ray == brute_ray,
                "ray from {point}: {ray} vs {brute_ray}"
            );
        }
    }
}
