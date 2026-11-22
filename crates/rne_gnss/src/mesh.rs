//! Triangle-mesh buildings for GNSS ray tracing (for example PLATEAU LOD2
//! city models).
//!
//! A [`TriangleMesh`] holds triangles with outward normals, a material and the
//! id of the object (building) they belong to. A bounding volume hierarchy
//! (median split on the longest centroid axis) answers closest-hit and
//! any-hit ray queries. The mesh also derives the edges that diffract GNSS
//! signals: boundary edges and convex creases (roof outlines, building
//! corners) of each object. Construction and queries visit triangles in a
//! fixed order, so results are deterministic.

use crate::scene::SurfaceMaterial;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Triangles per BVH leaf.
const LEAF_SIZE: usize = 4;
/// Two faces meeting at an edge closer than this to coplanar (cos of the
/// angle between normals) do not form a diffracting crease.
const CREASE_COS: f64 = 0.94;
/// Vertices closer than this are welded when finding shared edges, meters.
const WELD_M: f64 = 1.0e-3;

/// One triangle of a mesh.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MeshTriangle {
    /// Vertices in ENU meters, counter-clockwise seen from outside.
    pub vertices: [Vec3; 3],
    /// Outward unit normal.
    pub normal: Vec3,
    /// Surface material.
    pub material: SurfaceMaterial,
    /// Object (building) the triangle belongs to.
    pub object: u32,
}

impl MeshTriangle {
    /// A triangle with its normal from the vertex winding; `None` when
    /// degenerate.
    pub fn new(vertices: [Vec3; 3], material: SurfaceMaterial, object: u32) -> Option<Self> {
        let normal = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
        let length = normal.length();
        (length > 1.0e-9 && length.is_finite()).then(|| Self {
            vertices,
            normal: normal / length,
            material,
            object,
        })
    }

    /// Area in m².
    pub fn area_m2(&self) -> f64 {
        0.5 * (self.vertices[1] - self.vertices[0])
            .cross(self.vertices[2] - self.vertices[0])
            .length()
    }

    /// Centroid in ENU meters.
    pub fn centroid(&self) -> Vec3 {
        (self.vertices[0] + self.vertices[1] + self.vertices[2]) / 3.0
    }

    /// Ray parameter of the hit (Möller–Trumbore, two-sided), if any in
    /// `(0, max_t)`.
    pub fn intersect(&self, origin_m: Vec3, direction: Vec3, max_t: f64) -> Option<f64> {
        let [v0, v1, v2] = self.vertices;
        let (e1, e2) = (v1 - v0, v2 - v0);
        let p = direction.cross(e2);
        let det = e1.dot(p);
        if det.abs() < 1.0e-12 {
            return None;
        }
        let inv = 1.0 / det;
        let s = origin_m - v0;
        let u = s.dot(p) * inv;
        if !(-1.0e-9..=1.0 + 1.0e-9).contains(&u) {
            return None;
        }
        let q = s.cross(e1);
        let v = direction.dot(q) * inv;
        if v < -1.0e-9 || u + v > 1.0 + 1.0e-9 {
            return None;
        }
        let t = e2.dot(q) * inv;
        (t > 0.0 && t < max_t).then_some(t)
    }

    /// Whether a point on the triangle's plane lies inside it (with a small
    /// tolerance).
    pub fn contains_in_plane(&self, point_m: Vec3) -> bool {
        let [v0, v1, v2] = self.vertices;
        let n = self.normal;
        [(v0, v1), (v1, v2), (v2, v0)]
            .iter()
            .all(|&(a, b)| (b - a).cross(point_m - a).dot(n) >= -1.0e-7)
    }
}

/// An edge that diffracts signals: a roof outline, a building corner, or
/// another convex crease of one object.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DiffractionEdge {
    /// One end in ENU meters.
    pub start_m: Vec3,
    /// Other end in ENU meters.
    pub end_m: Vec3,
    /// Unit direction pointing away from the object at the edge (the mean of
    /// the adjacent faces' normals).
    pub outward: Vec3,
    /// Object the edge belongs to.
    pub object: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
struct BvhNode {
    min_m: Vec3,
    max_m: Vec3,
    /// Leaves: first index into `order`. Interior: index of the left child
    /// (the right child follows it).
    first: u32,
    /// Number of triangles in a leaf; 0 for interior nodes.
    count: u32,
}

/// Triangle geometry with a BVH and diffraction edges.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct TriangleMesh {
    triangles: Vec<MeshTriangle>,
    nodes: Vec<BvhNode>,
    order: Vec<u32>,
    edges: Vec<DiffractionEdge>,
}

impl TriangleMesh {
    /// Builds the BVH and diffraction edges.
    pub fn new(triangles: Vec<MeshTriangle>) -> Self {
        let mut mesh = Self {
            order: (0..triangles.len() as u32).collect(),
            triangles,
            nodes: Vec::new(),
            edges: Vec::new(),
        };
        if !mesh.triangles.is_empty() {
            let centroids: Vec<Vec3> = mesh.triangles.iter().map(MeshTriangle::centroid).collect();
            mesh.nodes.push(BvhNode {
                min_m: Vec3::ZERO,
                max_m: Vec3::ZERO,
                first: 0,
                count: 0,
            });
            let count = mesh.triangles.len();
            mesh.build(0, 0, count, &centroids);
        }
        mesh.edges = diffraction_edges(&mesh.triangles);
        mesh
    }

    /// The triangles, in construction order.
    pub fn triangles(&self) -> &[MeshTriangle] {
        &self.triangles
    }

    /// The diffracting edges.
    pub fn diffraction_edges(&self) -> &[DiffractionEdge] {
        &self.edges
    }

    /// Bounds of all triangles, or `None` when empty.
    pub fn bounds(&self) -> Option<(Vec3, Vec3)> {
        self.nodes.first().map(|n| (n.min_m, n.max_m))
    }

    fn build(&mut self, node: usize, start: usize, end: usize, centroids: &[Vec3]) {
        let (mut lo, mut hi) = (Vec3::splat(f64::INFINITY), Vec3::splat(f64::NEG_INFINITY));
        let (mut clo, mut chi) = (lo, hi);
        for &index in &self.order[start..end] {
            for v in self.triangles[index as usize].vertices {
                lo = lo.min(v);
                hi = hi.max(v);
            }
            clo = clo.min(centroids[index as usize]);
            chi = chi.max(centroids[index as usize]);
        }
        self.nodes[node].min_m = lo;
        self.nodes[node].max_m = hi;
        let extent = chi - clo;
        if end - start <= LEAF_SIZE || extent.max_element() < 1.0e-9 {
            self.nodes[node].first = start as u32;
            self.nodes[node].count = (end - start) as u32;
            return;
        }
        let axis = if extent.x >= extent.y && extent.x >= extent.z {
            0
        } else if extent.y >= extent.z {
            1
        } else {
            2
        };
        let mid = (start + end) / 2;
        self.order[start..end].sort_by(|&a, &b| {
            centroids[a as usize][axis]
                .total_cmp(&centroids[b as usize][axis])
                .then(a.cmp(&b))
        });
        let left = self.nodes.len();
        let empty = BvhNode {
            min_m: Vec3::ZERO,
            max_m: Vec3::ZERO,
            first: 0,
            count: 0,
        };
        self.nodes.push(empty);
        self.nodes.push(empty);
        self.nodes[node].first = left as u32;
        self.nodes[node].count = 0;
        self.build(left, start, mid, centroids);
        self.build(left + 1, mid, end, centroids);
    }

    /// Closest hit: `(distance, triangle index)`.
    pub fn raycast(&self, origin_m: Vec3, direction: Vec3, max_t: f64) -> Option<(f64, usize)> {
        let mut best: Option<(f64, usize)> = None;
        self.visit(origin_m, direction, max_t, &mut |index, limit| {
            let tri = &self.triangles[index];
            if let Some(t) = tri.intersect(origin_m, direction, limit) {
                if best.is_none_or(|(bt, bi)| t < bt || (t == bt && index < bi)) {
                    best = Some((t, index));
                }
            }
            (best.map_or(limit, |(t, _)| t), false)
        });
        best
    }

    /// Whether any triangle is hit before `max_t`.
    pub fn occluded(&self, origin_m: Vec3, direction: Vec3, max_t: f64) -> bool {
        let mut hit = false;
        self.visit(origin_m, direction, max_t, &mut |index, limit| {
            hit = self.triangles[index]
                .intersect(origin_m, direction, limit)
                .is_some();
            (limit, hit)
        });
        hit
    }

    /// Whether a point is inside a closed object: the first surface straight
    /// above it is seen from behind.
    pub fn contains(&self, point_m: Vec3) -> bool {
        self.raycast(point_m, Vec3::Z, f64::INFINITY)
            .is_some_and(|(_, index)| self.triangles[index].normal.z > 0.0)
    }

    /// Visits leaf triangles along a ray. The callback gets the triangle
    /// index and current limit and returns the new limit and whether to stop.
    fn visit(
        &self,
        origin_m: Vec3,
        direction: Vec3,
        max_t: f64,
        f: &mut dyn FnMut(usize, f64) -> (f64, bool),
    ) {
        if self.nodes.is_empty() {
            return;
        }
        let inv = Vec3::ONE / direction;
        let mut limit = max_t;
        let mut stack = vec![0_usize];
        while let Some(node) = stack.pop() {
            let n = &self.nodes[node];
            if !slab(n.min_m, n.max_m, origin_m, inv, limit) {
                continue;
            }
            if n.count > 0 {
                for &index in &self.order[n.first as usize..(n.first + n.count) as usize] {
                    let (new_limit, stop) = f(index as usize, limit);
                    if stop {
                        return;
                    }
                    limit = new_limit;
                }
            } else {
                // Visit the nearer child first.
                let (l, r) = (n.first as usize, n.first as usize + 1);
                let near_left = (self.nodes[l].min_m + self.nodes[l].max_m - origin_m * 2.0)
                    .dot(direction)
                    <= (self.nodes[r].min_m + self.nodes[r].max_m - origin_m * 2.0).dot(direction);
                if near_left {
                    stack.push(r);
                    stack.push(l);
                } else {
                    stack.push(l);
                    stack.push(r);
                }
            }
        }
    }
}

/// Ray / box overlap on `[0, limit]` by the slab method.
fn slab(min_m: Vec3, max_m: Vec3, origin_m: Vec3, inv: Vec3, limit: f64) -> bool {
    let (mut t0, mut t1) = (0.0_f64, limit);
    for axis in 0..3 {
        let (mut a, mut b) = (
            (min_m[axis] - origin_m[axis]) * inv[axis],
            (max_m[axis] - origin_m[axis]) * inv[axis],
        );
        if a.is_nan() || b.is_nan() {
            // Ray parallel to and on the slab boundary.
            if origin_m[axis] < min_m[axis] || origin_m[axis] > max_m[axis] {
                return false;
            }
            continue;
        }
        if a > b {
            std::mem::swap(&mut a, &mut b);
        }
        t0 = t0.max(a);
        t1 = t1.min(b);
        if t0 > t1 {
            return false;
        }
    }
    true
}

fn weld(v: Vec3) -> (i64, i64, i64) {
    (
        (v.x / WELD_M).round() as i64,
        (v.y / WELD_M).round() as i64,
        (v.z / WELD_M).round() as i64,
    )
}

/// Boundary edges and convex creases of each object.
fn diffraction_edges(triangles: &[MeshTriangle]) -> Vec<DiffractionEdge> {
    type Key = (u32, (i64, i64, i64), (i64, i64, i64));
    let mut faces: BTreeMap<Key, Vec<usize>> = BTreeMap::new();
    for (index, tri) in triangles.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (weld(tri.vertices[k]), weld(tri.vertices[(k + 1) % 3]));
            if a == b {
                continue;
            }
            let key = (tri.object, a.min(b), a.max(b));
            faces.entry(key).or_default().push(index);
        }
    }
    let mut edges = Vec::new();
    for ((object, a, b), adjacent) in faces {
        let to_vec = |k: (i64, i64, i64)| Vec3::new(k.0 as f64, k.1 as f64, k.2 as f64) * WELD_M;
        let (start, end) = (to_vec(a), to_vec(b));
        let outward = match adjacent.as_slice() {
            [single] => triangles[*single].normal,
            [f, g] => {
                let (tf, tg) = (&triangles[*f], &triangles[*g]);
                if tf.normal.dot(tg.normal) > CREASE_COS {
                    continue;
                }
                // Convex when the other face lies behind each face's plane.
                if (tg.centroid() - start).dot(tf.normal) > 1.0e-6 {
                    continue;
                }
                (tf.normal + tg.normal).normalize_or_zero()
            }
            _ => continue,
        };
        if outward == Vec3::ZERO {
            continue;
        }
        edges.push(DiffractionEdge {
            start_m: start,
            end_m: end,
            outward,
            object,
        });
    }
    edges
}

/// Triangulates a closed box into an object (for tests and simple scenes).
pub fn box_triangles(
    min_m: Vec3,
    max_m: Vec3,
    material: SurfaceMaterial,
    object: u32,
) -> Vec<MeshTriangle> {
    let c = |x: bool, y: bool, z: bool| {
        Vec3::new(
            if x { max_m.x } else { min_m.x },
            if y { max_m.y } else { min_m.y },
            if z { max_m.z } else { min_m.z },
        )
    };
    // Each face as a counter-clockwise quad seen from outside.
    let quads = [
        [
            c(true, false, false),
            c(true, true, false),
            c(true, true, true),
            c(true, false, true),
        ],
        [
            c(false, true, false),
            c(false, false, false),
            c(false, false, true),
            c(false, true, true),
        ],
        [
            c(true, true, false),
            c(false, true, false),
            c(false, true, true),
            c(true, true, true),
        ],
        [
            c(false, false, false),
            c(true, false, false),
            c(true, false, true),
            c(false, false, true),
        ],
        [
            c(false, false, true),
            c(true, false, true),
            c(true, true, true),
            c(false, true, true),
        ],
        [
            c(false, true, false),
            c(true, true, false),
            c(true, false, false),
            c(false, false, false),
        ],
    ];
    quads
        .iter()
        .flat_map(|q| {
            [
                MeshTriangle::new([q[0], q[1], q[2]], material, object),
                MeshTriangle::new([q[0], q[2], q[3]], material, object),
            ]
        })
        .flatten()
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn cube() -> TriangleMesh {
        TriangleMesh::new(box_triangles(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(10.0, 20.0, 30.0),
            SurfaceMaterial::CONCRETE,
            7,
        ))
    }

    #[test]
    fn box_triangles_face_outward() {
        let mesh = cube();
        assert_eq!(mesh.triangles().len(), 12);
        let center = Vec3::new(5.0, 10.0, 15.0);
        for tri in mesh.triangles() {
            assert!((tri.centroid() - center).dot(tri.normal) > 0.0, "{tri:?}");
        }
    }

    #[test]
    fn closest_hit_and_occlusion_match_the_box() {
        let mesh = cube();
        let (t, index) = mesh
            .raycast(Vec3::new(-5.0, 10.0, 10.0), Vec3::X, 100.0)
            .unwrap();
        assert_relative_eq!(t, 5.0, epsilon = 1.0e-9);
        assert_relative_eq!(mesh.triangles()[index].normal.x, -1.0, epsilon = 1.0e-12);
        assert!(mesh
            .raycast(Vec3::new(-5.0, 10.0, 10.0), Vec3::X, 4.0)
            .is_none());
        assert!(mesh.occluded(Vec3::new(-5.0, 10.0, 10.0), Vec3::X, 6.0));
        assert!(!mesh.occluded(Vec3::new(-5.0, 10.0, 40.0), Vec3::X, 100.0));
        assert!(mesh.contains(Vec3::new(5.0, 10.0, 15.0)));
        assert!(!mesh.contains(Vec3::new(-1.0, 10.0, 15.0)));
        assert!(!mesh.contains(Vec3::new(5.0, 10.0, 31.0)));
    }

    #[test]
    fn bvh_agrees_with_brute_force_on_many_boxes() {
        let mut triangles = Vec::new();
        for i in 0..40 {
            let x = (i % 8) as f64 * 15.0;
            let y = (i / 8) as f64 * 15.0;
            let h = 5.0 + (i * 7 % 13) as f64;
            triangles.extend(box_triangles(
                Vec3::new(x, y, 0.0),
                Vec3::new(x + 9.0, y + 9.0, h),
                SurfaceMaterial::GLASS,
                i,
            ));
        }
        let mesh = TriangleMesh::new(triangles.clone());
        for k in 0..200 {
            let a = k as f64 * 0.731;
            let origin = Vec3::new(-10.0 + (k % 17) as f64 * 7.0, -10.0, 1.0 + (k % 5) as f64);
            let direction =
                Vec3::new(a.cos(), a.sin().abs() + 0.2, 0.05 * (k % 3) as f64).normalize();
            let brute = triangles
                .iter()
                .filter_map(|t| t.intersect(origin, direction, f64::INFINITY))
                .fold(f64::INFINITY, f64::min);
            let bvh = mesh
                .raycast(origin, direction, f64::INFINITY)
                .map_or(f64::INFINITY, |h| h.0);
            assert_eq!(brute.is_finite(), bvh.is_finite());
            if brute.is_finite() {
                assert_relative_eq!(brute, bvh, epsilon = 1.0e-9);
            }
        }
    }

    #[test]
    fn a_box_has_twelve_diffraction_edges_and_coplanar_diagonals_are_not_edges() {
        let mesh = cube();
        let edges = mesh.diffraction_edges();
        assert_eq!(edges.len(), 12, "{edges:?}");
        // A roof edge points up and out.
        let roof_east = edges
            .iter()
            .find(|e| e.start_m.z == 30.0 && e.end_m.z == 30.0 && e.start_m.x == 10.0)
            .unwrap();
        assert!(roof_east.outward.x > 0.5 && roof_east.outward.z > 0.5);
        assert_eq!(roof_east.object, 7);
    }
}
