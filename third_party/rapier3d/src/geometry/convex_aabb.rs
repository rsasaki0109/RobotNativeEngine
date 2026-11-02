//! RNE patch: bounding boxes of large convex polyhedra by hill climbing.
//!
//! Parry computes a convex polyhedron's world-space AABB by transforming
//! every vertex (`point_cloud_aabb`). For hulls of scanned meshes (hundreds to
//! thousands of vertices) the broad phase's two AABBs per collider and step
//! then dominate a walking robot's step. Each extreme coordinate is attained
//! at a vertex that is a local maximum on the hull's edge graph, so walking
//! that graph from the previous step's extreme vertex finds it in a few
//! moves. Every candidate is transformed with the same isometry product as
//! `point_cloud_aabb`, so the bounds are the same floating-point values
//! whenever the walk reaches the vertex the full scan picks (it can stop one
//! rounding error short only where neighbouring vertices tie to within it).

use crate::math::{Isometry, Point, Real};
use parry::bounding_volume::Aabb;
use parry::shape::{ConvexPolyhedron, Shape};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex, PoisonError};

/// Hulls with fewer vertices keep Parry's full scan.
pub(crate) const MIN_HILL_CLIMB_VERTICES: usize = 64;

/// Vertex adjacency of a convex polyhedron, in compressed rows.
#[derive(Debug)]
pub(crate) struct HullGraph {
    offsets: Vec<u32>,
    neighbors: Vec<u32>,
}

impl HullGraph {
    fn new(polyhedron: &ConvexPolyhedron) -> Self {
        let count = polyhedron.points().len();
        let mut degree = vec![0u32; count];
        for edge in polyhedron.edges() {
            degree[edge.vertices[0] as usize] += 1;
            degree[edge.vertices[1] as usize] += 1;
        }
        let mut offsets = Vec::with_capacity(count + 1);
        offsets.push(0);
        for d in &degree {
            offsets.push(offsets.last().unwrap() + d);
        }
        let mut fill: Vec<u32> = offsets[..count].to_vec();
        let mut neighbors = vec![0u32; *offsets.last().unwrap() as usize];
        for edge in polyhedron.edges() {
            let (a, b) = (edge.vertices[0], edge.vertices[1]);
            neighbors[fill[a as usize] as usize] = b;
            fill[a as usize] += 1;
            neighbors[fill[b as usize] as usize] = a;
            fill[b as usize] += 1;
        }
        Self { offsets, neighbors }
    }

    fn neighbors(&self, vertex: u32) -> &[u32] {
        let start = self.offsets[vertex as usize] as usize;
        let end = self.offsets[vertex as usize + 1] as usize;
        &self.neighbors[start..end]
    }
}

/// A collider's hull graph and the vertices that bounded its last AABB.
#[derive(Debug)]
pub(crate) struct ConvexAabbHint {
    graph: Arc<HullGraph>,
    /// Extreme vertices: min x, max x, min y, max y, min z, max z.
    warm: [AtomicU32; 6],
    /// The last pose asked for and its AABB: the broad phase often asks
    /// twice for an unchanged pose within a step.
    last: Mutex<Option<(Isometry<Real>, Aabb)>>,
}

impl Clone for ConvexAabbHint {
    fn clone(&self) -> Self {
        Self {
            graph: self.graph.clone(),
            warm: std::array::from_fn(|i| AtomicU32::new(self.warm[i].load(Ordering::Relaxed))),
            last: Mutex::new(*self.last.lock().unwrap_or_else(PoisonError::into_inner)),
        }
    }
}

/// Bitwise pose equality, so a cached AABB is only reused for the very same input.
fn same_pose(a: &Isometry<Real>, b: &Isometry<Real>) -> bool {
    a.translation
        .vector
        .iter()
        .chain(a.rotation.coords.iter())
        .zip(b.translation.vector.iter().chain(b.rotation.coords.iter()))
        .all(|(x, y)| x.to_bits() == y.to_bits())
}

impl ConvexAabbHint {
    /// A hint for `shape` if it is a convex polyhedron worth hill climbing.
    pub(crate) fn for_shape(shape: &dyn Shape) -> Option<Self> {
        let polyhedron = shape.as_convex_polyhedron()?;
        if polyhedron.points().len() < MIN_HILL_CLIMB_VERTICES {
            return None;
        }
        Some(Self {
            graph: Arc::new(HullGraph::new(polyhedron)),
            warm: std::array::from_fn(|_| AtomicU32::new(0)),
            last: Mutex::new(None),
        })
    }

    /// The AABB of `shape` at `pos`; `shape` must be the polyhedron this hint
    /// was built for.
    pub(crate) fn aabb(&self, shape: &dyn Shape, pos: &Isometry<Real>) -> Aabb {
        let Some(polyhedron) = shape.as_convex_polyhedron() else {
            return shape.compute_aabb(pos);
        };
        let mut last = self.last.lock().unwrap_or_else(PoisonError::into_inner);
        if let Some((pose, aabb)) = *last {
            if same_pose(&pose, pos) {
                return aabb;
            }
        }
        let points = polyhedron.points();
        let world = |vertex: u32| -> Point<Real> { pos * points[vertex as usize] };
        let mut mins = Point::origin();
        let mut maxs = Point::origin();
        for axis in 0..3 {
            for (slot, sign) in [(2 * axis, -1.0 as Real), (2 * axis + 1, 1.0)] {
                let mut vertex = self.warm[slot].load(Ordering::Relaxed);
                let mut best = world(vertex)[axis];
                loop {
                    let mut moved = false;
                    for &next in self.graph.neighbors(vertex) {
                        let value = world(next)[axis];
                        if sign * value > sign * best {
                            best = value;
                            vertex = next;
                            moved = true;
                        }
                    }
                    if !moved {
                        break;
                    }
                }
                self.warm[slot].store(vertex, Ordering::Relaxed);
                if sign < 0.0 {
                    mins[axis] = best;
                } else {
                    maxs[axis] = best;
                }
            }
        }
        let aabb = Aabb::new(mins, maxs);
        *last = Some((*pos, aabb));
        aabb
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vector;
    use parry::bounding_volume::details::point_cloud_aabb;
    use parry::shape::SharedShape;

    fn lcg(state: &mut u64) -> Real {
        *state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((*state >> 40) as Real) / ((1u64 << 24) as Real) * 2.0 - 1.0
    }

    #[test]
    fn rne_hill_climbed_aabb_matches_the_full_scan() {
        // An irregular rounded blob, like a scanned link mesh.
        let mut state = 7u64;
        let points: Vec<Point<Real>> = (0..3000)
            .map(|_| {
                let v = Vector::new(lcg(&mut state), lcg(&mut state), lcg(&mut state));
                let v = v / v.norm().max(1.0e-3);
                Point::from(Vector::new(0.06 * v.x, 0.03 * v.y, 0.12 * v.z))
            })
            .collect();
        let shape = SharedShape::convex_hull(&points).unwrap();
        let hint = ConvexAabbHint::for_shape(&*shape).expect("large hull");
        let polyhedron = shape.as_convex_polyhedron().unwrap();
        let mut mismatches = 0;
        let mut pose = Isometry::identity();
        for step in 0..2000 {
            // A tumbling, translating pose, as a link moves between steps.
            let axis = Vector::new(lcg(&mut state), lcg(&mut state), lcg(&mut state));
            pose = Isometry::new(
                pose.translation.vector + axis * 1.0e-3,
                pose.rotation.scaled_axis() + axis * 0.02,
            );
            let fast = hint.aabb(&*shape, &pose);
            let full = point_cloud_aabb(&pose, polyhedron.points());
            if fast != full {
                mismatches += 1;
            }
            if step % 500 == 0 {
                // Cold starts from arbitrary vertices find the same bounds.
                for slot in &hint.warm {
                    slot.store((step as u32 * 37) % polyhedron.points().len() as u32, Ordering::Relaxed);
                }
            }
        }
        assert_eq!(mismatches, 0);
    }

    #[test]
    fn rne_small_and_non_polyhedral_shapes_keep_the_full_scan() {
        let cube = SharedShape::cuboid(0.1, 0.2, 0.3);
        assert!(ConvexAabbHint::for_shape(&*cube).is_none());
        let tetra = SharedShape::convex_hull(&[
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 1.0),
        ])
        .unwrap();
        assert!(ConvexAabbHint::for_shape(&*tetra).is_none());
    }
}
