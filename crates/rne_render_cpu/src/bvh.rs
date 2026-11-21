//! Bounding volume hierarchy over world-space triangles.

use glam::Vec3;

/// A world-space triangle with its shading attributes.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Triangle {
    pub(crate) v0: Vec3,
    pub(crate) e1: Vec3,
    pub(crate) e2: Vec3,
    pub(crate) n: [Vec3; 3],
    pub(crate) uv: [[f32; 2]; 3],
    /// Index of the scene item the triangle belongs to.
    pub(crate) item: u32,
}

impl Triangle {
    fn bounds(&self) -> Aabb {
        let a = self.v0;
        let b = self.v0 + self.e1;
        let c = self.v0 + self.e2;
        Aabb {
            min: a.min(b).min(c),
            max: a.max(b).max(c),
        }
    }

    fn centroid(&self) -> Vec3 {
        self.v0 + (self.e1 + self.e2) / 3.0
    }

    /// Möller–Trumbore: distance and barycentrics `(u, v)` of a hit in `(t_min, t_max)`.
    #[inline]
    pub(crate) fn intersect(
        &self,
        origin: Vec3,
        dir: Vec3,
        t_min: f32,
        t_max: f32,
    ) -> Option<(f32, f32, f32)> {
        let p = dir.cross(self.e2);
        let det = self.e1.dot(p);
        if det.abs() < 1e-12 {
            return None;
        }
        let inv = 1.0 / det;
        let s = origin - self.v0;
        let u = s.dot(p) * inv;
        if !(0.0..=1.0).contains(&u) {
            return None;
        }
        let q = s.cross(self.e1);
        let v = dir.dot(q) * inv;
        if v < 0.0 || u + v > 1.0 {
            return None;
        }
        let t = self.e2.dot(q) * inv;
        (t > t_min && t < t_max).then_some((t, u, v))
    }
}

/// Axis-aligned bounding box.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Aabb {
    pub(crate) min: Vec3,
    pub(crate) max: Vec3,
}

impl Aabb {
    pub(crate) const EMPTY: Self = Self {
        min: Vec3::splat(f32::INFINITY),
        max: Vec3::splat(f32::NEG_INFINITY),
    };

    pub(crate) fn union(self, other: Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    /// Entry distance of a ray, if it crosses the box within `(t_min, t_max)`.
    ///
    /// `inv_dir` is `dir.recip()`. An axis the ray does not move along
    /// (infinite reciprocal) only checks that the origin lies inside the slab;
    /// computing `0 * inf` there would give NaN and drop rays that run exactly
    /// along a face.
    #[inline]
    pub(crate) fn hit(&self, origin: Vec3, inv_dir: Vec3, t_min: f32, t_max: f32) -> Option<f32> {
        let (near, far) = slab_interval(self.min, self.max, origin, inv_dir, t_min, t_max)?;
        Some(near).filter(|_| near <= far)
    }
}

/// Parameter interval in which a ray is inside the box `[min, max]`, clipped
/// to `(t_min, t_max)`, with the axis that sets the entry; see [`Aabb::hit`].
#[inline]
pub(crate) fn slab_interval(
    min: Vec3,
    max: Vec3,
    origin: Vec3,
    inv_dir: Vec3,
    t_min: f32,
    t_max: f32,
) -> Option<(f32, f32)> {
    let mut near = t_min;
    let mut far = t_max;
    for axis in 0..3 {
        let inv = inv_dir[axis];
        if !inv.is_finite() {
            if origin[axis] < min[axis] || origin[axis] > max[axis] {
                return None;
            }
            continue;
        }
        let a = (min[axis] - origin[axis]) * inv;
        let b = (max[axis] - origin[axis]) * inv;
        near = near.max(a.min(b));
        far = far.min(a.max(b));
    }
    (near <= far).then_some((near, far))
}

#[derive(Clone, Copy, Debug)]
struct Node {
    bounds: Aabb,
    /// Leaf: first triangle; inner: index of the right child (left is next).
    start_or_right: u32,
    /// Triangle count for a leaf, zero for an inner node.
    count: u32,
}

/// Flattened median-split BVH. Triangles are reordered on build.
#[derive(Debug, Default)]
pub(crate) struct Bvh {
    nodes: Vec<Node>,
    pub(crate) triangles: Vec<Triangle>,
}

const LEAF_SIZE: usize = 4;

impl Bvh {
    pub(crate) fn build(mut triangles: Vec<Triangle>) -> Self {
        let mut nodes = Vec::with_capacity(triangles.len().max(1) * 2 / LEAF_SIZE + 1);
        if !triangles.is_empty() {
            let len = triangles.len();
            build_node(&mut nodes, &mut triangles, 0, len);
        }
        Self { nodes, triangles }
    }

    /// Closest hit: `(t, triangle index, u, v)`.
    pub(crate) fn closest(
        &self,
        origin: Vec3,
        dir: Vec3,
        t_min: f32,
        t_max: f32,
    ) -> Option<(f32, usize, f32, f32)> {
        if self.nodes.is_empty() {
            return None;
        }
        let inv_dir = dir.recip();
        let mut best: Option<(f32, usize, f32, f32)> = None;
        let mut limit = t_max;
        let mut stack = [0_u32; 64];
        let mut top = 1;
        while top > 0 {
            top -= 1;
            let node = &self.nodes[stack[top] as usize];
            if node.bounds.hit(origin, inv_dir, t_min, limit).is_none() {
                continue;
            }
            if node.count > 0 {
                let start = node.start_or_right as usize;
                for index in start..start + node.count as usize {
                    if let Some((t, u, v)) =
                        self.triangles[index].intersect(origin, dir, t_min, limit)
                    {
                        limit = t;
                        best = Some((t, index, u, v));
                    }
                }
            } else {
                let left = stack[top] + 1;
                let right = node.start_or_right;
                // Visit the nearer child first.
                let left_t = self.nodes[left as usize]
                    .bounds
                    .hit(origin, inv_dir, t_min, limit);
                let right_t = self.nodes[right as usize]
                    .bounds
                    .hit(origin, inv_dir, t_min, limit);
                match (left_t, right_t) {
                    (Some(l), Some(r)) => {
                        let (near, far) = if l <= r { (left, right) } else { (right, left) };
                        stack[top] = far;
                        stack[top + 1] = near;
                        top += 2;
                    }
                    (Some(_), None) => {
                        stack[top] = left;
                        top += 1;
                    }
                    (None, Some(_)) => {
                        stack[top] = right;
                        top += 1;
                    }
                    (None, None) => {}
                }
            }
        }
        best
    }

    /// Whether anything blocks the ray within `(t_min, t_max)`.
    pub(crate) fn occluded(&self, origin: Vec3, dir: Vec3, t_min: f32, t_max: f32) -> bool {
        if self.nodes.is_empty() {
            return false;
        }
        let inv_dir = dir.recip();
        let mut stack = [0_u32; 64];
        let mut top = 1;
        while top > 0 {
            top -= 1;
            let index = stack[top];
            let node = &self.nodes[index as usize];
            if node.bounds.hit(origin, inv_dir, t_min, t_max).is_none() {
                continue;
            }
            if node.count > 0 {
                let start = node.start_or_right as usize;
                if self.triangles[start..start + node.count as usize]
                    .iter()
                    .any(|tri| tri.intersect(origin, dir, t_min, t_max).is_some())
                {
                    return true;
                }
            } else {
                stack[top] = index + 1;
                stack[top + 1] = node.start_or_right;
                top += 2;
            }
        }
        false
    }
}

fn build_node(nodes: &mut Vec<Node>, triangles: &mut [Triangle], start: usize, end: usize) -> u32 {
    let slice = &mut triangles[start..end];
    let bounds = slice
        .iter()
        .fold(Aabb::EMPTY, |acc, tri| acc.union(tri.bounds()));
    let index = nodes.len() as u32;
    // Depth stays far below the traversal stack: median splits halve the count.
    if slice.len() <= LEAF_SIZE {
        nodes.push(Node {
            bounds,
            start_or_right: start as u32,
            count: slice.len() as u32,
        });
        return index;
    }
    let centroid_bounds = slice.iter().fold(Aabb::EMPTY, |acc, tri| {
        let c = tri.centroid();
        acc.union(Aabb { min: c, max: c })
    });
    let extent = centroid_bounds.max - centroid_bounds.min;
    let axis = if extent.x >= extent.y && extent.x >= extent.z {
        0
    } else if extent.y >= extent.z {
        1
    } else {
        2
    };
    let mid = slice.len() / 2;
    slice.select_nth_unstable_by(mid, |a, b| {
        a.centroid()[axis]
            .total_cmp(&b.centroid()[axis])
            .then_with(|| a.item.cmp(&b.item))
    });
    nodes.push(Node {
        bounds,
        start_or_right: 0,
        count: 0,
    });
    build_node(nodes, triangles, start, start + mid);
    let right = build_node(nodes, triangles, start + mid, end);
    nodes[index as usize].start_or_right = right;
    index
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tri(a: Vec3, b: Vec3, c: Vec3, item: u32) -> Triangle {
        Triangle {
            v0: a,
            e1: b - a,
            e2: c - a,
            n: [Vec3::Z; 3],
            uv: [[0.0; 2]; 3],
            item,
        }
    }

    #[test]
    fn bvh_matches_brute_force() {
        // A grid of small triangles at varying depths.
        let mut triangles = Vec::new();
        for i in 0..40 {
            for j in 0..40 {
                let x = i as f32 * 0.25;
                let y = j as f32 * 0.25;
                let z = ((i * 7 + j * 13) % 11) as f32 * 0.1;
                triangles.push(tri(
                    Vec3::new(x, y, z),
                    Vec3::new(x + 0.2, y, z),
                    Vec3::new(x, y + 0.2, z),
                    (i * 40 + j) as u32,
                ));
            }
        }
        let brute = triangles.clone();
        let bvh = Bvh::build(triangles);
        for k in 0..500 {
            let origin = Vec3::new((k % 37) as f32 * 0.27, (k % 23) as f32 * 0.41, 5.0);
            let dir = Vec3::new(0.01 * (k % 5) as f32, -0.02 * (k % 3) as f32, -1.0);
            let expected = brute
                .iter()
                .filter_map(|t| {
                    t.intersect(origin, dir, 0.0, f32::INFINITY)
                        .map(|h| (h.0, t.item))
                })
                .min_by(|a, b| a.0.total_cmp(&b.0));
            let got = bvh
                .closest(origin, dir, 0.0, f32::INFINITY)
                .map(|(t, index, _, _)| (t, bvh.triangles[index].item));
            assert_eq!(got.map(|g| g.0), expected.map(|e| e.0), "ray {k}");
            assert_eq!(
                bvh.occluded(origin, dir, 0.0, f32::INFINITY),
                expected.is_some()
            );
        }
    }
}
