//! Contact candidates between moving collider samples and static colliders.

use crate::mesh::MeshTree;
use rne_ecs::Entity;
use rne_math::{Quat, Vec3};
use rne_physics::{height_field_surface, ColliderShape};
use rne_world::Transform3;
use std::sync::Arc;

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
/// Spheres are exact; a capsule is a row of spheres along its axis; boxes,
/// convex hulls, and triangle meshes contribute their vertices. Planes and
/// height fields are not sampled.
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
        ColliderShape::TriMesh { vertices, .. } => {
            // At most MAX_HULL_SAMPLES vertices, evenly strided.
            let stride = vertices.len().div_ceil(MAX_HULL_SAMPLES).max(1);
            out.extend(vertices.iter().step_by(stride).map(|vertex| Sample {
                center_m: place(*vertex),
                radius_m: 0.0,
            }));
        }
        ColliderShape::Plane { .. } | ColliderShape::HeightField { .. } => {}
    }
}

/// Distance beyond the sample's radius, in meters, past which a triangle mesh
/// reports no closest point.
const MESH_REACH_M: f64 = 1.0;

/// A collider on a fixed or kinematic body, posed in the simulation frame.
#[derive(Clone, Debug)]
pub(crate) struct StaticCollider {
    /// Entity carrying the collider.
    pub entity: Entity,
    /// Collider shape.
    pub shape: ColliderShape,
    /// Collider pose in the simulation frame.
    pub pose: Transform3,
    /// Friction coefficient of its material.
    pub friction: f64,
    /// Trees of the triangle meshes in the shape, from [`mesh_trees`].
    pub meshes: Vec<Arc<MeshTree>>,
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
        query_shape(&self.shape, &self.pose, &self.meshes, center_m, radius_m)
    }
}

/// The tree of the mesh `(vertices, indices)` among `meshes`.
pub(crate) fn find_tree<'a>(
    meshes: &'a [Arc<MeshTree>],
    vertices: &Arc<[Vec3]>,
    indices: &Arc<[u32]>,
) -> Option<&'a MeshTree> {
    meshes
        .iter()
        .find(|tree| tree.is_built_from(vertices, indices))
        .map(|tree| &**tree)
}

/// Appends to `out` a tree for every triangle mesh in `shape` (compound
/// parts included), reusing the trees in `cache` built from the same mesh
/// allocations and adding the new ones to it.
pub(crate) fn mesh_trees(
    shape: &ColliderShape,
    cache: &mut Vec<Arc<MeshTree>>,
    out: &mut Vec<Arc<MeshTree>>,
) {
    match shape {
        ColliderShape::TriMesh { vertices, indices } => {
            let tree = match cache
                .iter()
                .find(|tree| tree.is_built_from(vertices, indices))
            {
                Some(tree) => tree.clone(),
                None => {
                    let tree = Arc::new(MeshTree::build(vertices, indices));
                    cache.push(tree.clone());
                    tree
                }
            };
            out.push(tree);
        }
        ColliderShape::Compound { parts } => {
            for part in parts.iter() {
                mesh_trees(&part.shape, cache, out);
            }
        }
        _ => {}
    }
}

/// [`StaticCollider::query`] for `shape` placed at `pose`.
fn query_shape(
    shape: &ColliderShape,
    pose: &Transform3,
    meshes: &[Arc<MeshTree>],
    center_m: Vec3,
    radius_m: f64,
) -> Option<Hit> {
    let local = pose.rotation.conjugate() * (center_m - pose.translation);
    let (distance, normal_local) = match shape {
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
            let surface = height_field_surface(shape, local.x, local.z)?;
            (
                (local.y - surface.height_m) * surface.normal.y,
                surface.normal,
            )
        }
        ColliderShape::TriMesh { vertices, indices } => {
            let closest =
                find_tree(meshes, vertices, indices)?.closest(local, radius_m + MESH_REACH_M)?;
            // The nearest face's outward normal tells outside from inside.
            let away = local - closest.point_m;
            let side = if away.dot(closest.face_normal) < 0.0 {
                -1.0
            } else {
                1.0
            };
            let normal = match away.try_normalize() {
                Some(direction) if closest.distance_m > 1.0e-12 => direction * side,
                _ => closest.face_normal,
            };
            (side * closest.distance_m, normal)
        }
        ColliderShape::Compound { parts } => {
            return parts
                .iter()
                .filter_map(|part| {
                    query_shape(
                        &part.shape,
                        &pose.mul_transform(&part.local_offset),
                        meshes,
                        center_m,
                        radius_m,
                    )
                })
                .min_by(|a, b| a.gap_m.total_cmp(&b.gap_m));
        }
        ColliderShape::ConvexHull { .. } => return None,
    };
    let normal = pose.rotation * normal_local;
    Some(Hit {
        gap_m: distance - radius_m,
        normal,
        point_m: center_m - normal * radius_m,
    })
}

/// Lateral slack, in meters, within which a corner still counts as over a face.
const FACE_SLACK_M: f64 = 1.0e-4;
/// Extra separation, in meters, an edge-pair axis needs over the best face
/// axis to be chosen, so resting faces keep their face contacts.
const EDGE_AXIS_BIAS_M: f64 = 5.0e-4;
/// Distance, in meters, below which two box contacts are one.
const COINCIDENT_M: f64 = 1.0e-3;
/// First feature index of an edge crossing on the separating faces.
pub(crate) const CROSSING_FEATURES: usize = 8;
/// First feature index of an edge-pair contact.
pub(crate) const EDGE_FEATURES: usize = CROSSING_FEATURES + 16;

/// A contact between two boxes, from [`box_box`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct BoxContact {
    /// Whether the contact point lies on the first box.
    pub on_first: bool,
    /// A corner (below [`CROSSING_FEATURES`], in [`collider_samples`] order),
    /// an edge crossing, or an edge pair (from [`EDGE_FEATURES`]).
    pub feature: usize,
    /// Contact point on its box, in the simulation frame.
    pub point_m: Vec3,
    /// Unit normal from the other box into the point's box.
    pub normal: Vec3,
    /// Signed distance from the other box's surface to the point, in meters.
    pub gap_m: f64,
}

impl BoxContact {
    fn midpoint(&self) -> Vec3 {
        self.point_m - self.normal * (0.5 * self.gap_m)
    }
}

/// A box in the simulation frame.
#[derive(Clone, Copy)]
struct OrientedBox {
    center: Vec3,
    axes: [Vec3; 3],
    half: [f64; 3],
}

impl OrientedBox {
    fn new(half: Vec3, pose: &Transform3) -> Self {
        Self {
            center: pose.translation,
            axes: [
                pose.rotation * Vec3::X,
                pose.rotation * Vec3::Y,
                pose.rotation * Vec3::Z,
            ],
            half: [half.x, half.y, half.z],
        }
    }

    /// Half the box's extent along the unit `axis`.
    fn radius(&self, axis: Vec3) -> f64 {
        (0..3)
            .map(|k| self.half[k] * self.axes[k].dot(axis).abs())
            .sum()
    }

    /// The edge parallel to axis `k` farthest along `direction`.
    fn support_edge(&self, k: usize, direction: Vec3) -> (Vec3, Vec3) {
        let mut middle = self.center;
        for m in (0..3).filter(|m| *m != k) {
            middle += self.axes[m] * self.half[m] * self.axes[m].dot(direction).signum();
        }
        let along = self.axes[k] * self.half[k];
        (middle - along, middle + along)
    }

    /// Corners of the face most along `direction`, in order around it.
    fn face(&self, direction: Vec3) -> [Vec3; 4] {
        let k = (0..3)
            .max_by(|a, b| {
                let a = self.axes[*a].dot(direction).abs();
                let b = self.axes[*b].dot(direction).abs();
                a.total_cmp(&b)
            })
            .unwrap_or(0);
        let sign = self.axes[k].dot(direction).signum();
        let middle = self.center + self.axes[k] * (sign * self.half[k]);
        let (u, v) = ((k + 1) % 3, (k + 2) % 3);
        let (u, v) = (self.axes[u] * self.half[u], self.axes[v] * self.half[v]);
        [
            middle + u + v,
            middle - u + v,
            middle - u - v,
            middle + u - v,
        ]
    }
}

/// Contacts between two boxes within `margin_m`, by the separating-axis test.
///
/// The candidate axes are the six face normals and the nine edge-pair
/// directions; the one with the least penetration (or the largest
/// separation) wins, with face axes preferred within [`EDGE_AXIS_BIAS_M`].
/// On a face axis the contacts are the corners of each box over the other
/// box's face plus the crossings of the two facing faces' edges, so boxes
/// stacked aligned, offset, or crosswise all get the corners of their
/// overlap. On an edge-pair axis the contact is the closest points of the two
/// edges.
pub(crate) fn box_box(
    first_half: Vec3,
    first: &Transform3,
    second_half: Vec3,
    second: &Transform3,
    margin_m: f64,
) -> Vec<BoxContact> {
    let (a, b) = (
        OrientedBox::new(first_half, first),
        OrientedBox::new(second_half, second),
    );
    let offset = a.center - b.center;
    let separation = |axis: Vec3| {
        let normal = if offset.dot(axis) >= 0.0 { axis } else { -axis };
        (
            offset.dot(normal) - a.radius(normal) - b.radius(normal),
            normal,
        )
    };
    let mut face: Option<(f64, Vec3)> = None;
    for axis in a.axes.iter().chain(&b.axes) {
        let (gap, normal) = separation(*axis);
        if face.is_none_or(|(best, _)| gap > best + 1.0e-9) {
            face = Some((gap, normal));
        }
    }
    let Some((face_gap, face_normal)) = face else {
        return Vec::new();
    };
    let mut edge: Option<(f64, Vec3, usize, usize)> = None;
    for i in 0..3 {
        for j in 0..3 {
            let cross = a.axes[i].cross(b.axes[j]);
            let length = cross.length();
            // Parallel edges: the face axes already cover the pair.
            if length < 1.0e-6 {
                continue;
            }
            let (gap, normal) = separation(cross / length);
            if edge.is_none_or(|(best, ..)| gap > best + 1.0e-9) {
                edge = Some((gap, normal, i, j));
            }
        }
    }
    if face_gap > margin_m || edge.is_some_and(|(gap, ..)| gap > margin_m) {
        return Vec::new();
    }
    if let Some((gap, normal, i, j)) = edge.filter(|(gap, ..)| *gap > face_gap + EDGE_AXIS_BIAS_M) {
        return edge_pair(&a, &b, normal, (i, j), margin_m).map_or_else(Vec::new, |point_m| {
            vec![BoxContact {
                on_first: true,
                feature: EDGE_FEATURES + 3 * i + j,
                point_m,
                normal,
                gap_m: gap,
            }]
        });
    }
    let mut contacts = Vec::new();
    // `face_normal` points from the second box into the first.
    corners_over_face(
        true,
        (first_half, first),
        &b,
        face_normal,
        margin_m,
        &mut contacts,
    );
    corners_over_face(
        false,
        (second_half, second),
        &a,
        -face_normal,
        margin_m,
        &mut contacts,
    );
    let (a_face, b_face) = (a.face(-face_normal), b.face(face_normal));
    for (i, j) in (0..4).flat_map(|i| (0..4).map(move |j| (i, j))) {
        let Some((point_m, gap_m)) = crossing(
            (a_face[i], a_face[(i + 1) % 4]),
            (b_face[j], b_face[(j + 1) % 4]),
            face_normal,
        ) else {
            continue;
        };
        let contact = BoxContact {
            on_first: true,
            feature: CROSSING_FEATURES + 4 * i + j,
            point_m,
            normal: face_normal,
            gap_m,
        };
        if gap_m <= margin_m
            && contacts.iter().all(|kept| {
                kept.midpoint().distance_squared(contact.midpoint()) >= COINCIDENT_M * COINCIDENT_M
            })
        {
            contacts.push(contact);
        }
    }
    contacts
}

/// Appends the corners of the box `(half, pose)` that lie over `face_box`'s
/// face toward `toward` (from the face's box into the corner's box) within
/// `margin_m`.
fn corners_over_face(
    on_first: bool,
    (corner_half, corner_pose): (Vec3, &Transform3),
    face_box: &OrientedBox,
    toward: Vec3,
    margin_m: f64,
    out: &mut Vec<BoxContact>,
) {
    let face_offset = face_box.radius(toward);
    let mut samples = Vec::with_capacity(8);
    collider_samples(
        &ColliderShape::Cuboid {
            half_extents_m: corner_half,
        },
        corner_pose,
        &mut samples,
    );
    for (corner, sample) in samples.iter().enumerate() {
        let relative = sample.center_m - face_box.center;
        let gap_m = relative.dot(toward) - face_offset;
        if gap_m > margin_m {
            continue;
        }
        // Over the face: within the box's extent along the other two axes.
        let over_face = face_box
            .axes
            .iter()
            .zip(face_box.half)
            .filter(|(axis, _)| axis.dot(toward).abs() < 0.5)
            .all(|(axis, half)| relative.dot(*axis).abs() <= half + FACE_SLACK_M);
        if over_face {
            out.push(BoxContact {
                on_first,
                feature: corner,
                point_m: sample.center_m,
                normal: toward,
                gap_m,
            });
        }
    }
}

/// Where the edges `first` and `second` cross seen along `normal`: the point
/// on `first` and its distance along `normal` from `second`.
fn crossing(first: (Vec3, Vec3), second: (Vec3, Vec3), normal: Vec3) -> Option<(Vec3, f64)> {
    let flat = |v: Vec3| v - normal * v.dot(normal);
    let (r, s) = (flat(first.1 - first.0), flat(second.1 - second.0));
    let cross = r.cross(s).dot(normal);
    if cross.abs() <= 1.0e-9 * r.length() * s.length() {
        return None;
    }
    let start = flat(second.0 - first.0);
    let t = start.cross(s).dot(normal) / cross;
    let w = start.cross(r).dot(normal) / cross;
    if !(0.0..=1.0).contains(&t) || !(0.0..=1.0).contains(&w) {
        return None;
    }
    let on_first = first.0 + (first.1 - first.0) * t;
    let on_second = second.0 + (second.1 - second.0) * w;
    Some((on_first, (on_first - on_second).dot(normal)))
}

/// The point of the first box's edge `i` closest to the second box's edge
/// `j`, when both closest points lie inside their edges.
fn edge_pair(
    a: &OrientedBox,
    b: &OrientedBox,
    normal: Vec3,
    (i, j): (usize, usize),
    margin_m: f64,
) -> Option<Vec3> {
    let (p0, p1) = a.support_edge(i, -normal);
    let (q0, q1) = b.support_edge(j, normal);
    let (d1, d2, r) = (p1 - p0, q1 - q0, p0 - q0);
    let (aa, bb, cc, dd, ee) = (d1.dot(d1), d1.dot(d2), d2.dot(d2), d1.dot(r), d2.dot(r));
    let denominator = aa * cc - bb * bb;
    if denominator <= 1.0e-12 * aa * cc {
        return None;
    }
    let t = (bb * ee - cc * dd) / denominator;
    let w = (aa * ee - bb * dd) / denominator;
    let inside = |x: f64| (-1.0e-9..=1.0 + 1.0e-9).contains(&x);
    let on_first = p0 + d1 * t;
    (inside(t) && inside(w) && (on_first - (q0 + d2 * w)).dot(normal) <= margin_m)
        .then_some(on_first)
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
            meshes: Vec::new(),
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
    fn box_box_finds_crossing_edges_of_boxes_stacked_crosswise() {
        // A plank along x across a beam along y: no corner of either lies
        // over the other's face, so the contacts are the four edge crossings.
        let beam = Transform3::from_translation_rotation(Vec3::new(0.0, 0.0, 0.05), Quat::IDENTITY);
        let plank =
            Transform3::from_translation_rotation(Vec3::new(0.0, 0.0, 0.152), Quat::IDENTITY);
        let contacts = box_box(
            Vec3::new(0.5, 0.05, 0.05),
            &plank,
            Vec3::new(0.05, 0.5, 0.05),
            &beam,
            0.02,
        );
        assert_eq!(contacts.len(), 4);
        for contact in &contacts {
            assert!(contact.on_first);
            assert!((CROSSING_FEATURES..EDGE_FEATURES).contains(&contact.feature));
            assert_eq!(contact.normal, Vec3::Z);
            assert!((contact.gap_m - 0.002).abs() < 1.0e-12);
            assert!((contact.point_m.x.abs() - 0.05).abs() < 1.0e-12);
            assert!((contact.point_m.y.abs() - 0.05).abs() < 1.0e-12);
            assert!((contact.point_m.z - 0.102).abs() < 1.0e-12);
        }
    }

    #[test]
    fn box_box_finds_the_closest_points_of_two_crossed_edges() {
        // A cube resting on its edge along x, under a cube balanced on its
        // edge along y: the edges meet at one point, which no face test sees;
        // the upper edge is offset along x, and its contact point sits over
        // the lower ridge (y = 0).
        let half = Vec3::splat(0.1);
        let ridge = 0.1 * std::f64::consts::SQRT_2;
        let lower = Transform3::from_translation_rotation(
            Vec3::ZERO,
            Quat::from_rotation_x(std::f64::consts::FRAC_PI_4),
        );
        let upper = Transform3::from_translation_rotation(
            Vec3::new(0.01, -0.02, 2.0 * ridge + 0.003),
            Quat::from_rotation_y(std::f64::consts::FRAC_PI_4),
        );
        let contacts = box_box(half, &upper, half, &lower, 0.02);
        assert_eq!(contacts.len(), 1);
        let contact = contacts[0];
        assert!(contact.on_first && contact.feature >= EDGE_FEATURES);
        assert!((contact.normal - Vec3::Z).length() < 1.0e-12);
        assert!((contact.gap_m - 0.003).abs() < 1.0e-12);
        assert!((contact.point_m - Vec3::new(0.01, 0.0, ridge + 0.003)).length() < 1.0e-12);
        // Lifted clear of the margin: nothing.
        let apart = Transform3::from_translation_rotation(
            upper.translation + Vec3::new(0.0, 0.0, 0.05),
            upper.rotation,
        );
        assert!(box_box(half, &apart, half, &lower, 0.02).is_empty());
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
