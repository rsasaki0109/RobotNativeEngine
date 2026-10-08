//! One step of every assembly in a world, with shared contacts.

use crate::assembly::{Assembly, ContactRecord};
use crate::collide::{box_box, StaticCollider};
use rne_dynamics::{
    contact_frame, contact_step_coupled, ContactAnchor, ContactStepConfig, CoupledContact,
};
use rne_ecs::Entity;
use rne_math::Vec3;
use rne_physics::{ColliderShape, CollisionGroups, PhysicsError};
use rne_world::Transform3;
use std::collections::HashMap;

/// Warm-start key: sample body entity, sample index, touched entity.
pub(crate) type WarmKey = (u32, usize, u32);

/// A contact candidate before it is split into islands.
struct Candidate {
    contact: CoupledContact,
    key: WarmKey,
    record: ContactRecord,
    /// Midpoint between the two surfaces in the simulation frame, the same
    /// whichever body's sample found the contact.
    place_m: Vec3,
}

/// The collider of one dynamic body, posed in the simulation frame.
struct MovingCollider {
    assembly: usize,
    body: usize,
    collider: StaticCollider,
    groups: CollisionGroups,
    /// Bounding radius about the collider origin, in meters.
    radius_m: f64,
}

fn interacts(a: CollisionGroups, b: CollisionGroups) -> bool {
    a.memberships & b.filter != 0 && b.memberships & a.filter != 0
}

/// Advances every assembly by `dt_s`. Assemblies that touch through a
/// contact are solved together; the others are solved one island each.
pub(crate) fn step_world(
    assemblies: &mut [Assembly],
    statics: &[StaticCollider],
    warm: &mut HashMap<WarmKey, [f64; 3]>,
    config: &ContactStepConfig,
    margin_m: f64,
    dt_s: f64,
) -> Result<Vec<ContactRecord>, PhysicsError> {
    let transforms = assemblies
        .iter()
        .map(Assembly::sim_transforms)
        .collect::<Result<Vec<_>, _>>()?;
    let moving = moving_colliders(assemblies, &transforms);
    let candidates = deduplicate(contact_candidates(
        assemblies,
        &transforms,
        statics,
        &moving,
        margin_m,
    ));

    // Islands: assemblies joined by a contact between them.
    let mut parent: Vec<usize> = (0..assemblies.len()).collect();
    fn find(parent: &mut [usize], index: usize) -> usize {
        let mut root = index;
        while parent[root] != root {
            root = parent[root];
        }
        parent[index] = root;
        root
    }
    for candidate in &candidates {
        if let Some(b) = candidate.contact.b {
            let (x, y) = (
                find(&mut parent, candidate.contact.a.body),
                find(&mut parent, b.body),
            );
            if x != y {
                parent[x.max(y)] = x.min(y);
            }
        }
    }
    let roots: Vec<usize> = (0..assemblies.len())
        .map(|i| find(&mut parent, i))
        .collect();

    let config = ContactStepConfig {
        step_time_s: dt_s,
        ..*config
    };
    let mut next_warm = HashMap::new();
    let mut records = Vec::with_capacity(candidates.len());
    for root in 0..assemblies.len() {
        if roots[root] != root {
            continue;
        }
        let members: Vec<usize> = (0..assemblies.len())
            .filter(|&i| roots[i] == root)
            .collect();
        let local = |global: usize| members.binary_search(&global).expect("island member");
        let island: Vec<&Candidate> = candidates
            .iter()
            .filter(|candidate| roots[candidate.contact.a.body] == root)
            .collect();
        let contacts: Vec<CoupledContact> = island
            .iter()
            .map(|candidate| {
                let mut contact = candidate.contact;
                contact.a.body = local(contact.a.body);
                if let Some(b) = contact.b.as_mut() {
                    b.body = local(b.body);
                }
                contact
            })
            .collect();
        let seed: Vec<[f64; 3]> = island
            .iter()
            .map(|candidate| warm.get(&candidate.key).copied().unwrap_or([0.0; 3]))
            .collect();
        let bodies: Vec<_> = members
            .iter()
            .map(|&index| assemblies[index].coupled_body())
            .collect();
        let step = contact_step_coupled(&bodies, &contacts, &config, Some(&seed))
            .map_err(|_| PhysicsError::InitializationFailed)?;
        for (candidate, outcome) in island.iter().zip(&step.contacts) {
            let [t1, t2, n] = contact_frame(candidate.contact.normal_world);
            let impulse = outcome.impulse_world_n_s;
            next_warm.insert(
                candidate.key,
                [impulse.dot(t1), impulse.dot(t2), impulse.dot(n)],
            );
            records.push(ContactRecord {
                normal_impulse_n_s: impulse.dot(n),
                ..candidate.record
            });
        }
        for (&index, body_step) in members.iter().zip(step.bodies) {
            assemblies[index].q = body_step.q;
            assemblies[index].qd = body_step.qd;
        }
    }
    *warm = next_warm;
    Ok(records)
}

fn moving_colliders(
    assemblies: &[Assembly],
    transforms: &[Vec<Transform3>],
) -> Vec<MovingCollider> {
    let mut moving = Vec::new();
    for (assembly_index, assembly) in assemblies.iter().enumerate() {
        for (body_index, body) in assembly.bodies.iter().enumerate() {
            let Some((shape, offset)) = &body.collider else {
                continue;
            };
            let radius_m = body
                .samples
                .iter()
                .map(|sample| (sample.center_m - offset.translation).length() + sample.radius_m)
                .fold(0.0, f64::max);
            if radius_m <= 0.0 {
                continue;
            }
            let pose = transforms[assembly_index][body.link_index].mul_transform(offset);
            moving.push(MovingCollider {
                assembly: assembly_index,
                body: body_index,
                collider: StaticCollider {
                    entity: body.entity,
                    shape: shape.clone(),
                    pose,
                    friction: body.friction,
                },
                groups: body.groups,
                radius_m,
            });
        }
    }
    moving
}

/// One side of a contact: a body of an assembly, or the static world.
#[derive(Clone, Copy)]
struct Side {
    assembly: usize,
    entity: Entity,
    link: Entity,
    /// Link pose in the simulation frame.
    pose: Transform3,
}

impl Side {
    fn anchor(&self, point_m: Vec3) -> ContactAnchor {
        ContactAnchor {
            body: self.assembly,
            link: self.link,
            point_local_m: self.pose.rotation.conjugate() * (point_m - self.pose.translation),
        }
    }
}

/// A contact pushing `a` at `point_m` along `normal` (from `b`, or a static
/// entity, into `a`).
struct Touch {
    a: Side,
    b: Option<Side>,
    other_entity: Entity,
    point_m: Vec3,
    normal: Vec3,
    gap_m: f64,
    friction: f64,
    feature: usize,
}

fn candidate(touch: Touch) -> Candidate {
    let surface = touch.point_m - touch.normal * touch.gap_m;
    Candidate {
        contact: CoupledContact {
            a: touch.a.anchor(touch.point_m),
            b: touch.b.map(|b| b.anchor(surface)),
            normal_world: touch.normal,
            gap_m: touch.gap_m,
            friction_coefficient: touch.friction,
        },
        key: (
            touch.a.entity.index(),
            touch.feature,
            touch.other_entity.index(),
        ),
        record: record(
            touch.a.entity,
            touch.other_entity,
            touch.normal,
            touch.gap_m,
        ),
        place_m: touch.point_m - touch.normal * (0.5 * touch.gap_m),
    }
}

/// Feature index of a box corner found from the other box's side.
const FACE_SIDE_FEATURES: usize = 1000;

/// Box-on-box contacts between `first` (a box) and `second` (a box, or the
/// static world when `second_side` is `None`).
#[allow(clippy::too_many_arguments)]
fn push_box_box(
    candidates: &mut Vec<Candidate>,
    first: (Side, Vec3, Transform3),
    second: (Option<Side>, Entity, Vec3, Transform3),
    friction: f64,
    margin_m: f64,
) {
    let (first_side, first_half, first_pose) = first;
    let (second_side, second_entity, second_half, second_pose) = second;
    for corner in box_box(first_half, &first_pose, second_half, &second_pose, margin_m) {
        let touch = match (corner.on_first, second_side) {
            (true, _) => Touch {
                a: first_side,
                b: second_side,
                other_entity: second_entity,
                point_m: corner.point_m,
                normal: corner.normal,
                gap_m: corner.gap_m,
                friction,
                feature: corner.corner,
            },
            (false, Some(second_side)) => Touch {
                a: second_side,
                b: Some(first_side),
                other_entity: first_side.entity,
                point_m: corner.point_m,
                normal: corner.normal,
                gap_m: corner.gap_m,
                friction,
                feature: corner.corner,
            },
            // A static corner under the box's face: the face is pushed off it.
            (false, None) => Touch {
                a: first_side,
                b: None,
                other_entity: second_entity,
                point_m: corner.point_m - corner.normal * corner.gap_m,
                normal: -corner.normal,
                gap_m: corner.gap_m,
                friction,
                feature: FACE_SIDE_FEATURES + corner.corner,
            },
        };
        candidates.push(candidate(touch));
    }
}

fn cuboid_half(shape: &ColliderShape) -> Option<Vec3> {
    match shape {
        ColliderShape::Cuboid { half_extents_m } => Some(*half_extents_m),
        _ => None,
    }
}

/// Contacts within `margin_m` between every body and the static colliders
/// and other bodies, in assembly, body, and sample order. Box pairs use
/// [`box_box`]; everything else samples the body's collider.
fn contact_candidates(
    assemblies: &[Assembly],
    transforms: &[Vec<Transform3>],
    statics: &[StaticCollider],
    moving: &[MovingCollider],
    margin_m: f64,
) -> Vec<Candidate> {
    let mut candidates = Vec::new();
    let side_of = |assembly: usize, body: usize| {
        let entry = &assemblies[assembly].bodies[body];
        Side {
            assembly,
            entity: entry.entity,
            link: entry.model_link,
            pose: transforms[assembly][entry.link_index],
        }
    };
    for (assembly_index, assembly) in assemblies.iter().enumerate() {
        for (body_index, body) in assembly.bodies.iter().enumerate() {
            let side = side_of(assembly_index, body_index);
            let body_box = body.collider.as_ref().and_then(|(shape, offset)| {
                Some((cuboid_half(shape)?, side.pose.mul_transform(offset)))
            });
            for collider in statics {
                let friction = 0.5 * (body.friction + collider.friction);
                if let (Some((half, pose)), Some(static_half)) =
                    (body_box, cuboid_half(&collider.shape))
                {
                    push_box_box(
                        &mut candidates,
                        (side, half, pose),
                        (None, collider.entity, static_half, collider.pose),
                        friction,
                        margin_m,
                    );
                    continue;
                }
                for (sample_index, sample) in body.samples.iter().enumerate() {
                    let center = side.pose.translation + side.pose.rotation * sample.center_m;
                    if let Some(hit) = collider.query(center, sample.radius_m) {
                        if hit.gap_m <= margin_m {
                            candidates.push(candidate(Touch {
                                a: side,
                                b: None,
                                other_entity: collider.entity,
                                point_m: hit.point_m,
                                normal: hit.normal,
                                gap_m: hit.gap_m,
                                friction,
                                feature: sample_index,
                            }));
                        }
                    }
                }
            }
            for other in moving {
                if (other.assembly == assembly_index
                    && (other.body == body_index || assembly.adjacent(body_index, other.body)))
                    || !interacts(body.groups, other.groups)
                {
                    continue;
                }
                let other_side = side_of(other.assembly, other.body);
                let friction = 0.5 * (body.friction + other.collider.friction);
                if let (Some((half, pose)), Some(other_half)) =
                    (body_box, cuboid_half(&other.collider.shape))
                {
                    // Each box pair once, from its first body.
                    if (assembly_index, body_index) < (other.assembly, other.body) {
                        push_box_box(
                            &mut candidates,
                            (side, half, pose),
                            (
                                Some(other_side),
                                other_side.entity,
                                other_half,
                                other.collider.pose,
                            ),
                            friction,
                            margin_m,
                        );
                    }
                    continue;
                }
                for (sample_index, sample) in body.samples.iter().enumerate() {
                    let center = side.pose.translation + side.pose.rotation * sample.center_m;
                    let reach = other.radius_m + sample.radius_m + margin_m;
                    if (center - other.collider.pose.translation).length_squared() > reach * reach {
                        continue;
                    }
                    if let Some(hit) = other.collider.query(center, sample.radius_m) {
                        if hit.gap_m <= margin_m {
                            candidates.push(candidate(Touch {
                                a: side,
                                b: Some(other_side),
                                other_entity: other_side.entity,
                                point_m: hit.point_m,
                                normal: hit.normal,
                                gap_m: hit.gap_m,
                                friction,
                                feature: sample_index,
                            }));
                        }
                    }
                }
            }
        }
    }
    candidates
}

/// Distance below which two contacts between the same pair of dynamic bodies
/// are one contact, in meters.
const DUPLICATE_DISTANCE_M: f64 = 1.0e-3;

/// Drops a contact between two dynamic bodies when an earlier one joins the
/// same pair at the same place. Face-to-face boxes sample each other's
/// corners from both sides; the coincident pairs would make the Delassus
/// matrix rank deficient and slow the solver without adding a constraint.
fn deduplicate(candidates: Vec<Candidate>) -> Vec<Candidate> {
    let mut kept: Vec<Candidate> = Vec::with_capacity(candidates.len());
    let mut places: HashMap<(u32, u32), Vec<Vec3>> = HashMap::new();
    for candidate in candidates {
        if candidate.contact.b.is_none() {
            kept.push(candidate);
            continue;
        }
        let (a, b) = (
            candidate.record.body_entity.index(),
            candidate.record.other_entity.index(),
        );
        let pair = (a.min(b), a.max(b));
        let place = candidate.place_m;
        let seen = places.entry(pair).or_default();
        if seen.iter().any(|other| {
            other.distance_squared(place) < DUPLICATE_DISTANCE_M * DUPLICATE_DISTANCE_M
        }) {
            continue;
        }
        seen.push(place);
        kept.push(candidate);
    }
    kept
}

fn record(body_entity: Entity, other_entity: Entity, normal: Vec3, gap_m: f64) -> ContactRecord {
    ContactRecord {
        body_entity,
        other_entity,
        normal,
        gap_m,
        normal_impulse_n_s: 0.0,
    }
}
