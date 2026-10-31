//! Physical kicking leg derived from the rendered human's right leg.
//!
//! Segment lengths, joint planes, trouser radii and the shoe box come from the
//! same glTF character that the renderer draws. The animated leg moves in a
//! plane that turns about one fixed hip axis, so the physical leg is a hip
//! abduction joint followed by a planar hip-knee-ankle chain. The hip follows
//! the animated pelvis as a commanded kinematic body, and joint torques track
//! the animated angles within fixed adult strength ceilings. The shoe reaches
//! the robot through ordinary physics contact, so the delivered impulse, peak
//! force and contact duration are measured outputs, not prescribed inputs.
//!
//! The kicker's torso is not pushed back by the reaction: only the leg joints
//! yield. Segment inertias use Dempster parameters tabulated by Winter for an
//! assumed 70 kg adult; the torque ceilings are constant and ignore the
//! force-velocity reduction of real muscle.

use rne_ai::UrdfSceneSim;
use rne_ecs::{spawn_named, Entity};
use rne_math::{Quat, Transform3 as NodeTransform, Vec3};
use rne_physics::{
    Collider, ColliderShape, CollisionGroups, CommandedKinematicPose, JointActuation, JointState,
    MultibodyLink, PhysicsMaterial, PhysicsOwnedPose, RevoluteJointDesc, RigidBody,
    RigidBodyInertia, RigidBodyType,
};
use rne_render::GltfSceneAsset;
use rne_robot::Link;
use rne_world::Transform3;
use serde_json::{json, Value};
use std::error::Error;

type KickResult<T> = Result<T, Box<dyn Error>>;

/// Assumed body mass of the 1.75 m character in kilograms.
const BODY_MASS_KG: f64 = 70.0;
/// Dempster segment parameters as tabulated by Winter: mass fraction, centre
/// of mass from the proximal joint and radius of gyration about the centre of
/// mass, both per segment length. Thigh, shank (leg) and foot.
const SEGMENT_FRACTIONS: [(f64, f64, f64); 3] = [
    (0.100, 0.433, 0.323),
    (0.0465, 0.433, 0.302),
    (0.0145, 0.5, 0.475),
];
const SEGMENT_NAMES: [&str; 3] = ["kick_thigh", "kick_shank", "kick_foot"];
const JOINT_NAMES: [&str; 4] = ["hip_abduction", "hip_flexion", "knee", "ankle"];
const BONES: [&str; 4] = ["thigh_r", "calf_r", "foot_r", "ball_r"];
const PELVIS: &str = "pelvis";
const SHOE_NODES: [&str; 2] = ["Shoe_r", "Sole_r"];
/// Small rigid link that carries the abduction coordinate between the
/// kinematic hip and the thigh. Its mass is negligible beside the leg.
const ABDUCTION_LINK_MASS_KG: f64 = 0.1;
const ABDUCTION_LINK_INERTIA_KG_M2: f64 = 1.0e-4;
/// Symmetric abduction, hip-flexion, knee and ankle torque ceilings in N·m.
/// Knee extension uses 2.9 N·m/kg (adult male isometric references span about
/// 2.9-3.5 N·m/kg); hip and ankle are rounded adult values. Constant ceilings
/// ignore the force-velocity reduction of muscle, so they are generous at speed.
pub(super) const TORQUE_LIMITS_NM: [f64; 4] = [120.0, 140.0, 200.0, 100.0];
/// Accepted multipliers of [`TORQUE_LIMITS_NM`]. Values above one model a
/// stronger, trained kicker; they are assumptions, not measured athletes.
const STRENGTH_SCALE_RANGE: std::ops::RangeInclusive<f64> = 0.5..=2.0;
/// Joint-space tracking gains. These are controller choices, not physiology.
const STIFFNESS_NM_PER_RAD: [f64; 4] = [2000.0, 3000.0, 2000.0, 150.0];
const DAMPING_NM_S_PER_RAD: [f64; 4] = [80.0, 100.0, 35.0, 1.5];
/// Joint ranges are the animated range widened by this margin.
const JOINT_RANGE_MARGIN_RAD: f64 = 0.35;
/// Kicker colliders ignore each other but touch the robot and the ground.
const KICKER_COLLISION_GROUP: u32 = 1 << 30;
/// Trouser vertices between these fractions of a bone define its radius.
const RADIUS_SPAN: (f64, f64) = (0.15, 0.85);
const RADIUS_PERCENTILE: f64 = 0.8;
const REFERENCE_DIFFERENCE_S: f64 = 5.0e-4;
/// Animation samples used to fit the leg planes, at 100 Hz over the clip.
const CLIP_SAMPLES: u32 = 590;

fn require(condition: bool, message: &str) -> KickResult<()> {
    if condition {
        Ok(())
    } else {
        Err(message.into())
    }
}

fn wrap_rad(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// Signed angle from `from` to `to` about `axis`; inputs need not be unit.
fn signed_angle(from: Vec3, to: Vec3, axis: Vec3) -> f64 {
    from.cross(to).dot(axis).atan2(from.dot(to))
}

fn project(vector: Vec3, normal: Vec3) -> Vec3 {
    vector - normal * vector.dot(normal)
}

fn compose(parent: NodeTransform, local: NodeTransform) -> NodeTransform {
    NodeTransform {
        translation: parent.translation + parent.rotation * (parent.scale * local.translation),
        rotation: parent.rotation * local.rotation,
        scale: parent.scale * local.scale,
    }
}

/// World transforms of every glTF node for the given node-local transforms.
pub(super) fn node_world(asset: &GltfSceneAsset, locals: &[NodeTransform]) -> Vec<NodeTransform> {
    fn resolve(
        index: usize,
        asset: &GltfSceneAsset,
        locals: &[NodeTransform],
        out: &mut [Option<NodeTransform>],
    ) -> NodeTransform {
        if let Some(world) = out[index] {
            return world;
        }
        let world = match asset.nodes[index].parent_index {
            Some(parent) => compose(resolve(parent, asset, locals, out), locals[index]),
            None => locals[index],
        };
        out[index] = Some(world);
        world
    }
    let mut out = vec![None; asset.nodes.len()];
    (0..asset.nodes.len())
        .map(|index| resolve(index, asset, locals, &mut out))
        .collect()
}

fn node_index(asset: &GltfSceneAsset, name: &str) -> KickResult<usize> {
    asset
        .nodes
        .iter()
        .position(|node| node.name.as_deref() == Some(name))
        .ok_or_else(|| format!("human asset has no node {name}").into())
}

/// Inertia of a segment that is symmetric about its +Y bone axis.
fn segment_inertia(center_m: Vec3, transverse_kg_m2: f64, axial_kg_m2: f64) -> RigidBodyInertia {
    RigidBodyInertia {
        center_of_mass_local_m: center_m,
        ixx_kg_m2: transverse_kg_m2,
        ixy_kg_m2: 0.0,
        ixz_kg_m2: 0.0,
        iyy_kg_m2: axial_kg_m2,
        iyz_kg_m2: 0.0,
        izz_kg_m2: transverse_kg_m2,
    }
}

/// Piecewise-linear map from recording time to animation time.
///
/// Idle, chamber, hold, retraction and landing keep the earlier visual timing.
/// Only the 350 ms authored extension is shortened by `speed_scale`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct KickSchedule {
    pub(super) speed_scale: f64,
}

impl KickSchedule {
    pub(super) fn new(speed_scale: f64) -> KickResult<Self> {
        require(
            speed_scale.is_finite() && (0.5..=4.0).contains(&speed_scale),
            "kick speed scale must be within 0.5..=4",
        )?;
        Ok(Self { speed_scale })
    }

    fn knots(self) -> [(f64, f64); 8] {
        let extended_s = 1.825 + 0.35 / self.speed_scale;
        [
            (0.0, 0.0),
            (1.2, 1.2),
            (1.825, 1.65),
            (extended_s, 2.0),
            (extended_s + 0.1, 2.2),
            (extended_s + 0.32, 2.5),
            (extended_s + 0.77, 2.95),
            (6.2, 5.9),
        ]
    }

    /// Recording time at which the authored extension finishes.
    pub(super) fn extension_end_s(self) -> f64 {
        self.knots()[3].0
    }

    pub(super) fn asset_time_s(self, time_s: f64) -> f64 {
        let knots = self.knots();
        if time_s <= knots[0].0 {
            return knots[0].1;
        }
        for pair in knots.windows(2) {
            let [(start_s, asset_start_s), (end_s, asset_end_s)] = pair else {
                unreachable!("two time knots");
            };
            if time_s <= *end_s {
                let fraction = (time_s - start_s) / (end_s - start_s);
                return asset_start_s + fraction * (asset_end_s - asset_start_s);
            }
        }
        knots[knots.len() - 1].1
    }

    fn configuration(self) -> Value {
        json!({"speed_scale":self.speed_scale,"knots_recording_to_animation_s":self.knots(),
            "authored_extension_s":0.35,"extension_s":0.35/self.speed_scale})
    }
}

/// One rigid leg segment in its own frame: origin at the proximal joint,
/// +X along the knee hinge axis and +Y towards the distal joint.
#[derive(Clone, Debug)]
struct Segment {
    length_m: f64,
    mass_kg: f64,
    inertia: RigidBodyInertia,
    collider: Collider,
}

/// Leg geometry, mass properties and references derived from the human asset.
#[derive(Clone, Debug)]
pub(super) struct KickLegModel {
    /// Animated clip that supplies the hip path and joint-angle references.
    clip: &'static str,
    clip_index: usize,
    pelvis: usize,
    bones: [usize; 4],
    /// Leg-plane normal in the first animation pose, in the world frame.
    standing_normal: Vec3,
    /// Fixed world axis about which the leg plane turns (hip abduction).
    abduction_axis: Vec3,
    anchor_rotation: Quat,
    segments: [Segment; 3],
    calibration: [Quat; 3],
    zero_rad: [f64; 4],
    limits_rad: [(f64, f64); 4],
    plane_axis_deviation_max_rad: f64,
    toe_plane_offset_max_m: f64,
    radii_m: [f64; 2],
    shoe_half_extents_m: Vec3,
}

impl KickLegModel {
    pub(super) fn from_asset(asset: &GltfSceneAsset, clip: &'static str) -> KickResult<Self> {
        let clip_index = asset
            .animations
            .iter()
            .position(|animation| {
                animation
                    .name
                    .as_deref()
                    .is_some_and(|name| name.contains(clip))
            })
            .ok_or_else(|| format!("human asset has no {clip} clip"))?;
        let pelvis = node_index(asset, PELVIS)?;
        let bones = [
            node_index(asset, BONES[0])?,
            node_index(asset, BONES[1])?,
            node_index(asset, BONES[2])?,
            node_index(asset, BONES[3])?,
        ];
        for pair in bones.windows(2) {
            require(
                asset.nodes[pair[1]].parent_index == Some(pair[0]),
                "the kicking leg bones must form one chain",
            )?;
        }
        require(
            asset.nodes[bones[0]].parent_index == Some(pelvis),
            "the thigh must hang from the pelvis",
        )?;
        for &bone in &bones[1..] {
            let local = asset.nodes[bone].bind_transform.translation;
            require(
                (local.y - local.length()).abs() < 1.0e-6,
                "each child bone must sit on its parent's +Y bone axis",
            )?;
        }
        let lengths_m = [1, 2, 3].map(|child| {
            asset.nodes[bones[child]]
                .bind_transform
                .translation
                .length()
        });
        let mut model = Self {
            clip,
            clip_index,
            pelvis,
            bones,
            standing_normal: Vec3::X,
            abduction_axis: Vec3::Z,
            anchor_rotation: Quat::IDENTITY,
            segments: [0, 1, 2].map(|i| Segment {
                length_m: lengths_m[i],
                mass_kg: 0.0,
                inertia: segment_inertia(Vec3::ZERO, 0.0, 0.0),
                collider: Collider::default(),
            }),
            calibration: [Quat::IDENTITY; 3],
            zero_rad: [0.0; 4],
            limits_rad: [(0.0, 0.0); 4],
            plane_axis_deviation_max_rad: 0.0,
            toe_plane_offset_max_m: 0.0,
            radii_m: [0.0; 2],
            shoe_half_extents_m: Vec3::ZERO,
        };
        model.fit_planes(asset)?;
        model.zero_rad = model.reference_angles(asset, 0.0);
        let mut low = [f64::INFINITY; 4];
        let mut high = [f64::NEG_INFINITY; 4];
        for sample in 0..=CLIP_SAMPLES {
            let world = model.world_at(asset, f64::from(sample) * 0.01);
            let angles = model.angles_from_world(&world);
            let points = model.joint_points(&world);
            let plane = model.chain_frame(angles[0]) * Vec3::X;
            model.toe_plane_offset_max_m = model
                .toe_plane_offset_max_m
                .max((points[3] - points[0]).dot(plane).abs());
            for joint in 0..4 {
                low[joint] = low[joint].min(angles[joint]);
                high[joint] = high[joint].max(angles[joint]);
            }
        }
        model.limits_rad = [0, 1, 2, 3].map(|joint| {
            (
                low[joint] - JOINT_RANGE_MARGIN_RAD,
                high[joint] + JOINT_RANGE_MARGIN_RAD,
            )
        });
        model.calibrate(asset)?;
        model.fit_volumes(asset)?;
        Ok(model)
    }

    fn world_at(&self, asset: &GltfSceneAsset, asset_time_s: f64) -> Vec<NodeTransform> {
        let locals = asset.animations[self.clip_index]
            .sample_node_transforms(&asset.nodes, asset_time_s as f32);
        node_world(asset, &locals)
    }

    fn joint_points(&self, world: &[NodeTransform]) -> [Vec3; 4] {
        self.bones.map(|bone| world[bone].translation)
    }

    /// Unit normal of the hip-knee-ankle plane, oriented like the knee bend.
    fn leg_normal(points: &[Vec3; 4]) -> Option<Vec3> {
        let bend = (points[1] - points[0]).cross(points[2] - points[1]);
        (bend.length() > 0.02).then(|| bend.normalize())
    }

    /// Fits the standing leg plane and the fixed axis it turns about.
    fn fit_planes(&mut self, asset: &GltfSceneAsset) -> KickResult<()> {
        let normals: Vec<_> = (0..=CLIP_SAMPLES)
            .map(|sample| {
                Self::leg_normal(
                    &self.joint_points(&self.world_at(asset, f64::from(sample) * 0.01)),
                )
                .ok_or("the animated knee straightens; its leg plane is undefined")
            })
            .collect::<Result<_, _>>()?;
        self.standing_normal = normals[0];
        let farthest = normals
            .iter()
            .copied()
            .max_by(|a, b| {
                a.angle_between(self.standing_normal)
                    .total_cmp(&b.angle_between(self.standing_normal))
            })
            .expect("leg normals");
        require(
            farthest.angle_between(self.standing_normal) > 0.01,
            "the leg plane does not turn during the kick",
        )?;
        self.abduction_axis = self.standing_normal.cross(farthest).normalize();
        self.plane_axis_deviation_max_rad = normals
            .iter()
            .map(|normal| {
                normal
                    .dot(self.abduction_axis)
                    .clamp(-1.0, 1.0)
                    .asin()
                    .abs()
            })
            .fold(0.0, f64::max);
        let down = project(-Vec3::Y, self.standing_normal);
        require(down.length() > 0.5, "the leg plane is nearly horizontal")?;
        let first = Quat::from_rotation_arc(Vec3::X, self.standing_normal);
        let turn = signed_angle(first * Vec3::Y, down.normalize(), self.standing_normal);
        self.anchor_rotation =
            (Quat::from_axis_angle(self.standing_normal, turn) * first).normalize();
        Ok(())
    }

    /// Abduction axis expressed in the hip anchor frame.
    fn abduction_axis_local(&self) -> Vec3 {
        (self.anchor_rotation.inverse() * self.abduction_axis).normalize()
    }

    /// World frame of the abducted leg plane: +X is the hinge axis.
    fn chain_frame(&self, abduction_rad: f64) -> Quat {
        (self.anchor_rotation * Quat::from_axis_angle(self.abduction_axis_local(), abduction_rad))
            .normalize()
    }

    /// Abduction, hip-flexion, knee and ankle angles of an animated pose.
    fn angles_from_world(&self, world: &[NodeTransform]) -> [f64; 4] {
        let points = self.joint_points(world);
        let normal = Self::leg_normal(&points).unwrap_or(self.standing_normal);
        let abduction = signed_angle(
            self.standing_normal,
            project(normal, self.abduction_axis),
            self.abduction_axis,
        );
        let frame = self.chain_frame(abduction);
        let hinge = frame * Vec3::X;
        let reference = frame * Vec3::Y;
        let absolute = [0, 1, 2]
            .map(|i| signed_angle(reference, project(points[i + 1] - points[i], hinge), hinge));
        [
            abduction,
            absolute[0],
            wrap_rad(absolute[1] - absolute[0]),
            wrap_rad(absolute[2] - absolute[1]),
        ]
    }

    /// Joint angles at an animation time.
    fn reference_angles(&self, asset: &GltfSceneAsset, asset_time_s: f64) -> [f64; 4] {
        self.angles_from_world(&self.world_at(asset, asset_time_s))
    }

    /// World rotations of the thigh, shank and foot for joint angles.
    fn segment_rotations(&self, angles: [f64; 4]) -> [Quat; 3] {
        let frame = self.chain_frame(angles[0]);
        let mut cumulative = 0.0;
        [angles[1], angles[2], angles[3]].map(|angle| {
            cumulative += angle;
            (frame * Quat::from_rotation_x(cumulative)).normalize()
        })
    }

    /// Hip, knee, ankle and toe-joint positions of the chain.
    fn chain_points(&self, hip: Vec3, angles: [f64; 4]) -> ([Vec3; 4], [Quat; 3]) {
        let rotations = self.segment_rotations(angles);
        let mut points = [hip; 4];
        for i in 0..3 {
            points[i + 1] = points[i] + rotations[i] * (Vec3::Y * self.segments[i].length_m);
        }
        (points, rotations)
    }

    /// Maps each physical segment frame to the rendered bone frame.
    fn calibrate(&mut self, asset: &GltfSceneAsset) -> KickResult<()> {
        let world = self.world_at(asset, 0.0);
        let physical = self.segment_rotations(self.zero_rad);
        for i in 0..3 {
            let bone = world[self.bones[i]].rotation;
            let swing = Quat::from_rotation_arc(
                (bone * Vec3::Y).normalize(),
                (physical[i] * Vec3::Y).normalize(),
            );
            self.calibration[i] = (physical[i].inverse() * swing * bone).normalize();
            require(
                ((physical[i] * self.calibration[i]) * Vec3::Y - physical[i] * Vec3::Y).length()
                    < 1.0e-9,
                "the rendered bone axis must follow its physical segment",
            )?;
        }
        Ok(())
    }

    /// Fits trouser capsules, the shoe box and Dempster mass properties.
    fn fit_volumes(&mut self, asset: &GltfSceneAsset) -> KickResult<()> {
        let bind = node_world(
            asset,
            &asset
                .nodes
                .iter()
                .map(|node| node.bind_transform)
                .collect::<Vec<_>>(),
        );
        let mut radial = [Vec::new(), Vec::new()];
        let mut shoe_low = Vec3::splat(f64::INFINITY);
        let mut shoe_high = Vec3::splat(f64::NEG_INFINITY);
        let mut shoe_vertices = 0;
        for part in &asset.parts {
            let (Some(skin_index), Some(weights)) = (part.skin_index, &part.skin_weights) else {
                continue;
            };
            let node = bind[part.node_index];
            require(
                node.translation.length() < 1.0e-6
                    && node.rotation.angle_between(Quat::IDENTITY) < 1.0e-6
                    && (node.scale - Vec3::ONE).length() < 1.0e-5,
                "skinned human parts must use an identity mesh node",
            )?;
            let skin = &asset.skins[skin_index];
            let shoe = asset.nodes[part.node_index]
                .name
                .as_deref()
                .is_some_and(|name| SHOE_NODES.contains(&name));
            for ((position, joints), influence) in part
                .render_part
                .mesh
                .positions
                .iter()
                .zip(&weights.joints)
                .zip(&weights.weights)
            {
                let (slot, weight) = influence
                    .iter()
                    .enumerate()
                    .max_by(|a, b| a.1.total_cmp(b.1))
                    .expect("four influences");
                if *weight < 0.5 {
                    continue;
                }
                let joint = &skin.joints[usize::from(joints[slot])];
                let vertex = Vec3::new(
                    f64::from(position[0]),
                    f64::from(position[1]),
                    f64::from(position[2]),
                );
                let local = joint.inverse_bind_matrix.transform_point3(vertex);
                if shoe && joint.node_index == self.bones[2] {
                    shoe_low = shoe_low.min(local);
                    shoe_high = shoe_high.max(local);
                    shoe_vertices += 1;
                } else if !shoe {
                    for (i, radii) in radial.iter_mut().enumerate() {
                        let span = self.segments[i].length_m;
                        if joint.node_index == self.bones[i]
                            && (RADIUS_SPAN.0 * span..=RADIUS_SPAN.1 * span).contains(&local.y)
                        {
                            radii.push(local.x.hypot(local.z));
                        }
                    }
                }
            }
        }
        require(shoe_vertices > 100, "the right shoe mesh is missing")?;
        for (i, radii) in radial.iter_mut().enumerate() {
            require(radii.len() > 50, "the right trouser leg mesh is missing")?;
            radii.sort_by(f64::total_cmp);
            self.radii_m[i] =
                radii[((radii.len() - 1) as f64 * RADIUS_PERCENTILE).round() as usize];
        }
        let shoe_center = (shoe_low + shoe_high) * 0.5;
        self.shoe_half_extents_m = (shoe_high - shoe_low) * 0.5;
        let foot_rotation = self.calibration[2];
        for (i, (mass_fraction, center_fraction, gyration_fraction)) in
            SEGMENT_FRACTIONS.into_iter().enumerate()
        {
            let segment = &mut self.segments[i];
            segment.mass_kg = BODY_MASS_KG * mass_fraction;
            let transverse = segment.mass_kg * (gyration_fraction * segment.length_m).powi(2);
            let radius_m = if i < 2 {
                self.radii_m[i]
            } else {
                self.shoe_half_extents_m.x.min(self.shoe_half_extents_m.z)
            };
            segment.inertia = segment_inertia(
                Vec3::Y * (center_fraction * segment.length_m),
                transverse,
                0.5 * segment.mass_kg * radius_m * radius_m,
            );
            require(
                segment.inertia.is_valid(),
                "derived leg inertia is not physical",
            )?;
            segment.collider = if i < 2 {
                Collider {
                    shape: ColliderShape::Capsule {
                        half_height_m: (0.5 * segment.length_m - radius_m).max(0.01),
                        radius_m,
                    },
                    material: PhysicsMaterial::default(),
                    local_offset: Transform3::from_translation_rotation(
                        Vec3::Y * (0.5 * segment.length_m),
                        Quat::IDENTITY,
                    ),
                    sensor: false,
                }
            } else {
                Collider {
                    shape: ColliderShape::Cuboid {
                        half_extents_m: self.shoe_half_extents_m,
                    },
                    material: PhysicsMaterial::default(),
                    local_offset: Transform3::from_translation_rotation(
                        foot_rotation * shoe_center,
                        foot_rotation,
                    ),
                    sensor: false,
                }
            };
        }
        Ok(())
    }

    /// Hip position (animation frame), joint angles and joint rates.
    fn reference(
        &self,
        asset: &GltfSceneAsset,
        schedule: KickSchedule,
        time_s: f64,
    ) -> (Vec3, [f64; 4], [f64; 4]) {
        let world = self.world_at(asset, schedule.asset_time_s(time_s));
        let angles = self.angles_from_world(&world);
        let before = self.reference_angles(
            asset,
            schedule.asset_time_s(time_s - REFERENCE_DIFFERENCE_S),
        );
        let after = self.reference_angles(
            asset,
            schedule.asset_time_s(time_s + REFERENCE_DIFFERENCE_S),
        );
        let rates =
            [0, 1, 2, 3].map(|j| wrap_rad(after[j] - before[j]) / (2.0 * REFERENCE_DIFFERENCE_S));
        (world[self.bones[0]].translation, angles, rates)
    }

    /// World corners of the shoe box for a chain pose.
    fn shoe_corners(&self, hip: Vec3, angles: [f64; 4]) -> [Vec3; 8] {
        let (points, rotations) = self.chain_points(hip, angles);
        let offset = self.segments[2].collider.local_offset;
        let half = self.shoe_half_extents_m;
        let mut corners = [Vec3::ZERO; 8];
        for (index, corner) in corners.iter_mut().enumerate() {
            let sign = Vec3::new(
                if index & 1 == 0 { -1.0 } else { 1.0 },
                if index & 2 == 0 { -1.0 } else { 1.0 },
                if index & 4 == 0 { -1.0 } else { 1.0 },
            );
            *corner =
                points[2] + rotations[2] * (offset.translation + offset.rotation * (half * sign));
        }
        corners
    }

    /// Shoe-box corners of the fully extended reference pose, in the animation
    /// frame (human origin at zero).
    pub(super) fn full_extension_shoe_corners(&self, asset: &GltfSceneAsset) -> [Vec3; 8] {
        let world = self.world_at(asset, 2.0);
        let angles = self.angles_from_world(&world);
        self.shoe_corners(world[self.bones[0]].translation, angles)
    }

    /// Places the human so the fully extended reference shoe passes
    /// `follow_through_m` beyond the robot's near surface along +Z, centred on
    /// the target's X. The kick then meets the robot during its extension.
    pub(super) fn placement(
        &self,
        asset: &GltfSceneAsset,
        target: Vec3,
        follow_through_m: f64,
    ) -> (Vec3, Value) {
        let corners = self.full_extension_shoe_corners(asset);
        let front_z = corners
            .iter()
            .map(|c| c.z)
            .fold(f64::NEG_INFINITY, f64::max);
        let center = corners.iter().fold(Vec3::ZERO, |sum, c| sum + *c) / 8.0;
        let origin = Vec3::new(
            target.x - center.x,
            0.0,
            target.z + follow_through_m - front_z,
        );
        (
            origin,
            json!({"target_surface_world_m":target.to_array(),"follow_through_m":follow_through_m,
                "reference_full_extension_shoe_center_world_m":(center+origin).to_array(),
                "reference_full_extension_shoe_front_z_m":front_z+origin.z,
                "human_origin_world_m":origin.to_array()}),
        )
    }

    /// Node-local transforms for rendering, with the right leg following the
    /// simulated segment rotations and the rest of the body from the clip.
    pub(super) fn posed_locals(
        &self,
        asset: &GltfSceneAsset,
        asset_time_s: f64,
        segment_rotations: [Quat; 3],
    ) -> Vec<NodeTransform> {
        let mut locals = asset.animations[self.clip_index]
            .sample_node_transforms(&asset.nodes, asset_time_s as f32);
        let world = node_world(asset, &locals);
        let mut parent = world[self.pelvis].rotation;
        for i in 0..3 {
            let rendered = (segment_rotations[i] * self.calibration[i]).normalize();
            locals[self.bones[i]].rotation = (parent.inverse() * rendered).normalize();
            parent = rendered;
        }
        locals
    }

    /// Rendered hip, knee and ankle positions (animation frame) of a pose.
    pub(super) fn rendered_joints(
        &self,
        asset: &GltfSceneAsset,
        locals: &[NodeTransform],
    ) -> [Vec3; 3] {
        let world = node_world(asset, locals);
        [0, 1, 2].map(|i| world[self.bones[i]].translation)
    }

    pub(super) fn configuration(&self) -> Value {
        let segments: Vec<_> = self
            .segments
            .iter()
            .zip(SEGMENT_NAMES)
            .zip(SEGMENT_FRACTIONS)
            .map(|((segment, name), fractions)| {
                json!({"name":name,"length_m":segment.length_m,"mass_kg":segment.mass_kg,
                    "dempster_winter_fractions":{"mass":fractions.0,"center_of_mass_from_proximal":fractions.1,
                        "radius_of_gyration_about_center":fractions.2},
                    "center_of_mass_local_m":segment.inertia.center_of_mass_local_m.to_array(),
                    "inertia_kg_m2":[segment.inertia.ixx_kg_m2,segment.inertia.iyy_kg_m2,segment.inertia.izz_kg_m2],
                    "collider":format!("{:?}",segment.collider.shape)})
            })
            .collect();
        json!({"model":"hip abduction then a planar hip-knee-ankle chain, with a commanded kinematic hip",
            "source":"right leg of assets/fixtures/kick_human/cc0_sport_human.glb","clip":self.clip,
            "assumed_body_mass_kg":BODY_MASS_KG,"segments":segments,
            "abduction_link_mass_kg":ABDUCTION_LINK_MASS_KG,
            "standing_leg_plane_normal_world":self.standing_normal.to_array(),
            "abduction_axis_world":self.abduction_axis.to_array(),
            "animated_leg_plane_deviation_from_abduction_rad":self.plane_axis_deviation_max_rad,
            "animated_toe_offset_from_simulated_plane_max_m":self.toe_plane_offset_max_m,
            "trouser_radii_m":self.radii_m,"shoe_half_extents_m":self.shoe_half_extents_m.to_array(),
            "joint_order":JOINT_NAMES,"torque_limits_nm":TORQUE_LIMITS_NM,
            "stiffness_nm_per_rad":STIFFNESS_NM_PER_RAD,"damping_nm_s_per_rad":DAMPING_NM_S_PER_RAD,
            "joint_limits_rad":self.limits_rad,
            "torque_limit_basis":"constant symmetric ceilings; knee 2.9 N.m/kg adult isometric reference, hip and ankle rounded adult values; no force-velocity reduction",
            "limitations":["kicker torso follows the animation and is not pushed back by the reaction",
                "animated toe rotation out of the leg plane is not simulated",
                "rigid shoe and robot shell: the impact resolves within one solver step, so the raw per-step peak force reflects the 1 ms step; a 10 ms moving-average peak is reported alongside",
                "contact impulse counts normal impulse only; friction impulse is excluded"]})
    }
}

/// Measured contact between the kicking leg and robot links in one step.
#[derive(Clone, Debug, Default)]
pub(super) struct KickContact {
    pub(super) impulse_on_robot_ns: Vec3,
    /// Normal impulse on each touched robot link, by link name.
    pub(super) impulse_by_link_ns: std::collections::BTreeMap<String, Vec3>,
    pub(super) touching_pairs: usize,
    pub(super) shoe_touching: bool,
    pub(super) shank_or_thigh_touching: bool,
}

/// Stops a fixed collider (the floor) from touching the kicking leg.
///
/// The kicker's torso is kinematic, so floor support of the kicking foot would
/// only fight the joint tracking; the leg is meant to touch the robot alone.
pub(super) fn ignore_kicker_contacts(sim: &mut UrdfSceneSim, entity: Entity) {
    sim.world_mut().entity_mut(entity).insert(CollisionGroups {
        memberships: u32::MAX,
        filter: !KICKER_COLLISION_GROUP,
    });
}

/// A spawned physical kicking leg.
#[derive(Debug)]
pub(super) struct KickLeg {
    model: KickLegModel,
    schedule: KickSchedule,
    origin: Vec3,
    anchor: Entity,
    /// Abduction link, thigh, shank and foot; each carries its parent joint.
    links: [Entity; 4],
    torque_limits_nm: [f64; 4],
    efforts_nm: [f64; 4],
    saturated_steps: [usize; 4],
    reference_error_max_rad: [f64; 4],
}

impl KickLeg {
    /// Spawns the leg at rest in the animation's first pose.
    pub(super) fn spawn(
        sim: &mut UrdfSceneSim,
        asset: &GltfSceneAsset,
        model: KickLegModel,
        schedule: KickSchedule,
        origin: Vec3,
        strength_scale: f64,
    ) -> KickResult<Self> {
        require(
            strength_scale.is_finite() && STRENGTH_SCALE_RANGE.contains(&strength_scale),
            "kicker strength scale must be within 0.5..=2",
        )?;
        let torque_limits_nm = TORQUE_LIMITS_NM.map(|limit| limit * strength_scale);
        let (hip, angles, _) = model.reference(asset, schedule, 0.0);
        for (angle, zero) in angles.into_iter().zip(model.zero_rad) {
            require(
                (angle - zero).abs() < 1.0e-9,
                "the leg must spawn at the animation's first pose",
            )?;
        }
        let hip = hip + origin;
        let (points, rotations) = model.chain_points(hip, angles);
        let world = sim.world_mut();
        let anchor = spawn_named(world, "kick_hip");
        world.entity_mut(anchor).insert((
            RigidBody {
                body_type: RigidBodyType::Kinematic,
                ..RigidBody::default()
            },
            MultibodyLink,
            CommandedKinematicPose,
            Transform3::from_translation_rotation(hip, model.anchor_rotation),
        ));
        let abduction = spawn_named(world, "kick_hip_abduction");
        let (lower, upper) = model.limits_rad[0];
        world.entity_mut(abduction).insert((
            RigidBody {
                body_type: RigidBodyType::Dynamic,
                mass_kg: ABDUCTION_LINK_MASS_KG,
                ..RigidBody::default()
            },
            segment_inertia(
                Vec3::ZERO,
                ABDUCTION_LINK_INERTIA_KG_M2,
                ABDUCTION_LINK_INERTIA_KG_M2,
            ),
            MultibodyLink,
            PhysicsOwnedPose,
            Transform3::from_translation_rotation(hip, model.chain_frame(angles[0])),
            RevoluteJointDesc {
                parent: anchor,
                axis: model.abduction_axis_local(),
                anchor_parent_m: Vec3::ZERO,
                anchor_child_m: Vec3::ZERO,
                relative_rotation: Quat::from_axis_angle(
                    model.abduction_axis_local(),
                    model.zero_rad[0],
                ),
                lower_rad: Some(lower - model.zero_rad[0]),
                upper_rad: Some(upper - model.zero_rad[0]),
            },
            JointActuation::RevoluteEffort {
                effort_nm: 0.0,
                max_effort_nm: torque_limits_nm[0],
            },
        ));
        let mut links = [abduction; 4];
        let mut parent = abduction;
        for i in 0..3 {
            let joint = i + 1;
            let segment = &model.segments[i];
            let entity = spawn_named(world, SEGMENT_NAMES[i]);
            let parent_length_m = if i == 0 {
                0.0
            } else {
                model.segments[i - 1].length_m
            };
            let (lower, upper) = model.limits_rad[joint];
            world.entity_mut(entity).insert((
                RigidBody {
                    body_type: RigidBodyType::Dynamic,
                    mass_kg: segment.mass_kg,
                    ..RigidBody::default()
                },
                segment.inertia,
                MultibodyLink,
                PhysicsOwnedPose,
                Transform3::from_translation_rotation(points[i], rotations[i]),
                RevoluteJointDesc {
                    parent,
                    axis: Vec3::X,
                    anchor_parent_m: Vec3::Y * parent_length_m,
                    anchor_child_m: Vec3::ZERO,
                    relative_rotation: Quat::from_rotation_x(model.zero_rad[joint]),
                    lower_rad: Some(lower - model.zero_rad[joint]),
                    upper_rad: Some(upper - model.zero_rad[joint]),
                },
                JointActuation::RevoluteEffort {
                    effort_nm: 0.0,
                    max_effort_nm: torque_limits_nm[joint],
                },
            ));
            world.entity_mut(entity).insert((
                segment.collider.clone(),
                CollisionGroups::without_self_collision(KICKER_COLLISION_GROUP),
            ));
            links[joint] = entity;
            parent = entity;
        }
        Ok(Self {
            model,
            schedule,
            origin,
            anchor,
            links,
            torque_limits_nm,
            efforts_nm: [0.0; 4],
            saturated_steps: [0; 4],
            reference_error_max_rad: [0.0; 4],
        })
    }

    pub(super) fn schedule(&self) -> KickSchedule {
        self.schedule
    }

    pub(super) fn origin(&self) -> Vec3 {
        self.origin
    }

    /// Absolute joint angles and rates from the latest physics readback.
    pub(super) fn joint_state(&self, sim: &UrdfSceneSim) -> [(f64, f64); 4] {
        [0, 1, 2, 3].map(|i| match sim.world().get::<JointState>(self.links[i]) {
            Some(JointState::Revolute {
                position_rad,
                velocity_rad_s,
            }) => (position_rad + self.model.zero_rad[i], *velocity_rad_s),
            _ => (self.model.zero_rad[i], 0.0),
        })
    }

    /// Commands the hip pose for the end of the next step and joint torques.
    pub(super) fn command(
        &mut self,
        sim: &mut UrdfSceneSim,
        asset: &GltfSceneAsset,
        time_s: f64,
        dt_s: f64,
    ) -> KickResult<()> {
        let (hip, angles, rates) = self.model.reference(asset, self.schedule, time_s + dt_s);
        let state = self.joint_state(sim);
        for joint in 0..4 {
            let error = wrap_rad(angles[joint] - state[joint].0);
            self.reference_error_max_rad[joint] =
                self.reference_error_max_rad[joint].max(error.abs());
            let demand = STIFFNESS_NM_PER_RAD[joint] * error
                + DAMPING_NM_S_PER_RAD[joint] * (rates[joint] - state[joint].1);
            require(demand.is_finite(), "kick joint torque is not finite")?;
            let limit = self.torque_limits_nm[joint];
            if demand.abs() > limit {
                self.saturated_steps[joint] += 1;
            }
            self.efforts_nm[joint] = demand.clamp(-limit, limit);
        }
        let world = sim.world_mut();
        world
            .entity_mut(self.anchor)
            .insert(Transform3::from_translation_rotation(
                hip + self.origin,
                self.model.anchor_rotation,
            ));
        for ((link, effort_nm), max_effort_nm) in self
            .links
            .into_iter()
            .zip(self.efforts_nm)
            .zip(self.torque_limits_nm)
        {
            world
                .entity_mut(link)
                .insert(JointActuation::RevoluteEffort {
                    effort_nm,
                    max_effort_nm,
                });
        }
        Ok(())
    }

    /// Normal contact impulse delivered to robot links during the last step,
    /// from world-frame contact points (force on B is along the A-to-B normal).
    pub(super) fn contact(&self, sim: &UrdfSceneSim, dt_s: f64) -> KickResult<KickContact> {
        let mut contact = KickContact::default();
        let kicker = |entity: Entity| {
            self.links[1..]
                .iter()
                .position(|link| *link == entity)
                .map(|segment| segment == 2)
        };
        let robot = |entity: Entity| sim.world().get::<Link>(entity).is_some();
        let link_name = |entity: Entity| {
            sim.world()
                .get::<Link>(entity)
                .map_or_else(String::new, |link| link.name.clone())
        };
        for sample in sim.physics_contact_points()? {
            require(
                sample.normal_force_n.is_finite() && sample.normal_a_to_b.is_finite(),
                "kick contact force is not finite",
            )?;
            if sample.normal_force_n <= 0.0 {
                continue;
            }
            let (shoe, sign, link) = match (kicker(sample.entity_a), kicker(sample.entity_b)) {
                (Some(shoe), None) if robot(sample.entity_b) => (shoe, 1.0, sample.entity_b),
                (None, Some(shoe)) if robot(sample.entity_a) => (shoe, -1.0, sample.entity_a),
                _ => continue,
            };
            let impulse_ns = sample.normal_a_to_b * (sign * sample.normal_force_n * dt_s);
            contact.impulse_on_robot_ns += impulse_ns;
            *contact
                .impulse_by_link_ns
                .entry(link_name(link))
                .or_insert(Vec3::ZERO) += impulse_ns;
            contact.touching_pairs += 1;
            if shoe {
                contact.shoe_touching = true;
            } else {
                contact.shank_or_thigh_touching = true;
            }
        }
        Ok(contact)
    }

    /// World positions and rotations of the simulated thigh, shank and foot.
    pub(super) fn segment_poses(&self, sim: &UrdfSceneSim) -> [(Vec3, Quat); 3] {
        [1, 2, 3].map(|link| {
            let pose = rne_world::world_transform_of(sim.world(), self.links[link]);
            (pose.translation, pose.rotation)
        })
    }

    pub(super) fn shoe_velocity_m_s(&self, sim: &UrdfSceneSim) -> Vec3 {
        sim.world()
            .get::<RigidBody>(self.links[3])
            .map_or(Vec3::ZERO, |body| body.linear_velocity_m_s)
    }

    /// Literal IEEE words of the leg's observed state for replay checks.
    pub(super) fn state_words(&self, sim: &UrdfSceneSim) -> Vec<u64> {
        let mut words = Vec::new();
        for entity in std::iter::once(self.anchor).chain(self.links) {
            let pose = rne_world::world_transform_of(sim.world(), entity);
            words.extend(
                pose.translation
                    .to_array()
                    .into_iter()
                    .chain(pose.rotation.to_array())
                    .map(f64::to_bits),
            );
            if let Some(body) = sim.world().get::<RigidBody>(entity) {
                words.extend(
                    body.linear_velocity_m_s
                        .to_array()
                        .into_iter()
                        .chain(body.angular_velocity_rad_s.to_array())
                        .map(f64::to_bits),
                );
            }
        }
        for (q, dq) in self.joint_state(sim) {
            words.extend([q.to_bits(), dq.to_bits()]);
        }
        words.extend(self.efforts_nm.map(f64::to_bits));
        words
    }

    pub(super) fn efforts_nm(&self) -> [f64; 4] {
        self.efforts_nm
    }

    pub(super) fn configuration(&self) -> Value {
        json!({"leg":self.model.configuration(),"schedule":self.schedule.configuration(),
            "human_origin_world_m":self.origin.to_array(),
            "applied_torque_limits_nm":self.torque_limits_nm})
    }

    pub(super) fn telemetry(&self) -> Value {
        let named = |values: [f64; 4]| -> Value {
            JOINT_NAMES
                .iter()
                .zip(values)
                .map(|(name, value)| ((*name).to_owned(), json!(value)))
                .collect::<serde_json::Map<_, _>>()
                .into()
        };
        json!({"saturated_steps":named(self.saturated_steps.map(|s| s as f64)),
            "max_reference_error_rad":named(self.reference_error_max_rad)})
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_render::load_gltf_scene;
    use std::path::Path;

    fn asset() -> GltfSceneAsset {
        load_gltf_scene(Path::new(super::super::HUMAN_ASSET)).expect("human asset")
    }

    #[test]
    fn schedule_is_monotonic_and_keeps_visual_phase_boundaries() {
        for scale in [0.5, 1.0, 2.0, 4.0] {
            let schedule = KickSchedule::new(scale).unwrap();
            let mut previous = schedule.asset_time_s(0.0);
            for sample in 1..=6200 {
                let asset_s = schedule.asset_time_s(f64::from(sample) * 1.0e-3);
                assert!(asset_s >= previous);
                previous = asset_s;
            }
            assert!((schedule.asset_time_s(1.825) - 1.65).abs() < 1e-12);
            assert!((schedule.asset_time_s(schedule.extension_end_s()) - 2.0).abs() < 1e-12);
            assert!(previous < 6.0, "the clip must not loop back");
        }
        assert!(KickSchedule::new(0.4).is_err());
        assert!(KickSchedule::new(f64::NAN).is_err());
    }

    #[test]
    fn leg_model_matches_the_rendered_leg_and_dempster_masses() {
        let asset = asset();
        for clip in ["low_kick", "mid_kick"] {
            check_leg_model(&asset, clip);
        }
    }

    fn check_leg_model(asset: &GltfSceneAsset, clip: &'static str) {
        let model = KickLegModel::from_asset(asset, clip).unwrap();
        let masses: Vec<_> = model.segments.iter().map(|s| s.mass_kg).collect();
        assert!((masses[0] - 7.0).abs() < 1e-12);
        assert!((masses[1] - 3.255).abs() < 1e-12);
        assert!((masses[2] - 1.015).abs() < 1e-12);
        assert!(
            model.plane_axis_deviation_max_rad < 1.0e-3,
            "the leg plane turns about one fixed axis"
        );
        assert!(model.toe_plane_offset_max_m < 0.09, "{clip}");
        assert!(model.radii_m.iter().all(|r| (0.03..0.12).contains(r)));
        for sample in 0..=CLIP_SAMPLES {
            let asset_s = f64::from(sample) * 0.01;
            let world = model.world_at(asset, asset_s);
            let angles = model.angles_from_world(&world);
            let (points, rotations) = model.chain_points(world[model.bones[0]].translation, angles);
            let animated = model.joint_points(&world);
            // The knee and ankle lie on the simulated chain to within the tiny
            // deviation from one abduction axis; only the toe twists further.
            assert!((points[1] - animated[1]).length() < 1e-3);
            assert!((points[2] - animated[2]).length() < 1e-3);
            assert!((points[3] - animated[3]).length() < 0.09);
            // Rendering the physical chain puts every bone joint on the chain.
            let locals = model.posed_locals(asset, asset_s, rotations);
            let rendered = node_world(asset, &locals);
            for i in 0..4 {
                assert!((rendered[model.bones[i]].translation - points[i]).length() < 1e-5);
            }
        }
    }

    #[test]
    fn placement_puts_the_reference_shoe_through_the_target_surface() {
        let asset = asset();
        let model = KickLegModel::from_asset(&asset, "low_kick").unwrap();
        let target = Vec3::new(0.01, 0.3, -0.05);
        let (origin, _) = model.placement(&asset, target, 0.1);
        let world = model.world_at(&asset, 2.0);
        let angles = model.angles_from_world(&world);
        let corners = model.shoe_corners(world[model.bones[0]].translation + origin, angles);
        let front = corners
            .iter()
            .map(|c| c.z)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!((front - (target.z + 0.1)).abs() < 1e-12);
        let center = corners.iter().fold(Vec3::ZERO, |s, c| s + *c) / 8.0;
        assert!((center.x - target.x).abs() < 1e-12);
        assert!(
            (0.15..0.45).contains(&center.y),
            "low kick strikes below the hip"
        );
    }
}
