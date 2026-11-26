//! A forward camera on the robot, rendered on the CPU by `rne_render_cpu`.
//!
//! People are the textured CC0 human mesh; cars and bicycles are built from
//! boxes and cylinders; structures and the ground are boxes. Each camera frame
//! gives a PNG and the exact 2D box of every object from the renderer's
//! item-id map. The LiDAR still scans the physics colliders (capsules and
//! boxes), so the two sensors see slightly different shapes for a person.

use std::f64::consts::FRAC_PI_2;
use std::path::Path;
use std::sync::Arc;

use rne_math::{DQuat, Transform3 as MathTransform3, Vec3};
use rne_render::{
    load_gltf_scene, Camera, GltfSceneAsset, ImageFrame, PbrMaterial, RenderScene, RenderSceneItem,
    TriangleMesh, VisualShape,
};
use rne_render_cpu::{CpuRayTracer, CpuRayTracerConfig};
use serde_json::{json, Value};

use crate::{to_engine, vehicles, yaw_rotation, Kind, Pose2, Task, SENSOR_HEIGHT_M};

const HUMAN_GLB: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../assets/fixtures/kick_human/cc0_sport_human.glb"
);

/// Camera height above the ground; it sits above the LiDAR.
pub(crate) const CAMERA_HEIGHT_M: f64 = 1.3;
/// Image size and vertical field of view.
pub(crate) const IMAGE_WIDTH: u32 = 640;
pub(crate) const IMAGE_HEIGHT: u32 = 480;
const FOV_Y_RAD: f64 = 60.0 * std::f64::consts::PI / 180.0;
/// Default: a camera frame every this many LiDAR scans (2 Hz); `--camera-every`.
pub(crate) const SCANS_PER_FRAME: u64 = 5;
/// Objects with fewer visible pixels are left out of the 2D truth.
const MIN_VISIBLE_PIXELS: usize = 30;
const SKY: [f32; 4] = [0.45, 0.6, 0.85, 1.0];
/// Height of the rider's pelvis joint above the saddle top.
const RIDER_SEAT_M: f64 = 0.08;

/// One person mesh part: geometry and material, shared by every copy.
struct HumanPart {
    mesh: Arc<TriangleMesh>,
    texture: Option<Arc<ImageFrame>>,
    material: PbrMaterial,
}

/// A vehicle mesh part: geometry and color, shared by every copy.
type VehiclePart = (Arc<TriangleMesh>, [f32; 3]);

/// Car paints, picked by mover index.
const PAINT: [[f32; 3]; 4] = [
    [0.55, 0.04, 0.04],
    [0.75, 0.75, 0.77],
    [0.04, 0.1, 0.3],
    [0.06, 0.06, 0.06],
];

/// The camera, the person and vehicle meshes, and the renderer.
pub(crate) struct CameraRig {
    human: Vec<HumanPart>,
    /// The person in the riding pose, placed on the bicycle's saddle (bicycle frame).
    rider: Vec<HumanPart>,
    rider_offset: Vec3,
    cars: Vec<Vec<VehiclePart>>,
    bicycle: Vec<VehiclePart>,
    tracer: CpuRayTracer,
    camera: Camera,
}

fn shared(parts: Vec<vehicles::Part>) -> Vec<VehiclePart> {
    parts
        .into_iter()
        .map(|p| (Arc::new(p.mesh), p.color))
        .collect()
}

/// Node-local transforms of the standing pose with `extra` rotations, each
/// about the scene's axes, applied to the named bone after its parents moved.
fn posed_locals(
    asset: &GltfSceneAsset,
    standing: &[MathTransform3],
    extra: &[(&str, DQuat)],
) -> Vec<MathTransform3> {
    fn global(
        index: usize,
        asset: &GltfSceneAsset,
        locals: &mut [MathTransform3],
        done: &mut [Option<DQuat>],
        extra: &[(&str, DQuat)],
    ) -> DQuat {
        if let Some(rotation) = done[index] {
            return rotation;
        }
        let parent = asset.nodes[index]
            .parent_index
            .map_or(DQuat::IDENTITY, |p| global(p, asset, locals, done, extra));
        let mut rotation = parent * locals[index].rotation;
        let name = asset.nodes[index].name.as_deref();
        if let Some((_, delta)) = extra.iter().find(|(n, _)| Some(*n) == name) {
            rotation = *delta * rotation;
            locals[index].rotation = parent.inverse() * rotation;
        }
        done[index] = Some(rotation);
        rotation
    }
    let mut locals = standing.to_vec();
    let mut done = vec![None; locals.len()];
    for index in 0..locals.len() {
        global(index, asset, &mut locals, &mut done, extra);
    }
    locals
}

/// Scene-frame position of a node, from node-local transforms.
fn node_position(asset: &GltfSceneAsset, locals: &[MathTransform3], name: &str) -> Option<Vec3> {
    let mut index = asset
        .nodes
        .iter()
        .position(|n| n.name.as_deref() == Some(name))?;
    let mut point = Vec3::ZERO;
    loop {
        point = locals[index].transform_point(point);
        match asset.nodes[index].parent_index {
            Some(parent) => index = parent,
            None => return Some(point),
        }
    }
}

/// One rendered frame and its 2D truth.
pub(crate) struct CameraFrame {
    pub(crate) rgba8: Vec<u8>,
    /// `(truth_id, class, [x_min, y_min, x_max, y_max], visible pixels)`.
    pub(crate) objects: Vec<(u32, u32, [u32; 4], usize)>,
}

impl CameraRig {
    pub(crate) fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let asset = load_gltf_scene(Path::new(HUMAN_GLB))?;
        // The first frame of a clip is a relaxed standing pose; sample it once.
        let standing: Vec<MathTransform3> = match asset.animations.first() {
            Some(clip) => clip.sample_node_transforms(&asset.nodes, 0.0),
            None => asset.nodes.iter().map(|n| n.bind_transform).collect(),
        };
        let riding = posed_locals(&asset, &standing, &vehicles::riding_pose());
        let sample = |locals: &[MathTransform3]| {
            asset
                .parts
                .iter()
                .enumerate()
                .map(|(index, part)| {
                    let mut material = part.render_part.material.clone();
                    if let Some(color) = part.render_part.base_color_rgba {
                        material.base_color_rgba = color;
                    }
                    Ok(HumanPart {
                        mesh: Arc::new(asset.sample_part_with_node_transforms(index, locals)?),
                        texture: part.render_part.base_color_texture.clone().map(Arc::new),
                        material,
                    })
                })
                .collect::<Result<Vec<_>, rne_render::AnimationSampleError>>()
        };
        let pelvis =
            node_position(&asset, &riding, "pelvis").ok_or("the person mesh has no pelvis")?;
        Ok(Self {
            human: sample(&standing)?,
            rider: sample(&riding)?,
            // Sit the pelvis a little above the saddle; the mesh faces +Z,
            // the bicycle +X.
            rider_offset: Vec3::from(vehicles::SADDLE) + Vec3::new(0.0, RIDER_SEAT_M, 0.0)
                - DQuat::from_rotation_y(FRAC_PI_2) * pelvis,
            cars: PAINT
                .iter()
                .map(|paint| shared(vehicles::car(*paint)))
                .collect(),
            bicycle: shared(vehicles::bicycle([0.1, 0.25, 0.6])),
            tracer: CpuRayTracer::new(CpuRayTracerConfig::default()),
            camera: Camera {
                width: IMAGE_WIDTH,
                height: IMAGE_HEIGHT,
                fov_y_rad: FOV_Y_RAD,
                near_m: 0.1,
                far_m: 150.0,
            },
        })
    }

    /// Renders the task at time `t` from the robot at `pose`.
    pub(crate) fn render(&self, task: &Task, t: f64, pose: Pose2) -> CameraFrame {
        let mut scene = RenderScene::new();
        add_box(
            &mut scene,
            [0.0, 0.0, -0.05],
            [400.0, 400.0, 0.1],
            0.0,
            [0.32, 0.32, 0.3],
        );
        for s in &task.structures {
            let color = if s.half_extents_m[0] < 0.1 {
                [0.25, 0.25, 0.27]
            } else {
                [0.62, 0.6, 0.58]
            };
            add_box(
                &mut scene,
                [s.center[0], s.center[1], s.height_m / 2.0],
                [
                    2.0 * s.half_extents_m[0],
                    2.0 * s.half_extents_m[1],
                    s.height_m,
                ],
                0.0,
                color,
            );
        }
        let mut owners: Vec<(u32, u32, Vec<u32>)> = Vec::new();
        for (index, mover) in task.movers.iter().enumerate() {
            let Some((position, _, heading)) = mover.trajectory.state(t) else {
                continue;
            };
            let first = scene.items.len() as u32;
            match mover.kind {
                Kind::Person => self.add_person(&mut scene, position, heading),
                Kind::Cyclist => {
                    add_vehicle(&mut scene, &self.bicycle, position, heading);
                    self.add_rider(&mut scene, position, heading);
                }
                Kind::Car => add_vehicle(
                    &mut scene,
                    &self.cars[index % self.cars.len()],
                    position,
                    heading,
                ),
            }
            let items = (first..scene.items.len() as u32).collect();
            owners.push((index as u32 + 1, mover.kind.class(), items));
        }

        let view = MathTransform3 {
            translation: to_engine(pose.x, pose.y, CAMERA_HEIGHT_M),
            // The camera looks down its local −Z; turn that onto the heading.
            rotation: yaw_rotation(pose.yaw) * DQuat::from_rotation_y(-FRAC_PI_2),
            scale: Vec3::ONE,
        };
        let output = self.tracer.render(&self.camera, &view, &scene, SKY);
        let objects = owners
            .into_iter()
            .filter_map(|(id, class, items)| {
                let pixels = output.item_ids.iter().filter(|i| items.contains(i)).count();
                let bbox = output
                    .item_boxes(&items)
                    .into_iter()
                    .flatten()
                    .reduce(|a, b| {
                        [
                            a[0].min(b[0]),
                            a[1].min(b[1]),
                            a[2].max(b[2]),
                            a[3].max(b[3]),
                        ]
                    })?;
                (pixels >= MIN_VISIBLE_PIXELS).then_some((id, class, bbox, pixels))
            })
            .collect();
        CameraFrame {
            rgba8: output.color.rgba8,
            objects,
        }
    }

    /// Renders each kind of object alone, from the side, at 45 degrees and
    /// head-on, near and far, for checking how they look (`--preview`).
    pub(crate) fn preview(&self, dir: &Path) -> Result<Vec<String>, Box<dyn std::error::Error>> {
        std::fs::create_dir_all(dir)?;
        let mut written = Vec::new();
        for kind in [Kind::Person, Kind::Cyclist, Kind::Car] {
            for (view, heading) in [
                ("side", FRAC_PI_2),
                ("oblique", 0.8),
                ("front", std::f64::consts::PI),
            ] {
                for distance in [7.0, 18.0] {
                    let mut scene = RenderScene::new();
                    add_box(
                        &mut scene,
                        [0.0, 0.0, -0.05],
                        [400.0, 400.0, 0.1],
                        0.0,
                        [0.32, 0.32, 0.3],
                    );
                    let position = [distance, 0.0];
                    match kind {
                        Kind::Person => self.add_person(&mut scene, position, heading),
                        Kind::Cyclist => {
                            add_vehicle(&mut scene, &self.bicycle, position, heading);
                            self.add_rider(&mut scene, position, heading);
                        }
                        Kind::Car => add_vehicle(&mut scene, &self.cars[0], position, heading),
                    }
                    let view_pose = MathTransform3 {
                        translation: to_engine(0.0, 0.0, CAMERA_HEIGHT_M),
                        rotation: DQuat::from_rotation_y(-FRAC_PI_2),
                        scale: Vec3::ONE,
                    };
                    let output = self.tracer.render(&self.camera, &view_pose, &scene, SKY);
                    let name = format!("{kind:?}_{view}_{distance:.0}m.png").to_lowercase();
                    crate::write_png(
                        &dir.join(&name),
                        &output.color.rgba8,
                        IMAGE_WIDTH,
                        IMAGE_HEIGHT,
                    )?;
                    written.push(name);
                }
            }
        }
        Ok(written)
    }

    /// Bounds of the seated rider in the bicycle's frame (tests).
    #[cfg(test)]
    pub(crate) fn rider_bounds(&self) -> ([f64; 3], [f64; 3]) {
        let rotation = DQuat::from_rotation_y(FRAC_PI_2);
        let mut lo = [f64::MAX; 3];
        let mut hi = [f64::MIN; 3];
        for part in &self.rider {
            for p in &part.mesh.positions {
                let q = rotation * Vec3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]))
                    + self.rider_offset;
                for (k, v) in [q.x, q.y, q.z].into_iter().enumerate() {
                    lo[k] = lo[k].min(v);
                    hi[k] = hi[k].max(v);
                }
            }
        }
        (lo, hi)
    }

    fn add_person(&self, scene: &mut RenderScene, position: [f64; 2], heading: f64) {
        // The mesh faces glTF +Z; turn that onto the heading.
        let transform = MathTransform3 {
            translation: to_engine(position[0], position[1], 0.0),
            rotation: yaw_rotation(heading) * DQuat::from_rotation_y(FRAC_PI_2),
            scale: Vec3::ONE,
        };
        add_human(scene, &self.human, transform);
    }

    fn add_rider(&self, scene: &mut RenderScene, position: [f64; 2], heading: f64) {
        let transform = MathTransform3 {
            translation: to_engine(position[0], position[1], 0.0)
                + yaw_rotation(heading) * self.rider_offset,
            rotation: yaw_rotation(heading) * DQuat::from_rotation_y(FRAC_PI_2),
            scale: Vec3::ONE,
        };
        add_human(scene, &self.rider, transform);
    }
}

fn add_human(scene: &mut RenderScene, parts: &[HumanPart], transform: MathTransform3) {
    for part in parts {
        let mut item = RenderScene::item_from_dynamic_mesh(empty_mesh(), [1.0; 4]);
        item.mesh = Some(Arc::clone(&part.mesh));
        item.transform = transform;
        item.material = part.material.clone();
        item.base_color_texture = part.texture.clone();
        scene.items.push(item);
    }
}

fn empty_mesh() -> TriangleMesh {
    TriangleMesh {
        positions: Vec::new(),
        normals: Vec::new(),
        texcoords: Vec::new(),
        indices: Vec::new(),
        skinning: None,
    }
}

/// Camera optical frame (x right, y down, z forward) to `LiDAR` frame (x
/// forward, y left, z up), row-major; the camera sits above the `LiDAR`.
pub(crate) fn camera_to_lidar() -> Vec<f64> {
    vec![
        0.0,
        0.0,
        1.0,
        0.0, //
        -1.0,
        0.0,
        0.0,
        0.0, //
        0.0,
        -1.0,
        0.0,
        CAMERA_HEIGHT_M - SENSOR_HEIGHT_M, //
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

/// Pinhole intrinsics matching the renderer's camera.
pub(crate) fn intrinsics() -> Value {
    let f = f64::from(IMAGE_HEIGHT) / 2.0 / (FOV_Y_RAD / 2.0).tan();
    json!({
        "fx": f, "fy": f,
        "cx": f64::from(IMAGE_WIDTH) / 2.0, "cy": f64::from(IMAGE_HEIGHT) / 2.0,
        "skew": 0.0, "image_width": IMAGE_WIDTH, "image_height": IMAGE_HEIGHT,
    })
}

/// A box centred at `center` (`map` frame) with size `[length, width,
/// height]`, its length along heading `yaw`.
fn add_box(scene: &mut RenderScene, center: [f64; 3], size: [f64; 3], yaw: f64, color: [f32; 3]) {
    scene.items.push(primitive(
        VisualShape::Box {
            size_m: Vec3::new(size[0], size[2], size[1]),
        },
        center,
        yaw_rotation(yaw),
        color,
    ));
}

fn primitive(
    shape: VisualShape,
    center: [f64; 3],
    rotation: DQuat,
    color: [f32; 3],
) -> RenderSceneItem {
    let mut item = RenderScene::item_from_visual(
        rne_world::Transform3::from_translation_rotation(Vec3::ZERO, DQuat::IDENTITY),
        shape,
        [color[0], color[1], color[2], 1.0],
        rne_world::Transform3::IDENTITY,
    );
    item.transform.translation = to_engine(center[0], center[1], center[2]);
    item.transform.rotation = rotation;
    item
}

/// A vehicle (object frame: +x forward, y up) on the ground at `position`.
fn add_vehicle(scene: &mut RenderScene, parts: &[VehiclePart], position: [f64; 2], heading: f64) {
    for (mesh, color) in parts {
        let mut item =
            RenderScene::item_from_dynamic_mesh(empty_mesh(), [color[0], color[1], color[2], 1.0]);
        item.mesh = Some(Arc::clone(mesh));
        item.transform = MathTransform3 {
            translation: to_engine(position[0], position[1], 0.0),
            rotation: yaw_rotation(heading),
            scale: Vec3::ONE,
        };
        scene.items.push(item);
    }
}

/// The camera frame record (everything but the image).
pub(crate) fn frame_record(timestamp_ns: i64, image: &str, frame: &CameraFrame) -> Value {
    json!({
        "type": "camera_frame",
        "timestamp_ns": timestamp_ns,
        "image": image,
        "objects": frame.objects.iter().map(|(id, class, bbox, pixels)| json!({
            "truth_id": id, "class": class, "bbox": bbox, "visible_pixels": pixels,
        })).collect::<Vec<_>>(),
    })
}
