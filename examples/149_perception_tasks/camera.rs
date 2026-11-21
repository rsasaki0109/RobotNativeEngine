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
    load_gltf_scene, Camera, GltfAnimationPlayer, ImageFrame, PbrMaterial, RenderScene,
    RenderSceneItem, TriangleMesh, VisualShape,
};
use rne_render_cpu::{CpuRayTracer, CpuRayTracerConfig};
use serde_json::{json, Value};

use crate::{to_engine, yaw_rotation, Kind, Pose2, Task, SENSOR_HEIGHT_M};

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
/// A camera frame every this many LiDAR scans (2 Hz).
pub(crate) const SCANS_PER_FRAME: u64 = 5;
/// Objects with fewer visible pixels are left out of the 2D truth.
const MIN_VISIBLE_PIXELS: usize = 30;
const SKY: [f32; 4] = [0.45, 0.6, 0.85, 1.0];

/// One person mesh part: geometry and material, shared by every copy.
struct HumanPart {
    mesh: Arc<TriangleMesh>,
    texture: Option<Arc<ImageFrame>>,
    material: PbrMaterial,
}

/// The camera, the person asset and the renderer.
pub(crate) struct CameraRig {
    human: Vec<HumanPart>,
    tracer: CpuRayTracer,
    camera: Camera,
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
        let player = GltfAnimationPlayer::new((!asset.animations.is_empty()).then_some(0));
        let human = asset
            .parts
            .iter()
            .enumerate()
            .map(|(index, part)| {
                let mut material = part.render_part.material.clone();
                if let Some(color) = part.render_part.base_color_rgba {
                    material.base_color_rgba = color;
                }
                Ok(HumanPart {
                    mesh: Arc::new(player.sample_part_cpu(&asset, index)?),
                    texture: part.render_part.base_color_texture.clone().map(Arc::new),
                    material,
                })
            })
            .collect::<Result<_, rne_render::AnimationSampleError>>()?;
        Ok(Self {
            human,
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
                Kind::Person => self.add_person(&mut scene, position, heading, 0.0),
                Kind::Cyclist => {
                    add_bicycle(&mut scene, position, heading);
                    self.add_person(&mut scene, position, heading, 0.25);
                }
                Kind::Car => add_car(&mut scene, position, heading, index),
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

    fn add_person(&self, scene: &mut RenderScene, position: [f64; 2], heading: f64, lift_m: f64) {
        // The mesh faces glTF +Z; turn that onto the heading.
        let transform = MathTransform3 {
            translation: to_engine(position[0], position[1], lift_m),
            rotation: yaw_rotation(heading) * DQuat::from_rotation_y(FRAC_PI_2),
            scale: Vec3::ONE,
        };
        for part in &self.human {
            let mut item = RenderScene::item_from_dynamic_mesh(empty_mesh(), [1.0; 4]);
            item.mesh = Some(Arc::clone(&part.mesh));
            item.transform = transform;
            item.material = part.material.clone();
            item.base_color_texture = part.texture.clone();
            scene.items.push(item);
        }
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

/// A point `forward` along and `left` across the heading from `position`.
fn offset(position: [f64; 2], heading: f64, forward: f64, left: f64) -> [f64; 2] {
    let (sin, cos) = heading.sin_cos();
    [
        position[0] + cos * forward - sin * left,
        position[1] + sin * forward + cos * left,
    ]
}

/// A wheel: a cylinder whose axis runs across the heading.
fn add_wheel(
    scene: &mut RenderScene,
    at: [f64; 2],
    heading: f64,
    radius: f64,
    width: f64,
    color: [f32; 3],
) {
    // Cylinders run along local Z (engine); turn Z onto the lateral axis.
    let rotation = yaw_rotation(heading);
    scene.items.push(primitive(
        VisualShape::Cylinder {
            radius_m: radius,
            length_m: width,
        },
        [at[0], at[1], radius],
        rotation,
        color,
    ));
}

fn add_car(scene: &mut RenderScene, position: [f64; 2], heading: f64, index: usize) {
    const PAINT: [[f32; 3]; 4] = [
        [0.6, 0.05, 0.05],
        [0.8, 0.8, 0.82],
        [0.05, 0.12, 0.35],
        [0.1, 0.1, 0.1],
    ];
    let paint = PAINT[index % PAINT.len()];
    let glass = [0.08, 0.1, 0.13];
    // Body, cabin (glass), roof.
    add_box(
        scene,
        [position[0], position[1], 0.65],
        [4.4, 1.8, 0.6],
        heading,
        paint,
    );
    let cabin = offset(position, heading, -0.3, 0.0);
    add_box(
        scene,
        [cabin[0], cabin[1], 1.15],
        [2.3, 1.6, 0.42],
        heading,
        glass,
    );
    add_box(
        scene,
        [cabin[0], cabin[1], 1.4],
        [2.1, 1.5, 0.08],
        heading,
        paint,
    );
    for (forward, left) in [(1.35, 0.8), (1.35, -0.8), (-1.35, 0.8), (-1.35, -0.8)] {
        add_wheel(
            scene,
            offset(position, heading, forward, left),
            heading,
            0.33,
            0.24,
            [0.05, 0.05, 0.05],
        );
    }
}

fn add_bicycle(scene: &mut RenderScene, position: [f64; 2], heading: f64) {
    let frame = [0.1, 0.25, 0.6];
    for forward in [0.55, -0.55] {
        add_wheel(
            scene,
            offset(position, heading, forward, 0.0),
            heading,
            0.34,
            0.04,
            [0.05, 0.05, 0.05],
        );
    }
    add_box(
        scene,
        [position[0], position[1], 0.55],
        [1.0, 0.05, 0.05],
        heading,
        frame,
    );
    let bars = offset(position, heading, 0.45, 0.0);
    add_box(
        scene,
        [bars[0], bars[1], 0.85],
        [0.05, 0.55, 0.04],
        heading,
        frame,
    );
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
