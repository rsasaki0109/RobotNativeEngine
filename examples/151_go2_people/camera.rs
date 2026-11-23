//! A forward camera on the Go2, rendered on the CPU by `rne_render_cpu`, and
//! a pretrained VLM (an external process) that finds people in its images.
//!
//! People are the textured CC0 human mesh in a standing pose; walls and the
//! floor are boxes. The LiDAR still scans the physics colliders (capsules and
//! boxes). The VLM process speaks one JSON line each way: `{"image": path}`
//! in, `{"people": [[x0, y0, x1, y1], ...]}` (pixels) out.

use std::f64::consts::FRAC_PI_2;
use std::io::{BufRead, BufReader, Write as _};
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::sync::Arc;

use rne_math::{DQuat, Transform3 as MathTransform3, Vec3};
use rne_render::{
    load_gltf_scene, Camera, ImageFrame, PbrMaterial, RenderScene, RenderSceneItem, TriangleMesh,
    VisualShape,
};
use rne_render_cpu::{CpuRayTracer, CpuRayTracerConfig};
use serde_json::{json, Value};

use crate::{Pose, Wall};

const HUMAN_GLB: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../assets/fixtures/kick_human/cc0_sport_human.glb"
);

/// The camera: on the Go2's head, looking forward, level.
const CAMERA_FORWARD_M: f64 = 0.3;
const CAMERA_HEIGHT_M: f64 = 0.45;
pub(crate) const IMAGE_WIDTH: u32 = 640;
pub(crate) const IMAGE_HEIGHT: u32 = 480;
const FOV_Y_RAD: f64 = 60.0 * std::f64::consts::PI / 180.0;
const WALL_HEIGHT_M: f64 = 1.2;
const SKY: [f32; 4] = [0.45, 0.6, 0.85, 1.0];

struct HumanPart {
    mesh: Arc<TriangleMesh>,
    texture: Option<Arc<ImageFrame>>,
    material: PbrMaterial,
}

/// The person mesh and the renderer.
pub(crate) struct CameraRig {
    human: Vec<HumanPart>,
    tracer: CpuRayTracer,
    camera: Camera,
}

impl CameraRig {
    pub(crate) fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let asset = load_gltf_scene(Path::new(HUMAN_GLB))?;
        let standing: Vec<MathTransform3> = match asset.animations.first() {
            Some(clip) => clip.sample_node_transforms(&asset.nodes, 0.0),
            None => asset.nodes.iter().map(|n| n.bind_transform).collect(),
        };
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
                    mesh: Arc::new(asset.sample_part_with_node_transforms(index, &standing)?),
                    texture: part.render_part.base_color_texture.clone().map(Arc::new),
                    material,
                })
            })
            .collect::<Result<Vec<_>, rne_render::AnimationSampleError>>()?;
        Ok(Self {
            human,
            tracer: CpuRayTracer::new(CpuRayTracerConfig::default()),
            camera: Camera {
                width: IMAGE_WIDTH,
                height: IMAGE_HEIGHT,
                fov_y_rad: FOV_Y_RAD,
                near_m: 0.1,
                far_m: 60.0,
            },
        })
    }

    /// Renders RGBA8 from the robot at `robot` (world plane), with `walls`
    /// and `people` as (position, velocity) in the world plane.
    pub(crate) fn render(
        &self,
        robot: Pose,
        walls: &[Wall],
        people: &[([f64; 2], [f64; 2])],
    ) -> Vec<u8> {
        let mut scene = RenderScene::new();
        add_box(
            &mut scene,
            [robot[0], robot[1], -0.05],
            [200.0, 200.0, 0.1],
            0.0,
            [0.32, 0.32, 0.3],
        );
        for wall in walls {
            add_box(
                &mut scene,
                [wall.center[0], wall.center[1], WALL_HEIGHT_M / 2.0],
                [2.0 * wall.half[0], 2.0 * wall.half[1], WALL_HEIGHT_M],
                0.0,
                [0.62, 0.6, 0.58],
            );
        }
        for (position, velocity) in people {
            let heading = if velocity[0].hypot(velocity[1]) > 0.05 {
                velocity[1].atan2(velocity[0])
            } else {
                // Standing people face the robot.
                (robot[1] - position[1]).atan2(robot[0] - position[0])
            };
            let transform = MathTransform3 {
                translation: to_engine(position[0], position[1], 0.0),
                rotation: DQuat::from_rotation_y(heading) * DQuat::from_rotation_y(FRAC_PI_2),
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
        let (sin, cos) = robot[2].sin_cos();
        let view = MathTransform3 {
            translation: to_engine(
                robot[0] + cos * CAMERA_FORWARD_M,
                robot[1] + sin * CAMERA_FORWARD_M,
                CAMERA_HEIGHT_M,
            ),
            // The camera looks down its local −Z; turn that onto the heading.
            rotation: DQuat::from_rotation_y(robot[2]) * DQuat::from_rotation_y(-FRAC_PI_2),
            scale: Vec3::ONE,
        };
        self.tracer
            .render(&self.camera, &view, &scene, SKY)
            .color
            .rgba8
    }
}

/// The image column (pixels) of a point at `local` (base frame, x forward,
/// y left), or `None` behind the camera.
pub(crate) fn column(local: [f64; 2]) -> Option<f64> {
    let depth = local[0] - CAMERA_FORWARD_M;
    if depth < 0.2 {
        return None;
    }
    let f = f64::from(IMAGE_HEIGHT) / 2.0 / (FOV_Y_RAD / 2.0).tan();
    let u = f64::from(IMAGE_WIDTH) / 2.0 - f * local[1] / depth;
    (0.0..f64::from(IMAGE_WIDTH)).contains(&u).then_some(u)
}

/// The VLM as a child process.
pub(crate) struct Vlm {
    child: Child,
    stdin: ChildStdin,
    stdout: BufReader<ChildStdout>,
    dir: PathBuf,
    count: u64,
}

impl Vlm {
    pub(crate) fn spawn(command: &[String], dir: &Path) -> std::io::Result<Self> {
        let (program, args) = command
            .split_first()
            .ok_or_else(|| std::io::Error::other("empty VLM command"))?;
        let mut child = Command::new(program)
            .args(args)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()?;
        let stdin = child
            .stdin
            .take()
            .ok_or_else(|| std::io::Error::other("no stdin"))?;
        let stdout = BufReader::new(
            child
                .stdout
                .take()
                .ok_or_else(|| std::io::Error::other("no stdout"))?,
        );
        std::fs::create_dir_all(dir)?;
        Ok(Self {
            child,
            stdin,
            stdout,
            dir: dir.to_path_buf(),
            count: 0,
        })
    }

    /// People boxes in an image, in pixels.
    pub(crate) fn people(&mut self, rgba8: &[u8]) -> std::io::Result<Vec<[f64; 4]>> {
        let path = self.dir.join(format!("frame_{:05}.png", self.count));
        self.count += 1;
        write_png(&path, rgba8, IMAGE_WIDTH, IMAGE_HEIGHT)?;
        writeln!(self.stdin, "{}", json!({ "image": path }))?;
        self.stdin.flush()?;
        let mut line = String::new();
        self.stdout.read_line(&mut line)?;
        let reply: Value = serde_json::from_str(&line).map_err(std::io::Error::other)?;
        Ok(reply["people"]
            .as_array()
            .map(|boxes| {
                boxes
                    .iter()
                    .filter_map(|b| {
                        let v: Vec<f64> = b.as_array()?.iter().filter_map(Value::as_f64).collect();
                        (v.len() == 4).then(|| [v[0], v[1], v[2], v[3]])
                    })
                    .collect()
            })
            .unwrap_or_default())
    }
}

impl Drop for Vlm {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

fn write_png(path: &Path, rgba8: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let mut encoder = png::Encoder::new(std::fs::File::create(path)?, width, height);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder
        .write_header()
        .and_then(|mut writer| writer.write_image_data(rgba8))
        .map_err(std::io::Error::other)
}

fn to_engine(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(x, z, -y)
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

/// A box centred at `center` (world plane, z up) with size `[x, y, z]`.
fn add_box(scene: &mut RenderScene, center: [f64; 3], size: [f64; 3], yaw: f64, color: [f32; 3]) {
    scene.items.push(primitive(
        VisualShape::Box {
            size_m: Vec3::new(size[0], size[2], size[1]),
        },
        center,
        DQuat::from_rotation_y(yaw),
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn columns_follow_the_bearing() {
        let centre = f64::from(IMAGE_WIDTH) / 2.0;
        assert_eq!(column([3.0, 0.0]), Some(centre));
        // To the left of the axis: left of the image centre.
        assert!(column([3.0, 0.5]).is_some_and(|u| u < centre));
        assert!(column([3.0, -0.5]).is_some_and(|u| u > centre));
        // Behind the camera or outside the field of view.
        assert_eq!(column([0.2, 0.0]), None);
        assert_eq!(column([1.0, 3.0]), None);
    }
}
