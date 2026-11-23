//! Replays externally simulated primitive scenes through the wgpu renderer.
//!
//! An external simulator (for example a Python research sandbox) writes a JSON
//! script of static items, and per frame the dynamic items and camera views to
//! render. This example renders every view with the RNE PBR pipeline and writes
//! PNG files, so a lightweight simulator can borrow RNE's renderer for media
//! without linking against it.
//!
//! ```text
//! cargo run -p scene_replay_render --example 150_scene_replay_render -- script.json out_dir
//! ```
//!
//! Without arguments, or with `RNE_SKIP_GPU` set, it renders a built-in one-frame
//! demo (or skips GPU work) so CI can exercise the JSON path headlessly.
//!
//! JSON schema (all lengths in meters, Y up, quaternions `[x, y, z, w]`):
//!
//! ```text
//! {
//!   "clear_rgba": [r, g, b, a],
//!   "static_items": [Item],
//!   "frames": [{ "items": [Item], "views": [View] }]
//! }
//! Item = { "shape": "box" | "sphere" | "cylinder" | "mesh", "size_m": [x, y, z],
//!          "radius_m": f, "length_m": f, "pos_m": [x, y, z], "rot": [x, y, z, w],
//!          "color_rgba": [r, g, b, a], "roughness": f, "emissive_rgb": [r, g, b],
//!          "path": "model.gltf", "scale": [x, y, z], "flat_color": bool }
//! ```
//!
//! A `"mesh"` item loads an STL, OBJ, glTF or GLB file; a relative `path` resolves
//! against the script's directory. Its own materials and textures are used unless
//! `flat_color` is set, which replaces them with `color_rgba` (for objects whose
//! colour carries meaning, such as an instructed target).
//!
//! ```text
//! View = { "out": "file.png", "width": u, "height": u, "fov_y_rad": f,
//!          "eye_m": [x, y, z], "rot": [x, y, z, w] }   // camera looks down local -Z
//! ```

use rne_math::{Quat, Vec3};
use rne_render::{
    Camera, EnvironmentLighting, EnvironmentMap, PbrMaterial, RenderBackend, RenderScene,
    RenderSceneItem, VisualShape,
};
use rne_render_wgpu::WgpuRenderBackend;
use rne_world::Transform3;
use serde::Deserialize;
use std::path::{Path, PathBuf};
use std::sync::Arc;

/// One primitive in the replay script.
#[derive(Clone, Debug, Deserialize)]
struct Item {
    shape: String,
    #[serde(default)]
    size_m: Option<[f64; 3]>,
    #[serde(default)]
    radius_m: Option<f64>,
    #[serde(default)]
    length_m: Option<f64>,
    pos_m: [f64; 3],
    #[serde(default = "identity_rot")]
    rot: [f64; 4],
    color_rgba: [f32; 4],
    #[serde(default = "default_roughness")]
    roughness: f32,
    #[serde(default)]
    emissive_rgb: [f32; 3],
    #[serde(default)]
    path: Option<String>,
    #[serde(default)]
    scale: Option<[f64; 3]>,
    #[serde(default)]
    flat_color: bool,
}

/// One camera view to render.
#[derive(Clone, Debug, Deserialize)]
struct View {
    out: String,
    width: u32,
    height: u32,
    fov_y_rad: f64,
    eye_m: [f64; 3],
    rot: [f64; 4],
}

/// Dynamic items and views of one frame.
#[derive(Clone, Debug, Deserialize)]
struct Frame {
    #[serde(default)]
    items: Vec<Item>,
    views: Vec<View>,
}

/// Whole replay script.
#[derive(Clone, Debug, Deserialize)]
struct Script {
    #[serde(default = "default_clear")]
    clear_rgba: [f32; 4],
    #[serde(default)]
    static_items: Vec<Item>,
    frames: Vec<Frame>,
}

fn identity_rot() -> [f64; 4] {
    [0.0, 0.0, 0.0, 1.0]
}

fn default_roughness() -> f32 {
    0.7
}

fn default_clear() -> [f32; 4] {
    [0.07, 0.08, 0.1, 1.0]
}

fn quat(r: [f64; 4]) -> Quat {
    Quat::from_xyzw(r[0], r[1], r[2], r[3]).normalize()
}

fn to_render_item(item: &Item) -> Result<RenderSceneItem, String> {
    let shape = match item.shape.as_str() {
        "box" => VisualShape::Box {
            size_m: Vec3::from_array(item.size_m.ok_or("box needs size_m")?),
        },
        "sphere" => VisualShape::Sphere {
            radius_m: item.radius_m.ok_or("sphere needs radius_m")?,
        },
        "cylinder" => VisualShape::Cylinder {
            radius_m: item.radius_m.ok_or("cylinder needs radius_m")?,
            length_m: item.length_m.ok_or("cylinder needs length_m")?,
        },
        "mesh" => VisualShape::Mesh {
            path: item.path.clone().ok_or("mesh needs path")?,
            scale: Vec3::from_array(item.scale.unwrap_or([1.0; 3])),
        },
        other => return Err(format!("unknown shape {other}")),
    };
    let world = Transform3::from_translation_rotation(Vec3::from_array(item.pos_m), quat(item.rot));
    let material = PbrMaterial {
        base_color_rgba: item.color_rgba,
        roughness: item.roughness.clamp(0.0, 1.0),
        emissive_rgb: item.emissive_rgb,
        ..PbrMaterial::default()
    };
    Ok(RenderScene::item_from_visual_with_material(
        world,
        shape,
        item.color_rgba,
        Transform3::IDENTITY,
        material,
    ))
}

/// Soft indoor sky: bright ceiling, neutral walls, warm floor bounce.
fn indoor_environment() -> EnvironmentLighting {
    const W: u32 = 64;
    const H: u32 = 32;
    let mut rgba = Vec::with_capacity((W * H * 4) as usize);
    for row in 0..H {
        let down = (row as f32 + 0.5) / H as f32;
        for _ in 0..W {
            let (r, g, b) = if down < 0.35 {
                let level = 1.1 + 0.6 * (1.0 - down / 0.35);
                (level, level, level * 1.02)
            } else if down < 0.6 {
                let level = 0.55 - 0.3 * (down - 0.35) / 0.25;
                (level * 0.97, level, level * 1.03)
            } else {
                (0.2, 0.18, 0.16)
            };
            rgba.extend_from_slice(&[r, g, b, 1.0]);
        }
    }
    let map = EnvironmentMap::from_rgba32f(W, H, rgba).expect("indoor environment map");
    EnvironmentLighting {
        map: Some(Arc::new(map)),
        intensity: 1.0,
        // modest fill so the directional light's shadow map stays visible
        diffuse_strength: 0.42,
        specular_strength: 0.3,
        rotation_rad: 0.0,
    }
}

/// Converts items and loads mesh files (relative paths resolve against `root`).
fn render_items(items: &[Item], root: &Path) -> Result<Vec<RenderSceneItem>, String> {
    let mut out = Vec::with_capacity(items.len());
    for item in items {
        let render = to_render_item(item)?;
        if !matches!(render.shape, VisualShape::Mesh { .. }) {
            out.push(render);
            continue;
        }
        let mut scene = RenderScene::new();
        scene.items.push(render);
        scene
            .resolve_mesh_assets_with_roots(&[root])
            .map_err(|error| format!("mesh {:?}: {error}", item.path))?;
        for mut part in scene.items {
            if item.flat_color {
                part.base_color_texture = None;
                part.color_rgba = item.color_rgba;
                part.material.base_color_rgba = [1.0; 4];
                part.material.roughness = item.roughness.clamp(0.0, 1.0);
            }
            out.push(part);
        }
    }
    Ok(out)
}

fn write_png(path: &Path, width: u32, height: u32, rgba: &[u8]) -> std::io::Result<()> {
    let file = std::io::BufWriter::new(std::fs::File::create(path)?);
    let mut encoder = png::Encoder::new(file, width, height);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().map_err(std::io::Error::other)?;
    writer.write_image_data(rgba).map_err(std::io::Error::other)
}

fn demo_script() -> Script {
    serde_json::from_str(
        r#"{
          "static_items": [
            {"shape": "box", "size_m": [4, 0.02, 4], "pos_m": [0, -0.01, 0], "color_rgba": [0.5, 0.45, 0.4, 1]},
            {"shape": "box", "size_m": [0.5, 0.5, 0.5], "pos_m": [0, 0.25, 0], "color_rgba": [0.8, 0.2, 0.2, 1]}
          ],
          "frames": [{"items": [{"shape": "sphere", "radius_m": 0.2, "pos_m": [0.8, 0.2, 0.3], "color_rgba": [0.2, 0.4, 0.9, 1]}],
                      "views": [{"out": "demo.png", "width": 96, "height": 64, "fov_y_rad": 0.9,
                                 "eye_m": [0, 1.5, 3], "rot": [-0.2588, 0, 0, 0.9659]}]}]
        }"#,
    )
    .expect("demo script parses")
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (script, out_dir, root) = if args.len() >= 2 {
        let text = std::fs::read_to_string(&args[0]).expect("read replay script");
        let script: Script = serde_json::from_str(&text).expect("parse replay script");
        let root = Path::new(&args[0])
            .parent()
            .map(Path::to_path_buf)
            .unwrap_or_default();
        (script, PathBuf::from(&args[1]), root)
    } else {
        (
            demo_script(),
            std::env::temp_dir().join("rne_scene_replay_demo"),
            PathBuf::new(),
        )
    };
    let static_items = render_items(&script.static_items, &root).expect("valid static items");
    let views: usize = script.frames.iter().map(|f| f.views.len()).sum();
    println!(
        "replay script: {} static items, {} frames, {} views",
        static_items.len(),
        script.frames.len(),
        views
    );
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        for frame in &script.frames {
            for item in &frame.items {
                to_render_item(item).expect("valid frame item");
            }
        }
        println!("RNE_SKIP_GPU set; validated script without rendering");
        return;
    }
    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("wgpu unavailable: {error}");
            return;
        }
    };
    backend.set_environment(indoor_environment());
    std::fs::create_dir_all(&out_dir).expect("create output directory");
    let clear = script.clear_rgba;
    for frame in &script.frames {
        let mut scene = RenderScene::new();
        scene.items.extend(static_items.iter().cloned());
        scene
            .items
            .extend(render_items(&frame.items, &root).expect("valid frame items"));
        for view in &frame.views {
            let camera = Camera::new(view.width, view.height, view.fov_y_rad);
            let pose = rne_math::Transform3::from_translation_rotation(
                Vec3::from_array(view.eye_m),
                quat(view.rot),
            );
            let output = backend
                .render_scene_camera(&camera, &pose, &scene, clear)
                .expect("render view");
            write_png(
                &out_dir.join(&view.out),
                view.width,
                view.height,
                &output.color.rgba8,
            )
            .expect("write PNG");
        }
    }
    println!("rendered {views} views into {}", out_dir.display());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn demo_script_items_convert_to_render_items() {
        let script = demo_script();
        assert_eq!(script.frames.len(), 1);
        for item in script.static_items.iter().chain(&script.frames[0].items) {
            let render = to_render_item(item).expect("valid item");
            assert_eq!(render.color_rgba, item.color_rgba);
        }
    }

    #[test]
    fn mesh_item_needs_a_path() {
        let mut item = demo_script().static_items[0].clone();
        item.shape = "mesh".to_owned();
        assert!(to_render_item(&item).is_err());
        item.path = Some("model.gltf".to_owned());
        assert!(matches!(
            to_render_item(&item).map(|r| r.shape),
            Ok(VisualShape::Mesh { .. })
        ));
    }

    #[test]
    fn unknown_shape_is_rejected() {
        let mut item = demo_script().static_items[0].clone();
        item.shape = "torus".to_owned();
        assert!(to_render_item(&item).is_err());
    }
}
