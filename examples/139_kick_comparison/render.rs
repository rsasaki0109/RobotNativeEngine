//! Renders recorded robot poses without changing the physics trace.
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{
    load_gltf_scene, Camera, EnvironmentLighting, EnvironmentMap, GltfAnimationPlayer,
    GltfSceneAsset, MeshRenderCache, PbrMaterial, RenderBackend, RenderScene, RenderSceneItem,
    VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use serde::Deserialize;
use std::{
    fs,
    path::{Path, PathBuf},
    sync::Arc,
};

#[derive(Debug, Deserialize)]
struct Item {
    transform: Transform3,
    shape: VisualShape,
    color_rgba: [f32; 4],
}
#[derive(Debug, Deserialize)]
struct Frame {
    time_s: f64,
    items: Vec<Item>,
}
#[derive(Debug, Deserialize)]
struct Trace {
    frames: Vec<Frame>,
    mesh_package_roots: Vec<PathBuf>,
}

fn scene_from_frame(frame: &Frame) -> RenderScene {
    let mut items: Vec<_> = frame
        .items
        .iter()
        .filter(|v| !matches!(v.shape, VisualShape::Box { .. }))
        .map(|v| RenderSceneItem {
            transform: v.transform,
            shape: v.shape.clone(),
            color_rgba: v.color_rgba,
            mesh: None,
            base_color_texture: None,
            material: PbrMaterial::default(),
        })
        .collect();
    items.sort_by_cached_key(|item| {
        serde_json::to_string(&(item.transform, &item.shape, item.color_rgba))
            .expect("finite recorded visual item")
    });
    RenderScene { items }
}

fn floor(scene: &mut RenderScene) {
    scene.items.push(RenderSceneItem {
        transform: Transform3 {
            translation: Vec3::new(0.0, -0.01, 0.0),
            rotation: Quat::IDENTITY,
            scale: Vec3::new(5.0, 0.02, 5.0),
        },
        shape: VisualShape::Box {
            size_m: Vec3::new(5.0, 0.02, 5.0),
        },
        color_rgba: [0.22, 0.24, 0.27, 1.0],
        mesh: None,
        base_color_texture: None,
        material: PbrMaterial::new([1.0; 4], 0.85, 0.0, [0.0; 3]),
    });
}

fn human(scene: &mut RenderScene, asset: &GltfSceneAsset, time_s: f64, origin: Vec3, high: bool) {
    let wanted = if high { "mid_kick" } else { "low_kick" };
    let clip = asset
        .animations
        .iter()
        .position(|v| v.name.as_deref().is_some_and(|n| n.contains(wanted)))
        .or_else(|| (!asset.animations.is_empty()).then_some(0));
    let player = GltfAnimationPlayer {
        animation_index: clip,
        time_s: time_s as f32,
        playback_rate: 1.0,
    };
    for (i, part) in asset.parts.iter().enumerate() {
        let mesh = player
            .sample_part_for_gpu(asset, i)
            .expect("sample human skin");
        let mut item = RenderScene::item_from_dynamic_mesh(mesh, [1.0; 4]);
        item.transform.translation = origin;
        item.material = part.render_part.material.clone();
        item.base_color_texture = part.render_part.base_color_texture.clone().map(Arc::new);
        if let Some(color) = part.render_part.base_color_rgba {
            item.material.base_color_rgba = color;
        }
        scene.items.push(item);
    }
}

fn write_png(path: &Path, bytes: &[u8], width: u32, height: u32) {
    let mut encoder =
        png::Encoder::new(fs::File::create(path).expect("create frame"), width, height);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder
        .write_header()
        .expect("png header")
        .write_image_data(bytes)
        .expect("png pixels");
}

fn measured_human_origin(path: &Path) -> Vec3 {
    let value: serde_json::Value =
        serde_json::from_slice(&fs::read(path).expect("trace bytes")).expect("trace data");
    for collection in ["frames", "observed_telemetry"] {
        if let Some(entries) = value[collection].as_array() {
            for entry in entries {
                for field in ["point_world_m", "application_point_world_m"] {
                    if let Some(point) = entry[field].as_array() {
                        return Vec3::new(
                            point[0].as_f64().expect("point x") + 0.196_567_6,
                            0.0,
                            point[2].as_f64().expect("point z") - 1.0,
                        );
                    }
                }
            }
        }
    }
    panic!("trace must include the measured force application point");
}

pub(super) fn render(
    output: &Path,
    human_path: &Path,
    start_frame: usize,
    frame_count: usize,
) -> Result<(), Box<dyn std::error::Error>> {
    let left_path = output.join("go2-trace.json");
    let right_path = output.join("g1-trace.json");
    let left: Trace = serde_json::from_slice(&fs::read(&left_path)?)?;
    let right: Trace = serde_json::from_slice(&fs::read(&right_path)?)?;
    let output_dir = output.join("frames");
    fs::create_dir_all(&output_dir)?;
    let person = load_gltf_scene(human_path)?;
    let left_human_origin = measured_human_origin(&left_path);
    let right_human_origin = measured_human_origin(&right_path);
    let mut backend = WgpuRenderBackend::new()?;
    let map = EnvironmentMap::solid([0.22, 0.25, 0.30, 1.0]).expect("studio environment");
    let mut lighting = EnvironmentLighting::from_map(Arc::new(map));
    lighting.diffuse_strength = 0.75;
    lighting.specular_strength = 0.25;
    backend.set_environment(lighting);
    let mut cache = MeshRenderCache::new();
    let camera = Camera::new(480, 420, std::f64::consts::FRAC_PI_4);
    let roots_left: Vec<_> = left
        .mesh_package_roots
        .iter()
        .map(PathBuf::as_path)
        .collect();
    let roots_right: Vec<_> = right
        .mesh_package_roots
        .iter()
        .map(PathBuf::as_path)
        .collect();
    assert_eq!(
        left.frames.len(),
        right.frames.len(),
        "synchronized frame count"
    );
    let left_start = left.frames.first().expect("Go2 frames").time_s;
    let right_start = right.frames.first().expect("G1 frames").time_s;
    for (i, (lf, rf)) in left
        .frames
        .iter()
        .zip(&right.frames)
        .enumerate()
        .skip(start_frame)
        .take(frame_count)
    {
        assert!(
            ((lf.time_s - left_start) - (rf.time_s - right_start)).abs() < 1e-9,
            "synchronized simulation clock"
        );
        let mut rendered = Vec::new();
        for (frame, roots, start, high, origin) in [
            (lf, &roots_left, left_start, false, left_human_origin),
            (rf, &roots_right, right_start, true, right_human_origin),
        ] {
            let mut scene = scene_from_frame(frame);
            cache
                .resolve_scene(&mut scene, roots)
                .expect("official robot meshes");
            floor(&mut scene);
            human(&mut scene, &person, frame.time_s - start, origin, high);
            let orbit = CameraOrbit {
                focus: Vec3::new(0.0, 0.75, -0.25),
                yaw_rad: -1.0,
                pitch_rad: 1.30,
                distance_m: 3.3,
            };
            rendered.push(
                backend
                    .render_scene_camera(
                        &camera,
                        &orbit.camera_transform(),
                        &scene,
                        [0.075, 0.09, 0.115, 1.0],
                    )
                    .expect("render measured panel")
                    .color
                    .rgba8,
            );
        }
        let mut composite = vec![0_u8; 960 * 420 * 4];
        for y in 0..420 {
            for side in 0..2 {
                composite[(y * 960 + side * 480) * 4..(y * 960 + (side + 1) * 480) * 4]
                    .copy_from_slice(&rendered[side][y * 480 * 4..(y + 1) * 480 * 4]);
            }
        }
        write_png(
            &output_dir.join(format!("frame-{i:03}.png")),
            &composite,
            960,
            420,
        );
        if i % 30 == 0 {
            println!("rendered frame {i}/{}", left.frames.len());
        }
    }
    Ok(())
}
