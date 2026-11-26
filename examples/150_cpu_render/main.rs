//! A street scene with textured people rendered on the CPU by `rne_render_cpu`.
//!
//! Writes a PNG, the depth range, and the exact 2D box of every person from the
//! renderer's item-id map, then times the render on one thread and on all of
//! them (the images are byte-identical).
//!
//! ```bash
//! cargo run --release -p cpu_render --example 150_cpu_render -- --out target/cpu_render
//! ```

use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

use rne_math::{DQuat, Transform3, Vec3};
use rne_render::{
    load_gltf_scene, Camera, GltfAnimationPlayer, GltfSceneAsset, RenderScene, VisualShape,
};
use rne_render_cpu::{CpuRayTracer, CpuRayTracerConfig};
use serde_json::json;

const HUMAN_GLB: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../assets/fixtures/kick_human/cc0_sport_human.glb"
);
const SKY: [f32; 4] = [0.45, 0.6, 0.85, 1.0];

/// A person standing at `(x, z)` on the ground (engine y-up), facing `yaw`.
struct Person {
    x: f64,
    z: f64,
    yaw_rad: f64,
    tint: [f32; 3],
}

fn add_box(scene: &mut RenderScene, center: Vec3, size: Vec3, color: [f32; 3]) {
    scene.items.push(RenderScene::item_from_visual(
        rne_world::Transform3::from_translation_rotation(center, DQuat::IDENTITY),
        VisualShape::Box { size_m: size },
        [color[0], color[1], color[2], 1.0],
        rne_world::Transform3::IDENTITY,
    ));
}

/// Adds one person (every mesh part of the asset) and returns its item indices.
fn add_person(scene: &mut RenderScene, asset: &GltfSceneAsset, person: &Person) -> Vec<u32> {
    // The first frame of a clip is a relaxed standing pose.
    let player = GltfAnimationPlayer::new((!asset.animations.is_empty()).then_some(0));
    let mut items = Vec::new();
    for (index, part) in asset.parts.iter().enumerate() {
        let mesh = player.sample_part_cpu(asset, index).expect("sample person");
        let [r, g, b] = person.tint;
        let mut item = RenderScene::item_from_dynamic_mesh(mesh, [r, g, b, 1.0]);
        item.transform = Transform3 {
            translation: Vec3::new(person.x, 0.0, person.z),
            rotation: DQuat::from_rotation_y(person.yaw_rad),
            scale: Vec3::ONE,
        };
        item.material = part.render_part.material.clone();
        if let Some(color) = part.render_part.base_color_rgba {
            item.material.base_color_rgba = color;
        }
        item.base_color_texture = part.render_part.base_color_texture.clone().map(Arc::new);
        items.push(scene.items.len() as u32);
        scene.items.push(item);
    }
    items
}

/// A sidewalk scene; returns it with each person's item indices.
fn street(asset: &GltfSceneAsset, people: &[Person]) -> (RenderScene, Vec<Vec<u32>>) {
    let mut scene = RenderScene::new();
    // Road, sidewalk, kerb and two building fronts.
    add_box(
        &mut scene,
        Vec3::new(0.0, -0.05, -30.0),
        Vec3::new(8.0, 0.1, 80.0),
        [0.12, 0.12, 0.13],
    );
    add_box(
        &mut scene,
        Vec3::new(-7.0, -0.02, -30.0),
        Vec3::new(6.0, 0.1, 80.0),
        [0.45, 0.44, 0.42],
    );
    add_box(
        &mut scene,
        Vec3::new(7.0, -0.02, -30.0),
        Vec3::new(6.0, 0.1, 80.0),
        [0.45, 0.44, 0.42],
    );
    add_box(
        &mut scene,
        Vec3::new(-11.0, 6.0, -30.0),
        Vec3::new(2.0, 12.0, 80.0),
        [0.55, 0.38, 0.3],
    );
    add_box(
        &mut scene,
        Vec3::new(11.0, 5.0, -30.0),
        Vec3::new(2.0, 10.0, 80.0),
        [0.6, 0.6, 0.62],
    );
    // A parked van and a pole.
    add_box(
        &mut scene,
        Vec3::new(3.0, 1.0, -16.0),
        Vec3::new(1.9, 2.0, 4.8),
        [0.85, 0.85, 0.88],
    );
    add_box(
        &mut scene,
        Vec3::new(-4.2, 1.5, -9.0),
        Vec3::new(0.12, 3.0, 0.12),
        [0.3, 0.3, 0.3],
    );
    let ids = people
        .iter()
        .map(|p| add_person(&mut scene, asset, p))
        .collect();
    (scene, ids)
}

fn people() -> Vec<Person> {
    vec![
        Person {
            x: -1.5,
            z: -6.0,
            yaw_rad: 0.4,
            tint: [1.0, 1.0, 1.0],
        },
        Person {
            x: 1.2,
            z: -8.5,
            yaw_rad: -2.6,
            tint: [0.9, 0.95, 1.0],
        },
        Person {
            x: -5.0,
            z: -11.0,
            yaw_rad: 1.6,
            tint: [1.0, 0.9, 0.85],
        },
        Person {
            x: 0.3,
            z: -14.0,
            yaw_rad: 3.1,
            tint: [0.85, 1.0, 0.9],
        },
        Person {
            x: -6.5,
            z: -20.0,
            yaw_rad: 0.0,
            tint: [1.0, 1.0, 1.0],
        },
    ]
}

fn camera() -> (Camera, Transform3) {
    let camera = Camera {
        width: 640,
        height: 480,
        fov_y_rad: 60.0_f64.to_radians(),
        near_m: 0.1,
        far_m: 120.0,
    };
    // 1.3 m high, pitched down 4 degrees, looking down the street (−Z).
    let view = Transform3 {
        translation: Vec3::new(0.0, 1.3, 0.0),
        rotation: DQuat::from_rotation_x(-4.0_f64.to_radians()),
        scale: Vec3::ONE,
    };
    (camera, view)
}

fn write_png(
    path: &Path,
    rgba8: &[u8],
    width: u32,
    height: u32,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut encoder = png::Encoder::new(fs::File::create(path)?, width, height);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.write_header()?.write_image_data(rgba8)?;
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut out = PathBuf::from("target/cpu_render");
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--out" => out = PathBuf::from(args.next().ok_or("--out needs a directory")?),
            other => return Err(format!("unknown argument {other}").into()),
        }
    }
    fs::create_dir_all(&out)?;
    let asset = load_gltf_scene(Path::new(HUMAN_GLB))?;
    let people = people();
    let (scene, ids) = street(&asset, &people);
    let (camera, view) = camera();

    let mut timings = Vec::new();
    let mut reference = None;
    let available = std::thread::available_parallelism().map_or(1, std::num::NonZeroUsize::get);
    for threads in [1, available] {
        let tracer = CpuRayTracer::new(CpuRayTracerConfig {
            threads,
            ..CpuRayTracerConfig::default()
        });
        let start = Instant::now();
        let output = tracer.render(&camera, &view, &scene, SKY);
        let seconds = start.elapsed().as_secs_f64();
        println!("{threads} thread(s): {seconds:.3} s");
        timings.push(json!({"threads": threads, "seconds": seconds}));
        if let Some(first) = &reference {
            assert_eq!(&output, first, "output must not depend on the thread count");
        } else {
            reference = Some(output);
        }
    }
    let output = reference.expect("rendered");
    write_png(
        &out.join("street.png"),
        &output.color.rgba8,
        camera.width,
        camera.height,
    )?;

    let boxes: Vec<_> = ids
        .iter()
        .map(|items| {
            output
                .item_boxes(items)
                .into_iter()
                .flatten()
                .reduce(|a, b| {
                    [
                        a[0].min(b[0]),
                        a[1].min(b[1]),
                        a[2].max(b[2]),
                        a[3].max(b[3]),
                    ]
                })
        })
        .collect();
    let report = json!({
        "image": "street.png",
        "width": camera.width,
        "height": camera.height,
        "triangles_per_person": asset.parts.iter().map(|p| p.render_part.mesh.indices.len() / 3).sum::<usize>(),
        "people": people.iter().zip(&boxes).map(|(p, b)| json!({
            "position_m": [p.x, 0.0, p.z],
            "box_px": b,
        })).collect::<Vec<_>>(),
        "timings": timings,
    });
    fs::write(
        out.join("street.json"),
        serde_json::to_string_pretty(&report)?,
    )?;
    println!("wrote {}", out.join("street.png").display());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_person_is_visible_and_boxes_shrink_with_distance() {
        let asset = load_gltf_scene(Path::new(HUMAN_GLB)).unwrap();
        let people = people();
        let (scene, ids) = street(&asset, &people);
        let (mut camera, view) = camera();
        camera.width = 160;
        camera.height = 120;
        let tracer = CpuRayTracer::new(CpuRayTracerConfig {
            samples_per_axis: 1,
            ..CpuRayTracerConfig::default()
        });
        let output = tracer.render(&camera, &view, &scene, SKY);
        let heights: Vec<u32> = ids
            .iter()
            .map(|items| {
                let b = output
                    .item_boxes(items)
                    .into_iter()
                    .flatten()
                    .reduce(|a, b| {
                        [
                            a[0].min(b[0]),
                            a[1].min(b[1]),
                            a[2].max(b[2]),
                            a[3].max(b[3]),
                        ]
                    });
                let b = b.expect("person visible");
                b[3] - b[1]
            })
            .collect();
        // Nearest person (6 m) is taller in the image than the farthest (20 m).
        assert!(heights[0] > 2 * heights[4], "{heights:?}");
        // Pinhole check: a 1.75 m person at 6 m spans about f * 1.75 / 6 px.
        let f = 60.0 / (30.0_f64.to_radians().tan());
        let expected = f * 1.75 / 6.0;
        assert!(
            (f64::from(heights[0]) - expected).abs() < 0.2 * expected,
            "{} vs {expected}",
            heights[0]
        );
    }
}
