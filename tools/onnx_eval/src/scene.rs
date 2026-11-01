//! A warehouse corner rendered by RNE, with exact per-pixel depth.

use std::path::PathBuf;
use std::sync::Arc;

use rne_math::{Quat, Transform3, Vec3};
use rne_render::{
    EnvironmentLighting, EnvironmentMap, PbrMaterial, RenderScene, RenderSceneItem, VisualShape,
};
use rne_render_wgpu::CameraOrbit;

/// CC0 Poly Haven props vendored with RNE.
pub fn props_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/props/polyhaven_warehouse")
}

/// Soft warehouse lighting: bright ceiling bands over dim walls and floor.
pub fn lighting() -> EnvironmentLighting {
    const W: u32 = 128;
    const H: u32 = 64;
    let mut rgba32f = Vec::with_capacity((W * H * 4) as usize);
    for row in 0..H {
        let down = (row as f32 + 0.5) / H as f32;
        for column in 0..W {
            let around = (column as f32 + 0.5) / W as f32;
            let (r, g, b) = if down < 0.30 {
                let bands = (around * std::f32::consts::TAU * 4.0).sin().max(0.0);
                let base = 0.75 + 0.85 * (1.0 - down / 0.30);
                let fixture = 4.5 * bands.powf(18.0);
                (base + fixture, base + fixture, base * 1.03 + fixture)
            } else if down < 0.62 {
                let level = 0.52 - 0.26 * (down - 0.30) / 0.32;
                (level * 0.94, level * 0.97, level)
            } else {
                (0.15, 0.145, 0.135)
            };
            rgba32f.extend_from_slice(&[r, g, b, 1.0]);
        }
    }
    EnvironmentLighting {
        map: Some(Arc::new(
            EnvironmentMap::from_rgba32f(W, H, rgba32f).expect("environment map"),
        )),
        intensity: 1.0,
        diffuse_strength: 0.72,
        specular_strength: 0.38,
        rotation_rad: 0.0,
    }
}

fn block(center: Vec3, size: Vec3, color: [f32; 4], roughness: f32) -> RenderSceneItem {
    RenderSceneItem {
        // Primitive boxes are sized by the transform scale.
        transform: Transform3 {
            translation: center,
            rotation: Quat::IDENTITY,
            scale: size,
        },
        shape: VisualShape::Box { size_m: size },
        color_rgba: color,
        mesh: None,
        base_color_texture: None,
        material: PbrMaterial::new(color, roughness, 0.0, [0.0; 3]),
    }
}

fn prop(path: &str, x: f64, z: f64, yaw: f64) -> RenderSceneItem {
    RenderScene::item_from_visual(
        rne_world::Transform3::from_translation_rotation(
            Vec3::new(x, 0.0, z),
            Quat::from_rotation_y(yaw),
        ),
        VisualShape::Mesh {
            path: path.into(),
            scale: Vec3::ONE,
        },
        [1.0; 4],
        rne_world::Transform3::IDENTITY,
    )
}

/// Floor, two walls, shelving and scanned props spread from 1 m to 6 m.
pub fn warehouse() -> RenderScene {
    let concrete = [0.42, 0.41, 0.39, 1.0];
    let wall = [0.62, 0.64, 0.66, 1.0];
    let steel = [0.20, 0.33, 0.55, 1.0];
    let mut items = vec![
        block(
            Vec3::new(0.0, -0.05, -2.0),
            Vec3::new(14.0, 0.1, 14.0),
            concrete,
            0.9,
        ),
        block(
            Vec3::new(0.0, 2.0, -5.5),
            Vec3::new(14.0, 4.0, 0.2),
            wall,
            0.8,
        ),
        block(
            Vec3::new(-5.0, 2.0, -2.0),
            Vec3::new(0.2, 4.0, 14.0),
            wall,
            0.8,
        ),
    ];
    // A two-bay shelf against the side wall.
    for (z, height) in [(-1.0, 1.6), (-2.6, 1.2)] {
        for (dx, dz) in [(0.0, -0.6), (0.0, 0.6), (0.9, -0.6), (0.9, 0.6)] {
            items.push(block(
                Vec3::new(-4.3 + dx, height / 2.0, z + dz),
                Vec3::new(0.06, height, 0.06),
                steel,
                0.5,
            ));
        }
        for level in [0.4, height - 0.05] {
            items.push(block(
                Vec3::new(-3.85, level, z),
                Vec3::new(1.0, 0.04, 1.3),
                [0.55, 0.42, 0.28, 1.0],
                0.7,
            ));
        }
    }
    items.extend([
        prop("cardboard_box_01/cardboard_box_01_1k.gltf", 0.9, -1.4, 0.3),
        prop("cardboard_box_01/cardboard_box_01_1k.gltf", 1.5, -1.7, -0.4),
        prop("cardboard_box_01/cardboard_box_01_1k.gltf", -3.9, -2.6, 0.1),
        prop("hand_truck/hand_truck_1k.gltf", -1.0, -2.0, 0.6),
        prop(
            "industrial_storage_cart/industrial_storage_cart_1k.gltf",
            2.0,
            -3.4,
            -0.3,
        ),
        prop(
            "korean_fire_extinguisher_01/korean_fire_extinguisher_01_1k.gltf",
            -2.4,
            -4.2,
            0.0,
        ),
        prop("WetFloorSign_01/WetFloorSign_01_1k.gltf", 0.1, -0.7, 0.5),
        prop("power_box_01/power_box_01_1k.gltf", 3.2, -4.6, 0.0),
        prop("wooden_crate_01/wooden_crate_01_1k.gltf", -1.8, -3.1, 0.2),
        prop(
            "steel_frame_shelves_02/steel_frame_shelves_02_1k.gltf",
            3.7,
            -1.8,
            -1.57,
        ),
        prop(
            "rollershutter_door/rollershutter_door_1k.gltf",
            0.6,
            -5.38,
            0.0,
        ),
    ]);
    RenderScene { items }
}

/// Eight viewpoints at standing camera height around the scene.
pub fn views() -> Vec<CameraOrbit> {
    (0..8)
        .map(|index| {
            let t = f64::from(index) / 7.0;
            CameraOrbit {
                focus: Vec3::new(-0.6 + 1.2 * t, 0.7, -2.4),
                yaw_rad: -0.75 + 1.5 * t,
                pitch_rad: 1.30 - 0.12 * (index % 2) as f64,
                distance_m: 3.6 + 0.8 * ((index * 3) % 4) as f64 / 3.0,
            }
        })
        .collect()
}
