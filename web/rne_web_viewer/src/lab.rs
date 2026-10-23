//! Static, metre-scaled robot laboratory scenery and procedural studio lighting.
use rne_math::{Quat, Vec3};
use rne_render::VisualShape;
#[cfg(any(target_arch = "wasm32", test))]
use rne_render::{EnvironmentLighting, EnvironmentMap};
use rne_render::{PbrMaterial, RenderScene};
use rne_world::Transform3;
use std::sync::Arc;

const WALL: [f32; 4] = [0.57, 0.59, 0.58, 1.0];
const STEEL: [f32; 4] = [0.12, 0.16, 0.18, 1.0];
const PAINT: [f32; 4] = [0.82, 0.57, 0.12, 1.0];

fn block(
    scene: &mut RenderScene,
    p: [f64; 3],
    size: [f64; 3],
    color: [f32; 4],
    roughness: f32,
    metallic: f32,
) {
    scene
        .items
        .push(RenderScene::item_from_visual_with_material(
            Transform3::from_translation_rotation(Vec3::from_array(p), Quat::IDENTITY),
            VisualShape::Box {
                size_m: Vec3::from_array(size),
            },
            color,
            Transform3::IDENTITY,
            PbrMaterial::new([1.0; 4], roughness, metallic, [0.0; 3]),
        ));
}

pub(crate) fn scene() -> RenderScene {
    let mut scene = RenderScene::new();
    let concrete = crate::surfaces::concrete();
    block(
        &mut scene,
        [0.0, -0.07, 0.0],
        [12.2, 0.12, 12.2],
        [0.18, 0.19, 0.18, 1.0],
        0.9,
        0.0,
    );
    for x in -3..3 {
        for z in -3..3 {
            let tint = 0.92 + f64::from((x * x + z * z) % 3) * 0.025;
            block(
                &mut scene,
                [f64::from(x) * 2.0 + 1.0, -0.006, f64::from(z) * 2.0 + 1.0],
                [1.994, 0.012, 1.994],
                [tint as f32, tint as f32, tint as f32, 1.0],
                0.86,
                0.0,
            );
            scene.items.last_mut().unwrap().base_color_texture = Some(Arc::clone(&concrete.color));
            scene.items.last_mut().unwrap().material = concrete.material.clone();
        }
    }
    // A roofless cutaway keeps the orbit camera and the operating area visible.
    block(
        &mut scene,
        [0.0, 1.8, -6.0],
        [12.2, 3.6, 0.16],
        WALL,
        0.88,
        0.0,
    );
    block(
        &mut scene,
        [-6.0, 1.8, 0.0],
        [0.16, 3.6, 12.0],
        WALL,
        0.88,
        0.0,
    );
    block(
        &mut scene,
        [0.0, 0.12, -5.89],
        [12.0, 0.24, 0.06],
        STEEL,
        0.65,
        0.35,
    );
    block(
        &mut scene,
        [-5.89, 0.12, 0.0],
        [0.06, 0.24, 12.0],
        STEEL,
        0.65,
        0.35,
    );
    // Recessed window bays, frames, and bright exterior panels.
    for x in [-3.6, 0.0, 3.6] {
        block(
            &mut scene,
            [x, 2.25, -5.86],
            [2.8, 1.65, 0.08],
            STEEL,
            0.35,
            0.65,
        );
        block(
            &mut scene,
            [x, 2.25, -5.8],
            [2.65, 1.48, 0.035],
            [0.55, 0.72, 0.8, 1.0],
            0.2,
            0.15,
        );
        scene.items.last_mut().unwrap().material.emissive_rgb = [0.25, 0.35, 0.43];
        block(
            &mut scene,
            [x, 2.25, -5.76],
            [0.045, 1.5, 0.05],
            STEEL,
            0.35,
            0.65,
        );
        block(
            &mut scene,
            [x, 1.47, -5.72],
            [2.95, 0.07, 0.32],
            WALL,
            0.65,
            0.0,
        );
    }
    // Flush painted operating-area boundary, outside the four checkpoint route.
    for edge in [-4.25, 4.25] {
        block(
            &mut scene,
            [edge, 0.003, 0.0],
            [0.045, 0.004, 8.5],
            PAINT,
            0.8,
            0.0,
        );
        block(
            &mut scene,
            [0.0, 0.003, edge],
            [8.5, 0.004, 0.045],
            PAINT,
            0.8,
            0.0,
        );
    }
    // Workbenches stay outside the 8x8 metre driving area.
    for x in [-3.5, 0.0] {
        block(
            &mut scene,
            [x, 0.88, -5.15],
            [2.4, 0.09, 0.9],
            [0.36, 0.27, 0.17, 1.0],
            0.6,
            0.0,
        );
        for dx in [-1.05, 1.05] {
            for z in [-5.48, -4.83] {
                block(
                    &mut scene,
                    [x + dx, 0.43, z],
                    [0.055, 0.86, 0.055],
                    STEEL,
                    0.4,
                    0.7,
                );
            }
        }
        block(
            &mut scene,
            [x, 1.24, -5.3],
            [0.8, 0.47, 0.055],
            STEEL,
            0.4,
            0.2,
        );
        block(
            &mut scene,
            [x, 1.24, -5.263],
            [0.71, 0.38, 0.012],
            [0.025, 0.11, 0.14, 1.0],
            0.4,
            0.0,
        );
        scene.items.last_mut().unwrap().material.emissive_rgb = [0.015, 0.08, 0.1];
        block(
            &mut scene,
            [x, 1.0, -5.32],
            [0.06, 0.18, 0.06],
            STEEL,
            0.4,
            0.7,
        );
        block(
            &mut scene,
            [x, 0.94, -4.95],
            [0.65, 0.025, 0.22],
            STEEL,
            0.8,
            0.0,
        );
    }
    // Equipment cabinet with shelf divisions and small status lamps.
    block(
        &mut scene,
        [3.7, 1.0, -5.23],
        [1.2, 2.0, 0.85],
        STEEL,
        0.45,
        0.55,
    );
    for shelf in 0..5 {
        let y = 0.22 + f64::from(shelf) * 0.36;
        block(
            &mut scene,
            [3.7, y, -4.78],
            [1.04, 0.28, 0.035],
            [0.21, 0.24, 0.25, 1.0],
            0.55,
            0.4,
        );
        block(
            &mut scene,
            [4.1, y, -4.75],
            [0.025, 0.025, 0.012],
            [0.1, 0.7, 0.35, 1.0],
            0.4,
            0.0,
        );
        scene.items.last_mut().unwrap().material.emissive_rgb = [0.05, 0.5, 0.12];
    }
    // Wall luminaires; emissive surfaces complement the renderer's shadow light.
    for z in [-3.5, 0.0, 3.5] {
        block(
            &mut scene,
            [-5.8, 3.25, z],
            [0.3, 0.12, 1.2],
            STEEL,
            0.4,
            0.7,
        );
        block(
            &mut scene,
            [-5.63, 3.24, z],
            [0.025, 0.06, 1.1],
            [0.9, 0.88, 0.76, 1.0],
            0.4,
            0.0,
        );
        scene.items.last_mut().unwrap().material.emissive_rgb = [1.4, 1.3, 1.0];
    }
    scene
}

#[cfg(any(target_arch = "wasm32", test))]
pub(crate) fn lighting() -> EnvironmentLighting {
    // Small procedural HDR map: broad cool daylight and warm overhead fill.
    let (width, height) = (64, 32);
    let mut pixels = Vec::with_capacity(width * height * 4);
    for y in 0..height {
        for x in 0..width {
            let top = y < height / 2;
            let rgb = if top && (8..20).contains(&x) && (7..14).contains(&y) {
                [2.8, 3.0, 3.3]
            } else if top {
                [0.65, 0.63, 0.57]
            } else {
                [0.18, 0.19, 0.2]
            };
            pixels.extend_from_slice(&[rgb[0], rgb[1], rgb[2], 1.0]);
        }
    }
    EnvironmentLighting {
        map: Some(Arc::new(
            EnvironmentMap::from_rgba32f(width as u32, height as u32, pixels).unwrap(),
        )),
        diffuse_strength: 0.8,
        specular_strength: 0.55,
        ..EnvironmentLighting::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn scenery_is_repeatable_and_keeps_driving_area_clear() {
        let a = scene();
        assert_eq!(a, scene());
        assert!(a.items.len() < 160, "keep scenery affordable in WebGPU");
        for item in &a.items {
            let p = item.transform.translation;
            let size = item.transform.scale;
            if p.y + size.y * 0.5 > 0.08 {
                assert!(
                    p.x.abs() - size.x * 0.5 > 4.0 || p.z.abs() - size.z * 0.5 > 4.0,
                    "scenery must not occupy the kinematic operating area"
                );
            }
        }
        let light = lighting();
        assert_eq!(light, light.sanitized());
    }
}
