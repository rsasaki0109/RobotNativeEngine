use super::*;
use approx::assert_relative_eq;
use rne_math::{DQuat, Vec3 as DVec3};
use rne_render::{PbrMaterial, TriangleMesh};
use std::sync::Arc;

fn item(shape: VisualShape, translation: DVec3, color: [f32; 4]) -> RenderSceneItem {
    RenderScene::item_from_visual(
        rne_world::Transform3::from_translation_rotation(translation, DQuat::IDENTITY),
        shape,
        color,
        rne_world::Transform3::IDENTITY,
    )
}

/// A camera at the origin looking down −Z.
fn camera(width: u32, height: u32) -> Camera {
    Camera {
        width,
        height,
        fov_y_rad: 1.0,
        near_m: 0.05,
        far_m: 100.0,
    }
}

fn tracer(threads: usize) -> CpuRayTracer {
    CpuRayTracer::new(CpuRayTracerConfig {
        threads,
        ..CpuRayTracerConfig::default()
    })
}

fn center(output: &CpuRenderOutput) -> usize {
    let w = output.color.width as usize;
    let h = output.color.height as usize;
    (h / 2) * w + w / 2
}

#[test]
fn primitives_are_hit_at_their_true_depth() {
    for (shape, near_face_m) in [
        (
            VisualShape::Box {
                size_m: DVec3::new(1.0, 1.0, 2.0),
            },
            1.0,
        ),
        (VisualShape::Sphere { radius_m: 0.75 }, 0.75),
        // Along local Z, i.e. its cap faces the camera.
        (
            VisualShape::Cylinder {
                radius_m: 0.3,
                length_m: 1.2,
            },
            0.6,
        ),
    ] {
        let mut scene = RenderScene::new();
        scene
            .items
            .push(item(shape.clone(), DVec3::new(0.0, 0.0, -5.0), [1.0; 4]));
        let output = tracer(1).render(&camera(33, 33), &Transform3::IDENTITY, &scene, [0.0; 4]);
        let c = center(&output);
        assert_eq!(output.item_ids[c], 0, "{shape:?}");
        assert_relative_eq!(output.depth.depth_m[c], 5.0 - near_face_m, epsilon = 1e-4);
        // Corners see the background at the far plane.
        assert_eq!(output.item_ids[0], BACKGROUND_ITEM);
        assert_relative_eq!(output.depth.depth_m[0], 100.0);
    }
}

#[test]
fn depth_is_view_depth_not_ray_length() {
    // A wall facing the camera: every pixel has the same view depth.
    let mut scene = RenderScene::new();
    scene.items.push(item(
        VisualShape::Box {
            size_m: DVec3::new(50.0, 50.0, 0.1),
        },
        DVec3::new(0.0, 0.0, -4.05),
        [1.0; 4],
    ));
    let output = tracer(1).render(&camera(16, 12), &Transform3::IDENTITY, &scene, [0.0; 4]);
    for depth in &output.depth.depth_m {
        assert_relative_eq!(*depth, 4.0, epsilon = 1e-4);
    }
}

fn busy_scene() -> RenderScene {
    let mut scene = RenderScene::new();
    scene.items.push(item(
        VisualShape::Box {
            size_m: DVec3::new(20.0, 0.1, 20.0),
        },
        DVec3::new(0.0, -1.05, -6.0),
        [0.6, 0.6, 0.6, 1.0],
    ));
    for i in 0..6 {
        let x = -2.5 + f64::from(i);
        scene.items.push(item(
            VisualShape::Sphere { radius_m: 0.35 },
            DVec3::new(x, -0.3, -5.0 - 0.4 * f64::from(i)),
            [0.8, 0.2 + 0.1 * i as f32, 0.3, 1.0],
        ));
    }
    scene
}

#[test]
fn output_does_not_depend_on_the_number_of_threads() {
    let scene = busy_scene();
    let view = Transform3 {
        translation: DVec3::new(0.2, 0.5, 0.0),
        rotation: DQuat::from_rotation_x(-0.15),
        scale: DVec3::ONE,
    };
    let serial = tracer(1).render(&camera(97, 61), &view, &scene, [0.3, 0.4, 0.6, 1.0]);
    for threads in [2, 3, 8] {
        assert_eq!(
            tracer(threads).render(&camera(97, 61), &view, &scene, [0.3, 0.4, 0.6, 1.0]),
            serial
        );
    }
    assert!(serial.item_ids.contains(&3));
}

#[test]
fn sun_casts_a_shadow() {
    let mut scene = RenderScene::new();
    // Ground below the camera and a box hovering over part of it.
    scene.items.push(item(
        VisualShape::Box {
            size_m: DVec3::new(20.0, 0.1, 20.0),
        },
        DVec3::new(0.0, -2.0, -5.0),
        [1.0; 4],
    ));
    scene.items.push(item(
        VisualShape::Box {
            size_m: DVec3::new(2.0, 0.2, 2.0),
        },
        DVec3::new(0.0, 2.0, -5.0),
        [1.0; 4],
    ));
    let mut config = CpuRayTracerConfig {
        sun_direction: [0.0, 1.0, 0.0],
        samples_per_axis: 1,
        threads: 1,
        ..CpuRayTracerConfig::default()
    };
    // Look straight down from above the box's side.
    let view = Transform3 {
        translation: DVec3::new(0.0, 0.0, -5.0),
        rotation: DQuat::from_rotation_x(-std::f64::consts::FRAC_PI_2),
        scale: DVec3::ONE,
    };
    let lit = |config: CpuRayTracerConfig| {
        let output = CpuRayTracer::new(config).render(&camera(41, 41), &view, &scene, [0.0; 4]);
        let c = center(&output);
        assert_eq!(
            output.item_ids[c], 0,
            "the camera sees the ground under the box"
        );
        output.color.rgba8[c * 4]
    };
    let shadowed = lit(config);
    config.shadows = false;
    let unshadowed = lit(config);
    assert!(shadowed + 60 < unshadowed, "{shadowed} vs {unshadowed}");
}

#[test]
fn textured_mesh_shows_its_texture() {
    // A quad facing the camera, left half red and right half blue.
    let mesh = TriangleMesh {
        positions: vec![
            [-1.0, -1.0, 0.0],
            [1.0, -1.0, 0.0],
            [1.0, 1.0, 0.0],
            [-1.0, 1.0, 0.0],
        ],
        normals: vec![[0.0, 0.0, 1.0]; 4],
        texcoords: vec![[0.0, 1.0], [1.0, 1.0], [1.0, 0.0], [0.0, 0.0]],
        indices: vec![0, 1, 2, 0, 2, 3],
        skinning: None,
    };
    let red = [255, 0, 0, 255];
    let blue = [0, 0, 255, 255];
    let texture = ImageFrame::from_rgba8(4, 1, [red, red, blue, blue].concat());
    let mut quad = RenderScene::item_from_dynamic_mesh(mesh, [1.0; 4]);
    quad.transform.translation = DVec3::new(0.0, 0.0, -3.0);
    quad.base_color_texture = Some(Arc::new(texture));
    quad.material = PbrMaterial {
        emissive_rgb: [0.0; 3],
        ..PbrMaterial::default()
    };
    let mut scene = RenderScene::new();
    scene.items.push(quad);
    let config = CpuRayTracerConfig {
        sun_direction: [0.0, 0.0, 1.0],
        threads: 1,
        ..CpuRayTracerConfig::default()
    };
    let output =
        CpuRayTracer::new(config).render(&camera(40, 40), &Transform3::IDENTITY, &scene, [0.0; 4]);
    let pixel = |x: usize, y: usize| &output.color.rgba8[(y * 40 + x) * 4..(y * 40 + x) * 4 + 3];
    let left = pixel(14, 20);
    let right = pixel(26, 20);
    assert!(left[0] > 150 && left[2] < 40, "left {left:?}");
    assert!(right[2] > 150 && right[0] < 40, "right {right:?}");
    let boxes = output.item_boxes(&[0, 7]);
    let [x0, y0, x1, y1] = boxes[0].expect("quad visible");
    assert!(x0 > 0 && y0 > 0 && x1 < 40 && y1 < 40 && x1 - x0 == y1 - y0);
    assert_eq!(boxes[1], None);
}

#[test]
fn render_backend_matches_render() {
    let scene = busy_scene();
    let mut backend = tracer(2);
    let pass = backend
        .render_scene_camera(
            &camera(32, 24),
            &Transform3::IDENTITY,
            &scene,
            [0.1, 0.2, 0.3, 1.0],
        )
        .unwrap();
    let direct = tracer(1).render(
        &camera(32, 24),
        &Transform3::IDENTITY,
        &scene,
        [0.1, 0.2, 0.3, 1.0],
    );
    assert_eq!(pass.color, direct.color);
    assert_eq!(pass.depth, direct.depth);
}
