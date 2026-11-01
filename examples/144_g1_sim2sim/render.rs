//! Renders the RNE walk with RNE's and MuJoCo's pelvis paths on the floor.
//!
//! Rendering only reads the simulated world after each policy step; it never
//! feeds back into physics, so a rendered run is the same run as a headless one.

use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use rne_ai::build_visual_render_scene;
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{
    Camera, EnvironmentLighting, EnvironmentMap, MeshRenderCache, PbrMaterial, RenderBackend,
    RenderScene, RenderSceneItem, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};

use crate::policy::Result;
use crate::walk::G1Walker;

/// Rendered panel size in pixels.
pub(crate) const WIDTH: u32 = 960;
pub(crate) const HEIGHT: u32 = 460;
const RNE_PATH_RGBA: [f32; 4] = [0.20, 0.55, 1.0, 1.0];
const MUJOCO_PATH_RGBA: [f32; 4] = [1.0, 0.55, 0.15, 1.0];

/// Offscreen renderer for one walk.
pub(crate) struct WalkRenderer {
    backend: WgpuRenderBackend,
    cache: MeshRenderCache,
    camera: Camera,
    orbit: CameraOrbit,
    roots: Vec<PathBuf>,
    output: PathBuf,
    frames: usize,
}

fn flat_box(center: Vec3, size: Vec3, color_rgba: [f32; 4], roughness: f32) -> RenderSceneItem {
    RenderSceneItem {
        // The renderer sizes primitive boxes by the transform scale.
        transform: Transform3 {
            translation: center,
            rotation: Quat::IDENTITY,
            scale: size,
        },
        shape: VisualShape::Box { size_m: size },
        color_rgba,
        mesh: None,
        base_color_texture: None,
        material: PbrMaterial::new(color_rgba, roughness, 0.0, [0.0; 3]),
    }
}

/// Converts a policy-frame (Z-up) floor point to RNE's Y-up world.
fn floor_point(xy: [f64; 2], height_m: f64) -> Vec3 {
    Vec3::new(xy[0], height_m, -xy[1])
}

fn checker_floor(scene: &mut RenderScene) {
    const TILE_M: f64 = 0.5;
    for row in -6..6 {
        for column in -6..6 {
            let shade = if (row + column) % 2 == 0 { 0.16 } else { 0.30 };
            let center = Vec3::new(
                (f64::from(column) + 0.5) * TILE_M,
                // Tops 2 mm above the scene's own ground visual, which
                // would otherwise z-fight with them at y = 0.
                -0.008,
                (f64::from(row) + 0.5) * TILE_M,
            );
            scene.items.push(flat_box(
                center,
                Vec3::new(TILE_M, 0.02, TILE_M),
                [shade, shade + 0.01, shade + 0.03, 1.0],
                0.9,
            ));
        }
    }
}

fn path(scene: &mut RenderScene, points: &[[f64; 2]], color: [f32; 4], lift_m: f64) {
    for point in points {
        scene.items.push(flat_box(
            floor_point(*point, lift_m),
            Vec3::new(0.035, 0.004, 0.035),
            color,
            0.5,
        ));
    }
}

fn write_png(path: &Path, bytes: &[u8]) -> Result<()> {
    let mut encoder = png::Encoder::new(fs::File::create(path)?, WIDTH, HEIGHT);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.write_header()?.write_image_data(bytes)?;
    Ok(())
}

impl WalkRenderer {
    pub(crate) fn new(walker: &G1Walker, output: &Path, orbit: CameraOrbit) -> Result<Self> {
        let output = output.join("frames");
        fs::create_dir_all(&output)?;
        let mut backend = WgpuRenderBackend::new()?;
        let map = EnvironmentMap::solid([0.22, 0.25, 0.30, 1.0])?;
        let mut lighting = EnvironmentLighting::from_map(Arc::new(map));
        lighting.diffuse_strength = 0.75;
        lighting.specular_strength = 0.25;
        backend.set_environment(lighting);
        Ok(Self {
            backend,
            cache: MeshRenderCache::new(),
            camera: Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4),
            orbit,
            roots: walker.sim.mesh_package_roots().to_vec(),
            output,
            frames: 0,
        })
    }

    /// Renders the current pose with both pelvis paths so far.
    pub(crate) fn frame(
        &mut self,
        walker: &G1Walker,
        rne_path: &[[f64; 2]],
        mujoco_path: &[[f64; 2]],
    ) -> Result<()> {
        let mut scene = build_visual_render_scene(walker.sim.world());
        let roots: Vec<_> = self.roots.iter().map(PathBuf::as_path).collect();
        self.cache.resolve_scene(&mut scene, &roots)?;
        checker_floor(&mut scene);
        path(&mut scene, mujoco_path, MUJOCO_PATH_RGBA, 0.004);
        path(&mut scene, rne_path, RNE_PATH_RGBA, 0.006);
        let rgba = self
            .backend
            .render_scene_camera(
                &self.camera,
                &self.orbit.camera_transform(),
                &scene,
                [0.075, 0.09, 0.115, 1.0],
            )?
            .color
            .rgba8;
        write_png(
            &self.output.join(format!("frame-{:03}.png", self.frames)),
            &rgba,
        )?;
        self.frames += 1;
        Ok(())
    }

    pub(crate) fn frame_count(&self) -> usize {
        self.frames
    }
}
