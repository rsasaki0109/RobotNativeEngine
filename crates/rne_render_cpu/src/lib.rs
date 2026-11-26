//! Deterministic, multi-threaded CPU ray tracer for [`rne_render`] scenes.
//!
//! [`CpuRayTracer`] renders a [`RenderScene`] without a GPU: boxes, spheres,
//! cylinders and triangle meshes (static or skinned on the CPU) with base-color
//! textures, lit by a sun with hard shadows and a sky/ground hemisphere. Each
//! render returns color, linear view depth and a per-pixel item id, so a camera
//! sensor gets exact 2D instance masks and boxes for free.
//!
//! It implements [`RenderBackend`], so the camera sensor model in `rne_sensor`
//! (distortion, rolling shutter, noise) runs on top of it unchanged.
//!
//! Rows are cut into blocks that worker threads take in turn. Every pixel is a
//! pure function of the scene and camera, so the output is byte-identical
//! whatever the number of threads.
//!
//! ```no_run
//! use rne_math::Transform3;
//! use rne_render::{Camera, RenderScene};
//! use rne_render_cpu::{CpuRayTracer, CpuRayTracerConfig};
//!
//! let tracer = CpuRayTracer::new(CpuRayTracerConfig::default());
//! let camera = Camera::new(640, 480, 1.0);
//! let output = tracer.render(&camera, &Transform3::IDENTITY, &RenderScene::new(), [0.5, 0.6, 0.8, 1.0]);
//! assert_eq!(output.item_ids.len(), 640 * 480);
//! ```

mod bvh;

use std::collections::HashMap;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use glam::Vec3;
use rne_math::Transform3;
use rne_render::{
    Camera, CameraPassOutput, DepthFrame, ImageFrame, RenderBackend, RenderError, RenderScene,
    RenderSceneItem, RenderTarget, VisualShape,
};

use bvh::{Aabb, Bvh, Triangle};

/// Item id of a pixel that sees no scene item.
pub const BACKGROUND_ITEM: u32 = u32::MAX;

/// Lighting and sampling settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CpuRayTracerConfig {
    /// Samples per pixel along each axis (`2` casts 4 color rays per pixel).
    pub samples_per_axis: u32,
    /// Direction toward the sun in world coordinates (need not be unit length).
    pub sun_direction: [f32; 3],
    /// Linear sun irradiance color.
    pub sun_rgb: [f32; 3],
    /// Linear ambient light from above (sky).
    pub sky_rgb: [f32; 3],
    /// Linear ambient light from below (ground bounce).
    pub ground_rgb: [f32; 3],
    /// Cast shadow rays toward the sun.
    pub shadows: bool,
    /// World up axis; the engine's world is y-up.
    pub up: [f32; 3],
    /// Worker threads; `0` uses the available parallelism.
    pub threads: usize,
}

impl Default for CpuRayTracerConfig {
    fn default() -> Self {
        Self {
            samples_per_axis: 2,
            sun_direction: [0.4, 1.0, 0.3],
            sun_rgb: [1.0, 0.96, 0.9],
            sky_rgb: [0.35, 0.4, 0.5],
            ground_rgb: [0.18, 0.16, 0.14],
            shadows: true,
            up: [0.0, 1.0, 0.0],
            threads: 0,
        }
    }
}

/// One rendered view.
#[derive(Clone, Debug, PartialEq)]
pub struct CpuRenderOutput {
    /// sRGB color.
    pub color: ImageFrame,
    /// Linear view-space depth in meters (`far_m` where nothing is hit).
    pub depth: DepthFrame,
    /// Index into [`RenderScene::items`] seen at each pixel centre, row-major;
    /// [`BACKGROUND_ITEM`] where nothing is hit.
    pub item_ids: Vec<u32>,
}

impl CpuRenderOutput {
    /// Pixel bounding box `[x_min, y_min, x_max, y_max]` (exclusive max) of each
    /// listed item, from the item-id map; `None` for items not visible.
    pub fn item_boxes(&self, items: &[u32]) -> Vec<Option<[u32; 4]>> {
        let width = self.color.width as usize;
        let mut boxes: Vec<Option<[u32; 4]>> = vec![None; items.len()];
        for (pixel, id) in self.item_ids.iter().enumerate() {
            let Some(slot) = items.iter().position(|item| item == id) else {
                continue;
            };
            let (x, y) = ((pixel % width) as u32, (pixel / width) as u32);
            let entry = boxes[slot].get_or_insert([x, y, x + 1, y + 1]);
            entry[0] = entry[0].min(x);
            entry[1] = entry[1].min(y);
            entry[2] = entry[2].max(x + 1);
            entry[3] = entry[3].max(y + 1);
        }
        boxes
    }
}

/// CPU ray tracer; see the crate documentation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CpuRayTracer {
    /// Lighting and sampling settings.
    pub config: CpuRayTracerConfig,
}

impl CpuRayTracer {
    /// Creates a ray tracer.
    pub fn new(config: CpuRayTracerConfig) -> Self {
        Self { config }
    }

    /// Renders `scene` from a camera at `view` (looking down its local −Z,
    /// up +Y), with `clear_color` (linear RGBA) where nothing is hit.
    pub fn render(
        &self,
        camera: &Camera,
        view: &Transform3,
        scene: &RenderScene,
        clear_color: [f32; 4],
    ) -> CpuRenderOutput {
        let prepared = Prepared::new(scene);
        let width = camera.width as usize;
        let height = camera.height as usize;
        let mut rgba8 = vec![0_u8; width * height * 4];
        let mut depth = vec![camera.far_m as f32; width * height];
        let mut item_ids = vec![BACKGROUND_ITEM; width * height];
        if width == 0 || height == 0 {
            return CpuRenderOutput {
                color: ImageFrame::from_rgba8(camera.width, camera.height, rgba8),
                depth: DepthFrame::new(camera.width, camera.height, depth),
                item_ids,
            };
        }
        let frame = Frame::new(camera, view, &self.config, clear_color);

        let threads = if self.config.threads == 0 {
            std::thread::available_parallelism().map_or(1, std::num::NonZeroUsize::get)
        } else {
            self.config.threads
        };
        let rows_per_block = 4;
        let blocks: Vec<RowBlock<'_>> = rgba8
            .chunks_mut(width * 4 * rows_per_block)
            .zip(depth.chunks_mut(width * rows_per_block))
            .zip(item_ids.chunks_mut(width * rows_per_block))
            .enumerate()
            .map(|(index, ((c, d), i))| (index * rows_per_block, c, d, i))
            .collect();
        let count = blocks.len();
        let queue = Mutex::new(blocks.into_iter().map(Some).collect::<Vec<_>>());
        let next = AtomicUsize::new(0);
        let work = || loop {
            let index = next.fetch_add(1, Ordering::Relaxed);
            if index >= count {
                break;
            }
            let block = queue
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)[index]
                .take();
            if let Some((first_row, color, depth, ids)) = block {
                for (offset, ((color_row, depth_row), id_row)) in color
                    .chunks_mut(width * 4)
                    .zip(depth.chunks_mut(width))
                    .zip(ids.chunks_mut(width))
                    .enumerate()
                {
                    frame.render_row(&prepared, first_row + offset, color_row, depth_row, id_row);
                }
            }
        };
        if threads <= 1 {
            work();
        } else {
            std::thread::scope(|scope| {
                for _ in 0..threads.min(count) {
                    scope.spawn(work);
                }
            });
        }
        CpuRenderOutput {
            color: ImageFrame::from_rgba8(camera.width, camera.height, rgba8),
            depth: DepthFrame::new(camera.width, camera.height, depth),
            item_ids,
        }
    }
}

impl RenderBackend for CpuRayTracer {
    fn render_clear(
        &mut self,
        target: RenderTarget,
        clear_color: [f32; 4],
    ) -> Result<ImageFrame, RenderError> {
        let pixel = encode(Vec3::new(clear_color[0], clear_color[1], clear_color[2]));
        let rgba8 = (0..(target.width * target.height) as usize)
            .flat_map(|_| pixel)
            .collect();
        Ok(ImageFrame::from_rgba8(target.width, target.height, rgba8))
    }

    fn render_scene_camera(
        &mut self,
        camera: &Camera,
        view: &Transform3,
        scene: &RenderScene,
        clear_color: [f32; 4],
    ) -> Result<CameraPassOutput, RenderError> {
        let output = self.render(camera, view, scene, clear_color);
        Ok(CameraPassOutput {
            color: output.color,
            depth: output.depth,
        })
    }
}

/// Rows handed to one worker: first row index, color, depth and item ids.
type RowBlock<'a> = (usize, &'a mut [u8], &'a mut [f32], &'a mut [u32]);

/// Per-render camera and lighting constants.
struct Frame {
    origin: Vec3,
    right: Vec3,
    up: Vec3,
    forward: Vec3,
    tan_half_y: f32,
    aspect: f32,
    width: usize,
    height: usize,
    near: f32,
    far: f32,
    samples: u32,
    sun: Vec3,
    sun_rgb: Vec3,
    sky: Vec3,
    ground: Vec3,
    world_up: Vec3,
    shadows: bool,
    clear: Vec3,
}

impl Frame {
    fn new(
        camera: &Camera,
        view: &Transform3,
        config: &CpuRayTracerConfig,
        clear: [f32; 4],
    ) -> Self {
        let rotate = |v: rne_math::Vec3| {
            let w = view.rotation * v;
            Vec3::new(w.x as f32, w.y as f32, w.z as f32)
        };
        let t = view.translation;
        Self {
            origin: Vec3::new(t.x as f32, t.y as f32, t.z as f32),
            right: rotate(rne_math::Vec3::X),
            up: rotate(rne_math::Vec3::Y),
            forward: rotate(-rne_math::Vec3::Z),
            tan_half_y: (camera.fov_y_rad as f32 * 0.5).tan(),
            aspect: camera.width as f32 / camera.height.max(1) as f32,
            width: camera.width as usize,
            height: camera.height as usize,
            near: camera.near_m as f32,
            far: camera.far_m as f32,
            samples: config.samples_per_axis.max(1),
            sun: Vec3::from(config.sun_direction).normalize_or(Vec3::Y),
            sun_rgb: Vec3::from(config.sun_rgb),
            sky: Vec3::from(config.sky_rgb),
            ground: Vec3::from(config.ground_rgb),
            world_up: Vec3::from(config.up).normalize_or(Vec3::Y),
            shadows: config.shadows,
            clear: Vec3::new(clear[0], clear[1], clear[2]),
        }
    }

    /// Unnormalized ray direction through image point `(x, y)` in pixels; its
    /// component along `forward` is 1, so the hit distance is the view depth.
    fn direction(&self, x: f32, y: f32) -> Vec3 {
        let ndc_x = (x / self.width as f32) * 2.0 - 1.0;
        let ndc_y = 1.0 - (y / self.height as f32) * 2.0;
        self.forward
            + self.right * (ndc_x * self.tan_half_y * self.aspect)
            + self.up * (ndc_y * self.tan_half_y)
    }

    fn render_row(
        &self,
        scene: &Prepared,
        row: usize,
        color: &mut [u8],
        depth: &mut [f32],
        ids: &mut [u32],
    ) {
        let n = self.samples;
        let inv = 1.0 / n as f32;
        for x in 0..self.width {
            // Depth and id from the pixel centre, so masks are crisp.
            let center = self.direction(x as f32 + 0.5, row as f32 + 0.5);
            if let Some(hit) = scene.closest(self.origin, center, self.near, self.far) {
                depth[x] = hit.t;
                ids[x] = hit.item;
            }
            let mut sum = Vec3::ZERO;
            for sy in 0..n {
                for sx in 0..n {
                    let dir = self.direction(
                        x as f32 + (sx as f32 + 0.5) * inv,
                        row as f32 + (sy as f32 + 0.5) * inv,
                    );
                    sum += match scene.closest(self.origin, dir, self.near, self.far) {
                        Some(hit) => self.shade(scene, &hit, dir),
                        None => self.clear,
                    };
                }
            }
            color[x * 4..x * 4 + 4].copy_from_slice(&encode(sum * (inv * inv)));
        }
    }

    fn shade(&self, scene: &Prepared, hit: &Hit, dir: Vec3) -> Vec3 {
        let item = &scene.items[hit.item as usize];
        let mut albedo = item.tint;
        if let (Some(texture), Some(uv)) = (&item.texture, hit.uv) {
            albedo *= texture.sample(uv);
        }
        // Two-sided surfaces: face the normal toward the viewer.
        let normal = if hit.normal.dot(dir) > 0.0 {
            -hit.normal
        } else {
            hit.normal
        };
        let point = self.origin + dir * hit.t;
        let lambert = normal.dot(self.sun).max(0.0);
        let lit = if lambert > 0.0 && self.shadows {
            let start = point + normal * 1e-3 * (1.0 + hit.t);
            if scene.occluded(start, self.sun, 0.0, 1e4) {
                0.0
            } else {
                lambert
            }
        } else {
            lambert
        };
        let hemisphere = normal.dot(self.world_up) * 0.5 + 0.5;
        let ambient = self.ground.lerp(self.sky, hemisphere);
        albedo * (self.sun_rgb * lit + ambient) + item.emissive
    }
}

/// A surface hit.
struct Hit {
    t: f32,
    item: u32,
    normal: Vec3,
    uv: Option<[f32; 2]>,
}

/// An analytic primitive in unit local space (box `[-0.5, 0.5]^3`, sphere of
/// radius 0.5, cylinder of radius 0.5 along Z from −0.5 to 0.5).
#[derive(Clone, Copy, Debug)]
enum PrimitiveKind {
    Box,
    Sphere,
    Cylinder,
}

#[derive(Clone, Copy, Debug)]
struct Primitive {
    kind: PrimitiveKind,
    item: u32,
    translation: Vec3,
    inv_rotation: glam::Quat,
    rotation: glam::Quat,
    inv_scale: Vec3,
    bounds: Aabb,
}

impl Primitive {
    fn intersect(&self, origin: Vec3, dir: Vec3, t_min: f32, t_max: f32) -> Option<(f32, Vec3)> {
        let o = self.inv_scale * (self.inv_rotation * (origin - self.translation));
        let d = self.inv_scale * (self.inv_rotation * dir);
        let (t, local_normal) = match self.kind {
            PrimitiveKind::Box => intersect_box(o, d, t_min, t_max)?,
            PrimitiveKind::Sphere => intersect_sphere(o, d, t_min, t_max)?,
            PrimitiveKind::Cylinder => intersect_cylinder(o, d, t_min, t_max)?,
        };
        // Normals transform with the inverse transpose: R * (n / s).
        let normal = (self.rotation * (local_normal * self.inv_scale)).normalize_or(Vec3::Y);
        Some((t, normal))
    }
}

fn intersect_box(o: Vec3, d: Vec3, t_min: f32, t_max: f32) -> Option<(f32, Vec3)> {
    let half = Vec3::splat(0.5);
    // The whole line's interval, then the first crossing after `t_min`.
    let (enter, exit) =
        bvh::slab_interval(-half, half, o, d.recip(), f32::NEG_INFINITY, f32::INFINITY)?;
    let (t, from_inside) = if enter > t_min {
        (enter, false)
    } else {
        (exit, true)
    };
    if t <= t_min || t >= t_max {
        return None;
    }
    let p = o + d * t;
    let a = p.abs();
    let mut normal = if a.x >= a.y && a.x >= a.z {
        Vec3::new(p.x.signum(), 0.0, 0.0)
    } else if a.y >= a.z {
        Vec3::new(0.0, p.y.signum(), 0.0)
    } else {
        Vec3::new(0.0, 0.0, p.z.signum())
    };
    if from_inside {
        normal = -normal;
    }
    Some((t, normal))
}

fn intersect_sphere(o: Vec3, d: Vec3, t_min: f32, t_max: f32) -> Option<(f32, Vec3)> {
    let a = d.dot(d);
    let b = o.dot(d);
    let c = o.dot(o) - 0.25;
    let disc = b * b - a * c;
    if disc < 0.0 {
        return None;
    }
    let root = disc.sqrt();
    let t = [(-b - root) / a, (-b + root) / a]
        .into_iter()
        .find(|t| *t > t_min && *t < t_max)?;
    Some((t, (o + d * t) * 2.0))
}

fn intersect_cylinder(o: Vec3, d: Vec3, t_min: f32, t_max: f32) -> Option<(f32, Vec3)> {
    let mut best: Option<(f32, Vec3)> = None;
    let mut consider = |t: f32, n: Vec3| {
        if t > t_min && t < t_max && best.is_none_or(|(b, _)| t < b) {
            best = Some((t, n));
        }
    };
    let a = d.x * d.x + d.y * d.y;
    if a > 1e-12 {
        let b = o.x * d.x + o.y * d.y;
        let c = o.x * o.x + o.y * o.y - 0.25;
        let disc = b * b - a * c;
        if disc >= 0.0 {
            let root = disc.sqrt();
            for t in [(-b - root) / a, (-b + root) / a] {
                let p = o + d * t;
                if p.z.abs() <= 0.5 {
                    consider(t, Vec3::new(p.x, p.y, 0.0) * 2.0);
                }
            }
        }
    }
    if d.z.abs() > 1e-12 {
        for z in [-0.5_f32, 0.5] {
            let t = (z - o.z) / d.z;
            let p = o + d * t;
            if p.x * p.x + p.y * p.y <= 0.25 {
                consider(t, Vec3::new(0.0, 0.0, z.signum()));
            }
        }
    }
    best
}

/// A base-color texture decoded to linear RGB.
#[derive(Debug)]
struct Texture {
    width: usize,
    height: usize,
    texels: Vec<Vec3>,
}

impl Texture {
    fn new(image: &ImageFrame) -> Option<Self> {
        let (width, height) = (image.width as usize, image.height as usize);
        if width == 0 || height == 0 || image.rgba8.len() < width * height * 4 {
            return None;
        }
        let lut: Vec<f32> = (0..256).map(|v| srgb_to_linear(v as f32 / 255.0)).collect();
        let texels = image
            .rgba8
            .chunks_exact(4)
            .take(width * height)
            .map(|p| Vec3::new(lut[p[0] as usize], lut[p[1] as usize], lut[p[2] as usize]))
            .collect();
        Some(Self {
            width,
            height,
            texels,
        })
    }

    /// Bilinear sample with repeat wrapping.
    fn sample(&self, uv: [f32; 2]) -> Vec3 {
        let x = uv[0].rem_euclid(1.0) * self.width as f32 - 0.5;
        let y = uv[1].rem_euclid(1.0) * self.height as f32 - 0.5;
        let (x0, y0) = (x.floor(), y.floor());
        let (fx, fy) = (x - x0, y - y0);
        let wrap = |v: f32, n: usize| (v as isize).rem_euclid(n as isize) as usize;
        let at = |xi: f32, yi: f32| {
            self.texels[wrap(yi, self.height) * self.width + wrap(xi, self.width)]
        };
        let top = at(x0, y0).lerp(at(x0 + 1.0, y0), fx);
        let bottom = at(x0, y0 + 1.0).lerp(at(x0 + 1.0, y0 + 1.0), fx);
        top.lerp(bottom, fy)
    }
}

/// Shading data of one scene item.
#[derive(Debug)]
struct ItemShading {
    tint: Vec3,
    emissive: Vec3,
    texture: Option<Arc<Texture>>,
}

/// The scene in ray-tracing form.
struct Prepared {
    bvh: Bvh,
    primitives: Vec<Primitive>,
    items: Vec<ItemShading>,
}

fn to_f32(v: rne_math::Vec3) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}

fn to_quat(q: rne_math::DQuat) -> glam::Quat {
    glam::Quat::from_xyzw(q.x as f32, q.y as f32, q.z as f32, q.w as f32).normalize()
}

impl Prepared {
    fn new(scene: &RenderScene) -> Self {
        let mut triangles = Vec::new();
        let mut primitives = Vec::new();
        let mut items = Vec::with_capacity(scene.items.len());
        // Many items share one texture (every copy of a person): decode each once.
        let mut textures: HashMap<*const ImageFrame, Option<Arc<Texture>>> = HashMap::new();
        for (index, item) in scene.items.iter().enumerate() {
            let texture = item.base_color_texture.as_ref().and_then(|image| {
                textures
                    .entry(Arc::as_ptr(image))
                    .or_insert_with(|| Texture::new(image).map(Arc::new))
                    .clone()
            });
            items.push(shading(item, texture));
            let index = index as u32;
            let translation = to_f32(item.transform.translation);
            let rotation = to_quat(item.transform.rotation);
            let scale = to_f32(item.transform.scale);
            let kind = match item.shape {
                VisualShape::Box { .. } => Some(PrimitiveKind::Box),
                VisualShape::Sphere { .. } => Some(PrimitiveKind::Sphere),
                VisualShape::Cylinder { .. } => Some(PrimitiveKind::Cylinder),
                VisualShape::Mesh { .. } | VisualShape::DynamicMesh => None,
            };
            if let Some(kind) = kind {
                if scale.min_element().abs() < 1e-9 {
                    continue;
                }
                // Bounds of the unit cube's corners cover every unit primitive.
                let bounds = (0..8).fold(Aabb::EMPTY, |acc, corner| {
                    let local = Vec3::new(
                        if corner & 1 == 0 { -0.5 } else { 0.5 },
                        if corner & 2 == 0 { -0.5 } else { 0.5 },
                        if corner & 4 == 0 { -0.5 } else { 0.5 },
                    );
                    let p = translation + rotation * (scale * local);
                    acc.union(Aabb { min: p, max: p })
                });
                primitives.push(Primitive {
                    kind,
                    item: index,
                    translation,
                    inv_rotation: rotation.inverse(),
                    rotation,
                    inv_scale: scale.recip(),
                    bounds,
                });
                continue;
            }
            let Some(mesh) = &item.mesh else {
                continue;
            };
            let place = |p: [f32; 3]| translation + rotation * (scale * Vec3::from(p));
            let place_normal =
                |n: [f32; 3]| (rotation * (Vec3::from(n) / scale)).normalize_or(Vec3::Y);
            for tri in mesh.indices.chunks_exact(3) {
                let [a, b, c] = [tri[0] as usize, tri[1] as usize, tri[2] as usize];
                let (Some(pa), Some(pb), Some(pc)) = (
                    mesh.positions.get(a),
                    mesh.positions.get(b),
                    mesh.positions.get(c),
                ) else {
                    continue;
                };
                let v0 = place(*pa);
                let (v1, v2) = (place(*pb), place(*pc));
                let face = (v1 - v0).cross(v2 - v0).normalize_or(Vec3::Y);
                let normal = |i: usize| mesh.normals.get(i).map_or(face, |n| place_normal(*n));
                let uv = |i: usize| mesh.texcoords.get(i).copied().unwrap_or([0.0, 0.0]);
                triangles.push(Triangle {
                    v0,
                    e1: v1 - v0,
                    e2: v2 - v0,
                    n: [normal(a), normal(b), normal(c)],
                    uv: [uv(a), uv(b), uv(c)],
                    item: index,
                });
            }
        }
        Self {
            bvh: Bvh::build(triangles),
            primitives,
            items,
        }
    }

    fn closest(&self, origin: Vec3, dir: Vec3, t_min: f32, t_max: f32) -> Option<Hit> {
        let mut best: Option<Hit> = None;
        let mut limit = t_max;
        if let Some((t, index, u, v)) = self.bvh.closest(origin, dir, t_min, limit) {
            let tri = &self.bvh.triangles[index];
            let w = 1.0 - u - v;
            let normal = (tri.n[0] * w + tri.n[1] * u + tri.n[2] * v)
                .normalize_or(tri.e1.cross(tri.e2).normalize_or(Vec3::Y));
            let uv = [
                tri.uv[0][0] * w + tri.uv[1][0] * u + tri.uv[2][0] * v,
                tri.uv[0][1] * w + tri.uv[1][1] * u + tri.uv[2][1] * v,
            ];
            limit = t;
            best = Some(Hit {
                t,
                item: tri.item,
                normal,
                uv: Some(uv),
            });
        }
        let inv_dir = dir.recip();
        for primitive in &self.primitives {
            if primitive
                .bounds
                .hit(origin, inv_dir, t_min, limit)
                .is_none()
            {
                continue;
            }
            if let Some((t, normal)) = primitive.intersect(origin, dir, t_min, limit) {
                limit = t;
                best = Some(Hit {
                    t,
                    item: primitive.item,
                    normal,
                    uv: None,
                });
            }
        }
        best
    }

    fn occluded(&self, origin: Vec3, dir: Vec3, t_min: f32, t_max: f32) -> bool {
        let inv_dir = dir.recip();
        self.primitives.iter().any(|p| {
            p.bounds.hit(origin, inv_dir, t_min, t_max).is_some()
                && p.intersect(origin, dir, t_min, t_max).is_some()
        }) || self.bvh.occluded(origin, dir, t_min, t_max)
    }
}

fn shading(item: &RenderSceneItem, texture: Option<Arc<Texture>>) -> ItemShading {
    let c = item.color_rgba;
    let b = item.material.base_color_rgba;
    let e = item.material.emissive_rgb;
    ItemShading {
        tint: Vec3::new(c[0] * b[0], c[1] * b[1], c[2] * b[2]),
        emissive: Vec3::from(e),
        texture,
    }
}

fn srgb_to_linear(v: f32) -> f32 {
    if v <= 0.04045 {
        v / 12.92
    } else {
        ((v + 0.055) / 1.055).powf(2.4)
    }
}

fn linear_to_srgb(v: f32) -> f32 {
    let v = v.clamp(0.0, 1.0);
    if v <= 0.003_130_8 {
        v * 12.92
    } else {
        1.055 * v.powf(1.0 / 2.4) - 0.055
    }
}

fn encode(linear: Vec3) -> [u8; 4] {
    let q = |v: f32| (linear_to_srgb(v) * 255.0 + 0.5) as u8;
    [q(linear.x), q(linear.y), q(linear.z), 255]
}

#[cfg(test)]
mod tests;
