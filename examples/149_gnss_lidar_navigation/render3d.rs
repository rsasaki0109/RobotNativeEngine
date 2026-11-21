//! 3D render of example 149 with Robot Native Engine's wgpu renderer.
//!
//! Reads the trace written by `149_gnss_lidar_navigation --trace`, builds the
//! city (glass, concrete and metal towers with floor bands and mullions, the
//! street and plaza, the office with its roof cut away), and for every
//! recorded instant renders the robot with its two GNSS antennas and LiDAR,
//! the ray-traced satellite signals as beams (line of sight green, NLOS
//! reflections red through their wall points, diffraction purple over the
//! edge), the LiDAR returns, and trails coloured by localization source,
//! from a chase camera. PNG frames plus `frames.json` go to `--output`;
//! `encode.py` adds the HUD and encodes the GIF.
//!
//! ```bash
//! cargo run --release -p gnss_lidar_navigation --example 149_gnss_lidar_navigation -- --trace target/gnss3d/trace.json
//! cargo run --release -p gnss_lidar_navigation --features capture --example 149_gnss_lidar_navigation_render -- \
//!     --trace target/gnss3d/trace.json --output target/gnss3d/frames
//! python3 examples/149_gnss_lidar_navigation/encode.py target/gnss3d docs/media/gnss-lidar-navigation-3d.gif
//! ```
//!
//! Without a GPU, Mesa's lavapipe Vulkan driver works (`mesa-vulkan-drivers`).

use rne_math::{Quat, Transform3, Vec3};
use rne_render::{
    Camera, EnvironmentLighting, EnvironmentMap, PbrMaterial, RenderBackend, RenderScene,
    RenderSceneItem, TriangleMesh,
};
use rne_render_wgpu::{CameraOrbit, TaaSettings, WgpuRenderBackend};
use serde::Deserialize;
use std::f64::consts::{FRAC_PI_2, TAU};
use std::path::PathBuf;
use std::sync::Arc;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

#[derive(Deserialize)]
struct TraceBox {
    min: [f64; 3],
    max: [f64; 3],
    material: String,
}

#[derive(Deserialize)]
struct TraceRay {
    kind: String,
    points: Vec<[f64; 3]>,
}

#[derive(Deserialize)]
struct TraceFrame {
    t: f64,
    truth: [f64; 3],
    estimate: [f64; 3],
    source: String,
    rays: Vec<TraceRay>,
    lidar: Vec<[f64; 2]>,
    trail_samples: usize,
}

#[derive(Deserialize)]
struct TraceSample {
    truth: [f64; 2],
    estimate: [f64; 2],
    source: String,
}

#[derive(Deserialize)]
struct TraceFile {
    body_height_m: f64,
    lever_arms_m: Vec<[f64; 3]>,
    base_m: [f64; 3],
    office: [f64; 4],
    boxes: Vec<TraceBox>,
    samples: Vec<TraceSample>,
    frames: Vec<TraceFrame>,
}

/// ENU (east, north, up) to the renderer's Y-up frame (x east, y up, z south).
fn p(e: f64, n: f64, u: f64) -> Vec3 {
    Vec3::new(e, u, -n)
}

#[derive(Default)]
struct SolidMesh {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    texcoords: Vec<[f32; 2]>,
    indices: Vec<u32>,
}

impl SolidMesh {
    fn face(&mut self, points: [Vec3; 4], normal: Vec3) {
        let base = self.positions.len() as u32;
        self.positions
            .extend(points.map(|p| p.as_vec3().to_array()));
        self.normals.extend([normal.as_vec3().to_array(); 4]);
        self.texcoords
            .extend([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
        self.indices
            .extend([base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    /// An oriented box in renderer coordinates.
    fn box_at(&mut self, center: Vec3, half: Vec3, rotation: Quat) {
        let corner = |x: f64, y: f64, z: f64| {
            center + rotation * Vec3::new(x * half.x, y * half.y, z * half.z)
        };
        let (a, b, c, d) = (
            corner(-1., -1., -1.),
            corner(1., -1., -1.),
            corner(1., 1., -1.),
            corner(-1., 1., -1.),
        );
        let (e, f, g, h) = (
            corner(-1., -1., 1.),
            corner(1., -1., 1.),
            corner(1., 1., 1.),
            corner(-1., 1., 1.),
        );
        for (points, normal) in [
            ([a, d, c, b], -Vec3::Z),
            ([e, f, g, h], Vec3::Z),
            ([b, c, g, f], Vec3::X),
            ([e, h, d, a], -Vec3::X),
            ([d, h, g, c], Vec3::Y),
            ([a, b, f, e], -Vec3::Y),
        ] {
            self.face(points, rotation * normal);
        }
    }

    /// An axis-aligned box from ENU corners.
    fn enu_box(&mut self, min: [f64; 3], max: [f64; 3]) {
        let (lo, hi) = (p(min[0], min[1], min[2]), p(max[0], max[1], max[2]));
        let center = (lo + hi) * 0.5;
        let half = (hi - lo).abs() * 0.5;
        self.box_at(center, half, Quat::IDENTITY);
    }

    fn cylinder(&mut self, center: Vec3, radius: f64, half_height: f64, segments: usize) {
        for i in 0..segments {
            let a = TAU * i as f64 / segments as f64;
            let b = TAU * (i + 1) as f64 / segments as f64;
            let ring = |angle: f64, y: f64| {
                center + Vec3::new(radius * angle.cos(), y, radius * angle.sin())
            };
            let n = Vec3::new(((a + b) * 0.5).cos(), 0.0, ((a + b) * 0.5).sin());
            self.face(
                [
                    ring(a, -half_height),
                    ring(a, half_height),
                    ring(b, half_height),
                    ring(b, -half_height),
                ],
                n,
            );
            let top = center + Vec3::new(0.0, half_height, 0.0);
            self.face(
                [
                    top,
                    ring(b, half_height),
                    ring(a, half_height),
                    ring(a, half_height),
                ],
                Vec3::Y,
            );
            let bottom = center - Vec3::new(0.0, half_height, 0.0);
            self.face(
                [
                    bottom,
                    ring(a, -half_height),
                    ring(b, -half_height),
                    ring(b, -half_height),
                ],
                -Vec3::Y,
            );
        }
    }

    /// A beam of square cross-section between two renderer points.
    fn beam(&mut self, a: Vec3, b: Vec3, width: f64) {
        let delta = b - a;
        let length = delta.length();
        if length < 1.0e-6 {
            return;
        }
        let rotation = Quat::from_rotation_arc(Vec3::X, delta / length);
        self.box_at(
            (a + b) * 0.5,
            Vec3::new(length * 0.5, width * 0.5, width * 0.5),
            rotation,
        );
    }

    fn is_empty(&self) -> bool {
        self.positions.is_empty()
    }

    fn item(
        self,
        color: [f32; 4],
        roughness: f32,
        metallic: f32,
        emissive: [f32; 3],
    ) -> RenderSceneItem {
        let mut item = RenderScene::item_from_dynamic_mesh(
            TriangleMesh {
                positions: self.positions,
                normals: self.normals,
                texcoords: self.texcoords,
                indices: self.indices,
                skinning: None,
            },
            [1.0; 4],
        );
        item.material = PbrMaterial::new(color, roughness, metallic, emissive);
        item
    }
}

fn linear(rgb: [u8; 3]) -> [f32; 4] {
    let c = rgb.map(|v| {
        let s = f32::from(v) / 255.0;
        if s <= 0.04045 {
            s / 12.92
        } else {
            ((s + 0.055) / 1.055).powf(2.4)
        }
    });
    [c[0], c[1], c[2], 1.0]
}

fn push(
    scene: &mut RenderScene,
    mesh: SolidMesh,
    color: [f32; 4],
    roughness: f32,
    metallic: f32,
    emissive: [f32; 3],
) {
    if !mesh.is_empty() {
        scene
            .items
            .push(mesh.item(color, roughness, metallic, emissive));
    }
}

/// A clear-day sky with a bright horizon and darker ground.
fn environment() -> EnvironmentLighting {
    let (width, height) = (64, 32);
    let mut rgba = Vec::new();
    for row in 0..height {
        let y = row as f32 / (height - 1) as f32;
        for _ in 0..width {
            let (r, g, b) = if y < 0.5 {
                let up = 1.0 - y * 2.0;
                (0.55 - 0.35 * up, 0.68 - 0.30 * up, 0.92 - 0.12 * up)
            } else {
                (0.18, 0.17, 0.16)
            };
            rgba.extend([r, g, b, 1.0]);
        }
    }
    EnvironmentLighting {
        map: Some(Arc::new(
            EnvironmentMap::from_rgba32f(width, height, rgba).expect("environment"),
        )),
        intensity: 0.9,
        diffuse_strength: 0.75,
        specular_strength: 0.55,
        rotation_rad: 0.0,
    }
}

fn inside(office: [f64; 4], x: f64, y: f64) -> bool {
    x >= office[0] && x <= office[2] && y >= office[1] && y <= office[3]
}

/// Facade detail of a tower: floor bands and vertical mullions.
fn tower_details(
    min: [f64; 3],
    max: [f64; 3],
    bands: &mut SolidMesh,
    mullions: &mut SolidMesh,
    spacing_m: f64,
) {
    let inset = 0.12;
    let mut z = 3.6;
    while z < max[2] - 0.5 {
        bands.enu_box(
            [min[0] - inset, min[1] - inset, z - 0.25],
            [max[0] + inset, max[1] + inset, z + 0.25],
        );
        z += 3.6;
    }
    // Parapet.
    bands.enu_box(
        [min[0] - inset, min[1] - inset, max[2]],
        [max[0] + inset, max[1] + inset, max[2] + 0.9],
    );
    for (along_x, fixed, from, to) in [
        (true, min[1], min[0], max[0]),
        (true, max[1], min[0], max[0]),
        (false, min[0], min[1], max[1]),
        (false, max[0], min[1], max[1]),
    ] {
        let mut s = from + spacing_m;
        while s < to - 0.5 {
            let (a, b) = if along_x {
                (
                    [s - 0.12, fixed - inset, 0.0],
                    [s + 0.12, fixed + inset, max[2]],
                )
            } else {
                (
                    [fixed - inset, s - 0.12, 0.0],
                    [fixed + inset, s + 0.12, max[2]],
                )
            };
            mullions.enu_box(a, b);
            s += spacing_m;
        }
    }
}

fn static_city(trace: &TraceFile) -> RenderScene {
    let mut scene = RenderScene::new();
    push_ground(&mut scene, trace.office);
    push_trees(&mut scene);
    push_buildings(&mut scene, trace);
    push_base_station(&mut scene, trace.base_m);
    scene
}

fn push_ground(scene: &mut RenderScene, office: [f64; 4]) {
    // Ground: asphalt, plaza paving, sidewalks, office forecourt and floor.
    let mut asphalt = SolidMesh::default();
    asphalt.enu_box([-80.0, -120.0, -0.2], [240.0, 120.0, 0.0]);
    push(scene, asphalt, linear([58, 60, 64]), 0.95, 0.0, [0.0; 3]);
    let mut paving = SolidMesh::default();
    paving.enu_box([-14.0, -14.0, 0.0], [40.0, 14.0, 0.02]);
    paving.enu_box([120.0, -14.0, 0.0], [office[2] + 6.0, 14.0, 0.02]);
    for y in [9.0, -15.0] {
        paving.enu_box([40.0, y, 0.0], [120.0, y + 6.0, 0.12]);
    }
    push(scene, paving, linear([176, 170, 160]), 0.85, 0.0, [0.0; 3]);
    let mut seams = SolidMesh::default();
    let mut x = -14.0;
    while x <= 40.0 {
        seams.enu_box([x - 0.03, -14.0, 0.02], [x + 0.03, 14.0, 0.025]);
        x += 3.0;
    }
    let mut y = -14.0;
    while y <= 14.0 {
        seams.enu_box([-14.0, y - 0.03, 0.02], [40.0, y + 0.03, 0.025]);
        y += 3.0;
    }
    push(scene, seams, linear([140, 134, 126]), 0.9, 0.0, [0.0; 3]);
    let mut paint = SolidMesh::default();
    let mut x = 42.0;
    while x < 118.0 {
        paint.enu_box([x, 4.4, 0.0], [x + 3.0, 4.6, 0.012]);
        paint.enu_box([x, -4.6, 0.0], [x + 3.0, -4.4, 0.012]);
        x += 6.0;
    }
    push(
        scene,
        paint,
        linear([235, 235, 225]),
        0.6,
        0.0,
        [0.02, 0.02, 0.02],
    );
    let mut floor = SolidMesh::default();
    floor.enu_box([office[0], office[1], 0.0], [office[2], office[3], 0.03]);
    push(scene, floor, linear([200, 196, 186]), 0.6, 0.0, [0.0; 3]);
}

fn push_trees(scene: &mut RenderScene) {
    // Trees along the plaza.
    let mut trunks = SolidMesh::default();
    let mut crowns = SolidMesh::default();
    for (e, n) in [
        (-6.0, 11.0),
        (4.0, 11.5),
        (14.0, 11.0),
        (24.0, 11.5),
        (-6.0, -11.0),
        (4.0, -11.5),
        (14.0, -11.0),
        (24.0, -11.5),
    ] {
        trunks.cylinder(p(e, n, 1.6), 0.18, 1.6, 8);
        crowns.cylinder(p(e, n, 4.2), 1.8, 1.1, 10);
        crowns.cylinder(p(e, n, 5.4), 1.2, 0.6, 10);
    }
    push(scene, trunks, linear([92, 66, 44]), 0.9, 0.0, [0.0; 3]);
    push(scene, crowns, linear([62, 118, 58]), 0.85, 0.0, [0.0; 3]);
}

fn push_buildings(scene: &mut RenderScene, trace: &TraceFile) {
    let office = trace.office;
    // Buildings.
    let mut glass = SolidMesh::default();
    let mut concrete = SolidMesh::default();
    let mut metal = SolidMesh::default();
    let mut bands = SolidMesh::default();
    let mut mullions = SolidMesh::default();
    let mut walls = SolidMesh::default();
    let mut desks = SolidMesh::default();
    let mut roof_edge = SolidMesh::default();
    for b in &trace.boxes {
        let indoor = inside(
            office,
            0.5 * (b.min[0] + b.max[0]),
            0.5 * (b.min[1] + b.max[1]),
        ) && b.max[2] < 5.0;
        if indoor {
            if b.min[2] > 1.0 {
                // Roof cut away; keep its rim so the building reads as roofed.
                let (lo, hi) = (b.min, b.max);
                for (a, c) in [
                    ([lo[0], lo[1], lo[2]], [hi[0], lo[1] + 0.3, hi[2]]),
                    ([lo[0], hi[1] - 0.3, lo[2]], [hi[0], hi[1], hi[2]]),
                    ([lo[0], lo[1], lo[2]], [lo[0] + 0.3, hi[1], hi[2]]),
                    ([hi[0] - 0.3, lo[1], lo[2]], [hi[0], hi[1], hi[2]]),
                ] {
                    roof_edge.enu_box(a, c);
                }
            } else if b.max[2] < 1.0 {
                desks.enu_box(b.min, b.max);
            } else {
                walls.enu_box(b.min, b.max);
            }
            continue;
        }
        let spacing = match b.material.as_str() {
            "glass" => {
                glass.enu_box(b.min, b.max);
                3.0
            }
            "metal" => {
                metal.enu_box(b.min, b.max);
                2.0
            }
            _ => {
                concrete.enu_box(b.min, b.max);
                4.5
            }
        };
        tower_details(b.min, b.max, &mut bands, &mut mullions, spacing);
    }
    push(scene, glass, linear([70, 120, 160]), 0.06, 0.85, [0.0; 3]);
    push(scene, concrete, linear([196, 188, 172]), 0.9, 0.0, [0.0; 3]);
    push(scene, metal, linear([170, 176, 184]), 0.25, 0.9, [0.0; 3]);
    push(scene, bands, linear([214, 210, 200]), 0.8, 0.0, [0.0; 3]);
    push(scene, mullions, linear([60, 64, 70]), 0.4, 0.6, [0.0; 3]);
    push(scene, walls, linear([232, 230, 224]), 0.85, 0.0, [0.0; 3]);
    push(scene, desks, linear([150, 110, 70]), 0.7, 0.0, [0.0; 3]);
    push(
        scene,
        roof_edge,
        linear([120, 120, 124]),
        0.7,
        0.2,
        [0.0; 3],
    );
}

fn push_base_station(scene: &mut RenderScene, base: [f64; 3]) {
    // RTK base station on the rooftop: a mast and an antenna disc.
    let mut mast = SolidMesh::default();
    mast.cylinder(p(base[0], base[1], base[2] - 1.0), 0.06, 1.0, 8);
    push(scene, mast, linear([90, 90, 96]), 0.5, 0.7, [0.0; 3]);
    let mut disc = SolidMesh::default();
    disc.cylinder(p(base[0], base[1], base[2] + 0.05), 0.35, 0.06, 16);
    push(
        scene,
        disc,
        linear([245, 245, 245]),
        0.4,
        0.0,
        [0.25, 0.2, 0.45],
    );
}

fn source_color(source: &str) -> ([f32; 4], [f32; 3]) {
    match source {
        "gnss" => (linear([26, 152, 80]), [0.05, 0.45, 0.15]),
        "lidar" => (linear([44, 127, 184]), [0.08, 0.3, 0.6]),
        _ => (linear([244, 109, 67]), [0.6, 0.2, 0.08]),
    }
}

fn frame_scene(city: &RenderScene, trace: &TraceFile, frame: &TraceFrame) -> RenderScene {
    let mut scene = city.clone();
    push_robot(&mut scene, trace, frame);
    push_beams(&mut scene, frame);
    push_trails(&mut scene, trace, frame);
    scene
}

fn push_robot(scene: &mut RenderScene, trace: &TraceFile, frame: &TraceFrame) {
    let [x, y, yaw] = frame.truth;
    // Robot body, wheels, antenna masts with discs, and the LiDAR puck.
    let pose = Transform3::from_translation_rotation(p(x, y, 0.0), Quat::from_rotation_y(yaw));
    let mut body = SolidMesh::default();
    body.box_at(
        Vec3::new(0.0, 0.26, 0.0),
        Vec3::new(0.48, 0.16, 0.32),
        Quat::IDENTITY,
    );
    let mut deck = SolidMesh::default();
    deck.box_at(
        Vec3::new(0.0, 0.43, 0.0),
        Vec3::new(0.44, 0.015, 0.29),
        Quat::IDENTITY,
    );
    let mut wheels = SolidMesh::default();
    for (wx, wz) in [(0.3, 0.36), (-0.3, 0.36), (0.3, -0.36), (-0.3, -0.36)] {
        wheels.box_at(
            Vec3::new(wx, 0.11, wz),
            Vec3::new(0.11, 0.11, 0.05),
            Quat::IDENTITY,
        );
    }
    let mut masts = SolidMesh::default();
    let mut discs = SolidMesh::default();
    for arm in &trace.lever_arms_m {
        let top = trace.body_height_m + arm[2];
        masts.cylinder(
            Vec3::new(arm[0], 0.44 + 0.5 * (top - 0.44), -arm[1]),
            0.018,
            0.5 * (top - 0.44),
            8,
        );
        discs.cylinder(Vec3::new(arm[0], top, -arm[1]), 0.09, 0.025, 12);
    }
    let mut lidar = SolidMesh::default();
    lidar.cylinder(Vec3::new(0.12, 0.5, 0.0), 0.07, 0.05, 12);
    for (mesh, color, roughness, metallic, emissive) in [
        (body, linear([240, 164, 40]), 0.45, 0.1, [0.06, 0.03, 0.0]),
        (deck, linear([40, 42, 46]), 0.6, 0.2, [0.0; 3]),
        (wheels, linear([25, 25, 28]), 0.9, 0.0, [0.0; 3]),
        (masts, linear([150, 150, 156]), 0.4, 0.8, [0.0; 3]),
        (discs, linear([248, 248, 248]), 0.4, 0.0, [0.3, 0.3, 0.3]),
        (lidar, linear([30, 30, 34]), 0.3, 0.4, [0.0, 0.25, 0.35]),
    ] {
        let mut item = mesh.item(color, roughness, metallic, emissive);
        item.transform = pose.mul_transform(&item.transform);
        scene.items.push(item);
    }
}

fn push_beams(scene: &mut RenderScene, frame: &TraceFrame) {
    // Satellite signal beams from the rear antenna.
    let mut los = SolidMesh::default();
    let mut nlos = SolidMesh::default();
    let mut diffracted = SolidMesh::default();
    let mut hits = SolidMesh::default();
    for ray in &frame.rays {
        let mesh = match ray.kind.as_str() {
            "los" => &mut los,
            "nlos" => &mut nlos,
            _ => &mut diffracted,
        };
        for pair in ray.points.windows(2) {
            let (a, b) = (
                p(pair[0][0], pair[0][1], pair[0][2]),
                p(pair[1][0], pair[1][1], pair[1][2]),
            );
            mesh.beam(a, b, if ray.kind == "los" { 0.05 } else { 0.08 });
        }
        for point in ray
            .points
            .iter()
            .skip(1)
            .take(ray.points.len().saturating_sub(2))
        {
            hits.box_at(
                p(point[0], point[1], point[2]),
                Vec3::splat(0.35),
                Quat::IDENTITY,
            );
        }
    }
    push(scene, los, [0.1, 0.9, 0.3, 1.0], 0.5, 0.0, [0.15, 1.4, 0.4]);
    push(
        scene,
        nlos,
        [1.0, 0.15, 0.1, 1.0],
        0.5,
        0.0,
        [1.8, 0.2, 0.12],
    );
    push(
        scene,
        diffracted,
        [0.6, 0.4, 1.0, 1.0],
        0.5,
        0.0,
        [0.8, 0.5, 1.6],
    );
    push(scene, hits, [1.0, 0.9, 0.3, 1.0], 0.5, 0.0, [2.0, 1.6, 0.3]);

    // LiDAR returns.
    let mut points = SolidMesh::default();
    for point in frame.lidar.iter().step_by(2) {
        points.box_at(
            p(point[0], point[1], 0.3),
            Vec3::splat(0.05),
            Quat::IDENTITY,
        );
    }
    push(
        scene,
        points,
        [0.1, 0.8, 1.0, 1.0],
        0.5,
        0.0,
        [0.1, 1.2, 1.6],
    );
}

fn push_trails(scene: &mut RenderScene, trace: &TraceFile, frame: &TraceFrame) {
    // Trails on the ground: truth (white) and the estimate by source.
    let samples = &trace.samples[..frame.trail_samples.min(trace.samples.len())];
    let mut truth_trail = SolidMesh::default();
    let mut estimate_trails: [SolidMesh; 3] = Default::default();
    let decimated: Vec<&TraceSample> = samples.iter().step_by(3).collect();
    for pair in decimated.windows(2) {
        truth_trail.beam(
            p(pair[0].truth[0], pair[0].truth[1], 0.04),
            p(pair[1].truth[0], pair[1].truth[1], 0.04),
            0.08,
        );
        let index = match pair[1].source.as_str() {
            "gnss" => 0,
            "lidar" => 1,
            _ => 2,
        };
        estimate_trails[index].beam(
            p(pair[0].estimate[0], pair[0].estimate[1], 0.07),
            p(pair[1].estimate[0], pair[1].estimate[1], 0.07),
            0.16,
        );
    }
    push(
        scene,
        truth_trail,
        [0.95, 0.95, 0.95, 1.0],
        0.6,
        0.0,
        [0.3, 0.3, 0.3],
    );
    for (mesh, source) in estimate_trails
        .into_iter()
        .zip(["gnss", "lidar", "dead_reckoning"])
    {
        let (color, emissive) = source_color(source);
        push(scene, mesh, color, 0.5, 0.0, emissive);
    }
    // The estimate: a ring on the ground in the source colour.
    let (color, emissive) = source_color(&frame.source);
    let mut ring = SolidMesh::default();
    let [ex, ey, _] = frame.estimate;
    for i in 0..24 {
        let a = TAU * i as f64 / 24.0;
        let b = TAU * (i + 1) as f64 / 24.0;
        ring.beam(
            p(ex + 0.8 * a.cos(), ey + 0.8 * a.sin(), 0.08),
            p(ex + 0.8 * b.cos(), ey + 0.8 * b.sin(), 0.08),
            0.09,
        );
    }
    push(scene, ring, color, 0.4, 0.0, emissive);
}

/// A chase camera that eases between an outdoor view (low, behind the robot,
/// showing the buildings and the sky) and an indoor view (high, looking
/// down through the cut-away roof).
struct ChaseCamera {
    yaw: f64,
    pitch: f64,
    distance: f64,
    focus: Vec3,
    initialized: bool,
}

impl ChaseCamera {
    fn update(&mut self, trace: &TraceFile, frame: &TraceFrame) -> CameraOrbit {
        let [x, y, heading] = frame.truth;
        let indoor = inside(trace.office, x, y) || inside(trace.office, x + 6.0, y);
        let (pitch, distance) = if indoor { (0.5, 11.0) } else { (1.02, 17.0) };
        // Behind the robot: eye offset (sin yaw, cos yaw) in (x, z).
        let yaw = (-heading.cos()).atan2(heading.sin());
        let focus = p(x, y, if indoor { 0.4 } else { 4.5 });
        if !self.initialized {
            *self = Self {
                yaw,
                pitch,
                distance,
                focus,
                initialized: true,
            };
        } else {
            let mut delta = yaw - self.yaw;
            while delta > std::f64::consts::PI {
                delta -= TAU;
            }
            while delta < -std::f64::consts::PI {
                delta += TAU;
            }
            self.yaw += 0.12 * delta;
            self.pitch += 0.15 * (pitch - self.pitch);
            self.distance += 0.15 * (distance - self.distance);
            self.focus += (focus - self.focus) * 0.6;
        }
        CameraOrbit {
            yaw_rad: self.yaw,
            pitch_rad: self.pitch.clamp(0.2, FRAC_PI_2 - 0.1),
            distance_m: self.distance,
            focus: self.focus,
        }
    }
}

fn write_png(path: &std::path::Path, width: u32, height: u32, rgba: &[u8]) -> Result<()> {
    let mut encoder = png::Encoder::new(std::fs::File::create(path)?, width, height);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.write_header()?.write_image_data(rgba)?;
    Ok(())
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let value = |flag: &str| {
        args.iter()
            .position(|a| a == flag)
            .and_then(|i| args.get(i + 1))
            .cloned()
    };
    let trace_path =
        PathBuf::from(value("--trace").unwrap_or_else(|| "target/gnss3d/trace.json".into()));
    let output = PathBuf::from(value("--output").unwrap_or_else(|| "target/gnss3d/frames".into()));
    let width: u32 = value("--width").map_or(Ok(960), |v| v.parse())?;
    let height: u32 = value("--height").map_or(Ok(540), |v| v.parse())?;
    let stride: usize = value("--stride").map_or(Ok(1), |v| v.parse())?;
    let limit: usize = value("--frames").map_or(Ok(usize::MAX), |v| v.parse())?;

    let trace: TraceFile = serde_json::from_slice(&std::fs::read(&trace_path)?)?;
    std::fs::create_dir_all(&output)?;
    let mut backend = WgpuRenderBackend::new()?;
    backend.set_environment(environment());
    backend.set_taa(TaaSettings {
        enabled: false,
        ..TaaSettings::default()
    });
    println!("adapter: {}", backend.adapter_info().name);
    let city = static_city(&trace);
    let mut camera = Camera::new(width, height, 0.9);
    camera.far_m = 600.0;
    let mut chase = ChaseCamera {
        yaw: 0.0,
        pitch: 0.0,
        distance: 0.0,
        focus: Vec3::ZERO,
        initialized: false,
    };
    let mut index_entries = Vec::new();
    let started = std::time::Instant::now();
    for (index, frame) in trace
        .frames
        .iter()
        .step_by(stride.max(1))
        .take(limit)
        .enumerate()
    {
        let orbit = chase.update(&trace, frame);
        let scene = frame_scene(&city, &trace, frame);
        let image = backend.render_scene_camera(
            &camera,
            &orbit.camera_transform(),
            &scene,
            [0.55, 0.68, 0.92, 1.0],
        )?;
        let name = format!("frame_{index:04}.png");
        write_png(&output.join(&name), width, height, &image.color.rgba8)?;
        index_entries.push(serde_json::json!({"file": name, "t": frame.t}));
        if index % 25 == 0 {
            println!(
                "rendered frame {index} (t = {:.1} s, {:.1} s elapsed)",
                frame.t,
                started.elapsed().as_secs_f64()
            );
        }
    }
    std::fs::write(
        output.join("frames.json"),
        serde_json::to_vec_pretty(&serde_json::json!({
            "renderer": "Robot Native Engine WgpuRenderBackend",
            "adapter": backend.adapter_info().name,
            "trace": trace_path,
            "width": width,
            "height": height,
            "stride": stride,
            "frames": index_entries,
        }))?,
    )?;
    println!(
        "wrote {} frames to {}",
        index_entries.len(),
        output.display()
    );
    Ok(())
}
