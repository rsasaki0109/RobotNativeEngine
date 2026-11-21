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
    kind: String,
}

#[derive(Deserialize)]
struct TraceCanopy {
    min: [f64; 3],
    max: [f64; 3],
}

#[derive(Deserialize)]
struct TraceVehicle {
    kind: String,
    min: [f64; 3],
    max: [f64; 3],
    color: usize,
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
    vehicles: Vec<TraceVehicle>,
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
    canopies: Vec<TraceCanopy>,
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
/// Facade detail of a tower: floor bands, a parapet and vertical mullions.
fn tower_details(
    min: [f64; 3],
    max: [f64; 3],
    bands: &mut SolidMesh,
    mullions: &mut SolidMesh,
    spacing_m: f64,
) {
    let inset = 0.12;
    let mut z = min[2] + 3.6;
    while z < max[2] - 0.5 {
        bands.enu_box(
            [min[0] - inset, min[1] - inset, z - 0.25],
            [max[0] + inset, max[1] + inset, z + 0.25],
        );
        z += 3.6;
    }
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
                    [s - 0.12, fixed - inset, min[2]],
                    [s + 0.12, fixed + inset, max[2]],
                )
            } else {
                (
                    [fixed - inset, s - 0.12, min[2]],
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
    push_buildings(&mut scene, trace);
    push_street_furniture(&mut scene, trace);
    push_base_station(&mut scene, trace.base_m);
    scene
}

fn push_ground(scene: &mut RenderScene, office: [f64; 4]) {
    let mut asphalt = SolidMesh::default();
    asphalt.enu_box([-120.0, -140.0, -0.2], [260.0, 140.0, 0.0]);
    push(scene, asphalt, linear([54, 56, 60]), 0.95, 0.0, [0.0; 3]);
    // Plaza paving, forecourt, sidewalks (raised kerbs), office floor.
    let mut paving = SolidMesh::default();
    paving.enu_box([-20.0, -18.0, 0.0], [40.0, 18.0, 0.02]);
    paving.enu_box([120.0, -15.0, 0.0], [office[0], 15.0, 0.02]);
    for y in [9.0, -15.0] {
        paving.enu_box([40.0, y, 0.0], [120.0, y + 6.0, 0.14]);
    }
    push(scene, paving, linear([178, 172, 162]), 0.85, 0.0, [0.0; 3]);
    let mut kerb = SolidMesh::default();
    for y in [8.85, -9.15] {
        kerb.enu_box([40.0, y, 0.0], [120.0, y + 0.3, 0.16]);
    }
    push(scene, kerb, linear([150, 148, 144]), 0.8, 0.0, [0.0; 3]);
    let mut seams = SolidMesh::default();
    let mut x = -20.0;
    while x <= 40.0 {
        seams.enu_box([x - 0.03, -18.0, 0.02], [x + 0.03, 18.0, 0.025]);
        x += 3.0;
    }
    let mut y = -18.0;
    while y <= 18.0 {
        seams.enu_box([-20.0, y - 0.03, 0.02], [40.0, y + 0.03, 0.025]);
        y += 3.0;
    }
    push(scene, seams, linear([138, 132, 124]), 0.9, 0.0, [0.0; 3]);
    // Lane markings, parking bays and zebra crossings at the cross streets.
    let mut paint = SolidMesh::default();
    let mut x = 42.0;
    while x < 118.0 {
        paint.enu_box([x, -0.1, 0.0], [x + 3.0, 0.1, 0.012]);
        x += 6.0;
    }
    for y in [-6.0, 6.0] {
        paint.enu_box([41.0, y - 0.08, 0.0], [119.0, y + 0.08, 0.012]);
    }
    for x0 in [64.0, 94.0] {
        let mut y = -8.6;
        while y < 8.6 {
            paint.enu_box([x0, y, 0.0], [x0 + 3.5, y + 0.5, 0.013]);
            y += 1.0;
        }
    }
    push(
        scene,
        paint,
        linear([236, 236, 228]),
        0.6,
        0.0,
        [0.03, 0.03, 0.03],
    );
    let mut floor = SolidMesh::default();
    floor.enu_box([office[0], office[1], 0.0], [office[2], office[3], 0.03]);
    push(scene, floor, linear([204, 200, 190]), 0.6, 0.0, [0.0; 3]);
}

/// Tree crowns from the trace's canopy volumes (the trunks are boxes).
/// Crowns between the camera and the robot are cut away, like the office
/// roof, so the robot stays visible.
fn push_trees(scene: &mut RenderScene, trace: &TraceFile, eye: Vec3, robot: Vec3) {
    let mut crowns = SolidMesh::default();
    let mut dark = SolidMesh::default();
    for c in &trace.canopies {
        let centre = p(
            0.5 * (c.min[0] + c.max[0]),
            0.5 * (c.min[1] + c.max[1]),
            0.5 * (c.min[2] + c.max[2]),
        );
        let along = robot - eye;
        let s = ((centre - eye).dot(along) / along.length_squared()).clamp(0.0, 1.0);
        let closest = eye + along * s;
        if (centre - closest).length() < 0.5 * (c.max[0] - c.min[0]) + 1.5 {
            continue;
        }
        let (cx, cy) = (0.5 * (c.min[0] + c.max[0]), 0.5 * (c.min[1] + c.max[1]));
        let r = 0.5 * (c.max[0] - c.min[0]);
        let (z0, z1) = (c.min[2], c.max[2]);
        let h = z1 - z0;
        crowns.cylinder(p(cx, cy, z0 + 0.45 * h), r, 0.25 * h, 12);
        crowns.cylinder(p(cx, cy, z0 + 0.75 * h), 0.75 * r, 0.18 * h, 12);
        dark.cylinder(p(cx, cy, z0 + 0.2 * h), 0.8 * r, 0.12 * h, 12);
    }
    push(scene, crowns, linear([72, 128, 62]), 0.85, 0.0, [0.0; 3]);
    push(scene, dark, linear([46, 92, 44]), 0.9, 0.0, [0.0; 3]);
}

fn push_buildings(scene: &mut RenderScene, trace: &TraceFile) {
    let mut glass = SolidMesh::default();
    let mut concrete = SolidMesh::default();
    let mut metal = SolidMesh::default();
    let mut bands = SolidMesh::default();
    let mut mullions = SolidMesh::default();
    let mut walls = SolidMesh::default();
    let mut desks = SolidMesh::default();
    let mut roof_edge = SolidMesh::default();
    for b in &trace.boxes {
        match b.kind.as_str() {
            "glass" => {
                glass.enu_box(b.min, b.max);
                tower_details(b.min, b.max, &mut bands, &mut mullions, 3.0);
            }
            "metal" => {
                metal.enu_box(b.min, b.max);
                tower_details(b.min, b.max, &mut bands, &mut mullions, 2.0);
            }
            "concrete" => {
                concrete.enu_box(b.min, b.max);
                tower_details(b.min, b.max, &mut bands, &mut mullions, 4.5);
            }
            "office" | "pillar" => walls.enu_box(b.min, b.max),
            "desk" => desks.enu_box(b.min, b.max),
            "roof" => {
                // Cut away so the inside shows; keep the rim.
                let (lo, hi) = (b.min, b.max);
                for (a, c) in [
                    ([lo[0], lo[1], lo[2]], [hi[0], lo[1] + 0.3, hi[2]]),
                    ([lo[0], hi[1] - 0.3, lo[2]], [hi[0], hi[1], hi[2]]),
                    ([lo[0], lo[1], lo[2]], [lo[0] + 0.3, hi[1], hi[2]]),
                    ([hi[0] - 0.3, lo[1], lo[2]], [hi[0], hi[1], hi[2]]),
                ] {
                    roof_edge.enu_box(a, c);
                }
            }
            _ => {}
        }
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

/// Lamp posts, traffic signals, the bus shelter, benches, the kiosk and the
/// tree trunks.
fn push_street_furniture(scene: &mut RenderScene, trace: &TraceFile) {
    let mut poles = SolidMesh::default();
    let mut lamps = SolidMesh::default();
    let mut signal_heads = SolidMesh::default();
    let mut red = SolidMesh::default();
    let mut green = SolidMesh::default();
    let mut shelter_frame = SolidMesh::default();
    let mut shelter_glass = SolidMesh::default();
    let mut wood = SolidMesh::default();
    let mut trunks = SolidMesh::default();
    let mut kiosk = SolidMesh::default();
    for b in &trace.boxes {
        let (cx, cy) = (0.5 * (b.min[0] + b.max[0]), 0.5 * (b.min[1] + b.max[1]));
        let h = b.max[2];
        match b.kind.as_str() {
            "lamp" => {
                poles.cylinder(p(cx, cy, 0.5 * h), 0.08, 0.5 * h, 8);
                // The arm leans over the road (toward y = 0).
                let toward = if cy < 0.0 { 1.0 } else { -1.0 };
                poles.beam(p(cx, cy, h), p(cx, cy + toward * 1.4, h + 0.2), 0.08);
                lamps.box_at(
                    p(cx, cy + toward * 1.5, h + 0.12),
                    Vec3::new(0.3, 0.06, 0.18),
                    Quat::IDENTITY,
                );
            }
            "signal" => {
                poles.cylinder(p(cx, cy, 0.5 * h), 0.09, 0.5 * h, 8);
                signal_heads.box_at(
                    p(cx, cy, h - 0.5),
                    Vec3::new(0.18, 0.5, 0.18),
                    Quat::IDENTITY,
                );
                red.box_at(
                    p(cx + 0.19, cy, h - 0.2),
                    Vec3::new(0.02, 0.1, 0.1),
                    Quat::IDENTITY,
                );
                green.box_at(
                    p(cx + 0.19, cy, h - 0.8),
                    Vec3::new(0.02, 0.1, 0.1),
                    Quat::IDENTITY,
                );
            }
            "shelter" => {
                shelter_glass.enu_box(
                    [b.min[0], b.max[1] - 0.06, 0.0],
                    [b.max[0], b.max[1], h - 0.1],
                );
                shelter_frame.enu_box(
                    [b.min[0] - 0.1, b.min[1] - 0.1, h - 0.12],
                    [b.max[0] + 0.1, b.max[1] + 0.1, h],
                );
                for x in [b.min[0], b.max[0]] {
                    shelter_frame.enu_box([x - 0.05, b.max[1] - 0.1, 0.0], [x + 0.05, b.max[1], h]);
                }
                wood.enu_box(
                    [b.min[0] + 0.4, b.max[1] - 0.5, 0.42],
                    [b.max[0] - 0.4, b.max[1] - 0.1, 0.48],
                );
            }
            "bench" => {
                wood.enu_box([b.min[0], b.min[1], 0.38], [b.max[0], b.max[1], h]);
                poles.enu_box(
                    [b.min[0] + 0.1, b.min[1], 0.0],
                    [b.min[0] + 0.2, b.max[1], 0.38],
                );
                poles.enu_box(
                    [b.max[0] - 0.2, b.min[1], 0.0],
                    [b.max[0] - 0.1, b.max[1], 0.38],
                );
            }
            "kiosk" => {
                kiosk.enu_box(b.min, b.max);
                shelter_frame.enu_box(
                    [b.min[0] - 0.4, b.min[1] - 0.4, b.max[2]],
                    [b.max[0] + 0.4, b.max[1] + 0.4, b.max[2] + 0.2],
                );
            }
            "trunk" => trunks.cylinder(p(cx, cy, 0.5 * h), 0.16, 0.5 * h, 8),
            _ => {}
        }
    }
    push(scene, poles, linear([70, 74, 80]), 0.5, 0.6, [0.0; 3]);
    push(
        scene,
        lamps,
        linear([250, 240, 210]),
        0.4,
        0.0,
        [1.2, 1.1, 0.8],
    );
    push(
        scene,
        signal_heads,
        linear([30, 30, 32]),
        0.6,
        0.2,
        [0.0; 3],
    );
    push(scene, red, [1.0, 0.1, 0.05, 1.0], 0.4, 0.0, [2.0, 0.2, 0.1]);
    push(
        scene,
        green,
        [0.1, 1.0, 0.4, 1.0],
        0.4,
        0.0,
        [0.2, 1.6, 0.6],
    );
    push(
        scene,
        shelter_frame,
        linear([90, 96, 104]),
        0.4,
        0.7,
        [0.0; 3],
    );
    push(
        scene,
        shelter_glass,
        linear([150, 190, 210]),
        0.05,
        0.8,
        [0.0; 3],
    );
    push(scene, wood, linear([140, 100, 62]), 0.7, 0.0, [0.0; 3]);
    push(scene, trunks, linear([92, 66, 44]), 0.9, 0.0, [0.0; 3]);
    push(scene, kiosk, linear([200, 70, 50]), 0.6, 0.0, [0.0; 3]);
}

/// Cars (body, glass cabin, wheels) and buses from the frame's vehicles.
fn push_vehicles(scene: &mut RenderScene, frame: &TraceFrame) {
    const PAINT: [[u8; 3]; 6] = [
        [200, 30, 36],
        [235, 235, 238],
        [30, 34, 40],
        [40, 90, 170],
        [160, 166, 172],
        [220, 170, 40],
    ];
    let mut bodies: [SolidMesh; 6] = Default::default();
    let mut glass = SolidMesh::default();
    let mut wheels = SolidMesh::default();
    let mut bus_body = SolidMesh::default();
    let mut lights = SolidMesh::default();
    for v in &frame.vehicles {
        let (x0, y0, x1, y1, h) = (v.min[0], v.min[1], v.max[0], v.max[1], v.max[2]);
        if v.kind == "bus" {
            bus_body.enu_box([x0, y0, 0.35], [x1, y1, h]);
            glass.enu_box([x0 - 0.02, y0 - 0.02, 1.4], [x1 + 0.02, y1 + 0.02, h - 0.5]);
        } else {
            let body = &mut bodies[v.color % PAINT.len()];
            body.enu_box([x0, y0, 0.3], [x1, y1, 0.95]);
            let inset = 0.22 * (x1 - x0);
            glass.enu_box(
                [x0 + inset, y0 + 0.08, 0.95],
                [x1 - inset * 0.8, y1 - 0.08, h],
            );
            body.enu_box(
                [x0 + inset + 0.1, y0 + 0.1, h - 0.06],
                [x1 - inset * 0.8 - 0.1, y1 - 0.1, h],
            );
            lights.enu_box([x1, y0 + 0.15, 0.6], [x1 + 0.03, y0 + 0.45, 0.75]);
            lights.enu_box([x1, y1 - 0.45, 0.6], [x1 + 0.03, y1 - 0.15, 0.75]);
        }
        let r = if v.kind == "bus" { 0.5 } else { 0.33 };
        for x in [x0 + 0.22 * (x1 - x0), x1 - 0.22 * (x1 - x0)] {
            for y in [y0 + 0.12, y1 - 0.12] {
                wheels.enu_box([x - r, y - 0.12, 0.0], [x + r, y + 0.12, 2.0 * r]);
            }
        }
    }
    for (mesh, color) in bodies.into_iter().zip(PAINT) {
        push(scene, mesh, linear(color), 0.25, 0.5, [0.0; 3]);
    }
    push(scene, bus_body, linear([30, 120, 90]), 0.35, 0.3, [0.0; 3]);
    push(scene, glass, linear([24, 30, 40]), 0.05, 0.6, [0.0; 3]);
    push(scene, wheels, linear([20, 20, 22]), 0.9, 0.0, [0.0; 3]);
    push(
        scene,
        lights,
        [1.0, 0.95, 0.8, 1.0],
        0.3,
        0.0,
        [1.5, 1.4, 1.1],
    );
}

fn push_base_station(scene: &mut RenderScene, base: [f64; 3]) {
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

fn frame_scene(
    city: &RenderScene,
    trace: &TraceFile,
    frame: &TraceFrame,
    eye: Vec3,
) -> RenderScene {
    let mut scene = city.clone();
    push_trees(
        &mut scene,
        trace,
        eye,
        p(frame.truth[0], frame.truth[1], 0.8),
    );
    push_vehicles(&mut scene, frame);
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
        let eye = orbit.camera_transform().translation;
        let scene = frame_scene(&city, &trace, frame, eye);
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
