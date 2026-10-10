//! Original warehouse diorama rendered from a simulator-produced fleet trace.
//! Rendering never changes robot poses, fleet state, or simulator telemetry.

use rne_math::{Quat, Transform3, Vec3};
use rne_render::{
    Camera, EnvironmentLighting, EnvironmentMap, PbrMaterial, RenderBackend, RenderScene,
    RenderSceneItem, TriangleMesh,
};
use rne_render_wgpu::{CameraOrbit, TaaSettings, WgpuRenderBackend};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap, error::Error, f64::consts::TAU, fs, path::PathBuf, sync::Arc,
    time::Instant,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Debug, Deserialize)]
struct Trace {
    schema_version: u32,
    robot_count: usize,
    extent_m: Extent,
    racks: Vec<Rack>,
    stations: Vec<Station>,
    #[serde(default)]
    lanes: Vec<[[f64; 2]; 2]>,
    frames: Vec<Frame>,
    #[serde(default)]
    validation: serde_json::Value,
    #[serde(default = "default_robot_radius")]
    robot_radius_m: f64,
    #[serde(default)]
    fixture_only: bool,
    #[serde(default)]
    source_provenance: Option<SourceProvenance>,
}

#[derive(Debug, Deserialize, Serialize, PartialEq, Eq)]
struct SourceProvenance {
    format_version: u32,
    simulation_source_sha256: String,
    package_manifest_sha256: String,
    workspace_manifest_sha256: String,
    workspace_lock_sha256: String,
    engine_sources_sha256: BTreeMap<String, String>,
}

fn compiled_source_provenance() -> SourceProvenance {
    let hash = |bytes: &[u8]| format!("{:x}", Sha256::digest(bytes));
    let inputs: [(&str, &[u8]); 8] = [
        (
            "crates/rne_core/Cargo.toml",
            include_bytes!("../../crates/rne_core/Cargo.toml"),
        ),
        (
            "crates/rne_core/src/rng.rs",
            include_bytes!("../../crates/rne_core/src/rng.rs"),
        ),
        (
            "crates/rne_core/src/time.rs",
            include_bytes!("../../crates/rne_core/src/time.rs"),
        ),
        (
            "crates/rne_nav/Cargo.toml",
            include_bytes!("../../crates/rne_nav/Cargo.toml"),
        ),
        (
            "crates/rne_nav/src/coordination.rs",
            include_bytes!("../../crates/rne_nav/src/coordination.rs"),
        ),
        (
            "crates/rne_nav/src/grid.rs",
            include_bytes!("../../crates/rne_nav/src/grid.rs"),
        ),
        (
            "crates/rne_world/Cargo.toml",
            include_bytes!("../../crates/rne_world/Cargo.toml"),
        ),
        (
            "crates/rne_world/src/resources.rs",
            include_bytes!("../../crates/rne_world/src/resources.rs"),
        ),
    ];
    SourceProvenance {
        format_version: 1,
        simulation_source_sha256: hash(include_bytes!("main.rs")),
        package_manifest_sha256: hash(include_bytes!("Cargo.toml")),
        workspace_manifest_sha256: hash(include_bytes!("../../Cargo.toml")),
        workspace_lock_sha256: hash(include_bytes!("../../Cargo.lock")),
        engine_sources_sha256: inputs
            .into_iter()
            .map(|(path, bytes)| (path.to_owned(), hash(bytes)))
            .collect(),
    }
}

fn default_robot_radius() -> f64 {
    0.65
}

#[derive(Debug, Deserialize)]
struct Extent {
    width: f64,
    depth: f64,
}
#[derive(Debug, Deserialize)]
struct Rack {
    x_m: f64,
    z_m: f64,
    width_m: f64,
    depth_m: f64,
    height_m: f64,
}
#[derive(Debug, Deserialize)]
struct Station {
    x_m: f64,
    z_m: f64,
    kind: String,
}
#[derive(Debug, Deserialize)]
struct Frame {
    time_s: f64,
    robots: Vec<Robot>,
    incident: Incident,
}
#[derive(Debug, Deserialize)]
struct Robot {
    id: usize,
    x_m: f64,
    z_m: f64,
    yaw_rad: f64,
    state: String,
    loaded: bool,
    #[serde(default)]
    remaining_path: Vec<[f64; 2]>,
}
#[derive(Debug, Deserialize)]
struct Incident {
    active: bool,
    x_m: f64,
    z_m: f64,
    width_m: f64,
    depth_m: f64,
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

    fn box_at(&mut self, center: Vec3, half: Vec3, rotation: Quat) {
        let a = Vec3::new(-half.x, -half.y, -half.z);
        let b = Vec3::new(half.x, -half.y, -half.z);
        let c = Vec3::new(half.x, half.y, -half.z);
        let d = Vec3::new(-half.x, half.y, -half.z);
        let e = Vec3::new(-half.x, -half.y, half.z);
        let f = Vec3::new(half.x, -half.y, half.z);
        let g = Vec3::new(half.x, half.y, half.z);
        let h = Vec3::new(-half.x, half.y, half.z);
        for (points, normal) in [
            ([a, d, c, b], -Vec3::Z),
            ([e, f, g, h], Vec3::Z),
            ([b, c, g, f], Vec3::X),
            ([e, h, d, a], -Vec3::X),
            ([d, h, g, c], Vec3::Y),
            ([a, b, f, e], -Vec3::Y),
        ] {
            self.face(points.map(|p| center + rotation * p), rotation * normal);
        }
    }

    fn cuboid(&mut self, center: Vec3, half: Vec3) {
        self.box_at(center, half, Quat::IDENTITY);
    }

    fn cylinder(&mut self, center: Vec3, radius: f64, half_length: f64, rotation: Quat) {
        const N: usize = 16;
        for i in 0..N {
            let a = TAU * i as f64 / N as f64;
            let b = TAU * (i + 1) as f64 / N as f64;
            let p = Vec3::new(radius * a.cos(), radius * a.sin(), -half_length);
            let q = Vec3::new(radius * b.cos(), radius * b.sin(), -half_length);
            let r = Vec3::new(radius * b.cos(), radius * b.sin(), half_length);
            let s = Vec3::new(radius * a.cos(), radius * a.sin(), half_length);
            let n = Vec3::new(((a + b) * 0.5).cos(), ((a + b) * 0.5).sin(), 0.0);
            self.face([p, q, r, s].map(|p| center + rotation * p), rotation * n);
            self.face(
                [Vec3::new(0.0, 0.0, half_length), s, r, r].map(|p| center + rotation * p),
                rotation * Vec3::Z,
            );
            self.face(
                [Vec3::new(0.0, 0.0, -half_length), q, p, p].map(|p| center + rotation * p),
                -rotation * Vec3::Z,
            );
        }
    }

    fn beveled_body(&mut self, half_x: f64, half_z: f64, bottom: f64, top: f64, bevel: f64) {
        let ring = [
            (-half_x + bevel, -half_z),
            (half_x - bevel, -half_z),
            (half_x, -half_z + bevel),
            (half_x, half_z - bevel),
            (half_x - bevel, half_z),
            (-half_x + bevel, half_z),
            (-half_x, half_z - bevel),
            (-half_x, -half_z + bevel),
        ];
        for i in 0..ring.len() {
            let (ax, az) = ring[i];
            let (bx, bz) = ring[(i + 1) % ring.len()];
            let n = Vec3::new(bz - az, 0.0, ax - bx).normalize();
            self.face(
                [
                    Vec3::new(ax, bottom, az),
                    Vec3::new(ax, top, az),
                    Vec3::new(bx, top, bz),
                    Vec3::new(bx, bottom, bz),
                ],
                n,
            );
            self.face(
                [
                    Vec3::new(0.0, top, 0.0),
                    Vec3::new(bx, top, bz),
                    Vec3::new(ax, top, az),
                    Vec3::new(ax, top, az),
                ],
                Vec3::Y,
            );
            self.face(
                [
                    Vec3::new(0.0, bottom, 0.0),
                    Vec3::new(ax, bottom, az),
                    Vec3::new(bx, bottom, bz),
                    Vec3::new(bx, bottom, bz),
                ],
                -Vec3::Y,
            );
        }
    }

    fn line(&mut self, a: Vec3, b: Vec3, width: f64) {
        let delta = b - a;
        if delta.length() < 1.0e-9 {
            return;
        }
        let yaw = -delta.z.atan2(delta.x);
        self.box_at(
            (a + b) * 0.5,
            Vec3::new(delta.length() * 0.5, 0.006, width * 0.5),
            Quat::from_rotation_y(yaw),
        );
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

fn linear_color(rgb: [u8; 3]) -> [f32; 4] {
    let linear = rgb.map(|channel| {
        let srgb = f32::from(channel) / 255.0;
        if srgb <= 0.04045 {
            srgb / 12.92
        } else {
            ((srgb + 0.055) / 1.055).powf(2.4)
        }
    });
    [linear[0], linear[1], linear[2], 1.0]
}

fn environment() -> EnvironmentLighting {
    let (width, height) = (64, 32);
    let mut rgba = Vec::new();
    for row in 0..height {
        let y = row as f32 / height as f32;
        for column in 0..width {
            let x = column as f32 / width as f32;
            let skylight = (1.0 - y * 2.0).max(0.0);
            let strip = (x * std::f32::consts::TAU * 4.0).sin().max(0.0).powf(14.0) * skylight;
            let base = if y > 0.55 {
                0.008
            } else {
                0.045 + 0.55 * skylight
            };
            rgba.extend([
                base + strip * 1.7,
                base + strip * 1.7,
                base * 1.06 + strip * 1.75,
                1.0,
            ]);
        }
    }
    EnvironmentLighting {
        map: Some(Arc::new(
            EnvironmentMap::from_rgba32f(width, height, rgba).unwrap(),
        )),
        intensity: 0.65,
        diffuse_strength: 0.5,
        specular_strength: 0.35,
        rotation_rad: 0.0,
    }
}

fn push_shell(scene: &mut RenderScene, trace: &Trace) {
    let (w, d) = (trace.extent_m.width, trace.extent_m.depth);
    let mut slab = SolidMesh::default();
    slab.cuboid(
        Vec3::new(w * 0.5, -0.22, d * 0.5),
        Vec3::new(w * 0.5 + 0.3, 0.22, d * 0.5 + 0.3),
    );
    scene
        .items
        .push(slab.item([0.24, 0.28, 0.32, 1.0], 0.88, 0.0, [0.0; 3]));
    let mut floor = SolidMesh::default();
    floor.cuboid(
        Vec3::new(w * 0.5, 0.008, d * 0.5),
        Vec3::new(w * 0.5, 0.008, d * 0.5),
    );
    scene
        .items
        .push(floor.item([0.24, 0.28, 0.29, 1.0], 0.94, 0.0, [0.0; 3]));
    let mut seams = SolidMesh::default();
    for x in (0..w as usize).step_by(4) {
        seams.cuboid(
            Vec3::new(x as f64, 0.025, d * 0.5),
            Vec3::new(0.006, 0.003, d * 0.5),
        );
    }
    for z in (0..d as usize).step_by(4) {
        seams.cuboid(
            Vec3::new(w * 0.5, 0.025, z as f64),
            Vec3::new(w * 0.5, 0.003, 0.006),
        );
    }
    scene
        .items
        .push(seams.item([0.30, 0.33, 0.34, 1.0], 0.9, 0.0, [0.0; 3]));
    let mut wall = SolidMesh::default();
    // Rear walls and low near curbs leave the operational floor visible.
    wall.cuboid(
        Vec3::new(w * 0.5, 1.6, -0.18),
        Vec3::new(w * 0.5 + 0.2, 1.6, 0.18),
    );
    wall.cuboid(
        Vec3::new(-0.18, 1.6, d * 0.5),
        Vec3::new(0.18, 1.6, d * 0.5),
    );
    wall.cuboid(
        Vec3::new(w * 0.5, 0.2, d + 0.18),
        Vec3::new(w * 0.5 + 0.2, 0.2, 0.18),
    );
    wall.cuboid(
        Vec3::new(w + 0.18, 0.2, d * 0.5),
        Vec3::new(0.18, 0.2, d * 0.5),
    );
    scene
        .items
        .push(wall.item([0.62, 0.66, 0.67, 1.0], 0.78, 0.0, [0.0; 3]));
    let mut wall_trim = SolidMesh::default();
    wall_trim.cuboid(
        Vec3::new(w * 0.5, 0.45, 0.02),
        Vec3::new(w * 0.5, 0.055, 0.06),
    );
    wall_trim.cuboid(
        Vec3::new(0.02, 0.45, d * 0.5),
        Vec3::new(0.06, 0.055, d * 0.5),
    );
    scene
        .items
        .push(wall_trim.item([0.04, 0.13, 0.17, 1.0], 0.55, 0.25, [0.0; 3]));
    let mut lamps = SolidMesh::default();
    for x in (3..w as usize).step_by(6) {
        lamps.cuboid(
            Vec3::new(x as f64, 2.85, 0.025),
            Vec3::new(1.25, 0.06, 0.025),
        );
    }
    scene
        .items
        .push(lamps.item([0.85, 0.91, 0.91, 1.0], 0.25, 0.0, [1.8, 2.1, 2.1]));
}

fn push_lanes(scene: &mut RenderScene, trace: &Trace) {
    let mut aisles = SolidMesh::default();
    let mut paint = SolidMesh::default();
    for lane in &trace.lanes {
        let a = Vec3::new(lane[0][0], 0.047, lane[0][1]);
        let b = Vec3::new(lane[1][0], 0.047, lane[1][1]);
        let distance = (b - a).length();
        if distance < 0.1 {
            continue;
        }
        let delta = (b - a) / distance;
        aisles.line(a - Vec3::Y * 0.022, b - Vec3::Y * 0.022, 1.8);
        let side = Vec3::new(-delta.z, 0.0, delta.x) * 0.86;
        for offset in [-side, side] {
            paint.line(a + offset, b + offset, 0.042);
        }
        let mut pos = 0.4;
        while pos < distance - 0.4 {
            paint.line(
                a + delta * pos,
                a + delta * (pos + 0.35).min(distance),
                0.075,
            );
            pos += 1.55;
        }
        let mut pos = 2.0;
        while pos < distance - 1.0 {
            let tip = a + delta * pos;
            let tail = tip - delta * 0.43;
            let wing = Vec3::new(-delta.z, 0.0, delta.x) * 0.22;
            paint.line(tip, tail + wing, 0.07);
            paint.line(tip, tail - wing, 0.07);
            pos += 5.0;
        }
    }
    scene
        .items
        .push(aisles.item([0.14, 0.19, 0.21, 1.0], 0.9, 0.0, [0.0; 3]));
    scene
        .items
        .push(paint.item([0.64, 0.70, 0.69, 1.0], 0.9, 0.0, [0.0; 3]));
}

fn push_racks(scene: &mut RenderScene, trace: &Trace) {
    let mut steel = SolidMesh::default();
    let mut beam = SolidMesh::default();
    let mut wood = SolidMesh::default();
    let mut stock = SolidMesh::default();
    let mut tape = SolidMesh::default();
    for (index, rack) in trace.racks.iter().enumerate() {
        let bays = (rack.width_m / 2.0).round().max(1.0) as usize;
        let bay_width = rack.width_m / bays as f64;
        for column in 0..=bays {
            let x = rack.x_m - rack.width_m * 0.5 + column as f64 * bay_width;
            for side in [-1.0, 1.0] {
                steel.cuboid(
                    Vec3::new(
                        x,
                        rack.height_m * 0.5,
                        rack.z_m + side * (rack.depth_m * 0.5 - 0.07),
                    ),
                    Vec3::new(0.075, rack.height_m * 0.5, 0.075),
                );
            }
        }
        for level in 0..3 {
            let y = 0.18 + level as f64 * rack.height_m / 3.0;
            for side in [-1.0, 1.0] {
                beam.cuboid(
                    Vec3::new(rack.x_m, y, rack.z_m + side * (rack.depth_m * 0.5 - 0.035)),
                    Vec3::new(rack.width_m * 0.5, 0.08, 0.075),
                );
            }
            for bay in 0..bays {
                let x = rack.x_m - rack.width_m * 0.5 + (bay as f64 + 0.5) * bay_width;
                wood.cuboid(
                    Vec3::new(x, y + 0.08, rack.z_m),
                    Vec3::new(bay_width * 0.43, 0.035, rack.depth_m * 0.43),
                );
                for package in 0..2 {
                    let px = x + (package as f64 - 0.5) * bay_width * 0.38;
                    let height = 0.30 + 0.06 * ((index + bay + level + package) % 3) as f64;
                    let half = Vec3::new(bay_width * 0.16, height * 0.5, rack.depth_m * 0.33);
                    let center = Vec3::new(px, y + 0.125 + height * 0.5, rack.z_m);
                    stock.cuboid(center, half);
                    tape.cuboid(
                        center + Vec3::new(0.0, half.y + 0.003, 0.0),
                        Vec3::new(0.04, 0.003, half.z),
                    );
                    tape.cuboid(
                        center + Vec3::new(0.0, 0.0, half.z + 0.002),
                        Vec3::new(0.04, half.y, 0.003),
                    );
                }
            }
        }
    }
    scene
        .items
        .push(steel.item([0.055, 0.24, 0.43, 1.0], 0.38, 0.5, [0.0; 3]));
    scene
        .items
        .push(beam.item([0.83, 0.27, 0.055, 1.0], 0.45, 0.2, [0.0; 3]));
    scene
        .items
        .push(wood.item([0.36, 0.24, 0.13, 1.0], 0.92, 0.0, [0.0; 3]));
    scene
        .items
        .push(stock.item([0.52, 0.37, 0.20, 1.0], 0.86, 0.0, [0.0; 3]));
    scene
        .items
        .push(tape.item([0.70, 0.59, 0.41, 1.0], 0.8, 0.0, [0.0; 3]));
}

fn push_stations(scene: &mut RenderScene, trace: &Trace) {
    for kind in ["pick", "drop", "charge"] {
        let mut pads = SolidMesh::default();
        let mut edge = SolidMesh::default();
        let mut hardware = SolidMesh::default();
        for station in trace.stations.iter().filter(|s| s.kind == kind) {
            pads.cuboid(
                Vec3::new(station.x_m, 0.033, station.z_m),
                Vec3::new(0.72, 0.008, 0.72),
            );
            for dz in [-0.70, 0.70] {
                edge.cuboid(
                    Vec3::new(station.x_m, 0.055, station.z_m + dz),
                    Vec3::new(0.72, 0.01, 0.035),
                );
            }
            for dx in [-0.70, 0.70] {
                edge.cuboid(
                    Vec3::new(station.x_m + dx, 0.055, station.z_m),
                    Vec3::new(0.035, 0.01, 0.72),
                );
            }
            if kind == "charge" {
                hardware.cuboid(
                    Vec3::new(station.x_m, 0.48, station.z_m - 0.80),
                    Vec3::new(0.28, 0.48, 0.10),
                );
                edge.cuboid(
                    Vec3::new(station.x_m, 0.76, station.z_m - 0.69),
                    Vec3::new(0.18, 0.055, 0.012),
                );
            }
        }
        let color = linear_color(match kind {
            "charge" => [179, 149, 252],
            "drop" => [45, 207, 167],
            _ => [83, 176, 245],
        });
        scene.items.push(pads.item(
            [color[0] * 0.25, color[1] * 0.25, color[2] * 0.25, 1.0],
            0.86,
            0.0,
            [0.0; 3],
        ));
        scene.items.push(edge.item(
            color,
            0.55,
            0.0,
            [color[0] * 0.1, color[1] * 0.1, color[2] * 0.1],
        ));
        if !hardware.positions.is_empty() {
            scene
                .items
                .push(hardware.item([0.13, 0.18, 0.20, 1.0], 0.45, 0.25, [0.0; 3]));
        }
    }
}

fn building(trace: &Trace) -> RenderScene {
    let mut scene = RenderScene::new();
    push_shell(&mut scene, trace);
    push_lanes(&mut scene, trace);
    push_racks(&mut scene, trace);
    push_stations(&mut scene, trace);
    scene.items.retain(|item| {
        item.mesh
            .as_ref()
            .is_none_or(|mesh| !mesh.indices.is_empty())
    });
    scene
}

struct RobotModel {
    body: Vec<RenderSceneItem>,
    payload: Vec<RenderSceneItem>,
    lights: Vec<RenderSceneItem>,
}

fn robot_model() -> RobotModel {
    let mut body = SolidMesh::default();
    let mut dark = SolidMesh::default();
    let mut deck = SolidMesh::default();
    let mut wheel = SolidMesh::default();
    let mut glass = SolidMesh::default();
    let mut lights = SolidMesh::default();
    body.beveled_body(0.53, 0.34, 0.16, 0.43, 0.10);
    dark.beveled_body(0.55, 0.355, 0.12, 0.22, 0.10);
    deck.beveled_body(0.45, 0.30, 0.43, 0.49, 0.065);
    for x in [-0.31, 0.31] {
        for z in [-0.385, 0.385] {
            wheel.cylinder(Vec3::new(x, 0.16, z), 0.16, 0.048, Quat::IDENTITY);
            deck.cylinder(Vec3::new(x, 0.16, z * 1.065), 0.077, 0.009, Quat::IDENTITY);
        }
    }
    let vertical = Quat::from_rotation_x(std::f64::consts::FRAC_PI_2);
    dark.cylinder(Vec3::new(0.39, 0.53, 0.0), 0.095, 0.072, vertical);
    glass.cylinder(Vec3::new(0.39, 0.555, 0.0), 0.10, 0.025, vertical);
    lights.cuboid(Vec3::new(0.535, 0.315, 0.0), Vec3::new(0.009, 0.022, 0.22));
    for z in [-0.345, 0.345] {
        lights.cuboid(Vec3::new(0.0, 0.32, z), Vec3::new(0.28, 0.022, 0.009));
    }
    // State-coloured roof trim and a nonphysical floor marker improve legibility.
    for z in [-0.28, 0.28] {
        lights.cuboid(Vec3::new(-0.02, 0.50, z), Vec3::new(0.34, 0.008, 0.03));
    }
    for i in 0..40 {
        let a = TAU * i as f64 / 40.0;
        let b = TAU * (i + 1) as f64 / 40.0;
        lights.face(
            [
                Vec3::new(0.63 * a.cos(), 0.047, 0.63 * a.sin()),
                Vec3::new(0.63 * b.cos(), 0.047, 0.63 * b.sin()),
                Vec3::new(0.66 * b.cos(), 0.047, 0.66 * b.sin()),
                Vec3::new(0.66 * a.cos(), 0.047, 0.66 * a.sin()),
            ],
            Vec3::Y,
        );
    }
    let body = vec![
        body.item([0.90, 0.96, 0.95, 1.0], 0.36, 0.12, [0.0; 3]),
        dark.item([0.025, 0.044, 0.056, 1.0], 0.52, 0.12, [0.0; 3]),
        deck.item([0.17, 0.22, 0.25, 1.0], 0.4, 0.35, [0.0; 3]),
        wheel.item([0.014, 0.019, 0.021, 1.0], 0.93, 0.0, [0.0; 3]),
        glass.item([0.006, 0.19, 0.23, 1.0], 0.18, 0.25, [0.025, 0.12, 0.15]),
    ];
    let mut carton = SolidMesh::default();
    let mut band = SolidMesh::default();
    carton.cuboid(Vec3::new(-0.04, 0.80, 0.0), Vec3::new(0.38, 0.31, 0.30));
    band.cuboid(Vec3::new(-0.04, 1.113, 0.0), Vec3::new(0.045, 0.004, 0.30));
    band.cuboid(Vec3::new(-0.04, 0.80, 0.304), Vec3::new(0.045, 0.31, 0.004));
    let payload = vec![
        carton.item([0.66, 0.46, 0.24, 1.0], 0.86, 0.0, [0.0; 3]),
        band.item([0.82, 0.71, 0.48, 1.0], 0.85, 0.0, [0.0; 3]),
    ];
    let light_mesh = Arc::new(TriangleMesh {
        positions: lights.positions,
        normals: lights.normals,
        texcoords: lights.texcoords,
        indices: lights.indices,
        skinning: None,
    });
    let colors = [
        [83u8, 176, 245],
        [179, 149, 252],
        [251, 186, 76],
        [140, 162, 177],
        [45, 207, 167],
    ]
    .map(linear_color);
    let lights = colors
        .into_iter()
        .map(|color| {
            let mut item = RenderScene::item_from_dynamic_mesh((*light_mesh).clone(), [1.0; 4]);
            item.mesh = Some(Arc::clone(&light_mesh));
            item.material = PbrMaterial::new(
                color,
                0.3,
                0.0,
                [color[0] * 1.5, color[1] * 1.5, color[2] * 1.5],
            );
            item
        })
        .collect();
    RobotModel {
        body,
        payload,
        lights,
    }
}

fn frame_scene(static_scene: &RenderScene, frame: &Frame, model: &RobotModel) -> RenderScene {
    let mut scene = static_scene.clone();
    for robot in &frame.robots {
        // The trace's +yaw rotates +X toward +Z, opposite the renderer's Y rotation.
        let pose = Transform3::from_translation_rotation(
            Vec3::new(robot.x_m, 0.0, robot.z_m),
            Quat::from_rotation_y(-robot.yaw_rad),
        );
        let index = match robot.state.as_str() {
            "charging" | "to_charge" => 1,
            "waiting" => 2,
            "idle" => 3,
            "to_drop" | "dropping" => 4,
            _ => 0,
        };
        for part in model
            .body
            .iter()
            .chain(std::iter::once(&model.lights[index]))
            .chain(if robot.loaded {
                model.payload.iter()
            } else {
                model.payload[..0].iter()
            })
        {
            let mut item = part.clone();
            item.transform = pose.mul_transform(&item.transform);
            scene.items.push(item);
        }
    }
    let mut routes = SolidMesh::default();
    for robot in frame.robots.iter().filter(|r| r.id % 13 == 0) {
        let mut previous = Vec3::new(robot.x_m, 0.05, robot.z_m);
        for point in robot.remaining_path.iter().take(7) {
            let next = Vec3::new(point[0], 0.05, point[1]);
            routes.line(previous, next, 0.06);
            previous = next;
        }
    }
    if !routes.positions.is_empty() {
        scene
            .items
            .push(routes.item([0.025, 0.55, 0.60, 1.0], 0.65, 0.0, [0.025, 0.10, 0.12]));
    }
    if frame.incident.active {
        let i = &frame.incident;
        let mut cones = SolidMesh::default();
        let mut white = SolidMesh::default();
        for x in [-0.5, 0.5] {
            for z in [-0.5, 0.5] {
                let pos = Vec3::new(i.x_m + x * i.width_m, 0.0, i.z_m + z * i.depth_m);
                cones.cuboid(pos + Vec3::new(0.0, 0.04, 0.0), Vec3::new(0.20, 0.04, 0.20));
                cones.cylinder(
                    pos + Vec3::new(0.0, 0.25, 0.0),
                    0.09,
                    0.19,
                    Quat::from_rotation_x(std::f64::consts::FRAC_PI_2),
                );
                white.cylinder(
                    pos + Vec3::new(0.0, 0.31, 0.0),
                    0.094,
                    0.035,
                    Quat::from_rotation_x(std::f64::consts::FRAC_PI_2),
                );
            }
        }
        let mut zone = SolidMesh::default();
        for sign in [-1.0, 1.0] {
            zone.line(
                Vec3::new(
                    i.x_m - i.width_m * 0.5,
                    0.045,
                    i.z_m + sign * i.depth_m * 0.5,
                ),
                Vec3::new(
                    i.x_m + i.width_m * 0.5,
                    0.045,
                    i.z_m + sign * i.depth_m * 0.5,
                ),
                0.07,
            );
            zone.line(
                Vec3::new(
                    i.x_m + sign * i.width_m * 0.5,
                    0.045,
                    i.z_m - i.depth_m * 0.5,
                ),
                Vec3::new(
                    i.x_m + sign * i.width_m * 0.5,
                    0.045,
                    i.z_m + i.depth_m * 0.5,
                ),
                0.07,
            );
        }
        scene
            .items
            .push(cones.item([0.91, 0.24, 0.025, 1.0], 0.75, 0.0, [0.0; 3]));
        scene
            .items
            .push(white.item([0.9, 0.92, 0.85, 1.0], 0.85, 0.0, [0.0; 3]));
        scene
            .items
            .push(zone.item([1.0, 0.41, 0.035, 1.0], 0.7, 0.0, [0.14, 0.04, 0.0]));
    }
    scene
}

fn write_png(path: &std::path::Path, width: u32, height: u32, rgba: &[u8]) -> Result<()> {
    let mut encoder = png::Encoder::new(fs::File::create(path)?, width, height);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.set_compression(png::Compression::Fast);
    encoder.write_header()?.write_image_data(rgba)?;
    Ok(())
}

fn camera_for_trace(trace: &Trace, width: u32, height: u32) -> (Camera, CameraOrbit, Transform3) {
    let mut camera = Camera::new(width, height, 0.68);
    camera.far_m = 180.0;
    let mut orbit = CameraOrbit {
        focus: Vec3::new(trace.extent_m.width * 0.5, 0.3, trace.extent_m.depth * 0.5),
        yaw_rad: 0.25,
        pitch_rad: 0.67,
        distance_m: 55.0,
    };
    // Fit the actual warehouse's bounds rather than depending on a guessed layout.
    loop {
        let vp = camera.view_projection(&orbit.camera_transform());
        let fits = [-0.6, trace.extent_m.width + 0.6].into_iter().all(|x| {
            [-0.6, trace.extent_m.depth + 0.6].into_iter().all(|z| {
                [0.0, 3.5].into_iter().all(|y| {
                    let clip = vp * Vec3::new(x, y, z).extend(1.0);
                    clip.w > 0.0
                        && (clip.x / clip.w).abs() < 0.965
                        && (clip.y / clip.w).abs() < 0.945
                })
            })
        });
        if fits {
            break;
        }
        orbit.distance_m += 0.5;
    }
    let view = orbit.camera_transform();
    (camera, orbit, view)
}

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let mut input = None;
    let mut output = None;
    let mut first = 0usize;
    let mut last = None;
    let mut allow_fixture = false;
    let mut allow_diagnostic = false;
    let (mut width, mut height) = (1024u32, 656u32);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--help" | "-h" => {
                println!("Render an actual recorded warehouse fleet trace.\n\n--trace PATH --output DIRECTORY [--frame INDEX | --range FIRST END_EXCLUSIVE]\n[--width 1024] [--height 656]\n\nFinal capture requires passed validation and matching compiled simulation sources.\n--preview-diagnostic and --allow-preview-fixture produce unqualified previews only.");
                return Ok(());
            }
            "--trace" => input = Some(PathBuf::from(args.next().ok_or("missing trace path")?)),
            "--output" => output = Some(PathBuf::from(args.next().ok_or("missing output path")?)),
            "--frame" => {
                first = args.next().ok_or("missing frame index")?.parse()?;
                last = Some(first + 1);
            }
            "--range" => {
                first = args.next().ok_or("missing first index")?.parse()?;
                last = Some(args.next().ok_or("missing exclusive end index")?.parse()?);
            }
            "--width" => width = args.next().ok_or("missing width")?.parse()?,
            "--height" => height = args.next().ok_or("missing height")?.parse()?,
            "--allow-preview-fixture" => allow_fixture = true,
            "--preview-diagnostic" => allow_diagnostic = true,
            _ => return Err(format!("unknown option {arg}").into()),
        }
    }
    let input = input.ok_or("--trace is required")?;
    let output = output.ok_or("--output is required")?;
    let trace_bytes = fs::read(&input)?;
    let trace_sha256 = format!("{:x}", Sha256::digest(&trace_bytes));
    let source_sha256 = format!("{:x}", Sha256::digest(include_bytes!("render.rs")));
    let manifest_sha256 = format!("{:x}", Sha256::digest(include_bytes!("Cargo.toml")));
    let compiled_provenance = compiled_source_provenance();
    let simulation_source_sha256 = compiled_provenance.simulation_source_sha256.clone();
    let trace: Trace = serde_json::from_slice(&trace_bytes)?;
    let source_provenance_verified = trace.source_provenance.as_ref() == Some(&compiled_provenance);
    if !(source_provenance_verified || allow_diagnostic || trace.fixture_only && allow_fixture) {
        return Err("final capture requires trace provenance matching the compiled example, workspace manifests/lock and eight declared engine inputs; stale or standalone traces cannot qualify".into());
    }
    if trace.fixture_only && !allow_fixture {
        return Err(
            "synthetic fixtures require --allow-preview-fixture and cannot qualify final media"
                .into(),
        );
    }
    let trace_validated = trace
        .validation
        .get("passed")
        .and_then(serde_json::Value::as_bool)
        == Some(true);
    if !(trace_validated || allow_diagnostic || trace.fixture_only && allow_fixture) {
        return Err("final capture requires trace.validation.passed=true; use --preview-diagnostic only for diagnostic previews".into());
    }
    if trace.schema_version != 1 || trace.robot_count != 40 {
        return Err("expected schema 1 and 40 robots".into());
    }
    let last = last.unwrap_or(trace.frames.len());
    if first >= last || last > trace.frames.len() {
        return Err("invalid frame range".into());
    }
    fs::create_dir_all(&output)?;
    let mut backend = WgpuRenderBackend::new()?;
    backend.set_environment(environment());
    backend.set_taa(TaaSettings {
        enabled: false,
        ..TaaSettings::default()
    });
    println!("adapter: {:?}", backend.adapter_info());
    let (camera, orbit, view) = camera_for_trace(&trace, width, height);
    let static_scene = building(&trace);
    let model = robot_model();
    let max_body_radius_m = model
        .body
        .iter()
        .chain(&model.payload)
        .filter_map(|item| item.mesh.as_ref())
        .flat_map(|mesh| &mesh.positions)
        .map(|p| (f64::from(p[0]).powi(2) + f64::from(p[2]).powi(2)).sqrt())
        .fold(0.0f64, f64::max);
    if max_body_radius_m > trace.robot_radius_m {
        return Err(format!(
            "robot body radius {max_body_radius_m} exceeds simulated collision radius"
        )
        .into());
    }
    let mut frame_metadata = Vec::new();
    let started = Instant::now();
    for (index, frame) in trace.frames.iter().enumerate().take(last).skip(first) {
        if frame.robots.len() != trace.robot_count {
            return Err(format!("frame {index} robot count mismatch").into());
        }
        let scene = frame_scene(&static_scene, frame, &model);
        let frame_started = Instant::now();
        let image =
            backend.render_scene_camera(&camera, &view, &scene, [0.019, 0.029, 0.042, 1.0])?;
        let name = format!("frame-{index:04}.png");
        write_png(&output.join(&name), width, height, &image.color.rgba8)?;
        let vp = camera.view_projection(&view);
        let positions: Vec<_> = frame
            .robots
            .iter()
            .map(|r| {
                let clip = vp * Vec3::new(r.x_m, 0.75, r.z_m).extend(1.0);
                serde_json::json!({"id":r.id,"x_px":(clip.x/clip.w*0.5+0.5)*width as f64,
                "y_px":(0.5-clip.y/clip.w*0.5)*height as f64})
            })
            .collect();
        frame_metadata.push(serde_json::json!({"index":index,"time_s":frame.time_s,"file":name,"robot_screen_positions":positions,
            "render_ms":frame_started.elapsed().as_secs_f64()*1000.0,
            "rgba_sha256":format!("{:x}",Sha256::digest(&image.color.rgba8))}));
        if index == first || index % 20 == 0 || index + 1 == last {
            println!(
                "rendered {index}/{last}, items={}, elapsed_s={:.2}",
                scene.items.len(),
                started.elapsed().as_secs_f64()
            );
        }
    }
    fs::write(
        output.join("render-metadata.json"),
        serde_json::to_vec_pretty(&serde_json::json!({
            "renderer":"Robot Native Engine WgpuRenderBackend", "source_trace":input,
            "trace_sha256":trace_sha256,"renderer_source_sha256":source_sha256,"renderer_manifest_sha256":manifest_sha256,
            "simulation_source_sha256":simulation_source_sha256,"simulation_source_provenance_verified":source_provenance_verified,
            "source_provenance":compiled_provenance,
            "width":width,"height":height,"frame_start":first,"frame_end_exclusive":last,
            "camera":{"yaw_rad":orbit.yaw_rad,"pitch_rad":orbit.pitch_rad,"distance_m":orbit.distance_m,
                "focus_m":orbit.focus.to_array(),"fov_y_rad":camera.fov_y_rad},
            "adapter_name":backend.adapter_info().name,"adapter_backend":format!("{:?}",backend.adapter_info().backend),
            "geometry":"Original procedural warehouse and AMR geometry; no copied WareTwin assets",
            "fixture_only":trace.fixture_only,"max_amr_body_radius_m":max_body_radius_m,
            "simulated_robot_radius_m":trace.robot_radius_m,"trace_validated":trace_validated,"diagnostic_only":!trace_validated || !source_provenance_verified || trace.fixture_only,
            "status_marker_is_physical":false,
            "taa_enabled":false,"capture_elapsed_s":started.elapsed().as_secs_f64(),
            "frames":frame_metadata,
        }))?,
    )?;
    Ok(())
}
