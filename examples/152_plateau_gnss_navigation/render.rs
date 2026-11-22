//! 3D render of a run through the PLATEAU city with RNE's wgpu renderer:
//! the LOD2 buildings (glass towers and concrete), the terrain, the street
//! furniture, the robot with its trail coloured by localization source, the
//! ray-traced satellite signals (line of sight green, NLOS red through their
//! reflection points, diffraction purple), the LiDAR beams and returns
//! (cyan), and pedestrians and rain when the run has them, from a chase
//! camera. PNG frames plus `frames.json` (for `encode.py`'s HUD) go to the
//! output directory.

use crate::gnss::{SolutionLabel, LEVER_ARMS_M};
use crate::nav::{FrameRecord, RunReport, LIDAR_HEIGHT_M};
use crate::plateau::PlateauCity;
use rne_gnss::{trace_satellite, GnssScene, PathKind, SignalPath, SurfaceMaterial};
use rne_math::{Quat, Vec3};
use rne_render::{
    Camera, EnvironmentLighting, EnvironmentMap, PbrMaterial, RenderBackend, RenderScene,
    RenderSceneItem, TriangleMesh,
};
use rne_render_wgpu::{CameraOrbit, TaaSettings, WgpuRenderBackend};
use rne_slam::LocalizationSource;
use std::f64::consts::{FRAC_PI_2, PI, TAU};
use std::path::Path;
use std::sync::Arc;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

const WIDTH: u32 = 960;
const HEIGHT: u32 = 540;
/// Satellite rays are drawn this far toward the sky, meters.
const SKY_REACH_M: f64 = 160.0;

/// ENU (east, north, up) to the renderer's Y-up frame (x east, y up, z south).
fn p(e: f64, n: f64, u: f64) -> Vec3 {
    Vec3::new(e, u, -n)
}

fn pv(v: Vec3) -> Vec3 {
    p(v.x, v.y, v.z)
}

#[derive(Default)]
struct SolidMesh {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    texcoords: Vec<[f32; 2]>,
    indices: Vec<u32>,
}

impl SolidMesh {
    fn triangle(&mut self, corners: [Vec3; 3], normal: Vec3) {
        let base = self.positions.len() as u32;
        self.positions
            .extend(corners.map(|c| c.as_vec3().to_array()));
        self.normals.extend([normal.as_vec3().to_array(); 3]);
        self.texcoords.extend([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]);
        self.indices.extend([base, base + 1, base + 2]);
    }

    fn face(&mut self, points: [Vec3; 4], normal: Vec3) {
        self.triangle([points[0], points[1], points[2]], normal);
        self.triangle([points[0], points[2], points[3]], normal);
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
    if !mesh.positions.is_empty() {
        scene
            .items
            .push(mesh.item(color, roughness, metallic, emissive));
    }
}

/// A clear-day sky.
fn environment() -> EnvironmentLighting {
    let (width, height) = (64, 32);
    let mut rgba = Vec::new();
    for row in 0..height {
        let y = row as f32 / (height - 1) as f32;
        for _ in 0..width {
            let (r, g, b) = if y < 0.5 {
                let up = 1.0 - y * 2.0;
                (0.58 - 0.35 * up, 0.70 - 0.30 * up, 0.93 - 0.12 * up)
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

/// Buildings, furniture, terrain and the planned route.
fn static_city(city: &PlateauCity, route: &[(f64, f64)]) -> RenderScene {
    let mut scene = RenderScene::new();
    let (mut glass, mut concrete, mut roofs, mut furniture) = (
        SolidMesh::default(),
        SolidMesh::default(),
        SolidMesh::default(),
        SolidMesh::default(),
    );
    for tri in city.mesh.triangles() {
        let corners = tri.vertices.map(pv);
        let normal = pv(tri.normal);
        let mesh = if tri.material == SurfaceMaterial::ABSORBER {
            if tri.object < 1_000_000 {
                continue; // ground and closure surfaces
            }
            &mut furniture
        } else if tri.normal.z > 0.7 {
            &mut roofs
        } else if tri.material == SurfaceMaterial::GLASS {
            &mut glass
        } else {
            &mut concrete
        };
        mesh.triangle(corners, normal);
    }
    push(
        &mut scene,
        glass,
        linear([120, 150, 175]),
        0.18,
        0.55,
        [0.0; 3],
    );
    push(
        &mut scene,
        concrete,
        linear([205, 198, 186]),
        0.85,
        0.0,
        [0.0; 3],
    );
    push(
        &mut scene,
        roofs,
        linear([150, 148, 145]),
        0.9,
        0.0,
        [0.0; 3],
    );
    push(
        &mut scene,
        furniture,
        linear([90, 92, 96]),
        0.6,
        0.3,
        [0.0; 3],
    );

    // Terrain around the route on a 4 m grid.
    let (mut x0, mut y0, mut x1, mut y1) = (
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    );
    for &(x, y) in route {
        x0 = x0.min(x);
        y0 = y0.min(y);
        x1 = x1.max(x);
        y1 = y1.max(y);
    }
    let margin = 220.0;
    let cell = 4.0;
    let (nx, ny) = (
        ((x1 - x0 + 2.0 * margin) / cell) as usize,
        ((y1 - y0 + 2.0 * margin) / cell) as usize,
    );
    let mut ground = SolidMesh::default();
    let at = |i: usize, j: usize| {
        let (x, y) = (x0 - margin + i as f64 * cell, y0 - margin + j as f64 * cell);
        p(x, y, city.ground_m(x, y) - 0.05)
    };
    for j in 0..ny {
        for i in 0..nx {
            let (a, b, c, d) = (at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1));
            let normal = (c - a).cross(b - a).normalize_or_zero();
            let normal = if normal.y < 0.0 { -normal } else { normal };
            ground.face([a, d, c, b], normal);
        }
    }
    push(
        &mut scene,
        ground,
        linear([70, 72, 76]),
        0.95,
        0.0,
        [0.0; 3],
    );

    // The planned route as a faint line on the ground.
    let mut line = SolidMesh::default();
    for pair in route.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        line.beam(
            p(a.0, a.1, city.ground_m(a.0, a.1) + 0.03),
            p(b.0, b.1, city.ground_m(b.0, b.1) + 0.03),
            0.25,
        );
    }
    push(
        &mut scene,
        line,
        linear([235, 235, 225]),
        0.7,
        0.0,
        [0.25, 0.25, 0.22],
    );
    scene
}

/// The robot, its trail, the satellite signals and the LiDAR returns.
fn frame_items(
    scene: &mut RenderScene,
    gnss_scene: &GnssScene,
    city: &PlateauCity,
    run: &RunReport,
    frame: &FrameRecord,
) {
    let (x, y, yaw, z) = (
        frame.truth.x_m,
        frame.truth.y_m,
        frame.truth.yaw_rad,
        frame.ground_m,
    );
    let mut body = SolidMesh::default();
    let rotation = Quat::from_rotation_y(yaw);
    body.box_at(p(x, y, z + 0.3), Vec3::new(0.45, 0.25, 0.3), rotation);
    let mut antennas = SolidMesh::default();
    for lever in LEVER_ARMS_M {
        let (s, c) = yaw.sin_cos();
        let (ax, ay) = (x + lever.x * c - lever.y * s, y + lever.x * s + lever.y * c);
        antennas.box_at(
            p(ax, ay, z + 0.85),
            Vec3::new(0.09, 0.05, 0.09),
            Quat::IDENTITY,
        );
    }
    push(
        scene,
        body,
        linear([250, 200, 40]),
        0.5,
        0.1,
        [0.25, 0.18, 0.02],
    );
    push(
        scene,
        antennas,
        linear([240, 240, 240]),
        0.4,
        0.2,
        [0.3, 0.3, 0.3],
    );

    // Trail coloured by localization source.
    let mut trails = [
        SolidMesh::default(),
        SolidMesh::default(),
        SolidMesh::default(),
    ];
    // The track is sampled every 0.5 s; this frame's tick is in it.
    let upto = ((frame.t / 0.5).round() as usize + 1).min(run.track.len());
    for pair in run.track[..upto].windows(2) {
        let (a, b) = (pair[0].0, pair[1].0);
        let k = match pair[1].2 {
            LocalizationSource::Gnss => 0,
            LocalizationSource::Lidar => 1,
            LocalizationSource::DeadReckoning => 2,
        };
        trails[k].beam(
            p(a.x_m, a.y_m, city.ground_m(a.x_m, a.y_m) + 0.08),
            p(b.x_m, b.y_m, city.ground_m(b.x_m, b.y_m) + 0.08),
            0.35,
        );
    }
    let [gnss_trail, lidar_trail, dr_trail] = trails;
    push(
        scene,
        gnss_trail,
        linear([50, 120, 255]),
        0.5,
        0.0,
        [0.1, 0.25, 0.6],
    );
    push(
        scene,
        lidar_trail,
        linear([230, 50, 230]),
        0.5,
        0.0,
        [0.5, 0.1, 0.5],
    );
    push(
        scene,
        dr_trail,
        linear([255, 140, 0]),
        0.5,
        0.0,
        [0.6, 0.3, 0.0],
    );

    push_signals(scene, gnss_scene, frame);
    push_lidar(scene, city, frame);
    push_people(scene, city, frame);
    push_rain(scene, frame);
}

/// The tracked satellite signals at the rear antenna.
fn push_signals(scene: &mut RenderScene, gnss_scene: &GnssScene, frame: &FrameRecord) {
    let (mut los, mut nlos, mut diffracted) = (
        SolidMesh::default(),
        SolidMesh::default(),
        SolidMesh::default(),
    );
    if let Some(epoch) = &frame.epoch {
        let antenna = epoch.antenna_enu_m;
        let trace = crate::nav::receiver_config().trace;
        for o in &epoch.observations {
            let Some(primary) = o.primary() else { continue };
            if !primary.path.is_tracked() {
                continue;
            }
            let (sin_az, cos_az) = o.azimuth_rad.sin_cos();
            let (sin_el, cos_el) = o.elevation_rad.sin_cos();
            let u = Vec3::new(sin_az * cos_el, cos_az * cos_el, sin_el);
            let mut points = vec![antenna];
            let mesh = match primary.path {
                SignalPath::LineOfSight | SignalPath::LineOfSightMultipath => &mut los,
                SignalPath::Diffracted => &mut diffracted,
                _ => &mut nlos,
            };
            if !matches!(
                primary.path,
                SignalPath::LineOfSight | SignalPath::LineOfSightMultipath
            ) {
                let propagation = trace_satellite(gnss_scene, antenna, u, &trace);
                if let Some(path) = propagation
                    .paths
                    .iter()
                    .filter(|p| p.kind != PathKind::Direct)
                    .min_by(|a, b| {
                        (a.excess_delay_m - primary.excess_path_m)
                            .abs()
                            .total_cmp(&(b.excess_delay_m - primary.excess_path_m).abs())
                    })
                {
                    points.extend(path.points_m.iter().copied());
                }
            }
            let last = *points.last().expect("antenna");
            points.push(last + u * SKY_REACH_M);
            for pair in points.windows(2) {
                mesh.beam(pv(pair[0]), pv(pair[1]), 0.18);
            }
        }
    }
    push(
        scene,
        los,
        linear([60, 230, 120]),
        0.4,
        0.0,
        [0.15, 0.9, 0.35],
    );
    push(
        scene,
        nlos,
        linear([255, 60, 50]),
        0.4,
        0.0,
        [1.0, 0.15, 0.1],
    );
    push(
        scene,
        diffracted,
        linear([170, 90, 255]),
        0.4,
        0.0,
        [0.55, 0.25, 1.0],
    );
}

/// The LiDAR scan: a beam from the sensor to every other return, and the
/// returns themselves.
fn push_lidar(scene: &mut RenderScene, city: &PlateauCity, frame: &FrameRecord) {
    let (mut beams, mut hits) = (SolidMesh::default(), SolidMesh::default());
    let (x, y) = (frame.truth.x_m, frame.truth.y_m);
    let sensor = p(x, y, frame.ground_m + LIDAR_HEIGHT_M);
    for (i, &(px, py)) in frame.scan.iter().enumerate() {
        let hit = p(px, py, city.ground_m(px, py) + LIDAR_HEIGHT_M);
        if i % 2 == 0 {
            beams.beam(sensor, hit, 0.03);
        }
        hits.box_at(hit, Vec3::splat(0.1), Quat::IDENTITY);
    }
    push(
        scene,
        beams,
        linear([40, 200, 230]),
        0.4,
        0.0,
        [0.05, 0.35, 0.45],
    );
    push(
        scene,
        hits,
        linear([60, 240, 255]),
        0.4,
        0.0,
        [0.2, 0.9, 1.0],
    );
}

/// Pedestrians: a body and a head each, in a few clothing colours.
fn push_people(scene: &mut RenderScene, city: &PlateauCity, frame: &FrameRecord) {
    const CLOTHES: [[u8; 3]; 4] = [[40, 45, 60], [150, 40, 40], [220, 215, 200], [60, 90, 70]];
    let mut bodies: [SolidMesh; 4] = Default::default();
    let mut heads = SolidMesh::default();
    for (i, &(px, py)) in frame.people.iter().enumerate() {
        let z = city.ground_m(px, py);
        bodies[i % CLOTHES.len()].box_at(
            p(px, py, z + 0.75),
            Vec3::new(0.22, 0.75, 0.15),
            Quat::IDENTITY,
        );
        heads.box_at(p(px, py, z + 1.6), Vec3::splat(0.11), Quat::IDENTITY);
    }
    for (mesh, colour) in bodies.into_iter().zip(CLOTHES) {
        push(scene, mesh, linear(colour), 0.8, 0.0, [0.0; 3]);
    }
    push(scene, heads, linear([225, 185, 150]), 0.7, 0.0, [0.0; 3]);
}

/// Rain streaks around the robot, different every frame (deterministic).
fn push_rain(scene: &mut RenderScene, frame: &FrameRecord) {
    if !frame.rain {
        return;
    }
    let mut streaks = SolidMesh::default();
    let mut state = (frame.t * 1000.0) as u64 ^ 0x9E37_79B9_7F4A_7C15;
    let mut next = || {
        state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (state >> 11) as f64 / (1_u64 << 53) as f64
    };
    for _ in 0..900 {
        let (dx, dy) = (next() * 50.0 - 25.0, next() * 50.0 - 25.0);
        let z = frame.ground_m + next() * 14.0;
        let (ex, ey) = (frame.truth.x_m + dx, frame.truth.y_m + dy);
        streaks.beam(p(ex, ey, z), p(ex + 0.05, ey, z + 0.7), 0.025);
    }
    push(
        scene,
        streaks,
        linear([200, 210, 225]),
        0.2,
        0.0,
        [0.25, 0.27, 0.3],
    );
}

/// A chase camera that follows behind the robot, smoothed.
struct Chase {
    yaw: f64,
    focus: Vec3,
    initialized: bool,
}

impl Chase {
    fn update(&mut self, frame: &FrameRecord) -> CameraOrbit {
        let heading = frame.truth.yaw_rad;
        let yaw = (-heading.cos()).atan2(heading.sin());
        let focus = p(frame.truth.x_m, frame.truth.y_m, frame.ground_m + 3.0);
        if self.initialized {
            let mut delta = yaw - self.yaw;
            while delta > PI {
                delta -= TAU;
            }
            while delta < -PI {
                delta += TAU;
            }
            self.yaw += 0.25 * delta;
            self.focus += (focus - self.focus) * 0.7;
        } else {
            *self = Self {
                yaw,
                focus,
                initialized: true,
            };
        }
        CameraOrbit {
            yaw_rad: self.yaw,
            pitch_rad: 0.95_f64.clamp(0.2, FRAC_PI_2 - 0.1),
            distance_m: 42.0,
            focus: self.focus,
        }
    }
}

fn write_png(path: &Path, width: u32, height: u32, rgba: &[u8]) -> Result<()> {
    let file = std::fs::File::create(path)?;
    let mut encoder = png::Encoder::new(std::io::BufWriter::new(file), width, height);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.write_header()?.write_image_data(rgba)?;
    Ok(())
}

/// Renders every `--render-stride`-th recorded frame of the run.
pub(crate) fn render(
    dir: &Path,
    city: &PlateauCity,
    route: &[(f64, f64)],
    _base: Vec3,
    run: &RunReport,
) -> Result<()> {
    let stride: usize = std::env::args()
        .skip_while(|a| a != "--render-stride")
        .nth(1)
        .map_or(Ok(2), |v| v.parse())?;
    std::fs::create_dir_all(dir)?;
    let mut backend = WgpuRenderBackend::new()?;
    backend.set_environment(environment());
    backend.set_taa(TaaSettings {
        enabled: false,
        ..TaaSettings::default()
    });
    let gnss_scene = GnssScene {
        mesh: Some(city.mesh.clone()),
        ..GnssScene::open_sky()
    };
    let base_scene = static_city(city, route);
    let mut camera = Camera::new(WIDTH, HEIGHT, 0.9);
    camera.far_m = 900.0;
    let mut chase = Chase {
        yaw: 0.0,
        focus: Vec3::ZERO,
        initialized: false,
    };
    let started = std::time::Instant::now();
    let mut index = Vec::new();
    for (k, frame) in run.frames.iter().step_by(stride.max(1)).enumerate() {
        let orbit = chase.update(frame);
        let mut scene = base_scene.clone();
        frame_items(&mut scene, &gnss_scene, city, run, frame);
        let image = backend.render_scene_camera(
            &camera,
            &orbit.camera_transform(),
            &scene,
            [0.58, 0.70, 0.93, 1.0],
        )?;
        let name = format!("frame_{k:04}.png");
        write_png(&dir.join(&name), WIDTH, HEIGHT, &image.color.rgba8)?;
        let error =
            (frame.estimate.x_m - frame.truth.x_m).hypot(frame.estimate.y_m - frame.truth.y_m);
        let (los, nlos) = frame.epoch.as_ref().map_or((0, 0), |e| {
            (
                e.count(SignalPath::LineOfSight) + e.count(SignalPath::LineOfSightMultipath),
                e.count(SignalPath::NonLineOfSight) + e.count(SignalPath::Diffracted),
            )
        });
        index.push(serde_json::json!({
            "file": name,
            "t": frame.t,
            "source": format!("{:?}", frame.source),
            "solution": match frame.label {
                Some(SolutionLabel::Fixed) => "RTK fixed",
                Some(SolutionLabel::Float) => "RTK float",
                Some(SolutionLabel::Single) => "single point",
                _ => "no solution",
            },
            "error_m": error,
            "los": los,
            "nlos": nlos,
        }));
        if k % 50 == 0 {
            println!(
                "    rendered frame {k} (t = {:.0} s, {:.0} s elapsed)",
                frame.t,
                started.elapsed().as_secs_f64()
            );
        }
    }
    std::fs::write(
        dir.join("frames.json"),
        serde_json::to_string(&serde_json::json!({"area": city.name, "frames": index}))?,
    )?;
    println!("  wrote {} frames to {}", index.len(), dir.display());
    Ok(())
}
