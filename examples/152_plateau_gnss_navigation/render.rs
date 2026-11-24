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
use crate::streets::{Streets, ASPHALT_TILE_M, ROAD_LIFT_M};
use rne_gnss::{trace_satellite, GnssScene, PathKind, SignalPath, SurfaceMaterial};
use rne_math::{Quat, Vec3};
use rne_nav::Pose2d;
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
pub(crate) fn p(e: f64, n: f64, u: f64) -> Vec3 {
    Vec3::new(e, u, -n)
}

fn pv(v: Vec3) -> Vec3 {
    p(v.x, v.y, v.z)
}

#[derive(Default)]
pub(crate) struct SolidMesh {
    pub(crate) positions: Vec<[f32; 3]>,
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

    /// A triangle with texture coordinates.
    pub(crate) fn textured(&mut self, corners: [Vec3; 3], normal: Vec3, uv: [[f32; 2]; 3]) {
        self.triangle(corners, normal);
        let n = self.texcoords.len();
        self.texcoords[n - 3..].copy_from_slice(&uv);
    }

    pub(crate) fn face(&mut self, points: [Vec3; 4], normal: Vec3) {
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

    pub(crate) fn item(
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

pub(crate) fn linear(rgb: [u8; 3]) -> [f32; 4] {
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
                (0.34, 0.32, 0.29)
            };
            rgba.extend([r, g, b, 1.0]);
        }
    }
    EnvironmentLighting {
        map: Some(Arc::new(
            EnvironmentMap::from_rgba32f(width, height, rgba).expect("environment"),
        )),
        intensity: 0.9,
        diffuse_strength: 1.1,
        specular_strength: 0.3,
        rotation_rad: 0.0,
    }
}

/// Buildings, furniture, terrain and the planned route.
fn static_city(city: &PlateauCity, route: &[(f64, f64)]) -> (RenderScene, Streets) {
    let mut scene = RenderScene::new();
    let (mut glass, mut concrete, mut roofs, mut furniture) = (
        SolidMesh::default(),
        SolidMesh::default(),
        SolidMesh::default(),
        SolidMesh::default(),
    );
    for tri in city.mesh.triangles() {
        if city
            .buildings
            .get(tri.object as usize)
            .is_some_and(|b| b.textured)
        {
            continue; // drawn with its photos below
        }
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
    scene.items.extend(facade_items(city));
    push(
        &mut scene,
        furniture,
        linear([90, 92, 96]),
        0.6,
        0.3,
        [0.0; 3],
    );

    // The streets and the ground around the route.
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
    let streets = Streets::new(city, [x0 - margin, y0 - margin, x1 + margin, y1 + margin]);
    let mut roads = SolidMesh::default();
    for corners in city.roads.iter() {
        drape(city, *corners, &mut roads);
    }
    scene
        .items
        .extend(crate::streets::street_items(city, &streets, roads));

    // The planned route as a faint line on the ground.
    let mut line = SolidMesh::default();
    for pair in route.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        line.beam(
            p(
                a.0,
                a.1,
                city.ground_m(a.0, a.1) + streets.surface_m(a.0, a.1) + 0.03,
            ),
            p(
                b.0,
                b.1,
                city.ground_m(b.0, b.1) + streets.surface_m(b.0, b.1) + 0.03,
            ),
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
    (scene, streets)
}

/// Draws a flat ENU road triangle on the terrain, split until its edges are
/// at most 4 m so it follows the slopes, textured as asphalt.
fn drape(city: &PlateauCity, corners: [Vec3; 3], mesh: &mut SolidMesh) {
    let [a, b, c] = corners;
    let longest = (a - b).length().max((b - c).length()).max((c - a).length());
    if longest > 4.0 {
        let (ab, bc, ca) = ((a + b) * 0.5, (b + c) * 0.5, (c + a) * 0.5);
        for part in [[a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]] {
            drape(city, part, mesh);
        }
        return;
    }
    let lifted = corners.map(|v| p(v.x, v.y, city.ground_m(v.x, v.y) + ROAD_LIFT_M));
    crate::streets::flat(mesh, lifted, ASPHALT_TILE_M);
}

/// Atlas size, pixels (within every GPU's texture limit).
const ATLAS_PX: u32 = 4096;
/// Edge pixels repeated around each photo, so filtering does not bleed.
const ATLAS_PAD_PX: u32 = 2;

/// The photo-textured buildings: their photos packed into atlases (one
/// draw item each) and their untextured surfaces in plain concrete.
fn facade_items(city: &PlateauCity) -> Vec<RenderSceneItem> {
    let facades = &city.facades;
    if facades.images.is_empty() {
        return Vec::new();
    }
    // Cells as large as the photo count allows: 256 px for up to 2048
    // photos over eight atlases, smaller beyond.
    let total = facades.images.len() as u32;
    let mut cell = 256;
    while cell > 32 && total.div_ceil((ATLAS_PX / cell).pow(2)) > 8 {
        cell /= 2;
    }
    let per_row = ATLAS_PX / cell;
    let per_atlas = per_row * per_row;
    let inner = cell - 2 * ATLAS_PAD_PX;
    let atlases = total.div_ceil(per_atlas) as usize;
    let mut pixels = vec![vec![0_u8; (ATLAS_PX * ATLAS_PX * 4) as usize]; atlases];
    let mut missing = 0;
    for (k, path) in facades.images.iter().enumerate() {
        let k = k as u32;
        let (atlas, slot) = ((k / per_atlas) as usize, k % per_atlas);
        let (cx, cy) = ((slot % per_row) * cell, (slot / per_row) * cell);
        let Ok(photo) = image::open(path) else {
            missing += 1;
            continue;
        };
        let photo = image::imageops::resize(
            &photo.to_rgba8(),
            inner,
            inner,
            image::imageops::FilterType::Triangle,
        );
        for y in 0..cell {
            for x in 0..cell {
                let sx = x.saturating_sub(ATLAS_PAD_PX).min(inner - 1);
                let sy = y.saturating_sub(ATLAS_PAD_PX).min(inner - 1);
                let o = (((cy + y) * ATLAS_PX + cx + x) * 4) as usize;
                pixels[atlas][o..o + 4].copy_from_slice(&photo.get_pixel(sx, sy).0);
            }
        }
    }
    if missing > 0 {
        println!("  {missing} facade photos could not be read");
    }
    let mut meshes: Vec<SolidMesh> = (0..atlases).map(|_| SolidMesh::default()).collect();
    let mut plain = SolidMesh::default();
    let size = ATLAS_PX as f32;
    for facade in &facades.triangles {
        let corners = facade.vertices.map(pv);
        let normal = (corners[1] - corners[0])
            .cross(corners[2] - corners[0])
            .normalize_or_zero();
        match facade.texture {
            Some((image, uv)) => {
                let (atlas, slot) = ((image / per_atlas) as usize, image % per_atlas);
                let (cx, cy) = ((slot % per_row) * cell, (slot / per_row) * cell);
                // CityGML's v runs up from the image's lower edge.
                let uv = uv.map(|[u, v]| {
                    [
                        (cx as f32 + ATLAS_PAD_PX as f32 + u.clamp(0.0, 1.0) * inner as f32) / size,
                        (cy as f32
                            + ATLAS_PAD_PX as f32
                            + (1.0 - v.clamp(0.0, 1.0)) * inner as f32)
                            / size,
                    ]
                });
                meshes[atlas].textured(corners, normal, uv);
            }
            None => plain.triangle(corners, normal),
        }
    }
    let mut items = Vec::new();
    for (mesh, rgba) in meshes.into_iter().zip(pixels) {
        if mesh.positions.is_empty() {
            continue;
        }
        // Aerial photos are dark beside the renderer's untextured greys.
        let mut item = mesh.item([1.25, 1.25, 1.25, 1.0], 0.8, 0.0, [0.0; 3]);
        item.base_color_texture = Some(Arc::new(rne_render::ImageFrame::from_rgba8(
            ATLAS_PX, ATLAS_PX, rgba,
        )));
        items.push(item);
    }
    if !plain.positions.is_empty() {
        items.push(plain.item(linear([190, 186, 178]), 0.85, 0.0, [0.0; 3]));
    }
    println!(
        "  {} facade photos in {atlases} atlases ({cell} px each), {} textured triangles",
        facades.images.len(),
        facades.triangles.len()
    );
    items
}

/// The robot, its trail, the satellite signals and the LiDAR returns.
fn frame_items(
    scene: &mut RenderScene,
    gnss_scene: &GnssScene,
    city: &PlateauCity,
    streets: &Streets,
    run: &RunReport,
    frame: &FrameRecord,
) {
    let (x, y, yaw) = (frame.truth.x_m, frame.truth.y_m, frame.truth.yaw_rad);
    let z = frame.ground_m + streets.surface_m(x, y);
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
            p(
                a.x_m,
                a.y_m,
                city.ground_m(a.x_m, a.y_m) + streets.surface_m(a.x_m, a.y_m) + 0.07,
            ),
            p(
                b.x_m,
                b.y_m,
                city.ground_m(b.x_m, b.y_m) + streets.surface_m(b.x_m, b.y_m) + 0.07,
            ),
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

    push_ghost(scene, city, streets, run, frame);
    push_signals(scene, gnss_scene, frame);
    if frame.scan3d.is_empty() {
        push_lidar(scene, city, frame);
    } else {
        push_lidar3d(scene, frame);
    }
    push_people(scene, city, streets, frame);
    push_vehicles(scene, frame);
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

/// The LiDAR scan: a beam from the sensor to every fourth return, and the
/// returns themselves.
fn push_lidar(scene: &mut RenderScene, city: &PlateauCity, frame: &FrameRecord) {
    let (mut beams, mut hits) = (SolidMesh::default(), SolidMesh::default());
    let (x, y) = (frame.truth.x_m, frame.truth.y_m);
    let sensor = p(x, y, frame.ground_m + LIDAR_HEIGHT_M);
    for (i, &(px, py)) in frame.scan.iter().enumerate() {
        let hit = p(px, py, city.ground_m(px, py) + LIDAR_HEIGHT_M);
        if i % 4 == 0 {
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

/// A 3D LiDAR scan: every other return as a small cube, coloured from cyan
/// near the ground to white up the facades.
fn push_lidar3d(scene: &mut RenderScene, frame: &FrameRecord) {
    let mut bands: [SolidMesh; 3] = Default::default();
    for (i, point) in frame.scan3d.iter().enumerate() {
        if i % 2 != 0 {
            continue;
        }
        let height = point.z - frame.ground_m;
        let band = if height < 1.5 {
            0
        } else if height < 6.0 {
            1
        } else {
            2
        };
        bands[band].box_at(pv(*point), Vec3::splat(0.07), Quat::IDENTITY);
    }
    for (mesh, (colour, glow)) in bands.into_iter().zip([
        ([40, 230, 255], [0.15, 0.8, 1.0]),
        ([120, 240, 255], [0.4, 0.85, 1.0]),
        ([220, 250, 255], [0.75, 0.9, 1.0]),
    ]) {
        push(scene, mesh, linear(colour), 0.4, 0.0, glow);
    }
}

/// The localizer without GNSS (`--ghost`): a red robot where it thinks the
/// robot is, under a tall red pole so it reads from the chase camera, its
/// track so far in red, and a red line to the robot.
fn push_ghost(
    scene: &mut RenderScene,
    city: &PlateauCity,
    streets: &Streets,
    run: &RunReport,
    frame: &FrameRecord,
) {
    let Some(ghost) = frame.ghost else { return };
    let at = |q: Pose2d, lift: f64| {
        p(
            q.x_m,
            q.y_m,
            city.ground_m(q.x_m, q.y_m) + streets.surface_m(q.x_m, q.y_m) + lift,
        )
    };
    let mut body = SolidMesh::default();
    body.box_at(
        at(ghost, 0.3),
        Vec3::new(0.45, 0.25, 0.3),
        Quat::from_rotation_y(ghost.yaw_rad),
    );
    body.beam(at(ghost, 0.6), at(ghost, 3.0), 0.12);
    body.box_at(at(ghost, 3.2), Vec3::new(0.3, 0.3, 0.3), Quat::IDENTITY);
    let mut trail = SolidMesh::default();
    let poses: Vec<Pose2d> = run
        .frames
        .iter()
        .take_while(|f| f.t <= frame.t + 1.0e-9)
        .filter_map(|f| f.ghost)
        .collect();
    for pair in poses.windows(2) {
        trail.beam(at(pair[0], 0.1), at(pair[1], 0.1), 0.25);
    }
    let mut gap = SolidMesh::default();
    gap.beam(at(frame.truth, 0.8), at(ghost, 0.8), 0.08);
    push(
        scene,
        body,
        linear([235, 30, 30]),
        0.4,
        0.0,
        [0.8, 0.05, 0.05],
    );
    push(
        scene,
        trail,
        linear([200, 40, 40]),
        0.5,
        0.0,
        [0.45, 0.05, 0.05],
    );
    push(
        scene,
        gap,
        linear([255, 60, 40]),
        0.5,
        0.0,
        [0.7, 0.1, 0.05],
    );
}

/// Pedestrians: a body and a head each, in a few clothing colours.
fn push_people(
    scene: &mut RenderScene,
    city: &PlateauCity,
    streets: &Streets,
    frame: &FrameRecord,
) {
    const CLOTHES: [[u8; 3]; 4] = [[40, 45, 60], [150, 40, 40], [220, 215, 200], [60, 90, 70]];
    let mut bodies: [SolidMesh; 4] = Default::default();
    let mut heads = SolidMesh::default();
    for (i, &(px, py)) in frame.movers.people.iter().enumerate() {
        let z = city.ground_m(px, py) + streets.surface_m(px, py);
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

/// Vehicles: a body and a darker glasshouse.
fn push_vehicles(scene: &mut RenderScene, frame: &FrameRecord) {
    let (mut bodies, mut windows) = (SolidMesh::default(), SolidMesh::default());
    for v in &frame.movers.vehicles {
        let rotation = Quat::from_rotation_y(v.yaw_rad);
        let (hl, hw, hh) = v.half;
        bodies.box_at(
            p(v.center.0, v.center.1, v.ground_m + 0.2 + 0.6 * hh),
            Vec3::new(hl, 0.6 * hh, hw),
            rotation,
        );
        windows.box_at(
            p(v.center.0, v.center.1, v.ground_m + 0.2 + 1.5 * hh),
            Vec3::new(0.9 * hl, 0.35 * hh, 0.95 * hw),
            rotation,
        );
    }
    push(scene, bodies, linear([200, 200, 205]), 0.3, 0.6, [0.0; 3]);
    push(scene, windows, linear([40, 50, 60]), 0.1, 0.2, [0.0; 3]);
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
    for _ in 0..400 {
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

/// What the HUD's top-down map needs when there is a ghost: the route and
/// the road triangles around it and around the ghost's track, rounded to
/// decimetres.
fn minimap(city: &PlateauCity, route: &[(f64, f64)], run: &RunReport) -> serde_json::Value {
    let ghosts: Vec<(f64, f64)> = run
        .frames
        .iter()
        .filter_map(|f| f.ghost.map(|g| (g.x_m, g.y_m)))
        .collect();
    if ghosts.is_empty() {
        return serde_json::Value::Null;
    }
    let (mut lo, mut hi) = (
        (f64::INFINITY, f64::INFINITY),
        (f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    for &(x, y) in route.iter().chain(&ghosts) {
        lo = (lo.0.min(x), lo.1.min(y));
        hi = (hi.0.max(x), hi.1.max(y));
    }
    let margin = 30.0;
    let inside = |v: &Vec3| {
        v.x > lo.0 - margin && v.x < hi.0 + margin && v.y > lo.1 - margin && v.y < hi.1 + margin
    };
    let dm = |v: f64| (v * 10.0).round() / 10.0;
    let roads: Vec<[f64; 6]> = city
        .roads
        .iter()
        .filter(|t| t.iter().any(inside))
        .map(|[a, b, c]| [dm(a.x), dm(a.y), dm(b.x), dm(b.y), dm(c.x), dm(c.y)])
        .collect();
    serde_json::json!({
        "bounds": [lo.0 - margin, lo.1 - margin, hi.0 + margin, hi.1 + margin],
        "route": route.iter().map(|&(x, y)| [dm(x), dm(y)]).collect::<Vec<_>>(),
        "roads": roads,
    })
}

/// Renders the city alone at eight points along the route, alternately from
/// the chase camera's distance and from 140 m (`--render-preview`), without
/// running the robot.
pub(crate) fn preview(dir: &Path, city: &PlateauCity, route: &[(f64, f64)]) -> Result<()> {
    std::fs::create_dir_all(dir)?;
    let mut backend = WgpuRenderBackend::new()?;
    backend.set_environment(environment());
    let (scene, _) = static_city(city, route);
    let mut camera = Camera::new(WIDTH, HEIGHT, 0.9);
    camera.far_m = 900.0;
    for k in 0..8 {
        let i = (k * (route.len() - 2)) / 7;
        let (a, b) = (route[i], route[i + 1]);
        let heading = (b.1 - a.1).atan2(b.0 - a.0);
        let orbit = CameraOrbit {
            yaw_rad: (-heading.cos()).atan2(heading.sin()),
            pitch_rad: if k % 2 == 0 { 0.95 } else { 0.55 },
            distance_m: if k % 2 == 0 { 42.0 } else { 140.0 },
            focus: p(a.0, a.1, city.ground_m(a.0, a.1) + 3.0),
        };
        let image = backend.render_scene_camera(
            &camera,
            &orbit.camera_transform(),
            &scene,
            [0.58, 0.70, 0.93, 1.0],
        )?;
        write_png(
            &dir.join(format!("preview_{k}.png")),
            WIDTH,
            HEIGHT,
            &image.color.rgba8,
        )?;
    }
    println!("  wrote 8 previews to {}", dir.display());
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
    let (base_scene, streets) = static_city(city, route);
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
        frame_items(&mut scene, &gnss_scene, city, &streets, run, frame);
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
            "ghost_error_m": frame.ghost.map(|g| {
                (g.x_m - frame.truth.x_m).hypot(g.y_m - frame.truth.y_m)
            }),
            "truth": [frame.truth.x_m, frame.truth.y_m],
            "ghost": frame.ghost.map(|g| [g.x_m, g.y_m]),
            "los": los,
            "nlos": nlos,
            "gnss": frame.status.gnss,
            "lidar": frame.status.lidar,
            "event": frame.status.event,
            "weather": match (
                !frame.movers.vehicles.is_empty(),
                frame.movers.people.is_empty(),
                frame.rain,
            ) {
                (true, _, false) => "rush hour",
                (true, _, true) => "rush hour + rain",
                (false, true, false) => "",
                (false, false, false) => "crowd",
                (false, true, true) => "rain",
                (false, false, true) => "crowd + rain",
            },
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
        serde_json::to_string(&serde_json::json!({
            "area": city.name,
            "constellation": run.constellation,
            "map": minimap(city, route, run),
            "frames": index,
        }))?,
    )?;
    println!("  wrote {} frames to {}", index.len(), dir.display());
    Ok(())
}
