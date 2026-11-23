//! Closed loop: an Ackermann car steers on its *visual* position in a 3DGS street.
//!
//! The car (RNE [`ackermann_kinematics`]) follows the KITTI 0018 SLAM path with pure pursuit,
//! but on its estimated pose, not the true one. Between fixes the estimate is dead-reckoned
//! from wheel speed and steering with a scale error and a yaw-rate bias. Every `--loc-every`
//! seconds the front camera is rendered from the splat at the *true* pose
//! (`gsplat_render_server`, visloc-rs) and localized against a prebuilt visloc-rs map
//! (`basalt_euroc_localize_demo`); a fix replaces the estimate. The car starts at a known pose.
//!
//! ```text
//! --path CSV --render-server BIN --ply PLY --localizer BIN --map DIR --out DIR
//! [--speed 7] [--loc-every 0.5] [--odom-scale 1.02] [--gyro-bias 0.003] [--no-vision]
//! [--manifest SPLAT.toml]   render the camera with RNE's own renderer instead of the server
//! [--segments LIST]         RNE renderer with one splat and map per stretch of the route:
//!                           lines `start_frame, manifest, map_dir`. The splat follows the
//!                           true pose (it is the world); the map follows the estimate.
//! [--loc-args "FLAGS"]      extra flags for the localizer, e.g. "--ratio 0.9 --min-inliers 12"
//! [--render-poses TUM --manifest SPLAT.toml --out DIR]
//!                           only render RNE camera images at SLAM camera poses into DIR/rgb
//! ```
//!
//! Frames: the SLAM world is z-up (x, y, z); RNE is y-up: rne = (x, z, -y), yaw is shared.

use kitti_splat_drive::{parse_path, parse_segments, rne_view_from_slam, segment_for, Segment};
use png::{BitDepth, ColorType, Encoder};
use rne_core::SimDuration;
use rne_ecs::{spawn_named, World};
use rne_math::{Quat, Seconds, Vec3};
use rne_physics::RigidBody;
use rne_robot::{ackermann_kinematics, pure_pursuit_steering, AckermannDrive};
use rne_world::Transform3;
use std::fs::{self, File};
use std::io::{BufRead, BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};

const SIM_HZ: f64 = 50.0;
const WHEELBASE_M: f64 = 2.71;
const LOOKAHEAD_M: f64 = 6.0;
const W: usize = 1226;
const H: usize = 370;
const CAMERA: &str = "PINHOLE 1226 370 707.0912 707.0912 613 185";
/// image_02 in the OXTS body frame (x forward, y left, z up).
const MOUNT: [f64; 3] = [1.144, -0.246, 0.726];

type M3 = [[f64; 3]; 3];

fn main() {
    let path_csv = PathBuf::from(arg("--path").expect("--path"));
    let out = PathBuf::from(arg("--out").unwrap_or_else(|| "visloc_loop".into()));
    let speed: f64 = arg("--speed").map_or(7.0, |v| v.parse().unwrap());
    let loc_every: f64 = arg("--loc-every").map_or(0.5, |v| v.parse().unwrap());
    let odom_scale: f64 = arg("--odom-scale").map_or(1.02, |v| v.parse().unwrap());
    let gyro_bias: f64 = arg("--gyro-bias").map_or(0.003, |v| v.parse().unwrap());
    let vision = !std::env::args().any(|a| a == "--no-vision");
    if let Some(poses) = arg("--render-poses") {
        render_poses(&poses, &arg("--manifest").expect("--manifest"), &out);
        return;
    }
    fs::create_dir_all(out.join("frames")).expect("out dir");

    let path = parse_path(&fs::read_to_string(&path_csv).expect("path csv")).expect("path");
    let (waypoints, start_yaw) = (path.points.clone(), path.start_yaw);
    let segments = segments_with_maps();
    let mut camera = vision.then(|| make_camera(segments.as_deref(), path.frames[0]));
    let localizer = arg("--localizer").unwrap_or_default();
    let fixed_map = arg("--map").unwrap_or_default();
    // Long enough to finish the path at the target speed, with margin.
    let path_length: f64 = waypoints.windows(2).map(|w| (w[1] - w[0]).length()).sum();
    let max_time = 30.0 + 1.5 * path_length / speed;
    let mut splat_segment = segments
        .as_ref()
        .map_or(0, |s| segment_for(s, path.frames[0]));
    let mut true_index = 0usize;

    let mut world = World::new();
    let car = spawn_named(&mut world, "car");
    world.entity_mut(car).insert((
        AckermannDrive {
            wheelbase_m: WHEELBASE_M,
            max_speed_m_s: 15.0,
            ..AckermannDrive::default()
        },
        Transform3 {
            translation: waypoints[0],
            rotation: Quat::from_rotation_y(start_yaw),
            scale: Vec3::ONE,
        },
        RigidBody::default(),
    ));
    // The car starts at a known pose (its parking spot); after that only the dead-reckoned
    // estimate and the visual fixes steer it.
    let mut estimate: Option<(Vec3, f64)> = Some((waypoints[0], start_yaw));

    let dt = 1.0 / SIM_HZ;
    let sim_dt = SimDuration::from_seconds(Seconds::new(dt));
    let mut log = BufWriter::new(File::create(out.join("loop.csv")).expect("loop.csv"));
    writeln!(
        log,
        "t_s,true_x,true_z,true_yaw,est_x,est_z,est_yaw,fix,est_err_m,cross_track_m,speed"
    )
    .unwrap();
    let (mut t, mut next_loc, mut index, mut frame) = (0.0, 0.0, 0usize, 0usize);
    let (mut fixes, mut tries) = (0usize, 0usize);
    let mut last_fix_t = 0.0;
    loop {
        let truth = *world.get::<Transform3>(car).expect("pose");
        let true_yaw = yaw_of(truth.rotation);

        let mut fix = false;
        if vision && t + 1e-9 >= next_loc {
            next_loc += loc_every;
            tries += 1;
            let map = match &segments {
                Some(segments) => {
                    // The world the camera sees: the splat of the true position's stretch.
                    true_index = advance(&waypoints, true_index, truth.translation);
                    let wanted = segment_for(segments, path.frames[true_index]);
                    if wanted != splat_segment {
                        splat_segment = wanted;
                        if let Some(Sensor::Rne(rne)) = camera.as_mut() {
                            rne.load(&segments[wanted].manifest.to_string_lossy());
                        }
                    }
                    // The map the car looks itself up in: chosen by its own estimate.
                    let mine = segment_for(segments, path.frames[index]);
                    segments[mine]
                        .map
                        .clone()
                        .unwrap()
                        .to_string_lossy()
                        .into_owned()
                }
                None => fixed_map.clone(),
            };
            let (r_wc, c) = camera_in_slam(truth.translation, true_yaw);
            let rgb = match camera.as_mut().unwrap() {
                Sensor::Server(server) => server.render(&r_wc, &c),
                Sensor::Rne(rne) => rne.render(truth.translation, truth.rotation),
            };
            let png_path = out.join("frames").join(format!("{frame:04}.png"));
            write_rgb_png(&png_path, &rgb);
            frame += 1;
            if let Some((r, c)) = localize(&localizer, &map, &png_path, &out) {
                let (p, yaw) = car_from_camera(&r, &c);
                // Gate: a fix far from the estimate is taken as a wrong match. The gate grows
                // with the time since the last fix, as dead reckoning drifts (3 m + 0.5 m/s).
                let gate = 3.0 + 0.5 * (t - last_fix_t);
                let (e, _) = estimate.unwrap();
                if ((p.x - e.x).powi(2) + (p.z - e.z).powi(2)).sqrt() < gate {
                    // Keep the road height of the truth: the map does not drive the car's y.
                    estimate = Some((Vec3::new(p.x, truth.translation.y, p.z), yaw));
                    fix = true;
                    fixes += 1;
                    last_fix_t = t;
                }
            }
        }

        // Path progress and pure pursuit on the estimated pose.
        let Some((est_p, est_yaw)) = estimate else {
            // Not localized yet: wait in place.
            ackermann_kinematics(&mut world, sim_dt);
            t += dt;
            if t > 30.0 {
                panic!("no visual fix in 30 s");
            }
            continue;
        };
        index = advance(&waypoints, index, est_p);
        let (drive_now, done) = steer(&mut world, car, &waypoints, index, (est_p, est_yaw), speed);

        let err = ((est_p.x - truth.translation.x).powi(2)
            + (est_p.z - truth.translation.z).powi(2))
        .sqrt();
        writeln!(
            log,
            "{t:.2},{:.3},{:.3},{true_yaw:.4},{:.3},{:.3},{est_yaw:.4},{},{err:.3},{:.3},{:.2}",
            truth.translation.x,
            truth.translation.z,
            est_p.x,
            est_p.z,
            fix as u8,
            cross_track(&waypoints, truth.translation),
            drive_now.speed_m_s
        )
        .unwrap();
        if done || t > max_time {
            break;
        }

        ackermann_kinematics(&mut world, sim_dt);
        if let Some(mut pose) = world.get_mut::<Transform3>(car) {
            pose.translation.y = height_at(&waypoints, pose.translation);
        }
        // Dead reckoning from wheel speed and steering, with a scale error and a gyro bias.
        let drive = world.get::<AckermannDrive>(car).expect("drive").clone();
        let v = drive.speed_m_s * odom_scale;
        let yaw_rate = drive.speed_m_s / WHEELBASE_M * drive.steering_rad.tan() + gyro_bias;
        let (p, yaw) = estimate.unwrap();
        let mid = yaw + 0.5 * yaw_rate * dt;
        estimate = Some((
            Vec3::new(p.x + v * mid.cos() * dt, p.y, p.z - v * mid.sin() * dt),
            yaw + yaw_rate * dt,
        ));
        t += dt;
    }
    println!(
        "drove {t:.1} s; visual fixes {fixes}/{tries}; log {}",
        out.join("loop.csv").display()
    );
}

/// Pure pursuit on the estimated pose; returns the drive state and whether the path is done.
fn steer(
    world: &mut World,
    car: rne_ecs::Entity,
    waypoints: &[Vec3],
    index: usize,
    (est_p, est_yaw): (Vec3, f64),
    speed: f64,
) -> (AckermannDrive, bool) {
    let mut target = index;
    while target + 1 < waypoints.len() && (waypoints[target] - est_p).length() < LOOKAHEAD_M {
        target += 1;
    }
    let done = index + 2 >= waypoints.len()
        || (target + 1 == waypoints.len() && (waypoints[target] - est_p).length() < 2.0);
    let est_pose = Transform3 {
        translation: est_p,
        rotation: Quat::from_rotation_y(est_yaw),
        scale: Vec3::ONE,
    };
    let mut drive = world.get_mut::<AckermannDrive>(car).expect("drive");
    let steering = pure_pursuit_steering(&est_pose, waypoints[target], WHEELBASE_M, LOOKAHEAD_M);
    drive.target_steering_rad = steering.clamp(-drive.max_steering_rad, drive.max_steering_rad);
    drive.target_speed_m_s = if done { 0.0 } else { speed };
    (drive.clone(), done)
}

fn segments_with_maps() -> Option<Vec<Segment>> {
    arg("--segments").map(|list| {
        let list = PathBuf::from(list);
        let base = list.parent().unwrap_or(Path::new(".")).to_path_buf();
        let segments =
            parse_segments(&fs::read_to_string(&list).expect("segments"), &base).expect("segments");
        assert!(
            segments.iter().all(|s| s.map.is_some()),
            "--segments needs a map per segment"
        );
        segments
    })
}

fn make_camera(segments: Option<&[Segment]>, first_frame: u32) -> Sensor {
    if let Some(segments) = segments {
        let first = segment_for(segments, first_frame);
        Sensor::Rne(Box::new(RneCamera::new(
            &segments[first].manifest.to_string_lossy(),
        )))
    } else if let Some(manifest) = arg("--manifest") {
        Sensor::Rne(Box::new(RneCamera::new(&manifest)))
    } else {
        Sensor::Server(RenderServer::start(
            &arg("--render-server").expect("--render-server or --manifest"),
            &arg("--ply").expect("--ply"),
        ))
    }
}

/// The true front camera in the SLAM world: world <- camera rotation and centre.
fn camera_in_slam(rne: Vec3, yaw: f64) -> (M3, [f64; 3]) {
    let base = [rne.x, -rne.z, rne.y];
    let rz = rot_z(yaw);
    let m = mul_v(&rz, &MOUNT);
    let c = [base[0] + m[0], base[1] + m[1], base[2] + m[2]];
    (mul(&rz, &base_from_optical()), c)
}

/// Car pose (RNE position at the base, yaw) from a localized camera (world <- camera, centre).
fn car_from_camera(r_wc: &M3, c: &[f64; 3]) -> (Vec3, f64) {
    let r_base = mul(r_wc, &transpose(&base_from_optical()));
    let yaw = r_base[1][0].atan2(r_base[0][0]);
    let rz = rot_z(yaw);
    let m = mul_v(&rz, &MOUNT);
    let base = [c[0] - m[0], c[1] - m[1], c[2] - m[2]];
    (Vec3::new(base[0], base[2], -base[1]), yaw)
}

/// Columns: the optical x (right), y (down), z (forward) axes in the base frame.
fn base_from_optical() -> M3 {
    [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
}

enum Sensor {
    /// visloc-rs `gsplat_render_server` (the renderer the map was built with).
    Server(RenderServer),
    /// RNE's own hybrid renderer (wgpu-3dgs-viewer).
    Rne(Box<RneCamera>),
}

struct RneCamera {
    backend: rne_render_wgpu::WgpuRenderBackend,
    background: rne_render_3dgs::GaussianSplatBackground,
    environment: rne_render::GaussianSplatEnvironment,
    camera: rne_render::Camera,
}

impl RneCamera {
    fn new(manifest: &str) -> Self {
        let backend = rne_render_wgpu::WgpuRenderBackend::new().expect("wgpu backend");
        let (environment, background) = Self::load_splat(&backend, manifest);
        // Same pinhole as CAMERA: fy 707.0912, principal point at the centre.
        let camera =
            rne_render::Camera::new(W as u32, H as u32, 2.0 * (0.5 * H as f64 / 707.0912).atan());
        Self {
            backend,
            background,
            environment,
            camera,
        }
    }

    fn load_splat(
        backend: &rne_render_wgpu::WgpuRenderBackend,
        manifest: &str,
    ) -> (
        rne_render::GaussianSplatEnvironment,
        rne_render_3dgs::GaussianSplatBackground,
    ) {
        let environment = rne_render::validate_gaussian_splat_manifest(Path::new(manifest))
            .expect("splat manifest");
        let background =
            rne_render_3dgs::load_gaussian_splat_background(backend.device(), &environment)
                .expect("splat background");
        println!("splat: {}", environment.ply_path.display());
        (environment, background)
    }

    /// Switches to another splat.
    fn load(&mut self, manifest: &str) {
        (self.environment, self.background) = Self::load_splat(&self.backend, manifest);
    }

    /// RGB8 front-camera image for the car at `position` / `rotation` (RNE frame).
    fn render(&mut self, position: Vec3, rotation: Quat) -> Vec<u8> {
        // Car frame: +X forward, +Y up, +Z right; RNE cameras look along local -Z.
        let mount = Vec3::new(MOUNT[0], MOUNT[2], -MOUNT[1]);
        let view = rne_math::Transform3::from_translation_rotation(
            position + rotation * mount,
            (rotation * Quat::from_rotation_y(-std::f64::consts::FRAC_PI_2)).normalize(),
        );
        self.render_view(&view)
    }

    /// RGB8 image of an RNE camera pose (looking along its local -Z, +Y up).
    fn render_view(&mut self, view: &rne_math::Transform3) -> Vec<u8> {
        let scene = rne_render::HybridRenderScene::new(
            self.environment.clone(),
            rne_render::RenderScene::new(),
        );
        let image = rne_render_3dgs::render_hybrid_scene_camera(
            &mut self.backend,
            &mut self.background,
            &self.camera,
            view,
            &scene,
            [0.0, 0.0, 0.0, 1.0],
        )
        .expect("rne render");
        image
            .color
            .rgba8
            .chunks_exact(4)
            .flat_map(|p| [p[0], p[1], p[2]])
            .collect()
    }
}

struct RenderServer {
    _child: Child,
    stdin: ChildStdin,
    stdout: ChildStdout,
}

impl RenderServer {
    fn start(bin: &str, ply: &str) -> Self {
        let mut child = Command::new(bin)
            .args(["--ply", ply, "--camera", CAMERA, "--depth"])
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()
            .expect("start gsplat_render_server");
        let stdin = child.stdin.take().unwrap();
        let stdout = child.stdout.take().unwrap();
        Self {
            _child: child,
            stdin,
            stdout,
        }
    }

    /// RGB8 image of the camera (world <- camera rotation, centre).
    fn render(&mut self, r_wc: &M3, c: &[f64; 3]) -> Vec<u8> {
        let r_cw = transpose(r_wc);
        let t = mul_v(&r_cw, c);
        let mut line = String::new();
        for row in &r_cw {
            for v in row {
                line.push_str(&format!("{v:.9} "));
            }
        }
        line.push_str(&format!("{:.9} {:.9} {:.9}\n", -t[0], -t[1], -t[2]));
        self.stdin.write_all(line.as_bytes()).unwrap();
        self.stdin.flush().unwrap();
        let mut buf = vec![0u8; W * H * 7];
        self.stdout.read_exact(&mut buf).expect("render");
        buf.truncate(W * H * 3);
        buf
    }
}

/// One image through `basalt_euroc_localize_demo`; world <- camera rotation and centre.
fn localize(bin: &str, map: &str, png: &Path, out: &Path) -> Option<(M3, [f64; 3])> {
    let euroc = out.join("query");
    let cam = euroc.join("mav0/cam0");
    let _ = fs::remove_dir_all(&euroc);
    fs::create_dir_all(cam.join("data")).ok()?;
    fs::copy(png, cam.join("data/1000000000.png")).ok()?;
    fs::write(
        cam.join("data.csv"),
        "#timestamp,filename\n1000000000,1000000000.png\n",
    )
    .ok()?;
    let result = out.join("localized");
    let status = Command::new(bin)
        .args(["--map", map, "--euroc-dir"])
        .arg(&euroc)
        .arg("--out-dir")
        .arg(&result)
        .args([
            "--max-hamming",
            "90",
            "--reprojection-px",
            "5",
            "--max-points",
            "1500",
        ])
        .args(["--min-inliers", "15"])
        // Extra localizer flags (later flags override the defaults above).
        .args(arg("--loc-args").unwrap_or_default().split_whitespace())
        .stdout(Stdio::null())
        .status()
        .ok()?;
    if !status.success() {
        return None;
    }
    let file = File::open(result.join("localization.tum")).ok()?;
    for line in BufReader::new(file).lines().map_while(Result::ok) {
        if line.starts_with('#') || line.trim().is_empty() {
            continue;
        }
        let v: Vec<f64> = line
            .split_whitespace()
            .map(|s| s.parse().unwrap())
            .collect();
        let r = quat_to_m3(v[4], v[5], v[6], v[7]);
        return Some((r, [v[1], v[2], v[3]]));
    }
    None
}

fn quat_to_m3(x: f64, y: f64, z: f64, w: f64) -> M3 {
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
        ],
        [
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
        ],
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

fn rot_z(a: f64) -> M3 {
    let (s, c) = a.sin_cos();
    [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
}

fn mul(a: &M3, b: &M3) -> M3 {
    let mut m = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            m[i][j] = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    m
}

fn mul_v(a: &M3, v: &[f64; 3]) -> [f64; 3] {
    [0, 1, 2].map(|i| (0..3).map(|k| a[i][k] * v[k]).sum())
}

fn transpose(a: &M3) -> M3 {
    let mut m = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            m[i][j] = a[j][i];
        }
    }
    m
}

fn yaw_of(rotation: Quat) -> f64 {
    let f = rotation * Vec3::X;
    (-f.z).atan2(f.x)
}

fn nearest(points: &[Vec3], p: Vec3) -> usize {
    (0..points.len())
        .min_by(|&a, &b| {
            let da = (points[a].x - p.x).powi(2) + (points[a].z - p.z).powi(2);
            let db = (points[b].x - p.x).powi(2) + (points[b].z - p.z).powi(2);
            da.total_cmp(&db)
        })
        .unwrap_or(0)
}

fn height_at(points: &[Vec3], p: Vec3) -> f64 {
    let i = nearest(points, p);
    let (a, b) = if i + 1 < points.len() {
        (points[i], points[i + 1])
    } else {
        (points[i.saturating_sub(1)], points[i])
    };
    let (dx, dz) = (b.x - a.x, b.z - a.z);
    let len2 = dx * dx + dz * dz;
    if len2 < 1e-9 {
        return a.y;
    }
    let s = (((p.x - a.x) * dx + (p.z - a.z) * dz) / len2).clamp(0.0, 1.0);
    a.y + s * (b.y - a.y)
}

fn cross_track(points: &[Vec3], p: Vec3) -> f64 {
    let i = nearest(points, p);
    let a = points[i.saturating_sub(1)];
    let b = points[(i + 1).min(points.len() - 1)];
    let (dx, dz) = (b.x - a.x, b.z - a.z);
    let len = (dx * dx + dz * dz).sqrt().max(1e-9);
    (dx * (p.z - a.z) - dz * (p.x - a.x)) / len
}

/// Path progress: the closest waypoint at or after `index`.
fn advance(points: &[Vec3], mut index: usize, p: Vec3) -> usize {
    while index + 1 < points.len()
        && (points[index + 1] - p).length() <= (points[index] - p).length()
    {
        index += 1;
    }
    index
}

/// Renders RNE front-camera images at SLAM camera poses (TUM: `t x y z qx qy qz qw`, world <-
/// camera, optical axes) into `out/rgb/<t>.png`, e.g. to build a localization map from the same
/// renderer the car's camera uses.
fn render_poses(poses: &str, manifest: &str, out: &Path) {
    fs::create_dir_all(out.join("rgb")).expect("out dir");
    let mut camera = RneCamera::new(manifest);
    let text = fs::read_to_string(poses).expect("poses");
    let mut count = 0;
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let f: Vec<&str> = line.split_whitespace().collect();
        let v: Vec<f64> = f[1..8].iter().map(|s| s.parse().expect("pose")).collect();
        let view = rne_view_from_slam(&quat_to_m3(v[3], v[4], v[5], v[6]), &[v[0], v[1], v[2]]);
        write_rgb_png(
            &out.join("rgb").join(format!("{}.png", f[0])),
            &camera.render_view(&view),
        );
        count += 1;
    }
    println!(
        "rendered {count} RNE frames into {}",
        out.join("rgb").display()
    );
}

fn write_rgb_png(path: &Path, rgb: &[u8]) {
    let file = File::create(path).expect("png");
    let mut encoder = Encoder::new(BufWriter::new(file), W as u32, H as u32);
    encoder.set_color(ColorType::Rgb);
    encoder.set_depth(BitDepth::Eight);
    encoder
        .write_header()
        .expect("png header")
        .write_image_data(rgb)
        .expect("png data");
}

fn arg(flag: &str) -> Option<String> {
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        if a == flag {
            return args.next();
        }
        if let Some(v) = a.strip_prefix(&format!("{flag}=")) {
            return Some(v.to_string());
        }
    }
    None
}
