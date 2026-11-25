//! A delivery robot drives a real street rebuilt from a 360-degree video, on ground truth.
//!
//! ```text
//! --street DIR       a visloc-rs street bundle (rne_bundle.py)
//! --out DIR          drive.csv, report.json and, unless --render-every 0, PNG frames
//! --render-every S   simulated seconds between rendered frames (default 0.5; 0: none)
//! --max-frames N     render at most N frames
//! --localizer BIN    steer on the robot's estimate: wheel odometry with errors, corrected by
//!                    visloc-rs fixes on the rendered front camera; BIN is visloc-rs
//!                    `visloc_map_localize_server` (default: ground truth)
//! --map DIR          its localization map (default: the bundle's `map`)
//! --loc-args "ARGS"  extra localizer flags (default: the Nav2 runs' tuned ones)
//! --crowd            add the bundle's crowd (crowd.json: walkers, cyclists, cars, a crossing)
//! --seed N           seeds the crowd's behaviour (default 0)
//! ```
//!
//! The robot plans on the bundle's lane map with `rne_nav` and follows the plan by pure
//! pursuit (see the library). Each rendered frame has the robot's front camera (the
//! bundle's camera height, 90 degree field of view) and a chase view 3.2 m behind and
//! 2.75 m up, with the robot drawn as a box the size of the delivery robot.

use png::{BitDepth, ColorType, Encoder};
use rne_math::{Quat, Transform3, Vec3};
use rne_nav::Pose2d;
use rne_render::{
    validate_gaussian_splat_manifest, Camera, HybridRenderScene, RenderScene, RenderSceneItem,
    VisualShape,
};
use rne_render_3dgs::{
    load_splat_actor, render_hybrid_scene_camera, GaussianSplatBackground, SplatActorAsset,
};
use rne_render_wgpu::WgpuRenderBackend;
use std::cell::RefCell;
use std::fs::{self, File};
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use street_splat_nav::crowd::{Crowd, Light};
use street_splat_nav::{drive_localized, map_to_rne, CrowdStats, NavConfig, RobotState, Street};

/// The tuned localizer flags of the visloc-rs Nav2 runs.
const TUNED: &str =
    "--max_hamming 90 --reprojection_threshold 5 --min_inliers 20 --projection_radius 40";

const CAM_SIZE: u32 = 640;
const CHASE_WIDTH: u32 = 960;
const CHASE_HEIGHT: u32 = 540;
const CLEAR: [f32; 4] = [0.0, 0.0, 0.0, 1.0];
/// The delivery robot: 0.87 m long, 0.55 m tall, 0.70 m wide.
const BODY_M: [f64; 3] = [0.87, 0.55, 0.70];

fn main() {
    let street_dir = PathBuf::from(arg("--street").expect("--street DIR"));
    let out = PathBuf::from(arg("--out").unwrap_or_else(|| "street_splat_nav".into()));
    let render_every: f64 = arg("--render-every").map_or(0.5, |v| v.parse().expect("seconds"));
    let max_frames: usize = arg("--max-frames").map_or(usize::MAX, |v| v.parse().expect("count"));
    fs::create_dir_all(&out).expect("output directory");
    let street = Street::load(&street_dir).expect("street bundle");
    println!(
        "street {}: {} goals. {}",
        street_dir.display(),
        street.goals.len(),
        street.credit
    );

    let seed: u64 = arg("--seed").map_or(0, |v| v.parse().expect("seed"));
    let mut crowd = if std::env::args().any(|a| a == "--crowd") {
        street.crowd(seed).expect("crowd.json")
    } else {
        None
    };
    let localizer = arg("--localizer");
    let renderer = RefCell::new(
        (render_every > 0.0 || localizer.is_some())
            .then(|| Renderer::new(&street, crowd.as_ref().map(|c| Looks::load(&street, c)))),
    );
    // where everyone was at the last step: the fix camera sees them too
    let snapshot = RefCell::new(crowd.as_ref().map(Snapshot::of).unwrap_or_default());
    let mut server = localizer.map(|bin| {
        let map = arg("--map").map_or_else(|| street.dir.join("map"), PathBuf::from);
        let loc_args = arg("--loc-args").unwrap_or_else(|| TUNED.into());
        Server::spawn(&bin, &map, &loc_args)
    });
    let fix_pgm = out.join("fix.pgm");
    let mut fix_log = BufWriter::new(File::create(out.join("fixes.csv")).expect("fixes.csv"));
    writeln!(
        fix_log,
        "t_s,true_x,true_y,est_x,est_y,fix_x,fix_y,fix_yaw,inliers"
    )
    .unwrap();
    let mut localize = |truth: &RobotState, estimate: Pose2d| -> Option<Pose2d> {
        let server = server.as_mut()?;
        let image = {
            let mut guard = renderer.borrow_mut();
            let r = guard.as_mut().expect("renderer");
            r.place_crowd(&street, &snapshot.borrow());
            r.front(&street, truth)
        };
        write_luma_pgm(&fix_pgm, &image);
        let camera_z = street.ground_at(estimate.x_m, estimate.y_m) + street.camera_height_m;
        let reply = server.ask(&fix_pgm, estimate, camera_z);
        let fix = reply.map(|(pose, _)| pose);
        writeln!(
            fix_log,
            "{:.2},{:.3},{:.3},{:.3},{:.3},{},{}",
            truth.t_s,
            truth.x_m,
            truth.y_m,
            estimate.x_m,
            estimate.y_m,
            fix.map_or(",,".into(), |f| format!(
                "{:.3},{:.3},{:.4}",
                f.x_m, f.y_m, f.yaw_rad
            )),
            reply.map_or(0, |(_, n)| n)
        )
        .unwrap();
        fix
    };
    let mut log = BufWriter::new(File::create(out.join("drive.csv")).expect("drive.csv"));
    writeln!(log, "t_s,x,y,z,yaw,pitch,speed,goal").unwrap();
    let mut next_frame = 0.0;
    let mut frame = 0usize;
    let config = NavConfig::default();
    let use_fixes = arg("--localizer").is_some();
    let mut stats = CrowdStats::default();
    let mut people_log = BufWriter::new(File::create(out.join("people.csv")).expect("people.csv"));
    writeln!(people_log, "t_s,id,kind,x,y,yaw,phase").unwrap();
    let report = drive_localized(
        &street,
        &config,
        use_fixes.then_some(&mut localize as _),
        crowd.as_mut(),
        |s, people| {
            if let Some(people) = people {
                *snapshot.borrow_mut() = Snapshot::of(people);
                if ((s.t_s / config.step_s).round() as u64).is_multiple_of(10) {
                    stats.sample(people, s, 10.0 * config.step_s);
                    for i in 0..people.len() {
                        let (kind, xy, yaw, phase) = people.agent(i);
                        writeln!(
                            people_log,
                            "{:.2},{i},{kind},{:.3},{:.3},{yaw:.4},{phase}",
                            s.t_s, xy[0], xy[1]
                        )
                        .unwrap();
                    }
                }
            }
            if ((s.t_s / config.step_s).round() as u64).is_multiple_of(5) {
                writeln!(
                    log,
                    "{:.2},{:.3},{:.3},{:.3},{:.4},{:.4},{:.3},{}",
                    s.t_s, s.x_m, s.y_m, s.z_m, s.yaw_rad, s.pitch_rad, s.speed_m_s, s.goal
                )
                .unwrap();
            }
            if render_every > 0.0 && s.t_s + 1e-9 >= next_frame && frame < max_frames {
                if let Some(r) = renderer.borrow_mut().as_mut() {
                    r.place_crowd(&street, &snapshot.borrow());
                    r.render(&street, s, &out, frame);
                    frame += 1;
                    next_frame += render_every;
                }
            }
        },
    )
    .expect("drive");
    log.flush().unwrap();
    let json = format!(
        "{{\"reached\": {}, \"goals\": {}, \"contacts\": {}, \"time_s\": {:.1}, \"distance_m\": {:.1}, \
         \"kerb_time_s\": {:.1}, \"kerb_max_speed_m_s\": {:.3}, \"min_solid_clearance_m\": {:.2}, \
         \"walk_distance_median_m\": {:.2}, \"walk_distance_max_m\": {:.2}, \"frames\": {frame}, \
         \"fixes_attempted\": {}, \"fixes_accepted\": {}, \"fixes_rejected\": {}, \
         \"pose_error_rmse_m\": {:.3}, \"pose_error_median_m\": {:.3}, \"pose_error_max_m\": {:.3}, \
         \"people_contacts\": {}, \"crowd_samples\": {}, \"people_on_road\": {}, \
         \"people_in_robot_body_m\": {:.3}, \"people_in_cars_m\": {:.3}, \"longest_still_s\": {:.1}, \
         \"trajectory_hash\": \"{:016x}\"}}\n",
        report.reached,
        report.goals,
        report.contacts,
        report.time_s,
        report.distance_m,
        report.kerb_time_s,
        report.kerb_max_speed_m_s,
        report.min_solid_clearance_m,
        report.walk_distance_median_m,
        report.walk_distance_max_m,
        report.fixes_attempted,
        report.fixes_accepted,
        report.fixes_rejected,
        report.pose_error_rmse_m,
        report.pose_error_median_m,
        report.pose_error_max_m,
        report.people_contacts,
        stats.samples,
        stats.on_road,
        stats.body_intrusion_m,
        stats.car_intrusion_m,
        stats.longest_still_s,
        report.trajectory_hash
    );
    fs::write(out.join("report.json"), &json).expect("report.json");
    print!("{json}");
}

struct Renderer {
    backend: WgpuRenderBackend,
    environment: rne_render::GaussianSplatEnvironment,
    background: GaussianSplatBackground,
    front: Camera,
    chase: Camera,
    looks: Option<Looks>,
}

/// The crowd's looks: every agent's gaussians at each gait phase (actor `i` is agent `i`),
/// then the crossing at each light state (green, amber, red).
struct Looks {
    phases: Vec<Vec<SplatActorAsset>>,
    shown: Vec<usize>,
    decor: Vec<SplatActorAsset>,
    light: Option<usize>,
}

/// Where everyone is, for the renderer: (kind, x, y, yaw, phase) per agent, and the light.
#[derive(Clone, Debug, Default)]
struct Snapshot {
    agents: Vec<(u8, [f64; 2], f64, usize)>,
    light: Option<Light>,
}

impl Snapshot {
    fn of(crowd: &Crowd) -> Self {
        Self {
            agents: (0..crowd.len()).map(|i| crowd.agent(i)).collect(),
            light: crowd.spec().crossing_s.map(|_| crowd.light()),
        }
    }
}

impl Looks {
    fn load(street: &Street, crowd: &Crowd) -> Self {
        let load = |name: &String| load_splat_actor(&street.dir.join(name)).expect("crowd actor");
        let spec = crowd.spec();
        let phases: Vec<Vec<SplatActorAsset>> = spec
            .agents
            .iter()
            .map(|a| a.phases.iter().map(load).collect())
            .collect();
        let decor = ["green", "amber", "red"]
            .iter()
            .filter_map(|k| spec.decor.get(*k).map(load))
            .collect();
        println!(
            "crowd looks: {} agents, {} gaussians at a time",
            phases.len(),
            phases.iter().map(|p| p[0].len()).sum::<usize>()
        );
        Self {
            shown: vec![0; phases.len()],
            phases,
            decor,
            light: None,
        }
    }

    fn actors(&self) -> Vec<SplatActorAsset> {
        self.phases
            .iter()
            .map(|p| p[0].clone())
            .chain(self.decor.iter().cloned())
            .collect()
    }
}

/// An agent-frame (z up, x forward) pose into RNE's y-up world.
fn actor_pose(x: f64, y: f64, z: f64, yaw: f64) -> Transform3 {
    let z_up_to_y_up = Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2);
    Transform3 {
        translation: map_to_rne(x, y, z),
        rotation: (Quat::from_rotation_y(yaw) * z_up_to_y_up).normalize(),
        scale: Vec3::ONE,
    }
}

impl Renderer {
    fn new(street: &Street, looks: Option<Looks>) -> Self {
        let backend = WgpuRenderBackend::new().expect("wgpu backend");
        let environment =
            validate_gaussian_splat_manifest(&street.manifest()).expect("splat manifest");
        let background = GaussianSplatBackground::from_environment_with_actors(
            backend.device(),
            &environment,
            looks.as_ref().map_or_else(Vec::new, Looks::actors),
        )
        .expect("splat background");
        Self {
            backend,
            environment,
            background,
            front: Camera::new(CAM_SIZE, CAM_SIZE, std::f64::consts::FRAC_PI_2),
            chase: Camera::new(CHASE_WIDTH, CHASE_HEIGHT, 60f64.to_radians()),
            looks,
        }
    }

    /// Puts everyone where the snapshot has them, in their gait phase, and lights the crossing.
    fn place_crowd(&mut self, street: &Street, snap: &Snapshot) {
        let Some(looks) = self.looks.as_mut() else {
            return;
        };
        let queue = self.backend.queue();
        for (i, &(_, xy, yaw, phase)) in snap.agents.iter().enumerate() {
            if looks.shown[i] != phase {
                self.background
                    .replace_actor_asset(queue, i, looks.phases[i][phase].clone())
                    .expect("phase");
                looks.shown[i] = phase;
            }
            let pose = actor_pose(xy[0], xy[1], street.ground_at(xy[0], xy[1]), yaw);
            self.background
                .set_actor_pose(queue, i, Some(&pose))
                .expect("agent pose");
        }
        let light = snap.light.map(|l| match l {
            Light::Green => 0,
            Light::Amber => 1,
            Light::Red => 2,
        });
        if light != looks.light && !looks.decor.is_empty() {
            let first = looks.phases.len();
            for k in 0..looks.decor.len() {
                let pose = (Some(k) == light).then(|| actor_pose(0.0, 0.0, 0.0, 0.0));
                self.background
                    .set_actor_pose(queue, first + k, pose.as_ref())
                    .expect("crossing");
            }
            looks.light = light;
        }
    }

    /// The front camera at the true pose, RGBA8.
    fn front(&mut self, street: &Street, s: &RobotState) -> rne_render::ImageFrame {
        let heading = Quat::from_rotation_y(s.yaw_rad);
        let body = (heading * Quat::from_rotation_z(s.pitch_rad)).normalize();
        let base = map_to_rne(s.x_m, s.y_m, s.z_m);
        let look_forward = Quat::from_rotation_y(-std::f64::consts::FRAC_PI_2);
        let front_view = Transform3::from_translation_rotation(
            base + body * Vec3::new(0.0, street.camera_height_m, 0.0),
            (body * look_forward).normalize(),
        );
        let empty = HybridRenderScene::new(self.environment.clone(), RenderScene::new());
        render_hybrid_scene_camera(
            &mut self.backend,
            &mut self.background,
            &self.front,
            &front_view,
            &empty,
            CLEAR,
        )
        .expect("front render")
        .color
    }

    fn render(&mut self, street: &Street, s: &RobotState, out: &Path, frame: usize) {
        // Robot frame: +X forward, +Y up, +Z right. Map yaw (about z up) is RNE yaw about +Y.
        let heading = Quat::from_rotation_y(s.yaw_rad);
        let body = (heading * Quat::from_rotation_z(s.pitch_rad)).normalize();
        let base = map_to_rne(s.x_m, s.y_m, s.z_m);
        // RNE cameras look along their local -Z.
        let look_forward = Quat::from_rotation_y(-std::f64::consts::FRAC_PI_2);
        let front_view = Transform3::from_translation_rotation(
            base + body * Vec3::new(0.0, street.camera_height_m, 0.0),
            (body * look_forward).normalize(),
        );
        let empty = HybridRenderScene::new(self.environment.clone(), RenderScene::new());
        let image = render_hybrid_scene_camera(
            &mut self.backend,
            &mut self.background,
            &self.front,
            &front_view,
            &empty,
            CLEAR,
        )
        .expect("front render");
        write_png(
            &out.join(format!("front_{frame:05}.png")),
            image.color.width,
            image.color.height,
            &image.color.rgba8,
        );

        let chase_view = Transform3::from_translation_rotation(
            base + heading * Vec3::new(-3.2, 2.75, 0.0),
            (heading * look_forward * Quat::from_rotation_x(-16f64.to_radians())).normalize(),
        );
        let mut foreground = RenderScene::new();
        foreground.items.push(RenderSceneItem {
            transform: Transform3 {
                translation: base + body * Vec3::new(0.0, BODY_M[1] / 2.0 + 0.08, 0.0),
                rotation: body,
                scale: Vec3::ONE,
            },
            shape: VisualShape::Box {
                size_m: Vec3::new(BODY_M[0], BODY_M[1], BODY_M[2]),
            },
            color_rgba: [0.93, 0.93, 0.9, 1.0],
            mesh: None,
            base_color_texture: None,
            material: Default::default(),
        });
        let hybrid = HybridRenderScene::new(self.environment.clone(), foreground);
        let image = render_hybrid_scene_camera(
            &mut self.backend,
            &mut self.background,
            &self.chase,
            &chase_view,
            &hybrid,
            CLEAR,
        )
        .expect("chase render");
        write_png(
            &out.join(format!("chase_{frame:05}.png")),
            image.color.width,
            image.color.height,
            &image.color.rgba8,
        );
    }
}

/// The localizer's input: 8-bit luma (BT.601), binary PGM.
fn write_luma_pgm(path: &Path, image: &rne_render::ImageFrame) {
    let mut bytes = format!("P5\n{} {}\n255\n", image.width, image.height).into_bytes();
    bytes.extend(image.rgba8.chunks_exact(4).map(|p| {
        ((299 * u32::from(p[0]) + 587 * u32::from(p[1]) + 114 * u32::from(p[2]) + 500) / 1000) as u8
    }));
    fs::write(path, bytes).expect("fix image");
}

/// visloc-rs `visloc_map_localize_server`, kept running for the whole drive.
struct Server {
    _child: Child,
    stdin: ChildStdin,
    stdout: BufReader<ChildStdout>,
}

impl Server {
    fn spawn(bin: &str, map: &Path, loc_args: &str) -> Self {
        let mut child = Command::new(bin)
            .arg("--map")
            .arg(map)
            .args(loc_args.split_whitespace())
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()
            .expect("localizer");
        let stdin = child.stdin.take().expect("localizer stdin");
        let stdout = BufReader::new(child.stdout.take().expect("localizer stdout"));
        Self {
            _child: child,
            stdin,
            stdout,
        }
    }

    /// A fix (x, y, yaw) and its inliers for one image, with the estimate as the prior.
    fn ask(&mut self, image: &Path, estimate: Pose2d, camera_z_m: f64) -> Option<(Pose2d, usize)> {
        // map <- camera with OpenCV axes (x right, y down, z forward) for a level camera
        let (s, c) = estimate.yaw_rad.sin_cos();
        let q = quat_from_columns([s, -c, 0.0], [0.0, 0.0, -1.0], [c, s, 0.0]);
        writeln!(
            self.stdin,
            "{} {} {} {} {} {} {} {}",
            image.display(),
            estimate.x_m,
            estimate.y_m,
            camera_z_m,
            q[0],
            q[1],
            q[2],
            q[3]
        )
        .expect("localizer write");
        self.stdin.flush().expect("localizer flush");
        let mut line = String::new();
        self.stdout.read_line(&mut line).expect("localizer read");
        let f: Vec<&str> = line.split_whitespace().collect();
        if f.first() != Some(&"ok") || f.len() < 9 {
            return None;
        }
        let v: Vec<f64> = f[1..8].iter().filter_map(|s| s.parse().ok()).collect();
        let inliers = f[8].parse().unwrap_or(0);
        // yaw from the camera's forward (z) axis in the map
        let (x, y, z, w) = (v[3], v[4], v[5], v[6]);
        let forward = [2.0 * (x * z + w * y), 2.0 * (y * z - w * x)];
        Some((
            Pose2d::new(v[0], v[1], forward[1].atan2(forward[0])),
            inliers,
        ))
    }
}

/// Quaternion (x, y, z, w) of the rotation with these columns.
fn quat_from_columns(x: [f64; 3], y: [f64; 3], z: [f64; 3]) -> [f64; 4] {
    let m = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]];
    let trace = m[0][0] + m[1][1] + m[2][2];
    if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        [
            (m[2][1] - m[1][2]) / s,
            (m[0][2] - m[2][0]) / s,
            (m[1][0] - m[0][1]) / s,
            0.25 * s,
        ]
    } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
        let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
        [
            0.25 * s,
            (m[0][1] + m[1][0]) / s,
            (m[0][2] + m[2][0]) / s,
            (m[2][1] - m[1][2]) / s,
        ]
    } else if m[1][1] > m[2][2] {
        let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
        [
            (m[0][1] + m[1][0]) / s,
            0.25 * s,
            (m[1][2] + m[2][1]) / s,
            (m[0][2] - m[2][0]) / s,
        ]
    } else {
        let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
        [
            (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s,
            0.25 * s,
            (m[1][0] - m[0][1]) / s,
        ]
    }
}

fn write_png(path: &Path, width: u32, height: u32, rgba8: &[u8]) {
    let file = File::create(path).expect("png file");
    let mut encoder = Encoder::new(BufWriter::new(file), width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header().expect("png header");
    writer.write_image_data(rgba8).expect("png pixels");
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
