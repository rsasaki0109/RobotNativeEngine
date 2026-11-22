//! An Ackermann car drives through a 3D Gaussian Splatting reconstruction of a real street.
//!
//! The splat was trained from KITTI raw drive 0018 with camera poses from LiDAR SLAM
//! (`lidar_slam`), so it is metric and gravity-aligned. The car follows the SLAM path with
//! pure pursuit ([`rne_robot::ackermann_kinematics`]); every `--render-every` seconds the
//! example renders the car's front camera (KITTI `image_02` intrinsics and mount) and a chase
//! view with the car drawn as a box over the splat.
//!
//! ```text
//! --manifest PATH   splat manifest (.rne.splat.toml)
//! --path PATH       CSV: t_s, x, y, z, yaw (RNE frame: +Y up)
//! --out DIR         PNG frames and drive.csv
//! --render-every S  seconds between captured frames (default 0.2)
//! --speed M_S       target speed (default 7.0)
//! ```

use png::{BitDepth, ColorType, Encoder};
use rne_core::SimDuration;
use rne_ecs::{spawn_named, World};
use rne_math::{Quat, Seconds, Transform3 as MathTransform, Vec3};
use rne_physics::RigidBody;
use rne_render::{
    validate_gaussian_splat_manifest, Camera, HybridRenderScene, RenderScene, RenderSceneItem,
    VisualShape,
};
use rne_render_3dgs::{load_gaussian_splat_background, render_hybrid_scene_camera};
use rne_render_wgpu::WgpuRenderBackend;
use rne_robot::{ackermann_kinematics, pure_pursuit_steering, AckermannDrive};
use rne_world::Transform3;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

const SIM_HZ: f64 = 50.0;
const WHEELBASE_M: f64 = 2.71;
const LOOKAHEAD_M: f64 = 6.0;
/// KITTI rectified image_02 at half resolution.
const CAM_WIDTH: u32 = 613;
const CAM_HEIGHT: u32 = 185;
const CAM_FY: f64 = 353.5456;
/// image_02 in the OXTS body frame (x forward, y left, z up), from the KITTI calibration.
const CAM_IN_BASE: [f64; 3] = [1.144, -0.246, 0.726];
/// OXTS unit height above the road.
const BASE_HEIGHT_M: f64 = 0.93;
const CHASE_WIDTH: u32 = 640;
const CHASE_HEIGHT: u32 = 360;
const CLEAR: [f32; 4] = [0.0, 0.0, 0.0, 1.0];

fn main() {
    let manifest = PathBuf::from(arg("--manifest").expect("--manifest"));
    let path_csv = PathBuf::from(arg("--path").expect("--path"));
    let out = PathBuf::from(arg("--out").unwrap_or_else(|| "kitti_splat_drive".into()));
    let render_every: f64 = arg("--render-every").map_or(0.2, |v| v.parse().unwrap());
    let speed: f64 = arg("--speed").map_or(7.0, |v| v.parse().unwrap());
    fs::create_dir_all(&out).expect("output directory");

    let (waypoints, start_yaw) = read_path(&path_csv);
    println!("{} waypoints, start yaw {start_yaw:.3}", waypoints.len());

    let environment = validate_gaussian_splat_manifest(&manifest).expect("splat manifest");
    let mut backend = WgpuRenderBackend::new().expect("wgpu backend");
    println!("adapter ready");
    let mut background =
        load_gaussian_splat_background(backend.device(), &environment).expect("splat background");
    println!("splat loaded: {}", environment.ply_path.display());

    let mut world = World::new();
    let car = spawn_named(&mut world, "car");
    let start = Transform3 {
        translation: waypoints[0],
        rotation: Quat::from_rotation_y(start_yaw),
        scale: Vec3::ONE,
    };
    world.entity_mut(car).insert((
        AckermannDrive {
            wheelbase_m: WHEELBASE_M,
            max_speed_m_s: 15.0,
            ..AckermannDrive::default()
        },
        start,
        RigidBody::default(),
    ));

    let dt = SimDuration::from_seconds(Seconds::new(1.0 / SIM_HZ));
    let front = Camera::new(
        CAM_WIDTH,
        CAM_HEIGHT,
        2.0 * (0.5 * CAM_HEIGHT as f64 / CAM_FY).atan(),
    );
    let chase = Camera::new(CHASE_WIDTH, CHASE_HEIGHT, 55f64.to_radians());
    let mut log = BufWriter::new(File::create(out.join("drive.csv")).expect("drive.csv"));
    writeln!(log, "t_s,x,y,z,yaw,speed,steering,cross_track_m,frame").unwrap();

    let mut index = 0usize;
    let mut frame = 0usize;
    let mut next_capture = 0.0;
    let mut t = 0.0;
    loop {
        let pose = *world.get::<Transform3>(car).expect("car pose");
        // Advance along the path: the closest waypoint at or after the last one.
        while index + 1 < waypoints.len()
            && (waypoints[index + 1] - pose.translation).length()
                <= (waypoints[index] - pose.translation).length()
        {
            index += 1;
        }
        let mut target = index;
        while target + 1 < waypoints.len()
            && (waypoints[target] - pose.translation).length() < LOOKAHEAD_M
        {
            target += 1;
        }
        let done = index + 1 >= waypoints.len()
            || (target + 1 == waypoints.len()
                && (waypoints[target] - pose.translation).length() < 1.5);
        {
            let mut drive = world.get_mut::<AckermannDrive>(car).expect("drive");
            let steering =
                pure_pursuit_steering(&pose, waypoints[target], WHEELBASE_M, LOOKAHEAD_M);
            drive.target_steering_rad = steering.clamp(-drive.max_steering_rad, drive.max_steering_rad);
            drive.target_speed_m_s = if done { 0.0 } else { speed };
        }

        if t + 1e-9 >= next_capture || done {
            let drive = world.get::<AckermannDrive>(car).expect("drive").clone();
            let yaw = yaw_of(pose.rotation);
            let cross = cross_track(&waypoints, index, pose.translation);
            let math_pose = MathTransform::from_translation_rotation(pose.translation, pose.rotation);
            render_frame(
                &mut backend,
                &mut background,
                &environment,
                &front,
                &chase,
                &math_pose,
                &out,
                frame,
            );
            writeln!(
                log,
                "{t:.2},{:.3},{:.3},{:.3},{yaw:.4},{:.2},{:.4},{cross:.3},{frame}",
                pose.translation.x,
                pose.translation.y,
                pose.translation.z,
                drive.speed_m_s,
                drive.steering_rad
            )
            .unwrap();
            println!(
                "t {t:6.2}s frame {frame:4} speed {:5.2} m/s cross-track {cross:+.2} m",
                drive.speed_m_s
            );
            frame += 1;
            next_capture += render_every;
        }
        if done {
            break;
        }
        ackermann_kinematics(&mut world, dt);
        t += 1.0 / SIM_HZ;
    }
    println!("drove {t:.1} s, {frame} frames -> {}", out.display());
}

#[allow(clippy::too_many_arguments)]
fn render_frame(
    backend: &mut WgpuRenderBackend,
    background: &mut rne_render_3dgs::GaussianSplatBackground,
    environment: &rne_render::GaussianSplatEnvironment,
    front: &Camera,
    chase: &Camera,
    car: &MathTransform,
    out: &Path,
    frame: usize,
) {
    // Car frame: +X forward, +Y up, +Z right; RNE cameras look along their local -Z.
    let look_forward = Quat::from_rotation_y(-std::f64::consts::FRAC_PI_2);
    let mount = Vec3::new(CAM_IN_BASE[0], CAM_IN_BASE[2], -CAM_IN_BASE[1]);
    let front_view = MathTransform::from_translation_rotation(
        car.translation + car.rotation * mount,
        (car.rotation * look_forward).normalize(),
    );
    let empty = HybridRenderScene::new(environment.clone(), RenderScene::new());
    let image = render_hybrid_scene_camera(backend, background, front, &front_view, &empty, CLEAR)
        .expect("front render");
    write_png(
        &out.join(format!("front_{frame:04}.png")),
        image.color.width,
        image.color.height,
        &image.color.rgba8,
    );

    let back = car.rotation * Vec3::new(-10.0, 4.5, 0.0);
    let chase_view = MathTransform::from_translation_rotation(
        car.translation + back,
        (car.rotation * look_forward * Quat::from_rotation_x(-0.32)).normalize(),
    );
    let mut foreground = RenderScene::new();
    foreground.items.push(RenderSceneItem {
        transform: MathTransform {
            translation: car.translation + car.rotation * Vec3::new(0.3, 0.75 - BASE_HEIGHT_M, 0.0),
            rotation: car.rotation,
            scale: Vec3::ONE,
        },
        shape: VisualShape::Box {
            size_m: Vec3::new(4.3, 1.5, 1.8),
        },
        color_rgba: [0.85, 0.12, 0.1, 1.0],
        mesh: None,
        base_color_texture: None,
        material: Default::default(),
    });
    let hybrid = HybridRenderScene::new(environment.clone(), foreground);
    let image = render_hybrid_scene_camera(backend, background, chase, &chase_view, &hybrid, CLEAR)
        .expect("chase render");
    write_png(
        &out.join(format!("chase_{frame:04}.png")),
        image.color.width,
        image.color.height,
        &image.color.rgba8,
    );
}

fn read_path(path: &Path) -> (Vec<Vec3>, f64) {
    let text = fs::read_to_string(path).expect("path csv");
    let mut points: Vec<Vec3> = Vec::new();
    let mut yaw = None;
    for line in text.lines().filter(|l| !l.starts_with('#') && !l.trim().is_empty()) {
        let v: Vec<f64> = line.split(',').map(|s| s.trim().parse().unwrap()).collect();
        let p = Vec3::new(v[1], v[2], v[3]);
        if points.last().is_none_or(|last| (p - *last).length() >= 1.0) {
            points.push(p);
            yaw.get_or_insert(v[4]);
        }
    }
    (points, yaw.unwrap_or(0.0))
}

fn yaw_of(rotation: Quat) -> f64 {
    let forward = rotation * Vec3::X;
    (-forward.z).atan2(forward.x)
}

fn cross_track(points: &[Vec3], index: usize, p: Vec3) -> f64 {
    let a = points[index.saturating_sub(1)];
    let b = points[(index + 1).min(points.len() - 1)];
    let ab = Vec3::new(b.x - a.x, 0.0, b.z - a.z);
    let ap = Vec3::new(p.x - a.x, 0.0, p.z - a.z);
    let len = ab.length().max(1e-9);
    // Positive when the car is to the right of the path (+Z is right of +X).
    (ab.x * ap.z - ab.z * ap.x) / len
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
