//! Traffic in a real street: a car splat cut out of the KITTI reconstruction drives ahead of the
//! ego car, which follows it with the Intelligent Driver Model.
//!
//! The lead car is a movable splat actor ([`rne_render_3dgs::SplatActorAsset`]) in the same GPU
//! buffer as the street, so it is sorted with the street's Gaussians and occludes and is
//! occluded by them. Both cars move along the SLAM path; the lead follows a script (cruise,
//! stop, pull away) and the ego reacts to the bumper gap. Every `--render-every` seconds the
//! ego's front camera is rendered.
//!
//! ```text
//! --manifest SPLAT.toml   street splat
//! --actor PLY             car splat in its own frame (+x forward, +y left, +z up, origin at
//!                         the bottom centre)
//! --path CSV              t_s, x, y, z, yaw (RNE frame, OXTS unit above the road)
//! --out DIR               front_NNNN.png and traffic.csv
//! [--render-every 0.2] [--duration 40] [--lead-start 22] [--stop-at 14]
//! ```

use kitti_splat_drive::{arc_length_near, arc_lengths, parse_path, pose_at, Idm};
use png::{BitDepth, ColorType, Encoder};
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{validate_gaussian_splat_manifest, Camera, HybridRenderScene, RenderScene};
use rne_render_3dgs::{load_splat_actor, render_hybrid_scene_camera, GaussianSplatBackground};
use rne_render_wgpu::WgpuRenderBackend;
use std::f64::consts::FRAC_PI_2;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

const SIM_HZ: f64 = 50.0;
const CAM_WIDTH: u32 = 1226;
const CAM_HEIGHT: u32 = 370;
const CAM_FY: f64 = 707.0912;
/// image_02 in the OXTS body frame (x forward, y left, z up).
const CAM_IN_BASE: [f64; 3] = [1.144, -0.246, 0.726];
/// OXTS unit height above the road.
const BASE_HEIGHT_M: f64 = 0.93;
/// Bumper-to-bumper offset between two cars' reference points.
const CAR_LENGTH_M: f64 = 4.4;

/// The lead car's script: speed over time.
fn lead_speed(t: f64, stop_at: f64) -> f64 {
    const CRUISE: f64 = 6.0;
    const DECEL: f64 = 2.5;
    const HOLD_S: f64 = 4.0;
    const ACCEL: f64 = 1.5;
    const FAST: f64 = 9.0;
    let stop_done = stop_at + CRUISE / DECEL;
    if t < stop_at {
        CRUISE
    } else if t < stop_done {
        CRUISE - DECEL * (t - stop_at)
    } else if t < stop_done + HOLD_S {
        0.0
    } else {
        (ACCEL * (t - stop_done - HOLD_S)).min(FAST)
    }
}

fn main() {
    let manifest = arg("--manifest").expect("--manifest");
    let actor_path = PathBuf::from(arg("--actor").expect("--actor"));
    let out = PathBuf::from(arg("--out").unwrap_or_else(|| "kitti_splat_traffic".into()));
    let render_every: f64 = arg("--render-every").map_or(0.2, |v| v.parse().unwrap());
    let duration: f64 = arg("--duration").map_or(40.0, |v| v.parse().unwrap());
    let lead_start: f64 = arg("--lead-start").map_or(22.0, |v| v.parse().unwrap());
    let stop_at: f64 = arg("--stop-at").map_or(14.0, |v| v.parse().unwrap());
    fs::create_dir_all(&out).expect("output directory");

    let path = parse_path(&fs::read_to_string(arg("--path").expect("--path")).expect("path"))
        .expect("path");
    let points = path.points;
    let lengths = arc_lengths(&points);

    let mut backend = WgpuRenderBackend::new().expect("wgpu backend");
    let environment = validate_gaussian_splat_manifest(Path::new(&manifest)).expect("manifest");
    let actor = load_splat_actor(&actor_path).expect("actor");
    println!("actor: {} Gaussians", actor.len());
    let mut background = GaussianSplatBackground::from_environment_with_actors(
        backend.device(),
        &environment,
        vec![actor],
    )
    .expect("splat");
    let scene = HybridRenderScene::new(environment.clone(), RenderScene::new());
    let camera = Camera::new(
        CAM_WIDTH,
        CAM_HEIGHT,
        2.0 * (0.5 * f64::from(CAM_HEIGHT) / CAM_FY).atan(),
    );

    let idm = Idm {
        desired_speed: 8.0,
        headway_s: 1.2,
        min_gap_m: 4.0,
        max_accel: 1.5,
        comfort_decel: 3.0,
    };
    let mut log = BufWriter::new(File::create(out.join("traffic.csv")).expect("traffic.csv"));
    writeln!(log, "t_s,frame,ego_s,ego_v,ego_a,lead_s,lead_v,gap_m").unwrap();

    let dt = 1.0 / SIM_HZ;
    let (mut ego_s, mut ego_v, mut lead_s) = (0.0, 6.0, lead_start);
    let mut hint = 0usize;
    let (mut t, mut next, mut frame) = (0.0, 0.0, 0usize);
    let end = *lengths.last().unwrap() - 5.0;
    while t <= duration && ego_s < end {
        let lead_v = lead_speed(t, stop_at);
        let gap = lead_s - ego_s - CAR_LENGTH_M;
        let accel = idm.accel(ego_v, Some(gap), lead_v).max(-8.0);
        if t + 1e-9 >= next {
            next += render_every;
            let (ego_p, ego_yaw) = pose_at(&points, &lengths, ego_s);
            // Keep the projection consistent with the path the actor is placed on.
            let (_, h) = arc_length_near(&points, &lengths, hint, ego_p);
            hint = h;
            let (lead_p, lead_yaw) = pose_at(&points, &lengths, lead_s);
            let lead_pose = Transform3::from_translation_rotation(
                lead_p - Vec3::new(0.0, BASE_HEIGHT_M, 0.0),
                // Actor frame (+x forward, +z up) -> RNE car frame (+x forward, +y up).
                Quat::from_rotation_y(lead_yaw) * Quat::from_rotation_x(-FRAC_PI_2),
            );
            background
                .set_actor_pose(backend.queue(), 0, Some(&lead_pose))
                .expect("actor pose");
            let image = render_front(
                &mut backend,
                &mut background,
                &camera,
                &scene,
                ego_p,
                ego_yaw,
            );
            write_png(&out.join(format!("front_{frame:04}.png")), &image);
            writeln!(
                log,
                "{t:.2},{frame},{ego_s:.3},{ego_v:.3},{accel:.3},{lead_s:.3},{lead_v:.3},{gap:.3}"
            )
            .unwrap();
            frame += 1;
        }
        ego_v = (ego_v + accel * dt).max(0.0);
        ego_s += ego_v * dt;
        lead_s += lead_v * dt;
        t += dt;
    }
    println!("{frame} frames, {t:.1} s -> {}", out.display());
}

fn render_front(
    backend: &mut WgpuRenderBackend,
    background: &mut GaussianSplatBackground,
    camera: &Camera,
    scene: &HybridRenderScene,
    position: Vec3,
    yaw: f64,
) -> Vec<u8> {
    let rotation = Quat::from_rotation_y(yaw);
    // Car frame: +X forward, +Y up, +Z right; RNE cameras look along local -Z.
    let mount = Vec3::new(CAM_IN_BASE[0], CAM_IN_BASE[2], -CAM_IN_BASE[1]);
    let view = Transform3::from_translation_rotation(
        position + rotation * mount,
        (rotation * Quat::from_rotation_y(-FRAC_PI_2)).normalize(),
    );
    render_hybrid_scene_camera(
        backend,
        background,
        camera,
        &view,
        scene,
        [0.0, 0.0, 0.0, 1.0],
    )
    .expect("render")
    .color
    .rgba8
}

fn write_png(path: &Path, rgba8: &[u8]) {
    let file = File::create(path).expect("png file");
    let mut encoder = Encoder::new(BufWriter::new(file), CAM_WIDTH, CAM_HEIGHT);
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
