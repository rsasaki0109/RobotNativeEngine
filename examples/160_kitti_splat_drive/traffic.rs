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
//! --segments LIST         instead of --manifest: one splat per stretch of the route
//!                         (`start_frame, manifest` lines, as for examples 160/161); the
//!                         actors move with the ego from splat to splat
//! --actor PLY             car splat in its own frame (+x forward, +y left, +z up, origin at
//!                         the bottom centre)
//! --path CSV              t_s, x, y, z, yaw (RNE frame, OXTS unit above the road)
//! --out DIR               front_NNNN.png and traffic.csv
//! [--render-every 0.2] [--duration 40] [--lead-start 22] [--stop-at 14]
//! [--cut-in S]            a second car parked at path arc length S, off to the right, pulls
//!                         out in front of the ego when the ego comes within 20 m
//! [--cut-in-actor PLY]    car splat for the cut-in car (default: the --actor car)
//! [--ego-actor PLY]       draw the ego car too, as this car splat
//! [--chase BACK,UP,PITCH] also render a third-person view: BACK m behind the ego along the
//!                         path, UP m above its camera height, pitched down PITCH degrees
//!                         (chase_NNNN.png)
//! ```

use kitti_splat_drive::{
    arc_length_near, arc_lengths, cut_in_lateral, parse_path, parse_segments, pose_at, segment_for,
    Idm, Segment, CUT_IN_TRIGGER_M, PARKED_OFFSET_M,
};
use png::{BitDepth, ColorType, Encoder};
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{validate_gaussian_splat_manifest, Camera, HybridRenderScene, RenderScene};
use rne_render_3dgs::{
    load_splat_actor, render_hybrid_scene_camera, GaussianSplatBackground, SplatActorAsset,
};
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
    let segments = load_segments();
    let actor_path = PathBuf::from(arg("--actor").expect("--actor"));
    let out = PathBuf::from(arg("--out").unwrap_or_else(|| "kitti_splat_traffic".into()));
    let render_every: f64 = arg("--render-every").map_or(0.2, |v| v.parse().unwrap());
    let duration: f64 = arg("--duration").map_or(40.0, |v| v.parse().unwrap());
    let lead_start: f64 = arg("--lead-start").map_or(22.0, |v| v.parse().unwrap());
    let stop_at: f64 = arg("--stop-at").map_or(14.0, |v| v.parse().unwrap());
    let cut_in: Option<f64> = arg("--cut-in").map(|v| v.parse().unwrap());
    let ego_actor = arg("--ego-actor").map(|p| load_splat_actor(Path::new(&p)).expect("ego actor"));
    let chase: Option<[f64; 3]> = arg("--chase").map(|v| {
        let parts: Vec<f64> = v
            .split(',')
            .map(|x| x.trim().parse().expect("--chase"))
            .collect();
        [parts[0], parts[1], parts[2]]
    });
    fs::create_dir_all(&out).expect("output directory");

    let path = parse_path(&fs::read_to_string(arg("--path").expect("--path")).expect("path"))
        .expect("path");
    let points = path.points.clone();
    let lengths = arc_lengths(&points);

    let mut backend = WgpuRenderBackend::new().expect("wgpu backend");
    let cut_in_actor = cut_in.map(|_| {
        arg("--cut-in-actor").map_or_else(
            || load_splat_actor(&actor_path).expect("actor"),
            |p| load_splat_actor(Path::new(&p)).expect("cut-in actor"),
        )
    });
    let actors = actor_list(&actor_path, cut_in_actor, ego_actor.clone());
    // The splat for a stretch of the route, with the same actors in it.
    let load = |backend: &WgpuRenderBackend, manifest: &Path| {
        let environment = validate_gaussian_splat_manifest(manifest).expect("manifest");
        println!("splat: {}", environment.ply_path.display());
        let background = GaussianSplatBackground::from_environment_with_actors(
            backend.device(),
            &environment,
            actors.clone(),
        )
        .expect("splat");
        let scene = HybridRenderScene::new(environment, RenderScene::new());
        (background, scene)
    };
    let segment_of = |index: usize| segment_for(&segments, path.frames[index]);
    let mut segment = segment_of(0);
    let (mut background, mut scene) = load(&backend, &segments[segment].manifest);
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
    writeln!(
        log,
        "t_s,frame,ego_s,ego_v,ego_a,lead_s,lead_v,cut_s,cut_lat_m,cut_v,leader,gap_m"
    )
    .unwrap();

    let dt = 1.0 / SIM_HZ;
    let (mut ego_s, mut ego_v, mut lead_s) = (0.0, 6.0, lead_start);
    // The parked car: arc length, speed, and when it started to pull out.
    let (mut cut_s, mut cut_v, mut cut_start) = (cut_in.unwrap_or(0.0), 0.0, None::<f64>);
    let mut hint = 0usize;
    let (mut t, mut next, mut frame) = (0.0, 0.0, 0usize);
    let end = *lengths.last().unwrap() - 5.0;
    while t <= duration && ego_s < end {
        let lead_v = lead_speed(t, stop_at);
        if cut_in.is_some() && cut_start.is_none() && cut_s - ego_s < CUT_IN_TRIGGER_M {
            cut_start = Some(t);
        }
        let (cut_lat, cut_rate) = match cut_start {
            Some(t0) => cut_in_lateral(t - t0),
            None => (PARKED_OFFSET_M, 0.0),
        };
        // The leader: the nearest car ahead that is in the ego's lane.
        let mut leader = (lead_s, lead_v, 0usize);
        if cut_in.is_some() && cut_lat < 1.3 && cut_s > ego_s && cut_s < lead_s {
            leader = (cut_s, cut_v, 1);
        }
        let gap = leader.0 - ego_s - CAR_LENGTH_M;
        let accel = idm.accel(ego_v, Some(gap), leader.1).max(-8.0);
        if t + 1e-9 >= next {
            next += render_every;
            let (ego_p, ego_yaw) = pose_at(&points, &lengths, ego_s);
            // Keep the projection consistent with the path the actors are placed on.
            let (_, h) = arc_length_near(&points, &lengths, hint, ego_p);
            hint = h;
            if segment_of(hint) != segment {
                segment = segment_of(hint);
                (background, scene) = load(&backend, &segments[segment].manifest);
            }
            let lead_pose = car_pose(&points, &lengths, lead_s, 0.0, 0.0);
            background
                .set_actor_pose(backend.queue(), 0, Some(&lead_pose))
                .expect("actor pose");
            if cut_in.is_some() {
                // Heading turns into the lane while the car moves sideways.
                let yaw_offset = (-cut_rate).atan2(cut_v.max(0.5));
                let pose = car_pose(&points, &lengths, cut_s, cut_lat, yaw_offset);
                background
                    .set_actor_pose(backend.queue(), 1, Some(&pose))
                    .expect("actor pose");
            }
            if ego_actor.is_some() {
                let ego_index = background.actor_count() - 1;
                let pose = car_pose(&points, &lengths, ego_s, 0.0, 0.0);
                background
                    .set_actor_pose(backend.queue(), ego_index, Some(&pose))
                    .expect("ego pose");
            }
            let image = render_front(
                &mut backend,
                &mut background,
                &camera,
                &scene,
                ego_p,
                ego_yaw,
            );
            write_png(&out.join(format!("front_{frame:04}.png")), &image);
            if let Some(chase) = chase {
                let image = render_chase(
                    &mut backend,
                    &mut background,
                    &camera,
                    &scene,
                    pose_at(&points, &lengths, (ego_s - chase[0]).max(0.0)),
                    chase,
                );
                write_png(&out.join(format!("chase_{frame:04}.png")), &image);
            }
            writeln!(
                log,
                "{t:.2},{frame},{ego_s:.3},{ego_v:.3},{accel:.3},{lead_s:.3},{lead_v:.3},\
                 {cut_s:.3},{cut_lat:.3},{cut_v:.3},{},{gap:.3}",
                leader.2
            )
            .unwrap();
            frame += 1;
        }
        ego_v = (ego_v + accel * dt).max(0.0);
        ego_s += ego_v * dt;
        lead_s += lead_v * dt;
        if cut_start.is_some() {
            cut_v = (cut_v + 2.0 * dt).min(6.5);
            cut_s += cut_v * dt;
        }
        t += dt;
    }
    println!("{frame} frames, {t:.1} s -> {}", out.display());
}

/// Pose of a car actor at arc length `s`, `lateral` metres right of the path, its heading
/// turned left by `yaw_offset`. Actor frame (+x forward, +z up) -> RNE car frame (+y up).
fn car_pose(points: &[Vec3], lengths: &[f64], s: f64, lateral: f64, yaw_offset: f64) -> Transform3 {
    let (p, yaw) = pose_at(points, lengths, s);
    let right = Quat::from_rotation_y(yaw) * Vec3::Z;
    Transform3::from_translation_rotation(
        p + right * lateral - Vec3::new(0.0, BASE_HEIGHT_M, 0.0),
        Quat::from_rotation_y(yaw + yaw_offset) * Quat::from_rotation_x(-FRAC_PI_2),
    )
}

/// `--segments LIST`, or one segment from `--manifest`.
fn load_segments() -> Vec<Segment> {
    match arg("--segments") {
        Some(list) => {
            let list = PathBuf::from(list);
            let base = list.parent().unwrap_or(Path::new(".")).to_path_buf();
            parse_segments(&fs::read_to_string(&list).expect("segments"), &base).expect("segments")
        }
        None => vec![Segment {
            start_frame: 0,
            manifest: PathBuf::from(arg("--manifest").expect("--manifest or --segments")),
            map: None,
        }],
    }
}

/// Lead car, then the cut-in car (if any), then the ego car (if drawn).
fn actor_list(
    path: &Path,
    cut_in: Option<SplatActorAsset>,
    ego: Option<SplatActorAsset>,
) -> Vec<SplatActorAsset> {
    let actor = load_splat_actor(path).expect("actor");
    println!("actor: {} Gaussians", actor.len());
    let mut actors = vec![actor];
    actors.extend(cut_in);
    actors.extend(ego);
    actors
}

/// Third-person view from the path point `(position, yaw)` behind the ego, raised by `up` and
/// pitched down `pitch` degrees (`chase = [back, up, pitch]`).
fn render_chase(
    backend: &mut WgpuRenderBackend,
    background: &mut GaussianSplatBackground,
    camera: &Camera,
    scene: &HybridRenderScene,
    (position, yaw): (Vec3, f64),
    [_, up, pitch]: [f64; 3],
) -> Vec<u8> {
    let rotation = Quat::from_rotation_y(yaw);
    let mount = Vec3::new(CAM_IN_BASE[0], CAM_IN_BASE[2] + up, -CAM_IN_BASE[1]);
    let view = Transform3::from_translation_rotation(
        position + rotation * mount,
        (rotation * Quat::from_rotation_y(-FRAC_PI_2) * Quat::from_rotation_x(-pitch.to_radians()))
            .normalize(),
    );
    render_hybrid_scene_camera(
        backend,
        background,
        camera,
        &view,
        scene,
        [0.0, 0.0, 0.0, 1.0],
    )
    .expect("chase render")
    .color
    .rgba8
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
