//! A delivery robot drives a real street rebuilt from a 360-degree video, on ground truth.
//!
//! ```text
//! --street DIR       a visloc-rs street bundle (rne_bundle.py)
//! --out DIR          drive.csv, report.json and, unless --render-every 0, PNG frames
//! --render-every S   simulated seconds between rendered frames (default 0.5; 0: none)
//! --max-frames N     render at most N frames
//! ```
//!
//! The robot plans on the bundle's lane map with `rne_nav` and follows the plan by pure
//! pursuit (see the library). Each rendered frame has the robot's front camera (the
//! bundle's camera height, 90 degree field of view) and a chase view 3.2 m behind and
//! 2.75 m up, with the robot drawn as a box the size of the delivery robot.

use png::{BitDepth, ColorType, Encoder};
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{
    validate_gaussian_splat_manifest, Camera, HybridRenderScene, RenderScene, RenderSceneItem,
    VisualShape,
};
use rne_render_3dgs::{load_gaussian_splat_background, render_hybrid_scene_camera};
use rne_render_wgpu::WgpuRenderBackend;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use street_splat_nav::{drive, map_to_rne, NavConfig, RobotState, Street};

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

    let mut renderer = (render_every > 0.0).then(|| Renderer::new(&street));
    let mut log = BufWriter::new(File::create(out.join("drive.csv")).expect("drive.csv"));
    writeln!(log, "t_s,x,y,z,yaw,pitch,speed,goal").unwrap();
    let mut next_frame = 0.0;
    let mut frame = 0usize;
    let config = NavConfig::default();
    let report = drive(&street, &config, |s| {
        if ((s.t_s / config.step_s).round() as u64).is_multiple_of(5) {
            writeln!(
                log,
                "{:.2},{:.3},{:.3},{:.3},{:.4},{:.4},{:.3},{}",
                s.t_s, s.x_m, s.y_m, s.z_m, s.yaw_rad, s.pitch_rad, s.speed_m_s, s.goal
            )
            .unwrap();
        }
        if let Some(r) = renderer.as_mut() {
            if s.t_s + 1e-9 >= next_frame && frame < max_frames {
                r.render(&street, s, &out, frame);
                frame += 1;
                next_frame += render_every;
            }
        }
    })
    .expect("drive");
    log.flush().unwrap();
    let json = format!(
        "{{\"reached\": {}, \"goals\": {}, \"contacts\": {}, \"time_s\": {:.1}, \"distance_m\": {:.1}, \
         \"kerb_time_s\": {:.1}, \"kerb_max_speed_m_s\": {:.3}, \"min_solid_clearance_m\": {:.2}, \
         \"walk_distance_median_m\": {:.2}, \"walk_distance_max_m\": {:.2}, \"frames\": {frame}, \
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
        report.trajectory_hash
    );
    fs::write(out.join("report.json"), &json).expect("report.json");
    print!("{json}");
}

struct Renderer {
    backend: WgpuRenderBackend,
    environment: rne_render::GaussianSplatEnvironment,
    background: rne_render_3dgs::GaussianSplatBackground,
    front: Camera,
    chase: Camera,
}

impl Renderer {
    fn new(street: &Street) -> Self {
        let backend = WgpuRenderBackend::new().expect("wgpu backend");
        let environment =
            validate_gaussian_splat_manifest(&street.manifest()).expect("splat manifest");
        let background = load_gaussian_splat_background(backend.device(), &environment)
            .expect("splat background");
        Self {
            backend,
            environment,
            background,
            front: Camera::new(CAM_SIZE, CAM_SIZE, std::f64::consts::FRAC_PI_2),
            chase: Camera::new(CHASE_WIDTH, CHASE_HEIGHT, 60f64.to_radians()),
        }
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
