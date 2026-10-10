//! Measured recovery under a common finite disturbance with exact observed replay.
//!
//! Controllers receive plant observations, never the disturbance schedule. The
//! animated human does not participate in physics, and robot poses are not edited.

use super::{
    disturbance::Impact, g1_controller::G1Controller, go2_controller::Go2Controller, model,
};
use rne_ai::{build_visual_render_scene, UrdfSceneSim};
use rne_core::SimDuration;
use rne_math::Vec3;
use rne_physics::{RigidBody, RigidBodyType};
use rne_robot::{Joint, JointKind, Link};
use rne_world::world_transform_of;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{error::Error, fs, io, path::Path};

const FRAME_COUNT: usize = 187;
const FRAME_RATE_HZ: f64 = 30.0;
const SETTLE_S: f64 = 2.0;
const DURATION_S: f64 = 0.08;
pub(super) const DEFAULT_IMPULSE_NS: f64 = 40.0;
const PUSH_OFFSET_Y_M: f64 = 0.06;
const SUPPORT_WINDOW_S: f64 = 0.5;
const SUPPORT_MEAN_LOAD_THRESHOLD_N: f64 = 0.5;
const GO2_FEET: [&str; 4] = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"];
const G1_FEET: [&str; 2] = ["left_ankle_roll_link", "right_ankle_roll_link"];
type CaptureResult<T> = Result<T, Box<dyn Error>>;

#[derive(Clone, Copy, Debug)]
enum Robot {
    Go2,
    G1,
}

impl Robot {
    fn name(self) -> &'static str {
        match self {
            Self::Go2 => "Go2",
            Self::G1 => "G1",
        }
    }
    fn base_link(self) -> &'static str {
        match self {
            Self::Go2 => "base",
            Self::G1 => "pelvis",
        }
    }
    fn feet(self) -> &'static [&'static str] {
        match self {
            Self::Go2 => &GO2_FEET,
            Self::G1 => &G1_FEET,
        }
    }
    fn minimum_height_m(self) -> f64 {
        match self {
            Self::Go2 => 0.14,
            Self::G1 => 0.50,
        }
    }
    fn recovered_height_m(self) -> f64 {
        match self {
            Self::Go2 => 0.22,
            Self::G1 => 0.70,
        }
    }
    fn load(self, dt_ticks: u64) -> CaptureResult<UrdfSceneSim> {
        let name = format!("{}.rne.scene.toml", self.name().to_lowercase());
        Ok(
            UrdfSceneSim::from_scene_path_with_solver_iterations_and_fixed_delta(
                &Path::new(env!("CARGO_MANIFEST_DIR")).join(name),
                32,
                SimDuration::from_ticks(dt_ticks),
            )?,
        )
    }
    fn validate(self, sim: &UrdfSceneSim) -> CaptureResult<Value> {
        model::validate_plant(
            sim,
            match self {
                Self::Go2 => 16.087000011,
                Self::G1 => 34.133857284,
            },
            match self {
                Self::Go2 => 1,
                Self::G1 => 4,
            },
            self.feet(),
        )
    }
    fn tilt_rad(self, sim: &UrdfSceneSim) -> CaptureResult<f64> {
        let pose = sim
            .named_transform(self.base_link())
            .ok_or_else(|| io::Error::other("missing robot root"))?;
        Ok((pose.rotation * Vec3::Z).y.clamp(-1.0, 1.0).acos())
    }
    fn height_m(self, sim: &UrdfSceneSim) -> CaptureResult<f64> {
        Ok(sim
            .named_link_position_m(self.base_link())
            .ok_or_else(|| io::Error::other("missing robot root"))?
            .y)
    }
}

#[derive(Debug)]
enum Controller {
    Go2(Go2Controller),
    G1(G1Controller),
}
impl Controller {
    fn new(robot: Robot, sim: &mut UrdfSceneSim, enabled: bool) -> CaptureResult<Self> {
        Ok(match robot {
            Robot::Go2 => Self::Go2(Go2Controller::new(sim, enabled)?),
            Robot::G1 => Self::G1(G1Controller::new(sim, enabled)?),
        })
    }
    fn step(&mut self, sim: &mut UrdfSceneSim) -> CaptureResult<()> {
        match self {
            Self::Go2(c) => c.step(sim),
            Self::G1(c) => c.step(sim),
        }
    }
    fn telemetry(&self) -> Value {
        match self {
            Self::Go2(c) => c.telemetry(),
            Self::G1(c) => c.telemetry(),
        }
    }
    fn configuration(&self) -> Value {
        match self {
            Self::Go2(c) => c.configuration(),
            Self::G1(c) => c.configuration(),
        }
    }
}

fn require(condition: bool, message: &str) -> CaptureResult<()> {
    if condition {
        Ok(())
    } else {
        Err(io::Error::other(message).into())
    }
}

fn contact_feet(robot: Robot, sim: &UrdfSceneSim) -> usize {
    robot
        .feet()
        .iter()
        .filter(|foot| sim.link_contact_impulse_ns(foot) > 0.0)
        .count()
}

/// Stable link-name ordering; compare literal IEEE bits before reporting hashes.
fn state_bits(robot: Robot, sim: &UrdfSceneSim) -> Vec<u64> {
    let mut links: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<Link>().map(|l| (l.name.as_str(), e.id())))
        .collect();
    links.sort_unstable_by(|a, b| a.0.cmp(b.0));
    let mut bits = Vec::new();
    for (name, entity) in links {
        let transform = world_transform_of(sim.world(), entity);
        bits.extend(
            transform
                .translation
                .to_array()
                .into_iter()
                .chain(transform.rotation.to_array())
                .chain(transform.scale.to_array())
                .map(f64::to_bits),
        );
        if let Some(body) = sim.world().get::<RigidBody>(entity) {
            bits.push(1);
            bits.extend(
                body.linear_velocity_m_s
                    .to_array()
                    .into_iter()
                    .chain(body.angular_velocity_rad_s.to_array())
                    .map(f64::to_bits),
            );
        } else {
            bits.push(0);
        }
        for value in [
            sim.named_joint_position(name),
            sim.named_joint_velocity(name),
        ] {
            if let Some(value) = value {
                bits.extend([1, value.to_bits()]);
            } else {
                bits.push(0);
            }
        }
    }
    bits.extend(
        robot
            .feet()
            .iter()
            .map(|foot| sim.link_contact_impulse_ns(foot).to_bits()),
    );
    bits
}

fn stable_hash(states: &[Vec<u64>]) -> String {
    let hash = states
        .iter()
        .flatten()
        .flat_map(|bits| bits.to_le_bytes())
        .fold(0xcbf2_9ce4_8422_2325_u64, |hash, byte| {
            (hash ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3)
        });
    format!("{hash:016x}")
}

/// Identifies the control and plant inputs present when this binary was built.
fn compiled_source_hashes() -> Value {
    let files = [
        (
            "examples/139_kick_comparison/main.rs",
            include_str!("main.rs"),
        ),
        (
            "examples/139_kick_comparison/physics.rs",
            include_str!("physics.rs"),
        ),
        (
            "examples/139_kick_comparison/render.rs",
            include_str!("render.rs"),
        ),
        (
            "examples/139_kick_comparison/model.rs",
            include_str!("model.rs"),
        ),
        (
            "examples/139_kick_comparison/disturbance.rs",
            include_str!("disturbance.rs"),
        ),
        (
            "examples/139_kick_comparison/go2_controller.rs",
            include_str!("go2_controller.rs"),
        ),
        (
            "examples/139_kick_comparison/g1_controller.rs",
            include_str!("g1_controller.rs"),
        ),
        (
            "examples/139_kick_comparison/go2.rne.robot.toml",
            include_str!("go2.rne.robot.toml"),
        ),
        (
            "examples/139_kick_comparison/g1.rne.robot.toml",
            include_str!("g1.rne.robot.toml"),
        ),
        (
            "examples/139_kick_comparison/go2.rne.scene.toml",
            include_str!("go2.rne.scene.toml"),
        ),
        (
            "examples/139_kick_comparison/g1.rne.scene.toml",
            include_str!("g1.rne.scene.toml"),
        ),
        (
            "examples/139_kick_comparison/models.json",
            include_str!("models.json"),
        ),
        (
            "assets/robots/go2_description/go2_description.rne.kick.urdf",
            include_str!("../../assets/robots/go2_description/go2_description.rne.kick.urdf"),
        ),
        (
            "assets/robots/g1_description/g1_23dof.kick.urdf",
            include_str!("../../assets/robots/g1_description/g1_23dof.kick.urdf"),
        ),
        (
            "crates/rne_physics_rapier/src/backend.rs",
            include_str!("../../crates/rne_physics_rapier/src/backend.rs"),
        ),
    ];
    let mut manifest: serde_json::Map<String, Value> = files
        .into_iter()
        .map(|(name, contents)| {
            (
                name.to_owned(),
                json!(format!("{:x}", Sha256::digest(contents.as_bytes()))),
            )
        })
        .collect();
    manifest.insert(
        "assets/fixtures/kick_human/cc0_sport_human.glb".to_owned(),
        json!(format!(
            "{:x}",
            Sha256::digest(include_bytes!(
                "../../assets/fixtures/kick_human/cc0_sport_human.glb"
            ))
        )),
    );
    Value::Object(manifest)
}

fn frame(sim: &UrdfSceneSim, step: usize, force: Vec3, point: Option<Vec3>) -> Value {
    let mut items: Vec<_> = build_visual_render_scene(sim.world()).items.into_iter().map(|item|
        json!({"shape":item.shape,"color_rgba":item.color_rgba,
            "transform":{"translation":item.transform.translation.to_array(),
                "rotation":item.transform.rotation.to_array(),"scale":item.transform.scale.to_array()}})).collect();
    items.sort_by_cached_key(Value::to_string);
    json!({"step":step,"time_s":step as f64 * sim.fixed_delta().as_seconds().value(),
        "force_n":force.length(),"force_world_n":force.to_array(),
        "application_point_world_m":point.map(|p|p.to_array()),"items":items})
}

#[derive(Clone, Copy, Debug)]
struct Case {
    name: &'static str,
    dt_ticks: u64,
    impulse_ns: f64,
    start_s: f64,
    direction: Vec3,
    balance_feedback_enabled: bool,
}
const NOMINAL: Case = Case {
    name: "nominal",
    dt_ticks: 1_000_000,
    impulse_ns: DEFAULT_IMPULSE_NS,
    start_s: 2.0,
    direction: Vec3::Z,
    balance_feedback_enabled: true,
};

#[derive(Debug)]
struct Recording {
    trace: Value,
    state_bits: Vec<Vec<u64>>,
}

fn ground_body_index(sim: &UrdfSceneSim) -> CaptureResult<u32> {
    let grounds: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|e| {
            let body = e.get::<RigidBody>()?;
            (body.body_type == RigidBodyType::Fixed && e.get::<Link>().is_none()).then_some(e.id())
        })
        .collect();
    require(
        grounds.len() == 1,
        "the recovery scene requires exactly one fixed ground body",
    )?;
    Ok(grounds[0].index())
}

fn settle(
    robot: Robot,
    sim: &mut UrdfSceneSim,
    controller: &mut Controller,
    states: &mut Vec<Vec<u64>>,
) -> CaptureResult<(Value, f64, f64)> {
    let dt_s = sim.fixed_delta().as_seconds().value();
    let mut max_fixed_position_error_m: f64 = 0.0;
    let mut max_fixed_rotation_error_rad: f64 = 0.0;
    for _ in 0..(SETTLE_S / dt_s).round() as usize {
        controller.step(sim)?;
        let (position, rotation) = model::fixed_joint_error(sim)?;
        max_fixed_position_error_m = max_fixed_position_error_m.max(position);
        max_fixed_rotation_error_rad = max_fixed_rotation_error_rad.max(rotation);
        states.push(state_bits(robot, sim));
    }
    let plant = robot.validate(sim)?;
    require(
        robot.tilt_rad(sim)? < 0.15 && robot.height_m(sim)? > robot.minimum_height_m(),
        "the physical plant did not settle upright before the disturbance",
    )?;
    Ok((
        plant,
        max_fixed_position_error_m,
        max_fixed_rotation_error_rad,
    ))
}

#[derive(Debug)]
struct Metrics {
    peak_tilt_rad: f64,
    min_height_m: f64,
    ever_fallen: bool,
    nonfoot_ground_contact: bool,
    nonfoot_ground_links: std::collections::BTreeSet<String>,
    min_contact_feet: usize,
    max_authored_speed_ratio: f64,
    max_fixed_position_error_m: f64,
    max_fixed_rotation_error_rad: f64,
    final_support_impulse_ns: Vec<f64>,
}

impl Metrics {
    fn new(robot: Robot, sim: &UrdfSceneSim, closure: (f64, f64)) -> CaptureResult<Self> {
        Ok(Self {
            peak_tilt_rad: 0.0,
            min_height_m: robot.height_m(sim)?,
            ever_fallen: false,
            nonfoot_ground_contact: false,
            nonfoot_ground_links: std::collections::BTreeSet::new(),
            min_contact_feet: robot.feet().len(),
            max_authored_speed_ratio: 0.0,
            max_fixed_position_error_m: closure.0,
            max_fixed_rotation_error_rad: closure.1,
            final_support_impulse_ns: vec![0.0; robot.feet().len()],
        })
    }

    fn measure(
        &mut self,
        robot: Robot,
        sim: &UrdfSceneSim,
        step: usize,
        steps: usize,
        ground_index: u32,
    ) -> CaptureResult<(f64, f64)> {
        let dt_s = sim.fixed_delta().as_seconds().value();
        let tilt = robot.tilt_rad(sim)?;
        let height = robot.height_m(sim)?;
        self.peak_tilt_rad = self.peak_tilt_rad.max(tilt);
        self.min_height_m = self.min_height_m.min(height);
        self.ever_fallen |= height < robot.minimum_height_m() || tilt > 1.1;
        self.min_contact_feet = self.min_contact_feet.min(contact_feet(robot, sim));
        let (position, rotation) = model::fixed_joint_error(sim)?;
        self.max_fixed_position_error_m = self.max_fixed_position_error_m.max(position);
        self.max_fixed_rotation_error_rad = self.max_fixed_rotation_error_rad.max(rotation);
        for contact in sim.physics_contact_events()? {
            require(
                contact.impulse.is_finite(),
                "a contact impulse is not finite",
            )?;
            if contact.impulse <= 1.0e-8 {
                continue;
            }
            let other = if contact.entity_a.index() == ground_index {
                Some(contact.entity_b)
            } else if contact.entity_b.index() == ground_index {
                Some(contact.entity_a)
            } else {
                None
            };
            if let Some(link) = other.and_then(|e| sim.world().get::<Link>(e)) {
                if !robot.feet().contains(&link.name.as_str()) {
                    self.nonfoot_ground_contact = true;
                    self.nonfoot_ground_links.insert(link.name.clone());
                }
            }
        }
        for entity in sim.world().iter_entities() {
            if let Some(joint) = entity.get::<Joint>() {
                if joint.kind != JointKind::Fixed && joint.limits.max_velocity > 0.0 {
                    let child = sim
                        .world()
                        .get::<Link>(joint.child_link)
                        .expect("joint child link");
                    let measured_velocity = sim
                        .named_joint_velocity(&child.name)
                        .ok_or_else(|| io::Error::other("a measured joint velocity is missing"))?;
                    require(
                        measured_velocity.is_finite(),
                        "a joint velocity is not finite",
                    )?;
                    self.max_authored_speed_ratio = self
                        .max_authored_speed_ratio
                        .max(measured_velocity.abs() / joint.limits.max_velocity);
                }
            }
        }
        if step > steps - (SUPPORT_WINDOW_S / dt_s).round() as usize {
            for (index, foot) in robot.feet().iter().enumerate() {
                self.final_support_impulse_ns[index] += sim.link_contact_impulse_ns(foot);
            }
        }
        Ok((tilt, height))
    }

    fn summary(
        &self,
        robot: Robot,
        sim: &UrdfSceneSim,
        case: Case,
        initial_root: Vec3,
    ) -> CaptureResult<Value> {
        let final_tilt_rad = robot.tilt_rad(sim)?;
        let final_height_m = robot.height_m(sim)?;
        let (_, velocity, _) = model::center_of_mass(sim)?;
        let final_speed_m_s = velocity.length();
        let final_contact_feet = contact_feet(robot, sim);
        let final_support_mean_loads_n: Vec<_> = self
            .final_support_impulse_ns
            .iter()
            .map(|j| j / SUPPORT_WINDOW_S)
            .collect();
        let final_support_feet = final_support_mean_loads_n
            .iter()
            .filter(|f| **f > SUPPORT_MEAN_LOAD_THRESHOLD_N)
            .count();
        let fixed_attachments_preserved =
            self.max_fixed_position_error_m <= 1e-4 && self.max_fixed_rotation_error_rad <= 5e-4;
        let recovered = !self.ever_fallen
            && !self.nonfoot_ground_contact
            && fixed_attachments_preserved
            && final_height_m > robot.recovered_height_m()
            && final_tilt_rad < 0.20
            && final_speed_m_s < 0.10
            && final_support_feet == robot.feet().len();
        let summary = json!({"recovered":recovered,"peak_tilt_rad":self.peak_tilt_rad,"final_tilt_rad":final_tilt_rad,
        "residual_tilt_rad":final_tilt_rad,"final_base_height_m":final_height_m,
        "final_pelvis_height_m":final_height_m,"final_speed_m_s":final_speed_m_s,"min_height_m":self.min_height_m,
        "lateral_displacement_m":sim.named_link_position_m(robot.base_link()).expect("root").z-initial_root.z,
        "final_contact_feet":final_contact_feet,"min_contact_feet":self.min_contact_feet,
        "final_support_window_s":SUPPORT_WINDOW_S,"final_support_mean_loads_n":final_support_mean_loads_n,
        "final_support_feet":final_support_feet,"support_mean_load_threshold_n":SUPPORT_MEAN_LOAD_THRESHOLD_N,
        "ever_fallen":self.ever_fallen,"nonfoot_ground_contact":self.nonfoot_ground_contact,
        "fixed_attachments_preserved":fixed_attachments_preserved,
        "nonfoot_ground_links":self.nonfoot_ground_links,
        "max_fixed_position_error_m":self.max_fixed_position_error_m,"max_fixed_rotation_error_rad":self.max_fixed_rotation_error_rad,
        "max_authored_speed_ratio":self.max_authored_speed_ratio});
        println!(
            "{} {} feedback={} recovered={} peak={:.6}rad min_height={:.6}m final_speed={:.6}m/s",
            robot.name(),
            case.name,
            case.balance_feedback_enabled,
            recovered,
            self.peak_tilt_rad,
            self.min_height_m,
            final_speed_m_s
        );
        Ok(summary)
    }
}

fn measured_telemetry(
    robot: Robot,
    sim: &UrdfSceneSim,
    controller: &Controller,
    time_s: f64,
    tilt: f64,
    height: f64,
) -> CaptureResult<Value> {
    let (com, velocity, _) = model::center_of_mass(sim)?;
    let feet: Vec<_> = robot
        .feet()
        .iter()
        .map(|name| {
            json!({"link":name,
                "normal_impulse_ns":sim.link_contact_impulse_ns(name),
                "position_world_m":sim.named_link_position_m(name).map(|p|p.to_array())})
        })
        .collect();
    Ok(json!({"time_s":time_s,"com_world_m":com.to_array(),
                "com_velocity_world_m_s":velocity.to_array(),"root_height_m":height,"tilt_rad":tilt,
                "feet":feet,"controller":controller.telemetry()}))
}

fn record(robot: Robot, case: Case, capture_frames: bool) -> CaptureResult<Recording> {
    let mut sim = robot.load(case.dt_ticks)?;
    robot.validate(&sim)?;
    let mut controller = Controller::new(robot, &mut sim, case.balance_feedback_enabled)?;
    let dt_s = sim.fixed_delta().as_seconds().value();
    let mut states = Vec::new();
    let ground_index = ground_body_index(&sim)?;
    let (plant, initial_fixed_position_error_m, initial_fixed_rotation_error_rad) =
        settle(robot, &mut sim, &mut controller, &mut states)?;
    let initial_root = sim
        .named_link_position_m(robot.base_link())
        .expect("validated root");
    let impact = Impact::new(
        (case.start_s / dt_s).round() as usize + 1,
        (DURATION_S / dt_s).round() as usize,
        dt_s,
        case.impulse_ns,
        case.direction,
    );
    let mut frames = Vec::new();
    if capture_frames {
        frames.push(frame(&sim, 0, Vec3::ZERO, None));
    }
    let mut force_history = Vec::new();
    let mut telemetry = Vec::new();
    let mut metrics = Metrics::new(
        robot,
        &sim,
        (
            initial_fixed_position_error_m,
            initial_fixed_rotation_error_rad,
        ),
    )?;
    let steps = (((FRAME_COUNT - 1) as f64 / FRAME_RATE_HZ) / dt_s).round() as usize;
    for step in 1..=steps {
        let force = impact.force_world_n(step);
        let point = if force.length_squared() > 0.0 {
            let point = sim
                .named_link_position_m(robot.base_link())
                .expect("validated root")
                + Vec3::Y * PUSH_OFFSET_Y_M;
            require(
                sim.apply_named_link_wrench(robot.base_link(), point, force, Vec3::ZERO),
                "the dynamic root rejected the disturbance wrench",
            )?;
            Some(point)
        } else {
            None
        };
        controller.step(&mut sim)?;
        let (tilt, height) = metrics.measure(robot, &sim, step, steps, ground_index)?;
        states.push(state_bits(robot, &sim));
        force_history.push(json!({"step":step,"time_s":step as f64*dt_s,"dt_s":dt_s,
            "force_world_n":force.to_array(),"application_point_world_m":point.map(|p|p.to_array())}));
        if step % (0.02 / dt_s).round() as usize == 0 {
            telemetry.push(measured_telemetry(
                robot,
                &sim,
                &controller,
                step as f64 * dt_s,
                tilt,
                height,
            )?);
        }
        if capture_frames
            && frames.len() < FRAME_COUNT
            && step == (frames.len() as f64 / FRAME_RATE_HZ / dt_s).round() as usize
        {
            frames.push(frame(&sim, step, force, point));
        }
    }
    let summary = metrics.summary(robot, &sim, case, initial_root)?;
    let roots: Vec<_> = sim
        .mesh_package_roots()
        .iter()
        .map(|p| p.to_string_lossy().into_owned())
        .collect();
    let hash = stable_hash(&states);
    let word_count: usize = states.iter().map(Vec::len).sum();
    Ok(Recording {
        trace: json!({"schema_version":2,"robot":robot.name(),"case":case.name,
        "compiled_source_sha256":compiled_source_hashes(),
        "backend":"Rapier through UrdfSceneSim","coordinate_system":"Y-up","dt_s":dt_s,
        "solver_iterations":32,"push_start_s":case.start_s,"push_duration_s":impact.duration_s(),
        "force_n":impact.peak_force_n(),"impulse_n_s":case.impulse_ns,
        "integrated_impulse_world_ns":impact.integrated_impulse_ns().to_array(),
        "force_link":robot.base_link(),"application_offset_m":[0.0,PUSH_OFFSET_Y_M,0.0],
        "pulse":"midpoint half-sine, normalized to the declared impulse",
        "plant":plant,"controller":controller.configuration(),"balance_feedback_enabled":case.balance_feedback_enabled,
        "mesh_package_roots":roots,"mechanism":"Prescribed external wrench; human is render-only; no edited robot poses",
        "summary":summary,"force_history":force_history,"telemetry":telemetry,"frames":frames,
        "observed_state_definition":"Link-name order: world pose/scale, body COM linear/angular velocity, joint q/qd with presence tags; fixed foot order normal impulse. Literal IEEE f64 bits, including settlement; excludes hidden solver caches.",
        "observed_state_hash":hash,"observed_state_word_count":word_count}),
        state_bits: states,
    })
}

fn replay(robot: Robot, case: Case, capture_frames: bool) -> CaptureResult<Value> {
    let mut first = record(robot, case, capture_frames)?;
    let repeat = record(robot, case, capture_frames)?;
    require(
        first.state_bits == repeat.state_bits,
        "the observed replay differs bit for bit",
    )?;
    require(
        first.trace["frames"] == repeat.trace["frames"],
        "the canonical render frames differ on replay",
    )?;
    first.trace["observed_state_bits_exact_repeat"] = json!(true);
    first.trace["canonical_render_frames_exact_repeat"] = json!(true);
    Ok(first.trace)
}

/// Captures both robots at 1 kHz under the same declared 80 ms lateral pulse.
pub(super) fn capture(output: &Path, impulse_ns: f64) -> CaptureResult<()> {
    fs::create_dir_all(output)?;
    let mut summaries = Vec::new();
    let mut all_recovered = true;
    for robot in [Robot::Go2, Robot::G1] {
        let trace = replay(
            robot,
            Case {
                impulse_ns,
                ..NOMINAL
            },
            true,
        )?;
        let filename = format!("{}-trace.json", robot.name().to_lowercase());
        fs::write(output.join(&filename), serde_json::to_vec(&trace)?)?;
        require(
            trace["frames"]
                .as_array()
                .is_some_and(|f| f.len() == FRAME_COUNT),
            "incorrect frame count",
        )?;
        require(
            trace["frames"]
                .as_array()
                .and_then(|f| f.iter().position(|v| v["force_n"] != 0.0))
                == Some(61),
            "the first force frame must be frame 61",
        )?;
        all_recovered &= trace["summary"]["recovered"] == true;
        summaries.push(json!({"robot":robot.name(),"trace":filename,"force_n":trace["force_n"],
            "dt_s":trace["dt_s"],"duration_s":trace["push_duration_s"],"impulse_n_s":trace["impulse_n_s"],
            "summary":trace["summary"],"plant":trace["plant"],"controller":trace["controller"],
            "observed_state_hash":trace["observed_state_hash"],"observed_state_bits_exact_repeat":true,
            "canonical_render_frames_exact_repeat":true}));
    }
    fs::write(
        output.join("summary.json"),
        serde_json::to_vec_pretty(&json!({"schema_version":2,
        "note":format!("Common {impulse_ns} N.s lateral disturbance; declared URDF inertials and attached feet; torque-bounded feedback. Human is visual, not a simulated collision or hardware validation."),
        "cases":summaries}))?,
    )?;
    require(
        all_recovered,
        "a robot did not meet the measured recovery gate; traces retained",
    )
}

/// Records replay, onset, direction, zero-input, rate, and matched ablation cases.
///
/// Sideways recovery is the declared envelope. Front/oblique impacts are retained
/// as observations of the limit, rather than silently included in that claim.
pub(super) fn validate_recovery(
    output: &Path,
    robot_filter: Option<&str>,
    impulse_ns: f64,
) -> CaptureResult<()> {
    fs::create_dir_all(output)?;
    let nominal = Case {
        impulse_ns,
        ..NOMINAL
    };
    let cases = [
        nominal,
        Case {
            name: "zero",
            impulse_ns: 0.0,
            ..nominal
        },
        Case {
            name: "reverse_early",
            direction: -Vec3::Z,
            start_s: 1.5,
            ..nominal
        },
        Case {
            name: "late",
            start_s: 2.25,
            ..nominal
        },
        Case {
            name: "rate_500hz",
            dt_ticks: 2_000_000,
            ..nominal
        },
        Case {
            name: "balance_off",
            balance_feedback_enabled: false,
            ..nominal
        },
        Case {
            name: "balance_off_zero",
            impulse_ns: 0.0,
            balance_feedback_enabled: false,
            ..nominal
        },
        Case {
            name: "front_limit",
            direction: Vec3::X,
            ..nominal
        },
        Case {
            name: "oblique_limit",
            direction: Vec3::new(1.0, 0.0, 1.0),
            ..nominal
        },
    ];
    let robots = selected_robots(robot_filter)?;
    let reports = record_case_matrix(robots, &cases, output, true)?;
    let all_passed = reports.iter().all(|report| {
        report["required_recovery"] != true || report["summary"]["recovered"] == true
    });
    fs::write(
        output.join("recovery-validation.json"),
        serde_json::to_vec_pretty(&json!({
        "required_cases_passed":all_passed,"trajectory_convergence_claimed":false,
        "robots":robots.iter().map(|r|r.name()).collect::<Vec<_>>(),"cases":reports}))?,
    )?;
    require(
        all_passed,
        "a required recovery case failed; all reports retained",
    )
}

/// Measures increasing lateral impulses without requiring the limit probes to recover.
///
/// Each strength uses the same plant, controller and 80 ms pulse, with exact replay.
pub(super) fn sweep_impulse(output: &Path, robot_filter: Option<&str>) -> CaptureResult<()> {
    fs::create_dir_all(output)?;
    let cases = [
        ("impulse_24", 24.0),
        ("impulse_32", 32.0),
        ("impulse_40", 40.0),
        ("impulse_48", 48.0),
        ("impulse_64", 64.0),
    ]
    .map(|(name, impulse_ns)| Case {
        name,
        impulse_ns,
        ..NOMINAL
    });
    let robots = selected_robots(robot_filter)?;
    let reports = record_case_matrix(robots, &cases, output, false)?;
    fs::write(
        output.join("impulse-sweep.json"),
        serde_json::to_vec_pretty(&json!({
            "schema_version":1,"pulse_duration_s":DURATION_S,
            "note":"Measured lateral limit probes, not an all-strength recovery claim",
            "robots":robots.iter().map(|r|r.name()).collect::<Vec<_>>(),"cases":reports
        }))?,
    )?;
    Ok(())
}

fn selected_robots(robot_filter: Option<&str>) -> CaptureResult<&'static [Robot]> {
    Ok(match robot_filter {
        None => &[Robot::Go2, Robot::G1],
        Some("go2") => &[Robot::Go2],
        Some("g1") => &[Robot::G1],
        Some(_) => return Err(io::Error::other("validation robot must be go2 or g1").into()),
    })
}

/// Records one selected lateral impulse with exact replay, including failed recovery.
pub(super) fn probe_impulse(
    output: &Path,
    robot_filter: Option<&str>,
    impulse_ns: f64,
    balance_feedback_enabled: bool,
) -> CaptureResult<()> {
    fs::create_dir_all(output)?;
    let robots = selected_robots(robot_filter)?;
    let reports = record_case_matrix(
        robots,
        &[Case {
            impulse_ns,
            balance_feedback_enabled,
            ..NOMINAL
        }],
        output,
        false,
    )?;
    fs::write(
        output.join("impulse-probe.json"),
        serde_json::to_vec_pretty(&json!({
            "schema_version":1,"cases":reports,
            "note":"A measured limit probe; recovery is reported, not assumed"
        }))?,
    )?;
    Ok(())
}

fn record_case_matrix(
    robots: &[Robot],
    cases: &[Case],
    output: &Path,
    require_recovery: bool,
) -> CaptureResult<Vec<Value>> {
    // Each worker owns an independent seeded world. Join in robot order so
    // report ordering is stable; each replay remains sequential within a worker.
    let results = std::thread::scope(|scope| {
        let handles: Vec<_> = robots
            .iter()
            .copied()
            .map(|robot| {
                let cases = &cases;
                scope.spawn(move || {
                    validate_robot_cases(robot, cases, output, require_recovery)
                        .map_err(|error| error.to_string())
                })
            })
            .collect();
        handles
            .into_iter()
            .map(|handle| {
                handle
                    .join()
                    .map_err(|_| io::Error::other("a recovery validation worker panicked"))?
                    .map_err(io::Error::other)
            })
            .collect::<Result<Vec<_>, _>>()
    })?;
    Ok(results.into_iter().flatten().collect())
}

fn validate_robot_cases(
    robot: Robot,
    cases: &[Case],
    output: &Path,
    require_recovery: bool,
) -> CaptureResult<Vec<Value>> {
    let mut reports = Vec::new();
    for &case in cases {
        let trace = replay(robot, case, false)?;
        fs::write(
            output.join(format!(
                "{}-{}.json",
                robot.name().to_lowercase(),
                case.name
            )),
            serde_json::to_vec(&trace)?,
        )?;
        let required = require_recovery
            && !matches!(case.name, "balance_off" | "front_limit" | "oblique_limit");
        reports.push(json!({"robot":robot.name(),"case":case.name,"required_recovery":required,
            "dt_s":trace["dt_s"],"impulse_n_s":trace["impulse_n_s"],"summary":trace["summary"],
            "push_start_s":trace["push_start_s"],"push_duration_s":trace["push_duration_s"],
            "integrated_impulse_world_ns":trace["integrated_impulse_world_ns"],
            "plant":trace["plant"],"controller":trace["controller"],
            "compiled_source_sha256":trace["compiled_source_sha256"],
            "observed_state_hash":trace["observed_state_hash"],"observed_state_bits_exact_repeat":true}));
    }
    Ok(reports)
}
