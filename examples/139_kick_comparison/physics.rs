//! Measured recovery under a common finite disturbance with exact observed replay.
//!
//! Controllers receive plant observations, never the disturbance schedule. The
//! animated human does not participate in physics, and robot poses are not edited.

use super::{
    diagnostic::{self, FinishedDiagnostics, RunDiagnostics},
    disturbance::Impact,
    feedback::{Estimator, Policy},
    g1_controller::G1Controller,
    go2_controller::Go2Controller,
    model,
    motor_diagnostics::{FinishedMotorDiagnostics, MotorRun},
    observation::{self, Pipeline, Profile},
};
use rne_ai::{build_visual_render_scene, UrdfSceneSim};
use rne_core::SimDuration;
use rne_math::Vec3;
use rne_physics::{RigidBody, RigidBodyType};
use rne_robot::{Joint, JointKind, Link};
use rne_world::world_transform_of;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs,
    io::{self, Write},
    path::Path,
};

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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ExperimentMode {
    OrientationAge,
    LateralGyro,
}

impl ExperimentMode {
    fn stem(self) -> &'static str {
        match self {
            Self::OrientationAge => "g1-orientation-age",
            Self::LateralGyro => "g1-lateral-gyro",
        }
    }

    fn policies(self, filter: Option<&str>) -> CaptureResult<Vec<Policy>> {
        match self {
            Self::OrientationAge => Policy::selected(filter),
            Self::LateralGyro => Policy::selected_lateral(filter),
        }
    }

    fn reference(self) -> Policy {
        match self {
            Self::OrientationAge => Policy::RawReference,
            Self::LateralGyro => Policy::OrientationKnownAge,
        }
    }

    fn candidate(self) -> Policy {
        match self {
            Self::OrientationAge => Policy::OrientationKnownAge,
            Self::LateralGyro => Policy::OrientationKnownAgeLateralGyroEma5ms,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct ExperimentFeedback {
    policy: Policy,
    mode: ExperimentMode,
}

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
enum Feedback {
    Go2(Go2Controller),
    G1(G1Controller),
}
impl Feedback {
    fn new(robot: Robot, sim: &mut UrdfSceneSim, enabled: bool) -> CaptureResult<Self> {
        Ok(match robot {
            Robot::Go2 => Self::Go2(Go2Controller::new(sim, enabled)?),
            Robot::G1 => Self::G1(G1Controller::new(sim, enabled)?),
        })
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
    fn replay_words(&self) -> Vec<u64> {
        match self {
            Self::Go2(c) => c.replay_words(),
            Self::G1(c) => c.replay_words(),
        }
    }
}

#[derive(Debug)]
struct Controller {
    feedback: Feedback,
    pipeline: Pipeline,
    robot: Robot,
    last_command_words: Vec<u64>,
    last_feedback: Value,
    diagnostics: Option<RunDiagnostics>,
    feedback_policy: Policy,
    orientation_age_experiment: bool,
    lateral_gyro_experiment: bool,
    estimator: Estimator,
}

impl Controller {
    fn new(
        robot: Robot,
        sim: &mut UrdfSceneSim,
        enabled: bool,
        profile: Profile,
    ) -> CaptureResult<Self> {
        Ok(Self {
            feedback: Feedback::new(robot, sim, enabled)?,
            pipeline: Pipeline::new(profile, sim)?,
            robot,
            last_command_words: Vec::new(),
            last_feedback: Value::Null,
            diagnostics: None,
            feedback_policy: Policy::default(),
            orientation_age_experiment: false,
            lateral_gyro_experiment: false,
            estimator: Estimator::default(),
        })
    }

    fn step(&mut self, sim: &mut UrdfSceneSim) -> CaptureResult<()> {
        self.pipeline
            .capture_if_due(sim, self.robot.base_link(), self.robot.feet())?;
        let arrived = self.pipeline.consume(sim.sim_time())?;
        let raw_estimate = arrived.as_ref().map(|frame| &frame.payload);
        let filter_before = self
            .lateral_gyro_experiment
            .then(|| self.estimator.diagnostic_state());
        let effective =
            self.estimator
                .effective(self.feedback_policy, arrived.as_ref(), sim.sim_time())?;
        let estimate = effective.as_deref();
        // This extra capture is evaluator-only and exists solely with logging on.
        // The feedback call below continues to receive only the arrived estimate.
        let diagnostic_truth = if self
            .diagnostics
            .as_ref()
            .is_some_and(|run| run.writer.is_some())
        {
            Some(diagnostic::evaluator_snapshot(sim)?)
        } else {
            None
        };
        let diagnostic_estimate = if diagnostic_truth.is_some() {
            raw_estimate
                .map(|estimate| estimate.diagnostic_fields())
                .unwrap_or(Value::Null)
        } else {
            Value::Null
        };
        let mut diagnostic_feedback = (diagnostic_truth.is_some() && self.orientation_age_experiment).then(||
            json!({"g1_orientation_age_experiment":true,"feedback_policy":self.feedback_policy.name(),
                "effective_feedback_estimate":estimate.map(|estimate|estimate.diagnostic_fields()),
                "feedback_policy_configuration":self.feedback_policy.configuration(),
                "feedback_channel_time_scope":{
                    "raw_capture_ticks":arrived.as_ref().map(|frame|frame.capture_time.ticks()),
                    "consumer_ticks":sim.sim_time().ticks(),
                    "orientation_reference_ticks":arrived.as_ref().map(|frame|
                        if self.feedback_policy != Policy::RawReference {sim.sim_time().ticks()} else {frame.capture_time.ticks()}),
                    "non_orientation_channel_ticks":arrived.as_ref().map(|frame|frame.capture_time.ticks()),
                    "orientation_is_fresh_measurement":false,"captured_gyro_and_roll_rate_unchanged":true
                }}));
        let dt_s = sim.fixed_delta().as_seconds().value();
        // Contact points are driver-only truth telemetry, never decision inputs.
        let support_points: Vec<_> = self
            .robot
            .feet()
            .iter()
            .flat_map(|name| sim.named_body_contact_points_m(name))
            .map(|point| point.to_array())
            .collect();
        let targets = match &mut self.feedback {
            Feedback::Go2(c) => c.step_targets(estimate, dt_s)?,
            Feedback::G1(c) => c.step_targets(estimate, dt_s)?,
        };
        self.last_command_words = targets
            .iter()
            .map(|target| target.position.to_bits())
            .collect();
        sim.step_joint_position_actuation_targets(&targets);
        self.last_feedback = self.feedback.telemetry();
        if matches!(self.robot, Robot::G1) {
            self.last_feedback["actual_support_points_world_m"] = json!(support_points);
        }
        self.last_feedback["observation_timing"] = self.pipeline.telemetry();
        self.last_feedback["actual_support_points_scope"] =
            json!("driver pre-step ground truth; not feedback input");
        if self.lateral_gyro_experiment {
            self.add_lateral_diagnostics(&mut diagnostic_feedback, filter_before)?;
        }
        if let Some(run) = &mut self.diagnostics {
            if self.orientation_age_experiment {
                run.record_with_feedback(
                    self.pipeline.telemetry(),
                    diagnostic_estimate,
                    diagnostic_truth,
                    &self.last_feedback,
                    diagnostic_feedback,
                )?;
            } else {
                run.record(
                    self.pipeline.telemetry(),
                    diagnostic_estimate,
                    diagnostic_truth,
                    &self.last_feedback,
                )?;
            }
        }
        Ok(())
    }

    fn add_lateral_diagnostics(
        &self,
        context: &mut Option<Value>,
        filter_before: Option<Value>,
    ) -> CaptureResult<()> {
        let Some(context) = context else {
            return Ok(());
        };
        let control_words = self.replay_words();
        let suffix = self.estimator.replay_words();
        let suffix_start = control_words.len() - suffix.len();
        require(
            control_words[suffix_start..] == suffix,
            "EMA state missing from control replay",
        )?;
        context["g1_lateral_gyro_experiment"] = json!(true);
        context["g1_observation_diagnosis"] = json!(true);
        context["lateral_gyro_filter_before"] = json!(filter_before);
        context["lateral_gyro_filter_after"] = self.estimator.diagnostic_state();
        context["lateral_gyro_control_replay"] = json!({
            "suffix_start_word":suffix_start,"full_control_word_count":control_words.len(),
            "suffix_words":&control_words[suffix_start..],
            "scope":"actual post-step Controller::replay_words; filter updates at this row's pre-step consumer tick and does not mutate during the plant step"
        });
        context["feedback_channel_time_scope"]
            .as_object_mut()
            .expect("feedback scope")
            .remove("captured_gyro_and_roll_rate_unchanged");
        context["feedback_channel_time_scope"]["raw_captured_gyro_unchanged"] = json!(true);
        context["feedback_channel_time_scope"]["captured_roll_rate_unchanged"] = json!(true);
        context["feedback_channel_time_scope"]["orientation_projection_gyro"] =
            json!("original captured world angular velocity; never EMA output");
        context["feedback_channel_time_scope"]["effective_world_gyro_x"] = json!("EMA policy at positive capture age: capture-time EMA; otherwise original captured value");
        context["feedback_channel_time_scope"]["effective_world_gyro_yz_and_roll_rate"] =
            json!("original capture time, unchanged");
        context["feedback_channel_time_scope"]["non_orientation_channel_ticks_scope"] = json!(
            "capture time for unchanged channels; filtered gyro X spans delivered capture history"
        );
        Ok(())
    }

    fn telemetry(&self) -> Value {
        self.last_feedback.clone()
    }

    fn configuration(&self) -> Value {
        self.feedback.configuration()
    }

    fn replay_words(&self) -> Vec<u64> {
        let mut words = self.pipeline.replay_words();
        let feedback = self.feedback.replay_words();
        words.push(feedback.len() as u64);
        words.extend(feedback);
        words.push(self.last_command_words.len() as u64);
        words.extend(&self.last_command_words);
        // No marker or length word is added until the opt-in filter initializes.
        words.extend(self.estimator.replay_words());
        words
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

/// Identifies selected release control, plant, and motor-readback source inputs.
///
/// This is not an exhaustive dependency closure. Test-only qualification sources
/// and the complete build/configuration scope are bound by external run manifests.
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
            "examples/139_kick_comparison/observation.rs",
            include_str!("observation.rs"),
        ),
        (
            "examples/139_kick_comparison/diagnostic.rs",
            include_str!("diagnostic.rs"),
        ),
        (
            "examples/139_kick_comparison/feedback.rs",
            include_str!("feedback.rs"),
        ),
        (
            "examples/139_kick_comparison/motor_diagnostics.rs",
            include_str!("motor_diagnostics.rs"),
        ),
        (
            "crates/rne_physics/src/components.rs",
            include_str!("../../crates/rne_physics/src/components.rs"),
        ),
        (
            "crates/rne_physics/src/lib.rs",
            include_str!("../../crates/rne_physics/src/lib.rs"),
        ),
        (
            "crates/rne_physics/src/backend.rs",
            include_str!("../../crates/rne_physics/src/backend.rs"),
        ),
        (
            "crates/rne_ai/src/env/urdf_scene/mod.rs",
            include_str!("../../crates/rne_ai/src/env/urdf_scene/mod.rs"),
        ),
        (
            "crates/rne_physics_rapier/Cargo.toml",
            include_str!("../../crates/rne_physics_rapier/Cargo.toml"),
        ),
        (
            "crates/rne_data/src/frame.rs",
            include_str!("../../crates/rne_data/src/frame.rs"),
        ),
        (
            "crates/rne_data/src/bus.rs",
            include_str!("../../crates/rne_data/src/bus.rs"),
        ),
        (
            "crates/rne_data/src/stream.rs",
            include_str!("../../crates/rne_data/src/stream.rs"),
        ),
        (
            "crates/rne_core/src/rng.rs",
            include_str!("../../crates/rne_core/src/rng.rs"),
        ),
        (
            "crates/rne_core/src/time.rs",
            include_str!("../../crates/rne_core/src/time.rs"),
        ),
        (
            "crates/rne_world/src/resources.rs",
            include_str!("../../crates/rne_world/src/resources.rs"),
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
        .chain(motor_readback_support_sources())
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

fn motor_readback_support_sources() -> [(&'static str, &'static str); 12] {
    [
        (
            "examples/139_kick_comparison/Cargo.toml",
            include_str!("Cargo.toml"),
        ),
        (
            "crates/rne_ai/Cargo.toml",
            include_str!("../../crates/rne_ai/Cargo.toml"),
        ),
        (
            "third_party/rapier3d/src/dynamics/joint/multibody_joint/unit_multibody_joint.rs",
            include_str!("../../third_party/rapier3d/src/dynamics/joint/multibody_joint/unit_multibody_joint.rs"),
        ),
        (
            "third_party/rapier3d/src/dynamics/solver/island_solver.rs",
            include_str!("../../third_party/rapier3d/src/dynamics/solver/island_solver.rs"),
        ),
        (
            "third_party/rapier3d/src/dynamics/solver/joint_constraint/joint_generic_constraint_builder.rs",
            include_str!("../../third_party/rapier3d/src/dynamics/solver/joint_constraint/joint_generic_constraint_builder.rs"),
        ),
        (
            "third_party/rapier3d/src/dynamics/solver/velocity_solver.rs",
            include_str!("../../third_party/rapier3d/src/dynamics/solver/velocity_solver.rs"),
        ),
        (
            "third_party/rapier3d/src/pipeline/mod.rs",
            include_str!("../../third_party/rapier3d/src/pipeline/mod.rs"),
        ),
        (
            "third_party/rapier3d/src/pipeline/physics_pipeline.rs",
            include_str!("../../third_party/rapier3d/src/pipeline/physics_pipeline.rs"),
        ),
        (
            "third_party/rapier3d/src/pipeline/multibody_motor_impulse.rs",
            include_str!("../../third_party/rapier3d/src/pipeline/multibody_motor_impulse.rs"),
        ),
        (
            "third_party/rapier3d/src/dynamics/solver/joint_constraint/joint_generic_constraint.rs",
            include_str!("../../third_party/rapier3d/src/dynamics/solver/joint_constraint/joint_generic_constraint.rs"),
        ),
        (
            "third_party/rapier3d/src/dynamics/joint/multibody_joint/multibody.rs",
            include_str!("../../third_party/rapier3d/src/dynamics/joint/multibody_joint/multibody.rs"),
        ),
        (
            "third_party/rapier3d/src/dynamics/joint/motor_model.rs",
            include_str!("../../third_party/rapier3d/src/dynamics/joint/motor_model.rs"),
        ),
    ]
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
    control_state_words: Vec<Vec<u64>>,
    diagnostics: Option<FinishedDiagnostics>,
    motor_diagnostics: Option<FinishedMotorDiagnostics>,
}

#[derive(Debug)]
enum MotorCapture {
    Default,
    Enabled(Box<MotorRun>),
    DisabledComparator,
}

fn step_with_motor_diagnostics(
    controller: &mut Controller,
    sim: &mut UrdfSceneSim,
    motor: &mut Option<MotorRun>,
) -> CaptureResult<()> {
    let consumer_ticks = sim.sim_time().ticks();
    controller.step(sim)?;
    if let Some(run) = motor {
        run.record(sim, consumer_ticks)?;
    }
    Ok(())
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
    control_states: &mut Vec<Vec<u64>>,
    diagnostic_initial_plant: Option<&Value>,
    evaluation: (&mut Vec<Value>, &mut Option<MotorRun>),
) -> CaptureResult<Settlement> {
    let (settlement_telemetry, motor) = evaluation;
    let dt_s = sim.fixed_delta().as_seconds().value();
    let settlement_steps = (SETTLE_S / dt_s).round() as usize;
    let initial_root = sim
        .named_link_position_m(robot.base_link())
        .expect("validated root");
    let ground_index = diagnostic_initial_plant
        .map(|_| ground_body_index(sim))
        .transpose()?;
    let mut ground_truth = diagnostic_initial_plant
        .map(|_| Metrics::new(robot, sim, (0.0, 0.0)))
        .transpose()?;
    let mut max_fixed_position_error_m: f64 = 0.0;
    let mut max_fixed_rotation_error_rad: f64 = 0.0;
    for step in 1..=settlement_steps {
        step_with_motor_diagnostics(controller, sim, motor)?;
        let (position, rotation) = model::fixed_joint_error(sim)?;
        max_fixed_position_error_m = max_fixed_position_error_m.max(position);
        max_fixed_rotation_error_rad = max_fixed_rotation_error_rad.max(rotation);
        states.push(state_bits(robot, sim));
        control_states.push(controller.replay_words());
        if let Some(metrics) = &mut ground_truth {
            metrics.measure(
                robot,
                sim,
                step,
                settlement_steps,
                ground_index.expect("diagnostic ground"),
            )?;
        }
        if diagnostic_initial_plant.is_some() && step % (0.02 / dt_s).round() as usize == 0 {
            let mut measurement = measured_telemetry(
                robot,
                sim,
                controller,
                step as f64 * dt_s,
                robot.tilt_rad(sim)?,
                robot.height_m(sim)?,
            )?;
            measurement["phase"] = json!("settlement");
            measurement["completed_sim_ticks"] = json!(sim.sim_time().ticks());
            settlement_telemetry.push(measurement);
        }
    }
    let fixed_attachments_preserved =
        max_fixed_position_error_m <= 1e-4 && max_fixed_rotation_error_rad <= 5e-4;
    // The initial plant contract was already validated strictly. A physical
    // attachment failure in a probe is retained instead of reclassified as input error.
    let plant = if !fixed_attachments_preserved {
        if let Some(initial) = diagnostic_initial_plant {
            initial.clone()
        } else {
            robot.validate(sim)?
        }
    } else {
        robot.validate(sim)?
    };
    let settled_upright = robot.tilt_rad(sim)? < 0.15
        && robot.height_m(sim)? > robot.minimum_height_m()
        && (diagnostic_initial_plant.is_none() || fixed_attachments_preserved)
        && ground_truth
            .as_ref()
            .is_none_or(|metrics| !metrics.ever_fallen && !metrics.nonfoot_ground_contact);
    let ground_truth = ground_truth
        .map(|metrics| {
            metrics.summary(
                robot,
                sim,
                Case {
                    name: "settlement",
                    impulse_ns: 0.0,
                    dt_ticks: sim.fixed_delta().ticks(),
                    ..NOMINAL
                },
                initial_root,
            )
        })
        .transpose()?;
    Ok(Settlement {
        plant,
        max_fixed_position_error_m,
        max_fixed_rotation_error_rad,
        settled_upright,
        ground_truth,
    })
}

#[derive(Debug)]
struct Settlement {
    plant: Value,
    max_fixed_position_error_m: f64,
    max_fixed_rotation_error_rad: f64,
    settled_upright: bool,
    ground_truth: Option<Value>,
}

fn settlement_metadata(
    robot: Robot,
    sim: &UrdfSceneSim,
    controller: &Controller,
    settlement: &Settlement,
    telemetry: Vec<Value>,
    impulse_ns: f64,
) -> CaptureResult<Value> {
    Ok(json!({"settled_upright":settlement.settled_upright,
        "impact_applied":settlement.settled_upright && impulse_ns > 0.0,
        "failure_phase":if settlement.settled_upright {None::<&str>} else {Some("settlement")},
        "maximum_fixed_position_error_m":settlement.max_fixed_position_error_m,
        "maximum_fixed_rotation_error_rad":settlement.max_fixed_rotation_error_rad,
        "final_ground_truth_tilt_rad":robot.tilt_rad(sim)?,
        "final_ground_truth_root_height_m":robot.height_m(sim)?,
        "completed_sim_ticks":sim.sim_time().ticks(),"ground_truth":settlement.ground_truth,
        "last_delivered_feedback":controller.telemetry(),"telemetry":telemetry}))
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

    fn recording_summary(
        &self,
        robot: Robot,
        sim: &UrdfSceneSim,
        case: Case,
        initial_root: Vec3,
        settled_upright: bool,
    ) -> CaptureResult<Value> {
        let mut summary = if settled_upright {
            self.summary(robot, sim, case, initial_root)?
        } else {
            json!({"recovered":false,"post_impact_evaluated":false,
                "unavailable_reason":"failed settlement; no impact applied",
                "peak_tilt_rad":null,"min_height_m":null,"ever_fallen":null,"nonfoot_ground_contact":null})
        };
        summary["settled_upright"] = json!(settled_upright);
        summary["impact_applied"] = json!(settled_upright && case.impulse_ns > 0.0);
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
    record_with_profile(robot, case, capture_frames, None)
}

fn apply_impact(
    robot: Robot,
    sim: &mut UrdfSceneSim,
    impact: &Impact,
    step: usize,
) -> CaptureResult<(Vec3, Option<Vec3>)> {
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
    Ok((force, point))
}

fn record_with_profile(
    robot: Robot,
    case: Case,
    capture_frames: bool,
    profile: Option<Profile>,
) -> CaptureResult<Recording> {
    record_with_diagnostics(robot, case, capture_frames, profile, None)
}

fn record_with_diagnostics(
    robot: Robot,
    case: Case,
    capture_frames: bool,
    profile: Option<Profile>,
    diagnostics: Option<RunDiagnostics>,
) -> CaptureResult<Recording> {
    record_with_feedback_policy(robot, case, capture_frames, profile, diagnostics, None)
}

fn record_with_feedback_policy(
    robot: Robot,
    case: Case,
    capture_frames: bool,
    profile: Option<Profile>,
    diagnostics: Option<RunDiagnostics>,
    feedback_policy: Option<Policy>,
) -> CaptureResult<Recording> {
    record_with_experiment(
        robot,
        case,
        capture_frames,
        profile,
        diagnostics,
        feedback_policy.map(|policy| ExperimentFeedback {
            policy,
            mode: ExperimentMode::OrientationAge,
        }),
    )
}

fn record_with_experiment(
    robot: Robot,
    case: Case,
    capture_frames: bool,
    profile: Option<Profile>,
    diagnostics: Option<RunDiagnostics>,
    feedback: Option<ExperimentFeedback>,
) -> CaptureResult<Recording> {
    record_with_motor(
        robot,
        case,
        capture_frames,
        profile,
        diagnostics,
        feedback,
        MotorCapture::Default,
    )
}

fn record_with_motor(
    robot: Robot,
    case: Case,
    capture_frames: bool,
    profile: Option<Profile>,
    diagnostics: Option<RunDiagnostics>,
    feedback: Option<ExperimentFeedback>,
    motor_capture: MotorCapture,
) -> CaptureResult<Recording> {
    let mut sim = robot.load(case.dt_ticks)?;
    let initial_plant_contract = robot.validate(&sim)?;
    let mut controller = Controller::new(
        robot,
        &mut sim,
        case.balance_feedback_enabled,
        profile.unwrap_or_else(|| Profile::ideal(case.dt_ticks)),
    )?;
    controller.diagnostics = diagnostics;
    controller.feedback_policy = feedback.map_or_else(Policy::default, |feedback| feedback.policy);
    controller.orientation_age_experiment = feedback.is_some();
    controller.lateral_gyro_experiment =
        feedback.is_some_and(|feedback| feedback.mode == ExperimentMode::LateralGyro);
    let (mut motor, explicit_motor_off) = configure_motor_capture(&mut sim, motor_capture)?;
    let dt_s = sim.fixed_delta().as_seconds().value();
    let mut states = Vec::new();
    let mut control_states = Vec::new();
    let mut settlement_telemetry = Vec::new();
    let ground_index = ground_body_index(&sim)?;
    let settlement = settle(
        robot,
        &mut sim,
        &mut controller,
        &mut states,
        &mut control_states,
        profile.map(|_| &initial_plant_contract),
        (&mut settlement_telemetry, &mut motor),
    )?;
    let settlement_summary = settlement_metadata(
        robot,
        &sim,
        &controller,
        &settlement,
        settlement_telemetry,
        case.impulse_ns,
    )?;
    let Settlement {
        plant,
        max_fixed_position_error_m: initial_fixed_position_error_m,
        max_fixed_rotation_error_rad: initial_fixed_rotation_error_rad,
        settled_upright,
        ground_truth: _,
    } = settlement;
    if profile.is_none() {
        require(
            settled_upright,
            "the physical plant did not settle upright before the disturbance",
        )?;
    }
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
    let steps = if settled_upright {
        (((FRAME_COUNT - 1) as f64 / FRAME_RATE_HZ) / dt_s).round() as usize
    } else {
        0
    };
    for step in 1..=steps {
        let (force, point) = apply_impact(robot, &mut sim, &impact, step)?;
        step_with_motor_diagnostics(&mut controller, &mut sim, &mut motor)?;
        let (tilt, height) = metrics.measure(robot, &sim, step, steps, ground_index)?;
        states.push(state_bits(robot, &sim));
        control_states.push(controller.replay_words());
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
    let summary = metrics.recording_summary(robot, &sim, case, initial_root, settled_upright)?;
    let roots: Vec<_> = sim
        .mesh_package_roots()
        .iter()
        .map(|p| p.to_string_lossy().into_owned())
        .collect();
    let hash = stable_hash(&states);
    let word_count: usize = states.iter().map(Vec::len).sum();
    let control_hash = stable_hash(&control_states);
    let control_word_count: usize = control_states.iter().map(Vec::len).sum();
    let diagnostics = controller
        .diagnostics
        .take()
        .map(RunDiagnostics::finish)
        .transpose()?;
    let motor_diagnostics = motor.map(MotorRun::finish).transpose()?;
    let recording = Recording {
        trace: json!({"schema_version":2,"robot":robot.name(),"case":case.name,
        "compiled_source_sha256":compiled_source_hashes(),
        "backend":"Rapier through UrdfSceneSim","coordinate_system":"Y-up","dt_s":dt_s,
        "solver_iterations":32,"push_start_s":case.start_s,"push_duration_s":impact.duration_s(),
        "force_n":impact.peak_force_n(),"impulse_n_s":case.impulse_ns,
        "integrated_impulse_world_ns":if settled_upright {impact.integrated_impulse_ns().to_array()} else {Vec3::ZERO.to_array()},
        "planned_integrated_impulse_world_ns":impact.integrated_impulse_ns().to_array(),
        "actual_applied_impulse_world_ns":if settled_upright {impact.integrated_impulse_ns().to_array()} else {Vec3::ZERO.to_array()},
        "force_link":robot.base_link(),"application_offset_m":[0.0,PUSH_OFFSET_Y_M,0.0],
        "pulse":"midpoint half-sine, normalized to the declared impulse",
        "plant":plant,"controller":controller.configuration(),"balance_feedback_enabled":case.balance_feedback_enabled,
        "observation_pipeline":controller.pipeline.configuration(),"observation_metrics":controller.pipeline.telemetry(),
        "settlement":settlement_summary,
        "mesh_package_roots":roots,"mechanism":"Prescribed external wrench; human is render-only; no edited robot poses",
        "summary":summary,"force_history":force_history,"telemetry":telemetry,"frames":frames,
        "observed_state_definition":"Link-name order: world pose/scale, body COM linear/angular velocity, joint q/qd with presence tags; fixed foot order normal impulse. Literal IEEE f64 bits, including settlement; excludes hidden solver caches.",
        "observed_state_hash":hash,"observed_state_word_count":word_count,
        "observation_control_state_hash":control_hash,"observation_control_state_word_count":control_word_count,
        "observation_control_state_definition":"Each tick including settlement: seed/domain and exact schedule, counters, bounded retained arrival frames and last consumed sample, canonical payload words, controller filters/trims/arms/quiet timer and sorted joint command words. Literal IEEE f64 bits and integer ticks; excludes hidden solver caches."}),
        state_bits: states,
        control_state_words: control_states,
        diagnostics,
        motor_diagnostics,
    };
    let recording = annotate_motor_recording(recording, explicit_motor_off)?;
    Ok(annotate_feedback_recording(recording, feedback))
}

fn configure_motor_capture(
    sim: &mut UrdfSceneSim,
    capture: MotorCapture,
) -> CaptureResult<(Option<MotorRun>, bool)> {
    let explicit_off = matches!(&capture, MotorCapture::DisabledComparator);
    let motor = match capture {
        MotorCapture::Default => None,
        MotorCapture::DisabledComparator => {
            require(
                sim.configure_position_motor_diagnostics(false)?,
                "the motor diagnostics backend-disabled comparator requires a supported backend",
            )?;
            None
        }
        MotorCapture::Enabled(run) => {
            require(
                sim.configure_position_motor_diagnostics(true)?,
                "--motor-effort-diagnostics requires a supported backend built with --features position-motor-diagnostics",
            )?;
            Some(*run)
        }
    };
    Ok((motor, explicit_off))
}

fn annotate_motor_recording(
    mut recording: Recording,
    explicit_motor_off: bool,
) -> CaptureResult<Recording> {
    if let Some(run) = &mut recording.motor_diagnostics {
        require(
            run.words.len() == recording.state_bits.len(),
            "motor readback missed a completed step",
        )?;
        run.metadata["replay_hash"] = json!(stable_hash(&run.words));
        run.metadata["replay_word_count"] = json!(run.words.iter().map(Vec::len).sum::<usize>());
        run.metadata["backend_configuration_accepted"] = json!(true);
        recording.trace["motor_effort_diagnostics"] = json!(true);
        recording.trace["position_motor_diagnostic"] = run.metadata.clone();
    }
    if explicit_motor_off {
        mark_motor_off(&mut recording.trace);
    }
    Ok(recording)
}

fn annotate_feedback_recording(
    recording: Recording,
    feedback: Option<ExperimentFeedback>,
) -> Recording {
    let mut recording =
        annotate_orientation_recording(recording, feedback.map(|feedback| feedback.policy));
    if feedback.is_some_and(|feedback| feedback.mode == ExperimentMode::LateralGyro) {
        mark_lateral_experiment(&mut recording.trace);
    }
    recording
}

fn annotate_orientation_recording(mut recording: Recording, policy: Option<Policy>) -> Recording {
    if let Some(policy) = policy {
        mark_orientation_experiment(&mut recording.trace, policy);
    }
    recording
}

fn mark_orientation_experiment(trace: &mut Value, policy: Policy) {
    trace["g1_orientation_age_experiment"] = json!(true);
    trace["g1_observation_diagnosis"] = json!(true);
    trace["feedback_policy"] = json!(policy.name());
    trace["feedback_policy_configuration"] = policy.configuration();
    trace["summary"]["g1_orientation_age_experiment"] = json!(true);
    trace["summary"]["feedback_policy"] = json!(policy.name());
}

fn mark_lateral_experiment(trace: &mut Value) {
    for scope in ["summary", "controller"] {
        trace[scope]["g1_lateral_gyro_experiment"] = json!(true);
        trace[scope]["g1_orientation_age_experiment"] = json!(true);
        trace[scope]["g1_observation_diagnosis"] = json!(true);
        trace[scope]["feedback_policy"] = trace["feedback_policy"].clone();
    }
    trace["g1_lateral_gyro_experiment"] = json!(true);
}

fn replay(robot: Robot, case: Case, capture_frames: bool) -> CaptureResult<Value> {
    let mut first = record(robot, case, capture_frames)?;
    let repeat = record(robot, case, capture_frames)?;
    verify_replay(&mut first, &repeat)?;
    Ok(first.trace)
}

fn verify_replay(first: &mut Recording, repeat: &Recording) -> CaptureResult<()> {
    require(
        first.state_bits == repeat.state_bits,
        "the observed replay differs bit for bit",
    )?;
    require(
        first.control_state_words == repeat.control_state_words,
        "the observation/control pipeline replay differs bit for bit",
    )?;
    require(
        first.trace["frames"] == repeat.trace["frames"],
        "the canonical render frames differ on replay",
    )?;
    first.trace["observed_state_bits_exact_repeat"] = json!(true);
    first.trace["canonical_render_frames_exact_repeat"] = json!(true);
    first.trace["observation_control_state_bits_exact_repeat"] = json!(true);
    match (&first.motor_diagnostics, &repeat.motor_diagnostics) {
        (Some(first_motor), Some(repeat_motor)) => {
            require(
                first_motor.words == repeat_motor.words,
                "the separate motor diagnostic replay differs bit for bit",
            )?;
            first.trace["position_motor_diagnostic"]["replay_bits_exact_fresh_repeat"] =
                json!(true);
        }
        (None, None) => {}
        _ => return Err(io::Error::other("motor diagnostic replay enablement differs").into()),
    }
    Ok(())
}

fn verify_motor_off(first: &mut Recording, off: &Recording) -> CaptureResult<()> {
    require(
        first.motor_diagnostics.is_some() && off.motor_diagnostics.is_none(),
        "motor ON/OFF comparison did not toggle backend instrumentation",
    )?;
    require(
        off.trace["position_motor_diagnostic"]["backend_configuration_accepted"] == true
            && off.trace["position_motor_diagnostic"]["enabled"] == false,
        "motor comparator did not explicitly disable backend collection",
    )?;
    require(
        first.state_bits == off.state_bits,
        "motor instrumentation changed literal plant words",
    )?;
    require(
        first.control_state_words == off.control_state_words,
        "motor instrumentation changed literal observation/control words",
    )?;
    for key in [
        "plant",
        "controller",
        "observation_pipeline",
        "observation_metrics",
        "settlement",
        "summary",
        "force_history",
        "telemetry",
        "frames",
    ] {
        require(
            first.trace[key] == off.trace[key],
            "motor instrumentation changed existing outcomes or render-frame data",
        )?;
    }
    let decisions_exact = if first.diagnostics.is_some() || off.diagnostics.is_some() {
        require(
            diagnosis_decisions(first)? == diagnosis_decisions(off)?,
            "motor instrumentation changed literal old decision fields",
        )?;
        Some(true)
    } else {
        None
    };
    first.trace["position_motor_diagnostic"]
        ["backend_on_off_legacy_plant_control_outcomes_exact"] = json!(true);
    first.trace["position_motor_diagnostic"]["backend_on_off_old_decision_fields_exact"] =
        json!(decisions_exact);
    first.trace["position_motor_diagnostic"]["fresh_backend_enabled_worlds"] = json!(2);
    first.trace["position_motor_diagnostic"]["fresh_backend_disabled_worlds"] = json!(1);
    first.trace["position_motor_diagnostic"]["motor_file_writer_on_off_tested_separately"] =
        json!(false);
    Ok(())
}

fn motor_run(
    output: &Path,
    stem: &str,
    enabled: bool,
    repeat: bool,
) -> CaptureResult<MotorCapture> {
    if enabled {
        Ok(MotorCapture::Enabled(Box::new(MotorRun::enabled(
            &output.join(format!(
                "{stem}.motor{}.jsonl",
                if repeat { ".repeat" } else { "" }
            )),
        )?)))
    } else {
        Ok(MotorCapture::Default)
    }
}

fn finish_motor_repeat(
    first: &mut Recording,
    output: &Path,
    stem: &str,
    enabled: bool,
) -> CaptureResult<()> {
    if enabled {
        let path = output.join(format!("{stem}.motor.jsonl"));
        let repeat = output.join(format!("{stem}.motor.repeat.jsonl"));
        diagnostic::require_same_bytes(&path, &repeat)?;
        first.trace["position_motor_diagnostic"]["jsonl_bytes_exact_fresh_repeat"] = json!(true);
        fs::remove_file(repeat)?;
    }
    Ok(())
}

fn mark_motor_off(trace: &mut Value) {
    trace["motor_effort_diagnostics"] = json!(true);
    trace["position_motor_diagnostic"] = json!({"enabled":false,"backend_configuration_accepted":true,"scope":"evaluator-only comparator after explicitly disabling backend collection; missing diagnostic data are unavailable, not zero effort"});
}

fn replay_with_motor(
    robot: Robot,
    case: Case,
    capture_frames: bool,
    output: &Path,
    stem: &str,
    enabled: bool,
) -> CaptureResult<Value> {
    if !enabled {
        return replay(robot, case, capture_frames);
    }
    let mut first = record_with_motor(
        robot,
        case,
        capture_frames,
        None,
        None,
        None,
        motor_run(output, stem, true, false)?,
    )?;
    {
        let repeat = record_with_motor(
            robot,
            case,
            capture_frames,
            None,
            None,
            None,
            motor_run(output, stem, true, true)?,
        )?;
        verify_replay(&mut first, &repeat)?;
        finish_motor_repeat(&mut first, output, stem, true)?;
    }
    {
        let off = record_with_motor(
            robot,
            case,
            capture_frames,
            None,
            None,
            None,
            MotorCapture::DisabledComparator,
        )?;
        verify_motor_off(&mut first, &off)?;
        write_diagnosis_trace(output, &format!("{stem}.motor-off.json"), &off.trace)?;
    }
    Ok(first.trace)
}

fn copy_motor_metadata(report: &mut Value, trace: &Value) {
    if trace["motor_effort_diagnostics"] == true {
        report["motor_effort_diagnostics"] = json!(true);
        report["position_motor_diagnostic"] = trace["position_motor_diagnostic"].clone();
        if report.get("case").is_none() {
            report["case"] = trace["case"].clone();
        }
    }
}

fn write_motor_report(output: &Path, cases: &[Value], enabled: bool) -> CaptureResult<()> {
    if enabled {
        let report = json!({"schema_version":1,"motor_effort_diagnostics":true,
            "scope":"headless evaluator-only completed position-motor intervals; no hardware validation, controller feedback change, candidate promotion, or fall-cause claim",
            "compiled_source_binding_scope":"selected actual release control/plant/readback inputs, not every transitive dependency; test-only qualification and exhaustive runtime/build inputs are separately bound by run manifests",
            "compiled_source_sha256":compiled_source_hashes(),
            "cases":cases});
        fs::write(
            output.join("motor-effort-diagnostics.json"),
            serde_json::to_vec_pretty(&report)?,
        )?;
    }
    Ok(())
}

/// Captures both robots at 1 kHz under the same declared 80 ms lateral pulse.
pub(super) fn capture(output: &Path, impulse_ns: f64, motor_enabled: bool) -> CaptureResult<()> {
    fs::create_dir_all(output)?;
    let mut summaries = Vec::new();
    let mut all_recovered = true;
    for robot in [Robot::Go2, Robot::G1] {
        let trace = replay_with_motor(
            robot,
            Case {
                impulse_ns,
                ..NOMINAL
            },
            true,
            output,
            &format!("{}-nominal", robot.name().to_lowercase()),
            motor_enabled,
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
            "observation_pipeline":trace["observation_pipeline"],
            "observation_control_state_hash":trace["observation_control_state_hash"],
            "observation_control_state_bits_exact_repeat":true,
            "canonical_render_frames_exact_repeat":true}));
        copy_motor_metadata(summaries.last_mut().expect("new nominal summary"), &trace);
    }
    write_motor_report(output, &summaries, motor_enabled)?;
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
    motor_enabled: bool,
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
    let reports = record_case_matrix(robots, &cases, output, true, motor_enabled)?;
    write_motor_report(output, &reports, motor_enabled)?;
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
pub(super) fn sweep_impulse(
    output: &Path,
    robot_filter: Option<&str>,
    motor_enabled: bool,
) -> CaptureResult<()> {
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
    let reports = record_case_matrix(robots, &cases, output, false, motor_enabled)?;
    write_motor_report(output, &reports, motor_enabled)?;
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
    motor_enabled: bool,
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
        motor_enabled,
    )?;
    write_motor_report(output, &reports, motor_enabled)?;
    fs::write(
        output.join("impulse-probe.json"),
        serde_json::to_vec_pretty(&json!({
            "schema_version":1,"cases":reports,
            "note":"A measured limit probe; recovery is reported, not assumed"
        }))?,
    )?;
    Ok(())
}

/// Records fixed, predeclared synthetic estimate profiles without requiring recovery.
pub(super) fn probe_observations(
    output: &Path,
    robot_filter: Option<&str>,
    profile_filter: Option<&str>,
    motor_enabled: bool,
) -> CaptureResult<()> {
    fs::create_dir_all(output)?;
    let robots = selected_robots(robot_filter)?;
    let profiles = observation::profiles(profile_filter)?;
    let results = std::thread::scope(|scope| {
        let workers: Vec<_> = robots.iter().copied().map(|robot| {
            let profiles = &profiles;
            scope.spawn(move || {
                let mut reports = Vec::new();
                for &profile in profiles {
                    let case = Case {
                        name: profile.name,
                        impulse_ns: if profile.zero_input {0.0} else {DEFAULT_IMPULSE_NS},
                        ..NOMINAL
                    };
                    let stem = format!("{}-observation-{}", robot.name().to_lowercase(), profile.name);
                    let mut first = record_with_motor(robot, case, false, Some(profile), None, None, motor_run(output,&stem,motor_enabled,false).map_err(|error|error.to_string())?)
                        .map_err(|error| error.to_string())?;
                    {
                        let repeat = record_with_motor(robot, case, false, Some(profile), None, None, motor_run(output,&stem,motor_enabled,true).map_err(|error|error.to_string())?)
                            .map_err(|error| error.to_string())?;
                        verify_replay(&mut first, &repeat).map_err(|error| error.to_string())?;
                        finish_motor_repeat(&mut first,output,&stem,motor_enabled).map_err(|error|error.to_string())?;
                    }
                    if motor_enabled {
                        let off = record_with_motor(robot,case,false,Some(profile),None,None,MotorCapture::DisabledComparator).map_err(|error|error.to_string())?;
                        verify_motor_off(&mut first,&off).map_err(|error|error.to_string())?;
                        write_diagnosis_trace(output,&format!("{stem}.motor-off.json"),&off.trace).map_err(|error|error.to_string())?;
                    }
                    let filename = format!("{}-observation-{}.json", robot.name().to_lowercase(), profile.name);
                    let trace_bytes = serde_json::to_vec(&first.trace).map_err(|error| error.to_string())?;
                    let trace_sha256 = format!("{:x}", Sha256::digest(&trace_bytes));
                    fs::write(output.join(&filename), &trace_bytes).map_err(|error| error.to_string())?;
                    reports.push(json!({"robot":robot.name(),"profile":profile.name,"trace":filename,"trace_sha256":trace_sha256,
                        "impulse_n_s":case.impulse_ns,"physics_period_ticks":case.dt_ticks,
                        "controller_period_ticks":case.dt_ticks,"observation_pipeline":first.trace["observation_pipeline"],
                        "observation_metrics":first.trace["observation_metrics"],"settlement":first.trace["settlement"],
                        "summary":first.trace["summary"],"compiled_source_sha256":first.trace["compiled_source_sha256"],
                        "actual_applied_impulse_world_ns":first.trace["actual_applied_impulse_world_ns"],
                        "observed_state_hash":first.trace["observed_state_hash"],"observed_state_bits_exact_repeat":true,
                        "observation_control_state_hash":first.trace["observation_control_state_hash"],
                        "observation_control_state_bits_exact_repeat":true}));
                    copy_motor_metadata(reports.last_mut().expect("new observation report"),&first.trace);
                }
                Ok::<_, String>(reports)
            })
        }).collect();
        workers
            .into_iter()
            .map(|worker| {
                worker
                    .join()
                    .map_err(|_| io::Error::other("an observation probe worker panicked"))?
                    .map_err(io::Error::other)
            })
            .collect::<Result<Vec<_>, _>>()
    })?;
    let cases: Vec<_> = results.into_iter().flatten().collect();
    write_motor_report(output, &cases, motor_enabled)?;
    fs::write(
        output.join("observation-probe.json"),
        serde_json::to_vec_pretty(&json!({
            "schema_version":1,"diagnostic_completed":true,"complete_matrix":robots.len() == 2 && profiles.len() == 8,
            "robots":robots.iter().map(|robot| robot.name()).collect::<Vec<_>>(),
            "profiles":profiles.iter().map(|profile| profile.name).collect::<Vec<_>>(),
            "scope":"Synthetic simulator-derived state-estimate timing and bounded-error sensitivity at unchanged 1 kHz physics and controller evaluation. Inner implicit position PD retains ideal joint feedback. Human remains render-only and disturbance a prescribed body wrench.",
            "required_nominal_envelope_changed":false,"hardware_validation":false,"identified_sensor_model":false,
            "recovery_required_for_probe_completion":false,"cases":cases
        }))?,
    )?;
    Ok(())
}

#[derive(Debug)]
struct DiagnosisPrefix {
    states: Vec<Vec<u64>>,
    controls: Vec<Vec<u64>>,
    decisions: Vec<Vec<u64>>,
    pipeline: Value,
}

fn diagnosis_prefix_count(recording: &Recording) -> usize {
    // Completed steps before the first force-bearing step, including settlement.
    recording
        .state_bits
        .len()
        .min(((SETTLE_S + NOMINAL.start_s) / 1e-3).round() as usize)
}

impl DiagnosisPrefix {
    fn from_recording(mut recording: Recording) -> Self {
        let count = diagnosis_prefix_count(&recording);
        recording.state_bits.truncate(count);
        recording.control_state_words.truncate(count);
        let mut decisions = recording
            .diagnostics
            .take()
            .expect("diagnostics")
            .decision_words;
        decisions.truncate(count);
        Self {
            states: recording.state_bits,
            controls: recording.control_state_words,
            decisions,
            pipeline: recording.trace["observation_pipeline"].clone(),
        }
    }

    fn compare(&self, kick: &Recording, name: &str) -> CaptureResult<Value> {
        let count = diagnosis_prefix_count(kick);
        require(
            self.states.len() == count && self.states == kick.state_bits[..count],
            "zero/kick pre-input plant prefix differs",
        )?;
        require(
            self.controls == kick.control_state_words[..count],
            "zero/kick pre-input control prefix differs",
        )?;
        require(
            self.decisions == diagnosis_decisions(kick)?[..count],
            "zero/kick pre-input decision prefix differs",
        )?;
        require(
            self.pipeline == kick.trace["observation_pipeline"],
            "zero/kick estimate configuration differs",
        )?;
        Ok(json!({"profile":name,"completed_steps":count,
            "consumer_start_ticks":0,"consumer_last_ticks":count.saturating_sub(1) as u64 * diagnostic::DT_TICKS,
            "completed_through_ticks":count as u64 * diagnostic::DT_TICKS,
            "planned_first_force_consumer_ticks":4_000_000_000_u64,
            "planned_first_force_completed_ticks":4_001_000_000_u64,
            "plant_words_exact":true,"control_words_exact":true,"existing_decision_fields_exact":true,
            "observation_pipeline_exact":true,"plant_prefix_hash":stable_hash(&self.states),
            "control_prefix_hash":stable_hash(&self.controls),"decision_prefix_hash":stable_hash(&self.decisions),
            "scope":"settlement plus recording prefix before first force-bearing step; impulse is the only case input difference"}))
    }
}

fn diagnosis_decisions(recording: &Recording) -> CaptureResult<&[Vec<u64>]> {
    recording
        .diagnostics
        .as_ref()
        .map(|run| run.decision_words.as_slice())
        .ok_or_else(|| io::Error::other("missing all-tick diagnosis decision words").into())
}

fn verify_diagnosis_behavior(first: &Recording, other: &Recording) -> CaptureResult<()> {
    require(
        first.state_bits == other.state_bits,
        "diagnosis changed literal plant words",
    )?;
    require(
        first.control_state_words == other.control_state_words,
        "diagnosis changed literal observation/control words",
    )?;
    require(
        diagnosis_decisions(first)? == diagnosis_decisions(other)?,
        "diagnosis changed literal existing decision fields",
    )?;
    for key in [
        "plant",
        "controller",
        "observation_pipeline",
        "observation_metrics",
        "settlement",
        "summary",
        "force_history",
        "telemetry",
    ] {
        require(
            first.trace[key] == other.trace[key],
            "diagnosis changed existing trace outcomes or telemetry",
        )?;
    }
    require(
        first.state_bits.len() == diagnosis_decisions(first)?.len(),
        "diagnosis missed a control decision",
    )
}

fn write_diagnosis_trace(output: &Path, filename: &str, trace: &Value) -> CaptureResult<String> {
    let bytes = serde_json::to_vec(trace)?;
    let hash = format!("{:x}", Sha256::digest(&bytes));
    fs::write(output.join(filename), bytes)?;
    Ok(hash)
}

/// Measures the eight predeclared G1 timing/error and zero/kick combinations.
///
/// Every case uses two fresh logging-on worlds and a logging-off world. Failure
/// to recover remains an outcome; exact replay and logging noninterference are
/// required independently. Only this opt-in path writes all-tick JSONL evidence.
pub(super) fn diagnose_g1_observations(
    output: &Path,
    profile_filter: Option<&str>,
    input_filter: Option<&str>,
    motor_enabled: bool,
) -> CaptureResult<()> {
    let names = diagnostic::profile_names(profile_filter)?;
    let inputs = diagnostic::inputs(input_filter)?;
    fs::create_dir_all(output)?;
    let mut reports = Vec::new();
    let mut prefixes = Vec::new();
    for name in &names {
        let profile = observation::profiles(Some(name))?[0];
        require(
            !profile.zero_input && profile.sample_period_ticks == diagnostic::DT_TICKS,
            "diagnosis must use the unchanged 1 kHz nonzero-input profile configuration",
        )?;
        let mut zero_prefix: Option<DiagnosisPrefix> = None;
        for &(input, impulse_ns) in &inputs {
            let case = Case {
                name,
                impulse_ns,
                ..NOMINAL
            };
            let stem = format!("g1-diagnosis-{name}-{input}");
            let jsonl_filename = format!("{stem}.jsonl");
            let jsonl_path = output.join(&jsonl_filename);
            let repeat_path = output.join(format!("{stem}.repeat.jsonl"));
            let mut first = record_with_motor(
                Robot::G1,
                case,
                false,
                Some(profile),
                Some(RunDiagnostics::enabled(&jsonl_path)?),
                None,
                motor_run(output, &stem, motor_enabled, false)?,
            )?;
            require(
                first.trace["observation_pipeline"]["world_seed"] == 2002,
                "G1 diagnosis seed must be 2002",
            )?;
            {
                let repeat = record_with_motor(
                    Robot::G1,
                    case,
                    false,
                    Some(profile),
                    Some(RunDiagnostics::enabled(&repeat_path)?),
                    None,
                    motor_run(output, &stem, motor_enabled, true)?,
                )?;
                verify_diagnosis_behavior(&first, &repeat)?;
                verify_replay(&mut first, &repeat)?;
                diagnostic::require_same_bytes(&jsonl_path, &repeat_path)?;
                finish_motor_repeat(&mut first, output, &stem, motor_enabled)?;
            }
            fs::remove_file(&repeat_path)?;
            let off_filename = format!("{stem}.logging-off.json");
            let off_sha256;
            {
                let mut off = record_with_motor(
                    Robot::G1,
                    case,
                    false,
                    Some(profile),
                    Some(RunDiagnostics::default()),
                    None,
                    if motor_enabled {
                        MotorCapture::DisabledComparator
                    } else {
                        MotorCapture::Default
                    },
                )?;
                verify_diagnosis_behavior(&first, &off)?;
                if motor_enabled {
                    verify_motor_off(&mut first, &off)?;
                }
                off.trace["g1_observation_diagnosis"] = json!(true);
                off.trace["all_tick_diagnostic_logging_enabled"] = json!(false);
                off_sha256 = write_diagnosis_trace(output, &off_filename, &off.trace)?;
            }
            let run = first.diagnostics.as_ref().expect("diagnostic recording");
            let mut artifact = run.artifact.clone().expect("enabled JSONL artifact");
            require(
                artifact["rows"] == first.state_bits.len(),
                "JSONL did not cover every control tick including settlement",
            )?;
            artifact["file"] = json!(jsonl_filename);
            artifact["bytes_exact_fresh_repeat"] = json!(true);
            let decisions_hash = stable_hash(&run.decision_words);
            let decisions_word_count: usize = run.decision_words.iter().map(Vec::len).sum();
            first.trace["g1_observation_diagnosis"] = json!(true);
            first.trace["all_tick_diagnostic_logging_enabled"] = json!(true);
            first.trace["all_tick_diagnostic"] = artifact.clone();
            first.trace["diagnosis_existing_decision_fields_hash"] = json!(decisions_hash);
            first.trace["diagnosis_existing_decision_fields_word_count"] =
                json!(decisions_word_count);
            first.trace["diagnosis_existing_decision_fields_exact_fresh_repeat"] = json!(true);
            first.trace["diagnosis_logging_on_off_plant_control_decisions_exact"] = json!(true);
            let filename = format!("{stem}.json");
            let trace_sha256 = write_diagnosis_trace(output, &filename, &first.trace)?;
            reports.push(json!({
                "g1_observation_diagnosis":true,
                "robot":"G1","profile":name,"input":input,"impulse_n_s":impulse_ns,
                "physics_period_ticks":case.dt_ticks,"controller_period_ticks":case.dt_ticks,
                "trace":filename,"trace_sha256":trace_sha256,
                "logging_off_trace":off_filename,"logging_off_trace_sha256":off_sha256,
                "compiled_source_sha256":first.trace["compiled_source_sha256"],
                "plant":first.trace["plant"],"controller":first.trace["controller"],
                "observation_pipeline":first.trace["observation_pipeline"],"observation_metrics":first.trace["observation_metrics"],
                "settlement":first.trace["settlement"],"summary":first.trace["summary"],
                "planned_integrated_impulse_world_ns":first.trace["planned_integrated_impulse_world_ns"],
                "actual_applied_impulse_world_ns":first.trace["actual_applied_impulse_world_ns"],
                "observed_state_hash":first.trace["observed_state_hash"],"observed_state_word_count":first.trace["observed_state_word_count"],
                "observation_control_state_hash":first.trace["observation_control_state_hash"],"observation_control_state_word_count":first.trace["observation_control_state_word_count"],
                "observed_state_bits_exact_repeat":true,"observation_control_state_bits_exact_repeat":true,
                "existing_decision_fields_hash":decisions_hash,"existing_decision_fields_word_count":decisions_word_count,
                "existing_decision_fields_exact_fresh_repeat":true,"diagnostic_jsonl":artifact,
                "logging_on_off_plant_words_exact":true,"logging_on_off_control_words_exact":true,
                "logging_on_off_existing_decision_fields_exact":true,"logging_on_off_existing_trace_outcomes_exact":true,
                "fresh_world_runs":3,"logging_on_fresh_runs":2,"logging_off_fresh_runs":1
            }));
            copy_motor_metadata(
                reports.last_mut().expect("new diagnosis report"),
                &first.trace,
            );
            if inputs.len() == 2 {
                if input == "zero" {
                    zero_prefix = Some(DiagnosisPrefix::from_recording(first));
                } else {
                    let zero = zero_prefix.take().expect("ordered zero then kick");
                    prefixes.push(zero.compare(&first, name)?);
                }
            }
        }
    }
    write_motor_report(output, &reports, motor_enabled)?;
    fs::write(
        output.join("g1-observation-diagnosis.json"),
        serde_json::to_vec_pretty(&json!({
            "schema_version":1,"g1_observation_diagnosis":true,"diagnostic_completed":true,
            "complete_matrix":names.len() == 4 && inputs.len() == 2,
            "predeclared_profiles":diagnostic::PROFILES,"selected_profiles":names,"selected_inputs":inputs.iter().map(|(name,_)|*name).collect::<Vec<_>>(),
            "predeclared_impulses_n_s":[0.0,40.0],"world_seed":2002,
            "physics_period_ticks":diagnostic::DT_TICKS,"controller_period_ticks":diagnostic::DT_TICKS,
            "settlement_s":SETTLE_S,"recording_horizon_s":6.2,"pulse_duration_s":DURATION_S,
            "maximum_control_rows_per_case":diagnostic::MAX_ROWS,
            "execution_scope":"cases processed sequentially; at most first and one comparator full literal word recording, plus one bounded pre-input prefix; all-tick JSON streamed with a 21-row evaluator truth history",
            "scope":"G1 synthetic simulator-derived state-estimate diagnosis. Evaluator truth/support never feed the controller. Inner implicit position PD retains ideal joints. Human is render-only; input remains a prescribed body wrench.",
            "recovery_required_for_completion":false,"recovery_predicate_changed":false,
            "controller_behavior_changed":false,"required_nominal_envelope_changed":false,
            "hardware_validation":false,"identified_sensor_model":false,
            "matched_zero_kick_prefixes":prefixes,"cases":reports
        }))?,
    )?;
    Ok(())
}

fn write_orientation_words(
    output: &Path,
    stem: &str,
    recording: &Recording,
) -> CaptureResult<Value> {
    let decisions = diagnosis_decisions(recording)?;
    require(
        recording.state_bits.len() == recording.control_state_words.len()
            && decisions.len() == recording.state_bits.len(),
        "orientation sidecar row counts differ",
    )?;
    let filename = format!("{stem}.words.bin");
    let mut writer = io::BufWriter::new(fs::File::create(output.join(&filename))?);
    let mut hash = Sha256::new();
    let mut bytes_count = 0;
    let mut write = |bytes: &[u8]| -> io::Result<()> {
        writer.write_all(bytes)?;
        hash.update(bytes);
        bytes_count += bytes.len();
        Ok(())
    };
    write(b"RNE_G1_WORDS_V1\0")?;
    write(&(recording.state_bits.len() as u64).to_le_bytes())?;
    for ((state, control), decision) in recording
        .state_bits
        .iter()
        .zip(&recording.control_state_words)
        .zip(decisions)
    {
        for words in [state, control, decision] {
            require(
                words.len() <= 4096,
                "zero-age literal sidecar vector exceeds 4096 words",
            )?;
            write(&(words.len() as u64).to_le_bytes())?;
            let bytes: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
            write(&bytes)?;
        }
    }
    writer.flush()?;
    Ok(
        json!({"file":filename,"sha256":format!("{:x}",hash.finalize()),"bytes":bytes_count,
        "rows":recording.state_bits.len(),"maximum_rows":diagnostic::MAX_ROWS,"maximum_words_per_vector":4096,
        "format":"16-byte RNE_G1_WORDS_V1 NUL header, little-endian u64 row count; each row has plant/control/old-decision vectors, each prefixed by u64 word count followed by literal u64 words",
        "scope":"full trajectory including settlement; immutable policy is bound separately and does not pollute mutable control words or old decision fields"}),
    )
}

fn feedback_experiment_case(
    output: &Path,
    profile: Profile,
    input: &str,
    impulse_ns: f64,
    policy: Policy,
    mode: ExperimentMode,
    motor_enabled: bool,
) -> CaptureResult<(Recording, Value)> {
    let case = Case {
        name: profile.name,
        impulse_ns,
        ..NOMINAL
    };
    let stem = format!("{}-{}-{}-{input}", mode.stem(), policy.name(), profile.name);
    let jsonl_filename = format!("{stem}.jsonl");
    let jsonl_path = output.join(&jsonl_filename);
    let repeat_path = output.join(format!("{stem}.repeat.jsonl"));
    let feedback = Some(ExperimentFeedback { policy, mode });
    let mut first = record_with_motor(
        Robot::G1,
        case,
        false,
        Some(profile),
        Some(RunDiagnostics::enabled(&jsonl_path)?),
        feedback,
        motor_run(output, &stem, motor_enabled, false)?,
    )?;
    require(
        first.trace["observation_pipeline"]["world_seed"] == 2002,
        "G1 experiment seed must be 2002",
    )?;
    {
        let repeat = record_with_motor(
            Robot::G1,
            case,
            false,
            Some(profile),
            Some(RunDiagnostics::enabled(&repeat_path)?),
            feedback,
            motor_run(output, &stem, motor_enabled, true)?,
        )?;
        verify_diagnosis_behavior(&first, &repeat)?;
        verify_replay(&mut first, &repeat)?;
        diagnostic::require_same_bytes(&jsonl_path, &repeat_path)?;
        finish_motor_repeat(&mut first, output, &stem, motor_enabled)?;
    }
    fs::remove_file(&repeat_path)?;
    let off_filename = format!("{stem}.logging-off.json");
    let off_sha256;
    {
        let mut off = record_with_motor(
            Robot::G1,
            case,
            false,
            Some(profile),
            Some(RunDiagnostics::default()),
            feedback,
            if motor_enabled {
                MotorCapture::DisabledComparator
            } else {
                MotorCapture::Default
            },
        )?;
        verify_diagnosis_behavior(&first, &off)?;
        if motor_enabled {
            verify_motor_off(&mut first, &off)?;
        }
        off.trace["all_tick_diagnostic_logging_enabled"] = json!(false);
        off_sha256 = write_diagnosis_trace(output, &off_filename, &off.trace)?;
    }
    let run = first.diagnostics.as_ref().expect("experiment diagnostics");
    let mut artifact = run.artifact.clone().expect("experiment JSONL");
    require(
        artifact["rows"] == first.state_bits.len(),
        "experiment JSONL missed a control tick",
    )?;
    artifact["file"] = json!(jsonl_filename);
    artifact["bytes_exact_fresh_repeat"] = json!(true);
    let decision_hash = stable_hash(&run.decision_words);
    let decision_word_count: usize = run.decision_words.iter().map(Vec::len).sum();
    let literal_sidecar = if ["ideal_reference", "bounded_error"].contains(&profile.name) {
        Some(write_orientation_words(output, &stem, &first)?)
    } else {
        None
    };
    first.trace["all_tick_diagnostic_logging_enabled"] = json!(true);
    first.trace["all_tick_diagnostic"] = artifact.clone();
    first.trace["diagnosis_existing_decision_fields_hash"] = json!(decision_hash);
    first.trace["diagnosis_existing_decision_fields_word_count"] = json!(decision_word_count);
    first.trace["diagnosis_existing_decision_fields_exact_fresh_repeat"] = json!(true);
    first.trace["diagnosis_logging_on_off_plant_control_decisions_exact"] = json!(true);
    first.trace["zero_age_literal_words"] = json!(literal_sidecar);
    let filename = format!("{stem}.json");
    let trace_sha256 = write_diagnosis_trace(output, &filename, &first.trace)?;
    let settlement_rows = first.state_bits.len().min(2000);
    let mut report = json!({
        "g1_orientation_age_experiment":true,"g1_observation_diagnosis":true,
        "robot":"G1","feedback_policy":policy.name(),"feedback_policy_configuration":policy.configuration(),
        "profile":profile.name,"input":input,"impulse_n_s":impulse_ns,
        "physics_period_ticks":case.dt_ticks,"controller_period_ticks":case.dt_ticks,
        "trace":filename,"trace_sha256":trace_sha256,"logging_off_trace":off_filename,"logging_off_trace_sha256":off_sha256,
        "compiled_source_sha256":first.trace["compiled_source_sha256"],"plant":first.trace["plant"],"controller":first.trace["controller"],
        "observation_pipeline":first.trace["observation_pipeline"],"observation_metrics":first.trace["observation_metrics"],
        "settlement":first.trace["settlement"],"summary":first.trace["summary"],
        "planned_integrated_impulse_world_ns":first.trace["planned_integrated_impulse_world_ns"],
        "actual_applied_impulse_world_ns":first.trace["actual_applied_impulse_world_ns"],
        "observed_state_hash":first.trace["observed_state_hash"],"observed_state_word_count":first.trace["observed_state_word_count"],
        "observation_control_state_hash":first.trace["observation_control_state_hash"],"observation_control_state_word_count":first.trace["observation_control_state_word_count"],
        "observed_state_bits_exact_repeat":true,"observation_control_state_bits_exact_repeat":true,
        "existing_decision_fields_hash":decision_hash,"existing_decision_fields_word_count":decision_word_count,
        "existing_decision_fields_exact_fresh_repeat":true,"diagnostic_jsonl":artifact,"zero_age_literal_words":literal_sidecar,
        "settlement_state_hash":stable_hash(&first.state_bits[..settlement_rows]),
        "settlement_control_hash":stable_hash(&first.control_state_words[..settlement_rows]),
        "settlement_decision_hash":stable_hash(&run.decision_words[..settlement_rows]),
        "logging_on_off_plant_words_exact":true,"logging_on_off_control_words_exact":true,
        "logging_on_off_existing_decision_fields_exact":true,"logging_on_off_existing_trace_outcomes_exact":true,
        "fresh_world_runs":3,"logging_on_fresh_runs":2,"logging_off_fresh_runs":1
    });
    if mode == ExperimentMode::LateralGyro {
        report["g1_lateral_gyro_experiment"] = json!(true);
    }
    copy_motor_metadata(&mut report, &first.trace);
    Ok((first, report))
}

fn orientation_outcome_gates(cases: &[Value], complete: bool, cross_policy: &[Value]) -> Value {
    let outcome = |policy, profile, input| {
        cases
            .iter()
            .find(|case| {
                case["feedback_policy"] == policy
                    && case["profile"] == profile
                    && case["input"] == input
            })
            .and_then(|case| case["summary"]["recovered"].as_bool())
    };
    let primary = match (
        outcome("raw_reference", "delay_5ms", "kick"),
        outcome("orientation_known_age", "delay_5ms", "kick"),
    ) {
        (Some(raw), Some(candidate)) => Some(!raw && candidate),
        _ => None,
    };
    let zero_preserved = complete.then(|| {
        diagnostic::PROFILES.iter().all(|profile| {
            outcome("raw_reference", profile, "zero") == Some(true)
                && outcome("orientation_known_age", profile, "zero") == Some(true)
        })
    });
    json!({"primary_delay_5ms_kick_failure_to_recovery":primary,"all_four_zero_input_recoveries_preserved":zero_preserved,
        "zero_age_four_cases_literal_plant_control_old_decisions_exact":complete.then(||cross_policy.len() == 4 && cross_policy.iter().all(|case|case["bytes_exact"] == true)),
        "secondary_delay_20ms_kick_recovered":outcome("orientation_known_age","delay_20ms_limit","kick"),
        "bounded_error_kick_unchanged_negative_control":match (outcome("raw_reference","bounded_error","kick"),outcome("orientation_known_age","bounded_error","kick")) {
            (Some(raw),Some(candidate))=>Some(!raw && !candidate),_=>None},
        "default_policy":"raw_reference","candidate_promoted":false,
        "scope":"physical recovery uses the unchanged complete predicate; primary failure and zero regressions remain outcomes, and no result promotes this opt-in candidate"})
}

fn lateral_gyro_outcome_gates(cases: &[Value], complete: bool, cross_policy: &[Value]) -> Value {
    let outcome = |policy, profile, input| {
        cases
            .iter()
            .find(|case| {
                case["feedback_policy"] == policy
                    && case["profile"] == profile
                    && case["input"] == input
            })
            .and_then(|case| case["summary"]["recovered"].as_bool())
    };
    let reference = Policy::OrientationKnownAge.name();
    let candidate = Policy::OrientationKnownAgeLateralGyroEma5ms.name();
    let primary = match (
        outcome(reference, "delay_20ms_limit", "kick"),
        outcome(candidate, "delay_20ms_limit", "kick"),
    ) {
        (Some(reference), Some(candidate)) => Some(!reference && candidate),
        _ => None,
    };
    let delay_5ms = match (
        outcome(reference, "delay_5ms", "kick"),
        outcome(candidate, "delay_5ms", "kick"),
    ) {
        (Some(reference), Some(candidate)) => Some(reference && candidate),
        _ => None,
    };
    let zeros = complete.then(|| {
        diagnostic::PROFILES.iter().all(|profile| {
            outcome(reference, profile, "zero") == Some(true)
                && outcome(candidate, profile, "zero") == Some(true)
        })
    });
    let age_zero = complete.then(|| {
        cross_policy.len() == 4 && cross_policy.iter().all(|case| case["bytes_exact"] == true)
    });
    let negative = match (
        outcome(reference, "bounded_error", "kick"),
        outcome(candidate, "bounded_error", "kick"),
    ) {
        (Some(reference), Some(candidate)) => Some(!reference && !candidate),
        _ => None,
    };
    json!({
        "primary_delay_20ms_kick_failure_to_recovery":primary,
        "delay_5ms_kick_recovery_preserved":delay_5ms,
        "all_four_zero_input_recoveries_preserved":zeros,
        "zero_age_four_cases_literal_plant_control_old_decisions_exact":age_zero,
        "bounded_error_kick_unchanged_negative_control":negative,
        "candidate_advancement_gates_passed":complete.then(|| [primary,delay_5ms,zeros,age_zero,negative].iter().all(|gate| *gate == Some(true))),
        "default_policy":"raw_reference","candidate_promoted":false,
        "scope":"unchanged complete physical recovery predicate; failures are retained and completion does not require recovery; no gain, time constant, plant, cap, horizon or predicate tuning"
    })
}

/// Evaluates the fixed orientation-only candidate without changing default feedback.
pub(super) fn experiment_g1_orientation_age(
    output: &Path,
    policy_filter: Option<&str>,
    profile_filter: Option<&str>,
    input_filter: Option<&str>,
    motor_enabled: bool,
) -> CaptureResult<()> {
    experiment_g1_feedback(
        output,
        policy_filter,
        profile_filter,
        input_filter,
        ExperimentMode::OrientationAge,
        motor_enabled,
    )
}

/// Evaluates a fixed capture-time gyro filter without promoting it to default.
pub(super) fn experiment_g1_lateral_gyro(
    output: &Path,
    policy_filter: Option<&str>,
    profile_filter: Option<&str>,
    input_filter: Option<&str>,
    motor_enabled: bool,
) -> CaptureResult<()> {
    experiment_g1_feedback(
        output,
        policy_filter,
        profile_filter,
        input_filter,
        ExperimentMode::LateralGyro,
        motor_enabled,
    )
}

fn experiment_g1_feedback(
    output: &Path,
    policy_filter: Option<&str>,
    profile_filter: Option<&str>,
    input_filter: Option<&str>,
    mode: ExperimentMode,
    motor_enabled: bool,
) -> CaptureResult<()> {
    let policies = mode.policies(policy_filter)?;
    let profiles = diagnostic::profile_names(profile_filter)?;
    let inputs = diagnostic::inputs(input_filter)?;
    fs::create_dir_all(output)?;
    let mut cases = Vec::new();
    let mut prefixes = Vec::new();
    let mut cross_policy = Vec::new();
    // Each mode's full study completes all eight fixed references before candidates.
    for policy in &policies {
        for name in &profiles {
            let profile = observation::profiles(Some(name))?[0];
            require(
                !profile.zero_input && profile.sample_period_ticks == diagnostic::DT_TICKS,
                "experiment profile configuration changed",
            )?;
            let mut zero_prefix: Option<DiagnosisPrefix> = None;
            for &(input, impulse_ns) in &inputs {
                let (first, report) = feedback_experiment_case(
                    output,
                    profile,
                    input,
                    impulse_ns,
                    *policy,
                    mode,
                    motor_enabled,
                )?;
                if policies.len() == 2
                    && *policy == mode.candidate()
                    && ["ideal_reference", "bounded_error"].contains(name)
                {
                    let raw_filename = format!(
                        "{}-{}-{name}-{input}.words.bin",
                        mode.stem(),
                        mode.reference().name()
                    );
                    let candidate_filename = report["zero_age_literal_words"]["file"]
                        .as_str()
                        .expect("zero-age sidecar");
                    diagnostic::require_same_bytes(
                        &output.join(&raw_filename),
                        &output.join(candidate_filename),
                    )?;
                    cross_policy.push(json!({"profile":name,"input":input,"bytes_exact":true,
                        "raw_file":raw_filename,"candidate_file":candidate_filename,
                        "rows":first.state_bits.len(),"scope":"full plant/control/typed old-decision vectors compared literally, including settlement; no policy/context fields in these words"}));
                }
                cases.push(report);
                if inputs.len() == 2 {
                    if input == "zero" {
                        zero_prefix = Some(DiagnosisPrefix::from_recording(first));
                    } else {
                        let mut prefix = zero_prefix
                            .take()
                            .expect("zero first")
                            .compare(&first, name)?;
                        prefix["feedback_policy"] = json!(policy.name());
                        if mode == ExperimentMode::LateralGyro {
                            prefix["planned_prefix_completed_steps"] = json!(4000);
                            prefix["full_pre_input_prefix_evaluated"] =
                                json!(prefix["completed_steps"] == 4000);
                            prefix["unavailable_reason"] = if prefix["completed_steps"] == 4000 {
                                Value::Null
                            } else {
                                json!("failed settlement; only existing pre-input rows compared")
                            };
                        }
                        prefixes.push(prefix);
                    }
                }
            }
        }
    }
    let complete = policies.len() == 2 && profiles.len() == 4 && inputs.len() == 2;
    let gates = match mode {
        ExperimentMode::OrientationAge => {
            orientation_outcome_gates(&cases, complete, &cross_policy)
        }
        ExperimentMode::LateralGyro => lateral_gyro_outcome_gates(&cases, complete, &cross_policy),
    };
    let mut report = json!({
        "schema_version":1,"g1_orientation_age_experiment":true,"g1_observation_diagnosis":true,
        "experiment_completed":true,"complete_matrix":complete,
        "predeclared_policies":[mode.reference().name(),mode.candidate().name()],
        "selected_policies":policies.iter().map(Policy::name).collect::<Vec<_>>(),"selected_profiles":profiles,
        "selected_inputs":inputs.iter().map(|(name,_)|*name).collect::<Vec<_>>(),
        "world_seed":2002,"physics_period_ticks":diagnostic::DT_TICKS,"controller_period_ticks":diagnostic::DT_TICKS,
        "settlement_s":SETTLE_S,"recording_horizon_s":6.2,"pulse_duration_s":DURATION_S,
        "maximum_control_rows_per_case":diagnostic::MAX_ROWS,
        "execution_scope":"raw eight before candidate eight; sequential worlds, first and one comparator full literal recording plus one bounded zero/kick prefix; streamed JSONL; zero-age cross-policy literal sidecars on disk",
        "recovery_required_for_completion":false,"recovery_predicate_changed":false,"default_policy_changed":false,
        "hardware_validation":false,"identified_sensor_model":false,"inner_pd_feedback":"ideal instantaneous joint coordinates unchanged",
        "matched_zero_kick_prefixes":prefixes,"zero_age_across_policy_literal_comparisons":cross_policy,
        "measured_outcome_gates":gates,"cases":cases
    });
    if mode == ExperimentMode::LateralGyro {
        report["g1_lateral_gyro_experiment"] = json!(true);
        report["execution_scope"] = json!("orientation-known-age eight before fixed lateral-gyro EMA eight; sequential worlds, first and one comparator full literal recording plus one bounded zero/kick prefix; streamed JSONL; zero-age cross-policy literal sidecars on disk");
    }
    write_motor_report(
        output,
        report["cases"].as_array().expect("experiment cases"),
        motor_enabled,
    )?;
    fs::write(
        output.join(format!("{}-experiment.json", mode.stem())),
        serde_json::to_vec_pretty(&report)?,
    )?;
    Ok(())
}

fn record_case_matrix(
    robots: &[Robot],
    cases: &[Case],
    output: &Path,
    require_recovery: bool,
    motor_enabled: bool,
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
                    validate_robot_cases(robot, cases, output, require_recovery, motor_enabled)
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
    motor_enabled: bool,
) -> CaptureResult<Vec<Value>> {
    let mut reports = Vec::new();
    for &case in cases {
        let trace = replay_with_motor(
            robot,
            case,
            false,
            output,
            &format!("{}-{}", robot.name().to_lowercase(), case.name),
            motor_enabled,
        )?;
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
            "observation_pipeline":trace["observation_pipeline"],
            "observation_control_state_hash":trace["observation_control_state_hash"],
            "observation_control_state_bits_exact_repeat":true,
            "observed_state_hash":trace["observed_state_hash"],"observed_state_bits_exact_repeat":true}));
        copy_motor_metadata(reports.last_mut().expect("new recovery case"), &trace);
    }
    Ok(reports)
}

#[cfg(test)]
mod diagnosis_tests {
    use super::*;

    #[test]
    fn motor_readback_is_post_step_and_leaves_old_plant_control_decisions_exact() {
        let output =
            std::env::temp_dir().join(format!("rne-g1-motor-readback-{}", std::process::id()));
        fs::create_dir_all(&output).unwrap();
        let mut capability_probe = Robot::G1.load(diagnostic::DT_TICKS).unwrap();
        if !capability_probe
            .configure_position_motor_diagnostics(true)
            .unwrap()
        {
            assert_eq!(capability_probe.sim_time().ticks(), 0);
            let error = record_with_motor(
                Robot::G1,
                NOMINAL,
                false,
                None,
                None,
                None,
                MotorCapture::Enabled(Box::new(
                    MotorRun::enabled(&output.join("unsupported.motor.jsonl")).unwrap(),
                )),
            )
            .unwrap_err();
            assert!(error
                .to_string()
                .contains("--features position-motor-diagnostics"));
            fs::remove_dir_all(output).unwrap();
            return;
        }
        let mut legacy = Vec::new();
        let mut motor_words = Vec::new();
        for (index, enabled) in [true, true, false].into_iter().enumerate() {
            let mut sim = Robot::G1.load(diagnostic::DT_TICKS).unwrap();
            let mut controller = Controller::new(
                Robot::G1,
                &mut sim,
                true,
                Profile::ideal(diagnostic::DT_TICKS),
            )
            .unwrap();
            controller.diagnostics = Some(RunDiagnostics::default());
            assert!(sim.configure_position_motor_diagnostics(enabled).unwrap());
            assert_eq!(sim.sim_time().ticks(), 0);
            let path = output.join(format!("world-{index}.motor.jsonl"));
            let mut motor = enabled.then(|| MotorRun::enabled(&path).unwrap());
            let mut states = Vec::new();
            let mut controls = Vec::new();
            for _ in 0..3 {
                step_with_motor_diagnostics(&mut controller, &mut sim, &mut motor).unwrap();
                states.push(state_bits(Robot::G1, &sim));
                controls.push(controller.replay_words());
            }
            let decisions = controller
                .diagnostics
                .take()
                .unwrap()
                .finish()
                .unwrap()
                .decision_words;
            legacy.push((states, controls, decisions));
            if let Some(motor) = motor {
                let run = motor.finish().unwrap();
                assert_eq!(run.words.len(), 3);
                let rows: Vec<Value> = fs::read_to_string(&path)
                    .unwrap()
                    .lines()
                    .map(|row| serde_json::from_str(row).unwrap())
                    .collect();
                for (row_index, row) in rows.iter().enumerate() {
                    assert_eq!(
                        row["command_consumer_ticks"],
                        row_index as u64 * diagnostic::DT_TICKS
                    );
                    assert_eq!(
                        row["completed_interval_end_ticks"],
                        (row_index as u64 + 1) * diagnostic::DT_TICKS
                    );
                    let samples = row["samples"].as_array().unwrap();
                    assert_eq!(samples.len(), 23);
                    assert!(samples
                        .iter()
                        .any(|sample| sample["availability"] == "available"));
                    assert!(samples
                        .windows(2)
                        .all(|pair| pair[0]["link"].as_str() < pair[1]["link"].as_str()));
                }
                motor_words.push(run.words);
            } else {
                assert!(!path.exists());
            }
        }
        assert_eq!(legacy[0], legacy[1]);
        assert_eq!(legacy[0], legacy[2]);
        assert_eq!(motor_words[0], motor_words[1]);
        diagnostic::require_same_bytes(
            &output.join("world-0.motor.jsonl"),
            &output.join("world-1.motor.jsonl"),
        )
        .unwrap();
        fs::remove_dir_all(output).unwrap();
    }

    #[test]
    fn experiment_modes_validate_fixed_filters_before_creating_outputs() {
        let output =
            std::env::temp_dir().join(format!("rne-g1-rejected-study-{}", std::process::id()));
        assert!(!output.exists());
        for mode in [ExperimentMode::OrientationAge, ExperimentMode::LateralGyro] {
            for filters in [
                (Some("unknown"), None, None),
                (None, Some("unknown"), None),
                (None, None, Some("unknown")),
            ] {
                assert!(experiment_g1_feedback(
                    &output, filters.0, filters.1, filters.2, mode, false
                )
                .is_err());
                assert!(!output.exists());
            }
        }
        assert!(ExperimentMode::OrientationAge
            .policies(Some("orientation_known_age_lateral_gyro_ema_5ms"))
            .is_err());
        assert!(ExperimentMode::LateralGyro
            .policies(Some("raw_reference"))
            .is_err());
    }

    #[test]
    fn lateral_markers_are_explicit_and_do_not_change_old_orientation_metadata() {
        let mut old = json!({"summary":{"recovered":false},"controller":{"existing":true}});
        mark_orientation_experiment(&mut old, Policy::OrientationKnownAge);
        assert!(old.get("g1_lateral_gyro_experiment").is_none());
        assert_eq!(old["controller"], json!({"existing":true}));
        let mut lateral = old.clone();
        mark_lateral_experiment(&mut lateral);
        for scope in [&lateral, &lateral["summary"], &lateral["controller"]] {
            for marker in [
                "g1_lateral_gyro_experiment",
                "g1_orientation_age_experiment",
                "g1_observation_diagnosis",
            ] {
                assert_eq!(scope[marker], true);
            }
            assert_eq!(scope["feedback_policy"], "orientation_known_age");
        }
        assert_eq!(lateral["summary"]["recovered"], false);
        assert_eq!(lateral["controller"]["existing"], true);
    }

    #[test]
    fn lateral_gyro_zero_age_keeps_literal_world_control_and_old_decisions() {
        let output =
            std::env::temp_dir().join(format!("rne-g1-gyro-age-zero-{}", std::process::id()));
        fs::create_dir_all(&output).unwrap();
        for name in ["ideal_reference", "bounded_error"] {
            let profile = observation::profiles(Some(name)).unwrap()[0];
            let mut results = Vec::new();
            for policy in Policy::selected_lateral(None).unwrap() {
                let mut sim = Robot::G1.load(diagnostic::DT_TICKS).unwrap();
                let mut controller = Controller::new(Robot::G1, &mut sim, true, profile).unwrap();
                controller.feedback_policy = policy;
                controller.orientation_age_experiment = true;
                controller.lateral_gyro_experiment = true;
                let path = output.join(format!("{}-{name}.jsonl", policy.name()));
                controller.diagnostics = Some(RunDiagnostics::enabled(&path).unwrap());
                let mut states = Vec::new();
                let mut controls = Vec::new();
                for _ in 0..3 {
                    controller.step(&mut sim).unwrap();
                    assert!(controller.estimator.replay_words().is_empty());
                    states.push(state_bits(Robot::G1, &sim));
                    controls.push(controller.replay_words());
                }
                let run = controller.diagnostics.take().unwrap().finish().unwrap();
                for row in fs::read_to_string(&path).unwrap().lines() {
                    let row: Value = serde_json::from_str(row).unwrap();
                    assert_eq!(row["g1_lateral_gyro_experiment"], true);
                    assert_eq!(row["g1_orientation_age_experiment"], true);
                    assert_eq!(row["g1_observation_diagnosis"], true);
                    assert_eq!(
                        row["delivered_estimate"],
                        row["effective_feedback_estimate"]
                    );
                    assert_eq!(
                        row["lateral_gyro_control_replay"]["suffix_words"],
                        json!([])
                    );
                    assert!(row["decision"].get("lateral_gyro_filter_after").is_none());
                }
                results.push((states, controls, run.decision_words));
            }
            assert_eq!(results[0], results[1]);
        }
        fs::remove_dir_all(output).unwrap();
    }

    #[test]
    fn lateral_gyro_diagnostic_suffix_is_actual_control_state_and_logging_is_causal() {
        let output =
            std::env::temp_dir().join(format!("rne-g1-gyro-causal-{}", std::process::id()));
        fs::create_dir_all(&output).unwrap();
        let profile = observation::profiles(Some("delay_20ms_limit")).unwrap()[0];
        let mut results = Vec::new();
        for index in 0..3 {
            let mut sim = Robot::G1.load(diagnostic::DT_TICKS).unwrap();
            let mut controller = Controller::new(Robot::G1, &mut sim, true, profile).unwrap();
            controller.feedback_policy = Policy::OrientationKnownAgeLateralGyroEma5ms;
            controller.orientation_age_experiment = true;
            controller.lateral_gyro_experiment = true;
            let path = output.join(format!("run-{index}.jsonl"));
            controller.diagnostics = Some(if index < 2 {
                RunDiagnostics::enabled(&path).unwrap()
            } else {
                RunDiagnostics::default()
            });
            let mut states = Vec::new();
            let mut controls = Vec::new();
            for tick in 0..25 {
                controller.step(&mut sim).unwrap();
                assert_eq!(controller.estimator.replay_words().is_empty(), tick < 20);
                states.push(state_bits(Robot::G1, &sim));
                controls.push(controller.replay_words());
            }
            let run = controller.diagnostics.take().unwrap().finish().unwrap();
            if index < 2 {
                for (tick, line) in fs::read_to_string(&path).unwrap().lines().enumerate() {
                    let row: Value = serde_json::from_str(line).unwrap();
                    let replay = &row["lateral_gyro_control_replay"];
                    let start = replay["suffix_start_word"].as_u64().unwrap() as usize;
                    let suffix: Vec<u64> = replay["suffix_words"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .map(|word| word.as_u64().unwrap())
                        .collect();
                    assert_eq!(replay["full_control_word_count"], controls[tick].len());
                    assert_eq!(suffix, controls[tick][start..]);
                    assert_eq!(suffix.is_empty(), tick < 20);
                    assert!(row["decision"].get("lateral_gyro_filter_after").is_none());
                }
            }
            results.push((states, controls, run.decision_words));
        }
        assert_eq!(results[0], results[1]);
        assert_eq!(results[0], results[2]);
        diagnostic::require_same_bytes(&output.join("run-0.jsonl"), &output.join("run-1.jsonl"))
            .unwrap();
        fs::remove_dir_all(output).unwrap();
    }

    #[test]
    fn zero_age_policies_preserve_literal_world_control_and_old_decision_sidecars() {
        let output = std::env::temp_dir().join(format!("rne-g1-zero-age-{}", std::process::id()));
        fs::create_dir_all(&output).unwrap();
        for name in ["ideal_reference", "bounded_error"] {
            let profile = observation::profiles(Some(name)).unwrap()[0];
            let mut records = Vec::new();
            for policy in Policy::selected(None).unwrap() {
                let mut sim = Robot::G1.load(diagnostic::DT_TICKS).unwrap();
                let mut controller = Controller::new(Robot::G1, &mut sim, true, profile).unwrap();
                controller.feedback_policy = policy;
                controller.orientation_age_experiment = true;
                let stem = format!("{}-{name}", policy.name());
                controller.diagnostics =
                    Some(RunDiagnostics::enabled(&output.join(format!("{stem}.jsonl"))).unwrap());
                let mut states = Vec::new();
                let mut controls = Vec::new();
                for _ in 0..3 {
                    controller.step(&mut sim).unwrap();
                    states.push(state_bits(Robot::G1, &sim));
                    controls.push(controller.replay_words());
                }
                let recording = Recording {
                    trace: Value::Null,
                    state_bits: states,
                    control_state_words: controls,
                    diagnostics: Some(controller.diagnostics.take().unwrap().finish().unwrap()),
                    motor_diagnostics: None,
                };
                let sidecar = write_orientation_words(&output, &stem, &recording).unwrap();
                let bytes = fs::read(output.join(sidecar["file"].as_str().unwrap())).unwrap();
                assert_eq!(&bytes[..16], b"RNE_G1_WORDS_V1\0");
                assert_eq!(u64::from_le_bytes(bytes[16..24].try_into().unwrap()), 3);
                assert_eq!(sidecar["sha256"], format!("{:x}", Sha256::digest(&bytes)));
                let lines: Vec<Value> = fs::read_to_string(output.join(format!("{stem}.jsonl")))
                    .unwrap()
                    .lines()
                    .map(|line| serde_json::from_str(line).unwrap())
                    .collect();
                for row in lines {
                    assert_eq!(row["feedback_policy"], policy.name());
                    assert_eq!(
                        row["delivered_estimate"],
                        row["effective_feedback_estimate"]
                    );
                    assert_eq!(row["observation_timing"]["observation_age_ticks"], 0);
                    assert!(row["decision"].get("feedback_policy").is_none());
                }
                records.push(recording);
            }
            assert_eq!(records[0].state_bits, records[1].state_bits);
            assert_eq!(
                records[0].control_state_words,
                records[1].control_state_words
            );
            assert_eq!(
                diagnosis_decisions(&records[0]).unwrap(),
                diagnosis_decisions(&records[1]).unwrap()
            );
            diagnostic::require_same_bytes(
                &output.join(format!("raw_reference-{name}.words.bin")),
                &output.join(format!("orientation_known_age-{name}.words.bin")),
            )
            .unwrap();
        }
        fs::remove_dir_all(output).unwrap();
    }

    #[test]
    fn all_tick_logging_preserves_real_startup_decisions_and_world_bits() {
        let output =
            std::env::temp_dir().join(format!("rne-g1-diagnosis-causal-{}", std::process::id()));
        fs::create_dir_all(&output).unwrap();
        for name in diagnostic::PROFILES {
            let profile = observation::profiles(Some(name)).unwrap()[0];
            let mut results = Vec::new();
            for run in 0..3 {
                let mut sim = Robot::G1.load(diagnostic::DT_TICKS).unwrap();
                let mut controller = Controller::new(Robot::G1, &mut sim, true, profile).unwrap();
                controller.diagnostics = Some(if run < 2 {
                    RunDiagnostics::enabled(&output.join(format!("{name}-{run}.jsonl"))).unwrap()
                } else {
                    RunDiagnostics::default()
                });
                let impact = Impact::new(23, 80, 1e-3, 40.0, Vec3::Z);
                let mut states = Vec::new();
                let mut controls = Vec::new();
                for step in 1..=25 {
                    apply_impact(Robot::G1, &mut sim, &impact, step).unwrap();
                    controller.step(&mut sim).unwrap();
                    states.push(state_bits(Robot::G1, &sim));
                    controls.push(controller.replay_words());
                }
                let diagnosis = controller.diagnostics.take().unwrap().finish().unwrap();
                assert_eq!(diagnosis.decision_words.len(), 25);
                if let Some(artifact) = &diagnosis.artifact {
                    assert_eq!(artifact["rows"], 25);
                }
                results.push((states, controls, diagnosis.decision_words));
            }
            assert_eq!(results[0], results[1], "fresh replay {name}");
            assert_eq!(results[0], results[2], "logging noninterference {name}");
            let first_path = output.join(format!("{name}-0.jsonl"));
            diagnostic::require_same_bytes(&first_path, &output.join(format!("{name}-1.jsonl")))
                .unwrap();
            let lines: Vec<Value> = fs::read_to_string(first_path)
                .unwrap()
                .lines()
                .map(|line| serde_json::from_str(line).unwrap())
                .collect();
            assert_eq!(lines[0]["consumer_ticks"], 0);
            assert_eq!(lines[0]["evaluator_current_truth"]["consumer_ticks"], 0);
            assert_eq!(
                lines[0]["evaluator_current_truth"]["contact_solution_available"],
                false
            );
            assert_eq!(
                lines[0]["decision"]["joint_feedback"]
                    .as_array()
                    .unwrap()
                    .len(),
                23
            );
            for (index, row) in lines.iter().enumerate() {
                assert_eq!(row["consumer_ticks"], index as u64 * diagnostic::DT_TICKS);
                assert_eq!(
                    row["evaluator_current_truth"]["consumer_ticks"],
                    row["consumer_ticks"]
                );
                assert_eq!(
                    row["command_completed_ticks"],
                    (index as u64 + 1) * diagnostic::DT_TICKS
                );
                let capture = row["observation_timing"]["capture_ticks"].as_u64();
                if let Some(capture) = capture {
                    assert!(capture <= row["consumer_ticks"].as_u64().unwrap());
                    let captured_truth = &lines[(capture / diagnostic::DT_TICKS) as usize]
                        ["evaluator_current_truth"]["observation"];
                    assert_eq!(&row["evaluator_delivered_capture_truth"], captured_truth);
                    if name != "bounded_error" {
                        assert_eq!(&row["delivered_estimate"], captured_truth);
                    }
                } else {
                    assert!(row["delivered_estimate"].is_null());
                    assert!(row["evaluator_delivered_capture_truth"].is_null());
                    assert_eq!(row["decision"]["no_observation"], true);
                }
            }
            if name == "delay_20ms_limit" {
                assert!(lines[..20]
                    .iter()
                    .all(|row| row["delivered_estimate"].is_null()));
                assert_eq!(lines[20]["observation_timing"]["capture_ticks"], 0);
                assert_eq!(
                    lines[20]["observation_timing"]["observation_age_ticks"],
                    20 * diagnostic::DT_TICKS
                );
                assert_ne!(
                    lines[20]["evaluator_delivered_capture_truth"],
                    lines[20]["evaluator_current_truth"]["observation"]
                );
            }
        }
        fs::remove_dir_all(output).unwrap();
    }
}
