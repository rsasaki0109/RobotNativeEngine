//! Collects one unchanged example-138 held scene with opt-in motor-stage data.
//!
//! The gzip JSONL stream retains actual native words, native entity checkpoint
//! payloads and completed ECS state. Collection does not certify calibration,
//! contact convergence, physical grasp acceptance or a cause of trajectory changes.

use flate2::write::GzEncoder;
use flate2::{Compression, GzBuilder};
use rne_core::SimDuration;
use rne_ecs::{Entity, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    Collider, JointActuation, JointMotorGainModel, JointState, MultibodyLink, PhysicsBackend,
    PhysicsMaterial, PhysicsWorldDesc, PhysicsWorldId, PrismaticJointDesc, RigidBody,
    RigidBodyType,
};
use rne_physics_rapier::RapierBackend;
use rne_world::Transform3;
use serde::Serialize;
use std::any::Any;
use std::fs::OpenOptions;
use std::io::{BufWriter, Write};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::path::PathBuf;

/// Errors preserve collection, serialization, I/O and finalization failures.
pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error + Send + Sync>>;

const SCHEMA_VERSION: u8 = 1;
const EXPECTED_ENTITY_COUNT: usize = 6;
const STEP_HZ: f64 = 500.0;
const GRAVITY_M_S2: f64 = 9.81;
const FRICTION: f32 = 0.8;
const CUBE_HALF_M: f64 = 0.05;
const CUBE_MASSES_KG: [f64; 3] = [5.0, 20.0, 50.0];
/// Height of the palm's center above the floor while it reaches down.
const PALM_Y_M: f64 = 0.26;
/// The finger pads hang this far below the palm's center.
const FINGER_DROP_M: f64 = 0.19;
const FINGER_X_M: f64 = 0.08;
const LIFT_M: f64 = 0.2;
const SETTLE_STEPS: usize = 250;
const LIFT_STEPS: usize = 500;
const HOLD_STEPS: usize = 500;

fn collider(half: Vec3) -> Collider {
    Collider {
        material: PhysicsMaterial {
            friction: FRICTION,
            ..PhysicsMaterial::default()
        },
        ..Collider::cuboid(half)
    }
}

/// The palm's lift servo at `target_m` above where it starts.
fn lift(target_m: f64) -> JointActuation {
    JointActuation::PrismaticPosition {
        target_position_m: target_m,
        stiffness_n_per_m: 1.0e5,
        damping_n_s_per_m: 1.0e4,
        max_force_n: 5000.0,
    }
}

/// The force per finger that holds a cube of `mass_kg` without margin.
fn holding_squeeze_n(mass_kg: f64) -> f64 {
    mass_kg * GRAVITY_M_S2 / (2.0 * f64::from(FRICTION))
}

const TOTAL_STEPS: usize = SETTLE_STEPS + LIFT_STEPS + HOLD_STEPS;
const CHECKPOINT_CODEC: &str = "postcard-1.1.3-raw-native";
const CHECKPOINT_SCOPE: &str = "sorted individual native bodies, colliders, unique multibodies, impulse joints, gravity and integration parameters; excludes full-set metadata and private pipeline/solver/contact caches";

/// Physical result with IEEE words and example 138's physical display row.
#[derive(Clone, Debug, Serialize)]
pub struct PhysicalReport {
    /// ECS pose words: translation xyz, quaternion xyzw, then scale xyz.
    pub cube_ecs_pose_ieee_words: Vec<u64>,
    /// Lift (m), palm rise (m), slip (m), tilt (rad), as native f64 words.
    pub physical_ieee_words: [u64; 4],
    /// Example 138's Rapier line without the timing field or trailing newline.
    pub physical_stdout_row: String,
}

#[derive(Clone, Debug)]
struct Options {
    mass_kg: f64,
    observer_enabled: bool,
    label: String,
    output: PathBuf,
}

impl Options {
    fn parse(args: impl IntoIterator<Item = String>) -> Result<Option<Self>> {
        let arguments: Vec<String> = args.into_iter().collect();
        if arguments == ["--help"] {
            return Ok(None);
        }
        let mut mass_kg = None;
        let mut observer_enabled = None;
        let mut label = None;
        let mut output = None;
        let mut arguments = arguments.into_iter();
        while let Some(argument) = arguments.next() {
            let value = arguments
                .next()
                .ok_or_else(|| format!("{argument} requires a value"))?;
            match argument.as_str() {
                "--mass-kg" if mass_kg.is_none() => {
                    mass_kg = Some(match value.as_str() {
                        "5" => 5.0,
                        "20" => 20.0,
                        "50" => 50.0,
                        _ => return Err("--mass-kg must be exactly 5, 20 or 50".into()),
                    });
                }
                "--observer" if observer_enabled.is_none() => {
                    observer_enabled = Some(match value.as_str() {
                        "on" => true,
                        "off" => false,
                        _ => return Err("--observer must be on or off".into()),
                    });
                }
                "--label" if label.is_none() => {
                    validate_label(&value)?;
                    label = Some(value);
                }
                "--output" if output.is_none() => {
                    if value.is_empty() {
                        return Err("--output cannot be empty".into());
                    }
                    output = Some(PathBuf::from(value));
                }
                _ => return Err(format!("unknown or repeated argument: {argument}").into()),
            }
        }
        Ok(Some(Self {
            mass_kg: mass_kg.ok_or("missing --mass-kg")?,
            observer_enabled: observer_enabled.ok_or("missing --observer")?,
            label: label.ok_or("missing --label")?,
            output: output.ok_or("missing --output")?,
        }))
    }
}

fn validate_label(label: &str) -> Result<()> {
    if label.is_empty()
        || label.len() > 64
        || !label
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_'))
    {
        return Err("label must contain 1..64 ASCII letters, digits, '-' or '_'".into());
    }
    Ok(())
}

#[derive(Debug, Serialize)]
struct DeclaredConditions {
    step_hz_ieee754_bits: u64,
    outer_duration_s_ieee754_bits: u64,
    gravity_y_m_s2_ieee754_bits: u64,
    friction_f32_ieee754_bits: u32,
    squeeze_n_ieee754_bits: u64,
    lift_distance_m_ieee754_bits: u64,
    lift_stiffness_n_per_m_ieee754_bits: u64,
    lift_damping_n_s_per_m_ieee754_bits: u64,
    lift_ceiling_n_ieee754_bits: u64,
    settle_steps: usize,
    lift_steps: usize,
    hold_steps: usize,
    expected_default_small_steps: usize,
    expected_default_pgs_passes: usize,
    expected_default_stabilization_passes: usize,
    expected_default_warmstart_f32_ieee754_bits: u32,
}

#[derive(Debug, Serialize)]
struct RunStart<'a> {
    kind: &'static str,
    schema_version: u8,
    run_label: &'a str,
    mass_kg_ieee754_bits: u64,
    observer_enabled: bool,
    expected_completed_outer_steps: usize,
    expected_ecs_entity_count: usize,
    ecs_word_schema_version: u8,
    checkpoint_codec: &'static str,
    checkpoint_scope: &'static str,
    declared_conditions: DeclaredConditions,
}

impl<'a> RunStart<'a> {
    fn new(mass_kg: f64, observer_enabled: bool, label: &'a str) -> Self {
        Self {
            kind: "run_start",
            schema_version: SCHEMA_VERSION,
            run_label: label,
            mass_kg_ieee754_bits: mass_kg.to_bits(),
            observer_enabled,
            expected_completed_outer_steps: TOTAL_STEPS,
            expected_ecs_entity_count: EXPECTED_ENTITY_COUNT,
            ecs_word_schema_version: 1,
            checkpoint_codec: CHECKPOINT_CODEC,
            checkpoint_scope: CHECKPOINT_SCOPE,
            declared_conditions: DeclaredConditions {
                step_hz_ieee754_bits: STEP_HZ.to_bits(),
                outer_duration_s_ieee754_bits: SimDuration::from_hertz(Hertz::new(STEP_HZ))
                    .as_seconds()
                    .value()
                    .to_bits(),
                gravity_y_m_s2_ieee754_bits: (-GRAVITY_M_S2).to_bits(),
                friction_f32_ieee754_bits: FRICTION.to_bits(),
                squeeze_n_ieee754_bits: (1.25 * holding_squeeze_n(mass_kg)).to_bits(),
                lift_distance_m_ieee754_bits: LIFT_M.to_bits(),
                lift_stiffness_n_per_m_ieee754_bits: 1.0e5_f64.to_bits(),
                lift_damping_n_s_per_m_ieee754_bits: 1.0e4_f64.to_bits(),
                lift_ceiling_n_ieee754_bits: 5000.0_f64.to_bits(),
                settle_steps: SETTLE_STEPS,
                lift_steps: LIFT_STEPS,
                hold_steps: HOLD_STEPS,
                expected_default_small_steps: 4,
                expected_default_pgs_passes: 1,
                expected_default_stabilization_passes: 2,
                expected_default_warmstart_f32_ieee754_bits: 1.0_f32.to_bits(),
            },
        }
    }
}

#[derive(Debug, Serialize)]
struct CompletedStep<'a, Checkpoint, Telemetry> {
    kind: &'static str,
    schema_version: u8,
    completed_outer_step: usize,
    native_checkpoint: &'a Checkpoint,
    native_checkpoint_postcard_bytes: Vec<u8>,
    ecs_physics_ieee_words: Vec<u64>,
    stage_telemetry: Option<&'a Telemetry>,
}

#[derive(Debug, Default)]
struct Progress {
    completed_outer_steps: usize,
    frames_written: usize,
}

#[derive(Debug, Serialize)]
struct RunEnd<'a> {
    kind: &'static str,
    schema_version: u8,
    status: &'static str,
    completed_outer_steps: usize,
    frames_written: usize,
    error: Option<String>,
    physical_report: Option<&'a PhysicalReport>,
}

#[derive(Debug)]
struct JsonLines<W> {
    output: W,
    progress: Progress,
}

impl<W: Write> JsonLines<W> {
    fn record(&mut self, record: &impl Serialize) -> Result<()> {
        // Buffer one record so a serialization error cannot leave half a JSON
        // record before the failure terminal. No completed trajectory is cached.
        let encoded = serde_json::to_vec(record)?;
        self.output.write_all(&encoded)?;
        self.output.write_all(b"\n")?;
        Ok(())
    }

    fn frame(&mut self, record: &impl Serialize) -> Result<()> {
        self.record(record)?;
        self.progress.frames_written += 1;
        Ok(())
    }
}

/// Runs one declared mass through the original ECS wiring and writes gzip JSONL.
///
/// Frames stream one at a time. Native checkpoint and stage fields are retained
/// literally; no residual, calibration predicate or contact force is inferred.
/// Errors and unwinding panics attempt a failed terminal before finalization.
/// A finalization/I/O failure is returned even if collection already completed.
pub fn run_mass_held<W: Write>(
    mass_kg: f64,
    observer_enabled: bool,
    label: &str,
    output: W,
) -> Result<PhysicalReport> {
    if !CUBE_MASSES_KG.contains(&mass_kg) {
        return Err("undeclared held mass".into());
    }
    validate_label(label)?;
    write_run(mass_kg, observer_enabled, label, output, |stream| {
        collect(mass_kg, observer_enabled, stream)
    })
}

fn write_run<W: Write>(
    mass_kg: f64,
    observer_enabled: bool,
    label: &str,
    output: W,
    collect: impl FnOnce(&mut JsonLines<GzEncoder<W>>) -> Result<PhysicalReport>,
) -> Result<PhysicalReport> {
    let encoder = GzBuilder::new()
        .mtime(0)
        .operating_system(255)
        .write(output, Compression::default());
    let mut stream = JsonLines {
        output: encoder,
        progress: Progress::default(),
    };
    let mut outcome = match stream.record(&RunStart::new(mass_kg, observer_enabled, label)) {
        Ok(()) => match catch_unwind(AssertUnwindSafe(|| collect(&mut stream))) {
            Ok(result) => result,
            Err(payload) => {
                Err(format!("collection panicked: {}", panic_message(&*payload)).into())
            }
        },
        Err(error) => Err(error),
    };
    if outcome.is_ok()
        && (stream.progress.completed_outer_steps != TOTAL_STEPS
            || stream.progress.frames_written != TOTAL_STEPS)
    {
        outcome = Err(format!(
            "incomplete collection: {} completed steps and {} frames, expected {TOTAL_STEPS}",
            stream.progress.completed_outer_steps, stream.progress.frames_written
        )
        .into());
    }
    let terminal = RunEnd {
        kind: "run_end",
        schema_version: SCHEMA_VERSION,
        status: if outcome.is_ok() {
            "collected"
        } else {
            "failed"
        },
        completed_outer_steps: stream.progress.completed_outer_steps,
        frames_written: stream.progress.frames_written,
        error: outcome.as_ref().err().map(ToString::to_string),
        physical_report: outcome.as_ref().ok(),
    };
    let terminal_result = stream.record(&terminal);
    let finalization_result = stream.output.finish().and_then(|mut output| output.flush());
    if terminal_result.is_err() || finalization_result.is_err() {
        let collection_error = outcome
            .as_ref()
            .err()
            .map(ToString::to_string)
            .unwrap_or_else(|| "none".into());
        let terminal_error = terminal_result
            .err()
            .map(|error| error.to_string())
            .unwrap_or_else(|| "none".into());
        let finalization_error = finalization_result
            .err()
            .map(|error| error.to_string())
            .unwrap_or_else(|| "none".into());
        return Err(format!("collection error: {collection_error}; terminal error: {terminal_error}; gzip finish/flush error: {finalization_error}").into());
    }
    outcome
}

fn panic_message(payload: &(dyn Any + Send)) -> &str {
    if let Some(message) = payload.downcast_ref::<String>() {
        message
    } else if let Some(message) = payload.downcast_ref::<&str>() {
        message
    } else {
        "non-string panic payload"
    }
}

/// Strict CLI for one case; all four flags are mandatory and occur once.
///
/// `--mass-kg 5|20|50 --observer on|off --label LABEL --output FILE.jsonl.gz`
/// uses exclusive creation and never overwrites a retained artifact. Success
/// means collection/finalization completed, not numerical or physical acceptance.
pub fn run_cli(args: impl IntoIterator<Item = String>) -> Result<u8> {
    let Some(options) = Options::parse(args)? else {
        println!("usage: 142_contact_motor_stages --mass-kg 5|20|50 --observer on|off --label LABEL --output FILE.jsonl.gz");
        return Ok(0);
    };
    let output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&options.output)?;
    let report = run_mass_held(
        options.mass_kg,
        options.observer_enabled,
        &options.label,
        BufWriter::new(output),
    )?;
    println!("{}", report.physical_stdout_row);
    println!("contact motor stage collection: ok");
    Ok(0)
}

fn collect<W: Write>(
    mass_kg: f64,
    observer_enabled: bool,
    stream: &mut JsonLines<W>,
) -> Result<PhysicalReport> {
    let squeeze_n = 1.25 * holding_squeeze_n(mass_kg);
    let mut backend = RapierBackend::new();
    let mut world = World::new();
    world.spawn((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        collider(Vec3::new(5.0, 0.5, 5.0)),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
    ));
    let cube = world
        .spawn((
            RigidBody {
                mass_kg,
                ..RigidBody::default()
            },
            collider(Vec3::splat(CUBE_HALF_M)),
            Transform3::from_translation_rotation(Vec3::new(0.0, CUBE_HALF_M, 0.0), Quat::IDENTITY),
        ))
        .id();
    let anchor_y_m = 1.0;
    let anchor = world
        .spawn((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            MultibodyLink,
            Transform3::from_translation_rotation(Vec3::new(0.0, anchor_y_m, 0.0), Quat::IDENTITY),
        ))
        .id();
    // Joint coordinates are zero in the spawn pose: each joint's parent
    // anchor sits where its child starts.
    let palm = world
        .spawn((
            RigidBody {
                mass_kg: 1.0,
                ..RigidBody::default()
            },
            collider(Vec3::new(0.12, 0.02, 0.05)),
            MultibodyLink,
            Transform3::from_translation_rotation(Vec3::new(0.0, PALM_Y_M, 0.0), Quat::IDENTITY),
            PrismaticJointDesc {
                parent: anchor,
                axis: Vec3::Y,
                anchor_parent_m: Vec3::new(0.0, PALM_Y_M - anchor_y_m, 0.0),
                anchor_child_m: Vec3::ZERO,
                relative_rotation: Quat::IDENTITY,
                lower_m: None,
                upper_m: None,
            },
            lift(0.0),
            JointMotorGainModel::ForceBased,
        ))
        .id();
    for side in [-1.0, 1.0] {
        world.spawn((
            RigidBody {
                mass_kg: 0.2,
                ..RigidBody::default()
            },
            collider(Vec3::new(0.01, 0.06, 0.04)),
            MultibodyLink,
            Transform3::from_translation_rotation(
                Vec3::new(side * FINGER_X_M, PALM_Y_M - FINGER_DROP_M, 0.0),
                Quat::IDENTITY,
            ),
            PrismaticJointDesc {
                parent: palm,
                axis: Vec3::X,
                anchor_parent_m: Vec3::new(side * FINGER_X_M, -FINGER_DROP_M, 0.0),
                anchor_child_m: Vec3::ZERO,
                relative_rotation: Quat::IDENTITY,
                lower_m: Some(-0.05),
                upper_m: Some(0.05),
            },
            JointActuation::PrismaticEffort {
                force_n: -side * squeeze_n,
                max_force_n: squeeze_n,
            },
        ));
    }

    let id = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    let dt = SimDuration::from_hertz(Hertz::new(STEP_HZ));
    let step = |backend: &mut RapierBackend, world: &mut World| {
        backend.sync_from_ecs(world, id).expect("sync from ECS");
        backend.step(id, dt).expect("step");
        backend.sync_to_ecs(world, id).expect("sync to ECS");
    };
    let height = |world: &World, entity: Entity| {
        world.get::<Transform3>(entity).expect("pose").translation.y
    };
    backend.configure_multibody_motor_stages(id, observer_enabled)?;
    for _ in 0..SETTLE_STEPS {
        step(&mut backend, &mut world);
        stream.progress.completed_outer_steps += 1;
        record_frame(&backend, &world, id, observer_enabled, stream)?;
    }
    let grip = height(&world, cube) - height(&world, palm);
    let palm_start = height(&world, palm);
    for index in 1..=LIFT_STEPS + HOLD_STEPS {
        let s = (index as f64 / LIFT_STEPS as f64).min(1.0);
        world.entity_mut(palm).insert(lift(LIFT_M * s));
        step(&mut backend, &mut world);
        stream.progress.completed_outer_steps += 1;
        record_frame(&backend, &world, id, observer_enabled, stream)?;
    }
    let cube_pose = *world.get::<Transform3>(cube).expect("cube pose");
    let palm_end = height(&world, palm);
    let lifted_m = cube_pose.translation.y - CUBE_HALF_M;
    let palm_rise_m = palm_end - palm_start;
    let slip_m = grip - (cube_pose.translation.y - palm_end);
    let tilt_rad = (cube_pose.rotation * Vec3::Y).y.clamp(-1.0, 1.0).acos();
    Ok(PhysicalReport {
        cube_ecs_pose_ieee_words: transform_words(&cube_pose),
        physical_ieee_words: [
            lifted_m.to_bits(),
            palm_rise_m.to_bits(),
            slip_m.to_bits(),
            tilt_rad.to_bits(),
        ],
        physical_stdout_row: format!(
            "  rapier: lifted {:6.3} m (palm {:5.3} m), slip {:8.1e} m, tilt {:7.1e} rad",
            lifted_m, palm_rise_m, slip_m, tilt_rad
        ),
    })
}

fn record_frame<W: Write>(
    backend: &RapierBackend,
    world: &World,
    id: PhysicsWorldId,
    observer_enabled: bool,
    stream: &mut JsonLines<W>,
) -> Result<()> {
    let checkpoint = backend.motor_stage_native_checkpoint(id)?;
    let stage = backend.completed_multibody_motor_stages(id)?;
    if stage.is_some() != observer_enabled {
        return Err("actual completed stage ON/OFF presence differs".into());
    }
    let record = CompletedStep {
        kind: "completed_step",
        schema_version: SCHEMA_VERSION,
        completed_outer_step: stream.progress.completed_outer_steps,
        native_checkpoint: &checkpoint,
        native_checkpoint_postcard_bytes: postcard::to_allocvec(&checkpoint)?,
        ecs_physics_ieee_words: ecs_words(world),
        stage_telemetry: stage,
    };
    stream.frame(&record)
}

fn transform_words(transform: &Transform3) -> Vec<u64> {
    transform
        .translation
        .to_array()
        .into_iter()
        .chain(transform.rotation.to_array())
        .chain(transform.scale.to_array())
        .map(f64::to_bits)
        .collect()
}

fn ecs_words(world: &World) -> Vec<u64> {
    let mut entities: Vec<Entity> = world.iter_entities().map(|entity| entity.id()).collect();
    entities.sort_unstable_by_key(|entity| entity.to_bits());
    let mut words = vec![1, entities.len() as u64];
    for entity in entities {
        words.push(entity.to_bits());
        match world.get::<Transform3>(entity) {
            Some(transform) => {
                words.push(1);
                words.extend(transform_words(transform));
            }
            None => words.push(0),
        }
        match world.get::<RigidBody>(entity) {
            Some(body) => {
                words.push(1);
                words.push(match body.body_type {
                    RigidBodyType::Dynamic => 0,
                    RigidBodyType::Fixed => 1,
                    RigidBodyType::Kinematic => 2,
                });
                words.push(body.mass_kg.to_bits());
                words.extend(
                    body.linear_velocity_m_s
                        .to_array()
                        .into_iter()
                        .chain(body.angular_velocity_rad_s.to_array())
                        .map(f64::to_bits),
                );
            }
            None => words.push(0),
        }
        match world.get::<JointState>(entity).copied() {
            None => words.push(0),
            Some(JointState::Fixed) => words.push(1),
            Some(JointState::Prismatic {
                position_m,
                velocity_m_s,
            }) => {
                words.extend([2, position_m.to_bits(), velocity_m_s.to_bits()]);
            }
            Some(JointState::Revolute {
                position_rad,
                velocity_rad_s,
            }) => {
                words.extend([3, position_rad.to_bits(), velocity_rad_s.to_bits()]);
            }
        }
    }
    words
}

#[cfg(test)]
mod tests;
