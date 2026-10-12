//! Prospective one-interval isolated motor-construction collector.
//!
//! This package is preparation only. Its collector must not be invoked with a
//! valid output until source/compiler/binary/schema/reader/workload registration
//! and independent review have been frozen. Its tests never construct a native
//! fixture. Collection completeness is distinct from any numerical criterion.

use flate2::{write::GzEncoder, Compression, GzBuilder};
use rapier3d::math::{Rotation, SpacialVector, Vector};
use rapier3d::pipeline::{
    MultibodyMotorConstructionTelemetry, MultibodyMotorImpulseRecord,
    MultibodyMotorImpulseTelemetry, MultibodyMotorImpulseValue, MultibodyMotorStageTelemetry,
};
use rapier3d::prelude::*;
use serde::{de::DeserializeOwned, Deserialize, Serialize};
use std::collections::BTreeMap;
use std::error::Error;
use std::fs::{File, OpenOptions};
use std::io::Write;
use std::num::NonZeroUsize;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::path::{Path, PathBuf};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const DT_S: f32 = 0.001;
const STIFFNESS_NM_PER_RAD: f32 = 300.0;
const DAMPING_NM_S_PER_RAD: f32 = 12.0;
const CEILING_NM: f32 = 2.0;
const STATE_SCOPE: &str = "sorted individual native rigid bodies, collider, unique multibody, exact integration parameters and public exposed IEEE words; excludes full-set metadata, broad/narrow phase, private pipeline/solver/contact caches and observer buffers";

/// Parse the fixed CLI and retain one exclusive, source-bound collection.
///
/// `--help` and malformed arguments construct no world. The valid CLI requires
/// target `negative` or `positive`, observer `on` or `off`, a label and an
/// absolute new gzip output file. Exclusive file creation and a durable initial
/// record precede Fixture construction. Exit zero establishes collection and
/// finalization only, never calibration accuracy or observer neutrality.
pub fn run_cli(args: impl IntoIterator<Item = String>) -> Result<u8> {
    let args: Vec<String> = args.into_iter().collect();
    if args == ["--help"] {
        println!("--target negative|positive --observer on|off --label LABEL --output ABSOLUTE_NEW_FILE.jsonl.gz");
        return Ok(0);
    }
    let options = Options::parse(args)?;
    run_claimed(&options, |writer, progress| {
        collect_native(&options, writer, progress)
    })?;
    println!("isolated motor construction collection: complete (1 outer interval)");
    Ok(0)
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum Target {
    Negative,
    Positive,
}

impl Target {
    fn position_rad(self) -> f32 {
        match self {
            Self::Negative => -0.001_f32,
            Self::Positive => 0.001_f32,
        }
    }
}

#[derive(Debug)]
struct Options {
    target: Target,
    observer_enabled: bool,
    label: String,
    output: PathBuf,
}

impl Options {
    fn parse(args: impl IntoIterator<Item = String>) -> Result<Self> {
        let (mut target, mut observer, mut label, mut output) = (None, None, None, None);
        let mut args = args.into_iter();
        while let Some(arg) = args.next() {
            let value = args
                .next()
                .ok_or_else(|| format!("missing value for {arg}"))?;
            match arg.as_str() {
                "--target" if target.is_none() => {
                    target = Some(match value.as_str() {
                        "negative" => Target::Negative,
                        "positive" => Target::Positive,
                        _ => return Err("target must be negative or positive".into()),
                    });
                }
                "--observer" if observer.is_none() => {
                    observer = Some(match value.as_str() {
                        "on" => true,
                        "off" => false,
                        _ => return Err("observer must be on or off".into()),
                    });
                }
                "--label" if label.is_none() => {
                    ensure(
                        !value.is_empty()
                            && value.len() <= 64
                            && value.bytes().all(|byte| {
                                byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-')
                            }),
                        "label must contain 1-64 ASCII letters, digits, underscores or hyphens",
                    )?;
                    label = Some(value);
                }
                "--output" if output.is_none() => {
                    let path = PathBuf::from(value);
                    ensure(
                        path.is_absolute() && path.file_name().is_some(),
                        "output must be an absolute file path",
                    )?;
                    output = Some(path);
                }
                _ => return Err(format!("unknown or repeated argument: {arg}").into()),
            }
        }
        Ok(Self {
            target: target.ok_or("--target is required")?,
            observer_enabled: observer.ok_or("--observer is required")?,
            label: label.ok_or("--label is required")?,
            output: output.ok_or("--output is required")?,
        })
    }
}

#[derive(Default, Debug, Serialize)]
struct Progress {
    fixture_construction_started: bool,
    fixture_constructed: bool,
    pipeline_step_started: bool,
    pipeline_step_returned: bool,
    completed_outer_intervals: usize,
    initial_records_written: usize,
    completed_records_written: usize,
    partial_records_written: usize,
}

#[derive(Serialize)]
struct RunStart<'a> {
    kind: &'static str,
    schema_version: u32,
    run_label: &'a str,
    target: Target,
    target_position_rad_word: u32,
    observer_enabled: bool,
    expected_outer_intervals: usize,
    native_scalar_bits: u8,
    state_codec: &'static str,
    state_scope: &'static str,
    declared_conditions: Conditions,
}

#[derive(Debug, Serialize)]
struct Conditions {
    sphere_radius_m_word: u32,
    sphere_mass_kg_word: u32,
    outer_dt_s_word: u32,
    stiffness_nm_per_rad_word: u32,
    damping_nm_s_per_rad_word: u32,
    ceiling_nm_word: u32,
    target_velocity_rad_s_word: u32,
    gravity_words: [u32; 3],
    initial_generalized_velocity_word: u32,
    passive_damping_word: u32,
    armature_word: u32,
    small_steps: usize,
    pgs_passes: usize,
    stabilization_passes: usize,
    contacts_enabled: bool,
    sleep_enabled: bool,
    ccd_enabled: bool,
    identity_coincident_x_frames: bool,
}

impl Conditions {
    fn fixed() -> Self {
        Self {
            sphere_radius_m_word: 0.1_f32.to_bits(),
            sphere_mass_kg_word: 1.0_f32.to_bits(),
            outer_dt_s_word: DT_S.to_bits(),
            stiffness_nm_per_rad_word: STIFFNESS_NM_PER_RAD.to_bits(),
            damping_nm_s_per_rad_word: DAMPING_NM_S_PER_RAD.to_bits(),
            ceiling_nm_word: CEILING_NM.to_bits(),
            target_velocity_rad_s_word: 0,
            gravity_words: [0; 3],
            initial_generalized_velocity_word: 0,
            passive_damping_word: 0,
            armature_word: 0,
            small_steps: 32,
            pgs_passes: 1,
            stabilization_passes: 2,
            contacts_enabled: false,
            sleep_enabled: false,
            ccd_enabled: false,
            identity_coincident_x_frames: true,
        }
    }
}

struct Recorder {
    encoder: GzEncoder<File>,
    records_written: usize,
}

impl Recorder {
    fn claim(options: &Options) -> Result<Self> {
        let file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&options.output)?;
        file.sync_all()?;
        sync_parent(&options.output)?;
        let mut writer = Self {
            encoder: GzBuilder::new()
                .mtime(0)
                .operating_system(255)
                .write(file, Compression::default()),
            records_written: 0,
        };
        writer.record(&RunStart {
            kind: "run_start",
            schema_version: 1,
            run_label: &options.label,
            target: options.target,
            target_position_rad_word: options.target.position_rad().to_bits(),
            observer_enabled: options.observer_enabled,
            expected_outer_intervals: 1,
            native_scalar_bits: 32,
            state_codec: "postcard-1.1.3-raw-exact-consumption",
            state_scope: STATE_SCOPE,
            declared_conditions: Conditions::fixed(),
        })?;
        Ok(writer)
    }

    fn record(&mut self, value: &impl Serialize) -> Result<()> {
        let bytes = serde_json::to_vec(value)?;
        self.encoder.write_all(&bytes)?;
        self.encoder.write_all(b"\n")?;
        self.encoder.flush()?;
        self.encoder.get_mut().sync_all()?;
        self.records_written += 1;
        Ok(())
    }

    fn finish(self, path: &Path) -> Result<()> {
        let mut file = self.encoder.finish()?;
        file.flush()?;
        file.sync_all()?;
        sync_parent(path)?;
        Ok(())
    }
}

fn sync_parent(path: &Path) -> Result<()> {
    File::open(path.parent().ok_or("output has no parent")?)?.sync_all()?;
    Ok(())
}

fn panic_text(payload: Box<dyn std::any::Any + Send>) -> String {
    if let Some(value) = payload.downcast_ref::<String>() {
        value.clone()
    } else if let Some(value) = payload.downcast_ref::<&str>() {
        (*value).to_owned()
    } else {
        "non-string panic payload".to_owned()
    }
}

fn run_claimed(
    options: &Options,
    collector: impl FnOnce(&mut Recorder, &mut Progress) -> Result<()>,
) -> Result<()> {
    let mut writer = Recorder::claim(options)?;
    let mut progress = Progress::default();
    let outcome = catch_unwind(AssertUnwindSafe(|| collector(&mut writer, &mut progress)));
    let mut error = match outcome {
        Ok(Ok(())) => None,
        Ok(Err(error)) => Some(error.to_string()),
        Err(panic) => Some(format!("panic: {}", panic_text(panic))),
    };
    if error.is_none()
        && !(progress.pipeline_step_returned
            && progress.completed_outer_intervals == 1
            && progress.initial_records_written == 1
            && progress.completed_records_written == 1)
    {
        error =
            Some("successful callback did not complete the declared native collection".to_owned());
    }
    #[derive(Serialize)]
    struct Terminal<'a> {
        kind: &'static str,
        schema_version: u32,
        status: &'static str,
        progress: &'a Progress,
        records_written_before_terminal: usize,
        error: Option<&'a str>,
    }
    let terminal_result = writer.record(&Terminal {
        kind: "run_end",
        schema_version: 1,
        status: if error.is_none() {
            "collected"
        } else {
            "failed"
        },
        progress: &progress,
        records_written_before_terminal: writer.records_written,
        error: error.as_deref(),
    });
    let finish_result = writer.finish(&options.output);
    if error.is_some() || terminal_result.is_err() || finish_result.is_err() {
        return Err(format!(
            "collector={}; terminal={}; finalization={}",
            error.as_deref().unwrap_or("completed"),
            terminal_result
                .as_ref()
                .err()
                .map(ToString::to_string)
                .as_deref()
                .unwrap_or("written"),
            finish_result
                .as_ref()
                .err()
                .map(ToString::to_string)
                .as_deref()
                .unwrap_or("completed")
        )
        .into());
    }
    Ok(())
}

struct Fixture {
    pipeline: PhysicsPipeline,
    parameters: IntegrationParameters,
    islands: IslandManager,
    broad: BroadPhaseMultiSap,
    narrow: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd: CCDSolver,
    parent: RigidBodyHandle,
    child: RigidBodyHandle,
    joint: MultibodyJointHandle,
}

impl Fixture {
    fn new(target: Target, observer_enabled: bool) -> Result<Self> {
        let mut bodies = RigidBodySet::new();
        let parent = bodies.insert(RigidBodyBuilder::fixed());
        let child = bodies.insert(
            RigidBodyBuilder::dynamic()
                .can_sleep(false)
                .ccd_enabled(false)
                .linear_damping(0.0)
                .angular_damping(0.0)
                .additional_solver_iterations(0),
        );
        let mut colliders = ColliderSet::new();
        colliders.insert_with_parent(ColliderBuilder::ball(0.1).mass(1.0), child, &mut bodies);
        bodies[child].recompute_mass_properties_from_colliders(&colliders);
        let joint = fixture_joint(target);
        let mut multibody_joints = MultibodyJointSet::new();
        let joint = multibody_joints
            .insert(parent, child, joint, true)
            .ok_or("joint insertion failed")?;
        let (multibody, _) = multibody_joints
            .get_mut(joint)
            .ok_or("missing initial multibody")?;
        multibody.forward_kinematics(&bodies, true);
        ensure(
            multibody.ndofs() == 1,
            "fixed-parent fixture is not one DOF",
        )?;
        multibody.damping_mut().fill(0.0);
        ensure(multibody.set_armature(0, 0.0), "zero armature rejected")?;
        multibody.generalized_velocity_mut().fill(0.0);
        let mut pipeline = PhysicsPipeline::new();
        pipeline.set_multibody_motor_impulse_telemetry_enabled(observer_enabled);
        pipeline.set_multibody_motor_stage_telemetry_enabled(observer_enabled);
        pipeline.set_multibody_motor_construction_telemetry_enabled(observer_enabled);
        Ok(Self {
            pipeline,
            parameters: IntegrationParameters {
                dt: DT_S,
                num_solver_iterations: NonZeroUsize::new(32).ok_or("zero small-step count")?,
                num_internal_pgs_iterations: 1,
                num_internal_stabilization_iterations: 2,
                max_ccd_substeps: 1,
                ..IntegrationParameters::default()
            },
            islands: IslandManager::new(),
            broad: BroadPhaseMultiSap::new(),
            narrow: NarrowPhase::new(),
            bodies,
            colliders,
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints,
            ccd: CCDSolver::new(),
            parent,
            child,
            joint,
        })
    }

    fn step(&mut self) {
        self.pipeline.step(
            &Vector::zeros(),
            &self.parameters,
            &mut self.islands,
            &mut self.broad,
            &mut self.narrow,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd,
            None,
            &(),
            &(),
        );
    }
}

fn fixture_joint(target: Target) -> RevoluteJoint {
    let mut joint = RevoluteJointBuilder::new(Vector::x_axis())
        .motor_model(MotorModel::ForceBased)
        .motor_position(
            target.position_rad(),
            STIFFNESS_NM_PER_RAD,
            DAMPING_NM_S_PER_RAD,
        )
        .motor_max_force(CEILING_NM)
        .contacts_enabled(false)
        .build();
    joint.data.local_frame1 = Isometry::identity();
    joint.data.local_frame2 = Isometry::identity();
    joint
}

#[derive(Debug, Serialize, Deserialize)]
struct SerializedJoint {
    data: GenericJoint,
    kinematic: bool,
    coords: SpacialVector<f32>,
    joint_rot: Rotation<f32>,
}

fn exact_native_bytes<T: Serialize + DeserializeOwned>(value: &T) -> Result<Vec<u8>> {
    let bytes = postcard::to_allocvec(value)?;
    let (decoded, remainder): (T, _) = postcard::take_from_bytes(&bytes)?;
    ensure(
        remainder.is_empty() && postcard::to_allocvec(&decoded)? == bytes,
        "native postcard exact-consumption/reencode failed",
    )?;
    Ok(bytes)
}

#[derive(Debug, Serialize)]
enum ObjectKind {
    Body,
    Collider,
    Multibody,
}
#[derive(Debug, Serialize)]
struct ObjectBytes {
    kind: ObjectKind,
    handle_words: [u32; 2],
    postcard_bytes: Vec<u8>,
}
#[derive(Debug, Serialize)]
struct BodyWords {
    handle_words: [u32; 2],
    position_words: Vec<u32>,
    linear_velocity_words: [u32; 3],
    angular_velocity_words: [u32; 3],
    user_force_words: [u32; 3],
    user_torque_words: [u32; 3],
    mass_query_kg_word: u32,
    effective_angular_inertia_query_kg_m2_words: Vec<u32>,
}
#[derive(Debug, Serialize)]
struct JointWords {
    link_index: usize,
    body_handle_words: [u32; 2],
    coordinates_words: Vec<u32>,
    joint_rotation_words: Vec<u32>,
    descriptor_motor_impulse_words: Vec<u32>,
}
#[derive(Debug, Serialize)]
struct MultibodyWords {
    handle_words: [u32; 2],
    ndofs: usize,
    links: usize,
    generalized_velocity_words: Vec<u32>,
    damping_words: Vec<u32>,
    joints: Vec<JointWords>,
}
#[derive(Debug, Serialize)]
struct ParametersWords {
    native_f32_words: [u32; 11],
    native_counts: [usize; 6],
    postcard_bytes: Vec<u8>,
}
#[derive(Debug, Serialize)]
struct Checkpoint {
    schema_version: u32,
    native_scalar_bits: u8,
    state_scope: &'static str,
    gravity_words: [u32; 3],
    integration_parameters: ParametersWords,
    records: Vec<ObjectBytes>,
    public_body_words: Vec<BodyWords>,
    public_multibody_words: Vec<MultibodyWords>,
    parent_handle_words: [u32; 2],
    child_handle_words: [u32; 2],
    joint_handle_words: [u32; 2],
    child_link_index: usize,
}

fn pair(parts: (u32, u32)) -> [u32; 2] {
    [parts.0, parts.1]
}
fn vector_words(value: &Vector<f32>) -> [u32; 3] {
    value.map(f32::to_bits).into()
}

impl Fixture {
    fn checkpoint(&self) -> Result<Checkpoint> {
        let mut records = Vec::new();
        let mut public_body_words = Vec::new();
        let mut bodies: Vec<_> = self.bodies.iter().collect();
        bodies.sort_by_key(|(handle, _)| handle.into_raw_parts());
        for (handle, body) in bodies {
            records.push(ObjectBytes {
                kind: ObjectKind::Body,
                handle_words: pair(handle.into_raw_parts()),
                postcard_bytes: exact_native_bytes(body)?,
            });
            public_body_words.push(BodyWords {
                handle_words: pair(handle.into_raw_parts()),
                position_words: body
                    .position()
                    .translation
                    .vector
                    .iter()
                    .chain(body.position().rotation.coords.iter())
                    .map(|x| x.to_bits())
                    .collect(),
                linear_velocity_words: vector_words(body.linvel()),
                angular_velocity_words: vector_words(body.angvel()),
                user_force_words: vector_words(&body.user_force()),
                user_torque_words: vector_words(&body.user_torque()),
                mass_query_kg_word: body.mass().to_bits(),
                effective_angular_inertia_query_kg_m2_words: {
                    let inertia = body.mass_properties().effective_angular_inertia();
                    // Copy public symmetric entries in column-major order; no matrix arithmetic.
                    [
                        inertia.m11,
                        inertia.m12,
                        inertia.m13,
                        inertia.m12,
                        inertia.m22,
                        inertia.m23,
                        inertia.m13,
                        inertia.m23,
                        inertia.m33,
                    ]
                    .map(f32::to_bits)
                    .to_vec()
                },
            });
        }
        let mut colliders: Vec<_> = self.colliders.iter().collect();
        colliders.sort_by_key(|(handle, _)| handle.into_raw_parts());
        for (handle, collider) in colliders {
            records.push(ObjectBytes {
                kind: ObjectKind::Collider,
                handle_words: pair(handle.into_raw_parts()),
                postcard_bytes: exact_native_bytes(collider)?,
            });
        }
        let mut multibodies = BTreeMap::new();
        for (_, link, multibody, _) in self.multibody_joints.iter() {
            multibodies
                .entry(link.multibody.0.into_raw_parts())
                .or_insert(multibody);
        }
        let mut public_multibody_words = Vec::new();
        for (handle, multibody) in multibodies {
            records.push(ObjectBytes {
                kind: ObjectKind::Multibody,
                handle_words: pair(handle),
                postcard_bytes: exact_native_bytes(multibody)?,
            });
            let mut joints = Vec::new();
            for (link_index, link) in multibody.links().enumerate() {
                let bytes = exact_native_bytes(link.joint())?;
                let (joint, remainder): (SerializedJoint, _) = postcard::take_from_bytes(&bytes)?;
                ensure(
                    remainder.is_empty() && postcard::to_allocvec(&joint)? == bytes,
                    "source-bound joint projection layout differs",
                )?;
                joints.push(JointWords {
                    link_index,
                    body_handle_words: pair(link.rigid_body_handle().into_raw_parts()),
                    coordinates_words: joint.coords.iter().map(|x| x.to_bits()).collect(),
                    joint_rotation_words: joint
                        .joint_rot
                        .coords
                        .iter()
                        .map(|x| x.to_bits())
                        .collect(),
                    descriptor_motor_impulse_words: joint
                        .data
                        .motors
                        .iter()
                        .map(|motor| motor.impulse.to_bits())
                        .collect(),
                });
            }
            public_multibody_words.push(MultibodyWords {
                handle_words: pair(handle),
                ndofs: multibody.ndofs(),
                links: multibody.num_links(),
                generalized_velocity_words: multibody
                    .generalized_velocity()
                    .iter()
                    .map(|x| x.to_bits())
                    .collect(),
                damping_words: multibody.damping().iter().map(|x| x.to_bits()).collect(),
                joints,
            });
        }
        let p = &self.parameters;
        let parameters = ParametersWords {
            native_f32_words: [
                p.dt.to_bits(),
                p.min_ccd_dt.to_bits(),
                p.contact_damping_ratio.to_bits(),
                p.contact_natural_frequency.to_bits(),
                p.joint_natural_frequency.to_bits(),
                p.joint_damping_ratio.to_bits(),
                p.warmstart_coefficient.to_bits(),
                p.length_unit.to_bits(),
                p.normalized_allowed_linear_error.to_bits(),
                p.normalized_max_corrective_velocity.to_bits(),
                p.normalized_prediction_distance.to_bits(),
            ],
            native_counts: [
                p.num_solver_iterations.get(),
                p.num_additional_friction_iterations,
                p.num_internal_pgs_iterations,
                p.num_internal_stabilization_iterations,
                p.min_island_size,
                p.max_ccd_substeps,
            ],
            postcard_bytes: exact_native_bytes(p)?,
        };
        let (_, child_link_index) = self
            .multibody_joints
            .get(self.joint)
            .ok_or("missing child multibody binding")?;
        Ok(Checkpoint {
            schema_version: 1,
            native_scalar_bits: 32,
            state_scope: STATE_SCOPE,
            gravity_words: [0; 3],
            integration_parameters: parameters,
            records,
            public_body_words,
            public_multibody_words,
            parent_handle_words: pair(self.parent.into_raw_parts()),
            child_handle_words: pair(self.child.into_raw_parts()),
            joint_handle_words: pair(self.joint.into_raw_parts()),
            child_link_index,
        })
    }
}

#[derive(Debug, Serialize)]
struct ImpulseValueWords {
    kind: &'static str,
    applied_impulse_word: u64,
    effort_ceiling_word: Option<u64>,
    rounded_bound_word: Option<u64>,
}
fn impulse_value(value: MultibodyMotorImpulseValue) -> ImpulseValueWords {
    match value {
        MultibodyMotorImpulseValue::Revolute {
            applied_impulse_nm_s,
            effective_effort_ceiling_nm,
            rounded_impulse_bound_nm_s,
        } => ImpulseValueWords {
            kind: "Revolute",
            applied_impulse_word: applied_impulse_nm_s.to_bits(),
            effort_ceiling_word: effective_effort_ceiling_nm.map(f64::to_bits),
            rounded_bound_word: rounded_impulse_bound_nm_s.map(f64::to_bits),
        },
        MultibodyMotorImpulseValue::Prismatic {
            applied_impulse_n_s,
            effective_force_ceiling_n,
            rounded_impulse_bound_n_s,
        } => ImpulseValueWords {
            kind: "Prismatic",
            applied_impulse_word: applied_impulse_n_s.to_bits(),
            effort_ceiling_word: effective_force_ceiling_n.map(f64::to_bits),
            rounded_bound_word: rounded_impulse_bound_n_s.map(f64::to_bits),
        },
    }
}
#[derive(Debug, Serialize)]
struct ImpulseSliceWords {
    ccd_partition_index: usize,
    internal_slice_index: usize,
    expected_internal_slice_count: usize,
    duration_s_word: u64,
    impulse: ImpulseValueWords,
    finite_positive_bound_eligible: bool,
    finite_positive_bound_active: bool,
}
#[derive(Debug, Serialize)]
struct ImpulseRecordWords {
    joint_handle_words: [u32; 2],
    link_id: usize,
    joint_dof_id: usize,
    axis: JointAxis,
    impulse: ImpulseValueWords,
    observed_solver_interval_s_word: u64,
    observed_solver_slice_count: usize,
    full_step_coverage: bool,
    observed_partition_count: usize,
    finite_positive_bound_eligible_slice_count: usize,
    finite_positive_bound_eligible_duration_s_word: u64,
    finite_positive_bound_active_slice_count: usize,
    finite_positive_bound_active_duration_s_word: u64,
    slices: Vec<ImpulseSliceWords>,
}
impl From<&MultibodyMotorImpulseRecord> for ImpulseRecordWords {
    fn from(record: &MultibodyMotorImpulseRecord) -> Self {
        Self {
            joint_handle_words: pair(record.joint_handle.into_raw_parts()),
            link_id: record.link_id,
            joint_dof_id: record.joint_dof_id,
            axis: record.axis,
            impulse: impulse_value(record.impulse),
            observed_solver_interval_s_word: record.observed_solver_interval_s.to_bits(),
            observed_solver_slice_count: record.observed_solver_slice_count,
            full_step_coverage: record.full_step_coverage,
            observed_partition_count: record.observed_partition_count,
            finite_positive_bound_eligible_slice_count: record
                .finite_positive_bound_eligible_slice_count,
            finite_positive_bound_eligible_duration_s_word: record
                .finite_positive_bound_eligible_duration_s
                .to_bits(),
            finite_positive_bound_active_slice_count: record
                .finite_positive_bound_active_slice_count,
            finite_positive_bound_active_duration_s_word: record
                .finite_positive_bound_active_duration_s
                .to_bits(),
            slices: record
                .slices
                .iter()
                .map(|slice| ImpulseSliceWords {
                    ccd_partition_index: slice.ccd_partition_index,
                    internal_slice_index: slice.internal_slice_index,
                    expected_internal_slice_count: slice.expected_internal_slice_count,
                    duration_s_word: slice.duration_s.to_bits(),
                    impulse: impulse_value(slice.impulse),
                    finite_positive_bound_eligible: slice.finite_positive_bound_eligible,
                    finite_positive_bound_active: slice.finite_positive_bound_active,
                })
                .collect(),
        }
    }
}
#[derive(Debug, Serialize)]
struct ImpulseTelemetryWords {
    scalar_bits: u8,
    numerical_outer_interval_s_word: u64,
    ccd_partition_duration_s_words: Vec<u64>,
    motors: Vec<ImpulseRecordWords>,
}
impl From<&MultibodyMotorImpulseTelemetry> for ImpulseTelemetryWords {
    fn from(value: &MultibodyMotorImpulseTelemetry) -> Self {
        Self {
            scalar_bits: 64,
            numerical_outer_interval_s_word: value.numerical_outer_interval_s.to_bits(),
            ccd_partition_duration_s_words: value
                .ccd_partition_durations_s
                .iter()
                .map(|x| x.to_bits())
                .collect(),
            motors: value.motors.iter().map(ImpulseRecordWords::from).collect(),
        }
    }
}

#[derive(Serialize)]
struct Initial<'a> {
    kind: &'static str,
    schema_version: u32,
    checkpoint: &'a Checkpoint,
}
#[derive(Serialize)]
struct Completed<'a> {
    kind: &'static str,
    schema_version: u32,
    completed_outer_interval: usize,
    checkpoint: &'a Checkpoint,
    construction_companion: Option<&'a MultibodyMotorConstructionTelemetry>,
    stage_v1: Option<&'a MultibodyMotorStageTelemetry>,
    impulse_v1_ieee_words: Option<ImpulseTelemetryWords>,
}
#[derive(Serialize)]
struct Partial<'a> {
    kind: &'static str,
    schema_version: u32,
    checkpoint: Option<Checkpoint>,
    checkpoint_error: Option<String>,
    completed_observers_available: [bool; 3],
    construction_companion: Option<&'a MultibodyMotorConstructionTelemetry>,
    stage_v1: Option<&'a MultibodyMotorStageTelemetry>,
    impulse_v1_ieee_words: Option<ImpulseTelemetryWords>,
}

fn collect_native(options: &Options, writer: &mut Recorder, progress: &mut Progress) -> Result<()> {
    progress.fixture_construction_started = true;
    let mut fixture = Fixture::new(options.target, options.observer_enabled)?;
    progress.fixture_constructed = true;
    let initial = fixture.checkpoint()?;
    writer.record(&Initial {
        kind: "initial_state",
        schema_version: 1,
        checkpoint: &initial,
    })?;
    progress.initial_records_written += 1;
    progress.pipeline_step_started = true;
    if let Err(panic) = catch_unwind(AssertUnwindSafe(|| fixture.step())) {
        let observation = catch_unwind(AssertUnwindSafe(|| fixture.checkpoint()));
        let (checkpoint, checkpoint_error) = match observation {
            Ok(Ok(state)) => (Some(state), None),
            Ok(Err(error)) => (None, Some(error.to_string())),
            Err(panic) => (None, Some(panic_text(panic))),
        };
        let availability = [
            fixture
                .pipeline
                .completed_multibody_motor_constructions()
                .is_some(),
            fixture
                .pipeline
                .completed_multibody_motor_stages()
                .is_some(),
            fixture
                .pipeline
                .completed_multibody_motor_impulses()
                .is_some(),
        ];
        let primary_error = format!("pipeline step panic: {}", panic_text(panic));
        let partial_result = writer.record(&Partial {
            kind: "partial_native_observation",
            schema_version: 1,
            checkpoint,
            checkpoint_error,
            completed_observers_available: availability,
            construction_companion: fixture.pipeline.completed_multibody_motor_constructions(),
            stage_v1: fixture.pipeline.completed_multibody_motor_stages(),
            impulse_v1_ieee_words: fixture
                .pipeline
                .completed_multibody_motor_impulses()
                .map(ImpulseTelemetryWords::from),
        });
        if partial_result.is_ok() {
            progress.partial_records_written += 1;
        }
        return Err(format!(
            "{primary_error}; partial record={}",
            partial_result
                .as_ref()
                .err()
                .map(ToString::to_string)
                .as_deref()
                .unwrap_or("written")
        )
        .into());
    }
    progress.pipeline_step_returned = true;
    progress.completed_outer_intervals = 1;
    let completed = fixture.checkpoint()?;
    let companion = fixture.pipeline.completed_multibody_motor_constructions();
    let stage = fixture.pipeline.completed_multibody_motor_stages();
    let impulse = fixture.pipeline.completed_multibody_motor_impulses();
    writer.record(&Completed {
        kind: "completed_interval",
        schema_version: 1,
        completed_outer_interval: 1,
        checkpoint: &completed,
        construction_companion: companion,
        stage_v1: stage,
        impulse_v1_ieee_words: impulse.map(ImpulseTelemetryWords::from),
    })?;
    progress.completed_records_written += 1;
    ensure(
        [companion.is_some(), stage.is_some(), impulse.is_some()] == [options.observer_enabled; 3],
        "completed ON/OFF observation availability differs",
    )?;
    Ok(())
}

fn ensure(condition: bool, message: &str) -> Result<()> {
    if condition {
        Ok(())
    } else {
        Err(message.into())
    }
}

#[cfg(test)]
mod tests;
