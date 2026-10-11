//! Isolated one-interval, small-error position-motor calibration.
//!
//! Collection and a successful numerical criterion are separate outcomes. This
//! fixture does not identify hardware actuators or calibrate a recovery policy.

use rapier3d::math::{Rotation, SpacialVector, Vector};
use rapier3d::na::DVector;
use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::error::Error;
use std::io::Write;
use std::num::NonZeroUsize;
use std::path::PathBuf;

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const OUTER_DT_S: f32 = 0.001;
const CAP_NM: f32 = 2.0;
const STABILIZATION_PASSES: usize = 2;
const INERTIA_ABS_TOLERANCE: f64 = 1.0e-8;
const INERTIA_REL_TOLERANCE: f64 = 2.0e-5;

/// Collect the fixed matrix or verify a retained report without advancing physics.
///
/// `--output FILE` writes all 24 cells and 72 fresh-world attempts, including
/// failed, partial and nonfinite measurements. `--require-calibration` returns
/// exit code 2 when the predeclared primary calibration criterion fails.
/// `--verify-report FILE` checks retained source/configuration, serialized state,
/// telemetry, replay and recomputed assessments; it creates no worlds.
/// Exit code 1 is returned for incomplete collection or failed replay qualification.
/// These checks establish internal report consistency, not external attestation.
pub fn run_cli(args: impl IntoIterator<Item = String>) -> Result<u8> {
    let options = Options::parse(args)?;
    let report = if let Some(path) = &options.verify {
        let report: Report = serde_json::from_slice(&std::fs::read(path)?)?;
        verify_report(&report)?;
        report
    } else {
        if let Some(parent) = options
            .output
            .parent()
            .filter(|p| !p.as_os_str().is_empty())
        {
            std::fs::create_dir_all(parent)?;
        }
        // Reject an existing attempt before constructing any physics world.
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&options.output)?;
        let report = collect_report();
        // Preserve the complete attempt even if a subsequent consistency check fails.
        file.write_all(&serde_json::to_vec_pretty(&report)?)?;
        file.write_all(b"\n")?;
        verify_report(&report)?;
        report
    };
    let summary = &report.summary;
    println!(
        "collection_complete={} completed_worlds={}/72 replay_qualification={} primary_calibration={} mode={}",
        summary.collection_complete, summary.completed_worlds,
        summary.replay_qualification_pass,
        if summary.primary_calibration_pass { "PASS" } else { "FAIL" },
        if options.verify.is_some() { "verify-report-no-worlds" } else { "collect" },
    );
    Ok(
        if !summary.collection_complete || !summary.replay_qualification_pass {
            1
        } else if options.require && !summary.primary_calibration_pass {
            2
        } else {
            0
        },
    )
}

#[derive(Debug)]
struct Options {
    output: PathBuf,
    verify: Option<PathBuf>,
    require: bool,
}

impl Options {
    fn parse(args: impl IntoIterator<Item = String>) -> Result<Self> {
        let mut options = Self {
            output: "target/rne-position-motor-calibration/report.json".into(),
            verify: None,
            require: false,
        };
        let mut args = args.into_iter();
        let mut output_seen = false;
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--output" if !output_seen => {
                    options.output = args.next().ok_or("--output requires a file")?.into();
                    output_seen = true;
                }
                "--verify-report" if options.verify.is_none() => {
                    options.verify =
                        Some(args.next().ok_or("--verify-report requires a file")?.into());
                }
                "--require-calibration" if !options.require => options.require = true,
                _ => return Err(format!("unknown or repeated argument: {arg}").into()),
            }
        }
        ensure(
            !(output_seen && options.verify.is_some()),
            "--output and --verify-report are mutually exclusive",
        )?;
        Ok(options)
    }
}

// JSON never silently maps NaN/Infinity to a measured zero or an unlabelled null.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
struct Scalar {
    finite: Option<f64>,
    ieee754_f64_bits: u64,
    native_f32_bits: Option<u32>,
}

impl Scalar {
    fn f64(value: f64) -> Self {
        Self {
            finite: value.is_finite().then_some(value),
            ieee754_f64_bits: value.to_bits(),
            native_f32_bits: None,
        }
    }
    fn f32(value: f32) -> Self {
        Self {
            native_f32_bits: Some(value.to_bits()),
            ..Self::f64(f64::from(value))
        }
    }
    fn value(self) -> f64 {
        f64::from_bits(self.ieee754_f64_bits)
    }
    fn validate(self) -> Result<()> {
        let value = self.value();
        ensure(
            self.finite.map(f64::to_bits) == value.is_finite().then_some(value.to_bits()),
            "scalar finite value/bits mismatch",
        )?;
        if let Some(bits) = self.native_f32_bits {
            ensure(
                f64::from(f32::from_bits(bits)).to_bits() == value.to_bits(),
                "scalar native f32/f64 bits mismatch",
            )?;
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
enum Gain {
    G1Posture,
    G1Ankle,
}

impl Gain {
    fn coefficients(self) -> (f32, f32) {
        match self {
            Self::G1Posture => (300.0, 12.0),
            Self::G1Ankle => (250.0, 8.0),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
struct Cell {
    gain: Gain,
    target_position_rad: Scalar,
    solver_small_steps: usize,
    internal_pgs_passes: usize,
}

impl Cell {
    fn primary(&self) -> bool {
        self.solver_small_steps == 32
            && self.internal_pgs_passes == 1
            && self.target_position_rad.value() != 0.0
    }
}

fn fixed_cells() -> Vec<Cell> {
    let mut cells = Vec::new();
    for gain in [Gain::G1Posture, Gain::G1Ankle] {
        for target in [-0.001_f32, 0.0, 0.001] {
            for solver_small_steps in [1, 32] {
                for internal_pgs_passes in [1, 2] {
                    cells.push(Cell {
                        gain,
                        target_position_rad: Scalar::f32(target),
                        solver_small_steps,
                        internal_pgs_passes,
                    });
                }
            }
        }
    }
    cells
}

fn protocol() -> Value {
    json!({
        "schema_version": 1, "fixture": "fixed parent plus coincident 1 kg, radius 0.1 m isotropic sphere; revolute X",
        "outer_dt_s": Scalar::f32(OUTER_DT_S), "outer_steps": 1, "effort_cap_nm": Scalar::f32(CAP_NM),
        "initial_q_rad": 0, "initial_v_rad_s": 0, "target_v_rad_s": 0,
        "gravity": [0,0,0], "passive_joint_damping": 0, "body_damping": 0, "armature": 0,
        "contacts": false, "limits": false, "ccd": false, "additional_solver_iterations": 0,
        "stabilization_passes": STABILIZATION_PASSES, "model": "ForceBased",
        "acquisition": "public body mass recompute and multibody forward_kinematics before first step; no time advances",
        "first_row_inverse_inertia_scope": "independently derived from pre-step native body Ixx under isolated isotropic/X/identity/COM-zero/zero-damping-armature assumptions; not captured native first-row LU",
        "post_step_lu_scope": "latest public augmented-mass LU.solve([1]); separately compared with derived pre-step inverse inertia, not relabelled first-row measurement",
        "inertia_comparison_tolerance": {"absolute": INERTIA_ABS_TOLERANCE, "relative": INERTIA_REL_TOLERANCE},
        "primary": "N32/PGS1 nonzero signs for both gains: final first-slice impulse after PGS/stabilization2 against frozen-row fixed point; all eight zero targets stay motionless",
        "impulse_tolerance": {"absolute_nm_s": 1.0e-8, "relative": 0.01},
        "fixed_point": "clip(-(independently f32-rounded rhs + pre native v)/(independently f32-rounded inverse I + independently f32-rounded g), native rounded +/-bound); native a/g bits compared separately; denominator evaluated after promotion to f64",
        "source_scope": "selected local source/configuration inputs, not an exhaustive compiler/dependency closure; lock/compiler/protocol are also bound externally before execution",
        "state_scope": "known rigid bodies sorted by handle and single native Multibody serialized individually with bincode1; private coords retained through source-bound serde layout; not all private pipeline state",
        "truth_boundary": "motor-only numerical calibration; no G1 contact, controller, kick, hardware or reliability inference"
    })
}

fn selected_sources() -> BTreeMap<String, String> {
    let mut sources = BTreeMap::new();
    macro_rules! source {
        ($name:literal, $path:literal) => {
            sources.insert($name.to_owned(), hash(include_bytes!($path)));
        };
    }
    source!("examples/141_position_motor_calibration/lib.rs", "lib.rs");
    source!("examples/141_position_motor_calibration/main.rs", "main.rs");
    source!(
        "examples/141_position_motor_calibration/Cargo.toml",
        "Cargo.toml"
    );
    source!("Cargo.toml", "../../Cargo.toml");
    source!("Cargo.lock", "../../Cargo.lock");
    source!(
        "third_party/rapier3d/Cargo.toml",
        "../../third_party/rapier3d/Cargo.toml"
    );
    source!(
        "third_party/rapier3d/src/dynamics/joint/motor_model.rs",
        "../../third_party/rapier3d/src/dynamics/joint/motor_model.rs"
    );
    source!(
        "third_party/rapier3d/src/dynamics/joint/multibody_joint/multibody.rs",
        "../../third_party/rapier3d/src/dynamics/joint/multibody_joint/multibody.rs"
    );
    source!(
        "third_party/rapier3d/src/dynamics/joint/multibody_joint/multibody_joint.rs",
        "../../third_party/rapier3d/src/dynamics/joint/multibody_joint/multibody_joint.rs"
    );
    source!(
        "third_party/rapier3d/src/dynamics/joint/multibody_joint/unit_multibody_joint.rs",
        "../../third_party/rapier3d/src/dynamics/joint/multibody_joint/unit_multibody_joint.rs"
    );
    source!("third_party/rapier3d/src/dynamics/solver/joint_constraint/joint_generic_constraint.rs", "../../third_party/rapier3d/src/dynamics/solver/joint_constraint/joint_generic_constraint.rs");
    source!(
        "third_party/rapier3d/src/dynamics/solver/velocity_solver.rs",
        "../../third_party/rapier3d/src/dynamics/solver/velocity_solver.rs"
    );
    source!(
        "third_party/rapier3d/src/dynamics/solver/island_solver.rs",
        "../../third_party/rapier3d/src/dynamics/solver/island_solver.rs"
    );
    source!(
        "third_party/rapier3d/src/pipeline/physics_pipeline.rs",
        "../../third_party/rapier3d/src/pipeline/physics_pipeline.rs"
    );
    source!(
        "third_party/rapier3d/src/pipeline/multibody_motor_impulse.rs",
        "../../third_party/rapier3d/src/pipeline/multibody_motor_impulse.rs"
    );
    source!(
        "third_party/rapier3d/src/dynamics/rigid_body.rs",
        "../../third_party/rapier3d/src/dynamics/rigid_body.rs"
    );
    source!(
        "third_party/rapier3d/src/dynamics/rigid_body_components.rs",
        "../../third_party/rapier3d/src/dynamics/rigid_body_components.rs"
    );
    sources
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
    fn new(cell: &Cell, enabled: bool) -> Result<Self> {
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
        let joint = fixture_joint(cell);
        let mut multibody_joints = MultibodyJointSet::new();
        let joint = multibody_joints
            .insert(parent, child, joint, true)
            .ok_or("joint insertion failed")?;
        let (multibody, _) = multibody_joints
            .get_mut(joint)
            .ok_or("missing initial multibody")?;
        // Public FK removes the default free-root DOFs without advancing time.
        multibody.forward_kinematics(&bodies, true);
        ensure(
            multibody.ndofs() == 1,
            "fixed-parent fixture is not one DOF",
        )?;
        multibody.damping_mut().fill(0.0);
        ensure(multibody.set_armature(0, 0.0), "zero armature rejected")?;
        multibody.generalized_velocity_mut().fill(0.0);
        let mut pipeline = PhysicsPipeline::new();
        pipeline.set_multibody_motor_impulse_telemetry_enabled(enabled);
        Ok(Self {
            pipeline,
            parameters: IntegrationParameters {
                dt: OUTER_DT_S,
                num_solver_iterations: NonZeroUsize::new(cell.solver_small_steps)
                    .ok_or("zero small-step count")?,
                num_internal_pgs_iterations: cell.internal_pgs_passes,
                num_internal_stabilization_iterations: STABILIZATION_PASSES,
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
    fn state(&self, completed: bool) -> Result<State> {
        let (multibody, link_id) = self
            .multibody_joints
            .get(self.joint)
            .ok_or("missing multibody")?;
        ensure(link_id == 1, "unexpected child link ordinal")?;
        let bodies = [
            (self.parent, &self.bodies[self.parent]),
            (self.child, &self.bodies[self.child]),
        ];
        state_from_native(&bodies, multibody, completed)
    }
}

fn fixture_joint(cell: &Cell) -> RevoluteJoint {
    let (stiffness, damping) = cell.gain.coefficients();
    let mut joint = RevoluteJointBuilder::new(Vector::x_axis())
        .motor_model(MotorModel::ForceBased)
        .motor_position(cell.target_position_rad.value() as f32, stiffness, damping)
        .motor_max_force(CAP_NM)
        .contacts_enabled(false)
        .build();
    // The fixture specifies identity/X frames; avoid a generated auxiliary basis.
    joint.data.local_frame1 = Isometry::identity();
    joint.data.local_frame2 = Isometry::identity();
    joint
}

// A source-bound public serde projection; this is not access to a public coords API.
#[derive(Debug, Serialize, Deserialize)]
struct SerializedJoint {
    data: GenericJoint,
    kinematic: bool,
    coords: SpacialVector<f32>,
    joint_rot: Rotation<f32>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
struct EntityBytes {
    kind: String,
    handle: [u32; 2],
    bytes: Vec<u8>,
    sha256: String,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
struct State {
    native_entities: Vec<EntityBytes>,
    exposed_state_f32_bits: Vec<u32>,
    actual_serialized_coordinate_rad: Scalar,
    generalized_velocity_rad_s: Scalar,
    native_body_inertia_x_kg_m2: Scalar,
    post_step_lu_inverse_inertia_kg_m2_inv: Option<Scalar>,
    initial_fixture_checks: Option<BTreeMap<String, bool>>,
    canonical_words: Vec<u64>,
}

fn entity_bytes(
    kind: &str,
    handle: RigidBodyHandle,
    value: &impl Serialize,
) -> Result<EntityBytes> {
    let bytes = bincode::serialize(value)?;
    let (index, generation) = handle.into_raw_parts();
    Ok(EntityBytes {
        kind: kind.to_owned(),
        handle: [index, generation],
        sha256: hash(&bytes),
        bytes,
    })
}

fn state_words(entities: &[EntityBytes], exposed: &[u32]) -> Vec<u64> {
    let mut words = vec![1, entities.len() as u64];
    for entity in entities {
        words.extend([
            u64::from(entity.kind == "multibody"),
            u64::from(entity.handle[0]),
            u64::from(entity.handle[1]),
            entity.bytes.len() as u64,
        ]);
        for chunk in entity.bytes.chunks(8) {
            let mut bytes = [0_u8; 8];
            bytes[..chunk.len()].copy_from_slice(chunk);
            words.push(u64::from_le_bytes(bytes));
        }
    }
    words.push(exposed.len() as u64);
    words.extend(exposed.iter().map(|value| u64::from(*value)));
    words
}

fn state_from_native(
    bodies: &[(RigidBodyHandle, &RigidBody)],
    multibody: &Multibody,
    completed: bool,
) -> Result<State> {
    ensure(
        bodies.len() == 2 && bodies[0].0.into_raw_parts() < bodies[1].0.into_raw_parts(),
        "unstable body ordering",
    )?;
    ensure(
        multibody.ndofs() == 1 && multibody.num_links() == 2,
        "serialized fixture is not one DOF/two links",
    )?;
    let link = multibody.link(1).ok_or("missing serialized child link")?;
    ensure(
        link.rigid_body_handle() == bodies[1].0,
        "serialized child handle differs",
    )?;
    let joint_bytes = bincode::serialize(link.joint())?;
    let joint: SerializedJoint = bincode::deserialize(&joint_bytes)?;
    ensure(
        bincode::serialize(&joint)? == joint_bytes,
        "native joint serde layout differs from declared projection",
    )?;
    let mut entities = Vec::new();
    let mut exposed = Vec::new();
    for &(handle, body) in bodies {
        entities.push(entity_bytes("rigid_body", handle, body)?);
        for value in body
            .position()
            .translation
            .vector
            .iter()
            .chain(body.position().rotation.coords.iter())
            .chain(body.linvel().iter())
            .chain(body.angvel().iter())
            .chain(body.user_force().iter())
            .chain(body.user_torque().iter())
        {
            exposed.push(value.to_bits());
        }
    }
    entities.push(entity_bytes("multibody", bodies[1].0, multibody)?);
    exposed.extend(
        multibody
            .generalized_velocity()
            .iter()
            .map(|value| value.to_bits()),
    );
    for native_link in multibody.links() {
        let projection: SerializedJoint =
            bincode::deserialize(&bincode::serialize(native_link.joint())?)?;
        exposed.extend(projection.coords.iter().map(|value| value.to_bits()));
        exposed.extend(
            native_link
                .joint()
                .data
                .motors
                .iter()
                .map(|motor| motor.impulse.to_bits()),
        );
    }
    let post_step_lu_inverse_inertia_kg_m2_inv = completed
        .then(|| {
            multibody
                .inv_augmented_mass()
                .solve(&DVector::from_element(1, 1.0))
                .map(|result| Scalar::f32(result[0]))
        })
        .flatten();
    Ok(State {
        canonical_words: state_words(&entities, &exposed),
        native_entities: entities,
        exposed_state_f32_bits: exposed,
        actual_serialized_coordinate_rad: Scalar::f32(joint.coords[3]),
        generalized_velocity_rad_s: Scalar::f32(multibody.joint_velocity(link)[0]),
        native_body_inertia_x_kg_m2: Scalar::f32(
            bodies[1]
                .1
                .mass_properties()
                .effective_angular_inertia()
                .m11,
        ),
        post_step_lu_inverse_inertia_kg_m2_inv,
        initial_fixture_checks: (!completed)
            .then(|| initial_fixture_checks(bodies, multibody, &joint)),
    })
}

fn initial_fixture_checks(
    bodies: &[(RigidBodyHandle, &RigidBody)],
    multibody: &Multibody,
    joint: &SerializedJoint,
) -> BTreeMap<String, bool> {
    let child = bodies[1].1;
    let inertia = child.mass_properties().effective_angular_inertia();
    let mut checks = BTreeMap::new();
    checks.insert(
        "fixed_identity_parent_dynamic_child".into(),
        bodies[0].1.is_fixed()
            && child.is_dynamic()
            && bodies[0].1.position() == &Isometry::identity()
            && child.position() == &Isometry::identity(),
    );
    checks.insert(
        "coincident_zero_com_pivot".into(),
        child
            .mass_properties()
            .world_com
            .coords
            .iter()
            .all(|v| *v == 0.0)
            && child
                .mass_properties()
                .local_mprops
                .local_com
                .coords
                .iter()
                .all(|v| *v == 0.0)
            && joint
                .data
                .local_frame1
                .translation
                .vector
                .iter()
                .chain(joint.data.local_frame2.translation.vector.iter())
                .all(|v| *v == 0.0),
    );
    checks.insert(
        "unit_mass_isotropic_inertia".into(),
        child.mass() == 1.0
            && [inertia.m11, inertia.m22, inertia.m33]
                .into_iter()
                .all(|value| close_inertia(f64::from(value), f64::from(inertia.m11)))
            && [inertia.m12, inertia.m13, inertia.m23]
                .into_iter()
                .all(|value| close_inertia(f64::from(value), 0.0)),
    );
    checks.insert(
        "revolute_x_force_based".into(),
        joint.data.locked_axes == JointAxesMask::LOCKED_REVOLUTE_AXES
            && joint.data.motors[JointAxis::AngX as usize].model == MotorModel::ForceBased
            && joint.data.local_frame1 == Isometry::identity()
            && joint.data.local_frame2 == Isometry::identity()
            && joint.data.local_axis1() == Vector::x_axis()
            && joint.data.local_axis2() == Vector::x_axis(),
    );
    checks.insert(
        "initial_coordinate_velocity_zero".into(),
        joint.coords.iter().all(|q| *q == 0.0)
            && multibody.generalized_velocity().iter().all(|v| *v == 0.0)
            && child
                .linvel()
                .iter()
                .chain(child.angvel().iter())
                .all(|v| *v == 0.0),
    );
    checks.insert(
        "no_passive_or_body_damping".into(),
        multibody.damping().iter().all(|v| *v == 0.0)
            && child.linear_damping() == 0.0
            && child.angular_damping() == 0.0,
    );
    checks.insert(
        "no_extra_iterations_or_user_wrench".into(),
        child.additional_solver_iterations() == 0
            && child
                .user_force()
                .iter()
                .chain(child.user_torque().iter())
                .all(|v| *v == 0.0),
    );
    checks
}

fn verify_state(state: &State, completed: bool) -> Result<()> {
    ensure(
        state.native_entities.len() == 3,
        "serialized state entity count differs",
    )?;
    let entities = &state.native_entities;
    ensure(
        entities[0].kind == "rigid_body"
            && entities[1].kind == "rigid_body"
            && entities[2].kind == "multibody",
        "serialized entity kinds/order differ",
    )?;
    for entity in entities {
        ensure(
            hash(&entity.bytes) == entity.sha256,
            "serialized entity digest differs",
        )?;
    }
    let parent: RigidBody = bincode::deserialize(&entities[0].bytes)?;
    let child: RigidBody = bincode::deserialize(&entities[1].bytes)?;
    let multibody: Multibody = bincode::deserialize(&entities[2].bytes)?;
    let parent_handle =
        RigidBodyHandle::from_raw_parts(entities[0].handle[0], entities[0].handle[1]);
    let child_handle =
        RigidBodyHandle::from_raw_parts(entities[1].handle[0], entities[1].handle[1]);
    ensure(
        entities[2].handle == entities[1].handle,
        "multibody snapshot identity differs",
    )?;
    let rebuilt = state_from_native(
        &[(parent_handle, &parent), (child_handle, &child)],
        &multibody,
        completed,
    )?;
    ensure(
        exact_json(state, &rebuilt)?,
        "state fields do not match retained native bytes",
    )
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum MotorValue {
    Revolute {
        applied_impulse_nm_s: Scalar,
        effective_effort_ceiling_nm: Option<Scalar>,
        rounded_impulse_bound_nm_s: Option<Scalar>,
    },
    Prismatic {
        applied_impulse_n_s: Scalar,
        effective_force_ceiling_n: Option<Scalar>,
        rounded_impulse_bound_n_s: Option<Scalar>,
    },
}

impl MotorValue {
    fn capture(value: MultibodyMotorImpulseValue) -> Self {
        match value {
            MultibodyMotorImpulseValue::Revolute {
                applied_impulse_nm_s,
                effective_effort_ceiling_nm,
                rounded_impulse_bound_nm_s,
            } => Self::Revolute {
                applied_impulse_nm_s: Scalar::f64(applied_impulse_nm_s),
                effective_effort_ceiling_nm: effective_effort_ceiling_nm.map(Scalar::f64),
                rounded_impulse_bound_nm_s: rounded_impulse_bound_nm_s.map(Scalar::f64),
            },
            MultibodyMotorImpulseValue::Prismatic {
                applied_impulse_n_s,
                effective_force_ceiling_n,
                rounded_impulse_bound_n_s,
            } => Self::Prismatic {
                applied_impulse_n_s: Scalar::f64(applied_impulse_n_s),
                effective_force_ceiling_n: effective_force_ceiling_n.map(Scalar::f64),
                rounded_impulse_bound_n_s: rounded_impulse_bound_n_s.map(Scalar::f64),
            },
        }
    }
    fn applied(&self) -> Scalar {
        match self {
            Self::Revolute {
                applied_impulse_nm_s,
                ..
            } => *applied_impulse_nm_s,
            Self::Prismatic {
                applied_impulse_n_s,
                ..
            } => *applied_impulse_n_s,
        }
    }
    fn ceiling(&self) -> Option<Scalar> {
        match self {
            Self::Revolute {
                effective_effort_ceiling_nm,
                ..
            } => *effective_effort_ceiling_nm,
            Self::Prismatic {
                effective_force_ceiling_n,
                ..
            } => *effective_force_ceiling_n,
        }
    }
    fn bound(&self) -> Option<Scalar> {
        match self {
            Self::Revolute {
                rounded_impulse_bound_nm_s,
                ..
            } => *rounded_impulse_bound_nm_s,
            Self::Prismatic {
                rounded_impulse_bound_n_s,
                ..
            } => *rounded_impulse_bound_n_s,
        }
    }
    fn is_revolute(&self) -> bool {
        matches!(self, Self::Revolute { .. })
    }
    fn words(&self, words: &mut Vec<u64>) {
        words.extend([
            u64::from(!self.is_revolute()),
            self.applied().ieee754_f64_bits,
        ]);
        for optional in [self.ceiling(), self.bound()] {
            words.push(u64::from(optional.is_some()));
            if let Some(value) = optional {
                words.push(value.ieee754_f64_bits);
            }
        }
    }
    fn validate(&self) -> Result<()> {
        self.applied().validate()?;
        for value in [self.ceiling(), self.bound()].into_iter().flatten() {
            value.validate()?;
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
struct Slice {
    ccd_partition_index: usize,
    internal_slice_index: usize,
    expected_internal_slice_count: usize,
    duration_s: Scalar,
    impulse: MotorValue,
    finite_positive_bound_eligible: bool,
    finite_positive_bound_active: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
struct MotorRecord {
    joint_handle: [u32; 2],
    link_id: usize,
    joint_dof_id: usize,
    axis: u32,
    impulse: MotorValue,
    observed_solver_interval_s: Scalar,
    observed_solver_slice_count: usize,
    full_step_coverage: bool,
    observed_partition_count: usize,
    finite_positive_bound_eligible_slice_count: usize,
    finite_positive_bound_eligible_duration_s: Scalar,
    finite_positive_bound_active_slice_count: usize,
    finite_positive_bound_active_duration_s: Scalar,
    slices: Vec<Slice>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
struct Telemetry {
    numerical_outer_interval_s: Scalar,
    ccd_partition_durations_s: Vec<Scalar>,
    motors: Vec<MotorRecord>,
    canonical_words: Vec<u64>,
}

impl Telemetry {
    fn capture(report: &MultibodyMotorImpulseTelemetry) -> Self {
        let motors = report
            .motors
            .iter()
            .map(|record| {
                let (index, generation) = record.joint_handle.into_raw_parts();
                MotorRecord {
                    joint_handle: [index, generation],
                    link_id: record.link_id,
                    joint_dof_id: record.joint_dof_id,
                    axis: record.axis as u32,
                    impulse: MotorValue::capture(record.impulse),
                    observed_solver_interval_s: Scalar::f64(record.observed_solver_interval_s),
                    observed_solver_slice_count: record.observed_solver_slice_count,
                    full_step_coverage: record.full_step_coverage,
                    observed_partition_count: record.observed_partition_count,
                    finite_positive_bound_eligible_slice_count: record
                        .finite_positive_bound_eligible_slice_count,
                    finite_positive_bound_eligible_duration_s: Scalar::f64(
                        record.finite_positive_bound_eligible_duration_s,
                    ),
                    finite_positive_bound_active_slice_count: record
                        .finite_positive_bound_active_slice_count,
                    finite_positive_bound_active_duration_s: Scalar::f64(
                        record.finite_positive_bound_active_duration_s,
                    ),
                    slices: record
                        .slices
                        .iter()
                        .map(|slice| Slice {
                            ccd_partition_index: slice.ccd_partition_index,
                            internal_slice_index: slice.internal_slice_index,
                            expected_internal_slice_count: slice.expected_internal_slice_count,
                            duration_s: Scalar::f64(slice.duration_s),
                            impulse: MotorValue::capture(slice.impulse),
                            finite_positive_bound_eligible: slice.finite_positive_bound_eligible,
                            finite_positive_bound_active: slice.finite_positive_bound_active,
                        })
                        .collect(),
                }
            })
            .collect();
        let mut result = Self {
            numerical_outer_interval_s: Scalar::f64(report.numerical_outer_interval_s),
            ccd_partition_durations_s: report
                .ccd_partition_durations_s
                .iter()
                .copied()
                .map(Scalar::f64)
                .collect(),
            motors,
            canonical_words: Vec::new(),
        };
        result.canonical_words = result.words();
        result
    }
    fn words(&self) -> Vec<u64> {
        let mut words = vec![
            1,
            self.numerical_outer_interval_s.ieee754_f64_bits,
            self.ccd_partition_durations_s.len() as u64,
        ];
        words.extend(
            self.ccd_partition_durations_s
                .iter()
                .map(|value| value.ieee754_f64_bits),
        );
        words.push(self.motors.len() as u64);
        for record in &self.motors {
            words.extend([
                u64::from(record.joint_handle[0]),
                u64::from(record.joint_handle[1]),
                record.link_id as u64,
                record.joint_dof_id as u64,
                u64::from(record.axis),
            ]);
            record.impulse.words(&mut words);
            words.extend([
                record.observed_solver_interval_s.ieee754_f64_bits,
                record.observed_solver_slice_count as u64,
                u64::from(record.full_step_coverage),
                record.observed_partition_count as u64,
                record.finite_positive_bound_eligible_slice_count as u64,
                record
                    .finite_positive_bound_eligible_duration_s
                    .ieee754_f64_bits,
                record.finite_positive_bound_active_slice_count as u64,
                record
                    .finite_positive_bound_active_duration_s
                    .ieee754_f64_bits,
                record.slices.len() as u64,
            ]);
            for slice in &record.slices {
                words.extend([
                    slice.ccd_partition_index as u64,
                    slice.internal_slice_index as u64,
                    slice.expected_internal_slice_count as u64,
                    slice.duration_s.ieee754_f64_bits,
                    u64::from(slice.finite_positive_bound_eligible),
                    u64::from(slice.finite_positive_bound_active),
                ]);
                slice.impulse.words(&mut words);
            }
        }
        words
    }
    fn validate(&self) -> Result<()> {
        self.numerical_outer_interval_s.validate()?;
        for duration in &self.ccd_partition_durations_s {
            duration.validate()?;
        }
        for record in &self.motors {
            record.impulse.validate()?;
            for duration in [
                record.observed_solver_interval_s,
                record.finite_positive_bound_eligible_duration_s,
                record.finite_positive_bound_active_duration_s,
            ] {
                duration.validate()?;
            }
            for slice in &record.slices {
                slice.duration_s.validate()?;
                slice.impulse.validate()?;
            }
        }
        ensure(
            self.words() == self.canonical_words,
            "telemetry fields/canonical words differ",
        )
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
struct Run {
    collector_enabled: bool,
    world_created: bool,
    completed_step: bool,
    initial: Option<State>,
    final_state: Option<State>,
    telemetry: Option<Telemetry>,
    error: Option<String>,
}

fn run_once(cell: &Cell, enabled: bool) -> Run {
    let mut run = Run {
        collector_enabled: enabled,
        world_created: false,
        completed_step: false,
        initial: None,
        final_state: None,
        telemetry: None,
        error: None,
    };
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| -> Result<()> {
        let mut fixture = Fixture::new(cell, enabled)?;
        run.world_created = true;
        run.initial = Some(fixture.state(false)?);
        fixture.step();
        run.completed_step = true;
        run.telemetry = fixture
            .pipeline
            .completed_multibody_motor_impulses()
            .map(Telemetry::capture);
        run.final_state = Some(fixture.state(true)?);
        Ok(())
    }));
    run.error = match result {
        Ok(Ok(())) => None,
        Ok(Err(error)) => Some(error.to_string()),
        Err(payload) => Some(format!(
            "physics/capture panic: {}",
            payload
                .downcast_ref::<String>()
                .map(String::as_str)
                .or_else(|| payload.downcast_ref::<&str>().copied())
                .unwrap_or("non-string payload")
        )),
    };
    run
}

fn inverse32(value: f32) -> f32 {
    if value.abs() <= 1.0e-20 {
        0.0
    } else {
        1.0 / value
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
struct Oracle {
    actual_slice_h_s: Scalar,
    native_api_a_s_inv: Scalar,
    native_api_cfm_coeff: Scalar,
    native_api_g_kg_m2_inv: Scalar,
    independently_rounded_a_s_inv: Scalar,
    independently_rounded_g_kg_m2_inv: Scalar,
    independently_rounded_rhs_rad_s: Scalar,
    pre_body_derived_inverse_inertia_kg_m2_inv: Scalar,
    independently_rounded_bound_nm_s: Scalar,
    actual_native_bound_nm_s: Option<Scalar>,
    expected_applied_impulse_nm_s: Scalar,
    actual_applied_impulse_nm_s: Scalar,
    error_nm_s: Scalar,
    tolerance_nm_s: Scalar,
    fixed_point_residual_rad_s: Scalar,
    native_and_independent_coefficients_bits_match: bool,
    native_bound_matches_independent_rounding: bool,
}

fn first_slice_oracle(cell: &Cell, initial: &State, slice: &Slice) -> Option<Oracle> {
    // A retained later slice cannot substitute for an unavailable first slice.
    if slice.ccd_partition_index != 0 || slice.internal_slice_index != 0 {
        return None;
    }
    let h = slice.duration_s.value() as f32;
    let (k, d) = cell.gain.coefficients();
    let (a, cfm, g) = MotorModel::ForceBased.combine_coefficients(h, k, d);
    let independent_a = k * inverse32(h * k + d);
    let independent_g = inverse32(h * h * k + h * d);
    let q = initial.actual_serialized_coordinate_rad.value() as f32;
    let target = cell.target_position_rad.value() as f32;
    let mut rhs = 0.0_f32;
    if independent_a != 0.0 {
        rhs += (q - target) * independent_a;
    }
    rhs += -0.0;
    let inverse_inertia = inverse32(initial.native_body_inertia_x_kg_m2.value() as f32);
    let rounded_bound = CAP_NM * h;
    let bound = slice.impulse.bound().map(Scalar::value).unwrap_or(f64::NAN);
    let v = initial.generalized_velocity_rad_s.value();
    let expected = frozen_fixed_point(
        f64::from(rhs),
        v,
        f64::from(inverse_inertia),
        f64::from(independent_g),
        bound,
    );
    let actual = slice.impulse.applied().value();
    Some(Oracle {
        actual_slice_h_s: Scalar::f32(h),
        native_api_a_s_inv: Scalar::f32(a),
        native_api_cfm_coeff: Scalar::f32(cfm),
        native_api_g_kg_m2_inv: Scalar::f32(g),
        independently_rounded_a_s_inv: Scalar::f32(independent_a),
        independently_rounded_g_kg_m2_inv: Scalar::f32(independent_g),
        independently_rounded_rhs_rad_s: Scalar::f32(rhs),
        pre_body_derived_inverse_inertia_kg_m2_inv: Scalar::f32(inverse_inertia),
        independently_rounded_bound_nm_s: Scalar::f32(rounded_bound),
        actual_native_bound_nm_s: slice.impulse.bound(),
        expected_applied_impulse_nm_s: Scalar::f64(expected),
        actual_applied_impulse_nm_s: slice.impulse.applied(),
        error_nm_s: Scalar::f64((actual - expected).abs()),
        tolerance_nm_s: Scalar::f64(1.0e-8_f64.max(0.01 * expected.abs())),
        fixed_point_residual_rad_s: Scalar::f64(
            (f64::from(rhs) + v) + (f64::from(inverse_inertia) + f64::from(independent_g)) * actual,
        ),
        native_and_independent_coefficients_bits_match: a.to_bits() == independent_a.to_bits()
            && g.to_bits() == independent_g.to_bits()
            && cfm.to_bits() == 0.0_f32.to_bits(),
        native_bound_matches_independent_rounding: bound.to_bits()
            == f64::from(rounded_bound).to_bits(),
    })
}

fn frozen_fixed_point(
    rhs: f64,
    velocity: f64,
    inverse_inertia: f64,
    cfm_gain: f64,
    bound: f64,
) -> f64 {
    if ![rhs, velocity, inverse_inertia, cfm_gain, bound]
        .into_iter()
        .all(f64::is_finite)
        || bound < 0.0
        || inverse_inertia + cfm_gain <= 0.0
    {
        return f64::NAN;
    }
    (-(rhs + velocity) / (inverse_inertia + cfm_gain)).clamp(-bound, bound)
}

fn close_inertia(actual: f64, expected: f64) -> bool {
    actual.is_finite()
        && expected.is_finite()
        && (actual - expected).abs()
            <= INERTIA_ABS_TOLERANCE.max(INERTIA_REL_TOLERANCE * expected.abs())
}

fn full_coverage(cell: &Cell, telemetry: &Telemetry, record: &MotorRecord) -> bool {
    let h = OUTER_DT_S / cell.solver_small_steps as f32;
    let expected_h = f64::from(h).to_bits();
    let expected_bound = f64::from(CAP_NM * h).to_bits();
    let mut sum_h = 0.0_f64;
    let mut sum_p: Option<f64> = None;
    let mut active = 0;
    let mut active_h = 0.0_f64;
    let slices_valid = record.slices.iter().enumerate().all(|(index, slice)| {
        sum_h += slice.duration_s.value();
        let p = slice.impulse.applied().value();
        sum_p = Some(sum_p.map_or(p, |previous| previous + p));
        if slice.finite_positive_bound_active {
            active += 1;
            active_h += slice.duration_s.value();
        }
        slice.ccd_partition_index == 0
            && slice.internal_slice_index == index
            && slice.expected_internal_slice_count == cell.solver_small_steps
            && slice.duration_s.ieee754_f64_bits == expected_h
            && slice.impulse.is_revolute()
            && slice.impulse.ceiling().map(|v| v.ieee754_f64_bits)
                == Some(f64::from(CAP_NM).to_bits())
            && slice.impulse.bound().map(|v| v.ieee754_f64_bits) == Some(expected_bound)
            && slice.impulse.applied().value().is_finite()
            && slice.finite_positive_bound_eligible
            && slice.finite_positive_bound_active
                == (slice.impulse.applied().value().abs().to_bits() == expected_bound)
    });
    slices_valid
        && record.full_step_coverage
        && telemetry.motors.len() == 1
        && record.axis == JointAxis::AngX as u32
        && record.link_id == 1
        && record.joint_dof_id == 0
        && record.slices.len() == cell.solver_small_steps
        && record.observed_solver_slice_count == cell.solver_small_steps
        && record.observed_partition_count == 1
        && record.observed_solver_interval_s.ieee754_f64_bits == sum_h.to_bits()
        && record.impulse.applied().ieee754_f64_bits == sum_p.unwrap_or(f64::NAN).to_bits()
        && record.finite_positive_bound_eligible_slice_count == cell.solver_small_steps
        && record
            .finite_positive_bound_eligible_duration_s
            .ieee754_f64_bits
            == sum_h.to_bits()
        && record.finite_positive_bound_active_slice_count == active
        && record
            .finite_positive_bound_active_duration_s
            .ieee754_f64_bits
            == active_h.to_bits()
        && telemetry.numerical_outer_interval_s.ieee754_f64_bits == f64::from(OUTER_DT_S).to_bits()
        && telemetry.ccd_partition_durations_s.len() == 1
        && telemetry.ccd_partition_durations_s[0].ieee754_f64_bits
            == f64::from(OUTER_DT_S).to_bits()
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
struct Assessment {
    primary_cell: bool,
    initial_fixture_qualification: bool,
    finite_exposed_state: bool,
    full_actual_slice_coverage: bool,
    pre_body_inertia_matches_nominal_sphere: bool,
    post_lu_matches_pre_body_derived_inverse: bool,
    first_slice_oracle: Option<Oracle>,
    first_slice_tolerance_pass: bool,
    zero_target_motionless: Option<bool>,
    signed_total_impulse_nm_s: Option<Scalar>,
    sum_absolute_slice_impulses_nm_s: Option<Scalar>,
    cancellation_ratio: Option<Scalar>,
    nonzero_slice_sign_reversals: Option<usize>,
    final_bound_active_duration_fraction: Option<Scalar>,
    final_kinetic_energy_j: Option<Scalar>,
}

fn assess(cell: &Cell, run: &Run) -> Assessment {
    let mut result = Assessment {
        primary_cell: cell.primary(),
        initial_fixture_qualification: false,
        finite_exposed_state: false,
        full_actual_slice_coverage: false,
        pre_body_inertia_matches_nominal_sphere: false,
        post_lu_matches_pre_body_derived_inverse: false,
        first_slice_oracle: None,
        first_slice_tolerance_pass: false,
        zero_target_motionless: None,
        signed_total_impulse_nm_s: None,
        sum_absolute_slice_impulses_nm_s: None,
        cancellation_ratio: None,
        nonzero_slice_sign_reversals: None,
        final_bound_active_duration_fraction: None,
        final_kinetic_energy_j: None,
    };
    let (Some(initial), Some(final_state), Some(telemetry)) =
        (&run.initial, &run.final_state, &run.telemetry)
    else {
        return result;
    };
    result.initial_fixture_qualification = initial
        .initial_fixture_checks
        .as_ref()
        .is_some_and(|checks| checks.len() == 7 && checks.values().all(|value| *value));
    result.finite_exposed_state = initial
        .exposed_state_f32_bits
        .iter()
        .chain(final_state.exposed_state_f32_bits.iter())
        .all(|bits| f32::from_bits(*bits).is_finite());
    let inertia = initial.native_body_inertia_x_kg_m2.value();
    let inverse = f64::from(inverse32(inertia as f32));
    result.pre_body_inertia_matches_nominal_sphere = close_inertia(inertia, 0.004);
    result.post_lu_matches_pre_body_derived_inverse = final_state
        .post_step_lu_inverse_inertia_kg_m2_inv
        .is_some_and(|actual| close_inertia(actual.value(), inverse));
    let v = final_state.generalized_velocity_rad_s.value();
    result.final_kinetic_energy_j = Some(Scalar::f64(0.5 * inertia * v * v));
    let Some(record) = telemetry.motors.first() else {
        return result;
    };
    result.full_actual_slice_coverage = full_coverage(cell, telemetry, record);
    if record.slices.is_empty() {
        return result;
    }
    let mut sum = 0.0_f64;
    let mut absolute = 0.0_f64;
    let mut previous = 0_i8;
    let mut reversals = 0;
    for (index, slice) in record.slices.iter().enumerate() {
        let p = slice.impulse.applied().value();
        if index == 0 {
            sum = p;
        } else {
            sum += p;
        }
        absolute += p.abs();
        if p != 0.0 && p.is_finite() {
            let sign = if p.is_sign_negative() { -1 } else { 1 };
            if previous != 0 && sign != previous {
                reversals += 1;
            }
            previous = sign;
        }
    }
    result.signed_total_impulse_nm_s = Some(Scalar::f64(sum));
    result.sum_absolute_slice_impulses_nm_s = Some(Scalar::f64(absolute));
    result.cancellation_ratio = (absolute > 0.0).then(|| Scalar::f64(1.0 - sum.abs() / absolute));
    result.nonzero_slice_sign_reversals = Some(reversals);
    let eligible = record.finite_positive_bound_eligible_duration_s.value();
    result.final_bound_active_duration_fraction = (eligible > 0.0)
        .then(|| Scalar::f64(record.finite_positive_bound_active_duration_s.value() / eligible));
    if cell.target_position_rad.value() == 0.0 {
        result.zero_target_motionless = Some(
            result.initial_fixture_qualification
                && result.finite_exposed_state
                && result.full_actual_slice_coverage
                && initial.exposed_state_f32_bits.len() == final_state.exposed_state_f32_bits.len()
                && initial
                    .exposed_state_f32_bits
                    .iter()
                    .zip(&final_state.exposed_state_f32_bits)
                    .all(|(a, b)| f32::from_bits(*a) == f32::from_bits(*b))
                && record
                    .slices
                    .iter()
                    .all(|slice| slice.impulse.applied().value() == 0.0),
        );
    }
    if let Some(oracle) = record
        .slices
        .iter()
        .find_map(|slice| first_slice_oracle(cell, initial, slice))
    {
        result.first_slice_tolerance_pass = run.completed_step
            && run.error.is_none()
            && result.initial_fixture_qualification
            && result.finite_exposed_state
            && result.full_actual_slice_coverage
            && result.pre_body_inertia_matches_nominal_sphere
            && result.post_lu_matches_pre_body_derived_inverse
            && oracle.native_and_independent_coefficients_bits_match
            && oracle.native_bound_matches_independent_rounding
            && oracle.error_nm_s.value().is_finite()
            && oracle.error_nm_s.value() <= oracle.tolerance_nm_s.value();
        result.first_slice_oracle = Some(oracle);
    }
    result
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
struct Replay {
    native_state_exact_fresh_repeat: bool,
    native_state_exact_on_off: bool,
    telemetry_exact_fresh_repeat: bool,
    disabled_telemetry_absent: bool,
}

fn replay(runs: &[Run; 3]) -> Result<Replay> {
    let available = |run: &Run| {
        run.completed_step
            && run.error.is_none()
            && run.initial.is_some()
            && run.final_state.is_some()
    };
    Ok(Replay {
        native_state_exact_fresh_repeat: available(&runs[0])
            && available(&runs[1])
            && exact_json(
                &(&runs[0].initial, &runs[0].final_state),
                &(&runs[1].initial, &runs[1].final_state),
            )?,
        native_state_exact_on_off: available(&runs[0])
            && available(&runs[2])
            && exact_json(
                &(&runs[0].initial, &runs[0].final_state),
                &(&runs[2].initial, &runs[2].final_state),
            )?,
        telemetry_exact_fresh_repeat: runs[0].telemetry.is_some()
            && exact_json(&runs[0].telemetry, &runs[1].telemetry)?,
        disabled_telemetry_absent: !runs[2].collector_enabled && runs[2].telemetry.is_none(),
    })
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
struct Case {
    cell: Cell,
    runs: [Run; 3],
    replay: Replay,
    assessment: Assessment,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
struct Summary {
    collection_complete: bool,
    worlds_attempted: usize,
    worlds_created: usize,
    completed_worlds: usize,
    replay_qualification_pass: bool,
    primary_calibration_pass: bool,
    primary_nonzero_cells: usize,
    primary_nonzero_passes: usize,
    zero_target_cells: usize,
    zero_target_passes: usize,
}

fn summarize(cases: &[Case]) -> Summary {
    let runs: Vec<&Run> = cases.iter().flat_map(|case| &case.runs).collect();
    let completed_worlds = runs
        .iter()
        .filter(|run| run.completed_step && run.error.is_none())
        .count();
    let primary_nonzero_cells = cases.iter().filter(|case| case.cell.primary()).count();
    let primary_nonzero_passes = cases
        .iter()
        .filter(|case| case.cell.primary() && case.assessment.first_slice_tolerance_pass)
        .count();
    let zero_target_cells = cases
        .iter()
        .filter(|case| case.cell.target_position_rad.value() == 0.0)
        .count();
    let zero_target_passes = cases
        .iter()
        .filter(|case| case.assessment.zero_target_motionless == Some(true))
        .count();
    Summary {
        collection_complete: cases.len() == 24 && runs.len() == 72 && completed_worlds == 72,
        worlds_attempted: runs.len(),
        worlds_created: runs.iter().filter(|run| run.world_created).count(),
        completed_worlds,
        replay_qualification_pass: cases.len() == 24
            && cases.iter().all(|case| {
                case.replay.native_state_exact_fresh_repeat
                    && case.replay.native_state_exact_on_off
                    && case.replay.telemetry_exact_fresh_repeat
                    && case.replay.disabled_telemetry_absent
            }),
        primary_calibration_pass: primary_nonzero_cells == 4
            && primary_nonzero_passes == 4
            && zero_target_cells == 8
            && zero_target_passes == 8,
        primary_nonzero_cells,
        primary_nonzero_passes,
        zero_target_cells,
        zero_target_passes,
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
struct Report {
    schema_version: u32,
    protocol: Value,
    selected_source_sha256: BTreeMap<String, String>,
    cases: Vec<Case>,
    numerical_sensitivity: Vec<Sensitivity>,
    summary: Summary,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
struct Sensitivity {
    factor: String,
    left_case_index: usize,
    right_case_index: usize,
    first_slice_applied_impulse_difference_nm_s: Option<Scalar>,
    final_coordinate_difference_rad: Option<Scalar>,
    final_generalized_velocity_difference_rad_s: Option<Scalar>,
}

fn sensitivity(cases: &[Case]) -> Vec<Sensitivity> {
    let mut rows = Vec::new();
    for (left_index, left) in cases.iter().enumerate() {
        for (right_index, right) in cases.iter().enumerate().skip(left_index + 1) {
            if left.cell.gain != right.cell.gain
                || left.cell.target_position_rad != right.cell.target_position_rad
            {
                continue;
            }
            let factor = if left.cell.internal_pgs_passes == right.cell.internal_pgs_passes
                && left.cell.solver_small_steps != right.cell.solver_small_steps
            {
                "N1_to_N32"
            } else if left.cell.solver_small_steps == right.cell.solver_small_steps
                && left.cell.internal_pgs_passes != right.cell.internal_pgs_passes
            {
                "PGS1_to_PGS2"
            } else {
                continue;
            };
            let first = left
                .assessment
                .first_slice_oracle
                .as_ref()
                .zip(right.assessment.first_slice_oracle.as_ref())
                .map(|(a, b)| {
                    Scalar::f64(
                        b.actual_applied_impulse_nm_s.value()
                            - a.actual_applied_impulse_nm_s.value(),
                    )
                });
            let states = left.runs[0]
                .final_state
                .as_ref()
                .zip(right.runs[0].final_state.as_ref());
            rows.push(Sensitivity {
                factor: factor.to_owned(),
                left_case_index: left_index,
                right_case_index: right_index,
                first_slice_applied_impulse_difference_nm_s: first,
                final_coordinate_difference_rad: states.map(|(a, b)| {
                    Scalar::f64(
                        b.actual_serialized_coordinate_rad.value()
                            - a.actual_serialized_coordinate_rad.value(),
                    )
                }),
                final_generalized_velocity_difference_rad_s: states.map(|(a, b)| {
                    Scalar::f64(
                        b.generalized_velocity_rad_s.value() - a.generalized_velocity_rad_s.value(),
                    )
                }),
            });
        }
    }
    rows
}

fn collect_report() -> Report {
    let cases: Vec<Case> = fixed_cells()
        .into_iter()
        .map(|cell| {
            let runs = [
                run_once(&cell, true),
                run_once(&cell, true),
                run_once(&cell, false),
            ];
            let replay = replay(&runs).expect("in-memory evidence serialization is infallible");
            let assessment = assess(&cell, &runs[0]);
            Case {
                cell,
                runs,
                replay,
                assessment,
            }
        })
        .collect();
    Report {
        schema_version: 1,
        protocol: protocol(),
        selected_source_sha256: selected_sources(),
        summary: summarize(&cases),
        numerical_sensitivity: sensitivity(&cases),
        cases,
    }
}

fn verify_report(report: &Report) -> Result<()> {
    ensure(
        report.schema_version == 1 && report.protocol == protocol(),
        "report protocol differs from the fixed fixture",
    )?;
    ensure(
        report.selected_source_sha256 == selected_sources(),
        "report selected sources are stale/mixed",
    )?;
    let expected = fixed_cells();
    ensure(
        report.cases.len() == expected.len(),
        "report does not retain all 24 cases",
    )?;
    for (case, cell) in report.cases.iter().zip(expected) {
        ensure(
            exact_json(&case.cell, &cell)?,
            "report case configuration/order differs",
        )?;
        ensure(
            case.runs
                .iter()
                .map(|run| run.collector_enabled)
                .eq([true, true, false]),
            "run ON/ON/OFF order differs",
        )?;
        for run in &case.runs {
            if let Some(state) = &run.initial {
                verify_state(state, false)?;
            }
            if let Some(state) = &run.final_state {
                verify_state(state, true)?;
            }
            if let Some(telemetry) = &run.telemetry {
                telemetry.validate()?;
            }
            ensure(
                !run.completed_step || run.world_created,
                "completed step without a created world",
            )?;
            ensure(
                !run.collector_enabled || run.world_created || run.error.is_some(),
                "missing collector world without retained error",
            )?;
        }
        ensure(
            exact_json(&case.replay, &replay(&case.runs)?)?,
            "retained replay result is inconsistent",
        )?;
        ensure(
            exact_json(&case.assessment, &assess(&case.cell, &case.runs[0]))?,
            "retained oracle/assessment is inconsistent",
        )?;
    }
    ensure(
        exact_json(&report.numerical_sensitivity, &sensitivity(&report.cases))?,
        "retained numerical sensitivity comparison differs",
    )?;
    ensure(
        exact_json(&report.summary, &summarize(&report.cases))?,
        "retained summary/calibration result is inconsistent",
    )
}

fn hash(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}
fn exact_json(a: &impl Serialize, b: &impl Serialize) -> Result<bool> {
    Ok(serde_json::to_vec(a)? == serde_json::to_vec(b)?)
}
fn ensure(condition: bool, message: &str) -> Result<()> {
    if condition {
        Ok(())
    } else {
        Err(message.into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_matrix_has_both_signs_and_all_zero_controls_without_a_best_setting() {
        let cells = fixed_cells();
        assert_eq!(cells.len(), 24);
        assert_eq!(cells.iter().filter(|cell| cell.primary()).count(), 4);
        assert_eq!(
            cells
                .iter()
                .filter(|cell| cell.target_position_rad.value() == 0.0)
                .count(),
            8
        );
        assert_eq!(cells[0].gain, Gain::G1Posture);
        assert_eq!(
            cells[0].target_position_rad.native_f32_bits,
            Some((-0.001_f32).to_bits())
        );
        assert_eq!(cells[23].gain, Gain::G1Ankle);
    }

    #[test]
    fn scalar_roundtrip_preserves_nan_payload_negative_zero_and_missing() {
        for value in [0.0, -0.0, f64::INFINITY, f64::from_bits(0x7ff8000000000025)] {
            let original = Scalar::f64(value);
            let loaded: Scalar =
                serde_json::from_slice(&serde_json::to_vec(&original).unwrap()).unwrap();
            loaded.validate().unwrap();
            assert_eq!(loaded.ieee754_f64_bits, value.to_bits());
            assert_eq!(loaded.finite.is_none(), !value.is_finite());
        }
        assert_ne!(
            Scalar::f64(-0.0).ieee754_f64_bits,
            Scalar::f64(0.0).ieee754_f64_bits
        );
        let mut invalid = Scalar::f64(1.0);
        invalid.finite = None;
        assert!(invalid.validate().is_err());
    }

    #[test]
    fn fixed_point_tolerance_is_not_replaced_with_cap_or_sign_qualification() {
        let expected = 1.0e-6_f64;
        let tolerance = 1.0e-8_f64.max(0.01 * expected.abs());
        let cap_bound = 2.0 * f64::from(OUTER_DT_S / 32.0);
        assert!((cap_bound - expected).abs() > tolerance);
        assert!(close_inertia(0.004, 0.004));
        assert!(!close_inertia(0.0041, 0.004));
    }

    #[test]
    fn canonical_native_bytes_include_length_order_and_signed_zero() {
        let entity = EntityBytes {
            kind: "rigid_body".into(),
            handle: [0, 0],
            bytes: vec![1],
            sha256: hash(&[1]),
        };
        let mut padded = entity.clone();
        padded.bytes.push(0);
        assert_ne!(
            state_words(std::slice::from_ref(&entity), &[0.0_f32.to_bits()]),
            state_words(&[padded], &[0.0_f32.to_bits()])
        );
        assert_ne!(
            state_words(std::slice::from_ref(&entity), &[0.0_f32.to_bits()]),
            state_words(&[entity], &[(-0.0_f32).to_bits()])
        );
    }

    #[test]
    fn empty_collection_cannot_claim_primary_calibration_or_replay_qualification() {
        let summary = summarize(&[]);
        assert!(!summary.collection_complete);
        assert!(!summary.primary_calibration_pass);
        assert!(!summary.replay_qualification_pass);
    }

    #[test]
    fn verifier_rejects_stale_sources_missing_cells_and_forged_summary_without_worlds() {
        let mut report = Report {
            schema_version: 1,
            protocol: protocol(),
            selected_source_sha256: selected_sources(),
            cases: Vec::new(),
            numerical_sensitivity: Vec::new(),
            summary: summarize(&[]),
        };
        assert!(verify_report(&report).is_err());
        report
            .selected_source_sha256
            .insert("Cargo.lock".into(), "stale".into());
        assert!(verify_report(&report)
            .unwrap_err()
            .to_string()
            .contains("stale/mixed"));
        report.selected_source_sha256 = selected_sources();
        report.protocol["effort_cap_nm"] = json!(200);
        assert!(verify_report(&report)
            .unwrap_err()
            .to_string()
            .contains("protocol"));
    }

    #[test]
    fn verify_and_output_are_exclusive_and_require_calibration_is_explicit() {
        assert!(Options::parse(
            ["--verify-report", "report.json", "--output", "new.json"].map(str::to_owned)
        )
        .is_err());
        let options = Options::parse(
            ["--verify-report", "report.json", "--require-calibration"].map(str::to_owned),
        )
        .unwrap();
        assert!(options.require && options.verify.is_some());
        assert!(Options::parse(["--solver-small-steps", "64"].map(str::to_owned)).is_err());
    }

    #[test]
    fn independent_fixed_point_handles_sign_cap_zero_and_unavailable_bounds() {
        assert_eq!(frozen_fixed_point(-0.5, 0.0, 1.0, 0.5, 2.0), 1.0 / 3.0);
        assert_eq!(frozen_fixed_point(0.5, 0.0, 1.0, 0.5, 2.0), -1.0 / 3.0);
        assert_eq!(frozen_fixed_point(-1.0, 0.0, 1.0, 0.0, 0.1), 0.1);
        assert_eq!(frozen_fixed_point(0.0, 0.0, 1.0, 0.5, 0.0), 0.0);
        assert!(frozen_fixed_point(-0.5, 0.0, 1.0, 0.5, f64::NAN).is_nan());
        assert!(frozen_fixed_point(-0.5, 0.0, 0.0, 0.0, 2.0).is_nan());
    }

    #[test]
    fn missing_first_slice_stays_unavailable_instead_of_using_a_later_partial_sample() {
        let cell = fixed_cells()[2].clone();
        let initial = State {
            native_entities: Vec::new(),
            exposed_state_f32_bits: Vec::new(),
            actual_serialized_coordinate_rad: Scalar::f32(0.0),
            generalized_velocity_rad_s: Scalar::f32(0.0),
            native_body_inertia_x_kg_m2: Scalar::f32(0.004),
            post_step_lu_inverse_inertia_kg_m2_inv: None,
            initial_fixture_checks: None,
            canonical_words: Vec::new(),
        };
        let h = OUTER_DT_S / cell.solver_small_steps as f32;
        let mut slice = Slice {
            ccd_partition_index: 0,
            internal_slice_index: 1,
            expected_internal_slice_count: cell.solver_small_steps,
            duration_s: Scalar::f64(f64::from(h)),
            impulse: MotorValue::Revolute {
                applied_impulse_nm_s: Scalar::f64(0.0),
                effective_effort_ceiling_nm: Some(Scalar::f64(f64::from(CAP_NM))),
                rounded_impulse_bound_nm_s: Some(Scalar::f64(f64::from(CAP_NM * h))),
            },
            finite_positive_bound_eligible: true,
            finite_positive_bound_active: false,
        };
        // The vendor retains this sample when ordinal zero was skipped as nonfinite.
        assert!(first_slice_oracle(&cell, &initial, &slice).is_none());
        slice.internal_slice_index = 0;
        slice.ccd_partition_index = 1;
        assert!(first_slice_oracle(&cell, &initial, &slice).is_none());
        slice.ccd_partition_index = 0;
        assert!(first_slice_oracle(&cell, &initial, &slice).is_some());
    }

    #[test]
    fn public_native_coefficients_match_independent_operation_order_for_every_fixed_cell() {
        for cell in fixed_cells() {
            let h = OUTER_DT_S / cell.solver_small_steps as f32;
            let (k, d) = cell.gain.coefficients();
            let (a, coefficient, g) = MotorModel::ForceBased.combine_coefficients(h, k, d);
            assert_eq!(a.to_bits(), (k * inverse32(h * k + d)).to_bits());
            assert_eq!(g.to_bits(), inverse32(h * h * k + h * d).to_bits());
            assert_eq!(coefficient.to_bits(), 0.0_f32.to_bits());
        }
    }

    #[test]
    fn fixed_joint_frames_are_identity_x_without_changing_motor_configuration() {
        for cell in fixed_cells() {
            let joint = fixture_joint(&cell);
            assert_eq!(joint.data.local_frame1, Isometry::identity());
            assert_eq!(joint.data.local_frame2, Isometry::identity());
            assert_eq!(joint.data.local_axis1(), Vector::x_axis());
            assert_eq!(joint.data.local_axis2(), Vector::x_axis());
            assert_eq!(joint.data.locked_axes, JointAxesMask::LOCKED_REVOLUTE_AXES);
            let motor = &joint.data.motors[JointAxis::AngX as usize];
            let (stiffness, damping) = cell.gain.coefficients();
            assert_eq!(
                motor.target_pos.to_bits(),
                (cell.target_position_rad.value() as f32).to_bits()
            );
            assert_eq!(motor.target_vel, 0.0);
            assert_eq!(motor.stiffness, stiffness);
            assert_eq!(motor.damping, damping);
            assert_eq!(motor.max_force, CAP_NM);
            assert_eq!(motor.model, MotorModel::ForceBased);
        }
    }

    #[test]
    fn failed_attempts_are_retained_and_a_forged_primary_success_is_rejected_without_worlds() {
        let cases: Vec<Case> = fixed_cells()
            .into_iter()
            .map(|cell| {
                let failed = |enabled| Run {
                    collector_enabled: enabled,
                    world_created: false,
                    completed_step: false,
                    initial: None,
                    final_state: None,
                    telemetry: None,
                    error: Some("synthetic unavailable fixture".to_owned()),
                };
                let runs = [failed(true), failed(true), failed(false)];
                Case {
                    assessment: assess(&cell, &runs[0]),
                    replay: replay(&runs).unwrap(),
                    cell,
                    runs,
                }
            })
            .collect();
        let mut report = Report {
            schema_version: 1,
            protocol: protocol(),
            selected_source_sha256: selected_sources(),
            summary: summarize(&cases),
            numerical_sensitivity: sensitivity(&cases),
            cases,
        };
        verify_report(&report).unwrap();
        assert_eq!(report.summary.worlds_attempted, 72);
        assert_eq!(report.summary.completed_worlds, 0);
        assert_eq!(report.numerical_sensitivity.len(), 24);
        report.summary.primary_calibration_pass = true;
        assert!(verify_report(&report)
            .unwrap_err()
            .to_string()
            .contains("summary"));
    }
}
