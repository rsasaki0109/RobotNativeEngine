//! Optional constructor inputs and distinct coordinate/velocity endpoints.
//!
//! This companion has its own schema. It does not change the version-one motor
//! stage structs or infer row construction inputs from a later pose.

use crate::dynamics::JointAxis;
use crate::math::Real;
use crate::pipeline::{motor_stage_word, MultibodyMotorStageRow};

/// Source contract identifier; compiled source hashes belong in external bindings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum MultibodyMotorConstructionSource {
    /// Actual locals from the internal unit-motor constructor, schema one.
    InternalUnitMotorConstructionV1,
}

/// Boundary of the separately versioned construction companion.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum MultibodyMotorConstructionPhase {
    /// No current-slice row storage has been initialized.
    BeforeExternalForces,
    /// External increments completed; current-slice rows remain unavailable.
    AfterExternalForces,
    /// Actual constructor inputs and initialized rows are available.
    AfterRowConstruction,
    /// Generic V has been copied to this multibody, before coordinates advance.
    BeforePositionIntegration,
    /// Coordinates advanced using the preceding integration-input velocity.
    AfterPositionIntegration,
    /// One complete joint stabilization pass, before its contact pass.
    AfterJointStabilization,
    /// Final solver slice state, before final velocity writeback.
    FinalSlice,
    /// Final generic V has been copied to multibody velocity storage.
    AfterVelocityWriteback,
}

/// Explicit availability of this event's supported internal motor rows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum MultibodyMotorConstructionAvailability {
    /// Current-slice rows have not been initialized and were not read.
    Uninitialized,
    /// Row storage is initialized but has no supported single-DoF motor.
    NoSupportedRows,
    /// At least one supported initialized row was captured.
    Available,
}

/// Raw actual inputs at one internal unit-motor construction.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct MultibodyMotorConstructionRow {
    /// Unchanged stage-v1 row, copied from the actual initialized constraint.
    pub row: MultibodyMotorStageRow,
    /// Actual constructor coordinate, in metres or radians according to axis.
    pub coordinate_word: u64,
    /// Actual motor target position, in metres or radians.
    pub target_position_word: u64,
    /// Original native motor target velocity, before optional limit clamping.
    pub target_velocity_word: u64,
    /// Actual native stiffness; units depend on gain model and axis.
    pub stiffness_word: u64,
    /// Actual native damping; units depend on gain model and axis.
    pub damping_word: u64,
    /// Actual constructor parameter duration in seconds.
    pub duration_word: u64,
    /// Actual computed inverse duration in reciprocal seconds.
    pub inverse_duration_word: u64,
    /// Actual optional limit coordinates, preserving absence and signed zeros.
    pub limits_words: Option<[u64; 2]>,
    /// Actual local target velocity after optional limit clamping.
    pub effective_target_velocity_word: u64,
    /// Actual computed motor ERP inverse-duration coefficient in reciprocal seconds.
    pub erp_inverse_duration_word: u64,
}

/// Current coordinate and actual multibody velocity at a named boundary.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct MultibodyMotorConstructionEndpoint {
    /// Current constraint snapshot using the row cache named by its event.
    pub row: MultibodyMotorStageRow,
    /// Current native joint coordinate, never substituted with a later ECS value.
    pub coordinate_word: u64,
    /// Actual multibody velocity storage, distinguished from event generic V.
    pub multibody_velocity_words: Vec<u64>,
}

/// One deterministic island/slice construction or coordinate boundary.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct MultibodyMotorConstructionEvent {
    /// Actual CCD partition index.
    pub ccd_partition_index: usize,
    /// Actual island index, drained after workers join in island order.
    pub island_index: usize,
    /// Actual small-step index.
    pub internal_slice_index: usize,
    /// Actual expected small-step count for this island.
    pub expected_internal_slice_count: usize,
    /// Actual stabilization pass, when applicable.
    pub iteration_index: Option<usize>,
    /// Exact operation boundary.
    pub phase: MultibodyMotorConstructionPhase,
    /// IEEE width of all words, 32 or 64.
    pub native_scalar_bits: u8,
    /// Actual small-step duration in seconds.
    pub duration_word: u64,
    /// Cache that initialized nested rows; absence means storage was not read.
    pub row_cache_slice_index: Option<usize>,
    /// Explicit row availability, distinct from a measured zero.
    pub availability: MultibodyMotorConstructionAvailability,
    /// Native index/generation of the root being integrated; absent at other phases.
    pub integration_multibody_handle_words: Option<[u32; 2]>,
    /// Actual full generalized solver V, not ordinary rigid-body solver storage.
    pub generalized_velocity_words: Vec<u64>,
    /// Present only at AfterRowConstruction; subsequent phases do not rebuild inputs.
    pub constructed_rows: Vec<MultibodyMotorConstructionRow>,
    /// Actual coordinate/velocity endpoints for supported current rows.
    pub endpoints: Vec<MultibodyMotorConstructionEndpoint>,
}

/// Completed read-only construction companion, disabled by default.
///
/// Empty or unsupported observations are not measured zero. This reports raw
/// inputs/endpoints, not a computed physical solution or neutrality guarantee.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct MultibodyMotorConstructionTelemetry {
    /// Independent companion schema version, currently one.
    pub schema_version: u32,
    /// Source contract; external source/compiler/binary bindings remain necessary.
    pub source_contract: MultibodyMotorConstructionSource,
    /// IEEE scalar width.
    pub native_scalar_bits: u8,
    /// Actual native outer duration in seconds.
    pub numerical_outer_interval_word: u64,
    /// Actual CCD partition durations, in recorded order.
    pub ccd_partition_duration_words: Vec<u64>,
    /// Construction and endpoint events in partition/island/native operation order.
    pub events: Vec<MultibodyMotorConstructionEvent>,
}

impl MultibodyMotorConstructionTelemetry {
    pub(crate) fn new(duration: Real) -> Self {
        Self {
            schema_version: 1,
            source_contract: MultibodyMotorConstructionSource::InternalUnitMotorConstructionV1,
            native_scalar_bits: (std::mem::size_of::<Real>() * 8) as u8,
            numerical_outer_interval_word: motor_stage_word(duration),
            ccd_partition_duration_words: Vec::new(),
            events: Vec::new(),
        }
    }
}

pub(crate) struct InternalMotorConstructionRecorder<'a> {
    pub rows: &'a mut Vec<MultibodyMotorConstructionRow>,
    pub constraint_base: usize,
    pub link_id: usize,
}

pub(crate) struct InternalMotorConstructionRowRecorder<'a> {
    pub rows: &'a mut Vec<MultibodyMotorConstructionRow>,
    pub constraint_index: usize,
    pub link_id: usize,
    pub axis: JointAxis,
}
