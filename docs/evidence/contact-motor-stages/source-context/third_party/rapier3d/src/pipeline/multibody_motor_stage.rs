use crate::dynamics::{JointAxis, MotorModel, MultibodyJointHandle};
use crate::math::Real;
use na::DVector;

/// Ordered solver boundaries for opt-in internal single-DoF motor observation.
///
/// Joint stages include other joint and limit rows. Contact stages aggregate
/// all contacts; they do not identify individual contact Jacobian columns.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum MultibodyMotorStage {
    /// Before this slice's external velocity increments; rows are unavailable.
    BeforeExternalForces,
    /// After external increments, before rebuilding joint rows.
    AfterExternalForces,
    /// Joint rows have been rebuilt from the actual current dynamics cache.
    AfterJointUpdate,
    /// Contact rows have updated their bias and warmstart state.
    AfterContactUpdate,
    /// Contact warmstarting has changed the solver velocities.
    AfterContactWarmstart,
    /// Before a complete ordered joint solve.
    BeforeJointSolve,
    /// Immediately before the selected motor row removes its positional bias.
    BeforeMotorBiasRemoval,
    /// Immediately before the selected motor row is solved.
    BeforeMotorSolve,
    /// Immediately after the same motor row is solved.
    AfterMotorSolve,
    /// After all joint rows, which may include subsequent limit writes.
    AfterJointSolve,
    /// After the complete normal/restitution contact stage.
    AfterContactNormal,
    /// After the complete contact friction stage.
    AfterContactFriction,
    /// After an additional final-slice contact friction stage.
    AfterAdditionalFriction,
    /// After integration and any nonfinal dynamics cache update.
    AfterIntegration,
    /// Before a complete joint stabilization pass.
    BeforeJointStabilization,
    /// After all joint rows in a stabilization pass.
    AfterJointStabilization,
    /// After normal contacts in a stabilization pass.
    AfterContactStabilization,
    /// After the final friction stage following stabilization.
    AfterStabilizationFriction,
    /// Final solver state before the next slice or writeback.
    FinalSlice,
}

/// Native words of one initialized internal single-DoF motor row.
///
/// Scalars are raw IEEE words, with the width given by their event. They retain
/// signed zero, subnormal, nonfinite and bound values without JSON float loss.
/// Linear rows use m/s for RHS and N s for impulse; angular rows use rad/s
/// and N m s. Jacobian, weighted Jacobian and CFM units depend on the full
/// generalized coordinate block and motor gain model. No units are inferred
/// from a scalar word alone.
/// The native accumulator's sign is the negative of applied motor impulse.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct MultibodyMotorStageRow {
    /// Stable child-joint handle, including its generation.
    pub joint_handle: MultibodyJointHandle,
    /// Actual child link index.
    pub link_id: usize,
    /// Actual free joint axis.
    pub axis: JointAxis,
    /// Native gain model from the motor descriptor.
    pub gain_model: MotorModel,
    /// Index in the generic one-body constraint array for this rebuild.
    pub constraint_index: usize,
    /// Actual row `solver_vel2`, the offset into generalized solver velocities.
    /// This is not an ordinary rigid-body solver-array index.
    pub solver_velocity_offset: usize,
    /// Actual initialized row `ndofs2`, the full multibody block length.
    /// A single-DoF joint may belong to a multibody with several generalized DOFs.
    pub ndofs2: usize,
    /// Actual initialized row `j_id2` in the joint generic-Jacobian buffer.
    /// Stored J occupies `j_id2..j_id2 + ndofs2`, followed by weighted J.
    /// This offset does not name a row in the separate contact-Jacobian buffer.
    pub j_id2: usize,
    /// Actual stored J words in coordinate order.
    pub jacobian_words: Vec<u64>,
    /// Actual stored weighted J words; no later mass matrix is substituted.
    pub weighted_jacobian_words: Vec<u64>,
    /// Current native row RHS word.
    pub rhs_word: u64,
    /// Native RHS without positional bias.
    pub rhs_without_bias_word: u64,
    /// Current native CFM gain word.
    pub cfm_gain_word: u64,
    /// Native CFM coefficient word.
    pub cfm_coefficient_word: u64,
    /// Actual rounded reciprocal used by the implementation.
    pub inverse_lhs_word: u64,
    /// Current native impulse accumulator word, before any sign conversion.
    pub impulse_word: u64,
    /// Native lower and upper accumulator bounds, including infinities.
    pub impulse_bound_words: [u64; 2],
    /// Native motor effort ceiling; force or torque according to the axis.
    pub effort_ceiling_word: u64,
}

/// One boundary event in native island, slice and solver-visit order.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct MultibodyMotorStageEvent {
    /// Actual CCD partition index in the completed outer step.
    pub ccd_partition_index: usize,
    /// Actual island index; output drains in island order after workers join.
    pub island_index: usize,
    /// Actual small-step index within this partition and island.
    pub internal_slice_index: usize,
    /// Actual expected small-step count, including additional iterations.
    pub expected_internal_slice_count: usize,
    /// PGS, stabilization or additional-friction visit ordinal, when applicable.
    pub iteration_index: Option<usize>,
    /// Actual joint-constraint visit ordinal for an immediate motor pair.
    pub joint_visit_index: Option<usize>,
    /// The observed operation boundary.
    pub stage: MultibodyMotorStage,
    /// IEEE width of every scalar word, 32 or 64 bits.
    pub native_scalar_bits: u8,
    /// Raw native small-step duration word, in seconds.
    pub duration_word: u64,
    /// Slice whose joint update initialized the observed row cache.
    /// None means row storage was not inspected; Some may have no supported rows.
    pub row_cache_slice_index: Option<usize>,
    /// Actual generalized solver velocity words, in native block order.
    /// Ordinary rigid-body solver storage is outside this vector's scope.
    pub generalized_velocity_words: Vec<u64>,
    /// Initialized supported motor rows. An empty list is not measured zero.
    pub rows: Vec<MultibodyMotorStageRow>,
}

/// Read-only stage data from the most recently completed pipeline step.
///
/// Disabled by default. Records contain no computed residual or inferred force.
/// Consumers must qualify cache identity, bounds, units and floating-point mode
/// before interpreting contact-only deltas. Private pipeline caches and
/// individual contact visit columns are outside this observation's scope.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct MultibodyMotorStageTelemetry {
    /// Raw native outer input duration word, in seconds.
    pub numerical_outer_interval_word: u64,
    /// IEEE width of native scalar words.
    pub native_scalar_bits: u8,
    /// Actual ordered CCD partition durations as raw native words.
    pub ccd_partition_duration_words: Vec<u64>,
    /// Events in partition, island and native solver-visit order.
    pub events: Vec<MultibodyMotorStageEvent>,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct InternalMotorStageBinding {
    pub joint_handle: MultibodyJointHandle,
    pub link_id: usize,
    pub axis: JointAxis,
    pub gain_model: MotorModel,
    pub constraint_index: usize,
    pub effort_ceiling: Real,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct InternalMotorStageContext {
    pub partition: usize,
    pub island: usize,
    pub slice: usize,
    pub expected_slices: usize,
    pub iteration: Option<usize>,
    pub stage: MultibodyMotorStage,
    pub duration: Real,
    pub initialized: bool,
}

pub(crate) fn motor_stage_word(value: Real) -> u64 {
    value.to_bits() as u64
}

impl MultibodyMotorStageTelemetry {
    pub(crate) fn new(duration: Real) -> Self {
        Self {
            numerical_outer_interval_word: motor_stage_word(duration),
            native_scalar_bits: (std::mem::size_of::<Real>() * 8) as u8,
            ccd_partition_duration_words: Vec::new(),
            events: Vec::new(),
        }
    }
}

pub(crate) fn motor_stage_event(
    context: InternalMotorStageContext,
    visit: Option<usize>,
    velocities: &DVector<Real>,
    rows: Vec<MultibodyMotorStageRow>,
) -> MultibodyMotorStageEvent {
    MultibodyMotorStageEvent {
        ccd_partition_index: context.partition,
        island_index: context.island,
        internal_slice_index: context.slice,
        expected_internal_slice_count: context.expected_slices,
        iteration_index: context.iteration,
        joint_visit_index: visit,
        stage: context.stage,
        native_scalar_bits: (std::mem::size_of::<Real>() * 8) as u8,
        duration_word: motor_stage_word(context.duration),
        row_cache_slice_index: context.initialized.then_some(context.slice),
        generalized_velocity_words: velocities
            .iter()
            .map(|value| motor_stage_word(*value))
            .collect(),
        rows,
    }
}

#[cfg(all(test, feature = "dim3"))]
mod tests;
