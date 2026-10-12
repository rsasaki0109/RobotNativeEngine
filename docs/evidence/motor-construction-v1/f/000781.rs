use crate::dynamics::{JointAxis, MultibodyJointHandle};
use crate::math::Real;
use std::collections::BTreeSet;

/// A signed generalized motor impulse and its native solver capacity.
///
/// Slice values preserve the actual finite native values promoted to f64.
/// Aggregate values sum impulse in slice order and retain a ceiling or bound
/// only when it was finite and bit-identical throughout the observed slices.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MultibodyMotorImpulseValue {
    /// A revolute motor's impulse, positive in its free joint coordinate.
    Revolute {
        /// Applied generalized angular impulse in N m s.
        applied_impulse_nm_s: f64,
        /// Finite native torque ceiling in N m, if known and constant.
        effective_effort_ceiling_nm: Option<f64>,
        /// Finite native impulse bound in N m s, if known and constant.
        rounded_impulse_bound_nm_s: Option<f64>,
    },
    /// A prismatic motor's impulse, positive in its free joint coordinate.
    Prismatic {
        /// Applied generalized linear impulse in N s.
        applied_impulse_n_s: f64,
        /// Finite native force ceiling in N, if known and constant.
        effective_force_ceiling_n: Option<f64>,
        /// Finite native impulse bound in N s, if known and constant.
        rounded_impulse_bound_n_s: Option<f64>,
    },
}

/// One final net internal-motor impulse after all solves of a small step.
#[derive(Clone, Debug, PartialEq)]
pub struct MultibodyMotorImpulseSlice {
    /// Actual CCD partition ordinal within the completed pipeline step.
    pub ccd_partition_index: usize,
    /// Actual small-step ordinal within this motor's island and partition.
    pub internal_slice_index: usize,
    /// Actual island small-step count, including additional iterations.
    pub expected_internal_slice_count: usize,
    /// Native small-step duration promoted to f64, in seconds.
    pub duration_s: f64,
    /// Signed applied motor impulse and actual rounded capacity.
    pub impulse: MultibodyMotorImpulseValue,
    /// Whether the native impulse bound is finite and strictly positive.
    pub finite_positive_bound_eligible: bool,
    /// Whether the final net impulse exactly equals an eligible native bound.
    /// This does not report attempted clipping or target limiting.
    pub finite_positive_bound_active: bool,
}

/// Completed-step observations of one internal single-DoF multibody motor.
#[derive(Clone, Debug, PartialEq)]
pub struct MultibodyMotorImpulseRecord {
    /// Stable handle of the child link's joint, including its generation.
    pub joint_handle: MultibodyJointHandle,
    /// Link index in the multibody observed by the solver.
    pub link_id: usize,
    /// Free coordinate index within this joint, distinct from the axis index.
    pub joint_dof_id: usize,
    /// Actual free linear or angular joint axis.
    pub axis: JointAxis,
    /// Signed net impulse summed in actual slice order.
    pub impulse: MultibodyMotorImpulseValue,
    /// Sum of observed native small-step durations, in seconds.
    /// Divide impulse by this value for an observed-interval mean effort.
    pub observed_solver_interval_s: f64,
    /// Number of qualified observed small steps, not PGS iterations.
    pub observed_solver_slice_count: usize,
    /// Exact-once coverage of every expected small step in every CCD partition.
    /// Float equality of nominal and observed durations is not used.
    pub full_step_coverage: bool,
    /// Number of distinct actual CCD partitions with a qualified sample.
    pub observed_partition_count: usize,
    /// Number of slices with finite strictly positive native impulse capacity.
    pub finite_positive_bound_eligible_slice_count: usize,
    /// Total duration of eligible small steps, in seconds.
    pub finite_positive_bound_eligible_duration_s: f64,
    /// Number of eligible slices whose final impulse is exactly at a bound.
    pub finite_positive_bound_active_slice_count: usize,
    /// Total duration of final bound-active small steps, in seconds.
    pub finite_positive_bound_active_duration_s: f64,
    /// Actual ordered slices, including rounded bounds and coverage ordinals.
    pub slices: Vec<MultibodyMotorImpulseSlice>,
}

/// Read-only observations from the most recently completed pipeline step.
///
/// Only solved internal single-DoF motors are included. Absence is not a
/// measured zero: an inactive, sleeping, missing, unsupported, or unqualified
/// motor has no record. A solved zero-capacity motor can have a zero record.
#[derive(Clone, Debug, PartialEq)]
pub struct MultibodyMotorImpulseTelemetry {
    /// Native outer input dt promoted to f64, separate from observed duration.
    pub numerical_outer_interval_s: f64,
    /// Actual ordered native CCD partition durations promoted to f64.
    pub ccd_partition_durations_s: Vec<f64>,
    /// Records sorted by stable joint handle, free coordinate, and axis.
    /// Impulses and durations are accumulated before this output sorting.
    pub motors: Vec<MultibodyMotorImpulseRecord>,
}

pub(crate) struct InternalMotorImpulseSample {
    pub joint_handle: MultibodyJointHandle,
    pub link_id: usize,
    pub joint_dof_id: usize,
    pub axis: JointAxis,
    pub slice: MultibodyMotorImpulseSlice,
}

fn retain_identical(first: Option<f64>, next: Option<f64>) -> Option<f64> {
    match (first, next) {
        (Some(a), Some(b)) if a.to_bits() == b.to_bits() => Some(a),
        _ => None,
    }
}

impl MultibodyMotorImpulseValue {
    pub(crate) fn from_solver(
        axis: JointAxis,
        solver_impulse: Real,
        ceiling: Real,
        bound: Real,
    ) -> Self {
        let impulse = -f64::from(solver_impulse);
        let ceiling = ceiling.is_finite().then(|| f64::from(ceiling));
        let bound = bound.is_finite().then(|| f64::from(bound));
        if (axis as usize) < crate::math::DIM {
            Self::Prismatic {
                applied_impulse_n_s: impulse,
                effective_force_ceiling_n: ceiling,
                rounded_impulse_bound_n_s: bound,
            }
        } else {
            Self::Revolute {
                applied_impulse_nm_s: impulse,
                effective_effort_ceiling_nm: ceiling,
                rounded_impulse_bound_nm_s: bound,
            }
        }
    }

    fn add(&mut self, next: Self) {
        match (self, next) {
            (
                Self::Revolute {
                    applied_impulse_nm_s: impulse,
                    effective_effort_ceiling_nm: ceiling,
                    rounded_impulse_bound_nm_s: bound,
                },
                Self::Revolute {
                    applied_impulse_nm_s,
                    effective_effort_ceiling_nm,
                    rounded_impulse_bound_nm_s,
                },
            ) => {
                *impulse += applied_impulse_nm_s;
                *ceiling = retain_identical(*ceiling, effective_effort_ceiling_nm);
                *bound = retain_identical(*bound, rounded_impulse_bound_nm_s);
            }
            (
                Self::Prismatic {
                    applied_impulse_n_s: impulse,
                    effective_force_ceiling_n: ceiling,
                    rounded_impulse_bound_n_s: bound,
                },
                Self::Prismatic {
                    applied_impulse_n_s,
                    effective_force_ceiling_n,
                    rounded_impulse_bound_n_s,
                },
            ) => {
                *impulse += applied_impulse_n_s;
                *ceiling = retain_identical(*ceiling, effective_force_ceiling_n);
                *bound = retain_identical(*bound, rounded_impulse_bound_n_s);
            }
            _ => unreachable!("a stable internal motor cannot change its free axis"),
        }
    }
}

impl MultibodyMotorImpulseRecord {
    fn append(&mut self, slice: MultibodyMotorImpulseSlice) {
        self.impulse.add(slice.impulse);
        self.observed_solver_interval_s += slice.duration_s;
        self.observed_solver_slice_count += 1;
        if slice.finite_positive_bound_eligible {
            self.finite_positive_bound_eligible_slice_count += 1;
            self.finite_positive_bound_eligible_duration_s += slice.duration_s;
        }
        if slice.finite_positive_bound_active {
            self.finite_positive_bound_active_slice_count += 1;
            self.finite_positive_bound_active_duration_s += slice.duration_s;
        }
        self.slices.push(slice);
    }

    fn finalize_coverage(&mut self, partition_count: usize) {
        self.observed_partition_count = self
            .slices
            .iter()
            .map(|slice| slice.ccd_partition_index)
            .collect::<BTreeSet<_>>()
            .len();
        self.full_step_coverage = false;
        let mut cursor = 0;
        for partition in 0..partition_count {
            let Some(first) = self.slices.get(cursor) else {
                return;
            };
            let expected = first.expected_internal_slice_count;
            if first.ccd_partition_index != partition || expected == 0 {
                return;
            }
            for ordinal in 0..expected {
                let Some(slice) = self.slices.get(cursor) else {
                    return;
                };
                if slice.ccd_partition_index != partition
                    || slice.expected_internal_slice_count != expected
                    || slice.internal_slice_index != ordinal
                {
                    return;
                }
                cursor += 1;
            }
        }
        self.full_step_coverage = partition_count > 0 && cursor == self.slices.len();
    }
}

impl MultibodyMotorImpulseTelemetry {
    pub(crate) fn new(outer_dt: Real) -> Self {
        Self {
            numerical_outer_interval_s: f64::from(outer_dt),
            ccd_partition_durations_s: Vec::new(),
            motors: Vec::new(),
        }
    }

    pub(crate) fn append(&mut self, samples: Vec<InternalMotorImpulseSample>) {
        for sample in samples {
            if let Some(record) = self.motors.iter_mut().find(|record| {
                record.joint_handle == sample.joint_handle
                    && record.joint_dof_id == sample.joint_dof_id
                    && record.axis == sample.axis
            }) {
                record.append(sample.slice);
            } else {
                let mut record = MultibodyMotorImpulseRecord {
                    joint_handle: sample.joint_handle,
                    link_id: sample.link_id,
                    joint_dof_id: sample.joint_dof_id,
                    axis: sample.axis,
                    impulse: sample.slice.impulse,
                    observed_solver_interval_s: sample.slice.duration_s,
                    observed_solver_slice_count: 1,
                    full_step_coverage: false,
                    observed_partition_count: 0,
                    finite_positive_bound_eligible_slice_count: 0,
                    finite_positive_bound_eligible_duration_s: 0.0,
                    finite_positive_bound_active_slice_count: 0,
                    finite_positive_bound_active_duration_s: 0.0,
                    slices: Vec::new(),
                };
                if sample.slice.finite_positive_bound_eligible {
                    record.finite_positive_bound_eligible_slice_count = 1;
                    record.finite_positive_bound_eligible_duration_s = sample.slice.duration_s;
                }
                if sample.slice.finite_positive_bound_active {
                    record.finite_positive_bound_active_slice_count = 1;
                    record.finite_positive_bound_active_duration_s = sample.slice.duration_s;
                }
                record.slices.push(sample.slice);
                self.motors.push(record);
            }
        }
    }

    pub(crate) fn finalize(&mut self) {
        for record in &mut self.motors {
            record.finalize_coverage(self.ccd_partition_durations_s.len());
        }
        self.motors.sort_by_key(|record| {
            (
                record.joint_handle.into_raw_parts(),
                record.joint_dof_id,
                record.axis as usize,
            )
        });
    }
}

#[cfg(all(test, feature = "dim3"))]
mod tests;
