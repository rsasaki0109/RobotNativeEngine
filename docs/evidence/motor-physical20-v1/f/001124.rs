#![allow(missing_docs)] // For downcast.

use crate::dynamics::joint::MultibodyLink;
use crate::dynamics::solver::{JointGenericOneBodyConstraint, MotorParameters, WritebackId};
use crate::dynamics::{IntegrationParameters, JointMotor, Multibody, MultibodyJointHandle};
use crate::math::Real;
use crate::pipeline::{
    motor_stage_word, InternalMotorConstructionRowRecorder, InternalMotorStageBinding,
    MultibodyMotorConstructionRow,
};
use na::DVector;

/// Initializes and generate the velocity constraints applicable to the multibody links attached
/// to this multibody_joint.
pub fn unit_joint_limit_constraint(
    params: &IntegrationParameters,
    multibody: &Multibody,
    link: &MultibodyLink,
    limits: [Real; 2],
    curr_pos: Real,
    dof_id: usize,
    j_id: &mut usize,
    jacobians: &mut DVector<Real>,
    constraints: &mut [JointGenericOneBodyConstraint],
    insert_at: &mut usize,
) {
    let ndofs = multibody.ndofs();
    let min_enabled = curr_pos < limits[0];
    let max_enabled = limits[1] < curr_pos;
    let erp_inv_dt = params.joint_erp_inv_dt();
    let cfm_coeff = params.joint_cfm_coeff();
    let rhs_bias = ((curr_pos - limits[1]).max(0.0) - (limits[0] - curr_pos).max(0.0)) * erp_inv_dt;
    let rhs_wo_bias = 0.0;

    let dof_j_id = *j_id + dof_id + link.assembly_id;
    jacobians.rows_mut(*j_id, ndofs * 2).fill(0.0);
    jacobians[dof_j_id] = 1.0;
    jacobians[dof_j_id + ndofs] = 1.0;
    multibody
        .inv_augmented_mass()
        .solve_mut(&mut jacobians.rows_mut(*j_id + ndofs, ndofs));

    let lhs = jacobians[dof_j_id + ndofs]; // = J^t * M^-1 J
    let impulse_bounds = [
        min_enabled as u32 as Real * -Real::MAX,
        max_enabled as u32 as Real * Real::MAX,
    ];

    let constraint = JointGenericOneBodyConstraint {
        solver_vel2: multibody.solver_id,
        ndofs2: ndofs,
        j_id2: *j_id,
        joint_id: usize::MAX,
        impulse: 0.0,
        impulse_bounds,
        inv_lhs: crate::utils::inv(lhs),
        rhs: rhs_wo_bias + rhs_bias,
        rhs_wo_bias,
        cfm_coeff,
        cfm_gain: 0.0,
        writeback_id: WritebackId::Limit(dof_id),
    };

    constraints[*insert_at] = constraint;
    *insert_at += 1;

    *j_id += 2 * ndofs;
}

/// Initializes and generate the velocity constraints applicable to the multibody links attached
/// to this multibody_joint.
pub fn unit_joint_motor_constraint(
    params: &IntegrationParameters,
    multibody: &Multibody,
    link: &MultibodyLink,
    motor: &JointMotor,
    curr_pos: Real,
    limits: Option<[Real; 2]>,
    dof_id: usize,
    j_id: &mut usize,
    jacobians: &mut DVector<Real>,
    constraints: &mut [JointGenericOneBodyConstraint],
    insert_at: &mut usize,
) {
    unit_joint_motor_constraint_with_construction_inputs(
        params,
        multibody,
        link,
        motor,
        curr_pos,
        limits,
        dof_id,
        j_id,
        jacobians,
        constraints,
        insert_at,
        None,
    );
}

pub(crate) fn unit_joint_motor_constraint_with_construction_inputs(
    params: &IntegrationParameters,
    multibody: &Multibody,
    link: &MultibodyLink,
    motor: &JointMotor,
    curr_pos: Real,
    limits: Option<[Real; 2]>,
    dof_id: usize,
    j_id: &mut usize,
    jacobians: &mut DVector<Real>,
    constraints: &mut [JointGenericOneBodyConstraint],
    insert_at: &mut usize,
    recorder: Option<InternalMotorConstructionRowRecorder<'_>>,
) {
    let inv_dt = params.inv_dt();
    let ndofs = multibody.ndofs();
    let motor_params = motor.motor_params(params.dt);

    let dof_j_id = *j_id + dof_id + link.assembly_id;
    jacobians.rows_mut(*j_id, ndofs * 2).fill(0.0);
    jacobians[dof_j_id] = 1.0;
    jacobians[dof_j_id + ndofs] = 1.0;
    multibody
        .inv_augmented_mass()
        .solve_mut(&mut jacobians.rows_mut(*j_id + ndofs, ndofs));

    let lhs = jacobians[dof_j_id + ndofs]; // = J^t * M^-1 J
    let impulse_bounds = [-motor_params.max_impulse, motor_params.max_impulse];

    let mut rhs_wo_bias = 0.0;
    if motor_params.erp_inv_dt != 0.0 {
        rhs_wo_bias += (curr_pos - motor_params.target_pos) * motor_params.erp_inv_dt;
    }

    let mut target_vel = motor_params.target_vel;
    if let Some(limits) = limits {
        target_vel = target_vel.clamp(
            (limits[0] - curr_pos) * inv_dt,
            (limits[1] - curr_pos) * inv_dt,
        );
    };

    rhs_wo_bias += -target_vel;

    let constraint = JointGenericOneBodyConstraint {
        solver_vel2: multibody.solver_id,
        ndofs2: ndofs,
        j_id2: *j_id,
        joint_id: usize::MAX,
        impulse: 0.0,
        impulse_bounds,
        cfm_coeff: motor_params.cfm_coeff,
        cfm_gain: motor_params.cfm_gain,
        inv_lhs: crate::utils::inv(lhs),
        rhs: rhs_wo_bias,
        rhs_wo_bias,
        // Metadata only: MAX still prevents internal impulse writeback.
        writeback_id: WritebackId::Motor(dof_id),
    };

    if let Some(recorder) = recorder {
        recorder.capture(
            crate::dynamics::MultibodyJointHandle(link.rigid_body_handle().0),
            motor,
            &motor_params,
            curr_pos,
            params.dt,
            inv_dt,
            limits,
            target_vel,
            &constraint,
            jacobians,
        );
    }

    constraints[*insert_at] = constraint;
    *insert_at += 1;

    *j_id += 2 * ndofs;
}
impl InternalMotorConstructionRowRecorder<'_> {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn capture(
        self,
        joint_handle: MultibodyJointHandle,
        motor: &JointMotor,
        motor_params: &MotorParameters<Real>,
        coordinate: Real,
        duration: Real,
        inverse_duration: Real,
        limits: Option<[Real; 2]>,
        effective_target_velocity: Real,
        constraint: &JointGenericOneBodyConstraint,
        jacobians: &DVector<Real>,
    ) {
        let row = InternalMotorStageBinding {
            joint_handle,
            link_id: self.link_id,
            axis: self.axis,
            gain_model: motor.model,
            constraint_index: self.constraint_index,
            // Same descriptor used by motor.motor_params in the constructor.
            effort_ceiling: motor.max_force,
        }
        .snapshot(constraint, jacobians);
        self.rows.push(MultibodyMotorConstructionRow {
            row,
            coordinate_word: motor_stage_word(coordinate),
            target_position_word: motor_stage_word(motor.target_pos),
            target_velocity_word: motor_stage_word(motor.target_vel),
            stiffness_word: motor_stage_word(motor.stiffness),
            damping_word: motor_stage_word(motor.damping),
            duration_word: motor_stage_word(duration),
            inverse_duration_word: motor_stage_word(inverse_duration),
            limits_words: limits.map(|values| values.map(motor_stage_word)),
            effective_target_velocity_word: motor_stage_word(effective_target_velocity),
            erp_inverse_duration_word: motor_stage_word(motor_params.erp_inv_dt),
        });
    }
}

#[cfg(test)]
mod rne_motor_construction_tests;
