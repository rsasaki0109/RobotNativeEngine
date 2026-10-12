use crate::dynamics::solver::categorization::categorize_joints;
use crate::dynamics::solver::solver_body::SolverBody;
use crate::dynamics::solver::solver_vel::SolverVel;
use crate::dynamics::solver::{
    reset_buffer, JointConstraintTypes, JointGenericOneBodyConstraint,
    JointGenericOneBodyConstraintBuilder, JointGenericTwoBodyConstraint,
    JointGenericTwoBodyConstraintBuilder, JointGenericVelocityOneBodyExternalConstraintBuilder,
    JointGenericVelocityOneBodyInternalConstraintBuilder, SolverConstraintsSet,
};
use crate::dynamics::{
    IntegrationParameters, IslandManager, JointGraphEdge, JointIndex, MultibodyJointSet,
    RigidBodySet,
};
use crate::pipeline::MultibodyMotorConstructionRow;
use crate::pipeline::{
    motor_stage_event, motor_stage_word, InternalMotorStageBinding, InternalMotorStageContext,
    MultibodyMotorStage, MultibodyMotorStageEvent, MultibodyMotorStageRow,
};
use na::DVector;
use parry::math::Real;

use crate::dynamics::solver::joint_constraint::joint_constraint_builder::{
    JointOneBodyConstraintBuilder, JointTwoBodyConstraintBuilder,
};
#[cfg(feature = "simd-is-enabled")]
use {
    crate::dynamics::solver::joint_constraint::joint_constraint_builder::{
        JointOneBodyConstraintBuilderSimd, JointTwoBodyConstraintBuilderSimd,
    },
    crate::math::SIMD_WIDTH,
};

pub type JointConstraintsSet = SolverConstraintsSet<JointConstraintTypes>;

impl JointConstraintsSet {
    pub fn init(
        &mut self,
        island_id: usize,
        islands: &IslandManager,
        bodies: &RigidBodySet,
        multibody_joints: &MultibodyJointSet,
        impulse_joints: &[JointGraphEdge],
        joint_constraint_indices: &[JointIndex],
    ) {
        // Generate constraints for impulse_joints.
        self.two_body_interactions.clear();
        self.one_body_interactions.clear();
        self.generic_two_body_interactions.clear();
        self.generic_one_body_interactions.clear();

        categorize_joints(
            bodies,
            multibody_joints,
            impulse_joints,
            joint_constraint_indices,
            &mut self.one_body_interactions,
            &mut self.two_body_interactions,
            &mut self.generic_one_body_interactions,
            &mut self.generic_two_body_interactions,
        );

        self.clear_constraints();
        self.clear_builders();

        self.interaction_groups.clear_groups();
        self.interaction_groups.group_joints(
            island_id,
            islands,
            bodies,
            impulse_joints,
            &self.two_body_interactions,
        );

        self.one_body_interaction_groups.clear_groups();
        self.one_body_interaction_groups.group_joints(
            island_id,
            islands,
            bodies,
            impulse_joints,
            &self.one_body_interactions,
        );
        // NOTE: uncomment this do disable SIMD joint resolution.
        // self.interaction_groups
        //     .nongrouped_interactions
        //     .append(&mut self.interaction_groups.simd_interactions);
        // self.one_body_interaction_groups
        //     .nongrouped_interactions
        //     .append(&mut self.one_body_interaction_groups.simd_interactions);

        let mut j_id = 0;
        self.compute_joint_constraints(bodies, impulse_joints);
        #[cfg(feature = "simd-is-enabled")]
        {
            self.simd_compute_joint_constraints(bodies, impulse_joints);
        }
        self.compute_generic_joint_constraints(bodies, multibody_joints, impulse_joints, &mut j_id);

        self.compute_joint_one_body_constraints(bodies, impulse_joints);
        #[cfg(feature = "simd-is-enabled")]
        {
            self.simd_compute_joint_one_body_constraints(bodies, impulse_joints);
        }
        self.compute_generic_one_body_joint_constraints(
            island_id,
            islands,
            bodies,
            multibody_joints,
            impulse_joints,
            &mut j_id,
        );
    }

    fn compute_joint_one_body_constraints(
        &mut self,
        bodies: &RigidBodySet,
        joints_all: &[JointGraphEdge],
    ) {
        let total_num_builders = self
            .one_body_interaction_groups
            .nongrouped_interactions
            .len();

        unsafe {
            reset_buffer(
                &mut self.velocity_one_body_constraints_builder,
                total_num_builders,
            );
        }

        let mut num_constraints = 0;
        for (joint_i, builder) in self
            .one_body_interaction_groups
            .nongrouped_interactions
            .iter()
            .zip(self.velocity_one_body_constraints_builder.iter_mut())
        {
            let joint = &joints_all[*joint_i].weight;
            JointOneBodyConstraintBuilder::generate(
                joint,
                bodies,
                *joint_i,
                builder,
                &mut num_constraints,
            );
        }

        unsafe {
            reset_buffer(&mut self.velocity_one_body_constraints, num_constraints);
        }
    }

    #[cfg(feature = "simd-is-enabled")]
    fn simd_compute_joint_one_body_constraints(
        &mut self,
        bodies: &RigidBodySet,
        joints_all: &[JointGraphEdge],
    ) {
        let total_num_builders =
            self.one_body_interaction_groups.simd_interactions.len() / SIMD_WIDTH;

        unsafe {
            reset_buffer(
                &mut self.simd_velocity_one_body_constraints_builder,
                total_num_builders,
            );
        }

        let mut num_constraints = 0;
        for (joints_i, builder) in self
            .one_body_interaction_groups
            .simd_interactions
            .chunks_exact(SIMD_WIDTH)
            .zip(self.simd_velocity_one_body_constraints_builder.iter_mut())
        {
            let joints_id = gather![|ii| joints_i[ii]];
            let impulse_joints = gather![|ii| &joints_all[joints_i[ii]].weight];
            JointOneBodyConstraintBuilderSimd::generate(
                impulse_joints,
                bodies,
                joints_id,
                builder,
                &mut num_constraints,
            );
        }

        unsafe {
            reset_buffer(
                &mut self.simd_velocity_one_body_constraints,
                num_constraints,
            );
        }
    }

    fn compute_joint_constraints(&mut self, bodies: &RigidBodySet, joints_all: &[JointGraphEdge]) {
        let total_num_builders = self.interaction_groups.nongrouped_interactions.len();

        unsafe {
            reset_buffer(&mut self.velocity_constraints_builder, total_num_builders);
        }

        let mut num_constraints = 0;
        for (joint_i, builder) in self
            .interaction_groups
            .nongrouped_interactions
            .iter()
            .zip(self.velocity_constraints_builder.iter_mut())
        {
            let joint = &joints_all[*joint_i].weight;
            JointTwoBodyConstraintBuilder::generate(
                joint,
                bodies,
                *joint_i,
                builder,
                &mut num_constraints,
            );
        }

        unsafe {
            reset_buffer(&mut self.velocity_constraints, num_constraints);
        }
    }

    fn compute_generic_joint_constraints(
        &mut self,
        bodies: &RigidBodySet,
        multibodies: &MultibodyJointSet,
        joints_all: &[JointGraphEdge],
        j_id: &mut usize,
    ) {
        let total_num_builders = self.generic_two_body_interactions.len();
        self.generic_velocity_constraints_builder.resize(
            total_num_builders,
            JointGenericTwoBodyConstraintBuilder::invalid(),
        );

        let mut num_constraints = 0;
        for (joint_i, builder) in self
            .generic_two_body_interactions
            .iter()
            .zip(self.generic_velocity_constraints_builder.iter_mut())
        {
            let joint = &joints_all[*joint_i].weight;
            JointGenericTwoBodyConstraintBuilder::generate(
                *joint_i,
                joint,
                bodies,
                multibodies,
                builder,
                j_id,
                &mut self.generic_jacobians,
                &mut num_constraints,
            );
        }

        self.generic_velocity_constraints
            .resize(num_constraints, JointGenericTwoBodyConstraint::invalid());
    }

    fn compute_generic_one_body_joint_constraints(
        &mut self,
        island_id: usize,
        islands: &IslandManager,
        bodies: &RigidBodySet,
        multibodies: &MultibodyJointSet,
        joints_all: &[JointGraphEdge],
        j_id: &mut usize,
    ) {
        let mut total_num_builders = self.generic_one_body_interactions.len();

        for handle in islands.active_island(island_id) {
            if let Some(link_id) = multibodies.rigid_body_link(*handle) {
                if JointGenericVelocityOneBodyInternalConstraintBuilder::num_constraints(
                    multibodies,
                    link_id,
                ) > 0
                {
                    total_num_builders += 1;
                }
            }
        }

        self.generic_velocity_one_body_constraints_builder.resize(
            total_num_builders,
            JointGenericOneBodyConstraintBuilder::invalid(),
        );

        let mut num_constraints = 0;
        for (joint_i, builder) in self.generic_one_body_interactions.iter().zip(
            self.generic_velocity_one_body_constraints_builder
                .iter_mut(),
        ) {
            let joint = &joints_all[*joint_i].weight;
            JointGenericVelocityOneBodyExternalConstraintBuilder::generate(
                *joint_i,
                joint,
                bodies,
                multibodies,
                builder,
                j_id,
                &mut self.generic_jacobians,
                &mut num_constraints,
            );
        }

        let mut curr_builder = self.generic_one_body_interactions.len();
        for handle in islands.active_island(island_id) {
            if curr_builder >= self.generic_velocity_one_body_constraints_builder.len() {
                break; // No more builder need to be generated.
            }

            if let Some(link_id) = multibodies.rigid_body_link(*handle) {
                let prev_num_constraints = num_constraints;
                JointGenericVelocityOneBodyInternalConstraintBuilder::generate(
                    multibodies,
                    link_id,
                    &mut self.generic_velocity_one_body_constraints_builder[curr_builder],
                    j_id,
                    &mut self.generic_jacobians,
                    &mut num_constraints,
                );
                if num_constraints != prev_num_constraints {
                    curr_builder += 1;
                }
            }
        }

        self.generic_velocity_one_body_constraints
            .resize(num_constraints, JointGenericOneBodyConstraint::invalid());
    }

    #[cfg(feature = "simd-is-enabled")]
    fn simd_compute_joint_constraints(
        &mut self,
        bodies: &RigidBodySet,
        joints_all: &[JointGraphEdge],
    ) {
        let total_num_builders = self.interaction_groups.simd_interactions.len() / SIMD_WIDTH;

        unsafe {
            reset_buffer(
                &mut self.simd_velocity_constraints_builder,
                total_num_builders,
            );
        }

        let mut num_constraints = 0;
        for (joints_i, builder) in self
            .interaction_groups
            .simd_interactions
            .chunks_exact(SIMD_WIDTH)
            .zip(self.simd_velocity_constraints_builder.iter_mut())
        {
            let joints_id = gather![|ii| joints_i[ii]];
            let impulse_joints = gather![|ii| &joints_all[joints_i[ii]].weight];
            JointTwoBodyConstraintBuilderSimd::generate(
                impulse_joints,
                bodies,
                joints_id,
                builder,
                &mut num_constraints,
            );
        }

        unsafe {
            reset_buffer(&mut self.simd_velocity_constraints, num_constraints);
        }
    }

    pub fn solve(
        &mut self,
        solver_vels: &mut [SolverVel<Real>],
        generic_solver_vels: &mut DVector<Real>,
    ) {
        let (jac, constraints) = self.iter_constraints_mut();
        for mut c in constraints {
            c.solve(jac, solver_vels, generic_solver_vels);
        }
    }

    pub fn solve_wo_bias(
        &mut self,
        solver_vels: &mut [SolverVel<Real>],
        generic_solver_vels: &mut DVector<Real>,
    ) {
        let (jac, constraints) = self.iter_constraints_mut();
        for mut c in constraints {
            c.remove_bias();
            c.solve(jac, solver_vels, generic_solver_vels);
        }
    }

    pub(crate) fn solve_with_motor_stages(
        &mut self,
        solver_vels: &mut [SolverVel<Real>],
        generic_solver_vels: &mut DVector<Real>,
        bindings: &[InternalMotorStageBinding],
        context: InternalMotorStageContext,
        events: &mut Vec<MultibodyMotorStageEvent>,
        remove_bias: bool,
    ) {
        // Use the exact same constraint iterator and solve calls as the disabled
        // path. Inspect only initialized Internal-builder motor rows.
        let (jac, constraints) = self.iter_constraints_mut();
        let mut one_body_index = 0;
        for (visit, mut c) in constraints.enumerate() {
            let binding = if matches!(
                c,
                crate::dynamics::solver::AnyConstraintMut::GenericOneBody(_)
            ) {
                let binding = bindings
                    .iter()
                    .find(|binding| binding.constraint_index == one_body_index);
                one_body_index += 1;
                binding
            } else {
                None
            };
            let observe = |constraint: &crate::dynamics::solver::AnyConstraintMut<
                '_,
                JointConstraintTypes,
            >,
                           stage,
                           velocities: &DVector<Real>,
                           events: &mut Vec<MultibodyMotorStageEvent>| {
                if let (
                    Some(binding),
                    crate::dynamics::solver::AnyConstraintMut::GenericOneBody(row),
                ) = (binding, constraint)
                {
                    events.push(motor_stage_event(
                        InternalMotorStageContext { stage, ..context },
                        Some(visit),
                        velocities,
                        vec![binding.snapshot(row, jac)],
                    ));
                }
            };
            if remove_bias {
                observe(
                    &c,
                    MultibodyMotorStage::BeforeMotorBiasRemoval,
                    generic_solver_vels,
                    events,
                );
                c.remove_bias();
            }
            observe(
                &c,
                MultibodyMotorStage::BeforeMotorSolve,
                generic_solver_vels,
                events,
            );
            c.solve(jac, solver_vels, generic_solver_vels);
            observe(
                &c,
                MultibodyMotorStage::AfterMotorSolve,
                generic_solver_vels,
                events,
            );
        }
    }

    pub fn writeback_impulses(&mut self, joints_all: &mut [JointGraphEdge]) {
        let (_, constraints) = self.iter_constraints_mut();
        for mut c in constraints {
            c.writeback_impulses(joints_all);
        }
    }

    pub fn update(
        &mut self,
        params: &IntegrationParameters,
        multibodies: &MultibodyJointSet,
        solver_bodies: &[SolverBody],
    ) {
        self.update_with_construction_inputs(params, multibodies, solver_bodies, None);
    }

    pub(crate) fn update_with_construction_inputs(
        &mut self,
        params: &IntegrationParameters,
        multibodies: &MultibodyJointSet,
        solver_bodies: &[SolverBody],
        mut recorder: Option<&mut Vec<MultibodyMotorConstructionRow>>,
    ) {
        for builder in &mut self.generic_velocity_constraints_builder {
            builder.update(
                params,
                multibodies,
                solver_bodies,
                &mut self.generic_jacobians,
                &mut self.generic_velocity_constraints,
            );
        }

        for builder in &mut self.generic_velocity_one_body_constraints_builder {
            if let Some(recorder) = recorder.as_deref_mut() {
                builder.update_with_construction_inputs(
                    params,
                    multibodies,
                    solver_bodies,
                    &mut self.generic_jacobians,
                    &mut self.generic_velocity_one_body_constraints,
                    Some(recorder),
                );
            } else {
                builder.update(
                    params,
                    multibodies,
                    solver_bodies,
                    &mut self.generic_jacobians,
                    &mut self.generic_velocity_one_body_constraints,
                );
            }
        }

        for builder in &mut self.velocity_constraints_builder {
            builder.update(params, solver_bodies, &mut self.velocity_constraints);
        }

        #[cfg(feature = "simd-is-enabled")]
        for builder in &mut self.simd_velocity_constraints_builder {
            builder.update(params, solver_bodies, &mut self.simd_velocity_constraints);
        }

        for builder in &mut self.velocity_one_body_constraints_builder {
            builder.update(
                params,
                solver_bodies,
                &mut self.velocity_one_body_constraints,
            );
        }

        #[cfg(feature = "simd-is-enabled")]
        for builder in &mut self.simd_velocity_one_body_constraints_builder {
            builder.update(
                params,
                solver_bodies,
                &mut self.simd_velocity_one_body_constraints,
            );
        }
    }
}

impl InternalMotorStageBinding {
    pub(crate) fn snapshot(
        &self,
        row: &JointGenericOneBodyConstraint,
        jacobians: &DVector<Real>,
    ) -> MultibodyMotorStageRow {
        MultibodyMotorStageRow {
            joint_handle: self.joint_handle,
            link_id: self.link_id,
            axis: self.axis,
            gain_model: self.gain_model,
            constraint_index: self.constraint_index,
            solver_velocity_offset: row.solver_vel2,
            ndofs2: row.ndofs2,
            j_id2: row.j_id2,
            jacobian_words: jacobians
                .rows(row.j_id2, row.ndofs2)
                .iter()
                .map(|value| motor_stage_word(*value))
                .collect(),
            weighted_jacobian_words: jacobians
                .rows(row.j_id2 + row.ndofs2, row.ndofs2)
                .iter()
                .map(|value| motor_stage_word(*value))
                .collect(),
            rhs_word: motor_stage_word(row.rhs),
            rhs_without_bias_word: motor_stage_word(row.rhs_wo_bias),
            cfm_gain_word: motor_stage_word(row.cfm_gain),
            cfm_coefficient_word: motor_stage_word(row.cfm_coeff),
            inverse_lhs_word: motor_stage_word(row.inv_lhs),
            impulse_word: motor_stage_word(row.impulse),
            impulse_bound_words: row.impulse_bounds.map(motor_stage_word),
            effort_ceiling_word: motor_stage_word(self.effort_ceiling),
        }
    }
}
