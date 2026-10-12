use super::{JointConstraintTypes, SolverConstraintsSet};
use crate::dynamics::solver::solver_body::SolverBody;
use crate::dynamics::Multibody;
use crate::dynamics::{
    solver::{ContactConstraintTypes, SolverVel},
    IntegrationParameters, IslandManager, JointGraphEdge, JointIndex, MultibodyJointSet,
    MultibodyLinkId, RigidBodySet,
};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use crate::math::Real;
use crate::pipeline::{
    motor_stage_event, InternalMotorImpulseSample, InternalMotorStageBinding,
    InternalMotorStageContext, MultibodyMotorStage, MultibodyMotorStageEvent,
};
use crate::pipeline::{
    motor_stage_word, MultibodyMotorConstructionAvailability, MultibodyMotorConstructionEndpoint,
    MultibodyMotorConstructionEvent, MultibodyMotorConstructionPhase,
    MultibodyMotorConstructionRow,
};
use crate::prelude::RigidBodyVelocity;
use crate::utils::SimdAngularInertia;
use na::DVector;

pub(crate) struct VelocitySolver {
    pub solver_bodies: Vec<SolverBody>,
    pub solver_vels: Vec<SolverVel<Real>>,
    pub solver_vels_increment: Vec<SolverVel<Real>>,
    pub generic_solver_vels: DVector<Real>,
    pub generic_solver_vels_increment: DVector<Real>,
    pub multibody_roots: Vec<MultibodyLinkId>,
    pub(crate) motor_impulse_samples: Option<Vec<InternalMotorImpulseSample>>,
    pub(crate) motor_impulse_partition_index: usize,
    pub(crate) motor_stage_events: Option<Vec<MultibodyMotorStageEvent>>,
    pub(crate) motor_stage_partition_index: usize,
    pub(crate) motor_stage_island_index: usize,
    pub(crate) motor_construction_events: Option<Vec<MultibodyMotorConstructionEvent>>,
    pub(crate) motor_construction_partition_index: usize,
    pub(crate) motor_construction_island_index: usize,
    pub(crate) motor_construction_final_context: Option<InternalMotorStageContext>,
    pub(crate) motor_construction_final_bindings: Vec<InternalMotorStageBinding>,
}

impl VelocitySolver {
    pub fn new() -> Self {
        Self {
            solver_bodies: Vec::new(),
            solver_vels: Vec::new(),
            solver_vels_increment: Vec::new(),
            generic_solver_vels: DVector::zeros(0),
            generic_solver_vels_increment: DVector::zeros(0),
            multibody_roots: Vec::new(),
            motor_impulse_samples: None,
            motor_impulse_partition_index: 0,
            motor_stage_events: None,
            motor_stage_partition_index: 0,
            motor_stage_island_index: 0,
            motor_construction_events: None,
            motor_construction_partition_index: 0,
            motor_construction_island_index: 0,
            motor_construction_final_context: None,
            motor_construction_final_bindings: Vec::new(),
        }
    }

    pub fn init_constraints(
        &self,
        island_id: usize,
        islands: &IslandManager,
        bodies: &mut RigidBodySet,
        multibodies: &mut MultibodyJointSet,
        manifolds_all: &mut [&mut ContactManifold],
        manifold_indices: &[ContactManifoldIndex],
        joints_all: &mut [JointGraphEdge],
        joint_indices: &[JointIndex],
        contact_constraints: &mut SolverConstraintsSet<ContactConstraintTypes>,
        joint_constraints: &mut SolverConstraintsSet<JointConstraintTypes>,
    ) {
        contact_constraints.init(
            island_id,
            islands,
            bodies,
            multibodies,
            manifolds_all,
            manifold_indices,
        );

        joint_constraints.init(
            island_id,
            islands,
            bodies,
            multibodies,
            joints_all,
            joint_indices,
        );
    }

    pub fn init_solver_velocities_and_solver_bodies(
        &mut self,
        params: &IntegrationParameters,
        island_id: usize,
        islands: &IslandManager,
        bodies: &mut RigidBodySet,
        multibodies: &mut MultibodyJointSet,
    ) {
        self.multibody_roots.clear();
        self.solver_bodies.clear();
        self.solver_bodies.resize(
            islands.active_island(island_id).len(),
            SolverBody::default(),
        );

        self.solver_vels_increment.clear();
        self.solver_vels_increment
            .resize(islands.active_island(island_id).len(), SolverVel::zero());
        self.solver_vels.clear();
        self.solver_vels
            .resize(islands.active_island(island_id).len(), SolverVel::zero());

        /*
         * Initialize solver bodies and delta-velocities (`solver_vels_increment`) with external forces (gravity etc):
         * NOTE: we compute this only once by neglecting changes of mass matrices.
         */

        // Assign solver ids to multibodies, and collect the relevant roots.
        // And init solver_vels for rigidb-bodies.
        let mut multibody_solver_id = 0;
        for handle in islands.active_island(island_id) {
            if let Some(link) = multibodies.rigid_body_link(*handle).copied() {
                let multibody = multibodies
                    .get_multibody_mut_internal(link.multibody)
                    .unwrap();

                if link.id == 0 || link.id == 1 && !multibody.root_is_dynamic {
                    multibody.solver_id = multibody_solver_id;
                    multibody_solver_id += multibody.ndofs();
                    self.multibody_roots.push(link);
                }
            } else {
                let rb = &bodies[*handle];
                let solver_vel = &mut self.solver_vels[rb.ids.active_set_offset];
                let solver_vel_incr = &mut self.solver_vels_increment[rb.ids.active_set_offset];
                let solver_body = &mut self.solver_bodies[rb.ids.active_set_offset];
                solver_body.copy_from(rb);

                // NOTE: `dvel.angular` is actually storing angular velocity delta multiplied
                //       by the square root of the inertia tensor:
                solver_vel_incr.angular =
                    rb.mprops.effective_world_inv_inertia_sqrt * rb.forces.torque * params.dt;
                solver_vel_incr.linear =
                    rb.forces.force.component_mul(&rb.mprops.effective_inv_mass) * params.dt;

                solver_vel.linear = rb.vels.linvel;
                // PERF: can we avoid the call to effective_angular_inertia_sqrt?
                solver_vel.angular = rb.mprops.effective_angular_inertia_sqrt() * rb.vels.angvel;
            }
        }

        // PERF: don’t reallocate at each iteration.
        self.generic_solver_vels_increment = DVector::zeros(multibody_solver_id);
        self.generic_solver_vels = DVector::zeros(multibody_solver_id);

        // init solver_vels for multibodies.
        for link in &self.multibody_roots {
            let multibody = multibodies
                .get_multibody_mut_internal(link.multibody)
                .unwrap();
            multibody.update_dynamics(params.dt, bodies);
            multibody.update_acceleration(bodies);

            let mut solver_vels_incr = self
                .generic_solver_vels_increment
                .rows_mut(multibody.solver_id, multibody.ndofs());
            let mut solver_vels = self
                .generic_solver_vels
                .rows_mut(multibody.solver_id, multibody.ndofs());

            solver_vels_incr.axpy(params.dt, &multibody.accelerations, 0.0);
            solver_vels.copy_from(&multibody.velocities);
        }
    }

    fn record_motor_stage(
        &mut self,
        context: InternalMotorStageContext,
        stage: MultibodyMotorStage,
        joint_constraints: &SolverConstraintsSet<JointConstraintTypes>,
        bindings: &[InternalMotorStageBinding],
    ) {
        if let Some(events) = &mut self.motor_stage_events {
            // Before this slice's update the row buffer may be uninitialized,
            // or retain previous-slice data. Neither is inspected here.
            let rows = if context.initialized {
                bindings
                    .iter()
                    .map(|binding| {
                        binding.snapshot(
                            &joint_constraints.generic_velocity_one_body_constraints
                                [binding.constraint_index],
                            &joint_constraints.generic_jacobians,
                        )
                    })
                    .collect()
            } else {
                Vec::new()
            };
            events.push(motor_stage_event(
                InternalMotorStageContext { stage, ..context },
                None,
                &self.generic_solver_vels,
                rows,
            ));
        }
    }

    fn construction_context(
        &self,
        context: InternalMotorStageContext,
    ) -> InternalMotorStageContext {
        InternalMotorStageContext {
            partition: self.motor_construction_partition_index,
            island: self.motor_construction_island_index,
            ..context
        }
    }

    fn construction_endpoints_for_multibody(
        multibody: &Multibody,
        joint_constraints: &SolverConstraintsSet<JointConstraintTypes>,
        bindings: &[InternalMotorStageBinding],
    ) -> Vec<MultibodyMotorConstructionEndpoint> {
        bindings
            .iter()
            .filter_map(|binding| {
                let link = multibody.link(binding.link_id)?;
                if link.rigid_body_handle().0 != binding.joint_handle.0 {
                    return None;
                }
                let row = joint_constraints
                    .generic_velocity_one_body_constraints
                    .get(binding.constraint_index)?;
                Some(MultibodyMotorConstructionEndpoint {
                    row: binding.snapshot(row, &joint_constraints.generic_jacobians),
                    coordinate_word: motor_stage_word(link.joint().coords[binding.axis as usize]),
                    multibody_velocity_words: multibody
                        .velocities
                        .iter()
                        .map(|value| motor_stage_word(*value))
                        .collect(),
                })
            })
            .collect()
    }

    #[allow(clippy::too_many_arguments)]
    fn push_construction_event(
        events: &mut Option<Vec<MultibodyMotorConstructionEvent>>,
        context: InternalMotorStageContext,
        phase: MultibodyMotorConstructionPhase,
        velocities: &DVector<Real>,
        constructed_rows: Vec<MultibodyMotorConstructionRow>,
        endpoints: Vec<MultibodyMotorConstructionEndpoint>,
        integration_handle: Option<[u32; 2]>,
    ) {
        if let Some(events) = events {
            let availability = if !context.initialized {
                MultibodyMotorConstructionAvailability::Uninitialized
            } else if constructed_rows.is_empty() && endpoints.is_empty() {
                MultibodyMotorConstructionAvailability::NoSupportedRows
            } else {
                MultibodyMotorConstructionAvailability::Available
            };
            events.push(MultibodyMotorConstructionEvent {
                ccd_partition_index: context.partition,
                island_index: context.island,
                internal_slice_index: context.slice,
                expected_internal_slice_count: context.expected_slices,
                iteration_index: context.iteration,
                phase,
                native_scalar_bits: (std::mem::size_of::<Real>() * 8) as u8,
                duration_word: motor_stage_word(context.duration),
                row_cache_slice_index: context.initialized.then_some(context.slice),
                availability,
                integration_multibody_handle_words: integration_handle,
                generalized_velocity_words: velocities
                    .iter()
                    .map(|value| motor_stage_word(*value))
                    .collect(),
                constructed_rows,
                endpoints,
            });
        }
    }

    fn record_construction_endpoint(
        &mut self,
        context: InternalMotorStageContext,
        phase: MultibodyMotorConstructionPhase,
        multibodies: &MultibodyJointSet,
        joint_constraints: &SolverConstraintsSet<JointConstraintTypes>,
        bindings: &[InternalMotorStageBinding],
        constructed_rows: Vec<MultibodyMotorConstructionRow>,
    ) {
        if self.motor_construction_events.is_none() {
            return;
        }
        let endpoints = if context.initialized {
            bindings
                .iter()
                .filter_map(|binding| {
                    let (multibody, link_id) = multibodies.get(binding.joint_handle)?;
                    if link_id != binding.link_id {
                        return None;
                    }
                    Self::construction_endpoints_for_multibody(
                        multibody,
                        joint_constraints,
                        std::slice::from_ref(binding),
                    )
                    .into_iter()
                    .next()
                })
                .collect()
        } else {
            Vec::new()
        };
        let context = self.construction_context(context);
        Self::push_construction_event(
            &mut self.motor_construction_events,
            context,
            phase,
            &self.generic_solver_vels,
            constructed_rows,
            endpoints,
            None,
        );
    }

    pub(crate) fn record_motor_construction_writeback(
        &mut self,
        multibodies: &MultibodyJointSet,
        joint_constraints: &SolverConstraintsSet<JointConstraintTypes>,
    ) {
        if let Some(context) = self.motor_construction_final_context {
            let bindings = self.motor_construction_final_bindings.clone();
            self.record_construction_endpoint(
                context,
                MultibodyMotorConstructionPhase::AfterVelocityWriteback,
                multibodies,
                joint_constraints,
                &bindings,
                Vec::new(),
            );
        }
    }

    pub fn solve_constraints(
        &mut self,
        params: &IntegrationParameters,
        num_substeps: usize,
        bodies: &mut RigidBodySet,
        multibodies: &mut MultibodyJointSet,
        contact_constraints: &mut SolverConstraintsSet<ContactConstraintTypes>,
        joint_constraints: &mut SolverConstraintsSet<JointConstraintTypes>,
    ) {
        for substep_id in 0..num_substeps {
            let is_last_substep = substep_id == num_substeps - 1;
            let mut stage_context = InternalMotorStageContext {
                partition: self.motor_stage_partition_index,
                island: self.motor_stage_island_index,
                slice: substep_id,
                expected_slices: num_substeps,
                iteration: None,
                stage: MultibodyMotorStage::BeforeExternalForces,
                duration: params.dt,
                initialized: false,
            };
            let mut stage_bindings = Vec::new();
            self.record_motor_stage(
                stage_context,
                MultibodyMotorStage::BeforeExternalForces,
                joint_constraints,
                &stage_bindings,
            );

            self.record_construction_endpoint(
                stage_context,
                MultibodyMotorConstructionPhase::BeforeExternalForces,
                multibodies,
                joint_constraints,
                &[],
                Vec::new(),
            );
            for (solver_vels, incr) in self
                .solver_vels
                .iter_mut()
                .zip(self.solver_vels_increment.iter())
            {
                solver_vels.linear += incr.linear;
                solver_vels.angular += incr.angular;
            }

            self.generic_solver_vels += &self.generic_solver_vels_increment;
            self.record_motor_stage(
                stage_context,
                MultibodyMotorStage::AfterExternalForces,
                joint_constraints,
                &stage_bindings,
            );

            /*
             * Update & solve constraints.
             */
            self.record_construction_endpoint(
                stage_context,
                MultibodyMotorConstructionPhase::AfterExternalForces,
                multibodies,
                joint_constraints,
                &[],
                Vec::new(),
            );
            let mut constructed_rows = Vec::new();
            if self.motor_construction_events.is_some() {
                joint_constraints.update_with_construction_inputs(
                    params,
                    multibodies,
                    &self.solver_bodies,
                    Some(&mut constructed_rows),
                );
            } else {
                joint_constraints.update(params, multibodies, &self.solver_bodies);
            }
            // Builder metadata and row values are read only after update has
            // initialized every row for the current slice.
            if self.motor_stage_events.is_some() || self.motor_construction_events.is_some() {
                for builder in &joint_constraints.generic_velocity_one_body_constraints_builder {
                    builder.append_internal_motor_stage_bindings(
                        multibodies,
                        &joint_constraints.generic_velocity_one_body_constraints,
                        &mut stage_bindings,
                    );
                }
            }
            stage_context.initialized = true;
            self.record_construction_endpoint(
                stage_context,
                MultibodyMotorConstructionPhase::AfterRowConstruction,
                multibodies,
                joint_constraints,
                &stage_bindings,
                constructed_rows,
            );
            self.record_motor_stage(
                stage_context,
                MultibodyMotorStage::AfterJointUpdate,
                joint_constraints,
                &stage_bindings,
            );
            contact_constraints.update(params, substep_id, multibodies, &self.solver_bodies);
            self.record_motor_stage(
                stage_context,
                MultibodyMotorStage::AfterContactUpdate,
                joint_constraints,
                &stage_bindings,
            );

            if params.warmstart_coefficient != 0.0 {
                contact_constraints.warmstart(&mut self.solver_vels, &mut self.generic_solver_vels);
                self.record_motor_stage(
                    stage_context,
                    MultibodyMotorStage::AfterContactWarmstart,
                    joint_constraints,
                    &stage_bindings,
                );
            }

            for iteration in 0..params.num_internal_pgs_iterations {
                stage_context.iteration = Some(iteration);
                self.record_motor_stage(
                    stage_context,
                    MultibodyMotorStage::BeforeJointSolve,
                    joint_constraints,
                    &stage_bindings,
                );
                if let Some(events) = &mut self.motor_stage_events {
                    joint_constraints.solve_with_motor_stages(
                        &mut self.solver_vels,
                        &mut self.generic_solver_vels,
                        &stage_bindings,
                        stage_context,
                        events,
                        false,
                    );
                } else {
                    joint_constraints.solve(&mut self.solver_vels, &mut self.generic_solver_vels);
                }
                self.record_motor_stage(
                    stage_context,
                    MultibodyMotorStage::AfterJointSolve,
                    joint_constraints,
                    &stage_bindings,
                );
                contact_constraints
                    .solve_restitution(&mut self.solver_vels, &mut self.generic_solver_vels);
                self.record_motor_stage(
                    stage_context,
                    MultibodyMotorStage::AfterContactNormal,
                    joint_constraints,
                    &stage_bindings,
                );
                contact_constraints
                    .solve_friction(&mut self.solver_vels, &mut self.generic_solver_vels);
                self.record_motor_stage(
                    stage_context,
                    MultibodyMotorStage::AfterContactFriction,
                    joint_constraints,
                    &stage_bindings,
                );
            }

            if is_last_substep {
                for iteration in 0..params.num_additional_friction_iterations {
                    stage_context.iteration = Some(iteration);
                    contact_constraints
                        .solve_friction(&mut self.solver_vels, &mut self.generic_solver_vels);
                    self.record_motor_stage(
                        stage_context,
                        MultibodyMotorStage::AfterAdditionalFriction,
                        joint_constraints,
                        &stage_bindings,
                    );
                }
            }

            /*
             * Integrate positions.
             */
            let construction_observation = self.motor_construction_events.as_ref().map(|_| {
                (
                    InternalMotorStageContext {
                        iteration: None,
                        ..self.construction_context(stage_context)
                    },
                    &*joint_constraints,
                    stage_bindings.as_slice(),
                )
            });
            if construction_observation.is_some() {
                self.integrate_positions_with_construction(
                    params,
                    is_last_substep,
                    bodies,
                    multibodies,
                    construction_observation,
                );
            } else {
                self.integrate_positions(params, is_last_substep, bodies, multibodies);
            }
            stage_context.iteration = None;
            self.record_motor_stage(
                stage_context,
                MultibodyMotorStage::AfterIntegration,
                joint_constraints,
                &stage_bindings,
            );

            /*
             * Resolution without bias.
             */
            if params.num_internal_stabilization_iterations > 0 {
                for iteration in 0..params.num_internal_stabilization_iterations {
                    stage_context.iteration = Some(iteration);
                    self.record_motor_stage(
                        stage_context,
                        MultibodyMotorStage::BeforeJointStabilization,
                        joint_constraints,
                        &stage_bindings,
                    );
                    if let Some(events) = &mut self.motor_stage_events {
                        joint_constraints.solve_with_motor_stages(
                            &mut self.solver_vels,
                            &mut self.generic_solver_vels,
                            &stage_bindings,
                            stage_context,
                            events,
                            true,
                        );
                    } else {
                        joint_constraints
                            .solve_wo_bias(&mut self.solver_vels, &mut self.generic_solver_vels);
                    }
                    self.record_motor_stage(
                        stage_context,
                        MultibodyMotorStage::AfterJointStabilization,
                        joint_constraints,
                        &stage_bindings,
                    );
                    self.record_construction_endpoint(
                        stage_context,
                        MultibodyMotorConstructionPhase::AfterJointStabilization,
                        multibodies,
                        joint_constraints,
                        &stage_bindings,
                        Vec::new(),
                    );
                    contact_constraints.solve_restitution_wo_bias(
                        &mut self.solver_vels,
                        &mut self.generic_solver_vels,
                    );
                    self.record_motor_stage(
                        stage_context,
                        MultibodyMotorStage::AfterContactStabilization,
                        joint_constraints,
                        &stage_bindings,
                    );
                }

                stage_context.iteration = None;
                contact_constraints
                    .solve_friction(&mut self.solver_vels, &mut self.generic_solver_vels);
                self.record_motor_stage(
                    stage_context,
                    MultibodyMotorStage::AfterStabilizationFriction,
                    joint_constraints,
                    &stage_bindings,
                );
            }

            stage_context.iteration = None;
            self.record_motor_stage(
                stage_context,
                MultibodyMotorStage::FinalSlice,
                joint_constraints,
                &stage_bindings,
            );

            self.record_construction_endpoint(
                stage_context,
                MultibodyMotorConstructionPhase::FinalSlice,
                multibodies,
                joint_constraints,
                &stage_bindings,
                Vec::new(),
            );
            if self.motor_construction_events.is_some() {
                self.motor_construction_final_context = Some(stage_context);
                self.motor_construction_final_bindings = stage_bindings.clone();
            }
            // Observe final net impulses once, after all stabilization passes.
            if let Some(samples) = &mut self.motor_impulse_samples {
                for builder in &joint_constraints.generic_velocity_one_body_constraints_builder {
                    builder.append_internal_motor_impulse_samples(
                        multibodies,
                        &joint_constraints.generic_velocity_one_body_constraints,
                        self.motor_impulse_partition_index,
                        substep_id,
                        num_substeps,
                        params.dt,
                        samples,
                    );
                }
            }
        }
    }

    pub fn integrate_positions(
        &mut self,
        params: &IntegrationParameters,
        is_last_substep: bool,
        bodies: &mut RigidBodySet,
        multibodies: &mut MultibodyJointSet,
    ) {
        self.integrate_positions_with_construction(
            params,
            is_last_substep,
            bodies,
            multibodies,
            None,
        );
    }

    fn integrate_positions_with_construction(
        &mut self,
        params: &IntegrationParameters,
        is_last_substep: bool,
        bodies: &mut RigidBodySet,
        multibodies: &mut MultibodyJointSet,
        construction_observation: Option<(
            InternalMotorStageContext,
            &SolverConstraintsSet<JointConstraintTypes>,
            &[InternalMotorStageBinding],
        )>,
    ) {
        // Integrate positions.
        for (solver_vels, solver_body) in self.solver_vels.iter().zip(self.solver_bodies.iter_mut())
        {
            let linvel = solver_vels.linear;
            let angvel = solver_body.sqrt_ii.transform_vector(solver_vels.angular);

            let mut new_vels = RigidBodyVelocity { linvel, angvel };
            new_vels = new_vels.apply_damping(params.dt, &solver_body.damping);
            let new_pos =
                new_vels.integrate(params.dt, &solver_body.position, &solver_body.local_com);
            solver_body.integrated_vels += new_vels;
            solver_body.position = new_pos;
            solver_body.world_com = new_pos * solver_body.local_com;
        }

        // Integrate multibody positions.
        for link in &self.multibody_roots {
            let multibody = multibodies
                .get_multibody_mut_internal(link.multibody)
                .unwrap();
            let solver_vels = self
                .generic_solver_vels
                .rows(multibody.solver_id, multibody.ndofs());
            multibody.velocities.copy_from(&solver_vels);
            if let Some((context, constraints, bindings)) = construction_observation {
                let (index, generation) = link.multibody.0.into_raw_parts();
                let endpoints =
                    Self::construction_endpoints_for_multibody(multibody, constraints, bindings);
                Self::push_construction_event(
                    &mut self.motor_construction_events,
                    context,
                    MultibodyMotorConstructionPhase::BeforePositionIntegration,
                    &self.generic_solver_vels,
                    Vec::new(),
                    endpoints,
                    Some([index, generation]),
                );
            }
            multibody.integrate(params.dt);
            if let Some((context, constraints, bindings)) = construction_observation {
                let (index, generation) = link.multibody.0.into_raw_parts();
                let endpoints =
                    Self::construction_endpoints_for_multibody(multibody, constraints, bindings);
                Self::push_construction_event(
                    &mut self.motor_construction_events,
                    context,
                    MultibodyMotorConstructionPhase::AfterPositionIntegration,
                    &self.generic_solver_vels,
                    Vec::new(),
                    endpoints,
                    Some([index, generation]),
                );
            }
            // PERF: don’t write back to the rigid-body poses `bodies` before the last step?
            multibody.forward_kinematics(bodies, false);
            multibody.update_rigid_bodies_internal(bodies, !is_last_substep, true, false);

            if !is_last_substep {
                // These are very expensive and not needed if we don’t
                // have to run another step.
                multibody.update_dynamics(params.dt, bodies);
                multibody.update_acceleration(bodies);

                let mut solver_vels_incr = self
                    .generic_solver_vels_increment
                    .rows_mut(multibody.solver_id, multibody.ndofs());
                solver_vels_incr.axpy(params.dt, &multibody.accelerations, 0.0);
            }
        }
    }

    pub fn writeback_bodies(
        &mut self,
        params: &IntegrationParameters,
        num_substeps: usize,
        islands: &IslandManager,
        island_id: usize,
        bodies: &mut RigidBodySet,
        multibodies: &mut MultibodyJointSet,
    ) {
        for handle in islands.active_island(island_id) {
            let link = if self.multibody_roots.is_empty() {
                None
            } else {
                multibodies.rigid_body_link(*handle).copied()
            };

            if let Some(link) = link {
                let multibody = multibodies
                    .get_multibody_mut_internal(link.multibody)
                    .unwrap();

                if link.id == 0 || link.id == 1 && !multibody.root_is_dynamic {
                    let solver_vels = self
                        .generic_solver_vels
                        .rows(multibody.solver_id, multibody.ndofs());
                    multibody.velocities.copy_from(&solver_vels);
                    multibody.refresh_kinematic_root_velocities(bodies);
                }
            } else {
                let rb = bodies.index_mut_internal(*handle);
                let solver_body = &self.solver_bodies[rb.ids.active_set_offset];
                let solver_vels = &self.solver_vels[rb.ids.active_set_offset];

                let dangvel = solver_body.sqrt_ii.transform_vector(solver_vels.angular);

                let mut new_vels = RigidBodyVelocity {
                    linvel: solver_vels.linear,
                    angvel: dangvel,
                };
                new_vels = new_vels.apply_damping(params.dt, &solver_body.damping);

                // NOTE: using integrated_vels instead of interpolation is interesting for
                //       high angular velocities. However, it is a bit inexact due to the
                //       solver integrating at intermediate sub-steps. Should we just switch
                //       to interpolation?
                rb.integrated_vels.linvel =
                    solver_body.integrated_vels.linvel / num_substeps as Real;
                rb.integrated_vels.angvel =
                    solver_body.integrated_vels.angvel / num_substeps as Real;
                rb.vels = new_vels;
                rb.pos.next_position = solver_body.position;
            }
        }
    }
}
