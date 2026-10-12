use super::*;
use crate::dynamics::{
    CCDSolver, GenericJoint, ImpulseJointSet, IntegrationParameters, IslandManager,
    MultibodyJointSet, PrismaticJointBuilder, RevoluteJointBuilder, RigidBodyBuilder,
    RigidBodyHandle, RigidBodySet,
};
use crate::geometry::{BroadPhaseMultiSap, ColliderBuilder, ColliderSet, NarrowPhase};
use crate::math::Vector;
use crate::pipeline::PhysicsPipeline;
use std::num::NonZeroUsize;

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
    child: RigidBodyHandle,
    joint: MultibodyJointHandle,
}

impl Fixture {
    fn new(angular: bool) -> Self {
        let mut bodies = RigidBodySet::new();
        let parent = bodies.insert(RigidBodyBuilder::fixed().translation(Vector::z()));
        let child = bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::z())
                .can_sleep(false),
        );
        let mut colliders = ColliderSet::new();
        colliders.insert_with_parent(ColliderBuilder::ball(0.1).mass(1.0), child, &mut bodies);
        let joint: GenericJoint = if angular {
            RevoluteJointBuilder::new(Vector::x_axis())
                .motor_model(MotorModel::ForceBased)
                .motor_position(0.4, 100.0, 1.0)
                .motor_max_force(2.0)
                .build()
                .into()
        } else {
            PrismaticJointBuilder::new(Vector::x_axis())
                .motor_model(MotorModel::ForceBased)
                .motor_position(0.4, 100.0, 1.0)
                .motor_max_force(2.0)
                .build()
                .into()
        };
        let mut multibody_joints = MultibodyJointSet::new();
        let joint = multibody_joints.insert(parent, child, joint, true).unwrap();
        Self {
            pipeline: PhysicsPipeline::new(),
            parameters: IntegrationParameters {
                dt: 0.001,
                min_island_size: 1,
                num_solver_iterations: NonZeroUsize::new(4).unwrap(),
                num_internal_pgs_iterations: 2,
                num_internal_stabilization_iterations: 2,
                num_additional_friction_iterations: 1,
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
            child,
            joint,
        }
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

    fn report(&self) -> &MultibodyMotorStageTelemetry {
        self.pipeline.completed_multibody_motor_stages().unwrap()
    }

    fn add_child_motor(&mut self) -> MultibodyJointHandle {
        let child = self.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::x() + Vector::z())
                .can_sleep(false),
        );
        self.colliders.insert_with_parent(
            ColliderBuilder::ball(0.1).mass(2.0),
            child,
            &mut self.bodies,
        );
        self.multibody_joints
            .insert(
                self.child,
                child,
                RevoluteJointBuilder::new(Vector::y_axis())
                    .local_anchor1(na::point![1.0, 0.0, 0.0])
                    .motor_model(MotorModel::ForceBased)
                    .motor_position(-0.4, 100.0, 1.0)
                    .motor_max_force(3.0),
                true,
            )
            .unwrap()
    }

    fn add_contact(&mut self) {
        let wall = self
            .bodies
            .insert(RigidBodyBuilder::fixed().translation(Vector::new(0.15, 0.0, 1.0)));
        self.colliders.insert_with_parent(
            ColliderBuilder::cuboid(0.1, 1.0, 1.0),
            wall,
            &mut self.bodies,
        );
    }

    fn state_words(&self) -> Vec<u64> {
        let mut words = Vec::new();
        for (handle, body) in self.bodies.iter() {
            let (index, generation) = handle.into_raw_parts();
            words.extend([u64::from(index), u64::from(generation)]);
            words.extend(
                body.position()
                    .translation
                    .vector
                    .iter()
                    .map(|v| motor_stage_word(*v)),
            );
            words.extend(
                body.position()
                    .rotation
                    .coords
                    .iter()
                    .map(|v| motor_stage_word(*v)),
            );
            words.extend(body.linvel().iter().map(|v| motor_stage_word(*v)));
            words.extend(body.angvel().iter().map(|v| motor_stage_word(*v)));
            words.extend(body.user_force().iter().map(|v| motor_stage_word(*v)));
            words.extend(body.user_torque().iter().map(|v| motor_stage_word(*v)));
        }
        for (handle, _, multibody, link) in self.multibody_joints.iter() {
            let (index, generation) = handle.into_raw_parts();
            words.extend([u64::from(index), u64::from(generation)]);
            words.extend(
                multibody
                    .generalized_velocity()
                    .iter()
                    .map(|v| motor_stage_word(*v)),
            );
            words.extend(link.joint().coords.iter().map(|v| motor_stage_word(*v)));
            words.extend(
                link.joint()
                    .data
                    .motors
                    .iter()
                    .map(|motor| motor_stage_word(motor.impulse)),
            );
        }
        words
    }
}

#[test]
fn stage_lifecycle_is_opt_in_and_independent_of_impulse_observation() {
    let mut fixture = Fixture::new(false);
    assert!(fixture
        .pipeline
        .completed_multibody_motor_stages()
        .is_none());
    fixture
        .pipeline
        .set_multibody_motor_impulse_telemetry_enabled(true);
    fixture.step();
    assert!(fixture
        .pipeline
        .completed_multibody_motor_stages()
        .is_none());
    assert!(fixture
        .pipeline
        .completed_multibody_motor_impulses()
        .is_some());
    fixture
        .pipeline
        .set_multibody_motor_stage_telemetry_enabled(true);
    assert!(fixture
        .pipeline
        .completed_multibody_motor_stages()
        .is_none());
    fixture.step();
    assert!(!fixture.report().events.is_empty());
    fixture
        .pipeline
        .set_multibody_motor_stage_telemetry_enabled(false);
    assert!(fixture
        .pipeline
        .completed_multibody_motor_stages()
        .is_none());
    assert!(fixture
        .pipeline
        .completed_multibody_motor_impulses()
        .is_some());
    fixture
        .pipeline
        .set_multibody_motor_stage_telemetry_enabled(true);
    let (body, link) = fixture.multibody_joints.get_mut(fixture.joint).unwrap();
    body.link_mut(link).unwrap().joint.data.motor_axes = crate::dynamics::JointAxesMask::empty();
    fixture.step();
    assert!(fixture
        .report()
        .events
        .iter()
        .all(|event| event.rows.is_empty()));
}

#[test]
fn initialized_rows_and_immediate_visits_preserve_bias_and_native_identity() {
    for angular in [false, true] {
        let mut fixture = Fixture::new(angular);
        fixture.bodies[fixture.child].set_additional_solver_iterations(3);
        fixture
            .pipeline
            .set_multibody_motor_stage_telemetry_enabled(true);
        fixture.step();
        let report = fixture.report();
        assert_eq!(
            report.native_scalar_bits,
            (std::mem::size_of::<Real>() * 8) as u8
        );
        assert_eq!(
            report.numerical_outer_interval_word,
            motor_stage_word(0.001)
        );
        for slice in 0..7 {
            let events: Vec<_> = report
                .events
                .iter()
                .filter(|event| event.internal_slice_index == slice)
                .collect();
            assert!(!events.is_empty());
            assert_eq!(events[0].stage, MultibodyMotorStage::BeforeExternalForces);
            assert_eq!(events[1].stage, MultibodyMotorStage::AfterExternalForces);
            for event in &events[..2] {
                assert!(event.rows.is_empty());
                assert_eq!(event.row_cache_slice_index, None);
            }
            for event in &events[2..] {
                assert_eq!(event.row_cache_slice_index, Some(slice));
                assert_eq!(event.expected_internal_slice_count, 7);
                assert_eq!(event.rows.len(), 1);
                let row = &event.rows[0];
                assert_eq!(row.joint_handle, fixture.joint);
                assert_eq!(row.link_id, 1);
                assert_eq!(
                    row.axis,
                    if angular {
                        JointAxis::AngX
                    } else {
                        JointAxis::LinX
                    }
                );
                assert_eq!(row.ndofs2, 1);
                assert_eq!(row.jacobian_words.len(), row.ndofs2);
                assert_eq!(row.weighted_jacobian_words.len(), row.ndofs2);
                assert_eq!(row.gain_model, MotorModel::ForceBased);
                assert_eq!(row.effort_ceiling_word, motor_stage_word(2.0));
            }
            let mut motor_visits = 0;
            let mut bias_visits = 0;
            for (index, event) in events.iter().enumerate() {
                if event.stage == MultibodyMotorStage::BeforeMotorSolve {
                    let after = events[index + 1];
                    assert_eq!(after.stage, MultibodyMotorStage::AfterMotorSolve);
                    assert_eq!(after.joint_visit_index, event.joint_visit_index);
                    assert!(event.joint_visit_index.is_some());
                    assert_eq!(after.iteration_index, event.iteration_index);
                    let mut expected = event.rows[0].clone();
                    expected.impulse_word = after.rows[0].impulse_word;
                    assert_eq!(expected, after.rows[0]);
                    motor_visits += 1;
                }
                if event.stage == MultibodyMotorStage::BeforeMotorBiasRemoval {
                    let before_solve = events[index + 1];
                    assert_eq!(before_solve.stage, MultibodyMotorStage::BeforeMotorSolve);
                    assert_eq!(before_solve.joint_visit_index, event.joint_visit_index);
                    let mut expected = event.rows[0].clone();
                    expected.rhs_word = expected.rhs_without_bias_word;
                    assert_eq!(expected, before_solve.rows[0]);
                    assert_eq!(
                        event.generalized_velocity_words,
                        before_solve.generalized_velocity_words
                    );
                    bias_visits += 1;
                }
            }
            assert_eq!(motor_visits, 4);
            assert_eq!(bias_visits, 2);
            assert_eq!(
                events.last().unwrap().stage,
                MultibodyMotorStage::FinalSlice
            );
        }
    }
}

#[test]
fn single_dof_child_motors_retain_the_full_coupled_multibody_block() {
    let mut fixture = Fixture::new(false);
    let second = fixture.add_child_motor();
    let (body, link) = fixture.multibody_joints.get_mut(fixture.joint).unwrap();
    body.link_mut(link)
        .unwrap()
        .joint
        .data
        .set_limits(JointAxis::LinX, [-0.001, 0.001]);
    fixture
        .pipeline
        .set_multibody_motor_stage_telemetry_enabled(true);
    fixture.step();
    for event in &fixture.report().events {
        assert_eq!(event.generalized_velocity_words.len(), 2);
        if event.row_cache_slice_index.is_none() {
            assert!(event.rows.is_empty());
            continue;
        }
        let immediate = event.joint_visit_index.is_some();
        assert_eq!(event.rows.len(), if immediate { 1 } else { 2 });
        for row in &event.rows {
            assert_eq!(row.ndofs2, 2);
            assert_eq!(row.solver_velocity_offset, 0);
            assert_eq!(row.jacobian_words.len(), 2);
            assert_eq!(row.weighted_jacobian_words.len(), 2);
            assert!(row.joint_handle == fixture.joint || row.joint_handle == second);
        }
        if !immediate {
            assert_ne!(
                event.rows[0].constraint_index,
                event.rows[1].constraint_index
            );
            assert_ne!(event.rows[0].j_id2, event.rows[1].j_id2);
        }
    }
}

#[test]
fn observation_on_off_and_fresh_runs_preserve_contact_and_joint_state_bits() {
    let mut on = Fixture::new(false);
    let mut off = Fixture::new(false);
    let mut fresh = Fixture::new(false);
    for fixture in [&mut on, &mut off, &mut fresh] {
        fixture.add_child_motor();
        fixture.add_contact();
        fixture.bodies[fixture.child].add_force(Vector::x() * 5.0, true);
        fixture
            .pipeline
            .set_multibody_motor_impulse_telemetry_enabled(true);
    }
    on.pipeline
        .set_multibody_motor_stage_telemetry_enabled(true);
    fresh
        .pipeline
        .set_multibody_motor_stage_telemetry_enabled(true);
    for _ in 0..8 {
        on.step();
        off.step();
        fresh.step();
        assert_eq!(on.state_words(), off.state_words());
        assert_eq!(on.state_words(), fresh.state_words());
        assert_eq!(on.report(), fresh.report());
        assert_eq!(
            on.pipeline.completed_multibody_motor_impulses(),
            off.pipeline.completed_multibody_motor_impulses()
        );
        assert!(off.pipeline.completed_multibody_motor_stages().is_none());
    }
}

#[test]
fn contact_normal_writes_are_separate_from_joint_stage_writes() {
    let mut fixture = Fixture::new(false);
    fixture.add_contact();
    fixture
        .pipeline
        .set_multibody_motor_stage_telemetry_enabled(true);
    fixture.step();
    assert!(fixture
        .narrow
        .contact_pairs()
        .any(|pair| pair.has_any_active_contact));
    let mut normal_changed = false;
    for events in fixture.report().events.windows(2) {
        if events[1].stage == MultibodyMotorStage::AfterContactNormal {
            assert_eq!(events[0].stage, MultibodyMotorStage::AfterJointSolve);
            assert_eq!(events[0].rows, events[1].rows);
            normal_changed |=
                events[0].generalized_velocity_words != events[1].generalized_velocity_words;
        }
    }
    assert!(
        normal_changed,
        "the fixture must exercise a contact-only generalized velocity write"
    );
}

#[test]
fn skipped_solver_passes_do_not_invent_motor_visits() {
    let mut fixture = Fixture::new(false);
    fixture.parameters.num_internal_pgs_iterations = 0;
    fixture.parameters.num_internal_stabilization_iterations = 0;
    fixture.parameters.num_additional_friction_iterations = 0;
    fixture.parameters.warmstart_coefficient = 0.0;
    fixture
        .pipeline
        .set_multibody_motor_stage_telemetry_enabled(true);
    fixture.step();
    for slice in 0..4 {
        let stages: Vec<_> = fixture
            .report()
            .events
            .iter()
            .filter(|event| event.internal_slice_index == slice)
            .map(|event| event.stage)
            .collect();
        assert_eq!(
            stages,
            [
                MultibodyMotorStage::BeforeExternalForces,
                MultibodyMotorStage::AfterExternalForces,
                MultibodyMotorStage::AfterJointUpdate,
                MultibodyMotorStage::AfterContactUpdate,
                MultibodyMotorStage::AfterIntegration,
                MultibodyMotorStage::FinalSlice,
            ]
        );
    }
}

#[test]
fn independent_islands_drain_in_actual_island_order_with_exact_replay() {
    let mut on = Fixture::new(false);
    let mut off = Fixture::new(false);
    let mut fresh = Fixture::new(false);
    for fixture in [&mut on, &mut off, &mut fresh] {
        let parent = fixture
            .bodies
            .insert(RigidBodyBuilder::fixed().translation(Vector::z() * 5.0));
        let child = fixture.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::z() * 5.0)
                .can_sleep(false),
        );
        fixture.bodies[child].set_additional_solver_iterations(3);
        fixture.colliders.insert_with_parent(
            ColliderBuilder::ball(0.1).mass(1.0),
            child,
            &mut fixture.bodies,
        );
        fixture
            .multibody_joints
            .insert(
                parent,
                child,
                PrismaticJointBuilder::new(Vector::x_axis())
                    .motor_velocity(-0.1, 1.0)
                    .motor_max_force(3.0),
                true,
            )
            .unwrap();
    }
    on.pipeline
        .set_multibody_motor_stage_telemetry_enabled(true);
    fresh
        .pipeline
        .set_multibody_motor_stage_telemetry_enabled(true);
    for _ in 0..4 {
        on.step();
        off.step();
        fresh.step();
        assert!(on.islands.num_islands() >= 2);
        assert_eq!(on.state_words(), off.state_words());
        assert_eq!(on.state_words(), fresh.state_words());
        assert_eq!(on.report(), fresh.report());
        assert!(on.report().events.windows(2).all(|pair| (
            pair[0].ccd_partition_index,
            pair[0].island_index
        ) <= (
            pair[1].ccd_partition_index,
            pair[1].island_index
        )));
        let finals: Vec<_> = on
            .report()
            .events
            .iter()
            .filter(|event| event.stage == MultibodyMotorStage::FinalSlice)
            .collect();
        assert!(finals
            .iter()
            .any(|event| event.expected_internal_slice_count == 4));
        assert!(finals
            .iter()
            .any(|event| event.expected_internal_slice_count == 7));
    }
}

#[test]
fn ccd_partitions_keep_actual_partition_ordinals_and_observer_parity() {
    let mut on = Fixture::new(false);
    let mut off = Fixture::new(false);
    for fixture in [&mut on, &mut off] {
        fixture.parameters.dt = 0.0001;
        fixture.parameters.max_ccd_substeps = 4;
        let wall = fixture
            .bodies
            .insert(RigidBodyBuilder::fixed().translation(Vector::x() * 2.0));
        fixture.colliders.insert_with_parent(
            ColliderBuilder::cuboid(0.1, 1.0, 0.5),
            wall,
            &mut fixture.bodies,
        );
        let projectile = fixture.bodies.insert(
            RigidBodyBuilder::dynamic()
                .linvel(Vector::x() * 500.0)
                .ccd_enabled(true)
                .can_sleep(false),
        );
        fixture.colliders.insert_with_parent(
            ColliderBuilder::ball(0.1).mass(1.0),
            projectile,
            &mut fixture.bodies,
        );
        fixture.step();
        fixture.parameters.dt = 0.01;
    }
    on.pipeline
        .set_multibody_motor_stage_telemetry_enabled(true);
    on.step();
    off.step();
    assert_eq!(on.state_words(), off.state_words());
    assert!(on.report().ccd_partition_duration_words.len() > 1);
    for partition in 0..on.report().ccd_partition_duration_words.len() {
        let events: Vec<_> = on
            .report()
            .events
            .iter()
            .filter(|event| event.ccd_partition_index == partition)
            .collect();
        assert!(!events.is_empty());
        assert!(events
            .iter()
            .any(|event| event.stage == MultibodyMotorStage::FinalSlice
                && event.rows.iter().any(|row| row.joint_handle == on.joint)));
    }
}

#[cfg(feature = "serde-serialize")]
#[test]
fn serialization_preserves_native_signed_zero_nonfinite_and_bound_words() {
    let row = MultibodyMotorStageRow {
        joint_handle: MultibodyJointHandle::from_raw_parts(17, 3),
        link_id: 1,
        axis: JointAxis::LinX,
        gain_model: MotorModel::ForceBased,
        constraint_index: 0,
        solver_velocity_offset: 0,
        ndofs2: 1,
        j_id2: 0,
        jacobian_words: vec![motor_stage_word(-0.0)],
        weighted_jacobian_words: vec![motor_stage_word(Real::NAN)],
        rhs_word: motor_stage_word(Real::INFINITY),
        rhs_without_bias_word: motor_stage_word(Real::NEG_INFINITY),
        cfm_gain_word: motor_stage_word(Real::MIN_POSITIVE / 2.0),
        cfm_coefficient_word: motor_stage_word(0.0),
        inverse_lhs_word: motor_stage_word(1.0),
        impulse_word: motor_stage_word(-0.0),
        impulse_bound_words: [
            motor_stage_word(Real::NEG_INFINITY),
            motor_stage_word(Real::INFINITY),
        ],
        effort_ceiling_word: motor_stage_word(Real::INFINITY),
    };
    let mut report = MultibodyMotorStageTelemetry::new(0.001);
    report.events.push(motor_stage_event(
        InternalMotorStageContext {
            partition: 0,
            island: 0,
            slice: 0,
            expected_slices: 1,
            iteration: Some(0),
            stage: MultibodyMotorStage::BeforeMotorSolve,
            duration: 0.001,
            initialized: true,
        },
        Some(0),
        &DVector::from_vec(vec![-0.0]),
        vec![row],
    ));
    let json = serde_json::to_vec(&report).unwrap();
    assert_eq!(
        report,
        serde_json::from_slice::<MultibodyMotorStageTelemetry>(&json).unwrap()
    );
}
