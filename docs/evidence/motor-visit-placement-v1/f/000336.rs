use super::*;
use crate::dynamics::{
    CCDSolver, GenericJoint, ImpulseJointSet, IntegrationParameters, IslandManager, MotorModel,
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
    fn new(angular: bool, target: Real, ceiling: Real) -> Self {
        let mut bodies = RigidBodySet::new();
        let parent = bodies.insert(RigidBodyBuilder::fixed().translation(Vector::z() * 10.0));
        let child = bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::z() * 10.0)
                .can_sleep(false),
        );
        let mut colliders = ColliderSet::new();
        colliders.insert_with_parent(ColliderBuilder::ball(0.1).mass(1.0), child, &mut bodies);
        let joint: GenericJoint = if angular {
            RevoluteJointBuilder::new(Vector::x_axis())
                .motor_model(MotorModel::ForceBased)
                .motor_position(target, 100.0, 1.0)
                .motor_max_force(ceiling)
                .build()
                .into()
        } else {
            PrismaticJointBuilder::new(Vector::x_axis())
                .motor_model(MotorModel::ForceBased)
                .motor_position(target, 100.0, 1.0)
                .motor_max_force(ceiling)
                .build()
                .into()
        };
        let mut multibody_joints = MultibodyJointSet::new();
        let joint = multibody_joints.insert(parent, child, joint, true).unwrap();
        Self {
            pipeline: PhysicsPipeline::new(),
            parameters: IntegrationParameters {
                dt: 0.001,
                num_solver_iterations: NonZeroUsize::new(4).unwrap(),
                num_internal_pgs_iterations: 7,
                num_internal_stabilization_iterations: 3,
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

    fn record(&self) -> &MultibodyMotorImpulseRecord {
        let report = self.pipeline.completed_multibody_motor_impulses().unwrap();
        assert_eq!(report.motors.len(), 1);
        &report.motors[0]
    }

    fn add_ccd_impact(&mut self) {
        // Prime integrated velocities before the interval whose CCD split is measured.
        self.parameters.dt = 0.0001;
        self.parameters.max_ccd_substeps = 4;
        let wall = self
            .bodies
            .insert(RigidBodyBuilder::fixed().translation(Vector::x() * 2.0));
        self.colliders.insert_with_parent(
            ColliderBuilder::cuboid(0.1, 1.0, 1.0),
            wall,
            &mut self.bodies,
        );
        let projectile = self.bodies.insert(
            RigidBodyBuilder::dynamic()
                .linvel(Vector::x() * 500.0)
                .ccd_enabled(true)
                .can_sleep(false),
        );
        self.colliders.insert_with_parent(
            ColliderBuilder::ball(0.1).mass(1.0),
            projectile,
            &mut self.bodies,
        );
        self.step();
        self.parameters.dt = 0.01;
    }

    fn joint_velocity(&self) -> Real {
        let (multibody, id) = self.multibody_joints.get(self.joint).unwrap();
        multibody.joint_velocity(multibody.link(id).unwrap())[0]
    }

    fn state_bits(&self) -> Vec<u32> {
        let mut words = Vec::new();
        for (handle, body) in self.bodies.iter() {
            let (index, generation) = handle.into_raw_parts();
            words.extend([index, generation]);
            words.extend(
                body.position()
                    .translation
                    .vector
                    .iter()
                    .map(|v| v.to_bits()),
            );
            words.extend(body.position().rotation.coords.iter().map(|v| v.to_bits()));
            words.extend(body.linvel().iter().map(|v| v.to_bits()));
            words.extend(body.angvel().iter().map(|v| v.to_bits()));
            words.extend(body.user_force().iter().map(|v| v.to_bits()));
            words.extend(body.user_torque().iter().map(|v| v.to_bits()));
        }
        for (handle, _, multibody, link) in self.multibody_joints.iter() {
            let (index, generation) = handle.into_raw_parts();
            words.extend([index, generation]);
            words.extend(multibody.generalized_velocity().iter().map(|v| v.to_bits()));
            words.extend(link.joint().coords.iter().map(|v| v.to_bits()));
            words.extend(
                link.joint()
                    .data
                    .motors
                    .iter()
                    .map(|motor| motor.impulse.to_bits()),
            );
        }
        words
    }
}

fn impulse(value: MultibodyMotorImpulseValue) -> f64 {
    match value {
        MultibodyMotorImpulseValue::Revolute {
            applied_impulse_nm_s,
            ..
        } => applied_impulse_nm_s,
        MultibodyMotorImpulseValue::Prismatic {
            applied_impulse_n_s,
            ..
        } => applied_impulse_n_s,
    }
}

// Preserve IEEE bits, integer counts, variant and Option presence, and order.
// Derived PartialEq alone would treat signed zero as equal.
fn report_words(report: Option<&MultibodyMotorImpulseTelemetry>) -> Vec<u64> {
    fn optional(words: &mut Vec<u64>, value: Option<f64>) {
        words.push(u64::from(value.is_some()));
        if let Some(value) = value {
            words.push(value.to_bits());
        }
    }

    fn value(words: &mut Vec<u64>, value: MultibodyMotorImpulseValue) {
        match value {
            MultibodyMotorImpulseValue::Revolute {
                applied_impulse_nm_s,
                effective_effort_ceiling_nm,
                rounded_impulse_bound_nm_s,
            } => {
                words.extend([0, applied_impulse_nm_s.to_bits()]);
                optional(words, effective_effort_ceiling_nm);
                optional(words, rounded_impulse_bound_nm_s);
            }
            MultibodyMotorImpulseValue::Prismatic {
                applied_impulse_n_s,
                effective_force_ceiling_n,
                rounded_impulse_bound_n_s,
            } => {
                words.extend([1, applied_impulse_n_s.to_bits()]);
                optional(words, effective_force_ceiling_n);
                optional(words, rounded_impulse_bound_n_s);
            }
        }
    }

    let Some(report) = report else {
        return vec![0];
    };
    let mut words = vec![1, report.numerical_outer_interval_s.to_bits()];
    words.push(report.ccd_partition_durations_s.len() as u64);
    words.extend(
        report
            .ccd_partition_durations_s
            .iter()
            .map(|dt| dt.to_bits()),
    );
    words.push(report.motors.len() as u64);
    for record in &report.motors {
        let (index, generation) = record.joint_handle.into_raw_parts();
        words.extend([
            u64::from(index),
            u64::from(generation),
            record.link_id as u64,
            record.joint_dof_id as u64,
            record.axis as u64,
        ]);
        value(&mut words, record.impulse);
        words.extend([
            record.observed_solver_interval_s.to_bits(),
            record.observed_solver_slice_count as u64,
            u64::from(record.full_step_coverage),
            record.observed_partition_count as u64,
            record.finite_positive_bound_eligible_slice_count as u64,
            record.finite_positive_bound_eligible_duration_s.to_bits(),
            record.finite_positive_bound_active_slice_count as u64,
            record.finite_positive_bound_active_duration_s.to_bits(),
            record.slices.len() as u64,
        ]);
        for slice in &record.slices {
            words.extend([
                slice.ccd_partition_index as u64,
                slice.internal_slice_index as u64,
                slice.expected_internal_slice_count as u64,
                slice.duration_s.to_bits(),
                u64::from(slice.finite_positive_bound_eligible),
                u64::from(slice.finite_positive_bound_active),
            ]);
            value(&mut words, slice.impulse);
        }
    }
    words
}

#[test]
fn motor_impulse_sign_units_and_momentum_match_both_joint_types() {
    for angular in [false, true] {
        for sign in [-1.0, 1.0] {
            for (base, extra) in [(1, 0), (32, 0), (4, 3)] {
                let mut fixture = Fixture::new(angular, sign * 0.4, 2.0);
                fixture
                    .pipeline
                    .set_multibody_motor_impulse_telemetry_enabled(true);
                fixture.parameters.num_solver_iterations = NonZeroUsize::new(base).unwrap();
                fixture.bodies[fixture.child].set_additional_solver_iterations(extra);
                let expected_slices = base + extra;
                fixture.step();
                let record = fixture.record();
                assert_eq!(record.joint_handle, fixture.joint);
                assert_eq!(record.joint_dof_id, 0);
                assert_eq!(
                    record.axis,
                    if angular {
                        JointAxis::AngX
                    } else {
                        JointAxis::LinX
                    }
                );
                assert!(record.full_step_coverage);
                assert_eq!(record.observed_solver_slice_count, expected_slices);
                assert_eq!(
                    record.finite_positive_bound_eligible_slice_count,
                    expected_slices
                );
                assert_eq!(
                    record.finite_positive_bound_active_slice_count,
                    expected_slices
                );
                let value = impulse(record.impulse);
                assert!(value * f64::from(sign) > 0.0);
                approx::assert_relative_eq!(
                    value,
                    f64::from(sign) * 2.0 * record.observed_solver_interval_s,
                    epsilon = 1.0e-10,
                );
                let body = &fixture.bodies[fixture.child];
                let momentum = if angular {
                    // A radius-0.1 uniform solid sphere has inertia 2 m r^2 / 5.
                    (2.0 / 5.0)
                        * f64::from(body.mass())
                        * 0.1_f64.powi(2)
                        * f64::from(fixture.joint_velocity())
                } else {
                    f64::from(body.mass()) * f64::from(fixture.joint_velocity())
                };
                approx::assert_relative_eq!(
                    value,
                    momentum,
                    epsilon = 1.0e-8,
                    max_relative = 2.0e-5
                );
                for slice in &record.slices {
                    assert_eq!(slice.expected_internal_slice_count, expected_slices);
                }
            }
        }
    }
}

#[test]
fn zero_capacity_is_measured_zero_and_inactive_or_disabled_is_unavailable() {
    let mut fixture = Fixture::new(true, 0.4, 0.0);
    assert!(fixture
        .pipeline
        .completed_multibody_motor_impulses()
        .is_none());
    fixture
        .pipeline
        .set_multibody_motor_impulse_telemetry_enabled(true);
    assert!(fixture
        .pipeline
        .completed_multibody_motor_impulses()
        .is_none());
    fixture.step();
    assert_eq!(impulse(fixture.record().impulse), 0.0);
    assert_eq!(
        fixture.record().finite_positive_bound_eligible_slice_count,
        0
    );
    assert_eq!(fixture.record().finite_positive_bound_active_slice_count, 0);
    let (body, link) = fixture.multibody_joints.get_mut(fixture.joint).unwrap();
    body.link_mut(link).unwrap().joint.data.motor_axes = crate::dynamics::JointAxesMask::empty();
    fixture.step();
    assert!(fixture
        .pipeline
        .completed_multibody_motor_impulses()
        .unwrap()
        .motors
        .is_empty());
    fixture
        .pipeline
        .set_multibody_motor_impulse_telemetry_enabled(false);
    assert!(fixture
        .pipeline
        .completed_multibody_motor_impulses()
        .is_none());
    fixture.step();
    assert!(fixture
        .pipeline
        .completed_multibody_motor_impulses()
        .is_none());
}

#[test]
fn infinite_native_capacity_is_unknown_and_not_a_positive_finite_bound() {
    let mut fixture = Fixture::new(false, 0.01, Real::INFINITY);
    // For mass 1 and dt .00025, cfm_gain * mass is less than 1 at this damping.
    // This analytically stable qualification fixture is not a G1 controller gain.
    let (body, id) = fixture.multibody_joints.get_mut(fixture.joint).unwrap();
    body.link_mut(id).unwrap().joint.data.motors[JointAxis::LinX as usize].damping = 10_000.0;
    fixture
        .pipeline
        .set_multibody_motor_impulse_telemetry_enabled(true);
    fixture.step();
    let record = fixture.record();
    assert!(record.full_step_coverage);
    assert!(fixture.joint_velocity().is_finite());
    assert_eq!(record.finite_positive_bound_eligible_slice_count, 0);
    assert!(matches!(
        record.impulse,
        MultibodyMotorImpulseValue::Prismatic {
            effective_force_ceiling_n: None,
            rounded_impulse_bound_n_s: None,
            ..
        }
    ));
    assert!(impulse(record.impulse).is_finite());
}

#[test]
fn hard_limit_impulse_is_not_reported_as_motor_impulse() {
    let mut fixture = Fixture::new(false, 0.4, 0.0);
    fixture
        .pipeline
        .set_multibody_motor_impulse_telemetry_enabled(true);
    let (body, link) = fixture.multibody_joints.get_mut(fixture.joint).unwrap();
    body.link_mut(link)
        .unwrap()
        .joint
        .data
        .set_limits(JointAxis::LinX, [0.0, 0.0]);
    fixture.bodies[fixture.child].add_force(Vector::x() * 1000.0, true);
    fixture.step();
    let record = fixture.record();
    assert!(record.full_step_coverage);
    assert_eq!(record.observed_solver_slice_count, 4);
    assert_eq!(impulse(record.impulse), 0.0);
    assert!(fixture.joint_velocity().abs() < 0.01);
}

#[test]
fn nonfinite_uncapped_solver_slices_do_not_become_full_interval_measurements() {
    // Retain the original uncapped fixture: its native ForceBased recurrence
    // overflows at these gains. The observer must not invent missing impulses.
    let mut fixture = Fixture::new(false, 0.01, Real::INFINITY);
    fixture
        .pipeline
        .set_multibody_motor_impulse_telemetry_enabled(true);
    fixture.step();
    let record = fixture.record();
    assert!(
        !fixture.joint_velocity().is_finite(),
        "expected native nonfinite corner, record={record:?}"
    );
    assert!(!record.full_step_coverage);
    assert!(record.observed_solver_slice_count < 4);
    assert!(record
        .slices
        .iter()
        .all(|slice| impulse(slice.impulse).is_finite()));
    assert_eq!(record.finite_positive_bound_eligible_slice_count, 0);
}

#[test]
fn multiple_motor_islands_have_stable_order_and_exact_parallel_replay() {
    let mut on = Fixture::new(true, 0.4, 2.0);
    let mut off = Fixture::new(true, 0.4, 2.0);
    let mut fresh = Fixture::new(true, 0.4, 2.0);
    let mut secondary = Vec::new();
    for fixture in [&mut on, &mut off, &mut fresh] {
        fixture.parameters.min_island_size = 1;
        let parent = fixture
            .bodies
            .insert(RigidBodyBuilder::fixed().translation(Vector::z() * 20.0));
        let child = fixture.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::z() * 20.0)
                .can_sleep(false),
        );
        fixture.bodies[child].set_additional_solver_iterations(3);
        fixture.colliders.insert_with_parent(
            ColliderBuilder::ball(0.1).mass(1.0),
            child,
            &mut fixture.bodies,
        );
        let joint = fixture
            .multibody_joints
            .insert(
                parent,
                child,
                PrismaticJointBuilder::new(Vector::x_axis())
                    .motor_model(MotorModel::ForceBased)
                    .motor_position(-0.4, 100.0, 1.0)
                    .motor_max_force(3.0),
                true,
            )
            .unwrap();
        secondary.push(joint);
    }
    on.pipeline
        .set_multibody_motor_impulse_telemetry_enabled(true);
    fresh
        .pipeline
        .set_multibody_motor_impulse_telemetry_enabled(true);
    for _ in 0..8 {
        on.step();
        off.step();
        fresh.step();
        assert!(
            on.islands.num_islands() >= 2,
            "the fixture must have independent active motor islands"
        );
        assert_eq!(on.state_bits(), off.state_bits());
        assert_eq!(on.state_bits(), fresh.state_bits());
        let report = on.pipeline.completed_multibody_motor_impulses().unwrap();
        assert_eq!(
            report_words(Some(report)),
            report_words(fresh.pipeline.completed_multibody_motor_impulses()),
        );
        assert_eq!(report.motors.len(), 2);
        assert!(
            report.motors[0].joint_handle.into_raw_parts()
                < report.motors[1].joint_handle.into_raw_parts()
        );
        for record in &report.motors {
            assert!(record.full_step_coverage);
            let (count, force) = if record.joint_handle == secondary[0] {
                (7, -3.0)
            } else {
                (4, 2.0)
            };
            assert_eq!(record.observed_solver_slice_count, count);
            approx::assert_relative_eq!(
                impulse(record.impulse),
                force * record.observed_solver_interval_s,
                epsilon = 1.0e-9
            );
        }
    }
}

#[test]
fn actual_ccd_partitions_and_small_steps_are_aggregated_without_pgs_double_counting() {
    let mut fixture = Fixture::new(false, 0.4, 2.0);
    fixture.add_ccd_impact();
    fixture
        .pipeline
        .set_multibody_motor_impulse_telemetry_enabled(true);
    let before_velocity = fixture.joint_velocity();
    fixture.step();
    let report = fixture
        .pipeline
        .completed_multibody_motor_impulses()
        .unwrap();
    assert!(
        report.ccd_partition_durations_s.len() > 1,
        "the fixture must actually split dt"
    );
    assert_eq!(
        report.ccd_partition_durations_s.len(),
        fixture.pipeline.counters.ccd.num_substeps
    );
    let record = fixture.record();
    assert!(record.full_step_coverage);
    assert_eq!(
        record.observed_partition_count,
        report.ccd_partition_durations_s.len()
    );
    assert_eq!(
        record.observed_solver_slice_count,
        4 * report.ccd_partition_durations_s.len()
    );
    let native_sum: f64 = record.slices.iter().map(|slice| slice.duration_s).sum();
    assert_eq!(
        native_sum.to_bits(),
        record.observed_solver_interval_s.to_bits()
    );
    approx::assert_relative_eq!(impulse(record.impulse), 2.0 * native_sum, epsilon = 1.0e-9,);
    let momentum = f64::from(fixture.bodies[fixture.child].mass())
        * f64::from(fixture.joint_velocity() - before_velocity);
    approx::assert_relative_eq!(
        impulse(record.impulse),
        momentum,
        epsilon = 1.0e-8,
        max_relative = 2.0e-5
    );
}

#[test]
fn enabled_disabled_and_fresh_repeats_preserve_literal_physics_state() {
    for angular in [true, false] {
        for ccd in [false, true] {
            let mut on = Fixture::new(angular, 0.4, 2.0);
            let mut off = Fixture::new(angular, 0.4, 2.0);
            let mut fresh = Fixture::new(angular, 0.4, 2.0);
            if ccd {
                on.add_ccd_impact();
                off.add_ccd_impact();
                fresh.add_ccd_impact();
            }
            on.pipeline
                .set_multibody_motor_impulse_telemetry_enabled(true);
            fresh
                .pipeline
                .set_multibody_motor_impulse_telemetry_enabled(true);
            for _ in 0..16 {
                on.step();
                off.step();
                fresh.step();
                assert_eq!(on.state_bits(), off.state_bits());
                assert_eq!(on.state_bits(), fresh.state_bits());
                assert_eq!(
                    report_words(on.pipeline.completed_multibody_motor_impulses()),
                    report_words(fresh.pipeline.completed_multibody_motor_impulses()),
                );
                #[cfg(feature = "serde-serialize")]
                {
                    assert_eq!(
                        bincode::serialize(&on.bodies).unwrap(),
                        bincode::serialize(&off.bodies).unwrap()
                    );
                    assert_eq!(
                        bincode::serialize(&on.multibody_joints).unwrap(),
                        bincode::serialize(&off.multibody_joints).unwrap()
                    );
                }
            }
        }
    }
}

fn sample(partition: usize, ordinal: usize, expected: usize) -> InternalMotorImpulseSample {
    InternalMotorImpulseSample {
        joint_handle: MultibodyJointHandle::from_raw_parts(7, 2),
        link_id: 1,
        joint_dof_id: 0,
        axis: JointAxis::AngX,
        slice: MultibodyMotorImpulseSlice {
            ccd_partition_index: partition,
            internal_slice_index: ordinal,
            expected_internal_slice_count: expected,
            duration_s: f64::from(0.1_f32),
            impulse: MultibodyMotorImpulseValue::from_solver(JointAxis::AngX, -0.2, 2.0, 0.2),
            finite_positive_bound_eligible: true,
            finite_positive_bound_active: true,
        },
    }
}

#[test]
fn integer_coverage_rejects_missing_duplicate_or_inconsistent_ordinals() {
    for ordinals in [
        vec![(0, 0, 2), (0, 1, 2)],
        vec![(0, 0, 2)],
        vec![(0, 0, 2), (0, 0, 2)],
        vec![(0, 0, 2), (0, 1, 3)],
    ] {
        let mut report = MultibodyMotorImpulseTelemetry::new(0.2);
        report.ccd_partition_durations_s.push(f64::from(0.2_f32));
        let valid = ordinals == vec![(0, 0, 2), (0, 1, 2)];
        report.append(
            ordinals
                .into_iter()
                .map(|(p, i, n)| sample(p, i, n))
                .collect(),
        );
        report.finalize();
        assert_eq!(report.motors[0].full_step_coverage, valid);
    }
    let mut report = MultibodyMotorImpulseTelemetry::new(0.4);
    report.ccd_partition_durations_s = vec![0.2, 0.2];
    report.append(vec![sample(0, 0, 2), sample(0, 1, 2)]);
    report.finalize();
    assert!(!report.motors[0].full_step_coverage);
}

#[test]
fn telemetry_word_encoding_distinguishes_signed_zero_and_optional_capacity() {
    let mut first = MultibodyMotorImpulseTelemetry::new(0.0);
    first.ccd_partition_durations_s.push(0.0);
    first.append(vec![sample(0, 0, 1)]);
    first.finalize();
    let mut second = first.clone();
    second.numerical_outer_interval_s = -0.0;
    assert_eq!(first, second);
    assert_ne!(report_words(Some(&first)), report_words(Some(&second)));

    second = first.clone();
    second.motors[0].slices[0].duration_s = -first.motors[0].slices[0].duration_s;
    assert_ne!(report_words(Some(&first)), report_words(Some(&second)));
    second = first.clone();
    second.motors[0].impulse = MultibodyMotorImpulseValue::Revolute {
        applied_impulse_nm_s: 0.0,
        effective_effort_ceiling_nm: None,
        rounded_impulse_bound_nm_s: Some(0.0),
    };
    let mut third = second.clone();
    third.motors[0].impulse = MultibodyMotorImpulseValue::Revolute {
        applied_impulse_nm_s: -0.0,
        effective_effort_ceiling_nm: Some(0.0),
        rounded_impulse_bound_nm_s: Some(0.0),
    };
    assert_ne!(report_words(Some(&second)), report_words(Some(&third)));
    assert_ne!(
        report_words(None),
        report_words(Some(&MultibodyMotorImpulseTelemetry::new(0.0)))
    );
}
