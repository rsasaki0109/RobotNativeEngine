use super::*;
use crate::dynamics::{JointAxis, MotorModel, MultibodyJointHandle};
#[cfg(feature = "serde-serialize")]
use crate::pipeline::{
    MultibodyMotorConstructionAvailability, MultibodyMotorConstructionEndpoint,
    MultibodyMotorConstructionEvent, MultibodyMotorConstructionPhase,
    MultibodyMotorConstructionTelemetry,
};

fn synthetic_record(limits: Option<[Real; 2]>) -> MultibodyMotorConstructionRow {
    let mut rows = Vec::new();
    let nan = Real::from_bits(0x7fc12345);
    let motor = JointMotor {
        target_pos: -0.0,
        target_vel: nan,
        stiffness: Real::INFINITY,
        damping: -0.0,
        max_force: Real::INFINITY,
        model: MotorModel::ForceBased,
        ..JointMotor::default()
    };
    let params = MotorParameters {
        erp_inv_dt: nan,
        cfm_coeff: -0.0,
        cfm_gain: Real::INFINITY,
        target_pos: motor.target_pos,
        target_vel: motor.target_vel,
        max_impulse: Real::INFINITY,
    };
    let row = JointGenericOneBodyConstraint {
        solver_vel2: 3,
        ndofs2: 1,
        j_id2: 0,
        joint_id: usize::MAX,
        impulse_bounds: [-0.0, 0.0],
        ..JointGenericOneBodyConstraint::invalid()
    };
    let jacobians = DVector::from_vec(vec![1.0, -0.0]);
    InternalMotorConstructionRowRecorder {
        rows: &mut rows,
        constraint_index: 7,
        link_id: 2,
        axis: JointAxis::AngX,
    }
    .capture(
        MultibodyJointHandle::from_raw_parts(12, 4),
        &motor,
        &params,
        nan,
        -0.0,
        Real::INFINITY,
        limits,
        -0.0,
        &row,
        &jacobians,
    );
    rows.pop().unwrap()
}

#[test]
fn actual_input_storage_preserves_ieee_payloads_and_identity_without_dynamics() {
    let captured = synthetic_record(Some([-0.0, 0.0]));
    assert_eq!(captured.coordinate_word, 0x7fc12345);
    assert_eq!(captured.target_position_word, 0x80000000);
    assert_eq!(captured.target_velocity_word, 0x7fc12345);
    assert_eq!(captured.stiffness_word, 0x7f800000);
    assert_eq!(captured.damping_word, 0x80000000);
    assert_eq!(captured.duration_word, 0x80000000);
    assert_eq!(captured.inverse_duration_word, 0x7f800000);
    assert_eq!(captured.erp_inverse_duration_word, 0x7fc12345);
    assert_eq!(captured.effective_target_velocity_word, 0x80000000);
    assert_eq!(captured.limits_words, Some([0x80000000, 0]));
    assert_eq!(captured.row.effort_ceiling_word, 0x7f800000);
    assert_eq!(captured.row.impulse_bound_words, [0x80000000, 0]);
    assert_eq!(captured.row.joint_handle.into_raw_parts(), (12, 4));
    assert_eq!(captured.row.link_id, 2);
    assert_eq!(captured.row.constraint_index, 7);
    assert_eq!(captured.row.solver_velocity_offset, 3);
    assert_eq!(captured.row.ndofs2, 1);
    assert_eq!(captured.row.j_id2, 0);
    assert_eq!(captured.row.jacobian_words, [0x3f800000]);
    assert_eq!(captured.row.weighted_jacobian_words, [0x80000000]);
}

#[test]
fn missing_limits_are_absent_not_a_zero_interval() {
    let missing = synthetic_record(None);
    let zero = synthetic_record(Some([-0.0, 0.0]));
    assert_eq!(missing.limits_words, None);
    assert_ne!(missing.limits_words, zero.limits_words);
}

#[cfg(feature = "serde-serialize")]
#[test]
fn companion_schema_and_raw_words_roundtrip_without_float_json() {
    let mut telemetry = MultibodyMotorConstructionTelemetry::new(-0.0);
    telemetry.events.push(MultibodyMotorConstructionEvent {
        ccd_partition_index: 1,
        island_index: 2,
        internal_slice_index: 3,
        expected_internal_slice_count: 4,
        iteration_index: None,
        phase: MultibodyMotorConstructionPhase::AfterRowConstruction,
        native_scalar_bits: 32,
        duration_word: 0x80000000,
        row_cache_slice_index: Some(3),
        availability: MultibodyMotorConstructionAvailability::Available,
        integration_multibody_handle_words: None,
        generalized_velocity_words: vec![0x7fc12345, 0x80000000],
        constructed_rows: vec![synthetic_record(None)],
        endpoints: Vec::new(),
    });
    let json = serde_json::to_value(&telemetry).unwrap();
    assert_eq!(json["schema_version"], 1);
    assert_eq!(json["source_contract"], "InternalUnitMotorConstructionV1");
    assert_eq!(json["events"][0]["phase"], "AfterRowConstruction");
    assert_eq!(json["events"][0]["availability"], "Available");
    assert_eq!(
        json["events"][0]["constructed_rows"][0]["limits_words"],
        serde_json::Value::Null
    );
    assert_eq!(
        json["events"][0]["constructed_rows"][0]["row"]["joint_handle"],
        serde_json::json!({"index":12,"generation":4})
    );
    assert_eq!(
        serde_json::from_value::<MultibodyMotorConstructionTelemetry>(json).unwrap(),
        telemetry
    );
}

#[cfg(feature = "serde-serialize")]
#[test]
fn endpoints_have_distinct_phase_and_actual_multibody_velocity_storage() {
    let captured = synthetic_record(None);
    let endpoint = MultibodyMotorConstructionEndpoint {
        row: captured.row,
        coordinate_word: 0x80000000,
        multibody_velocity_words: vec![0x7fc12345],
    };
    let json = serde_json::to_value(&endpoint).unwrap();
    assert_eq!(json["coordinate_word"], 0x80000000u64);
    assert_eq!(
        json["multibody_velocity_words"],
        serde_json::json!([0x7fc12345u64])
    );
    assert_ne!(
        MultibodyMotorConstructionPhase::BeforePositionIntegration,
        MultibodyMotorConstructionPhase::AfterVelocityWriteback
    );
    assert_ne!(
        MultibodyMotorConstructionAvailability::Uninitialized,
        MultibodyMotorConstructionAvailability::NoSupportedRows
    );
}
