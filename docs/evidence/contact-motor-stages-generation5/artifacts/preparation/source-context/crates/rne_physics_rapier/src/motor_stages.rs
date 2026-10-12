//! Read-only, source-bound native checkpoints for optional motor stage traces.

use super::RapierBackend;
use rapier3d::pipeline::MultibodyMotorStageTelemetry;
use rne_physics::{PhysicsError, PhysicsWorldId};
use serde::{de::DeserializeOwned, Deserialize, Serialize};
use std::collections::BTreeMap;

/// Kind of individually serialized native object, in checkpoint ordering.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum MotorStageNativeRecordKind {
    /// Native rigid body, including its source-bound private serde fields.
    Body,
    /// Native collider and shape.
    Collider,
    /// One unique native multibody, including generalized coordinates.
    Multibody,
    /// Native impulse joint, if present.
    ImpulseJoint,
}

/// One exact native object encoded with postcard raw wire format version 1.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MotorStageNativeRecord {
    /// Type of native object.
    pub kind: MotorStageNativeRecordKind,
    /// Native object index and generation, without truncation.
    pub handle_words: [u32; 2],
    /// Exact-consumption, decode/reencode-checked native bytes.
    pub postcard_bytes: Vec<u8>,
}

/// Source-bound integration settings with native f32 words preserved.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MotorStageIntegrationParameters {
    /// Outer timestep in seconds as an IEEE word.
    pub dt_word: u32,
    /// Minimum CCD timestep in seconds as an IEEE word.
    pub min_ccd_dt_word: u32,
    /// Contact damping ratio as an IEEE word.
    pub contact_damping_ratio_word: u32,
    /// Contact natural frequency as an IEEE word.
    pub contact_natural_frequency_word: u32,
    /// Joint natural frequency as an IEEE word.
    pub joint_natural_frequency_word: u32,
    /// Joint damping ratio as an IEEE word.
    pub joint_damping_ratio_word: u32,
    /// Contact warmstart coefficient as an IEEE word.
    pub warmstart_coefficient_word: u32,
    /// Length unit as an IEEE word.
    pub length_unit_word: u32,
    /// Normalized allowed linear error as an IEEE word.
    pub normalized_allowed_linear_error_word: u32,
    /// Normalized maximum corrective velocity as an IEEE word.
    pub normalized_max_corrective_velocity_word: u32,
    /// Normalized prediction distance as an IEEE word.
    pub normalized_prediction_distance_word: u32,
    /// Configured small-step count before per-island additional iterations.
    pub num_solver_iterations: usize,
    /// Additional final-slice contact friction iterations.
    pub num_additional_friction_iterations: usize,
    /// PGS iterations per slice.
    pub num_internal_pgs_iterations: usize,
    /// Stabilization iterations per slice.
    pub num_internal_stabilization_iterations: usize,
    /// Minimum dynamic island size.
    pub min_island_size: usize,
    /// Configured maximum CCD partitions.
    pub max_ccd_substeps: usize,
}

/// Stable ECS-to-native multibody joint identity, sorted by ECS entity bits.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MotorStageJointBinding {
    /// Complete ECS entity identifier, including generation.
    pub entity_bits: u64,
    /// Native joint handle index and generation.
    pub joint_handle_words: [u32; 2],
    /// Native multibody arena index and generation.
    pub multibody_handle_words: [u32; 2],
    /// Actual child link index inside that multibody.
    pub link_index: usize,
}

/// Exact individually serialized native state for stage-observer replay checks.
///
/// This contains bodies, colliders, unique multibodies, impulse joints, gravity,
/// integration settings and ECS joint bindings. It excludes full set metadata,
/// broad/narrow phase, private pipeline caches and observer buffers. Equality
/// therefore establishes equality of this declared scope, not all hidden state.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MotorStageNativeCheckpoint {
    /// Version of this field and serialization-order contract.
    pub schema_version: u32,
    /// Width of every native scalar (the RNE Rapier backend uses f32).
    pub native_scalar_bits: u8,
    /// Native gravity x/y/z words, in m/s².
    pub gravity_ieee_words: [u32; 3],
    /// Explicit runtime integration settings.
    pub integration_parameters: MotorStageIntegrationParameters,
    /// Exact native integration parameter serde bytes.
    pub integration_parameters_postcard_bytes: Vec<u8>,
    /// Objects grouped by kind, then sorted by native index and generation.
    pub records: Vec<MotorStageNativeRecord>,
    /// Stable identities used to select traced ECS joints.
    pub bindings: Vec<MotorStageJointBinding>,
}

/// A checkpoint could not be read or encoded without losing native state.
#[derive(Clone, Debug, PartialEq)]
pub enum MotorStageCheckpointError {
    /// The requested physics world does not exist.
    Physics(PhysicsError),
    /// Encoding, exact-consumption decoding or reencoding failed.
    Encoding(String),
    /// An ECS joint binding did not resolve to a live native joint.
    InvalidBinding(u64),
}

impl std::fmt::Display for MotorStageCheckpointError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Physics(error) => write!(f, "{error}"),
            Self::Encoding(error) => write!(f, "native checkpoint encoding failed: {error}"),
            Self::InvalidBinding(entity) => {
                write!(f, "unresolved native joint for entity {entity}")
            }
        }
    }
}

impl std::error::Error for MotorStageCheckpointError {}

fn exact_bytes<T: Serialize + DeserializeOwned>(
    value: &T,
) -> Result<Vec<u8>, MotorStageCheckpointError> {
    let encode = |value: &T| {
        postcard::to_allocvec(value)
            .map_err(|error| MotorStageCheckpointError::Encoding(error.to_string()))
    };
    let bytes = encode(value)?;
    let (decoded, remainder): (T, _) = postcard::take_from_bytes(&bytes)
        .map_err(|error| MotorStageCheckpointError::Encoding(error.to_string()))?;
    if !remainder.is_empty() || encode(&decoded)? != bytes {
        return Err(MotorStageCheckpointError::Encoding(
            "native decode/reencode changed bytes or left trailing bytes".into(),
        ));
    }
    Ok(bytes)
}

impl RapierBackend {
    /// Enables or disables ordered native motor stage observation for a world.
    ///
    /// Disabled by default. Toggling invalidates any completed trace. This is
    /// separate from completed-step impulse observation and changes no actuation.
    pub fn configure_multibody_motor_stages(
        &mut self,
        id: PhysicsWorldId,
        enabled: bool,
    ) -> Result<(), PhysicsError> {
        self.world_mut(id)?
            .physics_pipeline
            .set_multibody_motor_stage_telemetry_enabled(enabled);
        Ok(())
    }

    /// Returns the completed world's ordered raw native motor stage trace.
    ///
    /// Returns None while disabled, before a completed step, or after toggling.
    pub fn completed_multibody_motor_stages(
        &self,
        id: PhysicsWorldId,
    ) -> Result<Option<&MultibodyMotorStageTelemetry>, PhysicsError> {
        Ok(self
            .world(id)?
            .physics_pipeline
            .completed_multibody_motor_stages())
    }

    /// Serializes the declared native entity scope without mutating the world.
    ///
    /// Private serde layout is source-bound. Every object's bytes are checked
    /// through exact-consumption native decoding and reencoding before return.
    pub fn motor_stage_native_checkpoint(
        &self,
        id: PhysicsWorldId,
    ) -> Result<MotorStageNativeCheckpoint, MotorStageCheckpointError> {
        let state = self.world(id).map_err(MotorStageCheckpointError::Physics)?;
        let p = &state.integration_parameters;
        let parameters = MotorStageIntegrationParameters {
            dt_word: p.dt.to_bits(),
            min_ccd_dt_word: p.min_ccd_dt.to_bits(),
            contact_damping_ratio_word: p.contact_damping_ratio.to_bits(),
            contact_natural_frequency_word: p.contact_natural_frequency.to_bits(),
            joint_natural_frequency_word: p.joint_natural_frequency.to_bits(),
            joint_damping_ratio_word: p.joint_damping_ratio.to_bits(),
            warmstart_coefficient_word: p.warmstart_coefficient.to_bits(),
            length_unit_word: p.length_unit.to_bits(),
            normalized_allowed_linear_error_word: p.normalized_allowed_linear_error.to_bits(),
            normalized_max_corrective_velocity_word: p.normalized_max_corrective_velocity.to_bits(),
            normalized_prediction_distance_word: p.normalized_prediction_distance.to_bits(),
            num_solver_iterations: p.num_solver_iterations.get(),
            num_additional_friction_iterations: p.num_additional_friction_iterations,
            num_internal_pgs_iterations: p.num_internal_pgs_iterations,
            num_internal_stabilization_iterations: p.num_internal_stabilization_iterations,
            min_island_size: p.min_island_size,
            max_ccd_substeps: p.max_ccd_substeps,
        };
        let mut records = Vec::new();
        macro_rules! record_set {
            ($set:expr, $kind:ident) => {{
                let mut values: Vec<_> = $set.iter().collect();
                values.sort_by_key(|(handle, _)| handle.into_raw_parts());
                for (handle, value) in values {
                    let (index, generation) = handle.into_raw_parts();
                    records.push(MotorStageNativeRecord {
                        kind: MotorStageNativeRecordKind::$kind,
                        handle_words: [index, generation],
                        postcard_bytes: exact_bytes(value)?,
                    });
                }
            }};
        }
        record_set!(state.bodies, Body);
        record_set!(state.colliders, Collider);
        let mut multibodies = BTreeMap::new();
        let mut joints = BTreeMap::new();
        for (handle, link, multibody, _) in state.multibody_joints.iter() {
            let key = link.multibody.0.into_raw_parts();
            multibodies.entry(key).or_insert(multibody);
            joints.insert(handle.into_raw_parts(), (key, link.id));
        }
        for ((index, generation), multibody) in multibodies {
            records.push(MotorStageNativeRecord {
                kind: MotorStageNativeRecordKind::Multibody,
                handle_words: [index, generation],
                postcard_bytes: exact_bytes(multibody)?,
            });
        }
        record_set!(state.impulse_joints, ImpulseJoint);
        let mut entities: Vec<_> = state.entity_to_multibody_joint.iter().collect();
        entities.sort_by_key(|(entity, _)| entity.to_bits());
        let mut bindings = Vec::new();
        for (entity, handle) in entities {
            let (index, generation) = handle.into_raw_parts();
            let &((mb_index, mb_generation), link_index) = joints
                .get(&(index, generation))
                .ok_or(MotorStageCheckpointError::InvalidBinding(entity.to_bits()))?;
            bindings.push(MotorStageJointBinding {
                entity_bits: entity.to_bits(),
                joint_handle_words: [index, generation],
                multibody_handle_words: [mb_index, mb_generation],
                link_index,
            });
        }
        Ok(MotorStageNativeCheckpoint {
            schema_version: 1,
            native_scalar_bits: 32,
            gravity_ieee_words: state.gravity.map(f32::to_bits).into(),
            integration_parameters: parameters,
            integration_parameters_postcard_bytes: exact_bytes(p)?,
            records,
            bindings,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_core::SimDuration;
    use rne_physics::{PhysicsBackend, PhysicsWorldDesc};

    #[test]
    fn stage_queries_reject_unknown_worlds_and_clear_completed_data() {
        let mut backend = RapierBackend::new();
        let missing = PhysicsWorldId(77);
        assert_eq!(
            backend.configure_multibody_motor_stages(missing, true),
            Err(PhysicsError::WorldNotFound)
        );
        assert_eq!(
            backend.completed_multibody_motor_stages(missing),
            Err(PhysicsError::WorldNotFound)
        );
        assert_eq!(
            backend.motor_stage_native_checkpoint(missing),
            Err(MotorStageCheckpointError::Physics(
                PhysicsError::WorldNotFound
            ))
        );
        let id = backend.create_world(PhysicsWorldDesc::default()).unwrap();
        let before = backend.motor_stage_native_checkpoint(id).unwrap();
        assert_eq!(before, backend.motor_stage_native_checkpoint(id).unwrap());
        assert!(backend
            .completed_multibody_motor_stages(id)
            .unwrap()
            .is_none());
        backend.configure_multibody_motor_stages(id, true).unwrap();
        assert_eq!(before, backend.motor_stage_native_checkpoint(id).unwrap());
        backend
            .step(id, SimDuration::from_hertz(rne_math::Hertz::new(500.0)))
            .unwrap();
        assert!(backend
            .completed_multibody_motor_stages(id)
            .unwrap()
            .is_some());
        let completed = backend.motor_stage_native_checkpoint(id).unwrap();
        backend.configure_multibody_motor_stages(id, false).unwrap();
        assert!(backend
            .completed_multibody_motor_stages(id)
            .unwrap()
            .is_none());
        assert_eq!(
            completed,
            backend.motor_stage_native_checkpoint(id).unwrap()
        );
    }

    #[test]
    fn native_codec_preserves_ieee_edge_words() {
        let values = [
            0.0_f32,
            -0.0,
            f32::from_bits(1),
            f32::INFINITY,
            f32::from_bits(0x7fc0_0042),
        ];
        let bytes = exact_bytes(&values).unwrap();
        let (decoded, remainder): ([f32; 5], _) = postcard::take_from_bytes(&bytes).unwrap();
        assert!(remainder.is_empty());
        assert_eq!(values.map(f32::to_bits), decoded.map(f32::to_bits));
    }
}
