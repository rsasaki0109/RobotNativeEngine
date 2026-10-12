//! Rapier physics backend for Robot Native Engine.

#![deny(missing_docs)]

pub mod backend;
mod convert;

pub use backend::{step_physics, RapierBackend};
#[cfg(feature = "motor-stage-diagnostics")]
pub use backend::{
    MotorStageCheckpointError, MotorStageIntegrationParameters, MotorStageJointBinding,
    MotorStageNativeCheckpoint, MotorStageNativeRecord, MotorStageNativeRecordKind,
};
#[cfg(feature = "motor-stage-diagnostics")]
pub use rapier3d::pipeline::{
    MultibodyMotorStage, MultibodyMotorStageEvent, MultibodyMotorStageRow,
    MultibodyMotorStageTelemetry,
};
