//! Structure for combining the various physics components to perform an actual simulation.

pub use collision_pipeline::CollisionPipeline;
pub use event_handler::{ActiveEvents, ChannelEventCollector, EventHandler};
pub(crate) use multibody_motor_impulse::InternalMotorImpulseSample;
pub use multibody_motor_impulse::{
    MultibodyMotorImpulseRecord, MultibodyMotorImpulseSlice, MultibodyMotorImpulseTelemetry,
    MultibodyMotorImpulseValue,
};
pub(crate) use multibody_motor_stage::{
    motor_stage_event, motor_stage_word, InternalMotorStageBinding, InternalMotorStageContext,
};
pub use multibody_motor_stage::{
    MultibodyMotorStage, MultibodyMotorStageEvent, MultibodyMotorStageRow,
    MultibodyMotorStageTelemetry,
};
pub use physics_hooks::{ActiveHooks, ContactModificationContext, PairFilterContext, PhysicsHooks};
pub use physics_pipeline::PhysicsPipeline;
pub use query_pipeline::{
    generators as query_pipeline_generators, QueryFilter, QueryFilterFlags, QueryPipeline,
};

#[cfg(feature = "debug-render")]
pub use self::debug_render_pipeline::{
    DebugColor, DebugRenderBackend, DebugRenderMode, DebugRenderObject, DebugRenderPipeline,
    DebugRenderStyle,
};

mod collision_pipeline;
mod event_handler;
mod multibody_motor_impulse;
mod multibody_motor_stage;
mod physics_hooks;
mod physics_pipeline;
mod query_pipeline;
mod user_changes;

#[cfg(feature = "debug-render")]
mod debug_render_pipeline;
