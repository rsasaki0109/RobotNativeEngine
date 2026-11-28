//! Arm motion and grasp planning around a world that keeps moving.
//!
//! A motion planner that treats the workspace as static has two options
//! when a person reaches into it: stop, or replan from scratch.
//! `robot_manipulation` plans against the person's *forecast* instead:
//!
//! - [`SerialChain`] is a DH revolute arm with forward kinematics, link
//!   collision spheres and damped-least-squares position IK.
//! - [`MotionProblem`] pairs a static [`ManipScene`] with a
//!   [`robot_foresight::ForecastSet`] of predicted agents (hands, people,
//!   other arms).
//! - [`time_scale_path`] is path-velocity decomposition: any geometric
//!   joint path is retimed by a search over `(path position, time)`, so the
//!   arm slows, waits and resumes around the predicted motion. Every motion
//!   proposer reuses it, classical or learned.
//! - Motion proposers ([`DirectJointProposer`], [`RrtConnectProposer`],
//!   [`LearnedViaPointProposer`]) and grasp stages
//!   ([`AntipodalGraspSampler`], [`AntipodalQualityCritic`],
//!   [`LearnedGraspScorer`], [`ReachabilityCertifier`]) plug into
//!   [`robot_foresight::HybridPipeline`], gated by the same certifiers
//!   whatever produced them.

#![deny(missing_docs)]

mod chain;
mod grasp;
mod motion;
mod pipeline;
mod problem;
mod scene;
mod timing;
mod trajectory;

pub use chain::{DhParams, RevoluteJoint, SerialChain};
pub use grasp::{
    grasp_features, AntipodalGraspSampler, AntipodalQualityCritic, GraspCandidate, GraspProblem,
    LearnedGraspScorer, PointCloud, ReachabilityCertifier, GRASP_FEATURES,
};
pub use motion::{
    DirectJointProposer, JointPathLengthCritic, LearnedViaPointProposer, MotionCertifier,
    MotionDurationCritic, MotionRiskCritic, RrtConnectProposer,
};
pub use pipeline::{default_grasp_pipeline, default_motion_pipeline};
pub use problem::MotionProblem;
pub use scene::{Aabb, ManipScene, Sphere};
pub use timing::{time_scale_path, TimeScaling};
pub use trajectory::{JointTrajectory, JointWaypoint};
