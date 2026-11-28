//! Default classical manipulation pipelines.

use robot_foresight::HybridPipeline;

use crate::{
    AntipodalGraspSampler, AntipodalQualityCritic, DirectJointProposer, GraspCandidate,
    GraspProblem, JointPathLengthCritic, JointTrajectory, MotionCertifier, MotionDurationCritic,
    MotionProblem, MotionRiskCritic, ReachabilityCertifier, RrtConnectProposer,
};

/// Direct and RRT-Connect proposers (both retimed around the forecast);
/// cost = duration + 0.1 x joint path length + 5 x integrated risk; gated by
/// [`MotionCertifier`].
#[must_use]
pub fn default_motion_pipeline(seed: u64) -> HybridPipeline<MotionProblem, JointTrajectory> {
    HybridPipeline::new(seed)
        .with_proposer(DirectJointProposer::default())
        .with_proposer(RrtConnectProposer::default())
        .with_critic(1.0, MotionDurationCritic)
        .with_critic(0.1, JointPathLengthCritic)
        .with_critic(5.0, MotionRiskCritic::default())
        .with_certifier(MotionCertifier::default())
}

/// Antipodal sampling, priced by friction-cone usage, gated by
/// [`ReachabilityCertifier`]. Add a [`crate::LearnedGraspScorer`] with
/// [`HybridPipeline::with_critic`].
#[must_use]
pub fn default_grasp_pipeline(seed: u64) -> HybridPipeline<GraspProblem, GraspCandidate> {
    HybridPipeline::new(seed)
        .with_proposer(AntipodalGraspSampler::default())
        .with_critic(1.0, AntipodalQualityCritic)
        .with_certifier(ReachabilityCertifier::default())
}
