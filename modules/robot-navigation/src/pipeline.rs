//! The default classical navigation pipeline.

use robot_foresight::HybridPipeline;

use crate::{
    DurationCritic, ForecastCertifier, NavProblem, PathLengthCritic, RiskCritic, ShortcutRefiner,
    SpaceTimeAstar, StraightLineProposer, Trajectory3,
};

/// Space-time A* and straight-line proposers, shortcut refinement, cost =
/// duration + 0.1 x length + 5 x integrated risk, gated by
/// [`ForecastCertifier`]. Add learned stages with
/// [`HybridPipeline::with_proposer`] or [`HybridPipeline::with_critic`].
#[must_use]
pub fn default_pipeline(seed: u64) -> HybridPipeline<NavProblem, Trajectory3> {
    HybridPipeline::new(seed)
        .with_proposer(StraightLineProposer::default())
        .with_proposer(SpaceTimeAstar::default())
        .with_refiner(ShortcutRefiner::default())
        .with_critic(1.0, DurationCritic)
        .with_critic(0.1, PathLengthCritic)
        .with_critic(5.0, RiskCritic::default())
        .with_certifier(ForecastCertifier::default())
}
