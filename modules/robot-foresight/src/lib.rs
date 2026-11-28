//! Future world states and a hybrid learned/classical decision pipeline.
//!
//! `robot_foresight` is the shared core of the `robot_navigation` and
//! `robot_manipulation` modules. It gives both of them two abstractions that a
//! map-and-costmap navigation stack does not have:
//!
//! 1. **The world as a function of time.** A [`WorldForecast`] answers
//!    "how likely is this sphere to be hit at time `t`?" for predicted agents
//!    with several weighted futures ([`TrajectoryMode`]) whose uncertainty grows
//!    with the lookahead. [`Forecaster`]s turn observed [`AgentTrack`]s into a
//!    [`ForecastSet`]: [`ConstantVelocityForecaster`] is classical,
//!    [`LearnedForecaster`] wraps any [`InferenceModel`], and
//!    [`BlendedForecaster`] mixes the two so a learned forecast never removes
//!    the physically plausible one.
//! 2. **Learned and classical algorithms as peers.** A [`HybridPipeline`]
//!    gathers candidates from any mix of [`Proposer`]s, improves them with
//!    [`Refiner`]s, prices them with weighted [`Critic`]s and gates them with
//!    [`Certifier`]s. Every candidate carries its [`Provenance`]; every
//!    decision carries a full trace ([`Decision`]), including proposers that
//!    failed. Certifiers are the hard safety layer: a learned proposal is
//!    executed only if it passes the same checks as a classical one, and a
//!    classical proposer is the fallback when it does not.
//!
//! The crate depends on no RNE crate, renderer, physics backend or ROS 2, so
//! it can be split into its own repository unchanged. Everything is
//! deterministic: randomness comes from a seeded [`SplitMix64`], candidates are
//! evaluated in proposer order and ties are broken by that order.

#![deny(missing_docs)]

mod error;
mod forecast;
mod forecasters;
mod hybrid;
mod model;
mod rng;

pub use error::ForesightError;
pub use forecast::{AgentForecast, ForecastSet, TimedState, TrajectoryMode, WorldForecast};
pub use forecasters::{
    AgentTrack, BlendedForecaster, ConstantVelocityForecaster, Forecaster, LearnedForecaster,
};
pub use hybrid::{
    Candidate, Certificate, Certifier, CostTerm, Critic, Decision, Evaluated, HybridPipeline,
    ProposalContext, Proposer, ProposerFailure, Provenance, Refiner, SelectionPolicy,
};
pub use model::{
    check_output, Activation, DenseLayer, FnModel, InferenceModel, MlpModel, ModelSpec,
};
pub use rng::SplitMix64;

/// Re-exported vector type used for positions (metres) and velocities (m/s).
pub use glam::DVec3;
