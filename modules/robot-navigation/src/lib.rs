//! 3D navigation in space *and* time, over a forecast of the world.
//!
//! A 2D costmap navigator plans on a snapshot: moving obstacles are marked
//! where they are now and cleared once they leave. `robot_navigation` plans
//! in `(x, y, z, t)` instead:
//!
//! - The static world is a 3D [`VoxelGrid`] (drones, multi-level buildings,
//!   legged robots on stairs, any robot whose free space is not a plane).
//! - The dynamic world is a [`robot_foresight::ForecastSet`]: multimodal
//!   predicted agents with uncertainty growing over the lookahead, from a
//!   classical, learned or blended [`robot_foresight::Forecaster`].
//! - A solution is a timed [`Trajectory3`], so "wait for the person to pass,
//!   then go" and "fly over the forklift's predicted path" are plans, not
//!   recovery behaviours.
//!
//! Planners are [`robot_foresight::Proposer`]s on a [`NavProblem`], so a
//! learned policy ([`LearnedVelocityProposer`]) and a search
//! ([`SpaceTimeAstar`]) compete in one [`robot_foresight::HybridPipeline`];
//! [`ForecastCertifier`] gates both with the same collision and limit
//! checks. [`default_pipeline`] assembles the classical stages.
//!
//! For robots that only need a plane, [`planar`] carries the classical 2D
//! stack (occupancy grid, costmap, A*, pure pursuit, DWA) ported from RNE.

#![deny(missing_docs)]

mod certify;
mod critics;
mod pipeline;
pub mod planar;
mod problem;
mod proposers;
mod refine;
mod spacetime;
mod trajectory;
mod voxel;

pub use certify::{check_trajectory, ForecastCertifier, TrajectoryCheck};
pub use critics::{
    trajectory_features, DurationCritic, LearnedCostCritic, PathLengthCritic, RiskCritic,
    TRAJECTORY_FEATURES,
};
pub use pipeline::default_pipeline;
pub use problem::{KinematicLimits, NavProblem};
pub use proposers::{
    learned_velocity_features, LearnedVelocityProposer, StraightLineProposer,
    LEARNED_VELOCITY_INPUTS,
};
pub use refine::ShortcutRefiner;
pub use spacetime::SpaceTimeAstar;
pub use trajectory::{Trajectory3, Waypoint3};
pub use voxel::VoxelGrid;
