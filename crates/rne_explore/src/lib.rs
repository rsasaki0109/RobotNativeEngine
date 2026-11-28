//! Deterministic active-SLAM decisions: where a robot with no map and no
//! goals should go next.
//!
//! `rne_explore` reads an [`rne_nav::OccupancyGrid`], the robot's pose and an
//! inflated [`rne_nav::Costmap`], and returns ranked goal poses, or
//! [`Decision::Done`] when nothing reachable is left to explore. It does not
//! plan paths, drive or estimate: `rne_nav` plans and drives, `rne_slam`
//! estimates. See `docs/PLAN_ACTIVE_SLAM.md` and ADR 036.
//!
//! The pipeline, every step deterministic (row-major order, ties broken by
//! cell):
//!
//! 1. [`extract_frontiers`]: free cells next to unknown space, grouped into
//!    8-connected clusters.
//! 2. [`ReachMap`]: path length from the robot to every cell it can reach
//!    through known, non-inscribed space (one Dijkstra pass, not one plan per
//!    frontier).
//! 3. [`information_gain`]: unknown area a range sensor would see from a
//!    candidate goal, by ray casting on the grid.
//! 4. [`Explorer`]: scores each reachable frontier with a [`ScorePolicy`] and
//!    keeps its current goal unless another beats it by a margin.

#![deny(missing_docs)]

mod explorer;
mod frontier;
mod gain;
mod reach;
mod resistance;

pub use explorer::{Candidate, Decision, Explorer, ExplorerConfig, RevisitConfig, ScorePolicy};
pub use frontier::{extract_frontiers, Frontier, FrontierConfig};
pub use gain::{information_gain, SensorModel};
pub use reach::ReachMap;
pub use resistance::effective_resistance_to_end;
