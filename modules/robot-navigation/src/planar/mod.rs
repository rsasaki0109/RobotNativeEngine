//! Planar (2D) navigation: occupancy grids, inflated costmaps, A* global
//! planning, pure pursuit and DWA local control.
//!
//! Ported from `rne_nav` (ADR 037) so that a robot that only needs a plane
//! gets the classical 2D stack from this crate without depending on RNE.
//! Behaviour and tests are unchanged from the source; `Pose2d::to_transform3`
//! became [`Pose2d::to_affine3`] (a `glam::DAffine3`).

pub mod control;
pub mod costmap;
pub mod dwa;
pub mod grid;
pub mod path;
pub mod planner;
pub mod pose2d;

pub use control::{pure_pursuit_follow, FollowResult, PurePursuitConfig, VelocityCommand2d};
pub use costmap::{
    Costmap, CostmapConfig, COST_FREE, COST_INSCRIBED, COST_LETHAL, COST_NO_INFORMATION,
};
pub use dwa::{DwaConfig, DwaError, DwaOutcome, DwaPlanner};
pub use grid::{GridCoord, GridError, OccupancyGrid};
pub use path::{ClosestPoint, Path2d, PathError};
pub use planner::{plan_path, GlobalPlannerConfig, PlanError};
pub use pose2d::Pose2d;
