//! The navigation problem handed to every pipeline stage.

use std::sync::Arc;

use glam::DVec3;
use serde::{Deserialize, Serialize};

use robot_foresight::{ForecastSet, ForesightError};

use crate::VoxelGrid;

/// Motion limits of a holonomic 3D robot (multirotor, omni base with lift).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct KinematicLimits {
    /// Maximum speed (m/s).
    pub max_speed_mps: f64,
    /// Maximum vertical speed (m/s); `0` keeps the robot at its start height.
    pub max_vertical_speed_mps: f64,
}

/// A space-time navigation query.
#[derive(Clone, Debug)]
pub struct NavProblem {
    /// Static 3D occupancy.
    pub map: Arc<VoxelGrid>,
    /// Predicted dynamic agents.
    pub forecast: Arc<ForecastSet>,
    /// Start position (m).
    pub start_m: DVec3,
    /// Start time (s).
    pub start_t_s: f64,
    /// Goal position (m).
    pub goal_m: DVec3,
    /// Goal is reached within this distance (m).
    pub goal_tolerance_m: f64,
    /// Robot bounding-sphere radius (m).
    pub robot_radius_m: f64,
    /// Motion limits.
    pub limits: KinematicLimits,
    /// Largest acceptable collision probability with a predicted agent at
    /// any instant.
    pub risk_threshold: f64,
}

impl NavProblem {
    /// Validates the query.
    ///
    /// # Errors
    ///
    /// Returns [`ForesightError::InvalidInput`] for non-finite or
    /// out-of-range values, or an invalid forecast.
    pub fn validate(&self) -> Result<(), ForesightError> {
        let ok = self.start_m.is_finite()
            && self.goal_m.is_finite()
            && self.start_t_s.is_finite()
            && self.goal_tolerance_m > 0.0
            && self.robot_radius_m >= 0.0
            && self.limits.max_speed_mps > 0.0
            && self.limits.max_vertical_speed_mps >= 0.0
            && (0.0..=1.0).contains(&self.risk_threshold);
        if !ok {
            return Err(ForesightError::InvalidInput("navigation problem".into()));
        }
        self.forecast.validate()
    }

    /// Whether `p` is within the goal tolerance.
    #[must_use]
    pub fn at_goal(&self, p: DVec3) -> bool {
        p.distance(self.goal_m) <= self.goal_tolerance_m
    }

    /// Shortest time (s) to move by `delta` under the speed limits.
    #[must_use]
    pub fn min_travel_time_s(&self, delta: DVec3) -> f64 {
        let horizontal = delta.length() / self.limits.max_speed_mps;
        let vertical = if delta.z.abs() <= 1e-12 {
            0.0
        } else if self.limits.max_vertical_speed_mps <= 0.0 {
            f64::INFINITY
        } else {
            delta.z.abs() / self.limits.max_vertical_speed_mps
        };
        horizontal.max(vertical)
    }
}
