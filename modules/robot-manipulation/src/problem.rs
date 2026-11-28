//! The motion problem handed to every motion stage.

use std::sync::Arc;

use robot_foresight::{ForecastSet, ForesightError, WorldForecast};

use crate::{ManipScene, SerialChain};

/// Plan a timed joint trajectory from `start_q` to `goal_q`.
#[derive(Clone, Debug)]
pub struct MotionProblem {
    /// The arm.
    pub chain: Arc<SerialChain>,
    /// Static obstacles.
    pub scene: Arc<ManipScene>,
    /// Predicted moving agents (hands, people, other robots).
    pub forecast: Arc<ForecastSet>,
    /// Start configuration (rad).
    pub start_q: Vec<f64>,
    /// Goal configuration (rad).
    pub goal_q: Vec<f64>,
    /// Start time (s).
    pub start_t_s: f64,
    /// Largest acceptable collision probability per link sphere at any instant.
    pub risk_threshold: f64,
    /// Longest trajectory considered (s).
    pub max_duration_s: f64,
}

impl MotionProblem {
    /// Validates sizes, limits and the forecast.
    ///
    /// # Errors
    ///
    /// Returns [`ForesightError::InvalidInput`].
    pub fn validate(&self) -> Result<(), ForesightError> {
        self.chain.check_q(&self.start_q)?;
        self.chain.check_q(&self.goal_q)?;
        if !self.chain.within_limits(&self.start_q) || !self.chain.within_limits(&self.goal_q) {
            return Err(ForesightError::InvalidInput(
                "start or goal outside joint limits".into(),
            ));
        }
        if !(0.0..=1.0).contains(&self.risk_threshold)
            || self.max_duration_s.is_nan()
            || self.max_duration_s <= 0.0
            || !self.start_t_s.is_finite()
        {
            return Err(ForesightError::InvalidInput(
                "motion problem settings".into(),
            ));
        }
        self.forecast.validate()
    }

    /// Smallest static clearance (m) of the arm at `q`.
    #[must_use]
    pub fn static_clearance(&self, q: &[f64]) -> f64 {
        self.chain
            .collision_spheres(q)
            .iter()
            .map(|p| self.scene.clearance(*p, self.chain.link_radius_m))
            .fold(f64::INFINITY, f64::min)
    }

    /// Largest per-sphere collision probability of the arm at `q`, `t_s`.
    #[must_use]
    pub fn risk(&self, q: &[f64], t_s: f64) -> f64 {
        self.chain
            .collision_spheres(q)
            .iter()
            .map(|p| {
                self.forecast
                    .collision_probability(*p, self.chain.link_radius_m, t_s)
            })
            .fold(0.0, f64::max)
    }

    /// Smallest clearance (m) of the arm to predicted agents at `t_s`.
    #[must_use]
    pub fn agent_clearance(&self, q: &[f64], t_s: f64) -> f64 {
        self.chain
            .collision_spheres(q)
            .iter()
            .map(|p| self.forecast.clearance(*p, self.chain.link_radius_m, t_s))
            .fold(f64::INFINITY, f64::min)
    }

    /// Time (s) joint motion `from -> to` needs at the velocity limits.
    #[must_use]
    pub fn min_time_s(&self, from: &[f64], to: &[f64]) -> f64 {
        self.chain
            .joints
            .iter()
            .zip(from.iter().zip(to))
            .map(|(j, (a, b))| (b - a).abs() / j.max_velocity_radps)
            .fold(0.0, f64::max)
    }
}
