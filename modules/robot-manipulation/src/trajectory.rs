//! Timed joint-space trajectories.

use serde::{Deserialize, Serialize};

/// One timed configuration.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct JointWaypoint {
    /// Absolute time (s).
    pub t_s: f64,
    /// Joint positions (rad).
    pub q: Vec<f64>,
}

/// Piecewise-linear joint trajectory; holds the last configuration after
/// the end.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct JointTrajectory {
    /// Waypoints sorted by time.
    pub waypoints: Vec<JointWaypoint>,
}

impl JointTrajectory {
    /// Start time (s).
    #[must_use]
    pub fn start_t_s(&self) -> f64 {
        self.waypoints.first().map_or(0.0, |w| w.t_s)
    }

    /// End time (s).
    #[must_use]
    pub fn end_t_s(&self) -> f64 {
        self.waypoints.last().map_or(0.0, |w| w.t_s)
    }

    /// Duration (s).
    #[must_use]
    pub fn duration_s(&self) -> f64 {
        self.end_t_s() - self.start_t_s()
    }

    /// Sum over segments of the joint-space Euclidean distance (rad).
    #[must_use]
    pub fn path_length_rad(&self) -> f64 {
        self.waypoints
            .windows(2)
            .map(|w| {
                w[0].q
                    .iter()
                    .zip(&w[1].q)
                    .map(|(a, b)| (a - b).powi(2))
                    .sum::<f64>()
                    .sqrt()
            })
            .sum()
    }

    /// Configuration at `t_s`.
    #[must_use]
    pub fn q_at(&self, t_s: f64) -> Option<Vec<f64>> {
        let first = self.waypoints.first()?;
        if t_s <= first.t_s {
            return Some(first.q.clone());
        }
        let idx = self.waypoints.partition_point(|w| w.t_s <= t_s);
        if idx >= self.waypoints.len() {
            return self.waypoints.last().map(|w| w.q.clone());
        }
        let (a, b) = (&self.waypoints[idx - 1], &self.waypoints[idx]);
        let span = b.t_s - a.t_s;
        let u = if span <= 0.0 {
            1.0
        } else {
            (t_s - a.t_s) / span
        };
        Some(a.q.iter().zip(&b.q).map(|(x, y)| x + (y - x) * u).collect())
    }

    /// Time (s) the configuration first stops changing, i.e. the start of
    /// the final hold.
    #[must_use]
    pub fn wait_time_s(&self) -> f64 {
        self.waypoints
            .windows(2)
            .filter(|w| w[0].q == w[1].q)
            .map(|w| w[1].t_s - w[0].t_s)
            .fold(0.0, |a, b| a + b)
    }
}
