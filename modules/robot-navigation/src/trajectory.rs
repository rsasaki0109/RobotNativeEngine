//! Timed 3D trajectories.

use glam::DVec3;
use serde::{Deserialize, Serialize};

/// One timed position.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Waypoint3 {
    /// Absolute time (s).
    pub t_s: f64,
    /// Position (m).
    pub position_m: DVec3,
}

/// A piecewise-linear trajectory in space-time. Waypoints are sorted by
/// time; the robot holds the last position after the last waypoint.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Trajectory3 {
    /// Waypoints sorted by time.
    pub waypoints: Vec<Waypoint3>,
}

impl Trajectory3 {
    /// Builds a trajectory from waypoints.
    #[must_use]
    pub fn new(waypoints: Vec<Waypoint3>) -> Self {
        Self { waypoints }
    }

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

    /// Final position.
    #[must_use]
    pub fn end_position_m(&self) -> Option<DVec3> {
        self.waypoints.last().map(|w| w.position_m)
    }

    /// Path length (m).
    #[must_use]
    pub fn length_m(&self) -> f64 {
        self.waypoints
            .windows(2)
            .map(|w| w[0].position_m.distance(w[1].position_m))
            .sum()
    }

    /// Highest segment speed (m/s); infinite for a jump in zero time.
    #[must_use]
    pub fn max_speed_mps(&self) -> f64 {
        self.waypoints
            .windows(2)
            .map(|w| {
                let d = w[0].position_m.distance(w[1].position_m);
                let dt = w[1].t_s - w[0].t_s;
                if d <= 1e-12 {
                    0.0
                } else if dt <= 0.0 {
                    f64::INFINITY
                } else {
                    d / dt
                }
            })
            .fold(0.0, f64::max)
    }

    /// Position at `t_s` (held before the start and after the end).
    #[must_use]
    pub fn position_at(&self, t_s: f64) -> Option<DVec3> {
        let first = self.waypoints.first()?;
        if t_s <= first.t_s {
            return Some(first.position_m);
        }
        let idx = self.waypoints.partition_point(|w| w.t_s <= t_s);
        if idx >= self.waypoints.len() {
            return self.end_position_m();
        }
        let (a, b) = (self.waypoints[idx - 1], self.waypoints[idx]);
        let span = b.t_s - a.t_s;
        Some(if span <= 0.0 {
            b.position_m
        } else {
            a.position_m.lerp(b.position_m, (t_s - a.t_s) / span)
        })
    }

    /// Positions sampled every `dt_s` from start to end, end included.
    #[must_use]
    pub fn sample(&self, dt_s: f64) -> Vec<Waypoint3> {
        let (t0, t1) = (self.start_t_s(), self.end_t_s());
        if self.waypoints.is_empty() || dt_s <= 0.0 {
            return Vec::new();
        }
        let n = ((t1 - t0) / dt_s).ceil().max(0.0) as usize;
        (0..=n)
            .filter_map(|k| {
                let t = (t0 + k as f64 * dt_s).min(t1);
                self.position_at(t)
                    .map(|position_m| Waypoint3 { t_s: t, position_m })
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interpolates_and_measures() {
        let t = Trajectory3::new(vec![
            Waypoint3 {
                t_s: 1.0,
                position_m: DVec3::ZERO,
            },
            Waypoint3 {
                t_s: 3.0,
                position_m: DVec3::X * 4.0,
            },
            Waypoint3 {
                t_s: 4.0,
                position_m: DVec3::X * 4.0,
            },
        ]);
        assert_eq!(t.duration_s(), 3.0);
        assert_eq!(t.length_m(), 4.0);
        assert_eq!(t.max_speed_mps(), 2.0);
        assert_eq!(t.position_at(2.0), Some(DVec3::X * 2.0));
        assert_eq!(t.position_at(9.0), Some(DVec3::X * 4.0));
        assert_eq!(t.sample(1.0).len(), 4);
    }
}
