//! Multi-object tracking: detections in, [`AgentTrack`]s for forecasters out.
//!
//! Ported from `rne_nav::ObstacleTracker` (ADR 037): greedy, index-ordered
//! nearest-neighbour association with a distance gate and a smoothed
//! constant-velocity estimate, so a detection sequence replays
//! deterministically. Unlike the source, it works in 3D, keeps a bounded
//! timestamped history per track, and hands that history to any
//! [`crate::Forecaster`] through [`ObstacleTracker::agent_tracks`].

use std::collections::VecDeque;

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::{AgentTrack, ForesightError, TimedState};

/// One tracked object.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Track {
    /// Stable track id.
    pub id: u64,
    /// Latest position (m).
    pub position_m: DVec3,
    /// Smoothed velocity estimate (m/s).
    pub velocity_mps: DVec3,
    /// Bounding-sphere radius (m).
    pub radius_m: f64,
    /// Consecutive updates without an associated detection.
    pub missed: u32,
    /// Recent associated states, oldest first.
    pub history: VecDeque<TimedState>,
}

/// One detection.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Detection {
    /// Position (m).
    pub position_m: DVec3,
    /// Bounding-sphere radius (m).
    pub radius_m: f64,
}

/// Configuration of [`ObstacleTracker`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ObstacleTrackerConfig {
    /// Largest association distance (m).
    pub gate_m: f64,
    /// Updates a track may miss before it is dropped.
    pub max_missed: u32,
    /// Velocity smoothing gain in `(0, 1]`.
    pub velocity_gain: f64,
    /// States kept per track (at least 1).
    pub history_len: usize,
}

impl Default for ObstacleTrackerConfig {
    fn default() -> Self {
        Self {
            gate_m: 1.0,
            max_missed: 3,
            velocity_gain: 0.5,
            history_len: 8,
        }
    }
}

impl ObstacleTrackerConfig {
    fn is_valid(&self) -> bool {
        self.gate_m.is_finite()
            && self.gate_m > 0.0
            && self.velocity_gain.is_finite()
            && self.velocity_gain > 0.0
            && self.velocity_gain <= 1.0
            && self.history_len >= 1
    }
}

/// Greedy nearest-neighbour constant-velocity tracker with its own clock.
#[derive(Clone, Debug, PartialEq)]
pub struct ObstacleTracker {
    config: ObstacleTrackerConfig,
    tracks: Vec<Track>,
    next_id: u64,
    time_s: f64,
}

impl ObstacleTracker {
    /// An empty tracker whose clock starts at `start_t_s`.
    ///
    /// # Errors
    ///
    /// Returns [`ForesightError::InvalidConfig`] for an invalid configuration
    /// or start time.
    pub fn new(config: ObstacleTrackerConfig, start_t_s: f64) -> Result<Self, ForesightError> {
        if !config.is_valid() || !start_t_s.is_finite() {
            return Err(ForesightError::InvalidConfig("obstacle tracker".into()));
        }
        Ok(Self {
            config,
            tracks: Vec::new(),
            next_id: 0,
            time_s: start_t_s,
        })
    }

    /// Current tracks, in creation order.
    #[must_use]
    pub fn tracks(&self) -> &[Track] {
        &self.tracks
    }

    /// Time (s) of the latest update.
    #[must_use]
    pub fn time_s(&self) -> f64 {
        self.time_s
    }

    /// Advances the clock by `dt_s` and associates `detections`.
    ///
    /// # Errors
    ///
    /// Returns [`ForesightError::InvalidInput`] for a non-positive or
    /// non-finite step or a non-finite detection; the tracker is unchanged.
    pub fn update(&mut self, detections: &[Detection], dt_s: f64) -> Result<(), ForesightError> {
        if !(dt_s.is_finite() && dt_s > 0.0) {
            return Err(ForesightError::InvalidInput("tracker time step".into()));
        }
        if detections
            .iter()
            .any(|d| !d.position_m.is_finite() || !d.radius_m.is_finite())
        {
            return Err(ForesightError::InvalidInput("non-finite detection".into()));
        }
        self.time_s += dt_s;
        let t = self.time_s;
        let keep = self.config.history_len;
        let push = |history: &mut VecDeque<TimedState>, state: TimedState| {
            history.push_back(state);
            while history.len() > keep {
                history.pop_front();
            }
        };
        let mut assigned = vec![false; detections.len()];
        for track in &mut self.tracks {
            let mut best: Option<(usize, f64)> = None;
            for (index, detection) in detections.iter().enumerate() {
                if assigned[index] {
                    continue;
                }
                let distance = detection.position_m.distance(track.position_m);
                if distance <= self.config.gate_m && best.is_none_or(|(_, b)| distance < b) {
                    best = Some((index, distance));
                }
            }
            if let Some((index, _)) = best {
                assigned[index] = true;
                let detection = detections[index];
                // Velocity from the last associated observation, so missed
                // updates do not inflate it.
                let since = track.history.back().map_or(dt_s, |s| (t - s.t_s).max(dt_s));
                let measured = (detection.position_m - track.position_m) / since;
                track.velocity_mps += (measured - track.velocity_mps) * self.config.velocity_gain;
                track.position_m = detection.position_m;
                track.radius_m = detection.radius_m;
                track.missed = 0;
                push(
                    &mut track.history,
                    TimedState {
                        t_s: t,
                        position_m: track.position_m,
                        velocity_mps: track.velocity_mps,
                    },
                );
            } else {
                track.missed += 1;
            }
        }
        self.tracks
            .retain(|track| track.missed <= self.config.max_missed);
        for (index, detection) in detections.iter().enumerate() {
            if assigned[index] {
                continue;
            }
            let mut history = VecDeque::with_capacity(keep);
            push(
                &mut history,
                TimedState {
                    t_s: t,
                    position_m: detection.position_m,
                    velocity_mps: DVec3::ZERO,
                },
            );
            self.tracks.push(Track {
                id: self.next_id,
                position_m: detection.position_m,
                velocity_mps: DVec3::ZERO,
                radius_m: detection.radius_m,
                missed: 0,
                history,
            });
            self.next_id += 1;
        }
        Ok(())
    }

    /// Every track as an [`AgentTrack`] for a [`crate::Forecaster`].
    ///
    /// The last state carries the smoothed velocity, so a single-observation
    /// track and a constant-velocity forecaster agree with the tracker.
    #[must_use]
    pub fn agent_tracks(&self) -> Vec<AgentTrack> {
        self.tracks
            .iter()
            .map(|t| AgentTrack {
                agent_id: t.id,
                radius_m: t.radius_m,
                history: t.history.iter().copied().collect(),
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ConstantVelocityForecaster, Forecaster, WorldForecast};

    fn tracker() -> ObstacleTracker {
        ObstacleTracker::new(ObstacleTrackerConfig::default(), 0.0).unwrap()
    }

    #[test]
    fn tracks_a_moving_detection_and_estimates_velocity() {
        let mut t = tracker();
        for k in 0..10 {
            let x = 0.1 * f64::from(k);
            t.update(
                &[Detection {
                    position_m: DVec3::new(x, 0.0, 0.5),
                    radius_m: 0.3,
                }],
                0.1,
            )
            .unwrap();
        }
        assert_eq!(t.tracks().len(), 1);
        let track = &t.tracks()[0];
        assert!(
            (track.velocity_mps.x - 1.0).abs() < 0.01,
            "{:?}",
            track.velocity_mps
        );
        assert_eq!(track.history.len(), 8);
        assert!((t.time_s() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn drops_tracks_after_missing_updates() {
        let mut t = tracker();
        t.update(
            &[Detection {
                position_m: DVec3::ZERO,
                radius_m: 0.3,
            }],
            0.1,
        )
        .unwrap();
        for _ in 0..3 {
            t.update(&[], 0.1).unwrap();
        }
        assert_eq!(t.tracks().len(), 1);
        t.update(&[], 0.1).unwrap();
        assert!(t.tracks().is_empty());
    }

    #[test]
    fn far_detections_start_new_tracks_with_new_ids() {
        let mut t = tracker();
        t.update(
            &[Detection {
                position_m: DVec3::ZERO,
                radius_m: 0.3,
            }],
            0.1,
        )
        .unwrap();
        t.update(
            &[Detection {
                position_m: DVec3::X * 5.0,
                radius_m: 0.3,
            }],
            0.1,
        )
        .unwrap();
        let ids: Vec<u64> = t.tracks().iter().map(|x| x.id).collect();
        assert_eq!(ids, vec![0, 1]);
    }

    #[test]
    fn rejects_invalid_config_and_time() {
        let bad = ObstacleTrackerConfig {
            velocity_gain: 0.0,
            ..Default::default()
        };
        assert!(ObstacleTracker::new(bad, 0.0).is_err());
        assert!(tracker().update(&[], 0.0).is_err());
        assert!(tracker().update(&[], f64::NAN).is_err());
    }

    #[test]
    fn feeds_a_forecaster() {
        let mut t = tracker();
        for k in 0..5 {
            let y = 0.2 * f64::from(k);
            t.update(
                &[Detection {
                    position_m: DVec3::new(3.0, y, 1.0),
                    radius_m: 0.3,
                }],
                0.2,
            )
            .unwrap();
        }
        let f = ConstantVelocityForecaster {
            dt_s: 0.1,
            sigma0_m: 0.0,
            sigma_growth_mps: 0.0,
        };
        let set = f.forecast(&t.agent_tracks(), t.time_s(), 3.0).unwrap();
        // Moving +y at 1 m/s from (3, 0.8, 1) at t = 1.0 s.
        let ahead = DVec3::new(3.0, 2.8, 1.0);
        assert_eq!(set.collision_probability(ahead, 0.1, t.time_s() + 2.0), 1.0);
    }
}
