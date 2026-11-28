//! Forecasters: observed agent tracks in, a [`ForecastSet`] out.

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::{
    AgentForecast, ForecastSet, ForesightError, InferenceModel, Provenance, TimedState,
    TrajectoryMode,
};

/// The observed history of one agent.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AgentTrack {
    /// Stable agent identifier.
    pub agent_id: u64,
    /// Bounding-sphere radius (m).
    pub radius_m: f64,
    /// Observed states, oldest first, sorted by time.
    pub history: Vec<TimedState>,
}

impl AgentTrack {
    /// Latest observed state.
    #[must_use]
    pub fn last(&self) -> Option<&TimedState> {
        self.history.last()
    }

    /// Velocity estimate: finite difference of the last two states, or the
    /// last reported velocity with a single state.
    #[must_use]
    pub fn velocity_estimate(&self) -> Option<DVec3> {
        let n = self.history.len();
        match n {
            0 => None,
            1 => Some(self.history[0].velocity_mps),
            _ => {
                let (a, b) = (self.history[n - 2], self.history[n - 1]);
                let dt = b.t_s - a.t_s;
                Some(if dt > 1e-9 {
                    (b.position_m - a.position_m) / dt
                } else {
                    b.velocity_mps
                })
            }
        }
    }

    fn validate(&self) -> Result<(), ForesightError> {
        let ok = !self.history.is_empty()
            && self.radius_m.is_finite()
            && self.radius_m >= 0.0
            && self.history.iter().all(|s| {
                s.t_s.is_finite() && s.position_m.is_finite() && s.velocity_mps.is_finite()
            })
            && self.history.windows(2).all(|w| w[0].t_s <= w[1].t_s);
        if ok {
            Ok(())
        } else {
            Err(ForesightError::InvalidInput(format!(
                "track {}",
                self.agent_id
            )))
        }
    }
}

/// Turns observed tracks into a forecast of the world.
pub trait Forecaster {
    /// Where the forecast comes from.
    fn provenance(&self) -> Provenance;

    /// Forecasts every track from `now_s` over `horizon_s` seconds.
    ///
    /// # Errors
    ///
    /// Returns [`ForesightError`] on invalid tracks or a failed model.
    fn forecast(
        &self,
        tracks: &[AgentTrack],
        now_s: f64,
        horizon_s: f64,
    ) -> Result<ForecastSet, ForesightError>;
}

fn check_times(now_s: f64, horizon_s: f64, dt_s: f64) -> Result<(), ForesightError> {
    if now_s.is_finite() && horizon_s.is_finite() && horizon_s >= 0.0 && dt_s > 0.0 {
        Ok(())
    } else {
        Err(ForesightError::InvalidConfig("forecast times".into()))
    }
}

/// Rolls a constant velocity out from `start` (position held at `now_s`).
fn rollout(
    start_m: DVec3,
    velocity_mps: DVec3,
    now_s: f64,
    horizon_s: f64,
    dt_s: f64,
) -> Vec<TimedState> {
    let steps = (horizon_s / dt_s).ceil() as usize;
    (0..=steps)
        .map(|k| {
            let tau = (k as f64 * dt_s).min(horizon_s);
            TimedState {
                t_s: now_s + tau,
                position_m: start_m + velocity_mps * tau,
                velocity_mps,
            }
        })
        .collect()
}

/// Classical forecaster: every agent keeps its estimated velocity.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ConstantVelocityForecaster {
    /// Spacing (s) of the forecast states.
    pub dt_s: f64,
    /// Position standard deviation at the forecast time (m).
    pub sigma0_m: f64,
    /// Standard deviation growth per second of lookahead (m/s).
    pub sigma_growth_mps: f64,
}

impl Default for ConstantVelocityForecaster {
    fn default() -> Self {
        Self {
            dt_s: 0.1,
            sigma0_m: 0.05,
            sigma_growth_mps: 0.15,
        }
    }
}

impl Forecaster for ConstantVelocityForecaster {
    fn provenance(&self) -> Provenance {
        Provenance::classical("constant_velocity")
    }

    fn forecast(
        &self,
        tracks: &[AgentTrack],
        now_s: f64,
        horizon_s: f64,
    ) -> Result<ForecastSet, ForesightError> {
        check_times(now_s, horizon_s, self.dt_s)?;
        let mut agents = Vec::with_capacity(tracks.len());
        for track in tracks {
            track.validate()?;
            let last = track.history[track.history.len() - 1];
            let v = track.velocity_estimate().unwrap_or(DVec3::ZERO);
            let start = last.position_m + v * (now_s - last.t_s);
            agents.push(AgentForecast {
                agent_id: track.agent_id,
                radius_m: track.radius_m,
                sigma0_m: self.sigma0_m,
                sigma_growth_mps: self.sigma_growth_mps,
                modes: vec![TrajectoryMode {
                    probability: 1.0,
                    states: rollout(start, v, now_s, horizon_s, self.dt_s),
                }],
            });
        }
        Ok(ForecastSet {
            created_at_s: now_s,
            horizon_s,
            agents,
        })
    }
}

/// Learned multimodal forecaster around any [`InferenceModel`].
///
/// Input (`history_len * 3 + 3` values): the last `history_len` positions
/// relative to the latest one (oldest first, padded with the oldest
/// observation), then the latest velocity estimate. Output (`num_modes * 4`
/// values): per mode a logit and a velocity `[vx, vy, vz]`. Mode
/// probabilities are the softmax of the logits; each mode is rolled out at
/// its velocity. Richer decoders (full waypoint sequences, social pooling)
/// implement [`Forecaster`] directly.
#[derive(Clone, Debug)]
pub struct LearnedForecaster<M> {
    /// The model.
    pub model: M,
    /// Number of past positions fed to the model.
    pub history_len: usize,
    /// Number of output modes.
    pub num_modes: usize,
    /// Spacing (s) of the forecast states.
    pub dt_s: f64,
    /// Position standard deviation at the forecast time (m).
    pub sigma0_m: f64,
    /// Standard deviation growth per second of lookahead (m/s).
    pub sigma_growth_mps: f64,
}

impl<M: InferenceModel> LearnedForecaster<M> {
    /// Wraps `model`, checking its shape against `history_len` and `num_modes`.
    ///
    /// # Errors
    ///
    /// Returns [`ForesightError::InvalidConfig`] when the model's dimensions
    /// do not match.
    pub fn new(model: M, history_len: usize, num_modes: usize) -> Result<Self, ForesightError> {
        let spec = model.spec();
        if history_len == 0
            || num_modes == 0
            || spec.input_dim != history_len * 3 + 3
            || spec.output_dim != num_modes * 4
        {
            return Err(ForesightError::InvalidConfig(format!(
                "model `{}` shape {}->{} does not fit history {history_len}, modes {num_modes}",
                spec.model_id, spec.input_dim, spec.output_dim
            )));
        }
        Ok(Self {
            model,
            history_len,
            num_modes,
            dt_s: 0.1,
            sigma0_m: 0.05,
            sigma_growth_mps: 0.1,
        })
    }

    /// Feature vector for one track (see the type docs).
    #[must_use]
    pub fn features(&self, track: &AgentTrack) -> Vec<f32> {
        let n = track.history.len();
        let last = track.history[n - 1].position_m;
        let mut out = Vec::with_capacity(self.history_len * 3 + 3);
        for k in 0..self.history_len {
            // k = 0 is the oldest slot.
            let back = self.history_len - 1 - k;
            let idx = n.saturating_sub(1 + back);
            let rel = track.history[idx].position_m - last;
            out.extend([rel.x as f32, rel.y as f32, rel.z as f32]);
        }
        let v = track.velocity_estimate().unwrap_or(DVec3::ZERO);
        out.extend([v.x as f32, v.y as f32, v.z as f32]);
        out
    }
}

impl<M: InferenceModel> Forecaster for LearnedForecaster<M> {
    fn provenance(&self) -> Provenance {
        Provenance::learned(&self.model.spec().model_id)
    }

    fn forecast(
        &self,
        tracks: &[AgentTrack],
        now_s: f64,
        horizon_s: f64,
    ) -> Result<ForecastSet, ForesightError> {
        check_times(now_s, horizon_s, self.dt_s)?;
        let mut agents = Vec::with_capacity(tracks.len());
        for track in tracks {
            track.validate()?;
            let out = self.model.infer(&self.features(track))?;
            let last = track.history[track.history.len() - 1];
            let max_logit = (0..self.num_modes)
                .map(|m| f64::from(out[m * 4]))
                .fold(f64::NEG_INFINITY, f64::max);
            let modes = (0..self.num_modes)
                .map(|m| {
                    let o = &out[m * 4..m * 4 + 4];
                    let v = DVec3::new(f64::from(o[1]), f64::from(o[2]), f64::from(o[3]));
                    let start = last.position_m + v * (now_s - last.t_s);
                    TrajectoryMode {
                        probability: (f64::from(o[0]) - max_logit).exp(),
                        states: rollout(start, v, now_s, horizon_s, self.dt_s),
                    }
                })
                .collect();
            agents.push(AgentForecast {
                agent_id: track.agent_id,
                radius_m: track.radius_m,
                sigma0_m: self.sigma0_m,
                sigma_growth_mps: self.sigma_growth_mps,
                modes,
            });
        }
        Ok(ForecastSet {
            created_at_s: now_s,
            horizon_s,
            agents,
        })
    }
}

/// Mixes two forecasters: per agent, the modes of `secondary` are added with
/// total weight `secondary_weight` and those of `primary` with the rest;
/// the uncertainty is the larger of the two.
///
/// Typical use: a learned primary with a constant-velocity secondary, so
/// the physically plausible future is never dropped by the model.
#[derive(Clone, Debug)]
pub struct BlendedForecaster<A, B> {
    /// First forecaster (weight `1 - secondary_weight`).
    pub primary: A,
    /// Second forecaster (weight `secondary_weight`).
    pub secondary: B,
    /// Weight of the secondary's modes, in `[0, 1]`.
    pub secondary_weight: f64,
}

impl<A: Forecaster, B: Forecaster> Forecaster for BlendedForecaster<A, B> {
    fn provenance(&self) -> Provenance {
        Provenance::Hybrid {
            parts: vec![self.primary.provenance(), self.secondary.provenance()],
        }
    }

    fn forecast(
        &self,
        tracks: &[AgentTrack],
        now_s: f64,
        horizon_s: f64,
    ) -> Result<ForecastSet, ForesightError> {
        let w = self.secondary_weight;
        if !(0.0..=1.0).contains(&w) {
            return Err(ForesightError::InvalidConfig("secondary_weight".into()));
        }
        let a = self.primary.forecast(tracks, now_s, horizon_s)?;
        let b = self.secondary.forecast(tracks, now_s, horizon_s)?;
        let mut agents = Vec::with_capacity(a.agents.len());
        for pa in a.agents {
            let Some(pb) = b.agents.iter().find(|x| x.agent_id == pa.agent_id) else {
                agents.push(pa);
                continue;
            };
            let scaled = |modes: &[TrajectoryMode], weight: f64| {
                let total: f64 = modes.iter().map(|m| m.probability).sum::<f64>().max(1e-300);
                modes
                    .iter()
                    .map(|m| TrajectoryMode {
                        probability: m.probability / total * weight,
                        states: m.states.clone(),
                    })
                    .collect::<Vec<_>>()
            };
            let mut modes = scaled(&pa.modes, 1.0 - w);
            modes.extend(scaled(&pb.modes, w));
            modes.retain(|m| m.probability > 0.0);
            agents.push(AgentForecast {
                agent_id: pa.agent_id,
                radius_m: pa.radius_m.max(pb.radius_m),
                sigma0_m: pa.sigma0_m.max(pb.sigma0_m),
                sigma_growth_mps: pa.sigma_growth_mps.max(pb.sigma_growth_mps),
                modes,
            });
        }
        Ok(ForecastSet {
            created_at_s: now_s,
            horizon_s,
            agents,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{FnModel, WorldForecast};

    fn track() -> AgentTrack {
        AgentTrack {
            agent_id: 9,
            radius_m: 0.25,
            history: vec![
                TimedState {
                    t_s: 0.0,
                    position_m: DVec3::ZERO,
                    velocity_mps: DVec3::ZERO,
                },
                TimedState {
                    t_s: 0.5,
                    position_m: DVec3::X * 0.5,
                    velocity_mps: DVec3::ZERO,
                },
            ],
        }
    }

    #[test]
    fn constant_velocity_extrapolates_track() {
        let f = ConstantVelocityForecaster {
            sigma0_m: 0.0,
            sigma_growth_mps: 0.0,
            dt_s: 0.25,
        };
        let set = f.forecast(&[track()], 1.0, 2.0).unwrap();
        set.validate().unwrap();
        let pos = set.agents[0].modes[0].position_at(3.0).unwrap();
        assert!((pos - DVec3::X * 3.0).length() < 1e-12);
        assert_eq!(set.collision_probability(DVec3::X * 3.0, 0.1, 3.0), 1.0);
    }

    #[test]
    fn learned_forecaster_builds_softmax_modes() {
        // Two modes: keep going (logit ln 3) or turn to +Y (logit 0).
        let model = FnModel::new("intent", 2 * 3 + 3, 2 * 4, |x| {
            vec![3f32.ln(), x[6], x[7], x[8], 0.0, 0.0, 1.0, 0.0]
        });
        let f = LearnedForecaster::new(model, 2, 2).unwrap();
        assert_eq!(f.provenance(), Provenance::learned("intent"));
        let set = f.forecast(&[track()], 0.5, 2.0).unwrap();
        let a = &set.agents[0];
        let total: f64 = a.modes.iter().map(|m| m.probability).sum();
        assert!((a.modes[0].probability / total - 0.75).abs() < 1e-6);
        let turn = a.modes[1].position_at(1.5).unwrap();
        assert!((turn - DVec3::new(0.5, 1.0, 0.0)).length() < 1e-9);
    }

    #[test]
    fn learned_forecaster_checks_shape() {
        let model = FnModel::new("bad", 3, 4, |_| vec![0.0; 4]);
        assert!(LearnedForecaster::new(model, 2, 1).is_err());
    }

    #[test]
    fn blend_keeps_both_futures() {
        let model = FnModel::new("stop", 9, 4, |_| vec![0.0, 0.0, 0.0, 0.0]);
        let blended = BlendedForecaster {
            primary: LearnedForecaster::new(model, 2, 1).unwrap(),
            secondary: ConstantVelocityForecaster::default(),
            secondary_weight: 0.3,
        };
        let set = blended.forecast(&[track()], 0.5, 2.0).unwrap();
        let a = &set.agents[0];
        assert_eq!(a.modes.len(), 2);
        assert!((a.modes[0].probability - 0.7).abs() < 1e-12);
        assert!(matches!(blended.provenance(), Provenance::Hybrid { .. }));
    }
}
