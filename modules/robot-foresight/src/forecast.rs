//! The world as a function of time: predicted agents and collision risk.

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::ForesightError;

/// One timed state of a predicted agent.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TimedState {
    /// Absolute time (s).
    pub t_s: f64,
    /// Position (m).
    pub position_m: DVec3,
    /// Velocity (m/s).
    pub velocity_mps: DVec3,
}

/// One possible future of an agent, with its probability.
///
/// States are sorted by time. Between states the position is interpolated
/// linearly; after the last state it is extrapolated with the last velocity,
/// before the first state it is held.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TrajectoryMode {
    /// Probability of this mode (the modes of one agent are normalised when
    /// queried, so only ratios matter).
    pub probability: f64,
    /// Timed states, sorted by `t_s`.
    pub states: Vec<TimedState>,
}

impl TrajectoryMode {
    /// Position at absolute time `t_s`, or `None` for an empty mode.
    #[must_use]
    pub fn position_at(&self, t_s: f64) -> Option<DVec3> {
        let first = self.states.first()?;
        if t_s <= first.t_s {
            return Some(first.position_m);
        }
        let idx = self.states.partition_point(|s| s.t_s <= t_s);
        if idx >= self.states.len() {
            let last = self.states[self.states.len() - 1];
            return Some(last.position_m + last.velocity_mps * (t_s - last.t_s));
        }
        let a = self.states[idx - 1];
        let b = self.states[idx];
        let span = b.t_s - a.t_s;
        if span <= 0.0 {
            return Some(b.position_m);
        }
        Some(a.position_m.lerp(b.position_m, (t_s - a.t_s) / span))
    }
}

/// The multimodal forecast of one agent.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AgentForecast {
    /// Stable agent identifier.
    pub agent_id: u64,
    /// Agent bounding-sphere radius (m).
    pub radius_m: f64,
    /// Position standard deviation at the forecast time (m).
    pub sigma0_m: f64,
    /// Growth of the position standard deviation per second of lookahead (m/s).
    pub sigma_growth_mps: f64,
    /// Possible futures.
    pub modes: Vec<TrajectoryMode>,
}

impl AgentForecast {
    /// Validates radii, uncertainties, probabilities and state ordering.
    ///
    /// # Errors
    ///
    /// Returns [`ForesightError::InvalidInput`] on a non-finite or negative
    /// value, an empty mode list, or unsorted states.
    pub fn validate(&self) -> Result<(), ForesightError> {
        let bad =
            |what: &str| ForesightError::InvalidInput(format!("agent {}: {what}", self.agent_id));
        if !(self.radius_m.is_finite() && self.radius_m >= 0.0) {
            return Err(bad("radius_m"));
        }
        if !(self.sigma0_m.is_finite() && self.sigma0_m >= 0.0) {
            return Err(bad("sigma0_m"));
        }
        if !(self.sigma_growth_mps.is_finite() && self.sigma_growth_mps >= 0.0) {
            return Err(bad("sigma_growth_mps"));
        }
        if self.modes.is_empty() {
            return Err(bad("no modes"));
        }
        for mode in &self.modes {
            if !(mode.probability.is_finite() && mode.probability >= 0.0) || mode.states.is_empty()
            {
                return Err(bad("mode probability or states"));
            }
            let finite = mode.states.iter().all(|s| {
                s.t_s.is_finite() && s.position_m.is_finite() && s.velocity_mps.is_finite()
            });
            let sorted = mode.states.windows(2).all(|w| w[0].t_s <= w[1].t_s);
            if !finite || !sorted {
                return Err(bad("states must be finite and sorted by time"));
            }
        }
        if self.modes.iter().map(|m| m.probability).sum::<f64>() <= 0.0 {
            return Err(bad("mode probabilities sum to zero"));
        }
        Ok(())
    }

    /// Position standard deviation (m) at absolute time `t_s` for a forecast
    /// made at `created_at_s`.
    #[must_use]
    pub fn sigma_at(&self, created_at_s: f64, t_s: f64) -> f64 {
        self.sigma0_m + self.sigma_growth_mps * (t_s - created_at_s).max(0.0)
    }

    /// Probability that a sphere (`center_m`, `radius_m`) overlaps this agent
    /// at `t_s`, summed over the agent's normalised modes.
    ///
    /// Per mode the centre distance `d` is compared with the combined radius
    /// `R` under a Gaussian position error of standard deviation `sigma`:
    /// `p = Phi((R - d) / sigma)`. With `sigma = 0` this is the exact
    /// indicator `d < R`.
    #[must_use]
    pub fn collision_probability(
        &self,
        created_at_s: f64,
        center_m: DVec3,
        radius_m: f64,
        t_s: f64,
    ) -> f64 {
        let total: f64 = self.modes.iter().map(|m| m.probability).sum();
        if total <= 0.0 {
            return 0.0;
        }
        let sigma = self.sigma_at(created_at_s, t_s);
        let reach = radius_m + self.radius_m;
        let mut p = 0.0;
        for mode in &self.modes {
            let Some(pos) = mode.position_at(t_s) else {
                continue;
            };
            let d = pos.distance(center_m);
            let hit = if sigma <= 1e-12 {
                if d < reach {
                    1.0
                } else {
                    0.0
                }
            } else {
                normal_cdf((reach - d) / sigma)
            };
            p += mode.probability / total * hit;
        }
        p.clamp(0.0, 1.0)
    }

    /// Distance (m) from `center_m` to the surface of this agent's most
    /// likely position at `t_s`, minus `radius_m` (negative when overlapping).
    #[must_use]
    pub fn clearance(&self, center_m: DVec3, radius_m: f64, t_s: f64) -> f64 {
        self.modes
            .iter()
            .filter_map(|m| m.position_at(t_s))
            .map(|p| p.distance(center_m) - radius_m - self.radius_m)
            .fold(f64::INFINITY, f64::min)
    }
}

/// Anything that can say how risky a point in space-time is.
pub trait WorldForecast {
    /// Time (s) the forecast was made.
    fn created_at_s(&self) -> f64;
    /// Lookahead (s) the forecast was produced for. Queries beyond it are
    /// answered by extrapolation with growing uncertainty.
    fn horizon_s(&self) -> f64;
    /// Probability that a sphere at `center_m` with `radius_m` is hit by a
    /// predicted agent at absolute time `t_s`.
    fn collision_probability(&self, center_m: DVec3, radius_m: f64, t_s: f64) -> f64;
    /// Smallest surface distance (m) to any agent's modal positions at `t_s`
    /// (`f64::INFINITY` with no agents).
    fn clearance(&self, center_m: DVec3, radius_m: f64, t_s: f64) -> f64;
}

/// A set of agent forecasts made at one time.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ForecastSet {
    /// Time (s) the forecast was made.
    pub created_at_s: f64,
    /// Lookahead (s) the forecast was produced for.
    pub horizon_s: f64,
    /// Forecast agents, in a stable order.
    pub agents: Vec<AgentForecast>,
}

impl ForecastSet {
    /// An empty forecast: a world with nothing moving.
    #[must_use]
    pub fn empty(created_at_s: f64) -> Self {
        Self {
            created_at_s,
            horizon_s: 0.0,
            agents: Vec::new(),
        }
    }

    /// Validates every agent.
    ///
    /// # Errors
    ///
    /// Returns the first agent's [`ForesightError`].
    pub fn validate(&self) -> Result<(), ForesightError> {
        if !(self.created_at_s.is_finite() && self.horizon_s.is_finite() && self.horizon_s >= 0.0) {
            return Err(ForesightError::InvalidInput("forecast times".into()));
        }
        self.agents.iter().try_for_each(AgentForecast::validate)
    }
}

impl WorldForecast for ForecastSet {
    fn created_at_s(&self) -> f64 {
        self.created_at_s
    }

    fn horizon_s(&self) -> f64 {
        self.horizon_s
    }

    fn collision_probability(&self, center_m: DVec3, radius_m: f64, t_s: f64) -> f64 {
        let free: f64 = self
            .agents
            .iter()
            .map(|a| 1.0 - a.collision_probability(self.created_at_s, center_m, radius_m, t_s))
            .product();
        (1.0 - free).clamp(0.0, 1.0)
    }

    fn clearance(&self, center_m: DVec3, radius_m: f64, t_s: f64) -> f64 {
        self.agents
            .iter()
            .map(|a| a.clearance(center_m, radius_m, t_s))
            .fold(f64::INFINITY, f64::min)
    }
}

/// Standard normal CDF via `erf` (Abramowitz-Stegun 7.1.26, |error| < 1.5e-7).
fn normal_cdf(x: f64) -> f64 {
    0.5 * (1.0 + erf(x / std::f64::consts::SQRT_2))
}

fn erf(x: f64) -> f64 {
    let sign = x.signum();
    let x = x.abs();
    let t = 1.0 / (1.0 + 0.327_591_1 * x);
    let poly = t
        * (0.254_829_592
            + t * (-0.284_496_736
                + t * (1.421_413_741 + t * (-1.453_152_027 + t * 1.061_405_429))));
    sign * (1.0 - poly * (-x * x).exp())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn walker(sigma_growth_mps: f64) -> AgentForecast {
        AgentForecast {
            agent_id: 1,
            radius_m: 0.3,
            sigma0_m: 0.0,
            sigma_growth_mps,
            modes: vec![TrajectoryMode {
                probability: 1.0,
                states: vec![TimedState {
                    t_s: 0.0,
                    position_m: DVec3::ZERO,
                    velocity_mps: DVec3::X,
                }],
            }],
        }
    }

    #[test]
    fn extrapolates_with_last_velocity() {
        let a = walker(0.0);
        assert_eq!(a.modes[0].position_at(2.0), Some(DVec3::new(2.0, 0.0, 0.0)));
    }

    #[test]
    fn interpolates_between_states() {
        let mode = TrajectoryMode {
            probability: 1.0,
            states: vec![
                TimedState {
                    t_s: 0.0,
                    position_m: DVec3::ZERO,
                    velocity_mps: DVec3::ZERO,
                },
                TimedState {
                    t_s: 2.0,
                    position_m: DVec3::Y * 4.0,
                    velocity_mps: DVec3::ZERO,
                },
            ],
        };
        assert_eq!(mode.position_at(1.0), Some(DVec3::Y * 2.0));
        assert_eq!(mode.position_at(-1.0), Some(DVec3::ZERO));
    }

    #[test]
    fn exact_indicator_without_uncertainty() {
        let set = ForecastSet {
            created_at_s: 0.0,
            horizon_s: 5.0,
            agents: vec![walker(0.0)],
        };
        let at = DVec3::new(2.0, 0.0, 0.0);
        assert_eq!(set.collision_probability(at, 0.2, 2.0), 1.0);
        assert_eq!(set.collision_probability(at, 0.2, 0.0), 0.0);
    }

    #[test]
    fn uncertainty_spreads_risk_with_lookahead() {
        let set = ForecastSet {
            created_at_s: 0.0,
            horizon_s: 5.0,
            agents: vec![walker(0.5)],
        };
        // 1 m beside the predicted position: risk rises as sigma grows.
        let near = set.collision_probability(DVec3::new(1.0, 1.0, 0.0), 0.2, 1.0);
        let far = set.collision_probability(DVec3::new(4.0, 1.0, 0.0), 0.2, 4.0);
        assert!(far > near, "{far} <= {near}");
        assert!(near > 0.0 && far < 0.5);
    }

    #[test]
    fn modes_are_weighted() {
        let mut a = walker(0.0);
        a.modes.push(TrajectoryMode {
            probability: 3.0,
            states: vec![TimedState {
                t_s: 0.0,
                position_m: DVec3::ZERO,
                velocity_mps: -DVec3::X,
            }],
        });
        a.validate().unwrap();
        let p = a.collision_probability(0.0, DVec3::new(2.0, 0.0, 0.0), 0.1, 2.0);
        assert!((p - 0.25).abs() < 1e-12);
    }

    #[test]
    fn rejects_unsorted_states() {
        let mut a = walker(0.0);
        a.modes[0].states.insert(
            0,
            TimedState {
                t_s: 3.0,
                position_m: DVec3::ZERO,
                velocity_mps: DVec3::ZERO,
            },
        );
        assert!(a.validate().is_err());
    }

    #[test]
    fn cdf_matches_reference_points() {
        assert!((normal_cdf(0.0) - 0.5).abs() < 1e-7);
        assert!((normal_cdf(1.96) - 0.975).abs() < 1e-3);
        assert!(normal_cdf(-8.0) < 1e-6);
    }
}
