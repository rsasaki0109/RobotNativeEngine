//! Direct proposers: a classical straight line and a learned velocity policy.

use glam::DVec3;
use robot_foresight::{
    ForecastSet, ForesightError, InferenceModel, ProposalContext, Proposer, Provenance,
};
use serde::{Deserialize, Serialize};

use crate::{NavProblem, Trajectory3, Waypoint3};

/// Classical proposer: the straight segment to the goal at several fractions
/// of the speed limit (a one-dimensional velocity search: slower candidates
/// let a crossing agent pass first).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StraightLineProposer {
    /// Speed fractions in `(0, 1]`, one candidate each.
    pub speed_fractions: Vec<f64>,
}

impl Default for StraightLineProposer {
    fn default() -> Self {
        Self {
            speed_fractions: vec![1.0, 0.7, 0.45],
        }
    }
}

impl Proposer<NavProblem, Trajectory3> for StraightLineProposer {
    fn name(&self) -> &str {
        "straight_line"
    }

    fn provenance(&self) -> Provenance {
        Provenance::classical("straight_line")
    }

    fn propose(
        &self,
        problem: &NavProblem,
        _ctx: &mut ProposalContext,
    ) -> Result<Vec<Trajectory3>, ForesightError> {
        let t = problem.min_travel_time_s(problem.goal_m - problem.start_m);
        if !t.is_finite() {
            return Err(ForesightError::NoSolution(
                "goal height unreachable under limits".into(),
            ));
        }
        Ok(self
            .speed_fractions
            .iter()
            .filter(|f| **f > 0.0 && **f <= 1.0)
            .map(|f| {
                Trajectory3::new(vec![
                    Waypoint3 {
                        t_s: problem.start_t_s,
                        position_m: problem.start_m,
                    },
                    Waypoint3 {
                        t_s: problem.start_t_s + t / f,
                        position_m: problem.goal_m,
                    },
                ])
            })
            .collect())
    }
}

/// Input size of [`LearnedVelocityProposer`] models.
pub const LEARNED_VELOCITY_INPUTS: usize = 12;

/// Most likely position and velocity of the agent nearest to `p` at `t_s`.
fn nearest_agent(forecast: &ForecastSet, p: DVec3, t_s: f64) -> Option<(DVec3, DVec3)> {
    forecast
        .agents
        .iter()
        .filter_map(|a| {
            let mode = a.modes.iter().fold(
                None::<&robot_foresight::TrajectoryMode>,
                |best, m| match best {
                    Some(b) if b.probability >= m.probability => Some(b),
                    _ => Some(m),
                },
            )?;
            let now = mode.position_at(t_s)?;
            let next = mode.position_at(t_s + 0.1)?;
            Some((now, (next - now) / 0.1))
        })
        .fold(None::<(DVec3, DVec3)>, |best, cand| match best {
            Some(b) if b.0.distance(p) <= cand.0.distance(p) => Some(b),
            _ => Some(cand),
        })
}

/// Feature vector of [`LearnedVelocityProposer`] at position `p`, velocity
/// `v` and time `t_s`: goal offset (3), nearest agent offset (3), nearest
/// agent velocity (3), current velocity (3). With no agents the agent
/// offset is `[100, 100, 100]` and its velocity zero.
#[must_use]
pub fn learned_velocity_features(
    problem: &NavProblem,
    p: DVec3,
    v: DVec3,
    t_s: f64,
) -> [f32; LEARNED_VELOCITY_INPUTS] {
    let goal = problem.goal_m - p;
    let (rel, av) = nearest_agent(&problem.forecast, p, t_s)
        .map_or((DVec3::splat(100.0), DVec3::ZERO), |(a, av)| (a - p, av));
    let mut out = [0f32; LEARNED_VELOCITY_INPUTS];
    for (i, val) in [goal, rel, av, v]
        .iter()
        .flat_map(|d| [d.x, d.y, d.z])
        .enumerate()
    {
        out[i] = val as f32;
    }
    out
}

/// Clips a velocity to the limits (with a small safety factor so the
/// result passes the certifier's speed check exactly).
fn clip_velocity(problem: &NavProblem, v: DVec3) -> DVec3 {
    let scale = 0.999;
    let mut v = v;
    let vz_max = problem.limits.max_vertical_speed_mps * scale;
    if v.z.abs() > vz_max {
        v.z = v.z.signum() * vz_max;
    }
    let max = problem.limits.max_speed_mps * scale;
    if v.length() > max {
        v *= max / v.length();
    }
    v
}

/// Learned proposer: rolls out a velocity policy.
///
/// At every step of `dt_s` the model maps [`learned_velocity_features`] to a
/// velocity command `[vx, vy, vz]`, which is clipped to the limits and
/// integrated. Candidate 0 is the deterministic rollout; the other
/// `samples - 1` add seeded Gaussian noise of `noise_mps` to every command.
/// The policy never sees the voxel map, so its proposals rely on the
/// certifier for static safety.
#[derive(Clone, Debug)]
pub struct LearnedVelocityProposer<M> {
    /// The policy.
    pub model: M,
    /// Control period (s).
    pub dt_s: f64,
    /// Longest rollout (steps).
    pub max_steps: usize,
    /// Number of rollouts.
    pub samples: usize,
    /// Command noise (m/s) of the stochastic rollouts.
    pub noise_mps: f64,
}

impl<M: InferenceModel> LearnedVelocityProposer<M> {
    /// Wraps a model with 12 inputs and 3 outputs.
    ///
    /// # Errors
    ///
    /// Returns [`ForesightError::InvalidConfig`] for any other shape.
    pub fn new(model: M) -> Result<Self, ForesightError> {
        let spec = model.spec();
        if spec.input_dim != LEARNED_VELOCITY_INPUTS || spec.output_dim != 3 {
            return Err(ForesightError::InvalidConfig(format!(
                "velocity policy `{}` must map 12 -> 3",
                spec.model_id
            )));
        }
        Ok(Self {
            model,
            dt_s: 0.1,
            max_steps: 600,
            samples: 4,
            noise_mps: 0.3,
        })
    }

    fn rollout(
        &self,
        problem: &NavProblem,
        ctx: &mut ProposalContext,
        noisy: bool,
    ) -> Result<Trajectory3, ForesightError> {
        let (mut p, mut v, mut t) = (problem.start_m, DVec3::ZERO, problem.start_t_s);
        let mut waypoints = vec![Waypoint3 {
            t_s: t,
            position_m: p,
        }];
        for _ in 0..self.max_steps {
            if problem.at_goal(p) {
                break;
            }
            let out = self
                .model
                .infer(&learned_velocity_features(problem, p, v, t))?;
            let mut cmd = DVec3::new(f64::from(out[0]), f64::from(out[1]), f64::from(out[2]));
            if noisy {
                cmd += DVec3::new(ctx.rng.normal(), ctx.rng.normal(), ctx.rng.normal())
                    * self.noise_mps;
            }
            v = clip_velocity(problem, cmd);
            p += v * self.dt_s;
            t += self.dt_s;
            waypoints.push(Waypoint3 {
                t_s: t,
                position_m: p,
            });
        }
        if problem.at_goal(p) && p.distance(problem.goal_m) > 1e-9 {
            let dt = problem.min_travel_time_s(problem.goal_m - p);
            if dt.is_finite() {
                waypoints.push(Waypoint3 {
                    t_s: t + dt / 0.999,
                    position_m: problem.goal_m,
                });
            }
        }
        Ok(Trajectory3::new(waypoints))
    }
}

impl<M: InferenceModel> Proposer<NavProblem, Trajectory3> for LearnedVelocityProposer<M> {
    fn name(&self) -> &str {
        "learned_velocity_policy"
    }

    fn provenance(&self) -> Provenance {
        Provenance::learned(&self.model.spec().model_id)
    }

    fn propose(
        &self,
        problem: &NavProblem,
        ctx: &mut ProposalContext,
    ) -> Result<Vec<Trajectory3>, ForesightError> {
        (0..self.samples.max(1))
            .map(|k| self.rollout(problem, ctx, k > 0))
            .collect()
    }
}
