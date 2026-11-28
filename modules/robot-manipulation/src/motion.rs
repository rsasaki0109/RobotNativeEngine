//! Motion proposers, critics and the motion certifier.

use robot_foresight::{
    Certificate, Certifier, Critic, ForesightError, InferenceModel, ProposalContext, Proposer,
    Provenance, SplitMix64,
};
use serde::{Deserialize, Serialize};

use crate::{time_scale_path, JointTrajectory, MotionProblem, TimeScaling};

/// Classical proposer: the straight joint-space path, retimed around the
/// forecast.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DirectJointProposer {
    /// Retiming settings.
    pub scaling: TimeScaling,
}

impl Proposer<MotionProblem, JointTrajectory> for DirectJointProposer {
    fn name(&self) -> &str {
        "direct_joint"
    }
    fn provenance(&self) -> Provenance {
        Provenance::classical("direct_joint+time_scaling")
    }
    fn propose(
        &self,
        p: &MotionProblem,
        _: &mut ProposalContext,
    ) -> Result<Vec<JointTrajectory>, ForesightError> {
        p.validate()?;
        Ok(vec![time_scale_path(
            p,
            &[p.start_q.clone(), p.goal_q.clone()],
            &self.scaling,
        )?])
    }
}

/// Classical proposer: RRT-Connect around the static scene, randomly
/// shortcut, then retimed around the forecast.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RrtConnectProposer {
    /// Largest joint-space step (rad, Euclidean).
    pub step_rad: f64,
    /// Iteration budget.
    pub max_iterations: usize,
    /// Shortcut attempts.
    pub shortcut_iterations: usize,
    /// Edge checking resolution (rad).
    pub check_resolution_rad: f64,
    /// Retiming settings.
    pub scaling: TimeScaling,
}

impl Default for RrtConnectProposer {
    fn default() -> Self {
        Self {
            step_rad: 0.3,
            max_iterations: 5000,
            shortcut_iterations: 100,
            check_resolution_rad: 0.05,
            scaling: TimeScaling::default(),
        }
    }
}

fn dist(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).powi(2))
        .sum::<f64>()
        .sqrt()
}

fn steer(from: &[f64], to: &[f64], step: f64) -> Vec<f64> {
    let d = dist(from, to);
    if d <= step {
        return to.to_vec();
    }
    from.iter()
        .zip(to)
        .map(|(a, b)| a + (b - a) * step / d)
        .collect()
}

impl RrtConnectProposer {
    fn edge_free(&self, p: &MotionProblem, a: &[f64], b: &[f64]) -> bool {
        let n = (dist(a, b) / self.check_resolution_rad).ceil().max(1.0) as usize;
        (0..=n).all(|k| {
            let u = k as f64 / n as f64;
            let q: Vec<f64> = a.iter().zip(b).map(|(x, y)| x + (y - x) * u).collect();
            p.static_clearance(&q) > self.scaling.static_margin_m
        })
    }

    /// Extends `tree` towards `target`; returns the new node index.
    fn extend(
        &self,
        p: &MotionProblem,
        tree: &mut Vec<(Vec<f64>, usize)>,
        target: &[f64],
    ) -> Option<usize> {
        let near = (0..tree.len())
            .min_by(|&i, &j| dist(&tree[i].0, target).total_cmp(&dist(&tree[j].0, target)))?;
        let q = steer(&tree[near].0, target, self.step_rad);
        if !self.edge_free(p, &tree[near].0, &q) {
            return None;
        }
        tree.push((q, near));
        Some(tree.len() - 1)
    }

    fn branch(tree: &[(Vec<f64>, usize)], mut i: usize) -> Vec<Vec<f64>> {
        let mut out = vec![tree[i].0.clone()];
        while i != 0 {
            i = tree[i].1;
            out.push(tree[i].0.clone());
        }
        out
    }

    /// Geometric path around the static scene.
    ///
    /// # Errors
    ///
    /// Returns [`ForesightError::NoSolution`] when the budget runs out.
    pub fn geometric_path(
        &self,
        p: &MotionProblem,
        rng: &mut SplitMix64,
    ) -> Result<Vec<Vec<f64>>, ForesightError> {
        if self.edge_free(p, &p.start_q, &p.goal_q) {
            return Ok(vec![p.start_q.clone(), p.goal_q.clone()]);
        }
        let mut a = vec![(p.start_q.clone(), 0usize)];
        let mut b = vec![(p.goal_q.clone(), 0usize)];
        let mut a_is_start = true;
        for _ in 0..self.max_iterations {
            let sample: Vec<f64> = p
                .chain
                .joints
                .iter()
                .map(|j| rng.uniform(j.lower_rad, j.upper_rad))
                .collect();
            if let Some(new) = self.extend(p, &mut a, &sample) {
                let target = a[new].0.clone();
                // Connect: grow the other tree towards the new node.
                while let Some(i) = self.extend(p, &mut b, &target) {
                    if dist(&b[i].0, &target) < 1e-9 {
                        let mut path = Self::branch(&a, new);
                        path.reverse();
                        path.extend(Self::branch(&b, i).into_iter().skip(1));
                        if !a_is_start {
                            path.reverse();
                        }
                        return Ok(self.shortcut(p, path, rng));
                    }
                }
            }
            std::mem::swap(&mut a, &mut b);
            a_is_start = !a_is_start;
        }
        Err(ForesightError::NoSolution(
            "RRT-Connect budget exhausted".into(),
        ))
    }

    fn shortcut(
        &self,
        p: &MotionProblem,
        mut path: Vec<Vec<f64>>,
        rng: &mut SplitMix64,
    ) -> Vec<Vec<f64>> {
        for _ in 0..self.shortcut_iterations {
            if path.len() < 3 {
                break;
            }
            let i = rng.index(path.len() - 2);
            let j = i + 2 + rng.index(path.len() - i - 2);
            if self.edge_free(p, &path[i], &path[j]) {
                path.drain(i + 1..j);
            }
        }
        path
    }
}

impl Proposer<MotionProblem, JointTrajectory> for RrtConnectProposer {
    fn name(&self) -> &str {
        "rrt_connect"
    }
    fn provenance(&self) -> Provenance {
        Provenance::classical("rrt_connect+time_scaling")
    }
    fn propose(
        &self,
        p: &MotionProblem,
        ctx: &mut ProposalContext,
    ) -> Result<Vec<JointTrajectory>, ForesightError> {
        p.validate()?;
        let path = self.geometric_path(p, &mut ctx.rng)?;
        Ok(vec![time_scale_path(p, &path, &self.scaling)?])
    }
}

/// Learned proposer: a motion prior predicts a via configuration; the path
/// start -> via -> goal is retimed around the forecast.
///
/// Model input (`2 * dof + 3`): start configuration, goal configuration,
/// and the position of the nearest predicted agent relative to the arm base
/// at the start time (`[10, 10, 10]` with none). Output (`dof`): the via
/// configuration, clamped to the joint limits.
#[derive(Clone, Debug)]
pub struct LearnedViaPointProposer<M> {
    /// The motion prior.
    pub model: M,
    /// Retiming settings.
    pub scaling: TimeScaling,
}

impl<M: InferenceModel> LearnedViaPointProposer<M> {
    /// Wraps a model for an arm with `dof` joints.
    ///
    /// # Errors
    ///
    /// Returns [`ForesightError::InvalidConfig`] for a mismatched shape.
    pub fn new(model: M, dof: usize) -> Result<Self, ForesightError> {
        let s = model.spec();
        if s.input_dim != 2 * dof + 3 || s.output_dim != dof {
            return Err(ForesightError::InvalidConfig(format!(
                "motion prior `{}` must map {} -> {dof}",
                s.model_id,
                2 * dof + 3
            )));
        }
        Ok(Self {
            model,
            scaling: TimeScaling::default(),
        })
    }
}

impl<M: InferenceModel> Proposer<MotionProblem, JointTrajectory> for LearnedViaPointProposer<M> {
    fn name(&self) -> &str {
        "learned_via_point"
    }
    fn provenance(&self) -> Provenance {
        Provenance::learned(&self.model.spec().model_id).then(Provenance::classical("time_scaling"))
    }
    fn propose(
        &self,
        p: &MotionProblem,
        _: &mut ProposalContext,
    ) -> Result<Vec<JointTrajectory>, ForesightError> {
        p.validate()?;
        let base = p.chain.base.translation;
        let agent = p
            .forecast
            .agents
            .iter()
            .filter_map(|a| a.modes.first().and_then(|m| m.position_at(p.start_t_s)))
            .min_by(|x, y| x.distance(base).total_cmp(&y.distance(base)))
            .map_or(glam::DVec3::splat(10.0), |x| x - base);
        let input: Vec<f32> = p
            .start_q
            .iter()
            .chain(&p.goal_q)
            .copied()
            .chain([agent.x, agent.y, agent.z])
            .map(|v| v as f32)
            .collect();
        let via: Vec<f64> = self
            .model
            .infer(&input)?
            .into_iter()
            .map(f64::from)
            .collect();
        let via = p.chain.clamp(&via);
        Ok(vec![time_scale_path(
            p,
            &[p.start_q.clone(), via, p.goal_q.clone()],
            &self.scaling,
        )?])
    }
}

/// Hard checks: start, goal, joint and velocity limits, static clearance
/// and forecast risk sampled every `sample_dt_s`, plus `hold_s` at the goal.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MotionCertifier {
    /// Sampling interval (s).
    pub sample_dt_s: f64,
    /// Time (s) the goal configuration must stay safe after arrival.
    pub hold_s: f64,
}

impl Default for MotionCertifier {
    fn default() -> Self {
        Self {
            sample_dt_s: 0.05,
            hold_s: 0.5,
        }
    }
}

impl Certifier<MotionProblem, JointTrajectory> for MotionCertifier {
    fn name(&self) -> &str {
        "motion_certifier"
    }

    fn certify(&self, p: &MotionProblem, s: &JointTrajectory) -> Certificate {
        let reject = |reason: &str, t: f64| Certificate::Rejected {
            reason: reason.to_owned(),
            at_time_s: Some(t),
        };
        let (Some(first), Some(last)) = (s.waypoints.first(), s.waypoints.last()) else {
            return reject("empty trajectory", p.start_t_s);
        };
        if dist(&first.q, &p.start_q) > 1e-6 || (first.t_s - p.start_t_s).abs() > 1e-6 {
            return reject("does not start at the start state", first.t_s);
        }
        if dist(&last.q, &p.goal_q) > 1e-6 {
            return reject("does not reach the goal", last.t_s);
        }
        for w in s.waypoints.windows(2) {
            if !p.chain.within_limits(&w[1].q) {
                return reject("joint limits", w[1].t_s);
            }
            if w[1].t_s - w[0].t_s + 1e-9 < p.min_time_s(&w[0].q, &w[1].q) {
                return reject("velocity limits", w[0].t_s);
            }
        }
        let t_end = s.end_t_s() + self.hold_s;
        let n = ((t_end - first.t_s) / self.sample_dt_s).ceil() as usize;
        let mut margin = f64::INFINITY;
        for k in 0..=n {
            let t = (first.t_s + k as f64 * self.sample_dt_s).min(t_end);
            let Some(q) = s.q_at(t) else { break };
            let clear = p.static_clearance(&q);
            if clear <= 0.0 {
                return reject("static collision", t);
            }
            if p.risk(&q, t) > p.risk_threshold {
                return reject("predicted collision risk above threshold", t);
            }
            margin = margin.min(clear).min(p.agent_clearance(&q, t));
        }
        Certificate::Certified { margin }
    }
}

/// Arrival time after the start (s).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct MotionDurationCritic;

impl Critic<MotionProblem, JointTrajectory> for MotionDurationCritic {
    fn name(&self) -> &str {
        "duration_s"
    }
    fn provenance(&self) -> Provenance {
        Provenance::classical("duration")
    }
    fn cost(&self, p: &MotionProblem, s: &JointTrajectory) -> Result<f64, ForesightError> {
        Ok(s.end_t_s() - p.start_t_s)
    }
}

/// Joint-space path length (rad).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct JointPathLengthCritic;

impl Critic<MotionProblem, JointTrajectory> for JointPathLengthCritic {
    fn name(&self) -> &str {
        "joint_path_rad"
    }
    fn provenance(&self) -> Provenance {
        Provenance::classical("joint_path_length")
    }
    fn cost(&self, _: &MotionProblem, s: &JointTrajectory) -> Result<f64, ForesightError> {
        Ok(s.path_length_rad())
    }
}

/// Time-integrated largest per-sphere collision probability (s).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MotionRiskCritic {
    /// Sampling interval (s).
    pub sample_dt_s: f64,
}

impl Default for MotionRiskCritic {
    fn default() -> Self {
        Self { sample_dt_s: 0.05 }
    }
}

impl Critic<MotionProblem, JointTrajectory> for MotionRiskCritic {
    fn name(&self) -> &str {
        "integrated_risk_s"
    }
    fn provenance(&self) -> Provenance {
        Provenance::classical("forecast_risk")
    }
    fn cost(&self, p: &MotionProblem, s: &JointTrajectory) -> Result<f64, ForesightError> {
        let n = (s.duration_s() / self.sample_dt_s).ceil() as usize;
        Ok((0..=n)
            .filter_map(|k| {
                let t = s.start_t_s() + k as f64 * self.sample_dt_s;
                s.q_at(t).map(|q| p.risk(&q, t) * self.sample_dt_s)
            })
            .sum())
    }
}
