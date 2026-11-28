//! Critics: classical trajectory costs and a learned cost.

use robot_foresight::{Critic, ForesightError, InferenceModel, Provenance};
use serde::{Deserialize, Serialize};

use crate::{check_trajectory, NavProblem, Trajectory3};

/// Arrival time after the start (s).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct DurationCritic;

impl Critic<NavProblem, Trajectory3> for DurationCritic {
    fn name(&self) -> &str {
        "duration_s"
    }
    fn provenance(&self) -> Provenance {
        Provenance::classical("duration")
    }
    fn cost(&self, problem: &NavProblem, s: &Trajectory3) -> Result<f64, ForesightError> {
        Ok(s.end_t_s() - problem.start_t_s)
    }
}

/// Path length (m).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct PathLengthCritic;

impl Critic<NavProblem, Trajectory3> for PathLengthCritic {
    fn name(&self) -> &str {
        "length_m"
    }
    fn provenance(&self) -> Provenance {
        Provenance::classical("path_length")
    }
    fn cost(&self, _: &NavProblem, s: &Trajectory3) -> Result<f64, ForesightError> {
        Ok(s.length_m())
    }
}

/// Time-integrated predicted collision probability (s).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RiskCritic {
    /// Sampling interval (s).
    pub sample_dt_s: f64,
}

impl Default for RiskCritic {
    fn default() -> Self {
        Self { sample_dt_s: 0.05 }
    }
}

impl Critic<NavProblem, Trajectory3> for RiskCritic {
    fn name(&self) -> &str {
        "integrated_risk_s"
    }
    fn provenance(&self) -> Provenance {
        Provenance::classical("forecast_risk")
    }
    fn cost(&self, problem: &NavProblem, s: &Trajectory3) -> Result<f64, ForesightError> {
        // Risk accumulated up to the first violation, if any; the certifier
        // rejects violating candidates separately.
        let mut relaxed = problem.clone();
        relaxed.risk_threshold = 1.0;
        Ok(check_trajectory(&relaxed, s, self.sample_dt_s, 0.0).integrated_risk_s)
    }
}

/// Number of [`trajectory_features`].
pub const TRAJECTORY_FEATURES: usize = 6;

/// Summary features of a trajectory for learned critics: duration (s),
/// length (m), maximum risk, integrated risk (s), smallest agent clearance
/// (m, capped at 10) and mean height above the start (m).
#[must_use]
pub fn trajectory_features(problem: &NavProblem, s: &Trajectory3) -> [f32; TRAJECTORY_FEATURES] {
    let mut relaxed = problem.clone();
    relaxed.risk_threshold = 1.0;
    let check = check_trajectory(&relaxed, s, 0.05, 0.0);
    let samples = s.sample(0.1);
    let mean_dz = if samples.is_empty() {
        0.0
    } else {
        samples
            .iter()
            .map(|w| w.position_m.z - problem.start_m.z)
            .sum::<f64>()
            / samples.len() as f64
    };
    [
        s.duration_s() as f32,
        s.length_m() as f32,
        check.max_risk as f32,
        check.integrated_risk_s as f32,
        check.min_agent_clearance_m.min(10.0) as f32,
        mean_dz as f32,
    ]
}

/// Learned critic: a model maps [`trajectory_features`] to one cost value
/// (for example a social-comfort or energy model fitted to demonstrations).
#[derive(Clone, Debug)]
pub struct LearnedCostCritic<M> {
    /// The model (6 inputs, 1 output).
    pub model: M,
}

impl<M: InferenceModel> LearnedCostCritic<M> {
    /// Wraps a model with 6 inputs and 1 output.
    ///
    /// # Errors
    ///
    /// Returns [`ForesightError::InvalidConfig`] for any other shape.
    pub fn new(model: M) -> Result<Self, ForesightError> {
        let spec = model.spec();
        if spec.input_dim != TRAJECTORY_FEATURES || spec.output_dim != 1 {
            return Err(ForesightError::InvalidConfig(format!(
                "cost model `{}` must map 6 -> 1",
                spec.model_id
            )));
        }
        Ok(Self { model })
    }
}

impl<M: InferenceModel> Critic<NavProblem, Trajectory3> for LearnedCostCritic<M> {
    fn name(&self) -> &str {
        &self.model.spec().model_id
    }
    fn provenance(&self) -> Provenance {
        Provenance::learned(&self.model.spec().model_id)
    }
    fn cost(&self, problem: &NavProblem, s: &Trajectory3) -> Result<f64, ForesightError> {
        Ok(f64::from(
            self.model.infer(&trajectory_features(problem, s))?[0],
        ))
    }
}
