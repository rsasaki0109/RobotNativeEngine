//! Hard checks: limits, static collision and forecast risk over time.

use robot_foresight::{Certificate, Certifier, WorldForecast};
use serde::{Deserialize, Serialize};

use crate::{NavProblem, Trajectory3};

/// Result of [`check_trajectory`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TrajectoryCheck {
    /// Smallest distance (m) from the robot sphere to static occupancy
    /// (capped at the search distance).
    pub min_static_clearance_m: f64,
    /// Smallest distance (m) from the robot sphere to a predicted agent's
    /// modal position.
    pub min_agent_clearance_m: f64,
    /// Largest collision probability with predicted agents.
    pub max_risk: f64,
    /// Time-integrated collision probability (s).
    pub integrated_risk_s: f64,
    /// First violation: `(reason, time)`.
    pub violation: Option<(String, f64)>,
}

/// Checks a trajectory against a problem.
///
/// Samples every `sample_dt_s` (refined so the robot moves at most half a
/// voxel between samples) from the trajectory start until `hold_s` after its
/// end (the robot holds its final position), and checks: start position and
/// time, monotone time, speed limits, static collision, collision
/// probability against the problem's threshold, and that the end reaches the
/// goal.
#[must_use]
pub fn check_trajectory(
    problem: &NavProblem,
    trajectory: &Trajectory3,
    sample_dt_s: f64,
    hold_s: f64,
) -> TrajectoryCheck {
    let mut out = TrajectoryCheck {
        min_static_clearance_m: f64::INFINITY,
        min_agent_clearance_m: f64::INFINITY,
        max_risk: 0.0,
        integrated_risk_s: 0.0,
        violation: None,
    };
    let fail = |mut out: TrajectoryCheck, reason: &str, t: f64| {
        out.violation = Some((reason.to_owned(), t));
        out
    };
    let Some(first) = trajectory.waypoints.first() else {
        return fail(out, "empty trajectory", problem.start_t_s);
    };
    if first.position_m.distance(problem.start_m) > 1e-6
        || (first.t_s - problem.start_t_s).abs() > 1e-6
    {
        return fail(out, "does not start at the start state", first.t_s);
    }
    for w in trajectory.waypoints.windows(2) {
        let dt = w[1].t_s - w[0].t_s;
        let need = problem.min_travel_time_s(w[1].position_m - w[0].position_m);
        if dt.is_nan() || dt < 0.0 || dt + 1e-9 < need {
            return fail(out, "exceeds speed limits", w[0].t_s);
        }
    }
    let step_dt = (0.5 * problem.map.resolution_m() / problem.limits.max_speed_mps)
        .min(sample_dt_s)
        .max(1e-3);
    let t_end = trajectory.end_t_s() + hold_s.max(0.0);
    let n = ((t_end - first.t_s) / step_dt).ceil() as usize;
    let search = 2.0 * problem.map.resolution_m();
    for k in 0..=n {
        let t = (first.t_s + k as f64 * step_dt).min(t_end);
        let Some(p) = trajectory.position_at(t) else {
            break;
        };
        let clear = problem
            .map
            .sphere_clearance(p, problem.robot_radius_m, search);
        out.min_static_clearance_m = out.min_static_clearance_m.min(clear);
        if clear <= 0.0 {
            return fail(out, "static collision", t);
        }
        let risk = problem
            .forecast
            .collision_probability(p, problem.robot_radius_m, t);
        out.max_risk = out.max_risk.max(risk);
        out.integrated_risk_s += risk * step_dt;
        out.min_agent_clearance_m =
            out.min_agent_clearance_m
                .min(problem.forecast.clearance(p, problem.robot_radius_m, t));
        if risk > problem.risk_threshold {
            return fail(out, "predicted collision risk above threshold", t);
        }
    }
    if !trajectory
        .end_position_m()
        .is_some_and(|p| problem.at_goal(p))
    {
        return fail(out, "does not reach the goal", trajectory.end_t_s());
    }
    out
}

/// [`Certifier`] built on [`check_trajectory`]; margin is the smaller of the
/// static and agent clearances (m).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ForecastCertifier {
    /// Sampling interval (s).
    pub sample_dt_s: f64,
    /// Time (s) the final position must stay safe after arrival.
    pub hold_s: f64,
}

impl Default for ForecastCertifier {
    fn default() -> Self {
        Self {
            sample_dt_s: 0.05,
            hold_s: 1.0,
        }
    }
}

impl Certifier<NavProblem, Trajectory3> for ForecastCertifier {
    fn name(&self) -> &str {
        "forecast_certifier"
    }

    fn certify(&self, problem: &NavProblem, solution: &Trajectory3) -> Certificate {
        let check = check_trajectory(problem, solution, self.sample_dt_s, self.hold_s);
        match check.violation {
            Some((reason, t)) => Certificate::Rejected {
                reason,
                at_time_s: Some(t),
            },
            None => Certificate::Certified {
                margin: check
                    .min_static_clearance_m
                    .min(check.min_agent_clearance_m),
            },
        }
    }
}
