//! Path-velocity decomposition: retime a geometric path around the forecast.

use robot_foresight::ForesightError;
use serde::{Deserialize, Serialize};

use crate::{JointTrajectory, JointWaypoint, MotionProblem};

/// Settings of [`time_scale_path`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TimeScaling {
    /// Time step (s) of the search and of the output waypoints.
    pub dt_s: f64,
    /// Path samples the arm may advance per step at full speed.
    pub substeps: usize,
    /// Minimum static clearance (m) along the path.
    pub static_margin_m: f64,
}

impl Default for TimeScaling {
    fn default() -> Self {
        Self {
            dt_s: 0.1,
            substeps: 4,
            static_margin_m: 0.0,
        }
    }
}

/// Samples `path` so that consecutive samples are `step_s` apart at the
/// velocity limits.
fn resample(problem: &MotionProblem, path: &[Vec<f64>], step_s: f64) -> Vec<Vec<f64>> {
    let seg: Vec<f64> = path
        .windows(2)
        .map(|w| problem.min_time_s(&w[0], &w[1]))
        .collect();
    let total: f64 = seg.iter().sum();
    let n = (total / step_s).ceil().max(1.0) as usize;
    let mut out = Vec::with_capacity(n + 1);
    let (mut i, mut acc) = (0usize, 0.0);
    for k in 0..=n {
        let target = total * k as f64 / n as f64;
        while i + 1 < seg.len() && acc + seg[i] < target {
            acc += seg[i];
            i += 1;
        }
        let u = if seg.is_empty() || seg[i] <= 0.0 {
            1.0
        } else {
            ((target - acc) / seg[i]).clamp(0.0, 1.0)
        };
        let (a, b) = if seg.is_empty() {
            (&path[0], &path[0])
        } else {
            (&path[i], &path[i + 1])
        };
        out.push(a.iter().zip(b).map(|(x, y)| x + (y - x) * u).collect());
    }
    out
}

/// Retimes a geometric joint path against the problem's forecast.
///
/// The path is resampled so that one sample takes `dt_s / substeps` at the
/// velocity limits; then a breadth-first search over time layers of `dt_s`
/// tracks every reachable path sample, letting the arm advance by up to
/// `substeps` samples, wait, or anything between. A sample is admissible at
/// a step when every link sphere's collision probability is at most the
/// problem threshold at the start, middle and end of that step (every
/// sample swept in the step is checked). The result is the earliest arrival; with nothing moving
/// it is the path at full speed.
///
/// # Errors
///
/// Returns [`ForesightError::NoSolution`] when the path collides with the
/// static scene, the start is already at risk, or the goal cannot be
/// reached within `max_duration_s`.
pub fn time_scale_path(
    problem: &MotionProblem,
    path: &[Vec<f64>],
    scaling: &TimeScaling,
) -> Result<JointTrajectory, ForesightError> {
    if path.is_empty() || scaling.substeps == 0 || scaling.dt_s.is_nan() || scaling.dt_s <= 0.0 {
        return Err(ForesightError::InvalidInput(
            "empty path or invalid time scaling".into(),
        ));
    }
    for q in path {
        problem.chain.check_q(q)?;
        if !problem.chain.within_limits(q) {
            return Err(ForesightError::NoSolution(
                "path leaves the joint limits".into(),
            ));
        }
    }
    let samples = resample(problem, path, scaling.dt_s / scaling.substeps as f64);
    if let Some(k) = samples
        .iter()
        .position(|q| problem.static_clearance(q) <= scaling.static_margin_m)
    {
        return Err(ForesightError::NoSolution(format!(
            "path collides with the scene at sample {k}"
        )));
    }
    let n = samples.len() - 1;
    let t0 = problem.start_t_s;
    if problem.risk(&samples[0], t0) > problem.risk_threshold {
        return Err(ForesightError::NoSolution(
            "start configuration is at predicted risk".into(),
        ));
    }
    let layers = (problem.max_duration_s / scaling.dt_s).ceil() as usize;
    // parents[k][s] = path sample at layer k - 1 that led to s, or usize::MAX.
    let mut parents: Vec<Vec<usize>> = vec![vec![usize::MAX; n + 1]];
    parents[0][0] = 0;
    let (mut lo, mut hi) = (0usize, 0usize);
    for k in 1..=layers {
        let t = t0 + k as f64 * scaling.dt_s;
        let prev = &parents[k - 1];
        let top = (hi + scaling.substeps).min(n);
        // A sample swept during the step is admissible only if it is safe at
        // the start, middle and end of the step, so the piecewise-linear
        // trajectory stays safe in between.
        let valid: Vec<bool> = (lo..=top)
            .map(|s| {
                [t - scaling.dt_s, t - 0.5 * scaling.dt_s, t]
                    .iter()
                    .all(|&tt| problem.risk(&samples[s], tt) <= problem.risk_threshold)
            })
            .collect();
        let mut row = vec![usize::MAX; n + 1];
        let (mut new_lo, mut new_hi) = (usize::MAX, 0usize);
        #[allow(clippy::needless_range_loop)] // `s` indexes `row`, `valid` and `prev`.
        for s in lo..=top {
            // Walk back over consecutive admissible samples to a reachable one.
            let mut b = s;
            loop {
                if !valid[b - lo] {
                    break;
                }
                if prev[b] != usize::MAX {
                    row[s] = b;
                    break;
                }
                if b == lo || s - b >= scaling.substeps {
                    break;
                }
                b -= 1;
            }
            if row[s] != usize::MAX {
                new_lo = new_lo.min(s);
                new_hi = new_hi.max(s);
            }
        }
        parents.push(row);
        if new_lo == usize::MAX {
            return Err(ForesightError::NoSolution(format!(
                "every path sample is at predicted risk at t = {t:.2} s"
            )));
        }
        if parents[k][n] != usize::MAX {
            return Ok(backtrack(&parents, &samples, t0, scaling.dt_s));
        }
        (lo, hi) = (new_lo, new_hi);
    }
    Err(ForesightError::NoSolution(
        "goal not reached within max_duration_s".into(),
    ))
}

fn backtrack(parents: &[Vec<usize>], samples: &[Vec<f64>], t0: f64, dt: f64) -> JointTrajectory {
    let mut s = samples.len() - 1;
    let mut rev = Vec::with_capacity(parents.len());
    for k in (0..parents.len()).rev() {
        rev.push(JointWaypoint {
            t_s: t0 + k as f64 * dt,
            q: samples[s].clone(),
        });
        s = parents[k][s];
    }
    rev.reverse();
    JointTrajectory { waypoints: rev }
}
