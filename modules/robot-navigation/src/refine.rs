//! Space-time shortcutting.

use robot_foresight::{ForesightError, ProposalContext, Provenance, Refiner};
use serde::{Deserialize, Serialize};

use crate::{check_trajectory, NavProblem, Trajectory3, Waypoint3};

/// Classical refiner: greedily replaces runs of waypoints by one straight
/// space-time segment when the segment respects the limits, the map and the
/// forecast, then retimes each segment to the fastest safe pace.
///
/// Waypoint times are kept when shortcutting, so a plan that waits for an
/// agent still waits; the retiming pass moves the arrival earlier only when
/// the whole remainder of the trajectory stays certified.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ShortcutRefiner {
    /// Sampling interval (s) of the segment checks.
    pub sample_dt_s: f64,
}

impl Default for ShortcutRefiner {
    fn default() -> Self {
        Self { sample_dt_s: 0.05 }
    }
}

impl ShortcutRefiner {
    fn segment_ok(&self, problem: &NavProblem, a: Waypoint3, b: Waypoint3) -> bool {
        let mut sub = problem.clone();
        sub.start_m = a.position_m;
        sub.start_t_s = a.t_s;
        sub.goal_m = b.position_m;
        sub.goal_tolerance_m = 1e-6;
        check_trajectory(&sub, &Trajectory3::new(vec![a, b]), self.sample_dt_s, 0.0)
            .violation
            .is_none()
    }

    fn shortcut(&self, problem: &NavProblem, wps: &[Waypoint3]) -> Vec<Waypoint3> {
        let mut out = vec![wps[0]];
        let mut i = 0;
        while i + 1 < wps.len() {
            let mut j = wps.len() - 1;
            while j > i + 1 && !self.segment_ok(problem, wps[i], wps[j]) {
                j -= 1;
            }
            out.push(wps[j]);
            i = j;
        }
        out
    }

    /// Shifts later waypoints earlier where the fastest pace keeps the rest
    /// of the trajectory certified.
    fn retime(&self, problem: &NavProblem, mut wps: Vec<Waypoint3>) -> Vec<Waypoint3> {
        for k in 1..wps.len() {
            let fastest = wps[k - 1].t_s
                + problem.min_travel_time_s(wps[k].position_m - wps[k - 1].position_m) / 0.999;
            let shift = wps[k].t_s - fastest;
            if shift <= 1e-6 {
                continue;
            }
            let mut trial = wps.clone();
            for w in &mut trial[k..] {
                w.t_s -= shift;
            }
            if check_trajectory(
                problem,
                &Trajectory3::new(trial.clone()),
                self.sample_dt_s,
                0.0,
            )
            .violation
            .is_none()
            {
                wps = trial;
            }
        }
        wps
    }
}

impl Refiner<NavProblem, Trajectory3> for ShortcutRefiner {
    fn name(&self) -> &str {
        "spacetime_shortcut"
    }

    fn provenance(&self) -> Provenance {
        Provenance::classical("spacetime_shortcut")
    }

    fn refine(
        &self,
        problem: &NavProblem,
        solution: &Trajectory3,
        _ctx: &mut ProposalContext,
    ) -> Result<Trajectory3, ForesightError> {
        if solution.waypoints.len() < 3 {
            return Ok(solution.clone());
        }
        // Refine only what is valid to begin with; an invalid candidate is
        // left for the certifier to reject with its own reason.
        if check_trajectory(problem, solution, self.sample_dt_s, 0.0)
            .violation
            .is_some()
        {
            return Err(ForesightError::InvalidInput(
                "candidate is not valid; not refined".into(),
            ));
        }
        let short = self.shortcut(problem, &solution.waypoints);
        Ok(Trajectory3::new(self.retime(problem, short)))
    }
}
