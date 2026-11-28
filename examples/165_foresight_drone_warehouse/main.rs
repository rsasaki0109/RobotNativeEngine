//! Example 165: a drone in a warehouse aisle plans through space *and* time.
//!
//! A forklift drives head-on down the aisle, and a worker stands at the end
//! of a cross-aisle, about to cross. The same planner is run on three
//! models of the world, and each plan is then executed against what really
//! happens:
//!
//! 1. **snapshot**: nothing is moving (what a costmap of the present gives);
//! 2. **constant velocity**: a classical forecast (the worker is standing,
//!    so it predicts that they stay put);
//! 3. **blended**: a learned intent model (a stand-in here, see
//!    `intent_model`) mixed with the constant-velocity forecast, so the
//!    worker may cross.
//!
//! The planner is one `robot_foresight::HybridPipeline`: a classical
//! straight line and space-time A*, plus a learned velocity policy, all
//! refined, priced and certified by the same stages.
//!
//! ```bash
//! cargo run -p foresight_drone_warehouse --example 165_foresight_drone_warehouse
//! ```

use std::sync::Arc;

use glam::DVec3;
use robot_foresight::{
    AgentTrack, BlendedForecaster, ConstantVelocityForecaster, FnModel, ForecastSet, Forecaster,
    LearnedForecaster, TimedState,
};
use robot_navigation::{
    default_pipeline, KinematicLimits, LearnedVelocityProposer, NavProblem, Trajectory3, VoxelGrid,
};

const HORIZON_S: f64 = 15.0;
const FORKLIFT_RADIUS_M: f64 = 0.9;
const WORKER_RADIUS_M: f64 = 0.6;
const DRONE_RADIUS_M: f64 = 0.25;
/// Walking speed of the worker, and of the intent model's crossing mode.
const WORKER_SPEED_MPS: f64 = 0.9;

/// Where the two agents really are at `t_s`.
fn ground_truth(t_s: f64) -> [(DVec3, f64); 2] {
    let forklift = DVec3::new(18.0 - 1.0 * t_s, 6.0, 1.0);
    // The worker, standing until now, starts crossing at 0.9 m/s.
    let worker_y = 0.6 + WORKER_SPEED_MPS * t_s.max(0.0);
    let worker = DVec3::new(10.0, worker_y.min(11.4), 1.2);
    [(forklift, FORKLIFT_RADIUS_M), (worker, WORKER_RADIUS_M)]
}

/// Tracks observed up to t = 0 (two observations each).
fn observed_tracks() -> Vec<AgentTrack> {
    let obs = |id, radius_m, at: &dyn Fn(f64) -> DVec3| AgentTrack {
        agent_id: id,
        radius_m,
        history: [-0.5, 0.0]
            .iter()
            .map(|&t| TimedState {
                t_s: t,
                position_m: at(t),
                velocity_mps: DVec3::ZERO,
            })
            .collect(),
    };
    vec![
        obs(1, FORKLIFT_RADIUS_M, &|t| DVec3::new(18.0 - t, 6.0, 1.0)),
        obs(2, WORKER_RADIUS_M, &|_| DVec3::new(10.0, 0.6, 1.2)),
    ]
}

/// Stand-in for a trained intent model (2 past offsets + velocity in;
/// 2 modes x [logit, vx, vy, vz] out): an agent keeps its velocity, and a
/// standing agent at a cross-aisle end may also start crossing (+y) at
/// walking speed. A trained network replaces this closure through the same
/// `InferenceModel` interface.
fn intent_model() -> FnModel<impl Fn(&[f32]) -> Vec<f32>> {
    FnModel::new("aisle_intent_standin_v0", 9, 8, |x| {
        let v = [x[6], x[7], x[8]];
        let standing = v.iter().map(|c| c * c).sum::<f32>() < 0.01;
        let cross_logit = if standing { 0.4 } else { -20.0 };
        vec![
            0.0,
            v[0],
            v[1],
            v[2],
            cross_logit,
            0.0,
            WORKER_SPEED_MPS as f32,
            0.0,
        ]
    })
}

/// Stand-in for a trained velocity policy: head for the goal at 1.8 m/s.
fn policy_model() -> FnModel<impl Fn(&[f32]) -> Vec<f32>> {
    FnModel::new("goal_seeker_standin_v0", 12, 3, |x| {
        let g = glam::Vec3::new(x[0], x[1], 0.0).normalize_or_zero() * 1.8;
        vec![g.x, g.y, 0.0]
    })
}

fn warehouse() -> VoxelGrid {
    // 20 m x 12 m x 3 m at 0.25 m; shelves fill y < 4 and y > 8 to the
    // ceiling except a 2 m cross-aisle at x = 9..11.
    let mut map = VoxelGrid::new(DVec3::ZERO, 0.25, [80, 48, 12]).expect("grid");
    for (x0, x1) in [(2.0, 9.0), (11.0, 18.5)] {
        map.fill_box(DVec3::new(x0, 0.0, 0.0), DVec3::new(x1, 4.0, 3.0));
        map.fill_box(DVec3::new(x0, 8.0, 0.0), DVec3::new(x1, 12.0, 3.0));
    }
    map
}

/// Outcome of one variant.
#[derive(Debug)]
struct Outcome {
    name: &'static str,
    plan: Option<Trajectory3>,
    summary: String,
    /// Smallest true surface distance to an agent during execution (m).
    min_true_clearance_m: f64,
}

fn execute(traj: &Trajectory3) -> f64 {
    traj.sample(0.02)
        .iter()
        .flat_map(|w| {
            ground_truth(w.t_s).map(|(c, r)| w.position_m.distance(c) - r - DRONE_RADIUS_M)
        })
        .fold(f64::INFINITY, f64::min)
}

fn plan(name: &'static str, forecast: ForecastSet) -> Outcome {
    let problem = NavProblem {
        map: Arc::new(warehouse()),
        forecast: Arc::new(forecast),
        start_m: DVec3::new(1.0, 6.0, 1.5),
        start_t_s: 0.0,
        goal_m: DVec3::new(19.0, 6.0, 1.5),
        goal_tolerance_m: 0.2,
        robot_radius_m: DRONE_RADIUS_M,
        limits: KinematicLimits {
            max_speed_mps: 2.0,
            max_vertical_speed_mps: 0.8,
        },
        risk_threshold: 0.05,
    };
    let pipeline = default_pipeline(165)
        .with_proposer(LearnedVelocityProposer::new(policy_model()).expect("policy shape"));
    let decision = pipeline.decide(&problem);
    let summary = decision.summary();
    let plan = decision.into_solution();
    let min_true_clearance_m = plan.as_ref().map_or(f64::NAN, execute);
    Outcome {
        name,
        plan,
        summary,
        min_true_clearance_m,
    }
}

fn run() -> Vec<Outcome> {
    let tracks = observed_tracks();
    let cv = ConstantVelocityForecaster {
        dt_s: 0.1,
        sigma0_m: 0.05,
        sigma_growth_mps: 0.04,
    };
    let mut learned = LearnedForecaster::new(intent_model(), 2, 2).expect("intent shape");
    learned.sigma_growth_mps = 0.04;
    let blended = BlendedForecaster {
        primary: learned,
        secondary: cv,
        secondary_weight: 0.3,
    };
    vec![
        plan("snapshot", ForecastSet::empty(0.0)),
        plan(
            "constant velocity",
            cv.forecast(&tracks, 0.0, HORIZON_S).expect("cv"),
        ),
        plan(
            "blended (learned + cv)",
            blended.forecast(&tracks, 0.0, HORIZON_S).expect("blend"),
        ),
    ]
}

fn main() {
    for o in run() {
        println!("== world model: {}", o.name);
        print!("{}", o.summary);
        match &o.plan {
            Some(p) => println!(
                "   plan: {:.2} s, {:.2} m, peak height {:.2} m; executed against reality: \
                 closest approach {:.2} m -> {}\n",
                p.duration_s(),
                p.length_m(),
                p.waypoints
                    .iter()
                    .map(|w| w.position_m.z)
                    .fold(0.0, f64::max),
                o.min_true_clearance_m,
                if o.min_true_clearance_m > 0.0 {
                    "safe"
                } else {
                    "COLLISION"
                }
            ),
            None => println!("   no certified plan\n"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_the_blended_forecast_executes_safely() {
        let out = run();
        assert!(
            out[0].min_true_clearance_m <= 0.0,
            "snapshot should hit the forklift"
        );
        assert!(
            out[1].min_true_clearance_m <= 0.0,
            "cv should miss the worker"
        );
        assert!(out[2].min_true_clearance_m > 0.0, "{}", out[2].summary);
    }
}
