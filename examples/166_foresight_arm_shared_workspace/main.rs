//! Example 166: an arm shares its workspace with a person's hand.
//!
//! The arm must pick a box and swing it across the table. A person's hand
//! is resting where the swing passes and is forecast to withdraw at
//! t = 2 s. A planner that sees only the present can stop, or replan once
//! the hand is gone. Here the hand's *future* is part of the problem:
//!
//! 1. A grasp pipeline proposes antipodal grasps (classical), prices them
//!    with friction-cone usage (classical) plus a learned success scorer
//!    (a stand-in, see `grasp_scorer`), and certifies reachability by IK.
//! 2. A motion pipeline proposes the direct sweep, RRT-Connect and a learned
//!    via-point prior; every geometric path is retimed around the forecast
//!    (path-velocity decomposition) and certified against it. The classical
//!    sweeps wait for the hand to leave; the learned prior lifts the elbow
//!    over it and arrives first, and is selected because it certifies.
//!
//! ```bash
//! cargo run -p foresight_arm_shared_workspace --example 166_foresight_arm_shared_workspace
//! ```

use std::sync::Arc;

use glam::DVec3;
use robot_foresight::{
    AgentForecast, FnModel, ForecastSet, HybridPipeline, TimedState, TrajectoryMode,
};
use robot_manipulation::{
    default_grasp_pipeline, default_motion_pipeline, GraspCandidate, GraspProblem, JointTrajectory,
    LearnedGraspScorer, LearnedViaPointProposer, ManipScene, MotionProblem, PointCloud,
    SerialChain,
};

const HOME: [f64; 6] = [-0.8, -0.4, 0.8, 0.0, 0.0, 0.0];

/// Stand-in for a trained grasp-success network over `grasp_features`:
/// favours grasps near the object's centroid (feature 5) with little
/// friction-cone usage (features 1, 2).
fn grasp_scorer() -> FnModel<impl Fn(&[f32]) -> Vec<f32>> {
    FnModel::new("grasp_success_standin_v0", 6, 1, |x| {
        vec![4.0 - 40.0 * x[5] - 3.0 * (x[1] + x[2])]
    })
}

/// Stand-in for a learned motion prior: a via point that lifts the elbow.
fn motion_prior() -> FnModel<impl Fn(&[f32]) -> Vec<f32>> {
    FnModel::new("motion_prior_standin_v0", 15, 6, |x| {
        let mut via: Vec<f32> = (0..6).map(|i| 0.5 * (x[i] + x[6 + i])).collect();
        via[1] -= 1.0;
        via[2] -= 0.6;
        via
    })
}

fn hand_forecast(at: DVec3) -> ForecastSet {
    let still = |t_s| TimedState {
        t_s,
        position_m: at,
        velocity_mps: DVec3::ZERO,
    };
    ForecastSet {
        created_at_s: 0.0,
        horizon_s: 5.0,
        agents: vec![AgentForecast {
            agent_id: 1,
            radius_m: 0.06,
            sigma0_m: 0.01,
            sigma_growth_mps: 0.0,
            modes: vec![TrajectoryMode {
                probability: 1.0,
                states: vec![
                    still(0.0),
                    still(2.0),
                    TimedState {
                        t_s: 2.5,
                        position_m: at + DVec3::Z * 0.8,
                        velocity_mps: DVec3::Z * 1.6,
                    },
                ],
            }],
        }],
    }
}

/// What the example produces.
#[derive(Debug)]
struct Outcome {
    grasp: Option<GraspCandidate>,
    motion: Option<JointTrajectory>,
    /// The classical direct sweep, retimed around the hand.
    direct: Option<JointTrajectory>,
    selected_provenance: Option<String>,
    grasp_summary: String,
    motion_summary: String,
}

fn run() -> Outcome {
    let arm = Arc::new(SerialChain::six_dof_arm(DVec3::ZERO));

    let grasp_problem = GraspProblem {
        cloud: Arc::new(PointCloud::sample_box(
            DVec3::new(0.45, -0.35, 0.12),
            DVec3::new(0.05, 0.12, 0.08),
            0.01,
        )),
        chain: arm.clone(),
        seed_q: HOME.to_vec(),
        max_width_m: 0.085,
        friction_coefficient: 0.4,
        pregrasp_offset_m: 0.1,
    };
    let grasp_pipeline: HybridPipeline<_, _> = default_grasp_pipeline(166).with_critic(
        1.0,
        LearnedGraspScorer::new(grasp_scorer()).expect("scorer shape"),
    );
    let gd = grasp_pipeline.decide(&grasp_problem);
    let grasp_summary = gd.summary();
    let grasp = gd.into_solution();

    let mut mid = HOME;
    mid[0] = 0.0;
    let hand_at = arm.end_effector(&mid).translation;
    let mut goal = HOME;
    goal[0] = 0.8;
    let motion_problem = MotionProblem {
        chain: arm.clone(),
        scene: Arc::new(ManipScene::default()),
        forecast: Arc::new(hand_forecast(hand_at)),
        start_q: HOME.to_vec(),
        goal_q: goal.to_vec(),
        start_t_s: 0.0,
        risk_threshold: 0.01,
        max_duration_s: 10.0,
    };
    let motion_pipeline = default_motion_pipeline(166)
        .with_proposer(LearnedViaPointProposer::new(motion_prior(), 6).expect("prior shape"));
    let md = motion_pipeline.decide(&motion_problem);
    let motion_summary = md.summary();
    let direct = md
        .evaluated
        .iter()
        .find(|e| e.candidate.proposer == "direct_joint")
        .map(|e| e.candidate.solution.clone());
    let selected_provenance = md.selected().map(|e| e.candidate.provenance.label());
    Outcome {
        grasp,
        motion: md.into_solution(),
        direct,
        selected_provenance,
        grasp_summary,
        motion_summary,
    }
}

fn main() {
    let o = run();
    println!("== grasp candidates (antipodal sampler; friction + learned score; IK-certified)");
    print!("{}", o.grasp_summary);
    if let Some(g) = &o.grasp {
        println!(
            "   grasp: centre {:.3?}, width {:.3} m, closing {:.2?}\n",
            g.center_m, g.width_m, g.closing_axis
        );
    }
    println!("== swing across the table while the hand is forecast to leave at t = 2.0 s");
    print!("{}", o.motion_summary);
    if let Some(d) = &o.direct {
        println!(
            "   direct sweep: waits {:.2} s for the hand, arrives at {:.2} s \
             (unobstructed it takes 1.07 s)",
            d.wait_time_s(),
            d.end_t_s()
        );
    }
    if let (Some(m), Some(p)) = (&o.motion, &o.selected_provenance) {
        println!(
            "   selected ({p}): arrives at {:.2} s, waits {:.2} s",
            m.end_t_s(),
            m.wait_time_s()
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classical_sweep_waits_and_the_learned_route_over_the_hand_wins() {
        let o = run();
        assert!(o.grasp.is_some(), "{}", o.grasp_summary);
        let d = o.direct.expect("direct sweep");
        assert!(
            d.end_t_s() > 2.0 && d.wait_time_s() > 0.5,
            "{}",
            o.motion_summary
        );
        let m = o.motion.expect("certified motion");
        assert!(m.end_t_s() < d.end_t_s(), "{}", o.motion_summary);
        assert!(o.selected_provenance.unwrap().starts_with("learned:"));
    }
}
