//! Forecast-aware arm motion and hybrid grasp selection.

use std::sync::Arc;

use glam::DVec3;
use robot_foresight::{
    AgentForecast, Certifier, FnModel, ForecastSet, HybridPipeline, ProposalContext, Proposer,
    SplitMix64, TimedState, TrajectoryMode,
};
use robot_manipulation::{
    default_grasp_pipeline, default_motion_pipeline, Aabb, DirectJointProposer, GraspProblem,
    LearnedGraspScorer, LearnedViaPointProposer, ManipScene, MotionCertifier, MotionProblem,
    PointCloud, RrtConnectProposer, SerialChain,
};

const START: [f64; 6] = [-0.8, -0.4, 0.8, 0.0, 0.0, 0.0];
const GOAL: [f64; 6] = [0.8, -0.4, 0.8, 0.0, 0.0, 0.0];

fn arm() -> SerialChain {
    SerialChain::six_dof_arm(DVec3::ZERO)
}

/// A hand resting where the arm's sweep passes until t = 2 s, then
/// withdrawing upwards.
fn hand(at: DVec3) -> ForecastSet {
    let still = |t_s| TimedState {
        t_s,
        position_m: at,
        velocity_mps: DVec3::ZERO,
    };
    ForecastSet {
        created_at_s: 0.0,
        horizon_s: 4.0,
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

fn problem(scene: ManipScene, forecast: ForecastSet) -> MotionProblem {
    MotionProblem {
        chain: Arc::new(arm()),
        scene: Arc::new(scene),
        forecast: Arc::new(forecast),
        start_q: START.to_vec(),
        goal_q: GOAL.to_vec(),
        start_t_s: 0.0,
        risk_threshold: 0.01,
        max_duration_s: 10.0,
    }
}

fn mid_tip() -> DVec3 {
    let mut q = START;
    q[0] = 0.0;
    arm().end_effector(&q).translation
}

#[test]
fn the_arm_waits_for_the_hand_instead_of_stopping_or_hitting_it() {
    let p = problem(ManipScene::default(), hand(mid_tip()));
    let d = default_motion_pipeline(1).decide(&p);
    let traj = &d
        .selected()
        .unwrap_or_else(|| panic!("{}", d.summary()))
        .candidate
        .solution;
    assert!(
        traj.end_t_s() > 2.0,
        "arrived at {} s, before the hand left",
        traj.end_t_s()
    );
    assert!(traj.wait_time_s() > 0.5, "waited {}", traj.wait_time_s());
    // Without the hand it is a full-speed 1.07 s sweep.
    let free = problem(ManipScene::default(), ForecastSet::empty(0.0));
    let fast = DirectJointProposer::default()
        .propose(
            &free,
            &mut ProposalContext {
                rng: SplitMix64::new(0),
            },
        )
        .unwrap();
    assert!(fast[0].duration_s() < 1.3);
    // ... which the certifier rejects in the real (forecast) world.
    assert!(!MotionCertifier::default()
        .certify(&p, &fast[0])
        .is_certified());
}

#[test]
fn rrt_connect_goes_around_a_post_that_blocks_the_direct_sweep() {
    let tip = mid_tip();
    let scene = ManipScene {
        spheres: vec![],
        boxes: vec![Aabb {
            min_m: tip - DVec3::new(0.08, 0.08, 0.6),
            max_m: tip + DVec3::new(0.08, 0.08, 0.06),
        }],
    };
    let p = problem(scene, ForecastSet::empty(0.0));
    let d = default_motion_pipeline(2).decide(&p);
    assert_eq!(
        d.proposer_failures[0].proposer,
        "direct_joint",
        "{}",
        d.summary()
    );
    assert_eq!(d.selected().unwrap().candidate.proposer, "rrt_connect");
}

#[test]
fn a_learned_motion_prior_competes_with_the_planners() {
    // Stand-in prior: lift the elbow halfway, a typical demonstration.
    let prior = FnModel::new("motion_prior_v0", 15, 6, |x| {
        let mut via: Vec<f32> = (0..6).map(|i| 0.5 * (x[i] + x[6 + i])).collect();
        via[1] -= 1.0;
        via[2] -= 0.6;
        via
    });
    let tip = mid_tip();
    let scene = ManipScene {
        spheres: vec![],
        boxes: vec![Aabb {
            min_m: tip - DVec3::new(0.08, 0.08, 0.6),
            max_m: tip + DVec3::new(0.08, 0.08, 0.06),
        }],
    };
    let p = problem(scene, ForecastSet::empty(0.0));
    let pipeline =
        default_motion_pipeline(3).with_proposer(LearnedViaPointProposer::new(prior, 6).unwrap());
    let d = pipeline.decide(&p);
    let learned = d
        .evaluated
        .iter()
        .find(|e| e.candidate.proposer == "learned_via_point")
        .unwrap_or_else(|| panic!("{}", d.summary()));
    assert!(learned.is_selectable(), "{}", d.summary());
    assert!(learned.candidate.provenance.involves_learning());
    assert_eq!(pipeline.decide(&p), d, "replays");
}

fn grasp_problem(center: DVec3) -> GraspProblem {
    GraspProblem {
        cloud: Arc::new(PointCloud::sample_box(
            center,
            DVec3::new(0.05, 0.12, 0.08),
            0.01,
        )),
        chain: Arc::new(arm()),
        seed_q: START.to_vec(),
        max_width_m: 0.085,
        friction_coefficient: 0.4,
        pregrasp_offset_m: 0.1,
    }
}

#[test]
fn antipodal_grasps_fit_the_gripper_and_are_reachable() {
    let p = grasp_problem(DVec3::new(0.5, 0.1, 0.1));
    let d = default_grasp_pipeline(4).decide(&p);
    let g = d.selected().expect("a grasp").candidate.solution;
    assert!(g.width_m <= 0.085 + 1e-9);
    // Only the 5 cm and 8 cm faces fit an 8.5 cm gripper, never the 12 cm one.
    assert!(g.closing_axis.y.abs() < 0.5, "{g:?}");
    assert!(d.evaluated.len() > 1);
}

#[test]
fn unreachable_objects_are_rejected_by_the_certifier() {
    let p = grasp_problem(DVec3::new(2.0, 0.0, 0.1));
    let d = default_grasp_pipeline(4).decide(&p);
    assert!(d.selected().is_none());
    assert!(d
        .evaluated
        .iter()
        .all(|e| e.rejection().unwrap().contains("unreachable")));
}

#[test]
fn a_learned_scorer_reorders_certified_grasps() {
    let p = grasp_problem(DVec3::new(0.5, 0.1, 0.1));
    // Stand-in scorer: prefers narrow grasps (feature 0 = width).
    let narrow = FnModel::new("grasp_net_v0", 6, 1, |x| vec![10.0 - 200.0 * x[0]]);
    let pipeline = HybridPipeline::new(4)
        .with_proposer(robot_manipulation::AntipodalGraspSampler::default())
        .with_critic(1.0, LearnedGraspScorer::new(narrow).unwrap())
        .with_certifier(robot_manipulation::ReachabilityCertifier::default());
    let g = pipeline.decide(&p).into_solution().unwrap();
    assert!((g.width_m - 0.05).abs() < 1e-6, "{g:?}");
}

#[test]
fn rrt_respects_its_seed() {
    let tip = mid_tip();
    let scene = ManipScene {
        spheres: vec![],
        boxes: vec![Aabb {
            min_m: tip - DVec3::new(0.08, 0.08, 0.6),
            max_m: tip + DVec3::new(0.08, 0.08, 0.06),
        }],
    };
    let p = problem(scene, ForecastSet::empty(0.0));
    let rrt = RrtConnectProposer::default();
    let a = rrt.geometric_path(&p, &mut SplitMix64::new(9)).unwrap();
    let b = rrt.geometric_path(&p, &mut SplitMix64::new(9)).unwrap();
    assert_eq!(a, b);
}
