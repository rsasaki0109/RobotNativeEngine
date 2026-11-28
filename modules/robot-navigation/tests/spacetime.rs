//! Space-time navigation behaviour against forecast worlds.

use std::sync::Arc;

use glam::DVec3;
use robot_foresight::{
    AgentForecast, Certifier, FnModel, ForecastSet, Provenance, SelectionPolicy, TimedState,
    TrajectoryMode, WorldForecast,
};
use robot_navigation::{
    check_trajectory, default_pipeline, ForecastCertifier, KinematicLimits,
    LearnedVelocityProposer, NavProblem, SpaceTimeAstar, VoxelGrid,
};

fn room() -> VoxelGrid {
    // 12 m x 6 m x 4 m at 0.25 m.
    VoxelGrid::new(DVec3::ZERO, 0.25, [48, 24, 16]).unwrap()
}

fn problem(map: VoxelGrid, forecast: ForecastSet) -> NavProblem {
    NavProblem {
        map: Arc::new(map),
        forecast: Arc::new(forecast),
        start_m: DVec3::new(1.0, 3.0, 1.0),
        start_t_s: 0.0,
        goal_m: DVec3::new(11.0, 3.0, 1.0),
        goal_tolerance_m: 0.2,
        robot_radius_m: 0.2,
        limits: KinematicLimits {
            max_speed_mps: 2.0,
            max_vertical_speed_mps: 1.0,
        },
        risk_threshold: 0.05,
    }
}

/// A person walking across the corridor at x = 6 m, reaching the robot's
/// line when a full-speed robot would.
fn crossing() -> ForecastSet {
    ForecastSet {
        created_at_s: 0.0,
        horizon_s: 10.0,
        agents: vec![AgentForecast {
            agent_id: 7,
            radius_m: 0.35,
            sigma0_m: 0.05,
            sigma_growth_mps: 0.05,
            modes: vec![TrajectoryMode {
                probability: 1.0,
                states: vec![TimedState {
                    t_s: 0.0,
                    position_m: DVec3::new(6.0, -2.0, 1.0),
                    velocity_mps: DVec3::new(0.0, 2.0, 0.0),
                }],
            }],
        }],
    }
}

#[test]
fn astar_flies_over_a_wall_that_blocks_the_floor() {
    let mut map = room();
    // Wall across the room up to 2.5 m; free above.
    map.fill_box(DVec3::new(5.5, 0.0, 0.0), DVec3::new(6.0, 6.0, 2.5));
    let p = problem(map, ForecastSet::empty(0.0));
    let traj = SpaceTimeAstar::default().plan(&p).unwrap();
    let peak = traj
        .waypoints
        .iter()
        .map(|w| w.position_m.z)
        .fold(0.0, f64::max);
    assert!(peak > 2.5, "peak {peak}");
    let check = check_trajectory(&p, &traj, 0.05, 0.0);
    assert!(check.violation.is_none(), "{check:?}");
}

#[test]
fn full_speed_straight_line_meets_the_person_and_the_pipeline_avoids_it() {
    let p = problem(room(), crossing());
    let decision = default_pipeline(3).decide(&p);
    // The full-speed straight line is rejected for predicted risk.
    let first = &decision.evaluated[0];
    assert_eq!(first.candidate.proposer, "straight_line");
    assert!(
        first.rejection().unwrap().contains("risk"),
        "{}",
        decision.summary()
    );
    let chosen = decision.selected().expect("a certified plan");
    // Ground truth: the person follows the forecast exactly.
    let traj = &chosen.candidate.solution;
    for w in traj.sample(0.02) {
        let person = DVec3::new(6.0, -2.0 + 2.0 * w.t_s, 1.0);
        assert!(w.position_m.distance(person) > 0.55, "{w:?}");
    }
    assert!(
        p.forecast
            .collision_probability(traj.end_position_m().unwrap(), 0.2, traj.end_t_s())
            < 0.05
    );
}

#[test]
fn a_static_world_plan_collides_with_the_future() {
    // What a snapshot planner does: plan with nothing moving, then execute.
    let blind = problem(room(), ForecastSet::empty(0.0));
    let traj = SpaceTimeAstar::default().plan(&blind).unwrap();
    let aware = problem(room(), crossing());
    let check = check_trajectory(&aware, &traj, 0.05, 0.0);
    assert!(check.violation.is_some(), "snapshot plan unexpectedly safe");
}

fn goal_seeker() -> FnModel<impl Fn(&[f32]) -> Vec<f32>> {
    // Stand-in for a trained policy: head for the goal at 1.8 m/s.
    FnModel::new("goal_seeker_v0", 12, 3, |x| {
        let g = glam::Vec3::new(x[0], x[1], x[2]);
        let v = g.normalize_or_zero() * 1.8;
        vec![v.x, v.y, v.z]
    })
}

#[test]
fn learned_policy_is_preferred_when_it_certifies() {
    let p = problem(room(), ForecastSet::empty(0.0));
    let pipeline = robot_foresight::HybridPipeline::new(5)
        .with_policy(SelectionPolicy::FirstCertified)
        .with_proposer(LearnedVelocityProposer::new(goal_seeker()).unwrap())
        .with_proposer(SpaceTimeAstar::default())
        .with_certifier(ForecastCertifier::default());
    let d = pipeline.decide(&p);
    let chosen = d.selected().unwrap();
    assert_eq!(
        chosen.candidate.provenance,
        Provenance::learned("goal_seeker_v0")
    );
}

#[test]
fn classical_planner_takes_over_when_the_policy_hits_a_wall() {
    let mut map = room();
    map.fill_box(DVec3::new(5.5, 0.0, 0.0), DVec3::new(6.0, 6.0, 2.5));
    let p = problem(map, ForecastSet::empty(0.0));
    let pipeline = robot_foresight::HybridPipeline::new(5)
        .with_policy(SelectionPolicy::FirstCertified)
        .with_proposer(LearnedVelocityProposer::new(goal_seeker()).unwrap())
        .with_proposer(SpaceTimeAstar::default())
        .with_certifier(ForecastCertifier::default());
    let d = pipeline.decide(&p);
    assert!(d
        .evaluated
        .iter()
        .filter(|e| e.candidate.proposer == "learned_velocity_policy")
        .all(|e| !e.is_selectable()));
    assert_eq!(d.selected().unwrap().candidate.proposer, "spacetime_astar");
    assert!(
        d.evaluated[0].rejection().unwrap().contains("static"),
        "{}",
        d.summary()
    );
}

#[test]
fn decisions_replay_bit_for_bit() {
    let p = problem(room(), crossing());
    let pipeline =
        default_pipeline(11).with_proposer(LearnedVelocityProposer::new(goal_seeker()).unwrap());
    let a = pipeline.decide(&p);
    let b = pipeline.decide(&p);
    assert_eq!(a, b);
}

#[test]
fn certifier_rejects_speeding() {
    let p = problem(room(), ForecastSet::empty(0.0));
    let traj = robot_navigation::Trajectory3::new(vec![
        robot_navigation::Waypoint3 {
            t_s: 0.0,
            position_m: p.start_m,
        },
        robot_navigation::Waypoint3 {
            t_s: 1.0,
            position_m: p.goal_m,
        },
    ]);
    let cert = ForecastCertifier::default().certify(&p, &traj);
    assert!(!cert.is_certified());
}
