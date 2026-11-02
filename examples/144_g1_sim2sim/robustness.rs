//! Push and floor-friction sweeps with the same protocol as
//! `mujoco_robustness.py`, to compare where each engine's walk fails.
//!
//! A push is a horizontal force on the pelvis origin, held for 0.1 s after
//! 4 s of walking at 0.5 m/s and started at one of eight evenly spaced
//! points of the 0.8 s gait cycle. The run counts as a fall if the pelvis
//! drops below [`FALL_HEIGHT_M`] within the following 4 s. The friction
//! sweep walks 10 s on floors whose friction, and the feet's, is set to each
//! coefficient.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

use rne_math::Vec3;
use serde_json::{json, Value};

use crate::policy::{self, Result, Robot};
use crate::walk::{self, summarize, PlantConfig, FALL_HEIGHT_M};

/// Walking command during every trial.
pub(crate) const COMMAND: [f64; 3] = [0.5, 0.0, 0.0];
/// Policy steps walked before the earliest push (4 s).
pub(crate) const WALK_STEPS: usize = 200;
/// Gait points per 0.8 s cycle at which pushes start.
pub(crate) const PHASES: usize = 8;
/// Policy steps between consecutive push starts (0.1 s).
pub(crate) const PHASE_STEPS: usize = 5;
/// Policy steps the push lasts (0.1 s).
pub(crate) const PUSH_STEPS: usize = 5;
/// Policy steps observed after the push ends (4 s).
pub(crate) const RECOVERY_STEPS: usize = 200;
/// Policy steps walked in each friction trial (10 s).
pub(crate) const FRICTION_STEPS: usize = 500;

/// Push directions in the policy's Z-up world; the robot walks along +x.
pub(crate) const DIRECTIONS: [(&str, [f64; 2]); 4] = [
    ("forward", [1.0, 0.0]),
    ("backward", [-1.0, 0.0]),
    ("left", [0.0, 1.0]),
    ("right", [0.0, -1.0]),
];

/// Push forces in newtons, scaled by robot mass so both robots span their
/// failure range with the same number of levels.
pub(crate) fn push_forces_n(robot: Robot) -> Vec<f64> {
    let step = match robot {
        Robot::G1 => 50.0,
        Robot::H1 => 80.0,
    };
    (1..=20).map(|level| step * f64::from(level)).collect()
}

/// Floor and foot friction coefficients of the friction sweep.
pub(crate) const FRICTIONS: [f64; 10] = [1.0, 0.8, 0.6, 0.5, 0.4, 0.3, 0.25, 0.2, 0.15, 0.1];

#[derive(Clone, Copy, Debug)]
struct PushCase {
    direction: usize,
    force_n: f64,
    phase: usize,
}

/// Runs one push trial and returns when the pelvis first fell, if it did.
fn push_trial(robot: Robot, plant: PlantConfig, case: PushCase) -> Result<Option<f64>> {
    let mut walker = walk::walker(robot, policy::load(robot)?, plant)?;
    let [x, y] = DIRECTIONS[case.direction].1;
    // Z-up (x, y, z) is RNE's Y-up (x, z, -y).
    let force = Vec3::new(x * case.force_n, 0.0, -y * case.force_n);
    let start = WALK_STEPS + case.phase * PHASE_STEPS;
    for step in 0..start + PUSH_STEPS + RECOVERY_STEPS {
        walker.set_base_push_n((start..start + PUSH_STEPS).contains(&step).then_some(force));
        let record = walker.step(COMMAND)?;
        if record.base_position_m[2] < FALL_HEIGHT_M {
            return Ok(Some(record.time_s));
        }
    }
    Ok(None)
}

fn friction_trial(robot: Robot, plant: PlantConfig, friction: f64) -> Result<Value> {
    let plant = PlantConfig {
        floor_friction: friction,
        ..plant
    };
    let mut walker = walk::walker(robot, policy::load(robot)?, plant)?;
    let mut records = Vec::with_capacity(FRICTION_STEPS);
    for _ in 0..FRICTION_STEPS {
        records.push(walker.step(COMMAND)?);
    }
    Ok(summarize(&records, COMMAND))
}

/// Runs `jobs` on `threads` workers, keeping results in job order.
fn run_parallel<T: Send>(
    jobs: usize,
    threads: usize,
    job: impl Fn(usize) -> Result<T> + Sync,
) -> Result<Vec<T>> {
    let next = AtomicUsize::new(0);
    let results: Mutex<Vec<Option<std::result::Result<T, String>>>> =
        Mutex::new((0..jobs).map(|_| None).collect());
    std::thread::scope(|scope| {
        for _ in 0..threads.max(1) {
            scope.spawn(|| loop {
                let index = next.fetch_add(1, Ordering::Relaxed);
                if index >= jobs {
                    break;
                }
                let result = job(index).map_err(|error| error.to_string());
                results.lock().expect("results lock")[index] = Some(result);
            });
        }
    });
    results
        .into_inner()
        .expect("results lock")
        .into_iter()
        .map(|result| match result.expect("every job ran") {
            Ok(value) => Ok(value),
            Err(message) => Err(message.into()),
        })
        .collect()
}

/// The full sweep as JSON, matching `mujoco_robustness.py`'s report.
pub(crate) fn sweep(robot: Robot, plant: PlantConfig, threads: usize) -> Result<Value> {
    let forces = push_forces_n(robot);
    let cases: Vec<PushCase> = (0..DIRECTIONS.len())
        .flat_map(|direction| {
            forces.iter().flat_map(move |&force_n| {
                (0..PHASES).map(move |phase| PushCase {
                    direction,
                    force_n,
                    phase,
                })
            })
        })
        .collect();
    let falls = run_parallel(cases.len(), threads, |index| {
        push_trial(robot, plant, cases[index])
    })?;
    let push: Vec<Value> = cases
        .iter()
        .zip(&falls)
        .map(|(case, fall)| {
            json!({
                "direction": DIRECTIONS[case.direction].0,
                "force_n": case.force_n,
                "phase": case.phase,
                "fell": fall.is_some(),
                "fall_time_s": fall,
            })
        })
        .collect();
    let friction = run_parallel(FRICTIONS.len(), threads, |index| {
        friction_trial(robot, plant, FRICTIONS[index])
            .map(|summary| json!({"friction": FRICTIONS[index], "summary": summary}))
    })?;
    Ok(json!({
        "schema_version": 1,
        "engine": "RNE Rapier (UrdfSceneSim)",
        "robot": robot.name(),
        "solver_iterations": plant.solver_iterations,
        "self_collision": plant.self_collision,
        "protocol": protocol(robot),
        "push": push,
        "friction": friction,
    }))
}

/// The shared trial protocol, recorded in both engines' reports.
pub(crate) fn protocol(robot: Robot) -> Value {
    json!({
        "command": COMMAND,
        "walk_before_push_s": WALK_STEPS as f64 * 0.02,
        "phases_per_cycle": PHASES,
        "phase_spacing_s": PHASE_STEPS as f64 * 0.02,
        "push_duration_s": PUSH_STEPS as f64 * 0.02,
        "recovery_s": RECOVERY_STEPS as f64 * 0.02,
        "push_point": "pelvis origin",
        "directions": DIRECTIONS.iter().map(|(name, d)| json!({"name": name, "z_up_xy": d})).collect::<Vec<_>>(),
        "forces_n": push_forces_n(robot),
        "fall_height_m": FALL_HEIGHT_M,
        "friction_walk_s": FRICTION_STEPS as f64 * 0.02,
        "frictions": FRICTIONS,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_strong_side_push_topples_the_g1_and_none_does_not() {
        let plant = PlantConfig::default();
        let calm = PushCase {
            direction: 2,
            force_n: 0.0,
            phase: 0,
        };
        assert_eq!(push_trial(Robot::G1, plant, calm).unwrap(), None);
        let hard = PushCase {
            force_n: 1500.0,
            ..calm
        };
        assert!(push_trial(Robot::G1, plant, hard).unwrap().is_some());
    }

    #[test]
    fn a_right_push_topples_the_g1_only_when_its_legs_collide() {
        // At 200 N the left leg swings into the right one. MuJoCo's MJCF lets
        // them collide and the G1 falls; so does RNE with self-collision.
        let case = PushCase {
            direction: 3,
            force_n: 200.0,
            phase: 3,
        };
        let plant = PlantConfig::default();
        assert_eq!(push_trial(Robot::G1, plant, case).unwrap(), None);
        let colliding = PlantConfig {
            self_collision: true,
            ..plant
        };
        assert!(push_trial(Robot::G1, colliding, case).unwrap().is_some());
    }

    #[test]
    fn parallel_results_keep_job_order() {
        let values = run_parallel(17, 4, |index| Ok(index * 3)).unwrap();
        assert_eq!(values, (0..17).map(|i| i * 3).collect::<Vec<_>>());
    }
}
