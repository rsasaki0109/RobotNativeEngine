//! A linear residual on the pretrained walking policy, trained in RNE by
//! evolution strategies to recover from pushes.
//!
//! The residual reads five entries of the policy's own observation: the
//! scaled pelvis angular velocity and the x and y of gravity projected into
//! the pelvis frame. Both are zero for an upright pelvis at rest, so a small
//! residual leaves walking nearly as it was. It adds `W · features`, clamped
//! to ±[`LIMIT_RAD`], to every joint target for the next policy period,
//! computed from the same observation as the policy's action
//! ([`rne_ai::UnitreeRlGymWalker::offset_targets_rad`]). `mujoco_robustness.py
//! --residual` applies the same law in MuJoCo.
//!
//! Training (OpenAI-style evolution strategies with antithetic pairs, centred
//! ranks and Adam) scores each candidate on fixed push scenarios near the
//! forces where the plain policy falls. A survived push scores 1 plus up to
//! 0.5 for how upright the pelvis stayed (none at a tilt of [`TILT_REF_RAD`]
//! or more); a fall scores half the share of the observation window it stood
//! through. Candidates pay the mean squared target offset times a penalty.

use std::time::Instant;

use rne_math::Vec3;
use serde_json::{json, Value};

use crate::policy::{self, Result, Robot};
use crate::robustness::{run_parallel, COMMAND, DIRECTIONS, PHASE_STEPS, PUSH_STEPS};
use crate::walk::{self, PlantConfig, StepRecord, FALL_HEIGHT_M};

/// Observation entries the residual reads: angular velocity (3) and the
/// horizontal components of projected gravity (2).
pub(crate) const FEATURES: usize = 5;
/// Largest target offset the residual may add, in radians.
pub(crate) const LIMIT_RAD: f64 = 0.4;

/// Linear map from [`FEATURES`] observation entries to joint target offsets.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Residual {
    /// Row-major `joints × FEATURES` weights, radians per feature unit.
    weights: Vec<f64>,
}

impl Residual {
    pub(crate) fn zero(joints: usize) -> Self {
        Self {
            weights: vec![0.0; joints * FEATURES],
        }
    }

    #[cfg(test)]
    fn joints(&self) -> usize {
        self.weights.len() / FEATURES
    }

    /// Target offsets for the observation the policy just received.
    pub(crate) fn offsets(&self, observation: &[f32]) -> Vec<f64> {
        let features: Vec<f64> = observation[..FEATURES]
            .iter()
            .map(|&v| f64::from(v))
            .collect();
        self.weights
            .chunks_exact(FEATURES)
            .map(|row| {
                let offset: f64 = row.iter().zip(&features).map(|(w, f)| w * f).sum();
                offset.clamp(-LIMIT_RAD, LIMIT_RAD)
            })
            .collect()
    }

    pub(crate) fn to_json(&self) -> Value {
        json!({
            "features": ["angular_velocity_x_scaled", "angular_velocity_y_scaled",
                         "angular_velocity_z_scaled", "projected_gravity_x", "projected_gravity_y"],
            "limit_rad": LIMIT_RAD,
            "weights_rad": self.weights.chunks_exact(FEATURES).collect::<Vec<_>>(),
        })
    }

    pub(crate) fn from_json(value: &Value) -> Result<Self> {
        let rows = value["weights_rad"]
            .as_array()
            .ok_or("residual has no weights_rad")?;
        let mut weights = Vec::with_capacity(rows.len() * FEATURES);
        for row in rows {
            let row = row.as_array().ok_or("weights_rad rows must be arrays")?;
            if row.len() != FEATURES {
                return Err(format!("weights_rad rows need {FEATURES} entries").into());
            }
            for weight in row {
                weights.push(weight.as_f64().ok_or("weights must be numbers")?);
            }
        }
        if value["limit_rad"].as_f64() != Some(LIMIT_RAD) {
            return Err("residual was trained with a different offset limit".into());
        }
        Ok(Self { weights })
    }
}

/// Steps the walker, then offsets its next targets by the residual, if any.
pub(crate) fn step(
    walker: &mut rne_ai::UnitreeRlGymWalker,
    residual: Option<&Residual>,
    command: [f64; 3],
) -> Result<StepRecord> {
    let record = walker.step(command)?;
    if let Some(residual) = residual {
        walker.offset_targets_rad(&residual.offsets(&record.observation))?;
    }
    Ok(record)
}

/// Policy steps walked before a training push (1 s).
const TRAIN_WALK_STEPS: usize = 50;
/// Policy steps observed after a training push (1.5 s).
const TRAIN_RECOVERY_STEPS: usize = 75;
/// Pelvis tilt after the push at which a survived push earns no bonus.
pub(crate) const TILT_REF_RAD: f64 = 0.6;
/// Gait phases (of [`crate::robustness::PHASES`]) at which training pushes start.
const TRAIN_PHASES: [usize; 4] = [0, 2, 4, 6];
/// Multiples of each direction's base force that training pushes use.
const TRAIN_FORCE_SCALES: [f64; 2] = [1.0, 1.25];

/// Per-direction forces near which the plain policy falls half the time with
/// self-collision (forward, backward, left, right), from the robustness sweep.
fn base_forces_n(robot: Robot) -> [f64; 4] {
    match robot {
        Robot::G1 => [450.0, 600.0, 400.0, 300.0],
        Robot::H1 => [960.0, 880.0, 560.0, 720.0],
    }
}

#[derive(Clone, Copy, Debug)]
struct Scenario {
    direction: usize,
    force_n: f64,
    phase: usize,
}

fn scenarios(robot: Robot) -> Vec<Scenario> {
    let forces = base_forces_n(robot);
    let mut list = Vec::new();
    for (direction, base) in forces.iter().enumerate() {
        for scale in TRAIN_FORCE_SCALES {
            for phase in TRAIN_PHASES {
                list.push(Scenario {
                    direction,
                    force_n: base * scale,
                    phase,
                });
            }
        }
    }
    list
}

/// Scenario score and simulated policy steps.
fn run_scenario(
    robot: Robot,
    plant: PlantConfig,
    residual: &Residual,
    scenario: Scenario,
) -> Result<(f64, f64, usize)> {
    let mut walker = walk::walker(robot, policy::load(robot)?, plant)?;
    let [x, y] = DIRECTIONS[scenario.direction].1;
    let force = Vec3::new(x * scenario.force_n, 0.0, -y * scenario.force_n);
    let start = TRAIN_WALK_STEPS + scenario.phase * PHASE_STEPS;
    let end = start + PUSH_STEPS + TRAIN_RECOVERY_STEPS;
    let mut offset_squares = 0.0;
    let mut max_tilt_rad: f64 = 0.0;
    for step in 0..end {
        walker.set_base_push_n((start..start + PUSH_STEPS).contains(&step).then_some(force));
        let record = walker.step(COMMAND)?;
        let offsets = residual.offsets(&record.observation);
        offset_squares += offsets.iter().map(|o| o * o).sum::<f64>() / offsets.len() as f64;
        walker.offset_targets_rad(&offsets)?;
        if step >= start {
            // Projected gravity's z is -cos(tilt).
            let cos_tilt = -f64::from(record.observation[5]);
            max_tilt_rad = max_tilt_rad.max(cos_tilt.clamp(-1.0, 1.0).acos());
        }
        if record.base_position_m[2] < FALL_HEIGHT_M {
            let stood = (step + 1).saturating_sub(start) as f64 / (end - start) as f64;
            return Ok((0.5 * stood, offset_squares / (step + 1) as f64, step + 1));
        }
    }
    let upright = 0.5 * (1.0 - (max_tilt_rad / TILT_REF_RAD).min(1.0));
    Ok((1.0 + upright, offset_squares / end as f64, end))
}

/// Evolution-strategies settings.
#[derive(Clone, Copy, Debug)]
pub(crate) struct TrainConfig {
    pub(crate) generations: usize,
    /// Antithetic pairs per generation.
    pub(crate) pairs: usize,
    pub(crate) sigma: f64,
    pub(crate) learning_rate: f64,
    /// Weight on the mean squared target offset, in score per rad².
    pub(crate) offset_penalty: f64,
    pub(crate) seed: u64,
}

impl Default for TrainConfig {
    fn default() -> Self {
        Self {
            generations: 30,
            pairs: 16,
            sigma: 0.05,
            learning_rate: 0.02,
            offset_penalty: 1.0,
            seed: 2026,
        }
    }
}

/// SplitMix64 normal deviates (Box–Muller), so training replays exactly.
struct Normal(u64);

impl Normal {
    fn uniform(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        ((z >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    fn sample(&mut self) -> f64 {
        let (u, v) = (self.uniform(), self.uniform());
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

/// Mean score over the scenarios, its offset penalty, and simulated steps.
fn evaluate(
    robot: Robot,
    plant: PlantConfig,
    candidates: &[Residual],
    scenarios: &[Scenario],
    penalty: f64,
    threads: usize,
) -> Result<(Vec<f64>, Vec<f64>, usize)> {
    let jobs = candidates.len() * scenarios.len();
    let results = run_parallel(jobs, threads, |index| {
        let (candidate, scenario) = (index / scenarios.len(), index % scenarios.len());
        run_scenario(robot, plant, &candidates[candidate], scenarios[scenario])
    })?;
    let mut fitness = vec![0.0; candidates.len()];
    let mut survival = vec![0.0; candidates.len()];
    let mut steps = 0;
    for (index, (score, offset_square, taken)) in results.into_iter().enumerate() {
        let candidate = index / scenarios.len();
        let n = scenarios.len() as f64;
        fitness[candidate] += (score - penalty * offset_square) / n;
        survival[candidate] += f64::from(u8::from(score >= 1.0)) / n;
        steps += taken;
    }
    Ok((fitness, survival, steps))
}

/// Centred ranks in [-0.5, 0.5].
fn centred_ranks(values: &[f64]) -> Vec<f64> {
    let mut order: Vec<usize> = (0..values.len()).collect();
    order.sort_by(|&a, &b| values[a].total_cmp(&values[b]).then(a.cmp(&b)));
    let mut ranks = vec![0.0; values.len()];
    let denominator = (values.len() - 1).max(1) as f64;
    for (rank, index) in order.into_iter().enumerate() {
        ranks[index] = rank as f64 / denominator - 0.5;
    }
    ranks
}

/// Trains the residual; returns it and a log of the run.
pub(crate) fn train(
    robot: Robot,
    plant: PlantConfig,
    threads: usize,
    config: TrainConfig,
) -> Result<(Residual, Value)> {
    let joints = walk::walker(robot, policy::load(robot)?, plant)?
        .effort_limits_nm()
        .len();
    let scenarios = scenarios(robot);
    let dimension = joints * FEATURES;
    let mut theta = Residual::zero(joints).weights;
    let (mut m, mut v) = (vec![0.0; dimension], vec![0.0; dimension]);
    let (beta1, beta2) = (0.9, 0.999);
    let mut normal = Normal(config.seed);
    let begin = Instant::now();
    let mut simulated_steps = 0;
    let mut log = Vec::new();
    for generation in 0..=config.generations {
        let noise: Vec<Vec<f64>> = (0..config.pairs)
            .map(|_| (0..dimension).map(|_| normal.sample()).collect())
            .collect();
        let mut candidates = vec![Residual {
            weights: theta.clone(),
        }];
        if generation < config.generations {
            for epsilon in &noise {
                for sign in [1.0, -1.0] {
                    candidates.push(Residual {
                        weights: theta
                            .iter()
                            .zip(epsilon)
                            .map(|(t, e)| t + sign * config.sigma * e)
                            .collect(),
                    });
                }
            }
        }
        let (fitness, survival, steps) = evaluate(
            robot,
            plant,
            &candidates,
            &scenarios,
            config.offset_penalty,
            threads,
        )?;
        simulated_steps += steps;
        let entry = json!({
            "generation": generation,
            "fitness": fitness[0],
            "survived_share": survival[0],
            "population_mean_fitness": fitness.iter().sum::<f64>() / fitness.len() as f64,
            "wall_s": begin.elapsed().as_secs_f64(),
            "simulated_s": simulated_steps as f64 * 0.02,
        });
        println!("{entry}");
        log.push(entry);
        if generation == config.generations {
            break;
        }
        let ranks = centred_ranks(&fitness[1..]);
        let mut gradient = vec![0.0; dimension];
        for (pair, epsilon) in noise.iter().enumerate() {
            let weight = ranks[2 * pair] - ranks[2 * pair + 1];
            for (g, e) in gradient.iter_mut().zip(epsilon) {
                *g += weight * e / (config.pairs as f64 * config.sigma);
            }
        }
        let t = (generation + 1) as i32;
        for i in 0..dimension {
            m[i] = beta1 * m[i] + (1.0 - beta1) * gradient[i];
            v[i] = beta2 * v[i] + (1.0 - beta2) * gradient[i] * gradient[i];
            let (m_hat, v_hat) = (m[i] / (1.0 - beta1.powi(t)), v[i] / (1.0 - beta2.powi(t)));
            theta[i] += config.learning_rate * m_hat / (v_hat.sqrt() + 1e-8);
        }
    }
    let residual = Residual { weights: theta };
    let report = json!({
        "schema_version": 1,
        "robot": robot.name(),
        "self_collision": plant.self_collision,
        "solver_iterations": plant.solver_iterations,
        "residual": residual.to_json(),
        "training": {
            "method": "evolution strategies, antithetic pairs, centred ranks, Adam",
            "generations": config.generations,
            "pairs": config.pairs,
            "sigma": config.sigma,
            "learning_rate": config.learning_rate,
            "offset_penalty_per_rad2": config.offset_penalty,
            "seed": config.seed,
            "threads": threads,
            "scenarios": scenarios.iter().map(|s| json!({
                "direction": DIRECTIONS[s.direction].0, "force_n": s.force_n, "phase": s.phase,
            })).collect::<Vec<_>>(),
            "tilt_reference_rad": TILT_REF_RAD,
            "walk_before_push_s": TRAIN_WALK_STEPS as f64 * 0.02,
            "recovery_s": TRAIN_RECOVERY_STEPS as f64 * 0.02,
            "log": log,
        },
    });
    Ok((residual, report))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn residual_offsets_are_linear_clamped_and_round_trip() {
        let mut residual = Residual::zero(2);
        residual.weights[0] = 1.0; // joint 0 from angular velocity x
        residual.weights[FEATURES + 3] = -2.0; // joint 1 from gravity x
        let observation = [0.1_f32, 0.0, 0.0, 0.5, 0.0, 9.0];
        let offsets = residual.offsets(&observation);
        assert!((offsets[0] - f64::from(0.1_f32)).abs() < 1e-12);
        assert_eq!(offsets[1], -LIMIT_RAD);
        let again = Residual::from_json(&residual.to_json()).unwrap();
        assert_eq!(again, residual);
        assert_eq!(again.joints(), 2);
    }

    #[test]
    fn centred_ranks_span_the_population() {
        assert_eq!(centred_ranks(&[3.0, 1.0, 2.0]), vec![0.5, -0.5, 0.0]);
    }

    #[test]
    fn a_zero_residual_walks_like_the_plain_policy() {
        let plant = PlantConfig::default();
        let mut plain = walk::walker(Robot::G1, policy::load(Robot::G1).unwrap(), plant).unwrap();
        let mut zero = walk::walker(Robot::G1, policy::load(Robot::G1).unwrap(), plant).unwrap();
        let residual = Residual::zero(12);
        for _ in 0..25 {
            assert_eq!(
                plain.step(COMMAND).unwrap(),
                step(&mut zero, Some(&residual), COMMAND).unwrap()
            );
        }
    }
}
