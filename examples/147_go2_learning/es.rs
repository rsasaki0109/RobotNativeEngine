//! Evolution strategies: antithetic pairs, centred ranks, Adam, SplitMix64
//! normal deviates. `mujoco_go2_learning.py` repeats every step in MuJoCo.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;
use std::time::Instant;

use serde_json::{json, Value};

use crate::go2::{self, Outcome, Plant, CONTROL_DT_S};
use crate::policy::Mlp;
use crate::Result;

/// Training settings.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Config {
    pub(crate) generations: usize,
    /// Antithetic pairs per generation.
    pub(crate) pairs: usize,
    pub(crate) sigma: f64,
    pub(crate) learning_rate: f64,
    pub(crate) hidden: usize,
    pub(crate) seed: u64,
    /// Store the mean every this many generations (and at the end); 0 never.
    pub(crate) checkpoint_every: usize,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            generations: 200,
            pairs: 16,
            sigma: 0.05,
            learning_rate: 0.02,
            hidden: 32,
            seed: 2026,
            checkpoint_every: 10,
        }
    }
}

impl Config {
    pub(crate) fn to_json(self) -> Value {
        json!({"generations": self.generations, "pairs": self.pairs, "sigma": self.sigma,
               "learning_rate": self.learning_rate, "hidden": self.hidden, "seed": self.seed,
               "checkpoint_every": self.checkpoint_every})
    }
}

/// SplitMix64 normal deviates (Box–Muller), as example 144's residual draws.
pub(crate) struct Normal(pub(crate) u64);

impl Normal {
    fn uniform(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        ((z >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    pub(crate) fn sample(&mut self) -> f64 {
        let (u, v) = (self.uniform(), self.uniform());
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

/// Centred ranks in [-0.5, 0.5].
pub(crate) fn centred_ranks(values: &[f64]) -> Vec<f64> {
    let mut order: Vec<usize> = (0..values.len()).collect();
    order.sort_by(|&a, &b| values[a].total_cmp(&values[b]).then(a.cmp(&b)));
    let mut ranks = vec![0.0; values.len()];
    let denominator = (values.len() - 1).max(1) as f64;
    for (rank, index) in order.into_iter().enumerate() {
        ranks[index] = rank as f64 / denominator - 0.5;
    }
    ranks
}

/// Runs `job` for every index on `threads` threads, each with the state
/// `init` builds once.
pub(crate) fn run_parallel<S, T: Send>(
    jobs: usize,
    threads: usize,
    init: impl Fn() -> Result<S> + Sync,
    job: impl Fn(&mut S, usize) -> Result<T> + Sync,
) -> Result<Vec<T>> {
    let next = AtomicUsize::new(0);
    let results: Mutex<Vec<Option<std::result::Result<T, String>>>> =
        Mutex::new((0..jobs).map(|_| None).collect());
    std::thread::scope(|scope| {
        for _ in 0..threads.max(1) {
            scope.spawn(|| {
                let mut state = None;
                loop {
                    let index = next.fetch_add(1, Ordering::Relaxed);
                    if index >= jobs {
                        break;
                    }
                    let result = match &mut state {
                        Some(state) => job(state, index),
                        None => init().and_then(|built| job(state.insert(built), index)),
                    }
                    .map_err(|error| error.to_string());
                    results.lock().expect("results lock")[index] = Some(result);
                }
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

/// Each candidate's episode in RNE, on `threads` reused walkers.
fn evaluate(plant: Plant, candidates: &[Mlp], threads: usize) -> Result<Vec<Outcome>> {
    run_parallel(
        candidates.len(),
        threads,
        || go2::robot(plant),
        |robot, index| {
            go2::episode(robot.as_mut(), |observation| {
                candidates[index].act(observation)
            })
        },
    )
}

pub(crate) fn outcome_json(outcome: &Outcome) -> Value {
    json!({"fitness": outcome.fitness, "forward_m": outcome.forward_m,
           "lateral_m": outcome.lateral_m, "heading_rad": outcome.heading_rad,
           "survived_s": outcome.survived_s, "fell": outcome.fell})
}

/// Trains a walking policy from scratch in RNE; returns the training report.
pub(crate) fn train(plant: Plant, threads: usize, config: Config) -> Result<Value> {
    let mut normal = Normal(config.seed);
    let template = Mlp::initial(config.hidden, || normal.sample());
    let dimension = template.params.len();
    let mut theta = template.params.clone();
    let (mut m, mut v) = (vec![0.0; dimension], vec![0.0; dimension]);
    let (beta1, beta2) = (0.9, 0.999);
    let begin = Instant::now();
    let mut control_steps = 0;
    let (mut log, mut checkpoints) = (Vec::new(), Vec::new());
    for generation in 0..=config.generations {
        let noise: Vec<Vec<f64>> = (0..config.pairs)
            .map(|_| (0..dimension).map(|_| normal.sample()).collect())
            .collect();
        let mut candidates = vec![template.with_params(theta.clone())];
        if generation < config.generations {
            for epsilon in &noise {
                for sign in [1.0, -1.0] {
                    let params = theta
                        .iter()
                        .zip(epsilon)
                        .map(|(t, e)| t + sign * config.sigma * e)
                        .collect();
                    candidates.push(template.with_params(params));
                }
            }
        }
        let outcomes = evaluate(plant, &candidates, threads)?;
        control_steps += outcomes.iter().map(|o| o.control_steps).sum::<usize>();
        let fitness: Vec<f64> = outcomes.iter().map(|o| o.fitness).collect();
        let entry = json!({
            "generation": generation,
            "mean": outcome_json(&outcomes[0]),
            "population_mean_fitness": fitness.iter().sum::<f64>() / fitness.len() as f64,
            "population_best_fitness": fitness.iter().copied().fold(f64::NEG_INFINITY, f64::max),
            "wall_s": begin.elapsed().as_secs_f64(),
            "simulated_s": control_steps as f64 * CONTROL_DT_S,
        });
        println!("{entry}");
        if config.checkpoint_every > 0
            && (generation % config.checkpoint_every == 0 || generation == config.generations)
        {
            checkpoints.push(json!({
                "generation": generation,
                "wall_s": entry["wall_s"],
                "simulated_s": entry["simulated_s"],
                "policy": template.with_params(theta.clone()).to_json(),
            }));
        }
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
    Ok(json!({
        "schema_version": 1,
        "engine": plant.backend.name(),
        "plant": {"backend": plant.backend.name(),
                  "solver_iterations": plant.solver_iterations,
                  "inner_passes": plant.inner_passes,
                  "floor_friction": plant.floor_friction,
                  "terrain_seed": plant.terrain_seed},
        "threads": threads,
        "training": {
            "method": "evolution strategies, antithetic pairs, centred ranks, Adam",
            "parameters": dimension,
            "config": config.to_json(),
            "log": log,
            "checkpoints": checkpoints,
        },
        "policy": template.with_params(theta).to_json(),
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn centred_ranks_span_the_population() {
        assert_eq!(centred_ranks(&[3.0, 1.0, 2.0]), vec![0.5, -0.5, 0.0]);
    }

    #[test]
    fn normal_deviates_match_mujoco_go2_learning_py() {
        let fixture: Value = serde_json::from_str(include_str!("fixtures/mlp_check.json")).unwrap();
        let mut normal = Normal(7);
        for expected in fixture["normal_seed_7"].as_array().unwrap() {
            assert_eq!(normal.sample(), expected.as_f64().unwrap());
        }
    }
}
