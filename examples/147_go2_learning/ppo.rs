//! Proximal policy optimization (`--learner ppo`): the learner that
//! reinforcement learning for legged robots usually uses, as a second learner
//! beside evolution strategies. `mujoco_go2_learning.py` runs the same
//! algorithm with the same settings and random streams in MuJoCo.
//!
//! - **Policy.** A Gaussian around the same one-hidden-layer MLP the ES learns
//!   (`tanh` hidden units, `tanh` output in [-1, 1]), with a learned
//!   state-independent standard deviation per joint; the action applied is the
//!   sample clipped to [-1, 1]. Observations are normalized by running means
//!   and variances. The trained policy is exported with the normalization
//!   folded into its first layer, as an [`Mlp`] that `--evaluate` and MuJoCo
//!   run unchanged.
//! - **Critic.** A separate MLP with two `tanh` hidden layers.
//! - **Rollouts.** Every iteration, each of `envs` robots walks one episode
//!   with the stochastic policy, rewarded per control period so that the
//!   rewards sum to the ES fitness ([`go2::Step`]). A fall ends an episode;
//!   one that runs its full length bootstraps from the critic.
//! - **Update.** Generalized advantage estimation, advantages normalized over
//!   the iteration, then `epochs` passes of `minibatches` shuffled minibatches
//!   of the clipped surrogate and the squared value error, each network
//!   stepped by Adam after clipping its gradient's norm. The observation
//!   statistics take in the iteration's observations after the update.
//! - **Log.** Each iteration first walks the deterministic policy (the mean)
//!   once, from its own start, and logs that episode as the ES logs its mean.

use std::time::Instant;

use serde_json::{json, Value};

use crate::es::{outcome_json, run_parallel, Normal};
use crate::go2::{self, Outcome, Plant, Task, CONTROL_DT_S};
use crate::policy::{Mlp, ACTIONS};
use crate::Result;

/// Mixed into the seed for the action noise of iteration `i`, robot `e`
/// (`seed ^ NOISE_STREAM ^ (i << 20 | e)`), the minibatch shuffles
/// (`seed ^ SHUFFLE_STREAM ^ i`) and the critic's initial weights.
pub(crate) const NOISE_STREAM: u64 = 0x5050_4F4E_4F49_5345;
pub(crate) const SHUFFLE_STREAM: u64 = 0x5050_4F53_4855_4646;
pub(crate) const CRITIC_STREAM: u64 = 0x5050_4F43_5249_5449;
/// A minibatch's gradient is summed over this many fixed slices, in
/// parallel: the result does not depend on the thread count.
const GRADIENT_CHUNKS: usize = 8;

/// PPO settings.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Config {
    pub(crate) iterations: usize,
    /// Robots, one episode each, per iteration.
    pub(crate) envs: usize,
    /// Hidden units of the policy, and of each of the critic's two layers.
    pub(crate) hidden: usize,
    pub(crate) learning_rate: f64,
    /// Lower the learning rate linearly to zero over the run
    /// (`--anneal-learning-rate`), rather than hold it.
    pub(crate) anneal_learning_rate: bool,
    pub(crate) epochs: usize,
    pub(crate) minibatches: usize,
    pub(crate) clip: f64,
    pub(crate) gamma: f64,
    pub(crate) lambda: f64,
    /// Initial standard deviation of the action noise, in action units.
    pub(crate) initial_std: f64,
    pub(crate) max_grad_norm: f64,
    pub(crate) seed: u64,
    /// Store the policy every this many iterations (and at the end); 0 never.
    pub(crate) checkpoint_every: usize,
}

impl Default for Config {
    /// 400 iterations of 16 episodes: about the ES's budget, 200 generations
    /// of 33.
    fn default() -> Self {
        Self {
            iterations: 400,
            envs: 16,
            hidden: 64,
            learning_rate: 3e-4,
            anneal_learning_rate: false,
            epochs: 5,
            minibatches: 4,
            clip: 0.2,
            gamma: 0.99,
            lambda: 0.95,
            initial_std: 0.3,
            max_grad_norm: 1.0,
            seed: 2026,
            checkpoint_every: 20,
        }
    }
}

impl Config {
    pub(crate) fn to_json(self) -> Value {
        let mut value = json!({"learner": "ppo", "iterations": self.iterations, "envs": self.envs,
               "hidden": self.hidden, "learning_rate": self.learning_rate,
               "epochs": self.epochs, "minibatches": self.minibatches, "clip": self.clip,
               "gamma": self.gamma, "lambda": self.lambda, "initial_std": self.initial_std,
               "max_grad_norm": self.max_grad_norm, "seed": self.seed,
               "checkpoint_every": self.checkpoint_every});
        if self.anneal_learning_rate {
            value["anneal_learning_rate"] = json!(true);
        }
        value
    }

    /// The learning rate of the update after iteration `iteration`.
    fn learning_rate_at(self, iteration: usize) -> f64 {
        if self.anneal_learning_rate {
            self.learning_rate * (1.0 - iteration as f64 / self.iterations as f64)
        } else {
            self.learning_rate
        }
    }
}

/// Running observation means and variances (Chan's parallel update).
#[derive(Clone, Debug)]
pub(crate) struct Normalizer {
    count: f64,
    mean: Vec<f64>,
    m2: Vec<f64>,
}

impl Normalizer {
    fn new(width: usize) -> Self {
        Self {
            count: 0.0,
            mean: vec![0.0; width],
            m2: vec![0.0; width],
        }
    }

    /// Standard deviation of each input; 1 before any observation.
    fn scale(&self) -> Vec<f64> {
        self.m2
            .iter()
            .map(|m2| {
                if self.count > 0.0 {
                    (m2 / self.count + 1e-8).sqrt()
                } else {
                    1.0
                }
            })
            .collect()
    }

    fn normalize(&self, x: &[f64], scale: &[f64]) -> Vec<f64> {
        x.iter()
            .zip(&self.mean)
            .zip(scale)
            .map(|((x, mean), scale)| (x - mean) / scale)
            .collect()
    }

    fn update(&mut self, batch: &[Vec<f64>]) {
        if batch.is_empty() {
            return;
        }
        let n = batch.len() as f64;
        let width = self.mean.len();
        let mut mean = vec![0.0; width];
        for x in batch {
            for (m, v) in mean.iter_mut().zip(x) {
                *m += v / n;
            }
        }
        let mut m2 = vec![0.0; width];
        for x in batch {
            for ((s, v), m) in m2.iter_mut().zip(x).zip(&mean) {
                *s += (v - m) * (v - m);
            }
        }
        let total = self.count + n;
        for i in 0..width {
            let delta = mean[i] - self.mean[i];
            self.mean[i] += delta * n / total;
            self.m2[i] += m2[i] + delta * delta * self.count * n / total;
        }
        self.count = total;
    }

    /// `policy`, which reads normalized observations, as one that reads raw
    /// ones: x_n = (x - mean) / scale folded into its first layer.
    fn fold(&self, policy: &Mlp) -> Mlp {
        let scale = self.scale();
        let (inputs, hidden) = (policy.inputs, policy.hidden);
        let mut params = policy.params.clone();
        for unit in 0..hidden {
            let mut shift = 0.0;
            for input in 0..inputs {
                let weight = &mut params[unit * inputs + input];
                *weight /= scale[input];
                shift += *weight * self.mean[input];
            }
            params[hidden * inputs + unit] -= shift;
        }
        policy.with_params(params)
    }
}

/// The critic: inputs, two `tanh` layers of `hidden` units, one output.
/// Parameters flat: W1 (row per unit), b1, W2, b2, W3, b3.
#[derive(Clone, Debug)]
struct Critic {
    inputs: usize,
    hidden: usize,
    params: Vec<f64>,
}

impl Critic {
    fn new(inputs: usize, hidden: usize, normal: &mut Normal) -> Self {
        let count = hidden * inputs + hidden + hidden * hidden + hidden + hidden + 1;
        let mut params = vec![0.0; count];
        // W1 and W2 normal over √fan-in, the biases and W3 zero.
        for weight in &mut params[..hidden * inputs] {
            *weight = normal.sample() / (inputs as f64).sqrt();
        }
        let w2 = hidden * inputs + hidden;
        for weight in &mut params[w2..w2 + hidden * hidden] {
            *weight = normal.sample() / (hidden as f64).sqrt();
        }
        Self {
            inputs,
            hidden,
            params,
        }
    }

    /// The value, and the hidden activations for the backward pass.
    fn forward(&self, x: &[f64]) -> (f64, Vec<f64>, Vec<f64>) {
        let (inputs, hidden, p) = (self.inputs, self.hidden, &self.params);
        let (w1, rest) = p.split_at(hidden * inputs);
        let (b1, rest) = rest.split_at(hidden);
        let (w2, rest) = rest.split_at(hidden * hidden);
        let (b2, rest) = rest.split_at(hidden);
        let (w3, b3) = rest.split_at(hidden);
        let h1: Vec<f64> = (0..hidden)
            .map(|u| (dot(&w1[u * inputs..(u + 1) * inputs], x) + b1[u]).tanh())
            .collect();
        let h2: Vec<f64> = (0..hidden)
            .map(|u| (dot(&w2[u * hidden..(u + 1) * hidden], &h1) + b2[u]).tanh())
            .collect();
        (dot(w3, &h2) + b3[0], h1, h2)
    }

    /// Adds `d_value` times the value's gradient at `x` to `grad`.
    fn backward(&self, x: &[f64], h1: &[f64], h2: &[f64], d_value: f64, grad: &mut [f64]) {
        let (inputs, hidden, p) = (self.inputs, self.hidden, &self.params);
        let w2_at = hidden * inputs + hidden;
        let w3_at = w2_at + hidden * hidden + hidden;
        let mut d_h2 = vec![0.0; hidden];
        for u in 0..hidden {
            grad[w3_at + u] += d_value * h2[u];
            d_h2[u] = d_value * p[w3_at + u] * (1.0 - h2[u] * h2[u]);
        }
        grad[w3_at + hidden] += d_value;
        let mut d_h1 = vec![0.0; hidden];
        for u in 0..hidden {
            let row = w2_at + u * hidden;
            for v in 0..hidden {
                grad[row + v] += d_h2[u] * h1[v];
                d_h1[v] += d_h2[u] * p[row + v];
            }
            grad[w2_at + hidden * hidden + u] += d_h2[u];
        }
        for u in 0..hidden {
            let d = d_h1[u] * (1.0 - h1[u] * h1[u]);
            let row = u * inputs;
            for i in 0..inputs {
                grad[row + i] += d * x[i];
            }
            grad[hidden * inputs + u] += d;
        }
    }
}

/// Adam over a flat parameter vector, after scaling the gradient down to a
/// norm of at most `max_norm`.
#[derive(Clone, Debug)]
struct Adam {
    m: Vec<f64>,
    v: Vec<f64>,
    t: i32,
}

impl Adam {
    fn new(dimension: usize) -> Self {
        Self {
            m: vec![0.0; dimension],
            v: vec![0.0; dimension],
            t: 0,
        }
    }

    fn step(&mut self, params: &mut [f64], grad: &[f64], learning_rate: f64, max_norm: f64) {
        let norm = grad.iter().map(|g| g * g).sum::<f64>().sqrt();
        let scale = if norm > max_norm {
            max_norm / norm
        } else {
            1.0
        };
        let (beta1, beta2): (f64, f64) = (0.9, 0.999);
        self.t += 1;
        for i in 0..params.len() {
            let g = grad[i] * scale;
            self.m[i] = beta1 * self.m[i] + (1.0 - beta1) * g;
            self.v[i] = beta2 * self.v[i] + (1.0 - beta2) * g * g;
            let m_hat = self.m[i] / (1.0 - beta1.powi(self.t));
            let v_hat = self.v[i] / (1.0 - beta2.powi(self.t));
            params[i] -= learning_rate * m_hat / (v_hat.sqrt() + 1e-5);
        }
    }
}

/// One robot's episode with the stochastic policy.
struct Trajectory {
    /// Normalized observations, sampled (unclipped) actions and their log
    /// probabilities, and rewards, one per control period.
    observations: Vec<Vec<f64>>,
    raw_observations: Vec<Vec<f64>>,
    actions: Vec<[f64; ACTIONS]>,
    log_probs: Vec<f64>,
    rewards: Vec<f64>,
    /// The normalized observation after the last period, when the episode ran
    /// its full length (bootstrapped from the critic); None after a fall.
    bootstrap: Option<Vec<f64>>,
    outcome: Outcome,
}

/// The Gaussian's log density of `action` around `mean`.
fn log_prob(action: &[f64; ACTIONS], mean: &[f64; ACTIONS], log_std: &[f64; ACTIONS]) -> f64 {
    let half_log_tau = 0.5 * std::f64::consts::TAU.ln();
    (0..ACTIONS)
        .map(|j| {
            let z = (action[j] - mean[j]) / log_std[j].exp();
            -0.5 * z * z - log_std[j] - half_log_tau
        })
        .sum()
}

/// The policy's means for a normalized observation, and its hidden units.
fn actor_forward(actor: &Mlp, x: &[f64]) -> ([f64; ACTIONS], Vec<f64>) {
    let (inputs, hidden) = (actor.inputs, actor.hidden);
    let (w1, rest) = actor.params.split_at(hidden * inputs);
    let (b1, rest) = rest.split_at(hidden);
    let (w2, b2) = rest.split_at(ACTIONS * hidden);
    let h: Vec<f64> = (0..hidden)
        .map(|u| (dot(&w1[u * inputs..(u + 1) * inputs], x) + b1[u]).tanh())
        .collect();
    let mut mean = [0.0; ACTIONS];
    for (j, m) in mean.iter_mut().enumerate() {
        *m = (dot(&w2[j * hidden..(j + 1) * hidden], &h) + b2[j]).tanh();
    }
    (mean, h)
}

/// Adds `d_mean` back-propagated through the actor at `x` to `grad`.
fn actor_backward(
    actor: &Mlp,
    x: &[f64],
    h: &[f64],
    mean: &[f64; ACTIONS],
    d_mean: &[f64; ACTIONS],
    grad: &mut [f64],
) {
    let (inputs, hidden, p) = (actor.inputs, actor.hidden, &actor.params);
    let w2_at = hidden * inputs + hidden;
    let b2_at = w2_at + ACTIONS * hidden;
    let mut d_h = vec![0.0; hidden];
    for j in 0..ACTIONS {
        let d = d_mean[j] * (1.0 - mean[j] * mean[j]);
        let row = w2_at + j * hidden;
        for u in 0..hidden {
            grad[row + u] += d * h[u];
            d_h[u] += d * p[row + u];
        }
        grad[b2_at + j] += d;
    }
    for u in 0..hidden {
        let d = d_h[u] * (1.0 - h[u] * h[u]);
        for i in 0..inputs {
            grad[u * inputs + i] += d * x[i];
        }
        grad[hidden * inputs + u] += d;
    }
}

/// The deterministic policy (the Gaussian's mean) for raw observations.
fn deterministic(actor: &Mlp, normalizer: &Normalizer) -> Mlp {
    normalizer.fold(actor)
}

#[allow(clippy::too_many_arguments)]
fn rollout(
    robot: &mut dyn go2::Robot,
    task: Task,
    start: [f64; 2],
    actor: &Mlp,
    log_std: &[f64; ACTIONS],
    normalizer: &Normalizer,
    scale: &[f64],
    noise_seed: u64,
) -> Result<Trajectory> {
    let mut noise = Normal(noise_seed);
    let mut trajectory = Trajectory {
        observations: Vec::new(),
        raw_observations: Vec::new(),
        actions: Vec::new(),
        log_probs: Vec::new(),
        rewards: Vec::new(),
        bootstrap: None,
        outcome: Outcome::default(),
    };
    let mut fell = false;
    let (outcome, last) = {
        let trajectory = &mut trajectory;
        let rewards = std::cell::RefCell::new(Vec::new());
        let result = go2::episode_with(
            robot,
            task,
            start,
            |observation| {
                let x = normalizer.normalize(observation, scale);
                let (mean, _) = actor_forward(actor, &x);
                let mut action = [0.0; ACTIONS];
                let mut applied = [0.0; ACTIONS];
                for j in 0..ACTIONS {
                    action[j] = mean[j] + log_std[j].exp() * noise.sample();
                    applied[j] = action[j].clamp(-1.0, 1.0);
                }
                trajectory.log_probs.push(log_prob(&action, &mean, log_std));
                trajectory.actions.push(action);
                trajectory.raw_observations.push(observation.to_vec());
                trajectory.observations.push(x);
                applied
            },
            |step| {
                rewards.borrow_mut().push(step.reward);
                fell |= step.fell;
            },
        )?;
        trajectory.rewards = rewards.into_inner();
        result
    };
    if !fell {
        trajectory.bootstrap = Some(normalizer.normalize(&last, scale));
    }
    trajectory.outcome = outcome;
    Ok(trajectory)
}

/// Trains a walking policy with PPO in RNE; returns the training report,
/// shaped as `es::train`'s.
pub(crate) fn train(plant: Plant, task: Task, threads: usize, config: Config) -> Result<Value> {
    if task.random_starts && plant.terrain_seed.is_none() {
        return Err("random starts need a terrain (--terrain SEED)".into());
    }
    let inputs = task.observations();
    let mut normal = Normal(config.seed);
    let mut actor = Mlp::initial(inputs, config.hidden, || normal.sample());
    let mut critic = Critic::new(
        inputs,
        config.hidden,
        &mut Normal(config.seed ^ CRITIC_STREAM),
    );
    let mut log_std = [config.initial_std.ln(); ACTIONS];
    let mut normalizer = Normalizer::new(inputs);
    let (mut actor_adam, mut std_adam, mut critic_adam) = (
        Adam::new(actor.params.len()),
        Adam::new(ACTIONS),
        Adam::new(critic.params.len()),
    );
    // Each iteration's robots start at their own points, and the logged
    // deterministic episode at one more.
    let per_iteration = config.envs + 1;
    // (iterations + 1) × per_iteration points: the last iteration logs its
    // deterministic episode too.
    let starts = go2::training_starts(
        config.seed,
        (config.iterations + 1) * per_iteration - 1,
        &go2::plant_gaps(plant),
    );
    let begin = Instant::now();
    let mut control_steps = 0;
    let (mut log, mut checkpoints) = (Vec::new(), Vec::new());
    for iteration in 0..=config.iterations {
        let start_of = |index: usize| {
            if task.random_starts {
                starts[iteration * per_iteration + index]
            } else {
                [0.0, 0.0]
            }
        };
        let scale = normalizer.scale();
        let policy = deterministic(&actor, &normalizer);
        let jobs = if iteration < config.iterations {
            per_iteration
        } else {
            1
        };
        enum Job {
            Mean(Outcome),
            Rollout(Box<Trajectory>),
        }
        let results = run_parallel(
            jobs,
            threads,
            || go2::robot(plant),
            |robot, index| {
                if index == 0 {
                    let outcome = go2::episode(robot.as_mut(), task, start_of(config.envs), |x| {
                        policy.act(x)
                    })?;
                    Ok(Job::Mean(outcome))
                } else {
                    let env = index - 1;
                    let seed = config.seed ^ NOISE_STREAM ^ ((iteration as u64) << 20 | env as u64);
                    Ok(Job::Rollout(Box::new(rollout(
                        robot.as_mut(),
                        task,
                        start_of(env),
                        &actor,
                        &log_std,
                        &normalizer,
                        &scale,
                        seed,
                    )?)))
                }
            },
        )?;
        let mut mean_outcome = Outcome::default();
        let mut trajectories = Vec::new();
        for result in results {
            match result {
                Job::Mean(outcome) => mean_outcome = outcome,
                Job::Rollout(trajectory) => trajectories.push(*trajectory),
            }
        }
        control_steps += mean_outcome.control_steps
            + trajectories
                .iter()
                .map(|t| t.outcome.control_steps)
                .sum::<usize>();
        let fitness: Vec<f64> = trajectories.iter().map(|t| t.outcome.fitness).collect();
        let entry = json!({
            "generation": iteration,
            "mean": outcome_json(&mean_outcome),
            "population_mean_fitness": if fitness.is_empty() { Value::Null }
                else { json!(fitness.iter().sum::<f64>() / fitness.len() as f64) },
            "population_best_fitness": fitness.iter().copied().reduce(f64::max),
            "action_std": log_std.iter().map(|l| l.exp()).sum::<f64>() / ACTIONS as f64,
            "wall_s": begin.elapsed().as_secs_f64(),
            "simulated_s": control_steps as f64 * CONTROL_DT_S,
        });
        println!("{entry}");
        if config.checkpoint_every > 0
            && (iteration % config.checkpoint_every == 0 || iteration == config.iterations)
        {
            checkpoints.push(json!({
                "generation": iteration,
                "wall_s": entry["wall_s"],
                "simulated_s": entry["simulated_s"],
                "policy": policy.to_json(),
            }));
        }
        log.push(entry);
        if iteration == config.iterations {
            break;
        }
        update(
            &mut actor,
            &mut critic,
            &mut log_std,
            (&mut actor_adam, &mut std_adam, &mut critic_adam),
            &trajectories,
            iteration,
            config,
        );
        let observations: Vec<Vec<f64>> = trajectories
            .iter()
            .flat_map(|t| t.raw_observations.iter().cloned())
            .collect();
        normalizer.update(&observations);
    }
    let policy = deterministic(&actor, &normalizer);
    Ok(json!({
        "schema_version": 1,
        "engine": plant.backend.name(),
        "plant": go2::plant_json(plant),
        "task": task.to_json(),
        "threads": threads,
        "training": {
            "method": "proximal policy optimization, clipped surrogate, GAE, Adam",
            "parameters": actor.params.len() + ACTIONS + critic.params.len(),
            "config": config.to_json(),
            "log": log,
            "checkpoints": checkpoints,
        },
        "policy": policy.to_json(),
    }))
}

/// One control period of the iteration's experience.
#[derive(Clone, Copy)]
struct Sample<'a> {
    x: &'a [f64],
    action: &'a [f64; ACTIONS],
    old_log_prob: f64,
    advantage: f64,
    value_target: f64,
}

/// One PPO update from this iteration's trajectories.
fn update(
    actor: &mut Mlp,
    critic: &mut Critic,
    log_std: &mut [f64; ACTIONS],
    (actor_adam, std_adam, critic_adam): (&mut Adam, &mut Adam, &mut Adam),
    trajectories: &[Trajectory],
    iteration: usize,
    config: Config,
) {
    let learning_rate = config.learning_rate_at(iteration);
    // Generalized advantages and returns, per trajectory.
    let mut samples: Vec<Sample> = Vec::new();
    for t in trajectories {
        let values: Vec<f64> = t.observations.iter().map(|x| critic.forward(x).0).collect();
        let last_value = t.bootstrap.as_ref().map_or(0.0, |x| critic.forward(x).0);
        let mut advantages = vec![0.0; values.len()];
        let mut running = 0.0;
        for step in (0..values.len()).rev() {
            let next = if step + 1 < values.len() {
                values[step + 1]
            } else {
                last_value
            };
            let delta = t.rewards[step] + config.gamma * next - values[step];
            running = delta + config.gamma * config.lambda * running;
            advantages[step] = running;
        }
        for step in 0..values.len() {
            samples.push(Sample {
                x: &t.observations[step],
                action: &t.actions[step],
                old_log_prob: t.log_probs[step],
                advantage: advantages[step],
                value_target: advantages[step] + values[step],
            });
        }
    }
    if samples.is_empty() {
        return;
    }
    let count = samples.len() as f64;
    let mean = samples.iter().map(|s| s.advantage).sum::<f64>() / count;
    let std = (samples
        .iter()
        .map(|s| (s.advantage - mean).powi(2))
        .sum::<f64>()
        / count)
        .sqrt()
        + 1e-8;
    let mut shuffle = Normal(config.seed ^ SHUFFLE_STREAM ^ iteration as u64);
    let mut order: Vec<usize> = (0..samples.len()).collect();
    for _ in 0..config.epochs {
        // Fisher–Yates with SplitMix64 uniforms, as the Python copy.
        for i in (1..order.len()).rev() {
            let j = ((shuffle.uniform() * (i + 1) as f64) as usize).min(i);
            order.swap(i, j);
        }
        let size = order.len().div_ceil(config.minibatches);
        for batch in order.chunks(size) {
            let n = batch.len() as f64;
            // The batch's gradient, summed over GRADIENT_CHUNKS fixed slices in
            // parallel and then in slice order, so it does not depend on the
            // thread count.
            let (actor_ref, critic_ref, log_std_ref) = (&*actor, &*critic, &*log_std);
            let samples = &samples;
            let chunk_gradient = |chunk: &[usize]| {
                let mut actor_grad = vec![0.0; actor_ref.params.len()];
                let mut std_grad = [0.0; ACTIONS];
                let mut critic_grad = vec![0.0; critic_ref.params.len()];
                for &index in chunk {
                    let Sample {
                        x,
                        action,
                        old_log_prob,
                        advantage,
                        value_target,
                    } = samples[index];
                    let advantage = (advantage - mean) / std;
                    let (mean_action, h) = actor_forward(actor_ref, x);
                    let new_log_prob = log_prob(action, &mean_action, log_std_ref);
                    let ratio = (new_log_prob - old_log_prob).exp();
                    let clipped = (advantage > 0.0 && ratio > 1.0 + config.clip)
                        || (advantage < 0.0 && ratio < 1.0 - config.clip);
                    if !clipped {
                        // d(-ratio A)/d(log p) = -ratio A, averaged over the batch.
                        let d_log_prob = -ratio * advantage / n;
                        let mut d_mean = [0.0; ACTIONS];
                        for j in 0..ACTIONS {
                            let variance = (2.0 * log_std_ref[j]).exp();
                            let diff = action[j] - mean_action[j];
                            d_mean[j] = d_log_prob * diff / variance;
                            std_grad[j] += d_log_prob * (diff * diff / variance - 1.0);
                        }
                        actor_backward(actor_ref, x, &h, &mean_action, &d_mean, &mut actor_grad);
                    }
                    let (value, h1, h2) = critic_ref.forward(x);
                    critic_ref.backward(x, &h1, &h2, (value - value_target) / n, &mut critic_grad);
                }
                (actor_grad, std_grad, critic_grad)
            };
            let slice = batch.len().div_ceil(GRADIENT_CHUNKS);
            let parts: Vec<_> = std::thread::scope(|scope| {
                let handles: Vec<_> = batch
                    .chunks(slice)
                    .map(|chunk| scope.spawn(move || chunk_gradient(chunk)))
                    .collect();
                handles
                    .into_iter()
                    .map(|handle| handle.join().expect("gradient thread"))
                    .collect()
            });
            let mut actor_grad = vec![0.0; actor.params.len()];
            let mut std_grad = [0.0; ACTIONS];
            let mut critic_grad = vec![0.0; critic.params.len()];
            for (a, s, c) in &parts {
                for (total, part) in actor_grad.iter_mut().zip(a) {
                    *total += part;
                }
                for (total, part) in std_grad.iter_mut().zip(s) {
                    *total += part;
                }
                for (total, part) in critic_grad.iter_mut().zip(c) {
                    *total += part;
                }
            }
            actor_adam.step(
                &mut actor.params,
                &actor_grad,
                learning_rate,
                config.max_grad_norm,
            );
            std_adam.step(log_std, &std_grad, learning_rate, config.max_grad_norm);
            critic_adam.step(
                &mut critic.params,
                &critic_grad,
                learning_rate,
                config.max_grad_norm,
            );
        }
    }
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_folded_policy_reads_raw_observations_as_the_actor_reads_normalized_ones() {
        let mut normal = Normal(3);
        let actor = Mlp::initial(5, 4, || normal.sample());
        let actor = actor.with_params(actor.params.iter().map(|p| p + 0.1).collect());
        let mut normalizer = Normalizer::new(5);
        normalizer.update(&[
            vec![1.0, 2.0, 3.0, 4.0, 5.0],
            vec![2.0, 0.0, 1.0, 4.5, -5.0],
            vec![0.5, 1.0, 2.0, 3.0, 0.0],
        ]);
        let raw = [0.7, 1.1, -0.3, 4.2, 2.0];
        let scale = normalizer.scale();
        let (mean, _) = actor_forward(&actor, &normalizer.normalize(&raw, &scale));
        let folded = normalizer.fold(&actor).act(&raw);
        for j in 0..ACTIONS {
            assert!((mean[j] - folded[j]).abs() < 1e-12);
        }
    }

    #[test]
    fn a_short_run_from_random_starts_trains_and_exports_a_policy() {
        let plant = Plant {
            backend: go2::Backend::Locomotion,
            terrain_seed: Some(7),
            ..Plant::default()
        };
        let task = Task {
            height_scan: true,
            random_starts: true,
        };
        let config = Config {
            iterations: 2,
            envs: 2,
            hidden: 8,
            checkpoint_every: 1,
            ..Config::default()
        };
        let report = train(plant, task, 2, config).unwrap();
        let log = report["training"]["log"].as_array().unwrap();
        assert_eq!(log.len(), 3);
        assert_eq!(
            report["training"]["checkpoints"].as_array().unwrap().len(),
            3
        );
        let policy = Mlp::from_json(&report["policy"]).unwrap();
        assert_eq!((policy.inputs, policy.hidden), (59, 8));
        // The exported policy is the last logged one.
        let mut robot = go2::robot(plant).unwrap();
        let last = log.last().unwrap();
        let start = go2::training_starts(config.seed, 3 * (config.envs + 1) - 1, &[])[2 * 3 + 2];
        let outcome = go2::episode(robot.as_mut(), task, start, |x| policy.act(x)).unwrap();
        assert_eq!(outcome_json(&outcome), last["mean"]);
    }

    #[test]
    fn the_normalizer_matches_the_batch_statistics() {
        let batch: Vec<Vec<f64>> = (0..10).map(|i| vec![i as f64, (i * i) as f64]).collect();
        let mut whole = Normalizer::new(2);
        whole.update(&batch);
        let mut parts = Normalizer::new(2);
        parts.update(&batch[..3]);
        parts.update(&batch[3..]);
        for i in 0..2 {
            assert!((whole.mean[i] - parts.mean[i]).abs() < 1e-12);
            assert!((whole.m2[i] - parts.m2[i]).abs() < 1e-9);
        }
        assert!((whole.mean[0] - 4.5).abs() < 1e-12);
        assert!((whole.m2[0] / 10.0 - 8.25).abs() < 1e-12);
    }

    #[test]
    fn the_gradients_match_finite_differences() {
        let mut normal = Normal(5);
        let actor = Mlp::initial(4, 3, || normal.sample());
        let actor = actor.with_params(actor.params.iter().map(|p| p + 0.05).collect());
        let critic = Critic::new(4, 3, &mut Normal(9));
        let critic = Critic {
            params: critic.params.iter().map(|p| p + 0.07).collect(),
            ..critic
        };
        let x = [0.3, -0.2, 0.5, 0.1];
        let action = [0.2; ACTIONS];
        let log_std = [-1.0_f64; ACTIONS];
        // log p's gradient with respect to the actor's parameters.
        let (mean, h) = actor_forward(&actor, &x);
        let mut d_mean = [0.0; ACTIONS];
        for j in 0..ACTIONS {
            d_mean[j] = (action[j] - mean[j]) / (2.0 * log_std[j]).exp();
        }
        let mut grad = vec![0.0; actor.params.len()];
        actor_backward(&actor, &x, &h, &mean, &d_mean, &mut grad);
        for index in [0, 5, actor.params.len() - 1, actor.params.len() - 20] {
            let mut up = actor.clone();
            up.params[index] += 1e-6;
            let mut down = actor.clone();
            down.params[index] -= 1e-6;
            let numeric = (log_prob(&action, &actor_forward(&up, &x).0, &log_std)
                - log_prob(&action, &actor_forward(&down, &x).0, &log_std))
                / 2e-6;
            assert!(
                (numeric - grad[index]).abs() < 1e-6,
                "{index}: {numeric} {}",
                grad[index]
            );
        }
        // The value's gradient with respect to the critic's parameters.
        let (_, h1, h2) = critic.forward(&x);
        let mut grad = vec![0.0; critic.params.len()];
        critic.backward(&x, &h1, &h2, 1.0, &mut grad);
        for index in [0, 7, 13, critic.params.len() - 2, critic.params.len() - 1] {
            let mut up = critic.clone();
            up.params[index] += 1e-6;
            let mut down = critic.clone();
            down.params[index] -= 1e-6;
            let numeric = (up.forward(&x).0 - down.forward(&x).0) / 2e-6;
            assert!(
                (numeric - grad[index]).abs() < 1e-6,
                "{index}: {numeric} {}",
                grad[index]
            );
        }
    }
}
