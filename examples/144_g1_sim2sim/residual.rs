//! A residual on the pretrained walking policy, trained in RNE by evolution
//! strategies to recover from pushes.
//!
//! The residual adds offsets, bounded by ±[`LIMIT_RAD`], to every joint
//! target for the next policy period, computed from the same observation as
//! the policy's action ([`rne_ai::UnitreeRlGymWalker::offset_targets_rad`]).
//! `mujoco_robustness.py --residual` applies the same law in MuJoCo. Two
//! models exist:
//!
//! - **Linear**: `clamp(W · x)` on five observation entries, the scaled
//!   pelvis angular velocity and the x and y of gravity projected into the
//!   pelvis frame. Both are zero for an upright pelvis at rest.
//! - **MLP**: `LIMIT · tanh(W₂ · tanh(W₁ · x + b₁))` on those five entries
//!   plus the joint position offsets, the scaled joint velocities and the
//!   gait phase's sine and cosine. W₂ starts at zero, so training starts from
//!   the plain policy; W₁ starts from the seed's first normal deviates.
//!
//! Training (OpenAI-style evolution strategies with antithetic pairs, centred
//! ranks and Adam) scores each candidate on fixed push scenarios near the
//! forces where the plain policy falls. A survived push scores 1 plus up to
//! 0.5 for how upright the pelvis stayed (none at a tilt of [`TILT_REF_RAD`]
//! or more); a fall scores half the share of the observation window it stood
//! through. Candidates pay the mean squared target offset times a penalty.

use std::time::Instant;

use rne_ai::{LstmPolicy, UnitreeRlGymWalker};
use rne_math::Vec3;
use serde_json::{json, Value};

use crate::policy::{self, Result, Robot};
use crate::robustness::{run_parallel_with, COMMAND, DIRECTIONS, PHASE_STEPS, PUSH_STEPS};
use crate::walk::{self, PlantConfig, StepRecord, FALL_HEIGHT_M};

/// Observation entries the linear residual reads: angular velocity (3) and
/// the horizontal components of projected gravity (2).
pub(crate) const FEATURES: usize = 5;
/// Largest target offset the residual may add, in radians.
pub(crate) const LIMIT_RAD: f64 = 0.4;
/// Default extra penalty on offsets while walking undisturbed, for the MLP.
pub(crate) const MLP_WALK_OFFSET_PENALTY: f64 = 25.0;
/// Default penalty on the change in undisturbed walking speed, for the MLP,
/// in score per (m/s)².
pub(crate) const MLP_SPEED_PENALTY: f64 = 50.0;
/// Policy steps of the undisturbed speed check (4 s); the speed is measured
/// over the second half.
const SPEED_CHECK_STEPS: usize = 200;

/// Residual architecture.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Model {
    Linear,
    Mlp { hidden: usize },
}

impl Model {
    /// Parses `linear` or `mlp`, the MLP with `hidden` units.
    pub(crate) fn parse(name: &str, hidden: usize) -> Result<Self> {
        match name {
            "linear" => Ok(Self::Linear),
            "mlp" if hidden > 0 => Ok(Self::Mlp { hidden }),
            "mlp" => Err("--hidden must be at least 1".into()),
            other => Err(format!("unknown residual model {other} (linear or mlp)").into()),
        }
    }
}

/// Observation indices a model reads, for a policy with `joints` actions
/// (observation: angular velocity 3, gravity 3, command 3, joint positions,
/// joint velocities, previous action, phase sine and cosine).
fn input_indices(model: Model, joints: usize) -> Vec<usize> {
    match model {
        Model::Linear => (0..FEATURES).collect(),
        Model::Mlp { .. } => (0..FEATURES)
            .chain(9..9 + 2 * joints)
            .chain(9 + 3 * joints..11 + 3 * joints)
            .collect(),
    }
}

fn input_name(index: usize, joints: usize) -> String {
    match index {
        0..=2 => format!("angular_velocity_{}_scaled", ["x", "y", "z"][index]),
        3..=5 => format!("projected_gravity_{}", ["x", "y", "z"][index - 3]),
        i if (9..9 + joints).contains(&i) => format!("joint_position_offset_{}", i - 9),
        i if (9 + joints..9 + 2 * joints).contains(&i) => {
            format!("joint_velocity_scaled_{}", i - 9 - joints)
        }
        i if i == 9 + 3 * joints => "phase_sin".into(),
        i if i == 10 + 3 * joints => "phase_cos".into(),
        i => format!("observation_{i}"),
    }
}

/// A residual: its model, the observation entries it reads and its flat
/// parameter vector (linear: `joints × inputs` weights, row-major; MLP: W₁
/// `hidden × inputs`, then b₁, then W₂ `joints × hidden`, row-major).
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Residual {
    model: Model,
    joints: usize,
    inputs: Vec<usize>,
    params: Vec<f64>,
}

impl Residual {
    /// A residual that adds nothing (for the MLP, W₁ is drawn from `normal`).
    fn initial(model: Model, joints: usize, normal: &mut Normal) -> Self {
        let inputs = input_indices(model, joints);
        let params = match model {
            Model::Linear => vec![0.0; joints * inputs.len()],
            Model::Mlp { hidden } => {
                let scale = 1.0 / (inputs.len() as f64).sqrt();
                let mut params: Vec<f64> = (0..hidden * inputs.len())
                    .map(|_| scale * normal.sample())
                    .collect();
                params.resize(params.len() + hidden + joints * hidden, 0.0);
                params
            }
        };
        Self {
            model,
            joints,
            inputs,
            params,
        }
    }

    #[cfg(test)]
    pub(crate) fn zero(joints: usize) -> Self {
        Self::initial(Model::Linear, joints, &mut Normal(0))
    }

    #[cfg(test)]
    fn joints(&self) -> usize {
        self.joints
    }

    fn with_params(&self, params: Vec<f64>) -> Self {
        Self {
            params,
            ..self.clone()
        }
    }

    /// Target offsets for the observation the policy just received.
    pub(crate) fn offsets(&self, observation: &[f32]) -> Vec<f64> {
        let x: Vec<f64> = self
            .inputs
            .iter()
            .map(|&i| f64::from(observation[i]))
            .collect();
        let n = x.len();
        match self.model {
            Model::Linear => self
                .params
                .chunks_exact(n)
                .map(|row| dot(row, &x).clamp(-LIMIT_RAD, LIMIT_RAD))
                .collect(),
            Model::Mlp { hidden } => {
                let (w1, rest) = self.params.split_at(hidden * n);
                let (b1, w2) = rest.split_at(hidden);
                let h: Vec<f64> = w1
                    .chunks_exact(n)
                    .zip(b1)
                    .map(|(row, b)| (dot(row, &x) + b).tanh())
                    .collect();
                w2.chunks_exact(hidden)
                    .map(|row| LIMIT_RAD * dot(row, &h).tanh())
                    .collect()
            }
        }
    }

    pub(crate) fn to_json(&self) -> Value {
        let names: Vec<String> = self
            .inputs
            .iter()
            .map(|&i| input_name(i, self.joints))
            .collect();
        match self.model {
            Model::Linear => json!({
                "features": names,
                "limit_rad": LIMIT_RAD,
                "weights_rad": self.params.chunks_exact(self.inputs.len()).collect::<Vec<_>>(),
            }),
            Model::Mlp { hidden } => {
                let n = self.inputs.len();
                let (w1, rest) = self.params.split_at(hidden * n);
                let (b1, w2) = rest.split_at(hidden);
                json!({
                    "model": "mlp",
                    "features": names,
                    "observation_indices": self.inputs,
                    "hidden": hidden,
                    "activation": "tanh",
                    "output": "limit_rad * tanh(w2 . tanh(w1 . x + b1))",
                    "limit_rad": LIMIT_RAD,
                    "w1": w1.chunks_exact(n).collect::<Vec<_>>(),
                    "b1": b1,
                    "w2": w2.chunks_exact(hidden).collect::<Vec<_>>(),
                })
            }
        }
    }

    pub(crate) fn from_json(value: &Value) -> Result<Self> {
        if value["limit_rad"].as_f64() != Some(LIMIT_RAD) {
            return Err("residual was trained with a different offset limit".into());
        }
        let matrix = |key: &str, columns: usize| -> Result<Vec<Vec<f64>>> {
            let rows = value[key]
                .as_array()
                .ok_or_else(|| format!("residual has no {key}"))?;
            rows.iter()
                .map(|row| {
                    let row = row
                        .as_array()
                        .filter(|row| row.len() == columns)
                        .ok_or_else(|| format!("{key} rows need {columns} numbers"))?;
                    row.iter()
                        .map(|w| w.as_f64().ok_or_else(|| format!("{key} must hold numbers")))
                        .collect::<std::result::Result<Vec<_>, _>>()
                })
                .collect::<std::result::Result<Vec<_>, String>>()
                .map_err(Into::into)
        };
        if value["model"].as_str() == Some("mlp") {
            let hidden = value["hidden"]
                .as_u64()
                .ok_or("mlp residual has no hidden")? as usize;
            let inputs: Vec<usize> = value["observation_indices"]
                .as_array()
                .ok_or("mlp residual has no observation_indices")?
                .iter()
                .map(|i| {
                    i.as_u64()
                        .map(|i| i as usize)
                        .ok_or("bad observation index")
                })
                .collect::<std::result::Result<_, _>>()?;
            let w1 = matrix("w1", inputs.len())?;
            let b1: Vec<f64> = value["b1"]
                .as_array()
                .filter(|b| b.len() == hidden)
                .ok_or("b1 needs one number per hidden unit")?
                .iter()
                .map(|b| b.as_f64().ok_or("b1 must hold numbers"))
                .collect::<std::result::Result<_, _>>()?;
            let w2 = matrix("w2", hidden)?;
            if w1.len() != hidden {
                return Err("w1 needs one row per hidden unit".into());
            }
            let joints = w2.len();
            if input_indices(Model::Mlp { hidden }, joints) != inputs {
                return Err("mlp residual reads observation entries this example does not".into());
            }
            let mut params: Vec<f64> = w1.concat();
            params.extend(b1);
            params.extend(w2.concat());
            return Ok(Self {
                model: Model::Mlp { hidden },
                joints,
                inputs,
                params,
            });
        }
        let rows = matrix("weights_rad", FEATURES)?;
        Ok(Self {
            model: Model::Linear,
            joints: rows.len(),
            inputs: input_indices(Model::Linear, rows.len()),
            params: rows.concat(),
        })
    }
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(a, b)| a * b).sum()
}

/// The residual of a training report: its final one, or the checkpoint
/// stored at `generation`.
pub(crate) fn from_report(report: &Value, generation: Option<usize>) -> Result<Residual> {
    let Some(generation) = generation else {
        return Residual::from_json(&report["residual"]);
    };
    let checkpoint = report["training"]["checkpoints"]
        .as_array()
        .and_then(|list| {
            list.iter()
                .find(|c| c["generation"].as_u64() == Some(generation as u64))
        })
        .ok_or_else(|| format!("the report stores no checkpoint at generation {generation}"))?;
    Residual::from_json(&checkpoint["residual"])
}

/// Steps the walker, then offsets its next targets by the residual, if any.
pub(crate) fn step(
    walker: &mut UnitreeRlGymWalker,
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

/// What one push scenario did.
#[derive(Clone, Copy, Debug)]
struct Outcome {
    score: f64,
    /// Mean squared target offset over the whole scenario, rad².
    offset_square: f64,
    /// Mean squared target offset while walking undisturbed before the push.
    walk_offset_square: f64,
    steps: usize,
}

fn run_scenario(
    walker: &mut UnitreeRlGymWalker,
    residual: &Residual,
    scenario: Scenario,
) -> Result<Outcome> {
    walker.reset()?;
    let [x, y] = DIRECTIONS[scenario.direction].1;
    let force = Vec3::new(x * scenario.force_n, 0.0, -y * scenario.force_n);
    let start = TRAIN_WALK_STEPS + scenario.phase * PHASE_STEPS;
    let end = start + PUSH_STEPS + TRAIN_RECOVERY_STEPS;
    let (mut offset_squares, mut walk_offset_squares) = (0.0, 0.0);
    let mut max_tilt_rad: f64 = 0.0;
    let outcome = |score, offset_squares: f64, walk_offset_squares: f64, steps: usize| Outcome {
        score,
        offset_square: offset_squares / steps as f64,
        walk_offset_square: walk_offset_squares / steps.min(start) as f64,
        steps,
    };
    for step in 0..end {
        walker.set_base_push_n((start..start + PUSH_STEPS).contains(&step).then_some(force));
        let record = walker.step(COMMAND)?;
        let offsets = residual.offsets(&record.observation);
        let square = offsets.iter().map(|o| o * o).sum::<f64>() / offsets.len() as f64;
        offset_squares += square;
        if step < start {
            walk_offset_squares += square;
        }
        walker.offset_targets_rad(&offsets)?;
        if step >= start {
            // Projected gravity's z is -cos(tilt).
            let cos_tilt = -f64::from(record.observation[5]);
            max_tilt_rad = max_tilt_rad.max(cos_tilt.clamp(-1.0, 1.0).acos());
        }
        if record.base_position_m[2] < FALL_HEIGHT_M {
            let stood = (step + 1).saturating_sub(start) as f64 / (end - start) as f64;
            return Ok(outcome(
                0.5 * stood,
                offset_squares,
                walk_offset_squares,
                step + 1,
            ));
        }
    }
    let upright = 0.5 * (1.0 - (max_tilt_rad / TILT_REF_RAD).min(1.0));
    Ok(outcome(
        1.0 + upright,
        offset_squares,
        walk_offset_squares,
        end,
    ))
}

/// Mean forward speed over the second half of an undisturbed walk (zero if
/// it falls), and the policy steps taken.
fn walk_speed(walker: &mut UnitreeRlGymWalker, residual: &Residual) -> Result<(f64, usize)> {
    walker.reset()?;
    walker.set_base_push_n(None);
    let half = SPEED_CHECK_STEPS / 2;
    let mut halfway_x = 0.0;
    for step in 0..SPEED_CHECK_STEPS {
        let record = walk_step(walker, residual)?;
        if record.base_position_m[2] < FALL_HEIGHT_M {
            return Ok((0.0, step + 1));
        }
        if step + 1 == half {
            halfway_x = record.base_position_m[0];
        } else if step + 1 == SPEED_CHECK_STEPS {
            let seconds = (SPEED_CHECK_STEPS - half) as f64 * 0.02;
            return Ok((
                (record.base_position_m[0] - halfway_x) / seconds,
                SPEED_CHECK_STEPS,
            ));
        }
    }
    unreachable!("the loop returns at its last step")
}

fn walk_step(walker: &mut UnitreeRlGymWalker, residual: &Residual) -> Result<StepRecord> {
    let record = walker.step(COMMAND)?;
    walker.offset_targets_rad(&residual.offsets(&record.observation))?;
    Ok(record)
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
    /// Extra weight on the mean squared offset while walking undisturbed
    /// before the push, in score per rad², so the residual leaves the plain
    /// policy's walk alone.
    pub(crate) walk_offset_penalty: f64,
    /// Weight on the squared change in undisturbed walking speed from the
    /// plain policy's, in score per (m/s)²; zero skips the speed check.
    pub(crate) speed_penalty: f64,
    pub(crate) seed: u64,
    pub(crate) model: Model,
    /// Store the mean every this many generations (and at the end); 0 never.
    pub(crate) checkpoint_every: usize,
}

impl Default for TrainConfig {
    fn default() -> Self {
        Self {
            generations: 30,
            pairs: 16,
            sigma: 0.05,
            learning_rate: 0.02,
            offset_penalty: 1.0,
            walk_offset_penalty: 0.0,
            speed_penalty: 0.0,
            seed: 2026,
            model: Model::Linear,
            checkpoint_every: 0,
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

/// Per candidate: fitness (mean score less the penalties), share of pushes
/// survived, mean squared offset while walking before the push and
/// undisturbed walking speed (if checked); and the simulated policy steps.
struct Evaluation {
    fitness: Vec<f64>,
    survival: Vec<f64>,
    walk_square: Vec<f64>,
    speed: Vec<f64>,
    steps: usize,
}

enum Job {
    Push(Outcome),
    Speed(f64, usize),
}

/// What every candidate walks on.
struct Setup<'a> {
    robot: Robot,
    plant: PlantConfig,
    policy: &'a LstmPolicy,
}

fn evaluate(
    setup: &Setup,
    candidates: &[Residual],
    scenarios: &[Scenario],
    config: &TrainConfig,
    reference_speed: f64,
    threads: usize,
) -> Result<Evaluation> {
    // Each candidate's scenarios, then its speed check if there is one.
    let per_candidate = scenarios.len() + usize::from(config.speed_penalty > 0.0);
    let results = run_parallel_with(
        candidates.len() * per_candidate,
        threads,
        || walk::walker(setup.robot, setup.policy.clone(), setup.plant),
        |walker, index| {
            let (candidate, job) = (index / per_candidate, index % per_candidate);
            Ok(match scenarios.get(job) {
                Some(&scenario) => {
                    Job::Push(run_scenario(walker, &candidates[candidate], scenario)?)
                }
                None => {
                    let (speed, steps) = walk_speed(walker, &candidates[candidate])?;
                    Job::Speed(speed, steps)
                }
            })
        },
    )?;
    let n = scenarios.len() as f64;
    let mut evaluation = Evaluation {
        fitness: vec![0.0; candidates.len()],
        survival: vec![0.0; candidates.len()],
        walk_square: vec![0.0; candidates.len()],
        speed: vec![f64::NAN; candidates.len()],
        steps: 0,
    };
    for (index, job) in results.into_iter().enumerate() {
        let candidate = index / per_candidate;
        match job {
            Job::Push(outcome) => {
                evaluation.fitness[candidate] += (outcome.score
                    - config.offset_penalty * outcome.offset_square
                    - config.walk_offset_penalty * outcome.walk_offset_square)
                    / n;
                evaluation.survival[candidate] += f64::from(u8::from(outcome.score >= 1.0)) / n;
                evaluation.walk_square[candidate] += outcome.walk_offset_square / n;
                evaluation.steps += outcome.steps;
            }
            Job::Speed(speed, steps) => {
                evaluation.fitness[candidate] -=
                    config.speed_penalty * (speed - reference_speed).powi(2);
                evaluation.speed[candidate] = speed;
                evaluation.steps += steps;
            }
        }
    }
    Ok(evaluation)
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
    let policy = policy::load(robot)?;
    let joints = walk::walker(robot, policy.clone(), plant)?
        .effort_limits_nm()
        .len();
    let scenarios = scenarios(robot);
    let mut normal = Normal(config.seed);
    let template = Residual::initial(config.model, joints, &mut normal);
    let dimension = template.params.len();
    let mut theta = template.params.clone();
    let (mut m, mut v) = (vec![0.0; dimension], vec![0.0; dimension]);
    let (beta1, beta2) = (0.9, 0.999);
    // The plain policy's undisturbed speed, which candidates should keep.
    let mut reference_speed = f64::NAN;
    let mut simulated_steps = 0;
    if config.speed_penalty > 0.0 {
        let mut walker = walk::walker(robot, policy.clone(), plant)?;
        let plain = template.with_params(vec![0.0; dimension]);
        let (speed, steps) = walk_speed(&mut walker, &plain)?;
        reference_speed = speed;
        simulated_steps += steps;
    }
    let begin = Instant::now();
    let mut log = Vec::new();
    let mut checkpoints = Vec::new();
    for generation in 0..=config.generations {
        let noise: Vec<Vec<f64>> = (0..config.pairs)
            .map(|_| (0..dimension).map(|_| normal.sample()).collect())
            .collect();
        let mut candidates = vec![template.with_params(theta.clone())];
        if generation < config.generations {
            for epsilon in &noise {
                for sign in [1.0, -1.0] {
                    candidates.push(
                        template.with_params(
                            theta
                                .iter()
                                .zip(epsilon)
                                .map(|(t, e)| t + sign * config.sigma * e)
                                .collect(),
                        ),
                    );
                }
            }
        }
        let Evaluation {
            fitness,
            survival,
            walk_square,
            speed,
            steps,
        } = evaluate(
            &Setup {
                robot,
                plant,
                policy: &policy,
            },
            &candidates,
            &scenarios,
            &config,
            reference_speed,
            threads,
        )?;
        simulated_steps += steps;
        let entry = json!({
            "generation": generation,
            "fitness": fitness[0],
            "survived_share": survival[0],
            "population_mean_fitness": fitness.iter().sum::<f64>() / fitness.len() as f64,
            "walk_rms_offset_rad": walk_square[0].sqrt(),
            "walk_speed_m_s": speed[0].is_finite().then_some(speed[0]),
            "wall_s": begin.elapsed().as_secs_f64(),
            "simulated_s": simulated_steps as f64 * 0.02,
        });
        println!("{entry}");
        if config.checkpoint_every > 0
            && (generation % config.checkpoint_every == 0 || generation == config.generations)
        {
            checkpoints.push(json!({
                "generation": generation,
                "wall_s": entry["wall_s"],
                "simulated_s": entry["simulated_s"],
                "residual": template.with_params(theta.clone()).to_json(),
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
    let residual = template.with_params(theta);
    let report = json!({
        "schema_version": 1,
        "robot": robot.name(),
        "self_collision": plant.self_collision,
        "contact_softness": walk::plant_json(&plant)["contact_softness"],
        "solver_iterations": plant.solver_iterations,
        "residual": residual.to_json(),
        "training": {
            "method": "evolution strategies, antithetic pairs, centred ranks, Adam",
            "parameters": dimension,
            "initialization": match config.model {
                Model::Linear => "all weights zero",
                Model::Mlp { .. } => "w1 from the seed's first normal deviates over sqrt(inputs), b1 and w2 zero",
            },
            "generations": config.generations,
            "pairs": config.pairs,
            "sigma": config.sigma,
            "learning_rate": config.learning_rate,
            "offset_penalty_per_rad2": config.offset_penalty,
            "walk_offset_penalty_per_rad2": config.walk_offset_penalty,
            "speed_penalty_per_m2_s2": config.speed_penalty,
            "reference_speed_m_s": reference_speed.is_finite().then_some(reference_speed),
            "speed_check_s": SPEED_CHECK_STEPS as f64 * 0.02,
            "seed": config.seed,
            "threads": threads,
            "scenarios": scenarios.iter().map(|s| json!({
                "direction": DIRECTIONS[s.direction].0, "force_n": s.force_n, "phase": s.phase,
            })).collect::<Vec<_>>(),
            "tilt_reference_rad": TILT_REF_RAD,
            "walk_before_push_s": TRAIN_WALK_STEPS as f64 * 0.02,
            "recovery_s": TRAIN_RECOVERY_STEPS as f64 * 0.02,
            "log": log,
            "checkpoint_every": config.checkpoint_every,
            "checkpoints": checkpoints,
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
        residual.params[0] = 1.0; // joint 0 from angular velocity x
        residual.params[FEATURES + 3] = -2.0; // joint 1 from gravity x
        let observation = [0.1_f32, 0.0, 0.0, 0.5, 0.0, 9.0];
        let offsets = residual.offsets(&observation);
        assert!((offsets[0] - f64::from(0.1_f32)).abs() < 1e-12);
        assert_eq!(offsets[1], -LIMIT_RAD);
        let again = Residual::from_json(&residual.to_json()).unwrap();
        assert_eq!(again, residual);
        assert_eq!(again.joints(), 2);
    }

    #[test]
    fn an_mlp_starts_at_zero_reads_joint_state_and_round_trips() {
        let model = Model::parse("mlp", 8).unwrap();
        let mut normal = Normal(7);
        let mut residual = Residual::initial(model, 12, &mut normal);
        // Five tilt and spin entries, 12 joint positions, 12 velocities, phase.
        assert_eq!(residual.inputs.len(), 31);
        assert_eq!(residual.params.len(), 8 * 31 + 8 + 12 * 8);
        let observation: Vec<f32> = (0..47).map(|i| (i as f32 * 0.37).sin()).collect();
        assert!(residual.offsets(&observation).iter().all(|&o| o == 0.0));
        let n = residual.params.len();
        for (i, p) in residual.params[n - 12 * 8..].iter_mut().enumerate() {
            *p = (i as f64 * 0.7).cos();
        }
        let offsets = residual.offsets(&observation);
        assert!(offsets.iter().any(|&o| o != 0.0));
        assert!(offsets.iter().all(|o| o.abs() < LIMIT_RAD));
        let again = Residual::from_json(&residual.to_json()).unwrap();
        assert_eq!(again, residual);
        assert_eq!(again.offsets(&observation), offsets);
        assert!(Model::parse("mlp", 0).is_err() && Model::parse("tree", 4).is_err());
    }

    #[test]
    fn the_mlp_matches_mujoco_robustness_py() {
        // Offsets computed by mujoco_robustness.py's Residual for this MLP.
        let fixture: Value = serde_json::from_str(include_str!("fixtures/mlp_residual_check.json"))
            .expect("fixture parses");
        let residual = Residual::from_json(&fixture["residual"]).unwrap();
        let observation: Vec<f32> = fixture["observation"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap() as f32)
            .collect();
        let offsets = residual.offsets(&observation);
        let expected = fixture["offsets_rad"].as_array().unwrap();
        assert_eq!(offsets.len(), expected.len());
        for (offset, expected) in offsets.iter().zip(expected) {
            assert!((offset - expected.as_f64().unwrap()).abs() < 1e-12);
        }
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
