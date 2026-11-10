//! A Go2 learns to walk from scratch in RNE and in MuJoCo, with the same
//! algorithm, so that the two engines compare as places to learn.
//!
//! `--train` runs evolution strategies on a 32-unit MLP that maps the
//! robot's proprioception and a gait clock to joint targets around its
//! default pose, tracked by unitree_rl_gym's PD gains at 50 Hz. Every
//! candidate walks one 5 s episode after standing for 1 s; its fitness is the
//! base's forward travel less penalties for sideways travel, heading, a fall
//! and jerky actions. `mujoco_go2_learning.py` trains the same way in MuJoCo
//! from this report's settings and seed, and either engine evaluates a
//! policy trained in the other (`--evaluate`). `--backend locomotion` trains
//! on `rne_locomotion`'s hard-contact step instead of Rapier. On its rough
//! terrain, `--random-starts` starts every generation at its own point and
//! `--height-scan` lets the policy see the ground around it; `--evaluate
//! POLICY.json --starts N` walks from N held-out points and reports the mean.
//!
//! ```text
//! cargo run --release -p go2_learning --example 147_go2_learning -- --train --seed 2026
//! cargo run --release -p go2_learning --example 147_go2_learning -- --evaluate POLICY.json
//! ```

mod es;
mod go2;
mod locomotion;
mod native;
mod policy;
mod ppo;

use std::path::PathBuf;

use serde_json::{json, Value};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error + Send + Sync>>;

/// A policy file's policy: a training report's final policy, the checkpoint
/// at `generation`, or a bare policy, and the task its inputs fit.
fn load_policy(path: &PathBuf, generation: Option<usize>) -> Result<(policy::Mlp, go2::Task)> {
    let report: Value = serde_json::from_str(&std::fs::read_to_string(path)?)?;
    let value = match generation {
        Some(generation) => report["training"]["checkpoints"]
            .as_array()
            .and_then(|checkpoints| {
                checkpoints
                    .iter()
                    .find(|c| c["generation"].as_u64() == Some(generation as u64))
            })
            .map(|checkpoint| checkpoint["policy"].clone())
            .ok_or_else(|| format!("no checkpoint at generation {generation}"))?,
        None if report.get("policy").is_some() => report["policy"].clone(),
        None => report.clone(),
    };
    let policy = policy::Mlp::from_json(&value)?;
    let task = go2::Task {
        height_scan: policy.inputs > go2::PROPRIOCEPTION,
        random_starts: go2::Task::from_json(&report["task"]).random_starts,
    };
    if policy.inputs != task.observations() {
        return Err(format!("a policy with {} inputs fits no task", policy.inputs).into());
    }
    Ok((policy, task))
}

/// The mean of `outcomes`: a fall in any of them counts as a fall.
fn mean_outcome(outcomes: &[go2::Outcome]) -> go2::Outcome {
    let mean = |value: fn(&go2::Outcome) -> f64| {
        outcomes.iter().map(value).sum::<f64>() / outcomes.len() as f64
    };
    go2::Outcome {
        fitness: mean(|o| o.fitness),
        forward_m: mean(|o| o.forward_m),
        lateral_m: mean(|o| o.lateral_m),
        heading_rad: mean(|o| o.heading_rad),
        survived_s: mean(|o| o.survived_s),
        fell: outcomes.iter().any(|o| o.fell),
        control_steps: outcomes.iter().map(|o| o.control_steps).sum(),
    }
}

/// `--evaluate`: one episode of the policy at `path` from the origin, or the
/// mean over `starts` held-out starts, printed as JSON.
fn evaluate_policy(
    plant: go2::Plant,
    path: &PathBuf,
    generation: Option<usize>,
    starts: Option<usize>,
) -> Result<()> {
    let (policy, task) = load_policy(path, generation)?;
    let mut robot = go2::robot(plant)?;
    let points = starts.map_or(vec![[0.0, 0.0]], go2::evaluation_starts);
    let mut outcomes = Vec::new();
    for start in &points {
        outcomes.push(go2::episode(robot.as_mut(), task, *start, |observation| {
            policy.act(observation)
        })?);
    }
    let mut outcome = es::outcome_json(&mean_outcome(&outcomes));
    let mut report = json!({"engine": plant.backend.name(), "policy": path,
                            "generation": generation, "task": task.to_json()});
    if starts.is_some() {
        outcome["falls"] = json!(outcomes.iter().filter(|o| o.fell).count());
        report["episodes"] = json!(points
            .iter()
            .zip(&outcomes)
            .map(|(start, o)| json!({"start_m": start, "outcome": es::outcome_json(o)}))
            .collect::<Vec<_>>());
    }
    report["outcome"] = outcome;
    println!("{report}");
    Ok(())
}

/// A training report's file tag: the backend and floor, then the task and a
/// non-default learner.
fn report_tag(plant: go2::Plant, task: go2::Task, learner: &str) -> String {
    let mut tag = match plant.backend {
        go2::Backend::Rapier => "rne",
        go2::Backend::Native => "native",
        go2::Backend::Locomotion if plant.terrain_seed.is_some() => "locomotion-terrain",
        go2::Backend::Locomotion => "locomotion",
    }
    .to_string();
    if task.random_starts {
        tag += "-starts";
    }
    if task.height_scan {
        tag += "-scan";
    }
    if learner != "es" {
        tag += &format!("-{learner}");
    }
    tag
}

fn main() -> Result<()> {
    let mut plant = go2::Plant::default();
    let mut config = es::Config::default();
    let mut ppo_config = ppo::Config::default();
    let mut learner = "es".to_string();
    let mut hidden = None;
    let mut threads = std::thread::available_parallelism().map_or(1, |n| n.get());
    let mut output = PathBuf::from("target/go2-learning");
    let (mut train, mut evaluate, mut generation) = (false, None, None);
    let (mut bench, mut stand) = (None, None);
    let mut write_terrain = None;
    let mut task = go2::Task::default();
    let mut starts = None;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = || args.next().ok_or_else(|| format!("{arg} needs a value"));
        match arg.as_str() {
            "--train" => train = true,
            "--evaluate" => evaluate = Some(PathBuf::from(value()?)),
            "--bench" => bench = Some(value()?.parse::<usize>()?),
            "--stand" => stand = Some(value()?.parse::<f64>()?),
            "--generation" => generation = Some(value()?.parse()?),
            "--output" => output = PathBuf::from(value()?),
            "--threads" => threads = value()?.parse()?,
            "--solver-iterations" => plant.solver_iterations = value()?.parse()?,
            "--backend" => plant.backend = go2::Backend::parse(&value()?)?,
            "--inner-passes" => plant.inner_passes = value()?.parse()?,
            "--friction" => plant.floor_friction = value()?.parse()?,
            "--terrain" => plant.terrain_seed = Some(value()?.parse()?),
            "--terrain-frequency" => plant.relief.frequency_per_m = value()?.parse()?,
            "--terrain-amplitude" => plant.relief.amplitude_m = value()?.parse()?,
            "--terrain-size" => plant.relief.size_m = value()?.parse()?,
            "--write-terrain" => write_terrain = Some(PathBuf::from(value()?)),
            "--height-scan" => task.height_scan = true,
            "--random-starts" => task.random_starts = true,
            "--starts" => starts = Some(value()?.parse::<usize>()?),
            "--generations" => config.generations = value()?.parse()?,
            "--pairs" => config.pairs = value()?.parse()?,
            "--sigma" => config.sigma = value()?.parse()?,
            "--learning-rate" => config.learning_rate = value()?.parse()?,
            "--hidden" => hidden = Some(value()?.parse()?),
            "--seed" => {
                config.seed = value()?.parse()?;
                ppo_config.seed = config.seed;
            }
            "--learner" => learner = value()?,
            "--iterations" => ppo_config.iterations = value()?.parse()?,
            "--envs" => ppo_config.envs = value()?.parse()?,
            "--checkpoint-every" => config.checkpoint_every = value()?.parse()?,
            other => return Err(format!("unknown option {other}").into()),
        }
    }
    if let Some(hidden) = hidden {
        config.hidden = hidden;
        ppo_config.hidden = hidden;
    }
    if let Some(path) = write_terrain {
        let seed = plant
            .terrain_seed
            .ok_or("--write-terrain needs --terrain SEED")?;
        std::fs::write(
            &path,
            serde_json::to_string(&go2::terrain_json(seed, plant.relief)?)? + "\n",
        )?;
        println!("wrote {}", path.display());
        return Ok(());
    }
    if let Some(seconds) = stand {
        // Settle as an episode does, then hold the default targets.
        let mut robot = go2::robot(plant)?;
        go2::settle(robot.as_mut(), [0.0, 0.0])?;
        for _ in 0..(seconds / go2::CONTROL_DT_S).round() as usize {
            robot.step()?;
        }
        let ((position, rotation), (velocity, _)) =
            (robot.base_pose_z_up(), robot.base_velocity_z_up());
        let up = rotation * rne_math::Vec3::Z;
        let forward = rotation * rne_math::Vec3::X;
        let joints = robot.joint_states()?;
        println!(
            "{}",
            json!({"engine": plant.backend.name(), "plant": {
                       "solver_iterations": plant.solver_iterations,
                       "inner_passes": plant.inner_passes,
                       "floor_friction": plant.floor_friction},
                   "seconds_after_settling": seconds,
                   "base_m": position.to_array(), "base_speed_m_s": velocity.length(),
                   "base_quat_wxyz": [rotation.w, rotation.x, rotation.y, rotation.z],
                   "pitch_rad": forward.z.asin(), "roll_rad": (rotation * rne_math::Vec3::Y).z.asin(),
                   "tilt_rad": up.z.clamp(-1.0, 1.0).acos(),
                   "mass_kg": match plant.backend {
                       go2::Backend::Rapier => Some(go2::rapier_mass_kg(plant)?),
                       go2::Backend::Native => None,
                       go2::Backend::Locomotion => Some(
                           locomotion::LocomotionGo2::new(
                               plant.floor_friction, plant.terrain_seed, plant.relief)?
                               .mass_kg(),
                       ),
                   },
                   "joint_rad": joints.iter().map(|(q, _)| *q).collect::<Vec<_>>()})
        );
        return Ok(());
    }
    if let Some(episodes) = bench {
        // One thread, one robot: the initial policy's episodes, each from its
        // own start with random starts.
        let mut normal = es::Normal(config.seed);
        let policy = policy::Mlp::initial(task.observations(), config.hidden, || normal.sample());
        let starts = go2::training_starts(config.seed, episodes);
        let mut robot = go2::robot(plant)?;
        let begin = std::time::Instant::now();
        let mut steps = 0;
        for start in &starts[..episodes] {
            let start = if task.random_starts {
                *start
            } else {
                [0.0, 0.0]
            };
            steps += go2::episode(robot.as_mut(), task, start, |observation| {
                policy.act(observation)
            })?
            .control_steps;
        }
        let wall_s = begin.elapsed().as_secs_f64();
        let simulated_s = steps as f64 * go2::CONTROL_DT_S;
        println!(
            "{}",
            json!({"engine": plant.backend.name(), "solver_iterations": plant.solver_iterations,
                   "inner_passes": plant.inner_passes, "task": task.to_json(),
                   "episodes": episodes, "simulated_s": simulated_s, "wall_s": wall_s,
                   "simulated_s_per_wall_s": simulated_s / wall_s})
        );
        return Ok(());
    }
    if let Some(path) = evaluate {
        return evaluate_policy(plant, &path, generation, starts);
    }
    if !train {
        return Err("pass --train or --evaluate POLICY.json".into());
    }
    let report = match learner.as_str() {
        "es" => es::train(plant, task, threads, config)?,
        "ppo" => ppo::train(plant, task, threads, ppo_config)?,
        other => return Err(format!("unknown learner {other} (es or ppo)").into()),
    };
    let path = output.join(format!(
        "go2-learning-{}-{}.json",
        report_tag(plant, task, &learner),
        config.seed
    ));
    std::fs::create_dir_all(&output)?;
    std::fs::write(&path, serde_json::to_string_pretty(&report)? + "\n")?;
    println!("wrote {}", path.display());
    Ok(())
}
