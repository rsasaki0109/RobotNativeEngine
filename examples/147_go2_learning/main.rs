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
//! policy trained in the other (`--evaluate`).
//!
//! ```text
//! cargo run --release -p go2_learning --example 147_go2_learning -- --train --seed 2026
//! cargo run --release -p go2_learning --example 147_go2_learning -- --evaluate POLICY.json
//! ```

mod es;
mod go2;
mod native;
mod policy;

use std::path::PathBuf;

use serde_json::{json, Value};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error + Send + Sync>>;

/// A policy file's policy: a training report's final policy, the checkpoint
/// at `generation`, or a bare policy.
fn load_policy(path: &PathBuf, generation: Option<usize>) -> Result<policy::Mlp> {
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
        None => report,
    };
    policy::Mlp::from_json(&value)
}

fn main() -> Result<()> {
    let mut plant = go2::Plant::default();
    let mut config = es::Config::default();
    let mut threads = std::thread::available_parallelism().map_or(1, |n| n.get());
    let mut output = PathBuf::from("target/go2-learning");
    let (mut train, mut evaluate, mut generation) = (false, None, None);
    let (mut bench, mut stand) = (None, None);
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
            "--generations" => config.generations = value()?.parse()?,
            "--pairs" => config.pairs = value()?.parse()?,
            "--sigma" => config.sigma = value()?.parse()?,
            "--learning-rate" => config.learning_rate = value()?.parse()?,
            "--hidden" => config.hidden = value()?.parse()?,
            "--seed" => config.seed = value()?.parse()?,
            "--checkpoint-every" => config.checkpoint_every = value()?.parse()?,
            other => return Err(format!("unknown option {other}").into()),
        }
    }
    if let Some(seconds) = stand {
        // Settle as an episode does, then hold the default targets.
        let mut robot = go2::robot(plant)?;
        go2::settle(robot.as_mut())?;
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
                   "joint_rad": joints.iter().map(|(q, _)| *q).collect::<Vec<_>>()})
        );
        return Ok(());
    }
    if let Some(episodes) = bench {
        // One thread, one robot: the initial policy's episodes.
        let mut normal = es::Normal(config.seed);
        let policy = policy::Mlp::initial(config.hidden, || normal.sample());
        let mut robot = go2::robot(plant)?;
        let begin = std::time::Instant::now();
        let mut steps = 0;
        for _ in 0..episodes {
            steps +=
                go2::episode(robot.as_mut(), |observation| policy.act(observation))?.control_steps;
        }
        let wall_s = begin.elapsed().as_secs_f64();
        let simulated_s = steps as f64 * go2::CONTROL_DT_S;
        println!(
            "{}",
            json!({"engine": plant.backend.name(), "solver_iterations": plant.solver_iterations,
                   "inner_passes": plant.inner_passes,
                   "episodes": episodes, "simulated_s": simulated_s, "wall_s": wall_s,
                   "simulated_s_per_wall_s": simulated_s / wall_s})
        );
        return Ok(());
    }
    if let Some(path) = evaluate {
        let policy = load_policy(&path, generation)?;
        let mut robot = go2::robot(plant)?;
        let outcome = go2::episode(robot.as_mut(), |observation| policy.act(observation))?;
        println!(
            "{}",
            json!({"engine": plant.backend.name(), "policy": path, "generation": generation,
                   "outcome": es::outcome_json(&outcome)})
        );
        return Ok(());
    }
    if !train {
        return Err("pass --train or --evaluate POLICY.json".into());
    }
    let report = es::train(plant, threads, config)?;
    std::fs::create_dir_all(&output)?;
    let engine = match plant.backend {
        go2::Backend::Rapier => "rne",
        go2::Backend::Native => "native",
    };
    let path = output.join(format!("go2-learning-{engine}-{}.json", config.seed));
    std::fs::write(&path, serde_json::to_string_pretty(&report)? + "\n")?;
    println!("wrote {}", path.display());
    Ok(())
}
