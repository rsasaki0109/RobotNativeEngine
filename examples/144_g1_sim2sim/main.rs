//! Sim2sim: unitree_rl_gym's pretrained G1 walking policy on RNE.
//!
//! Runs the policy trained in Isaac Gym and deployed by Unitree in MuJoCo on
//! RNE's Rapier backend with the same 500 Hz PD loop, and compares walking
//! against the MuJoCo reference recorded by `mujoco_reference.py`.
//!
//! ```text
//! cargo run --release --example 144_g1_sim2sim                 # 4 commands vs MuJoCo
//! cargo run --release --example 144_g1_sim2sim -- --timing     # solver iteration sweep
//! cargo run --release --example 144_g1_sim2sim -- --command 0.5,0,0 --seconds 20
//! cargo run --release --example 144_g1_sim2sim -- --render target/g1-sim2sim-render
//! ```

mod policy;
mod render;
mod walk;

use std::path::PathBuf;
use std::time::Instant;

use serde_json::{json, Value};

use policy::{default_manifest, LstmPolicy, Result};
use walk::{summarize, G1Walker, PlantConfig, StepRecord};

const REFERENCE: &str = include_str!("fixtures/mujoco_reference.json");

struct Options {
    output: PathBuf,
    seconds: f64,
    command: Option<[f64; 3]>,
    timing: bool,
    ablation: bool,
    render: Option<PathBuf>,
    plant: PlantConfig,
}

fn parse_command(text: &str) -> Result<[f64; 3]> {
    let values: Vec<f64> = text
        .split(',')
        .map(str::parse)
        .collect::<std::result::Result<_, _>>()?;
    let command: [f64; 3] = values
        .try_into()
        .map_err(|_| "--command takes vx,vy,yaw_rate")?;
    if command
        .iter()
        .any(|value| !value.is_finite() || value.abs() > 2.0)
    {
        return Err("--command values must be finite and within +-2".into());
    }
    Ok(command)
}

fn parse() -> Result<Options> {
    let mut options = Options {
        output: PathBuf::from("target/g1-sim2sim"),
        seconds: 10.0,
        command: None,
        timing: false,
        ablation: false,
        render: None,
        plant: PlantConfig::default(),
    };
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = || args.next().ok_or_else(|| format!("{arg} needs a value"));
        match arg.as_str() {
            "--output" => options.output = PathBuf::from(value()?),
            "--seconds" => options.seconds = value()?.parse()?,
            "--command" => options.command = Some(parse_command(&value()?)?),
            "--solver-iterations" => options.plant.solver_iterations = value()?.parse()?,
            "--armature" => options.plant.armature_kg_m2 = value()?.parse()?,
            "--floor-friction" => options.plant.floor_friction = value()?.parse()?,
            "--timing" => options.timing = true,
            "--ablation" => options.ablation = true,
            "--render" => options.render = Some(PathBuf::from(value()?)),
            other => return Err(format!("unknown option {other}").into()),
        }
    }
    if !(options.seconds.is_finite() && options.seconds > 0.0 && options.seconds <= 600.0) {
        return Err("--seconds must be in (0, 600]".into());
    }
    if !(0.0..=1.0).contains(&options.plant.armature_kg_m2)
        || !(0.0..=2.0).contains(&options.plant.floor_friction)
        || options.plant.solver_iterations == 0
    {
        return Err("armature must be 0..=1, friction 0..=2, iterations >= 1".into());
    }
    Ok(options)
}

struct Run {
    records: Vec<StepRecord>,
    wall_s: f64,
}

fn run(plant: PlantConfig, command: [f64; 3], seconds: f64) -> Result<Run> {
    let policy = LstmPolicy::load(&default_manifest())?;
    let steps = (seconds / policy.deployment.control_dt_s()).round() as usize;
    let mut walker = G1Walker::new(policy, plant)?;
    let mut records = Vec::with_capacity(steps);
    let start = Instant::now();
    for _ in 0..steps {
        records.push(walker.step(command)?);
    }
    Ok(Run {
        records,
        wall_s: start.elapsed().as_secs_f64(),
    })
}

/// Turning walk shown in the GIF: both paths trace a circle of about 0.6 m.
const RENDER_COMMAND: [f64; 3] = [0.3, 0.0, 0.5];

fn render_walk(options: &Options, output: &std::path::Path) -> Result<()> {
    let command = options.command.unwrap_or(RENDER_COMMAND);
    let mujoco =
        reference_run(command).ok_or("--render needs a command with a MuJoCo reference")?;
    let mujoco_path: Vec<[f64; 2]> = serde_json::from_value(mujoco["base_path_xy_m_10hz"].clone())?;
    let policy = LstmPolicy::load(&default_manifest())?;
    let weights_sha256 = policy.weights_sha256.clone();
    let steps = (options.seconds / policy.deployment.control_dt_s()).round() as usize;
    let mut walker = G1Walker::new(policy, options.plant)?;
    let orbit = rne_render_wgpu::CameraOrbit {
        focus: rne_math::Vec3::new(0.0, 0.35, -0.55),
        yaw_rad: 0.35,
        pitch_rad: 0.95,
        distance_m: 3.1,
    };
    let mut renderer = render::WalkRenderer::new(&walker, output, orbit)?;
    let (mut records, mut rne_path, mut frame_times_s) = (Vec::new(), Vec::new(), Vec::new());
    for step in 0..steps {
        let record = walker.step(command)?;
        if step % 5 == 4 {
            rne_path.push([record.base_position_m[0], record.base_position_m[1]]);
        }
        if step % 2 == 1 {
            let shown = mujoco_path.len().min(rne_path.len());
            renderer.frame(&walker, &rne_path, &mujoco_path[..shown])?;
            frame_times_s.push(record.time_s);
        }
        records.push(record);
    }
    let summary = summarize(&records, command);
    std::fs::write(
        output.join("render-trace.json"),
        serde_json::to_string_pretty(&json!({
            "schema_version": 1, "command": command, "seconds": options.seconds,
            "policy_weights_sha256": weights_sha256,
            "frame_count": renderer.frame_count(), "frame_times_s": frame_times_s,
            "panel": [render::WIDTH, render::HEIGHT],
            "rne": summary, "mujoco": mujoco["summary"], "mujoco_engine": serde_json::from_str::<Value>(REFERENCE)?["engine"],
        }))? + "\n",
    )?;
    println!(
        "rendered {} frames to {}",
        renderer.frame_count(),
        output.display()
    );
    Ok(())
}

fn reference_summary(command: [f64; 3]) -> Option<Value> {
    reference_run(command).map(|run| run["summary"].clone())
}

fn reference_run(command: [f64; 3]) -> Option<Value> {
    let reference: Value = serde_json::from_str(REFERENCE).ok()?;
    reference["runs"]
        .as_array()?
        .iter()
        .find(|run| {
            run["command"].as_array().is_some_and(|values| {
                values
                    .iter()
                    .zip(command)
                    .all(|(v, c)| v.as_f64() == Some(c))
            })
        })
        .cloned()
}

fn report(
    label: &str,
    plant: PlantConfig,
    command: [f64; 3],
    seconds: f64,
    records_path: Option<&std::path::Path>,
) -> Result<Value> {
    let run = run(plant, command, seconds)?;
    if let Some(path) = records_path {
        let records: Vec<Value> = run
            .records
            .iter()
            .map(|r| {
                json!({"time_s": r.time_s, "obs": r.observation.to_vec(), "action": r.action,
                            "base_position_m": r.base_position_m,
                            "base_quaternion_wxyz": r.base_quaternion_wxyz})
            })
            .collect();
        std::fs::write(
            path,
            serde_json::to_string(&json!({"command": command, "records": records}))?,
        )?;
    }
    let summary = summarize(&run.records, command);
    let simulated_s = run.records.last().map_or(0.0, |r| r.time_s);
    let realtime_factor = simulated_s / run.wall_s;
    let saturation = run
        .records
        .iter()
        .map(|r| r.saturated_joints)
        .sum::<usize>() as f64
        / (run.records.len().max(1) * policy::ACTION_LEN) as f64;
    let mujoco = reference_summary(command);
    println!(
        "{label:<28} cmd {command:?}: {} | final {:?} | v_fwd {:+.3} (MuJoCo {}) | yaw {:+.3} (MuJoCo {}) | {:.1}x real time",
        if summary["fell"] == true { "FELL" } else { "walks" },
        summary["final_base_position_m"],
        summary["second_half_forward_speed_m_s"].as_f64().unwrap_or(f64::NAN),
        mujoco.as_ref().map_or("-".into(), |m| m["second_half_forward_speed_m_s"].to_string()),
        summary["second_half_yaw_rate_rad_s"].as_f64().unwrap_or(f64::NAN),
        mujoco.as_ref().map_or("-".into(), |m| m["second_half_yaw_rate_rad_s"].to_string()),
        realtime_factor,
    );
    Ok(json!({
        "label": label, "command": command, "seconds": seconds,
        "plant": {"solver_iterations": plant.solver_iterations, "armature_kg_m2": plant.armature_kg_m2,
                  "joint_damping_nm_s_per_rad": plant.joint_damping_nm_s_per_rad,
                  "joint_friction_nm": plant.joint_friction_nm, "floor_friction": plant.floor_friction},
        "rne": summary, "mujoco": mujoco,
        "wall_s": run.wall_s, "realtime_factor": realtime_factor,
        "torque_saturated_fraction": saturation,
    }))
}

fn main() -> Result<()> {
    let options = parse()?;
    if let Some(output) = &options.render {
        return render_walk(&options, output);
    }
    std::fs::create_dir_all(&options.output)?;
    let reference: Value = serde_json::from_str(REFERENCE)?;
    let commands: Vec<[f64; 3]> = match options.command {
        Some(command) => vec![command],
        None => reference["runs"]
            .as_array()
            .ok_or("reference runs")?
            .iter()
            .map(|run| serde_json::from_value(run["command"].clone()))
            .collect::<std::result::Result<_, _>>()?,
    };
    let mut results = Vec::new();
    if options.timing {
        for iterations in [1, 2, 4, 8, 16, 32] {
            let plant = PlantConfig {
                solver_iterations: iterations,
                ..options.plant
            };
            results.push(report(
                &format!("{iterations} solver iterations"),
                plant,
                commands[0],
                options.seconds,
                None,
            )?);
        }
    } else if options.ablation {
        let base = options.plant;
        let variants = [
            ("matched plant", base),
            (
                "no armature",
                PlantConfig {
                    armature_kg_m2: 0.0,
                    ..base
                },
            ),
            (
                "no joint friction",
                PlantConfig {
                    joint_friction_nm: 0.0,
                    joint_damping_nm_s_per_rad: 0.0,
                    ..base
                },
            ),
            (
                "floor friction 0.5",
                PlantConfig {
                    floor_friction: 0.5,
                    ..base
                },
            ),
        ];
        for (label, plant) in variants {
            for command in &commands {
                results.push(report(label, plant, *command, options.seconds, None)?);
            }
        }
    } else {
        let records_path = options.output.join("g1-sim2sim-records.json");
        let single = options.command.is_some();
        for command in commands {
            results.push(report(
                "matched plant",
                options.plant,
                command,
                options.seconds,
                single.then_some(records_path.as_path()),
            )?);
        }
    }
    let path = options.output.join("g1-sim2sim-report.json");
    std::fs::write(
        &path,
        serde_json::to_string_pretty(&json!({
            "schema_version": 1,
            "backend": "RNE Rapier (UrdfSceneSim), Y-up, 2 ms PD substeps, 50 Hz policy",
            "policy": "unitreerobotics/unitree_rl_gym deploy/pre_train/g1/motion.pt",
            "policy_weights_sha256": LstmPolicy::load(&default_manifest())?.weights_sha256,
            "mujoco_engine": reference["engine"],
            "runs": results,
        }))? + "\n",
    )?;
    println!("wrote {}", path.display());
    Ok(())
}
