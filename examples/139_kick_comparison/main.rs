//! Records dynamic Go2 and G1 responses and renders a visual human kick.
//!
//! The human is an animated mesh. Robot disturbances are prescribed body
//! wrenches, not simulated human-foot contacts. The panels use the same
//! impulse and declared URDF inertials, with bounded sensor-feedback control.

#![recursion_limit = "256"]

mod diagnostic;
mod disturbance;
mod feedback;
mod g1_controller;
mod go2_controller;
mod model;
mod observation;
mod physics;
mod render;

use rne_render::{load_gltf_scene, AnimationProperty, GltfAnimationPlayer};
use std::{
    error::Error,
    path::{Path, PathBuf},
};

const HUMAN_ASSET: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../assets/fixtures/kick_human/cc0_sport_human.glb"
);

fn validate_human() -> Result<(), Box<dyn Error>> {
    let asset = load_gltf_scene(Path::new(HUMAN_ASSET))?;
    for name in ["low_kick", "mid_kick"] {
        let (index, clip) = asset
            .animations
            .iter()
            .enumerate()
            .find(|(_, clip)| {
                clip.name
                    .as_deref()
                    .is_some_and(|value| value.contains(name))
            })
            .ok_or_else(|| format!("missing human clip {name}"))?;
        for channel in &clip.channels {
            let node_name = asset.nodes[channel.node_index].name.as_deref();
            if channel.property == AnimationProperty::Translation && node_name != Some("Root") {
                let first = channel
                    .values
                    .first()
                    .ok_or("empty human translation channel")?;
                if channel.values.iter().any(|value| {
                    value[..3]
                        .iter()
                        .zip(&first[..3])
                        .any(|(a, b)| (a - b).abs() > 1.0e-6)
                }) {
                    return Err(format!("translated limb joint in {name}: {node_name:?}").into());
                }
            }
            if channel.property == AnimationProperty::Scale
                && channel.values.iter().any(|value| {
                    value[..3]
                        .iter()
                        // Blender's exported matrix decomposition introduces
                        // a few f32 ULPs even with unit pose-bone scales.
                        .any(|component| (component - 1.0).abs() > 5.0e-6)
                })
            {
                return Err(format!("scaled limb in {name}: {node_name:?}").into());
            }
            if channel.property == AnimationProperty::Rotation {
                for values in channel.values.windows(2) {
                    let dot: f32 = values[0].iter().zip(values[1]).map(|(a, b)| a * b).sum();
                    if dot < 0.0 {
                        return Err(format!("discontinuous quaternion sign in {name}").into());
                    }
                }
            }
        }
        // Include subframes, so the check exercises interpolation as well as keys.
        for frame in 0..=720 {
            let player = GltfAnimationPlayer {
                animation_index: Some(index),
                time_s: render::human_animation_time_s(f64::from(frame) / 120.0) as f32,
                playback_rate: 1.0,
            };
            for part in 0..asset.parts.len() {
                let mesh = player.sample_part_for_gpu(&asset, part)?;
                if let Some(skin) = mesh.skinning {
                    if !skin.mesh_transform.is_finite()
                        || skin.joint_matrices.iter().any(|matrix| !matrix.is_finite())
                    {
                        return Err(format!("non-finite human pose in {name}").into());
                    }
                }
            }
        }
    }
    println!("human animation: two clips, keys and 120 Hz subframes ok");
    Ok(())
}

fn validate_impulse(impulse_ns: f64, impulse_probe: bool) -> Result<(), Box<dyn Error>> {
    if !impulse_ns.is_finite() || impulse_ns < 0.0 || (impulse_ns == 0.0 && !impulse_probe) {
        return Err(
            "--impulse-ns must be finite and positive (zero is allowed for a probe)".into(),
        );
    }
    Ok(())
}

fn report_completion(output: &Path, g1_observation_diagnosis: bool, observation_probe: bool) {
    let message = if g1_observation_diagnosis {
        "G1 observation diagnosis completed; measured outcomes"
    } else if observation_probe {
        "observation probe completed; measured recovery outcomes"
    } else {
        "kick comparison ok"
    };
    println!("{message}: {}", output.display());
}

fn validate_g1_study_mode(
    diagnosis: bool,
    experiment: bool,
    incompatible: bool,
    policy: Option<&str>,
    input: Option<&str>,
) -> Result<(), Box<dyn Error>> {
    if (diagnosis || experiment) && (incompatible || (diagnosis && experiment)) {
        return Err("G1 observation studies are separate fixed headless modes".into());
    }
    if policy.is_some() && !experiment {
        return Err("--feedback-policy requires --g1-orientation-age-experiment".into());
    }
    if input.is_some() && !diagnosis && !experiment {
        return Err("--diagnosis-input requires a G1 observation study".into());
    }
    Ok(())
}

#[derive(Debug)]
struct HeadlessOptions<'a> {
    orientation_age_experiment: bool,
    g1_observation_diagnosis: bool,
    observation_probe: bool,
    impulse_sweep: bool,
    impulse_probe: bool,
    validate_recovery: bool,
    balance_off: bool,
    impulse_ns: f64,
    validation_robot: Option<&'a str>,
    feedback_policy: Option<&'a str>,
    observation_profile: Option<&'a str>,
    diagnosis_input: Option<&'a str>,
}

fn run_headless(output: &Path, options: HeadlessOptions<'_>) -> Result<(), Box<dyn Error>> {
    if options.orientation_age_experiment {
        physics::experiment_g1_orientation_age(
            output,
            options.feedback_policy,
            options.observation_profile,
            options.diagnosis_input,
        )?;
    } else if options.g1_observation_diagnosis {
        physics::diagnose_g1_observations(
            output,
            options.observation_profile,
            options.diagnosis_input,
        )?;
    } else if options.observation_probe {
        physics::probe_observations(
            output,
            options.validation_robot,
            options.observation_profile,
        )?;
    } else if options.impulse_sweep {
        physics::sweep_impulse(output, options.validation_robot)?;
    } else if options.impulse_probe {
        physics::probe_impulse(
            output,
            options.validation_robot,
            options.impulse_ns,
            !options.balance_off,
        )?;
    } else if options.validate_recovery {
        physics::validate_recovery(output, options.validation_robot, options.impulse_ns)?;
    } else {
        physics::capture(output, options.impulse_ns)?;
    }
    Ok(())
}

fn validate_impulse_modes(
    sweep: bool,
    validation: bool,
    render: bool,
    explicit_dose: bool,
    probe: bool,
    balance_off: bool,
) -> Result<(), Box<dyn Error>> {
    if sweep && (validation || render || explicit_dose || probe) {
        return Err(
            "--impulse-sweep cannot be combined with rendering, validation or --impulse-ns".into(),
        );
    }
    if probe && (validation || render) {
        return Err("--impulse-probe cannot be combined with rendering or validation".into());
    }
    if balance_off && !probe {
        return Err("--balance-off requires --impulse-probe".into());
    }
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let arguments: Vec<_> = std::env::args().skip(1).collect();
    let mut output = PathBuf::from("target/rne-kick-comparison");
    let mut render = false;
    let mut validate_recovery = false;
    let mut impulse_sweep = false;
    let mut impulse_probe = false;
    let mut observation_probe = false;
    let mut g1_observation_diagnosis = false;
    let mut orientation_age_experiment = false;
    let mut feedback_policy: Option<String> = None;
    let mut diagnosis_input: Option<String> = None;
    let mut observation_profile: Option<String> = None;
    let mut balance_off = false;
    let mut impulse_ns = physics::DEFAULT_IMPULSE_NS;
    let mut impulse_requested = false;
    let mut validation_robot: Option<String> = None;
    let mut render_only = false;
    let mut start_frame = 0;
    let mut frame_count = usize::MAX;
    let mut render_frame_options_requested = false;
    let mut index = 0;
    while index < arguments.len() {
        match arguments[index].as_str() {
            "--headless" => {}
            "--validate-recovery" => validate_recovery = true,
            "--impulse-sweep" => impulse_sweep = true,
            "--impulse-probe" => impulse_probe = true,
            "--observation-probe" => observation_probe = true,
            "--g1-observation-diagnosis" => g1_observation_diagnosis = true,
            "--g1-orientation-age-experiment" => orientation_age_experiment = true,
            "--balance-off" => balance_off = true,
            "--render" => render = true,
            "--render-only" => {
                render = true;
                render_only = true;
            }
            "--output"
            | "--start-frame"
            | "--frame-count"
            | "--validation-robot"
            | "--impulse-ns"
            | "--observation-profile"
            | "--feedback-policy"
            | "--diagnosis-input" => {
                let option = &arguments[index];
                render_frame_options_requested |=
                    matches!(option.as_str(), "--start-frame" | "--frame-count");
                index += 1;
                let value = arguments.get(index).ok_or("missing option value")?;
                match option.as_str() {
                    "--output" => output = value.into(),
                    "--start-frame" => start_frame = value.parse()?,
                    "--validation-robot" => validation_robot = Some(value.clone()),
                    "--observation-profile" => observation_profile = Some(value.clone()),
                    "--diagnosis-input" => diagnosis_input = Some(value.clone()),
                    "--feedback-policy" => feedback_policy = Some(value.clone()),
                    "--impulse-ns" => {
                        impulse_ns = value.parse()?;
                        impulse_requested = true;
                    }
                    _ => frame_count = value.parse()?,
                }
            }
            unknown => return Err(format!("unknown option: {unknown}").into()),
        }
        index += 1;
    }
    validate_impulse(impulse_ns, impulse_probe)?;
    let incompatible_study_mode = observation_probe
        || validate_recovery
        || impulse_sweep
        || impulse_probe
        || render
        || balance_off
        || impulse_requested
        || render_frame_options_requested
        || validation_robot.is_some();
    validate_g1_study_mode(
        g1_observation_diagnosis,
        orientation_age_experiment,
        incompatible_study_mode,
        feedback_policy.as_deref(),
        diagnosis_input.as_deref(),
    )?;
    if observation_probe
        && (validate_recovery
            || impulse_sweep
            || impulse_probe
            || render
            || balance_off
            || impulse_requested)
    {
        return Err("--observation-probe is a separate fixed 40 N.s headless diagnostic".into());
    }
    if observation_profile.is_some()
        && !observation_probe
        && !g1_observation_diagnosis
        && !orientation_age_experiment
    {
        return Err(
            "--observation-profile requires --observation-probe, --g1-observation-diagnosis or --g1-orientation-age-experiment"
                .into(),
        );
    }
    if validation_robot.is_some()
        && !validate_recovery
        && !impulse_sweep
        && !impulse_probe
        && !observation_probe
    {
        return Err("--validation-robot requires validation or an impulse probe/sweep".into());
    }
    validate_impulse_modes(
        impulse_sweep,
        validate_recovery,
        render,
        impulse_requested,
        impulse_probe,
        balance_off,
    )?;
    validate_human()?;
    if !render_only {
        run_headless(
            &output,
            HeadlessOptions {
                orientation_age_experiment,
                g1_observation_diagnosis,
                observation_probe,
                impulse_sweep,
                impulse_probe,
                validate_recovery,
                balance_off,
                impulse_ns,
                validation_robot: validation_robot.as_deref(),
                feedback_policy: feedback_policy.as_deref(),
                observation_profile: observation_profile.as_deref(),
                diagnosis_input: diagnosis_input.as_deref(),
            },
        )?;
    }
    if render {
        render::render(&output, Path::new(HUMAN_ASSET), start_frame, frame_count)?;
    }
    report_completion(
        &output,
        g1_observation_diagnosis || orientation_age_experiment,
        observation_probe,
    );
    Ok(())
}
