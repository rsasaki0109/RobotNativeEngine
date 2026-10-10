//! Records dynamic Go2 and G1 responses and renders a visual human kick.
//!
//! The human is an animated mesh. Robot disturbances are prescribed body
//! wrenches, not simulated human-foot contacts. The panels use the same
//! impulse and declared URDF inertials, with bounded sensor-feedback control.

mod disturbance;
mod g1_controller;
mod go2_controller;
mod model;
mod physics;
mod render;

use rne_render::{load_gltf_scene, AnimationProperty, GltfAnimationPlayer};
use std::{error::Error, path::PathBuf};

const HUMAN_ASSET: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../assets/fixtures/kick_human/cc0_sport_human.glb"
);

fn validate_human() -> Result<(), Box<dyn Error>> {
    let asset = load_gltf_scene(std::path::Path::new(HUMAN_ASSET))?;
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
                time_s: frame as f32 / 120.0,
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

fn main() -> Result<(), Box<dyn Error>> {
    let arguments: Vec<_> = std::env::args().skip(1).collect();
    let mut output = PathBuf::from("target/rne-kick-comparison");
    let mut render = false;
    let mut validate_recovery = false;
    let mut validation_robot: Option<String> = None;
    let mut render_only = false;
    let mut start_frame = 0;
    let mut frame_count = usize::MAX;
    let mut index = 0;
    while index < arguments.len() {
        match arguments[index].as_str() {
            "--headless" => {}
            "--validate-recovery" => validate_recovery = true,
            "--render" => render = true,
            "--render-only" => {
                render = true;
                render_only = true;
            }
            "--output" | "--start-frame" | "--frame-count" | "--validation-robot" => {
                let option = &arguments[index];
                index += 1;
                let value = arguments.get(index).ok_or("missing option value")?;
                match option.as_str() {
                    "--output" => output = value.into(),
                    "--start-frame" => start_frame = value.parse()?,
                    "--validation-robot" => validation_robot = Some(value.clone()),
                    _ => frame_count = value.parse()?,
                }
            }
            unknown => return Err(format!("unknown option: {unknown}").into()),
        }
        index += 1;
    }
    if validation_robot.is_some() && !validate_recovery {
        return Err("--validation-robot requires --validate-recovery".into());
    }
    validate_human()?;
    if !render_only {
        if validate_recovery {
            physics::validate_recovery(&output, validation_robot.as_deref())?;
        } else {
            physics::capture(&output)?;
        }
    }
    if render {
        render::render(
            &output,
            std::path::Path::new(HUMAN_ASSET),
            start_frame,
            frame_count,
        )?;
    }
    println!("kick comparison ok: {}", output.display());
    Ok(())
}
