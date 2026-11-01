//! Runs an ONNX walking policy through tract and checks it against MuJoCo.

use std::path::Path;
use std::time::Instant;

use serde_json::{json, Value};
use tract_onnx::prelude::*;

use crate::Result;

/// Steps the recurrent policy over the recorded MuJoCo observations, compares
/// its actions with MuJoCo's, and times one step on the current executor.
pub fn evaluate(model_path: &Path, fixture_path: &Path) -> Result<Value> {
    let fixture: Value = serde_json::from_str(&std::fs::read_to_string(fixture_path)?)?;
    let steps = fixture["steps"].as_array().ok_or("fixture has no steps")?;
    let start = Instant::now();
    let model = tract_onnx::onnx()
        .model_for_path(model_path)?
        .into_optimized()?
        .into_runnable()?;
    let load_ms = start.elapsed().as_secs_f64() * 1e3;
    let values = |value: &Value| -> Result<Vec<f32>> {
        Ok(value
            .as_array()
            .ok_or("not an array")?
            .iter()
            .map(|v| v.as_f64().map(|v| v as f32).ok_or("not a number"))
            .collect::<std::result::Result<_, _>>()?)
    };
    let observations: Vec<Vec<f32>> = steps
        .iter()
        .map(|step| values(&step["obs"]))
        .collect::<Result<_>>()?;
    let hidden = 64;
    let zeros = || Tensor::zero::<f32>(&[1, 1, hidden]);
    let (mut h, mut c) = (zeros()?, zeros()?);
    let step = |observation: &[f32], h: &mut Tensor, c: &mut Tensor| -> Result<Vec<f32>> {
        let input = Tensor::from_shape(&[1, 1, observation.len()], observation)?;
        let mut outputs = model.run(tvec!(input.into(), h.clone().into(), c.clone().into()))?;
        *c = outputs.remove(2).into_tensor();
        *h = outputs.remove(1).into_tensor();
        Ok(outputs[0]
            .to_plain_array_view::<f32>()?
            .iter()
            .copied()
            .collect())
    };
    let mut worst = 0.0_f32;
    for (observation, recorded) in observations.iter().zip(steps) {
        let action = step(observation, &mut h, &mut c)?;
        for (value, expected) in action.iter().zip(values(&recorded["action"])?) {
            worst = worst.max((value - expected).abs());
        }
    }
    let mut latencies_us = Vec::new();
    for observation in observations.iter().cycle().take(2000) {
        let start = Instant::now();
        step(observation, &mut h, &mut c)?;
        latencies_us.push(start.elapsed().as_secs_f64() * 1e6);
    }
    latencies_us.sort_by(f64::total_cmp);
    Ok(json!({
        "model": model_path.display().to_string(),
        "steps_checked": observations.len(),
        "max_action_difference_vs_mujoco": worst,
        "load_ms": load_ms,
        "median_step_us": latencies_us[latencies_us.len() / 2],
        "p99_step_us": latencies_us[latencies_us.len() * 99 / 100],
    }))
}
