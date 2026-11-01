//! CPU evaluation of a pretrained monocular depth model on RNE renders.
//!
//! RNE renders a warehouse corner from several viewpoints together with its
//! exact linear depth. Depth Anything V2 Small, fetched as ONNX in float32
//! and in two int8 quantizations, runs on the CPU through tract. Each output
//! (relative inverse depth) is aligned to the true inverse depth by a
//! least-squares scale and shift, then scored with the usual AbsRel and
//! δ < 1.25 metrics. Latency is measured per image on one CPU thread.
//!
//! ```text
//! python3 tools/onnx_eval/fetch_models.py
//! cargo run --release --manifest-path tools/onnx_eval/Cargo.toml -- --output target/onnx-eval
//! ```

mod scene;

use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

use rne_render::{Camera, MeshRenderCache, RenderBackend};
use rne_render_wgpu::WgpuRenderBackend;
use serde::Deserialize;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use tract_onnx::prelude::*;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

#[derive(Debug, Deserialize)]
struct Manifest {
    model: Vec<ModelEntry>,
}

#[derive(Debug, Deserialize)]
struct ModelEntry {
    id: String,
    repository: String,
    revision: String,
    license: String,
    mean: [f32; 3],
    std: [f32; 3],
    file: Vec<ModelFile>,
}

#[derive(Debug, Deserialize)]
struct ModelFile {
    variant: String,
    path: String,
    bytes: u64,
    sha256: String,
}

struct Options {
    cache: PathBuf,
    output: PathBuf,
    size_px: u32,
    views: usize,
    threads: usize,
}

fn parse() -> Result<Options> {
    let repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let mut options = Options {
        cache: repo.join("target/models"),
        output: repo.join("target/onnx-eval"),
        size_px: 518,
        views: 8,
        threads: 1,
    };
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = || args.next().ok_or_else(|| format!("{arg} needs a value"));
        match arg.as_str() {
            "--cache" => options.cache = PathBuf::from(value()?),
            "--output" => options.output = PathBuf::from(value()?),
            "--size" => options.size_px = value()?.parse()?,
            "--views" => options.views = value()?.parse()?,
            "--threads" => options.threads = value()?.parse()?,
            other => return Err(format!("unknown option {other}").into()),
        }
    }
    if options.size_px == 0 || !options.size_px.is_multiple_of(14) || options.size_px > 1022 {
        return Err("--size must be a positive multiple of 14 up to 1022".into());
    }
    if !(1..=64).contains(&options.threads) {
        return Err("--threads must be 1..=64".into());
    }
    if !(1..=8).contains(&options.views) {
        return Err("--views must be 1..=8".into());
    }
    Ok(options)
}

/// One rendered view: RGBA8 color and linear depth in metres.
struct View {
    rgba: Vec<u8>,
    depth_m: Vec<f32>,
}

fn render(options: &Options) -> Result<(Vec<View>, f32)> {
    let mut backend = WgpuRenderBackend::new()?;
    backend.set_environment(scene::lighting());
    let camera = Camera::new(
        options.size_px,
        options.size_px,
        std::f64::consts::FRAC_PI_3,
    );
    let mut warehouse = scene::warehouse();
    MeshRenderCache::new().resolve_scene(&mut warehouse, &[scene::props_root().as_path()])?;
    let mut views = Vec::new();
    for orbit in scene::views().into_iter().take(options.views) {
        let output = backend.render_scene_camera(
            &camera,
            &orbit.camera_transform(),
            &warehouse,
            [0.62, 0.66, 0.72, 1.0],
        )?;
        views.push(View {
            rgba: output.color.rgba8,
            depth_m: output.depth.depth_m,
        });
    }
    Ok((views, camera.far_m as f32))
}

fn sha256_file(path: &Path) -> Result<String> {
    Ok(format!("{:x}", Sha256::digest(fs::read(path)?)))
}

fn load_model(path: &Path, size_px: u32) -> Result<std::sync::Arc<TypedRunnableModel>> {
    // The export declares symbolic intermediate shapes that tract cannot
    // unify; let tract infer them from the bound input size instead.
    let model = tract_onnx::onnx()
        .with_ignore_value_info(true)
        .with_ignore_output_shapes(true)
        .model_for_path(path)?;
    let symbols = model.symbols.clone();
    let subs: HashMap<Symbol, TDim> = [
        ("batch_size", 1),
        ("height", i64::from(size_px)),
        ("width", i64::from(size_px)),
    ]
    .into_iter()
    .map(|(name, value)| (symbols.sym(name), TDim::from(value)))
    .collect();
    Ok(model
        .into_typed()?
        .set_symbols(&subs)?
        .into_optimized()?
        .into_runnable()?)
}

fn input_tensor(view: &View, size_px: u32, mean: [f32; 3], std: [f32; 3]) -> Tensor {
    let size = size_px as usize;
    let mut values = vec![0.0_f32; 3 * size * size];
    for pixel in 0..size * size {
        for channel in 0..3 {
            let value = f32::from(view.rgba[pixel * 4 + channel]) / 255.0;
            values[channel * size * size + pixel] = (value - mean[channel]) / std[channel];
        }
    }
    tract_ndarray::Array4::from_shape_vec((1, 3, size, size), values)
        .expect("input shape")
        .into()
}

/// Least-squares scale and shift mapping `prediction` onto `target`.
fn align(prediction: &[f32], target: &[f32], valid: &[bool]) -> (f64, f64) {
    let (mut n, mut sx, mut sy, mut sxx, mut sxy) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for ((x, y), ok) in prediction.iter().zip(target).zip(valid) {
        if *ok {
            let (x, y) = (f64::from(*x), f64::from(*y));
            n += 1.0;
            sx += x;
            sy += y;
            sxx += x * x;
            sxy += x * y;
        }
    }
    let scale = (n * sxy - sx * sy) / (n * sxx - sx * sx);
    (scale, (sy - scale * sx) / n)
}

/// Aligned metric depth from relative inverse depth, plus AbsRel and δ1.
fn score(prediction: &[f32], view: &View, valid: &[bool], far_m: f32) -> (Vec<f32>, f64, f64) {
    let inverse: Vec<f32> = view.depth_m.iter().map(|d| 1.0 / d.max(1e-3)).collect();
    let (scale, shift) = align(prediction, &inverse, valid);
    let depth: Vec<f32> = prediction
        .iter()
        .map(|p| (1.0 / (scale * f64::from(*p) + shift).max(1.0 / f64::from(far_m))) as f32)
        .collect();
    let (mut count, mut abs_rel, mut delta) = (0.0, 0.0, 0.0);
    for ((estimate, truth), ok) in depth.iter().zip(&view.depth_m).zip(valid) {
        if *ok {
            let (estimate, truth) = (f64::from(*estimate), f64::from(*truth));
            count += 1.0;
            abs_rel += (estimate - truth).abs() / truth;
            if (estimate / truth).max(truth / estimate) < 1.25 {
                delta += 1.0;
            }
        }
    }
    (depth, abs_rel / count, delta / count)
}

fn median(values: &mut [f64]) -> f64 {
    values.sort_by(f64::total_cmp);
    values[values.len() / 2]
}

/// Turbo colormap (Mikhailov 2019, polynomial approximation), t in [0, 1].
fn turbo(t: f32) -> [u8; 3] {
    let t = t.clamp(0.0, 1.0);
    let channel = |c: [f32; 6]| {
        let v = c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * (c[4] + t * c[5]))));
        (v.clamp(0.0, 1.0) * 255.0) as u8
    };
    [
        channel([
            0.135_721_4,
            4.615_392_6,
            -42.660_32,
            132.131_09,
            -152.942_4,
            59.286_38,
        ]),
        channel([
            0.091_402_61,
            2.194_188_4,
            4.842_966_6,
            -14.185_033,
            4.277_298_6,
            2.829_566,
        ]),
        channel([
            0.106_673_3,
            12.641_946,
            -60.582_05,
            110.362_77,
            -89.903_11,
            27.348_25,
        ]),
    ]
}

/// Range of true inverse depth (2nd to 98th percentile) used to color a view.
fn inverse_range(depth_m: &[f32], valid: &[bool]) -> (f32, f32) {
    let mut inverse: Vec<f32> = depth_m
        .iter()
        .zip(valid)
        .filter(|(_, ok)| **ok)
        .map(|(d, _)| 1.0 / d)
        .collect();
    inverse.sort_by(f32::total_cmp);
    let at = |q: f32| inverse[((inverse.len() - 1) as f32 * q) as usize];
    (at(0.02), at(0.98))
}

/// Inverse depth through turbo (near is warm); invalid pixels are dark grey.
fn colorize(depth: &[f32], valid: &[bool], range: (f32, f32)) -> Vec<u8> {
    let mut rgba = Vec::with_capacity(depth.len() * 4);
    for (d, ok) in depth.iter().zip(valid) {
        if *ok {
            let [r, g, b] = turbo((1.0 / d - range.0) / (range.1 - range.0));
            rgba.extend_from_slice(&[r, g, b, 255]);
        } else {
            rgba.extend_from_slice(&[30, 30, 34, 255]);
        }
    }
    rgba
}

fn write_png(path: &Path, width: u32, height: u32, rgba: &[u8]) -> Result<()> {
    let mut encoder = png::Encoder::new(fs::File::create(path)?, width, height);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.write_header()?.write_image_data(rgba)?;
    Ok(())
}

/// Side-by-side tiles of equal size.
fn strip(tiles: &[Vec<u8>], size: usize) -> Vec<u8> {
    let mut out = vec![0u8; tiles.len() * size * size * 4];
    for (index, tile) in tiles.iter().enumerate() {
        for row in 0..size {
            let src = &tile[row * size * 4..(row + 1) * size * 4];
            let start = (row * tiles.len() + index) * size * 4;
            out[start..start + size * 4].copy_from_slice(src);
        }
    }
    out
}

fn main() -> Result<()> {
    let options = parse()?;
    if options.threads > 1 {
        let pool = tract_linalg::multithread::Executor::multithread(options.threads);
        return tract_linalg::multithread::multithread_tract_scope(pool, || evaluate(&options));
    }
    evaluate(&options)
}

fn evaluate(options: &Options) -> Result<()> {
    fs::create_dir_all(&options.output)?;
    let manifest: Manifest = toml::from_str(&fs::read_to_string(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("models.toml"),
    )?)?;
    let model = manifest
        .model
        .iter()
        .find(|model| model.id == "depth-anything-v2-small")
        .ok_or("models.toml has no depth-anything-v2-small")?;
    let (views, far_m) = render(options)?;
    let size = options.size_px as usize;
    let valid: Vec<Vec<bool>> = views
        .iter()
        .map(|view| view.depth_m.iter().map(|d| *d < far_m * 0.999).collect())
        .collect();
    println!("rendered {} views at {}x{}", views.len(), size, size);

    let mut results = Vec::new();
    let mut predictions: Vec<Vec<Vec<f32>>> = Vec::new();
    for file in &model.file {
        let path = options
            .cache
            .join(&model.id)
            .join(&model.revision)
            .join(&file.path);
        if fs::metadata(&path)?.len() != file.bytes || sha256_file(&path)? != file.sha256 {
            return Err(format!(
                "{} does not match models.toml; rerun fetch_models.py",
                path.display()
            )
            .into());
        }
        let start = Instant::now();
        let runnable = load_model(&path, options.size_px)?;
        let load_s = start.elapsed().as_secs_f64();
        // One warm-up run so the first measured image is not an outlier.
        runnable.run(tvec!(input_tensor(
            &views[0],
            options.size_px,
            model.mean,
            model.std
        )
        .into()))?;
        let (mut latencies_ms, mut abs_rel, mut delta1, mut outputs) =
            (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        for (view, valid) in views.iter().zip(&valid) {
            let input = input_tensor(view, options.size_px, model.mean, model.std);
            let start = Instant::now();
            let output = runnable.run(tvec!(input.into()))?;
            latencies_ms.push(start.elapsed().as_secs_f64() * 1e3);
            let prediction: Vec<f32> = output[0]
                .to_plain_array_view::<f32>()?
                .iter()
                .copied()
                .collect();
            if prediction.len() != size * size {
                return Err(format!("unexpected output of {} values", prediction.len()).into());
            }
            let (depth, rel, delta) = score(&prediction, view, valid, far_m);
            abs_rel.push(rel);
            delta1.push(delta);
            outputs.push(depth);
        }
        let mean = |values: &[f64]| values.iter().sum::<f64>() / values.len() as f64;
        let result = json!({
            "variant": file.variant, "file": file.path, "bytes": file.bytes, "sha256": file.sha256,
            "load_s": load_s, "median_latency_ms": median(&mut latencies_ms.clone()),
            "latency_ms": latencies_ms, "abs_rel": abs_rel, "delta1": delta1,
            "mean_abs_rel": mean(&abs_rel), "mean_delta1": mean(&delta1),
        });
        println!(
            "{:6} {:>4.0} MB  median {:>6.0} ms  AbsRel {:.3}  δ1 {:.3}",
            file.variant,
            file.bytes as f64 / 1e6,
            result["median_latency_ms"].as_f64().unwrap(),
            result["mean_abs_rel"].as_f64().unwrap(),
            result["mean_delta1"].as_f64().unwrap()
        );
        results.push(result);
        predictions.push(outputs);
    }

    // Agreement of each quantized model with float32, after alignment.
    for (index, result) in results.iter_mut().enumerate().skip(1) {
        let mut rel = Vec::new();
        for (view, valid) in valid.iter().enumerate() {
            let (sum, count) = predictions[index][view]
                .iter()
                .zip(&predictions[0][view])
                .zip(valid)
                .filter(|(_, ok)| **ok)
                .fold((0.0, 0.0), |(sum, count), ((q, f), _)| {
                    (sum + f64::from((q - f).abs() / f), count + 1.0)
                });
            rel.push(sum / count);
        }
        result["mean_abs_rel_vs_f32"] = json!(rel.iter().sum::<f64>() / rel.len() as f64);
    }

    for (view_index, view) in views.iter().enumerate() {
        let range = inverse_range(&view.depth_m, &valid[view_index]);
        let mut tiles = vec![
            view.rgba.clone(),
            colorize(&view.depth_m, &valid[view_index], range),
        ];
        for outputs in &predictions {
            tiles.push(colorize(&outputs[view_index], &valid[view_index], range));
        }
        let width = (size * tiles.len()) as u32;
        write_png(
            &options.output.join(format!("view-{view_index}.png")),
            width,
            size as u32,
            &strip(&tiles, size),
        )?;
    }
    let report: Value = json!({
        "schema_version": 1,
        "model": {"id": model.id, "repository": model.repository, "revision": model.revision,
                  "license": model.license},
        "runtime": format!("tract-onnx 0.23.9, {} CPU thread(s)", options.threads),
        "input_px": [options.size_px, options.size_px],
        "views": views.len(),
        "metric": "relative inverse depth aligned to true inverse depth by least squares (scale, shift); AbsRel and delta<1.25 on pixels closer than the far plane",
        "panels": "color | RNE depth | one aligned prediction per variant, in models.toml order",
        "variants": results,
    });
    fs::write(
        options.output.join("report.json"),
        serde_json::to_string_pretty(&report)? + "\n",
    )?;
    println!("wrote {}", options.output.display());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn alignment_recovers_scale_and_shift() {
        let prediction = [1.0_f32, 2.0, 3.0, 4.0];
        let target: Vec<f32> = prediction.iter().map(|p| 0.5 * p + 0.25).collect();
        let (scale, shift) = align(&prediction, &target, &[true; 4]);
        assert!((scale - 0.5).abs() < 1e-9 && (shift - 0.25).abs() < 1e-9);
    }

    #[test]
    fn an_affine_copy_of_true_inverse_depth_scores_perfectly() {
        let depth_m = vec![1.0_f32, 2.0, 4.0, 8.0, 50.0];
        let valid = vec![true, true, true, true, false];
        let prediction: Vec<f32> = depth_m.iter().map(|d| 3.0 / d + 0.1).collect();
        let view = View {
            rgba: Vec::new(),
            depth_m,
        };
        let (_, abs_rel, delta1) = score(&prediction, &view, &valid, 100.0);
        assert!(abs_rel < 1e-5, "AbsRel {abs_rel}");
        assert_eq!(delta1, 1.0);
    }
}
