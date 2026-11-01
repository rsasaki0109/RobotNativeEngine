# CPU evaluation of pretrained ONNX models on RNE scenes

![Depth Anything V2 Small on two RNE views: render, RNE's exact depth, and the float32, int8 and uint8 models' aligned depth](../../docs/media/onnx-depth-eval.png)

[Report](../../docs/media/onnx-depth-eval.json)

This tool fetches pretrained models published as ONNX, runs them on the CPU
with [tract](https://github.com/sonos/tract), and scores them against what RNE
knows exactly about its own scenes. The first model is
[Depth Anything V2 Small](https://huggingface.co/onnx-community/depth-anything-v2-small),
a 25M-parameter monocular depth network, in float32 and in the two int8
quantizations the publisher ships.

```bash
python3 tools/onnx_eval/fetch_models.py                       # ~150 MB into target/models
cargo run --release --manifest-path tools/onnx_eval/Cargo.toml -- --output target/onnx-eval
cargo run --release --manifest-path tools/onnx_eval/Cargo.toml -- --threads 4 --output target/onnx-eval-4t
python3 tools/onnx_eval/montage.py target/onnx-eval docs/media/onnx-depth-eval.png 1 4
```

Options: `--size` (input pixels, a multiple of 14; default 518), `--views`
(1–8), `--threads` (tract's multi-threaded matrix products), `--cache`.

## What it measures

RNE's wgpu renderer draws a warehouse corner from eight viewpoints. The scene
has a floor, two walls, shelving and the vendored CC0 Poly Haven props, spread
from about 1 m to 6 m. Each render returns RGBA and the exact linear depth of
every pixel. The model receives the ImageNet-normalized render and returns
relative inverse depth. A least-squares scale and shift aligns that output to
the true inverse depth on every pixel nearer than the far plane. The scores
are AbsRel (mean |d − d*| / d*) and δ1 (share of pixels within a factor 1.25),
the usual metrics for affine-invariant depth. Latency is the median over the
eight views after one warm-up run.

## Results

518 × 518 input, eight views, 4-core development container, no GPU:

| Variant | File | AbsRel | δ1 | Difference from f32 | 1 thread | 4 threads |
| --- | --- | --- | --- | --- | --- | --- |
| f32 | 99 MB | 0.018 | 0.997 | — | 4.8 s | 2.3 s |
| int8 (dynamic) | 27 MB | 0.020 | 0.997 | 1.1% | 5.4 s | 4.3 s |
| uint8 (dynamic) | 27 MB | 0.018 | 0.997 | 0.8% | 5.8 s | 4.8 s |

- **Quantization costs almost no accuracy.** Both int8 files are 3.6× smaller.
  Their aligned depth differs from float32 by about 1% and scores the same
  against the truth. Their errors show as fine speckle on smooth walls.
- **It does not make tract faster.** These are dynamic quantizations: the
  activations are quantized at run time, and tract has no fast int8 matrix
  path for them on x86. They run 10–20% slower than float32 on one thread,
  and about 2× slower on four, where float32 speeds up 2.1× and int8 only
  1.25×. The speed benefit of int8 needs a runtime with x86 int8 kernels
  (for example ONNX Runtime with VNNI), or a statically quantized model. On
  this runtime, the int8 files save memory and download, not time.
- **Smaller inputs are much cheaper.** At 252 × 252 the float32 model takes
  about 0.75 s on one thread, at AbsRel 0.05 on the views tried.
- **RNE's ground truth exposes model errors** that a photo dataset could not.
  The roller shutter on the back wall is drawn flat, but every variant
  predicts it standing out from the wall (top row of the image).

## Why a standalone crate

tract adds about 47 packages and twelve duplicated crate names (two `syn`, two
`prost`, three `bit-vec`, …) to the dependency graph. The root workspace
denies duplicate versions unless each is documented, so this evaluation tool
keeps its own `Cargo.lock` outside it, as `web/rne_web_viewer` does. Nothing
here ships with RNE. Moving ONNX inference into the engine itself would mean
recording those exceptions, or choosing a lighter runtime.

## Provenance and safety

`models.toml` pins each model by repository revision, file size and SHA-256,
with its license. `fetch_models.py` downloads only the listed ONNX files into
`target/models` and rejects any mismatch, and the evaluator checks the digests
again before loading. ONNX is a data format: tract parses the graph and
weights, and no code from the model repository runs. Depth Anything V2 Small
is Apache-2.0; the larger Depth Anything V2 models are CC-BY-NC and are not
listed.

## Limits

- The scenes are synthetic renders with rasterized PBR lighting, so they test
  how a photo-trained model transfers to RNE images, not real-camera accuracy.
- Scores are scale-and-shift aligned per image. They say nothing about metric
  depth without that alignment.
- Timings are from one container. tract runs no GPU code, and none of these
  variants was tuned for this CPU.
