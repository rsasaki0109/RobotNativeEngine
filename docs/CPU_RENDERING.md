# CPU ray tracing

`rne_render_cpu` renders a `RenderScene` on the CPU, so camera sensors work
on machines without a GPU (CI runners, cloud containers, laptops). It
implements `RenderBackend`. The camera model in `rne_sensor` (lens
distortion, rolling shutter, noise; see
[CAMERA_SIMULATION.md](CAMERA_SIMULATION.md)) therefore runs on it unchanged.

```rust
use rne_render_cpu::{CpuRayTracer, CpuRayTracerConfig};

let tracer = CpuRayTracer::new(CpuRayTracerConfig::default());
let output = tracer.render(&camera, &camera_pose, &scene, sky_rgba);
// output.color (sRGB), output.depth (view depth, m), output.item_ids (per pixel)
let boxes = output.item_boxes(&person_items); // exact 2D boxes of visible parts
```

## What it renders

| Feature | How |
| --- | --- |
| Boxes, spheres, cylinders | Analytic intersection in the item's unit local space (the same unit shapes as `rne_render_wgpu`) |
| Triangle meshes | Static, or skinned on the CPU (`GltfAnimationPlayer::sample_part_cpu`); median-split BVH with leaves of 4 triangles, rebuilt each frame |
| Color | `color_rgba × material.base_color_rgba × base-color texture` (bilinear, sRGB-decoded), as in the wgpu shader |
| Lighting | Lambert from one sun with a hard shadow ray, sky/ground hemisphere ambient, emissive |
| Anti-aliasing | `samples_per_axis²` color rays per pixel (default 4) |
| Depth | Linear view depth from the pixel centre, `far_m` where nothing is hit |
| Item ids | Index into `RenderScene::items` at each pixel centre, `BACKGROUND_ITEM` where nothing is hit |

It does not render metallic or specular response, normal maps, environment
maps, transparency or soft shadows. Each texture is decoded once per render,
however many items share it.

## Determinism and threads

Rows are cut into blocks of four, and worker threads take the next free block
(`CpuRayTracerConfig::threads`, 0 = available parallelism). Each pixel is a
pure function of the scene, the camera and the configuration, so the image is
byte-identical for any number of threads. The crate and example 150 test this.

## Speed

[Example 150](../examples/150_cpu_render/main.rs) renders a street with five
textured 30k-triangle people at 640 × 480 with 4 samples per pixel:

| Threads | Time |
| --- | --- |
| 1 | 0.67 s |
| 4 | 0.34 s |

The scene is prepared on one thread: triangles are transformed into the
world and the BVH is built for every render. That serial step limits the
speedup.

## Use in perception tasks

Example 149 (`--camera`) mounts a forward camera on the robot. It renders
people as the CC0 human mesh, and cars and bicycles (with a posed rider) as
procedural meshes. Each
frame comes with exact 2D boxes of the visible part of every object. The
camera calibration in each task's `setup.json` matches the renderer, so a
LiDAR detection's 3D box projects onto the rendered object; a test checks
this.
