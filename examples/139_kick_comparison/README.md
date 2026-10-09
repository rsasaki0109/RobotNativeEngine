# Go2/G1 kick disturbance comparison

This example records the existing dynamic Rapier scenes, repeats their physical
states bit for bit, then optionally renders their recorded poses beside a
skinned CC0 human. Go2 receives 120 N for 12 steps and G1 receives 50 N for
12 steps at approximately 60 Hz: 24 N.s and 10 N.s respectively. The human
is an animated visualization of the disturbance. The force is a prescribed
world-space body wrench, with no simulated human-foot collider. The scene
mass/inertia approximations are inherited from the existing robot scenes.

These different impulses illustrate recovery within each scene's tested range.
They do not measure hardware kick resistance or establish an equal-force
comparison. The earlier 320 N / 0.05 s experiment recovered on Go2 and fell
on G1; strong common-impulse recovery remains future controller work.

## Headless checks

```bash
cargo run --locked --release -p kick_comparison --example 139_kick_comparison -- \
  --headless --output target/rne-kick-comparison
```

This checks the two animation clips at keys and 120 Hz subframes without a GPU,
then captures both simulations and requires literal equality of their ordered
physical floating-point states on replay. It writes `go2-trace.json`,
`g1-trace.json`, and `summary.json`. Recovery gates require upright final body
height, small residual tilt and all support feet contacting the floor.
The clocks are aligned at 30 Hz over 187 frames (6.23 seconds); the force
first appears at frame 61. Recorded robot poses are never edited for capture.

## Render and encode

A working Vulkan/Metal/DX12 wgpu adapter is required. Mesa llvmpipe works for
offline Linux rendering. Pillow 10.1 or newer is required only for GIF encoding.

```bash
cargo run --locked --release -p kick_comparison --example 139_kick_comparison -- \
  --render --output target/rne-kick-comparison
python3 examples/139_kick_comparison/encode.py target/rne-kick-comparison
```

To render an existing capture without repeating the simulation, use
`--render-only`. `--start-frame N --frame-count N` can divide rendering into
disjoint batches. Any requested GPU failure is returned as an error.

The encoder uses a fixed global palette and a 30/30/40 ms duration pattern.
Its labels use the trace's actual force window. The 960 x 540 GIF, poster and
metadata are `kick-comparison.gif`, `.png`, and `.json`. Metadata includes
source, asset and trace SHA-256 hashes plus measured physics results.
Copy those three files to `docs/media/` when updating the README capture.

## Human motion and provenance

[The human asset](../../assets/fixtures/kick_human/README.md) uses pinned
MakeHuman/MPFB CC0 mesh, rig and skin weights, with original procedural
clothing textures, shoes and animation. It is an anatomical textured character,
not a scanned person. Only the root translates; limb bones use rotations
with constant local offsets and unit scales. The kick has weight transfer,
forward knee chamber, extension, retraction and controlled foot planting.
Both CPU and GPU glTF sampling use shortest-path quaternion interpolation.

The official robot meshes retain their bundled BSD-3-Clause notices in
`assets/robots/go2_description` and `assets/robots/g1_description`.
