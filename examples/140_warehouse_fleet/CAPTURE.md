# Warehouse fleet recording and capture

The headless `140_warehouse_fleet` example records forty AMRs doing seeded
pick-and-deliver work, using directed lanes, station pockets and cell leases.
The optional `140_warehouse_fleet_render` example replays those exact recorded
poses through RNE's Wgpu renderer. The Python encoder adds counters and event
labels from the trace and produces the README GIF, MP4 and poster.

The simulation uses bounded kinematics, ideal state and an explicitly configured
battery model. It does not model wheel traction, contact forces, physical cargo
pickup, sensor errors or hardware connections. Floor rings and the few displayed
remaining routes are visual annotations. Geometry and telemetry are original;
[WareTwin](https://github.com/WayneChou-bot/WareTwin) inspired the scenario and
dashboard direction. No WareTwin source or assets are included.

## Reproduce from the repository root

The Linux capture path needs Python, FFmpeg, DejaVu Sans fonts and a working
wgpu adapter; software rendering works. Capture dependencies are optional and
are not required for the simulator or its tests. Install Python dependencies in
an environment under `target`, then record and check a second fresh process:

```sh
python3 -m venv target/warehouse-fleet/.venv
target/warehouse-fleet/.venv/bin/python -m pip install \
  -r examples/140_warehouse_fleet/requirements.txt

cargo run --locked --release -p warehouse_fleet --example 140_warehouse_fleet -- \
  target/warehouse-fleet/trace.json target/warehouse-fleet/validation.json
cargo run --locked --release -p warehouse_fleet --example 140_warehouse_fleet -- \
  target/warehouse-fleet/independent-trace.json \
  target/warehouse-fleet/independent-validation.json

target/warehouse-fleet/.venv/bin/python \
  examples/140_warehouse_fleet/check_recording.py
```

Inspect a single preview before capturing the complete sequence:

```sh
cargo run --locked -p warehouse_fleet --features capture \
  --example 140_warehouse_fleet_render -- \
  --trace target/warehouse-fleet/trace.json \
  --output target/warehouse-fleet/preview --frame 0

cargo run --locked -p warehouse_fleet --features capture \
  --example 140_warehouse_fleet_render -- \
  --trace target/warehouse-fleet/trace.json \
  --output target/warehouse-fleet/frames-raw

target/warehouse-fleet/.venv/bin/python examples/140_warehouse_fleet/encode.py
```

Python defaults point to `target/warehouse-fleet`; they never write beside the
source by default. The renderer requires explicit `--trace` and `--output`.
Use `--range FIRST END_EXCLUSIVE` for part of a trace, or `--frame INDEX` for one
frame. The raw scene is 1024 by 656 pixels. The HUD produces 1280 by 800 pixels,
481 frames at 20 fps and 5x playback for the recorded 100–220 second interval.
Diagnostic or fixture previews require explicit switches and are marked
unqualified in their metadata; the encoder rejects them.

For an alternative output directory, pass paths explicitly to every stage.
The encoder accepts `--trace`, `--raw`, `--output`, `--preview INDEX` and
`--compose-only`. The independent checker accepts `--trace`, `--validation`,
`--independent-trace`, `--independent-validation` and an `--output` proof file.
The encoder produces `warehouse-fleet.gif`, `warehouse-fleet.mp4`,
`warehouse-fleet.png` and `media-proof.json`. PNG sequences are generated
artifacts and should remain under `target`.

## What the checks establish

The simulator retains acceptance metrics and compares complete internal state
between two fresh seeded runs. The independent checker also requires
byte-identical trace and validation files from a separate process, verifies
capture-interval motion and conservative clearance bounds, checks recorded
counters, and checks an actual recorded arrival at a charging station. Exact
replay is qualified within the same runtime; cross-platform bit identity and
physical robot safety are not established by this example.

Source provenance checks bind the example's `main.rs` and package manifest,
the workspace manifest and lockfile, and eight declared engine inputs: RNG, clock,
traffic coordination, grid and world resource sources, plus their three crate
manifests. The renderer compares these against compile-time bytes; Python
compares them against the current files. Raw capture evidence additionally
binds `render.rs`, the trace, frame indices and every decoded RGBA hash. These
are checks of the included inputs, not the entire compiler dependency graph or
hardware behaviour. Retain the engine revision with any distributed evidence.

External-input guards raise explicit errors even under `python -O` or
`PYTHONOPTIMIZE`. The regression checks exercise failed simulation acceptance,
stale example/engine inputs and altered pixels in normal and optimized Python:

```sh
target/warehouse-fleet/.venv/bin/python \
  examples/140_warehouse_fleet/test_capture_integrity.py
```
