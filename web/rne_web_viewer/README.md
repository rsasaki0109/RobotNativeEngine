# RNE G1 Playground and replay inspector

Control the vendored Unitree G1 23-DoF model in the browser with RNE's
`rne_urdf_import`, ECS transforms and `rne_render_wgpu` renderer. Use a
WebGPU-capable browser. The URDF and its 29 STL visuals are embedded in the WASM
bundle; no native host or external model download is required at runtime.

This is a **kinematic pose playground**. WASD/arrow keys and momentary screen
buttons translate/rotate the pelvis; they do not run a walking controller.
The 23 joint sliders use the source URDF's angular limits. There is no balance,
collision response or contact simulation. Each rendered tick advances by at
most 0.015 m and 0.025 rad; redraw rate therefore affects preview speed.
The base is confined to an 8 × 8 m area. Reset restores its initial pose and
zero joint targets; blur, backgrounding and pointer cancellation stop drive
input. Drag to orbit the camera and scroll to zoom.

## Laboratory environment

The metre-scaled, roofless laboratory has concrete floor panels, painted boundary
lines, framed windows, workbenches and equipment cabinets. All furniture stays
outside the 8 × 8 m driving area. Concrete grain and HDR studio lighting are
procedural and deterministic, with no external asset requests. Roughness, metallic
materials and the renderer’s existing directional shadows provide surface depth.
Each box face has a full UV map. Concrete uses shared, tileable color, normal
and roughness maps; G1 shells use separate satin paint and dark polymer coating
responses while retaining the source mesh colors. The static scenery and
material maps are cached when the G1 loads. Furniture is visual scenery;
this kinematic viewer does not add collisions or contact physics.

## Planning × Control experiments

Open **PLANNING × CONTROL** to select a task, global planner, local controller,
and layout seed, then run or compare all four method pairs. The first three
navigation tasks are an open-space goal, a static obstacle detour, and an
obstacle inserted after 60 simulation steps (2 seconds). The fixed seed controls
obstacle placement; the methods themselves are deterministic. Changing selectors
applies to the next run. Every run restores the same initial pose and velocity.

The straight-line baseline validates its entire path against the footprint-padded
costmap and reports planning failure if blocked. A* uses `rne_nav::plan_path`;
diagonal moves cannot cut through occupied cell corners. Local control uses
`rne_nav::pure_pursuit_follow` or `DwaPlanner`, with common 0.55 m/s, 1.6 rad/s,
1 m/s² and 3 rad/s² speed/acceleration limits. A newly inserted obstacle updates
the map and triggers one replan. The swept circular footprint detects contact;
controller failure and a 60-second simulation timeout are separate outcomes.

This is a **planar unicycle model with a 0.28 m circular footprint**, shown using
G1's animated mesh. It does not evaluate articulated balance, foot contact, or
bipedal walking performance. Navigation uses `(x, y, yaw)` internally; the visual
world maps these to `(X, -Z, yaw)` to keep RNE's Y-up convention.

A `SimClock` advances explicitly by 33,333,333 ns per simulation step (about
30 Hz), independent of elapsed browser time. Pause stops this clock and the
controller; **1 step** pauses and advances exactly one tick. The fast four-pair
comparison executes 32 of the same simulation steps per redraw; this changes
presentation speed, not experiment conditions. **Manual mode**, drive input,
Stop, Reset, or checkpoint start cancels automated navigation. Reset returns
home; manual takeover preserves the current pose. A completed result is saved
once and frozen, with the latest 12 completed runs kept in memory. A new four-pair
comparison clears this history to show one task/seed batch.

The 3D floor and the small overhead view show blue reference paths and green
executed traces, orange obstacles and a gold goal. Results report simulation
time, reference-tracking RMSE, travelled distance, minimum obstacle clearance,
latest planned length and replan count (expand a method in the results table
for the additional distance/clearance details). RMSE uses each executed tick against the
reference active at that tick. No-obstacle clearance and error with no executed
samples are shown as `—`; a failed plan is never presented as perfect tracking.
These are single deterministic runs, not aggregate success-rate or wall-clock
performance benchmarks. Planning compute time is not measured in this version.

Browser interaction validation with an installed Chromium:

```bash
python3 scripts/test_g1_experiments_browser.py --url http://127.0.0.1:8080/ \
  --browser /usr/bin/chromium --output work/experiments-browser
```

## Scene editor and saved settings

Open **Scene editing / Save** in the experiment panel. This selects the custom
task and stops an active experiment. Click the overhead editor or a visible
point on the 3D floor to place a goal or rectangular obstacle. Dragging the 3D
view still orbits the camera; a click moves geometry only after floor picking
completes. Coordinates are world **X/Z in metres**, with optional 0.1 m snapping.
The coordinate inputs provide an alternative to pointer input. Erase removes the
last obstacle under the pointer; undo keeps up to 40 editor states. Presets cover
an open floor, narrow corridor, alternating obstacles and an unreachable enclosure.

New obstacles can appear after 60 simulation steps (about two seconds). Imported
obstacles may specify any `appear_step` from 0 to 900. Zero means present at start.
All obstacles appearing at the same step trigger one map update and replan. The
preview draws future obstacles as flat blue markers and dashed outlines; they
are not collision obstacles until their insertion step. All four comparisons
snapshot the same scene at start, including scheduled obstacles. Later draft
changes cannot alter queued runs. Results include a scene ID so edited layouts
with the same seed are distinguishable. In a custom task the geometry is explicit;
seed is retained as metadata and does not randomize the edited layout.

**Save in browser** stores one document in this origin's local storage, and
**Load saved** restores it. JSON export/import lets a scene move between browsers.
Loading restores the planner, controller and seed, stops any active run and
previews the scene; it never starts playback automatically. File size is bounded
to 64 KiB. Unsupported versions, unknown fields, non-finite values, invalid
geometry and invalid selections are rejected before changing the current scene.
A second validation boundary in Rust checks geometry before the simulation uses it.

Version 1 document (planner/controller 0 or 1; seed is a u32):

```json
{
  "format": "rne-navigation-scene",
  "version": 1,
  "selection": {"planner": 1, "controller": 1, "seed": 42},
  "scene": {
    "goal": {"x_m": 2.8, "z_m": -0.55},
    "obstacles": [
      {"x_m": 1.8, "z_m": 0, "width_m": 0.44, "depth_m": 1, "appear_step": 60}
    ]
  }
}
```

Start remains at X/Z = 0 with heading +X and zero velocity. Goals are bounded to
±3.4 m and at least 0.55 m from the start. Up to 24 obstacles may have dimensions
0.2–3 m and outer edges within ±3.65 m. Static obstacles must leave 0.43 m from
the start/goal centre (robot radius plus clearance). Overlapping walls and a
valid but unreachable goal are allowed, so planning failure can be demonstrated.
Storage is local to the browser; there is no scene upload service.

Editor checks:

```bash
node --test web/rne_web_viewer/scene_editor.test.cjs
python3 scripts/test_g1_scene_editor_browser.py --url http://127.0.0.1:8080/ \
  --browser /usr/bin/chromium --output work/editor-browser
```

## Walking, waving and checkpoint run

**Walking motion ON** adds a bounded, alternating leg/arm pose animation while
moving or turning. Stopping blends back to the manual slider pose. Toggle it
off for manual inspection. **Wave** raises the left arm and waves for 180
preview ticks, then restores the pose; another press restarts the gesture.
All composed joint coordinates are clamped to the original URDF limits.
These are authored pose animations, not a physics-based walking controller.

**Mission start / Retry** restores the base and joint targets, clears the old
gesture, and starts a four-checkpoint course: (X, Z) = (1.5, 0), (1.5, -1.5),
(0, -1.5), (0, 0) metres. Enter each 0.35 m radius circle in order. The next
circle is gold, completed circles turn green, and the HUD shows progress,
distance and elapsed preview time. Finishing freezes the score; retry starts
over. Reset cancels the course and animation. Time is fixed tick count / 60,
so it is **preview time**, not a wall-clock or calibrated performance measure.

Model provenance and license: [upstream details](../../assets/robots/g1_description/UPSTREAM.md),
[Unitree BSD license](../../assets/robots/g1_description/LICENSE.unitree_ros).
The static bundle includes the model license.

The existing `mm_minimal` scene API remains available for callers. The replay
inspector reads a versioned `.rne-replay` artifact in the browser and exposes
a timeline, selected observations, and per-step hashes without rerunning policy
code or physics. Its loaded records do not drive the live G1.

## Publish on GitHub Pages

In repository **Settings → Pages → Build and deployment**, select **GitHub
Actions** as the source. The `.github/workflows/pages.yml` workflow tests
the controls and embedded model, checks the WASM target, builds a static
bundle, and deploys it on changes to `main` (or a manual workflow run).
Pull requests build and upload an artifact without deploying.

For this repository the expected project URL is
`https://rsasaki0109.github.io/RobotNativeEngine/`; the deployment job
reports the actual URL. The build uses `--public-url "./"`, so asset URLs
also work under a project subdirectory or a different repository name.
Pages must be enabled before the deployment job can succeed.

For a browser check against the built bundle (including a project subdirectory),
use existing Playwright/Pillow installations and Chromium:

```bash
python3 scripts/test_g1_web_browser.py --url http://127.0.0.1:8080/RobotNativeEngine/ \
  --browser /usr/bin/chromium --output work/g1-browser
```

## Prerequisites

```powershell
$env:Path += ";$env:USERPROFILE\.cargo\bin"
rustup target add wasm32-unknown-unknown
cargo install trunk
```

## Run locally

From this directory:

```powershell
trunk serve
```

Then open the URL printed by Trunk (default `http://127.0.0.1:8080`).

## Build static bundle

```powershell
trunk build --release --public-url "./"
```

Output lands in `dist/` (WASM + `index.html`).

## Controls

- **Left drag**: orbit camera
- **Mouse wheel**: zoom
- **WASD / arrow keys**: move and turn the G1 base
- **On-screen arrows**: hold to move/turn; release to stop
- **Joint sliders**: set all 23 joint coordinates in radians
- **Reset**: restore the initial base pose and joint targets
- **Replay file**: choose a `.rne-replay` or JSON artifact in the Replay inspector
- **Replay slider / Play**: inspect or step through the recorded interval

## Preview timing

Manual movement is a fixed kinematic increment per render tick, rather than
wall-clock simulation. The legacy `WebScene::frame` sinusoidal animation remains
available as an API but does not control the G1 playground.

## Replay inspection

Generate an artifact with the headless runner, then start this viewer and select
the file from `target/runs/`:

```powershell
cargo run --release -p rne_asset_cli -- run assets/runs/mesh_diff_drive.rne.run.toml
trunk serve
```

The panel validates schema version, frame count, sequential timestamps, tagged
wheel/joint actions, and the selected joint/sensor observation metadata before
enabling the timeline. It displays exact 64-bit physics and sensor payload
hashes without rerunning the recorded actions or invoking a simulator. When the
artifact carries full typed sensor payloads (produced by a run manifest with
`[[sensors]]` subscriptions), the frame line summarizes each payload — LiDAR
point count, camera RGB/D dimensions, or IMU / wheel-encoder sequence numbers.

The same file picker accepts `rne_behavior_replay` artifacts emitted by
`xtask behavior-ci`. Those are normalized into the existing timeline and show
the scenario seed, failed contract, first violating step, named dimensions,
G1 phase/contact observation, and exact state digest. The inspector never
reruns the scenario; use `cargo run -p xtask -- behavior-replay <artifact>` for
headless verification.

## Workspace checks

```powershell
cargo fmt --all
cargo clippy --workspace --all-targets -- -D warnings
cargo clippy -p rne_web_viewer --target wasm32-unknown-unknown -- -D warnings
trunk build
node replay.test.cjs
node controls.test.cjs
cargo test --lib
cargo check --target wasm32-unknown-unknown
cargo test --workspace
```
