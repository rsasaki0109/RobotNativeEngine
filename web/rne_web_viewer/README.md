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
