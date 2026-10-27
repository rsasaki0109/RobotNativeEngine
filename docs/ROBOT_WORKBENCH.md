# Robot workbench

The local workbench combines native joint control, saved poses, a floor/obstacle
editor, RGB/depth/LiDAR inspection, and URDF/MJCF loading in one browser window.
Rapier runs the physics and wgpu renders the images on the host. The browser
displays those images and sends commands; it does not synthesize robot motion or
sensor readings.

```bash
cargo run --release --locked -p robot_workbench
```

Open the printed loopback URL (normally `http://127.0.0.1:8765`). The default
robot is the already-vendored SO-101, with its authored mesh colors. No robot
pack, web framework, browser renderer, or JavaScript dependency download is
needed. A working native wgpu adapter is needed for the interactive host.

![SO-101 workbench with joint controls and native sensor views](media/robot-workbench.png)

## Control a robot and save a pose

Set named joint targets with the sliders or numeric fields, then press Play.
Targets and measured positions are shown separately; revolute coordinates use
radians and prismatic coordinates use metres. Pause stops new step requests.
Single Step advances one `4,166,667 ns` step; Play requests bounded batches of
four such steps, so browser redraw timing never becomes simulation time. The
host does not promise real-time playback speed.

The servo ramps its commanded position using the model's velocity limit and
caps motor effort at its effort limit. These are command/effort limits, not a
claim that contact or gravity cannot produce a larger measured velocity. When
the source has no actuator specification (including the current MJCF importer),
the UI labels the preview defaults: 1 rad/s and 10 N·m, or 1 m/s and 10 N for a
linear axis. Continuous joints have a ±2π target-entry window; this is not a
physical joint stop. Invalid, unknown, missing, non-finite, out-of-range or wrong-unit
targets are rejected before replacing the current targets.

Name a pose and choose **Save targets**. Applying a saved pose changes targets;
the robot moves when stepping resumes. A pose stores named, explicitly typed
angular/linear coordinates and the source robot's SHA-256 identity. It cannot
silently load onto a different source model.

## Edit the scene and retain a project

Select the floor or an obstacle, edit its position, dimensions and color, then
apply. Add Box creates another static obstacle; Delete removes the selected
object. Floors are ordinary thin boxes. Scene edits explicitly reset the robot
and simulation clock, so a stale physics world cannot disagree with the edited
geometry. The same colliders are used by physics and LiDAR.

Save JSON downloads a versioned `.rne.workbench.json` project containing the
source XML, target positions, saved poses, model orientation/inertia choice and
scene objects. Load JSON validates the candidate and constructs its physics
world and mesh resources before swapping live state. A rejected import leaves
the old project and physics state intact. File operations stay in the browser's
normal upload/download flow; the server does not overwrite project files.

## Inspect sensors

- RGB and depth use the same fixed 160 × 120 observation camera. Hover over a
  depth pixel to read metres. Black pixels are out of range, not zero distance.
- LiDAR casts 180 horizontal rays at world height 0.5 m with a 10 m range.
  Its top-down plot has 1 m rings; `null` ranges mean no return. An empty valid
  scan remains distinguishable from an error.
- Camera and LiDAR share the exact simulation timestamp of the displayed
  state. These workbench sensors are instantaneous and noiseless, with zero
  simulated transport latency. They are world-mounted inspection sensors,
  independent of the orbit camera used by the main robot view.

## Load another robot

The model picker offers SO-101, LeKiwi + SO-101, a prismatic URDF slider, and a
two-joint MJCF arm. The file picker accepts URDF or MJCF source XML. For meshes, set the asset
root to the robot package directory:

```bash
cargo run --release --locked -p robot_workbench -- \
  --robot assets/robots/so101/so101.urdf \
  --asset-root assets/robots/so101 --port 8765
```

Select Z-up to convert a Z-up model into the workbench's Y-up world; the SO-101
preset already declares this conversion. Mesh references must resolve beneath
the selected asset root. Mesh files are not embedded in the saved project, so
reopening a mesh project on another machine also requires that asset root.

The default **simple preview** mode deliberately uses the importer's existing
default link masses and collision-derived inertia. Select **URDF mass/inertia**
for models with valid declared inertials. Invalid tensors are rejected rather
than repaired silently. The SO-101 fixed-base preset defaults to simple preview. The importer treats
its near-massless dummy frame as a frame marker rather than a physical inertia. The fixed-base presets
fix the robot base and disable robot self-collision; it is an interactive
control/model inspection tool, not a calibrated hardware or locomotion proof.

MJCF support is exactly the existing strict `rne_mjcf` subset: hinge/slide
joints and supported primitive/mesh geometry. It does not import a MuJoCo
actuator model, derive authored inertials, or support free/ball joints and
arbitrary MJCF features. Physics mimic joints are also rejected by this host.
Errors are displayed without discarding the previously loaded scene.

Inputs are bounded: approximately 4 MiB per project/request, 2 MiB source XML,
128 objects/poses, 256 controllable joints, 512 links/joints and 32 MiB per mesh.
The host binds only to `127.0.0.1`, accepts same-origin JSON commands, and keeps
simulation state in one owned application instance.

## Validation

Model, pose, editing, native control, replay and LiDAR checks need no GPU:

```bash
cargo test --locked -p robot_workbench --no-default-features
cargo run --release --locked -p robot_workbench --no-default-features -- --smoke
cargo run --release --locked -p robot_workbench --no-default-features -- \
  --check-project robot.rne.workbench.json
```

The project checker above is for primitive-only projects; mesh projects need
the interactive host's explicit asset root. `xtask ci-headless` includes the
workbench's headless tests and smoke. Workspace lint/tests also compile its
interactive code and HTTP boundary tests.

With the server running, Playwright and an existing Chrome/Chromium installation
can exercise real UI interactions. This script downloads nothing:

```bash
python3 scripts/test_robot_workbench_browser.py \
  --url http://127.0.0.1:8765 --browser /usr/bin/google-chrome \
  --output /tmp/robot-workbench-checks
```

The browser check covers SO-101 motion, pose save/apply, scene editing,
project export/reload, rejected edits preserving physics state, URDF/MJCF file
uploads, sensor pixels, and desktop/mobile layout. Its screenshots and JSON
files are small diagnostic artifacts; normal use records no video.

All workbench code lives in the non-published application example
`examples/116_robot_workbench`. No public core API, frozen compatibility
baseline, ROS dependency, or rendering requirement for simulation tests changes.

## LeKiwi + SO-101 planar workbench

Run the vendored mobile manipulator without hardware:

```bash
cargo run --release --locked -p robot_workbench -- --preset lekiwi_so101
```

The workbench uses `lekiwi_so101_workbench.urdf`, regenerated with
`python3 scripts/gen_lekiwi_so101_urdf.py --workbench`. This separate preview
retains the SO-101 collision geometry and authored arm inertials, and aligns
the two Z-up source frames before the single whole-robot Y-up conversion. The
historical combined asset stripped arm collisions and inertials. The
existing generated asset and its consumers are unchanged.

The bundled presets now resolve meshes beneath `assets/robots`, including the
LeKiwi chassis and SO-101 arm. The LeKiwi preset converts the CAD Z-up frame to
RNE Y-up, starts the base at 0.06 m, and provides a floor, pickup table, target
block and inspection wall. The table and block are **static reference geometry**;
this movement preset does not implement grasping. Use `lekiwi_grasp` below for
the contact grasp task. Autonomous pick/place planning is not implemented.

The base panel accepts body-frame X/Z velocity in m/s and +Y yaw in rad/s.
Apply the command, then Play or Single Step. Translation magnitude is limited to
0.1 m/s and yaw to pi/6 rad/s; invalid commands leave the previous command
unchanged. Positive yaw turns +X toward -Z. Stop Base zeros only the base command,
so the six named SO-101 joint servos can continue. Pause suspends stepping and
retains the command; Reset and project/model replacement start with zero drive.

This is an explicitly **prescribed planar preview**: constant-twist integration
commands the kinematic root at the same fixed step as the articulated physics.
The three wheel motor setpoints use the existing LeKiwi mount geometry and
0.0508 m wheel radius; they are shown separately from measured arm positions.
Wheel/contact dynamics do not determine the chassis trajectory. Obstacles do
not stop the commanded base, and wheel slip, motor calibration, physical
odometry accuracy and hardware performance are not established. Arm links use
Rapier articulation and bounded servos. The preset enables authored SO-101
inertials; the chassis still uses preview properties where its source lacks them.
The RGB-D and LiDAR views remain the existing world-mounted inspection sensors.

Mobile projects use schema 2 with `planar_base.height_m`. Existing schema-1
fixed-base projects remain readable and are still saved without that field.
Drive commands are transient and are never restored by loading a project.

### Record and verify operations

Every Step/Play batch records the applied arm targets, base command, step count,
simulation timestamp and complete physics-state-v2 hash. **Save recording**
exports the initial project and those inputs. **Verify and replay recording**
constructs a fresh simulation, executes the inputs, and checks every recorded
batch before replacing the displayed session; rejected recordings leave the
current session intact. The result is shown paused at its final state with the
base command zeroed. This is input replay verification, not animated timeline
playback. Scene/model edits and Reset begin a new recording; save first if needed.
Records are bounded to 2,400 batches and 4 MiB. When full, stepping fails before
advancing; export and reset to continue. Mesh assets are not embedded. Exact
hash verification requires the same compatible engine/runtime and asset bytes.

The same workflow is available without a renderer:

```bash
cargo run --locked -p robot_workbench --no-default-features -- --smoke-lekiwi
cargo run --locked -p robot_workbench --no-default-features -- \
  --replay robot.rne.workbench-replay.json --asset-root assets/robots
```

The LeKiwi smoke checks diagonal motion, turning, stopping, six arm joints and
exact recording verification using the vendored model. Unit tests additionally
check rotated body-frame motion, invalid-command atomicity, a moving articulated
slider, schema compatibility, recording bounds and altered-recording rejection.


## Contact grasp without hardware

```bash
cargo run --release --locked -p robot_workbench -- --preset lekiwi_grasp
cargo run --release --locked -p robot_workbench --no-default-features -- --smoke-grasp
```

The **接触把持デモ** button resets a dedicated scene, drives the base 0.2 m,
closes the SO-101 jaw on a 20 g, 2 cm cube, lifts it and checks two seconds
of support. Success requires at least 50 mm of lift and positive contact
impulses on both finger pads at each of 20 hold samples. The displayed report
is the last demo result, not a continuously evaluated task score. Set the
gripper target to 0.8 and resume playback to release. The smoke command also
replays the recorded inputs and verifies every state hash. Tests include
release under gravity and a missed-block failure case.

The grasp variant replaces the two gripper mesh bounding boxes with explicit
box pads, preserves authored arm inertias, and caps gripper effort at 0.06 N·m.
It uses contact and Coulomb friction, without attaching, welding or prescribing
the cube pose. Regenerate with `python3 scripts/gen_lekiwi_so101_urdf.py --grasp`.
These simplified pads and tuned scene demonstrate simulated grasping; they do
not establish real SO-101 payload capacity. The base remains a prescribed
kinematic preview, and robot self-collision is disabled.

The scene editor supports fixed boxes and dynamic boxes with mass (0.001–1000 kg)
and friction (0–2). Dynamic boxes use exact solid-box inertia and require
project schema 3; static schema 1/2 files remain readable. Live object telemetry
shows position, height relative to the authored initial position, and contacting
robot links. Scene edits reset dynamics and the operation recording.


## Pick, carry and place

```bash
cargo run --release --locked -p robot_workbench -- --preset lekiwi_pick_place
cargo run --release --locked -p robot_workbench --no-default-features -- --smoke-pick-place
```

**初期化してピック＆プレースを実行** creates the dedicated two-table scene and
runs the contact grasp above. In the default scene the base travels 0.20 m
sideways at at most 0.025 m/s, with both finger contacts checked every
0.1 simulated seconds (80 samples).
The arm lowers onto the green destination table, verifies support before opening
the jaw, releases the cube, and retreats 0.10 m. No cube pose is prescribed and
no attachment/weld is introduced. The grasp/lowering sequence is scripted; carry translation can be derived from
an edited destination as described below. This is not perception, arbitrary-goal
planning or obstacle avoidance.

After one second of settling, success requires all of the following at each of
20 samples over two seconds:

- Positive contact impulse between the cube and `place-table`, and no contact
  impulses between the cube and robot links.
- The cube's rotated world-space bounds fit inside the tabletop footprint; its
  bottom is within 5 mm of the tabletop height (contact tolerance).
- Linear speed below 0.005 m/s, angular speed below 0.05 rad/s, and displacement
  from the start of the observation window below 1 mm.

The result panel reports the last completed demo. Reset, scene replacement and
recording import clear it; replay still verifies each recorded physics hash.
The headless smoke runs the complete sequence and verifies its recording.
Tests additionally move the destination out of reach to require a failure and
continue stepping after a successful release to check continued support.
Both demo buttons initialize their own presets; edited scenes do not change
their fixed scripts. The kinematic-base and simplified-finger limitations of
the grasp demo also apply here; real hardware has not been validated.

## Inspect pick/place one phase at a time

Choose `lekiwi_pick_place`, then **現在のシーンを初期化して準備** in
**工程ごとに実行**. This preserves scene edits, resets simulation time and the
recording, and restores neutral motor targets (zero clamped to joint limits).
The existing one-shot pick/place demo and CLI now use the same task executor.

- Play requests one 24-step batch (about 0.1 simulated seconds) per response.
  Pause stops new requests; an in-flight batch may finish. No wall-clock timer
  advances the physics. Reloading the page leaves the task paused in place.
- Single Step advances one batch; **次の工程まで** advances to the next phase
  boundary. A boundary request can include up to sixteen simulated seconds and
  completes before another command is handled.
- Progress shows the phase, batch count, and computed total (330 in the default scene). Phases are open,
  wrist roll, approach, close, lift, hold, carry, lower, release, retreat, settle,
  and placement verification. The same contact and stability checks apply.
- Failure zeros base velocity and retains the failed phase, reason and world
  state. Terminal tasks cannot advance. Export the operation recording to
  reproduce the physics up to failure; task progress itself is not saved in
  project/recording JSON and recording import clears the task.
- Manual stepping, drive, joint targets and saved-pose application are rejected
  while a task is attached. **終了して手動操作へ** clears the task, zeros base
  velocity and retains world state, joint targets and recording. It does not
  release a held object. Reset, model/project replacement and demos clear tasks.

This remains a scripted task, not an arbitrary-goal planner. Interactive pausing
freezes the simulated world; it is not a model of a real robot holding against
loads while external time continues. The default task's recording contains 330
batches; edited destinations change that count. Both verify using the existing
replay command.

Focused UI checks, including failure inspection and cancellation:

```bash
python3 scripts/test_workbench_task_browser.py --url http://127.0.0.1:8765 \
  --browser /usr/bin/chromium --output /tmp/workbench-task-checks
```


### Change the placement destination

In the `lekiwi_pick_place` scene, select **place-table** in the scene editor and
change its X/Z position, apply the edit, then use **現在のシーンを初期化して準備**.
The task panel previews the base translation, simulated duration and batch count.
For example, X = 0.68 m and Z = 0.254 m requests a diagonal transfer of
+0.08 m in X and +0.30 m in Z from the default pickup table. Negative Z transfers
are also supported. The one-shot demo button still restores the bundled scene;
use the phase runner for edited scenes.

The transfer uses table-center displacement. Duration rounds up to complete
24-step batches, and velocity is reduced to reach that endpoint without
exceeding 0.025 m/s. Each transport batch checks both finger contacts. Final
verification additionally requires the released block's horizontal center to
be within 15 mm of the destination center; the report shows that measured error.
The input recording includes the edited scene and derived velocity commands.

This version deliberately supports only equal-height fixed tables (height
matching tolerance 1 µm), translation distance 0.12–0.40 m, |X offset| ≤ 0.12 m,
and |Z offset| ≥ 0.12 m. Tables must be separated by at least 20 mm on at least
one horizontal axis. The destination footprint must fit the block's bounding
sphere diameter plus 20 mm. Invalid geometry is shown in the preview and rejected
before initialization replaces the current world. These bounds define the
implemented operating region, not hardware capability or collision-free motion.
There is no obstacle search, wheel-contact locomotion planning, or adaptation
of the original grasp pose to a relocated pickup table.

```bash
python3 scripts/test_workbench_task_browser.py --url http://127.0.0.1:8765 \
  --browser /usr/bin/chromium --output /tmp/workbench-destination-checks \
  --destination 0.68 0.254
```

### Evaluate the pick/place regression catalog

```bash
cargo run --locked -p robot_workbench --no-default-features -- \
  --evaluate-pick-place /tmp/lekiwi-evaluation-001
```

The output directory must be new and its parent must exist. The command never
reuses an existing directory. It writes `report.json` and a separate
`NAME.recording.json` for every completed or task-failed case. No GPU or hardware is needed.
The fixed `lekiwi-pick-place-v1` catalog includes nominal, diagonal and reverse
placements; a displaced block that must fail at the closing phase; and
unequal-height/out-of-range destinations that must be rejected before stepping.

The report separates physical completion from matching the expected outcome.
A healthy catalog has three completed cases out of four executed cases (75%
completion), one expected grasp failure, and two preflight rejections. All six
expectations must match for `passed: true` and exit status zero. Expected failures
must match the specified phase and reason; setup errors and replay divergence
always fail the evaluation. Setup/evaluation errors are counted separately in
`error_cases` and excluded from the completion-rate denominator. Rejected cases have no recording and
`replay_verified: null`, rather than claiming that a simulation was replayed.
The command continues after a case error and returns nonzero after writing the
report when any expectation is unmet.

Each executed case is replayed from its serialized recording and checked against
every recorded physics hash, including the failed grasp. Successful cases also
include the measured placement error, lift and stability report. Replay an
individual case with the existing `--replay` command and `--asset-root
assets/robots` (from the repository root). This small deterministic regression
catalog measures the bounded scripted task; it does not estimate real-world
reliability or coverage of arbitrary layouts.

Compare two saved evaluations without running physics or loading robot assets:

```bash
cargo run --locked -p robot_workbench --no-default-features -- \
  --compare-evaluations /tmp/lekiwi-evaluation-001/report.json \
  /tmp/lekiwi-evaluation-002/report.json
```

The baseline must pass. Both reports must use schema 1 and the same six unique
`lekiwi-pick-place-v1` cases, expectations, and planned destinations. Case order
is irrelevant. The comparator checks per-case outcomes and replay evidence even
if the top-level `passed` flag is stale. It fails if placement error (XZ Euclidean
norm) or maximum settled drift increases by more than 1 mm in any completed case.
It also fails if grasp lift or minimum hold lift decreases by more than 1 mm.
Completed placements must include finite, nonnegative grasp measurements; missing
grasp evidence cannot pass, even with a stale success flag. The same requirement
applies to the baseline. Exactly 1 mm of change is allowed using unrounded values.
Improvements pass. JSON comparison results go to stdout; regressions or invalid
reports produce a nonzero exit status. Inputs are limited to 1 MiB each and are
never modified. This checks reported evidence; use `--replay` to independently
verify the underlying recordings.

The CPU-only CI gate (`cargo run --locked -p xtask -- ci-headless`) also runs
this six-case catalog and exercises the comparison CLI. Run just this check with:

```bash
cargo run --locked -p xtask -- ci-lekiwi-evaluation
```

Python 3 (standard library only) is required. The check verifies all four executed
recordings, confirms identical reports pass, detects deliberately degraded
placement/replay and grasp evidence despite a stale success flag, rejects a missing case,
and checks that rerunning into an existing directory leaves evidence unchanged.
Each invocation retains a fresh `target/lekiwi-evaluation-runs/run-*/` directory
containing the evaluation report, recordings, CLI logs, comparison fixtures, and
`ci-summary.json`. These fixtures exercise regression detection; they are not
physical evaluation results. Each CLI invocation has a ten-minute timeout, with
partial logs retained. CI uploads the directory as the `lekiwi-evaluation`
artifact even when the gate fails (14-day retention). If an earlier headless
check fails before evaluation starts, no evaluation artifact is produced.
The gate compares every run against the reviewed reference at
`tests/golden/lekiwi_pick_place_v1/report.json`. A completed case whose XZ
placement error or settled drift increases by more than 1 mm, or whose grasp/hold
lift decreases by more than 1 mm, fails the gate,
even if its task still succeeds. The exact reference is retained as `baseline.json`,
with comparison logs and a baseline verdict in the job summary. The success
summary includes its SHA-256. This is a fixed repository reference, not the latest
successful CI artifact, and is never refreshed automatically. It does not claim
hardware validation. See the reference directory's README for provenance and the
review process for an intentional baseline update.

The gate also writes `summary.md` and appends it to the GitHub Actions job summary.
It separates the CI verdict from physical completion counts and shows each case's
outcome, expected-result check, replay status, and XZ placement error in millimeters.
Expected grasp failures remain labeled as failures; preflight rejections have no
replay claim. Setup failures or missing reports still produce a failure summary.

The summary's metric-change table shows baseline, current value, and signed change
for XZ placement error and settled drift, in millimeters. Positive changes indicate
larger error/drift; negative changes indicate improvements. Values are displayed to
three decimal places; the comparison gate uses unrounded measurements. Only cases
with completed placement evidence in both reports are listed, matched by case name.
Missing evidence is not shown as zero. Schema/catalog or destination changes suppress
the metric table with an explanation, since those measurements are not comparable.

The metric table highlights the case and metric with the least remaining margin:
`baseline + 1 mm - current`. Placement error and settled drift are both considered.
A positive margin is remaining allowance; zero is at the limit; a negative margin
exceeds it. The accompanying label uses unrounded values, so even an exceedance
that displays as `-0.000 mm` remains explicit. This is a placement-metric diagnostic,
not a replacement for the overall gate verdict (which also checks grasp lift, outcomes, and
replay). Missing or incompatible evidence produces no margin claim.

The failure and rejection diagnostics table shows recorded failure phases, completed
batch counts, task/setup reasons, and replay errors. It includes expected grasp
failures and preflight rejections as well as unexpected outcomes and replay failures
(even if placement completed). The outcome table above remains the source for whether
the expected result was met. Rejected cases are labeled `not executed`; missing
diagnostic fields are labeled `not reported`, without inventing a failure phase or
reason. Full recordings and CLI logs remain available in the evaluation artifact.

To inspect a downloaded artifact with the current summary formatter, select one
`run-*` directory (containing `ci-summary.json`) and run:

```bash
python3 scripts/test_lekiwi_evaluation.py --summarize /path/to/run-abc > /tmp/lekiwi-summary.md
```

This command needs only Python's standard library. It reads the retained CI verdict,
reports, and comparison log; it does not run Cargo, physics, or replay, modify the
artifact, or append to `GITHUB_STEP_SUMMARY`. Exit zero means rendering succeeded,
even when the recorded gate verdict is FAIL. A missing or invalid `ci-summary.json`
is an error (exit 2), rather than an inferred verdict. Missing optional evidence is
identified in the rendered summary. Running without arguments still runs the full
evaluation gate.
