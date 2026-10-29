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
The v1 catalog also requires at least 2 seconds of holding and 20 contact samples
for completed placements, independently of the height-change allowance. Missing
duration/sample evidence, shorter holds, or fewer samples fail even with a stale
success flag; invalid negative/non-finite durations are rejected as malformed input.
The baseline must meet the same contract. These are checks on recorded evidence;
replaying the underlying recording is a separate check.
Hold duration is measured from simulation-clock ticks, and contact samples count
successful batch-end pad-contact and lift checks. The resumable task accumulates
only batches that passed these checks; failed observations add no successful hold
evidence. Reading progress while paused does not advance either measurement. A
shortened hold window reports its observed duration/count rather than the normal
2-second/20-sample values. Samples describe batch-end observations, not continuous
contact at every physics step. Both the resumable task and standalone grasp check
derive these values from execution.
Transport contact samples likewise count successful Carry-phase pad checks rather
than planned batches. Placement samples and stable duration count only Verify
batches that pass support, robot-clearance, destination, and drift checks; duration
comes from simulation-clock ticks and excludes settling. Shortened verification
windows report their actual evidence. Failed observations add neither successful
samples nor stable time, and paused progress reads leave these measurements alone.
The normal placement window still yields 20 observations over 2.00000016 seconds.
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
placement/replay and grasp evidence despite a stale success flag, rejects incomplete
hold-contact evidence and a missing case,
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

A separate grasp-change table shows baseline/current grasp lift and minimum hold
lift, their signed changes, and the least remaining grasp margin. Its margin is
`current - (baseline - 1 mm)`: negative changes mean lower lift, and a negative
margin exceeds the allowance. Labels use the same unrounded lower-bound comparison
as the Rust gate, even if a tiny exceedance rounds to `-0.000 mm`. Missing or invalid
grasp evidence suppresses the grasp table with an explanation while leaving valid
placement metrics visible. Neither table replaces the overall gate verdict.

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

## Shared-world contact and torque arena (experimental)

The headless contact arena runs 1–8 LeKiwi + SO-101 agents in **one Rapier
world**. Bases and articulated links are dynamic: gravity, floor/wall contact,
robot–robot contact, and a shared dynamic box participate in the same solver.
There is no commanded base pose, planar velocity integration, grasp weld, or
position servo. All nine actuated joints per robot (three wheels and six arm /
gripper joints) accept direct torque. This is a physics/control experiment, not
a working cooperative pick/place policy or a hardware-calibrated digital twin.

Run the bundled two-agent episode:

```sh
cargo run -p robot_workbench --no-default-features -- \
  --run-contact-arena examples/116_robot_workbench/scenarios/contact_arena.json \
  > /tmp/contact-arena-observations.json
```

The JSON input contains `config.agents` (unique `id`, `position_m`, `yaw_rad`),
an explicit `config.self_collisions` flag, and `actions`. Each action specifies
`steps` (1–240) and a `torques_nm` map from agent ID to URDF **joint name** to
N·m. The example applies opposite shoulder-pan torques to `alice` and `bob`.
Output joint keys list the available names, including the three authored wheel
joint names. Inputs are limited to 4 MiB, 1,000 action frames, and 24,000 physics
steps (approximately 100 simulation seconds). The fixed step is 4,166,667 ns.

Every action advances all agents together. Unknown agent/joint IDs and nonfinite
torques reject the entire action before changing any command or advancing time.
Omitted agents/joints receive zero torque for that action; zero torque does not
hold position or stop motion. Commands clamp to the authored effort limit; the
three wheels, whose URDF has no effort rating, use an explicitly experimental
1 N·m ceiling. No velocity servo or implicit speed ceiling is added. The arm's
URDF ratings are simulation inputs, not verified hardware capabilities.

Each action returns the common simulation timestamp, stable physics-state hash,
per-agent base pose/velocity, joint position/velocity, capped commanded effort,
and backend-applied effort (not a torque-sensor model). Shared object positions
are included. Named contact pairs report positive normal solver impulses summed
over **all** physics steps in the action; these are N·s, not forces or final-step
contact flags. Observations are instantaneous simulator state without sensor
noise or latency. Agent IDs determine spawn order, so reordering the config's
agent array does not change a replay. Constructing a new arena resets the entire
shared world; per-agent resets and isolated parallel training workers are not
implemented by this interface. This mode is CLI-only, separate from the browser
Workbench's existing planar-base preview.

The supplied scenario enables self-contact (subject to connected-joint exclusions
in the backend). Setting `self_collisions` to false excludes only contacts within
each robot; it preserves collisions between different agents. CAD meshes use
AABB collision proxies and the grasp model's simplified pads. Those proxies can
overlap internally, creating self-contact at rest; enabling contact is **not** a
qualification of those shapes or of stable whole-body control. In the default CAD-based configuration, wheel rollers,
anisotropic rolling/sliding friction, calibrated wheel inertia, actuator dynamics,
and physical motor calibration are not reproduced. In particular, wheel torque
must not be interpreted as a validated omnidirectional velocity command.

Tests cover dynamic settling and floor contact, direct effort response and
per-agent isolation, clamping and zeroing, rejected-action atomicity, exact fresh
replay, self-contact, and robot–robot solver impulses in a gravity-driven drop
with within-robot contact disabled. The drop checks cross-agent collision routing;
it is not a coordinated driving or manipulation demonstration.

### Idealized passive-roller driving experiment

Set `config.idealized_omni` to `true` to select a separate experimental rolling
model. The default is `false`, preserving the CAD-cylinder arena and previous
pick/place behavior. This option changes collision geometry **and** base inertial
parameters; a difference from the default cannot be attributed to rollers alone.

```sh
cargo run -p robot_workbench --no-default-features -- \
  --run-contact-arena examples/116_robot_workbench/scenarios/contact_drive.json \
  > /tmp/contact-drive-observations.json
```

Each wheel has sixteen 8 mm-radius, 5 g spherical rollers on a 42.8 mm-radius
ring, giving a 50.8 mm outer radius. Each roller has a free continuous joint
about the tangent to the ring, with no motor or commanded effort. The ring is
centered on the authored drive axle and retains the wheel's axial offset.
Roller sphere inertia is `2 m r² / 5`. Contact and passive rotation are solved
by Rapier; the code does not inject a planar traction force or set a base pose.
The extra 48 degrees of freedom per agent make this mode substantially slower.
Only the original nine actuator joints accept commands; rollers are passive.

Overlapping base-part CAD AABBs are replaced by a 0.20 × 0.18 × 0.03 m chassis
collision box centered 0.04 m above the URDF base origin. The base link is 1.5 kg
with an isotropic 0.01 kg·m² inertia about that origin. Each wheel hub and its
fixed wheel body is 40 g; remaining base-part links use 1 g. Their diagonal
inertias are `mass × 0.0005 m²`. Original arm geometry and inertials are retained.
These are explicit experimental parameters, not measured LeKiwi mass properties.
CAD visuals remain authored and are not a collision-geometry rendering.

The scenario starts with the arm facing away from the wall, settles for 0.5 s,
drives toward +world X using wheel torques `[+1, -0.5, -0.5]` N·m in authored
wheel order (`-2`, `-1`, unsuffixed), then sets all torques to zero for 0.5 s.
There is no initial wall contact, velocity target, position tracking, or active
braking controller. A local run advanced about 16 cm in X and 10 cm laterally,
produced positive roller/wall impulses, and settled below 0.01 m/s planar speed.
Tests require forward travel, wall contact, nonzero passive roller velocity,
no wall crossing, and settling, as well as exact fresh replay of a short run.
This establishes torque-driven contact motion in this model; it does not
establish accurate straight-line tracking or free-space braking performance.

The spherical ring has a scalloped outer profile and can stick under low torque.
The arm can also touch the floor when unpowered. Actual barrel roller geometry,
bearing losses, realistic mass distribution, calibrated friction, and a torque
feedback controller remain necessary before treating this as a realistic kiwi
base. The driving scenario disables same-robot collision to isolate rolling;
robot/environment and cross-agent collisions remain enabled. Self-contact of the
new model is not qualified by this driving test.

### Wheel-speed feedback with explicit torque output

The sampled PI controller accepts wheel angular speeds and computes direct joint
torques from completed joint-velocity feedback at every 4,166,667 ns physics
step. It does not install backend velocity motors, command a base velocity, or
edit poses. Its state is independent for each agent and wheel.

```sh
cargo run -p robot_workbench --no-default-features -- \
  --run-wheel-control examples/116_robot_workbench/scenarios/wheel_feedback.json \
  > /tmp/wheel-feedback-observations.json
```

The input contains the arena `config`, `gain_nm_s_per_rad` (Kp), optional
`integral_gain_nm_per_rad` (Ki, default zero), and `phases`. Each phase specifies
`steps` and `wheel_speeds_rad_s`, a map from agent ID to three speeds in authored
wheel order (`-2`, `-1`, unsuffixed). Each speed must be finite and within
±20 rad/s; gains must be finite and in [0, 1]. Those input bounds are not a
stability guarantee. Each phase permits 1–2,400 steps and an episode permits at
most 1,000 phases and 24,000 total steps. All IDs, setpoints, and gains are
validated before stepping the world.

For each wheel, `error = target_speed - measured_speed`, and the controller
integrates `Ki * error * dt` in torque units. Torque and integral memory are
bounded by the actuator's effort ceiling. Conditional integration prevents
further winding up when the error would push the actuator farther into
saturation, while permitting the integral to unwind. Every new phase resets
integral memory, including the drive-to-brake transition. Use one phase for
continuous integration over a sustained setpoint.

An explicit `[0, 0, 0]` target enables zero-speed feedback. Omitting an agent
instead commands zero wheel torque (coasting); it is not active braking. Arm
joints default to zero torque; the optional `arm_hold` configuration below
keeps selected joints under direct-torque posture control. Existing `--run-contact-arena`
remains the interface for arbitrary direct joint torque sequences.

Output samples contain phase index, completed phase-step count, and an arena
observation every 24 physics steps, plus a final shorter window when needed.
Contact impulses are accumulated over every step in that window, not sampled
only at its end. Joint velocity, command, and backend-applied torque describe
the last step; the command was computed from that step's **preceding** feedback.
Feedback is instantaneous simulator state, not a hardware encoder/latency model.

The example settles, drives for 1 s, then requests zero speed for 1 s in open
space away from the wall and shared box. With Kp=0.03 and Ki=0.1, a local run
advanced about 12 cm in X, drifted about 3 cm laterally, and settled below
0.01 m/s planar speed and 0.1 rad/s wheel speed. The driven wheel speeds still
lag their `[12, -6, -6]` rad/s targets: this is not a qualified speed-tracking or
heading controller. A higher Kp=0.3 trial oscillated near the torque limits.

The matching coast counterfactual shares the exact drive states, changing only
the final phase's speed map to `{}`. Both cases stop quickly because the model
has substantial ground/arm friction; the measured final displacements during
stopping differed by only about 0.15 mm. Do not infer a useful braking-distance
advantage from this case. Tests check free-space motion, final speed, torque
bounds, active-versus-coast command semantics, identical pre-stop states, and
independent multi-agent commands with exact replay. Motor calibration, robust
gain tuning, arm posture control, and body-frame heading feedback remain open.

### Keep the arm off the floor while driving

`--run-wheel-control` also accepts an optional episode-level `arm_hold` map
from agent ID to arm-joint name to a PD target. Each target specifies
`position_rad`, `stiffness_nm_per_rad`, `damping_nm_s_per_rad`, and
`effort_limit_nm`. At every physics step the controller applies
`Kp * (target - measured_position) - Kd * measured_velocity`, capped by that
joint's configured effort ceiling. It uses direct effort actuation, not a
backend position motor, weld, or pose correction. Arm targets persist through
all wheel phases, including coasting and zero-speed braking. Omitted arm joints
remain unpowered; wheel names cannot be assigned to this controller.

```sh
cargo run -p robot_workbench --no-default-features -- \
  --run-wheel-control examples/116_robot_workbench/scenarios/wheel_arm_hold.json \
  > /tmp/wheel-arm-hold-observations.json
```

The supplied scenario holds all six arm joints near zero radians while using
the same wheel-speed phases as `wheel_feedback.json`. Targets must be inside
the authored joint limits. Gains must be finite and nonnegative, with Kp ≤20
N·m/rad and Kd ≤1 N·m·s/rad; the per-joint effort ceiling must not exceed the
model's actuator limit. These numeric bounds do not guarantee stable tuning.
All targets and agent IDs are validated before simulation advances.

In the local run, maximum sampled posture error was approximately 0.15 rad and
no positive arm/floor contact impulse was reported during settling, driving,
or braking. Only the wheel rollers supported the robot on the floor. The same
initial scene with the arm unpowered produced gripper/floor contact during
settling. Tests compare these cases, require sampled errors below 0.2 rad, check the
configured torque ceilings, and verify persistent independent arm holds for
two agents through wheel-command changes and exact replay.

The held-arm run moved roughly 6 cm in X and 9 cm laterally, then settled below
0.01 m/s planar speed. This is not straight-line tracking. Removing arm support
also did not establish a meaningful braking advantage: active and coast stop
displacements differed by only about 0.3 mm in this model. The passive spherical
roller profile and contact friction remain significant limitations. PD holding
has gravity-dependent position bias; there is no gravity feed-forward,
trajectory planner, grasp policy, or whole-body stabilizer in this controller.

## Independent environments for learning

`--serve-vector-arena CONFIG.json` keeps multiple **independent physics worlds**
alive for an external learner. Each environment has its own arena, robot
entities, object/contact state, torque commands, and simulation clock. Identical
agent IDs and overlapping coordinates across environments do not imply shared
bodies or collisions. The default vector example has one robot in each of two
worlds. Multiple agents inside one environment still share that environment's
physics; that is separate from independent training lanes.

```sh
cargo run -p robot_workbench --no-default-features -- \
  --serve-vector-arena examples/116_robot_workbench/scenarios/vector_arena.json
```

The config contains `environments`, an ordered array of 1–16 arena configs, and
`workers` (1–8). Each environment can opt into the idealized roller model.
Stepping uses disjoint CPU worker chunks, with at most `workers` scoped threads;
results remain in environment-index order regardless of scheduling. This is
CPU parallelism, not GPU-vectorized physics or a throughput qualification.

The process writes one JSON response per line and flushes it immediately. It
first emits `{"kind":"ready","observations":[...]}` at zero simulation time.
Send JSON-line requests on stdin:

```json
{"op":"step","actions":[{"steps":24,"torques_nm":{"alice":{"shoulder_pan":0.02}}},{"steps":24,"torques_nm":{}}]}
{"op":"reset","indices":[0]}
{"op":"observe"}
```

`step` requires exactly one direct-torque arena action per environment. Choose
identical step counts for a common control cadence; different counts are allowed
and each observation retains its own timestamp. The reply is
`{"kind":"step","results":[{"status":"stepped","observation":...},...]}`.
All action IDs, values, lengths, and step counts are validated before any lane
advances. Input errors return `{"kind":"error","error":"..."}` and permit
another request; invalid batches do not partially advance earlier lanes.

`reset` reconstructs only the selected environment indices from their original
configs, clearing their commands, physics state, and clocks. Returned indices
and observations follow the requested order. Other environments remain untouched.
Duplicate/out-of-range indices and construction failures leave all old worlds
in place. There is no implicit reset when stepping. `observe` neither advances
time nor synthesizes a new contact interval; its contact lists are empty.

A normal backend error during step returns `status:"failed"` for that lane;
successfully stepped peers retain their results. This is not rollback of a
physics failure. Reset failed lanes before the next step/observe request.
Worker panics terminate the process rather than pretending a valid transition
was produced. Request lines and the startup config are bounded to 4 MiB;
an oversized request terminates the process, while EOF shuts it down normally.

Tests compare sequential and parallel complete observations, including roller
contact worlds, and compare a lane against a separately stepped reference world.
They also verify selective-reset isolation, exact fresh-reset replay, and batch
validation before mutation. The JSON-lines smoke covers ready, step, reset,
observe, and recovery after an invalid request.

This interface supplies independent transition dynamics and reset semantics.
It does not yet define task rewards, success/termination/truncation conditions,
auto-reset, domain-randomization seeds, Gymnasium spaces, or a training algorithm.
Resets are deterministic copies of the supplied per-environment configuration.
The learner must provide torque actions; the standalone wheel/arm feedback runner
is not implicitly running inside these worlds. Those task/controller wrappers
can be added without mixing the underlying physics worlds.

### Gymnasium shoulder-target task

`python/rne_gym.py` adds `ShoulderVectorEnv` (Gymnasium 1.2.3 `VectorEnv`) and
`ShoulderEnv` (single-world `Env`) over that JSON-lines service. This is an
initial torque-learning task, not a pick-and-place policy or hardware model
qualification. Each lane must contain exactly one LeKiwi + SO-101 robot.
The world config can still select idealized rollers; the sample uses CAD proxies.

```sh
cargo build -p robot_workbench --no-default-features
python -m venv .venv-gym
.venv-gym/bin/pip install -r examples/116_robot_workbench/python/requirements.txt
.venv-gym/bin/python examples/116_robot_workbench/python/rollout.py \
  target/debug/rne-workbench --steps 60
.venv-gym/bin/python -m unittest discover -s examples/116_robot_workbench/python -v
```

`RNE_WORKBENCH` overrides the test binary path. The rollout collects seeded random
transitions and resets finished lanes; it does not train a policy or demonstrate
learning convergence. Use `ShoulderEnv` with learners accepting Gymnasium `Env`,
or `ShoulderVectorEnv` with learners supporting Gymnasium's **Disabled** autoreset
mode (this is not the Stable-Baselines3 VecEnv protocol).

The nine-dimensional action is normalized direct torque in `[-1, 1]`, ordered by
`env.joint_names` (sorted joint names). Each component is multiplied by that
joint's reported effort cap. There is no implicit arm holding or wheel controller.
Invalid shapes, nonfinite actions and out-of-range values are rejected before
sending the batch. Observations are float64 vectors in this order:

- Nine joint positions (rad), then nine joint velocities (rad/s), in joint order.
- Base position (m), quaternion (xyzw), and linear velocity (m/s).
- Shoulder-pan target (rad), consecutive hold count / 10, remaining horizon fraction.

The observation is partial state: it excludes passive roller states, contact
solver caches and base angular velocity. Continuous physical values use unbounded
Box limits rather than clipping physical measurements. Normalization for a
particular learner remains the caller's responsibility.

Every reset samples a shoulder-pan target with magnitude uniform in `[0.15, 0.3]`
rad and uniformly chosen sign. Integer seeds become `seed + lane_index`; a list
can specify per-lane seeds. Only reset lanes consume their own random streams.
Physics resets to the original config; this adds target randomization, not physics
or domain randomization. Default actions advance four physics ticks (~1/60 s).
The task reward per control step is
`-angle_error_rad² - 0.001 * mean(normalized_action²) + success - fallen`.
Success requires ten consecutive control observations within 0.05 rad and below
0.1 rad/s. Fall means base height below 0.02 m or the robot's local +Z axis more
than 60 degrees from world +Y. Success/fall set `terminated`; the default 200-step
limit sets `truncated` only if no termination occurred. The hold duration is
measured at control boundaries, so changing `physics_steps` changes its duration.

Call `reset()` before stepping. After any lane finishes, call
`reset(options={"reset_mask": terminated | truncated})` before the next batch.
Reset returns all lane observations; unselected lanes retain physics, targets,
counters and random streams. Terminal observations are returned directly by step
and are never overwritten by autoreset. `info` contains per-lane targets,
simulation timestamps and hashes; step additionally returns angle errors,
`is_success` and `fallen`. Physics/protocol errors raise and require resetting all
lanes (recreate the environment if the process exited), rather than supplying
fabricated terminal transitions. Close the environment
(or use the vector environment as a context manager) to reap the child process.

Tests run Gymnasium's single-environment checker against the real Rust process,
compare seeded serial/parallel transitions and hashes, check partial reset and
truncation, and use controlled observations for success/fall/backend-error cases.
A seeded direct-torque PD baseline also reaches success in a real contact world;
this demonstrates task feasibility for that case, not a learned policy.

### Parallel policy search and held-out evaluation

`python/train_shoulder.py` runs cross-entropy method (CEM) policy search using
only NumPy and Gymnasium. It learns **two shoulder-pan PD gains**, not a neural
policy or full-body controller. The policy explicitly applies fixed PD holding
to the other arm joints and zero wheel torque; those fixed gains come from the
existing arm-hold scenario. The environment itself still accepts nine torques.
This keeps the first learning experiment small enough to reproduce on CPU.

```sh
.venv-gym/bin/python examples/116_robot_workbench/python/train_shoulder.py \
  target/debug/rne-workbench /tmp/shoulder-search.json
.venv-gym/bin/python examples/116_robot_workbench/python/evaluate_shoulder.py \
  target/debug/rne-workbench /tmp/shoulder-search.json --check
```

The output path must be new. Defaults are six independent candidate worlds,
four generations, optimizer seed 42, and two training target seeds (11, 29).
All candidates in a generation see exactly the same targets and starting world.
The search samples bounded log gains, fits the best half, and retains the
incumbent. Initial shoulder gains are 0.3 Nm/rad and 0.02 Nm·s/rad; shoulder
torque is capped at 1 Nm by the policy. Control runs every physics tick, with a
300-tick horizon (~1.25 seconds); success still requires ten control observations.

The selected policy is frozen before evaluating held-out target seeds
101, 202, 303 and 404. Initial and learned policies run in paired independent
worlds. When one candidate finishes early, subsequent episodes in its lane are
excluded from its score while peers finish. Fitness is undiscounted episode
return under the existing reward, so both tracking error and time to success
matter. Training targets are deliberately few: these results do not establish
robustness to varied hardware, contacts, world layouts or unseen target ranges.

The committed example `python/results/shoulder_search.json` records the complete
world, fixed controller, exact learned log parameters, seed sets, candidate
scores, episode lengths, success/fall outcomes, final errors and physics hashes.
`evaluate_shoulder.py --check` reloads the policy and requires exact equality of
all saved evaluation records, including hashes. Exact replay is checked on this
runtime; cross-platform floating-point equivalence is not promised.

In the recorded CAD-proxy experiment, initial and learned policies both succeeded
on all four held-out targets. Mean return improved from -3.2276 to -0.7511, and
mean absolute final error fell from 0.04062 to 0.01398 rad. The selected shoulder
gains were approximately 0.86183 Nm/rad and 0.05086 Nm·s/rad. These are simulation
results from a structured feedback policy search, not measured robot performance.
Tests additionally verify seeded search replay and that resetting finished
candidate lanes cannot contaminate the original episode return.

### Simultaneous shoulder and elbow pose learning

`ArmPoseVectorEnv` and `ArmPoseEnv` extend the torque task to three simultaneous
joint targets: `shoulder_pan`, `shoulder_lift`, `elbow_flex`, in that order.
Each target independently chooses a sign and magnitude uniform in `[0.1, 0.3]`
rad, within the authored URDF limits. The 33-dimensional observation has the same
joint/base fields as the shoulder task, followed by these three targets, the
shared hold count / 10, and remaining horizon fraction. `target_rad` and
`angle_error_rad` in vector info have shape `(num_envs, 3)`.

All three angles must be within 0.05 rad and all three speeds below 0.1 rad/s for
ten consecutive control observations to succeed. An excursion of any one joint
resets the shared count. Reward uses the **mean squared error across all three
joints**, with the same action penalty, success bonus, fall penalty and timeout
semantics as the single-joint task. Single-joint observation shapes, seeds and
saved-policy replay remain compatible.

```sh
.venv-gym/bin/python examples/116_robot_workbench/python/train_arm_pose.py \
  target/debug/rne-workbench /tmp/arm-pose-search.json
.venv-gym/bin/python examples/116_robot_workbench/python/evaluate_shoulder.py \
  target/debug/rne-workbench /tmp/arm-pose-search.json --check
```

The replay command detects the task from the report. Training learns six log
parameters: proportional and damping gains for each of the three joints, in
that order. Wrist/gripper holding gains and the 1/2/1.5 Nm respective shoulder
pan/lift/elbow torque caps are fixed in the policy; wheels coast. The optimizer
uses eight independent CPU worlds, six generations, optimizer seed 42 and four
training target seeds (11, 29, 47, 83). It freezes the selected controller before
paired evaluation on eight held-out target seeds (101, 202, 303, 404, 505, 606,
707, 808). Each action lasts one physics tick, with a 480-tick (~2 s) horizon.
The ten-observation hold is therefore only about 42 ms, not a long-duration
stability qualification.

These are joint-space pose tasks with CAD contact proxies. They do not constrain
end-effector position, prohibit arm/floor support, randomize robot dynamics, or
establish collision-free manipulation. Full-body reinforcement learning,
cartesian reaching and grasping remain separate work. Tests cover all-joint
success/reward semantics, Gymnasium compliance, selective target reset,
serial/parallel policy replay and task-aware saved-controller replay.

The committed `python/results/arm_pose_search.json` is a **mixed result**, not a
qualified improved controller. On the eight held-out targets, initial gains
succeeded on 0/8 and selected gains on 3/8. Mean absolute final error decreased
from 0.06102 to 0.03865 rad, but mean return worsened from -4.8794 to -12.4871.
In particular, target seed 505 returned -78.0597 with the selected gains. All
non-successes reached the time limit; neither policy triggered the fall rule.
A small final error therefore does not establish a good trajectory or stable
holding. The selected policy was not retuned on these evaluation targets.
The result exposes limited generalization from four training targets and is
retained so later changes can be compared against the full outcome, not just
success rate. The report replays exactly, including per-joint errors and state
hashes, on the tested runtime.

### Bounded-reference PID and worst-case-first selection

The seed-505 regression prompted a controller change rather than another search
for higher PD gains. `arm_controller.py` holds a zero reference for 120 physics
ticks, then advances each of the three joint references at no more than 0.5
rad/s. PID uses the same direct torque interface and existing torque caps.
Integral gains are `[0.4, 4, 3]` Nm/(rad·s), with integral torque contributions
bounded to `[0.3, 1, 0.7]` Nm, in pan/lift/elbow order. Conditional integration
prevents accumulating error into saturation while permitting unwinding. All
controller timing uses the fixed simulation timestep (4,166,667 ns), not wall
clock. Wrist/gripper use the same fixed holding controller and wheels coast.

Controller reference, integral and elapsed-step state are owned per lane.
The evaluator resets only the controller lanes whose physical worlds it resets,
including lanes that finish while peers are still running. Controller state is
part of the policy, not added to the environment observation or reward. The
three-joint task's targets, torque limits, reward and success rule are unchanged.

```sh
.venv-gym/bin/python examples/116_robot_workbench/python/train_stable_arm.py \
  target/debug/rne-workbench /tmp/stable-arm-search.json
.venv-gym/bin/python examples/116_robot_workbench/python/train_stable_arm.py \
  target/debug/rne-workbench /tmp/stable-arm-search.json --replay
.venv-gym/bin/python examples/116_robot_workbench/python/diagnose_arm.py \
  target/debug/rne-workbench /tmp/stable-arm-search.json /tmp/seed-505.json --seed 505
```

CEM now selects lexicographically by success count, worst episode return, then
mean return. This avoids trading away a completed target for a good average.
The six-gain search uses six candidates, four generations, optimizer seed 42,
and development/training seeds 11, 29, 47, 83, 505, 909. Seed 505 is deliberately
a regression target used in development, **not held-out evidence**. Evaluation
uses fresh seeds 1001–1008 only after policy selection is frozen, comparing the
original PD, previous learned PD and new selected PID on the same targets.

`diagnose_arm.py` records target joint positions, speeds, errors and commanded
torques every twelve control steps and at termination. Peak metrics accumulate
over every control step, not just recorded samples. It also retains terminal
hashes, returns, lengths and outcomes. In the previous PD controller's seed-505
trajectory the shoulder pan reached 1.669 rad and shoulder lift speed peaked at
19.679 rad/s; a small final error had hidden the poor transient.
The reference ramp and integral correction address that transient and residual
error respectively, but do not provide a general stability guarantee or a
calibrated hardware controller. These experiments still use CAD contact proxies,
a short (~42 ms) success hold, and a two-second horizon.

The saved `python/results/stable_arm_search.json` compares all three controllers
on the new, held-out seeds:

| Metric (8 targets) | Original PD | Previous learned PD | Bounded-reference PID |
| --- | ---: | ---: | ---: |
| Successes | 0/8 | 7/8 | 8/8 |
| Mean episode return | -16.5015 | -13.1025 | -8.1353 |
| Mean final absolute joint error (rad) | 0.06144 | 0.08382 | 0.01582 |
| Mean episode length (ticks, including failures) | 480 | 365.25 | 348.25 |

This is improvement on this eight-target simulation evaluation, not broad
robustness evidence. The controller structure and selection objective changed
together; this comparison does not isolate their individual contributions.
The paired seed-505 regression trajectory is retained separately in
`python/results/stable_arm_seed_505.json`. Replay validates every saved evaluation
record, including terminal state hashes. Unit/integration coverage also checks
reference slew limits, integral anti-windup, selective controller resets,
lexicographic candidate selection, and repeated physical-episode replay after
resetting controller state.
For that development regression, the new policy succeeds at tick 361 and reduces
the peak absolute shoulder-pan angle from 1.66935 to 0.13475 rad. Peak lift and
elbow velocities still reach 12.85 and 16.55 rad/s during the episode, so this
should not be read as hardware-safe motion or a general joint-speed guarantee.

### Sustained-hold audit of frozen policies

The Gymnasium environments accept `hold_steps` (default 10). Success now uses
that configured count of consecutive qualifying control observations; the
observation's hold-progress component is normalized by the same count. Every
joint must remain within 0.05 rad and below 0.1 rad/s, and any violation resets
progress to zero. Existing callers retain the ten-step behavior.

```sh
.venv-gym/bin/python examples/116_robot_workbench/python/audit_arm_hold.py \
  target/debug/rne-workbench \
  examples/116_robot_workbench/python/results/stable_arm_search.json \
  /tmp/arm-sustained-hold.json
.venv-gym/bin/python examples/116_robot_workbench/python/train_stable_arm.py \
  target/debug/rne-workbench /tmp/arm-sustained-hold.json --replay
```

This audit freezes all three saved controllers and uses the same eight evaluation
targets; there is no retraining, candidate reselection or new held-out target
claim. It requires 480 consecutive observations at one physics tick per action
(2.00000016 s) and allows 2,400 ticks (~10 s) to reach that condition. The audit
report saves the hold requirement, time budget, outcomes, full source controller
settings and terminal physics hashes. It can be replayed by the existing
stable-arm replay command. Episode returns across different hold requirements
are not directly comparable because the episode lengths and termination times
change. The audit checks state at control boundaries, not continuous-time
interpolation between physics steps; it ends when the required hold is achieved.

The saved `python/results/arm_sustained_hold.json` records:

| Frozen controller | 2-second holds achieved | Mean final absolute error (rad) |
| --- | ---: | ---: |
| Original PD | 0/8 | 0.05639 |
| Previous learned PD | 3/8 | 0.03497 |
| Bounded-reference PID | 8/8 | 0.00829 |

The bounded-reference PID completes the required hold after 745–1,033 ticks
(~3.10–4.30 s), including the two-second hold itself. All unsuccessful episodes
reach the ten-second limit; none trigger the fall rule. The previous learned PD
had passed 7/8 under the ten-tick rule, showing why a brief successful observation
was not sufficient evidence of sustained holding. Results remain confined to
these eight target configurations in the existing uncalibrated simulation.
