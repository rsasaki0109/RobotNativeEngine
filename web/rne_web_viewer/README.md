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

**Walking motion ON** uses kinematic foot planting while moving or turning.
The support ankle keeps its world position and orientation; the other foot
follows a raised swing arc. URDF-based leg IK matches the actual base displacement
(including reverse motion and turns), with a 4 cm crouch for reachability. On stop,
the swing settles and both feet hold their positions. Toggle walking off for the
exact manual pose. Nonzero leg sliders also select manual leg poses; zero them or
reset to resume foot planting. Large pose jumps in accelerated comparisons reset
the foot anchors. Arm swing and waving remain pose animations. **Wave** raises the left arm and waves for 180
preview ticks, then restores the pose; another press restarts the gesture.
All composed joint coordinates are clamped to the original URDF limits.
Foot planting reduces visible skating; it does not add contact forces, balance
control or a physical biped controller. Recorded replays retain the fixed standing
pose so previous animation cannot alter recorded joint coordinates.
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


### Representative task comparisons

The task menu adds a 0.96 m corridor, a crossing obstacle, escape from a U-shaped
wall, and an ordered three-goal patrol. Each task explains its difficulty and what
to compare. The moving box follows a seed-phased triangle wave at 0.4 m/s using
fixed simulation ticks; the costmap and global plan refresh every 10 ticks (3 Hz).
Collision checks interpolate both robot and moving box through each step. This is
reactive navigation with current obstacle positions, without motion prediction or
waiting/recovery after planning failure. Failures are retained as outcomes; the
crossing scenario can fail for every pair. Static corridor, U-wall and patrol
geometry is fixed, so changing seed does not randomize these three tasks.

A four-pair comparison keeps actual traces in both the overhead map and 3D view:
green = straight/Pure Pursuit, orange = straight/DWA, purple = A*/Pure Pursuit,
cyan = A*/DWA. Legend checkboxes toggle individual traces without changing results.
Only the current comparison's traces are overlaid; a new run or editor session
cannot mix trajectories from different conditions. Obstacles and the blue planned
path depict the current/last pair's simulation time, not a synchronized replay.
The result table shows distance and minimum obstacle clearance directly; the
summary counts successes among completed pairs (one trial per pair, not a
multi-seed success-rate estimate). Patrol success requires all three goals, and
results include the reached-goal count. The planned-distance detail is the most
recent global plan, while travelled distance and RMSE accumulate over the run.

Run `python scripts/test_g1_experiments_browser.py --url http://127.0.0.1:8084/
--output work/comparison-browser` (on one line) against a built, locally served
bundle to check the catalog, comparison traces, visibility controls, distinct
U-wall outcomes, patrol completion and moving-obstacle single steps in real WASM.


### Navigation recovery and recorded-time replay

The optional recovery checkbox is captured with the next run's configuration,
including all four comparison pairs. A global planning failure enters Waiting:
the kinematic base stops immediately, simulation time and moving obstacles keep
advancing, and planning retries every 15 ticks (0.5 s). After 300 consecutive
waiting ticks (10 s), failure is terminal; the existing overall 60 s limit also
applies. A moving obstacle can still collide with a waiting robot. Contact and
control failures remain terminal. There is no obstacle-motion prediction. Waiting
seconds and the recovery policy are retained in results. With seed 42, the
straight/Pure Pursuit crossing example succeeds after 3 s of waiting. The checkbox
defaults off, so the original comparison behavior remains available.

Completed navigation runs record fixed-tick poses, obstacle locations, the current
plan, remaining patrol goals, metrics and state. The navigation replay selector
and timeline display one run's recorded frames, typically every 0.1 s plus state
changes and the terminal frame. Navigation records planar base position/yaw, not
walking-joint animation; replay uses a fixed stance, independent of redraw count.
The timeline selects actual recorded samples, so
its displayed time matches the visual state. It never reruns a controller or
changes measured results. Other comparison traces are hidden while replaying to
avoid mixing times. Starting another run, editing, manual control or reset exits
replay. A completed current run must exist to enable the replay selector.

### G1 manipulation experiments

Open **MANIPULATION** in the comparison panel for left-hand reaching, pick/place,
placement near a blocking box, or an unreachable target. Three planners (direct
joint goals, fixed elevated task-space waypoints, and RNE RRT-Connect) combine with
two servos
(velocity-limited proportional and acceleration-limited proportional). All run at
30 Hz with joint speed limited to 0.8 rad/s; the smoothed servo also limits its
command acceleration to 1.6 rad/s². The task uses the navigation seed for a small,
deterministic variation of the pick configuration. Selection changes apply only
to the next run. Navigation and manipulation are mutually exclusive.

The model is imported from the vendored G1 URDF and uses `rne_robot`'s real
`KinematicModel` FK and active-joint damped least-squares IK. Only the five left
arm joints move; a fixed tool frame is added 0.055 m along the rubber hand's local
X axis. Viewer joint order is explicitly mapped from model DOF order. A headless
test checks that FK and the rendered robot's actual tool position agree. IK is
position-only and respects the source joint limits; out-of-range targets produce
an explicit IK failure rather than a fabricated successful animation.

Orange is the object and green is the target. Pick/place uses a distance-gated
kinematic attachment (within 0.025 m), keeps the measured offset in the tool frame
while carrying, then releases without teleporting the object onto the target. Placement
must finish within 0.025 m. This is not force-based grasping: gravity, finger
contact, object orientation, torso/self-collision, and balance are not simulated.
The blocking-box check samples the swept left forearm/tool (0.018 m radius) and
held object (0.0225 m radius) against the 0.05 m box. The fixed elevated waypoint
planner does not search for a collision-free path. In the seed 42 blocking scene,
these original four pairs contact the box, while the two RRT-Connect pairs reach
the placement target. Failures are preserved and can be inspected in replay.

Arm results include final tool/object error, active-arm-joint target RMSE and
elapsed simulation time. RMSE is absent for failures before any control step.
Arm replay records joint coordinates, object position, attachment state and task
stage, so the robot and held object remain synchronized when seeking. Completed
comparison traces are colored consistently with the navigation palette. Replay
selects a single pair's trace and never changes its measured results.

Verification:

```bash
python scripts/test_g1_recovery_manipulation_browser.py --url http://127.0.0.1:8087/ --output work/recovery-browser
```

Run against a built static bundle served locally. The test exercises recovery,
recorded-time navigation replay, manipulation pause/step, all six pick/place
pairs, RRT selection and blocking-scene outcomes, grasp/release replay, contact
and unreachable failures, manual takeover,
and mobile touch, using actual WASM and Chromium WebGPU.


### Collision-aware arm planning

The arm menu adds **RRT-Connect** from the existing `rne_planning` crate. The
comparison now runs 3 planners × 2 controllers. It first checks the direct joint
segment, then uses seeded bidirectional search when that segment is blocked.
Each task stage has a 600-iteration search limit, a 0.20 rad extension step and
12 collision-check subdivisions per edge. Search iteration counts are shown in
results; CPU planning time is excluded from simulated execution time. A search
limit, colliding start, colliding goal, and IK failure remain distinct outcomes.
Seeds are captured with the run; the stage contributes a fixed seed offset.

The scene uses the same URDF-derived model and a torso-to-left-tool planning
group, holding every other joint fixed. Forearm and tool are conservative capsule
and sphere proxies (0.026 m radius); the attached object uses a 0.0255 m sphere at
its measured tool-local offset. These proxies are checked against the actual
0.05 m scene box. No full-body/self-collision or force-based grasping is added.
Execution still checks the original forearm/tool and held-object samples every
tick; a valid plan does not bypass collision detection or guarantee controller
tracking. The object follows a rigid tool-local offset consistently in planning,
execution and replay, rather than a world-fixed displacement.

The resulting joint waypoints become controller targets. Blue markers show the
planned tool path; colored markers show measured motion. Each replay frame keeps
its own immutable plan and search count, so seeking an earlier grasp stage does
not show a later transfer plan. The two RRT/controller pairs use pink and gold,
with matching markers in the results table.

Headless tests verify that the blocking transfer's direct segment is invalid,
every RRT edge passes a denser 24-subdivision check, placement succeeds, repeated
runs produce identical frames, and colliding endpoints are rejected explicitly.
The browser check also selects RRT directly, compares all six pairs on the same
blocking task, and seeks its stored plan in replay.

### 複数seedの腕ベンチマーク

「3 seedで18試行」は現在のタスクとseedを固定し、seed、seed+1、seed+2（u32の末尾では0へ循環）の各条件で3計画×2制御を実行します。実行中の条件変更は次の開始時に反映します。成功率の分母は完了した試行のみです。平均時間（シミュレーション秒）と平均手先移動量（各tickのFK位置間距離の合計、m）は成功した試行のみで計算し、成功なしは「—」です。未完了と中断は成功に数えません。終了すると各seedの試行を個別にリプレイできます。画面の軌跡は現在のseedのみを重ねます。これは運動学的評価で、力学・計算実時間の評価ではありません。

「腕の結果をCSV保存」で完了した試行をUTF-8 BOM付きCSVに保存できます。単独実行・6手法比較・18試行ベンチマークに対応します。schema_version=1、実行モード、全体状態（running/paused/interrupted/completed）、予定数・完了数、開始時に固定したタスク・seed・計画・制御、結果、時間 s・手先移動 m・最終誤差 m・関節目標RMSE rad・探索回数を含みます。RMSEが未定義なら空欄です。未完了の試行は行を作りません。CSVはクリック時点の記録で、リプレイ位置や後から変更した設定には影響されません。全試行が完了したCSVのみ、3 seed全体の比較に利用してください。

### 移動と腕操作をつないだ棚の運搬タスク

「移動 × 腕操作 · 棚から運んで配置」で、棚への移動・整列、把持・7cm持ち上げ、配置先への運搬・整列、配置の4段階を連続実行します。世界座標の物体をそのまま次段階へ引き継ぎ、ロボットや物体の位置を目標へ瞬間移動させません。開始時の移動・腕の手法とseedを固定し、設定変更は次回から適用します。移動4組比較では2計画×2制御を評価し、腕の計画・制御は固定です。

各移動区間は既存の直線/A*とPure Pursuit/DWAを使い、到達許容差3.5cm、停止後の向き合わせは最大0.8rad/sです。運搬中は腕を固定し、前腕・手先・荷物の水平範囲を含む半径に合わせて障害物を保守的に膨張させます。腕の段階は既存の位置IK・関節計画・制御を使い、左前腕・手先・荷物と棚板・茶色の障害物を検査します。検査対象は棚の支柱や全身を含みません。重力、物理的な支持、把持力、二足歩行の安定性は模擬しません。

結果は完了段階数、失敗理由、段階ごとの時間、合計時間、移動量、配置誤差を記録します。段階別成功率の分母は「その段階に到達した完了試行」のみで、平均段階時間は成功時のみです。リプレイはロボット位置・関節・物体・把持・段階・経路を同期して復元し、測定値を変えません。運搬CSV（schema_version=3）は全体状態と予定数・完了数を含み、未実行段階の時間は空欄です。手法IDは移動計画0=直線/1=A*、移動制御0=Pure Pursuit/1=DWA、腕計画0=関節直結/1=上方経由/2=RRT-Connect、腕制御0=速度制限P/1=加速度制限Pです。

最初は「成功例を実行（seed 42）」でA*＋DWA、腕RRT-Connect＋速度制限Pを設定できます。seed 42では、直線移動の2手法は最初の障害物で計画失敗、A*＋腕RRTの2制御は全段階成功します。A*＋DWAで腕の6手法を比較すると、関節直結の2制御と上方経由＋加速度制限Pは棚板に接触して把持段階で止まり、上方経由＋速度制限PとRRTの2制御は成功します。失敗もそのまま比較結果に残します。

ブラウザ検証: `python scripts/test_g1_mobile_browser.py --url http://127.0.0.1:8091/ --output work/mobile-browser`（事前にTrunkでビルドしdistをHTTP配信）。実WASMで4手法の成否、段階集計、停止・ステップ、条件固定、CSV、把持・配置のリプレイ、スマホからの手動切替を確認します。

運搬条件は3種類です。「標準の運搬」に加え、「把持後に運搬経路が閉鎖」は持ち上げ完了時にX=0の通路を柵で遮断し、荷物を保持した状態で運搬経路を計画します。「配置棚が到達範囲外」は配置先の棚を1.2m高くし、運搬後の位置IKで到達可否を評価します。失敗結果を決め打ちせず、既存のナビ計画・IKの結果を記録します。柵は把持前のリプレイには表示されません。

条件は開始時に固定し、結果・リプレイの名称にも残します。運搬CSV v2は従来の20列の末尾に`scenario_id`（0=標準、1=閉鎖、2=高い棚）と`scenario`を追加した22列です。schema_version列は2になります。腕単独CSVの形式は変わりません。「成功例を実行」は運搬条件も標準に戻します。

### 一時閉鎖と運搬の待機・再試行

4番目の条件「運搬経路が5秒後に開通」は、把持・持ち上げ完了時に柵を閉じ、運搬段階の150tick（5シミュレーション秒）後に開きます。「運搬時に待機・再試行」を有効にすると既存のナビ回復処理を使い、停止状態で15tick（0.5秒）ごとに計画し直します。最大300tick（10秒）で打ち切り、恒久的な閉鎖では荷物を保持したまま計画失敗になります。設定は開始時に固定し、棚への最初の移動には適用しません。

seed 42でA*＋腕RRTを使うと、回復なしは運搬段階で即時失敗、回復ありは5秒・10回の再試行後に再開して配置まで完了します。待機中は位置・関節・荷物を固定し、一時停止は柵の開通時計も止めます。リプレイは待機状態・待機時間・再試行回数・柵の開閉・再計画後の経路を保存したフレームから復元します。

運搬CSV v3はv2の22列の末尾に`recovery`（true/false）、`waiting_s`、`retries`を追加した25列です。schema_versionは3です。腕単独CSVは変更しません。`scripts/test_g1_mobile_wait_browser.py`は一時閉鎖の回復有無、待機中の停止・1ステップ、条件固定、CSV、待機と開通後のリプレイを実WASMで確認します。

### 再試行なし／ありを同じ条件で比較

「再試行なし／ありを比較」は選択した運搬条件・seed・移動計画・移動制御・腕計画・腕制御を固定し、再試行なし→ありの順で2試行を実行します。2試行の記録を同時に残し、段階集計は再試行の設定ごとに表示します。結果行には待機秒数と再試行回数を表示し、リプレイの選択肢にも再試行の有無を記載します。

「待機比較の例（seed 42）」は5秒後に開通する条件、A*＋DWA、腕RRT＋速度制限P、seed 42を設定して、この2試行比較を開始します。1試行目は運搬経路の計画失敗、2試行目は5秒・10回の再試行後に配置成功となります。比較中の設定変更は次の開始時に反映します。停止・1ステップ・中断・個別リプレイにも対応します。

運搬CSVの形式はv3・25列のままです。expected_trials=2、completed_trialsは完了した試行数、recovery=false/trueで2試行を区別します。通常の単独実行は予定数1、移動4手法比較は予定数4です。未完了の試行は行を作りません。`scripts/test_g1_mobile_recovery_pair_browser.py`で実WASMの比較例・設定固定・集計・CSV・両試行のリプレイを検証します。

### デモ一覧

PLANNING × CONTROLの先頭にある「デモ一覧」から7つのプリセットを開始できます。障害物迂回・横切る箱は移動4手法、腕の障害物回避は腕6手法、標準運搬・経路閉鎖は腕RRT固定の移動4手法、待機と再開はA*＋DWA・腕RRT固定の再試行2設定、腕の手法を変える運搬はA*＋DWA固定の腕6手法を比較します。seedはすべて42です。横切る箱はナビの再試行を有効にし、ほかの移動4手法の例は再試行なしです。

カードはエンジン準備後に有効になります。カードから既存の比較ボタンを起動し、進行・結果の表示へ移動します。「デモ一覧へ戻る」で一覧に戻り、キーボードのフォーカスも選んだカードへ戻します。一覧への移動だけでは実行を停止しません。次のカードを選ぶと、そのデモを新規に開始します。「最後に選んだデモ」は選択履歴で、手動変更後の設定を表すものではありません。細かい条件は従来の選択欄で変更できます。

`python scripts/test_g1_demo_gallery_browser.py --url http://127.0.0.1:8095/ --output work/gallery-browser`で、実WASM上のカード起動、領域間の切替、移動・腕・運搬の結果、キーボード操作とスマホ表示を確認できます。

### 運搬タイムラインからリプレイ

運搬の完了記録を、移動（青）・把持（紫）・運搬（緑）・配置（黄）・待機（橙）の時間区間で表示します。区間は保存した状態遷移のtickから作り、全試行で同じ時間軸を使います。短い失敗や未実行の段階を長く見せるための幅補正はしません。区間を選ぶと、その記録・時刻を既存リプレイで表示します。非常に短い区間はTab＋Enterでも選択でき、終了地点ボタンから成功・失敗時点へ直接移動できます。比較中・中断した比較の区間は再生できません。

タイムラインの操作は結果・集計・CSVを変更しません。`scripts/test_g1_mobile_timeline_browser.py`は実WASMで途中の無効状態、待機・運搬・配置と失敗地点への移動、スライダー同期、キーボード・スマホ操作と記録不変性を確認します。

### Compare arm methods during delivery

**腕6組を比較** runs all three arm planners and both arm controllers through the
complete delivery task. Navigation planner/controller, recovery policy, scenario,
seed and initial world are captured once and held fixed. Changing selectors while
a comparison runs affects only the next start. Each trial restarts the robot and
object; failures retain their actual stage rather than receiving zero times for
unexecuted stages. The six results share the existing CSV, stage summary and
clickable replay timeline. The gallery's **腕の手法を変えて運ぶ** preset uses
A* + DWA, standard delivery, recovery off and seed 42.

```bash
python3 scripts/test_g1_mobile_arm_comparison_browser.py --url http://127.0.0.1:8080/ \
  --output work/arm-delivery-browser
```

### Delivery benchmark across five seeds

**5 seedで腕30試行** repeats the six arm methods on five consecutive u32 seeds,
starting at the selected seed (wrapping at u32::MAX). Each seed is applied to both
the delivery layout and arm planning. Scenario, navigation methods and recovery
policy are captured at start; each method restarts the same world for that seed.
Seed variation uses the existing seeded pick-target placement and RRT sampling,
not arbitrary furniture randomization.

The summary shows completed/expected counts, successes/completed, success rate,
mean simulated time of successful trials only, and failure counts for each stage.
Pending trials never count as failures. Missing rates and means are shown as —.
Stopping retains completed rows and marks the batch interrupted. Once all thirty
trials finish, the per-seed buttons jump to each success/failure endpoint in the
saved replay. Replay selectors include arm methods and seed. CSV schema 3 remains
unchanged, with expected_trials=30 and all individual measurements. These five
deterministic layouts do not establish general robot or physical grasp performance.

```bash
python3 scripts/test_g1_mobile_benchmark_browser.py --url http://127.0.0.1:8080/ \
  --output work/mobile-benchmark-browser
```

## Japanese and English versions

Use **Language / 言語** to switch without reloading or resetting the simulation.
Direct links work under the GitHub Pages project path:

- Japanese: `https://rsasaki0109.github.io/RobotNativeEngine/?lang=ja`
- English: `https://rsasaki0109.github.io/RobotNativeEngine/?lang=en`

An explicit supported URL language takes priority over the saved browser choice;
otherwise Japanese is the initial default. Selection is remembered in local
storage when available. If storage is blocked, the URL and selector still work.
UI labels, descriptions, results, timeline text and accessibility labels are
localized. Numeric inputs, option values, saved scenes, replay payloads and CSV
records keep their existing canonical format (including Japanese result labels);
changing the display language never rewrites experiment evidence or restarts runs.

```bash
node web/rne_web_viewer/i18n.test.cjs
python3 scripts/test_g1_bilingual_browser.py --url http://127.0.0.1:8080/ \
  --output work/bilingual-browser
```

## Share experiment conditions

Open **比較条件をリンクで共有 / Share experiment conditions**, choose an execution
mode, then create a link from the current form settings. The link captures the
next-run task, planners/controllers, seed, recovery policies, delivery scenario,
and the selected language. Custom navigation tasks also include their validated
goal and obstacle geometry, including scheduled appearance steps.

Links use a versioned base64url JSON document in `#rne=…`, relative to the current
project URL. Nothing is uploaded. Opening a link validates the complete document
before changing any settings, previews the conditions, and does **not** start a
simulation. **Run shared conditions** reapplies the captured snapshot and invokes
the existing run/compare/benchmark action when the engine is ready. Later form
edits do not silently alter this snapshot; create a new link to share those edits.

Share links contain settings rather than recorded results or a frozen engine
version; running on a later engine build may produce different results. CSV and
replay exports remain the way to retain evidence. JSON is limited to 16 KiB, and
unknown versions/fields or invalid settings/geometry are rejected. When clipboard
access is unavailable, the generated link remains selectable for manual copying.

```bash
node web/rne_web_viewer/share.test.cjs
python3 scripts/test_g1_share_browser.py --url http://127.0.0.1:8080/ \
  --output work/share-browser
```

## Inspect completed delivery failures

Use **運搬結果の絞り込みと失敗確認 / Filter delivery results and inspect failures**
to combine outcome, terminal stage and seed filters. Results, trial diagnostics
and timeline rows are hidden/shown together; original record indices and the
full replay dropdown remain unchanged. Filter counts include completed trials
only. Stage 0–3 means the stage where a trial failed; stage 4 means all stages
succeeded. Empty matches have an explicit message and a reset control.

Expand trial diagnostics to see the actual terminal result, stage, simulated
time, accumulated waiting/retries and whether the object was held. After the
comparison completes, jump to either its saved endpoint or the latest captured
frame at/before one second before the end (the start for very short trials).
The displayed replay time is the actual saved tick. Incomplete/interrupted
comparisons retain read-only diagnostics.

Filters change presentation only: aggregate statistics, CSV exports, stored
snapshots and shared experiment conditions still cover the full batch. Changing
language keeps filter selections. Starting a new batch resets an unavailable
seed selection to All seeds; outcome/stage remain display preferences.

```bash
node web/rne_web_viewer/result_filters.test.cjs
python3 scripts/test_g1_mobile_diagnostics_browser.py --url http://127.0.0.1:8080/ \
  --output work/diagnostics-browser
```

## Play saved experiment replays

Navigation, arm and delivery replay panels have Play/Pause, previous/next saved
frame, and 0.5× / 1× / 2× / 4× playback speeds. Playback follows the actual saved
30 Hz tick values, including irregular capture intervals. The display chooses
the latest saved frame at/before the presentation time, so slow displays may
skip intermediate snapshots. The reported time is always the selected saved tick.
Previous/next moves between captured frames, rather than advancing simulation.

Scrubbing, choosing another record or a diagnostic/timeline jump pauses playback.
Reaching the end pauses; pressing Play there restarts that record. Changing speed
keeps playback active from the currently displayed tick. Leaving replay, starting
a new task, resetting or hiding the tab stops playback. One player runs at a
time. Language changes preserve playback and its selected speed.

This is a browser presentation clock over immutable records: it does not rerun
planning, control, physics or SimClock, and cannot change results, summaries or
CSV evidence. The separate imported-artifact replay inspector is unchanged.

```bash
node web/rne_web_viewer/replay_player.test.cjs
python3 scripts/test_g1_replay_player_browser.py --url http://127.0.0.1:8080/ \
  --output work/replay-player-browser
```

## Compare navigation × arm combinations

Use **Compare 24 navigation × arm pairs** in delivery to evaluate all 4
navigation methods against all 6 arm methods. Scenario, layout seed and recovery
policy are captured once; later form edits cannot affect queued trials. Execution
order is navigation planner/controller, then arm planner/controller.

The 4-row × 6-column matrix shows each completed trial's actual result, terminal
stage and simulated time. Pending/running trials are separate from failures.
After all 24 complete, select a cell to replay its saved endpoint. Interrupted
batches keep a read-only matrix, including unexecuted cells. Display filters do
not hide matrix cells. CSV schema 3 remains unchanged and reports 24 expected
trials; shared links support the `mobile-matrix` mode in Japanese and English.

**Expand comparison table** widens the existing experiment panel so desktop
screens can show all six arm columns together. On phones it uses the available
height. Row labels and column headers remain fixed while scrolling the matrix.
Return with the same button or Escape; keyboard focus returns to the button.
Closing the experiment/delivery panel or switching out of matrix mode restores
the normal layout. Live table updates and language changes preserve the view.
This display preference never restarts, pauses or changes experiment/replay data
and is not included in shared experiment conditions.

```bash
python3 scripts/test_g1_mobile_matrix_browser.py --url http://127.0.0.1:8080/ \
  --output work/matrix-browser
```
