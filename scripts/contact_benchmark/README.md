# H1 sphere-drop benchmark

This is a one-contact quality/cost audit of **RNE Native** and MuJoCo 3.9.0.
It does not change either solver. Sphere radius 0.05 m, mass 1 kg, initial
surface gap 0.1 m, world gravity `(0, -9.81, 0)`, duration 2 s, no initial
velocity. Both floors are 10 × 1 × 10 m boxes with top face at Y=0.
Both use the sphere's explicit `2mr²/5` inertia, zero friction and a requested
inelastic impact. MuJoCo represents the latter through the documented damped
contact settings, not a literal restitution coefficient.

```sh
cargo build --release -p rne_physics_native --example contact_drop
python -m venv .venv-contact
.venv-contact/bin/pip install -r examples/116_robot_workbench/python/requirements-mujoco.txt
RNE_CONTACT_DROP=target/release/examples/contact_drop .venv-contact/bin/python -m unittest discover -s scripts/contact_benchmark -v
.venv-contact/bin/python scripts/contact_benchmark/drop.py \
  target/release/examples/contact_drop /tmp/h1.json.gz
```

The output must be a new path. Linux CPU affinity is required: the driver pins
itself and its child process to the first allowed logical CPU. Native binaries
with debug assertions are rejected. Run without competing builds for meaningful
timing. The JSON stores all traces, input/settings, generated MuJoCo XML,
versions, script/binary hashes, CPU affinity, timing samples and failed gates.

## Frozen H1 v1 definitions

- Native: four timesteps (0.25/0.5/1/2 ms), iteration caps 20/200, normal
  regularization 0/0.001: 16 configurations. Other solver defaults are recorded.
- MuJoCo: same timesteps and caps, Newton/Euler, tolerance 1e-10, frictionless
  `condim=1`, zero margin/gap, solimp `0.999 0.999 0.001 0.5 2` and four solref
  choices: `[0.02,1]`, `[0.0002,1]`, `[-1e6,-2000]`, `[-1e8,-20000]`: 32
  configurations. Ref-safe remains enabled (positive time constants are bounded
  by the engine). These are a bounded search, not optimal MuJoCo tuning.
- Trace rows: `[time_s, post_step_center_y_minus_radius_m, vertical_velocity_m_s,
  normal_impulse_Ns_for_previous_step]`. Initial row has zero velocity/impulse.
  Neither backend's pre-step manifold gap is used for penetration.
- First support: first impulse larger than `mass * 1e-10 N s`. Its occurrence
  is bracketed by the preceding/current sample times; the distance from analytic
  impact time `sqrt(2h/g)` to this interval must be at most one timestep.
  This is an interval-resolution check, not an exact impact-time measurement.
- End of impact: first sample at/after first support with `|v_y| <= 1 mm/s`,
  no later than analytic impact plus 100 ms. If missing, impulse and rebound
  gates fail. Sum impulse through this sample, subtract `mg*(end_time-t_impact)`,
  then compare with `m*sqrt(2gh)` at relative tolerance 1%.
- Maximum penetration over **all** samples must be <=10 µm. Maximum speed in
  the last 0.5 s <=1 mm/s. Maximum positive gap after impact end <=10 µm.
- Mechanical energy `mg*gap + m*v_y²/2` may not exceed initial `mgh` by more
  than 1e-8 of initial energy. This catches net injection, not every local
  nonphysical energy exchange; no claim of full energy-conservation auditing.
- Normal impulses must be >=`-mass*1e-10 N s`. MuJoCo warnings or simulation
  clock reset fail backend health. Native impulse events use f32 magnitudes;
  do not infer sub-f32 impulse accuracy from them.
- Each audit must repeat exactly within its backend. Each timed run's final
  gap/velocity must exactly equal the corresponding audit. No tolerance is
  relaxed to make the two modes agree.

## Timing and selection

Audit time is not a performance sample. Timed runs perform a full batch without
traces, IPC per step, model construction, JSON, or ECS synchronization. Native
uses two timed segments, with penultimate-pose write-back between them outside
timing: its ECS velocity is a pose difference over the last write-back interval.
The final write-back is also untimed. This keeps final velocity comparable to
the audit without requiring per-step ECS export. MuJoCo uses one Python/C call
with `nstep`, without Python per-step overhead in timing mode.

One complete warm-up and five fresh-world repetitions are used for every setting.
All times, median and range are retained as seconds per simulated second. Each
backend's fastest **passing** nominal configuration is frozen and tested without
retuning at mass 0.1/10 kg × initial gap 0.01/0.5 m. If none pass, selection is
null, not the fastest failing setting. Holdout failures remain in the report.
`--require-native-pass` optionally writes the report and then fails if there are not four
passing native holdouts. This strict selection check can fail: a fastest nominal
setting is not guaranteed to generalize. CI retains these failures as benchmark
evidence; it instead gates a fixed rigid reference (regularization zero, 20
iterations, 2 ms) on the nominal case and four varied cases through integration
tests, independently of noisy timing-based selection. This does not reselect a
failed winner or turn its holdout failure into a pass.

This measures backend batch-step cost, not the full robot/ECS/control loop and
not the isolated constraint kernel. Solver sweep counts and convergence flags
are not exposed by this backend runner yet; iteration caps are not actual counts.
Discrete geometric samples do not prove the absence of inter-step tunnelling.
Multiple contacts, friction, angular motion, changing contact topology and parallel
environments remain separate tests. Use `plot.py` (matplotlib required) to plot
the committed report; benchmark execution itself does not need matplotlib.


## Matched-quality refinement (v2/v3)

`matched.py` reuses the v1 runners and gates unchanged. Default mode reproduces
v2's finer-step critical-damping search. `--overdamped` runs the separately
frozen v3 grid after v2 failed on rebound. Both calibrate on known 0.1/0.5 m
drops at 1 kg, then freeze settings and validate on 0.3/3 kg × 0.03/0.3 m.
No validation-based reselection is allowed, and no ratio is reported when
validation fails. RNE uses the fixed zero-regularization, 20-iteration, 2 ms
reference. See `docs/HARD_CONTACT_MATCHED_RESULTS.md` for grids and limitations.

```sh
.venv-contact/bin/python scripts/contact_benchmark/matched.py target/release/examples/contact_drop /tmp/h1-v3 --overdamped
.venv-contact/bin/python scripts/contact_benchmark/verify_matched.py /tmp/h1-v3/report.json --require-comparison
```

Use a fresh directory. Reports checkpoint after every case, retaining failed
quality cases. Full samples are lossless float64 `.npy.gz` files; Git stores the
small summaries, while CI uploads report and traces together. The current local
captures are provided as workspace archives, not asserted to be permanently
hosted. `verify_matched.py --traces DIR` verifies a stored report against its
matching archive, recomputing all quality metrics and the comparison summary.
The v1 capture uses lossless XZ compression below the ADR 034 inline size limit.

## H2 box contact audit

`box.py` runs a separate frozen 48-case native cube/floor diagnostic. It audits
all eight world-space corners and includes rotational kinetic energy and settled
angular speed. It does not reuse the sphere's impact/rebound assumptions or claim
a MuJoCo speedup. Build `--example box_drop`, run `box.py BINARY OUTPUT.json.xz`,
then `verify_box.py OUTPUT.json.xz --require-pass`. Set `RNE_BOX_DROP` for the
native four-corner regression test. See
[protocol and results](../../docs/HARD_CONTACT_BOX_RESULTS.md).

## H3 dynamic stacks

Build `--example stack_drop`, then run `stack.py BINARY OUTPUT.json.xz` and
`stack.py BINARY OUTPUT.json.xz --verify`. The original 24-case grid exposed failures at 20 iterations. After the normal-block
solver change, `RNE_STACK_DROP` gates all 24 combinations at both budgets. Verification checks stored records,
not universal quality success. Geometry uses all OBB separating axes and load
impulses are resolved per interface. See
[stack protocol and failures](../../docs/HARD_CONTACT_STACK_RESULTS.md).


## Paired stack solver comparison

`compare_stack.py BEFORE_BINARY AFTER_BINARY OUTPUT.json.xz` alternates execution
order for the 12 fixed 200-iteration reference cases. Every side has exact replay,
five fresh-world cost samples and the unchanged H3 quality gates; failed quality
suppresses the aggregate speedup. `verify_stack_pair.py OUTPUT.json.xz` recomputes
the saved full-state records and costs. See
[normal-block implementation and measured results](../../docs/HARD_CONTACT_BLOCK_SOLVE_RESULTS.md).

## MuJoCo stack calibration

`mujoco_stack.py OUTPUT.json.xz` scans 24 frozen H3 calibration cases with pinned
MuJoCo 3.9.0. `verify_mujoco_stack.py OUTPUT.json.xz` recomputes quality from all
saved poses. Calibration failures are retained and do not imply a valid speed
comparison. See [results and settings](../../docs/HARD_CONTACT_MUJOCO_STACK_RESULTS.md).

## Matched-quality stack study

`stack_matched.py BINARY NEW_DIRECTORY` freezes the fine timestep grid and
solref before calibrating on 3-level stacks with 1/10 kg top masses. The first
passing timestep is fixed for independent 0.3/3 kg validation at 2/3 levels.
`verify_stack_matched.py DIRECTORY` recomputes the raw records, calibration order
and conditional comparison. See
[protocol and results](../../docs/HARD_CONTACT_STACK_MATCHED_RESULTS.md).

## Impact-phase robustness

`stack_phase.py BINARY NEW_DIRECTORY` retains the selected settings and changes
initial gaps to vary analytic impact phase in both backends. `--verify` recomputes
all eight saved records. A failed or missing phase suppresses the comparison.
See [protocol and results](../../docs/HARD_CONTACT_STACK_PHASE_RESULTS.md).

## H4 frictional sliding

Build `--example friction_slide`, then run `slide.py BINARY NEW_OUTPUT.json.xz`
and repeat with `--verify` to recompute all 32 records. The fixed analytic gates
cover speed, stopping distance/time, penetration, energy and lateral/rotational
drift. `RNE_SLIDE` enables the native high-friction integration test. See
[protocol and results](../../docs/HARD_CONTACT_FRICTION_SLIDE_RESULTS.md).

## H5 static-friction threshold bracket

Build `--example friction_threshold` and run `threshold.py BINARY OUTPUT.json.xz`,
then add `--verify` to recompute all records. The constant COM load is represented
by horizontal body acceleration with a fixed floor; this does not exercise the
external-wrench API. Ratios ±0.8/±1.2 bracket μmg without claiming exact threshold
identification. `RNE_THRESHOLD` enables native integration coverage. See
[protocol and results](../../docs/HARD_CONTACT_FRICTION_THRESHOLD_RESULTS.md).
