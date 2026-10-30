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
