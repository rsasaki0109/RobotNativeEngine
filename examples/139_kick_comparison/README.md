# Go2/G1 lateral impact recovery

This example drives dynamic Go2 and G1 Rapier articulations with bounded
sensor-feedback control under the same lateral disturbance: a 40 N·s impulse
over 80 ms. The force follows a midpoint-sampled half-sine, normalized so its
commanded `sum(force * dt)` preserves the impulse when the physics rate changes.
The default plant runs at 1 kHz with 32 solver iterations. The animated human
visualizes the impact; robot motion comes from the physical simulation.

The disturbance acts in world +Z at the robot root position plus 0.06 m along
world Y. It is a prescribed body wrench with its physical moment arm, rather
than a simulated human-foot collision. This is a bounded lateral recovery
experiment. It does not establish real Unitree hardware kick resistance or
recovery from arbitrary directions and strengths.

## Physical models and control

The dedicated `go2.rne.scene.toml` and `g1.rne.scene.toml` use derived URDFs
with the source positive masses, COM offsets and complete inertia tensors.
They opt into joint-origin rotations, welded fixed children and preservation
of compound collision parts. Go2's attached feet participate in the
articulation, and G1 keeps all four collision spheres on each sole.
Rapier contact-point reporting includes each compound child's pose, so the
recorded sole support points and point-relative velocities use world frames.

| Plant | Declared total mass, including numerical markers | Standing control |
| --- | --- | --- |
| Go2 | 16.087000011 kg | Joint PD, COM-based contact-load regulation, IMU leg-length feedback; hip abduction ±0.45 rad |
| G1 | 34.133857284 kg | Joint PD, COM/DCM/IMU ankle and hip feedback, and outward arm reactions; hip roll ±0.24 rad |

Frame-only links with absent or zero inertials receive 1e-9 kg and diagonal
1e-12 kg·m² inertias to keep numerical articulated-body operations defined.
These are numerical markers, not inferred robot hardware. Go2's initial stance
also uses an equivalent coordinate shift: `q_physical = q_sim + offset`, with
joint-origin rotations and position limits shifted together. Geometry, physical
joint limits and source positive inertials are preserved. The offsets, source
hashes and derived-model hashes are recorded in [models.json](models.json).
Regenerate those fixtures with `python3 examples/139_kick_comparison/prepare_models.py`.

Go2 additionally low-pass filters angular rate and foot normal loads. It
adjusts small individual leg-length biases toward COM-based load shares in
a rectangle approximated from the measured front/rear and left/right foot
means; it is not a general contact-wrench optimizer. The correction uses
a zero-sum bias and a ±0.03 rad calf bound. These gains and filter time
constants are explicit controller design parameters, not identified hardware
parameters. Direct contact-force observations are ideal simulator signals;
hardware would require suitable sensors or estimators.

Both controllers use unit-bearing ForceBased implicit PD with the individual
URDF effort ceilings. Joint targets stay within position limits and change at
at most 6 rad/s, or the lower authored velocity limit. Go2 uses measured roll,
angular velocity and ideal contact loads;
G1 uses mass-weighted COM position and velocity, the
horizontal capture point `com + velocity / sqrt(g / height)`, and body attitude.
Controllers receive the current simulated state and never the disturbance
schedule, future force or animation pose. They apply joint commands; recovery
comes through contact with the ground. Robot poses and velocities are never
corrected directly, and controllers do not apply stabilizing body wrenches.

Default observations are ideal simulator state. The opt-in observation probe
below introduces synthetic estimate errors and timing separately. The model
omits an actual sensor-fusion estimator, actuator thermal and power limits,
detailed transmissions and robot
self-collision. Floor friction and primitive contact proxies are simulator
assumptions, not measured hardware contact properties. The recorded effort ceilings are enforced by the solver;
applied motor torque is not read back. This controller is not Unitree firmware
or a hardware-qualified whole-body controller.

## Headless checks

```bash
cargo run --locked --release -p kick_comparison --example 139_kick_comparison -- \
  --headless --output target/rne-kick-comparison
```

This checks both human animation clips at keys and 120 Hz subframes without a
GPU. It validates the physical plant, runs each robot twice and requires
literal equality of its ordered observed floating-point states throughout
settlement and recovery. Link poses/scales, body COM linear and angular
velocities, joint state and foot impulses participate. Hidden backend solver
caches are outside this replay assertion. The trace stores the observed-state
hash and word count, model and servo configuration, force history and feedback.
It also records SHA-256 hashes of the controller, capture, renderer and plant inputs
embedded at compilation. The encoder rejects a recording whose compiled
inputs differ from the current files; encoding-time source hashes are
recorded separately.

The command writes `go2-trace.json`, `g1-trace.json` and `summary.json`.
Recovery requires upright final height, residual tilt below 0.20 rad, COM speed
below 0.10 m/s and a mean normal load above 0.5 N at every foot over the
last 0.5 seconds. The instantaneous contact count is recorded separately:
Go2's instantaneous loaded-foot count varies while all four carry load
over that window. A fall, positive-impulse non-foot ground contact or a
broken fixed attachment fails the gate. Failed captures retain
their traces. Presentation uses 187 frames at approximately 30 Hz over
6.23 seconds, with the first sampled force frame at index 61.

Run the broader replay and recovery matrix separately:

```bash
cargo run --locked --release -p kick_comparison --example 139_kick_comparison -- \
  --validate-recovery --output target/rne-kick-validation
```

This repeats every case with exact observed-state comparison: nominal lateral
impact, zero force, an earlier opposite-side impact, later onset, 500 Hz,
matched balance-feedback-off cases and front/oblique limit probes. It writes
per-case traces and `recovery-validation.json`. `--validation-robot go2` or
`--validation-robot g1` can repeat only one plant; the report lists its scope.
Public GIF encoding requires the complete two-plant validation report.
At the default 40 N·s, all 12 required recovery cases pass with exact observed
replay. Front impacts topple both robots, and the oblique impact topples G1;
those three failures remain in the full 18-case report as limit observations.

`--impulse-ns J` selects a finite positive common impulse for capture and
validation. To measure a selected strength without requiring it to recover,
use `--impulse-probe --impulse-ns J`; zero is permitted for this diagnostic.
`--impulse-sweep` probes 24, 32, 40, 48 and 64 N·s with exact replay and retains
failed recoveries. Both diagnostics accept the same `--validation-robot` filter.
Their reports are observations of the recovery limit, not public capture gates.

```bash
cargo run --locked --release -p kick_comparison --example 139_kick_comparison -- \
  --impulse-sweep --output target/rne-kick-severity
python3 -m unittest discover -s examples/139_kick_comparison -p test_encode.py -v
```

Balance-feedback ablations
retain the same stance, joint PD gains, effort caps and target-rate limit;
encoder feedback inside the PD motor remains enabled. For Go2, the ablation
disables the IMU opposing-leg reflex while retaining contact-load stance
regulation; for G1, it disables the additional DCM/IMU ankle, hip and arm loops. A disabled balance
loop need not fail every disturbance: report the measured case, including its
zero-force baseline, rather than assuming that it must fall.

The complete report must pass all 12 required cases before public GIF encoding.
Front/oblique directions and balance-off cases remain measured envelope and
ablation observations; their outcomes appear in the generated metadata.
Passing recovery at 500/1,000 Hz does not establish convergence of the full
trajectory. Final values are in the generated summary and
[README capture metadata](../../docs/media/kick-comparison.json).

## Synthetic observation sensitivity

`--observation-probe` runs a separate headless sensitivity matrix for the same
40 N·s, 80 ms lateral wrench. It changes the observations available to the
outer balance controller. Physics, target slew, filters and controller evaluation
remain at 1 kHz; a 250 Hz capture profile holds a sample for four control ticks.
This differs from `rate_500hz` in the existing recovery matrix, which changes
the physics step itself.

One captured state estimate contains attitude/rates, COM position and velocity,
foot positions and Go2 normal loads from the same completed physics tick.
The controller reads only arrived timestamped frames from the DataBus. Pending
frames cannot supply feedback. A held frame retains its sample identity and
error draw. Before the first arrival, bounded nominal stance targets apply
without a current-truth fallback. Capture timing continues through settlement
and the disturbance.

The profiles are declared before measurement:

| Profile | Capture period | Arrival latency | Bounded synthetic errors |
| --- | --- | --- | --- |
| `ideal_reference` | 1 ms | 0 ms | none |
| `sample_500hz` | 2 ms | 0 ms | none |
| `sample_250hz` | 4 ms | 0 ms | none |
| `delay_5ms` | 1 ms | 5 ms | none |
| `delay_20ms_limit` | 1 ms | 20 ms | none |
| `bounded_error` | 1 ms | 0 ms | enabled |
| `combined` | 4 ms | 5 ms | enabled |
| `combined_zero_input` | 4 ms | 5 ms | enabled, zero disturbance |

Attitude error is a pair of world-X/world-Z rotations bounded by ±0.01 rad
each, applied to the captured orientation. Body axes, roll/pitch and gyro
projection use that same perturbed orientation; derived roll/pitch error is
not independently bounded by those rotation amplitudes. Other amplitudes are
±0.02 rad/s world angular rate, ±0.005 m COM/foot position and ±0.02 m/s
COM velocity per world-axis component, plus ±2 N Go2 foot load.
Independent signed-uniform draws use each scene's explicit WorldRandom seed
(Go2: 2003, G1: 2002), fixed domains and ordered channels;
loads saturate at zero with saturation recorded. These amplitudes are engineering
sensitivity bounds, not measured Unitree specifications or Gaussian standard
deviations. One fixed-seed result does not establish probabilistic reliability.

```bash
cargo run --locked --release -p kick_comparison --example 139_kick_comparison -- \
  --observation-probe --output target/rne-kick-observations
```

Use `--validation-robot go2` or `g1` and `--observation-profile combined`
to repeat a selected plant/profile. The report declares its selected scope.
The diagnostic uses the fixed 40 N·s matrix (including its zero-input profile)
and does not combine with rendering, impulse overrides or the qualification matrix.

Each profile is replayed from a fresh seeded plant. The diagnostic retains
failed recoveries and pre-impact settlement failures in its separate report,
including whether the disturbance was applied. Ground-truth evaluation uses
the unchanged recovery predicate. Observation timing, held/startup decisions,
and controller/pipeline words are checked separately from the ordered plant
state. Observation capture, arrival and consumer timestamps are absolute
simulation ticks, including settlement; recovery `time_s` is relative to the end
of settlement. The diagnostic does not qualify a new public GIF or expand the existing
12-case ideal-observation recovery claim.

The implicit position-PD actuator still uses instantaneous simulated joint
state. The probe degrades the outer state estimate, not that inner motor loop.
It contains no raw IMU/encoder fusion, identified sensor statistics, physical
human-foot collision or real-hardware validation. The published GIF retains
its recorded ideal-observation source revision; probe evidence belongs to its
own compiled input and trace hashes.

The fixed-seed matrix produced these results. Each row completed settlement;
each plant and observation/control recording matched its independent fresh replay.
Maximum observation age includes zero-order hold. Peak tilt is measured over
the post-settlement trajectory, including the zero-input baseline.

| Profile | Maximum observation age (ms) | Go2 recovered / peak tilt (rad) | G1 recovered / peak tilt (rad) |
| --- | --- | --- | --- |
| `ideal_reference` | 0 | yes / 0.069500 | yes / 0.289060 |
| `sample_500hz` | 1 | yes / 0.067894 | yes / 0.306257 |
| `sample_250hz` | 3 | yes / 0.066754 | yes / 0.325069 |
| `delay_5ms` | 5 | yes / 0.064574 | no / 1.638381 |
| `delay_20ms_limit` | 20 | yes / 0.072170 | no / 1.639157 |
| `bounded_error` | 0 | yes / 0.069215 | no / 1.646449 |
| `combined` | 8 | yes / 0.072912 | yes / 0.305127 |
| `combined_zero_input` | 8 | yes / 0.020220 | yes / 0.009499 |

G1's three failed profiles retain the measured fall, nonfoot contact and fixed
attachment errors. The combined profile's success alongside those failures
shows that this small fixed-seed matrix is not a monotone tolerance search;
it establishes no maximum safe latency or sensor-error bound.

Across all 18 existing ideal-observation qualification cases, serialization-only
old/new runs matched every ordered plant word and original controller decision,
including settlement: 139,400 completed physics steps and 109,916,900 plant words.
This checks compatibility separately from fresh replay. Hidden solver caches and newly
introduced pipeline state are outside the old/new equality assertion.
The normal binary also passes all 12 required recoveries and exact plant/control
replay across the original 18-case matrix. Source/configuration hashes, all 16
measured summaries and compact compatibility/qualification proofs are retained
in [the observation study](../../docs/media/kick-observation-study.json).

## Measured stronger kick

The default impulse increases from 24 to 40 N·s, or 67%, with the same 80 ms
pulse. Both earlier controllers fell at 40 N·s. The wider stances increase
lateral support using authored-range joint targets; gravity determines the
lower settled height. G1 also combines a bounded opposing hip motor reaction
with an outward shoulder-roll reflex based on roll and angular rate. It holds
an arm excursion until observations are quiet, then returns at 0.5 rad/s.
The hip-attitude gain is 0.5: a 1.0-gain prototype recovered nominally but
fell under late-onset and 500 Hz probes. Reducing that term preserves more
static support margin while retaining the opposing motor reaction.
The tests preserve mass, inertia, friction, servo gains and effort ceilings.

Measured 1 kHz nominal probes with two identical observed recordings
([strength-study data](../../docs/media/kick-strength-study.json)):

| Plant | Earlier controller at 40 N·s | Current controller at 40 N·s | Peak tilt | Final COM speed | Lateral base displacement |
| --- | --- | --- | --- | --- | --- |
| Go2 | falls, non-foot ground contact | recovers | 0.0695 rad | 0.0210 m/s | 0.545 m |
| G1 | falls, non-foot ground contact | recovers | 0.2891 rad | 0.0158 m/s | 0.008 m |

The final G1 controller recovers in the 32 N·s nominal probe and falls at
48 N·s; both outcomes retain exact replay. Stronger and directional probes
retain failures rather than establishing maximum-safe or hardware ratings.
The G1 changes were
measured together; the independent contribution of the arm reflex is not
established by the combined feedback-off ablation. Recovery permits actual
translation, so Go2 is pushed across the floor rather than pinned in place.

## Render and encode

A working Vulkan/Metal/DX12 wgpu adapter is required. Mesa llvmpipe works for
offline Linux rendering. Pillow 10.1 or newer is required only for GIF encoding.

```bash
cargo run --locked --release -p kick_comparison --example 139_kick_comparison -- \
  --render --output target/rne-kick-comparison
python3 examples/139_kick_comparison/encode.py target/rne-kick-comparison \
  --validation target/rne-kick-validation/recovery-validation.json
```

To render an existing capture without repeating the simulation, use
`--render-only`. `--start-frame N --frame-count N` can divide rendering into
disjoint batches. Any requested GPU failure is returned as an error.

The encoder uses a fixed global palette and a 30/30/40 ms duration pattern.
Its labels use the measured impulse and force window. The 960 x 540 GIF, poster
and metadata are `kick-comparison.gif`, `.png` and `.json`. Metadata includes
source, asset and trace SHA-256 hashes plus measured physics results.
Copy those three files to `docs/media/` when updating the README capture.

## Human motion and provenance

[The human asset](../../assets/fixtures/kick_human/README.md) uses pinned
MakeHuman/MPFB CC0 mesh, rig and skin weights, with original procedural
clothing textures, shoes and animation. It is an anatomical textured character,
not a scanned person. Only the root translates; limb bones use rotations
with constant local offsets and unit scales. The kick has weight transfer,
forward knee chamber, extension, retraction and controlled foot planting.
Rotation-only IK targets strike heights of 0.322176 m for Go2 and 0.826094 m
for G1, within 0.01 mm of the final nominal wrench points.
The renderer retimes the path: extension takes 175 ms instead of
350 ms, the impact pose lasts exactly the 80 ms pulse, and retraction takes
220 ms instead of 300 ms. The support-foot path, limb lengths and rig
remain unchanged. Headless checks sample the same retimed path as rendering.
Both CPU and GPU glTF sampling use shortest-path quaternion interpolation.
The human has no physical contact or force-feedback coupling to the robots.

The official robot meshes retain their bundled BSD-3-Clause notices in
`assets/robots/go2_description` and `assets/robots/g1_description`.
