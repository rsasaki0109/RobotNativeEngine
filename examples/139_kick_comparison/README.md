# Go2/G1 lateral impact recovery

This example drives dynamic Go2 and G1 Rapier articulations with bounded
sensor-feedback control under the same lateral disturbance: a 24 N·s impulse
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
| Go2 | 16.087000011 kg | Joint PD, COM-based contact-load regulation and IMU feedback changing opposing leg lengths; hip abduction ±0.25 rad |
| G1 | 34.133857284 kg | Joint PD and COM/DCM/IMU ankle and hip feedback; hip roll ±0.12 rad |

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
at most 6 rad/s, or the lower authored velocity limit. Go2 uses measured roll
and angular velocity; G1 uses mass-weighted COM position and velocity, the
horizontal capture point `com + velocity / sqrt(g / height)`, and body attitude.
Controllers receive the current simulated state and never the disturbance
schedule, future force or animation pose. They apply joint commands; recovery
comes through contact with the ground. Robot poses and velocities are never
corrected directly, and controllers do not apply stabilizing body wrenches.

The observations are ideal simulator state. The model omits sensor-estimation
error, actuator thermal and power limits, detailed transmissions and robot
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
It also records SHA-256 hashes of the controller, capture and plant inputs
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
Balance-feedback ablations
retain the same stance, joint PD gains, effort caps and target-rate limit;
encoder feedback inside the PD motor remains enabled. For Go2, the ablation
disables the IMU opposing-leg reflex while retaining contact-load stance
regulation; for G1, it disables the additional DCM and IMU ankle/hip loops. A disabled balance
loop need not fail every disturbance: report the measured case, including its
zero-force baseline, rather than assuming that it must fall.

In the recorded 1 kHz matrix, both balance-feedback-off plants still recover.
Go2's peak tilt increases from 0.262 rad with IMU feedback to 0.526 rad without
it. G1's peak tilt is similar (0.137 rad enabled, 0.133 rad disabled), while
its final residual tilt increases from 0.00145 to 0.01002 rad and COM speed
from 0.0198 to 0.0395 m/s. These results do not establish that the additional
loops are necessary for this particular disturbance.

All 12 required recovery cases pass. The declared recovery envelope is lateral
±Z; the separate 24 N·s front (+X) probe topples both plants, and the 45-degree
X/Z probe topples G1. Those G1 falls include shoulder ground contacts and exceed
the fixed-attachment rotation tolerance, so they also fail plant-integrity
checks. Go2 recovers in the oblique probe, with COM translation of approximately
0.211 m along X and 0.117 m along Z. The recovery gate permits that translation. The limit
probes remain recorded observations; extending the recovery envelope needs
further control and contact validation. Passing the 500/1,000 Hz recovery gates
does not establish convergence of the full trajectory. Final measured values
are in the generated summary and
[README capture metadata](../../docs/media/kick-comparison.json).

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
Both CPU and GPU glTF sampling use shortest-path quaternion interpolation.
The human has no physical contact or force-feedback coupling to the robots.

The official robot meshes retain their bundled BSD-3-Clause notices in
`assets/robots/go2_description` and `assets/robots/g1_description`.
