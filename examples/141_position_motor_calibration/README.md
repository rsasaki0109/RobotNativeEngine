# Isolated position-motor calibration

Example 141 measures the vendored Rapier internal ForceBased revolute motor in
an isolated, small-error fixture. It collects a fixed matrix and reports the
numerical calibration criterion separately from collection/replay completeness.
It changes no production solver, G1 controller, gain, ceiling, model or recovery
predicate. No hardware, fall-cause or recovery-performance claim follows.

```bash
cargo run --locked --release -p position_motor_calibration \
  --example 141_position_motor_calibration -- --output target/motor-calibration/report.json \
  --require-calibration
```

The output is created exclusively: an existing report is never overwritten.
Use a new path for an explicitly authorized new attempt. The command always
retains the collected report before deciding its exit status: 0 means complete
collection and replay qualification, 2 means the requested primary calibration
criterion failed, and 1 means collection/replay or report consistency failed.
Without `--require-calibration`, complete collection may exit 0 while clearly
printing `primary_calibration=FAIL`. Read that field rather than interpreting
collection success as numerical accuracy. A collection command runs 72 fresh
one-interval fixtures; verification below runs none.

## Fixed protocol

The 24 cells cross the existing G1 gain pairs `(300,12)` and `(250,8)` in
N·m/rad and N·m·s/rad, position targets `{-0.001,0,+0.001}` rad, small-step counts
`{1,32}`, and internal PGS passes `{1,2}`. Every cell uses two stabilization
passes and one 1 ms outer interval. There are no runtime gain/solver overrides
or best-setting selection.

The fixed identity parent and coincident dynamic sphere have one X revolute DOF,
mass 1 kg and radius 0.1 m, with nominal inertia 0.004 kg·m². Initial position and
generalized velocity, gravity, passive/body damping and armature are zero. There
are no contacts, hard limits, CCD or additional solver iterations; torque cap is
2 N·m. Each cell uses two fresh telemetry-ON worlds and a third fresh OFF world.
The example directly uses the local Rapier fork; it does not enable the optional
RNE backend feature or alter backend defaults.

Public mass recomputation and `forward_kinematics(&bodies,true)` initialize the
fixed-root structure before stepping, without advancing time. Pre-step native
body Ixx supplies an independently rounded inverse inertia only under the stated
isotropic, coincident, identity-axis, zero-damping/armature assumptions. It is not
an observed first-row native LU. The latest post-step public LU inverse scalar
is recorded separately and compared with absolute tolerance 1e-8 and relative
tolerance 2e-5; disagreement remains a failed qualification.

For actual first-slice duration `h`, retain the public native motor coefficients
and independently reproduce f32 operation order for `a=k/(h*k+d)`,
`g=1/(h*h*k+h*d)`, the initial row RHS, and rounded bound `cap*h`. Compare their
bits. Promote the rounded RHS, inverse inertia and g to f64 for
`p*=clip(-(rhs+v)/(inverse_inertia+g), -bound, +bound)`.
The primary four nonzero cells are N32/PGS1: final first-slice impulse after PGS
and stabilization must satisfy `abs(p-p*) <= max(1e-8 N·m·s, 0.01*abs(p*))`, with
finite exposed state and complete actual slice coverage. All eight zero-target
controls must retain zero impulse and no motion. The 1% requirement is a fixed
software calibration gate, not measured hardware accuracy.

## Retained evidence and verification

The report retains every case, including failed, nonfinite, partial or missing
measurements. Missing data stay unavailable. IEEE bits distinguish NaN payloads,
signed zero and Option presence. Known body handles are sorted; individual native
body/multibody serialization retains private joint coordinates through a
source-bound, byte-roundtripped serde projection. Generalized velocity comes from
`Multibody::joint_velocity`, rather than attached-body velocity. This captures
serialized native entity fields, not every private pipeline cache.

All three worlds retain full native state bytes and canonical words; both ON
worlds retain full ordered telemetry and words, including every final slice,
duration, rounded bound and coverage ordinal. ON/fresh and ON/OFF state comparisons
are literal; ON/fresh telemetry also compares type, presence, IEEE bits and order.
Signed sums, absolute sums, cancellation, nonzero-slice sign reversals, bound
exposure, final coordinate/velocity/energy and N/PGS differences are descriptive.
Final slice impulses do not expose within-pass iterates or hardware torque peaks.

```bash
cargo run --locked --release -p position_motor_calibration \
  --example 141_position_motor_calibration -- --verify-report target/motor-calibration/report.json
cargo test --locked -p position_motor_calibration
RUSTDOCFLAGS="-D warnings" cargo doc --locked -p position_motor_calibration --no-deps
```

Verification creates no worlds and recomputes configuration/source binding,
serialized state, telemetry words, replay, oracle assessments and summary. It
checks report consistency rather than attesting that arbitrary bytes came from
an external experiment. `--require-calibration` also applies during verification.
Selected source hashes are a bounded input manifest; the preregistered protocol,
lock/compiler/build receipt and actual run are bound separately before execution.
Neither a passing toy fixture nor a failed calibration selects a new G1 setting.

## Retained result

The [full report](../../docs/media/position-motor-calibration.json) retains all 24
conditions and all 72 separately constructed/completed worlds, with no omitted
native state bytes, canonical words or ordered motor slices. Collection and exact
ON/fresh/OFF physical replay pass; ON/fresh telemetry also matches exactly. All
eight zero-target controls remain motionless. Every nonzero condition fails the
fixed first-slice tolerance, including the four primary N32/PGS1 conditions:

| Gain pair (k,d) | Target (rad) | Expected impulse (N m s) | Actual impulse (N m s) | Absolute error (N m s) | Tolerance (N m s) |
|---|---:|---:|---:|---:|---:|
| (300,12) | -0.001 | -8.570856e-6 | -6.250000e-5 | 5.392915e-5 | 8.570856e-8 |
| (300,12) | +0.001 | +8.570856e-6 | +6.250000e-5 | 5.392915e-5 | 8.570856e-8 |
| (250,8) | -0.001 | -7.352520e-6 | -6.250000e-5 | 5.514748e-5 | 7.352520e-8 |
| (250,8) | +0.001 | +7.352520e-6 | +6.250000e-5 | 5.514748e-5 | 7.352520e-8 |

For completeness, the positive-target sensitivity cells retain these first
impulses; the full report also retains each negative-target cell and every zero
control. These observations do not choose a best setting:

| Gain pair | Small steps | PGS passes | Expected impulse (N m s) | Actual impulse (N m s) | Tolerance pass |
|---|---:|---:|---:|---:|---|
| (300,12) | 1 | 1 | 7.361963e-5 | 7.615160e-5 | FAIL |
| (300,12) | 1 | 2 | 7.361963e-5 | 7.279623e-5 | FAIL |
| (300,12) | 32 | 1 | 8.570856e-6 | 6.250000e-5 | FAIL |
| (300,12) | 32 | 2 | 8.570856e-6 | -6.250000e-5 | FAIL |
| (250,8) | 1 | 1 | 8.163266e-5 | 9.093692e-5 | FAIL |
| (250,8) | 1 | 2 | 8.163266e-5 | 7.712150e-5 | FAIL |
| (250,8) | 32 | 1 | 7.352520e-6 | 6.250000e-5 | FAIL |
| (250,8) | 32 | 2 | 7.352520e-6 | -6.250000e-5 | FAIL |

The authoritative collection used `--require-calibration`, saved the complete
report and returned exit 2. Verifying the retained report returns 0 and prints
`primary_calibration=FAIL`; it advances no worlds. All 24 conditions have finite
exposed state, qualified slice coverage, matching native/independent coefficient
bits and a passing separately acquired inertia cross-check. The N32/PGS1 nonzero
conditions record 30 nonzero-slice sign reversals over 32 final slices. These final
slice observations do not expose within-pass iterates, peak hardware torque or
the cause of a G1 fall.

The run starts from main base `0098eda761af0cbcc211bb7608c5f0cc4715127e` plus the
modified source snapshot explicitly recorded in the report. Its locked release
build uses Rust 1.95.0 (`59807616e`, LLVM 22.1.2) on
`x86_64-unknown-linux-gnu`, with the direct Rapier `serde-serialize` feature.
The binary SHA256 is `4804366a897a6ca335a0a40acaf24a5f58d6cd03e227abb8566994ee8fd8a042`;
report SHA256 is `a5a1a8856c58b5ee964a4fc6defbcceb65bebf5138bcb8126b59ece4d1bdc7ca`.
The matrix was executed once; the two ON runs already provide the fresh replay.
Compilation corrections and pure unit tests before that run constructed no
physics worlds. Production solver/control files and prior kick/GIF evidence
remain unchanged.
