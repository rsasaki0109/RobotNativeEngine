# Isolated motor construction and integration diagnostics

Example 143 records the actual inputs used to construct a Rapier internal
single-DoF motor row and the coordinate/velocity endpoints around integration and
stabilization. It complements example 141's scalar calibration and the existing
stage/impulse observations. It does not change the motor equation, gains or solver.

The separate schema-1 `InternalUnitMotorConstructionV1` companion captures actual
constructor locals: joint coordinate, target position and velocity, stiffness,
damping, duration and inverse duration, optional limits, limit-clamped target
velocity and ERP inverse-duration coefficient. Each constructed row contains its
actual initialized stage-v1 row, including J/W, RHS, CFM, accumulator and bounds.
Later endpoints retain the current coordinate and actual multibody velocity
storage alongside the event's generalized solver velocity.

These observations distinguish the velocity that advances a coordinate from the
velocity stored after stabilization. A completed-step velocity must not be
substituted for an earlier integration input.

| Companion phase | Interpretation |
|---|---|
| `BeforeExternalForces` / `AfterExternalForces` | Current-slice row storage is uninitialized and is not read. |
| `AfterRowConstruction` | Actual constructor inputs and initialized rows. |
| `BeforePositionIntegration` | Generic solver V has been copied into the named multibody, immediately before its coordinate integration. |
| `AfterPositionIntegration` | Coordinate/V immediately after integration, before forward kinematics and dynamics refresh. |
| `AfterJointStabilization` | One joint stabilization pass completed, before its contact pass. |
| `FinalSlice` | Final solver slice after its contact/friction work, before final velocity writeback. |
| `AfterVelocityWriteback` | Final generic V has been copied into multibody storage. |

Availability explicitly distinguishes uninitialized storage, no supported rows
and available rows. Missing observations are not measured zeros. Integration
events identify the actual multibody root; row cache/slice identity must remain
part of any subsequent interpretation.

## Fixed physical fixture and invocation

The fixture reproduces example 141's two G1-posture N32/PGS1 nonzero-sign cells:
a fixed coincident parent, one dynamic uniform sphere of radius 0.1 m and mass
1 kg, identity X-revolute frames, ForceBased position control with stiffness
300 Nm/rad, damping 12 Nm s/rad and a 2 Nm ceiling. Gravity, passive damping and
armature are zero; initial generalized V is zero, sleeping/CCD and joint contacts
are disabled, and there is no other collider. One 1 ms outer interval uses
32 small steps, one PGS pass and two stabilization passes. Targets are only
negative (-0.001 f32 rad) or positive (+0.001 f32 rad).

After the experiment's source/compiler/features/binary/reader/protocol and
independent review are frozen, one collection invocation is:

```sh
cargo run --locked --release -p isolated_motor_construction_collector \
  --example 143_motor_construction -- \
  --target negative --observer on --label negative-on \
  --output /absolute/existing-directory/negative-on.jsonl.gz
```

All four flags are mandatory and may occur only once. `--observer` is `on` or
`off`; labels contain 1–64 ASCII letters, digits, underscores or hyphens. Output
must be an absolute new file in an existing directory. `--help` and malformed
arguments construct no fixture. The CLI exposes no gain/count/timestep tuning,
retry or resume option. A registered study runs the same frozen executable in
six fresh processes: negative ON, negative fresh ON, negative OFF, positive ON,
positive fresh ON, positive OFF. Fresh ON uses `on` with its own label and output.

ON explicitly enables the constructor companion, existing stage-v1 and existing
impulse-v1 observers. OFF keeps all three observations null. Each process creates
one fixture and calls `pipeline.step` once. Preserve every actual exit, log and
whole output, including failed or partial attempts.

## Output, failure and state scope

The output is deterministic gzip JSONL. Successful framing is `run_start`,
`initial_state`, `completed_interval` ordinal 1, then `run_end`. The file is
claimed exclusively and its initial record made durable before fixture
construction. Existing files are never replaced. Errors/panics retain prior
records and attempt an explicit failed terminal; a caught step panic also attempts
the available partial native observations. Terminal writing and gzip finalization
can fail separately. Only actual process exit0 proves completed collection and
finalization; it is not a numerical PASS.

Native f32 values are raw IEEE words, including signed zero. Companion/stage
objects preserve their public serde shape and explicit scalar width. Existing
impulse telemetry exposes promoted/aggregate f64 values; its projection retains
their u64 words and optional absence without labeling them as native f32 storage.

Checkpoints retain sorted individually serialized native bodies, collider and
unique multibody, exact integration parameters, and exposed raw body/joint/
generalized-velocity words. Native object postcard bytes must decode with exact
consumption and reencode identically through their source-bound types. This
roundtrip is not an independent decoder of private layouts. Public mass/inertia
query words are distinguished from the motor constructor's cached matrix.
Full-set metadata, broad/narrow phase, private pipeline/solver/contact caches and
observer buffers are excluded; replay claims are limited to this captured scope.

## Qualification status

The integrated example passed formatting, strict Clippy, warnings-denied rustdoc,
seven world-free parser/claim/framing/opaque-byte tests and a release build. These
checks executed no native fixture and do not qualify observer neutrality or
constructor/integration arithmetic.

The fixed six-process cohort completed exactly once, with every original process
exit 0. The frozen reader passed all declared checks: captured initial/final
ON/fresh/OFF states match exactly by sign, fresh ON completed records match
literally, and OFF observations are absent. All 1,028 companion events and
3,328 v1 stage events are accounted for. The 128 constructor and 128 coordinate
checks pass in the declared source-ordered normal/zero domain, with no exclusions.
Independent pre-data and postcollection reviews preserve the original byte hashes,
process exits and previously blocked pre-data driver versions.

The last slice's directly recorded velocities illustrate why the phases matter:

| Target sign | Integration input (rad/s) | Final writeback (rad/s) |
|---|---:|---:|
| Negative | -0.0243901238 | -0.0312500075 |
| Positive | 0.0243901238 | 0.0312500075 |

A separate unchanged-example-141 72-world source-transition run preserves every
report byte except its precisely declared source object. It retains scalar
accuracy FAIL 0/4 and zero-target passes 8/8; this six-process diagnostic does
not replace that criterion. The inverse-(w+g) intervention remains rejected.
See the [whole data and retained provenance](../../docs/evidence/motor-construction-v1/README.md).
These observations verify the stated captured scope and source arithmetic,
not full private-state equality, motor accuracy/convergence, runtime float
registers, a physical force/contact cause, hardware behavior or greater maturity.
