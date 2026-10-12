# Contact and motor stage collection

This example records one of example 138's unchanged **held** Rapier cases. The
plant, spawn order, collider mass behavior, ECS synchronization and controller
are copied from 138. Each invocation settles for 250 steps, lifts for 500 and
holds for 500 at 500 Hz. The original 1.25 squeeze margin, lift stiffness
100,000 N/m, damping 10,000 N s/m and ceiling 5,000 N remain fixed. It uses
Rapier's existing default settings; it does not tune gains or solver iterations.

```sh
cargo run --locked --release -p contact_motor_stages --example 142_contact_motor_stages -- \
  --mass-kg 5 --observer on --label baseline-5kg-on \
  --output /tmp/baseline-5kg-on.jsonl.gz
```

All four flags are required exactly once. The mass is exactly `5`, `20` or `50`,
observer is `on` or `off`, and labels contain 1–64 ASCII letters, digits, `_` or
`-`. Output is created exclusively; an existing artifact is never replaced.
`--help` creates no physics world. A multi-run driver and its source/compiler/
workload registration are separate from this single-case collector.

The stream is UTF-8 JSONL compressed with pinned flate2, zero gzip mtime, OS 255
and no filename/comment. Only one completed frame is buffered for serialization.
Gzip finishing and the underlying writer's flush are required before the CLI
prints its physical row and `contact motor stage collection: ok`. This means
collection and finalization completed; it does not establish motor accuracy,
contact convergence, accepted grasp behavior or a cause of changed motion.

The three record types use `schema_version: 1`:

| `kind` | Retained fields |
| --- | --- |
| `run_start` | `run_label`, `mass_kg_ieee754_bits`, `observer_enabled`, expected 1250 steps and 6 entities, ECS word version, codec/scope and original conditions as IEEE words/counters |
| `completed_step` | `completed_outer_step` (1–1250), typed `native_checkpoint`, exact `native_checkpoint_postcard_bytes`, `ecs_physics_ieee_words` and actual `stage_telemetry` object or OFF `null` |
| `run_end` | `status` (`collected` or `failed`), completed-step/frame counters, error or `null`, physical report or `null` |

The checkpoint preserves each full serialized native body, collider, unique
multibody and impulse joint, stable identities, gravity and integration
parameters. Its postcard payloads are source-bound native bytes checked by the
backend through exact decoding and reencoding. Stage telemetry retains its
actual enum names, widths, offsets, native coefficient/impulse/velocity words,
cache and phase metadata; no residual is computed by this example. The complete
checkpoint wrapper is also encoded with postcard 1.1.3 without normalizing
fields. Equality has this declared scope: full set metadata, broad/narrow-phase
state, private pipeline/solver/contact caches and ordinary rigid-body solver
storage are excluded. Aggregate contact stages do not expose individual contact
Jacobian columns.

ECS words begin `[1, entity_count]`, with entities sorted numerically by their
full `entity.to_bits()`, matching native joint bindings. Each entity has its
identifier, Transform presence then
translation xyz/quaternion xyzw/scale xyz words, RigidBody presence then motion
type (0 dynamic, 1 fixed, 2 kinematic), mass and linear/angular xyz velocity
words, then JointState tag (0 absent, 1 fixed, 2 prismatic, 3 revolute) followed
by position/velocity words for tags 2 and 3. All ECS scalars are raw f64 words.
The existing ECS quaternion conversion is part of the plant; saved native data
is never normalized. Absence and signed zero remain distinct.

The final physical report retains ten cube-pose words and four words ordered
as lift (m), palm rise (m), slip (m), tilt (rad). Its `physical_stdout_row` uses
138's Rapier formatting after removal of only the timing field, with no trailing
newline. There is no numerical pass marker. Raw per-step replay and the full
original 133–138 compatibility gates remain separate qualifications.

On collection error or an unwinding panic, prior frames are retained and a
failed terminal is attempted before finishing gzip. Step and frame counters
remain separate; a partially serialized or missing frame cannot become a
collected run. Fatal I/O may prevent a terminal or valid footer; finalization
errors return nonzero and the partial file is kept. The independent driver must
retain process exit/error receipts and reject truncation rather than infer
success from an internal terminal alone. Unit tests use synthetic data only
for strict parsing, bit preservation, deterministic gzip and failure handling;
they create no native physics worlds.
