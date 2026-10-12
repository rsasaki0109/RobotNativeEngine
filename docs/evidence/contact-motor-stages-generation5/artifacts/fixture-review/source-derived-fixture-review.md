# Generation 5 pre-data fixture and row admission review

This is a source-only review of the unchanged example 142 collector and current
Rapier observer. It defines prospective admission for a new 18-case cohort. No
builds, tests, worlds, native trace reads, old generation 4 numerical analysis or
checkout edits were performed. Generation 3 remains **REJECTED**. Nothing here
promotes that candidate, repairs an old result or changes examples 133–138 gates.
The machine-readable companion is `source-derived-fixture-spec.json`.

## Exact scene and all three internal motor rows

Source spawn order is floor, cube, fixed anchor, palm, side-minus finger,
side-plus finger. These ordinals are not runtime entity bits or native handles.
All six start with identity rotation and zero authored velocities. Example 142
preserves example 138's six-body scene, collider sizes/material, anchors, input
schedule, `sync_from_ecs → step → sync_to_ecs`, and physical display expressions.
Its additions are observer configuration and read-only completed-step output.

| Reader role | Authored axis / native row axis | Source-derived link / selector | Native model | Native stiffness / damping / motor cap | Native row reserve |
|---|---|---|---|---|---|
| palm | world/local Y / `LinX` | link 1 / coordinate 0 | `ForceBased` | 1e5 N/m / 1e4 N s/m / 5000 N | Motor(0), one row |
| finger_slot_0 | world/local X / `LinX` | link 2 / coordinate 1 | `AccelerationBased` | +0 / +0 / +0 | Motor(0), then Limit(0), two rows |
| finger_slot_1 | world/local X / `LinX` | link 3 / coordinate 2 | `AccelerationBased` | +0 / +0 / +0 | Motor(0), then Limit(0), two rows |

Both finger motor rows exist. Direct-effort commands call native `set_motor` with
zero gains and cap; `set_motor` retains the free-axis motor bit, and the stage
binding filter accepts these rows. Their descriptor model remains the native
AccelerationBased default. A palm-only or positive-cap-only filter is therefore
inappropriate for the new cohort. The nonzero squeeze is a separate equal-and-
opposite child/parent force pair, not the finger motor-row cap.

Per mass 5, 20 and 50 kg, squeeze is exactly
`1.25 * (mass * 9.81 / (2.0 * f64::from(0.8f32)))`. Preserve that f32-to-f64 friction
conversion. The 1250 completed outer steps at 500 Hz comprise 250 settle, 500
lift and 500 hold steps. Palm target is zero for settle, then
`0.2 * min(index as f64 / 500 as f64, 1.0)` for index 1 through 1000. The backend
casts command values to native f32. The default 4 small steps, 1 PGS pass, 2
stabilization passes and contact warmstart 1 remain unchanged. No gain,
iteration, mass, friction, timestep or controller retuning is proposed.

The ECS authored masses identify the dynamic palm (1 kg) and finger pair
(0.2 kg each); the selected-mass cube is distinct. Native bodies use authored
`additional_mass` plus default-density collider mass. The authored mass is not
an assertion of native total mass or inverse mass.

## Identity and full-block admission

`MultibodyJointSet::do_insert` uses the child rigid-body handle as joint handle,
updates rb2mb vector indices and appends the child subtree to the parent. The
fixed anchor is the sole final root. Each prismatic child has one free DOF;
fixed-root conversion removes the temporary six free-root coordinates. The
resulting vector topology derives palm link 1 and finger pair links 2/3, with
three contiguous generalized coordinates and selector `link_index - 1`, even
if a finger subtree is inserted before its palm is attached to the anchor.

The backend uses `Entity Ord` to create bodies/joints. Saved ECS frames and
checkpoint bindings instead sort numerically by complete `entity.to_bits()`.
The Bevy allocator source was not independently inspected in this bounded
review; no entity index/generation, handle value, anatomical left/right label,
island number, constraint order or solver offset is guessed. Reader slots are
named by link index only. Checkpoint bindings must join each observed ECS
prismatic role to its native child handle, unique multibody record and link.
Stage `joint_handle` JSON must be exactly `{index, generation}` (u32 fields),
while checkpoint `handle_words` is a separate two-u32 array grammar.

Admission must require exactly all three unique roles in each initialized
aggregate gripper group, and one immediate selected-row pair for each role on
each PGS/stabilization visit. Empty unsupported groups remain distinguished
from missing admitted rows. All three rows use their full `ndofs2 = 3` V/J/W
block; this is not a length-one scalar block. All captured offsets are checked
against observed vectors and group identity.

Builder order follows actual active-island handles. Derive `constraint_index`
and `j_id2` from that observed role order: palm reserves one row and each finger
two. Under the admitted no-external-joint fixture, joint J-buffer and generic
one-body constraint storage begin at zero; each reserved row consumes six
J/W words. Do not assign a fixed permutation to the three captured motor
indices. Joint `j_id2` belongs to the joint Jacobian buffer, not a contact
Jacobian buffer.

The source append implementation shifts populated child-subtree `internal_id`
metadata differently from vector/rb2mb indices. This review does not impose an
unproved `internal_id == vector index` gate. The new reader joins checkpoint
`link_index`, event `link_id` and the captured J selector. Native multibody
object bytes are source-bound opaque records; no independent full native
multibody/private-field decoder is claimed.

## Native sign, signed zero and cache boundaries

The actual internal row executes, in source operation order:

```
vel2 = J.dot(V)
dvel = rhs + vel2
lambda_next = clamp(lambda + D * (dvel - g * lambda), lower, upper)
delta = lambda_next - lambda
lambda = lambda_next
V.axpy(-delta, W, 1.0)
```

Native lambda is the negative of applied motor impulse. The residual sign is
`rhs + J*V - g*lambda`. For these three translational coordinates, V/RHS are
m/s, lambda is N s, W and ForceBased g are 1/kg, and D is kg. `cfm_coeff` is
captured but unused by this internal solve path. The native `inv` returns +0
in inclusive ±1e-20 and otherwise computes `1/x`; preserve native f32 operation
order and the actual rounded stored D. Baseline uses `D=inv(w)`; the rejected
candidate changes only ForceBased to `D=inv(w+g)`. AccelerationBased is unchanged.
Reader native stored-D projection and common reference projection must remain
separate, with formulas/tolerances frozen before any new data.

Finger bounds are **[-0,+0]** for a positive finite slice duration. Finger RHS
is not unconditionally zero: zero target velocity is clamped to the interval
`[(min-q)*inv_dt, (max-q)*inv_dt]`, and an out-of-limit q can make it nonzero.
A residual with a zero-width bound is not alone nonconvergence. Preserve all
signed-zero words; even a ±0 impulse delta still executes native axpy, so do
not invent an unqualified bitwise-no-write assertion for its V operation.

Each joint update initializes motor lambda to +0 and rebuilds J/W from its
actual then-current inverse augmented mass. Motor positional ERP is stored in
`rhs_wo_bias`, with `rhs=rhs_wo_bias`; native motor bias removal therefore leaves
those RHS words equal. Before/after external-force events expose no rows or
cache; captures become available only after the current slice's joint update.
After integration they still refer to that frozen row cache. Do not substitute
a later inverse mass or post-step checkpoint state for captured W.

End-of-outer-step ECS/native joint position is not row-build-time position.
Absolute RHS cannot be independently rebuilt from that final position with
this capture. Current telemetry supports actual row words and immediate
operation audits, not that unobserved RHS-input reconstruction. Aggregate
joint phases include both uncaptured limit rows; contact phases aggregate all
normal/friction/warmstart writes. They do not identify an individual contact
column or prove a palm-only causal explanation.

## New cohort prerequisites and declared scope

The parent declares baseline and `rejected_inverse_w_plus_g` builds, each with
masses 5/20/50 and on/fresh_on/off: **18 fresh processes/worlds, all unrun here**.
Compiler is declared rustc 1.95.0, target architecture x86_64 and RUSTFLAGS=[];
actual source, build profile, full target, features, command and binary hashes
remain the parent's prospective freeze responsibility. The source feature
chain is example142 → backend motor-stage-diagnostics → Rapier serde-serialize,
with repository Rapier defaults dim3/f32. No extra parallel/SIMD/determinism
feature is requested. Candidate application is the already specified single
ForceBased unit-row diagonal hunk; production rollback remains mandatory.

Before worlds, freeze the all-three-row independent reader and pure synthetic
qualification, both source/binary bindings, all 18 case identities/output paths,
execution order, resource ceilings and process receipts. Require 1250 complete
frames, complete gzip/CRC/footer/EOF, successful process/finalization and valid
terminal counters. Retain failures and partial output; do not select a passing
subset or tune criteria from outcomes.

Within each build compare every checkpoint JSON/wrapper postcard/native object
byte vector, integration/gravity words and ECS physics words literally across
ON/fresh ON/OFF. ON/fresh ON ordered stage words must match literally; OFF must
be null. Preserve case-label differences only as declared identity fields.
Physical final words/stdout are compared in their declared scope; collection
`ok` alone is not physical acceptance. Existing examples 133–138 exact physical
stdout preservation remains unchanged and separate. Observer equivalence does
not promote the rejected diagonal candidate.

The checkpoint excludes full-set metadata, broad/narrow phase, private
pipeline/solver/contact caches, observer buffers and fields skipped by native
serde. Exact equality establishes only the recorded payload scope, not equality
of all hidden state. There is no hardware, outside-user, accuracy, causal,
performance or maturity increase claim in this review.

## Source binding

Baseline checkout head: `2c3586fa9168f13c774fb8f0c0d2cd359eba1287`.
The 33-file source inventory and clean initial git status are retained in
`source-review-start.json`; completion verifies each byte hash and head/status
again. This is source identity evidence, not a compiled binary binding.

- Collector142 lib: `34f7e4003375925cd2b55132ab0bbf1f02c1c4daf4b5ba80a5459038b0326b58`
- Backend checkpoint schema: `c209bf5748dc6c5025bf17098d3974ece6610de4f435400c729a13b6be5764da`
- Native observer stage schema: `991bf9151f122e187bd9263b95dafff0bfe5210506fd22134bea6eb00ffb666e`
- Current unit-row constructor: `0e8eaa25f2ed772fc7de1e15b084c0761700d9f2afb5eeb17b73247af51246fe`
- Generic native row solve: `1cb1d7a50ddb67b0120e8181f652b593cc1afde68ebe8c366695a84bbed9cecb`

Historical `postcollection-native-frame-schema-repair-v2/stage_schema.py` was
read only as a schema/source reference. No historical report or native data was
read, edited, analyzed or reclassified.
