# RNE Rapier armature and free-root COM patch

Based on the crates.io rapier3d 0.22.0 package (Apache-2.0, LICENSE retained).
The armature and free-root COM changes are in
`src/dynamics/joint/multibody_joint/multibody.rs`. `Cargo.toml` adds an empty
workspace so upstream tests run independently in nested research worktrees.

Adds a validated generalized-coordinate armature setter and a zero-initialized
armature vector. Armature follows damping's coordinate remapping on append,
split, growth and root dynamic/fixed transitions. Its diagonal is added to
both mass matrices before permutation/factorization. It introduces no force,
damping, link mass or external wrench. Zero armature skips the new arithmetic.
RNE exposes only a revolute multibody component, not Rapier types in core APIs.
Its backend feature `experimental-armature` gates calls to the patched setter;
default backend builds remain compatible with the unmodified upstream crate.

Source provenance is recorded in `RNE_UPSTREAM_SHA256.json`. Backend tests
exercise analytic acceleration, torque-limited implicit motor impulses,
invalid inputs and retained spatial mass. Changes to upstream should preserve
this contract or replace the patch with equivalent upstream support.

Run patch tests with `cargo test --manifest-path third_party/rapier3d/Cargo.toml --release --lib rne_armature_tests`.

The same module also fixes free-root translation coordinates for bodies with
nonzero local centers of mass. Rapier's root generalized linear velocity is a
COM velocity, so its integrated translation must locate that COM. The free
joint now places its second frame at the local COM and preserves body pose
when rebasing coordinates, reading a body pose, or changing root type.
This numerical correction applies to repository builds regardless of the
armature feature. Unmodified upstream remains API-compatible but does not
contain the correction. Zero COM retains the previous coordinate convention.
See `docs/adr/031-multibody-free-root-com.md` in the engine repository.

## Prescribed kinematic multibody root motion

The root-motion correction additionally changes the physics pipeline, serial velocity
solver, and generic one-body/two-body contact builders. Before the initial forward
kinematics pass it captures a commanded position-based kinematic root endpoint.
Each solver substep advances the same body-origin linear/spherical interpolation;
COM velocity includes angular transport. A change in the prescribed interval
velocity applies one generalized momentum correction, rather than distributing an
acceleration inconsistent with that interpolation. Generic contact right-hand
sides include the prescribed point velocity. Serial final writeback refreshes
moving-root body velocities at their final poses.

These numerical changes exist only in repository vendor builds. Public API
signatures remain unchanged; an unmodified upstream crate remains compatible with
the default adapter API but lacks this correction. `RNE_UPSTREAM_SHA256.json`
continues to describe the untouched upstream package, not these local patches.

The current 0.22 parallel feature uses island-level parallelism with per-island
serial solver buffers. Its generic contact/joint Jacobian builders now resize those
buffers regardless of the feature, removing a leftover condition for inactive
intra-island parallelism. The clean pre-correction backend fails four parallel tests
with matrix slicing bounds errors; the patched backend passes all 49 tests,
including prescribed-root contacts and an actual CCD-split fixture.

Private motion is ephemeral: its field is skipped by serde, and the prior body
velocity supplies the next boundary impulse.
Step-boundary bincode restoration with an offset COM and rotation passes exact
continuation checks, and the serialized field schema remains unchanged. Snapshot
restoration during a partially integrated step is not supported; resuming requires
a new physics step that reconstructs the root command. Graph
split/append and type-transition fixtures pass, including a former child root's
centripetal acceleration, but arbitrary graph edits during an active solver step
are not validated. Arbitrary locked axes, armature, and joint limits also need
broader physical coverage. Release API audits remain separate checks.

## Optional internal motor impulse diagnostics

The repository adapter's `position-motor-diagnostics` feature is disabled by
default, preserving its API compatibility with unmodified upstream Rapier. The
vendor `PhysicsPipeline` additionally requires an explicit runtime opt-in through
`set_multibody_motor_impulse_telemetry_enabled`; its completed getter remains
unavailable before the next completed step and clears on a configuration toggle.
The adapter qualifies explicit ForceBased single-DoF position motors. The vendor
collector observes only solved internal single-DoF motor constraints, excluding
contact and hard joint-limit impulses. Missing, inactive, or sleeping observations
remain unavailable; a solved zero-capacity motor is distinguishable as measured
zero and is ineligible for positive-bound activity.

Each final net motor impulse is captured exactly once after the small step's
PGS and stabilization passes. Applied generalized impulse has the opposite sign
from the internal constraint accumulator (`-impulse`); signed revolute/prismatic
fixtures and momentum checks qualify this convention. The collector never writes
its values into a motor accumulator or a future solve. The internal motor's
previously unused writeback tag now distinguishes Motor from Limit, while the
existing MAX joint-ID guard still prevents internal impulse writeback.

The completed report preserves the native outer dt, actual CCD partition
intervals, and each actual small-step duration/impulse/rounded bound promoted to
f64. Per-motor sums follow slice time order, with deterministic island-buffer
drain after a parallel join. Observed mean effort divides net impulse by summed
observed solver duration, separately from nominal ECS ticks. Full-step mean is
qualified only by exact-once integer slice coverage of every actual CCD partition,
including island-specific additional iterations. Final bound activity means
exact equality with a finite strictly positive native impulse bound; it does not
claim an attempted clip or explain a fall. Unknown infinite capacity remains
explicit, and malformed or nonfinite samples do not become measured zeros.

Observation state is ephemeral pipeline/solver workspace. It is disabled by
default and does not alter physics arithmetic, constraint order, warm starts,
serialized body/joint fields, or the step-boundary snapshot schema. The upstream
provenance and license remain unchanged; this section describes the additional
repository patch, not upstream functionality.

## Reconnecting a kinematic tree with a free-root subtree

A narrow integer-index repair lets a zero-DoF kinematic tree reconnect a free-root
subtree through a one-DoF joint without an intermediate unsigned subtraction.
Descendant assembly indices use their offset after the removed root coordinates.
Representable legacy damping views retain exactly their previous placement and
values. Only a negative mathematical legacy view start uses the new root joint's
bounded damping view; the old wrapped view was invalid and is not a result to
preserve. The general legacy damping placement remains unchanged for existing
defined cases. Regression tests cover the formerly invalid reconnection and the
literal damping values of a defined legacy append. This repair changes no solver
floating-point formula, motor gains, or constraint ordering.

## Construction and coordinate companion

This patch adds a separately versioned, default-disabled
`PhysicsPipeline` construction observer with its own enable/completed accessors.
It copies the actual internal single-DoF unit-motor constructor locals, including
coordinate, targets, gain model, gains, native duration/inverse duration, optional
limits, effective clamped target velocity and ERP. The nested version-one row
retains actual J/W/CFM/diagonal/bounds/identity; effort ceiling is copied from the
same motor descriptor passed to `motor_params`, not a later descriptor lookup.

Coordinate endpoints distinguish integration input from final stabilized and
written-back velocity. Before-position-integration observation occurs after the
generic-to-multibody velocity copy and before coordinate advance, per root.
Final writeback observation occurs after the final generic-to-multibody copy.
Uninitialized or unsupported observations remain explicit. Events drain in
partition/island/native order; there is no global mutable observation state.

The existing version-one stage public structs/schema and the physical fixtures
and numerical criteria of collectors141/142 remain unchanged. Example 141's
selected-source binding adds the companion module and updates the affected
source hashes. Constructor/solver arithmetic, gain defaults and constraint/
iteration order are not patched.

The original preparation checks constructed no native world. Their frozen
source/documentation snapshots remain historical evidence. A subsequently
preregistered six-process cohort completed and passed the declared captured-state
replay and constructor/coordinate arithmetic checks. A separate unchanged
72-world source-transition run retains scalar calibration FAIL 0/4. These results
do not establish full private-state equality, motor accuracy, convergence or
production readiness; the inverse-diagonal candidate remains rejected.


## Branched multibody island connectivity

`src/dynamics/island_manager.rs` keeps every reduced-coordinate multibody in a
single solver island when its root is fixed or kinematic. Dynamic-root traversal
is unchanged. Ordinary adjacency traversal stops at a fixed/kinematic
root, which can separate its dynamic branches (or let the first branch sleep
while another remains active). Internal joint constraints then reference a
multibody velocity slice that was not allocated for that island, causing a
matrix slicing panic. Only the root children are connected through link 1, the
existing solver allocation representative for a non-dynamic root, using linear
total traversal work. Ordinary traversal within each branch is preserved; adding
edges to all descendants also changed serial-chain solver ordering and regressed
the existing G1 workbench grasp smoke. This changes island membership/wake propagation, not contact
geometry or motor forces. Public APIs and serialized fields are unchanged.

The backend regression `kinematic_root_branches_share_a_solver_island` sets the
minimum island size to one so merging heuristics cannot hide the split. The
workbench contact-grasp test covers a three-wheel kinematic root, arm motors,
a separate dynamic object, sustained holding, release and exact input replay.
