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
