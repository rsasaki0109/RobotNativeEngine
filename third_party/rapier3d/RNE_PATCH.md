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

## Bit-identical multibody performance

Three changes make reduced-coordinate multibodies cheaper without changing a
single result bit. Rapier rebuilds both mass matrices and refactorizes them in
every solver substep, which made this the largest cost of a walking humanoid.

- **Column-sparse mass matrices.** `Multibody::update_inertias` now tries
  `rne_accumulate_inertias_sparse` before upstream's loop, which is kept
  unchanged as `accumulate_inertias_dense`. A link's body Jacobian and Coriolis
  terms are nonzero only in the columns of its ancestors' coordinates; on the
  12-DoF G1 legs that is 7–12 of 18 columns. With finite operands the dense
  `quadform`, `gemm` (3×3, below nalgebra's matrixmultiply threshold) and
  `gemm_tr` add exactly `+0.0` to every other entry, and those entries never
  hold `-0.0`, so the sparse loop skips them. It computes the kept entries in
  nalgebra's own operation order: `gemv` as `(α·a)·x + 1·y`, and each six-row
  dot product summed in row order from `+0.0`. If a Jacobian, mass matrix,
  Coriolis operand or `M·J` product is not finite, it returns `false` and the
  dense loop recomputes everything.
- **Leading-block copy.** The matrices passed to `LU::new` are copied column by
  column instead of through nalgebra's generic view iterator.
- **Unit Jacobian reuse.** A joint's limit and motor constraints on the same
  coordinate both store `[e_k, M⁻¹e_k]` for the same factorization. The second
  now copies the first's block instead of solving again. The public
  `unit_joint_limit_constraint` and `unit_joint_motor_constraint` keep their
  signatures.
- **Inactive limit rows.** Rapier builds a limit row for every limited
  coordinate at every step. Within the limits its impulse bounds are
  `[-0.0, +0.0]`, so every solve adds a signed zero times `M⁻¹e_k` to the
  velocities and changes none of them. Unless a motor on the same coordinate
  reuses its unit Jacobian block (an angular limit is built before its motor),
  such a row now keeps its slot and Jacobian space but gets no degrees of
  freedom: it skips the `M⁻¹e_k` solve, and
  `JointGenericOneBodyConstraint::solve` returns at once for it. Its impulse
  stays zero, as before. The one value it could change is an exactly zero
  solver velocity stored as `-0.0`, which the skipped `+0.0` addition would
  have made `+0.0`. `rne_physics_rapier` drops the motor axis of a joint driven
  by direct effort, whose zero motor would otherwise keep every limit row.
  Example 147's Go2 walks and example 144's G1 walks are bit-identical before
  and after; a walking Go2 steps 5% fewer instructions.

`rne_sparse_mass_tests` compare the sparse helpers with nalgebra's dense
routines bit for bit on random operands containing zero, `-0.0` and kept zero
columns. They also compare the complete sparse and dense accumulation on a
stepped free-root multibody with two branches (hinges about three axes, offset
anchors and centers of mass, and a prismatic link), including the Coriolis
workspaces, and check the dense fallback for NaN and infinite operands.
Reordering one dot product or one `gemv` product makes them fail. Example 144's
G1 walk records and example 141's 72 worlds are byte-identical before and after.

## Hill-climbed AABBs for large convex hulls

Parry computes a `ConvexPolyhedron`'s world AABB by transforming every vertex,
and the broad phase asks for two per collider and step (current and predicted
pose). A convex hull of a scanned link mesh has hundreds to thousands of
vertices; on the G1 with leg self-collision these scans took 22% of a step at
four solver iterations and 40% at one.

`src/geometry/convex_aabb.rs` builds, for a collider whose shape is a convex
polyhedron with at least 64 vertices, the hull's vertex adjacency from Parry's
edges. Each of the six bounds is found by walking that graph from the vertex
that attained it last time: a linear function on a convex polytope has no
local maximum on its edge graph that is not global. Candidates are
transformed with the same `Isometry * Point` product as Parry's
`point_cloud_aabb`, so the bounds are the same floats whenever the walk ends at
the vertex the scan picks; it could stop one rounding error short only where
adjacent vertices tie to within it. `Collider::compute_aabb`,
`compute_collision_aabb` and the broad phase's predicted-pose AABB use it;
`set_shape` rebuilds the graph, `shape_mut` drops it (the caller may change the
geometry), and serde skips it, so a restored collider scans until its shape is
set again. Smaller hulls and other shapes keep Parry's computation. The hint
also keeps the last pose it was asked for and that pose's AABB, and returns
the AABB again for a bitwise-equal pose: the broad phase often asks twice per
step for an unchanged pose.

`convex_aabb::tests` compare it with `point_cloud_aabb` over 2,000 tumbling
poses of a 3,000-point hull, with periodic restarts from arbitrary vertices,
and find no difference. Example 144's G1 and H1 walk records, with and without
self-collision, and its G1 push sweep are byte-identical before and after.

## Contact impulse totals

A contact point's reported impulse (`ContactData::impulse` and
`tangent_impulse`, read by `ContactPair::total_impulse` and by RNE's
`ContactEvent::impulse` and `ContactPointSample::normal_force_n`) is the
solver element's `impulse_accumulator + impulse`. Upstream adds the element's
impulse to the accumulator when each substep starts, the first included. At
the first substep that impulse is still the warm start carried in from the
previous step's last substep, so a step with n solver iterations reported n + 1
substeps' impulse: (n + 1) / n of what the solver applied, twice it at one
iteration. A block resting on the floor reported 1.25 times its weight at
four iterations.

Only warm-started constraints are affected. The generic two-body
(multibody against dynamic body) builder warm starts like the rigid ones;
the generic one-body builder (multibody against fixed geometry, such as a
walking robot's feet on the ground) starts every step from a zero impulse
and was already reported correctly. `OneBodyConstraintBuilder`,
`TwoBodyConstraintBuilder` and their SIMD variants, whose
`update_with_positions` the generic builders reuse, now accumulate only from
the second substep on (`solved_dt > 0`). Warm starting and the solve are untouched: with zero
restitution every trajectory is bitwise the same. The narrow phase treats a
contact whose last reported impulse is zero as new, which makes a contact
with restitution in (0, 1) bounce again if it carried no impulse over a whole
step and then collides; before, the carried warm start counted as an impulse
and kept it resting. RNE's
`resting_contact_forces_equal_the_supported_weight_at_any_solver_iteration_count`
checks that resting contacts report the supported weight within 1% at 1, 4
and 16 iterations, through ground and between two stacked bodies.
