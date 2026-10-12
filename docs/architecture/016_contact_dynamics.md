# 016 — Hard-contact time stepping (RaiSim reference)

## Status

Implemented. `rne_dynamics::contact` (with `contact_step_coupled`),
`rne_physics::terrain`,
`rne_physics_native`, `rne_locomotion`, `rne_ai::VectorizedEpisode::step_parallel`,
examples 133–137.

## Context

`rne_dynamics` (see [012](012_dynamics.md)) gave RNE the articulated-body
quantities — `M(q)`, `h(q, qd)`, Jacobians — but stopped short of contact. Its
`constrained_forward_dynamics` and `impulse_velocity` treat every contact as a
bilateral equality, so a foot can pull on the ground and never slips. The
compliant model in `rne_oc` is a penalty spring whose stiffness trades
penetration against step size. Simulating a legged robot natively, without a
physics backend, needs unilateral contact with Coulomb friction.

RaiSim (Hwangbo, Lee and Hutter) is the reference simulator for this regime: it
is the engine behind a large share of legged-locomotion reinforcement learning,
and its speed and accuracy come from a small number of design decisions rather
than a large feature set. This ADR records which of those decisions RNE adopts
and which it deliberately does not.

## Decision

### Adopted

1. **Per-contact iteration with bisection** (`solve_contact_impulses`). Contacts
   are solved in impulse space on the Delassus matrix `G = J M⁻¹ Jᵀ`, one
   contact at a time in Gauss-Seidel order. Each local problem is solved
   exactly into one of three cases — open, sticking (impulse inside the cone),
   or sliding (impulse on the cone boundary, zero normal velocity, tangential
   velocity opposing the impulse). The sliding direction is found by bisection
   on its angle. The cone is the exact Coulomb cone, never a pyramid, and the
   normal condition is a hard Signorini constraint, not a spring. This follows
   "Per-Contact Iteration Method for Solving Contact Dynamics" (IEEE RA-L 2018).
   Two additions serve heavy, redundant contact such as a grasp, where an
   object squeezed between two pads gives 24 constraint rows over about nine
   degrees of freedom and `G` is singular:
   - The sweeps also stop once the contact velocities settle (`max |G δλ|`
     per sweep, `velocity_tolerance_m_s`, default 1e-8 m/s): along `G`'s
     null space the impulses keep shifting long after the motion has
     converged, and that shift changes no velocity.
   - An optional normal compliance (`regularization`, relative to each
     contact's effective inverse mass) makes the normal impulses unique, so
     a pad's load spreads over its corners instead of following the sweep
     order. A lopsided split's friction twists a held object. Friction stays
     rigid, so a held object does not creep. `NativeBackend` uses `1e-3`; the
     dynamics crate defaults to the exact `0`.
2. **Implicit PD in generalized coordinates** (`JointPdControl`). RaiSim
   integrates its joint PD controller implicitly by folding the gains into the
   effective mass, `M̃ = M + dt Kd + dt² Kp`. RNE does the same inside
   `contact_step`, so very stiff position control is stable at ordinary step
   sizes. This is the RNE answer to the stiff-motor problem measured in
   [height field terrain](../HEIGHT_FIELD_TERRAIN.md), where an explicitly
   driven foot outruns the contact within one step.
3. **Contacts handed over before they touch.** A contact carries a signed gap.
   A positive gap lets the point close that distance within the step before
   the contact pushes, and a negative gap is recovered at a configurable
   fraction per step. Candidate contacts can therefore be generated with a
   margin and kept in a fixed order, which makes warm starting (also adopted)
   index-stable across steps.
4. **Contact impulses as first-class output.** Every step returns the world
   impulse and the resolved state of each contact, the data RaiSim exposes per
   contact for rewards, logging, and estimation.
5. **Fractal terrain generator** (`rne_physics::FractalTerrain`). RaiSim's
   `TerrainProperties` turns a few noise parameters into a height map, which
   legged training uses to randomize the ground per episode. RNE's version is
   seeded fBm over gradient noise with optional step quantization, producing an
   ordinary `ColliderShape::HeightField`, plus `height_field_surface` to sample
   height and normal without a physics backend.
6. **Joint limits as constraints in the same solve.** RaiSim treats joint
   position limits like contacts in its solver rather than as springs.
   `contact_step` does the same when `enforce_joint_limits` is set (the
   default): every finite revolute or prismatic limit
   (`ArticulatedModel::joint_position_limits`) becomes a unilateral,
   frictionless block whose normal row is `±1` on the joint coordinate, with
   the distance to the limit as its gap. It shares the speculative-gap and
   recovery rules of contacts, so a joint stops exactly at its limit even under
   a PD target far past it, and the step reports each engaged limit's impulse.
7. **`O(n)` forward dynamics** (`aba`). The articulated-body algorithm is
   RaiSim's per-step forward pass. `aba` matches the dense solve to round-off;
   `contact_step` still factors `M̃` densely, because the Delassus matrix needs
   `M̃⁻¹ Jᵀ` column by column and the implicit PD gains modify `M̃`.
8. **Actuator effort limits.** RaiSim clamps each joint's generalized force to
   its actuation limit. With `enforce_effort_limits` (the default), a joint
   whose feed-forward plus implicit PD torque would exceed its URDF effort
   (`ArticulatedModel::joint_effort_limit`) is held at the limit: its PD is
   taken out of the implicit solve, the clamped torque is applied explicitly,
   and the step is solved again (at most four extra solves). A saturated actuator is
   a constant torque without PD damping, as on the real motor. Each step reports
   the applied `actuator_torques` and the `saturated_joints`.
   `JointPdControl::effort_limits` adds a per-command limit; the tighter of it
   and the URDF limit holds.
9. **A simulation backend, not only a step** (`rne_physics_native`). RaiSim is a
   whole simulator; `NativeBackend` makes the step one too, behind
   `PhysicsBackend`. Every tree of dynamic bodies joined by
   `RevoluteJointDesc`, `PrismaticJointDesc`, or `FixedJointDesc` becomes one
   reduced-coordinate model (a lone body is a one-body model with a floating
   base), so a URDF robot wired by `attach_urdf_document_articulation` — the
   path the asset loader takes — runs unchanged. Moving colliders are sampled
   (spheres exactly, capsules as rows of spheres, boxes, hulls, and meshes
   by their vertices) against static planes, boxes, spheres, capsules, height
   fields, triangle meshes, convex hulls, and compounds of these; `JointActuation` position, velocity, and effort commands become
   implicit PD and feed-forward forces with their effort limits, and
   `JointPassiveDynamics` adds damping and Coulomb friction. The backend
   advertises `RigidBody`, `Articulation`, `DeterministicStep`,
   `ContactForce`, and `RaycastBatch` and passes the external conformance kit
   for all five. Raycasts hit every non-sensor collider at its current pose:
   spheres, capsules, boxes, planes, and convex hulls as solids, height
   fields (the same
   bilinear surface the contacts use, solved exactly cell by cell) and
   triangle meshes as surfaces, and compounds part by part.
10. **Batched environments** (`VectorizedEpisode::step_parallel`). raisimGym
    steps many RaiSim worlds with OpenMP. `reset_parallel` and `step_parallel`
    step any `Episode` batch on scoped threads in contiguous chunks and gather
    results in environment order, so the result, the automatic resets, and
    the replay digest are identical to the serial `step` for any thread count.
11. **A legged locomotion stack on top** (`rne_locomotion`). The pieces RaiSim
    users build first: a quadruped scene from a Unitree-convention URDF with
    measured leg geometry and analytic leg kinematics, a feedback trot
    (level-frame foot plan leaned against the measured tilt, Raibert
    touchdown, stance feet swept at the command plus a share of its error, an
    integrated heading, and feed-forward stance torques that carry the
    weight), and `QuadrupedTerrainEpisode`, an `rne_ai::Episode` that draws a
    new terrain, heading, and command at every reset and takes a residual
    joint action on top of the trot (or on top of the stand pose).
12. **One contact problem per world** (`contact_step_coupled`). RaiSim solves
    every contact of a world in one per-contact iteration, so a robot
    standing on a box that stands on the ground feels the ground through the
    box. `contact_step_coupled` steps any number of models together: a
    contact joins a point on one model's link to the static world or to a
    point on another link — of another model, or of the same model for
    self-collision — and its Jacobian is the relative velocity of the two
    points. The effective mass stays block diagonal (one factored block per
    model), joint and effort limits work per model as in `contact_step`, and
    with one model and only world contacts the two agree to round-off.
    `NativeBackend` builds these contacts between moving bodies (samples
    against spheres, capsules, boxes, triangle meshes, and compounds; box pairs by a full separating-axis
    test over the 6 face and 9 edge-pair axes, with the corners and edge
    crossings of the overlapping faces on a face axis and the closest points
    of the two edges on an edge-pair axis), between links of one robot that a joint does not join
    directly, filters them with `CollisionGroups`, and solves each island of
    touching assemblies in one call. A triangle mesh gets a bounding-volume
    tree, built once per mesh allocation and shared by contacts and rays; a
    sample's gap to it is its distance to the closest triangle, signed by
    that triangle's outward (counter-clockwise) face normal. Contacts between
    two entities along one normal are reduced to the deepest and the three
    that span the widest support around it, so a body standing on many
    coplanar points (a table's sixteen leg corners) keeps the per-contact
    solver converging.

### Prescribed moving bases in contact

`BaseMotion` supplies the inertial effects of a moving fixed base to RNEA/ABA,
but its velocity is not an actuated generalized coordinate. Contact stepping
therefore uses `J qd_free + v_prescribed`, with each point's world velocity
`v_base + omega_base x (point_world - base_origin)` projected into the contact
frame. Coupled contacts add this contribution for side `a` and subtract it
for side `b`. Floating bases already carry their motion in generalized
coordinates and receive no extra contribution. Joint-limit rows stay relative
to their joints. Zero prescribed point velocity adds no arithmetic, preserving
stationary examples and their exact replay.

Analytic unit-point-mass fixtures verify translation and rotation at an offset
point on a translated, rotated base through both stepping APIs. Two moving
bases verify the relative signs and cancellation of common motion. Native
backend regressions grip 5/20 kg cubes, lift them 0.2 m, and then move a
kinematic cart up to 0.15 m along the squeeze axis and 0.12 m along the friction
axis, turning up to 0.4 rad. A one-second quintic profile starts and stops at
rest and exercises acceleration and deceleration. The cube's pose relative to
the palm stays within 2 mm and 0.01 rad throughout the motion, and repeated
runs match a fixed-order trajectory digest.

The cart fixtures use centered, independently position-controlled fingers
(`1e5 N/m`, `1e3 N s/m`, force limits 160/600 N; target closing force 80/300 N
per finger). Equal-and-opposite effort-only fingers, as in the unchanged static
grasp fixtures, leave a free common translation along the squeeze axis: moving
the palm does not mechanically center those fingers or carry the object along
that axis. Position servos model a centered gripper rather than hiding that
degree of freedom by altering the solver. Before the velocity fix, all five
cart-motion regressions fail with these same centered fixtures.

### Not adopted

- **RaiSim's general collision pipeline, server, and visualizer.** RNE
  already owns collision geometry, rendering, and transport; the contact step
  takes contact points as input, so any broad phase can feed it. The native
  backend's contact generation is a point sampler with a box-box corner
  test, not a general narrow phase.
- **Material-pair tables.** Friction is per contact here; a pair table is a
  scene-level policy that belongs where contacts are generated, not in the
  solver.
- **Restitution and spring/wire elements.** Inelastic contact is what legged
  locomotion needs; elastic impacts and wire constraints are left for when a
  scene requires them.
- **raisimGymTorch's Python/PyTorch side.** The batch steps in Rust behind
  `rne_ai::Episode`; tensor exchange with a learner stays with `rne_py` and the
  accelerator contract.

## API

```rust
let step = contact_step(&model, &q, &qd, &tau, &contacts, Some(&pd), &config, warm)?;
// step.q, step.qd, step.contacts[i].impulse_world_n_s, step.contacts[i].state
```

- `ContactPoint { link, point_local_m, normal_world, gap_m, friction_coefficient }`
- `ContactStepConfig { step_time_s, penetration_recovery, solver }`
- `solve_contact_impulses(G, v_free, mu, config, warm)` for callers that build
  their own Delassus matrix.
- `integrate_configuration` advances a floating base on the rigid-body
  manifold (body twist to `(x, y, z, roll, pitch, yaw)`), replacing the naive
  `q + dt qd` that is wrong for the base orientation.

## Dependency boundary

Unchanged from 012: `rne_dynamics` depends only on `rne_math`, `rne_ecs`,
`rne_world`, `rne_robot`, and `rne_physics` components. The terrain generator
adds no dependency to `rne_physics`; it is seeded explicitly so callers derive
the seed from `WorldRandom`.

`rne_physics_native` depends on `rne_physics`, `rne_dynamics`, `rne_robot`,
`rne_world`, `rne_ecs`, `rne_core`, and `rne_math`, and on no other physics
engine. `rne_locomotion` adds `rne_ai` (for `Episode`) and `rne_urdf_import`
(to build its quadruped) and depends on no physics backend or renderer. Both
are unpublished, like `rne_dynamics`; the published `rne_ai` gains only the
parallel batch methods.

## Validation

Unit tests in `rne_dynamics::contact` pin:

- the open, sticking, and sliding closed forms of a single contact;
- Signorini complementarity, the exact cone, and maximum dissipation on 200
  random coupled single contacts and 20 random six-contact problems, and that a
  warm start from the solution converges in at most two sweeps;
- a box on a 20° incline that stays put (to 1 µm over one second) when
  `μ = 0.5 > tan 20°` and slides with exactly `g (sin θ − μ cos θ)` when
  `μ = 0.2`;
- a tilted box dropped from 0.2 m that lands flat with no bounce and less than
  1 mm of penetration;
- an implicit PD pendulum at `ω dt ≈ 32`, far beyond the explicit stability
  limit, settling at the target minus the analytic gravity sag;
- floating-base integration of body-frame velocity and yaw unwrapping;
- a pendulum falling onto its lower limit that stops there without passing it
  and whose limit torque equals the gravity torque `m g l cos q`;
- a PD target 0.5 rad past the upper limit at `kp = 10⁶`, which holds the joint
  at the limit with a limit torque balancing the PD pull and gravity, and
  reaches the target once limits are disabled;
- a joint started 0.1 rad past its limit being driven back to it;
- a stiff PD pendulum on a 5 N·m actuator that cannot hold 9.81 N·m: the
  applied torque never exceeds 5 N·m and the arm swings undamped past the
  angle where 5 N·m balances gravity, while a 20 N·m actuator holds it level
  with exactly the gravity torque;
- a 100 N·m feed-forward torque clamped to 5 N·m, giving the corresponding
  acceleration, and passing through unclamped when limits are disabled.

- a 100 N·m feed-forward torque under a per-command limit of 2, 0, 8, and
  unbounded N·m on a 5 N·m actuator, applying 2, 0, 5, and 5 N·m.

Unit tests in `rne_dynamics::aba` check the articulated-body algorithm against
the dense solve at 20 random states on a fixed-base and on a floating-base
branching tree with revolute, prismatic, continuous, and fixed joints, skewed
axes, offset origins, and full inertia tensors. Example 103 repeats the check
on the 18-DoF Go2 at a moving state.

Unit tests in `rne_physics::terrain` pin seed reproducibility, the amplitude
bound, step quantization, input validation, and exact plane recovery by the
surface sampler.

`examples/133_go2_contact_terrain` drops the 18-DoF Go2 0.42 m onto seeded
fractal terrain with stand-pose implicit PD and checks that all four feet end
in contact, the contact force carries the weight within 2 %, penetration stays
under 5 mm, the robot comes to rest, and two runs are bit-for-bit identical.

Unit tests in `rne_physics_native` pin free fall to semi-implicit Euler, a
tipped box landing flat on a box ground with its contact event carrying
`m g dt` within 0.1 %, a box held on a 0.3 rad slope by `μ = 0.6` and sliding
at `μ = 0.2`, a pendulum whose anchor stays put to 1 nm while it falls onto its
limit, position and limited effort commands, ECS pose edits that teleport a
body, and rejection of a prismatic command on a revolute joint; one test runs
the external conformance kit, which passes every advertised capability.

Unit tests in `rne_locomotion` pin the Go2's measured leg geometry against its
URDF, leg inverse kinematics against forward kinematics on every leg, the base
state against the integrated motion, the tilt's continuity across the heading
wrap, standing with the contacts carrying the weight, and the trot tracking
0.4 m/s on rough terrain, turning past 1.8 rad, and recovering from a push.
`rne_ai` checks that the parallel batch equals the serial one for 0 to 16
threads.

`examples/134_go2_terrain_trot` trots the Go2 for 8 s over seeded fractal
terrain with an open-loop diagonal gait on implicit PD, inside the Go2's
declared effort limits. It covers about 2.8 m (0.35 m/s) with roll and pitch
under 0.15 rad and the body at least 0.27 m above the ground, saturating an
actuator in about 1.5 % of the steps, and replays bit-for-bit. Five terrain
seeds give 2.66–2.80 m.

`examples/135_go2_feedback_trot` drives the Go2 with the feedback trot over
0.09 m of fractal relief: 0.36 m/s against a 0.4 m/s command, a 2.08 rad turn
against a 2.0 rad command, and a 250 N, 0.1 s lateral push after which the
body is back within 0.023 rad of level one second later (the open-loop gait of
example 134 is still tilted 0.229 rad and cannot turn).

`examples/136_go2_parallel_rl` runs 32 terrain episodes for 300 control steps:
none falls with the zero residual, random residuals earn less reward, and the
parallel batch replays the serial one bit for bit.

Unit tests in `rne_dynamics::contact` pin the coupled step against
`contact_step` for one body (a box landing, and a stiff PD pendulum on its
effort limit, to 1e-12), a box resting on another box with the ground carrying
both weights and the box between carrying one (to 1e-6), a frictionless
head-on collision ending at the momentum-conserving common velocity, and a
self-contact between a pendulum and a floor on its own base. Unit tests in
`rne_physics_native` add a three-box stack whose contact events carry the
weight above each interface within 0.1 %, two balls colliding inelastically,
and a hinged flap folding onto a non-adjacent link of its own chain and
stopping at the analytic contact angle (and reaching its limit when its
collision groups filter everything). A plank laid crosswise on a beam, where
no corner of either lies over the other's face, rests on the four edge
crossings with the beam carrying its weight within 0.1 % (before the edge
tests it fell through the beam), and two cubes balanced edge on edge meet at
the closest points of their edges. A box rests on a two-triangle floor and a
ball on a closed mesh cube, each contact carrying its body's weight within
0.1 %, and a ball rests on a free table made of a compound of five boxes, the
floor carrying both. The mesh tree's closest points and ray hits match a
brute-force search over every triangle. Convex hulls are built incrementally
from their points (closed, convex, and outward for a cube cloud with interior
points and a 60-point sphere cloud; none for flat clouds); a sample's gap to
one is exact above a face, past an edge, and inside, a box rests on a fixed
hull slab and a ball on a free hull block (each contact within 0.1 % of its
load), and rays hit hulls as solids. An arm hanging from a kinematic cart that
drives 1 m while turning a quarter turn keeps its shoulder on the cart to
1e-9 m at every step and holds its joint angle in the cart's frame. On a cart
accelerating at 2 m/s², a damped pendulum settles back at the analytic
-atan(a / g) = -0.201 rad within 2e-3 rad (it hangs straight down, at 0,
without the base motion). In `rne_dynamics`, a fixed base given the motion of
a floating base yields that floating base's joint forces to 1e-9, and ABA
inverts RNEA under the motion.

A palm on a lift joint with two effort-driven finger pads (friction 0.8)
squeezes a 10 cm cube and lifts it 0.2 m. With 40 N on a 5 kg cube and 150 N
on a 20 kg cube (needed: 30.7 N and 123 N), the cube rises, slips under
1e-4 m, and twists under 1e-3 rad; with 20 N and 100 N it slips out and
stays on the floor. Without the normal compliance, the 5 kg cube twisted
0.26 rad and slipped 9 mm, and the 20 kg one 1.35 rad. The velocity stop
cut the step from about 1.6 ms to 0.3 ms. Two coincident contacts on a point
mass show both additions in isolation: the exact solve gives one contact the
whole load, the compliant one splits it to within 1 %, and the velocity stop
reaches the same motion in under half the sweeps.

`examples/138_heavy_grasp` runs the same grasp through `PhysicsBackend`, with
the gripper as a multibody, on `NativeBackend` and on Rapier, for 5, 20, and
50 kg cubes. With a 25 % squeeze margin the cubes rise with the palm on both
backends: natively with slip under 0.03 mm and tilt under 1e-3 rad, on Rapier
with slip growing to 1.5 mm and tilt to 6e-3 rad at 50 kg. With 35 % too
little squeeze every cube stays on the floor on both. The native step costs
0.15–0.4 ms here against Rapier's 35 µs.

`examples/137_go2_native_backend` builds the Go2 the way the asset loader does
and drives it through `PhysicsBackend` on `NativeBackend` and on Rapier: on the
native backend it lands on all four feet, its contact events carry its weight
within 1 %, it comes to rest, and it replays bit for bit, at about 50–100 µs
per step including the ECS synchronization. Dropped instead onto a free 8 kg
crate beside a free five-box tower, it stands on the crate with its feet
carrying its weight and the crate carrying both onto the ground (each within
0.1 N), while the tower does not move. A ray cast down onto the standing
Go2's base hits the base, and 81 rays cast onto the terrain land on its
bilinear surface to round-off on the native backend (Rapier, which
triangulates each cell, lands within 0.2 mm).

## Step cost

Profiling example 138 with Callgrind 3.24.0 on the #391 baseline attributes
17.82 billion of 23.04 billion instructions (77.3 %, including the Rapier
comparison in the total) to `solve_contact_impulses`. The sweep's contact
velocity sums and sliding-direction sampling dominate this solver; assembling
the Delassus matrix is outside that measured 77.3 %.

The sweep now traverses contiguous Delassus rows and reuses its previous-impulse
and velocity-change buffers. Each scalar product and addition remains in the
original order, including zero terms. Sliding bracket endpoints are cached on
the stack and evaluated only when their bracket is visited, with the original
nearest-first order, angles, bisection budget, and degeneracy fallback. This
removes two allocations and unused trigonometric evaluations per sliding solve;
it does not add warm starting, reorder contacts, or change convergence settings.

Callgrind counts fall to 16.93 billion for the complete example (26.5 % less),
with 11.70 billion inside `solve_contact_impulses` (34.3 % less). Ordinary
release measurements on an AMD EPYC 9V74 cloud host, Rust 1.95.0, are below.
Each entry is the median of five alternating baseline/candidate runs pinned to
CPU 0; validation builds were restricted to CPUs 1–4. Times include the
example's ECS synchronization and all 1250 simulation steps per case, not just
the solver. The baseline is `82602d22` (#391).

| Cube | Squeeze | Before (µs/step) | After (µs/step) | Reduction |
|---|---|---:|---:|---:|
| 5 kg | held | 326 | 248 | 23.9 % |
| 5 kg | under-squeezed | 208 | 165 | 20.7 % |
| 20 kg | held | 319 | 254 | 20.4 % |
| 20 kg | under-squeezed | 323 | 273 | 15.5 % |
| 50 kg | held | 148 | 129 | 12.8 % |
| 50 kg | under-squeezed | 361 | 297 | 17.7 % |

Reproduce normal timings with
`cargo run --locked --release -p heavy_grasp --example 138_heavy_grasp`.
For instruction counts, build the same target with
`CARGO_PROFILE_RELEASE_DEBUG=1`, then run
`valgrind --tool=callgrind --callgrind-out-file=callgrind.out target/release/examples/138_heavy_grasp`
and inspect `callgrind_annotate --inclusive=yes --auto=no callgrind.out`.
Examples 133–138 retain their printed physical metrics and replay checks;
an independent-block solver test compares combined and individual impulses
bit for bit, including warm seeds, open contacts, frictionless contacts, and
sliding contacts.

A step evaluates forward kinematics once and shares it between the mass
matrix, the bias forces, the contact Jacobians (walked only along each contact
link's ancestors), and the base integration. The Delassus matrix is filled
symmetrically and skips empty rows, and a joint limit joins the solve only
when it lies within 1 mm (or 1 mrad) of the joint or within twice the distance
the unconstrained motion covers toward it in one step; a joint pushed past a
limit that was left out is caught and recovered on the next step.

The recursive algorithms (`mass_matrix`, `rnea`, `aba`, and the step) run on
dynamics bodies rather than links: every link welded to its parent by a fixed
joint is folded into the nearest moving ancestor's spatial inertia when the
model is built, so the Go2's 42 links become 13 bodies. The effective mass
`M̃` is factored with Featherstone's tree-sparse `LᵀL` factorization, whose
factor and solves walk only each coordinate's ancestors (`M̃` keeps the
branch-induced sparsity of `M` because implicit PD only adds to its diagonal).

Example 133 (Go2, 18 velocity coordinates, four feet, 24 joint limits) runs at
about 9 µs per step in a release build. The first optimization pass (one
forward-kinematics pass, ancestor-only Jacobians, reachable limits only) took
it from about 94 µs to 15–17 µs, and merging welded links and the
tree-sparse factor halved that again; callgrind counts about 0.19 M
instructions per step, against 2.8 M originally.

## Limitations and follow-ups

- Forward kinematics still visits every link, including welded ones, because
  contact points and sensors may sit on any link.
- `NativeBackend`'s contacts come from samples except between boxes:
  a sample near a sharp convex mesh edge (faces turning by more than 90°)
  can take the wrong side of an open or non-convex mesh (convex hulls decide
  the side by their face planes, so they are exact).
  Compound parts are tested by sampling, not by the box-box test. URDF robots spawned without
  self-collision share one collision group and so do not collide with each
  other, as in the Rapier backend. A fixed-base tree on a moving kinematic
  body takes the body's motion from its poses between steps, one step
  behind, so a teleported body kicks the tree once. Contact velocities include
  the prescribed base's own point motion as described above.
- The floating base's roll-pitch-yaw coordinates have a confined middle angle.
  In RNE's Y-up world that angle is the heading, so turning past a quarter turn
  flips roll and yaw by π (the pose stays right). `NativeBackend` and
  `rne_locomotion` therefore simulate in a Z-up frame, where the heading is
  the yaw coordinate.


## Static heavy-grasp comparison evidence

Example 138 exposes `--benchmark-json native|rapier|mujoco`; the optional
MuJoCo 3.9 adapter compiles the same ECS scene. The repeated
[comparison](../HEAVY_GRASP_COMPARISON.md) preserves unchanged task thresholds
and failed cases. It separates final-output repetition from task correctness
and labels timings as setup-inclusive ECS stepping. Native implicit PD,
Rapier motors, and MuJoCo's explicitly evaluated typed PD have different
discrete dynamics; equal gains are not proof of equivalent control. The
recorded run is an adapter diagnostic, not a best-tuned solver ranking.


## MuJoCo bounded typed feedback

The recorded static comparison above used explicitly sampled typed PD and
remains historical evidence. The ECS compiler now uses scalar general
actuators with fixed gain and affine position/velocity bias for typed
`JointActuation`. The entire feedback law is force-limited inside MuJoCo,
so actuator damping remains part of measured `actuator_force`, separate from
passive joint losses. `implicitfast` can account for velocity feedback; this
is not the native solver's fully implicit position-and-velocity update.

For typed-actuation updates that leave passive dynamics unchanged, the
actuator helper updates only coefficients and force ranges under the world's
data mutex. Model dimensions, signature and transmission remain unchanged.
Passive-dynamics changes, including returning to a legacy motor with different
damping, retain the existing model-rebuild path.
Disabled or zero-limit commands clear both feed-forward
and feedback. Legacy commands retain zero actuator bias and existing passive
damping; caller MJCF keeps the sampled-control path. Tests cover high-gain
lightweight tracking, dynamic commands, force bounds with passive losses,
model invariants, and bit-exact replay. These single-joint tests do not prove
heavy-grasp success or bit identity with the previous integration method.

Same-scene example 138 measurements with the official MuJoCo 3.9 runtime
show improved palm tracking, but all three held-mass cases still fail the
original grasp criteria. Under-squeezed cases remain accepted as intentional
failures. Contact traces and bounded Noslip/timestep experiments do not
establish a unique cause or an optimal setting; no solver or task threshold
changes are included in this control update.

### Rapier prescribed kinematic multibody root motion

The vendored Rapier correction keeps the adapter's public API unchanged. It captures
position-based kinematic root endpoints before forward kinematics, then follows
body-origin linear translation and spherical rotation during serial solver
substeps. COM velocity includes angular transport. Because each interval has a
constant prescribed origin/angular velocity, boundary velocity changes enter once
as a generalized momentum correction; repeating that acceleration during the
interval would be inconsistent with the prescribed pose trajectory. Rotational
COM centripetal acceleration remains part of the within-interval dynamics.

Generic contact builders include prescribed root point velocity in their normal
and tangent right-hand sides, including substep updates. Analytic free-slider
acceleration/deceleration and contact-transfer fixtures pass. A wall fixture with
nonzero initial relative velocity fails when the prescribed contact term alone is
removed, so its success requires the contact correction rather than only the
boundary momentum update. Serial final velocity writeback uses final link poses.

Unchanged example 138 static Native/Rapier physical values match the stored
pre-investigation values bit for bit. The unchanged moving-cart fixture now passes
Rapier root following, but all six Rapier grasp cases still fail the original
criteria; root following does not establish grasp success. Native passes six cases.
All twelve cart trajectories replay exactly. Additional tests cover actual CCD
splitting, the island-parallel feature, graph split/append and type transitions,
and offset-COM rotating step-boundary serialization continuation. Generic
Jacobians resize per-island buffers in parallel builds; the clean baseline's four
matrix-bounds test failures are removed. Serialized fields remain unchanged, but
restoring in the middle of an active integration interval is not supported: a new
physics step must reconstruct the prescribed root command. Arbitrary in-solver
graph mutation and broader constrained/armature models remain outside the validated
scope. Release API audits remain separate checks. See the vendor `RNE_PATCH.md`
for the repository-build boundary and unchanged upstream provenance.

### Kick comparison physical-control and visualization boundaries

Example 139 uses dedicated dynamic Go2/G1 Rapier scenes with source positive
URDF masses, COM offsets and inertia tensors. Joint-origin rotations, welded
fixed children and preserved collision parts make the feet part of the plant;
G1 retains all four spheres on each sole. Frame-only links receive 1e-9 kg and
diagonal 1e-12 kg·m² inertias to avoid singular numerical marker bodies. Total
masses are 16.087000011 kg and 34.133857284 kg. Go2's initial thigh/calf stance
is an equivalent joint-coordinate shift with origin rotations and position
limits transformed together; its physical geometry and limits are unchanged.
The fixture generator and source/derived hashes are local to example 139.
Existing scenes retain their previous model defaults.

Both plants receive a 40 N·s lateral world-space body wrench over 80 ms,
67% above the earlier 24 N·s README capture.
Midpoint half-sine samples are normalized by their commanded impulse sum;
changing the step rate preserves impulse and duration. The application point
is the current root position plus 0.06 m along world Y, so the backend includes
the force's moment arm. Default physics uses 1 kHz and 32 solver iterations.

Unit-bearing ForceBased implicit PD enforces each joint's authored effort cap.
Rate-limited, position-bounded targets hold a wider stance: Go2 hip abduction
±0.45 rad and G1 hip roll ±0.24 rad. Go2 filtered IMU feedback changes opposing leg
lengths. A slow stance loop filters normal loads and adjusts bounded,
zero-sum calf biases toward COM-based shares of a rectangular foot-support
approximation; this is not general contact-wrench optimization. The Go2
IMU ablation retains that stance loop. G1 ankle/hip feedback uses COM/DCM
and body attitude. Its hip correction combines capture-point error with
opposing pelvis-roll and angular-rate feedback. The hip-attitude gain is
0.5; the 1.0-gain prototype passed nominal recovery but failed late-onset and
500 Hz cases. A larger sustained attitude term can reduce static COM support
margin despite its opposing instantaneous motor reaction. An outward, contralateral
shoulder-roll reaction responds to measured attitude and rate, holds its
excursion until observed motion settles, and returns slowly. The same stance,
motor limits and gains are retained in each feedback-off ablation. These are
controller design parameters, not identified hardware settings. The example's
Rapier COM helper sums dynamic robot links in name order at their declared
COM offsets. Rapier `linvel` already measures body COM velocity; adding an
angular-offset velocity again would double count it. The controllers consume
current ideal simulated observations and cannot read the disturbance schedule.
They neither edit robot poses/velocities nor apply stabilizing body wrenches.
Recorded motor ceilings describe solver-enforced limits, not torque readback
or a calibrated hardware actuator model.

Rapier contact evidence composes the collider world pose with the manifold's
optional compound-child pose before transforming each local witness. The
world contact point is the midpoint of those witnesses, and relative surface
velocity is measured at that point. Omitting the child pose misplaces compound
sole contacts and their angular velocity contribution; a rotated, translated
compound regression covers both witnesses and canonical entity ordering.

Capture checks plant mass/inertia, attachment integrity, upright recovery and
foot-only ground support. Every foot must exceed 0.5 N mean normal load
over the final 0.5 s; instantaneous contact counts are recorded separately.
Repeated runs compare literal ordered observed
link poses/scales, linear/angular velocities, joint states and foot impulses
through settlement and recovery. Hidden backend caches are outside this
assertion. Rendering uses those recorded poses without altering dynamics.
The broader `--validate-recovery` matrix retains zero-input, rate, onset,
reverse-direction, matched feedback-ablation and front/oblique limit cases.
Ablations keep posture PD, stance, gains, caps and target-rate limits. Whether
a disabled balance loop recovers is measured rather than assumed. GIF encoding
requires all 12 required recovery cases in the complete two-plant report.
The final 40 N·s run passes all 12 with exact observed replay. The full
18-case report retains three directional failures: both front impacts and
G1's oblique impact.
The earlier controllers toppled under the same 40 N·s nominal pulse; the
wider-stance controllers recover, with roughly 0.545 m lateral base displacement
for Go2 and 0.008 m for G1. The final G1 nominal probe recovers at 32 N·s
and falls at 48 N·s, with both outcomes retained. Stronger probes retain
measured outcomes without establishing a safe maximum. Recovery permits
translation and does not imply position retention.
Front/oblique cases retain their measured outcomes as envelope probes.
Rate checks establish recovery under those discretizations, not convergence
of the complete trajectory or hardware kick resistance. The native contact
solver is unchanged by these example-specific controllers and fixtures.

The CC0 human is a visual actor with fixed-length rotation-only limb chains,
support-foot planting and staged knee chamber, extension and retraction.
Rotation-only IK generation targets 0.322176/0.826094 m strike heights,
within 0.01 mm of the final nominal wrench points; a continuous monotone
time map shortens extension to
175 ms, holds the strike for the 80 ms force pulse, and retracts over 220 ms.
The rig, bone lengths and support-foot path remain fixed.
Shortest-path glTF LINEAR rotation interpolation is shared by CPU and GPU
skinning. Human-foot collision impulses and reaction-driven human motion are
not computed. The GIF illustrates a prescribed lateral disturbance experiment;
it is not a coupled simulation of a human kicking physical hardware.

Compiled trace provenance includes controller, capture, renderer, plant and human
GLB inputs. The encoder checks those hashes against the current files and
independently integrates every recorded force sample to verify the common
impulse, direction, timing and normalized pulse shape. Old rendering inputs or
mixed-strength validation reports cannot qualify a new public GIF.

Example 139 also has an opt-in synthetic state-estimate observation probe.
An example-local collector captures all active outer-control inputs from one
completed physics tick: attitude/rates, mass-weighted COM position/velocity,
foot positions and Go2 normal loads. The existing timestamped `Frame`/`DataBus`
boundary supplies only arrived samples. Feedback calculations accept the captured
estimate without reading current world state. Sample identity and keyed error
remain fixed during zero-order hold; no arrived startup frame means bounded
nominal stance without an oracle fallback. The schedule, queue and sample index
continue across settlement and the presentation-time origin.

The probe varies capture period, fixed arrival latency and bounded signed-uniform
estimate errors while physics and controller integration remain at 1 kHz.
World-seeded keyed draws and canonical channel ordering preserve replay without
depending on hash-map iteration. The ideal profile preserves the existing
arithmetic and the 500 Hz qualification still uses its original physics step.
Diagnostic results retain failures, including pre-impact settlement failure,
and use the same physical recovery predicate. Replay includes literal plant
words and separate observation/controller/pending-state words. No generic
sensor component, backend interface or core dependency boundary changes.

These are declared simulation sensitivity settings, not measured Unitree noise
or latency. Instantaneous simulated joint feedback remains inside implicit
position PD; raw sensor fusion and motor-loop degradation are outside this
probe. Its evidence is separate from the ideal-observation GIF qualification,
with compiled inputs and trace hashes rather than relabelled historical media.
The measured fixed-seed matrix retains three G1 failures (5 ms latency, 20 ms
latency and bounded error); Go2 recovers in all eight profiles. Both combined
zero-input baselines recover, and all 16 plant/control recordings match fresh
replay. Combined-profile success does not imply a monotone tolerance boundary.
Serialization-only old/new runs across the original 18 ideal-observation cases
also match all ordered plant words and original controller decisions, including
settlement, while excluding hidden solver caches and new-only pipeline state.
The normal binary's original matrix separately passes all 12 required recoveries
and exact plant/control replay in all 18 cases. The compact measured proof is
retained in [the observation study](../media/kick-observation-study.json).

Example 139 additionally provides a matched G1 observation diagnosis. Four
fixed profiles (ideal, 5 ms delay, 20 ms delay and bounded error) are each run
with zero input and the same 40 N·s lateral wrench. The plant, seed, capture
schedule, controller limits and full recovery predicate remain fixed. This
comparison addresses the missing zero-input controls for the earlier three
failed profiles rather than changing the public recovery qualification.

The diagnosis streams bounded per-tick JSONL through settlement and recovery.
Arrived estimates and capture/arrival/consumer times remain the only outer
feedback available to control. Separately labelled evaluator truth and
original controller correction/target decisions share the pre-step tick;
contact-force samples retain their last-completed-step meaning. Evaluator
truth, disturbance parameters and future frames never become controller
inputs. Two fresh logging-on runs and a logging-off run compare actual
ordered plant/control words and original decisions; fresh streams are also
byte-compared. Paired pre-input trajectories are checked independently of
post-input recovery. No core API, solver setting or controller gain changes.

All eight measured cases settle upright. All four zero-input recordings
recover, while only the ideal-observation kicked recording recovers; delayed
and bounded-error kicked recordings reproduce the three earlier falls. Each
pair's 4,000-step pre-input prefix matches literally. These results locate the
failure within this fixed synthetic disturbance experiment without identifying
a physical sensor tolerance or a unique controller failure mechanism. The
[compact diagnostic evidence](../media/g1-observation-diagnosis.json) retains
all eight outcomes and the separate replay and compatibility proofs.

The separate known-age orientation experiment crosses those eight conditions
with raw and projected feedback. Projection is an example-local stateless
operation on the original arrived payload: integrate the captured world gyro
over capture age, left-multiply its quaternion, and recompute only orientation
axes and roll/pitch. Exact identity paths preserve zero-age and zero-rotation
observations; unsupported timestamps fail explicitly. Translation, rates,
feet, loads and joints retain their captured time scope. Raw and effective
feedback are separately labelled, and no evaluator truth or disturbance
schedule is admitted. No dynamics, contact solver, control gain, actuation
limit or core API changes are needed.

The predeclared primary gate is complete physical recovery for the 5 ms delayed
40 N·s case. All zero-input controls, zero-age cross-policy literal equality,
fresh replay, logging noninterference, matched prefixes and default recovery
qualification remain separate checks. A candidate that reduces orientation
error without passing this physical gate is not promoted. The candidate stays
an explicit opt-in software experiment; it is not a measured sensor model.

The completed 16-case fixed-seed experiment passes the primary gate: raw
feedback falls under the 5 ms delayed 40 N·s wrench, while the candidate
recovers with peak tilt 0.309082 rad instead of 1.638381 rad. Both policies
recover in all four zero-input profiles. The 20 ms delayed and bounded-error
kicked cases still fail. Literal full-trajectory words agree in all four
zero-age cross-policy conditions, and all eight matched pre-input prefixes
agree. Default feedback remains raw even after this primary success.
The [compact experiment evidence](../media/g1-orientation-age-experiment.json)
binds all 16 outcomes, replay/audit scopes and unchanged default qualification.

The separate lateral-gyro experiment compares orientation-only feedback with
a predeclared 5 ms EMA on controller-facing world gyro X at positive capture
age. Orientation projection still uses the original captured world gyro.
The controller owns constant-size state: first capture initializes raw X,
later updates use capture spacing, and held/missing frames leave history
unchanged. Zero-age feedback borrows the original payload. Other captured
channels, inner position PD, plant, gains, caps and recovery criteria stay fixed.

Initialized filter state contributes exactly 20 canonical control replay words;
uninitialized state contributes zero. Identity, raw fingerprint and numerical
state are replayed. Invalid timestamps, source transitions, held payloads or
arithmetic fail without committing state. Arrived estimates remain the only
outer feedback; evaluator truth and the prescribed input are excluded.

The 16-case primary gate requires full recovery at 20 ms delay and 40 N·s,
plus preserved 5 ms recovery, all eight zero-input recoveries, zero-age literal
equality/negative control, prior orientation references and default qualification
(18 cases, 12 required recoveries, examples 133–138). Replay, logging
noninterference and matched pre-input prefixes are separate gates. Failed
settlement retains only the available prefix, at most the planned 4,000 steps,
with zero actual applied dose and unavailable post-impact metrics.

Completion does not promote the candidate. Default raw feedback and the earlier
orientation mode retain their behavior. Experiment markers and non-raw nested
`feedback_policy_configuration.policy` cannot qualify public GIF encoding.
The study establishes no hardware sensor model or reliability claim.

The completed matrix fails both recovery advancement gates. At 20 ms and
40 N·s, both policies fall (peak tilt 1.639460 vs 1.635945 rad). At 5 ms,
the orientation reference recovers at peak tilt 0.309082 rad, while the EMA
falls at 1.638850 rad. All eight zero-input cases recover; four zero-age
conditions preserve literal cross-policy words and the bounded-error kicked
negative control remains unchanged. All 16 cases pass fresh replay and
logging noninterference; eight prefixes reach the full 4,000-step comparison.
An internal read-only 131,200-row audit verifies the recorded decisions and outcomes.
Earlier reference fields/hashes/counts and default qualification remain
compatible. The candidate is not promoted; the
[compact evidence](../media/g1-lateral-gyro-experiment.json) retains all failures.

### Opt-in completed position-motor readback

The optional neutral `JointPositionMotorDiagnostics` child-link component records
Rapier ForceBased single-DoF multibody position-motor impulse after completed
physics. `UrdfSceneSim::configure_position_motor_diagnostics` returns whether the
backend supports collection without advancing the clock. The default-off feature
is forwarded from example 139 through `rne_ai` to the Rapier backend; default
builds retain compatibility with the upstream dependency.

CI's isolated API-comparison workspaces do not inherit the root manifest's Rapier
patch. An owned temporary Cargo config supplies that same checkout fork to both
current and immutable-baseline API builds. Existing configuration is never
overwritten, and cleanup removes only the generated file. Feature selection,
baseline identity, and release checks remain unchanged. This consistent dependency
context checks API compatibility; it does not replay baseline physics or dependencies.

Example 139's orthogonal headless `--motor-effort-diagnostics` flag reads the
component after stepping, binding command-consumer `t` to completion `t+dt`.
Revolute N·m·s/N·m and prismatic N·s/N remain distinct. Both observed and
qualified-completed means divide signed impulse by actual observed solver time;
completed means require integer native CCD-partition/solver-slice coverage.
Nominal ticks, numerical outer duration, and observed durations are separate.
Missing samples are unavailable; zero ceilings and unbounded ceilings retain
distinct semantics. FINAL positive-finite-bound counts/durations describe actual
motor solver slices, independently of position/rate target clipping.

The evaluator sorts child-link names, streams a separate motor JSONL/hash, and
never adds readback to observations or controller replay words. Two fresh ON
worlds compare new motor words and actual stream bytes; the third explicitly OFF
world compares existing plant/control/decision values. Existing studies reuse
their logging-OFF third world; this does not separately test a motor-writer OFF
world. Recursive encoder and render-only guards reject this private evidence.

Fixed consumer windows `[2,4)`, `[4,4.08)`, and `[4.08,8.2)` report all 23 G1
joints. Interval means can cancel despite frequent FINAL bound occupancy; these
statistics are not within-slice torque peaks, hardware validation, or proof of a
fall cause. Actual source hashes/build receipts, not the baseline commit alone,
bind the opt-in readback. Default qualification and historical media remain exact.

The fixed 16-case/48-world instrumentation matrix preserves the prior 11 recovered
and five failed outcomes; no existing policy is promoted. An internal read-only
audit preserves actual old/new legacy rows and retained exposed values. All
131,200 first-ON rows contain 23 available full-step-qualified motor records;
measured zero interval means remain distinct from unavailable data. The
[compact interval/window evidence](../media/g1-motor-effort-diagnostics.json)
retains every joint, case, and support denominator. Original analysis paths are
historical context; current bindings distinguish 24 lossless gzip streams from
eight uncompressed streams with separate raw/stored digests and byte lengths.

### Isolated position-motor calibration

Example 141 isolates one ForceBased revolute motor on a uniform sphere, with a
fixed coincident parent, zero gravity/contact/passive damping/armature, a 2 N m
ceiling and a single 1 ms outer interval. Its fixed 24 conditions combine the
existing G1 posture/ankle gains, three small position targets, one/32 small steps,
and one/two PGS passes; two stabilization passes remain fixed. Two fresh ON worlds
and one OFF world per condition qualify observation and exact repeatability.

The first final slice is compared with the constrained fixed point of the
independently derived scalar row. Native-compatible f32 coefficients and RHS
are evaluated in the declared order and promoted for the independent comparison.
The predeclared requirement is an impulse error no larger than the greater of
1e-8 N m s and 1% of the reference magnitude. No gain or setting is selected from
the results. All failed, partial or nonfinite observations remain in the report.

Public mass recomputation and fixed-root kinematics initialize the fixture without
advancing time. Pre-step native effective angular inertia yields the scalar
inverse inertia for this coincident isotropic model. The public post-step LU is a
separate latest-cache observation, not a captured first-slice matrix; its agreement
is checked separately. Neither inference applies to G1's coupled contact problem.
Actual serialized native joint coordinates and ordered native state/telemetry
words preserve signed zero, nonfinite bits, integer coverage and absence.

Collection, replay qualification and numerical calibration have separate outcomes.
`--require-calibration` retains the complete report before returning a failing exit
when the numerical requirement is unmet. Verifying a retained report runs no new
worlds. This experiment does not alter the solver, production gains, G1 model,
recovery predicates, or prior kick/GIF evidence, and supplies no hardware claim.

The retained run completes all 72 worlds; all 24 conditions have finite exposed
state, qualified actual slice coverage, matching independently rounded/native
coefficients and the separately acquired inertia cross-check. ON/fresh/OFF native
entity state is exactly equal and ON/fresh telemetry is exactly equal. All eight
zero targets remain motionless. Every nonzero condition fails the fixed first-slice
tolerance, including all four primary N32/PGS1 conditions:

| Gain pair (k,d) | Target (rad) | Fixed-point impulse (N m s) | Measured final first-slice impulse (N m s) | Absolute error (N m s) | Allowed error (N m s) |
|---|---:|---:|---:|---:|---:|
| (300,12) | -0.001 | -8.570856e-6 | -6.250000e-5 | 5.392915e-5 | 8.570856e-8 |
| (300,12) | +0.001 | +8.570856e-6 | +6.250000e-5 | 5.392915e-5 | 8.570856e-8 |
| (250,8) | -0.001 | -7.352520e-6 | -6.250000e-5 | 5.514748e-5 | 7.352520e-8 |
| (250,8) | +0.001 | +7.352520e-6 | +6.250000e-5 | 5.514748e-5 | 7.352520e-8 |

These are the vendored Rapier internal ForceBased position motor's isolated
finite-sweep results, not a change to the RNE native contact solver. N32/PGS2
also retains a wrong-sign first impulse for both gains and signs; N1 sensitivity
controls are retained without selecting a production setting. The full
[report](../media/position-motor-calibration.json) preserves every case, run, native
serialized byte, replay word and ordered slice. Collection/verification success
does not replace the failed numerical criterion or establish a cause of G1 falls.

Schema 2 uses postcard 1.1.3 with strict whole-input consumption for native entity
bytes. Dependency policy rejected the initial bincode version as unmaintained;
no advisory exception was added. The original
[schema-1 report](../media/position-motor-calibration-bincode-generation-1.json)
remains byte-identical and retains its original source/binary binding. One new
preregistered 72-world generation changes only serialization and provenance.
All 9,072 exposed state IEEE words, actual joint/inertia/LU Scalars, 48 ON
telemetry records with 792 ordered slices, absence and numerical outcomes match
exactly across generations. Complete native bytes are independently roundtripped
and compared ON/fresh/OFF within each generation; differing byte encodings do not
prove every private field identical across codecs. The
[migration receipt](../media/position-motor-calibration-codec-migration.json)
records both reports, source/binary bindings and the pre-measurement comparison
tool. The current verifier explicitly rejects old schemas and stale sources.

### Rejected ForceBased inverse-diagonal intervention

A separately preregistered candidate changes only the internal single-DOF
ForceBased inverse diagonal from `inv(w)` to `inv(fl32(w + g))`, using the existing
CFM gain `g`. AccelerationBased retains its original expression. The unchanged
example 141 fixture executes one new 72-world matrix with frozen independent
oracle/audit tools and source/compiler/binary bindings. Primary calibration passes
4/4, all nonzero cases pass 16/16, and zero controls remain motionless 8/8. Exact
ON/fresh/OFF state and ON/fresh telemetry qualify within the candidate build.

Admission also requires examples 133–138's complete physical stdout to match the
declared baselines after only the existing three timing exclusions. All six
programs print `ok`; 133–137 match, but four Rapier grasp rows in 138 differ.
For the 50 kg held cube, lift changes 0.196 to 0.191 m and tilt changes 0.0057 to
0.023 rad. Native grasp rows are unchanged. This is a required compatibility
failure, so all three candidate files are rolled back without retuning or
relaxing the gate. A fresh post-rollback release run restores all six baselines.

The [historical evidence](../evidence/position-motor-diagonal-experiment/README.md)
retains the full matrix, exact rejected patch, failed unit compilation and
independent admission rejection. The attempted vendor unit tests execute no
worlds: compilation fails with private-module imports. Direct strict vendor lint
and docs do not have a passing receipt. These limitations are separate from the
rejected physical gate and from validation of the final evidence-only change.

The original two report files remain byte-identical to their published versions;
the candidate adds a third generation without replacing them. The current
source-bound verifier deliberately rejects this historical candidate report after
rollback; that evidence-only change verifies the original calibration FAIL. A passing
scalar calibration supplies no coupled G1 recovery, hardware or maturity claim.

### Strict validation of the vendored solver

Before adding stage observations for coupled grasp, direct warning-denied
validation of the unchanged vendored Rapier reproduces 39 Clippy diagnostics
and a rustdoc failure. Workspace dependency builds cap upstream warnings, so
their success alone does not establish standalone vendor qualification.
Mechanical cleanup removes ineffective annotations and unused private graph
helpers, spells elided return lifetimes explicitly, fixes documentation layout,
and uses equivalent derived defaults and integer predicates. Motor and contact
equations, iteration order and solver settings are unchanged.

Two cleanup files participate in example 141's selected-source binding:
`multibody.rs` changes lifetime spelling and a malformed documentation link;
`rigid_body_components.rs` removes
ineffective `must_use` attributes. Preserve the published reports byte for byte;
the unchanged verifier must reject their stale source binding. A separate
[current-source report](../media/position-motor-calibration-vendor-lint.json)
collects the unchanged fixed 24 conditions and 72 worlds. A comparison reader
frozen before collection permits only those two source hashes to change; every
other report field must match the original schema-2 report literally. CI verifies
the new report through the same verifier and unchanged numerical criteria.

This source-transition qualification is separate from the proposed contact-stage
measurement. It changes no calibration outcome, identifies no coupled failure
cause, and supplies no recovery, hardware or maturity claim. Validation commands
and actual outcomes are retained in the
[strict-vendor evidence](../evidence/rapier-strict-validation/README.md).

### Optional contact and motor stage observation

The `rne_physics_rapier/motor-stage-diagnostics` feature adds a backend-specific,
default-off observer without extending the backend-neutral physics trait. It is
separate from completed-step motor-impulse diagnostics. Toggling invalidates the
last completed trace; only a completed pipeline step publishes a new trace.
Default backend builds retain compatibility with unmodified crates.io Rapier.

The observer binds supported internal single-DOF motors through their builder,
link, free-axis and unique `Motor(0)` identity. It reads rows only after the
current joint update initializes them. Each row retains its actual full
multibody `ndofs2`, generalized-velocity offset and stored J/weighted-J block,
including native coefficient, accumulator and bound words. Complete ordered
events distinguish joint solves, immediate motor visits, bias removal,
integration/cache refresh and aggregate contact stages. Island buffers drain
in island order after the existing workers join. Native solve, bias-removal and
contact arithmetic retain their original operations and traversal order.

Axes are native joint-frame axes. The collector's palm joint is authored along
world Y, but the prismatic builder rotates its local X axis into that direction;
its recorded motor axis is therefore `LinX`. Supported rows also include native
finger motor rows; the observer does not select only the palm.

Example 142 copies example 138's held-grasp ECS plant and synchronization. It
streams every completed outer step into deterministic gzip JSONL, retaining
typed checkpoint fields, exact postcard bytes and ECS IEEE words. Checkpoints
cover individually serialized bodies, colliders, unique multibodies, impulse
joints, gravity, integration parameters and sorted ECS joint bindings. Full set
metadata, private pipeline/contact caches and ordinary rigid-body solver arrays
are outside this declared scope. Errors preserve prior frames and require an
explicit failure terminal when I/O permits it; gzip finalization errors remain
nonzero process failures.

For an unchanged cached motor row, independent analysis uses the exact-rational
reference `r = rhs + J V - g lambda` and `w = J W`, with conditional projection
`clamp(lambda + r / (w + g), lower, upper)` when the denominator is finite and
positive. A contact-only interval requires literal equality of row identity,
cache generation, J/W, RHS, coefficients, reciprocal, accumulator and bounds.
Its net residual change can then be compared with `J delta_V`. Source-ordered
f32 replay is assessed separately, with explicit floating-mode and subnormal
coverage. Saturation complementarity remains distinct from a zero residual.
Aggregate observations do not identify individual contact coupling columns or
establish a coupled failure cause. See the
[collector contract](../../examples/142_contact_motor_stages/README.md).

The fixed baseline/rejected-comparison matrix collected all eighteen worlds and
22,500 completed outer steps. Its original reader and two separately retained
post-collection metadata repairs fail: after the native handle and local-axis
interpretations are repaired, genuine finger rows violate the frozen palm-only
row gate. Keep this coupled numerical qualification failed/unavailable; do not
infer complete observer replay, contact reopening, a failure cause or candidate
admission from collection completion. The earlier inverse-diagonal candidate
remains rejected. The [retained evidence](../evidence/contact-motor-stages/README.md)
preserves the failures, late repair provenance and exact raw-trace hashes.

A separately preregistered generation 5 cohort admits all three source-derived
motor roles before eighteen new runs. Its one frozen audit passes captured-object
and ECS replay, ordered telemetry, and source-ordered normal/zero arithmetic:
22,500 outer steps, 2,160,000 row snapshots, 270,000 immediate motor visits and
540,000 aggregate contact-row intervals, with zero qualification issues. Its
exact complementarity classifications still retain 720,000 violated palm
snapshots across all observed phases; arithmetic fidelity does not establish
motor convergence. Absolute RHS construction, per-contact causes, full private
state and runtime float registers remain unmeasured. The
[new cohort evidence](../evidence/contact-motor-stages-generation5/README.md)
preserves all six complete detail streams externally and leaves generation 4's
failed outcome and the candidate's rejection unchanged. All eighteen new gzip
hashes match the original manifest, permitting byte-identical reconstruction of
the approved raw TAR. Persistent local retention is verified; upload returned
HTTP 401 and public download verification remains pending.

The separately collected
[earlier contact-stage scalar report](../media/position-motor-calibration-contact-stages.json)
retains the fixed 24 conditions and 72 worlds. Its selected-source object changes
six hashes and adds four reviewed inputs. Independent verification permits only
that object to differ from the vendor-lint report; every other raw report byte
matches. Collection and replay pass, the four primary nonzero conditions still
fail, and all eight zero targets pass. Historical reports and numerical criteria
remain unchanged; at that revision CI verified its source binding with the same verifier.

### Actual motor construction and coordinate integration

The separately versioned, default-disabled construction companion records the
actual single-DoF constructor coordinate, targets, gains, duration/inverse
duration, limits, clamped target velocity and ERP, alongside its initialized
version-one row. Inputs come from the constructor itself, including the original
motor descriptor's gain model and effort ceiling. Later endpoint snapshots keep
actual multibody velocity distinct from generalized solver velocity.

Coordinate integration copies generic velocity into the named multibody before
advancing its coordinate. Joint stabilization then changes generic velocity while
multibody storage retains the integration copy; final writeback copies the final
generic velocity. A final velocity cannot substitute for the earlier integration
input. In example 143's last slice, the two target signs use approximately
±0.024390124 rad/s for integration and write back ±0.031250007 rad/s.

Six preregistered fresh processes, two signs times ON/fresh ON/OFF, pass exact
captured initial/final-state replay and literal fresh-ON completed-record replay.
Their one frozen audit accounts for 1,028 construction events and 3,328 unchanged
v1 stage events, with 128 constructor and 128 coordinate-advance checks, no
arithmetic exclusions or qualification issues. Native words remain intact.
This verifies source-ordered normal/zero arithmetic and the declared captured
scope; private caches/full-set state and runtime float registers remain unmeasured.
It does not establish motor accuracy, convergence or a coupled-contact cause.
See the [collector contract](../../examples/143_motor_construction/README.md)
and [complete retained evidence](../evidence/motor-construction-v1/README.md).

A separate, unchanged 24-condition/72-world source-transition run retains
calibration FAIL 0/4 and motionless zero targets 8/8. Its source map adds the
companion module (21→22 entries) and changes exactly eleven existing hashes.
Every other byte matches the earlier scalar report. CI uses the same verifier
with the [new complete report](../media/position-motor-calibration-motor-construction-source.json);
old report bytes, numerical criteria and the rejected inverse-diagonal candidate
remain unchanged. Pre-data storage-fault reviews and corrected once-only drivers
are retained separately from the actual successful collections.

### Motor visit placement under public solver counts

A separate external study derived from example 143 varies only the public
internal PGS/stabilization counts: 1/2 (`split_1_2`), 1/0 (`early_1_0`) and
3/0 (`early_3_0`). It uses the unchanged source at PR #410 head `5e8e3627` and
the same isolated ForceBased fixture. Each profile/sign has ON, fresh ON and
OFF, giving eighteen original processes in a new cohort. The collector remains
an external study prototype; no new workspace example or invocation is added.

All original native/collection exits and the one frozen audit exit are zero.
The audit accounts for 2,572 construction-companion events, 8,192 v1 stage events
and 896 immediate motor visits, with 384 constructor and 384 coordinate-advance
checks in the declared source-ordered normal/zero domain. Captured ON/fresh/OFF
checkpoints match exactly within each profile/sign; fresh ON completed records
match literally and OFF observations are absent. There are no arithmetic
exclusions or mismatches. The preregistered first-slice discriminator is
INCONCLUSIVE for both signs; an unequal first-slice result was not required for
the fidelity gate.

The positive target's last slice (31) of the one 1 ms interval records:

| Profile (PGS/stabilization) | Coordinate after interval (rad) | Integration input (rad/s) | Final writeback (rad/s) |
|---|---:|---:|---:|
| `split_1_2` (1/2) | 2.43952054e-5 | 0.0243901238 | 0.0312500075 |
| `early_1_0` (1/0) | 2.43952054e-5 | 0.0243901238 | 0.0243901238 |
| `early_3_0` (3/0) | 2.34375093e-5 | 0.0312500075 | 0.0312500075 |

In this fixed fixture the no-stabilization one-PGS profile preserves the captured
coordinate and makes writeback equal the integration input. The three-PGS
profile has a different coordinate endpoint and equal integration/writeback
velocities. The recorded visit phases and bias-removal stages remain part of
these comparisons; they do not establish a general coupled-contact cause or
justify replacing production settings. Physical SI references are diagnostics
only. Calibration FAIL 0/4 and the inverse-(w+g) candidate's rejection remain
unchanged; this study recollects no earlier scalar worlds and changes no vendor
equation, controller or selected example-141 source binding.

The [whole evidence and provenance](../evidence/motor-visit-placement-v1/README.md)
retain all eighteen full raw streams. The complete 5,447,072-byte audit is retained
losslessly as one 886,291-byte gzip member; full restoration is bound to the
original audit SHA/length, without splitting or selecting fields. Public
retention is prepared separately from the scientific qualification. Captured
state/source arithmetic do not qualify full private caches, runtime floating-point
modes, physical motor accuracy, hardware behavior or increased practical maturity.

### Warehouse fleet visualization boundary

Example 140 demonstrates 40-robot logistics using `rne_nav::TrafficCoordinator`
and seeded `rne_world::WorldRandom`. Its directed routing, task dispatch,
scenario energy accounting, and bounded kinematics are example-local. Traffic
leases and conservative swept circles provide logistics clearance gates;
they do not compute wheel traction, contact impulses, or physical load pickup.
The native contact solver and physics backends are unchanged.

The renderer replays accepted recorded poses and task state. Actual charging
arrivals, delivery completions, waiting, and closure rerouting drive the HUD.
Exact full-state replay and source/pixel bindings qualify this simulation
capture within the tested runtime; they do not establish hardware fleet safety
or cross-platform bit identity. See the
[example instructions](../../examples/140_warehouse_fleet/README.md).
