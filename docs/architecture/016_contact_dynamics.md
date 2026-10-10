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

Both plants receive a 24 N·s lateral world-space body wrench over 80 ms.
Midpoint half-sine samples are normalized by their commanded impulse sum;
changing the step rate preserves impulse and duration. The application point
is the current root position plus 0.06 m along world Y, so the backend includes
the force's moment arm. Default physics uses 1 kHz and 32 solver iterations.

Unit-bearing ForceBased implicit PD enforces each joint's authored effort cap.
Rate-limited, position-bounded targets hold a wider stance: Go2 hip abduction
±0.25 rad and G1 hip roll ±0.12 rad. Go2 filtered IMU feedback changes opposing leg
lengths. A slow stance loop filters normal loads and adjusts bounded,
zero-sum calf biases toward COM-based shares of a rectangular foot-support
approximation; this is not general contact-wrench optimization. The Go2
IMU ablation retains that stance loop. G1 ankle/hip feedback uses COM/DCM
and body attitude. The example's
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
a disabled balance loop recovers is measured rather than assumed; both do
recover in the recorded 1 kHz nominal probe. The complete matrix passes all
12 required cases. Lateral ±Z is the recovery envelope. At the same impulse,
front impacts topple both plants and the oblique impact topples G1. Those G1
falls include non-foot ground contacts and fixed-attachment rotation errors
above the declared tolerance. Go2 recovers from the oblique probe while its
COM translates approximately 0.211 m forward and 0.117 m laterally. These
observations do not establish position retention or all-direction resistance.
Rate checks establish recovery under those discretizations, not convergence
of the complete trajectory or hardware kick resistance. The native contact
solver is unchanged by these example-specific controllers and fixtures.

The CC0 human is a visual actor with fixed-length rotation-only limb chains,
support-foot planting and staged knee chamber, extension and retraction.
Shortest-path glTF LINEAR rotation interpolation is shared by CPU and GPU
skinning. Human-foot collision impulses and reaction-driven human motion are
not computed. The GIF illustrates a prescribed lateral disturbance experiment;
it is not a coupled simulation of a human kicking physical hardware.
