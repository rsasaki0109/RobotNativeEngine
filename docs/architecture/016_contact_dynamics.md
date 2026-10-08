# 016 — Hard-contact time stepping (RaiSim reference)

## Status

Implemented. `rne_dynamics::contact`, `rne_physics::terrain`, example 133.

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

### Not adopted

- **RaiSim's object and collision pipeline, server, and visualizer.** RNE
  already owns collision geometry, rendering, and transport; the contact step
  takes contact points as input, so any broad phase (Rapier, an analytic
  ground, a height field sampler) can feed it.
- **Material-pair tables.** Friction is per contact here; a pair table is a
  scene-level policy that belongs where contacts are generated, not in the
  solver.
- **Restitution and spring/wire elements.** Inelastic contact is what legged
  locomotion needs; elastic impacts and wire constraints are left for when a
  scene requires them.
- **A vectorized environment wrapper (raisimGymTorch).** RNE's batched learning
  surface lives in `rne_ai` and the accelerator contract; this ADR concerns the
  dynamics only.

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
- floating-base integration of body-frame velocity and yaw unwrapping.

Unit tests in `rne_physics::terrain` pin seed reproducibility, the amplitude
bound, step quantization, input validation, and exact plane recovery by the
surface sampler.

`examples/133_go2_contact_terrain` drops the 18-DoF Go2 0.42 m onto seeded
fractal terrain with stand-pose implicit PD and checks that all four feet end
in contact, the contact force carries the weight within 2 %, penetration stays
under 5 mm, the robot comes to rest, and two runs are bit-for-bit identical.

## Limitations and follow-ups

- The step uses dense `O(n³)` factorization of `M̃`; RaiSim's speed also comes
  from exploiting the tree structure. An articulated-body (ABA) forward pass
  and a sparse `M̃` factorization are the natural next step.
- Joint limits are not yet contacts; they can be added as one-sided
  generalized-coordinate constraints in the same solver.
- No physics backend implements `PhysicsBackend` on top of this step yet; it is
  a dynamics-layer primitive used directly by examples and controllers.
