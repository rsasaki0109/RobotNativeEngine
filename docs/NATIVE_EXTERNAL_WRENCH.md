# Native backend external body wrench

2026-10-30. Native now advertises `PhysicsCapability::ExternalBodyWrench`.
This makes force-at-point disturbance and friction tests possible without
changing world gravity. The backend manifest engine version is
`coupled_contact_step_per_contact_bisection_v3`.

## Command contract

Sync the ECS world first, then call `PhysicsBackend::apply_external_body_wrench`
with an `ExternalBodyWrench` containing the dynamic body's entity,
`point_world_m`, `force_world_n`, and `torque_world_nm`. All vectors use world
coordinates. Torque is an additional free moment; the force's moment arm is
included separately. The application point remains the supplied world point,
so callers applying a persistent force at a moving center of mass must update
the point and submit the load before every step.

Commands add in submission order and are consumed after a successful step.
They are forces, not impulses: the step duration determines the impulse.
Sync preserves pending loads on surviving dynamic entities; removal or a change
to a non-dynamic target drops them. Full entity identity prevents a replacement
entity from inheriting a deleted body's load. Invalid step duration preserves
the queue. This does not introduce transactional rollback of an entire world
after an arbitrary solver failure.

Non-finite values, unsynced or non-dynamic targets, and generalized-load overflow
are rejected without adding that command. Projection is checked again at step
time because sync may change the configuration.

## Dynamics

Each link's world-space spatial Jacobian projects its wrench using `Jᵀ wrench`.
The coupled step accepts external generalized forces separately from motor
commands, so actuator effort limits do not clip disturbances. Contact and joint
constraints still act on the resulting motion. The existing no-load entry point
remains available without constructing external force vectors.

Conformance testing also exposed world-linear-velocity drift in a spinning free
body. For an isolated floating body with six velocity coordinates and no contact,
the step now removes the moving-frame linear transport term before solving and
rotates linear velocity into the new base frame afterward. This change is scoped
to that case; articulated and contact-coupled integration retain their existing
path. It is not a general conservation guarantee for arbitrary inertia or motion.

## Validation and remaining work

- Release library tests: 51 dynamics and 55 native tests passed, including the
  shared external-wrench conformance case.
- Force addition, point moments, rotated frames, offset center of mass,
  one-step lifetime, sync, removed targets, invalid input, and overflow rejection
  are covered. A joint with zero motor effort still responds to external force.
- A 3 kg cube with friction coefficient 0.5 receives actual API loads at
  `F / (μmg) = -1.2, -0.8, 0.8, 1.2` for 1,000 steps of 2 ms. The sub-threshold
  cases stay within 10 µm of rest; sliding position and velocity agree with the
  analytic solution within 3 mm and 2 mm/s. Each case is run twice and stable
  world/contact snapshot hashes must match.
- All 40 Python contact-benchmark regression tests passed against rebuilt
  release examples. These are regression checks, not new performance measurements.

The earlier [H5 result](HARD_CONTACT_FRICTION_THRESHOLD_RESULTS.md) remains an
equivalent-gravity experiment. This API test does not replace its 32-condition
matrix, establish MuJoCo superiority, or validate physical pushing by another
body. Next, run matched-quality friction/disturbance comparisons using actual
forces and measure cost with the same fixtures in both engines.

Workspace-wide checks still need CI: local full builds previously exhausted
disk space, and this run has less than 400 MB available. Targeted checks cover
the changed crates; they do not substitute for the workspace merge gates.
