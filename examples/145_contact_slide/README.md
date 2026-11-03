# 145 · A pushed block on four spheres: Rapier's contact against Coulomb and MuJoCo

Example 144 traced the push trials on which RNE and MuJoCo disagree to a foot
sliding and landing under a large horizontal load. This example takes the
foot alone. A 10 kg block (the inertia of a 0.24 × 0.12 × 0.08 m box) stands
on four 5 mm spheres, 0.2 × 0.1 m apart and 4 cm below its centre of mass,
like the G1's foot. It is pushed horizontally for 0.1 s, or dropped tilted
while moving, on a floor of friction 1, at the walking loop's 2 ms step.
`mujoco_contact_slide.py` builds the same block in MuJoCo (a free body with
the same mass and principal inertia, four sphere geoms, a plane, MuJoCo's
default contact parameters) from this example's report. It runs every
scenario and prints both engines next to the closed-form answer where
Coulomb friction between rigid bodies gives one.

```bash
cargo run --release -p contact_slide --example 145_contact_slide         # target/contact-slide/contact-slide.json
cargo run --release -p contact_slide --example 145_contact_slide -- --solver-iterations 16
python3 examples/145_contact_slide/mujoco_contact_slide.py target/contact-slide/contact-slide.json
python3 examples/145_contact_slide/mujoco_contact_slide.py target/contact-slide/contact-slide.json \
  --timestep-divisor 10      # MuJoCo at 0.2 ms; also --cone elliptic, --impratio, --noslip-iterations
```

## Scenarios

| Name | What happens | Closed form |
| --- | --- | --- |
| `stick` | Push of half the friction limit (0.5 m g) at the centre of mass | No slide; static friction F at the contact plane, 4.5 cm below the centre of mass, moves F · 0.045 / 0.1 of the load to the front spheres |
| `slide` | Push of twice the friction limit (2 m g) at the centre of mass | Speed (F/m − μg) T = 0.981 m/s after T = 0.1 s, slide 0.0981 m, stop 0.1 s after the push, μ m g · 0.045 / 0.1 of the load moved forward |
| `slide_mu_0.5` | The same on friction 0.5 (push m g) | The same formulas |
| `rock` | Push of 0.9 m g, 0.1 m above the centre of mass: it rocks onto its front spheres and falls back | Largest pitch 0.1274 rad, integrating the rigid block's rotation about its front contact line |
| `landing` | Dropped 2 cm, heel first (pitched 0.2 rad nose up), moving at 1 m/s | — |

Every scenario reports, averaged over every physics step:
- the slide;
- the speed when the push ends;
- when the block stops;
- its pitch range;
- the normal load on the front and back spheres during the push, after its first 20 ms;
- the share of steps without any contact force (`airborne_share`) and the largest rise of the centre of mass above its lowest point (`hop_m`). Both are taken over the 0.3 s after the push starts, or after a block that pitched lies flat again.

## Results

RNE at its default 4 solver iterations and MuJoCo 3.9 at its defaults
(pyramidal cone, `solref` 0.02 1); MuJoCo at a 0.2 ms step in the last
column. RNE's loads are the reported ones times 4/5, removing Rapier's
over-count (below); as reported they are 1.25 times larger:

| Scenario, quantity | Closed form | RNE | MuJoCo | MuJoCo, 0.2 ms |
| --- | --- | --- | --- | --- |
| stick, slide (mm) | 0 | 0.0 | 0.3 | 0.3 |
| stick, front / back load (N) | 60.1 / 38.0 | 59.9 / 38.2 | 60.1 / 38.0 | 60.1 / 38.0 |
| slide, speed after push (m/s) | 0.981 | 0.981 | 0.860 | 0.988 |
| slide, slide (mm) | 98.1 | 98.2 | 98.9 | 98.5 |
| slide, stops after (s) | 0.100 | 0.100 | 0.112 | 0.113 |
| slide, front / back load (N) | 71.1 / 27.0 | 70.8 / 27.3 | 82.3 / 29.2 | 67.7 / 26.4 |
| slide, steps without contact | — | 0% | 31% | 32% |
| slide, hop (mm) | — | 0.0 | 1.4 | 0.2 |
| slide μ 0.5, speed after push (m/s) | 0.491 | 0.491 | 0.499 | 0.490 |
| slide μ 0.5, steps without contact | — | 0% | 14% | 13% |
| rock, largest pitch (rad) | 0.1274 | 0.1251 | 0.1226 | 0.1226 |
| rock, rebound when it falls back (rad) | — | 0.001 | 0.025 | 0.026 |
| landing, nose-down overshoot (rad) | — | 0.009 | 0.055 | 0.062 |
| landing, hop after it lies flat (mm) | — | 0.6 | 3.7 | 4.6 |

- **RNE follows Coulomb's law.** The pushed block's speed and slide match the
  closed form to 0.1%, its stop to one 2 ms step, and the loads on the front
  and back spheres to 1% once Rapier's over-count is divided out. It does not leave the floor. At 1, 4 and 16 solver
  iterations the slide results agree to 0.2%, and at 16 iterations the rock
  reaches 0.1272 rad against 0.1274.
- **MuJoCo's sliding block skips.** While it slides, no sphere touches the
  floor in about a third of the steps (13–14% on friction 0.5). This happens
  with the elliptic cone, `impratio` 10 and 10 no-slip iterations too, and at
  a ten-times-shorter step, where the hops shrink from 1.4 to 0.2 mm but not
  their share. The skipping block stops 12% later than Coulomb predicts. At
  the 2 ms step it also leaves the push 12% slow, which the shorter step
  removes.
- **MuJoCo's contacts rebound; RNE's do not.** The rocked block falls back
  onto its rear spheres and bounces 0.025 rad nose up. The block landing heel
  first overshoots 0.055 rad nose down and hops 3.7 mm. Rapier's contacts,
  with zero restitution, stop both within 0.01 rad and 1 mm. MuJoCo's
  contacts are soft by design (`solref`, `solimp`) and behave as a stiff
  spring and damper rather than an inelastic impact. In the push that
  example 144 traced, the two walks part when the robot's swing foot lands
  while it is being driven forward, the situation these scenarios isolate.

## Contact forces reported by Rapier

This example found that Rapier 0.22 reports warm-started contact impulses,
and so RNE's `ContactEvent::impulse` and `ContactPointSample::normal_force_n`,
as (n + 1) / n of the impulse the solver applied over a step with n solver
iterations. Each contact accumulates its impulse at the start of every
substep, the first included, when the impulse is still the warm start
carried in from the previous step. The block at rest reports 1.25 times its
weight at 4 iterations (122.6 N for 98.1 N), twice it at 1 and 1.0625 times
at 16. The motion is unaffected: only the reported forces are too large.
This applies to contacts between rigid bodies and fixed geometry or each
other, and between a multibody link and a dynamic body. A multibody link
against fixed geometry, like example 144's feet on the ground, starts each
step without a warm start and is reported correctly. A fix to the vendored
Rapier is a separate change: code that feeds reported contact loads back
into the simulation, such as the mobility benchmark's tire models, moves
with it.

## Limits

- One rigid body: example 144's feet hang from a multibody, whose contacts
  with the ground go through Rapier's generic one-body constraints. Those
  were never over-counted; example 144's `standing_feet_carry_the_robots_weight`
  checks the G1's standing foot forces against its weight.
- The landing scenario has no closed form. "No rebound" follows from treating
  the contact as inelastic, which Rapier's zero restitution does and
  MuJoCo's soft contacts do not.
