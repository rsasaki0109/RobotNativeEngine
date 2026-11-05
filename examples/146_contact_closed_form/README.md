# 146 · Contact cases with closed-form answers: Rapier against them and MuJoCo

Example 145 checked a pushed block against Coulomb friction. This example
takes the other textbook contact cases and checks each against its closed-form
answer, at the walking loop's 2 ms step, in RNE (Rapier) and in MuJoCo.
`mujoco_contact_closed_form.py` builds every case in MuJoCo from this
example's report:
- a floor plane;
- free bodies with one box or sphere geom of the same mass, and so the same
  solid-shape inertia;
- the same friction and gravity;
- MuJoCo's default contact parameters and pyramidal cone unless
  `--cone elliptic`.

```bash
cargo run --release -p contact_closed_form --example 146_contact_closed_form   # target/contact-closed-form/contact-closed-form.json
cargo run --release -p contact_closed_form --example 146_contact_closed_form -- --contact-softness 240,5
python3 examples/146_contact_closed_form/mujoco_contact_closed_form.py target/contact-closed-form/contact-closed-form.json \
  --cone elliptic   # also --impratio, --noslip-iterations, --timestep-divisor
```

## Cases

Boxes are 0.2 × 0.2 × 0.1 m; balls are solid, 1 kg, radius 5 cm. An incline
tilts gravity toward +x instead of tilting the floor.

| Name | What happens | Closed form |
| --- | --- | --- |
| `slide_0`, `slide_22.5`, `slide_45` | A 10 kg box launched at 2 m/s on friction 0.5, 0°, 22.5° and 45° from +x | Stops after v² / (2 μ g) = 0.408 m on a straight line, below 0.01 m/s at (v − 0.01) / (μ g) = 0.406 s, whatever the direction |
| `incline_hold_20` | The box released on a 20° incline, friction 0.5 | Holds, since tan 20° = 0.36 < μ |
| `incline_slide_35` | The same at 35° | Slides at g (sin θ − μ cos θ): 0.804 m in 1 s |
| `roll_20` | A ball released on a 20° incline, friction 0.5 | Rolls without slipping at 5/7 g sin θ, since μ ≥ 2/7 tan θ |
| `roll_slip_20` | The same on friction 0.05 | Slides at g (sin θ − μ cos θ) while its spin grows at 5 μ g cos θ / (2 r) |
| `bounce_e0`, `bounce_e0.5` | A ball dropped from 0.2 m, restitution 0 and 0.5 | Rebounds to e² × 0.2 m |
| `stack_1`, `stack_10`, `stack_100` | A 1, 10 or 100 kg box resting on a 1 kg box | Nothing moves; the floor carries both weights and the lower box the upper one's |

Every metric is taken from every physics step:
- the slide along and across the launch direction, and when the box stops;
- the displacement down an incline after the run;
- a rolling ball's speed and spin, and its slip: the contact point's speed,
  with the centre's height as the rolling radius;
- the highest a dropped ball's bottom gets after it stops falling;
- how far the upper box of a stack sinks and drifts, and the mean normal load
  on the floor and between the boxes over the last 0.5 s.

## Results

RNE at its default 4 solver iterations and contact softness (30 Hz, damping
ratio 5), and with 240 Hz contacts; MuJoCo 3.9 with its default pyramidal
cone and with the elliptic one:

| Case, quantity | Closed form | RNE | RNE, 240 Hz | MuJoCo | MuJoCo, elliptic |
| --- | ---: | ---: | ---: | ---: | ---: |
| slide 0°, distance (m) | 0.408 | 0.408 | 0.407 | 0.406 | 0.406 |
| slide 22.5°, distance (m) | 0.408 | 0.408 | 0.407 | **0.486** | 0.408 |
| slide 22.5°, sideways (m) | 0 | 0.000 | 0.000 | **0.110** | 0.001 |
| slide 45°, distance (m) | 0.408 | 0.408 | 0.407 | **0.575** | 0.406 |
| slide 45°, stops after (s) | 0.406 | 0.406 | 0.406 | **0.582** | 0.412 |
| incline 20°, creep in 2 s (mm) | 0 | 0.5 | 0.0 | 3.6 | 1.8 |
| incline 35°, slide in 1 s (m) | 0.804 | 0.806 | 0.805 | 0.803 | 0.802 |
| roll 20°, speed after 1 s (m/s) | 2.397 | 2.381 | 2.391 | 2.391 | 2.387 |
| roll 20°, slip (m/s) | 0 | 0.031 | 0.012 | 0.003 | 0.009 |
| roll on friction 0.05, spin (rad/s) | 23.05 | 22.84 | 22.99 | 23.04 | 23.08 |
| bounce e = 0, rebound (mm) | 0 | −1.1 | −0.7 | −0.1 | −0.4 |
| bounce e = 0.5, rebound (m) | 0.050 | 0.050 | 0.050 | — | — |
| 1 kg on 1 kg, sink (mm) | 0 | 3.6 | 1.9 | 0.1 | 0.4 |
| 10 kg on 1 kg, sink (mm) | 0 | 10.8 | 2.1 | 0.4 | 1.3 |
| 100 kg on 1 kg, sink (mm) | 0 | **82.7** | 3.3 | 1.7 | 9.9 |
| 100 kg on 1 kg, floor / box load (N) | 990.8 / 981.0 | 991.2 / 981.3 | 990.8 / 981.0 | 990.8 / 981.0 | 990.8 / 981.0 |

- **RNE's friction is the same in every direction; MuJoCo's default is not.**
  MuJoCo's pyramidal cone approximates the friction cone by four edges, so a
  box sliding diagonally feels 1/√2 of the friction it should. At 45° it
  slides √2 times as far, 0.575 m against 0.408 m. At 22.5° the friction is
  no longer opposite the motion, so the path curves 0.11 m sideways. A ten
  times shorter step changes neither. The elliptic cone removes both. Rapier
  clamps friction to the cone itself and matches the closed form to 0.1% in
  every direction.
- **RNE holds better on an incline.** Below the friction angle, RNE's box
  creeps 0.5 mm in 2 s, MuJoCo's 3.6 mm (1.8 mm with the elliptic cone and
  0.2 mm with `impratio` 10 as well).
- **Both slide, roll and bounce as the closed forms predict**, to about 1%. A
  resting RNE body sits about 1 mm into the floor, the error Rapier allows
  before it corrects a contact. The ball therefore rolls on a radius 1.3 mm
  short of its own, which shows as its 0.03 m/s of slip. MuJoCo has no
  restitution coefficient; RNE's ball rebounds to e² of its drop.
- **A heavy box on a light one sinks in RNE.** Rapier's contact behaves as a
  spring whose stiffness scales with the effective mass at the contact. A
  light box supporting a heavy one therefore sinks in proportion to the mass
  ratio: 3.6, 10.8 and 82.7 mm at ratios 1, 10 and 100. Both boxes still
  carry the right loads. MuJoCo sinks 0.1 to 1.7 mm, and 9.9 mm with the
  elliptic cone. Stiffer contacts (`--contact-softness 240,5`, 240 Hz instead
  of 30) bring RNE to 1.9 to 3.3 mm. They move every other case by at most
  0.1%, mostly toward its closed form, and stop the creep on the incline.
  RNE's default stays at 30 Hz: example 144's walking and its MuJoCo
  comparison were measured at it.

## Limits

- Single bodies and a two-box stack on a flat floor. Example 145 covers a
  block standing on small spheres, and example 144 a walking robot whose feet
  hang from a multibody.
- MuJoCo runs at its defaults. Other `solref` and `solimp` settings move its
  penetration and creep; restitution has no direct MuJoCo equivalent, so the
  bouncing case with e = 0.5 runs in RNE alone.
