# 147 · A Go2 learns to walk from scratch in RNE and in MuJoCo

Example 144's push-recovery residuals corrected a pretrained walking policy.
This example learns a walk from nothing. Both engines train with the same
algorithm, policy, episode, fitness and seed, so only the simulator differs.
It then checks two things: how fast each engine gets a Go2 walking, and
whether a walk learned in one works in the other.

```bash
cargo run --release -p go2_learning --example 147_go2_learning -- --train --seed 2026   # target/go2-learning/go2-learning-rne-2026.json
cargo run --release -p go2_learning --example 147_go2_learning -- --train --seed 2026 \
  --solver-iterations 1 --inner-passes 4   # the cheaper solve; same output path
cargo run --release -p go2_learning --example 147_go2_learning -- --train --seed 2026 \
  --backend locomotion   # rne_locomotion's contact step; go2-learning-locomotion-2026.json
cargo run --release -p go2_learning --example 147_go2_learning -- --write-terrain terrain.json --terrain 7
cargo run --release -p go2_learning --example 147_go2_learning -- --train --seed 2026 \
  --backend locomotion --terrain 7   # go2-learning-locomotion-terrain-2026.json
python3 examples/147_go2_learning/mujoco_go2_learning.py check-terrain terrain.json
python3 examples/147_go2_learning/mujoco_go2_learning.py train \
  target/go2-learning/go2-learning-locomotion-terrain-2026.json OUT.json --terrain terrain.json
python3 examples/147_go2_learning/mujoco_go2_learning.py train \
  target/go2-learning/go2-learning-rne-2026.json target/go2-learning/go2-learning-mujoco-2026.json
cargo run --release -p go2_learning --example 147_go2_learning -- --evaluate target/go2-learning/go2-learning-mujoco-2026.json
python3 examples/147_go2_learning/mujoco_go2_learning.py evaluate target/go2-learning/go2-learning-rne-2026.json
cargo run --release -p go2_learning --example 147_go2_learning -- --stand 5   # settled stance, as JSON
python3 examples/147_go2_learning/mujoco_go2_learning.py stand 5
cargo run --release -p go2_learning --example 147_go2_learning -- --bench 8 [--solver-iterations 1] [--inner-passes 4] [--backend native|locomotion]
python3 examples/147_go2_learning/mujoco_go2_learning.py bench 8 [--cone pyramidal --impratio 1]
python3 examples/147_go2_learning/learning_compare.py SUMMARY.json FIGURE.svg \
  --training rne:2026=RNE_REPORT.json --training mujoco:2026=MUJOCO_REPORT.json \
  --evaluation mujoco/rne:2026=EVALUATION.json   # one --evaluation per engine and run
# Engines are rne (4 iterations), rne2x1 (2 iterations), rne1x4 (1 iteration, 4 inner passes), native, locomotion and mujoco.
```

## What runs

- **Robot.** The vendored Go2 URDF
  (`assets/robots/go2_description/go2_description.rne.urdf`). Its collision
  geometry is boxes, cylinders (capsules in RNE) and 22 mm foot spheres.
  - Both engines fuse the links fixed to another link into it: feet,
    lower-calf bumpers and head.
  - Links without an `<inertial>` are massless.
  - The robot totals 16.087 kg, the URDF's declared masses.
- **RNE.** `unitree_go2_fused.rne.scene.toml` runs on Rapier through
  `UrdfSceneSim` and `PdJointWalker`. Its robot asset sets
  `fuse_fixed_children` and `use_declared_inertial_masses`. Three settings:
  - four solver iterations;
  - two solver iterations (`--solver-iterations 2`, "RNE 2×1" below);
  - one solver iteration with four inner constraint passes
    (`--solver-iterations 1 --inner-passes 4`, "RNE 1×4" below; see
    [Speed](#speed)).
- **RNE locomotion step** (`--backend locomotion`, "RNE locomotion" below).
  `rne_locomotion::QuadrupedTerrainSim` steps the same fused 16.087 kg model
  with `rne_dynamics`' hard-contact step on a flat height field, without
  Rapier. Its contacts are the 22 mm foot spheres and every other collision
  shape in the URDF (`QuadrupedTerrainSim::set_body_spheres`), 106 in all:
  - the head's lower sphere as itself;
  - each cylinder (hips, the head's upper half, calves, lower-calf bumpers)
    as spheres of its radius along its axis, one radius apart;
  - each box (body, thighs) as its eight corners, where a box first meets a
    floor.

  The joints get the same explicit PD torque as the other engines. Its contact solver stops at 1e-5 N·s and 0.1 mm/s or 30 sweeps
  (see [Learning on RNE's locomotion step](#learning-on-rnes-locomotion-step)).
- **MuJoCo.** `mujoco_go2_learning.py` builds the robot from the same URDF
  with MuJoCo's importer (visuals and materials removed, `inertiafromgeom`
  off) and adds:
  - a free base;
  - a floor;
  - collisions between the robot and the floor only, as in RNE's scene;
  - no joint damping, friction or armature, as in RNE;
  - the elliptic cone and `impratio` 100, which MuJoCo Menagerie's Go2 sets.
- **Control.** unitree_rl_gym's Go2 settings:
  - PD gains of 20 N·m/rad and 0.5 N·m·s/rad;
  - its default joint angles;
  - a 5 ms physics step, four of them per 50 Hz control period.
  
  Both engines recompute `kp (target − q) − kd q̇` at every physics step and
  clamp it to the URDF effort limit. MuJoCo does this with position actuators.
- **Episode.** The robot starts at rest in its default pose, its base 0.36 m
  up (`RapierBackend::set_multibody_joint_positions` in RNE, `qpos` in
  MuJoCo). It stands for 1 s, then walks for 5 s. It has fallen if its base
  drops below 0.10 m or tilts more than 60°.
- **Policy.** A 32-unit tanh MLP maps 44 observations to 12 actions in
  [−1, 1]. The joint targets are the default angles plus 0.5 rad times the
  action. The observations are:
  - the base's angular velocity and the gravity direction in the base frame;
  - joint positions relative to the default pose, and joint velocities;
  - the previous action;
  - the sine and cosine of a 0.5 s gait clock.
  
  Its output layer starts at zero, so training starts from a robot that stands
  still.
- **Fitness.** The base's forward travel in metres, minus:
  - 0.5 per metre sideways;
  - 0.5 per radian of heading;
  - 0.02 times the mean squared change of the action per step;
  - 1 for a fall.
- **Training.** Evolution strategies with 16 antithetic pairs, so each
  generation runs 33 episodes. σ is 0.05; updates use centred ranks and Adam
  with learning rate 0.02. Training runs 200 generations. MuJoCo's trainer
  reads the settings and seed from RNE's report and draws the same SplitMix64
  deviates. RNE ran on 4 threads, MuJoCo on 4 processes, one engine after the
  other on the 4-core container.

## The first RNE plant stood on its calves

Earlier versions of this example used `unitree_go2_dynamic.rne.scene.toml` as
it then was (kept since as `unitree_go2_legacy.rne.scene.toml`; the dynamic
scene now fuses as below). Its robot asset left the URDF's fixed links as
separate free bodies and gave the eight lower-calf links, which declare no
inertial, 1 kg each.

- **Symptom.** The RNE Go2 stood on its calves among its own feet, bumpers
  and head, which lay loose on the floor. Its base settled 7 cm lower than
  MuJoCo's (0.187 m against 0.256 m).
- **How it was found.** The settled RNE pose, placed in MuJoCo's model, put
  the feet 11 cm below the floor: a geometric mismatch, not a contact one.
- **The fix.** `rne_urdf_import::fuse_urdf_fixed_children` and the fused
  scene above. MuJoCo also stopped deriving mass from geoms, which had added
  0.19 kg.
- **The stances now agree.** Settled on the default targets for 5 s, the
  two engines stand within 7 mm and 0.007 rad of each other:

  | Engine | Base height | Base x | Pitch | Front thigh / calf | Rear thigh / calf |
  | --- | ---: | ---: | ---: | --- | --- |
  | RNE | 0.254 m | −8.5 cm | 0.011 rad | 0.705 / −1.826 | 0.969 / −1.886 |
  | MuJoCo | 0.261 m | −7.9 cm | 0.011 rad | 0.711 / −1.827 | 0.973 / −1.879 |

  The robot lands from its 0.36 m start (feet 4–5 cm up) and slides back
  about 8 cm before it settles. MuJoCo's stance depends on its contact
  settings: its pyramidal cone settles at 0.220 m.

The earlier versions' learning results, speed figures and transfer failures
were all measured on the broken plant. Everything below is remeasured.

## Results

Three seeds per engine and setting:

| Seed | Trained in | Fitness 0.5 reached (generation / s) | Fitness 1 reached (generation / s) | Fitness after 200 | Forward in 5 s | In the other engine |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| 2026 | RNE | 63 / 163 | 106 / 239 | 5.12 | 5.46 m | falls after 3.8 s, 2.4 m |
| 2026 | RNE 2×1 | 73 / 62 | 150 / 126 | 1.32 | 1.56 m | walks, 1.70 m |
| 2026 | RNE 1×4 | 22 / 17 | — | 0.79 | 0.87 m | walks, 0.60 m |
| 2026 | MuJoCo | 74 / 38 | 99 / 54 | 2.29 | 2.90 m | walks, 2.57 m |
| 11 | RNE | 42 / 69 | 88 / 142 | 3.62 | 4.28 m | walks, 3.87 m |
| 11 | RNE 2×1 | 42 / 35 | 90 / 75 | 2.98 | 3.20 m | walks, 2.81 m |
| 11 | RNE 1×4 | 38 / 28 | — | 0.88 | 0.96 m | walks, 0.66 m |
| 11 | MuJoCo | 67 / 34 | 94 / 50 | 2.14 | 2.49 m | walks, 1.69 m |
| 12 | RNE | 45 / 70 | 96 / 148 | 2.80 | 3.29 m | walks, 2.80 m |
| 12 | RNE 2×1 | 81 / 68 | 156 / 130 | 1.36 | 1.50 m | falls after 3.7 s, 1.4 m |
| 12 | RNE 1×4 | 39 / 28 | — | 0.72 | 0.80 m | walks, 0.62 m |
| 12 | MuJoCo | 42 / 16 | 106 / 49 | 2.05 | 2.56 m | walks, 2.84 m |

"In the other engine" is MuJoCo for RNE's policies and RNE at four
iterations for MuJoCo's. MuJoCo's three walks also hold for 5 s at 2×1
(2.72, 1.51 and 2.76 m) and at 1×4 (2.57, 1.74 and 2.81 m).

![Fitness of the mean policy against wall-clock minutes and against generations, one line per run, for runs trained in RNE and in MuJoCo](../../docs/media/go2-learning.svg)

- **Per generation, RNE at four iterations learns as fast as MuJoCo, and
  further.**
  - It reached fitness 1 in 88–106 generations, MuJoCo in 94–106.
  - After 200 generations its walks scored 2.80–5.12, MuJoCo's 2.05–2.29.
  - Three seeds do not rank the engines.
- **RNE 2×1 learns it in half the wall time.** It reached fitness 1 on all
  three seeds, in 90–156 generations, and ended at 1.32–2.98.
- **RNE 1×4 does not learn this walk.** All three seeds plateau at
  0.72–0.88, a slow shuffle.
- **Per minute, MuJoCo learns about three to four times faster than RNE at
  four iterations, and 1.4–2.6 times faster than RNE 2×1.**
  - A generation took 0.58–0.60 s in MuJoCo, 1.49–1.57 s in RNE and 0.81–0.82
    s in RNE 2×1.
  - That is 330–339 simulated seconds per wall second, against 102–130 and
    237–240.
  - Reaching fitness 1 took MuJoCo 49–54 s, RNE 142–239 s and RNE 2×1 75–130 s.
- **Walks transfer.**
  - All three MuJoCo walks hold in RNE, at either setting. At four iterations
    they score 60–112% of their MuJoCo fitness.
  - Two of RNE's three walks hold in MuJoCo, at 79% and 96% of their RNE
    fitness. The third falls after 3.8 s and 2.4 m.
  - Two of RNE 2×1's three walks hold in MuJoCo; the third falls after
    3.7 s. RNE 1×4's short walks all hold.

The [summary](../../docs/media/go2-learning.json) holds each run's timing,
thresholds and final outcome, and every evaluation in both engines.

## Learning on RNE's locomotion step

The same training on `--backend locomotion`, three seeds, 4 threads, in the
development container (MuJoCo's figures here come from the runs above; see
[Side by side with MuJoCo](#side-by-side-with-mujoco) for both engines on one
machine):

| Seed | Fitness 0.5 reached (generation / s) | Fitness 1 reached (generation / s) | Fitness after 200 | Forward in 5 s | In RNE (4 iterations) | In RNE 2×1 |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| 2026 | 158 / 45 | 168 / 49 | 2.79 | 3.30 m | walks, 2.33 m | walks, 2.18 m |
| 11 | 115 / 43 | 166 / 68 | 2.44 | 3.06 m | walks, 1.97 m | walks, 1.92 m |
| 12 | 93 / 25 | 98 / 26 | 5.85 | 6.87 m | walks, 3.57 m | falls after 4.3 s, 2.8 m |

![Fitness of the mean policy against wall-clock minutes and against generations for the three runs on RNE's locomotion step](../../docs/media/go2-learning-locomotion.svg)

- **It learns the walk in MuJoCo's wall time, and further.**
  - It reached fitness 1 in 26–68 s, against MuJoCo's 49–54 s and RNE 2×1's
    75–130 s, but in more generations: 98–168, against MuJoCo's 94–106.
  - After 200 generations its walks scored 2.44–5.85, MuJoCo's 2.05–2.29.
- **It runs 1.3–2 times as fast as MuJoCo on four cores.**
  - A generation took a median 0.27–0.45 s, against MuJoCo's 0.58–0.60 s.
  - That is 444–670 simulated seconds per wall second, against 330–339.
  - The cost per step follows the gait's contacts, so the three runs differ.
- **Its walks hold on Rapier.** All three walk for 5 s at four iterations, at
  46–60% of their fitness; at two iterations seed 12's falls after 4.3 s.
  The other way fails: RNE 2×1's seed-11 walk falls after 0.56 s on the
  locomotion step. MuJoCo was not installed for these runs, so neither
  direction has been checked against it.
- **Two settings were needed to learn this walk.**
  - With `rne_locomotion`'s implicit joint PD in place of the explicit PD
    torque, and only foot contacts, seed 2026 found a 0.4 m shuffle in 200
    generations.
  - With the library's solver tolerances (1e-10 N·s and 1e-8 m/s, up to 200
    sweeps), seed 11's kneeling gait averaged 72 sweeps a step and trained at
    137 simulated seconds per wall second, and that run did not reach fitness 1.
    Its foot, knee and shin contacts on one calf are nearly redundant, so
    Gauss–Seidel converges slowly; the looser stopping rule changes the
    trained walks' fitness by less than 4%.

The [summary](../../docs/media/go2-learning-locomotion.json) holds each run's
timing, thresholds and final outcome, and its evaluations on Rapier.

## Side by side with MuJoCo

`.github/workflows/go2-learning.yml` trains each seed in RNE's locomotion
step and in MuJoCo 3.9.0 on one 4-core GitHub runner, one engine after the
other, then evaluates every final policy in both (and on Rapier for the flat
floor). Its flat-floor results reproduced bit for bit on a second runner, with
timings within 2%. The same seeds learn differently here than in the
development container: the engines' arithmetic is deterministic, but the
system math library's last bits differ, and 200 generations of ES amplify
them.

Flat floor:

| Seed | RNE: fitness 1 (generation / s) | MuJoCo: fitness 1 | RNE after 200 | MuJoCo after 200 | RNE's walk in MuJoCo | MuJoCo's walk on RNE's step (on Rapier) |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| 2026 | 171 / 64 | 99 / 68 | 2.65 | 2.29 | walks, 2.76 m | falls after 0.96 s (walks, 2.57 m) |
| 11 | — (0.54, a shuffle) | 94 / 66 | 0.54 | 2.14 | shuffles, 0.55 m | falls after 0.88 s (walks, 1.69 m) |
| 12 | 97 / 32 | 106 / 70 | 7.00 | 2.05 | falls after 0.92 s | falls after 0.76 s (walks, 2.84 m) |

![Fitness against wall-clock minutes and generations, RNE's locomotion step and MuJoCo on one runner, flat floor](../../docs/media/go2-learning-compare-flat.svg)

Rough terrain (`--terrain 7`: 5 cm of fractal relief, the same height field
in both engines; `check-terrain` ray-casts MuJoCo's copy at 40 of RNE's
heights before training):

| Seed | RNE: fitness 1 (generation / s) | MuJoCo: fitness 1 | RNE after 200 | MuJoCo after 200 | RNE's walk in MuJoCo | MuJoCo's walk on RNE's step |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| 2026 | 165 / 43 | 85 / 48 | 3.12 | 4.11 | walks, 3.37 m | falls after 0.84 s |
| 11 | 64 / 14 | 187 / 116 | 7.59 | 1.05 | falls after 1.7 s | falls after 0.94 s |
| 12 | 108 / 23 | — (0.53) | 5.22 | 0.53 | walks, 5.21 m | falls after 1.06 s |

![Fitness against wall-clock minutes and generations, RNE's locomotion step and MuJoCo on one runner, rough terrain](../../docs/media/go2-learning-compare-terrain.svg)

- **RNE trains 1.2–2.7 times as fast as MuJoCo on the same runner.**
  - Flat: 309–528 simulated seconds per wall second on four cores, against
    MuJoCo's 252–265; a generation took 0.34–0.63 s, against 0.74–0.81 s.
  - Terrain: 693–818, against 307–385.
  - One thread, untrained policy: 232–236 against MuJoCo's 180–188.
- **Which engine learns the walk first depends on the seed.** Flat, RNE got
  there first on two seeds of three and never on the third; on terrain RNE
  reached fitness 1 on all three and MuJoCo on two. Three seeds do not rank
  the engines' sample efficiency.
- **Transfer runs one way.** RNE's walks hold in MuJoCo on two seeds of three
  on each floor. MuJoCo's six walks all fall within about a second on RNE's
  locomotion step, though its three flat walks hold on Rapier. The locomotion
  step is further from MuJoCo than Rapier is: hard contacts and contact only
  at the feet and lower legs, where both MuJoCo and Rapier collide every
  link's geometry.

The summaries ([flat](../../docs/media/go2-learning-compare-flat.json),
[terrain](../../docs/media/go2-learning-compare-terrain.json)) hold each
run's timing, thresholds and final outcome, and every evaluation.

## Speed

Simulated seconds per wall second on one thread, for the untrained policy's
episodes (medians of three runs):

| Engine and setting | Simulated s / wall s |
| --- | ---: |
| RNE, Rapier, 4 solver iterations | 36 |
| RNE, Rapier, 2 solver iterations | 58 |
| RNE, Rapier, 1 solver iteration | 76 |
| RNE, Rapier, 1 solver iteration, 4 inner passes (`--inner-passes 4`) | 74 |
| RNE, native backend (`--backend native`) | 13 |
| RNE, locomotion step (`--backend locomotion`) | 247 (207–249) |
| MuJoCo, elliptic cone, `impratio` 100 | 231 (185–243) |
| MuJoCo, its defaults | 236 |

- **The gap.**
  - On one thread MuJoCo is 6.4 times as fast as RNE at four iterations and
    4 times as fast at two.
  - On four cores the gap narrows to 2.5–3.3 times: four RNE threads ran
    2.8–3.6 times as fast as one, four MuJoCo processes 1.4 times.
- **Where RNE's time goes.** For a trained walk at four iterations, Rapier's
  step is over 90%.
  - Rapier runs one substep per solver iteration, and in each it rebuilds and
    factors the multibody's two mass matrices: 57%, with 7% in the LU
    factorizations.
  - Forward kinematics takes 11%, contact handling about 10% and ECS
    synchronization 6%.
- **Inner passes.**
  - `RapierBackend::set_inner_solver_passes` re-solves the constraints within
    an iteration without rebuilding the mass matrix.
  - One iteration with four passes runs at one iteration's speed, twice as
    fast as four iterations. It still does not learn this walk.
  - At one iteration without inner passes, an earlier run on the broken plant
    reached only 0.35.
- **Fusing, not welding.** `weld_fixed_children` makes each fixed link a
  zero-DoF multibody link with its own share of every mass-matrix rebuild. A
  welded Go2 trained 1.6–1.9 times slower than the fused one.
- **What did not help.** Factoring the mass matrices along the kinematic tree
  without fill cuts the flops to a third at 18 DoF. Each solve still took
  about 2,950 instructions, against 2,160 for nalgebra's vectorized dense LU.
- **The locomotion step matches MuJoCo on one thread** (247 against 231
  simulated seconds per wall second), so on four threads it outruns
  MuJoCo's four processes.
  - It is `rne_dynamics::contact_step`. On a trained walk the contact solve
    takes about 27% of it and the Cholesky solves of the contact columns 18%.
  - It runs one forward-kinematics pass a step:
    `contact_step_with_kinematics` reuses the pass that places the contacts.
    With the tree factor's solves walking flattened ancestor chains, that
    cut a walking episode's instructions by 12%, with bit-identical results.
- **The native backend is slower still.**
  - It spends 85% of its time in `rne_dynamics`' coupled contact solve.
  - It has no compound colliders, so its Go2 still welds the fixed links and
    gives the eight lower-calf links 1 kg each. Its stance differs (0.19 m),
    and its speed is indicative only.

## Limits

- One robot, one task on a flat floor and on one rough terrain, three seeds,
  no domain randomization.
  The policies make no claim about transfer to hardware.
- MuJoCo's episode loop and policy run in Python, one call per 20 ms control
  period; RNE's run in Rust.
- Both engines ran on the same container, one after the other. The
  container's speed drifts between runs, so timings within a row compare
  better than timings across rows. Low-priority builds ran alongside parts of
  the RNE four-iteration and two-iteration runs.
- The locomotion-step runs came later, in a fresh container of the same
  4-core kind, and are compared with the MuJoCo runs above, not rerun beside
  them. Their plant is a simplification: hard contacts, spheres for the feet
  and lower legs only, and a looser contact solver.
