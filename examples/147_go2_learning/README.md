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
development container, with builds running alongside parts of it; see
[Side by side with MuJoCo](#side-by-side-with-mujoco) for both engines on one
machine:

| Seed | Fitness 0.5 reached (generation / s) | Fitness 1 reached (generation / s) | Fitness after 200 | Forward in 5 s | In RNE (4 iterations) | In RNE 2×1 |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| 2026 | 72 / 17 | 119 / 29 | 5.26 | 5.52 m | walks, 5.29 m | walks, 5.15 m |
| 11 | 46 / 10 | 87 / 22 | 2.74 | 3.42 m | walks, 3.34 m | walks, 3.42 m |
| 12 | 45 / 11 | 74 / 18 | 2.34 | 2.63 m | walks, 2.53 m | walks, 2.23 m |

![Fitness of the mean policy against wall-clock minutes and against generations for the three runs on RNE's locomotion step](../../docs/media/go2-learning-locomotion.svg)

- **Its walks hold on Rapier.** All three walk for 5 s at four and at two
  iterations, at 65–113% of their fitness.
- **Three settings were needed to learn walks that hold in other engines.**
  - **Every collision shape touches the floor.** The first version of this
    plant gave contacts to the feet, knees, calves and lower-calf bumpers
    only, so its thighs, hips, head and body passed through the floor.
    - MuJoCo's walks lean on the head's lower sphere: on Rapier it comes
      within 5 mm of the floor in 78–174 of an episode's 250 control steps.
      On the first plant the head sank instead, and all of MuJoCo's walks
      fell within about a second.
    - Its own walks found gaits that hold only without those contacts. On
      the CI runner, seed 12's ran 8.07 m in 5 s and fell after 0.92 s in
      MuJoCo, and seed 11's never got past a shuffle.
    - Its walks held on Rapier at 46–60% of their fitness.
    - On the current plant, seeds 11 and 12 lean on the head as MuJoCo's
      walks do (within 5 mm in 93 and 112 control steps), and seed 2026
      barely does (6).
  - **Explicit PD.** With `rne_locomotion`'s implicit joint PD in place of
    the explicit PD torque, and only foot contacts, seed 2026 found a 0.4 m
    shuffle in 200 generations.
  - **A looser solver stopping rule.** With the library's solver tolerances
    (1e-10 N·s and 1e-8 m/s, up to 200 sweeps), the first plant's seed-11
    kneeling gait averaged 72 sweeps a step and trained at 137 simulated
    seconds per wall second, and that run did not reach fitness 1. Its foot,
    knee and shin contacts on one calf are nearly redundant, so Gauss–Seidel
    converges slowly. The looser stopping rule changes the trained walks'
    fitness by less than 4%.
- **The extra contacts are free while clear of the floor.**
  `QuadrupedTerrainSim` skips the terrain query for a body sphere that cannot
  come within the contact margin this step. The untrained policy's episodes
  take 7% fewer instructions with the 106 body contacts than the first
  plant's took with 40.

The [summary](../../docs/media/go2-learning-locomotion.json) holds each run's
timing, thresholds and final outcome, and its evaluations on Rapier.

## Side by side with MuJoCo

`.github/workflows/go2-learning.yml` trains each seed in RNE's locomotion
step and in MuJoCo 3.9.0 on one 4-core GitHub runner, one engine after the
other. It then evaluates every final policy in both engines, and on Rapier
for the flat floor.

The same flat runs in the development container crossed fitness 0.5 and 1 in
the same generations and ended within 0.02 of these. On the first plant they
had diverged: the system math library's last bits differ between the
machines, and 200 generations of ES amplified them.

Flat floor:

| Seed | RNE: fitness 1 (generation / s) | MuJoCo: fitness 1 | RNE after 200 | MuJoCo after 200 | RNE's walk in MuJoCo | MuJoCo's walk on RNE's step (on Rapier) |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| 2026 | 119 / 35 | 99 / 48 | 5.28 | 2.29 | walks, 5.44 m | walks, 2.23 m (walks, 2.57 m) |
| 11 | 87 / 25 | 94 / 46 | 2.74 | 2.14 | walks, 3.08 m | walks, 2.39 m (walks, 1.69 m) |
| 12 | 74 / 22 | 106 / 49 | 2.34 | 2.05 | walks, 2.67 m | walks, 2.88 m (walks, 2.84 m) |

![Fitness against wall-clock minutes and generations, RNE's locomotion step and MuJoCo on one runner, flat floor](../../docs/media/go2-learning-compare-flat.svg)

Rough terrain (`--terrain 7`: 5 cm of fractal relief, the same height field
in both engines; `check-terrain` ray-casts MuJoCo's copy at 40 of RNE's
heights before training):

| Seed | RNE: fitness 1 (generation / s) | MuJoCo: fitness 1 | RNE after 200 | MuJoCo after 200 | RNE's walk in MuJoCo | MuJoCo's walk on RNE's step |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| 2026 | 146 / 49 | 85 / 71 | 2.11 | 4.11 | walks, 2.32 m | walks, 4.33 m |
| 11 | 130 / 42 | 187 / 165 | 2.67 | 1.05 | walks, 2.26 m | walks, 1.09 m |
| 12 | — (0.83) | — (0.53) | 0.83 | 0.53 | shuffles, 0.84 m | shuffles, 0.53 m |

![Fitness against wall-clock minutes and generations, RNE's locomotion step and MuJoCo on one runner, rough terrain](../../docs/media/go2-learning-compare-terrain.svg)

- **RNE trains 1.7–2.7 times as fast as MuJoCo on the same runner.**
  - Flat: 589–683 simulated seconds per wall second on four cores, against
    MuJoCo's 351–371. A generation took 0.29–0.34 s, against 0.53–0.59 s.
  - Terrain: 566–613, against 213–267.
  - One thread, untrained policy: 341–344 against 259–269 on the flat floor,
    and 327–332 against 175–176 on terrain.
  - The two floors ran on different runners, so compare speeds within a
    floor, not across floors.
- **RNE reaches the walk first in wall time.**
  - Flat: RNE reached fitness 1 first on all three seeds, in 22–35 s against
    MuJoCo's 46–49 s. It also took fewer generations on two seeds.
  - Terrain: RNE was first on seeds 2026 and 11. On seed 2026 it was quicker
    in wall time but needed more generations (146 against 85). On seed 12
    neither engine reached fitness 1.
  - After 200 generations RNE scored higher on five runs of six.
  - Three seeds do not rank the engines' sample efficiency.
- **Transfer runs both ways.** All twelve cross-evaluations walk or shuffle
  through the 5 s.
  - RNE's walks keep 68–100% of their fitness in MuJoCo.
  - MuJoCo's walks keep 56–127% of theirs on RNE's locomotion step.
  - On the first plant only four of the twelve held, and none of MuJoCo's
    six.

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
| RNE, locomotion step (`--backend locomotion`) | 308 (296–401) |
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
- **The locomotion step outruns MuJoCo on one thread.** In this container it
  ran 308 simulated seconds per wall second against MuJoCo's 231, measured in
  different sessions. On one CI runner it ran 341–344 against 259–269 (see
  [Side by side with MuJoCo](#side-by-side-with-mujoco)). On four threads the
  lead widens.
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
- The locomotion-step runs in its own section came from a later session in
  the development container. [Side by side with MuJoCo](#side-by-side-with-mujoco)
  reruns them beside MuJoCo on one CI runner.
- The locomotion step's plant is a simplification:
  - its contacts are hard;
  - a box touches only at its corners;
  - a cylinder is a chain of spheres;
  - its contact solver stops early.
