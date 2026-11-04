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
python3 examples/147_go2_learning/mujoco_go2_learning.py train \
  target/go2-learning/go2-learning-rne-2026.json target/go2-learning/go2-learning-mujoco-2026.json
cargo run --release -p go2_learning --example 147_go2_learning -- --evaluate target/go2-learning/go2-learning-mujoco-2026.json
python3 examples/147_go2_learning/mujoco_go2_learning.py evaluate target/go2-learning/go2-learning-rne-2026.json
cargo run --release -p go2_learning --example 147_go2_learning -- --bench 8 [--solver-iterations 1] [--inner-passes 4] [--backend native]
python3 examples/147_go2_learning/mujoco_go2_learning.py bench 8 [--cone pyramidal --impratio 1]
python3 examples/147_go2_learning/learning_compare.py SUMMARY.json FIGURE.svg \
  --training rne:2026=RNE_REPORT.json --training mujoco:2026=MUJOCO_REPORT.json \
  --evaluation mujoco/rne:2026=EVALUATION.json   # one --evaluation per engine and run
# Engines are rne (4 iterations), rne1x4 (1 iteration, 4 inner passes), native and mujoco.
```

## What runs

- **Robot.** The vendored Go2 URDF
  (`assets/robots/go2_description/go2_description.rne.urdf`). Its collision
  geometry is boxes, cylinders and 22 mm foot spheres. It totals 16.3 kg.
- **RNE.** The URDF runs on Rapier through `UrdfSceneSim` and
  `PdJointWalker`, in two settings:
  - four solver iterations;
  - one solver iteration with four inner constraint passes
    (`--solver-iterations 1 --inner-passes 4`, "RNE 1×4" below; see
    [Speed](#speed)).
- **MuJoCo.** `mujoco_go2_learning.py` builds the robot from the same URDF
  with MuJoCo's importer (visuals and materials removed) and adds:
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

## Results

Three seeds per engine and setting:

| Seed | Trained in | Fitness 0.5 reached (generation / s) | Fitness 1 reached (generation / s) | Fitness after 200 | Forward in 5 s | In the other engine |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| 2026 | RNE | 43 / 84 | 81 / 160 | 3.77 | 3.97 m | falls after 0.4 s |
| 2026 | RNE 1×4 | 82 / 71 | 97 / 84 | 2.76 | 3.39 m | falls after 0.8 s |
| 2026 | MuJoCo | 51 / 20 | 112 / 47 | 3.30 | 3.82 m | falls after 0.6 s |
| 11 | RNE | 172 / 331 | — | 0.48 | 0.52 m | falls after 1.7 s |
| 11 | RNE 1×4 | 99 / 85 | 104 / 89 | 2.31 | 2.88 m | falls after 0.7 s |
| 11 | MuJoCo | 57 / 21 | 153 / 63 | 1.50 | 1.71 m | falls after 0.5 s |
| 12 | RNE | 59 / 112 | 76 / 143 | 3.44 | 3.86 m | falls after 1.4 s |
| 12 | RNE 1×4 | — | — | 0.42 | 0.45 m | falls after 0.9 s |
| 12 | MuJoCo | 43 / 14 | 81 / 32 | 3.57 | 3.96 m | falls after 0.2 s |

MuJoCo's "in the other engine" is RNE at four iterations; at 1×4 its walks
fall after 0.6, 0.5 and 0.2 s.

![Fitness of the mean policy against wall-clock minutes and against generations, one line per run, for runs trained in RNE and in MuJoCo](../../docs/media/go2-learning.svg)

- **Per generation, the engines learn alike.**
  - On seeds 2026 and 12, RNE reached fitness 1 in 81 and 76 generations,
    MuJoCo in 112 and 81.
  - On seed 11, RNE stalled near 0.5 while MuJoCo reached 1.5.
  - RNE 1×4 reached fitness 1 in 97 and 104 generations on seeds 2026 and 11,
    and stalled near 0.4 on seed 12.
  - Three seeds do not rank the engines or the settings.
- **Per minute, MuJoCo learns about four times faster than RNE at four
  iterations, and about 1.8 times as fast as RNE 1×4.**
  - A generation took 0.41–0.48 s in MuJoCo, 1.83–1.93 s in RNE and
    0.80–0.85 s in RNE 1×4.
  - That is 421–449 simulated seconds per wall second, against 102–105 and
    233–237.
  - Reaching fitness 1 took MuJoCo 32–63 s, RNE 143–160 s and RNE 1×4
    84–89 s.
- **No walk transfers.** Every final policy falls within 0.2–1.7 s in the
  other engine.
  - Learned from nothing, each walk fits its own engine's contact.
  - The plants already part while the robot stands. On its default targets
    the robot sags to a base height of 0.187 m in RNE and 0.256 m in MuJoCo,
    from the same start.
  - Standing still, MuJoCo's robot creeps 49 mm forward in 5 s, RNE's 2 mm.
  - Example 144's residuals, small corrections to a robust pretrained walk,
    transferred both ways.

The [summary](../../docs/media/go2-learning.json) holds each run's timing,
thresholds and final outcome, and every evaluation in both engines.

## Speed

Simulated seconds per wall second on one thread, for the untrained policy's
episodes:

| Engine and setting | Simulated s / wall s |
| --- | ---: |
| RNE, Rapier, 4 solver iterations | 34 |
| RNE, Rapier, 2 solver iterations | 50 |
| RNE, Rapier, 1 solver iteration | 67 |
| RNE, Rapier, 1 solver iteration, 4 inner passes (`--inner-passes 4`) | 67 |
| RNE, native backend (`--backend native`) | 14 |
| MuJoCo, elliptic cone, `impratio` 100 | 225 |
| MuJoCo, its defaults | 261 |

These are medians of three runs; MuJoCo's elliptic-cone speed ranged from 195
to 258 across them. On one thread MuJoCo is about 3.4 times as fast as RNE
1×4. Training on four cores narrows that to about 1.8: four RNE threads ran
3.5 times as fast as one, four MuJoCo processes about 2 times.

- **Where RNE's time goes.** At four iterations, 92% of the time is Rapier's
  step. Rapier runs one substep per solver iteration, and in each it rebuilds
  the multibody's mass matrix and factors it. That work (`update_inertias`,
  `rne_quadform_columns`, `rne_gemm3_columns` and the LU factorization and
  solves) is about 47% of the total.
- **Inner passes buy the iterations back cheaply.** At one iteration, seed
  2026's best fitness in 200 generations was 0.35, against 4.0 at four.
  `RapierBackend::set_inner_solver_passes` re-solves the constraints within
  an iteration without rebuilding the mass matrix. Four passes cost little
  next to one iteration (67 against 67 here, 59 against 62 before this PR's
  other speedups), and RNE 1×4 learns (above).
- **Where RNE 1×4's time goes.** For a trained walk, Rapier's step is 83%:
  - rebuilding and factoring the two mass matrices, once per step: 24%;
  - solving the constraints: 14%, after building them with dense LU solves;
  - broad phase 10%, narrow phase 9%;
  - forward kinematics, run twice per step: 7%.

  Synchronizing the ECS takes 12%.
- **What did not help.** Factoring the mass matrices along the kinematic tree
  without fill has a third of dense LU's flops at 18 DoF, but each solve took
  about 2,950 instructions against 2,160 for nalgebra's vectorized dense LU.
- **The native backend is slower still.** It spends 85% of its time in
  `rne_dynamics`' coupled contact solve, about 365 µs per step for this robot.

## Limits

- One robot on a flat floor, one task, three seeds, no domain randomization.
  The policies make no claim about transfer to hardware.
- MuJoCo's episode loop and policy run in Python, one call per 20 ms control
  period; RNE's run in Rust.
- Both engines ran on the same container, one after the other. The
  container's speed drifts between runs, so timings within a row compare
  better than timings across rows.
- The RNE 1×4 runs and the speed table used this PR's final build. It steps a
  walking Go2 in 9% fewer instructions than the build of the four-iteration
  runs, with bit-identical walks (see the CHANGELOG).
