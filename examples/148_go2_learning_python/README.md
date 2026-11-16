# 148 · Example 147's Go2 walking task from Python

`rne_py_locomotion.Go2LearningBatch` is `rne_locomotion::go2_learning` as a native
parallel batch: the Go2 of example 147 with every URDF collision shape
touching the ground, on a flat floor or on a terrain with fractal relief and
gaps, observing its joints and body (44 values) and optionally a 15-ray
height scan (59). One Python call steps every environment on the available
cores.

The extension is `rne_py_locomotion`, apart from `rne_py`: it is built on
`rne_locomotion`, which is not published, while `rne_py` is a published
crate.

## Contract

- **Action** (12): joint offsets in [-1, 1], times 0.5 rad around the
  default pose, tracked by an explicit PD (20 N·m/rad, 0.5 N·m·s/rad) at
  5 ms physics steps, 4 to a 20 ms control period.
- **Observation** (44, or 59 with the scan): base angular velocity × 0.25,
  gravity in the base frame, joint offsets, joint velocities × 0.05, the
  previous action, a 0.5 s gait clock, then the scan readings.
- **Reward** per control period: forward progress, less 0.5 per meter of
  growth in the sideways offset and per radian in the heading, less 0.02 ×
  the squared action change / 250, less 1 on a fall. An episode's rewards
  sum to example 147's fitness.
- **Episode**: 50 periods of standing, then up to 250 (5 s). A fall ends it
  as terminated, the time limit as truncated. The batch resets a finished
  environment at once; `terminal_observations` holds the last observation.
- **Starts**: the origin, or with `random_starts=True` (a terrain only)
  points drawn from environment `i`'s stream, seed `seed + i`, clear of the
  gaps.
- **Gaps**: trenches across y, `gap_depth` m deep, between `gap_width`
  m wide (25–35 cm by default), with 0.8–1.6 m of floor between them;
  `gap_intervals()` lists them as (start, end) along x.

The TaskSpec, `rne.unitree_go2.learning.v1`, gives the same spaces in
machine-readable form (`task_spec_json()`).

## Run

```bash
.venv/bin/maturin develop -m crates/rne_py_locomotion/Cargo.toml --release
.venv/bin/python examples/148_go2_learning_python/run.py --smoke
.venv/bin/python examples/148_go2_learning_python/run.py --floor gaps --height-scan
.venv/bin/python examples/148_go2_learning_python/train_ppo.py --smoke
.venv/bin/python examples/148_go2_learning_python/train_ppo.py \
  --floor gaps --height-scan --timesteps 10000000 --n-envs 64 \
  --curriculum 0.05-0.10,0.10-0.20,0.18-0.28,0.25-0.35
```

`run.py` needs only `rne_py_locomotion`. `train_ppo.py` uses Stable-Baselines3's PPO
through `go2_vec_env.NativeGo2VecEnv`, a `VecEnv` over the batch.

## Crossing gaps with Stable-Baselines3

Example 147's own PPO never crossed a gap: its walks reach the first gap
and stop. `train_ppo.py` tries the usual remedies on the same task:

- a standard PPO (SB3; 256 periods per rollout, minibatches of 4096,
  5 epochs, ELU networks of 256 × 256, an initial action deviation of
  e^-0.5);
- many environments (64 by default in CI);
- observations and rewards normalized (`VecNormalize`);
- with `--curriculum`, easier gaps first: each entry is a stage's range of
  widths and, after `@`, its depth (the task's 0.3 m unless given), e.g.
  `0.05-0.10,0.25-0.35` widens the gaps and
  `0.25-0.35@0.03,0.25-0.35@0.3` deepens them.

Training runs in stages (`--stages`, or one per curriculum entry). After
each stage the deterministic policy walks one episode in each of
`--eval-envs` held-out environments (seeds 10 000 past training's) on the
task's gaps, and on the stage's own when they differ (`on_stage_gaps`).
`results.json` records the following for each stage:

- the mean return and distance;
- the falls, and the falls at a gap;
- the gaps reached and crossed: a gap counts as crossed once the base is
  0.25 m past its far edge, so the hind feet are across too;
- how the episodes ended: with a front foot in a gap, with only a rear
  foot in one (a foot centre more than 2 cm below the floor), or upright
  with the front feet within 0.6 m of the next gap and none in it
  (`ended_short_of_gap`).

The `Go2 SB3 PPO` workflow (`.github/workflows/go2-sb3.yml`) trains these
jobs and uploads each one's results:

- a flat floor;
- the task's gaps throughout;
- gaps widened from 5–10 cm;
- gaps deepened from 3 cm;
- shallow 5 cm gaps throughout, which ask whether the policy can cross at
  all when a foot that drops in can step out.

Each gap job runs once blind and once with the scan.

### Results

The first run (workflow run 43319917776): 10M timesteps per job, 64
environments, seed 2026, on a GitHub runner's 4 cores. These are the last
stage's evaluations, over 64 held-out episodes on the task's 25–35 cm gaps.

| Job | Return | Distance | Gaps reached | Crossed | Falls | Steps/s |
|---|---|---|---|---|---|---|
| Flat floor | 17.55 | 19.1 m | – | – | 64 at the arena edge | 11 186 |
| Gaps, blind | 1.30 | 1.43 m | 64 | 0 | 3 | 6 061 |
| Gaps, scan | 1.08 | 1.43 m | 64 | 0 | 15 | 8 504 |
| Gaps, blind, widening | 1.30 | 1.46 m | 64 | 0 | 4 | 7 693 |
| Gaps, scan, widening | 1.19 | 1.47 m | 64 | 0 | 6 | 5 228 |

- **Flat floor.** SB3 learns to walk: its policy runs to the arena's edge,
  19 m from the origin, before the 5 s are up. Every flat-floor
  evaluation starts at the origin, so its 64 episodes are one walk. Its
  return beats example 147's PPO (12.25).
- **Gaps.** No policy crossed a gap. Every one reached the first gap ahead
  of it, about 1.5 m from its start, and stopped there, mostly without
  falling. That is the same outcome as example 147's own PPO.
- **Widening.** Widening the gaps did not help. Its narrow-gap stages fell
  at the task's gaps more often, 29–54 of 64.

Since a standard PPO stops where example 147's did, the next question is
whether the task allows a crossing at all: whether a foot that drops into
a 30 cm trench can climb out, and how wide the gap is at the top once the
terrain's 5 cm samples slope its edges.

### Where the gap policies stop

The second run (workflow run 43330784351) has the same settings. It adds
how each episode ended, curricula that deepen the gaps, and shallow 5 cm
gaps throughout. Each stage's policy was evaluated on the task's 25–35 cm,
30 cm deep gaps and on its own stage's gaps. There are 64 episodes per
evaluation; "crossed" totals every episode's gaps.

| Job, last stage | Own gaps | Crossed | Falls | Ended with a front foot in a gap |
|---|---|---|---|---|
| Blind | 30 cm deep | 0 | 3 | 51 |
| Scan | 30 cm deep | 0 | 2 | 64 |
| Blind, widening | 30 cm deep | 0 | 15 | 39 (18 more a rear foot) |
| Scan, widening | 30 cm deep | 0 | 1 | 53 |
| Blind, shallow | 5 cm deep | 632 | 0 | 13 |
| Scan, shallow | 5 cm deep | 602 | 0 | 10 |

The deepening curricula went 3, 8, 15, then 30 cm. Crossings on each
stage's own gaps:

| Depth | Blind | Scan |
|---|---|---|
| 3 cm | 389 | 421 |
| 8 cm | 281 | 270 |
| 15 cm | 40 (8 falls) | 196 (1 fall) |
| 30 cm | 0 | 0 |

- **Trapped, not stopped short.** On 30 cm gaps the policies do not stop
  short: almost every episode ends with a front foot in the gap. A Go2
  foot that drops 30 cm into a trench does not climb back out.
- **Stepping through.** Over shallow gaps the same gait crosses about ten
  gaps per episode, 25–35 cm wide, and walks 16–17 m. It steps into each
  gap and out again rather than over it. Gaps that are only wide, or only
  shallow, are easy; deep ones need a foot placed beyond the gap.
- **Where the scan helps.** At 15 cm the scan policy crossed 196 gaps with
  one fall, against the blind policy's 40 with eight. This is one seed.
- **Deeper gaps.** No policy learned to step over a 30 cm gap in 10M
  timesteps. The task is not impossible, since a gait that never puts a
  foot into a gap crosses it, but nothing in these runs found such a gait.
  None of the stages carried over to the 30 cm gaps either: every
  curriculum's last stage behaves like training on them throughout.

### Depth levels

None of the stage-by-stage curricula carried over to the 30 cm gaps: each
stage changes every environment's gaps at once. Legged-robot
reinforcement learning usually moves each environment on its own instead,
up when it succeeds and down when it fails. `--depth-levels N` does that,
in the Rust batch:

- the gaps deepen in N levels along y (`gap_levels=N`), each a band of the
  start area, from 30 / N cm to the task's 30 cm;
- each environment starts its episodes in its level's band;
- when an episode ends, its environment moves up a level if the robot
  crossed `--promote-after-gaps` gaps (2) and down one if it crossed
  none, before the batch resets it.

`Go2LearningBatch.set_depth_curriculum`, `set_levels`, `levels` and
`level_depths` drive it from Python, as do
`Go2LearningBatch::set_curriculum` and `DepthCurriculum` in Rust. After
each stage the policy walks 64 held-out episodes at every level
(`by_level`), and `training_levels` counts the training environments at
each. The `Go2 SB3 PPO depth levels` workflow
(`.github/workflows/go2-sb3-levels.yml`) trains 50M timesteps on eight
levels, blind and with the scan, on three seeds each.

`--recycle-levels` sends an environment that masters the deepest level to
a level drawn uniformly from all of them (`DepthCurriculum::recycle`), so
the shallower levels stay in training. The workflow now passes it; the
results below are from before it existed.

Every run also writes `policy.json`: the deterministic policy's layers and
its observation normalization as plain numbers, for evaluation in another
simulator.

#### Results: the scan policies cross 30 cm gaps

The first run was workflow run 43351998314: 50M timesteps per job and
64 environments, taking about 1.5–2 hours per job at 6 900–10 100
steps/s. These are the last stage's walks at the deepest level, the
task's 30 cm gaps, and at 15 cm. Each row is 64 held-out episodes.

| Policy, seed | Crossed at 30 cm | Episodes crossing | Falls | Distance | Crossed at 15 cm |
|---|---|---|---|---|---|
| Blind, 2026 | 64 | 45 | 17 | 3.05 m | 113 |
| Blind, 2027 | 4 | 4 | 7 | 1.67 m | 176 |
| Blind, 2028 | 0 | 0 | 14 | 1.49 m | 153 |
| Scan, 2026 | 440 | 64 | 0 | 12.27 m | 319 |
| Scan, 2027 | 411 | 64 | 0 | 11.77 m | 312 |
| Scan, 2028 | 335 | 63 | 4 | 9.92 m | 282 |

- **The scan policies cross.** Every scan policy learned to cross the
  30 cm gaps. That is about six gaps per episode, so they walk 10–12 m in
  5 s and seldom fall.
- **Blind policies cross shallow gaps only.** Blind policies cross 15 cm
  gaps but rarely 30 cm ones, which they cannot see.
- **Scan training levels.** By the last stage 56–64 of the 64 scan
  training environments were at the deepest level.
- **Blind training levels.** The blind ones spread over levels 4–7: they
  move up and fall back.
- **Shallow gaps got worse with the scan.** With nearly every environment
  training on 30 cm gaps, the scan policies cross shallow gaps less well
  than deep ones. At 7.5–15 cm deep, 37–47 of 64 episodes end with a front
  foot in a gap. Such gaps barely move the scan's readings: 5 per meter,
  so 0.4–0.75. Legged-robot curricula usually send an environment that
  masters the last level back to a random one, which keeps the easier
  levels in training. This curriculum does not do that yet.
- **The per-environment curriculum works.** The stage-by-stage curricula
  never carried over to 30 cm gaps; this one does.

#### Results with recycling

The second run was workflow run 43374349191, with the same settings plus
`--recycle-levels`. These are gaps crossed over 64 held-out episodes at
each depth, last stage. The first three rows are the first run's scan
policies, for comparison.

| Policy, seed | 3.75 cm | 7.5 cm | 11.25 cm | 15 cm | 22.5 cm | 30 cm |
|---|---|---|---|---|---|---|
| Scan 2026, no recycling | 406 | 169 | 204 | 319 | 443 | 440 |
| Scan 2027, no recycling | 239 | 78 | 254 | 312 | 420 | 411 |
| Scan 2028, no recycling | 352 | 176 | 200 | 282 | 363 | 335 |
| Scan 2026 | 464 | 501 | 515 | 505 | 468 | 338 |
| Scan 2027 | 519 | 481 | 533 | 536 | 481 | 315 |
| Scan 2028 | 433 | 405 | 414 | 409 | 396 | 243 |
| Blind 2026 | 278 | 169 | 126 | 116 | 119 | 66 |
| Blind 2027 | 310 | 199 | 155 | 163 | 112 | 4 |
| Blind 2028 | 252 | 202 | 177 | 153 | 97 | 0 |

- **Shallow gaps recovered.** With recycling the scan policies cross gaps
  of every depth. At 7.5–15 cm deep they cross two to six times as many
  as before. Episodes ending with a front foot in such a gap fell from
  37–47 of 64 to 0–5 for two seeds and 9–15 for the third.
- **The deepest level costs something.** At 30 cm they cross 243–338 gaps
  against 335–440, and fall in 6–10 of 64 episodes against 0–4. Training
  time is now shared between the levels.
- **Blind policies barely change.** They rarely master the deepest level,
  so they are seldom recycled. Seed 2028 never was, and its results are
  identical to the first run's.

#### The same policies in MuJoCo

`mujoco_go2_learning.py evaluate` (example 147) reads `policy.json`, and
the `Go2 SB3 PPO in MuJoCo` workflow (`.github/workflows/go2-sb3-mujoco.yml`)
walks each policy of the recycling run in MuJoCo, from 64 held-out starts
on the task's terrain with 30 cm gaps. It walks them twice: as MuJoCo is
configured, and with five physics steps to each of RNE's and stiffer
contacts (a 4 ms time constant against the default 20 ms). Workflow run
43391649720:

| Policy, seed | RNE: crossed, falls | MuJoCo default: crossed, falls | MuJoCo stiff: crossed, falls, distance |
|---|---|---|---|
| Scan 2026 | 338, 6 | 2, 5 | 442, 0, 12.27 m |
| Scan 2027 | 315, 6 | 13, 16 | 479, 0, 13.12 m |
| Scan 2028 | 243, 10 | 7, 21 | 385, 1, 11.10 m |
| Blind 2026 | 66, 12 | 1, 8 | 61, 28, 3.25 m |
| Blind 2027 | 4, 16 | 14, 13 | 69, 18, 3.26 m |
| Blind 2028 | 0, 14 | 16, 18 | 70, 15, 3.31 m |

- **With stiff contacts the scan policies transfer.** Trained only in
  RNE, they cross the 30 cm gaps in MuJoCo in 63–64 of 64 episodes, with
  at most one fall.
- **With MuJoCo's default contacts they do not.** They cross 2–13 gaps in
  64 episodes. Example 147 found the same for its own walks: at the
  default 20 ms time constant the feet sink into the floor and catch on
  the height field's cells.
- **The scan matters in MuJoCo too.** With stiff contacts the blind
  policies cross 61–70 gaps and fall in 15–28 episodes; the scan policies
  cross 385–479.
- **The columns are not the same episodes.** RNE's numbers are the
  training script's evaluation, from its own starts in the deepest
  level's band; MuJoCo's are from example 147's held-out starts over the
  whole start area. The counts compare in size, not episode by episode.
  Why the scan policies cross more in stiff MuJoCo than in RNE, and why
  two blind policies cross there at all, is not explained yet.

#### The same starts in both simulators

`walk_policy.py` walks an exported policy in RNE from example 147's
held-out evaluation starts (`Go2LearningBatch.evaluation_starts` and
`set_starts`), the points MuJoCo's script walks from, and
`--against NAME=FILE` compares the two walks episode by episode. The
MuJoCo workflow runs it after its own walks.

Workflow run 43416011439 covered the recycling run's policies: 64
episodes, 30 cm gaps, MuJoCo with stiff contacts.

| Policy, seed | RNE: crossed, falls, distance | MuJoCo: crossed | Episodes where RNE crossed fewer / equal / more |
|---|---|---|---|
| Scan 2026 | 315, 4, 9.45 m | 442 | 57 / 6 / 1 |
| Scan 2027 | 277, 3, 8.51 m | 479 | 60 / 4 / 0 |
| Scan 2028 | 285, 1, 8.66 m | 382 | 24 / 36 / 4 |
| Blind 2026 | 53, 18, 3.02 m | 59 | 23 / 12 / 29 |
| Blind 2027 | 6, 11, 1.68 m | 69 | 40 / 24 / 0 |
| Blind 2028 | 0, 14, 1.67 m | 63 | 37 / 27 / 0 |

- **The difference is not the starts.** From the same 64 points the scan
  policies still cross about a third fewer gaps in RNE than in MuJoCo
  with stiff contacts. The settled start positions agree to a millimeter.
- **Slower, mostly.** Of the scan episodes where RNE crossed fewer, 12–41
  ended still walking, with no fall and no foot in a gap: the same policy
  covers 8.5–9.5 m in RNE and 11–13 m in MuJoCo. Another 11–16 ended with
  a front foot in a gap, which happens in RNE and hardly ever in MuJoCo
  (0–1 falls there, though its feet were not recorded).
- **Blind policies.** Two blind policies cross almost nothing in RNE, 0
  and 6 gaps, and 63–69 in MuJoCo. So a foot that drops into a gap fares
  differently in the two simulators, not just the walking speed. Which
  contact causes it is not established yet.
- **MuJoCo's own spread.** Its totals differ a little between this run
  and the earlier one on the same policies and starts (59 against 61, 63
  against 70, 382 against 385), so MuJoCo's walks are not bit-reproducible
  across runners.

#### The cause: spheres hit the air above a gap's rim

Tracing one episode in both simulators (`walk_policy.py --trace`, MuJoCo's
`evaluate --trace-episode`) showed where they part. Blind policy 2028
bounds toward its first gap at 2 m/s, and the two walks agree to about a
centimeter until the body is over the gap. MuJoCo carries it across. In
RNE the base stopped dead within two control periods, 25 cm above the
floor, and the only link in contact was the head.

RNE's spheres touched the terrain through the tangent plane below their
center, taken as endless. Over the far wall of a gap, which climbs 30 cm
across one 5 cm cell, that plane rises past the rim: a sphere 20 cm above
the floor was 3–8 cm from it and was pushed back level. `rne_locomotion`
now uses the plane only where it lies on the terrain, and the distance to
the terrain's nearest point elsewhere. Contacts on gentle relief and flat
floors are unchanged.

Workflow run 43523469012 walked the same policies again, from the same
starts, in RNE with the fix. The policies were still the ones trained
before it.

| Policy, seed | RNE before: crossed, falls | RNE fixed: crossed, falls, distance | MuJoCo stiff: crossed |
|---|---|---|---|
| Scan 2026 | 315, 4 | 443, 0, 12.37 m | 441 |
| Scan 2027 | 277, 3 | 496, 0, 13.60 m | 480 |
| Scan 2028 | 285, 1 | 407, 0, 11.62 m | 382 |
| Blind 2026 | 53, 18 | 80, 42, 3.64 m | 59 |
| Blind 2027 | 6, 11 | 82, 18, 3.53 m | 69 |
| Blind 2028 | 0, 14 | 84, 19, 3.76 m | 70 |

- **RNE and MuJoCo now agree.** The scan policies cross 407–496 gaps in
  RNE against 382–480 in MuJoCo with stiff contacts, and in 41–48 of 64
  episodes they cross the same number in both.
- **The earlier RNE numbers on 30 cm gaps were depressed by the fault.**
  That holds for every gap result above this section, including the
  training runs: the policies learned on gaps with an unseen wall over
  each far rim.
- **The scan still decides it.** Blind policies cross about 80 gaps and
  fall in 18–42 of 64 episodes; scan policies cross five times as many
  and do not fall.

#### Retrained with the fixed contact

Workflow run 43541974300 trained again with the same settings (50M
timesteps, eight depth levels, recycling) and the contact fixed. These
are the last stage's 64 held-out episodes at each depth.

| Policy, seed | 7.5 cm | 15 cm | 30 cm | Falls at 30 cm | Distance at 30 cm |
|---|---|---|---|---|---|
| Scan 2026 | 670 | 692 | 695 | 0 | 18.30 m |
| Scan 2027 | 634 | 658 | 659 | 0 | 17.32 m |
| Scan 2028 | 653 | 683 | 677 | 0 | 18.05 m |
| Blind 2026 | 494 | 470 | 471 | 2 | 13.01 m |
| Blind 2027 | 397 | 358 | 298 | 1 | 8.88 m |
| Blind 2028 | 478 | 423 | 378 | 1 | 10.95 m |

- **Without the unseen wall, the gaps are far easier.** The scan
  policies cross about eleven 30 cm gaps an episode without a fall, at
  3.6 m/s, close to their speed on a flat floor. Before the fix they
  crossed 243–338.
- **Blind policies cross them too.** They cross 298–471 gaps at 30 cm,
  where they crossed 0–66 before, by running fast enough that a foot
  seldom stays in a gap. The gaps no longer need sight. The scan is still
  worth about 1.5–2 times the crossings and the distance.
- **Every level is in training.** The training environments spread over
  all eight levels, as recycling intends.

The MuJoCo workflow (run 43557248261) walked these policies from the same
64 starts on 30 cm gaps:

| Policy, seed | RNE: crossed, falls, distance | MuJoCo stiff: crossed | MuJoCo default: crossed |
|---|---|---|---|
| Scan 2026 | 693, 0, 18.30 m | 452 | 31 |
| Scan 2027 | 663, 0, 17.35 m | 495 | 30 |
| Scan 2028 | 685, 0, 18.13 m | 310 | 6 |
| Blind 2026 | 485, 0, 13.23 m | 314 | 16 |
| Blind 2027 | 304, 2, 9.20 m | 261 | 18 |
| Blind 2028 | 362, 4, 10.54 m | 291 | 6 |

- **The faster walks transfer less well.** In MuJoCo with stiff contacts
  the scan policies cross 45–75% of what they cross in RNE, and scan 2026
  falls in 10 of 64 episodes there against none in RNE. The policies
  trained before the fix walked at 2.4 m/s and matched MuJoCo within a
  few percent once the contact was fixed.
- **Open.** Whether 3.6 m/s is a walk the real robot's actuators could
  produce, or one that leans on something RNE's plant allows and MuJoCo's
  does not, is not established.

#### The fast walks overdrive the calf motors

`trace_limits.py` sums a traced walk's joint speeds and PD torques against
the URDF's limits. Both simulators clamp torque to the effort limit, and
neither enforces the velocity limit. The retrained policies' traced
episodes (workflow run 43560671958):

| Walk | Base speed | Calf speed: 95th percentile, peak (limit 15.7 rad/s) | Calf samples over the limit | Thigh torque at its limit |
|---|---|---|---|---|
| Scan 2026, RNE | 3.65 m/s | 21.4, 28.8 rad/s | 17% | 6% |
| Scan 2026, MuJoCo | 3.41 m/s | 21.6, 26.4 rad/s | 20% | 6% |
| Scan 2027, RNE | 3.54 m/s | 21.3, 30.8 rad/s | 20% | 7% |
| Scan 2027, MuJoCo | 3.28 m/s | 21.4, 29.2 rad/s | 19% | 6% |
| Blind 2026, RNE | 1.86 m/s | 17.1, 27.7 rad/s | 7% | 1% |
| Blind 2026, MuJoCo | 2.58 m/s | 18.5, 26.1 rad/s | 10% | 1% |

- **The calves turn up to twice as fast as their motors allow.** The
  URDF gives the calf 15.7 rad/s; the scan walks exceed it in about a
  fifth of the samples and peak near 30 rad/s. Hips and thighs stay under
  their 30.1 rad/s.
- **Both simulators allow it.** The numbers agree between RNE and MuJoCo,
  so this is not what separates them. It is a limit the task's plant
  lacks: a real Go2 could not walk these gaits as modeled.
- **Torque is rarely the constraint.** The thighs sit at their effort
  limit in 6–7% of the samples, the other joints almost never.

`velocity_limits=True` (`train_ppo.py --velocity-limits`,
`Go2LearningConfig::velocity_limits`) makes the motors hold those limits:
a motor whose joint turns at its limit or faster gives no torque in the
direction it turns, though it still brakes. MuJoCo's script applies the
same rule (`evaluate --velocity-limits`, or a policy file whose task says
so). It is off by default, so example 147's task and the results above
are unchanged. The depth-levels workflow now trains with it.

#### Trained with the velocity limits: the rule is too weak

Workflow run 43563466782 trained with `--velocity-limits` and the same
settings otherwise, and run 43575923691 walked the policies in both
simulators from the same 64 starts. The table gives the 30 cm gaps.

| Policy, seed | RNE: crossed, falls, distance | Without the limits: crossed, distance | MuJoCo stiff: crossed, falls |
|---|---|---|---|
| Scan 2026 | 627, 3, 16.55 m | 693, 18.30 m | 239, 34 |
| Scan 2027 | 599, 0, 15.99 m | 663, 17.35 m | 406, 6 |
| Scan 2028 | 635, 0, 16.93 m | 685, 18.13 m | 167, 40 |
| Blind 2026 | 373, 2, 10.84 m | 485, 13.23 m | 266, 5 |
| Blind 2027 | 337, 17, 9.80 m | 304, 9.20 m | 239, 7 |
| Blind 2028 | 298, 13, 9.09 m | 362, 10.54 m | 282, 6 |

- **The walks are hardly slower.** The scan policies still cover 16–17 m
  in 5 s, about 8% less than without the limits.
- **The calves still pass their limit.** In the traced scan walks they are
  over 15.7 rad/s in 5–15% of the samples, against 17–20% before, and
  still peak at 28–31 rad/s. A motor that only stops pushing at its limit
  does not stop the joint: the leg's swing and the ground carry it past.
- **Transfer to MuJoCo got worse.** With stiff contacts the scan policies
  cross 167–406 gaps there and two of them fall in 34 and 40 of 64
  episodes. Why is not established. The two simulators apply the rule at
  different rates (every 5 ms in RNE, every 1 ms of MuJoCo's finer steps),
  which is one candidate.

So this rule does not make the task's walks ones the real motors could
turn. A motor model whose torque falls with speed and brakes past the
limit, or a penalty on joint speed in the reward, is still needed.

#### A torque-speed curve

`velocity_limits` now gives the motors the DC motor model legged-robot
simulators use, with the stall torque at the effort limit
(`motor_torque_range`): the torque a motor can give in the direction its
joint turns falls in a line from the effort limit at rest to zero at the
velocity limit, and past the limit the motor must brake. MuJoCo's script
applies the same curve each physics step. The results above this section
used the weaker rule, which gave no torque past the limit and nothing
else.

Trained with the curve (workflow run 43581407624) and walked in both
simulators from the same 64 starts on 30 cm gaps (run 43594323954):

| Policy, seed | RNE: crossed, falls, distance | MuJoCo stiff: crossed, falls | Calf peak speed in the traced walk: RNE, MuJoCo |
|---|---|---|---|
| Scan 2026 | 627, 0, 16.76 m | 276, 2 | 56.8, 19.9 rad/s |
| Scan 2027 | 601, 0, 16.05 m | 239, 28 | 52.9, 20.6 rad/s |
| Scan 2028 | 582, 0, 15.61 m | 510, 0 | 57.2, 20.2 rad/s |
| Blind 2026 | 336, 6, 9.77 m | 171, 2 | 52.5, 20.8 rad/s |
| Blind 2027 | 330, 5, 9.90 m | 263, 10 | 42.7, 19.0 rad/s |
| Blind 2028 | 248, 3, 7.90 m | 74, 26 | 46.4, 17.5 rad/s |

- **The curve holds in MuJoCo and not in RNE.** The same policies under
  the same curve turn the calves at 17–21 rad/s at most in MuJoCo, over
  the 15.7 rad/s limit in 1–7% of samples. In RNE they reach 43–57 rad/s,
  faster than with no limit at all, and the walks are as fast as before
  (16–17 m in 5 s).
- **Likely cause: the curve is applied explicitly at RNE's 5 ms step.**
  Past the limit the curve acts as a damper of 45.43 / 15.7 = 2.9 N·m·s/rad
  on the calf, whose inertia about the knee is about 0.005 kg·m². An
  explicit damper is stable only while damping × step / inertia stays
  under 2; here it is 2.9, so the torque overshoots and the joint
  chatters between the curve's ends. MuJoCo's stiff setting steps at 1 ms,
  where the same number is 0.6. This is arithmetic from the URDF, not yet
  a measurement of the chatter itself.
- **So these RNE walks are not a result about the motors.** The task
  needs the curve applied stably in RNE, by solving it together with the
  step or by finer steps, before training with it means anything.

#### Braking no harder than a step allows

A hop in RNE confirmed the chatter: with the curve as written, a calf in
the air went 30, −20, 20, 5, 25, −10 rad/s over successive 5 ms steps.
`motor_torque_range_for_step` now brakes a joint past its limit no harder
than brings it back to the limit in one step, were the joint as light as
it gets (each joint's own inertia with the calves folded: 0.005 kg·m² for
a calf). The same hop then settles at 30, 15, 21, 16, 17, 16 rad/s.
MuJoCo's script keeps the plain curve, which its 1 ms steps hold.

One thing remains in both simulators' plants: the joints have no rotor
inertia, so a PD kick of 20 N·m still takes a calf in the air from rest
to 30 rad/s in a single 5 ms step, before the curve can act.

Trained with the stable curve (workflow run 43596078138) and walked in
both simulators from the same 64 starts on 30 cm gaps (run 43611371646):

| Policy, seed | RNE: crossed, falls, distance | MuJoCo stiff: crossed, falls | Calf speed in the traced walk, RNE: 95th percentile, peak |
|---|---|---|---|
| Scan 2026 | 606, 0, 16.24 m | 231, 25 | 19.4, 26.3 rad/s |
| Scan 2027 | 614, 0, 16.49 m | 375, 16 | 19.0, 25.6 rad/s |
| Scan 2028 | 670, 0, 17.56 m | 275, 37 | 18.9, 26.3 rad/s |
| Blind 2026 | 338, 3, 9.90 m | 277, 4 | 15.8, 24.2 rad/s |
| Blind 2027 | 258, 18, 7.96 m | 249, 11 | 9.8, 19.1 rad/s |
| Blind 2028 | 330, 3, 10.14 m | 279, 4 | 14.2, 20.1 rad/s |

- **The curve now holds in RNE.** The calves peak at 19–26 rad/s where
  they reached 43–57, and RNE's and MuJoCo's joint speeds agree again
  (MuJoCo: 19–24 rad/s peak). The scan walks still pass the calf's
  15.7 rad/s in about a tenth of the samples.
- **The walks are no slower.** The scan policies still cover 16–17.5 m in
  5 s. The calves' overspeed was not what made them fast: with the motors
  held near their limits the gait still runs at about 3.3 m/s.
- **Blind policies now agree across the simulators.** They cross 258–338
  gaps in RNE and 249–279 in MuJoCo.
- **Scan policies do not.** They cross 606–670 gaps in RNE without a fall
  and 231–375 in MuJoCo, falling in 16–37 of 64 episodes there. What the
  faster scan gait relies on that MuJoCo does not give it is not
  established.
- **The scan is worth about twice the crossings** in RNE, as before.

#### Not contact softness; partly how often the torque is set

Is what is left between the simulators contact softness? MuJoCo walked
the same policies from the same starts with stiffer contacts still: ten
physics steps to each of RNE's with a 2 ms time constant, and twenty with
1 ms (run 43630116940). Crossed and falls of 64 episodes:

| Policy, seed | RNE | MuJoCo stiff (4 ms) | Stiffer (2 ms) | Stiffest (1 ms) |
|---|---|---|---|---|
| Scan 2026 | 606, 0 | 231, 25 | 202, 18 | 183, 22 |
| Scan 2027 | 614, 0 | 375, 16 | 390, 14 | 373, 17 |
| Scan 2028 | 670, 0 | 275, 37 | 287, 30 | 267, 27 |
| Blind 2026 | 338, 3 | 277, 4 | 265, 1 | 264, 3 |
| Blind 2027 | 258, 18 | 249, 11 | 266, 7 | 254, 11 |
| Blind 2028 | 330, 3 | 288, 2 | 277, 6 | 285, 5 |

Stiffer contacts do not bring MuJoCo closer to RNE, so contact softness is
not the gap.

RNE sets each joint's PD torque once per 5 ms step and holds it; MuJoCo's
position actuators recompute it every physics step, 1 ms in the stiff
setting. `mujoco_go2_learning.py --hold-torque` sets the torque once per
step of RNE's and holds it, with the same braking cap
(`motor_torque_range_for_step`). Stiff contacts, run 43631841078; crossed,
falls and distance:

| Policy, seed | RNE | MuJoCo stiff | MuJoCo stiff, torque held |
|---|---|---|---|
| Scan 2026 | 606, 0, 16.24 m | 231, 25, 7.52 m | 369, 17, 10.66 m |
| Scan 2027 | 614, 0, 16.49 m | 375, 16, 10.81 m | 450, 12, 12.44 m |
| Scan 2028 | 670, 0, 17.56 m | 275, 37, 8.41 m | 335, 30, 9.85 m |
| Blind 2026 | 338, 3, 9.90 m | 288, 6, 8.76 m | 290, 10, 8.83 m |
| Blind 2027 | 257, 19, 7.94 m | 253, 10, 7.89 m | 241, 10, 7.53 m |
| Blind 2028 | 330, 3, 10.14 m | 288, 2, 9.07 m | 234, 4, 7.68 m |

- **Holding the torque closes part of the scan gap.** With torques set as
  RNE sets them, MuJoCo's scan walks cross 60–138 more gaps, fall less and
  go 1.4–3.1 m further. That is about a third of the difference from RNE.
  The blind walks, which are slower, do not gain.
- **The fast scan gait uses the 5 ms hold.** A PD torque held over 5 ms
  acts on a joint that has moved meanwhile. The policy learned to drive the
  legs through that, and it is a property of RNE's plant, not of the Go2.
  The robot's motor drivers close the PD loop far faster than 200 Hz, so
  MuJoCo's per-millisecond PD is the more faithful plant here.
- **Two thirds of the gap remain**, with the torque update rate matched
  and contacts stiff.
- The counts move by a few percent between CI runs on different runners in
  both simulators (MuJoCo stiff, blind 2026: 277 then 288), so differences
  of that size are not results.

#### The PD integrated within each step

`Go2LearningConfig::implicit_pd` (`train_ppo.py --implicit-pd`) integrates
the joints' PD within each 5 ms physics step, from the state at its end,
as `rne_dynamics`' contact step already could: the PD then acts as a
continuous one does, as the robot's motor drivers do, and stays stable at
the 5 ms step. The torque-speed curve rides along as per-joint torque bounds
(`JointPdControl::torque_bounds`), set from the speed at the step's start.

First, the policies above, trained with the held torque, walked in RNE with
the implicit PD (run 43640346933; crossed and falls):

| Policy, seed | RNE, held | RNE, implicit | MuJoCo stiff |
|---|---|---|---|
| Scan 2026 | 606, 0 | 527, 0 | 231, 25 |
| Scan 2027 | 614, 0 | 506, 2 | 375, 16 |
| Scan 2028 | 670, 0 | 551, 2 | 276, 38 |
| Blind 2026 | 338, 3 | 257, 10 | 277, 4 |
| Blind 2027 | 257, 19 | 155, 12 | 253, 10 |
| Blind 2028 | 330, 3 | 187, 10 | 279, 4 |

The implicit PD alone does not make RNE behave like MuJoCo for these
policies: the scan walks lose a sixth of their crossings but still hardly
fall, and the blind walks do worse than in MuJoCo.

Then retrained with the implicit PD (and the torque-speed curve; run
43640346938) and walked in both simulators from the same 64 starts (run
43662833933); crossed, falls and distance:

| Policy, seed | RNE, implicit | MuJoCo stiff | MuJoCo stiff, torque held |
|---|---|---|---|
| Scan 2026 | 579, 0, 15.52 m | 517, 8, 14.11 m | 493, 16, 13.67 m |
| Scan 2027 | 565, 0, 15.14 m | 457, 6, 12.74 m | 501, 13, 13.84 m |
| Scan 2028 | 543, 0, 14.47 m | 352, 32, 10.44 m | 335, 38, 9.95 m |
| Blind 2026 | 290, 4, 8.95 m | 214, 10, 7.18 m | 206, 17, 6.94 m |
| Blind 2027 | 233, 4, 7.44 m | 168, 8, 5.77 m | 156, 11, 5.49 m |
| Blind 2028 | 298, 6, 9.07 m | 265, 6, 8.35 m | 242, 16, 7.74 m |

- **The scan policies now mostly transfer.** In MuJoCo they cross 352–517
  gaps where the held-torque policies crossed 231–375, and two of the three
  fall 6–8 times where they fell 16–25. They keep 65–90% of their RNE
  crossings, against 38–61% before.
- **They no longer gain from the held torque in MuJoCo**, so they no longer
  rely on it.
- **They walk a little slower**, 2.9–3.1 m/s in RNE against 3.3 before,
  still crossing about twice the gaps the blind policies do.
- **Seed 2028 still falls in MuJoCo** (32 of 64 episodes), and every policy
  crosses more gaps in RNE: what is left is not established.
- MuJoCo as configured by default (20 ms contact time constant, 5 ms steps)
  stays far off (0–14 crossings), as for every policy before.

#### Where MuJoCo's walks fall: the same physics, a brittle policy

Scan 2028 falls in 32 of its 64 MuJoCo walks and in none in RNE. 14 of
those falls end with the base over the far rim of the widest gap on the
course (6.39–6.73 m, 34 cm), its hind legs above the gap. A trace of one
(run 43671392747) shows how: about 0.9 s before the stall a front foot
lands on the far rim, slides 10 cm down the wall, the speed drops from 2.9
to 1.7 m/s, the robot pitches nose-down onto its head and stops. The other
policies' MuJoCo falls go the same two ways: a front foot lands on a rim,
or a hind foot drops into a gap and the calf strikes its far wall.

Does RNE's physics do otherwise? Example 147's `--replay` sets RNE to each
state of a MuJoCo trace (`Go2LearningRobot::set_state_z_up`) and steps it
with the targets MuJoCo applied there, one control period from each state
or open loop from one:

- From MuJoCo's state 0.2 s before the front foot reaches the rim, RNE
  with MuJoCo's targets follows MuJoCo's fall within 1–2 cm for 0.9 s: the
  foot slides into the gap, the head strikes, the robot stalls.
- One period at a time over whole walks (four policies), RNE's base
  velocity after a period differs from MuJoCo's by 1–10 mm/s on average,
  with a spread of 1–7 cm/s forward and about 5 cm/s up.

So the simulators agree on the fall. The walks part slowly instead:
from the same start, MuJoCo's walk is 35 cm ahead after 2.6 s, which brings
the robot to the wide gap at another phase of its stride, one where the
policy puts a front foot on the rim. Does a small disturbance in RNE do the
same? The policies walked again in RNE (run 43672568051) with Gaussian noise
on every action (standard deviation in action units; one unit is 0.5 rad)
and on other floors; crossed and falls:

| Policy, seed | RNE | Noise 0.05 | 0.1 | 0.2 | Friction 0.7 | Friction 1.3 | MuJoCo stiff |
|---|---|---|---|---|---|---|---|
| Scan 2026 | 579, 0 | 567, 0 | 499, 4 | 292, 18 | 540, 2 | 562, 1 | 517, 8 |
| Scan 2027 | 565, 0 | 555, 0 | 540, 1 | 380, 15 | 449, 6 | 553, 1 | 460, 6 |
| Scan 2028 | 543, 0 | 521, 1 | 509, 0 | 258, 30 | 452, 7 | 539, 0 | 363, 31 |
| Blind 2026 | 289, 5 | 277, 2 | 250, 4 | 192, 9 | 250, 11 | 214, 10 | 218, 11 |
| Blind 2027 | 233, 4 | 222, 4 | 193, 8 | 151, 8 | 208, 7 | 183, 10 | 168, 8 |
| Blind 2028 | 298, 6 | 302, 2 | 301, 4 | 203, 15 | 223, 16 | 295, 2 | 265, 6 |

- **The policies are brittle.** Trained in one deterministic plant, they
  fall in RNE too once it is disturbed: noise of 0.2 (0.1 rad on each
  target) makes the scan policies fall 15–30 times, and a floor of
  friction 0.7 costs up to a fifth of their crossings.
- **MuJoCo's walks look like disturbed RNE walks.** Scan 2028 falls 31
  times in MuJoCo and 30 in RNE with noise 0.2; the others sit between
  noise 0.1 and 0.2.
- **What is left between the simulators is thus not a fault of either.**
  It is a policy that has learned one plant's exact dynamics. Training it
  under disturbance (domain randomization: friction, masses, pushes,
  action noise or delay) is the standard remedy, and the next step.

#### A third simulator: Genesis

`examples/147_go2_learning/genesis_go2_learning.py` runs the MuJoCo
script's episode, observation, policies and starts on a
[Genesis](https://github.com/Genesis-Embodied-AI/Genesis) plant (1.4.6, on
the CPU): the same URDF, the terrain as a Genesis height field, Genesis's
position control at kp 20 and kd 0.5 within the effort limits, the
torque-speed curve before every 1 ms physics step. Genesis gives every
joint 0.1 kg·m² of rotor inertia by default, twenty times a calf's own;
that is turned off, as RNE and MuJoCo have none (until
[Rotor inertia](#rotor-inertia), where all three give 0.01 kg·m²). Checked first (run
43679869584): the robot weighs 16.087 kg and stands at 0.248 m, as in RNE,
and stood with its hind feet over a gap it sinks into it while on flat
floor it stands level, so the terrain lies as RNE's.

The same policies (run 43640346938) from the same 64 starts; crossed,
falls and distance:

| Policy, seed | RNE | MuJoCo stiff | Genesis |
|---|---|---|---|
| Scan 2026 | 579, 0, 15.5 m | 517, 8, 14.1 m | 464, 7, 12.7 m |
| Scan 2027 | 565, 0, 15.1 m | 460, 6, 12.8 m | 458, 7, 12.6 m |
| Scan 2028 | 543, 0, 14.5 m | 363, 31, 10.6 m | 330, 25, 9.8 m |
| Blind 2026 | 290, 4, 9.0 m | 218, 11, 7.2 m | 207, 10, 6.8 m |
| Blind 2027 | 233, 4, 7.4 m | 168, 8, 5.8 m | 143, 16, 5.2 m |
| Blind 2028 | 298, 6, 9.1 m | 265, 6, 8.3 m | 231, 19, 7.6 m |

- **Genesis lands where MuJoCo does.** Both cross 10–40% fewer gaps than
  RNE and fall in about as many episodes; scan 2028 is the weak policy in
  both (31 and 25 falls).
- **Episode by episode they agree no better than with RNE.** The
  correlation of gaps crossed per start is -0.47 to +0.26 between any two
  of the three simulators, MuJoCo and Genesis included, and the two share
  few of their falls (0–2 of 6–19, except scan 2028's 15 of 25–31 at the
  wide gap). Which start fails is chaos; how many fail is the policy.
- **That is what a policy fitted to one plant looks like.** It does best in
  the simulator it trained in and about equally worse in any other, as it
  does in RNE disturbed by action noise of 0.1–0.2. It does not tell RNE's
  physics from the other two's; a policy trained under disturbance should
  narrow the gap in both if the policy, not RNE, is the cause.

#### Trained under disturbance

`train_ppo.py --domain-randomization` trains under disturbance: each
episode draws the floor's friction from 0.5–1.25 and a factor of 0.9–1.1
on the PD gains, every action the robot tracks carries Gaussian noise of
0.1, and about once an episode the base is pushed by up to 0.5 m/s along x
and y (`Go2LearningConfig::randomization`). The held-out evaluation stays
undisturbed. Trained so (run 43674672352) and walked from the same 64
starts in RNE, in RNE disturbed, in MuJoCo (run 43684863741) and in Genesis
(`genesis_go2_learning.py`, run 43684866345); crossed and falls:

| Policy, seed | RNE | Noise 0.1 | Noise 0.2 | Friction 0.7 | MuJoCo stiff | Genesis |
|---|---|---|---|---|---|---|
| Scan 2026 | 497, 0 | 485, 0 | 445, 4 | 501, 0 | 448, 7 | 428, 12 |
| Scan 2027 | 592, 2 | 584, 0 | 521, 4 | 603, 0 | 530, 7 | 491, 14 |
| Scan 2028 | 547, 0 | 532, 0 | 479, 2 | 548, 0 | 483, 7 | 488, 10 |
| Blind 2026 | 266, 4 | 257, 6 | 222, 8 | 284, 2 | 276, 6 | 265, 4 |
| Blind 2027 | 266, 1 | 261, 7 | 235, 3 | 271, 3 | 252, 7 | 234, 9 |
| Blind 2028 | 168, 12 | 157, 13 | 132, 12 | 161, 12 | 140, 15 | 166, 9 |

Against the policies trained undisturbed (above): scan RNE 579/565/543,
MuJoCo 517/460/363 with 8/6/31 falls, Genesis 464/458/330 with 7/7/25;
under noise 0.2 they fell 18/15/30 times in RNE.

- **The gap between the simulators mostly closes.** The scan policies keep
  88–90% of their RNE crossings in MuJoCo and 83–89% in Genesis, against
  61–89% and 57–81% before; scan 2028, the weak one, now crosses 483 gaps
  in MuJoCo with 7 falls where it crossed 363 with 31. The blind policies
  cross about as many gaps in all three (83–104% of RNE).
- **The policies tolerate disturbance.** Noise of 0.2 now costs the scan
  policies 2–4 falls where it cost 15–30, and a floor of friction 0.7
  costs nothing.
- **In RNE they give up a little.** The scan policies cross 497–592 gaps,
  about what they crossed undisturbed-trained (543–579), and walk 13.6–15.9
  m; one blind seed (2028) learned a weaker gait.
- **So the gap was the policy.** Trained in one exact plant, the policies
  learned that plant; trained under disturbance, they walk about as well
  in any of the three. What is left, 7–14 falls in MuJoCo and Genesis
  against 0–2 in RNE, is not established.

#### Rotor inertia

A real Go2 joint turns a motor rotor through a gearbox, and the rotor's
inertia, multiplied by the gear ratio squared, adds to the joint's: MuJoCo
Menagerie's Go2 gives every joint 0.01 kg·m² of it, about twice a calf's
own inertia about its axis. `ArticulatedModel::set_joint_armature` adds such
an inertia to a joint's diagonal of the mass matrix (`GO2_ARMATURE_KG_M2`,
`Go2LearningConfig::armature_kg_m2`, `train_ppo.py --armature`); the walks
carry it into MuJoCo (`armature`) and Genesis. Trained so, under the same
disturbance as above (run 43699237646), and walked from the same 64 starts
in RNE, RNE disturbed, MuJoCo (run 43712341341) and Genesis (run
43712343553), all three with 0.01 kg·m²; crossed and falls:

| Policy, seed | RNE | Noise 0.1 | Noise 0.2 | Friction 0.7 | MuJoCo stiff | Genesis |
|---|---|---|---|---|---|---|
| Scan 2026 | 485, 0 | 477, 0 | 447, 0 | 481, 0 | 481, 2 | 475, 1 |
| Scan 2027 | 484, 0 | 473, 0 | 435, 0 | 483, 0 | 422, 9 | 472, 1 |
| Scan 2028 | 407, 0 | 394, 0 | 360, 1 | 408, 0 | 296, 22 | 379, 8 |
| Blind 2026 | 169, 9 | 173, 12 | 156, 10 | 187, 7 | 174, 8 | 161, 8 |
| Blind 2027 | 211, 11 | 198, 8 | 183, 8 | 204, 11 | 191, 7 | 179, 14 |
| Blind 2028 | 159, 7 | 154, 7 | 136, 6 | 141, 11 | 142, 7 | 148, 5 |

Against the policies trained without it (above): scan RNE 497/592/547,
MuJoCo 448/530/483 with 7/7/7 falls, Genesis 428/491/488 with 12/14/10.

- **Genesis now agrees with RNE.** The scan policies keep 93–98% of their
  RNE crossings in Genesis, against 83–89% without the inertia, and fall
  1/1/8 times where they fell 12/14/10.
- **MuJoCo agrees with two of three.** Scan 2026 keeps 99% of its
  crossings in MuJoCo with 2 falls; scan 2027 keeps 87% with 9. Scan
  2028, again the weak seed, keeps 73% and falls 22 times, against 7
  before. Where the two disagree start by start, RNE crosses more and is
  walking, not fallen.
- **The legs are slower, so the policies cross less.** With the same
  training budget the scan policies cross 407–485 gaps in RNE, against
  497–592, and walk 11.4–13.3 m, against 13.6–15.9 m; the blind policies
  cross 159–211, against 168–266. Noise of 0.2 costs them 0–1 falls.
- **So part of the rest was the plant.** A rotor every real Go2 has makes
  RNE and Genesis agree on all three scan policies; MuJoCo's falls gather
  in one policy, and why MuJoCo alone keeps them is not established.

#### What is left: MuJoCo's stiff contacts bounce

Scan 2028 still falls 22 times in MuJoCo and none in RNE. Two tools look
at why, both replaying MuJoCo's trace of a fall (run 43736761486) one
control period at a time from each of its states with the targets MuJoCo
applied: example 147's `--replay` steps RNE, `mujoco_go2_learning.py
replay` steps MuJoCo again, and each reports the ground's normal impulse
through every link it touched.

- **The 5 ms step biases the PD a little.** From MuJoCo's states RNE's
  implicit PD ends each period with the joints 0.21–0.27 rad/s slower than
  MuJoCo's, whose actuators recompute the PD every millisecond.
  `Go2LearningRobot::set_physics_substeps` (`--substeps`) splits each 5 ms
  step: at five, the bias is gone and the base's angular velocity after a
  period differs from MuJoCo's by a third as much. The walks hardly change
  (below), so this is not why MuJoCo's fall.
- **Feet agree; shins and the head do not.** Summed over the traces, the
  feet's impulses differ by 5%. But when a shin or the head strikes the
  floor, MuJoCo's stiff setting (4 ms, damping ratio 1) gives it about
  twice RNE's impulse (shins 14.4 against 7.0 N·s, 10.2 against 4.7):
  the robot's nose, coming down at 0.6–0.8 m/s, stops or rises (up to
  +0.2 m/s), where in RNE it still sinks at about 0.2 m/s. Twice the
  impulse is what a fully elastic bounce takes. The replays' errors gather there: periods
  with a shin or head contact differ by 7.4 cm/s in base velocity (median),
  the rest by 0.8 cm/s.
- **Damped, MuJoCo takes those strikes as RNE does.** With the contacts
  damped five times critically (`--dampratio 5`), MuJoCo's shin impulses
  come within about 10% of RNE's period by period (6.6 against 7.0 N·s,
  4.0 against 4.7) and the nose comes down as in RNE (−0.21 m/s against
  −0.21); over a trace the feet's impulses match RNE's to 0.3%. The head still bounces (16.6 against 11.0 N·s in
  scan 2028's fall).

The same policies (run 43699237646, with rotor inertia) from the same 64
starts; crossed and falls:

| Policy, seed | RNE | RNE, 1 ms | MuJoCo stiff | Damped 2× | Damped 5× |
|---|---|---|---|---|---|
| Scan 2026 | 485, 0 | 493, 0 | 476, 4 | 478, 2 | 450, 4 |
| Scan 2027 | 484, 0 | 482, 0 | 422, 9 | 449, 9 | 445, 5 |
| Scan 2028 | 407, 0 | 415, 0 | 296, 22 | 346, 13 | 374, 10 |
| Blind 2026 | 169, 9 | 141, 22 | 174, 8 | 167, 11 | 162, 9 |
| Blind 2027 | 211, 11 | 178, 10 | 191, 7 | 199, 10 | 189, 11 |
| Blind 2028 | 159, 7 | 132, 17 | 142, 7 | 141, 11 | 132, 10 |

- **About half of scan 2028's MuJoCo falls are the bounce.** Damped, it
  falls 10 times where it fell 22, crosses 374 gaps where it crossed 296,
  and matches RNE's walk in 42 of 64 episodes where it matched 29. Scan
  2027 falls 5 times where it fell 9.
- **At 1 ms RNE walks as at 5 ms**: the scan policies cross the same gaps
  and still never fall. The blind policies do worse at 1 ms (141–178
  crossed), which they never trained at.
- **What remains**, 4–10 falls of the scan policies in damped MuJoCo, is
  not established; the head strike is one candidate.

#### The head strike: friction in a soft contact

Damped, MuJoCo still gives a striking head half again RNE's impulse. The
same replays (run 43775129292) with one setting changed at a time tell why:
neither damping 20 times critically, nor impratio 1, nor the pyramidal cone
changes it (21.6, 21.5, 20.9 and 18.2 N·s in one strike, against RNE's
12.7), but without friction MuJoCo and RNE agree (7.0 against 7.3 N·s; 4.4
against 3.5, 3.5 against 3.5 in others). The head strikes while the robot
runs forward, and friction at the head, ahead of and below the centre of
mass, pitches the robot further into the floor: the strike's normal impulse
grows with the friction. How much is a question with a closed answer.

A 2 kg box, pitched 0.2 rad nose-down, slides forward at 2.6 m/s onto its
front bottom edge at 0.8 m/s, with no gravity. A rigid impact without
restitution, sliding throughout, gives the edge
`P_n = -u_n / (1/m + r_x²/I + μ r_x r_y/I)`: 0.559 N·s without friction and
1.138 N·s with μ = 1. `rne_dynamics`' test
`a_sliding_edge_strike_takes_the_coulomb_impulse` checks RNE, and
`examples/147_go2_learning/mujoco_strike.py` MuJoCo (run 43777399200);
normal impulse, N·s:

| Friction | Closed form | RNE | MuJoCo default | Stiff | Damped 5× | Pyramidal | Impratio 1 |
|---|---|---|---|---|---|---|---|
| 0 | 0.559 | 0.559 | 0.592 | 0.632 | 0.552 | 0.632 | 0.632 |
| 1 | 1.138 | 1.138 | 2.021 | 2.505 | 2.462 | 2.363 | 2.455 |

- **The check found a bug in RNE first.** Its solve brackets the sliding
  direction between sampled angles; here the solution fell exactly on a
  sample, rounding hid it, and the solve fell back to an impulse that broke
  the contact law: 2.749 N·s, the edge leaving the floor at 1.1 m/s. It is
  fixed (`CHANGELOG.md`); the Go2's replays never hit the case and are
  unchanged.
- **MuJoCo gives the frictional strike about twice the rigid impulse**, at
  any damping, cone or impratio, and the edge leaves the floor at 0.7–1.1
  m/s. Without friction it lands within 13% of the rigid answer, exactly
  when damped. Its soft contact lets the edge sink while friction keeps
  pitching the box into the floor, and the spring then throws the edge back
  out.
- **So the head strikes part where MuJoCo leaves the rigid answer.** In the
  walks a head that strikes at a run takes half again to twice the impulse
  in MuJoCo, and the robot pitches up and slows down where in RNE it does
  not. Whether all of scan 2028's remaining falls in damped MuJoCo run
  through such strikes is not established.

#### Three references

What the sections above leave as the comparison: RNE against Genesis and
against MuJoCo damped, the setting where MuJoCo's strikes match RNE's
period by period (1 ms steps, a 4 ms contact time constant, damping ratio
5; `go2-sb3-mujoco.yml` calls it `damped` and compares and traces against
it). The policies with rotor inertia (run 43699237646) from the same 64
starts, MuJoCo run 43793701165 and Genesis run 43793701153; crossed and
falls, and the episodes whose crossings match RNE's:

| Policy, seed | RNE | Genesis | MuJoCo damped | MuJoCo stiff | Same as RNE: Genesis, MuJoCo damped |
|---|---|---|---|---|---|
| Scan 2026 | 485, 0 | 477, 4 | 450, 4 | 476, 4 | 52, 49 |
| Scan 2027 | 484, 0 | 472, 1 | 455, 5 | 419, 9 | 54, 46 |
| Scan 2028 | 407, 0 | 381, 7 | 389, 6 | 299, 22 | 42, 47 |
| Blind 2026 | 169, 9 | 161, 8 | 162, 9 | 174, 8 | 21, 12 |
| Blind 2027 | 211, 11 | 185, 10 | 202, 11 | 203, 6 | 20, 20 |
| Blind 2028 | 159, 7 | 140, 12 | 132, 10 | 142, 7 | 26, 21 |

And the sliding strike against its closed form (normal impulse, N·s;
Genesis takes no friction below 0.01, whose closed form is 0.562):

| Friction | Closed form | RNE | Genesis | MuJoCo damped | MuJoCo stiff |
|---|---|---|---|---|---|
| 0 (Genesis 0.01) | 0.559 | 0.559 | 0.627 | 0.552 | 0.632 |
| 1 | 1.138 | 1.138 | 2.118 | 2.462 | 2.505 |

- **Genesis and MuJoCo damped now land together.** The scan policies keep
  94–98% of their RNE crossings in Genesis and 93–96% in MuJoCo damped, and
  fall 1–7 and 4–6 times; scan 2028, the weak seed, falls 7 and 6 times.
  The blind policies cross 83–96% of RNE's in both.
- **RNE alone gives the strike its rigid impulse.** Genesis, whose rigid
  solver also treats contacts as soft constraints, gives it 1.9 times, as
  MuJoCo does about 2 times. Where a body slides into the floor, then, the
  two references agree with each other and not with the rigid answer, and
  the few falls they keep and RNE does not are, at least in part, theirs.
- **What remains is small.** Against the 16–37 falls per scan policy
  MuJoCo gave before the plant was fixed (rims, velocity limits, implicit
  PD, rotor inertia, randomization), the references keep 1–7, and the
  replays put the disagreements where a head or shin strikes while the
  robot runs; that every remaining fall starts there is not established.

## Limits

- The batch is example 147's task, not a general Go2 environment: one
  reward, a fixed 5 s episode, no velocity command.
- `train_ppo.py` uses SB3's PPO with its own settings, not example 147's
  PPO; its returns compare with example 147's fitness only in that both
  sum the same rewards.
- The smoke runs check the contract and determinism, not learning.
