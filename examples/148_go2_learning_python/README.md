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

## Limits

- The batch is example 147's task, not a general Go2 environment: one
  reward, a fixed 5 s episode, no velocity command.
- `train_ppo.py` uses SB3's PPO with its own settings, not example 147's
  PPO; its returns compare with example 147's fitness only in that both
  sum the same rewards.
- The smoke runs check the contract and determinism, not learning.
