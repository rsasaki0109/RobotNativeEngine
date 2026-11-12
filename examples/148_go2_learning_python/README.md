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
- with `--curriculum`, gaps widened stage by stage, from 5–10 cm to the
  task's 25–35 cm.

Training runs in stages (`--stages`, or one per curriculum entry). After
each stage the deterministic policy walks one episode in each of
`--eval-envs` held-out environments (seeds 10 000 past training's) on the
task's gaps. `results.json` records, per stage, the mean return and
distance, the falls and the falls at a gap, and the gaps reached and
crossed; a gap counts as crossed once the base is 0.25 m past its far
edge, so the hind feet are across. The `Go2 SB3 PPO` workflow
(`.github/workflows/go2-sb3.yml`) trains a flat-floor job and four gap
jobs (blind or with the scan, with or without the curriculum) and uploads
each job's results.

### Results

The first run (workflow run 43319917776): 10M timesteps per job, 64
environments, seed 2026, on a GitHub runner's 4 cores. These are the last
stage's evaluations, over 64 held-out episodes on the task's 25–35 cm gaps.

| Job | Return | Distance | Gaps reached | Crossed | Falls | Steps/s |
|---|---|---|---|---|---|---|
| Flat floor | 17.55 | 19.1 m | – | – | 64 at the arena edge | 11 186 |
| Gaps, blind | 1.30 | 1.43 m | 64 | 0 | 3 | 6 061 |
| Gaps, scan | 1.08 | 1.43 m | 64 | 0 | 15 | 8 504 |
| Gaps, blind, curriculum | 1.30 | 1.46 m | 64 | 0 | 4 | 7 693 |
| Gaps, scan, curriculum | 1.19 | 1.47 m | 64 | 0 | 6 | 5 228 |

- **Flat floor.** SB3 learns to walk: its policy runs to the arena's edge,
  19 m from the origin, before the 5 s are up. Every flat-floor
  evaluation starts at the origin, so its 64 episodes are one walk. Its
  return beats example 147's PPO (12.25).
- **Gaps.** No policy crossed a gap. Every one reached the first gap ahead
  of it, about 1.5 m from its start, and stopped there, mostly without
  falling. That is the same outcome as example 147's own PPO.
- **Curriculum.** The curriculum did not help. Its narrow-gap stages fell
  at the task's gaps more often, 29–54 of 64.

Since a standard PPO stops where example 147's did, the next question is
whether the task allows a crossing at all: whether a foot that drops into
a 30 cm trench can climb out, and how wide the gap is at the top once the
terrain's 5 cm samples slope its edges.

## Limits

- The batch is example 147's task, not a general Go2 environment: one
  reward, a fixed 5 s episode, no velocity command.
- `train_ppo.py` uses SB3's PPO with its own settings, not example 147's
  PPO; its returns compare with example 147's fitness only in that both
  sum the same rewards.
- The smoke runs check the contract and determinism, not learning.
