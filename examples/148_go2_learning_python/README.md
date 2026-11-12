# 148 · Example 147's Go2 walking task from Python

`rne_py.Go2LearningBatch` is `rne_locomotion::go2_learning` as a native
parallel batch: the Go2 of example 147 with every URDF collision shape
touching the ground, on a flat floor or on a terrain with fractal relief and
gaps, observing its joints and body (44 values) and optionally a 15-ray
height scan (59). One Python call steps every environment on the available
cores.

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

The TaskSpec, `rne.unitree_go2.learning.v1`, gives the same spaces in
machine-readable form (`task_spec_json()`).

## Run

```bash
.venv/bin/maturin develop -m crates/rne_py/Cargo.toml --release
.venv/bin/python examples/148_go2_learning_python/run.py --smoke
.venv/bin/python examples/148_go2_learning_python/run.py --floor gaps --height-scan
.venv/bin/python examples/148_go2_learning_python/train_ppo.py --smoke
.venv/bin/python examples/148_go2_learning_python/train_ppo.py \
  --floor gaps --height-scan --timesteps 5000000 --n-envs 16
```

`run.py` needs only `rne_py`. `train_ppo.py` uses Stable-Baselines3's PPO
through `go2_vec_env.NativeGo2VecEnv`, a `VecEnv` over the batch.

## Limits

- The batch is example 147's task, not a general Go2 environment: one
  reward, a fixed 5 s episode, no velocity command.
- `train_ppo.py` uses SB3's default PPO settings, not example 147's PPO;
  its results are not comparable with example 147's.
- The smoke runs check the contract and determinism, not learning.
