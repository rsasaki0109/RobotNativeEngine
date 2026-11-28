# ADR 037: Foresight modules for space-time navigation and manipulation

- Status: Accepted
- Date: 2026-11-28

## Context

`rne_nav` covers what a 2D costmap stack covers: occupancy grids, inflation,
A*, DWA, pure pursuit, and moving obstacles written into the costmap a short
sweep ahead. Three things are outside that model:

1. **Free space that is not a plane.** Drones, multi-level structures, arms.
2. **The future as an input.** A costmap is a snapshot; "the person will be
   here in 3 s" can only be approximated by painting cells. A plan that
   should wait, slow down, or pass behind someone is not representable.
3. **Learned and classical algorithms as peers.** A learned policy is
   usually either the whole stack or a plugin bolted onto a classical one,
   with its own safety story.

The same three gaps appear in manipulation: a hand in the workspace is a
forecast, not an obstacle; a learned motion prior or grasp network should
compete with RRT-Connect and antipodal sampling under the same checks.

There is interest in maintaining navigation and manipulation as separate
repositories (`robot-navigation`, `robot-manipulation`), so the new code
should not be tied to the RNE ECS, physics or renderer.

## Decision

Add a top-level `modules/` directory with three crates:

- `modules/robot-foresight` (`robot_foresight`): `ForecastSet` (multimodal
  predicted agents, uncertainty growing with lookahead, collision
  probability of a sphere at a time), `Forecaster`s (constant velocity,
  learned, blended), the `InferenceModel` boundary to learned models, and
  `HybridPipeline` (proposers, refiners, critics, certifiers; provenance on
  every candidate; a full decision trace).
- `modules/robot-navigation` (`robot_navigation`): 3D voxel maps and
  space-time planning on `robot_foresight`.
- `modules/robot-manipulation` (`robot_manipulation`): forecast-aware arm
  motion (path-velocity decomposition) and grasp planning on
  `robot_foresight`.

Dependency rules (added to `AGENTS.md`):

- `robot_foresight` depends on `glam`, `serde`, `serde_json` and `thiserror`
  only; the other two add `robot_foresight`.
- None of them depends on an RNE crate, a renderer, a physics backend, ROS 2,
  Nav2 or a learning runtime. Learned models are `InferenceModel`
  implementations supplied by the caller (an adapter wraps ONNX or an
  accelerator runtime; `MlpModel` loads exported dense weights).
- RNE crates and examples may depend on the modules, not the reverse.
- `cargo run -p xtask -- lint-boundaries` rejects `rne_*`, adapter, ROS 2,
  Rapier, wgpu, Bevy and MuJoCo dependencies in any `modules/*` manifest.

All three are `publish = false` and outside the Rust API baseline until they
have settled.

Safety rule built into the pipeline: certifiers are the only gate to
execution, they see every candidate regardless of provenance, and a proposer
that errors is recorded and skipped, so a classical proposer is always a
fallback for a learned one.

## Splitting into repositories

Each module is a directory with its own `Cargo.toml`, README, sources and
tests, so `git subtree split --prefix modules/robot-navigation` yields its
history. A split crate replaces `*.workspace = true` with literal values
and depends on `robot_foresight` by git or registry. `robot_foresight`
either becomes a third repository or is vendored into the other two. The
split itself needs new repositories and is left to the maintainer.

## Alternatives considered

- **Extend `rne_nav` to 3D and time.** `rne_nav` is 2D throughout
  (`Pose2d`, grids, DWA) and ECS-integrated; adding a time axis would touch
  every type, and the result could not leave the workspace.
- **Put the hybrid pipeline in `rne_ai`.** `rne_ai` depends on Rapier,
  rendering and the URDF importer; the pipeline must stay small enough to
  run on a robot without the simulator.
- **One crate for everything.** Navigation and manipulation are meant to be
  separate repositories; the shared core is what both need.

## Consequences

- A fourth top-level source directory (`modules/`) next to `crates/`,
  `adapters/` and `examples/`.
- Forecast-aware planning is available to RNE examples and to code outside
  RNE alike.
- The collision-probability model is deliberately simple (Gaussian position
  error, `Phi((R - d) / sigma)` per mode, independent agents); a richer model
  implements `WorldForecast` without changing the planners.
