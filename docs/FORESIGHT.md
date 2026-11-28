# Foresight: navigation and manipulation against the future

`modules/robot-foresight`, `modules/robot-navigation` and
`modules/robot-manipulation` are standalone Rust crates for planning in a
world that moves. They depend on no RNE crate ([ADR 037](adr/037-foresight-modules.md))
and are meant to become their own repositories.

| | 2D costmap stack (`rne_nav`, Nav2-style) | foresight modules |
|---|---|---|
| Free space | plane (occupancy grid) | 3D voxels; joint space for arms |
| Moving obstacles | written into the costmap now (optionally swept ahead) | `ForecastSet`: multimodal futures, uncertainty growing with lookahead |
| Plan | path, then a controller tracks it | timed trajectory: waiting and slowing are part of the plan |
| Learned components | replace a stage or the stack | peers of classical stages in one `HybridPipeline`, behind the same certifiers |
| Why a plan was chosen | implicit | `Decision` trace: every candidate, its provenance, cost terms and verdict |

## The world as a function of time

```rust
use robot_foresight::{BlendedForecaster, ConstantVelocityForecaster, Forecaster,
                      LearnedForecaster, WorldForecast};

let cv = ConstantVelocityForecaster::default();              // classical
let learned = LearnedForecaster::new(my_model, 2, 2)?;       // any InferenceModel
let forecaster = BlendedForecaster { primary: learned, secondary: cv, secondary_weight: 0.3 };
let forecast = forecaster.forecast(&tracks, now_s, 10.0)?;
let p_hit = forecast.collision_probability(point_m, robot_radius_m, now_s + 3.0);
```

Each agent has weighted modes (timed states, interpolated, extrapolated with
the last velocity) and a position standard deviation
`sigma0 + growth * lookahead`. Per mode, a sphere at distance `d` with
combined radius `R` is hit with probability `Phi((R - d) / sigma)`; modes are
weighted, agents combine as independent events. The blend keeps the
physically plausible constant-velocity future even when a model is
confidently wrong.

## Learned and classical as peers

```rust
let pipeline = HybridPipeline::new(seed)
    .with_policy(SelectionPolicy::LowestCost)        // or FirstCertified: priority list
    .with_proposer(LearnedVelocityProposer::new(policy)?)   // learned
    .with_proposer(SpaceTimeAstar::default())               // classical
    .with_refiner(ShortcutRefiner::default())
    .with_critic(1.0, DurationCritic)
    .with_critic(0.5, LearnedCostCritic::new(comfort_model)?)
    .with_certifier(ForecastCertifier::default());
let decision = pipeline.decide(&problem);
println!("{}", decision.summary());
```

- Every candidate is refined, priced and certified by the same stages and
  carries a `Provenance` (`learned:policy_v3+classical:spacetime_shortcut`).
- A proposer error (a model producing NaN, a search out of budget) is
  recorded in `proposer_failures`; the others still run.
- Randomness comes from `SplitMix64` seeded from the pipeline seed and the
  stage index: decisions replay bit-for-bit, and adding a stage does not
  change the others' samples.
- Models enter only through `InferenceModel` (fixed-size `f32` in and out,
  checked for shape and finiteness). `MlpModel` loads dense networks from
  JSON; `FnModel` wraps a closure; ONNX or accelerator runtimes belong in an
  adapter.

## `robot_navigation`

- `VoxelGrid`: dense 3D occupancy; space outside the grid is occupied.
- `SpaceTimeAstar`: A* over (voxel, continuous time) with 26-connected moves
  at the speed limits plus a wait action; rejects states whose forecast risk
  exceeds the threshold; weighted heuristic (default 1.5) bounds suboptimality.
- `StraightLineProposer` (several speeds), `LearnedVelocityProposer`
  (rolls out a 12 → 3 velocity policy, deterministic plus noisy samples).
- `ShortcutRefiner`: space-time shortcuts and retiming that keep the plan
  certified.
- `ForecastCertifier`: start state, speed limits, static clearance, forecast
  risk over the trajectory and a hold period after arrival, goal reached.

- `planar`: the classical 2D stack ported from `rne_nav` (occupancy grid,
  inflated costmap, A* `plan_path`, pure pursuit, DWA), for robots that only
  need a plane.

Perception enters through `robot_foresight::ObstacleTracker` (ported from
`rne_nav`): detections in, timestamped `AgentTrack`s out, ready for any
forecaster.

## `robot_manipulation`

- `SerialChain`: DH revolute chain, link collision spheres, damped
  least-squares position IK. Self-collision is not modelled.
- `time_scale_path`: path-velocity decomposition. Any joint path is
  resampled and retimed by a breadth-first search over (path sample, time
  step); swept samples must be safe at the start, middle and end of the
  step. With nothing moving it is the path at full speed.
- Proposers: `DirectJointProposer`, `RrtConnectProposer` (seeded, shortcut),
  `LearnedViaPointProposer` (a `2·dof + 3 → dof` motion prior); all retimed.
- `MotionCertifier`: limits, static clearance and forecast risk sampled over
  time.
- Grasps: `AntipodalGraspSampler` (friction-cone pairs), `AntipodalQualityCritic`,
  `LearnedGraspScorer` (6 features → success logit), `ReachabilityCertifier`
  (IK to pre-grasp and grasp).

## Examples

- [Example 165](../examples/165_foresight_drone_warehouse/main.rs): a drone
  in a 20 × 12 × 3 m warehouse aisle, a forklift head-on and a worker about
  to cross. Planned on a snapshot, the drone hits the forklift (−0.65 m);
  on a constant-velocity forecast it passes the forklift on the worker's
  side and hits the worker (−0.65 m); on the blended learned + constant-velocity
  forecast it passes on the other side, 0.50 m clear of both.
- [Example 166](../examples/166_foresight_arm_shared_workspace/main.rs): an
  antipodal grasp is chosen by friction and a learned scorer and certified
  by IK; the swing across the table must pass where a hand rests until
  t = 2 s. The classical direct sweep waits 1.8 s and arrives at 3.00 s; the
  learned via-point route lifts the elbow over the hand, certifies, and
  arrives at 1.40 s.

The models in both examples are labelled stand-ins (`*_standin_v0`): small
closures with the shapes a trained network would have. The numbers above
show the plumbing and the safety gate, not the quality of a trained model.

## Limits

- Holonomic point-sphere robot in `robot_navigation`; no dynamics or
  acceleration limits yet. A* moves change speed instantly.
- The collision-probability model assumes isotropic Gaussian error and
  independent agents.
- Plans are one-shot; receding-horizon replanning is the caller's loop
  (forecast, `decide`, execute the first part, repeat).
