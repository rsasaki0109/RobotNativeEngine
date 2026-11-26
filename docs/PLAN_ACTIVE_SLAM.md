# Active SLAM plan

Status: P0 and P1 done (2026-11-26); P2 next. Target: a robot that is given **no map and no
goals** decides where to go so that the map gets complete and stays
accurate, in a headless deterministic test and a rendered demo.

## What exists and what is missing

RNE already has everything active SLAM drives, except the decision:

| Needed | Already in RNE |
| --- | --- |
| Online SLAM with loop closure | `rne_slam::Slam2d` (scan matching on a likelihood field, `PoseGraph` with loop-closure edges and diagonal information, robust optimization); `rne_slam` 3D LIO and `pose_graph3d` |
| Map with unknown space | `rne_nav::OccupancyGrid` (`-1` unknown, `0` free, `100` occupied) |
| Planning and control | `rne_nav::plan_path` (with `allow_unknown`), costmap layers, DWA, pure pursuit, `drive` |
| Navigating with no map given | example 131: Go2 + Mid-360, `Slam2d` online, A* through unknown space to **hand-given** goals |
| Camera world | example 163 (3DGS street), visloc-rs localization as a subprocess ([VISLOC_NAVIGATION_PLAN.md](VISLOC_NAVIGATION_PLAN.md)) |
| Determinism, replay | `SimClock`, `WorldRandom`, replay hash tests, `xtask ci-headless` |

Missing: **where to go next**. Example 131 still lists its goals by hand.
No frontier extraction, information-gain scoring or loop-closure-seeking
behaviour exists in the repository.

## Boundary

- New crate **`crates/rne_explore`**: exploration decisions only. It reads an
  `OccupancyGrid`, the robot's pose and a pose graph, and returns ranked goal
  poses (or "done"). It does not plan paths, drive or estimate:
  `rne_nav` plans and drives, `rne_slam` estimates.
- Allowed dependencies: `rne_nav`, `rne_slam`, `rne_core`, `rne_ecs`,
  `rne_math`. Not a renderer, physics backend, ROS 2 or visloc-rs. See
  [ADR 036](adr/036-active-slam-crate.md).
- Kept separate from `rne_slam` (estimation and decision have different
  lifecycles and tests) and from `rne_nav` (`rne_nav` must not gain a
  dependency on `rne_slam`).
- Camera-based belief (visloc-rs) enters later through an example crate,
  as in [VISLOC_NAVIGATION_PLAN.md](VISLOC_NAVIGATION_PLAN.md), never through
  `rne_explore`'s dependencies. `rne_explore` sees it only through a trait.
- `publish = false` until the API settles; then it joins
  `PUBLIC_RELEASE_PACKAGES` and the API baseline.

## Design of `rne_explore`

Plain functions over `rne_nav` / `rne_slam` types first, ECS resources and
systems (`resources.rs`, `systems.rs`) once an example needs them.

1. **Frontiers.** Free cells with an unknown 4-neighbour, grouped by
   8-connected flood fill in row-major order (deterministic). Clusters below
   `min_frontier_cells` are dropped. Each cluster yields a goal: the free
   cell nearest its centroid that the inflated costmap allows.
2. **Information gain.** Unknown cells a sensor of range `r` and field of
   view `fov` would see from the goal, by ray casting on the grid. The
   sensor model is a parameter so lidar (360°) and camera (90°) share it.
3. **Cost.** Path length from `plan_path` (not Euclidean, so goals behind
   walls are not favoured), plus a turn term.
4. **Uncertainty (P2).** The pose graph's D-optimality, approximated by the
   weighted tree connectivity `log det` of the reduced weighted Laplacian
   built from `PoseGraphEdge::information`. A *revisit* candidate is a past
   keyframe where a loop closure is likely; its utility is the expected
   `log det` gain of adding that edge. When the gain per metre beats the
   best frontier's, the robot closes the loop first.
5. **Selection.** `utility = gain - lambda * cost` (P1), plus
   `mu * uncertainty_gain` (P2). Ties broken by cell index. Hysteresis: keep
   the current goal until another beats it by `switch_margin`.
6. **Termination.** No reachable frontier cluster left, or a distance or
   time budget spent.
7. **Belief trait (P4).** `trait Belief { pose_covariance(); localizability(pose) }`
   so a camera localizer can say where it would fail to localize (bare
   walls) without `rne_explore` depending on it.

## Phases

Each phase ends with a headless test and a short docs entry.

### P0: frontiers on a fixed grid

- `rne_explore` with frontier extraction, gain, cost and selection on a
  given `OccupancyGrid`.
- Exit: unit tests on hand-made grids (one room, two rooms joined by a
  door, a dead end); the result is identical across runs and platforms.
- Done: `crates/rne_explore` with 12 unit tests. On a room with a dead end
  next to the robot and a doorway onto a hall far from it, nearest-frontier
  picks the dead end and the information-gain policy the hall; hysteresis
  keeps a goal until it is cleared; a closed room is `Done`. Not yet: ECS
  resources and systems (with P1's example), the pose-graph term (P2).

### P1: exploration with lidar SLAM (diff drive)

- New example: a diff-drive robot with a 2D lidar in the warehouse of
  example 127 (`warehouse.rs`: walls, two racks, a column, the office block,
  and pallets in their first day's slots, held fixed), `Slam2d` online, `rne_explore` choosing goals,
  `rne_nav` planning and driving. Ground truth only for scoring.
- Baseline: nearest-frontier (Yamauchi 1997) with the same code, `lambda`
  large and `gain` ignored.
- Metrics: explored free area against distance and time (coverage curve),
  map agreement with the true grid (cell accuracy), ATE of the SLAM
  trajectory, collisions, replay hash.
- Exit: full coverage of the scene with no collisions; the gain-weighted
  policy reaches 95% coverage in less distance than nearest-frontier; two
  runs give the same world hash.
- Done, with one exit criterion not met: example 164
  (`active_slam_explore`), below.

#### P1 result

![Two robots explore the same warehouse from the same start, top-down. Left: nearest frontier; right: information gain. The SLAM map grows from dark unknown to light free floor with black walls; orange frontier cells, a pink goal and a blue path show where each robot is heading; the green trail is the true path, and thin grey outlines are the true walls, racks and pallets. Both finish with the floor mapped; the left map sits visibly off the true outlines, the right one on them.](media/active-slam-explore.gif)

```bash
cargo run --release -p active_slam_explore --example 164_active_slam_explore
```

A diff-drive robot (0.25 m radius, 0.6 m/s) with a 360-beam 4 m LiDAR at
0.5 m and odometry with a 2 % scale error and a yaw bias runs `Slam2d` on
keyframes (0.3 m / 0.3 rad) and steers on the SLAM estimate. Every second
`Explorer::decide` picks a frontier; `Explorer::path_to` gives the path; a
heading-first follower drives it. On a bump (none happened) it backs off
and chooses afresh. Ground truth only scores: coverage is the share of the
true free floor, sampled every 10 cm, that the map knows; SLAM error is the
mean keyframe position error in the map frame, without alignment.

Four starts, one per aisle (start 0 is the GIF):

| start | policy | time | driven | mapped | driven to 90 % | SLAM error |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 0 (-5.0, -2.0) | nearest frontier | 72 s | 35.8 m | 93.0 % | 20.8 m | 0.239 m |
| | information gain | 70 s | 38.2 m | 96.1 % | 18.7 m | 0.113 m |
| 1 (5.0, -2.0) | nearest frontier | 72 s | 36.3 m | 95.7 % | 22.5 m | 0.142 m |
| | information gain | 65 s | 34.9 m | 96.3 % | 25.6 m | 0.070 m |
| 2 (-5.0, 1.9) | nearest frontier | 66 s | 31.2 m | 93.6 % | 21.5 m | 0.125 m |
| | information gain | 54 s | 28.5 m | 96.3 % | 19.2 m | 0.069 m |
| 3 (3.0, 1.5) | nearest frontier | 58 s | 28.1 m | 95.4 % | 23.0 m | 0.181 m |
| | information gain | 62 s | 30.5 m | 96.5 % | 25.7 m | 0.053 m |
| **mean** | nearest frontier | 67 s | 32.8 m | 94.4 % | 21.9 m | 0.172 m |
| | **information gain** | 63 s | 33.0 m | **96.3 %** | 22.3 m | **0.076 m** |

No run collided, and every run ended with `Decision::Done`.

- **Information gain maps more and keeps SLAM tighter, at the same cost.**
  It finishes in the same time and distance and leaves less of the floor
  unmapped on every start. Its SLAM error is lower on all four starts here,
  less than half on average; on the four held-out starts below it is lower
  on two and slightly higher on two (0.077 against 0.072 m, 0.150 against
  0.138 m), and less than half on average again. It sets fewer goals (158
  against 174 over the four starts); a likely reason for the tighter SLAM,
  not measured, is fewer short hops and turns between small frontiers. It
  was not designed for SLAM accuracy; P2 adds that on purpose.
- **Not met: reaching 90 % (or 95 %) coverage in less driving.** The mean
  distance to 90 % is 22.3 m against 21.9 m; start by start it wins two and
  loses two. In a warehouse this small a 4 m LiDAR sees most of an aisle
  from anywhere in it, so the order of the frontiers matters little.
- **The distance weight was chosen on these starts.** A sweep on starts
  0–3 (with a 24 m grid): weight 0.1 / 0.25 / 0.5 / 1 / 2 / 4 drove
  46.9 / 46.1 / 41.6 / 38.9 / 34.5 / 34.6 m in all; small weights chase
  far, large frontiers and drive much more. Weight 4 was then checked on
  four starts not used to choose it ((0, -2), (-2.3, 2), (6, 0),
  (-6.2, -3.9)): 54 s, 28.7 m, 96.0 % mapped, 20.6 m to 90 %, SLAM error
  0.078 m, against 56 s, 26.8 m, 95.2 %, 21.7 m and 0.148 m for nearest
  frontier.
- **Results move with small changes.** Changing only the grid from 24 m to
  32 m moved start 0's nearest-frontier run from 94.0 % to 93.0 % mapped and
  its SLAM error from 0.201 m to 0.239 m. Read the differences in coverage
  speed above as no difference.

What was found and fixed on the way:

- A robot whose estimate drifted into the inscribed band near a wall found
  nothing reachable and stopped exploring: `escape_radius_m`.
- Frontiers chosen by the explorer had no path in `rne_nav::plan_path`,
  which treats inscribed cells differently: `Explorer::path_to`.
- LiDAR bins left empty by the binning of returns were cleared as free
  space through walls, which mapped the outside of the warehouse (and
  inflated coverage by about 1 %): such bins are now skipped.
- `rne_nav::Costmap::cost_at` panicked one cell outside the map.

The gates in `--smoke` and the full run: done, no collision, coverage at
least 92 % (the lowest measured, over all eight starts, is 93.0 %; the
strip between the north pallets and the wall is narrower than the robot
with its clearance, and the LiDAR cannot see over the pallets), SLAM error
under 0.3 m, and a replay with the same hash.

### P2: uncertainty-aware exploration (active loop closure)

- Pose-graph D-optimality and revisit candidates (design item 4).
- The same warehouse as P1: the aisles around the racks, the column and the
  pallet rows form loops, so pure frontier exploration accumulates drift
  before it comes back. Using one scene for P1 and P2 makes the two policies
  directly comparable.
- Exit: lower ATE and map error than P1 at a comparable coverage; the
  distance spent on revisits is reported.

### P3: legged robot, no goals given

- Example 131's Go2 + Mid-360, with `rne_explore` instead of the hand-given
  goals.
- Exit: both rooms explored; localization error in the range of 131's
  (0.042 m RMS).

### P4: camera-based belief (with visloc-rs)

- An example crate that depends on visloc-rs (VISLOC_NAVIGATION_PLAN's
  pattern), implementing `Belief` from the visloc-rs localizer: pose
  covariance, and localizability from the landmarks a camera at a candidate
  pose would see.
- visloc-rs side, small: the long-lived localizer server planned in
  visloc-rs `docs/rne_migration.md` P2, extended to return the pose
  covariance (`visloc_slam::covariance`) and a periodic map snapshot, so
  candidates are scored inside RNE without a round trip per candidate.
- Scene: an indoor 3DGS interior, then the street of example 163.
- Exit: exploration that avoids routes where the camera cannot localize
  (bare walls), with fewer tracking losses than P1's policy on the same
  scene.

## Decisions

- P1 and P2 scene: the warehouse of example 127 (2026-11-26).

## Open questions

- Whether frontier extraction belongs in `rne_nav` (it is a pure grid
  operation) with only the scoring in `rne_explore`.
- Multi-robot exploration (`rne_nav::coordination`, `map_merge`): later, not
  in this plan.
