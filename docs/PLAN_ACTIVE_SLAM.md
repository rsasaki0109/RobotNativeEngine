# Active SLAM plan

Status: draft (2026-11-26). Target: a robot that is given **no map and no
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
