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

#### Generated worlds and the benchmark

One warehouse and four starts is too little to compare policies, and the
warehouse is too small for a 4 m LiDAR to make the order of frontiers
matter. Example 164 also builds three kinds of building from a seed
(`worlds.rs`, `WorldRandom`; a seed always gives the same world):

- **office**: a 2.2 m corridor with 3–5 rooms on each side, each with a
  1.4 m door, some joined to a neighbour, furniture against the outer wall;
- **loop**: a 20–26 m by 13–17 m ring corridor, 2.2–2.8 m wide, round a
  block with dead-end notches cut into it;
- **maze**: 6–8 by 4–5 cells of 2.6 m carved by a randomized depth-first
  search, with about 12 % of the remaining walls removed so it has loops.

Walls get small bumps (0.24–0.7 m wide, 0.12–0.3 m deep, 1.2–2.5 m apart)
from a separate random stream. Without them `Slam2d` lost the distance
along bare corridors: on seed 0 the maze's SLAM error was 1.0 m (nearest
frontier) and 0.86 m (information gain); with them 0.27 m and 0.16 m.
Tests check, on 20 seeds of each kind, that every start and every room or
cell centre is reachable with the robot's clearance, and that a seed gives
the same world.

```bash
cargo run --release -p active_slam_explore --example 164_active_slam_explore -- --world maze --seed 3
cargo run --release -p active_slam_explore --example 164_active_slam_explore -- --bench 10
```

`--bench 10`, both policies from each world's first start, seeds 0–9
(information gain at distance weight 4, chosen on the warehouse):

| world | policy | time | driven | mapped | driven to 90 % (runs reaching it) | SLAM error | IG better (distance / mapped / SLAM) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| office | nearest frontier | 77 s | 37.2 m | 95.7 % | 30.5 m (10) | 0.131 m | |
| | information gain | 85 s | 42.3 m | 95.5 % | 29.8 m (10) | 0.127 m | 3 / 4 / 5 of 10 |
| loop | nearest frontier | 107 s | 61.2 m | 85.3 % | 50.4 m (2) | 0.696 m | |
| | information gain | 115 s | 65.4 m | 82.9 % | 43.4 m (1) | 0.944 m | 4 / 4 / 2 of 10 |
| maze | nearest frontier | 113 s | 62.2 m | 96.6 % | 54.3 m (10) | 0.228 m | |
| | information gain | 116 s | 64.6 m | 97.1 % | 49.5 m (10) | 0.195 m | 4 / 7 / 7 of 10 |

All 60 runs ended with `Decision::Done`, none collided.

- **The warehouse's halved SLAM error does not generalize.** In the office
  the two policies are even on SLAM error (5 of 10) and coverage, and
  information gain drives 14 % more. In the maze information gain is ahead
  on coverage and SLAM error on 7 of 10 seeds and reaches 90 % about 5 m
  sooner, but drives more in total. On the loop it is worse.
- **The loop breaks SLAM, whichever policy.** SLAM error is 0.3–1.8 m,
  coverage (of the true floor, in the map frame) 72–96 %, and only 3 of 20
  runs reach 90 %: the heading error built up turning the corners is not
  corrected, and `Slam2d`'s loop closure rarely fires when the robot comes
  round (no closure at all in 15 of the 20 runs). That is the failure P2 is for:
  exploration that knows its uncertainty and goes back to close the loop.
- The distance weight 4 was chosen on the warehouse; it was not tuned on
  these worlds.

### P2: uncertainty-aware exploration (active loop closure)

- Pose-graph D-optimality and revisit candidates (design item 4).
- The generated **loop** worlds (P1's benchmark above), where both P1
  policies lose the map: SLAM error 0.7–0.9 m on average. The warehouse
  stays as a check that nothing regresses where SLAM already holds.
- Exit: on `--bench` loop seeds, lower SLAM error and higher coverage than
  both P1 policies, with no loss on the office and maze seeds; the distance
  spent on revisits is reported.

#### P2 result

Configuration: example 164 with `--rebuild-map --submap-loops --revisit`
(map rebuild, verified submap loop closure, longest-loop revisits; all
opt-in, so P1's numbers and the CI replay hash `afcbe980c2c754a1` are
unchanged). Means over 10 seeds (warehouse: 4 starts), against P1:

| world | policy | SLAM err P1 → P2 | mapped P1 → P2 | distance P1 → P2 | closures (bad) |
| --- | --- | --- | --- | --- | --- |
| loop | nearest | 0.696 → 0.667 m | 85.3 → 86.4 % | 61.2 → 87.2 m | 396 (0), both policies |
| loop | information gain | 0.944 → 0.836 m | 82.9 → 83.3 % | 65.4 → 90.8 m | |
| office | nearest | 0.131 → 0.113 m | 95.7 → 96.0 % | 37.2 → 43.5 m | 368 (3) |
| office | information gain | 0.127 → 0.119 m | 95.5 → 95.4 % | 42.3 → 46.9 m | |
| maze | nearest | 0.228 → 0.227 m | 96.6 → 96.6 % | 62.2 → 72.6 m | 416 (2) |
| maze | information gain | 0.195 → 0.211 m | 97.1 → 96.9 % | 64.6 → 79.1 m | |
| warehouse | nearest | 0.172 → 0.129 m | 94.4 → 95.3 % | 32.8 → 35.6 m | 72 (0) |
| warehouse | information gain | 0.076 → 0.065 m | 96.3 → 96.3 % | 33.0 → 37.5 m | |

- On the loop seeds, P2 lowers SLAM error on 7 (nearest) and 8
  (information gain) of 10 seeds and raises coverage on 7 and 6. With
  nearest frontier it beats both P1 policies on error and coverage, so the
  exit is **met for nearest frontier, narrowly**: the mean gain is 4 % in
  error and 1 point in coverage. Information gain improves on its own P1
  (11 % lower error) but stays behind nearest frontier.
- "No loss on office and maze" holds for nearest frontier; information
  gain loses a little on the maze (0.195 → 0.211 m, 97.1 → 96.9 %) and
  0.1 point of office coverage.
- The price is driving: revisits cost 13–15 m per loop run (40 % more
  driving in total), 3 m in the office, 8–10 m in the maze.
- Wrong loop closures, the failure that blocked P2 at first, are down from
  14–43 % to 5 of 1,252 (0.4 %). Without revisits, the loop worlds rarely
  bring the robot back near an old pose (58 closures in 20 runs, 4 bad),
  and the means stay at P1's.

What landed:

- `Slam2d::close_loops_against_submaps(SubmapLoopConfig)`: a closure
  candidate is an earlier keyframe within 2.5 m of the estimate and at
  least 8 m back along the trajectory. The current scan is matched, with a
  1 m window, against a submap of only the keyframes around the candidate;
  the default closure matches against the whole map, which already holds
  the robot's own recent scans, so it mostly confirmed the drifted
  estimate and could not see drift beyond its 0.3 m window. A match is
  rejected when it is degenerate (weaker score curvature under 0.1 of the
  stronger: a corridor), when its correction exceeds 0.3 m plus 5 % of the
  path since the candidate, or when the next keyframes do not confirm it
  (two closures in a row agreeing within 0.15 m). After an accepted
  closure, none is searched for over the next 2 m; corrections under 5 cm
  are added without re-optimizing.
- `Slam2d::rebuild_map_on_loop_closure()`: after a closure that moved the
  graph, re-integrate every keyframe scan at its optimized pose and move
  the live pose to the optimized node. Before it, a closure changed the
  pose graph and nothing else.
- `PoseGraph` factors only each row's envelope in its Cholesky solve. The
  skipped entries are exact zeros, so results are unchanged (examples 127,
  131, 132 and 164 print identical output), but a chain with a few loops
  no longer costs the cube of its length: with frequent closures the
  dense solve had made one loop run take over 20 minutes.
- `Explorer::with_revisits(RevisitConfig)`: once the robot has driven
  `open_loop_m` (15 m) without a closure, it goes back to a pose of its
  trajectory at least `min_loop_m` (6 m) back and within `max_detour_m`
  (15 m) by path, choosing the one that closes the **longest loop for the
  detour** (trajectory length between it and the robot, minus the path
  there). After `max_tries` (3) revisits without a closure it stops
  revisiting until one closes.
- Example 164: flags `--rebuild-map` (raises the default closure's score
  to 0.75), `--submap-loops`, `--loop-min-score`, `--revisit`; reports
  bad loop closures against ground truth (relative pose off by more than
  0.25 m) and why submap closures were rejected.

How it got there (all on the benchmark worlds):

- **Many of `Slam2d`'s loop closures are wrong**, even in the warehouse, at
  the default `loop_min_score` 0.55: on the smoke start, 6 of 12 (nearest
  frontier) and 5 of 26 (information gain). P1 got away with
  this only because closures changed nothing.
- **Rebuilding at score 0.55 is catastrophic.** It spreads the wrong
  closures over the map, so the next scans match against a broken map and
  more closures go wrong: 19 of 31 and 21 of 36 were bad, and SLAM error was
  1.20 m and 13.9 m.
- **At score 0.75 the warehouse is clean**, and rebuilding helps: 0 bad of
  7 and of 20, SLAM error 0.239 → 0.180 m and 0.113 → 0.082 m, and coverage
  93.0 → 95.0 % and 96.1 % (unchanged). Without rebuild, 0.75 still
  accepted 3 bad of 7 and 0 of 20, and 0.85 no fewer (3 of 6, 0 of 18):
  the rebuilt runs, which drive differently, happened to accept none, so
  this is one start, not proof that 0.75 is safe.
  Over all four warehouse starts with rebuild at 0.75, 1 of 36 closures was
  bad, and the means improve on P1: nearest frontier SLAM error 0.172 →
  0.124 m and coverage 94.4 → 95.4 %, information gain 0.076 → 0.067 m at
  96.3 % (unchanged), with the same driving distance.
- **Office and maze, seeds 0–2, rebuild at 0.75:** about neutral. Most runs
  are identical to P1 (no closure accepted); office-1 nearest frontier
  improves (0.268 → 0.242 m), office-1 information gain gets worse
  (0.280 → 0.323 m, coverage 92.2 → 91.5 %), and 4 of 88 closures were bad.
- **The loop worlds, 10 seeds, rebuild at 0.75:**

  | | nearest SLAM err | nearest mapped | IG SLAM err | IG mapped | distance (nearest / IG) |
  | --- | --- | --- | --- | --- | --- |
  | P1 | 0.697 m | 85.3 % | 0.946 m | 83.1 % | 61.2 / 65.8 m |
  | rebuild | 0.697 m | 85.3 % | 0.946 m | 83.1 % | 61.2 / 65.8 m |
  | rebuild + revisit | 0.766 m | 85.8 % | 0.852 m | 83.5 % | 103.6 / 98.7 m |

  Without revisits, closures happen in only 4 of 20 runs (56 closures, 24
  bad), and the means equal P1's to the printed digit. Revisits make
  closures happen in all 20 runs (1,523 closures, 218 bad) at about 20 m of
  revisit driving per run (40–60 % more driving in total), and they help
  where the closures are right: loop-1 nearest 0.311 → 0.229 m, loop-5
  IG 0.972 → 0.601 m. But three runs still accept 40 % bad closures
  (123/48, 120/50, 114/47) and get worse, and on average nearest frontier
  loses (0.697 → 0.766 m) while information gain gains (0.946 → 0.852 m).
  The exit (lower error *and* higher coverage than both P1 policies) is not
  met.
- **Verified submap closures, revisits to the nearest old place:** 0 bad
  closures, but SLAM error barely moved, and wider windows (2 m) or search
  radii (4 m) did not change that. The revisits went back down the
  corridor just driven, closing short loops that correct little. Scoring
  them by loop length with a 15 m detour helped nearest frontier on loop
  seeds 0–2 (0.741 → 0.633 m) but not information gain (0.723 →
  0.758 m); the 10-seed result above is with it.
- The degeneracy threshold was swept on loop seeds 0–2 (mean of both
  policies): 0.2, the covariance model's own, rejected most corridor
  matches (0.804 m); 0.1 gave 0.732 m; 0.05 and 0.02 were identical to
  each other (0.760 m). No setting accepted a bad closure.
- Without a cap, a policy whose revisits never close a loop kept
  revisiting: 190 revisits and 625 m on one run.

Open after P2:

- The gains on the loop worlds are small next to the seed-to-seed spread
  (SLAM error 0.16–1.28 m per run). Where the remaining error comes from
  was not measured; P1 put it on heading error built up at the corners,
  which only a closure around the whole ring removes. D-optimality (design item 4), choosing revisits by the
  pose-graph uncertainty they remove, was tried next; see below.
- The default `Slam2d` closure (whole map, score 0.55) is unchanged; its
  wrong closures are harmless only because it does not rebuild the map.
  Whether submap closure should become the default is a separate
  `rne_slam` decision.

#### D-optimal revisits (tried after P3; no measurable effect)

`Explorer::decide_with_graph` (example 164 `--dopt`) scores a revisit by the
pose graph's effective resistance between the place and the robot, with
odometry steps as resistors of their length and loop closures as 0.5 m,
instead of by the trajectory length between them. Closing a loop multiplies
the graph's weighted spanning trees, its D-optimality measure, by `1 + R/r`,
so the largest resistance is the most informative closure; a place already
tied to the robot by earlier closures counts as close. With no closures the
two scores agree.

On the 10 loop seeds (`--rebuild-map --submap-loops --revisit`, same
binary):

| policy | SLAM error | mapped | runs with lower error |
| --- | --- | --- | --- |
| nearest frontier, trajectory length | 0.667 m | 86.4 % | — |
| nearest frontier, `--dopt` | 0.648 m | 86.8 % | 3 of 10 |
| information gain, trajectory length | 0.836 m | 83.3 % | — |
| information gain, `--dopt` | 0.851 m | 83.2 % | 3 of 10 |

Most runs are identical: most revisits are chosen before any closure
exists, when the two scores agree, and on a ring the best loop to close
stays the same afterwards. `--dopt` stays opt-in.

What does limit the loop worlds was then measured (seeds 0–4, both
policies, per revisit and per third of the trajectory):

- Revisits work: 33 of 41 ended because a loop closed (a confirmed pair
  of closures each); 8 were cut short without one. No closure was wrong.
  The 3 revisits most runs show are not 3 failed tries: going round a ring
  produces about that many.
- The error sits at the end of the trajectory. Mean position error over
  the first, middle and last third of the keyframes: 0.11–0.34 m,
  0.28–1.36 m, 0.62–2.09 m (heading at the end off by up to 0.26 rad).
- Going back once more to close the last open loop before reporting done
  (tried as `RevisitConfig::close_before_done`) made it worse: SLAM error
  0.667 → 0.705 m and 0.836 → 0.921 m, no run better, 3–5 m more driving.
  It was reverted.

So closures happen and are right, yet the trajectory between and after
them stays off by tens of centimetres: the question has moved from the
exploration policy to how `Slam2d` applies a closure.

Measured further, per closure (seeds 0–1):

- No closure lowered the mean position error of the graph by more than a
  few millimetres (e.g. 0.105 → 0.105 m, 0.549 → 0.547 m).
- Most closures are short: 35–60 keyframes back (8 m), where the relative
  drift is small, so they correct little. The closure that matters, around
  the whole ring (370–430 keyframes), corrected about 1.4 m and 0.14 rad.
- Its optimization was not converged: after the 8 Gauss-Newton iterations
  `SubmapLoopConfig` uses, 100 more moved the nodes another 0.5 m. Running
  100 iterations, though, did not help either (SLAM error 0.667 → 0.681 m
  and 0.836 → 0.813 m over 10 seeds, about even per seed), and one run's
  optimization diverged. It was not adopted.

Even converged, then, closing the ring does not bring the trajectory
closer to the truth. What the graph holds is the next suspect: every edge,
odometry or closure, has unit information whatever its length or how well
its scan match was constrained, so a closure's correction is spread evenly
while the drift is not (it comes from heading errors at the corners).
Weighting edges by their match covariance is an `rne_slam` change of its
own.

#### The optimizer was not Gauss-Newton (found after P3)

Weighting edges was tried first and moved nothing (three weightings of
odometry against closures, yaw against position: 0.56–0.65 m against
0.58–0.67 m over 20 loop runs). Rebuilding a finished run's graph with
every odometry edge set to the true relative pose did not move it
towards the truth either, which no correct solver allows. The cause:
`PoseGraph::optimize` (and `PoseGraph3d::optimize`) added only each
node's own block `JᵀΩJ` to the normal equations and dropped the blocks
coupling the two nodes of an edge. Each step was block-Jacobi, not
Gauss-Newton: a 350-node chain bent away from exact measurements barely
moved in 100 iterations. That is why closures corrected nothing above,
and why more iterations did not help.

Both optimizers now add the coupling blocks, and the fixed damping
(`1e-3`, which alone still stalled the chain's softest bending) became
Levenberg-Marquardt that starts as Gauss-Newton. The chain converges in
four iterations; a finished run with true odometry edges now lands within
0.03–0.15 m of the truth.

A working optimizer exposed what the inert one had hidden: `Slam2d`'s
single-scan loop closures, accepted within 0.75 m of the prediction,
included wrong ones (3 of 8 and 7 of 28 on warehouse start 0), and they now
bend the trajectory (SLAM error 0.239 → 0.591 m). A Huber kernel and edge
weights did not stop that; gating closures at 0.1 m of the prediction
(`SlamConfig::loop_max_translation_residual_m`, default 0.75 → 0.1) did:
no wrong closure on the four warehouse starts.

| | warehouse, 4 starts | loop worlds, 20 runs |
| --- | --- | --- |
| nearest frontier, before | 0.172 m | 0.583 m |
| nearest frontier, after | 0.157 m | 0.616 m |
| information gain, before | 0.076 m | 0.671 m |
| information gain, after | 0.091 m | 0.485 m |

(Loop worlds: `--rebuild-map --submap-loops --revisit`, seeds 0–9, both
starts.) The averages move less than the per-run results do, because the
estimate steers the robot and every run takes a different route. With a
converged graph, what is left on the loop worlds is mostly the odometry
edges themselves: replacing them with the true relative poses brings most
runs to 0.03–0.15 m, while replacing the closures does little, except in
two runs where one closure was wrong. Example 164's replay hash is now
`230d7c872fe7b908`.

### P3: legged robot, no goals given

- Example 131's Go2 + Mid-360, with `rne_explore` instead of the hand-given
  goals.
- Exit: both rooms explored; localization error in the range of 131's
  (0.042 m RMS).

#### P3 result (exit met)

`131_go2_mid360_navigation --explore`: the welded Go2 trots out of room A
with no goals and no map, nearest frontier first, until `rne_explore`
reports nothing reachable left. Same simulation, sensor model, leg odometry
and `Slam2d` as example 131's goal run; CI runs it as a smoke next to the
goal run, whose output is unchanged.

| | goals given (131) | explored (P3) |
| --- | --- | --- |
| finished | both goals, 180 s budget | nothing left to explore at 50 s |
| room A / room B floor mapped | — | 96.4 % / 98.9 % |
| SLAM error | 0.042 m RMS | 0.031 m RMS (0.054 max) |
| odometry alone, max | — | 4.1 m |
| base / body to obstacles | > 0.2 m | 0.50 m / 0.29 m |
| map cells on a real obstacle | 100 % | 100 % |

The body clearance (head, hips, knees and feet against the furniture's
footprints, sampled at 50 Hz, link radii of 2–5 cm not subtracted) is
reported and gated (> 0.10 m): every fall on the way was a leg touching
furniture.

What exploration needs that the goal run does not:

- **An obstacle layer for navigation.** The Mid-360 rides 0.447 m up, above
  the 0.40–0.50 m crates, so a beam to a distant wall still clears a nearby
  crate's top, and the 2D scan's free-space carving erased the crate from
  the SLAM map (in one layout 1 of 400 cells was left). Exploration plans on
  the SLAM map plus a separate layer, updated every frame from the 3D
  returns: the nearest return 0.10–0.55 m above the floor marks a cell, and
  only rays that reach the floor clear cells, up to that nearest return. A
  floor ray descends steeply, so a low obstacle in its way blocks it rather
  than being cleared. SLAM's own map and scan band are 131's.
- **Paths through the middle of free space.** `plan_path` with its default
  cost weight (0.5) runs along the inscribed band, and the trot turning there
  put a foot within 1–2 cm of furniture. Weight 4 keeps paths centred; the
  inscribed band is made lethal (except within 0.3 m of the robot, so it can
  back out), because `plan_path` only refuses lethal cells.
- **A footprint guard** (`guard.rs`). A disc cannot stand for the Go2: its
  head reaches 0.29 m ahead of the base, its rear legs 0.38 m behind and
  0.18 m to the sides, so a turn in place swings the rear through an arc a
  disc does not have. At 20 Hz the guard rolls the footprint rectangle
  (0.40 m back, 0.31 m forward, ±0.20 m, measured from the links over a
  walk) forward 1 s under the follower's command, against a distance field
  of the navigation map. If it would come within 8 cm of an obstacle (or,
  already closer, get any closer), it picks instead the best of 25
  speed/yaw-rate candidates that do not, by heading to the lookahead point,
  progress, clearance and closeness to the command being sent, and holds it
  0.5 s. The disc radius the planner uses is then only about leaving room:
  0.38 m (0.42 m closed the 1.0 m doorway in one layout once its jambs fell
  on 5 cm cells).
- **Commands that change gently.** Speed changes at most 0.5 m/s², yaw rate
  1.0 rad/s²: the guard and the follower can disagree in sign, and a yaw
  rate flipped from +0.5 to −0.25 rad/s within 0.2 s made the trot fall
  without touching anything.
- **Giving up a frontier.** Where the planner's disc fits but the guard
  finds no way for the body, the robot stops making progress and the
  explorer would choose the same frontier forever (one layout ran out its
  300 s). A frontier is given up after 10 s with less than 0.25 m of
  progress toward it, and filled in on the map the explorer chooses on.

**Generated layouts** (`--explore --layout-seed N`, `layouts.rs`): the same
shell with the doorway moved and 6–8 seeded boxes, 0.40–1.20 m tall,
against a wall or at least 1.0 m from walls and each other, every floor
cell reachable (tested on 50 seeds). The dressed scene plus layouts:

| | runs | fell | body > 0.10 m from furniture | passed | worst room mapped |
| --- | --- | --- | --- | --- | --- |
| P3 as first merged (seeds 0–9) | 10 | 1 | 7 | 7 | 69.5 % |
| obstacle layer + step away (dressed, seeds 0–19) | 21 | 0 | 19 | 19 | 96.6 % |
| footprint guard (dressed, seeds 0–39) | 41 | 0 | 41 (closest 0.17 m) | 41 | 96.4 % |
| footprint guard, held out (seeds 40–59) | 20 | 0 | 20 (closest 0.21 m) | 19 | 99.4 % |

Seeds 0–39 were looked at while fixing things (the doorway radius and the
give-up rule came from seeds 32 and 11), so seeds 40–59 were run once,
untouched. The one of them that fails does so on map precision: 93.1 % of
the SLAM map's occupied cells lie within 10 cm of a real obstacle, under
the 95 % gate inherited from the goal run, while exploration itself
finished with 99.8 % of the floor mapped. That is the SLAM map's quality,
not the walk's, and is left open here. Over all 61 runs SLAM error was
0.017–0.061 m RMS; the guard stepped in 195 times over seeds 0–39 (in 21
runs, at most 73 in one), and 4 frontiers were given up (in 2 runs).
Layout seed 4 runs in CI next to the dressed scene.

On the way: an earlier version kept the SLAM scan band under the lowest
crate (0.35 m) instead of adding the layer. It passed the dressed scene but
not the layouts, because beams from the 0.447 m sensor to distant points
under 0.35 m still clear a crate's top. With the band lowered the explorer
also chose a frontier 0.2 m away over and over (the scan drops returns
within 0.4 m, the robot's body, so standing on a frontier cannot clear it);
skipping reached frontiers fixed that, but it never triggers with the final
settings and was left out. `rne_explore` is unchanged in P3.

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

- P1 scene: the warehouse of example 127 (2026-11-26).
- P2 scenes: the generated loop worlds, with the office, maze and
  warehouse as regression checks (2026-11-26).
- P2 configuration: map rebuild with verified submap closures and
  longest-loop revisits, all opt-in; `Slam2d`'s default closure unchanged
  (2026-11-27).
- P3 lives in example 131 behind `--explore`, not a new example: the scene,
  sensor model and estimator are the point of comparison (2026-11-27).

## Open questions

- Whether frontier extraction belongs in `rne_nav` (it is a pure grid
  operation) with only the scoring in `rne_explore`.
- Multi-robot exploration (`rne_nav::coordination`, `map_merge`): later, not
  in this plan.
