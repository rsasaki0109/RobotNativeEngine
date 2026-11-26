# ADR 036: `rne_explore`, a crate for active SLAM decisions

- Status: Proposed
- Date: 2026-11-26

## Context

RNE can map and navigate without a prior map (`rne_slam::Slam2d`,
`rne_nav::plan_path` with `allow_unknown`; example 131), but the goals are
still given by hand. Active SLAM adds the decision of where to go next:
frontier exploration, information gain, and loop-closure-seeking revisits
([PLAN_ACTIVE_SLAM.md](../PLAN_ACTIVE_SLAM.md)).

That decision needs the occupancy grid (`rne_nav`) and the pose graph
(`rne_slam`). The Module Boundary Rules in `AGENTS.md` let `rne_slam` depend
on `rne_nav`, not the other way round, and list no crate that may depend on
both for decision making.

## Decision

Add `crates/rne_explore`. It may depend on `rne_nav`, `rne_slam`,
`rne_core`, `rne_ecs` and `rne_math`. It must not depend on a renderer, a
physics backend, ROS 2, or visloc-rs. It returns goals; it does not plan,
drive or estimate. Camera-based belief enters through a trait implemented
in an example crate.

`AGENTS.md` gains:

- under Repository Layout: `crates/rne_explore`: active SLAM decisions
  (frontiers, information gain, uncertainty-aware goal selection);
- under Allowed dependencies: `rne_explore` may depend on `rne_nav`,
  `rne_slam`, `rne_core`, `rne_ecs`, and `rne_math`.

The crate is `publish = false` and outside the Rust API baseline until the
plan's P2 lands.

## Alternatives considered

- **Inside `rne_slam`.** Mixes estimation with behaviour; the decision code
  would be tested through the estimator.
- **Inside `rne_nav`.** Would need `rne_nav -> rne_slam`, inverting the
  existing dependency.
- **Inside visloc-rs.** visloc-rs keeps reconstruction and localization;
  planning and simulation moved to RNE (visloc-rs `docs/rne_migration.md`).
- **Example-only code.** Every example (131, the P1 example, 163) would
  copy it.

## Consequences

- One more workspace crate and one more dependency rule.
- Lidar and camera exploration share one decision crate.
- Frontier extraction may later move into `rne_nav` as a pure grid
  operation if other crates need it.
