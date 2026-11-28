# robot_navigation

3D navigation in space and time over a forecast of the world, with learned
and classical planners competing behind one certifier. See
[docs/FORESIGHT.md](../../docs/FORESIGHT.md) and
[example 165](../../examples/165_foresight_drone_warehouse/main.rs).

- `VoxelGrid`, `NavProblem`, `KinematicLimits`, `Trajectory3`
- Proposers: `SpaceTimeAstar`, `StraightLineProposer`, `LearnedVelocityProposer`
- `ShortcutRefiner`; critics `DurationCritic`, `PathLengthCritic`,
  `RiskCritic`, `LearnedCostCritic`
- `ForecastCertifier`, `check_trajectory`, `default_pipeline`
- `planar`: occupancy grid, costmap, A*, pure pursuit and DWA, ported from
  `rne_nav`

Dependencies: `robot_foresight`, `glam`, `serde`, `thiserror`. No RNE crate.

```bash
cargo test -p robot_navigation
cargo run -p foresight_drone_warehouse --example 165_foresight_drone_warehouse
```

To move this crate to its own repository (`robot-navigation`):
`git subtree split --prefix modules/robot-navigation -b robot-navigation`,
replace the `*.workspace = true` entries with literal values, and depend on
`robot_foresight` by git or registry.
