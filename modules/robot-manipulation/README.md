# robot_manipulation

Arm motion and grasp planning around a forecast of moving agents, with
learned and classical stages behind the same certifiers. See
[docs/FORESIGHT.md](../../docs/FORESIGHT.md) and
[example 166](../../examples/166_foresight_arm_shared_workspace/main.rs).

- `SerialChain` (DH, FK, collision spheres, IK), `ManipScene`, `MotionProblem`,
  `JointTrajectory`
- `time_scale_path`: path-velocity decomposition around the forecast
- Motion proposers: `DirectJointProposer`, `RrtConnectProposer`,
  `LearnedViaPointProposer`; `MotionCertifier`; duration, path and risk critics
- Grasps: `PointCloud`, `AntipodalGraspSampler`, `AntipodalQualityCritic`,
  `LearnedGraspScorer`, `ReachabilityCertifier`
- `default_motion_pipeline`, `default_grasp_pipeline`

Dependencies: `robot_foresight`, `glam`, `serde`. No RNE crate.

```bash
cargo test -p robot_manipulation
cargo run -p foresight_arm_shared_workspace --example 166_foresight_arm_shared_workspace
```

To move this crate to its own repository (`robot-manipulation`):
`git subtree split --prefix modules/robot-manipulation -b robot-manipulation`,
replace the `*.workspace = true` entries with literal values, and depend on
`robot_foresight` by git or registry.
