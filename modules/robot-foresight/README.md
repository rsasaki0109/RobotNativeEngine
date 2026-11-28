# robot_foresight

Forecasts of future world states and a pipeline in which learned and
classical algorithms are interchangeable. Shared core of
[`robot_navigation`](../robot-navigation) and
[`robot_manipulation`](../robot-manipulation). See
[docs/FORESIGHT.md](../../docs/FORESIGHT.md).

- `ForecastSet`, `AgentForecast`, `TrajectoryMode`, `WorldForecast`
- `ConstantVelocityForecaster`, `LearnedForecaster`, `BlendedForecaster`
- `HybridPipeline` with `Proposer`, `Refiner`, `Critic`, `Certifier`,
  `Provenance`, `Decision`
- `InferenceModel`, `MlpModel` (JSON weights), `FnModel`
- `SplitMix64`

Dependencies: `glam`, `serde`, `serde_json`, `thiserror`. No RNE crate.

```bash
cargo test -p robot_foresight
```

To move this crate to its own repository:
`git subtree split --prefix modules/robot-foresight -b robot-foresight`, then
replace the `*.workspace = true` entries in `Cargo.toml` with literal values.
