# Example 141 source transition: inactive limit rows

The vendored Rapier now skips the `M⁻¹e_k` solve of a multibody limit row
while its coordinate is within the limits and no motor on that coordinate
reuses the row's unit Jacobian. See "Inactive limit rows" in
[RNE_PATCH.md](../../../third_party/rapier3d/RNE_PATCH.md). Within its limits
such a row's impulse is clamped to zero, so it never changed a velocity.

Example 141's report binds three of the changed files:
- `multibody_joint.rs`;
- `unit_multibody_joint.rs`;
- `joint_generic_constraint.rs`.

The unchanged 24-condition, 72-world collector was run again on the new
sources:

```bash
cargo run --locked --release -p position_motor_calibration \
  --example 141_position_motor_calibration -- \
  --output target/motor-calibration-inactive-limit/report.json
```

It completed 72/72 worlds with replay qualification, and primary calibration
still fails (`primary_calibration=FAIL`), as before. The
[new report](../../media/position-motor-calibration-inactive-limit-source.json)
replaces the [previous one](../../media/position-motor-calibration-motor-construction-source.json)
in CI. The previous report keeps its bytes, and its stale-source check now
fails, as intended.

[compare_reports.py](compare_reports.py) checks the transition:
- every byte outside the `selected_source_sha256` objects is identical: all
  native state, telemetry, assessments and the summary;
- the source keys are the same;
- exactly the three files above change hash;
- each new hash matches the file in the checkout.

```bash
python3 docs/evidence/inactive-limit-rows/compare_reports.py \
  docs/media/position-motor-calibration-motor-construction-source.json \
  docs/media/position-motor-calibration-inactive-limit-source.json \
  third_party/rapier3d/src/dynamics/joint/multibody_joint/multibody_joint.rs \
  third_party/rapier3d/src/dynamics/joint/multibody_joint/unit_multibody_joint.rs \
  third_party/rapier3d/src/dynamics/solver/joint_constraint/joint_generic_constraint.rs
```

Its output at the transition is [comparison.json](comparison.json).

When #520 merged ahead of this transition, the new report's `Cargo.toml` and
`Cargo.lock` hashes were updated to the merged workspace, as every report
binding them is; `comparison.json` records the transition before that.
