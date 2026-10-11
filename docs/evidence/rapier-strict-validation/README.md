# Strict validation of the vendored Rapier solver

Restore standalone warning-denied validation before adding further motor/contact
diagnostics. This is mechanical cleanup, with no changed motor/contact equation,
iteration ordering, gains or solver settings. Contact-stage observation remains
planned outside the checkout; this change collects no coupled-grasp stage data.

| Actual standalone default-feature check | Merged #407 baseline | Cleaned source |
| --- | --- | --- |
| Clippy library, `-D warnings` | exit 101, 39 diagnostics | exit 0 |
| Clippy all targets, `-D warnings` | not run | exit 0 |
| rustdoc, `-D warnings`, no deps | exit 101 | exit 0 |

The checks use pinned Rust 1.95.0 and the unchanged standalone locked dependency
graph. Their scope is default dim3/f32; they do not qualify every feature mask.
Workspace dependency builds cap upstream warnings, so their success alone did
not establish these direct vendor results. See the retained
[baseline outcomes](vendor-checks/actual-baseline-results.json),
[diagnostic inventory](vendor-checks/baseline-diagnostic-inventory.json), and
[final direct checks](vendor-checks/after-documentation-fixes-checks/actual-results.json).
An intermediate rustdoc failure is retained rather than relabelled as a pass.

The 13 changed files spell elided lifetimes explicitly, remove two ineffective
attributes and three unused crate-private graph iterator types, fix documentation
layout/links and use equivalent default and integer expressions. The incomplete
reference snippet uses the standard `rust,ignore` fence instead of invalid
`.skip` syntax; rustdoc validation does not execute that snippet. No lint is
disabled. The [exact patch](source-cleanup.patch) preserves its original bytes.
Independent mechanical review reconstructs the owned changes and checks the
constructor, positive-divisor predicate and private-reference boundaries.

## A fresh source binding with unchanged measurements

Example 141 hashes 17 selected files. Cleanup changes two of them:
`multibody.rs` contains explicit lifetimes and a repaired link;
`rigid_body_components.rs` loses ineffective `must_use` attributes.
The collector, schema, codec, fixture, gains, tolerances and remaining 15 selected
sources are unchanged. Earlier reports stay byte-identical historical evidence.
The current verifier correctly rejects their stale source binding.

A [protocol](source-transition/preregistered-vendor-lint-report-transition-protocol.json)
and [independent reader](source-transition/audit_vendor_lint_report.py) are frozen
before the new worlds. Final source/compiler/features and the actual locked
release executable are bound before collection. Run the fixed 24 conditions once:
two fresh observer-ON worlds and one observer-OFF world per condition, 72 worlds.
The separately retained [current-source report](../../media/position-motor-calibration-vendor-lint.json)
has SHA256 `a2f9b90b964123b1ee5dc69d2f2d10e320882c2b3317d6f49b0c8b490dcef8b1`.

The [actual independent audit](source-transition/actual-independent-72-world-audit.json)
passes with actual exit 0. Replacing only the two preregistered source-hash values
in the original schema-2 report produces the new report's exact raw bytes. Every
other byte—including postcard state, exposed IEEE words, ordered telemetry,
oracle assessments, sensitivity metrics and summaries—is unchanged.

| Qualification | Result |
| --- | --- |
| Complete fixed worlds | 72/72 |
| Native ON/fresh/OFF state and ON/fresh telemetry equality | PASS |
| Primary accuracy criterion | 0/4, FAIL retained |
| All nonzero conditions | 0/16, FAIL retained |
| Motionless zero controls | 8/8 |
| Release examples 133–138, full physical stdout | 6/6 unchanged |

The comparison covers 144 native snapshots, 432 serialized entities, 247,248
postcard bytes, 42,264 state canonical words, 48 ON telemetry runs, 792 ordered
slices and 10,704 telemetry words. These scopes exclude private pipeline caches;
they are neither hardware accuracy nor cross-platform bit-identity claims.

CI loads the new report through the unchanged current-source verifier and
numerical criteria. Historical schema-1/schema-2 reports, codec migration and
the rejected inverse-diagonal report remain byte-identical. Generation 3 stays
REJECTED. This cleanup changes no calibration outcome or practical maturity.

## Receipts and reproduction

The [artifact manifest](artifact-manifest.json) binds this bundle and the new
report. The archived tools retain their original absolute paths and are
historical receipts, not portable current verifiers. The frozen executable is
retained locally and represented by its digest, rather than distributed here.
The 118-file source snapshot and extension-filtered command guards are bounded
evidence, not an exhaustive compiler/dependency closure.

```bash
cargo clippy --locked --manifest-path third_party/rapier3d/Cargo.toml \
  --all-targets -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --locked \
  --manifest-path third_party/rapier3d/Cargo.toml --no-deps
cargo run --locked -p position_motor_calibration \
  --example 141_position_motor_calibration -- \
  --verify-report docs/media/position-motor-calibration-vendor-lint.json
```

The direct vendor commands require the corresponding standalone dependency lock.
Whole-workspace CI and GitHub CI are separate validation results reported in the
PR. Release outputs and their baseline comparison are retained in
[release-regressions](release-regressions/results.json); only the original three
timing exclusions are applied, with no relaxed physical gate.
