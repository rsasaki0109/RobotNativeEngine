# Rejected ForceBased inverse-diagonal experiment

This is historical evidence for a rejected solver candidate, not the current
production solver. The candidate passed example 141's fixed scalar calibration
but failed the preregistered requirement to preserve examples 133–138's physical
stdout. All candidate Rust changes were rolled back before this evidence-only PR.

## Fixed intervention and outcome

The sole production change replaced `inv(w)` with `inv(fl32(w + g))` for internal
single-DOF **ForceBased** motor rows; `g` is the existing CFM gain. The
AccelerationBased branch retained its original expression. No gains, limits,
fixture inputs, integration counts, recovery predicates, or calibration tolerances
were changed. The full archived patch also contains attempted tests and an
experimental CI command, so it is evidence rather than a patch to apply blindly.

The independent exact-Fraction oracle and audit tool were frozen before the
candidate build and before any new physics world. One new 24-condition matrix
constructed 72 fresh worlds: two readback-ON worlds and one OFF world per condition.
The two original 72-world reports remain byte-identical to their previously
published versions: 216 matrix worlds across the three generations, without
selecting or rerunning a favorable cell.

| Criterion | Merged solver (generation 2) | Candidate (generation 3) |
|---|---:|---:|
| Primary N32/PGS1 nonzero calibration | 0/4 | 4/4 |
| All nonzero calibration conditions | 0/16 | 16/16 |
| Motionless zero-target controls | 8/8 | 8/8 |
| Complete matrix worlds | 72/72 | 72/72 |
| Exact ON/fresh/OFF state and ON/fresh telemetry within each build | PASS | PASS |

The candidate's maximum first-slice impulse error over all 16 nonzero conditions
was `9.548473936440288e-13 N m s`. All initial states, factors, rounded coefficients,
fixed-point references, and tolerances match the previous generation. Changed
nonzero final states are retained; within-build replay does not establish
unchanged physical results across solver versions.

## Required compatibility failure

All six release programs returned 0 and printed `ok`. Examples 133–137 matched
their declared physical baselines exactly. Example 138 did not. The reader used
only the three timing exclusions already fixed before the intervention. All six
native grasp rows matched; four of six Rapier rows changed. Selected held cases:

| Mass | Baseline lift (m) | Candidate lift (m) | Baseline slip (m) | Candidate slip (m) | Baseline tilt (rad) | Candidate tilt (rad) |
|---|---:|---:|---:|---:|---:|---:|
| 5 kg | 0.200 | 0.199 | 1.8e-4 | 3.7e-5 | 8.6e-4 | 5.6e-4 |
| 20 kg | 0.198 | 0.196 | 5.7e-4 | 1.0e-3 | 3.7e-4 | 8.8e-3 |
| 50 kg | 0.196 | 0.191 | 1.5e-3 | 3.1e-3 | 5.7e-3 | 2.3e-2 |

The 5 kg under-squeezed tilt also changed (`3.0e-5` to `3.1e-5 rad`). The complete
baseline and candidate stdout, all six command receipts, and independent rejection
audit are retained. Program-level `ok` is insufficient for the stronger numerical
preservation gate. The candidate is **REJECTED**, with no post-outcome retuning or
relaxation of that gate. A fresh post-rollback release run restores all six
original physical baselines exactly and prints `ok` for every example. Its
separate receipts are retained under `restored-default-release-regressions/`.

The attempted new vendor unit tests failed to compile (`cargo test --lib --no-run`,
exit 101, three E0603 private-module imports). They executed no test worlds and
provide no passing unit qualification. Their source is preserved in the archived
patch; a later test-only visibility proposal was never applied. Direct strict
vendor Clippy/rustdoc were not completed; unchanged pre-existing vendor warnings
are not an exemption or a passing result. No G1 recovery, hardware, cross-platform
bit identity, independent external adoption, or practical maturity gain follows
from this contact-free fixture.

## Evidence and verification boundary

`artifact-manifest.json` binds every retained artifact in this directory and the
new full report at `../../media/position-motor-diagonal-calibration.json`. The
preregistered protocol, exact candidate patch, selected 118-file source snapshot,
compiler/features/lock binding, frozen-binary digest, actual command exits,
independent numerical audit, and compatibility rejection remain distinct records.
They describe a bounded local source snapshot, not a complete dependency/compiler
closure. Original absolute paths are historical provenance, not portable commands.
The frozen executable remains outside the repository; its digest is retained.

The candidate report intentionally has a different solver source binding. The
current example 141 verifier must reject it as stale after rollback. Do not
change the production verifier, replace the existing generation-2 report, or
interpret the candidate's numerical PASS as the current default result. The
current CI continues to verify the original report and its calibration FAIL.

The `independent-oracle/` Python tools are archived unchanged for inspecting the
pre-measurement computation; the original audit checks its historical workspace
bindings and is not presented as a portable current-source verifier. Reading the
saved receipts does not advance physics.
