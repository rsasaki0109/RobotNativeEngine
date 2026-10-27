# LeKiwi pick/place reference

`report.json` is a measured six-case simulation report, retained unchanged from
`ci-lekiwi-evaluation` on source commit
`bb1da25138e5f9900f74f2b2e66c6dbcc11cf33e` (merged as PR #446).
It was generated on Linux x86_64 with Rust 1.95.0 and RNE 0.4.0, without GPU or
hardware. The run met all six expectations and verified all four executed
recordings. Placement errors were 1.372, 1.349, and 1.359 mm; settled drift was zero.

This file is a metric reference, not a replay bundle. Its recording filenames
refer to the original run; recordings are not vendored here. New CI runs generate
and verify their own recordings and retain a copy of this reference alongside them.

The gate uses `--compare-evaluations` with its fixed 1 mm allowance for increases
in XZ placement error and settled drift, and decreases in grasp lift and minimum
hold lift. Completed cases require grasp measurements. Outcomes, replay evidence, expected
failure reasons/phases, case identity, and destination compatibility are checked
by the comparator. Improvements pass. Incompatible schemas or catalogs fail.

To update intentionally:

1. Run `cargo run --locked -p xtask -- ci-lekiwi-evaluation`. A baseline mismatch
   may fail the gate after producing the candidate report and recordings.
2. Inspect the candidate `evaluation/report.json`, replay evidence, and baseline
   comparison logs. Explain the physical or controller change and any metric
   degradation in the PR. Do not replace the baseline merely to clear a failure.
3. Replace this report only with that measured report; record its source commit,
   platform/toolchain, and changed metrics here. Do not edit measured values.
4. Rerun the gate and review the baseline diff along with the implementation.

If the catalog intentionally changes, update its version and comparison contract
as well. A reference update is an explicit repository change, never a CI side effect.
