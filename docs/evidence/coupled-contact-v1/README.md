# Baseline coupled-contact diagnostic v1

Nine new baseline-only processes used the unchanged example-142 held-grasp
fixture: 5/20/50 kg, each ON/fresh ON/OFF, 1,250 steps. All original native,
outer collection and single scientific-reader exits are zero. The
[whole audit](audit.json) reports `PASS_DECLARED_CAPTURE_AND_SOURCE_DOMAIN`.

| Retained check | Completed coverage |
|---|---:|
| Captured native frames | 11,250 |
| Canonical ON frames analyzed | 3,750 |
| Immediate motor-row visits | 135,000 |
| Rebuilt row diagonals | 45,000 |
| Contact-only row intervals | 270,000 |

Fresh ON is literal replay, not another numerical sample. ON/fresh/OFF captured
native checkpoints and ECS words match exactly; fresh ON completed records and
ordered stage words match literally; OFF telemetry is absent. All three rows
are covered: ForceBased palm and two zero-cap AccelerationBased finger motors.
The fingers still apply external physical squeeze effort.

The exact reference is `r = rhs + J V - g lambda`, `w = J W`, and
`E_ref = clip(lambda + r/(w+g), bounds) - lambda`; native applied impulse is
`p = -lambda`. The separate source-rounded projection uses the captured native
D and full-block dot/clamp/AXPY order. These are conditional next-visit
predictions, not extra native solves. Fixed-row contact pairs verify
`delta r = +J delta V`. No strict `REOPENED` transition is counted. Contacts
change already nonzero projected defects; zero-authority defects are zero by
bounds. No favorable transition was required.

The maximum palm projected-defect magnitudes below are copied from the existing
result summary. They are cached impulse-coordinate update defects, not joint
position errors or interval torque errors. The native projection uses the captured
source diagonal, while the reference uses the common conditional denominator.
Their maxima do not quantify physical accuracy.

| Authored cube mass (kg) | Maximum palm reference projected defect | Maximum palm native projected defect | First maximum reference context |
|---|---:|---:|---|
| 5 | 0.0371822177323 | 0.0475513860583 | step 253, slice 1, AfterContactWarmstart |
| 20 | 0.116404581952 | 0.148866832256 | step 259, slice 1, AfterContactWarmstart |
| 50 | 0.268112021144 | 0.342881590128 | step 268, slice 1, AfterContactWarmstart |

All 360,000 palm snapshots are interior with a nonzero exact residual. The count
includes unsolved and cached-row phases and exact interpretations of f32 stored
operands; it is not a solver-convergence acceptance test. The complete audit
retains every row and contact-interval classification, including increases and
decreases of already nonzero defects.

Qualification covers primitive checkpoint encoding, captured native-object
bytes, ECS words and declared source-ordered normal/zero arithmetic. Individual
native-object payloads remain opaque. Full-set metadata/private pipeline or
contact caches, per-contact J/W columns and impulses, absolute constructor RHS
inputs, unselected limit-row arithmetic, anatomical finger identities and runtime
FP control registers remain unmeasured. This establishes no general motor
accuracy/convergence, historical example-138 cause, hardware result, solver
adoption or maturity increase. Historical failed reports, scalar FAIL 0/4 and
the rejected inverse-diagonal candidate are unchanged.

## Whole retention and availability

The local whole archive is **1,317,548,213 bytes**, SHA-256
`4dd915ab6cb549cce86241f36e7782743623396c603182b12749ee4050cc998a`.
It contains 19,447 typed objects, including 32 symlink objects stored without
following their targets. It retains all nine whole raw gzip streams, the complete
details stream, source/history and original lifecycle evidence. Thirty-three
[local-only descriptors](local-only-bindings.json) identify executables or
compiler/runtime payloads omitted as literal bytes; their bindings are retained.
This is not a hermetic public native build.

[Creation](archive-create-receipt.json) and
[whole byte verification](archive-verify-receipt.json) completed with original
exit 0 ([create exit](archive-create-original-exit.json),
[verify exit](archive-verify-original-exit.json)). Verification checked complete
lengths, digests, gzip CRC/EOF, typed inventory and retained original bytes. It
performed no scientific reader invocation or numerical reanalysis. Byte
consistency does not rerun or independently requalify the scientific experiment.

The [ADR-034 proposed pointer](coupled-contact-v1-b9a6c853.tar.gz.PROPOSED.pointer.json)
uses tag `evidence-coupled-contact-v1-b9a6c853`. Its HTTPS URL is **proposed**:
publication is approved, but upload returned HTTP 401 from `uploads.github.com`.
The named secure upload binding is absent in the runtime. Upload and public
download/hash verification remain pending; see [the actual status](publication/status.json)
and [original failed upload](publication/upload-original-exit.json).
The exact whole archive is not split, sampled or embedded in Git. The complete
161,320-byte audit is copied inline unchanged. No publication success or future promotion gate is asserted here. Current-head
validation results are recorded separately from these completed-study receipts.

## Portable byte verification

The [small verifier packet](verifier/README.md) contains the exact qualified
archive tool, synthetic tests and original qualification receipts. Use its
`verify --no-originals` mode to check the entire downloaded archive without
reading historical source paths or executing archived programs. The CLI checks
whole member and archive bytes; it does not rerun the scientific reader or
establish public availability. The 30 copied synthetic tests and one actual
portable CLI completed with original exit 0 on the retained archive.

The verifier packet retains its original preparation-time notes and byte manifest
unchanged. The publication status above supersedes historical approval-pending
wording inside those literal qualification records.
