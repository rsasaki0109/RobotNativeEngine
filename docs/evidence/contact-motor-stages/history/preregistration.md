# Coupled heavy-grasp stage measurement: independent preregistration

PREPARATION ONLY UNTIL THE SOURCE / SCHEMA / MODE / READER / BINARY FREEZE IS
RECORDED. No measured outcome exists in this document. Generation 3 remains
REJECTED; its numerical primary PASS and failed example-138 admission gate are
different facts. Prior public reports and outside proposals remain immutable.

## Fixed experiment and interpretation

Exactly eighteen authoritative coupled worlds are proposed: restored baseline
build first, then the exact previously rejected ForceBased-only inverse(w+g)
comparison build; within each build, masses 5, 20 and 50 kg each have observer ON,
fresh ON and OFF. Each world has 1,250 completed outer steps, with the original
example-138 ECS scene, spawn order, force/gain/cap/geometry/material parameters,
500 Hz, and 250 settle / 500 lift / 500 hold phases. There is no sweep, controller
retuning, rerun until favorable, convergence cutoff or substituted fixture.
Any failed/partial world is retained; replacing it requires a new explicit
registered generation rather than amending this experiment's eighteen cells.

A separate, explicitly counted seventy-two-world example-141 current-source
qualification uses the unchanged scalar collector/schema/conditions/thresholds.
It is not part of these eighteen coupled worlds and must have its own immutable
source/report binding and independent numerical audit. Historical reports are
not rebound. Unit fixture worlds, if authorized by the parent, are separately
counted; none are executed by this independent reader or its pure tests.

The first measurement asks whether a phase-qualified, fixed cached palm motor
row's projected residual changes or reopens after an aggregate contact-only
stage. It does not measure individual contact J/W columns, friction impulse
components, hidden pipeline state, or J_m W_c. No attribution to a particular
contact, causal explanation for generation 3, solver promotion, accuracy gain,
hardware validation or maturity gain follows from an algebraic identity.
Examples 133--138 retain the existing exact physical-stdout admission rule with
only the previously registered three timing regex exclusions. A mismatch keeps
the candidate rejected and triggers rollback without retuning.

## Physical and observation replay scope

Every completed outer step carries the exact typed checkpoint and its literal
postcard bytes. The independent reader reconstructs the wrapper postcard from
its ordered primitive fields and requires byte equality. Individual native
bodies/colliders/unique multibodies/impulse joints remain opaque source-bound
native postcard records; the Rust producer checks exact native consumption and
reencoding. The reader does not pretend to decode their private layouts.
Records are grouped by enum kind and sorted by native index/generation; bindings
are sorted by complete ECS entity bits. Native gravity and every explicit
IntegrationParameters IEEE word/count are included. Set metadata, broad/narrow
phase and private pipeline/solver/contact caches are excluded and unmeasured.

The independent ECS parser checks the declared six entities and complete
presence-tagged transform, body and joint-state IEEE words, including signed
zero and absent values without zero filling. Literal native checkpoint bytes,
typed checkpoint values, ECS words and final physical result words must match
ON / fresh ON / OFF within each build at all 1,250 steps. Ordered native stage
objects/words must match ON / fresh ON. OFF has null stage telemetry. Labels and
observer-presence fields are excluded only from the physical comparison; no
physical word, checkpoint field or trace event is excluded. Cross-build physical
differences are retained as measurements and are not an observer-replay failure.

JSONL framing requires one deterministic gzip member, UTF-8, no duplicate JSON
keys or JSON float/NaN literals, a run_start, exactly sequential completed_step
records 1..1250, and exactly one run_end. Gzip CRC/trailer/truncation, extra member
or trailing bytes, non-newline-terminated records, missing/error terminals,
duplicate steps, unknown fields and bounded-resource overflow are explicit
failures. All successful compression completion and actual process exits remain
separate collection receipts, never a numerical PASS label.

## Row availability, identity and ordered phase coverage

BeforeExternalForces and AfterExternalForces deliberately expose no rows and
row_cache_slice_index=None on every slice. Their absence is unavailable, not zero
motor residual. AfterJointUpdate onward retains the actual joint-row cache
generation, including after integration updates the separate multibody dynamics
cache. Current V is the actual full generic solver vector; the row names its
solver_velocity_offset, ndofs2 and j_id2. J/W come from the cached joint-Jacobian
buffer and are never reconstructed from a later mass matrix or from the separate
contact buffer. Full block lengths and bounds are checked before any dot.

The source-bound grammar validates partition / island / slice / iteration and
selected-row global joint visit order. Actual expected slice counts and actual
integration settings determine loop coverage. Missing warmstart / PGS / additional
friction / stabilization loops are recorded as skipped only when the source and
actual configuration require that skip; no fabricated zero events are inserted.
Each initialized supported row must have its immediate BeforeMotorSolve /
AfterMotorSolve pair. Stabilization also requires BeforeMotorBiasRemoval before
the bias replacement. Whole-joint boundaries include subsequent nonmotor limit
writes and cannot be substituted for that immediate pair. Empty supported-row
lists in a nonmotor island remain explicit unavailable coverage.

Contact-only eligible intervals are the literal source-adjacent boundaries for
warmstart, normal, friction, additional final-slice friction, stabilization normal
and final stabilization friction. Identity, cache context and all row words
(including lambda, J, W, RHS, bias-free RHS, g, coefficient, D, bounds and ceiling)
must be unchanged. Every interval is retained with eligibility or exact reasons;
no size, sign, significance or settling cutoff is applied. Force increments,
bias removal, rebuild/reset, integration and any joint/limit pass are not labeled
contact-only. Reset-to-+0 at each rebuilt motor row, unchanged bias-removal fields,
and exact selected-visit lambda/full-V results are independent structural checks.

## Common reference and source-specific native projection

For a finite valid cached row and its native generic velocity block:

    r = rhs + sum_k J_k V_k - g lambda
    w = sum_k J_k W_k
    denominator = w + g
    P_ref = lambda - clamp(lambda + r / denominator, lower, upper)

All quantities here use exact Fraction arithmetic from stored binary32 words.
Bounds can be signed infinite but cannot be NaN, reversed, or leave no finite
feasible accumulator. The denominator must be positive; otherwise the snapshot
is retained as unavailable. D is not substituted into the common denominator.
Linear units are r m/s, lambda/P N s, w/g 1/kg, D kg; angular units are rad/s,
N m s and rotational inertia. The native applied motor impulse is -lambda.

At lower bounds the fixed-point sign is r<=0, at upper bounds r>=0, and in the
interior r=0. Zero-width bounds report zero authority rather than successful
control; out-of-bound accumulators remain separately marked. Strict reopening
means an eligible before P equals exact zero and the after P does not. A row
already nonzero reports changed/increased/decreased/equal separately. P_ref and
P_native reopening counts are distinct; no tolerance is invented for either.

Native projection is independently replayed in the reviewed source order:
Dyn/Dyn nalgebra dot8; RN(rhs+dot); RN(g*lambda); RN subtraction; RN(D*r);
RN(lambda+correction); literal nested comparison clamp; RN(next-lambda).
An actual selected motor visit additionally requires every word of
V <- axpy(-delta,W,1,V), including unchanged outside-block coordinates.
The eight dot accumulators and four pair additions are retained even for a short
vector. The AXPY multiplications by one are retained. No FMA or reassociation is
assumed. The baseline row's D is checked against guarded inv(stored diagonal);
the comparison ForceBased D against guarded inv(RN(w_native+g_native)), with
AccelerationBased unchanged. One-hot J and the actual selected weighted word
must qualify the builder's stored diagonal; it is not an inferred later LU.

## Floating-point qualification and rounding accounting

Exact-reference availability is separate from native replay availability.
Native replay requires source/compiler/target/features and a registered scalar
mode establishing binary32 RN ties-to-even and the reviewed no-FMA scalar
operation lowering. Rapier's source-bound x86/SSE non-enhanced-determinism scope
sets MXCSR FTZ/DAZ while preserving other flags; the reader never infers the mode
from recorded values. This generation freezes a NORMAL-OR-ZERO replay domain:
any subnormal arithmetic input, nonzero exact intermediate with magnitude below
2^-126, nonfinite arithmetic or overflow is explicitly unqualified and retained.
Gradual-underflow calculations outside that domain are illustrative only and
cannot count as complete native replay. Any unknown mode/order binding likewise
prevents a complete native qualification. Normal/zero replay is valid under the
registered gradual or FTZ/DAZ mode only within this domain. No runtime FP mode
change is made to manufacture qualification.

For every eligible fixed-row contact interval, require the exact identity
delta_r = J_m delta_V. The exact finite native residual rounding errors e_before
and e_after are retained when native projection is qualified. The discrepancy
between separately rounded native residual change and exact J delta_V must be
exactly e_after-e_before. A separately computed native dot(delta_V) is never
substituted for those two source-ordered endpoint evaluations. These identities
are consistency checks; they do not replay or explain the hidden contact solve.

## Admission, freeze and retained outputs

The implemented reader, arithmetic module, pure-test sources/results, JSON schema,
fixed protocol, dependency operation source hashes, and resource ceilings are
hashed before any authoritative world. Actual production/fixture source maps,
compiler/target/features/float-mode contract, both binary hashes and exact
candidate diff are then bound before collection. A prepare-only hash is not a
completed freeze. No audit of measurement files occurs before the parent reports
the once-only collection terminal. Failures and partial outputs are preserved;
after-outcome reader or criteria changes require a new named generation.

Reader outputs retain every numerical snapshot and contact interval in compressed
audit details, including unavailable samples, and a hash-bound summary with all
cells, physical/trace/phase/word counts, native qualification coverage, immediate
visit word comparisons, saturation, exact/reference and native reopening counts,
and maximum magnitudes with first deterministic occurrence. The unqualified
domain is never summarized as PASS. Pure analytical IEEE/sign/bounds/rounding,
gzip framing and malformed/tampered payload checks execute zero native worlds.
No numerical causal result, adoption result or scientific outcome is preregistered.
