# Generation five: prospectively admitted all-motor coupled stage experiment

This is a new cohort. It does not reread or reclassify generation four. The
prior palm-only gate and failed/unqualified outcome remain immutable. The only
new role-admission scope is explicitly registered before this cohort's data:
all supported internal Motor(0) rows in the unchanged example 142 gripper,
including both zero-authority AccelerationBased finger rows. No numerical
arithmetic, tolerance, physical parameters or checkpoint scope is relaxed.
The archived generation-three inverse-(w+g) candidate remains REJECTED.

## Fixed workloads and separation

Freeze eighteen worlds, in build order `baseline`,
`rejected_inverse_w_plus_g`; mass order 5, 20, 50 kg; then mode order `on`,
`fresh_on`, `off`. Labels are `<build>-m<mass>-<mode>`. Each process constructs
one new world and completes 250 settle, 500 lift and 500 hold steps at 500 Hz,
1250 steps per world. Fresh ON is literal replay, not an independent numerical
sample. ON is the analyzed trace: six build/mass trajectories, 7500 outer steps.
No new scalar qualification worlds are part of this experiment. Historical
scalar accuracy failure is not improved or reinterpreted by this reader.

Root freezes both locked release binaries, compiler/target/flags/features,
source maps, exact rejected runtime-only patch, restored baseline, driver,
reader closure and this protocol before the first authoritative world. The
candidate differs only by the original ForceBased `D=inv(w+g)` reciprocal hunk;
AccelerationBased still uses `inv(w)`. The original complete historical patch
and its literal runtime-only first section are bound. Recorded before/after
source guards must equal the restored baseline for each process. The candidate
is associated with its historical compiled source variant, never a claim that
both variants were simultaneously checked out.

At audit time, the selected schema/runtime sources and unit-row implementation
must still equal the restored baseline. Broader recorded source guards are
historical collection receipts. Publication-only metadata outside the selected
runtime set may change after collection; this prospective policy does not
permit runtime-source edits, retuning or changed observed inputs.

## Source-derived identities and fields

Admit all six ECS entities, exact body types and joint-state coverage; identify
the unique dynamic 1 kg prismatic palm and two dynamic .2 kg prismatic fingers.
The typed checkpoint joins these entities to three unique native child handles
and one multibody. No numeric runtime entity/handle generation is guessed.
Require palm vector link 1, fingers vector links 2/3, native joint-frame LinX,
three generalized coordinates and exact unit J selector at link_index minus
one. Native append/fixed-root source supports this topology even when a finger
subtree is inserted before the anchor. Do not assert native internal_id equals
vector index. Finger names `finger_slot_0/1` mean link/append slots; anatomical
left/right frame labels remain unmeasured.

Native handle serde is exactly `{index:u32,generation:u32}`. Checkpoint handle
words have their separate two-word-array grammar. Unknown fields, arrays used
as native handles, bool/negative/overflow integers, model/axis/entity/link
mismatches, aliases, missing/extra/duplicated rows, wrong vector/J-buffer offsets,
wrong shape, or unexpected phases fail closed. Freeze runtime role identities
at the first checkpoint and require them thereafter.

All initialized motor-island aggregate stages contain exactly three supported
rows; ordinary-only islands may have zero. Observed active-island builder order
is respected, not forced to a palm-first permutation. Palm reserves one row;
each finger reserves Motor then Limit, two rows. Verify constraint indices and
six-word J/W reservations from the observed role order. Only the three motors
are captured; the two limit rows also write velocities during joint passes.
The fixture has no impulse joints, so the generic-one-body index equals the
actual global joint visit index. Immediate events contain the selected row
only, in aggregate builder order, once per pass. Begin/end lambda continuity
and each source-ordered full generic-V motor update are checked.

At each slice rebuild all lambdas are literal +0 and rhs equals rhs_without_bias.
Palm cap is 5000 N, c=+0, g=inv(h*h*100000+h*10000). Finger cap, c and g are +0;
the bounds are exactly [-0,+0]. Captured finger lambda may have either signed
zero. Direct finger effort is a separate external-force path. Finger RHS is
not forced to zero: joint-limit target-velocity clamping can make it nonzero.
Zero authority is reported separately and never counted as successful control
or a nonzero-RHS convergence failure. Its actual AXPY operations still run;
no unjustified bitwise-V no-op assumption is made.

Absolute RHS construction cannot be independently checked because the row-build
joint coordinates are not captured. Post-outer-step checkpoint/ECS positions
are not substituted. All numerical calculations use the actual captured RHS,
J/W, lambda, coefficients, bounds and contemporaneous generic V. Native
per-object payloads remain source-bound opaque postcard bytes, not independently
deserialized private topology or a full private simulation checkpoint.

## Strict framing and declared replay

Keep the existing literal checkpoint scope: sorted individual native bodies,
colliders, unique multibodies, impulse joints, gravity, IntegrationParameters
and ECS bindings, plus the ECS physical IEEE words. Independent primitive
postcard reconstruction checks the typed outer wrapper and parameters; the
producer's source-bound native per-object roundtrip is not relabeled as an
independent decoder. Full set metadata, private broad/narrow phase, solver,
contact caches and skipped native serde fields remain excluded.

Require every process exit 0, exactly one deterministic gzip member, CRC/footer,
UTF-8 JSONL with strict integer words and no duplicate JSON keys, sequential
steps 1..1250, exact collected terminal/counters, and complete EOF. All original
resource ceilings and original integration settings remain literal. Failed,
partial or truncated data are retained and cannot become PASS. Require every
completed checkpoint, postcard byte vector and ECS word vector equal within
ON/fresh ON/OFF; ON/fresh ON ordered stage telemetry must equal; OFF is null.
Final physical report must match the captured cube pose and exact process
stdout. Observed ECS physics scalars must be finite, with signed zero preserved.

Require exact partition/island/slice/iteration/visit framing from current source.
The fixed no-CCD fixture has one dt partition, four slices per island, one PGS
and two stabilization passes, contact warmstart 1, and zero additional final
friction passes. Before/after external-force events have rows=[]/cache=None on
every slice. Each joint update creates a new cache; post-integration events
still expose this slice's old J/W, not newly integrated dynamics. Integration,
contact-update and FinalSlice row-cache continuity are checked. All intervals,
including zeros and zero-authority fingers, are retained without cutoff.

## Independent arithmetic and scoped qualification

For each finite valid captured row, exact Fractions calculate
`r = rhs + sum(J*V) - g*lambda`, `w = sum(J*W)` and
`P_ref = lambda - clamp(lambda + r/(w+g), lower, upper)`.
Require positive common denominator. Bound complementarity is r<=0 at the
lower cap, r>=0 at the upper cap, r=0 in the interior, with zero-width bounds
explicitly zero authority. This conditional reference is distinct from actual
stored D and from measured physical motor accuracy. For this translational
fixture r is m/s, J dimensionless, W/g 1/kg, D kg, lambda/P N s. Applied motor
impulse is -lambda.

The arithmetic module is byte-identical to the reviewed historical source,
not imported from historical observations. Separately replay native f32 in
source order: nalgebra Dyn dot8/tail; rhs+dot; subtract rounded g*lambda; add
rounded D*r to lambda; native comparison clamp; delta; AXPY(-delta,W,1), with
all specified multiplications retained. RN ties to even is source/compiler and
ordinary Linux startup model; actual runtime MXCSR/rounding registers remain
unmeasured. The frozen binding documents FTZ/DAZ source/target predicate.
Qualified native replay uses only normal/zero operands and no tiny nonzero
exact intermediate, subnormal, overflow or nonfinite arithmetic. All exclusions
are counted and retained; they never become complete native replay. Literal
signed-zero words are compared. No approximate tolerance is introduced.

For contact-only aggregate boundaries require literal fixed row/cache/words,
then verify exact `delta_r=J*delta_V` and separately account for rounded native
endpoint discrepancy as the difference of endpoint rounding errors. Do not
replace it with rounded dot(delta_V). Warmstart, normal/restitution, friction,
stabilization normal/final friction intervals are separated. The two uncaptured
limit writers prevent assigning an aggregate joint-pass delta to the palm alone.
Per-contact columns, impulses, J_m W_c and individual coupling attribution are
unmeasured. No favorable numerical outcome is predicted.

PASS requires complete declared-state replay, all-role/phase coverage, every
finite-valid reference snapshot/contact interval, every native immediate visit,
projection, rebuilt diagonal and contact endpoint qualified, and zero issues.
Unavailable/excluded rows or intervals yield explicit INCOMPLETE/UNQUALIFIED,
not PASS; anomalous schema/framing gives a retained FAIL. Per-role counts/maxima
are reported separately; finger zero-authority projections cannot obscure palm
residuals. Any observed reopening is only a fixed-row aggregate stage finding,
not an individual-contact cause, validated hardware behavior or solver adoption.
Existing 133-138 physical stdout preservation remains the separate unchanged
production admission gate; this experiment cannot promote the rejected build.

## Independent checksum-only historical archive recovery

Root may prospectively bind the original public manifest, eighteen raw SHA/size
pairs, exact TAR headers/order, original archive SHA/size and recovery tool.
Compare all eighteen newly collected compressed files with those original
literal SHA/size values without numerical parsing or case selection. Only all
matches permit an attempted byte-identical historical TAR reconstruction. Keep
all attempts/mismatches; zero extra worlds, reruns, retuning or reader edits are
allowed. If any byte differs, treat the new archive separately. Even complete
identity is a reconstruction by a distinct newly registered cohort; generation
four's failed/unqualified outcome remains unchanged. Recovery is outside this
reader's numerical PASS criteria and produces no calibration/maturity gain.
