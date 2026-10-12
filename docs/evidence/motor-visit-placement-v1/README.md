# Motor visitation placement: retained evidence

Eighteen fresh Rapier ForceBased isolated-motor processes completed a new fixed
study: three public PGS/stabilization profiles, two target signs and
ON/fresh ON/OFF observations. All original native-process, coordinator, checked
CLI and root collection exits were zero. The one frozen scientific reader exited
zero with **384 constructor, 384 coordinate-advance and 896 motor-visit checks**,
2,572 construction-companion events and 8,192 stage-v1 events, without arithmetic
exclusions or qualification issues. Captured initial/final ON/fresh/OFF state
matched exactly; fresh-ON completed records matched literally; OFF had all three
observers absent. Independent retention and display checks passed.

The preregistered **first-slice discriminator is INCONCLUSIVE for both signs**.
Clipping and odd-cycle equality were explicitly allowed before data; unequal
coordinates were never an acceptance requirement. This does not imply equality
through all 32 slices. The last slice of the 1 ms interval records the following
positive-target values (negative-target displays are retained in the summary):

| PGS / stabilization passes | q after interval (rad) | V used for last position advance (rad/s) | V written back (rad/s) |
|---|---:|---:|---:|
| 1 / 2 | 2.43952054e-05 | 0.0243901238 | 0.0312500075 |
| 1 / 0 | 2.43952054e-05 | 0.0243901238 | 0.0243901238 |
| 3 / 0 | 2.34375093e-05 | 0.0312500075 | 0.0312500075 |

In this fixture, removing the two postintegration stabilization visits preserves
the captured final coordinate and makes writeback V equal the integration input.
Moving all three visits before integration changes the captured final coordinate.
These later observations are diagnostic only. Geometric backward-Euler SI
references remain separate from source-fidelity checks; this study selects no
profile, solver normalization, accuracy threshold or production setting.

The existing example-141 calibration remains **FAIL 0/4**, with motionless zero
targets **8/8**. The inverse-(w+g) intervention remains rejected. This new study
does not rerun, extend or reinterpret any earlier six/72/eighteen-world cohort.

## Whole data and original exits

All eighteen original gzip streams are retained whole, in fixed launch order.

| Case | Whole gzip | Original exit |
|---|---|---|
| split_1_2_negative_on | [raw](f/000129.jsonl.gz) | [0](f/000128.json) |
| split_1_2_negative_fresh | [raw](f/000115.jsonl.gz) | [0](f/000114.json) |
| split_1_2_negative_off | [raw](f/000122.jsonl.gz) | [0](f/000121.json) |
| split_1_2_positive_on | [raw](f/000150.jsonl.gz) | [0](f/000149.json) |
| split_1_2_positive_fresh | [raw](f/000136.jsonl.gz) | [0](f/000135.json) |
| split_1_2_positive_off | [raw](f/000143.jsonl.gz) | [0](f/000142.json) |
| early_1_0_negative_on | [raw](f/000043.jsonl.gz) | [0](f/000042.json) |
| early_1_0_negative_fresh | [raw](f/000028.jsonl.gz) | [0](f/000027.json) |
| early_1_0_negative_off | [raw](f/000036.jsonl.gz) | [0](f/000035.json) |
| early_1_0_positive_on | [raw](f/000064.jsonl.gz) | [0](f/000063.json) |
| early_1_0_positive_fresh | [raw](f/000050.jsonl.gz) | [0](f/000049.json) |
| early_1_0_positive_off | [raw](f/000057.jsonl.gz) | [0](f/000056.json) |
| early_3_0_negative_on | [raw](f/000085.jsonl.gz) | [0](f/000084.json) |
| early_3_0_negative_fresh | [raw](f/000071.jsonl.gz) | [0](f/000070.json) |
| early_3_0_negative_off | [raw](f/000078.jsonl.gz) | [0](f/000077.json) |
| early_3_0_positive_on | [raw](f/000106.jsonl.gz) | [0](f/000105.json) |
| early_3_0_positive_fresh | [raw](f/000092.jsonl.gz) | [0](f/000091.json) |
| early_3_0_positive_off | [raw](f/000099.jsonl.gz) | [0](f/000098.json) |

## Protocol, audit and source bindings

| Record | Whole retained files |
|---|---|
| Fixed protocol and source/binary registration | [protocol](f/000218.json), [candidate](f/000155.json), [admission](f/000158.json) |
| Actual compilation, features and selected inputs | [build](f/000190.json), [toolchain](f/000208.json), [sources](f/000346.json) |
| Native collection and original outer exits | [coordinator](f/000023.json), [checked CLI](f/000017.json), [reader0](f/000010.json) |
| Complete scientific audit and display | [audit gzip](f/000008.json.gz), [summary](f/000157.json), [compression binding](f/000367.json) |
| Independent retention/display/compression checks | [retention](f/000179.json), [display](f/000182.json), [compression](f/000188.json) |
| Original local bundle and byte qualification | [prepared manifest](f/000396.json), [review](f/000398.json) |
| Frozen actual reader and pure-test receipts | [reader](f/000369.py), [schema](f/000381.py), [pure27](f/000379.json) |
| Collector source and independent producer review | [collector](f/000215.rs), [review](f/000004.json) |
| Inactive predata preparation failures | [coverage](f/000458.json), [blocked source review](f/000400.json), [superseded pure review](f/000401.json), [dependency correction](f/000402.json) |
| Portable verifier and byte-only fault tests | [tool source](f/000460.py), [test source](f/000461.py), [31-test and actual-copy checks](f/000467.json) |

The original audit is one 5,447,072-byte JSON file, SHA-256
`d7022bb982ab997862da1f3aabbfa4306a6fa568b4707a2b2a9aa0bd9581acfe`.
Its entire lossless one-member gzip representation is 886,291 bytes, SHA-256
`4611f3fbeb711e83c749b986b71081c6297b7fd4e4f196b8ea18c4eea3d6c2c0`.
Decompression restores every original byte, including all rows, rational digits
and whitespace. No stream or audit was split, filtered, truncated or reformatted.
Each stored payload satisfies ADR 034's 1 MiB per-file threshold.

## Portable byte verification

From the repository root:

```sh
python3 docs/evidence/motor-visit-placement-v1/verify_literals.py \
  --root docs/evidence/motor-visit-placement-v1
```

To recover the original complete audit, decompress the whole retained file:

```sh
gzip -dc docs/evidence/motor-visit-placement-v1/f/000008.json.gz > /tmp/motor-visit-placement-audit.json
sha256sum /tmp/motor-visit-placement-audit.json
```

The portable verifier checks whole stored hashes/sizes, all origin mappings and
the exact gzip-restored audit hash/size. It does not authenticate the manifest
independently, execute archived scientific readers, reconstruct worlds or infer
scientific/CI success. [manifest.json](manifest.json) maps historical original
paths to whole payloads. Identical whole bytes may share a payload; original
absolute paths are metadata, not extraction destinations. Two local-only Python
and compiled-native executable bindings are recorded separately; their hashes do
not make the executable bytes available in this bundle. One inactive unadmitted
preparation executable also remains local, with its descriptor and zero-world
status retained in the preparation-coverage record.

## Interpretation and reproduction limits

The unchanged plant is a contact-free uniform sphere (radius 0.1 m, mass 1 kg)
on coincident identity fixed-parent X-revolute frames. ForceBased gains are
300 Nm/rad and 12 Nm s/rad, the ceiling is 2 Nm and targets are ±0.001 rad.
Gravity, passive damping, armature and initial q/V are zero; sleep and CCD are
disabled. Each fresh process performs one 1 ms interval with 32 small steps;
only the two public iteration counts differ. Full numerical words and captured
individual-object postcard bytes remain intact.

The measured source copy derives from
`5e8e3627cd91df767ae41e8364f9401a8cd4e2d7`; all 123 vendored files were restored
to byte-identical baseline source before data, and all 129 selected build inputs
matched before/after the actual build. Rapier features are default/dim3/f32/
serde-serialize, with no requested parallel or SIMD feature. The dependency
versions/checksums match the repository lock except the external collector's
package identity. Retained predata failures document the earlier dependency
update, dependency formatting and ordered-input lookup corrections; those
preparations are not the admitted build or successful scientific audit.

Runtime MXCSR/rounding/FTZ/DAZ, complete compiler/registry closure, full-set
metadata, private pipeline/solver/contact caches, mass-matrix/LU construction,
external-acceleration inputs and physical hardware remain outside the admitted
scope. Source-bound postcard roundtrip is not an independent private-state
decoder. Historical scripts preserve their original paths and once-only claims;
they are not current-checkout runnable examples. The source/binary record is
nonhermetic and does not guarantee a cross-machine identical rebuild.

These diagnostic consistency results establish no accuracy/convergence
improvement, coupled-contact cause, hardware or external-user validation,
candidate adoption or practical maturity gain.
