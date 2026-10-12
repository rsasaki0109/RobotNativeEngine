# Generation-5 durable once-only collection

These helpers prepare a **new independent cohort**. They do not restore or
reinterpret the old generation-4 collection, lost TAR, science result, or release.
The root agent owns source/build/binary freeze, rollback, independent science
review, and any separately preregistered recovery arm. These helpers neither build
nor publish anything.

## Fixed process order and roles

There are exactly 18 reserved cases, indexed 0 through 17. Build order is
`baseline`, then `rejected_inverse_w_plus_g`; each build runs masses 5, 20, 50 kg;
each mass runs `on`, `fresh_on`, `off`. Labels retain the collector's existing
`baseline-m5-on` form. `fresh_on` is a fresh process with `--observer on`.

The only native command is:

```text
FROZEN_BINARY --mass-kg MASS --observer on|off --label LABEL --output ABSOLUTE_NEW_RAW_PATH
```

`collect_once.py` checks the frozen spec and source/binary/reader/protocol inputs
before creating a durable cohort claim. It creates the cohort directory
exclusively; any existing directory blocks another invocation. There is no retry,
resume, cleanup, replace, or delete command. A crash after reserving a launch leaves
that case indeterminate and blocks all further execution of this cohort.

Each case retains:

- `source-before.json` and `source-after.json`: streaming SHA of the fixed sorted
  source map. This map is a **non-exhaustive compiler/dependency closure**.
- `launch-claim.json`: fsynced reservation before Popen. Reservation does not claim
  that a process started.
- `process-started.json`: actual PID and argv after Popen succeeds.
- `process-exit.json`: actual signed Popen returncode, fsynced before later hashing.
  A negative signal returncode is preserved without substitution.
- `stdout.log`, `stderr.log`, and the collector's unchanged exclusive-created gzip.
- `case-terminal.json`: exit/log/raw/guard descriptors and infrastructure errors.

Immutable index snapshots distinguish reserved, actually started, and exit-zero
process counts. Final `collection-index.json` uses schema 2, includes the generation
and cohort identity, and preserves all 18 entries including unattempted entries.
`collection-terminal.json` is the driver's result; original process exits remain in
their own receipts. SIGINT/SIGTERM requests stop new launches after the current
original process completes; the driver does not kill or restart that process.

A nonzero original process exit or missing raw output is retained and does not
filter the next fixed case. A changed frozen artifact/source input, spawn failure,
or infrastructure error stops new launches. No helper reads numerical fields to
select traces or tune parameters. Process completion is not a claim of 1250 valid
physical steps; the index declares the fixed requested steps, and independent
reader qualification remains separate.

## Frozen spec supplied by root

Create a new persistent spec from `spec-template.json`; do not edit the template
into a frozen record. Every descriptor is exactly `path`, `bytes`, `sha256`, and
every path is canonical under `/workspace`. Spec, protocol, readers, gate receipts,
bindings and binary copies must reside under generation-5. Source paths remain in
the repository. Driver modules must reside under this `driver` directory.

Required status is `FROZEN_BEFORE_GENERATION5_COLLECTION`, generation is 5,
`world_limit` is 18, and `authoritative_worlds_of_this_cohort_before_freeze` is 0.
The spec pins `protocol`, `reader_artifacts`, `reader_pure_test_receipt`,
`independent_readonly_review`, `source_guard_manifest`, and the three production
`driver_artifacts`. The two `bindings` descriptors point to records with:

```json
{
  "status": "FROZEN_BEFORE_GENERATION5_COHORT_WORLDS",
  "cohort_id": "ROOT_CHOOSES_NEW_GENERATION5_ID",
  "build": "baseline",
  "protocol_sha256": "ACTUAL_NEW_PROTOCOL_SHA",
  "authoritative_worlds_of_this_cohort_before_freeze": 0,
  "binary": {"path": "PERSISTENT_BINARY", "bytes": 0, "sha256": "ACTUAL_SHA"},
  "source_manifest": {"path": "PERSISTENT_BUILD_SOURCE_MANIFEST", "bytes": 0, "sha256": "ACTUAL_SHA"}
}
```

Both build bindings may retain additional build/source provenance. Zero is scoped
to this cohort and does not erase the earlier 18 or 90 worlds. The protocol must
pin the same literal `fixed_cells` and `authoritative_coupled_worlds: 18`. Reader
and review receipts are hash-bound; their independent validity is owned by root.
The driver requires the spec's `preflight_gates` to declare both PASS before launch.

The source guard is an object with a `scope` containing `non-exhaustive` and a
sorted `files` map of safe repository-relative paths to SHA256. Root registers
tracked Rust/Cargo/lock/config inputs and all vendored Rust/TOML inputs. The driver
hashes exactly that registered set; it does not silently widen the set or claim a
complete compiler closure. Additional source-manifest metadata is retained in the
pinned original artifact, but the per-case guard consists of `scope` and `files`.

Root may then run, once:

```text
python3 -B driver/collect_once.py run --spec ABSOLUTE_SPEC --spec-sha256 ACTUAL_SHA
```

Read-only inspection is available even after a partial cohort:

```text
python3 -B driver/collect_once.py inspect --cohort-root ABSOLUTE_COHORT_ROOT
```

Inspection never repairs a torn receipt or launches a world. An incomplete JSON
receipt is reported as incomplete, not as a verified terminal exit.

## Local archive preparation only

Once every fixed process has a durable terminal receipt and unchanged provenance:

```text
python3 -B driver/prepare_archive.py --spec ABSOLUTE_SPEC --spec-sha256 ACTUAL_SHA
```

The archive has a permanent exclusive preparation claim. It contains all 18 gzip
files in fixed case order and `literal-member-manifest.json` as the 19th member.
The manifest preserves size, compressed SHA, case identity and original exit; it
does not admit a numeric result. Gzip trailers and the complete uncompressed stream
hashes are checked without parsing or filtering numerical data.

USTAR headers are explicit: mtime 0, uid/gid 0, empty user/group names, regular-file
type and mode 0444. The first TAR is durably saved under
`archive-preparation/contact-motor-stages-generation5-raw.tar`. Every source stream
and saved member has size/SHA verified. A second regenerated stream is compared
byte-for-byte against the saved TAR while also independently computing its SHA and
size; a second large TAR is not stored. Original gzips, logs, partial outputs and
TAR are never deleted or overwritten.

Only `proposed-archive-pointer.json` with status
`PROPOSED_LOCAL_ARCHIVE_NOT_PUBLISHED` is generated. It has no URL/tag and makes no
claim to restore the old TAR. A nonzero collection outcome may still be archived
if all 18 raw containers and provenance qualify; its failed collection status and
original exit codes remain visible. Any truncated/corrupt gzip, missing case,
changed input/member, or second-stream mismatch blocks archive qualification.

The root wrapper's actual exit and a parseable archive terminal are required for
adoption. If a terminal write itself tears, its original bytes are retained and a
separate `archive-failure.json` is appended when storage permits. A previously
verified PROPOSED pointer can remain present after such an infrastructure failure;
that presence is not a successful wrapper outcome or publication.

## Synthetic tests

`test_durable_driver.py` runs only explicitly authored Python mock executables that
write marked synthetic gzips. It executes no collector, robot world, build, or
network action. Cases cover once-only claims/concurrency, unconfirmed/torn state,
operator stop, source/binary/matrix/freeze mismatch, preserved nonzero/signal exits,
invalid gzip, synthetic ENOSPC, byte mismatch and torn archive terminal. All mock
artifacts and test logs remain under `driver/synthetic-tests/`; no cleanup runs.

```text
python3 -B driver/test_durable_driver.py
```
