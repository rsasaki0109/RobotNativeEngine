# Complete generation-5 derived evidence retention

This helper stores the closed derived evidence and its completed provenance as a
new local archive. It executes no collector, world, build, scientific reader or
network operation. The frozen generation-5 driver, readers, protocols, bindings,
sources and original outputs are read only. Existing historical scientific
`FAILED_UNQUALIFIED` records remain literal and unchanged. The helper does not
interpret the new scientific result or turn archival integrity into adoption.

Before actual scanning/packing, root must explicitly release both the original
scientific auditor terminal and the final independent scientific review. The
release record's status must be `ROOT_RELEASED_AFTER_ORIGINAL_AUDIT_TERMINAL`; it
pins the actual terminal, final audit, completed independent review, collection
index, old authorized raw TAR, old raw manifest and retention review. The actual
terminal must contain its original integer exit, closed log descriptors, end time,
unchanged-input guard and `rerun_or_retry: false`. Any original terminal exit is
retained; archive integrity is separate from a scientific success criterion.

The release record must explicitly set `new_archive_publication_authorized: false`.
The old authorized raw TAR is still **LOCAL_NOT_PUBLIC**; its exact authorized
identity is 97,996,800 bytes / SHA256
`534e38498ccbbcba99bf1e1b26a77f37469d8c5596429cbf0dd13b5a55d329b0`.
Its earlier authorization does not authorize the new derived archive's bytes.

## Complete input boundary

After release, every completed regular file under generation-5 is enumerated by
its original relative ASCII path. Required inputs include all six **whole** detail
gzip files, the complete final audit, five reader modules, profile, source copies,
fixtures, pure tests, frozen protocol/bindings, all actual collection logs/receipts,
source guards, recovery receipts/reviews and the final independent scientific
review with its ledgers. No scientific file is selected by a numeric outcome;
payloads are opaque bytes and are never decompressed, split or truncated here.

The only exclusions are recorded with their complete path/size/SHA descriptor and
reason:

- Local native ELF binaries under `binaries/`; their original binding records
  remain included.
- The 18 files under `collection/raw/`, each mapped to its whole member in the
  exact old authorized local TAR. All full raw descriptors remain in the manifest.
- That actual old TAR file, referenced by exact descriptor rather than duplicated.
- This entire `derived-archive-preparation/` directory, including tooling and
  archive-generation/publication-verification artifacts. Files present here at
  freeze have full exclusion descriptors; newly created output artifacts receive
  descriptors in their separate verification/terminal records.

An exact literal copy of the small old raw member manifest is also archived as an
explicit external-reference file. It is read from its immutable original path.
The new archive does not claim that raw evidence is publicly downloadable.

This freezes the completed provenance **before** generating archive checksums and
later publication proofs. Those later proofs are outside the archive boundary to
avoid a circular checksum. The entire non-preparation file set and all frozen
included/excluded file bytes are checked again after packing. Root must therefore
avoid any generation-5 writes outside this preparation directory during packing.

## Canonical whole-file mapping

Original relative paths can exceed a TAR name field. The archive enumerates them
in relative ASCII order and maps each complete file one-to-one to short names
`artifacts/000000`, `artifacts/000001`, etc. `literal-member-manifest.json` records
the original relative/absolute path, complete size/SHA and archive name for every
file. This is whole-file mapping, not splitting, truncation or flattening without
provenance. The manifest is the last member; all archive names are ASCII-sorted.

The format is PAX with explicitly empty per-file/global PAX headers. Short names
avoid automatic `path` extensions. Every member is regular, mode 0444, uid/gid 0,
mtime 0, with empty user/group names. A file requiring a size extension fails the
strict header contract; no alternative encoding is silently substituted.

All input/member streams are SHA/size verified before and after consumption.
The saved first TAR's complete members, headers, order, SHA and sizes are checked.
A separately regenerated second stream is compared **byte-for-byte** against the
first TAR while calculating its own full SHA and size. No second large TAR is
saved. All files use exclusive creation, file fsync and parent-directory fsync.
The output directory is a permanent once-only claim; partial TARs and torn
receipts remain at their original paths. Nothing is deleted or overwritten.

Successful preparation creates only a local pointer with status
`PROPOSED_NEW_DERIVED_BYTES_FRESH_USER_AUTHORIZATION_PENDING`, no URL or tag and
`publication_authorized: false`. It is not the final ADR 034 HTTPS pointer schema.
Root must obtain fresh authorization before publishing these new bytes, then
verify actual download SHA/size and create a final pointer separately.

## Root release record

Create a new record under this preparation directory with these fields:

```text
status: ROOT_RELEASED_AFTER_ORIGINAL_AUDIT_TERMINAL
synthetic_only: false
new_archive_publication_authorized: false
cohort_root: absolute generation-5 root
cohort_id: actual new collection identity
excluded_preparation_directory: generation-5/derived-archive-preparation
output_directory: a new child such as derived-archive-preparation/actual-derived-archive
audit_terminal: full descriptor of original actual auditor terminal
collection_index: full descriptor of new 18-case index
final_audit: full descriptor of closed analysis/independent-numerical-audit.json
completed_scientific_review: full descriptor under postcollection-scientific-review
old_authorized_raw_archive: full descriptor of exact local old TAR
old_raw_member_manifest: full descriptor of original immutable small raw manifest
old_tar_retention_review: full descriptor of independent exact-old-TAR review
helper_artifacts: complete descriptors including prepare_derived_archive.py
```

Each descriptor has exactly `path`, `bytes`, `sha256`. All paths are canonical
under persistent `/workspace`. A small synthetic test cohort is accepted only
under this directory's `synthetic-tests/` namespace with `synthetic_only: true`.
Actual cohort identity cannot use that exception.

```text
python3 -B prepare_derived_archive.py --release ABSOLUTE_RELEASE --release-sha256 SHA
```

`test_derived_archive.py` uses only small marked synthetic fixtures. It checks
strict pre-release gating, whole six-file retention, long-path mapping/empty PAX
headers, complete scope, byte identity, post-freeze file-set drift, duplicate
attempt refusal and synthetic ENOSPC retention. It reads no actual audit/detail
file and retains all test artifacts and failures.
