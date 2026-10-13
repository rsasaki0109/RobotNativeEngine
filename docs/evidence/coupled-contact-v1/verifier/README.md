# Coupled-contact whole-archive byte verification

This packet contains exact copies of the source-qualified v3 archive tool and
its 30 synthetic tests. The portable operation is **verify --no-originals**:
historical absolute paths are inventory labels, never files to read, extraction
destinations or programs to execute. The verifier streams the archive and does
not execute archived helpers, native worlds or scientific readers. It does not
parse the numerical records contained in original payloads.

The tested runtime is Python3.13.5 on POSIX/Linux; Python standard-library modules
are sufficient. No repository checkout or historical source directories are
needed for portable verification. Do not invoke create or omit --no-originals
for this portable operation. The archive is an external artifact: this small
packet does not contain its1,317,548,213 bytes and does not claim public download
availability. New Release publication remains subject to separate approval.

Archive identity:

```text
coupled-contact-v1-b9a6c853.tar.gz
bytes: 1317548213
SHA-256: 4dd915ab6cb549cce86241f36e7782743623396c603182b12749ee4050cc998a
```

After obtaining the complete archive, run the exact portable command below.
Supply an absolute local path for the downloaded archive and a new absolute
output receipt outside this packet. An incomplete/corrupt archive must fail;
retain its original failure rather than replacing it with a partial excerpt.

```text
python3.13 -B /absolute/path/to/whole_archive.py verify --archive /absolute/path/to/coupled-contact-v1-b9a6c853.tar.gz --create-receipt /absolute/path/to/create-v3-receipt.json --output /absolute/path/to/new-byte-verification.json --no-originals
```

The verifier checks the full internal inventory; deterministic portable aliases
and exact original-path bindings; literal regular/symlink tar/PAX headers; every
whole regular member's length/SHA; exact symlink metadata and target strings;
all padding/end framing; one complete gzip CRC/EOF with no trailing members;
and the entire stored archive's SHA/length. It never follows or extracts a
symlink. Explicit local-only executable metadata does not provide those absent
executable bytes. The actual completed archive has19,447 declared original
objects, including32 symlinks, and33 local-only executable bindings. Its initial
generated inventory is a separate tar member.

Success is `PASS_WHOLE_LITERAL_ARCHIVE_BYTES_ONLY` with
`originals_rechecked:false`, `public_availability_verified:false`,
`scientific_reader_invocations:0` and `numerical_analysis:false`. This proves
agreement with the declared whole archival bytes. It does not establish physical
accuracy, a production solver decision, runtime floating-point mode, hardware
validation, absent executable recovery, scientific provenance beyond the
recorded bindings, or that an external URL currently serves the file. A public
download must separately match the complete archive SHA/length.

The packet preserves the original exit/stdout/stderr and before/after source
guards for one actual portable CLI run on the completed archive and for all30
synthetic tests on exact source copies. Tests cover Case/case whole byte retention,
alias/origin errors, symlink non-dereference and metadata mutation, executable
fixture distinctions, CRC/truncation/header/member tampering and capacity refusal.
Source identities and the prior independent review are included without rewriting
their historical paths. Generated test fixtures remain outside the public
payload. Whole qualification receipts, stdout and stderr are copied here.

To rerun synthetic tests, copy the three.py sources to a writable scratch
directory and run `python3.13 -B qualify_mock.py NEW_ATTEMPT` there. The qualifier
creates clearly marked synthetic fixtures and records its original unittest
exit and source guards. These tests do not need or read the archive or its
historical original directories.

Every file in this packet is at most1MiB. The archive itself exceeds ADR034's
stored-file threshold and remains external. No filtering, sampling, arbitrary
splitting, gzip repair, normalized JSON or reencoded scientific payload replaces
any whole original file.
