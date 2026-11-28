# Changelog fragments

Do not edit `CHANGELOG.md`'s `[Unreleased]` section by hand. Add one file per change
here instead. A new file never conflicts with another branch's new file, so parallel
branches stop conflicting on `CHANGELOG.md` every time one of them merges.

- Path: `changelog.d/<section>/<name>.md`. `<section>` is one of `added`, `changed`,
  `deprecated`, `removed`, `fixed` or `security`. `<name>` is anything unique, such
  as the branch or the feature (`lidarloc-remapping.md`).
- Body: one or more Markdown bullets (`- ...`), wrapped as in `CHANGELOG.md`, with no
  headings. The directory names the section.

Commands:

```bash
cargo run -p xtask -- changelog            # preview the fragments as CHANGELOG sections
cargo run -p xtask -- changelog --check    # validate them (ci-lint runs this)
cargo run -p xtask -- changelog --fold     # move them into [Unreleased]
cargo run -p xtask -- changelog --release 0.5.0 --date 2026-12-01
```

`--fold` and `--release` delete the fragments they move. `--release` also renames
`[Unreleased]` to the release and opens an empty `[Unreleased]` above it.
