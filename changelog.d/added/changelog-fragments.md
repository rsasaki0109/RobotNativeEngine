- Changelog fragments: entries go in `changelog.d/<section>/<name>.md` instead of
  `CHANGELOG.md`'s `[Unreleased]` section, so parallel branches no longer conflict
  on it. `cargo run -p xtask -- changelog` previews them, `--check` validates them
  (run by `ci-lint`), and `--fold` / `--release VERSION --date DATE` move them into
  `CHANGELOG.md`.
