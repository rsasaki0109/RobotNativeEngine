# Publishing to crates.io

Status: **nothing is published to crates.io yet.** The workspace is ready for
it, and a dry run of every publishable crate passes. Publishing is a manual,
irreversible step (see [Publishing](#publishing)).

The Python package is separate: `rne_py` ships to PyPI through
`.github/workflows/pypi.yml`.

## What gets published

Every crate without `publish = false`, except `rne_py`: 35 crates, from
`rne_math` and `rne_core` up to `rne_hardware_gateway` and
`rne_adapter_ros2`. Examples, tests and tools set `publish = false`.

`rne_py` stays in the frozen publishable set that `xtask` checks
(`publishable package set differs`) and in the `cargo-semver-checks` matrix, so
its manifest is left alone. It is excluded when publishing instead
(`--exclude rne_py`): a PyO3 `cdylib` is of no use as a crates.io dependency.

All `rne_*` dependencies are declared in the root `Cargo.toml` as
`{ version = "=0.4.0", path = ... }`, so the packaged crates resolve each other
on crates.io at exactly the workspace version. Bumping the version therefore
means changing the workspace version and every `=x.y.z` pin together.

## Rapier patch

The repository patches `rapier3d` with a local copy
(`third_party/rapier3d`, see `RNE_PATCH.md` there) through
`[patch.crates-io]`. `cargo publish` ignores `[patch]`, so a crate installed
from crates.io builds against **unmodified upstream Rapier 0.22**.

- Default builds are written to work with upstream Rapier.
- The `rne_physics_rapier` features `experimental-armature` and
  `motor-stage-diagnostics` need the patched copy. They do not work from a
  crates.io install; use a git checkout of this repository for them.

`wgpu-3dgs-viewer` is patched the same way, but only `rne_render_3dgs`
(`publish = false`) depends on it.

## Dry run

```bash
cargo publish --workspace --exclude rne_py --dry-run --locked
```

This packages each crate, orders them by dependency, and builds every packaged
crate against the others, without uploading. CI runs it as the
`crates.io dry run` workflow (manually, and on pull requests that touch a
`Cargo.toml`).

## Publishing

Not automated, on purpose. A published version can only be yanked, never
replaced or deleted, and the crate names become taken.

1. Check the names are free (`curl -s -o /dev/null -w '%{http_code}\n' https://crates.io/api/v1/crates/<name>`
   returns `404` for a free name) and that you own the crates.io account that
   will publish.
2. Run the dry run above on the release commit.
3. `cargo login`, then publish in one command:

   ```bash
   cargo publish --workspace --exclude rne_py --locked
   ```

   crates.io rate-limits new crates (a short burst, then one every several
   minutes), so all 35 can take hours. If it stops part-way, check which crates
   reached crates.io and publish the remaining ones with `-p <name>` (a
   published version cannot be uploaded again).
4. Tag the release and publish `rne_py` to PyPI as usual.

Consider adding the crates.io Trusted Publishing flow to a workflow once the
first manual publish has created the crates (it is configured per existing
crate).
