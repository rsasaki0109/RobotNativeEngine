# Required release133–138 preservation gate

Prepared before generation4 collection. Historical baseline and tools are unchanged. The current runner differs from `diagonal-intervention-generation-3/run_restored_default_release_regressions.py` only in its OUT assignment; all package/example pairs, Cargo argv, actual exit capture, stdout/stderr capture and literal comparison remain identical.

The unchanged timing exclusions are exactly:

1. `,\s*[0-9.]+ us per step` → empty text.
2. `\d+ env steps/s \(\d+ sim steps/s\)` → `<throughput>`.
3. `parallel speedup [0-9.]+x` → `parallel speedup <timing>`.

Require all six child exits 0, every actual baseline `: ok` line, and complete original stdout equality after only these substitutions. No new numerical tolerance, rebaselining, physical-line omission, or reinterpretation of rounded precision is allowed. The reader retains full original stdout/stderr hashes, process receipts/argv, wrapper logs and literal differences. It also checks the final selected source freeze and the unchanged before/after wrapper inputs. Neither source snapshot claims exhaustive compiler closure.

All six baseline transcripts were checked read-only. `preparation-source-freeze.json` is explicitly preparation-only and cannot authorize final verification. No Cargo or new release example was run here. This mandatory regression collection is separate from the planned18 coupled and72 scalar authoritative worlds; its contribution to those preregistered counts is zero. The examples may internally construct multiple regression scenes, so six example processes are not claimed to equal six simulation worlds.

## Root-only future command template

Do not execute until root authorizes the final restored-default collection, after temporary diagnostic changes are restored. The shell activation and these commands below are a template only; none was executed during preparation.

```bash
. /workspace/.rne-onboarding/env.sh
export CARGO_BUILD_JOBS=2
export RUST_TEST_THREADS=2
rne_release_gate='/workspace/.rne-kick-demo/single-joint-motor-calibration/next-experiment/generation-4/current-stage-qualification/release-regressions'
rne_release_tool_freeze_sha='27812c1652abc34d91e9c52be9e907ef3d9aeda6a7337b253b939e91be7cf026'
python3 "$rne_release_gate/freeze_source.py" --tool-freeze-sha "$rne_release_tool_freeze_sha" --phase final-restored-default --output "$rne_release_gate/final-source-freeze.json"
cd /workspace/RobotNativeEngine
python3 /workspace/.rne-kick-demo/single-joint-motor-calibration/run_checked.py --name gen4-current-stage-restored-default-release133-138 -- python3 "$rne_release_gate/run_release_regressions.py"
python3 "$rne_release_gate/verify_release_regressions.py" --tool-freeze-sha "$rne_release_tool_freeze_sha" --source-freeze "$rne_release_gate/final-source-freeze.json" --execution-check /workspace/.rne-kick-demo/single-joint-motor-calibration/checks/gen4-current-stage-restored-default-release133-138 --output "$rne_release_gate/actual-verification.json"
```

Paths are exclusive-create: prior final source/result/check/audit outputs are never overwritten. Preserve actual runner or wrapper failures; do not rerun the same output paths. If the runner exits1, run the read-only verifier separately against the failed receipt to preserve failure evidence; it returns1 and never changes the baseline. Partial completed rows cannot pass the six-example gate.

Each underlying command remains:

```text
cargo run --locked --release -p <frozen-package> --example <frozen-example>
```

Preparation validation executed only Python syntax compilation, source-file hashing without Git, and baseline transcript reading. Full current release collection, final restored source binding, build and CI qualification remain unexecuted here.
