# Static heavy-grasp backend comparison

Example 138 can emit structured results from the **same ECS scene and
`PhysicsBackend` loop** on native, Rapier, and the optional MuJoCo adapter.
This is a bounded diagnostic of the existing adapters with unchanged settings,
not a comparison of each engine's best achievable grasp or steady-state speed.
The ordinary example still runs without MuJoCo and retains its native checks.

The saved report is historical evidence for the implementation merged in
`5370520e03705f8c6ae3e7b5bfd5ee740fe13495` (#394). Use that revision to
reproduce its physical results. Later ECS MuJoCo typed feedback uses bounded
affine actuators, so a fresh report from current main can differ. The original
report is retained rather than rewritten; see [ADR 016](architecture/016_contact_dynamics.md)
for the control change and its unresolved heavy-grasp limitations.

## Reproduce (Linux x86_64)

The RNE MuJoCo bindings require the 3.9 ABI. This measurement uses the pinned
3.9.0 Python wheel's real native library, not the latest MuJoCo release.
Python is only the benchmark orchestrator; all simulation runs in the Rust
binary. Do not load another ABI with these bindings.

```bash
python3 -m venv target/heavy-grasp-venv
target/heavy-grasp-venv/bin/pip install mujoco==3.9.0
MJC_PACKAGE_DIR=$(target/heavy-grasp-venv/bin/python -c 'import pathlib, mujoco; print(pathlib.Path(mujoco.__file__).parent)')
mkdir -p target/heavy-grasp-runtime
ln -sf "$MJC_PACKAGE_DIR/libmujoco.so.3.9.0" target/heavy-grasp-runtime/libmujoco.so
ln -sf "$MJC_PACKAGE_DIR/libmujoco.so.3.9.0" target/heavy-grasp-runtime/libmujoco.so.3.9.0
export MUJOCO_DYNAMIC_LINK_DIR="$PWD/target/heavy-grasp-runtime"
export LD_LIBRARY_PATH="$MUJOCO_DYNAMIC_LINK_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cargo build --locked --release -p heavy_grasp --example 138_heavy_grasp --features mujoco
python3 scripts/compare_heavy_grasp.py --runs 5 \
  --runtime-library "$MUJOCO_DYNAMIC_LINK_DIR/libmujoco.so.3.9.0" \
  --output target/heavy-grasp-comparison.json
```

For official runtime archives and Windows loader setup, see
[the MuJoCo spike](PLAN_MUJOCO_SPIKE.md). The comparison script uses only Python's
standard library; wheels and runtime binaries are not committed. Use a fresh
release build rather than pointing at an old binary. The report records binary,
shared-library, lockfile, source and diff hashes, compiler, OS, CPU, and affinity.
The runtime-library argument must identify the library actually selected by the
platform loader. Its hash records provenance; it does not verify loader selection.

## Conditions and interpretation

- SI units, Y-up, gravity 9.81 m/s², 500 Hz, 1,250 steps per case.
- Cubes: 0.1 m per side, masses 5/20/50 kg. A 1 kg palm and two 0.2 kg
  pads share the original collider-derived cuboid inertia and joint geometry.
- Friction is the original `f32` 0.8 material on every collider.
- Each finger applies 1.25 or 0.65 times `mass * gravity / (2 * friction)`.
  The lift target ramps to 0.2 m over 1 s and holds for 1 s after 0.5 s settling.
- Lift position command: stiffness 1e5 N/m, damping 1e4 N s/m, force limit
  5,000 N. Joint coordinate zero is the spawn pose; Rapier uses MultibodyLink.
- Strict held criteria: cube/palm rise difference below 1 mm, palm rise above
  0.19 m, final relative slip below 0.1 mm, final tilt below 0.001 rad.
  Under-squeezed criterion: final cube rise magnitude below 1 mm. These are
  the existing native regression thresholds, now also reported for other backends.
- `PhysicsWorldDesc::default()` selects each adapter's defaults. Native uses
  the exact cone and normal regularization 1e-3 described in ADR 016. Rapier
  retains its default iterative contact/motor integration. MuJoCo uses the ECS
  compiler's implicitfast integrator with default contact softness, pyramidal
  cone, and solver defaults; the compiler does not override solref/solimp.
  Its typed PD actuator is evaluated/clamped explicitly before the motor step,
  unlike native implicit PD. Equal gains do **not** imply equal discrete dynamics.
- One discarded warm-up per backend; five retained trials, reversing backend
  order on alternating trials, one process at a time on the same host.
- Timing includes the first ECS synchronization/model compilation, all steps,
  ECS writeback and lift-command updates; it excludes ECS scene construction.
  It is setup-inclusive end-to-end time, not a solver microbenchmark. Failed
  tasks remain visible; their timings are not successful-task speed comparisons.
- Exact repeatability here compares **final JSON physical outputs**, excluding
  timing. It does not cover a full trajectory, other CPUs or compilers.

## Recorded cloud run (2026-10-09)

[Raw report](evidence/heavy-grasp-comparison/report.json) retains every trial,
including failures. Timing is from a shared Linux cloud host without CPU pinning;
the report records its actual CPU/OS/compiler and allowed affinity. It is not an
external independent reproduction or a universal performance ranking.

| Backend | Mass kg | Case | Strict threshold | Median µs/step | Min–max µs/step |
|---|---:|---|---|---:|---:|
| native | 5 | held | pass | 210.03 | 204.21–220.20 |
| native | 5 | under-squeezed | pass | 140.21 | 136.93–152.04 |
| native | 20 | held | pass | 214.26 | 209.18–226.46 |
| native | 20 | under-squeezed | pass | 213.17 | 212.42–217.25 |
| native | 50 | held | pass | 103.02 | 101.26–107.58 |
| native | 50 | under-squeezed | pass | 237.15 | 235.21–242.72 |
| rapier | 5 | held | fail | 26.50 | 26.27–50.88 |
| rapier | 5 | under-squeezed | fail | 25.14 | 25.06–25.72 |
| rapier | 20 | held | fail | 26.39 | 26.25–27.10 |
| rapier | 20 | under-squeezed | fail | 25.58 | 25.05–26.23 |
| rapier | 50 | held | fail | 26.46 | 26.23–27.25 |
| rapier | 50 | under-squeezed | fail | 25.36 | 25.17–25.81 |
| mujoco | 5 | held | fail | 40.00 | 38.90–41.11 |
| mujoco | 5 | under-squeezed | pass | 34.15 | 33.96–34.68 |
| mujoco | 20 | held | fail | 40.49 | 39.79–41.10 |
| mujoco | 20 | under-squeezed | pass | 39.35 | 39.22–42.30 |
| mujoco | 50 | held | fail | 38.68 | 38.58–39.32 |
| mujoco | 50 | under-squeezed | pass | 39.29 | 38.31–40.98 |

All final physical outputs repeated exactly across these five runs. Native
passes all six cases. Rapier lifts held cubes, but misses one or more strict
slip/rise/tilt tolerances; its under-squeezed cubes remain on the floor with
about 1.28 mm penetration, exceeding the 1 mm height tolerance. This does not
mean Rapier is unable to grasp objects.

MuJoCo's existing typed-PD adapter with these high gains fails the held cases:
the lift itself does not track the native target, and the cube remains on the
floor. A future control-integration comparison must address this mismatch
without silently weakening actuator bounds or excluding failed cases. It is
not evidence that MuJoCo's native position actuators cannot solve the task.

Next evidence stages are independently validated motor integration, moving-cart
comparison, force/penetration and full-trajectory traces, timestep sensitivity,
and setup-excluded scaling benchmarks. This bounded run alone does not justify
an increase in overall OSS maturity.
