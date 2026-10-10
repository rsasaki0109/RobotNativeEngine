# Forty-AMR warehouse fleet

Example 140 runs a recorded 40-robot warehouse logistics scenario. RNE supplies
`WorldRandom` and `TrafficCoordinator`; directed routing, task dispatch, the
scenario energy model, and bounded-motion control are local to this example.
The warehouse, robot meshes, HUD, code, and media are original. The scenario
and visual direction were inspired by
[WareTwin](https://github.com/WayneChou-bot/WareTwin), inspected at commit
`81f365f323e1a097e5819ca9045637544163f219`; no upstream code or assets are copied.

This is a kinematic logistics demonstration. It does not simulate wheel
traction, contact forces, physical load pickup, localization error, or hardware
connections. Cargo appearance represents the recorded loaded state. The
coloured floor rings and route lines are visual annotations.

## Run without a GPU

Run from the repository root:

```bash
cargo run --release --locked -p warehouse_fleet --example 140_warehouse_fleet
cargo test --locked -p warehouse_fleet
```

The default output is `target/warehouse-fleet/{trace,validation}.json`. Two
optional positional arguments select those paths:

```bash
cargo run --release --locked -p warehouse_fleet --example 140_warehouse_fleet -- \
  target/warehouse-fleet/independent-trace.json \
  target/warehouse-fleet/independent-validation.json
```

Each invocation runs two fresh seeded 240-second simulations, compares their
ordered full-state words exactly, and runs a same-seed control without the
aisle closure. Validation failure returns a failing exit status and retains
the measured results. The six unit tests cover graph connectivity, occupied
lease ownership, swept separation, bounded motion, junction deadlock, and
non-pose state participating in replay. `xtask ci-smoke media` also runs the
headless scenario with the same acceptance gates.

## Traffic and motion

Forty AMRs begin in specified off-lane parking/charging pockets in a 46 by
33 metre warehouse with 24 racks. Directed lanes and non-preemptive cell, edge,
and junction reservations coordinate arrivals. Loading and charging happen in
station pockets; idle robots return to parking. Robots stop and turn at
lane-cell transitions. Exact rest-to-rest profiles enforce 1.2 m/s translation,
1 rad/s rotation, and 1.8 m/s² acceleration. Conservative swept-circle checks
use a 0.65 metre collision radius per robot and inflate clearance for motion
between discrete samples.

The closure is requested at 120 seconds and activates after the affected
corridor drains. Route planning then excludes it until reopening at
165 seconds; leases do not evict occupied robots. The same-seed no-closure
control must traverse that corridor. A route change and six avoided cell-edge
entries are distinct from the control's single whole-corridor passage.

Battery use is configured at 0.018 percentage points per empty metre,
0.026 per loaded metre, and 0.002 per driving second. Charging adds 0.6
percentage points per second. These are scenario rates rather than measured
electrochemistry, energy consumption, or charger performance. Four robots
begin docked and are counted separately from R36's later routed charger visit.

## Recorded results

The qualified run uses seed `20261010` and a 0.05 second `SimClock` step.
Integer clock ticks select the step index; converting that index with the
fixed step preserves the recording's existing floating-point timestamp bits.
The 240-second run completes 16 deliveries with 16 distinct robots and
32 task assignments. The captured interval is 100–220 seconds: 481 frames at
0.25 second intervals, replayed at 5× speed as a 24.05 second, 20 fps GIF. The
visible completed-delivery counter advances from 4 to 15. All 40 robots appear
in every frame; the engaged count reflects the recorded operational state,
including charging and return to parking, and need not be 40.

The full-run conservative swept separation is at least 1.747 metres against a
required 1.30 metre diameter; collisions and rack incursions are zero. The
coarser exported-frame independent check still bounds separation at 1.720
metres with zero conservatively inflated rack intersections. R36 docks at
212.3 seconds, within the GIF interval. The closure produces one actual route
change and zero active-corridor occupancy violations.

The two internal runs compare 4,801 complete recorded world-state snapshots
(21,179,885 words). An additional fresh process produces byte-identical trace
and validation files. Bit identity is qualified within the tested runtime;
cross-platform bit identity and physical fleet safety are not established.

## Render and encode

The optional `capture` feature enables wgpu and PNG dependencies. See
[CAPTURE.md](CAPTURE.md) for the full preview/render/encode commands and Python
requirements. The renderer replays recorded poses; task counts, charging,
loaded meshes, closure markers, and the HUD come from the same trace.

The trace embeds compile-time hashes of `main.rs`, the example/workspace
manifests, workspace lockfile, and eight explicitly listed engine inputs.
The renderer verifies this same set and records its own compiled source hash,
each raw frame's RGBA hash, and all robot screen projections. The encoder
rejects failed/diagnostic captures, stale declared inputs, and altered pixels,
including under Python optimization. These selected input hashes are not a
complete compiler/dependency/GPU binary attestation.

The final [README metadata](../../docs/media/warehouse-fleet.json) retains the
simulation outcomes and media/source bindings. Simulator tests need no
rendering adapter. Public GIF generation requires the qualified trace and
matching raw capture; diagnostic previews cannot qualify public media.
