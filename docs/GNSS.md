# Ray-traced GNSS, libgnss++ RTK, and GNSS / LiDAR localization

`rne_gnss` simulates what GNSS receivers measure among buildings by ray
tracing every satellite's signal through the scene, on two frequencies and
with carrier phase. [libgnss++](https://github.com/rsasaki0109/gnssplusplus-library)
solves single-point, RTK and moving-baseline RTK positions from those
observations through `adapters/gnsspp/rne_adapter_gnsspp`. `rne_slam`'s
`HybridLocalizer` uses the GNSS solutions outdoors and LiDAR Monte Carlo
localization indoors. The simulation is deterministic, ROS-free and
renderer-free; libgnss++ stays in the adapter.

[Example 149](../examples/149_gnss_lidar_navigation/README.md) drives a robot
with two antennas from an open plaza down a street canyon into an office and
back, with an RTK base station on a rooftop.

![Example 149: GNSS outdoors, LiDAR indoors](media/gnss-lidar-navigation.gif)

## What the simulation models

| Stage | Model |
|-------|-------|
| Time, geodesy | GPS week / time of week; exact WGS-84 geodetic ↔ ECEF ↔ local ENU. |
| Satellites | Keplerian orbits; the nominal 24-slot GPS constellation and a Galileo Walker 24/3/1 (48 satellites). Exported as broadcast ephemerides (`NavigationMessage`) that reproduce the simulated orbits to 0.1 mm with the IS-GPS-200 algorithm. |
| Signal flight | Satellite position at transmission (light-time iteration) and the Sagnac rotation. |
| Scene | Axis-aligned boxes with a reflection loss per material (concrete 8 dB, glass 4 dB, metal 2 dB) and a ground plane (12 dB). |
| Direct path | A ray toward each satellite; any box blocks it. |
| Reflections | Up to `max_reflections` (default 2) specular bounces off walls and the ground by the image method: the look direction is the satellite direction mirrored through the facet sequence, and the ray is traced forward through every facet, bounds and occluders included. Excess path `Σ|Pᵢ − Pᵢ₋₁| − (Pk − A)·u`. |
| Diffraction | For blocked satellites, the point on each box edge minimising the path (Keller's law for a plane wave), when both legs are clear. Loss is the ITU-R P.526 knife-edge `J(ν)` with `ν = 2√(Δ/λ)`, so it depends on frequency. |
| Signals | GPS L1 C/A and L2C, Galileo E1 and E5a: frequency, chip length, relative power, iono scaling. |
| Tracking | The direct path when LOS, else the strongest indirect path (NLOS or diffracted). Later replicas within a chip distort the code (narrow-correlator envelope) and carrier (phasor sum), and fade the C/N0. |
| Pseudorange | Geometric range + receiver clock + satellite orbit/clock error (shared by all receivers) + true ionosphere (Klobuchar × truth scale, `(f₁/f)²`) + true troposphere (Saastamoinen × truth scale) + excess path + code multipath + DLL noise from C/N0. |
| Carrier phase | The same with the ionosphere sign reversed, carrier multipath, PLL noise, and an integer ambiguity per lock arc. Lock needs `phase_lock_threshold_dbhz`; slips occur at random near it, and whenever the tracked path changes by more than a quarter wavelength (its phase jumps by the path change). A new arc sets the loss-of-lock flag. |
| Doppler | Rate of the tracked range plus clock between epochs, with noise; absent after a slip. |
| Receivers | Clock bias with a drift random walk. `AntennaRig` mounts several antennas at body-frame lever arms, each with its own receiver; a base station is a receiver at a fixed point. |
| Built-in fix | Weighted least squares with the broadcast Klobuchar and Saastamoinen corrections, DOP, RAIM exclusion, and a sigma scaled by `χ²/dof`. |

Not modelled: multi-bounce paths combined with diffraction, diffraction while
the direct path is clear, polarisation beyond a per-bounce loss, receiver
acquisition time, and non-box geometry.

### Sensor behavior

* **Timestamp** — an epoch is measured at the simulation time passed to
  `GnssReceiver::step` / `observe` (GPS time `epoch + time_s`), with the
  antenna where the caller put it. No wall-clock time is read.
* **Latency** — the built-in fix of an epoch measured at `t` is released at
  `t + latency_s`; epochs fire at `update_rate_hz` (defaults 5 Hz, 50 ms).
* **Noise** — all draws come from `DeterministicRng`: a fixed number per
  satellite and signal in satellite-id order (so one satellite's visibility
  never shifts another's noise); satellite errors seeded by satellite id and
  `satellite_error_seed`; ambiguities seeded by satellite, signal and arc.
  Replays are bit-identical (`tests/determinism/tests/gnss.rs`).

## libgnss++ adapter

`rne_adapter_gnsspp` compiles a C ABI shim against a libgnss++ build and
exposes:

* `Navigation::new(&NavigationMessage)` — ephemerides and Klobuchar
  coefficients.
* `Spp` — libgnss++ single-point positioning.
* `Rtk` (`RtkMode::Kinematic`, `Static`, `MovingBase`) with `RtkConfig`
  (elevation mask, ratio threshold, libgnss++ presets such as `low-cost` or
  `moving-base`, ambiguity reset after consecutive non-fixed epochs) and
  `diagnostics()` (libgnss++'s AR telemetry).

Build libgnss++ once (needs git, CMake, a C++20 compiler and Eigen 3):

```bash
scripts/build_gnsspp.sh
```

It checks out a pinned revision into `target/gnsspp/src`, where the adapter's
`build.rs` finds it; `RNE_GNSSPP_DIR` points at another checkout. Without
libgnss++ the adapter builds as a stub whose constructors return
`GnssppError::Unavailable`, so the workspace never requires a C++ toolchain
(`RNE_GNSSPP_DISABLE=1` forces the stub).

On simulated open-sky data libgnss++ fixes kinematic RTK to centimetres and
a 1 m moving baseline to under 0.02 rad
(`adapters/gnsspp/rne_adapter_gnsspp/tests/rtk.rs`).

## GNSS outdoors, LiDAR indoors

`HybridLocalizer` keeps one `EkfFusion` driven by wheel speed and a gyro yaw
rate and decides which absolute source corrects it:

1. **GNSS** — a fix (`AbsolutePositionFix`, with a `FixKind` of single,
   float or fixed) is healthy if it uses enough satellites, its HDOP and
   reported sigma are small, it is RAIM-consistent with few exclusions, and
   it passes a 99 % Mahalanobis gate. Its variance has a floor per kind.
   Moving-baseline headings (`AbsoluteHeadingFix`) are fused, gated, while
   GNSS is in charge.
2. **Hand-over** — after `leave_gnss_epochs` unhealthy epochs the source drops
   to dead reckoning. Inside the prior occupancy map, each scan runs a wide
   correlative scan match sized by the EKF covariance; a good match seeds an
   `Amcl` particle filter.
3. **LiDAR** — each AMCL update is refined by a local scan match, and the
   refined pose and yaw are fused.
4. **Back outside** — `enter_gnss_epochs` consecutive healthy fixes of at
   least `enter_gnss_min_kind` give GNSS control again. A gate reset after
   `gate_reset_epochs` of disagreement needs an RTK fixed solution or an EKF
   that is already uncertain, so a sustained NLOS or float bias cannot drag
   a well-localized estimate.

### Results of example 149

313 m round trip at 1.2 m/s; 1 % wheel scale error, 0.06 °/s gyro bias.
Position RMSE / max in meters:

| Zone | Hybrid | EKF fusing every solution | Odometry only |
|------|--------|---------------------------|---------------|
| Open plaza | 0.07 / 0.68 | 0.07 / 0.68 | 11.34 / 20.24 |
| Urban canyon (30 m street, 25–40 m towers) | 0.52 / 1.71 | 6.94 / 13.95 | 5.09 / 12.68 |
| Office forecourt | 0.12 / 0.20 | 7.80 / 13.24 | 5.43 / 6.36 |
| Indoor office (roofed) | 0.05 / 0.14 | 10.56 / 13.74 | 7.84 / 8.75 |

In the open plaza libgnss++ fixes 99 % of epochs (1.1 cm RMSE) and the moving
baseline gives heading on 85 % (1.3° RMSE). In the canyon 4.8 of 12 tracked
satellites are NLOS: only 27 % of epochs fix, some of those fixes are 10 m
wrong, and float solutions are metres off. The arbiter rejects them and dead
reckons.

## Limits

* The scene is axis-aligned boxes; arbitrary meshes (for example a PLATEAU
  city model) would need a BVH and triangle intersection.
* libgnss++ RTK covariances are not used as reported (they come back
  constant); the example floors the variance per solution type instead.
* The hybrid arbiter is a rule-based switch with hysteresis, not a factor
  graph, and does not use 3D-map-aided GNSS (shadow matching) to correct NLOS.
