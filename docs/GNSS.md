# Ray-traced GNSS and GNSS / LiDAR localization

`rne_gnss` simulates what a GNSS receiver measures among buildings by ray
tracing every satellite's signal through the scene, and `rne_slam`'s
`HybridLocalizer` uses those fixes outdoors and LiDAR Monte Carlo localization
indoors. Both are deterministic, ROS-free, and renderer-free.

[Example 149](../examples/149_gnss_lidar_navigation/README.md) drives a robot
from an open plaza down a street canyon into an office and back.

## What the receiver simulation models

| Stage | Model |
|-------|-------|
| Geodesy | Exact WGS-84 geodetic ↔ ECEF ↔ local ENU (`EnuFrame`). |
| Satellites | Keplerian orbits (Newton solve of Kepler's equation, Earth rotation); the nominal 24-slot GPS constellation and a Galileo Walker 24/3/1. 48 satellites by default. |
| Signal flight | Satellite position at transmission time with the light-time iteration and the Sagnac rotation, as a receiver computes it from ephemerides. |
| Scene | Axis-aligned boxes (buildings, walls, roofs) with a `SurfaceMaterial` reflection loss (concrete 8 dB, glass 4 dB, metal 2 dB) and a ground plane (12 dB). |
| Line of sight | A ray from the antenna toward each satellite; any box in the way blocks it. |
| Reflections | Single-bounce specular reflections off every vertical wall and the ground (image method for a plane wave): the mirrored ray must hit the facet within its bounds unobstructed, and the reflection point must see the satellite. Excess path `d (1 − u·u')`. |
| Classification | `LineOfSight`, `LineOfSightMultipath` (a wall replica within one chip), `NonLineOfSight` (direct blocked, a reflection tracked), `Blocked`, `TooWeak`, `BelowMask`. |
| C/N0 | Antenna gain pattern by arrival elevation, reflection loss, and the carrier-phase interference (fading) of replicas with the direct signal. |
| Code errors | DLL thermal noise from C/N0 (early-minus-late power discriminator), the narrow-correlator multipath error envelope at the geometric carrier phase, the full excess delay of an NLOS reception, residual ionosphere (Klobuchar obliquity) and troposphere, a per-satellite orbit/clock error, and a receiver clock random walk. |
| Solution | Weighted least squares on pseudoranges (elevation and C/N0 weights), DOP, RAIM fault detection and exclusion with a chi-square test, and a reported sigma scaled by the a-posteriori variance factor `χ²/dof`. |

Not modelled: diffraction, multi-bounce paths, carrier-phase (RTK) positioning,
Doppler, signal acquisition time, and non-box geometry.

### Sensor behavior

* **Timestamp** — an epoch is measured at the simulation time passed to
  `GnssReceiver::step` / `observe`, with the antenna at the position given for
  that time. Time is simulation time; no wall-clock time is read.
* **Latency** — a fix measured at `t` is released at `t + latency_s`; epochs
  fire at `update_rate_hz` (defaults 5 Hz, 50 ms).
* **Noise** — every draw comes from a `DeterministicRng` seeded by
  `ReceiverConfig::seed`, drawn in satellite-id order with a fixed number of
  draws per satellite, so one satellite's visibility never shifts another's
  noise. Per-satellite errors are seeded by satellite id. Replays are
  bit-identical (`tests/determinism/tests/gnss.rs`).

### Using it

```rust
use rne_gnss::{Constellation, EnuFrame, Geodetic, GnssReceiver, GnssScene, ReceiverConfig, SceneBox, SurfaceMaterial};
use rne_math::Vec3;

let frame = EnuFrame::new(Geodetic::from_degrees(36.08, 140.08, 25.0));
let mut scene = GnssScene::open_sky();
scene.add_box(SceneBox::building(10.0, 15.0, 60.0, 40.0, 35.0, SurfaceMaterial::GLASS));
let constellation = Constellation::gps_galileo_nominal();
let mut receiver = GnssReceiver::new(ReceiverConfig::default()).unwrap();
let antenna = Vec3::new(20.0, 0.0, 0.8);
for fix in receiver.step(&scene, &constellation, &frame, antenna, 0.0) {
    println!("{:?} with {} satellites", fix.position_enu_m, fix.satellites_used);
}
// Per-satellite ground truth of the last epoch: LOS / NLOS / blocked, C/N0, error.
let epoch = receiver.last_epoch().unwrap();
```

For ECS use, insert a `GnssEnvironment` resource, attach `GnssAntenna`
components, write each antenna's position, and call `step_gnss_antennas(world,
time_s)`; antennas are stepped in entity order.

## GNSS outdoors, LiDAR indoors

`HybridLocalizer` keeps one `EkfFusion` driven by wheel speed and a gyro yaw
rate and decides which absolute source corrects it:

1. **GNSS** — a fix is *healthy* if it uses at least `min_satellites`, its HDOP
   and reported sigma are small, RAIM found it consistent, RAIM excluded at most
   `max_excluded` satellites, and it passes a 99 % Mahalanobis gate against the
   EKF. Fixes are fused only while GNSS is in charge.
2. **Hand-over** — after `leave_gnss_epochs` unhealthy epochs the source drops
   to dead reckoning. Inside the prior occupancy map (with a margin), each scan
   runs a wide correlative scan match whose window is 3σ of the EKF position
   covariance. A match scoring at least `lidar_init_min_score` relocalizes the
   robot and seeds an `Amcl` particle filter.
3. **LiDAR** — each AMCL update is refined by a local scan match, seeded by the
   odometry-predicted EKF pose (falling back to the AMCL estimate), and the
   refined pose and yaw are fused. Too many low scores, or leaving the map,
   drop back to dead reckoning and resume the search.
4. **Back outside** — `enter_gnss_epochs` consecutive healthy fixes give GNSS
   control again.

`AbsolutePositionFix` is a plain struct, so `rne_slam` does not depend on
`rne_gnss`; any receiver, simulated or real, can feed it.

### Results of example 149

320 m round trip at 1.2 m/s, 48 GPS + Galileo satellites, 1 % wheel scale error
and a 0.06 °/s gyro bias. Position RMSE / max in meters:

| Zone | Hybrid | EKF fusing every fix | Odometry only |
|------|--------|----------------------|---------------|
| Open plaza | 0.68 / 1.22 | 0.75 / 2.69 | 13.50 / 21.87 |
| Urban canyon (30 m street, 25–40 m towers) | 1.88 / 4.89 | 10.03 / 24.47 | 5.58 / 13.68 |
| Office forecourt | 0.18 / 0.96 | 8.69 / 12.97 | 5.69 / 6.76 |
| Indoor office (roofed) | 0.04 / 0.11 | 8.91 / 9.88 | 8.36 / 9.33 |

In the canyon the ray trace gives on average 6.8 line-of-sight and 4.0 NLOS
satellites per epoch; RAIM excludes 2.4 per epoch, yet the fixes still use
2.9 NLOS satellites on average and their RMSE is 16 m. The quality checks keep
the hybrid on dead reckoning there, while the EKF that fuses every fix is
pulled off by up to 24 m.

## Limits

* The scene is axis-aligned boxes; arbitrary meshes (for example a PLATEAU
  city model) would need a BVH and triangle intersection.
* One reflection per path and no diffraction: signals that reach the antenna
  only by bending over a roof edge are reported blocked.
* Single-frequency code positioning only; no RTK, PPP, or Doppler velocity.
* The hybrid arbiter is a rule-based switch with hysteresis, not a factor
  graph; it does not use 3D-map-aided GNSS (shadow matching) to correct NLOS.
