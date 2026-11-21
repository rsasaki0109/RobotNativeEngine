# Example 149: GNSS outdoors, LiDAR indoors

```bash
scripts/build_gnsspp.sh   # once: libgnss++ for RTK (optional)
cargo run --release -p gnss_lidar_navigation --example 149_gnss_lidar_navigation
```

A delivery robot drives a 370 m round trip through a small downtown laid out
as a real one is: a 4-lane main street (18 m carriageway, 6 m sidewalks)
crossed by two 2-lane streets, forming city blocks of 9–50 m buildings
(glass, concrete and metal; podiums with setback towers) built to the
property line, with planters along the shop fronts. The intersections have
zebra crossings and traffic signals on a 50 s cycle; cars and buses keep
their lanes, stop at the line on red, queue behind each other, and some are
parked at the kerb. Lamp posts, street trees and a bus stop line the
sidewalks.

The robot starts in a plaza on the south-west block, walks along the south
sidewalk of the main street, waits at each crossing for the pedestrian
signal, enters the office forecourt on the south-east block, goes through
the door into a single-storey office (a north and a south room, pillars,
desks), and back. Pure pursuit steers on the *estimated* pose, so
localization errors become driving errors. The robot stops if it has been
dead reckoning for more than 3 s, and a reactive LiDAR check stops forward
motion when anything is within 0.6 m ahead, whatever the localizer thinks.
The run takes about a minute in release mode and writes
`target/149_gnss_lidar_navigation.svg` (`--svg <path>` to choose).

![3D render in Robot Native Engine: the robot walks from the plaza along the main street, waits at the crossings while the traffic stops at the signals, and enters the office; beams show each tracked satellite signal (green line of sight, red NLOS reflections bending at the glass and concrete facades, purple diffraction), cyan dots the LiDAR returns, and the HUD the localization source, the libgnss++ RTK status and the sky at the robot](../../docs/media/gnss-lidar-navigation-3d.gif)

The 3D view is rendered by RNE's wgpu renderer (`rne_render_wgpu`, PBR with
shadows) from the run's trace; the office roof is cut away so the indoor part
is visible. To regenerate it (any GPU, or Mesa's lavapipe via
`mesa-vulkan-drivers`; Pillow and ffmpeg for the HUD and GIF):

```bash
cargo run --release -p gnss_lidar_navigation --example 149_gnss_lidar_navigation -- --trace target/gnss3d/trace.json
cargo run --release -p gnss_lidar_navigation --features capture --example 149_gnss_lidar_navigation_render -- \
    --trace target/gnss3d/trace.json --output target/gnss3d/frames
python3 examples/149_gnss_lidar_navigation/encode.py target/gnss3d docs/media/gnss-lidar-navigation-3d.gif
```

A top-down 2D animation of the same run is in
[docs/media/gnss-lidar-navigation.gif](../../docs/media/gnss-lidar-navigation.gif); the run summary as a top view and a canyon sky plot:

![Top view and canyon sky plot](gnss_lidar_navigation.svg)

To regenerate the animation (needs Python Playwright and ffmpeg):

```bash
cargo run --release -p gnss_lidar_navigation --example 149_gnss_lidar_navigation -- --frames target/gnss_frames
python3 scripts/svg_frames_to_gif.py target/gnss_frames docs/media/gnss-lidar-navigation.gif
```

* **World** — the sensors see everything: buildings, street furniture
  (lamps, signals, planters, the bus stop), tree
  trunks and canopies (which attenuate GNSS by 1 dB per metre of foliage),
  and the parked and moving vehicles (metal reflectors and LiDAR obstacles).
* **GNSS** — `rne_gnss` ray traces 48 GPS + Galileo satellites at 5 Hz for
  two antennas on the robot (front and rear, 0.8 m apart) and a base station
  on the roof of a 46 m tower across the street: direct paths, up to two bounces, knife-edge
  diffraction and foliage loss, on L1/E1 and L2C/E5a, with code, carrier
  phase and Doppler. Before RTK the satellites that the **3D city model**
  (buildings only, like a PLATEAU model) predicts are hidden at the estimated
  antenna positions are withheld; NLOS caused by trees and vehicles, which
  the model does not contain, still gets through. libgnss++ then solves RTK
  for the rear antenna against the base (`low-cost` preset) and
  moving-baseline RTK between the antennas for heading.
* **LiDAR** — a 360-beam scan at 0.3 m height, ray cast against the world at
  10 Hz with 2 cm noise, matched against a **prior map of the whole route**
  (buildings and permanent street furniture; never vehicles).
* **Odometry** — wheel speed with a 1 % scale error and a gyro with a
  0.06 °/s bias.
* **Arbiter** — `rne_slam::HybridLocalizer` fuses healthy, gated RTK
  solutions and headings (taking back control only on RTK fixed solutions);
  when they are not healthy it relocalizes against the LiDAR map with a wide
  scan match and tracks with AMCL refined by a local scan match, outdoors as
  well as indoors, fusing only matches within the 99 % gate of the
  estimate. Dead reckoning bridges only the instant of a hand-over.

Without libgnss++ the example uses the rear antenna's built-in single-point
fixes instead.

## Results

Position RMSE / max in meters (deterministic):

| Zone | Hybrid | EKF fusing every solution | Odometry only |
|------|--------|---------------------------|---------------|
| Plaza | 0.01 / 0.09 | 0.02 / 0.09 | 17.47 / 27.16 |
| Main street (sidewalk) | 0.02 / 0.11 | 0.37 / 4.94 | 8.47 / 23.25 |
| Office forecourt | 0.04 / 0.13 | 7.08 / 24.04 | 9.33 / 10.28 |
| Indoor office | 0.05 / 0.19 | 0.76 / 1.13 | 10.38 / 10.79 |

The robot reaches the goal after 368 s (38 s of it waiting at crossings). It
is localized by GNSS for 309 s and by LiDAR for 57 s; its longest
dead-reckoning stretch is 1.8 s, under the 3 s safety limit.

Ray-traced sky at the rear antenna, mean satellites per epoch:

| Zone | Tracked | LOS | LOS + wall multipath | NLOS | Diffracted | Blocked | Phase-locked |
|------|---------|-----|----------------------|------|------------|---------|--------------|
| Plaza | 11.6 | 8.3 | 1.7 | 0.7 | 0.9 | 1.9 | 10.8 |
| Main street (sidewalk) | 11.0 | 6.0 | 2.7 | 1.6 | 0.7 | 2.6 | 10.6 |
| Office forecourt | 11.2 | 5.7 | 2.4 | 2.4 | 0.7 | 2.1 | 10.5 |
| Indoor office | 0.4 | 0.1 | 0.0 | 0.1 | 0.2 | 12.6 | 0.4 |

libgnss++ solutions, % of epochs:

| Zone | Fixed | Float | Single | None | Fixed RMSE | Heading fixed | Heading RMSE |
|------|-------|-------|--------|------|------------|---------------|--------------|
| Plaza | 98 | 2 | 0 | 0 | 0.2 cm | 98 % | 0.06° |
| Main street (sidewalk) | 99 | 0 | 1 | 0 | 0.50 m | 83 % | 0.47° |
| Office forecourt | 83 | 2 | 10 | 5 | 2.5 m | 48 % | 0.70° |
| Indoor office | 0 | 0 | 0 | 100 | — | 0 % | — |

The fixed RMSEs in the street and forecourt come from a few wrong fixes
caused by NLOS that the city model does not predict (trees, buses), which
the arbiter's gate rejects.

### What each part contributes

| Configuration | Goal | Longest dead reckoning | Street RMSE (hybrid) |
|---------------|------|------------------------|----------------------|
| Full (map-aided GNSS + LiDAR map of the route) | reached, 368 s | 1.8 s | 0.02 m |
| `--indoor-map-only` | reached, 368 s | 2.2 s | 0.01 m |
| `--no-map-aided-gnss` | reached, 507 s, after a 132 s safety stop | 135 s | 5.27 m (street RTK fixed 75 % instead of 99 %, many wrong) |
| both | **not reached**: safety stop after 86 m, never relocalized | 828 s | — |

On this wide street the map-aided GNSS alone keeps the robot on RTK. Without
it, NLOS makes GNSS unhealthy at several points of the street and the LiDAR
map takes over, which works on the way out (errors under 0.8 m). On the way
back a wrong RTK fix accepted just before a hand-over leaves the estimate
2.5 m off; the LiDAR matches then disagree with the confident estimate and
are gated out, the error grows to 8 m, the arbiter drops to dead reckoning
and the robot stops after 3 s until RTK is fixed again 132 s later. Without
either, nothing relocalizes it in the street.

See [GNSS](../../docs/GNSS.md) for the models and their limits.
