# Example 149: GNSS outdoors, LiDAR indoors

```bash
scripts/build_gnsspp.sh   # once: libgnss++ for RTK (optional)
cargo run --release -p gnss_lidar_navigation --example 149_gnss_lidar_navigation
```

A delivery robot drives a 329 m round trip: across an open plaza with trees,
benches and a kiosk, along the south sidewalk of a 30 m street between 9–55 m
towers (glass, concrete and metal; podiums with setback towers; blocks
behind), past street trees, lamp posts, traffic signals, a bus stop, parked
cars and a bus, and moving traffic, into a roofed single-storey office with a
corridor, a north room, pillars and desks, and back. Pure pursuit steers on
the *estimated* pose, so localization errors become driving errors, and the
robot stops if it has been dead reckoning for more than 3 s. The run takes
about 45 s in release mode and writes `target/149_gnss_lidar_navigation.svg`
(`--svg <path>` to choose).

![3D render in Robot Native Engine: the robot drives from the plaza down the street canyon into the office; beams show each tracked satellite signal (green line of sight, red NLOS reflections bending at the glass and concrete facades, purple diffraction), cyan dots the LiDAR returns, and the HUD the localization source, the libgnss++ RTK status and the sky at the robot](../../docs/media/gnss-lidar-navigation-3d.gif)

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

* **World** — the sensors see everything: buildings, street furniture, tree
  trunks and canopies (which attenuate GNSS by 1 dB per metre of foliage),
  and the parked and moving vehicles (metal reflectors and LiDAR obstacles).
* **GNSS** — `rne_gnss` ray traces 48 GPS + Galileo satellites at 5 Hz for
  two antennas on the robot (front and rear, 0.8 m apart) and a base station
  on the roof of the 40 m tower: direct paths, up to two bounces, knife-edge
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
  well as indoors. Dead reckoning bridges only the instant of a hand-over.

Without libgnss++ the example uses the rear antenna's built-in single-point
fixes instead.

## Results

Position RMSE / max in meters (deterministic):

| Zone | Hybrid | EKF fusing every solution | Odometry only |
|------|--------|---------------------------|---------------|
| Open plaza | 0.14 / 1.39 | 0.14 / 1.39 | 12.83 / 22.35 |
| Street (sidewalk) | 0.10 / 0.29 | 12.17 / 46.99 | 5.64 / 14.33 |
| Office forecourt | 0.14 / 0.32 | 3.63 / 11.42 | 5.44 / 6.39 |
| Indoor office | 0.05 / 0.12 | 0.88 / 1.13 | 7.99 / 8.93 |

The robot is localized by GNSS for 102 s and by LiDAR for 200 s; its longest
dead-reckoning stretch is 0.0 s (it never triggers the 3 s safety stop).

Ray-traced sky at the rear antenna, mean satellites per epoch:

| Zone | Tracked | LOS | LOS + wall multipath | NLOS | Diffracted | Blocked | Phase-locked |
|------|---------|-----|----------------------|------|------------|---------|--------------|
| Open plaza | 10.4 | 6.6 | 2.4 | 0.9 | 0.5 | 3.0 | 10.1 |
| Street (sidewalk) | 10.5 | 4.0 | 2.9 | 3.0 | 0.6 | 3.0 | 10.1 |
| Office forecourt | 10.4 | 3.5 | 2.0 | 3.8 | 1.1 | 2.6 | 9.5 |
| Indoor office | 0.2 | 0.1 | 0.0 | 0.1 | 0.0 | 12.8 | 0.2 |

libgnss++ solutions, % of epochs:

| Zone | Fixed | Float | Single | None | Fixed RMSE | Heading fixed | Heading RMSE |
|------|-------|-------|--------|------|------------|---------------|--------------|
| Open plaza | 98 | 1 | 0 | 0 | 3.5 cm | 79 % | 0.3° |
| Street (sidewalk) | 40 | 42 | 16 | 2 | 3.2 m | 29 % | 3.4° |
| Office forecourt | 23 | 28 | 26 | 23 | 2.6 m | 4 % | 4.8° |
| Indoor office | 0 | 0 | 0 | 100 | — | 0 % | — |

The street and forecourt fixed RMSEs come from wrong fixes caused by NLOS
that the city model does not predict (trees, buses), which the arbiter's
gate rejects.

### What each part contributes

| Configuration | Goal | Longest dead reckoning | Street RMSE (hybrid) |
|---------------|------|------------------------|----------------------|
| Full (map-aided GNSS + LiDAR map of the route) | reached, 302 s | 0.0 s | 0.10 m |
| `--no-map-aided-gnss` | reached, 307 s | 0.1 s | 0.11 m (street RTK fix 20 % instead of 40 %) |
| `--indoor-map-only` | **not reached**: safety stop in the street after 93 m, never relocalized | 823 s | — |
| both | **not reached**: safety stop after 62 m | 850 s | — |

With a LiDAR map of the office only, GNSS in the street is too poor to hand
control back, the robot dead reckons for 3 s and stops, and nothing
relocalizes it: the outdoor LiDAR map is what keeps an autonomous robot
moving through an urban canyon.

See [GNSS](../../docs/GNSS.md) for the models and their limits.
