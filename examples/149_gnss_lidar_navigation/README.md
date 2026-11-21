# Example 149: GNSS outdoors, LiDAR indoors

```bash
scripts/build_gnsspp.sh   # once: libgnss++ for RTK (optional)
cargo run --release -p gnss_lidar_navigation --example 149_gnss_lidar_navigation
```

A differential-drive robot drives a 313 m round trip: an open plaza, a 30 m
wide street between 25–40 m glass, concrete and metal towers, and a roofed
single-storey office with a corridor, a north room, pillars and desks. Pure
pursuit steers on the *estimated* pose, so localization errors become driving
errors. The run takes about 18 s in release mode and writes
`target/149_gnss_lidar_navigation.svg` (`--svg <path>` to choose).

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

* **GNSS** — `rne_gnss` ray traces 48 GPS + Galileo satellites through the
  buildings at 5 Hz for two antennas on the robot (front and rear, 0.8 m
  apart) and a base station on the roof of the 40 m tower: direct paths, up
  to two wall/ground bounces, and knife-edge diffraction, on L1/E1 and
  L2C/E5a, with code, carrier phase and Doppler. libgnss++ solves RTK for the
  rear antenna against the base (its `low-cost` preset) and moving-baseline
  RTK between the antennas for heading. Dashed red lines in the top view run
  from the antenna to the wall point or edge of each NLOS or diffracted
  signal; the sky plot shows the building skyline from there.
* **LiDAR** — a 360-beam scan at 0.3 m height, ray cast against the same
  geometry at 10 Hz with 2 cm noise, matched against a prior occupancy map of
  the office rasterized from the scene.
* **Odometry** — wheel speed with a 1 % scale error and a gyro with a
  0.06 °/s bias.
* **Arbiter** — `rne_slam::HybridLocalizer` fuses healthy, gated RTK
  solutions and headings outdoors (taking back control only on RTK fixed
  solutions), falls back to dead reckoning when they are not, relocalizes
  with a wide scan match when the estimate enters the map, and then tracks
  with AMCL refined by a local scan match.

Without libgnss++ the example uses the rear antenna's built-in single-point
fixes instead (canyon RMSE 3.2 m instead of 0.5 m).

## Results

Position RMSE / max in meters (deterministic):

| Zone | Hybrid | EKF fusing every solution | Odometry only |
|------|--------|---------------------------|---------------|
| Open plaza | 0.07 / 0.68 | 0.07 / 0.68 | 11.34 / 20.24 |
| Urban canyon | 0.52 / 1.71 | 6.94 / 13.95 | 5.09 / 12.68 |
| Office forecourt | 0.12 / 0.20 | 7.80 / 13.24 | 5.43 / 6.36 |
| Indoor office | 0.05 / 0.14 | 10.56 / 13.74 | 7.84 / 8.75 |

Ray-traced sky at the rear antenna, mean satellites per epoch:

| Zone | Tracked | LOS | LOS + wall multipath | NLOS | Diffracted | Blocked | Phase-locked |
|------|---------|-----|----------------------|------|------------|---------|--------------|
| Open plaza | 11.8 | 8.7 | 2.2 | 0.8 | 0.2 | 1.6 | 11.7 |
| Urban canyon | 12.0 | 6.9 | 0.2 | 4.8 | 0.1 | 1.3 | 11.7 |
| Office forecourt | 10.1 | 5.0 | 0.5 | 3.6 | 1.0 | 2.9 | 9.7 |
| Indoor office | 0.4 | 0.0 | 0.0 | 0.3 | 0.0 | 12.6 | 0.4 |

libgnss++ solutions, % of epochs:

| Zone | Fixed | Float | Single | None | Fixed RMSE | Heading fixed | Heading RMSE |
|------|-------|-------|--------|------|------------|---------------|--------------|
| Open plaza | 99 | 1 | 0 | 0 | 1.1 cm | 85 % | 1.3° |
| Urban canyon | 27 | 49 | 25 | 0 | 0.51 m | 81 % | 2.4° |
| Office forecourt | 26 | 14 | 60 | 0 | 5.1 m | 43 % | 1.1° |
| Indoor office | 0 | 2 | 1 | 97 | — | 0 % | — |

The canyon and forecourt fixed RMSEs come from wrong fixes under NLOS
multipath (10 m off), which the arbiter's gate rejects. The robot keeps at
least 1.41 m from every wall and spends 96 s on GNSS, 89 s on LiDAR and 99 s
dead reckoning, mostly through the canyon.

See [GNSS](../../docs/GNSS.md) for the models and their limits.
