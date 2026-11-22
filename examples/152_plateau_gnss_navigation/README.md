# Example 152: GNSS and LiDAR navigation in real Tokyo streets (PLATEAU)

```bash
python3 scripts/fetch_plateau_tokyo.py            # once: ~450 MB of CityGML
scripts/build_gnsspp.sh                           # once: libgnss++ for RTK
cargo run --release -p plateau_gnss_navigation --example 152_plateau_gnss_navigation
```

The same delivery robot as [example 151](../151_gnss_lidar_navigation/README.md),
but in real streets: around **Shibuya Station** and the **Nishi-Shinjuku**
tower district, built from Project PLATEAU's LOD2 city model. The full run
(both areas, five configurations each) takes about 40 minutes;
`--quick` runs only the main configuration, `--config "factor graph"` the
configurations whose label contains the text, `--area shibuya` one area, and
`--survey` only the sky survey (seconds).

![3D render in RNE: the robot walks from the Shibuya scramble crossing west and up Koen-dori among the PLATEAU LOD2 buildings; green rays are line-of-sight satellites, red NLOS reflections off the facades, purple diffraction over roof edges; the trail is blue while RTK localizes it and magenta while the LiDAR map does](../../docs/media/plateau-gnss-shibuya.gif)

![The same in Nishi-Shinjuku, among the skyscrapers west of the station](../../docs/media/plateau-gnss-shinjuku.gif)

## What is simulated

* **City** — `scripts/fetch_plateau_tokyo.py` reads the 5.3 GB PLATEAU
  Tokyo 23-ward CityGML archive with HTTP range requests and extracts only
  the 1 km meshes around the two stations. `rne_plateau::parse_building_triangles`
  triangulates each building's LOD2 roof, wall and ground surfaces; the
  example converts them to a local ENU frame (PLATEAU heights are Tokyo
  Peil; the geoid, 36.7 m, makes them ellipsoidal) and keeps the buildings
  within 450 m: 944 buildings and 134 k triangles around Shibuya, 507 and
  93 k in Nishi-Shinjuku. PLATEAU's 2020 Tokyo model has no facade
  material, so walls of buildings at least 45 m tall are treated as glass
  curtain walls and the rest as concrete.
* **GNSS** — `rne_gnss` ray traces the triangle mesh through its BVH:
  direct paths, reflections off wall and roof triangles larger than 4 m²
  (smaller detail scatters rather than mirrors), two-bounce paths among the
  48 nearest facets, and diffraction over roof outlines and building
  corners (the mesh's convex creases). GPS, Galileo and **QZSS**
  (Michibiki: three quasi-zenith satellites with apogee over Japan and one
  geostationary, on L1 C/A and L2C) at 5 Hz for two antennas on the robot
  and an RTK reference station on a 2 m pole in a park 1.3 km away (Yoyogi
  Park for Shibuya, Shinjuku Gyoen for Shinjuku), solved by libgnss++ as in
  example 151, with satellites the city model predicts are hidden withheld
  before RTK.
* **LiDAR** — a 3D LiDAR reduced to a planar scan the usual way (ground
  removed, the nearest return per azimuth in a band 0.3 m above the local
  terrain), 360 beams, 40 m, 10 Hz, matched against a prior map of the same
  slice along the route. The terrain comes from the bases of the PLATEAU
  buildings (Shibuya's streets climb toward Dogenzaka and Yoyogi).
* **Street furniture** — LOD2 facades are flat, so at LiDAR height a street
  is a featureless corridor; real Tokyo sidewalks are lined with utility
  poles, signposts and vending machines. The example places them along the
  route (every 6–14 m, irregularly, near the facades); they are part of the
  world, the prior map and the GNSS scene.
* **Localization** — `rne_slam::HybridLocalizer` as in example 151 (or the
  sliding-window `FactorGraphLocalizer`, below), with `lidar_anisotropic` on: each scan match is fused with a covariance from
  the curvature of its score, so a match along a corridor constrains only
  the cross-track position, and the particle filter's own match (over a
  wider window) can re-anchor the EKF when it explains the scans better.
* **Route** — waypoints on the streets, joined by A* over the open street
  cells keeping clear of facades: Shibuya from the scramble crossing west
  toward Dogenzaka, north up Koen-dori and back (750 m); Shinjuku from near
  the station's west side among the towers and back (960 m).

## Results

### Sky along the streets (`--survey`)

Satellites in line of sight at 1.5 m above every street point within 300 m
of the centre (one epoch):

| Area | Constellation | In line of sight | ≥ 8 visible | < 4 visible |
|------|---------------|------------------|-------------|-------------|
| Shibuya | GPS + Galileo | 4.0 of 13 | 7 % | 41 % |
| Shibuya | GPS + Galileo + QZSS | 5.7 of 16 | 29 % | 23 % |
| Nishi-Shinjuku | GPS + Galileo | 4.6 of 13 | 9 % | 30 % |
| Nishi-Shinjuku | GPS + Galileo + QZSS | 6.5 of 16 | 34 % | 11 % |

QZSS's near-zenith satellites are what a canyon can still see.

![Sky map around Shibuya Station: satellites in line of sight (green many, red few) and the walked route](../../docs/media/plateau-gnss-shibuya-sky.png)

### Navigation

| Area | Configuration | Goal | Position RMSE / max | RTK fixed | GNSS / LiDAR time | Longest dead reckoning |
|------|---------------|------|---------------------|-----------|-------------------|------------------------|
| Shibuya | map-aided GNSS + QZSS | reached, 799 s | 0.24 / 2.70 m | 61 % | 400 / 398 s | 0.1 s |
| Shibuya | no map aid | reached, 795 s | 0.19 / 1.60 m | 34 % | 298 / 497 s | 0.1 s |
| Shibuya | no QZSS | reached, 790 s | 0.27 / 3.02 m | 25 % | 159 / 631 s | 0.1 s |
| Shibuya | **factor graph** | reached, 773 s | 0.12 / 2.31 m | 64 % | — | 0.1 s |
| Shibuya | factor graph, no map aid | reached, 776 s | 0.14 / 1.51 m | 45 % | — | 0.1 s |
| Nishi-Shinjuku | map-aided GNSS + QZSS | reached, 1014 s | 0.66 / 3.49 m | 35 % | 198 / 815 s | 0.1 s |
| Nishi-Shinjuku | no map aid | **not reached** (lost) | 6.99 / 10.73 m | 55 % | 37 / 1763 s | 0.1 s |
| Nishi-Shinjuku | no QZSS | reached, 1208 s, after a 178 s safety stop | 3.82 / 9.31 m | 19 % | 89 / 940 s | 178 s |
| Nishi-Shinjuku | **factor graph** | reached, 996 s | 0.74 / 4.18 m | 29 % | — | 0.8 s |
| Nishi-Shinjuku | factor graph, no map aid | **reached**, 1011 s | 1.47 / 35.66 m | 12 % | — | 0.1 s |

The LiDAR map carries the robot through Shibuya with or without the
GNSS aids. Among the Nishi-Shinjuku towers the hybrid arbiter needs both
the 3D-map NLOS exclusion and QZSS: without the exclusion, wrong RTK
solutions are accepted and the robot gets lost; without QZSS, too few
satellites remain and the robot has to stop.

The **factor graph** (`rne_slam::FactorGraphLocalizer`, `--config "factor
graph"`) weighs the last few seconds of odometry, GNSS and LiDAR together
instead of switching: GNSS fixes go in under a Cauchy kernel, so a wrong
RTK fix that disagrees with the odometry and the scans loses its weight
rather than being gated by one threshold. It halves the error in Shibuya
and reaches the goal in Nishi-Shinjuku even without the NLOS exclusion,
where the hybrid gets lost, though with one 36 m excursion before RTK
re-anchors it. Its "GNSS / LiDAR time" is not comparable (it does not
switch), so the table leaves it out.

## Limits

* LOD2 buildings are prisms with roofs: no balconies, signs, glass detail or
  ground-floor arcades, and the facade material is guessed from the height.
* The street furniture is synthetic, and there are no vehicles or
  pedestrians.
* The terrain is interpolated from building bases, not a road surface model.
* The constellation is nominal (24 GPS, 24 Galileo, 4 QZSS); a real receiver
  in Tokyo also tracks GLONASS and BeiDou and sees more satellites.

Data: 3D City Model (Tokyo 23 wards) 2020, Project PLATEAU / MLIT, used
under the PLATEAU site policy (compatible with CC BY 4.0). The model is
fetched at run time and not stored in this repository; the media in
`docs/media` are derived from it.

To regenerate the GIFs (any GPU, or Mesa's lavapipe; Pillow for the HUD):

```bash
cargo run --release -p plateau_gnss_navigation --features capture \
    --example 152_plateau_gnss_navigation -- --quick --render target/152_render
python3 examples/152_plateau_gnss_navigation/encode.py target/152_render/shibuya docs/media/plateau-gnss-shibuya.gif
python3 examples/152_plateau_gnss_navigation/encode.py target/152_render/shinjuku docs/media/plateau-gnss-shinjuku.gif
```
