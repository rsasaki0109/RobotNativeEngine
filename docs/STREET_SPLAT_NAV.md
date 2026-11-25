# Street splat navigation (example 163)

Example 163 (`examples/163_street_splat_nav`, package `street_splat_nav`) drives a delivery
robot along a real street rebuilt from a 360-degree video. The street comes from visloc-rs
`real2sim.py`: SfM, a metric, levelled 3D Gaussian Splatting reconstruction, the walk the
video was taken on, Nav2's lane map and the kerbs where the walk crosses a street. visloc-rs
writes all of it once as a *street bundle* (`ros2/visloc-ros2/gs/rne_bundle.py`, or
`real2sim.py --export-rne DIR`); this example loads the bundle and simulates.

```text
cargo run --release -p street_splat_nav --example 163_street_splat_nav -- \
  --street BUNDLE --out DIR [--render-every 0.5] [--max-frames N]
```

`--render-every 0` drives headless. A bundle holds:

| File | What |
| --- | --- |
| `street.rne.splat.toml` | the splat manifest; `rotation_xyzw` turns the z-up street into RNE's y-up world |
| `street.json` | start, goals, walk, kerbs, camera height, robot radius, credit |
| `solid.rne.map` | solid cells (walls, posts, trunks), `rne_nav` map JSON |
| `lane.rne.map` | the lane map: free on the lane, 0.90 off it, walls occupied |
| `ground.f32` | ground height grid (shape in `street.json`) |

The bundle's coordinates are the visloc-rs map frame (z up); `map_to_rne` gives RNE's
`(x, z, -y)`, so a pose compares directly with the ROS 2 / Nav2 runs in visloc-rs.

## How it drives

- **Plan.** A* (`rne_nav::plan_path`) on a costmap of the lane map with the solid cells added
  (what the robot's scan would show). Walls and posts are lethal, inflated for a 0.45 m
  footprint (the robot drawn is 0.87 x 0.70 m) plus 0.25 m. Off the lane costs 234 of 252:
  costly but not lethal, as in the Nav2 runs.
- **Follow.** Pure pursuit (`rne_nav::pure_pursuit_follow`), 0.5 m/s, on ground truth.
- **Kerbs.** The road between two kerbs is 10 cm lower. The robot's height and pitch come
  from its front and rear wheels (0.5 m apart), and it crawls at 0.12 m/s while a wheel is
  on a kerb edge.
- **Contacts.** A step that would put the robot's 0.2 m collision circle on a solid cell is
  not taken and counts as a contact.

## Results (OverMaps360 street, 4 goals over 111 m)

| | goals | contacts | time | on kerbs | nearest solid cell | from the walk, median / max |
| --- | --- | --- | --- | --- | --- | --- |
| RNE, ground truth | 4/4 | 0 | 284 s | 9.9 s at <= 0.12 m/s | 0.61 m | 0.30 / 1.03 m |

For reference, the visloc-rs ROS 2 simulator with Nav2 on visloc localization reached 4/4
goals with no contacts and spent 9.8 s on the kerbs (its thirteenth run, with people and
cars). Headless, the RNE drive takes about 2 s of wall time. With a front camera and a chase
view every simulated second, 285 frames, it takes 7 minutes on CPU lavapipe. The trajectory
hash is the same with and without rendering.

The bundle is not in the repository: the footage is OverMaps360 (OverTheReality),
CC BY-NC 4.0. The library's tests use a synthetic street (a lane, a post, a kerb crossing).

## Next

Camera-only localization (visloc-rs fixes on the rendered front camera, as in example 161),
then the crowd and the cars from the visloc-rs simulator as splat actors. The plan is in
visloc-rs `docs/rne_migration.md`.
