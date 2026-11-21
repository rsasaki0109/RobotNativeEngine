# Perception tasks

Nine scenes, each built to test one part of robot perception, are scanned by
RNE's simulated LiDAR. A cluster detector turns each scan into object
detections. The detections, the ego poses and the ground truth are written in
the replay format of
[robo-perception](https://github.com/rsasaki0109/robo-perception), so a tracker
can be scored on each scene. RNE does not depend on robo-perception; the two
share only JSONL files.

```bash
cargo run --release -p perception_tasks --example 149_perception_tasks -- --out target/perception_tasks
cargo run --release -p perception_tasks --example 149_perception_tasks -- --list
cargo run --release -p perception_tasks --example 149_perception_tasks -- --task occlusion --duration-s 5
```

All nine tasks take about 17 s on one CPU core.

## Tasks

| Task | Scene | What it tests |
| --- | --- | --- |
| `people_basic` | 4 people walking across, toward and away from a standing robot | Baseline tracking and velocity |
| `crowd_crossing` | Two groups of 7 people pass through each other in interleaved lanes | Identity through a crowd |
| `occlusion` | 5 people pass behind two pillars, a kiosk and each other | Keeping identity through full occlusions |
| `maneuvering` | Stop-and-go, a U-turn, a 3 m/s jogger, a zig-zag, a person who starts from standing | Constant-velocity motion model limits |
| `moving_robot` | The robot drives down a corridor at 1 m/s among 6 people; poles along the walls | Ego motion |
| `pose_jitter` | As `moving_robot`, but the poses given to the tracker jitter (1°, 0.1 m per scan) | Ego-pose error, landmark pose correction |
| `mixed_traffic` | Pedestrians, 2 cyclists and 5 cars (7–9 m/s) on a sidewalk, bike lane and road | Classes, fast objects |
| `long_range` | 5 people 12–35 m away, about 6 points each | Sparse returns |
| `side_by_side` | Three pairs walking 0.6 m apart; one pair splits up | Merged detections |

## Sensor and detector

- **LiDAR**: 16 channels from −15° to +15°, 900 columns (0.4°), 10 Hz,
  0.9 m above the ground, 0.02 m range noise, 1 % dropout
  (`rne_sensor::sample_lidar_keyed` on Rapier). Occlusion and point density
  come from the raycasts; the scene does not script them.
- **Objects**: people are 1.64 m capsules; cyclists are 1.7 × 0.5 × 1.7 m
  boxes; cars are 4.4 × 1.8 × 1.5 m boxes. Each is a kinematic body moved
  along a piecewise-linear path.
- **Detector**: removes points below 0.15 m and clusters the rest on a 0.15 m
  grid. The link distance grows with range, so a far car does not split into
  pieces. Each cluster is labelled by footprint and height: person, cyclist,
  car, pole (reported as a landmark), or ignored (walls, pillars). The
  detection is the centre of the cluster's footprint box. Its covariance is
  (0.08 m + 1 % of range)² horizontally. Its score rises with the point count.
- **Ego pose**: the scan always uses the true pose. In `pose_jitter` only the
  pose written to `setup.json` has noise.

## Outputs

Each task gets its own directory:

| File | Contents |
| --- | --- |
| `setup.json` | `map` ← `lidar` pose per scan (noisy in `pose_jitter`) |
| `inputs.jsonl` | One `lidar_batch` per scan: detections and pole landmarks in the LiDAR frame |
| `truth.jsonl` | One `truth3d` per scan: every object within 35 m, `map` frame |
| `summary.json` | Detector recall and precision against the truth |

`tasks.json` at the top level lists every task's summary.

## Detector results

A detection is a hit when it lies within 0.6 m (person), 1.0 m (cyclist) or
2.0 m (car) of a truth object of the same class. Each is matched at most once.

| Task | Recall | Precision | Median points per person |
| --- | --- | --- | --- |
| people_basic | 0.979 | 1.000 | 22 |
| crowd_crossing | 0.807 | 1.000 | 24 |
| occlusion | 0.674 | 1.000 | 24 |
| maneuvering | 0.949 | 1.000 | 24 |
| moving_robot | 0.967 | 0.985 | 70 |
| pose_jitter | 0.967 | 0.984 | 70 |
| mixed_traffic | 0.631 | 0.556 | 12 |
| long_range | 0.889 | 1.000 | 6 |
| side_by_side | 0.845 | 0.993 | 24 |

`mixed_traffic` shows the limits of a geometric detector:

- A cyclist seen end-on has a person's footprint.
- A car partly hidden by a cyclist or pedestrian breaks into pieces, which are
  labelled as people or cyclists.

`side_by_side` and `crowd_crossing` lose recall wherever two people are less
than about 0.3 m apart as seen from the sensor: they merge into one cluster.
Tracker scores on these outputs are in robo-perception's
`docs/simulation.md`.

## Limits

- Bodies are rigid: no limbs, gait or clothing. People are capsules.
- Movers pass through one another. Crowd lanes are 0.5 m apart, so they do
  not overlap.
- Each scan is instantaneous: no motion distortion within a sweep.
- Ground removal uses the known flat ground.
- There is no camera. Camera-LiDAR fusion tasks need rendered images.
