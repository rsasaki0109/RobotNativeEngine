# Go2 among people (closed loop)

Perception for a robot is good when the robot behaves well. In this example a
Go2 walks among people, and what it perceives decides how it walks. The score
is its behaviour: contacts, closest approaches, time to the goal, and whether
it follows the right person.

```bash
cargo run --release -p go2_people --example 151_go2_people
cargo run --release -p go2_people --example 151_go2_people -- --task corridor --out target/go2_people
# with robot-perception (https://github.com/rsasaki0109/robot-perception) as the tracker:
cargo run --release -p go2_people --example 151_go2_people -- --out target/go2_people --tracker \
    /path/to/robot-perception stream --setup target/go2_people/stream_setup.json \
    --max-association-distance-m 1.0 --recent-track-window-ns 0
```

The 9 runs (3 tasks × 3 perception sources) take about 70 s on 4 cores and
are deterministic. Each run writes a per-frame trace
(`<task>_<source>.jsonl`) and a line of `results.json`.

## Robot, sensor, controller

- **Robot**: the welded Go2 (`unitree_go2_jump`) on `rne_ai`'s model-based
  trot, at the speed and turn rate validated in examples 126 and 130
  (0.25 m/s, 0.5 rad/s). Commands are rate limited.
- **Sensor**: the upside-down Livox Mid-360 with the measured Go2 rig
  occlusion (`docs/LIVOX_MID360.md`), at 10 Hz. It looks mostly down, so a
  person 2 m away shows only from the shins to the knees.
- **Detection**: returns 0.15–1.9 m above the floor are clustered on a
  0.15 m grid. A person is at most 1 m wide, spans at least 0.2 m of height
  and has at least 4 returns. The scan is expressed in the base frame with
  the true attitude, as a perfect de-skew and IMU would give.
- **Odometry**: the controller works in the odometry frame, which has leg
  odometry's 4 % scale error and a gyro-grade yaw-rate bias (0.002 rad/s).
- **Controller**:
  - It waits when a crossing person's predicted path passes closer than
    0.9 m.
  - It changes lanes early, at walking speed, for someone coming head-on or
    standing in the way. It aims 1.1 m clear of the line that person walks
    along, and keeps its chosen side.
  - In `follow` it keeps one track as the target, 1.2 m behind it. When that
    track is lost, it takes the track nearest the target's predicted
    position.

## Perception sources

| Source | What the controller gets |
| --- | --- |
| `oracle` | True positions, velocities and identities within 12 m (perfect perception) |
| `nearest` | Mid-360 detections, greedy nearest-neighbour association, smoothed velocity |
| `tracker` | The same detections through an external tracker speaking robot-perception's JSONL stream (one `lidar_batch` in, one `lidar_frame` out, in lockstep) |

## Tasks

| Task | Mission | People |
| --- | --- | --- |
| `follow` | Follow a person 1.2 m behind through two turns and a 4 s stop | One cuts between the robot and its target, one walks past |
| `crossing` | Walk 12 m to a goal | Four cross the path (0.7–1.1 m/s), one diagonally |
| `corridor` | Walk 15 m down a 3.2 m wide corridor | Three come the other way (0.7–0.9 m/s) |

## Results

| task | perception | fell | contacts | closest (m) | < 0.8 m (s) | goal (s) | left to goal (m) | follow error (m) | wrong target | lost |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| follow | oracle | no | 0 | 0.90 | 0.00 | - | - | 0.25 | 0.00 | 0.00 |
| follow | nearest | no | 0 | 1.07 | 0.00 | - | - | 0.38 | 0.00 | 0.00 |
| follow | tracker | no | 0 | 1.06 | 0.00 | - | - | 0.38 | 0.00 | 0.00 |
| crossing | oracle | no | 0 | 0.84 | 0.00 | 52.90 | 1.02 | - | - | - |
| crossing | nearest | no | 0 | 0.87 | 0.00 | 53.30 | 1.00 | - | - | - |
| crossing | tracker | no | 0 | 0.89 | 0.00 | 53.30 | 1.00 | - | - | - |
| corridor | oracle | no | 0 | 0.77 | 0.47 | 64.30 | 1.33 | - | - | - |
| corridor | nearest | no | 2 | 0.38 | 2.91 | 69.70 | 1.26 | - | - | - |
| corridor | tracker | no | 1 | 0.52 | 1.21 | 65.40 | 1.25 | - | - | - |

A contact is a person's centre within 0.55 m of the base's centre. "Goal" is
when the robot believes it has arrived; "left to goal" is the true remaining
distance. It is about 1 m in every run, the 4 % odometry scale error over
12–15 m: a localization problem, not a perception one.

- **Following and crossing** work with every source. The trackers follow
  0.13 m less precisely than the oracle, from their velocity lag. They never
  switch to the person cutting between or to the one walking past.
- **The corridor separates the sources.** With perfect perception the robot
  passes everyone at 0.77 m or more. With Mid-360 detections it touches
  people: twice with the nearest-neighbour tracker and once with
  robot-perception. The cause is visible in the traces. The trackers report
  3.6–4.2 tracks per frame where about one person is present, because pieces
  of the corridor walls pass the person test. The controller then changes
  lanes for phantom people by the walls. Detection false positives, more than
  tracking, decide the outcome. robot-perception, which confirms tracks more
  carefully, comes closer to the oracle.

## Development notes

The numbers above are after these fixes, each found from the traces:

- Example 131's leg-odometry yaw bias (0.02 rad/s) is corrected there by
  SLAM. Uncorrected here, it curved the robot 0.8 rad off course in 40 s,
  into a wall (a fall) and into people's paths. A gyro-grade bias replaces it.
- At 0.4 m/s the Go2 fell within seconds. The validated 0.25 m/s does not.
- Lane changes first slowed to 0.1 m/s, and the robot then moved sideways at
  8 cm/s. Measuring the person's offset from the robot's own heading also
  misjudged people straight down the corridor whenever the robot was angled.

## Limits

- **People**: kinematic capsules on scripted paths. They do not react to the
  robot, and their colliders are sensors, so contacts are counted from
  distance and the robot is never pushed.
- **Robot motion**: the trot takes forward speed and yaw rate only, with no
  sidestepping.
- **Perception**: the scan uses the true attitude and a perfect de-skew. The
  robot has no map, so walls are only what the detector makes of them.
- **Not covered**: the camera, VLM cues and SLAM. Combining them with this
  loop is the next step.
