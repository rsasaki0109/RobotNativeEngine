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

One pass (4 tasks × 8 perception sources) takes about 4 minutes on 4 cores
and is deterministic. `--trials N` repeats it with each person (but a follow
target) starting up to 2 s earlier or later, from a seed, and prints a
summary over the trials. Each run writes a per-frame trace
(`<task>_<source>.jsonl`, with a `_t<trial>` suffix after the first trial) and a line of `results.json`. The trace records the tracks, the true people and the controller's lane decision.

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
    along. Someone drifting sideways (by the track's mean velocity, across
    the way to the goal) is passed behind. Otherwise the robot keeps to its
    side of the person's line, and holds that side once the track is 1 s
    old.
  - In `follow` it keeps one track as the target, 1.2 m behind it. When that
    track is lost, it takes the track nearest the target's predicted
    position.

## Perception sources

| Source | What the controller gets |
| --- | --- |
| `oracle` | True positions, velocities and identities within 12 m (perfect perception) |
| `oracle+slam` | The same, but the robot localizes and plans in the SLAM frame. It separates what SLAM does to the path from what it does to perception |
| `nearest` | Mid-360 detections, greedy nearest-neighbour association, smoothed velocity |
| `tracker` | The same detections through an external tracker speaking robot-perception's JSONL stream (one `lidar_batch` in, one `lidar_frame` out, in lockstep) |
| `nearest+map`, `tracker+map` | The same, after dropping detections on static structure (below) |
| `nearest+slam`, `tracker+slam` | The same as `+map`, localized by 2D SLAM instead of odometry alone (below) |

**Static map.** The robot builds a grid of 0.2 m cells in its odometry
frame from its own scans; no map is given. Each cell records when it was
first hit in the person band and in how many frames since. A cell is static
once it was first seen at least 6 s ago and has been hit in at least 15
frames. Walls are seen from afar long before the robot passes them. The
Mid-360's pattern hits a given cell only now and then, so a plain frame
count misses most of the wall. A cluster with more than half its returns on
static cells is dropped. A person who stands still for less than 6 s is not
absorbed.

**SLAM.** `rne_slam`'s 2D SLAM is set up as in example 131. Its scan holds
the returns 0.15–0.65 m above the floor, minus those within 0.5 m of a
detected person. The robot then perceives and acts in the SLAM-corrected
frame. Two safeguards came from failures:

- Run plainly, SLAM made localization worse. On open floor nothing static is
  in the scan but people, and the heading flipped by π. In the corridor the
  two parallel walls do not pin the motion along it, and matching slid 4 m.
- Scan-matching corrections are capped at 5 cm and 0.02 rad per keyframe.
- Degenerate scans are skipped. The eigenvalue ratio of the scan's
  surface-normal information measures it: 0.11–0.21 in the corridor and
  0.41–0.96 in the room; the threshold is 0.3.

| final localization error | odometry | SLAM |
| --- | --- | --- |
| crossing (open floor) | 0.69 m | 0.69 m (no structure to match) |
| corridor | 0.62 m | 0.38 m |
| room | 0.61 m | 0.04 m |

## Tasks

| Task | Mission | People |
| --- | --- | --- |
| `follow` | Follow a person 1.2 m behind through two turns and a 4 s stop | One cuts between the robot and its target, one walks past |
| `crossing` | Walk 12 m to a goal | Four cross the path (0.7–1.1 m/s), one diagonally |
| `corridor` | Walk 15 m down a 3.2 m wide corridor | Three come the other way (0.7–0.9 m/s) |
| `room` | Cross a furnished 14 m × 9 m room (walls, two tables, a shelf, two pillars) to a goal 11 m away | Three walk through it, one diagonally toward the robot |

## Results

Eight trials per task and source (`--trials 8`), with the rule controller
(the default):

| task | perception | runs | fell | runs with contact | contacts | closest, mean (m) | closest, min (m) | < 0.8 m, mean (s) | left to goal, mean (m) | follow error (m) | wrong target | lost |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| follow | oracle | 8 | 0 | 0 | 0 | 0.90 | 0.90 | 0.00 | - | 0.26 | 0.00 | 0.00 |
| follow | oracle+slam | 8 | 0 | 0 | 0 | 0.90 | 0.90 | 0.00 | - | 0.26 | 0.00 | 0.00 |
| follow | nearest | 8 | 0 | 1 | 1 | 0.96 | 0.54 | 0.93 | - | 0.55 | 0.12 | 0.07 |
| follow | nearest+map | 8 | 0 | 1 | 1 | 0.96 | 0.54 | 0.96 | - | 0.55 | 0.12 | 0.07 |
| follow | nearest+slam | 8 | 0 | 1 | 1 | 0.96 | 0.54 | 0.96 | - | 0.55 | 0.12 | 0.07 |
| follow | tracker | 8 | 0 | 0 | 0 | 1.02 | 0.61 | 0.40 | - | 0.47 | 0.06 | 0.03 |
| follow | tracker+map | 8 | 0 | 0 | 0 | 1.02 | 0.61 | 0.40 | - | 0.47 | 0.06 | 0.04 |
| follow | tracker+slam | 8 | 0 | 0 | 0 | 1.02 | 0.61 | 0.40 | - | 0.47 | 0.06 | 0.04 |
| crossing | oracle | 8 | 0 | 0 | 0 | 0.86 | 0.72 | 0.10 | 1.00 | - | - | - |
| crossing | oracle+slam | 8 | 0 | 0 | 0 | 0.86 | 0.72 | 0.10 | 1.00 | - | - | - |
| crossing | nearest | 8 | 0 | 0 | 0 | 0.94 | 0.76 | 0.07 | 0.99 | - | - | - |
| crossing | nearest+map | 8 | 0 | 0 | 0 | 0.94 | 0.76 | 0.07 | 0.99 | - | - | - |
| crossing | nearest+slam | 8 | 0 | 0 | 0 | 0.94 | 0.76 | 0.07 | 0.99 | - | - | - |
| crossing | tracker | 8 | 0 | 0 | 0 | 0.95 | 0.77 | 0.06 | 1.01 | - | - | - |
| crossing | tracker+map | 8 | 0 | 0 | 0 | 0.95 | 0.77 | 0.06 | 1.01 | - | - | - |
| crossing | tracker+slam | 8 | 0 | 0 | 0 | 0.95 | 0.77 | 0.06 | 1.01 | - | - | - |
| corridor | oracle | 8 | 0 | 0 | 0 | 0.73 | 0.67 | 0.60 | 1.28 | - | - | - |
| corridor | oracle+slam | 8 | 0 | 0 | 0 | 0.73 | 0.67 | 0.60 | 1.10 | - | - | - |
| corridor | nearest | 8 | 0 | 4 | 4 | 0.56 | 0.28 | 1.79 | 1.29 | - | - | - |
| corridor | nearest+map | 8 | 1 | 6 | 7 | 0.49 | 0.20 | 1.81 | 1.41 | - | - | - |
| corridor | nearest+slam | 8 | 0 | 6 | 7 | 0.49 | 0.20 | 1.81 | 1.24 | - | - | - |
| corridor | tracker | 8 | 0 | 5 | 5 | 0.53 | 0.43 | 1.80 | 1.32 | - | - | - |
| corridor | tracker+map | 8 | 0 | 1 | 1 | 0.61 | 0.51 | 1.76 | 1.39 | - | - | - |
| corridor | tracker+slam | 8 | 0 | 1 | 1 | 0.61 | 0.51 | 1.76 | 1.27 | - | - | - |
| room | oracle | 8 | 0 | 0 | 0 | 1.02 | 0.98 | 0.00 | 0.94 | - | - | - |
| room | oracle+slam | 8 | 0 | 0 | 0 | 1.01 | 0.99 | 0.00 | 0.42 | - | - | - |
| room | nearest | 8 | 0 | 0 | 0 | 0.92 | 0.79 | 0.03 | 0.97 | - | - | - |
| room | nearest+map | 8 | 0 | 0 | 0 | 0.96 | 0.87 | 0.00 | 0.95 | - | - | - |
| room | nearest+slam | 8 | 0 | 0 | 0 | 0.92 | 0.75 | 0.09 | 0.43 | - | - | - |
| room | tracker | 8 | 0 | 0 | 0 | 0.95 | 0.86 | 0.00 | 0.97 | - | - | - |
| room | tracker+map | 8 | 0 | 0 | 0 | 0.90 | 0.82 | 0.00 | 0.94 | - | - | - |
| room | tracker+slam | 8 | 0 | 0 | 0 | 0.96 | 0.84 | 0.00 | 0.42 | - | - | - |

One run falls (`nearest+map` in the corridor). A contact is a person's centre within 0.55 m of the base's
centre. "Left to goal" is the true distance when the robot believes it has
arrived.

- **Perfect perception never touches anyone** in 64 runs, with odometry or
  SLAM. Every contact below comes from perception.
- **Following**: robot-perception follows with no contact. It is on the
  wrong person 6 % of the time, against 12 % and one contact for the
  nearest-neighbour tracker. The map and SLAM play no part on open floor.
- **Crossing** is safe with every source.
- **Corridor**: wall pieces become phantom people. robot-perception with the
  static map (or SLAM) is the best real perception: 1 of 8 runs with a
  contact, against 5 of 8 without the map. The map does not help the
  nearest-neighbour tracker (4 → 6 of 8). It removes the phantoms, but that
  tracker's velocities are too noisy to plan a lane change around the people
  who remain.
- **Room**: no contacts with any source. SLAM localizes to 4 cm and halves
  the goal error (0.94 → 0.42 m).

### How the room contacts were found

Before the pass-side fix, 3 of 8 SLAM runs in the room touched
someone, and the trackers without the map 2 of 8. `oracle+slam` touched
nobody, so SLAM's path alone was not the cause. The traces (which record
tracks, true people and the controller's lane decision) showed that
localization (3 cm) and tracking (within 0.1 m of the truth) were fine.
The controller was at fault:

- It chose the side to pass a head-on person on from the robot's offset
  to that person's line, and kept it.
- Five metres away, a few degrees of velocity error move that line by a
  metre. A new track's first velocity put the diagonal walker's line on the
  wrong side, and the robot crossed in front of them.
- Perfect perception has no such error. SLAM only moved the robot to where
  this happened.

Two changes fixed it:

- Someone drifting across is passed behind, judged by the track's mean
  velocity.
- No side is kept from a track younger than 1 s.

A first version measured drift against the robot's heading. Mid lane change,
that made people walking straight down the corridor look like drifting, and
all 8 corridor runs touched someone, even with perfect perception. Drift is
now measured across the way to the goal.

In the corridor the fix moved each count by about one run of eight:
`tracker+map` 2 → 1, `tracker` 4 → 5, `nearest` 3 → 4. Following and
crossing did not change.

## Dynamic-window controller

`--controller dwa` replaces the rules with a dynamic window. Every pair of
6 forward speeds and 11 turn rates is rolled out for 3 s as a unicycle,
against each tracked person moving at its tracked velocity. Predicted
clearance under 0.75 m is all but forbidden, and clearance under 1.5 m costs
extra. Then come the distance to the aim, the heading to it, and changes of
turn rate. Over the same 8 trials it is worse overall, and the rule
controller stays the default (DWA as of this write-up; the rule rows are
after the pass-side fix):

| runs with contact (fell) of 8 | follow | crossing | corridor | room |
| --- | --- | --- | --- | --- |
| oracle, rule | 0 | 0 | 0 | 0 |
| oracle, DWA | 0 | 0 | 0 | 4 |
| oracle+slam, DWA | 0 | 0 | 0 | 7 |
| tracker+map, rule | 0 | 0 | 1 | 0 |
| tracker+map, DWA | 0 | 1 | 0 (3 fell) | 3 |
| nearest+map, DWA | 0 | 0 | 0 (2 fell) | 5 |

- **Corridor**: no contacts with any source, against 1–6 of 8 with the
  rules. But 13 of 48 runs with real perception fall. The window knows only
  people, not walls. A phantom person or a noisy velocity makes it swerve,
  and nothing stops the swerve at the wall. With perfect perception there
  are no phantoms and no falls.
- **Room**: contacts even with perfect perception. The window may plan to
  pass 0.75 m from a person's predicted position. The diagonal walker's
  constant-velocity prediction is only a few tenths of a metre off, and
  that is enough to come within 0.55 m. The rules wait for crossers, and
  that is what keeps the room safe.
- **Following** is safe but looser: the follow error rises from 0.26 to
  0.56 m with perfect perception.

To beat the rules, the window would need the scan's obstacles as well as
people, and a wider margin for people moving across its path.

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
- **Static map**: drift in the odometry frame smears it slowly. A person
  standing still longer than 6 s becomes part of it.
- **SLAM**: scan matching only; no IMU fusion, no relocalization.
- **DWA**: people only; walls and furniture are not obstacles to it.
- **Not covered**: the camera and VLM cues. Combining them with this loop is
  a next step.
