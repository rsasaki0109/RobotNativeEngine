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

One pass (5 tasks × 8 perception sources) takes about 6 minutes on 4 cores
and is deterministic. `--trials N` repeats it with each person (but a follow
target) starting up to 2 s earlier or later, from a seed, and prints a
summary over the trials. Each run writes a per-frame trace
(`<task>_<source>.jsonl`, with a `_t<trial>` suffix after the first trial) and a line of `results.json`. The trace records the tracks, the true people and the controller's lane decision.

## Robot, sensor, controller

- **Robot**: the welded Go2 (`unitree_go2_jump`) on `rne_ai`'s model-based
  trot, at the speed and turn rate validated in examples 126 and 130
  (0.25 m/s, 0.5 rad/s). A signed speed of −0.1 m/s was also tested for
  10 s with no fall, less than 0.2 rad of heading drift and more than
  0.5 m of backward travel. Commands are rate limited.
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
    along, from the track's velocity smoothed over about 1 s. Someone
    drifting sideways (across the way to the goal) is passed behind.
    Otherwise the robot keeps to its side of the person's line, and holds
    that side once the track is 1 s old.
  - A crowd check (below) weighs everyone at once: the rules above look at
    one person at a time.
  - A safety layer for the static map (below) stops forward motion.
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

**Safety layer.** A person who stands still for longer is absorbed: the
map cannot tell them from a pillar. Detections there are dropped, and the
robot would walk into them. So, with the map, the static returns are
clustered again. While a cluster that passes the person test has its centre
0.3–1.25 m ahead and within 0.45 m of the robot's axis, the robot does not
walk forward, and for 1 s after. It turns away and can back up when the
cluster is close and the rear is clear.
Such clusters up to 3 m ahead are also given to the controller as standing
people, so that it plans around them. Walls fail the person test.
`--no-safety` turns the layer off. Three earlier versions failed, and the reasons shape
this one:

- Stopping for any return 0.55–1.0 m ahead stopped the robot in front of
  people coming head-on, who do not react to it, and for corridor walls
  when it was angled mid lane change. Corridor runs with a contact went from
  0–1 of 8 to 6–8 of 8.
- Stopping for any static return still stopped for the walls.
- Counting static returns in a box flickered in the SLAM frame, where the
  cells shift with each correction. The rate-limited robot crept on between
  flickers, and closer than the box's near edge nothing blocked at all.

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
| `crowd` | Cross a 20 m × 9.6 m hall to a goal 14 m away | Nine: three head-on, two crossing, one diagonal, one overtaking from behind, and a pair standing in the way for 55 s, long enough to join the static map |

## Results

The table records the forward-only baseline, with eight trials per task
and source (`--trials 8`) using the default rule controller:

| task | perception | runs | fell | runs with contact | contacts | closest, mean (m) | closest, min (m) | < 0.8 m, mean (s) | left to goal, mean (m) | follow error (m) | wrong target | lost | safety stop, mean (s) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| follow | oracle | 8 | 0 | 0 | 0 | 0.91 | 0.90 | 0.00 | - | 0.26 | 0.00 | 0.00 | 0.00 |
| follow | oracle+slam | 8 | 0 | 0 | 0 | 0.91 | 0.90 | 0.00 | - | 0.26 | 0.00 | 0.00 | 0.00 |
| follow | nearest | 8 | 0 | 0 | 0 | 0.97 | 0.61 | 0.47 | - | 0.52 | 0.12 | 0.07 | 0.00 |
| follow | nearest+map | 8 | 0 | 0 | 0 | 0.98 | 0.64 | 0.39 | - | 0.53 | 0.12 | 0.07 | 3.89 |
| follow | nearest+slam | 8 | 0 | 0 | 0 | 0.98 | 0.64 | 0.39 | - | 0.53 | 0.12 | 0.07 | 3.89 |
| follow | tracker | 8 | 0 | 0 | 0 | 1.02 | 0.67 | 0.22 | - | 0.46 | 0.06 | 0.03 | 0.00 |
| follow | tracker+map | 8 | 0 | 0 | 0 | 1.03 | 0.72 | 0.21 | - | 0.47 | 0.06 | 0.03 | 4.31 |
| follow | tracker+slam | 8 | 0 | 0 | 0 | 1.03 | 0.72 | 0.21 | - | 0.47 | 0.06 | 0.03 | 4.31 |
| crossing | oracle | 8 | 0 | 0 | 0 | 0.86 | 0.72 | 0.10 | 1.00 | - | - | - | 0.00 |
| crossing | oracle+slam | 8 | 0 | 0 | 0 | 0.86 | 0.72 | 0.10 | 1.00 | - | - | - | 0.00 |
| crossing | nearest | 8 | 0 | 0 | 0 | 0.97 | 0.77 | 0.06 | 1.00 | - | - | - | 0.00 |
| crossing | nearest+map | 8 | 0 | 0 | 0 | 0.97 | 0.77 | 0.06 | 1.00 | - | - | - | 0.00 |
| crossing | nearest+slam | 8 | 0 | 0 | 0 | 0.97 | 0.77 | 0.06 | 1.00 | - | - | - | 0.00 |
| crossing | tracker | 8 | 0 | 0 | 0 | 0.98 | 0.77 | 0.06 | 1.02 | - | - | - | 0.00 |
| crossing | tracker+map | 8 | 0 | 0 | 0 | 0.98 | 0.77 | 0.06 | 1.02 | - | - | - | 0.00 |
| crossing | tracker+slam | 8 | 0 | 0 | 0 | 0.98 | 0.77 | 0.06 | 1.02 | - | - | - | 0.00 |
| corridor | oracle | 8 | 0 | 0 | 0 | 0.77 | 0.74 | 0.37 | 1.33 | - | - | - | 0.00 |
| corridor | oracle+slam | 8 | 0 | 0 | 0 | 0.77 | 0.74 | 0.37 | 1.22 | - | - | - | 0.00 |
| corridor | nearest | 8 | 0 | 0 | 0 | 0.74 | 0.65 | 0.61 | 1.16 | - | - | - | 0.00 |
| corridor | nearest+map | 8 | 0 | 0 | 0 | 0.74 | 0.64 | 0.54 | 1.24 | - | - | - | 0.15 |
| corridor | nearest+slam | 8 | 0 | 0 | 0 | 0.74 | 0.64 | 0.54 | 1.07 | - | - | - | 0.00 |
| corridor | tracker | 8 | 0 | 0 | 0 | 0.70 | 0.57 | 0.74 | 1.23 | - | - | - | 0.00 |
| corridor | tracker+map | 8 | 0 | 1 | 1 | 0.73 | 0.48 | 0.55 | 1.26 | - | - | - | 0.12 |
| corridor | tracker+slam | 8 | 0 | 1 | 1 | 0.73 | 0.48 | 0.55 | 1.09 | - | - | - | 0.12 |
| room | oracle | 8 | 0 | 0 | 0 | 1.01 | 0.97 | 0.00 | 0.94 | - | - | - | 0.00 |
| room | oracle+slam | 8 | 0 | 0 | 0 | 1.01 | 0.99 | 0.00 | 0.42 | - | - | - | 0.00 |
| room | nearest | 8 | 0 | 0 | 0 | 0.98 | 0.92 | 0.00 | 0.96 | - | - | - | 0.00 |
| room | nearest+map | 8 | 0 | 0 | 0 | 0.99 | 0.88 | 0.00 | 0.97 | - | - | - | 0.00 |
| room | nearest+slam | 8 | 0 | 0 | 0 | 0.98 | 0.94 | 0.00 | 0.43 | - | - | - | 0.00 |
| room | tracker | 8 | 0 | 0 | 0 | 0.97 | 0.91 | 0.00 | 0.98 | - | - | - | 0.00 |
| room | tracker+map | 8 | 0 | 0 | 0 | 0.93 | 0.79 | 0.04 | 0.94 | - | - | - | 0.00 |
| room | tracker+slam | 8 | 0 | 0 | 0 | 1.01 | 0.94 | 0.00 | 0.42 | - | - | - | 0.00 |
| crowd | oracle | 8 | 0 | 0 | 0 | 0.87 | 0.82 | 0.00 | 1.19 | - | - | - | 0.00 |
| crowd | oracle+slam | 8 | 0 | 0 | 0 | 0.75 | 0.71 | 0.78 | 0.43 | - | - | - | 0.00 |
| crowd | nearest | 8 | 0 | 3 | 3 | 0.55 | 0.16 | 1.48 | 1.15 | - | - | - | 0.00 |
| crowd | nearest+map | 8 | 0 | 4 | 4 | 0.40 | 0.04 | 1.11 | 1.21 | - | - | - | 2.01 |
| crowd | nearest+slam | 8 | 0 | 3 | 6 | 0.55 | 0.14 | 2.47 | 0.43 | - | - | - | 7.80 |
| crowd | tracker | 8 | 0 | 3 | 4 | 0.64 | 0.28 | 0.89 | 1.20 | - | - | - | 0.00 |
| crowd | tracker+map | 8 | 0 | 1 | 1 | 0.72 | 0.02 | 0.62 | 1.14 | - | - | - | 0.65 |
| crowd | tracker+slam | 8 | 0 | 1 | 1 | 0.68 | 0.26 | 3.02 | 0.43 | - | - | - | 5.64 |

No run falls. A contact is a person's centre within 0.55 m of the base's
centre. "Left to goal" is the true distance when the robot believes it has
arrived.

- **Perfect perception touches nobody** in 80 runs, with odometry or SLAM.
- **Following, crossing, room**: no contacts with any source.
  robot-perception is on the wrong person 6 % of the time when following,
  against 12 % for the nearest-neighbour tracker.
- **Corridor**: two contacts in 48 runs with real perception
  (robot-perception with the map, odometry and SLAM, in the same trial).
- **Crowd**: here the sources separate. robot-perception with the static
  map (with or without SLAM) has a contact in 1 of 8 runs. Without the map,
  or with the nearest-neighbour tracker, it is 3–4 of 8. The crowd check
  does not change that against scripted people (below).
  - The standing pair joins the static map. Without the safety layer, the
    robot walked into them in 7 of 8 map runs. With it, the robot waits
    about 10 s in front of them.
  - Without the crowd check, every remaining contact is with a person who was being tracked. 14 of
    16 happen at 20–24 s, when a head-on walker, a crosser and the
    overtaker are all within 2 m. The robot stops or slows for the crosser,
    and the head-on walker (11) or the overtaker (3) walks into it. With
    perfect perception the robot reaches that moment at a different time
    and never touches anyone. Track noise changes when it waits.
  - The crowd was first written with the diagonal walker and a second
    head-on walker in the same few seconds. Even perfect perception touched
    someone there, which measures the controller, not perception. They were
    spaced until perfect perception passed 16 of 16 runs.

### Crowd check

The rules weigh one person at a time, so the crowd's contacts came from
waiting for one person while another walked in. The crowd check rolls the
rules' command out for 3 s against every track (at constant velocity, a
follow target excepted). It tries stop and full forward speed, each with
hard left, hard right, straight, and the rules' turn. If the rear scan
is clear, it also tries backing up at −0.1 m/s going straight or using
the rules' turn. The same wall clearance check applies. The rules'
command stands unless it comes within 0.75 m of someone and an alternative
stays clear by 0.2 m more. `--no-crowd-check` turns it off.

| runs with contact of 8 (crowd) | without | first version | speed only | wall-checked forward |
| --- | --- | --- | --- | --- |
| scripted, nearest+map | 4 | 2 | 4 | 4 |
| scripted, tracker | 3 | 3 | 3 | 3 |
| scripted, tracker+map | 1 | 1 | 1 | 1 |
| reactive, nearest+map | 2 (1 stuck) | 0 | 2 (4 stuck) | 0 (1 stuck) |
| reactive, tracker+map | 1 (1 stuck) | 1 | 0 (1 stuck) | 0 (1 stuck) |
| corridor runs that fell (scripted, real perception), of 48 | 0 | 7 | 0 | 0 |

- **First version**: the rollout knew only people. Its hard turns cleared
  them and drove the robot into the corridor's walls: 7 falls in 48 real
  perception runs, and contacts in 3 sources.
- **Speed only** (stop, half or full, keeping the rules' turn): no falls,
  and nothing gained; against reactive people 4 nearest+map runs got stuck.
- **Wall-checked forward**: the turning alternatives again, but none that come within
  0.45 m of a scan return that is not a person (and closer than the rules'
  command does). No falls; against reactive people, the map sources'
  contacts go from 1–2 of 8 to 0. Against scripted people nothing changes:
  the head-on walker and the overtaker still walk into a robot that has
  little room to turn away in time.
- The first version had freed the stuck runs by turning toward whatever
  was free, walls included. The wall-checked forward rollout left them stuck.

### How the contacts were found

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

- Someone drifting across is passed behind, judged by the track's
  averaged velocity.
- No side is kept from a track younger than 1 s.

A first version measured drift against the robot's heading. Mid lane change,
that made people walking straight down the corridor look like drifting, and
all 8 corridor runs touched someone, even with perfect perception. Drift is
now measured across the way to the goal.

The corridor came next. Even with the static map, 1 of 8
robot-perception runs touched someone, and 5 of 8 without it. Again the
tracking was right: the person came head-on, 7 cm from the robot's lane.
But one frame's track velocity swung sideways by ±0.3 m/s. At 6 m that
moves the line the person walks along by up to 2 m. The robot kept
starting its lane change and calling it done, and after 6 s it was only
0.57 m across. The lane geometry now uses the same smoothed velocity: the
plain mean over a track's first frames, then a moving average of about
1 s.

| runs with contact of 8 | corridor before | corridor after | room before | room after |
| --- | --- | --- | --- | --- |
| robot-perception | 4 | 1 | 2 | 0 |
| robot-perception + map | 2 | 0 | 0 | 0 |
| robot-perception + map + SLAM | 2 | 0 | 3 | 0 |
| nearest neighbour | 3 | 0 | 2 | 0 |
| nearest neighbour + map | 6 | 0 | 0 | 0 |
| nearest neighbour + map + SLAM | 6 | 0 | 3 | 0 |

"Before" is the controller as first written; "after" has both fixes. The
contacts looked like perception failures (phantoms, SLAM) but were the
controller trusting single frames of correct, noisy tracks. A tracker's
velocity noise matters to the robot through decisions like this one, which
the tracking metrics do not show.

## Reactive people

The people above walk their scripts whatever the robot does, so a robot that
stops is walked into. With `--reactive-people`, a walking person slows when
the robot is in their way (closer than 1.5 m ahead, within 0.7 m of their
line) and stops 0.8 m from it. Each walks their script on their own clock
and resumes where they stopped. Standing people do not react, and nor does
the overtaker unless the robot is in its lane. The forward-only baseline
used eight trials per source:

| runs with contact of 8 | follow | crossing | corridor | room | crowd |
| --- | --- | --- | --- | --- | --- |
| oracle | 0 | 0 | 0 | 0 | 0 |
| oracle+slam | 0 | 0 | 0 | 0 | 0 |
| nearest | 0 | 0 | 0 | 0 | 2 |
| nearest+map | 0 | 0 | 0 | 0 | 0 (1 stuck) |
| nearest+slam | 0 | 0 | 0 | 0 | 0 |
| tracker | 0 | 0 | 0 | 0 | 1 |
| tracker+map | 0 | 0 | 0 | 0 | 0 (1 stuck) |
| tracker+slam | 0 | 0 | 0 | 0 | 0 |

"Stuck" runs did not reach the goal in 120 s.

- Contacts in the crowd fall from 1–4 of 8 to 0–2 of 8. Only the sources
  without the map have any.
- Reactive people exposed a deadlock. A person who stopped for the robot
  stood still for 6 s, joined the static map, and the safety layer then
  held the robot while the person waited for it. 8 of 32 map runs were
  stuck. Two changes cut that to 2: static person-shaped clusters ahead are
  planned around as standing people, and while one blocks the way the
  robot turns in place away from it.
- The 2 stuck runs (both odometry-map sources) are a limit of
  the earlier controller: it crept to 0.55 m of a person who had stopped
  for it. At that range most of their returns fall inside the 0.4 m that
  perception ignores around the base. Turning in place did
  not free it. Close by, the person's distance hovers at the contact
  threshold, and before the crowd check one of those runs counted 9
  contacts. With the wall-checked rollout, neither touched anyone.
- The safety layer now backs away once per block when a static person
  cluster is within 0.8 m ahead and the rear scan is clear. The retreat
  lasts up to 2 s at up to 0.1 m/s, with the turn away from the person.
  Every new scan checks the rear corridor (0.8 m deep, ±0.35 m wide).
  A return more than 0.05 m above the floor in that corridor cancels
  reverse immediately. This check includes returns within 0.4 m of the
  base, which the person detector and map exclude.

## Camera and VLM

`--vlm "<command>"` adds `tracker+map+vlm`: `tracker+map` with a forward
camera and a pretrained VLM. The camera sits on the Go2's head (0.45 m up,
level, 60° vertical field of view) and is rendered on the CPU by
`rne_render_cpu`, with the CC0 human mesh for people. The VLM is a separate
process speaking one JSON line each way (`{"image": path}` in, person boxes
out); robot-perception's `scripts/vlm/vlm_people_server.py` runs
Florence-2's `<OD>` prompt:

```bash
cargo run --release -p go2_people --example 151_go2_people -- --task crowd \
    --sources tracker+map,tracker+map+vlm --out target/go2_people \
    --vlm "python3 /path/to/robot-perception/scripts/vlm/vlm_people_server.py" \
    --tracker /path/to/robot-perception stream --setup target/go2_people/stream_setup.json \
    --max-association-distance-m 1.0 --recent-track-window-ns 0
```

The VLM is asked at most once a second, and only with a person-shaped
static cluster in view within 10 m. Its answer counts 1 s later, about what
Florence-2 takes on a CPU. Static clusters whose image column falls in a
person box go back to the tracker as people for 10 s, instead of being left
to the map. The VLM finds the rendered people reliably, standing pair
included. Eight trials in the crowd:

| crowd, 8 trials | people | runs with contact | safety stop, mean (s) | stuck | VLM queries per run | detections given back per run |
| --- | --- | --- | --- | --- | --- | --- |
| tracker+map | scripted | 1 | 0.60 | 0 | - | - |
| tracker+map+vlm | scripted | 2 | 0.19 | 0 | 43 | 452 |
| tracker+map | reactive | 1 | 11.0 | 1 | - | - |
| tracker+map+vlm | reactive | 1 | 10.5 | 1 | 46 | 451 |

The VLM changes what the tracker sees but not how the robot does, within
the noise of 8 trials. The geometric fixes above already cover the case it
was meant for: a standing person absorbed by the map is planned around and
stopped for whatever the map calls them. The remaining crowd contacts come
from moments when three or four people are close at once, and the stuck run
from a person inside the 0.4 m the scan ignores, where a camera 0.3 m ahead
of the base does not help either. One run costs about 3 minutes instead of
1.

## Dynamic-window controller

`--controller dwa` replaces the rules with a dynamic window. Every pair of
6 forward speeds and 11 turn rates is rolled out for 3 s as a unicycle,
against each tracked person moving at its tracked velocity. Predicted
clearance under 0.75 m is all but forbidden, and clearance under 1.5 m costs
extra. Then come the distance to the aim, the heading to it, and changes of
turn rate. Over the same 8 trials it is worse overall, and the rule
controller stays the default (DWA as of this write-up; the rule rows are
after the fixes above):

| runs with contact (fell) of 8 | follow | crossing | corridor | room |
| --- | --- | --- | --- | --- |
| oracle, rule | 0 | 0 | 0 | 0 |
| oracle, DWA | 0 | 0 | 0 | 4 |
| oracle+slam, DWA | 0 | 0 | 0 | 7 |
| tracker+map, rule | 0 | 0 | 0 | 0 |
| tracker+map, DWA | 0 | 1 | 0 (3 fell) | 3 |
| nearest+map, DWA | 0 | 0 | 0 (2 fell) | 5 |

- **Corridor**: no contacts with any source, against 0–1 of 8 with the
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

- **People**: kinematic capsules on scripted paths. By default they do not
  react to the robot (`--reactive-people` makes them stop for it, but not
  step aside), and their colliders are sensors, so contacts are counted from
  distance and the robot is never pushed.
- **Robot motion**: the trot takes signed longitudinal speed and yaw
  rate. The controller can back up with a clear rear scan. Sidestepping
  would require a lateral trot command.
- **Perception**: the scan uses the true attitude and a perfect de-skew. The
  robot has no map, so walls are only what the detector makes of them.
- **Static map**: drift in the odometry frame smears it slowly. A person
  standing still longer than 6 s becomes part of it. Planning and the
  safety layer then depend on finding a person-shaped static cluster
  in the scan.
- **SLAM**: scan matching only; no IMU fusion, no relocalization.
- **DWA**: people only; walls and furniture are not obstacles to it.
- **Crowd check**: people move at constant velocity in its rollout; walls
  are the scan returns of the moment, within 2 m. Reverse depends on
  what the rear LiDAR can see in the latest frame.
- **Camera**: one standing pose for every person; the image is rendered from
  the true pose, and the VLM sees no motion blur or exposure changes.
