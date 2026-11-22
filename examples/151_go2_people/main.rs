//! A Go2 among moving people, closed loop: what it perceives with its Mid-360
//! decides how it walks, and the score is how it behaves.
//!
//! Each task puts the welded Go2 (`unitree_go2_jump` on the model-based trot
//! of `rne_ai`) among scripted pedestrians and gives it a mission:
//!
//! * `follow`: walk 1.2 m behind one person through turns and a stop, while
//!   another person cuts between them and a third walks past;
//! * `crossing`: walk 12 m to a goal while four people cross its path;
//! * `corridor`: walk down a 3.2 m wide corridor while people come the other way.
//!
//! The robot sees the people only through its upside-down Livox Mid-360
//! (`rne_sensor::sample_livox_mid360`, the measured Go2 rig occlusion): each
//! 0.1 s frame is clustered into person detections, which a tracker turns
//! into tracks with velocities in the robot's odometry frame (leg odometry with
//! a 4 % scale error and a gyro-grade yaw-rate bias). The same controller then runs on
//! one of three perception sources:
//!
//! * `oracle`: the true positions and velocities (perfect perception);
//! * `nearest`: a greedy nearest-neighbour tracker built in here;
//! * `tracker`: any external tracker speaking the robot-perception JSONL
//!   stream protocol, passed after `--tracker`.
//!
//! The controller yields to people whose predicted path crosses its own, steers
//! around people coming head-on, and in `follow` keeps one track as its target,
//! re-acquiring it near its predicted position when the track is lost.
//!
//! Scores come from the simulation's truth: contacts (a person's centre within
//! 0.55 m of the base's), the closest approach, time closer than 0.8 m, time
//! to the goal, and for `follow` the distance error and the time spent
//! following the wrong person or nobody.
//!
//! People are kinematic capsules (1.64 m, 0.22 m radius) on scripted paths:
//! they do not react to the robot and their colliders are sensors, so the
//! robot is never pushed; contacts are counted from distance.
//!
//! ```text
//! cargo run --release -p go2_people --example 151_go2_people
//! cargo run --release -p go2_people --example 151_go2_people -- --task follow --out target/go2_people
//! cargo run --release -p go2_people --example 151_go2_people -- --tracker /path/to/robot-perception stream \
//!     --setup target/go2_people/stream_setup.json --max-association-distance-m 1.0 --recent-track-window-ns 0
//! ```

mod perception;

use std::f64::consts::PI;
use std::fs;
use std::path::{Path, PathBuf};

use perception::{detect_people, ExternalTracker, NearestTracker, Oracle, Track};
use rne_ai::{
    unitree_go2_mid360_mount, UnitreeGo2ModelTrot, UnitreeGo2TrotCommand, UrdfSceneSim,
    UNITREE_GO2_MODEL_TROT_CONTROL_HZ,
};
use rne_ecs::{spawn_named, Entity};
use rne_math::{Quat, Vec3};
use rne_nav::{FrameId, LaserScan2d, OccupancyGrid, Pose2d};
use rne_physics::{Collider, ColliderShape, RigidBody, RigidBodyType};
use rne_sensor::{
    livox_mid360_spec, LidarRigOcclusion, LidarSpec, LidarSweep, LivoxMid360Pattern, SensorNoiseKey,
};
use rne_slam::{Slam2d, SlamConfig};
use rne_world::Transform3;
use serde_json::{json, Value};

/// Control steps per Mid-360 frame (500 Hz control, 10 Hz frames).
const STEPS_PER_FRAME: u64 = 50;
const FRAME_S: f64 = 0.1;
/// People farther than this are not reported by the oracle.
const PERCEPTION_RANGE_M: f64 = 12.0;
/// A person's centre this close to the base's centre is a contact: the
/// person's radius plus about half the Go2's body length.
const CONTACT_M: f64 = 0.55;
/// Closer than this counts as a near miss.
const NEAR_MISS_M: f64 = 0.8;
/// The trot's validated walking speed and turn rate (examples 126, 130).
const MAX_SPEED_M_S: f64 = 0.25;
const MAX_YAW_RATE_RAD_S: f64 = 0.5;
/// Command changes are rate limited so stopping and starting do not jolt the trot.
const MAX_ACCEL_M_S2: f64 = 0.3;
const MAX_YAW_ACCEL_RAD_S2: f64 = 1.0;
const HEADING_GAIN_PER_S: f64 = 1.5;
/// Prediction horizon for conflicts with people.
const HORIZON_S: f64 = 2.5;
/// Predicted passing distances below this are conflicts.
const SAFE_PASS_M: f64 = 0.9;
/// People coming head-on closer than this ahead make the robot change lanes.
const LANE_LOOKAHEAD_M: f64 = 8.0;
/// Lateral clearance the robot passes people with.
const LANE_CLEARANCE_M: f64 = 1.1;
/// The new lane is aimed at this far ahead.
const LANE_AIM_M: f64 = 1.2;
/// A new track's velocity, and so the line it walks along, is unreliable for
/// this long; the side to pass it on is not kept before then.
const PASS_SIDE_SETTLE_S: f64 = 1.0;
/// A head-on person drifting sideways faster than this is passed behind.
const PASS_BEHIND_DRIFT_M_S: f64 = 0.15;
/// The follower keeps this far behind its target.
const FOLLOW_DISTANCE_M: f64 = 1.2;
/// A lost target is re-acquired within this distance of its prediction.
const REACQUIRE_M: f64 = 1.2;
/// Missions end within this distance of the goal.
const GOAL_TOLERANCE_M: f64 = 0.4;
/// Odometry errors: leg odometry's 4 % scale error (as in example 131), and a
/// gyro-grade yaw-rate bias. Example 131's 0.02 rad/s leg-odometry bias is
/// corrected there by SLAM; nothing corrects it here, and the robot would
/// curve 0.8 rad off course in 40 s.
const ODOM_SCALE_ERROR: f64 = 0.04;
const ODOM_YAW_RATE_BIAS_RAD_S: f64 = 0.002;
const PERSON_HALF_HEIGHT_M: f64 = 0.6;
const PERSON_RADIUS_M: f64 = 0.22;

fn repo_path(relative: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join(relative)
}

fn wrap(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

/// A piecewise-linear path in the task's plane; the person exists from the
/// first key time to the last.
#[derive(Clone, Debug)]
struct Trajectory {
    keys: Vec<(f64, [f64; 2])>,
}

impl Trajectory {
    fn path(points: &[[f64; 2]], speed_m_s: f64, start_s: f64) -> Self {
        let mut keys = vec![(start_s, points[0])];
        for pair in points.windows(2) {
            let length = (pair[1][0] - pair[0][0]).hypot(pair[1][1] - pair[0][1]);
            let time = keys.last().map_or(start_s, |k| k.0) + length / speed_m_s;
            keys.push((time, pair[1]));
        }
        Self { keys }
    }

    fn wait(mut self, duration_s: f64) -> Self {
        let (time, point) = *self.keys.last().expect("key");
        self.keys.push((time + duration_s, point));
        self
    }

    fn then(mut self, points: &[[f64; 2]], speed_m_s: f64) -> Self {
        let (time, point) = *self.keys.last().expect("key");
        let mut all = vec![point];
        all.extend_from_slice(points);
        let tail = Trajectory::path(&all, speed_m_s, time);
        self.keys.extend_from_slice(&tail.keys[1..]);
        self
    }

    /// The same path, `dt_s` later.
    fn shifted(&self, dt_s: f64) -> Self {
        Self {
            keys: self.keys.iter().map(|(t, p)| (t + dt_s, *p)).collect(),
        }
    }

    /// Position and velocity at `t`, or `None` outside the path's time span.
    fn state(&self, t: f64) -> Option<([f64; 2], [f64; 2])> {
        let (first, last) = (self.keys.first()?, self.keys.last()?);
        if t < first.0 || t > last.0 {
            return None;
        }
        for pair in self.keys.windows(2) {
            let ((t0, p0), (t1, p1)) = (pair[0], pair[1]);
            if t <= t1 && t1 > t0 {
                let s = (t - t0) / (t1 - t0);
                let v = [(p1[0] - p0[0]) / (t1 - t0), (p1[1] - p0[1]) / (t1 - t0)];
                return Some((
                    [p0[0] + s * (p1[0] - p0[0]), p0[1] + s * (p1[1] - p0[1])],
                    v,
                ));
            }
        }
        Some((last.1, [0.0, 0.0]))
    }
}

/// What the robot is asked to do.
#[derive(Clone, Copy, Debug)]
enum Mission {
    /// Walk to a point in the task plane.
    Goal([f64; 2]),
    /// Follow person `target` (index into the task's people).
    Follow { target: usize },
}

/// A wall standing on the floor: centre and half extents in the task plane.
#[derive(Clone, Copy, Debug)]
struct Wall {
    center: [f64; 2],
    half: [f64; 2],
}

#[derive(Clone, Debug)]
struct Task {
    name: &'static str,
    description: &'static str,
    duration_s: f64,
    mission: Mission,
    people: Vec<Trajectory>,
    walls: Vec<Wall>,
}

/// The task catalogue, in a plane where the robot starts at the origin facing +x.
fn tasks() -> Vec<Task> {
    vec![
        Task {
            name: "follow",
            description: "Follow a person 1.2 m behind through turns and a stop; one person cuts between, one walks past",
            duration_s: 70.0,
            mission: Mission::Follow { target: 0 },
            people: vec![
                Trajectory::path(&[[1.8, 0.0], [1.8, 0.0]], 1.0, 0.0)
                    .wait(3.0)
                    .then(&[[5.0, 0.0], [6.5, 2.0], [5.0, 3.5]], 0.22)
                    .wait(4.0)
                    .then(&[[2.0, 3.5]], 0.22),
                // Cuts between the robot and its target.
                Trajectory::path(&[[3.9, -3.5], [3.9, 3.5]], 0.6, 9.7),
                // Walks past the target, toward the robot's side.
                Trajectory::path(&[[2.0, 6.0], [7.0, 5.0], [7.0, -1.0]], 0.5, 25.0),
            ],
            walls: Vec::new(),
        },
        Task {
            name: "crossing",
            description: "Walk 12 m to a goal while four people cross the path",
            duration_s: 90.0,
            mission: Mission::Goal([12.0, 0.0]),
            people: vec![
                Trajectory::path(&[[3.5, -4.0], [3.5, 4.0]], 0.9, 10.5),
                Trajectory::path(&[[6.5, 4.0], [6.5, -4.0]], 1.1, 23.4),
                Trajectory::path(&[[9.5, -4.0], [9.5, 4.0]], 0.7, 33.3),
                Trajectory::path(&[[13.0, 3.0], [7.0, -3.0]], 0.8, 30.7),
            ],
            walls: Vec::new(),
        },
        Task {
            name: "corridor",
            description: "Walk down a 3.2 m wide corridor while people come the other way",
            duration_s: 90.0,
            mission: Mission::Goal([15.0, 0.0]),
            people: vec![
                Trajectory::path(&[[16.5, 0.3], [-1.5, 0.3]], 0.8, 2.0),
                Trajectory::path(&[[16.5, -0.5], [-1.5, -0.5]], 0.9, 20.0),
                Trajectory::path(&[[16.5, 0.6], [-1.5, 0.0]], 0.7, 35.0),
            ],
            walls: vec![
                Wall { center: [8.5, 1.7], half: [8.0, 0.1] },
                Wall { center: [8.5, -1.7], half: [8.0, 0.1] },
            ],
        },
        Task {
            name: "room",
            description: "Cross an 11 m room with furniture to a goal while three people walk through it",
            duration_s: 90.0,
            mission: Mission::Goal([11.0, -1.0]),
            people: vec![
                Trajectory::path(&[[9.0, 3.5], [3.0, -3.5]], 0.8, 6.0),
                Trajectory::path(&[[2.0, 3.8], [10.0, 3.0], [10.0, -3.5]], 0.6, 12.0),
                Trajectory::path(&[[11.5, -3.8], [6.0, 1.0], [1.0, 3.5]], 0.7, 25.0),
            ],
            walls: vec![
                // The room: 14 m x 9 m.
                Wall { center: [5.5, 4.5], half: [7.1, 0.1] },
                Wall { center: [5.5, -4.5], half: [7.1, 0.1] },
                Wall { center: [-1.5, 0.0], half: [0.1, 4.5] },
                Wall { center: [12.5, 0.0], half: [0.1, 4.5] },
                // Two tables, a shelf and two pillars, clear of the straight path.
                Wall { center: [3.0, 2.6], half: [0.8, 0.45] },
                Wall { center: [7.5, -2.9], half: [0.6, 0.6] },
                Wall { center: [9.0, 3.6], half: [1.2, 0.25] },
                Wall { center: [5.0, -1.9], half: [0.2, 0.2] },
                Wall { center: [8.5, 1.5], half: [0.2, 0.2] },
            ],
        },
        Task {
            name: "crowd",
            description: "Cross a 20 m x 9.6 m hall to a goal 14 m away among nine people: head-on, crossing, diagonal, overtaking, and a pair standing in the way",
            duration_s: 120.0,
            mission: Mission::Goal([14.0, 0.0]),
            people: vec![
                // Head-on, in three lanes.
                Trajectory::path(&[[17.0, 0.2], [-2.0, 0.2]], 0.9, 8.0),
                Trajectory::path(&[[17.0, 1.6], [-2.0, 1.6]], 0.8, 22.0),
                Trajectory::path(&[[17.0, 0.8], [-2.0, 0.8]], 0.7, 45.0),
                // Crossing, both ways.
                Trajectory::path(&[[6.0, 4.6], [6.0, -4.6]], 1.0, 18.0),
                Trajectory::path(&[[9.0, -4.6], [9.0, 4.6]], 0.8, 30.0),
                // Diagonal, toward the robot.
                Trajectory::path(&[[15.0, 4.0], [5.0, -4.0]], 0.7, 5.0),
                // Overtakes the robot from behind, 1 m to its left.
                Trajectory::path(&[[-2.2, 1.0], [17.0, 1.0]], 1.2, 15.0),
                // A pair standing in the way, 0.6 m apart, long enough for
                // the static map to take them in; then they walk off.
                Trajectory::path(&[[11.0, 0.4], [11.0, 0.4]], 1.0, 0.0)
                    .wait(55.0)
                    .then(&[[11.5, 4.5]], 0.6),
                Trajectory::path(&[[11.0, -0.2], [11.0, -0.2]], 1.0, 0.0)
                    .wait(55.0)
                    .then(&[[11.5, 4.5]], 0.6),
            ],
            walls: vec![
                Wall { center: [7.5, 4.9], half: [10.1, 0.1] },
                Wall { center: [7.5, -4.9], half: [10.1, 0.1] },
                Wall { center: [-2.6, 0.0], half: [0.1, 4.9] },
                Wall { center: [17.6, 0.0], half: [0.1, 4.9] },
            ],
        },
    ]
}

/// Trials shift every person's start by up to this much (seeded).
const TRIAL_SHIFT_S: f64 = 2.0;

/// The task for trial `trial`: trial 0 is the task as written; later trials
/// start each person (but a follow target) up to TRIAL_SHIFT_S earlier or
/// later, from a seed made of the task name, the trial and the person.
fn trial_task(task: &Task, trial: u32) -> Task {
    let mut task = task.clone();
    if trial == 0 {
        return task;
    }
    let target = match task.mission {
        Mission::Follow { target } => Some(target),
        Mission::Goal(_) => None,
    };
    let name_hash = task.name.bytes().fold(0xcbf2_9ce4_8422_2325_u64, |h, b| {
        (h ^ u64::from(b)).wrapping_mul(0x100_0000_01b3)
    });
    for (index, person) in task.people.iter_mut().enumerate() {
        if Some(index) == target {
            continue;
        }
        let mut z = name_hash ^ (u64::from(trial) << 32) ^ index as u64;
        z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        let unit = (z >> 11) as f64 / (1_u64 << 53) as f64;
        *person = person.shifted((2.0 * unit - 1.0) * TRIAL_SHIFT_S);
    }
    task
}

/// Where the robot's perception comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Source {
    Oracle,
    Nearest,
    Tracker,
    /// The nearest-neighbour tracker on detections filtered by the static map.
    NearestMap,
    /// The external tracker on detections filtered by the static map.
    TrackerMap,
    /// Perfect perception, localized by 2D SLAM: separates where SLAM makes
    /// the robot walk from what it does to perception.
    OracleSlam,
    /// `NearestMap` localized by 2D SLAM instead of odometry alone.
    NearestSlam,
    /// `TrackerMap` localized by 2D SLAM instead of odometry alone.
    TrackerSlam,
}

impl Source {
    fn name(self) -> &'static str {
        match self {
            Source::Oracle => "oracle",
            Source::Nearest => "nearest",
            Source::Tracker => "tracker",
            Source::NearestMap => "nearest+map",
            Source::TrackerMap => "tracker+map",
            Source::OracleSlam => "oracle+slam",
            Source::NearestSlam => "nearest+slam",
            Source::TrackerSlam => "tracker+slam",
        }
    }

    fn uses_map(self) -> bool {
        matches!(
            self,
            Source::NearestMap | Source::TrackerMap | Source::NearestSlam | Source::TrackerSlam
        )
    }

    fn uses_slam(self) -> bool {
        matches!(
            self,
            Source::OracleSlam | Source::NearestSlam | Source::TrackerSlam
        )
    }
}

/// Planar pose `[x, y, yaw]`.
type Pose = [f64; 3];

/// The robot's base pose in the task plane (world `[x, -z]`).
fn true_pose(sim: &UrdfSceneSim) -> Pose {
    let base = sim.named_transform("base").expect("base pose");
    let forward = base.rotation * Vec3::X;
    [
        base.translation.x,
        -base.translation.z,
        -forward.z.atan2(forward.x),
    ]
}

/// Task-plane point to the engine's y-up world, at `height`.
fn engine(p: [f64; 2], height: f64) -> Vec3 {
    Vec3::new(p[0], height, -p[1])
}

fn spawn_person(sim: &mut UrdfSceneSim, index: usize) -> Entity {
    let world = sim.world_mut();
    let entity = spawn_named(world, format!("person_{index}"));
    world.entity_mut(entity).insert((
        RigidBody {
            body_type: RigidBodyType::Kinematic,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Capsule {
                half_height_m: PERSON_HALF_HEIGHT_M,
                radius_m: PERSON_RADIUS_M,
            },
            // Seen by the LiDAR, never pushing the robot.
            sensor: true,
            ..Collider::default()
        },
        Transform3::from_translation_rotation(Vec3::new(0.0, -20.0, 0.0), Quat::IDENTITY),
    ));
    entity
}

fn spawn_wall(sim: &mut UrdfSceneSim, index: usize, wall: Wall) {
    let world = sim.world_mut();
    let entity = spawn_named(world, format!("wall_{index}"));
    world.entity_mut(entity).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: Vec3::new(wall.half[0], 0.6, wall.half[1]),
            },
            ..Collider::default()
        },
        Transform3::from_translation_rotation(engine(wall.center, 0.6), Quat::IDENTITY),
    ));
}

/// The robot's mission logic on its tracks (odometry frame).
/// How the controller chooses its command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ControllerKind {
    /// Rules: wait for crossing people, change lanes for oncoming ones.
    Rule,
    /// Dynamic window: roll out speed and turn-rate pairs and score them.
    Dwa,
}

/// Dynamic-window rollout: horizon, step, and the candidate grid.
const DWA_HORIZON_S: f64 = 3.0;
const DWA_STEP_S: f64 = 0.25;
const DWA_SPEEDS: usize = 6;
const DWA_TURNS: usize = 11;
/// Predicted passes closer than this are (softly) forbidden.
const DWA_SAFE_M: f64 = 0.75;
/// Below this predicted clearance a pass is penalised in proportion.
const DWA_COMFORT_M: f64 = 1.5;

struct Controller {
    kind: ControllerKind,
    /// Last chosen turn rate (DWA smoothness).
    last_yaw_rate: f64,
    /// The last lane decision, for the trace: track, side, offset, aim heading.
    last_lane: Option<(u64, f64, f64, f64)>,
    mission: Mission,
    target: Option<u64>,
    /// Last estimate of the target: position, velocity, time.
    target_memory: Option<([f64; 2], [f64; 2], f64)>,
    /// The goal or the target's start, in the odometry frame.
    goal: [f64; 2],
    /// The side chosen to pass each person on, kept until it is passed.
    pass_side: Vec<(u64, f64)>,
    /// Each track's history: a side is chosen from its mean velocity, and
    /// kept only once that has had PASS_SIDE_SETTLE_S to settle.
    history: Vec<TrackHistory>,
}

/// What the controller remembers of a track.
#[derive(Clone, Copy, Debug)]
struct TrackHistory {
    id: u64,
    first_seen_s: f64,
    /// Smoothed velocity: the plain mean over the first frames, then a
    /// moving average with VELOCITY_SMOOTHING_WEIGHT per frame (about 1 s).
    velocity: [f64; 2],
    frames: u32,
}

/// The weight of a new frame in a track's smoothed velocity, once it has
/// more than 1 / this many frames.
const VELOCITY_SMOOTHING_WEIGHT: f64 = 0.1;

impl TrackHistory {
    fn observe(&mut self, velocity: [f64; 2]) {
        self.frames += 1;
        let w = (1.0 / f64::from(self.frames)).max(VELOCITY_SMOOTHING_WEIGHT);
        self.velocity[0] += w * (velocity[0] - self.velocity[0]);
        self.velocity[1] += w * (velocity[1] - self.velocity[1]);
    }
}

impl Controller {
    /// Picks the target and returns the command and the chosen track.
    fn command(
        &mut self,
        tracks: &[Track],
        odom: Pose,
        time_s: f64,
        speed_now: f64,
    ) -> (UnitreeGo2TrotCommand, Option<Track>) {
        let (aim, desired_speed, target) = match self.mission {
            Mission::Goal(_) => {
                let d = (self.goal[0] - odom[0]).hypot(self.goal[1] - odom[1]);
                let speed = if d < GOAL_TOLERANCE_M {
                    0.0
                } else {
                    MAX_SPEED_M_S * (d / 1.0).clamp(0.3, 1.0)
                };
                (self.goal, speed, None)
            }
            Mission::Follow { .. } => {
                let target = self.track_target(tracks, odom, time_s);
                match (target, self.target_memory) {
                    (Some(t), _) => {
                        let d = (t.position[0] - odom[0]).hypot(t.position[1] - odom[1]);
                        (
                            t.position,
                            (0.8 * (d - FOLLOW_DISTANCE_M)).clamp(0.0, MAX_SPEED_M_S),
                            Some(t),
                        )
                    }
                    // Lost: turn toward where the target should be, slowly.
                    (None, Some((p, v, seen))) => {
                        let age = (time_s - seen).min(2.0);
                        ([p[0] + v[0] * age, p[1] + v[1] * age], 0.0, None)
                    }
                    (None, None) => (self.goal, 0.0, None),
                }
            }
        };
        if self.kind == ControllerKind::Dwa {
            let command = self.dwa(tracks, odom, aim, desired_speed, target);
            return (command, target);
        }
        let bearing = wrap((aim[1] - odom[1]).atan2(aim[0] - odom[0]) - odom[2]);
        let mut speed = desired_speed * bearing.cos().max(0.0).powi(2);
        let mut yaw_rate = HEADING_GAIN_PER_S * bearing;

        for track in tracks {
            match self.history.iter_mut().find(|h| h.id == track.id) {
                Some(h) => h.observe(track.velocity),
                None => self.history.push(TrackHistory {
                    id: track.id,
                    first_seen_s: time_s,
                    velocity: track.velocity,
                    frames: 1,
                }),
            }
        }
        self.last_lane = None;
        // Yield to crossing people; change lanes early for people coming
        // head-on or standing in the way.
        let heading = [odom[2].cos(), odom[2].sin()];
        let own_velocity = [
            heading[0] * speed_now.max(0.1),
            heading[1] * speed_now.max(0.1),
        ];
        // Nearest head-on person: (gap along its line, heading to steer).
        let mut lane: Option<(f64, f64)> = None;
        for track in tracks {
            if Some(track.id) == target.map(|t| t.id) {
                continue;
            }
            let d = [track.position[0] - odom[0], track.position[1] - odom[1]];
            let ahead = d[0] * heading[0] + d[1] * heading[1];
            if ahead < -0.3 {
                continue;
            }
            let across = -track.velocity[0] * heading[1] + track.velocity[1] * heading[0];
            let along = track.velocity[0] * heading[0] + track.velocity[1] * heading[1];
            // Someone coming toward the robot (even diagonally) does not clear
            // the path by the robot waiting: change lanes instead.
            if across.abs() > 0.3 && along > -0.3 {
                // Crossing: wait if its predicted path passes too close.
                let w = [
                    track.velocity[0] - own_velocity[0],
                    track.velocity[1] - own_velocity[1],
                ];
                let ww = w[0] * w[0] + w[1] * w[1];
                let t_close = if ww > 1e-6 {
                    (-(d[0] * w[0] + d[1] * w[1]) / ww).clamp(0.0, HORIZON_S)
                } else {
                    0.0
                };
                let pass = (d[0] + w[0] * t_close).hypot(d[1] + w[1] * t_close);
                if pass < SAFE_PASS_M || d[0].hypot(d[1]) < SAFE_PASS_M {
                    speed = 0.0;
                }
            } else {
                // Head-on or standing: measured from the line the person walks
                // along (the line to the goal for someone standing), not from
                // the robot's own heading, which may be skewed.
                // The line it walks along, from its smoothed velocity: one
                // frame's direction is off by up to 20 degrees, which moves
                // the line by 2 m at 6 m.
                let velocity = self
                    .history
                    .iter()
                    .find(|h| h.id == track.id)
                    .map_or(track.velocity, |h| h.velocity);
                let speed_p = velocity[0].hypot(velocity[1]);
                let dir = if speed_p > 0.2 {
                    [-velocity[0] / speed_p, -velocity[1] / speed_p]
                } else {
                    let g = [self.goal[0] - odom[0], self.goal[1] - odom[1]];
                    let n = g[0].hypot(g[1]).max(1e-6);
                    [g[0] / n, g[1] / n]
                };
                let normal = [-dir[1], dir[0]];
                let r = [-d[0], -d[1]];
                let offset = r[0] * normal[0] + r[1] * normal[1];
                let gap = -(r[0] * dir[0] + r[1] * dir[1]);
                if gap > 0.0
                    && gap < LANE_LOOKAHEAD_M
                    && offset.abs() < LANE_CLEARANCE_M
                    && lane.is_none_or(|(g, _)| gap < g)
                {
                    let history = self.history.iter().find(|h| h.id == track.id);
                    // Someone drifting across the robot's way is passed
                    // behind, judged by the track's smoothed velocity.
                    // Measured across the way to the goal, not the robot's
                    // heading: mid lane change, someone walking straight
                    // down a corridor would look like drifting.
                    let way = {
                        let g = [self.goal[0] - odom[0], self.goal[1] - odom[1]];
                        let n = g[0].hypot(g[1]).max(1e-6);
                        [g[0] / n, g[1] / n]
                    };
                    let left = [-way[1], way[0]];
                    let normal_left = normal[0] * left[0] + normal[1] * left[1];
                    let drift = history.map_or(0.0, |h| {
                        let v = h.velocity;
                        v[0] * left[0] + v[1] * left[1]
                    });
                    let side = if drift.abs() > PASS_BEHIND_DRIFT_M_S && normal_left.abs() > 0.3 {
                        -drift.signum() * normal_left.signum()
                    } else {
                        // Otherwise keep to the side the robot is on; when
                        // nearly on the line, take the side of the goal.
                        let remembered = self
                            .pass_side
                            .iter()
                            .find(|(id, _)| *id == track.id)
                            .map(|(_, s)| *s);
                        let side = remembered.unwrap_or_else(|| {
                            if offset.abs() > 0.2 {
                                offset.signum()
                            } else {
                                let g = [
                                    self.goal[0] - track.position[0],
                                    self.goal[1] - track.position[1],
                                ];
                                if g[0] * normal[0] + g[1] * normal[1] >= 0.0 {
                                    1.0
                                } else {
                                    -1.0
                                }
                            }
                        });
                        let settled =
                            history.is_some_and(|h| time_s - h.first_seen_s >= PASS_SIDE_SETTLE_S);
                        if remembered.is_none() && settled {
                            self.pass_side.retain(|(id, _)| *id != track.id);
                            self.pass_side.push((track.id, side));
                        }
                        side
                    };
                    let need = side * LANE_CLEARANCE_M - offset;
                    let aim = [
                        dir[0] * LANE_AIM_M + normal[0] * need,
                        dir[1] * LANE_AIM_M + normal[1] * need,
                    ];
                    lane = Some((gap, aim[1].atan2(aim[0])));
                    self.last_lane = Some((track.id, side, offset, aim[1].atan2(aim[0])));
                }
            }
        }
        if let Some((gap, aim_heading)) = lane {
            let error = wrap(aim_heading - odom[2]);
            yaw_rate = HEADING_GAIN_PER_S * error;
            // Walk the new lane at full speed (not slowed for facing away
            // from the goal), slowly only when the person is right there.
            if speed > 0.0 {
                speed = desired_speed.max(MAX_SPEED_M_S * 0.6) * error.cos().max(0.0);
            }
            if gap < 1.0 {
                speed = speed.min(0.1);
            }
        }
        (
            UnitreeGo2TrotCommand {
                forward_speed_m_s: speed,
                yaw_rate_rad_s: yaw_rate.clamp(-MAX_YAW_RATE_RAD_S, MAX_YAW_RATE_RAD_S),
            },
            target,
        )
    }

    /// Dynamic window: every pair of forward speed (0 to `max_speed`) and turn
    /// rate is rolled out for DWA_HORIZON_S as a unicycle, against every
    /// tracked person (but the target) moving at its tracked velocity. The
    /// cheapest wins: predicted clearance first, then the aim, the heading
    /// to it, progress and smoothness.
    fn dwa(
        &mut self,
        tracks: &[Track],
        odom: Pose,
        aim: [f64; 2],
        max_speed: f64,
        target: Option<Track>,
    ) -> UnitreeGo2TrotCommand {
        let others: Vec<&Track> = tracks
            .iter()
            .filter(|t| Some(t.id) != target.map(|t| t.id))
            .collect();
        let steps = (DWA_HORIZON_S / DWA_STEP_S).round() as usize;
        let follow = matches!(self.mission, Mission::Follow { .. });
        let mut best = (f64::MAX, UnitreeGo2TrotCommand::default());
        for i in 0..DWA_SPEEDS {
            let v = max_speed * i as f64 / (DWA_SPEEDS - 1) as f64;
            for j in 0..DWA_TURNS {
                let w = MAX_YAW_RATE_RAD_S * (2.0 * j as f64 / (DWA_TURNS - 1) as f64 - 1.0);
                let (mut x, mut y, mut th) = (odom[0], odom[1], odom[2]);
                let mut clearance = f64::MAX;
                for k in 1..=steps {
                    th += w * DWA_STEP_S;
                    x += v * th.cos() * DWA_STEP_S;
                    y += v * th.sin() * DWA_STEP_S;
                    let t = k as f64 * DWA_STEP_S;
                    for p in &others {
                        let d = (p.position[0] + p.velocity[0] * t - x)
                            .hypot(p.position[1] + p.velocity[1] * t - y);
                        clearance = clearance.min(d);
                    }
                }
                let to_aim = (aim[0] - x).hypot(aim[1] - y);
                let aim_cost = if follow {
                    (to_aim - FOLLOW_DISTANCE_M).abs()
                } else {
                    to_aim
                };
                let heading = wrap((aim[1] - y).atan2(aim[0] - x) - th).abs();
                let mut cost = aim_cost + 0.3 * heading + 0.2 * (w - self.last_yaw_rate).abs();
                if clearance < DWA_SAFE_M {
                    cost += 100.0 + 100.0 * (DWA_SAFE_M - clearance);
                } else if clearance < DWA_COMFORT_M {
                    cost += 1.5 * (DWA_COMFORT_M - clearance);
                }
                if cost < best.0 {
                    best = (
                        cost,
                        UnitreeGo2TrotCommand {
                            forward_speed_m_s: v,
                            yaw_rate_rad_s: w,
                        },
                    );
                }
            }
        }
        self.last_yaw_rate = best.1.yaw_rate_rad_s;
        best.1
    }

    /// The target track: the chosen id while it lasts; otherwise the track
    /// nearest to the target's predicted position (or, at the start, the
    /// nearest track ahead).
    fn track_target(&mut self, tracks: &[Track], odom: Pose, time_s: f64) -> Option<Track> {
        if let Some(track) = self
            .target
            .and_then(|id| tracks.iter().find(|t| t.id == id))
        {
            self.target_memory = Some((track.position, track.velocity, time_s));
            return Some(*track);
        }
        let predicted = match self.target_memory {
            Some((p, v, seen)) => {
                let age = (time_s - seen).min(2.0);
                [p[0] + v[0] * age, p[1] + v[1] * age]
            }
            None => self.goal,
        };
        let best = tracks
            .iter()
            .map(|t| {
                (
                    (t.position[0] - predicted[0]).hypot(t.position[1] - predicted[1]),
                    t,
                )
            })
            .filter(|(d, _)| *d <= REACQUIRE_M)
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, t)| *t)?;
        // Waiting for the robot to be facing it is not required: any track
        // near the prediction is taken.
        let _ = odom;
        self.target = Some(best.id);
        self.target_memory = Some((best.position, best.velocity, time_s));
        Some(best)
    }
}

/// Scores of one run, from the simulation's truth.
#[derive(Debug, Default)]
struct Score {
    contacts: u32,
    in_contact: bool,
    min_distance_m: f64,
    near_miss_s: f64,
    goal_time_s: Option<f64>,
    follow_frames: u32,
    follow_error_sum_m: f64,
    wrong_target_frames: u32,
    lost_frames: u32,
    tracks_per_frame_sum: u64,
    frames: u32,
    fell: bool,
    /// Time the safety layer held the robot.
    safety_stop_s: f64,
}

/// Task-plane point to the world plane: rotated by the start heading, then translated.
fn place(start: Pose, p: [f64; 2]) -> [f64; 2] {
    let (sin, cos) = start[2].sin_cos();
    [
        start[0] + cos * p[0] - sin * p[1],
        start[1] + sin * p[0] + cos * p[1],
    ]
}

/// Moves every person to its place at `t` (or parks it) and returns
/// `(id, position, velocity)` of those present, in the world plane.
fn place_people(
    sim: &mut UrdfSceneSim,
    task: &Task,
    entities: &[Entity],
    start: Pose,
    t: f64,
) -> Vec<(u64, [f64; 2], [f64; 2])> {
    let mut present = Vec::new();
    let (sin, cos) = start[2].sin_cos();
    for (i, (path, entity)) in task.people.iter().zip(entities).enumerate() {
        let transform = match path.state(t) {
            Some((p, v)) => {
                present.push((
                    i as u64,
                    place(start, p),
                    [cos * v[0] - sin * v[1], sin * v[0] + cos * v[1]],
                ));
                Transform3::from_translation_rotation(
                    engine(place(start, p), PERSON_HALF_HEIGHT_M + PERSON_RADIUS_M),
                    Quat::IDENTITY,
                )
            }
            None => {
                Transform3::from_translation_rotation(Vec3::new(0.0, -20.0, 0.0), Quat::IDENTITY)
            }
        };
        sim.world_mut().entity_mut(*entity).insert(transform);
    }
    present
}

/// One control step of leg odometry: the body's planar velocity and yaw rate
/// with the scale error and bias, integrated into `odom`.
fn integrate_odometry(sim: &UrdfSceneSim, odom: &mut Pose, dt: f64) {
    let observed = sim.observe();
    let truth = true_pose(sim);
    let (vx, vy) = (
        observed.base_linear_velocity_x_m_s,
        -observed.base_linear_velocity_z_m_s,
    );
    let (sin, cos) = truth[2].sin_cos();
    let (forward, left) = (cos * vx + sin * vy, -sin * vx + cos * vy);
    let scale = 1.0 + ODOM_SCALE_ERROR;
    let (s, c) = odom[2].sin_cos();
    odom[0] += (c * forward - s * left) * scale * dt;
    odom[1] += (s * forward + c * left) * scale * dt;
    odom[2] += (observed.base_angular_velocity_y_rad_s + ODOM_YAW_RATE_BIAS_RAD_S) * dt;
}

/// Distance from the base to the nearest person, or `f64::MAX`.
fn closest_person(people: &[(u64, [f64; 2], [f64; 2])], truth: Pose) -> f64 {
    people
        .iter()
        .map(|(_, p, _)| (p[0] - truth[0]).hypot(p[1] - truth[1]))
        .fold(f64::MAX, f64::min)
}

/// A frame's returns as `[forward, left, height above floor]` in the base
/// frame at the frame's end, beyond 0.4 m.
fn base_frame_points(cloud: &rne_data::PointCloud, truth: Pose) -> Vec<[f64; 3]> {
    let (sin, cos) = truth[2].sin_cos();
    cloud
        .points_m
        .iter()
        .filter_map(|p| {
            let (dx, dy) = (p.x - truth[0], -p.z - truth[1]);
            let local = [cos * dx + sin * dy, -sin * dx + cos * dy];
            (local[0].hypot(local[1]) > 0.4).then_some([local[0], local[1], p.y])
        })
        .collect()
}

/// Safety layer for the static map. A person standing still long enough
/// joins the static map and is no longer detected. The robot does not walk
/// forward while the centre of a person-shaped cluster of static returns is
/// in this region ahead of the base, nor for SAFETY_HOLD_S after. Walls
/// (which fail the person test) and returns not on the map are left to the
/// controller: stopping in front of someone walking toward the robot, or
/// for a wall at an angle mid lane change, makes things worse.
const SAFETY_AHEAD_M: (f64, f64) = (0.3, 1.25);
const SAFETY_HALF_WIDTH_M: f64 = 0.45;
const SAFETY_HOLD_S: f64 = 1.0;

/// Whether a person-shaped static cluster (centre in the base frame) is in
/// the way.
fn static_person_ahead(centres: &[[f64; 2]]) -> bool {
    centres.iter().any(|c| {
        c[0] > SAFETY_AHEAD_M.0 && c[0] < SAFETY_AHEAD_M.1 && c[1].abs() < SAFETY_HALF_WIDTH_M
    })
}

/// The new command, moved toward `next` at the allowed rates.
fn rate_limited(
    command: UnitreeGo2TrotCommand,
    next: UnitreeGo2TrotCommand,
) -> UnitreeGo2TrotCommand {
    let speed_step = MAX_ACCEL_M_S2 * FRAME_S;
    let yaw_step = MAX_YAW_ACCEL_RAD_S2 * FRAME_S;
    UnitreeGo2TrotCommand {
        forward_speed_m_s: command.forward_speed_m_s
            + (next.forward_speed_m_s - command.forward_speed_m_s).clamp(-speed_step, speed_step),
        yaw_rate_rad_s: command.yaw_rate_rad_s
            + (next.yaw_rate_rad_s - command.yaw_rate_rad_s).clamp(-yaw_step, yaw_step),
    }
}

impl Score {
    /// Contact, closest approach and near-miss time after one control step.
    fn observe_step(&mut self, people: &[(u64, [f64; 2], [f64; 2])], truth: Pose, dt: f64) {
        let closest = closest_person(people, truth);
        self.min_distance_m = self.min_distance_m.min(closest);
        if closest < NEAR_MISS_M {
            self.near_miss_s += dt;
        }
        let touching = closest < CONTACT_M;
        if touching && !self.in_contact {
            self.contacts += 1;
        }
        self.in_contact = touching;
    }

    /// Following scores after one frame: distance error, and whether the
    /// chosen track is the target person, someone else or nobody.
    fn observe_follow(
        &mut self,
        target: u64,
        people: &[(u64, [f64; 2], [f64; 2])],
        truth: Pose,
        odom: Pose,
        chosen: Option<Track>,
    ) {
        let Some((_, target_pos, _)) = people.iter().find(|(id, _, _)| *id == target) else {
            return;
        };
        self.follow_frames += 1;
        let d = (target_pos[0] - truth[0]).hypot(target_pos[1] - truth[1]);
        self.follow_error_sum_m += (d - FOLLOW_DISTANCE_M).abs();
        let Some(track) = chosen else {
            self.lost_frames += 1;
            return;
        };
        // The chosen track in the world: relative to the robot by odometry,
        // placed at the true pose.
        let rel = [track.position[0] - odom[0], track.position[1] - odom[1]];
        let (s, c) = odom[2].sin_cos();
        let world = perception::to_odom(truth, [c * rel[0] + s * rel[1], -s * rel[0] + c * rel[1]]);
        let nearest = people
            .iter()
            .min_by(|a, b| {
                let da = (a.1[0] - world[0]).hypot(a.1[1] - world[1]);
                let db = (b.1[0] - world[0]).hypot(b.1[1] - world[1]);
                da.total_cmp(&db)
            })
            .map(|(id, _, _)| *id);
        if nearest != Some(target) {
            self.wrong_target_frames += 1;
        }
    }

    fn json(
        &self,
        task: &Task,
        source: Source,
        duration_s: f64,
        goal_distance_m: Option<f64>,
    ) -> Value {
        let follow = matches!(task.mission, Mission::Follow { .. });
        let per = |n: u32| {
            if self.follow_frames == 0 {
                0.0
            } else {
                f64::from(n) / f64::from(self.follow_frames)
            }
        };
        json!({
            "task": task.name,
            "goal_distance_m": goal_distance_m,
            "source": source.name(),
            "duration_s": duration_s,
            "fell": self.fell,
            "contacts": self.contacts,
            "min_distance_m": self.min_distance_m,
            "near_miss_s": self.near_miss_s,
            "goal_time_s": self.goal_time_s,
            "follow_error_m": follow.then(|| self.follow_error_sum_m / f64::from(self.follow_frames.max(1))),
            "wrong_target_fraction": follow.then(|| per(self.wrong_target_frames)),
            "lost_fraction": follow.then(|| per(self.lost_frames)),
            "tracks_per_frame": self.tracks_per_frame_sum as f64 / f64::from(self.frames.max(1)),
            "safety_stop_s": self.safety_stop_s,
        })
    }
}

/// The robot's perception: one of the three sources.
struct Perceiver {
    source: Source,
    oracle: Oracle,
    nearest: NearestTracker,
    external: Option<ExternalTracker>,
    /// Static structure seen so far (map sources only).
    map: Option<perception::StaticMap>,
}

impl Perceiver {
    fn tracks(
        &mut self,
        detections: &[perception::Detection],
        people: &[(u64, [f64; 2], [f64; 2])],
        truth: Pose,
        odom: Pose,
        time_s: f64,
    ) -> Vec<Track> {
        match self.source {
            Source::Oracle | Source::OracleSlam => self.oracle.tracks(people, truth, odom),
            Source::Nearest | Source::NearestMap | Source::NearestSlam => {
                self.nearest.update(detections, time_s, FRAME_S)
            }
            Source::Tracker | Source::TrackerMap | Source::TrackerSlam => self
                .external
                .as_mut()
                .expect("--tracker is required for the tracker source")
                .update(detections, time_s)
                .expect("tracker update"),
        }
    }
}

/// 2D SLAM (`rne_slam`, as in example 131) on a scan of the returns 0.15–0.65 m
/// above the floor, without the returns of detected people, correcting the
/// odometry on keyframes by at most a few centimetres each.
struct Localizer {
    slam: Slam2d,
    slam_pose: Pose2d,
    odom_at_slam: Pose2d,
    /// Keyframes left out because their scan was degenerate.
    skipped_degenerate: u32,
    /// Keyframes processed.
    keyframes: u32,
    /// Normal spread of the last keyframe scan.
    last_spread: Option<f64>,
}

/// Below this ratio of the smaller to the larger eigenvalue of the scan's
/// normal information, the scan constrains only one direction. Measured in
/// these tasks: 0.11-0.21 in the corridor, 0.41-0.96 in the room.
const DEGENERACY_RATIO: f64 = 0.3;

/// Whether a scan's surfaces all face (nearly) one way, so that scan
/// matching cannot fix the motion along them. Normals come from neighbouring
/// beams; `sum(n n^T)` has one small eigenvalue for a corridor.
fn degenerate(ranges: &[f64], increment: f64) -> bool {
    normal_spread(ranges, increment).is_some_and(|ratio| ratio < DEGENERACY_RATIO)
}

/// Smaller over larger eigenvalue of the scan's normal information, or
/// `None` when the scan has too few surfaces to tell.
fn normal_spread(ranges: &[f64], increment: f64) -> Option<f64> {
    let point = |i: usize| {
        let a = -PI + (i as f64 + 0.5) * increment;
        [ranges[i] * a.cos(), ranges[i] * a.sin()]
    };
    let (mut xx, mut xy, mut yy) = (0.0, 0.0, 0.0);
    for i in 0..ranges.len() {
        let j = (i + 1) % ranges.len();
        if ranges[i].is_nan() || ranges[j].is_nan() {
            continue;
        }
        let (p, q) = (point(i), point(j));
        let t = [q[0] - p[0], q[1] - p[1]];
        let length = t[0].hypot(t[1]);
        // Skip jumps between surfaces.
        if !(1e-6..0.3).contains(&length) {
            continue;
        }
        let n = [-t[1] / length, t[0] / length];
        xx += n[0] * n[0];
        xy += n[0] * n[1];
        yy += n[1] * n[1];
    }
    let trace = xx + yy;
    if trace < 1.0 {
        return None;
    }
    let root = ((xx - yy).powi(2) / 4.0 + xy * xy).sqrt();
    Some((trace / 2.0 - root) / (trace / 2.0 + root))
}

/// Scan band for SLAM: low enough to see walls and furniture.
const SLAM_BAND_M: (f64, f64) = (0.15, 0.65);
const SLAM_BEAMS: usize = 720;
const SLAM_RANGE_M: (f64, f64) = (0.4, 20.0);
/// A keyframe every this much motion (as in example 131).
const KEYFRAME_TRANSLATION_M: f64 = 0.15;
const KEYFRAME_ROTATION_RAD: f64 = 0.15;
/// The SLAM grid covers the task area around the start.
const SLAM_GRID_ORIGIN_M: [f64; 2] = [-5.0, -8.0];
const SLAM_GRID_SIZE_M: [f64; 2] = [26.0, 18.0];
const SLAM_RESOLUTION_M: f64 = 0.05;
/// Scan-matching search half-windows per keyframe.
const SLAM_MAX_CORRECTION_M: f64 = 0.05;
const SLAM_MAX_CORRECTION_RAD: f64 = 0.02;
/// Beams on mapped structure a match needs.
const SLAM_MIN_BEAMS: usize = 80;
/// Returns this close to a detected person are left out of the SLAM scan.
const SLAM_PERSON_CLEARANCE_M: f64 = 0.5;

fn pose2d(p: Pose) -> Pose2d {
    Pose2d::new(p[0], p[1], p[2])
}

impl Localizer {
    fn new(start: Pose) -> Self {
        let grid = OccupancyGrid::new(
            (SLAM_GRID_SIZE_M[0] / SLAM_RESOLUTION_M) as usize,
            (SLAM_GRID_SIZE_M[1] / SLAM_RESOLUTION_M) as usize,
            SLAM_RESOLUTION_M,
            Pose2d::new(
                start[0] + SLAM_GRID_ORIGIN_M[0],
                start[1] + SLAM_GRID_ORIGIN_M[1],
                0.0,
            ),
        )
        .expect("SLAM grid");
        let mut config = SlamConfig {
            max_beams: 360,
            ..SlamConfig::default()
        };
        config.matcher.linear_samples = 7;
        config.matcher.angular_samples = 7;
        config.matcher.levels = 4;
        // Small corrections only: on open floor the scan holds little but
        // people, and in a corridor nothing pins the motion along it, so a
        // wide search slides or flips the pose. Odometry carries the rest.
        config.matcher.linear_window_m = SLAM_MAX_CORRECTION_M;
        config.matcher.angular_window_rad = SLAM_MAX_CORRECTION_RAD;
        // Scans with little static structure are not matched at all.
        config.matcher.min_valid_beams = SLAM_MIN_BEAMS;
        Self {
            slam: Slam2d::new(grid, config),
            slam_pose: pose2d(start),
            odom_at_slam: pose2d(start),
            skipped_degenerate: 0,
            keyframes: 0,
            last_spread: None,
        }
    }

    /// The best pose now: the last SLAM pose moved by the odometry since.
    fn estimate(&self, odom: Pose) -> Pose {
        let p = self
            .slam_pose
            .compose(self.odom_at_slam.inverse().compose(pose2d(odom)));
        [p.x_m, p.y_m, p.yaw_rad]
    }

    /// Processes a frame (base-frame points) as a keyframe when the robot
    /// has moved enough since the last one.
    fn update(&mut self, points: &[[f64; 3]], odom: Pose, time_s: f64, people: &[[f64; 2]]) {
        let since = self.odom_at_slam.inverse().compose(pose2d(odom));
        let keyframe = self.slam.scans_processed() == 0
            || since.x_m.hypot(since.y_m) >= KEYFRAME_TRANSLATION_M
            || since.yaw_rad.abs() >= KEYFRAME_ROTATION_RAD;
        if !keyframe {
            return;
        }
        let increment = 2.0 * PI / SLAM_BEAMS as f64;
        let mut ranges = vec![f64::NAN; SLAM_BEAMS];
        for p in points {
            if p[2] < SLAM_BAND_M.0 || p[2] > SLAM_BAND_M.1 {
                continue;
            }
            let range = p[0].hypot(p[1]);
            if range < SLAM_RANGE_M.0 || range > SLAM_RANGE_M.1 {
                continue;
            }
            // People move: their returns would drag the match.
            if people
                .iter()
                .any(|q| (q[0] - p[0]).hypot(q[1] - p[1]) < SLAM_PERSON_CLEARANCE_M)
            {
                continue;
            }
            let beam = ((p[1].atan2(p[0]) + PI) / increment).floor() as usize % SLAM_BEAMS;
            if ranges[beam].is_nan() || range < ranges[beam] {
                ranges[beam] = range;
            }
        }
        self.last_spread = normal_spread(&ranges, increment);
        if degenerate(&ranges, increment) {
            // Nothing in the scan pins the motion along one direction (two
            // parallel walls): matching would slide along it. Odometry only.
            self.skipped_degenerate += 1;
            return;
        }
        let scan = LaserScan2d {
            time_s,
            frame: FrameId::new("base_level"),
            angle_min_rad: -PI + 0.5 * increment,
            angle_increment_rad: increment,
            range_min_m: SLAM_RANGE_M.0,
            range_max_m: SLAM_RANGE_M.1,
            ranges_m: ranges,
        };
        // The points are already in the base frame.
        let update = self
            .slam
            .process(&scan, pose2d(odom), Pose2d::new(0.0, 0.0, 0.0))
            .expect("SLAM update");
        self.keyframes += 1;
        self.slam_pose = update.pose;
        self.odom_at_slam = pose2d(odom);
    }
}

/// One task under one perception source: the score and a per-frame trace.
fn run(
    task: &Task,
    source: Source,
    tracker_command: Option<&[String]>,
    kind: ControllerKind,
    safety: bool,
) -> (Value, Vec<Value>) {
    let mut sim =
        UrdfSceneSim::from_scene_path(&repo_path("assets/scenes/unitree_go2_jump.rne.scene.toml"))
            .expect("load Go2 scene");
    let mut trot = UnitreeGo2ModelTrot::stand_up(&mut sim).with_command_ease_in_s(1.0);
    let start = true_pose(&sim);
    let people: Vec<Entity> = (0..task.people.len())
        .map(|i| spawn_person(&mut sim, i))
        .collect();
    for (i, wall) in task.walls.iter().enumerate() {
        // Walls run along the task's x axis; the start heading is ~0 here.
        spawn_wall(
            &mut sim,
            i,
            Wall {
                center: place(start, wall.center),
                half: wall.half,
            },
        );
    }
    let rig: LidarRigOcclusion = serde_json::from_str(
        &fs::read_to_string(repo_path(
            "assets/sensors/livox_mid360/go2_rig_occlusion.json",
        ))
        .expect("rig table"),
    )
    .expect("parse rig table");
    let spec: LidarSpec = livox_mid360_spec();
    let pattern = LivoxMid360Pattern::new();
    let mut perceiver = Perceiver {
        source,
        oracle: Oracle {
            range_m: PERCEPTION_RANGE_M,
        },
        nearest: NearestTracker::default(),
        map: source.uses_map().then(perception::StaticMap::default),
        external: match (source, tracker_command) {
            (Source::Tracker | Source::TrackerMap | Source::TrackerSlam, Some(command)) => {
                Some(ExternalTracker::spawn(command).expect("start the tracker"))
            }
            _ => None,
        },
    };
    let mut controller = Controller {
        kind,
        last_yaw_rate: 0.0,
        last_lane: None,
        mission: task.mission,
        target: None,
        target_memory: None,
        pass_side: Vec::new(),
        history: Vec::new(),
        goal: match task.mission {
            Mission::Goal(g) => place(start, g),
            Mission::Follow { target } => place(start, task.people[target].keys[0].1),
        },
    };
    // Odometry starts at the true start pose, so its frame is the world plane.
    let mut odom = start;
    let mut localizer = source.uses_slam().then(|| Localizer::new(start));
    let mut command = UnitreeGo2TrotCommand::default();
    let mut frame_start = sim
        .named_mount_transform("base", &unitree_go2_mid360_mount())
        .expect("mount");
    let mut score = Score {
        min_distance_m: f64::MAX,
        ..Score::default()
    };
    let dt = 1.0 / UNITREE_GO2_MODEL_TROT_CONTROL_HZ;
    let mut frame = 0_u64;
    let mut people_now = Vec::new();
    let mut trace = Vec::new();
    let mut safety_hold_until_s = f64::NEG_INFINITY;

    while trot.time_s() < task.duration_s && score.goal_time_s.is_none() && !score.fell {
        for _ in 0..STEPS_PER_FRAME {
            people_now = place_people(&mut sim, task, &people, start, trot.time_s());
            trot.step(&mut sim, command);
            integrate_odometry(&sim, &mut odom, dt);
            // What the robot believes its pose is.
            let belief = localizer.as_ref().map_or(odom, |l| l.estimate(odom));
            score.fell |= sim.observe().base_y_m < 0.15;
            score.observe_step(&people_now, true_pose(&sim), dt);
            // Arrived when the robot believes it is at the goal (its
            // odometry); the true remaining distance is reported apart.
            if let Mission::Goal(goal) = task.mission {
                let g = place(start, goal);
                if (g[0] - belief[0]).hypot(g[1] - belief[1]) < GOAL_TOLERANCE_M {
                    score.goal_time_s = Some(trot.time_s());
                }
            }
        }

        // One Mid-360 frame, expressed in the base frame at the frame's end.
        let end = sim
            .named_mount_transform("base", &unitree_go2_mid360_mount())
            .expect("mount");
        let cloud = sim.sample_livox_mid360(
            &LidarSweep::new(frame_start, end),
            &spec,
            &pattern,
            frame,
            Some(&rig),
            SensorNoiseKey::new(sim.world_seed(), spec.seed, 1, frame),
        );
        frame_start = end;
        frame += 1;
        let truth = true_pose(&sim);
        let points = base_frame_points(&cloud, truth);
        if let Some(localizer) = localizer.as_mut() {
            // Person clusters in the base frame (identity pose).
            let people: Vec<[f64; 2]> = detect_people(&points, [0.0, 0.0, 0.0])
                .iter()
                .map(|d| d.position)
                .collect();
            localizer.update(&points, odom, trot.time_s(), &people);
        }
        // The pose the robot perceives and acts in: SLAM-corrected, or odometry.
        let odom_raw = odom;
        let odom = localizer
            .as_ref()
            .map_or(odom_raw, |l| l.estimate(odom_raw));
        let detections = match perceiver.map.as_mut() {
            Some(map) => {
                // Filter with what was static before this frame, then add it.
                let detections = perception::detect_moving_people(&points, odom, Some(map));
                map.update(&points, odom);
                detections
            }
            None => detect_people(&points, odom),
        };
        let time_s = trot.time_s();
        let tracks = perceiver.tracks(&detections, &people_now, truth, odom, time_s);
        let observed = sim.observe();
        let speed_now = observed
            .base_linear_velocity_x_m_s
            .hypot(observed.base_linear_velocity_z_m_s);
        let (mut next, chosen) = controller.command(&tracks, odom, time_s, speed_now);
        if safety
            && perceiver.map.as_ref().is_some_and(|map| {
                let on_static: Vec<[f64; 3]> = points
                    .iter()
                    .filter(|p| map.is_static(perception::to_odom(odom, [p[0], p[1]])))
                    .copied()
                    .collect();
                let centres: Vec<[f64; 2]> = detect_people(&on_static, [0.0, 0.0, 0.0])
                    .iter()
                    .map(|d| d.position)
                    .collect();
                static_person_ahead(&centres)
            })
        {
            safety_hold_until_s = time_s + SAFETY_HOLD_S;
        }
        let blocked = time_s < safety_hold_until_s;
        if blocked {
            next.forward_speed_m_s = 0.0;
            score.safety_stop_s += FRAME_S;
        }
        command = rate_limited(command, next);
        score.frames += 1;
        score.tracks_per_frame_sum += tracks.len() as u64;
        let closest = closest_person(&people_now, truth);
        trace.push(json!({
            "t": time_s, "pose": truth, "odom": odom,
            "command": [command.forward_speed_m_s, command.yaw_rate_rad_s],
            "detections": detections.len(), "tracks": tracks.len(),
            "target": chosen.map(|t| t.id), "closest_m": (closest < f64::MAX).then_some(closest),
            "spread": localizer.as_ref().and_then(|l| l.last_spread),
            "track_list": tracks.iter().map(|t| json!([t.id, t.position, t.velocity])).collect::<Vec<_>>(),
            "lane": controller.last_lane,
            "blocked": blocked,
            "people": people_now.iter().map(|(id, p, v)| json!([id, p, v])).collect::<Vec<_>>(),
        }));
        if let Mission::Follow { target } = task.mission {
            score.observe_follow(target as u64, &people_now, truth, odom, chosen);
        }
    }

    let end = true_pose(&sim);
    let goal_distance = match task.mission {
        Mission::Goal(g) => {
            let g = place(start, g);
            Some((g[0] - end[0]).hypot(g[1] - end[1]))
        }
        Mission::Follow { .. } => None,
    };
    (
        score.json(task, source, trot.time_s(), goal_distance),
        trace,
    )
}

/// One row per task and source, over all trials.
fn print_summary(results: &[Value], tasks: &[Task], sources: &[Source]) {
    let f = |v: Option<f64>| v.map_or("-".to_owned(), |x| format!("{x:.2}"));
    let mean = |values: Vec<f64>| {
        (!values.is_empty()).then(|| values.iter().sum::<f64>() / values.len() as f64)
    };
    println!("| task | perception | runs | fell | runs with contact | contacts | closest, mean (m) | closest, min (m) | < 0.8 m, mean (s) | left to goal, mean (m) | follow error (m) | wrong target | lost | safety stop, mean (s) |");
    println!(
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"
    );
    for task in tasks {
        for source in sources {
            let rows: Vec<&Value> = results
                .iter()
                .filter(|r| r["task"] == task.name && r["source"] == source.name())
                .collect();
            let field = |key: &str| {
                rows.iter()
                    .filter_map(|r| r[key].as_f64())
                    .collect::<Vec<f64>>()
            };
            let contacts: Vec<u64> = rows
                .iter()
                .map(|r| r["contacts"].as_u64().unwrap_or(0))
                .collect();
            println!(
                "| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |",
                task.name,
                source.name(),
                rows.len(),
                rows.iter().filter(|r| r["fell"] == true).count(),
                contacts.iter().filter(|c| **c > 0).count(),
                contacts.iter().sum::<u64>(),
                f(mean(field("min_distance_m"))),
                f(field("min_distance_m").into_iter().reduce(f64::min)),
                f(mean(field("near_miss_s"))),
                f(mean(field("goal_distance_m"))),
                f(mean(field("follow_error_m"))),
                f(mean(field("wrong_target_fraction"))),
                f(mean(field("lost_fraction"))),
                f(mean(field("safety_stop_s"))),
            );
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut out = PathBuf::from("target/go2_people");
    let mut only: Option<String> = None;
    let mut tracker: Option<Vec<String>> = None;
    let mut duration: Option<f64> = None;
    let mut trials: u32 = 1;
    let mut kind = ControllerKind::Rule;
    let mut safety = true;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--out" => out = PathBuf::from(args.next().ok_or("--out needs a directory")?),
            "--task" => only = Some(args.next().ok_or("--task needs a name")?),
            "--trials" => trials = args.next().ok_or("--trials needs a count")?.parse()?,
            "--no-safety" => safety = false,
            "--controller" => {
                kind = match args.next().as_deref() {
                    Some("rule") => ControllerKind::Rule,
                    Some("dwa") => ControllerKind::Dwa,
                    _ => return Err("--controller needs rule or dwa".into()),
                }
            }
            "--duration-s" => {
                duration = Some(args.next().ok_or("--duration-s needs seconds")?.parse()?)
            }
            // Everything after --tracker is the tracker's command line.
            "--tracker" => {
                tracker = Some(args.by_ref().collect());
            }
            other => return Err(format!("unknown argument {other}").into()),
        }
    }
    fs::create_dir_all(&out)?;
    // The stream tracker's setup: detections arrive already in the odometry
    // frame, so the LiDAR frame is the tracking frame and there are no poses.
    let identity = [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];
    fs::write(
        out.join("stream_setup.json"),
        serde_json::to_string_pretty(&json!({
            "tracking_frame": "odom", "lidar_frame": "odom", "camera_frame": "camera",
            "lidar_to_reference": identity, "camera_to_reference": identity,
            "intrinsics": {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0, "skew": 0.0,
                           "image_width": 640, "image_height": 480},
            "near_plane": 0.3,
        }))?,
    )?;
    let mut sources = vec![
        Source::Oracle,
        Source::OracleSlam,
        Source::Nearest,
        Source::NearestMap,
        Source::NearestSlam,
    ];
    if tracker.is_some() {
        sources.extend([Source::Tracker, Source::TrackerMap, Source::TrackerSlam]);
    }
    let selected: Vec<Task> = tasks()
        .into_iter()
        .filter(|t| only.as_deref().is_none_or(|n| n == t.name))
        .map(|mut t| {
            if let Some(d) = duration {
                t.duration_s = d;
            }
            t
        })
        .collect();
    if selected.is_empty() {
        return Err("no task matched --task".into());
    }
    let mut jobs: Vec<(Task, Source, u32)> = Vec::new();
    for task in &selected {
        for trial in 0..trials {
            let shifted = trial_task(task, trial);
            for source in &sources {
                jobs.push((shifted.clone(), *source, trial));
            }
        }
    }
    // A pool as large as the machine: each run holds a full simulation.
    let workers = std::thread::available_parallelism().map_or(1, std::num::NonZeroUsize::get);
    let next = std::sync::atomic::AtomicUsize::new(0);
    let mut runs: Vec<(usize, Value, Vec<Value>)> = std::thread::scope(|scope| {
        let handles: Vec<_> = (0..workers.min(jobs.len()))
            .map(|_| {
                scope.spawn(|| {
                    let mut mine = Vec::new();
                    loop {
                        let index = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                        let Some((task, source, trial)) = jobs.get(index) else {
                            break;
                        };
                        let (mut result, trace) =
                            run(task, *source, tracker.as_deref(), kind, safety);
                        result["trial"] = json!(trial);
                        mine.push((index, result, trace));
                    }
                    mine
                })
            })
            .collect();
        handles
            .into_iter()
            .flat_map(|h| h.join().unwrap_or_else(|p| std::panic::resume_unwind(p)))
            .collect()
    });
    runs.sort_by_key(|(index, _, _)| *index);
    let mut results = Vec::new();
    for (_, result, trace) in runs {
        {
            // Trial 0 keeps the plain name; later trials get a suffix.
            let trial = result["trial"].as_u64().unwrap_or(0);
            let suffix = if trial == 0 {
                String::new()
            } else {
                format!("_t{trial}")
            };
            let name = format!(
                "{}_{}{suffix}.jsonl",
                result["task"].as_str().unwrap_or(""),
                result["source"].as_str().unwrap_or("")
            );
            let lines: String = trace.iter().map(|r| format!("{r}\n")).collect();
            fs::write(out.join(name), lines)?;
        }
        results.push(result);
    }
    print_summary(&results, &selected, &sources);
    fs::write(
        out.join("results.json"),
        serde_json::to_string_pretty(&results)?,
    )?;
    let descriptions: Vec<Value> = selected
        .iter()
        .map(|t| json!({"task": t.name, "description": t.description}))
        .collect();
    fs::write(
        out.join("tasks.json"),
        serde_json::to_string_pretty(&descriptions)?,
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn controller(mission: Mission, goal: [f64; 2]) -> Controller {
        Controller {
            kind: ControllerKind::Rule,
            last_yaw_rate: 0.0,
            last_lane: None,
            mission,
            target: None,
            target_memory: None,
            goal,
            pass_side: Vec::new(),
            history: Vec::new(),
        }
    }

    fn track(id: u64, position: [f64; 2], velocity: [f64; 2]) -> Track {
        Track {
            id,
            position,
            velocity,
        }
    }

    #[test]
    fn yields_to_a_person_crossing_its_path() {
        let mut c = controller(Mission::Goal([10.0, 0.0]), [10.0, 0.0]);
        // 1 m ahead, 0.8 m to the right, crossing to the left at 1 m/s: it
        // would pass 0.8 m from the robot.
        let tracks = [track(1, [1.0, -0.8], [0.0, 1.0])];
        let (command, _) = c.command(&tracks, [0.0, 0.0, 0.0], 0.0, MAX_SPEED_M_S);
        assert_eq!(command.forward_speed_m_s, 0.0);
        // A person crossing far ahead does not stop it.
        let tracks = [track(1, [7.0, -3.0], [0.0, 1.0])];
        let (command, _) = c.command(&tracks, [0.0, 0.0, 0.0], 0.0, MAX_SPEED_M_S);
        assert!(command.forward_speed_m_s > 0.1);
    }

    #[test]
    fn changes_lanes_for_a_person_coming_head_on_and_keeps_the_side() {
        let mut c = controller(Mission::Goal([10.0, 0.0]), [10.0, 0.0]);
        // Coming straight at the robot, slightly to its left: pass on the right.
        let tracks = [track(7, [5.0, 0.3], [-0.9, 0.0])];
        let (command, _) = c.command(&tracks, [0.0, 0.0, 0.0], 0.0, MAX_SPEED_M_S);
        assert!(command.yaw_rate_rad_s < -0.1, "{command:?}");
        assert!(command.forward_speed_m_s > 0.1, "{command:?}");
        // After a second the side is kept...
        let tracks = [track(7, [4.1, 0.3], [-0.9, 0.0])];
        c.command(&tracks, [0.0, 0.0, 0.0], 1.0, MAX_SPEED_M_S);
        // ...even when its estimate drifts to the right of the robot.
        let tracks = [track(7, [3.6, -0.1], [-0.9, 0.05])];
        let (command, _) = c.command(&tracks, [0.0, 0.0, 0.0], 1.5, MAX_SPEED_M_S);
        assert!(command.yaw_rate_rad_s < 0.0, "{command:?}");
    }

    #[test]
    fn safety_layer_stops_for_a_static_person_right_ahead_only() {
        assert!(static_person_ahead(&[[0.7, 0.1]]));
        assert!(static_person_ahead(&[[0.35, -0.3]]));
        // Beside the path or farther ahead: not in the way.
        assert!(!static_person_ahead(&[[0.7, 0.8]]));
        assert!(!static_person_ahead(&[[1.6, 0.0]]));
        assert!(!static_person_ahead(&[]));
    }

    #[test]
    fn passes_behind_someone_drifting_across_whatever_one_frame_says() {
        let mut c = controller(Mission::Goal([10.0, 0.0]), [10.0, 0.0]);
        // Ahead, a little to the right, walking toward the robot and
        // drifting to its left.
        for (i, t) in [0.0, 0.1, 0.2, 0.3].iter().enumerate() {
            let x = 5.0 - 0.1 * i as f64;
            let tracks = [track(7, [x, -0.3 + 0.025 * i as f64], [-0.8, 0.25])];
            c.command(&tracks, [0.0, 0.0, 0.0], *t, MAX_SPEED_M_S);
        }
        // One frame's velocity says it walks straight at the robot, along a
        // line nearly through it; the smoothed velocity still says it
        // drifts left.
        let tracks = [track(7, [4.6, -0.15], [-0.85, -0.05])];
        let (command, _) = c.command(&tracks, [0.0, 0.0, 0.0], 0.4, MAX_SPEED_M_S);
        assert!(
            command.yaw_rate_rad_s < 0.0,
            "pass behind, on the right: {command:?}"
        );
    }

    #[test]
    fn follower_reacquires_its_target_near_the_prediction() {
        let mut c = controller(Mission::Follow { target: 0 }, [2.0, 0.0]);
        let (_, chosen) = c.command(
            &[track(3, [2.0, 0.0], [0.3, 0.0])],
            [0.0, 0.0, 0.0],
            0.0,
            0.0,
        );
        assert_eq!(chosen.map(|t| t.id), Some(3));
        // The track is lost for 1 s; a new id appears where the target should
        // be, another one elsewhere.
        let tracks = [
            track(9, [2.35, 0.05], [0.3, 0.0]),
            track(4, [2.0, 2.0], [0.0, 0.0]),
        ];
        let (_, chosen) = c.command(&tracks, [0.0, 0.0, 0.0], 1.0, 0.0);
        assert_eq!(chosen.map(|t| t.id), Some(9));
    }

    #[test]
    fn dwa_turns_away_from_a_person_coming_head_on_and_keeps_going() {
        let mut c = controller(Mission::Goal([10.0, 0.0]), [10.0, 0.0]);
        c.kind = ControllerKind::Dwa;
        let tracks = [track(7, [3.0, 0.2], [-0.9, 0.0])];
        let (command, _) = c.command(&tracks, [0.0, 0.0, 0.0], 0.0, MAX_SPEED_M_S);
        // Slightly to the left: pass it on the right, still walking.
        assert!(command.yaw_rate_rad_s < 0.0, "{command:?}");
        // With nobody around it heads straight for the goal at full speed.
        let mut c = controller(Mission::Goal([10.0, 0.0]), [10.0, 0.0]);
        c.kind = ControllerKind::Dwa;
        let (command, _) = c.command(&[], [0.0, 0.0, 0.0], 0.0, MAX_SPEED_M_S);
        assert_eq!(command.yaw_rate_rad_s, 0.0);
        assert!((command.forward_speed_m_s - MAX_SPEED_M_S).abs() < 1e-12);
    }

    #[test]
    fn dwa_does_not_walk_into_a_crossing_person() {
        let mut c = controller(Mission::Goal([10.0, 0.0]), [10.0, 0.0]);
        c.kind = ControllerKind::Dwa;
        // Will be 1 m ahead on the robot's line in about 1.2 s.
        let tracks = [track(1, [1.0, -1.2], [0.0, 1.0])];
        let (command, _) = c.command(&tracks, [0.0, 0.0, 0.0], 0.0, MAX_SPEED_M_S);
        let (mut x, mut y, mut th) = (0.0, 0.0, 0.0);
        for k in 1..=12 {
            th += command.yaw_rate_rad_s * 0.25;
            x += command.forward_speed_m_s * f64::cos(th) * 0.25;
            y += command.forward_speed_m_s * f64::sin(th) * 0.25;
            let t = f64::from(k) * 0.25;
            assert!(
                (1.0 - x).hypot(-1.2 + t - y) >= DWA_SAFE_M,
                "{command:?} at {t} s"
            );
        }
    }

    #[test]
    fn trajectory_waits_and_ends() {
        let path = Trajectory::path(&[[0.0, 0.0], [1.0, 0.0]], 0.5, 1.0).wait(1.0);
        assert!(path.state(0.5).is_none());
        let (p, v) = path.state(2.0).unwrap();
        assert!((p[0] - 0.5).abs() < 1e-12 && (v[0] - 0.5).abs() < 1e-12);
        assert_eq!(path.state(3.5).unwrap().1, [0.0, 0.0]);
        assert!(path.state(4.5).is_none());
    }
}
