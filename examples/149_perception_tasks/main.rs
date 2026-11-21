//! Perception tasks simulated end to end: scenes with people, cyclists and cars
//! moving around a robot, scanned by a simulated 3D `LiDAR`, segmented by a
//! simple cluster detector, and written as the replay input of an external
//! tracker together with ground truth.
//!
//! Each task isolates one difficulty of robot perception (crowds, occlusion,
//! manoeuvres, ego motion, ego-pose noise, mixed traffic, range, people walking
//! side by side). The output of every task is a directory with
//!
//! - `setup.json`: frames and the ego pose of every scan (`map` <- `lidar`),
//! - `inputs.jsonl`: one `lidar_batch` per scan (detections and static
//!   landmarks in the `LiDAR` frame),
//! - `truth.jsonl`: one `truth3d` record per scan (objects in the `map` frame),
//! - `summary.json`: detector recall and precision against the truth.
//!
//! The format is the replay format of robot-perception
//! (<https://github.com/rsasaki0109/robot-perception>); nothing here depends on it.
//!
//! ```bash
//! cargo run --release -p perception_tasks --example 149_perception_tasks -- \
//!     --out target/perception_tasks
//! ```
//!
//! The scene is described in a `map` frame with x forward, y left and z up and
//! mapped to the engine's y-up world for raycasting.

mod camera;

use std::collections::{BTreeMap, VecDeque};
use std::fs;
use std::path::{Path, PathBuf};

use rne_core::SimDuration;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, PhysicsBackend, PhysicsWorldDesc, PhysicsWorldId, RigidBody,
    RigidBodyType,
};
use rne_physics_rapier::RapierBackend;
use rne_sensor::{sample_lidar_keyed, LidarSpec, SensorNoiseKey};
use rne_world::Transform3;
use serde_json::{json, Value};

/// Scan period of the simulated `LiDAR` (10 Hz).
const SCAN_PERIOD_S: f64 = 0.1;
/// Height of the `LiDAR` above the ground.
const SENSOR_HEIGHT_M: f64 = 0.9;
/// Objects farther than this from the robot are left out of the truth.
const TRUTH_RANGE_M: f64 = 35.0;
/// Cell size of the clustering grid; cells that touch (8-neighbourhood) join.
const CLUSTER_CELL_M: f64 = 0.15;
/// Points at or below this height are treated as ground.
const GROUND_CLEARANCE_M: f64 = 0.15;
/// Clusters with fewer points are dropped.
const MIN_CLUSTER_POINTS: usize = 3;
/// Clusters taller than this are structure (poles, pillars, walls), not objects.
const STRUCTURE_HEIGHT_M: f64 = 2.4;
/// Elevation of the `LiDAR`'s top beam.
const TOP_BEAM_ELEVATION_RAD: f64 = 15.0 * std::f64::consts::PI / 180.0;
/// Clusters narrower than this and taller than [`POLE_MIN_HEIGHT_M`] are poles.
const POLE_MAX_WIDTH_M: f64 = 0.25;
/// Poles reach above this height; people do not.
const POLE_MIN_HEIGHT_M: f64 = 1.9;
/// Landmarks (poles) are reported only within this range.
const LANDMARK_RANGE_M: f64 = 20.0;
/// A detection counts as a hit of a truth object of its class within this
/// distance: larger objects are seen from one side, so their centre is off more.
fn detection_match_m(kind: Kind) -> f64 {
    match kind {
        Kind::Person => 0.6,
        Kind::Cyclist => 1.0,
        Kind::Car => 2.0,
    }
}
/// Smallest box side written for a detection (box sizes must be positive).
const MIN_BOX_SIZE_M: f64 = 0.05;
/// Where hidden movers are parked: under the ground slab, out of every ray's reach.
const PARKED_DEPTH_M: f64 = -50.0;

/// Object categories, with the class ids written to the outputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Person,
    Cyclist,
    Car,
}

impl Kind {
    fn class(self) -> u32 {
        match self {
            Kind::Person => 1,
            Kind::Cyclist => 2,
            Kind::Car => 3,
        }
    }

    fn center_height_m(self) -> f64 {
        match self {
            Kind::Person => 0.82,
            Kind::Cyclist => 0.85,
            Kind::Car => 0.75,
        }
    }

    /// Collider in the engine's y-up body frame; the body's local +x is its heading.
    fn shape(self) -> ColliderShape {
        match self {
            // 1.64 m tall, 0.44 m across.
            Kind::Person => ColliderShape::Capsule {
                half_height_m: 0.6,
                radius_m: 0.22,
            },
            Kind::Cyclist => ColliderShape::Cuboid {
                half_extents_m: Vec3::new(0.85, 0.85, 0.25),
            },
            Kind::Car => ColliderShape::Cuboid {
                half_extents_m: Vec3::new(2.2, 0.75, 0.9),
            },
        }
    }
}

/// A piecewise-linear trajectory in the `map` plane. The object exists from
/// the first key time to the last.
#[derive(Clone, Debug)]
struct Trajectory {
    keys: Vec<(f64, [f64; 2])>,
}

impl Trajectory {
    /// Walks through `points` at `speed_m_s`, starting at `start_s`.
    fn path(points: &[[f64; 2]], speed_m_s: f64, start_s: f64) -> Self {
        let mut keys = vec![(start_s, points[0])];
        for pair in points.windows(2) {
            let length =
                ((pair[1][0] - pair[0][0]).powi(2) + (pair[1][1] - pair[0][1]).powi(2)).sqrt();
            let time = keys.last().map_or(start_s, |key| key.0) + length / speed_m_s;
            keys.push((time, pair[1]));
        }
        Self { keys }
    }

    /// Stands at the current end for `duration_s`.
    fn wait(mut self, duration_s: f64) -> Self {
        let (time, point) = *self.keys.last().expect("trajectory has a key");
        self.keys.push((time + duration_s, point));
        self
    }

    /// Continues through `points` at `speed_m_s`.
    fn then(mut self, points: &[[f64; 2]], speed_m_s: f64) -> Self {
        let (time, point) = *self.keys.last().expect("trajectory has a key");
        let mut all = vec![point];
        all.extend_from_slice(points);
        let tail = Trajectory::path(&all, speed_m_s, time);
        self.keys.extend_from_slice(&tail.keys[1..]);
        self
    }

    /// Position, velocity and heading at `t`, or `None` outside the trajectory.
    fn state(&self, t: f64) -> Option<([f64; 2], [f64; 2], f64)> {
        let first = self.keys.first()?;
        let last = self.keys.last()?;
        if t < first.0 || t > last.0 {
            return None;
        }
        let mut heading = 0.0;
        for pair in self.keys.windows(2) {
            let (t0, p0) = pair[0];
            let (t1, p1) = pair[1];
            let delta = [p1[0] - p0[0], p1[1] - p0[1]];
            if delta[0].hypot(delta[1]) > 1e-9 {
                heading = delta[1].atan2(delta[0]);
            }
            if t <= t1 && t1 > t0 {
                let s = (t - t0) / (t1 - t0);
                let velocity = [delta[0] / (t1 - t0), delta[1] / (t1 - t0)];
                return Some((
                    [p0[0] + s * delta[0], p0[1] + s * delta[1]],
                    velocity,
                    heading,
                ));
            }
        }
        Some((last.1, [0.0, 0.0], heading))
    }
}

/// A moving object.
#[derive(Clone, Debug)]
struct Mover {
    kind: Kind,
    trajectory: Trajectory,
}

fn person(trajectory: Trajectory) -> Mover {
    Mover {
        kind: Kind::Person,
        trajectory,
    }
}

/// A fixed box standing on the ground.
#[derive(Clone, Copy, Debug)]
struct Structure {
    center: [f64; 2],
    half_extents_m: [f64; 2],
    height_m: f64,
}

fn structure(center: [f64; 2], half_extents_m: [f64; 2], height_m: f64) -> Structure {
    Structure {
        center,
        half_extents_m,
        height_m,
    }
}

/// A pole: a thin structure the detector reports as a landmark.
fn pole(x: f64, y: f64) -> Structure {
    structure([x, y], [0.07, 0.07], 3.0)
}

/// Ego-pose noise added to the poses handed to the tracker (not to the scan).
#[derive(Clone, Copy, Debug)]
struct PoseNoise {
    yaw_std_rad: f64,
    translation_std_m: f64,
}

/// One perception task.
#[derive(Clone, Debug)]
struct Task {
    name: &'static str,
    description: &'static str,
    duration_s: f64,
    ego: Trajectory,
    movers: Vec<Mover>,
    structures: Vec<Structure>,
    pose_noise: Option<PoseNoise>,
    seed: u64,
}

fn standing_robot() -> Trajectory {
    Trajectory {
        keys: vec![(0.0, [0.0, 0.0]), (1.0e6, [0.0, 0.0])],
    }
}

fn corridor() -> Vec<Structure> {
    let mut structures = vec![
        structure([12.5, 4.3], [20.0, 0.1], 3.0),
        structure([12.5, -4.3], [20.0, 0.1], 3.0),
    ];
    for i in 0..7 {
        let x = f64::from(i) * 5.0;
        structures.push(pole(x, 3.4));
        structures.push(pole(x + 2.5, -3.4));
    }
    structures
}

fn corridor_people() -> Vec<Mover> {
    vec![
        person(Trajectory::path(&[[30.0, 1.5], [-5.0, 1.5]], 1.2, 0.0)),
        person(Trajectory::path(&[[28.0, -1.6], [-5.0, -1.6]], 1.0, 3.0)),
        person(Trajectory::path(&[[3.0, -0.8], [30.0, -0.8]], 1.3, 0.0)),
        person(Trajectory::path(&[[6.0, 2.4], [30.0, 2.4]], 0.9, 0.0)),
        person(Trajectory::path(&[[15.0, -3.6], [15.0, 3.6]], 1.0, 8.0)),
        person(Trajectory::path(&[[21.0, 3.6], [21.0, -3.6]], 1.1, 14.0)),
    ]
}

fn crowd() -> Vec<Mover> {
    let mut crowd = Vec::new();
    for (i, x) in [5.0, 6.0, 7.0, 8.0, 9.0, 5.5, 7.5].into_iter().enumerate() {
        let row = if i < 5 { 0.0 } else { 1.5 };
        let speed = 1.1 + 0.05 * i as f64;
        crowd.push(person(Trajectory::path(
            &[[x, 9.0 + row], [x, -9.0]],
            speed,
            0.2 * i as f64,
        )));
        crowd.push(person(Trajectory::path(
            &[[x + 0.5, -9.0 - row], [x + 0.5, 9.0]],
            speed + 0.07,
            0.15 * i as f64,
        )));
    }
    crowd
}

fn maneuvering_people() -> Vec<Mover> {
    vec![
        person(
            Trajectory::path(&[[6.0, -6.0], [6.0, -0.5]], 1.4, 0.0)
                .wait(2.5)
                .then(&[[6.0, 3.0]], 1.4)
                .wait(2.0)
                .then(&[[6.0, 8.0]], 1.4),
        ),
        person(Trajectory::path(
            &[
                [10.0, -6.0],
                [10.0, 3.0],
                [10.4, 3.5],
                [10.8, 3.0],
                [10.8, -7.0],
            ],
            1.2,
            0.0,
        )),
        person(
            Trajectory::path(&[[3.0, -12.0], [3.0, 12.0]], 3.0, 2.0)
                .then(&[[3.6, 12.0], [3.6, -12.0]], 3.0),
        ),
        person(Trajectory::path(
            &[
                [14.0, -7.0],
                [15.5, -3.5],
                [13.5, 0.0],
                [15.5, 3.5],
                [13.5, 7.0],
            ],
            1.3,
            0.0,
        )),
        person(
            Trajectory::path(&[[8.0, 5.0], [8.0, 5.0]], 1.0, 0.0)
                .wait(8.0)
                .then(&[[16.0, 9.0]], 1.5),
        ),
    ]
}

fn mixed_traffic_movers() -> Vec<Mover> {
    vec![
        person(Trajectory::path(&[[-5.0, -1.0], [22.0, -1.0]], 1.3, 0.0)),
        person(Trajectory::path(&[[18.0, 1.0], [-5.0, 1.0]], 1.1, 2.0)),
        person(
            Trajectory::path(&[[10.0, -2.0], [10.0, 2.3]], 1.4, 3.0)
                .wait(3.0)
                .then(&[[10.0, -2.0]], 1.4),
        ),
        Mover {
            kind: Kind::Cyclist,
            trajectory: Trajectory::path(&[[-20.0, 3.4], [30.0, 3.4]], 4.5, 1.0),
        },
        Mover {
            kind: Kind::Cyclist,
            trajectory: Trajectory::path(&[[30.0, 4.3], [-20.0, 4.3]], 4.0, 5.0),
        },
        Mover {
            kind: Kind::Car,
            trajectory: Trajectory::path(&[[-30.0, 6.5], [40.0, 6.5]], 8.0, 0.0),
        },
        Mover {
            kind: Kind::Car,
            trajectory: Trajectory::path(&[[-30.0, 6.5], [40.0, 6.5]], 8.0, 6.0),
        },
        Mover {
            kind: Kind::Car,
            trajectory: Trajectory::path(&[[-30.0, 6.5], [40.0, 6.5]], 7.0, 12.0),
        },
        Mover {
            kind: Kind::Car,
            trajectory: Trajectory::path(&[[40.0, 10.0], [-30.0, 10.0]], 7.0, 2.0),
        },
        Mover {
            kind: Kind::Car,
            trajectory: Trajectory::path(&[[40.0, 10.0], [-30.0, 10.0]], 9.0, 9.0),
        },
    ]
}

/// The task catalogue.
fn tasks() -> Vec<Task> {
    vec![
        Task {
            name: "people_basic",
            description: "Four people walking across, toward and away from a standing robot",
            duration_s: 20.0,
            ego: standing_robot(),
            movers: vec![
                person(Trajectory::path(&[[4.0, -6.0], [4.0, 8.0]], 1.2, 0.0)),
                person(Trajectory::path(&[[12.0, 7.0], [12.0, -7.0]], 1.0, 2.0)),
                person(Trajectory::path(&[[2.0, 3.0], [16.0, 3.0]], 1.3, 1.0)),
                person(Trajectory::path(&[[18.0, -2.0], [6.0, -2.0]], 1.1, 4.0)),
            ],
            structures: Vec::new(),
            pose_noise: None,
            seed: 1,
        },
        Task {
            name: "crowd_crossing",
            description: "Two groups of seven people crossing through each other in front of the robot",
            duration_s: 20.0,
            ego: standing_robot(),
            movers: crowd(),
            structures: Vec::new(),
            pose_noise: None,
            seed: 2,
        },
        Task {
            name: "occlusion",
            description: "People walking behind pillars, a kiosk and each other",
            duration_s: 20.0,
            ego: standing_robot(),
            movers: vec![
                person(Trajectory::path(&[[7.0, -8.0], [7.0, 8.0]], 1.2, 0.0)),
                person(Trajectory::path(&[[8.0, 8.0], [8.0, -8.0]], 1.1, 1.0)),
                person(Trajectory::path(&[[8.6, -8.0], [8.6, 8.0]], 1.0, 1.5)),
                person(Trajectory::path(&[[11.5, 8.0], [11.5, -8.0]], 1.3, 0.0)),
                person(Trajectory::path(&[[13.0, -8.0], [13.0, 8.0]], 0.9, 2.0)),
            ],
            structures: vec![
                structure([5.0, 1.2], [0.4, 0.4], 3.0),
                structure([5.0, -2.0], [0.4, 0.4], 3.0),
                structure([10.0, 0.0], [0.6, 1.6], 2.6),
            ],
            pose_noise: None,
            seed: 3,
        },
        Task {
            name: "maneuvering",
            description: "Stop-and-go, a U-turn, a jogger, a zig-zag and a person who starts from standing",
            duration_s: 20.0,
            ego: standing_robot(),
            movers: maneuvering_people(),
            structures: Vec::new(),
            pose_noise: None,
            seed: 4,
        },
        Task {
            name: "moving_robot",
            description: "The robot drives down a corridor at 1 m/s among oncoming, overtaken and crossing people",
            duration_s: 25.0,
            ego: Trajectory::path(&[[0.0, 0.0], [25.0, 0.0]], 1.0, 0.0),
            movers: corridor_people(),
            structures: corridor(),
            pose_noise: None,
            seed: 5,
        },
        Task {
            name: "pose_jitter",
            description: "As moving_robot, but the ego poses given to the tracker jitter by 1 deg and 0.1 m",
            duration_s: 25.0,
            ego: Trajectory::path(&[[0.0, 0.0], [25.0, 0.0]], 1.0, 0.0),
            movers: corridor_people(),
            structures: corridor(),
            pose_noise: Some(PoseNoise {
                yaw_std_rad: 1.0_f64.to_radians(),
                translation_std_m: 0.1,
            }),
            seed: 6,
        },
        Task {
            name: "mixed_traffic",
            description: "Pedestrians on a sidewalk next to a bike lane and a two-lane road",
            duration_s: 20.0,
            ego: standing_robot(),
            movers: mixed_traffic_movers(),
            structures: Vec::new(),
            pose_noise: None,
            seed: 7,
        },
        Task {
            name: "long_range",
            description: "People walking 15 to 35 m from the robot, a few LiDAR points each",
            duration_s: 20.0,
            ego: standing_robot(),
            movers: vec![
                person(Trajectory::path(&[[16.0, -8.0], [16.0, 8.0]], 1.2, 0.0)),
                person(Trajectory::path(&[[22.0, 8.0], [22.0, -8.0]], 1.1, 1.0)),
                person(Trajectory::path(&[[28.0, -8.0], [28.0, 8.0]], 1.3, 0.0)),
                person(Trajectory::path(&[[33.0, 6.0], [33.0, -6.0]], 1.0, 2.0)),
                person(Trajectory::path(&[[12.0, -3.0], [34.0, -3.0]], 1.3, 0.0)),
            ],
            structures: Vec::new(),
            pose_noise: None,
            seed: 8,
        },
        Task {
            name: "side_by_side",
            description: "Pairs walking 0.6 m apart (centre to centre); one pair splits up",
            duration_s: 20.0,
            ego: standing_robot(),
            movers: vec![
                person(Trajectory::path(&[[6.0, -8.0], [6.0, 8.0]], 1.2, 0.0)),
                person(Trajectory::path(&[[6.6, -8.0], [6.6, 8.0]], 1.2, 0.0)),
                person(Trajectory::path(&[[10.0, 8.0], [10.0, -8.0]], 1.0, 1.0)),
                person(Trajectory::path(&[[10.6, 8.0], [10.6, -8.0]], 1.0, 1.0)),
                person(Trajectory::path(&[[3.0, 2.0], [14.0, 2.0], [18.0, 1.0]], 1.2, 0.0)),
                person(Trajectory::path(&[[3.0, 2.6], [14.0, 2.6], [15.0, 7.0]], 1.2, 0.0)),
            ],
            structures: Vec::new(),
            pose_noise: None,
            seed: 9,
        },
    ]
}

/// `map` (x forward, y left, z up) to the engine's y-up world.
fn to_engine(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(x, z, -y)
}

/// The engine's y-up world to `map`.
fn from_engine(v: Vec3) -> [f64; 3] {
    [v.x, -v.z, v.y]
}

/// Rotation about the `map` z axis by `yaw` (a rotation about the engine's +y).
fn yaw_rotation(yaw: f64) -> Quat {
    Quat::from_rotation_y(yaw)
}

/// Deterministic splitmix64 generator with an explicit seed.
struct SplitMix(u64);

impl SplitMix {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1_u64 << 53) as f64
    }

    fn normal(&mut self) -> f64 {
        let u1 = self.uniform().max(1e-12);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

/// A 16-channel, 30-degree, 10 Hz spinning `LiDAR` (VLP-16 class).
fn lidar_spec(seed: u64) -> LidarSpec {
    LidarSpec {
        ray_count: 900,
        channel_count: 16,
        min_elevation_rad: (-15.0_f64).to_radians(),
        max_elevation_rad: TOP_BEAM_ELEVATION_RAD,
        min_range_m: 0.3,
        max_range_m: 60.0,
        height_offset_m: 0.0,
        range_noise_stddev_m: 0.02,
        dropout_probability: 0.01,
        seed,
        ..LidarSpec::default()
    }
}

/// One cluster of non-ground points, in the `map` frame.
#[derive(Clone, Debug)]
struct Cluster {
    centroid: [f64; 3],
    point_count: usize,
    major_extent_m: f64,
    minor_extent_m: f64,
    top_m: f64,
    /// Corners of the footprint box from the ground to the top, `map` frame.
    corners_m: [[f64; 3]; 8],
}

/// What the detector makes of a cluster.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Label {
    Object(Kind),
    Landmark,
    Ignored,
}

/// Labels a cluster `range_m` from the sensor.
fn classify(cluster: &Cluster, range_m: f64) -> Label {
    let height = cluster.top_m;
    // A thin cluster that reaches above head height, or reaches the top beam,
    // is a pole. Near the sensor the top beam passes below a pole's top; a
    // person close enough to reach it is wider than a pole.
    let top_beam_m = SENSOR_HEIGHT_M + range_m * TOP_BEAM_ELEVATION_RAD.tan();
    let reaches_top = height > POLE_MIN_HEIGHT_M || height > top_beam_m - 0.1;
    if cluster.major_extent_m < POLE_MAX_WIDTH_M && reaches_top {
        return Label::Landmark;
    }
    if height > STRUCTURE_HEIGHT_M || cluster.point_count < MIN_CLUSTER_POINTS {
        return Label::Ignored;
    }
    let major = cluster.major_extent_m;
    if (3.0..=6.0).contains(&major) && (1.2..=2.2).contains(&height) {
        Label::Object(Kind::Car)
    } else if (1.2..3.0).contains(&major)
        && cluster.minor_extent_m < 0.9
        && (1.3..=2.0).contains(&height)
    {
        Label::Object(Kind::Cyclist)
    } else if major < 1.2 && (0.9..=POLE_MIN_HEIGHT_M).contains(&height) {
        Label::Object(Kind::Person)
    } else {
        Label::Ignored
    }
}

/// Euclidean clustering of the non-ground points on a horizontal grid.
///
/// Cells within `link` cells of each other join. The link grows with range so
/// that neighbouring azimuth columns, which spread apart with distance, stay in
/// one cluster: a far car must not fall apart into person-sized pieces.
fn cluster_points(points: &[[f64; 3]], sensor: [f64; 2], azimuth_step_rad: f64) -> Vec<Cluster> {
    let mut cells: BTreeMap<(i64, i64), Vec<usize>> = BTreeMap::new();
    for (index, point) in points.iter().enumerate() {
        let key = (
            (point[0] / CLUSTER_CELL_M).floor() as i64,
            (point[1] / CLUSTER_CELL_M).floor() as i64,
        );
        cells.entry(key).or_default().push(index);
    }
    let mut visited: BTreeMap<(i64, i64), bool> = cells.keys().map(|key| (*key, false)).collect();
    let mut clusters = Vec::new();
    for start in cells.keys() {
        if visited[start] {
            continue;
        }
        let mut members = Vec::new();
        let mut queue = VecDeque::from([*start]);
        visited.insert(*start, true);
        while let Some(cell) = queue.pop_front() {
            members.extend_from_slice(&cells[&cell]);
            let range = ((cell.0 as f64 + 0.5) * CLUSTER_CELL_M - sensor[0])
                .hypot((cell.1 as f64 + 0.5) * CLUSTER_CELL_M - sensor[1]);
            let link = 1 + (1.5 * range * azimuth_step_rad / CLUSTER_CELL_M).floor() as i64;
            for dx in -link..=link {
                for dy in -link..=link {
                    let next = (cell.0 + dx, cell.1 + dy);
                    if visited.get(&next) == Some(&false) {
                        visited.insert(next, true);
                        queue.push_back(next);
                    }
                }
            }
        }
        clusters.push(describe(points, &members));
    }
    clusters
}

fn describe(points: &[[f64; 3]], members: &[usize]) -> Cluster {
    let n = members.len() as f64;
    let mut centroid = [0.0; 3];
    let mut top = f64::MIN;
    for &i in members {
        for (axis, value) in centroid.iter_mut().enumerate() {
            *value += points[i][axis] / n;
        }
        top = top.max(points[i][2]);
    }
    // Principal axes of the footprint.
    let (mut sxx, mut sxy, mut syy) = (0.0, 0.0, 0.0);
    for &i in members {
        let dx = points[i][0] - centroid[0];
        let dy = points[i][1] - centroid[1];
        sxx += dx * dx;
        sxy += dx * dy;
        syy += dy * dy;
    }
    let angle = 0.5 * (2.0 * sxy).atan2(sxx - syy);
    let (sin, cos) = angle.sin_cos();
    let (mut major_min, mut major_max) = (f64::MAX, f64::MIN);
    let (mut minor_min, mut minor_max) = (f64::MAX, f64::MIN);
    for &i in members {
        let dx = points[i][0] - centroid[0];
        let dy = points[i][1] - centroid[1];
        let major = dx * cos + dy * sin;
        let minor = -dx * sin + dy * cos;
        major_min = major_min.min(major);
        major_max = major_max.max(major);
        minor_min = minor_min.min(minor);
        minor_max = minor_max.max(minor);
    }
    let (a, b) = (major_max - major_min, minor_max - minor_min);
    // Report the centre of the footprint box rather than the point mean,
    // which leans toward the side the sensor sees.
    let mid_major = 0.5 * (major_max + major_min);
    let mid_minor = 0.5 * (minor_max + minor_min);
    let center = [
        centroid[0] + mid_major * cos - mid_minor * sin,
        centroid[1] + mid_major * sin + mid_minor * cos,
        centroid[2],
    ];
    let mut corners_m = [[0.0; 3]; 8];
    for (k, corner) in corners_m.iter_mut().enumerate() {
        let u = if k & 1 == 0 { major_min } else { major_max };
        let v = if k & 2 == 0 { minor_min } else { minor_max };
        let z = if k & 4 == 0 { 0.0 } else { top };
        *corner = [
            centroid[0] + u * cos - v * sin,
            centroid[1] + u * sin + v * cos,
            z,
        ];
    }
    Cluster {
        centroid: center,
        point_count: members.len(),
        corners_m,
        major_extent_m: a.max(b),
        minor_extent_m: a.min(b),
        top_m: top,
    }
}

/// Rigid pose of the `LiDAR` in the `map` plane.
#[derive(Clone, Copy, Debug)]
struct Pose2 {
    x: f64,
    y: f64,
    yaw: f64,
}

impl Pose2 {
    /// `map` point into the `LiDAR` frame.
    fn to_sensor(self, p: [f64; 3]) -> [f64; 3] {
        let (sin, cos) = self.yaw.sin_cos();
        let dx = p[0] - self.x;
        let dy = p[1] - self.y;
        [
            cos * dx + sin * dy,
            -sin * dx + cos * dy,
            p[2] - SENSOR_HEIGHT_M,
        ]
    }

    /// Row-major `map` <- `lidar` transform.
    fn matrix(self) -> Vec<f64> {
        let (sin, cos) = self.yaw.sin_cos();
        vec![
            cos,
            -sin,
            0.0,
            self.x,
            sin,
            cos,
            0.0,
            self.y,
            0.0,
            0.0,
            1.0,
            SENSOR_HEIGHT_M,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
    }
}

/// Everything one task writes.
#[derive(Debug)]
struct TaskOutput {
    setup: Value,
    inputs: Vec<Value>,
    truth: Vec<Value>,
    summary: Value,
    /// One `camera_frame` record per rendered frame (empty without a camera).
    camera_frames: Vec<Value>,
}

fn spawn_box(
    world: &mut World,
    name: &str,
    body_type: RigidBodyType,
    shape: ColliderShape,
) -> Entity {
    let entity = spawn_named(world, name);
    world.entity_mut(entity).insert((
        RigidBody {
            body_type,
            ..RigidBody::default()
        },
        Collider {
            shape,
            ..Collider::default()
        },
        Transform3::from_translation_rotation(Vec3::new(0.0, PARKED_DEPTH_M, 0.0), Quat::IDENTITY),
    ));
    entity
}

fn build_world(task: &Task) -> (World, Vec<Entity>) {
    let mut world = World::new();
    let ground = spawn_box(
        &mut world,
        "ground",
        RigidBodyType::Fixed,
        ColliderShape::Cuboid {
            half_extents_m: Vec3::new(200.0, 0.5, 200.0),
        },
    );
    world
        .entity_mut(ground)
        .insert(Transform3::from_translation_rotation(
            Vec3::new(0.0, -0.5, 0.0),
            Quat::IDENTITY,
        ));
    for (i, s) in task.structures.iter().enumerate() {
        let entity = spawn_box(
            &mut world,
            &format!("structure_{i}"),
            RigidBodyType::Fixed,
            ColliderShape::Cuboid {
                half_extents_m: Vec3::new(
                    s.half_extents_m[0],
                    s.height_m / 2.0,
                    s.half_extents_m[1],
                ),
            },
        );
        world
            .entity_mut(entity)
            .insert(Transform3::from_translation_rotation(
                to_engine(s.center[0], s.center[1], s.height_m / 2.0),
                Quat::IDENTITY,
            ));
    }
    let movers = task
        .movers
        .iter()
        .enumerate()
        .map(|(i, mover)| {
            spawn_box(
                &mut world,
                &format!("mover_{i}"),
                RigidBodyType::Kinematic,
                mover.kind.shape(),
            )
        })
        .collect();
    (world, movers)
}

/// Places every mover at time `t` (or parks it) and returns the truth objects
/// as `(truth_id, kind, map position)`.
fn place_movers(
    world: &mut World,
    task: &Task,
    entities: &[Entity],
    t: f64,
) -> Vec<(u32, Kind, [f64; 3])> {
    let mut present = Vec::new();
    for (i, (mover, entity)) in task.movers.iter().zip(entities).enumerate() {
        let transform = match mover.trajectory.state(t) {
            Some((position, _, heading)) => {
                let center = [position[0], position[1], mover.kind.center_height_m()];
                present.push((i as u32 + 1, mover.kind, center));
                Transform3::from_translation_rotation(
                    to_engine(center[0], center[1], center[2]),
                    yaw_rotation(heading),
                )
            }
            None => Transform3::from_translation_rotation(
                Vec3::new(0.0, PARKED_DEPTH_M, 0.0),
                Quat::IDENTITY,
            ),
        };
        world.entity_mut(*entity).insert(transform);
    }
    present
}

fn ego_pose(task: &Task, t: f64) -> Pose2 {
    let (position, _, heading) = task
        .ego
        .state(t)
        .expect("the ego trajectory covers the task");
    Pose2 {
        x: position[0],
        y: position[1],
        yaw: heading,
    }
}

/// Detections of one scan in the `map` frame.
#[derive(Clone, Debug)]
struct ScanDetections {
    objects: Vec<(Kind, [f64; 3], usize)>,
    /// Box corners of each object, `map` frame.
    corners: Vec<[[f64; 3]; 8]>,
    landmarks: Vec<[f64; 3]>,
}

fn scan(
    backend: &RapierBackend,
    physics_world: PhysicsWorldId,
    world: &World,
    pose: Pose2,
    spec: &LidarSpec,
    key: SensorNoiseKey,
) -> ScanDetections {
    let mount = Transform3::from_translation_rotation(
        to_engine(pose.x, pose.y, SENSOR_HEIGHT_M),
        yaw_rotation(pose.yaw),
    );
    let cloud = sample_lidar_keyed(backend, physics_world, world, &mount, spec, key);
    let points: Vec<[f64; 3]> = cloud
        .points_m
        .iter()
        .map(|p| from_engine(*p))
        .filter(|p| p[2] > GROUND_CLEARANCE_M)
        .collect();
    let mut detections = ScanDetections {
        objects: Vec::new(),
        corners: Vec::new(),
        landmarks: Vec::new(),
    };
    let azimuth_step_rad = (spec.max_angle_rad - spec.min_angle_rad) / f64::from(spec.ray_count);
    for cluster in cluster_points(&points, [pose.x, pose.y], azimuth_step_rad) {
        let range = (cluster.centroid[0] - pose.x).hypot(cluster.centroid[1] - pose.y);
        match classify(&cluster, range) {
            Label::Object(kind) => {
                detections
                    .objects
                    .push((kind, cluster.centroid, cluster.point_count));
                detections.corners.push(cluster.corners_m);
            }
            Label::Landmark => {
                if range <= LANDMARK_RANGE_M {
                    detections.landmarks.push(cluster.centroid);
                }
            }
            Label::Ignored => {}
        }
    }
    detections
}

/// Greedy one-to-one matching of detections to truth of the same class.
fn count_hits(objects: &[(Kind, [f64; 3], usize)], truth: &[(u32, Kind, [f64; 3])]) -> usize {
    let mut pairs = Vec::new();
    for (i, (kind, position, _)) in objects.iter().enumerate() {
        for (j, (_, truth_kind, truth_position)) in truth.iter().enumerate() {
            if kind != truth_kind {
                continue;
            }
            let d = (position[0] - truth_position[0]).hypot(position[1] - truth_position[1]);
            if d <= detection_match_m(*kind) {
                pairs.push((d, i, j));
            }
        }
    }
    pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut used_detections = vec![false; objects.len()];
    let mut used_truth = vec![false; truth.len()];
    let mut hits = 0;
    for (_, i, j) in pairs {
        if !used_detections[i] && !used_truth[j] {
            used_detections[i] = true;
            used_truth[j] = true;
            hits += 1;
        }
    }
    hits
}

/// The `lidar_batch` detection records of one scan, in the LiDAR frame.
fn detection_records(detections: &ScanDetections, pose: Pose2) -> Vec<Value> {
    detections
        .objects
        .iter()
        .zip(&detections.corners)
        .map(|((kind, position, count), corners)| {
            let sensor = pose.to_sensor(*position);
            // Box corners relative to the detection, LiDAR frame, for
            // projecting the detection into the camera.
            let offsets: Vec<f64> = corners
                .iter()
                .flat_map(|corner| {
                    let c = pose.to_sensor(*corner);
                    [c[0] - sensor[0], c[1] - sensor[1], c[2] - sensor[2]]
                })
                .collect();
            // Box size from the corners (corner bits: 1 = far along the major
            // axis, 2 = across, 4 = top); a one-column cluster has no width.
            let edge = |a: usize, b: usize| {
                (corners[a][0] - corners[b][0])
                    .hypot(corners[a][1] - corners[b][1])
                    .max(MIN_BOX_SIZE_M)
            };
            let top = corners[7][2].max(MIN_BOX_SIZE_M);
            let range = sensor[0].hypot(sensor[1]);
            let sigma = 0.08 + 0.01 * range;
            let score = 1.0 - (-(*count as f64) / 12.0).exp();
            json!({
                "position": sensor,
                "covariance": [sigma * sigma, 0.0, 0.0, 0.0, sigma * sigma, 0.0, 0.0, 0.0, 0.04],
                "class": kind.class(),
                "score": (score * 1000.0).round() / 1000.0,
                "shape": {"height": top, "width": edge(2, 0), "length": edge(1, 0), "rotation_y": 0.0, "corners": offsets},
            })
        })
        .collect()
}

/// Where camera frames go, when the task has a camera.
struct CameraOutput<'a> {
    rig: &'a camera::CameraRig,
    /// Task directory; images go to its `camera/` folder.
    dir: &'a Path,
}

fn write_png(path: &Path, rgba8: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let mut encoder = png::Encoder::new(fs::File::create(path)?, width, height);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder
        .write_header()
        .and_then(|mut writer| writer.write_image_data(rgba8))
        .map_err(std::io::Error::other)
}

fn simulate(task: &Task, duration_s: f64, camera: Option<&CameraOutput>) -> TaskOutput {
    let (mut world, movers) = build_world(task);
    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    let spec = lidar_spec(task.seed);
    let mut noise = SplitMix(task.seed.wrapping_mul(0x2545_F491_4F6C_DD1D));
    let scans = (duration_s / SCAN_PERIOD_S).round() as u64;
    let step = SimDuration::from_hertz(Hertz::new(1000.0));

    let mut inputs = Vec::new();
    let mut truth = Vec::new();
    let mut poses = Vec::new();
    let (mut truth_count, mut detection_count, mut hit_count) = (0_usize, 0_usize, 0_usize);
    let mut person_points = Vec::new();
    let mut camera_frames = Vec::new();
    if let Some(output) = camera {
        fs::create_dir_all(output.dir.join("camera")).expect("camera directory");
    }

    for k in 0..scans {
        let t = k as f64 * SCAN_PERIOD_S;
        let timestamp_ns = (k * 100_000_000) as i64;
        let present = place_movers(&mut world, task, &movers, t);
        backend
            .sync_from_ecs(&mut world, physics_world)
            .expect("sync scene");
        // Colliders follow their (teleported) bodies only once the world steps.
        // Nothing here is dynamic, so the step moves nothing else.
        backend.step(physics_world, step).expect("step scene");
        let pose = ego_pose(task, t);
        let key = SensorNoiseKey::new(task.seed, spec.seed, 1, k);
        let detections = scan(&backend, physics_world, &world, pose, &spec, key);

        let visible: Vec<(u32, Kind, [f64; 3])> = present
            .into_iter()
            .filter(|(_, _, p)| (p[0] - pose.x).hypot(p[1] - pose.y) <= TRUTH_RANGE_M)
            .collect();
        truth_count += visible.len();
        detection_count += detections.objects.len();
        hit_count += count_hits(&detections.objects, &visible);
        person_points.extend(
            detections
                .objects
                .iter()
                .filter(|(kind, _, _)| *kind == Kind::Person)
                .map(|(_, _, n)| *n),
        );

        // The pose handed to the tracker; the scan itself used the true pose.
        let reported = match task.pose_noise {
            Some(noise_spec) => Pose2 {
                x: pose.x + noise.normal() * noise_spec.translation_std_m,
                y: pose.y + noise.normal() * noise_spec.translation_std_m,
                yaw: pose.yaw + noise.normal() * noise_spec.yaw_std_rad,
            },
            None => pose,
        };
        poses.push(json!({"timestamp_ns": timestamp_ns, "transform": reported.matrix()}));

        if let Some(output) = camera.filter(|_| k % camera::SCANS_PER_FRAME == 0) {
            let frame = output.rig.render(task, t, pose);
            let image = format!("camera/{k:06}.png");
            write_png(
                &output.dir.join(&image),
                &frame.rgba8,
                camera::IMAGE_WIDTH,
                camera::IMAGE_HEIGHT,
            )
            .expect("write camera frame");
            camera_frames.push(camera::frame_record(timestamp_ns, &image, &frame));
        }

        let detection_records = detection_records(&detections, pose);
        let landmark_records: Vec<Value> = detections
            .landmarks
            .iter()
            .map(|p| json!({"position": pose.to_sensor(*p)}))
            .collect();
        inputs.push(json!({
            "type": "lidar_batch",
            "schema_version": 1,
            "sensor_id": 1,
            "sequence": k,
            "event_time_ns": timestamp_ns,
            "arrival_time_ns": timestamp_ns,
            "frame_id": "lidar",
            "detections": detection_records,
            "landmarks": landmark_records,
        }));
        truth.push(json!({
            "type": "truth3d",
            "schema_version": 1,
            "timestamp_ns": timestamp_ns,
            "objects": visible
                .iter()
                .map(|(id, kind, p)| json!({"truth_id": id, "class": kind.class(), "position": p}))
                .collect::<Vec<_>>(),
        }));
    }

    person_points.sort_unstable();
    let ratio = |a: usize, b: usize| if b == 0 { 0.0 } else { a as f64 / b as f64 };
    let summary = json!({
        "task": task.name,
        "description": task.description,
        "duration_s": duration_s,
        "scans": scans,
        "objects": task.movers.len(),
        "truth_object_scans": truth_count,
        "detections": detection_count,
        "detection_recall": ratio(hit_count, truth_count),
        "detection_precision": ratio(hit_count, detection_count),
        "median_points_per_person_detection": person_points.get(person_points.len() / 2).copied().unwrap_or(0),
        "ego_pose_noise": task.pose_noise.map(|n| json!({
            "yaw_std_deg": n.yaw_std_rad.to_degrees(),
            "translation_std_m": n.translation_std_m,
        })),
    });
    let identity = [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];
    let setup = json!({
        "tracking_frame": "map",
        "lidar_frame": "lidar",
        "camera_frame": "camera",
        "lidar_to_reference": identity,
        "camera_to_reference": camera::camera_to_lidar(),
        "intrinsics": camera::intrinsics(),
        "near_plane": 0.3,
        "poses": poses,
        "pose_max_gap_ns": 200_000_000,
    });
    TaskOutput {
        setup,
        inputs,
        truth,
        summary,
        camera_frames,
    }
}

fn jsonl(records: &[Value]) -> String {
    let mut text = String::new();
    for record in records {
        text.push_str(&record.to_string());
        text.push('\n');
    }
    text
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut out = PathBuf::from("target/perception_tasks");
    let mut only: Option<String> = None;
    let mut duration: Option<f64> = None;
    let mut with_camera = false;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--out" => out = PathBuf::from(args.next().ok_or("--out needs a directory")?),
            "--task" => only = Some(args.next().ok_or("--task needs a name")?),
            "--duration-s" => {
                duration = Some(args.next().ok_or("--duration-s needs seconds")?.parse()?)
            }
            "--camera" => with_camera = true,
            "--list" => {
                for task in tasks() {
                    println!("{:16} {}", task.name, task.description);
                }
                return Ok(());
            }
            other => return Err(format!("unknown argument {other}").into()),
        }
    }

    let selected: Vec<Task> = tasks()
        .into_iter()
        .filter(|task| only.as_deref().is_none_or(|name| name == task.name))
        .collect();
    // Tasks are independent worlds: simulate them side by side. Each scan is
    // also split across threads by `rne_sensor`; outputs do not depend on either.
    let rig = if with_camera {
        Some(camera::CameraRig::new()?)
    } else {
        None
    };
    let dirs: Vec<PathBuf> = selected.iter().map(|task| out.join(task.name)).collect();
    let outputs: Vec<TaskOutput> = std::thread::scope(|scope| {
        let handles: Vec<_> = selected
            .iter()
            .zip(&dirs)
            .map(|(task, dir)| {
                let rig = rig.as_ref();
                scope.spawn(move || {
                    let camera = rig.map(|rig| CameraOutput { rig, dir });
                    simulate(task, duration.unwrap_or(task.duration_s), camera.as_ref())
                })
            })
            .collect();
        handles
            .into_iter()
            .map(|handle| {
                handle
                    .join()
                    .unwrap_or_else(|panic| std::panic::resume_unwind(panic))
            })
            .collect()
    });
    let mut manifest = Vec::new();
    for (task, output) in selected.iter().zip(outputs) {
        let dir = out.join(task.name);
        fs::create_dir_all(&dir)?;
        fs::write(
            dir.join("setup.json"),
            serde_json::to_string_pretty(&output.setup)?,
        )?;
        fs::write(dir.join("inputs.jsonl"), jsonl(&output.inputs))?;
        fs::write(dir.join("truth.jsonl"), jsonl(&output.truth))?;
        if !output.camera_frames.is_empty() {
            fs::write(
                dir.join("camera_frames.jsonl"),
                jsonl(&output.camera_frames),
            )?;
        }
        fs::write(
            dir.join("summary.json"),
            serde_json::to_string_pretty(&output.summary)?,
        )?;
        println!(
            "{:16} recall {:.3}  precision {:.3}  median points/person {:>3}  -> {}",
            task.name,
            output.summary["detection_recall"].as_f64().unwrap_or(0.0),
            output.summary["detection_precision"]
                .as_f64()
                .unwrap_or(0.0),
            output.summary["median_points_per_person_detection"],
            dir.display()
        );
        manifest.push(output.summary);
    }
    if manifest.is_empty() {
        return Err("no task matched --task; see --list".into());
    }
    fs::create_dir_all(&out)?;
    fs::write(
        out.join("tasks.json"),
        serde_json::to_string_pretty(&manifest)?,
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn single_walker() -> Task {
        Task {
            name: "test_walker",
            description: "one person walking across",
            duration_s: 3.0,
            ego: standing_robot(),
            movers: vec![person(Trajectory::path(
                &[[5.0, -2.0], [5.0, 2.0]],
                1.0,
                0.0,
            ))],
            structures: vec![pole(3.0, 3.0)],
            pose_noise: None,
            seed: 11,
        }
    }

    #[test]
    fn trajectory_interpolates_waits_and_ends() {
        let trajectory = Trajectory::path(&[[0.0, 0.0], [2.0, 0.0]], 1.0, 1.0).wait(1.0);
        assert!(trajectory.state(0.5).is_none());
        let (position, velocity, _) = trajectory.state(2.0).unwrap();
        assert!((position[0] - 1.0).abs() < 1e-12 && (velocity[0] - 1.0).abs() < 1e-12);
        let (position, velocity, _) = trajectory.state(3.5).unwrap();
        assert!((position[0] - 2.0).abs() < 1e-12 && velocity[0].abs() < 1e-12);
        assert!(trajectory.state(4.5).is_none());
    }

    #[test]
    fn moving_person_is_detected_where_it_is() {
        let task = single_walker();
        let output = simulate(&task, task.duration_s, None);
        assert_eq!(output.inputs.len(), 30);
        let mut hits = 0;
        for (k, input) in output.inputs.iter().enumerate() {
            let truth_y = -2.0 + k as f64 * SCAN_PERIOD_S;
            let detections = input["detections"].as_array().unwrap();
            if detections.iter().any(|d| {
                let p = &d["position"];
                d["class"] == 1
                    && (p[0].as_f64().unwrap() - 5.0).abs() < 0.4
                    && (p[1].as_f64().unwrap() - truth_y).abs() < 0.3
            }) {
                hits += 1;
            }
            // The pole is reported as a landmark, never as a detection.
            assert_eq!(input["landmarks"].as_array().unwrap().len(), 1);
            assert!(detections.len() <= 1);
        }
        assert!(hits >= 28, "person detected in {hits} of 30 scans");
        assert!(output.summary["detection_recall"].as_f64().unwrap() > 0.9);
    }

    #[test]
    fn detections_are_in_the_moving_lidar_frame() {
        let mut task = single_walker();
        task.ego = Trajectory::path(&[[0.0, 0.0], [3.0, 0.0]], 1.0, 0.0);
        task.movers = vec![person(Trajectory::path(
            &[[6.0, 0.0], [6.0, 0.1]],
            0.01,
            0.0,
        ))];
        let output = simulate(&task, 2.0, None);
        for (k, input) in output.inputs.iter().enumerate() {
            let expected_x = 6.0 - k as f64 * SCAN_PERIOD_S;
            let detection = &input["detections"][0];
            let x = detection["position"][0].as_f64().unwrap();
            // The centroid sits on the visible (near) side of the body.
            assert!(
                (x - expected_x).abs() < 0.3,
                "scan {k}: {x} vs {expected_x}"
            );
        }
    }

    #[test]
    fn side_by_side_people_merge_into_one_cluster() {
        let task = Task {
            movers: vec![
                // Side by side as seen from the sensor: walking away from it.
                person(Trajectory::path(&[[5.0, -0.3], [7.0, -0.3]], 1.0, 0.0)),
                person(Trajectory::path(&[[5.0, 0.3], [7.0, 0.3]], 1.0, 0.0)),
            ],
            structures: Vec::new(),
            ..single_walker()
        };
        let output = simulate(&task, 1.0, None);
        for input in &output.inputs {
            assert_eq!(input["detections"].as_array().unwrap().len(), 1);
        }
    }

    /// Projects a LiDAR-frame point into the image with the written calibration.
    fn project(setup: &Value, p: [f64; 3]) -> [f64; 2] {
        let m: Vec<f64> = setup["camera_to_reference"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect();
        // Inverse of a rigid transform: R^T (p - t).
        let d = [p[0] - m[3], p[1] - m[7], p[2] - m[11]];
        let c: Vec<f64> = (0..3)
            .map(|col| m[col] * d[0] + m[4 + col] * d[1] + m[8 + col] * d[2])
            .collect();
        let k = &setup["intrinsics"];
        let f = |key: &str| k[key].as_f64().unwrap();
        [
            f("fx") * c[0] / c[2] + f("cx"),
            f("fy") * c[1] / c[2] + f("cy"),
        ]
    }

    #[test]
    fn camera_truth_matches_projected_lidar_boxes() {
        let rig = camera::CameraRig::new().unwrap();
        let dir = std::env::temp_dir().join(format!("rne_149_camera_{}", std::process::id()));
        let task = single_walker();
        let output = simulate(
            &task,
            1.0,
            Some(&CameraOutput {
                rig: &rig,
                dir: &dir,
            }),
        );
        assert_eq!(output.camera_frames.len(), 2);
        for (frame, input) in output.camera_frames.iter().zip(
            output
                .inputs
                .iter()
                .step_by(camera::SCANS_PER_FRAME as usize),
        ) {
            assert!(dir.join(frame["image"].as_str().unwrap()).exists());
            let objects = frame["objects"].as_array().unwrap();
            assert_eq!(objects.len(), 1, "{frame}");
            let truth: Vec<f64> = objects[0]["bbox"]
                .as_array()
                .unwrap()
                .iter()
                .map(|v| v.as_f64().unwrap())
                .collect();
            // The person walks on the right of the image (y < 0 at first).
            assert!(truth[0] > f64::from(camera::IMAGE_WIDTH) / 2.0, "{truth:?}");
            // The LiDAR detection's box, projected with the written
            // calibration, lands on the rendered person.
            let detection = &input["detections"][0];
            let position: Vec<f64> = detection["position"]
                .as_array()
                .unwrap()
                .iter()
                .map(|v| v.as_f64().unwrap())
                .collect();
            let corners: Vec<f64> = detection["shape"]["corners"]
                .as_array()
                .unwrap()
                .iter()
                .map(|v| v.as_f64().unwrap())
                .collect();
            let (mut x0, mut y0, mut x1, mut y1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
            for c in corners.chunks(3) {
                let [u, v] = project(
                    &output.setup,
                    [position[0] + c[0], position[1] + c[1], position[2] + c[2]],
                );
                (x0, y0, x1, y1) = (x0.min(u), y0.min(v), x1.max(u), y1.max(v));
            }
            let ix = (x1.min(truth[2]) - x0.max(truth[0])).max(0.0);
            let iy = (y1.min(truth[3]) - y0.max(truth[1])).max(0.0);
            let inter = ix * iy;
            let union =
                (x1 - x0) * (y1 - y0) + (truth[2] - truth[0]) * (truth[3] - truth[1]) - inter;
            assert!(
                inter / union > 0.5,
                "IoU {} for {:?} vs {truth:?}",
                inter / union,
                [x0, y0, x1, y1]
            );
        }
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn simulation_is_deterministic() {
        let task = tasks()
            .into_iter()
            .find(|t| t.name == "pose_jitter")
            .unwrap();
        let a = simulate(&task, 0.5, None);
        let b = simulate(&task, 0.5, None);
        assert_eq!(jsonl(&a.inputs), jsonl(&b.inputs));
        assert_eq!(a.setup, b.setup);
        assert_eq!(jsonl(&a.truth), jsonl(&b.truth));
    }
}
