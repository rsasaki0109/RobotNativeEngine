//! Headless deterministic kinematic warehouse logistics experiment.
//!
//! Forty AMRs dispatch seeded pick/transport/drop tasks with non-preemptive
//! cell, edge, and crossing reservations. The recorded model uses bounded
//! kinematics and scenario energy accounting, without wheel/contact dynamics.

use rne_core::{SimClock, SimDuration};
use rne_nav::{GridCoord, TrafficConfig, TrafficCoordinator};
use rne_world::WorldRandom;
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::cmp::Ordering;
use std::collections::{BTreeMap, BinaryHeap, VecDeque};
use std::fs;
use std::path::PathBuf;

const SEED: u64 = 20261010;
const DT: f64 = 0.05;
const STEPS: usize = 4800;
const CAPTURE_START_S: f64 = 100.0;
const CAPTURE_END_S: f64 = 220.0;
const INCIDENT_START_S: f64 = 120.0;
const INCIDENT_END_S: f64 = 165.0;
const INTERPOLATION_ERROR_M: f64 = ACCEL_M_S2 * DT * DT / 4.0;
const ROBOTS: usize = 40;
const RADIUS_M: f64 = 0.65;
const SAFETY_M: f64 = 1.4;
const ACCEL_M_S2: f64 = 1.8;
const MAX_SPEED_M_S: f64 = 1.2;
const CHARGE_TRIGGER_PERCENT: f64 = 26.0;
const LEASE_S: f64 = 1.0;
const JUNCTION_ZONE_M: f64 = 2.01;
const YAW_RATE_RAD_S: f64 = 1.0;
const CHARGE_RATE_PERCENT_S: f64 = 0.6;
const FAIRNESS_DISTANCE_M: f64 = 18.0;
const EMPTY_ENERGY_PERCENT_M: f64 = 0.018;
const LOADED_ENERGY_PERCENT_M: f64 = 0.026;
const DRIVE_ENERGY_PERCENT_S: f64 = 0.002;
const X_LANES: [i32; 7] = [3, 10, 17, 24, 31, 38, 43];
const Z_LANES: [i32; 5] = [3, 10, 17, 24, 29];

#[derive(Clone, Copy, Debug, Serialize)]
struct Point {
    x: f64,
    z: f64,
}

impl Point {
    fn distance(self, other: Self) -> f64 {
        (self.x - other.x).hypot(self.z - other.z)
    }
    fn lerp(self, other: Self, t: f64) -> Self {
        Self {
            x: self.x + (other.x - self.x) * t,
            z: self.z + (other.z - self.z) * t,
        }
    }
    fn array(self) -> [f64; 2] {
        [self.x, self.z]
    }
}

#[derive(Clone, Debug, Serialize)]
struct Rack {
    x_m: f64,
    z_m: f64,
    width_m: f64,
    depth_m: f64,
    height_m: f64,
}

#[derive(Clone, Debug, Serialize)]
struct Station {
    id: usize,
    x_m: f64,
    z_m: f64,
    kind: &'static str,
    #[serde(skip)]
    node: usize,
}

#[derive(Clone, Debug)]
struct Graph {
    points: Vec<Point>,
    edges: Vec<Vec<usize>>,
    racks: Vec<Rack>,
    stations: Vec<Station>,
    lanes: Vec<[[f64; 2]; 2]>,
    corridor: Vec<usize>,
    corridor_entry: usize,
    parking: Vec<usize>,
    junctions: Vec<usize>,
    lane_node_count: usize,
}

impl Graph {
    fn new() -> Self {
        let mut graph = Self {
            points: Vec::new(),
            edges: Vec::new(),
            racks: Vec::new(),
            stations: Vec::new(),
            lanes: Vec::new(),
            corridor: Vec::new(),
            corridor_entry: 0,
            parking: Vec::new(),
            junctions: Vec::new(),
            lane_node_count: 0,
        };
        let mut ids = BTreeMap::<(i32, i32), usize>::new();
        for x in 3..=43 {
            for z in 3..=29 {
                if X_LANES.contains(&x) || Z_LANES.contains(&z) {
                    ids.insert((x, z), graph.points.len());
                    graph.points.push(Point {
                        x: f64::from(x),
                        z: f64::from(z),
                    });
                    graph.edges.push(Vec::new());
                }
            }
        }
        for (&(x, z), &id) in &ids {
            if X_LANES.contains(&x) && Z_LANES.contains(&z) {
                graph.junctions.push(id);
            }
            if Z_LANES.contains(&z) {
                let east = z == 3 || z == 17;
                let nx = x + if east { 1 } else { -1 };
                if let Some(&next) = ids.get(&(nx, z)) {
                    graph.edges[id].push(next);
                }
            }
            if X_LANES.contains(&x) {
                let north = matches!(x, 10 | 24 | 38 | 43);
                let nz = z + if north { 1 } else { -1 };
                if let Some(&next) = ids.get(&(x, nz)) {
                    graph.edges[id].push(next);
                }
            }
        }
        for &z in &Z_LANES {
            let east = z == 3 || z == 17;
            graph.lanes.push(if east {
                [[3.0, z as f64], [43.0, z as f64]]
            } else {
                [[43.0, z as f64], [3.0, z as f64]]
            });
        }
        for &x in &X_LANES {
            let north = matches!(x, 10 | 24 | 38 | 43);
            graph.lanes.push(if north {
                [[x as f64, 3.0], [x as f64, 29.0]]
            } else {
                [[x as f64, 29.0], [x as f64, 3.0]]
            });
        }
        for pair_x in X_LANES.windows(2) {
            for pair_z in Z_LANES.windows(2) {
                graph.racks.push(Rack {
                    x_m: (pair_x[0] + pair_x[1]) as f64 / 2.0,
                    z_m: (pair_z[0] + pair_z[1]) as f64 / 2.0,
                    width_m: ((pair_x[1] - pair_x[0]) as f64 - 5.2).max(0.6),
                    depth_m: (pair_z[1] - pair_z[0]) as f64 - 3.2,
                    height_m: 2.3,
                });
            }
        }
        graph.lane_node_count = graph.points.len();
        // Station bypass pockets have separate ingress and egress nodes.
        for (x, kind, offset) in [
            (10, "pick", 1.8),
            (31, "pick", 1.8),
            (17, "drop", -1.8),
            (38, "drop", -1.8),
            (3, "charge", -1.8),
        ] {
            for z in [6, 13, 20, 27] {
                let north = matches!(x, 10 | 38);
                let upstream = ids[&(x, z + if north { -1 } else { 1 })];
                let downstream = ids[&(x, z + if north { 1 } else { -1 })];
                let node = graph.points.len();
                graph.points.push(Point {
                    x: x as f64 + offset,
                    z: z as f64,
                });
                graph.edges.push(vec![downstream]);
                graph.edges[upstream].push(node);
                graph.stations.push(Station {
                    id: graph.stations.len(),
                    x_m: x as f64 + offset,
                    z_m: z as f64,
                    kind,
                    node,
                });
            }
        }
        for (z, offset, east) in [(3, -1.8, true), (29, 1.8, false)] {
            for x in (4..=42).step_by(2) {
                let upstream = ids[&(x + if east { -1 } else { 1 }, z)];
                let downstream = ids[&(x + if east { 1 } else { -1 }, z)];
                let node = graph.points.len();
                graph.points.push(Point {
                    x: x as f64,
                    z: z as f64 + offset,
                });
                graph.edges.push(vec![downstream]);
                graph.edges[upstream].push(node);
                graph.parking.push(node);
            }
        }
        for edges in &mut graph.edges {
            edges.sort_unstable();
        }
        graph.corridor = (11..=16).map(|z| ids[&(24, z)]).collect();
        graph.corridor_entry = ids[&(24, 10)];
        graph
    }

    fn plan(&self, start: usize, goal: usize, closed: bool) -> Option<VecDeque<usize>> {
        if start == goal {
            return Some(VecDeque::new());
        }
        let mut dist = vec![f64::INFINITY; self.points.len()];
        let mut previous = vec![usize::MAX; self.points.len()];
        let mut open = BinaryHeap::new();
        dist[start] = 0.0;
        open.push(Open {
            node: start,
            cost: 0.0,
        });
        while let Some(current) = open.pop() {
            if current.cost > dist[current.node] {
                continue;
            }
            if current.node == goal {
                let mut route = VecDeque::new();
                let mut node = goal;
                while node != start {
                    route.push_front(node);
                    node = previous[node];
                }
                return Some(route);
            }
            for &next in &self.edges[current.node] {
                if closed && self.corridor.contains(&next) {
                    continue;
                }
                let cost = current.cost + self.points[current.node].distance(self.points[next]);
                if cost < dist[next] {
                    dist[next] = cost;
                    previous[next] = current.node;
                    open.push(Open { node: next, cost });
                }
            }
        }
        None
    }

    fn path_length(&self, start: usize, route: &VecDeque<usize>) -> f64 {
        let mut previous = start;
        let mut distance = 0.0;
        for &node in route {
            distance += self.points[previous].distance(self.points[node]);
            previous = node;
        }
        distance
    }
}

#[derive(Clone, Copy, Debug)]
struct Open {
    node: usize,
    cost: f64,
}
impl PartialEq for Open {
    fn eq(&self, other: &Self) -> bool {
        self.node == other.node && self.cost.to_bits() == other.cost.to_bits()
    }
}
impl Eq for Open {}
impl Ord for Open {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .total_cmp(&self.cost)
            .then_with(|| other.node.cmp(&self.node))
    }
}
impl PartialOrd for Open {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    ToPick,
    Picking,
    ToDrop,
    Dropping,
    ToCharge,
    Charging,
}
impl State {
    fn label(self) -> &'static str {
        match self {
            Self::Idle => "idle",
            Self::ToPick => "to_pick",
            Self::Picking => "picking",
            Self::ToDrop => "to_drop",
            Self::Dropping => "dropping",
            Self::ToCharge => "to_charge",
            Self::Charging => "charging",
        }
    }
}

#[derive(Clone, Debug)]
struct Motion {
    from: usize,
    to: usize,
    distance_m: f64,
    progress_m: f64,
    elapsed_s: f64,
}

#[derive(Clone, Debug)]
struct Robot {
    id: usize,
    node: usize,
    position: Point,
    yaw_rad: f64,
    speed_m_s: f64,
    battery: f64,
    state: State,
    path: VecDeque<usize>,
    motion: Option<Motion>,
    goal: Option<usize>,
    task: Option<usize>,
    loaded: bool,
    dwell_s: f64,
    waiting: bool,
    wait_s: f64,
    travelled_m: f64,
    completed: usize,
    charge_visits: usize,
    replans: usize,
}

#[derive(Clone, Debug)]
struct Task {
    id: usize,
    pick: usize,
    drop: usize,
    robot: Option<usize>,
    completed: bool,
    released_s: f64,
}

#[derive(Clone, Debug, Serialize)]
struct Event {
    time_s: f64,
    kind: &'static str,
    robot_id: Option<usize>,
    message: String,
}

#[derive(Clone, Debug, Serialize)]
struct RobotFrame {
    id: usize,
    x_m: f64,
    z_m: f64,
    yaw_rad: f64,
    speed_m_s: f64,
    battery_percent: f64,
    state: &'static str,
    operational_state: &'static str,
    task_id: Option<usize>,
    loaded: bool,
    remaining_path: Vec<[f64; 2]>,
}

#[derive(Clone, Debug, Serialize)]
struct Kpis {
    completed_tasks: usize,
    waiting: usize,
    charging: usize,
    assigned_tasks: usize,
    active: usize,
}

#[derive(Clone, Debug, Serialize)]
struct Incident {
    active: bool,
    label: &'static str,
    x_m: f64,
    z_m: f64,
    width_m: f64,
    depth_m: f64,
}

#[derive(Clone, Debug, Serialize)]
struct Frame {
    time_s: f64,
    robots: Vec<RobotFrame>,
    kpis: Kpis,
    incident: Incident,
}

#[derive(Clone, Debug, Serialize)]
struct Validation {
    passed: bool,
    model: &'static str,
    robot_count: usize,
    simulation_duration_s: f64,
    captured_frame_count: usize,
    replay_full_state_bit_exact: bool,
    full_state_sha256: String,
    replay_compared_steps: usize,
    replay_compared_words: usize,
    completed_tasks: usize,
    robots_with_task_completions: usize,
    assigned_tasks: usize,
    waiting_robot_seconds: f64,
    reservation_denials: usize,
    charge_visits: usize,
    completed_charge_visits: usize,
    initial_docked_charge_sessions: usize,
    routed_charge_visits: usize,
    maximum_observed_speed_m_s: f64,
    maximum_observed_yaw_rate_rad_s: f64,
    maximum_observed_acceleration_m_s2: f64,
    speed_limit_m_s: f64,
    yaw_rate_limit_rad_s: f64,
    acceleration_limit_m_s2: f64,
    configured_charge_rate_percent_s: f64,
    configured_empty_energy_percent_m: f64,
    configured_loaded_energy_percent_m: f64,
    configured_drive_energy_percent_s: f64,
    per_step_swept_interpolation_error_bound_m: f64,
    conservative_swept_spacing_lower_bound_m: f64,
    captured_frames_with_charging: usize,
    captured_frames_with_loaded_transport: usize,
    captured_frames_with_active_incident: usize,
    total_distance_m: f64,
    minimum_swept_robot_spacing_m: f64,
    required_robot_spacing_m: f64,
    minimum_rack_clearance_m: f64,
    collision_count: usize,
    rack_incursion_count: usize,
    incident_started_s: Option<f64>,
    incident_ended_s: f64,
    incident_route_changes: usize,
    incident_avoided_edge_entries: usize,
    incident_blocked_corridor_entries: usize,
    incident_active_occupancy_violations: usize,
    incident_passages_without_closure_control: usize,
    task_pair_count: usize,
    task_queue_remaining: usize,
    minimum_battery_percent: f64,
    state_hash_scope: &'static str,
}

#[derive(Serialize)]
struct Extent {
    width: f64,
    depth: f64,
}

#[derive(Debug, Serialize)]
struct SourceProvenance {
    format_version: u32,
    simulation_source_sha256: String,
    package_manifest_sha256: String,
    workspace_manifest_sha256: String,
    workspace_lock_sha256: String,
    engine_sources_sha256: BTreeMap<&'static str, String>,
}

impl SourceProvenance {
    fn compiled() -> Self {
        // This explicit input set covers the navigation/RNG behavior used here,
        // rather than claiming to describe the entire compiler dependency graph.
        let engine_sources: [(&str, &[u8]); 8] = [
            (
                "crates/rne_core/Cargo.toml",
                include_bytes!("../../crates/rne_core/Cargo.toml"),
            ),
            (
                "crates/rne_core/src/rng.rs",
                include_bytes!("../../crates/rne_core/src/rng.rs"),
            ),
            (
                "crates/rne_core/src/time.rs",
                include_bytes!("../../crates/rne_core/src/time.rs"),
            ),
            (
                "crates/rne_nav/Cargo.toml",
                include_bytes!("../../crates/rne_nav/Cargo.toml"),
            ),
            (
                "crates/rne_nav/src/coordination.rs",
                include_bytes!("../../crates/rne_nav/src/coordination.rs"),
            ),
            (
                "crates/rne_nav/src/grid.rs",
                include_bytes!("../../crates/rne_nav/src/grid.rs"),
            ),
            (
                "crates/rne_world/Cargo.toml",
                include_bytes!("../../crates/rne_world/Cargo.toml"),
            ),
            (
                "crates/rne_world/src/resources.rs",
                include_bytes!("../../crates/rne_world/src/resources.rs"),
            ),
        ];
        Self {
            format_version: 1,
            simulation_source_sha256: format!("{:x}", Sha256::digest(include_bytes!("main.rs"))),
            package_manifest_sha256: format!("{:x}", Sha256::digest(include_bytes!("Cargo.toml"))),
            workspace_manifest_sha256: format!(
                "{:x}",
                Sha256::digest(include_bytes!("../../Cargo.toml"))
            ),
            workspace_lock_sha256: format!(
                "{:x}",
                Sha256::digest(include_bytes!("../../Cargo.lock"))
            ),
            engine_sources_sha256: engine_sources
                .into_iter()
                .map(|(path, bytes)| (path, format!("{:x}", Sha256::digest(bytes))))
                .collect(),
        }
    }
}

#[derive(Serialize)]
struct Trace {
    schema_version: u32,
    source_provenance: SourceProvenance,
    seed: u64,
    simulation_dt_s: f64,
    capture_dt_s: f64,
    playback_speed: f64,
    robot_count: usize,
    extent_m: Extent,
    robot_radius_m: f64,
    model: &'static str,
    racks: Vec<Rack>,
    stations: Vec<Station>,
    lanes: Vec<[[f64; 2]; 2]>,
    frames: Vec<Frame>,
    events: Vec<Event>,
    validation: Validation,
}

#[derive(Debug)]
struct Simulation {
    graph: Graph,
    random: WorldRandom,
    robots: Vec<Robot>,
    tasks: Vec<Task>,
    traffic: TrafficCoordinator,
    leases: BTreeMap<GridCoord, (usize, f64)>,
    charge_booking: Vec<Option<usize>>,
    pick_booking: Vec<Option<usize>>,
    events: Vec<Event>,
    frames: Vec<Frame>,
    words: Vec<Vec<u64>>,
    step: usize,
    completed: usize,
    denials: usize,
    min_spacing: f64,
    min_rack_clearance: f64,
    collisions: usize,
    incursions: usize,
    incident_enabled: bool,
    incident_draining: bool,
    incident_active: bool,
    incident_started_s: Option<f64>,
    incident_changes: usize,
    incident_avoided_entries: usize,
    control_corridor_passages: usize,
    incident_blocked_entries: usize,
    incident_violations: usize,
    completed_charges: usize,
    initial_charge_sessions: usize,
    routed_charge_visits: usize,
    max_speed: f64,
    max_yaw_rate: f64,
    max_acceleration: f64,
    minimum_battery: f64,
}

impl Simulation {
    fn new(seed: u64, incident_enabled: bool) -> Self {
        let graph = Graph::new();
        let mut random = WorldRandom::new(seed);
        let mut robots = Vec::new();
        for id in 0..ROBOTS {
            let node = if matches!(id, 0 | 9 | 18 | 27) {
                graph.stations[16 + id / 9].node
            } else {
                graph.parking[id]
            };
            let battery = if id % 9 == 0 {
                23.0 + random.uniform_f64(0.0, 1.0)
            } else {
                random.uniform_f64(47.0, 93.0)
            };
            robots.push(Robot {
                id,
                node,
                position: graph.points[node],
                yaw_rad: 0.0,
                speed_m_s: 0.0,
                battery,
                state: State::Idle,
                path: VecDeque::new(),
                motion: None,
                goal: None,
                task: None,
                loaded: false,
                dwell_s: 0.0,
                waiting: false,
                wait_s: 0.0,
                travelled_m: 0.0,
                completed: 0,
                charge_visits: 0,
                replans: 0,
            });
        }
        let station_count = graph.stations.len();
        let mut sim = Self {
            graph,
            random,
            robots,
            tasks: Vec::new(),
            traffic: TrafficCoordinator::new(TrafficConfig {
                claim_horizon_s: LEASE_S,
            })
            .expect("valid lease config"),
            leases: BTreeMap::new(),
            charge_booking: vec![None; station_count],
            pick_booking: vec![None; station_count],
            events: Vec::new(),
            frames: Vec::new(),
            words: Vec::new(),
            step: 0,
            completed: 0,
            denials: 0,
            min_spacing: f64::INFINITY,
            min_rack_clearance: f64::INFINITY,
            collisions: 0,
            incursions: 0,
            incident_enabled,
            incident_draining: false,
            incident_active: false,
            incident_started_s: None,
            incident_changes: 0,
            incident_avoided_entries: 0,
            control_corridor_passages: 0,
            incident_blocked_entries: 0,
            incident_violations: 0,
            completed_charges: 0,
            initial_charge_sessions: 0,
            routed_charge_visits: 0,
            max_speed: 0.0,
            max_yaw_rate: 0.0,
            max_acceleration: 0.0,
            minimum_battery: 100.0,
        };
        for id in 0..ROBOTS {
            sim.renew(id, 0.0);
        }
        sim
    }

    fn event(&mut self, now: f64, kind: &'static str, robot: Option<usize>, message: String) {
        self.events.push(Event {
            time_s: now,
            kind,
            robot_id: robot,
            message,
        });
    }

    fn coords(&self, id: usize) -> Vec<GridCoord> {
        let robot = &self.robots[id];
        let mut coords = vec![GridCoord {
            x: robot.node as isize,
            y: 0,
        }];
        if let Some(motion) = &robot.motion {
            coords.push(GridCoord {
                x: motion.to as isize,
                y: 0,
            });
            let a = motion.from.min(motion.to);
            let b = motion.from.max(motion.to);
            coords.push(GridCoord {
                x: (a * self.graph.points.len() + b) as isize,
                y: 1,
            });
        }
        let (a, b) = if let Some(motion) = &robot.motion {
            (self.graph.points[motion.from], self.graph.points[motion.to])
        } else {
            (robot.position, robot.position)
        };
        for &junction in &self.graph.junctions {
            if (robot.motion.is_some() || robot.node < self.graph.lane_node_count)
                && point_segment_distance(self.graph.points[junction], a, b) <= JUNCTION_ZONE_M
            {
                coords.push(GridCoord {
                    x: junction as isize,
                    y: 2,
                });
            }
        }
        coords
    }

    fn renew(&mut self, id: usize, now: f64) {
        self.traffic.release(id as u32);
        self.leases.retain(|_, (owner, _)| *owner != id);
        let coords = self.coords(id);
        assert!(
            self.traffic
                .request(id as u32, &coords, now)
                .expect("finite time"),
            "occupied lease was stolen"
        );
        for coord in coords {
            self.leases.insert(coord, (id, now + LEASE_S));
        }
    }

    fn route(&mut self, id: usize, goal: usize) -> bool {
        let start = self.robots[id]
            .motion
            .as_ref()
            .map_or(self.robots[id].node, |motion| motion.to);
        let in_corridor = self.graph.corridor.contains(&start);
        let closed = (self.incident_active || self.incident_draining) && !in_corridor;
        if let Some(path) = self.graph.plan(start, goal, closed) {
            self.robots[id].goal = Some(goal);
            self.robots[id].path = path;
            true
        } else {
            false
        }
    }

    fn generate_tasks(&mut self, now: f64) {
        // Each task is sampled from the explicit world stream, never a cyclic path.
        if self.step.is_multiple_of(15)
            && self
                .tasks
                .iter()
                .filter(|task| !task.completed && task.robot.is_none())
                .count()
                < 60
        {
            let picks: Vec<_> = self
                .graph
                .stations
                .iter()
                .filter(|s| s.kind == "pick")
                .map(|s| s.id)
                .collect();
            let drops: Vec<_> = self
                .graph
                .stations
                .iter()
                .filter(|s| s.kind == "drop")
                .map(|s| s.id)
                .collect();
            let pick = picks[self.random.uniform_usize(picks.len())];
            let drop = drops[self.random.uniform_usize(drops.len())];
            self.tasks.push(Task {
                id: self.tasks.len(),
                pick,
                drop,
                robot: None,
                completed: false,
                released_s: now,
            });
        }
        // Seed an initial backlog through the same stream.
        if self.step == 0 {
            for _ in 0..55 {
                let pick = self.random.uniform_usize(8);
                let drop = 8 + self.random.uniform_usize(8);
                self.tasks.push(Task {
                    id: self.tasks.len(),
                    pick,
                    drop,
                    robot: None,
                    completed: false,
                    released_s: now,
                });
            }
        }
    }

    fn dispatch(&mut self, now: f64) {
        // Robots with low energy receive an exclusive available charge pocket.
        for id in 0..ROBOTS {
            if self.robots[id].state != State::Idle
                || self.robots[id].battery > CHARGE_TRIGGER_PERCENT
            {
                continue;
            }
            let mut best: Option<(f64, usize)> = None;
            for station in &self.graph.stations {
                if station.kind != "charge" || self.charge_booking[station.id].is_some() {
                    continue;
                }
                if let Some(path) = self.graph.plan(
                    self.robots[id].node,
                    station.node,
                    self.incident_active || self.incident_draining,
                ) {
                    let distance = self.graph.path_length(self.robots[id].node, &path);
                    if best.is_none_or(|(d, _)| distance < d) {
                        best = Some((distance, station.id));
                    }
                }
            }
            if let Some((_, station_id)) = best {
                self.charge_booking[station_id] = Some(id);
                self.robots[id].state = State::ToCharge;
                let goal = self.graph.stations[station_id].node;
                assert!(self.route(id, goal));
                self.event(
                    now,
                    "charge_dispatch",
                    Some(id),
                    format!("Low battery: route to charger C{}", station_id - 16),
                );
            }
        }
        // Minimize actual directed route distance over available tasks and robots.
        // Stable task-id / robot-id tie order supplies deterministic fairness.
        loop {
            let mut best: Option<(f64, usize, usize)> = None;
            for task in &self.tasks {
                if task.completed || task.robot.is_some() || self.pick_booking[task.pick].is_some()
                {
                    continue;
                }
                for robot in &self.robots {
                    if robot.state != State::Idle
                        || robot.battery <= CHARGE_TRIGGER_PERCENT
                        || robot.motion.is_some()
                    {
                        continue;
                    }
                    let goal = self.graph.stations[task.pick].node;
                    if let Some(path) = self.graph.plan(
                        robot.node,
                        goal,
                        self.incident_active || self.incident_draining,
                    ) {
                        let distance = self.graph.path_length(robot.node, &path);
                        let age = now - task.released_s;
                        let score =
                            distance - 0.1 * age + FAIRNESS_DISTANCE_M * robot.completed as f64;
                        if best.is_none_or(|(previous, _, _)| score < previous) {
                            best = Some((score, task.id, robot.id));
                        }
                    }
                }
            }
            let Some((_, task_id, id)) = best else {
                break;
            };
            let pick = self.tasks[task_id].pick;
            self.tasks[task_id].robot = Some(id);
            self.pick_booking[pick] = Some(id);
            self.robots[id].task = Some(task_id);
            self.robots[id].state = State::ToPick;
            assert!(self.route(id, self.graph.stations[pick].node));
            self.event(
                now,
                "task_dispatch",
                Some(id),
                format!(
                    "Task {}: P{} -> D{}",
                    task_id,
                    pick,
                    self.tasks[task_id].drop - 8
                ),
            );
        }
    }

    fn update_incident(&mut self, now: f64) {
        if !self.incident_enabled {
            return;
        }
        if self.step == (INCIDENT_START_S / DT) as usize {
            self.incident_draining = true;
            self.event(
                now,
                "aisle_closing",
                None,
                "Aisle X=24 m, Z=10..17 m drains; new routes detour".to_owned(),
            );
            for id in 0..ROBOTS {
                let start = self.robots[id]
                    .motion
                    .as_ref()
                    .map_or(self.robots[id].node, |m| m.to);
                if self.graph.corridor.contains(&start) {
                    continue;
                }
                if let Some(goal) = self.robots[id].goal {
                    let old = self.robots[id].path.clone();
                    assert!(self.route(id, goal));
                    if self.robots[id].path != old {
                        self.incident_changes += 1;
                        self.robots[id].replans += 1;
                        self.event(
                            now,
                            "route_replan",
                            Some(id),
                            "Reserved aisle closed: computed a different directed route".to_owned(),
                        );
                    }
                }
            }
        }
        if self.incident_draining {
            let occupied = self.robots.iter().any(|r| {
                self.graph.corridor.contains(&r.node)
                    || r.motion
                        .as_ref()
                        .is_some_and(|m| self.graph.corridor.contains(&m.to))
            });
            if !occupied {
                self.incident_draining = false;
                self.incident_active = true;
                self.incident_started_s = Some(now);
                self.event(
                    now,
                    "aisle_closed",
                    None,
                    "Barrier active after occupied corridor cleared".to_owned(),
                );
            }
        }
        if self.step == (INCIDENT_END_S / DT) as usize {
            self.incident_draining = false;
            self.incident_active = false;
            self.event(
                now,
                "aisle_reopened",
                None,
                "Temporary obstruction removed; short routes restored".to_owned(),
            );
            for id in 0..ROBOTS {
                if let Some(goal) = self.robots[id].goal {
                    assert!(self.route(id, goal));
                }
            }
        }
        if self.incident_active {
            for robot in &self.robots {
                if self.graph.corridor.contains(&robot.node)
                    || robot
                        .motion
                        .as_ref()
                        .is_some_and(|motion| self.graph.corridor.contains(&motion.to))
                {
                    self.incident_violations += 1;
                }
            }
        }
    }

    fn advance_dwell(&mut self, id: usize, now: f64) {
        let state = self.robots[id].state;
        if !matches!(state, State::Picking | State::Dropping | State::Charging) {
            return;
        }
        self.robots[id].dwell_s -= DT;
        if state == State::Charging {
            self.robots[id].battery =
                (self.robots[id].battery + CHARGE_RATE_PERCENT_S * DT).min(100.0);
            if self.robots[id].battery < 92.0 {
                return;
            }
        } else if self.robots[id].dwell_s > 0.0 {
            return;
        }
        match state {
            State::Picking => {
                let task_id = self.robots[id].task.expect("picking assigned task");
                self.robots[id].loaded = true;
                self.robots[id].state = State::ToDrop;
                let goal = self.graph.stations[self.tasks[task_id].drop].node;
                assert!(self.route(id, goal));
                self.event(now, "picked", Some(id), format!("Loaded task {}", task_id));
            }
            State::Dropping => {
                let task_id = self.robots[id].task.take().expect("dropping assigned task");
                self.tasks[task_id].completed = true;
                self.robots[id].completed += 1;
                self.completed += 1;
                self.robots[id].loaded = false;
                self.robots[id].state = State::Idle;
                self.robots[id].goal = None;
                self.event(
                    now,
                    "delivered",
                    Some(id),
                    format!("Completed task {}", task_id),
                );
                self.exit_station(id);
            }
            State::Charging => {
                self.completed_charges += 1;
                self.robots[id].state = State::Idle;
                self.robots[id].goal = None;
                self.event(
                    now,
                    "charged",
                    Some(id),
                    "Charge complete: 92% energy, returning to dispatch".to_owned(),
                );
                self.exit_station(id);
            }
            _ => unreachable!(),
        }
    }

    fn exit_station(&mut self, id: usize) {
        let node = self.robots[id].node;
        if self.graph.stations.iter().any(|s| s.node == node) {
            // An idle robot exits its pocket before a new assignment/charger grant.
            let next = self.graph.edges[node][0];
            self.robots[id].path = VecDeque::from([next]);
        }
    }

    fn at_goal(&mut self, id: usize, now: f64) {
        if self.robots[id].motion.is_some() || !self.robots[id].path.is_empty() {
            return;
        }
        match self.robots[id].state {
            State::ToPick => {
                self.robots[id].state = State::Picking;
                self.robots[id].dwell_s = 1.2;
            }
            State::ToDrop => {
                self.robots[id].state = State::Dropping;
                self.robots[id].dwell_s = 1.0;
            }
            State::ToCharge => {
                self.robots[id].state = State::Charging;
                self.robots[id].charge_visits += 1;
                if self.robots[id].travelled_m == 0.0 {
                    self.initial_charge_sessions += 1;
                } else {
                    self.routed_charge_visits += 1;
                }
                self.robots[id].dwell_s = 0.0;
                self.event(
                    now,
                    "charge_started",
                    Some(id),
                    "Docked at reserved charger".to_owned(),
                );
            }
            _ => {}
        }
    }

    fn can_start(&self, id: usize, next: usize) -> bool {
        let from = self.robots[id].node;
        let a = self.graph.points[from];
        let b = self.graph.points[next];
        for other in &self.robots {
            if other.id == id {
                continue;
            }
            // Occupied cells and whole committed edges cannot be preempted.
            // Geometric edge guards also protect pocket/crossing swept footprints.
            let (c, d) = if let Some(motion) = &other.motion {
                (self.graph.points[motion.from], self.graph.points[motion.to])
            } else {
                (other.position, other.position)
            };
            if segment_distance(a, b, c, d) < SAFETY_M {
                return false;
            }
        }
        true
    }

    fn move_robot(&mut self, id: usize, now: f64) {
        self.robots[id].waiting = false;
        if self.robots[id].motion.is_none() {
            let Some(next) = self.robots[id].path.front().copied() else {
                self.robots[id].speed_m_s = 0.0;
                return;
            };
            let from = self.robots[id].node;
            if (self.incident_draining || self.incident_active)
                && from == self.graph.corridor_entry
                && self.graph.corridor.contains(&next)
            {
                self.incident_blocked_entries += 1;
                self.robots[id].waiting = true;
                if let Some(goal) = self.robots[id].goal {
                    assert!(self.route(id, goal));
                }
                return;
            }
            let a = self.graph.points[from];
            let b = self.graph.points[next];
            let desired = (b.z - a.z).atan2(b.x - a.x);
            let error = wrap_angle(desired - self.robots[id].yaw_rad);
            if error.abs() > 0.02 {
                self.robots[id].yaw_rad = wrap_angle(
                    self.robots[id].yaw_rad
                        + error.clamp(-YAW_RATE_RAD_S * DT, YAW_RATE_RAD_S * DT),
                );
                self.robots[id].speed_m_s = 0.0;
                return;
            }
            self.robots[id].yaw_rad = desired;
            let motion = Motion {
                from,
                to: next,
                distance_m: a.distance(b),
                progress_m: 0.0,
                elapsed_s: 0.0,
            };
            if !self.can_start(id, next) {
                self.robots[id].waiting = true;
                self.robots[id].wait_s += DT;
                self.denials += 1;
                self.robots[id].speed_m_s = 0.0;
                return;
            }
            self.robots[id].motion = Some(motion);
            let coords = self.coords(id);
            if !self
                .traffic
                .request(id as u32, &coords, now)
                .expect("finite time")
            {
                self.robots[id].motion = None;
                self.robots[id].waiting = true;
                self.robots[id].wait_s += DT;
                self.denials += 1;
                self.robots[id].speed_m_s = 0.0;
                return;
            }
            for coord in coords {
                self.leases.insert(coord, (id, now + LEASE_S));
            }
            self.robots[id].path.pop_front();
            if self.graph.corridor.contains(&next)
                && !self.incident_enabled
                && (INCIDENT_START_S..INCIDENT_END_S).contains(&now)
            {
                self.incident_avoided_entries += 1;
                if from == self.graph.corridor_entry {
                    self.control_corridor_passages += 1;
                }
            }
        }
        let robot = &mut self.robots[id];
        let motion = robot.motion.as_mut().expect("started motion");
        motion.elapsed_s += DT;
        let (progress, speed) = motion_profile(motion.distance_m, motion.elapsed_s);
        let distance = progress - motion.progress_m;
        robot.speed_m_s = speed;
        motion.progress_m = progress;
        robot.position = self.graph.points[motion.from].lerp(
            self.graph.points[motion.to],
            motion.progress_m / motion.distance_m,
        );
        robot.travelled_m += distance;
        robot.battery = (robot.battery
            - distance
                * if robot.loaded {
                    LOADED_ENERGY_PERCENT_M
                } else {
                    EMPTY_ENERGY_PERCENT_M
                }
            - DT * DRIVE_ENERGY_PERCENT_S)
            .max(0.0);
        if motion.progress_m >= motion.distance_m - 1e-12 {
            let from = motion.from;
            robot.node = motion.to;
            robot.position = self.graph.points[motion.to];
            robot.motion = None;
            robot.speed_m_s = 0.0;
            for station in &self.graph.stations {
                if station.node == from {
                    if self.pick_booking[station.id] == Some(id) {
                        self.pick_booking[station.id] = None;
                    }
                    if self.charge_booking[station.id] == Some(id) {
                        self.charge_booking[station.id] = None;
                    }
                }
            }
            self.renew(id, now);
        }
        self.minimum_battery = self.minimum_battery.min(self.robots[id].battery);
    }

    fn validate_geometry(&mut self, before: &[Point]) {
        for a in 0..ROBOTS {
            for b in (a + 1)..ROBOTS {
                let distance = synchronous_distance(
                    before[a],
                    self.robots[a].position,
                    before[b],
                    self.robots[b].position,
                );
                self.min_spacing = self.min_spacing.min(distance);
                if distance - INTERPOLATION_ERROR_M < 2.0 * RADIUS_M - 1e-9 {
                    self.collisions += 1;
                }
            }
            for rack in &self.graph.racks {
                let clearance =
                    swept_box_distance(before[a], self.robots[a].position, rack) - RADIUS_M;
                self.min_rack_clearance = self.min_rack_clearance.min(clearance);
                if clearance < -1e-9 {
                    self.incursions += 1;
                }
            }
        }
    }

    fn capture(&mut self, now: f64) {
        let robots = self
            .robots
            .iter()
            .map(|robot| {
                let mut path = Vec::new();
                if let Some(motion) = &robot.motion {
                    path.push(self.graph.points[motion.to].array());
                }
                path.extend(
                    robot
                        .path
                        .iter()
                        .map(|&node| self.graph.points[node].array()),
                );
                RobotFrame {
                    id: robot.id,
                    x_m: robot.position.x,
                    z_m: robot.position.z,
                    yaw_rad: robot.yaw_rad,
                    speed_m_s: robot.speed_m_s,
                    battery_percent: robot.battery,
                    state: if robot.waiting {
                        "waiting"
                    } else {
                        robot.state.label()
                    },
                    operational_state: robot.state.label(),
                    task_id: robot.task,
                    loaded: robot.loaded,
                    remaining_path: path,
                }
            })
            .collect();
        let kpis = Kpis {
            completed_tasks: self.completed,
            waiting: self.robots.iter().filter(|r| r.waiting).count(),
            charging: self
                .robots
                .iter()
                .filter(|r| r.state == State::Charging)
                .count(),
            assigned_tasks: self.tasks.iter().filter(|t| t.robot.is_some()).count(),
            active: self
                .robots
                .iter()
                .filter(|r| r.state != State::Idle)
                .count(),
        };
        let incident = Incident {
            active: self.incident_active,
            label: if self.incident_draining {
                "AISLE DRAINING / DETOUR"
            } else if self.incident_active {
                "AISLE BLOCKED / DETOUR"
            } else {
                "ALL AISLES OPEN"
            },
            x_m: 24.0,
            z_m: 14.0,
            width_m: 2.0,
            depth_m: 3.0,
        };
        self.frames.push(Frame {
            time_s: now,
            robots,
            kpis,
            incident,
        });
    }

    fn full_state_words(&self) -> Vec<u64> {
        let mut words = vec![
            self.step as u64,
            self.random.snapshot().seed,
            self.random.snapshot().main_rng_state,
            self.completed as u64,
            self.denials as u64,
            self.incident_enabled as u64,
            self.incident_draining as u64,
            self.incident_active as u64,
            self.incident_started_s.map_or(u64::MAX, f64::to_bits),
            self.incident_changes as u64,
            self.incident_avoided_entries as u64,
            self.control_corridor_passages as u64,
            self.incident_blocked_entries as u64,
            self.incident_violations as u64,
            self.completed_charges as u64,
            self.initial_charge_sessions as u64,
            self.routed_charge_visits as u64,
            self.max_speed.to_bits(),
            self.max_yaw_rate.to_bits(),
            self.max_acceleration.to_bits(),
            self.min_spacing.to_bits(),
            self.min_rack_clearance.to_bits(),
            self.minimum_battery.to_bits(),
            self.collisions as u64,
            self.incursions as u64,
        ];
        for robot in &self.robots {
            words.extend([
                robot.id as u64,
                robot.node as u64,
                robot.position.x.to_bits(),
                robot.position.z.to_bits(),
                robot.yaw_rad.to_bits(),
                robot.speed_m_s.to_bits(),
                robot.battery.to_bits(),
                robot.state as u64,
                robot.goal.map_or(u64::MAX, |v| v as u64),
                robot.task.map_or(u64::MAX, |v| v as u64),
                robot.loaded as u64,
                robot.dwell_s.to_bits(),
                robot.waiting as u64,
                robot.wait_s.to_bits(),
                robot.travelled_m.to_bits(),
                robot.completed as u64,
                robot.charge_visits as u64,
                robot.replans as u64,
                robot.path.len() as u64,
            ]);
            words.extend(robot.path.iter().map(|&node| node as u64));
            if let Some(m) = &robot.motion {
                words.extend([
                    1,
                    m.from as u64,
                    m.to as u64,
                    m.distance_m.to_bits(),
                    m.progress_m.to_bits(),
                    m.elapsed_s.to_bits(),
                ]);
            } else {
                words.push(0);
            }
        }
        words.push(self.tasks.len() as u64);
        for task in &self.tasks {
            words.extend([
                task.id as u64,
                task.pick as u64,
                task.drop as u64,
                task.robot.map_or(u64::MAX, |v| v as u64),
                task.completed as u64,
                task.released_s.to_bits(),
            ]);
        }
        words.push(self.leases.len() as u64);
        for (coord, (id, expiry)) in &self.leases {
            words.extend([
                coord.x as u64,
                coord.y as u64,
                *id as u64,
                expiry.to_bits(),
                self.traffic.owner(*coord).map_or(u64::MAX, u64::from),
            ]);
        }
        for booking in self.charge_booking.iter().chain(&self.pick_booking) {
            words.push(booking.map_or(u64::MAX, |v| v as u64));
        }
        // Immutable world and controller configuration belong to every snapshot.
        words.extend([
            DT.to_bits(),
            MAX_SPEED_M_S.to_bits(),
            ACCEL_M_S2.to_bits(),
            SAFETY_M.to_bits(),
            RADIUS_M.to_bits(),
            LEASE_S.to_bits(),
            CHARGE_TRIGGER_PERCENT.to_bits(),
            CHARGE_RATE_PERCENT_S.to_bits(),
            JUNCTION_ZONE_M.to_bits(),
            YAW_RATE_RAD_S.to_bits(),
            CAPTURE_START_S.to_bits(),
            CAPTURE_END_S.to_bits(),
            INCIDENT_START_S.to_bits(),
            INCIDENT_END_S.to_bits(),
            INTERPOLATION_ERROR_M.to_bits(),
        ]);
        words.push(self.graph.corridor_entry as u64);
        words.push(self.graph.lane_node_count as u64);
        words.extend([
            FAIRNESS_DISTANCE_M.to_bits(),
            EMPTY_ENERGY_PERCENT_M.to_bits(),
            LOADED_ENERGY_PERCENT_M.to_bits(),
            DRIVE_ENERGY_PERCENT_S.to_bits(),
        ]);
        for list in [
            &self.graph.corridor,
            &self.graph.junctions,
            &self.graph.parking,
        ] {
            words.push(list.len() as u64);
            words.extend(list.iter().map(|&node| node as u64));
        }
        for (id, point) in self.graph.points.iter().enumerate() {
            words.extend([
                point.x.to_bits(),
                point.z.to_bits(),
                self.graph.edges[id].len() as u64,
            ]);
            words.extend(self.graph.edges[id].iter().map(|&v| v as u64));
        }
        for rack in &self.graph.racks {
            words.extend([
                rack.x_m.to_bits(),
                rack.z_m.to_bits(),
                rack.width_m.to_bits(),
                rack.depth_m.to_bits(),
                rack.height_m.to_bits(),
            ]);
        }
        for station in &self.graph.stations {
            words.extend([
                station.id as u64,
                station.node as u64,
                station.x_m.to_bits(),
                station.z_m.to_bits(),
                station.kind.len() as u64,
            ]);
            words.extend(station.kind.as_bytes().iter().map(|v| u64::from(*v)));
        }
        words
    }

    fn assert_lease_invariants(&self, universe: &[GridCoord], now: f64) {
        for &coord in universe {
            let expected = self.leases.get(&coord).map(|(owner, _)| *owner as u32);
            assert_eq!(self.traffic.owner(coord), expected, "lease mirror differs");
        }
        for (coord, (_, expiry_s)) in &self.leases {
            assert!(universe.binary_search(coord).is_ok(), "untracked claim");
            assert!(*expiry_s > now, "physical occupancy lease expired");
        }
    }

    fn run(mut self, retain_words: bool) -> Self {
        let mut universe = std::collections::BTreeSet::new();
        for (from, edges) in self.graph.edges.iter().enumerate() {
            universe.insert(GridCoord {
                x: from as isize,
                y: 0,
            });
            for &to in edges {
                universe.insert(GridCoord {
                    x: (from.min(to) * self.graph.points.len() + from.max(to)) as isize,
                    y: 1,
                });
            }
        }
        for &junction in &self.graph.junctions {
            universe.insert(GridCoord {
                x: junction as isize,
                y: 2,
            });
        }
        let universe: Vec<_> = universe.into_iter().collect();
        // Initial conditions are physically validated before any warmup step.
        self.validate_geometry(&self.robots.iter().map(|r| r.position).collect::<Vec<_>>());
        self.minimum_battery = self.robots.iter().map(|r| r.battery).fold(100.0, f64::min);
        let mut clock = SimClock::new(SimDuration::from_ticks(50_000_000));
        loop {
            let step = (clock.sim_time().ticks() / clock.fixed_delta().ticks()) as usize;
            self.step = step;
            // Preserve the established floating-point timestamp convention.
            // Exact fixed advances leave no residual; the hashed step word
            // therefore also represents the clock's simulation time.
            let now = step as f64 * DT;
            self.generate_tasks(now);
            self.update_incident(now);
            for id in 0..ROBOTS {
                self.renew(id, now);
                self.advance_dwell(id, now);
                self.at_goal(id, now);
            }
            if step.is_multiple_of(10) {
                self.dispatch(now);
            }
            for id in 0..ROBOTS {
                if self.robots[id].state == State::Idle
                    && self.robots[id].motion.is_none()
                    && self.robots[id].path.is_empty()
                    && self.robots[id].node != self.graph.parking[id]
                {
                    assert!(self.route(id, self.graph.parking[id]));
                }
            }
            self.assert_lease_invariants(&universe, now);
            if retain_words {
                self.words.push(self.full_state_words());
            }
            if (CAPTURE_START_S..=CAPTURE_END_S).contains(&now) && step.is_multiple_of(5) {
                self.capture(now);
            }
            if step == STEPS {
                break;
            }
            let before: Vec<_> = self.robots.iter().map(|r| r.position).collect();
            let speed_before: Vec<_> = self.robots.iter().map(|r| r.speed_m_s).collect();
            let yaw_before: Vec<_> = self.robots.iter().map(|r| r.yaw_rad).collect();
            // Rotate arbitration order without changing the order of integration sums.
            for offset in 0..ROBOTS {
                let id = (offset + step / 10) % ROBOTS;
                self.move_robot(id, now);
            }
            for id in 0..ROBOTS {
                self.max_speed = self
                    .max_speed
                    .max(self.robots[id].speed_m_s)
                    .max(before[id].distance(self.robots[id].position) / DT);
                self.max_yaw_rate = self
                    .max_yaw_rate
                    .max(wrap_angle(self.robots[id].yaw_rad - yaw_before[id]).abs() / DT);
                self.max_acceleration = self
                    .max_acceleration
                    .max((self.robots[id].speed_m_s - speed_before[id]).abs() / DT);
                self.minimum_battery = self.minimum_battery.min(self.robots[id].battery);
            }
            self.validate_geometry(&before);
            self.assert_lease_invariants(&universe, now);
            assert_eq!(clock.advance(clock.fixed_delta()), 1);
        }
        self
    }

    fn validation(&self, replay: &Self, control: &Self) -> Validation {
        let mut hash = Sha256::new();
        let mut count = 0;
        for words in &self.words {
            count += words.len();
            for &word in words {
                hash.update(word.to_le_bytes());
            }
        }
        let pairs: std::collections::BTreeSet<_> =
            self.tasks.iter().map(|t| (t.pick, t.drop)).collect();
        Validation {
            passed: false,
            model: "bounded-speed kinematic logistics; no wheel/contact dynamics or hardware qualification",
            robot_count: ROBOTS,
            simulation_duration_s: STEPS as f64 * DT,
            captured_frame_count: self.frames.len(),
            replay_full_state_bit_exact: self.words == replay.words,
            full_state_sha256: format!("{:x}", hash.finalize()),
            replay_compared_steps: self.words.len(),
            replay_compared_words: count,
            completed_tasks: self.completed,
            robots_with_task_completions: self.robots.iter().filter(|r| r.completed > 0).count(),
            assigned_tasks: self.tasks.iter().filter(|t| t.robot.is_some()).count(),
            waiting_robot_seconds: self.robots.iter().map(|r| r.wait_s).sum(),
            reservation_denials: self.denials,
            charge_visits: self.robots.iter().map(|r| r.charge_visits).sum(),
            completed_charge_visits: self.completed_charges,
            initial_docked_charge_sessions: self.initial_charge_sessions,
            routed_charge_visits: self.routed_charge_visits,
            maximum_observed_speed_m_s: self.max_speed,
            maximum_observed_yaw_rate_rad_s: self.max_yaw_rate,
            maximum_observed_acceleration_m_s2: self.max_acceleration,
            speed_limit_m_s: MAX_SPEED_M_S,
            yaw_rate_limit_rad_s: YAW_RATE_RAD_S,
            acceleration_limit_m_s2: ACCEL_M_S2,
            configured_charge_rate_percent_s: CHARGE_RATE_PERCENT_S,
            configured_empty_energy_percent_m: EMPTY_ENERGY_PERCENT_M,
            configured_loaded_energy_percent_m: LOADED_ENERGY_PERCENT_M,
            configured_drive_energy_percent_s: DRIVE_ENERGY_PERCENT_S,
            per_step_swept_interpolation_error_bound_m: INTERPOLATION_ERROR_M,
            conservative_swept_spacing_lower_bound_m: self.min_spacing - INTERPOLATION_ERROR_M,
            captured_frames_with_charging: self
                .frames
                .iter()
                .filter(|f| f.robots.iter().any(|r| r.operational_state == "charging"))
                .count(),
            captured_frames_with_loaded_transport: self
                .frames
                .iter()
                .filter(|f| {
                    f.robots
                        .iter()
                        .any(|r| r.loaded && r.operational_state == "to_drop")
                })
                .count(),
            captured_frames_with_active_incident: self
                .frames
                .iter()
                .filter(|f| f.incident.active)
                .count(),
            total_distance_m: self.robots.iter().map(|r| r.travelled_m).sum(),
            minimum_swept_robot_spacing_m: self.min_spacing,
            required_robot_spacing_m: 2.0 * RADIUS_M,
            minimum_rack_clearance_m: self.min_rack_clearance,
            collision_count: self.collisions,
            rack_incursion_count: self.incursions,
            incident_started_s: self.incident_started_s,
            incident_ended_s: INCIDENT_END_S,
            incident_route_changes: self.incident_changes,
            incident_avoided_edge_entries: control.incident_avoided_entries,
            incident_blocked_corridor_entries: self.incident_blocked_entries,
            incident_active_occupancy_violations: self.incident_violations,
            incident_passages_without_closure_control: control.control_corridor_passages,
            task_pair_count: pairs.len(),
            task_queue_remaining: self
                .tasks
                .iter()
                .filter(|t| !t.completed && t.robot.is_none())
                .count(),
            minimum_battery_percent: self.minimum_battery,
            state_hash_scope: "all per-tick robot poses, yaw, speeds, motion progress, controller/dwell/wait state, batteries, loads, goals, full routes, tasks, bookings, coordinator owners and lease expiry, world RNG, graph/racks/stations and controller constants; same-runtime bit identity",
        }
    }
}

fn motion_profile(distance_m: f64, elapsed_s: f64) -> (f64, f64) {
    let acceleration_time = (MAX_SPEED_M_S / ACCEL_M_S2).min((distance_m / ACCEL_M_S2).sqrt());
    let peak_speed = ACCEL_M_S2 * acceleration_time;
    let cruise_time =
        ((distance_m - ACCEL_M_S2 * acceleration_time * acceleration_time) / peak_speed).max(0.0);
    let duration = 2.0 * acceleration_time + cruise_time;
    let elapsed = elapsed_s.min(duration);
    if elapsed >= duration {
        return (distance_m, 0.0);
    }
    if elapsed < acceleration_time {
        return (0.5 * ACCEL_M_S2 * elapsed * elapsed, ACCEL_M_S2 * elapsed);
    }
    if elapsed < acceleration_time + cruise_time {
        return (
            0.5 * ACCEL_M_S2 * acceleration_time * acceleration_time
                + peak_speed * (elapsed - acceleration_time),
            peak_speed,
        );
    }
    let remaining = duration - elapsed;
    (
        distance_m - 0.5 * ACCEL_M_S2 * remaining * remaining,
        ACCEL_M_S2 * remaining,
    )
}

fn wrap_angle(mut angle: f64) -> f64 {
    while angle > std::f64::consts::PI {
        angle -= 2.0 * std::f64::consts::PI;
    }
    while angle < -std::f64::consts::PI {
        angle += 2.0 * std::f64::consts::PI;
    }
    angle
}

fn point_segment_distance(p: Point, a: Point, b: Point) -> f64 {
    let dx = b.x - a.x;
    let dz = b.z - a.z;
    let length2 = dx * dx + dz * dz;
    if length2 == 0.0 {
        return p.distance(a);
    }
    let t = (((p.x - a.x) * dx + (p.z - a.z) * dz) / length2).clamp(0.0, 1.0);
    p.distance(a.lerp(b, t))
}

fn segment_distance(a: Point, b: Point, c: Point, d: Point) -> f64 {
    let cross =
        |p: Point, q: Point, r: Point| (q.x - p.x) * (r.z - p.z) - (q.z - p.z) * (r.x - p.x);
    let ab_c = cross(a, b, c);
    let ab_d = cross(a, b, d);
    let cd_a = cross(c, d, a);
    let cd_b = cross(c, d, b);
    if ((ab_c > 0.0 && ab_d < 0.0) || (ab_c < 0.0 && ab_d > 0.0))
        && ((cd_a > 0.0 && cd_b < 0.0) || (cd_a < 0.0 && cd_b > 0.0))
    {
        return 0.0;
    }
    point_segment_distance(a, c, d)
        .min(point_segment_distance(b, c, d))
        .min(point_segment_distance(c, a, b))
        .min(point_segment_distance(d, a, b))
}

fn synchronous_distance(a0: Point, a1: Point, b0: Point, b1: Point) -> f64 {
    point_segment_distance(
        Point { x: 0.0, z: 0.0 },
        Point {
            x: a0.x - b0.x,
            z: a0.z - b0.z,
        },
        Point {
            x: a1.x - b1.x,
            z: a1.z - b1.z,
        },
    )
}

fn swept_box_distance(a: Point, b: Point, rack: &Rack) -> f64 {
    let min_x = rack.x_m - rack.width_m / 2.0;
    let max_x = rack.x_m + rack.width_m / 2.0;
    let min_z = rack.z_m - rack.depth_m / 2.0;
    let max_z = rack.z_m + rack.depth_m / 2.0;
    let inside = |p: Point| p.x >= min_x && p.x <= max_x && p.z >= min_z && p.z <= max_z;
    if inside(a) || inside(b) {
        return 0.0;
    }
    let corners = [
        Point { x: min_x, z: min_z },
        Point { x: max_x, z: min_z },
        Point { x: max_x, z: max_z },
        Point { x: min_x, z: max_z },
    ];
    (0..4)
        .map(|i| segment_distance(a, b, corners[i], corners[(i + 1) % 4]))
        .fold(f64::INFINITY, f64::min)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    let default_output =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target/warehouse-fleet");
    let trace_path = args
        .get(1)
        .map_or_else(|| default_output.join("trace.json"), PathBuf::from);
    let validation_path = args
        .get(2)
        .map_or_else(|| default_output.join("validation.json"), PathBuf::from);
    for output in [&trace_path, &validation_path] {
        if let Some(parent) = output
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
        {
            fs::create_dir_all(parent)?;
        }
    }
    let first = Simulation::new(SEED, true).run(true);
    let replay = Simulation::new(SEED, true).run(true);
    let control = Simulation::new(SEED, false).run(false);
    let mut validation = first.validation(&replay, &control);
    validation.passed = validation.replay_full_state_bit_exact
        && validation.collision_count == 0
        && validation.rack_incursion_count == 0
        && validation.incident_active_occupancy_violations == 0
        && validation.completed_tasks >= 15
        && validation.robots_with_task_completions >= 10
        && validation.completed_charge_visits >= 2
        && validation.incident_route_changes >= 1
        && validation.incident_passages_without_closure_control > 0
        && validation.routed_charge_visits >= 1
        && validation.captured_frames_with_charging > 0
        && validation.captured_frames_with_loaded_transport > 0
        && validation.captured_frames_with_active_incident > 0
        && validation
            .incident_started_s
            .is_some_and(|t| t < INCIDENT_END_S)
        && validation.maximum_observed_speed_m_s <= MAX_SPEED_M_S + 1e-9
        && validation.maximum_observed_yaw_rate_rad_s <= YAW_RATE_RAD_S + 1e-9
        && validation.maximum_observed_acceleration_m_s2 <= ACCEL_M_S2 + 1e-9;
    fs::write(validation_path, serde_json::to_vec_pretty(&validation)?)?;
    println!("{}", serde_json::to_string_pretty(&validation)?);
    let trace = Trace {
        schema_version: 1,
        source_provenance: SourceProvenance::compiled(),
        seed: SEED,
        simulation_dt_s: DT,
        capture_dt_s: 0.25,
        playback_speed: 5.0,
        robot_count: ROBOTS,
        extent_m: Extent {
            width: 46.0,
            depth: 33.0,
        },
        robot_radius_m: RADIUS_M,
        model: validation.model,
        racks: first.graph.racks,
        stations: first.graph.stations,
        lanes: first.graph.lanes,
        frames: first.frames,
        events: first.events,
        validation: validation.clone(),
    };
    fs::write(trace_path, serde_json::to_vec(&trace)?)?;
    assert!(validation.passed, "complete fleet acceptance gates failed");
    assert!(
        validation.replay_full_state_bit_exact,
        "full-state replay changed"
    );
    assert_eq!(validation.collision_count, 0, "robot swept collisions");
    assert_eq!(
        validation.rack_incursion_count, 0,
        "rack footprint incursions"
    );
    assert_eq!(
        validation.incident_active_occupancy_violations, 0,
        "closed aisle occupied"
    );
    assert!(
        validation.completed_tasks >= 15,
        "fleet did not deliver enough tasks"
    );
    assert!(
        validation.robots_with_task_completions >= 10,
        "fleet progress concentrated in too few robots"
    );
    assert!(
        validation.completed_charge_visits >= 2,
        "charge pipeline did not complete"
    );
    assert!(
        validation.incident_route_changes >= 1,
        "incident did not alter actual routes"
    );
    assert!(
        validation.incident_passages_without_closure_control > 0,
        "closure negative control did not use incident aisle"
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn directed_graph_reaches_all_stations_with_and_without_incident() {
        let graph = Graph::new();
        for from in &graph.stations {
            for to in &graph.stations {
                for closed in [false, true] {
                    assert!(graph.plan(from.node, to.node, closed).is_some());
                }
            }
        }
    }

    #[test]
    fn swept_collision_guard_finds_between_sample_crossing() {
        let a = Point { x: -1.0, z: 0.0 };
        let b = Point { x: 1.0, z: 0.0 };
        let c = Point { x: 0.0, z: -1.0 };
        let d = Point { x: 0.0, z: 1.0 };
        assert_eq!(synchronous_distance(a, b, c, d), 0.0);
        assert_eq!(segment_distance(a, b, c, d), 0.0);
    }

    #[test]
    fn lease_denial_never_evicts_occupied_cell() {
        let mut sim = Simulation::new(SEED, true);
        let cell = GridCoord {
            x: sim.robots[0].node as isize,
            y: 0,
        };
        assert!(!sim.traffic.request(1, &[cell], 0.0).unwrap());
        assert_eq!(sim.traffic.owner(cell), Some(0));
    }

    #[test]
    fn rest_to_rest_profile_obeys_speed_and_acceleration_limits() {
        for length in [0.2, 1.0, 2.0591260281974, 7.0] {
            let mut previous_speed = 0.0;
            let mut previous_position = 0.0;
            for step in 0..300 {
                let (position, speed) = motion_profile(length, step as f64 * DT);
                assert!(speed <= MAX_SPEED_M_S + 1e-12);
                assert!((speed - previous_speed).abs() <= ACCEL_M_S2 * DT + 1e-12);
                assert!(position >= previous_position && position <= length);
                previous_speed = speed;
                previous_position = position;
            }
            assert_eq!(previous_position, length);
            assert_eq!(previous_speed, 0.0);
        }
    }

    #[test]
    fn junction_reservation_prevents_mutual_approach_deadlock() {
        let mut sim = Simulation::new(SEED, true);
        let node = |graph: &Graph, x: f64, z: f64| {
            graph
                .points
                .iter()
                .position(|p| p.x == x && p.z == z)
                .unwrap()
        };
        let a = node(&sim.graph, 10.0, 7.0);
        let a_next = node(&sim.graph, 10.0, 8.0);
        let b = node(&sim.graph, 13.0, 10.0);
        let b_next = node(&sim.graph, 12.0, 10.0);
        for (id, from, next, yaw) in [
            (0, a, a_next, std::f64::consts::FRAC_PI_2),
            (1, b, b_next, std::f64::consts::PI),
        ] {
            sim.traffic.release(id as u32);
            sim.leases.retain(|_, (owner, _)| *owner != id);
            sim.robots[id].node = from;
            sim.robots[id].position = sim.graph.points[from];
            sim.robots[id].path = VecDeque::from([next]);
            sim.robots[id].yaw_rad = yaw;
            sim.renew(id, 0.0);
        }
        sim.move_robot(0, 0.0);
        sim.move_robot(1, 0.0);
        assert!(sim.robots[0].motion.is_some());
        assert!(sim.robots[1].motion.is_none());
        assert!(sim.robots[1].waiting);
        let junction = node(&sim.graph, 10.0, 10.0);
        assert_eq!(
            sim.traffic.owner(GridCoord {
                x: junction as isize,
                y: 2
            }),
            Some(0)
        );
    }

    #[test]
    fn full_state_replay_words_include_non_pose_operational_state() {
        let mut sim = Simulation::new(SEED, true);
        let before = sim.full_state_words();
        sim.robots[1].battery -= 0.001;
        assert_ne!(before, sim.full_state_words());
        let battery_state = sim.full_state_words();
        sim.pick_booking[0] = Some(1);
        assert_ne!(battery_state, sim.full_state_words());
        let booking_state = sim.full_state_words();
        sim.graph.junctions.pop();
        assert_ne!(booking_state, sim.full_state_words());
    }
}
