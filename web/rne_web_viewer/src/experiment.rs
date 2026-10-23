//! Reproducible planner/controller experiments on a planar circular footprint.
use crate::scene_config::{BoxSpec, Point, SceneSpec};
use rne_core::{SimClock, SimDuration};
use rne_math::{Quat, Vec3};
use rne_nav::{
    plan_path, pure_pursuit_follow, Costmap, CostmapConfig, DwaConfig, DwaOutcome, DwaPlanner,
    GlobalPlannerConfig, OccupancyGrid, Path2d, Pose2d, PurePursuitConfig, VelocityCommand2d,
    COST_LETHAL,
};
use rne_render::{RenderScene, VisualShape};
use rne_world::Transform3;
use std::collections::VecDeque;
use std::fmt::Write;
use std::sync::Arc;

const DT_S: f64 = 33_333_333.0 / 1_000_000_000.0;
const RADIUS_M: f64 = 0.28;
const MAX_TICKS: u32 = 1800;
const TASKS: [&str; 8] = [
    "目的地へ移動",
    "障害物を迂回",
    "途中で障害物を追加",
    "カスタムシーン",
    "狭い通路",
    "横切る障害物",
    "行き止まりを迂回",
    "3地点を巡回",
];
const PLANNERS: [&str; 2] = ["直線", "A*"];
const CONTROLLERS: [&str; 2] = ["Pure Pursuit", "DWA"];

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Config {
    pub(crate) task: usize,
    pub(crate) planner: usize,
    pub(crate) controller: usize,
    pub(crate) seed: u32,
    pub(crate) recovery: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Running,
    Waiting,
    Success,
    PlanFailed,
    Collision,
    ControlFailed,
    Timeout,
}
impl State {
    fn label(self) -> &'static str {
        match self {
            Self::Running => "実行中",
            Self::Waiting => "待機・再試行",
            Self::Success => "成功",
            Self::PlanFailed => "計画失敗",
            Self::Collision => "接触で停止",
            Self::ControlFailed => "制御失敗",
            Self::Timeout => "時間切れ",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Obstacle {
    x: f64,
    y: f64,
    hx: f64,
    hy: f64,
}
impl Obstacle {
    fn clearance(self, x: f64, y: f64) -> f64 {
        ((x - self.x).abs() - self.hx)
            .max(0.0)
            .hypot(((y - self.y).abs() - self.hy).max(0.0))
            - RADIUS_M
    }
}

fn make_map(obstacles: &[Obstacle]) -> Costmap {
    let mut values = Vec::with_capacity(80 * 80);
    for y in 0..80 {
        for x in 0..80 {
            let (px, py) = (
                -4.0 + (f64::from(x) + 0.5) * 0.1,
                -4.0 + (f64::from(y) + 0.5) * 0.1,
            );
            let blocked = px.abs() > 3.65
                || py.abs() > 3.65
                || obstacles.iter().any(|o| o.clearance(px, py) < 0.10);
            values.push(if blocked { 3500 } else { -2000 });
        }
    }
    let grid =
        OccupancyGrid::from_log_odds(80, 80, 0.1, Pose2d::new(-4.0, -4.0, 0.0), values).unwrap();
    Costmap::from_occupancy(
        &grid,
        &CostmapConfig {
            inscribed_radius_m: 0.0,
            inflation_radius_m: 0.35,
            cost_scaling_factor: 5.0,
            ..CostmapConfig::default()
        },
    )
    .unwrap()
}

fn point(p: Pose2d) -> Vec3 {
    Vec3::new(p.x_m, p.y_m, 0.0)
}
fn distance(a: Pose2d, b: Pose2d) -> f64 {
    (a.x_m - b.x_m).hypot(a.y_m - b.y_m)
}
fn line_free(map: &Costmap, a: Pose2d, b: Pose2d) -> bool {
    let steps = (distance(a, b) / 0.025).ceil().max(1.0) as usize;
    (0..=steps).all(|i| {
        let t = i as f64 / steps as f64;
        let p = point(a) * (1.0 - t) + point(b) * t;
        map.world_to_grid(p)
            .and_then(|c| map.cost_at(c))
            .is_some_and(|v| v < COST_LETHAL)
    })
}

#[derive(Clone, Debug, PartialEq)]
struct Snapshot {
    remaining_goals: VecDeque<Pose2d>,
    tick: u32,
    pose: Pose2d,
    goal: Pose2d,
    obstacles: Vec<Obstacle>,
    path: Arc<Path2d>,
    state: State,
    trace_len: usize,
    travelled_m: f64,
    error_sq: f64,
    samples: u32,
    clearance_m: f64,
    replans: u32,
    reached_goals: u32,
    waiting_total: u32,
}

#[derive(Debug)]
struct Run {
    config: Config,
    pose: Pose2d,
    command: VelocityCommand2d,
    goal: Pose2d,
    goal_tolerance_m: f64,
    remaining_goals: VecDeque<Pose2d>,
    reached_goals: u32,
    obstacles: Vec<Obstacle>,
    pending: Vec<BoxSpec>,
    scene_id: Option<String>,
    map: Costmap,
    path: Arc<Path2d>,
    trace: Vec<Pose2d>,
    state: State,
    ticks: u32,
    clock: SimClock,
    error_sq: f64,
    samples: u32,
    travelled_m: f64,
    min_clearance_m: f64,
    replans: u32,
    recorded: bool,
    waiting_span: u32,
    waiting_total: u32,
    frames: Vec<Snapshot>,
}

impl Run {
    #[cfg(test)]
    fn new(config: Config) -> Self {
        Self::with_scene(config, None)
    }

    fn with_scene(config: Config, custom: Option<&SceneSpec>) -> Self {
        let shift = f64::from(config.seed.wrapping_mul(1664525).wrapping_add(1013904223) >> 24)
            / 255.0
            * 0.2
            - 0.1;
        let spec = if config.task == 3 {
            custom.cloned().unwrap_or_default()
        } else {
            SceneSpec {
                goal: Point {
                    x_m: 2.8,
                    z_m: if config.task == 0 { -0.55 } else { 0.0 },
                },
                obstacles: match config.task {
                    1 => vec![BoxSpec {
                        x_m: 1.4,
                        z_m: -shift,
                        width_m: 0.5,
                        depth_m: 1.0,
                        appear_step: 0,
                    }],
                    2 => vec![BoxSpec {
                        x_m: 1.8,
                        z_m: -(f64::from(config.seed % 11) * 0.015 - 0.075),
                        width_m: 0.44,
                        depth_m: 1.0,
                        appear_step: 60,
                    }],
                    4 => vec![
                        box_spec(1.6, -0.58, 2.0, 0.2),
                        box_spec(1.6, 0.58, 2.0, 0.2),
                    ],
                    5 => vec![box_spec(1.55, -moving_y(config.seed, 0), 0.4, 0.4)],
                    // U opens toward the start: the robot must exit sideways first.
                    6 => vec![
                        box_spec(1.25, 0.0, 0.2, 2.0),
                        box_spec(0.65, -1.0, 1.4, 0.2),
                        box_spec(0.65, 1.0, 1.4, 0.2),
                    ],
                    _ => vec![],
                },
            }
        };
        let obstacles: Vec<_> = spec
            .obstacles
            .iter()
            .filter(|o| o.appear_step == 0)
            .map(obstacle)
            .collect();
        let pending = spec
            .obstacles
            .iter()
            .filter(|o| o.appear_step > 0)
            .cloned()
            .collect();
        let scene_id = (config.task == 3).then(|| spec.id());
        let mut goals: VecDeque<_> = if config.task == 7 {
            [
                Pose2d::new(1.5, 0.8, 0.0),
                Pose2d::new(2.6, -0.8, 0.0),
                Pose2d::new(0.3, -1.3, 0.0),
            ]
            .into()
        } else {
            [Pose2d::new(spec.goal.x_m, -spec.goal.z_m, 0.0)].into()
        };
        let mut run = Self {
            config,
            pose: Pose2d::IDENTITY,
            command: VelocityCommand2d::ZERO,
            goal: goals.pop_front().unwrap(),
            goal_tolerance_m: 0.12,
            remaining_goals: goals,
            reached_goals: 0,
            map: make_map(&obstacles),
            obstacles,
            pending,
            scene_id,
            path: Arc::new(Path2d::from_points(&[])),
            trace: vec![Pose2d::IDENTITY],
            state: State::Running,
            ticks: 0,
            clock: SimClock::new(SimDuration::from_ticks(33_333_333)),
            error_sq: 0.0,
            samples: 0,
            travelled_m: 0.0,
            min_clearance_m: f64::INFINITY,
            replans: 0,
            recorded: false,
            waiting_span: 0,
            waiting_total: 0,
            frames: vec![],
        };
        run.min_clearance_m = run
            .obstacles
            .iter()
            .map(|o| o.clearance(run.pose.x_m, run.pose.y_m))
            .fold(f64::INFINITY, f64::min);
        run.plan();
        run.capture();
        run
    }

    fn plan(&mut self) {
        self.state = State::Running;
        self.path = Arc::new(if self.config.planner == 0 {
            Path2d::from_points(&[point(self.pose), point(self.goal)])
        } else {
            match plan_path(
                &self.map,
                point(self.pose),
                point(self.goal),
                &GlobalPlannerConfig::default(),
            ) {
                Ok(path) => path,
                Err(_) => {
                    self.state = if self.config.recovery {
                        State::Waiting
                    } else {
                        State::PlanFailed
                    };
                    return;
                }
            }
        });
        if self
            .path
            .waypoints()
            .windows(2)
            .any(|w| !line_free(&self.map, w[0], w[1]))
        {
            self.state = if self.config.recovery {
                State::Waiting
            } else {
                State::PlanFailed
            };
        }
    }

    fn capture(&mut self) {
        self.frames.push(Snapshot {
            remaining_goals: self.remaining_goals.clone(),
            tick: self.ticks,
            pose: self.pose,
            goal: self.goal,
            obstacles: self.obstacles.clone(),
            path: self.path.clone(),
            state: self.state,
            trace_len: self.trace.len(),
            travelled_m: self.travelled_m,
            error_sq: self.error_sq,
            samples: self.samples,
            clearance_m: self.min_clearance_m,
            replans: self.replans,
            reached_goals: self.reached_goals,
            waiting_total: self.waiting_total,
        });
    }
    fn tick(&mut self) {
        if !matches!(self.state, State::Running | State::Waiting) {
            return;
        }
        let previous = self.state;
        if self.state == State::Waiting {
            self.command = VelocityCommand2d::ZERO;
            if self.config.task == 5 {
                for n in 0..=4 {
                    let y = moving_y(self.config.seed, self.ticks)
                        + (moving_y(self.config.seed, self.ticks + 1)
                            - moving_y(self.config.seed, self.ticks))
                            * f64::from(n)
                            / 4.0;
                    let mut o = self.obstacles[0];
                    o.y = y;
                    let clearance = o.clearance(self.pose.x_m, self.pose.y_m);
                    self.min_clearance_m = self.min_clearance_m.min(clearance);
                    if clearance <= 0.0 {
                        self.obstacles[0] = o;
                        self.state = State::Collision;
                        break;
                    }
                }
                if self.state != State::Collision {
                    self.obstacles[0].y = moving_y(self.config.seed, self.ticks + 1);
                }
            }
            self.ticks += 1;
            self.clock.advance(self.clock.fixed_delta());
            self.waiting_span += 1;
            self.waiting_total += 1;
            let due: Vec<_> = self
                .pending
                .iter()
                .filter(|o| o.appear_step <= self.ticks)
                .map(obstacle)
                .collect();
            self.obstacles.extend(due);
            self.pending.retain(|o| o.appear_step > self.ticks);
            let clearance = self
                .obstacles
                .iter()
                .map(|o| o.clearance(self.pose.x_m, self.pose.y_m))
                .fold(f64::INFINITY, f64::min);
            self.min_clearance_m = self.min_clearance_m.min(clearance);
            if clearance <= 0.0 {
                self.state = State::Collision;
            }
            if self.state == State::Waiting && self.waiting_span.is_multiple_of(15) {
                self.map = make_map(&self.obstacles);
                self.replans += 1;
                self.plan();
                if self.state == State::Running {
                    self.waiting_span = 0;
                }
            }
            if self.state == State::Waiting && self.waiting_span >= 300 {
                self.state = State::PlanFailed;
            }
            if matches!(self.state, State::Waiting | State::Running) && self.ticks >= MAX_TICKS {
                self.state = State::Timeout;
            }
        } else {
            self.tick_running();
            if self.state == State::Waiting {
                self.command = VelocityCommand2d::ZERO;
            }
        }
        if self.ticks.is_multiple_of(3)
            || self.state != previous
            || !matches!(self.state, State::Running | State::Waiting)
        {
            self.capture();
        }
    }
    fn tick_running(&mut self) {
        if self.state != State::Running {
            return;
        }
        // The obstacle follows a seeded triangle wave at fixed simulation ticks.
        // Refresh its physical position every tick; replan at a fixed 3 Hz.
        if self.config.task == 5 {
            self.obstacles[0].y = moving_y(self.config.seed, self.ticks);
            let clearance = self.obstacles[0].clearance(self.pose.x_m, self.pose.y_m);
            self.min_clearance_m = self.min_clearance_m.min(clearance);
            if clearance <= 0.0 {
                self.state = State::Collision;
                return;
            }
            if self.ticks.is_multiple_of(10) {
                self.map = make_map(&self.obstacles);
                self.replans += 1;
                self.plan();
                if self.state != State::Running {
                    return;
                }
            }
        }
        let due: Vec<_> = self
            .pending
            .iter()
            .filter(|o| o.appear_step == self.ticks)
            .map(obstacle)
            .collect();
        if !due.is_empty() {
            self.obstacles.extend(due);
            self.pending.retain(|o| o.appear_step != self.ticks);
            self.map = make_map(&self.obstacles);
            self.min_clearance_m = self.min_clearance_m.min(
                self.obstacles
                    .iter()
                    .map(|o| o.clearance(self.pose.x_m, self.pose.y_m))
                    .fold(f64::INFINITY, f64::min),
            );
            self.replans += 1;
            self.plan();
            if self.state != State::Running {
                return;
            }
        }
        if distance(self.pose, self.goal) <= self.goal_tolerance_m {
            self.reach_goal();
            return;
        }
        let desired = if self.config.controller == 0 {
            pure_pursuit_follow(
                &self.path,
                self.pose,
                &PurePursuitConfig {
                    lookahead_m: 0.45,
                    max_linear_m_s: 0.55,
                    max_angular_rad_s: 1.6,
                    goal_tolerance_m: self.goal_tolerance_m,
                    slow_radius_m: 0.5,
                },
            )
            .map(|r| r.command)
            .map_err(|_| ())
        } else {
            DwaPlanner::new(DwaConfig {
                max_linear_m_s: 0.55,
                max_angular_rad_s: 1.6,
                linear_samples: 7,
                angular_samples: 15,
                simulate_time_s: 1.2,
                simulation_step_s: 0.1,
                goal_tolerance_m: self.goal_tolerance_m,
                heading_lookahead_m: 0.65,
                heading_weight: 2.0,
                path_weight: 2.0,
                clearance_weight: 0.25,
                velocity_weight: 0.7,
                ..DwaConfig::default()
            })
            .compute_command(&self.map, &self.path, self.pose, self.command)
            .map(|r| match r {
                DwaOutcome::Command(c) => c,
                DwaOutcome::Reached => VelocityCommand2d::ZERO,
            })
            .map_err(|_| ())
        };
        let Ok(desired) = desired else {
            self.state = State::ControlFailed;
            return;
        };
        // Both methods share actuator speed and acceleration limits.
        self.command.linear_m_s +=
            (desired.linear_m_s - self.command.linear_m_s).clamp(-DT_S, DT_S);
        self.command.angular_rad_s +=
            (desired.angular_rad_s - self.command.angular_rad_s).clamp(-3.0 * DT_S, 3.0 * DT_S);
        let old = self.pose;
        let half_yaw = old.yaw_rad + self.command.angular_rad_s * DT_S * 0.5;
        let next = Pose2d::new(
            old.x_m + self.command.linear_m_s * DT_S * half_yaw.cos(),
            old.y_m + self.command.linear_m_s * DT_S * half_yaw.sin(),
            old.yaw_rad + self.command.angular_rad_s * DT_S,
        );
        self.ticks += 1;
        self.clock.advance(self.clock.fixed_delta());
        // Sweep the circular footprint along the actual step, not just its endpoint.
        for i in 0..=4 {
            let t = f64::from(i) / 4.0;
            let x = old.x_m + (next.x_m - old.x_m) * t;
            let y = old.y_m + (next.y_m - old.y_m) * t;
            let clearance = self
                .obstacles
                .iter()
                .map(|o| {
                    let mut sample = *o;
                    if self.config.task == 5 {
                        sample.y += (moving_y(self.config.seed, self.ticks) - sample.y) * t;
                    }
                    sample.clearance(x, y)
                })
                .fold(f64::INFINITY, f64::min);
            self.min_clearance_m = self.min_clearance_m.min(clearance);
            if clearance <= 0.0 || x.abs() > 3.7 || y.abs() > 3.7 {
                // Keep the displayed trace and metrics at the detected contact,
                // rather than recording a collision while leaving the old pose.
                self.pose = Pose2d::new(x, y, old.yaw_rad + (next.yaw_rad - old.yaw_rad) * t);
                self.travelled_m += distance(old, self.pose);
                let error = self
                    .path
                    .closest_point(point(self.pose))
                    .unwrap()
                    .distance_m;
                self.error_sq += error * error;
                self.samples += 1;
                self.trace.push(self.pose);
                if self.config.task == 5 {
                    let y = self.obstacles[0].y;
                    self.obstacles[0].y = y + (moving_y(self.config.seed, self.ticks) - y) * t;
                }
                self.command = VelocityCommand2d::ZERO;
                self.state = State::Collision;
                return;
            }
        }
        self.pose = next;
        if self.config.task == 5 {
            self.obstacles[0].y = moving_y(self.config.seed, self.ticks);
        }
        self.travelled_m += distance(old, next);
        let error = self.path.closest_point(point(next)).unwrap().distance_m;
        self.error_sq += error * error;
        self.samples += 1;
        if self.ticks.is_multiple_of(3) {
            self.trace.push(next);
        }
        if distance(next, self.goal) <= self.goal_tolerance_m {
            self.reach_goal();
        } else if self.ticks >= MAX_TICKS {
            self.state = State::Timeout;
        }
    }

    fn reach_goal(&mut self) {
        self.reached_goals += 1;
        if let Some(goal) = self.remaining_goals.pop_front() {
            self.goal = goal;
            self.replans += 1;
            self.plan();
        } else {
            self.state = State::Success;
        }
    }

    fn rmse(&self) -> Option<f64> {
        (self.samples > 0).then(|| (self.error_sq / f64::from(self.samples)).sqrt())
    }
}

#[derive(Clone, Debug, PartialEq)]
struct Record {
    config: Config,
    scene_id: Option<String>,
    state: State,
    elapsed_s: f64,
    rmse_m: Option<f64>,
    travelled_m: f64,
    clearance_m: f64,
    planned_m: f64,
    replans: u32,
    reached_goals: u32,
    trace: Vec<Pose2d>,
    batch_id: u64,
    waiting_total: u32,
    frames: Vec<Snapshot>,
}

#[derive(Debug, Default)]
pub(crate) struct Experiment {
    run: Option<Run>,
    queue: VecDeque<Config>,
    records: Vec<Record>,
    pub(crate) paused: bool,
    pub(crate) hidden_traces: u8,
    batch: bool,
    batch_id: u64,
    pub(crate) draft: SceneSpec,
    batch_scene: Option<SceneSpec>,
    pub(crate) editor_visible: bool,
    replay_run: Option<Run>,
    replay_key: Option<(usize, u32)>,
}

impl Experiment {
    pub(crate) fn start(&mut self, config: Config, batch: bool) {
        self.queue.clear();
        self.replay_run = None;
        self.replay_key = None;
        self.paused = false;
        self.batch = batch;
        self.batch_id = self.batch_id.wrapping_add(1);
        self.batch_scene = (config.task == 3).then(|| self.draft.clone());
        if batch {
            self.records.clear();
            for planner in 0..2 {
                for controller in 0..2 {
                    self.queue.push_back(Config {
                        planner,
                        controller,
                        ..config
                    });
                }
            }
            self.run = Some(Run::with_scene(
                self.queue.pop_front().unwrap(),
                self.batch_scene.as_ref(),
            ));
        } else {
            self.run = Some(Run::with_scene(config, self.batch_scene.as_ref()));
        }
    }
    pub(crate) fn stop(&mut self) {
        self.run = None;
        self.replay_run = None;
        self.replay_key = None;
        self.queue.clear();
        self.paused = false;
    }
    pub(crate) fn active(&self) -> bool {
        self.run.is_some()
    }
    pub(crate) fn advance(&mut self, step: bool) {
        if self.replay_run.is_some() || (self.paused && !step) {
            return;
        }
        for _ in 0..if self.batch && !step { 32 } else { 1 } {
            let Some(run) = self.run.as_mut() else {
                return;
            };
            if matches!(run.state, State::Running | State::Waiting) {
                run.tick();
            }
            if !matches!(run.state, State::Running | State::Waiting) && !run.recorded {
                if run.trace.last().copied() != Some(run.pose) {
                    run.trace.push(run.pose);
                }
                if run.frames.last().is_none_or(|f| {
                    f.tick != run.ticks || f.state != run.state || f.trace_len != run.trace.len()
                }) {
                    run.capture();
                }
                self.records.push(Record {
                    config: run.config,
                    scene_id: run.scene_id.clone(),
                    state: run.state,
                    elapsed_s: run.clock.sim_time().as_seconds().value(),
                    rmse_m: run.rmse(),
                    travelled_m: run.travelled_m,
                    clearance_m: run.min_clearance_m,
                    planned_m: run.path.length_m(),
                    replans: run.replans,
                    reached_goals: run.reached_goals,
                    trace: run.trace.clone(),
                    batch_id: self.batch_id,
                    waiting_total: run.waiting_total,
                    frames: run.frames.clone(),
                });
                if self.records.len() > 12 {
                    self.records.remove(0);
                }
                run.recorded = true;
            }
            if run.recorded {
                if let Some(config) = self.queue.pop_front() {
                    self.run = Some(Run::with_scene(config, self.batch_scene.as_ref()));
                } else {
                    return;
                }
            }
        }
    }
    fn view(&self) -> Option<&Run> {
        self.replay_run.as_ref().or(self.run.as_ref())
    }
    pub(crate) fn replaying(&self) -> bool {
        self.replay_run.is_some()
    }
    pub(crate) fn clear_replay(&mut self) {
        self.replay_run = None;
        self.replay_key = None;
    }
    pub(crate) fn replay_options(&self) -> String {
        if !self.queue.is_empty() || !self.run.as_ref().is_some_and(|r| r.recorded) {
            return String::new();
        }
        self.records
            .iter()
            .enumerate()
            .map(|(i, r)| {
                format!(
                    "<option value='{i}' data-times='{}'>{} · {} × {}</option>",
                    r.frames
                        .iter()
                        .map(|f| f.tick.to_string())
                        .collect::<Vec<_>>()
                        .join(","),
                    TASKS[r.config.task],
                    PLANNERS[r.config.planner],
                    CONTROLLERS[r.config.controller]
                )
            })
            .collect()
    }
    pub(crate) fn seek_replay(&mut self, index: usize, tick: u32) {
        if self.replay_key == Some((index, tick))
            || !self.queue.is_empty()
            || !self.run.as_ref().is_some_and(|r| r.recorded)
        {
            return;
        }
        let Some(record) = self.records.get(index) else {
            return;
        };
        let Some(f) = record
            .frames
            .iter()
            .rev()
            .find(|f| f.tick <= tick)
            .or_else(|| record.frames.first())
        else {
            return;
        };
        let mut r = Run::with_scene(record.config, None);
        r.pose = f.pose;
        r.goal = f.goal;
        r.remaining_goals = f.remaining_goals.clone();
        r.scene_id = record.scene_id.clone();
        r.obstacles = f.obstacles.clone();
        r.path = f.path.clone();
        r.state = f.state;
        r.ticks = f.tick;
        r.trace = record.trace[..f.trace_len.min(record.trace.len())].to_vec();
        r.clock = SimClock::new(SimDuration::from_ticks(33_333_333));
        r.clock
            .advance(SimDuration::from_ticks(u64::from(f.tick) * 33_333_333));
        r.travelled_m = f.travelled_m;
        r.error_sq = f.error_sq;
        r.samples = f.samples;
        r.min_clearance_m = f.clearance_m;
        r.replans = f.replans;
        r.reached_goals = f.reached_goals;
        r.waiting_total = f.waiting_total;
        self.replay_run = Some(r);
        self.replay_key = Some((index, tick));
    }
    pub(crate) fn pose(&self) -> Option<(f64, f64, f64)> {
        self.view()
            .map(|r| (r.pose.x_m, -r.pose.y_m, r.pose.yaw_rad))
            .or_else(|| self.editor_visible.then_some((0.0, 0.0, 0.0)))
    }
    pub(crate) fn full_map(&self) -> bool {
        self.view().is_some_and(|r| r.config.task >= 3)
    }
    pub(crate) fn status(&self) -> String {
        let Some(r) = self.view() else {
            return if self.editor_visible {
                "シーン編集中 · 編集を終えて選択条件で実行"
            } else {
                "手動操作 · タスクと手法を選んで実行"
            }
            .into();
        };
        format!(
            "{} · {} × {} · {}{} · {:.2} s · 再計画 {}回 · 到達 {}地点",
            TASKS[r.config.task],
            PLANNERS[r.config.planner],
            CONTROLLERS[r.config.controller],
            if self.replay_run.is_some() {
                "リプレイ"
            } else {
                r.state.label()
            },
            if self.paused {
                "（一時停止）"
            } else {
                ""
            },
            r.clock.sim_time().as_seconds().value(),
            r.replans,
            r.reached_goals
        )
    }
    pub(crate) fn metrics(&self) -> String {
        let Some(r) = self.view() else {
            return "青＝計画 / 緑＝実軌跡 / 橙＝障害物".into();
        };
        format!(
            "目標まで {:.2} m · 追従RMSE {} m · 移動 {:.2} m · 最小余裕 {} m · 待機 {:.2} s",
            distance(r.pose, r.goal),
            number(r.rmse()),
            r.travelled_m,
            number(r.min_clearance_m.is_finite().then_some(r.min_clearance_m)),
            f64::from(r.waiting_total) * DT_S
        )
    }
    pub(crate) fn results_html(&self) -> String {
        let mut html = String::new();
        for r in self.records.iter().rev() {
            let tag = TASKS[r.config.task];
            let scene_tag = r
                .scene_id
                .as_ref()
                .map_or_else(String::new, |id| format!("<br>scene {}", &id[..8]));
            let color = trace_color(r.config);
            let recovery = if r.config.recovery {
                "待機あり"
            } else {
                "即時終了"
            };
            let wait_s = f64::from(r.waiting_total) * DT_S;
            let _ = write!(html, "<tr><td title='{}'>{tag}<br>seed {}{scene_tag}<br>{recovery}</td><td><details><summary><span style='color:{color}'>●</span> {}<br>{}</summary><p>移動 {:.2} m<br>最小余裕 {} m<br>計画 {:.2} m<br>再計画 {}回<br>到達 {}地点<br>待機 {wait_s:.2} s</p></details></td><td>{}</td><td>{:.2}</td><td>{}</td><td>{:.2}</td><td>{}</td></tr>",
                TASKS[r.config.task], r.config.seed, PLANNERS[r.config.planner], CONTROLLERS[r.config.controller],
                r.travelled_m, number(r.clearance_m.is_finite().then_some(r.clearance_m)), r.planned_m, r.replans, r.reached_goals,
                r.state.label(), r.elapsed_s, number(r.rmse_m), r.travelled_m, number(r.clearance_m.is_finite().then_some(r.clearance_m)));
        }
        html
    }
    pub(crate) fn map_svg(&self) -> String {
        let Some(r) = self.view() else {
            return String::new();
        };
        let mut svg = String::from("<rect x='-4' y='-4' width='8' height='8' fill='#101d28'/>");
        for o in &r.obstacles {
            let _ = write!(
                svg,
                "<rect x='{}' y='{}' width='{}' height='{}' fill='#c68446'/>",
                o.x - o.hx,
                -o.y - o.hy,
                o.hx * 2.0,
                o.hy * 2.0
            );
        }
        let mut line = |poses: &[Pose2d], color: &str| {
            let _ = write!(
                svg,
                "<polyline fill='none' stroke='{color}' stroke-width='.035' points='"
            );
            for p in poses {
                let _ = write!(svg, "{},{} ", p.x_m, -p.y_m);
            }
            svg.push_str("'/>");
        };
        line(r.path.waypoints(), "#68bfff");
        if self.batch && self.replay_run.is_none() {
            for record in self.records.iter().filter(|record| {
                record.batch_id == self.batch_id && self.trace_visible(record.config)
            }) {
                line(&record.trace, trace_color(record.config));
            }
        }
        if (!r.recorded || !self.batch || self.replay_run.is_some()) && self.trace_visible(r.config)
        {
            line(&r.trace, trace_color(r.config));
        }
        for (i, goal) in r.remaining_goals.iter().enumerate() {
            let _ = write!(svg, "<circle cx='{}' cy='{}' r='.1' fill='none' stroke='#ffd773' stroke-width='.035'/><text x='{}' y='{}' fill='#ffd773' font-size='.2'>{}</text>", goal.x_m, -goal.y_m, goal.x_m+0.15, -goal.y_m, r.reached_goals as usize+i+2);
        }
        let _ = write!(svg, "<circle cx='{}' cy='{}' r='.12' fill='#ffd773'/><circle cx='{}' cy='{}' r='{RADIUS_M}' fill='#e9f2fb' fill-opacity='.25' stroke='#e9f2fb' stroke-width='.02'/><line x1='{}' y1='{}' x2='{}' y2='{}' stroke='#fff' stroke-width='.04'/>",
            r.goal.x_m,-r.goal.y_m,r.pose.x_m,-r.pose.y_m,r.pose.x_m,-r.pose.y_m,
            r.pose.x_m+0.4*r.pose.yaw_rad.cos(),-r.pose.y_m-0.4*r.pose.yaw_rad.sin());
        if r.config.task == 7 {
            let _ = write!(
                svg,
                "<text x='{}' y='{}' fill='#ffd773' font-size='.2'>{}</text>",
                r.goal.x_m + 0.15,
                -r.goal.y_m,
                r.reached_goals.min(2) + 1
            );
        }
        svg
    }
    fn trace_visible(&self, config: Config) -> bool {
        self.hidden_traces & (1 << (config.planner * 2 + config.controller)) == 0
    }
    pub(crate) fn comparison_summary(&self) -> String {
        if !self.batch || self.run.is_none() {
            return "4組比較すると、同条件の実軌跡を重ねて表示します。".into();
        }
        let records: Vec<_> = self
            .records
            .iter()
            .filter(|r| r.batch_id == self.batch_id)
            .collect();
        let success = records.iter().filter(|r| r.state == State::Success).count();
        format!(
            "同条件の比較 · {}/4組完了 · 成功 {success}/{}組（各1試行）",
            records.len(),
            records.len()
        )
    }
    pub(crate) fn overlay(&self, scene: &mut RenderScene) {
        let Some(r) = self.view() else {
            if self.editor_visible {
                for spec in &self.draft.obstacles {
                    draw_obstacle(scene, obstacle(spec), spec.appear_step > 0);
                }
                draw_goal(scene, self.draft.goal);
            }
            return;
        };
        for o in &r.obstacles {
            scene.items.push(RenderScene::item_from_visual(
                Transform3::from_translation_rotation(Vec3::new(o.x, 0.3, -o.y), Quat::IDENTITY),
                VisualShape::Box {
                    size_m: Vec3::new(o.hx * 2.0, 0.6, o.hy * 2.0),
                },
                [0.55, 0.24, 0.07, 1.0],
                Transform3::IDENTITY,
            ));
        }
        draw_trace(scene, r.path.waypoints(), [0.1, 0.5, 1.0, 1.0]);
        if self.batch && self.replay_run.is_none() {
            for record in self.records.iter().filter(|record| {
                record.batch_id == self.batch_id && self.trace_visible(record.config)
            }) {
                draw_trace(scene, &record.trace, trace_rgba(record.config));
            }
        }
        if (!self.batch || !r.recorded || self.replay_run.is_some()) && self.trace_visible(r.config)
        {
            draw_trace(scene, &r.trace, trace_rgba(r.config));
        }
        for goal in &r.remaining_goals {
            draw_goal(
                scene,
                Point {
                    x_m: goal.x_m,
                    z_m: -goal.y_m,
                },
            );
        }
        scene.items.push(RenderScene::item_from_visual(
            Transform3::from_translation_rotation(
                Vec3::new(r.goal.x_m, 0.03, -r.goal.y_m),
                Quat::IDENTITY,
            ),
            VisualShape::Box {
                size_m: Vec3::new(0.2, 0.02, 0.2),
            },
            [1.0, 0.7, 0.1, 1.0],
            Transform3::IDENTITY,
        ));
    }
}
fn moving_y(seed: u32, ticks: u32) -> f64 {
    let phase = (f64::from(ticks) * DT_S * 0.4 + f64::from(seed % 11) * 0.02) % 5.6;
    -1.4 + if phase <= 2.8 { phase } else { 5.6 - phase }
}
fn box_spec(x_m: f64, z_m: f64, width_m: f64, depth_m: f64) -> BoxSpec {
    BoxSpec {
        x_m,
        z_m,
        width_m,
        depth_m,
        appear_step: 0,
    }
}
fn trace_color(c: Config) -> &'static str {
    ["#72edb1", "#ffb45e", "#cd9bff", "#55def5"][c.planner * 2 + c.controller]
}
fn trace_rgba(c: Config) -> [f32; 4] {
    [
        [0.45, 0.93, 0.69, 1.0],
        [1.0, 0.71, 0.37, 1.0],
        [0.80, 0.61, 1.0, 1.0],
        [0.33, 0.87, 0.96, 1.0],
    ][c.planner * 2 + c.controller]
}
fn draw_trace(scene: &mut RenderScene, poses: &[Pose2d], color: [f32; 4]) {
    let stride = poses.len().div_ceil(100).max(1);
    let mut reduced: Vec<_> = poses.iter().step_by(stride).copied().collect();
    if let Some(last) = poses.last() {
        if reduced.last() != Some(last) {
            reduced.push(*last);
        }
    }
    for w in reduced.windows(2) {
        let (a, b) = (w[0], w[1]);
        let length = distance(a, b);
        if length < 1e-6 {
            continue;
        }
        scene.items.push(RenderScene::item_from_visual(
            Transform3::from_translation_rotation(
                Vec3::new((a.x_m + b.x_m) * 0.5, 0.025, -(a.y_m + b.y_m) * 0.5),
                Quat::from_rotation_y((b.y_m - a.y_m).atan2(b.x_m - a.x_m)),
            ),
            VisualShape::Box {
                size_m: Vec3::new(length, 0.008, 0.025),
            },
            color,
            Transform3::IDENTITY,
        ));
    }
}
fn obstacle(spec: &BoxSpec) -> Obstacle {
    Obstacle {
        x: spec.x_m,
        y: -spec.z_m,
        hx: spec.width_m * 0.5,
        hy: spec.depth_m * 0.5,
    }
}
fn draw_obstacle(scene: &mut RenderScene, o: Obstacle, timed: bool) {
    scene.items.push(RenderScene::item_from_visual(
        Transform3::from_translation_rotation(
            Vec3::new(o.x, if timed { 0.02 } else { 0.3 }, -o.y),
            Quat::IDENTITY,
        ),
        VisualShape::Box {
            size_m: Vec3::new(o.hx * 2.0, if timed { 0.03 } else { 0.6 }, o.hy * 2.0),
        },
        if timed {
            [0.1, 0.45, 0.8, 1.0]
        } else {
            [0.55, 0.24, 0.07, 1.0]
        },
        Transform3::IDENTITY,
    ));
}
fn draw_goal(scene: &mut RenderScene, goal: Point) {
    scene.items.push(RenderScene::item_from_visual(
        Transform3::from_translation_rotation(Vec3::new(goal.x_m, 0.03, goal.z_m), Quat::IDENTITY),
        VisualShape::Box {
            size_m: Vec3::new(0.2, 0.02, 0.2),
        },
        [1.0, 0.7, 0.1, 1.0],
        Transform3::IDENTITY,
    ));
}

fn number(value: Option<f64>) -> String {
    value.map_or_else(|| "—".into(), |v| format!("{v:.3}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    fn complete(config: Config) -> Run {
        let mut r = Run::new(config);
        for _ in 0..MAX_TICKS + 1 {
            if !matches!(r.state, State::Running | State::Waiting) {
                break;
            }
            r.tick();
        }
        r
    }
    #[test]
    fn recovery_advances_time_retries_and_replay_preserves_measured_results() {
        let mut run = Run::new(Config {
            task: 5,
            recovery: true,
            seed: 42,
            ..Config::default()
        });
        for _ in 0..MAX_TICKS + 1 {
            if !matches!(run.state, State::Running | State::Waiting) {
                break;
            }
            run.tick();
        }
        println!(
            "recovery moving: {:?}, wait {}, ticks {}, x {}",
            run.state, run.waiting_total, run.ticks, run.pose.x_m
        );
        assert!(run.waiting_total > 0);
        assert_eq!(run.state, State::Success);
        assert_ne!(run.state, State::Waiting);
        assert!(run.ticks <= MAX_TICKS);
        let blocked = complete(Config {
            task: 1,
            recovery: true,
            ..Config::default()
        });
        assert_eq!(blocked.state, State::PlanFailed);
        assert_eq!(blocked.waiting_total, 300);
        assert_eq!(blocked.ticks, 300);
        assert_eq!(blocked.travelled_m, 0.0);
        let mut e = Experiment::default();
        e.start(
            Config {
                task: 5,
                recovery: true,
                seed: 42,
                ..Config::default()
            },
            true,
        );
        for _ in 0..MAX_TICKS {
            e.advance(false);
            if e.records.len() == 4 {
                break;
            }
        }
        assert_eq!(e.records.len(), 4);
        assert!(!e.replay_options().is_empty());
        let records = e.records.clone();
        e.seek_replay(0, 0);
        assert!(e.replaying());
        assert_eq!(e.view().unwrap().ticks, 0);
        assert_eq!(e.pose(), Some((0.0, 0.0, 0.0)));
        let final_tick = records[0].frames.last().unwrap().tick;
        e.seek_replay(0, final_tick);
        assert_eq!(
            e.view().unwrap().obstacles,
            records[0].frames.last().unwrap().obstacles
        );
        let scene = e.map_svg();
        e.advance(false);
        assert_eq!(scene, e.map_svg());
        assert_eq!(e.records, records);
        e.clear_replay();
        assert!(e.replay_run.is_none());
    }
    #[test]
    fn navigation_method_matrix_has_real_outcomes_and_reproducible_results() {
        for planner in 0..2 {
            for controller in 0..2 {
                let c = Config {
                    planner,
                    controller,
                    seed: 42,
                    task: 0,
                    recovery: false,
                };
                let a = complete(c);
                let b = complete(c);
                assert_eq!(a.state, State::Success, "{c:?}");
                assert_eq!(a.pose, b.pose);
                assert_eq!(a.ticks, b.ticks);
                assert_eq!(a.error_sq, b.error_sq);
                assert!(a.travelled_m > 2.5);
            }
        }
        let failed = complete(Config {
            task: 1,
            ..Config::default()
        });
        assert_eq!(failed.state, State::PlanFailed);
        assert_eq!(failed.samples, 0);
        assert_eq!(failed.rmse(), None);
        for task in [1, 2] {
            let r = complete(Config {
                task,
                planner: 1,
                controller: 1,
                seed: 42,
                recovery: false,
            });
            assert_eq!(
                r.state,
                State::Success,
                "task {task}: {:?}, pose {:?}",
                r.state,
                r.pose
            );
            assert!(r.min_clearance_m > 0.0);
            assert_eq!(r.replans, u32::from(task == 2));
        }
    }
    #[test]
    fn custom_batch_snapshots_geometry_and_records_its_identity() {
        let mut e = Experiment {
            editor_visible: true,
            ..Experiment::default()
        };
        assert!(!e.active());
        assert_eq!(e.pose(), Some((0.0, 0.0, 0.0)));
        let id = e.draft.id();
        e.start(
            Config {
                task: 3,
                seed: 42,
                ..Config::default()
            },
            true,
        );
        assert!(e.full_map());
        e.draft.goal.x_m = -2.0; // Later edits must not change queued comparisons.
        assert_ne!(e.draft.id(), id);
        for _ in 0..MAX_TICKS {
            e.advance(false);
            if e.records.len() == 4 {
                break;
            }
        }
        assert_eq!(e.records.len(), 4);
        assert!(e
            .records
            .iter()
            .all(|r| r.state == State::Success && r.scene_id.as_ref() == Some(&id)));
        assert!(e.results_html().contains(&id[..8]));
    }
    #[test]
    fn custom_enclosure_is_valid_but_unreachable_and_timed_scene_replans() {
        let mut spec = SceneSpec {
            goal: Point { x_m: 2.0, z_m: 0.0 },
            obstacles: vec![],
        };
        for (x, z, w, d) in [
            (1.1, 0.0, 0.2, 2.0),
            (2.9, 0.0, 0.2, 2.0),
            (2.0, -0.9, 1.8, 0.2),
            (2.0, 0.9, 1.8, 0.2),
        ] {
            spec.obstacles.push(BoxSpec {
                x_m: x,
                z_m: z,
                width_m: w,
                depth_m: d,
                appear_step: 0,
            });
        }
        spec.validate().unwrap();
        let r = Run::with_scene(
            Config {
                task: 3,
                planner: 1,
                ..Config::default()
            },
            Some(&spec),
        );
        assert_eq!(r.state, State::PlanFailed);
        assert_eq!(r.rmse(), None);
        let mut timed = SceneSpec::default();
        timed.obstacles.push(BoxSpec {
            x_m: 1.8,
            z_m: 0.0,
            width_m: 0.44,
            depth_m: 1.0,
            appear_step: 60,
        });
        timed.validate().unwrap();
        let mut r = Run::with_scene(
            Config {
                task: 3,
                ..Config::default()
            },
            Some(&timed),
        );
        for _ in 0..61 {
            r.tick();
        }
        assert_eq!(r.replans, 1);
        assert_eq!(r.obstacles.len(), 1);
        assert_eq!(r.state, State::PlanFailed);
    }
    #[test]
    fn representative_tasks_and_patrol_are_deterministic() {
        for task in 4..8 {
            let config = Config {
                task,
                planner: 1,
                controller: 1,
                seed: 42,
                recovery: false,
            };
            let a = complete(config);
            let b = complete(config);
            assert_eq!(a.state, b.state);
            assert_eq!(a.trace, b.trace);
            assert_eq!(a.ticks, b.ticks);
            println!(
                "representative {task}: {:?}, reached {}, clearance {}",
                a.state, a.reached_goals, a.min_clearance_m
            );
            if task != 5 {
                assert_eq!(a.state, State::Success);
            }
            if task == 7 {
                assert_eq!(a.reached_goals, 3);
                assert_eq!(a.replans, 2);
            }
        }
        assert!((moving_y(42, 30) - moving_y(42, 0) - 0.4).abs() < 1e-6);
        assert_ne!(moving_y(0, 30), moving_y(42, 30));
        // An obstacle can enter the footprint even while robot speed is near zero.
        let mut crossing = Run::new(Config {
            task: 5,
            seed: 42,
            ..Config::default()
        });
        crossing.ticks = 1; // Between global-plan updates.
        crossing.pose = Pose2d::new(1.55, moving_y(42, 1) + 0.485, 0.0);
        crossing.tick();
        assert_eq!(crossing.state, State::Collision);
        assert!(crossing.min_clearance_m <= 0.0);
    }
    #[test]
    fn comparison_traces_share_a_batch_and_visibility_never_changes_metrics() {
        let mut e = Experiment::default();
        e.start(
            Config {
                seed: 42,
                ..Config::default()
            },
            true,
        );
        for _ in 0..MAX_TICKS {
            e.advance(false);
            if e.records.len() == 4 {
                break;
            }
        }
        let records = e.records.clone();
        assert_eq!(records.len(), 4);
        assert!(records.iter().all(|r| r.trace.len() > 2
            && r.trace.first() == Some(&Pose2d::IDENTITY)
            && r.batch_id == e.batch_id));
        assert_eq!(e.map_svg().matches("<polyline").count(), 5);
        assert!(e.comparison_summary().contains("成功 4/4"));
        e.hidden_traces = 15;
        assert_eq!(e.map_svg().matches("<polyline").count(), 1);
        assert_eq!(e.records, records);
        e.hidden_traces = 0;
        e.start(
            Config {
                task: 6,
                planner: 1,
                ..Config::default()
            },
            false,
        );
        assert_eq!(e.map_svg().matches("<polyline").count(), 2);
        assert!(!e.comparison_summary().contains("成功 4/4"));
    }
    #[test]
    fn contact_and_no_admissible_command_are_distinct_terminal_failures() {
        let mut contact = Run::new(Config::default());
        contact.obstacles.push(Obstacle {
            x: 0.5,
            y: 0.0,
            hx: 0.1,
            hy: 0.5,
        });
        for _ in 0..100 {
            contact.tick();
        }
        assert_eq!(contact.state, State::Collision);
        let frozen = contact.pose;
        let ticks = contact.ticks;
        contact.tick();
        assert_eq!(contact.pose, frozen);
        assert_eq!(contact.ticks, ticks);
        let mut blocked = Run::new(Config {
            controller: 1,
            ..Config::default()
        });
        blocked.obstacles.push(Obstacle {
            x: 0.0,
            y: 0.0,
            hx: 4.0,
            hy: 4.0,
        });
        blocked.map = make_map(&blocked.obstacles);
        blocked.tick();
        assert_eq!(blocked.state, State::ControlFailed);
        assert_eq!(blocked.pose, Pose2d::IDENTITY);
    }
    #[test]
    fn all_task_method_pairs_finish_with_finite_metrics() {
        for seed in [0, 42, 99] {
            for task in [0, 1, 2, 4, 5, 6, 7] {
                for planner in 0..2 {
                    for controller in 0..2 {
                        let c = Config {
                            seed,
                            task,
                            planner,
                            controller,
                            recovery: false,
                        };
                        let r = complete(c);
                        assert_ne!(r.state, State::Running, "{c:?}");
                        assert!(r.pose.is_finite());
                        assert!(r.travelled_m.is_finite());
                        assert!(r.rmse().is_none_or(|v| v.is_finite()));
                        if seed == 42 {
                            println!(
                                "{c:?}: {:?} {:.2}s",
                                r.state,
                                r.clock.sim_time().as_seconds().value()
                            );
                        }
                    }
                }
            }
        }
    }
    #[test]
    fn pause_step_batch_and_stop_preserve_experiment_boundaries() {
        let c = Config {
            seed: 42,
            ..Config::default()
        };
        let mut e = Experiment::default();
        e.start(c, false);
        e.paused = true;
        e.advance(false);
        assert_eq!(e.run.as_ref().unwrap().ticks, 0);
        e.advance(true);
        assert_eq!(e.run.as_ref().unwrap().ticks, 1);
        assert!(e.paused);
        e.start(c, true);
        for _ in 0..MAX_TICKS {
            e.advance(false);
            if e.records.len() == 4 {
                break;
            }
        }
        assert_eq!(e.records.len(), 4);
        assert!(e.records.iter().all(|r| r.state == State::Success));
        for record in &e.records {
            let reference = complete(record.config);
            assert_eq!(
                record.elapsed_s,
                reference.clock.sim_time().as_seconds().value()
            );
            assert_eq!(record.rmse_m, reference.rmse());
            assert_eq!(record.travelled_m, reference.travelled_m);
        }
        assert!(e.status().contains("成功"));
        assert!(e.metrics().contains("追従RMSE"));
        assert!(e.results_html().contains("seed 42"));
        assert!(e.map_svg().contains("polyline"));
        assert!(e.pose().is_some());
        let mut scene = RenderScene::new();
        e.overlay(&mut scene);
        assert!(!scene.items.is_empty());
        let records = e.records.clone();
        e.advance(false);
        assert_eq!(e.records, records);
        e.stop();
        assert!(!e.active());
        assert_eq!(e.records, records);
    }
}

/// A navigation leg reused by the mobile manipulation task, in world X/Z coordinates.
#[derive(Debug)]
pub(crate) struct NavigationLeg(Run);
impl NavigationLeg {
    pub(crate) fn new(
        config: Config,
        start: (f64, f64, f64),
        goal: (f64, f64),
        obstacles: Vec<BoxSpec>,
    ) -> Self {
        let scene = SceneSpec {
            goal: Point {
                x_m: goal.0,
                z_m: goal.1,
            },
            obstacles,
        };
        let mut run = Run::with_scene(Config { task: 3, ..config }, Some(&scene));
        run.pose = Pose2d::new(start.0, -start.1, start.2);
        run.goal_tolerance_m = 0.035;
        run.trace = vec![run.pose];
        run.frames.clear();
        run.plan();
        Self(run)
    }
    pub(crate) fn tick(&mut self) {
        self.0.tick();
    }
    /// Update geometry without resetting the existing waiting/retry budget.
    pub(crate) fn replace_obstacles(&mut self, obstacles: Vec<BoxSpec>) {
        self.0.obstacles = obstacles.iter().map(obstacle).collect();
        self.0.map = make_map(&self.0.obstacles);
        self.0.pending.clear();
        if self.0.state == State::Running {
            self.0.plan();
        }
    }
    pub(crate) fn recovery_progress(&self) -> (bool, u32, u32) {
        (
            self.0.state == State::Waiting,
            self.0.waiting_total,
            self.0.replans,
        )
    }
    pub(crate) fn pose(&self) -> (f64, f64, f64) {
        (self.0.pose.x_m, -self.0.pose.y_m, self.0.pose.yaw_rad)
    }
    pub(crate) fn outcome(&self) -> Option<Result<(), &'static str>> {
        match self.0.state {
            State::Running | State::Waiting => None,
            State::Success => Some(Ok(())),
            state => Some(Err(state.label())),
        }
    }
    pub(crate) fn path(&self) -> Vec<Vec3> {
        self.0
            .path
            .waypoints()
            .iter()
            .map(|p| Vec3::new(p.x_m, 0.025, -p.y_m))
            .collect()
    }
}
