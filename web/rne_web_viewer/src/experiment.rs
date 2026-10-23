//! Reproducible planner/controller experiments on a planar circular footprint.
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

const DT_S: f64 = 33_333_333.0 / 1_000_000_000.0;
const RADIUS_M: f64 = 0.28;
const MAX_TICKS: u32 = 1800;
const TASKS: [&str; 3] = ["目的地へ移動", "障害物を迂回", "途中で障害物を追加"];
const PLANNERS: [&str; 2] = ["直線", "A*"];
const CONTROLLERS: [&str; 2] = ["Pure Pursuit", "DWA"];

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Config {
    pub(crate) task: usize,
    pub(crate) planner: usize,
    pub(crate) controller: usize,
    pub(crate) seed: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Running,
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

#[derive(Debug)]
struct Run {
    config: Config,
    pose: Pose2d,
    command: VelocityCommand2d,
    goal: Pose2d,
    obstacles: Vec<Obstacle>,
    map: Costmap,
    path: Path2d,
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
}

impl Run {
    fn new(config: Config) -> Self {
        let shift = f64::from(config.seed.wrapping_mul(1664525).wrapping_add(1013904223) >> 24)
            / 255.0
            * 0.2
            - 0.1;
        let obstacles = if config.task == 1 {
            vec![Obstacle {
                x: 1.4,
                y: shift,
                hx: 0.25,
                hy: 0.5,
            }]
        } else {
            vec![]
        };
        let mut run = Self {
            config,
            pose: Pose2d::IDENTITY,
            command: VelocityCommand2d::ZERO,
            goal: Pose2d::new(2.8, if config.task == 0 { 0.55 } else { 0.0 }, 0.0),
            map: make_map(&obstacles),
            obstacles,
            path: Path2d::from_points(&[]),
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
        };
        run.min_clearance_m = run
            .obstacles
            .iter()
            .map(|o| o.clearance(run.pose.x_m, run.pose.y_m))
            .fold(f64::INFINITY, f64::min);
        run.plan();
        run
    }

    fn plan(&mut self) {
        self.path = if self.config.planner == 0 {
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
                    self.state = State::PlanFailed;
                    return;
                }
            }
        };
        if self
            .path
            .waypoints()
            .windows(2)
            .any(|w| !line_free(&self.map, w[0], w[1]))
        {
            self.state = State::PlanFailed;
        }
    }

    fn tick(&mut self) {
        if self.state != State::Running {
            return;
        }
        if self.config.task == 2 && self.ticks == 60 {
            let shift = f64::from(self.config.seed % 11) * 0.015 - 0.075;
            self.obstacles.push(Obstacle {
                x: 1.8,
                y: shift,
                hx: 0.22,
                hy: 0.5,
            });
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
        if distance(self.pose, self.goal) <= 0.12 {
            self.state = State::Success;
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
                    goal_tolerance_m: 0.12,
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
                goal_tolerance_m: 0.12,
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
                .map(|o| o.clearance(x, y))
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
                self.command = VelocityCommand2d::ZERO;
                self.state = State::Collision;
                return;
            }
        }
        self.pose = next;
        self.travelled_m += distance(old, next);
        let error = self.path.closest_point(point(next)).unwrap().distance_m;
        self.error_sq += error * error;
        self.samples += 1;
        if self.ticks.is_multiple_of(3) {
            self.trace.push(next);
        }
        if distance(next, self.goal) <= 0.12 {
            self.state = State::Success;
        } else if self.ticks >= MAX_TICKS {
            self.state = State::Timeout;
        }
    }

    fn rmse(&self) -> Option<f64> {
        (self.samples > 0).then(|| (self.error_sq / f64::from(self.samples)).sqrt())
    }
}

#[derive(Clone, Debug, PartialEq)]
struct Record {
    config: Config,
    state: State,
    elapsed_s: f64,
    rmse_m: Option<f64>,
    travelled_m: f64,
    clearance_m: f64,
    planned_m: f64,
    replans: u32,
}

#[derive(Debug, Default)]
pub(crate) struct Experiment {
    run: Option<Run>,
    queue: VecDeque<Config>,
    records: Vec<Record>,
    pub(crate) paused: bool,
    batch: bool,
}

impl Experiment {
    pub(crate) fn start(&mut self, config: Config, batch: bool) {
        self.queue.clear();
        self.paused = false;
        self.batch = batch;
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
            self.run = Some(Run::new(self.queue.pop_front().unwrap()));
        } else {
            self.run = Some(Run::new(config));
        }
    }
    pub(crate) fn stop(&mut self) {
        self.run = None;
        self.queue.clear();
        self.paused = false;
    }
    pub(crate) fn active(&self) -> bool {
        self.run.is_some()
    }
    pub(crate) fn advance(&mut self, step: bool) {
        if self.paused && !step {
            return;
        }
        for _ in 0..if self.batch && !step { 32 } else { 1 } {
            let Some(run) = self.run.as_mut() else {
                return;
            };
            if run.state == State::Running {
                run.tick();
            }
            if run.state != State::Running && !run.recorded {
                if run.trace.last().copied() != Some(run.pose) {
                    run.trace.push(run.pose);
                }
                self.records.push(Record {
                    config: run.config,
                    state: run.state,
                    elapsed_s: run.clock.sim_time().as_seconds().value(),
                    rmse_m: run.rmse(),
                    travelled_m: run.travelled_m,
                    clearance_m: run.min_clearance_m,
                    planned_m: run.path.length_m(),
                    replans: run.replans,
                });
                if self.records.len() > 12 {
                    self.records.remove(0);
                }
                run.recorded = true;
            }
            if run.recorded {
                if let Some(config) = self.queue.pop_front() {
                    self.run = Some(Run::new(config));
                } else {
                    return;
                }
            }
        }
    }
    pub(crate) fn pose(&self) -> Option<(f64, f64, f64)> {
        self.run
            .as_ref()
            .map(|r| (r.pose.x_m, -r.pose.y_m, r.pose.yaw_rad))
    }
    pub(crate) fn status(&self) -> String {
        let Some(r) = &self.run else {
            return "手動操作 · タスクと手法を選んで実行".into();
        };
        format!(
            "{} · {} × {} · {}{} · {:.2} s · 再計画 {}回",
            TASKS[r.config.task],
            PLANNERS[r.config.planner],
            CONTROLLERS[r.config.controller],
            r.state.label(),
            if self.paused {
                "（一時停止）"
            } else {
                ""
            },
            r.clock.sim_time().as_seconds().value(),
            r.replans
        )
    }
    pub(crate) fn metrics(&self) -> String {
        let Some(r) = &self.run else {
            return "青＝計画 / 緑＝実軌跡 / 橙＝障害物".into();
        };
        format!(
            "目標まで {:.2} m · 追従RMSE {} m · 移動 {:.2} m · 最小余裕 {} m",
            distance(r.pose, r.goal),
            number(r.rmse()),
            r.travelled_m,
            number(r.min_clearance_m.is_finite().then_some(r.min_clearance_m))
        )
    }
    pub(crate) fn results_html(&self) -> String {
        let mut html = String::new();
        for r in self.records.iter().rev() {
            let tag = ["移動", "迂回", "途中追加"][r.config.task];
            let _ = write!(html, "<tr><td title='{}'>{tag}<br>seed {}</td><td><details><summary>{}<br>{}</summary><p>移動 {:.2} m<br>最小余裕 {} m<br>計画 {:.2} m<br>再計画 {}回</p></details></td><td>{}</td><td>{:.2}</td><td>{}</td></tr>",
                TASKS[r.config.task], r.config.seed, PLANNERS[r.config.planner], CONTROLLERS[r.config.controller],
                r.travelled_m, number(r.clearance_m.is_finite().then_some(r.clearance_m)), r.planned_m, r.replans,
                r.state.label(), r.elapsed_s, number(r.rmse_m));
        }
        html
    }
    pub(crate) fn map_svg(&self) -> String {
        let Some(r) = &self.run else {
            return String::new();
        };
        let mut svg = String::from("<rect x='-0.6' y='-2' width='4.1' height='4' fill='#101d28'/>");
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
        line(&r.trace, "#72edb1");
        let _ = write!(svg, "<circle cx='{}' cy='{}' r='.12' fill='#ffd773'/><circle cx='{}' cy='{}' r='{RADIUS_M}' fill='#e9f2fb' fill-opacity='.25' stroke='#e9f2fb' stroke-width='.02'/><line x1='{}' y1='{}' x2='{}' y2='{}' stroke='#fff' stroke-width='.04'/>",
            r.goal.x_m,-r.goal.y_m,r.pose.x_m,-r.pose.y_m,r.pose.x_m,-r.pose.y_m,
            r.pose.x_m+0.4*r.pose.yaw_rad.cos(),-r.pose.y_m-0.4*r.pose.yaw_rad.sin());
        svg
    }
    pub(crate) fn overlay(&self, scene: &mut RenderScene) {
        let Some(r) = &self.run else {
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
        for (poses, color) in [
            (r.path.waypoints(), [0.1, 0.5, 1.0, 1.0]),
            (r.trace.as_slice(), [0.1, 0.9, 0.4, 1.0]),
        ] {
            // Limit rendered markers; metric accumulation still uses every tick.
            let stride = (poses.len() / 100).max(1);
            let reduced: Vec<_> = poses.iter().step_by(stride).copied().collect();
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
fn number(value: Option<f64>) -> String {
    value.map_or_else(|| "—".into(), |v| format!("{v:.3}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    fn complete(config: Config) -> Run {
        let mut r = Run::new(config);
        for _ in 0..MAX_TICKS + 1 {
            if r.state != State::Running {
                break;
            }
            r.tick();
        }
        r
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
            for task in 0..3 {
                for planner in 0..2 {
                    for controller in 0..2 {
                        let c = Config {
                            seed,
                            task,
                            planner,
                            controller,
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
