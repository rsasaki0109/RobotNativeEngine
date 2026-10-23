//! Deterministic G1 left-arm experiments using RNE's URDF-derived FK and IK.
use rne_ecs::{Entity, World};
use rne_math::{Pose3, Quat, Vec3};
use rne_planning::{
    GoalConstraint, JointInterpolationPlanner, MotionPlanRequest, MotionPlanner, PlanningGroup,
    PlanningOptions, PlanningScene, RrtConnectPlanner,
};
use rne_render::{RenderScene, VisualShape};
use rne_robot::{ColliderShape, CollisionPrimitive, IkOptions, KinematicModel};
use rne_urdf_import::{parse_urdf, spawn_urdf_robot_with_config, UrdfJointType, UrdfSpawnConfig};
use rne_world::Transform3;
use std::{collections::VecDeque, fmt::Write, sync::Arc};

const DT: f64 = 1.0 / 30.0;
const LIMIT: u32 = 1800;
const SEARCH_LIMIT: usize = 600;
const TASKS: [&str; 4] = [
    "手先リーチ",
    "持ち上げて配置",
    "障害物越しに配置",
    "到達範囲外",
];
const PLANNERS: [&str; 3] = ["関節直結", "上方経由", "RRT-Connect"];
const CONTROLLERS: [&str; 2] = ["速度制限P", "加速度制限P"];
const COLORS: [[f32; 4]; 6] = [
    [0.45, 0.93, 0.69, 1.0],
    [1.0, 0.71, 0.37, 1.0],
    [0.80, 0.61, 1.0, 1.0],
    [0.33, 0.87, 0.96, 1.0],
    [0.95, 0.61, 0.76, 1.0],
    [0.96, 0.89, 0.47, 1.0],
];

#[derive(Debug)]
struct SearchFailure {
    label: &'static str,
    iterations: usize,
}

#[derive(Debug)]
struct Arm {
    model: KinematicModel,
    planning: PlanningScene,
    hand: Entity,
    elbow: Entity,
    active: Vec<bool>,
    viewer_order: Vec<usize>,
}
impl Arm {
    fn load() -> Self {
        // Add a fixed tool frame at the rubber hand's distal grasp point.
        let urdf=include_str!("../../../assets/robots/g1_description/g1_23dof.urdf").replace("</robot>",r#"<link name="rne_left_tool"/><joint name="rne_left_tool_joint" type="fixed"><parent link="left_wrist_roll_rubber_hand"/><child link="rne_left_tool"/><origin xyz="0.055 0 0" rpy="0 0 0"/></joint></robot>"#);
        let doc = parse_urdf(&urdf).unwrap();
        let mut world = World::new();
        let robot = spawn_urdf_robot_with_config(
            &mut world,
            &doc,
            UrdfSpawnConfig {
                attach_physics: false,
                attach_colliders: false,
                attach_mesh_colliders: false,
                ..UrdfSpawnConfig::default()
            },
        )
        .unwrap();
        world
            .entity_mut(robot.base_link)
            .insert(Transform3::from_translation_rotation(
                Vec3::new(0.0, 0.80, 0.0),
                Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2),
            ));
        let model = KinematicModel::from_robot(&world, robot.robot).unwrap();
        let names = model.movable_dof_names();
        let active = names
            .iter()
            .map(|n| {
                n.starts_with("left_")
                    && (n.contains("shoulder") || n.contains("elbow") || n.contains("wrist"))
            })
            .collect();
        let viewer_order = doc
            .joints
            .iter()
            .filter(|j| j.joint_type == UrdfJointType::Revolute)
            .map(|j| names.iter().position(|n| n == &j.name).unwrap())
            .collect();
        let hand = robot.links["rne_left_tool"];
        let elbow = robot.links["left_elbow_link"];
        let group =
            PlanningGroup::chain(&model, "left_arm", robot.links["torso_link"], hand).unwrap();
        let mut planning = PlanningScene::from_world(&world, robot.robot)
            .unwrap()
            .with_group(group);
        let q = vec![0.0; model.dof()];
        let fk = model.forward_kinematics(&q).unwrap();
        let e = fk.link_transform(elbow).unwrap();
        let segment =
            e.rotation.inverse() * (fk.link_transform(hand).unwrap().translation - e.translation);
        let touch: Vec<_> = (0..model.link_count())
            .map(|i| model.link_entity(i).unwrap())
            .collect();
        planning
            .attach_body(
                "forearm",
                elbow,
                ColliderShape::Capsule {
                    half_height_m: segment.length() / 2.0,
                    radius_m: 0.026,
                },
                Transform3::from_translation_rotation(
                    segment * 0.5,
                    Quat::from_rotation_arc(Vec3::Y, segment.normalize()),
                ),
                touch.clone(),
            )
            .unwrap();
        planning
            .attach_body(
                "tool",
                hand,
                ColliderShape::Sphere { radius_m: 0.026 },
                Transform3::IDENTITY,
                touch,
            )
            .unwrap();
        Self {
            hand,
            elbow,
            model,
            planning,
            active,
            viewer_order,
        }
    }
    fn configure_scene(&mut self, blocker: Option<Vec3>, held_offset: Option<Vec3>) {
        self.planning.remove_collision_object("blocker");
        if let Some(center_m) = blocker {
            self.planning.add_collision_object(
                "blocker",
                CollisionPrimitive::Cuboid {
                    center_m,
                    axes: [Vec3::X, Vec3::Y, Vec3::Z],
                    half_extents_m: Vec3::splat(0.025),
                },
            );
        }
        self.planning.detach_body("held");
        if let Some(offset) = held_offset {
            let touch = (0..self.model.link_count())
                .map(|i| self.model.link_entity(i).unwrap())
                .collect();
            // The object keeps its measured offset in the tool frame.
            self.planning
                .attach_body(
                    "held",
                    self.hand,
                    ColliderShape::Sphere { radius_m: 0.0255 },
                    Transform3::from_translation_rotation(offset, Quat::IDENTITY),
                    touch,
                )
                .unwrap();
        }
    }
    fn search(
        &mut self,
        start: &[f64],
        goal: &[f64],
        seed: u64,
    ) -> Result<(VecDeque<Vec<f64>>, usize), SearchFailure> {
        let request = MotionPlanRequest::new(
            start.to_vec(),
            GoalConstraint::joint(goal.to_vec()),
            PlanningOptions {
                seed,
                max_iterations: SEARCH_LIMIT,
                step_size: 0.20,
                collision_check_steps: 12,
                waypoint_count: 2,
                ..PlanningOptions::default()
            },
        )
        .with_group("left_arm");
        let response = JointInterpolationPlanner::new()
            .plan(&self.planning, &request)
            .or_else(|_| RrtConnectPlanner::new().plan(&self.planning, &request));
        response
            .map(|r| {
                (
                    r.trajectory
                        .points()
                        .iter()
                        .skip(1)
                        .map(|p| p.positions.clone())
                        .collect(),
                    r.iterations,
                )
            })
            .map_err(|e| SearchFailure {
                iterations: if matches!(e, rne_planning::PlanningError::IterationLimit) {
                    SEARCH_LIMIT
                } else {
                    0
                },
                label: match e {
                    rne_planning::PlanningError::StartInCollision => "開始姿勢が衝突",
                    rne_planning::PlanningError::GoalInCollision => "目標姿勢が衝突",
                    _ => "経路未発見（探索上限）",
                },
            })
    }

    fn position(&self, q: &[f64], link: Entity) -> Vec3 {
        self.model
            .forward_kinematics(q)
            .unwrap()
            .link_transform(link)
            .unwrap()
            .translation
    }
    fn tip(&self, q: &[f64]) -> Vec3 {
        self.position(q, self.hand)
    }
    fn solve(&self, target: Vec3, seed: &[f64]) -> Option<Vec<f64>> {
        self.model
            .inverse_kinematics_active(
                &Pose3 {
                    translation: target,
                    rotation: Quat::IDENTITY,
                },
                self.hand,
                seed,
                &IkOptions {
                    solve_orientation: false,
                    max_iterations: 250,
                    position_tolerance_m: 0.002,
                    step_size: 0.6,
                    damping: 0.02,
                    ..IkOptions::default()
                },
                &self.active,
            )
            .ok()
            .map(|s| s.joint_positions)
    }
    fn known_pose(&self, pitch: f64, roll: f64, elbow: f64) -> Vec<f64> {
        let mut q = vec![0.0; self.model.dof()];
        for (i, n) in self.model.movable_dof_names().iter().enumerate() {
            q[i] = match n.as_str() {
                "left_shoulder_pitch_joint" => pitch,
                "left_shoulder_roll_joint" => roll,
                "left_elbow_joint" => elbow,
                _ => 0.0,
            };
        }
        q
    }
}
#[derive(Clone, Copy, Debug)]
pub(crate) struct ArmConfig {
    pub(crate) task: usize,
    pub(crate) planner: usize,
    pub(crate) controller: usize,
    pub(crate) seed: u32,
}
#[derive(Clone, Debug, PartialEq)]
struct Frame {
    tick: u32,
    q: Vec<f64>,
    tip: Vec3,
    object: Vec3,
    attached: bool,
    stage: usize,
    state: &'static str,
    planned: Arc<Vec<Vec3>>,
    iterations: usize,
}
#[derive(Debug)]
struct Run {
    config: ArmConfig,
    q: Vec<f64>,
    velocity: Vec<f64>,
    goals: Vec<Vec3>,
    joint_goal: Option<Vec<f64>>,
    route: VecDeque<Vec<f64>>,
    planned: Arc<Vec<Vec3>>,
    iterations: usize,
    stage: usize,
    attach_at: usize,
    release_at: usize,
    object: Vec3,
    place: Vec3,
    attached: bool,
    offset: Vec3,
    blocker: Option<Vec3>,
    state: &'static str,
    tick: u32,
    error_sq: f64,
    samples: u32,
    trace: Vec<Vec3>,
    frames: Vec<Frame>,
    recorded: bool,
}
impl Run {
    fn new(arm: &mut Arm, config: ArmConfig) -> Self {
        let q = vec![0.0; arm.model.dof()];
        let pick = arm.tip(&arm.known_pose(-0.85, 0.15 + f64::from(config.seed % 11) * 0.005, 0.7));
        let place = arm.tip(&arm.known_pose(-1.05, 0.65, 0.8));
        let start = arm.tip(&q);
        let (goals, attach_at, release_at) = if config.task == 3 {
            (vec![Vec3::new(1.8, 1.6, -0.25)], usize::MAX, usize::MAX)
        } else if config.task == 0 {
            (
                if config.planner != 1 {
                    vec![pick]
                } else {
                    vec![(start + pick) * 0.5 + Vec3::Y * 0.12, pick]
                },
                usize::MAX,
                usize::MAX,
            )
        } else if config.planner != 1 {
            (vec![pick, place], 0, 1)
        } else {
            (
                vec![
                    pick + Vec3::Y * 0.05,
                    pick,
                    pick + Vec3::Y * 0.08,
                    place + Vec3::Y * 0.08,
                    place,
                    place + Vec3::Y * 0.05,
                ],
                1,
                4,
            )
        };
        let blocker = (config.task == 2).then_some((pick + place) * 0.5 - Vec3::Y * 0.04);
        let mut r = Self {
            velocity: vec![0.0; q.len()],
            q,
            config,
            goals,
            joint_goal: None,
            route: VecDeque::new(),
            planned: Arc::new(vec![]),
            iterations: 0,
            stage: 0,
            attach_at,
            release_at,
            object: pick,
            place,
            attached: false,
            offset: Vec3::ZERO,
            blocker,
            state: "実行中",
            tick: 0,
            error_sq: 0.0,
            samples: 0,
            trace: vec![start],
            frames: vec![],
            recorded: false,
        };
        r.plan(arm);
        r.capture(arm);
        r
    }
    fn plan(&mut self, arm: &mut Arm) {
        self.route.clear();
        self.planned = Arc::new(vec![]);
        let Some(goal) = arm.solve(self.goals[self.stage], &self.q) else {
            self.state = "IK失敗";
            return;
        };
        if self.config.planner == 2 {
            arm.configure_scene(self.blocker, self.attached.then_some(self.offset));
            match arm.search(
                &self.q,
                &goal,
                u64::from(self.config.seed) + self.stage as u64 * 7919,
            ) {
                Ok((route, iterations)) => {
                    self.route = route;
                    self.iterations += iterations;
                }
                Err(failure) => {
                    self.iterations += failure.iterations;
                    self.state = failure.label;
                    return;
                }
            }
        } else {
            self.route.push_back(goal);
        }
        let mut points = vec![arm.tip(&self.q)];
        let mut previous = self.q.as_slice();
        for waypoint in &self.route {
            for n in 1..=8 {
                let t = f64::from(n) / 8.0;
                let q: Vec<_> = previous
                    .iter()
                    .zip(waypoint)
                    .map(|(a, b)| a + (b - a) * t)
                    .collect();
                points.push(arm.tip(&q));
            }
            previous = waypoint;
        }
        self.planned = Arc::new(points);
        self.joint_goal = self.route.pop_front();
        if self.joint_goal.is_none() {
            self.state = "経路が空";
        }
    }
    fn capture(&mut self, arm: &Arm) {
        self.frames.push(Frame {
            tick: self.tick,
            q: self.q.clone(),
            tip: arm.tip(&self.q),
            object: self.object,
            attached: self.attached,
            stage: self.stage,
            state: self.state,
            planned: self.planned.clone(),
            iterations: self.iterations,
        });
    }
    fn tick(&mut self, arm: &mut Arm) {
        if self.state != "実行中" {
            return;
        }
        let old = self.q.clone();
        let old_stage = self.stage;
        let old_attached = self.attached;
        let target = self.joint_goal.as_ref().unwrap();
        for (i, goal) in target.iter().enumerate() {
            let desired = (4.0 * (goal - self.q[i])).clamp(-0.8, 0.8);
            self.velocity[i] = if self.config.controller == 0 {
                desired
            } else {
                self.velocity[i] + (desired - self.velocity[i]).clamp(-1.6 * DT, 1.6 * DT)
            };
            self.q[i] += self.velocity[i] * DT;
        }
        self.tick += 1;
        let limits = arm.model.joint_limits();
        for (i, limit) in limits.iter().enumerate() {
            let clamped = self.q[i].clamp(limit.lower, limit.upper);
            if clamped != self.q[i] {
                self.velocity[i] = 0.0;
                self.q[i] = clamped;
            }
        }
        let mut hit = false;
        if let Some(center) = self.blocker {
            for n in 0..=8 {
                let t = f64::from(n) / 8.0;
                let q: Vec<_> = old
                    .iter()
                    .zip(&self.q)
                    .map(|(a, b)| a + (b - a) * t)
                    .collect();
                let fk = arm.model.forward_kinematics(&q).unwrap();
                let tool = fk.link_transform(arm.hand).unwrap();
                let tip = tool.translation;
                let elbow = fk.link_transform(arm.elbow).unwrap().translation;
                for j in 0..=8 {
                    let p = elbow + (tip - elbow) * (f64::from(j) / 8.0);
                    let delta = (p - center).abs() - Vec3::splat(0.025);
                    if delta.max(Vec3::ZERO).length() <= 0.018 {
                        hit = true;
                        break;
                    }
                }
                if self.attached {
                    let object = tip + tool.rotation * self.offset;
                    if ((object - center).abs() - Vec3::splat(0.025))
                        .max(Vec3::ZERO)
                        .length()
                        <= 0.0225
                    {
                        hit = true;
                    }
                }
                if hit {
                    self.q = q;
                    break;
                }
            }
        }
        let fk = arm.model.forward_kinematics(&self.q).unwrap();
        let tool = fk.link_transform(arm.hand).unwrap();
        let tip = tool.translation;
        if self.attached {
            self.object = tip + tool.rotation * self.offset;
        }
        let error = target
            .iter()
            .zip(&self.q)
            .zip(&arm.active)
            .filter(|(_, active)| **active)
            .map(|((a, b), _)| (a - b).powi(2))
            .sum::<f64>()
            / arm.active.iter().filter(|a| **a).count() as f64;
        self.error_sq += error;
        self.samples += 1;
        if hit {
            self.state = "腕が障害物に接触";
        } else if !self.route.is_empty() {
            let remaining = target
                .iter()
                .zip(&self.q)
                .map(|(a, b)| (a - b).abs())
                .fold(0.0, f64::max);
            if remaining < 0.008 {
                self.joint_goal = self.route.pop_front();
            }
        } else if (tip - self.goals[self.stage]).length() < 0.008 {
            if self.stage == self.attach_at {
                self.attached = (tip - self.object).length() <= 0.025;
                if !self.attached {
                    self.state = "把持距離超過";
                }
                self.offset = tool.rotation.inverse() * (self.object - tip);
            }
            if self.stage == self.release_at {
                self.attached = false;
            }
            if self.state == "実行中" {
                self.stage += 1;
                if self.stage == self.goals.len() {
                    self.state =
                        if self.config.task == 0 || (self.object - self.place).length() < 0.025 {
                            "成功"
                        } else {
                            "配置誤差超過"
                        };
                } else {
                    self.plan(arm);
                }
            }
        }
        if self.state == "実行中" && self.tick >= LIMIT {
            self.state = "時間切れ";
        }
        self.trace.push(tip);
        if self.tick.is_multiple_of(3)
            || self.state != "実行中"
            || old_stage != self.stage
            || old_attached != self.attached
        {
            self.capture(arm);
        }
    }
}
#[derive(Debug)]
struct Record {
    config: ArmConfig,
    state: &'static str,
    error_m: f64,
    rmse_rad: Option<f64>,
    frames: Vec<Frame>,
    trace: Vec<Vec3>,
    goal: Vec3,
    blocker: Option<Vec3>,
}
impl Record {
    fn time_s(&self) -> f64 {
        f64::from(self.frames.last().unwrap().tick) * DT
    }
    fn movement_m(&self) -> f64 {
        self.trace
            .windows(2)
            .fold(0.0, |travel, w| travel + (w[1] - w[0]).length())
    }
}

fn csv_text(value: &str) -> String {
    format!("\"{}\"", value.replace('"', "\"\""))
}

#[derive(Debug, Default)]
pub(crate) struct Manipulation {
    arm: Option<Arm>,
    run: Option<Run>,
    queue: VecDeque<ArmConfig>,
    records: Vec<Record>,
    pub(crate) paused: bool,
    batch: bool,
    benchmark: Option<ArmConfig>,
    pub(crate) replay: Option<(usize, u32)>,
}
impl Manipulation {
    pub(crate) fn active(&self) -> bool {
        self.run.is_some()
    }
    pub(crate) fn stop(&mut self) {
        self.run = None;
        self.queue.clear();
        self.replay = None;
        self.paused = false;
    }
    pub(crate) fn start(&mut self, c: ArmConfig, batch: bool) {
        self.start_trials(c, batch, false);
    }
    /// Run all six methods on three consecutive, wrapping u32 seeds.
    pub(crate) fn start_benchmark(&mut self, c: ArmConfig) {
        self.start_trials(c, true, true);
    }
    fn start_trials(&mut self, c: ArmConfig, batch: bool, benchmark: bool) {
        if self.arm.is_none() {
            self.arm = Some(Arm::load());
        }
        self.queue.clear();
        self.records.clear();
        self.batch = batch;
        self.benchmark = benchmark.then_some(c);
        self.paused = false;
        self.replay = None;
        if batch {
            for offset in 0..if benchmark { 3 } else { 1 } {
                for planner in 0..3 {
                    for controller in 0..2 {
                        self.queue.push_back(ArmConfig {
                            planner,
                            controller,
                            seed: c.seed.wrapping_add(offset),
                            ..c
                        });
                    }
                }
            }
        } else {
            self.queue.push_back(c);
        }
        self.run = Some(Run::new(
            self.arm.as_mut().unwrap(),
            self.queue.pop_front().unwrap(),
        ));
    }
    pub(crate) fn advance(&mut self, step: bool) {
        if self.replay.is_some() || (self.paused && !step) {
            return;
        }
        for _ in 0..if self.batch && !step { 32 } else { 1 } {
            let (Some(arm), Some(run)) = (&mut self.arm, &mut self.run) else {
                return;
            };
            run.tick(arm);
            if run.state != "実行中" && !run.recorded {
                if run.frames.last().unwrap().tick != run.tick {
                    run.capture(arm);
                }
                self.records.push(Record {
                    config: run.config,
                    state: run.state,
                    error_m: if run.config.task == 0 || run.config.task == 3 {
                        (arm.tip(&run.q) - run.goals.last().unwrap()).length()
                    } else {
                        (run.object - run.place).length()
                    },
                    rmse_rad: (run.samples > 0)
                        .then(|| (run.error_sq / f64::from(run.samples)).sqrt()),
                    frames: run.frames.clone(),
                    trace: run.trace.clone(),
                    goal: if run.config.task == 1 || run.config.task == 2 {
                        run.place
                    } else {
                        *run.goals.last().unwrap()
                    },
                    blocker: run.blocker,
                });
                run.recorded = true;
            }
            if run.recorded {
                if let Some(c) = self.queue.pop_front() {
                    self.run = Some(Run::new(arm, c));
                } else {
                    return;
                }
            }
        }
    }
    fn frame(&self) -> Option<Frame> {
        if let Some((index, tick)) = self.replay {
            if let Some(r) = self.records.get(index) {
                return r
                    .frames
                    .iter()
                    .rev()
                    .find(|f| f.tick <= tick)
                    .cloned()
                    .or_else(|| r.frames.first().cloned());
            }
        }
        let (Some(r), Some(arm)) = (&self.run, &self.arm) else {
            return None;
        };
        Some(Frame {
            tick: r.tick,
            q: r.q.clone(),
            tip: arm.tip(&r.q),
            object: r.object,
            attached: r.attached,
            stage: r.stage,
            state: r.state,
            planned: r.planned.clone(),
            iterations: r.iterations,
        })
    }
    pub(crate) fn seek(&mut self, index: usize, tick: u32) {
        if self.queue.is_empty()
            && self.run.as_ref().is_some_and(|r| r.recorded)
            && index < self.records.len()
        {
            self.replay = Some((index, tick));
        }
    }
    pub(crate) fn targets(&self) -> Option<Vec<f64>> {
        let f = self.frame()?;
        Some(
            self.arm
                .as_ref()?
                .viewer_order
                .iter()
                .map(|i| f.q[*i])
                .collect(),
        )
    }
    pub(crate) fn status(&self) -> String {
        let Some(f) = self.frame() else {
            return "手先リーチ・物体配置を選んで実行".into();
        };
        let c = self
            .replay
            .and_then(|(i, _)| self.records.get(i).map(|r| r.config))
            .unwrap_or(self.run.as_ref().unwrap().config);
        format!(
            "{} · {} × {} · {} · {:.2} s · 完了段階 {} · 探索 {}回 · {}{}",
            TASKS[c.task],
            PLANNERS[c.planner],
            CONTROLLERS[c.controller],
            f.state,
            f64::from(f.tick) * DT,
            f.stage,
            f.iterations,
            if f.attached {
                "物体保持中"
            } else {
                "未保持"
            },
            if self.replay.is_some() {
                " · リプレイ"
            } else {
                ""
            }
        )
    }
    /// Export a snapshot of completed trials, including captured conditions and units.
    /// Empty until the first trial finishes; replay does not alter the snapshot.
    pub(crate) fn results_csv(&self) -> String {
        if self.records.is_empty() {
            return String::new();
        }
        let expected = if self.benchmark.is_some() {
            18
        } else if self.batch {
            6
        } else {
            1
        };
        let mode = if self.benchmark.is_some() {
            "benchmark"
        } else if self.batch {
            "comparison"
        } else {
            "single"
        };
        let state = if self.records.len() == expected {
            "completed"
        } else if self.run.is_none() {
            "interrupted"
        } else if self.paused {
            "paused"
        } else {
            "running"
        };
        let mut csv = String::from("schema_version,mode,batch_state,expected_trials,completed_trials,task_id,task,seed,planner_id,planner,controller_id,controller,result,time_s,hand_travel_m,final_error_m,joint_target_rmse_rad,planning_iterations\r\n");
        for r in &self.records {
            let c = r.config;
            let _ = write!(csv, "1,{mode},{state},{expected},{},{},{},{},{},{},{},{},{},{:.6},{:.6},{:.6},{},{}\r\n",
                self.records.len(), c.task, csv_text(TASKS[c.task]), c.seed,
                c.planner, csv_text(PLANNERS[c.planner]), c.controller, csv_text(CONTROLLERS[c.controller]),
                csv_text(r.state), r.time_s(), r.movement_m(), r.error_m,
                r.rmse_rad.map_or_else(String::new, |v| format!("{v:.6}")),
                r.frames.last().unwrap().iterations);
        }
        csv
    }
    /// Completed trials only; averages include successful trials only.
    pub(crate) fn benchmark_html(&self) -> String {
        let Some(c) = self.benchmark else {
            return String::new();
        };
        let mut s = format!("<p>{} · seed {}, {}, {} · 完了 {}/18{} · 実行中・未実行は集計対象外。平均は成功時のみ。</p><table><caption>6手法 × 3 seed</caption><thead><tr><th>計画 × 制御</th><th>成功 / 完了</th><th>成功率</th><th>平均時間 s</th><th>平均手先移動 m</th></tr></thead><tbody>", TASKS[c.task], c.seed, c.seed.wrapping_add(1), c.seed.wrapping_add(2), self.records.len(), if self.run.is_none() { " · 中断" } else if self.records.len() == 18 { " · 評価完了" } else if self.paused { " · 一時停止" } else { " · 評価中" });
        for (planner, name) in PLANNERS.iter().enumerate() {
            for (controller, control) in CONTROLLERS.iter().enumerate() {
                let trials: Vec<_> = self
                    .records
                    .iter()
                    .filter(|r| r.config.planner == planner && r.config.controller == controller)
                    .collect();
                let successes: Vec<_> = trials.iter().filter(|r| r.state == "成功").collect();
                let mean = |total: f64| {
                    if successes.is_empty() {
                        "—".into()
                    } else {
                        format!("{:.2}", total / successes.len() as f64)
                    }
                };
                let time = mean(successes.iter().map(|r| r.time_s()).sum());
                let movement = mean(successes.iter().map(|r| r.movement_m()).sum());
                let rate = if trials.is_empty() {
                    "—".into()
                } else {
                    format!(
                        "{:.0}%",
                        100.0 * successes.len() as f64 / trials.len() as f64
                    )
                };
                let _ = write!(s, "<tr><td>{name} × {control}</td><td>{}/{}</td><td>{rate}</td><td>{time}</td><td>{movement}</td></tr>", successes.len(), trials.len());
            }
        }
        s.push_str("</tbody></table>");
        s
    }
    pub(crate) fn results_html(&self) -> String {
        let mut s = String::new();
        for r in &self.records {
            let color = [
                "#72edb1", "#ffb45e", "#cd9bff", "#55def5", "#f29bc1", "#f6e379",
            ][r.config.planner * 2 + r.config.controller];
            let _ = write!(
                s,
                "<tr><td><span style='color:{color}'>●</span> {} × {} · seed {}</td><td>{}</td><td>{:.3}</td><td>{}</td><td>{:.2}</td><td>{}</td></tr>",
                PLANNERS[r.config.planner],
                CONTROLLERS[r.config.controller],
                r.config.seed,
                r.state,
                r.error_m,
                r.rmse_rad.map_or_else(|| "—".into(), |v| format!("{v:.3}")),
                r.time_s(),
                r.frames.last().unwrap().iterations
            );
        }
        s
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
                    "<option value='{i}' data-times='{}'>{} × {} · seed {} · {}</option>",
                    r.frames
                        .iter()
                        .map(|f| f.tick.to_string())
                        .collect::<Vec<_>>()
                        .join(","),
                    PLANNERS[r.config.planner],
                    CONTROLLERS[r.config.controller],
                    r.config.seed,
                    r.state
                )
            })
            .collect()
    }
    pub(crate) fn overlay(&self, scene: &mut RenderScene) {
        let (Some(r), Some(f)) = (&self.run, self.frame()) else {
            return;
        };
        let cube = |scene: &mut RenderScene, p: Vec3, size: Vec3, color: [f32; 4]| {
            scene.items.push(RenderScene::item_from_visual(
                Transform3::from_translation_rotation(p, Quat::IDENTITY),
                VisualShape::Box { size_m: size },
                color,
                Transform3::IDENTITY,
            ))
        };
        let selected = self.replay.and_then(|(i, _)| self.records.get(i));
        let goal = selected.map_or_else(
            || {
                if r.config.task == 1 || r.config.task == 2 {
                    r.place
                } else {
                    *r.goals.last().unwrap()
                }
            },
            |record| record.goal,
        );
        if r.config.task == 1 || r.config.task == 2 {
            cube(scene, f.object, Vec3::splat(0.045), [1.0, 0.55, 0.1, 1.0]);
            cube(scene, goal, Vec3::splat(0.065), [0.15, 0.8, 0.3, 0.35]);
        } else {
            cube(scene, goal, Vec3::splat(0.045), [0.1, 0.9, 0.4, 1.0]);
        }
        if let Some(b) = selected.map_or(r.blocker, |record| record.blocker) {
            cube(scene, b, Vec3::splat(0.05), [0.7, 0.25, 0.1, 1.0]);
        }
        for (index, record) in self.records.iter().enumerate() {
            if self.replay.is_some_and(|(i, _)| i != index)
                || (self.replay.is_none() && record.config.seed != r.config.seed)
            {
                continue;
            }
            for (n, p) in record.trace.iter().enumerate().step_by(8) {
                if self.replay.is_some_and(|(_, tick)| n as u32 > tick) {
                    break;
                }
                cube(
                    scene,
                    *p,
                    Vec3::splat(0.008),
                    COLORS[record.config.planner * 2 + record.config.controller],
                );
            }
        }
        if !r.recorded && self.replay.is_none() {
            for p in r.trace.iter().step_by(8) {
                cube(
                    scene,
                    *p,
                    Vec3::splat(0.008),
                    COLORS[r.config.planner * 2 + r.config.controller],
                );
            }
        }
        for p in f.planned.iter() {
            cube(scene, *p, Vec3::splat(0.007), [0.1, 0.5, 1.0, 1.0]);
        }
        cube(scene, f.tip, Vec3::splat(0.018), [0.9, 0.95, 1.0, 1.0]);
    }
}
#[cfg(test)]
mod tests {
    #[test]
    fn csv_exports_captured_trials_and_preserves_results_during_replay() {
        let mut m = Manipulation::default();
        let config = ArmConfig {
            task: 0,
            planner: 2,
            controller: 1,
            seed: 42,
        };
        assert!(m.results_csv().is_empty());
        m.start(config, false);
        assert!(m.results_csv().is_empty());
        for _ in 0..1801 {
            m.advance(false);
        }
        assert_eq!(m.records.len(), 1);
        let csv = m.results_csv();
        let lines: Vec<_> = csv.lines().collect();
        assert_eq!(lines.len(), 2);
        let row: Vec<_> = lines[1].split(',').collect();
        assert_eq!(row.len(), 18);
        assert_eq!(&row[..6], &["1", "single", "completed", "1", "1", "0"]);
        assert_eq!(row[7], "42");
        assert_eq!(row[8], "2");
        assert_eq!(row[10], "1");
        assert_eq!(row[12], "\"成功\"");
        for i in [13, 14, 15, 16] {
            let actual = row[i].parse::<f64>().unwrap();
            assert!(actual.is_finite() && actual >= 0.0);
        }
        assert!((row[13].parse::<f64>().unwrap() - m.records[0].time_s()).abs() < 1e-6);
        assert!((row[14].parse::<f64>().unwrap() - m.records[0].movement_m()).abs() < 1e-6);
        m.seek(0, 0);
        m.advance(false);
        assert_eq!(m.results_csv(), csv);
        m.stop();
        assert_eq!(m.results_csv(), csv);
    }
    #[test]
    fn csv_exports_partial_and_failed_trials_without_inventing_metrics() {
        let mut m = Manipulation::default();
        let config = ArmConfig {
            task: 3,
            planner: 0,
            controller: 0,
            seed: u32::MAX,
        };
        m.start_benchmark(config);
        m.paused = true;
        m.advance(true);
        let csv = m.results_csv();
        assert_eq!(csv.lines().count(), 2);
        assert!(csv
            .lines()
            .nth(1)
            .unwrap()
            .starts_with("1,benchmark,paused,18,1,3,"));
        let row: Vec<_> = csv.lines().nth(1).unwrap().split(',').collect();
        assert_eq!(row[13], "0.000000");
        assert_eq!(row[14], "0.000000");
        assert!(row[15].parse::<f64>().unwrap() > 0.0);
        assert_eq!(row[16], "");
        assert_eq!(row[17], "0");
        m.stop();
        assert!(m.results_csv().contains("benchmark,interrupted,18,1,"));
        m.start_benchmark(config);
        assert!(m.results_csv().is_empty());
        m.advance(false);
        let csv = m.results_csv();
        assert_eq!(csv.lines().count(), 19);
        assert_eq!(csv.matches("benchmark,completed,18,18,").count(), 18);
        assert!(csv.lines().nth(7).unwrap().contains(",0,0,\"関節直結\""));
        assert!(!csv.contains("NaN") && !csv.contains("inf"));
        assert_eq!(csv_text("x,\"y\""), "\"x,\"\"y\"\"\"");
    }
    #[test]
    fn benchmark_counts_completed_trials_and_repeats_seeded_metrics() {
        let config = ArmConfig {
            task: 0,
            planner: 0,
            controller: 0,
            seed: 42,
        };
        let mut m = Manipulation::default();
        m.start_benchmark(config);
        assert_eq!(m.queue.len(), 17);
        assert!(m.benchmark_html().contains("完了 0/18"));
        assert!(!m.benchmark_html().contains("NaN"));
        m.paused = true;
        m.advance(false);
        assert_eq!(m.run.as_ref().unwrap().tick, 0);
        m.advance(true);
        assert_eq!(m.run.as_ref().unwrap().tick, 1);
        m.paused = false;
        for _ in 0..2000 {
            m.advance(false);
        }
        assert_eq!(m.records.len(), 18);
        assert!(m.records.iter().all(|r| r.state == "成功"));
        let summary = m.benchmark_html();
        assert_eq!(summary.matches("100%").count(), 6);
        assert_eq!(summary.matches("3/3").count(), 6);
        for (i, r) in m.records.iter().enumerate() {
            assert_eq!(r.config.seed, 42 + (i / 6) as u32);
            assert!(
                r.trace
                    .windows(2)
                    .map(|w| (w[1] - w[0]).length())
                    .sum::<f64>()
                    > 0.0
            );
        }
        m.seek(0, 3);
        m.advance(false);
        assert_eq!(m.benchmark_html(), summary);
        assert!(m.replay_options().contains("seed 42"));
        m.start_benchmark(config);
        for _ in 0..2000 {
            m.advance(false);
        }
        assert_eq!(m.benchmark_html(), summary);
        // A failed trial contributes to the denominator but not either mean.
        m.records[0].state = "接触";
        let mixed = m.benchmark_html();
        let expected_time = (f64::from(m.records[6].frames.last().unwrap().tick)
            + f64::from(m.records[12].frames.last().unwrap().tick))
            * DT
            / 2.0;
        let expected_distance = [6, 12]
            .iter()
            .map(|i| {
                m.records[*i]
                    .trace
                    .windows(2)
                    .map(|w| (w[1] - w[0]).length())
                    .sum::<f64>()
            })
            .sum::<f64>()
            / 2.0;
        assert!(mixed.contains(&format!(
            "<td>2/3</td><td>67%</td><td>{expected_time:.2}</td><td>{expected_distance:.2}</td>"
        )));
    }
    #[test]
    fn benchmark_failures_wrapping_seeds_and_interruption_are_explicit() {
        let mut m = Manipulation::default();
        let config = ArmConfig {
            task: 3,
            planner: 0,
            controller: 0,
            seed: u32::MAX,
        };
        m.start_benchmark(config);
        m.paused = true;
        m.advance(true);
        assert_eq!(m.records.len(), 1);
        assert!(m.benchmark_html().contains("完了 1/18"));
        m.stop();
        assert!(m.benchmark_html().contains("中断"));
        assert_eq!(m.records.len(), 1);
        m.start_benchmark(config);
        m.advance(false);
        assert_eq!(m.records.len(), 18);
        assert_eq!(m.records[6].config.seed, 0);
        assert_eq!(m.records[12].config.seed, 1);
        let html = m.benchmark_html();
        assert_eq!(html.matches("0/3").count(), 6);
        assert_eq!(html.matches("0%").count(), 6);
        assert_eq!(html.matches("<td>—</td>").count(), 12);
        m.start(config, true);
        assert!(m.benchmark_html().is_empty());
        m.advance(false);
        assert_eq!(m.records.len(), 6);
    }
    use super::*;
    #[test]
    fn g1_arm_uses_urdf_limits_and_real_ik_and_rejects_unreachable_targets() {
        let arm = Arm::load();
        assert_eq!(arm.viewer_order.len(), 23);
        let q = arm.known_pose(-0.85, 0.2, 0.7);
        let p = arm.tip(&q);
        println!(
            "pick {p:?}, home {:?}",
            arm.tip(&vec![0.0; arm.model.dof()])
        );
        let solved = arm.solve(p, &vec![0.0; arm.model.dof()]).unwrap();
        let mut viewer = crate::humanoid::G1Scene::load().unwrap();
        viewer
            .navigation_frame(
                crate::humanoid::G1Input {
                    forward: 0.0,
                    turn: 0.0,
                    reset: true,
                    targets_rad: arm.viewer_order.iter().map(|i| solved[*i]).collect(),
                },
                false,
                (0.0, 0.0, 0.0),
            )
            .unwrap();
        assert!((viewer.left_tool_position() - arm.tip(&solved)).length() < 1e-9);
        assert!((arm.tip(&solved) - p).length() < 0.003);
        assert!(arm.solve(Vec3::new(2.0, 2.0, 2.0), &q).is_none());
        for (i, limit) in arm.model.joint_limits().iter().enumerate() {
            assert!(solved[i] >= limit.lower && solved[i] <= limit.upper);
        }
    }
    #[test]
    fn rrt_routes_a_blocked_transfer_and_repeats_the_same_result() {
        let config = ArmConfig {
            task: 2,
            planner: 2,
            controller: 0,
            seed: 42,
        };
        let mut arm = Arm::load();
        let mut r = Run::new(&mut arm, config);
        for _ in 0..LIMIT {
            if r.stage == 1 || r.state != "実行中" {
                break;
            }
            r.tick(&mut arm);
        }
        assert_eq!(r.state, "実行中");
        assert_eq!(r.stage, 1);
        assert!(r.iterations > 0);
        let route: Vec<_> = std::iter::once(r.q.clone())
            .chain(r.joint_goal.clone())
            .chain(r.route.iter().cloned())
            .collect();
        let direct = MotionPlanRequest::new(
            r.q.clone(),
            GoalConstraint::joint(route.last().unwrap().clone()),
            PlanningOptions {
                collision_check_steps: 24,
                ..PlanningOptions::default()
            },
        )
        .with_group("left_arm");
        assert!(JointInterpolationPlanner::new()
            .plan(&arm.planning, &direct)
            .is_err());
        assert!(route
            .windows(2)
            .all(|w| arm.planning.is_motion_valid(&w[0], &w[1], 24).unwrap()));
        for _ in 0..LIMIT {
            if r.state != "実行中" {
                break;
            }
            r.tick(&mut arm);
        }
        assert_eq!(r.state, "成功");
        assert!((r.object - r.place).length() < 0.025);
        let mut repeated = Run::new(&mut arm, config);
        for _ in 0..LIMIT {
            if repeated.state != "実行中" {
                break;
            }
            repeated.tick(&mut arm);
        }
        assert_eq!(repeated.frames, r.frames);
    }
    #[test]
    fn rrt_rejects_colliding_endpoints_without_claiming_success() {
        let mut arm = Arm::load();
        let start = vec![0.0; arm.model.dof()];
        let goal = arm.known_pose(-0.85, 0.2, 0.7);
        arm.configure_scene(Some(arm.tip(&start)), None);
        assert_eq!(
            arm.search(&start, &goal, 42).unwrap_err().label,
            "開始姿勢が衝突"
        );
        arm.configure_scene(Some(arm.tip(&goal)), None);
        assert_eq!(
            arm.search(&start, &goal, 42).unwrap_err().label,
            "目標姿勢が衝突"
        );
    }
    #[test]
    fn manipulation_matrix_completes_and_replay_does_not_change_results() {
        let mut m = Manipulation::default();
        for task in 0..4 {
            m.start(
                ArmConfig {
                    task,
                    planner: 0,
                    controller: 0,
                    seed: 42,
                },
                true,
            );
            for _ in 0..8000 {
                m.advance(false);
                if m.records.len() == 6 {
                    break;
                }
            }
            assert_eq!(m.records.len(), 6);
            for r in &m.records {
                println!(
                    "task {task}, {} {}: {} {}",
                    r.config.planner, r.config.controller, r.state, r.error_m
                );
                if task <= 1 {
                    assert_eq!(r.state, "成功");
                    assert!(r.error_m < 0.025);
                }
                if task == 3 {
                    assert_eq!(r.state, "IK失敗");
                }
                if task == 2 && r.config.planner == 2 {
                    assert_eq!(r.state, "成功");
                    assert!(r.frames.last().unwrap().iterations > 0);
                }
                if task == 2 && r.config.planner < 2 {
                    assert_eq!(r.state, "腕が障害物に接触");
                }
            }
            assert!(m.active());
            assert!(!m.status().is_empty());
            assert!(!m.replay_options().is_empty());
            let mut scene = RenderScene::default();
            m.overlay(&mut scene);
            assert!(!scene.items.is_empty());
            for record in &m.records {
                for f in &record.frames {
                    for (q, limit) in f.q.iter().zip(m.arm.as_ref().unwrap().model.joint_limits()) {
                        assert!(*q >= limit.lower && *q <= limit.upper);
                    }
                }
            }
            let result = m.results_html();
            m.seek(0, 0);
            assert_eq!(m.frame().unwrap().tick, 0);
            assert_eq!(m.results_html(), result);
            m.stop();
            assert!(m.targets().is_none());
        }
    }
}
