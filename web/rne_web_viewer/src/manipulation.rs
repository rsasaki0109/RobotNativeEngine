//! Deterministic G1 left-arm experiments using RNE's URDF-derived FK and IK.
use rne_ecs::{Entity, World};
use rne_math::{Pose3, Quat, Vec3};
use rne_render::{RenderScene, VisualShape};
use rne_robot::{IkOptions, KinematicModel};
use rne_urdf_import::{parse_urdf, spawn_urdf_robot_with_config, UrdfJointType, UrdfSpawnConfig};
use rne_world::Transform3;
use std::{collections::VecDeque, fmt::Write};

const DT: f64 = 1.0 / 30.0;
const LIMIT: u32 = 1800;
const TASKS: [&str; 4] = [
    "手先リーチ",
    "持ち上げて配置",
    "障害物越しに配置",
    "到達範囲外",
];
const PLANNERS: [&str; 2] = ["関節直結", "上方経由"];
const CONTROLLERS: [&str; 2] = ["速度制限P", "加速度制限P"];
const COLORS: [[f32; 4]; 4] = [
    [0.45, 0.93, 0.69, 1.0],
    [1.0, 0.71, 0.37, 1.0],
    [0.80, 0.61, 1.0, 1.0],
    [0.33, 0.87, 0.96, 1.0],
];

#[derive(Debug)]
struct Arm {
    model: KinematicModel,
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
        Self {
            hand: robot.links["rne_left_tool"],
            elbow: robot.links["left_elbow_link"],
            model,
            active,
            viewer_order,
        }
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
}
#[derive(Debug)]
struct Run {
    config: ArmConfig,
    q: Vec<f64>,
    velocity: Vec<f64>,
    goals: Vec<Vec3>,
    joint_goal: Option<Vec<f64>>,
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
    fn new(arm: &Arm, config: ArmConfig) -> Self {
        let q = vec![0.0; arm.model.dof()];
        let pick = arm.tip(&arm.known_pose(-0.85, 0.15 + f64::from(config.seed % 11) * 0.005, 0.7));
        let place = arm.tip(&arm.known_pose(-1.05, 0.65, 0.8));
        let start = arm.tip(&q);
        let (goals, attach_at, release_at) = if config.task == 3 {
            (vec![Vec3::new(1.8, 1.6, -0.25)], usize::MAX, usize::MAX)
        } else if config.task == 0 {
            (
                if config.planner == 0 {
                    vec![pick]
                } else {
                    vec![(start + pick) * 0.5 + Vec3::Y * 0.12, pick]
                },
                usize::MAX,
                usize::MAX,
            )
        } else if config.planner == 0 {
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
    fn plan(&mut self, arm: &Arm) {
        self.joint_goal = arm.solve(self.goals[self.stage], &self.q);
        if self.joint_goal.is_none() {
            self.state = "IK失敗";
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
        });
    }
    fn tick(&mut self, arm: &Arm) {
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
                let tip = arm.tip(&q);
                let elbow = arm.position(&q, arm.elbow);
                for j in 0..=8 {
                    let p = elbow + (tip - elbow) * (f64::from(j) / 8.0);
                    let delta = (p - center).abs() - Vec3::splat(0.025);
                    if delta.max(Vec3::ZERO).length() <= 0.018 {
                        hit = true;
                        break;
                    }
                }
                if self.attached {
                    let object = tip + self.offset;
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
        let tip = arm.tip(&self.q);
        if self.attached {
            self.object = tip + self.offset;
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
        } else if (tip - self.goals[self.stage]).length() < 0.008 {
            if self.stage == self.attach_at {
                self.attached = (tip - self.object).length() <= 0.025;
                if !self.attached {
                    self.state = "把持距離超過";
                }
                self.offset = self.object - tip;
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
}
#[derive(Debug, Default)]
pub(crate) struct Manipulation {
    arm: Option<Arm>,
    run: Option<Run>,
    queue: VecDeque<ArmConfig>,
    records: Vec<Record>,
    pub(crate) paused: bool,
    batch: bool,
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
        if self.arm.is_none() {
            self.arm = Some(Arm::load());
        }
        self.queue.clear();
        self.records.clear();
        self.batch = batch;
        self.paused = false;
        self.replay = None;
        if batch {
            for planner in 0..2 {
                for controller in 0..2 {
                    self.queue.push_back(ArmConfig {
                        planner,
                        controller,
                        ..c
                    });
                }
            }
        } else {
            self.queue.push_back(c);
        }
        self.run = Some(Run::new(
            self.arm.as_ref().unwrap(),
            self.queue.pop_front().unwrap(),
        ));
    }
    pub(crate) fn advance(&mut self, step: bool) {
        if self.replay.is_some() || (self.paused && !step) {
            return;
        }
        for _ in 0..if self.batch && !step { 32 } else { 1 } {
            let (Some(arm), Some(run)) = (&self.arm, &mut self.run) else {
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
            "{} · {} × {} · {} · {:.2} s · 完了段階 {} · {}{}",
            TASKS[c.task],
            PLANNERS[c.planner],
            CONTROLLERS[c.controller],
            f.state,
            f64::from(f.tick) * DT,
            f.stage,
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
    pub(crate) fn results_html(&self) -> String {
        let mut s = String::new();
        for r in &self.records {
            let color = ["#72edb1", "#ffb45e", "#cd9bff", "#55def5"]
                [r.config.planner * 2 + r.config.controller];
            let _ = write!(
                s,
                "<tr><td><span style='color:{color}'>●</span> {} × {}</td><td>{}</td><td>{:.3}</td><td>{}</td><td>{:.2}</td></tr>",
                PLANNERS[r.config.planner],
                CONTROLLERS[r.config.controller],
                r.state,
                r.error_m,
                r.rmse_rad.map_or_else(|| "—".into(), |v| format!("{v:.3}")),
                f64::from(r.frames.last().unwrap().tick) * DT
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
                    "<option value='{i}' data-times='{}'>{} × {} · {}</option>",
                    r.frames
                        .iter()
                        .map(|f| f.tick.to_string())
                        .collect::<Vec<_>>()
                        .join(","),
                    PLANNERS[r.config.planner],
                    CONTROLLERS[r.config.controller],
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
        if r.config.task == 1 || r.config.task == 2 {
            cube(scene, f.object, Vec3::splat(0.045), [1.0, 0.55, 0.1, 1.0]);
            cube(scene, r.place, Vec3::splat(0.065), [0.15, 0.8, 0.3, 0.35]);
        } else {
            cube(
                scene,
                *r.goals.last().unwrap(),
                Vec3::splat(0.045),
                [0.1, 0.9, 0.4, 1.0],
            );
        }
        if let Some(b) = r.blocker {
            cube(scene, b, Vec3::splat(0.05), [0.7, 0.25, 0.1, 1.0]);
        }
        for (index, record) in self.records.iter().enumerate() {
            if self.replay.is_some_and(|(i, _)| i != index) {
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
        cube(scene, f.tip, Vec3::splat(0.018), [0.9, 0.95, 1.0, 1.0]);
    }
}
#[cfg(test)]
mod tests {
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
                if m.records.len() == 4 {
                    break;
                }
            }
            assert_eq!(m.records.len(), 4);
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
