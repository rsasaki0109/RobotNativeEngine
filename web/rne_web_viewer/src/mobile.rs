//! Navigation and arm manipulation composed into a deterministic delivery episode.
use crate::experiment::{Config, NavigationLeg};
use crate::manipulation::{
    world_point, ArmConfig, MobileArm, CONTROLLERS as ARM_CONTROLLERS, PLANNERS as ARM_PLANNERS,
};
use crate::scene_config::BoxSpec;
use rne_math::{Quat, Vec3};
use rne_render::{RenderScene, VisualShape};
use rne_world::Transform3;
use std::{collections::VecDeque, fmt::Write, sync::Arc};

const DT: f64 = 1.0 / 30.0;
const STAGES: [&str; 4] = [
    "棚へ移動・整列",
    "把持・持ち上げ",
    "配置先へ運搬・整列",
    "配置",
];
const PICK_DOCK: (f64, f64, f64) = (1.4, 0.0, 0.0);
const PLACE_DOCK: (f64, f64, f64) = (-1.0, -1.3, 0.0);
const PLANNERS: [&str; 2] = ["直線", "A*"];
const CONTROLLERS: [&str; 2] = ["Pure Pursuit", "DWA"];

#[derive(Clone, Copy, Debug)]
pub(crate) struct MobileConfig {
    pub(crate) nav: Config,
    pub(crate) arm: ArmConfig,
}

#[derive(Clone, Debug, PartialEq)]
struct Frame {
    tick: u32,
    pose: (f64, f64, f64),
    joints: Vec<f64>,
    object: Vec3,
    tip: Vec3,
    attached: bool,
    stage: usize,
    state: &'static str,
    stage_ticks: [u32; 4],
    base_travel_m: f64,
    hand_travel_m: f64,
    path: Arc<Vec<Vec3>>,
}

#[derive(Debug)]
struct Episode {
    config: MobileConfig,
    arm: MobileArm,
    nav: NavigationLeg,
    docking: bool,
    frame: Frame,
    frames: Vec<Frame>,
    pick: Vec3,
    place: Vec3,
    obstacles: Vec<BoxSpec>,
    surfaces: Vec<(Vec3, Vec3)>,
    recorded: bool,
}
impl Episode {
    fn new(config: MobileConfig) -> Self {
        let arm = MobileArm::new(config.arm);
        let pick = world_point(PICK_DOCK, arm.pick_target(config.nav.seed));
        let place = world_point(PLACE_DOCK, arm.place_target());
        let obstacles = vec![BoxSpec {
            x_m: 0.7,
            z_m: 0.0,
            width_m: 0.22,
            depth_m: 0.55,
            appear_step: 0,
        }];
        let surfaces = vec![
            (
                pick + Vec3::X * 0.09 - Vec3::Y * 0.05,
                Vec3::new(0.10, 0.015, 0.10),
            ),
            (
                place + Vec3::X * 0.09 - Vec3::Y * 0.05,
                Vec3::new(0.10, 0.015, 0.10),
            ),
            (Vec3::new(0.7, 0.3, 0.0), Vec3::new(0.11, 0.3, 0.275)),
        ];
        let nav = NavigationLeg::new(
            config.nav,
            (0.0, 0.0, 0.0),
            (PICK_DOCK.0, PICK_DOCK.1),
            obstacles.clone(),
        );
        let frame = Frame {
            tick: 0,
            pose: (0.0, 0.0, 0.0),
            joints: arm.targets(),
            object: pick,
            tip: arm.tip((0.0, 0.0, 0.0)),
            attached: false,
            stage: 0,
            state: "実行中",
            stage_ticks: [0; 4],
            base_travel_m: 0.0,
            hand_travel_m: 0.0,
            path: Arc::new(nav.path()),
        };
        Self {
            config,
            arm,
            nav,
            docking: false,
            frames: vec![frame.clone()],
            frame,
            pick,
            place,
            obstacles,
            surfaces,
            recorded: false,
        }
    }
    fn tick(&mut self) {
        if self.frame.state != "実行中" {
            return;
        }
        let old = self.frame.clone();
        self.frame.tick += 1;
        self.frame.stage_ticks[self.frame.stage] += 1;
        if self.frame.stage == 0 || self.frame.stage == 2 {
            if !self.docking {
                self.nav.tick();
                self.frame.pose = self.nav.pose();
                match self.nav.outcome() {
                    Some(Ok(())) => self.docking = true,
                    Some(Err(reason)) => self.frame.state = reason,
                    None => {}
                }
            }
            if self.frame.stage == 2 {
                self.frame.object = self.arm.object(self.frame.pose);
            }
            if self.docking && self.frame.state == "実行中" {
                // Dock heading uses bounded angular motion; position is never snapped.
                let yaw = self.frame.pose.2;
                let error = (yaw + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                    - std::f64::consts::PI;
                self.frame.pose.2 -= error.clamp(-0.8 * DT, 0.8 * DT);
                if self.frame.stage == 2 {
                    self.frame.object = self.arm.object(self.frame.pose);
                }
                if error.abs() <= 0.005 {
                    self.frame.stage += 1;
                    let grasp = self.frame.stage == 1;
                    let target = if grasp {
                        self.pick + Vec3::Y * 0.07
                    } else {
                        self.place
                    };
                    self.arm.begin(
                        self.config.arm,
                        self.frame.pose,
                        self.frame.object,
                        target,
                        grasp,
                        &self.surfaces,
                    );
                }
            }
        } else {
            self.arm.tick();
            self.frame.object = self.arm.object(self.frame.pose);
            self.frame.attached = self.arm.attached();
            match self.arm.outcome() {
                Some(Ok(())) if self.frame.stage == 1 => {
                    if !self.frame.attached {
                        self.frame.state = "把持できていない";
                    } else {
                        self.frame.stage = 2;
                        self.docking = false;
                        let extra = (self.arm.envelope_m() - 0.28).max(0.0);
                        let obstacles = self
                            .obstacles
                            .iter()
                            .map(|o| BoxSpec {
                                width_m: o.width_m + 2.0 * extra,
                                depth_m: o.depth_m + 2.0 * extra,
                                ..o.clone()
                            })
                            .collect();
                        self.nav = NavigationLeg::new(
                            self.config.nav,
                            self.frame.pose,
                            (PLACE_DOCK.0, PLACE_DOCK.1),
                            obstacles,
                        );
                        self.frame.path = Arc::new(self.nav.path());
                    }
                }
                Some(Ok(())) => {
                    if self.frame.attached {
                        self.frame.state = "物体が未解放";
                    } else if (self.frame.object - self.place).length() > 0.025 {
                        self.frame.state = "配置誤差超過";
                    } else {
                        self.frame.stage = 4;
                        self.frame.state = "成功";
                    }
                }
                Some(Err(reason)) => self.frame.state = reason,
                None => {}
            }
        }
        self.frame.joints = self.arm.targets();
        self.frame.tip = self.arm.tip(self.frame.pose);
        self.frame.base_travel_m +=
            (self.frame.pose.0 - old.pose.0).hypot(self.frame.pose.1 - old.pose.1);
        self.frame.hand_travel_m += (self.frame.tip - old.tip).length();
        if self.frame.state == "実行中" && self.frame.tick >= 7200 {
            self.frame.state = "全体時間切れ";
        }
        if self.frame.tick.is_multiple_of(3)
            || old.stage != self.frame.stage
            || old.attached != self.frame.attached
            || self.frame.state != "実行中"
        {
            self.frames.push(self.frame.clone());
        }
    }
}

#[derive(Debug)]
struct Record {
    config: MobileConfig,
    frames: Vec<Frame>,
    pick: Vec3,
    place: Vec3,
}

#[derive(Debug, Default)]
pub(crate) struct MobileTask {
    episode: Option<Episode>,
    queue: VecDeque<MobileConfig>,
    records: Vec<Record>,
    batch: bool,
    pub(crate) paused: bool,
    pub(crate) replay: Option<(usize, u32)>,
}
impl MobileTask {
    pub(crate) fn active(&self) -> bool {
        self.episode.is_some()
    }
    pub(crate) fn stop(&mut self) {
        self.episode = None;
        self.queue.clear();
        self.paused = false;
        self.replay = None;
    }
    pub(crate) fn start(&mut self, c: MobileConfig, batch: bool) {
        self.queue.clear();
        self.records.clear();
        self.paused = false;
        self.replay = None;
        self.batch = batch;
        if batch {
            for planner in 0..2 {
                for controller in 0..2 {
                    self.queue.push_back(MobileConfig {
                        nav: Config {
                            planner,
                            controller,
                            ..c.nav
                        },
                        ..c
                    });
                }
            }
        } else {
            self.queue.push_back(c);
        }
        self.episode = Some(Episode::new(self.queue.pop_front().unwrap()));
    }
    pub(crate) fn advance(&mut self, step: bool) {
        if self.replay.is_some() || (self.paused && !step) {
            return;
        }
        for _ in 0..if self.batch && !step { 32 } else { 1 } {
            let Some(e) = self.episode.as_mut() else {
                return;
            };
            e.tick();
            if e.frame.state != "実行中" && !e.recorded {
                self.records.push(Record {
                    config: e.config,
                    frames: e.frames.clone(),
                    pick: e.pick,
                    place: e.place,
                });
                e.recorded = true;
            }
            if e.recorded {
                if let Some(c) = self.queue.pop_front() {
                    self.episode = Some(Episode::new(c));
                } else {
                    return;
                }
            }
        }
    }
    fn frame(&self) -> Option<&Frame> {
        if let Some((i, tick)) = self.replay {
            let frames = &self.records.get(i)?.frames;
            return frames
                .iter()
                .rev()
                .find(|f| f.tick <= tick)
                .or_else(|| frames.first());
        }
        self.episode.as_ref().map(|e| &e.frame)
    }
    pub(crate) fn pose(&self) -> Option<(f64, f64, f64)> {
        self.frame().map(|f| f.pose)
    }
    pub(crate) fn targets(&self) -> Option<Vec<f64>> {
        self.frame().map(|f| f.joints.clone())
    }
    pub(crate) fn seek(&mut self, index: usize, tick: u32) {
        if self.queue.is_empty()
            && self.episode.as_ref().is_some_and(|e| e.recorded)
            && index < self.records.len()
        {
            self.replay = Some((index, tick));
        }
    }
    pub(crate) fn status(&self) -> String {
        let Some(f) = self.frame() else {
            return "棚から取り、運んで置くタスク".into();
        };
        format!(
            "{} · {} · {:.2} s · {}{}",
            if f.stage == 4 {
                "全4段階完了"
            } else {
                STAGES[f.stage]
            },
            f.state,
            f64::from(f.tick) * DT,
            if f.attached {
                "物体保持中"
            } else {
                "未保持"
            },
            if self.replay.is_some() {
                " · リプレイ"
            } else if self.paused {
                " · 一時停止"
            } else {
                ""
            }
        )
    }
    pub(crate) fn summary_html(&self) -> String {
        if !self.active() && self.records.is_empty() {
            return String::new();
        }
        let expected = if self.batch { 4 } else { 1 };
        let state = if self.records.len() == expected {
            "評価完了"
        } else if !self.active() {
            "中断"
        } else if self.paused {
            "一時停止"
        } else {
            "実行中"
        };
        let mut s=format!("<p>{state} · 完了 {}/{expected}試行。段階の成功率は、その段階まで到達した完了試行のみで計算します。</p><table><thead><tr><th>段階</th><th>成功 / 到達</th><th>成功率</th><th>成功時平均 s</th></tr></thead><tbody>",self.records.len());
        for (i, name) in STAGES.iter().enumerate() {
            let reached = self
                .records
                .iter()
                .filter(|r| r.frames.last().unwrap().stage >= i)
                .count();
            let success: Vec<_> = self
                .records
                .iter()
                .filter_map(|r| {
                    let f = r.frames.last().unwrap();
                    (f.stage > i).then_some(f)
                })
                .collect();
            let rate = if reached == 0 {
                "—".into()
            } else {
                format!("{:.0}%", success.len() as f64 / reached as f64 * 100.0)
            };
            let mean = if success.is_empty() {
                "—".into()
            } else {
                format!(
                    "{:.2}",
                    success
                        .iter()
                        .map(|f| f64::from(f.stage_ticks[i]) * DT)
                        .sum::<f64>()
                        / success.len() as f64
                )
            };
            let _ = write!(
                s,
                "<tr><td>{name}</td><td>{}/{reached}</td><td>{rate}</td><td>{mean}</td></tr>",
                success.len()
            );
        }
        s.push_str("</tbody></table>");
        s
    }
    pub(crate) fn results_html(&self) -> String {
        let mut html = String::new();
        for r in &self.records {
            let f = r.frames.last().unwrap();
            let times: Vec<_> = f
                .stage_ticks
                .iter()
                .enumerate()
                .map(|(i, t)| {
                    if i <= f.stage {
                        format!(
                            "{:.2}{}",
                            f64::from(*t) * DT,
                            if i == f.stage && f.state != "成功" {
                                " (失敗)"
                            } else {
                                ""
                            }
                        )
                    } else {
                        "—".into()
                    }
                })
                .collect();
            let _=write!(html,"<tr><td>{} × {} · 腕 {} × {} · seed {}</td><td>{} · {}/4完了</td><td>{}</td><td>{:.2}</td><td>{:.2}</td><td>{:.3}</td></tr>",PLANNERS[r.config.nav.planner],CONTROLLERS[r.config.nav.controller],ARM_PLANNERS[r.config.arm.planner],ARM_CONTROLLERS[r.config.arm.controller],r.config.nav.seed,f.state,f.stage,times.join(" / "),f64::from(f.tick)*DT,f.base_travel_m,(f.object-r.place).length());
        }
        html
    }
    pub(crate) fn replay_options(&self) -> String {
        if !self.queue.is_empty() || !self.episode.as_ref().is_some_and(|e| e.recorded) {
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
                    PLANNERS[r.config.nav.planner],
                    CONTROLLERS[r.config.nav.controller],
                    r.frames.last().unwrap().state
                )
            })
            .collect()
    }
    pub(crate) fn csv(&self) -> String {
        if self.records.is_empty() {
            return String::new();
        }
        let mut csv=String::from("schema_version,task,batch_state,expected_trials,completed_trials,seed,nav_planner,nav_controller,arm_planner,arm_controller,result,completed_stages,approach_s,grasp_s,carry_s,place_s,total_s,base_travel_m,hand_travel_m,final_error_m\r\n");
        let expected = if self.batch { 4 } else { 1 };
        let state = if self.records.len() == expected {
            "completed"
        } else if !self.active() {
            "interrupted"
        } else if self.paused {
            "paused"
        } else {
            "running"
        };
        for r in &self.records {
            let f = r.frames.last().unwrap();
            let times: Vec<_> = f
                .stage_ticks
                .iter()
                .enumerate()
                .map(|(i, t)| {
                    if i <= f.stage {
                        format!("{:.6}", f64::from(*t) * DT)
                    } else {
                        String::new()
                    }
                })
                .collect();
            let _ = write!(
                csv,
                "1,shelf_delivery,{state},{expected},{},{},{},{},{},{},{},{},{},{:.6},{:.6},{:.6},{:.6}\r\n",
                self.records.len(),
                r.config.nav.seed,
                r.config.nav.planner,
                r.config.nav.controller,
                r.config.arm.planner,
                r.config.arm.controller,
                f.state,
                f.stage,
                times.join(","),
                f64::from(f.tick) * DT,
                f.base_travel_m,
                f.hand_travel_m,
                (f.object - r.place).length()
            );
        }
        csv
    }
    pub(crate) fn overlay(&self, scene: &mut RenderScene) {
        let (Some(e), Some(f)) = (&self.episode, self.frame()) else {
            return;
        };
        let selected = self.replay.and_then(|(i, _)| self.records.get(i));
        let pick = selected.map_or(e.pick, |r| r.pick);
        let place = selected.map_or(e.place, |r| r.place);
        let cube = |scene: &mut RenderScene, p, size, color| {
            scene.items.push(RenderScene::item_from_visual(
                Transform3::from_translation_rotation(p, Quat::IDENTITY),
                VisualShape::Box { size_m: size },
                color,
                Transform3::IDENTITY,
            ))
        };
        for p in [pick, place] {
            cube(
                scene,
                p + Vec3::X * 0.09 - Vec3::Y * 0.05,
                Vec3::new(0.2, 0.03, 0.2),
                [0.48, 0.57, 0.64, 1.0],
            );
            cube(
                scene,
                Vec3::new(p.x + 0.09, p.y * 0.5 - 0.0325, p.z),
                Vec3::new(0.04, p.y - 0.065, 0.04),
                [0.25, 0.30, 0.35, 1.0],
            );
        }
        for o in &e.obstacles {
            cube(
                scene,
                Vec3::new(o.x_m, 0.3, o.z_m),
                Vec3::new(o.width_m, 0.6, o.depth_m),
                [0.65, 0.3, 0.12, 1.0],
            );
        }
        cube(scene, place, Vec3::splat(0.065), [0.15, 0.8, 0.3, 0.35]);
        cube(scene, f.object, Vec3::splat(0.045), [1.0, 0.55, 0.1, 1.0]);
        for p in f.path.iter().step_by(2) {
            cube(scene, *p, Vec3::splat(0.022), [0.2, 0.7, 1.0, 0.8]);
        }
        let frames = selected.map_or(e.frames.as_slice(), |r| r.frames.as_slice());
        for past in frames.iter().filter(|p| p.tick <= f.tick).step_by(8) {
            cube(
                scene,
                Vec3::new(past.pose.0, 0.02, past.pose.1),
                Vec3::splat(0.025),
                [0.8, 0.8, 0.3, 1.0],
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn config() -> MobileConfig {
        MobileConfig {
            nav: Config {
                planner: 1,
                seed: 42,
                ..Config::default()
            },
            arm: ArmConfig {
                task: 1,
                planner: 2,
                controller: 0,
                seed: 42,
            },
        }
    }
    fn finish(task: &mut MobileTask) {
        for _ in 0..1000 {
            task.advance(false);
        }
    }
    #[test]
    fn delivery_matrix_records_real_stage_outcomes_and_continuous_payload() {
        let mut task = MobileTask::default();
        task.start(config(), true);
        finish(&mut task);
        assert_eq!(task.records.len(), 4);
        for (i, r) in task.records.iter().enumerate() {
            let end = r.frames.last().unwrap();
            assert_eq!(end.stage_ticks.iter().sum::<u32>(), end.tick);
            if i < 2 {
                assert_eq!(end.state, "計画失敗");
                assert_eq!(end.stage, 0);
                assert_eq!(&end.stage_ticks[1..], &[0, 0, 0]);
                assert_eq!(end.object, r.pick);
            } else {
                assert_eq!(end.state, "成功");
                assert_eq!(end.stage, 4);
                assert!(!end.attached);
                assert!((end.object - r.place).length() < 0.025);
                assert!(end.stage_ticks.iter().all(|t| *t > 0));
                let carry: Vec<_> = r.frames.iter().filter(|f| f.stage == 2).collect();
                assert!(carry.len() > 10);
                let local = |f: &Frame| {
                    Quat::from_rotation_y(f.pose.2).inverse()
                        * (f.object - Vec3::new(f.pose.0, 0.0, f.pose.1))
                };
                let held = local(carry[0]);
                for f in carry {
                    assert!(f.attached);
                    assert!((local(f) - held).length() < 1e-9);
                }
                for f in &r.frames {
                    assert_eq!(f.joints.len(), 23);
                }
                for w in r.frames.windows(2) {
                    let dt = f64::from(w[1].tick - w[0].tick) * DT;
                    assert!(
                        (w[1].pose.0 - w[0].pose.0).hypot(w[1].pose.1 - w[0].pose.1)
                            <= 0.56 * dt + 1e-9
                    );
                }
                let grasp = r.frames.iter().find(|f| f.stage == 1).unwrap();
                assert!((grasp.pose.0 - PICK_DOCK.0).hypot(grasp.pose.1 - PICK_DOCK.1) <= 0.035);
                assert!(grasp.pose.2.abs() < 1e-8);
            }
        }
        let summary = task.summary_html();
        assert!(summary.contains("2/4"));
        assert_eq!(summary.matches("2/2").count(), 3);
        assert_eq!(summary.matches("100%").count(), 3);
        assert_eq!(task.csv().lines().count(), 5);
        for row in task.csv().lines().skip(1) {
            assert_eq!(row.split(',').count(), 20);
        }
    }
    #[test]
    fn delivery_replay_freezes_metrics_and_restores_full_world_state() {
        let mut task = MobileTask::default();
        task.start(config(), true);
        assert!(task.active());
        assert!(task.csv().is_empty());
        task.paused = true;
        task.advance(false);
        assert_eq!(task.frame().unwrap().tick, 0);
        task.advance(true);
        assert_eq!(task.records.len(), 1);
        task.seek(0, 0);
        assert!(task.replay.is_none());
        task.paused = false;
        finish(&mut task);
        let csv = task.csv();
        let results = task.results_html();
        let summary = task.summary_html();
        assert!(!task.replay_options().is_empty());
        let frames = task.records[2].frames.clone();
        let mut viewer = crate::humanoid::G1Scene::load().unwrap();
        for f in frames.iter().step_by(13) {
            task.seek(2, f.tick);
            assert_eq!(task.frame().unwrap(), f);
            assert_eq!(task.pose(), Some(f.pose));
            assert_eq!(task.targets().unwrap(), f.joints);
            viewer
                .recorded_frame(
                    crate::humanoid::G1Input {
                        forward: 0.0,
                        turn: 0.0,
                        reset: false,
                        targets_rad: f.joints.clone(),
                    },
                    f.pose,
                )
                .unwrap();
            assert!((viewer.left_tool_position() - f.tip).length() < 1e-9);
            task.advance(false);
            assert_eq!(task.frame().unwrap(), f);
            assert!(task.status().contains("リプレイ"));
            assert_eq!(task.csv(), csv);
            assert_eq!(task.results_html(), results);
            assert_eq!(task.summary_html(), summary);
        }
        let mut scene = RenderScene::default();
        task.overlay(&mut scene);
        assert!(!scene.items.is_empty());
        task.stop();
        assert!(!task.active());
        assert!(task.pose().is_none());
        assert!(task.targets().is_none());
    }
    #[test]
    fn delivery_failures_prevent_later_phases_and_partial_batches_are_marked() {
        let mut task = MobileTask::default();
        task.start(config(), true);
        task.paused = true;
        task.advance(true);
        task.stop();
        let csv = task.csv();
        assert_eq!(csv.lines().count(), 2);
        assert!(csv.contains("shelf_delivery,interrupted,4,1,"));
        assert!(task.summary_html().contains("中断"));
        task.start(config(), false);
        let e = task.episode.as_mut().unwrap();
        e.frame.pose = PICK_DOCK;
        e.nav = NavigationLeg::new(config().nav, PICK_DOCK, (PICK_DOCK.0, PICK_DOCK.1), vec![]);
        e.pick = Vec3::new(3.0, 2.0, 0.0);
        e.frame.object = e.pick;
        for _ in 0..3 {
            task.advance(false);
        }
        let end = task.records[0].frames.last().unwrap();
        assert_eq!(end.state, "IK失敗");
        assert_eq!(end.stage, 1);
        assert_eq!(&end.stage_ticks[2..], &[0, 0]);
        assert!(!end.attached);
        assert!(task.results_html().contains("失敗"));
    }
    #[test]
    fn delivery_compares_all_six_captured_arm_methods() {
        for planner in 0..3 {
            for controller in 0..2 {
                let mut c = config();
                c.nav.controller = 1;
                c.arm.planner = planner;
                c.arm.controller = controller;
                let mut task = MobileTask::default();
                task.start(c, true);
                finish(&mut task);
                assert_eq!(task.records.len(), 4);
                for r in &task.records {
                    assert_eq!(r.config.arm.planner, planner);
                    assert_eq!(r.config.arm.controller, controller);
                }
                let end = task.records[3].frames.last().unwrap();
                let expected = [
                    ["腕が障害物に接触", "腕が障害物に接触"],
                    ["成功", "腕が障害物に接触"],
                    ["成功", "成功"],
                ][planner][controller];
                assert_eq!(end.state, expected, "arm {planner}/{controller}");
                assert_eq!(end.stage, if expected == "成功" { 4 } else { 1 });
                assert_eq!(end.stage_ticks.iter().sum::<u32>(), end.tick);
            }
        }
    }
    #[test]
    fn delivery_repeats_the_same_seeded_episode() {
        let mut a = MobileTask::default();
        let mut b = MobileTask::default();
        a.start(config(), true);
        b.start(config(), true);
        finish(&mut a);
        finish(&mut b);
        assert_eq!(a.csv(), b.csv());
        for (a, b) in a.records.iter().zip(&b.records) {
            assert_eq!(a.frames, b.frames);
        }
    }
}
