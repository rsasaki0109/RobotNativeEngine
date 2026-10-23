//! winit application loop for the browser viewer.

use crate::experiment::{Config, Experiment};
use crate::humanoid::{G1Input, G1Scene};
use crate::manipulation::{ArmConfig, Manipulation};
use crate::mobile::{MobileConfig, MobileTask};
use crate::scene_config::{pick_floor, SceneSpec};
use rne_render_wgpu::{CameraOrbit, InteractiveViewer, ViewerError};
use std::sync::Arc;
use winit::application::ApplicationHandler;
use winit::dpi::PhysicalPosition;
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, EventLoop, EventLoopProxy};
use winit::platform::web::EventLoopExtWebSys;
use winit::window::{Window, WindowId};

const CLEAR_COLOR: [f32; 4] = [0.28, 0.33, 0.37, 1.0];

fn status(message: &str) {
    if let Some(element) = web_sys::window()
        .and_then(|window| window.document())
        .and_then(|document| document.get_element_by_id("engine-status"))
    {
        element.set_text_content(Some(message));
    }
}

/// GPU initialization completion event for wasm async startup.
struct ViewerInitEvent(Result<InteractiveViewer, ViewerError>);

#[derive(Default)]
enum InitState {
    #[default]
    Pending,
    Ready {
        viewer: Box<InteractiveViewer>,
        scene: Box<G1Scene>,
    },
    Failed(String),
}

/// Runs the viewer event loop until the page is closed.
pub fn run() {
    let event_loop = EventLoop::<ViewerInitEvent>::with_user_event()
        .build()
        .expect("create event loop");
    let proxy = event_loop.create_proxy();
    let app = App {
        proxy: Some(proxy),
        orbit: CameraOrbit {
            yaw_rad: 0.75,
            pitch_rad: 1.0,
            distance_m: 4.5,
            ..CameraOrbit::default()
        },
        ..App::default()
    };
    event_loop.spawn_app(app);
}

#[derive(Default)]
struct App {
    proxy: Option<EventLoopProxy<ViewerInitEvent>>,
    window: Option<Arc<Window>>,
    init: InitState,
    pending_scene: Option<G1Scene>,
    render_size: Option<(u32, u32)>,
    orbit: CameraOrbit,
    reset_counter: u32,
    wave_counter: u32,
    mission_counter: u32,
    experiment: Experiment,
    experiment_counters: [u32; 5],
    manipulation: Manipulation,
    manipulation_counters: [u32; 5],
    mobile: MobileTask,
    mobile_counters: [u32; 5],
    rendered_results: String,
    scene_revision: u32,
    floor_counter: u32,
    scene_error: Option<String>,
    dragging: bool,
    last_cursor: Option<PhysicalPosition<f64>>,
}

impl ApplicationHandler<ViewerInitEvent> for App {
    fn resumed(&mut self, _event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }

        let scene = match G1Scene::load() {
            Ok(scene) => scene,
            Err(error) => {
                status(&format!("G1読み込みエラー: {error}"));
                self.init = InitState::Failed(error);
                return;
            }
        };
        self.orbit.focus = scene.focus();
        self.pending_scene = Some(scene);

        #[cfg_attr(
            not(target_arch = "wasm32"),
            expect(unused_mut, reason = "wasm attaches canvas")
        )]
        let mut attributes = Window::default_attributes().with_title("RNE G1 Playground");

        #[cfg(target_arch = "wasm32")]
        {
            use wasm_bindgen::JsCast;
            use winit::platform::web::WindowAttributesExtWebSys;
            let canvas = web_sys::window()
                .and_then(|window| window.document())
                .and_then(|document| document.get_element_by_id("canvas"))
                .and_then(|element| element.dyn_into::<web_sys::HtmlCanvasElement>().ok())
                .expect("canvas element with id `canvas`");
            attributes = attributes.with_canvas(Some(canvas));
        }

        let window = Arc::new(
            _event_loop
                .create_window(attributes)
                .expect("create window"),
        );
        self.window = Some(window.clone());
        self.init = InitState::Pending;

        let proxy = self
            .proxy
            .clone()
            .expect("event loop proxy should be set before resumed");
        #[cfg(target_arch = "wasm32")]
        wasm_bindgen_futures::spawn_local(async move {
            let result = InteractiveViewer::new_async(window).await;
            let _ = proxy.send_event(ViewerInitEvent(result));
        });
        #[cfg(not(target_arch = "wasm32"))]
        {
            let result = InteractiveViewer::new(window);
            let _ = proxy.send_event(ViewerInitEvent(result));
        }
    }

    fn user_event(&mut self, event_loop: &ActiveEventLoop, event: ViewerInitEvent) {
        let Some(scene) = self.pending_scene.take() else {
            return;
        };

        match event.0 {
            Ok(mut viewer) => {
                viewer.set_environment(crate::lab::lighting());
                status("操作できます · G1 / 23関節");
                self.init = InitState::Ready {
                    viewer: Box::new(viewer),
                    scene: Box::new(scene),
                };
                if let Some(window) = &self.window {
                    window.request_redraw();
                }
            }
            Err(error) => {
                status(&format!("GPU初期化エラー: {error}"));
                self.init = InitState::Failed(error.to_string());
                event_loop.exit();
            }
        }
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(_) => {
                self.render_size = None;
            }
            WindowEvent::MouseInput {
                state,
                button: MouseButton::Left,
                ..
            } => {
                self.dragging = state == ElementState::Pressed;
                if state == ElementState::Released {
                    self.last_cursor = None;
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                if !self.dragging {
                    return;
                }
                if let Some(last) = self.last_cursor {
                    let dx = position.x - last.x;
                    let dy = position.y - last.y;
                    self.orbit.yaw_rad += dx * 0.005;
                    self.orbit.pitch_rad = (self.orbit.pitch_rad + dy * 0.004).clamp(0.15, 1.45);
                }
                self.last_cursor = Some(position);
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let scroll = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y as f64,
                    MouseScrollDelta::PixelDelta(delta) => delta.y * 0.002,
                };
                self.orbit.distance_m = (self.orbit.distance_m - scroll * 0.35).clamp(1.5, 12.0);
            }
            WindowEvent::RedrawRequested => {
                if let Err(error) = self.draw_frame() {
                    status(&format!("描画エラー: {error}"));
                    #[cfg(target_arch = "wasm32")]
                    web_sys::console::error_1(&error.into());
                    #[cfg(not(target_arch = "wasm32"))]
                    eprintln!("render error: {error}");
                    event_loop.exit();
                }
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }
}

impl App {
    fn draw_frame(&mut self) -> Result<(), String> {
        let InitState::Ready { viewer, scene } = &mut self.init else {
            if let InitState::Failed(error) = &self.init {
                return Err(error.clone());
            }
            return Ok(());
        };

        let browser = web_sys::window().ok_or("missing browser window")?;
        let pixel_ratio = browser.device_pixel_ratio();
        let document = browser.document().ok_or("missing document")?;
        let canvas = document
            .get_element_by_id("canvas")
            .ok_or("missing canvas")?;
        let width = (f64::from(canvas.client_width()) * pixel_ratio).max(1.0) as u32;
        let height = (f64::from(canvas.client_height()) * pixel_ratio).max(1.0) as u32;
        if self.render_size != Some((width, height)) {
            viewer.resize(width, height);
            self.render_size = Some((width, height));
        }
        let value = |id: &str| -> Result<f64, String> {
            use wasm_bindgen::JsCast;
            document
                .get_element_by_id(id)
                .and_then(|element| {
                    element
                        .dyn_ref::<web_sys::HtmlInputElement>()
                        .map(|input| input.value())
                        .or_else(|| {
                            element
                                .dyn_ref::<web_sys::HtmlSelectElement>()
                                .map(|select| select.value())
                        })
                })
                .and_then(|text| text.parse::<f64>().ok())
                .filter(|value| value.is_finite())
                .ok_or_else(|| format!("invalid control: {id}"))
        };
        let scene_revision = value("scene-revision")? as u32;
        if scene_revision != self.scene_revision {
            use wasm_bindgen::JsCast;
            let json = document
                .get_element_by_id("scene-json")
                .and_then(|e| e.dyn_into::<web_sys::HtmlInputElement>().ok())
                .map(|e| e.value())
                .unwrap_or_default();
            match SceneSpec::parse(&json) {
                Ok(spec) => {
                    self.experiment.draft = spec;
                    self.scene_error = None;
                }
                Err(error) => {
                    self.scene_error = Some(error);
                }
            }
            if let Some(element) = document.get_element_by_id("scene-native-status") {
                element.set_text_content(self.scene_error.as_deref());
            }
            self.scene_revision = scene_revision;
        }
        self.experiment.editor_visible = value("scene-editor-visible")? == 1.0;
        let floor_counter = value("scene-floor-click")? as u32;
        if floor_counter != self.floor_counter {
            let hit = pick_floor(
                viewer.camera(),
                &self.orbit.camera_transform(),
                value("scene-click-x")?,
                value("scene-click-y")?,
            );
            if let Some(element) = document.get_element_by_id("scene-floor-result") {
                element.set_text_content(Some(&hit.map_or_else(
                    || format!("{floor_counter},error,error"),
                    |p| format!("{floor_counter},{},{}", p.x_m, p.z_m),
                )));
            }
            self.floor_counter = floor_counter;
        }
        let reset_counter = value("control-reset-counter")? as u32;
        let wave_counter = value("control-wave-counter")? as u32;
        let mission_counter = value("control-mission-counter")? as u32;
        let targets_rad = (0..scene.joint_count())
            .map(|index| value(&format!("control-joint-{index}")))
            .collect::<Result<Vec<_>, _>>()?;
        let counters: [u32; 5] = [
            "experiment-run",
            "experiment-pause",
            "experiment-step",
            "experiment-compare",
            "experiment-manual",
        ]
        .map(|id| value(id).map(|v| v as u32))
        .into_iter()
        .collect::<Result<Vec<_>, _>>()?
        .try_into()
        .unwrap();
        let changed: [bool; 5] =
            std::array::from_fn(|i| counters[i] != self.experiment_counters[i]);
        let reset = reset_counter != self.reset_counter;
        let mission = mission_counter != self.mission_counter;
        let intent = value("experiment-intent")? as u32;
        if reset || mission {
            self.manipulation.stop();
            self.mobile.stop();
            self.experiment.stop();
        } else if (changed[0] || changed[3]) && intent != 0 {
            let config = Config {
                task: (value("experiment-task")? as usize).min(7),
                planner: (value("experiment-planner")? as usize).min(1),
                controller: (value("experiment-controller")? as usize).min(1),
                seed: value("experiment-seed")?.clamp(0.0, f64::from(u32::MAX)) as u32,
                recovery: value("experiment-recovery")? == 1.0,
            };
            if config.task == 3 && self.scene_error.is_some() {
                self.experiment.stop();
            } else {
                self.manipulation.stop();
                self.mobile.stop();
                self.experiment.start(config, intent == 2);
            }
        }
        if changed[4] && intent == 0 {
            self.manipulation.stop();
            self.mobile.stop();
            self.experiment.stop();
        }
        if changed[1] && self.experiment.active() {
            self.experiment.paused = !self.experiment.paused;
        }
        if changed[2] && self.experiment.active() {
            self.experiment.paused = true;
        }
        self.experiment.hidden_traces = value("experiment-hidden-traces")?.clamp(0.0, 15.0) as u8;
        if value("nav-replay-enabled")? == 1.0 {
            self.experiment.seek_replay(
                value("nav-replay-record")? as usize,
                value("nav-replay-tick")? as u32,
            );
        } else {
            self.experiment.clear_replay();
        }
        self.experiment.advance(changed[2]);
        let arm_counters: [u32; 5] = [
            "arm-run",
            "arm-pause",
            "arm-step",
            "arm-compare",
            "arm-stop",
        ]
        .map(|id| value(id).map(|v| v as u32))
        .into_iter()
        .collect::<Result<Vec<_>, _>>()?
        .try_into()
        .unwrap();
        let arm_changed: [bool; 5] =
            std::array::from_fn(|i| arm_counters[i] != self.manipulation_counters[i]);
        let arm_intent = value("arm-intent")? as u32;
        if (arm_changed[0] || arm_changed[3]) && arm_intent != 0 && !reset && !mission {
            self.experiment.stop();
            let config = ArmConfig {
                task: (value("arm-task")? as usize).min(3),
                planner: (value("arm-planner")? as usize).min(2),
                controller: (value("arm-controller")? as usize).min(1),
                seed: value("experiment-seed")?.clamp(0.0, f64::from(u32::MAX)) as u32,
            };
            if arm_intent == 3 {
                self.manipulation.start_benchmark(config);
            } else {
                self.manipulation.start(config, arm_intent == 2);
            }
        }
        if arm_changed[4] && arm_intent == 0 {
            self.manipulation.stop();
            self.mobile.stop();
        }
        if arm_changed[1] {
            self.manipulation.replay = None;
            self.manipulation.paused = !self.manipulation.paused;
        }
        if arm_changed[2] {
            self.manipulation.replay = None;
            self.manipulation.paused = true;
        }
        if value("arm-replay-enabled")? == 1.0 {
            self.manipulation.seek(
                value("arm-replay-record")? as usize,
                value("arm-replay-tick")? as u32,
            );
        } else {
            self.manipulation.replay = None;
        }
        self.manipulation.advance(arm_changed[2]);
        let mobile_counters: [u32; 5] = [
            "mobile-run",
            "mobile-pause",
            "mobile-step",
            "mobile-compare",
            "mobile-stop",
        ]
        .map(|id| value(id).map(|v| v as u32))
        .into_iter()
        .collect::<Result<Vec<_>, _>>()?
        .try_into()
        .unwrap();
        let mobile_changed: [bool; 5] =
            std::array::from_fn(|i| mobile_counters[i] != self.mobile_counters[i]);
        let mobile_intent = value("mobile-intent")? as u32;
        if (mobile_changed[0] || mobile_changed[3]) && mobile_intent != 0 && !reset && !mission {
            self.experiment.stop();
            self.manipulation.stop();
            let seed = value("experiment-seed")?.clamp(0.0, f64::from(u32::MAX)) as u32;
            self.mobile.start(
                MobileConfig {
                    scenario: (value("mobile-scenario")? as usize).min(2),
                    nav: Config {
                        task: 3,
                        planner: (value("experiment-planner")? as usize).min(1),
                        controller: (value("experiment-controller")? as usize).min(1),
                        seed,
                        recovery: false,
                    },
                    arm: ArmConfig {
                        task: 1,
                        planner: (value("arm-planner")? as usize).min(2),
                        controller: (value("arm-controller")? as usize).min(1),
                        seed,
                    },
                },
                mobile_intent == 2,
            );
        }
        if mobile_changed[4] && mobile_intent == 0 {
            self.mobile.stop();
        }
        if mobile_changed[1] {
            self.mobile.replay = None;
            self.mobile.paused = !self.mobile.paused;
        }
        if mobile_changed[2] {
            self.mobile.replay = None;
            self.mobile.paused = true;
        }
        if value("mobile-replay-enabled")? == 1.0 {
            self.mobile.seek(
                value("mobile-replay-record")? as usize,
                value("mobile-replay-tick")? as u32,
            );
        } else {
            self.mobile.replay = None;
        }
        self.mobile.advance(mobile_changed[2]);
        self.mobile_counters = mobile_counters;
        self.manipulation_counters = arm_counters;
        self.experiment_counters = counters;
        let mut input = G1Input {
            forward: value("control-forward")?,
            turn: value("control-turn")?,
            targets_rad,
            reset,
        };
        let animate = value("control-motion-enabled")? == 1.0;
        if self.experiment.active() && (changed[0] || changed[3]) {
            scene.play_frame(
                G1Input {
                    forward: 0.0,
                    turn: 0.0,
                    targets_rad: input.targets_rad.clone(),
                    reset: true,
                },
                false,
                false,
                false,
            )?;
        }
        if let Some(targets) = self.manipulation.targets() {
            input.targets_rad = targets;
        }
        if self.manipulation.active() && (arm_changed[0] || arm_changed[3]) {
            scene.play_frame(
                G1Input {
                    forward: 0.0,
                    turn: 0.0,
                    targets_rad: input.targets_rad.clone(),
                    reset: true,
                },
                false,
                false,
                false,
            )?;
        }
        if let Some(targets) = self.mobile.targets() {
            input.targets_rad = targets;
        }
        let mut render_scene = if let Some(pose) = self.mobile.pose() {
            scene.recorded_frame(input, pose)?
        } else if self.manipulation.active() {
            scene.recorded_frame(input, (0.0, 0.0, 0.0))?
        } else if let Some(pose) = self.experiment.pose() {
            if self.experiment.replaying() {
                scene.recorded_frame(input, pose)?
            } else {
                scene.navigation_frame(input, animate, pose)?
            }
        } else {
            scene.play_frame(input, animate, wave_counter != self.wave_counter, mission)?
        };
        self.experiment.overlay(&mut render_scene);
        self.manipulation.overlay(&mut render_scene);
        self.mobile.overlay(&mut render_scene);
        if let Some(body) = document.get_element_by_id("playground") {
            let _ = body.set_attribute(
                "data-experiment-active",
                if self.experiment.active()
                    || self.experiment.editor_visible
                    || self.manipulation.active()
                    || self.mobile.active()
                {
                    "true"
                } else {
                    "false"
                },
            );
        }
        for (id, text) in [
            ("experiment-status", self.experiment.status()),
            ("arm-status", self.manipulation.status()),
            ("mobile-status", self.mobile.status()),
            ("experiment-metrics", self.experiment.metrics()),
            (
                "experiment-comparison",
                self.experiment.comparison_summary(),
            ),
        ] {
            if let Some(element) = document.get_element_by_id(id) {
                element.set_text_content(Some(&text));
            }
        }
        if let Some(element) = document.get_element_by_id("experiment-map") {
            element.set_inner_html(&self.experiment.map_svg());
            let _ = element.set_attribute(
                "viewBox",
                if self.experiment.full_map() {
                    "-4 -4 8 8"
                } else {
                    "-0.6 -2 4.1 4"
                },
            );
        }
        let results = self.experiment.results_html();
        if self.rendered_results != results {
            if let Some(element) = document.get_element_by_id("experiment-records") {
                element.set_inner_html(&results);
                self.rendered_results.clone_from(&results);
            }
        }
        for (prefix, csv) in [
            ("arm", self.manipulation.results_csv()),
            ("mobile", self.mobile.csv()),
        ] {
            if let Some(element) = document.get_element_by_id(&format!("{prefix}-export-data")) {
                if element.text_content().as_deref() != Some(csv.as_str()) {
                    element.set_text_content(Some(&csv));
                }
            }
            if let Some(element) = document.get_element_by_id(&format!("{prefix}-export-button")) {
                if csv.is_empty() {
                    let _ = element.set_attribute("disabled", "");
                } else {
                    let _ = element.remove_attribute("disabled");
                }
            }
        }
        for (id, html) in [
            ("arm-records", self.manipulation.results_html()),
            ("mobile-records", self.mobile.results_html()),
            ("mobile-summary", self.mobile.summary_html()),
            ("mobile-replay-record", self.mobile.replay_options()),
            ("arm-benchmark", self.manipulation.benchmark_html()),
            ("nav-replay-record", self.experiment.replay_options()),
            ("arm-replay-record", self.manipulation.replay_options()),
        ] {
            if let Some(element) = document.get_element_by_id(id) {
                if element.get_attribute("data-rendered").as_deref() != Some(html.as_str()) {
                    element.set_inner_html(&html);
                    let _ = element.set_attribute("data-rendered", &html);
                }
            }
        }
        if let Some(element) = document.get_element_by_id("mobile-pause-button") {
            let _ = element.set_attribute(
                "aria-pressed",
                if self.mobile.paused { "true" } else { "false" },
            );
        }
        if let Some(element) = document.get_element_by_id("arm-pause-button") {
            let _ = element.set_attribute(
                "aria-pressed",
                if self.manipulation.paused {
                    "true"
                } else {
                    "false"
                },
            );
            element.set_text_content(Some(if self.manipulation.paused {
                "再開"
            } else {
                "一時停止"
            }));
        }
        if let Some(element) = document.get_element_by_id("experiment-results-button") {
            if results.is_empty() {
                let _ = element.set_attribute("disabled", "");
            } else {
                let _ = element.remove_attribute("disabled");
            }
        }
        if let Some(element) = document.get_element_by_id("control-wave") {
            if self.experiment.active()
                || self.experiment.editor_visible
                || self.manipulation.active()
            {
                let _ = element.set_attribute("disabled", "");
            } else {
                let _ = element.remove_attribute("disabled");
            }
        }
        if let Some(element) = document.get_element_by_id("experiment-pause-button") {
            element.set_text_content(Some(if self.experiment.paused {
                "再開"
            } else {
                "一時停止"
            }));
            let _ = element.set_attribute(
                "aria-pressed",
                if self.experiment.paused {
                    "true"
                } else {
                    "false"
                },
            );
        }
        self.reset_counter = reset_counter;
        self.wave_counter = wave_counter;
        self.mission_counter = mission_counter;
        if let Some(mission) = document.get_element_by_id("mission-status") {
            mission.set_text_content(Some(&scene.mission_status()));
        }
        if let Some(motion) = document.get_element_by_id("engine-motion") {
            motion.set_text_content(Some(&format!(
                "{} · 向き {:.0}°",
                scene.motion_status(),
                scene.heading_rad().to_degrees()
            )));
        }
        self.orbit.focus = scene.focus();
        if let Some(position) = document.get_element_by_id("engine-position") {
            position.set_text_content(Some(&format!(
                "X {:.2} m · Z {:.2} m",
                self.orbit.focus.x, self.orbit.focus.z
            )));
        }

        let view = self.orbit.camera_transform();
        viewer
            .render(&view, &render_scene, CLEAR_COLOR)
            .map_err(|error| error.to_string())
    }
}
