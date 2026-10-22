//! winit application loop for the browser viewer.

use crate::humanoid::{G1Input, G1Scene};
use rne_render_wgpu::{CameraOrbit, InteractiveViewer, ViewerError};
use std::sync::Arc;
use winit::application::ApplicationHandler;
use winit::dpi::PhysicalPosition;
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, EventLoop, EventLoopProxy};
use winit::platform::web::EventLoopExtWebSys;
use winit::window::{Window, WindowId};

const CLEAR_COLOR: [f32; 4] = [0.05, 0.08, 0.12, 1.0];

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
            yaw_rad: 0.9,
            pitch_rad: 1.15,
            distance_m: 3.2,
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
            Ok(viewer) => {
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
                .and_then(|element| element.dyn_into::<web_sys::HtmlInputElement>().ok())
                .and_then(|input| input.value().parse::<f64>().ok())
                .filter(|value| value.is_finite())
                .ok_or_else(|| format!("invalid control: {id}"))
        };
        let reset_counter = value("control-reset-counter")? as u32;
        let wave_counter = value("control-wave-counter")? as u32;
        let mission_counter = value("control-mission-counter")? as u32;
        let targets_rad = (0..scene.joint_count())
            .map(|index| value(&format!("control-joint-{index}")))
            .collect::<Result<Vec<_>, _>>()?;
        let render_scene = scene.play_frame(
            G1Input {
                forward: value("control-forward")?,
                turn: value("control-turn")?,
                targets_rad,
                reset: reset_counter != self.reset_counter,
            },
            value("control-motion-enabled")? == 1.0,
            wave_counter != self.wave_counter,
            mission_counter != self.mission_counter,
        )?;
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
