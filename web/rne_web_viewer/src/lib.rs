//! Browser WASM viewer for Robot Native Engine.

#![deny(missing_docs)]

#[cfg(target_arch = "wasm32")]
mod app;
#[cfg(any(target_arch = "wasm32", test))]
mod experiment;
mod footstep;
mod humanoid;
mod lab;
#[cfg(any(target_arch = "wasm32", test))]
mod manipulation;
#[cfg(any(target_arch = "wasm32", test))]
mod mobile;
mod physics_walk;
mod playground;
mod scene;
#[cfg(any(target_arch = "wasm32", test))]
mod scene_config;
mod surfaces;

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

/// Starts the interactive web viewer on the page canvas.
#[cfg(target_arch = "wasm32")]
#[wasm_bindgen(start)]
pub fn start() {
    // Report panics in the console instead of a bare `unreachable` trap.
    std::panic::set_hook(Box::new(|info| {
        web_sys::console::error_1(&format!("RNE panic: {info}").into());
    }));
    app::run();
}

pub use humanoid::{G1Input, G1Scene};
pub use scene::WebScene;
