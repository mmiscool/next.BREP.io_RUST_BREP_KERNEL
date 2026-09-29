//! Rendering, picking, and sketch editing for native and WebAssembly hosts.
//!
//! [`scene`] stores named geometry, [`pipeline`] evaluates feature histories,
//! and [`render`] draws into a host-provided texture view. Cameras, picking,
//! sketch state, and display settings are shared across presentation targets.

// `RenderSettings::to_json` is one `json!` literal over every setting, and
// `json_internal!` recurses once per key — the default 128 stops short of the
// full settings object.
#![recursion_limit = "512"]

// Re-export the kernel crate so downstream shells (e.g. `brep-app`'s wasm
// `WorkerRunner`) can name the run-boundary types the `runner::HistoryRunner`
// trait exposes (`brep_kernel::HistoryRequest`, …) WITHOUT taking their own
// direct path dependency — which would have to mirror this crate's kernel
// feature split (`default-features = false` / native `parallel`) or feature
// unification bites. One canonical instance, reached as `brep_render::brep_kernel`.
pub use brep_kernel;

pub mod assembly_status;
pub mod camera;
pub mod color;
pub mod constraint_overlays;
pub mod controls;
pub mod engine_state;
pub mod feature_dimensions;
pub mod features;
pub mod face_transform_help;
pub mod formatting;
mod geometry2d;
mod geometry3d;
pub mod history;
mod json_support;
pub mod metadata;
pub mod pick;
pub mod pipeline;
pub mod render;
pub mod run_trace;
pub mod runner;
pub mod scene;
pub mod sheets;
pub mod sketch;
pub mod style;
pub mod thumbnail;
pub mod visibility;
pub mod view;
pub mod widgets;

// The wasm-bindgen browser API (attach/resize/events/camera/pick/scene-feed/
// settings/world→screen) — the R3 seam the host UI programs against. wasm only;
// its GPU wrapper needs a canvas/WebGPU context.
#[cfg(target_arch = "wasm32")]
pub mod engine;

