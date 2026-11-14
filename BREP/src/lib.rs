//! # BREP
//!
//! Umbrella crate for the BREP CAD kernel family. It re-exports the geometry
//! kernel unconditionally and the higher layers behind feature flags, so a
//! single `BREP` dependency scales from headless geometry to the full app:
//!
//! | Module      | Feature    | Backing crate  |
//! |-------------|------------|----------------|
//! | [`kernel`]  | *(always)* | `BREP_kernel`  |
//! | `gizmos`  | `gizmos`   | `BREP_gizmos`  |
//! | `render`  | `render`   | `BREP_render`  |
//! | `app`     | `app`      | `BREP_app`     |
//! | `reconstruction` | `reconstruction` | `BREP_reconstruction` |
//!
//! `full` is an alias for `app` (which also enables `render` and `gizmos`); it
//! does not enable `reconstruction` or `parallel`. `parallel` forwards to the
//! kernel's rayon parallelism (tessellation and STEP import).
//!
//! ```toml
//! BREP = "0.21"                                     # kernel only
//! BREP = { version = "0.21", features = ["render"] } # + the wgpu render engine
//! BREP = { version = "0.21", features = ["full"] }   # the eframe app, render and gizmos
//! ```
//!
//! ```no_run
//! use brep::kernel; // always present
//! # #[cfg(feature = "render")]
//! use brep::render; // with `features = ["render"]`
//! ```

/// The BREP geometry kernel — always available.
pub use brep_kernel as kernel;

/// In-scene gizmos and overlay widgets. Enable the `gizmos` feature.
#[cfg(feature = "gizmos")]
pub use brep_gizmos as gizmos;

/// The windowing-agnostic wgpu render/pick engine. Enable the `render` feature.
#[cfg(feature = "render")]
pub use brep_render as render;

/// The eframe (egui + wgpu) CAD application. Enable the `app` feature.
#[cfg(feature = "app")]
pub use brep_app as app;

/// RANSAC recognition-to-kernel reconstruction. Enable `reconstruction`.
#[cfg(feature = "reconstruction")]
pub use brep_reconstruction as reconstruction;
