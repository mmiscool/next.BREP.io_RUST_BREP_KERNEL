//! brep_mcp — the BREP MCP automation server, hosted headlessly.
//!
//! The server itself lives in `brep_mcp_core` (re-exported here module for
//! module, so `brep_mcp::server::BrepServer` is the same type the app embeds
//! behind `brep-app --mcp`). This crate adds what needs the app compiled in:
//! the [`headless`] host — the whole `BrepApp` under `egui_kittest` on a wgpu
//! device — the [`annotate`] overlay the documentation walkthroughs are drawn
//! with, and the `brep-mcp` binary (`serve`, `schema`, `test`, `shot`,
//! `render-mesh`).
pub use brep_mcp_core::{
    generate, host, http, image, plm_fixture, render_mesh, runner, schema, script, server, session, tools, validate,
};

pub mod annotate;
pub mod headless;

/// The backend every `brep-mcp` server runs on: one headless app per session.
pub fn headless_backend() -> host::Backend {
    host::Backend::Spawn { name: "headless", spawn: std::sync::Arc::new(|cfg| headless::spawn(cfg)) }
}
