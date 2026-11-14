//! brep_mcp — the BREP MCP automation server, hosted headlessly.
//!
//! The server itself lives in `brep_mcp_core`. This crate re-exports its
//! main modules (`server`, `http`, `host`, `session`, `tools`, `schema`,
//! `validate`, `script`, `program`, `runner`, `image`, `render_mesh`,
//! `generate`, `plm_fixture`), so `brep_mcp::server::BrepServer` is the same
//! type the app embeds behind `brep-app --mcp`. It adds what needs the app
//! compiled in:
//!
//! - the [`headless`] host — the whole `BrepApp` under `egui_kittest` on a
//!   wgpu device — and [`headless_backend`], the backend every `brep-mcp`
//!   server runs on (one headless app per session);
//! - [`command_descriptions`], the application's static command metadata;
//! - the [`annotate`] overlay the documentation walkthroughs are drawn with;
//! - the `brep-mcp` binary (`serve`, `schema`, `test`, `shot`, `render-mesh`,
//!   `typescript`, `skills`).
//!
//! See the README for installation, client configuration and the tool groups.
pub use brep_mcp_core::{
    generate, host, http, image, plm_fixture, program, render_mesh, runner, schema, script, server, session, tools, validate,
};

pub mod annotate;
pub mod headless;

/// The backend every `brep-mcp` server runs on: one headless app per session.
pub fn headless_backend() -> host::Backend {
    host::Backend::Spawn { name: "headless", spawn: std::sync::Arc::new(|cfg| headless::spawn(cfg)) }
}

/// Static application command metadata; discovery does not create a GPU.
pub fn command_descriptions() -> serde_json::Value { brep_app::automation::command::describe() }
