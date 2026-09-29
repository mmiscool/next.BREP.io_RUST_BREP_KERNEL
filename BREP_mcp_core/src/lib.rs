//! brep_mcp_core — the BREP MCP server, host-agnostic.
//!
//! The governing rule is that **the shape of the engine drives the MCP**: every
//! tool, schema and resource this crate serves is read from a registry the
//! engine or the app exposes, never from a table kept here. What lives in this
//! crate is the translation to the Model Context Protocol, session management,
//! the server-side compositions (image post-processing, scripts, validation)
//! that are MCP concerns rather than engine concerns, and the two transports.
//!
//! The crate does not depend on the app. A *host* owns an app instance and
//! answers the [`host`] protocol; two exist: the app's own window (`brep-app
//! --mcp`, `BREP_app/src/mcp.rs`) and BREP_mcp's headless `egui_kittest`
//! harness. That is what lets the app embed the server without a dependency
//! cycle.
//!
//! Module map:
//! - [`tools`]   — the server's view of a tool: `ToolSpec` (name, group, doc,
//!   derived input schema, annotations, handler) and the sets that produce them.
//! - [`server`]  — the dynamic `rmcp::ServerHandler` over those sets, stdio.
//! - [`http`]    — the same server over streamable HTTP on a loopback listener.
//! - [`schema`]  — the kernel feature catalogue rendered as JSON Schema.
//! - [`validate`]— feature-parameter validation against the catalogue.
//! - [`script`]  — the `test-mcp` script format and its expectation operators.
//! - [`image`]   — PNG decode/encode, crop, scale, diff for screenshots.
//! - [`render_mesh`] — the CPU mesh renderer and silhouette comparison that
//!   replaced step-validation's headless three.js/Playwright renderer.
//! - [`runner`]  — plays a script through a tool set and writes the artefacts.
//! - [`generate`]— renders the registries to the checked-in generated docs.
//! - [`host`]    — the host protocol: envelopes, replies, the backend seam.
//! - [`session`] — one app instance under one host, its directory and recorder.
//! - [`plm_fixture`] — a real `brep-plm serve` a script runs against (`plm`).
pub mod generate;
pub mod host;
pub mod http;
pub mod image;
pub mod plm_fixture;
pub mod render_mesh;
pub mod runner;
pub mod schema;
pub mod script;
pub mod server;
pub mod session;
pub mod tools;
pub mod validate;
