//! brep_mcp_core — the BREP MCP server, host-agnostic.
//!
//! The governing rule is that **the shape of the engine drives the MCP**:
//! every tool, schema and resource this crate serves is read from a registry
//! the engine or the app exposes, never from a table kept here. What lives in
//! this crate is the translation to the Model Context Protocol, session
//! management, the server-side compositions (image post-processing, scripts,
//! validation) that are MCP concerns rather than engine concerns, and the two
//! transports.
//!
//! The crate does not depend on the app. A *host* owns an app instance and
//! answers the [`host`] protocol; two exist: the app's own window (`brep-app
//! --mcp`, in the `BREP_app` crate) and the `BREP_mcp` crate's headless
//! `egui_kittest` harness. That is what lets the app embed the server without
//! a dependency cycle.
//!
//! Module map:
//! - [`tools`]   — the server's view of a tool: `ToolSpec` (name, group, doc,
//!   derived input schema, annotations, handler) and the sets that produce them.
//! - [`host`]    — the host protocol: envelopes, replies, the backend seam and
//!   the [`host::Platform`] a host supplies.
//! - [`session`] — one app instance under one host, its directory and recorder.
//! - [`schema`]  — the kernel feature catalogue rendered as JSON Schema.
//! - [`validate`]— feature-parameter validation against the catalogue.
//! - [`script`]  — the JSON step-list script format and its `expect` operators.
//! - [`script_sdk`] — the `brep.<group>.<tool>(args)` functions every host's
//!   code runner exposes.
//! - [`files`]   — bounded file operations under named roots.
//! - [`image`]   — PNG decode/encode, crop, scale, diff for screenshots.
//!
//! With the `server` feature (default):
//! - [`server`]  — the dynamic `rmcp::ServerHandler` over those sets, stdio;
//!   [`server::BrepServer`] is the entry point a host constructs.
//! - [`http`]    — the same server over streamable HTTP on a loopback listener.
//! - [`cad`]     — the CAD operation service: JavaScript/TypeScript code
//!   execution (`run_script`), the front-door tools, artifacts and journals.
//! - [`program`] — JavaScript test programs played through the code runner.
//! - [`runner`]  — plays a script through a tool set and writes the artefacts.
//! - [`render_mesh`] — a CPU mesh renderer and silhouette comparison.
//! - [`generate`]— renders the registries to Markdown/JSON reference docs and
//!   TypeScript declarations.
//! - [`launch_state`] — the native launch policy for execution journals.
//! - [`skills`], [`skill_distribution`], [`prompts`] — the embedded reviewed
//!   workflows, their offline bundle installer, and MCP prompts.
//! - [`plm_fixture`] — a real `brep-plm serve` a script runs against (`plm`).
//!
//! Without `server` the crate is the tool set over the host protocol alone,
//! for a host that supplies its own platform — the browser build of the app,
//! which compiles this for wasm32 so a page serves the same tools the desktop
//! does. See the README for the transports, credentials and launch options.
#[cfg(feature = "server")]
pub mod cad;
#[cfg(feature = "server")]
pub mod generate;
pub mod host;
pub mod files;
#[cfg(feature = "server")]
pub mod http;
#[cfg(feature = "server")]
pub mod launch_state;
pub mod image;
#[cfg(feature = "server")]
pub mod plm_fixture;
#[cfg(feature = "server")]
pub mod program;
#[cfg(feature = "server")]
pub mod render_mesh;
#[cfg(feature = "server")]
pub mod runner;
pub mod schema;
pub mod script;
pub mod script_sdk;
#[cfg(feature = "server")]
pub mod server;
pub mod session;
pub mod tools;
pub mod validate;

#[cfg(feature = "server")]
pub mod skills;
#[cfg(feature = "server")]
pub mod skill_distribution;

#[cfg(feature = "server")]
pub mod prompts;


#[cfg(feature = "server")]
mod progress;
