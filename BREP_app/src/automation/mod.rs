//! The automation layer — the one command channel the app exposes to a host
//! (`brep-mcp` headless / window, or a dial-in adapter).
//!
//! Three registries live here; the engine's feature catalogue is the fourth:
//! - [`command`] — every command registers a [`CommandSpec`](crate::automation::command::CommandSpec) (name, group,
//!   doc, derived argument/result schemas, frame phase, annotations, handler)
//!   in its module's `COMMANDS` static; [`command::registry`] gathers them.
//!   A host reads the registry and generates its tool list; nothing lists
//!   commands by hand.
//! - [`registry`] — the state registry: the JSON blobs the app publishes each
//!   frame (`__brep*`), with a doc string and, where the blob is typed, a
//!   schema. Always compiled; on wasm it also mirrors to `window.*`.
//! - [`hit_keys`] — the widget hit-key docs each panel registers beside its
//!   hit-rect publisher.
//!
//! [`queue::AutomationQueue`] is the channel: a host submits an [`command::Envelope`]
//! and gets a [`command::Reply`]; the app drains the queue at three fixed
//! points in the frame (§4.4) — `raw_input_hook` for input, the top of `ui`
//! for mutations, the bottom of `ui` after the registry is rebuilt for reads.
pub mod hit_keys;
pub(crate) mod hit_rects;
pub mod registry;
pub mod state_blobs;
#[cfg(feature = "automation")]
pub mod actions;

/// Where the toolbar's Help button points: the generated help site `brep-docs`
/// writes next to the served page. Declared here — outside the `automation`
/// feature gate — so the button and the `help_open` command that presses it
/// without a pointer read ONE literal.
pub const HELP_URL: &str = "help/index.html";

#[cfg(feature = "automation")]
pub mod command;
#[cfg(feature = "automation")]
pub mod pointer;
#[cfg(feature = "automation")]
pub mod queue;

#[cfg(feature = "automation")]
mod cmd_assembly;
mod assembly_output_schema;
// The output-schema vocabulary (U4d); some helpers only serve automation-gated schemas.
#[cfg_attr(not(feature = "automation"), allow(dead_code))]
mod schema_dsl;
#[cfg(feature = "automation")]
mod cmd_classes;
mod cmd_files;
#[cfg(feature = "automation")]
mod cmd_camera;
#[cfg(feature = "automation")]
pub mod cmd_capture;
#[cfg(feature = "automation")]
mod cmd_document;
#[cfg(feature = "automation")]
mod cmd_frame;
#[cfg(feature = "automation")]
mod cmd_history;
#[cfg(feature = "automation")]
mod cmd_input;
#[cfg(feature = "automation")]
mod cmd_metadata;
#[cfg(feature = "automation")]
mod cmd_perf;
#[cfg(feature = "automation")]
mod cmd_pmi;
#[cfg(feature = "automation")]
mod cmd_scene;
#[cfg(feature = "automation")]
mod cmd_sheets;
#[cfg(feature = "automation")]
mod cmd_settings;
#[cfg(feature = "automation")]
mod cmd_shell;
#[cfg(feature = "automation")]
mod cmd_ecad;
#[cfg(feature = "automation")]
mod cmd_state;
#[cfg(feature = "automation")]
mod cmd_wire_harness;
#[cfg(feature = "automation")]
mod cmd_cam;
mod cmd_exploded;
mod cmd_instructions;
mod cmd_instructions_motion;
mod cmd_simulation;
mod cmd_simulation_play;
mod cmd_cam_sim;
#[cfg(all(feature = "automation", not(target_arch = "wasm32")))]
mod cmd_plm;
#[cfg(all(feature = "automation", not(target_arch = "wasm32")))]
mod cmd_kicad;

/// How a host builds the app. `store: None` = the default store (the user's
/// own config directory); a host MUST pass an isolated store.
/// `seed: false` starts on an empty document instead of the seed model.
pub struct AppOptions {
    pub store: Option<Box<dyn crate::store::ModelStore>>,
    pub seed: bool,
}

impl Default for AppOptions {
    fn default() -> Self {
        Self { store: None, seed: true }
    }
}

#[cfg(feature = "automation")]
mod cmd_plugins;
mod plugin_output_schema;

#[cfg(feature = "automation")]
mod catalogue_output_schema;

#[cfg(feature = "automation")]
mod pmi_output_schema;

#[cfg(feature = "automation")]
mod diagnostic_output_schema;

#[cfg(feature = "automation")]
mod sheet_output_schema;

#[cfg(feature = "automation")]
mod cam_sim_output_schema;

#[cfg(feature = "automation")]
mod cam_tool_output_schema;

#[cfg(feature = "automation")]
mod cam_path_output_schema;

#[cfg(feature = "automation")]
mod cam_setup_output_schema;
#[cfg(feature = "automation")]
mod cam_autoprogram_output_schema;

#[cfg(feature = "automation")]
mod domain_output_schema;

#[cfg(feature = "automation")]
mod cam_state_output_schema;
