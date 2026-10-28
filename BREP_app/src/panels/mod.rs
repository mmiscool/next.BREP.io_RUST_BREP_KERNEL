//! The left-control-panel modules — one file per panel, each following ONE
//! pattern so parallel agents can add a new panel with minimal conflict:
//!
//! * a small state struct (its own editing buffers only);
//! * a `show(&mut self, ui: &mut egui::Ui, state: &mut EngineState, …)` method
//!   the shell calls — `EngineState` (brep-render) stays the single brain,
//!   borrowed in; pass the `ModelStore` where a panel persists.
//!
//! Adding a panel = add `panels/<name>.rs` (state struct + `show`), `pub mod
//! <name>;` here, one field on `BrepApp`, and one `self.<name>.show(…)` call in
//! the shell. See `brep-app/README.md` → "Adding a panel".

pub mod action_rail;

/// Where the toolbar/ribbon ends this frame, recorded by the shell right after
/// drawing it (`set_chrome_bottom`); [`below_chrome`] reads it.
const CHROME_BOTTOM: &str = "brep-chrome-bottom";

/// Record the bottom of the top chrome (menu bar + toolbar or ribbon).
pub fn set_chrome_bottom(ctx: &egui::Context, y: f32) {
    ctx.data_mut(|d| d.insert_temp(egui::Id::new(CHROME_BOTTOM), y));
}

/// A floating window's first-open position: `x` across, and `dy` below the
/// toolbar or ribbon, whatever its height. The windows used to name a fixed
/// y (56, 80, 96, …) that the Ribbon, taller than the old toolbar, now covers:
/// Settings and Part Properties opened over its buttons and Plugins, with no
/// default, over the menu bar (lane U audit item 4).
pub fn below_chrome(ctx: &egui::Context, x: f32, dy: f32) -> egui::Pos2 {
    let top = ctx.data(|d| d.get_temp::<f32>(egui::Id::new(CHROME_BOTTOM))).unwrap_or(56.0);
    egui::pos2(x, top + 8.0 + dy)
}
pub mod assembly_components;
pub mod assembly_constraints;
pub mod exploded_views;
pub mod bom;
pub mod bom_plm;
pub mod bom_columns;
pub mod bom_configuration;
pub mod bug_report;
pub mod cam;
pub mod instructions;
pub mod instructions_player;
pub mod cam_setup;
pub mod cam_tool;
pub mod cam_library;
pub mod cam_operation;
pub mod cam_machine;
pub mod cam_generate;
pub mod cam_multiaxis;
pub mod cam_post;
pub mod cam_autoprogram;
pub mod cam_sim;
pub mod busy;
pub mod component_actions;
pub mod context_bar;
pub mod dock;
pub mod document_tabs;
pub mod mode_bar;
pub mod expressions;
pub mod fabrication_export;
pub mod ecad_parts;
pub mod file;
pub mod file_explorer;
pub mod history;
pub mod info;
pub mod info_windows;
pub mod auto_constraints;
pub mod interference;
pub mod kicad_import;
// Native only: the `--kicad-library` window and the bulk sweep it runs. Both
// are `#![cfg(not(target_arch = "wasm32"))]` inside, so the wasm build compiles
// them away to nothing.
pub mod kicad_bulk;
pub mod kicad_library;
pub mod part_properties;
pub mod parts_library;
pub mod pick_bar;
pub mod plm;
pub mod plm_follow;
pub mod plm_host;
pub mod pmi;
pub mod plm_attachments;
pub mod plm_launch;
pub mod plm_workspace;
pub mod plm_import;
pub mod plm_tab_follow;
pub mod plm_parts;
/// The family table pane on a PLM store (S9): Generate on the server, the member part type, the category keys.
pub mod plm_family;
/// Review and the inbox (plm-cad-integration-todo §3 S4).
pub mod plm_review;
pub mod qualify;
pub mod replace_component;
pub mod family_table_editor;
pub mod scene;
pub mod selection;
pub mod sketch;
pub mod sketch3d;
pub mod spline_anchors;
pub mod cage_editor;
pub mod settings;
pub mod sheets;
pub mod step_parts;
pub mod stl_import;
pub mod toasts;
pub mod toolbar;
pub mod toolbar_button;
pub mod workbench_toolbar;
pub mod tree;
pub mod update_components;
pub mod wire_harness;

pub mod ribbon;
pub mod simulation;
