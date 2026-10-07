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
pub mod assembly_components;
pub mod assembly_constraints;
pub mod bom;
pub mod bom_plm;
pub mod bom_columns;
pub mod bom_configuration;
pub mod bug_report;
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
// Native only: the `--kicad-library` window, the GitLab fetch behind its
// download path, and the bulk sweep over both. All three are
// `#![cfg(not(target_arch = "wasm32"))]` inside, so the wasm build compiles them
// away to nothing.
pub mod kicad_bulk;
pub mod kicad_library;
pub mod kicad_remote;
pub mod part_properties;
pub mod parts_library;
pub mod plm;
pub mod plm_follow;
pub mod plm_host;
pub mod pmi;
pub mod plm_attachments;
pub mod plm_launch;
pub mod plm_workspace;
pub mod plm_import;
pub mod plm_parts;
/// The family table pane on a PLM store (S9): Generate on the server, the member part type, the category keys.
pub mod plm_family;
/// Review, change orders and the inbox (plm-cad-integration-todo §3 S4).
pub mod plm_review;
pub mod qualify;
pub mod family_table_editor;
pub mod scene;
pub mod selection;
pub mod sketch;
pub mod spline_anchors;
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
