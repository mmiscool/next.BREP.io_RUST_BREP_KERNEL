//! PMI (Product & Manufacturing Information) — a COMMAND SET, not a workbench.
//!
//! PMI annotates the model being worked on, so it has no workbench of its own:
//! the workbenches in [`HOSTS`] (Modeling, Sheet Metal, Assembly — and All,
//! which sees everything) each claim the one PMI panel and show the one "PMI"
//! ribbon tab ([`TAB`]). This file is the single declaration of that set:
//!
//! - [`PANEL_ID`], the shared PMI side panel every host claims;
//! - [`BUTTONS`], the **Capture view** button, carried by every row
//!   ([`super::shared_buttons`]) and offered wherever the panel is shown;
//! - [`group_of`], which ribbon group each annotation type's button sits in.
//!   The buttons themselves are derived from the kernel's annotation table
//!   (`workbench/commands.rs`), one per type, under the same panel gate.
//!
//! A view being active is the editing mode: activating one applies its camera,
//! visibility and wireframe, and deactivating it — or switching to a workbench
//! that shows no PMI panel — restores the modeling state (`app.rs` drives the
//! engine's `pmi_enter_workbench` / `pmi_leave_workbench` off the panel's
//! visibility). Switching between hosts keeps the active view.
//!
//! Drawing sheets, which PLACE the views captured here, are the Drawing
//! workbench's (`workbench/drawing.rs`).

use super::{ButtonState, WorkbenchButton};

/// The PMI panel's id (claim + registration key).
pub const PANEL_ID: &str = "pmi";

/// The ribbon tab every PMI command sits under.
pub const TAB: &str = "PMI";

/// The workbenches that claim the PMI panel and so show the PMI tab.
pub const HOSTS: [&str; 4] = ["modeling", "sheetMetal", "assembly", "welding"];

/// The id the retired PMI workbench had. A stored settings value naming it is
/// rewritten to Modeling where it is read (`RenderSettings::apply_json`).
pub const RETIRED_WORKBENCH_ID: &str = "pmi";

/// The Capture-view toolbar button id.
pub const CAPTURE_BUTTON_ID: &str = "pmi.capture_view";

/// Whether the active workbench shows the PMI panel: the one gate every PMI
/// command is offered under.
pub fn offered(state: &ButtonState) -> bool {
    super::panel_visible(&state.engine.settings.workbench, PANEL_ID, state)
}

/// The PMI buttons that are not annotation types.
pub static BUTTONS: &[WorkbenchButton] = &[WorkbenchButton {
    id: CAPTURE_BUTTON_ID,
    // A viewfinder round a cube — a view is a captured camera. Not the page
    // (U+1F5CE) it used to draw: that is the Note annotation's icon, and the
    // strip puts the two side by side.
    glyph: "\u{E076}",
    ribbon_path: "PMI/Views/Capture a PMI view", size: crate::workbench::CommandSize::Large, tooltip: "Capture a PMI view (camera + visibility) and start annotating it",
    when: Some(offered),
    pressed: None,
    disabled: None,
    caption: None,
    menu: &[],
}];

/// The PMI tab's group for annotation type `type_id`. A type the table below
/// does not name lands in "Annotations", so a plugin's or a new kernel type's
/// button is still on the tab; the built-in table is held total by a test.
pub fn group_of(type_id: &str) -> &'static str {
    GROUPS
        .iter()
        .find(|(_, types)| types.contains(&type_id))
        .map_or("Annotations", |(group, _)| group)
}

/// Group name, then the annotation types in it. Group names carry no `/`
/// (a ribbon path is `Tab/Group/Label`).
pub const GROUPS: &[(&str, &[&str])] = &[
    ("Dimensions", &["linear", "radial", "angle", "holeCallout"]),
    ("Notes", &["leader", "balloon", "note", "weldCallout"]),
    ("Center marks", &["centerline"]),
    ("Datums and tolerances", &["datum", "fcf"]),
    ("Explode and hide", &["explode", "hide"]),
];
