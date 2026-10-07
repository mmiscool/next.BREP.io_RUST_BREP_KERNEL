//! The "Pads" workbench — a part's footprint pads, in eCAD's footprint editor.
//! The pads live in the part document itself; pads match the symbol's pins by
//! their label.
//!
//! Like PMI and Drawing it creates no features. Its buttons are the pads
//! editor's tools — eCAD's own `pads.tool.*` actions, whose ids already lead
//! with this workbench's — and the snapping grid, a menu of `pads.grid.*`.

use super::ecad::{ecad_button, no_features};
use super::{ButtonState, Workbench, WorkbenchButton};

/// The grid menu's id. Not an eCAD action: its entries are.
pub const GRID_MENU_ID: &str = "pads.grid";

/// The snapping grid's steps, eCAD's `pads.grid.*` actions, one pressed. The
/// editor's own toolbar carries the grid as a drop-down, and the app does not
/// draw that toolbar, so without these the grid could not be chosen at all:
/// pads clicked at a 1.27 mm pitch landed 1.25 mm apart on the 50 µm default.
pub static GRIDS: &[WorkbenchButton] = &[
    ecad_button!("pads.grid.10", "\u{E08D}", "Grid 0.01 mm", "Home/Pads/Grid 0.01 mm", toggle),
    ecad_button!("pads.grid.50", "\u{E08D}", "Grid 0.05 mm", "Home/Pads/Grid 0.05 mm", toggle),
    ecad_button!("pads.grid.100", "\u{E08D}", "Grid 0.1 mm", "Home/Pads/Grid 0.1 mm", toggle),
    ecad_button!("pads.grid.250", "\u{E08D}", "Grid 0.25 mm", "Home/Pads/Grid 0.25 mm", toggle),
    ecad_button!("pads.grid.500", "\u{E08D}", "Grid 0.5 mm", "Home/Pads/Grid 0.5 mm", toggle),
    ecad_button!("pads.grid.635", "\u{E08D}", "Grid 0.635 mm (25 mil)", "Home/Pads/Grid 0.635 mm (25 mil)", toggle),
    ecad_button!("pads.grid.1270", "\u{E08D}", "Grid 1.27 mm (50 mil)", "Home/Pads/Grid 1.27 mm (50 mil)", toggle),
    ecad_button!("pads.grid.2540", "\u{E08D}", "Grid 2.54 mm (100 mil)", "Home/Pads/Grid 2.54 mm (100 mil)", toggle),
];

/// The grid menu's live caption: the step a click snaps to, so the closed
/// menu still says it.
fn grid_menu_caption(state: &ButtonState) -> String {
    GRIDS
        .iter()
        .find(|entry| entry.offered(state) && entry.is_pressed(state))
        .map_or_else(|| "Grid".to_string(), |entry| entry.detail(state))
}

static BUTTONS: &[WorkbenchButton] = &[
    ecad_button!("pads.tool.select", "\u{1F446}", "Select", "Home/Pads/Select", toggle),
    ecad_button!("pads.tool.smd", "\u{E08A}", "SMD pad", "Home/Pads/SMD pad", toggle),
    ecad_button!("pads.tool.through_hole", "\u{E08B}", "Through-hole pad", "Home/Pads/Through-hole pad", toggle),
    ecad_button!("pads.tool.line", "\u{E06E}", "Line", "Home/Pads/Line", toggle),
    ecad_button!("pads.tool.rectangle", "\u{2610}", "Rectangle", "Home/Pads/Rectangle", toggle),
    ecad_button!("pads.tool.circle", "\u{25EF}", "Circle", "Home/Pads/Circle", toggle),
    WorkbenchButton {
        id: GRID_MENU_ID,
        glyph: "\u{E08D}",
        ribbon_path: "Home/Pads/Snapping grid", size: crate::workbench::CommandSize::Compact, tooltip: "Snapping grid",
        when: None,
        pressed: None,
        disabled: None,
        caption: Some(grid_menu_caption),
        menu: GRIDS,
    },
];

pub static PADS: Workbench = Workbench {
    id: "pads",
    label: "Pads",
    glyph: "\u{E07C}",
    includes: no_features,
    buttons: BUTTONS,
    panels: &[super::ecad::INSPECTOR_PANEL_ID],
};
