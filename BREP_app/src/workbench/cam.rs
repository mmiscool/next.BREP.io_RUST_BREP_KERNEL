//! The "CAM" workbench — Fusion 360's Manufacture workspace and SOLIDWORKS
//! CAM in this app: setups, tools and operations that machine the modelled
//! part, generated into toolpaths and posted to NC programs.
//!
//! CAM is NOT feature creation: the `includes` predicate rejects every
//! catalogue entry (the Add-feature palette is empty here), and the history
//! stays fully editable as everywhere. What a setup machines is the model the
//! history already built. What the workbench adds is its CAM pane — the
//! browser tree of Setups › Operations, NC Programs and the Tool Library —
//! and the four toolbar buttons below, each of which surfaces the pane and
//! hands the pane its action.
//!
//! Everything CAM makes lives in the document's own `cam` block, never in the
//! history: user inputs (setups, tools, operations, post settings, target
//! references) are saved; toolpaths, meshes, indexes and stock states are
//! derived and regenerated.

use super::{FeatureInfo, Workbench, WorkbenchButton};

/// The CAM pane's id (claim + registration key).
pub const PANEL_ID: &str = "cam";

/// The toolbar buttons' ids — named because the shell arm that answers for
/// each and the pane that performs it refer to one button, not to a spelling.
pub const NEW_SETUP_BUTTON_ID: &str = "cam.new_setup";
pub const NEW_TOOL_BUTTON_ID: &str = "cam.new_tool";
pub const NEW_OPERATION_BUTTON_ID: &str = "cam.new_operation";
pub const GENERATE_BUTTON_ID: &str = "cam.generate";

/// No creatable features: CAM data is not a history feature.
fn includes(_feature: &FeatureInfo<'_>) -> bool {
    false
}

/// The four CAM actions, in the order a program is written: a setup, its
/// tools, its operations, then Generate. Each is always offered; the pane
/// says what is missing (an operation wants a setup and a tool) rather than
/// the row hiding the button, so the way in is always on screen.
static BUTTONS: &[WorkbenchButton] = &[
    WorkbenchButton {
        id: NEW_SETUP_BUTTON_ID,
        // Stock block with the work-coordinate triad at its corner.
        glyph: "\u{E0C8}",
        ribbon_path: "Home/CAM/New setup", size: crate::workbench::CommandSize::Large, tooltip: "New setup: a work coordinate system, stock and heights for the model",
        when: None,
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: NEW_TOOL_BUTTON_ID,
        // An end mill: shank, flutes and a flat tip.
        glyph: "\u{E0C9}",
        ribbon_path: "Home/CAM/New tool", size: crate::workbench::CommandSize::Large, tooltip: "New tool: add a cutter to the document's tool library",
        when: None,
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: NEW_OPERATION_BUTTON_ID,
        // A zig-zag toolpath over a surface.
        glyph: "\u{E0CA}",
        ribbon_path: "Home/CAM/New operation", size: crate::workbench::CommandSize::Large, tooltip: "New operation: a machining strategy against picked geometry, in the selected setup",
        when: None,
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: GENERATE_BUTTON_ID,
        // A play arrow over a toolpath: generate every operation.
        glyph: "\u{E0CB}",
        ribbon_path: "Home/CAM/Generate", size: crate::workbench::CommandSize::Large, tooltip: "Generate: compute the toolpaths of every operation that needs it, in the background",
        when: None,
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
];

static PANELS: &[&str] = &[PANEL_ID];

pub static CAM: Workbench = Workbench {
    id: "cam",
    label: "CAM",
    // A milling cutter over a block of stock.
    glyph: "\u{E0C7}",
    includes,
    buttons: BUTTONS,
    panels: PANELS,
};
