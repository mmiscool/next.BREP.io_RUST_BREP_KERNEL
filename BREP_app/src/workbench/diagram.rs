//! The "Diagram" workbench — wiring and systems diagrams, whose output is the
//! point-to-point connection list, in eCAD's sheet editor on a WIRING document.
//!
//! Like PMI and Drawing it creates no features. A wiring diagram has no nets
//! and no board: eCAD refuses the net tools there (`set_tool`), refuses the
//! board view (`set_view`) and does not offer the view switch, and its
//! connections are dragged pin to pin with Select. So of the eCAD actions the
//! PCB row carries, the only two a wiring document can ever offer are its
//! buttons — `every_declared_diagram_button_is_all_a_wiring_editor_offers`
//! holds that against eCAD's predicates.

use super::ecad::{ecad_button, no_features};
use super::{Workbench, WorkbenchButton};

static BUTTONS: &[WorkbenchButton] = &[
    ecad_button!("diagram.sheet.tool.select", "\u{1F446}", "Select", toggle),
    ecad_button!("diagram.sheet.place.rotate", "\u{E083}", "Rotate 90°", command),
];

pub static DIAGRAM: Workbench = Workbench {
    id: "diagram",
    label: "Diagram",
    glyph: "\u{E079}",
    includes: no_features,
    buttons: BUTTONS,
    panels: &[super::ecad::LIBRARY_PANEL_ID, super::ecad::INSPECTOR_PANEL_ID],
};
