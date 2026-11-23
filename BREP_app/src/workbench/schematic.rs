//! The "PCB diagram" workbench — schematic capture, in eCAD's sheet-and-board
//! editor on the schematic document, held on its schematic view
//! ([`super::ecad::BENCHES`]). The board of the same document is the PCB
//! workbench's ([`super::pcb`]); the two are one design, so a part placed here is
//! on the board in the same undo step, as it was when they were one workbench's
//! two views.
//!
//! Its OWN buttons are the schematic's half of eCAD's sheet actions
//! ([`super::ecad`]): the net tools and the Context commands of the tool in hand,
//! each offered, lit, greyed and captioned by eCAD's own predicate. After them the
//! row carries Assembly's ADD COMPONENT, borrowed as PCB borrows it
//! ([`super::BORROWED`]): every symbol on this sheet is an assembly component.

use super::ecad::{ecad_button, no_features};
use super::{Workbench, WorkbenchButton};

/// In eCAD's own toolbar order: Tool, Context.
static BUTTONS: &[WorkbenchButton] = &[
    ecad_button!("schematic.sheet.tool.select", "\u{1F446}", "Select", "Home/PCB diagram/Select", toggle),
    ecad_button!("schematic.sheet.tool.wire", "\u{E07F}", "Draw wire", "Home/PCB diagram/Draw wire", toggle),
    ecad_button!("schematic.sheet.tool.junction", "\u{E080}", "Junction", "Home/PCB diagram/Junction", toggle),
    ecad_button!("schematic.sheet.tool.label", "\u{E081}", "Net label", "Home/PCB diagram/Net label", toggle),
    ecad_button!("schematic.sheet.tool.no_connect", "\u{E0C0}", "No-connect", "Home/PCB diagram/No-connect", toggle),
    ecad_button!("schematic.sheet.tool.power_flag", "\u{E0C1}", "Power flag", "Home/PCB diagram/Power flag", toggle),
    ecad_button!("schematic.sheet.tool.power", "\u{E0C2}", "Power symbol", "Home/PCB diagram/Power symbol", toggle),
    ecad_button!("schematic.sheet.place.rotate", "\u{E083}", "Rotate 90°", "Home/PCB diagram/Rotate 90°", command),
    ecad_button!("schematic.sheet.wire.bend", "\u{E084}", "Bend direction", "Home/PCB diagram/Bend direction", command),
    ecad_button!("schematic.sheet.wire.finish", "\u{E087}", "Finish wire", "Home/PCB diagram/Finish wire", command),
];

pub static SCHEMATIC: Workbench = Workbench {
    id: "schematic",
    label: "PCB diagram",
    glyph: "\u{E07D}",
    includes: no_features,
    buttons: BUTTONS,
    panels: &[super::ecad::LIBRARY_PANEL_ID, super::ecad::INSPECTOR_PANEL_ID],
};
