//! The "PCB" workbench — the electronics flow: schematic capture and board
//! layout, in eCAD's sheet-and-board editor on a schematic document, with the
//! schematic and the board as two views of the one workbench.
//!
//! Its OWN buttons are eCAD's actions ([`super::ecad`]): the view switch, the
//! schematic's four tools and the board's two, the copper-layer menu, and the
//! Context commands of the tool in hand. Each is offered, lit, greyed and
//! captioned by eCAD's own predicate, so the row shows exactly what eCAD's
//! toolbar would: the schematic's tools in the schematic, the board's on the
//! board.
//!
//! It declares no feature of its own — `includes` is still [`no_features`], so
//! the creation palette offers nothing here — but it is no longer true that
//! nothing on this row builds one. After its own buttons the row carries
//! Assembly's ADD COMPONENT, BORROWED rather than copied ([`super::BORROWED`]): the very button Assembly
//! declares, under its own id, dispatched by the same shell arm. A board IS an
//! assembly of placed parts, and the components the schematic and the board
//! place are the same `ACOMP` features Assembly's button appends, so this is
//! one action reached from a second place rather than a second action.
//!
//! `includes` and the row answer different questions: `includes` filters
//! feature CREATION IN THE PALETTE, and a button is an action with its own
//! flow. Widening it to `ACOMP` would put the raw feature in the palette
//! beside eCAD's tools, which is not what was asked for and not how a board
//! gets its parts.

use super::ecad::{ecad_button, no_features, LAYER_MENU_ID};
use super::{Workbench, WorkbenchButton};

/// The copper-layer menu's entries — eCAD's six layer actions, offered as far
/// as the board has layers, each captioned with the layer's name
/// (`F.Cu`, `In1.Cu` … `B.Cu`). They draw the menu's picture: a layer is told
/// apart by its name, which no fixed picture could carry, because it depends
/// on the layer count (`layer_name(1, 2)` is `B.Cu`, `layer_name(1, 4)` is
/// `In1.Cu`).
pub static LAYERS: &[WorkbenchButton] = &[
    ecad_button!("pcb.board.layer.1", "\u{E08C}", "Copper layer 1", "Home/PCB/Copper layer 1", toggle),
    ecad_button!("pcb.board.layer.2", "\u{E08C}", "Copper layer 2", "Home/PCB/Copper layer 2", toggle),
    ecad_button!("pcb.board.layer.3", "\u{E08C}", "Copper layer 3", "Home/PCB/Copper layer 3", toggle),
    ecad_button!("pcb.board.layer.4", "\u{E08C}", "Copper layer 4", "Home/PCB/Copper layer 4", toggle),
    ecad_button!("pcb.board.layer.5", "\u{E08C}", "Copper layer 5", "Home/PCB/Copper layer 5", toggle),
    ecad_button!("pcb.board.layer.6", "\u{E08C}", "Copper layer 6", "Home/PCB/Copper layer 6", toggle),
];

/// In eCAD's own toolbar order: View, Tool, the layer menu, Context.
static BUTTONS: &[WorkbenchButton] = &[
    ecad_button!("pcb.view.schematic", "\u{E07D}", "Schematic", "Home/View/Schematic", toggle),
    ecad_button!("pcb.view.board", "\u{E07E}", "PCB", "Home/View/PCB", toggle),
    ecad_button!("pcb.sheet.tool.select", "\u{1F446}", "Select", "Home/PCB/Select", toggle),
    ecad_button!("pcb.sheet.tool.wire", "\u{E07F}", "Draw wire", "Home/PCB/Draw wire", toggle),
    ecad_button!("pcb.sheet.tool.junction", "\u{E080}", "Junction", "Home/PCB/Junction", toggle),
    ecad_button!("pcb.sheet.tool.label", "\u{E081}", "Net label", "Home/PCB/Net label", toggle),
    ecad_button!("pcb.sheet.tool.no_connect", "\u{E0C0}", "No-connect", "Home/PCB/No-connect", toggle),
    ecad_button!("pcb.sheet.tool.power_flag", "\u{E0C1}", "Power flag", "Home/PCB/Power flag", toggle),
    ecad_button!("pcb.sheet.tool.power", "\u{E0C2}", "Power symbol", "Home/PCB/Power symbol", toggle),
    ecad_button!("pcb.board.tool.select", "\u{1F446}", "Select", "Home/PCB/Select", toggle),
    ecad_button!("pcb.board.tool.route", "\u{E082}", "Route track", "Home/PCB/Route track", toggle),
    ecad_button!("pcb.board.tool.zone", "\u{E0B0}", "Draw zone", "Home/PCB/Draw zone", toggle),
    WorkbenchButton {
        id: LAYER_MENU_ID,
        glyph: "\u{E08C}",
        ribbon_path: "Home/PCB/Copper layer", size: crate::workbench::CommandSize::Compact, tooltip: "Copper layer",
        when: None,
        pressed: None,
        disabled: None,
        caption: Some(super::ecad::layer_menu_caption),
        menu: LAYERS,
    },
    ecad_button!("pcb.sheet.place.rotate", "\u{E083}", "Rotate 90°", "Home/PCB/Rotate 90°", command),
    ecad_button!("pcb.sheet.wire.bend", "\u{E084}", "Bend direction", "Home/PCB/Bend direction", command),
    ecad_button!("pcb.sheet.wire.finish", "\u{E087}", "Finish wire", "Home/PCB/Finish wire", command),
    ecad_button!("pcb.board.route.corner", "\u{E085}", "Corner direction", "Home/PCB/Corner direction", command),
    ecad_button!("pcb.board.route.via", "\u{E086}", "Place via", "Home/PCB/Place via", command),
    ecad_button!("pcb.board.route.finish", "\u{E087}", "Finish track", "Home/PCB/Finish track", command),
    ecad_button!("pcb.board.zone.finish", "\u{E0B2}", "Close zone", "Home/PCB/Close zone", command),
    ecad_button!("pcb.board.zones.fill", "\u{E0B1}", "Fill zones", "Home/PCB/Fill zones", command),
];

pub static PCB: Workbench = Workbench {
    id: "pcb",
    label: "PCB",
    glyph: "\u{E07A}",
    includes: no_features,
    buttons: BUTTONS,
    panels: &[super::ecad::LIBRARY_PANEL_ID, super::ecad::INSPECTOR_PANEL_ID],
};
