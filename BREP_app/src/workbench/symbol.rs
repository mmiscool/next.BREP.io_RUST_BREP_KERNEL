//! The "Symbol" workbench — a part's schematic symbol, in eCAD's symbol
//! editor. The symbol lives in the part document itself: BREP does not use a
//! KiCad library directly, and a placed component takes a copy of its part's.
//!
//! Like PMI and Drawing it creates no features. Its buttons are the symbol
//! editor's drawing tools — eCAD's own `symbol.tool.*` actions, whose ids
//! already lead with this workbench's.

use super::ecad::{ecad_button, no_features};
use super::{Workbench, WorkbenchButton};

static BUTTONS: &[WorkbenchButton] = &[
    ecad_button!("symbol.tool.select", "\u{1F446}", "Select", toggle),
    ecad_button!("symbol.tool.line", "\u{E06E}", "Line", toggle),
    ecad_button!("symbol.tool.rectangle", "\u{2610}", "Rectangle", toggle),
    ecad_button!("symbol.tool.circle", "\u{25EF}", "Circle", toggle),
    ecad_button!("symbol.tool.pin", "\u{E088}", "Pin", toggle),
    ecad_button!("symbol.tool.text", "\u{E089}", "Text", toggle),
];

pub static SYMBOL: Workbench = Workbench {
    id: "symbol",
    label: "Symbol",
    glyph: "\u{E07B}",
    includes: no_features,
    buttons: BUTTONS,
    panels: &[super::ecad::INSPECTOR_PANEL_ID],
};
