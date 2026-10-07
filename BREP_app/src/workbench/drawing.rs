//! The "Drawing" workbench — the mode in which drawing SHEETS are made: saved
//! PMI views placed on paper, dimensioned, sectioned and detailed.
//!
//! Like PMI it is NOT feature creation: the `includes` predicate rejects every
//! catalogue entry, and the history stays fully editable as everywhere. What
//! it adds is the **Sheets** panel — its own, claimed by no other workbench but
//! All — and the sheet's tools as buttons in the workbench row. The views a
//! sheet places are captured and annotated in the PMI workbench; this one only
//! reads them.
//!
//! Leaving the workbench closes an open sheet (`app.rs` drives that off the
//! Sheets panel's visibility), for the reason PMI leaving deactivates its view:
//! a workbench that hides the Sheets pane must not leave the central tile
//! drawing paper.
//!
//! # The buttons
//!
//! **Add sheet** and **Place view** are always offered. The rest are offered
//! only while a sheet is OPEN ([`WorkbenchButton::when`]), so the row grows when
//! the viewport turns to paper and shrinks back when it does not: **Back to
//! 3D**, the six dimension constructions ([`DIM_TOOLS`]), the two ordinate sets
//! ([`ORD_TOOLS`]), the SECTION VIEW ([`SECTION_BUTTON_ID`]) and the DETAIL VIEW
//! ([`DETAIL_BUTTON_ID`]).
//!
//! # The workflow is the PMI annotation buttons'
//!
//! A construction button is momentary: it CREATES its object on the open sheet
//! with nothing picked yet and opens the object's dialog in the Sheets pane.
//! What the object measures or cuts is then picked from the dialog's reference
//! rows with the same reference picker every other dialog uses — Select, click
//! anchors on the paper, Finish — and where its line sits is a field of the
//! dialog (and a drag of its value box on the paper). There is no armed tool
//! and no click that means "place".
//!
//! Fit and zoom are NOT here: the main toolbar's own Zoom-to-fit fits the open
//! sheet's paper (`BrepApp::zoom_to_fit`) and the wheel zooms the paper the way
//! it zooms the 3D scene, so neither needs a second button.

use super::{ButtonState, FeatureInfo, Workbench, WorkbenchButton};
use brep_render::sheets::dimension::{
    ALIGNED, ANGULAR, DIAMETRAL, HORIZONTAL, LINEAR, RADIAL, VERTICAL,
};

/// The Sheets panel's id (claim + registration key).
pub const SHEETS_PANEL_ID: &str = "sheets";

/// **Add sheet** — a new sheet at the default paper size, opened on the paper.
pub const ADD_SHEET_BUTTON_ID: &str = "drawing.add_sheet";

/// **Place view** — place a saved PMI view on the open sheet and open the
/// placement's dialog, where the view, position and scale are chosen.
pub const PLACE_VIEW_BUTTON_ID: &str = "drawing.place_view";

/// The **Back to 3D** button id — leaving the open sheet. The Sheets pane's own
/// row menu (**Close sheet**) and its `Open` tick still do the same thing; this
/// is the copy that lives where every other mode's actions live, so leaving the
/// paper never means hunting for a pane.
pub const SHEET_CLOSE_BUTTON_ID: &str = "drawing.sheet_close";

/// The six dimension constructions the open sheet offers, as
/// `(button id, kind, alignment)` — ONE table, read by the buttons below and by
/// `BrepApp::dispatch_workbench_button`, which creates that kind. The ids
/// are the whole contract: a script presses
/// `workbench:btn:drawing.dim.horizontal`, or names the same id to the
/// `workbench_button` command.
///
/// An ANGULAR construction has no alignment to carry — its direction is the
/// two picked edges' — so it files [`ALIGNED`] the way the two radial
/// constructions do: the column is a linear dimension's and nothing else reads
/// it.
pub const DIM_TOOLS: &[(&str, &str, &str)] = &[
    ("drawing.dim.horizontal", LINEAR, HORIZONTAL),
    ("drawing.dim.vertical", LINEAR, VERTICAL),
    ("drawing.dim.aligned", LINEAR, ALIGNED),
    ("drawing.dim.angular", ANGULAR, ALIGNED),
    ("drawing.dim.radius", RADIAL, ALIGNED),
    ("drawing.dim.diameter", DIAMETRAL, ALIGNED),
];

/// The `(kind, alignment)` a dimension button creates, or `None` for any other id.
pub fn dim_tool(id: &str) -> Option<(&'static str, &'static str)> {
    DIM_TOOLS
        .iter()
        .find(|(button, _, _)| *button == id)
        .map(|(_, kind, alignment)| (*kind, *alignment))
}

/// The two ORDINATE SET constructions, as `(button id, axis)` — the same
/// one-table rule the dimensions follow. They are a separate table because an
/// ordinate set is a separate OBJECT.
pub const ORD_TOOLS: &[(&str, &str)] = &[
    ("drawing.ord.horizontal", HORIZONTAL),
    ("drawing.ord.vertical", VERTICAL),
];

/// The axis an ordinate button creates, or `None` for any other id.
pub fn ord_tool(id: &str) -> Option<&'static str> {
    ORD_TOOLS.iter().find(|(button, _)| *button == id).map(|(_, axis)| *axis)
}

/// The SECTION VIEW button id. One button, so it needs no table: a section has
/// no kinds and no axes, only a cutting line on a placed view.
pub const SECTION_BUTTON_ID: &str = "drawing.section";

/// The INSERT BOM TABLE button id: one table per sheet, so one button that
/// inserts it (or, once inserted, opens the sheet's dialog on its columns).
pub const BOM_BUTTON_ID: &str = "drawing.bom";

/// The DETAIL VIEW button id: a circle on a placed view, redrawn larger.
pub const DETAIL_BUTTON_ID: &str = "drawing.detail";

/// No creatable features: sheets are not history features.
fn includes(_feature: &FeatureInfo<'_>) -> bool {
    false
}

/// A sheet is open — the condition every paper tool is offered under. With no
/// paper on screen there is nothing to close and nothing to dimension.
fn sheet_is_open(state: &ButtonState) -> bool {
    state.engine.sheet_open().is_some()
}

static BUTTONS: &[WorkbenchButton] = &[
    WorkbenchButton {
        id: ADD_SHEET_BUTTON_ID,
        glyph: "\u{E077}",
        ribbon_path: "Home/Drawing/Add sheet", size: crate::workbench::CommandSize::Compact, tooltip: "Add sheet \u{2014} a new drawing sheet at the default paper size, opened on the paper",
        when: None,
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: PLACE_VIEW_BUTTON_ID,
        glyph: "\u{E078}",
        ribbon_path: "Home/Drawing/Place view", size: crate::workbench::CommandSize::Compact, tooltip: "Place view \u{2014} put a saved PMI view on the open sheet and open its dialog",
        when: None,
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    // --- the OPEN SHEET's tools, in row order: the way out first, then the
    // six constructions, the two ordinate sets, the section and the detail.
    WorkbenchButton {
        id: SHEET_CLOSE_BUTTON_ID,
        glyph: "\u{E068}",
        ribbon_path: "Home/Drawing/Back to 3D", size: crate::workbench::CommandSize::Compact, tooltip: "Back to 3D (close the open sheet; it keeps its contents)",
        when: Some(sheet_is_open),
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "drawing.dim.horizontal",
        glyph: "\u{E069}",
        ribbon_path: "Home/Drawing/Horizontal dimension", size: crate::workbench::CommandSize::Compact, tooltip: "Horizontal dimension \u{2014} the horizontal distance between two anchors",
        when: Some(sheet_is_open),
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "drawing.dim.vertical",
        glyph: "\u{E06A}",
        ribbon_path: "Home/Drawing/Vertical dimension", size: crate::workbench::CommandSize::Compact, tooltip: "Vertical dimension \u{2014} the vertical distance between two anchors",
        when: Some(sheet_is_open),
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "drawing.dim.aligned",
        glyph: "\u{E06B}",
        ribbon_path: "Home/Drawing/Aligned dimension", size: crate::workbench::CommandSize::Compact, tooltip: "Aligned dimension \u{2014} the true distance between two anchors",
        when: Some(sheet_is_open),
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "drawing.dim.angular",
        glyph: "\u{E06F}",
        ribbon_path: "Home/Drawing/Angular dimension", size: crate::workbench::CommandSize::Compact, tooltip: "Angular dimension \u{2014} the angle between two straight edges AS THIS VIEW PROJECTS THEM",
        when: Some(sheet_is_open),
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "drawing.dim.radius",
        glyph: "\u{E06C}",
        ribbon_path: "Home/Drawing/Radius dimension", size: crate::workbench::CommandSize::Compact, tooltip: "Radius dimension \u{2014} a projected circle's radius (R6.000)",
        when: Some(sheet_is_open),
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "drawing.dim.diameter",
        glyph: "\u{E06D}",
        ribbon_path: "Home/Drawing/Diameter dimension", size: crate::workbench::CommandSize::Compact, tooltip: "Diameter dimension \u{2014} a projected circle's diameter (\u{2300}12.000)",
        when: Some(sheet_is_open),
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "drawing.ord.horizontal",
        glyph: "\u{E070}",
        ribbon_path: "Home/Drawing/Horizontal ordinate set", size: crate::workbench::CommandSize::Compact, tooltip: "Horizontal ordinate set \u{2014} a datum, then a run of members reading their distance along the paper's X",
        when: Some(sheet_is_open),
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: "drawing.ord.vertical",
        glyph: "\u{E071}",
        ribbon_path: "Home/Drawing/Vertical ordinate set", size: crate::workbench::CommandSize::Compact, tooltip: "Vertical ordinate set \u{2014} a datum, then a run of members reading their distance up the paper",
        when: Some(sheet_is_open),
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: SECTION_BUTTON_ID,
        glyph: "\u{E072}",
        ribbon_path: "Home/Drawing/Section view", size: crate::workbench::CommandSize::Compact, tooltip: "Section view \u{2014} a new section; pick its cutting line on a placed view in its dialog",
        when: Some(sheet_is_open),
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: BOM_BUTTON_ID,
        glyph: "\u{E08E}",
        ribbon_path: "Home/Drawing/BOM table", size: crate::workbench::CommandSize::Compact, tooltip: "BOM table \u{2014} insert the sheet's BOM table (one per sheet) and open the sheet's dialog, where its columns, position and column width are chosen",
        when: Some(sheet_is_open),
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
    WorkbenchButton {
        id: DETAIL_BUTTON_ID,
        glyph: "\u{E074}",
        ribbon_path: "Home/Drawing/Detail view", size: crate::workbench::CommandSize::Compact, tooltip: "Detail view \u{2014} a new detail; pick its circle's centre and rim on a placed view in its dialog",
        when: Some(sheet_is_open),
        pressed: None,
        disabled: None,
        caption: None,
        menu: &[],
    },
];

static PANELS: &[&str] = &[SHEETS_PANEL_ID];

pub static DRAWING: Workbench = Workbench {
    id: "drawing",
    label: "Drawing",
    glyph: "\u{E075}",
    includes,
    buttons: BUTTONS,
    panels: PANELS,
};
