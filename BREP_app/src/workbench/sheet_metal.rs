//! The "Sheet Metal" workbench: the sheet-metal feature set plus the shared
//! building blocks.
//!
//! Includes the sheet-metal (`SM.*`) features AND the shared building blocks
//! (sketch `S` / datum `D` / plane `P`) — a sheet-metal part still starts from a
//! sketch on a datum. Pure modeling features are filtered out of the creation UI.
//! Classification lives HERE, off the feature type code (the kernel is untouched).
//!
//! Phase 2 adds the real flat-pattern (unfold/export) toolbar button to
//! [`BUTTONS`]; v1 ships none (no dead / "coming soon" affordance).

use super::{FeatureInfo, Workbench, WorkbenchButton};

/// The sheet-metal (`SM.*`) features plus the shared building blocks `S`/`D`/`P`.
fn includes(feature: &FeatureInfo<'_>) -> bool {
    feature.type_code.starts_with("SM.") || matches!(feature.type_code, "S" | "D" | "P")
}

/// Phase 2: the flat-pattern (unfold) export button. A click surfaces
/// `"sheetmetal.flat_pattern"` out of the toolbar to the shell, which opens the
/// export modal in its DXF / SVG flat-pattern mode. Because "All" derives the
/// DEDUPED UNION of every workbench's buttons, this same button also appears in
/// the All workbench automatically (no hand-maintained list).
static BUTTONS: &[WorkbenchButton] = &[WorkbenchButton {
    id: "sheetmetal.flat_pattern",
    // ▤ (U+25A4, square with horizontal fill) — a flat sheet with fold lines;
    // renders in the bundled DejaVu font (no tofu).
    glyph: "\u{25A4}",
    tooltip: "Export flat pattern (DXF / SVG)",
    when: None,
    pressed: None,
    disabled: None,
    caption: None,
    menu: &[],
}];

/// No panel claims (v1).
static PANELS: &[&str] = &[];

pub static SHEET_METAL: Workbench = Workbench {
    id: "sheetMetal",
    label: "Sheet Metal",
    glyph: "\u{E062}",
    includes,
    buttons: BUTTONS,
    panels: PANELS,
};
