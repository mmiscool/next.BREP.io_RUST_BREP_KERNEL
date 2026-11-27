//! The "Modeling" workbench (the DEFAULT): general solid modeling.
//!
//! Includes every feature EXCEPT the sheet-metal ones, the gear types (the Gears workbench,
//! [`super::gears`]) and the assembly component and its pattern — i.e. the modeling features, the
//! weld types (the Welding tab, [`super::welding`]) and the shared building blocks (sketch / datum
//! / plane). The `SM.*` codes, `ACOMP` and `APAT` are filtered out of the creation UI (the Assembly
//! workbench's creatables). Classification lives HERE, off the feature type code (the kernel is
//! untouched).

use super::{FeatureInfo, Workbench, WorkbenchButton};

/// Everything that is not sheet-metal (`SM.*`), the assembly component
/// (`ACOMP`) or its pattern (`APAT`) — which naturally keeps the modeling
/// features, the weld types AND the common building blocks (`S` / `D` / `P`).
fn includes(feature: &FeatureInfo<'_>) -> bool {
    !feature.type_code.starts_with("SM.")
        && !matches!(feature.type_code, "ACOMP" | "APAT")
        && !super::gears::is_gear_type(feature.type_code)
}

/// The Draft Analysis toggle's id; the shell turns the view on or off.
pub const DRAFT_ANALYSIS_BUTTON_ID: &str = "modeling.draft_analysis";

/// Modeling's own button: Draft Analysis, beside Draft in the Blend group.
/// It reads PRESSED while the analysis is on.
static BUTTONS: &[WorkbenchButton] = &[WorkbenchButton {
    id: DRAFT_ANALYSIS_BUTTON_ID,
    // The Draft block with its faces painted by their draft (U+E0FC).
    glyph: "\u{E0FC}",
    ribbon_path: "Home/Blend/Draft Analysis",
    size: super::CommandSize::Compact,
    tooltip: "Draft Analysis \u{2014} colour faces by their draft against a pull direction and a minimum (display only)",
    // Not in All: All's single row is as wide as the automation window allows
    // (`the_automation_width_holds_every_workbench_row_on_one_line`), the way
    // eCAD's actions are in All's union but never offered there. Not while a
    // sketch is edited either: the sketch's tools are the row then.
    when: Some(|state| state.engine.settings.workbench != "all" && !state.engine.any_sketch_mode()),
    pressed: Some(|state| state.engine.draft_analysis_active()),
    disabled: None,
    caption: None,
    menu: &[],
}];

/// The shared PMI panel (Modeling is one of its hosts).
static PANELS: &[&str] = &[super::pmi::PANEL_ID];

pub static MODELING: Workbench = Workbench {
    id: "modeling",
    label: "Modeling",
    glyph: "\u{E061}",
    includes,
    buttons: BUTTONS,
    panels: PANELS,
};
