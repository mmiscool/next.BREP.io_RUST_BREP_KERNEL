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

/// No extra toolbar buttons in v1.
static BUTTONS: &[WorkbenchButton] = &[];

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
