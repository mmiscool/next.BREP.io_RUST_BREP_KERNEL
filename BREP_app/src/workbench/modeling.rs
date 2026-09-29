//! The "Modeling" workbench (the DEFAULT): general solid modeling.
//!
//! Includes every feature EXCEPT the sheet-metal ones and the assembly
//! component — i.e. the modeling features plus the shared building blocks
//! (sketch / datum / plane). The `SM.*` codes and `ACOMP` are filtered out of
//! the creation UI (ACOMP is the Assembly workbench's creatable).
//! Classification lives HERE, off the feature type code (the kernel is
//! untouched).

use super::{FeatureInfo, Workbench, WorkbenchButton};

/// Everything that is not sheet-metal (`SM.*`) or the assembly component
/// (`ACOMP`) — which naturally keeps the modeling features AND the common
/// building blocks (`S` / `D` / `P`).
fn includes(feature: &FeatureInfo<'_>) -> bool {
    !feature.type_code.starts_with("SM.") && feature.type_code != "ACOMP"
}

/// No extra toolbar buttons in v1.
static BUTTONS: &[WorkbenchButton] = &[];

/// No panel claims (v1).
static PANELS: &[&str] = &[];

pub static MODELING: Workbench = Workbench {
    id: "modeling",
    label: "Modeling",
    glyph: "\u{E061}",
    includes,
    buttons: BUTTONS,
    panels: PANELS,
};
