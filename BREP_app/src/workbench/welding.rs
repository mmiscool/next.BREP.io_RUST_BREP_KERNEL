//! Welds belong to the current part or assembly history.
//!
//! Welding is a workbench AND a command set: every weld type is a feature
//! whose schema puts its button on the one "Welding" ribbon tab ([`TAB`]), and
//! the workbenches in [`HOSTS`] (Modeling, Sheet Metal, Assembly, Welding — and
//! All, which sees everything) include every weld type ([`is_weld_type`]), so
//! the tab, the add-feature palette and the context bar offer welds there.
use super::{FeatureInfo, Workbench};

/// The ribbon tab every weld type's button sits under.
pub const TAB: &str = "Welding";

/// The workbenches that offer weld features (and so show the Welding tab).
pub const HOSTS: [&str; 4] = ["modeling", "sheetMetal", "assembly", "welding"];

/// Whether `type_code` is a weld feature: the fillet weld `WELD`, or one of
/// the other kinds `WELD.<kind>`.
pub fn is_weld_type(type_code: &str) -> bool {
    brep_render::brep_kernel::is_weld_type(type_code)
}

pub static WELDING: Workbench = Workbench {
    id: "welding",
    label: "Welding",
    glyph: "\u{E0D6}",
    includes: |feature: &FeatureInfo<'_>| {
        is_weld_type(feature.type_code)
            || matches!(feature.type_code, "S" | "D" | "P" | "ACOMP" | "APAT")
    },
    buttons: &[],
    panels: &[
        super::pmi::PANEL_ID,
        super::assembly::CONSTRAINTS_PANEL_ID,
        super::assembly::BOM_PANEL_ID,
    ],
};
