//! Welds belong to the current part or assembly history.
use super::{FeatureInfo, Workbench};

pub static WELDING: Workbench = Workbench {
    id: "welding",
    label: "Welding",
    glyph: "\u{E0D6}",
    includes: |feature: &FeatureInfo<'_>| {
        matches!(
            feature.type_code,
            "WELD" | "S" | "D" | "P" | "ACOMP" | "APAT"
        )
    },
    buttons: &[],
    panels: &[
        super::pmi::PANEL_ID,
        super::assembly::CONSTRAINTS_PANEL_ID,
        super::assembly::BOM_PANEL_ID,
    ],
};
