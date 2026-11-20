//! The Gears workbench: one feature per gear type (`GEAR.*`), each building an
//! ordinary solid that later Modeling features cut and join.
//!
//! Every gear type's schema puts its button on the one "Gears" ribbon tab
//! ([`TAB`]), drawn in the Gears workbench (and All, which sees everything).
//! The Gears workbench offers the gears plus the sketch, datum, plane and
//! primitives a bore or a hub is made from, and the Modeling features that cut
//! them. Modeling does not offer gears (they would crowd its palette); a part
//! made in the Gears workbench edits normally under Modeling.
use super::{FeatureInfo, Workbench};

/// The ribbon tab every gear type's button sits under.
pub const TAB: &str = "Gears";

/// The workbenches that offer gear features (and so show the Gears tab).
pub const HOSTS: [&str; 1] = ["gears"];

/// Whether `type_code` is a gear feature (`GEAR.<kind>`).
pub fn is_gear_type(type_code: &str) -> bool {
    brep_render::brep_kernel::is_gear_type(type_code)
}

/// Gears plus the building blocks and the cuts a gear is finished with.
fn includes(feature: &FeatureInfo<'_>) -> bool {
    is_gear_type(feature.type_code)
        || matches!(
            feature.type_code,
            "S" | "D" | "P" | "P.CU" | "P.CY" | "P.CO" | "E" | "R" | "H" | "B" | "F" | "CH" | "PATTERN" | "M" | "XFORM"
        )
}

pub static GEARS: Workbench = Workbench {
    id: "gears",
    label: "Gears",
    // Two meshing gears.
    glyph: "\u{E0E3}",
    includes,
    buttons: &[],
    panels: &[],
};
