//! Fixed (`FIXD`) — ground ONE component in place. The only one-element type,
//! and the only one with no `map`: [`super::super::lifecycle`] grounds the
//! resolved component (`set_component_fixed`) BEFORE body building, so every
//! later fixed-flag read sees it.

use super::{elements_field, id_field, schema_entry, ConstraintTypeDef};
use crate::feature_pipeline::SelectionProbe;

pub(super) const DEF: ConstraintTypeDef = ConstraintTypeDef {
    type_id: "fixed",
    label: "Fixed",
    short_name: "FIXD",
    icon: "\u{23DA}",
    long_name: "\u{23DA} Fixed",
    min_elements: 1,
    max_elements: 1,
    duplicate_family: false,
    applicable,
};

/// Fixed grounds ONE component, selected via its member solid(s).
fn applicable(probe: &SelectionProbe) -> bool {
    probe.all_component && probe.components == 1 && probe.solids >= 1
}

pub(super) fn schema() -> serde_json::Value {
    schema_entry(&DEF, serde_json::json!({
        "id": id_field(),
        "elements": elements_field(&["COMPONENT"], 1, 1,
            "Select the component to ground (fix in place)"),
    }))
}
