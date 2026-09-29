//! Parallel (`PARA`) — two direction-bearing elements align, with the
//! captured facing preference.

use super::super::mapping::{
    angle_between_deg, effective_align, mate, require_direction, ConstraintFailure,
    MappedConstraint, ResolvedElement,
};
use super::{elements_field, id_field, pair_applicable, schema_entry, ConstraintTypeDef};
use crate::feature_pipeline::assembly::ConstraintEntry;
use crate::feature_pipeline::SelectionProbe;
use crate::MateKind;

pub(super) const DEF: ConstraintTypeDef = ConstraintTypeDef {
    type_id: "parallel",
    label: "Parallel",
    short_name: "PARA",
    icon: "\u{2225}",
    long_name: "\u{2225} Parallel",
    min_elements: 2,
    max_elements: 2,
    duplicate_family: false,
    applicable,
};

/// Parallel pairs direction-bearing faces/edges.
fn applicable(probe: &SelectionProbe) -> bool {
    pair_applicable(probe, probe.faces + probe.edges)
}

pub(super) fn schema() -> serde_json::Value {
    schema_entry(&DEF, serde_json::json!({
        "id": id_field(),
        "elements": elements_field(&["FACE", "EDGE"], 2, 2,
            "Select two direction-bearing elements to make parallel"),
        "reverse": {
            "type": "boolean",
            "default_value": false,
            "hint": "Flip the captured facing preference"
        },
    }))
}

pub(in crate::feature_pipeline::assembly) fn map(
    entry: &mut ConstraintEntry,
    a: &ResolvedElement,
    b: &ResolvedElement,
) -> Result<MappedConstraint, ConstraintFailure> {
    let reverse = entry.flag("reverse");
    let (wa, la) = require_direction(a)?;
    let (wb, lb) = require_direction(b)?;
    let align = effective_align(entry, wa, wb, reverse);
    let angle = angle_between_deg(wa, wb);
    Ok(MappedConstraint {
        mates: vec![mate(a, b, MateKind::Parallel {
            direction_a: [la.x, la.y, la.z],
            direction_b: [lb.x, lb.y, lb.z],
            align,
        })],
        measured: Some((angle, "deg")),
        ..Default::default()
    })
}
