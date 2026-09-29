//! Perpendicular (`PERP`) — two direction-bearing elements at right
//! angles.

use super::super::mapping::{
    angle_between_deg, mate, require_direction, ConstraintFailure, MappedConstraint,
    ResolvedElement,
};
use super::{elements_field, id_field, pair_applicable, schema_entry, ConstraintTypeDef};
use crate::feature_pipeline::SelectionProbe;
use crate::MateKind;

pub(super) const DEF: ConstraintTypeDef = ConstraintTypeDef {
    type_id: "perpendicular",
    label: "Perpendicular",
    short_name: "PERP",
    icon: "\u{22A5}",
    long_name: "\u{22A5} Perpendicular",
    min_elements: 2,
    max_elements: 2,
    duplicate_family: true,
    applicable,
};

/// Perpendicular pairs direction-bearing faces/edges.
fn applicable(probe: &SelectionProbe) -> bool {
    pair_applicable(probe, probe.faces + probe.edges)
}

pub(super) fn schema() -> serde_json::Value {
    schema_entry(&DEF, serde_json::json!({
        "id": id_field(),
        "elements": elements_field(&["FACE", "EDGE"], 2, 2,
            "Select two direction-bearing elements to make perpendicular"),
    }))
}

pub(in crate::feature_pipeline::assembly) fn map(
    a: &ResolvedElement,
    b: &ResolvedElement,
) -> Result<MappedConstraint, ConstraintFailure> {
    let (wa, la) = require_direction(a)?;
    let (wb, lb) = require_direction(b)?;
    Ok(MappedConstraint {
        mates: vec![mate(a, b, MateKind::Perpendicular {
            direction_a: [la.x, la.y, la.z],
            direction_b: [lb.x, lb.y, lb.z],
        })],
        measured: Some((angle_between_deg(wa, wb), "deg")),
        ..Default::default()
    })
}
