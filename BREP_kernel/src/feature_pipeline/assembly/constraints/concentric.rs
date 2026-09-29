//! Concentric (`CONC`) — two axis-bearing elements share an axis:
//! cylindrical/conical faces, circular edges.

use super::super::mapping::{
    effective_align, line_line_distance, mate, require_axis, require_direction, ConstraintFailure,
    MappedConstraint, ResolvedElement,
};
use super::{elements_field, id_field, pair_applicable, schema_entry, ConstraintTypeDef};
use crate::feature_pipeline::assembly::ConstraintEntry;
use crate::feature_pipeline::SelectionProbe;
use crate::{MateKind, SelectionGeometry};

pub(super) const DEF: ConstraintTypeDef = ConstraintTypeDef {
    type_id: "concentric",
    label: "Concentric",
    short_name: "CONC",
    icon: "\u{25CE}",
    long_name: "\u{25CE} Concentric",
    min_elements: 2,
    max_elements: 2,
    duplicate_family: true,
    applicable,
};

/// Concentric pairs axis-bearing faces/edges.
fn applicable(probe: &SelectionProbe) -> bool {
    pair_applicable(probe, probe.faces + probe.edges)
}

pub(super) fn schema() -> serde_json::Value {
    schema_entry(&DEF, serde_json::json!({
        "id": id_field(),
        "elements": elements_field(&["FACE", "EDGE"], 2, 2,
            "Select two axis-bearing elements (cylindrical/conical face, circular edge) to make concentric"),
        "reverse": {
            "type": "boolean",
            "default_value": false,
            "hint": "Flip the captured axis-sense preference"
        },
    }))
}

pub(in crate::feature_pipeline::assembly) fn map(
    entry: &mut ConstraintEntry,
    a: &ResolvedElement,
    b: &ResolvedElement,
) -> Result<MappedConstraint, ConstraintFailure> {
    let reverse = entry.flag("reverse");
    let (wa, _) = require_direction(a)?;
    let (wb, _) = require_direction(b)?;
    if matches!(a.world, SelectionGeometry::Plane { .. })
        || matches!(b.world, SelectionGeometry::Plane { .. })
    {
        return Err(ConstraintFailure::unsupported(
            "concentric needs two axis-bearing selections (cylindrical/conical face or circular edge)",
        ));
    }
    let align = effective_align(entry, wa, wb, reverse);
    let gap = line_line_distance(
        a.world.representative_point(), wa,
        b.world.representative_point(), wb,
    );
    Ok(MappedConstraint {
        mates: vec![mate(a, b, MateKind::ConcentricAxisAxis {
            axis_a: require_axis(a)?,
            axis_b: require_axis(b)?,
            align,
        })],
        measured: Some((gap, "mm")),
        ..Default::default()
    })
}
