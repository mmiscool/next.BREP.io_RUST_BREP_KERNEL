//! Touch Align (`TALN`) — two like elements touch/align: plane/plane →
//! coplanar with the captured facing preference, axis/axis → collinear,
//! point/point → coincident.

use super::super::mapping::{
    direction_of, effective_align, line_line_distance, mate, require_axis, require_direction,
    require_plane, ConstraintFailure, MappedConstraint, ResolvedElement,
};
use super::{elements_field, id_field, pair_applicable, schema_entry, ConstraintTypeDef};
use crate::feature_pipeline::assembly::ConstraintEntry;
use crate::feature_pipeline::SelectionProbe;
use crate::{MateKind, SelectionGeometry};

pub(super) const DEF: ConstraintTypeDef = ConstraintTypeDef {
    type_id: "touch_align",
    label: "Touch Align",
    short_name: "TALN",
    icon: "\u{2AA5}",
    long_name: "\u{2AA5} Touch Align",
    min_elements: 2,
    max_elements: 2,
    duplicate_family: true,
    applicable,
};

/// Touch-align pairs faces/edges.
fn applicable(probe: &SelectionProbe) -> bool {
    pair_applicable(probe, probe.faces + probe.edges)
}

pub(super) fn schema() -> serde_json::Value {
    schema_entry(&DEF, serde_json::json!({
        "id": id_field(),
        "elements": elements_field(&["FACE", "EDGE", "VERTEX"], 2, 2,
            "Select two like elements to touch/align"),
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
    match (&a.world, &b.world) {
        (SelectionGeometry::Plane { origin: oa, normal: na },
         SelectionGeometry::Plane { origin: ob, .. }) => {
            let (wa, _) = require_direction(a)?;
            let (wb, _) = require_direction(b)?;
            let align = effective_align(entry, wa, wb, reverse);
            let separation = ob.sub(*oa).dot(*na);
            Ok(MappedConstraint {
                mates: vec![mate(a, b, MateKind::CoincidentPlanePlane {
                    plane_a: require_plane(a)?,
                    plane_b: require_plane(b)?,
                    align,
                })],
                measured: Some((separation.abs(), "mm")),
                ..Default::default()
            })
        }
        (SelectionGeometry::Point { .. }, SelectionGeometry::Point { .. }) => {
            super::coincident::map(a, b)
        }
        (ga, gb)
            if direction_of(ga).is_some()
                && direction_of(gb).is_some()
                && !matches!(ga, SelectionGeometry::Plane { .. })
                && !matches!(gb, SelectionGeometry::Plane { .. }) =>
        {
            let (wa, _) = require_direction(a)?;
            let (wb, _) = require_direction(b)?;
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
        _ => Err(ConstraintFailure::unsupported(format!(
            "touch-align selections must be two faces, two edges, or two vertices \
             ('{}' vs '{}' mix kinds)",
            a.name, b.name
        ))),
    }
}
