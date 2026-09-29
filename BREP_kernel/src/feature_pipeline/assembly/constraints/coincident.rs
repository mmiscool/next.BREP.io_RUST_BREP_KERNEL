//! Coincident (`COIN`) — two elements touch: exactly one planar side →
//! point-on-plane, anything else → the representative points coincide.

use super::super::mapping::{
    local_point, mate, require_plane, ConstraintFailure, MappedConstraint, ResolvedElement,
};
use super::{elements_field, id_field, pair_applicable, schema_entry, ConstraintTypeDef};
use crate::feature_pipeline::SelectionProbe;
use crate::{MateKind, SelectionGeometry};

pub(super) const DEF: ConstraintTypeDef = ConstraintTypeDef {
    type_id: "coincident",
    label: "Coincident",
    short_name: "COIN",
    icon: "\u{2261}",
    long_name: "\u{2261} Coincident",
    min_elements: 2,
    max_elements: 2,
    duplicate_family: false,
    applicable,
};

/// Coincident seeds faces, edges, or whole components (via member solids).
fn applicable(probe: &SelectionProbe) -> bool {
    pair_applicable(probe, probe.faces + probe.edges + probe.solids)
}

pub(super) fn schema() -> serde_json::Value {
    schema_entry(&DEF, serde_json::json!({
        "id": id_field(),
        "elements": elements_field(&["VERTEX", "EDGE", "FACE", "COMPONENT"], 2, 2,
            "Select two elements to make coincident"),
    }))
}

pub(in crate::feature_pipeline::assembly) fn map(
    a: &ResolvedElement,
    b: &ResolvedElement,
) -> Result<MappedConstraint, ConstraintFailure> {
    // Exactly one planar side → point-on-plane; anything else → the
    // representative points coincide.
    let mapped = match (&a.world, &b.world) {
        (SelectionGeometry::Plane { origin, normal }, other)
            if !matches!(other, SelectionGeometry::Plane { .. }) =>
        {
            let height = (b.world.representative_point().sub(*origin)).dot(*normal);
            MappedConstraint {
                mates: vec![mate(b, a, MateKind::CoincidentPointPlane {
                    point_a: local_point(b),
                    plane_b: require_plane(a)?,
                })],
                measured: Some((height.abs(), "mm")),
                ..Default::default()
            }
        }
        (other, SelectionGeometry::Plane { origin, normal })
            if !matches!(other, SelectionGeometry::Plane { .. }) =>
        {
            let height = (a.world.representative_point().sub(*origin)).dot(*normal);
            MappedConstraint {
                mates: vec![mate(a, b, MateKind::CoincidentPointPlane {
                    point_a: local_point(a),
                    plane_b: require_plane(b)?,
                })],
                measured: Some((height.abs(), "mm")),
                ..Default::default()
            }
        }
        _ => {
            let gap = a
                .world
                .representative_point()
                .sub(b.world.representative_point())
                .length();
            MappedConstraint {
                mates: vec![mate(a, b, MateKind::CoincidentPointPoint {
                    point_a: local_point(a),
                    point_b: local_point(b),
                })],
                measured: Some((gap, "mm")),
                ..Default::default()
            }
        }
    };
    Ok(mapped)
}
