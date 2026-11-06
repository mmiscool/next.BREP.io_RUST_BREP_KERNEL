//! Coincident (`COIN`) — two elements touch: two planar faces → coplanar
//! (owner 2026-11-05: it used to pin their representative points together),
//! exactly one planar side → point-on-plane, anything else → the
//! representative points coincide.

use super::super::mapping::{
    effective_align, local_point, mate, require_direction, require_plane, ConstraintFailure, MappedConstraint,
    ResolvedElement, FACING,
};
use crate::feature_pipeline::assembly::ConstraintEntry;
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
        "facing": super::super::mapping::facing_field("Two faces: Aligned or Opposed normals on the shared plane. Unset, a new constraint takes the facing it is placed with"),
    }))
}

/// Two planar faces lie in ONE plane, with the constraint's explicit facing.
///
/// A row saved before this (no `facing`, already solved) held the two faces'
/// representative points together instead. Its faces are coplanar there when
/// they are parallel, so it becomes coplanar with the facing it has and no
/// part moves; when they are NOT parallel it keeps the point-on-point reading
/// it was solved with, and says so (coplanar would turn the part).
pub(in crate::feature_pipeline::assembly) fn map_entry(
    entry: &mut ConstraintEntry,
    a: &ResolvedElement,
    b: &ResolvedElement,
) -> Result<MappedConstraint, ConstraintFailure> {
    let (SelectionGeometry::Plane { origin, normal }, SelectionGeometry::Plane { .. }) = (&a.world, &b.world) else {
        return map(a, b);
    };
    let (wa, _) = require_direction(a)?;
    let (wb, _) = require_direction(b)?;
    // Solved under the old rule: a `kernel` solve record is only kept on a
    // satisfied or adjusting row.
    let legacy = entry.input_params.get(FACING).is_none() && entry.persistent("kernel").is_some();
    if legacy && wa.dot(wb).abs() < 1.0 - 1e-9 {
        let mut mapped = map(a, b)?;
        mapped.note = Some(
            "Saved before Coincident made two faces coplanar, and these faces are not parallel: it still holds their points together. Pick the faces again to make them coplanar".into(),
        );
        return Ok(mapped);
    }
    let align = effective_align(entry, wa, wb, "reverse");
    let height = b.world.representative_point().sub(*origin).dot(*normal);
    Ok(MappedConstraint {
        mates: vec![mate(a, b, MateKind::CoincidentPlanePlane {
            plane_a: require_plane(a)?,
            plane_b: require_plane(b)?,
            align,
        })],
        measured: Some((height.abs(), "mm")),
        ..Default::default()
    })
}

pub(in crate::feature_pipeline::assembly) fn map(
    a: &ResolvedElement,
    b: &ResolvedElement,
) -> Result<MappedConstraint, ConstraintFailure> {
    // Exactly one planar side → point-on-plane; anything else (two faces
    // reach here only through a legacy row, `map_entry`) → the
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
