//! Tangent (`TANG`) — a sphere-or-cylinder face against a planar face,
//! side-preserving: the plane's local normal is flipped when the round side
//! currently sits on the negative side, so tangency preserves the assembled
//! side (the signed-distance convention).

use super::super::mapping::{
    mate, require_axis, require_direction, require_plane, ConstraintFailure, MappedConstraint,
    ResolvedElement,
};
use super::{elements_field, id_field, pair_applicable, schema_entry, ConstraintTypeDef};
use crate::feature_pipeline::SelectionProbe;
use crate::{MateKind, SelectionGeometry};

pub(super) const DEF: ConstraintTypeDef = ConstraintTypeDef {
    type_id: "tangent",
    label: "Tangent",
    short_name: "TANG",
    icon: "\u{2312}",
    long_name: "\u{2312} Tangent",
    min_elements: 2,
    max_elements: 2,
    duplicate_family: true,
    applicable,
};

/// Tangent pairs faces only (sphere/cylinder against a plane).
fn applicable(probe: &SelectionProbe) -> bool {
    pair_applicable(probe, probe.faces)
}

pub(super) fn schema() -> serde_json::Value {
    schema_entry(&DEF, serde_json::json!({
        "id": id_field(),
        "elements": elements_field(&["FACE"], 2, 2,
            "Select a sphere or cylinder face and a planar face to make tangent"),
    }))
}

pub(in crate::feature_pipeline::assembly) fn map(
    a: &ResolvedElement,
    b: &ResolvedElement,
) -> Result<MappedConstraint, ConstraintFailure> {
    // One sphere-or-cylinder side, one planar side, either order. The plane's
    // local normal is flipped when the round side currently sits on the
    // negative side, so tangency preserves the assembled side (the signed-
    // distance convention).
    let (round, plane) = match (&a.world, &b.world) {
        (SelectionGeometry::Plane { .. }, _) => (b, a),
        _ => (a, b),
    };
    let mut plane_local = require_plane(plane)?;
    let (plane_normal_world, _) = require_direction(plane)?;
    let plane_origin_world = plane.world.representative_point();
    match round.world {
        SelectionGeometry::Sphere { center, radius } => {
            if center.sub(plane_origin_world).dot(plane_normal_world) < 0.0 {
                plane_local.normal = [
                    -plane_local.normal[0],
                    -plane_local.normal[1],
                    -plane_local.normal[2],
                ];
            }
            let SelectionGeometry::Sphere { center: local_center, .. } = round.local else {
                unreachable!("local mirrors world kind");
            };
            let height = center.sub(plane_origin_world).dot(plane_normal_world).abs();
            Ok(MappedConstraint {
                mates: vec![mate(round, plane, MateKind::TangentSpherePlane {
                    center_a: [local_center.x, local_center.y, local_center.z],
                    radius,
                    plane_b: plane_local,
                })],
                measured: Some((height, "mm")),
                target: Some(radius),
                ..Default::default()
            })
        }
        SelectionGeometry::Axis {
            origin,
            radius: Some(radius),
            ..
        } => {
            if origin.sub(plane_origin_world).dot(plane_normal_world) < 0.0 {
                plane_local.normal = [
                    -plane_local.normal[0],
                    -plane_local.normal[1],
                    -plane_local.normal[2],
                ];
            }
            let height = origin.sub(plane_origin_world).dot(plane_normal_world).abs();
            Ok(MappedConstraint {
                mates: vec![mate(round, plane, MateKind::TangentCylinderPlane {
                    axis_a: require_axis(round)?,
                    radius,
                    plane_b: plane_local,
                })],
                measured: Some((height, "mm")),
                target: Some(radius),
                ..Default::default()
            })
        }
        SelectionGeometry::Axis { radius: None, .. } => Err(ConstraintFailure::unsupported(
            format!(
                "selection '{}' is not a constant-radius cylinder (tangent supports sphere and cylinder faces)",
                round.name
            ),
        )),
        _ => Err(ConstraintFailure::unsupported(format!(
            "tangent needs a sphere-or-cylinder face and a planar face ('{}' vs '{}')",
            a.name, b.name
        ))),
    }
}
