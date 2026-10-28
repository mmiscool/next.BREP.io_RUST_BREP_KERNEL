//! Cam (`CAM`) — a follower stays in contact with a cam's profile (the "cam"
//! mechanical mate). Elements: the follower (a roller: cylindrical face or
//! circular edge; a ball: spherical face; or a point: vertex) and the cam's
//! profile edge (a flat curve: the rim of the cam's face), on two components,
//! in either order. The follower rides the outside of a closed profile (the
//! left of an open one, seen from its plane's normal) unless `flip` puts it
//! inside; a roller stays parallel to the profile's normal, a ball or point
//! stays in its plane.

use super::super::mapping::{mate, ConstraintFailure, MappedConstraint, ResolvedElement};
use super::{elements_field, id_field, pair_applicable, schema_entry, ConstraintTypeDef};
use crate::feature_pipeline::assembly::ConstraintEntry;
use crate::feature_pipeline::{Env, SelectionProbe};
use crate::{MateKind, SelectionGeometry, Vec3};

pub(super) const DEF: ConstraintTypeDef = ConstraintTypeDef {
    type_id: "cam",
    label: "Cam",
    short_name: "CAM",
    icon: "\u{25D0}",
    long_name: "\u{25D0} Cam",
    min_elements: 2,
    max_elements: 2,
    duplicate_family: false,
    applicable,
};

fn applicable(probe: &SelectionProbe) -> bool {
    probe.edges >= 1 && pair_applicable(probe, probe.faces + probe.edges + probe.vertices)
}

pub(super) fn schema() -> serde_json::Value {
    schema_entry(&DEF, serde_json::json!({
        "id": id_field(),
        "elements": elements_field(&["VERTEX", "FACE", "EDGE"], 2, 2,
            "Select the follower (a roller's cylindrical face, a ball, or a point) and the cam's profile edge"),
        "flip": {
            "type": "boolean",
            "default_value": false,
            "hint": "Ride the inside of the profile (a groove or a ring cam) instead of the outside"
        },
    }))
}

/// The follower's centre, roller axis (if any) and radius, component-local.
fn follower(element: &ResolvedElement) -> Result<([f64; 3], Option<[f64; 3]>, f64), ConstraintFailure> {
    let triple = |v: Vec3| [v.x, v.y, v.z];
    match &element.local {
        SelectionGeometry::Axis { origin, direction, radius: Some(radius) } => {
            Ok((triple(*origin), Some(triple(*direction)), *radius))
        }
        SelectionGeometry::Circle { center, axis, radius } => Ok((triple(*center), Some(triple(*axis)), *radius)),
        SelectionGeometry::Sphere { center, radius } => Ok((triple(*center), None, *radius)),
        SelectionGeometry::Point { position } => Ok((triple(*position), None, 0.0)),
        _ => Err(ConstraintFailure::unsupported(format!(
            "'{}' cannot follow a cam: pick a roller (cylindrical face or circular edge), a ball, or a vertex",
            element.name
        ))),
    }
}

pub(in crate::feature_pipeline::assembly) fn map(
    entry: &ConstraintEntry,
    a: &ResolvedElement,
    b: &ResolvedElement,
    _env: &Env,
) -> Result<MappedConstraint, ConstraintFailure> {
    let (rider, profile) = match (a.curve.is_some(), b.curve.is_some()) {
        (_, true) => (a, b),
        (true, false) => (b, a),
        (false, false) => {
            return Err(ConstraintFailure::unsupported(
                "Cam needs the cam's profile edge: select the follower and then the profile edge",
            ))
        }
    };
    let (center, axis, radius) = follower(rider)?;
    let curve = profile.curve.clone().expect("checked above");
    // The profile's plane: Newell's normal over samples of the curve.
    let mut normal = Vec3::new(0.0, 0.0, 0.0);
    let count = 64;
    let point = |i: usize| curve.curve.evaluate(curve.t0 + (curve.t1 - curve.t0) * i as f64 / count as f64);
    let mut previous = point(0).map_err(|error| ConstraintFailure::new("error", error))?;
    let first = previous;
    for i in 1..=count {
        let next = if i == count { first } else { point(i).map_err(|error| ConstraintFailure::new("error", error))? };
        normal = normal.add(previous.sub(first).cross(next.sub(first)));
        previous = next;
    }
    let normal = normal.normalized().map_err(|_| {
        ConstraintFailure::unsupported(format!(
            "'{}' is straight: a cam profile is a curved, flat edge (a flat cam face takes a Tangent)",
            profile.name
        ))
    })?;
    let kind = MateKind::Cam {
        center_a: center,
        axis_a: axis,
        radius,
        profile_b: curve,
        normal_b: [normal.x, normal.y, normal.z],
        flip: entry.flag("flip"),
    };
    Ok(MappedConstraint {
        mates: vec![mate(rider, profile, kind)],
        ..Default::default()
    })
}
