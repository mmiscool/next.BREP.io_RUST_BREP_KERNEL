//! Path (`PATH`) — a point of one component follows an edge of another (spec
//! §4, the "path" mechanical mate). Elements: the point (a vertex, or the
//! centre of a sphere, circle or the origin of an axis face) and the path
//! edge (straight, circular or freeform), on two components, in either order.
//! The point is free along the edge unless `pinAlong` holds it at `along`
//! percent of the edge from its start (from its end with `reverse`).

use super::super::mapping::{local_point, mate, ConstraintFailure, MappedConstraint, ResolvedElement};
use super::{elements_field, id_field, pair_applicable, schema_entry, ConstraintTypeDef};
use crate::feature_pipeline::assembly::ConstraintEntry;
use crate::feature_pipeline::{Env, SelectionProbe};
use crate::MateKind;

pub(super) const DEF: ConstraintTypeDef = ConstraintTypeDef {
    type_id: "path",
    label: "Path",
    short_name: "PATH",
    icon: "\u{223F}",
    long_name: "\u{223F} Path",
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
            "Select the point that follows (a vertex, or a sphere, circle or axis) and the path edge, on two components"),
        "pinAlong": {
            "type": "boolean",
            "default_value": false,
            "label": "Fix position",
            "hint": "Hold the point at a set place along the path instead of letting it slide"
        },
        "along": {
            "type": "number",
            "step": 1.0,
            "default_value": 0,
            "label": "Along (%)",
            "hint": "Where along the path, in percent of the edge from its start"
        },
        "reverse": {
            "type": "boolean",
            "default_value": false,
            "hint": "Measure the position from the other end of the path"
        },
    }))
}

pub(in crate::feature_pipeline::assembly) fn map(
    entry: &ConstraintEntry,
    a: &ResolvedElement,
    b: &ResolvedElement,
    env: &Env,
) -> Result<MappedConstraint, ConstraintFailure> {
    // The follower and the path, whichever order they were picked in. A
    // straight or circular edge can be either; the second pick is the path.
    let (follower, path) = match (a.curve.is_some(), b.curve.is_some()) {
        (_, true) => (a, b),
        (true, false) => (b, a),
        (false, false) => {
            return Err(ConstraintFailure::unsupported(
                "Path needs an edge to follow: select the point and then the path edge",
            ))
        }
    };
    let curve = path.curve.clone().expect("checked above");
    let along = if entry.flag("pinAlong") {
        Some(entry.number("along", env, 0.0).map_err(|error| ConstraintFailure::new("error", error))?)
    } else {
        None
    };
    let kind = MateKind::PointOnCurve {
        point_a: local_point(follower),
        curve_b: curve,
        along,
        reverse: entry.flag("reverse"),
    };
    Ok(MappedConstraint {
        mates: vec![mate(follower, path, kind)],
        ..Default::default()
    })
}
