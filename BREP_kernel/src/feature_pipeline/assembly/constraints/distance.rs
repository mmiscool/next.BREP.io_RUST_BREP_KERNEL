//! Distance (`DIST`) — hold two elements at a target separation.
//!
//! SIGN CONVENTION (shared verbatim with the viewport arrow's drag — the two
//! must never disagree): when a pairing involves a planar face, that face is
//! the BASE face (element 0 when both are planes) and the target is the SIGNED
//! offset of the other element measured along the base face's outward normal —
//! `d = (P_other − P_base)·n̂_base`. A negative target therefore places the
//! other element BEHIND the base face; the solver's `DistancePlanePlane` /
//! `DistancePointPlane` residuals measure exactly this quantity, so the value
//! maps through unchanged. Pairings without a plane (lines/points — no side to
//! be on) use the magnitude `|d|`.
//!
//! Measurement classification: planes stay planes; straight edges and
//! cylindrical/conical faces measure on their INFINITE carrier lines; circular
//! edges and spheres from their CENTERS; vertices and whole components from
//! their points. A new constraint adopts the current (signed) separation on
//! first solve.

use super::super::mapping::{
    effective_align, first_solve_target, line_line_distance, local_point, mate,
    point_line_distance, require_axis, require_direction, require_plane, ConstraintFailure,
    MappedConstraint, ResolvedElement,
};
use super::{elements_field, id_field, pair_applicable, schema_entry, ConstraintTypeDef};
use crate::feature_pipeline::assembly::ConstraintEntry;
use crate::feature_pipeline::{Env, SelectionProbe};
use crate::{MateKind, SelectionGeometry, Vec3};

pub(super) const DEF: ConstraintTypeDef = ConstraintTypeDef {
    type_id: "distance",
    label: "Distance",
    short_name: "DIST",
    icon: "\u{27FA}",
    long_name: "\u{27FA} Distance",
    min_elements: 2,
    max_elements: 2,
    duplicate_family: true,
    applicable,
};

/// Distance pairs faces/edges (vertex refs come from the modal picker only).
fn applicable(probe: &SelectionProbe) -> bool {
    pair_applicable(probe, probe.faces + probe.edges)
}

pub(super) fn schema() -> serde_json::Value {
    schema_entry(&DEF, serde_json::json!({
        "id": id_field(),
        "elements": elements_field(&["FACE", "VERTEX", "EDGE"], 2, 2,
            "Select two elements to hold at a distance"),
        "distance": {
            "type": "number",
            "step": 0.1,
            "default_value": 0,
            "hint": "Signed target: offset along the base face's outward normal (negative = the opposite side); pairings without a face use the magnitude. A new constraint adopts the current separation"
        },
        "opposeNormals": {
            "type": "boolean",
            "default_value": false,
            "hint": "Face-face mode: flip the second face's normal before alignment"
        },
    }))
}

pub(in crate::feature_pipeline::assembly) fn map(
    entry: &mut ConstraintEntry,
    a: &ResolvedElement,
    b: &ResolvedElement,
    env: &Env,
) -> Result<MappedConstraint, ConstraintFailure> {
    // SIGNED as authored: plane-based pairings consume the sign directly (the
    // module-doc convention); the plane-less arms below take |target| — a
    // line/point separation has no side for the sign to select.
    let target = entry
        .number("distance", env, 0.0)
        .map_err(|error| ConstraintFailure::new("error", error))?;

    // Distance measurement classification: planes stay planes; straight edges
    // and cylindrical/conical faces measure on their INFINITE carrier lines;
    // circular edges and spheres measure from their CENTERS; vertices and
    // whole components from their points.
    enum Side {
        Plane,
        Line(Vec3, Vec3),
        Point(Vec3),
    }
    let classify = |element: &ResolvedElement| match element.world {
        SelectionGeometry::Plane { .. } => Side::Plane,
        SelectionGeometry::Line { origin, direction }
        | SelectionGeometry::Axis {
            origin, direction, ..
        } => Side::Line(origin, direction),
        SelectionGeometry::Circle { center, .. } | SelectionGeometry::Sphere { center, .. } => {
            Side::Point(center)
        }
        SelectionGeometry::Point { position } => Side::Point(position),
    };

    match (classify(a), classify(b)) {
        // FACE/FACE: the SIGNED offset of plane B along plane A's outward
        // normal — `d = (P_b − P_a)·n̂_a`, exactly what the mate's residual
        // measures, so the authored target maps through verbatim (negative =
        // plane B behind face A). First solve adopts the current SIGNED
        // separation — honest about the side, and satisfied without motion.
        (Side::Plane, Side::Plane) => {
            let (wa, _) = require_direction(a)?;
            let (wb, _) = require_direction(b)?;
            let oppose = entry.flag("opposeNormals");
            let align = effective_align(entry, wa, wb, oppose);
            let separation = b
                .world
                .representative_point()
                .sub(a.world.representative_point())
                .dot(wa);
            let (signed, pending) = first_solve_target(entry, "distance", "initializedDistance", target, separation);
            Ok(MappedConstraint {
                mates: vec![mate(a, b, MateKind::DistancePlanePlane {
                    plane_a: require_plane(a)?,
                    plane_b: require_plane(b)?,
                    distance: signed,
                    align,
                })],
                pending_params: pending.0,
                pending_persistent: pending.1,
                measured: Some((separation, "mm")),
                target: Some(signed),
                ..Default::default()
            })
        }
        // FACE/point-like (or line-like): the plane is the BASE face whichever
        // side of the pair it is — the target is the SIGNED height of the
        // point above its outward normal (negative = behind the face), the
        // same quantity the `DistancePointPlane` residual measures.
        (Side::Plane, other) | (other, Side::Plane) => {
            let (plane, point_side) = if matches!(classify(a), Side::Plane) {
                (a, b)
            } else {
                (b, a)
            };
            let point_world = match other {
                Side::Line(origin, _) => origin,
                Side::Point(point) => point,
                Side::Plane => unreachable!("both-plane handled above"),
            };
            let (normal, _) = require_direction(plane)?;
            let height = point_world
                .sub(plane.world.representative_point())
                .dot(normal);
            Ok(MappedConstraint {
                mates: vec![mate(point_side, plane, MateKind::DistancePointPlane {
                    point_a: local_point(point_side),
                    plane_b: require_plane(plane)?,
                    distance: target,
                })],
                measured: Some((height, "mm")),
                target: Some(target),
                ..Default::default()
            })
        }
        // No base plane below here: a line/point separation has no side to be
        // on, so the target is a MAGNITUDE — `|target|`.
        (Side::Line(oa, da), Side::Line(ob, db)) => Ok(MappedConstraint {
            mates: vec![mate(a, b, MateKind::DistanceLineLine {
                axis_a: require_axis(a)?,
                axis_b: require_axis(b)?,
                distance: target.abs(),
            })],
            measured: Some((line_line_distance(oa, da, ob, db), "mm")),
            target: Some(target.abs()),
            ..Default::default()
        }),
        (Side::Point(point), Side::Line(origin, direction)) => Ok(MappedConstraint {
            mates: vec![mate(a, b, MateKind::DistancePointLine {
                point_a: local_point(a),
                axis_b: require_axis(b)?,
                distance: target.abs(),
            })],
            measured: Some((point_line_distance(point, origin, direction), "mm")),
            target: Some(target.abs()),
            ..Default::default()
        }),
        (Side::Line(origin, direction), Side::Point(point)) => Ok(MappedConstraint {
            mates: vec![mate(b, a, MateKind::DistancePointLine {
                point_a: local_point(b),
                axis_b: require_axis(a)?,
                distance: target.abs(),
            })],
            measured: Some((point_line_distance(point, origin, direction), "mm")),
            target: Some(target.abs()),
            ..Default::default()
        }),
        (Side::Point(pa), Side::Point(pb)) => Ok(MappedConstraint {
            mates: vec![mate(a, b, MateKind::DistancePointPoint {
                point_a: local_point(a),
                point_b: local_point(b),
                distance: target.abs(),
            })],
            measured: Some((pa.sub(pb).length(), "mm")),
            target: Some(target.abs()),
            ..Default::default()
        }),
    }
}
