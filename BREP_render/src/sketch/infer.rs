//! Infer coincidence or point-on-line constraints after a single-point drag.
//!
//! Existing coincidence paths are checked before adding a constraint. Drawn
//! endpoints instead share point IDs through `SketchDoc::snap_or_add_point`;
//! inference acts on distinct existing points and adds removable constraints.

use crate::sketch::doc::{id_key, SketchDoc};
use crate::sketch::trim::add_constraint_if_missing;
use serde_json::Value;
use std::collections::HashSet;

/// BFS over the `≡` graph: are `a` and `b` already (transitively) coincident?
/// Ported from the previous app so drop-inference never adds a
/// duplicate or contradictory coincidence.
pub(crate) fn are_points_coincident(doc: &SketchDoc, a: &Value, b: &Value) -> bool {
    let (ak, bk) = (id_key(a), id_key(b));
    if ak == bk {
        return true;
    }
    let mut visited = HashSet::new();
    visited.insert(ak.clone());
    let mut stack = vec![ak];
    while let Some(cur) = stack.pop() {
        for c in &doc.constraints {
            if c.ctype() != Some("≡") {
                continue;
            }
            let pts = c.points();
            if pts.len() < 2 {
                continue;
            }
            let (p0, p1) = (id_key(&pts[0]), id_key(&pts[1]));
            let next = if p0 == cur {
                Some(p1)
            } else if p1 == cur {
                Some(p0)
            } else {
                None
            };
            if let Some(n) = next {
                if n == bk {
                    return true;
                }
                if visited.insert(n.clone()) {
                    stack.push(n);
                }
            }
        }
    }
    false
}

/// After a single-point drag ends, infer a coincident (`≡`) or point-on-line
/// (`⏛` / endpoint `≡`) constraint from the dropped position. `world_tol` is the
/// drop radius in world units (the point grab radius). Returns whether a
/// constraint was added. Coincidence wins over point-on-line (a point dropped on
/// a line's endpoint should coincide with the endpoint, not slide on the line).
pub(crate) fn infer_drop_constraint(doc: &mut SketchDoc, dragged: &Value, world_tol: f64) -> bool {
    if maybe_add_coincident(doc, dragged, world_tol) {
        return true;
    }
    maybe_add_point_on_line(doc, dragged, world_tol)
}

/// `#maybeAddCoincidentOnDrop`: add `≡` to the NEAREST other point within `tol`
/// that is not already coincident and not a sibling endpoint of a geometry the
/// dragged point belongs to (coinciding with a sibling would collapse the curve).
fn maybe_add_coincident(doc: &mut SketchDoc, dragged: &Value, tol: f64) -> bool {
    let Some(p) = doc.point(dragged) else {
        return false;
    };
    let (px, py) = (p.x, p.y);
    let dk = id_key(dragged);
    // Every point sharing a geometry with the dragged point — never coincide with
    // one (it would zero-length a line / collapse an arc).
    let siblings: HashSet<String> = doc
        .geometries
        .iter()
        .filter(|g| g.points.iter().any(|pid| id_key(pid) == dk))
        .flat_map(|g| g.points.iter().map(id_key))
        .collect();
    let mut best: Option<(f64, Value)> = None;
    for q in &doc.points {
        let qk = id_key(&q.id);
        if qk == dk || siblings.contains(&qk) {
            continue;
        }
        let d = (q.x - px).hypot(q.y - py);
        if d > tol {
            continue;
        }
        if are_points_coincident(doc, dragged, &q.id) {
            continue;
        }
        if best.as_ref().map_or(true, |(bd, _)| d < *bd) {
            best = Some((d, q.id.clone()));
        }
    }
    match best {
        Some((_, target)) => add_constraint_if_missing(doc, "≡", vec![target, dragged.clone()]),
        None => false,
    }
}

/// `#maybeAddPointOnLineOnDrop`: pin the dropped point onto the NEAREST line
/// segment within `tol` (perpendicular distance) — `≡` when it lands on an
/// endpoint, else `⏛`. Skips lines the point is an endpoint of, and lines whose
/// endpoint is already coincident with it (a redundant `⏛` would eat a DOF).
fn maybe_add_point_on_line(doc: &mut SketchDoc, dragged: &Value, tol: f64) -> bool {
    let Some(p) = doc.point(dragged) else {
        return false;
    };
    let (px, py) = (p.x, p.y);
    let dk = id_key(dragged);
    let mut best: Option<(f64, Value, f64)> = None; // (perp dist, line id, param t)
    for g in &doc.geometries {
        if g.geom_type != "line" || g.points.len() < 2 {
            continue;
        }
        if g.points.iter().any(|pid| id_key(pid) == dk) {
            continue;
        }
        let (a_id, b_id) = (&g.points[0], &g.points[1]);
        if are_points_coincident(doc, dragged, a_id) || are_points_coincident(doc, dragged, b_id) {
            continue;
        }
        let (Some(a), Some(b)) = (doc.point(a_id), doc.point(b_id)) else {
            continue;
        };
        let (ax, ay, dx, dy) = (a.x, a.y, b.x - a.x, b.y - a.y);
        let len2 = dx * dx + dy * dy;
        if len2 <= 1e-12 {
            continue;
        }
        let t = (((px - ax) * dx + (py - ay) * dy) / len2).clamp(0.0, 1.0);
        let dist = (px - (ax + dx * t)).hypot(py - (ay + dy * t));
        if dist > tol {
            continue;
        }
        if best.as_ref().map_or(true, |(bd, _, _)| dist < *bd) {
            best = Some((dist, g.id.clone(), t));
        }
    }
    let Some((_, line_id, t)) = best else {
        return false;
    };
    let Some(g) = doc.geometry(&line_id).cloned() else {
        return false;
    };
    let (a_id, b_id) = (g.points[0].clone(), g.points[1].clone());
    const ENDPOINT_EPS: f64 = 1e-3;
    if t <= ENDPOINT_EPS {
        add_constraint_if_missing(doc, "≡", vec![a_id, dragged.clone()])
    } else if t >= 1.0 - ENDPOINT_EPS {
        add_constraint_if_missing(doc, "≡", vec![b_id, dragged.clone()])
    } else {
        add_constraint_if_missing(doc, "⏛", vec![a_id, b_id, dragged.clone()])
    }
}

