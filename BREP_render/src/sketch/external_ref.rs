//! Project solid edges into a sketch as fixed construction references.
//!
//! Projected polylines become lines, circles, arcs, or line-chain fallbacks.
//! Ground constraints pin the generated points. Edge names identify persisted
//! references, so picking an edge again updates its coordinates in place.
//!
//! The sketch feature's `projectedEdges` list is the ONE record of which edges
//! a sketch links: an edge picked inside the editor is written into that list
//! on Finish, and an edge put into the list from the feature dialog is
//! materialized here on the next enter, through the same classifier. The
//! `persistentData.externalRefs` entries only remember which points and
//! geometry each listed edge became. When a listed edge stops resolving (its
//! feature was deleted, or a regeneration lost the edge) the link is RELEASED:
//! the geometry stays in the sketch as ordinary fixed construction geometry
//! ([`release_ref`]), the entry and the list item go.

use crate::geometry2d::{distance as dist, point_segment_distance};

use std::collections::HashSet;

use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

use super::doc::{id_key, SketchConstraint, SketchDoc, SketchGeometry, SketchPoint};
use super::PlaneFrame;

/// The classification of a projected edge polyline, in plane `(u, v)` coordinates.
#[derive(Clone, Debug, PartialEq)]
pub enum EdgeLink {
    /// A straight edge — link as a `line` through the two endpoints.
    Line { a: (f64, f64), b: (f64, f64) },
    /// A closed circular edge — link as a `circle` (center + a rim point).
    Circle { center: (f64, f64), rim: (f64, f64) },
    /// A circular ARC — link as an `arc` (center, start, end; CCW start→end).
    /// The endpoints are NORMALIZED to that convention: a polyline sampled
    /// clockwise about its center is published with its ends swapped, so the
    /// arc covers the sampled points and never their complement.
    Arc {
        center: (f64, f64),
        start: (f64, f64),
        end: (f64, f64),
    },
    /// A faithful fallback for anything else — a chain of `line` segments through the
    /// projected polyline samples.
    Polyline { pts: Vec<(f64, f64)> },
}

impl EdgeLink {
    /// The classified shape type; `polyline` materializes as a chain of lines.
    pub fn kind(&self) -> &'static str {
        match self {
            EdgeLink::Line { .. } => "line",
            EdgeLink::Circle { .. } => "circle",
            EdgeLink::Arc { .. } => "arc",
            EdgeLink::Polyline { .. } => "polyline",
        }
    }

    /// The ordered `(u, v)` coordinates of the points this link materializes (its
    /// geometry references them in order).
    pub fn point_uvs(&self) -> Vec<(f64, f64)> {
        match self {
            EdgeLink::Line { a, b } => vec![*a, *b],
            EdgeLink::Circle { center, rim } => vec![*center, *rim],
            EdgeLink::Arc { center, start, end } => vec![*center, *start, *end],
            EdgeLink::Polyline { pts } => pts.clone(),
        }
    }
}

/// Project a world-space polyline into the plane's `(u, v)` frame (orthogonal
/// projection; the off-plane component is dropped). Mirrors the previous
/// world→UV projection applied per vertex.
pub fn project_polyline(plane: &PlaneFrame, world: &[[f64; 3]]) -> Vec<(f64, f64)> {
    world.iter().map(|&w| plane.to_uv(w)).collect()
}

/// Classify a projected polyline (in plane `(u, v)`) as a straight line, a circle /
/// arc, or a polyline fallback. Tolerances are RELATIVE to the polyline's extent so
/// the same thresholds work at any sketch scale:
///
/// - **STRAIGHT** when every interior sample lies within `1e-4·extent` of the chord
///   between the endpoints (a 2-sample polyline is trivially straight).
/// - **CIRCULAR** when a circle fit through 3 well-spaced samples has a max radial
///   residual under `1e-3·extent` (and a non-degenerate radius). Coincident
///   endpoints → a closed `Circle`; else an `Arc`.
/// - else the **Polyline** fallback.
pub fn classify_uv(uv: &[(f64, f64)]) -> EdgeLink {
    let n = uv.len();
    if n < 2 {
        // Degenerate — surface it as a (possibly zero-length) polyline; callers guard
        // against < 2 samples before linking.
        return EdgeLink::Polyline { pts: uv.to_vec() };
    }
    let a = uv[0];
    let b = uv[n - 1];
    let extent = polyline_extent(uv).max(1e-9);
    let straight_tol = 1e-4 * extent;
    let closed = dist(a, b) <= straight_tol;

    // Two samples (or all-interior-on-chord and open) → a straight line.
    if !closed {
        let max_dev = uv[1..n - 1]
            .iter()
            .map(|&p| point_segment_distance(p, a, b).0)
            .fold(0.0_f64, f64::max);
        if n == 2 || max_dev <= straight_tol {
            return EdgeLink::Line { a, b };
        }
    }

    // Circle fit through three well-spaced samples. Sampling at 0 / n/3 / 2n/3 (NOT
    // the last index) keeps the three distinct even for a CLOSED loop, where the
    // first and last samples coincide.
    if n >= 3 {
        if let Some((cx, cy, r)) = fit_circle(uv[0], uv[n / 3], uv[(2 * n) / 3]) {
            let circle_tol = 1e-3 * extent;
            let residual = uv
                .iter()
                .map(|&p| (dist(p, (cx, cy)) - r).abs())
                .fold(0.0_f64, f64::max);
            if r.is_finite() && r > straight_tol && residual <= circle_tol {
                if closed {
                    return EdgeLink::Circle {
                        center: (cx, cy),
                        rim: (cx + r, cy),
                    };
                }
                // The sketch convention sweeps CCW start→end (the kernel's
                // `(end − start).rem_euclid(TAU)`, the overlay's `arc_poly`): a
                // polyline that runs CLOCKWISE about the center must publish its
                // ends swapped, or the arc covers the complement of the edge.
                let (start, end) = if winding_ccw(uv, (cx, cy)) { (a, b) } else { (b, a) };
                return EdgeLink::Arc {
                    center: (cx, cy),
                    start,
                    end,
                };
            }
        }
    }

    EdgeLink::Polyline { pts: uv.to_vec() }
}

/// A per-session external-reference mapping: the linked scene edge (by name + owning
/// solid) and the sketch entities materialized for it. Persisted to / loaded from
/// `persistentData.externalRefs` so a linked edge round-trips a commit + re-enter.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExternalRef {
    /// Kernel edge name (the dedup key).
    #[serde(rename = "edgeName")]
    pub edge_name: String,
    /// Owning solid's scene name (metadata; may be empty).
    #[serde(rename = "solidName", default)]
    pub solid_name: String,
    /// The materialized point ids (order matches the link's geometry).
    #[serde(rename = "pointIds", default)]
    pub point_ids: Vec<Value>,
    /// The materialized geometry ids (one for line/circle/arc; N-1 for a polyline
    /// chain).
    #[serde(rename = "geomIds", default)]
    pub geom_ids: Vec<Value>,
    /// The link classification (`"line"|"circle"|"arc"|"polyline"`).
    #[serde(default)]
    pub kind: String,
}

/// Link (or update) a picked scene edge into the sketch as an external reference.
///
/// Projects `world_poly` into `plane`, classifies it, and materializes external-ref
/// points + `⏚` grounds + construction geometry, recording an [`ExternalRef`] in
/// `refs`. Dedup: when `refs` already holds an entry for `edge_name` whose structure
/// matches the new classification, the existing points are UPDATED in place (no
/// duplication); when the structure differs, the old entities are removed and fresh
/// ones created; otherwise a new ref is appended.
///
/// Returns whether the doc actually changed (a new/rebuilt ref, or moved coordinates)
/// — `false` on a redundant re-link of an unchanged edge, so the caller can drop a
/// dead undo step. Never solves; the caller re-solves.
pub fn link_or_update(
    doc: &mut SketchDoc,
    refs: &mut Vec<ExternalRef>,
    edge_name: &str,
    solid_name: &str,
    world_poly: &[[f64; 3]],
    plane: &PlaneFrame,
) -> bool {
    if world_poly.len() < 2 {
        return false;
    }
    let uv = project_polyline(plane, world_poly);
    let link = classify_uv(&uv);
    let new_uvs = link.point_uvs();

    if let Some(pos) = refs.iter().position(|r| r.edge_name == edge_name) {
        let structure_matches =
            refs[pos].kind == link.kind() && refs[pos].point_ids.len() == new_uvs.len();
        if structure_matches {
            // Update the existing points' coordinates in place (keeps ids + geometry).
            let mut moved = false;
            let point_ids = refs[pos].point_ids.clone();
            for (id, (u, v)) in point_ids.iter().zip(new_uvs.iter()) {
                if let Some(p) = doc.point_mut(id) {
                    if (p.x - u).abs() > 1e-12 || (p.y - v).abs() > 1e-12 {
                        moved = true;
                    }
                    p.x = *u;
                    p.y = *v;
                    p.fixed = true;
                    p.construction = true;
                    p.external_reference = true;
                }
            }
            if refs[pos].solid_name != solid_name {
                refs[pos].solid_name = solid_name.to_string();
            }
            return moved;
        }
        // Structure changed (e.g. a straight edge became curved after a model edit) —
        // drop the stale entities and rebuild the ref fresh.
        remove_ref_entities(doc, &refs[pos].clone());
        let (point_ids, geom_ids) = add_external_ref(doc, &link);
        refs[pos] = ExternalRef {
            edge_name: edge_name.to_string(),
            solid_name: solid_name.to_string(),
            point_ids,
            geom_ids,
            kind: link.kind().to_string(),
        };
        return true;
    }

    // A brand-new reference for this edge.
    let (point_ids, geom_ids) = add_external_ref(doc, &link);
    refs.push(ExternalRef {
        edge_name: edge_name.to_string(),
        solid_name: solid_name.to_string(),
        point_ids,
        geom_ids,
        kind: link.kind().to_string(),
    });
    true
}

/// Materialize an [`EdgeLink`] into the doc: push external-ref points (`fixed`,
/// `construction`, `externalReference`) with a `⏚` ground each, then the construction
/// geometry referencing them. Returns `(point_ids, geom_ids)`.
pub fn add_external_ref(doc: &mut SketchDoc, link: &EdgeLink) -> (Vec<Value>, Vec<Value>) {
    let mut point_ids = Vec::new();
    for (u, v) in link.point_uvs() {
        let id = doc.next_point_id();
        doc.points.push(SketchPoint {
            id: id.clone(),
            x: u,
            y: v,
            fixed: true,
            construction: true,
            external_reference: true,
        });
        push_ground(doc, &id);
        point_ids.push(id);
    }
    let geom_ids = match link {
        EdgeLink::Line { .. } => vec![push_construction_geometry(
            doc,
            "line",
            vec![point_ids[0].clone(), point_ids[1].clone()],
        )],
        EdgeLink::Circle { .. } => vec![push_construction_geometry(
            doc,
            "circle",
            vec![point_ids[0].clone(), point_ids[1].clone()],
        )],
        EdgeLink::Arc { .. } => vec![push_construction_geometry(
            doc,
            "arc",
            vec![
                point_ids[0].clone(),
                point_ids[1].clone(),
                point_ids[2].clone(),
            ],
        )],
        EdgeLink::Polyline { .. } => point_ids
            .windows(2)
            .map(|w| push_construction_geometry(doc, "line", vec![w[0].clone(), w[1].clone()]))
            .collect(),
    };
    (point_ids, geom_ids)
}

/// Remove every entity an [`ExternalRef`] materialized: its geometries, its points,
/// and any constraint referencing one of its points (the `⏚` grounds).
fn remove_ref_entities(doc: &mut SketchDoc, r: &ExternalRef) {
    let pt_keys: HashSet<String> = r.point_ids.iter().map(id_key).collect();
    let geo_keys: HashSet<String> = r.geom_ids.iter().map(id_key).collect();
    doc.geometries.retain(|g| !geo_keys.contains(&id_key(&g.id)));
    doc.points.retain(|p| !pt_keys.contains(&id_key(&p.id)));
    doc.constraints
        .retain(|c| !c.points().iter().any(|p| pt_keys.contains(&id_key(p))));
}

/// Drop every external-reference entry whose materialized entities no longer resolve
/// in `doc` — i.e. the linked edge's points / geometry were deleted. A dropped entry
/// frees its `edge_name` so the SAME edge can be RE-LINKED: without this, the stale
/// entry (keyed by `edge_name`, holding now-dangling `point_ids`) makes
/// [`link_or_update`]'s dedup take the "structure matches" branch, find every point
/// missing (`doc.point_mut` → `None`), and return `false` — permanently blocking the
/// re-link. Called both after a delete AND on sketch-enter (to heal docs already
/// poisoned by that pre-fix delete). Returns whether any entry was pruned.
pub fn prune_dead_refs(doc: &SketchDoc, refs: &mut Vec<ExternalRef>) -> bool {
    let before = refs.len();
    refs.retain(|r| {
        r.point_ids.iter().all(|id| doc.point(id).is_some())
            && r.geom_ids.iter().all(|id| doc.geometry(id).is_some())
    });
    refs.len() != before
}

/// The `persistentData` key a unified commit stamps: once it is set, the
/// `projectedEdges` list and the `externalRefs` entries describe the SAME set
/// of edges, so an entry whose edge the dialog no longer lists was removed by
/// the user. A document without it predates the unification: its entries were
/// never listed, so they are adopted into the list, never released.
pub const LINKS_LISTED_KEY: &str = "linksInProjectedEdges";

/// Release a linked edge whose source edge is gone: its materialized points stay
/// where they are, `fixed` and grounded (`⏚`), but stop being external
/// references, so the line / arc / circle becomes an ordinary sketch entity the
/// user owns. It stays CONSTRUCTION: flipping that on a feature deletion would
/// add profile edges and change what a downstream extrude builds. Returns
/// whether any point changed (`false` when the entry's points are all gone).
pub fn release_ref(doc: &mut SketchDoc, r: &ExternalRef) -> bool {
    let mut changed = false;
    for id in &r.point_ids {
        if let Some(p) = doc.point_mut(id) {
            if p.external_reference || !p.fixed {
                changed = true;
            }
            p.external_reference = false;
            p.fixed = true;
        }
    }
    changed
}

/// [`release_ref`] on the persisted JSON doc (`persistentData.sketch`), for the
/// regenerate-time fold that cannot go through the editor: every point the entry
/// materialized loses `externalReference` and keeps (or regains) `fixed`; nothing
/// else in the doc is touched, so the fold is lossless. Returns whether any
/// point changed.
pub fn release_ref_json(sketch: &mut Value, r: &ExternalRef) -> bool {
    let keys: HashSet<String> = r.point_ids.iter().map(id_key).collect();
    let Some(points) = sketch.get_mut("points").and_then(Value::as_array_mut) else {
        return false;
    };
    let mut changed = false;
    for point in points.iter_mut() {
        if !point.get("id").is_some_and(|id| keys.contains(&id_key(id))) {
            continue;
        }
        if point.get("externalReference").and_then(Value::as_bool) != Some(false) {
            changed = true;
        }
        if point.get("fixed").and_then(Value::as_bool) != Some(true) {
            changed = true;
        }
        point["externalReference"] = Value::Bool(false);
        point["fixed"] = Value::Bool(true);
    }
    changed
}

/// The `projectedEdges` list a Finish writes: every name the feature listed
/// before, in its order, minus the ones whose link the session dropped (deleted
/// in the editor, or released), plus every live link the list did not name yet.
/// `kept_unlinked` are listed names that had nothing to materialize at enter
/// (the edge did not resolve in the backdrop) — they stay listed for the kernel
/// to report.
pub fn listed_edge_names(
    listed: &[String],
    refs: &[ExternalRef],
    kept_unlinked: &[String],
) -> Vec<String> {
    let mut out: Vec<String> = listed
        .iter()
        .filter(|name| {
            refs.iter().any(|r| &r.edge_name == *name) || kept_unlinked.contains(name)
        })
        .cloned()
        .collect();
    for r in refs {
        if !out.contains(&r.edge_name) {
            out.push(r.edge_name.clone());
        }
    }
    out
}

/// Ensure the sketch carries the PART ORIGIN reference point: world `(0, 0, 0)`
/// projected onto `plane`, under the reserved id
/// [`brep_kernel::SKETCH_ORIGIN_POINT_ID`], flagged fixed + construction +
/// external reference, with one `⏚` ground (reserved id too). Adds it when
/// missing and re-projects it when the plane moved. It is something to constrain
/// to inside the sketch only — the kernel never publishes it. Returns whether the
/// doc changed.
pub fn ensure_origin_point(doc: &mut SketchDoc, plane: &PlaneFrame) -> bool {
    let id = Value::String(brep_kernel::SKETCH_ORIGIN_POINT_ID.to_string());
    let (u, v) = plane.to_uv([0.0, 0.0, 0.0]);
    let mut changed = false;
    match doc.point_mut(&id) {
        Some(p) => {
            if (p.x - u).abs() > 1e-12 || (p.y - v).abs() > 1e-12 {
                changed = true;
            }
            if !(p.fixed && p.construction && p.external_reference) {
                changed = true;
            }
            p.x = u;
            p.y = v;
            p.fixed = true;
            p.construction = true;
            p.external_reference = true;
        }
        None => {
            doc.points.push(SketchPoint {
                id: id.clone(),
                x: u,
                y: v,
                fixed: true,
                construction: true,
                external_reference: true,
            });
            changed = true;
        }
    }
    let grounded = doc.constraints.iter().any(|c| {
        c.ctype() == Some("⏚") && c.points().first().map(id_key) == Some(id_key(&id))
    });
    if !grounded {
        let mut raw = Map::new();
        raw.insert("id".to_string(), id.clone());
        raw.insert("type".to_string(), Value::String("⏚".to_string()));
        raw.insert("points".to_string(), Value::Array(vec![id]));
        doc.constraints.push(SketchConstraint { raw });
        changed = true;
    }
    changed
}

/// Whether `id` names the reserved part-origin point (or its ground).
pub fn is_origin_id(id: &Value) -> bool {
    brep_kernel::is_sketch_origin_point_id(id)
}

/// Push a `⏚` GROUND constraint pinning point `pid` (mirrors the S4 toggle-ground
/// factory).
fn push_ground(doc: &mut SketchDoc, pid: &Value) {
    let cid = doc.next_constraint_id();
    let mut raw = Map::new();
    raw.insert("id".to_string(), cid);
    raw.insert("type".to_string(), Value::String("⏚".to_string()));
    raw.insert("points".to_string(), Value::Array(vec![pid.clone()]));
    doc.constraints.push(SketchConstraint { raw });
}

/// Push a construction geometry (`construction: true` — dashed, non-modeling) with a
/// freshly minted id, returning that id.
fn push_construction_geometry(doc: &mut SketchDoc, geom_type: &str, points: Vec<Value>) -> Value {
    let id = doc.next_geometry_id();
    let mut extra = Map::new();
    extra.insert("construction".to_string(), Value::Bool(true));
    doc.geometries.push(SketchGeometry {
        id: id.clone(),
        geom_type: geom_type.to_string(),
        points,
        extra,
    });
    id
}

// --- geometry helpers ---------------------------------------------------------

/// The bounding-box diagonal of a `(u, v)` polyline — the relative-tolerance scale.
fn polyline_extent(uv: &[(f64, f64)]) -> f64 {
    let (mut minx, mut miny) = (f64::INFINITY, f64::INFINITY);
    let (mut maxx, mut maxy) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
    for &(x, y) in uv {
        minx = minx.min(x);
        miny = miny.min(y);
        maxx = maxx.max(x);
        maxy = maxy.max(y);
    }
    ((maxx - minx).powi(2) + (maxy - miny).powi(2)).sqrt()
}

/// Whether `uv` travels counter-clockwise about `center`: the sum of the WRAPPED
/// signed angle steps between consecutive samples (each in `(−π, π]`) is positive.
/// Summing the per-step turns, rather than comparing one midpoint's side of the
/// chord, stays correct for sweeps beyond 180°.
fn winding_ccw(uv: &[(f64, f64)], center: (f64, f64)) -> bool {
    use std::f64::consts::{PI, TAU};
    let angle = |p: (f64, f64)| (p.1 - center.1).atan2(p.0 - center.0);
    let total: f64 = uv
        .windows(2)
        .map(|w| {
            let step = angle(w[1]) - angle(w[0]);
            // Wrap into (−π, π] so a sample straddling the ±π cut does not read
            // as an almost-full turn the other way.
            step - TAU * ((step + PI) / TAU).floor()
        })
        .sum();
    total >= 0.0
}

/// Fit a circle through three points (circumcenter + radius), or `None` when the
/// points are (near) collinear.
fn fit_circle(p1: (f64, f64), p2: (f64, f64), p3: (f64, f64)) -> Option<(f64, f64, f64)> {
    let (ax, ay) = p1;
    let (bx, by) = p2;
    let (cx, cy) = p3;
    // 2·(signed area of the triangle) — zero when collinear.
    let d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    if d.abs() < 1e-12 {
        return None;
    }
    let a2 = ax * ax + ay * ay;
    let b2 = bx * bx + by * by;
    let c2 = cx * cx + cy * cy;
    let ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d;
    let uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d;
    let r = dist((ux, uy), p1);
    if !ux.is_finite() || !uy.is_finite() || !r.is_finite() {
        return None;
    }
    Some((ux, uy, r))
}

