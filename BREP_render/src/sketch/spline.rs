//! Insert spline anchors by de Casteljau subdivision without changing the curve.
//!
//! Sketch splines are chained cubic Béziers with `3n + 1` point IDs and anchors
//! at indices 0, 3, 6, … . Splitting `P0 P1 P2 P3` at parameter `t` gives:
//!
//! ```text
//! A = lerp(P0,P1,t)  B = lerp(P1,P2,t)  C = lerp(P2,P3,t)
//! D = lerp(A,B,t)    E = lerp(B,C,t)    S = lerp(D,E,t)
//! ```
//!
//! The resulting polygon `P0, A, D, S, E, C, P3` preserves the `3n + 1` invariant.
//! `P0` and `P3` keep their IDs; `P1` and `P2` are reused for `A` and `C` so their
//! constraints survive. Their endpoint tangent directions are unchanged. Only
//! `D`, `S`, and `E` receive new IDs.
//!
//! The new anchor and its handles are collinear (`S = lerp(D,E,t)`), satisfying
//! the solver's implicit G1 join constraint before the next solve.

use serde_json::Value;

use super::doc::{id_key, SketchDoc, SketchGeometry, SketchPoint};

/// Per-span sampling resolution used to locate the click along a spline — the same
/// 64 the overlay tessellator draws a span with, so "where the curve looks like it
/// is" and "where the click lands on it" agree. The chord projection inside each
/// sample interval recovers the parameter to far better than the sample spacing.
const SPAN_SAMPLES: usize = 64;

/// Refuse a subdivision within this much of either end of the clicked SPAN (in that
/// span's own `0..1` parameter). Splitting at the very end mints a zero-length span,
/// which is a degenerate control polygon, not a refinement. In practice the
/// point-priority rule in [`insert_anchor`] catches these clicks first — an anchor is
/// a point, and points win the pick — so this is the backstop for the sliver between
/// the two radii.
const MIN_SPAN_PARAM: f64 = 1e-3;

/// Whether `geom_type` names a spline. Mirrors the kernel solver's
/// `is_spline_geometry_type`: `"bezier"` is what the tools author, `"spline"` is the
/// accepted alias. Defined here rather than imported because the sketch module is
/// deliberately kernel-free.
pub fn is_spline_type(geom_type: &str) -> bool {
    geom_type == "bezier" || geom_type == "spline"
}

/// The ids [`insert_anchor`] minted: the new on-curve `anchor` and the two off-curve
/// handles flanking it (`before` precedes it in the polygon, `after` follows it). The
/// caller decides what to hang on them — the tool draws each anchor's handle guides.
#[derive(Clone, Debug)]
pub struct InsertedAnchor {
    /// The new on-curve anchor (`S`).
    pub anchor: Value,
    /// The handle immediately BEFORE the anchor (`D`) — it ends the first half-span.
    pub before: Value,
    /// The handle immediately AFTER the anchor (`E`) — it starts the second half-span.
    pub after: Value,
}

/// Insert an anchor into the spline under the plane click `(u, v)`, or `None` when
/// the click should mean something else. Shape-preserving: the curve through the
/// grown control polygon is the curve that was there before.
///
/// The picking rules, in order:
/// * A POINT within `radius` wins — the whole sketcher gives points priority over
///   geometry, and a click on an anchor or handle would split at a span end anyway.
/// * Otherwise the nearest SPLINE within `radius` is the target. Only splines are
///   candidates, deliberately: the bezier tool hangs a dashed construction guide off
///   each end handle, and on a shallow spline (near-collinear handles) those guides
///   hug the curve along its whole length. Ranking all geometry together would let a
///   spline's own guides shadow it and make the very splines a user most wants to
///   refine the ones that cannot be refined.
/// * The clicked span must have four DISTINCT ids and the click must land clear of
///   the span's ends ([`MIN_SPAN_PARAM`]) — a cusp built by repeating an id cannot be
///   subdivided without corrupting the other role, and a split at a span end is
///   degenerate.
///
/// Does NOT solve, refresh or record undo — the engine wrapper owns that.
pub fn insert_anchor(doc: &mut SketchDoc, u: f64, v: f64, radius: f64) -> Option<InsertedAnchor> {
    // Points win the pick, exactly as they do for hover, select, drag and trim — the
    // sketcher has ONE priority rule and a spline click does not get to break it.
    if doc
        .points
        .iter()
        .any(|p| (p.x - u).hypot(p.y - v) <= radius)
    {
        return None;
    }

    let (geo_id, span, t) = nearest_spline_span(doc, u, v, radius)?;
    if !(MIN_SPAN_PARAM..=1.0 - MIN_SPAN_PARAM).contains(&t) {
        return None;
    }

    // Resolve the clicked span's four ids + coordinates BEFORE any mutation: the
    // splice below shifts every id after the split point.
    let ids = {
        let geo = doc.geometry(&geo_id)?;
        let i0 = span * 3;
        [
            geo.points.get(i0)?.clone(),
            geo.points.get(i0 + 1)?.clone(),
            geo.points.get(i0 + 2)?.clone(),
            geo.points.get(i0 + 3)?.clone(),
        ]
    };
    let keys: Vec<String> = ids.iter().map(id_key).collect();
    for i in 0..keys.len() {
        for j in (i + 1)..keys.len() {
            if keys[i] == keys[j] {
                return None; // a repeated id plays two roles — subdividing would corrupt one.
            }
        }
    }
    let controls = span_controls(doc, &ids)?;

    // de Casteljau at `t` — the two halves' control polygons.
    let [p0, p1, p2, p3] = controls;
    let a = lerp(p0, p1, t);
    let b = lerp(p1, p2, t);
    let c = lerp(p2, p3, t);
    let d = lerp(a, b, t);
    let e = lerp(b, c, t);
    let s = lerp(d, e, t);

    // The old handles slide inward onto the new half-spans' outer handles, keeping
    // their ids (and everything constrained to them) attached to the same curve ends.
    // Both resolved a moment ago through `span_controls`, so neither move can miss.
    move_point(doc, &ids[1], a)?;
    move_point(doc, &ids[2], c)?;
    // Minted in polygon order so the ids read left-to-right along the curve.
    let d_id = add_point(doc, d);
    let s_id = add_point(doc, s);
    let e_id = add_point(doc, e);

    // `P0 A | P3` → `P0 A D S E C P3`: the three new ids go between the two reused
    // handles, i.e. right after slot `i0 + 1`.
    let geo = doc.geometry_mut(&geo_id)?;
    let at = span * 3 + 2;
    geo.points
        .splice(at..at, [d_id.clone(), s_id.clone(), e_id.clone()]);

    Some(InsertedAnchor { anchor: s_id, before: d_id, after: e_id })
}

/// The `(geometry id, span index, span parameter)` of the point nearest `(u, v)` over
/// every spline in `doc`, or `None` when none passes within `radius`. Splines whose
/// control count violates `3n + 1` are skipped — they are corrupt, not pickable.
fn nearest_spline_span(doc: &SketchDoc, u: f64, v: f64, radius: f64) -> Option<(Value, usize, f64)> {
    let mut best: Option<(f64, Value, usize, f64)> = None;
    for geo in &doc.geometries {
        if !is_spline_type(&geo.geom_type) {
            continue;
        }
        let ids = &geo.points;
        if ids.len() < 4 || (ids.len() - 1) % 3 != 0 {
            continue;
        }
        for span in 0..(ids.len() - 1) / 3 {
            let i0 = span * 3;
            let Some(controls) = span_controls(
                doc,
                &[
                    ids[i0].clone(),
                    ids[i0 + 1].clone(),
                    ids[i0 + 2].clone(),
                    ids[i0 + 3].clone(),
                ],
            ) else {
                continue;
            };
            let (dist, t) = closest_on_span(&controls, u, v);
            if dist <= radius && best.as_ref().map_or(true, |(bd, ..)| dist < *bd) {
                best = Some((dist, geo.id.clone(), span, t));
            }
        }
    }
    best.map(|(_, id, span, t)| (id, span, t))
}

/// The `(distance, parameter)` of the point on one cubic span nearest `(u, v)`.
/// Sampled at [`SPAN_SAMPLES`] chords, then the click is projected onto the winning
/// chord — the same sampled-polyline treatment trim gives a curve, and accurate to
/// the chord's sagitta (well under a pixel at overlay resolution).
fn closest_on_span(controls: &[[f64; 2]; 4], u: f64, v: f64) -> (f64, f64) {
    let mut prev = eval_cubic(controls, 0.0);
    let mut best = ((u - prev[0]).hypot(v - prev[1]), 0.0);
    for i in 1..=SPAN_SAMPLES {
        let t1 = i as f64 / SPAN_SAMPLES as f64;
        let next = eval_cubic(controls, t1);
        let (dx, dy) = (next[0] - prev[0], next[1] - prev[1]);
        let len2 = (dx * dx + dy * dy).max(1e-24);
        let s = (((u - prev[0]) * dx + (v - prev[1]) * dy) / len2).clamp(0.0, 1.0);
        let dist = (u - (prev[0] + dx * s)).hypot(v - (prev[1] + dy * s));
        if dist < best.0 {
            let t0 = (i - 1) as f64 / SPAN_SAMPLES as f64;
            best = (dist, t0 + (t1 - t0) * s);
        }
        prev = next;
    }
    best
}

/// Evaluate the cubic Bernstein sum over one span's four control points.
fn eval_cubic(controls: &[[f64; 2]; 4], t: f64) -> [f64; 2] {
    let mt = 1.0 - t;
    let (w0, w1, w2, w3) = (
        mt * mt * mt,
        3.0 * mt * mt * t,
        3.0 * mt * t * t,
        t * t * t,
    );
    [
        w0 * controls[0][0] + w1 * controls[1][0] + w2 * controls[2][0] + w3 * controls[3][0],
        w0 * controls[0][1] + w1 * controls[1][1] + w2 * controls[2][1] + w3 * controls[3][1],
    ]
}

/// Resolve four control-point ids to coordinates, or `None` if any is missing.
fn span_controls(doc: &SketchDoc, ids: &[Value; 4]) -> Option<[[f64; 2]; 4]> {
    let mut out = [[0.0; 2]; 4];
    for (slot, id) in ids.iter().enumerate() {
        let p = doc.point(id)?;
        out[slot] = [p.x, p.y];
    }
    Some(out)
}

fn lerp(a: [f64; 2], b: [f64; 2], t: f64) -> [f64; 2] {
    [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]
}

/// Move an existing point to `(x, y)`, or `None` if it vanished.
fn move_point(doc: &mut SketchDoc, id: &Value, at: [f64; 2]) -> Option<()> {
    let p = doc.point_mut(id)?;
    p.x = at[0];
    p.y = at[1];
    Some(())
}

/// Mint a fresh free point at `at` and return its id. Never snaps to a neighbour
/// (unlike [`SketchDoc::snap_or_add_point`]): a subdivision's three new control points
/// are new roles in the polygon even when they land on top of something.
fn add_point(doc: &mut SketchDoc, at: [f64; 2]) -> Value {
    let id = doc.next_point_id();
    doc.points.push(SketchPoint {
        id: id.clone(),
        x: at[0],
        y: at[1],
        fixed: false,
        construction: false,
        external_reference: false,
    });
    id
}

/// If the point pair `(a, b)` is an adjacent (anchor, handle) pair of some spline
/// in `doc` — an on-curve anchor at polygon slot `3i` next to one of its off-curve
/// handles at `3i ± 1`, in either order — return it canonically as
/// `(anchor, handle)`.
///
/// This is the shell's copy of the kernel solver's `spline_anchor_info` (the sketch
/// module is deliberately kernel-free), and it has to stay the SAME predicate: the
/// palette offers `⌒` for exactly the pairs the solver's tangent dispatch reads as a
/// spline tangent, so an offered tangency is one that actually solves.
pub fn anchor_handle_pair(doc: &SketchDoc, a: &Value, b: &Value) -> Option<(Value, Value)> {
    anchor_handle_side(doc, a, b).map(|side| (side.anchor, side.handle))
}

/// One side of a spline joint as the palette sees it: which spline the
/// `(anchor, handle)` pair belongs to and where that anchor sits in the control
/// polygon. The shell's copy of the kernel's `spline_curvature_side`, minus the
/// second control point — the palette only has to decide what to OFFER, and the
/// solver derives the rest from the same polygon.
#[derive(Clone, Debug)]
pub struct GuideSide {
    /// Id of the spline geometry.
    pub geometry: Value,
    /// Polygon slot `3i` of the anchor.
    pub anchor_slot: usize,
    /// Span count `n` of that spline.
    pub seg_count: usize,
    pub anchor: Value,
    pub handle: Value,
}

impl GuideSide {
    /// Whether the anchor is an INTERIOR anchor — the only place the solver's
    /// implied `⏛` already holds both sides' tangents together.
    pub fn interior(&self) -> bool {
        self.anchor_slot > 0 && self.anchor_slot < 3 * self.seg_count
    }
}

/// [`anchor_handle_pair`] with the spline and slot it was found on.
pub fn anchor_handle_side(doc: &SketchDoc, a: &Value, b: &Value) -> Option<GuideSide> {
    let (key_a, key_b) = (id_key(a), id_key(b));
    for geo in &doc.geometries {
        if !is_spline_type(&geo.geom_type) || geo.points.len() < 4 {
            continue;
        }
        let seg_count = (geo.points.len() - 1) / 3;
        for i in 0..=seg_count {
            let anchor_slot = 3 * i;
            let anchor_key = id_key(&geo.points[anchor_slot]);
            if anchor_key != key_a && anchor_key != key_b {
                continue;
            }
            let before = (i > 0).then(|| anchor_slot - 1);
            let after = (i < seg_count).then_some(anchor_slot + 1);
            for slot in [before, after].into_iter().flatten() {
                let handle_key = id_key(&geo.points[slot]);
                let (anchor, handle) = if anchor_key == key_a && handle_key == key_b {
                    (a.clone(), b.clone())
                } else if anchor_key == key_b && handle_key == key_a {
                    (b.clone(), a.clone())
                } else {
                    continue;
                };
                return Some(GuideSide {
                    geometry: geo.id.clone(),
                    anchor_slot,
                    seg_count,
                    anchor,
                    handle,
                });
            }
        }
    }
    None
}

/// The `(anchor, handle)` side a HANDLE GUIDE stands for, when it is one.
pub fn guide_side(doc: &SketchDoc, geo: &SketchGeometry) -> Option<GuideSide> {
    if geo.geom_type != "line" || geo.points.len() < 2 {
        return None;
    }
    anchor_handle_side(doc, &geo.points[0], &geo.points[1])
}

/// Whether two selected geometries are the two sides of ONE curvature joint the
/// solver can hold curvature continuous — the pairs the `ϰ` palette offer is
/// gated on:
///
/// * the two handle guides flanking one INTERIOR anchor of a spline, where the
///   implied `⏛` already provides the tangency, or
/// * two spline END guides whose anchors are joined — the same point, or held
///   together by a coincident — including one spline closed on itself.
///
/// Two ends that merely exist are NOT a joint: with nothing holding the anchors
/// together the constraint would state curvature continuity across a gap.
pub fn is_curvature_joint(doc: &SketchDoc, a: &SketchGeometry, b: &SketchGeometry) -> bool {
    let (Some(side_a), Some(side_b)) = (guide_side(doc, a), guide_side(doc, b)) else {
        return false;
    };
    if id_key(&side_a.handle) == id_key(&side_b.handle) {
        return false;
    }
    let same_anchor = id_key(&side_a.anchor) == id_key(&side_b.anchor);
    if id_key(&side_a.geometry) == id_key(&side_b.geometry)
        && side_a.anchor_slot == side_b.anchor_slot
        && side_a.interior()
        && same_anchor
    {
        return true;
    }
    if side_a.interior() || side_b.interior() {
        return false;
    }
    same_anchor || anchors_coincident(doc, &side_a.anchor, &side_b.anchor)
}

/// Whether a coincident constraint joins the two anchors.
fn anchors_coincident(doc: &SketchDoc, a: &Value, b: &Value) -> bool {
    let (key_a, key_b) = (id_key(a), id_key(b));
    doc.constraints.iter().any(|c| {
        if c.ctype() != Some("≡") {
            return false;
        }
        let keys: Vec<String> = c.points().iter().map(id_key).collect();
        keys.contains(&key_a) && keys.contains(&key_b)
    })
}

/// Whether `geo` is a spline HANDLE GUIDE: the dashed construction line the bezier
/// tool hangs from an anchor to each neighbouring handle (at a drawn end and at an
/// inserted interior anchor alike). Selecting one is how a user grabs the spline's
/// tangent direction there, so the constraint palette reads such a line as the
/// spline — the same reading the solver's tangent dispatch already makes.
pub fn is_handle_guide(doc: &SketchDoc, geo: &SketchGeometry) -> bool {
    guide_side(doc, geo).is_some()
}

/// Whether `geo` is a handle guide at a spline END anchor — where a curvature
/// constraint against a circle or arc is offered. The solver accepts the same
/// residual at an interior anchor (that side's curvature takes the circle's),
/// but an end is where continuing a curve into an arc is the question being
/// asked, and it is the case the plan derives.
pub fn is_end_handle_guide(doc: &SketchDoc, geo: &SketchGeometry) -> bool {
    guide_side(doc, geo).is_some_and(|side| !side.interior())
}

// ---------------------------------------------------------------------------
// Test-only geometry readers.
//
// Shared by this module's unit tests AND the engine-level tool tests
// (`engine_state::sketch_mode_tests`), so "the curve did not move" is measured by
// ONE implementation in both places — analytically, against the control polygons,
// never through a sampled polyline (whose chord gaps run to ~1e-2 at any sane sample
// count and would swallow the deviation these tests exist to catch).
// ---------------------------------------------------------------------------






