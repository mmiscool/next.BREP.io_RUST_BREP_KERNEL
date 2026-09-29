//! Trim sketch geometry between sampled-polyline intersections around the click.
//!
//! Samples carry curve parameters for choosing trim bounds. Lines split into
//! segments, circles become arcs, arcs shorten, and Béziers keep surviving
//! control-point ranges. Without bracketing intersections, the geometry is deleted.
//!
//! Cut points are pinned onto the crossing line or arc/circle, using coincidence
//! near line endpoints. A surviving arc carries its radius through its endpoints;
//! the trimmed circle's old radius point need not survive. Unreferenced points
//! are removed after each trim by [`cleanup_orphan_points`].

use std::cmp::Ordering;
use std::collections::HashSet;
use std::f64::consts::PI;

use serde_json::{Map, Value};

use super::doc::{id_key, SketchConstraint, SketchDoc, SketchGeometry, SketchPoint};

/// Circle sampling resolution for trim (the previous 96-gon — finer
/// than the S2 overlay's 64-gon so intersection params land accurately).
const CIRCLE_SAMPLES: usize = 96;
/// Bezier per-span sampling resolution (the previous 24/span).
const BEZIER_SEG_SAMPLES: usize = 24;
/// Snap radius when reusing an existing point for a cut vertex (`#findExistingPointId`).
const SNAP_EPS: f64 = 1e-6;

// ---------------------------------------------------------------------------
// Sampling — geometry → (x, y, param) polyline (a port of `#sampleGeometry`).
// ---------------------------------------------------------------------------

/// One polyline sample: an in-plane `(x, y)` point plus its curve parameter (the
/// value `select_trim_bounds` brackets the click against).
#[derive(Clone, Copy, Debug)]
struct Sample {
    x: f64,
    y: f64,
    param: f64,
}

/// A sampled geometry: its polyline plus the metadata the bracket logic needs —
/// `closed` (a full circle / full arc wraps), `max_param` (1 for line/circle/arc,
/// `seg_count` for a bezier), and `seg_count` (bezier spans).
struct Sampled {
    closed: bool,
    max_param: f64,
    seg_count: usize,
    samples: Vec<Sample>,
}

/// Sample one geometry into a `(x, y, param)` polyline, or `None` if a referenced
/// point is missing or the geometry is degenerate (a faithful port of
/// `#sampleGeometry`).
fn sample_geometry(geo: &SketchGeometry, doc: &SketchDoc) -> Option<Sampled> {
    let ids = &geo.points;
    match geo.geom_type.as_str() {
        "line" if ids.len() >= 2 => {
            let p0 = doc.point(&ids[0])?;
            let p1 = doc.point(&ids[1])?;
            Some(Sampled {
                closed: false,
                max_param: 1.0,
                seg_count: 1,
                samples: vec![
                    Sample { x: p0.x, y: p0.y, param: 0.0 },
                    Sample { x: p1.x, y: p1.y, param: 1.0 },
                ],
            })
        }
        "circle" if ids.len() >= 2 => {
            let pc = doc.point(&ids[0])?;
            let pr = doc.point(&ids[1])?;
            let r = (pr.x - pc.x).hypot(pr.y - pc.y);
            if !r.is_finite() || r < 1e-9 {
                return None;
            }
            let mut samples = Vec::with_capacity(CIRCLE_SAMPLES + 1);
            for i in 0..=CIRCLE_SAMPLES {
                let t = i as f64 / CIRCLE_SAMPLES as f64;
                let a = t * 2.0 * PI;
                samples.push(Sample {
                    x: pc.x + r * a.cos(),
                    y: pc.y + r * a.sin(),
                    param: t,
                });
            }
            Some(Sampled { closed: true, max_param: 1.0, seg_count: 1, samples })
        }
        "arc" if ids.len() >= 3 => {
            let pc = doc.point(&ids[0])?;
            let pa = doc.point(&ids[1])?;
            let pb = doc.point(&ids[2])?;
            let r = (pa.x - pc.x).hypot(pa.y - pc.y);
            if !r.is_finite() || r < 1e-9 {
                return None;
            }
            let a0 = (pa.y - pc.y).atan2(pa.x - pc.x);
            let a1 = (pb.y - pc.y).atan2(pb.x - pc.x);
            let mut d = (a1 - a0).rem_euclid(2.0 * PI);
            let full = d < 1e-6;
            if full {
                d = 2.0 * PI;
            }
            let segs = ((CIRCLE_SAMPLES as f64 * d / (2.0 * PI)).ceil() as usize).max(8);
            let mut samples = Vec::with_capacity(segs + 1);
            for i in 0..=segs {
                let t = i as f64 / segs as f64;
                let a = a0 + d * t;
                samples.push(Sample {
                    x: pc.x + r * a.cos(),
                    y: pc.y + r * a.sin(),
                    param: t,
                });
            }
            Some(Sampled { closed: full, max_param: 1.0, seg_count: 1, samples })
        }
        "bezier" if ids.len() >= 4 => {
            let seg_count = (ids.len() - 1) / 3;
            if seg_count < 1 {
                return None;
            }
            let mut samples = Vec::new();
            for seg in 0..seg_count {
                let i0 = seg * 3;
                let p0 = doc.point(&ids[i0])?;
                let p1 = doc.point(&ids[i0 + 1])?;
                let p2 = doc.point(&ids[i0 + 2])?;
                let p3 = doc.point(&ids[i0 + 3])?;
                for i in 0..=BEZIER_SEG_SAMPLES {
                    if seg > 0 && i == 0 {
                        continue; // shared knot already emitted
                    }
                    let t = i as f64 / BEZIER_SEG_SAMPLES as f64;
                    let mt = 1.0 - t;
                    let bx = mt * mt * mt * p0.x
                        + 3.0 * mt * mt * t * p1.x
                        + 3.0 * mt * t * t * p2.x
                        + t * t * t * p3.x;
                    let by = mt * mt * mt * p0.y
                        + 3.0 * mt * mt * t * p1.y
                        + 3.0 * mt * t * t * p2.y
                        + t * t * t * p3.y;
                    samples.push(Sample { x: bx, y: by, param: seg as f64 + t });
                }
            }
            Some(Sampled {
                closed: false,
                max_param: seg_count as f64,
                seg_count,
                samples,
            })
        }
        _ => None,
    }
}

/// The curve param of the point on `samples` nearest to `(px, py)` (a port of
/// `#closestParamOnSamples`/`#closestPointOnSamples`), or `None` for < 2 samples.
fn closest_param_on_samples(px: f64, py: f64, samples: &[Sample]) -> Option<f64> {
    closest_point_on_samples(px, py, samples).map(|(param, _)| param)
}

/// The `(param, dist)` of the closest point on `samples` to `(px, py)`.
fn closest_point_on_samples(px: f64, py: f64, samples: &[Sample]) -> Option<(f64, f64)> {
    if samples.len() < 2 {
        return None;
    }
    let mut best_param = samples[0].param;
    let mut best_dist = f64::INFINITY;
    for w in samples.windows(2) {
        let (a, b) = (w[0], w[1]);
        let vx = b.x - a.x;
        let vy = b.y - a.y;
        let l2 = (vx * vx + vy * vy).max(1e-12);
        let t = (((px - a.x) * vx + (py - a.y) * vy) / l2).clamp(0.0, 1.0);
        let nx = a.x + vx * t;
        let ny = a.y + vy * t;
        let d = (px - nx).hypot(py - ny);
        if d < best_dist {
            best_dist = d;
            best_param = a.param + (b.param - a.param) * t;
        }
    }
    Some((best_param, best_dist))
}

/// The bounding-box-diagonal-derived tolerance for an endpoint touch (a port of
/// `#sampleTol`): `clamp(diag * 1e-3, 1e-5, 1e-2)`.
fn sample_tol(samples: &[Sample]) -> f64 {
    if samples.is_empty() {
        return 1e-3;
    }
    let (mut min_x, mut min_y, mut max_x, mut max_y) =
        (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for s in samples {
        min_x = min_x.min(s.x);
        min_y = min_y.min(s.y);
        max_x = max_x.max(s.x);
        max_y = max_y.max(s.y);
    }
    let diag = (max_x - min_x).hypot(max_y - min_y);
    if !diag.is_finite() || diag < 1e-9 {
        return 1e-3;
    }
    (diag * 1e-3).clamp(1e-5, 1e-2)
}

// ---------------------------------------------------------------------------
// Segment-segment intersection + intersection collection.
// ---------------------------------------------------------------------------

/// A segment-segment crossing: the world point plus the clamped parameters on each
/// segment (`ta` on the target segment, `tb` on the other).
#[derive(Clone, Copy, Debug)]
struct SegHit {
    x: f64,
    y: f64,
    ta: f64,
    tb: f64,
}

/// Intersect segments `a→b` and `c→d` (a port of `#segmentIntersection`). Two
/// segments `(a, a+r)` and `(c, c+s)` cross where `t = ((c−a)×s)/(r×s)` and
/// `u = ((c−a)×r)/(r×s)` both lie in `[0, 1]`; `None` when `r×s ≈ 0` (parallel /
/// collinear) or the hit is off either segment.
fn segment_intersection(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2], eps: f64) -> Option<SegHit> {
    let rdx = b[0] - a[0];
    let rdy = b[1] - a[1];
    let sdx = d[0] - c[0];
    let sdy = d[1] - c[1];
    let denom = rdx * sdy - rdy * sdx;
    if denom.abs() < eps {
        return None;
    }
    let t = ((c[0] - a[0]) * sdy - (c[1] - a[1]) * sdx) / denom;
    let u = ((c[0] - a[0]) * rdy - (c[1] - a[1]) * rdx) / denom;
    if t < -eps || t > 1.0 + eps || u < -eps || u > 1.0 + eps {
        return None;
    }
    let tt = t.clamp(0.0, 1.0);
    Some(SegHit {
        x: a[0] + rdx * tt,
        y: a[1] + rdy * tt,
        ta: tt,
        tb: u.clamp(0.0, 1.0),
    })
}

/// One intersection of the target curve with another geometry: the target param,
/// the world point, the other curve's param (for endpoint proximity), the other
/// geometry's id (for the cut constraint), and whether it was an endpoint touch.
#[derive(Clone, Debug)]
struct Intersection {
    param: f64,
    x: f64,
    y: f64,
    other_param: Option<f64>,
    other_geo_id: Value,
    endpoint_snap: bool,
}

/// Collect crossings of `target` with `other` (a port of `#collectIntersections`):
/// every target-segment × other-segment crossing, PLUS the other curve's endpoints
/// (its first/last sample, when it is not closed) that touch the target within
/// tolerance (`#collectEndpointIntersectionsOnTarget`).
fn collect_intersections(
    target: &Sampled,
    other: &Sampled,
    other_geo: &SketchGeometry,
    out: &mut Vec<Intersection>,
) {
    let a = &target.samples;
    let b = &other.samples;
    if a.len() < 2 || b.len() < 2 {
        return;
    }
    for i in 0..a.len() - 1 {
        let (a0, a1) = (a[i], a[i + 1]);
        for j in 0..b.len() - 1 {
            let (b0, b1) = (b[j], b[j + 1]);
            if let Some(hit) =
                segment_intersection([a0.x, a0.y], [a1.x, a1.y], [b0.x, b0.y], [b1.x, b1.y], 1e-9)
            {
                out.push(Intersection {
                    param: a0.param + (a1.param - a0.param) * hit.ta,
                    x: hit.x,
                    y: hit.y,
                    other_param: Some(b0.param + (b1.param - b0.param) * hit.tb),
                    other_geo_id: other_geo.id.clone(),
                    endpoint_snap: false,
                });
            }
        }
    }
    // Endpoint touches: a non-closed curve's first/last sample landing on the target
    // (a T-junction the segment crossing above may miss). Circles/full arcs have no
    // endpoints (they are closed), matching `#getGeometryEndpointInfos`.
    if !other.closed {
        let tol = sample_tol(&target.samples);
        for end in [b.first(), b.last()].into_iter().flatten() {
            if let Some((param, dist)) = closest_point_on_samples(end.x, end.y, &target.samples) {
                if param.is_finite() && dist.is_finite() && dist <= tol {
                    out.push(Intersection {
                        param,
                        x: end.x,
                        y: end.y,
                        other_param: Some(end.param),
                        other_geo_id: other_geo.id.clone(),
                        endpoint_snap: true,
                    });
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Bracket selection — the two intersections around the click (`#selectTrimBounds`).
// ---------------------------------------------------------------------------

/// The two intersections bracketing the click, plus the target's `max_param` (used
/// by the circle-span check). `prev`/`next` are `None` when the click is before the
/// first / after the last intersection on an open curve.
struct Bounds {
    prev: Option<Intersection>,
    next: Option<Intersection>,
    max_param: f64,
}

/// Choose the two intersections bracketing `click_param` (a port of
/// `#selectTrimBounds`): normalize + sort + dedup the intersection params (a
/// closed curve wraps mod `max_param`, an open one drops the endpoints), then for a
/// closed curve take the nearest crossing ahead (`next`) and behind (`prev`) the
/// click going CCW; for an open curve take the last crossing before and first after
/// the click. `None` when nothing brackets the click.
fn select_trim_bounds(intersections: &[Intersection], click_param: f64, target: &Sampled) -> Option<Bounds> {
    let max_param = if target.max_param > 0.0 { target.max_param } else { 1.0 };
    let param_eps = (1e-5f64).max(max_param * 1e-4);

    let mut cleaned: Vec<Intersection> = Vec::new();
    for inter in intersections {
        if !inter.param.is_finite() {
            continue;
        }
        let mut p = inter.param;
        if target.closed {
            p = p.rem_euclid(max_param);
            if p < param_eps || p > max_param - param_eps {
                p = 0.0;
            }
        } else if p <= param_eps || p >= max_param - param_eps {
            continue;
        }
        let mut c = inter.clone();
        c.param = p;
        cleaned.push(c);
    }
    cleaned.sort_by(|a, b| a.param.partial_cmp(&b.param).unwrap_or(Ordering::Equal));

    // Dedup near-equal params; a genuine endpoint touch supersedes a coincident
    // segment crossing at the same spot.
    let mut uniq: Vec<Intersection> = Vec::new();
    for inter in cleaned {
        match uniq.last() {
            Some(prev) if (inter.param - prev.param).abs() <= param_eps => {
                if !prev.endpoint_snap && inter.endpoint_snap {
                    *uniq.last_mut().unwrap() = inter;
                }
            }
            _ => uniq.push(inter),
        }
    }

    if target.closed {
        if uniq.len() < 2 {
            return None;
        }
        let mut prev = None;
        let mut next = None;
        let mut best_next = f64::INFINITY;
        let mut best_prev = f64::NEG_INFINITY;
        for inter in &uniq {
            let delta = (inter.param - click_param).rem_euclid(max_param);
            if delta < param_eps {
                continue;
            }
            if delta < best_next {
                best_next = delta;
                next = Some(inter.clone());
            }
            if delta > best_prev {
                best_prev = delta;
                prev = Some(inter.clone());
            }
        }
        match (prev, next) {
            (Some(prev), Some(next)) => Some(Bounds { prev: Some(prev), next: Some(next), max_param }),
            _ => None,
        }
    } else {
        if uniq.is_empty() {
            return None;
        }
        let mut prev = None;
        let mut next = None;
        for inter in &uniq {
            if inter.param < click_param - param_eps {
                prev = Some(inter.clone());
            } else if inter.param > click_param + param_eps {
                next = Some(inter.clone());
                break;
            }
        }
        if prev.is_none() && next.is_none() {
            return None;
        }
        Some(Bounds { prev, next, max_param })
    }
}

// ---------------------------------------------------------------------------
// Doc mutation helpers — point/geometry factory + orphan sweep.
// ---------------------------------------------------------------------------

/// Reuse an existing point within `SNAP_EPS` of `(x, y)`, else mint a fresh one
/// (`#getOrCreatePointId`).
fn get_or_create_point(doc: &mut SketchDoc, x: f64, y: f64) -> Value {
    doc.snap_or_add_point(x, y, SNAP_EPS)
}

/// Always mint a fresh point at `(x, y)` (`#createPointAtUV`) — the bezier split
/// anchors, which must not collapse onto a neighbor.
fn create_point(doc: &mut SketchDoc, x: f64, y: f64) -> Value {
    let id = doc.next_point_id();
    doc.points.push(SketchPoint {
        id: id.clone(),
        x,
        y,
        fixed: false,
        construction: false,
        external_reference: false,
    });
    id
}

/// Append a geometry with a fresh id, inheriting `construction` from the trimmed
/// source (`#addGeometry`). Returns the new id.
fn add_geometry(doc: &mut SketchDoc, geom_type: &str, points: Vec<Value>, construction: bool) -> Value {
    let id = doc.next_geometry_id();
    let mut extra = Map::new();
    extra.insert("construction".to_string(), Value::Bool(construction));
    doc.geometries.push(SketchGeometry {
        id: id.clone(),
        geom_type: geom_type.to_string(),
        points,
        extra,
    });
    id
}

/// Remove the geometry with `id`; returns whether one was removed.
fn remove_geometry(doc: &mut SketchDoc, id: &Value) -> bool {
    let key = id_key(id);
    let before = doc.geometries.len();
    doc.geometries.retain(|g| id_key(&g.id) != key);
    doc.geometries.len() != before
}

/// Drop points referenced by neither a surviving geometry nor a surviving
/// constraint (so a trimmed-away stub leaves no dangling point/constraint). Points
/// still named by a constraint (e.g. a crossing arc's rim pinned by `⇌`) are kept.
fn cleanup_orphan_points(doc: &mut SketchDoc) {
    let mut referenced: HashSet<String> = HashSet::new();
    for g in &doc.geometries {
        for pid in &g.points {
            referenced.insert(id_key(pid));
        }
    }
    for c in &doc.constraints {
        for pid in c.points() {
            referenced.insert(id_key(pid));
        }
    }
    doc.points.retain(|p| referenced.contains(&id_key(&p.id)));
}

// ---------------------------------------------------------------------------
// Cut constraints — pin each cut point onto the crossing geometry.
// ---------------------------------------------------------------------------

/// Whether a constraint already equals `(ctype, points)` under the previous
/// order-aware matching — the trim-add dedup.
fn constraint_matches(c: &SketchConstraint, ctype: &str, points: &[Value]) -> bool {
    if c.ctype() != Some(ctype) {
        return false;
    }
    let cp = c.points();
    let k = |v: &Value| id_key(v);
    match ctype {
        "≡" => {
            if points.len() < 2 || cp.len() < 2 {
                return false;
            }
            let (a, b) = (k(&points[0]), k(&points[1]));
            let (p0, p1) = (k(&cp[0]), k(&cp[1]));
            (p0 == a && p1 == b) || (p0 == b && p1 == a)
        }
        "⏛" => {
            if points.len() < 3 || cp.len() < 3 {
                return false;
            }
            let (a, b, p) = (k(&points[0]), k(&points[1]), k(&points[2]));
            let (p0, p1, p2) = (k(&cp[0]), k(&cp[1]), k(&cp[2]));
            ((p0 == a && p1 == b) || (p0 == b && p1 == a)) && p2 == p
        }
        "⇌" => {
            if points.len() < 4 || cp.len() < 4 {
                return false;
            }
            let (a, b, c0, d) = (k(&points[0]), k(&points[1]), k(&points[2]), k(&points[3]));
            let (p0, p1, p2, p3) = (k(&cp[0]), k(&cp[1]), k(&cp[2]), k(&cp[3]));
            let same = |x0: &str, x1: &str, y0: &str, y1: &str| {
                (x0 == y0 && x1 == y1) || (x0 == y1 && x1 == y0)
            };
            (same(&p0, &p1, &a, &b) && same(&p2, &p3, &c0, &d))
                || (same(&p0, &p1, &c0, &d) && same(&p2, &p3, &a, &b))
        }
        _ => false,
    }
}

/// Append a constraint of `(ctype, points)` unless an equal one exists (a port of
/// `#addConstraintIfMissing`), carrying the base fields so it round-trips + the
/// solver seeds any value. Returns whether one was added. Shared with
/// [`super::infer`] (drop-time constraint inference).
pub(crate) fn add_constraint_if_missing(doc: &mut SketchDoc, ctype: &str, points: Vec<Value>) -> bool {
    if doc.constraints.iter().any(|c| constraint_matches(c, ctype, &points)) {
        return false;
    }
    let id = doc.next_constraint_id();
    let mut raw = Map::new();
    raw.insert("id".to_string(), id);
    raw.insert("type".to_string(), Value::String(ctype.to_string()));
    raw.insert("points".to_string(), Value::Array(points));
    raw.insert("labelX".to_string(), Value::from(0));
    raw.insert("labelY".to_string(), Value::from(0));
    raw.insert("displayStyle".to_string(), Value::String(String::new()));
    raw.insert("value".to_string(), Value::Null);
    raw.insert("valueNeedsSetup".to_string(), Value::Bool(true));
    doc.constraints.push(SketchConstraint { raw });
    true
}

/// Pin `point_id` onto a crossing LINE (`#ensurePointOnLineConstraint`): a
/// coincident `≡` when the cut lands on the line's endpoint, else a point-on-line
/// `⏛`.
fn ensure_point_on_line(
    doc: &mut SketchDoc,
    line_geo: &SketchGeometry,
    point_id: &Value,
    inter: &Intersection,
) -> bool {
    let ids = &line_geo.points;
    if ids.len() < 2 {
        return false;
    }
    let a_id = ids[0].clone();
    let b_id = ids[1].clone();
    let (a, b, p) = match (doc.point(&a_id), doc.point(&b_id), doc.point(point_id)) {
        (Some(a), Some(b), Some(p)) => ((a.x, a.y), (b.x, b.y), (p.x, p.y)),
        _ => return false,
    };
    let len = {
        let l = (b.0 - a.0).hypot(b.1 - a.1);
        if l == 0.0 {
            1.0
        } else {
            l
        }
    };
    let eps_param = 1e-3;
    let eps_dist = (len * 1e-3).clamp(1e-5, 1e-2);
    let near_a = inter.other_param.map_or(false, |op| op <= eps_param)
        || (p.0 - a.0).hypot(p.1 - a.1) <= eps_dist;
    let near_b = inter.other_param.map_or(false, |op| op >= 1.0 - eps_param)
        || (p.0 - b.0).hypot(p.1 - b.1) <= eps_dist;

    if near_a && id_key(point_id) != id_key(&a_id) {
        return add_constraint_if_missing(doc, "≡", vec![a_id, point_id.clone()]);
    }
    if near_b && id_key(point_id) != id_key(&b_id) {
        return add_constraint_if_missing(doc, "≡", vec![b_id, point_id.clone()]);
    }
    if id_key(point_id) == id_key(&a_id) || id_key(point_id) == id_key(&b_id) {
        return false;
    }
    add_constraint_if_missing(doc, "⏛", vec![a_id, b_id, point_id.clone()])
}

/// Pin `point_id` onto a crossing ARC/CIRCLE (`#ensurePointOnArcConstraint`): a
/// point-on-arc `⇌ [center, rim, center, point]` (equal-distance to the rim).
fn ensure_point_on_arc(doc: &mut SketchDoc, arc_geo: &SketchGeometry, point_id: &Value) -> bool {
    let ids = &arc_geo.points;
    if ids.len() < 2 {
        return false;
    }
    let center = ids[0].clone();
    let rim = ids[1].clone();
    // Never pin one of the arc's OWN defining points onto it. Centre and rim are
    // degenerate; the far endpoint (`ids[2]`) would restate the arc's implied
    // equal-radius row verbatim, and the solver counts that implied row, so the
    // duplicate would read as a redundant constraint the user never authored.
    if ids.iter().any(|id| id_key(id) == id_key(point_id)) {
        return false;
    }
    add_constraint_if_missing(doc, "⇌", vec![center.clone(), rim, center, point_id.clone()])
}

/// Pin a cut `point_id` onto the CROSSING geometry named by `inter` (a port of
/// `#applyTrimIntersectionConstraint`): point-on-line for a line crosser,
/// point-on-arc for an arc/circle crosser.
fn apply_trim_intersection_constraint(doc: &mut SketchDoc, point_id: &Value, inter: &Intersection) {
    let other = match doc.geometry(&inter.other_geo_id) {
        Some(g) => g.clone(),
        None => return,
    };
    match other.geom_type.as_str() {
        "line" => {
            ensure_point_on_line(doc, &other, point_id, inter);
        }
        "arc" | "circle" => {
            ensure_point_on_arc(doc, &other, point_id);
        }
        _ => {}
    }
}

// ---------------------------------------------------------------------------
// Per-type split (`#trimLineGeometry` / `#trimCircleGeometry` / `#trimArcGeometry`
// / `#trimBezierGeometry`).
// ---------------------------------------------------------------------------

/// Add a line `a→b` unless degenerate (same id / coincident). Returns whether added.
fn add_line_if_valid(doc: &mut SketchDoc, a_id: &Value, b_id: &Value, construction: bool) -> bool {
    if id_key(a_id) == id_key(b_id) {
        return false;
    }
    let ok = match (doc.point(a_id), doc.point(b_id)) {
        (Some(a), Some(b)) => (a.x - b.x).hypot(a.y - b.y) >= 1e-7,
        _ => false,
    };
    if !ok {
        return false;
    }
    add_geometry(doc, "line", vec![a_id.clone(), b_id.clone()], construction);
    true
}

/// Add an arc `[center, start, end]` unless degenerate. Returns whether added.
fn add_arc_if_valid(
    doc: &mut SketchDoc,
    center: &Value,
    start: &Value,
    end: &Value,
    construction: bool,
) -> bool {
    if id_key(start) == id_key(end) {
        return false;
    }
    let ok = match (doc.point(start), doc.point(end)) {
        (Some(a), Some(b)) => (a.x - b.x).hypot(a.y - b.y) >= 1e-7,
        _ => false,
    };
    if !ok {
        return false;
    }
    add_geometry(doc, "arc", vec![center.clone(), start.clone(), end.clone()], construction);
    true
}

/// Trim a LINE (`#trimLineGeometry`): keep the sub-segment(s) OUTSIDE the bracketed
/// span — `[start, prev]` and/or `[next, end]` — minting the cut points and pinning
/// each onto its crossing geometry. `false` (→ delete the whole line) when nothing
/// survives.
fn trim_line(doc: &mut SketchDoc, geo: &SketchGeometry, bounds: &Bounds) -> bool {
    let ids = &geo.points;
    if ids.len() < 2 {
        return false;
    }
    let id0 = ids[0].clone();
    let id1 = ids[1].clone();
    let (p0, p1) = match (doc.point(&id0), doc.point(&id1)) {
        (Some(a), Some(b)) => ((a.x, a.y), (b.x, b.y)),
        _ => return false,
    };
    let eps = 1e-5;
    let use_prev = bounds.prev.as_ref().map_or(false, |b| b.param > eps);
    let use_next = bounds.next.as_ref().map_or(false, |b| b.param < 1.0 - eps);
    if !use_prev && !use_next {
        return false;
    }
    if use_prev && use_next {
        let pp = bounds.prev.as_ref().unwrap().param;
        let np = bounds.next.as_ref().unwrap().param;
        if np - pp <= eps {
            return false;
        }
    }
    let point_at = |t: f64| (p0.0 + (p1.0 - p0.0) * t, p0.1 + (p1.1 - p0.1) * t);
    let construction = geo.construction();

    let prev_id = if use_prev {
        let (x, y) = point_at(bounds.prev.as_ref().unwrap().param);
        get_or_create_point(doc, x, y)
    } else {
        id0.clone()
    };
    let next_id = if use_next {
        let (x, y) = point_at(bounds.next.as_ref().unwrap().param);
        get_or_create_point(doc, x, y)
    } else {
        id1.clone()
    };

    let mut added = false;
    if use_prev {
        added |= add_line_if_valid(doc, &id0, &prev_id, construction);
    }
    if use_next {
        added |= add_line_if_valid(doc, &next_id, &id1, construction);
    }
    if use_prev {
        apply_trim_intersection_constraint(doc, &prev_id, bounds.prev.as_ref().unwrap());
    }
    if use_next {
        apply_trim_intersection_constraint(doc, &next_id, bounds.next.as_ref().unwrap());
    }
    if added {
        remove_geometry(doc, &geo.id);
    }
    added
}

/// Trim an ARC (`#trimArcGeometry`): keep the shorter arc(s) `[center, start, prev]`
/// and/or `[center, next, end]` outside the bracketed span.
fn trim_arc(doc: &mut SketchDoc, geo: &SketchGeometry, bounds: &Bounds) -> bool {
    let ids = &geo.points;
    if ids.len() < 3 {
        return false;
    }
    let id_c = ids[0].clone();
    let id_a = ids[1].clone();
    let id_b = ids[2].clone();
    let (pc, pa) = match (doc.point(&id_c), doc.point(&id_a)) {
        (Some(c), Some(a)) => ((c.x, c.y), (a.x, a.y)),
        _ => return false,
    };
    let r = (pa.0 - pc.0).hypot(pa.1 - pc.1);
    if !r.is_finite() || r < 1e-9 {
        return false;
    }
    let eps = 1e-5;
    let use_prev = bounds.prev.as_ref().map_or(false, |b| b.param > eps);
    let use_next = bounds.next.as_ref().map_or(false, |b| b.param < 1.0 - eps);
    if !use_prev && !use_next {
        return false;
    }
    if use_prev && use_next {
        let pp = bounds.prev.as_ref().unwrap().param;
        let np = bounds.next.as_ref().unwrap().param;
        if np - pp <= eps {
            return false;
        }
    }
    let on_circle = |inter: &Intersection| {
        let ang = (inter.y - pc.1).atan2(inter.x - pc.0);
        (pc.0 + r * ang.cos(), pc.1 + r * ang.sin())
    };
    let prev_id = if use_prev {
        let (x, y) = on_circle(bounds.prev.as_ref().unwrap());
        get_or_create_point(doc, x, y)
    } else {
        id_a.clone()
    };
    let next_id = if use_next {
        let (x, y) = on_circle(bounds.next.as_ref().unwrap());
        get_or_create_point(doc, x, y)
    } else {
        id_b.clone()
    };
    let construction = geo.construction();

    let mut added = false;
    if use_prev {
        added |= add_arc_if_valid(doc, &id_c, &id_a, &prev_id, construction);
    }
    if use_next {
        added |= add_arc_if_valid(doc, &id_c, &next_id, &id_b, construction);
    }
    if use_prev {
        apply_trim_intersection_constraint(doc, &prev_id, bounds.prev.as_ref().unwrap());
    }
    if use_next {
        apply_trim_intersection_constraint(doc, &next_id, bounds.next.as_ref().unwrap());
    }
    if added {
        remove_geometry(doc, &geo.id);
    }
    added
}

/// Trim a CIRCLE (`#trimCircleGeometry`): replace it with the arc `[center, next,
/// prev]` — the surviving span that does NOT contain the click (CCW from the `next`
/// intersection around to `prev`).
fn trim_circle(doc: &mut SketchDoc, geo: &SketchGeometry, bounds: &Bounds) -> bool {
    let ids = &geo.points;
    if ids.len() < 2 {
        return false;
    }
    let center = ids[0].clone();
    let (pc, pr) = match (doc.point(&center), doc.point(&ids[1])) {
        (Some(c), Some(r)) => ((c.x, c.y), (r.x, r.y)),
        _ => return false,
    };
    let r = (pr.0 - pc.0).hypot(pr.1 - pc.1);
    if !r.is_finite() || r < 1e-9 {
        return false;
    }
    let (prev, next) = match (&bounds.prev, &bounds.next) {
        (Some(prev), Some(next)) => (prev, next),
        _ => return false,
    };
    let max_param = if bounds.max_param > 0.0 { bounds.max_param } else { 1.0 };
    let delta = (next.param - prev.param).rem_euclid(max_param);
    if delta < 1e-5 || delta > max_param - 1e-5 {
        return false;
    }
    let on_circle = |inter: &Intersection| {
        let ang = (inter.y - pc.1).atan2(inter.x - pc.0);
        (pc.0 + r * ang.cos(), pc.1 + r * ang.sin())
    };
    let (px, py) = on_circle(prev);
    let prev_id = get_or_create_point(doc, px, py);
    let (nx, ny) = on_circle(next);
    let next_id = get_or_create_point(doc, nx, ny);
    if id_key(&prev_id) == id_key(&next_id) {
        return false;
    }
    let construction = geo.construction();
    add_geometry(doc, "arc", vec![center, next_id.clone(), prev_id.clone()], construction);
    apply_trim_intersection_constraint(doc, &prev_id, prev);
    apply_trim_intersection_constraint(doc, &next_id, next);
    remove_geometry(doc, &geo.id);
    true
}

/// De Casteljau split of the bezier span `seg_index` at `t` (a port of
/// `#splitBezierAt`): shrink the span's control legs and splice three fresh anchors
/// (`r0, s, r1`) into the geometry's point list. Returns the new mid-anchor's index.
fn split_bezier_at(doc: &mut SketchDoc, geo_id: &Value, seg_index: usize, t: f64) -> Option<usize> {
    let ids: Vec<Value> = doc.geometry(geo_id)?.points.clone();
    let seg_count = ids.len().saturating_sub(1) / 3;
    if seg_index >= seg_count {
        return None;
    }
    let base = seg_index * 3;
    let id0 = ids.get(base)?.clone();
    let id1 = ids.get(base + 1)?.clone();
    let id2 = ids.get(base + 2)?.clone();
    let id3 = ids.get(base + 3)?.clone();
    let (p0, p1, p2, p3) = {
        let g0 = doc.point(&id0)?;
        let g1 = doc.point(&id1)?;
        let g2 = doc.point(&id2)?;
        let g3 = doc.point(&id3)?;
        ((g0.x, g0.y), (g1.x, g1.y), (g2.x, g2.y), (g3.x, g3.y))
    };
    let tt = t.clamp(0.0001, 0.9999);
    let lerp = |a: (f64, f64), b: (f64, f64)| (a.0 + (b.0 - a.0) * tt, a.1 + (b.1 - a.1) * tt);
    let q0 = lerp(p0, p1);
    let q1 = lerp(p1, p2);
    let q2 = lerp(p2, p3);
    let r0 = lerp(q0, q1);
    let r1 = lerp(q1, q2);
    let s = lerp(r0, r1);

    if let Some(pm) = doc.point_mut(&id1) {
        pm.x = q0.0;
        pm.y = q0.1;
    }
    if let Some(pm) = doc.point_mut(&id2) {
        pm.x = q2.0;
        pm.y = q2.1;
    }
    let r0_id = create_point(doc, r0.0, r0.1);
    let s_id = create_point(doc, s.0, s.1);
    let r1_id = create_point(doc, r1.0, r1.1);
    let at = base + 2;
    let g = doc.geometry_mut(geo_id)?;
    g.points.splice(at..at, [r0_id, s_id, r1_id]);
    Some(base + 3)
}

/// Trim a BEZIER (`#trimBezierGeometry`): split the span(s) at the bracketing
/// intersections and re-emit the surviving control-point ranges as fresh bezier
/// geometries. `false` (→ delete the whole bezier) when nothing survives.
fn trim_bezier(doc: &mut SketchDoc, geo: &SketchGeometry, bounds: &Bounds, target: &Sampled) -> bool {
    let geo_id = geo.id.clone();
    let seg_count = if target.seg_count >= 1 {
        target.seg_count
    } else {
        geo.points.len().saturating_sub(1) / 3
    };
    if seg_count < 1 {
        return false;
    }
    let prev_int = bounds.prev.clone();
    let next_int = bounds.next.clone();
    if prev_int.is_none() && next_int.is_none() {
        return false;
    }
    if let (Some(p), Some(n)) = (&prev_int, &next_int) {
        if n.param - p.param <= 1e-5 {
            return false;
        }
    }

    /// A cut boundary: which end (`prev`=0/`next`=1), which span + local `t`, its
    /// absolute param (for ordering), and the mid-anchor index once split.
    struct Boundary {
        kind: u8,
        seg_index: usize,
        t: f64,
        pos: f64,
        anchor_index: Option<usize>,
    }
    let mk = |kind: u8, param: f64| -> Boundary {
        let seg_index = (param.floor() as isize).clamp(0, seg_count as isize - 1) as usize;
        Boundary {
            kind,
            seg_index,
            t: param - seg_index as f64,
            pos: param,
            anchor_index: None,
        }
    };
    let mut boundaries: Vec<Boundary> = Vec::new();
    if let Some(p) = &prev_int {
        boundaries.push(mk(0, p.param));
    }
    if let Some(n) = &next_int {
        boundaries.push(mk(1, n.param));
    }
    boundaries.sort_by(|a, b| a.pos.partial_cmp(&b.pos).unwrap_or(Ordering::Equal));

    let mut splits_before = 0usize;
    if boundaries.len() == 2 && boundaries[0].seg_index == boundaries[1].seg_index {
        let first_t = boundaries[0].t;
        let second_t = boundaries[1].t;
        if second_t - first_t < 1e-5 {
            return false;
        }
        let seg = boundaries[0].seg_index;
        let res1 = match split_bezier_at(doc, &geo_id, seg + splits_before, first_t) {
            Some(a) => a,
            None => return false,
        };
        boundaries[0].anchor_index = Some(res1);
        splits_before += 1;
        let t2 = (second_t - first_t) / (1.0 - first_t);
        let res2 = match split_bezier_at(doc, &geo_id, seg + splits_before, t2) {
            Some(a) => a,
            None => return false,
        };
        boundaries[1].anchor_index = Some(res2);
    } else {
        for b in boundaries.iter_mut() {
            let res = match split_bezier_at(doc, &geo_id, b.seg_index + splits_before, b.t) {
                Some(a) => a,
                None => return false,
            };
            b.anchor_index = Some(res);
            splits_before += 1;
        }
    }

    let total_segs = match doc.geometry(&geo_id) {
        Some(g) => g.points.len().saturating_sub(1) / 3,
        None => return false,
    };
    let prev_seg = boundaries
        .iter()
        .find(|b| b.kind == 0)
        .map(|b| b.anchor_index.unwrap_or(0) / 3)
        .unwrap_or(0);
    let next_seg = boundaries
        .iter()
        .find(|b| b.kind == 1)
        .map(|b| b.anchor_index.unwrap_or(0) / 3)
        .unwrap_or(total_segs);
    if next_seg <= prev_seg {
        return false;
    }
    let mut keep_ranges: Vec<(usize, usize)> = Vec::new();
    if prev_seg > 0 {
        keep_ranges.push((0, prev_seg));
    }
    if next_seg < total_segs {
        keep_ranges.push((next_seg, total_segs));
    }
    let construction = geo.construction();
    let mut added = false;
    for (a, b) in keep_ranges {
        if b <= a {
            continue;
        }
        let start_idx = a * 3;
        let end_idx = b * 3;
        let plen = match doc.geometry(&geo_id) {
            Some(g) => g.points.len(),
            None => return false,
        };
        if end_idx >= plen {
            continue;
        }
        let pts: Vec<Value> = doc.geometry(&geo_id).unwrap().points[start_idx..=end_idx].to_vec();
        if pts.len() >= 4 {
            add_geometry(doc, "bezier", pts, construction);
            added = true;
        }
    }
    if added {
        remove_geometry(doc, &geo_id);
    }
    added
}

// ---------------------------------------------------------------------------
// Public entry points.
// ---------------------------------------------------------------------------

/// The id of the geometry whose sampled polyline passes nearest to `(u, v)` within
/// `radius`, or `None`. The geometry-only picker the trim tool uses (points are NOT
/// candidates — a trim click acts on a curve).
pub fn pick_geometry_id(doc: &SketchDoc, u: f64, v: f64, radius: f64) -> Option<Value> {
    let mut best: Option<(f64, Value)> = None;
    for g in &doc.geometries {
        let poly = super::tessellate::geometry_polyline_uv(g, doc);
        if poly.len() < 2 {
            continue;
        }
        let mut dmin = f64::INFINITY;
        for seg in poly.windows(2) {
            let d = point_seg_dist(u, v, seg[0], seg[1]);
            if d < dmin {
                dmin = d;
            }
        }
        if dmin <= radius && best.as_ref().map_or(true, |(bd, _)| dmin < *bd) {
            best = Some((dmin, g.id.clone()));
        }
    }
    best.map(|(_, id)| id)
}

/// Trim (or delete) the geometry `geo_id` at the plane click `(u, v)` — the port of
/// `#trimGeometry`. Samples the target, collects its intersections with every other
/// geometry, brackets the click, and splits per type; with no bracketing
/// intersections the whole geometry is deleted. Returns whether the doc changed
/// (the caller then re-solves). Does NOT solve or touch selection/undo — the engine
/// wrapper owns that.
pub fn trim_geometry(doc: &mut SketchDoc, geo_id: &Value, u: f64, v: f64) -> bool {
    let geo = match doc.geometry(geo_id) {
        Some(g) => g.clone(),
        None => return false,
    };
    let target = match sample_geometry(&geo, doc) {
        Some(t) if t.samples.len() >= 2 => t,
        _ => return false,
    };
    let click_param = match closest_param_on_samples(u, v, &target.samples) {
        Some(p) if p.is_finite() => p,
        _ => return false,
    };

    let mut intersections: Vec<Intersection> = Vec::new();
    for other in &doc.geometries {
        if id_key(&other.id) == id_key(geo_id) {
            continue;
        }
        if let Some(sample) = sample_geometry(other, doc) {
            collect_intersections(&target, &sample, other, &mut intersections);
        }
    }

    let changed = match select_trim_bounds(&intersections, click_param, &target) {
        None => remove_geometry(doc, geo_id),
        Some(bounds) => {
            let trimmed = match geo.geom_type.as_str() {
                "line" => trim_line(doc, &geo, &bounds),
                "circle" => trim_circle(doc, &geo, &bounds),
                "arc" => {
                    if target.closed {
                        trim_circle(doc, &geo, &bounds)
                    } else {
                        trim_arc(doc, &geo, &bounds)
                    }
                }
                "bezier" => trim_bezier(doc, &geo, &bounds, &target),
                _ => false,
            };
            if trimmed {
                true
            } else {
                remove_geometry(doc, geo_id)
            }
        }
    };
    if changed {
        cleanup_orphan_points(doc);
    }
    changed
}

/// Euclidean distance from `(px, py)` to segment `a`–`b` (plane uv).
fn point_seg_dist(px: f64, py: f64, a: [f64; 2], b: [f64; 2]) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len2 = dx * dx + dy * dy;
    let t = if len2 <= 1e-18 {
        0.0
    } else {
        (((px - a[0]) * dx + (py - a[1]) * dy) / len2).clamp(0.0, 1.0)
    };
    let (cx, cy) = (a[0] + t * dx, a[1] + t * dy);
    (px - cx).hypot(py - cy)
}

