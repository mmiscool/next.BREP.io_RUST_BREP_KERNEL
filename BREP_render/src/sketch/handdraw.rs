//! Recognize freehand strokes as lines, circles, arcs, or cubic Beziers.
//!
//! Classification tolerances scale with the stroke's bounding-box extent.
//! Open strokes near their endpoint chord become lines. A good least-squares
//! circle fit produces a circle for closed strokes or an arc for open strokes.
//! The fallback cubic uses endpoints and samples at one-third and two-thirds
//! of the stroke's arc length.

use crate::geometry2d::{distance as dist, point_segment_distance};

use serde_json::{Map, Value};

use super::doc::{SketchDoc, SketchGeometry, SketchPoint};

/// Endpoint gap (as a fraction of the stroke extent) under which the stroke counts
/// as CLOSED — a freehand loop rarely returns to its exact start.
const CLOSED_GAP_FRAC: f64 = 0.15;
/// Max interior perpendicular deviation from the endpoint chord (fraction of extent)
/// for the stroke to read as a straight LINE.
const LINE_DEV_FRAC: f64 = 0.02;
/// Max radial residual of the circle fit (fraction of extent) for the stroke to read
/// as a CIRCLE / ARC — loose enough to absorb hand wobble.
const CIRCLE_RESIDUAL_FRAC: f64 = 0.08;
/// A fitted radius below this fraction of the extent is a degenerate dot, not a
/// circle.
const MIN_RADIUS_FRAC: f64 = 0.02;

/// A recognized handdraw shape, in plane `(u, v)` coordinates. Point semantics match
/// the solver's geometry (`line = [a, b]`, `circle = [center, rim]`,
/// `arc = [center, start, end]`, `bezier = [p0, c1, c2, p1]`).
#[derive(Clone, Debug, PartialEq)]
pub enum HandDrawShape {
    /// A straight segment through the stroke's two endpoints.
    Line { a: (f64, f64), b: (f64, f64) },
    /// A closed circle — the fitted center + a rim point on `+u`.
    Circle { center: (f64, f64), rim: (f64, f64) },
    /// A circular arc — the fitted center + the stroke's start/end (CCW start→end).
    Arc {
        center: (f64, f64),
        start: (f64, f64),
        end: (f64, f64),
    },
    /// A cubic Bezier fallback: `[p0, c1, c2, p1]` (endpoints + two on-curve controls).
    Bezier { controls: [(f64, f64); 4] },
}

impl HandDrawShape {
    /// The solver geometry `type` this shape materializes as.
    pub fn kind(&self) -> &'static str {
        match self {
            HandDrawShape::Line { .. } => "line",
            HandDrawShape::Circle { .. } => "circle",
            HandDrawShape::Arc { .. } => "arc",
            HandDrawShape::Bezier { .. } => "bezier",
        }
    }
}

/// Classify a raw uv `stroke` into one recognized [`HandDrawShape`]. See the module
/// docs for the tolerance rules. A stroke of fewer than 2 samples degenerates to a
/// zero-length line (callers guard against tiny strokes before recognizing).
pub fn recognize(stroke: &[(f64, f64)]) -> HandDrawShape {
    let n = stroke.len();
    if n < 2 {
        let p = stroke.first().copied().unwrap_or((0.0, 0.0));
        return HandDrawShape::Line { a: p, b: p };
    }
    let a = stroke[0];
    let b = stroke[n - 1];
    let extent = stroke_extent(stroke).max(1e-9);
    let closed = dist(a, b) <= CLOSED_GAP_FRAC * extent;

    // LINE — an open stroke whose interior hugs the endpoint chord (a 2-sample stroke
    // is trivially straight).
    if !closed {
        let max_dev = stroke[1..n - 1]
            .iter()
            .map(|&p| point_segment_distance(p, a, b).0)
            .fold(0.0_f64, f64::max);
        if n == 2 || max_dev <= LINE_DEV_FRAC * extent {
            return HandDrawShape::Line { a, b };
        }
    }

    // CIRCLE / ARC — a good least-squares circle fit (non-degenerate radius, small
    // radial residual). Closed → a full circle; open → an arc through the endpoints.
    if n >= 3 {
        if let Some((cx, cy, r)) = fit_circle_lsq(stroke) {
            let residual = stroke
                .iter()
                .map(|&p| (dist(p, (cx, cy)) - r).abs())
                .fold(0.0_f64, f64::max);
            if r.is_finite()
                && r > MIN_RADIUS_FRAC * extent
                && residual <= CIRCLE_RESIDUAL_FRAC * extent
            {
                if closed {
                    return HandDrawShape::Circle {
                        center: (cx, cy),
                        rim: (cx + r, cy),
                    };
                }
                return HandDrawShape::Arc {
                    center: (cx, cy),
                    start: a,
                    end: b,
                };
            }
        }
    }

    // BEZIER fallback — a single cubic through the stroke.
    HandDrawShape::Bezier {
        controls: fit_cubic(stroke),
    }
}

/// Materialize a recognized [`HandDrawShape`] into `doc`: mint its points (endpoints
/// snap to an existing PRE-STROKE point within `snap_radius` so a stroke drawn onto
/// prior geometry coincides) and append the geometry (non-construction; the bezier
/// fallback also adds its two dashed control-handle guide lines, matching the bezier
/// tool). Never solves; the caller re-solves.
pub fn emit_shape(doc: &mut SketchDoc, shape: &HandDrawShape, snap_radius: f64) {
    // Only points that existed BEFORE this emit are snap targets, so a shape's own
    // freshly-minted points never collapse into one another (a short stroke's
    // endpoints / a bezier's controls stay distinct).
    let base = doc.points.len();
    match shape {
        HandDrawShape::Line { a, b } => {
            let a_id = snap_new_point(doc, base, a.0, a.1, snap_radius);
            let b_id = snap_new_point(doc, base, b.0, b.1, snap_radius);
            push_geometry(doc, "line", vec![a_id, b_id], false);
        }
        HandDrawShape::Circle { center, rim } => {
            let c = snap_new_point(doc, base, center.0, center.1, snap_radius);
            let r = snap_new_point(doc, base, rim.0, rim.1, snap_radius);
            push_geometry(doc, "circle", vec![c, r], false);
        }
        HandDrawShape::Arc { center, start, end } => {
            let c = snap_new_point(doc, base, center.0, center.1, snap_radius);
            let s = snap_new_point(doc, base, start.0, start.1, snap_radius);
            let e = snap_new_point(doc, base, end.0, end.1, snap_radius);
            push_geometry(doc, "arc", vec![c, s, e], false);
        }
        HandDrawShape::Bezier { controls } => {
            let ids: Vec<Value> = controls
                .iter()
                .map(|&(u, v)| snap_new_point(doc, base, u, v, snap_radius))
                .collect();
            push_geometry(doc, "bezier", ids.clone(), false);
            // Dashed control-handle guides (end0→ctrl0, end1→ctrl1) — matches the
            // click-driven bezier tool.
            push_geometry(doc, "line", vec![ids[0].clone(), ids[1].clone()], true);
            push_geometry(doc, "line", vec![ids[3].clone(), ids[2].clone()], true);
        }
    }
}

/// The bounding-box diagonal of a uv stroke — the relative-tolerance scale (and the
/// engine's "too tiny to recognize" gauge).
pub fn stroke_extent(stroke: &[(f64, f64)]) -> f64 {
    let (mut minx, mut miny) = (f64::INFINITY, f64::INFINITY);
    let (mut maxx, mut maxy) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
    for &(x, y) in stroke {
        minx = minx.min(x);
        miny = miny.min(y);
        maxx = maxx.max(x);
        maxy = maxy.max(y);
    }
    if !minx.is_finite() {
        return 0.0;
    }
    ((maxx - minx).powi(2) + (maxy - miny).powi(2)).sqrt()
}

/// Fit a cubic through the stroke: endpoints = first/last, the two controls sampled
/// at ~1/3 and ~2/3 of the ARC LENGTH (on-curve approximation, mirroring the previous
/// cubic-from-stroke fit). A zero-length stroke collapses to its endpoints.
fn fit_cubic(stroke: &[(f64, f64)]) -> [(f64, f64); 4] {
    let n = stroke.len();
    let first = stroke[0];
    let last = stroke[n - 1];
    let mut cum = vec![0.0_f64; n];
    for i in 1..n {
        cum[i] = cum[i - 1] + dist(stroke[i - 1], stroke[i]);
    }
    let total = cum[n - 1];
    if total < 1e-9 {
        return [first, first, last, last];
    }
    let c1 = sample_arc(stroke, &cum, total, 1.0 / 3.0);
    let c2 = sample_arc(stroke, &cum, total, 2.0 / 3.0);
    [first, c1, c2, last]
}

/// Sample the stroke at fractional arc length `t ∈ [0, 1]` (linear between the two
/// bracketing samples).
fn sample_arc(stroke: &[(f64, f64)], cum: &[f64], total: f64, t: f64) -> (f64, f64) {
    let target = total * t;
    let mut idx = 0;
    while idx < cum.len() && cum[idx] < target {
        idx += 1;
    }
    if idx == 0 {
        return stroke[0];
    }
    if idx >= cum.len() {
        return stroke[stroke.len() - 1];
    }
    let (d0, d1) = (cum[idx - 1], cum[idx]);
    let span = (d1 - d0).max(1e-9);
    let tt = ((target - d0) / span).clamp(0.0, 1.0);
    let p0 = stroke[idx - 1];
    let p1 = stroke[idx];
    (p0.0 + (p1.0 - p0.0) * tt, p0.1 + (p1.1 - p0.1) * tt)
}

/// A modified (centered Kåsa) least-squares circle fit over all samples: robust to
/// hand wobble and dense sampling. Returns `(cx, cy, r)` or `None` when the samples
/// are (near) collinear.
fn fit_circle_lsq(pts: &[(f64, f64)]) -> Option<(f64, f64, f64)> {
    let n = pts.len();
    if n < 3 {
        return None;
    }
    let nf = n as f64;
    let (mut mx, mut my) = (0.0_f64, 0.0_f64);
    for &(x, y) in pts {
        mx += x;
        my += y;
    }
    mx /= nf;
    my /= nf;
    // Centered moments (subtracting the centroid conditions the normal equations).
    let (mut sxx, mut sxy, mut syy) = (0.0_f64, 0.0_f64, 0.0_f64);
    let (mut sxz, mut syz) = (0.0_f64, 0.0_f64);
    for &(x, y) in pts {
        let u = x - mx;
        let v = y - my;
        let z = u * u + v * v;
        sxx += u * u;
        sxy += u * v;
        syy += v * v;
        sxz += u * z;
        syz += v * z;
    }
    let det = sxx * syy - sxy * sxy;
    if det.abs() < 1e-12 {
        return None; // collinear
    }
    // Solve [sxx sxy; sxy syy][uc; vc] = [sxz/2; syz/2].
    let uc = (sxz * syy - syz * sxy) / (2.0 * det);
    let vc = (sxx * syz - sxy * sxz) / (2.0 * det);
    let cx = uc + mx;
    let cy = vc + my;
    let r = (uc * uc + vc * vc + (sxx + syy) / nf).sqrt();
    if !cx.is_finite() || !cy.is_finite() || !r.is_finite() {
        return None;
    }
    Some((cx, cy, r))
}

/// Snap `(u, v)` to the nearest point among the first `base` doc points within
/// `radius` (reusing its id so a stroke endpoint coincides with prior geometry), else
/// mint a fresh free point. Freshly-appended points (index `>= base`) are never snap
/// targets, so a single shape's points stay distinct.
fn snap_new_point(doc: &mut SketchDoc, base: usize, u: f64, v: f64, radius: f64) -> Value {
    let mut best: Option<(f64, Value)> = None;
    for p in doc.points.iter().take(base) {
        let d = ((p.x - u).powi(2) + (p.y - v).powi(2)).sqrt();
        if d <= radius && best.as_ref().map_or(true, |(bd, _)| d < *bd) {
            best = Some((d, p.id.clone()));
        }
    }
    if let Some((_, id)) = best {
        return id;
    }
    let id = doc.next_point_id();
    doc.points.push(SketchPoint {
        id: id.clone(),
        x: u,
        y: v,
        fixed: false,
        construction: false,
        external_reference: false,
    });
    id
}

/// Append a geometry with a freshly minted id and an explicit `construction` flag.
fn push_geometry(doc: &mut SketchDoc, geom_type: &str, points: Vec<Value>, construction: bool) {
    let id = doc.next_geometry_id();
    let mut extra = Map::new();
    extra.insert("construction".to_string(), Value::Bool(construction));
    doc.geometries.push(SketchGeometry {
        id,
        geom_type: geom_type.to_string(),
        points,
        extra,
    });
}

