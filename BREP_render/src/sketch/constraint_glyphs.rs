//! Screen-sized line-art glyphs for geometric constraints.
//!
//! Dimensional annotations live in [`super::dimensions`]. Solver-internal
//! temporary constraints are omitted. Glyphs use `world_per_pixel` for sizing
//! and have their own overlay group so they can update independently.

use std::collections::HashMap;
use std::f64::consts::PI;

use serde_json::Value;

use super::dimensions::line_intersection;
use super::doc::{id_key, SketchConstraint, SketchDiagnostics, SketchDoc, SketchPoint};
use super::PlaneFrame;
use crate::style::SketchColors;

/// The constraint-glyph overlay group name (its own group so it upserts/clears
/// independently of the other sketch overlay groups).
pub const OVERLAY_CONSTRAINT_GLYPHS: &str = "sketch-constraint-glyphs";

/// The glyph "half size" in CSS px — a glyph spans roughly `2 × GLYPH_PX`. Sized a
/// touch larger than the 8px sketch point so the marks are legible without crowding.
const GLYPH_PX: f64 = 7.0;

/// Half the offset (in glyph half-sizes) that pushes a midpoint mark off its segment
/// so the glyph does not sit on top of the line it annotates.
const OFFSET_FACTOR: f64 = 2.2;

/// Resolve a constraint's point id to its solved plane `(u, v)`.
fn get<'a>(by_id: &HashMap<String, &'a SketchPoint>, id: &Value) -> Option<[f64; 2]> {
    by_id.get(&id_key(id)).map(|p| [p.x, p.y])
}

/// The screen-constant glyph scale (a glyph half-size in plane units) at this zoom.
fn glyph_scale(wpp: f64) -> f64 {
    (wpp * GLYPH_PX).max(0.05)
}

/// Whether a constraint contributes NO glyph here — the dimensional constraints
/// (owned by [`super::dimensions`]) and solver-internal `temporary` helpers.
fn is_dimensional_or_temporary(c: &SketchConstraint) -> bool {
    if c.temporary() {
        return true;
    }
    matches!(c.ctype(), Some("⟺") | Some("↥") | Some("∠"))
}

/// Push a template — a set of local-frame strokes (`x`/`y` in roughly `[-1, 1]`) —
/// placed at `anchor`, scaled by `scale`, rotated by `rot` radians.
fn stamp(
    out: &mut Vec<([f64; 2], [f64; 2])>,
    anchor: [f64; 2],
    scale: f64,
    rot: f64,
    strokes: &[&[[f64; 2]]],
) {
    let (c, s) = (rot.cos(), rot.sin());
    let map = |p: [f64; 2]| {
        let (x, y) = (p[0] * scale, p[1] * scale);
        [anchor[0] + x * c - y * s, anchor[1] + x * s + y * c]
    };
    for stroke in strokes {
        for w in stroke.windows(2) {
            out.push((map(w[0]), map(w[1])));
        }
    }
}

// --- local-frame stroke templates ------------------------------------------------

const T_DASH_H: [[f64; 2]; 2] = [[-1.0, 0.0], [1.0, 0.0]]; // — horizontal dash
const T_DASH_V: [[f64; 2]; 2] = [[0.0, -1.0], [0.0, 1.0]]; // │ vertical dash
const T_CHEV: [[f64; 2]; 3] = [[-0.5, -0.8], [0.5, 0.0], [-0.5, 0.8]]; // › along +x
const T_EQ_A: [[f64; 2]; 2] = [[-0.8, -0.32], [0.8, -0.32]]; // = upper bar
const T_EQ_B: [[f64; 2]; 2] = [[-0.8, 0.32], [0.8, 0.32]]; // = lower bar
const T_SQUARE: [[f64; 2]; 5] = [
    [-0.8, -0.8],
    [0.8, -0.8],
    [0.8, 0.8],
    [-0.8, 0.8],
    [-0.8, -0.8],
]; // ▢ coincident
const T_DIAMOND: [[f64; 2]; 5] = [
    [0.0, -1.0],
    [1.0, 0.0],
    [0.0, 1.0],
    [-1.0, 0.0],
    [0.0, -1.0],
]; // ◇ midpoint
const T_DOT_A: [[f64; 2]; 2] = [[-0.85, 0.0], [-0.6, 0.0]]; // ⋰ three ticks along +x
const T_DOT_B: [[f64; 2]; 2] = [[-0.12, 0.0], [0.12, 0.0]];
const T_DOT_C: [[f64; 2]; 2] = [[0.6, 0.0], [0.85, 0.0]];
const T_BOW_L: [[f64; 2]; 4] = [[-1.0, -0.8], [0.0, 0.0], [-1.0, 0.8], [-1.0, -0.8]]; // ⋈ left wing
const T_BOW_R: [[f64; 2]; 4] = [[1.0, -0.8], [0.0, 0.0], [1.0, 0.8], [1.0, -0.8]]; // ⋈ right wing
/// ∿ the straight thing a span-point tangency touches — a bar laid across the
/// crown of [`arc_cup`], which is where the unit arch's tangent is horizontal.
const T_TOUCH_BAR: [[f64; 2]; 2] = [[-1.0, 1.0], [1.0, 1.0]];
const T_GND_STEM: [[f64; 2]; 2] = [[0.0, 1.0], [0.0, 0.0]]; // ⏚ stem
const T_GND_BASE: [[f64; 2]; 2] = [[-1.0, 0.0], [1.0, 0.0]]; // ⏚ base bar
const T_GND_H1: [[f64; 2]; 2] = [[-0.6, -0.5], [0.6, -0.5]]; // ⏚ hatch 1
const T_GND_H2: [[f64; 2]; 2] = [[-0.25, -1.0], [0.25, -1.0]]; // ⏚ hatch 2

/// A regular octagon outline (closed) at unit radius — the `◎` concentric ring shape.
fn octagon() -> Vec<[f64; 2]> {
    (0..=8)
        .map(|i| {
            let t = (i as f64 / 8.0) * 2.0 * PI;
            [t.cos(), t.sin()]
        })
        .collect()
}

/// A shallow upward arc (`⌒`) sampled as a polyline in the local frame.
fn arc_cup() -> Vec<[f64; 2]> {
    (0..=8)
        .map(|i| {
            let t = PI * (0.15 + 0.7 * (i as f64 / 8.0)); // ~27°..~153°
            [t.cos(), t.sin()]
        })
        .collect()
}

/// Unit direction from `a` to `b`, or `None` when the two coincide.
fn unit(a: [f64; 2], b: [f64; 2]) -> Option<[f64; 2]> {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l = dx.hypot(dy);
    (l > 1e-9).then(|| [dx / l, dy / l])
}

fn midpoint(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]
}

/// The plane-space glyph segments for one geometric constraint, or `None` when the
/// constraint is dimensional / temporary / unresolvable. Every segment pair is in the
/// sketch's `(u, v)` frame (embedded in world space by the buffer builder).
fn glyph_segments(
    c: &SketchConstraint,
    by_id: &HashMap<String, &SketchPoint>,
    wpp: f64,
) -> Option<Vec<([f64; 2], [f64; 2])>> {
    if is_dimensional_or_temporary(c) {
        return None;
    }
    let ctype = c.ctype()?;
    let pts = c.points();
    let s = glyph_scale(wpp);
    let mut out: Vec<([f64; 2], [f64; 2])> = Vec::new();

    match ctype {
        // — horizontal / vertical: a dash offset off the segment midpoint.
        "━" | "│" => {
            let (a, b) = (get(by_id, pts.first()?)?, get(by_id, pts.get(1)?)?);
            let mid = midpoint(a, b);
            // Offset along the segment normal (fall back to +v for a degenerate span).
            let dir = unit(a, b).unwrap_or([1.0, 0.0]);
            let normal = [-dir[1], dir[0]];
            let anchor = [mid[0] + normal[0] * s * OFFSET_FACTOR, mid[1] + normal[1] * s * OFFSET_FACTOR];
            let tmpl: &[[f64; 2]] = if ctype == "━" { &T_DASH_H } else { &T_DASH_V };
            stamp(&mut out, anchor, s, 0.0, &[tmpl]);
        }
        // ∥ parallel: a chevron on each of the two lines, pointing along the line.
        "∥" if pts.len() >= 4 => {
            for (pa, pb) in [(pts[0].clone(), pts[1].clone()), (pts[2].clone(), pts[3].clone())] {
                let (a, b) = (get(by_id, &pa)?, get(by_id, &pb)?);
                let dir = unit(a, b)?;
                let anchor = midpoint(a, b);
                stamp(&mut out, anchor, s, dir[1].atan2(dir[0]), &[&T_CHEV]);
            }
        }
        // ⟂ perpendicular: a right-angle corner at the shared vertex (or the two lines'
        // intersection), drawn from the two outgoing edge directions.
        "⟂" if pts.len() >= 4 => {
            let (l1a, l1b) = (get(by_id, &pts[0])?, get(by_id, &pts[1])?);
            let (l2a, l2b) = (get(by_id, &pts[2])?, get(by_id, &pts[3])?);
            // The shared corner: whichever endpoint of line 1 coincides with an
            // endpoint of line 2 (adjacent edges); else the infinite-line intersection.
            let key = |p: [f64; 2]| (p[0].to_bits(), p[1].to_bits());
            let shared = [l1a, l1b]
                .into_iter()
                .find(|&e| key(e) == key(l2a) || key(e) == key(l2b));
            let (corner, d1, d2) = if let Some(corner) = shared {
                let o1 = if key(corner) == key(l1a) { l1b } else { l1a };
                let o2 = if key(corner) == key(l2a) { l2b } else { l2a };
                (corner, unit(corner, o1)?, unit(corner, o2)?)
            } else {
                let corner = line_intersection(l1a, l1b, l2a, l2b);
                (corner, unit(l1a, l1b)?, unit(l2a, l2b)?)
            };
            let p1 = [corner[0] + d1[0] * s, corner[1] + d1[1] * s];
            let p3 = [corner[0] + d2[0] * s, corner[1] + d2[1] * s];
            let p2 = [p1[0] + d2[0] * s, p1[1] + d2[1] * s];
            out.push((p1, p2));
            out.push((p2, p3));
        }
        // ≡ coincident: a small square at the (shared) point.
        "≡" => {
            let anchor = get(by_id, pts.first()?)?;
            stamp(&mut out, anchor, s, 0.0, &[&T_SQUARE]);
        }
        // ⇌ equal distance: `=` ticks at each of the two segments' midpoints.
        "⇌" if pts.len() >= 4 => {
            for (pa, pb) in [(pts[0].clone(), pts[1].clone()), (pts[2].clone(), pts[3].clone())] {
                let (a, b) = (get(by_id, &pa)?, get(by_id, &pb)?);
                let dir = unit(a, b).unwrap_or([1.0, 0.0]);
                stamp(&mut out, midpoint(a, b), s, dir[1].atan2(dir[0]), &[&T_EQ_A, &T_EQ_B]);
            }
        }
        // ⊜ equal radius: `=` ticks at each circle's rim point (points[1], points[3]).
        "⊜" if pts.len() >= 4 => {
            for rim in [pts[1].clone(), pts[3].clone()] {
                let anchor = get(by_id, &rim)?;
                stamp(&mut out, anchor, s, 0.0, &[&T_EQ_A, &T_EQ_B]);
            }
        }
        // ◎ concentric: two nested rings at the shared center.
        "◎" => {
            let anchor = get(by_id, pts.first()?)?;
            let ring = octagon();
            stamp(&mut out, anchor, s, 0.0, &[&ring]);
            stamp(&mut out, anchor, s * 0.45, 0.0, &[&ring]);
        }
        // ⌒ tangent: a small arch at the centroid of the involved points.
        "⌒" => {
            let anchor = centroid(by_id, pts)?;
            let cup = arc_cup();
            stamp(&mut out, anchor, s, 0.0, &[&cup]);
        }
        // ϰ curvature (G2): the tangent arch DOUBLED — two concentric arches at
        // the centroid, reading as "the same bend on both sides". Curvature
        // continuity implies tangency, and the mark says so by containing the
        // tangent's.
        "ϰ" => {
            let anchor = centroid(by_id, pts)?;
            let cup = arc_cup();
            stamp(&mut out, anchor, s, 0.0, &[&cup]);
            stamp(&mut out, anchor, s * 0.55, 0.0, &[&cup]);
        }
        // ∿ tangency at a point interior to a span: the tangent arch with a
        // straight bar laid across its crown — a curve and the thing it touches,
        // meeting at ONE point. `⌒`'s bare arch says "tangent"; the bar is what
        // says the touch point is somewhere along the curve rather than at an
        // anchor, which is the whole difference between the two constraints.
        //
        // Anchored at the centroid of the constraint's points, as `⌒` and `ϰ`
        // are. It is deliberately not the touch point: this layer resolves POINTS
        // only, and the foot is a point on a curve — reaching it would mean
        // handing the glyph builder the spline's geometry and re-solving the foot
        // parameter per frame, to move a mark the user reads as "these two are
        // tangent" either way.
        "∿" => {
            let anchor = centroid(by_id, pts)?;
            let cup = arc_cup();
            stamp(&mut out, anchor, s, 0.0, &[&cup, &T_TOUCH_BAR]);
        }
        // ⋰ collinear: three ticks along the point run.
        "⋰" if pts.len() >= 2 => {
            let a = get(by_id, pts.first()?)?;
            let b = get(by_id, pts.last()?)?;
            let dir = unit(a, b).unwrap_or([1.0, 0.0]);
            stamp(&mut out, midpoint(a, b), s, dir[1].atan2(dir[0]), &[&T_DOT_A, &T_DOT_B, &T_DOT_C]);
        }
        // ⏛ point-on-line: a short line with a dot at the constrained point.
        "⏛" if pts.len() >= 3 => {
            let (la, lb) = (get(by_id, &pts[0])?, get(by_id, &pts[1])?);
            let anchor = get(by_id, &pts[2])?;
            let dir = unit(la, lb).unwrap_or([1.0, 0.0]);
            let rot = dir[1].atan2(dir[0]);
            stamp(&mut out, anchor, s, rot, &[&T_DASH_H]);
            stamp(&mut out, anchor, s * 0.35, 0.0, &[&T_SQUARE]);
        }
        // ⋯ midpoint: a diamond at the point that sits mid-way between the other two.
        "⋯" if pts.len() >= 3 => {
            let p: Vec<[f64; 2]> = pts.iter().take(3).map(|id| get(by_id, id)).collect::<Option<_>>()?;
            // The midpoint vertex minimizes its distance to the midpoint of the others.
            let mut best = (f64::INFINITY, 0usize);
            for k in 0..3 {
                let m = midpoint(p[(k + 1) % 3], p[(k + 2) % 3]);
                let d = (p[k][0] - m[0]).hypot(p[k][1] - m[1]);
                if d < best.0 {
                    best = (d, k);
                }
            }
            stamp(&mut out, p[best.1], s * 0.8, 0.0, &[&T_DIAMOND]);
        }
        // ⋈ symmetric: a bowtie at the midpoint of the two symmetric points.
        "⋈" if pts.len() >= 4 => {
            let (p1, p2) = (get(by_id, &pts[2])?, get(by_id, &pts[3])?);
            stamp(&mut out, midpoint(p1, p2), s, 0.0, &[&T_BOW_L, &T_BOW_R]);
        }
        // ⏚ ground: a ground symbol at (just below) the fixed point. Linked/reference
        // geometry is implicitly pinned by BEING a reference — its per-point `⏚` is
        // solver bookkeeping (see `external_ref::add_external_ref`), not a user
        // constraint — so painting a fixed glyph on it is pure noise. Suppress the glyph
        // for external-reference points; a user-grounded point still shows `⏚`. Render /
        // pick only — the solver still sees the ground, so mobility is unchanged.
        "⏚" => {
            let pid = pts.first()?;
            if by_id
                .get(&id_key(pid))
                .map_or(false, |p| p.external_reference)
            {
                return None;
            }
            let p = get(by_id, pid)?;
            let anchor = [p[0], p[1] - s * 0.4];
            stamp(&mut out, anchor, s, 0.0, &[&T_GND_STEM, &T_GND_BASE, &T_GND_H1, &T_GND_H2]);
        }
        _ => return None,
    }

    (!out.is_empty()).then_some(out)
}

/// The centroid (average) of a constraint's resolvable points, or `None` if none
/// resolve.
fn centroid(by_id: &HashMap<String, &SketchPoint>, pts: &[Value]) -> Option<[f64; 2]> {
    let mut sum = [0.0, 0.0];
    let mut n = 0.0;
    for id in pts {
        if let Some(p) = get(by_id, id) {
            sum[0] += p[0];
            sum[1] += p[1];
            n += 1.0;
        }
    }
    (n > 0.0).then(|| [sum[0] / n, sum[1] / n])
}

/// The plane-space glyph line segments for ONE geometric constraint, resolved
/// against `doc` (empty for a dimensional / temporary / unresolvable constraint).
/// The picking + selection-emphasis path measures the cursor against exactly these
/// segments, so "click near the glyph mark" is precisely the drawn glyph.
pub fn constraint_glyph_segments(
    c: &SketchConstraint,
    doc: &SketchDoc,
    world_per_pixel: f64,
) -> Vec<([f64; 2], [f64; 2])> {
    let by_id = super::doc::point_index(doc);
    glyph_segments(c, &by_id, world_per_pixel).unwrap_or_default()
}

/// The world-space glyph line segments for every geometric constraint, as flat
/// `(positions, colors)` buffers (6 position + 6 color floats per segment) ready to
/// feed the `sketch-constraint-glyphs` overlay group.
pub fn constraint_glyphs_buffers(
    doc: &SketchDoc,
    diag: &SketchDiagnostics,
    plane: &PlaneFrame,
    world_per_pixel: f64,
    colors: &SketchColors,
) -> (Vec<f32>, Vec<f32>) {
    constraint_glyphs_buffers_with_state(doc, diag, plane, world_per_pixel, colors, None, &[])
}

/// Like [`constraint_glyphs_buffers`], but emphasizes the hovered / selected
/// constraint (matched by a `{"kind":"constraint","id":…}` ref) in the SAME amber /
/// light-blue as a selected / hovered point or geometry (via
/// [`super::tessellate::interaction_color`]); an empty hover + selection reproduce the
/// plain-green output.
pub fn constraint_glyphs_buffers_with_state(
    doc: &SketchDoc,
    diag: &SketchDiagnostics,
    plane: &PlaneFrame,
    world_per_pixel: f64,
    colors: &SketchColors,
    hovered: Option<&Value>,
    selection: &[Value],
) -> (Vec<f32>, Vec<f32>) {
    let by_id = super::doc::point_index(doc);
    let mut positions: Vec<f32> = Vec::new();
    let mut color_buf: Vec<f32> = Vec::new();
    for c in &doc.constraints {
        let Some(segments) = glyph_segments(c, &by_id, world_per_pixel) else {
            continue;
        };
        // Emphasis is keyed by the constraint's own id: green normally, RED while
        // the solver names it in a conflict, amber when selected, light blue when
        // hovered — selection and hover still win, so a conflicting glyph the user
        // picks reads as picked.
        let color = match c.raw.get("id") {
            Some(id) => super::tessellate::interaction_color(
                colors,
                super::constraint_base_color(colors, diag, id),
                hovered,
                selection,
                "constraint",
                id,
            ),
            None => colors.constraint,
        };
        let rgb = crate::color::hex_to_srgb_f32(color);
        for (a, b) in segments {
            let wa = plane.to_world(a[0], a[1]);
            let wb = plane.to_world(b[0], b[1]);
            positions.extend_from_slice(&[
                wa[0] as f32,
                wa[1] as f32,
                wa[2] as f32,
                wb[0] as f32,
                wb[1] as f32,
                wb[2] as f32,
            ]);
            color_buf.extend_from_slice(&[rgb[0], rgb[1], rgb[2], rgb[0], rgb[1], rgb[2]]);
        }
    }
    (positions, color_buf)
}

/// The `set_overlay` JSON for the `sketch-constraint-glyphs` group (always emitted,
/// empty when the sketch has no geometric constraints, so a stale group clears on the
/// next refresh).
pub fn constraint_glyphs_overlay_json(
    doc: &SketchDoc,
    diag: &SketchDiagnostics,
    plane: &PlaneFrame,
    world_per_pixel: f64,
    colors: &SketchColors,
) -> String {
    constraint_glyphs_overlay_json_with_state(doc, diag, plane, world_per_pixel, colors, None, &[])
}

/// Like [`constraint_glyphs_overlay_json`], but colors the hovered / selected
/// constraint's glyph distinctly (S: constraint selection).
pub fn constraint_glyphs_overlay_json_with_state(
    doc: &SketchDoc,
    diag: &SketchDiagnostics,
    plane: &PlaneFrame,
    world_per_pixel: f64,
    colors: &SketchColors,
    hovered: Option<&Value>,
    selection: &[Value],
) -> String {
    let (positions, color_buf) =
        constraint_glyphs_buffers_with_state(doc, diag, plane, world_per_pixel, colors, hovered, selection);
    serde_json::json!({
        "groups": [
            {
                "name": OVERLAY_CONSTRAINT_GLYPHS,
                "renderOrder": 10004,
                "lines": { "positions": positions, "colors": color_buf },
            }
        ]
    })
    .to_string()
}

