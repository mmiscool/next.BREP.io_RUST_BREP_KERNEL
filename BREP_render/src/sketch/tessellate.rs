//! Tessellate solved sketch geometry into world-space overlay lines and points.
//!
//! Curves are sampled in the sketch plane; construction geometry uses dashed
//! polylines. Colors come from solver mobility and interaction state, with
//! selection taking precedence over hover. Overlay colors use normalized sRGB.

use std::f64::consts::PI;

use serde_json::Value;

use super::doc::{id_key, point_index, SketchDiagnostics, SketchDoc, SketchGeometry, SketchPoint};
use super::PlaneFrame;
use crate::color::hex_to_srgb_f32 as rgb;
use crate::style::SketchColors;

/// Circle/bezier-span segment count (the previous overlay's 64-gon).
const CIRCLE_SEGMENTS: usize = 64;

/// The `set_overlay` group names (byte-identical to the previous sketcher's).
pub const OVERLAY_GEOMETRY: &str = "sketch-geometry";
pub const OVERLAY_POINTS: &str = "sketch-points";
/// The in-progress draw-tool rubber-band preview group (S3a).
pub const OVERLAY_PREVIEW: &str = "sketch-preview";

/// Screen-constant point size in CSS px (the previous `pointSizePx` default). The
/// sketch pick/highlight radius is derived from this (see
/// `EngineState::sketch_pick_radius`) so the two track one number.
pub(crate) const POINT_SIZE_PX: f32 = 8.0;

/// Flat overlay buffers ready for the `set_overlay` channel: `line_*` hold
/// consecutive segment endpoints (xyz per vertex / rgb per vertex), `point_*` hold
/// one billboarded point per vertex.
#[derive(Clone, Debug, Default)]
pub struct SketchTessellation {
    pub line_positions: Vec<f32>,
    pub line_colors: Vec<f32>,
    pub point_positions: Vec<f32>,
    pub point_colors: Vec<f32>,
}

impl SketchTessellation {
    /// Number of line SEGMENTS (2 vertices = 6 position floats each).
    pub fn line_segment_count(&self) -> usize {
        self.line_positions.len() / 6
    }

    /// Number of overlay POINTS (3 position floats each).
    pub fn point_count(&self) -> usize {
        self.point_positions.len() / 3
    }
}

/// Tessellate without interaction coloring. `world_per_pixel` sizes construction dashes.
pub fn tessellate(
    doc: &SketchDoc,
    diag: &SketchDiagnostics,
    plane: &PlaneFrame,
    world_per_pixel: f64,
    colors: &SketchColors,
) -> SketchTessellation {
    tessellate_with_state(doc, diag, plane, world_per_pixel, colors, None, &[])
}

/// Tessellate a solved sketch, coloring the hovered + selected entities distinctly:
/// a selected entity paints `colors.selected` (amber), a hovered
/// one `colors.hovered` (light blue); selection beats hover, and both override the
/// mobility color. Every color comes from `colors` (a [`SketchColors`] view of the
/// display settings). `hovered` / `selection` are entity refs
/// (`{"kind":"point"|"geometry","id":<id>}`); an empty hover + selection reproduce
/// the read-only [`tessellate`] output byte-for-byte.
pub fn tessellate_with_state(
    doc: &SketchDoc,
    diag: &SketchDiagnostics,
    plane: &PlaneFrame,
    world_per_pixel: f64,
    colors: &SketchColors,
    hovered: Option<&Value>,
    selection: &[Value],
) -> SketchTessellation {
    let mut out = SketchTessellation::default();

    let by_id = point_index(doc);
    let get = |id: &serde_json::Value| by_id.get(&id_key(id)).copied();

    let dash_len = (8.0 * world_per_pixel).max(0.02);
    let gap_len = (6.0 * world_per_pixel).max(0.01);

    // --- curves -> overlay line segments --------------------------------------
    for geo in &doc.geometries {
        let pts = match sample_geometry(geo, &get, plane) {
            Some(pts) if pts.len() >= 2 => pts,
            _ => continue,
        };
        let mobility = match diag.geometry_movable(&geo.id) {
            None => colors.geometry,
            Some(true) => colors.movable,
            Some(false) => colors.locked,
        };
        let color = interaction_color(colors, mobility, hovered, selection, "geometry", &geo.id);
        let rgb = rgb(color);
        if geo.construction() {
            push_dashed(&mut out, &pts, rgb, dash_len, gap_len);
        } else {
            push_solid(&mut out, &pts, rgb);
        }
    }

    // --- points -> billboarded overlay points ---------------------------------
    let constrained = doc.constrained_point_keys();
    for p in &doc.points {
        let mobility = diag.point_movable(&p.id);
        let base = if p.construction {
            colors.construction_point
        } else if let Some(movable) = mobility {
            if movable {
                colors.movable
            } else {
                colors.locked
            }
        } else {
            // No solver mobility (shouldn't happen for a solved sketch): fall back
            // to the default heuristic — a free, unconstrained, unpinned point reads as
            // under-constrained.
            let under = !p.fixed && !constrained.contains(&id_key(&p.id));
            if under {
                colors.under_constrained_point
            } else {
                colors.point
            }
        };
        let color = interaction_color(colors, base, hovered, selection, "point", &p.id);
        let w = plane.to_world(p.x, p.y);
        out.point_positions
            .extend_from_slice(&[w[0] as f32, w[1] as f32, w[2] as f32]);
        let c = rgb(color);
        out.point_colors.extend_from_slice(&c);
    }

    out
}

/// Resolve the final draw color for an entity: selection (`colors.selected`, amber)
/// beats hover (`colors.hovered`, light blue), and either overrides the passed-in
/// `base` mobility color. `pub(crate)` so the constraint-annotation overlays (glyphs +
/// dim leaders) emphasize a selected / hovered constraint with the SAME amber /
/// light-blue as points + geometry.
pub(crate) fn interaction_color(
    colors: &SketchColors,
    base: u32,
    hovered: Option<&Value>,
    selection: &[Value],
    kind: &str,
    id: &Value,
) -> u32 {
    if selection.iter().any(|r| ref_matches(r, kind, id)) {
        colors.selected
    } else if hovered.map_or(false, |r| ref_matches(r, kind, id)) {
        colors.hovered
    } else {
        base
    }
}

/// Whether an entity ref (`{"kind":…,"id":…}`) names the `(kind, id)` entity —
/// ids compared via [`id_key`] so a numeric `4` and string `"4"` match the solver's
/// identity keying. `pub(crate)` so the constraint overlays share the one matcher.
pub(crate) fn ref_matches(r: &Value, kind: &str, id: &Value) -> bool {
    r.get("kind").and_then(Value::as_str) == Some(kind)
        && r.get("id").map(id_key) == Some(id_key(id))
}

/// Build the `set_overlay` JSON (`{groups:[…]}`) for a solved sketch — two named
/// groups (`sketch-geometry` lines, `sketch-points` points) fed as-is to
/// `EngineState::set_overlay_json`.
pub fn overlay_json(
    doc: &SketchDoc,
    diag: &SketchDiagnostics,
    plane: &PlaneFrame,
    world_per_pixel: f64,
    colors: &SketchColors,
) -> String {
    overlay_json_with_state(doc, diag, plane, world_per_pixel, colors, None, &[])
}

/// Like [`overlay_json`], but colors the hovered + selected entities (S2).
pub fn overlay_json_with_state(
    doc: &SketchDoc,
    diag: &SketchDiagnostics,
    plane: &PlaneFrame,
    world_per_pixel: f64,
    colors: &SketchColors,
    hovered: Option<&Value>,
    selection: &[Value],
) -> String {
    let tess = tessellate_with_state(doc, diag, plane, world_per_pixel, colors, hovered, selection);
    serde_json::json!({
        "groups": [
            {
                "name": OVERLAY_GEOMETRY,
                "renderOrder": 10000,
                "lines": { "positions": tess.line_positions, "colors": tess.line_colors },
            },
            {
                "name": OVERLAY_POINTS,
                "renderOrder": 10001,
                "points": {
                    "positions": tess.point_positions,
                    "colors": tess.point_colors,
                    "size": POINT_SIZE_PX,
                },
            },
        ]
    })
    .to_string()
}

/// Sample one geometry into an ordered world-space polyline, or `None` if a
/// referenced point is missing or the geometry is degenerate. Mirrors the previous
/// per-type overlay sampling. The in-plane sampling lives in
/// [`sample_geometry_uv`]; this just embeds it in world space via the plane.
fn sample_geometry<'a>(
    geo: &SketchGeometry,
    get: &impl Fn(&serde_json::Value) -> Option<&'a SketchPoint>,
    plane: &PlaneFrame,
) -> Option<Vec<[f64; 3]>> {
    let uv = sample_geometry_uv(geo, get)?;
    Some(uv.into_iter().map(|[u, v]| plane.to_world(u, v)).collect())
}

/// Sample one geometry into an ordered in-plane `(u, v)` polyline (the same
/// per-type sampling [`sample_geometry`] embeds in world space), or `None` if a
/// referenced point is missing or the geometry is degenerate. This is the polyline
/// the S2 hit-test measures point-to-segment distance against.
fn sample_geometry_uv<'a>(
    geo: &SketchGeometry,
    get: &impl Fn(&serde_json::Value) -> Option<&'a SketchPoint>,
) -> Option<Vec<[f64; 2]>> {
    let ids = &geo.points;
    match geo.geom_type.as_str() {
        "line" if ids.len() >= 2 => {
            let p0 = get(&ids[0])?;
            let p1 = get(&ids[1])?;
            Some(vec![[p0.x, p0.y], [p1.x, p1.y]])
        }
        "circle" if ids.len() >= 2 => {
            let pc = get(&ids[0])?;
            let pr = get(&ids[1])?;
            let r = (pr.x - pc.x).hypot(pr.y - pc.y);
            if !r.is_finite() || r < 1e-9 {
                return None;
            }
            let mut pts = Vec::with_capacity(CIRCLE_SEGMENTS + 1);
            for i in 0..=CIRCLE_SEGMENTS {
                let t = (i as f64 / CIRCLE_SEGMENTS as f64) * 2.0 * PI;
                pts.push([pc.x + r * t.cos(), pc.y + r * t.sin()]);
            }
            Some(pts)
        }
        "arc" if ids.len() >= 3 => {
            let pc = get(&ids[0])?;
            let pa = get(&ids[1])?;
            let pb = get(&ids[2])?;
            let r = (pa.x - pc.x).hypot(pa.y - pc.y);
            if !r.is_finite() || r < 1e-9 {
                return None;
            }
            let a0 = (pa.y - pc.y).atan2(pa.x - pc.x);
            let a1 = (pb.y - pc.y).atan2(pb.x - pc.x);
            let mut d = a1 - a0;
            d = d.rem_euclid(2.0 * PI);
            if d.abs() < 1e-6 {
                d = 2.0 * PI;
            }
            let segs = (((CIRCLE_SEGMENTS as f64) * d / (2.0 * PI)).ceil() as usize).max(8);
            let mut pts = Vec::with_capacity(segs + 1);
            for i in 0..=segs {
                let t = a0 + d * (i as f64 / segs as f64);
                pts.push([pc.x + r * t.cos(), pc.y + r * t.sin()]);
            }
            Some(pts)
        }
        "ellipse" if ids.len() >= 3 => {
            // Affine image of the unit circle: c + cos(t)·(maj-c) + sin(t)·(min-c).
            let pc = get(&ids[0])?;
            let pmaj = get(&ids[1])?;
            let pmin = get(&ids[2])?;
            let (ex, ey) = (pmaj.x - pc.x, pmaj.y - pc.y);
            let (fx, fy) = (pmin.x - pc.x, pmin.y - pc.y);
            let mut pts = Vec::with_capacity(CIRCLE_SEGMENTS + 1);
            for i in 0..=CIRCLE_SEGMENTS {
                let t = (i as f64 / CIRCLE_SEGMENTS as f64) * 2.0 * PI;
                let (c, s) = (t.cos(), t.sin());
                pts.push([pc.x + c * ex + s * fx, pc.y + c * ey + s * fy]);
            }
            Some(pts)
        }
        "bezier" if ids.len() >= 4 => {
            let seg_count = (ids.len() - 1) / 3;
            if seg_count < 1 {
                return None;
            }
            let mut pts = Vec::new();
            for seg in 0..seg_count {
                let i0 = seg * 3;
                let p0 = get(&ids[i0])?;
                let p1 = get(&ids[i0 + 1])?;
                let p2 = get(&ids[i0 + 2])?;
                let p3 = get(&ids[i0 + 3])?;
                for i in 0..=CIRCLE_SEGMENTS {
                    if seg > 0 && i == 0 {
                        continue; // avoid duplicating the shared knot
                    }
                    let t = i as f64 / CIRCLE_SEGMENTS as f64;
                    let mt = 1.0 - t;
                    let bx = mt * mt * mt * p0.x
                        + 3.0 * mt * mt * t * p1.x
                        + 3.0 * mt * t * t * p2.x
                        + t * t * t * p3.x;
                    let by = mt * mt * mt * p0.y
                        + 3.0 * mt * mt * t * p1.y
                        + 3.0 * mt * t * t * p2.y
                        + t * t * t * p3.y;
                    pts.push([bx, by]);
                }
            }
            Some(pts)
        }
        _ => None,
    }
}

/// The in-plane `(u, v)` polyline for one geometry, resolving its point ids against
/// `doc` (the public entry the S2 hit-test uses). Empty when a referenced point is
/// missing or the geometry is degenerate/unknown.
pub fn geometry_polyline_uv(geom: &SketchGeometry, doc: &SketchDoc) -> Vec<[f64; 2]> {
    let by_id = point_index(doc);
    let get = |id: &serde_json::Value| by_id.get(&id_key(id)).copied();
    sample_geometry_uv(geom, &get).unwrap_or_default()
}

/// Push a solid polyline as consecutive line segments (per-vertex color).
fn push_solid(out: &mut SketchTessellation, pts: &[[f64; 3]], rgb: [f32; 3]) {
    for pair in pts.windows(2) {
        push_seg(out, pair[0], pair[1], rgb);
    }
}

/// Push a dashed polyline by walking its cumulative arc length (a port of the previous
/// overlay's dashed-polyline builder).
fn push_dashed(out: &mut SketchTessellation, pts: &[[f64; 3]], rgb: [f32; 3], dl: f64, gl: f64) {
    let mut on = true;
    let mut need = dl;
    for pair in pts.windows(2) {
        let (mut ax, mut ay, mut az) = (pair[0][0], pair[0][1], pair[0][2]);
        let (bx, by, bz) = (pair[1][0], pair[1][1], pair[1][2]);
        let mut seg_len = ((bx - ax).powi(2) + (by - ay).powi(2) + (bz - az).powi(2)).sqrt();
        if !(seg_len > 1e-12) {
            continue;
        }
        let (dx, dy, dz) = (
            (bx - ax) / seg_len,
            (by - ay) / seg_len,
            (bz - az) / seg_len,
        );
        while seg_len > 1e-9 {
            let step = need.min(seg_len);
            let (nx, ny, nz) = (ax + dx * step, ay + dy * step, az + dz * step);
            if on {
                push_seg(out, [ax, ay, az], [nx, ny, nz], rgb);
            }
            ax = nx;
            ay = ny;
            az = nz;
            seg_len -= step;
            need -= step;
            if need <= 1e-9 {
                on = !on;
                need = if on { dl } else { gl };
            }
        }
    }
}

fn push_seg(out: &mut SketchTessellation, a: [f64; 3], b: [f64; 3], rgb: [f32; 3]) {
    out.line_positions.extend_from_slice(&[
        a[0] as f32,
        a[1] as f32,
        a[2] as f32,
        b[0] as f32,
        b[1] as f32,
        b[2] as f32,
    ]);
    out.line_colors
        .extend_from_slice(&[rgb[0], rgb[1], rgb[2], rgb[0], rgb[1], rgb[2]]);
}

// ---------------------------------------------------------------------------
// S3a: draw-tool rubber-band preview.
//
// A separate `sketch-preview` overlay group carrying a dim, dashed polyline of the
// geometry the active tool is about to place — derived from the tool, the pending
// anchor points (in uv), and the current hovered uv. The group is ALWAYS emitted
// (empty when there is nothing to preview) so a stale rubber-band is cleared on the
// next refresh.
// ---------------------------------------------------------------------------

/// Build the `set_overlay` JSON (`{groups:[{name:"sketch-preview", …}]}`) for the
/// in-progress draw-tool geometry. `pending_uv` are the already-placed anchor
/// points (in plane uv), `hover_uv` the live cursor position; `stroke_uv` is the raw
/// freehand handdraw stroke being captured (S6b-3), rendered as a dim solid polyline.
/// `None`/empty inputs emit an empty (clearing) group.
pub fn preview_overlay_json(
    tool: Option<&str>,
    pending_uv: &[[f64; 2]],
    hover_uv: Option<(f64, f64)>,
    stroke_uv: &[[f64; 2]],
    plane: &PlaneFrame,
    world_per_pixel: f64,
    colors: &SketchColors,
) -> String {
    let mut out = SketchTessellation::default();
    let preview = rgb(colors.preview);
    // The live handdraw stroke: a dim SOLID polyline through the raw uv samples.
    if stroke_uv.len() >= 2 {
        let world: Vec<[f64; 3]> = stroke_uv
            .iter()
            .map(|&[u, v]| plane.to_world(u, v))
            .collect();
        for pair in world.windows(2) {
            push_seg(&mut out, pair[0], pair[1], preview);
        }
    }
    if let (Some(tool), Some((hu, hv))) = (tool, hover_uv) {
        let poly = preview_polyline_uv(tool, pending_uv, [hu, hv]);
        if poly.len() >= 2 {
            let world: Vec<[f64; 3]> = poly.iter().map(|&[u, v]| plane.to_world(u, v)).collect();
            let dash = (8.0 * world_per_pixel).max(0.02);
            let gap = (6.0 * world_per_pixel).max(0.01);
            push_dashed(&mut out, &world, preview, dash, gap);
        }
    }
    serde_json::json!({
        "groups": [
            {
                "name": OVERLAY_PREVIEW,
                "renderOrder": 10002,
                "lines": { "positions": out.line_positions, "colors": out.line_colors },
            }
        ]
    })
    .to_string()
}

/// The in-plane preview polyline for the active tool given its pending anchors + the
/// cursor. Empty when there is nothing to preview yet (e.g. `point`, or a tool with
/// no anchor placed).
fn preview_polyline_uv(tool: &str, pending: &[[f64; 2]], cursor: [f64; 2]) -> Vec<[f64; 2]> {
    match tool {
        // Chain segment from the last placed point to the cursor.
        "line" if !pending.is_empty() => vec![*pending.last().unwrap(), cursor],
        // Bezier: the CONTROL POLYGON — a guide polyline through the placed control
        // points and on to the cursor (the geom itself commits on the 4th click, so
        // the partial cubic is never previewed).
        "bezier" if !pending.is_empty() => {
            let mut poly: Vec<[f64; 2]> = pending.to_vec();
            poly.push(cursor);
            poly
        }
        // Axis-aligned (in uv) rectangle from corner A to the cursor (corner B).
        "rect" if !pending.is_empty() => {
            let [ax, ay] = pending[0];
            let [bx, by] = cursor;
            vec![[ax, ay], [bx, ay], [bx, by], [ax, by], [ax, ay]]
        }
        // Circle centered on the first click, radius = |cursor - center|.
        "circle" if !pending.is_empty() => {
            let [cx, cy] = pending[0];
            let r = (cursor[0] - cx).hypot(cursor[1] - cy);
            if r < 1e-9 {
                Vec::new()
            } else {
                circle_poly(cx, cy, r)
            }
        }
        // Only the center placed → preview the radius line to the cursor.
        "arc" if pending.len() == 1 => vec![pending[0], cursor],
        // Center + start placed → preview the CCW arc start→cursor.
        "arc" if pending.len() >= 2 => {
            let [cx, cy] = pending[0];
            let [sx, sy] = pending[1];
            let r = (sx - cx).hypot(sy - cy);
            if r < 1e-9 {
                Vec::new()
            } else {
                arc_poly(cx, cy, sx, sy, cursor[0], cursor[1], r)
            }
        }
        _ => Vec::new(),
    }
}

/// A closed 64-gon approximation of a circle (uv), matching the overlay renderer.
fn circle_poly(cx: f64, cy: f64, r: f64) -> Vec<[f64; 2]> {
    (0..=CIRCLE_SEGMENTS)
        .map(|i| {
            let t = (i as f64 / CIRCLE_SEGMENTS as f64) * 2.0 * PI;
            [cx + r * t.cos(), cy + r * t.sin()]
        })
        .collect()
}

/// A CCW arc polyline (uv) from `start` to `end` about `center` at radius `r`,
/// matching the overlay renderer's proportional segmentation.
fn arc_poly(cx: f64, cy: f64, sx: f64, sy: f64, ex: f64, ey: f64, r: f64) -> Vec<[f64; 2]> {
    let a0 = (sy - cy).atan2(sx - cx);
    let a1 = (ey - cy).atan2(ex - cx);
    let mut d = (a1 - a0).rem_euclid(2.0 * PI);
    if d.abs() < 1e-6 {
        d = 2.0 * PI;
    }
    let segs = (((CIRCLE_SEGMENTS as f64) * d / (2.0 * PI)).ceil() as usize).max(8);
    (0..=segs)
        .map(|i| {
            let t = a0 + d * (i as f64 / segs as f64);
            [cx + r * t.cos(), cy + r * t.sin()]
        })
        .collect()
}
