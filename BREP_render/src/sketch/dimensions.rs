//! Dimension leaders and labels for distance, point-line distance, angle,
//! radius, and diameter constraints.
//!
//! `dim_offsets[cid] = {du, dv}` is the plane-space vector from a dimension's
//! anchor to its label. Leaders and labels share this offset, with per-type
//! defaults when absent. Interaction state overrides the configured constraint color.

use std::collections::HashMap;
use std::f64::consts::PI;

use serde_json::Value;

use super::doc::{id_key, point_index, SketchConstraint, SketchDiagnostics, SketchDoc, SketchPoint};
use super::PlaneFrame;
use crate::style::SketchColors;

/// The dimension-leaders overlay group name (its own group so it upserts/clears
/// independently of geometry/points/preview).
pub const OVERLAY_DIM_LEADERS: &str = "sketch-dim-leaders";

/// The label placement + metadata for one dimensional constraint.
#[derive(Clone, Debug)]
pub struct DimLabel {
    /// The constraint id (raw `Value`).
    pub id: Value,
    /// The rendered text (`value.toFixed(3)`, `R…`, `⌀…`, or the angle value).
    pub text: String,
    /// The label anchor in world space (`plane.to_world(label_uv)`).
    pub world: [f64; 3],
    /// The stored numeric solver value (a radius for radial dims), if finite.
    pub value: Option<f64>,
    /// The stored `valueExpr` string (if any).
    pub value_expr: Option<String>,
    /// The display mode: `"distance" | "radius" | "diameter" | "angle"`.
    pub mode: &'static str,
    /// The solver names this dimension in a CONFLICT — brep-app paints the value
    /// text red to match its leader. The label text is drawn by the app (not the
    /// overlay), so the flag has to ride along with the placement.
    pub conflicting: bool,
}

/// The plane-space geometry + label anchor for one dimensional constraint.
struct DimGeometry {
    /// Leader/arrow segments as `(a, b)` pairs in plane `(u, v)`.
    segments: Vec<([f64; 2], [f64; 2])>,
    /// The label anchor in plane `(u, v)`.
    label_uv: [f64; 2],
    text: String,
    value: Option<f64>,
    value_expr: Option<String>,
    mode: &'static str,
}

/// Whether `c` is a RADIAL dimension (`⟺` on `[center, rim]` with a
/// radius/diameter display style) — port of `isRadialDimensionConstraint`.
fn is_radial_dimension(c: &SketchConstraint) -> bool {
    if c.ctype() != Some("⟺") || c.points().len() < 2 {
        return false;
    }
    matches!(
        c.raw.get("displayStyle").and_then(Value::as_str),
        Some("radius") | Some("diameter")
    )
}

/// Read the persisted `{du, dv}` plane offset for a constraint id, or `None` when
/// absent / non-finite (so callers can apply the per-type default).
fn saved_offset(dim_offsets: &serde_json::Map<String, Value>, id: &Value) -> Option<[f64; 2]> {
    let entry = dim_offsets.get(&id_key(id))?;
    let du = entry.get("du").and_then(Value::as_f64);
    let dv = entry.get("dv").and_then(Value::as_f64);
    match (du, dv) {
        (Some(du), Some(dv)) if du.is_finite() && dv.is_finite() => Some([du, dv]),
        // A partial offset still counts (either finite counts as set).
        (Some(du), None) if du.is_finite() => Some([du, 0.0]),
        (None, Some(dv)) if dv.is_finite() => Some([0.0, dv]),
        _ => None,
    }
}

/// The linear-distance endpoints for a `⟺` (the two points) or a `↥`
/// point-line-distance (the point projected onto the infinite line + the point) —
/// port of `resolveLinearDistanceEndpoints`. Returns `(a, b, anchor_a)` where
/// `anchor_a` is where the extension line starts (the nearest point on the finite
/// segment for `↥`, else `a`).
fn linear_endpoints(
    c: &SketchConstraint,
    by_id: &HashMap<String, &SketchPoint>,
) -> Option<([f64; 2], [f64; 2], [f64; 2])> {
    let get = |id: &Value| by_id.get(&id_key(id)).copied();
    let pts = c.points();
    match c.ctype() {
        Some("⟺") if pts.len() >= 2 && !is_radial_dimension(c) => {
            let p0 = get(&pts[0])?;
            let p1 = get(&pts[1])?;
            let a = [p0.x, p0.y];
            Some((a, [p1.x, p1.y], a))
        }
        Some("↥") if pts.len() >= 3 => {
            let a = get(&pts[0])?;
            let b = get(&pts[1])?;
            let cpt = get(&pts[2])?;
            let dx = b.x - a.x;
            let dy = b.y - a.y;
            let len_sq = dx * dx + dy * dy;
            if !(len_sq > 1e-12) {
                return Some(([a.x, a.y], [cpt.x, cpt.y], [a.x, a.y]));
            }
            let t_raw = ((cpt.x - a.x) * dx + (cpt.y - a.y) * dy) / len_sq;
            let t_raw = if t_raw.is_finite() { t_raw } else { 0.0 };
            let t_seg = t_raw.clamp(0.0, 1.0);
            Some((
                [a.x + t_raw * dx, a.y + t_raw * dy],
                [cpt.x, cpt.y],
                [a.x + t_seg * dx, a.y + t_seg * dy],
            ))
        }
        _ => None,
    }
}

/// The default radial label offset from the CENTER — port of `radialOffsetToPlane`
/// (the no-saved branch).
fn radial_default_offset(pc: [f64; 2], pr: [f64; 2]) -> [f64; 2] {
    let vx = pr[0] - pc[0];
    let vy = pr[1] - pc[1];
    let l = vx.hypot(vy);
    let (rx, ry) = if l > 1e-9 { (vx / l, vy / l) } else { (1.0, 0.0) };
    let (nx, ny) = (-ry, rx);
    let base = (l * 0.35).max(0.2);
    [
        vx + rx * base + nx * base * 0.35,
        vy + ry * base + ny * base * 0.35,
    ]
}

/// Push a two-line arrowhead at `tip` pointing along unit `(dx, dy)` with head
/// length `ah` and half-width factor `s` (port of the previous arrowhead builder).
fn push_arrow(
    segments: &mut Vec<([f64; 2], [f64; 2])>,
    tip: [f64; 2],
    dx: f64,
    dy: f64,
    ah: f64,
    s: f64,
) {
    let len = dx.hypot(dy);
    if !(len > 1e-9) {
        return;
    }
    let (tx, ty) = (dx / len, dy / len);
    let (wx, wy) = (-ty, tx);
    let a = [tip[0] + tx * ah + wx * ah * s, tip[1] + ty * ah + wy * ah * s];
    let b = [tip[0] + tx * ah - wx * ah * s, tip[1] + ty * ah - wy * ah * s];
    segments.push((tip, a));
    segments.push((tip, b));
}

/// Assemble the leader geometry + label anchor for one dimensional constraint, or
/// `None` for a non-dimensional / unresolvable constraint.
fn dim_geometry(
    c: &SketchConstraint,
    by_id: &HashMap<String, &SketchPoint>,
    dim_offsets: &serde_json::Map<String, Value>,
    wpp: f64,
) -> Option<DimGeometry> {
    let ctype = c.ctype()?;
    let id = c.raw.get("id")?;
    let value = c
        .raw
        .get("value")
        .and_then(Value::as_f64)
        .filter(|v| v.is_finite());
    let value_expr = c
        .raw
        .get("valueExpr")
        .and_then(Value::as_str)
        .filter(|s| !s.is_empty())
        .map(str::to_string);

    // Screen-constant sizes (port of the previous `worldPerPixel`-scaled bases).
    let ah = (wpp * 6.0).max(0.06);
    let arrow_s = 0.6;

    if is_radial_dimension(c) {
        // --- radius / diameter -------------------------------------------------
        let pts = c.points();
        let pc = by_id.get(&id_key(&pts[0])).copied()?;
        let pr = by_id.get(&id_key(&pts[1])).copied()?;
        let center = [pc.x, pc.y];
        let radius = (pr.x - pc.x).hypot(pr.y - pc.y);
        if !(radius > 1e-9) {
            return None;
        }
        let diameter = c.raw.get("displayStyle").and_then(Value::as_str) == Some("diameter");
        let off = saved_offset(dim_offsets, id).unwrap_or_else(|| radial_default_offset(center, [pr.x, pr.y]));
        let label_uv = [center[0] + off[0], center[1] + off[1]];

        // Direction center -> label (fall back to center -> rim).
        let (mut dx, mut dy) = (label_uv[0] - center[0], label_uv[1] - center[1]);
        let mut l = dx.hypot(dy);
        if !(l > 1e-9) {
            dx = pr.x - pc.x;
            dy = pr.y - pc.y;
            l = dx.hypot(dy).max(1e-9);
        }
        let (ux, uy) = (dx / l, dy / l);
        let near = [center[0] + ux * radius, center[1] + uy * radius];
        let far = [center[0] - ux * radius, center[1] - uy * radius];

        let mut segments = Vec::new();
        if diameter {
            segments.push((far, near));
            segments.push((near, label_uv));
            // Arrowheads point inward toward the center from both ends.
            push_arrow(&mut segments, near, -ux, -uy, ah, arrow_s);
            push_arrow(&mut segments, far, ux, uy, ah, arrow_s);
        } else {
            segments.push((center, near));
            segments.push((near, label_uv));
            push_arrow(&mut segments, near, -ux, -uy, ah, arrow_s);
        }

        let safe = value.unwrap_or(0.0);
        let (text, mode) = if diameter {
            (format!("⌀{:.3}", 2.0 * safe), "diameter")
        } else {
            (format!("R{safe:.3}"), "radius")
        };
        return Some(DimGeometry {
            segments,
            label_uv,
            text,
            value,
            value_expr,
            mode,
        });
    }

    if ctype == "⟺" || ctype == "↥" {
        // --- linear distance ---------------------------------------------------
        let (a, b, anchor_a) = linear_endpoints(c, by_id)?;
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let len = dx.hypot(dy).max(1e-9);
        let (tx, ty) = (dx / len, dy / len);
        let (nx, ny) = (-ty, tx);
        let base = (wpp * 20.0).max(0.1);
        let mid = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
        // Offset from the midpoint to the label; default a normal push of `base`.
        let off = saved_offset(dim_offsets, id).unwrap_or([nx * base, ny * base]);
        let label_uv = [mid[0] + off[0], mid[1] + off[1]];
        // The dimension line runs parallel to the segment through the label's
        // normal component (project the offset onto the normal so the line stays
        // parallel and the extension lines meet it squarely).
        let off_n = off[0] * nx + off[1] * ny;
        let ou = nx * off_n;
        let ov = ny * off_n;

        let a_off = [a[0] + ou, a[1] + ov];
        let b_off = [b[0] + ou, b[1] + ov];
        let mut segments = vec![
            (a_off, b_off),       // dimension line
            (anchor_a, a_off),    // extension line at a
            (b, b_off),           // extension line at b
        ];
        // Opposed arrowheads at the dimension-line ends, pointing toward the span.
        push_arrow(&mut segments, a_off, tx, ty, ah, arrow_s);
        push_arrow(&mut segments, b_off, -tx, -ty, ah, arrow_s);

        // Leader: the label rides the (extended) dimension line, so when it is
        // dragged PAST the span it detaches visually. Connect it back to the nearest
        // end of the dimension line so the user can tell which dimension it belongs
        // to. (Within the span the label sits on the line, so the gap is ~0 and no
        // leader is drawn — matching the radial dimension's rim→label leader.)
        let t_lbl = (label_uv[0] - a_off[0]) * tx + (label_uv[1] - a_off[1]) * ty;
        let foot = {
            let t = t_lbl.clamp(0.0, len);
            [a_off[0] + tx * t, a_off[1] + ty * t]
        };
        if (label_uv[0] - foot[0]).hypot(label_uv[1] - foot[1]) > wpp * 2.0 {
            segments.push((foot, label_uv));
        }

        let text = format!("{:.3}", value.unwrap_or(0.0));
        return Some(DimGeometry {
            segments,
            label_uv,
            text,
            value,
            value_expr,
            mode: "distance",
        });
    }

    if ctype == "∠" && c.points().len() >= 4 {
        // --- angle -------------------------------------------------------------
        let pts = c.points();
        let get = |i: usize| by_id.get(&id_key(&pts[i])).copied();
        let (p0, p1, p2, p3) = (get(0)?, get(1)?, get(2)?, get(3)?);
        let inter = line_intersection([p0.x, p0.y], [p1.x, p1.y], [p2.x, p2.y], [p3.x, p3.y]);
        let (d1x, d1y) = (p1.x - p0.x, p1.y - p0.y);
        let (d2x, d2y) = (p3.x - p2.x, p3.y - p2.y);
        if (d1x * d1x + d1y * d1y) < 1e-12 || (d2x * d2x + d2y * d2y) < 1e-12 {
            return None;
        }
        let a0 = d1y.atan2(d1x);
        let a1 = d2y.atan2(d2x);
        let mut signed = a1 - a0;
        while signed <= -PI {
            signed += 2.0 * PI;
        }
        while signed > PI {
            signed -= 2.0 * PI;
        }
        let default_deg = (a1 - a0).to_degrees().rem_euclid(360.0);
        let mut target_deg = match value {
            Some(v) => {
                let abs = v.abs();
                let mut t = abs % 360.0;
                if t < 1e-6 && abs > 0.0 {
                    t = 360.0;
                }
                t
            }
            None => default_deg,
        };
        if target_deg < 1e-6 {
            target_deg = 1e-6;
        }
        let mut dir_sign = if signed == 0.0 { 1.0 } else { signed.signum() };
        if target_deg > 180.0 && target_deg < 360.0 - 1e-6 {
            dir_sign = -dir_sign;
        }
        let mut d = target_deg.to_radians() * dir_sign;
        let two_pi = 2.0 * PI;
        if d.abs() > two_pi {
            d = d.signum() * (two_pi - 1e-6);
        }

        let base_r = (wpp * 24.0).max(0.3);
        let off = saved_offset(dim_offsets, id).unwrap_or([0.0, 0.0]);
        let r = base_r + off[0].hypot(off[1]);
        let (cx, cy) = (inter[0], inter[1]);

        // Choose the arc side to sit on the label's side (port of the bisector flip).
        let mut a_start = a0;
        if off[0] != 0.0 || off[1] != 0.0 {
            let label_ang = off[1].atan2(off[0]);
            let norm = |a: f64| {
                let t = two_pi;
                let m = a % t;
                if m < 0.0 {
                    m + t
                } else {
                    m
                }
            };
            let ang_diff = |a: f64, b: f64| {
                let mut x = norm(a - b);
                if x > PI {
                    x = two_pi - x;
                }
                x.abs()
            };
            let bisector = a_start + d * 0.5;
            let bisector_opp = bisector + PI;
            if ang_diff(label_ang, bisector_opp) + 1e-6 < ang_diff(label_ang, bisector) {
                a_start += PI;
            }
        }

        // Arc polyline.
        let segs = 48usize;
        let mut segments = Vec::with_capacity(segs + 4);
        let mut prev = [cx + a_start.cos() * r, cy + a_start.sin() * r];
        for i in 1..=segs {
            let t = a_start + d * (i as f64 / segs as f64);
            let cur = [cx + t.cos() * r, cy + t.sin() * r];
            segments.push((prev, cur));
            prev = cur;
        }
        // Arrowheads at both arc ends (tangential, facing each other).
        let dir_start = if d >= 0.0 { 1.0 } else { -1.0 };
        let add_arc_arrow = |segments: &mut Vec<([f64; 2], [f64; 2])>, t: f64, dir: f64| {
            let tip = [cx + t.cos() * r, cy + t.sin() * r];
            let (tx, ty) = (-t.sin() * dir, t.cos() * dir);
            push_arrow(segments, tip, tx, ty, ah, arrow_s);
        };
        add_arc_arrow(&mut segments, a_start, dir_start);
        add_arc_arrow(&mut segments, a_start + d, -dir_start);

        // Label at the arc midpoint.
        let mid_ang = a_start + d * 0.5;
        let label_uv = [cx + mid_ang.cos() * r, cy + mid_ang.sin() * r];

        // Angle label: degrees with a ° symbol (the stored value IS degrees). An
        // unset angle shows the measured default so the label is never blank.
        let text = format!("{:.1}°", value.unwrap_or(default_deg));
        return Some(DimGeometry {
            segments,
            label_uv,
            text,
            value,
            value_expr,
            mode: "angle",
        });
    }

    None
}

/// Robust 2D infinite-line intersection (port of `intersect`); falls back to `a`
/// on (near-)parallel lines to avoid NaNs. Shared with [`super::constraint_glyphs`].
pub(super) fn line_intersection(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> [f64; 2] {
    let r = [b[0] - a[0], b[1] - a[1]];
    let s = [d[0] - c[0], d[1] - c[1]];
    let rxs = r[0] * s[1] - r[1] * s[0];
    if rxs.abs() < 1e-12 {
        return a;
    }
    let t = ((c[0] - a[0]) * s[1] - (c[1] - a[1]) * s[0]) / rxs;
    [a[0] + t * r[0], a[1] + t * r[1]]
}

/// The plane-space LEADER segments for ONE dimensional constraint, resolved against
/// `doc` (`None` for a non-dimensional / unresolvable constraint). These are the
/// pick + selection-emphasis target for a dimension: the leader lines (extension +
/// dimension line + arrows), NOT the value label — the label is the egui value-edit
/// affordance, so "click the leader" selects and "click the label" edits.
pub fn constraint_dim_segments(
    c: &SketchConstraint,
    doc: &SketchDoc,
    dim_offsets: &serde_json::Map<String, Value>,
    world_per_pixel: f64,
) -> Option<Vec<([f64; 2], [f64; 2])>> {
    let by_id = point_index(doc);
    dim_geometry(c, &by_id, dim_offsets, world_per_pixel).map(|g| g.segments)
}

/// Build the world-space leader/arrow line segments for every dimensional
/// constraint, as flat `(positions, colors)` buffers (6 position + 6 color floats
/// per segment) ready to feed the `sketch-dim-leaders` overlay group.
pub fn dimension_leaders_buffers(
    doc: &SketchDoc,
    diag: &SketchDiagnostics,
    plane: &PlaneFrame,
    dim_offsets: &serde_json::Map<String, Value>,
    world_per_pixel: f64,
    colors: &SketchColors,
) -> (Vec<f32>, Vec<f32>) {
    dimension_leaders_buffers_with_state(
        doc,
        diag,
        plane,
        dim_offsets,
        world_per_pixel,
        colors,
        None,
        &[],
    )
}

/// Like [`dimension_leaders_buffers`], but emphasizes the hovered / selected
/// dimensional constraint (matched by a `{"kind":"constraint","id":…}` ref) in the
/// SAME amber / light-blue as a selected / hovered point or geometry; an empty hover
/// + selection reproduce the plain-green output.
pub fn dimension_leaders_buffers_with_state(
    doc: &SketchDoc,
    diag: &SketchDiagnostics,
    plane: &PlaneFrame,
    dim_offsets: &serde_json::Map<String, Value>,
    world_per_pixel: f64,
    colors: &SketchColors,
    hovered: Option<&Value>,
    selection: &[Value],
) -> (Vec<f32>, Vec<f32>) {
    let by_id = point_index(doc);
    let mut positions: Vec<f32> = Vec::new();
    let mut color_buf: Vec<f32> = Vec::new();
    for c in &doc.constraints {
        let Some(geom) = dim_geometry(c, &by_id, dim_offsets, world_per_pixel) else {
            continue;
        };
        // Green normally, RED while the solver names this dimension in a conflict;
        // selection (amber) and hover (light blue) still win over both.
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
        for (a, b) in geom.segments {
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

/// The `set_overlay` JSON for the `sketch-dim-leaders` group (always emitted, empty
/// when there are no dimensions, so a stale group is cleared).
pub fn dimension_leaders_overlay_json(
    doc: &SketchDoc,
    diag: &SketchDiagnostics,
    plane: &PlaneFrame,
    dim_offsets: &serde_json::Map<String, Value>,
    world_per_pixel: f64,
    colors: &SketchColors,
) -> String {
    dimension_leaders_overlay_json_with_state(
        doc,
        diag,
        plane,
        dim_offsets,
        world_per_pixel,
        colors,
        None,
        &[],
    )
}

/// Like [`dimension_leaders_overlay_json`], but colors the hovered / selected
/// dimensional constraint's leader distinctly (S: constraint selection).
pub fn dimension_leaders_overlay_json_with_state(
    doc: &SketchDoc,
    diag: &SketchDiagnostics,
    plane: &PlaneFrame,
    dim_offsets: &serde_json::Map<String, Value>,
    world_per_pixel: f64,
    colors: &SketchColors,
    hovered: Option<&Value>,
    selection: &[Value],
) -> String {
    let (positions, color_buf) = dimension_leaders_buffers_with_state(
        doc,
        diag,
        plane,
        dim_offsets,
        world_per_pixel,
        colors,
        hovered,
        selection,
    );
    serde_json::json!({
        "groups": [
            {
                "name": OVERLAY_DIM_LEADERS,
                "renderOrder": 10003,
                "lines": { "positions": positions, "colors": color_buf },
            }
        ]
    })
    .to_string()
}

/// The per-constraint label placements + metadata (one [`DimLabel`] per dimensional
/// constraint), with each label anchored in world space.
pub fn dimension_labels(
    doc: &SketchDoc,
    diag: &SketchDiagnostics,
    plane: &PlaneFrame,
    dim_offsets: &serde_json::Map<String, Value>,
    world_per_pixel: f64,
) -> Vec<DimLabel> {
    let by_id = point_index(doc);
    let mut out = Vec::new();
    for c in &doc.constraints {
        let Some(geom) = dim_geometry(c, &by_id, dim_offsets, world_per_pixel) else {
            continue;
        };
        let Some(id) = c.raw.get("id").cloned() else {
            continue;
        };
        out.push(DimLabel {
            conflicting: diag.constraint_conflicting(&id),
            id,
            text: geom.text,
            world: plane.to_world(geom.label_uv[0], geom.label_uv[1]),
            value: geom.value,
            value_expr: geom.value_expr,
            mode: geom.mode,
        });
    }
    out
}

/// The plane `(u, v)` ANCHOR a dimension's label offset is measured FROM — the
/// midpoint for a linear distance, the center for a radial dim, the line
/// intersection for an angle. Used by the drag-to-reposition mapping so the stored
/// `{du, dv}` = (label uv) − (anchor uv). `None` for a non-dimensional constraint.
pub fn dimension_anchor_uv(
    c: &SketchConstraint,
    doc: &SketchDoc,
) -> Option<[f64; 2]> {
    let by_id = point_index(doc);
    let get = |id: &Value| by_id.get(&id_key(id)).copied();
    if is_radial_dimension(c) {
        let pc = get(&c.points()[0])?;
        return Some([pc.x, pc.y]);
    }
    match c.ctype() {
        Some("⟺") | Some("↥") => {
            let (a, b, _) = linear_endpoints(c, &by_id)?;
            Some([(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0])
        }
        Some("∠") if c.points().len() >= 4 => {
            let pts = c.points();
            let g = |i: usize| get(&pts[i]);
            let (p0, p1, p2, p3) = (g(0)?, g(1)?, g(2)?, g(3)?);
            Some(line_intersection(
                [p0.x, p0.y],
                [p1.x, p1.y],
                [p2.x, p2.y],
                [p3.x, p3.y],
            ))
        }
        _ => None,
    }
}

