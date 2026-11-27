//! The hovered entity's remaining freedoms, drawn on the canvas.
//!
//! The solver reports, per movable point, how many ways it can still move and
//! along which directions ([`SketchDiagnostics::point_freedom`]). Hovering a point
//! draws a double-headed arrow along each of its free directions; hovering a
//! geometry draws them at each of its movable points. The sign of a free
//! direction means nothing (the point can go either way), hence two heads.

use serde_json::Value;

use super::doc::{id_key, SketchDiagnostics, SketchDoc};
use super::PlaneFrame;
use crate::style::SketchColors;

/// The arrows' overlay group (its own group so it clears independently).
pub const OVERLAY_DOF_ARROWS: &str = "sketch-dof-arrows";

/// Half an arrow's length, in CSS px.
const ARROW_HALF_PX: f64 = 22.0;
/// An arrowhead's length, in CSS px.
const HEAD_PX: f64 = 6.0;

/// The plane segments of the free-direction arrows for the hovered entity
/// (`{"kind": "point" | "geometry", "id"}`), or none.
pub fn freedom_arrow_segments(
    doc: &SketchDoc,
    diag: &SketchDiagnostics,
    world_per_pixel: f64,
    hovered: Option<&Value>,
) -> Vec<([f64; 2], [f64; 2])> {
    let mut out = Vec::new();
    let Some(hovered) = hovered else { return out };
    let Some(id) = hovered.get("id") else { return out };
    let point_ids: Vec<Value> = match hovered.get("kind").and_then(Value::as_str) {
        Some("point") => vec![id.clone()],
        Some("geometry") => doc
            .geometries
            .iter()
            .find(|g| id_key(&g.id) == id_key(id))
            .map(|g| g.points.clone())
            .unwrap_or_default(),
        _ => return out,
    };
    let wpp = if world_per_pixel > 0.0 && world_per_pixel.is_finite() { world_per_pixel } else { 0.05 };
    let (half, head) = (ARROW_HALF_PX * wpp, HEAD_PX * wpp);
    let mut seen: Vec<String> = Vec::new();
    for pid in &point_ids {
        let key = id_key(pid);
        if seen.contains(&key) {
            continue;
        }
        seen.push(key.clone());
        let (Some(p), Some(freedom)) = (doc.point(pid), diag.point_freedom.get(&key)) else { continue };
        for d in &freedom.directions {
            let len = d[0].hypot(d[1]);
            if !(len > 1e-12) {
                continue;
            }
            let (ux, uy) = (d[0] / len, d[1] / len);
            let a = [p.x - ux * half, p.y - uy * half];
            let b = [p.x + ux * half, p.y + uy * half];
            out.push((a, b));
            // A head at each end, pointing outward.
            for (tip, sx, sy) in [(b, ux, uy), (a, -ux, -uy)] {
                let back = [tip[0] - sx * head, tip[1] - sy * head];
                let (wx, wy) = (-sy * head * 0.5, sx * head * 0.5);
                out.push((tip, [back[0] + wx, back[1] + wy]));
                out.push((tip, [back[0] - wx, back[1] - wy]));
            }
        }
    }
    out
}

/// The `set_overlay` JSON for the [`OVERLAY_DOF_ARROWS`] group, in the movable
/// colour (always emitted, empty when nothing movable is hovered).
pub fn freedom_arrows_overlay_json(
    doc: &SketchDoc,
    diag: &SketchDiagnostics,
    plane: &PlaneFrame,
    world_per_pixel: f64,
    colors: &SketchColors,
    hovered: Option<&Value>,
) -> String {
    let rgb = crate::color::hex_to_srgb_f32(colors.movable);
    let mut positions: Vec<f32> = Vec::new();
    let mut color_buf: Vec<f32> = Vec::new();
    for (a, b) in freedom_arrow_segments(doc, diag, world_per_pixel, hovered) {
        let wa = plane.to_world(a[0], a[1]);
        let wb = plane.to_world(b[0], b[1]);
        positions.extend_from_slice(&[
            wa[0] as f32, wa[1] as f32, wa[2] as f32, wb[0] as f32, wb[1] as f32, wb[2] as f32,
        ]);
        color_buf.extend_from_slice(&[rgb[0], rgb[1], rgb[2], rgb[0], rgb[1], rgb[2]]);
    }
    serde_json::json!({
        "groups": [{
            "name": OVERLAY_DOF_ARROWS,
            "renderOrder": 10005,
            "lines": { "positions": positions, "colors": color_buf },
        }]
    })
    .to_string()
}

/// The hover readout for an entity: `"2 DOF free"`, `"1 DOF free"` or
/// `"Fully constrained"` for a point or geometry, `None` for anything else (or a
/// stale id).
pub fn freedom_label(doc: &SketchDoc, diag: &SketchDiagnostics, entity: &Value) -> Option<String> {
    let id = entity.get("id")?;
    let dof = match entity.get("kind").and_then(Value::as_str)? {
        "point" => {
            doc.point(id)?;
            diag.point_freedom.get(&id_key(id)).map_or(0, |f| f.dof)
        }
        "geometry" => {
            doc.geometries.iter().find(|g| id_key(&g.id) == id_key(id))?;
            diag.geometry_freedom.get(&id_key(id)).copied().unwrap_or(0)
        }
        _ => return None,
    };
    Some(if dof > 0 { format!("{dof} DOF free") } else { "Fully constrained".to_string() })
}

