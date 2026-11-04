//! Expand persisted, normalized text contours after solving the construction frame.
//! Glyph handles are derived geometry: they never enter the constraint solver.
use serde_json::{json, Value};

/// Replace text objects with closed cubic contours in a solved sketch copy.
/// The editor persists font outlines, so regeneration needs no installed font.
pub fn expand_text(sketch: &Value) -> Result<Value, String> {
    let mut result = sketch.clone();
    let Some(geometries) = sketch["geometries"].as_array() else {
        return Ok(result);
    };
    if !geometries.iter().any(|g| g["type"] == "text") {
        return Ok(result);
    }
    let mut points = sketch["points"]
        .as_array()
        .cloned()
        .ok_or("text: missing points")?;
    let mut point_ids: std::collections::HashSet<String> =
        points.iter().map(|p| p["id"].to_string()).collect();
    let mut geometry_ids: std::collections::HashSet<String> =
        geometries.iter().map(|g| g["id"].to_string()).collect();
    let mut output = Vec::new();
    for geometry in geometries {
        if geometry["type"] != "text" {
            output.push(geometry.clone());
            continue;
        }
        let corners = geometry["points"]
            .as_array()
            .filter(|p| p.len() == 4)
            .ok_or("text: expected four corners")?;
        let corner = |i: usize| -> Result<[f64; 2], String> {
            let p = points
                .iter()
                .find(|p| p["id"] == corners[i])
                .ok_or("text: missing corner")?;
            Ok([
                p["x"].as_f64().ok_or("text: invalid corner")?,
                p["y"].as_f64().ok_or("text: invalid corner")?,
            ])
        };
        let a = corner(0)?;
        let b = corner(1)?;
        let d = corner(3)?;
        let contours = geometry["textContours"]
            .as_array()
            .ok_or("text: missing font outlines")?;
        for (ci, contour) in contours.iter().enumerate() {
            let coords: Vec<[f64; 2]> = serde_json::from_value(contour.clone())
                .map_err(|e| format!("text: invalid outline: {e}"))?;
            if coords.len() < 4
                || (coords.len() - 1) % 3 != 0
                || coords.first() != coords.last()
                || coords.iter().flatten().any(|x| !x.is_finite())
            {
                return Err("text: expected a closed cubic outline".into());
            }
            let gid = format!("text:{}:{ci}", geometry["id"]);
            let mut ids: Vec<Value> = Vec::new();
            for (i, [u, v]) in coords.iter().copied().enumerate() {
                if i == coords.len() - 1 {
                    ids.push(ids[0].clone());
                    break;
                }
                let id = json!(format!("{gid}:{i}"));
                if !point_ids.insert(id.to_string()) {
                    return Err("text: derived point id collision".into());
                }
                points.push(json!({"id":id,"x":a[0]+u*(b[0]-a[0])+v*(d[0]-a[0]),"y":a[1]+u*(b[1]-a[1])+v*(d[1]-a[1]),"fixed":true,"construction":true}));
                ids.push(id);
            }
            // Keep glyph corners as topological vertices, and avoid a single
            // high-span swept surface for an entire character contour.
            for (span, control) in coords.windows(4).step_by(3).enumerate() {
                let edge_id = format!("{gid}:{span}");
                if !geometry_ids.insert(json!(edge_id).to_string()) {
                    return Err("text: derived geometry id collision".into());
                }
                let d = [control[3][0] - control[0][0], control[3][1] - control[0][1]];
                let length2 = d[0] * d[0] + d[1] * d[1];
                let linear = length2 > 0.
                    && control[1..3].iter().all(|p| {
                        let q = [p[0] - control[0][0], p[1] - control[0][1]];
                        let projection = q[0] * d[0] + q[1] * d[1];
                        (q[0] * d[1] - q[1] * d[0]).abs() <= 1e-12 * length2
                            && projection >= 0.
                            && projection <= length2
                    });
                let start = span * 3;
                let (kind, handles) = if linear {
                    ("line", vec![ids[start].clone(), ids[start + 3].clone()])
                } else {
                    ("bezier", ids[start..start + 4].to_vec())
                };
                output.push(json!({"id":edge_id,"type":kind,"points":handles,"textOwner":geometry["id"],"construction":false}));
            }
        }
    }
    result["points"] = json!(points);
    result["geometries"] = json!(output);
    Ok(result)
}
