//! Read-only selection measurements for the status bar and its MCP state blob.
//! Engine object-info queries are cached and deduplicated while in flight; the
//! engine invalidates that cache on geometry edits, including undo/redo. Never
//! call kernel integration directly from the frame loop.
use brep_render::engine_state::EngineState;
use serde_json::{json, Value};
use super::info_windows::num;

#[derive(Default)]
pub struct Cache {
    key: Option<(u64, u64, u64, String)>,
    value: Value,
}

impl Cache {
    pub fn read(&mut self, state: &mut EngineState, document: u64) -> Value {
        if state.any_sketch_mode() || state.ref_select_active()
            || crate::workbench::ecad::Target::of_workbench(&state.settings.workbench).is_some()
        {
            return json!({"shown":false,"text":"","pending":false,"lengthUnit":"mm",
                "areaUnit":"mm²","totals":{},"items":[]});
        }
        let selection = json!({"selection":state.selection_value(),
            "vertices":state.emphasis.selected_vertices.iter().map(|v| json!([v.solid, v.position])).collect::<Vec<_>>()}).to_string();
        let key = (document, state.applied_generation(), state.scene.exact_revision(), selection);
        if self.key.as_ref() != Some(&key) || self.value["pending"] == true {
            self.value = readout(state);
            self.key = Some(key);
        }
        self.value.clone()
    }
}

pub fn readout(state: &mut EngineState) -> Value {
    let solids: Vec<_> = state.emphasis.selected_solids.iter().cloned().collect();
    let faces: Vec<_> = state.emphasis.selected_faces.iter().cloned().collect();
    let edges: Vec<_> = state.emphasis.selected_edges.iter().cloned().collect();
    let vertices = state.emphasis.selected_vertices.clone();
    let mut parts = Vec::new();
    let mut items = Vec::new();
    let mut totals = serde_json::Map::new();
    let mut pending = false;
    let mut unavailable = false;
    // The document's current length convention is millimetres (the same as
    // Properties/Info); no independent unit preference exists in the document.
    if !solids.is_empty() {
        let (mut area, mut volume, mut any_volume, mut complete) = (0.0, 0.0, false, true);
        for name in &solids {
            let info: Value = serde_json::from_str(&state.object_info_json(name)).unwrap_or(Value::Null);
            let waiting = info["pending"].as_bool().unwrap_or(false);
            pending |= waiting;
            // A committed sketch picks as a solid but is a sheet: it reports
            // `area` and has no volume, which is not a failed measurement.
            let sheet = info["kind"] == "sketch";
            let measured_area = info["surfaceArea"].as_f64().or(info["area"].as_f64()).filter(|v| v.is_finite());
            let measured_volume = info["volume"].as_f64().filter(|v| v.is_finite());
            match (measured_area, measured_volume) {
                (Some(a), Some(v)) => { area += a; volume += v; any_volume = true; }
                (Some(a), None) if sheet => area += a,
                _ => { complete = false; unavailable |= !waiting; }
            }
            items.push(info);
        }
        if complete {
            let prefix = if solids.len() > 1 { "Total " } else { "" };
            totals.insert("surfaceArea".into(), json!(area));
            parts.push(format!("{prefix}Surface Area: {} mm²", num(area)));
            if any_volume {
                totals.insert("volume".into(), json!(volume));
                parts.push(format!("{prefix}Volume: {} mm³", num(volume)));
            }
        }
    }
    for (names, key, label, unit) in [
        (&faces, "area", "Area", "mm²"),
        (&edges, "length", "Length", "mm"),
    ] {
        if names.is_empty() { continue; }
        let mut total = 0.0;
        let mut complete = true;
        let mut radius = None;
        for name in names {
            let info: Value = serde_json::from_str(&state.object_info_json(name)).unwrap_or(Value::Null);
            pending |= info["pending"].as_bool().unwrap_or(false);
            if let Some(value) = info[key].as_f64().filter(|v| v.is_finite()) {
                total += value;
            } else {
                complete = false;
                unavailable |= !info["pending"].as_bool().unwrap_or(false);
            }
            radius = info["radius"].as_f64();
            items.push(info);
        }
        if complete {
            totals.insert(key.into(), json!(total));
            let prefix = if names.len() > 1 { "Total " } else { "" };
            parts.push(format!("{prefix}{label}: {} {unit}", num(total)));
            if names.len() == 1 {
                if let Some(r) = radius {
                    parts.push(format!("Radius: {} mm", num(r)));
                    if key == "length" { parts.push(format!("Diameter: {} mm", num(2.0 * r))); }
                }
            }
        }
    }
    for vertex in &vertices {
        let [x, y, z] = vertex.position;
        parts.push(format!("Vertex: ({}, {}, {}) mm", num(x), num(y), num(z)));
        items.push(json!({"kind":"vertex", "solid":vertex.solid, "position":vertex.position}));
    }
    if pending { parts.push("Measuring…".into()); }
    if unavailable { parts.push("Measurement unavailable".into()); }
    json!({"shown":!parts.is_empty(), "text":parts.join(" · "), "pending":pending,
        "lengthUnit":"mm", "areaUnit":"mm²", "totals":totals, "items":items})
}

