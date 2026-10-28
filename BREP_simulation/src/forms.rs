//! The Simulation pane's forms: a motor's and a clip's fields as the app's
//! schema-driven form engine reads them (an `inputParamsSchema` of `number`,
//! `options`, `boolean` and `reference_selection` fields), a FLAT view of the
//! item for the form to edit (the engine binds top-level keys), which fields
//! hide for the item's current mode, and the way back — the patch
//! `simulation_patch` merges into the item. Pure functions: the pane draws,
//! this decides what it draws and what an edit means.

use crate::model::{Clip, ClipBody, Drive, Motor, Shape, DEFAULT_ACCEL};
use serde_json::{json, Map, Value};

/// A mate a motor can drive, as the form offers it.
#[derive(Debug, Clone, PartialEq)]
pub struct MateChoice {
    /// The constraint id (`DIST2`).
    pub id: String,
    /// `distance` or `angle`.
    pub kind: String,
    /// Its document value, what a profile starts from.
    pub value: f64,
}

/// What the forms' option lists are built from: the assembly's mates, every
/// constraint (a suppress clip's), the PMI views with a camera, the PMI views
/// with a section, the exploded views.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FormChoices {
    pub mates: Vec<MateChoice>,
    pub constraints: Vec<String>,
    pub camera_views: Vec<String>,
    pub section_views: Vec<String>,
    pub exploded_views: Vec<String>,
}

/// A camera clip offers this many view slots (`view1`…), in play order.
pub const CAMERA_VIEW_SLOTS: usize = 4;
/// The option an empty camera-view slot shows.
pub const NO_VIEW: &str = "(none)";

/// The field key a parts list binds to (the reference field a pick writes).
pub const OCCURRENCES: &str = "occurrences";

fn options(ids: impl IntoIterator<Item = String>, current: Option<&str>) -> Vec<String> {
    let mut list: Vec<String> = ids.into_iter().collect();
    if let Some(current) = current.filter(|c| !c.is_empty()) {
        if !list.iter().any(|id| id == current) {
            // A reference the document no longer has still shows as what it is.
            list.push(current.to_string());
        }
    }
    list
}

fn number(params: &Value, key: &str) -> Result<f64, String> {
    match params.get(key) {
        Some(Value::Number(n)) => n.as_f64().ok_or_else(|| format!("{key}: a number")),
        Some(Value::String(text)) => text.trim().parse::<f64>().map_err(|_| format!("{key}: a number, not '{text}'")),
        None | Some(Value::Null) => Err(format!("{key}: a number")),
        Some(other) => Err(format!("{key}: a number, not {other}")),
    }
}

fn text<'a>(params: &'a Value, key: &str) -> &'a str {
    params.get(key).and_then(Value::as_str).unwrap_or("")
}

// ---------------------------------------------------------------------------
// Motors
// ---------------------------------------------------------------------------

/// The motor form's schema: the mate it drives, its span, the drive mode and
/// the mode's own fields (speed, or a profile's shape, start, end and the
/// trapezoid's acceleration share).
pub fn motor_schema(motor: &Motor, choices: &FormChoices) -> Value {
    let unit = choices.mates.iter().find(|m| m.id == motor.constraint).map_or("mm", |m| if m.kind == "angle" { "°" } else { "mm" });
    json!({
        "type": "simulationMotor",
        "shortName": "MOTOR",
        "longName": "Motor",
        "inputParamsSchema": {
            "constraint": { "type": "options", "label": "Mate", "options": options(choices.mates.iter().map(|m| m.id.clone()), Some(&motor.constraint)),
                "hint": "The distance or angle mate the motor drives" },
            "from": { "type": "number", "label": "From (s)", "default_value": 0.0 },
            "to": { "type": "number", "label": "To (s)", "default_value": 1.0, "hint": "The motor runs inside its span and holds its last value after" },
            "mode": { "type": "options", "label": "Drive", "options": ["speed", "profile"], "default_value": "speed" },
            "speed": { "type": "number", "label": format!("Speed ({unit}/s)"), "default_value": 10.0, "hint": "From the mate's own value, at a constant rate; negative runs backwards" },
            "shape": { "type": "options", "label": "Profile", "options": ["linear", "ease", "trapezoid"], "default_value": "linear" },
            "start": { "type": "number", "label": format!("Start ({unit})"), "default_value": 0.0 },
            "end": { "type": "number", "label": format!("End ({unit})"), "default_value": 20.0 },
            "accel": { "type": "number", "label": "Acceleration share", "default_value": DEFAULT_ACCEL, "hint": "Trapezoid: the share of the span spent accelerating, and again decelerating (0–0.5)" }
        }
    })
}

/// The motor as the form edits it: its fields flat, the drive's under
/// `mode`, `speed`, `shape`, `start`, `end`, `accel`.
pub fn motor_params(motor: &Motor) -> Value {
    let mut params = json!({ "constraint": motor.constraint, "from": motor.from, "to": motor.to });
    match &motor.drive {
        Drive::Speed { speed } => {
            params["mode"] = json!("speed");
            params["speed"] = json!(speed);
        }
        Drive::Profile { shape, start, end, accel } => {
            params["mode"] = json!("profile");
            params["shape"] = json!(shape_key(*shape));
            params["start"] = json!(start);
            params["end"] = json!(end);
            if let Some(accel) = accel {
                params["accel"] = json!(accel);
            }
        }
    }
    params
}

/// The motor fields the current mode has no use for.
pub fn motor_hidden(params: &Value) -> Vec<String> {
    let mut hidden = Vec::new();
    if text(params, "mode") == "profile" {
        hidden.push("speed".to_string());
        if text(params, "shape") != "trapezoid" {
            hidden.push("accel".to_string());
        }
    } else {
        hidden.extend(["shape", "start", "end", "accel"].map(str::to_string));
    }
    hidden
}

/// The patch an edited motor form means: `constraint`, `from`, `to` and the
/// `drive` rebuilt from the mode. A mode just switched has no fields of its
/// own yet, so they are seeded as the pane always did: a speed of 10 mm/s or
/// 90°/s, a profile from the mate's value 20 mm or 360° on, a trapezoid's
/// default acceleration share.
pub fn motor_patch(params: &Value, choices: &FormChoices) -> Result<Value, String> {
    let constraint = text(params, "constraint").to_string();
    let mate = choices.mates.iter().find(|m| m.id == constraint);
    let angle = mate.is_some_and(|m| m.kind == "angle");
    let drive = match text(params, "mode") {
        "profile" => {
            let shape = match text(params, "shape") {
                "" | "linear" => Shape::Linear,
                "ease" => Shape::Ease,
                "trapezoid" => Shape::Trapezoid,
                other => return Err(format!("shape: linear, ease or trapezoid, not '{other}'")),
            };
            let base = mate.map_or(0.0, |m| m.value);
            let start = number(params, "start").unwrap_or(base);
            let end = number(params, "end").unwrap_or(base + if angle { 360.0 } else { 20.0 });
            let accel = match shape {
                Shape::Trapezoid => Some(number(params, "accel").unwrap_or(DEFAULT_ACCEL)),
                _ => None,
            };
            let mut drive = json!({ "mode": "profile", "shape": shape_key(shape), "start": start, "end": end });
            if let Some(accel) = accel {
                drive["accel"] = json!(accel);
            }
            drive
        }
        "" | "speed" => json!({ "mode": "speed", "speed": number(params, "speed").unwrap_or(if angle { 90.0 } else { 10.0 }) }),
        other => return Err(format!("mode: speed or profile, not '{other}'")),
    };
    Ok(json!({ "constraint": constraint, "from": number(params, "from")?, "to": number(params, "to")?, "drive": drive }))
}

fn shape_key(shape: Shape) -> &'static str {
    match shape {
        Shape::Linear => "linear",
        Shape::Ease => "ease",
        Shape::Trapezoid => "trapezoid",
    }
}

// ---------------------------------------------------------------------------
// Clips
// ---------------------------------------------------------------------------

/// The clip kind's word in the schema (`suppress`, `camera`, …), the same
/// `kind` the block stores.
pub fn clip_kind(clip: &Clip) -> &'static str {
    match clip.body {
        ClipBody::Suppress { .. } => "suppress",
        ClipBody::Camera { .. } => "camera",
        ClipBody::Visibility { .. } => "visibility",
        ClipBody::Transparency { .. } => "transparency",
        ClipBody::Section { .. } => "section",
        ClipBody::Explode { .. } => "explode",
    }
}

/// The clip form's title word.
pub fn clip_title(clip: &Clip) -> &'static str {
    match clip.body {
        ClipBody::Suppress { .. } => "Suppress a mate",
        ClipBody::Camera { .. } => "Camera path",
        ClipBody::Visibility { .. } => "Hide parts",
        ClipBody::Transparency { .. } => "Fade parts",
        ClipBody::Section { .. } => "Section",
        ClipBody::Explode { .. } => "Exploded view",
    }
}

/// The clip form's schema: the span, then the kind's own fields — the mate
/// (suppress), the parts as a reference field picked in the viewport
/// (visibility, transparency: solids or components), the fade's end
/// opacities, the view slots (camera), the section view and its sweep, the
/// exploded view and Collapse.
pub fn clip_schema(clip: &Clip, choices: &FormChoices) -> Value {
    let mut fields = Map::new();
    fields.insert("from".into(), json!({ "type": "number", "label": "From (s)", "default_value": 0.0 }));
    fields.insert("to".into(), json!({ "type": "number", "label": "To (s)", "default_value": 1.0 }));
    let parts = json!({
        "type": "reference_selection", "label": "Parts", "selectionFilter": ["SOLID", "COMPONENT"], "multiple": true,
        "minSelections": 0, "maxSelections": 256, "default_value": [],
        "hint": "Click parts in the view; a solid stands for the component that holds it"
    });
    match &clip.body {
        ClipBody::Suppress { constraint } => {
            fields.insert("constraint".into(), json!({ "type": "options", "label": "Mate", "options": options(choices.constraints.iter().cloned(), Some(constraint)), "hint": "Off (suppressed) inside the span" }));
        }
        ClipBody::Camera { views } => {
            for slot in 0..CAMERA_VIEW_SLOTS {
                let mut list = vec![NO_VIEW.to_string()];
                list.extend(options(choices.camera_views.iter().cloned(), views.get(slot).map(String::as_str)));
                fields.insert(format!("view{}", slot + 1), json!({ "type": "options", "label": format!("View {}", slot + 1), "options": list, "default_value": NO_VIEW,
                    "hint": "The camera moves through the views in this order, evenly over the span" }));
            }
        }
        ClipBody::Visibility { .. } => {
            fields.insert(OCCURRENCES.into(), parts);
        }
        ClipBody::Transparency { .. } => {
            fields.insert(OCCURRENCES.into(), parts);
            fields.insert("opacityFrom".into(), json!({ "type": "number", "label": "Opacity at start", "default_value": 1.0, "hint": "1 opaque, 0 invisible" }));
            fields.insert("opacityTo".into(), json!({ "type": "number", "label": "Opacity at end", "default_value": 0.2 }));
        }
        ClipBody::Section { view, .. } => {
            fields.insert("view".into(), json!({ "type": "options", "label": "Section view", "options": options(choices.section_views.iter().cloned(), Some(view)), "hint": "A PMI view with a section: its cut is on inside the span" }));
            fields.insert("sweepEnabled".into(), json!({ "type": "boolean", "label": "Sweep the planes", "default_value": false }));
            fields.insert("sweepFrom".into(), json!({ "type": "number", "label": "Sweep from (mm)", "default_value": 0.0, "hint": "Along the planes' normals, over the span" }));
            fields.insert("sweepTo".into(), json!({ "type": "number", "label": "Sweep to (mm)", "default_value": 0.0 }));
        }
        ClipBody::Explode { view, .. } => {
            fields.insert("view".into(), json!({ "type": "options", "label": "Exploded view", "options": options(choices.exploded_views.iter().cloned(), Some(view)) }));
            fields.insert("collapse".into(), json!({ "type": "boolean", "label": "Collapse", "default_value": false, "hint": "Play the steps backwards: the view comes together" }));
        }
    }
    json!({ "type": format!("simulationClip:{}", clip_kind(clip)), "shortName": "CLIP", "longName": clip_title(clip), "inputParamsSchema": Value::Object(fields) })
}

/// The clip as the form edits it, flat: the span and the kind's fields.
pub fn clip_params(clip: &Clip) -> Value {
    let mut params = json!({ "from": clip.from, "to": clip.to });
    match &clip.body {
        ClipBody::Suppress { constraint } => params["constraint"] = json!(constraint),
        ClipBody::Camera { views } => {
            for slot in 0..CAMERA_VIEW_SLOTS {
                params[format!("view{}", slot + 1)] = json!(views.get(slot).cloned().unwrap_or_else(|| NO_VIEW.to_string()));
            }
        }
        ClipBody::Visibility { occurrences } => params[OCCURRENCES] = json!(occurrences),
        ClipBody::Transparency { occurrences, opacity } => {
            params[OCCURRENCES] = json!(occurrences);
            params["opacityFrom"] = json!(opacity[0]);
            params["opacityTo"] = json!(opacity[1]);
        }
        ClipBody::Section { view, sweep } => {
            params["view"] = json!(view);
            params["sweepEnabled"] = json!(sweep.is_some());
            let sweep = sweep.unwrap_or([0.0, 0.0]);
            params["sweepFrom"] = json!(sweep[0]);
            params["sweepTo"] = json!(sweep[1]);
        }
        ClipBody::Explode { view, collapse } => {
            params["view"] = json!(view);
            params["collapse"] = json!(collapse);
        }
    }
    params
}

/// The clip fields the current state has no use for (a sweep that is off).
pub fn clip_hidden(clip: &Clip, params: &Value) -> Vec<String> {
    match clip.body {
        ClipBody::Section { .. } if params.get("sweepEnabled") != Some(&Value::Bool(true)) => vec!["sweepFrom".into(), "sweepTo".into()],
        _ => Vec::new(),
    }
}

/// The patch an edited clip form means for `clip`'s kind.
pub fn clip_patch(clip: &Clip, params: &Value) -> Result<Value, String> {
    let mut patch = json!({ "from": number(params, "from")?, "to": number(params, "to")? });
    match &clip.body {
        ClipBody::Suppress { .. } => patch["constraint"] = json!(text(params, "constraint")),
        ClipBody::Camera { .. } => {
            let views: Vec<String> = (0..CAMERA_VIEW_SLOTS)
                .map(|slot| text(params, &format!("view{}", slot + 1)).to_string())
                .filter(|view| !view.is_empty() && view != NO_VIEW)
                .collect();
            patch["views"] = json!(views);
        }
        ClipBody::Visibility { .. } => patch[OCCURRENCES] = occurrences(params),
        ClipBody::Transparency { .. } => {
            patch[OCCURRENCES] = occurrences(params);
            patch["opacity"] = json!([number(params, "opacityFrom")?, number(params, "opacityTo")?]);
        }
        ClipBody::Section { .. } => {
            patch["view"] = json!(text(params, "view"));
            patch["sweep"] = if params.get("sweepEnabled") == Some(&Value::Bool(true)) {
                json!([number(params, "sweepFrom")?, number(params, "sweepTo")?])
            } else {
                Value::Null
            };
        }
        ClipBody::Explode { .. } => {
            patch["view"] = json!(text(params, "view"));
            patch["collapse"] = json!(params.get("collapse") == Some(&Value::Bool(true)));
        }
    }
    Ok(patch)
}

fn occurrences(params: &Value) -> Value {
    let list: Vec<String> = params
        .get(OCCURRENCES)
        .and_then(Value::as_array)
        .map(|items| items.iter().filter_map(Value::as_str).map(str::to_string).filter(|s| !s.is_empty()).collect())
        .unwrap_or_default();
    json!(list)
}

