//! The TOOL dialog of the CAM pane: one schema-driven form over a tool of
//! the shared model (`brep_cam::model::Tool`) — its kind and geometry, its
//! holder, its coolant and its feeds and speeds — on the setup dialog's
//! pattern ([`crate::panels::cam_setup`]): a flat params object mapped both
//! ways, a patch applied by field name, every refusal naming its field.
//!
//! Validation is lane C12's `library::validate_tool`, so a tool the dialog
//! accepts is a tool the library, the feeds calculator and the cutter
//! profiles accept; its message is mapped back onto the field it names.

use brep_cam::feeds::{self, WorkMaterial};
use brep_cam::library::holders::{HolderDesign, HolderFamily};
use brep_cam::library::{validate_tool, LibraryUnits};
use brep_cam::model::{Coolant, Machine, MachineKind, SpindleDirection, Tool, ToolKind};
use brep_render::style::{FieldKind, FormField};
use serde_json::{json, Value};
use std::collections::HashSet;

/// The tool kinds, as the model's serde tags (its `ToolKind::as_str`).
pub const TOOL_KINDS: &[&str] = &[
    "flat", "ball", "bull", "chamfer", "v-bit", "tapered-ball", "tapered-flat", "lollipop", "face-mill", "slot", "t-slot",
    "dovetail", "engraver", "thread-mill", "drill", "spot-drill", "center-drill", "tap", "reamer", "boring-bar", "lathe-insert",
];

/// The work materials the feeds calculator knows, as serde tags.
pub const WORK_MATERIALS: &[&str] =
    &["aluminium", "mild-steel", "alloy-steel", "stainless-steel", "cast-iron", "brass", "titanium", "plastics", "wood", "mdf"];

/// The holder families the holder library designs, as serde tags; `none`
/// is the form's own word for "no holder".
pub const HOLDER_FAMILIES: &[&str] = &["none", "er16", "er25", "er32", "side-lock", "shrink-fit", "hydraulic"];

/// The coolant modes, as serde tags.
pub const COOLANTS: &[&str] = &["off", "flood", "mist", "through-tool", "air", "flood-and-through-tool", "flood-and-mist"];

fn field(key: &str, label: &str, group: &str, kind: FieldKind) -> FormField {
    FormField { path: vec![key.to_string()], label: label.to_string(), group: group.to_string(), kind }
}

fn scalar(key: &str, label: &str, group: &str) -> FormField {
    field(key, label, group, FieldKind::Scalar { step: 0.1 })
}

fn choice(key: &str, label: &str, group: &str, variants: &[&str]) -> FormField {
    field(key, label, group, FieldKind::Enum { variants: variants.iter().map(|s| s.to_string()).collect() })
}

/// The form's fields, in dialog order: Tool, Geometry, Holder, Feeds,
/// Coolant. [`hidden_fields`] hides the geometry a kind does not use and
/// the holder fields while there is no holder.
pub fn tool_form_fields() -> Vec<FormField> {
    vec![
        field("name", "Name", "Tool", FieldKind::Text { read_only: false }),
        field("number", "Tool number", "Tool", FieldKind::Number { min: 1.0, max: 999.0, step: 1.0 }),
        choice("kind", "Kind", "Tool", TOOL_KINDS),
        field("description", "Description", "Tool", FieldKind::Text { read_only: false }),
        field("material", "Tool material (carbide or HSS for Calculate feeds)", "Tool", FieldKind::Text { read_only: false }),
        scalar("diameter", "Diameter", "Geometry"),
        scalar("cuttingLength", "Cutting length", "Geometry"),
        scalar("shaftDiameter", "Shaft diameter (0 = the diameter)", "Geometry"),
        scalar("shaftLength", "Shaft length", "Geometry"),
        scalar("overallLength", "Overall length (0 = cutting + shaft)", "Geometry"),
        scalar("cornerRadius", "Corner radius", "Geometry"),
        scalar("includedAngleDeg", "Included angle (deg)", "Geometry"),
        scalar("ballDiameter", "Ball diameter", "Geometry"),
        scalar("tipDiameter", "Tip diameter", "Geometry"),
        scalar("threadPitch", "Thread pitch", "Geometry"),
        field("fluteCount", "Flutes", "Geometry", FieldKind::Number { min: 0.0, max: 64.0, step: 1.0 }),
        field("leftHand", "Left hand", "Geometry", FieldKind::Bool),
        choice("holderFamily", "Holder", "Holder", HOLDER_FAMILIES),
        scalar("holderBore", "Holder bore", "Holder"),
        scalar("holderHeight", "Holder height", "Holder"),
        scalar("spindleRpm", "Spindle speed (rpm)", "Feeds"),
        choice("spindleDirection", "Spindle direction", "Feeds", &["cw", "ccw"]),
        scalar("cuttingFeed", "Cutting feed", "Feeds"),
        scalar("plungeFeed", "Plunge feed", "Feeds"),
        choice("feedsMaterial", "Work material (for Calculate)", "Feeds", WORK_MATERIALS),
        field("calculateFeeds", "", "Feeds", FieldKind::Button { label: "Calculate feeds from the material".to_string() }),
        choice("coolant", "Coolant", "Coolant", COOLANTS),
    ]
}

/// The field keys the kind and holder in `params` do not use.
pub fn hidden_fields(params: &Value) -> HashSet<String> {
    let kind = params.get("kind").and_then(Value::as_str).unwrap_or("flat");
    let mut hidden: HashSet<String> = HashSet::new();
    let uses = |key: &str| -> bool {
        match key {
            "cornerRadius" => matches!(kind, "bull" | "tapered-flat"),
            "includedAngleDeg" => matches!(kind, "chamfer" | "v-bit" | "tapered-ball" | "tapered-flat" | "drill" | "spot-drill" | "center-drill" | "engraver" | "dovetail"),
            "ballDiameter" => matches!(kind, "lollipop" | "tapered-ball"),
            "tipDiameter" => matches!(kind, "chamfer" | "v-bit" | "engraver" | "tapered-flat"),
            "threadPitch" => matches!(kind, "tap" | "thread-mill"),
            _ => true,
        }
    };
    for key in ["cornerRadius", "includedAngleDeg", "ballDiameter", "tipDiameter", "threadPitch"] {
        if !uses(key) {
            hidden.insert(key.to_string());
        }
    }
    if params.get("holderFamily").and_then(Value::as_str).unwrap_or("none") == "none" {
        hidden.insert("holderBore".into());
        hidden.insert("holderHeight".into());
    }
    hidden
}

fn opt(v: Option<f64>) -> Value {
    json!(v.unwrap_or(0.0))
}

/// The holder family a stored holder was designed from, read back from its
/// name (`HolderDesign::build` names it after the family), or `none`.
fn holder_family_of(tool: &Tool) -> String {
    let Some(holder) = &tool.holder else { return "none".into() };
    let name = holder.name.to_ascii_lowercase();
    HOLDER_FAMILIES.iter().skip(1).find(|f| name.replace('_', "-").contains(*f)).map(|f| f.to_string()).unwrap_or_else(|| "side-lock".into())
}

/// The form's params for a tool.
pub fn tool_form_params(tool: &Tool) -> Value {
    let g = &tool.geometry;
    let holder = tool.holder.as_ref();
    json!({
        "name": tool.name,
        "number": tool.number,
        "kind": tool.kind.as_str(),
        "description": tool.description,
        "material": tool.material,
        "diameter": g.diameter,
        "cuttingLength": g.cutting_length,
        "shaftDiameter": opt(g.shaft_diameter),
        "shaftLength": g.shaft_length,
        "overallLength": opt(g.overall_length),
        "cornerRadius": opt(g.corner_radius),
        "includedAngleDeg": opt(g.included_angle_deg),
        "ballDiameter": opt(g.ball_diameter),
        "tipDiameter": opt(g.tip_diameter),
        "threadPitch": opt(g.thread_pitch),
        "fluteCount": g.flute_count.unwrap_or(0),
        "leftHand": g.left_hand,
        "holderFamily": holder_family_of(tool),
        "holderBore": holder.and_then(|h| h.segments.first()).map(|s| s.lower_diameter).unwrap_or(g.shaft_diameter()),
        "holderHeight": holder.map(|h| h.height()).unwrap_or(40.0),
        "spindleRpm": tool.feeds.spindle_rpm,
        "spindleDirection": match tool.feeds.spindle_direction { SpindleDirection::Cw => "cw", SpindleDirection::Ccw => "ccw" },
        "cuttingFeed": tool.feeds.cutting_feed,
        "plungeFeed": tool.feeds.plunge_feed,
        "feedsMaterial": "aluminium",
        "coolant": serde_json::to_value(tool.coolant).ok().and_then(|v| v.as_str().map(str::to_string)).unwrap_or_else(|| "off".into()),
    })
}

fn number(params: &Value, key: &str) -> Result<Option<f64>, String> {
    match params.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::Number(n)) => Ok(n.as_f64()),
        Some(Value::Bool(b)) => Ok(Some(if *b { 1.0 } else { 0.0 })),
        Some(Value::String(s)) if s.trim().is_empty() => Ok(None),
        Some(Value::String(s)) => s.trim().parse::<f64>().map(Some).map_err(|_| format!("{key}: `{s}` is not a number")),
        Some(other) => Err(format!("{key}: {other} is not a number")),
    }
}

/// A positive number or, at zero, "not set" — the form's reading of the
/// model's optional dimensions.
fn optional(params: &Value, key: &str) -> Result<Option<Option<f64>>, String> {
    Ok(number(params, key)?.map(|v| if v > 0.0 { Some(v) } else { None }))
}

fn tag<T: serde::de::DeserializeOwned>(params: &Value, key: &str, list: &[&str]) -> Result<Option<T>, String> {
    let Some(s) = params.get(key).and_then(Value::as_str) else { return Ok(None) };
    serde_json::from_value(Value::String(s.to_string())).map(Some).map_err(|_| format!("{key}: no `{s}` (one of: {})", list.join(", ")))
}

/// Map a `validate_tool` message onto the field it names, so the dialog
/// marks the field rather than only the banner.
fn field_of_message(message: &str) -> String {
    let m = message.to_ascii_lowercase();
    let key = if m.starts_with("tool id") {
        "name"
    } else if m.starts_with("tool number") {
        "number"
    } else if m.starts_with("diameter") {
        "diameter"
    } else if m.starts_with("cutting length") {
        "cuttingLength"
    } else if m.starts_with("shaft diameter") {
        "shaftDiameter"
    } else if m.starts_with("shaft length") {
        "shaftLength"
    } else if m.starts_with("overall length") {
        "overallLength"
    } else if m.contains("corner radius") {
        "cornerRadius"
    } else if m.contains("included angle") {
        "includedAngleDeg"
    } else if m.contains("ball") {
        "ballDiameter"
    } else if m.contains("tip") {
        "tipDiameter"
    } else if m.contains("thread") {
        "threadPitch"
    } else if m.contains("flute") {
        "fluteCount"
    } else if m.contains("holder") {
        "holderFamily"
    } else if m.contains("tool material") {
        "material"
    } else if m.contains("feed") || m.contains("spindle") || m.contains("surface speed") {
        "spindleRpm"
    } else {
        "kind"
    };
    format!("{key}: {message}")
}

/// Apply the form's params (or a PATCH: absent keys unchanged) to a tool.
/// Nothing is applied unless the result passes `validate_tool`; a refusal
/// names the field.
pub fn apply_tool_form(params: &Value, tool: &mut Tool) -> Result<(), String> {
    let mut next = tool.clone();
    if let Some(name) = params.get("name").and_then(Value::as_str) {
        if !name.trim().is_empty() {
            next.name = name.to_string();
        }
    }
    if let Some(n) = number(params, "number")? {
        if n < 1.0 || n.fract() != 0.0 {
            return Err(format!("number: {n} is not a tool number (a positive whole number)"));
        }
        next.number = n as u32;
    }
    if let Some(kind) = tag::<ToolKind>(params, "kind", TOOL_KINDS)? {
        next.kind = kind;
    }
    if let Some(s) = params.get("description").and_then(Value::as_str) {
        next.description = s.to_string();
    }
    if let Some(s) = params.get("material").and_then(Value::as_str) {
        next.material = s.to_string();
    }
    let g = &mut next.geometry;
    if let Some(v) = number(params, "diameter")? {
        g.diameter = v;
    }
    if let Some(v) = number(params, "cuttingLength")? {
        g.cutting_length = v;
    }
    if let Some(v) = optional(params, "shaftDiameter")? {
        g.shaft_diameter = v;
    }
    if let Some(v) = number(params, "shaftLength")? {
        g.shaft_length = v;
    }
    if let Some(v) = optional(params, "overallLength")? {
        g.overall_length = v;
    }
    if let Some(v) = optional(params, "cornerRadius")? {
        g.corner_radius = v;
    }
    if let Some(v) = optional(params, "includedAngleDeg")? {
        g.included_angle_deg = v;
    }
    if let Some(v) = optional(params, "ballDiameter")? {
        g.ball_diameter = v;
    }
    if let Some(v) = optional(params, "tipDiameter")? {
        g.tip_diameter = v;
    }
    if let Some(v) = optional(params, "threadPitch")? {
        g.thread_pitch = v;
    }
    if let Some(v) = number(params, "fluteCount")? {
        g.flute_count = if v > 0.0 { Some(v as u32) } else { None };
    }
    if let Some(b) = params.get("leftHand").and_then(Value::as_bool) {
        g.left_hand = b;
    }
    // The holder: designed from its family, bore and height through C12's
    // holder library; `none` removes it.
    let holder_keys = ["holderFamily", "holderBore", "holderHeight"];
    if holder_keys.iter().any(|k| params.get(k).is_some()) {
        let current = tool_form_params(tool);
        let get = |key: &str| params.get(key).or_else(|| current.get(key)).cloned().unwrap_or(Value::Null);
        let merged = json!({ "holderFamily": get("holderFamily"), "holderBore": get("holderBore"), "holderHeight": get("holderHeight") });
        let family = merged.get("holderFamily").and_then(Value::as_str).unwrap_or("none").to_string();
        if family == "none" {
            next.holder = None;
        } else {
            let family: HolderFamily = tag(&merged, "holderFamily", HOLDER_FAMILIES)?.ok_or_else(|| "holderFamily: missing".to_string())?;
            let bore = number(&merged, "holderBore")?.unwrap_or(next.geometry.shaft_diameter());
            let height = number(&merged, "holderHeight")?.unwrap_or(40.0);
            let design = HolderDesign::starting(family, bore, height, LibraryUnits::Millimeters).map_err(|e| format!("holderBore: {e}"))?;
            let holder = design
                .build(format!("{}-holder", next.id), format!("{} holder", merged["holderFamily"].as_str().unwrap_or("")))
                .map_err(|e| format!("holderHeight: {e}"))?;
            next.holder = Some(holder);
        }
    }
    if let Some(v) = number(params, "spindleRpm")? {
        next.feeds.spindle_rpm = v;
    }
    if let Some(d) = tag::<SpindleDirection>(params, "spindleDirection", &["cw", "ccw"])? {
        next.feeds.spindle_direction = d;
    }
    if let Some(v) = number(params, "cuttingFeed")? {
        next.feeds.cutting_feed = v;
    }
    if let Some(v) = number(params, "plungeFeed")? {
        next.feeds.plunge_feed = v;
    }
    if let Some(c) = tag::<Coolant>(params, "coolant", COOLANTS)? {
        next.coolant = c;
    }
    if let Some(value) = params.get("turningGeometry") {
        next.turning_geometry = serde_json::from_value(value.clone()).map_err(|e| format!("turningGeometry: {e}"))?;
    }
    if let Some(geometry) = &next.turning_geometry {
        brep_cam::turning::envelope(&next, geometry, 0.001).map_err(|e| format!("turningGeometry: {e}"))?;
    }
    validate_tool(&next).map_err(|m| field_of_message(&m))?;
    *tool = next;
    Ok(())
}

/// Feeds and speeds for `tool` cutting `material` on a default 3-axis
/// mill, through C12's calculator; written into the tool's feeds. Returns
/// the calculator's adjustments (what it clamped and why).
pub fn calculate_feeds(tool: &mut Tool, material: &str) -> Result<Vec<String>, String> {
    let material: WorkMaterial = serde_json::from_value(Value::String(material.to_string()))
        .map_err(|_| format!("feedsMaterial: no `{material}` (one of: {})", WORK_MATERIALS.join(", ")))?;
    let machine = Machine::new("default", "3-axis mill", MachineKind::default());
    let calculation = feeds::calculate(tool, &machine, LibraryUnits::Millimeters, material, None).map_err(|m| field_of_message(&m))?;
    tool.feeds = calculation.feeds;
    Ok(calculation.adjustments)
}

/// A copy of `tool` under `id` with tool number `number`, named
/// `<name> (copy)`: what Duplicate writes (a library refuses a duplicate id
/// OR number, so both are fresh).
pub fn duplicate(tool: &Tool, id: &str, number: u32) -> Tool {
    let mut copy = tool.clone();
    copy.id = id.into();
    copy.number = number;
    copy.name = format!("{} (copy)", tool.name);
    if let Some(holder) = &mut copy.holder {
        holder.id = format!("{id}-holder").into();
    }
    copy
}

