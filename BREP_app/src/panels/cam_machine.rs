//! The CAM machine dialog: one form over a `brep_cam::model::Machine` — its
//! kind (a change starts the kind's conventional axis layout), the post it is
//! posted with, spindle and feed limits, optional travel limits, every rotary
//! axis of the kind, a mill-turn's Y axis, and the collision solids' count
//! (those are lane C10/C17's to edit). Every value is read back as the
//! model's type and checked by `Machine::validate`; a refusal names its field
//! with nothing applied.

use brep_cam::model::{Kinematics, Machine, MachineKind, Point3, RotaryAxis, TravelLimits};

/// The machine kinds a machine can be switched to, in the model's order.
pub const MACHINE_KINDS: &[&str] = &["mill3", "mill4", "mill5", "lathe", "millTurn"];

/// A rotary axis named `name` about `direction` through the origin.
fn rotary(name: &str, direction: Point3) -> RotaryAxis {
    RotaryAxis { name: name.to_string(), direction, origin: Point3::default(), limits_deg: None }
}

/// The machine kind `kind` with a conventional layout, for a kind CHANGE in the
/// editor (a machine starts as a copy of a built-in): a 4-axis mill's A turns
/// about X; a 5-axis mill is table-table, A about X carrying C about Z; a
/// mill-turn's C turns about +Z (the model requires it), without a Y axis.
pub fn machine_kind_preset(kind: &str) -> Result<MachineKind, String> {
    Ok(match kind.trim() {
        "mill3" => MachineKind::Mill3,
        "mill4" => MachineKind::Mill4 { rotary: rotary("A", Point3::new(1.0, 0.0, 0.0)) },
        "mill5" => MachineKind::Mill5 { kinematics: Kinematics::TableTable { primary: rotary("A", Point3::new(1.0, 0.0, 0.0)), secondary: rotary("C", Point3::new(0.0, 0.0, 1.0)) } },
        "lathe" => MachineKind::Lathe,
        "millTurn" => MachineKind::MillTurn { c_axis: rotary("C", Point3::new(0.0, 0.0, 1.0)), y_axis: false },
        other => return Err(format!("kind: no machine kind `{other}` (one of: {})", MACHINE_KINDS.join(", "))),
    })
}
use brep_render::style::{FieldKind, FormField};
use serde_json::{json, Value};
use std::collections::HashSet;

/// The five-axis layouts, in the model's order (`Kinematics`' serde tags).
pub const KINEMATICS: &[&str] = &["tableTable", "headHead", "headTable"];

fn field(path: &[&str], label: &str, group: &str, kind: FieldKind) -> FormField {
    FormField { path: path.iter().map(|s| s.to_string()).collect(), label: label.to_string(), group: group.to_string(), kind }
}

fn scalar(path: &[&str], label: &str, group: &str) -> FormField {
    field(path, label, group, FieldKind::Scalar { step: 10.0 })
}

/// The kind's serde tag (`mill3`, `mill5`, ...).
pub fn kind_tag(kind: &MachineKind) -> String {
    serde_json::to_value(kind).ok().and_then(|v| v.get("kind").and_then(Value::as_str).map(str::to_string)).unwrap_or_default()
}

/// The kind's rotary axes in dialog order, with their labels.
fn axes(kind: &MachineKind) -> Vec<(&'static str, &RotaryAxis)> {
    match kind {
        MachineKind::Mill3 | MachineKind::Lathe => Vec::new(),
        MachineKind::Mill4 { rotary } => vec![("Rotary axis", rotary)],
        MachineKind::MillTurn { c_axis, .. } => vec![("C axis", c_axis)],
        MachineKind::Mill5 { kinematics } => match kinematics {
            Kinematics::TableTable { primary, secondary } | Kinematics::HeadHead { primary, secondary } => {
                vec![("Primary axis", primary), ("Secondary axis", secondary)]
            }
            Kinematics::HeadTable { head, table } => vec![("Head axis", head), ("Table axis", table)],
        },
        #[allow(unreachable_patterns)]
        _ => Vec::new(),
    }
}

fn axes_mut(kind: &mut MachineKind) -> Vec<&mut RotaryAxis> {
    match kind {
        MachineKind::Mill3 | MachineKind::Lathe => Vec::new(),
        MachineKind::Mill4 { rotary } => vec![rotary],
        MachineKind::MillTurn { c_axis, .. } => vec![c_axis],
        MachineKind::Mill5 { kinematics } => match kinematics {
            Kinematics::TableTable { primary, secondary } | Kinematics::HeadHead { primary, secondary } => vec![primary, secondary],
            Kinematics::HeadTable { head, table } => vec![head, table],
        },
        #[allow(unreachable_patterns)]
        _ => Vec::new(),
    }
}

/// The post choices: none, then every built-in post.
pub fn post_choices() -> Vec<String> {
    std::iter::once(String::new()).chain(brep_cam::post::BUILTIN_POSTS.iter().map(|p| p.id.to_string())).collect()
}

/// The dialog's fields for `machine`, in dialog order.
pub fn machine_form_fields(machine: &Machine) -> Vec<FormField> {
    let mut fields = vec![
        field(&["name"], "Name", "Machine", FieldKind::Text { read_only: false }),
        field(&["kind"], "Kind (a change starts its axis layout)", "Machine", FieldKind::Enum { variants: MACHINE_KINDS.iter().map(|s| s.to_string()).collect() }),
        field(&["post"], "Post processor", "Machine", FieldKind::Enum { variants: post_choices() }),
        field(&["toolChangeSeconds"], "Tool change (s)", "Machine", FieldKind::Scalar { step: 0.5 }),
        scalar(&["spindle", "minRpm"], "Spindle minimum (rpm)", "Limits"),
        scalar(&["spindle", "maxRpm"], "Spindle maximum (rpm)", "Limits"),
        scalar(&["feed", "maxCutting"], "Maximum cutting feed", "Limits"),
        scalar(&["feed", "rapid"], "Rapid feed", "Limits"),
        field(&["accelerationSet"], "Limit acceleration", "Limits", FieldKind::Bool),
        scalar(&["feed", "acceleration"], "Acceleration", "Limits"),
        field(&["travelSet"], "Limit travel", "Travel", FieldKind::Bool),
        field(&["travel", "min"], "Travel minimum", "Travel", FieldKind::Vec3 { step: 10.0 }),
        field(&["travel", "max"], "Travel maximum", "Travel", FieldKind::Vec3 { step: 10.0 }),
    ];
    if matches!(machine.kind, MachineKind::Mill5 { .. }) {
        fields.push(field(&["kinematics"], "Layout", "Axes", FieldKind::Enum { variants: KINEMATICS.iter().map(|s| s.to_string()).collect() }));
    }
    if matches!(machine.kind, MachineKind::MillTurn { .. }) {
        fields.push(field(&["yAxis"], "Y axis", "Axes", FieldKind::Bool));
    }
    for (i, (label, _)) in axes(&machine.kind).into_iter().enumerate() {
        let i = i.to_string();
        let group = format!("Axes \u{00b7} {label}");
        fields.push(field(&["axes", &i, "name"], "Name", &group, FieldKind::Text { read_only: false }));
        fields.push(field(&["axes", &i, "direction"], "Direction", &group, FieldKind::Vec3 { step: 0.1 }));
        fields.push(field(&["axes", &i, "origin"], "Origin", &group, FieldKind::Vec3 { step: 1.0 }));
        fields.push(field(&["axes", &i, "limitsSet"], "Limit the angle", &group, FieldKind::Bool));
        fields.push(field(&["axes", &i, "limitsMin"], "Minimum angle (deg)", &group, FieldKind::Scalar { step: 5.0 }));
        fields.push(field(&["axes", &i, "limitsMax"], "Maximum angle (deg)", &group, FieldKind::Scalar { step: 5.0 }));
    }
    fields.push(field(&["collision"], "Collision solids", "Collision", FieldKind::Text { read_only: true }));
    fields
}

/// The keys the switches hide: acceleration without its limit, travel
/// without its limits.
pub fn hidden_fields(params: &Value) -> HashSet<String> {
    let mut hidden = HashSet::new();
    if !params.get("accelerationSet").and_then(Value::as_bool).unwrap_or(false) {
        hidden.insert("acceleration".to_string());
    }
    if !params.get("travelSet").and_then(Value::as_bool).unwrap_or(false) {
        hidden.insert("min".to_string());
        hidden.insert("max".to_string());
    }
    hidden
}

fn xyz(p: Point3) -> Value {
    json!([p.x, p.y, p.z])
}

/// The dialog's params for `machine`.
pub fn machine_form_params(machine: &Machine) -> Value {
    let travel = machine.travel.clone().unwrap_or(TravelLimits { min: Point3::default(), max: Point3::default() });
    let mut axes_map = serde_json::Map::new();
    for (i, (_, axis)) in axes(&machine.kind).into_iter().enumerate() {
        let [lo, hi] = axis.limits_deg.unwrap_or([-360.0, 360.0]);
        axes_map.insert(
            i.to_string(),
            json!({
                "name": axis.name,
                "direction": xyz(axis.direction),
                "origin": xyz(axis.origin),
                "limitsSet": axis.limits_deg.is_some(),
                "limitsMin": lo,
                "limitsMax": hi,
            }),
        );
    }
    let mut params = json!({
        "name": machine.name,
        "kind": kind_tag(&machine.kind),
        "post": machine.post.clone().unwrap_or_default(),
        "toolChangeSeconds": machine.tool_change_seconds,
        "spindle": { "minRpm": machine.spindle.min_rpm, "maxRpm": machine.spindle.max_rpm },
        "feed": { "maxCutting": machine.feed.max_cutting, "rapid": machine.feed.rapid, "acceleration": machine.feed.acceleration.unwrap_or(1000.0) },
        "accelerationSet": machine.feed.acceleration.is_some(),
        "travelSet": machine.travel.is_some(),
        "travel": { "min": xyz(travel.min), "max": xyz(travel.max) },
        "axes": Value::Object(axes_map),
        "collision": format!("{} solid{}", machine.collision.solids.len(), if machine.collision.solids.len() == 1 { "" } else { "s" }),
    });
    if let MachineKind::Mill5 { kinematics } = &machine.kind {
        params["kinematics"] = serde_json::to_value(kinematics).ok().and_then(|v| v.get("kind").cloned()).unwrap_or(Value::Null);
    }
    if let MachineKind::MillTurn { y_axis, .. } = &machine.kind {
        params["yAxis"] = json!(y_axis);
    }
    params
}

fn value_at<'a>(params: &'a Value, path: &[&str]) -> Option<&'a Value> {
    path.iter().try_fold(params, |cur, seg| cur.get(*seg))
}

fn number(params: &Value, path: &[&str]) -> Result<Option<f64>, String> {
    let key = path.join(".");
    match value_at(params, path) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::Number(n)) => Ok(n.as_f64()),
        Some(Value::String(s)) if s.trim().is_empty() => Ok(None),
        Some(Value::String(s)) => s.trim().parse::<f64>().map(Some).map_err(|_| format!("{key}: `{s}` is not a number")),
        Some(other) => Err(format!("{key}: {other} is not a number")),
    }
}

fn point(params: &Value, path: &[&str]) -> Result<Option<Point3>, String> {
    let key = path.join(".");
    let Some(raw) = value_at(params, path) else { return Ok(None) };
    let xyz: Option<Vec<f64>> = raw.as_array().map(|l| l.iter().filter_map(|c| c.as_f64().or_else(|| c.as_str().and_then(|s| s.trim().parse().ok()))).collect());
    match xyz {
        Some(v) if v.len() == 3 => Ok(Some(Point3::new(v[0], v[1], v[2]))),
        _ => Err(format!("{key}: three numbers [x, y, z]")),
    }
}

fn flag(params: &Value, key: &str) -> Option<bool> {
    params.get(key).and_then(Value::as_bool)
}

/// Apply the dialog's `params` (or a patch of them) to `machine`: nothing is
/// applied when any of it is refused; the refusal names the field.
pub fn apply_machine_form(params: &Value, machine: &mut Machine) -> Result<(), String> {
    let mut next = machine.clone();
    if let Some(name) = params.get("name").and_then(Value::as_str) {
        if !name.trim().is_empty() {
            next.name = name.to_string();
        }
    }
    // A new kind starts its conventional layout; this patch's axis values
    // describe the old one and are not applied over it.
    let mut kind_changed = false;
    if let Some(kind) = params.get("kind").and_then(Value::as_str) {
        if kind != kind_tag(&next.kind) {
            next.kind = machine_kind_preset(kind)?;
            kind_changed = true;
        }
    }
    if let Some(post) = params.get("post").and_then(Value::as_str) {
        next.post = match post.trim() {
            "" => None,
            id if brep_cam::post::BUILTIN_POSTS.iter().any(|p| p.id == id) => Some(id.to_string()),
            id => return Err(format!("post: no post `{id}` (one of: {})", post_choices()[1..].join(", "))),
        };
    }
    if let Some(v) = number(params, &["toolChangeSeconds"])? {
        next.tool_change_seconds = v;
    }
    if let Some(v) = number(params, &["spindle", "minRpm"])? {
        next.spindle.min_rpm = v;
    }
    if let Some(v) = number(params, &["spindle", "maxRpm"])? {
        next.spindle.max_rpm = v;
    }
    if let Some(v) = number(params, &["feed", "maxCutting"])? {
        next.feed.max_cutting = v;
    }
    if let Some(v) = number(params, &["feed", "rapid"])? {
        next.feed.rapid = v;
    }
    let acceleration_set = flag(params, "accelerationSet").unwrap_or(next.feed.acceleration.is_some());
    next.feed.acceleration = if acceleration_set { Some(number(params, &["feed", "acceleration"])?.or(next.feed.acceleration).unwrap_or(1000.0)) } else { None };
    let travel_set = flag(params, "travelSet").unwrap_or(next.travel.is_some());
    next.travel = if travel_set {
        let current = next.travel.clone().unwrap_or(TravelLimits { min: Point3::default(), max: Point3::default() });
        Some(TravelLimits { min: point(params, &["travel", "min"])?.unwrap_or(current.min), max: point(params, &["travel", "max"])?.unwrap_or(current.max) })
    } else {
        None
    };
    if !kind_changed {
        if let (MachineKind::Mill5 { kinematics }, Some(layout)) = (&mut next.kind, params.get("kinematics").and_then(Value::as_str)) {
            let (a, b) = match kinematics {
                Kinematics::TableTable { primary, secondary } | Kinematics::HeadHead { primary, secondary } => (primary.clone(), secondary.clone()),
                Kinematics::HeadTable { head, table } => (head.clone(), table.clone()),
            };
            *kinematics = match layout {
                "tableTable" => Kinematics::TableTable { primary: a, secondary: b },
                "headHead" => Kinematics::HeadHead { primary: a, secondary: b },
                "headTable" => Kinematics::HeadTable { head: a, table: b },
                other => return Err(format!("kinematics: no layout `{other}` (one of: {})", KINEMATICS.join(", "))),
            };
        }
        if let (MachineKind::MillTurn { y_axis, .. }, Some(v)) = (&mut next.kind, flag(params, "yAxis")) {
            *y_axis = v;
        }
        if let Some(list) = params.get("axes").and_then(Value::as_object) {
            let mut axes = axes_mut(&mut next.kind);
            for (key, entry) in list {
                let i: usize = key.parse().map_err(|_| format!("axes.{key}: not an axis index"))?;
                let axis = axes.get_mut(i).ok_or_else(|| format!("axes.{i}: this machine has no axis {i}"))?;
                let path = |k: &str| format!("axes.{i}.{k}");
                if let Some(name) = entry.get("name").and_then(Value::as_str) {
                    axis.name = name.trim().to_string();
                }
                if let Some(p) = point(entry, &["direction"]).map_err(|_| format!("{}: three numbers [x, y, z]", path("direction")))? {
                    axis.direction = p;
                }
                if let Some(p) = point(entry, &["origin"]).map_err(|_| format!("{}: three numbers [x, y, z]", path("origin")))? {
                    axis.origin = p;
                }
                let limits_set = entry.get("limitsSet").and_then(Value::as_bool).unwrap_or(axis.limits_deg.is_some());
                axis.limits_deg = if limits_set {
                    let [lo0, hi0] = axis.limits_deg.unwrap_or([-360.0, 360.0]);
                    let lo = number(entry, &["limitsMin"]).map_err(|_| format!("{}: not a number", path("limitsMin")))?.unwrap_or(lo0);
                    let hi = number(entry, &["limitsMax"]).map_err(|_| format!("{}: not a number", path("limitsMax")))?.unwrap_or(hi0);
                    Some([lo, hi])
                } else {
                    None
                };
            }
        }
    }
    next.validate().map_err(|e| format!("machine: {e}"))?;
    *machine = next;
    Ok(())
}

