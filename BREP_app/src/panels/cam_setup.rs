//! The SETUP dialog of the CAM pane: one schema-driven form
//! ([`crate::form_view`]) over a setup's work coordinate system, stock,
//! heights and bodies, with the frame and the stock drawn in the viewport.
//!
//! The form edits a flat JSON params object ([`setup_form_params`]) that this
//! module maps to and from the model's `Setup` and its orientation picks
//! ([`apply_setup_form`]) — the same shape every other dialog uses, so the
//! field renderer, the picker and the automation (`cam_update_setup` takes a
//! patch of these keys) are shared rather than rewritten per field. Picks
//! commit through the engine's CAM ref-select target by a block path
//! ([`pick_path`]); the frame they resolve to is written back as
//! `modelToMachine` by the pane ([`crate::document::cam::resolve_frame`]).

use crate::document::cam::{CamDocument, SetupFrame, SetupOrientation};
use brep_cam::kinematics::library;
use brep_cam::model::{Kinematics, Machine, MachineKind, Point3, RotaryAxis, Setup, Stock, TargetRef};
use brep_render::scene::RenderScene;
use brep_render::style::{FieldKind, FormField};
use serde_json::{json, Value};
use std::collections::HashSet;

/// The stock kinds the form offers, as the model's serde tags.
pub const STOCK_KINDS: &[&str] = &["boundsOffset", "box", "cylinder", "fromSolid", "fromRest"];

fn field(key: &str, label: &str, group: &str, kind: FieldKind) -> FormField {
    FormField { path: vec![key.to_string()], label: label.to_string(), group: group.to_string(), kind }
}

fn scalar(key: &str, label: &str, group: &str) -> FormField {
    field(key, label, group, FieldKind::Scalar { step: 0.5 })
}

fn reference(key: &str, label: &str, group: &str, filter: &[&str], multiple: bool) -> FormField {
    field(key, label, group, FieldKind::Reference { filter: filter.iter().map(|s| s.to_string()).collect(), multiple })
}

/// The form's fields, in dialog order: Setup, Orientation, Stock, Heights,
/// Bodies. Every stock parameter is a field; [`hidden_fields`] hides the ones
/// the chosen stock kind does not use.
pub fn setup_form_fields() -> Vec<FormField> {
    vec![
        field("name", "Name", "Setup", FieldKind::Text { read_only: false }),
        field("wcsNumber", "Work offset (1 = G54)", "Setup", FieldKind::Number { min: 1.0, max: 99.0, step: 1.0 }),
        reference("zRef", "Z axis: a planar face (its normal), a cylindrical face (its axis), a straight edge or a feature's axis", "Orientation", &["FACE", "EDGE"], false),
        field("zDir", "Z direction instead of a pick (+X, -Y, ... or x,y,z): a tilted setup", "Orientation", FieldKind::Text { read_only: false }),
        field("flipZ", "Flip Z", "Orientation", FieldKind::Bool),
        reference("xRef", "X axis: a straight edge", "Orientation", &["EDGE"], false),
        reference("originRef", "Origin: a vertex or a face", "Orientation", &["VERTEX", "FACE"], false),
        field("stockKind", "Stock", "Stock", FieldKind::Enum { variants: STOCK_KINDS.iter().map(|s| s.to_string()).collect() }),
        scalar("stockX", "Side offset X", "Stock"),
        scalar("stockY", "Side offset Y", "Stock"),
        scalar("stockTop", "Top offset", "Stock"),
        scalar("stockBottom", "Bottom offset", "Stock"),
        field("stockMin", "Box corner (min)", "Stock", FieldKind::Vec3 { step: 0.5 }),
        field("stockMax", "Box corner (max)", "Stock", FieldKind::Vec3 { step: 0.5 }),
        field("stockBase", "Cylinder base", "Stock", FieldKind::Vec3 { step: 0.5 }),
        field("stockAxis", "Cylinder axis", "Stock", FieldKind::Vec3 { step: 0.1 }),
        scalar("stockRadius", "Cylinder radius", "Stock"),
        scalar("stockHeight", "Cylinder height", "Stock"),
        reference("stockSolid", "Stock body", "Stock", &["SOLID"], false),
        field("stockSetup", "Rest of setup (its simulated stock, an earlier setup's id)", "Stock", FieldKind::Text { read_only: false }),
        scalar("clearance", "Clearance height", "Heights"),
        scalar("retract", "Retract height", "Heights"),
        scalar("feed", "Feed height", "Heights"),
        reference("targets", "Bodies to machine", "Bodies", &["SOLID"], true),
        reference("protected", "Protected bodies (fixtures)", "Bodies", &["SOLID"], true),
        field("machine", "Machine", "Machine", FieldKind::Enum { variants: machine_choices() }),
        field("machineBase", "Custom: kinematics from", "Machine", FieldKind::Enum { variants: library::builtin_ids() }),
        field("machineName", "Custom: name", "Machine", FieldKind::Text { read_only: false }),
        field("machinePost", "Custom: post-processor", "Machine", FieldKind::Enum { variants: post_ids() }),
        scalar("rotary1Min", "Custom: first rotary min (deg, empty = continuous)", "Machine"),
        scalar("rotary1Max", "Custom: first rotary max (deg)", "Machine"),
        scalar("rotary2Min", "Custom: second rotary min (deg, empty = continuous)", "Machine"),
        scalar("rotary2Max", "Custom: second rotary max (deg)", "Machine"),
    ]
}

/// The machine picker's choices: none, every built-in, custom.
pub fn machine_choices() -> Vec<String> {
    let mut out = vec!["none".to_string()];
    out.extend(library::builtin_ids());
    out.push("custom".to_string());
    out
}

/// The built-in post-processor ids.
pub fn post_ids() -> Vec<String> {
    brep_cam::post::BUILTIN_POSTS.iter().map(|p| p.id.to_string()).collect()
}

/// The rotary axes of a machine, in its kinematic order (mutable).
fn rotaries_mut(machine: &mut Machine) -> Vec<&mut RotaryAxis> {
    match &mut machine.kind {
        MachineKind::Mill4 { rotary } => vec![rotary],
        MachineKind::MillTurn { c_axis, .. } => vec![c_axis],
        MachineKind::Mill5 { kinematics } => match kinematics {
            Kinematics::TableTable { primary, secondary } | Kinematics::HeadHead { primary, secondary } => vec![primary, secondary],
            Kinematics::HeadTable { head, table } => vec![head, table],
        },
        _ => Vec::new(),
    }
}

/// The rotary axes of a machine, in its kinematic order.
fn rotaries(machine: &Machine) -> Vec<RotaryAxis> {
    let mut m = machine.clone();
    rotaries_mut(&mut m).into_iter().map(|a| a.clone()).collect()
}

/// The built-in a machine's kinematics came from (same kind), if any.
fn base_of(machine: &Machine) -> Option<String> {
    library::builtin_machines().into_iter().find(|b| std::mem::discriminant(&b.kind) == std::mem::discriminant(&machine.kind) && rotaries(b).iter().map(|a| (&a.name, a.direction)).eq(rotaries(machine).iter().map(|a| (&a.name, a.direction)))).map(|b| b.id.as_str().to_string())
}

/// The machine keys of the setup form, read from the document.
pub fn machine_form_params(doc: &CamDocument, setup: &Setup) -> Value {
    let id = setup.machine.as_ref().map(|m| m.as_str().to_string());
    let custom = id.as_deref().and_then(|id| doc.machines.iter().find(|m| m.id.as_str() == id));
    let choice = match (&id, custom) {
        (None, _) => "none".to_string(),
        (Some(_), Some(_)) => "custom".to_string(),
        (Some(id), None) => id.clone(),
    };
    let mut params = json!({
        "machine": choice,
        "machineBase": "vmc-3axis",
        "machineName": "",
        "machinePost": "fanuc",
        "rotary1Min": "", "rotary1Max": "", "rotary2Min": "", "rotary2Max": "",
    });
    if let Some(m) = custom {
        params["machineBase"] = json!(base_of(m).unwrap_or_else(|| "vmc-3axis".into()));
        params["machineName"] = json!(m.name);
        params["machinePost"] = json!(m.post.clone().unwrap_or_default());
        for (i, axis) in rotaries(m).iter().enumerate().take(2) {
            let (lo, hi) = match axis.limits_deg {
                Some([lo, hi]) => (json!(lo), json!(hi)),
                None => (json!(""), json!("")),
            };
            params[format!("rotary{}Min", i + 1)] = lo;
            params[format!("rotary{}Max", i + 1)] = hi;
        }
    }
    params
}

/// Merge the machine keys into the form's params (the pane draws one form).
pub fn merge_machine_params(params: &mut Value, doc: &CamDocument, setup: &Setup) {
    if let (Some(target), Value::Object(machine)) = (params.as_object_mut(), machine_form_params(doc, setup)) {
        target.extend(machine);
    }
}

/// Apply the machine keys of a params object (or patch) to setup `setup_id`:
/// `machine` is `none`, a built-in id, `custom` (a copy of `machineBase`, or
/// of the current built-in, under a fresh document id unless the setup
/// already has a custom machine), or a document machine's id; a custom
/// machine then takes `machineName`, `machinePost`, a new `machineBase`
/// (its kinematics, travel and collision re-copied) and the rotary limits.
/// A value the machine refuses is an error naming the field; nothing is
/// applied then.
pub fn apply_machine_form(params: &Value, doc: &mut CamDocument, setup_id: &str) -> Result<(), String> {
    let mut next = doc.clone();
    let current = next.setup(setup_id).ok_or_else(|| format!("no setup `{setup_id}`"))?.machine.as_ref().map(|m| m.as_str().to_string());
    let is_custom = |d: &CamDocument, id: &Option<String>| id.as_deref().map_or(false, |id| d.machines.iter().any(|m| m.id.as_str() == id));
    let mut machine_id = current.clone();
    if let Some(choice) = params.get("machine").and_then(Value::as_str) {
        machine_id = match choice {
            "none" | "" => None,
            "custom" => {
                if is_custom(&next, &current) {
                    current.clone()
                } else {
                    // Copy the current built-in, else `machineBase`, else the
                    // 3-axis VMC; a `machineBase` that differs is applied below.
                    let base_id = current
                        .clone()
                        .filter(|c| library::builtin(c).is_some())
                        .or_else(|| params.get("machineBase").and_then(Value::as_str).map(str::to_string))
                        .unwrap_or_else(|| "vmc-3axis".to_string());
                    let base = library::builtin(&base_id).ok_or_else(|| format!("machineBase: no built-in machine `{base_id}`"))?;
                    Some(next.add_machine(base, None))
                }
            }
            id if library::builtin(id).is_some() || next.machines.iter().any(|m| m.id.as_str() == id) => Some(id.to_string()),
            other => return Err(format!("machine: no machine `{other}` (one of: {})", machine_choices().join(", "))),
        };
    }
    if let Some(setup) = next.setup_mut(setup_id) {
        setup.machine = machine_id.as_deref().map(Into::into);
    }
    if is_custom(&next, &machine_id) {
        let id = machine_id.clone().unwrap();
        let machine = next.machine_mut(&id).unwrap();
        if let Some(base_id) = params.get("machineBase").and_then(Value::as_str) {
            let base = library::builtin(base_id).ok_or_else(|| format!("machineBase: no built-in machine `{base_id}`"))?;
            if base_of(machine).as_deref() != Some(base_id) {
                machine.kind = base.kind;
                machine.travel = base.travel;
                machine.collision = base.collision;
                machine.spindle = base.spindle;
                if params.get("machinePost").is_none() {
                    machine.post = base.post;
                }
            }
        }
        if let Some(name) = params.get("machineName").and_then(Value::as_str) {
            if !name.trim().is_empty() {
                machine.name = name.to_string();
            }
        }
        if let Some(post) = params.get("machinePost").and_then(Value::as_str) {
            if !post_ids().iter().any(|p| p == post) {
                return Err(format!("machinePost: no built-in post `{post}` (one of: {})", post_ids().join(", ")));
            }
            machine.post = Some(post.to_string());
        }
        let mut axes = rotaries_mut(machine);
        for i in 0..2 {
            let (lo_key, hi_key) = (format!("rotary{}Min", i + 1), format!("rotary{}Max", i + 1));
            if params.get(&lo_key).is_none() && params.get(&hi_key).is_none() {
                continue;
            }
            let Some(axis) = axes.get_mut(i) else { continue };
            let lo = number(params, &lo_key)?;
            let hi = number(params, &hi_key)?;
            axis.limits_deg = match (lo, hi) {
                (None, None) => None,
                (Some(lo), Some(hi)) => Some([lo, hi]),
                _ => return Err(format!("{lo_key}/{hi_key}: give both limits, or neither for a continuous axis")),
            };
        }
        machine.validate().map_err(|e| format!("machine: {e}"))?;
    }
    *doc = next;
    Ok(())
}

/// The field keys the stock kind in `params` does not use.
pub fn hidden_fields(params: &Value) -> HashSet<String> {
    let kind = params.get("stockKind").and_then(Value::as_str).unwrap_or("boundsOffset");
    let offsets = ["stockX", "stockY", "stockTop", "stockBottom"];
    let boxed = ["stockMin", "stockMax"];
    let cylinder = ["stockBase", "stockAxis", "stockRadius", "stockHeight"];
    let solid = ["stockSolid"];
    let rest = ["stockSetup"];
    let mut hidden_machine: Vec<String> = Vec::new();
    let custom_keys = ["machineBase", "machineName", "machinePost", "rotary1Min", "rotary1Max", "rotary2Min", "rotary2Max"];
    if params.get("machine").and_then(Value::as_str) != Some("custom") {
        hidden_machine.extend(custom_keys.iter().map(|k| k.to_string()));
    } else if let Some(base) = params.get("machineBase").and_then(Value::as_str).and_then(library::builtin) {
        let n = rotaries(&base).len();
        if n < 2 {
            hidden_machine.extend(["rotary2Min", "rotary2Max"].iter().map(|k| k.to_string()));
        }
        if n < 1 {
            hidden_machine.extend(["rotary1Min", "rotary1Max"].iter().map(|k| k.to_string()));
        }
    }
    let keep: &[&str] = match kind {
        "box" => &boxed,
        "cylinder" => &cylinder,
        "fromSolid" => &solid,
        "fromRest" => &rest,
        _ => &offsets,
    };
    offsets
        .iter()
        .chain(&boxed)
        .chain(&cylinder)
        .chain(&solid)
        .chain(&rest)
        .filter(|key| !keep.contains(key))
        .map(|key| key.to_string())
        .chain(hidden_machine)
        .collect()
}

fn vec3(p: Point3) -> Value {
    json!([p.x, p.y, p.z])
}

fn refs(list: &[TargetRef]) -> Value {
    json!(list.iter().map(|r| r.solid.clone()).collect::<Vec<_>>())
}

/// The form's params for a setup and its picks.
pub fn setup_form_params(setup: &Setup, orientation: &SetupOrientation) -> Value {
    let mut params = json!({
        "name": setup.name,
        "wcsNumber": setup.wcs.number,
        "zRef": orientation.z_ref.clone().unwrap_or_default(),
        "zDir": orientation.z_dir.clone().unwrap_or_default(),
        "flipZ": orientation.flip_z,
        "xRef": orientation.x_ref.clone().unwrap_or_default(),
        "originRef": orientation.origin_ref.clone().unwrap_or_default(),
        "stockKind": "boundsOffset",
        "stockX": 0.0, "stockY": 0.0, "stockTop": 0.0, "stockBottom": 0.0,
        "stockMin": [0.0, 0.0, 0.0], "stockMax": [10.0, 10.0, 10.0],
        "stockBase": [0.0, 0.0, 0.0], "stockAxis": [0.0, 0.0, 1.0], "stockRadius": 10.0, "stockHeight": 10.0,
        "stockSolid": "",
        "stockSetup": "",
        "clearance": setup.heights.clearance,
        "retract": setup.heights.retract,
        "feed": setup.heights.feed,
        "targets": refs(&setup.targets),
        "protected": refs(&setup.protected),
    });
    match &setup.stock {
        Stock::BoundsOffset { x, y, top, bottom } => {
            params["stockKind"] = json!("boundsOffset");
            params["stockX"] = json!(x);
            params["stockY"] = json!(y);
            params["stockTop"] = json!(top);
            params["stockBottom"] = json!(bottom);
        }
        Stock::Box { min, max } => {
            params["stockKind"] = json!("box");
            params["stockMin"] = vec3(*min);
            params["stockMax"] = vec3(*max);
        }
        Stock::Cylinder { base, axis, radius, height } => {
            params["stockKind"] = json!("cylinder");
            params["stockBase"] = vec3(*base);
            params["stockAxis"] = vec3(*axis);
            params["stockRadius"] = json!(radius);
            params["stockHeight"] = json!(height);
        }
        Stock::FromSolid { solid } => {
            params["stockKind"] = json!("fromSolid");
            params["stockSolid"] = json!(solid);
        }
        Stock::FromRest { setup } => {
            params["stockKind"] = json!("fromRest");
            params["stockSetup"] = json!(setup.as_str());
        }
        #[allow(unreachable_patterns)]
        _ => {}
    }
    params
}

/// A number a form field wrote: a JSON number, or a numeric string (the
/// scalar field accepts expressions; a plain literal is what a setup takes).
fn number(params: &Value, key: &str) -> Result<Option<f64>, String> {
    match params.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::Number(n)) => Ok(n.as_f64()),
        Some(Value::String(s)) if s.trim().is_empty() => Ok(None),
        Some(Value::String(s)) => s.trim().parse::<f64>().map(Some).map_err(|_| format!("{key}: `{s}` is not a number")),
        Some(other) => Err(format!("{key}: {other} is not a number")),
    }
}

fn point(params: &Value, key: &str) -> Result<Option<Point3>, String> {
    let Some(value) = params.get(key) else { return Ok(None) };
    let Some(list) = value.as_array() else { return Err(format!("{key}: not a 3-vector")) };
    if list.len() != 3 {
        return Err(format!("{key}: not a 3-vector"));
    }
    let mut out = [0.0; 3];
    for (i, item) in list.iter().enumerate() {
        out[i] = match item {
            Value::Number(n) => n.as_f64().unwrap_or(0.0),
            Value::String(s) => s.trim().parse::<f64>().map_err(|_| format!("{key}: `{s}` is not a number"))?,
            other => return Err(format!("{key}: {other} is not a number")),
        };
    }
    Ok(Some(Point3::new(out[0], out[1], out[2])))
}

fn name_list(params: &Value, key: &str) -> Option<Vec<String>> {
    params.get(key).and_then(Value::as_array).map(|list| list.iter().filter_map(Value::as_str).map(str::to_string).collect())
}

fn name_opt(params: &Value, key: &str) -> Option<Option<String>> {
    params.get(key).and_then(Value::as_str).map(|s| if s.trim().is_empty() { None } else { Some(s.to_string()) })
}

/// Apply the form's params (or a PATCH of them: absent keys are unchanged)
/// to a setup and its picks. A value that cannot be read is an error naming
/// the field, and nothing is applied; the stock is validated the way the
/// model does, so an impossible box is refused here rather than at
/// generation.
pub fn apply_setup_form(params: &Value, setup: &mut Setup, orientation: &mut SetupOrientation) -> Result<(), String> {
    let mut next = setup.clone();
    let mut picks = orientation.clone();
    if let Some(name) = params.get("name").and_then(Value::as_str) {
        if !name.trim().is_empty() {
            next.name = name.to_string();
        }
    }
    if let Some(number) = number(params, "wcsNumber")? {
        if number < 1.0 || number > 99.0 || number.fract() != 0.0 {
            return Err(format!("wcsNumber: {number} is not a work offset number (1-99)"));
        }
        next.wcs.number = number as u32;
    }
    if let Some(flip) = params.get("flipZ").and_then(Value::as_bool) {
        picks.flip_z = flip;
    }
    let z_dir = params.get("zDir").and_then(Value::as_str).map(str::trim);
    if let (Some(Some(_)), Some(dir)) = (name_opt(params, "zRef"), z_dir) {
        if !dir.is_empty() {
            return Err("zDir: a Z direction and a Z pick (zRef) exclude each other; give one".into());
        }
    }
    if let Some(value) = name_opt(params, "zRef") {
        if value.is_some() {
            picks.z_dir = None;
        }
        picks.z_ref = value;
    }
    if let Some(dir) = z_dir {
        if dir.is_empty() {
            picks.z_dir = None;
        } else {
            crate::document::cam::parse_direction(dir).map_err(|e| format!("zDir: {e}"))?;
            picks.z_dir = Some(dir.to_string());
            picks.z_ref = None;
        }
    }
    if let Some(value) = name_opt(params, "xRef") {
        picks.x_ref = value;
    }
    if let Some(value) = name_opt(params, "originRef") {
        picks.origin_ref = value;
    }
    let current = setup_form_params(setup, orientation);
    let merged = {
        let mut m = current.clone();
        if let (Some(target), Some(patch)) = (m.as_object_mut(), params.as_object()) {
            for (k, v) in patch {
                target.insert(k.clone(), v.clone());
            }
        }
        m
    };
    let kind = merged.get("stockKind").and_then(Value::as_str).unwrap_or("boundsOffset");
    let stock_changed = ["stockKind", "stockX", "stockY", "stockTop", "stockBottom", "stockMin", "stockMax", "stockBase", "stockAxis", "stockRadius", "stockHeight", "stockSolid", "stockSetup"]
        .iter()
        .any(|key| params.get(key).is_some());
    if stock_changed {
        next.stock = match kind {
            "boundsOffset" => Stock::BoundsOffset {
                x: number(&merged, "stockX")?.unwrap_or(0.0),
                y: number(&merged, "stockY")?.unwrap_or(0.0),
                top: number(&merged, "stockTop")?.unwrap_or(0.0),
                bottom: number(&merged, "stockBottom")?.unwrap_or(0.0),
            },
            "box" => Stock::Box {
                min: point(&merged, "stockMin")?.unwrap_or(Point3::ZERO),
                max: point(&merged, "stockMax")?.unwrap_or(Point3::new(10.0, 10.0, 10.0)),
            },
            "cylinder" => Stock::Cylinder {
                base: point(&merged, "stockBase")?.unwrap_or(Point3::ZERO),
                axis: point(&merged, "stockAxis")?.unwrap_or(Point3::new(0.0, 0.0, 1.0)),
                radius: number(&merged, "stockRadius")?.unwrap_or(10.0),
                height: number(&merged, "stockHeight")?.unwrap_or(10.0),
            },
            "fromSolid" => Stock::FromSolid { solid: merged.get("stockSolid").and_then(Value::as_str).unwrap_or("").to_string() },
            "fromRest" => {
                let id = merged.get("stockSetup").and_then(Value::as_str).unwrap_or("").trim();
                if id.is_empty() {
                    return Err("stockSetup: a rest stock names the earlier setup whose simulated stock it starts from".into());
                }
                if id == next.id.as_str() {
                    return Err(format!("stockSetup: setup `{id}` cannot start from its own rest"));
                }
                Stock::FromRest { setup: id.into() }
            }
            other => return Err(format!("stockKind: no stock kind `{other}` (one of: {})", STOCK_KINDS.join(", "))),
        };
        next.stock.validate().map_err(|e| format!("stock: {e}"))?;
    }
    if let Some(v) = number(params, "clearance")? {
        next.heights.clearance = v;
    }
    if let Some(v) = number(params, "retract")? {
        next.heights.retract = v;
    }
    if let Some(v) = number(params, "feed")? {
        next.heights.feed = v;
    }
    next.heights.validate().map_err(|e| format!("heights: {e}"))?;
    if let Some(list) = name_list(params, "targets") {
        next.targets = list.into_iter().map(TargetRef::solid).collect();
    }
    if let Some(list) = name_list(params, "protected") {
        next.protected = list.into_iter().map(TargetRef::solid).collect();
    }
    if let Some(value) = params.get("turningStickOut") {
        next.turning_stick_out = serde_json::from_value(value.clone()).map_err(|e| format!("turningStickOut: {e}"))?;
        if let Some(stick) = next.turning_stick_out {
            if !stick.is_finite() || stick <= 0.0 { return Err("turningStickOut: expected a positive finite length".into()); }
        }
    }
    *setup = next;
    *orientation = picks;
    Ok(())
}

/// Where a reference field's pick commits: the block path from the root,
/// the entity kinds it takes, whether it is a list, and its label. `None`
/// for a key that is not a reference field.
pub fn pick_path(setup_id: &str, key: &str) -> Option<(Vec<String>, Vec<String>, bool, String)> {
    let names = |kinds: &[&str]| kinds.iter().map(|k| k.to_string()).collect::<Vec<_>>();
    Some(match key {
        "zRef" => (vec!["orientations".into(), setup_id.into(), "zRef".into()], names(&["FACE", "EDGE"]), false, "Z axis: face or edge".into()),
        "xRef" => (vec!["orientations".into(), setup_id.into(), "xRef".into()], names(&["EDGE"]), false, "X axis edge".into()),
        "originRef" => (vec!["orientations".into(), setup_id.into(), "originRef".into()], names(&["VERTEX", "FACE"]), false, "Origin".into()),
        "stockSolid" => (vec!["setups".into(), format!("#{setup_id}"), "stock".into(), "solid".into()], names(&["SOLID"]), false, "Stock body".into()),
        "targets" => (vec!["setups".into(), format!("#{setup_id}"), "targets".into()], names(&["SOLID"]), true, "Bodies to machine".into()),
        "protected" => (vec!["setups".into(), format!("#{setup_id}"), "protected".into()], names(&["SOLID"]), true, "Protected bodies".into()),
        _ => return None,
    })
}

/// The model-space bounds the stock is sized from: the setup's target
/// bodies, or every visible solid when none is named. `None` with no mesh.
pub fn target_bounds(setup: &Setup, scene: &RenderScene) -> Option<(Point3, Point3)> {
    let named: Vec<&str> = setup.targets.iter().map(|t| t.solid.as_str()).collect();
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    let mut any = false;
    for solid in scene.solids() {
        let wanted = if named.is_empty() { solid.visible } else { named.contains(&solid.name.as_str()) };
        if !wanted {
            continue;
        }
        for p in &solid.mesh.positions {
            for axis in 0..3 {
                min[axis] = min[axis].min(p[axis] as f64);
                max[axis] = max[axis].max(p[axis] as f64);
            }
            any = true;
        }
    }
    any.then(|| (Point3::new(min[0], min[1], min[2]), Point3::new(max[0], max[1], max[2])))
}

/// The stock's model-space bounds for the viewport box: the model's own
/// rule for offset and explicit stocks; a stock body's bounds for
/// `fromSolid`; `None` when it cannot be sized (no target mesh, no such body).
pub fn stock_bounds(setup: &Setup, scene: &RenderScene) -> Option<(Point3, Point3)> {
    match &setup.stock {
        Stock::FromSolid { solid } => {
            let body = scene.solid(solid)?;
            let mut min = [f64::INFINITY; 3];
            let mut max = [f64::NEG_INFINITY; 3];
            for p in &body.mesh.positions {
                for axis in 0..3 {
                    min[axis] = min[axis].min(p[axis] as f64);
                    max[axis] = max[axis].max(p[axis] as f64);
                }
            }
            (!body.mesh.positions.is_empty()).then(|| (Point3::new(min[0], min[1], min[2]), Point3::new(max[0], max[1], max[2])))
        }
        Stock::BoundsOffset { .. } => {
            // The offsets follow the setup's frame (BREP_cam's resolver); the box
            // itself stays in the model frame.
            let (min, max) = target_bounds(setup, scene)?;
            let b = brep_cam::stock::bounds_offset_box(setup, &brep_cam::model::Bounds3::new(min, max))?;
            Some((b.bounds.min, b.bounds.max))
        }
        _ => {
            let (min, max) = target_bounds(setup, scene)?;
            let bounds = setup.stock.model_bounds(&brep_cam::model::Bounds3::new(min, max))?;
            Some((bounds.min, bounds.max))
        }
    }
}

/// A `BoundsOffset` stock's box in MACHINE coordinates, as generation
/// resolves it (`brep_cam::stock::bounds_offset_box`); `None` for
/// another stock kind or no targets.
pub fn stock_bounds_machine(setup: &Setup, scene: &RenderScene) -> Option<brep_cam::model::Bounds3> {
    let (min, max) = target_bounds(setup, scene)?;
    brep_cam::stock::bounds_offset_box(setup, &brep_cam::model::Bounds3::new(min, max)).map(|b| b.machine_bounds())
}

/// The overlay group the viewport draws for a setup: the stock box's twelve
/// edges in grey and the machine frame's triad at its origin (X red, Y green,
/// Z blue), sized from the stock. Lines only, so it never occludes the model.
pub fn overlay_group(frame: &SetupFrame, stock: Option<(Point3, Point3)>) -> Value {
    const STOCK: [f32; 3] = [0.62, 0.65, 0.70];
    const X: [f32; 3] = [0.93, 0.26, 0.21];
    const Y: [f32; 3] = [0.30, 0.78, 0.40];
    const Z: [f32; 3] = [0.25, 0.55, 0.95];
    let mut positions: Vec<f64> = Vec::new();
    let mut colors: Vec<f32> = Vec::new();
    let mut line = |a: Point3, b: Point3, c: [f32; 3]| {
        positions.extend_from_slice(&[a.x, a.y, a.z, b.x, b.y, b.z]);
        colors.extend_from_slice(&c);
        colors.extend_from_slice(&c);
    };
    let mut size = 10.0;
    if let Some((min, max)) = stock {
        let c = |i: u8| Point3::new(if i & 1 == 0 { min.x } else { max.x }, if i & 2 == 0 { min.y } else { max.y }, if i & 4 == 0 { min.z } else { max.z });
        for (a, b) in [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)] {
            line(c(a), c(b), STOCK);
        }
        size = ((max.x - min.x).max(max.y - min.y).max(max.z - min.z) * 0.35).max(1.0);
    }
    let z = frame.z_axis.normalized().unwrap_or(Point3::new(0.0, 0.0, 1.0));
    let x0 = frame.x_axis.sub(z.scale(frame.x_axis.dot(z)));
    let x = x0.normalized().unwrap_or(Point3::new(1.0, 0.0, 0.0));
    let y = z.cross(x);
    line(frame.origin, frame.origin.add(x.scale(size)), X);
    line(frame.origin, frame.origin.add(y.scale(size)), Y);
    line(frame.origin, frame.origin.add(z.scale(size)), Z);
    json!({ "name": "cam-setup", "renderOrder": 5, "lines": { "positions": positions, "colors": colors } })
}

