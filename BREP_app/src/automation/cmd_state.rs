//! Read the state registry and the hit-key registry; drain notices; dismiss
//! toasts before a deterministic capture.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Notice, Outcome, Phase};
use serde::{Deserialize, Serialize};
use serde_json::{json, Map, Value};
use std::collections::BTreeMap;

#[derive(Serialize, schemars::JsonSchema)]
pub struct StateEntry {
    pub name: String,
    pub doc: String,
    /// True when the blob was published from a typed value (its schema is known).
    pub typed: bool,
    pub bytes: usize,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct StateList {
    pub entries: Vec<StateEntry>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct StateGetArgs {
    /// Registry names, e.g. `__brepHistory`, `__brepSelection`, `__brepView`.
    pub names: Vec<String>,
}

/// Plural, like [`StateGetArgs`] — one argument NAME for "which blobs", so a
/// caller never has to remember which of the two state commands takes a list.
#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct StateSchemaArgs {
    /// Registry names, e.g. `__brepHistory`.
    pub names: Vec<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct HitRectsArgs {
    /// Restrict to keys starting with this `panel/key` prefix, e.g. `history/step:`.
    #[serde(default)]
    pub prefix: Option<String>,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct HitRects {
    /// `panel/key` → `[x, y, w, h]` in egui points, this frame's layout.
    pub rects: BTreeMap<String, [f32; 4]>,
    pub ppp: f32,
    pub frame: u64,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct Notices {
    pub notices: Vec<Notice>,
}

fn state_list(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let _ = ctx;
    let entries = crate::automation::registry::lock()
        .iter()
        .map(|(name, p)| StateEntry { name: name.to_string(), doc: p.doc.to_string(), typed: p.schema.is_some(), bytes: p.json.len() })
        .collect();
    serde_json::to_value(StateList { entries }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn state_get(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: StateGetArgs = parse_args(args)?;
    let _ = ctx;
    let mut out = Map::new();
    let reg = crate::automation::registry::lock();
    for name in a.names {
        match reg.get(&name) {
            Some(p) => {
                let v = serde_json::from_str::<Value>(&p.json).unwrap_or_else(|_| Value::String(p.json.clone()));
                out.insert(name, v);
            }
            None => return Err(format!("no state blob `{name}` (see state_list)")),
        }
    }
    Ok(Outcome::Done(Value::Object(out)))
}

fn state_schema(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: StateSchemaArgs = parse_args(args)?;
    let _ = ctx;
    let reg = crate::automation::registry::lock();
    let mut out = Map::new();
    for name in a.names {
        let p = reg.get(&name).ok_or_else(|| format!("no state blob `{name}` (see state_list)"))?;
        out.insert(
            name,
            json!({
                "doc": p.doc,
                "schema": p.schema.clone(),
                "example": serde_json::from_str::<Value>(&p.json).unwrap_or(Value::Null),
            }),
        );
    }
    Ok(Outcome::Done(Value::Object(out)))
}

fn hit_rects(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: HitRectsArgs = parse_args(args)?;
    let rects = crate::automation::registry::lock().hit_rects(a.prefix.as_deref());
    serde_json::to_value(HitRects { rects, ppp: ctx.egui.pixels_per_point(), frame: ctx.egui.cumulative_frame_nr() })
        .map(Outcome::Done)
        .map_err(|e| e.to_string())
}

fn hit_key_docs(_ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(json!({ "keys": crate::automation::hit_keys::hit_key_docs_json() })))
}

fn notices(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let notices = ctx.app.automation.take_notices();
    serde_json::to_value(Notices { notices }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn toasts_dismiss(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let n = ctx.app.toasts.dismiss_all();
    Ok(Outcome::Done(json!({ "dismissed": n })))
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ToastsRaiseArgs {
    /// Each text is queued as one engine notice, in order.
    pub texts: Vec<String>,
    /// How every one of them is drawn: `error` (the default, a refusal's red),
    /// `warning` (amber) or `info` (a success's green).
    #[serde(default)]
    pub severity: ToastSeverity,
}

#[derive(Deserialize, schemars::JsonSchema, Default, Clone, Copy)]
#[serde(rename_all = "lowercase")]
pub enum ToastSeverity {
    #[default]
    Error,
    Warning,
    Info,
}

/// Queue notices on the ENGINE, the lane every refusal takes, so they reach
/// the toast overlay and `notices` exactly as a real one does next frame.
fn toasts_raise(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ToastsRaiseArgs = parse_args(args)?;
    let n = a.texts.len();
    let severity = match a.severity {
        ToastSeverity::Error => brep_render::engine_state::NoticeSeverity::Error,
        ToastSeverity::Warning => brep_render::engine_state::NoticeSeverity::Warning,
        ToastSeverity::Info => brep_render::engine_state::NoticeSeverity::Info,
    };
    for text in a.texts {
        ctx.app.docs.engine_mut().push_notice_as(severity, text);
    }
    Ok(Outcome::Done(json!({ "raised": n })))
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "state_list", group: "state", doc: "Every state blob the app publishes this frame, with its doc line and whether a schema is known.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<StateList>, handler: Handler::App(state_list) },
    CommandSpec { name: "state_get", group: "state", doc: "Read named state blobs, parsed as JSON, from this frame's layout.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<StateGetArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(state_get) },
    CommandSpec { name: "state_schema", group: "state", doc: "The JSON Schema of named state blobs (where a blob is published from a typed value) and each one's current value as an example. Takes `names`, like `state_get`.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<StateSchemaArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(state_schema) },
    CommandSpec { name: "hit_rects", group: "widgets", doc: "Every clickable widget the app published this frame as `panel/key` → [x, y, w, h] in egui points. `panel/panel:clip` is a pane's visible region; a rect outside it must be scrolled into view first.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<HitRectsArgs>, result_schema: schema_of::<HitRects>, handler: Handler::App(hit_rects) },
    CommandSpec { name: "hit_key_docs", group: "widgets", doc: "The documented hit-key prefixes per panel: what each `panel/key` means.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(hit_key_docs) },
    CommandSpec { name: "notices", group: "state", doc: "Drain the notices (panics, logged errors, runner refusals, toasts) that arrived since the last drain.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Notices>, handler: Handler::App(notices) },
    CommandSpec { name: "toasts_raise", group: "state", doc: "Queue `texts` as engine notices, so each shows as a toast (and drains from `notices`) from the next frame, as a refusal's would — for checking the toast overlay itself: its cap, its coalescing of repeats, where its cards sit, and (`severity`: error, the default, warning or info) how each kind is drawn.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<ToastsRaiseArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(toasts_raise) },
    CommandSpec { name: "toasts_dismiss", group: "state", doc: "Dismiss every toast so a capture is deterministic; their text stays in `notices`.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(toasts_dismiss) },
];
