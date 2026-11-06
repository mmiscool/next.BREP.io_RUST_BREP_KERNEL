//! Motion studies (`brep_render::engine_state` `simulation_ops`,
//! `brep_simulation`): studies of an assembly, motors on its distance and
//! angle mates, and the timeline's clips. Every edit is one undo step and is
//! refused, with nothing written, when the study would not validate.
use crate::automation::command::{app_command, parse_args, schema_of, CommandSpec, Ctx, NoArgs, Outcome};
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct NewStudyArgs {
    /// The study's name; numbered (`Study 2`) when absent or empty.
    #[serde(default)]
    pub name: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddMotorArgs {
    /// The study id; the active study when absent (a new one when there is none).
    #[serde(default)]
    pub study: Option<String>,
    /// The distance or angle constraint to drive (`ANGL1`); the first such
    /// mate when absent.
    #[serde(default)]
    pub constraint: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddClipArgs {
    /// The study id; the active study when absent (a new one when there is none).
    #[serde(default)]
    pub study: Option<String>,
    /// `disable` (alias `suppress`, the stored kind), `camera`, `visibility`,
    /// `transparency`, `section` or `explode`.
    pub kind: String,
    /// What the clip acts on, by the kind: a constraint id, a PMI view id, an
    /// occurrence id, an exploded view id. The natural default when absent
    /// (the first constraint; every PMI view with a camera; the selected
    /// occurrences or the first component; the first exploded view).
    #[serde(default)]
    pub target: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UpdateArgs {
    /// The study id.
    pub study: String,
    /// A motor or clip of the study to edit; the study itself when absent.
    #[serde(default)]
    pub item: Option<String>,
    /// Fields to set. A study: `name`, `duration` (s), `fps`, `interference`
    /// (`{enabled}`). A motor: `constraint`, `from`, `to` (s) and `drive`
    /// (`{mode: speed, speed}` or `{mode: profile, shape: linear|ease|trapezoid,
    /// start, end, accel?}`). A clip: `from`, `to` and its kind's fields.
    /// `null` removes an optional field.
    #[serde(default)]
    pub patch: Option<Value>,
    /// Make this study the active one.
    #[serde(default)]
    pub activate: Option<bool>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RemoveArgs {
    /// The study id.
    pub study: String,
    /// A motor or clip of the study; the whole study when absent.
    #[serde(default)]
    pub item: Option<String>,
}

fn state(ctx: &mut Ctx<'_>) -> Value {
    serde_json::from_str(&crate::panels::simulation::state_json(ctx.app.docs.engine(), &ctx.app.simulation)).unwrap_or(Value::Null)
}

fn with(mut state: Value, key: &str, value: Value) -> Value {
    state[key] = value;
    state
}

fn simulation_state(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(state(ctx)))
}

fn simulation_new_study(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: NewStudyArgs = parse_args(args)?;
    let id = ctx.app.docs.engine_mut().simulation_add_study(a.name.as_deref().unwrap_or(""))?;
    Ok(Outcome::Done(with(state(ctx), "study", json!(id))))
}

fn simulation_add_motor(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AddMotorArgs = parse_args(args)?;
    let (study, motor) = ctx.app.docs.engine_mut().simulation_add_motor(a.study.as_deref(), a.constraint.as_deref())?;
    let out = with(state(ctx), "study", json!(study));
    Ok(Outcome::Done(with(out, "motor", json!(motor))))
}

fn simulation_add_clip(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AddClipArgs = parse_args(args)?;
    let (study, clip) = ctx.app.docs.engine_mut().simulation_add_clip(a.study.as_deref(), &a.kind, a.target.as_deref())?;
    let out = with(state(ctx), "study", json!(study));
    Ok(Outcome::Done(with(out, "clip", json!(clip))))
}

fn simulation_update(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: UpdateArgs = parse_args(args)?;
    if a.patch.is_none() && a.activate != Some(true) {
        return Err("give a patch, or activate: true".into());
    }
    let engine = ctx.app.docs.engine_mut();
    if let Some(patch) = &a.patch {
        engine.simulation_patch(&a.study, a.item.as_deref(), patch, None)?;
    }
    if a.activate == Some(true) {
        engine.simulation_set_active(&a.study)?;
    }
    Ok(Outcome::Done(state(ctx)))
}

fn simulation_remove(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: RemoveArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().simulation_remove(&a.study, a.item.as_deref())?;
    Ok(Outcome::Done(state(ctx)))
}

fn simulation_object(properties: Value) -> Value {
    let required: Vec<_> = properties.as_object().unwrap().keys().cloned().collect();
    json!({"type":"object","properties":properties,"required":required,"additionalProperties":false})
}

fn simulation_result_schema() -> Value {
    let mut block = schema_of::<brep_simulation::Simulation>();
    block.as_object_mut().unwrap().remove("$schema");
    let defs = block.as_object_mut().unwrap().remove("$defs").unwrap_or_else(|| json!({}));
    let nullable_text = json!({"type":["string","null"]});
    let pane = simulation_object(json!({"notice":nullable_text,"error":nullable_text,"open":nullable_text}));
    let mate = simulation_object(json!({
        "id":{"type":"string"},"kind":{"enum":["distance","angle"]},
        "value":{"type":["number","null"]},"enabled":{"type":"boolean"}
    }));
    let populated = simulation_object(json!({
        "available":{"type":"boolean"},
        "block":{"anyOf":[{"type":"null"},block,simulation_object(json!({"error":{"type":"string"}}))]},
        "mates":{"type":"array","items":mate},
        "playback":super::cmd_simulation_play::simulation_playback_result_schema(),
        "pane":{"anyOf":[{"type":"null"},pane]}
    }));
    // Preserve the native parse fallback; the MCP adapter wraps null as value:null.
    json!({"oneOf":[{"type":"null"},populated],"$defs":defs})
}

fn simulation_extended(properties: Value) -> Value {
    let mut result = simulation_result_schema();
    // Indexing the native null fallback creates an object containing only
    // the command's identifiers. Close that branch to avoid validation bypass.
    result["oneOf"][0] = simulation_object(properties.clone());
    let full = &mut result["oneOf"][1];
    for (key, schema) in properties.as_object().unwrap() {
        full["properties"][key] = schema.clone();
        full["required"].as_array_mut().unwrap().push(json!(key));
    }
    result
}
fn simulation_new_study_result_schema() -> Value { simulation_extended(json!({"study":{"type":"string"}})) }
fn simulation_add_motor_result_schema() -> Value { simulation_extended(json!({"study":{"type":"string"},"motor":{"type":"string"}})) }
fn simulation_add_clip_result_schema() -> Value { simulation_extended(json!({"study":{"type":"string"},"clip":{"type":"string"}})) }

pub static COMMANDS: &[CommandSpec] = &[
    app_command!("simulation", "simulation_state", "The active document's motion studies: `available` (an assembly a study can drive), `block` (the `simulation` block: the active study and each study's duration, fps, motors with their mate, span and drive, clips and interference setting; null when the document has none) and `mates` (the distance and angle constraints a motor can drive, with their kind, value and enabled flag), and `pane` (the Simulation pane's last notice and error).", NoArgs, simulation_state, Read, READ, simulation_result_schema),
    app_command!("simulation", "simulation_new_study", "Add a motion study to the active assembly (4 s at 30 fps, named `name` or numbered) and make it the active one. Refused on a document with no assembly. One undo step. Returns the state plus the new `study` id.", NewStudyArgs, simulation_new_study, Mutate, MUTATE, simulation_new_study_result_schema),
    app_command!("simulation", "simulation_add_motor", "Add a motor to a study (the active one; a new study when there is none): it drives the distance or angle mate `constraint` (the first such mate when absent) for the whole study at 90 °/s for an angle mate or 10 mm/s for a distance mate; simulation_update changes its span and drive. Refused when the mate is not a distance or angle constraint of this assembly. One undo step. Returns the state plus `study` and `motor`.", AddMotorArgs, simulation_add_motor, Mutate, MUTATE, simulation_add_motor_result_schema),
    app_command!("simulation", "simulation_add_clip", "Add a clip of `kind` to a study (the active one; a new study when there is none), spanning the whole study, on `target` or the natural default: `disable` (alias and stored kind `suppress`; the panel's Disable a mate) switches a constraint off inside the span (the first constraint); `camera` moves the camera through PMI views' cameras, evenly over the span, and holds the last (every view with a camera, or the one named); `visibility` hides occurrences inside the span (the selected ones, else the first component); `transparency` fades occurrences from `opacity[0]` to `opacity[1]` over the span (the selected ones, else the first component; 1 → 0.3); `section` cuts the scene on a PMI view's section planes inside the span, `sweep` moving them along their normals (the first view with a section); `explode` plays an exploded view's steps over the span (the first exploded view; `collapse` runs them backwards). simulation_update changes the span and the kind's fields. Refused when the kind has nothing to act on (no PMI view, no view with a section, no exploded view). One undo step. Returns the state plus `study` and `clip`.", AddClipArgs, simulation_add_clip, Mutate, MUTATE, simulation_add_clip_result_schema),
    app_command!("simulation", "simulation_update", "Edit a study (`item` absent) or one of its motors or clips with `patch` (the fields are listed on the argument), and/or `activate` the study. Given fields replace the current ones; an edit that leaves the study invalid (a span outside the study, two motors on one mate at once, an unknown field value) is refused with nothing written. One undo step. Returns the state.", UpdateArgs, simulation_update, Mutate, MUTATE, simulation_result_schema),
    app_command!("simulation", "simulation_remove", "Delete a study, or one of its motors or clips (`item`). Deleting the active study makes the first remaining one active. One undo step. Returns the state.", RemoveArgs, simulation_remove, Mutate, MUTATE, simulation_result_schema),
];
