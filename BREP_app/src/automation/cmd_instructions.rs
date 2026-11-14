//! Manufacturing instructions (`brep_render::engine_state` `instructions_ops`,
//! `brep_instructions`): make instructions from an assembly, run the wizard,
//! and edit, merge, split and reorder steps. Every edit is one undo step and
//! is refused, with nothing written, when the block would not validate. An
//! edit that changes which step adds what also brings the step views up to
//! date (hidden parts, balloons), keeping each step's camera.
use crate::automation::command::{app_command, parse_args, schema_of, CommandSpec, Ctx, NoArgs, Outcome};
use brep_instructions::edit;
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct NewArgs {
    /// The store key of the assembly; the active tab's (saved) document when
    /// absent.
    #[serde(default)]
    pub from: Option<String>,
    /// Run the wizard on the new instructions (default true).
    #[serde(default)]
    pub wizard: Option<bool>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct StepArgs {
    /// The step id (`MS3`).
    pub step: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UpdateStepArgs {
    /// The step id.
    pub step: String,
    /// Fields to set: `title`, `notes`, `view` (a PMI view id or null),
    /// `tools` (`[{toolId?, name, spec, unlinked?}]`), `torque`
    /// (`{value, unit}` or null; an authored torque is an override),
    /// `callouts` (`[{kind: caution|note|check, text, occurrence?}]`),
    /// `arrows` (`[{occurrence, direction, distance}]`), `signoff`
    /// (`{required}`), `motion`, `fields` (the whole list of step fields,
    /// each `{label, kind, …}` as `instructions_add_field` takes it; one
    /// carrying its `id` keeps it).
    pub patch: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddFieldArgs {
    /// The step the field joins.
    pub step: String,
    /// The field: `{label, kind, required?, hint?}` plus the kind's own keys.
    /// `kind` is `check` (one box to tick), `choice` (`options: [text]`,
    /// `multiple?`: any number may be picked, `other?`: a free-text Other
    /// value is allowed), `text` (`lines?`: 1 = one line, up to 20 = a box),
    /// `number` (a measurement: `unit?`, `nominal?`, `min?`, `max?`,
    /// `decimals?`, `requiredInLimits?` default true: a value outside the
    /// limits blocks the step) or `serial` (a serial number not tied to a
    /// part).
    pub field: Value,
    /// Where among the step's fields (0 = first); the end when absent.
    #[serde(default)]
    pub index: Option<usize>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UpdateFieldArgs {
    /// The step holding the field.
    pub step: String,
    /// The field id (`MF12`).
    pub field: String,
    /// Keys to set, as in `instructions_add_field`'s `field`; `null` clears
    /// an optional one (`nominal`, `min`, `max`, `decimals`). A new `kind`
    /// keeps the label, the hint and `required` and takes the kind's
    /// defaults for the rest.
    pub patch: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FieldArgs {
    /// The step holding the field.
    pub step: String,
    /// The field id (`MF12`).
    pub field: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MoveFieldArgs {
    /// The step holding the field.
    pub step: String,
    /// The field id (`MF12`).
    pub field: String,
    /// Its new index among the step's fields (clamped to the end).
    pub index: usize,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MergeArgs {
    /// Two or more sibling step ids; they merge into the first in their
    /// order.
    pub steps: Vec<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SplitArgs {
    /// The step id.
    pub step: String,
    /// Indices of the step's parts (`adds`) that move to a new step right
    /// after it.
    pub adds: Vec<usize>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MoveStepArgs {
    /// The step id.
    pub step: String,
    /// Its new index among its siblings (clamped to the end), or among the
    /// new parent's children when `parent` is given.
    pub index: usize,
    /// The step to nest it under (children and all); `""` for the top
    /// level; absent: it stays under its parent.
    #[serde(default)]
    pub parent: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddPartsArgs {
    /// The step the parts join.
    pub step: String,
    /// Occurrence paths of the assembly (`ACOMP1:ACOMP3`), a BOM row's.
    #[serde(default)]
    pub occurrences: Option<Vec<String>>,
    /// Or a part name: every occurrence of it at the assembly's top level.
    #[serde(default)]
    pub part: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RemoveAddArgs {
    /// The step the part leaves.
    pub step: String,
    /// The part's index in that step's `adds`.
    pub add: usize,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MoveAddArgs {
    /// The step the part leaves.
    pub from: String,
    /// The part's index in that step's `adds`.
    pub add: usize,
    /// The step the part joins (a sibling).
    pub to: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct NewStepArgs {
    /// The step the new one follows, as its next sibling; the end of the
    /// top level when absent.
    #[serde(default)]
    pub after: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UpdateArgs {
    /// The page template: `{stepsPerPage: 1|2|4, orientation:
    /// portrait|landscape, paper, style: lineArt|shaded}`; given keys
    /// replace the current ones.
    #[serde(default)]
    pub template: Option<Value>,
    /// The front matter: `{title, subtitle, coverStep, partsList, toolsList}`;
    /// given keys replace the current ones.
    #[serde(default)]
    pub front: Option<Value>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PlayArgs {
    /// Where to go: `first`, `next`, `back`, `replay` (the shown step, its
    /// parts coming in again) or `step` (with `step`). Absent: stay.
    #[serde(default)]
    pub to: Option<String>,
    /// The step id, for `to: step`.
    #[serde(default)]
    pub step: Option<String>,
    /// The Player pane's mode: `shop` or `customer`.
    #[serde(default)]
    pub mode: Option<String>,
    /// Hold the shown step's insertion at this fraction (0 = where its parts
    /// start, 1 = seated) until the next step is played, so a picture of
    /// it part way home is deterministic; nothing when no part moves.
    #[serde(default)]
    pub hold: Option<f64>,
}

fn state(ctx: &mut Ctx<'_>) -> Value {
    ctx.app.docs.engine().instructions_state_json()
}

fn instructions_new(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: NewArgs = parse_args(args)?;
    let (_, docs, store) = ctx.app.file_docs_store();
    let source = match a.from.as_deref() {
        Some(key) => crate::instructions::Source::Stored(key),
        None => crate::instructions::Source::Active,
    };
    let index = crate::instructions::new_instructions(docs, store, source, a.wizard.unwrap_or(true))?;
    let mut out = state(ctx);
    out["index"] = json!(index);
    Ok(Outcome::Done(out))
}

fn instructions_state(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(state(ctx)))
}

fn instructions_wizard(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().instructions_run_wizard()?;
    Ok(Outcome::Done(state(ctx)))
}

fn instructions_retarget_approve(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().instructions_retarget_approve()?;
    Ok(Outcome::Done(state(ctx)))
}

fn instructions_retarget_reject(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().instructions_retarget_reject()?;
    Ok(Outcome::Done(state(ctx)))
}

fn instructions_capture_camera(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: StepArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().instructions_capture_camera(&a.step)?;
    Ok(Outcome::Done(state(ctx)))
}

fn instructions_reset_view(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: StepArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().instructions_reset_view(&a.step)?;
    Ok(Outcome::Done(state(ctx)))
}

fn instructions_show_step(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: StepArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().instructions_show_step(&a.step)?;
    Ok(Outcome::Done(state(ctx)))
}

fn instructions_update_step(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: UpdateStepArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().instructions_edit(None, |block| edit::update_step(block, &a.step, &a.patch))?;
    Ok(Outcome::Done(state(ctx)))
}

fn instructions_add_field(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AddFieldArgs = parse_args(args)?;
    let id = ctx.app.docs.engine_mut().instructions_edit(None, |block| edit::add_field(block, &a.step, &a.field, a.index))?;
    let mut out = state(ctx);
    out["field"] = json!(id);
    Ok(Outcome::Done(out))
}

fn instructions_update_field(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: UpdateFieldArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().instructions_edit(None, |block| edit::update_field(block, &a.step, &a.field, &a.patch))?;
    Ok(Outcome::Done(state(ctx)))
}

fn instructions_remove_field(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: FieldArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().instructions_edit(None, |block| edit::remove_field(block, &a.step, &a.field).map(drop))?;
    Ok(Outcome::Done(state(ctx)))
}

fn instructions_move_field(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: MoveFieldArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().instructions_edit(None, |block| edit::move_field(block, &a.step, &a.field, a.index))?;
    Ok(Outcome::Done(state(ctx)))
}

fn instructions_merge(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: MergeArgs = parse_args(args)?;
    let ids: Vec<&str> = a.steps.iter().map(String::as_str).collect();
    let kept = ctx.app.docs.engine_mut().instructions_restructure(|block| edit::merge(block, &ids))?;
    let mut out = state(ctx);
    out["step"] = json!(kept);
    Ok(Outcome::Done(out))
}

fn instructions_split(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: SplitArgs = parse_args(args)?;
    let new = ctx.app.docs.engine_mut().instructions_restructure(|block| edit::split(block, &a.step, &a.adds))?;
    let mut out = state(ctx);
    out["step"] = json!(new);
    Ok(Outcome::Done(out))
}

fn instructions_move_step(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: MoveStepArgs = parse_args(args)?;
    match a.parent.as_deref() {
        None => ctx.app.docs.engine_mut().instructions_restructure(|block| edit::move_step(block, &a.step, a.index))?,
        Some("") => ctx.app.docs.engine_mut().instructions_nest_step(&a.step, None, a.index)?,
        Some(parent) => ctx.app.docs.engine_mut().instructions_nest_step(&a.step, Some(parent), a.index)?,
    }
    Ok(Outcome::Done(state(ctx)))
}

fn instructions_add_parts(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AddPartsArgs = parse_args(args)?;
    let report = match (a.occurrences, a.part) {
        (Some(occurrences), None) => ctx.app.docs.engine_mut().instructions_add_parts(&a.step, &occurrences)?,
        (None, Some(part)) => ctx.app.docs.engine_mut().instructions_add_part(&a.step, &part)?,
        _ => return Err("give `occurrences` or `part`, one of them".into()),
    };
    let mut out = state(ctx);
    out["added"] = json!(report.added);
    out["left"] = serde_json::to_value(&report.left).unwrap_or(Value::Null);
    Ok(Outcome::Done(out))
}

fn instructions_remove_add(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: RemoveAddArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().instructions_remove_add(&a.step, a.add)?;
    Ok(Outcome::Done(state(ctx)))
}

fn instructions_move_add(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: MoveAddArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().instructions_restructure(|block| edit::move_add(block, &a.from, a.add, &a.to))?;
    Ok(Outcome::Done(state(ctx)))
}

fn instructions_delete_step(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: StepArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().instructions_restructure(|block| edit::delete_step(block, &a.step))?;
    Ok(Outcome::Done(state(ctx)))
}

fn instructions_play(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    use brep_render::engine_state::PlayTo;
    let a: PlayArgs = parse_args(args)?;
    let mode = match a.mode.as_deref() {
        Some(text) => Some(brep_instructions::player::Mode::parse(text).ok_or_else(|| format!("mode '{text}': shop or customer"))?),
        None => None,
    };
    let to = match (a.to.as_deref(), a.step) {
        (None, None) => None,
        (Some("first"), None) => Some(PlayTo::First),
        (Some("next"), None) => Some(PlayTo::Next),
        (Some("back"), None) => Some(PlayTo::Back),
        (Some("replay"), None) => Some(PlayTo::Replay),
        (Some("step") | None, Some(step)) => Some(PlayTo::Step(step)),
        (Some("step"), None) => return Err("to: step needs `step`".into()),
        (Some(other), _) => return Err(format!("to '{other}': first, next, back, replay or step")),
    };
    if let Some(to) = to {
        ctx.app.docs.engine_mut().instructions_play(to)?;
    }
    if let Some(fraction) = a.hold {
        ctx.app.docs.engine_mut().instructions_hold_insertion(fraction);
    }
    if let Some(mode) = mode {
        ctx.app.instructions_player.set_mode(mode);
    }
    let mut out = state(ctx);
    out["mode"] = json!(ctx.app.instructions_player.mode().as_str());
    Ok(Outcome::Done(out))
}

fn instructions_new_step(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: NewStepArgs = parse_args(args)?;
    let new = ctx.app.docs.engine_mut().instructions_new_step(a.after.as_deref())?;
    let mut out = state(ctx);
    out["step"] = json!(new);
    Ok(Outcome::Done(out))
}

/// Overlay `patch`'s keys onto `current`'s JSON and read it back.
fn overlay<T: serde::Serialize + serde::de::DeserializeOwned>(current: &T, patch: &Value, what: &str) -> Result<T, String> {
    let object = patch.as_object().ok_or_else(|| format!("{what} is a JSON object"))?;
    let mut value = serde_json::to_value(current).map_err(|e| e.to_string())?;
    for (key, field) in object {
        if value.get(key).is_none() && !matches!(key.as_str(), "coverStep") {
            return Err(format!("{what} has no field '{key}'"));
        }
        value[key] = field.clone();
    }
    serde_json::from_value(value).map_err(|e| format!("{what}: {e}"))
}

fn instructions_update(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: UpdateArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().instructions_edit(None, |block| {
        if let Some(template) = &a.template {
            block.template = overlay(&block.template, template, "the template")?;
        }
        if let Some(front) = &a.front {
            block.front = overlay(&block.front, front, "the front matter")?;
        }
        Ok(())
    })?;
    Ok(Outcome::Done(state(ctx)))
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ExportArgs {
    /// `pdf` (the printed instructions), `png` or `svg` (one step's picture),
    /// `images` (every step's PNG and SVG in one zip) or `html` (the offline
    /// player).
    pub format: String,
    /// The step a `png` or `svg` shows.
    #[serde(default)]
    pub step: Option<String>,
    /// A `png`'s edge in pixels (default 800).
    #[serde(default)]
    pub size: Option<u32>,
    /// The `html` player's mode: `shop` (default) or `customer`.
    #[serde(default)]
    pub mode: Option<String>,
}

fn instructions_export(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ExportArgs = parse_args(args)?;
    // The drawn formats need the exact solids. They are ASKED for here and
    // the export finishes on the frame they have all landed — the frame never
    // waits for the runner; a reply that has not come within the engine's
    // wait is said rather than drawn around.
    if matches!(a.format.as_str(), "pdf" | "svg" | "images") && !ctx.app.docs.engine_mut().instructions_request_topology() {
        let deadline = web_time::Instant::now() + brep_render::engine_state::TOPOLOGY_WAIT;
        return Ok(Outcome::Poll(Box::new(move |ctx: &mut Ctx<'_>| {
            if !ctx.app.docs.engine().topology_pending() {
                Some(export_now(ctx, &a))
            } else if web_time::Instant::now() >= deadline {
                Some(Err(format!(
                    "the exact geometry is still loading after {} s: wait_idle, then export again",
                    brep_render::engine_state::TOPOLOGY_WAIT.as_secs()
                )))
            } else {
                None
            }
        })));
    }
    export_now(ctx, &a)
}

fn export_now(ctx: &mut Ctx<'_>, a: &ExportArgs) -> Result<Outcome, String> {
    use crate::automation::cmd_document::base64_encode;
    let engine = ctx.app.docs.engine_mut();
    let step = || a.step.clone().ok_or_else(|| format!("format {}: name the `step`", a.format));
    let out = match a.format.as_str() {
        "pdf" => {
            let (pdf, pages) = engine.instructions_pdf()?;
            json!({ "format": "pdf", "bytes": pdf.len(), "base64": base64_encode(&pdf), "pages": pages,
                    "hiddenLine": brep_render::sheets::pdf::hidden_line_of(&pdf),
                    "texts": brep_render::sheets::pdf::text_strings(&pdf) })
        }
        "png" => {
            let (step, size) = (step()?, a.size.unwrap_or(brep_render::engine_state::INSTRUCTIONS_IMAGE_PX));
            if !(16..=4096).contains(&size) {
                return Err(format!("size {size}: 16 to 4096 pixels"));
            }
            let png = engine.instructions_step_png(&step, size)?;
            json!({ "format": "png", "step": step, "bytes": png.len(), "base64": base64_encode(&png), "width": size, "height": size })
        }
        "svg" => {
            let step = step()?;
            let svg = engine.instructions_step_svg(&step)?;
            json!({ "format": "svg", "step": step, "bytes": svg.len(), "hiddenLine": brep_render::sheets::svg::hidden_lines(&svg).first(),
                    "texts": brep_render::sheets::svg::text_strings(&svg), "text": svg })
        }
        "images" => {
            let zip = crate::panels::instructions::images_zip(engine)?;
            let files: Vec<Value> = brep_ecad_core::fabrication::zip_entries(&zip)
                .into_iter()
                .map(|(name, bytes)| json!({ "name": name, "bytes": bytes }))
                .collect();
            json!({ "format": "images", "bytes": zip.len(), "base64": base64_encode(&zip), "files": files })
        }
        "html" => {
            let mode = match a.mode.as_deref() {
                None => brep_instructions::player::Mode::Shop,
                Some(text) => brep_instructions::player::Mode::parse(text).ok_or_else(|| format!("mode '{text}': shop or customer"))?,
            };
            // The tools' pictures: what the library answered so far is inlined,
            // the rest asked for (a script waits idle and exports again).
            let mut data = engine.instructions_player_data()?;
            let ids = crate::panels::file::linked_tool_ids(ctx.app.docs.engine());
            if let Some(client) = ctx.app.model_store().plm_client().filter(|_| !ids.is_empty()) {
                crate::plm::tool_pictures::request(&client, &ids);
            }
            let inlined = data.inline_tool_pictures(&crate::plm::tool_pictures::fetched(&ids));
            let pending = crate::plm::tool_pictures::pending_of(&ids);
            let html = brep_instructions::player::offline_html(&data, mode);
            let steps: Vec<Value> = data
                .steps
                .iter()
                .map(|step| {
                    json!({ "id": step.id, "number": step.number, "parent": step.parent, "depth": step.depth, "title": step.title, "placed": step.placed.len(), "adding": step.adding.len(),
                            "insertion": step.insertion.len(), "camera": step.camera.is_some(), "image": step.image.is_some(),
                            "thumbnails": step.adds.iter().filter(|add| add.thumbnail.is_some()).count(),
                            "pmi": step.pmi.as_ref().map_or(0, |pmi| pmi.annotations), "fields": step.fields.len() })
                })
                .collect();
            json!({ "format": "html", "mode": mode.as_str(), "bytes": html.len(), "title": data.title, "meshes": data.meshes.len(),
                    "toolPictures": inlined, "toolPicturesPending": pending, "toolPicturesMissing": ids.len().saturating_sub(inlined + pending),
                    "edges": data.meshes.iter().filter(|mesh| !mesh.edges.is_empty()).count(),
                    "nodes": data.nodes.len(), "steps": steps,
                    "assembly": data.assembly.len(), "assemblyThumbnails": data.assembly.iter().filter(|row| row.thumbnail.is_some()).count(),
                    "selfContained": !html.contains("{{") && !html.contains("://") })
        }
        other => return Err(format!("format '{other}': pdf, png, svg, images or html")),
    };
    Ok(Outcome::Done(out))
}

// The state/edit replies share the engine's state serializer.
// Keep its recursive authored DTO definitions at the result root so local
// references remain valid in both native metadata and MCP composition.
fn instructions_object(properties: Value) -> Value {
    let required: Vec<_> = properties.as_object().unwrap().keys().cloned().collect();
    json!({"type":"object","properties":properties,"required":required})
}

fn instructions_derived<T: schemars::JsonSchema>(defs: &mut Value) -> Value {
    let mut schema = schema_of::<T>();
    let root = schema.as_object_mut().unwrap();
    root.remove("$schema");
    if let Some(Value::Object(nested)) = root.remove("$defs") {
        for (name, definition) in nested {
            if let Some(existing) = defs.get(&name) { assert_eq!(existing, &definition); }
            defs[&name] = definition;
        }
    }
    schema
}

fn instructions_result_schema() -> Value {
    let mut defs = json!({});
    let block = instructions_derived::<brep_instructions::Instructions>(&mut defs);
    let strings = json!({"type":"array","items":{"type":"string"}});
    let populated = instructions_object(json!({
        "instructions":block,"steps":{"type":"integer","minimum":0},
        "wizardPending":{"type":"boolean"},"viewsPending":{"type":"boolean"},
        "shownStep":{"type":["string","null"]},"ghosted":strings,
        "insertion":{"type":["number","null"]},"emptySteps":strings,
        "unplanned":{"type":"array","items":instructions_object(json!({"occurrence":{"type":"string"},"part":{"type":"string"}}))},
        "wrapper":{"anyOf":[{"type":"null"},instructions_object(json!({"id":{"type":"string"},"part":{"type":"string"}}))]}
    }));
    // A parse error is returned as a successful state snapshot with an error
    // string; an ordinary non-instructions document omits that field.
    let absent = json!({"type":"object","properties":{"instructions":{"type":"null"},"error":{"type":"string"}},"required":["instructions"]});
    json!({"type":"object","oneOf":[absent,populated],"$defs":defs})
}

fn instructions_extended(properties: Value) -> Value {
    let mut result = instructions_result_schema();
    for branch in result["oneOf"].as_array_mut().unwrap() {
        for (name,schema) in properties.as_object().unwrap() {
            branch["properties"][name] = schema.clone();
            branch["required"].as_array_mut().unwrap().push(json!(name));
        }
    }
    result
}
fn instructions_new_result_schema() -> Value {
    instructions_extended(json!({"index":{"type":"integer","minimum":0}}))
}
fn instructions_step_result_schema() -> Value {
    instructions_extended(json!({"step":{"type":"string"}}))
}
fn instructions_field_result_schema() -> Value {
    instructions_extended(json!({"field":{"type":"string"}}))
}
fn instructions_play_result_schema() -> Value {
    instructions_extended(json!({"mode":{"enum":["shop","customer"]}}))
}
fn instructions_add_parts_result_schema() -> Value {
    let mut defs = json!({});
    let left = instructions_derived::<Option<Vec<edit::LeftOccurrence>>>(&mut defs);
    let mut result = instructions_extended(json!({"added":{"type":"integer","minimum":0},"left":left}));
    for (name, definition) in defs.as_object().unwrap() {
        if let Some(existing) = result["$defs"].get(name) { assert_eq!(existing, definition); }
        result["$defs"][name] = definition.clone();
    }
    result
}

fn instructions_export_result_schema() -> Value {
    let count = json!({"type":"integer","minimum":0});
    let string = json!({"type":"string"});
    let strings = json!({"type":"array","items":string});
    let hidden_line = json!({"type":["string","null"]});
    let pdf = instructions_object(json!({
        "format":{"const":"pdf"},"bytes":count,"base64":string,"pages":count,
        "hiddenLine":hidden_line,"texts":strings
    }));
    let png = instructions_object(json!({
        "format":{"const":"png"},"step":string,"bytes":count,"base64":string,
        "width":{"type":"integer","minimum":16,"maximum":4096},
        "height":{"type":"integer","minimum":16,"maximum":4096}
    }));
    let svg = instructions_object(json!({
        "format":{"const":"svg"},"step":string,"bytes":count,"hiddenLine":hidden_line,
        "texts":strings,"text":string
    }));
    let images = instructions_object(json!({
        "format":{"const":"images"},"bytes":count,"base64":string,
        "files":{"type":"array","items":instructions_object(json!({"name":string,"bytes":count}))}
    }));
    let html = instructions_object(json!({
        "format":{"const":"html"},"mode":{"enum":["shop","customer"]},"bytes":count,
        "title":string,"meshes":count,"toolPictures":count,"toolPicturesPending":count,
        "toolPicturesMissing":count,"edges":count,"nodes":count,"selfContained":{"type":"boolean"},
        "steps":{"type":"array","items":instructions_object(json!({
            "id":string,"number":count,"parent":{"type":["string","null"]},"depth":count,
            "title":string,"placed":count,"adding":count,"insertion":count,
            "camera":{"type":"boolean"},"image":{"type":"boolean"},"thumbnails":count,"pmi":count,"fields":count
        }))}
    }));
    json!({"type":"object","oneOf":[pdf,png,svg,images,html]})
}

pub static COMMANDS: &[CommandSpec] = &[
    app_command!("instructions", "instructions_state", "The active document's manufacturing instructions: `instructions` (the block: target assembly revision, template, front matter, the step tree `steps` where each step holds its child `steps` with their adds, view, tools, torque, callouts, arrows, notes, sign-off and `fields`; a pending `retarget`), `steps` (the count), `shownStep` (the step whose view is active), `ghosted` (the parts drawn as translucent ghosts: in a Shaded document, the parts earlier steps placed; empty in a Line art document, which keeps them in the scene drawn flat white with grey lines), `insertion` (the player's insertion fraction while the step's parts move home, else null), `emptySteps`, `unplanned` (the assembly's parts in no step: occurrence and part) and `wrapper` (the component placing the assembly). `instructions: null` on any other document.", NoArgs, instructions_state, Read, READ, instructions_result_schema),
    app_command!("instructions", "instructions_new", "Open a new tab holding manufacturing instructions for an assembly: `from` a store key, else the active (saved) document. The assembly is placed as one grounded component and its key is the instructions' target. With `wizard` (default true) the wizard proposes the step tree: a sub-assembly is one step whose children build it (read before it), then mate order (or an exploded view's reverse order), fasteners last, each step with its own view; it runs once the assembly is built, so `wizardPending` may be true in the reply (wait_idle, then instructions_state). Returns the `instructions_state` plus the tab `index`. Refused for a part, an unsaved document or instructions.", NewArgs, instructions_new, Mutate, MUTATE, instructions_new_result_schema),
    app_command!("instructions", "instructions_wizard", "Rerun the wizard on the instructions' assembly: the step tree is replaced (the target, template and front matter stay) and the old steps' views are removed. One undo step. Returns the `instructions_state`.", NoArgs, instructions_wizard, Mutate, MUTATE, instructions_result_schema),
    app_command!("instructions", "instructions_retarget_approve", "Approve the pending re-target (`instructions.retarget` in the state). When Update components (component_update) or a linked tab brings the instructions a new state of their assembly, the wizard reruns on the changed parts only, in the same undo step: an added part gets a step after its mated neighbour's, a removed one leaves its step, a replaced one takes its new part, and the change is listed (added, removed, replaced, touchedSteps, emptySteps) until approved. Approving makes the new revision the target and deletes the steps it left empty. Refused when none waits; a further update is refused until it is approved. One undo step. Returns the `instructions_state`.", NoArgs, instructions_retarget_approve, Mutate, MUTATE, instructions_result_schema),
    app_command!("instructions", "instructions_retarget_reject", "Reject the pending re-target (`instructions.retarget` in the state): the one undo that takes the new revision of the assembly and its re-target back together, as Undo does, so the instructions keep describing the revision they did. Refused when none waits, when the re-target came in with the file (nothing in this session takes it back: approve it, or update again later), and when edits followed it (they would be undone too: undo them first, or approve). Leaves a redo step, as the undo would. Returns the `instructions_state`.", NoArgs, instructions_retarget_reject, Mutate, MUTATE, instructions_result_schema),
    app_command!("instructions", "instructions_show_step", "Show a step's view: its camera, the parts not placed yet hidden, and the balloons on the parts it adds. Refused for a step without a view.", StepArgs, instructions_show_step, Mutate, MUTATE, instructions_result_schema),
    app_command!("instructions", "instructions_update_step", "Set a step's fields from `patch` (title, notes, view, tools, torque, callouts, arrows, signoff, motion, fields: the whole list of step fields). Unknown fields and invalid values are refused with nothing written; parts move only through merge, split and instructions_move_add.", UpdateStepArgs, instructions_update_step, Mutate, MUTATE, instructions_result_schema),
    app_command!("instructions", "instructions_add_field", "Add a field to a step: something the person doing the work records there (any number per step, in order). `field` is `{label, kind, required?, hint?}` plus the kind's keys: `check` (one box to tick), `choice` (`options`, `multiple?`, `other?`: a dropdown in the players, a box per option on paper), `text` (`lines?` 1 to 20: one line or a box), `number` (a measurement: `unit?`, `nominal?`, `min?`, `max?`, `decimals?`, `requiredInLimits?` default true), `serial` (a serial number not tied to a part). The players show the fields in shop mode and keep the values as the build's record (never the document's); a required field left empty, or a measurement outside limits that must be inside, keeps the step from being signed off or marked done; the printed pages draw them as a paper form in the step's Record block. A step with fields is on a build's checklist even when its sign-off is not required. `index` places it (the end when absent). One undo step; a malformed field or a missing step is refused with nothing written. Returns the state plus the new `field` id (`MF…`).", AddFieldArgs, instructions_add_field, Mutate, MUTATE, instructions_field_result_schema),
    app_command!("instructions", "instructions_update_field", "Set keys of one step field (by its id) from `patch`: label, required, hint, and the kind's own (options, multiple, other; lines; unit, nominal, min, max, decimals, requiredInLimits). `null` clears an optional key. A new `kind` keeps the id, label, hint and `required` and takes that kind's defaults. Unknown keys, an id change and values the field cannot hold (no options, min above max, more than 20 lines) are refused with nothing written. One undo step. Returns the state.", UpdateFieldArgs, instructions_update_field, Mutate, MUTATE, instructions_result_schema),
    app_command!("instructions", "instructions_remove_field", "Take one field (by its id) off its step. Its id is not handed out again. One undo step. Returns the state.", FieldArgs, instructions_remove_field, Mutate, MUTATE, instructions_result_schema),
    app_command!("instructions", "instructions_move_field", "Move one field (by its id) to `index` among its step's fields (clamped to the end): the order the players and the printed Record block list them in. One undo step. Returns the state.", MoveFieldArgs, instructions_move_field, Mutate, MUTATE, instructions_result_schema),
    app_command!("instructions", "instructions_merge", "Merge two or more sibling steps (steps under one parent) into the first of them in their order (it keeps its place; parts, tools, callouts, arrows and child steps are appended, notes joined; parts of one BOM row become one row with the quantity). Returns the state plus the kept `step`.", MergeArgs, instructions_merge, Mutate, MUTATE, instructions_step_result_schema),
    app_command!("instructions", "instructions_split", "Split the parts at `adds` (indices) out of a step into a new step right after it, with the arrows and callouts that point at them. At least one part must stay. Returns the state plus the new `step`.", SplitArgs, instructions_split, Mutate, MUTATE, instructions_step_result_schema),
    app_command!("instructions", "instructions_capture_camera", "Capture the viewport's camera as a step's view camera (point the camera first with `camera_set` / `standard_view`, or orbit the shown step). It is the author's from then on: every edit keeps it, until instructions_reset_view; `cameraCaptured` on the step says so. The PMI panel's Update camera on a step's view does the same. One undo step. Returns the state.", StepArgs, instructions_capture_camera, Mutate, MUTATE, instructions_result_schema),
    app_command!("instructions", "instructions_reset_view", "Drop a step's captured camera: its view's camera is generated again, framing everything the step shows (the parts it adds, its children's included, and the ghosts placed before it) from its parent step's camera direction, or the isometric (orthographic) at the top level, fitted in a fixed 1280×800 viewport so it is the same from any window. One undo step. Returns the state.", StepArgs, instructions_reset_view, Mutate, MUTATE, instructions_result_schema),
    app_command!("instructions", "instructions_add_parts", "A BOM row dragged into a step: add `occurrences` of the assembly (or every occurrence of `part` at the assembly's top level, what a packed BOM row stands for) to `step`. An occurrence already in a step, or inside a sub-assembly another step places, is left where it is and reported in `left` (with that step), unless `step` is inside that sub-assembly's group; `added` counts the rest. One BOM row per part, a fastener bringing its torque when the step has none. An occurrence the assembly lacks or a missing step is refused with nothing written. The step views follow in the same undo step. Returns the state plus `added` and `left`; `unplanned` in the state lists the assembly's parts in no step.", AddPartsArgs, instructions_add_parts, Mutate, MUTATE, instructions_add_parts_result_schema),
    app_command!("instructions", "instructions_remove_add", "Take a part (by its index in `step`'s adds) out of its step: it is in no step then (`unplanned` in the state), its insertion arrows and the callouts pointing at it go with it, and a torque that came from it is dropped. One undo step. Returns the state.", RemoveAddArgs, instructions_remove_add, Mutate, MUTATE, instructions_result_schema),
    app_command!("instructions", "instructions_move_step", "Move a step to `index` among its siblings, or, with `parent`, under that step (its children with it; `\"\"` is the top level): a step cannot go inside itself, and its id never changes. Like every edit that changes which step adds what (merge, split, new step, move a part, delete), the step views follow: the parts hidden and the balloons are made again, a generated camera is made again for the step's parts, a captured camera (instructions_capture_camera) and the author's own annotations stay.", MoveStepArgs, instructions_move_step, Mutate, MUTATE, instructions_result_schema),
    app_command!("instructions", "instructions_move_add", "Move one part (by its index in `from`'s adds) to a sibling step `to` (one under the same parent); the step it leaves may be left empty (listed in `emptySteps`).", MoveAddArgs, instructions_move_add, Mutate, MUTATE, instructions_result_schema),
    app_command!("instructions", "instructions_play", "The in-app player: go `to` first, next, back, replay or a `step` (its view shown, the parts earlier steps placed ghosted in a Shaded document or drawn flat white in a Line art one, its arrowed parts moving home along black arrows (a Line art step without arrows of its own slides its parts down the view's up): `insertion` in the state is the fraction until they arrive, then null; `hold` keeps it at a fraction), and/or set the Player pane's `mode` (shop, customer). Next on the last step and Back on the first stay put. Returns the `instructions_state` plus the `mode`.", PlayArgs, instructions_play, Mutate, MUTATE, instructions_play_result_schema),
    app_command!("instructions", "instructions_new_step", "Insert an empty step `after` a step (as its next sibling), or at the end of the top level, with its own view. Parts come to it by instructions_move_add. Returns the state plus the new `step`.", NewStepArgs, instructions_new_step, Mutate, MUTATE, instructions_step_result_schema),
    app_command!("instructions", "instructions_delete_step", "Delete an empty step. A step that still adds parts is refused: move them first.", StepArgs, instructions_delete_step, Mutate, MUTATE, instructions_result_schema),
    app_command!("instructions", "instructions_export", "Write the instructions out, as File ▸ Export does: `pdf` (the cover, the parts list, the tools list and the steps on the page template, each step's fields printed under its text as a paper form in a Record block: a box per tick and per choice option, ruled lines for text, measurements and serial numbers, Signed and Date lines where a sign-off is required, continued on Record pages when a slot is full; each step's view drawn as a drawing sheet draws a PMI view, hidden lines removed by the exact pass over the solids' B-reps, which the export asks the runner for first and waits for; `pages`, `hiddenLine` (`analytic`, or `mesh: ` naming the pictures a B-rep that never came left to the mesh approximation) and every shown string in `texts`, read back from the bytes), `png` (one `step`'s picture from its view's camera, `size` pixels square, default 800: in Line art (the template's `style`, the default for a new document) a flat illustration on light grey, what the step adds flat blue under a black outline, what earlier steps placed flat white with grey lines, and a black arrow from where each sliding part starts; in Shaded, what it adds in its colours, what earlier steps placed in grey), `svg` (one `step`'s number and title over its view as lines and text; `hiddenLine` and `texts` read back), `images` (every step's PNG and SVG in one zip, `step-01-MS2.png` …, listed in `files`), or `html` (the offline player: one self-contained page in `mode` shop or customer, the meshes, step cameras, step pictures, part thumbnails and the linked tools' pictures inlined as data:image (`toolPictures` the count inlined, `toolPicturesPending` those still being fetched from the PLM tools library, `toolPicturesMissing` those the library has none of or no session can fetch; wait_idle and export again for the pending ones); returns its `bytes`, `meshes`, `edges` (meshes carrying their edge lines), `nodes`, `selfContained` (no placeholder left, no URL to fetch) and per step what it places, adds and moves home and how many `fields` it records; in shop mode the page shows each step's fields as inputs and keeps what is entered, with the part ticks, serials and done marks, in the browser's localStorage under the document's own key, with Export record (JSON, CSV) and Clear this build). Binary files come back in `base64`. What the viewport shows (a step in the player, its ghosts) changes no file.", ExportArgs, instructions_export, Read, READ, instructions_export_result_schema),
    app_command!("instructions", "instructions_update", "Set the page template (`template`: stepsPerPage 1, 2 or 4, orientation portrait or landscape, paper A4, A3, Letter, …, style lineArt (the default for a new document: flat white placed parts with grey lines, the step's parts flat blue under a black outline sliding in along black arrows, on light grey) or shaded (the earlier lit look; what a document saved without a style reads as)) and the front matter (`front`: title, subtitle, coverStep, partsList, toolsList). Given keys replace the current ones; unknown keys are refused.", UpdateArgs, instructions_update, Mutate, MUTATE, instructions_result_schema),
];
