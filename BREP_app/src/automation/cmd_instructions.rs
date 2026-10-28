//! Manufacturing instructions (`brep_render::engine_state` `instructions_ops`,
//! `brep_instructions`): make instructions from an assembly, run the wizard,
//! and edit, merge, split and reorder steps. Every edit is one undo step and
//! is refused, with nothing written, when the block would not validate. An
//! edit that changes which step adds what also brings the step views up to
//! date (hidden parts, balloons), keeping each step's camera.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Outcome, Phase};
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
    /// (`{required}`), `motion`.
    pub patch: Value,
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
    /// portrait|landscape, paper}`; given keys replace the current ones.
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
    use crate::automation::cmd_document::base64_encode;
    let a: ExportArgs = parse_args(args)?;
    let engine = ctx.app.docs.engine_mut();
    let step = || a.step.clone().ok_or_else(|| format!("format {}: name the `step`", a.format));
    // The drawn formats wait for the exact solids; a reply that has not come
    // within the engine's wait is said rather than drawn around.
    if matches!(a.format.as_str(), "pdf" | "svg" | "images") && !engine.instructions_topology_ready() {
        return Err(format!(
            "the exact geometry is still loading after {} s: wait_idle, then export again",
            brep_render::engine_state::TOPOLOGY_WAIT.as_secs()
        ));
    }
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
                            "thumbnails": step.adds.iter().filter(|add| add.thumbnail.is_some()).count() })
                })
                .collect();
            json!({ "format": "html", "mode": mode.as_str(), "bytes": html.len(), "title": data.title, "meshes": data.meshes.len(),
                    "toolPictures": inlined, "toolPicturesPending": pending, "toolPicturesMissing": ids.len().saturating_sub(inlined + pending),
                    "edges": data.meshes.iter().filter(|mesh| !mesh.edges.is_empty()).count(),
                    "nodes": data.nodes.len(), "steps": steps, "selfContained": !html.contains("{{") && !html.contains("://") })
        }
        other => return Err(format!("format '{other}': pdf, png, svg, images or html")),
    };
    Ok(Outcome::Done(out))
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "instructions_state", group: "instructions", doc: "The active document's manufacturing instructions: `instructions` (the block: target assembly revision, template, front matter, the step tree `steps` where each step holds its child `steps` with their adds, view, tools, torque, callouts, arrows, notes, sign-off; a pending `retarget`), `steps` (the count), `shownStep` (the step whose view is active), `ghosted` (the parts drawn as ghosts), `insertion` (the player's insertion fraction while the step's parts move home, else null), `emptySteps`, `unplanned` (the assembly's parts in no step: occurrence and part) and `wrapper` (the component placing the assembly). `instructions: null` on any other document.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_state) },
    CommandSpec { name: "instructions_new", group: "instructions", doc: "Open a new tab holding manufacturing instructions for an assembly: `from` a store key, else the active (saved) document. The assembly is placed as one grounded component and its key is the instructions' target. With `wizard` (default true) the wizard proposes the step tree: a sub-assembly is one step whose children build it (read before it), then mate order (or an exploded view's reverse order), fasteners last, each step with its own view; it runs once the assembly is built, so `wizardPending` may be true in the reply (wait_idle, then instructions_state). Returns the `instructions_state` plus the tab `index`. Refused for a part, an unsaved document or instructions.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<NewArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_new) },
    CommandSpec { name: "instructions_wizard", group: "instructions", doc: "Rerun the wizard on the instructions' assembly: the step tree is replaced (the target, template and front matter stay) and the old steps' views are removed. One undo step. Returns the `instructions_state`.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_wizard) },
    CommandSpec { name: "instructions_retarget_approve", group: "instructions", doc: "Approve the pending re-target (`instructions.retarget` in the state). When Update components (component_update) or a linked tab brings the instructions a new state of their assembly, the wizard reruns on the changed parts only, in the same undo step: an added part gets a step after its mated neighbour's, a removed one leaves its step, a replaced one takes its new part, and the change is listed (added, removed, replaced, touchedSteps, emptySteps) until approved. Approving makes the new revision the target and deletes the steps it left empty. Refused when none waits; a further update is refused until it is approved. One undo step. Returns the `instructions_state`.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_retarget_approve) },
    CommandSpec { name: "instructions_retarget_reject", group: "instructions", doc: "Reject the pending re-target (`instructions.retarget` in the state): the one undo that takes the new revision of the assembly and its re-target back together, as Undo does, so the instructions keep describing the revision they did. Refused when none waits, when the re-target came in with the file (nothing in this session takes it back: approve it, or update again later), and when edits followed it (they would be undone too: undo them first, or approve). Leaves a redo step, as the undo would. Returns the `instructions_state`.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_retarget_reject) },
    CommandSpec { name: "instructions_show_step", group: "instructions", doc: "Show a step's view: its camera, the parts not placed yet hidden, and the balloons on the parts it adds. Refused for a step without a view.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<StepArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_show_step) },
    CommandSpec { name: "instructions_update_step", group: "instructions", doc: "Set a step's fields from `patch` (title, notes, view, tools, torque, callouts, arrows, signoff, motion). Unknown fields and invalid values are refused with nothing written; parts move only through merge, split and instructions_move_add.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<UpdateStepArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_update_step) },
    CommandSpec { name: "instructions_merge", group: "instructions", doc: "Merge two or more sibling steps (steps under one parent) into the first of them in their order (it keeps its place; parts, tools, callouts, arrows and child steps are appended, notes joined; parts of one BOM row become one row with the quantity). Returns the state plus the kept `step`.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<MergeArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_merge) },
    CommandSpec { name: "instructions_split", group: "instructions", doc: "Split the parts at `adds` (indices) out of a step into a new step right after it, with the arrows and callouts that point at them. At least one part must stay. Returns the state plus the new `step`.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<SplitArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_split) },
    CommandSpec { name: "instructions_capture_camera", group: "instructions", doc: "Capture the viewport's camera as a step's view camera (point the camera first with `camera_set` / `standard_view`, or orbit the shown step). It is the author's from then on: every edit keeps it, until instructions_reset_view; `cameraCaptured` on the step says so. The PMI workbench's Update camera on a step's view does the same. One undo step. Returns the state.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<StepArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_capture_camera) },
    CommandSpec { name: "instructions_reset_view", group: "instructions", doc: "Drop a step's captured camera: its view's camera is generated again, framing everything the step shows (the parts it adds, its children's included, and the ghosts placed before it) from its parent step's camera direction, or the isometric (orthographic) at the top level, fitted in a fixed 1280×800 viewport so it is the same from any window. One undo step. Returns the state.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<StepArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_reset_view) },
    CommandSpec { name: "instructions_add_parts", group: "instructions", doc: "A BOM row dragged into a step: add `occurrences` of the assembly (or every occurrence of `part` at the assembly's top level, what a packed BOM row stands for) to `step`. An occurrence already in a step, or inside a sub-assembly another step places, is left where it is and reported in `left` (with that step), unless `step` is inside that sub-assembly's group; `added` counts the rest. One BOM row per part, a fastener bringing its torque when the step has none. An occurrence the assembly lacks or a missing step is refused with nothing written. The step views follow in the same undo step. Returns the state plus `added` and `left`; `unplanned` in the state lists the assembly's parts in no step.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<AddPartsArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_add_parts) },
    CommandSpec { name: "instructions_remove_add", group: "instructions", doc: "Take a part (by its index in `step`'s adds) out of its step: it is in no step then (`unplanned` in the state), its insertion arrows and the callouts pointing at it go with it, and a torque that came from it is dropped. One undo step. Returns the state.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<RemoveAddArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_remove_add) },
    CommandSpec { name: "instructions_move_step", group: "instructions", doc: "Move a step to `index` among its siblings, or, with `parent`, under that step (its children with it; `\"\"` is the top level): a step cannot go inside itself, and its id never changes. Like every edit that changes which step adds what (merge, split, new step, move a part, delete), the step views follow: the parts hidden and the balloons are made again, a generated camera is made again for the step's parts, a captured camera (instructions_capture_camera) and the author's own annotations stay.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<MoveStepArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_move_step) },
    CommandSpec { name: "instructions_move_add", group: "instructions", doc: "Move one part (by its index in `from`'s adds) to a sibling step `to` (one under the same parent); the step it leaves may be left empty (listed in `emptySteps`).", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<MoveAddArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_move_add) },
    CommandSpec { name: "instructions_play", group: "instructions", doc: "The in-app player: go `to` first, next, back, replay or a `step` (its view shown, the parts earlier steps placed ghosted, its arrowed parts moving home: `insertion` in the state is the fraction until they arrive, then null), and/or set the Player pane's `mode` (shop, customer). Next on the last step and Back on the first stay put. Returns the `instructions_state` plus the `mode`.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<PlayArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_play) },
    CommandSpec { name: "instructions_new_step", group: "instructions", doc: "Insert an empty step `after` a step (as its next sibling), or at the end of the top level, with its own view. Parts come to it by instructions_move_add. Returns the state plus the new `step`.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<NewStepArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_new_step) },
    CommandSpec { name: "instructions_delete_step", group: "instructions", doc: "Delete an empty step. A step that still adds parts is refused: move them first.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<StepArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_delete_step) },
    CommandSpec { name: "instructions_export", group: "instructions", doc: "Write the instructions out, as File ▸ Export does: `pdf` (the cover, the parts list, the tools list and the steps on the page template, each step's view drawn as a drawing sheet draws a PMI view, hidden lines removed by the exact pass over the solids' B-reps, which the export asks the runner for first and waits for; `pages`, `hiddenLine` (`analytic`, or `mesh: ` naming the pictures a B-rep that never came left to the mesh approximation) and every shown string in `texts`, read back from the bytes), `png` (one `step`'s picture from its view's camera, `size` pixels square, default 800: what it adds in colour, what earlier steps placed in grey), `svg` (one `step`'s number and title over its view as lines and text; `hiddenLine` and `texts` read back), `images` (every step's PNG and SVG in one zip, `step-01-MS2.png` …, listed in `files`), or `html` (the offline player: one self-contained page in `mode` shop or customer, the meshes, step cameras, step pictures, part thumbnails and the linked tools' pictures inlined as data:image (`toolPictures` the count inlined, `toolPicturesPending` those still being fetched from the PLM tools library, `toolPicturesMissing` those the library has none of or no session can fetch; wait_idle and export again for the pending ones); returns its `bytes`, `meshes`, `edges` (meshes carrying their edge lines), `nodes`, `selfContained` (no placeholder left, no URL to fetch) and per step what it places, adds and moves home). Binary files come back in `base64`. What the viewport shows (a step in the player, its ghosts) changes no file.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<ExportArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_export) },
    CommandSpec { name: "instructions_update", group: "instructions", doc: "Set the page template (`template`: stepsPerPage 1, 2 or 4, orientation portrait or landscape, paper A4, A3, Letter, …) and the front matter (`front`: title, subtitle, coverStep, partsList, toolsList). Given keys replace the current ones; unknown keys are refused.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<UpdateArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_update) },
];
