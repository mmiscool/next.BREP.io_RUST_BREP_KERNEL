//! The CAM workbench: setups, tools, operations and NC programs in the
//! document's `cam` block (`document::cam`, on the shared `brep_cam::model`),
//! and the pane's selection. These commands are the operations `panels::cam`
//! performs, on the same helpers, so an agent authors the same document a
//! person does. Generation, dialogs and target picking arrive with their rows
//! (lane C1 rows 3-7); until then `cam_generate` says so rather than
//! succeeding at nothing.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Outcome, Phase};
use crate::automation::cmd_document::parse;
use crate::panels::cam;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct NamedArgs {
    /// The record's name; a default (`Setup N` / `Tool N`) is minted when absent.
    #[serde(default)]
    pub name: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddOperationArgs {
    /// The setup the operation goes into (`cam_state` lists them).
    pub setup: String,
    /// The tool it uses (`cam_state` lists them).
    pub tool: String,
    /// The strategy kind (`cam_state` lists them under `strategies`:
    /// `parallel`, `contour3D`, `pocket2D`, `drill`, …); every parameter
    /// starts at its default, which the operation dialog edits.
    pub strategy: String,
    /// The operation's name; `<strategy> N` when absent.
    #[serde(default)]
    pub name: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddProgramArgs {
    /// The program's name; `Program N` when absent.
    #[serde(default)]
    pub name: Option<String>,
    /// The post-processor's id (a built-in name or a plugin script id);
    /// empty takes the default post of the first operation's setup machine.
    #[serde(default)]
    pub post: String,
    /// The operations to post, in order; each must exist.
    #[serde(default)]
    pub operations: Vec<String>,
    /// Post property values by name (a built-in post's names and enum
    /// values are checked); absent ones take the post's defaults.
    #[serde(default)]
    pub properties: std::collections::BTreeMap<String, Value>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UpdateProgramArgs {
    /// The NC program (`NCn`).
    pub id: String,
    /// Post properties to set; a `null` value clears one back to the post's default.
    #[serde(default)]
    pub properties: std::collections::BTreeMap<String, Value>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct OpenArgs {
    /// The setup whose dialog to open; absent closes the open one.
    #[serde(default)]
    pub id: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UpdateSetupArgs {
    pub id: String,
    /// A patch of the setup form's keys (`name`, `wcsNumber`, `zRef` | `zDir` (a direction: `+X`, `-Y`, ... or
    /// `x,y,z`, for a tilted setup), `flipZ`, `xRef`, `originRef`,
    /// `stockKind` + `stockX`/`stockY`/`stockTop`/`stockBottom` | `stockMin`/`stockMax` |
    /// `stockBase`/`stockAxis`/`stockRadius`/`stockHeight` | `stockSolid` | `stockSetup` (`fromRest`: an earlier
    /// setup's simulated stock), `clearance`,
    /// `retract`, `feed`, `targets`, `protected`, `turningStickOut` (positive length or null)); absent keys are unchanged.
    pub patch: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UpdateOperationArgs {
    pub id: String,
    /// A patch of the operation form's keys, nested as the form has them (deep-merged; absent keys
    /// unchanged): `name`, `enabled`, `tool`, `liveTool` (true/false/null),
    /// `liveToolMode` (live/off/auto), `feedsOverride` + `feeds.{spindleRpm,cuttingFeed,plungeFeed}`,
    /// `targets.{targetSolids,driveFaces,avoidFaces,protectedSolids}` (entity names), `heightsOverride` +
    /// `heights.{clearance,retract,feed}`, `tolerance`, `stockToLeave.{radial,axial}`, `strategy.*` (the
    /// strategy's own parameters, by the model's field names) and `linking.*`.
    pub patch: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SetRefArgs {
    /// The setup or the operation.
    pub id: String,
    /// A setup's reference field (`zRef`, `xRef`, `originRef`, `stockSolid`, `targets`, `protected`) or an
    /// operation's (`targetSolids`, `driveFaces`, `avoidFaces`, `protectedSolids`, `chainEdges`, `chainProfile`, `boundaryProfile`).
    pub field: String,
    /// The entity names to pick (a face `Box_PZ`, an edge, a vertex `Box@8,0,0`, a solid);
    /// a single-valued field takes the first. Empty clears the field.
    #[serde(default)]
    pub names: Vec<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddMachineArgs {
    /// The machine to copy: a built-in id (`vmc-3axis`, ...; see `cam_state.machines`) or a document machine's id.
    pub base: String,
    /// The copy's name; absent = `<base name> (custom)`.
    #[serde(default)]
    pub name: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UpdateMachineArgs {
    pub id: String,
    /// A nested patch of the machine editor's keys (`name`, `kind`, `post`, `toolChangeSeconds`,
    /// `spindle.minRpm`/`maxRpm`, `feed.maxCutting`/`rapid`/`acceleration`, `accelerationSet`,
    /// `travelSet`, `travel.min`/`max`, `kinematics`, `yAxis`, `axes.<i>.name`/`direction`/
    /// `origin`/`limitsSet`/`limitsMin`/`limitsMax`); absent keys are unchanged.
    pub patch: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UpdateToolArgs {
    pub id: String,
    /// A patch of the tool form's keys (`name`, `number`, `kind`, `description`, `material`,
    /// `diameter`, `cuttingLength`, `shaftDiameter`, `shaftLength`, `overallLength`, `cornerRadius`,
    /// `includedAngleDeg`, `ballDiameter`, `tipDiameter`, `threadPitch`, `fluteCount`, `leftHand`,
    /// `holderFamily`/`holderBore`/`holderHeight`, `spindleRpm`, `spindleDirection`, `cuttingFeed`,
    /// `plungeFeed`, `coolant`, `turningGeometry` (tagged insert/blade geometry or null)); absent keys are unchanged.
    pub patch: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FeedsArgs {
    pub id: String,
    /// The work material (`aluminium`, `mild-steel`, `alloy-steel`, `stainless-steel`, `cast-iron`,
    /// `brass`, `titanium`, `plastics`, `wood`, `mdf`).
    pub material: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct LibraryArgs {
    /// `list` (the user library's tools), `save` (document tool `id` into the user library),
    /// `add` (user-library tool `id` into the document), `remove` (from the user library),
    /// `import` (`name` + `text` of a Fusion 360 / CSV / library file into the document),
    /// `export` (the document's tools, or the user library with `id: "user"`, as library text).
    pub action: String,
    #[serde(default)]
    pub id: Option<String>,
    #[serde(default)]
    pub name: Option<String>,
    #[serde(default)]
    pub text: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct GenerateArgs {
    /// The operations to generate; absent = every enabled operation.
    #[serde(default)]
    pub ids: Option<Vec<String>>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct VisibleArgs {
    /// The operation id.
    pub id: String,
    /// Whether its toolpath is drawn in the viewport.
    pub visible: bool,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct HoldArgs {
    /// Keep finished generation replies back (true) or deliver them (false).
    pub hold: bool,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ScrubArgs {
    /// The preview's scrub position on the selected operation, 0..1 along its segments; absent clears it.
    #[serde(default)]
    pub fraction: Option<f64>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct IdArgs {
    pub id: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SelectArgs {
    /// The row to select; absent clears the selection.
    #[serde(default)]
    pub id: Option<String>,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct Added {
    /// The id the block minted (`SETUP1`, `T2`, `OP3`).
    pub id: String,
}

fn cam_state(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    ctx.app.cam.refresh_geometry(ctx.app.docs.engine());
    Ok(Outcome::Done(parse(&cam::cam_state_json(ctx.app.docs.engine(), &ctx.app.cam))))
}

fn added(id: String) -> Result<Outcome, String> {
    serde_json::to_value(Added { id }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn cam_add_setup(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: NamedArgs = parse_args(args)?;
    let id = cam::add_setup(ctx.app.docs.engine_mut(), a.name.as_deref())?;
    ctx.app.cam.select(Some(id.clone()));
    added(id)
}

fn cam_add_tool(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: NamedArgs = parse_args(args)?;
    let id = cam::add_tool(ctx.app.docs.engine_mut(), a.name.as_deref())?;
    ctx.app.cam.select(Some(id.clone()));
    added(id)
}

fn cam_add_machine(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AddMachineArgs = parse_args(args)?;
    let id = cam::add_machine(ctx.app.docs.engine_mut(), &a.base, a.name.as_deref())?;
    ctx.app.cam.select(Some(id.clone()));
    added(id)
}

fn cam_update_machine(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: UpdateMachineArgs = parse_args(args)?;
    cam::update_machine(ctx.app.docs.engine_mut(), &a.id, &a.patch, None)?;
    Ok(Outcome::Done(json!({})))
}

fn cam_add_operation(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AddOperationArgs = parse_args(args)?;
    let id = cam::add_operation(ctx.app.docs.engine_mut(), &a.setup, &a.tool, &a.strategy, a.name.as_deref())?;
    ctx.app.cam.select(Some(id.clone()));
    added(id)
}

fn cam_add_program(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AddProgramArgs = parse_args(args)?;
    let id = cam::add_program_with(ctx.app.docs.engine_mut(), a.name.as_deref(), &a.post, &a.operations, &a.properties)?;
    ctx.app.cam.select(Some(id.clone()));
    added(id)
}

fn cam_update_program(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: UpdateProgramArgs = parse_args(args)?;
    cam::set_program_properties(ctx.app.docs.engine_mut(), &a.id, &a.properties)?;
    Ok(Outcome::Done(json!({})))
}

fn cam_open(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: OpenArgs = parse_args(args)?;
    if let Some(id) = &a.id {
        let doc = cam::cam_document(ctx.app.docs.engine())?;
        if doc.setup(id).is_none() && doc.tool(id).is_none() && doc.operation(id).is_none() && !doc.machines.iter().any(|m| m.id.as_str() == id) {
            return Err(format!("no setup, tool, operation or document machine `{id}` (see cam_state)"));
        }
        ctx.app.cam.select(Some(id.clone()));
    } else if ctx.app.docs.engine().ref_select_active() {
        ctx.app.docs.engine_mut().finish_ref_select();
    }
    ctx.app.cam.set_open(a.id);
    ctx.app.show_pane(crate::panels::dock::PaneKind::Cam);
    Ok(Outcome::Done(json!({ "open": ctx.app.cam.open_setup() })))
}

fn cam_update_setup(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: UpdateSetupArgs = parse_args(args)?;
    cam::update_setup(ctx.app.docs.engine_mut(), &a.id, &a.patch, None)?;
    Ok(Outcome::Done(json!({})))
}

fn cam_set_ref(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: SetRefArgs = parse_args(args)?;
    let engine = ctx.app.docs.engine_mut();
    let is_operation = cam::cam_document(engine)?.operation(&a.id).is_some();
    if is_operation {
        cam::begin_operation_pick(engine, &a.id, &a.field)?;
    } else {
        cam::begin_setup_pick(engine, &a.id, &a.field)?;
    }
    // The seed is the field's current names; the call's names REPLACE them.
    while !engine.ref_select_names().is_empty() {
        engine.ref_select_remove(0);
    }
    for name in &a.names {
        engine.ref_select_add_name(name.clone());
    }
    engine.finish_ref_select();
    let frame_error = if is_operation {
        cam::sync_operation_faces(engine, &a.id)?;
        None
    } else {
        cam::sync_setup_frame(engine, &a.id)
    };
    ctx.app.cam.refresh_geometry(ctx.app.docs.engine());
    Ok(Outcome::Done(json!({ "frameError": frame_error })))
}

fn cam_update_operation(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: UpdateOperationArgs = parse_args(args)?;
    cam::update_operation(ctx.app.docs.engine_mut(), &a.id, &a.patch, None)?;
    Ok(Outcome::Done(json!({})))
}

fn cam_update_tool(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: UpdateToolArgs = parse_args(args)?;
    cam::update_tool(ctx.app.docs.engine_mut(), &a.id, &a.patch, None)?;
    Ok(Outcome::Done(json!({})))
}

fn cam_duplicate_tool(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: IdArgs = parse_args(args)?;
    let id = cam::duplicate_tool(ctx.app.docs.engine_mut(), &a.id)?;
    ctx.app.cam.select(Some(id.clone()));
    added(id)
}

fn cam_calculate_feeds(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: FeedsArgs = parse_args(args)?;
    let adjustments = cam::calculate_tool_feeds(ctx.app.docs.engine_mut(), &a.id, &a.material)?;
    let doc = cam::cam_document(ctx.app.docs.engine())?;
    let feeds = doc.tool(&a.id).map(|t| serde_json::to_value(&t.feeds).unwrap_or(Value::Null)).unwrap_or(Value::Null);
    Ok(Outcome::Done(json!({ "feeds": feeds, "adjustments": adjustments })))
}

fn cam_library(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    use crate::panels::cam_library;
    let a: LibraryArgs = parse_args(args)?;
    let store = ctx.app.model_store.as_ref();
    let need_id = || a.id.clone().ok_or_else(|| format!("`{}` needs `id`", a.action));
    match a.action.as_str() {
        "list" => {
            let library = cam_library::load(store)?;
            Ok(Outcome::Done(json!({ "name": library.library.name, "tools": library.library.tools })))
        }
        "save" => {
            let id = need_id()?;
            let doc = cam::cam_document(ctx.app.docs.engine())?;
            let tool = doc.tool(&id).ok_or_else(|| format!("no tool `{id}`"))?;
            cam_library::save_tool(store, tool)?;
            ctx.app.cam.refresh_user_library(store);
            Ok(Outcome::Done(json!({})))
        }
        "add" => {
            let id = need_id()?;
            let new_id = cam::add_tool_from_library(ctx.app.docs.engine_mut(), store, &id)?;
            ctx.app.cam.select(Some(new_id.clone()));
            added(new_id)
        }
        "remove" => {
            cam_library::remove_tool(store, &need_id()?)?;
            ctx.app.cam.refresh_user_library(store);
            Ok(Outcome::Done(json!({})))
        }
        "import" => {
            let name = a.name.clone().unwrap_or_else(|| "tools.json".to_string());
            let text = a.text.clone().ok_or_else(|| "`import` needs `text`".to_string())?;
            let (ids, issues) = cam::import_tools(ctx.app.docs.engine_mut(), &name, &text)?;
            Ok(Outcome::Done(json!({ "ids": ids, "skipped": issues })))
        }
        "export" => {
            let text = if a.id.as_deref() == Some("user") { cam_library::export_text(store)? } else { cam::export_document_tools(ctx.app.docs.engine())? };
            Ok(Outcome::Done(json!({ "text": text })))
        }
        other => Err(format!("no library action `{other}` (list, save, add, remove, import, export)")),
    }
}

fn cam_remove(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: IdArgs = parse_args(args)?;
    cam::remove_record(ctx.app.docs.engine_mut(), &a.id)?;
    if ctx.app.cam.selected() == Some(a.id.as_str()) {
        ctx.app.cam.select(None);
    }
    Ok(Outcome::Done(json!({})))
}

fn cam_select(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: SelectArgs = parse_args(args)?;
    if let Some(id) = &a.id {
        if !cam::cam_document(ctx.app.docs.engine())?.has(id) {
            return Err(format!("no CAM record `{id}` (see cam_state)"));
        }
    }
    ctx.app.cam.select(a.id);
    Ok(Outcome::Done(json!({ "selected": ctx.app.cam.selected() })))
}

fn cam_generate(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: GenerateArgs = parse_args(args)?;
    let doc = cam::cam_document(ctx.app.docs.engine())?;
    if doc.operations.is_empty() {
        return Err("nothing to generate: add an operation first".to_string());
    }
    let (queued, refused) = cam::generate(ctx.app.docs.engine_mut(), a.ids.as_deref())?;
    // Read before any frame runs: a fast job can land on the very next frame,
    // so this is the one moment the job in flight is certain.
    let engine = ctx.app.docs.engine();
    // The chord each queued job's part mesh is tessellated at (a quarter of
    // its operation's tolerance, half for scallop), for a reader to check.
    let chords: serde_json::Map<String, Value> = queued
        .iter()
        .filter_map(|id| doc.operation(id).and_then(|op| crate::panels::cam_generate::job_chord(id, op).ok()).map(|c| (id.clone(), json!(c))))
        .collect();
    Ok(Outcome::Done(json!({
        "queued": queued,
        "chords": chords,
        "refused": refused.iter().map(|(id, why)| json!({ "id": id, "reason": why })).collect::<Vec<_>>(),
        "inFlight": engine.cam.in_flight(),
        "queue": engine.cam.queued(),
    })))
}

fn cam_hold_replies(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: HoldArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().cam_hold_replies(a.hold);
    Ok(Outcome::Done(json!({ "hold": a.hold })))
}

fn cam_cancel(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let dropped = ctx.app.docs.engine_mut().cam_cancel();
    Ok(Outcome::Done(json!({ "dropped": dropped })))
}

fn cam_toolpath(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: IdArgs = parse_args(args)?;
    crate::panels::cam_generate::toolpath_json(ctx.app.docs.engine(), &a.id)
        .map(Outcome::Done)
        .ok_or_else(|| format!("no generated toolpath for `{}` (see cam_state statuses)", a.id))
}

fn cam_toolpath_visible(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: VisibleArgs = parse_args(args)?;
    let doc = cam::cam_document(ctx.app.docs.engine())?;
    if doc.operation(&a.id).is_none() {
        return Err(format!("no operation `{}`", a.id));
    }
    ctx.app.cam.set_toolpath_visible(&a.id, a.visible);
    Ok(Outcome::Done(json!({ "id": a.id, "visible": a.visible, "hidden": ctx.app.cam.hidden_toolpaths().iter().collect::<Vec<_>>() })))
}

fn cam_scrub(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ScrubArgs = parse_args(args)?;
    ctx.app.cam.set_scrub(a.fraction);
    let doc = cam::cam_document(ctx.app.docs.engine())?;
    let point = ctx.app.cam.selected().and_then(|op| {
        crate::panels::cam_generate::scrub_world(ctx.app.docs.engine(), &doc, op, a.fraction?).map(|(p, index)| json!({ "point": [p.x, p.y, p.z], "segment": index }))
    });
    Ok(Outcome::Done(json!({ "scrub": a.fraction, "at": point })))
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "cam_autoprogram", group: "cam", doc: "Preview automatic programming from a setup, including additional fixed-axis setups for certified side features and named refusals for unreachable features. Document tools are preferred; builtin fallback tools are labelled. Select or untick setups (selectSetup) and features, then create setups, carried-stock links, tools and ordered operations in one undo step. State reports busy/stale; model edits re-recognise bound features at generation, or refuse missing features by name.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<crate::panels::cam_autoprogram::CommandArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(crate::panels::cam_autoprogram::command) },
    CommandSpec { name: "cam_state", group: "cam", doc: "The document's CAM block in the shared model's form: every setup (with the ids of its operations), the tools, the operations (each naming its setup, tool and a `kind`-tagged strategy), the NC programs and the id counter; `strategies` lists the strategy kinds; plus the CAM pane's selected row, its last notice, and `error` when the block cannot be read (then every edit is refused); `generation` (busy, the operation in flight, the queue, progress, every result's summary) and `statuses` (per operation: idle / queued / running / ok / stale / error with its motion count or reason) describe the background generation, which never writes the document.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_state) },
    CommandSpec { name: "cam_add_setup", group: "cam", doc: "Add a setup (the toolbar's New setup) and select it. Returns its id. The work coordinate system, stock and heights are edited in the setup dialog once it lands.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<NamedArgs>, result_schema: schema_of::<Added>, handler: Handler::App(cam_add_setup) },
    CommandSpec { name: "cam_add_tool", group: "cam", doc: "Add a tool to the document's library (the toolbar's New tool) with the next free tool number, and select it. Returns its id.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<NamedArgs>, result_schema: schema_of::<Added>, handler: Handler::App(cam_add_tool) },
    CommandSpec { name: "cam_add_operation", group: "cam", doc: "Add an operation of strategy kind `strategy` (every parameter at its default) to `setup` using `tool` and select it; refused, naming what is missing, when the setup, the tool or the kind does not exist. Returns its id.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<AddOperationArgs>, result_schema: schema_of::<Added>, handler: Handler::App(cam_add_operation) },
    CommandSpec { name: "cam_add_program", group: "cam", doc: "Add an NC program: an ordered group of existing operations posted through one post-processor to one file (the post lane assembles and writes it). Returns its id.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<AddProgramArgs>, result_schema: schema_of::<Added>, handler: Handler::App(cam_add_program) },
    CommandSpec { name: "cam_update_program", group: "cam", doc: "Set or clear (null) an NC program's post properties: a mill-turn lathe post's builder codes, a program number. A built-in post's names and enum values are checked against its own property list.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<UpdateProgramArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_update_program) },
    CommandSpec { name: "cam_add_machine", group: "cam", doc: "Copy a machine into the document (the Machines section's New machine): `base` is a built-in id or a document machine's id; the copy gets a document id (`M5`), keeps the base's kinematics, limits, post and collision solids, and is selected. A setup names it through `cam_update_setup { patch: { machine } }`.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<AddMachineArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_add_machine) },
    CommandSpec { name: "cam_update_machine", group: "cam", doc: "Apply a nested patch of the machine editor's keys to a DOCUMENT machine `id` (see the argument): one undo step; a refused value names its field (`spindle.maxRpm: ...`, `post: no post ...`, `machine: <the model's rule>`) with nothing applied; a new `kind` starts that kind's axis layout. A built-in is refused: copy it first with `cam_add_machine`.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<UpdateMachineArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_update_machine) },
    CommandSpec { name: "cam_update_tool", group: "cam", doc: "Patch a tool through its form's keys (what typing in the tool dialog does): name, number, kind, description, material, the geometry, the holder (family, bore, height: designed through the holder library), spindle and feeds, coolant. Validated by the library's validate_tool; a refusal names the field with nothing applied.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<UpdateToolArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_update_tool) },
    CommandSpec { name: "cam_duplicate_tool", group: "cam", doc: "Copy a tool under a fresh id and the next free tool number (`<name> (copy)`) and select it. Returns the new id.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<IdArgs>, result_schema: schema_of::<Added>, handler: Handler::App(cam_duplicate_tool) },
    CommandSpec { name: "cam_calculate_feeds", group: "cam", doc: "Feeds and speeds for a tool cutting `material` on a default 3-axis mill, through the feeds calculator, written into the tool. Returns the feeds and the calculator's adjustments.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<FeedsArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_calculate_feeds) },
    CommandSpec { name: "cam_library", group: "cam", doc: "The USER tool library (kept across documents in the model store): `list`; `save` a document tool into it; `add` one of its tools to the document (fresh id and number); `remove`; `import` a Fusion 360 (.json / .tools), CSV or library file's `text` into the document's tools; `export` the document's tools (or the user library with id `user`) as library text. The pane's Import… and Export buttons do the same through the file chooser and the save door.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<LibraryArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_library) },
    CommandSpec { name: "cam_update_operation", group: "cam", doc: "Patch an operation through its form's keys (what typing in the operation dialog does): name, enabled, tool, liveTool (true/false/null) or liveToolMode (live/off/auto), feed and height overrides, the bodies and faces by name, tolerance, stock to leave, and `strategy.*` — the strategy's own parameters by the model's field names, which the dialog walks generically — and `linking.*`. Read back as the model's types and validated; a refusal names the field or group with nothing applied.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<UpdateOperationArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_update_operation) },
    CommandSpec { name: "cam_open", group: "cam", doc: "Open a setup's, a tool's or an operation's dialog in the CAM pane (a setup: its work coordinate system by picking, stock, heights and bodies, with the frame and stock drawn in the viewport; a tool: its kind, geometry, holder, feeds and coolant; an operation: Tool / Geometry / Heights / Passes / Linking over its strategy's parameters), or close the open one with no id.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<OpenArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_open) },
    CommandSpec { name: "cam_update_setup", group: "cam", doc: "Patch a setup through its form's keys (what typing in the setup dialog does): name, work offset number, the orientation references, the stock kind and its parameters, the heights, the bodies. A value the field refuses is refused here by field name with nothing applied; the frame the references resolve to is written back as `modelToMachine`. The Machine group: `machine` is `none`, a built-in id (3-axis VMC, 4th-axis A table, trunnions A/C and B/C, head-head, head-table, lathe, mill-turn C/Y) or `custom` (a copy under a document id with `machineBase`, `machineName`, `machinePost` and `rotary1Min/Max`, `rotary2Min/Max` editable, empty = continuous); generation, the simulation player and an empty-post NC program use it.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<UpdateSetupArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_update_setup) },
    CommandSpec { name: "cam_set_ref", group: "cam", doc: "Pick a setup's or an operation's reference field by entity name, without a pointer: arms the picker on the field, takes the names, finishes. A setup's frame is re-resolved (`frameError` says why when a reference does not resolve); an operation's picked faces get their owning solid from the scene. `boundaryProfile` (a strategy with a containment) takes a sketch profile and samples its loops, in the setup's machine XY within a quarter of the operation's tolerance, into the containment boundary; anything but one sketch profile across the setup's Z is refused by name and leaves the boundary as it was.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<SetRefArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_set_ref) },
    CommandSpec { name: "cam_remove", group: "cam", doc: "Delete a setup (and its operations), an operation, a tool (refused while an operation uses it) or an NC program by id.", phase: Phase::Mutate, annotations: Annotations::DESTRUCTIVE, args_schema: schema_of::<IdArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_remove) },
    CommandSpec { name: "cam_select", group: "cam", doc: "Select a row of the CAM pane by id (a setup, operation, tool or program), or clear the selection with no id. The selection is what New operation reads: the setup it goes into and the tool it uses.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<SelectArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_select) },
    CommandSpec { name: "cam_generate", group: "cam", doc: "Queue the generation of every enabled operation (or `ids`) on the background runner, one operation per job: returns the ids queued (an id named twice, or already queued or in flight, is queued once), per operation that cannot be queued why (no targets, a missing tool, a body not on screen), the chord each queued job's part mesh is tessellated at (`chords`: a quarter of its operation's tolerance, half for scallop, which certifies the mesh sag itself), and the job in flight and the queue as they stand when the command returns — before any frame can land a fast job's reply. The results land over the following frames — `cam_state` `generation` (busy, in flight, queue, progress) and `statuses` (per operation: idle / queued / running / ok / stale / error) say when; `wait_state` on `/__brepCam/generation/busy` eq false waits for them. A toolpath is never stored in the document.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<GenerateArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_generate) },
    CommandSpec { name: "cam_cancel", group: "cam", doc: "Cancel the generation: the queue is dropped at once; the operation in flight stops at its next work chunk where the runner can reach it (the native thread) and its reply is dropped either way; results already landed stay. Returns how many jobs were dropped.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_cancel) },
    CommandSpec { name: "cam_hold_replies", group: "cam", doc: "TEST HOLD for scripts: while `hold` is true the runner's finished generation replies are kept back, so a job stays in flight on the app's side however fast it ran — a scripted `cam_cancel` then always meets it, and on release its reply is dropped as cancelled. `hold: false` delivers what was kept. Not a user feature; the browser worker ignores it.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<HoldArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_hold_replies) },
    CommandSpec { name: "cam_toolpath", group: "cam", doc: "One operation's generated toolpath: the typed motion stream (rapid / plunge / cut / arcs / retract / leads / links / cycles / spindle and coolant), the generator's report, the cutting length and `cutBounds` (the box around every cut's end point, machine coordinates; null without a cut). Refused while the operation has no result.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<IdArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_toolpath) },
    CommandSpec { name: "cam_toolpath_visible", group: "cam", doc: "Show or hide one operation's generated toolpath in the viewport (the row menu's Hide / Show toolpath): pane state, not the document's; `cam_state` lists the hidden ids under `hiddenToolpaths`. Refused for an operation the document does not have.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<VisibleArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_toolpath_visible) },
    CommandSpec { name: "cam_scrub", group: "cam", doc: "Set the toolpath preview's scrub position on the SELECTED operation (0..1 along its segments): the viewport marks the tool position there; returns that world point and the segment index. Absent clears the marker.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<ScrubArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_scrub) },
];
