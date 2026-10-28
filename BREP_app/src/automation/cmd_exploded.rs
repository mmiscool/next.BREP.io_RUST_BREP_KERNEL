//! Assembly exploded views (`brep_render::engine_state` `exploded_ops`): an
//! exploded view is a PMI view whose explode rows are its ordered steps. These
//! are the operations the Constraints pane's Exploded views section performs,
//! on the same engine calls. Steps are ordinary PMI explode annotations, so
//! `pmi_update_annotation` / `pmi_remove_annotation` / `pmi_move_annotation`
//! edit them.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Outcome, Phase};
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AutoExplodeArgs {
    /// The view's name; `Exploded view N` when absent.
    #[serde(default)]
    pub name: Option<String>,
    /// How far each component moves, as a multiple of half the assembly's
    /// diagonal (default 1).
    #[serde(default)]
    pub spread: Option<f64>,
    /// How many levels open: 1 (default) moves the assembly's own
    /// placements; more opens each sub-assembly too, its placements moving
    /// away from its own base.
    #[serde(default)]
    pub depth: Option<usize>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ProgressArgs {
    /// Steps shown, `0` (collapsed) to the step count (fully exploded); a
    /// fraction shows the next step part of the way.
    pub progress: f64,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AnimateArgs {
    /// `true` plays the steps out (Explode), `false` brings them home (Collapse).
    pub explode: bool,
    /// Speed in steps per second (default 1.5, or as fast as plays every
    /// step in about 4 s when there are more than 6).
    #[serde(default)]
    pub steps_per_second: Option<f64>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddStepArgs {
    /// Occurrence paths to move (`ACOMP2`, `ACOMP1:ACOMP2`); the selection's
    /// placements when absent.
    #[serde(default)]
    pub targets: Option<Vec<String>>,
}

fn exploded_add_step(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AddStepArgs = parse_args(args)?;
    let engine = ctx.app.docs.engine_mut();
    let id = engine.exploded_add_step(&a.targets.unwrap_or_default())?;
    Ok(Outcome::Done(json!({ "id": id, "state": engine.exploded_state_json() })))
}

fn exploded_state(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(ctx.app.docs.engine().exploded_state_json()))
}

fn assembly_auto_explode(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AutoExplodeArgs = parse_args(args)?;
    let id = ctx.app.docs.engine_mut().assembly_auto_explode_deep(a.name.as_deref(), a.spread.unwrap_or(1.0), a.depth.unwrap_or(1))?;
    Ok(Outcome::Done(json!({ "view": id })))
}

fn exploded_set_progress(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ProgressArgs = parse_args(args)?;
    let engine = ctx.app.docs.engine_mut();
    if engine.pmi_active_view().is_none() {
        return Err("activate an exploded view first".into());
    }
    engine.exploded_set_progress(a.progress);
    Ok(Outcome::Done(engine.exploded_state_json()))
}

fn exploded_animate(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AnimateArgs = parse_args(args)?;
    let engine = ctx.app.docs.engine_mut();
    let speed = a.steps_per_second.unwrap_or_else(|| 1.5f64.max(engine.exploded_step_count() as f64 / 4.0));
    engine.exploded_animate(a.explode, speed)?;
    Ok(Outcome::Done(json!({})))
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "exploded_state", group: "assembly", doc: "The assembly's exploded views (PMI views holding explode steps: id, name, and per step its targets, translation, trail and enabled flag) and the active view's playback: step count, progress (0 = collapsed, the step count = fully exploded) and whether an Explode / Collapse animation is running.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(exploded_state) },
    CommandSpec { name: "assembly_auto_explode", group: "assembly", doc: "Make a new exploded view: one step per top-level component, each moved straight away from the base component (the first fixed one, else the largest) by `spread` times half the assembly's diagonal, the outermost first, trails on. `depth` > 1 then opens each sub-assembly too (the base included): its placements move away from its own base by `spread` times half its own size, level by level after the assembly's own steps, a nested step moving on top of those above it. The view is framed on the exploded parts and becomes active. Returns its id.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<AutoExplodeArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(assembly_auto_explode) },
    CommandSpec { name: "exploded_set_progress", group: "assembly", doc: "Show the active exploded view's steps up to `progress` (clamped to 0 ..= the step count; step k is shown at clamp(progress - k, 0, 1)), stopping any animation. Display only: nothing is saved.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<ProgressArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(exploded_set_progress) },
    CommandSpec { name: "exploded_animate", group: "assembly", doc: "Play the active exploded view's steps out (`explode: true`, from collapsed when fully exploded) or back home, at `steps_per_second`. The app advances it once per frame; `exploded_state` `animating` says when it is done.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<AnimateArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(exploded_animate) },
    CommandSpec { name: "exploded_add_step", group: "assembly", doc: "Add a step to the shown exploded view that moves `targets` (the selection's placements when absent: a part inside a sub-assembly is its occurrence path, `ACOMP1:ACOMP2`) up by their own size, and select it so its gizmo drags it. Returns `{id, state}` (the `exploded_state` after it). Refused with no exploded view shown or nothing to move.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<AddStepArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(exploded_add_step) },
];
