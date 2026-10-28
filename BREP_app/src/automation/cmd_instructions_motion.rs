//! A motion study's published motion on a manufacturing-instructions step
//! (`brep_render::engine_state` `instructions_motion`): what the assembly
//! published, attach one to a step, take it off.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Outcome, Phase};
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AttachArgs {
    /// The step id (`MS3`).
    pub step: String,
    /// The study id; the assembly's only published study when absent.
    #[serde(default)]
    pub study: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct DetachArgs {
    /// The step id.
    pub step: String,
}

fn motions(ctx: &mut Ctx<'_>) -> Result<Value, String> {
    let engine = ctx.app.docs.engine();
    let motions = engine.instructions_published_motions()?;
    let block = engine.instructions()?.ok_or("the active document is not manufacturing instructions")?;
    let attached: Vec<Value> = block
        .steps()
        .filter_map(|(_, step)| step.motion.as_ref().map(|m| json!({ "step": step.id, "study": m.get("study"), "frames": m.get("frames").and_then(Value::as_array).map_or(0, Vec::len) })))
        .collect();
    Ok(json!({ "motions": motions, "attached": attached }))
}

fn instructions_motions(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(motions(ctx)?))
}

fn instructions_attach_motion(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AttachArgs = parse_args(args)?;
    let study = ctx.app.docs.engine_mut().instructions_attach_motion(&a.step, a.study.as_deref())?;
    let mut out = motions(ctx)?;
    out["study"] = json!(study);
    out["step"] = json!(a.step);
    Ok(Outcome::Done(out))
}

fn instructions_detach_motion(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: DetachArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().instructions_detach_motion(&a.step)?;
    Ok(Outcome::Done(motions(ctx)?))
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "instructions_motions", group: "instructions", doc: "The motion studies the assembly published (the Simulation workbench's Publish writes a study's baked frames into the assembly's document; the instructions read them from the placed copy): `motions` [{study, name, frames, duration, fps, current}] — `current` false when the motion was published before the assembly changed, so it cannot be attached until published again — and `attached` [{step, study, frames}], the steps that carry one.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_motions) },
    CommandSpec { name: "instructions_attach_motion", group: "instructions", doc: "Give `step` the published motion of `study` (the assembly's only published study when absent) as its motion: a copy of the study's frames, played in the player in place of the step's insertion animation and kept with the instructions. Refused when the study is not published, was published before the assembly changed, or several are published and none is named; nothing is written then. One undo step. Returns `instructions_motions` plus `step` and `study`.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<AttachArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_attach_motion) },
    CommandSpec { name: "instructions_detach_motion", group: "instructions", doc: "Take the motion off `step`; its insertion animation plays again. One undo step. Returns `instructions_motions`.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<DetachArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(instructions_detach_motion) },
];
