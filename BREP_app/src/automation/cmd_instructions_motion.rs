//! A motion study's published motion on a manufacturing-instructions step
//! (`brep_render::engine_state` `instructions_motion`): what the assembly
//! published, attach one to a step, take it off.
use crate::automation::command::{app_command, parse_args, CommandSpec, Ctx, NoArgs, Outcome};
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

fn motion_object(properties: Value) -> Value {
    let required: Vec<_> = properties.as_object().unwrap().keys().cloned().collect();
    json!({"type":"object","properties":properties,"required":required,"additionalProperties":false})
}
fn instructions_motions_result_schema() -> Value {
    let count = json!({"type":"integer","minimum":0});
    let published = motion_object(json!({
        "study":{"type":"string"},"name":{"type":"string"},"frames":count,
        "duration":{"type":"number"},"fps":count,"current":{"type":"boolean"}
    }));
    // Authored step.motion is arbitrary JSON. Its study member is copied
    // without coercion, including null when absent, so retain that wire type.
    let attached = motion_object(json!({"step":{"type":"string"},"study":{},"frames":count}));
    motion_object(json!({"motions":{"type":"array","items":published},"attached":{"type":"array","items":attached}}))
}
fn instructions_attach_motion_result_schema() -> Value {
    let mut schema = instructions_motions_result_schema();
    for key in ["study", "step"] {
        schema["properties"][key] = json!({"type":"string"});
        schema["required"].as_array_mut().unwrap().push(json!(key));
    }
    schema
}

pub static COMMANDS: &[CommandSpec] = &[
    app_command!("instructions", "instructions_motions", "The motion studies the assembly published (the Simulation workbench's Publish writes a study's baked frames into the assembly's document; the instructions read them from the placed copy): `motions` [{study, name, frames, duration, fps, current}] — `current` false when the motion was published before the assembly changed, so it cannot be attached until published again — and `attached` [{step, study, frames}], the steps that carry one.", NoArgs, instructions_motions, Read, READ, instructions_motions_result_schema),
    app_command!("instructions", "instructions_attach_motion", "Give `step` the published motion of `study` (the assembly's only published study when absent) as its motion: a copy of the study's frames, played in the player in place of the step's insertion animation and kept with the instructions. Refused when the study is not published, was published before the assembly changed, or several are published and none is named; nothing is written then. One undo step. Returns `instructions_motions` plus `step` and `study`.", AttachArgs, instructions_attach_motion, Mutate, MUTATE, instructions_attach_motion_result_schema),
    app_command!("instructions", "instructions_detach_motion", "Take the motion off `step`; its insertion animation plays again. One undo step. Returns `instructions_motions`.", DetachArgs, instructions_detach_motion, Mutate, MUTATE, instructions_motions_result_schema),
];
