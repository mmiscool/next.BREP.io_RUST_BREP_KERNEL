//! The mesh import preview's automation hook: holding its runner replies, so a
//! script can act on a preview in flight without racing the reconstruction.
use crate::automation::command::{app_command, parse_args, schema_of, CommandSpec, Ctx, Outcome};
use serde::{Deserialize, Serialize};
use serde_json::Value;

#[derive(Debug, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct HoldArgs {
    /// `true` holds the replies, `false` releases them (the preview then takes
    /// whatever has arrived on its next frame).
    pub hold: bool,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct HoldResult {
    /// The hold now in force.
    pub held: bool,
    /// Whether a preview is open.
    pub open: bool,
}

fn import_preview_hold(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: HoldArgs = parse_args(args)?;
    let open = ctx.app.set_mesh_preview_hold(a.hold);
    Ok(Outcome::Done(serde_json::to_value(HoldResult { held: a.hold, open }).map_err(|e| e.to_string())?))
}

pub static COMMANDS: &[CommandSpec] = &[
    app_command!("document", "import_preview_hold", "Hold (`hold: true`) or release the mesh import preview's runner replies, for the open preview and any opened later: while held, a reconstruction that has finished is not taken, so the preview reads busy and not ready (`__brepImportPreview`) until released or cancelled. For scripts that must act on a preview in flight (Cancel during a reconstruction) without racing how fast it finishes. Answers the hold and whether a preview is open.", HoldArgs, import_preview_hold, Mutate, MUTATE_NOWAIT, schema_of::<HoldResult>),
];
