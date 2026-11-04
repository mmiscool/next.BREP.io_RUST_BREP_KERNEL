//! Host file operations. The public MCP and browser wrappers call this same
//! handler for documents/workspace; browser writes finish only after persistence.
use super::command::{Annotations, CommandSpec, Ctx, Handler, Outcome, Phase};
use brep_mcp_core::files::{self, Request};
use serde_json::Value;

fn execute(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let request: Request = super::command::parse_args(args)?;
    let (_, _, store) = ctx.app.file_docs_store();
    let result = serde_json::to_value(store.files(&request)?).map_err(|e| e.to_string())?;
    if request.operation.annotations().read_only {
        return Ok(Outcome::Done(result));
    }
    if let Some(settled) = store.files_settled(&request) {
        settled?;
        return Ok(Outcome::Done(result));
    }
    Ok(Outcome::Poll(Box::new(move |ctx| {
        let (_, _, store) = ctx.app.file_docs_store();
        store
            .files_settled(&request)
            .map(|settled| settled.map(|()| Outcome::Done(result.clone())))
    })))
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec {
        name: "files", group: "script",
        doc: "Bounded file operations under named documents/workspace roots. Browser mutations wait for persistence and report failures. The MCP files front door and brep.files methods share this handler.",
        phase: Phase::Mutate, annotations: Annotations::DESTRUCTIVE,
        args_schema: files::input_schema, result_schema: files::output_schema,
        handler: Handler::App(execute),
    },
];
