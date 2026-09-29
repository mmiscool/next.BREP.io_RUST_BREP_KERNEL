//! The PLM's bake queue, driven from a session (plm-cad-integration-todo §3
//! S10): one pass of the bake worker, the same pass `brep-app --bake-worker`
//! runs. It needs a PLM server and a token, and it touches no open document:
//! each job is baked on an engine of its own (`crate::plm::bake`).
//!
//! Native only. The pass blocks the frame while it talks to the server, which
//! is what a headless session and a test want, and the browser build has no
//! worker to run.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, Outcome, Phase};
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct BakePassArgs {
    /// The PLM server's base URL, e.g. `http://127.0.0.1:8080`.
    pub url: String,
    /// An API token. A `worker`-scoped one is enough.
    pub token: String,
    /// Stop after this many jobs. Default: until the queue is empty (at most 1000).
    #[serde(default)]
    pub max_jobs: Option<usize>,
}

fn plm_bake_pass(_ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: BakePassArgs = parse_args(args)?;
    let started = web_time::Instant::now();
    let outcomes = crate::plm::bake::pass_blocking(&a.url, &a.token, a.max_jobs.unwrap_or(1000))?;
    Ok(Outcome::Done(json!({
        "jobs": outcomes.iter().map(|o| o.to_json()).collect::<Vec<_>>(),
        "ms": started.elapsed().as_millis() as u64,
    })))
}

/// Ask the change feed at the next frame instead of at the interval, so a
/// script sees another client's change without waiting the interval out.
fn plm_follow_now(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: crate::automation::command::NoArgs = parse_args(args)?;
    ctx.app.plm.follow_now();
    let follower = ctx.app.plm.follower();
    Ok(Outcome::Done(json!({ "requests": follower.requests, "moves": follower.moves })))
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "plm_follow_now", group: "plm", doc: "Ask the PLM change feed at the next frame instead of at its interval (the pane follows other clients: a moved feed re-reads every open PLM document's part and the inbox). Answers the follower's counts so far: feed requests and moves.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<crate::automation::command::NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(plm_follow_now) },
    CommandSpec { name: "plm_bake_pass", group: "plm", doc: "Run one pass of the PLM bake worker: claim each queued job in turn (`POST /api/bake/next`), open the document the server assembled on an engine of its own, build it, and keep it (`PUT …/result`) or report why not (`POST …/fail`, with the build's feature errors). Returns each job's `{id, number, revision, source, status: done|failed|dropped, error|why}` and the pass's `ms`. The open documents are untouched. An error means the server did not answer or refused the token.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<BakePassArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(plm_bake_pass) },
];
