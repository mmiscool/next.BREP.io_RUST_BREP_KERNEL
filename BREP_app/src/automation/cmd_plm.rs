//! The PLM's bake queue, driven from a session (plm-cad-integration-todo §3
//! S10): one pass of the bake worker, the same pass `brep-app --bake-worker`
//! runs. It needs a PLM server and a token, and it touches no open document:
//! each job is baked on an engine of its own (`crate::plm::bake`).
//!
//! Native only. The pass blocks the frame while it talks to the server, which
//! is what a headless session and a test want, and the browser build has no
//! worker to run.
use crate::automation::command::{app_command, parse_args, CommandSpec, Ctx, Outcome};
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
    // The pass runs on the frame thread's task queue and lands in `slot`;
    // the command answers on the frame it finishes (`Outcome::Poll`).
    let slot = crate::plm::bake::pass_spawned(&a.url, &a.token, a.max_jobs.unwrap_or(1000));
    Ok(Outcome::Poll(Box::new(move |_ctx: &mut Ctx<'_>| {
        let done = slot.lock().unwrap().take()?;
        Some(done.map(|jobs| Outcome::Done(json!({ "jobs": jobs, "ms": started.elapsed().as_millis() as u64 }))))
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
    app_command!("plm", "plm_follow_now", "Ask the PLM change feed at the next frame instead of at its interval (the pane follows other clients: a moved feed re-reads every open PLM document's part and the inbox). Answers the follower's counts so far: feed requests and moves.", crate::automation::command::NoArgs, plm_follow_now, Mutate, MUTATE, super::domain_output_schema::plm_follow_now),
    app_command!("plm", "plm_bake_pass", "Run one pass of the PLM bake worker: claim each queued job in turn (`POST /api/bake/next`), open the document the server assembled on an engine of its own, build it, and keep it (`PUT …/result`) or report why not (`POST …/fail`, with the build's feature errors). Returns each job's `{id, number, revision, source, status: done|failed|dropped, error|why}` and the pass's `ms`. The open documents are untouched. An error means the server did not answer or refused the token.", BakePassArgs, plm_bake_pass, Mutate, MUTATE, super::domain_output_schema::plm_bake_pass),
];
