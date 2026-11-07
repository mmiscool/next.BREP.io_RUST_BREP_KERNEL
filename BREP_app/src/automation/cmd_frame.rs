//! Frame-level commands: liveness and the per-frame facts every wait loop
//! polls.
use crate::automation::command::{app_command, parse_args, schema_of, CommandSpec, Ctx, NoArgs, Outcome};
use serde::{Deserialize, Serialize};
use serde_json::Value;

#[derive(Serialize, schemars::JsonSchema)]
pub struct ViewRect {
    pub x: f32,
    pub y: f32,
    pub w: f32,
    pub h: f32,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct PendingWork {
    pub run: bool,
    pub queries: bool,
    pub mesh_imports: bool,
    pub step_probes: bool,
    /// A drawing sheet's request for solid topology is in flight.
    pub topology: bool,
    /// A drawing sheet's exact hidden-line pass or an instructions PDF export
    /// is still being prepared. Sheet previews draw a mesh approximation;
    /// PDF export waits for exact lines before delivering the file.
    pub projection: bool,
    /// A write the store accepted is still on its way to where it persists:
    /// a write-behind store's pending writes (the PLM, the browser's
    /// IndexedDB), or `@recovery`'s background mirror to the PLM. A file
    /// store writes before it returns, so it is never pending there.
    pub store_writes: bool,
    /// A PLM request is in flight on the native executor: a sign-in or check
    /// from the Settings PLM tab, a document prefetch, a mirror. Always false
    /// with no PLM.
    pub plm_requests: bool,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct RunProgress {
    pub generation: u64,
    pub index: usize,
    pub total: usize,
    pub feature_id: String,
    pub feature_type: String,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct FrameInfo {
    pub frame: u64,
    pub ppp: f32,
    /// Surface size in egui points.
    pub surface: [f32; 2],
    /// The 3D viewport rect in egui points, if it was drawn last frame.
    pub view: Option<ViewRect>,
    pub pending: PendingWork,
    /// True when nothing is pending: the idle contract's condition.
    pub idle: bool,
    pub progress: Option<RunProgress>,
    pub cancelled: Option<String>,
    pub history_len: usize,
    pub step: usize,
    pub poisoned: bool,
}

pub fn frame_info(ctx: &Ctx<'_>) -> FrameInfo {
    frame_info_except(ctx, &[])
}

/// [`frame_info`], with the store writes and PLM tasks of the documents
/// stored under `others` left out: one document's view of the app's work, for
/// `settle` on that document. Work that belongs to no document (a sign-in, a
/// settings write, `@recovery`'s mirror) still counts.
fn frame_info_except(ctx: &Ctx<'_>, others: &[&str]) -> FrameInfo {
    let engine = ctx.app.docs.engine();
    let pending = PendingWork {
        // A Work part edited in context is idle only once its assembly has
        // taken the latest part state and re-run.
        run: engine.run_pending() || ctx.app.docs.work_part_settling(),
        queries: engine.queries_pending(),
        mesh_imports: engine.mesh_imports_pending(),
        step_probes: engine.step_probes_pending(),
        topology: engine.topology_pending(),
        projection: engine.sheet_lines_pending() || ctx.app.instructions_pdf_pending(),
        store_writes: ctx.app.model_store().pending_writes_except(others) > 0 || store_mirror_in_flight(),
        plm_requests: plm_requests_in_flight(others) || ctx.app.plm.busy() || ctx.app.toolbar.inbox().busy(),
    };
    let idle = !(pending.run
        || pending.queries
        || pending.mesh_imports
        || pending.step_probes
        || pending.topology
        || pending.projection
        || pending.store_writes
        || pending.plm_requests);
    let rect = ctx.egui.content_rect();
    FrameInfo {
        frame: ctx.egui.cumulative_frame_nr(),
        ppp: ctx.egui.pixels_per_point(),
        surface: [rect.width(), rect.height()],
        view: ctx.app.viewport.last_rect().map(|r| ViewRect { x: r.min.x, y: r.min.y, w: r.width(), h: r.height() }),
        pending,
        idle,
        progress: engine.run_progress().map(|p| RunProgress {
            generation: p.generation,
            index: p.index,
            total: p.total,
            feature_id: p.feature_id.clone(),
            feature_type: p.feature_type.clone(),
        }),
        cancelled: engine.cancelled_run().map(str::to_string),
        history_len: engine.history_len(),
        step: engine.history_rollback(),
        poisoned: ctx.app.automation.is_poisoned(),
    }
}

/// `@recovery`'s background mirror (the PLM backend) is still sending.
fn store_mirror_in_flight() -> bool {
    crate::plm::backend::mirror_in_flight()
}

/// Anything unfinished on the native PLM executor that is not the work of
/// the documents stored under `others`.
fn plm_requests_in_flight(others: &[&str]) -> bool {
    #[cfg(not(target_arch = "wasm32"))]
    {
        crate::plm::native::pending_except(others) > 0
    }
    #[cfg(target_arch = "wasm32")]
    {
        let _ = others;
        false
    }
}

/// The store names of every open document except the one at `index`: whose
/// saves `settle` on that document does not wait for.
fn other_documents(ctx: &Ctx<'_>, index: usize) -> Vec<String> {
    ctx.app.docs.iter().enumerate().filter(|(i, _)| *i != index).filter_map(|(_, doc)| doc.name().map(str::to_string)).collect()
}

fn ping_result_schema() -> Value {
    serde_json::json!({"type":"object","properties":{},"additionalProperties":false})
}

fn describe_commands_result_schema() -> Value {
    serde_json::json!({"type":"array","items":{
        "type":"object","properties":{
            "name":{"type":"string"},"group":{"type":"string"},"doc":{"type":"string"},
            "phase":{"type":"string","enum":["input","mutate","read"]},
            "annotations":{"type":"object","properties":{
                "read_only":{"type":"boolean"},"destructive":{"type":"boolean"},
                "idempotent":{"type":"boolean"},"waits":{"type":"boolean"}
            },"required":["read_only","destructive","idempotent","waits"],"additionalProperties":false},
            "argsSchema":{},"resultSchema":{}
        },"required":["name","group","doc","phase","annotations","argsSchema","resultSchema"],"additionalProperties":false
    }})
}

fn ping(_ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(serde_json::json!({})))
}

fn frame_info_cmd(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    serde_json::to_value(frame_info(ctx)).map(Outcome::Done).map_err(|e| e.to_string())
}

fn describe_commands(_ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(crate::automation::command::describe()))
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SettleArgs {
    /// How many consecutive checks after the first idle one must also find
    /// the app idle (work a command started can begin a moment after it
    /// answered). Default 2.
    #[serde(default = "default_settle")]
    pub settle: u32,
    /// Give up after this long (milliseconds). Default 60000.
    #[serde(default = "default_settle_timeout")]
    pub timeout_ms: u64,
    /// Settle THIS document (a `doc:` handle), background tab or not, so
    /// another tab's work (its run, queries, saves and PLM requests) neither
    /// delays nor hides its own. Default: the visible document. Work that is
    /// no document's (a sign-in, a settings write, `@recovery`'s mirror)
    /// always counts.
    #[serde(default)]
    pub document: Option<String>,
}

fn default_settle() -> u32 {
    2
}

fn default_settle_timeout() -> u64 {
    60_000
}

fn settle_result_schema() -> Value {
    serde_json::json!({"type":"object","properties":{
        "info":{"type":"object","description":"frame_info at the settled check"},
        "frame":{"type":"integer","minimum":0},
        "step":{"type":"integer","minimum":0},
        "listing":{},"report":{},"model":{}
    },"required":["info","frame","step","listing","report","model"],"additionalProperties":false})
}

/// The flags of `pending` that are set, comma-separated: what a timeout was
/// still waiting for.
fn pending_names(pending: &PendingWork) -> String {
    let flags = [
        ("run", pending.run),
        ("queries", pending.queries),
        ("mesh_imports", pending.mesh_imports),
        ("step_probes", pending.step_probes),
        ("topology", pending.topology),
        ("projection", pending.projection),
        ("store_writes", pending.store_writes),
        ("plm_requests", pending.plm_requests),
    ];
    let names: Vec<&str> = flags.iter().filter(|(_, set)| *set).map(|(name, _)| *name).collect();
    if names.is_empty() { "nothing, but the app had not been idle long enough".to_string() } else { names.join(", ") }
}

/// Answer once the app has been idle (`frame_info`'s `idle`) for `settle`
/// further checks, with the settled history listing, report and model: the
/// wait every mutating tool ends with, done where the work is. The queue
/// checks it whenever the app services its work (a drawn frame, or the
/// desktop loop's service step, which the finishing work wakes), so the
/// host neither polls nor sleeps.
fn settle(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: SettleArgs = parse_args(args)?;
    let deadline = web_time::Instant::now() + std::time::Duration::from_millis(a.timeout_ms);
    let mut seen = 0u32;
    let document = a.document.clone();
    let check = move |ctx: &mut Ctx<'_>| -> Option<Result<Outcome, String>> {
        // Route to the named document for the check, as a context routes an
        // engine-only command (`automation::queue::ENGINE_ONLY`).
        let (route, index) = match &document {
            Some(handle) => match ctx.app.docs.index_of_handle(handle) {
                Some(index) => (Some(ctx.app.docs.route_begin(index)), index),
                None => return Some(Err("document_unknown: no open document has that handle (closed, or never opened)".into())),
            },
            None => (None, ctx.app.docs.active_index()),
        };
        // Another tab's saves and PLM work neither delay this settle nor
        // count as this document's.
        let others = other_documents(ctx, index);
        let others: Vec<&str> = others.iter().map(String::as_str).collect();
        let info = frame_info_except(ctx, &others);
        let listing = (info.idle && seen + 1 > a.settle).then(|| crate::automation::cmd_history::listing_json(ctx));
        if let Some(visible) = route {
            ctx.app.docs.route_end(visible);
        }
        if info.poisoned {
            return Some(Err("session poisoned".into()));
        }
        if info.idle {
            seen += 1;
            if let Some(listing) = listing {
                let body = serde_json::json!({
                    "frame": info.frame,
                    "step": listing["step"],
                    "listing": listing["listing"],
                    "report": listing["report"],
                    "model": listing["model"],
                    "info": info,
                });
                return Some(Ok(Outcome::Done(body)));
            }
        } else {
            seen = 0;
            if web_time::Instant::now() >= deadline {
                let progress = serde_json::to_value(&info.progress).unwrap_or(Value::Null);
                return Some(Err(format!(
                    "timeout after {} ms; still pending: {}; run progress: {progress}",
                    a.timeout_ms,
                    pending_names(&info.pending)
                )));
            }
        }
        None
    };
    // The first check is the queue's poll right after this dispatch; each
    // later one comes from a later service step or frame, so `settle`
    // counts checks spread over time, never one instant counted twice.
    let _ = ctx;
    Ok(Outcome::Poll(Box::new(check)))
}

pub static COMMANDS: &[CommandSpec] = &[
    app_command!("frame", "describe_commands", "The app's command registry: every command with its group, doc, phase, annotations and derived argument/result schemas. A host generates its tool list from this.", NoArgs, describe_commands, Read, READ, describe_commands_result_schema),
    app_command!("frame", "ping", "Liveness, and one frame: replies from the next pass's read phase (a drawn frame, or on an undrawn desktop window a pass the app runs itself). The composite tools step the app with it (`wait_frames`, `wait_state`, a dwell).", NoArgs, ping, Read, READ, ping_result_schema),
    app_command!("frame", "settle", "Wait, in the app, until it has been idle (`frame_info`'s `idle`) for `settle` further checks, then answer with the frame info and the settled history listing, report and model. The wait every mutating tool ends with; no polling.", SettleArgs, settle, Read, READ, settle_result_schema),
    app_command!("frame", "frame_info", "Frame counter, pixels-per-point, surface and viewport rects (egui points), pending runner work, run progress, history length and rollback step. `idle` is the wait condition for mutating tools.", NoArgs, frame_info_cmd, Read, READ, schema_of::<FrameInfo>),
];
