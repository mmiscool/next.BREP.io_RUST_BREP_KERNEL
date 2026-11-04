//! Frame-level commands: liveness and the per-frame facts every wait loop
//! polls.
use crate::automation::command::{app_command, parse_args, schema_of, CommandSpec, Ctx, NoArgs, Outcome};
use serde::Serialize;
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
        store_writes: ctx.app.model_store().pending_writes() > 0 || store_mirror_in_flight(),
        plm_requests: plm_requests_in_flight() || ctx.app.plm.busy() || ctx.app.toolbar.inbox().busy(),
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

/// Anything unfinished on the native PLM executor.
fn plm_requests_in_flight() -> bool {
    #[cfg(not(target_arch = "wasm32"))]
    {
        crate::plm::native::pending() > 0
    }
    #[cfg(target_arch = "wasm32")]
    {
        false
    }
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

pub static COMMANDS: &[CommandSpec] = &[
    app_command!("frame", "describe_commands", "The app's command registry: every command with its group, doc, phase, annotations and derived argument/result schemas. A host generates its tool list from this.", NoArgs, describe_commands, Read, READ, describe_commands_result_schema),
    app_command!("frame", "ping", "Liveness: replies from the next frame's read phase.", NoArgs, ping, Read, READ, ping_result_schema),
    app_command!("frame", "frame_info", "Frame counter, pixels-per-point, surface and viewport rects (egui points), pending runner work, run progress, history length and rollback step. `idle` is the wait condition for mutating tools.", NoArgs, frame_info_cmd, Read, READ, schema_of::<FrameInfo>),
];
