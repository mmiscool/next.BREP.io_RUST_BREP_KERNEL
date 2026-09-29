//! Frame-level commands: liveness and the per-frame facts every wait loop
//! polls.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Outcome, Phase};
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
    /// A drawing sheet's exact hidden-line pass is on the runner (or waiting
    /// for it); its placement draws the mesh approximation until it lands.
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
        run: engine.run_pending(),
        queries: engine.queries_pending(),
        mesh_imports: engine.mesh_imports_pending(),
        step_probes: engine.step_probes_pending(),
        topology: engine.topology_pending(),
        projection: engine.sheet_lines_pending(),
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
    CommandSpec { name: "describe_commands", group: "frame", doc: "The app's command registry: every command with its group, doc, phase, annotations and derived argument/result schemas. A host generates its tool list from this.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(describe_commands) },
    CommandSpec { name: "ping", group: "frame", doc: "Liveness: replies from the next frame's read phase.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(ping) },
    CommandSpec { name: "frame_info", group: "frame", doc: "Frame counter, pixels-per-point, surface and viewport rects (egui points), pending runner work, run progress, history length and rollback step. `idle` is the wait condition for mutating tools.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<FrameInfo>, handler: Handler::App(frame_info_cmd) },
];
