//! The per-frame timing record as commands: read the rolling window, clear it,
//! and drive the one interaction the record exists to measure — a camera spin.
//!
//! A measurement is "reset, spin, read". Without the reset the window still
//! holds the idle frames that preceded the spin, and an idle frame is cheap, so
//! their average hides exactly the cost being looked for. See [`crate::perf`]
//! for what each phase brackets.
//!
//! The spin is stepped by the APP, one pointer move per frame
//! ([`crate::app::BrepApp::start_spin`]), not by the host issuing one move per
//! call: a host-driven drag puts a round trip between every pair of frames, and
//! `dt` would then report the host's latency instead of the app's frame period.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Outcome, Phase};
use serde::Deserialize;
use serde_json::Value;

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PerfArgs {
    /// Drop every sample AFTER reading, so the next read covers only what
    /// happened since this call.
    #[serde(default)]
    pub reset: bool,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SpinArgs {
    /// How many frames to orbit for. The timing window holds 120, so a spin
    /// shorter than that leaves pre-spin frames in the average.
    #[serde(default = "default_frames")]
    pub frames: u32,
    /// Pointer step per frame in logical px, `[dx, dy]`. The default sweeps
    /// horizontally, which turns the camera about the up axis.
    #[serde(default)]
    pub step: Option<[f64; 2]>,
    /// Clear the timing window as the spin starts (default true), so the read
    /// afterwards is the spin and nothing else.
    #[serde(default = "default_true")]
    pub reset: bool,
}

fn default_frames() -> u32 {
    240
}

fn default_true() -> bool {
    true
}

fn perf_stats(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let args: PerfArgs = parse_args(args)?;
    let json = crate::perf::json();
    let runs = brep_render::run_trace::tally_json();
    if args.reset {
        crate::perf::reset();
        brep_render::run_trace::reset();
    }
    let mut value: Value = serde_json::from_str(&json).map_err(|e| e.to_string())?;
    // The poll condition: a host reads until the spin it asked for is over.
    if let Some(map) = value.as_object_mut() {
        map.insert("spinning".into(), Value::Bool(ctx.app.spinning()));
        // The history-replay tally. Frame milliseconds cannot tell a duplicate
        // replay from a slow machine; this counts the executions themselves.
        map.insert("runs".into(), runs);
    }
    Ok(Outcome::Done(value))
}

fn perf_spin(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let args: SpinArgs = parse_args(args)?;
    let [dx, dy] = args.step.unwrap_or([3.0, 0.0]);
    if args.reset {
        crate::perf::reset();
        brep_render::run_trace::reset();
    }
    ctx.app.start_spin(args.frames, (dx, dy));
    Ok(Outcome::Done(serde_json::json!({ "frames": args.frames, "step": [dx, dy] })))
}

fn perf_reset(_ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    crate::perf::reset();
    brep_render::run_trace::reset();
    Ok(Outcome::Done(serde_json::json!({})))
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "perf_stats", group: "frame", doc: "Where the last ~120 frames' milliseconds went, in ms. `dt` is the frame PERIOD a user feels; `ui` brackets the whole UI body; `publish`, `sync`, `fit`, `overlays` and `draw` are nested inside it (the remainder is egui's own layout and painting). Each is {avg, p95, max}. `spinning` says whether a `perf_spin` is still running — poll this until it is false, then read. `runs` is the history-replay tally since the last reset: one {count, ms, features, reused, errors} per SITE that called `execute_history`, plus their total. `runs.sites.runner` is the one execution an edit is supposed to cost, and any other site is a duplicate replay of the whole document — a thing frame milliseconds cannot state, because a duplicate and a slow machine look alike. Bracket an interaction with `perf_reset` and read this after it.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<PerfArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(perf_stats) },
    CommandSpec { name: "perf_spin", group: "frame", doc: "Orbit the camera for `frames` frames, one pointer step per frame, through the same drag path the mouse takes. Clears the timing window and the history-replay counts first (unless `reset` is false), so `perf_stats` afterwards reports the spin alone. Returns at once — poll `perf_stats.spinning`.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<SpinArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(perf_spin) },
    CommandSpec { name: "perf_reset", group: "frame", doc: "Drop every timing sample and every history-replay count. Use before an interaction whose cost is the question; `perf_stats` afterwards then reports that interaction alone, in frame milliseconds AND in `execute_history` calls.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(perf_reset) },
];
