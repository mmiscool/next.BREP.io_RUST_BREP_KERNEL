//! `cam_sim_*` automation (lane C17, row 4): the simulation player of the
//! CAM pane by command — load, state, play/pause, scrub, step, settings,
//! the deviation comparison and the STL export.

use serde::Deserialize;
use serde_json::{json, Value};

use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Outcome, Phase};
use crate::panels::cam;

#[derive(Debug, Default, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct LoadArgs {
    /// The setup to simulate (`None`: the setup of the first generated operation).
    #[serde(default)]
    pub setup: Option<String>,
    /// A machine (the model's `Machine`, camelCase) whose `collision` solids
    /// are checked along the resolved motion; kept for later loads.
    #[serde(default)]
    pub machine: Option<Value>,
    /// Simulate on the UI thread instead of the background runner
    /// (`inline` is a name the WebMCP adapter reserves for screenshots).
    #[serde(default)]
    pub ui_thread: Option<bool>,
    /// Toolpaths (the model's `Toolpath`, camelCase) that replace operations'
    /// generated ones by operation id, kept for later loads; `{}` clears them.
    /// An operation given one needs no generation result; an omitted
    /// `report.status` reads `ok` (an `error` status is refused at load).
    #[serde(default)]
    pub toolpaths: Option<serde_json::Map<String, Value>>,
}

#[derive(Debug, Default, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct PlayArgs {
    /// `true` plays, `false` pauses; omitted toggles.
    #[serde(default)]
    pub play: Option<bool>,
}

#[derive(Debug, Default, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct ScrubArgs {
    /// Fraction of the program's time, `0..=1`.
    #[serde(default)]
    pub fraction: Option<f64>,
    /// Or a motion index to jump to the start of.
    #[serde(default)]
    pub motion: Option<usize>,
    /// Or an absolute time in seconds.
    #[serde(default)]
    pub seconds: Option<f64>,
}

#[derive(Debug, Default, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct StepArgs {
    #[serde(default)]
    pub back: bool,
}

#[derive(Debug, Default, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct SetArgs {
    #[serde(default)]
    pub speed: Option<f64>,
    #[serde(default)]
    pub stop_at_collision: Option<bool>,
    #[serde(default)]
    pub show_deviation: Option<bool>,
    /// Colour the stock by remaining material at the playback point.
    #[serde(default)]
    pub show_remaining: Option<bool>,
    /// Dexel cell size for the next load.
    #[serde(default)]
    pub cell: Option<f64>,
    /// Motions simulated per frame.
    #[serde(default)]
    pub budget_per_frame: Option<usize>,
}

#[derive(Debug, Default, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct DeviationArgs {
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Colour saturation distance beyond the tolerance.
    #[serde(default)]
    pub scale: Option<f64>,
    /// List up to this many excess samples (largest first) under `samples`.
    #[serde(default)]
    pub samples: Option<usize>,
}

fn session_mut<'a>(ctx: &'a mut Ctx<'_>) -> Result<&'a mut crate::panels::cam_sim::SimSession, String> {
    ctx.app.cam.sim.session.as_mut().ok_or_else(|| "no simulation loaded (cam_sim_load first)".to_string())
}

fn cam_sim_load(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: LoadArgs = parse_args(args)?;
    if let Some(m) = a.machine {
        let machine: brep_cam::model::Machine = serde_json::from_value(m).map_err(|e| format!("machine: {e}"))?;
        machine.validate().map_err(|e| format!("machine: {e}"))?;
        ctx.app.cam.sim.machine = Some(machine);
    }
    if let Some(map) = a.toolpaths {
        let mut toolpaths = std::collections::BTreeMap::new();
        for (id, v) in map {
            let mut tp: brep_cam::model::Toolpath = serde_json::from_value(v).map_err(|e| format!("toolpaths.{id}: {e}"))?;
            // A given path is usable: an omitted report status reads `ok`.
            if tp.report.status == brep_cam::model::OperationStatus::Pending {
                tp.report.status = brep_cam::model::OperationStatus::Ok;
            }
            toolpaths.insert(id, tp);
        }
        ctx.app.cam.sim.toolpaths = toolpaths;
    }
    if let Some(ui_thread) = a.ui_thread {
        ctx.app.cam.sim.force_inline = ui_thread;
    }
    let doc = cam::cam_document(ctx.app.docs.engine())?;
    ctx.app.cam.sim.load(ctx.app.docs.engine_mut(), &doc, a.setup.as_deref())?;
    Ok(Outcome::Done(ctx.app.cam.sim.state_json()))
}

fn cam_sim_unload(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    ctx.app.cam.sim.unload(ctx.app.docs.engine_mut());
    Ok(Outcome::Done(json!({})))
}

fn cam_sim_cancel(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let stopped = ctx.app.cam.sim.cancel(ctx.app.docs.engine_mut());
    Ok(Outcome::Done(json!({ "stopped": stopped, "state": ctx.app.cam.sim.state_json() })))
}

fn cam_sim_state(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(ctx.app.cam.sim.state_json()))
}

fn cam_sim_play(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: PlayArgs = parse_args(args)?;
    let s = session_mut(ctx)?;
    match a.play {
        Some(true) => s.player.play(),
        Some(false) => s.player.pause(),
        None => s.player.toggle(),
    }
    Ok(Outcome::Done(json!({ "playing": s.player.state == brep_cam::multiaxis::sim::PlayState::Playing })))
}

fn cam_sim_scrub(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ScrubArgs = parse_args(args)?;
    let s = session_mut(ctx)?;
    s.player.pause();
    match (a.fraction, a.motion, a.seconds) {
        (Some(f), _, _) => s.player.scrub(f),
        (_, Some(m), _) => s.player.seek_motion(m),
        (_, _, Some(t)) => s.player.scrub_time(t),
        _ => return Err("cam_sim_scrub needs `fraction`, `motion` or `seconds`".into()),
    }
    let pose = s.player.pose().map(|(tip, axis)| json!({ "tip": [tip.x, tip.y, tip.z], "axis": [axis.x, axis.y, axis.z] }));
    Ok(Outcome::Done(json!({ "motion": s.player.position.motion, "fraction": s.player.position.fraction, "time": s.player.time(), "completed": s.player.completed(), "pose": pose })))
}

fn cam_sim_step(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: StepArgs = parse_args(args)?;
    let s = session_mut(ctx)?;
    s.player.pause();
    if a.back {
        s.player.step_back();
    } else {
        s.player.step_forward();
    }
    Ok(Outcome::Done(json!({ "motion": s.player.position.motion, "fraction": s.player.position.fraction, "completed": s.player.completed() })))
}

fn cam_sim_set(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: SetArgs = parse_args(args)?;
    let sim = &mut ctx.app.cam.sim;
    if let Some(speed) = a.speed {
        if !speed.is_finite() || speed <= 0.0 {
            return Err("speed must be finite and positive".into());
        }
        sim.speed = speed;
        if let Some(s) = sim.session.as_mut() {
            s.player.set_speed(speed).map_err(|e| e.to_string())?;
        }
    }
    if let Some(stop) = a.stop_at_collision {
        sim.stop_at_collision = stop;
        if let Some(s) = sim.session.as_mut() {
            s.player.stop_at_collision = stop;
        }
    }
    if let Some(show) = a.show_deviation {
        sim.show_deviation = show;
    }
    if let Some(show) = a.show_remaining {
        sim.show_remaining = show;
    }
    if let Some(cell) = a.cell {
        if !cell.is_finite() || cell <= 0.0 {
            return Err("cell must be finite and positive".into());
        }
        sim.cell = cell;
    }
    if let Some(b) = a.budget_per_frame {
        if b == 0 {
            return Err("budget must be positive".into());
        }
        sim.budget_per_frame = b;
        sim.budget_pinned = true;
    }
    Ok(Outcome::Done(sim.state_json()))
}

/// The excess samples of the simulated stock, largest first, at most `cap`,
/// and how many there were. A tri-dexel stock is read from all six sides
/// (multi-axis parts have faces turned every way); a Z-dexel stock from +Z,
/// the map `compare` just built. Points are given in machine coordinates (`p`,
/// `nearest`, as toolpaths are) and model coordinates (`pModel`).
fn excess_sample_list(s: &crate::panels::cam_sim::SimSession, options: &brep_cam::verify::DeviationOptions, cap: usize) -> Result<(Vec<Value>, usize), String> {
    use brep_cam::multiaxis::sim::StockModel;
    let part = s.part.as_ref().ok_or("no part mesh: the setup's target bodies are not on screen")?;
    let mut samples: Vec<(&str, brep_cam::verify::ExcessSample)> = match &s.driver.stock {
        StockModel::TriDexel(td) => brep_cam::multiaxis::sim::deviation::excess_samples_all_views(td, part, options)
            .map_err(|e| e.to_string())?
            .into_iter()
            .map(|v| (v.view, v.sample))
            .collect(),
        _ => {
            let map = s.deviation.as_ref().ok_or("no deviation map")?;
            let index = brep_cam::BruteForceIndex::build(part);
            brep_cam::verify::excess_samples(map, part, &index, options).into_iter().map(|x| ("+Z", x)).collect()
        }
    };
    let total = samples.len();
    samples.sort_by(|a, b| b.1.excess.total_cmp(&a.1.excess));
    samples.truncate(cap);
    let xyz = |p: brep_cam::model::Point3| json!([p.x, p.y, p.z]);
    let list = samples
        .into_iter()
        .map(|(view, x)| {
            let face = &part.faces[x.face as usize];
            json!({ "view": view, "p": xyz(x.point), "pModel": xyz(s.to_model.apply_point(x.point)), "excess": x.excess, "solid": face.solid, "face": face.face, "nearest": xyz(x.nearest) })
        })
        .collect();
    Ok((list, total))
}

fn cam_sim_deviation(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: DeviationArgs = parse_args(args)?;
    let mut options = brep_cam::verify::DeviationOptions::default();
    if let Some(t) = a.tolerance {
        options.tolerance = t;
    }
    let scale = a.scale.unwrap_or(1.0);
    let s = session_mut(ctx)?;
    s.compare(&options, scale)?;
    let map = s.deviation.as_ref().unwrap();
    let faces: Vec<Value> = map
        .faces
        .iter()
        .map(|f| json!({ "solid": f.solid, "face": f.face_name, "samples": f.samples, "min": f.min, "max": f.max, "mean": f.mean, "gouged": f.gouged, "excess": f.excess }))
        .collect();
    let out = json!({
        "legend": s.legend,
        "faces": faces,
        "columns": map.values.iter().filter(|v| v.is_some()).count(),
        "belowPart": map.below_part,
        "belowPartVolume": map.below_part_volume,
    });
    let mut out = out;
    if let Some(cap) = a.samples {
        let (list, total) = excess_sample_list(s, &options, cap)?;
        out["samples"] = Value::Array(list);
        out["samplesTotal"] = json!(total);
    }
    ctx.app.cam.sim.show_deviation = true;
    Ok(Outcome::Done(out))
}

#[derive(Debug, Default, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct PostTextArgs {
    /// The NC program (`NC<n>`, see cam_state ncPrograms).
    pub id: String,
    /// Also hand the file to the user as the pane's Save would (the
    /// Save-file chooser on desktop, a download in the browser).
    #[serde(default)]
    pub save: bool,
}

fn cam_post_text(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: PostTextArgs = parse_args(args)?;
    let doc = cam::cam_document(ctx.app.docs.engine())?;
    let posted = ctx.app.cam.post.open(ctx.app.docs.engine(), &doc, &a.id)?.to_json();
    if a.save {
        ctx.app.cam.post.save();
    }
    Ok(Outcome::Done(posted))
}

fn cam_sim_export(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let stl = ctx.app.cam.sim.export_stl().ok_or_else(|| "no simulation loaded (cam_sim_load first)".to_string())?;
    let triangles = stl.matches("facet normal").count();
    Ok(Outcome::Done(json!({ "stl": stl, "triangles": triangles })))
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "cam_sim_load", group: "cam", doc: "Load the simulation of a setup's generated operations (`setup`, or the setup of the first generated operation): the operations are assembled into one program, the stock comes from the setup (box, cylinder, a solid on screen) as C7's Z-dexel grid or, when a tool axis tilts, the tri-dexel; C7's checked simulation marks collisions and gouges of 3-axis streams; with `machine` (the model's Machine with `collision` link solids) the resolved machine motion is checked against the machine and the stock and the first hit is marked. The machine (given, else the setup's) supplies the kinematic chain: indexed and rotary streams (RotaryPosition, LinearRotary) are cut in the part frame on the Z-dexel through C7's simulate_on_machine; without a machine a nonzero rotary word is reported unsupported. `toolpaths` replaces operations' generated toolpaths by id for the session (an operation given one needs no generation). The stock lives on the background runner (advanced in chunks of up to 8 × budgetPerFrame motions, exactly budgetPerFrame once `cam_sim_set` sets it; `session/runner` reports it) unless `uiThread: true` or the runner cannot. Returns the state (see cam_sim_state). Refused when nothing is generated.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<LoadArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_sim_load) },
    CommandSpec { name: "cam_sim_cancel", group: "cam", doc: "Stop simulating (the pane's Stop): the chunk on the background runner is cancelled between two motions where the runner can (the native thread) and its reply dropped (in the browser the chunk finishes, bounded by the runner chunk size, and is dropped); the stock stays the last accepted one and the transport pauses at it. Returns whether anything was stopped and the state.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_sim_cancel) },
    CommandSpec { name: "cam_sim_unload", group: "cam", doc: "Drop the loaded simulation and its viewport groups.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_sim_unload) },
    CommandSpec { name: "cam_sim_state", group: "cam", doc: "The player's state: loaded, speed, stopAtCollision, cell, and under `session` the setup, multiAxis, playing, motion/fraction/time/progress, motions and completed counts, applied (motions the stock has absorbed) and caughtUp, removedVolume, stockVolume, unsupported (tilted motions the Z-dexel skipped), the operation at the position and every operation's range and seconds, marks and collisionMotions, the deviation legend once compared, stockVersion, and with showRemaining the remaining-stock legend under `remaining` (tolerance, bands with their mm edges and colours, vertex counts within/excess/gouge/excessHigh, maxExcess, minValue, vertices, the stockVersion it was computed for).", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_sim_state) },
    CommandSpec { name: "cam_sim_play", group: "cam", doc: "Play (`play: true`), pause (`false`) or toggle the simulation; playback advances by frame time × speed and the stock follows in chunks of at most `budgetPerFrame` motions per frame on the UI thread, up to 8 × that per runner chunk; with stopAtCollision it pauses at the first collision motion entered.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<PlayArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_sim_play) },
    CommandSpec { name: "cam_sim_scrub", group: "cam", doc: "Pause and jump: `fraction` of the program's time, `motion` index (its start) or `seconds`. Returns the position, the motions completed (the stock catches up over the following frames: `wait_state` on `/__brepCamSim/session/caughtUp`) and the tool pose (tip and axis, machine coordinates).", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<ScrubArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_sim_scrub) },
    CommandSpec { name: "cam_sim_step", group: "cam", doc: "Pause and step to the end of the current motion (or the next), or with `back` to its start (or the previous).", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<StepArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_sim_step) },
    CommandSpec { name: "cam_sim_set", group: "cam", doc: "Settings: `speed` (× machining time), `stopAtCollision`, `showDeviation`, `showRemaining` (colour the stock at the playback point by the material left over the part: red below -tol, green within ±tol, blue to 10·tol, yellow above, tol the operation tolerance there; read under `session/remaining`), `cell` (dexel cell for the next load), `budgetPerFrame` (motions per frame; setting it also pins each runner chunk to exactly that many). Returns the state.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<SetArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_sim_set) },
    CommandSpec { name: "cam_sim_deviation", group: "cam", doc: "Compare the simulated stock (as it stands) with the setup's target bodies through C7's deviation map: per part face the samples, min, max, mean, gouged and excess counts; belowPart and belowPartVolume (columns of stock under the part's lowest surface, never attributed to a face); the legend (min, max, tolerance, colour stops); the colour map is shown on the stock. `tolerance` (0.05) and `scale` (1.0, colour saturation beyond the tolerance). `samples: N` also lists the N largest excess samples (`samples`: view, stock point `p` and `pModel`, `excess`, nearest `solid`/`face` and `nearest` part point; machine coordinates unless named) and `samplesTotal`; a tri-dexel (multi-axis) stock is read from all six sides (views +Z, -Z, +X, -X, +Y, -Y), a Z-dexel stock from +Z.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<DeviationArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_sim_deviation) },
    CommandSpec { name: "cam_post_text", group: "cam", doc: "Post NC program `id` (the pane's Post on its row): its operations, in the program's order, are assembled with their generated toolpaths (each must be generated and current) and run through the program's built-in post; the preview opens in the CAM pane. Returns `text`, `fileName` (where it would save: the program name with the post's extension), `post`, `operations`, `warnings` and `lines`. `save: true` also hands the file to the user as the pane's Save does. `cam_post` (the MCP tool) writes it to a path.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<PostTextArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_post_text) },
    CommandSpec { name: "cam_sim_export", group: "cam", doc: "The simulated stock as ASCII STL text in model coordinates (what the pane's Export stock button saves) and its triangle count.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(cam_sim_export) },
];
