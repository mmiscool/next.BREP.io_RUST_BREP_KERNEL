//! Motion-study playback (`brep_render::engine_state` `simulation_play`):
//! play, pause, stop, scrub and loop the active assembly's study. The study
//! is baked once (its frames solved in order through the kernel, time-sliced
//! inside the app's frames) and every command reads the same baked frames;
//! nothing is written to the document.
use crate::automation::command::{app_command, parse_args, CommandSpec, Ctx, NoArgs, Outcome};
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PlayArgs {
    /// The study id; the active study when absent.
    #[serde(default)]
    pub study: Option<String>,
    /// Repeat at the end (kept until changed).
    #[serde(default, rename = "loop")]
    pub looping: Option<bool>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ExportArgs {
    /// The study id; the active study when absent.
    #[serde(default)]
    pub study: Option<String>,
    /// `gif`, `mp4` or `html` (the interactive player as one self-contained page).
    pub format: String,
    /// Pixels, 16..=4096 a side (MP4: even); 960×540 when absent. Not used by `html`.
    #[serde(default)]
    pub width: Option<u32>,
    #[serde(default)]
    pub height: Option<u32>,
    /// Frames per second the video plays at; the study's when absent.
    #[serde(default)]
    pub fps: Option<u32>,
    /// Where to write the file (native path; a relative one lands in the app's
    /// working folder). Without it the bytes come back as `base64`.
    #[serde(default)]
    pub path: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SeekArgs {
    /// The study id; the active study when absent.
    #[serde(default)]
    pub study: Option<String>,
    /// Seconds from the study's start (clamped to its duration).
    pub time: f64,
}

fn playback(ctx: &mut Ctx<'_>) -> Value {
    ctx.app.docs.engine().simulation_playback_json()
}

fn simulation_playback(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(playback(ctx)))
}

fn simulation_play(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: PlayArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().simulation_play(a.study.as_deref(), a.looping)?;
    ctx.app.show_pane(crate::panels::dock::PaneKind::Simulation);
    Ok(Outcome::Done(playback(ctx)))
}

fn simulation_pause(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().simulation_pause();
    Ok(Outcome::Done(playback(ctx)))
}

fn simulation_stop(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().simulation_stop();
    Ok(Outcome::Done(playback(ctx)))
}

fn simulation_seek(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: SeekArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().simulation_seek(a.study.as_deref(), a.time)?;
    ctx.app.show_pane(crate::panels::dock::PaneKind::Simulation);
    Ok(Outcome::Done(playback(ctx)))
}

/// Write `bytes` to `path` (native; the parent folders made) and name it.
fn write_export(path: &str) -> Result<Option<std::path::PathBuf>, String> {
    let path = path.trim();
    if path.is_empty() {
        return Ok(None);
    }
    let path = std::path::PathBuf::from(path);
    if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent).map_err(|e| format!("{}: {e}", parent.display()))?;
    }
    Ok(Some(path))
}

fn simulation_export(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ExportArgs = parse_args(args)?;
    if a.format.eq_ignore_ascii_case("html") {
        // The interactive player: no GPU, every target.
        let (html, data) = ctx.app.docs.engine_mut().simulation_html(a.study.as_deref())?;
        let bytes = html.into_bytes();
        let frames = data.motion.as_ref().map_or(0, |m| m.frames.len());
        let path = match a.path.as_deref().and_then(|p| write_export(p).transpose()) {
            Some(path) => {
                let path = path?;
                std::fs::write(&path, &bytes).map_err(|e| format!("{}: {e}", path.display()))?;
                Some(path.display().to_string())
            }
            None => None,
        };
        let head = String::from_utf8_lossy(&bytes[..bytes.len().min(16)]).into_owned();
        let mut out = json!({ "study": data.title, "format": "html", "frames": frames, "bytes": bytes.len(), "head": head, "fps": data.motion.as_ref().map_or(0, |m| m.fps) });
        match path {
            Some(path) => out["path"] = Value::String(path),
            None => out["base64"] = Value::String(crate::automation::cmd_document::base64_encode(&bytes)),
        }
        return Ok(Outcome::Done(out));
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        let spec = crate::simulation_export::ExportSpec { study: a.study, format: a.format, width: a.width.unwrap_or(960), height: a.height.unwrap_or(540), fps: a.fps, path: a.path };
        let (report, bytes) = crate::simulation_export::export(ctx.app, &spec)?;
        let mut out = serde_json::to_value(&report).map_err(|e| e.to_string())?;
        if report.path.is_none() {
            out["base64"] = Value::String(crate::automation::cmd_document::base64_encode(&bytes));
        }
        Ok(Outcome::Done(out))
    }
    #[cfg(target_arch = "wasm32")]
    {
        Err(format!("{} export is native only: the web app plays studies and writes the html player, not video files", a.format))
    }
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PublishArgs {
    /// The study id; the active study when absent.
    #[serde(default)]
    pub study: Option<String>,
}

fn simulation_publish(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: PublishArgs = parse_args(args)?;
    let (study, frames) = ctx.app.docs.engine_mut().simulation_publish(a.study.as_deref())?;
    Ok(Outcome::Done(json!({ "study": study, "frames": frames })))
}

fn playback_object(properties: Value) -> Value {
    let required: Vec<_> = properties.as_object().unwrap().keys().cloned().collect();
    json!({"type":"object","properties":properties,"required":required})
}
fn simulation_publish_result_schema() -> Value {
    playback_object(json!({"study":{"type":"string"},"frames":{"type":"integer","minimum":0}}))
}

fn simulation_export_result_schema() -> Value {
    let mut variants = Vec::new();
    for format in ["gif", "mp4", "html"] {
        for destination in ["path", "base64"] {
            let mut properties = json!({
                "study":{"type":"string"},"format":{"const":format},
                "fps":{"type":"integer","minimum":0},"frames":{"type":"integer","minimum":0},
                "bytes":{"type":"integer","minimum":0},"head":{"type":"string"}
            });
            properties[destination] = json!({"type":"string", "description": if destination == "base64" { "Complete exported file encoded as base64." } else { "Path of the written export file." }});
            if format != "html" {
                properties["fps"] = json!({"type":"integer","minimum":1});
                properties["width"] = json!({"type":"integer","minimum":16,"maximum":4096});
                properties["height"] = properties["width"].clone();
            }
            let mut branch = playback_object(properties);
            branch["additionalProperties"] = json!(false);
            variants.push(branch);
        }
    }
    json!({"oneOf":variants})
}

pub(super) fn simulation_playback_result_schema() -> Value {
    let count = json!({"type":"integer","minimum":0});
    let string = json!({"type":"string"});
    let number = json!({"type":"number"});
    let strings = json!({"type":"array","items":string});
    let counts = json!({"type":"array","items":count});
    let vector = json!({"type":"array","items":{"type":["number","null"]},"minItems":3,"maxItems":3});
    let pose = playback_object(json!({"translate":vector,"rotateEulerDeg":vector}));
    let populated = playback_object(json!({
        "study":string,"time":number,"playheadFrame":count,"shown":{"type":["integer","null"],"minimum":0},
        "baked":count,"total":count,"playing":{"type":"boolean"},"looping":{"type":"boolean"},
        "failed":counts,"error":{"type":["string","null"]},
        "poses":{"type":"object","additionalProperties":pose},
        "interference":{"type":"boolean"},"clashes":counts,
        "shownClashes":{"type":"array","items":{"type":"array","items":string,"minItems":2,"maxItems":2}},
        "hidden":strings,"explode":{"type":"array","items":playback_object(json!({"view":string,"progress":number}))},
        "camera":{"anyOf":[{"type":"null"},playback_object(json!({"from":string,"to":string,"t":number}))]},
        "cameraHeld":{"type":"boolean"},"opacity":{"type":"object","additionalProperties":number},
        "section":{"anyOf":[{"type":"null"},playback_object(json!({"view":string,"offset":number}))]}
    }));
    // Native callers see null; the MCP adapter wraps that branch as value:null.
    json!({"oneOf":[{"type":"null"},populated]})
}

pub static COMMANDS: &[CommandSpec] = &[
    app_command!("simulation", "simulation_publish", "Publish a study (the active one): bake it to the end and write its motion into the assembly's `simulation` block as the study's `motion` — the player's frames (moved occurrences as pose deltas keyed by occurrence, hidden ones, opacities, camera, clashes) with `study` and `assemblySignature` (the assembly document's signature, its published motions left out) beside them — for a manufacturing-instructions step to attach (`instructions_attach_motion`). One undo step; the bake is kept. Returns `study` and `frames`.", PublishArgs, simulation_publish, Mutate, MUTATE, simulation_publish_result_schema),
    app_command!("simulation", "simulation_playback", "The motion-study playback: `study`, `time` (s), `playheadFrame` (the frame at the playhead; a reply's top-level `frame` is the app frame the command ran on), `shown` (the frame the displays show, null at rest), `baked` of `total` frames (the bake is time-sliced over the app's frames: wait for baked == total before reading a late frame), `playing`, `looping`, `failed` (frames the mates could not hold; they show the previous frame's poses), `error` (the last such frame's reason) `poses` (the occurrences the shown frame moved from their document poses, {translate, rotateEulerDeg}), `interference` (whether the study checks it), `clashes` (the baked frames where parts interfere: an edge of one part's mesh pierces a face of another's; contact is not a clash) and `shownClashes` (the [a, b] occurrence pairs clashing at the shown frame; their displays are tinted). Null when no study has been played or scrubbed.", NoArgs, simulation_playback, Read, READ, simulation_playback_result_schema),
    app_command!("simulation", "simulation_play", "Play a study (the active one) from its playhead — from 0 when it stands at the end — baking it first if need be; `loop` sets whether it repeats. Displays are posed frame by frame and the document is untouched. Shows the Simulation pane. Returns the playback.", PlayArgs, simulation_play, Mutate, MUTATE, simulation_playback_result_schema),
    app_command!("simulation", "simulation_pause", "Hold the playhead where it is; the frame stays shown. Returns the playback.", NoArgs, simulation_pause, Mutate, MUTATE, simulation_playback_result_schema),
    app_command!("simulation", "simulation_stop", "Stop: every display back at its document pose, the playhead at 0; the bake is kept. Returns the playback.", NoArgs, simulation_stop, Mutate, MUTATE, simulation_playback_result_schema),
    app_command!("simulation", "simulation_export", "Write a study (the active one) as a video or as the interactive player: `format` gif (every frame its own palette, loops), mp4 (H.264) or html (one self-contained page: the instructions player template with the study's frames on a timeline — every target, no GPU), `width`×`height` pixels (16..=4096; MP4 even; 960×540 when absent; not used by html), at `fps` (the study's when absent). The study is baked first if need be, every baked frame is drawn at that size through the viewport's renderer with the current display settings (poses, clips, camera, tint), and the playhead goes back where it stood. With `path` the file is written there (native) and the reply carries `path`, `bytes`, `frames` and `head` (the first bytes as text: `GIF89a`, or an MP4's `ftyp`); without it the file comes back as `base64`. Native only.", ExportArgs, simulation_export, Mutate, MUTATE, simulation_export_result_schema),
    app_command!("simulation", "simulation_seek", "Scrub: put the playhead at `time` seconds (clamped to the study) and show that frame, pausing. A frame not baked yet is shown once it is (read `baked` and `shown`). Shows the Simulation pane. Returns the playback.", SeekArgs, simulation_seek, Mutate, MUTATE, simulation_playback_result_schema),
];
