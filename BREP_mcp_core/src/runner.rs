//! The `test-mcp` script runner: plays a [`Script`] through a [`ToolSet`],
//! checks every `expect`, saves and compares images, and writes the per-step
//! artefacts (`NNNN-<tool>.json`, images, `result.json`) under an output
//! directory. It knows nothing about hosts or sessions: the tool set it is
//! handed is whatever the session generated, so a script written against the
//! running app and one written against the catalogue tools run the same way.
use crate::image;
use crate::script::{exit, Frame, Script, Step};
use crate::tools::ToolSet;
use serde::Serialize;
use serde_json::{json, Value};
use std::path::{Path, PathBuf};
use std::time::Instant;

#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Outcome {
    Pass,
    ExpectationFailed,
    ToolError,
    Timeout,
}

#[derive(Debug, Clone, Serialize)]
pub struct StepResult {
    pub index: usize,
    pub tool: String,
    pub outcome: Outcome,
    pub elapsed_ms: u128,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub failures: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub image: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub diff_ratio: Option<f64>,
}

/// What a script's walkthrough recording produced.
#[derive(Debug, Clone, Serialize)]
pub struct VideoResult {
    /// The GIF's path as the script declared it.
    pub out: String,
    /// Where it was actually written (one entry per destination).
    pub written: Vec<String>,
    pub frames: usize,
    pub bytes: usize,
    /// Pixel size of every frame.
    pub size: [u32; 2],
    /// How long the whole GIF plays, in milliseconds.
    pub duration_ms: u32,
    /// Encode time alone — not the capture, which is most of the wall clock.
    pub encode_ms: u128,
}

#[derive(Debug, Clone, Serialize)]
pub struct RunResult {
    pub script: String,
    pub passed: bool,
    pub exit_code: i32,
    pub steps: Vec<StepResult>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub failed_step: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub video: Option<VideoResult>,
}

/// The step the runner answers itself, from the script's PLM fixture.
pub const PLM_REQUEST: &str = "plm_request";

pub struct RunOptions {
    pub out_dir: PathBuf,
    pub baselines_dir: PathBuf,
    pub update_baselines: bool,
    pub stop_on_fail: bool,
    /// Where a script's `video.out` is resolved, when a walkthrough GIF is to
    /// reach the repository at all. `None` — every ordinary run, including the
    /// `test-mcp` gate — still records and encodes, but writes the GIF into the
    /// run's own output directory only: the gate cannot touch `docs/`.
    pub video_root: Option<PathBuf>,
    /// The script's PLM server, when it declares one: `plm_request` steps are
    /// answered from it, and `${plm:NAME}` resolves in every step's strings.
    pub plm: Option<std::sync::Arc<crate::plm_fixture::PlmFixture>>,
}

/// Play `script` through `tools`. Returns the result (also written to
/// `<out_dir>/result.json`).
pub async fn run(script: &Script, tools: &ToolSet, opts: &RunOptions) -> RunResult {
    let _ = std::fs::create_dir_all(&opts.out_dir);
    let deadline = Instant::now() + std::time::Duration::from_millis(script.timeout_ms);
    let mut steps = Vec::new();
    let mut failed_step = None;
    // THE WALKTHROUGH: every step carrying a `frame` contributes one held GIF
    // frame, recorded before the step runs. The total is counted up front so
    // the `n / total` counter on the band cannot drift from the script.
    let total_frames = script.steps.iter().filter(|s| s.frame.is_some()).count();
    let mut shots: Vec<image::GifFrame> = Vec::new();
    // The counter on the caption band is the step's ordinal among the framed
    // steps, NOT the GIF frame number: a FILMED drag contributes a still plus
    // its motion frames and is still ONE step of the walkthrough, so `shots`
    // cannot stand in for it.
    let mut framed = 0usize;
    let mut recording_failed = false;
    for (index, step) in script.steps.iter().enumerate() {
        if Instant::now() > deadline {
            steps.push(StepResult {
                index,
                tool: step.tool.clone(),
                outcome: Outcome::Timeout,
                elapsed_ms: 0,
                failures: vec![format!("script timeout ({} ms) reached before step {index}", script.timeout_ms)],
                image: None,
                diff_ratio: None,
            });
            failed_step = Some(index);
            break;
        }
        // A framed step is recorded one of two ways. An ORDINARY one is a single
        // still, captured before the step runs — the control unpressed and
        // ringed, with the caption saying what pressing it will do. A step whose
        // frame carries a `motion` is a DRAG, and it is FILMED: the annotation
        // is set here, the tool is asked to capture as it drags, and the
        // pictures it hands back become the frames. Either way the step is one
        // `n / total` on the band.
        //
        // The filming branch is gated on the script RECORDING as well as on the
        // frame: `Script::parse` refuses `motion` without a `video`, but a
        // `Script` built in code (the tests build them) would otherwise reach it
        // with no `capture` sent and fail on a picture count rather than simply
        // running the drag.
        let motion = script
            .video
            .as_ref()
            .and(step.frame.as_ref())
            .and_then(|f| f.motion.as_ref());
        let mut capture_args = None;
        if let (Some(video), Some(frame)) = (&script.video, &step.frame) {
            framed += 1;
            let recorded = match motion {
                None => capture_frame(tools, video, frame, framed, total_frames)
                    .await
                    .map(|shot| shots.push(shot)),
                Some(motion) => {
                    capture_args = Some(json!({ "capture": {
                        "frames": motion.frames,
                        "max_width": video.width,
                        "cursor_scale": motion.cursor_scale,
                    }}));
                    // Set ONCE, for the whole drag. The overlay re-resolves a
                    // `highlight` from the live publication on every frame it
                    // paints, so the ring FOLLOWS the handle as the drag moves
                    // it — and re-annotating per frame could fail mid-drag, with
                    // the button still down, on a handle whose pick momentarily
                    // answers something else.
                    set_annotation(tools, video, frame, framed, total_frames).await
                }
            };
            if let Err(why) = recorded {
                steps.push(StepResult {
                    index,
                    tool: "annotate+screenshot".into(),
                    outcome: Outcome::ToolError,
                    elapsed_ms: 0,
                    failures: vec![format!("recording the walkthrough frame before step {index}: {why}")],
                    image: None,
                    diff_ratio: None,
                });
                recording_failed = true;
                failed_step = Some(index);
                break;
            }
        }
        let (result, images) = run_step(index, step, tools, opts, capture_args).await;
        if let (Some(motion), Some(frame)) = (motion, step.frame.as_ref()) {
            // Clear the overlay whatever happened, so a failed drag cannot leave
            // the rest of the script captioned.
            if let Some(annotate) = tools.get("annotate") {
                let _ = (annotate.handler)(json!({})).await;
            }
            match motion_frames(&images, frame, motion) {
                Ok(mut filmed) => shots.append(&mut filmed),
                Err(why) => {
                    // The drag's OWN result goes in first: it says whether the
                    // drag itself worked, which is the first thing anyone
                    // reading `result.json` about a failed filming wants.
                    steps.push(result);
                    steps.push(StepResult {
                        index,
                        tool: "annotate+screenshot".into(),
                        outcome: Outcome::ToolError,
                        elapsed_ms: 0,
                        failures: vec![format!("filming the drag at step {index}: {why}")],
                        image: None,
                        diff_ratio: None,
                    });
                    recording_failed = true;
                    failed_step = Some(index);
                    break;
                }
            }
        }
        let failed = result.outcome != Outcome::Pass;
        steps.push(result);
        if failed {
            failed_step = Some(index);
            if opts.stop_on_fail {
                break;
            }
        }
    }
    let mut video = None;
    if let Some(spec) = &script.video {
        if !recording_failed {
            match write_video(spec, &shots, opts) {
                Ok(v) => video = Some(v),
                Err(why) => {
                    steps.push(StepResult {
                        index: script.steps.len(),
                        tool: "video".into(),
                        outcome: Outcome::ToolError,
                        elapsed_ms: 0,
                        failures: vec![why],
                        image: None,
                        diff_ratio: None,
                    });
                    failed_step.get_or_insert(script.steps.len());
                }
            }
        }
    }
    let exit_code = steps
        .iter()
        .map(|s| match s.outcome {
            Outcome::Pass => exit::PASS,
            Outcome::ExpectationFailed => exit::EXPECTATION_FAILED,
            Outcome::ToolError => exit::TOOL_ERROR,
            Outcome::Timeout => exit::TIMEOUT,
        })
        .max()
        .unwrap_or(exit::PASS);
    let result = RunResult {
        script: script.name.clone(),
        passed: exit_code == exit::PASS,
        exit_code,
        steps,
        failed_step,
        video,
    };
    if let Ok(text) = serde_json::to_string_pretty(&result) {
        let _ = std::fs::write(opts.out_dir.join("result.json"), text);
    }
    result
}

/// Record ONE walkthrough frame: set the annotation, capture the surface,
/// clear the annotation again.
///
/// The capture goes through the ordinary `screenshot` tool at the video's
/// width, so the frame is the tool's inline image — already downscaled, with
/// the overlay scaled with it because the overlay was painted by the app's own
/// egui context before the pixels existed.
///
/// The annotation is CLEARED afterwards so the session is left as it was found
/// and an ordinary `save`/`compare` screenshot elsewhere in the same script is
/// not silently captioned.
async fn capture_frame(
    tools: &ToolSet,
    video: &crate::script::Video,
    frame: &Frame,
    n: usize,
    total: usize,
) -> Result<image::GifFrame, String> {
    let screenshot = tools.get("screenshot").ok_or("no `screenshot` tool in this session")?;
    set_annotation(tools, video, frame, n, total).await?;
    let shot = (screenshot.handler)(json!({
        "region": "full",
        "max_width": video.width,
        "cursor": false
    }))
    .await?;
    let png = &shot.images.first().ok_or("the screenshot tool produced no image")?.png;
    let image = image::decode_png(png)?;
    let annotate = tools.get("annotate").ok_or("no `annotate` tool in this session")?;
    (annotate.handler)(json!({})).await?;
    Ok(image::GifFrame { image, hold_ms: frame.hold_ms })
}

/// Set the overlay the host paints over the app's next frames — the caption
/// band, the `n / total` counter and the rings — and LEAVE it set.
///
/// Its own function because a FILMED drag sets it once and captures many frames
/// under it, where an ordinary still sets it, captures, and clears it again in
/// one go.
async fn set_annotation(
    tools: &ToolSet,
    video: &crate::script::Video,
    frame: &Frame,
    n: usize,
    total: usize,
) -> Result<(), String> {
    let annotate = tools
        .get("annotate")
        .ok_or("no `annotate` tool in this session: a walkthrough needs a live app host")?;
    let mut spec = json!({ "caption": frame.caption, "step": [n, total] });
    if let Some(title) = &video.title {
        spec["title"] = json!(title);
    }
    if let Some(highlight) = &frame.highlight {
        spec["highlight"] = highlight.clone();
    }
    if let Some(entity) = &frame.entity {
        spec["entity"] = entity.clone();
    }
    if let Some(point) = &frame.point {
        spec["point"] = point.clone();
    }
    (annotate.handler)(spec).await?;
    Ok(())
}

/// Turn the pictures a FILMED drag handed back into walkthrough frames.
///
/// The tool returns `motion.frames + 1` of them: the pointer parked on the
/// handle with the button still up, then one per waypoint as it travels. The
/// first is held for the frame's own `hold_ms` — it is the still a reader needs
/// in order to see WHAT is about to be dragged — and the rest for the motion's,
/// which is short, because that stretch is a film rather than a slide.
///
/// A count that does not match is a hard error rather than a quietly shorter
/// video: it means the tool did not honour the `capture` it was given, and the
/// whole point of the step was the movement.
fn motion_frames(
    images: &[crate::tools::ToolImage],
    frame: &Frame,
    motion: &crate::script::Motion,
) -> Result<Vec<image::GifFrame>, String> {
    let expected = motion.frames as usize + 1;
    if images.len() != expected {
        return Err(format!(
            "the drag returned {} pictures, expected {expected} (the still plus {} motion frames) — does this tool take a `capture`?",
            images.len(),
            motion.frames
        ));
    }
    images
        .iter()
        .enumerate()
        .map(|(i, img)| {
            Ok(image::GifFrame {
                image: image::decode_png(&img.png)?,
                hold_ms: if i == 0 { frame.hold_ms } else { motion.hold_ms },
            })
        })
        .collect()
}

/// Encode the recorded frames and write the GIF.
///
/// Always into the run's own output directory; additionally to
/// `<video_root>/<out>` when a video root was given. Without one the gate
/// cannot write into the checkout, which is deliberate: `test-mcp` runs every
/// script and must not rewrite documentation assets as a side effect.
fn write_video(spec: &crate::script::Video, shots: &[image::GifFrame], opts: &RunOptions) -> Result<VideoResult, String> {
    if shots.is_empty() {
        return Err(format!(
            "the script declares a video (`{}`) but no step carries a `frame`, so there is nothing to record",
            spec.out
        ));
    }
    // `out` is joined to a root the caller chose, so it stays a relative path
    // INSIDE it: a script is data, and a `..` in it would let one write outside
    // the tree the operator pointed the recorder at.
    let out = Path::new(&spec.out);
    if out.is_absolute() || out.components().any(|c| matches!(c, std::path::Component::ParentDir)) {
        return Err(format!("video.out `{}` must be a relative path with no `..`", spec.out));
    }
    let started = Instant::now();
    let bytes = image::encode_gif(shots, spec.speed)?;
    let encode_ms = started.elapsed().as_millis();
    let name = out
        .file_name()
        .ok_or_else(|| format!("video.out `{}` has no file name", spec.out))?;
    let mut written = Vec::new();
    let local = opts.out_dir.join(name);
    std::fs::write(&local, &bytes).map_err(|e| format!("{}: {e}", local.display()))?;
    written.push(local.display().to_string());
    if let Some(root) = &opts.video_root {
        let dst = root.join(&spec.out);
        if let Some(parent) = dst.parent() {
            std::fs::create_dir_all(parent).map_err(|e| format!("{}: {e}", parent.display()))?;
        }
        std::fs::write(&dst, &bytes).map_err(|e| format!("{}: {e}", dst.display()))?;
        written.push(dst.display().to_string());
    }
    let (w, h) = shots[0].image.dimensions();
    Ok(VideoResult {
        out: spec.out.clone(),
        written,
        frames: shots.len(),
        bytes: bytes.len(),
        size: [w, h],
        duration_ms: shots.iter().map(|f| f.hold_ms).sum(),
        encode_ms,
    })
}

/// Run one step and hand back its result together with every image the tool
/// produced.
///
/// The images come back rather than being dropped because a FILMED drag returns
/// its motion frames that way — [`run`] folds them into the walkthrough.
/// `extra_args` is merged over the step's own, which is how the recorder asks a
/// drag to film itself without the script having to know the video's width.
async fn run_step(
    index: usize,
    step: &Step,
    tools: &ToolSet,
    opts: &RunOptions,
    extra_args: Option<Value>,
) -> (StepResult, Vec<crate::tools::ToolImage>) {
    let started = Instant::now();
    let mut res = StepResult {
        index,
        tool: step.tool.clone(),
        outcome: Outcome::Pass,
        elapsed_ms: 0,
        failures: Vec::new(),
        image: None,
        diff_ratio: None,
    };
    // `${plm:NAME}` first, then `${out}`: a value the server handed back that
    // happens to contain `${out}` would be expanded too. Harmless (it is the
    // run's own directory), and ids never contain it.
    let step_args = if step.args.is_null() { json!({}) } else { step.args.clone() };
    // The expectations too, so a step can assert an id an earlier one bound.
    let substituted = match &opts.plm {
        Some(plm) => plm.substitute(step_args).and_then(|args| {
            let expect = serde_json::to_value(&step.expect).map_err(|e| e.to_string())?;
            let expect = serde_json::from_value(plm.substitute(expect)?).map_err(|e| e.to_string())?;
            Ok((args, expect))
        }),
        None => Ok((step_args, step.expect.clone())),
    };
    let (step_args, expects): (Value, Vec<crate::script::Expect>) = match substituted {
        Ok(both) => both,
        Err(e) => {
            write_step_json(opts, index, &step.tool, &json!({ "error": e }));
            match &step.expect_error {
                Some(pattern) if crate::script::glob_match(pattern, &e) => {}
                Some(pattern) => {
                    res.outcome = Outcome::ExpectationFailed;
                    res.failures.push(format!("tool failed as expected but with `{e}`, which does not match `{pattern}`"));
                }
                None => {
                    res.outcome = Outcome::ToolError;
                    res.failures.push(e);
                }
            }
            res.elapsed_ms = started.elapsed().as_millis();
            return (res, Vec::new());
        }
    };
    // THE SECOND CLIENT: `plm_request` is answered by the runner from the
    // script's PLM fixture, not by the app, so a script can seed the server
    // and read back what the app wrote. Any HTTP status is a result the step's
    // `expect` reads (`/status`, `/json/...`); only no answer is an error.
    if step.tool == PLM_REQUEST {
        let outcome = match &opts.plm {
            None => Err("`plm_request` needs the script to declare `plm`".to_string()),
            Some(plm) => match serde_json::from_value::<crate::script::PlmCall>(step_args) {
                Err(e) => Err(format!("plm_request arguments: {e}")),
                Ok(call) => {
                    let plm = plm.clone();
                    match tokio::task::spawn_blocking(move || plm.call(&call).map(|a| a.to_value())).await {
                        Ok(answer) => answer,
                        Err(e) => Err(e.to_string()),
                    }
                }
            },
        };
        let output = match outcome {
            Ok(v) => crate::tools::ToolOutput::json(v),
            Err(e) => crate::tools::ToolOutput::json(json!({ "error": e })),
        };
        let failed = output.json.get("error").and_then(Value::as_str).map(str::to_string);
        write_step_json(opts, index, &step.tool, &output.json);
        match (failed, &step.expect_error) {
            (Some(e), Some(pattern)) if crate::script::glob_match(pattern, &e) => {}
            (Some(e), Some(pattern)) => {
                res.outcome = Outcome::ExpectationFailed;
                res.failures.push(format!("tool failed as expected but with `{e}`, which does not match `{pattern}`"));
            }
            (Some(e), None) => {
                res.outcome = Outcome::ToolError;
                res.failures.push(e);
            }
            (None, Some(pattern)) => {
                res.outcome = Outcome::ExpectationFailed;
                res.failures.push(format!("expected the tool to fail with `{pattern}` but it succeeded"));
            }
            (None, None) => {
                for e in &expects {
                    if let Err(why) = e.check(&output.json) {
                        res.failures.push(why);
                    }
                }
                if !res.failures.is_empty() {
                    res.outcome = Outcome::ExpectationFailed;
                }
            }
        }
        res.elapsed_ms = started.elapsed().as_millis();
        return (res, Vec::new());
    }
    let Some(spec) = tools.get(&step.tool) else {
        res.outcome = Outcome::ToolError;
        res.failures.push(format!("no tool `{}` in this session", step.tool));
        res.elapsed_ms = started.elapsed().as_millis();
        return (res, Vec::new());
    };
    // `${out}` in a string argument is the script's output directory, so a
    // script can save files without knowing where it runs.
    let mut args = substitute_out(step_args, &opts.out_dir);
    if let (Some(Value::Object(extra)), Value::Object(into)) = (extra_args, &mut args) {
        into.extend(extra);
    }
    let output = match (spec.handler)(args).await {
        Ok(o) => {
            if let Some(pattern) = &step.expect_error {
                res.outcome = Outcome::ExpectationFailed;
                res.failures.push(format!("expected the tool to fail with `{pattern}` but it succeeded"));
                res.elapsed_ms = started.elapsed().as_millis();
                write_step_json(opts, index, &step.tool, &o.json);
                return (res, Vec::new());
            }
            o
        }
        Err(e) => {
            write_step_json(opts, index, &step.tool, &json!({ "error": e }));
            if let Some(pattern) = &step.expect_error {
                if !crate::script::glob_match(pattern, &e) {
                    res.outcome = Outcome::ExpectationFailed;
                    res.failures.push(format!("tool failed as expected but with `{e}`, which does not match `{pattern}`"));
                }
                res.elapsed_ms = started.elapsed().as_millis();
                return (res, Vec::new());
            }
            res.outcome = Outcome::ToolError;
            res.failures.push(e);
            res.elapsed_ms = started.elapsed().as_millis();
            return (res, Vec::new());
        }
    };
    write_step_json(opts, index, &step.tool, &output.json);
    if !step.bind.is_empty() {
        let bound = match &opts.plm {
            Some(plm) => plm.bind(&output.json, &step.bind),
            None => Err("`bind` needs the script to declare `plm`, which holds the variables".to_string()),
        };
        if let Err(e) = bound {
            res.failures.push(e);
        }
    }
    for e in &expects {
        if let Err(why) = e.check(&output.json) {
            res.failures.push(why);
        }
    }
    if let Some(first) = output.images.first() {
        let name = step
            .save
            .clone()
            .unwrap_or_else(|| format!("{index:04}-{}.png", step.tool));
        let path = opts.out_dir.join(&name);
        let _ = std::fs::write(&path, &first.png);
        res.image = Some(name.clone());
        if let Some(cmp) = &step.compare {
            let baseline = opts.baselines_dir.join(&cmp.baseline);
            match (image::decode_png(&first.png), std::fs::read(&baseline)) {
                (Ok(actual), Ok(base_bytes)) => match image::decode_png(&base_bytes) {
                    Ok(base) => {
                        let d = image::diff(&base, &actual, 8);
                        res.diff_ratio = Some(d.ratio);
                        if d.ratio > cmp.max_diff_ratio {
                            if opts.update_baselines {
                                let _ = std::fs::create_dir_all(baseline.parent().unwrap_or(Path::new(".")));
                                let _ = std::fs::write(&baseline, &first.png);
                            } else {
                                let _ = std::fs::write(opts.out_dir.join(format!("{name}.diff.png")), image::encode_png(&d.mask).unwrap_or_default());
                                res.failures.push(format!(
                                    "image differs from baseline `{}` by {:.4} (max {:.4})",
                                    cmp.baseline, d.ratio, cmp.max_diff_ratio
                                ));
                            }
                        }
                    }
                    Err(e) => res.failures.push(format!("baseline `{}`: {e}", cmp.baseline)),
                },
                (Ok(_), Err(_)) => {
                    if opts.update_baselines {
                        let _ = std::fs::create_dir_all(baseline.parent().unwrap_or(Path::new(".")));
                        let _ = std::fs::write(&baseline, &first.png);
                    } else {
                        res.failures.push(format!(
                            "no baseline `{}` (run with --update-baselines to create it)",
                            cmp.baseline
                        ));
                    }
                }
                (Err(e), _) => res.failures.push(format!("step image: {e}")),
            }
        }
    } else if step.compare.is_some() || step.save.is_some() {
        res.failures.push(format!("step {index} asked to save/compare an image but `{}` produced none", step.tool));
    }
    if !res.failures.is_empty() {
        res.outcome = Outcome::ExpectationFailed;
    }
    res.elapsed_ms = started.elapsed().as_millis();
    (res, output.images)
}

/// The SESSION STORE for a run: `<out_dir>/store`, emptied first.
///
/// The runner hands this to `session_start` as `store_root`, so the app's own
/// file store is what the file modal lists, what Save writes into and what the
/// autosave blob lands in — the real code path, one fresh directory per script.
/// Without it every session gets a private `store-<stamp>` nobody can reach,
/// and no script can open a document through the interface.
/// `fixtures` are copied into `<store>/models` before the app starts, so the
/// Open and Import dialogs list them: the native counterpart of the browser
/// verifier's file-chooser interception, and the only way to drive a real
/// import through the real explorer without pointing it at the checkout — the
/// STEP assembly import writes its part documents to whatever directory the
/// explorer is browsing.
pub fn prepare_store(out_dir: &Path, fixtures: &[String]) -> Result<PathBuf, String> {
    let store = out_dir.join("store");
    if store.exists() {
        // A run must not inherit the last one's saved models, settings or
        // recovery blob: those are exactly what the file and recovery scripts
        // assert the absence of on a cold boot.
        std::fs::remove_dir_all(&store).map_err(|e| format!("clear store {}: {e}", store.display()))?;
    }
    std::fs::create_dir_all(&store).map_err(|e| format!("store {}: {e}", store.display()))?;
    if !fixtures.is_empty() {
        let models = store.join("models");
        std::fs::create_dir_all(&models).map_err(|e| format!("store models {}: {e}", models.display()))?;
        for fixture in fixtures {
            let from = Path::new(fixture);
            let name = from
                .file_name()
                .ok_or_else(|| format!("fixture `{fixture}` has no file name"))?;
            std::fs::copy(from, models.join(name)).map_err(|e| format!("fixture {fixture}: {e}"))?;
        }
    }
    Ok(store)
}

/// One script's START, recorded by an order check: which pass recorded it and
/// what the session looked like before the script's first step ran.
///
/// The order check exists because a test-mcp run plays many scripts through one
/// PROCESS — a fresh app and a fresh store each time, but the same address
/// space, the same warmed allocator and kernel caches, the same statics. "Every
/// script runs through its own headless app session" is only true of the things
/// a session owns; anything else a script leaves behind reaches the next one,
/// and the symptom is a red in a script that passes on its own.
#[derive(Debug, Clone)]
pub struct BootSample {
    pub script: String,
    /// Which pass of the order check this sample came from (`forward`, `reverse`).
    pub pass: String,
    /// The session's state before step 0: the camera and the published viewport
    /// rect + probe points, with the frame counter dropped.
    pub state: Value,
}

/// Every script whose START moved between the passes of an order check — one
/// message per script, naming the passes and the fields that differ. An empty
/// list is the pass condition.
///
/// Two samples of the same script must be IDENTICAL: the script has not run
/// yet, so nothing it does can explain a difference. What it catches is the
/// state a *previous* script left where this one can see it; what it does NOT
/// catch is a leak that only shows once the script is under way, and a leak
/// that reaches the next script only sometimes is caught with the probability
/// it shows in one of the passes — repetition raises that (pass the same list
/// twice and each script gets four samples), position does not.
pub fn order_check_failures(samples: &[BootSample]) -> Vec<String> {
    let mut by_script: std::collections::BTreeMap<&str, Vec<&BootSample>> = std::collections::BTreeMap::new();
    for s in samples {
        by_script.entry(s.script.as_str()).or_default().push(s);
    }
    let mut failures = Vec::new();
    for (script, group) in by_script {
        let Some(first) = group.first() else { continue };
        for other in group.iter().skip(1) {
            if other.state == first.state {
                continue;
            }
            let mut diffs = Vec::new();
            diff_paths(&first.state, &other.state, "", &mut diffs);
            if diffs.len() > 8 {
                let rest = diffs.len() - 8;
                diffs.truncate(8);
                diffs.push(format!("    (+{rest} more)"));
            }
            failures.push(format!(
                "{script}: the session it starts in differs between pass `{}` and pass `{}`\n{}",
                first.pass,
                other.pass,
                diffs.join("\n")
            ));
            break;
        }
    }
    failures
}

/// Every JSON-pointer path at which two values differ, as `    /a/b: x vs y`.
fn diff_paths(a: &Value, b: &Value, at: &str, out: &mut Vec<String>) {
    match (a, b) {
        (Value::Object(x), Value::Object(y)) => {
            let mut keys: Vec<&String> = x.keys().chain(y.keys()).collect();
            keys.sort();
            keys.dedup();
            for k in keys {
                let null = Value::Null;
                diff_paths(x.get(k).unwrap_or(&null), y.get(k).unwrap_or(&null), &format!("{at}/{k}"), out);
            }
        }
        (Value::Array(x), Value::Array(y)) if x.len() == y.len() => {
            for (i, (xi, yi)) in x.iter().zip(y).enumerate() {
                diff_paths(xi, yi, &format!("{at}/{i}"), out);
            }
        }
        _ if a != b => out.push(format!("    {at}: {a} vs {b}")),
        _ => {}
    }
}

fn substitute_out(v: Value, out: &Path) -> Value {
    match v {
        Value::String(s) if s.contains("${out}") => Value::String(s.replace("${out}", &out.display().to_string())),
        Value::Array(a) => Value::Array(a.into_iter().map(|x| substitute_out(x, out)).collect()),
        Value::Object(o) => Value::Object(o.into_iter().map(|(k, x)| (k, substitute_out(x, out))).collect()),
        other => other,
    }
}

fn write_step_json(opts: &RunOptions, index: usize, tool: &str, value: &Value) {
    if let Ok(text) = serde_json::to_string_pretty(value) {
        let _ = std::fs::write(opts.out_dir.join(format!("{index:04}-{tool}.json")), text);
    }
}

