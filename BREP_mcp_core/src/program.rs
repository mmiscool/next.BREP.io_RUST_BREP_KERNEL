//! test-mcp's SCRIPT MODE: a `BREP_mcp/tests/scripts/*.js` program played
//! through code mode, the way an agent drives the app through `run_script`.
//!
//! A program is a module whose `export const meta = {…}` carries the JSON
//! format's session keys (name, note, document, seed, size, ppp, fixtures,
//! plm, timeout_ms) and whose `main` drives the app with
//! `await brep.<group>.<tool>(args)` and checks with the `test` assertion
//! library. The runner puts [`PRELUDE`] in front of it, which makes every tool
//! call one STEP: each step is reported back through `brep.runner.step_record`
//! as it ends, so the per-step artefacts (`NNNN-<tool>.json`, images,
//! `result.json`) are those of the JSON runner and survive a killed worker.
//!
//! The `runner` group exists only while a program runs: the step record, the
//! PLM fixture's `plm_request` / `plm_server`, `${plm:NAME}` / `${out}`
//! substitution, `bind`, and `session_restart` (whose own `session` group is
//! not script-callable).
use crate::runner::{keep_image, stale_frames, substitute_out, Outcome, RunOptions, RunResult, StepResult};
use crate::script::{exit, Script};
use crate::server::ServerState;
use crate::tools::{Annotations, ToolOutput, ToolSpec};
use serde_json::{json, Value};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// The step recorder and assertion library put in front of every program.
pub const PRELUDE: &str = include_str!("test_prelude.js");

/// The group of the tools the runner serves a program.
pub const RUNNER_GROUP: &str = "runner";

/// Catalogue tools: a program that calls only these (and the runner's own)
/// needs no app session, as with the JSON runner.
const SESSIONLESS: &[&str] = &["feature_catalogue", "feature_schema"];

/// A converted script: its session description and its program.
#[derive(Debug)]
pub struct Program {
    /// The `meta` keys as a [`Script`] with no steps.
    pub script: Script,
    pub source: String,
}

impl Program {
    pub fn parse(text: &str) -> Result<Program, String> {
        const MARK: &str = "export const meta = ";
        let at = text.find(MARK).ok_or("a test-mcp program declares `export const meta = {…}` (its name and session)")?;
        let mut values = serde_json::Deserializer::from_str(&text[at + MARK.len()..]).into_iter::<Value>();
        let mut meta = match values.next() {
            Some(Ok(v @ Value::Object(_))) => v,
            Some(Ok(_)) => return Err("`meta` must be a JSON object".into()),
            Some(Err(e)) => return Err(format!("`meta` must be plain JSON: {e}")),
            None => return Err("`meta` is empty".into()),
        };
        meta["steps"] = json!([]);
        let script: Script = serde_json::from_value(meta).map_err(|e| format!("meta: {e}"))?;
        if script.video.is_some() {
            return Err("a walkthrough (`video`) records frames between steps; it stays a JSON script under tests/videos".into());
        }
        if !text.contains("export default") {
            return Err("a test-mcp program has `export default async function main()`".into());
        }
        Ok(Program { script, source: text.to_string() })
    }

    /// Every `brep.<group>.<tool>` the program names.
    pub fn calls(&self) -> Vec<(String, String)> {
        let ident = |s: &str| s.chars().take_while(|c| c.is_ascii_alphanumeric() || *c == '_').collect::<String>();
        let mut out = Vec::new();
        let mut rest = self.source.as_str();
        while let Some(i) = rest.find("brep.") {
            let preceded = rest[..i].chars().next_back().is_some_and(|c| c.is_ascii_alphanumeric() || c == '_' || c == '.');
            rest = &rest[i + 5..];
            if preceded {
                continue;
            }
            let group = ident(rest);
            let Some(after) = rest[group.len()..].strip_prefix('.') else { continue };
            let tool = ident(after);
            if !group.is_empty() && !tool.is_empty() {
                out.push((group, tool));
            }
        }
        out
    }

    /// Whether the program needs an app session: anything beyond the
    /// catalogue tools and the runner's PLM fixture.
    pub fn needs_app(&self) -> bool {
        self.calls().iter().any(|(group, tool)| {
            !(SESSIONLESS.contains(&tool.as_str()) || (group == RUNNER_GROUP && tool.starts_with("plm_")))
        })
    }
}

/// What the runner's tools share for one program.
struct Cx {
    state: Arc<ServerState>,
    out_dir: PathBuf,
    baselines_dir: PathBuf,
    update_baselines: bool,
    plm: Option<Arc<crate::plm_fixture::PlmFixture>>,
    steps: Mutex<Vec<StepResult>>,
    output_chunks: Mutex<String>,
}

fn any_schema() -> Value {
    json!({ "type": "object", "additionalProperties": true })
}

fn runner_tool(
    name: &'static str,
    doc: &'static str,
    cx: &Arc<Cx>,
    f: impl Fn(Arc<Cx>, Value) -> crate::tools::ToolFuture + Send + Sync + 'static,
) -> ToolSpec {
    let cx = cx.clone();
    ToolSpec::new(name, RUNNER_GROUP, doc, any_schema(), Annotations { read_only: false, destructive: false, idempotent: false, waits: false }, move |args| f(cx.clone(), args))
}

/// The `runner` group for one program.
fn runner_tools(cx: &Arc<Cx>) -> Vec<ToolSpec> {
    vec![
        runner_tool("step_output_chunk", "test-mcp: append a large reply chunk.", cx, |cx, args| Box::pin(async move {
            let chunk = args["chunk"].as_str().ok_or("missing output chunk")?;
            let mut output = cx.output_chunks.lock().unwrap();
            if output.len() + chunk.len() > 256 * 1024 * 1024 { return Err("step output exceeds 256 MiB".into()); }
            output.push_str(chunk);
            Ok(ToolOutput::json(json!({})))
        })),
        runner_tool("step_record", "test-mcp: record one step (the prelude calls it).", cx, |cx, args| Box::pin(async move { Ok(ToolOutput::json(step_record(&cx, &args))) })),
        runner_tool("substitute", "test-mcp: `${plm:NAME}` then `${out}` in {value}.", cx, |cx, args| {
            Box::pin(async move {
                let value = args.get("value").cloned().unwrap_or(Value::Null);
                let value = match &cx.plm {
                    Some(plm) => plm.substitute(value)?,
                    None => value,
                };
                Ok(ToolOutput::json(json!({ "value": substitute_out(value, &cx.out_dir) })))
            })
        }),
        runner_tool("bind", "test-mcp: bind {value} at {bind: {NAME: pointer}} for `${plm:NAME}`.", cx, |cx, args| {
            Box::pin(async move {
                let names: std::collections::BTreeMap<String, String> = serde_json::from_value(args.get("bind").cloned().unwrap_or(json!({}))).map_err(|e| e.to_string())?;
                let bound = match &cx.plm {
                    Some(plm) => plm.bind(args.get("value").unwrap_or(&Value::Null), &names),
                    None => Err("`bind` needs the script to declare `plm`, which holds the variables".to_string()),
                };
                Ok(ToolOutput::json(json!({ "failures": bound.err().into_iter().collect::<Vec<_>>() })))
            })
        }),
        runner_tool("plm_request", "test-mcp: one call to the script's PLM server ({method, path, body|raw, as, browser, bind}); answers {status, json, …}.", cx, |cx, args| {
            Box::pin(async move {
                let plm = cx.plm.clone().ok_or("`plm_request` needs the script to declare `plm`")?;
                let args = plm.substitute(args)?;
                let call: crate::script::PlmCall = serde_json::from_value(args).map_err(|e| format!("plm_request arguments: {e}"))?;
                let answer = tokio::task::spawn_blocking(move || plm.call(&call).map(|a| a.to_value())).await.map_err(|e| e.to_string())??;
                Ok(ToolOutput::json(answer))
            })
        }),
        runner_tool("plm_server", "test-mcp: {action: stop | start} the script's PLM server.", cx, |cx, args| {
            Box::pin(async move {
                let plm = cx.plm.clone().ok_or("`plm_server` needs the script to declare `plm`")?;
                let args = plm.substitute(args)?;
                let action = args["action"].as_str().unwrap_or_default().to_string();
                let answer = tokio::task::spawn_blocking(move || plm.server(&action)).await.map_err(|e| e.to_string())??;
                Ok(ToolOutput::json(answer))
            })
        }),
        runner_tool("resource_read", "test-mcp: {uri} read as a client's `resources/read` reads a static resource (a skill://… SKILL.md, brep://schema/…); answers {uri, mimeType, text}.", cx, |cx, args| {
            Box::pin(async move {
                let uri = args["uri"].as_str().ok_or("`resource_read` needs {uri}")?.to_string();
                let spec = cx.state.resources.read().await.get(&uri).cloned().ok_or_else(|| format!("no resource `{uri}`"))?;
                let text = (spec.read)()?;
                Ok(ToolOutput::json(json!({ "uri": uri, "mimeType": spec.mime, "text": text })))
            })
        }),
        runner_tool("session_restart", "test-mcp: the session's own `session_restart` (stop the app, build it again on the same store).", cx, |cx, args| {
            Box::pin(async move {
                let spec = cx.state.tools.read().await.specs.iter().find(|t| t.name == "session_restart" && t.group == "session").cloned();
                let spec = spec.ok_or("no `session_restart` in this session")?;
                (spec.handler)(args).await
            })
        }),
    ]
}

/// One step's record from the prelude: write its JSON and image, compare
/// it, and keep its result. Answers the failures the image added.
fn step_record(cx: &Cx, args: &Value) -> Value {
    let index = args["index"].as_u64().unwrap_or(0) as usize;
    let tool = args["tool"].as_str().unwrap_or("?").to_string();
    let outcome = match args["outcome"].as_str() {
        Some("pass") => Outcome::Pass,
        Some("tool_error") => Outcome::ToolError,
        Some("timeout") => Outcome::Timeout,
        _ => Outcome::ExpectationFailed,
    };
    let failures: Vec<String> = args["failures"].as_array().map(|a| a.iter().map(|f| f.as_str().map_or_else(|| f.to_string(), str::to_string)).collect()).unwrap_or_default();
    let mut res = StepResult {
        group: args["group"].as_str().map(str::to_string),
        expected_error: args["expected_error"].as_bool(),
        index,
        tool: tool.clone(),
        outcome,
        elapsed_ms: args["elapsed_ms"].as_u64().unwrap_or(0) as u128,
        failures,
        image: None,
        diff_ratio: None,
    };
    let before = res.failures.len();
    let chunks = std::mem::take(&mut *cx.output_chunks.lock().unwrap());
    let text = if args["chunked"].as_bool() == Some(true) {
        serde_json::from_str::<Value>(&chunks).map(|_| chunks)
    } else {
        serde_json::to_string_pretty(&args["output"])
    };
    match text {
        Ok(text) => if let Err(e) = std::fs::write(cx.out_dir.join(format!("{index:04}-{tool}.json")), text) { res.failures.push(e.to_string()); },
        Err(e) => res.failures.push(e.to_string()),
    }
    // Steps that ended in an error, or were expected to, make no image (as
    // in the JSON runner, which returned before the image).
    if res.outcome != Outcome::ToolError && !args["skip_image"].as_bool().unwrap_or(false) {
        let png = args["image"].as_str().and_then(|id| {
            let owner = crate::cad::artifacts::Owner { scope: crate::cad::tools::caller_of_transport().scope, execution: None };
            let bytes = cx.state.cad.artifacts.lease(&owner, id).ok().map(|(b, _)| b);
            cx.state.cad.artifacts.release(id);
            bytes
        });
        let compare: Option<crate::script::Compare> = serde_json::from_value(args["compare"].clone()).ok().flatten();
        keep_image(index, &tool, png.as_deref(), args["save"].as_str(), compare.as_ref(), &cx.out_dir, &cx.baselines_dir, cx.update_baselines, &mut res);
    }
    let added: Vec<String> = res.failures[before..].to_vec();
    if !res.failures.is_empty() && res.outcome == Outcome::Pass {
        res.outcome = Outcome::ExpectationFailed;
    }
    cx.steps.lock().unwrap().push(res);
    json!({ "failures": added })
}

/// Play `program` through code mode on `state`'s session.
pub async fn run(program: &Program, state: &Arc<ServerState>, opts: &RunOptions) -> RunResult {
    let script = &program.script;
    let _ = std::fs::create_dir_all(&opts.out_dir);
    let stale_before = stale_frames(&*state.tools.read().await).await;
    let cx = Arc::new(Cx {
        state: state.clone(),
        out_dir: opts.out_dir.clone(),
        baselines_dir: opts.baselines_dir.clone(),
        update_baselines: opts.update_baselines,
        plm: opts.plm.clone(),
        steps: Mutex::new(Vec::new()),
        output_chunks: Mutex::new(String::new()),
    });
    // The runner's tools join the registry for this program only; the slot
    // keeps them through a `session_restart`'s rebuild.
    let extra = runner_tools(&cx);
    *state.extra_tools.lock().unwrap() = extra.clone();
    state.tools.write().await.extend(extra);

    let out_text = opts.out_dir.display().to_string();
    let prelude = PRELUDE.replace("__TEST_OUT__", &out_text.replace('\\', "\\\\").replace('"', "\\\""));
    let source = format!("{prelude}\n{}", program.source);
    let mut extra_steps: Vec<StepResult> = Vec::new();
    let next = |cx: &Cx| cx.steps.lock().unwrap().iter().map(|s| s.index + 1).max().unwrap_or(0);
    let caller = crate::cad::tools::caller_of_transport();
    let (key, _) = crate::cad::keys::issue(&caller.scope, crate::cad::keys::Purpose::Execution);
    let req = json!({
        "manifest": state.cad.catalogue.identity(),
        "language": "javascript",
        "source": source,
        "initialDocuments": [],
        "requestedCapabilities": [],
        "idempotencyKey": key,
    });
    // The host's policy for a gate script: its own timeout, and room for a
    // few hundred steps' replies to be parsed.
    let limits = crate::cad::code::Limits {
        source_bytes: 4 << 20,
        js_cpu_ms: script.timeout_ms.max(60_000),
        js_heap_bytes: 2 << 30,
        wall_ms: script.timeout_ms + 30_000,
        log_bytes: 1 << 20,
        result_bytes: 1 << 20,
        artifact_count: 256,
        artifact_bytes: 256 << 20,
        ..Default::default()
    };
    match state.cad.code.submit_with_limits(&caller, &req, limits) {
        Err(e) => extra_steps.push(error_step(0, "run_script", format!("{}: {}", e.code, e.message), Outcome::ToolError)),
        Ok(status) => {
            let id = status["executionId"].as_str().unwrap_or_default().to_string();
            let deadline = Instant::now() + Duration::from_millis(script.timeout_ms);
            let mut halted = false;
            let result = loop {
                match state.cad.code.result(&caller, &json!({ "executionId": id })) {
                    Ok(r) => break r,
                    Err(e) if e.code == "not_ready" => {
                        if !halted && Instant::now() > deadline {
                            let _ = state.cad.code.cancel(&caller, &json!({ "executionId": id }));
                            halted = true;
                        }
                        tokio::time::sleep(Duration::from_millis(20)).await;
                    }
                    Err(e) => break json!({ "state": "failed", "error": { "code": e.code, "message": e.message } }),
                }
            };
            if halted {
                let index = next(&cx);
                extra_steps.push(error_step(index, "script", format!("script timeout ({} ms) reached at step {index}", script.timeout_ms), Outcome::Timeout));
            } else if result["state"] != "succeeded" {
                // `test.run` reports every step; an execution that still
                // failed never got that far (a syntax error, a host limit).
                let error = &result["error"];
                let message = format!("{}: {}", error["code"].as_str().unwrap_or("error"), error["message"].as_str().unwrap_or_default());
                extra_steps.push(error_step(next(&cx), "script", message, Outcome::ToolError));
            } else if result["output"]["steps"].is_null() {
                extra_steps.push(error_step(next(&cx), "script", "main did not return test.run(…)'s report".into(), Outcome::ToolError));
            }
            state.cad.code.forget(&id);
        }
    }
    state.extra_tools.lock().unwrap().clear();
    state.tools.write().await.specs.retain(|t| t.group != RUNNER_GROUP);

    let mut steps = std::mem::take(&mut *cx.steps.lock().unwrap());
    steps.append(&mut extra_steps);
    let mut failed_step = steps.iter().find(|s| s.outcome != Outcome::Pass).map(|s| s.index);
    // THE STALE-FRAME CHECK, as in the JSON runner.
    match (stale_before, stale_frames(&*state.tools.read().await).await) {
        (Some(before), Some(after)) => {
            eprintln!("stale-frame check: {} {}", script.name, after - before);
            if after > before {
                let index = steps.len();
                steps.push(error_step(
                    index,
                    "stale-frame check",
                    format!("{} frame(s) changed what the viewport draws without a render (BREP_STALE_FRAME lines on stderr name the inputs)", after - before),
                    Outcome::ExpectationFailed,
                ));
                failed_step.get_or_insert(index);
            }
        }
        _ => eprintln!("stale-frame check: {} unavailable", script.name),
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
    let result = RunResult { script: script.name.clone(), passed: exit_code == exit::PASS, exit_code, steps, failed_step, video: None };
    if let Ok(text) = serde_json::to_string_pretty(&result) {
        let _ = std::fs::write(opts.out_dir.join("result.json"), text);
    }
    result
}

fn error_step(index: usize, tool: &str, why: String, outcome: Outcome) -> StepResult {
    StepResult { group: None, expected_error: None, index, tool: tool.into(), outcome, elapsed_ms: 0, failures: vec![why], image: None, diff_ratio: None }
}

