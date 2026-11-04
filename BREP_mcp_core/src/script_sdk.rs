//! The script functions every host's code runner shares: each
//! script-callable registry tool as `brep.<group>.<tool>(args)`, reached
//! through the realm's bridge (`tool.<name>`). Pure and target-free, so the
//! native server (`cad::sdk`, beside the `cad.*` client) and the browser's
//! runner (`BREP_app/src/webmcp.rs`, a Web Worker) generate the same module.
use crate::tools::ToolSpec;

/// Validate the small public execution envelope before starting a session or
/// invoking a handler. Shared by native MCP and the browser adapter.
pub fn validate_front_door(name: &str, args: &serde_json::Value) -> Result<(), String> {
    if name == "files" {
        if args.to_string().len() > 1 << 20 { return Err("arguments exceed 1 MiB".into()); }
        return serde_json::from_value::<crate::files::Request>(args.clone()).map(|_| ()).map_err(|e| e.to_string());
    }
    let obj = args.as_object().ok_or("arguments must be an object")?;
    if args.to_string().len() > 1 << 20 { return Err("arguments exceed 1 MiB".into()); }
    let fields: &[&str] = match name {
        "run_script" => &["source", "language", "wait_ms", "key"],
        "script_result" => &["executionId", "wait_ms"],
        "script_halt" => &["executionId"],
        "input" => &["steps"],
        "screenshot" => &["region", "max_width", "cursor", "cursor_scale", "save_as", "settings", "clear_selection", "inline"],
        _ => return Err(format!("unknown front-door tool `{name}`")),
    };
    if let Some(k) = obj.keys().find(|k| !fields.contains(&k.as_str())) { return Err(format!("unknown argument `{k}`")); }
    for key in match name { "run_script" => &["source"][..], "script_result" | "script_halt" => &["executionId"][..], _ => &[] } {
        if !obj.get(*key).is_some_and(|v| v.as_str().is_some_and(|s| !s.is_empty())) { return Err(format!("{key} must be a nonempty string")); }
    }
    if let Some(v) = obj.get("source") { if v.as_str().unwrap().len() > 256 << 10 { return Err("source exceeds 256 KiB".into()); } }
    if let Some(v) = obj.get("language") { if !matches!(v.as_str(), Some("javascript" | "typescript")) { return Err("language must be javascript or typescript".into()); } }
    if let Some(v) = obj.get("wait_ms") { if !v.as_u64().is_some_and(|n| n <= 600_000) { return Err("wait_ms must be an integer from 0 to 600000".into()); } }
    if let Some(v) = obj.get("key") { if !v.as_str().is_some_and(|s| !s.is_empty() && s.len() <= 128) { return Err("key must be 1 to 128 bytes".into()); } }
    if name == "screenshot" {
        for key in ["cursor", "clear_selection", "inline"] {
            if obj.get(key).is_some_and(|v| !v.is_boolean()) { return Err(format!("{key} must be a boolean")); }
        }
        for (key, max) in [("max_width", 16384), ("cursor_scale", 32)] {
            if obj.get(key).is_some_and(|v| !v.as_u64().is_some_and(|n| n > 0 && n <= max)) { return Err(format!("{key} must be an integer from 1 to {max}")); }
        }
        if obj.get("settings").is_some_and(|v| !v.is_object()) { return Err("settings must be an object".into()); }
        if obj.get("save_as").is_some_and(|v| !v.as_str().is_some_and(|s| !s.is_empty())) { return Err("save_as must be a nonempty string".into()); }
        if let Some(region) = obj.get("region") {
            let valid = matches!(region.as_str(), Some("full" | "viewport")) || region.as_object().is_some_and(|r| {
                r.len() == 4 && ["x", "y", "w", "h"].iter().all(|k| r.get(*k).is_some_and(|v| v.is_number()))
                    && ["w", "h"].iter().all(|k| r[*k].as_f64().is_some_and(|n| n > 0.0 && n <= 16384.0))
            });
            if !valid { return Err("region must be full, viewport, or numeric {x,y,w,h} with positive dimensions at most 16384".into()); }
        }
    }
    if name == "input" {
        let steps = obj.get("steps").and_then(|v| v.as_array()).ok_or("steps must be an array")?;
        if steps.len() > 256 { return Err("steps may contain at most 256 entries".into()); }
        for (i, step) in steps.iter().enumerate() {
            let s = step.as_object().ok_or_else(|| format!("steps[{i}] must be an object"))?;
            if s.keys().any(|k| k != "tool" && k != "args") || !s.get("tool").is_some_and(|v| v.as_str().is_some_and(|s| !s.is_empty())) || s.get("args").is_some_and(|v| !v.is_object()) {
                return Err(format!("steps[{i}] requires tool: string and optional args: object"));
            }
        }
    }
    Ok(())
}

/// One registry tool as a script sees it.
#[derive(Debug, Clone)]
pub struct ScriptTool {
    pub name: String,
    pub group: String,
    pub doc: String,
    pub input_schema: serde_json::Value,
    pub output_schema: serde_json::Value,
}

impl ScriptTool {
    pub fn of(spec: &ToolSpec) -> ScriptTool {
        ScriptTool { name: spec.name.clone(), group: spec.group.to_string(), doc: spec.doc.clone(), input_schema: spec.input_schema.clone(), output_schema: spec.output_schema.clone() }
    }
}

/// Whether a registry tool is a script function. Not: the `cad_*` group (a
/// script has `@brep/cad` for the service), the session's own lifecycle (a
/// script runs INSIDE a session), and the MCP's front door (a script does
/// not start or halt scripts).
pub fn script_callable(group: &str, name: &str) -> bool {
    matches!(name, "session_info" | "session_record" | "session_script") || !matches!(group, "cad" | "session" | "script")
}

/// The script-callable tools of a set, in its order.
pub fn script_tools<'a>(specs: impl IntoIterator<Item = &'a ToolSpec>) -> Vec<ScriptTool> {
    specs.into_iter().filter(|t| script_callable(t.group, &t.name)).map(ScriptTool::of).collect()
}

/// A JavaScript identifier for a registry name (`doc-new` → `doc_new`).
pub fn ident(name: &str) -> String {
    let mut out: String = name.chars().map(|c| if c.is_ascii_alphanumeric() || c == '_' { c } else { '_' }).collect();
    if out.chars().next().is_some_and(|c| c.is_ascii_digit()) {
        out.insert(0, '_');
    }
    out
}

/// Public method name for a namespaced registry entry.
pub fn method_name(group: &str, name: &str) -> String {
    ident(if group == "files" { name.strip_prefix("files_").unwrap_or(name) } else { name })
}

/// The first sentence of a tool's doc, for its JSDoc line.
pub fn summary(doc: &str) -> String {
    let first = doc.split(". ").next().unwrap_or(doc).trim();
    first.replace("*/", "* /").chars().take(200).collect()
}

/// `export const brep`: every tool as `brep.<group>.<tool>(args)`, by group —
/// the tool's own handler, reached through the bridge. Needs the module's
/// `call` and `unwrap` in scope.
pub fn tool_bindings(tools: &[ScriptTool]) -> String {
    let mut groups: std::collections::BTreeMap<String, Vec<String>> = Default::default();
    for t in tools {
        groups.entry(ident(&t.group)).or_default().push(format!("    /** {} */\n    {}: (args) => tool({:?}, args),", summary(&t.doc), method_name(&t.group, &t.name), t.name));
    }
    let body = groups
        .iter()
        .map(|(g, ms)| format!("  {g}: {{\n{}\n  }},", ms.join("\n")))
        .collect::<Vec<_>>()
        .join("\n");
    format!(
        "// The app's tool registry as functions: each call runs that tool's own\n// handler on the host, exactly as an MCP tool call would.\nasync function tool(name, args) {{\n  return unwrap(await call(\"tool.\" + name, args === undefined ? {{}} : args));\n}}\n\nexport const brep = {{\n{body}\n}};\n"
    )
}

/// The browser's `@brep/cad`: the bridge, `ApiFailure`, and [`tool_bindings`].
/// The `cad` operation client and `service` are the native server's (its
/// operation service does not run in a page); naming either says so.
pub fn browser_module(tools: &[ScriptTool]) -> String {
    format!("{BROWSER_CORE}\n{}", tool_bindings(tools))
}

const BROWSER_CORE: &str = r#"// @brep/cad in the browser: the bridge to the page and the app's tools as
// functions. The `cad.*` operation client is native-only.
const host = globalThis.__brep_host;
delete globalThis.__brep_host;
delete globalThis.__brep_manifest;
export class ApiFailure extends Error {
  constructor(error) {
    super(error && error.message ? error.message : "API failure");
    this.name = "ApiFailure";
    this.error = error;
  }
}
async function call(method, request) {
  return JSON.parse(await host(method, JSON.stringify(request === undefined ? {} : request)));
}
function unwrap(response) {
  if (response && response.ok === true) return response.value;
  throw new ApiFailure(response ? response.error : null);
}
const nativeOnly = (name) => new Proxy({}, { get() { throw new Error(name + " (the cad.* operation service) runs in the native app and brep-mcp; in the browser use brep.<group>.<tool>()"); } });
export const cad = nativeOnly("cad");
export const service = nativeOnly("service");
"#;

