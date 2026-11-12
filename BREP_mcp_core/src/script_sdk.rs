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

/// A script call that passes `context` to a tool whose schema does not
/// declare one: such a tool would drop it, and the caller would believe a
/// revision check ran. Every native app command declares it (the frame
/// thread checks it); a tool composed by the server declares it only when it
/// forwards it. Shared by the native bridge and the browser's.
pub fn undeclared_context(name: &str, input_schema: &serde_json::Value, args: &serde_json::Value) -> Option<String> {
    if args.get("context").is_none() || input_schema.pointer("/properties/context").is_some() {
        return None;
    }
    Some(format!("`{name}` does not check a document context, so it cannot honour `context`: call it without one. It acts on the visible document; brep.document.docs_list() names that document and its revision"))
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
        "// The app's tool registry as functions: each call runs that tool's own\n// handler on the host, exactly as an MCP tool call would.\nasync function tool(name, args) {{\n  return unwrap(await call(\"tool.\" + name, args === undefined ? {{}} : args));\n}}\n{GUARD}\nexport const brep = guardTree({{\n{body}\n}}, \"brep\");\n"
    )
}

/// `guardTree(object, path)`: the namespace a script reaches through, so a
/// name that is not there says what is. A member spelled another way
/// (`featureAdd`, `Feature-Add`) that matches exactly one member is that
/// member; any other unknown member throws a TypeError naming the closest
/// members and, for a function in another group, its full path. Without it
/// Boa answers `brep.history.feature_add(...)` with "not a callable
/// function" and the agent guesses again.
const GUARD: &str = r#"
const passThrough = new Set(["then", "toJSON", "constructor", "valueOf", "toString", "inspect", "prototype", "__proto__", "length", "name"]);
const squash = (k) => k.toLowerCase().replace(/[^a-z0-9]/g, "");
function distance(a, b) {
  const row = Array.from({ length: b.length + 1 }, (_, j) => j);
  for (let i = 1; i <= a.length; i++) {
    let prev = row[0];
    row[0] = i;
    for (let j = 1; j <= b.length; j++) {
      const next = Math.min(row[j] + 1, row[j - 1] + 1, prev + (a[i - 1] === b[j - 1] ? 0 : 1));
      prev = row[j];
      row[j] = next;
    }
  }
  return row[b.length];
}
function guardTree(target, path, root) {
  const top = root || target;
  for (const [k, v] of Object.entries(target)) {
    if (v && typeof v === "object" && Object.getPrototypeOf(v) === Object.prototype) target[k] = guardTree(v, path + "." + k, top);
  }
  return new Proxy(target, {
    get(t, key, receiver) {
      if (typeof key !== "string" || key in t || passThrough.has(key)) return Reflect.get(t, key, receiver);
      const same = Object.keys(t).filter((k) => squash(k) === squash(key));
      if (same.length === 1) return t[same[0]];
      const near = Object.keys(t).map((k) => [distance(squash(k), squash(key)), k]).filter(([d]) => d <= Math.max(2, Math.floor(key.length / 3))).sort((a, b) => a[0] - b[0]).slice(0, 3).map(([, k]) => path + "." + k);
      const elsewhere = [];
      if (t !== top) {
        for (const [g, members] of Object.entries(top)) {
          if (members && typeof members === "object") for (const m of Object.keys(members)) if (squash(m) === squash(key)) elsewhere.push(path.split(".")[0] + "." + g + "." + m);
        }
      }
      const hints = elsewhere.concat(near);
      const group = path.split(".").slice(1).join(".");
      const listing = path.startsWith("brep") ? (group ? "cad.docs.sdk(\"" + group + "\") lists " + path + ".*" : "cad.docs.sdk() lists the groups") : "cad.docs.sdk(\"cad\") declares the cad client";
      throw new TypeError(path + "." + key + " does not exist" + (hints.length ? "; did you mean " + hints.join(" or ") + "?" : "") + " (" + listing + ")");
    },
  });
}
"#;

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

