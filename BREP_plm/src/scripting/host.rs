//! The server's host bindings: what a fully trusted script can reach.
//!
//! The operator's call (§3.5): scripts are written by administrators and are
//! trusted with EVERYTHING — the network, the PLM, the server's file system and
//! child processes. There is no allowlist. These bindings are server-only; the
//! interpreter core in [`super::engine`] does not depend on them, so the CAD
//! app can later embed the core with its own host.
//!
//! | Global | Function | Returns |
//! | --- | --- | --- |
//! | `http` | `request(url or { url, method, headers, body, timeoutMs })` | `{ status, headers, body, json }` |
//! | `fs` | `readFile(path)`, `writeFile(path, text)`, `exists(path)`, `listDir(path)` | text / nothing / bool / names |
//! | `process` | `run(command, args?, { cwd, stdin }?)` | `{ status, stdout, stderr }` |
//! | `plm` | `user`, `part(idOrNumber)`, `findParts(query)`, `partTypes()`, `readDocument(part, revision)`, `updatePart(part, fields)`, `manufacturers()`, `suppliers()`, `addManufacturerPart(part, fields)`, `addOffer(part, mpId, fields)`, `bom(part, revision, { levels, flat })`, `whereUsed(part, revision?, levels?)`, `attachments(part, revision?)` | JSON |
//!
//! Every call is synchronous — a Boa host function cannot await. An HTTP call
//! carries its own timeout (default [`HTTP_TIMEOUT_MS`]) because nothing else
//! bounds it; a child process is bounded by nothing but itself. A relative
//! file path, and a child's default working directory, is the scripts
//! directory.
//!
//! `plm.*` goes through the ordinary [`Db`] methods, each taking the store lock
//! itself for its own moment. That is safe only because hooks never run while
//! the lock is held (see [`crate::db`]); a binding here must never be called
//! from inside [`Db::mutate`].

use std::io::Write;
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::time::Duration;

use boa_engine::object::ObjectInitializer;
use boa_engine::property::Attribute;
use boa_engine::{js_string, Context, JsError, JsNativeError, JsResult, JsValue, NativeFunction};
use serde_json::{json, Value};

use crate::db::Db;
use crate::model::User;

/// How long an `http.request` waits when the script does not say.
pub const HTTP_TIMEOUT_MS: u64 = 30_000;

/// The bindings for one hook call: the store, and who the call acts for.
#[derive(Clone)]
pub struct Host {
    db: Db,
    user: User,
}

type Binding = fn(&Host, &[JsValue], &mut Context) -> JsResult<JsValue>;

impl Host {
    pub fn new(db: Db, user: User) -> Self {
        Host { db, user }
    }

    fn function(&self, body: Binding) -> NativeFunction {
        let host = self.clone();
        // SAFETY: the closure captures a `Host` — an `Arc`-backed store handle
        // and a plain user record — and a function pointer. None of it holds a
        // garbage-collected value, so the collector has nothing in it to trace.
        unsafe { NativeFunction::from_closure(move |_, args, context| body(&host, args, context)) }
    }

    /// Define `http`, `fs`, `process` and `plm` on the global object.
    pub fn install(&self, context: &mut Context) -> JsResult<()> {
        let http = ObjectInitializer::new(context)
            .function(self.function(http_request), js_string!("request"), 1)
            .build();
        context.register_global_property(js_string!("http"), http, Attribute::all())?;

        let files = ObjectInitializer::new(context)
            .function(self.function(fs_read), js_string!("readFile"), 1)
            .function(self.function(fs_write), js_string!("writeFile"), 2)
            .function(self.function(fs_exists), js_string!("exists"), 1)
            .function(self.function(fs_list), js_string!("listDir"), 1)
            .build();
        context.register_global_property(js_string!("fs"), files, Attribute::all())?;

        let process = ObjectInitializer::new(context)
            .function(self.function(process_run), js_string!("run"), 1)
            .build();
        context.register_global_property(js_string!("process"), process, Attribute::all())?;

        let user = JsValue::from_json(&super::user_json(&self.user), context)?;
        let plm = ObjectInitializer::new(context)
            .property(js_string!("user"), user, Attribute::all())
            .function(self.function(plm_part), js_string!("part"), 1)
            .function(self.function(plm_find), js_string!("findParts"), 1)
            .function(self.function(plm_types), js_string!("partTypes"), 0)
            .function(self.function(plm_read_document), js_string!("readDocument"), 2)
            .function(self.function(plm_update_part), js_string!("updatePart"), 2)
            .function(self.function(plm_manufacturers), js_string!("manufacturers"), 0)
            .function(self.function(plm_suppliers), js_string!("suppliers"), 0)
            .function(self.function(plm_add_manufacturer_part), js_string!("addManufacturerPart"), 2)
            .function(self.function(plm_add_offer), js_string!("addOffer"), 3)
            .function(self.function(plm_bom), js_string!("bom"), 3)
            .function(self.function(plm_where_used), js_string!("whereUsed"), 3)
            .function(self.function(plm_attachments), js_string!("attachments"), 2)
            .build();
        context.register_global_property(js_string!("plm"), plm, Attribute::all())?;
        Ok(())
    }

    fn path(&self, text: &str) -> PathBuf {
        let path = PathBuf::from(text);
        if path.is_absolute() {
            path
        } else {
            self.db.scripts().dir().join(path)
        }
    }
}

// ---------------------------------------------------------------- helpers

fn throw(message: impl Into<String>) -> JsError {
    JsNativeError::error().with_message(message.into()).into()
}

fn arg_json(args: &[JsValue], index: usize, context: &mut Context) -> JsResult<Value> {
    match args.get(index) {
        None => Ok(Value::Null),
        Some(value) => Ok(value.to_json(context)?.unwrap_or(Value::Null)),
    }
}

fn arg_string(args: &[JsValue], index: usize, what: &str, context: &mut Context) -> JsResult<String> {
    match args.get(index) {
        Some(value) if !value.is_undefined() && !value.is_null() => {
            Ok(value.to_string(context)?.to_std_string_escaped())
        }
        _ => Err(throw(format!("{what} is required"))),
    }
}

fn to_js(value: &Value, context: &mut Context) -> JsResult<JsValue> {
    JsValue::from_json(value, context)
}

fn plm_error(error: crate::Error) -> JsError {
    throw(error.message)
}

// ------------------------------------------------------------------ http

fn http_request(_: &Host, args: &[JsValue], context: &mut Context) -> JsResult<JsValue> {
    let options = match arg_json(args, 0, context)? {
        Value::String(url) => json!({ "url": url }),
        Value::Object(map) => Value::Object(map),
        _ => return Err(throw("http.request needs a URL or { url, method, headers, body, timeoutMs }")),
    };
    let url = options["url"].as_str().ok_or_else(|| throw("http.request: url is required"))?.to_string();
    let method = options["method"].as_str().unwrap_or("GET").to_ascii_uppercase();
    let timeout = options["timeoutMs"].as_u64().unwrap_or(HTTP_TIMEOUT_MS);

    let agent = ureq::AgentBuilder::new()
        .timeout(Duration::from_millis(timeout))
        .build();
    let mut request = agent.request(&method, &url);
    let mut has_content_type = false;
    if let Some(headers) = options["headers"].as_object() {
        for (name, value) in headers {
            let value = value.as_str().map(str::to_string).unwrap_or_else(|| value.to_string());
            has_content_type |= name.eq_ignore_ascii_case("content-type");
            request = request.set(name, &value);
        }
    }
    let result = match &options["body"] {
        Value::Null => request.call(),
        Value::String(text) => request.send_string(text),
        other => {
            if !has_content_type {
                request = request.set("Content-Type", "application/json");
            }
            request.send_string(&other.to_string())
        }
    };
    let response = match result {
        Ok(response) => response,
        // A non-2xx answer is still an answer: the script decides what 404
        // means. Only a transport failure throws.
        Err(ureq::Error::Status(_, response)) => response,
        Err(error) => return Err(throw(format!("{method} {url}: {error}"))),
    };
    let status = response.status();
    let mut headers = serde_json::Map::new();
    for name in response.headers_names() {
        if let Some(value) = response.header(&name) {
            headers.insert(name.to_ascii_lowercase(), Value::String(value.to_string()));
        }
    }
    let body = response
        .into_string()
        .map_err(|e| throw(format!("{method} {url}: reading the body failed: {e}")))?;
    let parsed = serde_json::from_str::<Value>(&body).unwrap_or(Value::Null);
    to_js(&json!({ "status": status, "headers": headers, "body": body, "json": parsed }), context)
}

// -------------------------------------------------------------------- fs

fn fs_read(host: &Host, args: &[JsValue], context: &mut Context) -> JsResult<JsValue> {
    let path = host.path(&arg_string(args, 0, "fs.readFile: path", context)?);
    let text = std::fs::read_to_string(&path).map_err(|e| throw(format!("fs.readFile {}: {e}", path.display())))?;
    Ok(JsValue::from(boa_engine::JsString::from(text)))
}

fn fs_write(host: &Host, args: &[JsValue], context: &mut Context) -> JsResult<JsValue> {
    let path = host.path(&arg_string(args, 0, "fs.writeFile: path", context)?);
    let text = arg_string(args, 1, "fs.writeFile: text", context)?;
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).map_err(|e| throw(format!("fs.writeFile {}: {e}", path.display())))?;
    }
    std::fs::write(&path, text).map_err(|e| throw(format!("fs.writeFile {}: {e}", path.display())))?;
    Ok(JsValue::undefined())
}

fn fs_exists(host: &Host, args: &[JsValue], context: &mut Context) -> JsResult<JsValue> {
    let path = host.path(&arg_string(args, 0, "fs.exists: path", context)?);
    Ok(JsValue::from(path.exists()))
}

fn fs_list(host: &Host, args: &[JsValue], context: &mut Context) -> JsResult<JsValue> {
    let path = host.path(&arg_string(args, 0, "fs.listDir: path", context)?);
    let mut names: Vec<String> = std::fs::read_dir(&path)
        .map_err(|e| throw(format!("fs.listDir {}: {e}", path.display())))?
        .flatten()
        .map(|entry| entry.file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    to_js(&json!(names), context)
}

// --------------------------------------------------------------- process

fn process_run(host: &Host, args: &[JsValue], context: &mut Context) -> JsResult<JsValue> {
    let program = arg_string(args, 0, "process.run: command", context)?;
    let arguments: Vec<String> = match arg_json(args, 1, context)? {
        Value::Array(items) => items
            .into_iter()
            .map(|v| v.as_str().map(str::to_string).unwrap_or_else(|| v.to_string()))
            .collect(),
        Value::Null => Vec::new(),
        _ => return Err(throw("process.run: args must be an array of strings")),
    };
    let options = arg_json(args, 2, context)?;
    let cwd = options["cwd"]
        .as_str()
        .map(|c| host.path(c))
        .unwrap_or_else(|| host.db.scripts().dir().to_path_buf());
    let stdin = options["stdin"].as_str().map(str::to_string);

    let mut command = Command::new(&program);
    command
        .args(&arguments)
        .current_dir(&cwd)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .stdin(if stdin.is_some() { Stdio::piped() } else { Stdio::null() });
    let mut child = command.spawn().map_err(|e| throw(format!("process.run {program}: {e}")))?;
    if let (Some(text), Some(mut pipe)) = (stdin, child.stdin.take()) {
        // Written on its own thread so a child that fills its stdout before
        // reading all of stdin cannot deadlock against us.
        std::thread::spawn(move || {
            let _ = pipe.write_all(text.as_bytes());
        });
    }
    let output = child
        .wait_with_output()
        .map_err(|e| throw(format!("process.run {program}: {e}")))?;
    to_js(
        &json!({
            "status": output.status.code(),
            "stdout": String::from_utf8_lossy(&output.stdout),
            "stderr": String::from_utf8_lossy(&output.stderr),
        }),
        context,
    )
}

// ------------------------------------------------------------------- plm

fn plm_part(host: &Host, args: &[JsValue], context: &mut Context) -> JsResult<JsValue> {
    let key = arg_string(args, 0, "plm.part: id or number", context)?;
    let part = host.db.read(|state| state.part_by_id_or_number(&key).cloned());
    match part {
        Some(part) => to_js(&serde_json::to_value(part).map_err(|e| throw(e.to_string()))?, context),
        None => Ok(JsValue::null()),
    }
}

fn plm_find(host: &Host, args: &[JsValue], context: &mut Context) -> JsResult<JsValue> {
    let query = match args.first() {
        Some(value) if !value.is_undefined() && !value.is_null() => value.to_string(context)?.to_std_string_escaped(),
        _ => String::new(),
    };
    let rows = host.db.search_parts(&query);
    let rows: Vec<Value> = rows
        .iter()
        .map(|part| {
            let latest = part.latest();
            json!({
                "id": part.id,
                "number": part.number,
                "name": part.name,
                "part_type": part.part_type,
                "category": part.category,
                "tags": part.tags,
                "attributes": part.attributes,
                "mpns": part.sourcing.iter().map(|mp| mp.mpn.clone()).collect::<Vec<_>>(),
                "external_ref": part.external_ref,
                "document_class": part.document_class,
                "latest_label": latest.map(|r| r.label.clone()),
                "latest_state": latest.map(|r| r.lifecycle.as_str()),
            })
        })
        .collect();
    to_js(&Value::Array(rows), context)
}

fn plm_types(host: &Host, _: &[JsValue], context: &mut Context) -> JsResult<JsValue> {
    let types = host.db.read(|state| serde_json::to_value(&state.part_types));
    to_js(&types.map_err(|e| throw(e.to_string()))?, context)
}

fn plm_read_document(host: &Host, args: &[JsValue], context: &mut Context) -> JsResult<JsValue> {
    let key = arg_string(args, 0, "plm.readDocument: part", context)?;
    let revision = arg_string(args, 1, "plm.readDocument: revision label or id", context)?;
    let document_key = host
        .db
        .read(|state| {
            let part = state.part_by_id_or_number(&key)?;
            let rev = part
                .revision(&revision)
                .or_else(|| part.revision_by_label(&revision))?;
            Some(rev.document_key(&part.id))
        })
        .ok_or_else(|| throw(format!("plm.readDocument: no revision '{revision}' of '{key}'")))?;
    match host.db.read_document(&document_key).map_err(plm_error)? {
        Some(text) => Ok(JsValue::from(boa_engine::JsString::from(text))),
        None => Ok(JsValue::null()),
    }
}

fn plm_update_part(host: &Host, args: &[JsValue], context: &mut Context) -> JsResult<JsValue> {
    let key = arg_string(args, 0, "plm.updatePart: part", context)?;
    let fields = arg_json(args, 1, context)?;
    let part = host.db.update_part(&key, &fields).map_err(plm_error)?;
    to_js(&serde_json::to_value(part).map_err(|e| throw(e.to_string()))?, context)
}

fn plm_companies(host: &Host, which: crate::sourcing::Companies, context: &mut Context) -> JsResult<JsValue> {
    let list = host.db.read(|state| serde_json::to_value(which.list(state)));
    to_js(&list.map_err(|e| throw(e.to_string()))?, context)
}

fn plm_manufacturers(host: &Host, _: &[JsValue], context: &mut Context) -> JsResult<JsValue> {
    plm_companies(host, crate::sourcing::Companies::Manufacturers, context)
}

fn plm_suppliers(host: &Host, _: &[JsValue], context: &mut Context) -> JsResult<JsValue> {
    plm_companies(host, crate::sourcing::Companies::Suppliers, context)
}

fn plm_add_manufacturer_part(host: &Host, args: &[JsValue], context: &mut Context) -> JsResult<JsValue> {
    let key = arg_string(args, 0, "plm.addManufacturerPart: part", context)?;
    let fields = arg_json(args, 1, context)?;
    let mp = host.db.add_manufacturer_part(&key, &fields).map_err(plm_error)?;
    to_js(&serde_json::to_value(mp).map_err(|e| throw(e.to_string()))?, context)
}

fn plm_add_offer(host: &Host, args: &[JsValue], context: &mut Context) -> JsResult<JsValue> {
    let key = arg_string(args, 0, "plm.addOffer: part", context)?;
    let mp = arg_string(args, 1, "plm.addOffer: manufacturer part id", context)?;
    let fields = arg_json(args, 2, context)?;
    let offer = host.db.add_offer(&key, &mp, &fields).map_err(plm_error)?;
    to_js(&serde_json::to_value(offer).map_err(|e| throw(e.to_string()))?, context)
}

fn plm_bom(host: &Host, args: &[JsValue], context: &mut Context) -> JsResult<JsValue> {
    let key = arg_string(args, 0, "plm.bom: part", context)?;
    let revision = arg_string(args, 1, "plm.bom: revision label or id", context)?;
    let options = arg_json(args, 2, context)?;
    let levels = options.get("levels").and_then(Value::as_u64).unwrap_or(0) as usize;
    let flat = options.get("flat").and_then(Value::as_bool).unwrap_or(false);
    let bom = host.db.read(|state| {
        let part = state.part_by_id_or_number(&key)?;
        let rev = crate::bom::find_revision(part, &revision)?;
        Some(crate::bom::bom(state, part, rev, levels, flat))
    });
    let bom = bom.ok_or_else(|| throw(format!("plm.bom: no revision '{revision}' of '{key}'")))?;
    to_js(&serde_json::to_value(bom).map_err(|e| throw(e.to_string()))?, context)
}

fn plm_where_used(host: &Host, args: &[JsValue], context: &mut Context) -> JsResult<JsValue> {
    let key = arg_string(args, 0, "plm.whereUsed: part", context)?;
    let revision = match args.get(1) {
        Some(value) if !value.is_undefined() && !value.is_null() => value.to_string(context)?.to_std_string_escaped(),
        _ => String::new(),
    };
    let levels = arg_json(args, 2, context)?.as_u64().unwrap_or(0) as usize;
    let answer = host.db.read(|state| {
        let part = state.part_by_id_or_number(&key).ok_or_else(|| format!("plm.whereUsed: no part '{key}'"))?;
        let rev = match revision.trim() {
            "" => None,
            label => Some(
                crate::bom::find_revision(part, label)
                    .ok_or_else(|| format!("plm.whereUsed: no revision '{label}' of '{key}'"))?,
            ),
        };
        Ok::<_, String>(crate::bom::where_used(state, part, rev, levels))
    });
    let answer = answer.map_err(throw)?;
    to_js(&serde_json::to_value(answer).map_err(|e| throw(e.to_string()))?, context)
}

/// A part's attachments, or one revision's: each record with `path`, the
/// blob's absolute path, so a (fully trusted) script can read the bytes with
/// `fs.readFile` or hand them to `process.run`.
fn plm_attachments(host: &Host, args: &[JsValue], context: &mut Context) -> JsResult<JsValue> {
    let key = arg_string(args, 0, "plm.attachments: part", context)?;
    let revision = match args.get(1) {
        Some(value) if !value.is_undefined() && !value.is_null() => value.to_string(context)?.to_std_string_escaped(),
        _ => String::new(),
    };
    let list = host.db.read(|state| {
        let part = state.part_by_id_or_number(&key).ok_or_else(|| format!("plm.attachments: no part '{key}'"))?;
        let files = match revision.trim() {
            "" => part.attachments.clone(),
            label => crate::bom::find_revision(part, label)
                .ok_or_else(|| format!("plm.attachments: no revision '{label}' of '{key}'"))?
                .attachments
                .clone(),
        };
        Ok::<_, String>(files)
    });
    let list = list.map_err(throw)?;
    let with_paths: Vec<serde_json::Value> = list
        .iter()
        .map(|a| {
            let mut v = serde_json::to_value(a).unwrap_or_default();
            v["path"] = serde_json::json!(host.db.blob_path(&a.sha256).display().to_string());
            v
        })
        .collect();
    to_js(&serde_json::Value::Array(with_paths), context)
}
