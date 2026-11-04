//! Bounded file operations shared by MCP, CodeMode and the browser store.
//! Hosts provide storage primitives; this module owns request semantics and limits.
use base64::{engine::general_purpose::STANDARD, Engine};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

#[cfg(feature = "server")]
pub mod native;

/// Byte encoding shared with host adapters; avoids a second codec dependency.
pub fn base64_encode(bytes: &[u8]) -> String { STANDARD.encode(bytes) }
pub fn base64_decode(text: &str) -> Result<Vec<u8>, String> { STANDARD.decode(text).map_err(|_| "invalid base64 data".into()) }

pub const MAX_TRANSFER: usize = 256 * 1024;
pub const MAX_FILE: u64 = 32 * 1024 * 1024;
pub const MAX_TREE_BYTES: u64 = 128 * 1024 * 1024;
pub const MAX_ENTRIES: usize = 1024;
pub const MAX_PAGE: usize = 256;
pub const MAX_DEPTH: usize = 64;

#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
// Unknown operation fields are rejected by the flattened tagged enum.
// Serde cannot combine flatten with deny_unknown_fields on this outer struct.
pub struct Request {
    pub root: String,
    #[serde(default)]
    pub path: String,
    #[serde(flatten)]
    pub operation: Operation,
}

#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(tag = "op", rename_all = "snake_case", deny_unknown_fields)]
pub enum Operation {
    List { #[serde(default)] offset: usize, #[serde(default = "page_size")] limit: usize },
    Stat,
    Read { #[serde(default)] offset: u64, #[serde(default)] length: Option<usize>, #[serde(default)] encoding: Encoding },
    Write { data: String, #[serde(default)] encoding: Encoding, #[serde(default)] overwrite: bool },
    Mkdir,
    Copy { destination: String, #[serde(default)] overwrite: bool, #[serde(default)] recursive: bool },
    Move { destination: String, #[serde(default)] overwrite: bool },
    Delete { #[serde(default)] recursive: bool },
}
fn page_size() -> usize { 100 }
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Encoding { #[default] Text, Base64 }
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Kind { File, Directory }
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct Entry {
    pub name: String,
    pub kind: Kind,
    pub size: Option<u64>,
    /// Unix seconds, or null when the store does not retain timestamps.
    pub mtime: Option<f64>,
}
#[derive(Clone, Debug, Serialize, JsonSchema)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum Reply {
    List { entries: Vec<Entry>, next_offset: Option<usize> },
    Stat { entry: Entry },
    Read { data: String, encoding: Encoding, bytes: usize, next_offset: Option<u64> },
    Write { bytes: usize },
    Mkdir,
    Copy { entries: usize, bytes: u64 },
    Move,
    Delete { entries: usize },
}

/// Schemas are derived from the exact wire types used by both hosts.
pub fn input_schema() -> serde_json::Value {
    { let mut schema = serde_json::to_value(schemars::schema_for!(Request)).expect("file request schema is JSON"); schema["type"] = "object".into(); schema }
}
pub fn output_schema() -> serde_json::Value {
    { let mut schema = serde_json::to_value(schemars::schema_for!(Reply)).expect("file reply schema is JSON"); schema["type"] = "object".into(); schema }
}

/// A host must enforce root containment and refuse special files on every
/// primitive, including destinations. Reads and directory enumeration must
/// stop at the supplied bound rather than allocate an unbounded result.
pub trait Backend {
    fn stat(&self, root: &str, path: &str) -> Result<Entry, String>;
    fn list(&self, root: &str, path: &str, max: usize) -> Result<Vec<Entry>, String>;
    fn read(&self, root: &str, path: &str, offset: u64, length: usize) -> Result<Vec<u8>, String>;
    fn write(&self, root: &str, path: &str, bytes: &[u8], overwrite: bool) -> Result<(), String>;
    fn mkdir(&self, root: &str, path: &str) -> Result<(), String>;
    fn remove(&self, root: &str, path: &str, kind: Kind) -> Result<(), String>;
    fn rename(&self, root: &str, path: &str, destination: &str, overwrite: bool) -> Result<(), String>;
}

pub fn relative(path: &str) -> Result<(), String> {
    if path.len() > 4096 { return Err("path exceeds limit 4096 bytes".into()); }
    if path.ends_with('/') || path.contains("//") {
        return Err("file paths must use single separators without a trailing slash".into());
    }
    if path.starts_with('/') || path.contains('\\') || path.contains(':') || path.contains('\0')
        || path.split('/').any(|p| p == ".." || p == ".") {
        return Err("path must be relative to a named root; absolute paths and dot components are refused".into());
    }
    if path.split('/').filter(|p| !p.is_empty()).count() > MAX_DEPTH {
        return Err(format!("path exceeds depth limit {MAX_DEPTH}"));
    }
    Ok(())
}
fn mutable_path(path: &str) -> Result<(), String> {
    relative(path)?;
    if path.trim_matches('/').is_empty() { return Err("a root itself cannot be modified".into()); }
    Ok(())
}
fn join(parent: &str, name: &str) -> Result<String, String> {
    if name.is_empty() || name.contains('/') { return Err("store returned an invalid entry name".into()); }
    relative(name)?;
    Ok(if parent.is_empty() { name.to_string() } else { format!("{parent}/{name}") })
}
impl Operation {
    pub fn annotations(&self) -> crate::tools::Annotations {
        use crate::tools::Annotations;
        match self {
            Self::List { .. } | Self::Stat | Self::Read { .. } => Annotations::READ,
            Self::Delete { .. } | Self::Move { .. } | Self::Write { overwrite: true, .. } | Self::Copy { overwrite: true, .. } => Annotations::DESTRUCTIVE,
            _ => Annotations::MUTATE,
        }
    }
}

pub fn execute(store: &dyn Backend, request: &Request) -> Result<Reply, String> {
    let Request { root, path, operation } = request;
    if root.is_empty() || root.len() > 64 || !root.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-') {
        return Err("root must be a configured name of 1 to 64 ASCII letters, digits, underscores or hyphens".into());
    }
    relative(path)?;
    match operation {
        Operation::List { offset, limit } => {
            if *limit == 0 || *limit > MAX_PAGE { return Err(format!("list limit must be 1..={MAX_PAGE}")); }
            if *offset > MAX_ENTRIES { return Err(format!("list offset exceeds limit {MAX_ENTRIES}")); }
            let mut entries = store.list(root, path, MAX_ENTRIES)?;
            if entries.len() > MAX_ENTRIES { return Err(format!("directory exceeds entry limit {MAX_ENTRIES}")); }
            entries.sort_by(|a, b| a.name.cmp(&b.name));
            let end = offset.saturating_add(*limit).min(entries.len());
            let next_offset = (end < entries.len()).then_some(end);
            Ok(Reply::List { entries: entries.into_iter().skip(*offset).take(*limit).collect(), next_offset })
        }
        Operation::Stat => Ok(Reply::Stat { entry: store.stat(root, path)? }),
        Operation::Read { offset, length, encoding } => {
            let entry = store.stat(root, path)?;
            if entry.kind != Kind::File { return Err("read requires a regular file".into()); }
            let size = entry.size.ok_or("store cannot determine file size")?;
            let length = match length {
                Some(n) => *n,
                None => usize::try_from(size.saturating_sub(*offset)).unwrap_or(usize::MAX),
            };
            if length > MAX_TRANSFER { return Err(format!("read exceeds transfer limit {MAX_TRANSFER} bytes; specify offset and length for a chunk")); }
            let bytes = store.read(root, path, *offset, length)?;
            if bytes.len() > length { return Err("store exceeded requested read limit".into()); }
            let next = offset.saturating_add(bytes.len() as u64);
            let data = match encoding {
                Encoding::Text => String::from_utf8(bytes.clone()).map_err(|_| "read is not UTF-8; use base64 encoding")?,
                Encoding::Base64 => STANDARD.encode(&bytes),
            };
            Ok(Reply::Read { data, encoding: *encoding, bytes: bytes.len(), next_offset: (next < size).then_some(next) })
        }
        Operation::Write { data, encoding, overwrite } => {
            mutable_path(path)?;
            if data.len() > MAX_TRANSFER.div_ceil(3) * 4 { return Err(format!("write exceeds transfer limit {MAX_TRANSFER} decoded bytes")); }
            let bytes = match encoding {
                Encoding::Text => data.as_bytes().to_vec(),
                Encoding::Base64 => STANDARD.decode(data).map_err(|_| "invalid base64 data")?,
            };
            if bytes.len() > MAX_TRANSFER { return Err(format!("write exceeds transfer limit {MAX_TRANSFER} bytes")); }
            store.write(root, path, &bytes, *overwrite)?;
            Ok(Reply::Write { bytes: bytes.len() })
        }
        Operation::Mkdir => { mutable_path(path)?; store.mkdir(root, path)?; Ok(Reply::Mkdir) }
        Operation::Move { destination, overwrite } => {
            mutable_path(path)?; mutable_path(destination)?;
            reject_overlap(path, destination)?;
            store.rename(root, path, destination, *overwrite)?;
            Ok(Reply::Move)
        }
        Operation::Copy { destination, overwrite, recursive } => {
            mutable_path(path)?; mutable_path(destination)?;
            reject_overlap(path, destination)?;
            let tree = collect(store, root, path, *recursive)?;
            let mut bytes = 0;
            for (source, entry) in &tree {
                let target = format!("{destination}{}", &source[path.len()..]);
                if entry.kind == Kind::Directory { store.mkdir(root, &target)?; }
                else {
                    let n = entry.size.ok_or("store cannot determine file size")?;
                    let content = store.read(root, source, 0, n as usize)?;
                    if content.len() as u64 != n { return Err("file changed during copy; partial destination may remain".into()); }
                    store.write(root, &target, &content, *overwrite)?;
                    bytes += n;
                }
            }
            Ok(Reply::Copy { entries: tree.len(), bytes })
        }
        Operation::Delete { recursive } => {
            mutable_path(path)?;
            let entry = store.stat(root, path)?;
            if entry.kind == Kind::Directory && !recursive {
                store.remove(root, path, Kind::Directory)?;
                return Ok(Reply::Delete { entries: 1 });
            }
            let tree = collect(store, root, path, *recursive)?;
            for (path, entry) in tree.iter().rev() { store.remove(root, path, entry.kind)?; }
            Ok(Reply::Delete { entries: tree.len() })
        }
    }
}
fn reject_overlap(source: &str, destination: &str) -> Result<(), String> {
    let a = source.trim_end_matches('/'); let b = destination.trim_end_matches('/');
    if a == b || b.starts_with(&format!("{a}/")) || a.starts_with(&format!("{b}/")) {
        return Err("source and destination must not overlap".into());
    }
    Ok(())
}
fn collect(store: &dyn Backend, root: &str, path: &str, recursive: bool) -> Result<Vec<(String, Entry)>, String> {
    let mut pending = vec![(path.to_string(), 0)];
    let mut result = Vec::new();
    let mut bytes = 0u64;
    while let Some((path, depth)) = pending.pop() {
        if depth > MAX_DEPTH { return Err(format!("tree exceeds depth limit {MAX_DEPTH}")); }
        let entry = store.stat(root, &path)?;
        if entry.kind == Kind::Directory {
            if !recursive { return Err("directory copy requires recursive: true".into()); }
            for child in store.list(root, &path, MAX_ENTRIES)? {
                if pending.len() + result.len() + 1 >= MAX_ENTRIES { return Err(format!("tree exceeds entry limit {MAX_ENTRIES}")); }
                pending.push((join(&path, &child.name)?, depth + 1));
            }
        } else {
            let n = entry.size.ok_or("store cannot determine file size")?;
            if n > MAX_FILE { return Err(format!("file exceeds limit {MAX_FILE} bytes")); }
            bytes = bytes.saturating_add(n);
            if bytes > MAX_TREE_BYTES { return Err(format!("tree exceeds byte limit {MAX_TREE_BYTES}")); }
        }
        result.push((path, entry));
    }
    Ok(result)
}


pub const OPERATIONS: &[&str] = &["list", "stat", "read", "write", "mkdir", "copy", "move", "delete"];

fn variant(schema: &serde_json::Value, op: &str) -> serde_json::Value {
    let branches = schema.get("oneOf").or_else(|| schema.get("anyOf")).and_then(serde_json::Value::as_array).expect("tagged enum has branches");
    let mut selected = branches.iter().find(|v| {
        let tag = &v["properties"]["op"];
        tag["const"].as_str() == Some(op) || tag["enum"].as_array().is_some_and(|values| values.iter().any(|v| v.as_str()==Some(op)))
    }).unwrap_or_else(|| panic!("missing files schema variant {op}")).clone();
    if let Some(defs) = schema.get("$defs") { selected["$defs"] = defs.clone(); }
    selected
}

/// Operation-specific script signatures, derived from the same wire enum.
pub fn operation_input_schema(op: &str) -> serde_json::Value {
    let schema = serde_json::to_value(schemars::schema_for!(Operation)).unwrap();
    let mut selected = variant(&schema, op);
    selected["properties"].as_object_mut().unwrap().remove("op");
    let required = selected["required"].as_array_mut().unwrap();
    required.retain(|v| v.as_str()!=Some("op"));
    required.push(serde_json::json!("root"));
    selected["properties"]["root"] = serde_json::json!({"type":"string","minLength":1,"maxLength":64,"description":"documents/workspace, artifacts, or a host-configured root"});
    selected["properties"]["path"] = serde_json::json!({"type":"string","default":"","maxLength":4096,"description":"Path relative to root; no absolute or parent components"});
    selected
}
pub fn operation_output_schema(op: &str) -> serde_json::Value { variant(&output_schema(), op) }

pub fn operation_request(op: &str, mut args: serde_json::Value) -> Result<serde_json::Value, String> {
    let object = args.as_object_mut().ok_or("arguments must be an object")?;
    if object.contains_key("op") { return Err("script file methods do not take op; the method selects it".into()); }
    object.insert("op".into(), op.into());
    Ok(args)
}

/// Reusable script registry; the host supplies a single request handler.
pub fn script_tools(handler: std::sync::Arc<dyn Fn(serde_json::Value) -> crate::tools::ToolFuture + Send + Sync>) -> Vec<crate::tools::ToolSpec> {
    OPERATIONS.iter().map(|&op| {
        let handler = handler.clone();
        let annotations = match op {
            "list"|"stat"|"read" => crate::tools::Annotations::READ,
            "mkdir" => crate::tools::Annotations::MUTATE,
            _ => crate::tools::Annotations::DESTRUCTIVE,
        };
        let mut spec = crate::tools::ToolSpec::new(format!("files_{op}"), "files", format!("{op} files under named roots. Bounded file operations; overwriting requires overwrite:true; directory copy/delete requires recursive:true. Metadata conservatively marks methods that can overwrite as destructive."), operation_input_schema(op), annotations, move |args| {
            let handler = handler.clone();
            Box::pin(async move { handler(operation_request(op, args)?).await })
        });
        spec.output_schema = operation_output_schema(op);
        spec
    }).collect()
}
