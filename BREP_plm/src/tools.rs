//! The tools library: the hand tools, drivers and fixtures manufacturing
//! instructions name in their steps (a name, a picture and a spec).
//!
//! A step keeps the tool's id AND a snapshot of its name and spec, so a PDF or
//! an offline copy needs no server; the library is where the picker finds
//! them. A tool is therefore never deleted, only retired: old instructions
//! still name it, and the picker stops offering it.
//!
//! The list lives in the state like the companies of [`crate::sourcing`]:
//! every write is audited by the store's diff, and a picture is a blob of the
//! attachment store ([`crate::attach`]), kept while a tool references it.

use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

use crate::attach::{self, Staged};
use crate::db::{now, Db, State};
use crate::model::{Timestamp, User};
use crate::Error;

/// Picture media types: raster images a browser shows without running
/// anything (no SVG, which can carry script).
pub const PICTURE_TYPES: [&str; 3] = ["image/png", "image/jpeg", "image/webp"];

/// The largest picture, in bytes.
pub const PICTURE_MAX: u64 = 2 * 1024 * 1024;

const MAX_NAME: usize = 200;
const MAX_TEXT: usize = 2000;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Tool {
    /// `TL-000001`: handed out in order, never reused.
    pub id: String,
    pub name: String,
    /// What a step needs to know about it: a size, a range, a setting.
    #[serde(default)]
    pub spec: String,
    /// Free text a library is sorted by (`Torque wrench`, `Fixture`).
    #[serde(default)]
    pub kind: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub picture: Option<Picture>,
    /// Hidden from the picker; instructions that name it keep their snapshot.
    #[serde(default)]
    pub retired: bool,
    pub created_by: String,
    pub created_at: Timestamp,
    pub modified_at: Timestamp,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Picture {
    pub sha256: String,
    pub media_type: String,
    pub size: u64,
}

/// The picture blobs the library references ([`attach::referenced`]).
pub fn pictures(state: &State) -> impl Iterator<Item = &str> {
    state.tools.iter().filter_map(|t| t.picture.as_ref().map(|p| p.sha256.as_str()))
}

fn text(fields: &Map<String, Value>, key: &str, max: usize) -> Result<Option<String>, Error> {
    match fields.get(key) {
        None => Ok(None),
        Some(Value::String(s)) if s.chars().count() <= max => Ok(Some(s.trim().to_string())),
        Some(Value::String(_)) => Err(Error::bad_request(format!("a tool's {key} is at most {max} characters"))),
        Some(_) => Err(Error::bad_request(format!("a tool's {key} is text"))),
    }
}

/// Apply `fields` (`name`, `spec`, `kind`, `retired`) to `tool`, refusing
/// anything else and a name another live tool has.
fn apply(state: &State, tool: &mut Tool, fields: &Value) -> Result<(), Error> {
    let fields = fields.as_object().ok_or_else(|| Error::bad_request("a tool is a JSON object"))?;
    if let Some(key) = fields.keys().find(|k| !["name", "spec", "kind", "retired"].contains(&k.as_str())) {
        return Err(Error::bad_request(format!("a tool has no field '{key}'")));
    }
    if let Some(name) = text(fields, "name", MAX_NAME)? {
        tool.name = name;
    }
    if let Some(spec) = text(fields, "spec", MAX_TEXT)? {
        tool.spec = spec;
    }
    if let Some(kind) = text(fields, "kind", MAX_NAME)? {
        tool.kind = kind;
    }
    match fields.get("retired") {
        None => {}
        Some(Value::Bool(retired)) => tool.retired = *retired,
        Some(_) => return Err(Error::bad_request("retired is true or false")),
    }
    if tool.name.is_empty() {
        return Err(Error::bad_request("a tool needs a name"));
    }
    if !tool.retired {
        if let Some(twin) = state.tools.iter().find(|t| t.id != tool.id && !t.retired && t.name.eq_ignore_ascii_case(&tool.name)) {
            return Err(Error::conflict(format!("{} is already called '{}'", twin.id, twin.name)));
        }
    }
    Ok(())
}

impl Db {
    pub fn create_tool(&self, user: &User, fields: &Value) -> Result<Tool, Error> {
        let fields = fields.clone();
        let user_id = user.id.clone();
        self.mutate(move |state| {
            state.next_tool += 1;
            let stamp = now();
            let mut tool = Tool {
                id: format!("TL-{:06}", state.next_tool),
                name: String::new(),
                spec: String::new(),
                kind: String::new(),
                picture: None,
                retired: false,
                created_by: user_id,
                created_at: stamp,
                modified_at: stamp,
            };
            apply(state, &mut tool, &fields)?;
            state.tools.push(tool.clone());
            Ok(tool)
        })
    }

    pub fn update_tool(&self, id: &str, fields: &Value) -> Result<Tool, Error> {
        let (id, fields) = (id.to_string(), fields.clone());
        self.mutate(move |state| {
            let index = state.tools.iter().position(|t| t.id == id).ok_or_else(|| Error::not_found("tool"))?;
            let mut tool = state.tools[index].clone();
            apply(state, &mut tool, &fields)?;
            tool.modified_at = now();
            state.tools[index] = tool.clone();
            Ok(tool)
        })
    }

    /// Set a tool's picture to an uploaded image; the old one's blob goes
    /// when nothing else references it.
    pub fn set_tool_picture(&self, id: &str, staged: Staged, media_type: &str) -> Result<Tool, Error> {
        let media_type = media_type.split(';').next().unwrap_or("").trim().to_ascii_lowercase();
        if !PICTURE_TYPES.contains(&media_type.as_str()) {
            return Err(Error::bad_request(format!("a tool picture is PNG, JPEG or WebP, not '{media_type}'")));
        }
        if staged.size > PICTURE_MAX {
            return Err(Error {
                status: axum::http::StatusCode::PAYLOAD_TOO_LARGE,
                message: format!("a tool picture is at most {} MB", PICTURE_MAX / (1024 * 1024)),
            });
        }
        let (id, root) = (id.to_string(), self.root().to_path_buf());
        self.mutate(move |state| {
            let index = state.tools.iter().position(|t| t.id == id).ok_or_else(|| Error::not_found("tool"))?;
            attach::place(&root, &staged)?;
            let old = state.tools[index].picture.replace(Picture { sha256: staged.sha256.clone(), media_type, size: staged.size });
            state.tools[index].modified_at = now();
            if let Some(old) = old {
                attach::remove_if_unreferenced(state, &root, &old.sha256)?;
            }
            Ok(state.tools[index].clone())
        })
    }
}
