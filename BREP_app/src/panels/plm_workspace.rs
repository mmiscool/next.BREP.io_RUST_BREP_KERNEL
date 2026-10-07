//! The workspace browser (plm-cad-integration-todo §3 S14): in PLM mode it replaces the
//! file explorer. Decisions D9, D12 and D13; the server half is P6
//! (`BREP_plm/src/workspace.rs`).
//!
//! * **Folders of links and files, per user.** A user arranges folders; a **link** points at
//!   a part and follows its newest revision (drafts included), or is pinned to one revision.
//!   A link is never a copy: opening it opens the part's document, and deleting it never
//!   touches the part. A folder can also hold **files** (a customer's photo, notes), whose
//!   every replace keeps the version before.
//! * **Other users' workspaces**, read-only, when the administrator lets workspaces be
//!   browsed (`workspaces_browsable`, on by default). The server refuses otherwise, and its
//!   sentence is shown.
//! * **Open by number (D13).** Part and Revision fields: a part id or number fills Revision
//!   with the newest revision, drafts included; the user may pick another before opening.
//! * **The verbs are the workspace's (D9).** New folder, Rename, Move and Delete act on the
//!   workspace alone. Delete removes the link or the file, never a part or a revision.
//! * **Files:** add one (the host hands the bytes in — [`WorkspaceBrowser::add_file`]; the
//!   shell has no native file chooser, so a file arrives by drag and drop), open or download
//!   a version, list versions, restore one (as a new version), and **Promote to attachment**
//!   on a part or revision, which obeys every attachment refusal (S8).
//!
//! Every refusal is the server's own sentence, shown as sent. The browser talks through
//! [`Workspaces`], which S1's client implements ([`PlmWorkspaces`]); the golden tests below
//! run it against the REAL router. Nothing here is constructed without a server.

use crate::plm::PlmFuture;
use eframe::egui;
use serde::Deserialize;
use serde_json::{json, Value};
use std::collections::HashMap;
use std::task::Waker;

use super::plm_attachments::{human_size, media_type_for, Attachment};
use super::plm_parts::Pending;

// --- the wire ------------------------------------------------------------------
//
// These mirror `BREP_plm/src/workspace.rs` and `src/api/workspace.rs`. The golden tests
// below pin them against the REAL router. Every field defaults, so a server that adds a
// field breaks nothing.

/// A workspace the user may open (`GET /api/workspaces`).
#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct WorkspaceRow {
    pub user_id: String,
    pub username: String,
    pub display_name: String,
    pub mine: bool,
    pub entries: usize,
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct WorkspaceList {
    pub workspaces: Vec<WorkspaceRow>,
    /// The administrator's setting: whether other users' workspaces may be browsed.
    pub browsable: bool,
}

/// What a link resolves to now.
#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct LinkView {
    pub thumbnail_url: String,
    pub part_id: String,
    /// Pinned to one revision; `false` follows the newest, drafts included.
    pub pinned: bool,
    pub number: String,
    pub name: String,
    pub document_class: String,
    pub revision_id: String,
    pub revision_label: String,
    pub lifecycle: String,
    /// `part/<part>/rev/<revision>`: what opens.
    pub document_key: String,
    pub part_missing: bool,
    /// Pinned to a draft that was since deleted.
    pub revision_missing: bool,
}

/// A file's current version, in brief.
#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct FileView {
    pub version: u32,
    pub versions: usize,
    pub size: u64,
    pub media_type: String,
    pub sha256: String,
    pub uploaded_at: u64,
    pub uploaded_by_name: String,
}

/// One entry: a folder, a link or a file.
#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct Entry {
    pub id: String,
    pub owner: String,
    pub owner_name: String,
    pub parent: String,
    pub name: String,
    /// `folder`, `link` or `file`.
    pub kind: String,
    pub created_at: u64,
    pub modified_at: u64,
    /// A folder's entry count.
    pub children: Option<usize>,
    pub link: Option<LinkView>,
    pub file: Option<FileView>,
}

impl Entry {
    pub fn is_folder(&self) -> bool {
        self.kind == "folder"
    }

    /// What the row says after the name.
    pub fn detail(&self) -> String {
        match (&self.link, &self.file, self.children) {
            (Some(link), _, _) if link.part_missing => "the part is gone".into(),
            (Some(link), _, _) if link.revision_missing => format!("{} — the pinned revision was deleted", link.number),
            (Some(link), _, _) => {
                let how = if link.pinned { "pinned" } else { "newest" };
                format!("{} rev {} ({}, {how})", link.number, link.revision_label, link.lifecycle)
            }
            (_, Some(file), _) => format!("v{} of {} · {}", file.version, file.versions, human_size(file.size)),
            (_, _, Some(n)) => format!("{n} item(s)"),
            _ => String::new(),
        }
    }
}

#[derive(Deserialize)]
struct Folder {
    #[serde(default)]
    entries: Vec<Entry>,
}

#[derive(Deserialize)]
struct Made {
    entry: Entry,
}

#[derive(Deserialize)]
struct Promoted {
    attachment: Attachment,
}

/// One version of a file (`GET …/versions`).
#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct Version {
    pub version: u32,
    pub sha256: String,
    pub size: u64,
    pub media_type: String,
    pub uploaded_at: u64,
    pub uploaded_by_name: String,
    /// The version this one restored.
    pub restored_from: Option<u32>,
    pub current: bool,
}

#[derive(Deserialize)]
struct Versions {
    #[serde(default)]
    versions: Vec<Version>,
}

/// A revision as `GET /api/parts/:id` lists it.
#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct RevisionRef {
    pub id: String,
    pub label: String,
    pub lifecycle: String,
    pub document_key: String,
}

/// A part looked up by id or number (D13).
#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct PartLookup {
    pub id: String,
    pub number: String,
    pub name: String,
    /// The newest by creation, drafts included — what the Revision field is filled with.
    pub newest_revision: Option<RevisionRef>,
    pub revision_views: Vec<RevisionRef>,
}

/// Where a file is promoted to.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PromoteTo {
    /// A part id or number.
    pub part: String,
    /// A revision id or label; empty attaches it to the part.
    pub revision: String,
    /// 0: the current version.
    pub version: u32,
    pub kind: String,
}

/// The PLM's workspace routes, as the browser needs them. An `Err` is the sentence to
/// show: the server's refusal, word for word, or why no answer came.
pub trait Workspaces {
    fn session(&self) -> Option<std::rc::Rc<crate::plm::client::PlmClient>> { None }
    fn thumbnail(&self, _url: &str) -> PlmFuture<Option<Vec<u8>>> {
        Box::pin(async { Ok(None) })
    }
    fn workspaces(&self) -> PlmFuture<WorkspaceList>;
    /// One folder of `owner`'s workspace (empty `owner`: mine; empty `parent`: the top).
    fn folder(&self, owner: &str, parent: &str) -> PlmFuture<Vec<Entry>>;
    fn create_folder(&self, parent: &str, name: &str) -> PlmFuture<Entry>;
    /// Link a part (id or number) here; `revision` (id or label) pins it, empty follows.
    fn link(&self, parent: &str, part: &str, revision: &str) -> PlmFuture<Entry>;
    fn add_file(&self, parent: &str, name: &str, bytes: Vec<u8>) -> PlmFuture<Entry>;
    fn replace_file(&self, id: &str, name: &str, bytes: Vec<u8>) -> PlmFuture<Entry>;
    /// `{name}`, `{parent}`, `{revision}` — any of them.
    fn update(&self, id: &str, fields: Value) -> PlmFuture<Entry>;
    fn delete(&self, id: &str, recursive: bool) -> PlmFuture<()>;
    fn versions(&self, id: &str) -> PlmFuture<Vec<Version>>;
    fn restore(&self, id: &str, version: u32) -> PlmFuture<Entry>;
    /// A version's bytes (0: the current one).
    fn download(&self, id: &str, version: u32) -> PlmFuture<Vec<u8>>;
    fn promote(&self, id: &str, to: &PromoteTo) -> PlmFuture<Attachment>;
    /// `GET /api/parts/:id` by id or number.
    fn lookup(&self, part: &str) -> PlmFuture<PartLookup>;
}

/// [`Workspaces`] over S1's [`PlmClient`](crate::plm::client::PlmClient).
pub struct PlmWorkspaces {
    pub client: std::rc::Rc<crate::plm::client::PlmClient>,
}

fn encode(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for byte in text.bytes() {
        if byte.is_ascii_alphanumeric() || b"-._~".contains(&byte) {
            out.push(byte as char);
        } else {
            out.push_str(&format!("%{byte:02X}"));
        }
    }
    out
}

impl PlmWorkspaces {
    fn read<T: for<'de> Deserialize<'de> + 'static>(
        &self,
        method: &'static str,
        path: String,
        body: Option<Value>,
    ) -> PlmFuture<T> {
        let client = self.client.clone();
        Box::pin(async move {
            let body = body.map(|b| serde_json::to_vec(&b).unwrap_or_default());
            let response = client.call(method, &path, body).await.map_err(|e| e.to_string())?;
            serde_json::from_slice(&response.body)
                .map_err(|e| format!("the PLM answered {path} with something this app cannot read: {e}"))
        })
    }

    fn entry(&self, method: &'static str, path: String, body: Option<Value>) -> PlmFuture<Entry> {
        let made: PlmFuture<Made> = self.read(method, path, body);
        Box::pin(async move { Ok(made.await?.entry) })
    }

    fn bytes_entry(&self, method: &'static str, path: String, name: &str, bytes: Vec<u8>) -> PlmFuture<Entry> {
        let client = self.client.clone();
        let media_type = media_type_for(name).to_string();
        Box::pin(async move {
            let response = client.call_typed(method, &path, bytes, &media_type).await.map_err(|e| e.to_string())?;
            serde_json::from_slice::<Made>(&response.body)
                .map(|m| m.entry)
                .map_err(|e| format!("the PLM answered the upload with something this app cannot read: {e}"))
        })
    }
}

impl Workspaces for PlmWorkspaces {
    fn session(&self) -> Option<std::rc::Rc<crate::plm::client::PlmClient>> { Some(self.client.clone()) }
    fn thumbnail(&self, url: &str) -> PlmFuture<Option<Vec<u8>>> {
        crate::plm::thumbnail_view::fetch(self.client.clone(), url.to_string())
    }
    fn workspaces(&self) -> PlmFuture<WorkspaceList> {
        self.read("GET", "/api/workspaces".into(), None)
    }

    fn folder(&self, owner: &str, parent: &str) -> PlmFuture<Vec<Entry>> {
        let folder: PlmFuture<Folder> =
            self.read("GET", format!("/api/workspace/entries?owner={}&parent={}", encode(owner), encode(parent)), None);
        Box::pin(async move { Ok(folder.await?.entries) })
    }

    fn create_folder(&self, parent: &str, name: &str) -> PlmFuture<Entry> {
        self.entry("POST", "/api/workspace/folders".into(), Some(json!({ "parent": parent, "name": name })))
    }

    fn link(&self, parent: &str, part: &str, revision: &str) -> PlmFuture<Entry> {
        self.entry("POST", "/api/workspace/links".into(), Some(json!({ "parent": parent, "part": part, "revision": revision })))
    }

    fn add_file(&self, parent: &str, name: &str, bytes: Vec<u8>) -> PlmFuture<Entry> {
        let path = format!("/api/workspace/files?parent={}&name={}", encode(parent), encode(name));
        self.bytes_entry("POST", path, name, bytes)
    }

    fn replace_file(&self, id: &str, name: &str, bytes: Vec<u8>) -> PlmFuture<Entry> {
        self.bytes_entry("PUT", format!("/api/workspace/entries/{}/content", encode(id)), name, bytes)
    }

    fn update(&self, id: &str, fields: Value) -> PlmFuture<Entry> {
        self.entry("PATCH", format!("/api/workspace/entries/{}", encode(id)), Some(fields))
    }

    fn delete(&self, id: &str, recursive: bool) -> PlmFuture<()> {
        let client = self.client.clone();
        let path = format!("/api/workspace/entries/{}?recursive={recursive}", encode(id));
        Box::pin(async move {
            client.call("DELETE", &path, None).await.map_err(|e| e.to_string())?;
            Ok(())
        })
    }

    fn versions(&self, id: &str) -> PlmFuture<Vec<Version>> {
        let versions: PlmFuture<Versions> = self.read("GET", format!("/api/workspace/entries/{}/versions", encode(id)), None);
        Box::pin(async move { Ok(versions.await?.versions) })
    }

    fn restore(&self, id: &str, version: u32) -> PlmFuture<Entry> {
        self.entry("POST", format!("/api/workspace/entries/{}/versions/{version}/restore", encode(id)), None)
    }

    fn download(&self, id: &str, version: u32) -> PlmFuture<Vec<u8>> {
        let client = self.client.clone();
        let path = format!("/api/workspace/entries/{}/content?version={version}", encode(id));
        Box::pin(async move { Ok(client.call("GET", &path, None).await.map_err(|e| e.to_string())?.body) })
    }

    fn promote(&self, id: &str, to: &PromoteTo) -> PlmFuture<Attachment> {
        let body = json!({ "part": to.part, "revision": to.revision, "version": to.version, "kind": to.kind });
        let promoted: PlmFuture<Promoted> = self.read("POST", format!("/api/workspace/entries/{}/promote", encode(id)), Some(body));
        Box::pin(async move { Ok(promoted.await?.attachment) })
    }

    fn lookup(&self, part: &str) -> PlmFuture<PartLookup> {
        self.read("GET", format!("/api/parts/{}", encode(part.trim())), None)
    }
}

// --- the host's half ---------------------------------------------------------------

/// `part/<part>/rev/<revision>` out of a store name, however the store spells it
/// (`/models/part/<p>/rev/<r>.nbrep` natively, the bare key on the web).
pub fn key_of(name: &str) -> Option<String> {
    let tail = &name[name.find("part/")?..];
    let segments: Vec<&str> = tail.split('/').collect();
    match segments.as_slice() {
        ["part", part, "rev", revision] if !part.is_empty() && !revision.is_empty() => {
            let revision = revision.split('.').next().unwrap_or(revision);
            (!revision.is_empty()).then(|| crate::plm::identity::document_key(&part, &revision))
        }
        _ => None,
    }
}

/// The name the store opens the document `key` under: its one spelling, as the explorer
/// spelled it before S14 (`/models/part/<p>/rev/<r>.nbrep` on the mirror store), so a
/// document opened from the workspace and one opened any other way are the same tab.
pub fn document_name(store: &dyn crate::store::ModelStore, key: &str) -> String {
    store.canonical_identity(key)
}

/// Files dropped on the window this frame: (name, bytes). The web hands the bytes; a
/// native drop hands a path, read here. This is how a file reaches the workspace — the
/// shell has no file chooser of its own for arbitrary files.
pub fn dropped(ctx: &egui::Context) -> Vec<(String, Vec<u8>)> {
    let files = ctx.input(|i| i.raw.dropped_files.clone());
    files
        .into_iter()
        .filter_map(|file| {
            let name = if file.name.is_empty() {
                file.path.as_ref()?.file_name()?.to_string_lossy().into_owned()
            } else {
                file.name.clone()
            };
            let bytes = match (&file.bytes, &file.path) {
                (Some(bytes), _) => bytes.to_vec(),
                (None, Some(path)) => std::fs::read(path).ok()?,
                _ => return None,
            };
            Some((name, bytes))
        })
        .collect()
}

// --- the browser -----------------------------------------------------------------

/// A pinned folder: its path from the top of the user's own workspace, (id, name) each.
pub type Pin = Vec<(String, String)>;

/// How a workspace folder is spelled in `@pinned` (the reserved preference S0 set aside
/// for workspace folders in PLM mode): `workspace:` and the folder's path as JSON. The
/// list stays an array of strings, the shape the file explorer keeps its pins in.
pub fn pin_text(pin: &Pin) -> String {
    format!("workspace:{}", serde_json::to_string(pin).unwrap_or_default())
}

/// The workspace folders in an `@pinned` value; anything else in it is left alone.
pub fn pins_in(value: Option<&str>) -> Vec<Pin> {
    let list: Vec<String> = value.and_then(|v| serde_json::from_str(v).ok()).unwrap_or_default();
    list.iter()
        .filter_map(|item| serde_json::from_str::<Pin>(item.strip_prefix("workspace:")?).ok())
        .filter(|pin| !pin.is_empty())
        .collect()
}

/// `value` (an `@pinned` value) with its workspace folders replaced by `pins`; the other
/// entries are kept as they were.
pub fn with_pins(value: Option<&str>, pins: &[Pin]) -> String {
    let list: Vec<String> = value.and_then(|v| serde_json::from_str(v).ok()).unwrap_or_default();
    let mut out: Vec<String> = list.into_iter().filter(|item| !item.starts_with("workspace:")).collect();
    out.extend(pins.iter().map(pin_text));
    serde_json::to_string(&out).unwrap_or_else(|_| "[]".into())
}

/// Read the pinned workspace folders from the store's `@pinned`.
pub fn load_pins(store: &dyn crate::store::ModelStore) -> Vec<Pin> {
    pins_in(store.read(crate::store::PINNED_KEY).as_deref())
}

/// Write `pins` into the store's `@pinned`, keeping whatever else it holds.
pub fn save_pins(store: &dyn crate::store::ModelStore, pins: &[Pin]) -> Result<(), String> {
    let current = store.read(crate::store::PINNED_KEY);
    store.write(crate::store::PINNED_KEY, &with_pins(current.as_deref(), pins))
}

/// What the host does after a frame.
#[derive(Debug, Default, PartialEq)]
pub struct WorkspaceOutcome {
    /// Open this document (`part/<part>/rev/<revision>`): a link was activated, or
    /// Open by number was pressed.
    pub open: Option<String>,
    /// A file's bytes, to save where the user chooses: (file name, bytes).
    pub downloaded: Option<(String, Vec<u8>)>,
    /// The pinned folders changed: save this list to `@pinned` ([`save_pins`]).
    pub pins: Option<Vec<Pin>>,
    /// "Add file…" was pressed: open the file chooser, then
    /// [`WorkspaceBrowser::add_file`] what it delivers.
    pub pick_file: bool,
}

/// What a finished request changed.
enum Done {
    Entry(Entry),
    Nothing,
    Attachment(Attachment),
}

/// One user's workspace, one folder at a time.
pub struct WorkspaceBrowser {
    session: Option<std::rc::Rc<crate::plm::client::PlmClient>>,
    thumbnails: crate::plm::thumbnail_view::ThumbnailCache,
    /// Whether the host has a file chooser: "Add file…" beside drag and drop.
    pub can_pick_files: bool,
    pub browse_only: bool,
    /// "Add file…" was pressed this frame.
    pick_asked: bool,
    /// Whose workspace: empty is the signed-in user's own.
    pub owner: String,
    pub owner_label: String,
    /// The folders from the top down to the one shown: (id, name).
    pub path: Vec<(String, String)>,
    pub entries: Option<Vec<Entry>>,
    pub workspaces: Option<WorkspaceList>,
    /// The selected entry, by id.
    pub selected: Option<String>,
    /// An entry cut for a move, pasted into the folder shown.
    pub cut: Option<Entry>,
    /// A file's versions, once asked for: (entry id, versions).
    pub versions: Option<(String, Vec<Version>)>,
    /// The last refusal or failure, word for word.
    pub problem: Option<String>,
    /// What just happened, for the status line.
    pub notice: Option<String>,
    pub new_folder_name: String,
    pub rename: Option<String>,
    pub link_part: String,
    pub link_revision: String,
    pub open_part: String,
    pub open_revision: String,
    /// The part Open by number looked up, for its revision picker.
    pub open_lookup: Option<PartLookup>,
    pub promote: Option<PromoteTo>,
    /// Delete was pressed once on the selected entry; the second press deletes.
    pub delete_armed: bool,
    /// The user's pinned folders (`@pinned`), once the host has read them.
    pub pins: Option<Vec<Pin>>,
    list_pending: Option<Pending<Vec<Entry>>>,
    owners_pending: Option<Pending<WorkspaceList>>,
    /// Every change still out: two started in one frame (a catalog link and a New part)
    /// both land.
    action_pending: Vec<(String, Pending<Done>)>,
    /// A lookup, and whether to open what it finds.
    lookup_pending: Option<(bool, Pending<PartLookup>)>,
    download_pending: Option<(String, Pending<Vec<u8>>)>,
    versions_pending: Option<(String, Pending<Vec<Version>>)>,
    pub hits: HashMap<String, egui::Rect>,
    /// A pin toggled while the browser drew, handed out in the frame's outcome.
    pins_changed: Option<Vec<Pin>>,
}

impl Default for WorkspaceBrowser {
    fn default() -> Self {
        Self::new()
    }
}

impl WorkspaceBrowser {
    /// The signed-in user's own workspace, at the top.
    pub fn new() -> Self {
        WorkspaceBrowser {
            session: None,
            thumbnails: Default::default(),
            can_pick_files: false,
            browse_only: false,
            pick_asked: false,
            owner: String::new(),
            owner_label: "My workspace".into(),
            path: Vec::new(),
            entries: None,
            workspaces: None,
            selected: None,
            cut: None,
            versions: None,
            problem: None,
            notice: None,
            new_folder_name: String::new(),
            rename: None,
            link_part: String::new(),
            link_revision: String::new(),
            open_part: String::new(),
            open_revision: String::new(),
            open_lookup: None,
            promote: None,
            delete_armed: false,
            pins: None,
            list_pending: None,
            owners_pending: None,
            action_pending: Vec::new(),
            lookup_pending: None,
            download_pending: None,
            versions_pending: None,
            hits: HashMap::new(),
            pins_changed: None,
        }
    }

    /// The folder shown, by id; empty at the top. A new part saved from here is linked
    /// into it (D9).
    pub fn current_folder(&self) -> &str {
        self.path.last().map(|(id, _)| id.as_str()).unwrap_or("")
    }

    pub fn thumbnail_entries(&self) -> Vec<&str> {
        self.entries.iter().flatten().filter(|entry| entry.link.as_ref()
            .is_some_and(|link| self.thumbnails.ready(&link.thumbnail_url)))
            .map(|entry| entry.id.as_str()).collect()
    }

    /// Another user's workspace: browse only.
    pub fn read_only(&self) -> bool {
        self.browse_only || !self.owner.is_empty()
    }

    pub fn busy(&self) -> bool {
        self.thumbnails.busy()
            || self.list_pending.as_ref().is_some_and(Pending::is_open)
            || self.owners_pending.as_ref().is_some_and(Pending::is_open)
            || self.action_pending.iter().any(|(_, p)| p.is_open())
            || self.lookup_pending.as_ref().is_some_and(|(_, p)| p.is_open())
            || self.download_pending.as_ref().is_some_and(|(_, p)| p.is_open())
            || self.versions_pending.as_ref().is_some_and(|(_, p)| p.is_open())
    }

    /// Ask again for the folder shown, and for whose workspaces may be opened.
    pub fn reload(&mut self, plm: &dyn Workspaces) {
        self.list_pending = Some(Pending::new(plm.folder(&self.owner, self.current_folder())));
        self.owners_pending = Some(Pending::new(plm.workspaces()));
    }

    fn reload_folder(&mut self, plm: &dyn Workspaces) {
        self.list_pending = Some(Pending::new(plm.folder(&self.owner, self.current_folder())));
    }

    /// Show another user's workspace (read-only), or the user's own (`owner` empty).
    pub fn set_owner(&mut self, plm: &dyn Workspaces, owner: &str, label: &str) {
        self.owner = owner.to_string();
        self.owner_label = label.to_string();
        self.path.clear();
        self.selection_cleared();
        self.entries = None;
        self.reload_folder(plm);
    }

    fn selection_cleared(&mut self) {
        self.selected = None;
        self.rename = None;
        self.promote = None;
        self.versions = None;
        self.delete_armed = false;
    }

    /// Walk into a folder of the one shown.
    pub fn enter(&mut self, plm: &dyn Workspaces, folder: &Entry) {
        self.path.push((folder.id.clone(), folder.name.clone()));
        self.selection_cleared();
        self.entries = None;
        self.reload_folder(plm);
    }

    /// Go to a pinned folder of the user's own workspace.
    pub fn go_to_pin(&mut self, plm: &dyn Workspaces, pin: &Pin) {
        self.owner.clear();
        self.owner_label = "My workspace".into();
        self.path = pin.clone();
        self.selection_cleared();
        self.entries = None;
        self.reload_folder(plm);
    }

    /// Pin the folder shown, or unpin it if it is pinned: the new list for `@pinned`.
    pub fn toggle_pin(&mut self) -> Option<Vec<Pin>> {
        if self.read_only() || self.path.is_empty() {
            return None;
        }
        let mut pins = self.pins.clone().unwrap_or_default();
        let here = self.current_folder().to_string();
        let before = pins.len();
        pins.retain(|pin| pin.last().map(|(id, _)| id.as_str()) != Some(here.as_str()));
        if pins.len() == before {
            pins.push(self.path.clone());
        }
        self.pins = Some(pins.clone());
        Some(pins)
    }

    /// Walk back up to `depth` folders from the top (0: the top).
    pub fn up_to(&mut self, plm: &dyn Workspaces, depth: usize) {
        self.path.truncate(depth);
        self.selection_cleared();
        self.entries = None;
        self.reload_folder(plm);
    }

    fn entry(&self, id: &str) -> Option<&Entry> {
        self.entries.as_ref()?.iter().find(|e| e.id == id)
    }

    /// The selected entry.
    pub fn selection(&self) -> Option<&Entry> {
        self.entry(self.selected.as_deref()?)
    }

    /// Select the entry named `name` in the folder shown (what a click does).
    pub fn select_named(&mut self, name: &str) -> bool {
        let found = self.entries.as_ref().and_then(|list| list.iter().find(|e| e.name == name)).map(|e| e.id.clone());
        let changed = found != self.selected;
        if changed {
            self.selection_cleared();
        }
        self.selected = found;
        self.selected.is_some()
    }

    /// A double click: walk into a folder, open a link's document, download a file.
    pub fn activate(&mut self, plm: &dyn Workspaces, entry: &Entry, outcome: &mut WorkspaceOutcome) {
        match (entry.kind.as_str(), &entry.link) {
            ("folder", _) => self.enter(plm, entry),
            ("link", Some(link)) if link.part_missing => self.problem = Some(format!("{}: the part it links is gone", entry.name)),
            ("link", Some(link)) if link.revision_missing || link.document_key.is_empty() => {
                self.problem = Some(format!(
                    "{}: it is pinned to a revision of {} that was deleted — pin it to another, or let it follow the newest",
                    entry.name, link.number
                ))
            }
            ("link", Some(link)) => outcome.open = Some(link.document_key.clone()),
            ("file", _) => self.download(plm, entry, 0),
            _ => {}
        }
    }

    fn act(&mut self, what: impl Into<String>, future: PlmFuture<Done>) {
        self.problem = None;
        self.action_pending.push((what.into(), Pending::new(future)));
    }

    fn map<T: 'static>(future: PlmFuture<T>, done: impl FnOnce(T) -> Done + 'static) -> PlmFuture<Done> {
        Box::pin(async move { future.await.map(done) })
    }

    /// New folder in the folder shown.
    pub fn new_folder(&mut self, plm: &dyn Workspaces, name: &str) {
        let future = Self::map(plm.create_folder(self.current_folder(), name.trim()), Done::Entry);
        self.act(format!("made folder {}", name.trim()), future);
    }

    /// Link a part (id or number) into the folder shown; `revision` pins it, empty follows.
    pub fn link(&mut self, plm: &dyn Workspaces, part: &str, revision: &str) {
        let future = Self::map(plm.link(self.current_folder(), part.trim(), revision.trim()), Done::Entry);
        self.act(format!("linked {}", part.trim()), future);
    }

    /// A file the host was handed (dropped on the pane): a new file in the folder shown,
    /// or — when a file of that name is already there — its next version, the one before
    /// kept.
    pub fn add_file(&mut self, plm: &dyn Workspaces, name: &str, bytes: Vec<u8>) {
        let existing = self
            .entries
            .as_ref()
            .and_then(|list| list.iter().find(|e| e.kind == "file" && e.name.eq_ignore_ascii_case(name)))
            .map(|e| e.id.clone());
        match existing {
            Some(id) => {
                let future = Self::map(plm.replace_file(&id, name, bytes), Done::Entry);
                self.act(format!("{name}: a new version, the one before kept"), future);
            }
            None => {
                let future = Self::map(plm.add_file(self.current_folder(), name, bytes), Done::Entry);
                self.act(format!("added {name}"), future);
            }
        }
    }

    pub fn rename(&mut self, plm: &dyn Workspaces, id: &str, name: &str) {
        let future = Self::map(plm.update(id, json!({ "name": name.trim() })), Done::Entry);
        self.act(format!("renamed to {}", name.trim()), future);
    }

    /// Cut an entry, to paste it into another folder of the same workspace.
    pub fn cut(&mut self, id: &str) {
        self.cut = self.entry(id).cloned();
        if let Some(entry) = &self.cut {
            self.notice = Some(format!("{} cut — open a folder and paste it there", entry.name));
        }
    }

    /// Move the cut entry into the folder shown.
    pub fn paste(&mut self, plm: &dyn Workspaces) {
        let Some(entry) = self.cut.take() else { return };
        let future = Self::map(plm.update(&entry.id, json!({ "parent": self.current_folder() })), Done::Entry);
        self.act(format!("moved {}", entry.name), future);
    }

    /// Remove an entry: a link or a file, never a part (D9). A folder goes with everything in
    /// it — the browser asks first ([`Self::delete_armed`]).
    pub fn delete(&mut self, plm: &dyn Workspaces, id: &str) {
        let Some(entry) = self.entry(id).cloned() else { return };
        let future = Self::map(plm.delete(&entry.id, entry.is_folder()), |()| Done::Nothing);
        let what = match entry.kind.as_str() {
            "link" => format!("removed the link {} — the part is untouched", entry.name),
            _ => format!("deleted {}", entry.name),
        };
        self.selection_cleared();
        self.act(what, future);
    }

    /// Pin a link to `revision` (id or label), or let it follow the newest (empty).
    pub fn pin(&mut self, plm: &dyn Workspaces, id: &str, revision: &str) {
        let future = Self::map(plm.update(id, json!({ "revision": revision })), Done::Entry);
        let what = if revision.is_empty() { "the link follows the newest revision".to_string() } else { format!("pinned to {revision}") };
        self.act(what, future);
    }

    pub fn load_versions(&mut self, plm: &dyn Workspaces, id: &str) {
        self.versions_pending = Some((id.to_string(), Pending::new(plm.versions(id))));
    }

    /// Make version `n` current again — as a new version, so none is lost.
    pub fn restore(&mut self, plm: &dyn Workspaces, id: &str, version: u32) {
        let future = Self::map(plm.restore(id, version), Done::Entry);
        self.act(format!("restored version {version} as a new version"), future);
    }

    pub fn download(&mut self, plm: &dyn Workspaces, entry: &Entry, version: u32) {
        self.download_pending = Some((entry.name.clone(), Pending::new(plm.download(&entry.id, version))));
    }

    /// Promote a file to a real attachment; the server's attachment refusals apply (S8).
    pub fn promote(&mut self, plm: &dyn Workspaces, id: &str, to: &PromoteTo) {
        let future = Self::map(plm.promote(id, to), Done::Attachment);
        self.act("promoted", future);
    }

    /// Open by number: look the part up and fill Revision with its newest revision
    /// (drafts included). With `open`, open it once found — or the revision the user
    /// typed, if they typed one.
    pub fn look_up(&mut self, plm: &dyn Workspaces, open: bool) {
        self.problem = None;
        self.lookup_pending = Some((open, Pending::new(plm.lookup(&self.open_part))));
    }

    /// Open the revision in the Revision field of the part looked up.
    pub fn open_by_number(&mut self, plm: &dyn Workspaces, outcome: &mut WorkspaceOutcome) {
        let looked_up = self.open_lookup.as_ref().filter(|p| {
            let typed = self.open_part.trim();
            p.id == typed || p.number.eq_ignore_ascii_case(typed)
        });
        match looked_up {
            Some(part) => {
                let wanted = self.open_revision.trim();
                let revision = part
                    .revision_views
                    .iter()
                    .find(|r| r.id == wanted || r.label.eq_ignore_ascii_case(wanted))
                    .or(part.newest_revision.as_ref().filter(|_| wanted.is_empty()));
                match revision {
                    Some(r) if !r.document_key.is_empty() => {
                        outcome.open = Some(r.document_key.clone());
                        // Opened: the fields start empty next time, not as a picker of
                        // this part's revisions.
                        self.open_part.clear();
                        self.open_revision.clear();
                        self.open_lookup = None;
                    }
                    _ => self.problem = Some(format!("{} has no revision '{wanted}'", part.number)),
                }
            }
            None => self.look_up(plm, true),
        }
    }

    /// Drive every request: apply what the server answered.
    pub fn poll(&mut self, waker: &Waker, plm: &dyn Workspaces, outcome: &mut WorkspaceOutcome) {
        self.thumbnails.poll(waker);
        if let Some(answer) = self.list_pending.as_mut().and_then(|p| p.poll(waker)) {
            self.list_pending = None;
            match answer {
                Ok(entries) => {
                    if self.selected.as_deref().is_some_and(|id| !entries.iter().any(|e| e.id == id)) {
                        self.selection_cleared();
                    }
                    self.entries = Some(entries);
                }
                Err(problem) => {
                    // The rows stay up while a reload is in flight, but not past
                    // its refusal: a listing the server would not give (a
                    // workspace made private under a viewer) is not shown from
                    // memory either (plan S14; `workspaces_browsable` off).
                    if self.selected.is_some() { self.selection_cleared(); }
                    self.entries = Some(Vec::new());
                    self.problem = Some(problem);
                }
            }
        }
        if let Some(answer) = self.owners_pending.as_mut().and_then(|p| p.poll(waker)) {
            self.owners_pending = None;
            match answer {
                Ok(list) => self.workspaces = Some(list),
                Err(problem) => self.problem = Some(problem),
            }
        }
        let mut answered = Vec::new();
        self.action_pending.retain_mut(|(what, pending)| match pending.poll(waker) {
            Some(answer) => {
                answered.push((std::mem::take(what), answer));
                false
            }
            None => true,
        });
        for (what, answer) in answered {
            {
                match answer {
                    Ok(done) => {
                        self.notice = Some(what);
                        match done {
                            Done::Entry(entry) => {
                                if self.versions.as_ref().is_some_and(|(id, _)| *id == entry.id) {
                                    self.load_versions(plm, &entry.id);
                                }
                                self.selected = Some(entry.id);
                            }
                            Done::Attachment(a) => {
                                self.notice = Some(format!("promoted to an attachment: {} ({})", a.name, a.kind));
                                self.promote = None;
                            }
                            Done::Nothing => {}
                        }
                        self.reload_folder(plm);
                    }
                    Err(problem) => self.problem = Some(problem),
                }
            }
        }
        if let Some((open, pending)) = self.lookup_pending.as_mut() {
            let open = *open;
            if let Some(answer) = pending.poll(waker) {
                self.lookup_pending = None;
                match answer {
                    Ok(part) => {
                        let typed = self.open_revision.trim().to_string();
                        if typed.is_empty() || !part.revision_views.iter().any(|r| r.label.eq_ignore_ascii_case(&typed) || r.id == typed) {
                            self.open_revision = part.newest_revision.as_ref().map(|r| r.label.clone()).unwrap_or_default();
                        }
                        self.open_part = part.number.clone();
                        self.open_lookup = Some(part);
                        if open {
                            self.open_by_number(plm, outcome);
                        }
                    }
                    Err(problem) => {
                        self.open_lookup = None;
                        self.problem = Some(problem);
                    }
                }
            }
        }
        if let Some((name, pending)) = self.download_pending.as_mut() {
            if let Some(answer) = pending.poll(waker) {
                let name = std::mem::take(name);
                self.download_pending = None;
                match answer {
                    Ok(bytes) => outcome.downloaded = Some((name, bytes)),
                    Err(problem) => self.problem = Some(problem),
                }
            }
        }
        if let Some((id, pending)) = self.versions_pending.as_mut() {
            if let Some(answer) = pending.poll(waker) {
                let id = std::mem::take(id);
                self.versions_pending = None;
                match answer {
                    Ok(list) => self.versions = Some((id, list)),
                    Err(problem) => self.problem = Some(problem),
                }
            }
        }
    }
}

// --- drawing ----------------------------------------------------------------------

/// A waker that repaints: an answer is drawn the frame it arrives.
fn repaint_waker(ctx: &egui::Context) -> Waker {
    struct Repaint(egui::Context);
    impl std::task::Wake for Repaint {
        fn wake(self: std::sync::Arc<Self>) {
            self.0.request_repaint();
        }
    }
    Waker::from(std::sync::Arc::new(Repaint(ctx.clone())))
}

/// A row's kind, in plain text: the app ships no symbol font, and an emoji folder draws
/// as an empty box.
fn icon(entry: &Entry) -> &'static str {
    match entry.kind.as_str() {
        "folder" => "[folder]",
        "link" => "[link]",
        _ => "[file]",
    }
}

impl WorkspaceBrowser {
    fn hit(&mut self, prefix: &str, key: &str, response: &egui::Response) {
        self.hits.insert(format!("{prefix}ws:{key}"), response.rect);
    }

    /// Draw the browser, poll its requests, and say what the host must do. `prefix` is
    /// the host's hit-key prefix (`plm/workspace:` in the PLM pane, `open:` in the Open
    /// modal).
    pub fn show(&mut self, ui: &mut egui::Ui, plm: &dyn Workspaces, prefix: &str) -> WorkspaceOutcome {
        if let Some(client) = plm.session() {
            if self.session.as_ref().is_some_and(|old| !std::rc::Rc::ptr_eq(old, &client)) {
                // The Open modal reuses this browser across connections too.
                let can_pick_files = self.can_pick_files;
                let browse_only = self.browse_only;
                *self = Self::new();
                self.can_pick_files = can_pick_files;
                self.browse_only = browse_only;
            }
            self.session = Some(client);
        }
        let mut outcome = WorkspaceOutcome::default();
        let ctx = ui.ctx().clone();
        let waker = repaint_waker(&ctx);
        if self.entries.is_none() && !self.busy() {
            self.reload(plm);
        }
        self.poll(&waker, plm, &mut outcome);
        self.hits.clear();

        self.owner_row(ui, plm, prefix);
        self.open_row(ui, plm, prefix, &mut outcome);
        ui.separator();
        self.crumbs(ui, plm, prefix);
        self.rows(ui, plm, prefix, &mut outcome);
        self.selection_verbs(ui, plm, prefix);
        if !self.read_only() {
            ui.separator();
            self.make_row(ui, plm, prefix);
        }
        outcome.pick_file = std::mem::take(&mut self.pick_asked);
        if let Some(problem) = &self.problem {
            ui.colored_label(ui.visuals().error_fg_color, problem);
        } else if let Some(notice) = &self.notice {
            ui.label(egui::RichText::new(notice).weak());
        }
        // Anything the frame started is answered on a later one: poll again now so a
        // request that is already answered (the in-process router) lands this frame.
        self.poll(&waker, plm, &mut outcome);
        outcome.pins = self.pins_changed.take();
        outcome
    }

    fn owner_row(&mut self, ui: &mut egui::Ui, plm: &dyn Workspaces, prefix: &str) {
        let rows: Vec<WorkspaceRow> = self.workspaces.as_ref().map(|w| w.workspaces.clone()).unwrap_or_default();
        let mut chosen: Option<(String, String)> = None;
        ui.horizontal(|ui| {
            let combo = egui::ComboBox::from_id_salt((prefix, "ws-owner")).selected_text(&self.owner_label).show_ui(ui, |ui| {
                for row in &rows {
                    let label = if row.mine { "My workspace".to_string() } else { format!("{}'s workspace", row.display_name.trim().is_empty().then_some(&row.username).unwrap_or(&row.display_name)) };
                    let owner = if row.mine { String::new() } else { row.user_id.clone() };
                    if ui.selectable_label(self.owner == owner, &label).clicked() {
                        chosen = Some((owner, label));
                    }
                }
            });
            self.hit(prefix, "owner", &combo.response);
            if self.read_only() {
                ui.label(egui::RichText::new("read-only").weak());
            }
            if self.workspaces.as_ref().is_some_and(|w| !w.browsable) {
                ui.label(egui::RichText::new("other workspaces are private on this server").weak());
            }
            let reload = ui.small_button("\u{21BB}").on_hover_text("Reload");
            self.hit(prefix, "reload", &reload);
            if reload.clicked() {
                self.reload(plm);
            }
        });
        if let Some((owner, label)) = chosen {
            self.set_owner(plm, &owner, &label);
        }
    }

    /// The Revision field's width, the SAME for the text box shown before a
    /// lookup answers and the combo shown after. The widgets are drawn in
    /// one row with the Open button to their right, and the lookup answers
    /// asynchronously — a click already pressed on Open while the field was
    /// the 60 pt text box released on a combo 100 pt wide (egui's default)
    /// that had pushed the button 40 pt to the right; egui saw a press on one
    /// widget and a release on another, and nothing opened (2026-10-03, the
    /// open-by-number scripts). A width shared by both widgets keeps the
    /// button where the pointer is.
    const REVISION_FIELD_WIDTH: f32 = 80.0;

    fn open_row(&mut self, ui: &mut egui::Ui, plm: &dyn Workspaces, prefix: &str, outcome: &mut WorkspaceOutcome) {
        ui.horizontal(|ui| {
            ui.label("Part");
            let part = ui.add(egui::TextEdit::singleline(&mut self.open_part).desired_width(110.0).hint_text("number or id"));
            self.hit(prefix, "open-part", &part);
            if part.lost_focus() && !self.open_part.trim().is_empty() {
                // Typing a part fills Revision with its newest (D13).
                self.look_up(plm, false);
            }
            ui.label("Revision");
            let labels: Vec<(String, String)> = self
                .open_lookup
                .as_ref()
                .map(|p| p.revision_views.iter().map(|r| (r.label.clone(), format!("{} ({})", r.label, r.lifecycle))).collect())
                .unwrap_or_default();
            let revision = if labels.is_empty() {
                ui.add_sized(
                    [Self::REVISION_FIELD_WIDTH, ui.spacing().interact_size.y],
                    egui::TextEdit::singleline(&mut self.open_revision).hint_text("newest"),
                )
            } else {
                let text = self.open_revision.clone();
                egui::ComboBox::from_id_salt((prefix, "ws-open-revision"))
                    .width(Self::REVISION_FIELD_WIDTH)
                    .selected_text(text)
                    .show_ui(ui, |ui| {
                        for (label, shown) in labels.iter().rev() {
                            ui.selectable_value(&mut self.open_revision, label.clone(), shown);
                        }
                    })
                    .response
            };
            self.hit(prefix, "open-revision", &revision);
            let go = ui.add_enabled(!self.open_part.trim().is_empty(), egui::Button::new("Open"));
            self.hit(prefix, "open-go", &go);
            if go.clicked() {
                self.open_by_number(plm, outcome);
            }
        });
    }

    fn crumbs(&mut self, ui: &mut egui::Ui, plm: &dyn Workspaces, prefix: &str) {
        let mut up_to: Option<usize> = None;
        let mut paste = false;
        ui.horizontal_wrapped(|ui| {
            let top = ui.link(&self.owner_label);
            self.hit(prefix, "crumb:0", &top);
            if top.clicked() {
                up_to = Some(0);
            }
            for (depth, (_, name)) in self.path.clone().iter().enumerate() {
                ui.label("/");
                let crumb = ui.link(name);
                self.hit(prefix, &format!("crumb:{}", depth + 1), &crumb);
                if crumb.clicked() {
                    up_to = Some(depth + 1);
                }
            }
            if let Some(cut) = self.cut.clone().filter(|_| !self.read_only()) {
                let button = ui.small_button(format!("Paste {} here", cut.name));
                self.hit(prefix, "paste", &button);
                paste = button.clicked();
            }
            if !self.read_only() && !self.path.is_empty() {
                let here = self.current_folder().to_string();
                let pinned = self.pins.as_ref().is_some_and(|pins| pins.iter().any(|p| p.last().map(|(id, _)| id.as_str()) == Some(here.as_str())));
                let button = ui.small_button(if pinned { "Unpin folder" } else { "Pin folder" });
                self.hit(prefix, "pin-folder", &button);
                if button.clicked() {
                    self.pins_changed = self.toggle_pin();
                }
            }
        });
        let mut go_to: Option<Pin> = None;
        if let Some(pins) = self.pins.clone().filter(|pins| !pins.is_empty()) {
            ui.horizontal_wrapped(|ui| {
                ui.label(egui::RichText::new("Pinned:").weak());
                for (index, pin) in pins.iter().enumerate() {
                    let name = pin.last().map(|(_, name)| name.as_str()).unwrap_or("");
                    let button = ui.small_button(name).on_hover_text(pin.iter().map(|(_, n)| n.as_str()).collect::<Vec<_>>().join(" / "));
                    self.hit(prefix, &format!("pinned:{index}"), &button);
                    if button.clicked() {
                        go_to = Some(pin.clone());
                    }
                }
            });
        }
        if let Some(pin) = go_to {
            self.go_to_pin(plm, &pin);
        }
        if let Some(depth) = up_to {
            self.up_to(plm, depth);
        }
        if paste {
            self.paste(plm);
        }
    }

    fn rows(&mut self, ui: &mut egui::Ui, plm: &dyn Workspaces, prefix: &str, outcome: &mut WorkspaceOutcome) {
        let Some(entries) = self.entries.clone() else {
            ui.label(egui::RichText::new("Reading the workspace…").weak());
            return;
        };
        if entries.is_empty() {
            let empty = if self.read_only() { "Empty." } else { "Empty. Make a folder, link a part, or drop a file here." };
            ui.label(egui::RichText::new(empty).weak());
        }
        let mut activated: Option<Entry> = None;
        egui::ScrollArea::vertical().id_salt((prefix, "ws-rows")).max_height(260.0).show(ui, |ui| {
            for entry in &entries {
                ui.push_id(&entry.id, |ui| {
                let selected = self.selected.as_deref() == Some(entry.id.as_str());
                let text = format!("{} {}   {}", icon(entry), entry.name, entry.detail());
                let row = ui.horizontal(|ui| {
                    let url = entry.link.as_ref().map_or("", |link| link.thumbnail_url.as_str());
                    let preview = self.thumbnails.show(ui, url, |url| plm.thumbnail(url));
                    ui.add_sized(
                        [ui.available_width(), crate::plm::thumbnail_view::SIDE],
                        egui::Button::selectable(selected, text).wrap_mode(egui::TextWrapMode::Truncate),
                    ).union(preview)
                }).inner;
                self.hit(prefix, &format!("entry:{}", entry.name), &row);
                // Click selects, double click opens (every tree in this app).
                if row.clicked() {
                    self.select_named(&entry.name);
                }
                if row.double_clicked() {
                    activated = Some(entry.clone());
                }
                });
            }
        });
        if let Some(entry) = activated {
            self.activate(plm, &entry, outcome);
        }
    }

    fn selection_verbs(&mut self, ui: &mut egui::Ui, plm: &dyn Workspaces, prefix: &str) {
        let Some(entry) = self.selection().cloned() else { return };
        let writable = !self.read_only();
        ui.separator();
        ui.label(egui::RichText::new(format!("{} {}", icon(&entry), entry.name)).strong());
        ui.horizontal_wrapped(|ui| {
            if writable {
                let rename = ui.small_button("Rename");
                self.hit(prefix, "sel:rename", &rename);
                if rename.clicked() {
                    self.rename = Some(entry.name.clone());
                }
                let cut = ui.small_button("Move…");
                self.hit(prefix, "sel:cut", &cut);
                if cut.clicked() {
                    self.cut(&entry.id);
                }
                let label = if self.delete_armed {
                    "Really delete?"
                } else if entry.kind == "link" {
                    "Remove link"
                } else {
                    "Delete"
                };
                let delete = ui.small_button(label).on_hover_text(match entry.kind.as_str() {
                    "link" => "Removes the link only: the part and its revisions are untouched.",
                    "folder" => "Deletes the folder and everything in it (links, files). No part is touched.",
                    _ => "Deletes the file and every version of it.",
                });
                self.hit(prefix, if self.delete_armed { "sel:delete-confirm" } else { "sel:delete" }, &delete);
                if delete.clicked() {
                    if self.delete_armed || entry.kind == "link" {
                        self.delete(plm, &entry.id);
                    } else {
                        self.delete_armed = true;
                    }
                }
            }
            if let Some(link) = entry.link.clone() {
                if writable && !link.part_missing {
                    if link.pinned {
                        let follow = ui.small_button("Follow newest");
                        self.hit(prefix, "sel:follow", &follow);
                        if follow.clicked() {
                            self.pin(plm, &entry.id, "");
                        }
                    } else if !link.revision_label.is_empty() {
                        let pin = ui.small_button(format!("Pin to {}", link.revision_label));
                        self.hit(prefix, "sel:pin", &pin);
                        if pin.clicked() {
                            self.pin(plm, &entry.id, &link.revision_id);
                        }
                    }
                }
            }
            if entry.kind == "file" {
                let download = ui.small_button("Download");
                self.hit(prefix, "sel:download", &download);
                if download.clicked() {
                    self.download(plm, &entry, 0);
                }
                let versions = ui.small_button("Versions");
                self.hit(prefix, "sel:versions", &versions);
                if versions.clicked() {
                    self.load_versions(plm, &entry.id);
                }
                if writable {
                    let promote = ui.small_button("Promote to attachment…");
                    self.hit(prefix, "sel:promote", &promote);
                    if promote.clicked() {
                        self.promote = Some(PromoteTo { kind: "other".into(), ..PromoteTo::default() });
                    }
                }
            }
        });
        if let Some(mut name) = self.rename.take() {
            let mut keep = true;
            ui.horizontal(|ui| {
                let field = ui.text_edit_singleline(&mut name);
                self.hit(prefix, "sel:rename-name", &field);
                let go = ui.small_button("Rename");
                self.hit(prefix, "sel:rename-go", &go);
                if go.clicked() {
                    self.rename(plm, &entry.id, &name);
                    keep = false;
                }
            });
            if keep {
                self.rename = Some(name);
            }
        }
        if let Some((id, versions)) = self.versions.clone().filter(|(id, _)| *id == entry.id) {
            for v in versions.iter().rev() {
                ui.horizontal(|ui| {
                    let restored = v.restored_from.map(|n| format!(", restored from v{n}")).unwrap_or_default();
                    ui.label(format!(
                        "v{}{} · {} · {}{restored}",
                        v.version,
                        if v.current { " (current)" } else { "" },
                        human_size(v.size),
                        v.uploaded_by_name
                    ));
                    let download = ui.small_button("Download");
                    self.hit(prefix, &format!("version:{}:download", v.version), &download);
                    if download.clicked() {
                        self.download(plm, &entry, v.version);
                    }
                    if writable && !v.current {
                        let restore = ui.small_button("Restore");
                        self.hit(prefix, &format!("version:{}:restore", v.version), &restore);
                        if restore.clicked() {
                            self.restore(plm, &id, v.version);
                        }
                    }
                });
            }
        }
        if let Some(mut to) = self.promote.take() {
            let mut keep = true;
            ui.horizontal_wrapped(|ui| {
                ui.label("Part");
                let part = ui.add(egui::TextEdit::singleline(&mut to.part).desired_width(100.0).hint_text("number"));
                self.hit(prefix, "sel:promote-part", &part);
                ui.label("Revision");
                let revision = ui.add(egui::TextEdit::singleline(&mut to.revision).desired_width(50.0).hint_text("part"));
                self.hit(prefix, "sel:promote-revision", &revision);
                egui::ComboBox::from_id_salt((prefix, "ws-promote-kind")).selected_text(&to.kind).show_ui(ui, |ui| {
                    for kind in ["drawing", "datasheet", "spec", "image", "other"] {
                        ui.selectable_value(&mut to.kind, kind.to_string(), kind);
                    }
                });
                let go = ui.add_enabled(!to.part.trim().is_empty(), egui::Button::new("Promote"));
                self.hit(prefix, "sel:promote-go", &go);
                if go.clicked() {
                    self.promote(plm, &entry.id, &to);
                }
                if ui.small_button("Cancel").clicked() {
                    keep = false;
                }
            });
            if keep {
                self.promote = Some(to);
            }
        }
    }

    fn make_row(&mut self, ui: &mut egui::Ui, plm: &dyn Workspaces, prefix: &str) {
        ui.horizontal(|ui| {
            let name = ui.add(egui::TextEdit::singleline(&mut self.new_folder_name).desired_width(120.0).hint_text("folder name"));
            self.hit(prefix, "new-folder-name", &name);
            let go = ui.add_enabled(!self.new_folder_name.trim().is_empty(), egui::Button::new("New folder"));
            self.hit(prefix, "new-folder", &go);
            if go.clicked() {
                let name = std::mem::take(&mut self.new_folder_name);
                self.new_folder(plm, &name);
            }
        });
        ui.horizontal(|ui| {
            let part = ui.add(egui::TextEdit::singleline(&mut self.link_part).desired_width(100.0).hint_text("part number"));
            self.hit(prefix, "link-part", &part);
            let revision = ui.add(egui::TextEdit::singleline(&mut self.link_revision).desired_width(50.0).hint_text("newest"));
            self.hit(prefix, "link-revision", &revision);
            let go = ui.add_enabled(!self.link_part.trim().is_empty(), egui::Button::new("Link here"));
            self.hit(prefix, "link-go", &go);
            if go.clicked() {
                let (part, revision) = (std::mem::take(&mut self.link_part), std::mem::take(&mut self.link_revision));
                self.link(plm, &part, &revision);
            }
        });
        ui.horizontal_wrapped(|ui| {
            if self.can_pick_files {
                let add = ui.button("Add file\u{2026}").on_hover_text("Choose a file on this machine to add to this folder");
                self.hit(prefix, "add-file", &add);
                self.pick_asked |= add.clicked();
            }
            ui.label(egui::RichText::new("Drop a file on this pane to add it here (a file of the same name gets a new version).").weak().small());
        });
    }
}


