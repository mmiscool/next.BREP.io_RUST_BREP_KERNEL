//! Workspaces: each user's folders of links and files (plm-cad-integration
//! D12, prerequisite P6). In PLM mode the CAD app's file explorer becomes a
//! browser of these (S14).
//!
//! # What an entry is
//!
//! A [`WorkspaceEntry`] is a folder, a link or a file, in one user's tree:
//!
//! - A **folder** holds entries. Names are unique within a folder, ignoring
//!   ASCII case, as on the file systems the explorer came from.
//! - A **link** points at a part by id: it follows the part's newest
//!   revision, drafts included (D13), or is pinned to one revision. It is
//!   never a copy, and deleting it never touches the part. Because it holds
//!   the id, a part renamed or renumbered is still found. Parts are never
//!   deleted, but a draft revision can be; a link pinned to one then says
//!   `revision_missing` rather than failing.
//! - A **file** is any bytes a user keeps with their work (a customer's
//!   photo, notes). Its versions are blobs in the attachment store, under the
//!   same size limit. **Every replace keeps the version before it**, a
//!   restore adds the old bytes as a new version (so nothing is ever lost by
//!   restoring either), and a file can be **promoted** to a real attachment
//!   on a part or revision, which obeys every attachment rule.
//!
//! # Who may do what
//!
//! Only the owner changes a workspace. Reading one is for its owner, any
//! administrator, and everyone else only while the administrator setting
//! [`crate::model::Settings::workspaces_browsable`] is on (the default).
//! Any signed-in user has a workspace, viewers included: it changes no part.
//! Promoting a file to an attachment needs the author group, as attaching
//! does.

use std::collections::BTreeSet;

use serde::Serialize;
use serde_json::Value;

use crate::attach::{self, Describe, Staged};
use crate::db::{now, Db, State};
use crate::model::{Attachment, FileVersion, Origin, Part, User, WorkspaceEntry, WorkspaceKind, WorkspaceLink};
use crate::Error;

const MAX_NAME: usize = 200;

/// Folders nest at most this deep, so a walk up the tree always ends.
pub const MAX_DEPTH: usize = 64;

/// An entry's name, tidied: the rules of an attachment's file name (no path
/// separators, control characters or quotes; not `.` or `..`; at most 200
/// characters).
pub fn clean_name(raw: &str) -> Result<String, Error> {
    let trimmed = raw.trim();
    if trimmed.contains(['/', '\\']) {
        return Err(Error::bad_request(format!("'{trimmed}' — a name cannot contain / or \\")));
    }
    let name: String = trimmed.chars().filter(|c| !c.is_control() && *c != '"').collect();
    let name = name.trim().to_string();
    if name.is_empty() || name == "." || name == ".." {
        return Err(Error::bad_request("a workspace entry needs a name"));
    }
    if name.chars().count() > MAX_NAME {
        return Err(Error::bad_request(format!("a name is at most {MAX_NAME} characters")));
    }
    Ok(name)
}

/// The user a request names by id or username; empty is the caller.
pub fn owner_of(state: &State, viewer: &User, key: &str) -> Result<User, Error> {
    let key = key.trim();
    if key.is_empty() {
        return Ok(viewer.clone());
    }
    state
        .user(key)
        .or_else(|| state.user_by_name(key))
        .cloned()
        .ok_or_else(|| Error::not_found("user"))
}

/// Refuse `viewer` reading `owner`'s workspace, unless it is theirs, they
/// administer, or workspaces are browsable.
pub fn may_read(state: &State, viewer: &User, owner: &User) -> Result<(), Error> {
    if viewer.id == owner.id || viewer.is_admin() || state.settings.workspaces_browsable {
        Ok(())
    } else {
        Err(Error::forbidden(format!("{}'s workspace is private", owner.username)))
    }
}

fn entry<'a>(state: &'a State, id: &str) -> Result<&'a WorkspaceEntry, Error> {
    state.workspace.get(id).ok_or_else(|| Error::not_found("workspace entry"))
}

fn entry_mut<'a>(state: &'a mut State, id: &str) -> Result<&'a mut WorkspaceEntry, Error> {
    state.workspace.get_mut(id).ok_or_else(|| Error::not_found("workspace entry"))
}

/// The entry `id`, which `user` must own.
fn owned<'a>(state: &'a State, user: &User, id: &str) -> Result<&'a WorkspaceEntry, Error> {
    let found = entry(state, id)?;
    if found.owner != user.id {
        return Err(Error::forbidden("only its owner changes a workspace"));
    }
    Ok(found)
}

/// Check `parent` is empty (the top) or a folder of `owner`'s.
fn check_folder(state: &State, owner: &str, parent: &str) -> Result<(), Error> {
    if parent.is_empty() {
        return Ok(());
    }
    let folder = entry(state, parent).map_err(|_| Error::not_found("folder"))?;
    if folder.owner != owner {
        return Err(Error::forbidden("that folder is in another user's workspace"));
    }
    if folder.kind != WorkspaceKind::Folder {
        return Err(Error::bad_request(format!("'{}' is a {}, not a folder", folder.name, folder.kind.as_str())));
    }
    Ok(())
}

/// Refuse a name another entry of the same folder already has.
fn check_unique(state: &State, owner: &str, parent: &str, name: &str, except: &str) -> Result<(), Error> {
    let clash = state.workspace.iter().any(|e| {
        e.owner == owner && e.parent == parent && e.id != except && e.name.eq_ignore_ascii_case(name)
    });
    if clash {
        return Err(Error::conflict(format!("this folder already holds '{name}'")));
    }
    Ok(())
}

/// The folders from `id` up to the top: `id` first. Bounded by
/// [`MAX_DEPTH`] + 1 steps, so a damaged tree cannot loop.
fn ancestry(state: &State, id: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut at = id.to_string();
    while !at.is_empty() && out.len() <= MAX_DEPTH {
        out.push(at.clone());
        at = state.workspace.iter().find(|e| e.id == at).map(|e| e.parent.clone()).unwrap_or_default();
    }
    out
}

/// `id` and everything beneath it.
fn subtree(state: &State, id: &str) -> BTreeSet<String> {
    let mut out = BTreeSet::from([id.to_string()]);
    loop {
        let more: Vec<String> = state
            .workspace
            .iter()
            .filter(|e| out.contains(&e.parent) && !out.contains(&e.id))
            .map(|e| e.id.clone())
            .collect();
        if more.is_empty() {
            return out;
        }
        out.extend(more);
    }
}

/// How a link resolves right now.
#[derive(Debug, Clone, Serialize)]
pub struct LinkView {
    pub part_id: String,
    /// Whether the link is pinned to one revision; false follows the newest.
    pub pinned: bool,
    /// The part's number and name as they are now — a rename shows here.
    pub number: String,
    pub name: String,
    pub document_class: String,
    /// The revision it opens: the pinned one, or the newest (drafts
    /// included). Empty when there is none.
    pub revision_id: String,
    pub revision_label: String,
    pub lifecycle: String,
    /// `part/<part>/rev/<revision>`, the store key the CAD app opens.
    pub document_key: String,
    /// The part is gone. Parts are never deleted, so this is a damaged store.
    pub part_missing: bool,
    /// The link is pinned to a revision that was deleted (a draft).
    pub revision_missing: bool,
    /// Where to fetch its thumbnail: the pinned revision's, or the part's
    /// ([`crate::thumbnail`]); empty for none.
    pub thumbnail_url: String,
}

/// The current version of a file, in brief.
#[derive(Debug, Clone, Serialize)]
pub struct FileView {
    pub version: u32,
    pub versions: usize,
    pub size: u64,
    pub media_type: String,
    pub sha256: String,
    pub uploaded_at: u64,
    pub uploaded_by_name: String,
}

/// An entry as a client sees it: the record, its owner's name, and what a
/// link resolves to or a file holds.
#[derive(Debug, Clone, Serialize)]
pub struct EntryView {
    pub id: String,
    pub owner: String,
    pub owner_name: String,
    pub parent: String,
    pub name: String,
    pub kind: WorkspaceKind,
    pub created_at: u64,
    pub modified_at: u64,
    /// For a folder: how many entries are directly in it.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub children: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub link: Option<LinkView>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub file: Option<FileView>,
}

fn user_name(state: &State, id: &str) -> String {
    state.user(id).map(|u| u.username.clone()).unwrap_or_else(|| "(removed user)".into())
}

/// Resolve a link against the store as it is.
pub fn resolve(state: &State, link: &WorkspaceLink) -> LinkView {
    let pinned = !link.revision_id.is_empty();
    let Some(part) = state.part(&link.part_id) else {
        return LinkView {
            part_id: link.part_id.clone(),
            pinned,
            number: String::new(),
            name: String::new(),
            document_class: String::new(),
            revision_id: link.revision_id.clone(),
            revision_label: String::new(),
            lifecycle: String::new(),
            document_key: String::new(),
            part_missing: true,
            revision_missing: pinned,
            thumbnail_url: String::new(),
        };
    };
    let revision = if pinned { part.revision(&link.revision_id) } else { part.latest() };
    let class = serde_json::to_value(part.document_class).ok().and_then(|v| v.as_str().map(str::to_string));
    LinkView {
        part_id: part.id.clone(),
        pinned,
        number: part.number.clone(),
        name: part.name.clone(),
        document_class: class.unwrap_or_default(),
        revision_id: revision.map(|r| r.id.clone()).unwrap_or_else(|| link.revision_id.clone()),
        revision_label: revision.map(|r| r.label.clone()).unwrap_or_default(),
        lifecycle: revision.map(|r| r.lifecycle.as_str().to_string()).unwrap_or_default(),
        document_key: revision.map(|r| r.document_key(&part.id)).unwrap_or_default(),
        part_missing: false,
        revision_missing: pinned && revision.is_none(),
        thumbnail_url: match (pinned, revision) {
            (true, Some(r)) => crate::thumbnail::revision_url(&part.id, r),
            (true, None) => String::new(),
            (false, _) => crate::thumbnail::part_url(part),
        },
    }
}

/// An entry as [`EntryView`].
pub fn view(state: &State, e: &WorkspaceEntry) -> EntryView {
    EntryView {
        id: e.id.clone(),
        owner: e.owner.clone(),
        owner_name: user_name(state, &e.owner),
        parent: e.parent.clone(),
        name: e.name.clone(),
        kind: e.kind,
        created_at: e.created_at,
        modified_at: e.modified_at,
        children: (e.kind == WorkspaceKind::Folder)
            .then(|| state.workspace.iter().filter(|c| c.parent == e.id).count()),
        link: e.link.as_ref().map(|l| resolve(state, l)),
        file: e.versions.last().map(|v| FileView {
            version: v.version,
            versions: e.versions.len(),
            size: v.size,
            media_type: v.media_type.clone(),
            sha256: v.sha256.clone(),
            uploaded_at: v.uploaded_at,
            uploaded_by_name: user_name(state, &v.uploaded_by),
        }),
    }
}

/// A workspace another user may open, as the list of them shows it.
#[derive(Debug, Clone, Serialize)]
pub struct WorkspaceRow {
    pub user_id: String,
    pub username: String,
    pub display_name: String,
    pub mine: bool,
    pub entries: usize,
}

/// A file version as the version list shows it.
#[derive(Debug, Clone, Serialize)]
pub struct VersionView {
    #[serde(flatten)]
    pub version: FileVersion,
    pub uploaded_by_name: String,
    pub current: bool,
}

/// What a promote asks for.
#[derive(Debug, Clone, Default)]
pub struct Promote {
    /// A part id or number.
    pub part: String,
    /// A revision id or label; empty attaches to the part.
    pub revision: String,
    /// Which version; 0 is the current one.
    pub version: u32,
    /// The attachment's name; empty keeps the entry's.
    pub name: String,
    pub kind: String,
    pub note: String,
}

fn blank(owner: &User, parent: &str, name: String, kind: WorkspaceKind) -> WorkspaceEntry {
    let at = now();
    WorkspaceEntry {
        id: crate::auth::new_id(),
        owner: owner.id.clone(),
        parent: parent.to_string(),
        name,
        kind,
        link: None,
        versions: Vec::new(),
        created_at: at,
        modified_at: at,
    }
}

fn depth_ok(state: &State, parent: &str) -> Result<(), Error> {
    if ancestry(state, parent).len() >= MAX_DEPTH {
        return Err(Error::bad_request(format!("folders nest at most {MAX_DEPTH} deep")));
    }
    Ok(())
}

/// The revision a link is pinned to, by id or label; empty follows.
fn pin_of(state: &State, part_id: &str, revision_key: &str) -> Result<String, Error> {
    let key = revision_key.trim();
    if key.is_empty() {
        return Ok(String::new());
    }
    let part = state.part(part_id).ok_or_else(|| Error::not_found("part"))?;
    crate::bom::find_revision(part, key).map(|r| r.id.clone()).ok_or_else(|| Error::not_found("revision"))
}

/// **The one rule for linking a new part into its creator's workspace**
/// (operator, 2026-09-25: "when a user creates a part it should be
/// automatically added to their workspace"). Answers the folder to link into
/// (empty: the top), or `None` for no link.
///
/// A person making a part one at a time gets a link that follows the newest
/// revision: `POST /api/parts` (the web page's New part, the CAD app's New
/// part and Save As → a new part) and a template spin-out. `folder` is the
/// request's `workspace_folder`.
///
/// Bulk machine paths never link, since they make hundreds of parts:
/// - **Generate** of a family's members calls `create_part_with` itself and
///   never asks this rule;
/// - the **adoption importer** (S13) sends `bulk: true`;
/// - a **library import** (KiCad, S13) sends `origin: "imported"` — and
///   `bulk` too — so an importer that predates `bulk` links nothing either;
/// - the **bake worker** creates no parts.
pub fn link_on_create(origin: Origin, bulk: bool, folder: &str) -> Option<String> {
    if bulk || origin == Origin::Imported {
        return None;
    }
    Some(folder.trim().to_string())
}

/// Link the just-created `part` into `owner`'s folder `parent`, following its
/// newest revision, inside the create's own [`Db::mutate`] (so a refusal here
/// refuses the part too, and the link is audited with it). A folder that
/// already links the part following changes nothing. The name is the part's
/// number, or `<number> (2)`, `(3)`, … when another entry has taken it: a
/// clash of names never refuses a new part.
pub(crate) fn link_new_part(state: &mut State, owner: &str, parent: &str, part: &Part) -> Result<(), Error> {
    check_folder(state, owner, parent)?;
    let already = state.workspace.iter().any(|e| {
        e.owner == owner
            && e.parent == parent
            && e.link.as_ref().is_some_and(|l| l.part_id == part.id && l.revision_id.is_empty())
    });
    if already {
        return Ok(());
    }
    let base = clean_name(&part.number)?;
    let taken = |name: &str| {
        state.workspace.iter().any(|e| e.owner == owner && e.parent == parent && e.name.eq_ignore_ascii_case(name))
    };
    let mut name = base.clone();
    let mut n = 2;
    while taken(&name) {
        name = format!("{base} ({n})");
        n += 1;
    }
    let at = now();
    state.workspace.push(WorkspaceEntry {
        id: crate::auth::new_id(),
        owner: owner.to_string(),
        parent: parent.to_string(),
        name,
        kind: WorkspaceKind::Link,
        link: Some(WorkspaceLink { part_id: part.id.clone(), revision_id: String::new() }),
        versions: Vec::new(),
        created_at: at,
        modified_at: at,
    });
    Ok(())
}

impl Db {
    /// The workspaces `viewer` may open: their own first, then every other
    /// active user's while workspaces are browsable (or to an admin).
    pub fn workspaces(&self, viewer: &User) -> Vec<WorkspaceRow> {
        self.read(|state| {
            let others = viewer.is_admin() || state.settings.workspaces_browsable;
            let mut rows: Vec<WorkspaceRow> = state
                .users
                .iter()
                .filter(|u| u.id == viewer.id || (others && u.active))
                .map(|u| WorkspaceRow {
                    user_id: u.id.clone(),
                    username: u.username.clone(),
                    display_name: u.display_name.clone(),
                    mine: u.id == viewer.id,
                    entries: state.workspace.iter().filter(|e| e.owner == u.id).count(),
                })
                .collect();
            rows.sort_by_key(|r| (!r.mine, r.username.to_ascii_lowercase()));
            rows
        })
    }

    /// The entries directly in one folder of `owner`'s workspace (the top
    /// when `parent` is empty): folders first, then by name.
    pub fn workspace_folder(&self, viewer: &User, owner: &str, parent: &str) -> Result<Vec<EntryView>, Error> {
        self.read(|state| {
            let owner = owner_of(state, viewer, owner)?;
            may_read(state, viewer, &owner)?;
            let parent = parent.trim();
            if !parent.is_empty() {
                let folder = entry(state, parent).map_err(|_| Error::not_found("folder"))?;
                if folder.owner != owner.id || folder.kind != WorkspaceKind::Folder {
                    return Err(Error::not_found("folder"));
                }
            }
            let mut out: Vec<EntryView> = state
                .workspace
                .iter()
                .filter(|e| e.owner == owner.id && e.parent == parent)
                .map(|e| view(state, e))
                .collect();
            out.sort_by_key(|e| (e.kind != WorkspaceKind::Folder, e.name.to_ascii_lowercase()));
            Ok(out)
        })
    }

    /// One entry, if `viewer` may read its workspace.
    pub fn workspace_entry(&self, viewer: &User, id: &str) -> Result<EntryView, Error> {
        self.read(|state| {
            let e = entry(state, id)?;
            let owner = state.user(&e.owner).cloned().ok_or_else(|| Error::not_found("user"))?;
            may_read(state, viewer, &owner)?;
            Ok(view(state, e))
        })
    }

    /// A file's versions, oldest first.
    pub fn workspace_versions(&self, viewer: &User, id: &str) -> Result<Vec<VersionView>, Error> {
        self.read(|state| {
            let e = entry(state, id)?;
            let owner = state.user(&e.owner).cloned().ok_or_else(|| Error::not_found("user"))?;
            may_read(state, viewer, &owner)?;
            if e.kind != WorkspaceKind::File {
                return Err(Error::bad_request(format!("'{}' is a {}, not a file", e.name, e.kind.as_str())));
            }
            let last = e.versions.last().map(|v| v.version);
            Ok(e.versions
                .iter()
                .map(|v| VersionView {
                    version: v.clone(),
                    uploaded_by_name: user_name(state, &v.uploaded_by),
                    current: Some(v.version) == last,
                })
                .collect())
        })
    }

    /// One version of a file (0: the current one), with the entry's name —
    /// what a download needs.
    pub fn workspace_file(&self, viewer: &User, id: &str, version: u32) -> Result<(String, FileVersion), Error> {
        self.read(|state| {
            let e = entry(state, id)?;
            let owner = state.user(&e.owner).cloned().ok_or_else(|| Error::not_found("user"))?;
            may_read(state, viewer, &owner)?;
            let found = if version == 0 { e.versions.last() } else { e.versions.iter().find(|v| v.version == version) };
            let found = found.ok_or_else(|| {
                if e.kind == WorkspaceKind::File { Error::not_found("version") } else { Error::bad_request(format!("'{}' is not a file", e.name)) }
            })?;
            Ok((e.name.clone(), found.clone()))
        })
    }

    /// Make a folder in `user`'s own workspace.
    pub fn create_folder(&self, user: &User, parent: &str, name: &str) -> Result<EntryView, Error> {
        let name = clean_name(name)?;
        let (user, parent) = (user.clone(), parent.trim().to_string());
        self.mutate(move |state| {
            check_folder(state, &user.id, &parent)?;
            depth_ok(state, &parent)?;
            check_unique(state, &user.id, &parent, &name, "")?;
            let made = blank(&user, &parent, name, WorkspaceKind::Folder);
            state.workspace.push(made.clone());
            Ok(view(state, &made))
        })
    }

    /// Link a part (by id or number) into `user`'s workspace, following its
    /// newest revision, or pinned to `revision` (an id or label). The name
    /// defaults to the part's number.
    ///
    /// Idempotent: when the folder already links that part the same way
    /// (following, or pinned to the same revision) the answer is that entry,
    /// and nothing is added. A client that retries, or that links a part the
    /// server already linked on create ([`link_on_create`]), makes one link.
    pub fn create_link(&self, user: &User, parent: &str, name: &str, part: &str, revision: &str) -> Result<EntryView, Error> {
        let (user, parent) = (user.clone(), parent.trim().to_string());
        let (name, part, revision) = (name.to_string(), part.to_string(), revision.to_string());
        self.mutate(move |state| {
            check_folder(state, &user.id, &parent)?;
            let target = state.part_by_id_or_number(part.trim()).ok_or_else(|| Error::not_found("part"))?;
            let part_id = target.id.clone();
            let name = clean_name(if name.trim().is_empty() { &target.number } else { &name })?;
            let revision_id = pin_of(state, &part_id, &revision)?;
            let same = state.workspace.iter().find(|e| {
                e.owner == user.id
                    && e.parent == parent
                    && e.link.as_ref().is_some_and(|l| l.part_id == part_id && l.revision_id == revision_id)
            });
            if let Some(same) = same {
                return Ok(view(state, same));
            }
            check_unique(state, &user.id, &parent, &name, "")?;
            let mut made = blank(&user, &parent, name, WorkspaceKind::Link);
            made.link = Some(WorkspaceLink { part_id, revision_id });
            state.workspace.push(made.clone());
            Ok(view(state, &made))
        })
    }

    /// Store a new file in `user`'s workspace: version 1.
    pub fn create_file(&self, user: &User, parent: &str, name: &str, media_type: &str, staged: Staged) -> Result<EntryView, Error> {
        let name = clean_name(name)?;
        let media_type = attach::clean_media_type(media_type);
        let (user, parent) = (user.clone(), parent.trim().to_string());
        let root = self.root().to_path_buf();
        self.mutate(move |state| {
            check_folder(state, &user.id, &parent)?;
            check_unique(state, &user.id, &parent, &name, "")?;
            attach::place(&root, &staged)?;
            let mut made = blank(&user, &parent, name, WorkspaceKind::File);
            made.versions.push(FileVersion {
                version: 1,
                sha256: staged.sha256.clone(),
                size: staged.size,
                media_type,
                uploaded_by: user.id.clone(),
                uploaded_at: made.created_at,
                restored_from: None,
            });
            state.workspace.push(made.clone());
            Ok(view(state, &made))
        })
    }

    /// Replace a file's bytes. The version before stays in its list.
    pub fn replace_file(&self, user: &User, id: &str, media_type: &str, staged: Staged) -> Result<EntryView, Error> {
        let media_type = attach::clean_media_type(media_type);
        let (user, id) = (user.clone(), id.to_string());
        let root = self.root().to_path_buf();
        self.mutate(move |state| {
            let e = owned(state, &user, &id)?;
            if e.kind != WorkspaceKind::File {
                return Err(Error::bad_request(format!("'{}' is a {}, not a file", e.name, e.kind.as_str())));
            }
            attach::place(&root, &staged)?;
            let at = now();
            let e = entry_mut(state, &id)?;
            let next = e.versions.last().map(|v| v.version).unwrap_or(0) + 1;
            // An upload without a type keeps the one the file had.
            let media_type = if media_type == "application/octet-stream" {
                e.versions.last().map(|v| v.media_type.clone()).unwrap_or(media_type)
            } else {
                media_type
            };
            e.versions.push(FileVersion {
                version: next,
                sha256: staged.sha256.clone(),
                size: staged.size,
                media_type,
                uploaded_by: user.id.clone(),
                uploaded_at: at,
                restored_from: None,
            });
            e.modified_at = at;
            let e = e.clone();
            Ok(view(state, &e))
        })
    }

    /// Make an old version current again, as a NEW version with its bytes,
    /// so the versions after it are kept too.
    pub fn restore_version(&self, user: &User, id: &str, version: u32) -> Result<EntryView, Error> {
        let (user, id) = (user.clone(), id.to_string());
        self.mutate(move |state| {
            let e = owned(state, &user, &id)?;
            if e.kind != WorkspaceKind::File {
                return Err(Error::bad_request(format!("'{}' is a {}, not a file", e.name, e.kind.as_str())));
            }
            let old = e.versions.iter().find(|v| v.version == version).cloned().ok_or_else(|| Error::not_found("version"))?;
            let at = now();
            let e = entry_mut(state, &id)?;
            let next = e.versions.last().map(|v| v.version).unwrap_or(0) + 1;
            e.versions.push(FileVersion {
                version: next,
                uploaded_by: user.id.clone(),
                uploaded_at: at,
                restored_from: Some(old.version),
                ..old
            });
            e.modified_at = at;
            let e = e.clone();
            Ok(view(state, &e))
        })
    }

    /// Rename, move, or re-pin an entry. `fields` may carry `name`,
    /// `parent` (a folder id, or empty for the top) and, for a link,
    /// `revision` (an id or label to pin to, or empty to follow the newest).
    pub fn update_entry(&self, user: &User, id: &str, fields: &Value) -> Result<EntryView, Error> {
        let fields = fields.as_object().cloned().ok_or_else(|| Error::bad_request("the changes must be an object"))?;
        for key in fields.keys() {
            if !["name", "parent", "revision"].contains(&key.as_str()) {
                return Err(Error::bad_request(format!("a workspace entry has no field '{key}' to change")));
            }
        }
        let text = |key: &str| -> Result<Option<String>, Error> {
            match fields.get(key) {
                None => Ok(None),
                Some(Value::String(s)) => Ok(Some(s.trim().to_string())),
                Some(Value::Null) => Ok(Some(String::new())),
                Some(_) => Err(Error::bad_request(format!("'{key}' must be text"))),
            }
        };
        let name = text("name")?.map(|n| clean_name(&n)).transpose()?;
        let parent = text("parent")?;
        let revision = text("revision")?;
        let (user, id) = (user.clone(), id.to_string());
        self.mutate(move |state| {
            let e = owned(state, &user, &id)?.clone();
            let new_parent = parent.unwrap_or_else(|| e.parent.clone());
            let new_name = name.unwrap_or_else(|| e.name.clone());
            if new_parent != e.parent {
                check_folder(state, &user.id, &new_parent)?;
                if ancestry(state, &new_parent).contains(&e.id) {
                    return Err(Error::bad_request(format!("'{}' cannot move into itself", e.name)));
                }
                let deepest = subtree(state, &e.id).iter().map(|d| ancestry(state, d).len() - ancestry(state, &e.id).len()).max().unwrap_or(0);
                if ancestry(state, &new_parent).len() + deepest + 1 > MAX_DEPTH {
                    return Err(Error::bad_request(format!("folders nest at most {MAX_DEPTH} deep")));
                }
            }
            if new_parent != e.parent || new_name != e.name {
                check_unique(state, &user.id, &new_parent, &new_name, &e.id)?;
            }
            let link = match (&revision, &e.link) {
                (None, link) => link.clone(),
                (Some(key), Some(link)) => Some(WorkspaceLink { part_id: link.part_id.clone(), revision_id: pin_of(state, &link.part_id, key)? }),
                (Some(_), None) => return Err(Error::bad_request(format!("'{}' is not a link; only a link has a revision", e.name))),
            };
            let at = now();
            let stored = entry_mut(state, &id)?;
            stored.parent = new_parent;
            stored.name = new_name;
            stored.link = link;
            stored.modified_at = at;
            let stored = stored.clone();
            Ok(view(state, &stored))
        })
    }

    /// Remove an entry — a link, a file with every version, or a folder.
    /// A folder that holds anything is refused unless `recursive`. Never
    /// touches a part or a revision; a file's blobs go when nothing else
    /// references them. Answers the ids removed.
    pub fn delete_entry(&self, user: &User, id: &str, recursive: bool) -> Result<Vec<String>, Error> {
        let (user, id) = (user.clone(), id.to_string());
        let root = self.root().to_path_buf();
        self.mutate(move |state| {
            let e = owned(state, &user, &id)?;
            let doomed = subtree(state, &e.id);
            if doomed.len() > 1 && !recursive {
                return Err(Error::conflict(format!(
                    "the folder '{}' holds {} entries — empty it, or delete it with everything in it",
                    e.name,
                    doomed.len() - 1
                )));
            }
            let blobs: BTreeSet<String> = state
                .workspace
                .iter()
                .filter(|e| doomed.contains(&e.id))
                .flat_map(|e| e.versions.iter().map(|v| v.sha256.clone()))
                .collect();
            state.workspace.remove(&doomed);
            for sha in &blobs {
                attach::remove_if_unreferenced(state, &root, sha)?;
            }
            Ok(doomed.into_iter().collect())
        })
    }

    /// Promote a version of a workspace file to a real attachment on a part
    /// or one of its revisions. Every attachment rule applies: the author
    /// group, a revision in work, not checked out by someone else. The bytes
    /// are shared, not copied, and the file stays in the workspace.
    pub fn promote(&self, user: &User, id: &str, ask: &Promote) -> Result<Attachment, Error> {
        let (name, version) = self.read(|state| {
            let e = owned(state, user, id)?;
            let found = if ask.version == 0 { e.versions.last() } else { e.versions.iter().find(|v| v.version == ask.version) };
            let found = found.ok_or_else(|| {
                if e.kind == WorkspaceKind::File { Error::not_found("version") } else { Error::bad_request(format!("'{}' is not a file", e.name)) }
            })?;
            Ok::<_, Error>((e.name.clone(), found.clone()))
        })?;
        let describe = Describe {
            name: if ask.name.trim().is_empty() { name } else { ask.name.clone() },
            media_type: version.media_type.clone(),
            kind: ask.kind.clone(),
            note: ask.note.clone(),
        };
        let (entry_id, sha) = (id.to_string(), version.sha256.clone());
        // Inside the lock: the version must still be there, or its blob may
        // already be gone with the entry.
        self.attach_blob(user, &ask.part, &ask.revision, &version.sha256, version.size, None, &describe, move |state| {
            let still = state
                .workspace
                .iter()
                .any(|e| e.id == entry_id && e.versions.iter().any(|v| v.sha256 == sha));
            if still { Ok(()) } else { Err(Error::conflict("that file was changed or removed meanwhile — try again")) }
        })
    }
}

// ===========================================================================
// The in-memory collection
// ===========================================================================

/// Every user's workspace entries, journaled ([`crate::journal`]).
///
/// [`Db::mutate`] serializes and diffs the state outside its journaled lists
/// on every change. Kept there, workspaces made every write on the server
/// O(all entries): measured, a folder created with 10,000 entries in the
/// store took 330 ms and with 50,000 took 2.45 s (0.5 ms with none).
pub type Workspace = crate::journal::Journaled<WorkspaceEntry>;

impl crate::journal::Keyed for WorkspaceEntry {
    fn key(&self) -> &str {
        &self.id
    }
}
