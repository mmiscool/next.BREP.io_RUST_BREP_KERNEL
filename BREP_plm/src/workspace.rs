//! Workspaces: folders of links and files, and each user's Home
//! (plm-cad-integration D12, prerequisite P6). In PLM mode the CAD app's file
//! explorer becomes a browser of these (S14).
//!
//! # What an entry is
//!
//! A [`WorkspaceEntry`] is a folder, a link or a file:
//!
//! - A **folder** is stored once, like a part, and sits nowhere: every place
//!   it appears (a user's Home, another folder) is a **link** to it. Any
//!   number of places, in any users' Homes, link the same folder; its name,
//!   owner and contents are the same from each. Removing a link never touches
//!   the folder, as removing a link to a part never touches the part.
//! - A **link** points at a part or a folder by id. A link to a part follows
//!   the part's newest revision, drafts included (D13), or is pinned to one
//!   revision. Because it holds the id, a part renamed or renumbered is still
//!   found. Parts are never deleted, but a draft revision can be; a link
//!   pinned to one then says `revision_missing` rather than failing.
//! - A **file** is any bytes a user keeps with their work (a customer's
//!   photo, notes). Like a folder it is stored once, and every place it
//!   appears is a link to it. Its versions are blobs in the attachment
//!   store, under the same size limit. **Every replace keeps the version
//!   before it**, a restore adds the old bytes as a new version, and a file
//!   can be **promoted** to a real attachment on a part or revision, which
//!   obeys every attachment rule.
//! - A link can also point at a part's or revision's **attachment**: the
//!   attachment shown in a folder, read-only there, changed on its part.
//! - A user's **Home** is the links and files at its top (`parent` empty,
//!   `owner` the user). It is not a folder anyone else can hold.
//!
//! Names are unique, ignoring ASCII case, among the part links and files of
//! one folder or Home. A folder's name is its own, wherever it is linked.
//!
//! # Who may do what
//!
//! Ownership is individual and has nothing to do with where a thing is
//! linked. A folder's **owner** renames it, gives it a new owner, and adds
//! to or removes from it; a user does the same in their own Home. Anyone who
//! may read a folder may link it into a folder or Home of their own; only its
//! owner puts it in someone else's Home. Reading a folder is for its owner,
//! any administrator, everyone who has it in their Home (linked there,
//! directly or through other folders), and everyone while the administrator
//! setting [`crate::model::Settings::workspaces_browsable`] is on (the
//! default). Any signed-in user has a Home, viewers included: it changes no
//! part. Promoting a file to an attachment needs the author group, as
//! attaching does.

use std::collections::BTreeSet;

use serde::Serialize;
use serde_json::Value;

use crate::attach::{self, Describe, Staged};
use crate::db::{now, Db, State};
use crate::model::{Attachment, FileVersion, Origin, Part, User, WorkspaceEntry, WorkspaceKind, WorkspaceLink};
use crate::Error;

/// The workspace model `migrate` brings a store up to: 1, folders stored
/// once and placed by links; 2, files too.
pub const MODEL: u32 = 2;

const MAX_NAME: usize = 200;


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

/// The folder a link points at; `None` for any other entry.
fn linked_folder(e: &WorkspaceEntry) -> Option<&str> {
    e.link.as_ref().map(|l| l.folder_id.as_str()).filter(|f| !f.is_empty())
}

/// The stored folder or file a link points at; `None` for any other entry.
fn linked_object(e: &WorkspaceEntry) -> Option<&str> {
    e.link.as_ref().map(|l| if l.folder_id.is_empty() { l.file_id.as_str() } else { l.folder_id.as_str() }).filter(|f| !f.is_empty())
}

/// A stored folder or file, which sits nowhere and is placed by links.
fn stored(e: &WorkspaceEntry) -> bool {
    matches!(e.kind, WorkspaceKind::Folder | WorkspaceKind::File)
}

/// A link to a part (not to a part's attachment).
fn part_link(l: &WorkspaceLink) -> bool {
    !l.part_id.is_empty() && l.attachment_id.is_empty()
}

/// The stored folder or file `e` is or points at.
fn object_of<'a>(state: &'a State, e: &'a WorkspaceEntry) -> Option<&'a WorkspaceEntry> {
    if stored(e) { Some(e) } else { linked_object(e).and_then(|o| state.workspace.get(o)) }
}

/// The stored file `id` names: the file, or the file a link points at. A
/// link to an attachment is refused: that is changed on its part.
fn resolve_file<'a>(state: &'a State, id: &str) -> Result<&'a WorkspaceEntry, Error> {
    let found = entry(state, id)?;
    if found.link.as_ref().is_some_and(|l| !l.attachment_id.is_empty()) {
        return Err(Error::bad_request(format!("'{}' is an attachment of a part: change it on the part", found.name)));
    }
    match object_of(state, found) {
        Some(file) if file.kind == WorkspaceKind::File => Ok(file),
        _ => Err(Error::bad_request(format!("'{}' is not a file", found.name))),
    }
}

/// The folder `id` names: a folder, or the folder a link points at. Every
/// folder id a request gives comes through here, so a folder is the same
/// folder by its own id or any link's.
fn resolve_folder<'a>(state: &'a State, id: &str) -> Result<&'a WorkspaceEntry, Error> {
    let found = entry(state, id).map_err(|_| Error::not_found("folder"))?;
    let found = match linked_folder(found) {
        Some(target) => entry(state, target).map_err(|_| Error::not_found("folder"))?,
        None => found,
    };
    if found.kind != WorkspaceKind::Folder {
        return Err(Error::bad_request(format!("'{}' is a {}, not a folder", found.name, found.kind.as_str())));
    }
    Ok(found)
}

/// Whether `user` has the stored folder or file `object` in their Home: a
/// link at their Home's top points at it, or at a folder that (through
/// links) holds it.
fn in_home_of(state: &State, user: &str, object: &str) -> bool {
    let mut seen = BTreeSet::new();
    let mut pending = vec![object.to_string()];
    while let Some(at) = pending.pop() {
        if !seen.insert(at.clone()) {
            continue;
        }
        for link in state.workspace.iter().filter(|e| linked_object(e) == Some(at.as_str())) {
            if link.parent.is_empty() {
                if link.owner == user {
                    return true;
                }
            } else {
                pending.push(link.parent.clone());
            }
        }
    }
    false
}

/// May `viewer` read the stored folder or file `f`?
fn may_read_folder(state: &State, viewer: &User, f: &WorkspaceEntry) -> Result<(), Error> {
    if f.owner == viewer.id || viewer.is_admin() || state.settings.workspaces_browsable || in_home_of(state, &viewer.id, &f.id) {
        Ok(())
    } else {
        Err(Error::forbidden(format!("the {} '{}' is private", f.kind.as_str(), f.name)))
    }
}

/// May `viewer` read the entry `e`: a stored folder or file as itself, a
/// link what its folder or Home allows.
fn may_see(state: &State, viewer: &User, e: &WorkspaceEntry) -> Result<(), Error> {
    if stored(e) {
        return may_read_folder(state, viewer, e);
    }
    if e.parent.is_empty() {
        let holder = state.user(&e.owner).cloned().ok_or_else(|| Error::not_found("user"))?;
        return may_read(state, viewer, &holder);
    }
    may_read_folder(state, viewer, entry(state, &e.parent)?)
}

/// Who may change what is in the folder or Home `parent` (empty: the Home
/// of `home`): the folder's owner, or the Home's user.
fn container_owner(state: &State, parent: &str, home: &str) -> Result<String, Error> {
    if parent.is_empty() { Ok(home.to_string()) } else { Ok(entry(state, parent)?.owner.clone()) }
}

/// The entry `id`, which `user` may change: a folder or file they own, or a
/// link in a folder they own or in their Home.
fn owned<'a>(state: &'a State, user: &User, id: &str) -> Result<&'a WorkspaceEntry, Error> {
    let found = entry(state, id)?;
    let owner = if stored(found) { found.owner.clone() } else { container_owner(state, &found.parent, &found.owner)? };
    if owner != user.id {
        return Err(Error::forbidden(if stored(found) {
            format!("only the owner of the {} '{}' changes it", found.kind.as_str(), found.name)
        } else {
            "only the owner of the folder (or Home) it is in changes it".to_string()
        }));
    }
    Ok(found)
}

/// The stored file `id` names (itself or a link's), which `user` owns: who
/// adds its versions, restores and promotes it.
fn owned_file<'a>(state: &'a State, user: &User, id: &str) -> Result<&'a WorkspaceEntry, Error> {
    let file = resolve_file(state, id)?;
    if file.owner != user.id {
        return Err(Error::forbidden(format!("only the owner of the file '{}' changes it", file.name)));
    }
    Ok(file)
}

/// Refuse a new version from a caller who may not add it before an upload
/// is read ([`crate::api::workspace`]).
pub fn check_may_change(state: &State, user: &User, id: &str) -> Result<(), Error> {
    owned_file(state, user, id).map(|_| ())
}

/// Where a new entry for `parent` goes, and the `owner` it gets: the top of
/// `user`'s Home (owner: the user) when `parent` is empty, else the folder
/// it names (a link resolved to its folder; owner: `user`), which `user` must
/// own.
fn container(state: &State, user: &User, parent: &str) -> Result<(String, String), Error> {
    let parent = parent.trim();
    if parent.is_empty() {
        return Ok((String::new(), user.id.clone()));
    }
    let folder = resolve_folder(state, parent)?;
    if folder.owner != user.id {
        return Err(Error::forbidden(format!("only the owner of '{}' adds to it", folder.name)));
    }
    Ok((folder.id.clone(), user.id.clone()))
}

/// Whether the folder `to` is `from` or held in it, following the folders
/// linked in each: putting a link to `from` in `to` would loop.
fn within(state: &State, from: &str, to: &str) -> bool {
    let mut seen = BTreeSet::new();
    let mut pending = vec![from.to_string()];
    while let Some(at) = pending.pop() {
        if at == to {
            return true;
        }
        if !seen.insert(at.clone()) {
            continue;
        }
        pending.extend(state.workspace.iter().filter(|e| e.parent == at).filter_map(|e| linked_folder(e).map(str::to_string)));
    }
    false
}

/// Refuse a name another link placed in the same folder (or, at the top,
/// the same Home) already has. Checked when something is placed or moved;
/// an owner renaming a folder or file is not refused by a name in a place
/// they may not even see.
fn check_unique(state: &State, owner: &str, parent: &str, name: &str, except: &str) -> Result<(), Error> {
    let clash = state.workspace.iter().any(|e| {
        e.parent == parent
            && (!parent.is_empty() || e.owner == owner)
            && !stored(e)
            && e.id != except
            && e.name.eq_ignore_ascii_case(name)
    });
    if clash {
        return Err(Error::conflict(format!("this folder already holds '{name}'")));
    }
    Ok(())
}

/// One way from `user`'s Home down to the folder `folder`: its names, top
/// first. `None` when it is not in their Home.
fn path_in_home(state: &State, user: &str, folder: &str) -> Option<Vec<String>> {
    let mut seen = BTreeSet::new();
    let mut pending = vec![(folder.to_string(), Vec::<String>::new())];
    while let Some((at, mut below)) = pending.pop() {
        if !seen.insert(at.clone()) {
            continue;
        }
        let name = state.workspace.get(&at)?.name.clone();
        below.insert(0, name);
        for link in state.workspace.iter().filter(|e| linked_folder(e) == Some(at.as_str())) {
            if link.parent.is_empty() {
                if link.owner == user {
                    return Some(below);
                }
            } else {
                pending.push((link.parent.clone(), below.clone()));
            }
        }
    }
    None
}

/// A folder that links a part ([`Db::folders_linking`]): its id (empty for
/// the top of the Home), its name and a path to it from the Home
/// (`Frames/Welded`).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct LinkingFolder {
    pub id: String,
    pub name: String,
    pub path: String,
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
    /// Who owns it: for a folder (or a link to one), the folder's owner; for
    /// an entry at the top of a Home, that Home's user; else its creator.
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
    /// For a folder: the folder itself, which `id` (a link placing it here)
    /// points at. List and add to it by either id.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub folder_id: Option<String>,
    /// For a file: the file itself, which `id` (a link placing it here)
    /// points at. Its versions and content by either id.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub file_id: Option<String>,
    /// For a link to a part's attachment: which, and where it is.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub attachment: Option<AttachmentRef>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub link: Option<LinkView>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub file: Option<FileView>,
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

/// A part's (or revision's) attachment a link shows in a folder.
#[derive(Debug, Clone, Serialize)]
pub struct AttachmentRef {
    pub id: String,
    pub part_id: String,
    pub number: String,
    /// Empty for an attachment of the part itself.
    pub revision_id: String,
    pub revision_label: String,
    pub kind: String,
    /// The attachment is gone from its part (a draft revision deleted, or the
    /// attachment removed).
    pub missing: bool,
}

fn user_name(state: &State, id: &str) -> String {
    state.user(id).map(|u| u.username.clone()).unwrap_or_else(|| "(removed user)".into())
}

fn file_view(state: &State, versions: &[FileVersion]) -> Option<FileView> {
    versions.last().map(|v| FileView {
        version: v.version,
        versions: versions.len(),
        size: v.size,
        media_type: v.media_type.clone(),
        sha256: v.sha256.clone(),
        uploaded_at: v.uploaded_at,
        uploaded_by_name: user_name(state, &v.uploaded_by),
    })
}

/// An attachment as the one version of a file.
fn attachment_version(a: &Attachment) -> FileVersion {
    FileVersion {
        version: 1,
        sha256: a.sha256.clone(),
        size: a.size,
        media_type: a.media_type.clone(),
        uploaded_by: a.uploaded_by.clone(),
        uploaded_at: a.uploaded_at,
        restored_from: None,
    }
}

/// The attachment a link points at, where it is now.
fn linked_attachment(state: &State, l: &WorkspaceLink) -> Option<attach::Located> {
    if l.attachment_id.is_empty() {
        return None;
    }
    attach::locate_scoped(state, &l.attachment_id, &l.part_id, &l.revision_id)
}

/// An entry as [`EntryView`]. A folder or file, or a link to one, shows as
/// the folder or file itself (its name, owner, contents or versions); a
/// link's `id` is the link's and its `parent` where it is placed. A link to
/// an attachment shows as a file.
pub fn view(state: &State, e: &WorkspaceEntry) -> EntryView {
    if let Some(l) = e.link.as_ref().filter(|l| !l.attachment_id.is_empty()) {
        let found = linked_attachment(state, l);
        let part = state.part(&l.part_id);
        return EntryView {
            id: e.id.clone(),
            owner: e.owner.clone(),
            owner_name: user_name(state, &e.owner),
            parent: e.parent.clone(),
            name: found.as_ref().map(|f| f.attachment.name.clone()).unwrap_or_else(|| e.name.clone()),
            kind: WorkspaceKind::File,
            created_at: e.created_at,
            modified_at: e.modified_at,
            children: None,
            folder_id: None,
            file_id: None,
            attachment: Some(AttachmentRef {
                id: l.attachment_id.clone(),
                part_id: l.part_id.clone(),
                number: part.map(|p| p.number.clone()).unwrap_or_default(),
                revision_id: l.revision_id.clone(),
                revision_label: found.as_ref().map(|f| f.revision_label.clone()).unwrap_or_default(),
                kind: found.as_ref().map(|f| f.attachment.kind.clone()).unwrap_or_default(),
                missing: found.is_none(),
            }),
            link: None,
            file: found.map(|f| file_view(state, &[attachment_version(&f.attachment)])).unwrap_or_default(),
        };
    }
    if let Some(file) = object_of(state, e).filter(|o| o.kind == WorkspaceKind::File) {
        return EntryView {
            id: e.id.clone(),
            owner: file.owner.clone(),
            owner_name: user_name(state, &file.owner),
            parent: e.parent.clone(),
            name: file.name.clone(),
            kind: WorkspaceKind::File,
            created_at: file.created_at,
            modified_at: file.modified_at,
            children: None,
            folder_id: None,
            file_id: Some(file.id.clone()),
            attachment: None,
            link: None,
            file: file_view(state, &file.versions),
        };
    }
    let folder = if e.kind == WorkspaceKind::Folder { Some(e) } else { linked_folder(e).and_then(|f| state.workspace.get(f)) };
    if let Some(folder) = folder {
        return EntryView {
            id: e.id.clone(),
            owner: folder.owner.clone(),
            owner_name: user_name(state, &folder.owner),
            parent: e.parent.clone(),
            name: folder.name.clone(),
            kind: WorkspaceKind::Folder,
            created_at: folder.created_at,
            modified_at: folder.modified_at,
            children: Some(state.workspace.iter().filter(|c| c.parent == folder.id).count()),
            folder_id: Some(folder.id.clone()),
            file_id: None,
            attachment: None,
            link: None,
            file: None,
        };
    }
    EntryView {
        id: e.id.clone(),
        owner: e.owner.clone(),
        owner_name: user_name(state, &e.owner),
        parent: e.parent.clone(),
        name: e.name.clone(),
        kind: e.kind,
        created_at: e.created_at,
        modified_at: e.modified_at,
        children: None,
        folder_id: None,
        file_id: None,
        attachment: None,
        link: e.link.as_ref().map(|l| resolve(state, l)),
        file: None,
    }
}

/// A workspace another user may open, as the list of them shows it.
#[derive(Debug, Clone, Serialize)]
pub struct WorkspaceRow {
    pub user_id: String,
    pub username: String,
    pub display_name: String,
    pub mine: bool,
    /// How many links they have placed (folders and files are stored apart).
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

fn blank(owner: &str, parent: &str, name: String, kind: WorkspaceKind) -> WorkspaceEntry {
    let at = now();
    WorkspaceEntry {
        id: crate::auth::new_id(),
        owner: owner.to_string(),
        parent: parent.to_string(),
        name,
        kind,
        link: None,
        versions: Vec::new(),
        created_at: at,
        modified_at: at,
    }
}

/// A link placing the stored folder or file `object` in `parent` (empty:
/// the Home of `owner`).
fn folder_link(owner: &str, parent: &str, object: &WorkspaceEntry) -> WorkspaceEntry {
    let mut made = blank(owner, parent, object.name.clone(), WorkspaceKind::Link);
    let (folder_id, file_id) = if object.kind == WorkspaceKind::Folder { (object.id.clone(), String::new()) } else { (String::new(), object.id.clone()) };
    made.link = Some(WorkspaceLink { folder_id, file_id, ..WorkspaceLink::default() });
    made
}

/// What [`Db::paste_copy`] copies.
#[derive(Debug, Clone)]
pub enum CopyOf {
    /// A folder, by its id or a link's.
    Folder(String),
    /// A file, by its id or a link's.
    File(String),
    /// A part's attachment (`revision` empty) or a revision's.
    Attachment { id: String, part: String, revision: String },
}

/// A new stored copy of `source` (a folder or file, or a file not yet
/// stored) named `name` and owned by `user`, pushed into the store. A
/// folder's folders and files are copied too, its other links linked again;
/// `seen` stops a folder that holds itself through links.
fn copy_object(state: &mut State, user: &User, source: &WorkspaceEntry, name: &str, seen: &mut BTreeSet<String>) -> Result<WorkspaceEntry, Error> {
    let mut copy = blank(&user.id, "", name.to_string(), source.kind);
    copy.versions = source.versions.clone();
    state.workspace.push(copy.clone());
    if source.kind == WorkspaceKind::Folder && seen.insert(source.id.clone()) {
        let inside: Vec<WorkspaceEntry> = state.workspace.iter().filter(|e| e.parent == source.id).cloned().collect();
        for placed in inside {
            let object = linked_object(&placed).and_then(|o| state.workspace.get(o)).cloned();
            let mut link = match object {
                Some(object) if !seen.contains(&object.id) => {
                    let inner = copy_object(state, user, &object, &object.name, seen)?;
                    folder_link(&user.id, &copy.id, &inner)
                }
                _ => WorkspaceEntry { id: crate::auth::new_id(), ..placed.clone() },
            };
            link.parent = copy.id.clone();
            link.owner = user.id.clone();
            state.workspace.push(link);
        }
    }
    Ok(copy)
}

/// Bring a store's workspaces up to [`MODEL`], once, inside the caller's
/// [`Db::mutate`]. Model 0 kept each folder in its creator's tree, in a
/// parent folder or at the top: each becomes a folder stored on its own, and
/// a link takes its place. Its id is kept, so what is in it, pins and
/// routes that name it, still find it.
///
/// Model 1 kept each file in the folder (or Home) it was put in: it becomes
/// a file stored on its own in the same way, keeping its id and versions.
pub(crate) fn migrate(state: &mut State) -> Result<bool, Error> {
    if state.workspace_model >= MODEL {
        return Ok(false);
    }
    let kinds: &[WorkspaceKind] = if state.workspace_model < 1 { &[WorkspaceKind::Folder, WorkspaceKind::File] } else { &[WorkspaceKind::File] };
    let folders: Vec<WorkspaceEntry> = state.workspace.iter().filter(|e| kinds.contains(&e.kind)).cloned().collect();
    for folder in folders {
        let link = WorkspaceEntry { created_at: folder.created_at, modified_at: folder.modified_at, ..folder_link(&folder.owner, &folder.parent, &folder) };
        state.workspace.push(link);
        entry_mut(state, &folder.id)?.parent = String::new();
    }
    state.workspace_model = MODEL;
    Ok(true)
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

/// Link the just-created `part` into the folder `parent` of its creator
/// (empty: their Home), following its newest revision, inside the create's
/// own [`Db::mutate`] (so a refusal here refuses the part too, and the link
/// is audited with it). A folder that already links the part following
/// changes nothing. The name is the part's number, or `<number> (2)`, `(3)`,
/// … when another entry has taken it: a clash of names never refuses a new
/// part.
pub(crate) fn link_new_part(state: &mut State, creator: &str, parent: &str, part: &Part) -> Result<(), Error> {
    let user = state.user(creator).cloned().ok_or_else(|| Error::not_found("user"))?;
    let (parent, owner) = container(state, &user, parent)?;
    let home = parent.is_empty();
    let already = state.workspace.iter().any(|e| {
        e.parent == parent
            && (!home || e.owner == owner)
            && e.link.as_ref().is_some_and(|l| part_link(l) && l.part_id == part.id && l.revision_id.is_empty())
    });
    if already {
        return Ok(());
    }
    let base = clean_name(&part.number)?;
    let mut name = base.clone();
    let mut n = 2;
    while check_unique(state, &owner, &parent, &name, "").is_err() {
        name = format!("{base} ({n})");
        n += 1;
    }
    let mut made = blank(&owner, &parent, name, WorkspaceKind::Link);
    made.link = Some(WorkspaceLink { part_id: part.id.clone(), ..WorkspaceLink::default() });
    state.workspace.push(made);
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
                    entries: state.workspace.iter().filter(|e| e.owner == u.id && !stored(e)).count(),
                })
                .collect();
            rows.sort_by_key(|r| (!r.mine, r.username.to_ascii_lowercase()));
            rows
        })
    }

    /// The folders in `viewer`'s Home holding a link to `part` (an id or
    /// number), each once, by path: what an assembly opened some other way
    /// than from the workspace is filed under. The top of the Home is the
    /// folder with an empty id.
    pub fn folders_linking(&self, viewer: &User, part: &str) -> Result<Vec<LinkingFolder>, Error> {
        self.read(|state| {
            let part = state.part_by_id_or_number(part.trim()).ok_or_else(|| Error::not_found("part"))?;
            let mut out: Vec<LinkingFolder> = Vec::new();
            for link in state.workspace.iter().filter(|e| e.link.as_ref().is_some_and(|l| part_link(l) && l.part_id == part.id)) {
                if out.iter().any(|f| f.id == link.parent) {
                    continue;
                }
                if link.parent.is_empty() {
                    if link.owner == viewer.id {
                        out.push(LinkingFolder { id: String::new(), name: String::new(), path: String::new() });
                    }
                } else if let Some(names) = path_in_home(state, &viewer.id, &link.parent) {
                    let name = names.last().cloned().unwrap_or_default();
                    out.push(LinkingFolder { id: link.parent.clone(), name, path: names.join("/") });
                }
            }
            out.sort_by_key(|f| f.path.to_ascii_lowercase());
            Ok(out)
        })
    }

    /// What is in one folder (by its id, or any link's to it), or at the top
    /// of `owner`'s Home when `parent` is empty: folders first, then by name.
    pub fn workspace_folder(&self, viewer: &User, owner: &str, parent: &str) -> Result<Vec<EntryView>, Error> {
        self.read(|state| {
            let owner = owner_of(state, viewer, owner)?;
            let parent = parent.trim();
            let (parent, home) = if parent.is_empty() {
                may_read(state, viewer, &owner)?;
                (String::new(), true)
            } else {
                let folder = resolve_folder(state, parent)?;
                may_read_folder(state, viewer, folder)?;
                (folder.id.clone(), false)
            };
            let mut out: Vec<EntryView> = state
                .workspace
                .iter()
                .filter(|e| e.parent == parent && !stored(e) && (!home || e.owner == owner.id))
                .map(|e| view(state, e))
                .collect();
            out.sort_by_key(|e| (e.kind != WorkspaceKind::Folder, e.name.to_ascii_lowercase()));
            Ok(out)
        })
    }

    /// Every folder `viewer` may read, by name: the folders as parts are
    /// found in the catalog, linked anywhere or nowhere.
    pub fn workspace_folders(&self, viewer: &User) -> Vec<EntryView> {
        self.stored_of(viewer, WorkspaceKind::Folder)
    }

    /// Every file `viewer` may read, by name, linked anywhere or nowhere.
    pub fn workspace_files(&self, viewer: &User) -> Vec<EntryView> {
        self.stored_of(viewer, WorkspaceKind::File)
    }

    fn stored_of(&self, viewer: &User, kind: WorkspaceKind) -> Vec<EntryView> {
        self.read(|state| {
            let mut out: Vec<EntryView> = state
                .workspace
                .iter()
                .filter(|e| e.kind == kind && may_read_folder(state, viewer, e).is_ok())
                .map(|e| view(state, e))
                .collect();
            out.sort_by_key(|e| e.name.to_ascii_lowercase());
            out
        })
    }

    /// One entry, if `viewer` may read where it is.
    pub fn workspace_entry(&self, viewer: &User, id: &str) -> Result<EntryView, Error> {
        self.read(|state| {
            let e = entry(state, id)?;
            may_see(state, viewer, e)?;
            Ok(view(state, e))
        })
    }

    /// A file's versions, oldest first, by the file's id or a link's. A
    /// link to an attachment has the attachment as its one version.
    pub fn workspace_versions(&self, viewer: &User, id: &str) -> Result<Vec<VersionView>, Error> {
        self.read(|state| {
            let e = entry(state, id)?;
            may_see(state, viewer, e)?;
            let versions: Vec<FileVersion> = match e.link.as_ref().and_then(|l| linked_attachment(state, l).map(|f| (l, f))) {
                Some((_, found)) => vec![attachment_version(&found.attachment)],
                None => resolve_file(state, id)?.versions.clone(),
            };
            let last = versions.last().map(|v| v.version);
            Ok(versions
                .iter()
                .map(|v| VersionView {
                    version: v.clone(),
                    uploaded_by_name: user_name(state, &v.uploaded_by),
                    current: Some(v.version) == last,
                })
                .collect())
        })
    }

    /// One version of a file (0: the current one), by the file's id or a
    /// link's, with its name — what a download needs.
    pub fn workspace_file(&self, viewer: &User, id: &str, version: u32) -> Result<(String, FileVersion), Error> {
        self.read(|state| {
            let e = entry(state, id)?;
            may_see(state, viewer, e)?;
            if let Some(found) = e.link.as_ref().and_then(|l| linked_attachment(state, l)) {
                if version > 1 {
                    return Err(Error::not_found("version"));
                }
                return Ok((found.attachment.name.clone(), attachment_version(&found.attachment)));
            }
            let file = resolve_file(state, id)?;
            let found = if version == 0 { file.versions.last() } else { file.versions.iter().find(|v| v.version == version) };
            let found = found.ok_or_else(|| Error::not_found("version"))?;
            Ok((file.name.clone(), found.clone()))
        })
    }

    /// Make a folder, owned by `user`, and link it into their folder
    /// `parent` (empty: their Home). Answers the link.
    pub fn create_folder(&self, user: &User, parent: &str, name: &str) -> Result<EntryView, Error> {
        let name = clean_name(name)?;
        let (user, parent) = (user.clone(), parent.trim().to_string());
        self.mutate(move |state| {
            let (parent, owner) = container(state, &user, &parent)?;
            check_unique(state, &owner, &parent, &name, "")?;
            let folder = blank(&user.id, "", name, WorkspaceKind::Folder);
            let link = folder_link(&owner, &parent, &folder);
            state.workspace.push(folder);
            state.workspace.push(link.clone());
            Ok(view(state, &link))
        })
    }

    /// Link a part (by id or number) into a folder `user` owns, or their
    /// Home, following its newest revision, or pinned to `revision` (an id or
    /// label). The name defaults to the part's number.
    ///
    /// Idempotent: when the folder already links that part the same way
    /// (following, or pinned to the same revision) the answer is that entry,
    /// and nothing is added. A client that retries, or that links a part the
    /// server already linked on create ([`link_on_create`]), makes one link.
    pub fn create_link(&self, user: &User, parent: &str, name: &str, part: &str, revision: &str) -> Result<EntryView, Error> {
        let (user, parent) = (user.clone(), parent.trim().to_string());
        let (name, part, revision) = (name.to_string(), part.to_string(), revision.to_string());
        self.mutate(move |state| {
            let (parent, owner) = container(state, &user, &parent)?;
            let target = state.part_by_id_or_number(part.trim()).ok_or_else(|| Error::not_found("part"))?;
            let part_id = target.id.clone();
            let name = clean_name(if name.trim().is_empty() { &target.number } else { &name })?;
            let revision_id = pin_of(state, &part_id, &revision)?;
            let same = state.workspace.iter().find(|e| {
                e.parent == parent
                    && (!parent.is_empty() || e.owner == owner)
                    && e.link.as_ref().is_some_and(|l| l.part_id == part_id && l.revision_id == revision_id)
            });
            if let Some(same) = same {
                return Ok(view(state, same));
            }
            check_unique(state, &owner, &parent, &name, "")?;
            let mut made = blank(&owner, &parent, name, WorkspaceKind::Link);
            made.link = Some(WorkspaceLink { part_id, revision_id, ..WorkspaceLink::default() });
            state.workspace.push(made.clone());
            Ok(view(state, &made))
        })
    }

    /// Link a folder (its id, or any link's to it) into a folder `user` owns
    /// or their Home, or, with `home` naming a user, at the top of that
    /// user's Home, which only the folder's owner does. It is the same
    /// folder in every place; nothing is copied. Linking a folder needs only
    /// that `user` may read it. Idempotent: the same place again answers the
    /// link already there.
    pub fn link_folder(&self, user: &User, parent: &str, folder: &str, home: &str) -> Result<EntryView, Error> {
        let (user, parent, folder, home) = (user.clone(), parent.trim().to_string(), folder.trim().to_string(), home.trim().to_string());
        self.mutate(move |state| {
            let target = resolve_folder(state, &folder)?.clone();
            may_read_folder(state, &user, &target)?;
            let (parent, owner) = if home.is_empty() {
                container(state, &user, &parent)?
            } else {
                if !parent.is_empty() {
                    return Err(Error::bad_request("a folder goes to the top of another user's Home; give no 'parent' with 'home'"));
                }
                if target.owner != user.id {
                    return Err(Error::forbidden(format!("only the owner of '{}' puts it in another user's Home", target.name)));
                }
                let to = owner_of(state, &user, &home)?;
                if !to.active {
                    return Err(Error::bad_request(format!("{} is not an active user", to.username)));
                }
                (String::new(), to.id)
            };
            let same = state.workspace.iter().find(|e| {
                e.parent == parent && (!parent.is_empty() || e.owner == owner) && linked_folder(e) == Some(target.id.as_str())
            });
            if let Some(same) = same {
                return Ok(view(state, same));
            }
            if !parent.is_empty() && within(state, &target.id, &parent) {
                return Err(Error::bad_request(format!("'{}' cannot be placed inside itself", target.name)));
            }
            check_unique(state, &owner, &parent, &target.name, "")?;
            let made = folder_link(&owner, &parent, &target);
            state.workspace.push(made.clone());
            Ok(view(state, &made))
        })
    }

    /// Store a new file, owned by `user`, at version 1, and link it into a
    /// folder they own or their Home. Answers the link.
    pub fn create_file(&self, user: &User, parent: &str, name: &str, media_type: &str, staged: Staged) -> Result<EntryView, Error> {
        let name = clean_name(name)?;
        let media_type = attach::clean_media_type(media_type);
        let (user, parent) = (user.clone(), parent.trim().to_string());
        let root = self.root().to_path_buf();
        self.mutate(move |state| {
            let (parent, owner) = container(state, &user, &parent)?;
            check_unique(state, &owner, &parent, &name, "")?;
            attach::place(&root, &staged)?;
            let mut file = blank(&user.id, "", name, WorkspaceKind::File);
            file.versions.push(FileVersion {
                version: 1,
                sha256: staged.sha256.clone(),
                size: staged.size,
                media_type,
                uploaded_by: user.id.clone(),
                uploaded_at: file.created_at,
                restored_from: None,
            });
            let link = folder_link(&owner, &parent, &file);
            state.workspace.push(file);
            state.workspace.push(link.clone());
            Ok(view(state, &link))
        })
    }

    /// Replace a file's bytes. The version before stays in its list.
    pub fn replace_file(&self, user: &User, id: &str, media_type: &str, staged: Staged) -> Result<EntryView, Error> {
        self.replace_file_checked(user, id, media_type, staged, None)
    }

    pub fn replace_file_checked(&self, user: &User, id: &str, media_type: &str, staged: Staged, expected: Option<String>) -> Result<EntryView, Error> {
        let media_type = attach::clean_media_type(media_type);
        let (user, id) = (user.clone(), id.to_string());
        let root = self.root().to_path_buf();
        self.mutate(move |state| {
            let e = owned_file(state, &user, &id)?;
            let file = e.id.clone();
            if let Some(expected) = &expected {
                if e.versions.last().map(|v| &v.sha256) != Some(expected) {
                    return Err(Error::conflict("This file changed since you opened it. Reopen it before saving."));
                }
            }
            attach::place(&root, &staged)?;
            let at = now();
            let e = entry_mut(state, &file)?;
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
            Ok(view(state, entry(state, &id)?))
        })
    }

    /// Make an old version current again, as a NEW version with its bytes,
    /// so the versions after it are kept too.
    pub fn restore_version(&self, user: &User, id: &str, version: u32) -> Result<EntryView, Error> {
        let (user, id) = (user.clone(), id.to_string());
        self.mutate(move |state| {
            let e = owned_file(state, &user, &id)?;
            let file = e.id.clone();
            let old = e.versions.iter().find(|v| v.version == version).cloned().ok_or_else(|| Error::not_found("version"))?;
            let at = now();
            let e = entry_mut(state, &file)?;
            let next = e.versions.last().map(|v| v.version).unwrap_or(0) + 1;
            e.versions.push(FileVersion {
                version: next,
                uploaded_by: user.id.clone(),
                uploaded_at: at,
                restored_from: Some(old.version),
                ..old
            });
            e.modified_at = at;
            Ok(view(state, entry(state, &id)?))
        })
    }

    /// Link a file (its id, or any link's to it) into a folder `user` owns
    /// or their Home. It is the same file in every place: its name, owner
    /// and versions. Needs only that `user` may read it. Idempotent.
    pub fn link_file(&self, user: &User, parent: &str, file: &str) -> Result<EntryView, Error> {
        let (user, parent, file) = (user.clone(), parent.trim().to_string(), file.trim().to_string());
        self.mutate(move |state| {
            let target = resolve_file(state, &file)?.clone();
            may_read_folder(state, &user, &target)?;
            let (parent, owner) = container(state, &user, &parent)?;
            let same = state.workspace.iter().find(|e| {
                e.parent == parent && (!parent.is_empty() || e.owner == owner) && linked_object(e) == Some(target.id.as_str())
            });
            if let Some(same) = same {
                return Ok(view(state, same));
            }
            check_unique(state, &owner, &parent, &target.name, "")?;
            let made = folder_link(&owner, &parent, &target);
            state.workspace.push(made.clone());
            Ok(view(state, &made))
        })
    }

    /// Link a part's attachment (`revision` empty) or a revision's into a
    /// folder `user` owns or their Home. It shows there as a file, and stays
    /// the attachment: read-only in the folder, changed on its part.
    /// Idempotent.
    pub fn link_attachment(&self, user: &User, parent: &str, attachment: &str, part: &str, revision: &str) -> Result<EntryView, Error> {
        let (user, parent) = (user.clone(), parent.trim().to_string());
        let (attachment, part, revision) = (attachment.trim().to_string(), part.trim().to_string(), revision.trim().to_string());
        self.mutate(move |state| {
            let found = attach::locate_scoped(state, &attachment, &part, &revision).ok_or_else(|| Error::not_found("attachment"))?;
            let (parent, owner) = container(state, &user, &parent)?;
            let link = WorkspaceLink {
                part_id: found.part_id.clone(),
                revision_id: found.revision_id.clone(),
                attachment_id: found.attachment.id.clone(),
                ..WorkspaceLink::default()
            };
            let same = state.workspace.iter().find(|e| {
                e.parent == parent && (!parent.is_empty() || e.owner == owner) && e.link.as_ref() == Some(&link)
            });
            if let Some(same) = same {
                return Ok(view(state, same));
            }
            check_unique(state, &owner, &parent, &found.attachment.name, "")?;
            let mut made = blank(&owner, &parent, found.attachment.name.clone(), WorkspaceKind::Link);
            made.link = Some(link);
            state.workspace.push(made.clone());
            Ok(view(state, &made))
        })
    }

    /// Paste a **new unique copy** of a folder, a file or an attachment into
    /// a folder `user` owns or their Home (`what` names one; see
    /// [`CopyOf`]). Owned by `user`, named as the original, or `<name> (2)`,
    /// `(3)`, … when the place holds that name.
    ///
    /// - a file or an attachment: a new file holding the same bytes (shared
    ///   blobs, nothing re-uploaded), its versions its own from then on — a
    ///   file keeps its whole history;
    /// - a folder: a new folder, and in it new copies of its folders and
    ///   files, all the way down; what it links (parts, attachments) is linked
    ///   again, never copied.
    ///
    /// A new copy of a part is [`Db::copy_part`]'s. Answers the link placing
    /// the copy.
    pub fn paste_copy(&self, user: &User, parent: &str, what: &CopyOf) -> Result<EntryView, Error> {
        let (user, parent, what) = (user.clone(), parent.trim().to_string(), what.clone());
        self.mutate(move |state| {
            let (parent, owner) = container(state, &user, &parent)?;
            let source: WorkspaceEntry = match &what {
                CopyOf::Folder(id) => {
                    let found = resolve_folder(state, id)?.clone();
                    may_read_folder(state, &user, &found)?;
                    found
                }
                CopyOf::File(id) => {
                    let found = resolve_file(state, id)?.clone();
                    may_read_folder(state, &user, &found)?;
                    found
                }
                CopyOf::Attachment { id, part, revision } => {
                    let found = attach::locate_scoped(state, id, part, revision).ok_or_else(|| Error::not_found("attachment"))?;
                    let mut file = blank(&user.id, "", found.attachment.name.clone(), WorkspaceKind::File);
                    file.versions.push(FileVersion { uploaded_by: user.id.clone(), uploaded_at: file.created_at, ..attachment_version(&found.attachment) });
                    file
                }
            };
            let mut name = source.name.clone();
            let mut n = 2;
            while check_unique(state, &owner, &parent, &name, "").is_err() {
                name = format!("{} ({n})", source.name);
                n += 1;
            }
            let copy = copy_object(state, &user, &source, &name, &mut BTreeSet::new())?;
            let link = folder_link(&owner, &parent, &copy);
            state.workspace.push(link.clone());
            Ok(view(state, &link))
        })
    }

    /// Paste a **new unique copy** of a part: a new part, with its own number,
    /// of the same type, category, tags and attributes, holding the model of
    /// `revision` (an id or label; empty: the newest) and every attachment of
    /// the part and of that revision (the bytes shared, nothing re-uploaded).
    /// It is linked into `parent` (a folder `user` owns, or their Home)
    /// following its newest revision, as a part a user makes is. `number` is
    /// the new part's for a type that takes a typed number (empty: the
    /// type's counter or script). Needs the author group. Answers the new
    /// part.
    pub fn copy_part(&self, user: &User, parent: &str, part: &str, revision: &str, number: &str) -> Result<Part, Error> {
        let (source, rev, document_key) = self.read(|state| {
            let source = state.part_by_id_or_number(part.trim()).cloned().ok_or_else(|| Error::not_found("part"))?;
            let rev = if revision.trim().is_empty() { source.latest() } else { crate::bom::find_revision(&source, revision.trim()) }
                .cloned()
                .ok_or_else(|| Error::not_found("revision"))?;
            let key = (!rev.content_hash.is_empty()).then(|| rev.document_key(&source.id));
            Ok::<_, Error>((source, rev, key))
        })?;
        let document = match &document_key {
            Some(key) => self.read_document(key)?,
            None => None,
        };
        let made = self.create_part_with(
            user,
            &crate::db::PartSpec {
                part_type: source.part_type.clone(),
                number: number.trim().to_string(),
                name: source.name.clone(),
                description: source.description.clone(),
                category: source.category.clone(),
                document_class: source.document_class,
                tags: source.tags.clone(),
                attributes: source.attributes.clone().into_iter().collect(),
                workspace: Some(parent.trim().to_string()),
                ..crate::db::PartSpec::default()
            },
        )?;
        let first = made.revisions.first().map(|r| r.id.clone()).ok_or_else(|| Error::internal("a new part has no revision"))?;
        if let Some(body) = document {
            self.write_document(&made.id, &first, &body)?;
        }
        let carried = source.attachments.iter().map(|a| (a, "")).chain(rev.attachments.iter().map(|a| (a, first.as_str())));
        for (a, onto) in carried {
            let describe = Describe { name: a.name.clone(), media_type: a.media_type.clone(), kind: a.kind.clone(), note: a.note.clone() };
            self.attach_blob(user, &made.id, onto, &a.sha256, a.size, None, &describe, |_| Ok(()))?;
        }
        self.read(|state| state.part(&made.id).cloned()).ok_or_else(|| Error::not_found("part"))
    }

    /// Rename, move, re-pin or re-own an entry. `fields` may carry `name`,
    /// `parent` (a folder id, or empty for the Home's top), for a link to a
    /// part `revision` (an id or label to pin to, or empty to follow the
    /// newest), and for a folder `owner` (a user id or username).
    ///
    /// Through a link to a folder, a name or an owner is the folder's (and
    /// so in every place it is linked), which its owner changes; a parent
    /// moves the link, which the owners of both places allow. The owner is a
    /// property of the folder: it moves nothing.
    pub fn update_entry(&self, user: &User, id: &str, fields: &Value) -> Result<EntryView, Error> {
        let fields = fields.as_object().cloned().ok_or_else(|| Error::bad_request("the changes must be an object"))?;
        for key in fields.keys() {
            if !["name", "parent", "revision", "owner"].contains(&key.as_str()) {
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
        let new_owner = text("owner")?;
        let (user, id) = (user.clone(), id.to_string());
        self.mutate(move |state| {
            let e = entry(state, &id)?.clone();
            // A folder or file, by its own id or a link's: its name and owner
            // are its own, its owner's to change.
            let folder = object_of(state, &e).cloned();
            if e.link.as_ref().is_some_and(|l| !l.attachment_id.is_empty()) && name.is_some() {
                return Err(Error::bad_request(format!("'{}' is an attachment of a part: rename it on the part", e.name)));
            }
            let mut folder_changes = false;
            if let Some(folder) = &folder {
                let renamed = name.as_ref().is_some_and(|n| *n != folder.name);
                let reowned = new_owner.is_some();
                if renamed || reowned {
                    if folder.owner != user.id {
                        return Err(Error::forbidden(format!("only the owner of the {} '{}' renames it or gives it a new owner", folder.kind.as_str(), folder.name)));
                    }
                    folder_changes = true;
                }
            } else if new_owner.is_some() {
                return Err(Error::bad_request(format!("'{}' is a link to a part; only a folder or a file has an owner to change", e.name)));
            }
            let to = match &new_owner {
                None => None,
                Some(key) if key.is_empty() => return Err(Error::bad_request("a folder or file needs an owner")),
                Some(key) => {
                    let to = state.user(key).or_else(|| state.user_by_name(key)).cloned().ok_or_else(|| Error::not_found("user"))?;
                    if !to.active && folder.as_ref().is_some_and(|f| f.owner != to.id) {
                        return Err(Error::bad_request(format!("{} is not an active user", to.username)));
                    }
                    Some(to.id)
                }
            };
            // The placement's fields: the owner of where it is.
            let moved = parent.as_ref().is_some_and(|p| *p != e.parent);
            let place_changes = !stored(&e) && (moved || revision.is_some() || (folder.is_none() && name.is_some()));
            if stored(&e) && parent.is_some() {
                return Err(Error::bad_request(format!("a {} is stored once and sits nowhere; move a link to it", e.kind.as_str())));
            }
            if place_changes {
                owned(state, &user, &id)?;
            }
            let (new_parent, new_owner_of_place) = match &parent {
                Some(p) if moved => container(state, &user, p)?,
                _ => (e.parent.clone(), e.owner.clone()),
            };
            if moved {
                if let Some(folder) = folder.as_ref().filter(|f| f.kind == WorkspaceKind::Folder) {
                    if !new_parent.is_empty() && within(state, &folder.id, &new_parent) {
                        return Err(Error::bad_request(format!("'{}' cannot move into itself", folder.name)));
                    }
                }
            }
            let new_name = match &folder {
                Some(f) if folder_changes => name.clone().unwrap_or_else(|| f.name.clone()),
                Some(f) => f.name.clone(),
                None => name.clone().unwrap_or_else(|| e.name.clone()),
            };
            if !stored(&e) && (moved || (folder.is_none() && new_name != e.name)) {
                check_unique(state, &new_owner_of_place, &new_parent, &new_name, &e.id)?;
            }
            let link = match (&revision, &e.link) {
                (None, link) => link.clone(),
                (Some(key), Some(link)) if part_link(link) => {
                    Some(WorkspaceLink { revision_id: pin_of(state, &link.part_id, key)?, ..link.clone() })
                }
                (Some(_), _) => return Err(Error::bad_request(format!("'{}' is not a link to a part; only that has a revision", e.name))),
            };
            let at = now();
            if let (Some(folder), true) = (&folder, folder_changes) {
                let stored = entry_mut(state, &folder.id)?;
                if let Some(n) = &name {
                    stored.name = n.clone();
                }
                if let Some(to) = &to {
                    stored.owner = to.clone();
                }
                stored.modified_at = at;
                let new_name = stored.name.clone();
                // Each link carries its name too.
                let links: Vec<String> = state.workspace.iter().filter(|l| linked_object(l) == Some(folder.id.as_str())).map(|l| l.id.clone()).collect();
                for l in links {
                    entry_mut(state, &l)?.name = new_name.clone();
                }
            }
            if !stored(&e) && (moved || link != e.link || (folder.is_none() && new_name != e.name)) {
                let stored = entry_mut(state, &id)?;
                stored.parent = new_parent;
                stored.owner = new_owner_of_place;
                stored.name = new_name;
                stored.link = link;
                stored.modified_at = at;
            }
            let stored = entry(state, &id)?.clone();
            Ok(view(state, &stored))
        })
    }

    /// Remove an entry. A link — to a part, a folder, a file or an
    /// attachment — goes, and never touches what it points at. A folder or
    /// file itself (by its own id) is deleted only by its owner, and every
    /// link to it goes with it: a folder only empty unless `recursive`, which
    /// removes the links in it (what they point at stays); a file with every
    /// version. A file's blobs go when nothing else references them. Answers
    /// the ids removed.
    pub fn delete_entry(&self, user: &User, id: &str, recursive: bool) -> Result<Vec<String>, Error> {
        let (user, id) = (user.clone(), id.to_string());
        let root = self.root().to_path_buf();
        self.mutate(move |state| {
            let e = owned(state, &user, &id)?.clone();
            let mut doomed = BTreeSet::from([e.id.clone()]);
            if e.kind == WorkspaceKind::Folder {
                let inside: Vec<String> = state.workspace.iter().filter(|c| c.parent == e.id).map(|c| c.id.clone()).collect();
                if !inside.is_empty() && !recursive {
                    return Err(Error::conflict(format!(
                        "the folder '{}' holds {} entries — empty it, or delete it with everything in it",
                        e.name,
                        inside.len()
                    )));
                }
                doomed.extend(inside);
            }
            if stored(&e) {
                doomed.extend(state.workspace.iter().filter(|l| linked_object(l) == Some(e.id.as_str())).map(|l| l.id.clone()));
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
        let (name, version, file) = self.read(|state| {
            let e = owned_file(state, user, id)?;
            let found = if ask.version == 0 { e.versions.last() } else { e.versions.iter().find(|v| v.version == ask.version) };
            let found = found.ok_or_else(|| Error::not_found("version"))?;
            Ok::<_, Error>((e.name.clone(), found.clone(), e.id.clone()))
        })?;
        let describe = Describe {
            name: if ask.name.trim().is_empty() { name } else { ask.name.clone() },
            media_type: version.media_type.clone(),
            kind: ask.kind.clone(),
            note: ask.note.clone(),
        };
        let (entry_id, sha) = (file, version.sha256.clone());
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
