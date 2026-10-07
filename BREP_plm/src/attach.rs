//! Attachments: files on a part or on one of its revisions.
//!
//! # Where the bytes are
//!
//! `<data>/blobs/<first two hex>/<sha256>` — content-addressed, so the same
//! file attached twice (to two parts, or carried to a new revision) is stored
//! once. An upload streams into `<data>/blobs/tmp/` while it is hashed and
//! counted against the size limit, and is never held in memory whole; it is
//! moved to its content address inside the store's write lock, in the same
//! change that adds its reference. A blob is removed inside the write lock
//! too, by the change that drops its last reference, so an upload and a
//! delete of the same content can never cross.
//!
//! # The rules (attachment defaults)
//!
//! - A PART attachment follows the part, like sourcing: any author may add or
//!   remove one at any time, released revisions or not.
//! - A REVISION attachment is frozen with the revision: it is added or removed
//!   only while the revision is in work and nobody else holds its lock. A new
//!   revision starts with references to the same files.
//! - Download is for anyone signed in. A small list of media types that a
//!   browser renders without running anything (raster images and PDF) may be
//!   shown inline; everything else is served as a download. The page's
//!   Content-Security-Policy and `nosniff` stay on every response.

use std::collections::BTreeSet;
use std::io;
use std::path::{Path, PathBuf};

use serde::Serialize;
use sha2::{Digest, Sha256};

use crate::db::{now, Db, State};
use crate::model::{Attachment, Part, User};
use crate::Error;

/// The kinds an attachment may be filed as.
pub const KINDS: [&str; 5] = ["datasheet", "drawing", "spec", "image", "other"];

/// Media types a browser may show inline: they render without script.
/// SVG is an image that can carry script, so it is NOT here.
pub const INLINE_TYPES: [&str; 5] = ["image/png", "image/jpeg", "image/gif", "image/webp", "application/pdf"];

const MAX_NAME: usize = 200;
const MAX_NOTE: usize = 500;

/// The blob directory under a data directory.
pub fn blob_root(data: &Path) -> PathBuf {
    data.join("blobs")
}

/// Where the blob with this hash lives.
pub fn blob_path(data: &Path, sha256: &str) -> PathBuf {
    blob_root(data).join(&sha256[..2]).join(sha256)
}

/// Where uploads stream to before they have a name.
pub fn staging_dir(data: &Path) -> PathBuf {
    blob_root(data).join("tmp")
}

/// Whether `text` is a hex SHA-256 — the only thing ever joined to a blob path.
pub fn is_sha256(text: &str) -> bool {
    text.len() == 64 && text.bytes().all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}

/// An upload that has been written, hashed and counted, and waits in the
/// staging directory for its reference. Dropping it removes the file, so an
/// upload that is refused or abandoned leaves nothing behind.
#[derive(Debug)]
pub struct Staged {
    pub path: PathBuf,
    pub sha256: String,
    pub size: u64,
}

impl Drop for Staged {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.path);
    }
}

/// Hashes and counts an upload as it streams to the staging directory.
pub struct Upload {
    file: std::fs::File,
    path: PathBuf,
    hasher: Sha256,
    size: u64,
    limit: u64,
    done: bool,
}

impl Upload {
    pub fn start(data: &Path, limit: u64) -> io::Result<Upload> {
        let dir = staging_dir(data);
        std::fs::create_dir_all(&dir)?;
        let path = dir.join(format!("{}.part", crate::auth::new_id()));
        let file = std::fs::File::create(&path)?;
        Ok(Upload { file, path, hasher: Sha256::new(), size: 0, limit, done: false })
    }

    /// Append a chunk. Over the limit is a `413`, and the partial file goes.
    pub fn write(&mut self, chunk: &[u8]) -> Result<(), Error> {
        use std::io::Write;
        self.size += chunk.len() as u64;
        if self.size > self.limit {
            return Err(Error {
                status: axum::http::StatusCode::PAYLOAD_TOO_LARGE,
                message: format!("the file is over the {} MB attachment limit", self.limit / (1024 * 1024)),
            });
        }
        self.hasher.update(chunk);
        self.file.write_all(chunk).map_err(Error::internal)
    }

    /// The file is complete: flush it and hand it on.
    pub fn finish(self) -> Result<Staged, Error> { self.finish_with_empty(false) }

    pub(crate) fn finish_with_empty(mut self, allow_empty: bool) -> Result<Staged, Error> {
        if self.size == 0 && !allow_empty {
            return Err(Error::bad_request("the file is empty"));
        }
        self.file.sync_all().map_err(Error::internal)?;
        self.done = true;
        let sha256 = format!("{:x}", std::mem::take(&mut self.hasher).finalize());
        Ok(Staged { path: std::mem::take(&mut self.path), sha256, size: self.size })
    }
}

impl Drop for Upload {
    fn drop(&mut self) {
        if !self.done {
            let _ = std::fs::remove_file(&self.path);
        }
    }
}

/// What the uploader says about a file.
#[derive(Debug, Clone, Default)]
pub struct Describe {
    pub name: String,
    pub media_type: String,
    pub kind: String,
    pub note: String,
}

/// A file name that is safe to store and to put in a `Content-Disposition`:
/// the last path segment, no control characters or quotes, not empty.
pub fn clean_name(raw: &str) -> Result<String, Error> {
    let last = raw.rsplit(['/', '\\']).next().unwrap_or("").trim();
    let name: String = last.chars().filter(|c| !c.is_control() && *c != '"').collect();
    let name = name.trim().to_string();
    if name.is_empty() || name == "." || name == ".." {
        return Err(Error::bad_request("an attachment needs a file name"));
    }
    if name.chars().count() > MAX_NAME {
        return Err(Error::bad_request(format!("a file name is at most {MAX_NAME} characters")));
    }
    Ok(name)
}

/// A media type in `type/subtype` form, lower case, parameters dropped; else
/// `application/octet-stream`.
pub fn clean_media_type(raw: &str) -> String {
    let base = raw.split(';').next().unwrap_or("").trim().to_ascii_lowercase();
    let ok = |part: &str| {
        !part.is_empty() && part.bytes().all(|b| b.is_ascii_alphanumeric() || b"!#$&-^_.+".contains(&b))
    };
    match base.split_once('/') {
        Some((a, b)) if ok(a) && ok(b) => base,
        _ => "application/octet-stream".to_string(),
    }
}

fn clean_kind(raw: &str, media_type: &str) -> Result<String, Error> {
    let kind = raw.trim().to_ascii_lowercase();
    if kind.is_empty() {
        return Ok(if media_type.starts_with("image/") { "image" } else { "other" }.to_string());
    }
    if KINDS.contains(&kind.as_str()) {
        Ok(kind)
    } else {
        Err(Error::bad_request(format!("'{kind}' is not a kind of attachment — {}", KINDS.join(", "))))
    }
}

/// Whether a response may show this type inline.
pub fn inline_ok(media_type: &str) -> bool {
    INLINE_TYPES.contains(&media_type)
}

/// Where an attachment hangs, found by its id.
#[derive(Debug, Clone, Serialize)]
pub struct Located {
    pub attachment: Attachment,
    pub part_id: String,
    pub number: String,
    /// Empty for a part attachment.
    pub revision_id: String,
    pub revision_label: String,
}

/// Find an attachment by id anywhere in the store.
pub fn locate(state: &State, id: &str) -> Option<Located> {
    for part in state.parts.iter() {
        if let Some(a) = part.attachments.iter().find(|a| a.id == id) {
            return Some(Located {
                attachment: a.clone(),
                part_id: part.id.clone(),
                number: part.number.clone(),
                revision_id: String::new(),
                revision_label: String::new(),
            });
        }
        for revision in &part.revisions {
            if let Some(a) = revision.attachments.iter().find(|a| a.id == id) {
                return Some(Located {
                    attachment: a.clone(),
                    part_id: part.id.clone(),
                    number: part.number.clone(),
                    revision_id: revision.id.clone(),
                    revision_label: revision.label.clone(),
                });
            }
        }
    }
    None
}

/// Resolve a file in a particular part/revision; carried attachments can share ids.
pub fn locate_scoped(state: &State, id: &str, part_key: &str, revision_key: &str) -> Option<Located> {
    if part_key.is_empty() { return locate(state, id); }
    let part = state.part_by_id_or_number(part_key)?;
    let revision = if revision_key.is_empty() { None } else { Some(crate::bom::find_revision(part, revision_key)?) };
    let files = revision.map(|r| &r.attachments).unwrap_or(&part.attachments);
    let attachment = files.iter().find(|a| a.id == id)?.clone();
    Some(Located { attachment, part_id: part.id.clone(), number: part.number.clone(),
        revision_id: revision.map(|r| r.id.clone()).unwrap_or_default(),
        revision_label: revision.map(|r| r.label.clone()).unwrap_or_default() })
}

pub fn attachment_writable(state: &State, user: &User, found: &Located) -> Result<(), Error> {
    if !user.can_author() { return Err(Error::forbidden("editing an attachment needs the author group")); }
    let part = state.part(&found.part_id).ok_or_else(|| Error::not_found("part"))?;
    if !found.revision_id.is_empty() { revision_writable(state, user, part, &found.revision_id)?; }
    Ok(())
}

/// Every blob the store references: attachments, revision thumbnails
/// ([`crate::thumbnail`]), and every version of every workspace file
/// ([`crate::workspace`]).
pub fn referenced(state: &State) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    for part in state.parts.iter() {
        out.extend(part.attachments.iter().map(|a| a.sha256.clone()));
        for revision in &part.revisions {
            out.extend(revision.attachments.iter().map(|a| a.sha256.clone()));
            out.extend(revision.thumbnail.iter().map(|t| t.sha256.clone()));
        }
    }
    for entry in state.workspace.iter() {
        out.extend(entry.versions.iter().map(|v| v.sha256.clone()));
    }
    out
}

/// Remove the blob `sha256` if nothing in `state` references it any more.
/// Called inside the write lock by the change that dropped a reference.
pub fn remove_if_unreferenced(state: &State, root: &Path, sha256: &str) -> Result<(), Error> {
    if !is_sha256(sha256) || referenced(state).contains(sha256) {
        return Ok(());
    }
    match std::fs::remove_file(blob_path(root, sha256)) {
        Err(error) if error.kind() != io::ErrorKind::NotFound => Err(Error::internal(error)),
        _ => Ok(()),
    }
}

/// Move a staged upload to its content address, unless that blob is already
/// there. Inside the write lock, in the change that adds its reference.
pub fn place(root: &Path, staged: &Staged) -> Result<(), Error> {
    let target = blob_path(root, &staged.sha256);
    if !target.exists() {
        std::fs::create_dir_all(target.parent().expect("a blob has a directory")).map_err(Error::internal)?;
        std::fs::rename(&staged.path, &target).map_err(Error::internal)?;
    }
    Ok(())
}

/// The refusal for changing a revision's files, if any: it must be in work
/// and not checked out by someone else.
fn revision_writable(state: &State, user: &User, part: &Part, revision_id: &str) -> Result<(), Error> {
    let revision = part.revision(revision_id).ok_or_else(|| Error::not_found("revision"))?;
    if !revision.lifecycle.is_editable() {
        return Err(Error::conflict(format!(
            "{} revision {} is {} — its files are frozen with it; attach to a new revision",
            part.number,
            revision.label,
            revision.lifecycle.as_str()
        )));
    }
    if let Some(lock) = revision.lock.as_ref().filter(|l| l.user_id != user.id) {
        let holder = state.user(&lock.user_id).map(|u| u.username.clone()).unwrap_or_else(|| "another user".into());
        return Err(Error::conflict(format!(
            "{} revision {} is checked out by {holder}",
            part.number, revision.label
        )));
    }
    Ok(())
}

impl Db {
    /// The data directory's blob path for `sha256`.
    pub fn blob_path(&self, sha256: &str) -> PathBuf {
        blob_path(self.root(), sha256)
    }

    /// Begin streaming an upload, with the server's size limit.
    pub fn start_upload(&self) -> Result<Upload, Error> {
        Upload::start(self.root(), self.security().config.attachment_limit()).map_err(Error::internal)
    }

    /// Attach a staged upload to a part (`revision` empty) or to one of its
    /// revisions.
    pub fn attach(
        &self,
        user: &User,
        part_key: &str,
        revision_key: &str,
        staged: Staged,
        describe: &Describe,
    ) -> Result<Attachment, Error> {
        let (sha256, size) = (staged.sha256.clone(), staged.size);
        self.attach_blob(user, part_key, revision_key, &sha256, size, Some(staged), describe, |_| Ok(()))
    }

    /// [`Db::attach`] for bytes that are either a staged upload or, with
    /// `staged` absent, a blob the store already holds (a workspace file
    /// promoted to an attachment). `check` runs inside the lock first, so a
    /// caller can prove its blob is still referenced at the write.
    #[allow(clippy::too_many_arguments)]
    pub fn attach_blob(
        &self,
        user: &User,
        part_key: &str,
        revision_key: &str,
        sha256: &str,
        size: u64,
        staged: Option<Staged>,
        describe: &Describe,
        check: impl FnOnce(&State) -> Result<(), Error>,
    ) -> Result<Attachment, Error> {
        if !user.can_author() {
            return Err(Error::forbidden("attaching a file needs the author group"));
        }
        let name = clean_name(&describe.name)?;
        let media_type = clean_media_type(&describe.media_type);
        let kind = clean_kind(&describe.kind, &media_type)?;
        let note = describe.note.trim().to_string();
        if note.chars().count() > MAX_NOTE {
            return Err(Error::bad_request(format!("a note is at most {MAX_NOTE} characters")));
        }
        if !is_sha256(sha256) {
            return Err(Error::internal("an attachment names no valid blob"));
        }
        let root = self.root().to_path_buf();
        let user_id = user.id.clone();
        let user = user.clone();
        self.mutate(move |state| {
            check(state)?;
            let part = state.part_by_id_or_number(part_key).ok_or_else(|| Error::not_found("part"))?;
            let part_id = part.id.clone();
            let revision_id = if revision_key.trim().is_empty() {
                None
            } else {
                let r = crate::bom::find_revision(part, revision_key).ok_or_else(|| Error::not_found("revision"))?;
                Some(r.id.clone())
            };
            if let Some(revision_id) = &revision_id {
                revision_writable(state, &user, part, revision_id)?;
            }
            let attachment = Attachment {
                id: crate::auth::new_id(),
                name,
                media_type,
                size,
                sha256: sha256.to_string(),
                kind,
                note,
                uploaded_by: user_id,
                uploaded_at: now(),
            };
            // The bytes reach their address inside the lock, so no delete of
            // the same content can remove them between here and the commit.
            match &staged {
                Some(staged) => place(&root, staged)?,
                None if !blob_path(&root, sha256).exists() => {
                    return Err(Error::internal(format!("the stored file {sha256} is missing")))
                }
                None => {}
            }
            let part = state.parts.get_mut(&part_id).ok_or_else(|| Error::not_found("part"))?;
            match &revision_id {
                None => part.attachments.push(attachment.clone()),
                Some(id) => {
                    let revision = part.revision_mut(id).ok_or_else(|| Error::not_found("revision"))?;
                    revision.attachments.push(attachment.clone());
                    revision.modified_at = now();
                }
            }
            Ok(attachment)
        })
    }

    /// Replace exactly one attachment reference atomically. A fresh id protects
    /// copies carried into other revisions; their original bytes remain intact.
    pub fn replace_attachment(&self, user: &User, id: &str, part: &str, revision: &str,
        expected_hash: &str, staged: Staged) -> Result<Located, Error> {
        let root = self.root().to_path_buf();
        self.mutate(|state| {
            let found = locate_scoped(state, id, part, revision).ok_or_else(|| Error::not_found("attachment"))?;
            attachment_writable(state, user, &found)?;
            if found.attachment.sha256 != expected_hash { return Err(Error::conflict("This file changed since you opened it. Reopen it before saving.")); }
            let mut changed = found.clone();
            changed.attachment.id = crate::auth::new_id();
            changed.attachment.sha256 = staged.sha256.clone();changed.attachment.size = staged.size;
            changed.attachment.uploaded_by = user.id.clone();changed.attachment.uploaded_at = now();
            place(&root, &staged)?;
            let part = state.parts.get_mut(&found.part_id).ok_or_else(|| Error::not_found("part"))?;
            let files = if found.revision_id.is_empty() { &mut part.attachments } else {
                let r = part.revision_mut(&found.revision_id).ok_or_else(|| Error::not_found("revision"))?;
                r.modified_at = now();&mut r.attachments
            };
            let file = files.iter_mut().find(|a| a.id == id).ok_or_else(|| Error::not_found("attachment"))?;
            *file = changed.attachment.clone();
            remove_if_unreferenced(state, &root, &found.attachment.sha256)?;
            Ok(changed)
        })
    }

    /// Remove one attachment reference; the blob goes with its last one.
    pub fn detach(&self, user: &User, id: &str) -> Result<Located, Error> {
        if !user.can_author() {
            return Err(Error::forbidden("removing a file needs the author group"));
        }
        let root = self.root().to_path_buf();
        let user = user.clone();
        self.mutate(move |state| {
            let found = locate(state, id).ok_or_else(|| Error::not_found("attachment"))?;
            let part = state.part(&found.part_id).ok_or_else(|| Error::not_found("part"))?;
            if !found.revision_id.is_empty() {
                revision_writable(state, &user, part, &found.revision_id)?;
            }
            let part = state.parts.get_mut(&found.part_id).ok_or_else(|| Error::not_found("part"))?;
            if found.revision_id.is_empty() {
                part.attachments.retain(|a| a.id != id);
            } else {
                let revision = part.revision_mut(&found.revision_id).ok_or_else(|| Error::not_found("revision"))?;
                revision.attachments.retain(|a| a.id != id);
                revision.modified_at = now();
            }
            remove_if_unreferenced(state, &root, &found.attachment.sha256)?;
            Ok(found)
        })
    }

    /// Find an attachment by id.
    pub fn attachment(&self, id: &str) -> Option<Located> {
        self.read(|state| locate(state, id))
    }

    /// Remove blobs nothing references and uploads left in staging — what a
    /// crash between moving a blob and committing its reference can leave.
    /// For start-up, before the server takes requests (one process owns the
    /// directory, see [`crate::dirlock`]); returns how many files went.
    pub fn sweep_blobs(&self) -> io::Result<usize> {
        let root = blob_root(self.root());
        if !root.exists() {
            return Ok(0);
        }
        let keep = self.read(referenced);
        let staging = staging_dir(self.root());
        let mut removed = 0;
        if staging.exists() {
            for entry in std::fs::read_dir(&staging)? {
                std::fs::remove_file(entry?.path())?;
                removed += 1;
            }
        }
        for shard in std::fs::read_dir(&root)? {
            let shard = shard?.path();
            if !shard.is_dir() || shard == staging {
                continue;
            }
            for entry in std::fs::read_dir(&shard)? {
                let path = entry?.path();
                let name = path.file_name().and_then(|n| n.to_str()).unwrap_or("").to_string();
                if is_sha256(&name) && !keep.contains(&name) {
                    std::fs::remove_file(&path)?;
                    removed += 1;
                }
            }
        }
        Ok(removed)
    }
}
