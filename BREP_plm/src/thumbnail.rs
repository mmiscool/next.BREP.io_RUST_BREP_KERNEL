//! Revision thumbnails (P9): a small PNG per revision, made by a CAD client
//! when it saves and by the bake worker for generated members. The server
//! never links the kernel, so it never renders one; it keeps what it is sent.
//!
//! - **Keyed by the document.** A thumbnail records the revision's
//!   `content_hash` when it was made, and the renderer's version. It is
//!   *current* only while that hash is still the revision's; a save makes it
//!   stale, and a stale one is never served as the revision's picture (the
//!   GET answers `204`, the listings leave it out). It is kept, not deleted,
//!   until a new one replaces it: the next save's upload does.
//! - **Stored like an attachment.** The bytes are a content-addressed blob in
//!   `<data>/blobs/` ([`crate::attach`]), referenced from the revision, so
//!   [`crate::attach::referenced`] and the backup see it and it goes when the
//!   last reference does.
//! - **Who may upload.** An author, or the bake worker (its group and its
//!   token scope). No lock and no editable revision are needed: the upload
//!   says which document it pictures, and the hash check is the whole rule,
//!   so a newer renderer can re-picture a released revision.
//! - **A part's thumbnail** is its newest revision's that is current.

use crate::attach::{self, Staged};
use crate::db::{now, Db, State};
use crate::model::{Part, Revision, Thumbnail, User};
use crate::Error;

/// The largest PNG kept, in bytes.
pub const MAX_BYTES: u64 = 1024 * 1024;

/// The largest edge, in pixels.
pub const MAX_EDGE: u32 = 1024;

/// The media type every thumbnail is.
pub const MEDIA_TYPE: &str = "image/png";

/// A PNG's width and height from its header, or why the bytes are not one.
pub fn png_size(bytes: &[u8]) -> Result<(u32, u32), Error> {
    const SIGNATURE: [u8; 8] = [0x89, b'P', b'N', b'G', b'\r', b'\n', 0x1a, b'\n'];
    if bytes.len() < 24 || bytes[..8] != SIGNATURE || &bytes[12..16] != b"IHDR" {
        return Err(Error::bad_request("a thumbnail must be a PNG image"));
    }
    let word = |at: usize| u32::from_be_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]]);
    let (width, height) = (word(16), word(20));
    if width == 0 || height == 0 || width > MAX_EDGE || height > MAX_EDGE {
        return Err(Error::bad_request(format!(
            "a thumbnail is at most {MAX_EDGE}×{MAX_EDGE} pixels; this one is {width}×{height}"
        )));
    }
    Ok((width, height))
}

/// The revision's thumbnail if it pictures the document as it is now.
pub fn current(revision: &Revision) -> Option<&Thumbnail> {
    revision
        .thumbnail
        .as_ref()
        .filter(|t| !revision.content_hash.is_empty() && t.content_hash == revision.content_hash)
}

/// A part's thumbnail: the newest revision (by creation, drafts included)
/// whose thumbnail is current.
pub fn of_part(part: &Part) -> Option<(&Revision, &Thumbnail)> {
    part.revisions.iter().rev().find_map(|r| current(r).map(|t| (r, t)))
}

/// The URL a page shows a part's thumbnail from, versioned by the blob so a
/// browser may cache it for good; empty when the part has none.
pub fn part_url(part: &Part) -> String {
    of_part(part).map(|(_, t)| format!("/api/parts/{}/thumbnail?v={}", crate::identity::segment(&part.id), t.sha256)).unwrap_or_default()
}

/// The same for one revision.
pub fn revision_url(part_id: &str, revision: &Revision) -> String {
    current(revision)
        .map(|t| format!("/api/parts/{}/revisions/{}/thumbnail?v={}", crate::identity::segment(part_id), crate::identity::segment(&revision.id), t.sha256))
        .unwrap_or_default()
}

/// What the uploader says the picture is of.
#[derive(Debug, Clone, Default)]
pub struct Claim {
    /// The document's `content_hash` the picture was made from.
    pub content_hash: String,
    pub renderer: String,
}

/// A portable model preview staged before the document transaction. Import and
/// ordinary saves use the same path, so an image never needs a second upload.
pub(crate) struct Embedded {
    staged: attach::Staged,
    width: u32,
    height: u32,
    renderer: String,
}

impl Embedded {
    pub(crate) fn place(&self, root: &std::path::Path, content_hash: &str, user: &str) -> Result<Thumbnail, Error> {
        attach::place(root, &self.staged)?;
        Ok(Thumbnail {
            sha256: self.staged.sha256.clone(), size: self.staged.size,
            width: self.width, height: self.height, content_hash: content_hash.into(),
            renderer: self.renderer.clone(), uploaded_by: user.into(), uploaded_at: now(),
        })
    }
}

pub(crate) fn embedded(root: &std::path::Path, body: &str) -> Result<Option<Embedded>, Error> {
    use base64::Engine;
    let Ok(document) = serde_json::from_str::<serde_json::Value>(body) else { return Ok(None) };
    let Some(image) = document.get("thumbnail").filter(|v| !v.is_null()) else { return Ok(None) };
    if image.get("mimeType").and_then(|v| v.as_str()) != Some(MEDIA_TYPE) {
        return Err(Error::bad_request("the embedded thumbnail must have mimeType image/png"));
    }
    let data = image.get("data").and_then(|v| v.as_str())
        .ok_or_else(|| Error::bad_request("the embedded thumbnail needs base64 PNG data"))?;
    if data.len() as u64 > MAX_BYTES.div_ceil(3) * 4 {
        return Err(Error::bad_request("the embedded thumbnail exceeds 1 MiB"));
    }
    let bytes = base64::engine::general_purpose::STANDARD.decode(data)
        .map_err(|_| Error::bad_request("the embedded thumbnail is not valid base64"))?;
    if bytes.len() as u64 > MAX_BYTES { return Err(Error::bad_request("the embedded thumbnail exceeds 1 MiB")); }
    let (width, height) = png_size(&bytes)?;
    let renderer = image.get("renderer").and_then(|v| v.as_str()).unwrap_or("embedded-png").trim();
    if renderer.is_empty() || renderer.len() > 64 { return Err(Error::bad_request("invalid embedded thumbnail renderer")); }
    let mut upload = attach::Upload::start(root, MAX_BYTES).map_err(Error::internal)?;
    upload.write(&bytes)?;
    Ok(Some(Embedded { staged: upload.finish()?, width, height, renderer: renderer.into() }))
}

impl Db {
    /// Keep `staged` (already checked by [`png_size`]) as the thumbnail of
    /// `revision_key` of `part_key`. Refused `409` when the revision's
    /// document is not the one pictured.
    pub fn put_thumbnail(
        &self,
        user: &User,
        part_key: &str,
        revision_key: &str,
        staged: Staged,
        (width, height): (u32, u32),
        claim: &Claim,
    ) -> Result<Thumbnail, Error> {
        if !user.can_author() && !user.can_bake() {
            return Err(Error::forbidden("a thumbnail is uploaded by an author or the bake worker"));
        }
        let renderer = claim.renderer.trim().to_string();
        if renderer.is_empty() || renderer.len() > 64 {
            return Err(Error::bad_request("name the renderer that made the thumbnail (renderer=…, at most 64 characters)"));
        }
        let root = self.root().to_path_buf();
        let user_id = user.id.clone();
        let (part_key, revision_key, claimed) = (part_key.to_string(), revision_key.to_string(), claim.content_hash.trim().to_string());
        self.mutate(move |state| {
            let part = state.part_by_id_or_number(&part_key).ok_or_else(|| Error::not_found("part"))?;
            let (part_id, number) = (part.id.clone(), part.number.clone());
            let revision = crate::bom::find_revision(part, &revision_key).ok_or_else(|| Error::not_found("revision"))?;
            let revision_id = revision.id.clone();
            if revision.content_hash.is_empty() {
                return Err(Error::conflict(format!("{number} revision {} has no document to picture", revision.label)));
            }
            if claimed != revision.content_hash {
                return Err(Error::conflict(format!(
                    "the thumbnail pictures document {}, but {number} revision {} is now {} — render it again",
                    short(&claimed),
                    revision.label,
                    short(&revision.content_hash)
                )));
            }
            attach::place(&root, &staged)?;
            let thumbnail = Thumbnail {
                sha256: staged.sha256.clone(),
                size: staged.size,
                width,
                height,
                content_hash: claimed,
                renderer,
                uploaded_by: user_id,
                uploaded_at: now(),
            };
            let part = state.parts.get_mut(&part_id).ok_or_else(|| Error::not_found("part"))?;
            let revision = part.revision_mut(&revision_id).ok_or_else(|| Error::not_found("revision"))?;
            let old = revision.thumbnail.replace(thumbnail.clone());
            if let Some(old) = old.filter(|o| o.sha256 != thumbnail.sha256) {
                attach::remove_if_unreferenced(state, &root, &old.sha256)?;
            }
            // A separate CAD thumbnail upload can land after the document
            // save. Readers must hear about both while checkout stays held.
            state.touch(crate::identity::document_key(&part_id, &revision_id));
            Ok(thumbnail)
        })
    }

    /// The thumbnail to serve for a revision (`revision_key` non-empty) or a
    /// part: `None` when there is no current one. `Err` only for an unknown
    /// part or revision.
    pub fn thumbnail(&self, part_key: &str, revision_key: &str) -> Result<Option<(String, Thumbnail)>, Error> {
        self.read(|state: &State| {
            let part = state.part_by_id_or_number(part_key).ok_or_else(|| Error::not_found("part"))?;
            if revision_key.is_empty() {
                return Ok(of_part(part).map(|(r, t)| (r.label.clone(), t.clone())));
            }
            let revision = crate::bom::find_revision(part, revision_key).ok_or_else(|| Error::not_found("revision"))?;
            Ok(current(revision).map(|t| (revision.label.clone(), t.clone())))
        })
    }
}

fn short(hash: &str) -> &str {
    if hash.is_empty() { "(none)" } else { &hash[..hash.len().min(12)] }
}
