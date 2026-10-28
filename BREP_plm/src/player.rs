//! Serving the manufacturing instructions player per instructions revision.
//!
//! The player page is TRUSTED code: the template of the `BREP_instructions`
//! crate ([`brep_instructions::player`]), served under a policy of its own
//! with a nonce per response. What a CAD client uploads is only DATA (the
//! crate's `PlayerData` JSON: steps, meshes, poses). Authors never
//! upload HTML: a page with their script on the PLM's origin would run with
//! every viewer's session.
//!
//! - **Keyed by the document**, like a thumbnail ([`crate::thumbnail`]): the
//!   upload names the revision's `content_hash` it was made from, and the
//!   data is served only while that is still the revision's. An author
//!   uploads it; no lock is needed, so a newer player can re-make a
//!   released revision's data.
//! - **Stored as a blob** of the attachment store ([`crate::attach`]),
//!   referenced from the revision and in every backup.
//! - **`/instructions/<number>/<rev>`** serves the page to anyone signed in,
//!   `?mode=shop|customer`, and `&job=JOB…` in shop mode signs that job's
//!   steps ([`crate::jobs`]).
//! - **Customer links** (`/s/<token>`) serve a RELEASED revision's player in
//!   customer mode to anyone holding the link: no sign-in, no sign-off, and
//!   nothing but that revision's player data. The token is shown once and
//!   kept only as its SHA-256; a link is revoked by deleting it.

use crate::attach::{self, Staged};
use crate::db::{now, Db, State};
use crate::model::{DocumentClass, Lifecycle, Part, PlayerArtifact, Revision, Share, User};
use crate::Error;

/// The largest player data, in bytes (meshes for a few hundred parts).
pub const MAX_BYTES: u64 = 64 * 1024 * 1024;

/// The policy the player page is served under: its one script and one style
/// block by nonce, pictures from the data or this server, and requests only
/// back to this server.
pub fn policy(nonce: &str) -> String {
    format!(
        "default-src 'none'; script-src 'nonce-{nonce}'; style-src 'nonce-{nonce}'; img-src 'self' data:; \
         connect-src 'self'; base-uri 'none'; form-action 'none'; frame-ancestors 'none'"
    )
}

/// The data to serve for a revision: present and made from its current document.
pub fn current(revision: &Revision) -> Option<&PlayerArtifact> {
    revision.player.as_ref().filter(|p| !revision.content_hash.is_empty() && p.content_hash == revision.content_hash)
}

/// The id a share is listed and revoked by: the first 12 hex of its hash.
fn share_id(token_sha256: &str) -> String {
    token_sha256[..12].to_string()
}

/// A part whose revisions carry player data: manufacturing instructions (the
/// instructions player) or an ordinary part (a motion study's player, made
/// by the CAD app from the assembly's active study). Families and templates
/// have none.
fn playable<'a>(state: &'a State, part_key: &str, revision_key: &str) -> Result<(&'a Part, &'a Revision), Error> {
    let part = state.part_by_id_or_number(part_key).ok_or_else(|| Error::not_found("part"))?;
    if !matches!(part.document_class, DocumentClass::Instructions | DocumentClass::Normal) {
        return Err(Error::bad_request(format!("{} has no player: it is a {}", part.number, part.document_class.as_str())));
    }
    let revision = crate::bom::find_revision(part, revision_key).ok_or_else(|| Error::not_found("revision"))?;
    Ok((part, revision))
}

fn instructions<'a>(state: &'a State, part_key: &str, revision_key: &str) -> Result<(&'a Part, &'a Revision), Error> {
    let part = state.part_by_id_or_number(part_key).ok_or_else(|| Error::not_found("part"))?;
    if part.document_class != DocumentClass::Instructions {
        return Err(Error::bad_request(format!("{} is not manufacturing instructions", part.number)));
    }
    let revision = crate::bom::find_revision(part, revision_key).ok_or_else(|| Error::not_found("revision"))?;
    Ok((part, revision))
}

impl Db {
    /// Keep `staged` (already checked to be player data) as the player data
    /// of a revision — instructions, or an ordinary part's motion study —
    /// made from document `claimed`.
    pub fn put_player_data(&self, user: &User, part_key: &str, revision_key: &str, staged: Staged, claimed: &str) -> Result<PlayerArtifact, Error> {
        if !user.can_author() {
            return Err(Error::forbidden("player data is uploaded by an author"));
        }
        let root = self.root().to_path_buf();
        let (part_key, revision_key, claimed, user_id) = (part_key.to_string(), revision_key.to_string(), claimed.trim().to_string(), user.id.clone());
        self.mutate(move |state| {
            let (part, revision) = playable(state, &part_key, &revision_key)?;
            let (part_id, revision_id) = (part.id.clone(), revision.id.clone());
            if revision.content_hash.is_empty() {
                return Err(Error::conflict(format!("{} revision {} has no document to play", part.number, revision.label)));
            }
            if claimed != revision.content_hash {
                return Err(Error::conflict(format!(
                    "the player data was made from another document than {} revision {} holds now: make it again",
                    part.number, revision.label
                )));
            }
            attach::place(&root, &staged)?;
            let artifact = PlayerArtifact { sha256: staged.sha256.clone(), size: staged.size, content_hash: claimed, uploaded_by: user_id, uploaded_at: now() };
            let revision = state.parts.get_mut(&part_id).and_then(|p| p.revision_mut(&revision_id)).ok_or_else(|| Error::not_found("revision"))?;
            let old = revision.player.replace(artifact.clone());
            if let Some(old) = old.filter(|o| o.sha256 != artifact.sha256) {
                attach::remove_if_unreferenced(state, &root, &old.sha256)?;
            }
            Ok(artifact)
        })
    }

    /// A revision's current player data (instructions, or an ordinary part's
    /// motion study), with its part and revision ids; `None` when there is
    /// none or it is stale.
    pub fn player_data(&self, part_key: &str, revision_key: &str) -> Result<(String, String, Option<PlayerArtifact>), Error> {
        self.read(|state| {
            let (part, revision) = playable(state, part_key, revision_key)?;
            Ok((part.id.clone(), revision.id.clone(), current(revision).cloned()))
        })
    }

    /// The same for manufacturing instructions only: the `/instructions/…`
    /// page refuses an ordinary part (its study plays at `/motion/…`).
    pub fn instructions_player_data(&self, part_key: &str, revision_key: &str) -> Result<(String, String, Option<PlayerArtifact>), Error> {
        self.read(|state| {
            let (part, revision) = instructions(state, part_key, revision_key)?;
            Ok((part.id.clone(), revision.id.clone(), current(revision).cloned()))
        })
    }

    /// Make a customer link to a released instructions revision: the share
    /// and its token, which is never shown again.
    pub fn create_share(&self, user: &User, part_key: &str, revision_key: &str, label: &str) -> Result<(Share, String), Error> {
        if !user.can_author() {
            return Err(Error::forbidden("a customer link is made by an author"));
        }
        let label = label.trim().to_string();
        if label.chars().count() > 200 {
            return Err(Error::bad_request("a link's label is at most 200 characters"));
        }
        let token = format!("{}{}", crate::auth::random_token(16), crate::auth::random_token(16));
        let token_sha256 = crate::auth::content_hash(&token);
        let (part_key, revision_key, user_id) = (part_key.to_string(), revision_key.to_string(), user.id.clone());
        let share = self.mutate(move |state| {
            let (part, revision) = instructions(state, &part_key, &revision_key)?;
            if revision.lifecycle != Lifecycle::Released {
                return Err(Error::conflict(format!(
                    "{} revision {} is {}: customers get released instructions only",
                    part.number,
                    revision.label,
                    revision.lifecycle.as_str()
                )));
            }
            let (part_id, revision_id) = (part.id.clone(), revision.id.clone());
            let share = Share { id: share_id(&token_sha256), token_sha256, label, created_by: user_id, created_at: now() };
            let revision = state.parts.get_mut(&part_id).and_then(|p| p.revision_mut(&revision_id)).ok_or_else(|| Error::not_found("revision"))?;
            revision.shares.push(share.clone());
            Ok(share)
        })?;
        Ok((share, token))
    }

    pub fn revoke_share(&self, part_key: &str, revision_key: &str, id: &str) -> Result<(), Error> {
        let (part_key, revision_key, id) = (part_key.to_string(), revision_key.to_string(), id.to_string());
        self.mutate(move |state| {
            let (part, revision) = instructions(state, &part_key, &revision_key)?;
            if !revision.shares.iter().any(|s| s.id == id) {
                return Err(Error::not_found("customer link"));
            }
            let (part_id, revision_id) = (part.id.clone(), revision.id.clone());
            let revision = state.parts.get_mut(&part_id).and_then(|p| p.revision_mut(&revision_id)).ok_or_else(|| Error::not_found("revision"))?;
            revision.shares.retain(|s| s.id != id);
            Ok(())
        })
    }

    /// What a customer link opens: its revision's current player data, while
    /// the revision is released. `None` for an unknown or revoked token.
    pub fn shared_player(&self, token: &str) -> Option<(String, Option<PlayerArtifact>)> {
        if token.len() < 40 || !token.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_') {
            return None;
        }
        let hash = crate::auth::content_hash(token);
        self.read(|state| {
            state.parts.iter().filter(|p| p.document_class == DocumentClass::Instructions).find_map(|part| {
                part.revisions
                    .iter()
                    .find(|r| r.lifecycle == Lifecycle::Released && r.shares.iter().any(|s| s.token_sha256 == hash))
                    .map(|r| (format!("{} rev {}", part.number, r.label), current(r).cloned()))
            })
        })
    }
}
