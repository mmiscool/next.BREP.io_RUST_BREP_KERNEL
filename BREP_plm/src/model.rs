//! The PLM record types, and nothing that acts on them.
//!
//! The shape follows the four identity layers the architecture settled on:
//! **Part** (permanent, owns the number) → **Revision** (the released unit) →
//! the CAD **document** the revision points at → the **occurrence**, which
//! lives in the CAD document and never in this server.
//!
//! # Slots that are deliberately inert
//!
//! [`Origin`] and [`Part::external_ref`] are recorded and served but nothing
//! here branches on them yet. They are the seams the later
//! work needs — the KiCad importer stamps `Origin::Imported` and an
//! `external_ref` of `Timer:NE555D`, a family stamps `Origin::Generated` — and
//! a field added now costs one line, while a field added after parts exist
//! costs a migration. Behaviour built ON them is NOT here.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

/// Whole Unix seconds. The only time representation in the server: it
/// serializes as a number, sorts correctly, and needs no date library.
pub type Timestamp = u64;

// ===========================================================================
// Users, groups and sessions
// ===========================================================================

/// The built-in group names. Groups are plain strings on the user so an
/// operator can invent more without a migration; these four are the ones the
/// server itself checks.
pub mod groups {
    /// Full authority, including user and part-type administration.
    pub const ADMIN: &str = "admin";
    /// May release revisions and BREAK another user's lock — the "check-in
    /// group" in the architecture.
    pub const CHECKIN: &str = "checkin";
    /// May create parts and drafts, and check out their own work.
    pub const AUTHOR: &str = "author";
    /// Read-only.
    pub const VIEWER: &str = "viewer";
    /// A headless CAD worker: may take bake jobs from the queue and write
    /// their results (round 8). Nothing else — a worker account cannot
    /// create parts or release anything.
    pub const WORKER: &str = "worker";
}

/// One account. The password is never stored — only the PBKDF2 verifier in
/// [`crate::auth`], which carries its own salt and iteration count so a later
/// change of cost does not invalidate existing accounts.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct User {
    pub id: String,
    pub username: String,
    #[serde(default)]
    pub display_name: String,
    #[serde(default)]
    pub email: String,
    /// `pbkdf2$<iterations>$<b64 salt>$<b64 hash>` — see [`crate::auth`].
    pub password: String,
    #[serde(default)]
    pub groups: Vec<String>,
    #[serde(default = "yes")]
    pub active: bool,
    pub created_at: Timestamp,
}

fn yes() -> bool {
    true
}

impl User {
    /// Whether this user is in `group`. `admin` answers yes to everything, so
    /// no call site has to remember to check for it as well.
    pub fn in_group(&self, group: &str) -> bool {
        self.active
            && (self.groups.iter().any(|g| g == group)
                || self.groups.iter().any(|g| g == groups::ADMIN))
    }

    /// May create parts and revisions.
    pub fn can_author(&self) -> bool {
        self.in_group(groups::AUTHOR)
    }

    /// May release a revision and break another user's lock.
    pub fn can_checkin(&self) -> bool {
        self.in_group(groups::CHECKIN)
    }

    /// May administer users and part types.
    pub fn is_admin(&self) -> bool {
        self.in_group(groups::ADMIN)
    }

    /// May claim bake jobs and write their results.
    pub fn can_bake(&self) -> bool {
        self.in_group(groups::WORKER)
    }
}

/// A logged-in browser. Sessions live in the database file like everything
/// else, so a restarted server does not sign everyone out.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Session {
    pub token: String,
    pub user_id: String,
    pub created_at: Timestamp,
    pub last_seen: Timestamp,
    /// The CSRF token of this session (synchronizer pattern): handed to the
    /// page at sign-in and by `/api/me`, and required back in `X-CSRF-Token`
    /// on every request that changes anything. Empty for a session written
    /// before hardening; `/api/me` issues one on first use.
    #[serde(default)]
    pub csrf: String,
}

/// What an API token may do. A token is how a caller that is not a browser —
/// the CAD app, a bake worker, an organization's own script — signs in. It
/// acts as the user who owns it, narrowed by its scope; it never acts as more.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum TokenScope {
    /// Everything its owner may do, except managing tokens and passwords.
    #[default]
    Full,
    /// Only reads (`GET`).
    Read,
    /// Reads, the bake queue and the document store: what a headless CAD
    /// worker needs and nothing more.
    Worker,
}

impl TokenScope {
    pub fn as_str(self) -> &'static str {
        match self {
            TokenScope::Full => "full",
            TokenScope::Read => "read",
            TokenScope::Worker => "worker",
        }
    }
}

/// A bearer token for a non-browser caller. The secret is shown ONCE, when it
/// is created; the server keeps only its SHA-256 (the secret is 256 random
/// bits, so a slow hash buys nothing) and its first characters, so a person
/// can tell tokens apart in a list.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApiToken {
    pub id: String,
    pub user_id: String,
    /// What the owner called it ("bake worker on cad-01").
    pub name: String,
    #[serde(default)]
    pub scope: TokenScope,
    /// Hex SHA-256 of the whole secret.
    pub hash: String,
    /// The secret's first characters, e.g. `plm_3fQ9`.
    pub prefix: String,
    pub created_at: Timestamp,
    #[serde(default)]
    pub last_used_at: Option<Timestamp>,
    /// When it stops working; `None` never expires.
    #[serde(default)]
    pub expires_at: Option<Timestamp>,
}

// ===========================================================================
// Part types — the configurable numbering scheme
// ===========================================================================

/// A class of part with its OWN numbering scheme. `CPART` + 9 digits is the
/// seeded default, not a hard-coded rule: prefix and width are data, an
/// operator adds more types, and [`format_number`](Self::format_number) is the
/// one place a number is spelled.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PartType {
    /// Stable key used in URLs and on parts (`"component"`).
    pub id: String,
    /// What the UI shows (`"Component"`).
    pub name: String,
    /// Literal text before the digits (`"CPART"`).
    pub prefix: String,
    /// How many digits follow the prefix. The formatted number is ALWAYS this
    /// long — zero-padded — so every number of a type sorts as text.
    pub digits: u32,
    /// The next sequence value to hand out. Allocation bumps it. Used only in
    /// [`NumberMode::Counter`]; the other modes keep it so switching back to a
    /// counter resumes where it left off.
    pub next: u64,
    pub created_at: Timestamp,
    /// How this type's numbers are made. Absent in a file written before modes
    /// existed, which reads as the counter every type used to be.
    #[serde(default)]
    pub mode: NumberMode,
}

/// How a part type's numbers are made. In EVERY mode a number is unique
/// across all parts, not only within its type — that check lives inside the
/// store's write, not here.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum NumberMode {
    /// `prefix` + `digits` + an auto-incrementing counter. A typed number is
    /// refused: the counter owns the numbers of a counter type.
    #[default]
    Counter,
    /// Any number the user types — e.g. the ERP purchasing number of a library
    /// part.
    Free,
    /// A typed number that must match `regex` in full.
    Pattern { regex: String },
    /// The admin script at `script` (relative to the scripts directory)
    /// validates the typed number or allocates one, through its `partNumber`
    /// function.
    Script { script: String },
}

impl NumberMode {
    pub fn kind(&self) -> &'static str {
        match self {
            NumberMode::Counter => "counter",
            NumberMode::Free => "free",
            NumberMode::Pattern { .. } => "pattern",
            NumberMode::Script { .. } => "script",
        }
    }
}

impl PartType {
    /// Spell `sequence` as a part number of this type. The width comes from
    /// `digits`, never from a literal.
    pub fn format_number(&self, sequence: u64) -> String {
        format!(
            "{prefix}{sequence:0width$}",
            prefix = self.prefix,
            sequence = sequence,
            width = self.digits as usize
        )
    }

    /// The highest sequence this type can spell. `digits` 9 → 999_999_999.
    pub fn capacity(&self) -> u64 {
        10u64.saturating_pow(self.digits).saturating_sub(1)
    }
}

// ===========================================================================
// Parts and revisions
// ===========================================================================

/// Where a revision's content came from. Inert today (see the module header).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Origin {
    /// A person made it in the CAD app.
    #[default]
    Authored,
    /// A library import produced it (KiCad today).
    Imported,
    /// A part family generated it from a parameter tuple.
    Generated,
}

/// What a part's document is for, shown by its file extension. Chosen when
/// the part is created and NEVER changed: turning a normal part into a family
/// means a new part.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum DocumentClass {
    /// `.nBREP` — placed as-is. Imported parts and family members are normal.
    #[default]
    Normal,
    /// `.fBREP` — a family seed: the model plus the table of its members.
    Family,
    /// `.tBREP` — a template whose values are driven by expressions.
    Template,
}

impl DocumentClass {
    pub fn as_str(self) -> &'static str {
        match self {
            DocumentClass::Normal => "normal",
            DocumentClass::Family => "family",
            DocumentClass::Template => "template",
        }
    }

    pub fn parse(text: &str) -> Option<Self> {
        match text.trim().to_ascii_lowercase().trim_start_matches('.') {
            "" | "normal" | "nbrep" => Some(DocumentClass::Normal),
            "family" | "fbrep" => Some(DocumentClass::Family),
            "template" | "tbrep" => Some(DocumentClass::Template),
            _ => None,
        }
    }
}

/// The lifecycle states, in the order a revision moves through them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Lifecycle {
    Draft,
    InReview,
    Released,
    Superseded,
    Obsolete,
}

impl Lifecycle {
    /// Whether the revision's document may still be written. This is the ONE
    /// predicate immutability rests on — see [`crate::lifecycle`].
    pub fn is_editable(self) -> bool {
        matches!(self, Lifecycle::Draft | Lifecycle::InReview)
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Lifecycle::Draft => "draft",
            Lifecycle::InReview => "inreview",
            Lifecycle::Released => "released",
            Lifecycle::Superseded => "superseded",
            Lifecycle::Obsolete => "obsolete",
        }
    }
}

/// Who holds a draft revision checked out.
///
/// There is no lease and no expiry: the architecture chose MANUAL break, so a
/// lock persists until its holder checks in or someone in the check-in group
/// breaks it. `acquired_at` is what makes a stale lock visible to the person
/// deciding whether to break it.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Lock {
    pub user_id: String,
    /// Which client of that user holds it — the same person can be in the
    /// browser and the desktop build at once.
    #[serde(default)]
    pub client_id: String,
    pub acquired_at: Timestamp,
}

/// One revision of a part: the released unit of engineering intent, and the
/// thing a CAD document hangs off.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Revision {
    /// Whether the saved document produces BREP faces. Derived, never user editable.
    #[serde(default)]
    pub has_geometry: bool,
    /// Hash of the document used to derive geometry metadata.
    #[serde(default)]
    pub geometry_content_hash: String,
    /// Values defined by this part's numbered PLM type, owned by this revision.
    #[serde(default)]
    pub attributes: BTreeMap<String, serde_json::Value>,
    #[serde(default)]
    pub attributes_initialized: bool,
    /// Individual placements, identified by their CAD feature ids.
    #[serde(default)]
    pub occurrences: Vec<crate::bom_config::Occurrence>,
    /// The revision label itself; its database key is (part_id, id).
    pub id: String,
    /// What people call it. FREE TEXT: whoever creates the revision may type it,
    /// and the store enforces only that it is unique within its part. `A, B, C`
    /// from [`crate::lifecycle::next_revision_label`] is the suggestion when
    /// nobody types one. Revisions are ordered by creation — their position in
    /// [`Part::revisions`] — never by label.
    pub label: String,
    pub lifecycle: Lifecycle,
    #[serde(default)]
    pub origin: Origin,
    /// SHA-256 of the document as last written, so a client can detect drift
    /// without fetching the payload. Empty until something is written.
    #[serde(default)]
    pub content_hash: String,
    #[serde(default)]
    pub size: u64,
    /// `None` when nobody has it checked out.
    #[serde(default)]
    pub lock: Option<Lock>,
    pub created_by: String,
    pub created_at: Timestamp,
    #[serde(default)]
    pub modified_at: Timestamp,
    #[serde(default)]
    pub released_by: Option<String>,
    #[serde(default)]
    pub released_at: Option<Timestamp>,
    /// Set on a family member's revision by Generate: which family row made
    /// it, and the fingerprints the next Generate compares to skip an
    /// unchanged row. `None` for anything a family did not write.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub family: Option<FamilyStamp>,
    /// Set on the first revision of a part spun out of a template.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub template: Option<TemplateStamp>,
    /// The bake this revision's document is waiting for, or had. `None` for
    /// a document nothing asked to be baked.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bake: Option<Bake>,
    /// What this revision is built from: the "uses" list the CAD app
    /// publishes on save and release (round 1). The server never reads
    /// geometry — this list IS the assembly structure, and the BOM,
    /// where-used and the release gate are all computed from it. Frozen with
    /// the document once the revision is released.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub uses: Vec<Use>,
    /// Every review this revision has been through, oldest first: the last
    /// one is the current round ([`Review`]). A revision is reviewed where it
    /// lives, so a review costs what its part costs to write.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub reviews: Vec<Review>,
    /// The discussion on this revision, oldest first; a reply names the
    /// comment it answers. Anyone who can read the part reads it.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub comments: Vec<Comment>,
    /// Files that belong to THIS revision — its drawing, its test report.
    /// Frozen with the revision at release, like its document; a new revision
    /// starts with the same references (the files are shared, not copied).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub attachments: Vec<Attachment>,
    /// A picture of the document, made and uploaded by a CAD client or the
    /// bake worker (the server never renders). It describes the document
    /// whose hash it carries: once the document changes it is stale, and
    /// [`crate::thumbnail::current`] no longer serves it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub thumbnail: Option<Thumbnail>,
}

/// A revision's thumbnail: a PNG blob beside the attachments, keyed by the
/// document it pictures.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Thumbnail {
    /// Lower-case hex SHA-256 of the PNG, and the blob's name.
    pub sha256: String,
    pub size: u64,
    pub width: u32,
    pub height: u32,
    /// The revision's `content_hash` when the picture was made.
    pub content_hash: String,
    /// The producer's renderer version (`brep-thumb/1`).
    pub renderer: String,
    pub uploaded_by: String,
    pub uploaded_at: Timestamp,
}

/// A file attached to a part or to one of its revisions.
///
/// The bytes live once under `<data>/blobs/`, named by their SHA-256, so two
/// references to the same file share them; this record is the reference.
/// A blob goes when the last reference to it does.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Attachment {
    pub id: String,
    /// The file name shown and offered on download.
    pub name: String,
    /// What the uploader said it is (`application/pdf`). Only a short list of
    /// safe types is ever shown inline; everything else downloads.
    pub media_type: String,
    pub size: u64,
    /// Lower-case hex SHA-256 of the bytes, and the blob's name.
    pub sha256: String,
    /// `datasheet`, `drawing`, `spec`, `image` or `other`.
    #[serde(default = "other_kind")]
    pub kind: String,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub note: String,
    pub uploaded_by: String,
    pub uploaded_at: Timestamp,
}

fn other_kind() -> String {
    "other".to_string()
}

/// One line of an assembly revision's "uses" list: which part, which of its
/// revisions, and how many.
///
/// Every occurrence of the same part and revision in the CAD document folds
/// into ONE line with their count as `quantity`; two lines for the same child
/// differ in revision or find number.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Use {
    /// The child [`Part`], by id.
    pub part: String,
    /// The child revision, by id — PINNED. Empty is a FLOATING reference:
    /// whatever revision of the child is currently released (else its newest),
    /// resolved each time the BOM is read.
    #[serde(default)]
    pub revision: String,
    /// How many per ONE of the parent. Greater than zero; fractional for
    /// things bought by length or weight.
    #[serde(default = "one")]
    pub quantity: f64,
    /// What `quantity` counts: `each` unless it says otherwise (`mm`, `g`).
    #[serde(default = "each")]
    pub unit: String,
    /// The balloon number on the drawing, when there is one.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub find_number: String,
    /// Reference designators (`R1, R2`) or occurrence names, free text.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub reference: String,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub notes: String,
}

fn one() -> f64 {
    1.0
}

fn each() -> String {
    "each".to_string()
}

impl Revision {
    /// A new, empty draft.
    pub fn draft(id: String, label: String, created_by: String, stamp: Timestamp) -> Self {
        Revision {
            has_geometry: false,
            geometry_content_hash: String::new(),
            attributes: BTreeMap::new(),
            attributes_initialized: false,
            occurrences: Vec::new(),
            id,
            label,
            lifecycle: Lifecycle::Draft,
            origin: Origin::default(),
            content_hash: String::new(),
            size: 0,
            lock: None,
            created_by,
            created_at: stamp,
            modified_at: stamp,
            released_by: None,
            released_at: None,
            family: None,
            template: None,
            bake: None,
            uses: Vec::new(),
            reviews: Vec::new(),
            comments: Vec::new(),
            attachments: Vec::new(),
            thumbnail: None,
        }
    }

    /// The current review round: the last one, whatever its status.
    pub fn review(&self) -> Option<&Review> {
        self.reviews.last()
    }

    /// The current review round, if it is still undecided or approved —
    /// the one a decision or a release acts on.
    pub fn live_review_mut(&mut self) -> Option<&mut Review> {
        self.reviews.last_mut().filter(|r| r.is_live())
    }

    /// The document still waits for a bake — queued, taken, or failed. A
    /// revision in this state cannot release: once released its bytes are
    /// frozen and the bake could never be written.
    pub fn needs_bake(&self) -> bool {
        self.bake.as_ref().is_some_and(|bake| bake.status != BakeStatus::Done)
    }

    /// The store key this revision's document lives at — the key the CAD app's
    /// `sourceKey` becomes. `part/<part id>/rev/<revision id>`.
    pub fn document_key(&self, part_id: &str) -> String {
        crate::identity::document_key(part_id, &self.id)
    }
}

/// A part: permanent identity, the number, and its revisions oldest-first.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Part {
    /// The part number itself, stored directly as the database primary key.
    pub id: String,
    /// The number (`CPART000000001`, or whatever the type's mode accepted).
    /// Unique across the server, compared without regard to ASCII case.
    pub number: String,
    /// Which [`PartType`] minted or accepted the number.
    pub part_type: String,
    /// The counter value the number was spelled from, kept so a renumbering or
    /// an audit does not have to re-parse the string. `0` for a number that
    /// did not come from a counter.
    pub sequence: u64,
    /// What the part's document is for. Fixed at creation.
    #[serde(default)]
    pub document_class: DocumentClass,
    pub name: String,
    #[serde(default)]
    pub description: String,
    /// The part's ONE primary [`Category`], by id. Empty is uncategorized.
    /// A value that names no category — text written before the catalog
    /// existed — loads as-is and reads as uncategorized; it is replaced the
    /// next time someone sets the category.
    #[serde(default)]
    pub category: String,
    #[serde(default)]
    pub tags: Vec<String>,
    /// The upstream library identity for an imported part (`"Timer:NE555D"`),
    /// which is what makes a re-import idempotent. Inert today.
    #[serde(default)]
    pub external_ref: String,
    /// Catalog attribute values, keyed by [`AttributeDef::key`]. Checked
    /// against the category's schema when written; COMPLETE (every required
    /// key present) only at release, because a draft may be unfinished. A
    /// value whose key the current category does not define is kept — moving
    /// a part between categories never throws data away — and is inert.
    #[serde(default)]
    pub attributes: BTreeMap<String, serde_json::Value>,
    /// Who makes this part and who sells it: the approved manufacturer parts,
    /// each with its supplier offers. Part-level, like `attributes`, and NOT
    /// frozen by [`Settings::lock_released_attributes`] — sourcing changes
    /// with the market, not with the design.
    #[serde(default)]
    pub sourcing: Vec<ManufacturerPart>,
    /// For a family or a template: the part type its generated parts are
    /// created in — a family's new members, a template's spin-outs. Empty
    /// means the family's or template's own type. Inert on a normal part.
    #[serde(default)]
    pub member_part_type: String,
    /// Files that belong to the part whatever its revision — a datasheet, a
    /// supplier drawing. Not frozen by a release: like sourcing, they follow
    /// the part, not one design of it. See [`Attachment`].
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub attachments: Vec<Attachment>,
    pub created_by: String,
    pub created_at: Timestamp,
    #[serde(default)]
    pub revisions: Vec<Revision>,
}

impl Part {
    pub fn revision(&self, revision_id: &str) -> Option<&Revision> {
        self.revisions.iter().find(|r| r.id == revision_id)
    }

    pub fn revision_mut(&mut self, revision_id: &str) -> Option<&mut Revision> {
        self.revisions.iter_mut().find(|r| r.id == revision_id)
    }

    /// The newest revision, whatever its state. "Newest" is creation order —
    /// the list's order — because a free-text label says nothing about age.
    pub fn latest(&self) -> Option<&Revision> {
        self.revisions.last()
    }

    /// The revision labelled `label`, compared without regard to ASCII case —
    /// `a` and `A` would be two revisions nobody could tell apart aloud.
    pub fn revision_by_label(&self, label: &str) -> Option<&Revision> {
        let label = label.trim();
        self.revisions
            .iter()
            .find(|r| r.label.eq_ignore_ascii_case(label))
    }

    /// The first revision still in work (Draft or InReview), if any. Whether a
    /// second may be started beside it is the administrator's
    /// [`Settings::allow_multiple_open_drafts`].
    pub fn open_draft(&self) -> Option<&Revision> {
        self.revisions.iter().find(|r| r.lifecycle.is_editable())
    }

    /// The revision currently Released. A release supersedes every other, so
    /// there is at most one.
    pub fn current_release(&self) -> Option<&Revision> {
        self.revisions.iter().rev().find(|r| r.lifecycle == Lifecycle::Released)
    }

    /// Whether [`Settings::lock_released_attributes`] would freeze this part's
    /// catalog values: some revision has been released (whatever it is now),
    /// and none is in work to carry a change.
    pub fn catalog_frozen(&self) -> bool {
        self.revisions.iter().any(|r| r.released_at.is_some()) && self.open_draft().is_none()
    }
}

// ===========================================================================
// Families, templates and the bake queue
// ===========================================================================

/// What a family's Generate records on a member revision. The document
/// carries the CAD app's own `familySource` stamp; this is the server's
/// copy, which is what the family view and the next Generate read without
/// opening every member document.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FamilyStamp {
    /// The family part's id.
    pub family_part: String,
    /// The family revision whose document the member was generated from.
    pub family_revision: String,
    /// The row's values, expression source keyed by expression name.
    #[serde(default)]
    pub values: BTreeMap<String, String>,
    #[serde(default)]
    pub description: String,
    /// The CAD app's `row_hash` of the row: part number, description, values.
    pub values_hash: String,
    /// The CAD app's `history_hash` of the family: its model less the table.
    pub history_hash: String,
    /// [`Revision::content_hash`] of the document as Generate (or its bake)
    /// wrote it. A member whose hash has moved on since was edited by hand.
    pub generated_hash: String,
    pub generated_by: String,
    pub generated_at: Timestamp,
}

/// What a template spin-out records on the new part's first revision.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TemplateStamp {
    pub template_part: String,
    pub template_revision: String,
    /// The input values the copy was made with.
    #[serde(default)]
    pub values: BTreeMap<String, String>,
    /// The CAD app's `template_hash` of the template document at the time.
    pub content_hash: String,
    pub created_at: Timestamp,
}

/// Where a bake is. The server never links the kernel, so a document it
/// assembled (a family member, a template copy) is checked and rebuilt by a
/// headless CAD worker, which takes the job from the queue.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum BakeStatus {
    /// Waiting for a worker.
    Pending,
    /// A worker has it (see [`Bake::claimed_at`] for the lease).
    Claimed,
    /// The worker reported a problem; a person retries it.
    Failed,
    /// Baked, or superseded by a document someone saved.
    Done,
}

impl BakeStatus {
    pub fn as_str(self) -> &'static str {
        match self {
            BakeStatus::Pending => "pending",
            BakeStatus::Claimed => "claimed",
            BakeStatus::Failed => "failed",
            BakeStatus::Done => "done",
        }
    }
}

/// One revision's bake job. The job's id is the revision's id: a revision has
/// at most one.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Bake {
    /// A Force resave job rebuilds this exact saved document. Older generated
    /// jobs have no snapshot hash and retain their existing bake behavior.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub resave_hash: Option<String>,
    pub status: BakeStatus,
    /// Why it was queued ("generated from family CPART000000012 row M6x20").
    #[serde(default)]
    pub reason: String,
    pub requested_by: String,
    pub requested_at: Timestamp,
    #[serde(default)]
    pub claimed_by: Option<String>,
    #[serde(default)]
    pub claimed_at: Option<Timestamp>,
    /// How many times a worker has taken it.
    #[serde(default)]
    pub attempts: u32,
    /// The worker's report of the last failure.
    #[serde(default)]
    pub error: String,
    #[serde(default)]
    pub finished_by: Option<String>,
    #[serde(default)]
    pub finished_at: Option<Timestamp>,
}

// ===========================================================================
// Sourcing — manufacturers, suppliers, and what each offers for a part
// ===========================================================================

/// A manufacturer or a supplier. The two are separate lists (round 1)
/// even though they share a shape: a manufacturer part names a
/// manufacturer, an offer names a supplier, and a company that is both —
/// one that sells what it makes — appears in each list.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Company {
    /// Generated; never shown to a person.
    pub id: String,
    /// Unique within its list, compared without regard to ASCII case.
    pub name: String,
    #[serde(default)]
    pub website: String,
    #[serde(default)]
    pub notes: String,
    #[serde(default)]
    pub created_at: Timestamp,
}

/// Where a manufacturer part stands in its maker's catalogue.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum SourcingStatus {
    #[default]
    Active,
    /// Not recommended for new designs.
    Nrnd,
    Obsolete,
}

impl SourcingStatus {
    pub fn as_str(self) -> &'static str {
        match self {
            SourcingStatus::Active => "active",
            SourcingStatus::Nrnd => "nrnd",
            SourcingStatus::Obsolete => "obsolete",
        }
    }
}

/// One manufacturer's part that satisfies one of our parts: the MPN.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ManufacturerPart {
    pub id: String,
    /// The [`Company`] id in the manufacturers list.
    pub manufacturer: String,
    /// The manufacturer's part number. Unique per manufacturer within one
    /// part, compared without regard to ASCII case.
    pub mpn: String,
    #[serde(default)]
    pub status: SourcingStatus,
    /// The one to buy first. At most one per part.
    #[serde(default)]
    pub preferred: bool,
    #[serde(default)]
    pub datasheet: String,
    #[serde(default)]
    pub notes: String,
    /// Who sells this MPN, and on what terms.
    #[serde(default)]
    pub offers: Vec<SupplierOffer>,
    #[serde(default)]
    pub created_at: Timestamp,
}

/// One supplier's offer for one manufacturer part.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SupplierOffer {
    pub id: String,
    /// The [`Company`] id in the suppliers list.
    pub supplier: String,
    /// The supplier's own part number (SPN).
    #[serde(default)]
    pub spn: String,
    #[serde(default)]
    pub url: String,
    /// A currency code as the supplier quotes it (`USD`). Text, not a list:
    /// nothing here converts between currencies.
    #[serde(default)]
    pub currency: String,
    /// Unit price by quantity, ascending by `qty`.
    #[serde(default)]
    pub price_breaks: Vec<PriceBreak>,
    #[serde(default)]
    pub lead_time_days: Option<u32>,
    /// Minimum order quantity.
    #[serde(default)]
    pub moq: Option<u64>,
    /// Stock on hand when last checked, if known.
    #[serde(default)]
    pub stock: Option<u64>,
    #[serde(default)]
    pub notes: String,
    /// When the offer was last written — how stale the price is.
    #[serde(default)]
    pub updated_at: Timestamp,
}

/// Buy at least `qty` and each costs `unit_price`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PriceBreak {
    pub qty: u64,
    pub unit_price: f64,
}

// ===========================================================================
// The audit log — who changed what, and when
// ===========================================================================

/// Who made a change.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Actor {
    /// The user's id; empty for the server itself.
    #[serde(default)]
    pub user_id: String,
    /// The username at the time, kept so the log reads right after a rename
    /// or a deletion.
    #[serde(default)]
    pub username: String,
    /// How the change arrived: `session` (a browser), `token` (an API token),
    /// or `system` (the server itself, or a caller with no signed-in user —
    /// start-up, a test, a sign-in before it succeeds).
    #[serde(default)]
    pub via: String,
    /// The client's address, when the request carried one.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub ip: String,
}

impl Actor {
    /// The server itself.
    pub fn system() -> Self {
        Actor { user_id: String::new(), username: String::new(), via: "system".into(), ip: String::new() }
    }
}

/// What a change was made to.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EntityRef {
    /// `part`, `revision`, `user`, `token`, `part-type`, `category`,
    /// `manufacturer`, `supplier`, `settings`, `script` — or, for a
    /// collection a later slice adds, its name in the store.
    pub kind: String,
    pub id: String,
    /// What a person calls it: a part number, `CPART000000001 rev B`, a
    /// username, a category name.
    #[serde(default)]
    pub label: String,
    /// For a revision, its part's id, so a part's history includes its
    /// revisions' events.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub part_id: String,
}

/// One field's value before and after. `null` on a side where the field did
/// not exist.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FieldChange {
    pub before: serde_json::Value,
    pub after: serde_json::Value,
}

/// One entry of the audit log. Append-only: nothing edits or deletes one.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AuditEvent {
    /// 1, 2, 3 … in the order events were written.
    pub id: u64,
    pub at: Timestamp,
    pub actor: Actor,
    /// What happened: `create`, `update`, `delete`, `release`, `checkout`,
    /// `checkin`, `break-lock`, `document`, `uses`, `sourcing`, `sign-in`,
    /// `sign-in-failed`, `session-ended`, `script-write`, … See
    /// `crate::audit` for the full list.
    pub action: String,
    pub entity: EntityRef,
    /// The fields that changed. Secrets (password verifiers, token hashes)
    /// appear as `"(changed)"`, never as their value.
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub changes: BTreeMap<String, FieldChange>,
    /// A sentence, for events a field diff does not explain.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub detail: String,
    /// The store's sequence number after the change; 0 for an event that
    /// changed nothing in the store (a failed sign-in, a script file).
    #[serde(default)]
    pub seq: u64,
}

// ===========================================================================
// Review and approval
// ===========================================================================

/// Where a review round stands.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ReviewStatus {
    /// Waiting for approvals.
    #[default]
    Open,
    /// Its rule is met: enough approvals, no rejection. It may still be
    /// rejected until the subject is released.
    Approved,
    /// A reviewer rejected it; the subject went back to Draft. Final.
    Rejected,
    /// The subject was pulled back to Draft by its author. Final.
    Withdrawn,
}

impl ReviewStatus {
    pub fn as_str(self) -> &'static str {
        match self {
            ReviewStatus::Open => "open",
            ReviewStatus::Approved => "approved",
            ReviewStatus::Rejected => "rejected",
            ReviewStatus::Withdrawn => "withdrawn",
        }
    }
}

/// Who may review: one user, or everyone in a group.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReviewerRef {
    /// `user` or `group`.
    pub kind: ReviewerKind,
    /// A user's id, or a group's name.
    pub id: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ReviewerKind {
    User,
    Group,
}

/// A reviewer's verdict.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Verdict {
    Approve,
    Reject,
}

impl Verdict {
    pub fn as_str(self) -> &'static str {
        match self {
            Verdict::Approve => "approve",
            Verdict::Reject => "reject",
        }
    }
}

/// One reviewer's decision. A rejection always carries a comment.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Decision {
    pub user_id: String,
    pub verdict: Verdict,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub comment: String,
    pub at: Timestamp,
    /// What was decided on: the subject's content hash when the decision was
    /// given — a revision's `content_hash`, or a change order's
    /// [`crate::review::eco_subject_hash`]. An approval counts only while this
    /// equals the subject's hash NOW (operator ruling D8, 2026-09-25:
    /// approvals go stale when the document changes). `None` is a decision
    /// recorded before decisions carried a hash: what it was given on is
    /// unknown, so it never counts — it is "on an older version".
    #[serde(default)]
    pub content_hash: Option<String>,
}

impl Decision {
    /// Whether this decision was given on the subject as it is now.
    pub fn is_current(&self, current: &str) -> bool {
        self.content_hash.as_deref() == Some(current)
    }
}

/// One round of review of a revision or a change order, opened when its
/// subject moves to review.
///
/// The rule it is judged by is COPIED in when it opens (`required_approvals`,
/// `reviewers`, `allow_self_approval`): an administrator changing the
/// settings afterwards changes the next round, not one already under way.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Review {
    pub id: String,
    #[serde(default)]
    pub status: ReviewStatus,
    /// Who submitted the subject for review.
    pub opened_by: String,
    pub opened_at: Timestamp,
    /// Who may approve or reject. Empty: anyone in the check-in group.
    #[serde(default)]
    pub reviewers: Vec<ReviewerRef>,
    /// Approvals needed before the subject may release. 0 means none.
    #[serde(default)]
    pub required_approvals: u32,
    /// Whether the person who submitted it may approve it themselves.
    #[serde(default)]
    pub allow_self_approval: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub due: Option<Timestamp>,
    /// What decided the rule: `settings`, `part-type:<id>`,
    /// `category:<id>`, and `+script` when the reviewers hook changed it.
    #[serde(default)]
    pub rule_source: String,
    #[serde(default)]
    pub decisions: Vec<Decision>,
    /// Set when the round ends: a rejection, a withdrawal, or the release it
    /// allowed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub closed_by: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub closed_at: Option<Timestamp>,
}

impl Review {
    /// Still deciding, or approved and not yet used: a decision or a
    /// release may act on it.
    pub fn is_live(&self) -> bool {
        matches!(self.status, ReviewStatus::Open | ReviewStatus::Approved) && self.closed_at.is_none()
    }

    /// Each user's LATEST decision, if it is an approval that the rule lets
    /// count (never the submitter's unless the rule allows it) — whatever
    /// version it was given on.
    fn standing_approvals(&self) -> Vec<&Decision> {
        let mut latest: Vec<&Decision> = Vec::new();
        for decision in &self.decisions {
            latest.retain(|d| d.user_id != decision.user_id);
            latest.push(decision);
        }
        latest
            .into_iter()
            .filter(|d| d.verdict == Verdict::Approve && (self.allow_self_approval || d.user_id != self.opened_by))
            .collect()
    }

    /// The users whose approval counts: a standing approval given on the
    /// subject as it is now, `current` being its content hash.
    pub fn approvers(&self, current: &str) -> Vec<&str> {
        self.standing_approvals().into_iter().filter(|d| d.is_current(current)).map(|d| d.user_id.as_str()).collect()
    }

    /// Standing approvals given on an older version of the subject (or before
    /// decisions recorded one): they need renewing to count.
    pub fn stale_approvers(&self, current: &str) -> Vec<&str> {
        self.standing_approvals().into_iter().filter(|d| !d.is_current(current)).map(|d| d.user_id.as_str()).collect()
    }

    /// Whether the rule is met on the subject as it is now: enough current
    /// approvals, and no rejection.
    pub fn is_met(&self, current: &str) -> bool {
        !self.decisions.iter().any(|d| d.verdict == Verdict::Reject)
            && self.approvers(current).len() as u32 >= self.required_approvals
    }
}

/// A comment on a revision or a change order.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Comment {
    pub id: String,
    pub author: String,
    pub at: Timestamp,
    pub body: String,
    /// The comment this one answers; empty for a new thread.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub parent: String,
}

/// The rule a review round is opened with.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct ReviewRule {
    /// Approvals needed before release. 0 — the default — means a release
    /// needs no review at all, as before reviews existed.
    pub required_approvals: u32,
    /// Who may decide. Empty: anyone in the check-in group.
    pub reviewers: Vec<ReviewerRef>,
    /// Days from submission to the due date; 0 for none.
    pub due_days: u32,
    /// Whether the submitter's own approval counts.
    pub allow_self_approval: bool,
}

/// A rule for the parts of one part type, or of one category and every
/// category under it. A category's beats a part type's, and the nearest
/// category beats its ancestors.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReviewOverride {
    /// `part_type` or `category`.
    pub scope: String,
    pub id: String,
    pub rule: ReviewRule,
}

// ===========================================================================
// Change orders (ECO)
// ===========================================================================

/// Where a change order stands. `Released` and `Cancelled` are final.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum EcoState {
    #[default]
    Draft,
    InReview,
    Approved,
    Released,
    Cancelled,
}

impl EcoState {
    pub fn as_str(self) -> &'static str {
        match self {
            EcoState::Draft => "draft",
            EcoState::InReview => "inreview",
            EcoState::Approved => "approved",
            EcoState::Released => "released",
            EcoState::Cancelled => "cancelled",
        }
    }

    /// Still going somewhere: it holds the revisions it names.
    pub fn is_open(self) -> bool {
        matches!(self, EcoState::Draft | EcoState::InReview | EcoState::Approved)
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum EcoPriority {
    Low,
    #[default]
    Normal,
    High,
    Critical,
}

impl EcoPriority {
    pub fn as_str(self) -> &'static str {
        match self {
            EcoPriority::Low => "low",
            EcoPriority::Normal => "normal",
            EcoPriority::High => "high",
            EcoPriority::Critical => "critical",
        }
    }
}

/// What a change order does to one revision when it is released.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum EcoAction {
    /// Release this Draft or In-review revision.
    Release,
    /// Obsolete this Released or Superseded revision.
    Obsolete,
}

impl EcoAction {
    pub fn as_str(self) -> &'static str {
        match self {
            EcoAction::Release => "release",
            EcoAction::Obsolete => "obsolete",
        }
    }
}

/// One revision a change order acts on.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EcoItem {
    pub part_id: String,
    pub revision_id: String,
    pub action: EcoAction,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub note: String,
    /// For a release, set when the change order releases: the revision that
    /// was current before, which this one superseded. What the item changed
    /// stays readable after the release.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub replaced: Option<String>,
}

/// A change order: a set of revisions reviewed together and released — or
/// obsoleted — together, all or none (round 9 item 5).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ChangeOrder {
    pub id: String,
    /// Its number, from [`State::eco_numbering`]. Unique among change orders.
    pub number: String,
    pub title: String,
    #[serde(default)]
    pub description: String,
    /// Why the change is made.
    #[serde(default)]
    pub reason: String,
    #[serde(default)]
    pub priority: EcoPriority,
    #[serde(default)]
    pub state: EcoState,
    #[serde(default)]
    pub items: Vec<EcoItem>,
    pub created_by: String,
    pub created_at: Timestamp,
    #[serde(default)]
    pub modified_at: Timestamp,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub released_by: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub released_at: Option<Timestamp>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cancelled_by: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cancelled_at: Option<Timestamp>,
    /// Its review rounds, oldest first ([`Review`]).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub reviews: Vec<Review>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub comments: Vec<Comment>,
}

impl ChangeOrder {
    /// The item acting on `revision_id`, if any.
    pub fn item(&self, part_id: &str, revision_id: &str) -> Option<&EcoItem> {
        self.items.iter().find(|i| i.part_id == part_id && i.revision_id == revision_id)
    }
}

/// How change orders are numbered: a part type's numbering scheme — mode,
/// prefix, digits and counter — used for change orders alone.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(transparent)]
pub struct EcoNumbering(pub PartType);

impl Default for EcoNumbering {
    fn default() -> Self {
        EcoNumbering(default_eco_numbering())
    }
}

impl std::ops::Deref for EcoNumbering {
    type Target = PartType;
    fn deref(&self) -> &PartType {
        &self.0
    }
}

/// How change orders are numbered when nobody has changed it: `ECO` + 6
/// digits by counter.
pub fn default_eco_numbering() -> PartType {
    PartType {
        id: "eco".into(),
        name: "Change order".into(),
        prefix: "ECO".into(),
        digits: 6,
        next: 1,
        created_at: 0,
        mode: NumberMode::Counter,
    }
}

// ===========================================================================
// Server settings — the administrator's policy choices
// ===========================================================================

/// Configurable names and availability of workflow status options.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct StatusOption {
    pub state: Lifecycle,
    pub name: String,
    pub enabled: bool,
}

pub fn default_status_options() -> Vec<StatusOption> {
    [(Lifecycle::Draft, "none"), (Lifecycle::InReview, "review"),
     (Lifecycle::Released, "released"), (Lifecycle::Obsolete, "obsolete"),
     (Lifecycle::Superseded, "superseded")].into_iter()
        .map(|(state, name)| StatusOption { state, name: name.into(), enabled: true }).collect()
}

/// Policy the administrator decides, not code (round 7). Each defaults
/// to the PERMISSIVE choice, and a file written before settings existed
/// loads with those defaults.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    pub setup_completed: bool,
    pub status_options: Vec<StatusOption>,
    /// Refuse changes to a part's category and attribute values while the
    /// part has no revision in work and at least one released one — so a
    /// released part's catalog values change only by starting a new revision.
    /// Off by default: attributes stay editable after release.
    pub lock_released_attributes: bool,
    /// Let a part have more than one revision in work (Draft or InReview) at
    /// once. On by default, which is what lets a family row create the
    /// revision it names while another is open.
    pub allow_multiple_open_drafts: bool,
    /// Refuse to release an assembly revision while anything in its "uses"
    /// list is not Released — the classic release gate. Off by default: the
    /// release goes ahead and says which children are unreleased.
    pub require_released_children: bool,
    /// A browser session ends after this many minutes without a request.
    /// 14 days by default, which is what sessions lasted before this setting.
    pub session_idle_minutes: u64,
    /// A browser session ends this many hours after sign-in however busy it
    /// is. 30 days by default; 0 means no absolute limit.
    pub session_max_hours: u64,
    /// Let administrators edit and test-run scripts in the browser. On by
    /// default. Scripts are fully trusted, so an admin login with the
    /// editor on is a shell on the server; turning it off leaves the
    /// scripts directory live and editable only on the host. The server's
    /// `--lock-script-editor` flag forces it off regardless.
    pub script_editor_enabled: bool,
    /// The review rule for a revision whose part no override covers. The
    /// default needs 0 approvals, so a release works as it did before
    /// reviews existed until an administrator sets one.
    pub review_rule: ReviewRule,
    /// Rules for particular part types and categories ([`ReviewOverride`]).
    pub review_overrides: Vec<ReviewOverride>,
    /// The review rule a change order opens its round with. 0 approvals by
    /// default: submitting approves it at once.
    pub eco_review_rule: ReviewRule,
    /// Refuse to release or obsolete, on its own, a revision an open change
    /// order names. Off by default: it goes ahead, with a warning naming the
    /// change order.
    pub eco_holds_revisions: bool,
    /// Let a signed-in user browse (read only) other users' workspaces
    /// ([`WorkspaceEntry`]). On by default, the permissive choice of rounds
    /// 6-7: a workspace holds links to parts everyone may already read, and
    /// files a colleague often needs to see (a customer's photo of a
    /// failure). Off, a workspace is its owner's alone; an administrator can
    /// still read any, to support a user.
    pub workspaces_browsable: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            setup_completed: false,
            status_options: default_status_options(),
            lock_released_attributes: false,
            allow_multiple_open_drafts: true,
            require_released_children: false,
            session_idle_minutes: 14 * 24 * 60,
            session_max_hours: 30 * 24,
            script_editor_enabled: true,
            review_rule: ReviewRule::default(),
            review_overrides: Vec::new(),
            eco_review_rule: ReviewRule::default(),
            eco_holds_revisions: false,
            workspaces_browsable: true,
        }
    }
}

// ===========================================================================
// Workspaces — each user's folders of links and files (plm-cad-integration D12)
// ===========================================================================

/// One entry of a user's workspace: a folder, a link to a part, or a file.
///
/// A workspace replaces directories in PLM mode. It is a tree per user,
/// stored flat: each entry names its parent folder (empty at the top). A
/// link is never a copy of the part; deleting one never touches the part. A
/// file's bytes are blobs in the attachment store, and every replace keeps
/// the version before it ([`FileVersion`]).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceEntry {
    pub id: String,
    /// The user whose workspace this is.
    pub owner: String,
    /// The folder it is in; empty at the top of the workspace.
    #[serde(default)]
    pub parent: String,
    /// Unique among its folder's entries, ignoring ASCII case.
    pub name: String,
    pub kind: WorkspaceKind,
    /// For a link: the part, and the revision it is pinned to.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub link: Option<WorkspaceLink>,
    /// For a file: every version, oldest first; the last is current.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub versions: Vec<FileVersion>,
    pub created_at: Timestamp,
    #[serde(default)]
    pub modified_at: Timestamp,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum WorkspaceKind {
    Folder,
    Link,
    File,
}

impl WorkspaceKind {
    pub fn as_str(self) -> &'static str {
        match self {
            WorkspaceKind::Folder => "folder",
            WorkspaceKind::Link => "link",
            WorkspaceKind::File => "file",
        }
    }
}

/// What a link points at. Held by id, so a part renamed or renumbered is
/// still found.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceLink {
    pub part_id: String,
    /// Empty: the link follows the part's newest revision, drafts included
    /// (D13). Else the revision it is pinned to.
    #[serde(default)]
    pub revision_id: String,
}

/// One version of a workspace file.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FileVersion {
    /// 1, 2, 3 … in upload order; never reused.
    pub version: u32,
    pub sha256: String,
    pub size: u64,
    pub media_type: String,
    pub uploaded_by: String,
    pub uploaded_at: Timestamp,
    /// The version this one restored, when it was made by a restore.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub restored_from: Option<u32>,
}

// ===========================================================================
// The catalog — categories and their typed attribute schemas
// ===========================================================================

/// A node of the catalog tree. A part has ONE primary category plus free tags
/// (round 1). A child category INHERITS every attribute of its ancestors and
/// may add its own, but may not redefine one — so "Fasteners" can require a
/// material and "Fasteners/Screws" add a thread without ever disagreeing
/// about what a material is.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Category {
    /// Stable key, stored on parts (`"screws"`). Never changes.
    pub id: String,
    /// What the UI shows (`"Screws"`).
    pub name: String,
    /// The parent's id; empty for a top-level category.
    #[serde(default)]
    pub parent: String,
    /// The attributes THIS category adds. Its effective schema is these plus
    /// every ancestor's.
    #[serde(default)]
    pub attributes: Vec<AttributeDef>,
    #[serde(default)]
    pub created_at: Timestamp,
}

/// One typed attribute a category defines.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AttributeDef {
    /// The key values are stored under on a part (`"thread"`).
    pub key: String,
    /// What the UI shows (`"Thread size"`).
    pub name: String,
    #[serde(flatten)]
    pub kind: AttributeKind,
    /// Must have a value before a revision of the part can release. A draft
    /// may leave it empty.
    #[serde(default)]
    pub required: bool,
}

/// What values an attribute takes. Serialized under `type`, beside the
/// attribute's other fields: `{"key":"length","name":"Length","type":"number","unit":"mm"}`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum AttributeKind {
    Text,
    /// A number. `unit` is a label the UI shows, not a conversion.
    Number {
        #[serde(default)]
        unit: String,
    },
    Bool,
    /// One of a fixed list of values.
    Enum { values: Vec<String> },
}

impl AttributeKind {
    pub fn as_str(&self) -> &'static str {
        match self {
            AttributeKind::Text => "text",
            AttributeKind::Number { .. } => "number",
            AttributeKind::Bool => "bool",
            AttributeKind::Enum { .. } => "enum",
        }
    }
}

