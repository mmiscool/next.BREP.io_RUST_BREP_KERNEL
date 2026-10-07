//! Parts, revisions, checkout and lifecycle.
//!
//! The handlers are thin: every rule that could be got wrong twice — number
//! allocation, one-draft-per-part, who may break a lock, what a released
//! revision may become — lives in [`crate::db`] and [`crate::lifecycle`], and
//! these functions decide only who is asking.

use axum::extract::{Path, Query, State};
use axum::http::HeaderMap;
use axum::response::{IntoResponse, Response};
use axum::Json;
use serde::{Deserialize, Serialize};

use super::{blocking, ok_seq, require_author, require_user, Shared};
use crate::catalog::{self, Effective};
use crate::db::{suggest_label, PartFilter, PartSpec};
use crate::model::{DocumentClass, Lifecycle, Part, Revision};
use crate::Error;

/// A part in a list: enough to show a row, never the revision payloads.
#[derive(Debug, Serialize)]
pub struct PartRow {
    pub has_geometry: bool,
    /// Whether the part has the current preview used by the Parts listing.
    pub has_thumbnail: bool,
    pub id: String,
    pub number: String,
    pub name: String,
    pub part_type: String,
    pub document_class: DocumentClass,
    /// The stored category text: a category id, empty, or text from before
    /// the catalog that names no category.
    pub category: String,
    /// "Fasteners / Screws" when `category` names a category; empty when the
    /// part is uncategorized.
    pub category_path: String,
    pub tags: Vec<String>,
    pub revisions: usize,
    pub latest_label: String,
    pub latest_state: &'static str,
    pub locked: bool,
    /// The preferred MPN, or else the first; empty with no sourcing.
    pub mpn: String,
    /// Where the part comes from in another system; empty for none.
    pub external_ref: String,
    /// The newest revision (by creation, drafts included): its id, its
    /// origin (`authored`, `imported`, `generated`) and its document's
    /// content hash (lowercase hex SHA-256 of the bytes as PUT; empty with
    /// no document). What a library re-import compares against.
    pub latest_revision_id: String,
    pub latest_origin: Option<crate::model::Origin>,
    pub latest_content_hash: String,
    /// The part's catalog values, canonical (numbers and flags as JSON
    /// numbers and booleans), when the listing asks `include=attributes`.
    /// Every stored value, inert ones included.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub attributes: Option<std::collections::BTreeMap<String, serde_json::Value>>,
    /// Where to fetch the part's thumbnail (its newest revision's that is
    /// current, [`crate::thumbnail`]), versioned by the picture; empty when
    /// it has none.
    pub thumbnail_url: String,
}

impl PartRow {
    fn new(categories: &[crate::model::Category], part: &Part) -> Self {
        let latest = part.latest();
        Self {
            has_geometry: latest.is_some_and(|r| r.has_geometry),
            has_thumbnail: crate::thumbnail::of_part(part).is_some(),
            id: part.id.clone(),
            number: part.number.clone(),
            name: part.name.clone(),
            part_type: part.part_type.clone(),
            document_class: part.document_class,
            category: part.category.clone(),
            category_path: catalog::path_name(categories, &part.category),
            tags: part.tags.clone(),
            revisions: part.revisions.len(),
            latest_label: latest.map(|r| r.label.clone()).unwrap_or_default(),
            latest_state: latest.map(|r| r.lifecycle.as_str()).unwrap_or("—"),
            locked: part.revisions.iter().any(|r| r.lock.is_some()),
            mpn: part
                .sourcing
                .iter()
                .find(|mp| mp.preferred)
                .or(part.sourcing.first())
                .map(|mp| mp.mpn.clone())
                .unwrap_or_default(),
            attributes: None,
            external_ref: part.external_ref.clone(),
            latest_revision_id: latest.map(|r| r.id.clone()).unwrap_or_default(),
            latest_origin: latest.map(|r| r.origin),
            latest_content_hash: latest.map(|r| r.content_hash.clone()).unwrap_or_default(),
            thumbnail_url: crate::thumbnail::part_url(part),
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct Search {
    #[serde(default)]
    pub q: String,
    /// A category id — the parts in it and beneath it — or `_none` for the
    /// uncategorized.
    #[serde(default)]
    pub category: String,
    #[serde(default)]
    pub tag: String,
    /// A manufacturer id or name: parts with a manufacturer part from it.
    #[serde(default)]
    pub manufacturer: String,
    /// A supplier id or name: parts with an offer from it.
    #[serde(default)]
    pub supplier: String,
    /// Page size, default 50. `next` is `null` on the last page.
    #[serde(default)]
    pub limit: Option<usize>,
    /// The `next` of the page before.
    #[serde(default)]
    pub after: Option<String>,
    /// `attributes`: each row also carries the part's attribute values.
    #[serde(default)]
    pub include: String,
    /// Exact: the part(s) a library import made for this entry.
    #[serde(default)]
    pub external_ref: String,
    /// Exact part type id.
    #[serde(default)]
    pub part_type: String,
}

/// The largest page a client may ask for.
pub const MAX_PAGE: usize = 1000;

/// Search and filter parts ([`crate::db::Db::find_parts_page`]), one page
/// and the cursor to the next.
pub async fn list_parts(
    State(db): State<Shared>,
    headers: HeaderMap,
    Query(search): Query<Search>,
    Query(pairs): Query<Vec<(String, String)>>,
) -> Result<Json<serde_json::Value>, Error> {
    require_user(&db, &headers)?;
    let filter = PartFilter {
        q: search.q,
        category: search.category,
        tag: search.tag,
        manufacturer: search.manufacturer,
        supplier: search.supplier,
        attributes: crate::db::AttributeFilter::from_query(&pairs)?,
        external_ref: search.external_ref,
        part_type: search.part_type,
    };
    // Conditions that do not read are refused, not answered with nothing.
    db.read(|state| crate::db::attribute_matchers(state, &filter))?;
    let with_attributes = match search.include.trim() {
        "" => false,
        "attributes" => true,
        other => return Err(Error::bad_request(format!("include={other}: the only thing a row can include is 'attributes'"))),
    };
    let limit = search.limit.unwrap_or(50).clamp(1, MAX_PAGE);
    let page = db.find_parts_page(&filter, search.after.as_deref(), Some(limit));
    let rows: Vec<PartRow> = db.read(|state| {
        page.parts
            .iter()
            .map(|p| {
                let mut row = PartRow::new(&state.categories, p);
                if with_attributes {
                    let mut attributes = p.attributes.clone();
                    attributes.insert("has_geometry".into(), serde_json::json!(row.has_geometry));
                    attributes.insert("has_thumbnail".into(), serde_json::json!(row.has_thumbnail));
                    row.attributes = Some(attributes);
                }
                row
            })
            .collect()
    });
    Ok(Json(serde_json::json!({ "parts": rows, "next": page.next })))
}

/// One part with its revisions, each carrying who holds it.
#[derive(Debug, Serialize)]
pub struct PartDetail {
    pub has_geometry: bool,
    /// Whether the part has the current preview used by the Parts listing.
    pub has_thumbnail: bool,
    #[serde(flatten)]
    pub part: Part,
    pub revision_views: Vec<RevisionView>,
    /// The label the next revision gets when nobody types one.
    pub suggested_label: String,
    /// "Fasteners / Screws"; empty when uncategorized.
    pub category_path: String,
    /// Every attribute the part's category defines, inherited ones first.
    /// Empty when uncategorized.
    pub schema: Vec<Effective>,
    /// Keys the part holds a value for that its category does not define —
    /// left by a move between categories. Kept, shown, and inert.
    pub inert: Vec<String>,
    /// The administrator's lock on released catalog values applies to this
    /// part right now: its category and attributes change only once a new
    /// revision is in work.
    pub catalog_locked: bool,
    /// Whether another revision may be started while one is in work.
    pub multiple_open_drafts: bool,
    /// The part's sourcing with company names resolved beside their ids
    /// ([`crate::sourcing::resolved`]). `sourcing` on the part itself carries
    /// the ids alone.
    pub sourcing_view: serde_json::Value,
    /// The revision "open by number" fills in (D13): the newest by
    /// creation, drafts included — the last of `revision_views`, named so a
    /// client need not know that. Absent for a part with no revision.
    pub newest_revision: Option<NewestRevision>,
    /// The part's thumbnail URL ([`PartRow::thumbnail_url`]); empty for none.
    pub thumbnail_url: String,
}

/// The newest revision of a part, whatever its state.
#[derive(Debug, Serialize)]
pub struct NewestRevision {
    pub id: String,
    pub label: String,
    pub lifecycle: &'static str,
    pub document_key: String,
}

/// A revision with the lock resolved to a name — the id alone tells the page
/// nothing it can display.
#[derive(Debug, Serialize)]
pub struct RevisionView {
    pub id: String,
    pub label: String,
    pub lifecycle: &'static str,
    pub editable: bool,
    pub content_hash: String,
    pub size: u64,
    pub created_at: u64,
    pub modified_at: u64,
    pub locked_by: Option<String>,
    pub locked_by_me: bool,
    pub locked_at: Option<u64>,
    pub released_by: Option<String>,
    pub released_at: Option<u64>,
    pub document_key: String,
    /// `authored`, `imported` or `generated`.
    pub origin: crate::model::Origin,
    /// The family that generated this revision, when one did.
    pub family: Option<Provenance>,
    /// The template this revision was spun out of, when it was.
    pub template: Option<Provenance>,
    /// `pending`, `claimed`, `failed` or `done` — absent when nothing was
    /// queued to bake.
    pub bake: Option<&'static str>,
    pub bake_error: String,
    /// A generated member whose document changed since Generate wrote it.
    pub hand_edited: bool,
    /// How many lines the revision's uses list has: more than none makes it
    /// an assembly.
    pub uses: usize,
    /// The current review round, in brief — absent when it never had one.
    pub review: Option<ReviewSummary>,
    /// How many comments its discussion has.
    pub comments: usize,
    /// The open change order that names this revision, if one does.
    pub eco: Option<EcoRef>,
    /// This revision's thumbnail, current or stale; absent with none.
    pub thumbnail: Option<ThumbnailView>,
}

/// A revision's thumbnail in brief.
#[derive(Debug, Serialize)]
pub struct ThumbnailView {
    /// Whether it pictures the document as it is now. A stale one is not
    /// served; this says one existed.
    pub current: bool,
    /// Where to fetch it; empty when stale.
    pub url: String,
    pub width: u32,
    pub height: u32,
    pub renderer: String,
    pub content_hash: String,
}

/// The open change order holding a revision, in brief.
#[derive(Debug, Serialize)]
pub struct EcoRef {
    pub id: String,
    pub number: String,
    pub state: &'static str,
    /// `release` or `obsolete`.
    pub action: &'static str,
}

/// A review round in one line of the revision table.
#[derive(Debug, Serialize)]
pub struct ReviewSummary {
    pub status: &'static str,
    pub live: bool,
    /// Approvals given on the document as it is now: the ones that count.
    pub approvals: usize,
    /// Approvals given on an older version: they need renewing (D8).
    pub stale_approvals: usize,
    pub required_approvals: u32,
    /// Whether the person looking is one of its reviewers.
    pub may_decide: bool,
}

/// Where a generated revision came from, with the source resolved to a
/// number a person recognises.
#[derive(Debug, Serialize)]
pub struct Provenance {
    pub part_id: String,
    pub number: String,
    pub revision_label: String,
    pub values: std::collections::BTreeMap<String, String>,
}

pub async fn get_part(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
) -> Result<Json<PartDetail>, Error> {
    let user = require_user(&db, &headers)?;
    db.read(|state| {
        // An id, or a number as a person types it (D13).
        let part = state.part_by_id_or_number(&id).ok_or_else(|| Error::not_found("part"))?;
        let revision_views = part
            .revisions
            .iter()
            .map(|revision| view(state, part, revision, &user.id))
            .collect();
        let schema = match catalog::find(&state.categories, &part.category) {
            Some(category) => catalog::schema(&state.categories, &category.id)?,
            None => Vec::new(),
        };
        let inert = part
            .attributes
            .keys()
            .filter(|key| !schema.iter().any(|e| &e.def.key == *key))
            .cloned()
            .collect();
        Ok(Json(PartDetail {
            has_geometry: part.latest().is_some_and(|r| r.has_geometry),
            has_thumbnail: crate::thumbnail::of_part(part).is_some(),
            part: part.clone(),
            revision_views,
            suggested_label: suggest_label(part),
            category_path: catalog::path_name(&state.categories, &part.category),
            schema,
            inert,
            catalog_locked: state.settings.lock_released_attributes && part.catalog_frozen(),
            multiple_open_drafts: state.settings.allow_multiple_open_drafts,
            sourcing_view: crate::sourcing::resolved(state, part),
            newest_revision: part.latest().map(|r| NewestRevision {
                id: r.id.clone(),
                label: r.label.clone(),
                lifecycle: r.lifecycle.as_str(),
                document_key: r.document_key(&part.id),
            }),
            thumbnail_url: crate::thumbnail::part_url(part),
        }))
    })
}

fn view(
    state: &crate::db::State,
    part: &Part,
    revision: &Revision,
    viewer_id: &str,
) -> RevisionView {
    let name_of = |id: &str| {
        state
            .user(id)
            .map(|u| {
                if u.display_name.is_empty() {
                    u.username.clone()
                } else {
                    u.display_name.clone()
                }
            })
            .unwrap_or_else(|| "(removed user)".to_string())
    };
    RevisionView {
        id: revision.id.clone(),
        label: revision.label.clone(),
        lifecycle: revision.lifecycle.as_str(),
        editable: revision.lifecycle.is_editable(),
        content_hash: revision.content_hash.clone(),
        size: revision.size,
        created_at: revision.created_at,
        modified_at: revision.modified_at,
        locked_by: revision.lock.as_ref().map(|l| name_of(&l.user_id)),
        locked_by_me: revision
            .lock
            .as_ref()
            .is_some_and(|l| l.user_id == viewer_id),
        locked_at: revision.lock.as_ref().map(|l| l.acquired_at),
        released_by: revision.released_by.as_deref().map(name_of),
        released_at: revision.released_at,
        document_key: revision.document_key(&part.id),
        origin: revision.origin,
        family: revision.family.as_ref().map(|stamp| provenance(state, &stamp.family_part, &stamp.family_revision, &stamp.values)),
        template: revision
            .template
            .as_ref()
            .map(|stamp| provenance(state, &stamp.template_part, &stamp.template_revision, &stamp.values)),
        bake: revision.bake.as_ref().map(|b| b.status.as_str()),
        bake_error: revision.bake.as_ref().map(|b| b.error.clone()).unwrap_or_default(),
        hand_edited: revision.family.as_ref().is_some_and(|s| s.generated_hash != revision.content_hash),
        uses: revision.uses.len(),
        review: revision.review().map(|r| ReviewSummary {
            status: r.status.as_str(),
            live: r.is_live(),
            approvals: r.approvers(&revision.content_hash).len(),
            stale_approvals: r.stale_approvers(&revision.content_hash).len(),
            required_approvals: r.required_approvals,
            may_decide: r.is_live() && state.user(viewer_id).is_some_and(|u| crate::review::is_reviewer(r, u)),
        }),
        comments: revision.comments.len(),
        eco: crate::eco::standalone_holder(state, &part.id, &revision.id).map(|e| EcoRef {
            id: e.id.clone(),
            number: e.number.clone(),
            state: e.state.as_str(),
            action: e.item(&part.id, &revision.id).map(|i| i.action.as_str()).unwrap_or(""),
        }),
        thumbnail: revision.thumbnail.as_ref().map(|t| ThumbnailView {
            current: crate::thumbnail::current(revision).is_some(),
            url: crate::thumbnail::revision_url(&part.id, revision),
            width: t.width,
            height: t.height,
            renderer: t.renderer.clone(),
            content_hash: t.content_hash.clone(),
        }),
    }
}

fn provenance(
    state: &crate::db::State,
    part_id: &str,
    revision_id: &str,
    values: &std::collections::BTreeMap<String, String>,
) -> Provenance {
    let part = state.part(part_id);
    Provenance {
        part_id: part_id.to_string(),
        number: part.map(|p| p.number.clone()).unwrap_or_else(|| "(removed part)".into()),
        revision_label: part
            .and_then(|p| p.revision(revision_id))
            .map(|r| r.label.clone())
            .unwrap_or_else(|| "(deleted revision)".into()),
        values: values.clone(),
    }
}

#[derive(Debug, Deserialize)]
pub struct NewPart {
    pub part_type: String,
    pub name: String,
    #[serde(default)]
    pub description: String,
    #[serde(default)]
    pub category: String,
    /// The typed number. Refused by a counter type, required by free-text and
    /// pattern types, handed to the script of a script type.
    #[serde(default)]
    pub number: String,
    /// `normal`, `family` or `template` (or `.nBREP` / `.fBREP` / `.tBREP`).
    #[serde(default)]
    pub document_class: String,
    /// The first revision's label; empty takes `A`.
    #[serde(default)]
    pub label: String,
    #[serde(default)]
    pub tags: Vec<String>,
    /// Catalog values, checked against `category`'s schema.
    #[serde(default)]
    pub attributes: serde_json::Map<String, serde_json::Value>,
    /// Where the part comes from in another system (a KiCad library id);
    /// unique among parts of the same type (`409` naming the one that has it).
    #[serde(default)]
    pub external_ref: String,
    /// The first revision's origin: `authored` (default) or `imported`.
    #[serde(default)]
    pub origin: crate::model::Origin,
    /// The creator's workspace folder the new part is linked into; empty is
    /// the top ([`crate::workspace::link_on_create`]).
    #[serde(default)]
    pub workspace_folder: String,
    /// A machine import making many parts (the adoption importer, a library
    /// import): nothing is linked.
    #[serde(default)]
    pub bulk: bool,
}

/// Create a part. It may run the type's number script and the label script,
/// so it runs on the blocking pool — a script can wait on the network.
pub async fn create_part(
    State(db): State<Shared>,
    headers: HeaderMap,
    Json(body): Json<NewPart>,
) -> Result<Json<Part>, Error> {
    let user = require_author(&db, &headers)?;
    let document_class = DocumentClass::parse(&body.document_class).ok_or_else(|| {
        Error::bad_request(format!(
            "'{}' is not a document class — normal, family or template",
            body.document_class
        ))
    })?;
    let spec = PartSpec {
        part_type: body.part_type,
        number: body.number,
        name: body.name,
        description: body.description,
        category: body.category,
        document_class,
        label: body.label,
        tags: body.tags,
        attributes: body.attributes,
        exact: false,
        external_ref: body.external_ref,
        origin: body.origin,
        workspace: crate::workspace::link_on_create(body.origin, body.bulk, &body.workspace_folder),
    };
    let part = blocking(move || db.create_part_with(&user, &spec)).await?;
    Ok(Json(part))
}

/// Change a part's name, description, category, tags or attribute values
/// (`attributes` merges; `null` clears a key). The author group and up, the
/// same people who create parts. The number, type and document class are
/// identity and are refused ([`crate::db::Db::update_part`]).
///
/// No lock is needed: these are the PART's fields, not a revision's document,
/// and checkout guards documents.
pub async fn update_part(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Json(body): Json<serde_json::Value>,
) -> Result<Json<Part>, Error> {
    require_author(&db, &headers)?;
    // Only by id here: a part number in a URL could collide with an id.
    if db.read(|state| state.part(&id).is_none()) {
        return Err(Error::not_found("part"));
    }
    Ok(Json(db.update_part(&id, &body)?))
}

#[derive(Debug, Deserialize, Default)]
pub struct NewRevision {
    /// Free text; empty takes the suggestion.
    #[serde(default)]
    pub label: String,
    /// `authored` (default) or `imported` — a library re-import's revision.
    #[serde(default)]
    pub origin: crate::model::Origin,
}

pub async fn create_revision(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    body: Option<Json<NewRevision>>,
) -> Result<Json<Revision>, Error> {
    let user = require_author(&db, &headers)?;
    let (label, origin) = body.map(|Json(b)| (b.label, b.origin)).unwrap_or_default();
    let revision = blocking(move || db.create_revision_from(&user, &id, &label, origin)).await?;
    Ok(Json(revision))
}

#[derive(Debug, Deserialize, Default)]
pub struct CheckoutBody {
    /// Which client of this user is taking the lock. The same person can hold
    /// a browser and a desktop session at once, and the break dialog has to be
    /// able to say which.
    #[serde(default)]
    pub client_id: String,
}

pub async fn checkout(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((id, revision)): Path<(String, String)>,
    body: Option<Json<CheckoutBody>>,
) -> Result<Response, Error> {
    let user = require_author(&db, &headers)?;
    let client = body.map(|Json(b)| b.client_id).unwrap_or_default();
    let client = if client.trim().is_empty() { "web".to_string() } else { client };
    db.checkout(&user, &id, &revision, &client)?;
    Ok(ok_seq(&db))
}

#[derive(Debug, Deserialize, Default)]
pub struct CheckinBody {
    /// Break someone else's lock. Needs the check-in group; the server checks,
    /// not the page.
    #[serde(default)]
    pub force: bool,
}

pub async fn checkin(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((id, revision)): Path<(String, String)>,
    body: Option<Json<CheckinBody>>,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let force = body.map(|Json(b)| b.force).unwrap_or(false);
    db.checkin(&user, &id, &revision, force)?;
    Ok(ok_seq(&db))
}

#[derive(Debug, Deserialize)]
pub struct StateBody {
    pub to: String,
}

pub async fn set_state(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((id, revision)): Path<(String, String)>,
    Json(body): Json<StateBody>,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let to = match body.to.trim().to_ascii_lowercase().as_str() {
        "draft" => Lifecycle::Draft,
        "inreview" | "in-review" | "review" => Lifecycle::InReview,
        "released" | "release" => Lifecycle::Released,
        "superseded" => Lifecycle::Superseded,
        "obsolete" => Lifecycle::Obsolete,
        other => return Err(Error::bad_request(format!("'{other}' is not a lifecycle state"))),
    };
    let handle = db.clone();
    let warnings = blocking(move || handle.transition(&user, &id, &revision, to)).await?;
    let seq = db.read(|state| state.seq);
    Ok(Json(serde_json::json!({ "ok": true, "seq": seq, "warnings": warnings })).into_response())
}

pub async fn delete_revision(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((id, revision)): Path<(String, String)>,
) -> Result<Response, Error> {
    let user = require_author(&db, &headers)?;
    db.delete_draft(&user, &id, &revision)?;
    Ok(ok_seq(&db))
}
