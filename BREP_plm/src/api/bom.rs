//! Assembly structure over HTTP: a revision's uses list, its BOM, the diff
//! between two revisions' lists, and where a part is used.
//!
//! The rules are in [`crate::bom`] and [`crate::db::Db::set_uses`]; these
//! handlers decide only who is asking and how the answer is shaped. The CAD
//! app will write the uses list through the same `PUT` the web page uses.

use axum::extract::{Path, Query, State};
use axum::http::{header, HeaderMap};
use axum::response::{IntoResponse, Response};
use axum::Json;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{require_author, require_user, Shared};
use crate::bom;
use crate::db::State as Store;
use crate::model::{Part, Revision};
use crate::Error;

/// The part named by id or number, and one of its revisions by id or label.
fn locate<'a>(state: &'a Store, part: &str, revision: &str) -> Result<(&'a Part, &'a Revision), Error> {
    let part = state.part_by_id_or_number(part).ok_or_else(|| Error::not_found("part"))?;
    let revision = bom::find_revision(part, revision).ok_or_else(|| Error::not_found("revision"))?;
    Ok((part, revision))
}

/// `levels=all` or a count; absent or `0` is every level.
fn levels(text: &str) -> Result<usize, Error> {
    match text.trim() {
        "" | "all" | "0" => Ok(0),
        n => n
            .parse::<usize>()
            .map_err(|_| Error::bad_request(format!("levels must be a number or 'all', not '{n}'"))),
    }
}

/// What the list says, with each child resolved to a number and label.
pub async fn get_uses(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((id, rev)): Path<(String, String)>,
) -> Result<Json<Value>, Error> {
    require_user(&db, &headers)?;
    db.read(|state| {
        let (_, revision) = locate(state, &id, &rev)?;
        Ok(Json(json!({ "uses": bom::resolved_uses(state, revision) })))
    })
}

/// Replace the list. The author group, holding the revision's lock, on a
/// revision in work — the same conditions as saving its document.
pub async fn put_uses(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((id, rev)): Path<(String, String)>,
    Json(body): Json<Value>,
) -> Result<Json<Value>, Error> {
    let user = require_author(&db, &headers)?;
    let (part_id, revision_id) = db.read(|state| {
        let (part, revision) = locate(state, &id, &rev)?;
        Ok::<_, Error>((part.id.clone(), revision.id.clone()))
    })?;
    db.set_uses(&user, &part_id, &revision_id, &body)?;
    db.read(|state| {
        let (_, revision) = locate(state, &part_id, &revision_id)?;
        Ok(Json(json!({ "ok": true, "seq": state.seq, "uses": bom::resolved_uses(state, revision) })))
    })
}

#[derive(Debug, Deserialize, Default)]
pub struct BomQuery {
    #[serde(default)]
    pub levels: String,
    #[serde(default)]
    pub flat: bool,
    /// `csv` for a download; anything else is JSON.
    #[serde(default)]
    pub format: String,
    /// Add an `Attachments` column to the CSV.
    #[serde(default)]
    pub attachments: bool,
    #[serde(default)]
    pub occurrences: bool,
}

pub async fn get_bom(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((id, rev)): Path<(String, String)>,
    Query(query): Query<BomQuery>,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let depth = levels(&query.levels)?;
    let result = db.read(|state| {
        let (part, revision) = locate(state, &id, &rev)?;
        let mut result = bom::bom(state, part, revision, depth, if query.occurrences {false} else {query.flat});
        if query.occurrences { result.comparison_lines=Some(result.lines.clone()); crate::bom_config::expand_bom(state, &mut result, query.flat); }
        Ok::<_, Error>(result)
    })?;
    if query.format.eq_ignore_ascii_case("csv") {
        let name = format!(
            "{}-{}-bom{}.csv",
            result.number,
            result.revision_label,
            if result.flat { "-flat" } else { "" }
        )
        .replace(|c: char| !(c.is_ascii_alphanumeric() || c == '-' || c == '_' || c == '.'), "_");
        return Ok((
            [
                (header::CONTENT_TYPE, "text/csv; charset=utf-8".to_string()),
                (header::CONTENT_DISPOSITION, format!("attachment; filename=\"{name}\"")),
            ],
            db.read(|state| {
                let layout=state.bom_configuration.selections.get(&user.id).and_then(|id|state.bom_configuration.layouts.iter().find(|l|l.id==*id&&(l.owner.is_none()||l.owner.as_deref()==Some(&user.id))));
                layout.filter(|_|query.occurrences).map(|layout|crate::bom_config::layout_csv(&result,layout,&crate::bom_config::fields(state))).unwrap_or_else(||bom::bom_csv_with(&result,query.attachments))
            }),
        )
            .into_response());
    }
    Ok(Json(result).into_response())
}

#[derive(Debug, Deserialize)]
pub struct DiffQuery {
    pub from: String,
    pub to: String,
}

/// What changed in the uses list between two revisions of one part.
pub async fn diff(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Query(query): Query<DiffQuery>,
) -> Result<Json<bom::BomDiff>, Error> {
    require_user(&db, &headers)?;
    db.read(|state| {
        let (part, from) = locate(state, &id, &query.from)?;
        let to = bom::find_revision(part, &query.to).ok_or_else(|| Error::not_found("revision"))?;
        Ok(Json(bom::diff(state, part, from, to)))
    })
}

#[derive(Debug, Deserialize, Default)]
pub struct WhereUsedQuery {
    /// A revision id or label: only uses that point at it. Empty: any.
    #[serde(default)]
    pub revision: String,
    #[serde(default)]
    pub levels: String,
}

pub async fn where_used(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Query(query): Query<WhereUsedQuery>,
) -> Result<Json<bom::WhereUsed>, Error> {
    require_user(&db, &headers)?;
    let depth = levels(&query.levels)?;
    db.read(|state| {
        let part = state.part_by_id_or_number(&id).ok_or_else(|| Error::not_found("part"))?;
        let revision = match query.revision.trim() {
            "" => None,
            key => Some(bom::find_revision(part, key).ok_or_else(|| Error::not_found("revision"))?),
        };
        Ok(Json(bom::where_used(state, part, revision, depth)))
    })
}

/// Replace a part (or one revision of it) in every assembly that uses it
/// ([`crate::replace`]). `dry_run` answers what would happen and writes
/// nothing. The author group.
pub async fn replace(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Json(request): Json<crate::replace::ReplaceRequest>,
) -> Result<Json<crate::replace::ReplaceReport>, Error> {
    let user = require_author(&db, &headers)?;
    Ok(Json(super::blocking(move || db.replace_everywhere(&user, &id, &request)).await?))
}
