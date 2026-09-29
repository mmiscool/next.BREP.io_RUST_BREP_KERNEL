//! Reading the audit log ([`crate::audit`]).
//!
//! An administrator reads all of it. Everyone else reads the events on parts
//! and their revisions — the records every signed-in user can already see —
//! and nothing about users, tokens, sign-ins, settings or scripts.

use axum::extract::{Path, Query, State};
use axum::http::{header, HeaderMap};
use axum::response::{IntoResponse, Response};
use axum::Json;
use serde::Deserialize;

use super::{require_user, Shared};
use crate::audit::{self, AuditFilter};
use crate::Error;

/// The most one query returns; a caller pages with `before`.
pub const MAX_LIMIT: usize = 5000;
const DEFAULT_LIMIT: usize = 200;

#[derive(Debug, Deserialize, Default)]
pub struct Format {
    /// `csv` for a download.
    #[serde(default)]
    pub format: String,
}

/// `GET /api/audit?kind=&entity=&part=&user=&action=&since=&until=&before=&limit=&format=csv`
pub async fn query(
    State(db): State<Shared>,
    headers: HeaderMap,
    Query(mut filter): Query<AuditFilter>,
    Query(format): Query<Format>,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    if !user.is_admin() {
        filter.kinds = vec!["part".into(), "revision".into()];
    }
    answer(&db, filter, &format.format)
}

/// `GET /api/parts/:id/history`: the part's events and its revisions'.
pub async fn part_history(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Query(mut filter): Query<AuditFilter>,
    Query(format): Query<Format>,
) -> Result<Response, Error> {
    require_user(&db, &headers)?;
    let part_id = db
        .read(|state| state.part_by_id_or_number(&id).map(|p| p.id.clone()))
        .ok_or_else(|| Error::not_found("part"))?;
    filter.part = part_id;
    filter.kinds = vec!["part".into(), "revision".into()];
    answer(&db, filter, &format.format)
}

fn answer(db: &crate::db::Db, mut filter: AuditFilter, format: &str) -> Result<Response, Error> {
    let limit = filter.limit.unwrap_or(DEFAULT_LIMIT);
    if limit == 0 || limit > MAX_LIMIT {
        return Err(Error::bad_request(format!("limit must be between 1 and {MAX_LIMIT}")));
    }
    filter.limit = Some(limit);
    let events = db.audit_query(&filter)?;
    if format == "csv" {
        return Ok((
            [
                (header::CONTENT_TYPE, "text/csv; charset=utf-8"),
                (header::CONTENT_DISPOSITION, "attachment; filename=\"plm-audit.csv\""),
            ],
            audit::to_csv(&events),
        )
            .into_response());
    }
    Ok(Json(events).into_response())
}
