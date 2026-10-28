//! Manufacturing instructions on the PLM: which instructions revisions
//! instruct an assembly ([`crate::model::Revision::instructs`]).

use axum::extract::{Path, State};
use axum::http::HeaderMap;
use axum::Json;
use serde::Serialize;

use super::{require_user, Shared};
use crate::model::DocumentClass;
use crate::Error;

/// One instructions revision that instructs a revision of the assembly.
#[derive(Debug, Serialize)]
pub struct Instructing {
    pub part: String,
    pub number: String,
    pub name: String,
    pub revision: String,
    pub label: String,
    pub state: String,
    pub target_revision: String,
    pub target_label: String,
    /// The assembly has a newer revision than the one instructed: the
    /// instructions want a re-target.
    pub stale: bool,
}

/// `GET /api/parts/:id/instructions`: every instructions revision that
/// instructs any revision of this assembly, by instructions number, oldest
/// revision first.
pub async fn list(State(db): State<Shared>, headers: HeaderMap, Path(id): Path<String>) -> Result<Json<Vec<Instructing>>, Error> {
    require_user(&db, &headers)?;
    db.read(|state| {
        let assembly = state.part_by_id_or_number(&id).ok_or_else(|| Error::not_found("part"))?;
        let latest = assembly.latest().map(|r| r.id.as_str());
        let mut rows = Vec::new();
        for part in state.parts.iter().filter(|p| p.document_class == DocumentClass::Instructions) {
            for revision in &part.revisions {
                let Some(target) = revision.instructs.as_ref().filter(|t| t.part == assembly.id) else { continue };
                rows.push(Instructing {
                    part: part.id.clone(),
                    number: part.number.clone(),
                    name: part.name.clone(),
                    revision: revision.id.clone(),
                    label: revision.label.clone(),
                    state: revision.lifecycle.as_str().into(),
                    target_revision: target.revision.clone(),
                    target_label: assembly.revision(&target.revision).map_or_else(String::new, |r| r.label.clone()),
                    stale: latest.is_some_and(|latest| latest != target.revision),
                });
            }
        }
        rows.sort_by(|a, b| a.number.cmp(&b.number));
        Ok(Json(rows))
    })
}
