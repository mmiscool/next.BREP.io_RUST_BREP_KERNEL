//! Sourcing over HTTP: the manufacturer and supplier lists, and a part's
//! manufacturer parts and offers.
//!
//! Who may do what:
//! - everyone signed in reads;
//! - the author group adds and edits companies, and edits a part's sourcing —
//!   a buyer adding a second source is ordinary work, not administration;
//! - only an administrator deletes a company, and only one nothing names.
//!
//! The rules are in [`crate::sourcing`]; these handlers decide who is asking.

use axum::extract::{Path, State};
use axum::http::HeaderMap;
use axum::response::Response;
use axum::Json;
use serde::Serialize;
use serde_json::Value;

use super::{ok_seq, require_admin, require_author, require_user, Shared};
use crate::model::{Company, ManufacturerPart, SupplierOffer};
use crate::sourcing::{self, Companies};
use crate::Error;

/// A company as the page lists it, with how much sourcing names it.
#[derive(Debug, Serialize)]
pub struct CompanyRow {
    #[serde(flatten)]
    pub company: Company,
    /// Manufacturer parts (for a manufacturer) or offers (for a supplier)
    /// that name it.
    pub uses: usize,
    /// Distinct parts those belong to.
    pub parts: usize,
}

fn rows(db: &Shared, which: Companies) -> Vec<CompanyRow> {
    db.read(|state| {
        let mut out: Vec<CompanyRow> = which
            .list(state)
            .iter()
            .map(|company| CompanyRow {
                company: company.clone(),
                uses: sourcing::uses(state, which, &company.id),
                parts: state
                    .parts
                    .iter()
                    .filter(|p| {
                        p.sourcing.iter().any(|mp| match which {
                            Companies::Manufacturers => mp.manufacturer == company.id,
                            Companies::Suppliers => mp.offers.iter().any(|o| o.supplier == company.id),
                        })
                    })
                    .count(),
            })
            .collect();
        out.sort_by_key(|row| row.company.name.to_ascii_lowercase());
        out
    })
}

macro_rules! company_routes {
    ($which:expr, $list:ident, $create:ident, $update:ident, $remove:ident) => {
        pub async fn $list(State(db): State<Shared>, headers: HeaderMap) -> Result<Json<Vec<CompanyRow>>, Error> {
            require_user(&db, &headers)?;
            Ok(Json(rows(&db, $which)))
        }

        pub async fn $create(
            State(db): State<Shared>,
            headers: HeaderMap,
            Json(body): Json<Value>,
        ) -> Result<Json<Company>, Error> {
            require_author(&db, &headers)?;
            Ok(Json(db.create_company($which, &body)?))
        }

        pub async fn $update(
            State(db): State<Shared>,
            headers: HeaderMap,
            Path(id): Path<String>,
            Json(body): Json<Value>,
        ) -> Result<Json<Company>, Error> {
            require_author(&db, &headers)?;
            Ok(Json(db.update_company($which, &id, &body)?))
        }

        pub async fn $remove(
            State(db): State<Shared>,
            headers: HeaderMap,
            Path(id): Path<String>,
        ) -> Result<Response, Error> {
            require_admin(&db, &headers)?;
            db.delete_company($which, &id)?;
            Ok(ok_seq(&db))
        }
    };
}

company_routes!(Companies::Manufacturers, list_manufacturers, create_manufacturer, update_manufacturer, delete_manufacturer);
company_routes!(Companies::Suppliers, list_suppliers, create_supplier, update_supplier, delete_supplier);

/// Only by part id in these paths — a part number in a URL could collide
/// with an id.
fn known_part(db: &Shared, id: &str) -> Result<(), Error> {
    if db.read(|state| state.part(id).is_none()) {
        return Err(Error::not_found("part"));
    }
    Ok(())
}

pub async fn add_manufacturer_part(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(part): Path<String>,
    Json(body): Json<Value>,
) -> Result<Json<ManufacturerPart>, Error> {
    require_author(&db, &headers)?;
    known_part(&db, &part)?;
    Ok(Json(db.add_manufacturer_part(&part, &body)?))
}

pub async fn update_manufacturer_part(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((part, mp)): Path<(String, String)>,
    Json(body): Json<Value>,
) -> Result<Json<ManufacturerPart>, Error> {
    require_author(&db, &headers)?;
    known_part(&db, &part)?;
    Ok(Json(db.update_manufacturer_part(&part, &mp, &body)?))
}

pub async fn delete_manufacturer_part(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((part, mp)): Path<(String, String)>,
) -> Result<Response, Error> {
    require_author(&db, &headers)?;
    known_part(&db, &part)?;
    db.delete_manufacturer_part(&part, &mp)?;
    Ok(ok_seq(&db))
}

pub async fn add_offer(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((part, mp)): Path<(String, String)>,
    Json(body): Json<Value>,
) -> Result<Json<SupplierOffer>, Error> {
    require_author(&db, &headers)?;
    known_part(&db, &part)?;
    Ok(Json(db.add_offer(&part, &mp, &body)?))
}

pub async fn update_offer(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((part, mp, offer)): Path<(String, String, String)>,
    Json(body): Json<Value>,
) -> Result<Json<SupplierOffer>, Error> {
    require_author(&db, &headers)?;
    known_part(&db, &part)?;
    Ok(Json(db.update_offer(&part, &mp, &offer, &body)?))
}

pub async fn delete_offer(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((part, mp, offer)): Path<(String, String, String)>,
) -> Result<Response, Error> {
    require_author(&db, &headers)?;
    known_part(&db, &part)?;
    db.delete_offer(&part, &mp, &offer)?;
    Ok(ok_seq(&db))
}
