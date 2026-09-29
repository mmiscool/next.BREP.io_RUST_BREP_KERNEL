//! The catalog tree: every signed-in user reads it, administrators shape it.
//!
//! The rules — inheritance, no redefinition, no cycles, no deleting what is
//! in use — are in [`crate::catalog`] and [`crate::db::Db`]'s category
//! operations; these handlers decide only who is asking.

use axum::extract::{Path, State};
use axum::http::HeaderMap;
use axum::response::Response;
use axum::Json;
use serde::{Deserialize, Serialize};

use super::{ok_seq, require_admin, require_user, Shared};
use crate::catalog::{self, Effective};
use crate::db::CategoryChange;
use crate::model::{AttributeDef, Category};
use crate::Error;

/// A category as the page lists it: its own record plus where it sits.
#[derive(Debug, Serialize)]
pub struct CategoryView {
    #[serde(flatten)]
    pub category: Category,
    /// "Fasteners / Screws".
    pub path: String,
    /// 0 for a top-level category.
    pub depth: usize,
    /// Parts filed directly in this category (not in its sub-categories).
    pub parts: usize,
    /// Parts in this category and every category beneath it.
    pub parts_within: usize,
}

/// The whole tree, depth-first with siblings by name — the order the page
/// draws it in, so the page does no tree-building of its own.
pub async fn list(
    State(db): State<Shared>,
    headers: HeaderMap,
) -> Result<Json<Vec<CategoryView>>, Error> {
    require_user(&db, &headers)?;
    Ok(Json(db.read(|state| {
        let categories = &state.categories;
        let mut out = Vec::with_capacity(categories.len());
        let mut roots: Vec<&Category> = categories
            .iter()
            .filter(|c| c.parent.is_empty() || catalog::find(categories, &c.parent).is_none())
            .collect();
        roots.sort_by_key(|c| c.name.to_ascii_lowercase());
        let mut stack: Vec<(&Category, usize)> = roots.into_iter().rev().map(|c| (c, 0)).collect();
        let mut seen = std::collections::BTreeSet::new();
        while let Some((category, depth)) = stack.pop() {
            if !seen.insert(category.id.clone()) {
                continue;
            }
            let within = catalog::descendants(categories, &category.id);
            let count = |ids: &dyn Fn(&str) -> bool| {
                state.parts.iter().filter(|p| ids(&p.category.to_ascii_lowercase())).count()
            };
            let own = category.id.to_ascii_lowercase();
            out.push(CategoryView {
                category: category.clone(),
                path: catalog::path_name(categories, &category.id),
                depth,
                parts: count(&|id| id == own),
                parts_within: count(&|id| within.contains(id)),
            });
            let mut children: Vec<&Category> = categories
                .iter()
                .filter(|c| c.parent.eq_ignore_ascii_case(&category.id))
                .collect();
            children.sort_by_key(|c| c.name.to_ascii_lowercase());
            stack.extend(children.into_iter().rev().map(|c| (c, depth + 1)));
        }
        out
    })))
}

#[derive(Debug, Deserialize)]
pub struct NewCategory {
    pub id: String,
    pub name: String,
    #[serde(default)]
    pub parent: String,
    #[serde(default)]
    pub attributes: Vec<AttributeDef>,
}

pub async fn create(
    State(db): State<Shared>,
    headers: HeaderMap,
    Json(body): Json<NewCategory>,
) -> Result<Json<Category>, Error> {
    require_admin(&db, &headers)?;
    Ok(Json(db.create_category(
        &body.id,
        CategoryChange {
            name: Some(body.name),
            parent: Some(body.parent),
            attributes: Some(body.attributes),
        },
    )?))
}

#[derive(Debug, Deserialize)]
pub struct Change {
    #[serde(default)]
    pub name: Option<String>,
    /// `""` moves the category to the top level.
    #[serde(default)]
    pub parent: Option<String>,
    /// Replaces the category's own attribute list.
    #[serde(default)]
    pub attributes: Option<Vec<AttributeDef>>,
}

pub async fn update(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Json(body): Json<Change>,
) -> Result<Json<Category>, Error> {
    require_admin(&db, &headers)?;
    Ok(Json(db.update_category(
        &id,
        CategoryChange { name: body.name, parent: body.parent, attributes: body.attributes },
    )?))
}

pub async fn remove(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
) -> Result<Response, Error> {
    require_admin(&db, &headers)?;
    db.delete_category(&id)?;
    Ok(ok_seq(&db))
}

/// A category's effective schema: what a part filed in it can carry.
#[derive(Debug, Serialize)]
pub struct Schema {
    pub category: String,
    pub path: String,
    pub attributes: Vec<Effective>,
}

pub async fn schema(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
) -> Result<Json<Schema>, Error> {
    require_user(&db, &headers)?;
    db.read(|state| {
        let category = catalog::find(&state.categories, &id)
            .ok_or_else(|| Error::not_found(format!("category '{id}'")))?;
        Ok(Json(Schema {
            category: category.id.clone(),
            path: catalog::path_name(&state.categories, &category.id),
            attributes: catalog::schema(&state.categories, &category.id)?,
        }))
    })
}
