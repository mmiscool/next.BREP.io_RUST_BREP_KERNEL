use super::{require_admin, require_author, require_user, Shared};
use crate::{
    bom_config::{self, Layout},
    model::AttributeDef,
    Error,
};
use axum::{
    extract::{Path, Query, State},
    http::HeaderMap,
    Json,
};
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize, Default)]
pub struct SnapshotQuery {
    #[serde(default)]
    pub keys: String,
}
pub async fn configuration(
    State(db): State<Shared>,
    headers: HeaderMap,
    Query(query): Query<SnapshotQuery>,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    Ok(Json(db.read(|state|json!({
        "fields":bom_config::fields(state),
        "part_fields":state.bom_configuration.part_fields,
        "occurrence_fields":state.bom_configuration.occurrence_fields,
        "layouts":state.bom_configuration.layouts.iter().filter(|l|l.owner.is_none()||l.owner.as_deref()==Some(&user.id)).collect::<Vec<_>>(),
        "selected":state.bom_configuration.selections.get(&user.id),
        "is_admin":user.is_admin(),
        "snapshots": query.keys.split(',').filter_map(|key| {
            let (part,rev)=crate::identity::split_key(key)?;
            let p=state.part(&part)?;let r=p.revision(&rev)?;
            Some((key.to_string(),json!({"part_type":p.part_type,"record":{"name":p.name,"description":p.description},"attributes":r.attributes,"occurrences":r.occurrences,"editable":user.can_author()&&r.lifecycle.is_editable()&&r.lock.as_ref().is_none_or(|l|l.user_id==user.id),"content_hash":r.content_hash})))
        }).collect::<std::collections::BTreeMap<_,_>>(),
    }))))
}
#[derive(Deserialize)]
pub struct Definitions {
    pub fields: Vec<AttributeDef>,
}
pub async fn type_fields(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
) -> Result<Json<Value>, Error> {
    require_user(&db, &headers)?;
    db.read(|state|{
        if !state.part_types.iter().any(|t|t.id==id){return Err(Error::not_found("part type"))}
        Ok(Json(json!({"fields":state.bom_configuration.part_fields.get(&id).cloned().unwrap_or_default()})))
    })
}
pub async fn set_type_fields(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Json(body): Json<Definitions>,
) -> Result<Json<Value>, Error> {
    require_admin(&db, &headers)?;
    db.set_type_fields(&id, body.fields)?;
    Ok(Json(json!({"ok":true})))
}
pub async fn set_occurrence_fields(
    State(db): State<Shared>,
    headers: HeaderMap,
    Json(body): Json<Definitions>,
) -> Result<Json<Value>, Error> {
    require_admin(&db, &headers)?;
    db.set_occurrence_fields(body.fields)?;
    Ok(Json(json!({"ok":true})))
}
#[derive(Deserialize)]
pub struct LayoutRequest {
    #[serde(default)]
    pub id: String,
    pub name: String,
    pub columns: Vec<String>,
    #[serde(default)]
    pub shared: bool,
}
pub async fn save_layout(
    State(db): State<Shared>,
    headers: HeaderMap,
    Json(body): Json<LayoutRequest>,
) -> Result<Json<Layout>, Error> {
    let user = require_user(&db, &headers)?;
    Ok(Json(db.save_layout(
        &user,
        &body.id,
        &body.name,
        body.columns,
        body.shared,
    )?))
}
pub async fn delete_layout(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    db.mutate(|state| {
        let at = state
            .bom_configuration
            .layouts
            .iter()
            .position(|l| l.id == id)
            .ok_or_else(|| Error::not_found("BOM configuration"))?;
        let l = &state.bom_configuration.layouts[at];
        if l.owner.as_deref().is_some_and(|owner| owner != user.id)
            || (l.owner.is_none() && !user.is_admin())
        {
            return Err(Error::forbidden(
                "This configuration belongs to another user",
            ));
        }
        state.bom_configuration.layouts.remove(at);
        state
            .bom_configuration
            .selections
            .retain(|_, selected| selected != &id);
        Ok(())
    })?;
    Ok(Json(json!({"ok":true})))
}
#[derive(Deserialize)]
pub struct Selection {
    pub id: String,
}
pub async fn select_layout(
    State(db): State<Shared>,
    headers: HeaderMap,
    Json(body): Json<Selection>,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    db.mutate(|state| {
        if !body.id.is_empty()
            && !state.bom_configuration.layouts.iter().any(|l| {
                l.id == body.id && (l.owner.is_none() || l.owner.as_deref() == Some(&user.id))
            })
        {
            return Err(Error::not_found("BOM configuration"));
        }
        state
            .bom_configuration
            .selections
            .insert(user.id.clone(), body.id);
        Ok(())
    })?;
    Ok(Json(json!({"ok":true})))
}
pub async fn revision_attributes(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((part, rev)): Path<(String, String)>,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    db.read(|state|{
        let p=state.part_by_id_or_number(&part).ok_or_else(||Error::not_found("part"))?;
        let revision=crate::bom::find_revision(p,&rev).ok_or_else(||Error::not_found("revision"))?;
        Ok(Json(json!({"part_type":p.part_type,"attributes":revision.attributes,"fields":state.bom_configuration.part_fields.get(&p.part_type).cloned().unwrap_or_default(),"occurrences":revision.occurrences,
            "editable":user.can_author() && revision.lifecycle.is_editable() && revision.lock.as_ref().is_none_or(|l|l.user_id==user.id),
            "record_fields":[
                {"key":"number","name":"Part number","type":"text","value":p.number,"editable":false},
                {"key":"name","name":"Name","type":"text","value":p.name,"editable":user.can_author()},
                {"key":"description","name":"Description","type":"text","value":p.description,"editable":user.can_author()},
                {"key":"part_type","name":"Part type","type":"text","value":p.part_type,"editable":false}
            ]})))
    })
}
#[derive(Deserialize)]
pub struct AttributeChanges {
    pub attributes: Value,
}
pub async fn patch_revision_attributes(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((part, rev)): Path<(String, String)>,
    Json(body): Json<AttributeChanges>,
) -> Result<Json<Value>, Error> {
    let user = require_author(&db, &headers)?;
    db.patch_revision_attributes(&user, &part, &rev, &body.attributes)?;
    Ok(Json(json!({"ok":true})))
}
#[derive(Deserialize)]
pub struct OccurrenceChanges {
    pub ids: Vec<String>,
    pub attributes: Value,
}
pub async fn patch_occurrences(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((part, rev)): Path<(String, String)>,
    Json(body): Json<OccurrenceChanges>,
) -> Result<Json<Value>, Error> {
    let user = require_author(&db, &headers)?;
    db.patch_occurrences(&user, &part, &rev, &body.ids, &body.attributes)?;
    Ok(Json(json!({"ok":true})))
}
