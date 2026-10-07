//! Revision and occurrence fields and the BOM layouts shared by CAD and PLM.
use crate::{
    auth, catalog,
    db::{find_revision_mut, Db, State},
    model::{AttributeDef, BakeStatus, Revision, User},
    Error,
};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Configuration {
    #[serde(default)]
    pub part_fields: BTreeMap<String, Vec<AttributeDef>>,
    #[serde(default)]
    pub occurrence_fields: Vec<AttributeDef>,
    #[serde(default)]
    pub layouts: Vec<Layout>,
    #[serde(default)]
    pub selections: BTreeMap<String, String>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Layout {
    pub id: String,
    pub name: String,
    /// None is shared; otherwise this is a user's private configuration.
    pub owner: Option<String>,
    pub columns: Vec<String>,
}
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Occurrence {
    pub id: String,
    pub part: String,
    pub revision: String,
    #[serde(default)]
    pub attributes: BTreeMap<String, Value>,
}
/// The editable resource and payload key are part of the server field registry.
#[derive(Debug, Clone, Serialize)]
pub struct EditTarget {
    pub resource: String,
    pub key: String,
}
#[derive(Debug, Clone, Serialize)]
pub struct Field {
    pub id: String,
    pub scope: String,
    pub part_type: Option<String>,
    pub key: String,
    pub name: String,
    pub editable: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub edit: Option<EditTarget>,
    #[serde(flatten)]
    pub kind: crate::model::AttributeKind,
    pub cad_field: String,
}

pub fn fields(state: &State) -> Vec<Field> {
    use crate::model::AttributeKind;
    let mut out = Vec::new();
    for (key, name, cad) in [
        ("number", "Part number", "Part_Number"),
        ("name", "Name", "Description"),
        ("revision_label", "Revision", "Revision"),
        ("state", "State", "Lifecycle_State"),
        ("quantity", "Quantity", "Quantity"),
        ("total", "Total quantity", "Total"),
        ("part_type", "Part type", "Part_Type"),
        ("unit", "Unit", "Unit"),
        (
            "mpn",
            "Manufacturer part number",
            "Manufacturer_Part_Number",
        ),
        ("manufacturer", "Manufacturer", "Manufacturer"),
        ("supplier", "Supplier", "Supplier"),
        ("spn", "Supplier part number", "Supplier_Part_Number"),
        ("unit_price", "Unit price", "Unit_Price"),
        ("extended", "Extended price", "Extended_Price"),
    ] {
        out.push(Field {
            id: format!("builtin.{key}"),
            scope: "builtin".into(),
            part_type: None,
            key: key.into(),
            name: name.into(),
            editable: key == "name",
            edit: (key == "name").then(|| EditTarget {
                resource: "part".into(),
                key: key.into(),
            }),
            kind: AttributeKind::Text,
            cad_field: cad.into(),
        });
    }
    for (key, name, cad) in [
        ("find_number", "Callout / find number", "Find_Number"),
        ("reference", "Reference designator", "Reference_Designator"),
        ("notes", "Installation notes", "Notes"),
    ] {
        out.push(Field {
            id: format!("builtin.{key}"),
            scope: "occurrence".into(),
            part_type: None,
            key: key.into(),
            name: name.into(),
            editable: true,
            edit: Some(EditTarget {
                resource: "occurrence".into(),
                key: cad.into(),
            }),
            kind: AttributeKind::Text,
            cad_field: cad.into(),
        });
    }
    for kind in &state.part_types {
        for def in state
            .bom_configuration
            .part_fields
            .get(&kind.id)
            .into_iter()
            .flatten()
        {
            out.push(Field {
                id: format!("part.{}.{}", kind.id, def.key),
                scope: "part".into(),
                part_type: Some(kind.id.clone()),
                key: def.key.clone(),
                name: format!("{} ({})", def.name, kind.name),
                editable: true,
                edit: Some(EditTarget {
                    resource: "revision".into(),
                    key: def.key.clone(),
                }),
                kind: def.kind.clone(),
                cad_field: def.key.clone(),
            });
        }
    }
    for def in &state.bom_configuration.occurrence_fields {
        out.push(Field {
            id: format!("occurrence.{}", def.key),
            scope: "occurrence".into(),
            part_type: None,
            key: def.key.clone(),
            name: def.name.clone(),
            editable: true,
            edit: Some(EditTarget {
                resource: "occurrence".into(),
                key: def.key.clone(),
            }),
            kind: def.kind.clone(),
            cad_field: def.key.clone(),
        });
    }
    out
}

pub fn checked_values(
    defs: &[AttributeDef],
    existing: &mut BTreeMap<String, Value>,
    changes: &Value,
) -> Result<(), Error> {
    let changes = changes
        .as_object()
        .ok_or_else(|| Error::bad_request("attributes must be an object"))?;
    for (key, value) in changes {
        let def = defs
            .iter()
            .find(|def| def.key == *key)
            .ok_or_else(|| Error::bad_request(format!("Unknown attribute '{key}'")))?;
        match catalog::check_value(def, value).map_err(Error::bad_request)? {
            Some(value) => {
                existing.insert(key.clone(), value);
            }
            None => {
                existing.remove(key);
            }
        }
    }
    Ok(())
}

fn can_edit(revision: &Revision, user: &User) -> Result<(), Error> {
    if !revision.lifecycle.is_editable() {
        return Err(Error::conflict(
            "Start a new revision to edit these attributes",
        ));
    }
    if revision
        .lock
        .as_ref()
        .is_some_and(|lock| lock.user_id != user.id)
    {
        return Err(Error::conflict(
            "This revision is checked out by another user",
        ));
    }
    Ok(())
}

/// Overlay authoritative values onto a document. CAD saves cannot replace
/// an attribute accepted from the other interface with a stale cached value.
pub fn overlay(revision: &Revision, defs: &[AttributeDef], document: &mut Value) {
    if !defs.is_empty() {
        if !document["partAttributes"].is_object() {
            document["partAttributes"] = json!({});
        }
        for def in defs {
            let aliases: Vec<String> = document["partAttributes"]
                .as_object()
                .unwrap()
                .keys()
                .filter(|key| key.to_ascii_lowercase() == def.key && *key != &def.key)
                .cloned()
                .collect();
            for alias in aliases {
                if let Some(value) = revision.attributes.get(&def.key) {
                    document["partAttributes"][&alias] = value.clone();
                } else {
                    document["partAttributes"]
                        .as_object_mut()
                        .unwrap()
                        .remove(&alias);
                }
            }
            if let Some(value) = revision.attributes.get(&def.key) {
                document["partAttributes"][&def.key] = value.clone();
            } else {
                document["partAttributes"]
                    .as_object_mut()
                    .unwrap()
                    .remove(&def.key);
            }
        }
    }
    if let Some(features) = document["features"].as_array_mut() {
        for feature in features {
            let id = feature["inputParams"]["id"].as_str().unwrap_or_default();
            if let Some(occurrence) = revision.occurrences.iter().find(|o| o.id == id) {
                if !occurrence.attributes.is_empty() || feature["inputParams"].get("bom").is_some()
                {
                    feature["inputParams"]["bom"] = json!(occurrence.attributes);
                }
            }
        }
    }
}

impl Db {
    pub fn set_type_fields(&self, id: &str, defs: Vec<AttributeDef>) -> Result<(), Error> {
        let defs = catalog::check_definitions(defs)?;
        self.mutate(|state| {
            if !state.part_types.iter().any(|kind| kind.id == id) {
                return Err(Error::not_found("part type"));
            }
            let old = state
                .bom_configuration
                .part_fields
                .insert(id.into(), defs.clone())
                .unwrap_or_default();
            let ids: Vec<String> = state
                .parts
                .iter()
                .filter(|p| p.part_type == id)
                .map(|p| p.id.clone())
                .collect();
            for id in ids {
                let part = state.part(&id).unwrap().clone();
                for revision in &part.revisions {
                    let document = self
                        .read_document(&revision.document_key(&id))?
                        .and_then(|body| serde_json::from_str::<Value>(&body).ok())
                        .unwrap_or_default();
                    let stored = find_revision_mut(state, &id, &revision.id)?.1;
                    for def in &defs {
                        if old.iter().any(|old| old.key == def.key)
                            || stored.attributes.contains_key(&def.key)
                        {
                            continue;
                        }
                        let value = document["partAttributes"]
                            .as_object()
                            .and_then(|attrs| {
                                attrs
                                    .iter()
                                    .find(|(key, _)| key.to_ascii_lowercase() == def.key)
                                    .map(|(_, value)| value)
                            })
                            .or_else(|| part.attributes.get(&def.key));
                        if let Some(value) = value {
                            if let Ok(Some(value)) = catalog::check_value(def, value) {
                                stored.attributes.insert(def.key.clone(), value);
                            }
                        }
                    }
                    stored.attributes_initialized = true;
                }
                state.touch_part(&id);
            }
            Ok(())
        })
    }
    pub fn set_occurrence_fields(&self, defs: Vec<AttributeDef>) -> Result<(), Error> {
        let defs = catalog::check_definitions(defs)?;
        self.mutate(|state| {
            let old =
                std::mem::replace(&mut state.bom_configuration.occurrence_fields, defs.clone());
            let mut touched = Vec::new();
            for part in state.parts.iter_mut() {
                let mut changed = false;
                for revision in &mut part.revisions {
                    for occurrence in &mut revision.occurrences {
                        for def in &defs {
                            if old.iter().any(|old| old.key == def.key)
                                || occurrence.attributes.contains_key(&def.key)
                            {
                                continue;
                            }
                            let value = occurrence
                                .attributes
                                .iter()
                                .find(|(key, _)| key.to_ascii_lowercase() == def.key)
                                .map(|(_, value)| value.clone());
                            if let Some(value) = value {
                                if let Ok(Some(value)) = catalog::check_value(def, &value) {
                                    occurrence.attributes.insert(def.key.clone(), value);
                                    changed = true;
                                }
                            }
                        }
                    }
                }
                if changed {
                    touched.push(part.id.clone());
                }
            }
            for id in touched {
                state.touch_part(&id);
            }
            Ok(())
        })
    }
    pub fn save_layout(
        &self,
        user: &User,
        id: &str,
        name: &str,
        columns: Vec<String>,
        shared: bool,
    ) -> Result<Layout, Error> {
        if shared && !user.is_admin() {
            return Err(Error::forbidden(
                "Only administrators manage shared BOM configurations",
            ));
        }
        if name.trim().is_empty() {
            return Err(Error::bad_request("Give the configuration a name"));
        }
        self.mutate(|state| {
            let known: BTreeSet<String> = fields(state).into_iter().map(|f| f.id).collect();
            let mut seen = BTreeSet::new();
            if columns
                .iter()
                .any(|id| !known.contains(id) || !seen.insert(id.clone()))
            {
                return Err(Error::bad_request("Select available fields once each"));
            }
            let existing = state
                .bom_configuration
                .layouts
                .iter()
                .position(|l| l.id == id);
            if let Some(at) = existing {
                let l = &state.bom_configuration.layouts[at];
                if l.owner.as_deref().is_some_and(|owner| owner != user.id)
                    || (l.owner.is_none() && !user.is_admin())
                {
                    return Err(Error::forbidden(
                        "This configuration belongs to another user",
                    ));
                }
            }
            let layout = Layout {
                id: existing
                    .map(|_| id.to_string())
                    .unwrap_or_else(auth::new_id),
                name: name.trim().into(),
                owner: if shared { None } else { Some(user.id.clone()) },
                columns,
            };
            if let Some(at) = existing {
                state.bom_configuration.layouts[at] = layout.clone();
            } else {
                state.bom_configuration.layouts.push(layout.clone());
            }
            Ok(layout)
        })
    }
    pub fn patch_revision_attributes(
        &self,
        user: &User,
        part: &str,
        rev: &str,
        changes: &Value,
    ) -> Result<(), Error> {
        if !user.can_author() {
            return Err(Error::forbidden(
                "Editing attributes requires author permission",
            ));
        }
        self.mutate(|state| {
            let p = state
                .part_by_id_or_number(part)
                .ok_or_else(|| Error::not_found("part"))?;
            let id = p.id.clone();
            let defs = state
                .bom_configuration
                .part_fields
                .get(&p.part_type)
                .cloned()
                .unwrap_or_default();
            let revision =
                crate::bom::find_revision(p, rev).ok_or_else(|| Error::not_found("revision"))?;
            can_edit(revision, user)?;
            let rid = revision.id.clone();
            let revision = find_revision_mut(state, &id, &rid)?.1;
            checked_values(&defs, &mut revision.attributes, changes)?;
            revision.attributes_initialized = true;
            self.sync_attribute_document(state, &id, &rid, user)?;
            Ok(())
        })
    }
    pub fn patch_occurrences(
        &self,
        user: &User,
        part: &str,
        rev: &str,
        ids: &[String],
        changes: &Value,
    ) -> Result<(), Error> {
        if !user.can_author() {
            return Err(Error::forbidden(
                "Editing attributes requires author permission",
            ));
        }
        self.mutate(|state| {
            let p = state
                .part_by_id_or_number(part)
                .ok_or_else(|| Error::not_found("part"))?;
            let id = p.id.clone();
            let revision =
                crate::bom::find_revision(p, rev).ok_or_else(|| Error::not_found("revision"))?;
            can_edit(revision, user)?;
            let rid = revision.id.clone();
            let mut defs = state.bom_configuration.occurrence_fields.clone();
            for key in ["Find_Number", "Reference_Designator", "Notes"] {
                defs.push(AttributeDef {
                    key: key.into(),
                    name: key.into(),
                    kind: crate::model::AttributeKind::Text,
                    required: false,
                });
            }
            let revision = find_revision_mut(state, &id, &rid)?.1;
            for oid in ids {
                let occurrence = revision
                    .occurrences
                    .iter_mut()
                    .find(|o| o.id == *oid)
                    .ok_or_else(|| Error::not_found(format!("occurrence {oid}")))?;
                checked_values(&defs, &mut occurrence.attributes, changes)?;
                if let Some(changes) = changes.as_object() {
                    for key in changes.keys() {
                        let aliases: Vec<String> = occurrence
                            .attributes
                            .keys()
                            .filter(|alias| *alias != key && alias.eq_ignore_ascii_case(key))
                            .cloned()
                            .collect();
                        for alias in aliases {
                            if let Some(value) = occurrence.attributes.get(key).cloned() {
                                occurrence.attributes.insert(alias, value);
                            } else {
                                occurrence.attributes.remove(&alias);
                            }
                        }
                    }
                }
            }
            self.sync_attribute_document(state, &id, &rid, user)?;
            Ok(())
        })
    }
    /// Keep the model, revision metadata, hash and change feed together.
    fn sync_attribute_document(
        &self,
        state: &mut State,
        part: &str,
        rev: &str,
        user: &User,
    ) -> Result<(), Error> {
        let kind = state.part(part).unwrap().part_type.clone();
        let defs = state
            .bom_configuration
            .part_fields
            .get(&kind)
            .cloned()
            .unwrap_or_default();
        let key = crate::identity::document_key(part, rev);
        let path = self.document_path(&key)?;
        let body = match std::fs::read_to_string(&path) {
            Ok(body) => Some(body),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => None,
            Err(e) => return Err(Error::internal(e)),
        };
        let revision = find_revision_mut(state, part, rev)?.1;
        if let Some(body) = body {
            let mut document: Value = serde_json::from_str(&body).map_err(Error::internal)?;
            overlay(revision, &defs, &mut document);
            let body = serde_json::to_string(&document).map_err(Error::internal)?;
            crate::db::write_atomic(&path, &body).map_err(Error::internal)?;
            revision.content_hash = auth::content_hash(&body);
            revision.size = body.len() as u64;
            if let Some(thumbnail) = revision.thumbnail.as_mut() {
                thumbnail.content_hash = revision.content_hash.clone();
            }
        }
        if let Some(bake) = revision
            .bake
            .as_mut()
            .filter(|b| b.status != BakeStatus::Done)
        {
            bake.status = BakeStatus::Done;
            bake.error = "superseded by saved attributes".into();
        }
        synchronize_uses(revision);
        revision.modified_at = crate::db::now();
        crate::review::document_changed(state, part, rev);
        state.touch(key);
        let _ = user;
        Ok(())
    }
}

/// Import the occurrence identities on CAD publish. Attribute changes for an
/// existing identity arrive through the explicit edit API, never stale saves.
pub fn capture_occurrences(
    state: &State,
    part: &str,
    rev: &str,
    body: &Value,
) -> Result<Option<Vec<Occurrence>>, Error> {
    let Some(raw) = body.get("occurrences") else {
        return Ok(None);
    };
    let mut sent: Vec<Occurrence> = serde_json::from_value(raw.clone())
        .map_err(|_| Error::bad_request("Invalid occurrences"))?;
    let existing = state
        .part(part)
        .and_then(|p| p.revision(rev))
        .ok_or_else(|| Error::not_found("revision"))?;
    let mut seen = BTreeSet::new();
    for occurrence in &mut sent {
        if occurrence.id.is_empty() || !seen.insert(occurrence.id.clone()) {
            return Err(Error::bad_request("Occurrences need unique stable ids"));
        }
        let child = state
            .part_by_id_or_number(&occurrence.part)
            .ok_or_else(|| Error::not_found("occurrence part"))?;
        occurrence.part = child.id.clone();
        occurrence.revision = crate::bom::find_revision(child, &occurrence.revision)
            .ok_or_else(|| Error::not_found("occurrence revision"))?
            .id
            .clone();
        if let Some(old) = existing.occurrences.iter().find(|o| o.id == occurrence.id) {
            occurrence.attributes = old.attributes.clone();
        }
        // Unknown document fields are retained for standalone CAD compatibility.
        for def in &state.bom_configuration.occurrence_fields {
            if let Some(value) = occurrence.attributes.get(&def.key).cloned() {
                match catalog::check_value(def, &value).map_err(Error::bad_request)? {
                    Some(value) => {
                        occurrence.attributes.insert(def.key.clone(), value);
                    }
                    None => {
                        occurrence.attributes.remove(&def.key);
                    }
                }
            }
        }
    }
    Ok(Some(sent))
}

/// A configurable BOM exposes the actual placements. Packing includes all
/// occurrence values, so changing the visible columns cannot merge distinct data.
pub fn expand_bom(state: &State, bom: &mut crate::bom::Bom, flat: bool) {
    let Some(part) = state.part(&bom.part_id) else {
        return;
    };
    let Some(revision) = part.revision(&bom.revision_id) else {
        return;
    };
    let mut lines = crate::bom::occurrence_lines(state, part, revision);
    if flat {
        let mut packed: Vec<crate::bom::BomLine> = Vec::new();
        for mut row in lines {
            if let Some(existing) = packed.iter_mut().find(|e| {
                e.part_id == row.part_id
                    && e.revision_id == row.revision_id
                    && e.unit == row.unit
                    && e.owner_part == row.owner_part
                    && e.owner_revision == row.owner_revision
                    && e.occurrence_attributes == row.occurrence_attributes
            }) {
                existing.total += row.total;
                existing.quantity = existing.total;
                for id in row.occurrence_ids {
                    if !existing.occurrence_ids.contains(&id) {
                        existing.occurrence_ids.push(id);
                    }
                }
            } else {
                row.level = 0;
                row.position.clear();
                row.quantity = row.total;
                packed.push(row);
            }
        }
        lines = packed;
    }
    for row in &mut lines {
        crate::bom::reprice(state, row);
    }
    bom.flat = flat;
    bom.lines = lines;
    let mut totals: BTreeMap<String, (f64, usize)> = BTreeMap::new();
    bom.unpriced = 0;
    for line in &bom.lines {
        if line.assembly {
            continue;
        }
        if let Some(cost) = line.extended.filter(|_| line.costed) {
            let t = totals.entry(line.currency.clone()).or_default();
            t.0 += cost;
            t.1 += 1;
        } else {
            bom.unpriced += 1;
        }
    }
    bom.totals = totals
        .into_iter()
        .map(|(currency, (total, lines))| crate::bom::CurrencyTotal {
            currency,
            total,
            lines,
        })
        .collect();
    if !flat && bom.levels > 0 {
        bom.lines.retain(|line| line.level <= bom.levels);
    }
}

pub fn document_occurrences(document: &Value) -> Option<Vec<Occurrence>> {
    let features = document.get("features")?.as_array()?;
    let library = document.get("partsLibrary").and_then(Value::as_object);
    let occurrences = features
        .iter()
        .filter(|feature| {
            matches!(
                feature["type"].as_str(),
                Some("ACOMP") | Some("ASSEMBLY COMPONENT")
            )
        })
        .filter_map(|feature| {
            let params = &feature["inputParams"];
            let id = params["id"].as_str()?.to_string();
            let entry = library?.get(params["partName"].as_str()?)?;
            let source = entry["sourceKey"]
                .as_str()?
                .strip_prefix("/models/")
                .unwrap_or(entry["sourceKey"].as_str()?);
            let source = [".nbrep", ".abrep", ".fbrep", ".tbrep", ".json"]
                .iter()
                .find_map(|suffix| source.strip_suffix(suffix))
                .unwrap_or(source);
            let (part, revision) = crate::identity::split_key(source)?;
            let attributes = serde_json::from_value(
                params
                    .get("bom")
                    .filter(|v| v.is_object())
                    .cloned()
                    .unwrap_or_else(|| json!({})),
            )
            .ok()?;
            Some(Occurrence {
                id,
                part,
                revision,
                attributes,
            })
        })
        .collect();
    Some(occurrences)
}
impl Db {
    pub fn initialize_bom_occurrences(&self) -> Result<(), Error> {
        let candidates = self.read(|state| {
            state
                .parts
                .iter()
                .flat_map(|p| {
                    p.revisions
                        .iter()
                        .filter(|r| r.occurrences.is_empty() && !r.uses.is_empty())
                        .map(|r| (p.id.clone(), r.id.clone()))
                })
                .collect::<Vec<_>>()
        });
        let mut updates = Vec::new();
        for (part, rev) in candidates {
            if let Some(body) = self.read_document(&crate::identity::document_key(&part, &rev))? {
                if let Ok(document) = serde_json::from_str::<Value>(&body) {
                    if let Some(occurrences) =
                        document_occurrences(&document).filter(|o| !o.is_empty())
                    {
                        updates.push((part, rev, occurrences));
                    }
                }
            }
        }
        if updates.is_empty() {
            return Ok(());
        }
        self.mutate(|state| {
            for (part, rev, occurrences) in updates {
                find_revision_mut(state, &part, &rev)?.1.occurrences = occurrences;
                state.touch(crate::identity::document_key(&part, &rev));
            }
            Ok(())
        })
    }
}

pub fn synchronize_uses(revision: &mut Revision) {
    for line in &mut revision.uses {
        let occurrences: Vec<&Occurrence> = revision
            .occurrences
            .iter()
            .filter(|o| {
                o.part == line.part && (line.revision.is_empty() || o.revision == line.revision)
            })
            .collect();
        if occurrences.is_empty() {
            continue;
        }
        line.reference = occurrences
            .iter()
            .map(|o| {
                o.attributes
                    .get("Reference_Designator")
                    .and_then(Value::as_str)
                    .filter(|s| !s.is_empty())
                    .unwrap_or(&o.id)
            })
            .collect::<Vec<_>>()
            .join(", ");
        let common = |key: &str| {
            let first = occurrences[0]
                .attributes
                .get(key)
                .and_then(Value::as_str)
                .unwrap_or("");
            if occurrences
                .iter()
                .all(|o| o.attributes.get(key).and_then(Value::as_str).unwrap_or("") == first)
            {
                first.to_string()
            } else {
                String::new()
            }
        };
        line.find_number = common("Find_Number");
        line.notes = common("Notes");
    }
}
pub fn release_problems(
    state: &State,
    part: &crate::model::Part,
    revision: &Revision,
) -> Vec<String> {
    let mut problems = Vec::new();
    let mut check = |defs: &[AttributeDef], values: &BTreeMap<String, Value>, owner: &str| {
        for def in defs {
            match values.get(&def.key).map(|v| catalog::check_value(def, v)) {
                Some(Ok(Some(_))) => {}
                Some(Err(error)) => problems.push(format!("{owner}: {error}")),
                _ if def.required => problems.push(format!("{owner}: {} is required", def.name)),
                _ => {}
            }
        }
    };
    if let Some(defs) = state.bom_configuration.part_fields.get(&part.part_type) {
        check(defs, &revision.attributes, "Revision attributes");
    }
    for occurrence in &revision.occurrences {
        check(
            &state.bom_configuration.occurrence_fields,
            &occurrence.attributes,
            &occurrence.id,
        );
    }
    problems
}

pub fn layout_csv(bom: &crate::bom::Bom, layout: &Layout, fields: &[Field]) -> String {
    fn cell(value: &str) -> String {
        if value.contains([',', '"', '\n', '\r']) {
            format!("\"{}\"", value.replace('"', "\"\""))
        } else {
            value.into()
        }
    }
    let columns: Vec<&Field> = layout
        .columns
        .iter()
        .filter_map(|id| fields.iter().find(|f| f.id == *id))
        .collect();
    let mut csv = columns
        .iter()
        .map(|f| cell(&f.name))
        .collect::<Vec<_>>()
        .join(",");
    csv.push('\n');
    for line in &bom.lines {
        let builtins = serde_json::to_value(line).unwrap_or_default();
        let cells = columns
            .iter()
            .map(|field| {
                let value = if field.scope == "part" {
                    if field.part_type.as_deref() == Some(&line.part_type) {
                        line.part_values.get(&field.key)
                    } else {
                        None
                    }
                } else if field.scope == "occurrence" && !field.id.starts_with("builtin.") {
                    line.occurrence_attributes.get(&field.key)
                } else {
                    builtins.get(&field.key)
                };
                cell(&match value {
                    None | Some(Value::Null) => String::new(),
                    Some(Value::String(s)) => s.clone(),
                    Some(v) => v.to_string(),
                })
            })
            .collect::<Vec<_>>()
            .join(",");
        csv.push_str(&cells);
        csv.push('\n');
    }
    csv
}
