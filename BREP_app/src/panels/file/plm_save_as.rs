use super::*;
use crate::panels::plm_parts::{
    CreatedPart, NewPartRequest, PartCatalog, PartType, Pending, PlmPartCatalog,
};
use crate::plm::client::PlmClient;
use std::rc::Rc;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
enum Destination {
    #[default]
    NewPart,
    NewRevision,
    Existing,
}

impl Destination {
    fn slug(self) -> &'static str {
        match self {
            Destination::NewPart => "new-part",
            Destination::NewRevision => "new-revision",
            Destination::Existing => "existing",
        }
    }
}

/// Where a Save As starts: on a PLM revision, a new revision of it; off one,
/// a new part; and from the family member hand-edit prompt's "Save as a new
/// part", a NEW PART whatever the document is named — never the generated
/// member's own revision (integrator ruling 2026-10-03).
fn initial_destination(member_new_part: bool, on_a_revision: bool) -> Destination {
    if member_new_part || !on_a_revision {
        Destination::NewPart
    } else {
        Destination::NewRevision
    }
}

/// The document a Save As writes. A copy that becomes a part of its own (a
/// new part, or another part's revision) carries no family stamp — the
/// generated member's `familySource` names the family that wrote the member,
/// and the copy is not it (what "Save as a new part" means, as the local Save
/// As has always done). The session document itself is not touched here: it
/// drops the stamp only once the copy has landed.
fn copy_document(document: &str, strip_family_stamp: bool) -> String {
    if !strip_family_stamp {
        return document.to_string();
    }
    match serde_json::from_str::<serde_json::Value>(document) {
        Ok(mut value) => {
            if let Some(object) = value.as_object_mut() {
                object.remove(family_table::FAMILY_SOURCE_KEY);
            }
            value.to_string()
        }
        Err(_) => document.to_string(),
    }
}

#[derive(Default)]
pub(super) struct SaveAs {
    document: u64,
    destination: Destination,
    part: String,
    revision: String,
    name: String,
    number: String,
    part_type: String,
    types: Vec<PartType>,
    loading: Option<Pending<Vec<PartType>>>,
    saving: Option<Pending<String>>,
    signature: Option<String>,
    snapshot: Option<String>,
    /// The copy being written carries no family stamp (see
    /// [`copy_document`]); once it lands, the session document drops its own.
    strip_stamp: bool,
}

impl SaveAs {
    /// What the automation state publishes about the open Save As.
    pub(super) fn state_json(&self) -> serde_json::Value {
        serde_json::json!({
            "destination": self.destination.slug(),
            "part": self.part,
            "revision": self.revision,
            "name": self.name,
            "number": self.number,
            "saving": self.saving.is_some(),
            "stripsFamilyStamp": self.strip_stamp,
        })
    }
}

/// Save the captured session copy, retaining checkout for subsequent Save.
/// A refusal leaves the caller's document and identity untouched.
async fn write_copy(client: &PlmClient, key: &str, document: &str) -> Result<(), String> {
    let (part, revision) =
        crate::panels::plm_host::revision_of(key).ok_or("choose a PLM revision")?;
    let base = crate::plm::identity::revision_path(&part, &revision);
    client
        .call(
            "POST",
            &format!("{base}/checkout"),
            Some(br#"{"client_id":"brep-app"}"#.to_vec()),
        )
        .await
        .map_err(|e| e.to_string())?;
    client
        .call(
            "PUT",
            &format!("/api/store/doc/{key}"),
            Some(document.as_bytes().to_vec()),
        )
        .await
        .map_err(|e| format!("{key} is checked out, but saving failed: {e}"))?;
    Ok(())
}

async fn save_copy(
    client: &PlmClient,
    destination: Destination,
    part: &str,
    revision: &str,
    request: NewPartRequest,
    document: &str,
    other_open: &[String],
) -> Result<String, String> {
    match destination {
        Destination::NewRevision => {
            crate::panels::plm_parts::save_as_new_revision(client, part, revision, document).await
        }
        Destination::NewPart => {
            let response = client
                .call(
                    "POST",
                    "/api/parts",
                    Some(serde_json::to_vec(&request).map_err(|e| e.to_string())?),
                )
                .await
                .map_err(|e| e.to_string())?;
            let created: CreatedPart =
                serde_json::from_slice(&response.body).map_err(|e| e.to_string())?;
            let key = created
                .first_revision_key()
                .ok_or("the new part has no revision")?;
            write_copy(client, &key, document).await?;
            Ok(key)
        }
        Destination::Existing => {
            let path = format!("/api/parts/{}", crate::plm::identity::segment(part));
            let response = client
                .call("GET", &path, None)
                .await
                .map_err(|e| e.to_string())?;
            let detail: serde_json::Value =
                serde_json::from_slice(&response.body).map_err(|e| e.to_string())?;
            let id = detail["id"]
                .as_str()
                .ok_or("the target part was not found")?;
            let revisions = detail["revisions"]
                .as_array()
                .ok_or("the target has no revisions")?;
            let target = if revision.is_empty() {
                revisions.last()
            } else {
                revisions.iter().find(|r| {
                    r["id"].as_str() == Some(revision) || r["label"].as_str() == Some(revision)
                })
            }
            .ok_or("the target revision was not found")?;
            let key = crate::plm::identity::document_key(
                id,
                target["id"]
                    .as_str()
                    .ok_or("the target revision has no id")?,
            );
            if other_open.contains(&key) {
                return Err("That revision is already open in another tab; save from that tab or choose another destination".into());
            }
            write_copy(client, &key, document).await?;
            Ok(key)
        }
    }
}

impl FileDialog {
    pub(super) fn poll_plm_save_as(
        &mut self,
        ctx: &egui::Context,
        docs: &mut Documents,
        store: &dyn ModelStore,
    ) {
        let Some(state) = self.plm_save_as.as_mut() else {
            return;
        };
        if state.saving.is_some() {
            ctx.request_repaint_after(std::time::Duration::from_millis(30));
        }
        if let Some(result) = state
            .saving
            .as_mut()
            .and_then(|p| p.poll(std::task::Waker::noop()))
        {
            state.saving = None;
            match result {
                Ok(key) => {
                    let name = store.canonical_identity(&key);
                    if let Some(snapshot) = state.snapshot.take() {
                        store.remember_saved_document(&name, &snapshot);
                    }
                    if let Some(doc) = docs.iter_mut().find(|d| d.id() == state.document) {
                        doc.set_name(Some(name.clone()));
                        doc.set_plm_save_access(crate::document::Access::Editable);
                        if state.strip_stamp {
                            // The tab is now the copy: a part of its own, no
                            // family stamp. Its clean baseline is the copy as
                            // saved — captured WITHOUT the stamp below — so an
                            // edit made while the save was in flight reads
                            // dirty, and undoing it back to the saved content
                            // reads clean.
                            doc.engine
                                .history
                                .set_document_block_no_undo(family_table::FAMILY_SOURCE_KEY, None);
                        }
                        if let Some(signature) = state.signature.take() {
                            doc.mark_saved_signature(signature);
                        }
                    }
                    store.refresh_plm_index(&[key], false);
                    self.save_generation += 1;
                    self.status = format!("saved {name}");
                    if self.mode == Mode::SaveAs {
                        self.open = false;
                    }
                    self.plm_save_as = None;
                    return;
                }
                Err(problem) => {
                    self.status =
                        format!("Save failed: {problem}. Your session edits are unchanged.")
                }
            }
        }
    }

    pub(super) fn show_save_as_plm(
        &mut self,
        ctx: &egui::Context,
        docs: &mut Documents,
        store: &dyn ModelStore,
        client: Rc<PlmClient>,
    ) {
        let id = docs.active_id();
        if self
            .plm_save_as
            .as_ref()
            .is_none_or(|s| s.document != id && s.saving.is_none())
        {
            let source = docs
                .active()
                .name()
                .and_then(crate::panels::plm_host::revision_of);
            let catalog = PlmPartCatalog::new(client.clone());
            let member_new_part = std::mem::take(&mut self.member_new_part);
            self.plm_save_as = Some(SaveAs {
                document: id,
                destination: initial_destination(member_new_part, source.is_some()),
                part: source.map(|(p, _)| p).unwrap_or_default(),
                name: docs.active().title(),
                loading: Some(Pending::new(catalog.part_types())),
                ..Default::default()
            });
        }
        self.member_new_part = false;
        let state = self.plm_save_as.as_mut().unwrap();
        if let Some(result) = state
            .loading
            .as_mut()
            .and_then(|p| p.poll(std::task::Waker::noop()))
        {
            state.loading = None;
            match result {
                Ok(types) => {
                    state.part_type = types
                        .iter()
                        .find(|t| t.id == "component")
                        .or(types.first())
                        .map(|t| t.id.clone())
                        .unwrap_or_default();
                    state.types = types;
                }
                Err(problem) => self.status = problem,
            }
        }
        let busy = state.saving.is_some();
        if busy || state.loading.is_some() {
            ctx.request_repaint_after(std::time::Duration::from_millis(30));
        }
        let mut save = false;
        let modal = egui::Modal::new(egui::Id::new("brep-file-saveas")).show(ctx, |ui| {
            ui.heading("Save as");
            ui.add_enabled_ui(!busy, |ui| {
                for (value, label, key) in [
                    (
                        Destination::NewRevision,
                        "A new revision",
                        "saveas:new-revision",
                    ),
                    (Destination::NewPart, "A new part", "saveas:new-part"),
                    (
                        Destination::Existing,
                        "An existing revision",
                        "saveas:existing",
                    ),
                ] {
                    let choice = ui.radio_value(&mut state.destination, value, label);
                    self.hits.insert(key.into(), choice.rect);
                }
                if state.destination == Destination::NewPart {
                    ui.label("Part type");
                    egui::ComboBox::from_id_salt("saveas:type")
                        .selected_text(&state.part_type)
                        .show_ui(ui, |ui| {
                            for t in &state.types {
                                ui.selectable_value(&mut state.part_type, t.id.clone(), &t.name);
                            }
                        });
                    for (label, field, key) in [
                        ("Name", &mut state.name, "saveas:name"),
                        (
                            "Number (blank for automatic numbering)",
                            &mut state.number,
                            "saveas:number",
                        ),
                    ] {
                        ui.label(label);
                        let r = ui.text_edit_singleline(field);
                        self.hits.insert(key.into(), r.rect);
                    }
                } else {
                    ui.label("Part number");
                    let r = ui.text_edit_singleline(&mut state.part);
                    self.hits.insert("saveas:part".into(), r.rect);
                }
                ui.label(if state.destination == Destination::Existing {
                    "Revision (blank for newest)"
                } else {
                    "Revision label (blank for automatic)"
                });
                let r = ui.text_edit_singleline(&mut state.revision);
                self.hits.insert("saveas:revision".into(), r.rect);
                if state.destination == Destination::Existing {
                    ui.label(
                        "Check out this revision and replace its model with your session copy.",
                    );
                }
                let ready = if state.destination == Destination::NewPart {
                    !state.part_type.is_empty() && !state.name.trim().is_empty()
                } else {
                    !state.part.trim().is_empty()
                };
                let button = ui.add_enabled(ready, egui::Button::new("Save"));
                self.hits.insert("saveas:save".into(), button.rect);
                save = button.clicked();
            });
            if busy {
                ui.label("Saving…");
            }
            if !self.status.is_empty() {
                ui.label(&self.status);
            }
            let cancel = ui.button(if busy { "Keep editing" } else { "Cancel" });
            self.hits.insert("saveas:cancel".into(), cancel.rect);
            cancel.clicked()
        });
        if modal.inner || modal.should_close() {
            self.open = false;
            if !busy {
                self.plm_save_as = None;
            }
            return;
        }
        if save {
            // A copy that keeps the part's identity (its own next revision,
            // or its own existing one) keeps a member's family stamp; a copy
            // that becomes another part drops it.
            let source_part = docs
                .active()
                .name()
                .and_then(crate::panels::plm_host::revision_of)
                .map(|(part, _)| part)
                .unwrap_or_default();
            let keeps_identity = match state.destination {
                Destination::NewRevision => true,
                Destination::Existing => state.part.trim() == source_part,
                Destination::NewPart => false,
            };
            let stamped = docs
                .engine()
                .history
                .document_block(family_table::FAMILY_SOURCE_KEY)
                .is_some();
            state.strip_stamp = stamped && !keeps_identity;
            // The baseline of what is being saved: without the stamp when the
            // copy is written without it.
            state.signature = Some(if state.strip_stamp {
                docs.active().save_signature_without(family_table::FAMILY_SOURCE_KEY)
            } else {
                docs.active().save_signature()
            });
            let document = copy_document(&docs.engine().history_request_json(), state.strip_stamp);
            state.snapshot = Some(document.clone());
            let other_open = docs
                .iter()
                .filter(|d| d.id() != id)
                .filter_map(|d| {
                    d.name()
                        .and_then(crate::panels::plm_host::revision_of)
                        .map(|(p, r)| crate::plm::identity::document_key(&p, &r))
                })
                .collect::<Vec<_>>();
            let (destination, part, revision) = (
                state.destination,
                state.part.trim().to_string(),
                state.revision.trim().to_string(),
            );
            let request = NewPartRequest {
                part_type: state.part_type.clone(),
                name: state.name.trim().into(),
                number: state.number.trim().into(),
                label: revision.clone(),
                document_class: document_class::document_class(docs.engine()).slug().into(),
                ..Default::default()
            };
            state.saving = Some(Pending::new(Box::pin(async move {
                save_copy(
                    &client,
                    destination,
                    &part,
                    &revision,
                    request,
                    &document,
                    &other_open,
                )
                .await
            })));
            self.status = "Saving…".into();
            ctx.request_repaint();
        }
    }
}

