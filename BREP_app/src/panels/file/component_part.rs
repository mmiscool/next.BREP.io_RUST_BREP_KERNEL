//! Recover a standalone component document without changing the active assembly
//! until the user confirms and storage accepts the write.
use super::*;
use crate::panels::component_actions::part_source_key;
use crate::store::{read_now, ReadNow};

pub(super) struct Recovery {
    assembly_id: u64,
    component_id: String,
    source: Option<String>,
    destination: Option<String>,
}

impl FileDialog {
    pub fn edit_component_part(
        &mut self,
        docs: &mut Documents,
        store: &dyn ModelStore,
        component_id: &str,
    ) {
        let source = part_source_key(docs.engine(), component_id);
        if let Some(key) = &source {
            match read_now(store, key) {
                ReadNow::Ready(_) => {
                    self.open_document(docs, store, key);
                    if self.status.starts_with("open failed:") {
                        docs.engine_mut().push_notice(self.status.clone());
                    }
                    return;
                }
                ReadNow::Loading => {
                    docs.engine_mut().push_notice(format!("This part's source document '{key}' is still loading — Edit Part again in a moment"));
                    return;
                }
                ReadNow::Absent => {}
            }
        }
        let Some(info) = docs.engine().component_info(component_id) else {
            docs.engine_mut()
                .push_notice(format!("'{component_id}' is not an assembly component"));
            return;
        };
        let destination = source.clone().filter(|key| {
            let file = crate::store::file_name_of(key);
            !key.starts_with('@')
                && key.trim() == key
                && !file.is_empty()
                && file != "."
                && file != ".."
        });
        self.name_buf = destination
            .as_deref()
            .map(model_display_name)
            .unwrap_or(info.part_name);
        self.status.clear();
        self.use_folder(Purpose::Models, store);
        self.component_part = Some(Recovery {
            assembly_id: docs.active_id(),
            component_id: component_id.into(),
            source,
            destination,
        });
        self.open_modal(Mode::CreateComponentPart);
    }

    fn cancel_component_part(&mut self) {
        self.component_part = None;
        self.open = false;
    }

    pub(super) fn show_create_component_part(
        &mut self,
        ctx: &egui::Context,
        docs: &mut Documents,
        store: &dyn ModelStore,
    ) {
        let mut create = false;
        let mut cancel = false;
        let source = self
            .component_part
            .as_ref()
            .and_then(|p| p.destination.clone());
        let modal = egui::Modal::new(egui::Id::new("brep-create-component-part")).show(ctx, |ui| {
            ui.heading("Create part file?");
            ui.label("The component has no available source file. Save its embedded part definition and open it in its own tab.");
            if let Some(key) = source {
                ui.label(format!("Source: {key}"));
            } else {
                ui.label(format!("Folder: {}", store.browser_location()));
                ui.label("File name");
                let field = ui.add(egui::TextEdit::singleline(&mut self.name_buf).hint_text("part name"));
                self.hit("field:name", &field);
                if self.want_focus {
                    field.request_focus();
                    self.want_focus = false;
                }
            }
            ui.horizontal(|ui| {
                let button = ui.button("Create part file");
                self.hit("createpart", &button);
                create = button.clicked();
                let button = ui.button("Cancel");
                self.hit("cancel", &button);
                cancel = button.clicked();
            });
            if !self.status.is_empty() { ui.colored_label(ui.visuals().error_fg_color, &self.status); }
        });
        if cancel || modal.should_close() {
            self.cancel_component_part();
        } else if create {
            if let Err(error) = self.create_component_part(docs, store) {
                self.status = format!("Create part failed: {error}");
            }
        }
    }

    fn create_component_part(
        &mut self,
        docs: &mut Documents,
        store: &dyn ModelStore,
    ) -> Result<(), String> {
        let pending = self
            .component_part
            .as_ref()
            .ok_or("no component selected")?;
        let index = docs
            .iter()
            .position(|doc| doc.id() == pending.assembly_id)
            .ok_or("the assembly tab was closed")?;
        let assembly = docs.get(index).unwrap();
        let info = assembly
            .engine
            .component_info(&pending.component_id)
            .ok_or("the component was removed")?;
        if part_source_key(&assembly.engine, &pending.component_id) != pending.source {
            return Err("the component source changed; cancel and Edit Part again".into());
        }
        // PLM revisions must be created by the PLM's New part/revision workflow.
        // Never reinterpret a remote reference as a successful local file save.
        if store.plm_client().is_some()
            || pending
                .source
                .as_deref()
                .is_some_and(|key| crate::plm::uses::revision_key_in(key).is_some())
        {
            return Err("this storage cannot create a missing PLM revision here; create or restore the part through PLM, then retry Edit Part".into());
        }
        let identity = match &pending.destination {
            Some(key) => {
                if key.starts_with('@') || key.trim().is_empty() {
                    return Err("invalid source file identity".into());
                }
                key.clone()
            }
            None => {
                let name = self.name_buf.trim();
                if name.is_empty()
                    || name.starts_with('@')
                    || name.contains(['/', '\\'])
                    || name == "."
                    || name == ".."
                {
                    return Err("enter a valid file name without folders".into());
                }
                let file = crate::store::model_file_name(name);
                let folder = store.browser_location();
                // Native and IndexedDB explorers expose absolute folders. Flat
                // stores keep their canonical bare document identities.
                if std::path::Path::new(&folder).is_absolute() {
                    sibling_identity(&format!("{}/_", folder.trim_end_matches('/')), &file)
                } else {
                    store.canonical_identity(&file)
                }
            }
        };
        if docs.iter().any(|doc| {
            doc.name().is_some_and(|name| {
                store.canonical_identity(name) == store.canonical_identity(&identity)
            })
        }) {
            return Err("that part already has an open tab; cancel and open that tab".into());
        }
        ensure_absent(store, &identity)?;
        let mut document: serde_json::Value =
            serde_json::from_str(&assembly.engine.history_request_json())
                .map_err(|e| e.to_string())?;
        let entry = &mut document["partsLibrary"][&info.part_name];
        let part = entry
            .get("document")
            .filter(|part| {
                part.is_object()
                    && part
                        .get("features")
                        .and_then(|v| v.as_array())
                        .is_some_and(|features| !features.is_empty())
            })
            .ok_or(
                "the component has no recoverable embedded part history; no blank file was created",
            )?;
        let contents = part.to_string();
        // Prepare the standalone document before writing, so malformed history
        // never leaves a saved file that cannot be opened.
        let mut engine = docs.spawn_engine();
        engine
            .load_model_and_fit(&contents)
            .map_err(|e| format!("embedded part cannot be opened: {e}"))?;
        let mut standalone = Document::new(engine);
        standalone.set_name(Some(identity.clone()));
        let signature = document_signature(&contents);
        let reconnect = entry["sourceKey"].as_str() != Some(&identity)
            || entry["sourceSignature"].as_str() != Some(&signature);
        if reconnect && !assembly.access().is_editable() {
            return Err("the assembly is read-only; its source link cannot be updated".into());
        }
        entry["sourceKey"] = serde_json::json!(identity);
        entry["sourceSignature"] = serde_json::json!(signature);
        // Part preparation can take time; check again immediately before saving.
        ensure_absent(store, &identity)?;
        store.write(&identity, &contents)?;
        if reconnect {
            docs.get_mut(index).unwrap().engine.edit_document_json(&document.to_string())
                .map_err(|e| format!("file saved at {identity}, but reconnect failed: {e}; cancel and open the saved part"))?;
        }
        docs.open_document(standalone);
        self.save_generation += 1;
        self.status = format!("created and opened {identity}");
        self.cancel_component_part();
        Ok(())
    }
}

fn ensure_absent(store: &dyn ModelStore, identity: &str) -> Result<(), String> {
    // A loading read is never permission to create or overwrite. This also
    // catches files that appeared while the offer was open.
    match read_now(store, identity) {
        ReadNow::Ready(_) => {
            return Err("the file now exists; cancel and open it (it was not overwritten)".into())
        }
        ReadNow::Loading => return Err("the file is still loading; retry in a moment".into()),
        ReadNow::Absent => {}
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        let path = std::path::PathBuf::from(store.canonical_identity(identity));
        if path.is_absolute()
            && path
                .try_exists()
                .map_err(|e| format!("cannot check destination: {e}"))?
        {
            return Err(
                "the destination exists but could not be read; it was not overwritten".into(),
            );
        }
    }
    Ok(())
}

