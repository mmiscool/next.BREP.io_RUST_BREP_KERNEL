//! The BOM panel's PLM half (plan S6): the server's BOM of the open revision,
//! and part-attribute edits routed through the server (D11).
//!
//! - **The PLM view** reads `GET …/revisions/:rev/bom`, indented or flat,
//!   and shows every line as the server sent it: number, revision, state,
//!   catalog attributes, quantity and total, preferred MPN, manufacturer and
//!   supplier, and the unit and extended prices with the currency totals. It
//!   recomputes nothing; a cost is the server's. Beside it, the document's
//!   own BOM in the same terms ([`crate::plm::bom::document_bom`]) and the
//!   line-for-line comparison, so "the PLM's BOM is the document's" is a
//!   thing the panel says, not a thing a user has to check.
//! - **An attribute edit** on a part that came from the PLM is a `PATCH` of
//!   that part (no checkout). NOTHING local changes until the server says
//!   yes: a refusal (a released part under `lock_released_attributes`, a key
//!   the category does not define) leaves the document exactly as it was and
//!   shows the server's sentence.
use crate::plm::bom::{self, DocumentLine, ServerBom};
use crate::plm::client::{PlmClient, PlmError};
use brep_render::engine_state::EngineState;
use eframe::egui;
use serde_json::{json, Value};
use std::cell::{Cell, RefCell};
use std::collections::HashMap;
use std::rc::Rc;

type Slot<T> = Rc<RefCell<Option<T>>>;

/// What the view shows for one fetch.
struct Fetched {
    /// The revision and view it is of.
    key: (String, String, bool),
    slot: Slot<Result<ServerBom, String>>,
    fetching: Rc<Cell<bool>>,
    refresh_error: Rc<RefCell<Option<String>>>,
    /// The document's BOM at the time of the fetch.
    document: Result<Vec<DocumentLine>, Vec<String>>,
}

/// An attribute edit waiting for the server.
struct Patch {
    part_name: String,
    occurrence_ids: Option<Vec<String>>,
    field: String,
    value: Value,
    slot: Slot<Result<(), PlmError>>,
}

#[derive(Default)]
pub struct PlmBomView {
    /// The PLM view is showing (else the document's own BOM).
    pub active: bool,
    pub flat: bool,
    fetched: Option<Fetched>,
    last_fetch: Option<web_time::Instant>,
    patches: Vec<Patch>,
    /// The last attribute edit's outcome, for the header.
    pub last_edit: Option<Result<String, String>>,
}

fn spawn(task: impl std::future::Future<Output = ()> + 'static) {
    #[cfg(target_arch = "wasm32")]
    wasm_bindgen_futures::spawn_local(task);
    #[cfg(not(target_arch = "wasm32"))]
    crate::plm::native::spawn(task);
}

impl PlmBomView {
    /// Start fetching the BOM of `part`/`revision` unless it is already
    /// fetched for the same view. The document's side is read now.
    fn ensure(&mut self, client: &Rc<PlmClient>, part: &str, revision: &str, state: &EngineState, force: bool) {
        let key = (part.to_string(), revision.to_string(), self.flat);
        if !self.fetched.as_ref().is_some_and(|f| f.key == key) {
            self.fetched = Some(Fetched {
                key, slot: Rc::default(), fetching: Rc::default(), refresh_error: Rc::default(),
                document: Ok(Vec::new()),
            });
            self.last_fetch = None;
        }
        let fetched = self.fetched.as_mut().unwrap();
        if fetched.fetching.get() || (!force && self.last_fetch.is_some_and(|last| last.elapsed() < web_time::Duration::from_secs(3))) { return; }
        fetched.document = serde_json::from_str::<Value>(&state.history_request_json())
            .map_err(|e| vec![e.to_string()])
            .and_then(|doc| bom::document_bom(&doc));
        fetched.fetching.set(true);
        self.last_fetch = Some(web_time::Instant::now());
        let (into, fetching, error) = (fetched.slot.clone(), fetched.fetching.clone(), fetched.refresh_error.clone());
        let (client, part, revision, flat) = (client.clone(), part.to_string(), revision.to_string(), self.flat);
        spawn(async move {
            let answer = bom::fetch_occurrence_bom(&client, &part, &revision, flat).await.map_err(|e| e.to_string());
            let mut shown = into.borrow_mut();
            match answer {
                Ok(bom) => { *shown = Some(Ok(bom)); *error.borrow_mut() = None; }
                Err(message) if shown.as_ref().is_some_and(Result::is_ok) => { *error.borrow_mut() = Some(message); }
                Err(message) => { *shown = Some(Err(message)); }
            }
            fetching.set(false);
        });
    }

    /// Send one part-attribute edit to the server. The engine is not touched
    /// until [`Self::settle`] sees the server's yes.
    pub fn queue_patch(&mut self, client: &Rc<PlmClient>, part_id: &str, part_name: &str, field: &str, value: Value) {
        let slot: Slot<Result<(), PlmError>> = Rc::default();
        let (into, client, part_id, key, sent) = (slot.clone(), client.clone(), part_id.to_string(), field.to_string(), value.clone());
        spawn(async move {
            let got = bom::patch_attribute(&client, &part_id, &key, &sent).await;
            *into.borrow_mut() = Some(got);
        });
        self.patches.push(Patch { part_name: part_name.to_string(), occurrence_ids: None, field: field.to_string(), value, slot });
    }

    pub fn queue_part_record_patch(&mut self,client:&Rc<PlmClient>,part:&str,part_name:&str,key:&str,cad_field:&str,value:Value){
        let slot:Slot<Result<(),PlmError>>=Rc::default();
        let (into,client,path,sent)=(slot.clone(),client.clone(),crate::plm::identity::part_path(part),json!({key:value.clone()}));
        spawn(async move{*into.borrow_mut()=Some(client.call("PATCH",&path,Some(sent.to_string().into_bytes())).await.map(|_|()));});
        self.patches.push(Patch{part_name:part_name.into(),occurrence_ids:None,field:cad_field.into(),value,slot});
    }
    pub fn queue_revision_patch(&mut self,client:&Rc<PlmClient>,part:&str,revision:&str,part_name:&str,field:&str,value:Value){
        let slot:Slot<Result<(),PlmError>>=Rc::default();
        let (into,client,path,sent)=(slot.clone(),client.clone(),format!("{}/attributes",crate::plm::identity::revision_path(part,revision)),json!({"attributes":{field:value.clone()}}));
        spawn(async move{*into.borrow_mut()=Some(client.call("PATCH",&path,Some(sent.to_string().into_bytes())).await.map(|_|()));});
        self.patches.push(Patch{part_name:part_name.into(),occurrence_ids:None,field:field.into(),value,slot});
    }
    pub fn queue_occurrence_patch(&mut self,client:&Rc<PlmClient>,part:&str,revision:&str,ids:&[String],field:&str,value:Value,local:bool){
        let slot:Slot<Result<(),PlmError>>=Rc::default();
        let (into,client,path,sent)=(slot.clone(),client.clone(),format!("{}/occurrences",crate::plm::identity::revision_path(part,revision)),json!({"ids":ids,"attributes":{field:value.clone()}}));
        spawn(async move{*into.borrow_mut()=Some(client.call("PATCH",&path,Some(sent.to_string().into_bytes())).await.map(|_|()));});
        self.patches.push(Patch{part_name:part.into(),occurrence_ids:Some(if local{ids.to_vec()}else{Vec::new()}),field:field.into(),value,slot});
    }

    /// Apply the edits the server accepted, in the order they were made, and
    /// report the refused ones in its words. A refused edit changes nothing.
    pub fn settle(&mut self, state: &mut EngineState) {
        while let Some(first) = self.patches.first() {
            let Some(result) = first.slot.borrow_mut().take() else { break };
            let patch = self.patches.remove(0);
            match result {
                Ok(()) => match if let Some(ids)=&patch.occurrence_ids {if ids.is_empty(){Ok(())}else{state.set_occurrence_attribute(ids,&patch.field,patch.value.clone())}} else if patch.part_name.is_empty(){Ok(())}else{state.set_part_attribute(&patch.part_name, &patch.field, patch.value.clone())} {
                    Ok(()) => {
                        self.last_edit = Some(Ok(format!("{} of {} saved on the PLM", patch.field, patch.part_name)));
                        // A new server value moves the BOM: fetch again.
                        self.last_fetch = None;
                    }
                    Err(error) => self.last_edit = Some(Err(format!("saved on the PLM, but not here: {error}"))),
                },
                Err(refused) => {
                    let sentence = format!("{} of {}: {refused}", patch.field, patch.part_name);
                    state.push_notice(format!("BOM: {sentence}"));
                    self.last_edit = Some(Err(sentence));
                }
            }
        }
    }

    /// Edits still waiting for the server.
    pub fn pending(&self) -> usize {
        self.patches.len()
    }

    /// Draw the PLM view of `part`/`revision`.
    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        state: &EngineState,
        client: &Rc<PlmClient>,
        part: &str,
        revision: &str,
        configuration: &crate::panels::bom_configuration::BomConfiguration,
        hits: &mut HashMap<String, egui::Rect>,
    ) {
        let mut refresh = false;
        ui.horizontal(|ui| {
            let indented = ui.selectable_label(!self.flat, "Indented").on_hover_text("Every level, as the tree it is");
            hits.insert("bom:plm:indented".into(), indented.rect);
            if indented.clicked() {
                self.flat = false;
            }
            let flat = ui.selectable_label(self.flat, "Flat").on_hover_text("One line per part, totals summed over every path");
            hits.insert("bom:plm:flat".into(), flat.rect);
            if flat.clicked() {
                self.flat = true;
            }
            let again = ui.button("Refresh").on_hover_text("Ask the PLM again");
            hits.insert("bom:plm:refresh".into(), again.rect);
            refresh = again.clicked();
        });
        self.ensure(client, part, revision, state, refresh);
        if let Some(outcome) = &self.last_edit {
            match outcome {
                Ok(text) => ui.label(text),
                Err(text) => ui.label(egui::RichText::new(text).color(crate::panels::assembly_components::OUTDATED_AMBER)),
            };
        }
        let Some(fetched) = &self.fetched else { return };
        if fetched.fetching.get() { ui.ctx().request_repaint(); }
        if let Some(error) = fetched.refresh_error.borrow().as_ref() {
            ui.colored_label(crate::panels::assembly_components::OUTDATED_AMBER, error);
        }
        let answer = fetched.slot.borrow();
        let bom = match answer.as_ref() {
            None => {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.label("Asking the PLM for this revision's BOM…");
                });
                ui.ctx().request_repaint();
                return;
            }
            Some(Err(error)) => {
                ui.label(egui::RichText::new(error).color(crate::panels::assembly_components::OUTDATED_AMBER));
                return;
            }
            Some(Ok(bom)) => bom,
        };
        ui.label(format!("{} {} rev {} ({})", bom.number, bom.name, bom.revision_label, bom.state));
        if !self.flat {
            match &fetched.document {
                Ok(lines) => {
                    let differences = bom::compare(lines, &bom.comparison_lines);
                    if differences.is_empty() {
                        ui.label(format!("The PLM's BOM is this document's, line for line ({} lines).", lines.len()));
                    } else {
                        ui.label(egui::RichText::new(format!("{} difference(s) from this document:", differences.len())).color(crate::panels::assembly_components::OUTDATED_AMBER));
                        for difference in &differences {
                            ui.label(difference);
                        }
                    }
                }
                Err(refused) => {
                    ui.label(egui::RichText::new("This document places parts that are no PLM revisions:").color(crate::panels::assembly_components::OUTDATED_AMBER));
                    for why in refused {
                        ui.label(why);
                    }
                }
            }
        }
        let fields:Vec<crate::panels::bom_configuration::Field>=configuration.config.as_ref().map(|config|configuration.columns.iter().filter_map(|id|config.fields.iter().find(|f|f.id==*id).cloned()).collect()).unwrap_or_default();
        let mut edits=Vec::new();
        let grid=egui::ScrollArea::horizontal().show(ui,|ui|egui::Grid::new("bom-plm-lines").striped(true).show(ui,|ui|{
            ui.strong("Pos");for field in &fields{ui.strong(&field.name);}ui.end_row();
            for line in &bom.lines {
                ui.label(&line.position);
                let values=serde_json::to_value(line).unwrap_or_default();
                for field in &fields {
                    let value=if field.scope=="part" {if field.part_type.as_deref()==Some(&line.part_type){line.part_values.get(&field.key)}else{None}}
                        else if field.scope=="occurrence" {if field.id.starts_with("builtin."){values.get(&field.key)}else{line.occurrence_attributes.get(&field.key)}}else{values.get(&field.key)};
                    let editable=field.editable&&(field.scope!="part"||field.part_type.as_deref()==Some(&line.part_type))&&(field.scope!="occurrence"||!line.occurrence_ids.is_empty());
                    let kind=if editable{field.cell_kind()}else{crate::column_tree::CellKind::ReadOnly};
                    let (edit,rect)=crate::column_tree::value_editor(ui,egui::Id::new(("plm-bom-cell",part,revision,&line.position,&line.part_id,&line.owner_part,&line.occurrence_ids,&field.id)),&kind,value,egui::vec2(145.0,ui.spacing().interact_size.y));
                    hits.insert(format!("bom:plm:cell:{}:{}",line.position,field.id),rect);
                    if let Some(value)=edit{edits.push((line.clone(),field.clone(),value));}
                }
                ui.end_row();
            }
        }));
        hits.insert("bom:plm:lines".into(),grid.inner.response.rect);
        for total in &bom.totals {
            ui.label(format!("Total: {} {} over {} line(s)", total.total, total.currency, total.lines));
        }
        if bom.unpriced > 0 {
            ui.label(format!("{} line(s) have no price, and the totals miss them.", bom.unpriced));
        }
        for warning in &bom.warnings {
            ui.label(egui::RichText::new(warning).color(crate::panels::assembly_components::OUTDATED_AMBER));
        }
        drop(answer);
        for (line,field,value) in edits {
            if let Some(target)=field.edit.as_ref().filter(|target|target.resource=="part") {
                self.queue_part_record_patch(client,&line.part_id,"",&target.key,&field.cad_field,value);
            } else if field.scope=="part" {
                let document:Value=serde_json::from_str(&state.history_request_json()).unwrap_or_default();
                let part_name=document["partsLibrary"].as_object().and_then(|l|l.iter().find(|(_,entry)|crate::plm::bom::revision_of_document(entry["sourceKey"].as_str().unwrap_or("")).as_ref()==Some(&(line.part_id.clone(),line.revision_id.clone()))).map(|(name,_)|name.clone())).unwrap_or_default();
                self.queue_revision_patch(client,&line.part_id,&line.revision_id,&part_name,&field.key,value);
            }else{self.queue_occurrence_patch(client,&line.owner_part,&line.owner_revision,&line.occurrence_ids,&field.cad_field,value,line.owner_part==part&&line.owner_revision==revision);}
        }

    }

    /// The `__brepPlmBom` state: what the view shows, and the comparison.
    pub fn state_json(&self, available: bool) -> Value {
        let Some(fetched) = &self.fetched else {
            return json!({ "available": available, "active": self.active, "flat": self.flat, "loading": false, "pendingEdits": self.patches.len(), "lastEdit": edit_json(&self.last_edit) });
        };
        let answer = fetched.slot.borrow();
        let (loading, error, server) = match answer.as_ref() {
            None => (true, None, None),
            Some(Err(e)) => (false, Some(e.clone()), None),
            Some(Ok(bom)) => (false, None, Some(bom)),
        };
        let lines: Vec<Value> = server
            .map(|bom| {
                bom.lines
                    .iter()
                    .map(|l| {
                        json!({
                            "level": l.level, "position": l.position, "partId": l.part_id, "number": l.number,
                            "revisionId": l.revision_id, "revision": l.revision_label, "state": l.state,
                            "quantity": l.quantity, "total": l.total, "attributes": l.attributes,
                            "mpn": l.mpn, "manufacturer": l.manufacturer, "supplier": l.supplier,
                            "currency": l.currency, "unitPrice": l.unit_price, "extended": l.extended,
                        })
                    })
                    .collect()
            })
            .unwrap_or_default();
        let (document, refused) = match &fetched.document {
            Ok(lines) => (
                lines
                    .iter()
                    .map(|l| json!({ "level": l.level, "position": l.position, "partId": l.part_id, "revisionId": l.revision_id, "quantity": l.quantity, "total": l.total }))
                    .collect::<Vec<_>>(),
                Vec::new(),
            ),
            Err(refused) => (Vec::new(), refused.clone()),
        };
        let differences = match (&fetched.document, server) {
            (Ok(doc), Some(bom)) if !fetched.key.2 => Some(bom::compare(doc, &bom.comparison_lines)),
            _ => None,
        };
        json!({
            "available": available, "active": self.active, "flat": fetched.key.2, "loading": loading, "error": error,
            "number": server.map(|b| b.number.clone()), "revision": server.map(|b| b.revision_label.clone()),
            "lines": lines,
            "totals": server.map(|b| b.totals.iter().map(|t| json!({ "currency": t.currency, "total": t.total, "lines": t.lines })).collect::<Vec<_>>()).unwrap_or_default(),
            "unpriced": server.map(|b| b.unpriced), "warnings": server.map(|b| b.warnings.clone()).unwrap_or_default(),
            "document": document, "documentRefused": refused, "differences": differences,
            "pendingEdits": self.patches.len(), "lastEdit": edit_json(&self.last_edit),
        })
    }
}

fn edit_json(edit: &Option<Result<String, String>>) -> Value {
    match edit {
        None => Value::Null,
        Some(Ok(text)) => json!({ "ok": true, "text": text }),
        Some(Err(text)) => json!({ "ok": false, "text": text }),
    }
}

/// The PLM pane's where-used section (S6): every assembly that uses the open
/// revision's part, every level up, before anyone changes it. It fetches when
/// the part changes, and again on Refresh. Hung in S3's `PlmHost` as the
/// section `where-used`; `put` is that section's hit-key sink.
#[derive(Default)]
pub struct WhereUsedSection {
    part: Option<String>,
    slot: Option<Slot<Result<bom::WhereUsed, String>>>,
}

impl WhereUsedSection {
    /// A request is still unanswered (the idle contract waits for it).
    pub fn busy(&self) -> bool {
        self.slot.as_ref().is_some_and(|s| s.borrow().is_none())
    }

    fn fetch(&mut self, client: &Rc<PlmClient>, part: &str) {
        let slot: Slot<Result<bom::WhereUsed, String>> = Rc::default();
        let (into, client, id) = (slot.clone(), client.clone(), part.to_string());
        spawn(async move {
            let got = bom::where_used(&client, &id, None).await.map_err(|e| e.to_string());
            *into.borrow_mut() = Some(got);
        });
        self.part = Some(part.to_string());
        self.slot = Some(slot);
    }

    /// Draw the section for `part`, fetching when it is new.
    pub fn draw(&mut self, ui: &mut egui::Ui, client: &Rc<PlmClient>, part: &str, put: &mut dyn FnMut(&str, egui::Rect)) {
        if self.part.as_deref() != Some(part) {
            self.fetch(client, part);
        }
        let refresh = ui.button("Refresh").on_hover_text("Ask the PLM again");
        put("refresh", refresh.rect);
        if refresh.clicked() {
            self.fetch(client, part);
        }
        let Some(slot) = &self.slot else { return };
        match slot.borrow().as_ref() {
            None => {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.label("Asking the PLM where this part is used…");
                });
                ui.ctx().request_repaint();
            }
            Some(Err(error)) => {
                ui.label(egui::RichText::new(error).color(crate::panels::assembly_components::OUTDATED_AMBER));
            }
            Some(Ok(used)) => {
                let shown = ui.scope(|ui| bom::show_where_used(ui, used)).response;
                put("lines", shown.rect);
            }
        }
    }

    /// For the pane's state: `{part, loading, error, lines:[{level, number, revision, state, current, uses, quantity, top}]}`.
    pub fn state_json(&self) -> Value {
        let Some(slot) = &self.slot else { return Value::Null };
        let answer = slot.borrow();
        match answer.as_ref() {
            None => json!({ "part": self.part, "loading": true }),
            Some(Err(e)) => json!({ "part": self.part, "loading": false, "error": e }),
            Some(Ok(used)) => json!({
                "part": self.part, "loading": false, "number": used.number,
                "lines": used.lines.iter().map(|l| json!({
                    "level": l.level, "partId": l.part_id, "number": l.number, "revision": l.revision_label,
                    "state": l.state, "current": l.current, "uses": l.uses_revision, "quantity": l.quantity, "top": l.top,
                })).collect::<Vec<_>>(),
            }),
        }
    }
}
