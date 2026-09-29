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
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

type Slot<T> = Rc<RefCell<Option<T>>>;

/// What the view shows for one fetch.
struct Fetched {
    /// The revision and view it is of.
    key: (String, String, bool),
    slot: Slot<Result<ServerBom, String>>,
    /// The document's BOM at the time of the fetch.
    document: Result<Vec<DocumentLine>, Vec<String>>,
}

/// An attribute edit waiting for the server.
struct Patch {
    part_name: String,
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
        if !force && self.fetched.as_ref().is_some_and(|f| f.key == key) {
            return;
        }
        let slot: Slot<Result<ServerBom, String>> = Rc::default();
        let (into, client, part_id, rev, flat) = (slot.clone(), client.clone(), part.to_string(), revision.to_string(), self.flat);
        spawn(async move {
            let got = bom::fetch_bom(&client, &part_id, &rev, flat).await.map_err(|e| e.to_string());
            *into.borrow_mut() = Some(got);
        });
        let document = serde_json::from_str::<Value>(&state.history_request_json())
            .map_err(|e| vec![e.to_string()])
            .and_then(|doc| bom::document_bom(&doc));
        self.fetched = Some(Fetched { key, slot, document });
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
        self.patches.push(Patch { part_name: part_name.to_string(), field: field.to_string(), value, slot });
    }

    /// Apply the edits the server accepted, in the order they were made, and
    /// report the refused ones in its words. A refused edit changes nothing.
    pub fn settle(&mut self, state: &mut EngineState) {
        while let Some(first) = self.patches.first() {
            let Some(result) = first.slot.borrow_mut().take() else { break };
            let patch = self.patches.remove(0);
            match result {
                Ok(()) => match state.set_part_attribute(&patch.part_name, &patch.field, patch.value.clone()) {
                    Ok(()) => {
                        self.last_edit = Some(Ok(format!("{} of {} saved on the PLM", patch.field, patch.part_name)));
                        // A new server value moves the BOM: fetch again.
                        self.fetched = None;
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
                    let differences = bom::compare(lines, &bom.lines);
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
        let grid = egui::Grid::new("bom-plm-lines").striped(true).show(ui, |ui| {
            for heading in ["Pos", "Number", "Name", "Rev", "State", "Qty", "Total", "Attributes", "MPN", "Manufacturer", "Supplier", "Unit price", "Extended"] {
                ui.strong(heading);
            }
            ui.end_row();
            for line in &bom.lines {
                let indent = "  ".repeat(line.level.saturating_sub(1));
                ui.label(if self.flat { String::new() } else { line.position.clone() });
                ui.label(format!("{indent}{}", line.number));
                ui.label(&line.name);
                ui.label(&line.revision_label);
                ui.label(if line.flags.is_empty() { line.state.clone() } else { format!("{} ({})", line.state, line.flags.join(", ")) });
                ui.label(format!("{} {}", number(line.quantity), line.unit));
                ui.label(number(line.total));
                ui.label(line.attributes.join("; "));
                ui.label(&line.mpn);
                ui.label(&line.manufacturer);
                ui.label(&line.supplier);
                ui.label(line.unit_price.map(|p| format!("{p} {}", line.currency)).unwrap_or_default());
                ui.label(line.extended.map(|p| format!("{p} {}", line.currency)).unwrap_or_default());
                ui.end_row();
            }
        });
        hits.insert("bom:plm:lines".into(), grid.response.rect);
        for total in &bom.totals {
            ui.label(format!("Total: {} {} over {} line(s)", total.total, total.currency, total.lines));
        }
        if bom.unpriced > 0 {
            ui.label(format!("{} line(s) have no price, and the totals miss them.", bom.unpriced));
        }
        for warning in &bom.warnings {
            ui.label(egui::RichText::new(warning).color(crate::panels::assembly_components::OUTDATED_AMBER));
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
            (Ok(doc), Some(bom)) if !fetched.key.2 => Some(bom::compare(doc, &bom.lines)),
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

/// A quantity without a trailing `.0`.
fn number(value: f64) -> String {
    if value.fract() == 0.0 && value.abs() < 1e15 {
        format!("{}", value as i64)
    } else {
        format!("{value}")
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
